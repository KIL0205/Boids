"""sim/sardine_gpu.py — WebGPU (wgpu-py) compute backend for sardine flocking.

Replaces the Python cell-bucket loop in Simulation._batch_compute_sardine_forces
with a WGSL compute shader executed via wgpu-py (Metal on macOS via wgpu-native).

Architecture
------------
CPU builds a GPU-friendly sorted spatial hash each frame:
  1. Assign each sardine to a grid cell (col * nr + row).
  2. argsort sardine indices by cell ID → sorted_idxs (n,).
  3. Rearrange positions and velocities into sorted order.
  4. Compute cell_starts[nc*nr] and cell_counts[nc*nr] via bincount + cumsum.

GPU receives (all float32 / uint32):
  - sorted_pos    (n, 2)   — sardine positions in sorted order
  - sorted_vel    (n, 2)   — sardine velocities in sorted order
  - cell_starts   (nc*nr,) — start index into sorted array per cell
  - cell_counts   (nc*nr,) — count of sardines per cell
  - sailfish_pos  (S, 2)   — predator positions (up to _MAX_SAILFISH)
  - params        96 bytes — all sim-config scalars (Params uniform struct)

GPU outputs (in sorted order):
  - out_acc (n, 2) — flocking accelerations (flee + sep + ali + coh)
  Noise is NOT computed on GPU; the CPU adds it after scattering the result.

CPU scatter:
  sa[sorted_idxs] = out_acc.astype(float64)

Not included in GPU path
------------------------
- The small-population fast path (n < 500) still uses per-sardine Python loops.
- Pass 3 (noise) is kept on CPU: avoids WGSL PRNG complexity.
- Ecology, births/deaths, baitball detection, sailfish FSM.
"""

from __future__ import annotations

import logging
import math
import struct
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from config.config import SimConfig

log = logging.getLogger(__name__)

# ── wgpu availability ─────────────────────────────────────────────────────────
try:
    import wgpu  # type: ignore[import-untyped]
    _WGPU_AVAILABLE = True
except ImportError:
    _WGPU_AVAILABLE = False

# Maximum sailfish count whose positions are uploaded each frame.
# If actual count exceeds this, the GPU flee force is computed for only the
# first _MAX_SAILFISH predators (remainder ignored for flee — rare edge case).
_MAX_SAILFISH: int = 32

# Workgroup size — must match @workgroup_size in WGSL.
_WORKGROUP_SIZE: int = 64

# Params uniform buffer size (96 bytes = 24 × 4, multiple of 16 ✓).
_PARAMS_SIZE: int = 96

# ── WGSL compute shader ───────────────────────────────────────────────────────
_WGSL: str = r"""
// Sardine flocking forces — WebGPU compute shader.
//
// Each invocation handles one sardine (by sorted spatial-hash index).
// Computes flee (sailfish + mouse), separation, alignment, and cohesion.
// Noise is intentionally omitted — the CPU adds it after reading back results.

struct Params {
    // --- offset  0: 4 x u32 (16 bytes) ---
    n:               u32,   // active sardine count
    nc:              u32,   // grid columns
    nr:              u32,   // grid rows
    coh_span:        u32,   // cell radius of neighbourhood (usually 1)
    // --- offset 16: 4 x f32 (16 bytes) ---
    W:               f32,   // world width  (pixels)
    H:               f32,   // world height (pixels)
    cell_size:       f32,   // spatial-hash cell side (pixels)
    sep_r2:          f32,   // separation radius²
    // --- offset 32: 4 x f32 (16 bytes) ---
    ali_r2:          f32,   // alignment radius²
    coh_r2:          f32,   // cohesion radius²
    flee_r2:         f32,   // flee-from-sailfish radius²
    max_speed:       f32,   // sardine max speed
    // --- offset 48: 4 x f32 (16 bytes) ---
    max_force:       f32,   // sardine max steering force
    w_sep:           f32,   // weight_separation
    w_ali:           f32,   // weight_alignment
    w_coh:           f32,   // weight_cohesion
    // --- offset 64: 1 x f32 + 1 x u32 + 2 x f32 (16 bytes) ---
    w_flee:          f32,   // weight_flee
    n_sailfish:      u32,   // active sailfish count (capped to _MAX_SAILFISH)
    mouse_x:         f32,   // mouse predator x
    mouse_y:         f32,   // mouse predator y
    // --- offset 80: 2 x f32 + 2 x u32 (16 bytes) ---
    mouse_r2:        f32,   // mouse predator radius²
    mouse_flee_w:    f32,   // mouse flee weight multiplier
    mouse_active:    u32,   // 1 = mouse predator active, 0 = inactive
    _pad:            u32,   // padding to reach 96-byte multiple-of-16 size
};

@group(0) @binding(0) var<storage, read>       pos_buf:     array<f32>; // (n*2): x,y interleaved
@group(0) @binding(1) var<storage, read>       vel_buf:     array<f32>; // (n*2): vx,vy interleaved
@group(0) @binding(2) var<storage, read>       cell_starts: array<u32>; // (nc*nr)
@group(0) @binding(3) var<storage, read>       cell_counts: array<u32>; // (nc*nr)
@group(0) @binding(4) var<storage, read>       sf_pos_buf:  array<f32>; // (n_sailfish*2)
@group(0) @binding(5) var<uniform>             params:      Params;
@group(0) @binding(6) var<storage, read_write> out_acc:     array<f32>; // (n*2): ax,ay interleaved

const EPS:  f32 = 1e-9;
const EPS2: f32 = 1e-18;

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.n { return; }

    let my_x  = pos_buf[i * 2u];
    let my_y  = pos_buf[i * 2u + 1u];
    let my_vx = vel_buf[i * 2u];
    let my_vy = vel_buf[i * 2u + 1u];

    let W  = params.W;
    let H  = params.H;
    let hw = W * 0.5;
    let hh = H * 0.5;

    var acc_x: f32 = 0.0;
    var acc_y: f32 = 0.0;
    var is_fleeing: bool = false;

    // ── Pass 1: Flee from sailfish ─────────────────────────────────────────
    if params.n_sailfish > 0u && params.w_flee > 0.0 && params.flee_r2 > 0.0 {
        var flee_x: f32 = 0.0;
        var flee_y: f32 = 0.0;
        var any_flee: bool = false;
        for (var s = 0u; s < params.n_sailfish; s++) {
            let sf_x = sf_pos_buf[s * 2u];
            let sf_y = sf_pos_buf[s * 2u + 1u];
            var dx = my_x - sf_x;
            var dy = my_y - sf_y;
            // Toroidal correction
            if dx >  hw { dx -= W; }
            if dx < -hw { dx += W; }
            if dy >  hh { dy -= H; }
            if dy < -hh { dy += H; }
            let d2 = dx * dx + dy * dy;
            if d2 > EPS2 && d2 <= params.flee_r2 {
                let inv_d = 1.0 / sqrt(max(d2, EPS2));
                flee_x += dx * inv_d;
                flee_y += dy * inv_d;
                any_flee = true;
            }
        }
        if any_flee {
            let fm = sqrt(flee_x * flee_x + flee_y * flee_y);
            if fm > EPS {
                is_fleeing = true;
                let des_x = flee_x / fm * params.max_speed;
                let des_y = flee_y / fm * params.max_speed;
                var stx = des_x - my_vx;
                var sty = des_y - my_vy;
                let stm = sqrt(stx * stx + sty * sty);
                if stm > params.max_force {
                    stx = stx / stm * params.max_force;
                    sty = sty / stm * params.max_force;
                }
                acc_x += stx * params.w_flee;
                acc_y += sty * params.w_flee;
            }
        }
    }

    // ── Pass 1b: Mouse predator flee ───────────────────────────────────────
    if params.mouse_active != 0u && params.mouse_r2 > 0.0 {
        var dx = my_x - params.mouse_x;
        var dy = my_y - params.mouse_y;
        if dx >  hw { dx -= W; }
        if dx < -hw { dx += W; }
        if dy >  hh { dy -= H; }
        if dy < -hh { dy += H; }
        let d2 = dx * dx + dy * dy;
        if d2 > EPS2 && d2 < params.mouse_r2 {
            is_fleeing = true;
            let inv_d = 1.0 / sqrt(max(d2, EPS2));
            let fdx = dx * inv_d;
            let fdy = dy * inv_d;
            let fm  = sqrt(fdx * fdx + fdy * fdy);
            if fm > EPS {
                let des_x = fdx / fm * params.max_speed;
                let des_y = fdy / fm * params.max_speed;
                var stx = des_x - my_vx;
                var sty = des_y - my_vy;
                let stm = sqrt(stx * stx + sty * sty);
                if stm > params.max_force {
                    stx = stx / stm * params.max_force;
                    sty = sty / stm * params.max_force;
                }
                acc_x += stx * params.w_flee * params.mouse_flee_w;
                acc_y += sty * params.w_flee * params.mouse_flee_w;
            }
        }
    }

    // ── Pass 2: Flocking — sep + ali + coh ────────────────────────────────
    // Determine this sardine's grid cell from its position.
    let cell_col = u32(my_x / params.cell_size) % params.nc;
    let cell_row = u32(my_y / params.cell_size) % params.nr;

    var sep_x: f32 = 0.0;  var sep_y: f32 = 0.0;  var sep_cnt: u32 = 0u;
    var ali_x: f32 = 0.0;  var ali_y: f32 = 0.0;  var ali_cnt: u32 = 0u;
    var coh_x: f32 = 0.0;  var coh_y: f32 = 0.0;  var coh_cnt: u32 = 0u;

    let span = i32(params.coh_span);
    for (var dc = -span; dc <= span; dc++) {
        let col = u32((i32(cell_col) + dc + i32(params.nc)) % i32(params.nc));
        for (var dr = -span; dr <= span; dr++) {
            let row = u32((i32(cell_row) + dr + i32(params.nr)) % i32(params.nr));
            let cid   = col * params.nr + row;
            let start = cell_starts[cid];
            let cnt   = cell_counts[cid];
            for (var k = start; k < start + cnt; k++) {
                if k == i { continue; }   // skip self-pair
                let n_x  = pos_buf[k * 2u];
                let n_y  = pos_buf[k * 2u + 1u];
                let n_vx = vel_buf[k * 2u];
                let n_vy = vel_buf[k * 2u + 1u];
                var dx = my_x - n_x;
                var dy = my_y - n_y;
                if dx >  hw { dx -= W; }
                if dx < -hw { dx += W; }
                if dy >  hh { dy -= H; }
                if dy < -hh { dy += H; }
                let d2 = dx * dx + dy * dy;
                if d2 > EPS2 && d2 <= params.coh_r2 {
                    // Cohesion: sum of vectors pointing toward neighbor.
                    coh_x -= dx;
                    coh_y -= dy;
                    coh_cnt++;
                    // Alignment: independent of sep_r2 check.
                    if d2 < params.ali_r2 {
                        ali_x += n_vx;
                        ali_y += n_vy;
                        ali_cnt++;
                    }
                    // Separation: independent of ali_r2 check.
                    if d2 < params.sep_r2 {
                        let inv_d = 1.0 / sqrt(max(d2, EPS2));
                        sep_x += dx * inv_d;
                        sep_y += dy * inv_d;
                        sep_cnt++;
                    }
                }
            }
        }
    }

    // Apply separation.
    if params.w_sep != 0.0 && sep_cnt > 0u {
        let mag = sqrt(sep_x * sep_x + sep_y * sep_y);
        if mag > EPS {
            let des_x = sep_x / mag * params.max_speed;
            let des_y = sep_y / mag * params.max_speed;
            var stx = des_x - my_vx;
            var sty = des_y - my_vy;
            let stm = sqrt(stx * stx + sty * sty);
            if stm > params.max_force {
                stx = stx / stm * params.max_force;
                sty = sty / stm * params.max_force;
            }
            acc_x += stx * params.w_sep;
            acc_y += sty * params.w_sep;
        }
    }

    // Apply alignment.
    if params.w_ali != 0.0 && ali_cnt > 0u {
        let mag = sqrt(ali_x * ali_x + ali_y * ali_y);
        if mag > EPS {
            let des_x = ali_x / mag * params.max_speed;
            let des_y = ali_y / mag * params.max_speed;
            var stx = des_x - my_vx;
            var sty = des_y - my_vy;
            let stm = sqrt(stx * stx + sty * sty);
            if stm > params.max_force {
                stx = stx / stm * params.max_force;
                sty = sty / stm * params.max_force;
            }
            acc_x += stx * params.w_ali;
            acc_y += sty * params.w_ali;
        }
    }

    // Apply cohesion (suppressed while fleeing, matching CPU behavior).
    if params.w_coh != 0.0 && coh_cnt > 0u && !is_fleeing {
        let mag = sqrt(coh_x * coh_x + coh_y * coh_y);
        if mag > EPS {
            let des_x = coh_x / mag * params.max_speed;
            let des_y = coh_y / mag * params.max_speed;
            var stx = des_x - my_vx;
            var sty = des_y - my_vy;
            let stm = sqrt(stx * stx + sty * sty);
            if stm > params.max_force {
                stx = stx / stm * params.max_force;
                sty = sty / stm * params.max_force;
            }
            acc_x += stx * params.w_coh;
            acc_y += sty * params.w_coh;
        }
    }

    out_acc[i * 2u]      = acc_x;
    out_acc[i * 2u + 1u] = acc_y;
}
"""

# Params struct format for struct.pack (little-endian, no implicit padding).
# 24 fields × 4 bytes = 96 bytes = 6 × 16 → valid WGSL uniform alignment.
# Field order MUST match the WGSL Params struct exactly.
_PARAMS_FMT: str = "<4I4f4f4ffI2f2fII"


# ─────────────────────────────────────────────────────────────────────────────


class SardineGPUBackend:
    """Manages GPU buffers and compute pipeline for sardine flocking forces.

    Lifecycle
    ---------
    1. Construct once; this compiles the WGSL shader and allocates GPU resources.
    2. Call ``compute_forces()`` each frame instead of the Python cell-bucket loop.
    3. The GPU backend is destroyed implicitly when the Python object is GC'd.
       Call ``destroy()`` explicitly on shutdown for deterministic cleanup.

    Thread safety: not thread-safe — call exclusively from the simulation thread.
    """

    def __init__(self) -> None:
        adapter = wgpu.gpu.request_adapter_sync(power_preference="high-performance")
        self._device: wgpu.GPUDevice = adapter.request_device_sync()
        dev = self._device

        # Compile WGSL once; Metal/Vulkan IR is cached by wgpu-native.
        self._shader = dev.create_shader_module(code=_WGSL)

        # Fixed bind group layout — independent of buffer capacity.
        self._bgl = dev.create_bind_group_layout(entries=[
            {"binding": 0, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
            {"binding": 1, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
            {"binding": 2, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
            {"binding": 3, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
            {"binding": 4, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.read_only_storage}},
            {"binding": 5, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.uniform}},
            {"binding": 6, "visibility": wgpu.ShaderStage.COMPUTE,
             "buffer": {"type": wgpu.BufferBindingType.storage}},
        ])
        pipeline_layout = dev.create_pipeline_layout(bind_group_layouts=[self._bgl])
        self._pipeline = dev.create_compute_pipeline(
            layout=pipeline_layout,
            compute={"module": self._shader, "entry_point": "main"},
        )

        # Fixed-size buffers (not resized as sardine count changes).
        self._sf_pos_buf = dev.create_buffer(
            size=_MAX_SAILFISH * 2 * 4,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_DST,
        )
        self._params_buf = dev.create_buffer(
            size=_PARAMS_SIZE,
            usage=wgpu.BufferUsage.UNIFORM | wgpu.BufferUsage.COPY_DST,
        )

        # Variable-capacity buffers and bind group — rebuilt in _ensure_capacity.
        self._cap:          int = 0
        self._total_cells:  int = 0
        self._pos_buf           = None
        self._vel_buf           = None
        self._cell_starts_buf   = None
        self._cell_counts_buf   = None
        self._out_buf           = None
        self._bind_group        = None

        # Pre-allocated CPU scratch arrays — grown alongside GPU buffers.
        self._cpu_cell_starts: np.ndarray = np.empty(0, dtype=np.uint32)
        self._cpu_cell_counts: np.ndarray = np.empty(0, dtype=np.uint32)

    # ── Capacity management ───────────────────────────────────────────────────

    def _ensure_capacity(self, n: int, total_cells: int) -> None:
        """Grow GPU buffers when n or total_cells exceeds current allocation."""
        if n <= self._cap and total_cells == self._total_cells:
            return
        dev = self._device
        new_cap  = max(n * 2, 64)
        new_tc   = total_cells
        self._cap         = new_cap
        self._total_cells = new_tc

        sz_agent = new_cap * 2 * 4   # bytes for (cap, 2) float32
        sz_cells = new_tc * 4        # bytes for (total_cells,) uint32

        self._pos_buf = dev.create_buffer(
            size=sz_agent,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_DST,
        )
        self._vel_buf = dev.create_buffer(
            size=sz_agent,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_DST,
        )
        self._cell_starts_buf = dev.create_buffer(
            size=sz_cells,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_DST,
        )
        self._cell_counts_buf = dev.create_buffer(
            size=sz_cells,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_DST,
        )
        # Output buffer: writable by GPU shader, readable back to CPU.
        self._out_buf = dev.create_buffer(
            size=sz_agent,
            usage=wgpu.BufferUsage.STORAGE | wgpu.BufferUsage.COPY_SRC,
        )

        # Rebuild bind group referencing the new buffer objects.
        self._bind_group = dev.create_bind_group(
            layout=self._bgl,
            entries=[
                {"binding": 0, "resource": {"buffer": self._pos_buf,
                                             "offset": 0, "size": sz_agent}},
                {"binding": 1, "resource": {"buffer": self._vel_buf,
                                             "offset": 0, "size": sz_agent}},
                {"binding": 2, "resource": {"buffer": self._cell_starts_buf,
                                             "offset": 0, "size": sz_cells}},
                {"binding": 3, "resource": {"buffer": self._cell_counts_buf,
                                             "offset": 0, "size": sz_cells}},
                {"binding": 4, "resource": {"buffer": self._sf_pos_buf,
                                             "offset": 0,
                                             "size": _MAX_SAILFISH * 2 * 4}},
                {"binding": 5, "resource": {"buffer": self._params_buf,
                                             "offset": 0, "size": _PARAMS_SIZE}},
                {"binding": 6, "resource": {"buffer": self._out_buf,
                                             "offset": 0, "size": sz_agent}},
            ],
        )

        # Grow CPU scratch arrays alongside GPU buffers.
        self._cpu_cell_starts = np.empty(new_tc, dtype=np.uint32)
        self._cpu_cell_counts = np.empty(new_tc, dtype=np.uint32)

    # ── Sorted spatial hash build (CPU) ───────────────────────────────────────

    def _build_sorted_cells(
        self,
        n: int,
        pos: np.ndarray,
        cell_size: float,
        nc: int,
        nr: int,
    ) -> np.ndarray:
        """Sort sardines by grid cell and build cell_starts / cell_counts.

        Returns sorted_idxs (n,) int64: sorted_idxs[sorted_rank] = orig_idx.
        Side-effects: fills self._cpu_cell_starts and self._cpu_cell_counts.
        """
        total_cells = nc * nr
        col_ids = (pos[:n, 0] / cell_size).astype(np.int32) % nc
        row_ids = (pos[:n, 1] / cell_size).astype(np.int32) % nr
        cell_ids = col_ids * nr + row_ids    # (n,) int32

        sorted_idxs = np.argsort(cell_ids, kind="stable")   # (n,) int64
        sorted_cids = cell_ids[sorted_idxs]                  # (n,) sorted

        counts = np.bincount(sorted_cids, minlength=total_cells).astype(np.uint32)
        starts = np.empty(total_cells, dtype=np.uint32)
        starts[0] = 0
        if total_cells > 1:
            np.cumsum(counts[:-1], out=starts[1:])

        self._cpu_cell_starts[:total_cells] = starts
        self._cpu_cell_counts[:total_cells] = counts
        return sorted_idxs

    # ── Params packing ────────────────────────────────────────────────────────

    @staticmethod
    def _pack_params(
        n: int, nc: int, nr: int, coh_span: int,
        W: float, H: float, cell_size: float,
        sep_r2: float, ali_r2: float, coh_r2: float, flee_r2: float,
        max_speed: float, max_force: float,
        w_sep: float, w_ali: float, w_coh: float, w_flee: float,
        n_sailfish: int,
        mouse_x: float, mouse_y: float, mouse_r2: float,
        mouse_flee_w: float, mouse_active: int,
    ) -> bytes:
        return struct.pack(
            _PARAMS_FMT,
            # offset  0: 4 × u32
            n, nc, nr, coh_span,
            # offset 16: 4 × f32
            W, H, cell_size, sep_r2,
            # offset 32: 4 × f32
            ali_r2, coh_r2, flee_r2, max_speed,
            # offset 48: 4 × f32
            max_force, w_sep, w_ali, w_coh,
            # offset 64: f32, u32, f32, f32
            w_flee, n_sailfish, mouse_x, mouse_y,
            # offset 80: f32, f32, u32, u32
            mouse_r2, mouse_flee_w, mouse_active, 0,  # 0 = _pad
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def compute_forces(
        self,
        n: int,
        pos: np.ndarray,
        vel: np.ndarray,
        sailfish_list: list,
        mouse_stimulus: object,
        cfg: "SimConfig",
    ) -> np.ndarray:
        """Compute sardine flocking forces (flee + sep + ali + coh) on GPU.

        Parameters
        ----------
        n:              Number of active sardines.
        pos:            _s_pos (N_cap, 2) float64 — positions (first n rows used).
        vel:            _s_vel (N_cap, 2) float64 — velocities (first n rows used).
        sailfish_list:  list[Sailfish] — active predators.
        mouse_stimulus: _MouseStimulus | None — mouse predator state.
        cfg:            Current SimConfig.

        Returns
        -------
        (n, 2) float64 array of accelerations in original sardine-index order.
        Noise (pass 3) is NOT included; the caller must add it.
        """
        W         = float(cfg.window_width)
        H         = float(cfg.window_height)
        cell_size = float(cfg.cell_size)
        nc        = max(1, math.ceil(W / cell_size))
        nr        = max(1, math.ceil(H / cell_size))
        total_cells = nc * nr

        self._ensure_capacity(n, total_cells)

        coh_span = max(1, math.ceil(float(cfg.radius_cohesion) / cell_size))

        # ── Build sorted cell structure on CPU ─────────────────────────────
        sorted_idxs = self._build_sorted_cells(n, pos, cell_size, nc, nr)

        # Rearrange positions and velocities into sorted cell order (float32).
        sorted_pos = pos[sorted_idxs].astype(np.float32)   # (n, 2) C-contiguous
        sorted_vel = vel[sorted_idxs].astype(np.float32)   # (n, 2) C-contiguous

        # ── Pack sailfish positions ────────────────────────────────────────
        n_sf   = min(len(sailfish_list), _MAX_SAILFISH)
        sf_arr = np.empty(n_sf * 2, dtype=np.float32)
        for si, sf in enumerate(sailfish_list[:n_sf]):
            sf_arr[si * 2]     = float(sf._gx)
            sf_arr[si * 2 + 1] = float(sf._gy)

        # ── Determine mouse predator state ────────────────────────────────
        ms = mouse_stimulus
        mouse_active = (
            1 if (ms is not None and ms.active and cfg.mouse_predator_enabled) else 0
        )
        if mouse_active:
            mouse_x       = float(ms.x)
            mouse_y       = float(ms.y)
            mouse_r2      = float(ms.radius_sq)
            mouse_flee_w  = float(ms.flee_weight_mult)
        else:
            mouse_x = mouse_y = mouse_r2 = 0.0
            mouse_flee_w = 1.0

        # Effective flee weight: 0 when no sailfish and mouse not active.
        have_flee = n_sf > 0 or mouse_active
        w_flee = float(cfg.weight_flee) if have_flee else 0.0

        # ── Pack params uniform struct ────────────────────────────────────
        params_bytes = self._pack_params(
            n=n, nc=nc, nr=nr, coh_span=coh_span,
            W=W, H=H, cell_size=cell_size,
            sep_r2=float(cfg.radius_separation) ** 2,
            ali_r2=float(cfg.radius_alignment) ** 2,
            coh_r2=float(cfg.radius_cohesion) ** 2,
            flee_r2=float(cfg.radius_flee) ** 2,
            max_speed=float(cfg.sardine_max_speed),
            max_force=float(cfg.sardine_max_force),
            w_sep=float(cfg.weight_separation),
            w_ali=float(cfg.weight_alignment),
            w_coh=float(cfg.weight_cohesion),
            w_flee=w_flee,
            n_sailfish=n_sf,
            mouse_x=mouse_x, mouse_y=mouse_y,
            mouse_r2=mouse_r2, mouse_flee_w=mouse_flee_w,
            mouse_active=mouse_active,
        )

        # ── Upload to GPU ─────────────────────────────────────────────────
        q = self._device.queue
        q.write_buffer(self._pos_buf,         0, sorted_pos)
        q.write_buffer(self._vel_buf,         0, sorted_vel)
        q.write_buffer(self._cell_starts_buf, 0, self._cpu_cell_starts[:total_cells])
        q.write_buffer(self._cell_counts_buf, 0, self._cpu_cell_counts[:total_cells])
        if n_sf > 0:
            q.write_buffer(self._sf_pos_buf, 0, sf_arr)
        q.write_buffer(self._params_buf, 0, params_bytes)

        # ── Dispatch compute shader ───────────────────────────────────────
        encoder = self._device.create_command_encoder()
        cp = encoder.begin_compute_pass()
        cp.set_pipeline(self._pipeline)
        cp.set_bind_group(0, self._bind_group)
        cp.dispatch_workgroups(math.ceil(n / _WORKGROUP_SIZE))
        cp.end()
        self._device.queue.submit([encoder.finish()])

        # ── Read back and scatter to original order ───────────────────────
        raw     = self._device.queue.read_buffer(self._out_buf, 0, n * 2 * 4)
        out_f32 = np.frombuffer(raw, dtype=np.float32).reshape(n, 2)

        # Scatter sorted → original index order: out_orig[sorted_idxs[j]] = out_f32[j]
        out_orig = np.empty((n, 2), dtype=np.float64)
        out_orig[sorted_idxs] = out_f32

        return out_orig

    def destroy(self) -> None:
        """Destroy the underlying GPU device and release all GPU resources."""
        self._device.destroy()


# ── Module-level factory ──────────────────────────────────────────────────────

def try_create_backend() -> "SardineGPUBackend | None":
    """Attempt to create a SardineGPUBackend.

    Returns the backend on success, or None if wgpu is unavailable or
    initialization fails.  Any failure is logged as a warning so the caller
    can silently fall back to the CPU path.
    """
    if not _WGPU_AVAILABLE:
        log.warning(
            "sardine_gpu: wgpu not installed — GPU backend unavailable. "
            "Run: pip install wgpu"
        )
        return None
    try:
        backend = SardineGPUBackend()
        log.info(
            "sardine_gpu: WebGPU compute backend initialized (wgpu-py %s).",
            wgpu.__version__,
        )
        return backend
    except Exception as exc:  # noqa: BLE001
        log.warning(
            "sardine_gpu: GPU backend init failed (%s). Falling back to CPU path.", exc
        )
        return None
