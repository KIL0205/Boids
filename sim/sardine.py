"""
sim/sardine.py
==============
``Sardine`` — a flocking boid that implements the three classic Reynolds rules:

* **Separation** — maintain personal space; push away from too-close neighbours,
  weighted by 1/distance so the closest fish exert the strongest repulsion.
* **Alignment** — match the average heading of nearby neighbours.
* **Cohesion** — steer toward the centre of mass of the local group.

All neighbour lookups go through the shared ``SpatialHash`` grid injected at
construction time.  The grid is rebuilt by ``Simulation`` every frame before
calling ``update(dt)``, so queries always reflect current positions.

Weights and perception radii are read from ``SimConfig`` — no magic numbers
here.
"""

from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING

import numpy as np

from sim.boid import Boid
from config.config import SimConfig
from utils.vec2 import Vec2, wrap_delta

if TYPE_CHECKING:
    from sim.spatial_hash import SpatialHash

# Guard against zero-distance division (squared, in pixels²).
_EPS2: float = 1e-18


class Sardine(Boid):
    """Flocking sardine boid.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config — kinematics, flocking weights, radii.
    grid:
        The sardine-only spatial hash.  Must be the *same object* rebuilt
        by ``Simulation`` every frame so flocking queries always see current
        sardine positions.  Sailfish are never inserted into this grid.
    sailfish_grid:
        The sailfish-only spatial hash used for predator detection.  When
        provided, sardines flee any sailfish within ``config.radius_flee``.
        Pass ``None`` (default) to disable flee behaviour — useful for
        Phase-1 tests or standalone flocking experiments.
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        grid: "SpatialHash",
        sailfish_grid: "SpatialHash | None" = None,
        mouse_stimulus: "object | None" = None,
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.sardine_max_speed,
            max_force=config.sardine_max_force,
            config=config,
            min_speed=config.sardine_min_speed,
        )
        self._grid: "SpatialHash" = grid
        self._sailfish_grid: "SpatialHash | None" = sailfish_grid
        # Shared mutable mouse-predator state from Simulation (or None when
        # sardines are created in isolation for unit tests).
        self._mouse_stimulus: "object | None" = mouse_stimulus

        # Cache per-instance derived values — avoids repeated config lookups and
        # float() conversions in the per-frame _compute_forces hot path.
        self._W:      float = float(config.window_width)
        self._H:      float = float(config.window_height)
        self._hw:     float = self._W * 0.5   # half-world for toroidal wrap
        self._hh:     float = self._H * 0.5
        self._sep_r2:   float = config.radius_separation ** 2
        self._ali_r2:   float = config.radius_alignment  ** 2
        self._coh_r2:   float = config.radius_cohesion   ** 2
        self._flee_r2:  float = config.radius_flee       ** 2
        # Pre-computed cell-span values (= max(1, ceil(radius / cell_size))).
        # Used in the inline cell-traversal loops to avoid max()+ceil() each frame.
        # Updated in apply_config() whenever the relevant radius changes.
        self._coh_span: int = max(1, math.ceil(config.radius_cohesion / grid.cell_size))
        self._flee_span: int = (
            max(1, math.ceil(config.radius_flee / sailfish_grid.cell_size))
            if sailfish_grid is not None else 1
        )

        # ── Ecology counters (Phase 8) ────────────────────────────────
        # food_counter: number of algae eaten; used in Stage 8.4 to trigger
        # reproduction when it exceeds sardine_eat_threshold.
        # hunger_timer: seconds elapsed since last algae contact; reset to 0.0
        # each time the sardine eats.  Initialised from config so freshly
        # spawned sardines don't all die at the same simulation time.
        # Used in Stage 8.5 for starvation.
        self.food_counter: float = 0.0
        self.hunger_timer: float = config.sardine_initial_hunger_time

        # Liveness flag — set to False by Simulation when this sardine is eaten
        # or starved so sailfish can detect target death without querying the grid.
        self.is_alive: bool = True

    # ── Flocking rules ────────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Apply flee, separation, alignment, and cohesion steering forces.

        Order of priority (highest to lowest):

        1. **Flee** — if any sailfish is within *radius_flee*, steer directly
           away from the combined threat and suppress cohesion so the school
           can split.  Fires even when there are no sardine neighbours.
        2. **Separation** — maintain personal space.
        3. **Alignment** — match local headings (kept during flee so
           sub-groups stay coordinated while escaping).
        4. **Cohesion** — steer toward group centre.  Zeroed while fleeing;
           resumes automatically once the sailfish exits *radius_flee*.
        5. **Noise** — small organic perturbation.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        # Cache array ref once (saves one LOAD_ATTR for y vs two self.position
        # lookups), then convert elements to Python floats so the inner-loop
        # arithmetic chain stays in CPython's native float path.  numpy scalar
        # ops (when px/py are numpy.float64) are 2–4× slower per operation.
        _pos = self.position
        px   = float(_pos[0])
        py   = float(_pos[1])

        # ── 1. Flee — inline cell traversal (sailfish grid) ──────────────────
        # Walks the sailfish grid directly without building a predators list.
        # When no sailfish are in flee radius (the common case), this exits
        # almost instantly — 9 dict misses — vs the full query() pipeline
        # (span setup, max/ceil, 9 dict.get, list alloc, empty return) per
        # sardine per frame.
        fleeing = False
        _sf_grid = self._sailfish_grid
        if _sf_grid is not None:
            _sf_cells   = _sf_grid._cells
            _sf_cs      = _sf_grid.cell_size
            _sf_nc      = _sf_grid._num_cols
            _sf_nr      = _sf_grid._num_rows
            _flee_r2    = self._flee_r2
            _sf_span    = self._flee_span
            _sf_col_c   = int(px // _sf_cs)
            _sf_row_c   = int(py // _sf_cs)
            fvx = fvy   = 0.0

            if _sf_nc > 2 * _sf_span and _sf_nr > 2 * _sf_span:
                # Fast path — no visited-set needed (same guarantee as query()).
                for _dc in range(-_sf_span, _sf_span + 1):
                    _col = (_sf_col_c + _dc) % _sf_nc
                    for _dr in range(-_sf_span, _sf_span + 1):
                        _cell = _sf_cells.get((_col, (_sf_row_c + _dr) % _sf_nr))
                        if _cell is None:
                            continue
                        for sf in _cell:
                            dx = px - sf._gx    # pre-cached Python float from insert()
                            dy = py - sf._gy
                            if dx > hw:    dx -= W
                            elif dx < -hw: dx += W
                            if dy > hh:    dy -= H
                            elif dy < -hh: dy += H
                            d2 = dx * dx + dy * dy
                            if d2 <= _flee_r2 and d2 > _EPS2:
                                inv_d = 1.0 / math.sqrt(d2)
                                fvx += dx * inv_d
                                fvy += dy * inv_d
            else:
                # Slow path (tiny worlds / tests) — use query() for correctness.
                for sf in _sf_grid.query(self, cfg.radius_flee):
                    dx = px - sf._gx
                    dy = py - sf._gy
                    if dx > hw:    dx -= W
                    elif dx < -hw: dx += W
                    if dy > hh:    dy -= H
                    elif dy < -hh: dy += H
                    d2 = dx * dx + dy * dy
                    if d2 > _EPS2:
                        inv_d = 1.0 / math.sqrt(d2)
                        fvx += dx * inv_d
                        fvy += dy * inv_d

            if fvx * fvx + fvy * fvy > _EPS2:
                self._steer_apply_xy(fvx, fvy, cfg.weight_flee)
                fleeing = True

        # ── 1b. Mouse predator flee ───────────────────────────────────────
        # The same flee weight is used so a mouse press feels like a real
        # predator.  flee_weight_mult (default 1.0) allows independent tuning.
        # Skipped when mouse_stimulus is None (standalone / test sardines).
        ms = self._mouse_stimulus
        if ms is not None and ms.active and cfg.mouse_predator_enabled:
            dx = px - ms.x
            dy = py - ms.y
            # Inline toroidal wrap — same pattern as the sailfish flee loop.
            if dx > hw:    dx -= W
            elif dx < -hw: dx += W
            if dy > hh:    dy -= H
            elif dy < -hh: dy += H
            d2 = dx * dx + dy * dy
            if 0.0 < d2 < ms.radius_sq:
                inv_d = 1.0 / math.sqrt(d2)
                self._steer_apply_xy(
                    dx * inv_d, dy * inv_d,
                    cfg.weight_flee * ms.flee_weight_mult,
                )
                fleeing = True

        # ── 2–4. Flocking rules — inline cell traversal ───────────────────
        # By walking the spatial grid cells directly we avoid:
        #   (a) materialising a neighbour list (allocation + N appends),
        #   (b) a second pass that recomputes dx/dy/d_sq for each neighbour
        #       that query() already computed for the inclusion check.
        # The slow path (tiny worlds / tests) falls back to query() + loop.
        sep_r2 = self._sep_r2
        ali_r2 = self._ali_r2
        coh_r2 = self._coh_r2

        # Float-pair accumulators avoid 3× np.zeros(2) allocations per frame
        # and save numpy iadd overhead in the alignment accumulation.
        sx = sy = 0.0    # separation: weighted unit away-vectors
        alx = aly = 0.0  # alignment:  sum of neighbour velocities
        cx = cy = 0.0    # cohesion:   vector toward group centre
        sep_count = ali_count = coh_count = 0

        _grid  = self._grid
        _cells = _grid._cells
        _cs    = _grid.cell_size
        _nc    = _grid._num_cols
        _nr    = _grid._num_rows
        _span  = self._coh_span
        _col_c = int(px // _cs)
        _row_c = int(py // _cs)

        if _nc > 2 * _span and _nr > 2 * _span:
            # Fast path — nc/nr always >> 2 in any real world (cell_size ≥ radius
            # means span=1, and worlds are dozens of cells wide in each dimension).
            for _dc in range(-_span, _span + 1):
                _col = (_col_c + _dc) % _nc
                for _dr in range(-_span, _span + 1):
                    _cell = _cells.get((_col, (_row_c + _dr) % _nr))
                    if _cell is None:
                        continue
                    for other in _cell:
                        if other is self:
                            continue
                        dx = px - other._gx    # pre-cached Python float from insert()
                        dy = py - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        d_sq = dx * dx + dy * dy
                        if d_sq > coh_r2:
                            continue

                        # ── Separation: unit away-vector weighted by 1/distance ──
                        if d_sq < sep_r2:
                            if d_sq > _EPS2:
                                inv_d = 1.0 / math.sqrt(d_sq)
                                sx += dx * inv_d
                                sy += dy * inv_d
                            sep_count += 1

                        # ── Alignment: accumulate neighbour velocities ────────────
                        if d_sq < ali_r2:
                            alx += other._gvx   # pre-cached velocity from insert()
                            aly += other._gvy
                            ali_count += 1

                        # ── Cohesion: accumulate toward-neighbour vectors ─────────
                        cx -= dx
                        cy -= dy
                        coh_count += 1
        else:
            # Slow path (tiny worlds / tests) — use query() for correctness.
            for other in _grid.query(self, cfg.radius_cohesion):
                dx = px - other._gx
                dy = py - other._gy
                if dx > hw:    dx -= W
                elif dx < -hw: dx += W
                if dy > hh:    dy -= H
                elif dy < -hh: dy += H
                d_sq = dx * dx + dy * dy

                if d_sq < sep_r2:
                    if d_sq > _EPS2:
                        inv_d = 1.0 / math.sqrt(d_sq)
                        sx += dx * inv_d
                        sy += dy * inv_d
                    sep_count += 1
                if d_sq < ali_r2:
                    alx += other._gvx
                    aly += other._gvy
                    ali_count += 1
                cx -= dx
                cy -= dy
                coh_count += 1

        if not coh_count:
            return

        # Inline Reynolds steer for all four force directions to eliminate
        # 4× Python function-call overhead per sardine per frame.
        # ``self.velocity``, ``self.max_speed``, ``self.max_force``, and
        # ``self.acceleration`` are each cached once; reusing them across all
        # four directions saves 12 redundant LOAD_ATTR + conversion ops per
        # sardine per frame (~18 000 saved at 1 500 sardines).
        _vel = self.velocity
        _vx  = float(_vel[0])
        _vy  = float(_vel[1])
        _ms  = self.max_speed
        _mf  = self.max_force
        _acc = self.acceleration

        # ── Separation ─────────────────────────────────────────────────────────────
        if sep_count:
            _mag = math.sqrt(sx * sx + sy * sy)
            if _mag > 1e-9:
                _inv = _ms / _mag
                _stx = sx * _inv - _vx
                _sty = sy * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_separation
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w

        # ── Alignment ──────────────────────────────────────────────────────────────
        if ali_count:
            _mag = math.sqrt(alx * alx + aly * aly)
            if _mag > 1e-9:
                _inv = _ms / _mag
                _stx = alx * _inv - _vx
                _sty = aly * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_alignment
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w

        # ── Cohesion (suppressed while fleeing) ────────────────────────────────
        # Cohesion zeroed while fleeing — allows sub-groups to split and
        # scatter.  Resumes automatically once the sailfish exits radius_flee.
        if coh_count and not fleeing:
            _mag = math.sqrt(cx * cx + cy * cy)
            if _mag > 1e-9:
                _inv = _ms / _mag
                _stx = cx * _inv - _vx
                _sty = cy * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_cohesion
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w

        # ── 5. Noise: small random perturbation ───────────────────────────────
        # random.gauss(0, 1) returns a Python float — zero allocation,
        # no numpy dispatch.
        if cfg.weight_noise > 0.0:
            _nx = random.gauss(0.0, 1.0)
            _ny = random.gauss(0.0, 1.0)
            _mag = math.sqrt(_nx * _nx + _ny * _ny)
            if _mag > 1e-9:
                _inv = _ms / _mag
                _stx = _nx * _inv - _vx
                _sty = _ny * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_noise
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w
