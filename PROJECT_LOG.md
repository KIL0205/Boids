# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

> **Working reference for active development.** Optimised for quick orientation, not history.
> Historical build notes, phase changelists, and implementation archaeology → [ARCHIVE.md](ARCHIVE.md)

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

- **Runtime**: `python main.py` — resizable window at screen resolution, 60 fps target.
- **Tests**: `python -m tests.tests` — 53 tests, ~2.3 s.

### Current capabilities
- Sardines flock using Reynolds separation / alignment / cohesion + small noise term.
- Sailfish hunt sardine schools by steering toward the local centre of mass.
- Sardines detect nearby sailfish, flee with full-force escape, and scatter (cohesion suppressed while fleeing).
- Live window resize and fullscreen toggle (world geometry updates in-place).
- Runtime config menu (TAB): 15+ parameters editable live; settings persist across runs.
- Velocity-based fish colouring: 7 sardine palette modes, 5 sailfish colour modes.
- Dynamic population controls from keyboard.
- HUD overlay (FPS, frame timing, agent counts).

### Tech stack
- Python 3.13+, pyglet ≥ 2.1, numpy 2.x
- No pygame. Uses pyglet's OpenGL batch renderer for all drawing.

---

## 2. Architecture

### Folder structure
```
Boids/
├── main.py                   # Entry point — wires config + sim + renderer
├── boids_overrides.json      # Runtime overrides (gitignored; auto-created on save)
├── requirements.txt
├── PROJECT_LOG.md            # This file
├── ARCHIVE.md                # Historical build notes and phase changelists
│
├── config/
│   ├── __init__.py           # Re-exports SimConfig, load_overrides, save_config, ConfigMenu
│   ├── config.py             # SimConfig frozen dataclass — all tunable parameters
│   ├── config_io.py          # load_overrides() / save_config() — JSON persistence
│   └── config_menu.py        # Runtime overlay UI (ConfigMenu class)
│
├── sim/
│   ├── __init__.py           # Re-exports Simulation, Boid, Sardine, Sailfish, SpatialHash
│   ├── boid.py               # Base Boid: kinematics, Euler integration, _compute_forces() hook
│   ├── sardine.py            # Sardine(Boid): flocking + flee
│   ├── sailfish.py           # Sailfish(Boid): COM pursuit
│   ├── simulation.py         # Simulation: owns agents + grids, drives update loop
│   └── spatial_hash.py       # Uniform-grid spatial hash — O(1) neighbour lookup
│
├── rendering/
│   ├── __init__.py           # Re-exports Renderer
│   └── renderer.py           # Renderer + Hud; velocity-colour logic; pyglet batch draw
│
├── tests/
│   ├── __init__.py
│   └── tests.py              # 53 unit + integration tests
│
└── utils/
    ├── __init__.py
    └── vec2.py               # normalize, limit, distance, wrap_delta, wrap_position, etc.
```

### Key class responsibilities

| Class | Module | Role |
|---|---|---|
| `SimConfig` | `config/config.py` | Frozen dataclass — single source of truth for all params |
| `Boid` | `sim/boid.py` | Position/velocity/acceleration; `update(dt)`; `_steer_apply_xy()`; speed clamp; boundary |
| `Sardine(Boid)` | `sim/sardine.py` | Flocking (sep/ali/coh/noise) + flee; holds sardine + sailfish grid refs |
| `Sailfish(Boid)` | `sim/sailfish.py` | COM pursuit; holds sardine grid ref; coasts when no sardines visible |
| `SpatialHash` | `sim/spatial_hash.py` | `insert()`, `query()`, `clear()`, `resize()` with toroidal cell-key wrapping |
| `Simulation` | `sim/simulation.py` | Spawns agents; owns two grids; `update(dt)`, `resize()`, `apply_config()` |
| `Renderer` | `rendering/renderer.py` | `reset()`, `draw()`, `on_resize()`, `toggle_hud()`; velocity-colour logic |
| `ConfigMenu` | `config/config_menu.py` | `toggle()`, `handle_key()`, `draw()`, `on_resize()` |

### Data flow (one frame)

```
pyglet scheduler → update(dt)
  └─ Simulation.update(dt)
       ├─ sardine_grid.clear() + insert all sardines
       ├─ sailfish_grid.clear() + insert all sailfish
       └─ for each agent:
            agent.update(dt)
              ├─ _compute_forces()        ← sardine: flock + flee; sailfish: COM pursuit
              ├─ Euler integration        v += a·dt;  p += v·dt
              ├─ _clamp_speed()           [min_speed, max_speed]
              └─ _apply_boundary()        toroidal wrap

pyglet on_draw → Renderer.draw(simulation)
  ├─ _sync_shapes()  or  _sync_shapes_vel_color()   ← write 3 verts per fish
  ├─ batch.draw()                                    ← one GL call
  ├─ Hud.draw()
  └─ ConfigMenu.draw()  (no-op when closed)
```

**Why two separate grids**: sardines query `_grid` for flocking. If sailfish were in it,
sardines would align and cohere with sailfish. `_sailfish_grid` is queried only for flee.
Sailfish query `_grid` for COM pursuit.

---

## 3. Core Systems

### Simulation loop
- `Simulation.update(dt)` performs two sequential passes: full grid rebuild, then all-agent update.
- `dt` is variable (pyglet scheduler, framerate-independent); clamped to 50 ms to prevent Euler blow-up.
- `_compute_forces()` is the override hook — never call `update()` from inside it.

### Spatial partitioning
- Uniform grid spatial hash. `cell_size ≥ radius_cohesion` keeps neighbour queries to a single 3×3 ring.
- Larger radii work correctly but scan proportionally more cells (O(1) per cell, more cells).
- Toroidal cell-key wrapping: `_candidate_cells()` mod-wraps all (col, row) keys; set deduplication prevents double-visits at boundaries.
- Toroidal distance correction applied inside `query()`.

### Rendering (pyglet)
- One `pyglet.shapes.Triangle` per agent, all in a single `pyglet.graphics.Batch`.
- One GL draw call per frame regardless of population.
- `_sync_shapes()` (white mode) updates only vertex positions — pure Python float math.
- `_sync_shapes_vel_color()` updates positions **and** `tri.color` for dynamic colour modes.
- Mode transition (e.g. turning velocity colour off) triggers one O(N) colour-restore pass on the transition frame only — zero per-frame cost at steady state.

### Runtime config menu
- TAB toggles the overlay. Simulation keeps running while open.
- Modal guard: `handle_key()` returns `True` when open — population controls cannot fire.
- 15 editable parameter rows + 5 group headers + 1 Save row. ~528 px tall panel.
- Every value change immediately calls `Simulation.apply_config()` — effect is visible in the next frame.
- `S` key (from any row) or Enter/←/→ on the Save row writes `boids_overrides.json`.
- Resize-safe: `_build(w, h)` recreates all pyglet objects; panel y clamped to `max(0, py)`.

### Config persistence
- `config_io.load_overrides()` reads `boids_overrides.json` at startup; returns `{}` on any error.
- `window_width` and `window_height` are always skipped (set from the live window).
- Type coercion via `typing.get_type_hints(SimConfig)` — unknown fields and type mismatches skipped.
- Atomic write: write to `.tmp`, then `Path.replace()` rename — crash-safe.
- `Simulation._sync_population_counts()` keeps `config.num_sardines/num_sailfish` in sync after every `add_*`/`remove_*` call so saved population counts match live counts.

### Color system
- `sardine_color_mode` (str, 7 values) and `sailfish_color_mode` (str, 5 values) in `SimConfig`.
- **Sardine modes**: `white` (static), `rainbow` (full HSV wheel), `cool`, `warm`, `ocean`, `sunset`, `mono`.
- **Sailfish modes**: `default` (orange-red), `white`, `red`, `blue`, `black`.
- Dynamic sardine modes use heading as the primary hue axis; speed adds a fine offset; turn rate desaturates.
- **Cyclic palette mapping** (restricted modes): cosine formula `hue = (center + half·cos(raw_hue·2π)) % 1.0` guarantees `raw_hue=0` and `raw_hue=1` produce identical colours — eliminates heading-wrap flicker.
- `rainbow` uses raw_hue directly (inherently cyclic). `mono` encodes speed as brightness; hue is irrelevant.
- Four kinematic weights in config: `vel_color_heading_weight`, `vel_color_speed_weight`, `vel_color_turn_weight`, `vel_color_smoothing` (EMA coefficient).
- V key cycles all 7 sardine modes in order (not saved). Save with S inside the menu to persist.
- Migration: old `boids_overrides.json` with `use_velocity_coloring: true` → `sardine_color_mode = "rainbow"` automatically.

---

## 4. Controls & UI

### Global keys (always active)

| Key | Action |
|---|---|
| `V` | Cycle sardine colour mode (7 modes, not saved) |
| `TAB` | Toggle config menu |
| `H` | Toggle HUD overlay |
| `F` | Toggle fullscreen (fires `on_resize` automatically) |
| `ESC` | Quit (or close menu if open) |

### Population controls (menu closed only)

| Key | Action |
|---|---|
| `↑` | Add 50 sardines |
| `↓` | Remove 50 sardines |
| `→` | Add 1 sailfish |
| `←` | Remove 1 sailfish |

### Config menu (TAB to open/close)

| Key | Action |
|---|---|
| `↑` / `↓` | Navigate rows (skips group headers, wraps) |
| `←` / `→` | Decrease / increase value; cycle option; activate Save row |
| `Shift+←` / `Shift+→` | Coarse step (×10) on numeric rows |
| `↵ Enter` | Cycle forward on cycle rows; activate Save row |
| `S` | Save config from any row |
| `ESC` or `TAB` | Close menu |

### Config menu rows (current, top to bottom)

```
SARDINE FLOCKING
  Sep weight         ←→ 0.1 / Shift 1.0  [0–10]
  Ali weight         ←→ 0.1 / Shift 1.0  [0–10]
  Coh weight         ←→ 0.1 / Shift 1.0  [0–10]
  Noise weight       ←→ 0.01 / Shift 0.1 [0–0.5]
KINEMATICS
  Sardine speed      ←→ 10 / Shift 50     [50–500]
  Sardine force      ←→ 10 / Shift 50     [10–500]
PERCEPTION RADII
  Sep radius         ←→ 2 / Shift 20      [5–100]
  Ali radius         ←→ 2 / Shift 20      [10–150]
  Coh radius         ←→ 2 / Shift 20      [10–150]
FLEE / AVOIDANCE
  Flee weight        ←→ 0.5 / Shift 5     [0–30]
  Flee radius        ←→ 5 / Shift 50      [20–300]
SAILFISH
  Pursuit wt         ←→ 0.1 / Shift 1.0  [0–10]
  Pursuit radius     ←→ 5 / Shift 50      [50–400]
  Sailfish speed     ←→ 10 / Shift 50     [50–600]
  Sailfish force     ←→ 10 / Shift 50     [10–600]
VISUAL
  Sardine color      ←→ cycles 7 modes
  Sailfish color     ←→ cycles 5 modes
  Vel heading wt     ←→ 0.1 / Shift 0.5  [0–2]
  Vel speed wt       ←→ 0.1 / Shift 0.5  [0–2]
  Vel turn wt        ←→ 0.1 / Shift 0.5  [0–2]
  Vel smoothing      ←→ 0.05 / Shift 0.1 [0–0.99]
  Save to file       ←→ / Enter / S
```

### Dynamic window resize
- Dragging the window or pressing F fires `on_resize(w, h)`.
- Flow: `simulation.resize(w, h)` → updates `SimConfig`, agent dims, both spatial hashes, positions; then `renderer.on_resize(w, h)` → repositions HUD; then `menu.on_resize(w, h)` → rebuilds menu shapes.
- Minimum guard: `width < 200 or height < 150` → skip to prevent degenerate state.
- No `renderer.reset()` needed — existing Triangles remain valid.

---

## 5. Configuration System

### Defaults
All defaults live in `config/config.py` → `SimConfig` frozen dataclass. This is the canonical baseline.

### Key config fields

| Field | Default | Notes |
|---|---|---|
| `num_sardines` | 900 | Synced to live count after add/remove |
| `num_sailfish` | 8 | Synced to live count after add/remove |
| `sardine_max_speed` | 150.0 | px/s |
| `sardine_min_speed` | 50.0 | px/s; prevents stall |
| `sardine_max_force` | 150.0 | px/s²; max_force/max_speed ≈ 1.0 = natural turning arc |
| `weight_separation` | 2.0 | Highest priority |
| `weight_alignment` | 1.3 | |
| `weight_cohesion` | 1.0 | |
| `weight_noise` | 0.04 | Set 0.0 to disable |
| `radius_separation` | 40.0 | px; ~3 body-lengths |
| `radius_alignment` | 75.0 | px |
| `radius_cohesion` | 80.0 | px; should satisfy sep < ali ≤ coh ≤ cell_size |
| `cell_size` | 80.0 | Spatial hash cell (px); ≥ `radius_cohesion` for O(1) lookup |
| `radius_flee` | 100.0 | px; sailfish within this → flee fires |
| `weight_flee` | 10.0 | Must exceed `weight_separation` (2.0) |
| `sailfish_cell_size` | 120.0 | px; ≥ `radius_flee` for best perf |
| `sailfish_max_speed` | 250.0 | px/s |
| `sailfish_min_speed` | 80.0 | px/s |
| `sailfish_max_force` | 200.0 | px/s² |
| `radius_sailfish_awareness` | 200.0 | px |
| `weight_pursuit` | 1.5 | |
| `sardine_color_mode` | `"white"` | 7 modes |
| `sailfish_color_mode` | `"default"` | 5 modes |
| `vel_color_heading_weight` | 1.0 | |
| `vel_color_speed_weight` | 0.5 | |
| `vel_color_turn_weight` | 1.0 | |
| `vel_color_smoothing` | 0.85 | EMA coefficient |
| `show_hud` | True | |
| `boundary` | `"wrap"` | Toroidal; `"clamp"` for reflective |
| `background_color` | `(0,20,50,255)` | Deep-water blue |

### Override file
- `boids_overrides.json` (gitignored) — partial override dict.
- Loaded at startup and merged over defaults via `**load_overrides()`.
- Saved via S key in the config menu (atomic write).
- `window_width`/`window_height` always re-read from live window, never from file.

### `apply_config()` propagation
`Simulation.apply_config(new_config)` propagates a new `SimConfig` to all live agents:
- Updates `agent._config` for all.
- Updates `Boid.max_speed`, `max_force`, `min_speed` instance attrs (read every frame by `_steer_apply_xy` / `_clamp_speed`).
- Refreshes `_sep_r2`, `_ali_r2` squared-radius caches on sardines.
- Does **not** change world geometry (that's `resize()`).

---

## 6. Current State

### What works
- All modules import cleanly; no circular imports.
- 53/53 tests pass (`python -m tests.tests`).
- `python main.py` opens a resizable window; all features operational.
- Sardine flocking: stable, tuned, numerically bounded.
- Sailfish pursuit + sardine flee: working, two-grid architecture clean.
- Live resize and fullscreen: in-place world geometry update, no respawn.
- Config menu: 15+ params editable live; save/load round-trip lossless.
- Velocity-based colouring: 7 modes, cosine cyclic mapping (no flicker), sailfish colour modes.
- Population synced to config on add/remove for correct save behaviour.

### Partially complete
- Tests for `config_io` and `apply_config` not yet written (listed in TODO).
- `num_sardines` / `num_sailfish` not editable via the config menu (only via ↑/↓/←/→ keys).

### Known limitations
- **Menu height fixed** (~528 px, 21 rows). No scroll. Adding >5 more rows risks overflow on short windows.
- **No mouse support** in config menu.
- **Cross-field radius constraints** not enforced (`radius_separation < radius_alignment ≤ radius_cohesion`). Agents run at extreme values but flocking may be odd.
- **`cell_size` not updated** when radii change via menu. If `radius_cohesion` is pushed above `cell_size=80`, neighbours beyond 80 px are missed — behaviour degrades gracefully.
- **No undo / reset-to-defaults**. Revert by deleting `boids_overrides.json` or manually editing values.
- **No HiDPI scaling**: if shapes appear half-sized on Retina, add `pyglet.options['hidpi'] = True` in `main.py` before `Window()`.

---

## 7. Performance Notes

### Key optimisations (already applied)
| What | Where | Gain |
|---|---|---|
| `math.sqrt` instead of `np.linalg.norm` | `vec2.py`, `boid.py` | 5–10× for 2-D vectors |
| `_steer_apply_xy(tx, ty, weight)` | `boid.py` | Inlines normalize+limit+accumulate; eliminates 2–4 temp numpy arrays per call |
| Float-pair accumulators (`sx/sy`, `alx/aly`, `cx/cy`) | `sardine.py`, `sailfish.py` | ~2700 `np.zeros(2)` allocations/frame eliminated |
| Plain `dict` + `math.floor` in spatial hash | `spatial_hash.py` | Eliminates numpy dispatch + defaultdict factory overhead |
| Inlined `wrap_delta` in `query()` | `spatial_hash.py` | ~90k fewer function calls/frame at 900 sardines |
| Cached `_W/_H/_hw/_hh/_sep_r2/_ali_r2` | `sardine.py`, `sailfish.py` | Avoids repeated config attribute chain per agent per frame |
| Direct `self.sardines` / `self.sailfish` iteration | `simulation.py` | Eliminates one list allocation per frame |
| Pure Python float math in `_sync_shapes()` | `renderer.py` | ~604 numpy arrays/frame eliminated in rendering hot path |
| Set dedup in `_candidate_cells()` + inlined into `query()` / `query_position()` | `spatial_hash.py` | O(n)→O(1) per key; ~65k tuple comparisons/frame eliminated; method call + intermediate list gone |
| `random.gauss(0,1)` instead of `np.random.randn(2)` for noise | `sardine.py` | 900 numpy array allocations/frame eliminated; Python float return, zero GC pressure |
| Single-sqrt `_clamp_speed()` with in-place velocity modification | `boid.py` | 2→1 `math.sqrt` per agent/frame; eliminates `limit()` call + its possible array allocation |
| `_W` / `_H` cached on `Boid` base class | `boid.py` | Eliminates 2 `float()`+attribute-chain lookups per agent per frame (~1800+/frame) |
| `math.sqrt()` instead of `** 0.5` in `_sync_shapes()` | `renderer.py` | Faster scalar exponentiation; 900+ calls/frame |
| `sardine_size_f` / `sailfish_size_f` cached as locals in `draw()` | `renderer.py` | Avoids `float()` coercion on frozen-dataclass fields every frame |
| Separate `sardines` / `sailfish` loops in `resize()` | `simulation.py` | Eliminates temporary merged list allocation on every resize/fullscreen toggle |

### Scaling limits
- **Bottleneck**: Python per-object loop in `simulation.update()`. Saturates at ~1 500 sardines.
- **Next threshold**: beyond ~1 500, numpy-batched force computation (all-sardine positions in a single `(N,2)` array) would eliminate per-object interpreter overhead.
- **Sailfish**: each queries the sardine grid once per frame, O(k). Negligible at ≤ 30 sailfish.
- **Renderer**: `_sync_shapes` is O(N) pure Python; already optimised. At 1 500 agents ≈ 3–5 ms.
- **`query()` result list**: still allocates a `list["Boid"]` per call (~1800+/frame). Eliminating this would require a callback/iterator API — significant refactor with limited payoff at current scale.

---

## 8. TODO / Next Steps

### High priority
1. **Add `TestConfigPersistence` and `TestApplyConfig` test suites** to `tests/tests.py`.
   - `TestConfigPersistence`: `load_overrides` edge cases (missing file, corrupt JSON, unknown fields, type coercion, round-trip).
   - `TestApplyConfig`: confirm `max_speed`/`max_force`/`min_speed` propagation; confirm `_sep_r2`/`_ali_r2` refresh; confirm population count sync.

### Medium priority
2. **`num_sardines` / `num_sailfish` as editable menu rows** — they already persist correctly. Needs respawn or population-adjustment logic on value change (not trivial; separate from the slider model).
3. **Cross-field constraint enforcement** — clamp `radius_separation < radius_alignment` and `radius_alignment ≤ radius_cohesion` inside `_adjust_value()`.

### Phase 7 (future)
4. **Nearest-prey targeting** — individual pursuit rather than group COM; biologically more accurate for a single sailfish.
5. **Sailfish burst acceleration** — short high-speed sprint with cooldown.
6. **Per-sardine danger memory** — smooth cohesion restoration after escape (timer-based rather than instantaneous).
7. **Numpy batched force computation** — vectorise all-sardine positions into `(N,2)` array for SIMD-level speedup beyond ~1 500 sardines.
8. **HiDPI / Retina support** — `pyglet.options['hidpi'] = True` before `Window()`.

---

## 9. References

- **[ARCHIVE.md](ARCHIVE.md)** — Phase-by-phase build history, implementation archaeology, subclass interface contract, algorithmic complexity reference, historical tuning table.
- **[tests/tests.py](tests/tests.py)** — 53 tests across 5 suites: `TestVec2`, `TestBoidKinematics`, `TestSpatialHash`, `TestSimulationIntegrity`, `TestPhase2`.
- **[config/config.py](config/config.py)** — Authoritative parameter definitions with inline comments.
