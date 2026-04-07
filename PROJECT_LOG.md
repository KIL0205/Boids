# PROJECT_LOG.md
## Sardine Schooling & Sailfish Hunting Simulation

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

**Current Phase:** Phase 5 complete — Remapped sailfish controls (arrow keys), runtime window resize, and fullscreen toggle

### High-level description
- Sardines flock using classic Reynolds boids rules (separation, alignment, cohesion).
- Sailfish (Phase 2) will hunt sardine schools by steering toward local centers of mass.
- Sardines (Phase 2) will detect nearby sailfish, temporarily reduce cohesion, and split before regrouping.

### Key design decisions
| Decision | Choice | Rationale |
|---|---|---|
| Renderer | pyglet ≥ 2.0 | No pygame; pyglet is pure-Python, actively maintained, supports OpenGL batch rendering |
| Vector math | numpy float64 arrays | Fast, readable, easy to batch; no custom Vec2 class needed |
| World boundary | Toroidal wrap | Natural for open-sea schooling; eliminates edge-avoidance special-casing |
| Neighbor lookup | Uniform grid spatial hash | O(1) average insert/query; cell_size = max_perception_radius |
| Timestep | Variable delta-time (pyglet scheduler) | Framerate-independent; dt passed to every update() |
| Boid hierarchy | Boid → Sardine, Boid → Sailfish | Shared kinematics; per-species force logic in subclasses |
| Config | Frozen dataclass (SimConfig) | Single source of truth; immutable after startup; easy to override in tests |

---

## 2. Architecture

### File structure
```
Boids/
├── main.py            # Entry point — wires config, simulation, renderer
├── config.py          # All tunable parameters (SimConfig frozen dataclass)
├── boid.py            # Base Boid: position/velocity/accel, Euler integration, boundary
├── sardine.py         # Sardine subclass — separation / alignment / cohesion
├── sailfish.py        # Sailfish subclass — COM pursuit of sardine schools (Phase 2 ✓)
├── simulation.py      # Simulation controller — owns agents, rebuilds grid, calls update(dt)
├── spatial_hash.py    # Uniform-grid spatial hash — O(1) neighbor lookup, toroidal cell wrapping
├── renderer.py        # pyglet 2.x batch renderer — heading-oriented triangles
├── tests.py           # Unit + integration tests (python tests.py)
├── utils/
│   ├── __init__.py    # Sub-package marker
│   └── vec2.py        # Vector math helpers: normalize, limit, distance, wrap
├── requirements.txt   # pip dependencies
├── boidsNumpy.py      # Original reference file (pygame-based; NOT used in new sim)
└── PROJECT_LOG.md     # This file
```

### Core classes

| Class | File | Responsibilities |
|---|---|---|
| `SimConfig` | config.py | Immutable parameter container; module-level `CONFIG` singleton |
| `Boid` | boid.py | Position, velocity, acceleration; `update(dt)`; `apply_force()`; `steer_toward()`; `_clamp_speed()` (min + max); boundary; `_compute_forces()` override hook |
| `Sardine` | sardine.py | Overrides `_compute_forces()` — single-pass separation + alignment + cohesion via spatial hash; holds injected grid reference |
| `Sailfish` | sailfish.py | Overrides `_compute_forces()` — queries sardine grid, computes toroidal COM, steers toward it |
| `SpatialHash` | spatial_hash.py | `insert()`, `clear()`, `query(agent, radius)` with toroidal cell-key wrapping and distance correction |
| `Simulation` | simulation.py | Spawns agents; each frame: rebuild two grids → update all agents; `diagnostics()` snapshot |
| `Renderer` | renderer.py | `reset(sardines, sailfish)` builds batch of Triangle shapes; `draw(simulation)` syncs vertices + submits one draw call per frame |
| *(functions)* | utils/vec2.py | `normalize`, `limit`, `distance`, `distance_sq`, `wrap_delta`, `wrap_position`, `toroidal_displacement` |

### Patterns
- **Spatial hash**: each frame clear → insert → query.  cell_size ≥ max_perception_radius ensures a 1-ring lookup always suffices.
- **Force accumulation**: `Boid.update()` calls `_compute_forces()` (override point) then integrates once.  Acceleration is zeroed at end of each frame.
- **Read-only grid access**: `Simulation.grid` property lets sardines call `grid.query()` inside `_compute_forces()` without a circular import.

---

## 3. Completed Features

- [x] Project file structure defined and created
- [x] `SimConfig` frozen dataclass with all Phase 1 parameters
- [x] `Boid` base class fully implemented:
  - Attributes: `position`, `velocity`, `acceleration`, `max_speed`, `min_speed`, `max_force`
  - `apply_force(force)` — accumulate steering force into acceleration
  - `steer_toward(target_velocity)` — Reynolds steering with `max_force` clamp
  - `_clamp_speed()` — enforces `[min_speed, max_speed]` bounds on velocity magnitude
  - `update(dt)` — calls `_compute_forces()`, Euler integration, speed clamp, boundary
  - `_compute_forces()` — empty override hook for subclasses
  - `_apply_boundary()` — toroidal wrap or reflective clamp (driven by `config.boundary`)
  - Properties: `speed`, `heading`
  - Type hints and docstrings throughout; no rendering concerns
- [x] `SpatialHash`: insert, clear, query with toroidal distance correction, multi-cell overlap support
- [x] `simulation.py` fully implemented:
  - `Simulation.__init__`: creates `SpatialHash`; calls `_spawn_sardines()` to populate world
  - `_spawn_sardines()`: locally imports `Sardine` (avoids module-level circular import); fixed seed=42 for reproducible spawning; spawns `num_sardines` boids at uniform random positions and random headings at `sardine_min_speed`; passes the same grid object reference to every `Sardine`
  - `update(dt)`: two-phase frame update — phase 1 rebuilds grid (`clear` + `insert` all agents), phase 2 calls `agent.update(dt)` on all agents; grid is fully populated before any `_compute_forces()` runs
  - `grid` property: read-only `SpatialHash` reference for agents and renderer without importing `Simulation`
- `_all_agents()`: returns `list(self.sardines) + list(self.sailfish)`
  - `sardines` list: public; renderer reads `position` and `heading` from here each frame
- [x] `main.py` complete: wires `CONFIG → Simulation → Renderer`; pyglet event loop with `schedule_interval` update and `on_draw` draw
- [x] `requirements.txt` updated (version comment added: Python 3.14, numpy 2.x, pyglet 2.1.x)
- [x] `utils/vec2.py`: `normalize`, `limit`, `distance`, `distance_sq`, `wrap_delta`, `wrap_position`, `toroidal_displacement` — all with type hints and docstrings
- [x] `boid.py` refactored to import from `utils.vec2` (no duplicated helpers)
- [x] `spatial_hash.py` refactored to use shared `wrap_delta` from `utils.vec2`
- [x] `sardine.py` — `Sardine(Boid)` subclass fully implemented:
  - Constructor: `(x, y, vx, vy, config, grid, sailfish_grid=None)` — stores both grids; passes `min_speed` to `super()`. `sailfish_grid=None` is backward-compatible.
  - `_compute_forces()`: single spatial hash query at `radius_cohesion`, single-pass neighbour loop
  - Separation: 1/distance weighted repulsion for neighbours within `radius_separation`
  - Alignment: average velocity of neighbours within `radius_alignment`
  - Cohesion: steer toward COM of all neighbours within `radius_cohesion`
  - All three forces weighted by `weight_separation / alignment / cohesion` from config
  - Uses `wrap_delta` for correct toroidal displacement in all distance/direction calculations
  - No rendering code; no magic numbers
- [x] `simulation._spawn_sardines()` implemented:
  - Imports `Sardine` locally (avoids module-level circular import risk)
  - Fixed RNG seed (42) for reproducible spawning
  - Spawns `num_sardines` boids at uniformly random positions, random headings, initial speed = `sardine_min_speed`
  - Passes `self._grid` reference to every Sardine at construction (grid is same object each frame)
  - `numpy` import added at top of `simulation.py`
- [x] `sardine.py` smoke-tested:
  - Sardine instantiates with correct kinematic params ✓
  - Zero acceleration when alone ✓
  - Separation pushes away from close neighbour (accel_x = −192 px/s²) ✓
  - Cohesion pulls toward distant neighbour (accel_x = +120 px/s²) ✓
  - Simulation spawns 300 `Sardine` instances ✓
  - 10 frames run cleanly; all speeds in `[min_speed, max_speed]` ✓
  - All positions within world bounds after toroidal wrap ✓
- [x] Long-run stability test: 600 frames (~10s simulated)
  - Speed range: [60.0, 131.1] px/s (well within [60, 180] bounds)
  - Mean speed: 82.2 px/s — flocking forces actively modulate speed, no boids stuck at cap
  - No position escapes, no NaN/Inf detected
- [x] `renderer.py` fully implemented:
  - `Renderer(config, window)`: creates `pyglet.graphics.Batch`; sets GL clear colour via `glClearColor`
  - `reset(sardines)`: creates one `pyglet.shapes.Triangle` per sardine, adds to batch; safe to call again on population change
  - `draw(simulation)`: updates all triangle vertices using internal attribute batch write (`_x, _y, _x2, _y2, _x3, _y3` + single `_update_translation()` + `_update_vertices()` per shape), then calls `self._batch.draw()` — one GL draw call regardless of sardine count
  - `_boid_verts(boid, size)`: module-level helper; tip = pos + heading × 2size, base corners = pos − heading × size ± perp × size; returns three (2,) float64 arrays
  - No simulation logic; reads only `position` and `heading` from each `Boid`
  - Sardine colour: `(210, 240, 230, 255)` — silvery blue-white on deep-water background
- [x] `main.py` and renderer integration headless-tested:
  - All module syntax verified via `ast.parse` ✓
  - All imports resolve cleanly ✓
  - `_boid_verts` geometry verified: tip, left, right at correct absolute positions for rightward-heading boid ✓
  - 60 simulated frames post-renderer integration: speeds [60.0, 139.7] px/s ✓
  - Full GL test not run (macOS EGL headless unavailable); code verified by inspection and logic ✓
- [x] Phase 1 flocking tuned (see Phase 1 Tuning entry below)
- [x] **`sailfish.py` fully implemented (Phase 2)**:
  - `Sailfish(Boid)` constructor: `(x, y, vx, vy, config, sardine_grid)` — stores sardine grid ref; passes sailfish kinematics to `super()`
  - `_compute_forces()`: queries sardine grid at `radius_sailfish_awareness`; if no sardines visible, coasts at min_speed with no force; otherwise computes toroidal COM offset via per-component `wrap_delta`; calls `steer_toward(com_offset) * weight_pursuit`
  - Toroidal COM: each axis wrapped independently — correct when school straddles world edge
  - No forces applied when no sardines in awareness radius (idle drift)
- [x] **`simulation.py` extended for Phase 2**:
  - `self.sailfish: list[Boid]` added (uncommenting old stub)
  - `_spawn_sailfish()`: seed 123, spawns `num_sailfish` sailfish at random positions/headings at `sailfish_min_speed`; passes `self._grid` (the sardine-only grid) as `sardine_grid`
  - `update()` refactored: sardine grid rebuilt from `self.sardines` only (sailfish excluded to prevent sardines treating them as flocking neighbours); all agents (sardines + sailfish) updated in `_all_agents()` loop
  - `_all_agents()`: now returns `list(self.sardines) + list(self.sailfish)`
- [x] **`config.py` extended**: Phase 2 parameter block added — `num_sailfish`, `sailfish_max_speed`, `sailfish_min_speed`, `sailfish_max_force`, `radius_sailfish_awareness`, `weight_pursuit`, `sailfish_size`, `sailfish_color`
- [x] **`renderer.py` extended**:
  - `_SAILFISH_COLOR` constant added
  - `self._sailfish_shapes` list added
  - `reset(sardines, sailfish=[])` — backward-compatible signature; creates sailfish triangles in the same batch with orange-red colour and `sailfish_size` scaling
  - `draw()` — iterates sailfish shapes and syncs vertices after sardine pass; one `batch.draw()` call as before
- [x] **`main.py` updated**: `renderer.reset(simulation.sardines, simulation.sailfish)`; window caption updated to "Sardine Schooling & Sailfish Hunt"
- [x] **`spatial_hash.py` toroidal cell-key bug fixed**: `_candidate_cells()` now mod-wraps all (col,row) keys; `set` deduplication prevents double-visits at wrap boundaries. Confirmed by `test_toroidal_x_wrap` / `test_toroidal_y_wrap`.
- [x] **`Simulation.diagnostics()` added**: returns `count`, `mean_speed`, `min_speed`, `max_speed`, `center_of_mass` as a plain dict; tested (`test_diagnostics_*` suite)
- [x] **`tests.py` created and updated**: 53 `unittest` tests across 5 suites; run with `python tests.py`
  - `TestVec2` (14 tests): `normalize`, `limit`, `distance`, `distance_sq`, `wrap_delta`, `wrap_position`, `toroidal_displacement`
  - `TestBoidKinematics` (8 tests): speed property, heading, max_speed clamp, min_speed floor, steer_toward clamp, force accumulation, acceleration reset, boundary wrap
  - `TestSpatialHash` (11 tests): hit, miss, excludes-self, boundary hit, clear, multiple neighbours, toroidal x/y wrap, no duplicates, query_position hit/miss
  - `TestSimulationIntegrity` (9 tests): spawn count, 60-frame speed bounds, position bounds, dt spike, diagnostics (5 tests)
  - `TestPhase2` (7 tests): sailfish spawn count, sailfish speed bounds, sailfish absent from sardine grid, flee fires, flee suppressed without grid, cohesion zeroed when fleeing, cohesion fires when safe
- [x] **Sardine flee behaviour implemented (Phase 2 complete)**:
- [x] **Phase 5 — Remapped sailfish controls, runtime window resize, and fullscreen toggle**:
  - `main.py`: sailfish count controls remapped from `[`/`]` to `←`/`→` (LEFT/RIGHT arrow keys); `F` key added for fullscreen toggle; window opened with `resizable=True`; `_ready` guard added so spurious `on_resize` events during startup are ignored; `on_resize` handler added — calls `simulation.resize()` then `renderer.on_resize()`, enforces a 200×150 px minimum before acting.
  - `simulation.py`: `resize(new_w, new_h)` method added — creates a new `SimConfig` via `dataclasses.replace`, propagates it to every agent's `_config`, updates `_W/_H/_hw/_hh` caches on all sardines and sailfish, calls `SpatialHash.resize()` on both grids, and toroidally clamps all position coordinates to the new bounds.
  - `spatial_hash.py`: `resize(world_width, world_height)` method added — updates `world_width/height`, `_num_cols/_num_rows`, `_half_w/_half_h`, and clears the cell dict (the grid is rebuilt on the next frame anyway).
  - `renderer.py`: `Hud.reposition(window_height)` added — moves the label's `y` anchor when the window height changes; `Renderer.on_resize(width, height)` added — calls `_hud.reposition()` if the HUD is visible; HUD key-binding line updated from `[/] sailfish (+/-1)` to `←/→ sailfish (+/-1)  F fullscreen`.
  - All 53 tests pass.
- [x] **Phase 4 — Performance optimization pass + runtime dynamic population controls**:
  - `utils/vec2.py`: replaced `np.linalg.norm` with `math.sqrt(v[0]*v[0]+v[1]*v[1])` in `normalize`, `limit`, `distance`, `distance_sq` — 5–10× faster for 2-D vectors; `normalize` and `limit` are on the critical path of every steering call.
  - `spatial_hash.py`: removed `defaultdict` (replaced with plain `dict` + conditional insert); replaced `np.floor`/`np.ceil` with `math.floor`/`math.ceil` and `//` operator; inlined `wrap_delta` logic in `query` and `query_position` (saves ~2 function calls per candidate neighbour pair); added `_half_w/_half_h` cached half-dimensions; `insert` now avoids `_cell_key` method call overhead.
  - `boid.py`: `speed` property and `_clamp_speed` use `math.sqrt` instead of `np.linalg.norm`; added `_steer_apply_xy(tx, ty, weight)` — a zero-allocation steering helper that inlines normalize+limit+accumulate into a single Python-float-math function, adding to `acceleration` directly with no intermediate numpy arrays.
  - `sardine.py`: cached `_W`, `_H`, `_hw`, `_hh`, `_sep_r2`, `_ali_r2` as instance attributes at `__init__` (avoids repeated config lookups and `**` ops per frame); replaced 3× `np.zeros(2)` accumulators with 6 plain Python float vars (`sx/sy`, `alx/aly`, `cx/cy`) — eliminates 2 700 numpy array allocations per frame at default population; replaced `ali_sum += other.velocity` (numpy iadd temp array) with `alx += float(...)` scalar addition; inlined toroidal wrap in both flee and flocking loops; switched all three steer-apply calls to `_steer_apply_xy`.
  - `sailfish.py`: same pattern — cached dims, float-pair COM accumulator, inlined wrap, `_steer_apply_xy`.
  - `simulation.py`: `update()` now iterates `self.sardines` then `self.sailfish` directly rather than calling `_all_agents()` (avoids creating a merged list every frame); added `add_sardines(n)`, `remove_sardines(n)`, `add_sailfish(n)`, `remove_sailfish(n)` for runtime population management — each spawns/removes agents and is safe to call at any point between frames.
  - `main.py`: keyboard controls for dynamic fish counts — `↑`/`↓` add/remove 50 sardines; `[`/`]` add/remove 1 sailfish; each calls `renderer.reset()` to sync shapes.
  - `renderer.py`: HUD label now includes a one-line key-binding guide: `↑/↓ sardines (+/-50)  [/] sailfish (+/-1)  H HUD`.
  - `main.py` rewritten: detects screen dimensions at startup (`pyglet.canvas.get_display().get_default_screen()`); creates the window first so macOS can adjust client-area height; reads actual `window.width/height` back; creates `SimConfig(window_width=..., window_height=...)` so the simulation world exactly fills the renderable pixels; ESC and H key bindings added; update timing measured via `time.perf_counter()` and forwarded to `renderer.draw()`
  - `config.py`: `num_sardines` raised to 600, `num_sailfish` raised to 4, `show_hud: bool = True` added, density comment updated
  - `renderer.py`: `import time` added; `Hud` class added — `pyglet.text.Label` anchored top-left, text rebuilt at most 4×/second showing FPS, frame/update/render ms, agent counts, and world dimensions; `Renderer.__init__` creates `Hud` when `config.show_hud` is True; `Renderer.draw()` now accepts `update_ms` kwarg, times its own batch-draw pass, and calls `Hud.update() / Hud.draw()`; `Renderer.toggle_hud()` added for the H key binding; `_sync_shapes()` hot path rewritten to use pure Python float math (heading normalisation + inline vertex computation) eliminating ~604 numpy array allocations per frame at the default population
  - `config.py`: added `radius_flee=100.0`, `weight_flee=4.0`, `sailfish_cell_size=120.0`
  - `sardine.py`: `sailfish_grid=None` optional param added to `__init__`; flee block prepended to `_compute_forces()` — queries sailfish grid, sums unit away-vectors per predator, applies `steer_toward(flee_vec) * weight_flee`; cohesion gated on `not fleeing` so school splits when threatened and regroups automatically when safe
  - `simulation.py`: `_sailfish_grid` SpatialHash added; rebuilt from `self.sailfish` before agent updates; passed to each `Sardine` at spawn
  - All 53 tests pass

---

## 4. Current State

### What works
- All modules import cleanly with no circular imports.
- `SimConfig` is complete and usable with Phase 1 + Phase 2 parameters.
- `utils/vec2.py` is fully functional and tested.
- `Boid` base class is complete and tested.
- `SpatialHash` is fully functional with toroidal cell wrapping.
- `Sardine` flocking is fully implemented, numerically stable, and tuned.
- **`Sailfish` is fully implemented and integrated (Phase 2 ✅)**:
  - 2 sailfish spawn at random positions/headings.
  - Each frame: sailfish query the sardine-only grid within `radius_sailfish_awareness=200 px`, compute the toroidal COM, and apply a Reynolds pursuit force toward it.
  - When no sardines are within awareness radius, sailfish coast at `sailfish_min_speed=80 px/s`.
  - Sailfish are **not** inserted into the sardine grid — sardines cannot flock with them.
  - 60-frame headless test: sailfish speeds in `[80.0, 250.0]` px/s, positions wrapped correctly.
- **Sardine flee behaviour is fully implemented (Phase 2 ✅)**:
  - Each sardine holds a reference to `_sailfish_grid` (injected at spawn) and queries it at the **start** of `_compute_forces()` — before the sardine-neighbour early-return — so flee fires even when isolated.
  - If any sailfish is within `radius_flee=100 px`, the sardine sums unit away-vectors from each predator to form `flee_vec` and applies `steer_toward(flee_vec) * weight_flee=4.0` — the largest force the sardine can receive.
  - Cohesion is zeroed while fleeing so sub-groups can split and scatter. Alignment is kept so each sub-group remains coordinated during the escape. Separation is always active to prevent crowding.
  - When the sailfish exits `radius_flee`, flee force ceases and normal cohesion resumes automatically in the next frame — no state to reset.
- **`python main.py` opens a resizable window sized to the display, with sardines and sailfish. Press F to toggle fullscreen, H to toggle the HUD overlay, ESC to quit.**
- **Window resize** is handled live: dragging the window frame or pressing F to enter/exit fullscreen updates world dimensions, spatial hash cell counts, per-agent cached dims, and HUD placement with no restart required.
- **Fullscreen toggle (F key)**: calls `window.set_fullscreen()`, which automatically fires `on_resize` with the screen's full pixel dimensions.

### How the update loop works

Each call to `Simulation.update(dt)` performs two sequential passes:

```
1a. Sardine grid rebuild
    self._grid.clear()                  # drop all prior-frame entries
    for s in self.sardines:             # insert sardines only
        self._grid.insert(s)            # sailfish intentionally excluded

1b. Sailfish grid rebuild
    self._sailfish_grid.clear()
    for sf in self.sailfish:            # insert sailfish only
        self._sailfish_grid.insert(sf)  # sardines intentionally excluded

2. All-agent update
   for agent in self._all_agents():    # sardines + sailfish
       agent.update(dt)
       # Sardine._compute_forces(): flees from _sailfish_grid, flocks via _grid
       # Sailfish._compute_forces(): queries self._sardine_grid (= self._grid)
       # Euler: v += a*dt, p += v*dt
       # _clamp_speed(): [min_speed, max_speed]
       # _apply_boundary(): toroidal wrap
       # acceleration zeroed
```

**Why sailfish are excluded from the sardine grid**: `Sardine._compute_forces()` queries `self._grid` for all three flocking rules. If sailfish were in this grid, sardines would attempt to align and cohere with sailfish — physically nonsensical. Exclusion is enforced by only iterating `self.sardines` in the grid-rebuild pass.

**dt handling**: `dt` is provided by the pyglet scheduler (variable, framerate-independent). Clamped to `1/20` (50 ms) at the top of `update()` to prevent Euler integration blow-up on frame-drop spikes.

### Assumptions (simulation controller)
- The `SpatialHash` grid is the sole spatial index; all neighbour queries go through it.
- `_spawn_sardines()` is called once at construction; the world population does not change during a run in Phase 1.
- `dt` is supplied by the caller (pyglet scheduler); `Simulation` itself is fully framerate-independent.
- Sardines are the only agents whose flocking grid is populated; sailfish query that grid but are never inserted into it.
- `Simulation.update(dt)` is the sole entry point for advancing world state; callers must not modify `sardines` directly between frames.
- The renderer reads `simulation.sardines` (and later `simulation.sailfish`) each frame; it must not mutate these lists.

### Rendering notes
- Each frame: `renderer.draw(simulation)` computes 3 vertices per sardine, writes them via internal pyglet Triangle attributes, then calls `batch.draw()` — one GL draw call for all 300 triangles.
- `pyglet.shapes.Triangle` in pyglet 2.1.x stores vertex 1 as the GPU translation origin (`_x, _y`); vertices 2 and 3 are stored as absolute world coordinates (`_x2, _y2, _x3, _y3`). The `_update_translation()` + `_update_vertices()` pair flushes all 6 coordinates in one go.
- Background colour is set via `gl.glClearColor` in `Renderer.__init__`; `window.clear()` in `on_draw` uses it.

### Known rendering limitations / tuning notes
- **sardine_size = 6.0**: triangles are 18 px long, 12 px wide. Increase to 8–10 if they appear too small on a high-DPI display.
- **No HiDPI scaling**: pyglet may apply a pixel-ratio scale on Retina displays. If sardines appear half-sized, set `pyglet.options['hidpi'] = True` in `main.py`.

---

## 5. TODO / Next Steps

### Phase 2 (complete ✅)
All Phase 2 items are done: sailfish COM pursuit, sardine flee with cohesion suppression, two-grid architecture, `TestPhase2` suite (53/53 tests passing).

### Tuning suggestions (observable after `python main.py`)
- `radius_flee` (100 px): increase toward 150 if sardines don't react early enough; decrease toward 60 for more dramatic last-second dashes.
- `weight_flee` (4.0): increase to 6.0 if sardines feel sluggish to escape; already exceeds separation (2.0).
- `sailfish_cell_size` must stay ≥ `radius_flee`.
- `HiDPI check`: if shapes appear half-sized on Retina, add `pyglet.options['hidpi'] = True` in `main.py` before `Window()`.

### Phase 6 (future)
- Nearest-prey targeting (individual pursuit rather than group COM)
- Sailfish burst acceleration model
- Per-sardine danger memory + smooth cohesion restoration
- Live resize support (window resized → rebuild SimConfig + respawn/reposition agents)
- `pyglet.options['hidpi'] = True` for Retina display support
- Numpy batched force computation (vectorize all-sardine positions into a single (N,2) array for SIMD-level speedup beyond ~1500 sardines)

---

## 6. Testing Status

### How to run tests
```
python tests.py
```
53 tests, 5 suites. No external dependencies beyond the stdlib `unittest` module.

### Test suites
| Suite | Count | Covers |
|---|---|---|
| `TestVec2` | 14 | `normalize`, `limit`, `distance`, `distance_sq`, `wrap_delta`, `wrap_position`, `toroidal_displacement` |
| `TestBoidKinematics` | 8 | speed property, heading, max/min speed clamping, `steer_toward`, force accumulation, acceleration reset, boundary wrap |
| `TestSpatialHash` | 11 | hit/miss/self-exclusion/boundary/clear/multiple neighbours/toroidal x+y wrap/no duplicates/`query_position` |
| `TestSimulationIntegrity` | 9 | spawn count, 60-frame speed bounds, position bounds, dt spike, `diagnostics()` (5 sub-tests) |
| `TestPhase2` | 7 | sailfish spawn count, sailfish speed bounds, sailfish absent from sardine grid, flee fires, flee absent without grid, cohesion zeroed when fleeing, cohesion fires when safe |

### Bugs found and fixed
- **`SpatialHash` toroidal cell-key wrapping** (critical): `_candidate_cells()` used raw unbounded (col, row) keys, so agents near a world edge could not find neighbours on the opposite side. The distance check inside `query()` was already toroidal, but it only ran on neighbours in candidate cells — which never included the wrapped cells. Fixed by modulo-wrapping all candidate keys against `_num_cols` / `_num_rows` and using a set for deduplication. Verified by `test_toroidal_x_wrap` and `test_toroidal_y_wrap`.
- **`pyglet.text.Label` `bold=True` keyword removed in pyglet 2.1.x** (Phase 6 config menu crash): The `ConfigMenu._build()` method passed `bold=True` to `pyglet.text.Label()` for the header label. In pyglet 2.1.x the `bold` keyword was replaced by `weight: str` (e.g. `weight='bold'`); passing the old kwarg raises `TypeError: Label.__init__() got an unexpected keyword argument 'bold'`, crashing `main()` before the window is displayed. A secondary `AttributeError: 'Label' object has no attribute '_boxes'` was printed during interpreter shutdown — this is noise from CPython's garbage collector calling `Label.__del__()` on the partially-constructed object whose `__init__` raised before `self._boxes` was set; it disappears once the constructor succeeds. The macOS/Cocoa `AttributeError` exceptions (`PygletDelegate has no attribute did_pause_exclusive_mouse`, `PygletDelegate has no attribute _window`) are the same Cocoa-bridge teardown artefacts seen in the Phase 3 crash — they fire when an unhandled Python exception corrupts the Cocoa delegate lifecycle and are **not** caused by new code; they disappear once the primary crash is resolved. **Fix**: replaced `bold=True` with `weight='bold'` in `ConfigMenu._build()`. File changed: `config_menu.py` (one line).

- **`pyglet.clock.get_fps()` removed in pyglet 2.x** (Phase 3 runtime crash): The `Hud.update()` method called `pyglet.clock.get_fps()`, which no longer exists in pyglet ≥ 2.0. Any call to `Renderer.draw()` immediately raised `AttributeError: module 'pyglet.clock' has no attribute 'get_fps'`, crashing the event loop on the very first rendered frame. A cascade of secondary macOS/Cocoa `AttributeError` exceptions (`PygletDelegate has no attribute did_pause_exclusive_mouse`, `PygletView has no attribute _window`) followed — these are noise from the Cocoa bridge receiving window-lifecycle events after the Python-side objects were partially torn down by the unhandled exception; they are **not** caused by the code changes and disappear once the primary crash is resolved. **Fix**: replaced the `pyglet.clock.get_fps()` call with manual FPS tracking inside `Hud` — `_frame_count` is incremented on every `update()` call and divided by the wall-clock elapsed time at each 0.25 s refresh window. The simulation-state read block (`len(simulation.sardines)` etc.) is now wrapped in a `try/except` so a future API change can never crash the app — it falls back to `"n/a"` for any metric that fails. File changed: `renderer.py`.

### What the tests cover
- All `utils/vec2.py` public functions ✓
- `Boid` force accumulation, integration, and speed clamping logic ✓
- `SpatialHash` hit, miss, self-exclusion, clear, multi-neighbour, **both toroidal axes**, no duplicates ✓
- `Simulation` spawn count, 60-frame stability, dt spike safety, `diagnostics()` completeness ✓

### What still needs testing
- **Visual quality**: school + pursuit dynamics and flee responsiveness require live window
- `Sailfish._compute_forces()` when sardines straddle world edge (toroidal COM correctness) — covered by spatial hash toroidal tests; visual confirm pending
- Performance at 600 sardines + 4 sailfish at 60 fps on target hardware
- HiDPI / Retina display rendering
- Renderer GL correctness (GL headless unavailable on macOS without EGL)

### Known bugs
- None. The `pyglet.clock.get_fps()` crash (Phase 3) and the `Label bold=True` crash (Phase 6 config menu) have both been fixed.

### Platform notes (macOS)
- When the simulation window is created at full-screen dimensions without `fullscreen=True`, macOS adjusts the client area for the menu bar. Reading back `window.width/height` after construction (before creating `SimConfig`) ensures the simulation world exactly fills the renderable area.
- The secondary Cocoa `AttributeError` exceptions (`PygletDelegate`/`PygletView`) seen in the original traceback are a pyglet macOS backend artefact that fire when an unhandled Python exception corrupts the Cocoa delegate lifecycle. They do not occur during normal operation and require no code-side mitigation.

---

## 7. Parameters / Tunables

All values live in `config.py` → `SimConfig`.

| Parameter | **Tuned** | Previous | Meaning |
|---|---|---|---|
| `radius_flee` | 100.0 | — | Sailfish detection radius (px); triggers escape |
| `weight_flee` | **6.0** | 4.0 | Flee force weight; must exceed weight_separation (2.0) |
| `sailfish_cell_size` | 120.0 | — | Sailfish-only grid cell size (px); must be ≥ radius_flee |
| `show_hud` | True | — | Show live diagnostics overlay (H key toggles at runtime) |
| `num_sailfish` | **4** | 2 | Number of sailfish predators |
| `window_width` | 1400 | 1400 | Window width (px) |
| `window_height` | 700 | 700 | Window height (px) |
| `target_fps` | 60 | 60 | Target frame rate |
| `num_sardines` | **600** | 300 | Number of sardine boids |
| `sardine_max_speed` | **150.0** | 180.0 | Max sardine speed (px/s) |
| `sardine_min_speed` | **50.0** | 60.0 | Minimum speed to prevent stalling |
| `sardine_max_force` | **150.0** | 240.0 | Steering force budget per tick (px/s²); = max_speed → ratio 1.0 |
| `weight_separation` | **2.0** | 1.8 | Separation force weight (highest priority) |
| `weight_alignment` | **1.3** | 1.0 | Alignment force weight |
| `weight_cohesion` | 1.0 | 1.0 | Cohesion force weight |
| `weight_noise` | **0.04** | — | Random perturbation weight (new; 0.0 to disable) |
| `radius_separation` | **40.0** | 30.0 | Personal-space radius (px); ~3 body-lengths at size=6 |
| `radius_alignment` | **75.0** | 60.0 | Heading-match radius (px) |
| `radius_cohesion` | 80.0 | 80.0 | Group-attraction radius (px) |
| `cell_size` | 80.0 | 80.0 | Spatial hash cell size (px); must be ≥ radius_cohesion |
| `boundary` | `"wrap"` | `"wrap"` | `"wrap"` (toroidal) or `"clamp"` (reflective) |
| `sardine_size` | **6.0** | 5.0 | Triangle half-length for rendering (px); body ≈ 18 px |
| `background_color` | (0,20,50,255) | (0,20,50,255) | Deep-water blue window background |

---

## 8. Subclass Interface Contract

Rules that **all `Boid` subclasses** (Sardine, Sailfish, …) must follow:

| Rule | Detail |
|---|---|
| Call `super().__init__()` | Pass `x, y, vx, vy, max_speed, max_force, config`, and set `min_speed` if the species must not stall |
| Override `_compute_forces()` | Accumulate steering forces via `self.apply_force(force)` — never integrate velocity directly |
| Do not call `update()` from `_compute_forces()` | `update()` is only called by `Simulation`; recursion will break integration |
| Use `self.steer_toward(desired_velocity)` | Returns a force already clamped to `max_force`; pass the result to `apply_force()` |
| Stay rendering-free | Subclasses must not import or call pyglet/renderer code; rendering reads `position` and `heading` externally |
| Access the grid via injection | Receive the `SpatialHash` grid as a parameter to `_compute_forces(grid)` or store it during `Simulation` setup; do not import `Simulation` |

### Subclass convention
Subclasses store their grid references as instance attributes (injected at construction by `Simulation`) and query them inside `_compute_forces()`:

```python
class MyAgent(Boid):
    def __init__(self, x, y, vx, vy, config, grid):
        super().__init__(x, y, vx, vy, max_speed=..., max_force=..., config=config, min_speed=...)
        self._grid = grid  # SpatialHash injected at spawn

    def _compute_forces(self) -> None:
        neighbors = self._grid.query(self, self._config.radius_cohesion)
        # … accumulate forces via self.apply_force() …
```

Never pass the grid as a parameter to `_compute_forces()` — `Boid.update()` calls it with no arguments.

---

## 9. Notes / Ideas

### Future behavioral improvements
- **Sardine speed modulation**: fish speed up when fleeing, slow in tight schools.
- **Weighted alignment**: weight neighbours by inverse distance for tighter local matching.
- **Sailfish ambush**: sailfish accelerates in bursts rather than constant pursuit.
- **Splitting heuristic**: when a sardine detects a sailfish, it zeros cohesion for N seconds, then gradually restores it.
- **Multiple sailfish**: second sailfish coordinates attack from opposite side (emergent pincer behavior).
- **Trail rendering**: faint fade trail behind sailfish for dramatic effect.
- **School detection**: cluster sardines into named schools for stats/HUD display.

### Architecture notes
- `boidsNumpy.py` is the original reference (pygame-based); it is kept for reference only and should not be imported.
- If sardine count exceeds ~1000, consider per-frame numpy batching (store all positions in a single (N,2) array and vectorize force computation) rather than per-instance loops.
- The two-grid architecture (`_grid` for sardines, `_sailfish_grid` for sailfish) is already in place. Adding a third species would follow the same pattern: add a new `SpatialHash`, rebuild it in `update()`, inject it into any agent that needs to query it.

---

## 10. Phase 2 Summary

Phase 2 is complete. All items in the status table below were implemented.

### Status

| Step | File | Status |
|---|---|---|
| Phase 2 params in `config.py` | config.py | ✅ Done |
| `Sailfish(Boid)` — COM pursuit | sailfish.py | ✅ Done |
| `Simulation` integration (spawn, grid isolation, update loop) | simulation.py | ✅ Done |
| Renderer — sailfish triangles | renderer.py | ✅ Done |
| `main.py` wired for sailfish | main.py | ✅ Done |
| Sardine flee behaviour | sardine.py | ✅ Done |
| `Simulation._sailfish_grid` | simulation.py | ✅ Done |
| `TestPhase2` test suite | tests.py | ✅ Done (53/53 passing) |

### Design decisions

| Decision | Choice | Rationale |
|---|---|---|
| Sailfish targeting | Local sardine COM within `radius_sailfish_awareness` | Stable target; naturally creates herding behaviour; avoids jittery target-switching |
| Flee detection | Per-frame sailfish spatial hash query (stateless) | No per-boid flags; flee force fades automatically when sailfish exits `radius_flee` |
| Cohesion during flee | **Zeroed** when any sailfish detected within `radius_flee` | School splits naturally; sub-groups fleeing different directions diverge |
| Alignment during flee | **Kept at normal weight** | Sardines remain coordinated within each fleeing sub-group — biologically realistic |
| Regrouping when safe | **Automatic** — normal flocking resumes | Cohesion pulls scattered fish back together; no state to reset |
| Grid architecture | **Two separate SpatialHash instances** | `_grid` = sardines only; `_sailfish_grid` = sailfish only; no type-filtering needed |
| `Sardine` constructor | `sailfish_grid=None` optional param | Fully backward-compatible; all existing tests pass unchanged |
| Nearest-prey targeting | Deferred to Phase 3 (optional) | COM targeting is more stable for first pass; individual prey can be added with a flag |
| `test_sailfish_speed_bounds` | 60-frame run: all sailfish speeds in `[sailfish_min_speed, sailfish_max_speed]` |
| `test_sailfish_not_in_sardine_grid` | sailfish absent from sardine grid after `update()` |
| `test_sardine_flee_fires` | Sardine with sailfish inside `radius_flee` → non-zero accel before integration |
| `test_sardine_cohesion_skipped_when_fleeing` | Same → cohesion force component absent |
| `test_sardine_safe_cohesion_applied` | Sailfish outside `radius_flee` → cohesion fires normally |
| `test_sardine_no_flee_without_sailfish_grid` | `sailfish_grid=None` → no flee force even if sailfish nearby |

---

### Flee force magnitude check

At `dt = 1/60` the per-frame velocity contribution of each force is `weight × max_force × dt`:

| Force | weight | contribution/frame (px/s) |
|---|---|---|
| Flee | 4.0 | 4.0 × 150 / 60 ≈ **10.0** |
| Separation | 2.0 | 2.0 × 150 / 60 ≈ **5.0** |
| Cohesion | 1.0 | 1.0 × 150 / 60 ≈ **2.5** |

Flee > separation ✓ (sardines always flee before maintaining personal space).  
Cohesion zeroed during flee ✓ (school can split).  
Alignment (1.3 × 150 / 60 ≈ 3.3 px/s) kept ✓ (sardines flee as a coordinated sub-group).

---

### Phase 2 tuning targets (empirical)

| Parameter | Starting value | Too low → | Too high → |
|---|---|---|---|
| `weight_flee` | 4.0 | sardines barely react | sardines teleport away |
| `radius_flee` | 100 | no split before contact | constant fleeing before sailfish is visible |
| `sailfish_max_speed` | 250 | can't catch sardines | sardines never escape |
| `radius_sailfish_awareness` | 200 | sailfish wanders aimlessly | locks onto sardines from across world |
| `num_sailfish` | 2 | one school never pressured | total chaos; sardines never regroup |


| Decision | Choice | Rationale |
|---|---|---|
| Sailfish targeting | Local sardine COM within `radius_sailfish_awareness` | Stable target; naturally creates herding behaviour; avoids jittery target-switching |
| Flee detection | Per-frame sailfish spatial hash query (stateless) | No per-boid flags; flee force fades automatically when sailfish exits `radius_flee` |
| Cohesion during flee | **Zeroed** when any sailfish detected within `radius_flee` | School splits naturally; sub-groups fleeing different directions diverge |
| Alignment during flee | **Kept at normal weight** | Sardines remain coordinated within each fleeing sub-group — biologically realistic |
| Regrouping when safe | **Automatic** — normal flocking resumes | Cohesion pulls scattered fish back together; no state to reset |
| Grid architecture | **Two separate SpatialHash instances** | `_grid` = sardines only; `_sailfish_grid` = sailfish only; no type-filtering needed |
| `Sardine` constructor | `sailfish_grid=None` optional param | Fully backward-compatible; all existing tests pass unchanged |
| Nearest-prey targeting | Deferred to Phase 3 (optional) | COM targeting is more stable for first pass; individual prey can be added with a flag |

---

## 11. Phase 4 Optimization Summary

Phase 4 is complete. All code changes verified with 53/53 tests passing.

### Bottleneck audit

| Location | Problem | Severity |
|---|---|---|
| `utils/vec2.normalize`, `limit` | `np.linalg.norm` for 2-D vector — 5–10× slower than scalar `math.sqrt` | Critical — called on every steering application per agent per frame |
| `spatial_hash._cell_key`, `_candidate_cells` | `np.floor` / `np.ceil` for a single Python `float` — numpy dispatch overhead | High — called for every `insert` and every candidate-cell loop |
| `spatial_hash.query` / `query_position` | `wrap_delta(...)` function call ×2 per candidate neighbour | High — ~90 000 calls/frame at 900 sardines |
| `sardine._compute_forces` | 3× `np.zeros(2)` per frame for sep/ali/coh accumulators; `ali_sum += other.velocity` allocates a numpy iadd temp array | High — 2 700+ array allocations/frame at default population |
| `sardine`/`sailfish._compute_forces` | `float(cfg.window_width)` etc. re-evaluated each call via config attribute chain | Medium — repeated attribute lookup per frame per agent |
| `simulation.update` | `_all_agents()` creates a merged list every frame | Low — one list allocation + copy per frame |
| `spatial_hash._cells` (was `defaultdict`) | Miss path calls `list` factory on every unknown-cell lookup; `dict.get(key, [])` creates a throwaway empty list per miss | Low–medium |

### Optimizations applied

| File | Change | Per-frame savings at 900 sardines |
|---|---|---|
| `utils/vec2.py` | `math.sqrt(v[0]*v[0]+v[1]*v[1])` in `normalize`, `limit`, `distance`, `distance_sq` | Eliminates numpy dispatch for ~2 700+ norm calls/frame |
| `spatial_hash.py` | `//` integer division and `math.floor`/`math.ceil` instead of numpy ops; plain `dict` instead of `defaultdict`; inlined `wrap_delta` in `query` and `query_position`; None-sentinel instead of empty-list default; `_half_w`/`_half_h` cached | ~90 000 fewer Python function calls; no np.floor dispatch; no factory allocation per miss |
| `boid.py` | `math.sqrt` in `speed` property and `_clamp_speed`; new `_steer_apply_xy(tx, ty, weight)` — inlines normalize+limit+accumulate into pure scalar math, writes directly to `self.acceleration[0/1]` | 2–4 fewer intermediate numpy arrays per steering application |
| `sardine.py` | Float-pair accumulators `sx/sy`, `alx/aly`, `cx/cy` replace 3× `np.zeros(2)`; `alx += float(other.velocity[0])` replaces numpy iadd; `_W/_H/_hw/_hh/_sep_r2/_ali_r2` cached at `__init__`; toroidal wrap inlined; all steer calls via `_steer_apply_xy` | ~2 700 `np.zeros(2)` + ~3 600 steer temp arrays eliminated/frame |
| `sailfish.py` | Float-pair COM accumulator `cx/cy`; cached dims; inlined wrap; `_steer_apply_xy` | ~8 `np.zeros(2)` + ~8 steer temp arrays eliminated/frame |
| `simulation.py` | `update()` iterates `self.sardines` then `self.sailfish` directly — no `_all_agents()` list merge | 1 list allocation/frame eliminated |

### Key new primitive: `Boid._steer_apply_xy`

```python
def _steer_apply_xy(self, tx: float, ty: float, weight: float) -> None:
    mag = math.sqrt(tx * tx + ty * ty)
    if mag < _EPS_F:
        return
    inv_mag = self.max_speed / mag
    sx = tx * inv_mag - float(self.velocity[0])
    sy = ty * inv_mag - float(self.velocity[1])
    fmag = math.sqrt(sx * sx + sy * sy)
    if fmag > self.max_force:
        inv_fmag = self.max_force / fmag
        sx *= inv_fmag
        sy *= inv_fmag
    self.acceleration[0] += sx * weight
    self.acceleration[1] += sy * weight
```

Replaces the old `steer_toward(desired) → force; apply_force(force * weight)` chain which created 2–4 temporary `np.ndarray` objects per call. All arithmetic is now Python-native floats; only the final `+=` touches numpy.

### Runtime population controls

| Key | Action |
|---|---|
| ↑ | Add 50 sardines (spawned at random positions/headings) |
| ↓ | Remove 50 sardines (from end of list) |
| `[` | Add 1 sailfish *(Phase 4 original; replaced in Phase 5)* |
| `]` | Remove 1 sailfish *(Phase 4 original; replaced in Phase 5)* |
| H | Toggle HUD overlay |
| ESC | Quit |

> **Note (Phase 5 update):** `[`/`]` bindings were replaced by `←`/`→` (LEFT/RIGHT arrow keys). `F` was added for fullscreen toggle. See Phase 5 summary below.

New `Simulation` API:

| Method | Behaviour |
|---|---|
| `add_sardines(n)` | Spawns `n` Sardines at uniformly random positions; appends to `self.sardines` |
| `remove_sardines(n)` | `del self.sardines[-n:]`; returns count actually removed |
| `add_sailfish(n)` | Spawns `n` Sailfish at random positions; appends to `self.sailfish` |
| `remove_sailfish(n)` | `del self.sailfish[-n:]` |

### Algorithmic complexity — unchanged

All Phase 4 changes are **constant-factor** improvements; the asymptotic complexity was already optimal:

| Operation | Complexity |
|---|---|
| Grid rebuild per frame | O(N) inserts |
| Neighbour query per agent | O(1) average — fixed-radius ring of ~9 cells regardless of N |
| Force computation per agent | O(k) where k = mean neighbours in radius |
| Full frame update | O(N × k) ≈ O(N) for constant density |

### Scaling limits (current architecture)

- **Python per-object loop**: the bottleneck is `for agent in self.sardines: agent.update(dt)`. At ~1 500 sardines the interpreter call overhead dominates and frame time grows above 16 ms on a typical laptop even though per-sardine work is O(1).
- **Next threshold**: beyond ~1 500 sardines, the right approach is numpy batched force computation — store all sardine positions in a single `(N, 2)` array and perform flocking math with numpy broadcasting. This eliminates per-object Python call overhead and moves the bottleneck to SIMD memory bandwidth.
- **Sailfish count**: each sailfish queries the sardine grid once per frame in O(k) time. At default counts (≤ 20 sailfish) their contribution is negligible.
- **Renderer**: `_sync_shapes` is O(N) pure-Python float math per frame; already optimised in Phase 3. At 1 500 agents this pass takes ~3–5 ms.
- **Spatial hash**: O(1) average per query; O(N) full rebuild (2 × N inserts/frame). With `//` and `math.floor` the rebuild is fast; the dominant cost is the Python loop, not the hash arithmetic.

---

## 12. Phase 5 Summary

### Changes made

#### Control remapping (sailfish count)

`[`/`]` (BRACKETLEFT / BRACKETRIGHT) replaced by `←`/`→` (LEFT / RIGHT arrow keys).

| Key | Action |
|---|---|
| ↑ | Add 50 sardines |
| ↓ | Remove 50 sardines |
| → | Add 1 sailfish |
| ← | Remove 1 sailfish |
| F | Toggle fullscreen |
| H | Toggle HUD overlay |
| ESC | Quit |

No conflict with sardine controls; arrow axes are orthogonal (vertical = fish count, horizontal = predator count).

#### Dynamic window resize

Resize is handled event-driven via pyglet's `on_resize(width, height)` callback.

Flow on resize / fullscreen toggle:

```
on_resize(w, h)
  └─ simulation.resize(w, h)
       ├─ dataclasses.replace(config, window_width=w, window_height=h)  → new_config
       ├─ propagate new_config to all agents (.config)
       ├─ update _W/_H/_hw/_hh on every Sardine and Sailfish
       ├─ SpatialHash.resize(w, h)  — both grids rebuilt in-place
       └─ positions %= (w, h)  — toroidal clamp to new bounds
  └─ renderer.on_resize(w, h)
       └─ Hud.reposition(h)   — moves label.y to h - 8
```

No `renderer.reset()` is called on resize because existing `pyglet.shapes.Triangle` objects remain valid; `_sync_shapes` already updates all six vertex coordinates every frame.

A minimum dimension guard (`width < 200 or height < 150`) is enforced before acting to avoid degenerate grid / rendering state.

A `_ready` flag (set `True` after all event handlers are registered and `renderer.reset()` has run) prevents any stray `on_resize` events during startup from corrupting uninitialised state.

#### Fullscreen toggle

`window.set_fullscreen(not window.fullscreen)` — pyglet 2.x API.  Automatically fires `on_resize` with the screen's full pixel resolution, which propagates through the resize pipeline above.  Exiting fullscreen (`F` again) fires `on_resize` with the restored windowed dimensions.

#### Files changed

| File | Change |
|---|---|
| `main.py` | `resizable=True`; `on_resize` handler; `F` key; LEFT/RIGHT replace BRACKETLEFT/BRACKETRIGHT; `_ready` guard |
| `simulation.py` | `resize(new_w, new_h)` method added |
| `spatial_hash.py` | `resize(world_width, world_height)` method added |
| `renderer.py` | `Hud.reposition(window_height)` added; `Renderer.on_resize(width, height)` added; HUD key-guide line updated |

### Platform notes (macOS)

- Fullscreen on macOS uses the native Spaces fullscreen mechanism via pyglet.  The menu bar disappears; the `on_resize` event fires with the full screen resolution (typically `screen.width × screen.height` or the full Retina backing-store size on HiDPI).
- Rapid window dragging fires `on_resize` repeatedly at high frequency.  Because `simulation.resize()` does not reallocate agents (only updates scalar attributes and clears the hash dict), each resize call is O(N) in agent count and cheap enough to run every frame without measurable overhead.
- The `_ready` guard is necessary on macOS because pyglet's Cocoa backend fires `on_resize` once during `Window.__init__` (before our handler is registered in some versions), and potentially again when the window is first shown.

---

## 13. Phase 6 Design — Runtime Config Menu & Config Persistence

**Status: Design complete. Implementation not yet started.**

---

### 13.1 Problem statement

All 29 tunable parameters in `SimConfig` are baked in at startup.  Experimenting currently requires editing `config.py`, restarting, and re-observing — a slow feedback loop.  The goal is an in-app overlay menu that lets the user change key parameters live while watching the effect in real time, and optionally persist those settings to disk.

---

### 13.2 Simulation continues while menu is open (no pause)

**Decision: the simulation keeps running while the menu is visible.**

Rationale: the only reason to open the menu is to see the effect of a parameter change.  Pausing defeats the purpose — you'd have to close the menu to see anything.  The overhead of rendering the overlay is negligible (a handful of labels and one filled rectangle).  The arrow-key conflict (population controls vs menu navigation) is resolved modally: when the menu is visible, arrow keys are captured exclusively by the menu and do not fire population changes.

---

### 13.3 Menu visual design

The menu is a translucent dark overlay panel anchored to the left edge of the window.

```
┌────────────────────────────────────────────┐
│  CONFIG  (TAB to close · S to save)        │
├────────────────────────────────────────────┤
│ > Sep weight        2.0   ←/→ to change   │
│   Ali weight        1.3                    │
│   Coh weight        1.0                    │
│   Noise weight      0.04                   │
│   Flee weight      10.0                    │
│   Pursuit weight    1.5                    │
│   Sep radius       40.0                    │
│   Ali radius       75.0                    │
│   Coh radius       80.0                    │
│   Flee radius     100.0                    │
│   Sardine speed   150.0                    │
└────────────────────────────────────────────┘
```

- `>` marks the selected (focused) row.
- ↑/↓ move the cursor between rows.
- ←/→ decrease/increase the value on the selected row by one `step`.
- Holding Shift + ←/→ applies a 10× coarse step.
- `S` saves current config to `boids_overrides.json` and flashes a brief confirmation in the menu header.
- `TAB` closes the menu (same key that opened it); the normal arrow-key population controls resume instantly.
- No confirm/cancel distinction — every change is applied immediately and live. "Cancel" means manually reverting the value.

**Rendering**: one `pyglet.graphics.Batch` per menu, drawn on top of the simulation batch in `Renderer.draw()`. The backdrop is a `pyglet.shapes.Rectangle` with `opacity=180` (≈70% alpha). Row labels and values are `pyglet.text.Label` instances, rebuilt only when the selection or a value changes (not every frame).

---

### 13.4 Menu navigation keybindings

| Key | Context | Action |
|---|---|---|
| `TAB` | Always | Toggle menu open/closed |
| `↑` | Menu open | Move cursor up one row |
| `↓` | Menu open | Move cursor down one row |
| `←` | Menu open | Decrease selected value by `step` |
| `→` | Menu open | Increase selected value by `step` |
| `Shift+←` | Menu open | Decrease by `10 × step` |
| `Shift+→` | Menu open | Increase by `10 × step` |
| `S` | Menu open | Save overrides to `boids_overrides.json` |
| `TAB` or `ESC` | Menu open | Close menu |
| `↑ ↓ ← →` | Menu closed | Existing population controls (unchanged) |

> `ESC` while the menu is open closes the menu rather than quitting, preventing accidental exits. `ESC` while the menu is closed still quits.

---

### 13.5 Editable parameters — first batch (11 items)

| Row label | `SimConfig` field | Step | Shift step | Min | Max | Constraint |
|---|---|---|---|---|---|---|
| Sep weight | `weight_separation` | 0.1 | 1.0 | 0.0 | 10.0 | — |
| Ali weight | `weight_alignment` | 0.1 | 1.0 | 0.0 | 10.0 | — |
| Coh weight | `weight_cohesion` | 0.1 | 1.0 | 0.0 | 10.0 | — |
| Noise weight | `weight_noise` | 0.01 | 0.1 | 0.0 | 0.5 | — |
| Flee weight | `weight_flee` | 0.5 | 5.0 | 0.0 | 30.0 | — |
| Pursuit weight | `weight_pursuit` | 0.1 | 1.0 | 0.0 | 10.0 | — |
| Sep radius | `radius_separation` | 2.0 | 20.0 | 5.0 | 100.0 | must be < `radius_alignment` |
| Ali radius | `radius_alignment` | 2.0 | 20.0 | 10.0 | 150.0 | must be ≤ `radius_cohesion` |
| Coh radius | `radius_cohesion` | 2.0 | 20.0 | 10.0 | 150.0 | capped at `cell_size` (80 px) to keep spatial hash valid |
| Flee radius | `radius_flee` | 5.0 | 50.0 | 20.0 | 300.0 | capped at `sailfish_cell_size` (120 px) |
| Sardine max speed | `sardine_max_speed` | 10.0 | 50.0 | 50.0 | 500.0 | must be ≥ `sardine_min_speed` |

**Not in first batch** (require respawn, grid rebuild, or renderer reset): `num_sardines`, `num_sailfish`, `cell_size`, `sailfish_cell_size`, `boundary`, `sardine_size`, `sailfish_size`. These already have keyboard shortcuts or require more invasive plumbing.

---

### 13.6 Live parameter propagation

`SimConfig` is a **frozen** dataclass.  Changing a parameter means creating a new `SimConfig` via `dataclasses.replace()` and propagating it to all agents — exactly the pattern already used in `simulation.resize()`.

New method: `Simulation.apply_config(new_config: SimConfig)`:

```python
def apply_config(self, new_config: SimConfig) -> None:
    self._config = new_config
    for agent in self.sardines:
        agent._config = new_config
        # refresh cached float attrs derived from config
        agent._sep_r2 = new_config.radius_separation ** 2
        agent._ali_r2 = new_config.radius_alignment ** 2
    for agent in self.sailfish:
        agent._config = new_config
```

`ConfigMenu` calls `simulation.apply_config(dataclasses.replace(current_config, **{field: new_val}))` whenever a value changes.  Cost: one `dataclasses.replace()` + O(N) attribute writes — cheap even at 1000 agents; runs at most a few times per second (key held down).

---

### 13.7 Config persistence design

#### Files involved

| File | Role |
|---|---|
| `config.py` | Unchanged.  Default values are the canonical baseline. |
| `config_io.py` | **New file.** Pure I/O module — `load_overrides()`, `save_config()`, `make_config()`. |
| `boids_overrides.json` | **New file (auto-created on first save).** Partial override dict — only stores non-default values. Added to `.gitignore`. |

#### `config_io.py` interface

```python
import json, pathlib, dataclasses
from config import SimConfig

OVERRIDE_FILE = pathlib.Path(__file__).parent / "boids_overrides.json"

def load_overrides() -> dict:
    """Return saved override dict, or {} if no file exists or file is invalid."""
    if not OVERRIDE_FILE.exists():
        return {}
    try:
        return json.loads(OVERRIDE_FILE.read_text())
    except (json.JSONDecodeError, OSError):
        return {}  # corrupt / unreadable file → silently use defaults

def save_config(config: SimConfig) -> None:
    """Serialise all SimConfig fields to boids_overrides.json."""
    OVERRIDE_FILE.write_text(json.dumps(dataclasses.asdict(config), indent=2))

def make_config(**overrides) -> SimConfig:
    """Construct a SimConfig, applying caller-supplied overrides."""
    return SimConfig(**overrides)
```

#### Startup flow (`main.py`)

```python
from config_io import load_overrides

overrides = load_overrides()                    # {} on first run
config = SimConfig(
    window_width=window.width,
    window_height=window.height,
    **overrides,                                # apply saved prefs over defaults
)
```

Only `window_width` and `window_height` are always set from the live screen; all other saved fields are applied as-is.  If the override file contains a key that is no longer in `SimConfig` (e.g. after a rename), the `SimConfig(**overrides)` call will raise `TypeError`.  `load_overrides()` should strip unknown keys before returning to be safe — add an `{k: v for k, v in d.items() if hasattr(SimConfig, k)}` filter.

#### Save trigger

Pressing `S` while the menu is open calls `save_config(current_config)` and updates the menu header text to `"CONFIG  ✓ saved"` for 1.5 s.  No auto-save on quit.

---

### 13.8 New file: `config_menu.py`

A standalone class `ConfigMenu` with no simulation or rendering imports from the app. It receives references at construction:

```python
class ConfigMenu:
    def __init__(self, window: pyglet.window.Window, simulation: Simulation) -> None:
        ...

    # State
    visible: bool = False
    _selected: int = 0        # index into self._rows

    # Methods
    def toggle(self) -> None: ...
    def handle_key(self, symbol: int, modifiers: int) -> bool:
        """Process key; returns True if consumed (caller should not propagate)."""
        ...
    def draw(self) -> None:
        """Draw overlay on top of simulation. Call after Renderer.draw()."""
        ...
```

`_rows` is a list of `_MenuItem` named-tuples or dataclass instances:

```python
@dataclasses.dataclass
class _MenuItem:
    label: str
    field: str          # SimConfig attribute name
    step: float
    coarse: float       # 10× step
    min_val: float
    max_val: float
```

`draw()` reconstructs label text only when `_dirty` is True (set on cursor move or value change); otherwise it just calls `self._batch.draw()`.

---

### 13.9 Integration into `main.py` and `renderer.py`

**`main.py` changes (planned)**:
- Import `ConfigMenu` and `load_overrides` from their modules.
- Apply `load_overrides()` when constructing `SimConfig`.
- Instantiate `ConfigMenu(window, simulation)` after renderer is ready.
- In `on_key_press`: route `TAB` to `menu.toggle()`; route all other keys through `menu.handle_key()` first — if it returns `True` (consumed), skip existing handlers.
- Existing population controls (↑/↓/←/→) are untouched in code; the modal guard in `menu.handle_key()` returning `True` prevents them from firing when the menu is open.

**`renderer.py` changes (planned)**:
- `Renderer.draw()` gains an optional `menu: ConfigMenu | None = None` parameter (or the menu draws itself and `main.py` calls `menu.draw()` separately after `renderer.draw()`).  Separate call is cleaner — no renderer → menu coupling.

---

### 13.10 Implementation order

| Step | File(s) | What | Status |
|---|---|---|---|
| 1 | `config_io.py` | `load_overrides()`, `save_config()` | ✅ **Done (Phase 6a)** |
| 2 | `main.py` | Apply `load_overrides()` at startup | ✅ **Done (Phase 6a)** |
| 3 | `simulation.py` | `apply_config(new_config)` method | ✅ **Done (Phase 6c)** |
| 4 | `config_menu.py` | `ConfigMenu` class, draw logic, key handler | ✅ **Done (Phase 6b)** |
| 5 | `main.py` | Wire `TAB` + modal key routing + `menu.draw()` call | ✅ **Done (Phase 6b)** |
| 6 | `main.py` | `save_config()` on `S` key inside menu | ✅ **Done (Phase 6c)** |
| 7 | `tests.py` | Tests for `config_io` and `apply_config` | Not started |

---

### 13.11 Key binding summary (after Phase 6)

| Key | Context | Action |
|---|---|---|
| `TAB` | Always | Toggle config menu |
| `↑` | Menu closed | Add 50 sardines |
| `↓` | Menu closed | Remove 50 sardines |
| `→` | Menu closed | Add 1 sailfish |
| `←` | Menu closed | Remove 1 sailfish |
| `↑` | Menu open | Move cursor up |
| `↓` | Menu open | Move cursor down |
| `←` | Menu open | Decrease value (1× step) |
| `→` | Menu open | Increase value (1× step) |
| `Shift+←` | Menu open | Decrease value (10× step) |
| `Shift+→` | Menu open | Increase value (10× step) |
| `S` | Menu open | Save config to `boids_overrides.json` |
| `ESC` | Menu open | Close menu |
| `ESC` | Menu closed | Quit |
| `F` | Always | Toggle fullscreen |
| `H` | Always | Toggle HUD |

---

## 14. Phase 6a — Config Persistence (completed)

**Status: Done. 53/53 tests still passing.**

### What was implemented

#### New file: `config_io.py`

Pure I/O module — no rendering, simulation, or UI imports.  Two public functions:

| Function | Signature | Description |
|---|---|---|
| `load_overrides()` | `() -> dict[str, Any]` | Read `boids_overrides.json`; return validated, splat-safe dict |
| `save_config(config)` | `(SimConfig) -> None` | Atomically write all SimConfig fields to `boids_overrides.json` |

Module-level constant: `OVERRIDE_FILE = pathlib.Path(__file__).parent / "boids_overrides.json"`

Internal helpers (private):
- `_type_hints()` — calls `typing.get_type_hints(SimConfig)` once per use; returns `{field: type}`.
- `_coerce(field_name, value, hint)` — type-coerces a raw JSON value to the annotated SimConfig type; raises `ValueError` / `TypeError` on mismatch so the caller can skip the entry.

#### `load_overrides()` behaviour

| Situation | Behaviour |
|---|---|
| File absent | Return `{}` |
| File unreadable / corrupt JSON | Return `{}` |
| Top-level JSON is not an object | Return `{}` |
| Unknown field name | Skip that key silently |
| `window_width` or `window_height` | Always skipped (`_RUNTIME_FIELDS`); set from the live window at startup |
| Type-incompatible value | Skip that key; code default is used for that field |
| Valid entry | Coerced to the correct Python type and included |

Type coercion rules:
- `float` fields: accept `int` or `float` from JSON (JSON makes no type distinction).
- `int` fields: accept whole-number floats (e.g. `40.0 → 40`); reject non-integer floats.
- `bool` fields: must be a JSON boolean; `0`/`1` integers are rejected.
- `tuple[int,int,int,int]` (RGBA colour fields): accept a 4-element JSON array; each element coerced to `int`.
- `str` fields: must be a JSON string.

#### `save_config()` behaviour

- Serialises `dataclasses.asdict(config)` to a 2-space-indented JSON file.
- Colour tuples are serialised as JSON arrays (`[R, G, B, A]`).
- Uses an atomic write pattern: writes to `boids_overrides.json.tmp`, then `Path.replace()` renames it over the target.  On macOS / POSIX this is an atomic filesystem operation — a crash mid-write never corrupts the file.
- `window_width` and `window_height` **are** included in the saved file as a snapshot but are silently skipped on the next `load_overrides()` call.

#### Updated: `main.py`

- Added `from config_io import load_overrides`.
- `SimConfig(...)` construction now uses `**load_overrides()` to layer saved preferences over code defaults:

  ```python
  config = SimConfig(
      window_width=window.width,
      window_height=window.height,
      **load_overrides(),   # saved user prefs layered over code defaults
  )
  ```
- First-run behaviour (no file): `load_overrides()` returns `{}`, so `SimConfig` uses all code defaults.
- Subsequent runs: saved fields are applied; `window_width` / `window_height` are always overridden by the live screen.

#### New file: `.gitignore`

Created with standard Python entries plus:
```
boids_overrides.json
boids_overrides.json.tmp
```

### Known limitations / future work

- `save_config()` writes **all** fields, including values that match the code defaults.  This makes the file a full snapshot (easy to read and edit), at the cost of a noisier diff when only one value changed.  A "diff-only" save (omit fields equal to default) could be added later.
- `load_overrides()` does not validate **ranges** (e.g. it will happily load `radius_cohesion = 999.0`).  Range clamping can be added to `_coerce()` once the menu is built and per-field step/min/max constants are defined there — no need to duplicate the table in two places.
- There is no user-visible feedback for a corrupt/missing file at startup; it silently runs with defaults.  Once the HUD + menu exist, a brief "defaults loaded" vs "overrides applied" message could be added to the HUD.
- The override file is not yet wired to the `S` key in-game — that requires the menu (Phase 6b/c).

### Next recommended step

Implement `Simulation.apply_config(new_config: SimConfig)` — the live-propagation method that the menu will call on every value change.  This is a ~15-line addition to `simulation.py` following the existing `resize()` pattern, has no UI dependency, and can be tested headlessly.  After that, `config_menu.py` can be built on top of an already-working propagation layer.

---

## 15. Phase 6b — Config Menu Shell (completed)

**Status: Done. 53/53 tests still passing.**

### What was implemented

#### New file: `config_menu.py`

Standalone overlay class.  No imports from `simulation.py`, `renderer.py`, or `main.py`; it receives live references at construction and reads `simulation.config` to display values.

| Symbol | Role |
|---|---|
| `ConfigMenu(window, simulation)` | Constructor — creates all shapes and labels for the initial window size |
| `toggle()` | Open if closed; close if open; marks labels dirty so values refresh on first draw |
| `handle_key(symbol, modifiers) → bool` | Modal key handler — returns `True` if consumed (caller skips remaining handlers) |
| `on_resize(width, height)` | Rebuilds all shapes and labels for the new window size |
| `draw()` | No-op when not visible; calls `_refresh_rows()` if dirty, then draws all pyglet objects |

**Key-consumption table (inside `handle_key`)**:

| Key | Action | Returns |
|---|---|---|
| ↑ | Move cursor up (wraps) | True |
| ↓ | Move cursor down (wraps) | True |
| ← | (placeholder — reserved for value editing) | True |
| → | (placeholder — reserved for value editing) | True |
| ESC or TAB | Close menu | True |
| All other keys | Not consumed | False |

**Visual layout (left panel, top-anchored)**:
```
┌─ 305 px wide, anchored 20 px from top-left ──────────────────┐
│ CONFIG   ↑↓ navigate   TAB close          ← dark header strip │
├───────────────────────────────────────────────────────────────┤
│   Sep weight            2.0                                    │
│   Ali weight            1.3                                    │
│ ▌ Coh weight          [ 1.0 ]  ← blue highlight bar on cursor │
│   ...                                                          │
└───────────────────────────────────────────────────────────────┘
```
- Panel bg: dark navy `(10, 18, 35)` at 73 % opacity.
- Header strip: slightly lighter `(15, 28, 55)` at 84 % opacity.
- Cursor highlight bar: `(45, 100, 195)` at 39 % opacity — subtle blue wash under selected row.
- Selected row text: bright white-blue `(215, 235, 255, 255)`.
- Unselected row text: muted grey-blue `(115, 145, 175, 185)`.
- Font: Menlo → Consolas → Courier New, 13 pt.
- Row format: `"  {label:<16}{value:>8}"` — fixed monospace columns.

**Resize / fullscreen**: `_build(w, h)` is called from both `__init__` and `on_resize`.  It runs `_delete_all()` first so all prior pyglet objects are released before new ones are created; no leaks on repeated resize.

**Simulation runs while open**: no clock pausing, no guard in `update()`.  The modal guard in `handle_key` is purely about keyboard routing.

#### Updated: `main.py`

| Change | Detail |
|---|---|
| `from config_menu import ConfigMenu` | New import |
| `menu = ConfigMenu(window, simulation)` | Instantiated after renderer is ready |
| `on_key_press` | `menu.handle_key(symbol, modifiers)` called first; if it returns `True`, handler returns immediately.  Explicit `if symbol == key.TAB: menu.toggle()` follows for the open case. |
| `on_draw` | `menu.draw()` called after `renderer.draw()` — always safe (no-op when not visible) |
| `on_resize` | `menu.on_resize(width, height)` added after `renderer.on_resize()` |

**ESC behaviour (new)**:
- Menu open → closes menu (handle_key returns True, `window.close()` never reached)
- Menu closed → quits as before

#### Updated: `renderer.py`

HUD key guide line updated:
```
↑/↓ sardines (+/-50)  ←/→ sailfish (+/-1)  F fullscreen  H HUD  TAB config
```

### Current limitations

- **No value editing yet**: ← / → are consumed (to prevent population controls firing) but do nothing. Editing requires `Simulation.apply_config()` first.
- **No `S` key save inside menu**: `save_config()` is not yet wired to a key inside the menu.
- **No visual indicator for ← / →**: the header text mentions `↑↓ navigate` but not `←/→ edit` since editing is not active.
- **Labels rebuild on `_build()` only read config once**: if the simulation config changes externally between opens (not currently possible but will be once editing is added), call `menu._dirty = True` to force a refresh.

### Next recommended step

Implement `Simulation.apply_config(new_config: SimConfig)` in `simulation.py`, then wire live value editing into `ConfigMenu` — add step/min/max to each `_ROWS` entry, handle ← / → to call `apply_config`, and wire `S` to `save_config`.  This completes the full editing round-trip.

---

## 16. Phase 6c — Menu Navigation, Editing & Save (completed)

**Status: Done. 53/53 tests still passing.**

### What was implemented

#### Updated: `simulation.py`

New method `apply_config(new_config: SimConfig)`:

```python
def apply_config(self, new_config: SimConfig) -> None:
    self.config = new_config
    for agent in self.sardines:
        agent._config    = new_config
        agent._sep_r2    = new_config.radius_separation ** 2
        agent._ali_r2    = new_config.radius_alignment  ** 2
        agent.max_speed  = new_config.sardine_max_speed
        agent.max_force  = new_config.sardine_max_force
        agent.min_speed  = new_config.sardine_min_speed
    for agent in self.sailfish:
        agent._config = new_config
```

Follows the same pattern as `resize()` but does not change world geometry or reallocate agents. Called on every ← / → key press in the menu. Cost: O(N) attribute writes — cheap even at 1 000 agents; typically runs a few times per second at most while the user is holding a key.  Does **not** update `_W`/`_H`/`_hw`/`_hh` (those are geometry-only; handled by `resize()`).

**Kinematic propagation**: `max_speed`, `max_force`, and `min_speed` are instance attributes on `Boid` read directly inside `_steer_apply_xy()` and `_clamp_speed()` every frame. Updating them in `apply_config()` means changes to `sardine_max_speed` / `sardine_max_force` / `sardine_min_speed` take effect on the very next physics tick — agents immediately become faster/slower or more/less agile.

#### Updated: `config_menu.py`

**`_MenuItem` dataclass (new)**:
```python
@dataclasses.dataclass(frozen=True)
class _MenuItem:
    label:   str    # display label (max 16 chars)
    field:   str    # SimConfig attribute name
    step:    float  # fine step for ← / →
    coarse:  float  # Shift step (typically 10× fine)
    min_val: float  # inclusive lower bound
    max_val: float  # inclusive upper bound
```

**`_ROWS` converted** from `list[tuple[str,str]]` to `list[_MenuItem]` with full step/range metadata:

| Label | Field | Step | Shift | Min | Max |
|---|---|---|---|---|---|
| Sep weight | `weight_separation` | 0.1 | 1.0 | 0.0 | 10.0 |
| Ali weight | `weight_alignment` | 0.1 | 1.0 | 0.0 | 10.0 |
| Coh weight | `weight_cohesion` | 0.1 | 1.0 | 0.0 | 10.0 |
| Noise weight | `weight_noise` | 0.01 | 0.1 | 0.0 | 0.5 |
| Flee weight | `weight_flee` | 0.5 | 5.0 | 0.0 | 30.0 |
| Pursuit wt | `weight_pursuit` | 0.1 | 1.0 | 0.0 | 10.0 |
| Sep radius | `radius_separation` | 2.0 | 20.0 | 5.0 | 100.0 |
| Ali radius | `radius_alignment` | 2.0 | 20.0 | 10.0 | 150.0 |
| Coh radius | `radius_cohesion` | 2.0 | 20.0 | 10.0 | 150.0 |
| Flee radius | `radius_flee` | 5.0 | 50.0 | 20.0 | 300.0 |
| Sardine speed | `sardine_max_speed` | 10.0 | 50.0 | 50.0 | 500.0 |
| Sardine force | `sardine_max_force` | 10.0 | 50.0 | 10.0 | 500.0 |

**New `_row_text(item, cfg, is_selected)` signature**: selected row shows `▶` prefix; unselected rows show `"  "`. Same fixed-width column layout.

**New `_footer_text(item)`**: one-line hint showing the step/range for the currently focused row, e.g.:  
`  step 0.10  ·  Shift ×1.0  ·  [0.0–10.0]`

**New layout element — footer strip**: `_FOOTER_H = 22 px` strip at the bottom of the panel with darker background and the footer label drawn over it. Panel height increased by `_FOOTER_H`.

**Header text updated** to show all keys: `CONFIG   ↑↓ move  ←→ edit  S save  TAB`

**Save feedback**: pressing `S` calls `save_config(simulation.config)` then sets `_save_until = time.perf_counter() + 1.5`. While active, header text changes to `CONFIG   ✓ saved` in green. When the timer expires (checked in `draw()` each frame), `_dirty = True` restores normal header text automatically.

**`_adjust_value(symbol, modifiers)`** (new private method):
1. Reads `cur = float(getattr(cfg, item.field))`
2. `step = item.coarse if (modifiers & Key.MOD_SHIFT) else item.step`
3. `delta = +step (RIGHT) or −step (LEFT)`
4. `new_val = round(max(min_val, min(max_val, cur + delta)), 6)` — range-clamped, rounded to 6 dp to prevent float accumulation drift
5. If `new_val == cur` (at boundary), returns early — no spurious `apply_config` calls
6. Calls `simulation.apply_config(dataclasses.replace(cfg, **{item.field: new_val}))`

**`_save()` method** (new): calls `save_config()`; sets timer on success; silently ignores `OSError`.

### Full keyboard interaction model

| Key | Context | Action |
|---|---|---|
| `TAB` | Menu closed | Open menu |
| `TAB` | Menu open | Close menu |
| `ESC` | Menu open | Close menu |
| `ESC` | Menu closed | Quit app |
| `↑` | Menu open | Move cursor up (wraps) |
| `↓` | Menu open | Move cursor down (wraps) |
| `←` | Menu open | Decrease value by fine step |
| `→` | Menu open | Increase value by fine step |
| `Shift+←` | Menu open | Decrease value by coarse step (10×) |
| `Shift+→` | Menu open | Increase value by coarse step (10×) |
| `S` | Menu open | Save config → `boids_overrides.json`; 1.5 s ✓ feedback |
| `H` | Any | Toggle HUD (falls through menu handler) |
| `F` | Any | Toggle fullscreen (falls through menu handler) |
| `↑/↓` | Menu closed | Add/remove 50 sardines |
| `←/→` | Menu closed | Remove/add 1 sailfish |

### Selection / highlight model

| Layer | Visual | Trigger |
|---|---|---|
| Blue rectangle | `(45, 100, 195)` at 39 % opacity under selected row | Always tracks cursor |
| Row text color | Bright white-blue `(215, 235, 255)` | Selected row |
| Row text color | Muted grey-blue `(115, 145, 175)` | Unselected rows |
| `▶` prefix | Unicode ▶ (U+25B6) at start of selected row text | Selected row |
| Footer strip | Darker blue-grey; shows step + range for the current row | Updates on cursor move |

### Current limitations

- Cross-field constraints not yet enforced (e.g. `radius_separation < radius_alignment ≤ radius_cohesion`). Values can currently be set outside these relationships; agents will still run but behaviour may be odd at extremes.
- No undo / reset-to-default. Revert by closing without saving, or editing `boids_overrides.json` manually.
- Radius changes take effect immediately on the next `_compute_forces()` call but the spatial hash `cell_size` is not changed (it would require `SpatialHash.resize()`). If `radius_cohesion` is raised above `cell_size=80`, behaviour degrades gracefully (neighbours beyond 80 px won't be found) rather than crashing.
- Sailfish kinematics (`sailfish_max_speed`, `sailfish_max_force`, `sailfish_min_speed`) are not yet in the menu. `apply_config()` only propagates the config reference to sailfish, not their `Boid` instance attrs. Intentional — predator params not yet exposed.

### Next recommended step

Add tests for `apply_config` and `config_io` to `tests.py` (`TestConfigPersistence`, `TestApplyConfig` suites).

---

## 17. Phase 6d — First Batch of Editable Runtime Config Values (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Ensure that every parameter exposed in the config menu *actually* affects the live simulation — and add `sardine_max_force` (previously missing) so the full sardine kinematic triple is available.

### Changes

#### `simulation.py` — `apply_config()` kinematic propagation

Extended the per-sardine loop to also update the three `Boid` instance attributes that govern physics:

```python
agent.max_speed  = new_config.sardine_max_speed   # read by _steer_apply_xy
agent.max_force  = new_config.sardine_max_force   # read by _steer_apply_xy
agent.min_speed  = new_config.sardine_min_speed   # read by _clamp_speed
```

Previously only `_sep_r2` / `_ali_r2` were refreshed; `max_speed`, `max_force`, and `min_speed` were set once at construction and never updated, so changing them in the menu had no observable effect on already-spawned agents.

**Why this works**: `_steer_apply_xy()` references `self.max_speed` and `self.max_force` directly at each call site (Python attribute lookup, no caching). `_clamp_speed()` reads `self.max_speed` and `self.min_speed` every frame. Updating the instance attribute therefore takes effect on the very next physics tick, with no further infrastructure needed.

#### `config_menu.py` — added `sardine_max_force` row

```python
_MenuItem("Sardine force", "sardine_max_force", 10.0, 50.0, 10.0, 500.0),
```

Inserted immediately after the `Sardine speed` row. Panel height auto-adjusts because `_panel_h()` is `_HEADER_H + _PAD_V + _NUM_ROWS * _ROW_H + _PAD_V + _FOOTER_H` and `_NUM_ROWS = len(_ROWS)` is computed at import time.

### Editable parameters — full current list

| Menu label | Config field | Propagates to | Notes |
|---|---|---|---|
| Sep weight | `weight_separation` | `agent._config` | Read from config in `_compute_forces` each frame |
| Ali weight | `weight_alignment` | `agent._config` | ↑ |
| Coh weight | `weight_cohesion` | `agent._config` | ↑ |
| Noise weight | `weight_noise` | `agent._config` | ↑ |
| Flee weight | `weight_flee` | `agent._config` | ↑ |
| Pursuit wt | `weight_pursuit` | `agent._config` | ↑ (sailfish only) |
| Sep radius | `radius_separation` | `agent._config` + `agent._sep_r2` | Squared cache refreshed |
| Ali radius | `radius_alignment` | `agent._config` + `agent._ali_r2` | Squared cache refreshed |
| Coh radius | `radius_cohesion` | `agent._config` | Read from config each frame |
| Flee radius | `radius_flee` | `agent._config` | ↑ |
| Sardine speed | `sardine_max_speed` | `agent._config` + `agent.max_speed` | Boid instance attr updated |
| Sardine force | `sardine_max_force` | `agent._config` + `agent.max_force` | Boid instance attr updated |

### Remaining limitations

- **`sardine_min_speed`** is propagated in `apply_config()` but has no menu row yet (not requested in Phase 6d). Add a `_MenuItem` row if needed.
- **Sailfish kinematics** (`sailfish_max_speed`, `sailfish_max_force`, `sailfish_min_speed`) are propagated as `_config` only. Their `Boid` instance attrs (`max_speed`, `max_force`, `min_speed`) are not yet updated by `apply_config()` because the menu does not expose those rows. If sailfish rows are added in future, extend the sailfish loop the same way as sardines.
- **Cross-field constraints** (`radius_separation < radius_alignment ≤ radius_cohesion`) still not enforced at edit time.
- **Spatial hash `cell_size`** is not updated when radii exceed it; behaviour degrades gracefully.

### Next recommended step

Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py`.

---

## 18. Phase 6e — Predator / Sailfish Parameters in Menu (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Expose sailfish-specific parameters in the config menu so predator behaviour can be tuned at runtime, and reorganise the `_ROWS` list into logical groups.

### Changes

#### `simulation.py` — extend `apply_config()` sailfish loop

Added kinematic attribute propagation for sailfish (same pattern as sardines, Phase 6d):

```python
for agent in self.sailfish:
    agent._config   = new_config
    agent.max_speed = new_config.sailfish_max_speed   # _steer_apply_xy
    agent.max_force = new_config.sailfish_max_force   # _steer_apply_xy
    agent.min_speed = new_config.sailfish_min_speed   # _clamp_speed
```

`radius_sailfish_awareness` does not require a separate cache — `_compute_forces()` reads it as `cfg.radius_sailfish_awareness` on every frame, so propagating `_config` is sufficient.

#### `config_menu.py` — new rows + logical grouping

`_ROWS` reorganised into four named groups (inline comments) and three new rows added:

| Group | Label | Field | Step | Shift | Min | Max |
|---|---|---|---|---|---|---|
| Sardine flocking weights | Sep weight | `weight_separation` | 0.1 | 1.0 | 0.0 | 10.0 |
| | Ali weight | `weight_alignment` | 0.1 | 1.0 | 0.0 | 10.0 |
| | Coh weight | `weight_cohesion` | 0.1 | 1.0 | 0.0 | 10.0 |
| | Noise weight | `weight_noise` | 0.01 | 0.1 | 0.0 | 0.5 |
| Sardine kinematics | Sardine speed | `sardine_max_speed` | 10.0 | 50.0 | 50.0 | 500.0 |
| | Sardine force | `sardine_max_force` | 10.0 | 50.0 | 10.0 | 500.0 |
| Sardine perception radii | Sep radius | `radius_separation` | 2.0 | 20.0 | 5.0 | 100.0 |
| | Ali radius | `radius_alignment` | 2.0 | 20.0 | 10.0 | 150.0 |
| | Coh radius | `radius_cohesion` | 2.0 | 20.0 | 10.0 | 150.0 |
| Predator avoidance (sardine-side) | Flee weight | `weight_flee` | 0.5 | 5.0 | 0.0 | 30.0 |
| | Flee radius | `radius_flee` | 5.0 | 50.0 | 20.0 | 300.0 |
| Sailfish / predator | Pursuit wt | `weight_pursuit` | 0.1 | 1.0 | 0.0 | 10.0 |
| *(new)* | Pursuit radius | `radius_sailfish_awareness` | 5.0 | 50.0 | 50.0 | 400.0 |
| *(new)* | Sailfish speed | `sailfish_max_speed` | 10.0 | 50.0 | 50.0 | 600.0 |
| *(new)* | Sailfish force | `sailfish_max_force` | 10.0 | 50.0 | 10.0 | 600.0 |

Total: 15 rows.  Panel height = 28 + 8 + 15×22 + 8 + 22 = **396 px**.  Fits on all target screens without scrolling.

### Propagation details — full current table

| Menu label | Config field | Target agents | How it propagates |
|---|---|---|---|
| Sep weight | `weight_separation` | sardines | `_config` ref; read each frame in `_compute_forces` |
| Ali weight | `weight_alignment` | sardines | ↑ |
| Coh weight | `weight_cohesion` | sardines | ↑ |
| Noise weight | `weight_noise` | sardines | ↑ |
| Sardine speed | `sardine_max_speed` | sardines | `_config` + `Boid.max_speed` instance attr |
| Sardine force | `sardine_max_force` | sardines | `_config` + `Boid.max_force` instance attr |
| Sep radius | `radius_separation` | sardines | `_config` + `_sep_r2` squared cache |
| Ali radius | `radius_alignment` | sardines | `_config` + `_ali_r2` squared cache |
| Coh radius | `radius_cohesion` | sardines | `_config`; read each frame |
| Flee weight | `weight_flee` | sardines | `_config`; read each frame |
| Flee radius | `radius_flee` | sardines | `_config`; read each frame |
| Pursuit wt | `weight_pursuit` | sailfish | `_config`; read each frame |
| Pursuit radius | `radius_sailfish_awareness` | sailfish | `_config`; read each frame (no caching needed) |
| Sailfish speed | `sailfish_max_speed` | sailfish | `_config` + `Boid.max_speed` instance attr |
| Sailfish force | `sailfish_max_force` | sailfish | `_config` + `Boid.max_force` instance attr |

### Remaining gaps before persistence UI is complete

| Gap | Notes |
|---|---|
| `sardine_min_speed` / `sailfish_min_speed` | Both propagated in `apply_config()` but no menu rows. Add when needed. |
| Cross-field radius constraints | `radius_separation < radius_alignment ≤ radius_cohesion` not enforced at edit time; agents degrade gracefully. |
| Spatial hash `cell_size` | Not updated when radii change; neighbours beyond `cell_size=80` px go undetected. Would require `SpatialHash.resize()`. |
| No undo / reset-to-defaults | Revert manually by deleting `boids_overrides.json` or pressing `S` after restoring values. |
| Tests | `TestConfigPersistence` and `TestApplyConfig` suites not yet written (Step 7 in Phase 6 plan). |

### Next recommended step

Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py` (the last unchecked item in the Phase 6 implementation plan).

---

## 19. Phase 6f — Visible Save Menu Item (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Make Save discoverable through the menu cursor, not just a memorised keyboard shortcut.  Previously `S` was the only way; users who open the menu for the first time have no signal that Save exists.

### Changes

#### `config_menu.py` — `_SaveRow` dataclass + 16th menu row

**New type `_SaveRow`** (frozen dataclass, alongside `_MenuItem`):
```python
@dataclasses.dataclass(frozen=True)
class _SaveRow:
    """A non-editable menu row that activates the save-to-disk action."""
    label: str = "Save to file"
```

**`_ROWS` type updated** from `list[_MenuItem]` to `list[_MenuItem | _SaveRow]`. A single `_SaveRow()` is appended as the 16th row under a `# ── Actions` comment group.

**`_NUM_ROWS = len(_ROWS) = 16`** — panel height auto-adjusts (= `28 + 8 + 16×22 + 8 + 22 = 418 px`).

**`_row_text`** extended:
- New signature: `_row_text(item, cfg, is_selected, save_state="") -> str`
- For `_SaveRow`: value column shows the current save state —
  - `""` (idle) → `[S/Ret]` (7 chars, right-aligned to 8)
  - `"saved"` → `✓ saved` (green when combined with label color)
  - `"error"` → `✗ error` (red)

**`_footer_text`** extended:
- For `_SaveRow`: returns `"  ↵ Enter  ·  ←→  ·  writes: boids_overrides.json"` — shows activation keys and the destination file name.

**New color** `_C_HDR_ERR = (240, 90, 70, 255)` — red, used for write-error feedback in both header and Save row label.

**New constant** `_SAVE_ERR_SECS = 2.5` — how long the error state is shown (slightly longer than the success flash to ensure the user notices).

**New instance attr** `_save_error_until: float = 0.0` — perf_counter deadline for error state, parallel to `_save_until`.

**New method `_save_row_state() -> str`**:
```python
def _save_row_state(self) -> str:
    now = time.perf_counter()
    if self._save_until       and now < self._save_until:       return "saved"
    if self._save_error_until and now < self._save_error_until: return "error"
    return ""
```
Called by `_hdr_text`, `_build`, and `_refresh_rows` — single source of truth for the three-state save display.

**`_hdr_text`** updated to show three states:
- `"CONFIG   ✓ saved"` (green, 1.5 s)
- `"CONFIG   ✗ write error"` (red, 2.5 s)
- normal key-guide (idle)

**`handle_key`** changes:
- `←` / `→` on the Save row now call `_save()` instead of `_adjust_value()`
- New `ENTER` / `RETURN` branch: triggers `_save()` when the Save row is selected; returns `False` (not consumed) on all other rows

**`_save()`** error path filled in:
```python
except OSError:
    self._save_error_until = time.perf_counter() + _SAVE_ERR_SECS
    self._save_until       = 0.0
```
Previously `OSError` was silently swallowed with no feedback.

**`draw()`** extended to expire `_save_error_until` as well as `_save_until`.

### How Save works end to end

1. User navigates to the `Save to file` row with ↑/↓ (or presses `S` from any row).
2. Row shows `[S/Ret]` idle hint; footer shows `↵ Enter  ·  ←→  ·  writes: boids_overrides.json`.
3. Activating (Enter, ←, →, or `S`) calls `_save()`:
   - Calls `save_config(simulation.config)` from `config_io.py`.
   - On success: `_save_until` set; header turns green `CONFIG   ✓ saved`; row value turns green `✓ saved` for 1.5 s.
   - On `OSError`: `_save_error_until` set; header turns red `CONFIG   ✗ write error`; row value turns red `✗ error` for 2.5 s.
4. Feedback expires naturally in `draw()` each frame.

### Save file details

| Property | Value |
|---|---|
| File | `boids_overrides.json` (same directory as `main.py`) |
| Written by | `save_config()` in `config_io.py` — atomic write (tmp → rename) |
| Created if missing | Yes — `open(..., "w")` creates the file; no pre-existence required |
| On write error | Red `✗ write error` in header + `✗ error` in Save row for 2.5 s; simulation unaffected |
| Format | Full JSON dict of all `SimConfig` fields (including `window_width`/`height`); `window_*` are skipped on next `load_overrides()` |

### Known edge cases

- If the disk is full, `OSError` is raised and the error feedback shows. The previous `boids_overrides.json` is unchanged (atomic write uses a `.tmp` file, which would also fail to write, so the original is never touched).
- If `boids_overrides.json` is open in another process (Windows file-lock), write will raise `PermissionError` (subclass of `OSError`) — caught and shown as error feedback.
- No auto-save on quit. If the user closes the window without saving after editing, changes are lost (as designed).

### Next recommended step

Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py`.


## 20. Phase 6g — Startup Config Loading Verification + Population-Count Fix (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Verify that saved menu changes (via `boids_overrides.json`) are correctly loaded and applied at startup, so settings persist across runs.  Also identify and fix any gaps.

### Verification (all 18 headless checks pass)

**Persistence pipeline (10 checks):**
- Missing `boids_overrides.json` → `load_overrides()` returns `{}`
- Corrupt JSON → returns `{}`
- Valid overrides loaded correctly
- `window_width` / `window_height` always skipped (runtime-only fields)
- Unknown field skipped
- Type-mismatched value skipped; valid sibling field kept
- Save → load round-trip is lossless
- `SimConfig` built correctly from loaded overrides
- Tuple color field (`sardine_color`) survives round-trip
- Bool field (`show_hud`) survives round-trip

**Runtime integration (8 checks):**
- Sardines spawned with loaded config values
- Sailfish spawned with loaded config values
- Config-menu row text reflects loaded value
- `add_sardines` uses loaded config for newly spawned fish
- `add_sailfish` uses loaded config for newly spawned fish
- `remove` + `add` cycle preserves loaded config
- `apply_config` after load propagates to all live agents
- 30-frame stability after config load and `apply_config`

Conclusion: the existing persistence pipeline was complete and correct.  No startup-loading changes were needed.

### Gap identified: `num_sardines` / `num_sailfish` staleness

**Problem:** `add_sardines`, `remove_sardines`, `add_sailfish`, and `remove_sailfish` in `simulation.py` mutated the agent lists but never updated `self.config`.  So if the user changed the population at runtime (↑/↓/←/→ keys) and then saved with `S`, the saved JSON stored the *startup* count rather than the live count.  On the next run the simulation would spawn the wrong number of agents.

**Fix (`simulation.py`):**

Added a new private helper `_sync_population_counts()` and called it at the end of each of the four population-management methods:

```python
def _sync_population_counts(self) -> None:
    from dataclasses import replace as dc_replace
    live_ns = len(self.sardines)
    live_nf = len(self.sailfish)
    if (
        self.config.num_sardines != live_ns
        or self.config.num_sailfish != live_nf
    ):
        self.config = dc_replace(
            self.config,
            num_sardines=live_ns,
            num_sailfish=live_nf,
        )
```

The guard (`!=` check) avoids allocating a new `SimConfig` on no-op calls (e.g. `remove_sardines(0)`).

**Verification (7 additional headless checks — all pass):**
- `simulation.config.num_sardines` matches `len(simulation.sardines)` after construction
- Matches after `add_sardines(50)`
- Matches after `remove_sardines(30)`
- Same for sailfish: initial / after `add_sailfish(3)` / after `remove_sailfish(1)`
- Saved JSON `num_sardines` and `num_sailfish` match the live counts after save

### Next recommended step

Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py`.


## 21. Phase 6h — Runtime Config Menu Polish (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Improve the readability, layout, and usability of the runtime config menu without overengineering or breaking existing functionality.

### Changes (`config_menu.py`)

#### 1. Section-header rows (`_GroupRow`)

Added a new `_GroupRow` frozen dataclass alongside `_MenuItem` / `_SaveRow`:

```python
@dataclasses.dataclass(frozen=True)
class _GroupRow:
    """Non-navigable section-header row rendered as a dim divider."""
    label: str
```

Five `_GroupRow` entries inserted in `_ROWS`, one before each logical group:
- `_GroupRow("SARDINE FLOCKING")` — before Sep/Ali/Coh/Noise weight
- `_GroupRow("KINEMATICS")` — before Sardine speed/force
- `_GroupRow("PERCEPTION RADII")` — before Sep/Ali/Coh radius
- `_GroupRow("FLEE / AVOIDANCE")` — before Flee weight/radius
- `_GroupRow("SAILFISH")` — before Pursuit wt/radius, Sailfish speed/force

Group header rows render at font-size 10 in dim colour (`_C_GROUP_TEXT`) and are
**never selectable** — the UP/DOWN navigation skips them with a `while isinstance(_ROWS[new], _GroupRow)` guard. `_NUM_ROWS` is still `len(_ROWS)` (now 21) so panel height adjusts automatically.

#### 2. Left-edge selection accent bar (`_sel_bar`)

A new 3 × `_ROW_H` px bright-blue (`rgba 100, 160, 255, 230`) `pyglet.shapes.Rectangle`
sits at the left edge of the cursor highlight, giving a clear "active row" indicator on top of the translucent background. It moves with the cursor bar in `_refresh_rows`.

#### 3. Cursor highlight brightness

`_C_CURSOR_ALPHA` raised from 100 → 140, making the full-width cursor bar noticeably more visible, especially over complex backgrounds.

#### 4. Panel width

`_PANEL_W` widened from 305 → 320 px, giving a few extra pixels of horizontal breathing room for labels and values at 13 pt Menlo.

#### 5. Group header colour constant

`_C_GROUP_TEXT = (80, 108, 150, 165)` added — a dim steel-blue used for non-navigable section labels, clearly distinct from both the selected (`_C_ROW_SEL`) and unselected (`_C_ROW_DIM`) parameter rows.

#### 6. Cleaner header key-guide

Normal-state header updated from:
```
CONFIG   ↑↓ move  ←→ edit  S save  TAB
```
to:
```
CONFIG  ↑↓ nav  ←→ edit  S save  ESC
```
`"move"` → `"nav"` (shorter), `"TAB"` → `"ESC"` (more prominent close-key).

#### 7. Cleaner footer hint text

`_MenuItem` footer updated from:
```
  step 0.10  ·  Shift ×1.0  ·  [0.0–10.0]
```
to:
```
  ←→ ±0.10  ·  Shift ±1.00  ·  [0.0–10.0]
```
- `"step N"` → `"←→ ±N"` — shows *which keys* and a ± to signal bidirectional change.
- `"Shift ×cs"` → `"Shift ±cs"` — removes the confusing × symbol.

`_SaveRow` footer updated from:
```
  ↵ Enter  ·  ←→  ·  writes: boids_overrides.json
```
to:
```
  ↵ Enter / ←→  ·  saves to boids_overrides.json
```

#### 8. Window-height safety clamp

`_panel_origin()` now clamps the panel's bottom edge to `y ≥ 0` so the panel never extends below the screen on small windows or during fullscreen toggles:

```python
py = win_h - _PANEL_TOP - self._panel_h()
return _PANEL_X, max(0, py)
```

#### 9. Initial cursor position

`self._selected` initialised to the first non-`_GroupRow` index instead of hard-coded `0`.  Since `_ROWS[0]` is now a group header, this prevents the cursor from landing on a non-navigable row at startup.

### `_ROWS` type annotation

Updated from `list[_MenuItem | _SaveRow]` to `list[_MenuItem | _SaveRow | _GroupRow]`.

### Navigation summary (updated)

| Key | Open menu behaviour |
|---|---|
| `↑` / `↓` | Move cursor, **skipping** `_GroupRow` section headers |
| `←` / `→` | Edit value (parameter rows) / activate Save (Save row) |
| `Shift` + `←/→` | Coarse edit (parameter rows only) |
| `↵ Enter` | Activate Save (Save row only) |
| `S` | Save from any row |
| `ESC` / `TAB` | Close menu |

### Remaining limitations

- **No scroll**: the panel is a fixed-height list.  Adding more rows (e.g. `num_sardines`) would eventually overflow a short window even with the y-clamp.  If more than ~5 more rows are added, a scroll mechanism or a second-panel approach should be considered.
- **No mouse support**: navigation is keyboard-only.
- **Font fallback**: if Menlo/Consolas/Courier New are all absent, pyglet falls back to its built-in bitmap font which may look misaligned with the fixed column widths.

### Next recommended steps

1. Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py`.
2. Consider adding `num_sardines` and `num_sailfish` as editable `_MenuItem` rows (they already persist correctly via the staleness fix).
3. Phase 7: burst-acceleration for sailfish + nearest-prey targeting.


## 22. Phase 6i — Validation & Cleanup Pass (completed)

**Status: Done.  53/53 tests still passing.**

### Objective

Perform a systematic validation of the runtime config menu and config persistence system, clean up any small issues found, and ensure PROJECT_LOG.md accurately reflects the implemented system.

### Validation checklist

| # | Behaviour | Result |
|---|---|---|
| 1 | Menu open/close (TAB / ESC / TAB while open) | ✓ Correct — `handle_key` consumes ESC/TAB when open; `on_key_press` calls `toggle()` when closed |
| 2 | Navigation (↑/↓ skip `_GroupRow`, wrap around) | ✓ Correct — `while isinstance(_ROWS[new], _GroupRow)` guard in both directions |
| 3 | Editable values update live simulation | ✓ Correct — `_adjust_value` calls `simulation.apply_config` on every key press |
| 4 | Save writes config atomically | ✓ Correct — `tmp.write_text → tmp.replace(OVERRIDE_FILE)`; OSError caught and shown as red feedback |
| 5 | Restart restores saved values | ✓ Correct — `load_overrides()` called at startup in `main.py`; all field types validated; runtime fields skipped |
| 6 | Resize / fullscreen compatibility | ✓ Correct — `on_resize` → `menu.on_resize` → `_build(w, h)`; panel y clamped to `max(0, py)` |
| 7 | HUD and menu coexist | ✓ Non-overlapping — HUD at `y = window_height − 8` (top-left corner); menu panel top at `window_height − 20`; HUD text ends well above menu header on any normal window size |
| 8 | Dynamic population controls (↑/↓/←/→) | ✓ Correct — keys consumed by `handle_key` when menu is open; pass through to `add/remove_sardines/sailfish` + `renderer.reset` when closed |
| 9 | No crash on repeated value changes | ✓ Correct — boundary clamping + float round(6 dp) in `_adjust_value`; `SimConfig` replaced atomically via `dataclasses.replace` |

### Issues found and fixed

#### 1. `config_menu.py` — `_footer_text` latent crash on `_GroupRow`

`_footer_text` had no guard for `_GroupRow` items.  If called with a `_GroupRow` it would fall through to `item.step` which does not exist on that type.  Although the function was never actually called with a `_GroupRow` at runtime (navigation always skips them), the lack of a guard was a latent bug.

**Fix:** Added an early-return guard at the top of `_footer_text`:
```python
if isinstance(item, _GroupRow):
    return ""
```
Updated the type annotation from `"_MenuItem | _SaveRow"` to `"_MenuItem | _SaveRow | _GroupRow"`.

#### 2. `config_menu.py` — Dead `# e.g.` comments in `_footer_text`

Two `# e.g. ...` comment lines were placed after `return` statements inside the function body — unreachable code.

**Fix:** Removed both dead comment lines.

#### 3. `config_menu.py` — `_row_text` docstring unclear about `_GroupRow`

Nothing in the docstring explained that `_GroupRow` items are handled separately (in `_build` and `_refresh_rows`) and never passed to `_row_text`.

**Fix:** Added a `Note:` paragraph to the `_row_text` docstring clarifying this.

#### 4. `config.py` — `cell_size` / `sailfish_cell_size` comments overstated constraint

Both comments said "Must be ≥ radius …" implying a correctness requirement.  In reality, `_candidate_cells` in `spatial_hash.py` computes the lookup ring from `floor((x ± radius) / cell_size)` so it handles any radius correctly by scanning proportionally more cells.  Keeping cell_size ≥ radius is a **performance** guideline (avoids scanning more than a 3×3 ring), not a hard requirement.

**Fix:**
- `cell_size`: `"cell_size >= radius_cohesion ensures a single 3×3 ring lookup suffices"` → `"For best performance cell_size >= radius_cohesion … _candidate_cells handles larger radii correctly but at proportional CPU cost."`
- `sailfish_cell_size`: `"Must be ≥ radius_flee so a single 3×3 ring lookup always suffices"` → `"For best performance, keep this >= max(radius_flee, radius_sailfish_awareness) … Larger radii work correctly but cost proportionally more per frame."`

#### 5. `PROJECT_LOG.md` Phase 6g — wrong field name

The Phase 6g verification section recorded `Bool field ('wrap_edges') survives round-trip`.  No such field exists in `SimConfig`.  The actual bool field tested was `show_hud`.

**Fix:** Changed `wrap_edges` → `show_hud` in that bullet point.

### No-change findings (verified correct as-is)

- `apply_config()` correctly refreshes all cached derived values (`_sep_r2`, `_ali_r2`, `max_speed`, `max_force`, `min_speed`) for both sardines and sailfish.  `radius_cohesion` and `radius_sailfish_awareness` are read from `self._config` directly each frame, no stale cache.
- `resize()` correctly propagates new world dimensions and clamps agent positions toroidally.  It does not touch kinematics (correct — resize only changes world geometry).
- `_sync_population_counts()` keeps `config.num_sardines` / `num_sailfish` in sync after every add/remove call.
- `save_config()` atomic write leaves the previous file intact on any failure mid-write.
- `load_overrides()` correctly skips `window_width`/`window_height`, unknown fields, and type-mismatched values.
- Navigation wraps correctly at both ends of `_ROWS` (initial selection, wrap from SaveRow back to first `_MenuItem`, wrap from first `_MenuItem` up to `_SaveRow`).
- TAB/ESC/F/H keys all route correctly between menu-open and menu-closed states.

### Remaining known limitations

- Panel is fixed height (~528 px for 21 rows).  On windows shorter than ~580 px the top rows may be clipped; the y-clamp prevents a crash but cannot shrink the panel.
- No scroll support.  Adding more than ~5 new rows risks overflow on small screens.
- No mouse interaction.
- `radius_flee` or `radius_sailfish_awareness` enlarged past `sailfish_cell_size` (120 px default) causes more spatial hash cells to be scanned per frame (correct results, minor CPU cost increase).

### Next recommended steps

1. Add `TestConfigPersistence` and `TestApplyConfig` suites to `tests.py`.
2. Consider adding `num_sardines` / `num_sailfish` as editable `_MenuItem` rows.
3. Phase 7: sailfish burst acceleration + nearest-prey targeting.

---

## 23. Repository Hygiene — Production `.gitignore` (completed)

**Date:** 2026-04-07

### What was done

Replaced the stub `.gitignore` (previously just `boids_overrides.json` + `boids_overrides.json.tmp` added in Phase 6a) with a comprehensive, production-quality `.gitignore` covering all real artifact classes found in the repository.

### Analysis performed

| Category | Finding |
|---|---|
| Python bytecode | `__pycache__/` present at root and `utils/`; `.pyc` files for CPython 3.13 and 3.14 both present |
| Virtual environments | None present; common directory names (`venv/`, `.venv/`, `env/`) added proactively |
| OS artifacts | macOS `.DS_Store` and related Finder metadata files present (standard macOS dev environment) |
| IDE artifacts | VS Code `.vscode/` folder (`.git/` confirmed — repo already initialised) |
| Runtime-generated config | `boids_overrides.json` — written by `config_io.save_config()` on any `S` keypress in-game |
| Profiling output | `*.prof` / `*.pstats` — not present but plausible given the Phase 4 perf work |
| Temp atomic write file | `boids_overrides.json.tmp` — written by `save_config()`'s atomic write pattern then renamed away; may persist on crash |
| Legacy file | `boidsNumpy.py` (pygame-based reference) mentioned in log but no longer present in working tree |

### Config override persistence decision

`boids_overrides.json` is **excluded from git** for the following reasons:

1. **Screen-specific geometry**: the file always contains `window_width`/`window_height` snapshot values from the machine that saved it.  Another contributor's screen resolution differs.
2. **Session-tuned population counts**: `num_sardines=1250`, `num_sailfish=12` in the current saved file are experiment values, not stable defaults.
3. **Canonical defaults are in code**: `config.py` → `SimConfig` is the authoritative baseline.  Contributors always start from those defaults and tune from there.
4. **No shared config baseline**: this is a single-developer project with no need to share tuning sessions.

The `*_overrides*.json` glob is also ignored to cover any experiment backups created manually.

### What is explicitly NOT ignored (safety check)

All active source files verified present and unignored:

| File | Tracked |
|---|---|
| `main.py` | ✓ |
| `config.py` | ✓ (default parameter values) |
| `config_io.py` | ✓ (persistence logic) |
| `config_menu.py` | ✓ (overlay UI) |
| `boid.py` | ✓ |
| `sardine.py` | ✓ |
| `sailfish.py` | ✓ |
| `simulation.py` | ✓ |
| `spatial_hash.py` | ✓ |
| `renderer.py` | ✓ |
| `tests.py` | ✓ |
| `utils/__init__.py` | ✓ |
| `utils/vec2.py` | ✓ |
| `requirements.txt` | ✓ |
| `PROJECT_LOG.md` | ✓ |

### `.gitignore` sections

| Section | Key entries |
|---|---|
| Python | `__pycache__/`, `*.py[cod]`, `*.so`, `dist/`, `build/`, `*.egg-info/`, coverage/type-checker caches |
| Virtual environments | `venv/`, `.venv/`, `env/`, `ENV/`, `Pipfile.lock`, `.conda/` |
| macOS | `.DS_Store`, `._*`, `.Spotlight-V100`, etc. |
| IDE / editor | `.vscode/`, `.idea/`, Sublime Text, Vim/Emacs swap files |
| Logs / runtime artifacts | `*.log`, `*.out`, `*.prof`, `*.pstats`, `*.tmp`, `*.bak` |
| Project-specific | `boids_overrides.json`, `*_overrides*.json`, `*.snapshot`, `*.replay` |

---

## Section 24 — Workspace Reorganisation + Velocity-Based Fish Colouring

### 24.1 Motivation

Two independent improvements were requested together:

1. **Visual**: Re-introduce fish colouring as a toggleable feature so schooling
   fish show emergent colour patterns (velocity-based HSV mapping).
2. **Structural**: Reorganise all modules into logical sub-packages to reduce
   root-level clutter and make import relationships explicit.

### 24.2 New Folder Structure

```
Boids/
├── main.py                    ← entry point (imports updated)
├── boids_overrides.json       ← runtime config overrides (gitignored)
├── requirements.txt
├── PROJECT_LOG.md
├── .gitignore
│
├── config/                    ← NEW package
│   ├── __init__.py            ← re-exports SimConfig, CONFIG, load_overrides, save_config, ConfigMenu
│   ├── config.py              ← SimConfig frozen dataclass (+ use_velocity_coloring field)
│   ├── config_io.py           ← save/load JSON overrides
│   └── config_menu.py         ← runtime overlay UI (+ _ToggleItem, Vel coloring row)
│
├── sim/                       ← NEW package
│   ├── __init__.py            ← re-exports Simulation, Boid, Sardine, Sailfish, SpatialHash
│   ├── boid.py
│   ├── sardine.py
│   ├── sailfish.py
│   ├── simulation.py
│   └── spatial_hash.py
│
├── rendering/                 ← NEW package
│   ├── __init__.py            ← re-exports Renderer
│   └── renderer.py            ← Renderer + Hud + velocity-colour logic
│
├── tests/                     ← NEW package
│   ├── __init__.py
│   └── tests.py               ← all 53 tests (imports updated)
│
└── utils/                     ← existing package (unchanged)
    ├── __init__.py
    └── vec2.py
```

Old root-level files (`config.py`, `config_io.py`, `config_menu.py`, `boid.py`,
`sardine.py`, `sailfish.py`, `simulation.py`, `spatial_hash.py`, `renderer.py`,
`tests.py`) were deleted after the packages were verified.

### 24.3 Velocity-Based Fish Colouring

#### Feature overview

When `SimConfig.use_velocity_coloring` is `True`, each sardine's triangle
colour is recalculated every frame based on its current kinematics.

| HSV channel | Source | Meaning |
|---|---|---|
| **Hue** | `speed / sardine_max_speed` → `[0.50, 0.80]` | Blue (slow) → Teal (fast) |
| **Saturation** | `1 − clamp(smooth_turn_rate / (π/6))` → `[0.30, 0.95]` | Vivid (straight swim) → Desaturated (tight turn) |
| **Value** | constant `0.95` | Always bright |

Turn rate is estimated frame-to-frame from the change in heading angle, smoothed
by a 0.85-weight exponential moving average to tame single-frame noise.

**Emergent behaviour**: Fish that school together converge to similar hues
because they share speed and heading.  Fleeing or scattered fish diverge
visibly — fast-fleeing fish shift teal; tightly-turning fish grey out.

#### New config field

```python
# config/config.py  (SimConfig frozen dataclass)
use_velocity_coloring: bool = False
```

Persisted in `boids_overrides.json` like every other config field.

#### Toggle paths

| Method | Effect |
|---|---|
| **V key** (global shortcut, `main.py`) | Instant toggle, not saved |
| **TAB → VISUAL → Vel coloring ← / → or Enter** (config menu) | Toggle via menu; save with S to persist |
| **`boids_overrides.json`** | Set `"use_velocity_coloring": true` to start with it on |

#### Implementation: `rendering/renderer.py`

Key additions:

- `_VelColorState` — per-fish dataclass storing `prev_angle`, `smooth_turn`,
  `frames` — allocated in `Renderer.reset()` alongside the Triangle shapes.
- `_sync_shapes_vel_color()` — replaces the flat `_sync_shapes()` call in the
  hot path when velocity colouring is active.  Updates vertex positions **and**
  `tri.color` in one pass.
- `_hsv_to_rgb()` — pure-Python inline converter; avoids the `colorsys` import
  for a negligible but consistent gain in the 60 fps loop.
- `Renderer.draw()` — branches on `simulation.config.use_velocity_coloring`
  each frame.  When the flag is toggled off, stored `_VelColorState.frames`
  are reset to `0` so the first frame after re-enabling does not produce a
  stale turn-rate jump.
- HUD shows `[VEL COLOR]` indicator when the mode is active.

### 24.4 Config Menu Changes (`config/config_menu.py`)

New `_ToggleItem` row type added alongside the existing `_MenuItem`, `_GroupRow`,
`_SaveRow`:

```python
@dataclasses.dataclass(frozen=True)
class _ToggleItem:
    label: str   # display label (≤ 16 chars)
    field: str   # SimConfig boolean attribute name
```

New `_GroupRow("VISUAL")` section with one row:

```
_ToggleItem("Vel coloring", "use_velocity_coloring")
```

Key handling: LEFT / RIGHT and ENTER all toggle the boolean in-place via
`_toggle_bool()` → `dataclasses.replace()` → `simulation.apply_config()`.
The current ON/OFF state is shown in the right column of the menu.

### 24.5 Import Path Migration

| Old import | New import |
|---|---|
| `from config import SimConfig` | `from config.config import SimConfig` |
| `from config_io import load_overrides` | `from config.config_io import load_overrides` |
| `from config_menu import ConfigMenu` | `from config.config_menu import ConfigMenu` |
| `from simulation import Simulation` | `from sim.simulation import Simulation` |
| `from renderer import Renderer` | `from rendering.renderer import Renderer` |
| `from boid import Boid` | `from sim.boid import Boid` |
| `from sardine import Sardine` | `from sim.sardine import Sardine` |
| `from sailfish import Sailfish` | `from sim.sailfish import Sailfish` |
| `from spatial_hash import SpatialHash` | `from sim.spatial_hash import SpatialHash` |

`utils/vec2.py` imports are unaffected (`from utils.vec2 import ...` was correct
before and after).

`config/config_io.py` OVERRIDE_FILE path changed from
`pathlib.Path(__file__).parent / "boids_overrides.json"` to
`pathlib.Path(__file__).parent.parent / "boids_overrides.json"` since
`config_io.py` is now one directory deeper.

### 24.6 Test Results

```
Ran 53 tests in 2.276s
OK
```

All tests pass unchanged.  Run command:

```bash
python -m tests.tests
```

### 24.7 Run Commands (updated)

| Task | Command |
|---|---|
| Run simulation | `python main.py` |
| Run tests | `python -m tests.tests` |
| Toggle velocity colouring | **V** key (instant) or TAB → VISUAL → Vel coloring |
