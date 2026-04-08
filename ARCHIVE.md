# ARCHIVE.md — Historical Development Notes

This file stores detailed build history, phase-by-phase implementation notes, and
step-by-step changelists migrated from `PROJECT_LOG.md` to keep that document clean.

---

## Phase 1 — Core Boid Simulation

Built the foundational classes: `SimConfig` (frozen dataclass), `Boid` (base), `Sardine`
(flocking), `SpatialHash` (uniform grid), `Simulation`, and `Renderer` (pyglet 2.x batch).

Key decisions:
- **pyglet ≥ 2.0** over pygame — pure-Python, OpenGL batch rendering, actively maintained.
- **numpy float64 arrays** for vector math — fast and readable; no custom Vec2 class initially.
- **Toroidal world** — natural for open-sea schooling; eliminates edge-avoidance special-casing.
- **Uniform grid spatial hash** — O(1) average insert/query; `cell_size = radius_cohesion`.
- **Frozen dataclass SimConfig** — single source of truth; immutable after startup.
- **`_compute_forces()` hook** — subclasses override and accumulate forces; `Boid.update()` integrates.

Sardine flocking forces (single-pass neighbour loop):
- Separation: `1/distance` weighted repulsion within `radius_separation`.
- Alignment: average velocity within `radius_alignment`.
- Cohesion: steer toward COM within `radius_cohesion`.

Long-run stability test (600 frames): speeds in `[60, 131]` px/s, no NaN/Inf, no escapes.

Initial tuned values: `radius_separation=30`, `radius_alignment=60`, `weight_separation=1.8`,
`weight_alignment=1.0`, `sardine_max_speed=180`, `sardine_max_force=240`.

---

## Phase 2 — Sailfish & Sardine Flee

Added `Sailfish(Boid)` and sardine flee behaviour.

**Sailfish design:**
- Queries sardine-only grid at `radius_sailfish_awareness`; steers toward toroidal COM.
- Coasts at `sailfish_min_speed` when no sardines visible.
- Excluded from the sardine grid — sardines cannot flock with sailfish.

**Sardine flee:**
- Queries sailfish grid at start of `_compute_forces()` (before the sardine-neighbour
  early-return), so flee fires even when sardine is isolated.
- Sums unit away-vectors from all sailfish within `radius_flee`; applies
  `steer_toward(flee_vec) × weight_flee`.
- **Cohesion zeroed while fleeing** → school splits.
- **Alignment kept** → fleeing sub-groups remain coordinated.
- Flee is stateless: normal flocking resumes automatically when sailfish exits radius.

**Two-grid architecture:**
- `_grid` rebuilt from `self.sardines` only.
- `_sailfish_grid` rebuilt from `self.sailfish` only.
- No type-filtering needed at query time.

Bug fixed: `SpatialHash._candidate_cells()` used unbounded (col,row) keys → agents near a
world edge couldn't find wrapped neighbours. Fixed with modulo-wrapping + set deduplication.

`TestPhase2` suite added (7 tests). Total: 53/53 tests.

---

## Phase 3 — Renderer HUD + Pyglet 2.x Fixes

Added `Hud` class — `pyglet.text.Label` with FPS, frame/update/render ms, agent counts,
world dimensions. Rebuilt at most 4×/second.

**Bugs fixed:**
- `pyglet.clock.get_fps()` removed in pyglet 2.x → replaced with manual `_frame_count`
  increment + wall-clock FPS calculation.
- `pyglet.text.Label(bold=True)` removed in pyglet 2.1.x → replaced with `weight='bold'`.
- Secondary macOS Cocoa `AttributeError` exceptions (`PygletDelegate has no attribute…`)
  are teardown artefacts that disappear once the primary crash is resolved — no code fix needed.

`Renderer._sync_shapes()` rewritten to use pure Python float math, eliminating ~604 numpy
array allocations per frame at default population.

---

## Phase 4 — Performance Optimization

**Bottleneck audit:**

| Location | Problem |
|---|---|
| `utils/vec2.normalize`, `limit` | `np.linalg.norm` for 2-D vector — 5–10× slower than `math.sqrt` |
| `spatial_hash` | `np.floor`/`np.ceil` for single floats — numpy dispatch overhead |
| `spatial_hash.query` | `wrap_delta()` function call ×2 per candidate pair (~90k calls/frame) |
| `sardine._compute_forces` | 3× `np.zeros(2)` + numpy iadd temps per frame (2700+ allocations) |
| Config attribute chain | `float(cfg.window_width)` etc. re-evaluated every frame per agent |
| `simulation.update` | `_all_agents()` creates a merged list every frame |

**Optimizations applied:**
- `utils/vec2`: `math.sqrt(v[0]*v[0]+v[1]*v[1])` in all 2-D norm calls.
- `spatial_hash`: plain `dict` (no `defaultdict`); `//` and `math.floor`/`ceil`; inlined
  `wrap_delta`; `_half_w/_half_h` cached; `insert` avoids `_cell_key` method call.
- `boid.py`: `_steer_apply_xy(tx, ty, weight)` — inlines normalize+limit+accumulate into
  pure float arithmetic, writes directly to `self.acceleration[0/1]`. Eliminates 2–4 temp
  arrays per steering call.
- `sardine.py`: `sx/sy`, `alx/aly`, `cx/cy` float accumulators replace `np.zeros(2)`;
  `_W/_H/_hw/_hh/_sep_r2/_ali_r2` cached as instance attrs at `__init__`.
- `sailfish.py`: same float-pair pattern; cached dims; inlined wrap.
- `simulation.update`: iterates `self.sardines` then `self.sailfish` directly (no list merge).

Added runtime population controls: `add_sardines(n)`, `remove_sardines(n)`, `add_sailfish(n)`,
`remove_sailfish(n)`. Each spawns/removes agents safely between frames.

Added keyboard controls: `↑/↓` add/remove 50 sardines; `[/]` add/remove 1 sailfish (later
remapped in Phase 5). HUD key guide added to renderer.

---

## Phase 5 — Resize, Fullscreen, and Control Remapping

**Changes:**
- `resizable=True` window; `on_resize(w, h)` handler in `main.py`.
- `_ready` guard prevents spurious `on_resize` during startup.
- Minimum size guard: `width < 200 or height < 150` → skip.
- `simulation.resize(w, h)` — creates new `SimConfig` via `dataclasses.replace`; propagates
  to all agents; updates `_W/_H/_hw/_hh`; calls `SpatialHash.resize()` on both grids;
  toroidally clamps all positions.
- `SpatialHash.resize(w, h)` — updates internal dimensions and clears cell dict.
- `Hud.reposition(h)` — moves label anchor on resize.
- `Renderer.on_resize(w, h)` — calls `_hud.reposition()`.
- `F` key: `window.set_fullscreen(not window.fullscreen)` — auto-fires `on_resize`.
- `[/]` bindings replaced by `←/→` (LEFT/RIGHT arrows). `H` key added for HUD toggle.

No `renderer.reset()` needed on resize — existing Triangle shapes remain valid;
`_sync_shapes` updates vertices every frame.

---

## Phase 6 — Runtime Config Menu & Config Persistence

### Phase 6a — Config Persistence
- New `config_io.py`: `load_overrides() → dict`, `save_config(SimConfig) → None`.
- `OVERRIDE_FILE = path/to/boids_overrides.json` (sibling of `main.py`).
- `load_overrides()` skips `window_width`/`window_height`, unknown fields, and
  type-mismatched values. Type coercion via `typing.get_type_hints()` introspection.
- `save_config()` uses atomic write: write to `.tmp`, then `Path.replace()` rename.
- `main.py` applies `**load_overrides()` when constructing `SimConfig`.
- `.gitignore` created with `boids_overrides.json` excluded (screen-specific, session-tuned values).

### Phase 6b — Config Menu Shell
- `config_menu.py`: `ConfigMenu(window, simulation)` class.
- `toggle()`, `handle_key(symbol, modifiers) → bool` (modal guard), `on_resize()`, `draw()`.
- Visual: dark navy panel (305 px wide), left-anchored; dark header strip; blue cursor bar.
- Simulation runs while menu is open — no pause.
- Arrow key conflict resolved modally: `handle_key` returns `True` when menu is open,
  preventing population controls from firing.

### Phase 6c — Value Editing & Save
- `Simulation.apply_config(new_config)` added — O(N) attribute propagation to all agents.
- `_MenuItem` dataclass with `field`, `step`, `coarse`, `min_val`, `max_val`.
- `_adjust_value()`: reads current, adds ±step (Shift = coarse), clamps, rounds to 6dp,
  calls `apply_config(dataclasses.replace(...))`.
- `S` key: calls `save_config()`; 1.5 s green `✓ saved` header feedback.
- ESC while menu open → closes menu (not quit); ESC while menu closed → quit.

### Phase 6d — Kinematic Propagation Fix
- `apply_config()` now updates `Boid.max_speed`, `max_force`, `min_speed` instance attrs
  directly (not just `_config`). These are read on every physics tick — changes from the
  menu take effect immediately.

### Phase 6e — Sailfish Rows
- Sailfish kinematic propagation added to `apply_config()` sailfish loop.
- `_ROWS` reorganised into four groups; three new rows: Pursuit radius, Sailfish speed,
  Sailfish force. Total: 15 parameter rows.

### Phase 6f — Visible Save Row
- `_SaveRow` dataclass added as 16th row (non-editable, cursor-navigable).
- Enter / ←→ on Save row triggers `_save()`. `OSError` shown as 2.5 s red error feedback.

### Phase 6g — Population-Count Staleness Fix
- `add/remove_sardines/sailfish` did not update `self.config`. On save, stored startup
  counts not live counts.
- Fix: `_sync_population_counts()` called at end of each add/remove method; only creates
  a new `SimConfig` if counts actually differ.

### Phase 6h — Menu Polish
- `_GroupRow` non-navigable section headers (5 groups). Navigation skips `_GroupRow` with
  `while isinstance(_ROWS[new], _GroupRow)` guard.
- Left-edge 3px blue accent bar (`_sel_bar`) added.
- Panel widened to 320 px. Cursor alpha raised to 140. Group header colour dim steel-blue.
- Initial cursor skips to first non-`_GroupRow` row.
- Window-height safety clamp: `py = max(0, py)` in `_panel_origin()`.

### Phase 6i — Validation & Cleanup
- `_footer_text` latent crash fixed: added `_GroupRow` early-return guard.
- Dead `# e.g.` comments after return statements removed.
- `config.py` comments corrected: `cell_size`/`sailfish_cell_size` are **performance**
  guidelines, not hard correctness requirements.
- Full validation checklist: all 9 menu behaviours confirmed correct.

---

## Section 24 — Workspace Reorganisation & Velocity Colouring

Root-level files reorganised into `config/`, `sim/`, `rendering/`, `tests/` sub-packages.
Old flat files deleted after packages were verified.

**Import path migration summary:**

| Old | New |
|---|---|
| `from config import SimConfig` | `from config.config import SimConfig` |
| `from config_io import …` | `from config.config_io import …` |
| `from config_menu import …` | `from config.config_menu import …` |
| `from simulation import …` | `from sim.simulation import …` |
| `from renderer import …` | `from rendering.renderer import …` |

`config_io.py` OVERRIDE_FILE path updated: `.parent` → `.parent.parent` (one level up from `config/`).

**Initial velocity colouring (since superseded by Section 26):**
- `use_velocity_coloring: bool = False` config field (now removed).
- HSV channels: Hue from heading, Saturation from turn rate EMA, Value constant 0.95.
- `_VelColorState` per-fish dataclass storing turn-rate EMA state.
- V key toggled it on/off.

---

## Section 25 — Vel-Color Refinement (since superseded)

**Toggle-off color restore bug fix:**  
`_sync_shapes()` never touches `tri.color`, so disabling did not restore default color.  
Fix: track `_prev_vel_coloring`; on transition-off frame, run one O(N) `tri.color = _SARDINE_COLOR` pass.

Added four config fields: `vel_color_heading_weight`, `vel_color_speed_weight`,
`vel_color_turn_weight`, `vel_color_smoothing`.

**Improved color mapping**: heading as primary hue axis (full wheel) instead of speed-only
blue-to-teal range. Fish schools with different headings are visually distinct.

Formula:
```
raw_hue = (heading_01 × w_heading + speed_norm × 0.25 × w_speed) % 1.0
hue     = raw_hue                   # (in rainbow mode)
```

Note: `use_velocity_coloring` bool and `sailfish_color` tuple were removed in Section 26;
these fields are superseded by `sardine_color_mode` and `sailfish_color_mode` strings.

---

## Section 26 — Expanded Palette System (original implementation)

Replaced `use_velocity_coloring: bool` with `sardine_color_mode: str` (7 modes) and
`sailfish_color: tuple` with `sailfish_color_mode: str` (5 modes).

**Migration in `load_overrides()`:**
- `"use_velocity_coloring": true` → `sardine_color_mode = "rainbow"` (if not also explicitly saved).
- `"sailfish_color": [R,G,B,A]` → silently dropped; `sailfish_color_mode = "default"`.

`_CycleItem` dataclass added to config menu. V key changed from bool toggle to cycling all
7 modes in order.

**Linear band mapping (since replaced by cosine in Section 27):**
```
hue = hue_min + raw_hue × band_width
```
This caused visible heading-wrap flicker on restricted palettes. See Section 27.

---

## Section 27 — Cyclic Palette Fix

**Root cause of heading-wrap flicker:**
Linear band mapping `hue = hue_min + raw_hue × band_width` had a discontinuity when
`raw_hue` crossed a multiple of 1.0. At that wrap boundary, hue jumped from `hue_max`
back to `hue_min` (e.g. purple → teal in `cool` mode).

**Fix — cosine cyclic mapping:**
```python
hue_center = (hue_min + hue_max) * 0.5
band_half  = (hue_max - hue_min) * 0.5
hue = (hue_center + band_half * math.cos(raw_hue * TWO_PI)) % 1.0
```
`cos(0) = cos(2π)` guarantees identical color at both ends of the raw_hue range.
Fish with the same heading/speed map to the same palette position regardless of
the wrap boundary. School coherence fully preserved.

**Symmetry tradeoff:** Two schools heading in exactly opposite directions receive the same
palette color (cos is symmetric). In practice not noticeable — diverging schools quickly
separate in position.

`rainbow` and `mono` unaffected (rainbow uses raw_hue directly; mono ignores hue).

---

## Subclass Interface Contract

Rules for adding new `Boid` subclasses:

| Rule | Detail |
|---|---|
| Call `super().__init__()` | Pass `x, y, vx, vy, max_speed, max_force, config`, optionally `min_speed` |
| Override `_compute_forces()` | Accumulate via `self.apply_force()` or `_steer_apply_xy()` — never integrate directly |
| Do not call `update()` from `_compute_forces()` | Breaks integration |
| Use `self.steer_toward(desired)` or `_steer_apply_xy(tx, ty, w)` | Returns force clamped to `max_force` |
| No rendering imports | Subclasses must not touch pyglet/renderer |
| Grid injection | Receive `SpatialHash` as constructor param; store as instance attr; never import `Simulation` |

Adding a third species: add a new `SpatialHash`, rebuild it in `Simulation.update()`,
inject it into any agent that needs to query it. The two-grid architecture is the established pattern.

---

## Algorithmic Complexity Reference

| Operation | Complexity |
|---|---|
| Grid rebuild per frame | O(N) inserts |
| Neighbour query per agent | O(1) average — fixed-radius ring of ~9 cells |
| Force computation per agent | O(k) where k = mean neighbours in radius |
| Full frame update | O(N × k) ≈ O(N) for constant density |
| `Simulation.apply_config()` | O(N) attribute writes |
| `Simulation.resize()` | O(N) position clamp + attribute writes |

**Scaling limit**: Python per-object loop saturates at ~1 500 sardines. Beyond that,
numpy-batched force computation (all-sardine positions in a single (N,2) array) would
eliminate the per-object interpreter overhead.

---

## Tuning Reference Table (historical)

Full parameter tuning progression across phases:

| Parameter | Current | Phase 1 start | Notes |
|---|---|---|---|
| `num_sardines` | 900 | 300 | Raised in Phase 4, then 900 |
| `num_sailfish` | 8 | 2 | Raised over phases |
| `sardine_max_speed` | 150.0 | 180.0 | Reduced to limit chaos |
| `sardine_min_speed` | 50.0 | 60.0 | Reduced |
| `sardine_max_force` | 150.0 | 240.0 | Reduced; ratio to max_speed → 1.0 |
| `weight_separation` | 2.0 | 1.8 | Increased |
| `weight_alignment` | 1.3 | 1.0 | Increased |
| `weight_cohesion` | 1.0 | 1.0 | Stable |
| `weight_noise` | 0.04 | (new) | Added Phase 4 |
| `radius_separation` | 40.0 | 30.0 | Increased |
| `radius_alignment` | 75.0 | 60.0 | Increased |
| `radius_cohesion` | 80.0 | 80.0 | Stable |
| `radius_flee` | 100.0 | — | Phase 2 |
| `weight_flee` | 10.0 | 4.0 → 6.0 | Raised; must exceed sep (2.0) |
| `sailfish_cell_size` | 120.0 | 120.0 | Phase 2 |

---

## Phase 8 — Ecosystem: Algae, Food Chains, and Population Dynamics

> Status: Fully implemented. All stages 8.1–8.8 shipped. Current behavior described in PROJECT_LOG.md.

### Overview

Three-level food chain:

```
Algae  →  Sardines eat algae  →  Sardines reproduce
Sardines  →  Sailfish eat sardines
Each predator starves if it goes too long without eating.
```

Population is self-regulating: sardines grow when algae is plentiful; sailfish keep sardine numbers in check; starvation provides a top-down population floor for predators.

### Architecture additions

#### `sim/algae.py` — `Algae(Boid)`

Algae subclasses `Boid` (same base as `Sardine` and `Sailfish`). Rationale: `Boid` already handles position/velocity/acceleration, Euler integration, boundary wrapping, and speed clamping. The `_compute_forces()` hook is exactly the override point for algae's light flocking. Algae must live in a spatial hash so sardines can query it during eating.

`Algae._compute_forces()`: alignment-dominant Reynolds flocking with no predator awareness. `min_speed = 0.0` (can drift to near-stop).

#### `food_counter` and `hunger_timer`

Added as plain instance attributes on `Sardine` and `Sailfish` (not a new subclass):

| Attribute | Type | Sardine | Sailfish |
|---|---|---|---|
| `food_counter` | `float` | algae eaten (toward reproduction threshold) | sardines eaten (toward reproduction threshold) |
| `hunger_timer` | `float` | seconds since last algae eaten | seconds since last sardine eaten |

Both initialised with `hunger_timer = config.<species>_initial_hunger_time` to stagger startup deaths.

#### `_algae_grid` — third SpatialHash

| Grid | Contents | Queried by |
|---|---|---|
| `_grid` | Sardines only | Sardines (flocking) + Sailfish (pursuit) |
| `_sailfish_grid` | Sailfish only | Sardines (flee) |
| `_algae_grid` | Algae only | Algae (flocking) + ecology pass (eating) |

#### Ecology pass in `Simulation.update()`

Runs after all `agent.update(dt)` calls (world mutation must not occur during iteration):

1. Rebuild `_algae_grid`
2. Increment `hunger_timer` on all sardines and sailfish by `dt`
3. Sardine–algae eating: sardines query `_algae_grid` at `algae_eat_radius`; mark algae for removal; `food_counter += 1`; `hunger_timer = 0.0`
4. Sailfish–sardine eating: sailfish query `_grid` at `sailfish_eat_radius`; one kill per sailfish per tick (prevents mass kill); `food_counter += 1`; `hunger_timer = 0.0`
5. Starvation: mark agents where `hunger_timer > starvation_time`
6. Reproduction: sardine/sailfish with `food_counter >= threshold` spawn one offspring at perpendicular offset; `food_counter -= threshold` (not reset, preserves excess)
7. Algae respawn: accumulator-based fractional spawn up to `algae_max_count`
8. Flush removals and additions; update renderer side-channels

**Why one kill per sailfish per tick**: prevents a single sailfish from eating an entire nearby school in one frame.

### Original Phase 8 config defaults (pre-tuning)

| Field | Original default | Final default | Change |
|---|---|---|---|
| `num_sardines` | 900 | 300 | Root cause of early collapse: 900 sardines depleted 500 algae in ≤2 s |
| `num_algae` | 500 | 800 | |
| `algae_respawn_rate` | 3.0 | 8.0 | |
| `algae_max_count` | — | 1200 | Added as soft cap |
| `sardine_eat_threshold` | 5 | 3 | |
| `sardine_starvation_time` | 20.0 | 30.0 | |
| `sardine_initial_hunger_time` | 10.0 | 15.0 | |
| `sailfish_eat_radius` | 20.0 | 25.0 | |
| `sailfish_starvation_time` | 45.0 | 90.0 | |
| `sailfish_initial_hunger_time` | 20.0 | 40.0 | |
| `algae_max_speed` | 30.0 | 25.0 | |
| `algae_max_force` | 15.0 | 12.0 | |
| `algae_eat_radius` | 12.0 | 14.0 | |
| `weight_algae_separation` | 0.2 | 0.3 | |
| `radius_algae_separation` | 12.0 | 18.0 | |
| `radius_algae_alignment` | 50.0 | 55.0 | sep:coh = 1:3 |
| `radius_algae_cohesion` | 55.0 | 55.0 | |
| `algae_cell_size` | 55.0 | 60.0 | |

### Staged implementation

| Stage | What | Status |
|---|---|---|
| 8.1 | `Algae(Boid)` + `_algae_grid` + spawning + algae flocking | ✅ |
| 8.2 | Renderer algae pass (reddish-brown circles, HUD algae count) | ✅ |
| 8.3 | Sardine `food_counter`/`hunger_timer`; ecology pass; algae eating; `Eaten ↑N` HUD | ✅ |
| 8.4 | Sardine reproduction | ✅ |
| 8.5 | Sardine starvation | ✅ |
| 8.6 | Sailfish `food_counter`/`hunger_timer`; sailfish–sardine eating | ✅ |
| 8.7 | Sailfish starvation | ✅ |
| 8.8 | `apply_config()` and `resize()` extended for algae | ✅ |
| 8.9 | Config menu rows for ecosystem params (8 sections, 40 rows, scrollable) | ✅ |
| 8.10 | Ecosystem stat counters + HUD expansion (7-line HUD) | ✅ |

### Sailfish reproduction (added post-8.10)

Added sailfish reproduction mirroring sardine reproduction: `food_counter >= sailfish_eat_threshold` → spawn one offspring; `food_counter -= threshold`; HUD shows cumulative births. `TestSailfishReproduction` suite added.

### Design constraints (resolved)

- **Menu height overflow**: solved by scroll mechanism — fixed 418 px panel height, 16 visible rows, ↑/↓ navigation with ▲/▼ scroll indicators.
- **Algae respawn vs. population**: `algae_max_count` soft cap; respawn pauses above it.
- **Sardine minimum population floor**: `sardine_initial_hunger_time` staggers startup deaths; `sardine_starvation_time=0` disables mechanic.
- **Offspring placement**: perpendicular offset (`radius_separation`) from parent; separation force naturally disperses.
- **Algae color**: fixed reddish-brown `(170, 50, 20, 255)` hard-coded in renderer; `algae_color_mode` not added (no demonstrated need).

### Test suites added (Phase 8)

`TestAlgaeKinematics`, `TestEcologyContactDetection`, `TestSardineReproduction`, `TestSailfishEcology`, `TestSardineStarvation`, `TestSailfishStarvation`, `TestAlgaeRespawn`, `TestHudEcosystem`, `TestConfigMenuEcosystem`, `TestSailfishReproduction` — 132 tests added to the original 53, totalling 185.
