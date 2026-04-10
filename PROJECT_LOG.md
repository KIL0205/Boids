# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

> This file is the authoritative source of truth for the current system. All AI agents must read this first before making changes and should update the log after task completion.
> **Working reference for active development.** Optimised for quick orientation, not history.
> Historical build notes, phase changelists, and implementation archaeology → [memory/ARCHIVE.md](memory/ARCHIVE.md)

---

## Log Maintenance
- Full maintenance and cleanup pass (April 2026): test counts corrected to 268/23 suites; config menu rows updated to current 7-section layout; completed Phase 9 TODO items removed; optimization pass implementation details (Phases 1–4C) migrated to [ARCHIVE.md](memory/ARCHIVE.md).

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

- **Runtime**: `python main.py` — resizable window at screen resolution, 60 fps target.
- **Tests**: `python -m unittest tests.tests` — 268 tests, ~0.9 s.
- **Benchmarks**: `python -m tests.bench` — 4 headless scenarios (light/medium/heavy/extreme).

### Current capabilities
- Sardines flock using Reynolds separation / alignment / cohesion + small noise term.
- Sailfish hunt sardine schools by steering toward the local centre of mass.
- Sardines detect nearby sailfish, flee with full-force escape, and scatter (cohesion suppressed while fleeing).
- **Algae drift** slowly across the world; alignment-dominant flocking gives a water-current feel.
- **Sardine feeding**: sardines eat algae on contact (`algae_eat_radius`); eaten algae vanish instantly, sardine `food_counter` and `hunger_timer` tracked.
- **Sardine reproduction**: when `food_counter` reaches `sardine_eat_threshold`, the sardine splits — one offspring spawned beside it (perpendicular offset = `radius_separation`), inheriting parent velocity; `food_counter` decremented by threshold (not reset to 0) so leftover progress carries forward.
- **Sailfish predation**: each sailfish eats the nearest sardine within `sailfish_eat_radius` each frame (one per sailfish per tick); the sardine is removed immediately; sailfish `food_counter` and `hunger_timer` track feeding state. HUD shows cumulative predation count (`Pred ↑N`).
- **Sailfish individual targeting**: when a sardine enters `sailfish_target_acquire_radius`, the sailfish locks onto that specific fish and pursues it with predictive interception (`predicted_pos = target.pos + target.vel * t`). At long range, sailfish still chase the school COM. Lock expires after `sailfish_target_lock_time` seconds or when the target is eaten/leaves `sailfish_target_drop_radius`.
- **Sailfish vulnerable-target preference**: target acquisition now uses a scoring function combining proximity, isolation from the school COM, and sardine speed. Sardines on the school periphery or moving slowly score higher and are preferred over identical ones buried in the school core. Weights are live-tuneable in the config menu.
- **Sailfish burst attack**: when a locked sardine enters `sailfish_burst_trigger_radius`, the sailfish fires a short-duration speed/force boost, then enters a cooldown before the next burst is allowed.
- **Sailfish shared-target reduction**: when multiple sailfish compete for the same sardine and an alternative exists nearby, the secondary sailfish redirects to the next-best scored candidate.
- **Sailfish school-pressure circling**: when a sailfish sees a dense school (≥ `sailfish_school_pressure_density_threshold` sardines in awareness radius) but hasn't locked an individual target, a tangential orbit force is added that causes sailfish to arc around the school edge rather than always diving straight into the tight center. Deactivates immediately once a target lock forms.
- **Sailfish reproduction**: when a sailfish's `food_counter` reaches `sailfish_eat_threshold` (sardines eaten), it spawns one offspring beside it (perpendicular offset = `radius_separation`), inheriting parent velocity; `food_counter` decremented by threshold (not reset to 0); HUD shows cumulative births.
- Live window resize and fullscreen toggle (world geometry updates in-place).
- Runtime config menu (TAB): 25+ parameters editable live across 8 sections; panel scrolls automatically; settings persist across runs.
- Velocity-based fish colouring: 7 sardine palette modes, 5 sailfish colour modes.
- Dynamic population controls from keyboard.
- HUD overlay (FPS, frame timing, per-species counts, totals, full ecosystem event stats, average sardine/sailfish hunger).
- **Mouse predator**: holding the left mouse button causes nearby sardines to flee the cursor — same flee mechanics as a real sailfish. A faint red radius indicator appears at the cursor. Configurable toggle, radius, and flee-weight multiplier.
- **Population graph overlay**: `D` key toggles a live population-vs-time graph in the bottom-right corner. Sardines (cyan-blue), sailfish (orange-red), and algae (amber-brown) are plotted independently with a legend and axis labels. History is stored in `simulation.pop_history` (a `collections.deque` sampled at ~1 Hz; capped at 3600 points ≈ 1 hour).

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
│   ├── sardine.py            # Sardine(Boid): flocking + flee; food_counter/hunger_timer
│   ├── sailfish.py           # Sailfish(Boid): COM pursuit; food_counter/hunger_timer
│   ├── algae.py              # Algae(Boid): alignment-dominant drift; no predator awareness
│   ├── simulation.py         # Simulation: owns agents + grids, drives update loop; ecology pass
│   └── spatial_hash.py       # Uniform-grid spatial hash — O(1) neighbour lookup
│
├── rendering/
│   ├── __init__.py           # Re-exports Renderer
│   └── renderer.py           # Renderer + Hud; velocity-colour logic; pyglet batch draw
│
├── tests/
│   ├── __init__.py
│   ├── bench.py              # Headless benchmark — python -m tests.bench [--profile] [--test]
│   └── tests.py              # 268 tests across 23 suites
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
| `Sardine(Boid)` | `sim/sardine.py` | Flocking (sep/ali/coh/noise) + flee; holds sardine + sailfish grid refs; `food_counter`, `hunger_timer` |
| `Sailfish(Boid)` | `sim/sailfish.py` | Two-stage hunting: COM pursuit at long range + individual predictive interception at close range; sailfish-to-sailfish flocking (sep/ali/coh); holds sardine grid ref + sailfish grid ref; `_target_sardine`, `_target_lock_timer`; `food_counter`, `hunger_timer` |
| `Algae(Boid)` | `sim/algae.py` | Alignment-dominant drift entity; holds algae grid ref; no predator awareness |
| `SpatialHash` | `sim/spatial_hash.py` | `insert()`, `query()`, `clear()`, `resize()` with toroidal cell-key wrapping |
| `Simulation` | `sim/simulation.py` | Spawns agents; owns three grids; `update(dt)`, `resize()`, `apply_config()`; ecology pass |
| `Renderer` | `rendering/renderer.py` | `reset()`, `draw()`, `on_resize()`, `toggle_hud()`, `toggle_pop_graph()`; velocity-colour logic; algae/sardine/sailfish shapes |
| `PopGraph` | `rendering/renderer.py` | Population-vs-time graph overlay; reads `simulation.pop_history`; drawn outside the batch |
| `ConfigMenu` | `config/config_menu.py` | `toggle()`, `handle_key()`, `draw()`, `on_resize()` |

### Data flow (one frame)

```
pyglet scheduler → update(dt)
  └─ Simulation.update(dt)
       ├─ sardine_grid.clear() + insert all sardines
       ├─ sailfish_grid.clear() + insert all sailfish
       ├─ algae_grid.clear() + insert all algae
       └─ for each agent:
            agent.update(dt)
              ├─ _compute_forces()        ← sardine: flock + flee; sailfish: COM pursuit; algae: drift
              ├─ Euler integration        v += a·dt;  p += v·dt
              ├─ _clamp_speed()           [min_speed, max_speed]
              └─ _apply_boundary()        toroidal wrap

pyglet on_draw → Renderer.draw(simulation)
  ├─ _sync_shapes()  (algae — fixed green, drawn first / behind)
  ├─ _sync_shapes()  or  _sync_shapes_vel_color()   ← sardines
  ├─ _sync_shapes()  (sailfish)
  ├─ batch.draw()                                    ← one GL call
  ├─ Hud.draw()
  ├─ PopGraph.draw()    (only when shown)
  └─ ConfigMenu.draw()  (no-op when closed)
```

**Why three separate grids**: sardines query `_grid` for flocking (sailfish excluded so sardines don't align with them). `_sailfish_grid` is queried only by sardines for flee. Sailfish query `_grid` for COM pursuit. `_algae_grid` is queried only by algae for their own flocking; sardines and sailfish are excluded so algae drift independently.

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
- One `pyglet.shapes.Triangle` per sardine/sailfish agent, all in a single `pyglet.graphics.Batch`.
- One `pyglet.shapes.Circle` per algae agent in the same batch (reddish-brown `(170, 50, 20, 255)`, radius = `algae_size`).
- One GL draw call per frame regardless of population.
- `_sync_shapes()` (white mode) updates only triangle vertex positions — pure Python float math.
- `_sync_shapes_vel_color()` updates positions **and** `tri.color` for dynamic colour modes.
- `_sync_circle_positions()` updates only `circle.x / circle.y` for algae — no heading math needed.
- Mode transition (e.g. turning velocity colour off) triggers one O(N) colour-restore pass on the transition frame only — zero per-frame cost at steady state.

### Runtime config menu
- TAB toggles the overlay.  Simulation keeps running while open.
- Modal guard: `handle_key()` returns `True` when open — population controls cannot fire.
- **57 data rows** across 7 group sections + 1 Save row.  `_VISIBLE_ROWS = 16`; rows scroll automatically as you navigate with ↑/↓.  Scroll indicators (▲/▼) appear in the footer strip when off-screen rows exist.
  - Sections: **SARDINES** (movement, flocking, flee, color); **MOUSE PREDATOR**; **SAILFISH — FLOCKING** (kinematics, sf-to-sf flocking, color); **SAILFISH — HUNTING** (COM pursuit, individual targeting, vulnerability scoring, school-pressure/orbit); **SAILFISH — BURST** (burst trigger/duration/cooldown/multipliers); **ALGAE** (kinematics, flocking weights, respawn/ecology); **ECOLOGY** (sardine + sailfish reproduction/starvation).
- **Int field safety**: `sardine_eat_threshold` and `algae_max_count` are int fields in `SimConfig`.  `_adjust_value()` detects `isinstance(cur_typed, int)` and casts back to `int` before `dataclasses.replace()` to avoid a `TypeError` on the frozen dataclass.
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
| `D` | Toggle population graph overlay |
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
SARDINES
  Sardine speed      ←→ 10 / Shift 50     [50–500]
  Sardine force      ←→ 10 / Shift 50     [10–500]
  Sep weight         ←→ 0.1 / Shift 1.0  [0–10]
  Ali weight         ←→ 0.1 / Shift 1.0  [0–10]
  Coh weight         ←→ 0.1 / Shift 1.0  [0–10]
  Noise weight       ←→ 0.01 / Shift 0.1 [0–0.5]
  Sep radius         ←→ 2 / Shift 20      [5–100]
  Ali radius         ←→ 2 / Shift 20      [10–150]
  Coh radius         ←→ 2 / Shift 20      [10–150]
  Flee weight        ←→ 0.5 / Shift 5     [0–30]
  Flee radius        ←→ 5 / Shift 50      [20–300]
  Color mode         ←→ cycles 7 sardine modes
  Vel heading wt     ←→ 0.1 / Shift 0.5  [0–2]
  Vel speed wt       ←→ 0.1 / Shift 0.5  [0–2]
  Vel turn wt        ←→ 0.1 / Shift 0.5  [0–2]
  Vel smoothing      ←→ 0.05 / Shift 0.1 [0–0.99]
MOUSE PREDATOR
  Mouse predator     ←→ / Enter           ON/OFF
  Mouse radius       ←→ 5 / Shift 50      [20–400]
  Mouse flee wt      ←→ 0.1 / Shift 0.5  [0–5]
SAILFISH — FLOCKING
  Sailfish speed     ←→ 10 / Shift 50     [50–600]
  Sailfish force     ←→ 10 / Shift 50     [10–600]
  SF sep weight      ←→ 0.1 / Shift 1.0  [0–10]
  SF sep radius      ←→ 2 / Shift 20      [5–200]
  SF ali weight      ←→ 0.1 / Shift 1.0  [0–10]
  SF ali radius      ←→ 5 / Shift 50      [20–400]
  SF coh weight      ←→ 0.1 / Shift 1.0  [0–10]
  SF coh radius      ←→ 5 / Shift 50      [20–400]
  Color mode         ←→ cycles 5 sailfish modes
SAILFISH — HUNTING
  Pursuit wt         ←→ 0.1 / Shift 1.0  [0–10]
  Pursuit radius     ←→ 5 / Shift 50      [50–400]
  Acquire radius     ←→ 5 / Shift 50      [20–400]
  Drop radius        ←→ 5 / Shift 50      [20–600]
  Lock time          ←→ 0.5 / Shift 1.0  [0–30]   (s; lock expires after this)
  Target wt          ←→ 0.1 / Shift 1.0  [0–10]
  Tgt dist wt        ←→ 0.1 / Shift 0.5  [0–5]    (vulnerability scoring weight)
  Tgt iso wt         ←→ 0.1 / Shift 0.5  [0–5]
  Tgt spd wt         ←→ 0.1 / Shift 0.5  [0–5]
  Pressure str       ←→ 0.1 / Shift 0.5  [0–5]    (school-pressure orbit force strength)
  Pressure dens      ←→ 1 / Shift 5       [1–100]  (min sardines visible to activate orbit)
SAILFISH — BURST
  Burst trig r       ←→ 5 / Shift 50      [10–300]
  Burst duration     ←→ 0.1 / Shift 0.5  [0–5]    (s)
  Burst cooldown     ←→ 0.5 / Shift 1.0  [0–15]   (s)
  Burst spd mult     ←→ 0.05 / Shift 0.1 [1–3]    (×max_speed during burst)
  Burst frc mult     ←→ 0.1 / Shift 0.5  [1–4]    (×max_force during burst)
ALGAE
  Algae speed        ←→ 1 / Shift 5       [1–100]
  Algae align wt     ←→ 0.1 / Shift 0.5  [0–5]
  Algae sep wt       ←→ 0.1 / Shift 0.5  [0–5]
  Algae respawn      ←→ 2 / Shift 5       [0–200]  (algae/s; pauses at max)
  Algae max          ←→ 50 / Shift 200    [100–5000] (int soft cap)
  Algae eat r        ←→ 1 / Shift 5       [4–50]   (sardine↔algae contact radius px)
ECOLOGY
  Sardine repro      ←→ 1 / Shift 5       [0–20]   (int; algae eaten before sardine splits)
  Sardine starve     ←→ 1 / Shift 10      [0–300]  (s; 0 = disabled)
  Sailfish eat r     ←→ 1 / Shift 5       [10–80]  (sailfish↔sardine contact radius px)
  Sailfish repro     ←→ 1 / Shift 5       [0–75]   (int; sardines eaten before sailfish splits)
  Sailfish starve    ←→ 5 / Shift 30      [0–600]  (s; 0 = disabled)
  SF starvation      ←→ / Enter           ON/OFF
  SF repro           ←→ / Enter           ON/OFF
  Save to file       ←→ / Enter / S
```

**Mouse predator** (left-click/drag to activate):
- Hold left mouse button → sardines within `mouse_predator_radius` flee the cursor
- Release → effect stops immediately
- A faint translucent red circle shows the danger radius while active
- Toggle, radius, and flee-weight tunable via config menu (MOUSE PREDATOR section)

**Note**: `sardine_initial_hunger_time` and `sailfish_initial_hunger_time` are intentionally
omitted from the menu.  `sailfish_initial_hunger_time` is kept only for JSON backwards-compatibility
— the `Sailfish` constructor always sets `hunger_timer = 0.0` regardless of its value.
`sardine_initial_hunger_time` still controls the sardine initial hunger on construction.

### Dynamic window resize
- Dragging the window or pressing F fires `on_resize(w, h)`.
- Flow: `simulation.resize(w, h)` → updates `SimConfig`, agent dims, both spatial hashes, positions; then `renderer.on_resize(w, h)` → repositions HUD; then `menu.on_resize(w, h)` → rebuilds menu shapes; then `renderer.reset(...)` → fully rebuilds all shape maps.
- Minimum guard: `width < 200 or height < 150` → skip to prevent degenerate state.
- `renderer.reset()` is called on every `on_resize` to ensure shape-map consistency after geometry changes (resize events are rare, so the O(N) rebuild is acceptable).

### Renderer shape-cache safety
- Each entity type has an identity→shape map (`_algae_shape_map`, `_sardine_shape_map`, `_sailfish_shape_map`) keyed by `id()` of the Python boid object.
- **macOS reentrant-idle crash (fixed)**: On macOS, fullscreen/resize events call `setFrameSize_` → `app.event_loop.idle()` in a reentrant way inside the Cocoa event loop. This causes `Simulation.update()` to run a second time before `Renderer.draw()` has processed the side channels (`_algae_born`, `_algae_eaten_ids`, etc.) from the first `update()` call. The second `update()` **clears** `_algae_born`, so algae spawned in the first update land in `simulation.algae` but never get circles added to `_algae_shape_map`. When `draw()` then hits the `eaten_ids`-triggered rebuild, `_algae_shape_map[id(a)]` raised `KeyError` on those algae, causing a crash and freeze.
- **Fix**: Two defences applied together:
  1. **Birth guard**: before creating a circle/triangle for a born agent, check `if id(agent) not in shape_map` — prevents duplicate shapes when `_algae_born`/`_sardine_born` are processed twice.
  2. **Lazy creation in rebuild paths**: replace direct `shape_map[id(a)]` indexing with `.get()` + fallback shape creation — any agent in `simulation.algae`/`sardines`/`sailfish` without a cached shape gets one allocated on the fly, so no `KeyError` can propagate.
  3. **`renderer.reset()` on `on_resize`**: guarantees a perfectly clean state after fullscreen/resize transitions; the lazy-creation path (above) handles any side-channel data that arrives in the next `draw()` call after the reset.

---

## 5. Configuration System

### Defaults
All defaults live in `config/config.py` → `SimConfig` frozen dataclass. This is the canonical baseline.

### Key config fields

| Field | Default | Notes |
|---|---|---|
| `num_sardines` | **300** | Reduced from 900; synced to live count after add/remove |
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
| `weight_sailfish_separation` | 3.0 | Sailfish-to-sailfish; stronger than sardine sep |
| `weight_sailfish_alignment` | 0.5 | Mild heading match between sailfish |
| `weight_sailfish_cohesion` | 0.0 | Off by default; pursuit handles grouping |
| `radius_sailfish_separation` | 80.0 | px; ~2 body-lengths at sailfish_size=12 |
| `radius_sailfish_alignment` | 150.0 | px |
| `radius_sailfish_cohesion` | 150.0 | px |
| `sardine_color_mode` | `"white"` | 7 modes |
| `sailfish_color_mode` | `"default"` | 5 modes |
| `vel_color_heading_weight` | 1.0 | |
| `vel_color_speed_weight` | 0.5 | |
| `vel_color_turn_weight` | 1.0 | |
| `vel_color_smoothing` | 0.85 | EMA coefficient |
| `show_hud` | True | |
| `boundary` | `"wrap"` | Toroidal; `"clamp"` for reflective |
| `background_color` | `(0,20,50,255)` | Deep-water blue |
| `num_algae` | **800** | Increased from 500; initial algae population |
| `algae_respawn_rate` | **8.0** | New algae per simulated second; 0.0 = disabled |
| `algae_max_count` | **1200** | Soft cap; respawn pauses when `len(algae) >= this` |
| `algae_max_speed` | 25.0 | px/s (~6× slower than sardines) |
| `algae_max_force` | 12.0 | px/s²; force/speed ≈ 0.48 — gentle arc |
| `weight_algae_separation` | 0.3 | Slightly stronger than cohesion |
| `weight_algae_alignment` | 1.5 | Dominant — drives current-drift feel |
| `weight_algae_cohesion` | 0.1 | Very weak — prevents tight schooling |
| `weight_algae_noise` | 0.05 | Low — smooth drift, not chaotic; fires even when isolated |
| `radius_algae_separation` | 18.0 | px; sep:coh = 1:3 (prevents clumping) |
| `radius_algae_alignment` | 55.0 | px; == coh_radius so all neighbours align |
| `radius_algae_cohesion` | 55.0 | px |
| `algae_cell_size` | 60.0 | px; > both radii for clean 3×3 ring lookup |
| `algae_eat_radius` | 14.0 | px; sardine–algae contact detection radius |
| `sardine_eat_threshold` | **3** | Reduced from 5; algae eaten before sardine reproduces; 0 = disabled |
| `sailfish_eat_radius` | **25.0** | Widened from 20 px; sailfish–sardine contact detection radius; one kill per sailfish per tick |
| `algae_size` | 4.0 | px (circle radius) |
| `sailfish_starvation_enabled` | `True` | Runtime toggle; when `False` the starvation pass is skipped entirely |
| `sailfish_reproduction_enabled` | `True` | Runtime toggle; when `False` the reproduction pass is skipped entirely |
| `mouse_predator_enabled` | `True` | Toggle; when `False` sardines ignore the cursor entirely |
| `mouse_predator_radius` | `100.0` | px; sardines within this distance of the cursor flee (same default as `radius_flee`) |
| `mouse_predator_flee_weight` | `1.0` | Multiplier on `weight_flee` for mouse-flee force; 1.0 = same intensity as a real sailfish |

### Override file
- `boids_overrides.json` (gitignored) — partial override dict.
- Loaded at startup and merged over defaults via `**load_overrides()`.
- Saved via S key in the config menu (atomic write).
- `window_width`/`window_height` always re-read from live window, never from file.

### `apply_config()` propagation
`Simulation.apply_config(new_config)` propagates a new `SimConfig` to all live agents:
- Updates `agent._config` for all.
- Updates `Boid.max_speed`, `max_force`, `min_speed` instance attrs (read every frame by `_steer_apply_xy` / `_clamp_speed`).
- Refreshes `_sep_r2`, `_ali_r2` squared-radius caches on sardines and algae.
- Does **not** change world geometry (that's `resize()`).

---

## 6. Current State

### What works
- All modules import cleanly; no circular imports.
- 268 tests pass (`python -m unittest tests.tests`).
- `python main.py` opens a resizable window; all features operational.
- **Sardine flocking**: separation / alignment / cohesion / noise — stable, tuned, and numerically bounded.
- **Sailfish pursuit + sardine flee**: sailfish steer toward sardine COM; sardines flee on detection and scatter (cohesion suppressed while fleeing); three-grid architecture clean.
- **Algae drift**: 800 algae with alignment-dominant flocking; rendered as reddish-brown circles (`(170, 50, 20, 255)`, radius 4 px) behind sardines/sailfish; respawn accumulator-based up to `algae_max_count` (1200).
- **Three-level food chain**: sardines eat algae on contact; sardines reproduce when fed; sailfish eat sardines; both species starve if unfed; all ecology params configurable live.
- **Reproduction**: sardines and sailfish both reproduce via `food_counter >= threshold`; offspring spawn beside parent at perpendicular offset; excess counter carries forward. HUD shows cumulative births for each.
- **Starvation**: configurable per species; `hunger_timer` incremented each frame; every new sailfish (startup, `add_sailfish()`, reproduction) starts with `hunger_timer = 0.0` — equivalent to "just eaten" — so no spawn path can produce an immediately-dying fish; `starvation_time=0.0` disables mechanic. **`sailfish_starvation_enabled`** toggle allows instant on/off without clearing the numeric threshold. (`sailfish_initial_hunger_time` remains in `SimConfig` for JSON backwards-compatibility but is no longer read by the constructor.)
- **Sailfish reproduction toggle**: **`sailfish_reproduction_enabled`** bool allows disabling reproduction independently of the numeric threshold.
- **HUD ecosystem metrics**: 7 lines refreshed at 4 Hz — FPS/timing, live counts per species, algae eaten/spawned, sardine born/starved/eaten + avg hunger, sailfish starved + avg hunger + `[STARV OFF]`/`[REPRO OFF]` indicators when toggles are off, world size, control hints.
- **Live resize and fullscreen**: world geometry updates in-place; algae positions wrapped; all three grids resized.
- **Config menu**: 57 data rows across 7 sections (SARDINES, MOUSE PREDATOR, SAILFISH — FLOCKING, SAILFISH — HUNTING, SAILFISH — BURST, ALGAE, ECOLOGY) + Save; scrollable panel (16 visible rows, ▲/▼ indicators); int fields (`sardine_eat_threshold`, `algae_max_count`) preserved through `apply_config`; save/load round-trip lossless.
- **Velocity-based colouring**: 7 sardine modes, 5 sailfish modes; cosine cyclic mapping eliminates heading-wrap flicker.
- **Config persistence**: `boids_overrides.json` atomic write; population counts synced to live state on save.
- **Mouse predator**: left-click/drag scatters sardines using existing flee mechanics; `_MouseStimulus` shared object on `Simulation`; read lock-free by all sardines via duck-typed ref; faint red danger-radius circle drawn outside batch; fully configurable via menu; 16 tests in `TestMousePredator`.


### Partially complete
- `num_sardines` / `num_sailfish` not editable via the config menu (only via ↑/↓/←/→ keys).

### Known limitations
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
| **Scalar element-wise Euler integration** | `boid.py` | Replaced `v += a*dt` / `p += v*dt` (each creates a `(2,)` float64 temp array) with `v[0] += a[0]*dt` element ops — saves ~2 temp array allocs/agent/frame (~240 000/s at 2 000 agents×60 fps) |
| **`_boundary_wrap` bool cache on `Boid`** | `boid.py`, `simulation.py` | Replaces `self._config.boundary == "wrap"` (2 attr lookups + string comparison) with a single cached bool per agent per frame; propagated by `apply_config()` |
| **Cached `other.position`/`other.velocity` in neighbour loops** | `sardine.py`, `algae.py`, `sailfish.py` | `opos = other.position` / `ovel = other.velocity` cached once per neighbour halves attribute lookups in the inner loop and removes redundant `float()` wrappers on numpy scalars |
| **`_EMPTY_LIST` sentinel in `query()` / `query_position()`** | `spatial_hash.py` | Shared module-level `[]` constant returned for empty queries instead of allocating a fresh list — eliminates ~1 500 list allocs/frame when sardines query the sparse sailfish grid |
| **Persistent `_respawn_rng` on `Simulation`** | `simulation.py` | `np.random.default_rng()` called once at init; reused across all `_spawn_algae_batch()` calls |
| **`map(id, …)` in ecology set updates** | `simulation.py` | Replaced generator expressions with C-level `map(id, …)` in all four `set.update()` ecology-pass calls |
| **`query()` fast path — skip `visited` set** | `spatial_hash.py` | When `nc > 2·span` and `nr > 2·span` (all real worlds), the `(2·span+1)²` cell keys are guaranteed distinct — no deduplication set needed; eliminates ~4 200 `set()` allocs/frame + 9+ hash ops per query at 1 500 sardines+algae |
| **`agent.position` cached in `insert()`** | `spatial_hash.py` | One `LOAD_ATTR` per insert instead of two; saves N_total attr lookups per frame during grid rebuild |
| **`other.position` cached in `query()` inner loop** | `spatial_hash.py` | `opos = other.position` once per candidate boid — halves attr lookups in the distance-check inner loop across all three grids |
| **`self.velocity`, `self.max_force`, `self.acceleration` cached in `_steer_apply_xy`** | `boid.py` | 3 attributes cached as locals; removes 2–3 repeated `LOAD_ATTR` + array-index ops per call; `float()` wrappers on velocity components removed (numpy scalar arithmetic suffices); with 3–5 calls per sardine per frame: ~30 000 attr lookups/frame saved at 1 500 sardines |
| **Grid `.insert` methods bound as locals before insert loops** | `simulation.py` | `_insert = self._grid.insert` saved once; inner loop avoids `self._grid` + `.insert` chain per iteration — saves 2 × N_total LOAD_ATTR per frame during grid rebuild |
| **`_gx/_gy/_gvx/_gvy` pre-cached Python floats on every `Boid`** | `spatial_hash.py:insert()`, `boid.py` | Written once per agent per frame during grid rebuild; eliminates `other.position` LOAD_ATTR + numpy subscript + numpy scalar dispatch in ALL query inner loops and species flocking loops — dominant cost reduction at scale |
| **`query()` / `query_position()` inner loops use `other._gx/_gy`** | `spatial_hash.py` | All candidate-pair distance checks now use Python-float arithmetic; eliminated ~143 560 numpy scalar ops/frame at 1 500 sardines |
| **`sardine._compute_forces` and `algae._compute_forces` flocking loops use `_gx/_gy/_gvx/_gvy`** | `sardine.py`, `algae.py` | Eliminated per-neighbor `other.position` LOAD_ATTR + `float(opos[0/1])` and `other.velocity` LOAD_ATTR + `float(ovel[0/1])` — ~30 000 + 27 000 expensive ops/frame reduced to simple LOAD_ATTR returning Python float |
| **Inline `_steer_apply_xy` in `sardine._compute_forces` and `algae._compute_forces`** | `sardine.py`, `algae.py` | 4 (sardine) + 4 (algae) Python function calls per agent per frame eliminated; shared `_vx/_vy/_ms/_mf/_acc` locals cache velocity + force limits once for all steer directions per fish per frame |
| **Inline cell traversal in sardine flocking** | `sardine.py` | `query()` replaced by direct `_grid._cells` loop; eliminates `results=[]` allocation + N `list.append` calls + a full second pass that re-computed dx/dy/d_sq for every accepted neighbour — list.append calls dropped from 22M/300-frames to 2.4M/300-frames |
| **Inline cell traversal in algae flocking** | `algae.py` | Same as sardine: single-pass accumulation; no materialized neighbour list |
| **Inline cell traversal in sardine flee** | `sardine.py` | Sailfish-grid query replaced by direct `_sailfish_grid._cells` loop; eliminates ~300k query() calls per 300-frame benchmark that previously built empty lists when no sailfish nearby |
| **`query_first()` for sailfish eating** | `spatial_hash.py`, `simulation.py` | Sailfish one-kill-per-tick now stops at first match; saves building a full list just to take `[0]` |
| **`_coh_r2 / _flee_r2` cached on sardine/algae** | `sardine.py`, `algae.py`, `simulation.py` | Avoids `radius ** 2` per sardine per frame; updated in `apply_config()` |
| **`_coh_span / _flee_span` cached on sardine/algae** | `sardine.py`, `algae.py`, `simulation.py` | Avoids `max(1, ceil(radius / cell_size))` inside the hot inline traversal (2× per sardine per frame); updated in `apply_config()` |
| **`_s_pos/_s_vel/_s_acc` batch arrays on `Simulation`** | `simulation.py` | Phase 4A: contiguous `(N,2)` float64 arrays; populated each frame from `_gx/_gy/_gvx/_gvy`; 2× capacity-growth policy; sets `sardine._idx` per agent |
| **`_batch_integrate_sardines(dt)` vectorised Euler** | `simulation.py` | Phase 4B: replaces N per-sardine `boid.update()` calls; gather acc → vectorised vel+clamp+pos+boundary → scatter back; eliminates per-sardine `_clamp_speed`+`_apply_boundary` dispatch |

| **`_batch_compute_sardine_forces()` — Phase 4C** | `simulation.py` | Replaces per-sardine `_compute_forces()` Python loop for N≥500 with numpy cell-bucket vectorisation: (a) vectorised N×S sardine–sailfish flee, (b) per-cell (M×K) pairwise sep/ali/coh via numpy broadcasting, (c) single `np.random.normal()` call for all sardine noise. Writes directly to `_s_acc` — no gather loop in `_batch_integrate_sardines` needed. For N<500 falls back to per-sardine loop (numpy overhead exceeds Python loop cost at small N). |

### Scaling limits
- **Post-Phase 4C ceiling**: new dominant cost is `_batch_compute_sardine_forces` cell-bucket Python overhead (dict.get, list.append building neighbour lists per cell). At 1 500 sardines: **11.6 ms/frame (86 fps-equiv)**; at 2 000 sardines: **14.9 ms/frame (67 fps-equiv)** — previously 26 ms and 43 ms.
- **Phase 4C speedup**: 2.27× at 1 500 sardines, 2.89× at 2 000 sardines. Light (300 sardines) unchanged (uses per-sardine fallback path).
- **Next bottleneck**: `spatial_hash.query()` (18% of heavy profile) called by sailfish `_compute_forces()` and ecology pass; also per-cell `neigh_list` construction in `_batch_compute_sardine_forces`. Potential Phase 4D: pre-build CSR cell-index array once per frame for O(1) neighbour gather with no per-cell Python overhead.
- **Viable population**: 3 000+ sardines now plausible at ~25–30 ms/frame (40+ fps-equiv). Previous limit was ~1 500 sardines at 45 fps.

---

## 8. TODO / Next Steps

### High priority
1. **`load_overrides` edge-case test coverage** — corrupt JSON, unknown fields, colour-mode validation. `TestConfigMenuEcosystem` partially covers config persistence.

### Medium priority
2. **`num_sardines` / `num_sailfish` as editable menu rows** — they already persist correctly. Needs population-adjustment logic on value change (not trivial; separate from the slider model).
3. **Cross-field constraint enforcement** — clamp `radius_separation < radius_alignment` and `radius_alignment ≤ radius_cohesion` inside `_adjust_value()`.

### Future
4. **Phase 4D — CSR cell-index pre-build** — pre-construct a flat sardine-index array + per-cell offsets once per frame during the grid-rebuild pass so `_batch_compute_sardine_forces()` can gather neighbour rows with a single index slice instead of the per-cell `neigh_list` Python loop. Expected: another 20–30% at 2 000+ sardines.
5. **Per-sardine danger memory** — smooth cohesion restoration after escape (timer-based rather than instantaneous).
6. **HiDPI / Retina support** — `pyglet.options['hidpi'] = True` before `Window()`.

---

## 9. References

- **[memory/ARCHIVE.md](memory/ARCHIVE.md)** — Phase-by-phase build history, implementation archaeology, subclass interface contract, algorithmic complexity reference, historical tuning table, Phase 8 ecosystem plan, and detailed optimization pass writeups (Phases 1–4C).
- **[tests/tests.py](tests/tests.py)** — 268 tests across 23 suites: `TestVec2`, `TestBoidKinematics`, `TestSpatialHash`, `TestQueryFirst`, `TestInlineTraversalCorrectness`, `TestSimulationIntegrity`, `TestPhase2`, `TestAlgaeKinematics`, `TestEcologyContactDetection`, `TestSardineReproduction`, `TestSailfishEcology`, `TestSardineStarvation`, `TestSailfishStarvation`, `TestAlgaeRespawn`, `TestHudEcosystem`, `TestConfigMenuEcosystem`, `TestSailfishReproduction`, `TestSailfishSpawnInit`, `TestSailfishToggles`, `TestMousePredator`, `TestPhase4Batching`, `TestSailfishFlocking`, `TestPhase4CBatchedForces`.
- **[tests/bench.py](tests/bench.py)** — Headless benchmark. Usage: `python -m tests.bench [--profile] [--test] [--frames N]`. Scenarios: light (300s), medium (800s), heavy (1500s), extreme (2000s).
- **[config/config.py](config/config.py)** — Authoritative parameter definitions with inline comments.

---

> Detailed implementation notes for Optimization Passes (Phases 1–4C) have been migrated to [memory/ARCHIVE.md](memory/ARCHIVE.md).
