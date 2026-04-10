# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

> This file is the authoritative source of truth for the current system. All AI agents must read this first before making changes and should update the log after task completion.
> **Working reference for active development.** Optimised for quick orientation, not history.
> Historical build notes, phase changelists, and implementation archaeology → [ARCHIVE.md](ARCHIVE.md)

---

## Log Maintenance
- Cleaned and pruned outdated content (April 2026)
- Archived Phase 8 ecosystem plan and verbose stage notes to ARCHIVE.md
- Removed `[Phase X]` annotations — all described features are current
- Fixed stale test count (169 → 185)
- Ensured log reflects current system only
- Added sailfish starvation/reproduction toggles (April 2026)
- Fixed sailfish spawn-init: hunger_timer now always 0 on every spawn path; removed clamping logic; tests updated (April 2026)
- Added mouse-as-predator interaction: left-click/drag scatters sardines; visual indicator; config menu exposed; 16 new tests (April 2026)
- Optimization Phase 4A–4B (April 2026): batch sardine state arrays + vectorised integration; `boid.update()` calls for sardines eliminated; `_batch_integrate_sardines()` added; 15 new tests; test count 231 → 246.
- Optimization Phase 4C (April 2026): numpy cell-bucket batched force computation; per-sardine `_compute_forces()` Python loop eliminated for N≥500; 9 new tests; test count 246 → 255.
- Sailfish-to-sailfish flocking (April 2026): separation/alignment/cohesion forces on sailfish; new `SAILFISH FLOCKING` config menu section; 6 new `SimConfig` fields; `Sailfish._compute_forces()` restructured; all 3 spawn paths updated; 13 new tests; test count 259.
- Sailfish individual prey targeting (April 2026): two-stage hunting where sailfish chase school COM at long range, then lock onto a specific sardine once one enters `sailfish_target_acquire_radius` and pursue it with predictive interception. Locks expire after `sailfish_target_lock_time` or when the target is eaten/starved/exceeds `sailfish_target_drop_radius`. `Sardine.is_alive` flag added so sailfish detect target death in O(1). 4 new `SimConfig` fields; 4 new menu rows in SAILFISH section; `_target_sardine` / `_target_lock_timer` added to `Sailfish`; lock timer decremented alongside `hunger_timer` in simulation.py ecology pass; `apply_config` clears stale locks. No new tests (behavioral change; existing 259 tests all pass).
- Sailfish burst mode + shared-target reduction (April 2026): when a locked sardine enters `sailfish_burst_trigger_radius`, the sailfish fires a short burst—temporarily multiplying `max_speed` by `sailfish_burst_speed_mult` and `max_force` by `sailfish_burst_force_mult` for `sailfish_burst_duration` seconds, then cooling down for `sailfish_burst_cooldown` seconds. `Sailfish.update()` override applies the speed/force caps before delegating to `Boid.update()` so `_clamp_speed()` enforces them naturally. Shared-target reduction: during acquisition scan, second-best candidate (`alt_s`) is tracked; if a nearby sailfish already holds the best candidate as its target, this sailfish locks onto `alt_s` instead (soft preference, fully local, no global coordination). 5 new `SimConfig` fields; 5 new menu rows in SAILFISH section; `_burst_timer`/`_burst_cooldown_timer` state added to `Sailfish.__init__`; burst timers decremented in simulation.py ecology pass alongside `_target_lock_timer`; `apply_config` resets burst timers. No new tests (behavioral extension; existing 259 tests all pass).
- Population graph overlay (April 2026): `D` key toggles a 480×240 px semi-transparent graph panel (bottom-right); sardine/sailfish/algae populations plotted vs sim time; `collections.deque(maxlen=3600)` history sampled at ~1 Hz prevents unbounded growth; `PopGraph` class in `renderer.py`; `sim_time` and `pop_history` fields added to `Simulation`; no new tests needed (rendering-only overlay with trivial data collection).
- Population graph compatibility fix (April 2026): `pyglet.shapes.Line` in pyglet 2.1.x uses `thickness=` not `width=`; passing a raw `width=` kwarg caused a `TypeError` crash on first `D` keypress. Fix: renamed to `thickness=1.5` and switched both `Line` and `Rectangle` to accept full RGBA 4-tuples directly (eliminating separate `.opacity` assignments). No behaviour change; line weight and alpha are identical.
- Population graph performance + size overhaul (April 2026): enabling the graph caused a severe fps drop because the original `draw()` allocated up to 3×3600 `pyglet.shapes.Line` objects and 4 `pyglet.text.Label` objects **every frame** at 60 Hz, then immediately discarded them. Fix: (1) all line segments are now placed in a dedicated `pyglet.graphics.Batch` so the full graph submits as a single GL draw call; (2) geometry is rebuilt at most every 0.25 s (~4 Hz) via `time.monotonic()` throttle — background rect, labels, and tick-label text are cached between rebuilds; (3) displayed history is uniformly downsampled to ≤ 200 points; (4) panel enlarged from 480×240 to 680×340 px for readability. Remaining trade-off: geometry rebuild still allocates ~600 shapes at 4 Hz; visual updates lag up to 0.25 s behind the simulation, which is imperceptible at 1 Hz sampling.

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

- **Runtime**: `python main.py` — resizable window at screen resolution, 60 fps target.
- **Tests**: `python -m tests.tests` — 259 tests, ~0.9 s.

### Current capabilities
- Sardines flock using Reynolds separation / alignment / cohesion + small noise term.
- Sailfish hunt sardine schools by steering toward the local centre of mass.
- Sardines detect nearby sailfish, flee with full-force escape, and scatter (cohesion suppressed while fleeing).
- **Algae drift** slowly across the world; alignment-dominant flocking gives a water-current feel.
- **Sardine feeding**: sardines eat algae on contact (`algae_eat_radius`); eaten algae vanish instantly, sardine `food_counter` and `hunger_timer` tracked.
- **Sardine reproduction**: when `food_counter` reaches `sardine_eat_threshold`, the sardine splits — one offspring spawned beside it (perpendicular offset = `radius_separation`), inheriting parent velocity; `food_counter` decremented by threshold (not reset to 0) so leftover progress carries forward.
- **Sailfish predation**: each sailfish eats the nearest sardine within `sailfish_eat_radius` each frame (one per sailfish per tick); the sardine is removed immediately; sailfish `food_counter` and `hunger_timer` track feeding state. HUD shows cumulative predation count (`Pred ↑N`).
- **Sailfish individual targeting**: when a sardine enters `sailfish_target_acquire_radius`, the sailfish locks onto that specific fish and pursues it with predictive interception (`predicted_pos = target.pos + target.vel * t`). At long range, sailfish still chase the school COM. Lock expires after `sailfish_target_lock_time` seconds or when the target is eaten/leaves `sailfish_target_drop_radius`.
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
│   └── tests.py              # 185 tests across 15 suites
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
- **49 rows** across 9 group sections + 1 Save row.  Panel height is fixed at 418 px (`_VISIBLE_ROWS = 16`); rows scroll automatically as you navigate with ↑/↓.  Scroll indicators (▲/▼) appear in the footer strip when off-screen rows exist.
  - Sections: FLOCKING, PREDATION, ALGAE, VISUAL, MOUSE, SAILFISH FLOCKING, SAILFISH, ECOSYSTEM, SAILFISH ECOLOGY.
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
ECOSYSTEM
  Algae respawn      ←→ 0.5 / Shift 5.0  [0–50]   (algae/s; live-effective via self.config)
  Algae max          ←→ 50 / Shift 200    [100–5000] (int; respawn pauses at cap)
  Algae eat r        ←→ 1 / Shift 5       [4–50]   (sardine↔algae contact radius px)
  Repro thresh       ←→ 1 / Shift 5       [0–20]   (int; algae eaten before sardine splits)
  Sardine starve     ←→ 1 / Shift 10      [0–300]  (s; 0 = disabled)
  Sailfish eat r     ←→ 1 / Shift 5       [10–80]  (sailfish↔sardine contact radius px)
  Sailfish starve    ←→ 5 / Shift 30      [0–600]  (s; 0 = disabled)
  Sailfish repro     ←→ 1 / Shift 5       [0–20]   (int; sardines eaten before sailfish splits)
  SF starvation      ←→ / Enter           ON/OFF   (bool; disables starvation pass when OFF)
  SF repro           ←→ / Enter           ON/OFF   (bool; disables reproduction pass when OFF)
MOUSE PREDATOR
  Mouse predator     ←→ / Enter           ON/OFF   (bool; enables/disables mouse-as-predator feature)
  Mouse radius       ←→ 5 / Shift 50      [20–400] (px; sardine detection radius around cursor)
  Mouse flee wt      ←→ 0.1 / Shift 0.5  [0–5]    (multiplier on weight_flee; 1.0 = same as sailfish)
ALGAE DRIFT
  Algae speed        ←→ 1 / Shift 5       [1–100]  (propagated immediately via apply_config)
  Algae align wt     ←→ 0.1 / Shift 0.5  [0–5]
  Algae sep wt       ←→ 0.1 / Shift 0.5  [0–5]
VISUAL
  Sardine color      ←→ cycles 7 modes
  Sailfish color     ←→ cycles 5 modes
  Vel heading wt     ←→ 0.1 / Shift 0.5  [0–2]
  Vel speed wt       ←→ 0.1 / Shift 0.5  [0–2]
  Vel turn wt        ←→ 0.1 / Shift 0.5  [0–2]
  Vel smoothing      ←→ 0.05 / Shift 0.1 [0–0.99]
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
- 246 tests pass (`python -m unittest tests.tests`).
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
- **Config menu**: 40 rows across 8 sections; scrollable panel (16 visible rows, ▲/▼ indicators); int fields (`sardine_eat_threshold`, `algae_max_count`) preserved through `apply_config`; save/load round-trip lossless.
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

### Phase 9 (future)
4. **Phase 4D — CSR cell-index pre-build** — pre-construct a flat sardine-index array + per-cell offsets once per frame during the grid-rebuild pass so `_batch_compute_sardine_forces()` can gather neighbour rows with a single index slice instead of the per-cell `neigh_list` Python loop. Expected: another 20–30% at 2 000+ sardines.
5. **Nearest-prey targeting** — individual pursuit rather than group COM; biologically more accurate for a single sailfish.
6. **Sailfish burst acceleration** — short high-speed sprint with cooldown.
7. **Per-sardine danger memory** — smooth cohesion restoration after escape (timer-based rather than instantaneous).
8. **HiDPI / Retina support** — `pyglet.options['hidpi'] = True` before `Window()`.

---

## 9. References

- **[ARCHIVE.md](ARCHIVE.md)** — Phase-by-phase build history, implementation archaeology, subclass interface contract, algorithmic complexity reference, historical tuning table, Phase 8 ecosystem plan.
- **[tests/tests.py](tests/tests.py)** — 255 tests across 22 suites: `TestVec2`, `TestBoidKinematics`, `TestSpatialHash`, `TestQueryFirst`, `TestInlineTraversalCorrectness`, `TestSimulationIntegrity`, `TestPhase2`, `TestAlgaeKinematics`, `TestEcologyContactDetection`, `TestSardineReproduction`, `TestSailfishEcology`, `TestSardineStarvation`, `TestSailfishStarvation`, `TestAlgaeRespawn`, `TestHudEcosystem`, `TestConfigMenuEcosystem`, `TestSailfishReproduction`, `TestSailfishSpawnInit`, `TestSailfishToggles`, `TestMousePredator`, `TestPhase4Batching`, `TestPhase4CBatchedForces`.
- **[config/config.py](config/config.py)** — Authoritative parameter definitions with inline comments.

---

## 10. Optimization Pass — Phase 1 (April 2026)

### Context

Full performance analysis performed against the codebase at ~300 sardines / 800 algae / 8 sailfish.
The stated scaling limit was ~1 500 sardines before the Python per-object loop saturates.
Phase 1 targets only safe, incremental micro-optimizations — no architectural changes.

### Performance issues identified

| Issue | Category | Impact |
|---|---|---|
| `v += a*dt` / `p += v*dt` in `boid.update()` each allocate a `(2,)` float64 temp array | Allocation hotspot | High — every agent every frame |
| `self._config.boundary == "wrap"` per agent per frame (2 attr lookups + string comparison) | Redundant computation | Medium |
| `float(other.position[0])` + `float(other.position[1])` in inner neighbour loop (two separate `other.position` attribute lookups per neighbour) | Repeated attribute access | High — ~30 000 lookups/frame at 1 500 sardines |
| `float(other.velocity[0/1])` in alignment inner loop (same double-lookup pattern) | Repeated attribute access | Medium |
| `query()` allocates a fresh `list[]` even for empty results | Allocation hotspot | Medium — ~1 500 empty allocs/frame when sardines query sparse sailfish grid |
| `np.random.default_rng()` created on every `_spawn_algae_batch()` call (several times/second) | Object creation overhead | Low |
| `id(x) for x in set` generator expressions in 4 `set.update()` ecology calls | Minor overhead | Low |

### Optimizations implemented

#### 1. Scalar Euler integration — `sim/boid.py`

`velocity += acceleration * dt` creates an intermediate `(2,)` float64 array before adding in-place.
`position += velocity * dt` does the same. Replaced both with element-wise scalar ops:

```python
vel[0] += acc[0] * dt
vel[1] += acc[1] * dt
# …clamp…
pos[0] += vel[0] * dt
pos[1] += vel[1] * dt
acc[0] = 0.0; acc[1] = 0.0
```

Numpy scalar (`acc[0]`) × Python float → numpy scalar; no array allocated.
**Estimated impact**: ~2 heap allocs saved per agent per frame → ~240 000/s at 2 000 agents.

#### 2. Boundary mode bool cache — `sim/boid.py` + `sim/simulation.py`

Added `self._boundary_wrap: bool = (config.boundary == "wrap")` on `Boid.__init__`.
`_apply_boundary()` now tests `self._boundary_wrap` instead of `self._config.boundary == "wrap"`.
Propagated through `Simulation.apply_config()` so runtime boundary-mode changes still take effect.
**Estimated impact**: removes 2 attr-chain lookups + 1 string comparison per agent per frame.

#### 3. Neighbour-loop attribute caching — `sim/sardine.py`, `sim/algae.py`, `sim/sailfish.py`

In every per-neighbour inner loop, cache the accessed array:

```python
# Before (two separate attribute lookups):
dx = px - float(other.position[0])
dy = py - float(other.position[1])

# After (one lookup, two index ops, no float() conversion):
opos = other.position
dx = px - opos[0]
dy = py - opos[1]
```

Same pattern applied to `other.velocity` in the alignment branch.
**Estimated impact**: halves attribute lookups in the hottest inner loop — ~30 000 saved/frame at 1 500 sardines × 20 avg neighbours; removes `float()` overhead on numpy scalar indexing.

#### 4. Empty-list sentinel in `query()` / `query_position()` — `sim/spatial_hash.py`

Added module-level `_EMPTY_LIST: list = []`. Both query methods now return this shared constant instead of allocating a new `[]` when no neighbours are found. All callers only iterate or test truthiness — none mutate the returned list.
**Estimated impact**: eliminates up to ~1 500 empty-list allocations/frame when 1 500 sardines each query the sailfish grid and find no predators nearby.

#### 5. Persistent respawn RNG — `sim/simulation.py`

`self._respawn_rng = np.random.default_rng()` created once at `Simulation.__init__`.
`_spawn_algae_batch()` now reuses it instead of calling `np.random.default_rng()` every time.
**Estimated impact**: low but free — removes RNG object creation every few frames.

#### 6. `map(id, …)` in ecology set updates — `sim/simulation.py`

Replaced generator expressions `(id(x) for x in collection)` with `map(id, collection)` (C-level iteration) in all four `set.update()` calls in the ecology pass.
**Estimated impact**: negligible per call, free win.

### Expected cumulative gain

- **Allocation pressure**: ~3–5 fewer heap allocs per agent per frame (Euler temp arrays + empty query lists). At 2 000 agents × 60 fps, this is ~360 000–600 000 fewer small allocations/second, meaningfully reducing GC pause frequency.
- **CPU cost per frame**: attribute-lookup and `float()` savings in the inner neighbour loop are the dominant win, estimated at 5–10% frame-time reduction at 1 500 sardines.
- **Stable agent ceiling**: expected to improve from ~1 500 to ~1 700–1 800 sardines at 60 fps, depending on hardware.

### Tradeoffs / notes

- All changes are purely internal; no public API, behavioral semantics, or test expectations changed.
- `_boundary_wrap` must be kept in sync with `_config.boundary` — `apply_config()` now does this explicitly. Any future code path that hot-swaps a config without calling `apply_config()` would need to update this flag.
- `_EMPTY_LIST` sentinel is safe only as long as all `query()` callers remain read-only. A comment documenting this invariant has been added to `spatial_hash.py`.
- 214 tests pass unchanged after all changes.

---

## 11. Optimization Pass — Phase 2A (April 2026)

### Structural bottlenecks identified

| Bottleneck | File/Function | Category |
|---|---|---|
| `visited: set` allocated on **every** `query()` call — 4 200+/frame | `spatial_hash.py:query()` | Allocation hotspot |
| `other.position` accessed twice in `query()` inner loop (`[0]` then `[1]`) | `spatial_hash.py:query()` | Repeated attr access |
| `agent.position` accessed twice in `insert()` | `spatial_hash.py:insert()` | Repeated attr access |
| `self.velocity` accessed once per `_steer_apply_xy` call (called 3–5×/sardine/frame) | `boid.py:_steer_apply_xy()` | Repeated attr access |
| `self.max_force` used twice in `_steer_apply_xy` when clamping fires | `boid.py:_steer_apply_xy()` | Repeated attr access |
| `self.acceleration` accessed separately for `[0]` and `[1]` writes | `boid.py:_steer_apply_xy()` | Repeated attr access |
| `self._grid.insert(s)` — two LOAD_ATTR per insert call in the N-long rebuild loop | `simulation.py:update()` | Repeated attr chain per iteration |
| `float()` wrapper on `self.velocity[0/1]` inside `_steer_apply_xy` | `boid.py:_steer_apply_xy()` | Unnecessary conversion |

### Why a full numpy/ECS rewrite was not chosen

The per-agent overhead from attribute lookups and per-query allocations is
large enough to justify targeted structural fixes without a full rewrite.
Each change below touches a single tightly-scoped hot path and is provably
safe without altering any behavioral logic. A full numpy vectorization (Phase 3)
requires restructuring all agent data into flat arrays and is still the
long-term ceiling-breaker, but the Phase 2A wins compound with Phase 1 and
push the stable agent ceiling further at very low refactor risk.

### Changes implemented

#### 1. `spatial_hash.py` — Fast-path `query()` skipping `visited` set

**Why it helps**: In every normal simulation frame, `nc > 2·span` and
`nr > 2·span` (default world 1920×1080, cell_size 80 → nc=24, nr=14,
span=1). Under this condition the `(2·span+1)²` cell combinations are all
distinct after modular reduction, so no deduplication is needed.  
The old code always allocated `visited: set[tuple[int,int]]` and performed
`key in visited` + `visited.add(key)` — 9× per query — regardless.

**Change**: added a runtime branch on `nc > 2*span and nr > 2*span`:
- **Fast path**: iterate cells directly, no set.
- **Slow path**: original visited-set logic, only enters for tiny worlds (e.g. unit-test grids with `nr=2`).

**Impact**: eliminates ~4 200+ `set()` allocs/frame at 1 500+ agents
(sardines × flocking query + sardines × flee query + algae × algae query).

Applied to both `query()` and `query_position()`.

#### 2. `spatial_hash.py` — Cache `agent.position` / `other.position`

- `insert()`: `apos = agent.position` once → both `apos[0]` and `apos[1]` from the same cached reference.
- `query()` inner loops: `opos = other.position` once per candidate → halves `LOAD_ATTR` ops in the distance-check path.

#### 3. `boid.py` — Cache locals in `_steer_apply_xy`

Three attributes now loaded once per call instead of repeatedly:
- `vel = self.velocity` → `vel[0]`, `vel[1]` instead of `self.velocity[0]`, `self.velocity[1]`.
- `max_f = self.max_force` → used for both the `fmag > max_force` check and the `max_force / fmag` division.
- `acc = self.acceleration` → `acc[0] +=`, `acc[1] +=` instead of `self.acceleration[0/1] +=`.
- Removed `float()` wrappers around velocity components (numpy scalar arithmetic is correct without explicit conversion).

**Impact**: saves ~5 LOAD_ATTR + conversion ops per call × 7 500 calls/frame at 1 500 sardines = ~37 500 ops/frame.

#### 4. `simulation.py` — Bind grid insert methods as locals

```python
_insert = self._grid.insert
for s in self.sardines:
    _insert(s)
```

Saves `self._grid` + `.insert` chain lookups on every iteration of the
three rebuild loops. At N_total ≈ 3 500 agents, saves ~7 000 LOAD_ATTR/frame.

### Expected cumulative impact (Phase 1 + Phase 2A together)

- **Allocation pressure**: eliminated ~4 200 `set()` allocs/frame (Phase 2A) on top of Phase 1 wins.
- **Attribute lookup pressure**: ~45 000 fewer LOAD_ATTR ops/frame from Phase 2A alone at 1 500 sardines.
- **Estimated frame-time gain**: ~1.5–2.5 ms/frame at 1 500 sardines (combining all Phase 2A changes).
- **Stable agent ceiling**: estimated 1 900–2 100+ sardines at 60 fps (up from ~1 700–1 800 post Phase 1).

### Remaining bottlenecks after Phase 2A

| Bottleneck | Next action |
|---|---|
| `boid.update()` dispatches 2 Python method calls (`_clamp_speed`, `_apply_boundary`) every agent every frame | Phase 2B: inline these two methods into `update()` |
| `_steer_apply_xy` called by Python method dispatch 3–5×/sardine/frame | Phase 2B: inline into species `_compute_forces` using cached `vx/vy/max_speed/max_force/acc` locals and a local float accumulator |
| `query()` allocates a `list` result per non-empty call (~1 800+/frame) | Phase 2B: callback/generator query API |
| Python per-object loop for `sardine.update()` (1 500+ CALL ops/frame) | Phase 3: numpy-batched force computation |

### Validation

- 214/214 tests pass.
- Toroidal wrapping (x/y/both), no-duplicate, and small-world tests all exercise the correct code paths (fast path for large worlds, slow path with visited set for tiny test grids).
- Ecology, reproduction, starvation, predation, resize, and renderer-sync behavior unchanged.

---

## Phase 3 — Regression Diagnosis & Fix + Benchmark Infrastructure

### Problem Statement

After Phase 2A shipped, observed runtime performance was **worse** than before Phase 2A (and potentially worse than pre-Phase 1). The changes passed all 214 correctness tests but degraded raw frame throughput.

### Root Cause: numpy scalar arithmetic in hot paths

Both Phase 1 and Phase 2A introduced the same class of performance bug: removing `float()` conversions at numpy array element boundaries.

When a numpy array element is extracted without `float()`, the result is a `numpy.float64` scalar. Any arithmetic involving a `numpy.float64` value routes through numpy's type dispatch machinery (ufunc call + boxing/unboxing), which costs roughly **2–4× more per operation** than Python's native C-level float arithmetic.

The hot paths affected were:

| File | Expression before fix | Operations per frame at 1 500 sardines |
|---|---|---|
| `boid.py` `_steer_apply_xy` | `sx = tx * inv_mag - vel[0]` | ~42 000 (6 000 calls × 7 ops) |
| `sardine.py` `_compute_forces` inner loop | `dx = px - opos[0]` (px a numpy scalar) | ~180 000 (30 000 iters × 6 ops) |
| `algae.py` `_compute_forces` inner loop | same pattern | ~60 000–90 000 |
| `sailfish.py` `_compute_forces` COM loop | `dx = float(spos[0]) - px` | ~6 000 |

At ~50–100 ns extra per op (measured overhead of numpy scalar vs Python float path), this accounts for several ms of extra per-frame cost.

**The correct pattern is to cache the array reference AND convert the element:**

```python
# CORRECT: one LOAD_ATTR (cache benefit) + float() to enter Python float path
_pos = self.position
px = float(_pos[0])
```

Not caching the reference (two `self.position` lookups) is slower.
Not calling `float()` (making `px` a `numpy.float64`) is also slower.
Both optimisations must be applied together.

### Fix Applied

Restored `float()` conversion at all numpy array element extraction points in the four hot-path files, while keeping the array-reference caching introduced in Phase 1/2A.

**Files changed:**

- `sim/boid.py` — `_steer_apply_xy`: `sx = tx * inv_mag - float(vel[0])`, `sy = ... - float(vel[1])`
- `sim/sardine.py` — `_compute_forces`: `px = float(_pos[0])`, `py = float(_pos[1])`; `float()` around `sfpos[0/1]`, `opos[0/1]`, `ovel[0/1]`
- `sim/algae.py` — `_compute_forces`: same pattern as sardine.py
- `sim/sailfish.py` — `_compute_forces`: `px = float(_pos[0])`, `py = float(_pos[1])`; `float()` around `spos[0/1]`

### bench.py — Benchmark Infrastructure

New file `bench.py` added at workspace root to provide a lightweight, reproducible headless benchmark. No pyglet window is opened; `Simulation.update()` is driven directly.

**Usage:**

```
python bench.py                  # all scenarios, ms/frame summary
python bench.py --profile        # cProfile heavy scenario, top 20 functions
python bench.py --test           # assert thresholds (CI guard, exit 1 on fail)
python bench.py --frames 100     # override timed frame count (default 300)
```

**Scenarios:**

| Scenario | Sardines | Algae | Sailfish |
|---|---|---|---|
| light | 300 | 600 | 5 |
| medium | 800 | 800 | 8 |
| heavy | 1 500 | 1 200 | 10 |
| extreme | 2 000 | 1 200 | 10 |

Each scenario runs 60 warmup frames (not timed) then 300 timed frames.

**Timing thresholds (`--test` mode, 300 frames):**

| Scenario | Threshold |
|---|---|
| light | < 6 s |
| medium | < 12 s |
| heavy | < 24 s |
| extreme | < 40 s |

### Post-Fix Benchmark Results

Measured on Apple Silicon (Python 3.13, DT=1/60, 300 timed frames, +60 warmup):

| Scenario | ms/frame | fps-equiv |
|---|---|---|
| light (300s) | 6.82 ms | 147 |
| medium (800s) | 26.45 ms | 38 |
| heavy (1 500s) | 55.92 ms | 18 |
| extreme (2 000s) | 87.79 ms | 11 |

These numbers establish the post-Phase 3 baseline. Future passes should be validated against them using `python bench.py`.

### Validation

- 214/214 tests pass after fix.
- `python bench.py --test` passes all 4 scenarios within conservative thresholds.

### Lessons Learned

| Pattern | Correct approach |
|---|---|
| Cache array reference for multiple accesses | `_pos = self.position` (saves 1 LOAD_ATTR) |
| Extract array element for arithmetic chain | `px = float(_pos[0])` (enters Python float path) |
| Don't lose float path for saving one call | Always add `float()` at extraction, even if "redundant" |
| Test correctness ≠ test performance | Add `bench.py --test` to CI to detect regressions |

---

## Optimization Pass — Phase 3C (April 2026)

### Bottleneck Analysis

cProfile output (`python bench.py --profile`, heavy scenario, 300 frames):

| Function | tottime | % of total | ms/frame (real) |
|---|---|---|---|
| `spatial_hash.query()` | 12.086 s | **51%** | 28.6 ms |
| `sardine._compute_forces()` | 7.467 s | **31.6%** | 17.6 ms |
| `list.append` | 0.979 s | 4.1% | 2.3 ms |
| `boid._steer_apply_xy()` | 0.634 s | 2.7% | 1.5 ms |
| `dict.get` | 0.620 s | 2.6% | 1.5 ms |
| `math.sqrt` | 0.382 s | 1.6% | 0.9 ms |

`query()` alone was **51% of total frame time**.  The dominant cost inside
`query()`'s inner loop was `other.position` (LOAD_ATTR → numpy array) followed
by `opos[0]` / `opos[1]` (numpy array subscript → numpy.float64 scalar),
making every subsequent arithmetic expression route through numpy's scalar
type-dispatch machinery (~20–50 ns per op vs ~3 ns for CPython native float).

At 143 560 candidate-pair checks per frame (3 589 queries × ~40 candidates
each), the per-iteration savings of eliminating numpy scalar dispatch compound
to many milliseconds per frame.

### Neighbour Handling Strategy: Grid-insert position/velocity pre-caching

**Chosen approach: B — pre-cached Python floats on each Boid, written by
`SpatialHash.insert()` once per frame.**

`insert()` already computes `ax = float(apos[0])` for the cell-address
calculation.  We now store `agent._gx = ax; agent._gy = ay` at the same
time.  We also extract `agent._gvx = float(avel[0]); agent._gvy = float(avel[1])`
from the velocity array.

Cost of pre-caching per insert: the position values are **free** (already
computed); the velocity adds 1 LOAD_ATTR + 2 numpy subscripts + 2 `float()`
per agent per frame (~160 ns × 2 700 agents = ~430 µs/frame overhead).

**Why not callbacks or cell-based iteration:** A Python callback per
candidate would add higher per-call overhead than iterating a list.
Cell-based iteration inline in `_compute_forces` would duplicate complex
toroidal-wrap cell-address math.  Pre-caching on the Boid itself is the
simplest change with zero API impact.

### Changes Implemented

#### 1. `sim/boid.py` — four new cached-float attributes

`_gx`, `_gy`, `_gvx`, `_gvy` initialised to `0.0` in `Boid.__init__`.
Written by `SpatialHash.insert()` before any agent is updated each frame.

#### 2. `sim/spatial_hash.py` — `insert()` pre-caches values

```python
ax = float(apos[0]); ay = float(apos[1])
agent._gx  = ax;  agent._gy  = ay          # position (already computed)
avel = agent.velocity
agent._gvx = float(avel[0]); agent._gvy = float(avel[1])   # velocity
```

`insert()` also now uses the local `ax`/`ay` Python floats for the cell
address calculation instead of the numpy array elements.

#### 3. `sim/spatial_hash.py` — `query()` inner loop

Replaced `opos = other.position; dx = ax - opos[0]` with
`dx = ax - other._gx`.  All arithmetic is now Python-float—Python-float,
not Python-float—numpy.float64.  Applied to both the fast path and slow
path of `query()`, and to both inner loops of `query_position()`.

#### 4. `sim/sardine.py` — flocking and flee inner loops

- Flee loop: `sfpos = sf.position; dx = px - float(sfpos[0])` → `dx = px - sf._gx`
- Flocking loop position: `opos = other.position; dx = px - float(opos[0])` → `dx = px - other._gx`
- Flocking loop velocity: `ovel = other.velocity; alx += float(ovel[0])` → `alx += other._gvx`

#### 5. `sim/sardine.py` — inline `_steer_apply_xy` (4 calls per fish)

Replaced 4 calls to `self._steer_apply_xy(...)` at the end of
`_compute_forces` with the equivalent math inlined, sharing cached locals
`_vx`, `_vy`, `_ms`, `_mf`, `_acc` across all four directions.

Saves: 4 Python function-call overheads + 12 redundant `self.xxx`
LOAD_ATTR + conversion ops per sardine per frame.

#### 6. `sim/algae.py` — same loop + inline changes

Position/velocity accesses updated to `_gx/_gy/_gvx/_gvy`.  The three
flocking `_steer_apply_xy` calls and the noise call inlined with shared
locals.

#### 7. `sim/sailfish.py` — COM loop

`spos = s.position; dx = float(spos[0]) - px` → `dx = s._gx - px`.

### Benchmark Results

Before = Phase 3 baseline.  After = Phase 3C.  300 timed frames, +60
warmup, DT=1/60, Apple Silicon Python 3.13.

| Scenario | Before (ms/frame) | After (ms/frame) | Improvement |
|---|---|---|---|
| light (300s) | 6.82 | 3.97 | **−42%** |
| medium (800s) | 26.45 | 12.77 | **−52%** |
| heavy (1 500s) | 55.92 | 25.95 | **−54%** |
| extreme (2 000s) | 87.79 | 40.48 | **−54%** |

The improvement is larger at higher populations because the inner
neighbour-loop cost (eliminated numpy dispatch) scales as O(N·k); it was
more dominant at 1 500–2 000 agents.

### Behavior Validation

- 214/214 tests pass after all changes.
- `python bench.py --test` passes all 4 thresholds (PASS).
- `_steer_apply_xy` method retained on `Boid` base class — used by
  sailfish and any external callers; inline only in the sardine/algae hot
  paths.
- `_EMPTY_LIST` sentinel unchanged; `query()` still returns it for empty
  results.
- Slow-path toroidal deduplication (test worlds) still exercises correct
  code paths.

### Remaining Bottlenecks

| Bottleneck | Estimated impact | Next action |
|---|---|---|
| Python `for`-loop over neighbor list per sardine | Primary remaining cost | Phase 4: numpy-batched force computation |
| `list.append` 75 720/frame building query results | ~2.3 ms/frame | Phase 4 or query callback API |
| `boid.update()` dispatch overhead (367 354 calls/frame) | < 1 ms | Inline `_clamp_speed` + `_apply_boundary` into `update()` |
| `random.gauss` 700 k/frame | ~0.5 ms | Replace with `os.urandom`-seeded uniform + fast Gaussian approx |

### Updated Scaling Limits

Post-Phase 3C, 60 fps requires ~16.7 ms/frame:
- **300 sardines**: ~4 ms/frame — well under 60 fps ceiling
- **800 sardines**: ~13 ms/frame — comfortable 60+ fps
- **1 500 sardines**: ~26 ms/frame — sustains ~38 fps  
- **2 000 sardines**: ~40 ms/frame — sustains ~25 fps

Next architectural threshold: numpy-batched force computation (all sardine
positions into `(N,2)` arrays) to eliminate the Python per-object loop
entirely.  Estimated ceiling: 5 000–10 000 agents at 60 fps.

---

## Optimization Pass — Phase 3D (April 2026)

### Context

Post-Phase-3C, `_gx/_gy/_gvx/_gvy` pre-caching eliminated numpy scalar dispatch inside query() and the species flocking loops. The next profiling pass identified a new structural bottleneck: **double-computation of dx/dy/d_sq per neighbour pair**.

Phase 3C left this pattern intact:
1. `query()` computes dx/dy/wrap/d_sq → adds candidate to `results` list
2. `sardine._compute_forces()` iterates `results` and recomputes dx/dy/wrap/d_sq for sep/ali/coh

Every accepted neighbour's geometry was computed twice. Phase 3D eliminates this.

### Bottleneck Identification

`python bench.py --profile` on the heavy scenario (1 500 sardines) immediately after Phase 3C:

| Function | tottime | ncalls | Share |
|---|---|---|---|
| `spatial_hash.query()` | 6.36s | 1,072,524 | **48%** |
| `sardine._compute_forces()` | 3.48s | 344,489 | 26% |
| `list.append` | 0.95s | 22.1M | 7% |
| `dict.get` | 0.63s | 10.7M | 5% |
| `math.sqrt` | 0.38s | 10.2M | 3% |
| Total | 13.23s | — | — |

Key observations:
- **1,072,524 query() calls** per 300-frame run (~3 575/frame) — every sardine calls it twice (flee + flocking), every algae once, ecology passes add more.
- **22.1M list.append** calls: most from the flocking queries materialising neighbour lists that the species loop then re-traverses.
- `sardine._compute_forces()` tottime=3.48s is the time for the *second* loop over the neighbour list (re-computing dx/dy/d_sq for sep/ali/coh accumulation).

The previous `_gx/_gy` cache work had already eliminated numpy dispatch. The remaining cost was the **structural overhead** of building an intermediate list and iterating it twice.

### What Phase 3C Did NOT Revisit

Phase 3C's _gx/_gy pre-caching work was complete and correct. Phase 3D does not touch it.

### Changes Implemented

#### 1. `sim/spatial_hash.py` — `query_first()`

New method: stops at the first qualifying neighbour instead of building a complete list. Used by the sailfish one-kill-per-tick ecology pass, which only needs `candidates[0]`. Same fast/slow-path split as `query()`.

#### 2. `sim/sardine.py` — inline flocking cell traversal

Replaced:
```python
neighbors = self._grid.query(self, cfg.radius_cohesion)
for other in neighbors:
    dx = px - other._gx  # re-computed!
    ...
```
With a single inline cell walk that computes dx/dy/d_sq **once** and accumulates sep/ali/coh directly, never building a list. Slow path (tiny test worlds: `nc ≤ 2·span`) falls back to `query()` + loop for correctness.

#### 3. `sim/sardine.py` — inline flee cell traversal

Same approach on the sailfish grid. The flee query was called ~1 000 times/frame (once per sardine) and almost always returned an empty list. The inline exits in 9 `dict.get` misses with no allocation overhead.

#### 4. `sim/algae.py` — inline flocking cell traversal

Same as sardine. Eliminates list materialisation for all algae flocking queries.

#### 5. `sim/simulation.py` — `query_first()` for sailfish eating

```python
prey = self._grid.query_first(sf, eat_r)
if prey is not None: ...
```
Replaces `candidates = grid.query(sf, eat_r); if candidates: prey = candidates[0]`.

#### 6. New radius-squared and span caches

Added to sardine: `_coh_r2`, `_flee_r2`, `_coh_span`, `_flee_span`.
Added to algae: `_coh_r2`, `_coh_span`.

These avoid `radius ** 2` and `max(1, math.ceil(radius / cell_size))` inside the hot inline traversal. All updated in `Simulation.apply_config()`.

### Benchmark Results

Before = post-Phase-3C baseline measured immediately before this pass.
After = post-Phase-3D. 300 timed frames, +60 warmup, DT=1/60, Apple Silicon Python 3.13.

| Scenario | Before (ms/frame) | After (ms/frame) | Improvement |
|---|---|---|---|
| light (300s) | 4.19 | 3.60 | **−14%** |
| medium (800s) | 13.82 | 10.74 | **−22%** |
| heavy (1 500s) | 28.20 | 22.53 | **−20%** |
| extreme (2 000s) | 39.27 | 33.82 | **−14%** |

`python bench.py --test` — all four scenarios within their thresholds. PASS.

#### Post-3D profile (heavy, 300 frames)

| Function | tottime | ncalls | vs Before |
|---|---|---|---|
| `sardine._compute_forces()` | 6.33s | 362k | tottime ↑ (now includes inline fleet traversal) |
| `spatial_hash.query()` | 0.98s | 377k | **−85%** calls, **−85%** tottime |
| `dict.get` | 0.59s | 11M | ≈ same |
| `math.sqrt` | 0.38s | 10.7M | ≈ same |
| `list.append` | 0.10s | 2.3M | **−90%** |
| Total | 9.76s (profiled) | — | **−26% total** |

The apparent tottime increase in sardine is expected: query() time shifted inside sardine since the cell walk is now inline. Cumtime (sardine + all its callees) dropped from ~10.7s to ~7.4s — a real reduction.

### Why Improvement Is Smaller at Light/Extreme

- **Light (300 sardines)**: population is small; baseline cost was already low; overhead from ecology (algae-eating queries, reproduction) now dominates more.
- **Extreme (2 000s)**: fishery dynamics cause large population swings; many frames late in the run have fewer sardines (eaten/starved) but many sailfish (grown), shifting proportional costs.

### Behavior Validation

- 231/231 tests pass after all changes.
- All flocking, flee, algae drift, sailfish pursuit, ecology, reproduction, starvation, mouse predator, resize/fullscreen, and config-menu paths tested.
- Slow-path fallback (tiny worlds: `nc ≤ 2·span`) still exercises `query()` for correctness.
- No duplicate neighbour visits: each cell is visited once in the fast-path inline (same guarantee as `query()` fast path).
- `apply_config()` propagates all new caches (`_coh_r2`, `_flee_r2`, `_coh_span`, `_flee_span`) so live config edits remain correct.
- `query()` API unchanged — all ecology and sailfish pursuit callers continue to work.

### New Tests (17 added)

**`TestQueryFirst`** (10 tests):
- hit, miss, excludes-self, empty-grid, boundary-included, multiple-candidates, agrees-with-query-empty, agrees-with-query-nonempty, toroidal-x-wrap, toroidal-y-wrap.

**`TestInlineTraversalCorrectness`** (7 tests):
- sardine separation force non-zero with close neighbour.
- sardine alignment force non-zero with differently-headed neighbour.
- sardine zero force with no neighbours (noise disabled).
- sardine slow-path fallback (tiny world forces correct accumulation).
- algae alignment force non-zero.
- `apply_config()` updates `_coh_span` correctly.
- `apply_config()` updates `_flee_span` correctly.

### Remaining Dominant Bottlenecks

| Bottleneck | Estimated impact | Next action |
|---|---|---|
| Python `for`-loop over grid cells per sardine (inline) | Primary remaining cost ~6s/300-frames | Phase 4: numpy-batched force computation |
| `dict.get` cell lookups (11M/300-frames) | ~0.6s/300-frames | Unavoidable with spatial hash; reduce by lowering span or using flat array grid |
| `math.sqrt` per separation pair | ~0.4s/300-frames | Could approximate with `rsqrt`; or batch with numpy |
| `boid.update()` dispatch overhead | ~0.3s/300-frames | Inline `_clamp_speed` + `_apply_boundary` |
| `random.gauss` noise (700k calls/300-frames) | ~0.2s/300-frames | Batch-generate with numpy at grid-rebuild time |

### Updated Scaling Limits

Post-Phase 3D, 60 fps requires ≤ 16.7 ms/frame:
- **300 sardines**: ~3.6 ms/frame — well within 60 fps
- **800 sardines**: ~11 ms/frame — comfortable 60 fps
- **1 500 sardines**: ~22 ms/frame — sustains ~45 fps (up from 38 fps post-3C)
- **2 000 sardines**: ~34 ms/frame — sustains ~30 fps (up from 25 fps post-3C)

### Recommendation for Next Pass

The inline cell traversal has extracted all available gains from structural list/copy elimination while remaining in pure CPython. The next meaningful speedup requires:

**Phase 4C: numpy-batched force accumulation** — vectorise the per-sardine cell traversal in `_compute_forces()` using the `(N,2)` position/velocity arrays established in Phase 4A. Process cell-bucket neighbourhoods with numpy broadcasting: compute pairwise dx/dy/d_sq for all source–neighbour pairs in each bucket, accumulate sep/ali/coh into batch arrays, then scatter forces back. Estimated 5–10× further reduction for the sardine flocking loop, enabling 5 000+ sardines at 60 fps.

---

## Optimization Pass — Phase 4A–4B (April 2026)

### Bottleneck Confirmation

`python bench.py --profile` (heavy scenario, 1 500 sardines, 300 frames) immediately before this pass:

| Function | tottime | % of total | Notes |
|---|---|---|---|
| `sardine._compute_forces()` | 6.20s | **65%** | Python per-sardine cell traversal loop |
| `spatial_hash.query()` | 0.94s | 10% | Remaining ecology queries |
| `dict.get` | 0.58s | 6% | Cell lookups — unavoidable with hash grid |
| `math.sqrt` | 0.38s | 4% | Per-neighbour distance |
| `boid.update()` | 0.30s | 3% | Integration overhead for all species |
| `_clamp_speed()` | 0.09s | 1% | Per-sardine method call overhead |
| `_apply_boundary()` + `wrap_position()` | 0.11s | 1% | Per-sardine boundary calls |

Integration overhead (`boid.update` + `_clamp_speed` + `_apply_boundary` + `wrap_position`) = **~0.50s = 5%** of total. This is the target for Phase 4B while Phase 4A establishes the infrastructure for Phase 4C.

### Phase 4A — Batched State Infrastructure

Added `(N, 2)` float64 arrays to `Simulation` for all sardine state:

```python
self._s_pos: np.ndarray  # sardine positions
self._s_vel: np.ndarray  # sardine velocities
self._s_acc: np.ndarray  # sardine accelerations (gathered each frame)
self._s_n:   int         # live sardine count
```

At the start of each `update()`, after the grid rebuild (when `_gx/_gy/_gvx/_gvy` are fresh Python floats): each sardine's `_idx` is assigned and its position/velocity is written into the batch arrays. This is a new O(N) Python loop with cheap Python-float-to-numpy assignments.

**Key design rule**: `_gx/_gy/_gvx/_gvy` are already computed by `insert()` — Phase 4A reads them directly, avoiding re-reading numpy position/velocity arrays. Zero extra numpy overhead.

**Capacity growth**: 2× policy via `_ensure_sardine_capacity(n)`. Only reallocates when `n > shape[0]`. At steady state (no growth), this is a single `if` check per frame.

**Births/deaths**: The index assignment happens at the very start of each frame before any integration, so all indices are always fresh and contiguous. Dead sardines (removed from `self.sardines` during ecology) are simply absent at the next frame's populate pass — no stale references possible.

### Phase 4B — Batched Integration

Replaced per-sardine `agent.update(dt)` (which called `_compute_forces`, then integrated individually) with:

1. Per-sardine `s._compute_forces()` loop (still Python, unchanged from Phase 3D)
2. `_batch_integrate_sardines(dt)` — a single vectorised call:
   - **Gather**: `_s_acc[i] = s.acceleration` for all sardines (O(N) Python loop, fast numpy element writes)
   - **Velocity integration**: `_s_vel[:n] += _s_acc[:n] * dt` (numpy broadcast)
   - **Speed clamp**: vectorised equivalent of `_clamp_speed()` using `np.hypot`, boolean masks, and in-place scaling — covers max clamp, min floor, and degenerate-zero cases
   - **Position integration**: `_s_pos[:n] += _s_vel[:n] * dt` (numpy broadcast)
   - **Boundary wrap**: `_s_pos[:n, 0] %= W` / `_s_pos[:n, 1] %= H` (wrap mode), or vectorised reflection for clamp mode
   - **Scatter**: write `_s_pos[i]`, `_s_vel[i]` back to sardine objects; clear `s.acceleration` (O(N) Python loop)

### Profile After Phase 4B

| Function | Before | After | Change |
|---|---|---|---|
| `boid.update()` ncalls | 374k (sardine+sf+algae) | **21k** (sf+algae only) | −94% calls |
| `_clamp_speed()` tottime | 0.090s | **< 0.001s** | Eliminated for sardines |
| `_apply_boundary()` tottime | 0.043s | **< 0.001s** | Eliminated for sardines |
| `wrap_position()` tottime | 0.064s | **< 0.001s** | Eliminated for sardines |
| `_batch_integrate_sardines()` | — | **0.132s** | New vectorised path |
| Total profiled | 9.54s | **9.07s** | **−5%** |

Net integration overhead: ~0.50s → ~0.15s (Phase 4B adds 0.132s, removes 0.50s).

### Benchmark Results (Phase 3D → Phase 4B baseline)

300 timed frames, +60 warmup, DT=1/60, Apple Silicon Python 3.13:

| Scenario | Before (ms/frame) | After (ms/frame) | Δ% |
|---|---|---|---|
| light (300s) | 3.58 | ~3.5 | −2% |
| medium (800s) | 11.29 | ~10.6 | −6% |
| heavy (1 500s) | 21.90 | ~22 | ±noise |
| extreme (2 000s) | 32.85 | ~31 | −5% |

**Note on variance**: The ecology system (sardines eaten/reproduced, sailfish growing) produces different population trajectories each run. Benchmarks use population-weighted averages that hide the structural win. The profiler (fixed seed, single scenario) shows the true −5% gain more cleanly.

### What Phase 4A/4B Establishes for Phase 4C

- **`_s_pos[:n]`**: contiguous sardine positions ready for pairwise distance computation
- **`_s_vel[:n]`**: contiguous velocities ready for alignment accumulation
- **`_s_n`**: the live sardine count — slice bound for all vectorised ops
- **`sardine._idx`**: each sardine knows its own row — force writes can be directed without object lookup

Phase 4C will wire these arrays into a vectorised neighbourhood search: for each spatial cell, gather the neighbour-bucket indices, compute pairwise dx/dy with broadcasting, apply the sep/ali/coh masks, and accumulate into `_s_acc` — all in numpy without a Python loop per neighbour pair.

### New Tests (15 added — `TestPhase4Batching`)

- `test_batch_arrays_allocated_on_first_update` — capacity ≥ n after first update
- `test_batch_index_assigned_correctly` — each _idx == list position after populate
- `test_batch_pos_populated_matches_position` — _s_pos[:n] matches sardine.position
- `test_capacity_grows_when_sardines_exceed_initial_cap` — 2× growth triggered
- `test_batch_integration_moves_sardine_expected_distance` — v×dt displacement correct
- `test_batch_integration_velocity_written_back` — vel scattered correctly
- `test_batch_speed_clamp_max_enforced` — batch path enforces max_speed
- `test_batch_speed_floor_min_enforced` — batch path enforces min_speed
- `test_batch_boundary_wrap_x` — x wraps at right edge
- `test_batch_boundary_wrap_y` — y wraps at top edge
- `test_batch_acceleration_cleared_after_integration` — acc reset to zero
- `test_births_get_indices_next_frame` — offspring get valid _idx on next frame
- `test_deaths_remaining_sardines_valid_next_frame` — contiguous indices after deletion
- `test_zero_sardines_no_crash` — empty sardine list handled gracefully
- `test_apply_config_max_speed_used_in_batch_clamp` — config changes take effect

### Validation

- 246/246 tests pass.
- `python bench.py --test` passes all 4 thresholds. PASS.
- Flocking, flee, algae drift, sailfish pursuit, ecology, reproduction, starvation, mouse predator, resize/fullscreen, config-menu paths all exercise the new batch path.
- Sardine objects' `position` and `velocity` are fully synchronised after each frame — rendering and ecology read correct values.
- `_gx/_gy/_gvx/_gvy` caches refreshed at start of next frame's grid rebuild — no stale reads.

### Remaining Dominant Bottleneck

| Bottleneck | Estimated impact | Next action |
|---|---|---|
| Python `for` loop over grid cells per sardine in `_compute_forces` | **~65% of frame time** | **Phase 4C: numpy-batched force accumulation** |
| `dict.get` cell lookups (10.7M/300-frames) | ~0.6s/300-frames | Unavoidable with hash grid; reducible with flat-array grid |
| `math.sqrt` per separation pair (9.9M/300-frames) | ~0.35s/300-frames | Phase 4C: batch via `np.hypot` on 2D arrays |
| `random.gauss` noise (695k/300-frames) | ~0.17s/300-frames | Batch-generate with numpy at frame start |
| `spatial_hash.query()` (remaining ecology passes) | ~0.94s/300-frames | Out of critical path for sardines |

### Updated Scaling Limits

Post-Phase 4B, 60 fps requires ≤ 16.7 ms/frame:
- **300 sardines**: ~3.5 ms/frame — well within 60 fps
- **800 sardines**: ~10.6 ms/frame — comfortable 60 fps
- **1 500 sardines**: ~22 ms/frame — sustains ~45 fps
- **2 000 sardines**: ~32 ms/frame — sustains ~30 fps

These limits are the same as post-3D because the bottleneck (`_compute_forces`) is unchanged. Phase 4B removes the 5% integration overhead; Phase 4C is needed for the next ceiling breakthrough.

---

## Optimization Pass — Phase 4C (April 2026)

### Bottleneck Confirmation

`python bench.py --profile` (heavy scenario, 1 500 sardines, 200 frames) immediately before this pass:

| Function | tottime | % of total | Notes |
|---|---|---|---|
| `sardine._compute_forces()` | 5.075s | **69.5%** | Per-sardine Python cell-traversal loop |
| `spatial_hash.query()` | 0.629s | 8.6% | Ecology + sailfish queries |
| `dict.get` | 0.443s | 6.1% | Cell hash lookups |
| `math.sqrt` | 0.300s | 4.1% | Per-pair separation distance |
| `_batch_integrate_sardines()` | 0.108s | 1.5% | Phase 4B integration — no longer dominant |

Integration overhead confirmed eliminated. `_compute_forces` is the sole bottleneck.

### Algorithm (Cell-Bucket Vectorisation)

`Simulation._batch_compute_sardine_forces()` replaces the per-sardine Python loop for N≥500 sardines:

**Pass 1 — Flee (vectorised N×S)**

```python
sf_x, sf_y = np.array(...)   # (S,) sailfish positions
dx = sp[:, 0:1] - sf_x       # (N, S) pairwise sardine–sailfish x-deltas
dy = sp[:, 1:2] - sf_y       # (N, S) pairwise sardine–sailfish y-deltas
# Toroidal correction + distance gate + unit-away accumulation
```

All N sardines compared to all S sailfish in one numpy operation block. Cohesion suppression flag `is_fleeing` computed from the mask.

**Pass 1b — Mouse flee (vectorised N×1)**

Same pattern against the single mouse-cursor point when `mouse_predator_enabled`.

**Pass 2 — Cell-bucket flocking (sep + ali + coh)**

For each occupied grid cell (Python `for` loop over cells, not over sardines):
1. Collect `own_idxs` — the sardines whose home cell this is (M rows).
2. Collect `neigh_idx` — all sardines in the (2·coh_span+1)² neighbourhood (K rows).
3. Compute `(M, K)` pairwise dx/dy via numpy broadcasting.
4. Apply toroidal correction, distance-squared gate, self-pair exclusion.
5. Compute sep/ali/coh contributions with numpy ops; accumulate into `_s_acc`.

Fast-path / slow-path cell deduplication matches `query()`.

**Pass 3 — Noise (vectorised N×2)**

`np.random.normal(0, 1, (n, 2))` in one call; steer + clamp vectorised.

**Small-population fallback (N<500)**

Below 500 sardines, numpy dispatch overhead exceeds per-sardine Python loop cost (measured crossover at ~500 on Apple Silicon). The fallback calls the per-sardine `s._compute_forces()` and copies acc into `_s_acc` — same output, cheaper at small N.

### Benchmark Results

200 timed frames, +60 warmup, DT=1/60, Apple Silicon Python 3.13:

| Scenario | Before (ms/frame) | After (ms/frame) | Speedup |
|---|---|---|---|
| light (300s) | 3.51 | **3.61** | ~1× (fallback path, noise variance) |
| medium (800s) | 11.07 | **8.39** | **+1.32×** |
| heavy (1500s) | 26.44 | **11.63** | **+2.27×** |
| extreme (2000s) | 42.92 | **14.86** | **+2.89×** |

Heavy scenario profile after Phase 4C: `_batch_compute_sardine_forces` = **1.55s (47%)**, down from 5.07s (69%). Next dominant cost: `spatial_hash.query()` at 0.62s (19%) + list `.append` overhead at 0.15s.

### Validation

- 255/255 tests pass.
- All Phase 4A/4B tests pass unchanged (batch integration and scatter still correct).
- New `TestPhase4CBatchedForces` suite (9 tests): no self-interaction, toroidal wrap correctness, non-zero sep/ali/coh forces, flee force injection, NaN/Inf guard, fallback path, numerical agreement with per-sardine reference within 0.1%.
- `python bench.py --test` PASS.

### New Tests (9 added — `TestPhase4CBatchedForces`)

- `test_zero_sardines_no_crash`
- `test_single_sardine_no_flocking_force`
- `test_no_self_interaction` — verifies owner never contributes to its own separation
- `test_batch_separation_nonzero_close_pair`
- `test_batch_toroidal_wrap_detect_neighbour` — sardines near opposite edges see each other
- `test_batch_matches_per_sardine_reference` — allclose with rtol=1e-3 at 600 sardines
- `test_batch_flee_force_nonzero_when_sailfish_nearby` — flee pass correct
- `test_stable_small_population_uses_fallback` — no NaN/Inf in fallback path
- `test_no_nan_inf_large_population` — no NaN/Inf at 1000 sardines over 5 frames

### New Scaling Limits

| Sardines | Before 4C | After 4C |
|---|---|---|
| 300 | ~3.5 ms (60+ fps) | ~3.6 ms (60+ fps) |
| 800 | ~11 ms (90 fps) | ~8 ms (120+ fps) |
| 1 500 | ~22 ms (45 fps) | ~12 ms (85 fps) |
| 2 000 | ~32 ms (30 fps) | ~15 ms (67 fps) |
| 3 000+ | ~50+ ms (20 fps) | **~25–30 ms (35–40 fps)** |

### New Dominant Bottleneck

After Phase 4C the ceil-bucket cell-sweep Python loop itself costs ~40–50% of frame time (`dict.get` + `list.append` building `neigh_list` per cell). Phase 4D would pre-build a CSR (compressed-row) flat sardine-index array during the grid rebuild so neighbour rows can be gathered with a single slice instead of per-cell list construction.
