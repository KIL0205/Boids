# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

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

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

- **Runtime**: `python main.py` — resizable window at screen resolution, 60 fps target.
- **Tests**: `python -m tests.tests` — 214 tests, ~1.25 s.

### Current capabilities
- Sardines flock using Reynolds separation / alignment / cohesion + small noise term.
- Sailfish hunt sardine schools by steering toward the local centre of mass.
- Sardines detect nearby sailfish, flee with full-force escape, and scatter (cohesion suppressed while fleeing).
- **Algae drift** slowly across the world; alignment-dominant flocking gives a water-current feel.
- **Sardine feeding**: sardines eat algae on contact (`algae_eat_radius`); eaten algae vanish instantly, sardine `food_counter` and `hunger_timer` tracked.
- **Sardine reproduction**: when `food_counter` reaches `sardine_eat_threshold`, the sardine splits — one offspring spawned beside it (perpendicular offset = `radius_separation`), inheriting parent velocity; `food_counter` decremented by threshold (not reset to 0) so leftover progress carries forward.
- **Sailfish predation**: each sailfish eats the nearest sardine within `sailfish_eat_radius` each frame (one per sailfish per tick); the sardine is removed immediately; sailfish `food_counter` and `hunger_timer` track feeding state. HUD shows cumulative predation count (`Pred ↑N`).
- **Sailfish reproduction**: when a sailfish's `food_counter` reaches `sailfish_eat_threshold` (sardines eaten), it spawns one offspring beside it (perpendicular offset = `radius_separation`), inheriting parent velocity; `food_counter` decremented by threshold (not reset to 0); HUD shows cumulative births.
- Live window resize and fullscreen toggle (world geometry updates in-place).
- Runtime config menu (TAB): 25+ parameters editable live across 8 sections; panel scrolls automatically; settings persist across runs.
- Velocity-based fish colouring: 7 sardine palette modes, 5 sailfish colour modes.
- Dynamic population controls from keyboard.
- HUD overlay (FPS, frame timing, per-species counts, totals, full ecosystem event stats, average sardine/sailfish hunger).
- **Mouse predator**: holding the left mouse button causes nearby sardines to flee the cursor — same flee mechanics as a real sailfish. A faint red radius indicator appears at the cursor. Configurable toggle, radius, and flee-weight multiplier.

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
| `Sailfish(Boid)` | `sim/sailfish.py` | COM pursuit; holds sardine grid ref; coasts when no sardines visible; `food_counter`, `hunger_timer` |
| `Algae(Boid)` | `sim/algae.py` | Alignment-dominant drift entity; holds algae grid ref; no predator awareness |
| `SpatialHash` | `sim/spatial_hash.py` | `insert()`, `query()`, `clear()`, `resize()` with toroidal cell-key wrapping |
| `Simulation` | `sim/simulation.py` | Spawns agents; owns three grids; `update(dt)`, `resize()`, `apply_config()`; ecology pass |
| `Renderer` | `rendering/renderer.py` | `reset()`, `draw()`, `on_resize()`, `toggle_hud()`; velocity-colour logic; algae/sardine/sailfish shapes |
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
- **42 rows** across 8 group sections + 1 Save row.  Panel height is fixed at 418 px (`_VISIBLE_ROWS = 16`); rows scroll automatically as you navigate with ↑/↓.  Scroll indicators (▲/▼) appear in the footer strip when off-screen rows exist.
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
- 198 tests pass (`python -m unittest tests.tests`).
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

### Scaling limits
- **Bottleneck**: Python per-object loop in `simulation.update()`. Saturates at ~1 500 sardines.
- **Next threshold**: beyond ~1 500, numpy-batched force computation (all-sardine positions in a single `(N,2)` array) would eliminate per-object interpreter overhead.
- **Sailfish**: each queries the sardine grid once per frame, O(k). Negligible at ≤ 30 sailfish.
- **Renderer**: `_sync_shapes` is O(N) pure Python; already optimised. At 1 500 agents ≈ 3–5 ms.
- **`query()` result list**: still allocates a `list["Boid"]` per call (~1800+/frame). Eliminating this would require a callback/iterator API — significant refactor with limited payoff at current scale.

---

## 8. TODO / Next Steps

### High priority
1. **`load_overrides` edge-case test coverage** — corrupt JSON, unknown fields, colour-mode validation. `TestConfigMenuEcosystem` partially covers config persistence.

### Medium priority
2. **`num_sardines` / `num_sailfish` as editable menu rows** — they already persist correctly. Needs population-adjustment logic on value change (not trivial; separate from the slider model).
3. **Cross-field constraint enforcement** — clamp `radius_separation < radius_alignment` and `radius_alignment ≤ radius_cohesion` inside `_adjust_value()`.

### Phase 9 (future)
4. **Nearest-prey targeting** — individual pursuit rather than group COM; biologically more accurate for a single sailfish.
5. **Sailfish burst acceleration** — short high-speed sprint with cooldown.
6. **Per-sardine danger memory** — smooth cohesion restoration after escape (timer-based rather than instantaneous).
7. **Numpy batched force computation** — vectorise all-sardine positions into `(N,2)` array for SIMD-level speedup beyond ~1 500 sardines.
8. **HiDPI / Retina support** — `pyglet.options['hidpi'] = True` before `Window()`.

---

## 9. References

- **[ARCHIVE.md](ARCHIVE.md)** — Phase-by-phase build history, implementation archaeology, subclass interface contract, algorithmic complexity reference, historical tuning table, Phase 8 ecosystem plan.
- **[tests/tests.py](tests/tests.py)** — 214 tests across 18 suites: `TestVec2`, `TestBoidKinematics`, `TestSpatialHash`, `TestSimulationIntegrity`, `TestPhase2`, `TestAlgaeKinematics`, `TestEcologyContactDetection`, `TestSardineReproduction`, `TestSailfishEcology`, `TestSardineStarvation`, `TestSailfishStarvation`, `TestAlgaeRespawn`, `TestHudEcosystem`, `TestConfigMenuEcosystem`, `TestSailfishReproduction`, `TestSailfishSpawnInit`, `TestSailfishToggles`, `TestMousePredator`.
- **[config/config.py](config/config.py)** — Authoritative parameter definitions with inline comments.
