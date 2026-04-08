# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

> **Working reference for active development.** Optimised for quick orientation, not history.
> Historical build notes, phase changelists, and implementation archaeology → [ARCHIVE.md](ARCHIVE.md)

---

## 1. Project Overview

A 2D real-time Python simulation of sardine schooling and sailfish predation.

- **Runtime**: `python main.py` — resizable window at screen resolution, 60 fps target.
- **Tests**: `python -m tests.tests` — 185 tests, ~1.5 s.

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
│   ├── sardine.py            # Sardine(Boid): flocking + flee [+ food_counter/hunger_timer in Phase 8]
│   ├── sailfish.py           # Sailfish(Boid): COM pursuit [+ food_counter/hunger_timer in Phase 8]
│   ├── algae.py              # [Phase 8] Algae(Boid): alignment-dominant drift; no predator awareness
│   ├── simulation.py         # Simulation: owns agents + grids, drives update loop [+ ecology pass in Phase 8]
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
| `Sardine(Boid)` | `sim/sardine.py` | Flocking (sep/ali/coh/noise) + flee; holds sardine + sailfish grid refs; [Phase 8] `food_counter`, `hunger_timer` |
| `Sailfish(Boid)` | `sim/sailfish.py` | COM pursuit; holds sardine grid ref; coasts when no sardines visible; [Phase 8] `food_counter`, `hunger_timer` |
| `Algae(Boid)` | `sim/algae.py` | *[Phase 8]* Alignment-dominant drift entity; holds algae grid ref; no predator awareness |
| `SpatialHash` | `sim/spatial_hash.py` | `insert()`, `query()`, `clear()`, `resize()` with toroidal cell-key wrapping |
| `Simulation` | `sim/simulation.py` | Spawns agents; owns three grids; `update(dt)`, `resize()`, `apply_config()`; [Phase 8] ecology pass |
| `Renderer` | `rendering/renderer.py` | `reset()`, `draw()`, `on_resize()`, `toggle_hud()`; velocity-colour logic; [Phase 8] algae shapes |
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
- **40 rows** across 8 group sections + 1 Save row.  Panel height is fixed at 418 px (`_VISIBLE_ROWS = 16`); rows scroll automatically as you navigate with ↑/↓.  Scroll indicators (▲/▼) appear in the footer strip when off-screen rows exist.
- Every value change immediately calls `Simulation.apply_config()` — effect is visible in the next frame.
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

**Note**: `sardine_initial_hunger_time` and `sailfish_initial_hunger_time` are intentionally
omitted from the menu.  They only affect the `hunger_timer` set on construction and have no
effect on existing agents at runtime — changing them live would be misleading.

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
| `sailfish_eat_radius` | **25.0** | Widened from 20 px; sailfish–sardine contact detection radius; one kill per sailfish per tick || `algae_size` | 4.0 | px (circle radius) |

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
- 169/169 tests pass (`python -m unittest tests.tests`).
- `python main.py` opens a resizable window; all features operational.
- Sardine flocking: stable, tuned, numerically bounded.
- Sailfish pursuit + sardine flee: working, three-grid architecture clean.
- **Algae drifting**: 800 algae spawn randomly; alignment-dominant flocking gives a gentle water-current drift; rendered as small **reddish-brown circles** (`(170, 50, 20, 255)`, radius 4 px) layered behind sardines/sailfish; HUD shows algae count + total eaten.
- **Sardine–algae eating (Stage 8.3)**: each frame, sardines consume algae within `algae_eat_radius` (14 px); eaten algae are removed from `simulation.algae` and their Triangle shapes removed from the renderer in O(K) via `_algae_shape_map`; `Sardine.food_counter` and `hunger_timer` tracked. HUD now shows `Eaten ↑N`.
- **Sardine reproduction (Stage 8.4)**: when `food_counter >= sardine_eat_threshold` (default 3), a new offspring Sardine is appended to `simulation.sardines` at a perpendicular offset from the parent (= `radius_separation`); `food_counter -= threshold` preserves excess; `_sardine_born` list consumed by renderer each frame to allocate one new Triangle per offspring with zero full-reset overhead. `stats_sardine_births` tracks cumulative births.
- **Sailfish predation (Stage 8.6)**: each sailfish queries the sardine grid within `sailfish_eat_radius` each frame; the first candidate is eaten (one per sailfish per tick to prevent mass-kill); the sardine is removed from `simulation.sardines`; sailfish `food_counter += 1` and `hunger_timer = 0.0`; `_sardines_eaten_ids` set consumed by renderer to delete the matched Triangle and VelColorState in O(K) without a full reset; `stats_sardines_eaten` tracks cumulative predation.
- **Sardine starvation (Stage 8.5)**: after each eating pass, sardines with `hunger_timer > sardine_starvation_time` are removed; `_sardines_starved_ids` side-channel consumed by renderer (unioned with `_sardines_eaten_ids`) for O(K) shape removal; `stats_sardine_starvations` accumulates. Configurable initial hunger time (`sardine_initial_hunger_time=15.0`) staggers startup deaths. `sardine_starvation_time=0.0` disables the mechanic.
- **Sailfish starvation (Stage 8.7)**: mirror of sardine starvation for sailfish; `hunger_timer` incremented before eating so a sailfish that eats the same frame it would starve survives; `_sailfish_died_ids` side-channel + `_sailfish_shape_map` O(K) renderer removal; `stats_sailfish_starvations` accumulates. Configurable via `sailfish_starvation_time` (default 90 s) and `sailfish_initial_hunger_time` (default 40 s).
- **HUD ecosystem metrics**: HUD now shows 7 lines of data refreshed at 4 Hz (`_REFRESH = 0.25 s`): (1) FPS + frame/update/render timing; (2) per-species live counts + total; (3) algae eaten ↑N  spawned ↑N; (4) sardine born ↑N  starved ↑N  eaten ↑N  avg hunger Xs; (5) sailfish starved ↑N  avg hunger Xs  [COLOR]; (6) world size; (7) control hints. Average hunger is `sum(agent.hunger_timer)/N` — O(N) but runs only 4×/s, negligible at N=300. Label widened to 520 px to accommodate the extra lines without wrapping key metrics.
- **Ecosystem balance tuning**: default counts rebalanced: `num_sardines` 900 → 300, `num_algae` 500 → 800; `sardine_eat_threshold` 5 → 3; `sardine_starvation_time` 20 → 30 s; `sardine_initial_hunger_time` 10 → 15 s; `sailfish_eat_radius` 20 → 25 px; `sailfish_starvation_time` 45 → 90 s; `sailfish_initial_hunger_time` 20 → 40 s.  Root cause of prior guaranteed collapse: algae were a finite resource — 900 sardines depleted 500 algae in ≤2 s, leaving no time for reproduction or starvation dynamics to play out.
- Live resize and fullscreen: in-place world geometry update, no respawn (algae positions wrapped on resize, `_algae_grid` resized).
- Config menu: 25+ params across 8 sections, panel scrolls (fixed 418 px height, 16 visible rows at once); int fields (`sardine_eat_threshold`, `algae_max_count`) cast correctly; all ecosystem parameters editable live; save/load round-trip lossless.
- **Ecosystem config params in live menu**: all major ecology knobs (`algae_respawn_rate`, `algae_max_count`, `algae_eat_radius`, `sardine_eat_threshold`, `sardine_starvation_time`, `sailfish_eat_radius`, `sailfish_starvation_time`, `algae_max_speed`, `weight_algae_alignment`, `weight_algae_separation`) editable at runtime; int-typed fields preserved through `apply_config`; all changes visible within the ecology pass on the very next `update()` call.  Menu panel scrolls automatically — no overflow on any window size.
- Velocity-based colouring: 7 modes, cosine cyclic mapping (no flicker), sailfish colour modes.
- Population synced to config on add/remove for correct save behaviour.


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
1. ~~**Add `TestConfigPersistence` and `TestApplyConfig` test suites**~~ — partially covered by new `TestConfigMenuEcosystem` (7 tests: starvation/threshold/radius live-apply, int field types, persistence round-trip).  Remaining gap: broader `load_overrides` edge-case coverage (corrupt JSON, unknown fields, colour-mode validation).

### Medium priority
2. **`num_sardines` / `num_sailfish` as editable menu rows** — they already persist correctly. Needs respawn or population-adjustment logic on value change (not trivial; separate from the slider model).
3. **Cross-field constraint enforcement** — clamp `radius_separation < radius_alignment` and `radius_alignment ≤ radius_cohesion` inside `_adjust_value()`.

### Phase 8 (ecosystem — complete)
See **Section 10** for the original staged plan (now fully implemented — all stages 8.1–8.8 shipped).

### Phase 9 (future, post-ecosystem)
4. **Nearest-prey targeting** — individual pursuit rather than group COM; biologically more accurate for a single sailfish.
5. **Sailfish burst acceleration** — short high-speed sprint with cooldown.
6. **Per-sardine danger memory** — smooth cohesion restoration after escape (timer-based rather than instantaneous).
7. **Numpy batched force computation** — vectorise all-sardine positions into `(N,2)` array for SIMD-level speedup beyond ~1 500 sardines.
8. **HiDPI / Retina support** — `pyglet.options['hidpi'] = True` before `Window()`.

---

## 9. References

- **[ARCHIVE.md](ARCHIVE.md)** — Phase-by-phase build history, implementation archaeology, subclass interface contract, algorithmic complexity reference, historical tuning table.
- **[tests/tests.py](tests/tests.py)** — 185 tests across 15 suites: `TestVec2`, `TestBoidKinematics`, `TestSpatialHash`, `TestSimulationIntegrity`, `TestPhase2`, `TestAlgaeKinematics`, `TestEcologyContactDetection`, `TestSardineReproduction`, `TestSailfishEcology`, `TestSardineStarvation`, `TestSailfishStarvation`, `TestAlgaeRespawn`, `TestHudEcosystem`, `TestConfigMenuEcosystem`, `TestSailfishReproduction`.
- **[config/config.py](config/config.py)** — Authoritative parameter definitions with inline comments.

---

## 10. Phase 8: Ecosystem — Algae, Food Chains, and Population Dynamics

> **Status**: Complete. All stages implemented and tested.
> Original branch: `ecosystem`

---

### 10.1 Overview

Add a three-level food chain:

```
Algae  →  Sardines eat algae  →  Sardines reproduce
Sardines  →  Sailfish eat sardines
Each predator starves if it goes too long without eating.
```

Population is self-regulating: sardines grow when algae is plentiful; sailfish keep sardine numbers in check; starvation provides a top-down population floor for predators.

---

### 10.2 Architecture changes

#### New file: `sim/algae.py` — `Algae(Boid)`

Algae **subclasses `Boid`** (the same base as `Sardine` and `Sailfish`). Rationale:

- `Boid` already handles position/velocity/acceleration, Euler integration, boundary wrapping, and speed clamping — all of which algae need.
- The `_compute_forces()` hook is exactly the override point for algae's light flocking.
- Algae must live in a spatial hash so sardines can query it during eating; the hash already works on any `Boid` subclass.
- A fully custom "lightweight particle" entity would save one `np.zeros(2)` allocation per algae per spawn (negligible) at the cost of duplicating all kinematics logic. Not worth it.

`Algae._compute_forces()` implements a **low-weight Reynolds flocking with no predator awareness**:
- Alignment-dominant (strong): gives the "drift with current" visual.
- Separation very weak (prevents overlap but doesn't scatter).
- Cohesion very weak (loose aggregate, not tight school).
- Moderate noise weight: organic drift feel.
- No flee behaviour. Algae does not react to sardines or sailfish.

Algae has `min_speed = 0.0` (can drift to near-stop in calm regions).

#### New per-agent state: `food_counter` and `hunger_timer`

Added as plain instance attributes on `Sardine` and `Sailfish` (not a new subclass). These are simple floats initialised in `__init__`:

| Attribute | Type | Sardine | Sailfish |
|---|---|---|---|
| `food_counter` | `float` | algae eaten (increments toward reproduction threshold) | sardines eaten (currently unused beyond hunger reset) |
| `hunger_timer` | `float` | seconds since last algae eaten | seconds since last sardine eaten |

Both agents are initialised with `hunger_timer = config.<species>_initial_hunger_time` so the full population does not immediately die at startup (the initial timer is below the starvation threshold).

#### New spatial hash: `_algae_grid`

A third `SpatialHash` owned by `Simulation`. Algae query it in `_compute_forces()` for flocking; the ecology pass queries it to find algae near each sardine.

Updated grid ownership in `Simulation`:

| Grid | Contents | Queried by |
|---|---|---|
| `_grid` | Sardines only | Sardines (flocking) + Sailfish (pursuit) |
| `_sailfish_grid` | Sailfish only | Sardines (flee) |
| `_algae_grid` *(new)* | Algae only | Algae (flocking) + Simulation ecology pass (eating) |

Algae need a reference to `_algae_grid` injected at construction (same pattern as Sardine receiving `_grid`).

#### New ecology pass in `Simulation.update()`

After all agents have called `agent.update(dt)`, a new ecology pass runs. This is the right place (not in `_compute_forces()`) because eating and dying modify the world's agent lists, which must not be mutated while iterating.

```python
# Ecology pass (runs after all agent.update(dt) calls):
1. Rebuild _algae_grid (same phase as sardine/sailfish grids at frame start)
2. Increment hunger_timer on all sardines and sailfish by dt
3. Sardine-algae eating:
     for each sardine:
         algae_nearby = _algae_grid.query(sardine, algae_eat_radius)
         for each algae in algae_nearby:
             mark algae for removal
             sardine.food_counter += 1
             sardine.hunger_timer = 0.0
4. Sailfish-sardine eating:
     for each sailfish:
         sardines_nearby = _grid.query(sailfish, sailfish_eat_radius)
         for each nearby sardine:
             mark sardine for removal (only first, or one per sailfish per frame)
             sailfish.food_counter += 1
             sailfish.hunger_timer = 0.0
             break  # one sardine per sailfish per tick (avoids mass kill)
5. Starvation:
     mark sardines where hunger_timer > sardine_starvation_time for removal
     mark sailfish where hunger_timer > sailfish_starvation_time for removal
6. Reproduction:
     for each sardine with food_counter >= sardine_eat_threshold:
         queue one offspring sardine at parent position + small offset
         sardine.food_counter = 0
7. Algae respawn:
     accrue fractional algae_spawn_accumulator += algae_respawn_rate * dt
     while accumulator >= 1.0: spawn one algae at random position; accumulator -= 1.0
8. Flush removals (iterate removal sets; remove from lists; remove Triangle shapes)
9. Flush additions (append offspring sardines; allocate shapes)
```

**Why one kill per sailfish per tick**: prevents a single sailfish from eating an entire nearby school in one frame if the query returns many fish. One eat per tick at 60 fps is already very fast relative to starvation times.

#### Renderer changes

`Renderer` needs a third entity pass. The existing pattern (one shape per agent in a single batch) extends naturally:

- `self.algae_shapes: list[pyglet.shapes.Circle]` — reddish-brown circles (`(170, 50, 20, 255)`), radius driven by `algae_size`.
- `_sync_circle_positions()` updates only `circle.x / circle.y` per frame (no heading math needed).
- `_make_algae_shapes()` allocates circles; `_make_shapes()` allocates triangles for sardines/sailfish.
- Algae use a fixed reddish-brown colour (no velocity-colour modes).
- HUD updated to show algae count alongside sardine/sailfish counts.

#### `Simulation.apply_config()` additions

Must propagate new algae kinematic config fields to all `Algae` agents and refresh cached values (same pattern as sardines/sailfish).

#### `Simulation.resize()` additions

Must wrap algae positions and update `_algae_grid.resize()` similarly to sardines and sailfish.

---

### 10.3 New config fields

All added to `SimConfig` in `config/config.py`.

#### Algae

| Field | Default | Notes |
|---|---|---|
| `num_algae` | `500` | Initial population |
| `algae_max_speed` | `30.0` | px/s; slow drift |
| `algae_min_speed` | `0.0` | Can stop |
| `algae_max_force` | `15.0` | px/s²; gentle steering |
| `weight_algae_alignment` | `1.5` | Dominant — currents feel |
| `weight_algae_separation` | `0.2` | Very weak |
| `weight_algae_cohesion` | `0.2` | Very weak |
| `weight_algae_noise` | `0.1` | Moderate organic drift |
| `radius_algae_separation` | `12.0` | px |
| `radius_algae_alignment` | `50.0` | px |
| `radius_algae_cohesion` | `55.0` | px |
| `algae_cell_size` | `55.0` | px; ≥ `radius_algae_cohesion` |
| `algae_size` | `4.0` | px; circle radius (smaller than sardines) |
| `algae_eat_radius` | `12.0` | px; contact distance for sardine eating |
| `algae_respawn_rate` | `3.0` | new algae per second (Simulation spawns them) |

#### Sardine ecology

| Field | Default | Notes |
|---|---|---|
| `sardine_eat_threshold` | `5` | Algae pieces eaten before reproducing |
| `sardine_starvation_time` | `20.0` | s; hunger_timer > this → death |
| `sardine_initial_hunger_time` | `10.0` | s; initial hunger_timer (< starvation); stagger startup deaths |

#### Sailfish ecology

| Field | Default | Notes |
|---|---|---|
| `sailfish_eat_radius` | `20.0` | px; contact distance for sailfish eating sardine |
| `sailfish_starvation_time` | `45.0` | s; longer than sardines — big predators last longer |
| `sailfish_initial_hunger_time` | `20.0` | s; initial hunger_timer |

**Note on algae color**: algae will use a fixed `(20, 180, 80, 255)` green colour. A `algae_color_mode` field can be added later if palette cycling is desired. A `algae_color` config tuple is not added immediately — it is hard-coded in the renderer for Phase 8 and promoted to config only if needed.

---

### 10.4 New statistics / metrics

#### HUD additions

| Metric | Where | Notes |
|---|---|---|
| Algae count | HUD, agents row | Third count after sardines/sailfish |
| Sardine births | HUD (optional, epoch) | Running total or sliding window |
| Sardine deaths (starvation) | HUD (optional, epoch) | |
| Sardines eaten (by sailfish) | HUD (optional, epoch) | |
| Sailfish deaths (starvation) | HUD (optional, epoch) | |

For Phase 8 launch, adding only the algae count to the HUD is sufficient. The per-epoch birth/death counters are tracked internally by `Simulation` (simple int attributes) and accessible for future HUD or logging expansion.

#### `Simulation` internal counters (new attributes)

```python
self.stats_sardine_births:        int = 0
self.stats_sardine_starvations:   int = 0   # ✅ implemented
self.stats_sardines_eaten:        int = 0
self.stats_sailfish_starvations:  int = 0   # ✅ implemented
```

These increment each ecology pass; never reset (monotonic). The renderer/HUD can read them as needed.

Side-channels (cleared every frame, read by renderer):

```python
self._sardines_eaten_ids:   set[int]  # predation deaths
self._sardines_starved_ids: set[int]  # starvation deaths  ✅ implemented
self._sailfish_died_ids:    set[int]  # sailfish starvation ✅ implemented
self._sardine_born:         list      # new offspring
```

---

### 10.5 Staged implementation order

Each stage is independently testable. Do not merge a stage until its tests pass.

| Stage | What | Key files changed |
|---|---|---|
| **8.1** ✅ | `Algae(Boid)` entity + `_algae_grid` + spawning + algae flocking (no eating) | `sim/algae.py` (new), `sim/simulation.py`, `config/config.py` |
| **8.2** ✅ | Renderer algae pass (green triangles, HUD algae count) | `rendering/renderer.py` |
| **8.3** ✅ | Sardine `food_counter` + `hunger_timer`; ecology pass: sardine-algae eating; algae removal; `Eaten ↑N` HUD | `sim/sardine.py`, `sim/simulation.py`, `rendering/renderer.py`, `config/config.py` |
| **8.4** ✅ | Sardine reproduction (`food_counter >= threshold` → offspring spawn, counter decremented) | `sim/simulation.py`, `rendering/renderer.py`, `config/config.py` |
| **8.5** ✅ | Sardine starvation | `sim/simulation.py`, `config/config.py`, `sim/sardine.py`, `rendering/renderer.py` |
| **8.6** ✅ | Sailfish `food_counter` + `hunger_timer`; ecology pass: sailfish-sardine eating; sardine removal | `sim/sailfish.py`, `sim/simulation.py`, `rendering/renderer.py`, `config/config.py` |
| **8.7** ✅ | Sailfish starvation | `sim/simulation.py`, `config/config.py`, `sim/sailfish.py`, `rendering/renderer.py` |
| **8.8** | `apply_config()` and `resize()` extended for algae | `sim/simulation.py` |
| **8.9** | Config menu rows for key ecosystem params | `config/config_menu.py` |
| **8.10** | Ecosystem stat counters + optional HUD expansion | `sim/simulation.py`, `rendering/renderer.py` |

**Recommended order rationale**: Stages 8.1–8.2 establish the algae visual layer with zero gameplay change, making it safe to ship and demo. Stages 8.3–8.5 add sardine-side ecology. Stages 8.6–8.7 add sailfish-side ecology. Only after both sides work do we wire up config menu and stats.

---

### 10.6 Testing plan

New test suites to add to `tests/tests.py` (alongside the 53 existing tests).

#### `TestAlgaeKinematics` (Stage 8.1)

| Test | What is verified |
|---|---|
| `test_algae_moves_after_update` | `algae.update(dt)` advances position |
| `test_algae_respects_max_speed` | speed never exceeds `algae_max_speed` |
| `test_algae_boundary_wrap` | position wraps at world edges |
| `test_algae_can_stop` | `min_speed=0` means speed can reach 0 |
| `test_algae_grid_insertion` | algae correctly inserted and queried from `SpatialHash` |
| `test_algae_flocking_alignment` | two aligned algae do not diverge in heading (net force pushes toward alignment) |

#### `TestEcologyContactDetection` (Stage 8.3)

| Test | What is verified |
|---|---|
| `test_sardine_eats_algae_on_contact` | sardine within `algae_eat_radius` → `food_counter` increments |
| `test_sardine_does_not_eat_distant_algae` | sardine outside radius → no increment |
| `test_eaten_algae_marked_for_removal` | algae is added to removal set on contact |
| `test_hunger_timer_increments_with_dt` | `hunger_timer` grows by `dt` each frame |
| `test_hunger_timer_resets_on_eat` | eating algae resets `hunger_timer` to 0 |

#### `TestSardineReproduction` (Stage 8.4)

| Test | What is verified |
|---|---|
| `test_reproduction_when_threshold_met` | sardine with `food_counter == eat_threshold` spawns offspring |
| `test_food_counter_resets_after_reproduction` | parent's `food_counter` is 0 after split |
| `test_offspring_near_parent` | offspring position is close to parent (not random) |
| `test_no_reproduction_below_threshold` | `food_counter < threshold` → no new sardine |
| `test_population_grows_after_eating` | sardine count increases over a simulated eating sequence |

#### `TestSardineStarvation` (Stage 8.5)

| Test | What is verified |
|---|---|
| `test_sardine_removed_when_starved` | `hunger_timer > starvation_time` → sardine removed from simulation |
| `test_sardine_survives_below_threshold` | `hunger_timer < starvation_time` → sardine stays |
| `test_algae_respawn_adds_algae` | over N frames, algae count rises toward `num_algae` when no sardines present |

#### `TestSailfishEcology` (Stages 8.6–8.7)

| Test | What is verified |
|---|---|
| `test_sailfish_eats_sardine_on_contact` | sailfish within `sailfish_eat_radius` → sardine removed, `food_counter` increments |
| `test_sailfish_hunger_timer_increments` | `hunger_timer` grows each frame |
| `test_sailfish_hunger_resets_after_eat` | eating resets hunger to 0 |
| `test_sailfish_removed_when_starved` | `hunger_timer > sailfish_starvation_time` → removal |
| `test_sailfish_does_not_eat_distant_sardine` | outside radius → no kill |

#### `TestEcosystemIntegrity` (Stage 8.10, integration)

| Test | What is verified |
|---|---|
| `test_ecosystem_does_not_collapse_early` | at default params, all three species co-exist for at least N simulated seconds |
| `test_sardine_population_bounded` | sardine count does not grow unboundedly with unlimited algae |
| `test_algae_population_recovers` | after removing all sardines, algae respawn to near `num_algae` within T seconds |
| `test_stat_counters_increment` | `stats_sardine_births`, `stats_sardines_eaten`, etc. accumulate correctly |

---

### 10.7 Known design constraints and open questions

- **Menu height overflow**: the config menu is already ~528 px with 21 rows. Ecosystem adds ~8–10 new tunable parameters. A scroll mechanism or a second "Ecosystem" config panel will be needed. Plan: add a horizontal page-switch (`[` / `]`) to cycle between KINEMATICS and ECOSYSTEM pages, each independently scrollable.
- **Algae respawn rate vs. population**: tuning is needed. If `algae_respawn_rate` is too low, sardines eat algae to extinction; too high, algae overflow the renderer. The `num_algae` target could be a soft cap, with spawning paused above it.
- **Sardine minimum population floor**: if all sardines starve before eating enough algae to reproduce, the simulation empties. An initial hunger time (`sardine_initial_hunger_time`) partially mitigates this; a `sardine_starvation_enabled: bool` toggle is a useful debug escape hatch.
- **Sailfish solo start**: at 8 sailfish with `sailfish_starvation_time = 45 s` and ~900 sardines, sailfish should survive comfortably. If sardine population collapses first, sailfish will starve. This is ecologically correct but may need tuning.
- **Offspring placement**: reproduction offspring should be placed at parent position + small random offset (< sardine body length), not at a random world position, so the school effect is preserved. If spawned exactly at the parent, they'll immediately separate due to separation force — which is fine.
- **One-eat-per-sailfish-per-tick limitation**: prevents mass kill but means a sailfish surrounded by 20 sardines eats only 1/tick. This is biologically reasonable and prevents population crashes from tight-school contacts.
