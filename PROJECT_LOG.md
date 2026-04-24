# PROJECT_LOG.md — Sardine Schooling & Sailfish Hunting Simulation

This is the authoritative current-state document for active development.

Use this file first for:
- architecture and runtime data flow
- current behavior and feature set
- config and UI wiring
- current constraints and next work

Use supporting files for detail:
- historical milestones and completed phase changelists: [memory/ARCHIVE.md](memory/ARCHIVE.md)
- implementation patterns, profiling notes, and engineering heuristics: [memory/DEV_NOTES.md](memory/DEV_NOTES.md)
- stable conventions and quick team memory: [memory/MEMORY.md](memory/MEMORY.md)

---

## Log Maintenance (Chronological)

- 2026-04 (early): Log hygiene pass. Corrected test counts to 268 tests across 23 suites, removed stale TODOs, and moved old optimization archaeology out of this file to [memory/ARCHIVE.md](memory/ARCHIVE.md).  
  Why: keep the active log readable.  
  Where: [PROJECT_LOG.md](PROJECT_LOG.md), [memory/ARCHIVE.md](memory/ARCHIVE.md).

- 2026-04 (mid): Added soft population caps for sardines and sailfish (`sardine_max_count`, `sailfish_max_count`), enforced in reproduction paths and exposed in ECOLOGY menu rows.  
  Why: prevent unbounded autonomous population growth.  
  Where: [sim/simulation.py](sim/simulation.py), [config/config.py](config/config.py), [config/config_menu.py](config/config_menu.py).

- 2026-04-13 (earlier): Added fish lifetime timers (`age`) and continuous age updates for sardines and sailfish in simulation runtime seconds.  
  Why: provide stable lifetime signal for behavior/visualization.  
  Where: [sim/sardine.py](sim/sardine.py), [sim/sailfish.py](sim/sailfish.py), [sim/simulation.py](sim/simulation.py).

- 2026-04-13 (later): Sardine color architecture refactor from combined mode to driver + palette:
  - driver: `sardine_color_driver` (`velocity` | `age`)
  - palette: `sardine_color_palette` (`white`, `rainbow`, `cool`, `warm`, `ocean`, `sunset`, `mono`)
  Legacy `sardine_color_mode` and `use_velocity_coloring` now migrate through config IO.  
  Why: separate scalar source from palette mapping and remove age-as-theme coupling.  
  Where: [rendering/renderer.py](rendering/renderer.py), [config/config.py](config/config.py), [config/config_io.py](config/config_io.py), [config/config_menu.py](config/config_menu.py), [main.py](main.py).

- 2026-04-13 (cleanup): Moved low-level optimization/test-pattern detail from this file to [memory/DEV_NOTES.md](memory/DEV_NOTES.md).  
  Why: preserve high-signal current-state documentation in this file.  
  Where: [PROJECT_LOG.md](PROJECT_LOG.md), [memory/DEV_NOTES.md](memory/DEV_NOTES.md).

---

## 1) Project Overview

Real-time 2D ecosystem simulation of sardine schooling, sailfish predation, and algae drift/feeding.

Runtime and validation:
- Run: `python main.py`
- Tests: `python -m unittest tests.tests` (268 tests)
- Benchmarks: `python -m tests.bench`

Core capabilities:
- Sardine flocking: separation/alignment/cohesion/noise.
- Sailfish hunting: school COM pursuit, individual target lock, burst, vulnerability scoring, school-pressure circling.
- Three-level ecology: algae -> sardines -> sailfish, with feeding, starvation, reproduction, and soft caps.
- Sardine coloring: driver/palette split (velocity or age driver + independent palette).
- Runtime configuration menu with live editing and persistence.
- HUD diagnostics and optional population graph.

---

## 2) Architecture

Workspace layout (current):
- [main.py](main.py): app wiring, input bindings, update/draw loop.
- [config/config.py](config/config.py): authoritative `SimConfig` dataclass.
- [config/config_io.py](config/config_io.py): overrides load/save + migrations.
- [config/config_menu.py](config/config_menu.py): runtime config UI.
- [sim/simulation.py](sim/simulation.py): world owner, update loop, ecology passes.
- [sim/sardine.py](sim/sardine.py), [sim/sailfish.py](sim/sailfish.py), [sim/algae.py](sim/algae.py): entity behavior.
- [sim/spatial_hash.py](sim/spatial_hash.py): neighbor partitioning.
- [rendering/renderer.py](rendering/renderer.py): batch rendering, HUD, graph, sardine color pipeline.

One-frame data flow:
1. Rebuild species grids.
2. Compute/update agent state.
3. Run ecology passes (feeding, starvation, reproduction, respawn).
4. Render from simulation state through batched draw path.

Grid model:
- sardine grid: sardine flocking + sailfish prey query
- sailfish grid: sardine flee query
- algae grid: algae flocking + sardine food query

---

## 3) Current Behavior (Authoritative)

Sardines:
- Flock with Reynolds rules.
- Flee sailfish and active mouse predator.
- Eat algae, reproduce by `sardine_eat_threshold`, starve by `sardine_starvation_time`.
- Track `hunger_timer`, `food_counter`, and `age` (seconds).

Sailfish:
- Pursue sardine schools, lock vulnerable individuals, burst attack near target.
- Eat sardines (one kill per sailfish per tick), reproduce by `sailfish_eat_threshold`, optional starvation toggle.
- Track `hunger_timer`, `food_counter`, and `age` (seconds).

Algae:
- Slow alignment-dominant drift in own grid.
- Respawn through accumulator up to `algae_max_count`.

Rendering:
- Single batch for fish/algae shapes.
- HUD + optional population graph overlays.
- Sardine color transition resets are handled to avoid stale-color carryover.

---

## 4) Sardine Color Pipeline (Current)

Configuration split:
- `sardine_color_driver`
  - `velocity`: scalar from heading/speed/turn channels.
  - `age`: scalar from `min(age / sardine_age_gradient_seconds, 1.0)`.
- `sardine_color_palette`
  - `white`, `rainbow`, `cool`, `warm`, `ocean`, `sunset`, `mono`.

Pipeline contract:
1. Compute normalized driver scalar.
2. Apply selected palette mapping to that scalar.

Behavior notes:
- `white` bypasses dynamic color writes.
- Restricted palettes use cyclic mapping to avoid wrap flicker.
- `mono` maps scalar intensity to brightness.

Migration behavior:
- Old `use_velocity_coloring: true` -> velocity driver + rainbow palette.
- Old `sardine_color_mode` -> best-effort mapping into driver/palette fields.

---

## 5) Config and UI (Current)

Runtime menu:
- 60 data rows across 7 sections + Save row.
- Sections: SARDINES, MOUSE PREDATOR, SAILFISH — FLOCKING, SAILFISH — HUNTING, SAILFISH — BURST, ALGAE, ECOLOGY.
- Visible rows: 16, with scrolling indicators.

Sardine color controls in menu:
- Color driver (velocity/age)
- Color palette (7 palettes)
- Velocity channel weights
- Age gradient seconds

Persistence:
- Save to [boids_overrides.json](boids_overrides.json) via atomic write.
- Load with type coercion + enum validation + migration support.

Keyboard controls (global):
- `V`: cycle sardine color palette
- `TAB`: toggle config menu
- `H`: toggle HUD
- `D`: toggle population graph
- `F`: toggle fullscreen
- `ESC`: close menu or quit

---

## 6) Known Constraints

- Menu still does not expose direct editable rows for `num_sardines` / `num_sailfish`.
- Cross-field radius constraints are not hard-enforced in menu logic.
- `cell_size` is not auto-adjusted when radii are changed live.
- No in-app undo/reset-to-defaults flow.

---

## 7) Distribution Notes (What Moved)

To keep this file concise and current-state focused:
- Detailed phase-by-phase history and legacy implementation narrative moved to [memory/ARCHIVE.md](memory/ARCHIVE.md).
- Detailed performance pass mechanics, profiling lessons, cache-checklists, and test-pattern notes moved to [memory/DEV_NOTES.md](memory/DEV_NOTES.md).
- Persistent quick-reference conventions kept in [memory/MEMORY.md](memory/MEMORY.md).

---

## 8) References

- Historical changelists: [memory/ARCHIVE.md](memory/ARCHIVE.md)
- Implementation patterns and engineering notes: [memory/DEV_NOTES.md](memory/DEV_NOTES.md)
- Stable project memory: [memory/MEMORY.md](memory/MEMORY.md)
- Tests: [tests/tests.py](tests/tests.py)
- Benchmarks: [tests/bench.py](tests/bench.py)
- Config baseline: [config/config.py](config/config.py)
