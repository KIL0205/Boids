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

- 2026-05-03: Sailfish state machine implemented using `transitions` (pytransitions).  
  Why: give sailfish explicit, readable behavioral phases instead of implicit flag-based logic.  
  What changed:
  - [sim/sailfish.py](sim/sailfish.py): full rewrite with `transitions.Machine` on each instance. Five states: PATROL → TRACK_SCHOOL → TARGET_LOCK → BURST_ATTACK → RECOVER. `_compute_forces` restructured as state-driven blocks using plain `if`-chains so single-frame transitions fall through correctly. Target selection and school-pressure extracted to private helpers. Added `_no_sardines_timer`, `_recovery_timer`, `_dt` fields.
  - [sim/simulation.py](sim/simulation.py): ecology pass extended to fire timer-based transitions: burst end → RECOVER (`sm_to_recover`), recovery end → TARGET_LOCK or TRACK_SCHOOL, no-sardines timeout → PATROL (`sm_to_patrol`).
  - [config/config.py](config/config.py): three new fields: `sailfish_recovery_duration` (1.5s), `sailfish_prediction_time` (0.5s, replaces hardcoded cap), `sailfish_no_sardines_timeout` (5.0s).
  - [config/config_menu.py](config/config_menu.py): three new menu rows added to SAILFISH — HUNTING (Predict time, No-sar timeout) and SAILFISH — BURST (Recovery dur). Row count: 72.
  - [rendering/renderer.py](rendering/renderer.py): HUD sailfish line now shows per-state counts `[P# T# L# B# R#]` (Patrol/Track/Lock/Burst/Recover) at 4 Hz.  
  Verification: 268/268 tests pass. Smoke test confirms all 5 states are reachable, sardines are eaten, sailfish reproduce, invalid triggers silently ignored.

- 2026-05-03: Synced default `SimConfig` values with active overrides in [boids_overrides.json](boids_overrides.json).  
  Why: make tuned runtime values the new baseline while keeping override support intact.  
  What changed: updated default population, sardine flocking/color, sailfish hunting/targeting, flee, and ecology cap/threshold values in [config/config.py](config/config.py) to match the active override file (excluding runtime-only window dimensions).

- 2026-05-03: Added cooperative sailfish baitball herding and BAITBALL_STRIKE state.  
  Why: make sailfish hunt dense sardine schools as a coordinated team rather than as independent individuals.  
  What changed:
  - [sim/simulation.py](sim/simulation.py): Added `_BaitballInfo` dataclass (cx, cy, radius, count, density, bid). Added `_detect_baitballs()` which scans occupied sardine-grid cells and their spatial neighborhoods each frame without pairwise O(n²) work. Detected baitballs are pushed to each sailfish via `sf.set_baitball_context(baitballs, sf_count)` before `sf.update()`. Added `debug_baitball_count` and `debug_herding_sailfish` counters. Strike timer countdown for `baitball_strike` state handled in ecology pass.
  - [sim/sailfish.py](sim/sailfish.py): Added sixth state `baitball_strike`. Transitions: `sm_to_baitball_strike` from TRACK_SCHOOL or TARGET_LOCK; `sm_to_recover` extended to include BAITBALL_STRIKE as source. New instance vars: `_baitballs_frame`, `_sailfish_count` (set by `set_baitball_context`), `_baitball`, `_baitball_memory_timer`, `_is_herding`, `_sid` (stable pseudo-ID for slot assignment), `_baitball_strike_timer`, `_baitball_strike_cooldown_timer`, `_baitball_strike_exit_x/y`. Herding pressure (`_apply_herding_pressure`) is applied in TRACK_SCHOOL whenever a baitball is active, supplementing pursuit forces. Angular slot assignment distributes sailfish evenly around baitball perimeter. `_begin_baitball_strike` computes a through-baitball exit point. BAITBALL_STRIKE speed/force multipliers applied in `update()`.
  - [config/config.py](config/config.py): 17 new fields across three groups — baitball detection (`sailfish_baitball_enabled`, `_detection_radius`, `_min_sardines`, `_density_threshold`, `_memory_seconds`), herding (`sailfish_herding_enabled`, `_radius_multiplier`, `_pressure_strength`, `_slot_jitter`, `_min_sailfish`), and strike (`sailfish_baitball_strike_enabled`, `_density_threshold`, `_chance`, `_speed_multiplier`, `_duration_seconds`, `_cooldown_seconds`, `_exit_distance`). Also fixed `sailfish_initial_hunger_time` to 7.0 (must be < `sailfish_starvation_time=14.0`).
  - [config/config_menu.py](config/config_menu.py): Added new SAILFISH — BAITBALL section and rows for all new fields. Updated `_NUM_ROWS`.
  - [rendering/renderer.py](rendering/renderer.py): HUD sailfish line now shows `[P# T# L# B# R# S#]` (S = baitball_strike) plus `BB:# Herd:#` debug counters when nonzero.
  Verification: 271/271 tests pass.

- 2026-05-03: Refined sailfish baitball behavior to be patience-first and orbit-cooperative.  
  Why: reduce chaotic individual lock-ons near baitballs and favor sustained coordinated compression before deliberate strikes.  
  What changed:
  - [sim/sailfish.py](sim/sailfish.py): Added `orbit_baitball` state and transitions for cooperative orbit flow (`TRACK_SCHOOL → ORBIT_BAITBALL`, `ORBIT_BAITBALL → BAITBALL_STRIKE|TARGET_LOCK|TRACK_SCHOOL`). Added patience gates (`_orbit_timer`, `_orbit_patience_goal`) and strike-mode split: center pass-through (`baitball_strike`) vs edge-target lock. Added suppression for non-edge `target_lock` when baitball cooperation is active.
  - [sim/simulation.py](sim/simulation.py): Added group strike cooldown coordination and new debug/stats counters (`debug_orbit_sailfish`, `debug_baitball_max_density`, `debug_group_strike_cooldown`, `stats_center_strikes`, `stats_edge_strikes`). Recovery exit now prefers ORBIT_BAITBALL when baitball context is still valid.
  - [config/config.py](config/config.py): Added cooperative pacing/orbit fields (`sailfish_target_lock_suppressed_near_baitball`, baitball patience/min-orbit/jitter/group-cooldown, edge/center strike chance split, edge band width, orbit tangent/inward/rotation controls). Tuned defaults toward longer orbit windows and less frequent strikes.
  - [config/config_menu.py](config/config_menu.py): Added rows for new baitball suppression, patience, strike split, and orbit tuning parameters under SAILFISH — HUNTING.
  - [rendering/renderer.py](rendering/renderer.py): HUD sailfish state counts now include orbit (`O#`) and show baitball density, orbiting count, strike counters, and group cooldown.
  - [tests/tests.py](tests/tests.py): Added and updated cooperative behavior tests for orbit participation, edge-target filtering, and recover-to-orbit behavior.
  Verification: 273/273 tests pass.

---

## 1) Project Overview

Real-time 2D ecosystem simulation of sardine schooling, sailfish predation, and algae drift/feeding.

Runtime and validation:
- Run: `python main.py`
- Tests: `python -m unittest tests.tests` (273 tests)
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
- Seven-state machine (PATROL → TRACK_SCHOOL → ORBIT_BAITBALL → TARGET_LOCK or BAITBALL_STRIKE → BURST_ATTACK/RECOVER) managed by `transitions.Machine` on each instance.
- PATROL: wander + sailfish flocking until sardines appear within awareness radius.
- TRACK_SCHOOL: COM pursuit + vulnerability-scored target acquisition; transitions into ORBIT_BAITBALL when a useful dense baitball exists.
- ORBIT_BAITBALL: distributed slot orbit around baitball perimeter with tangential movement and mild inward pressure; deliberate strikes are patience/cooldown gated.
- TARGET_LOCK: stable single-target lock with predictive interception (`prediction_time` cap). Near active baitballs, lock is suppressed unless the target is on the outer edge.
- BURST_ATTACK: same interception at burst speed/force multipliers. Transitions to RECOVER when burst timer expires.
- BAITBALL_STRIKE: high-speed center pass-through across baitball. Used as one deliberate strike mode after orbit patience.
- RECOVER: post-burst/strike recovery; exits to ORBIT_BAITBALL when baitball context remains valid, otherwise to TRACK_SCHOOL/TARGET_LOCK.
- Cooperative herding: each frame `simulation.py` detects dense sardine clusters (`_detect_baitballs`) and pushes `_BaitballInfo` to each sailfish. Sailfish in ORBIT_BAITBALL use stable slot assignment + orbit rotation to maintain distributed pressure.
- All existing ecology (eating, hunger, starvation, reproduction, soft cap) is unchanged.
- HUD shows per-state counts `[P# T# O# L# B# X# R#]` and baitball/orbit/strike debug counters at 4 Hz.

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
- Data rows across 8 sections + Save row.
- Sections: SARDINES, MOUSE PREDATOR, SAILFISH — FLOCKING, SAILFISH — HUNTING, SAILFISH — BURST, SAILFISH — BAITBALL, ALGAE, ECOLOGY.
- SAILFISH — HUNTING: Predict time (`sailfish_prediction_time`), No-sar timeout (`sailfish_no_sardines_timeout`).
- SAILFISH — BURST: Recovery dur (`sailfish_recovery_duration`).
- SAILFISH — BAITBALL: all baitball detection, herding, and strike config fields.
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
