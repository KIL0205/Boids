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

- 2026-05-07: Refined sailfish baitball behavior — anti-conga-line orbit, lateralization, proto-collaboration, baitball anchoring, attack pressure suppression, inter-sailfish spacing.  
  Why: sailfish were forming conga-line orbits (synchronized ring rotation) and all striking simultaneously; needed independent hunters using shared opportunity with patient turn-taking.  
  What changed:
  - [sim/sailfish.py](sim/sailfish.py): Per-fish orbit vars (`_orbit_angle_accum`, `_orbit_speed_mult`, `_orbit_dir`, `_orbit_radius_mult`) initialized at spawn with random jitter; each fish accumulates its own orbit angle independently so fish drift apart instead of rotating in lockstep. Stable per-fish lateralization (`attack_side`, `lateralization_bias`) set at spawn; biases `_select_edge_target` toward the preferred flank and offsets the center pass-through corridor in `_begin_baitball_strike`. Proto-collaboration: `_individual_strike_cooldown` timer per fish; `_is_baitball_attacker` flag; both `_should_start_baitball_strike` and the edge-strike gate now check `sailfish_max_simultaneous_baitball_strikes` cap and per-fish cooldown. `set_baitball_context` accepts `active_baitball_strikes` count. `_mark_group_strike` sets per-fish cooldown and attacker flag. `_apply_herding_pressure` fully replaced: uses per-fish orbit angle accumulator; attack pressure suppression (`sailfish_attack_pressure_suppression`) scales herding to near-zero for the attacking fish; inward pressure capped (`sailfish_baitball_max_inward_pressure`) and linearly tapers to zero inside falloff radius (`sailfish_baitball_inner_pressure_falloff`); orbit direction set per-fish (`_orbit_dir`); drift damping estimates baitball velocity frame-to-frame and applies counter-force; inter-sailfish spacing (`sailfish_spacing_enabled`) pushes nearby orbiting fish apart laterally with following penalty to break conga-line following.
  - [sim/simulation.py](sim/simulation.py): Counts active baitball attackers each frame and injects into `set_baitball_context`. Ecology timer pass decrements `_individual_strike_cooldown` per fish and clears `_is_baitball_attacker` when cooldown expires and fish is not in an attack state. `apply_config` resets all new per-fish collaboration/tracking fields.
  - [config/config.py](config/config.py): Added 18 new fields — orbit variation (`sailfish_orbit_speed_jitter`, `sailfish_orbit_radius_jitter`, `sailfish_orbit_ccw_fraction`), lateralization (`sailfish_lateralization_enabled`, `sailfish_lateralization_strength`), proto-collaboration (`sailfish_max_simultaneous_baitball_strikes`, `sailfish_individual_strike_cooldown_seconds`), baitball anchor (`sailfish_baitball_anchor_enabled`, `sailfish_baitball_drift_damping_strength`, `sailfish_baitball_max_inward_pressure`, `sailfish_baitball_inner_pressure_falloff`), attack suppression (`sailfish_attack_pressure_suppression`), spacing (`sailfish_spacing_enabled`, `sailfish_spacing_radius`, `sailfish_spacing_strength`, `sailfish_following_penalty_strength`). Retuned orbit defaults: patience 2.4→3.5s, group cooldown 2.2→4.5s, orbit radius mult 1.55→1.70, inward pressure 0.34→0.18, rotation speed 0.52→0.32, tangent strength 1.05→0.90, edge band 14→18px, center strike chance 0.08→0.06.
  - [config/config_menu.py](config/config_menu.py): Added 16 new rows for all new fields under the SAILFISH — BAITBALL section.
  - [boids_overrides.json](boids_overrides.json): Enabled `sailfish_baitball_enabled: true`. Added all new field values. Updated orbit patience/cooldown/pressure/rotation defaults to match config baseline. Both files kept in sync.
  Remaining tuning notes: `sailfish_orbit_speed_jitter`, `sailfish_spacing_strength`, and `sailfish_individual_strike_cooldown_seconds` are the most impactful knobs for spacing feel. `sailfish_baitball_drift_damping_strength` controls baitball anchoring aggressiveness. Per-fish `attack_side` / `lateralization_bias` are set at spawn and not re-rolled on config change — this is intentional.  
  Verification: 273/273 tests pass.

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

- 2026-05-07: Baitball pressure tuning pass + dead config cleanup.  
  Why: analysis revealed two dead config fields (never read by sailfish.py), and baitball pressure was producing over-compressed, drifting schools.  
  What changed:
  - [sim/sailfish.py](sim/sailfish.py): Wired `sailfish_baitball_inward_pressure_strength` into the inward pressure calculation. Previously the inward force used `sailfish_baitball_max_inward_pressure` as both the base value and cap; now uses `min(inward_pressure_strength, max_inward_pressure)` so the strength field is the primary tuning dial (accessible via config menu "Orbit inward") and max_inward_pressure is a safety ceiling.
  - [config/config.py](config/config.py): Updated comments on `sailfish_baitball_inward_pressure_strength` (now documented as primary dial) and `sailfish_baitball_max_inward_pressure` (now documented as hard safety ceiling on top of strength). Marked `sailfish_herding_radius_multiplier` as DEPRECATED with a clear explanation — it was never referenced in sailfish.py; the effective orbit radius multiplier is `sailfish_baitball_orbit_radius_multiplier`. Field is retained in the dataclass only for backward JSON compatibility. Updated four tuning values (see below).
  - [boids_overrides.json](boids_overrides.json): Updated four tuning values to match.
  - Tuning changes (applied in both config.py defaults and boids_overrides.json):

    | Field | Before | After | Effect |
    |---|---|---|---|
    | `sailfish_baitball_orbit_radius_multiplier` | 1.70 | 1.95 | More standoff; sardines less likely to flee during orbit |
    | `sailfish_baitball_max_inward_pressure` | 0.38 | 0.16 | Hard cap reduced; inward compression cut by ~58% |
    | `sailfish_baitball_inner_pressure_falloff` | 0.70 | 0.85 | Wider taper zone; fish at typical orbit distance get lower effective inward force |
    | `sailfish_baitball_drift_damping_strength` | 0.45 | 0.60 | Stronger anchoring to counteract residual drift from asymmetric orbit forces |

  Note on `sailfish_herding_radius_multiplier`: field retained as deprecated dead weight (frozen dataclass cannot drop fields without breaking JSON override files that include it). `config_io.py` silently skips it on load since it remains in the dataclass type hints; no migration pass needed.  
  Verification: 273/273 tests pass.

- 2026-05-07: Improved baitball detection to merge same-school duplicate candidates.  
  Why: the per-cell detector generates one candidate per occupied sardine grid cell; a single large school (50–200 sardines) could occupy 30–100 cells, each independently passing the baitball threshold, producing many overlapping false-separate baitballs. The old inline merge used `min(r1, r2) × 0.75` which was far too conservative to consolidate same-school candidates whose centers were often 80–150 px apart.  
  Strategy: post-detection greedy merge (not blob detection). The candidate list is small (O(occupied cells)), so an O(C²) merge is cheap even at high sardine counts (~100 candidates → 10 k iterations, negligible). Blob detection was considered but the existing per-cell candidate list is already a good approximation; restructuring around connected components would require larger refactor for minimal quality gain.  
  What changed:
  - [sim/simulation.py](sim/simulation.py):
    - Removed the too-conservative inline merge loop from `_detect_baitballs`. All candidates that pass the density threshold are now collected without pre-filtering.
    - Added `_merge_baitball_candidates(candidates, W, H, merge_dist_mult, overlap_thresh, max_final)` private method. Greedy, score-sorted (density × count descending). Two merge tests (either may trigger): `center_dist < merge_dist_mult × max(r1, r2)` and `center_dist < (r1 + r2) × overlap_thresh`. Merged baitball uses count-weighted center (toroidal-correct), enclosing radius computed as `max(d × w_ex + r_ex, d × w_c + r_c)` (exact — no sardine re-scan), density = combined_count / new_radius. Cap enforced at `max_final` distinct baitballs; candidates beyond the cap are still checked for merging into existing baitballs.
  - [config/config.py](config/config.py): Added 3 new fields in the BAITBALL DETECTION section:
    - `sailfish_baitball_merge_distance_multiplier = 1.2` — proximity test: merge if `center_dist < 1.2 × max(r1, r2)`.
    - `sailfish_baitball_merge_overlap_threshold = 0.55` — overlap test: merge if `center_dist < 0.55 × (r1 + r2)`.
    - `sailfish_baitball_max_final_baitballs = 4` — replaces the old hardcoded `[:4]` trim.
  - [boids_overrides.json](boids_overrides.json): Added all 3 new fields at their default values.
  Tuning notes: `merge_distance_multiplier=1.2` handles compact round schools (merge if centers within 1.2× larger radius). `merge_overlap_threshold=0.55` handles elongated or non-uniform schools where radii differ. Raise `merge_distance_multiplier` to 1.5–2.0 if elongated schools still fragment. Lower `merge_overlap_threshold` to 0.4 if genuinely separate adjacent schools are accidentally merged. `max_final_baitballs=4` is unchanged from the previous cap — raise if large worlds reliably have 5+ distinct dense schools.  
  Follow-up ideas: (1) Temporal smoothing of merged baitball centers (EMA over 3–5 frames) would further reduce jitter on diffuse or transitioning schools. (2) A `debug_baitball_raw_candidates_enabled` toggle could be added to the overlay if raw-vs-merged comparison is needed during tuning.  
  Verification: 273/273 tests pass.

- 2026-05-07: Fixed `TypeError: Line.__init__() got an unexpected keyword argument 'width'` crash in debug overlay.  
  Why: `pyglet.shapes.Line` in Pyglet 2.1.14 uses `thickness=` not `width=`. All six `pyglet.shapes.Line(...)` calls in [rendering/debug_overlay.py](rendering/debug_overlay.py) were using the unsupported `width=` keyword.  
  What changed: replaced `width=1` and `width=2` with `thickness=1` / `thickness=2` across all six Line constructors. Matches the pattern already used in renderer.py's PopGraph line drawing.  
  Verification: 273/273 tests pass.

- 2026-05-07: Fixed `AttributeError: 'Sailfish' object has no attribute 'x'` crash in debug overlay.  
  Why: `debug_overlay.py` was written with `sf.x` / `sf.y` / `target.x` / `target.y`, but all Boid subclasses store position as a numpy array (`boid.position`, a `Vec2 = np.ndarray`), accessed via `position[0]` / `position[1]`.  
  What changed:
  - [rendering/debug_overlay.py](rendering/debug_overlay.py): Added module-level `_pos(boid)` helper that returns `(float(boid.position[0]), float(boid.position[1]))`. All five sub-renderers updated to use `_pos(sf)`, `_pos(target)` instead of `.x`/`.y`. Stale sardine target guard (`_draw_strike_paths`) strengthened with `hasattr(target, 'position')` check in addition to `is not None`, protecting against eaten sardines whose reference may linger for one frame. Removed unused `import math`.  
  Verification: 273/273 tests pass.

- 2026-05-07: Added optional debug visualization overlay for baitball / herding behavior.  
  Why: orbit dynamics, slot assignments, and strike sequencing were not observable at runtime; diagnosing tuning problems required printf-debugging or reading simulation state in a debugger.  
  What changed:
  - [rendering/debug_overlay.py](rendering/debug_overlay.py): New file. `BaitballDebugOverlay` class with a stateless `draw(simulation, cfg)` method. Draws all visuals outside the Pyglet batch (immediate `.draw()` calls) so they always appear on top. Six visual sub-systems, each individually gated:
    1. **BB shading** — translucent filled circle + perimeter ring + center dot per baitball. Opacity scales with density score.
    2. **BB labels** — small monospace text label above each baitball (sardine count, density, radius).
    3. **SF–BB links** — line from each herding/striking sailfish to its baitball center. Color varies by state: dim cyan (orbit/track), bright cyan (target_lock), yellow (active strike).
    4. **Orbit slot markers** — green cross + stub line showing each sailfish's computed orbit slot target. Uses `sf.debug_orbit_slot_x/y` written by `_apply_herding_pressure`.
    5. **Strike paths** — yellow exit line for center-pass baitball strikes; red target highlight + link for edge-target (TARGET_LOCK) strikes.
    6. **State labels** — small PTRL/TRCK/ORBT/LOCK/BRST/STRK/RCVR abbreviation above each sailfish. Independently gated by `debug_sailfish_state_labels_enabled` (does not require master toggle).
  - [sim/sailfish.py](sim/sailfish.py): Added `debug_orbit_slot_x` and `debug_orbit_slot_y` instance variables (initialized to -1). Written in `_apply_herding_pressure` after `(tx, ty)` is computed; cleared to -1 at the top of `_compute_forces` each frame so slot is only visible when the fish is actively orbiting.
  - [config/config.py](config/config.py): Added `DEBUG VISUALIZATION` section with 7 new fields: `debug_baitballs_enabled` (master), `debug_baitball_shading_enabled`, `debug_baitball_labels_enabled`, `debug_sailfish_baitball_links_enabled`, `debug_sailfish_orbit_slots_enabled`, `debug_sailfish_state_labels_enabled`, `debug_sailfish_strike_paths_enabled`. All default False (or True but suppressed by master toggle). Fully documented.
  - [boids_overrides.json](boids_overrides.json): Added all 7 new debug fields (all `false`).
  - [config/config_menu.py](config/config_menu.py): Added `DEBUG VISUALIZATION` section at bottom of `_ROWS` with group header and toggle rows for all 7 fields.
  - [rendering/renderer.py](rendering/renderer.py): Added `from rendering.debug_overlay import BaitballDebugOverlay`; instantiates `self._debug_overlay` in `__init__`; calls `self._debug_overlay.draw(simulation, cfg)` in `draw()` after `batch.draw()` and mouse indicator, before HUD.
  Usage: TAB → scroll to DEBUG VISUALIZATION → toggle "Debug overlays" (master switch), then enable individual sub-toggles as needed.  
  Performance: all overlays use data already computed by the simulation. When `debug_baitballs_enabled=False` the draw path adds one branch check per frame. Text labels (BB labels, state labels) have the highest per-item cost; keep disabled in large populations.  
  Verification: 273/273 tests pass.

- 2026-05-07: Comprehensive documentation pass on `config/config.py`.  
  Why: every config field previously had at best a terse one-line comment; developer onboarding and tuning required reading the source code across multiple files to understand intent.  
  What changed:
  - [config/config.py](config/config.py): Full rewrite of all inline comments. Every field now documents: purpose, units, behavioral effect, interactions with other fields, and tuning guidance. Module-level docstring added explaining the override mechanism and JSON sync workflow. Fields organized under clearly labeled section banners (WINDOW / RENDERING, POPULATION, SARDINE MOVEMENT, SARDINE FLOCKING, SARDINE PERCEPTION RADII, SPATIAL HASH / PERFORMANCE, WORLD BOUNDARY, SARDINE VISUAL APPEARANCE, SARDINE COLOR SYSTEM, SAILFISH MOVEMENT, SAILFISH STATE MACHINE — TARGET LOCK / BURST / RECOVERY, BAITBALL DETECTION, PROTO-COLLABORATION, BAITBALL STRIKE, BAITBALL ORBIT FORCE MODEL, BAITBALL HERDING, ANTI-CONGA-LINE ORBIT VARIATION, SAILFISH LATERALIZATION, PROTO-COLLABORATION CAPS, BAITBALL ANCHOR / DRIFT DAMPING, ATTACK PRESSURE SUPPRESSION, INTER-SAILFISH SPACING, SAILFISH VULNERABLE-TARGET SCORING, SAILFISH SCHOOL-PRESSURE ORBIT, SAILFISH FLOCKING, SARDINE FLEE, ALGAE MOVEMENT / FLOCKING / RADII / ECOLOGY, SARDINE ECOLOGY, SAILFISH ECOLOGY, MOUSE PREDATOR, HUD). No field names, values, or logic changed.  
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
- ORBIT_BAITBALL: each sailfish accumulates its own orbit angle at its own speed and direction (per-fish `_orbit_speed_mult`, `_orbit_dir`, `_orbit_radius_mult` set at spawn), so fish naturally spread out around the perimeter instead of rotating together in a conga line. Stable per-fish lateralization (`attack_side` ±1, `lateralization_bias`) biases edge-target selection toward a preferred flank. Inward pressure tapers to zero inside the falloff radius and is capped per-frame. Drift damping counteracts detected baitball movement. Inter-sailfish spacing force breaks conga-line following. Deliberate strikes are patience/cooldown gated AND limited by `sailfish_max_simultaneous_baitball_strikes` and per-fish `_individual_strike_cooldown`.
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
