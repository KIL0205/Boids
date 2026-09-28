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

- 2026-05-08: Full project cleanup, consistency pass, and documentation maintenance.
  Why: follow-up after sailfish hunting refactor, ecology tightening, sleep-prevention addition, and starvation renderer fix to eliminate drift between config, UI, comments, and runtime behavior.
  What changed:

  1. **Two dead config fields tombstoned** — [config/config.py](config/config.py):
     - `sailfish_baitball_edge_target_chance`: marked `# LEGACY / NOT READ`. Was the per-second probability for edge-target strikes; superseded by the `query_hunting_target()` scoring system (2026-05-08 refactor). Edge-target opportunities are now scored and compared against baitball pressure rather than rolled as independent per-frame probabilities.
     - `sailfish_herding_slot_jitter`: marked `# LEGACY / NOT READ`. Was the angular slot jitter for the old herding system; superseded by `sailfish_baitball_slot_jitter` (BAITBALL ORBIT FORCE MODEL section). `_apply_herding_pressure()` never reads it.
     Both fields are retained in the dataclass for backward JSON compatibility. Both were already present as silently-ignored values in `boids_overrides.json`.

  2. **Cross-reference comment corrected** — [config/config.py](config/config.py): the `sailfish_baitball_strike_chance` LEGACY tombstone comment previously cited `sailfish_baitball_edge_target_chance` as the "active replacement"; updated to accurately describe the current replacement (scoring system + `sailfish_baitball_center_strike_chance`).

  3. **Stale internal comment removed** — [sim/simulation.py](sim/simulation.py): `sardine.age += dt` comment said "Stage 8.6 for max age death" — a planned-but-never-implemented feature. Replaced with an accurate description of what the age field is actually used for.

  4. **Dead menu row annotated** — [config/config_menu.py](config/config_menu.py): added a note above the `sailfish_baitball_edge_target_chance` menu row explaining it has no behavioral effect.

  5. **PROJECT_LOG.md Section 3 (Current Behavior / Rendering)** updated to mention the renderer reconciliation safety-net added in the Phase 5 starvation fix.

  6. **PROJECT_LOG.md Section 5 (Config and UI)** corrected: section count updated from 8 to 10; removed the nonexistent "SAILFISH — BAITBALL" section; added DEBUG VISUALIZATION, PERFORMANCE, and POWER MANAGEMENT sections; updated SAILFISH — HUNTING description to reflect that baitball/orbit/strike/scoring rows are now inside that section.

  No behavioral changes. Verification:
  - Full suite: `./.conda/bin/python -m tests.tests` → **268/268 pass**.
  - Bench: `./.conda/bin/python -m tests.bench --scenario worst_case --frames 120 --warmup 40 --verbose` → **39.59 ms avg**.

- 2026-05-08: Frozen starved-fish sprites fix (renderer lifecycle reconciliation).  
  Why: occasionally, fish that had already starved in simulation remained visible as frozen triangles until a manual population change triggered `renderer.reset()`.  
  Root cause:

  1. Simulation death/birth side-channels (`_sardines_starved_ids`, `_sardines_eaten_ids`, `_sailfish_died_ids`, `_algae_eaten_ids`, `_..._born`) are cleared at the **start** of each `update()` call in [sim/simulation.py](sim/simulation.py).
  2. When pyglet executes multiple `update()` ticks before one `on_draw` (frame hitch, event-loop jitter, resize/fullscreen transitions, etc.), renderer can miss a prior tick's death ids because they are cleared by the next tick before `Renderer.draw()` consumes them.
  3. Result: stale shapes survive in renderer maps/lists even though boids were removed from simulation lists.
  4. Manual add/remove appears to "fix" it because [main.py](main.py) explicitly calls `renderer.reset(...)` after those UI actions, fully rebuilding shape maps from live simulation lists and dropping stale orphan shapes.

  What changed:

  1. **Central stale-cache cleanup helper** — [rendering/renderer.py](rendering/renderer.py): added `_prune_removed_shapes(shape_map, live_ids, extra_maps=...)`.
    - Deletes stale shapes immediately (`shape.delete()`).
    - Removes stale ids from associated side maps (e.g. sardine velocity-state map).
    - Used as centralized cleanup path for starvation/eating/manual mismatch recovery.

  2. **Renderer reconciliation safety-net** — [rendering/renderer.py](rendering/renderer.py), inside `Renderer.draw(...)`:
    - For sardines, sailfish, and algae: if `len(shape_map) != len(live_list)`, reconcile against authoritative live ids from simulation.
    - Prune stale cached shapes and lazily allocate missing shapes.
    - Rebuild ordered draw lists from live simulation order after reconciliation.
    - This runs only on mismatch (not every frame), so normal-path performance remains unchanged.

  3. **Regression tests** — [tests/tests.py](tests/tests.py): added `TestRendererLifecycleCleanup` (3 tests) validating stale-shape pruning behavior and side-map cleanup.

  Verification:
  - Targeted tests: `python3 -m unittest tests.tests.TestRendererLifecycleCleanup` → **3/3 pass**.
  - Full suite: `python3 -m unittest tests.tests` → **310/310 pass** (4 skipped).
  - Perf sanity: `python -m tests.bench --scenario worst_case --frames 120 --warmup 40 --verbose` → **39.30 ms avg**, in family with prior worst-case numbers; no measurable regression attributable to the mismatch-only reconciliation guard.

  Remaining lifecycle risk:
  - The underlying side-channel-clear-at-update-start model still exists; renderer reconciliation now hardens correctness when draw cadence lags. A future deeper cleanup could move to explicit "consumed by renderer" queues, but current fix is minimal, architecture-preserving, and robust in observed scenarios.

- 2026-05-08: Sleep prevention — prevent host from sleeping while simulation is open.  
  Why: long observation sessions cause the display to dim/sleep mid-run; wanted a one-time, clean-lifecycle solution with no frame-timing cost.  
  What changed:

  1. **New module** — [utils/sleep_prevention.py](utils/sleep_prevention.py):
     - `enable_sleep_prevention() → bool` — starts prevention; returns `True` on success.
     - `disable_sleep_prevention()` — releases the assertion/process cleanly.
     - `is_active() → bool` — query current state.
     - **macOS**: spawns a single persistent `caffeinate -d -i -m` subprocess (display sleep, idle sleep, disk sleep inhibited). Subprocess is started once at app launch and `terminate()`d on close. Falls back gracefully if `caffeinate` is absent.
     - **Windows**: calls `SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED)` at enable; `SetThreadExecutionState(ES_CONTINUOUS)` at disable.
     - **Linux**: not supported; logs a warning and returns `False` from `enable`.
     - All calls are idempotent; `disable` without a prior `enable` is a no-op.
     - Module-level state (`_caffeinate_proc`, `_win_active`) means zero per-frame cost.

  2. **Config field** — [config/config.py](config/config.py): `prevent_system_sleep: bool = True` added in a new POWER MANAGEMENT section after `sardine_use_gpu`. Disabling gives back OS idle policy (useful for unattended/headless runs).

  3. **Config menu** — [config/config_menu.py](config/config_menu.py): new `_GroupRow("POWER MANAGEMENT")` section with `_ToggleItem("Prevent sleep", "prevent_system_sleep")` appended before `_SaveRow()`.

  4. **Runtime override** — [boids_overrides.json](boids_overrides.json): `"prevent_system_sleep": true` added as master runtime default.

  5. **main.py lifecycle integration** — [main.py](main.py):
     - `from utils import sleep_prevention` import added.
     - After config load: `if config.prevent_system_sleep: sleep_prevention.enable_sleep_prevention()` (single call at startup; logs success/failure via `logging`).
     - `on_close` event handler added to call `sleep_prevention.disable_sleep_prevention()` then `window.close()`.
     - `pyglet.app.run()` wrapped in `try/finally` — `disable_sleep_prevention()` also called in the `finally` block as a belt-and-suspenders release in case of exception or signal exit (second call is a no-op if already disabled via `on_close`).

  Platform support:
  - macOS: caffeinate, standard since OS X 10.8
  - Windows: SetThreadExecutionState via ctypes
  - Linux: not supported; fails silently with warning log

  Limitations:
  - Toggle in the config menu changes the *stored* config value for future launches but does not enable/disable prevention mid-run (would require calling enable/disable from `apply_config`). Current behavior: state is set once at startup. For now this is the intended design.
  - Linux could be supported via `systemd-inhibit` or `org.freedesktop.PowerManagement` D-Bus, but is deferred.

  Verification:
  - Unit tests: **307/307 pass** (`python3 -m unittest tests.tests`, 4 skipped).  
    New test class `TestSleepPrevention` (18 tests) covers: macOS enable/disable/idempotent/graceful-failure/is_active, Windows enable/disable/idempotent, Linux no-op, config field existence and override.
  - No runaway subprocess: the `finally` block and the `on_close` handler both terminate caffeinate; the idempotent guard prevents double-terminate.
  - Frame timing unaffected: `enable`/`disable` are called once at startup/shutdown only; no per-frame work added.

- 2026-05-08: Sailfish smart target-decision layer (FSM-preserving refactor).  
  Why: hunting needed a dedicated target-query/scoring layer without overloading ecology eating; orbit baitball eating remains intentionally allowed as active cooperative hunting.  
  What changed:

  1. **`query_first` fixed to true nearest** — [sim/spatial_hash.py](sim/spatial_hash.py): `query_first()` now scans local candidate cells and returns the closest sardine within radius (not first cell-order hit). Kept simple and grid-local for ecology eating.

  2. **New hunting decision query function** — [sim/sailfish.py](sim/sailfish.py): added `query_hunting_target(sardines, baitball) -> _HuntingDecision` with explicit outcomes:
    - `isolated_target`
    - `baitball_orbit`
    - `baitball_strike`
    - `track_com`
    - `no_target`

    The scorer is local/grid-based and uses: sailfish distance, school-COM isolation, baitball-edge position, sardine speed vulnerability, hunger aggression, and shared-target penalty (nearby sailfish already locking same sardine). Baitball scoring uses edge distance, density, radius, cooperative availability, group cooldown pressure, and hunger.

  3. **FSM integration at decision points** — [sim/sailfish.py](sim/sailfish.py): `TRACK_SCHOOL`, `ORBIT_BAITBALL`, and `TARGET_LOCK` now consult `query_hunting_target()` for transition/target choice. Existing FSM states and timers/caps are preserved (group cooldown, orbit patience, simultaneous attacker caps, strike cooldowns).

  4. **Orbit eating enabled by gate** — [sim/simulation.py](sim/simulation.py): sailfish eating gate now allows `orbit_baitball` again (`track_school`, `orbit_baitball`, `target_lock`, `burst_attack`, `baitball_strike` allowed). `query_first` remains the one-kill nearest eater. PATROL/RECOVER still blocked.

  5. **Decision diagnostics counters** — [sim/simulation.py](sim/simulation.py): added cumulative counters:
    - `stats_target_decision_isolated`
    - `stats_target_decision_baitball_orbit`
    - `stats_target_decision_baitball_strike`
    - `stats_target_decision_track_com`
    - `stats_target_decision_none`
    These aggregate per-frame sailfish decision categories for tuning and benchmarks.

  6. **Config + menu tuning knobs (conservative)** — [config/config.py](config/config.py), [config/config_menu.py](config/config_menu.py), [boids_overrides.json](boids_overrides.json):
    - `sailfish_target_edge_weight`
    - `sailfish_target_hunger_weight`
    - `sailfish_target_shared_penalty_weight`
    - `sailfish_target_decision_margin`
    - `sailfish_baitball_score_edge_weight`
    - `sailfish_baitball_score_density_weight`
    - `sailfish_baitball_score_radius_weight`
    - `sailfish_baitball_score_coop_weight`

  7. **Tests + benchmark updates**:
    - [tests/tests.py](tests/tests.py):
     - added nearest-neighbor regression test for `query_first`
     - deduplicated accidentally duplicated per-state gate class
     - updated gate expectations for ORBIT_BAITBALL-eating-allowed
     - added `TestSailfishTargetDecision` for shared-target penalty and baitball-orbit preference
    - [tests/bench.py](tests/bench.py): added target-decision scenarios and verbose decision-count output (`target_dense_one`, `target_two_baitballs`, `target_scattered`, `target_mixed`), plus CSV columns for decision counts.

  Verification:
  - Unit tests: **292/292 pass** (`python -m unittest tests.tests`, 4 skipped).
  - Bench sample (120 frames + 40 warmup, headless):
   - `target_dense_one`: 10.35 ms avg, decisions orbit=1200
   - `target_two_baitballs`: 15.61 ms avg, decisions orbit=1440
   - `target_scattered`: 14.33 ms avg, decisions isolated=709, orbit=271, none=220
   - `target_mixed`: 13.02 ms avg, decisions orbit=1440
   - `worst_case`: 37.58 ms avg (close to prior logged 38.01 ms baseline; no material regression observed)

  Follow-up tuning note: dense scenarios currently bias heavily to orbit (few/no strikes in short runs). If desired, tune `sailfish_target_decision_margin`, `sailfish_baitball_score_density_weight`, and strike chances to increase edge-lock/strike frequency without collapsing into all-fish simultaneous attacks.

- 2026-05-08: Medium-fidelity ecology tightening (5 code changes, 15 new tests).  
  Why: follow-up to the sailfish hunting analysis that identified config defects and a state-independent eating hole.  
  What changed:

  1. **Duplicate config field removed** — [config/config.py](config/config.py): the second declaration of `sailfish_target_lock_suppressed_near_baitball` (in the BAITBALL DETECTION section, ~line 513) was deleted. The canonical declaration in the INDIVIDUAL TARGET LOCK section (~line 365) is kept. No behavioral change; Python was silently keeping the last (identical) declaration.

  2. **Dead field tombstoned** — `sailfish_baitball_strike_chance` in [config/config.py](config/config.py) now carries a `# LEGACY / NOT READ` comment block explaining it was superseded by `sailfish_baitball_center_strike_chance`. Field is retained (value `0.08`) for JSON backward compatibility only; a test at [tests/tests.py](tests/tests.py) line ≈4953 still passes it as a config override. No code reads it.

  3. **`_select_active_baitball` now ranks by edge distance** — [sim/sailfish.py](sim/sailfish.py): replaced center-distance-squared ranking (`best_d2`) with true edge distance `math.sqrt(dx*dx+dy*dy) - b.radius` tracked as `best_edge_dist`. A large distant baitball whose edge is farther than a smaller close baitball will no longer win the selection.

  4. **`_should_take_individual_target` dead branch clarified** — [sim/sailfish.py](sim/sailfish.py): the final `return self._orbit_timer >= self._orbit_patience_goal` was always `False` in TRACK_SCHOOL (orbit timer not incremented there). Replaced with explicit `return False` and a multi-line comment explaining the invariant and why this branch is only reached in edge cases where the orbit redirect was skipped.

  5. **Per-state eating gate** — [sim/simulation.py](sim/simulation.py): the ecology eating pass now only allows kills when the sailfish is in `{"track_school", "target_lock", "burst_attack", "baitball_strike"}`. PATROL, RECOVER, and ORBIT_BAITBALL are excluded. Gate is defined as `_EATING_STATES: frozenset[str]` local to the pass (no module-level constant added). New `stats_eat_blocked_by_state: int = 0` diagnostic counter on `Simulation` (with docstring) tracks blocked attempts for starvation monitoring.

  Test changes: [tests/tests.py](tests/tests.py) — `test_stats_sardines_eaten_cumulative` updated: added `sailfish_burst_trigger_radius=0.0` to keep sailfish in TARGET_LOCK across both frames (without this, frame 2 has sf in RECOVER after the burst, blocking the second eat). New class `TestSailfishPerStateEatingGate` (7 tests) covers: PATROL blocks eating + counter increments, RECOVER blocks eating, TRACK_SCHOOL allows eating, BURST_ATTACK allows eating, counter starts at zero, counter not incremented on successful eat.

  Test count: **288 / 288 pass** (was 281 before this session; 7 net new tests).  
  Performance: frozenset lookup is O(1) per sailfish per ecology tick — negligible at all observed population sizes.

- 2026-05-08: Sailfish hunting behavior deep analysis (analysis only — no code changed).  
  Why: requested review of all sailfish detection, targeting, attack, eating, ecology, and interaction systems before any further behavioral changes.  
  Key findings:
  1. **Duplicate config field** — `sailfish_target_lock_suppressed_near_baitball` is declared twice in [config/config.py](config/config.py) (lines 365 and 513). Python keeps the last definition; no behavioral effect currently but a maintenance hazard.
  2. **Dead legacy field** — `sailfish_baitball_strike_chance` is documented as superseded but still present in the dataclass. No code reads it.
  3. **Eating is state-independent** — `query_first(sf, eat_r=25px)` runs on every sailfish every ecology tick regardless of FSM state (PATROL, RECOVER, etc.). A PATROL sailfish can eat and reset its hunger timer without entering the hunting sequence. This weakens starvation pressure.
  4. **`query_first` is not nearest-first** — returns the first sardine found in hash cell order, not necessarily the closest within `eat_r`.
  5. **`_select_active_baitball` ignores baitball radius** — prefers nearest center, not nearest edge. A large distant baitball can be ranked above a small close one even when its edge is farther away.
  6. **Dead suppression branch in `_should_take_individual_target`** — final `return self._orbit_timer >= self._orbit_patience_goal` is never True in TRACK_SCHOOL (orbit timer not incremented there), making individual lock always suppressed when baitball+herding conditions are met. No behavioral bug because `_should_enter_orbit_baitball` redirects to ORBIT first, but the intent is misleading.
  7. **One-frame force double-application in RECOVER → ORBIT_BAITBALL** — the COM steer and `_apply_herding_pressure` both run on the transition frame. Benign but architecturally messy.
  8. **Mouse predator**: operates entirely independently of sailfish FSM (fleet force on sardine flee pass only). Mouse disrupting a baitball can cause orbit sailfish to fall back to TRACK_SCHOOL — an appropriate emergent interaction.
  9. **`boids_overrides.json` merge threshold drift** — config.py defaults for `sailfish_baitball_merge_distance_multiplier` (0.8) and `sailfish_baitball_merge_overlap_threshold` (0.2) appear to have been tuned down from the original 1.2/0.55 values noted in the 2026-05-07 log entry. Verify both files are synchronized.
  10. **Performance**: sailfish FSM cost is not a bottleneck (0.87–0.93 ms/frame at 12–15 fish). All proposed fixes are safe for real-time.

  Recommended next phase: ecology tightening (fix duplicate field, annotate dead field, add per-state eating gate) then validate with a steady-state population benchmark before attempting behavioral changes like hunger-modulated aggression.  
  No code changed. Full analysis in conversation log.

- 2026-05-08: WebGPU sardine compute backend (wgpu-py / WGSL / Metal).  
  Why: CPU sardine force pass still dominated all high-population scenarios (14–25 ms/frame). WebGPU compute shaders on the Apple M5 Metal backend proved 5–9× faster than the Python numpy cell-bucket loop for sardine flocking forces, reducing worst_case from 40 ms to 16.5 ms.  
  What changed:
  - [sim/sardine_gpu.py](sim/sardine_gpu.py): New file. `SardineGPUBackend` builds a WGSL compute shader that computes flee + separation + alignment + cohesion for all sardines in parallel. Each invocation handles one sardine; passes 1 / 1b / 2 of `_batch_compute_sardine_forces` are replaced. CPU sorts sardines into a contiguous spatial hash (argsort + bincount + cumsum), uploads float32 buffers, dispatches the shader, reads back float32 results, scatters to original-index float64 output. Pass 3 (noise) stays on CPU. Graceful fallback: `try_create_backend()` returns `None` on wgpu import error or device init failure. Requires `pip install wgpu` (wgpu-py 0.31+).
  - [sim/simulation.py](sim/simulation.py): `__init__` adds `_gpu_backend` and `_gpu_init_tried` fields (lazy init). `_batch_compute_sardine_forces` checks `cfg.sardine_use_gpu` before the CPU cell-bucket loop; if GPU fails mid-frame it disables itself, resets accumulator, and falls through to CPU. `apply_config` allows re-init when GPU is toggled on via config menu.
  - [config/config.py](config/config.py): New field `sardine_use_gpu: bool = False`.
  - [config/config_menu.py](config/config_menu.py): New `_ToggleItem("GPU sardine forces", "sardine_use_gpu")` in PERFORMANCE section.
  - [boids_overrides.json](boids_overrides.json): Added `sardine_use_gpu: false` (default off).
  - [tests/bench.py](tests/bench.py): Two GPU scenarios (`gpu_baseline`, `gpu_worst_case`) added to `GPU_SCENARIOS`. `--compare-gpu` flag runs CPU vs GPU head-to-head comparison with delta table and speedup ratio. `run_gpu_comparison()` helper function.
  - [tests/tests.py](tests/tests.py): New `TestSardineGPUBackend` class (8 tests): backend availability, import-error fallback, CPU fallback when GPU fails, positions within bounds, no NaN/Inf, velocity clamping, sardines cluster over time. All 8 pass (invariant tests skip gracefully when wgpu unavailable).

  **CPU vs WebGPU comparison (200 frames, 60 warmup, Apple M5, Metal backend):**

  | Scenario | CPU avg | GPU avg | Delta | GPU sardine ms |
  |---|---|---|---|---|
  | baseline (1500 sardines) | 16.39 ms | 3.24 ms | **−13.15 ms (5.1×)** | 1.61 ms |
  | worst_case (2500s + ecology) | 38.01 ms | 16.55 ms | **−21.46 ms (2.3×)** | 1.73 ms |

  worst_case GPU breakdown: sardine 1.73 ms, sailfish 1.48 ms, baitball 7.21 ms, algae 0.64 ms.
  Baitball detection and sailfish FSM remain on CPU (no change).

  **How to enable GPU:** `Tab → PERFORMANCE → GPU sardine forces → ON`, or set `sardine_use_gpu: true` in `boids_overrides.json`.

  **Platform notes:**
  - macOS (Apple Silicon): Metal backend, tested on M5. No installation issues.
  - Linux: Vulkan or Metal via wgpu-native (untested here; should work).
  - Windows: DX12 or Vulkan via wgpu-native (untested here; should work).
  - wgpu-py uses wgpu-native (a Rust WebGPU implementation), not a browser; no browser required.
  - Fallback to CPU is automatic on unsupported hardware or missing wgpu.

  **Remaining bottlenecks with GPU enabled:** baitball detection (7.2 ms in worst_case) is the next largest cost. Moving it to GPU would require a significant WGSL rewrite of the connected-component merge logic.

  Verification: 281/281 tests pass.

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

- 2026-05-08: Expanded benchmark suite to cover all major simulation subsystems.  
  Why: the old bench.py only measured four population-scaling scenarios (light/medium/heavy/extreme); it had no coverage of sailfish FSM cost, baitball detection pipeline, cooperative herding overhead, or debug overlay impact. It also reported only avg ms — no p95/p99/min/max.  
  What changed:
  - [tests/bench.py](tests/bench.py): Complete rewrite. Key additions:
    - **`BenchResult` dataclass** with raw per-frame timing list; derives `avg_ms`, `p95_ms`, `p99_ms`, `min_ms`, `max_ms`, `fps`, `elapsed_s` via numpy.
    - **`Scenario` dataclass** with `config_kwargs` (passed directly to `SimConfig`), `collect_subsystem` flag, and `note` for headless limitations.
    - **8 named benchmark scenarios** (`BENCHMARK_SCENARIOS`), each isolated to measure a different bottleneck:

      | # | Name | Purpose |
      |---|---|---|
      | 1 | `baseline` | Pure sardine flocking — no predators, no ecology |
      | 2 | `standard_ecology` | Full three-tier ecosystem with reproduction + starvation |
      | 3 | `sailfish_stress` | 15 sailfish with full FSM (baitball, orbit, herding, strike) |
      | 4 | `baitball_detect` | 2000 sardines in 900×600 world, detection every frame (no stagger) |
      | 5 | `coop_herding` | 12 sailfish with cooperative orbit, lateralization, spacing, strike scheduling |
      | 6 | `debug_off` | Full debug config but master toggle OFF (verifies zero idle overhead) |
      | 7 | `debug_on` | Full debug config ON (confirms same simulation cost as debug_off) |
      | 8 | `worst_case` | Max load: 2500 sardines, 1200 algae, 10 sailfish, all features |

    - **Per-frame subsystem timing** via `debug_frame_timing_enabled=True`; avg per-frame ms collected for sardine, sailfish, baitball, algae subsystems when `collect_subsystem=True`.
    - **Regression guard preserved** (`--test` still runs light/medium/heavy/extreme with existing thresholds).
    - **New CLI flags**: `--scenario NAME`, `--warmup N`, `--verbose` (subsystem breakdown), `--csv PATH`, `--profile-scenario NAME`.
    - **Headless note**: scenarios 6/7 confirm that `debug_baitballs_enabled` has zero simulation-side overhead; actual pyglet shape/label rendering cost is only measurable with a display.

  Full benchmark results (300 timed frames, 60 warmup, DT=1/60, world 1920×1080):

  | Scenario | avg ms | p95 ms | p99 ms | fps-equiv | sardine | sailfish | baitball | algae |
  |---|---|---|---|---|---|---|---|---|
  | baseline | 16.59 | 18.86 | 19.60 | 60.3 | 15.31 | — | — | — |
  | standard_ecology | 17.00 | 19.82 | 21.69 | 58.8 | 11.38 | 0.46 | 1.26 | 1.62 |
  | sailfish_stress | 10.70 | 14.67 | 16.33 | 93.5 | 8.27 | 0.93 | 0.78 | — |
  | baitball_detect | 29.04 | 30.01 | 31.12 | 34.4 | 22.26 | — | 5.05 | — |
  | coop_herding | 15.34 | 19.63 | 21.48 | 65.2 | 10.23 | 0.87 | 3.33 | — |
  | debug_off | 19.68 | 23.69 | 25.07 | 50.8 | 13.73 | 0.55 | 1.52 | 1.23 |
  | debug_on | 19.61 | 24.35 | 24.95 | 51.0 | 13.67 | 0.52 | 1.52 | 1.27 |
  | worst_case | 43.66 | 46.54 | 52.61 | 22.9 | 27.63 | 1.83 | 7.79 | 0.70 |

  Key findings from results:
  - **debug_off vs debug_on**: 19.68ms vs 19.61ms — within measurement noise, confirming `debug_baitballs_enabled` adds zero simulation overhead.
  - **baitball_detect**: 5.05ms for baitball detection every frame in a dense world (2000 sardines, 900×600) — highest detection cost of any scenario. With `baitball_detect_interval=3` (stagger default), this drops to ~1.7ms effective cost.
  - **sailfish_stress**: 15 sailfish cost only 0.93ms/frame — individual sailfish AI is cheap; the sardine batch force pass dominates.
  - **worst_case** baitball 7.79ms (detection every frame at 2500 sardines) confirms stagger is critical for high-population worlds.
  - Subsystem timing collection uses `debug_frame_timing_enabled=True`; this adds ~0.1ms overhead but is disabled in non-collect scenarios.

  Usage:
  ```
  python -m tests.bench                          # all 8 scenarios
  python -m tests.bench --scenario baitball_detect --verbose
  python -m tests.bench --profile --profile-scenario worst_case
  python -m tests.bench --test                   # CI regression guard
  python -m tests.bench --csv results.csv        # longitudinal comparison
  ```
  Verification: 273/273 tests pass.

- 2026-05-07: Performance optimization pass — profiling, numpy vectorization, baitball detection stagger.  
  Why: at 1500 sardines (heavy) the simulation was consuming 34.9ms/frame. cProfile identified five hot functions accounting for ~78% of simulation CPU.  
  Profiling findings (heavy scenario, 200 frames):

  | Function | tottime | share | Root cause |
  |---|---|---|---|
  | `_batch_compute_sardine_forces` | 3.42s | 35% | Per-cell list alloc + `np.array()` per cell |
  | `_detect_baitballs` | 2.19s | 22.5% | Per-sardine `_wrap_delta()` Python calls × 9.95M/200 frames |
  | `spatial_hash.query` | 0.95s | 9.8% | Algae + baitball grid traversal |
  | `_wrap_delta` | 0.52s | 5.4% | 9.95M bare Python function calls |
  | `list.append` | 0.49s | 5.1% | 12.1M calls from per-cell neighbor accumulation |
  | `_select_edge_target` (sailfish) | 0.13s | 1.3% | Per-sardine `math.sqrt` loop × ~300 sardines/call |

  What changed:
  - [sim/simulation.py](sim/simulation.py):
    - **`_batch_compute_sardine_forces`**: Replaced per-cell `neigh_list: list = []` + `np.array(neigh_list)` with a pre-allocated reusable `_sf_neigh_buf` (numpy int32, 2× growth). Inner loop now writes `neigh_buf[nb_n] = nb._idx`. `neigh_idx = neigh_buf[:nb_n]` is a view (no allocation). Eliminates 12.1M `list.append` + 20K `np.array()` calls per 200 frames.
    - **`_detect_baitballs` — full vectorized rewrite**: Neighbor gather loop uses `_bb_neigh_buf` (pre-allocated int32 buffer, grown lazily). Distance pass is fully numpy: `idxs = neigh_buf[:nb_n]`, `P = _s_pos[idxs]`, vectorized toroidal correction via `np.where()`, `d2 = dx*dx + dy*dy`, mask and center computed with numpy reductions. Eliminates 9.95M `_wrap_delta()` Python calls and all per-cell intermediate list allocations.
    - **`_detect_baitballs` — batch array fallback for direct test calls**: When `_s_n == 0` but sardines exist (test bypasses `update()`), populates `_s_pos` and assigns `_idx` on demand so direct calls work correctly.
    - **`_merge_baitball_candidates`**: Replaced `dataclasses.replace(ex, ...)` with direct `_BaitballInfo(bid=ex.bid, ...)` constructor calls. Eliminates ~17K `dataclasses.replace` calls per 200 frames.
    - **Baitball detection stagger**: Added `_baitball_detect_counter` / `_baitball_detect_interval` to `__init__`. `update()` only calls `_detect_baitballs()` once every `baitball_detect_interval` frames; cached result reused between frames. Counter initialized to `interval−1` so detection fires on frame 1 (not delayed).
    - **Per-subsystem timing**: Added `dbg_ms_sardine_forces`, `dbg_ms_baitball_detect`, `dbg_ms_sailfish_update`, `dbg_ms_algae_update` float fields. When `cfg.debug_frame_timing_enabled`, `update()` wraps each subsystem with `time.perf_counter()` and stores elapsed ms.
    - **`apply_config`**: Refreshes `_baitball_detect_interval` from `new_config.baitball_detect_interval` on live config changes.
  - [sim/sailfish.py](sim/sailfish.py):
    - **`_select_edge_target` — vectorized rewrite**: Replaced per-sardine Python loop (`math.sqrt` per fish) with numpy arrays. Gathers `gxs, gys, gvxs, gvys`, computes toroidal `dx/dy` with `np.where()`, vectorized `dists`, `edge_mask`, `speeds`, `scores`, lateralization bias. `scores[~edge_mask] = -1e18`; `scores.argmax()` selects best. Eliminates O(N_sardines) `math.sqrt` calls per sailfish per frame.
    - Added `import numpy as np`.
  - [config/config.py](config/config.py): Added two new fields:
    - `debug_frame_timing_enabled: bool = False` — enables per-subsystem ms timing row in HUD.
    - `baitball_detect_interval: int = 3` — controls detection stagger; 1 = every frame, 3 = every 3rd frame (~50ms lag at 60fps).
  - [config/config_menu.py](config/config_menu.py): Added new PERFORMANCE section with `_ToggleItem("Frame timing HUD")` and `_MenuItem("BB detect intv")` rows.
  - [rendering/renderer.py](rendering/renderer.py): HUD shows optional timing row `[Timing] Sardine X.XXms  BB X.XXms  SF X.XXms  Algae X.XXms` when `debug_frame_timing_enabled` is true.
  - [boids_overrides.json](boids_overrides.json): Added `debug_frame_timing_enabled: false` and `baitball_detect_interval: 3`.

  Benchmark results (300 timed frames + 60 warmup, DT=0.0167):

  | Scenario | Before | After | Δ |
  |---|---|---|---|
  | light (300 sardines) | 10.95ms | 9.21ms | **−16%** |
  | medium (800 sardines) | 19.34ms | 16.35ms | **−15%** |
  | heavy (1500 sardines) | 34.91ms | 28.99ms | **−17%** |
  | extreme (2000 sardines) | 48.31ms | 46.93ms | **−3%** |

  Remaining opportunities: `spatial_hash.query` (9.8%) could be accelerated with Cython or pre-sorted cell lists; algae update is not yet batched.  
  Verification: 273/273 tests pass.

- 2026-05-08: Performance optimization pass #2 — allocation pressure reduction in sardine/baitball hot paths.  
  Why: the expanded benchmark suite showed persistent cost concentration in `_batch_compute_sardine_forces` and `_detect_baitballs`, with worst-case still at ~44ms/frame. cProfile confirmed repeated numpy temp-array creation and per-cell index allocations as dominant overhead in dense scenarios.  
  What changed:
  - [sim/simulation.py](sim/simulation.py):
    - Added persistent reusable int32 buffers on `Simulation` init: `_sf_neigh_buf`, `_sf_owner_buf`, `_bb_neigh_buf`.
    - `_batch_compute_sardine_forces`: replaced per-cell `np.empty()` owner-index allocation with reusable `_sf_owner_buf` slices; reused `_sf_neigh_buf` without `hasattr` checks; cached `cells.get` in local variable for inner loops.
    - `_batch_compute_sardine_forces`: switched toroidal corrections from nested `np.where(...)` chains to in-place masked updates (`arr[mask] +=/-= world_size`) for flee, mouse-flee, and cell-pair delta passes, reducing temporary ndarray churn.
    - `_detect_baitballs`: reused `_bb_neigh_buf` without `hasattr`; switched both toroidal correction passes to in-place masked updates to cut temporary array creation.
  - [sim/spatial_hash.py](sim/spatial_hash.py):
    - `insert`: cached `self._cells` local in hot path.
    - `query`, `query_position`, `query_first`: cached `self._cells.get` and `results.append` as locals to reduce repeated attribute lookups in tight loops.

  Full-suite benchmark comparison (300 timed frames, +60 warmup, same machine/session):

  | Scenario | Before | After | Delta |
  |---|---|---|---|
  | baseline | 16.88 ms | 16.26 ms | **-3.7%** |
  | standard_ecology | 17.21 ms | 16.85 ms | **-2.1%** |
  | sailfish_stress | 10.60 ms | 10.76 ms | +1.5% (noise) |
  | baitball_detect | 30.21 ms | 26.86 ms | **-11.1%** |
  | coop_herding | 15.14 ms | 14.69 ms | **-3.0%** |
  | debug_off | 20.81 ms | 19.21 ms | **-7.7%** |
  | debug_on | 19.83 ms | 18.90 ms | **-4.7%** |
  | worst_case | 44.42 ms | 40.28 ms | **-9.3%** |

  Subsystem deltas in key scenarios:
  - baseline sardine: 15.60 -> 14.97 ms
  - baitball_detect sardine: 22.99 -> 20.28 ms; baitball: 5.38 -> 4.83 ms
  - worst_case sardine: 27.71 -> 24.90 ms; baitball: 8.16 -> 7.19 ms

  Validation: 273/273 unit tests pass after the optimization pass (`python -m unittest tests.tests`).

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
- Tests: `python -m unittest tests.tests` (310 tests)
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
- Renderer reconciliation safety-net detects and prunes stale cached shapes on count mismatch (no normal-path cost).

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
- Data rows across 10 sections + Save row.
- Sections: SARDINES, MOUSE PREDATOR, SAILFISH — FLOCKING, SAILFISH — HUNTING, SAILFISH — BURST, ALGAE, ECOLOGY, DEBUG VISUALIZATION, PERFORMANCE, POWER MANAGEMENT.
- SAILFISH — HUNTING: all pursuit, targeting, baitball detection/orbit/strike scoring, lateralization, orbit variation, proto-collaboration, anchor, attack suppression, and inter-sailfish spacing fields.
- SAILFISH — BURST: burst trigger/duration/cooldown, speed/force multipliers, recovery duration.
- DEBUG VISUALIZATION: master toggle + per-layer sub-toggles for all debug overlays.
- PERFORMANCE: frame timing HUD toggle, baitball detect interval, GPU sardine forces toggle.
- POWER MANAGEMENT: prevent system sleep toggle.
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
