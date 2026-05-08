"""
config/config.py
================
Central configuration for the sardine/sailfish boid simulation.

All tunable parameters live here as a frozen dataclass so they can be
imported by any module.  Change values here; do NOT scatter magic numbers
throughout the codebase.

This file is the authoritative developer reference for all simulation
tuning.  Every field documents its purpose, units, behavioral effect,
interactions with other fields, and recommended tuning guidance.

Override mechanism
------------------
At startup, ``config_io.py`` reads ``boids_overrides.json`` and replaces
any fields listed there via ``dataclasses.replace()``.  The live config
menu (TAB) further replaces fields at runtime.  Neither mechanism changes
this file — all changes are ephemeral or written back to the JSON file.
Keep this file's defaults as the stable "clean slate" baseline.

JSON override alignment
-----------------------
When both files carry the same field, they must agree.  After any tuning
session, use the in-app Save (S key) to write the current state back to
``boids_overrides.json`` so the two files stay synchronized.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimConfig:
    """Immutable simulation configuration.

    Instantiate once in ``main.py`` and pass into ``Simulation`` /
    ``Renderer``.  All agents hold a reference to the same instance;
    ``apply_config()`` swaps it atomically when the menu edits a value.
    """

    # ═══════════════════════════════════════════════════════════════════
    # WINDOW / RENDERING
    # ═══════════════════════════════════════════════════════════════════

    # Logical window width in pixels.  The window may be resized at runtime
    # via ``Simulation.resize()`` + ``Renderer.on_resize()``; this value is
    # only used at startup and for the initial world-boundary calculation.
    window_width: int = 1400

    # Logical window height in pixels.  Same startup-only caveat as
    # ``window_width``.  Fish positions wrap toroidally inside this boundary.
    window_height: int = 700

    # Target frame rate in frames per second.  The main loop uses a pyglet
    # scheduled update at this rate.  Actual CPU cost scales with agent count,
    # not the target FPS.  Lowering this (e.g. 30) reduces CPU load at the
    # cost of choppier motion; raising it beyond 120 has diminishing returns.
    target_fps: int = 60

    # ═══════════════════════════════════════════════════════════════════
    # POPULATION
    # ═══════════════════════════════════════════════════════════════════

    # Starting number of sardines spawned at simulation launch.  This is not
    # the same as ``sardine_max_count``: the simulation always starts with
    # exactly this many sardines regardless of the cap.  The runtime
    # population then drifts based on starvation and reproduction.
    # CPU cost scales roughly linearly with sardine count because the batched
    # sardine force computation (Phase 4C) dominates the update budget.
    # Values above ~3 000 may cause noticeable frame-rate drops on older
    # hardware; the spatial-hash mitigates but does not eliminate O(n) cost.
    num_sardines: int = 2518

    # Starting number of sailfish spawned at simulation launch.  Sailfish
    # are individually updated (not batched), so their per-fish cost is higher
    # than sardines.  More than ~20 sailfish rarely adds visual interest and
    # can destabilize the ecosystem by wiping out sardines faster than they
    # can reproduce.
    num_sailfish: int = 11

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE MOVEMENT
    # ═══════════════════════════════════════════════════════════════════

    # Maximum swim speed for sardines (pixels per second).  At 1 px ≈ 1 cm,
    # 150 px/s ≈ 1.5 m/s, which is biologically plausible for burst-swimming
    # sardines.  Raising this makes sardines harder to catch but also causes
    # the school to spread out more when fleeing, which can break tight
    # baitball formations.  Lowering it makes sardines easier prey and tends
    # to produce denser, more compact schools.
    # Interaction: sailfish_max_speed should be ≥ 1.3× this value to ensure
    # predators can eventually close on healthy prey.
    sardine_max_speed: float = 150.0

    # Minimum swim speed for sardines (pixels per second).  Prevents sardines
    # from decelerating to a full stop, which would look unnatural and break
    # the flocking physics (zero-velocity fish cannot contribute to alignment).
    # Keep at roughly 30–40% of sardine_max_speed.
    sardine_min_speed: float = 50.0

    # Maximum steering force budget for sardines (pixels per second squared).
    # This is the total steer capacity applied per frame before normalization.
    # The ratio max_force / max_speed ≈ 1.0 produces natural turning arcs; a
    # lower ratio gives sluggish cornering, a higher ratio gives snap-turns.
    # At 60 fps each flocking force contributes roughly weight × max_force × dt
    # pixels/second of velocity change per frame.
    sardine_max_force: float = 150.0

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE FLOCKING (Reynolds rules)
    # ═══════════════════════════════════════════════════════════════════
    #
    # The three core Reynolds rules are applied in priority order:
    #   separation > alignment > cohesion
    # Each weight is a dimensionless multiplier on the steer force contributed
    # by that rule.  Per-frame velocity change ≈ weight × max_force × dt.
    # At 60 fps with max_force=150:  weight 2.0 → ~5 px/s per frame.

    # Separation weight: pushes sardines away from immediate neighbors to
    # prevent overlap.  This is the highest-priority rule — without it schools
    # collapse into a single point.  Increase for more personal space and
    # looser schools; decrease to allow denser packing (risks visual clumping).
    weight_separation: float = 2.0

    # Alignment weight: steers each sardine toward the average heading of
    # its neighbors.  The primary driver of coordinated school turns and
    # polarized waves.  Higher values produce tighter heading lock and faster
    # collective turns; lower values make the school look like a loose swarm.
    weight_alignment: float = 1.2

    # Cohesion weight: pulls each sardine toward the center of mass of its
    # neighbors.  Keeps the school from fragmenting.  Increasing this relative
    # to separation causes fish to pack tighter; decreasing it allows the school
    # to spread into loose, extended formations.
    weight_cohesion: float = 1.2

    # Noise weight: adds a small random force each frame to break perfect
    # symmetry and create organic micro-variation in direction.  Keep ≤ 0.05
    # so noise never dominates a real flocking force.  Set to 0.0 for a
    # deterministic, crystalline school (useful for tests).
    weight_noise: float = 0.02

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE PERCEPTION RADII
    # ═══════════════════════════════════════════════════════════════════
    #
    # Hard constraint: radius_separation < radius_alignment ≤ radius_cohesion.
    # For best spatial-hash performance all radii should be ≤ cell_size so a
    # single 3×3-ring lookup covers all neighbors.  Larger radii require
    # scanning more cells (proportionally more CPU).

    # Radius within which a sardine applies separation force away from each
    # neighbor (pixels).  Roughly 3 sardine body-lengths at the default size.
    # Must be the smallest of the three radii.  Increasing it gives each fish
    # more personal space and produces a looser school overall.
    radius_separation: float = 34.0

    # Radius within which a sardine matches heading with its neighbors
    # (pixels).  Broader than separation so more fish contribute to the
    # school's average direction.  Increasing this tightens heading lock
    # across larger subgroups; decreasing fragments the school into pods.
    radius_alignment: float = 85.0

    # Radius within which a sardine is attracted toward the local center of
    # mass (pixels).  Should be ≥ radius_alignment.  Setting it much larger
    # than alignment causes fish to chase the COM of distant neighbors they
    # cannot see, which collapses the school into one clump.
    radius_cohesion: float = 90.0

    # ═══════════════════════════════════════════════════════════════════
    # SPATIAL HASH / PERFORMANCE
    # ═══════════════════════════════════════════════════════════════════

    # Cell size for the primary sardine spatial hash (pixels).  Neighbor
    # queries scan a 3×3 ring of cells centered on the querying fish.  Setting
    # cell_size ≥ radius_cohesion ensures one ring scan covers all relevant
    # neighbors.  Smaller cells mean more cells to scan per query (slower);
    # larger cells mean each cell contains more fish (slower inner loop).
    # The sweet spot is cell_size ≈ max(radius_alignment, radius_cohesion).
    # Do NOT change this while the simulation is running without also calling
    # Simulation.apply_config() — the grid is rebuilt each frame but the span
    # caches on each sardine are refreshed only in apply_config.
    cell_size: float = 80.0

    # ═══════════════════════════════════════════════════════════════════
    # WORLD BOUNDARY
    # ═══════════════════════════════════════════════════════════════════

    # Boundary wrapping mode.  "wrap" = toroidal: fish that exit one edge
    # re-enter the opposite edge, simulating open ocean without walls.
    # Any other value activates reflective/clamping walls (not fully
    # implemented — "wrap" is strongly recommended for flocking stability).
    boundary: str = "wrap"

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE VISUAL APPEARANCE
    # ═══════════════════════════════════════════════════════════════════

    # Half-length of a sardine's body triangle in pixels.  The rendered body
    # spans roughly 3× this value.  Increasing this makes sardines easier to
    # see individually at large populations; decreasing allows more fish to
    # fit on screen without crowding.  Does not affect any physics radius.
    sardine_size: float = 6.0

    # Background fill color as RGBA bytes (0–255 each).  Default is a
    # deep-water midnight blue.  Applied once per frame via glClear.
    background_color: tuple[int, int, int, int] = (0, 20, 50, 255)

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE COLOR SYSTEM — DRIVER + PALETTE
    # ═══════════════════════════════════════════════════════════════════
    #
    # The color pipeline has two independent stages:
    #   1. Driver: selects a per-fish normalized scalar in [0, 1].
    #   2. Palette: maps that scalar to an RGB color.
    # Cycle driver with the config menu; cycle palette with the V key at runtime.

    # Which kinematic signal drives the per-fish color scalar.
    #   "velocity" — composite of heading direction, forward speed, and
    #                turn rate (legacy default behavior).
    #   "age"      — normalized lifetime: 0.0 at birth → 1.0 at
    #                sardine_age_gradient_seconds.  Older fish appear at
    #                the palette's high end.  Useful for seeing generation
    #                turnover and identifying long-lived individuals.
    sardine_color_driver: str = "velocity"

    # Which color palette maps the scalar to a visible color.
    #   "white"   — static silvery white; driver value ignored.  Best for
    #               performance or when visual color adds no information.
    #   "rainbow" — full HSV wheel (0°–360° mapped to scalar 0–1).
    #   "cool"    — blues / teals / purples; constrained hue band.
    #   "warm"    — reds / oranges / yellows; constrained hue band.
    #   "ocean"   — deep aquatic blue-teal-green; slightly dimmer than cool.
    #   "sunset"  — purples / magentas / pinks with orange wrap.
    #   "mono"    — grayscale; scalar maps to brightness only.
    sardine_color_palette: str = "cool"

    # Per-channel weights for the velocity color driver.  All are
    # dimensionless multipliers; 0.0 disables that channel entirely.

    # How strongly the sardine's heading direction (angle) cycles through the
    # palette hue.  1.2 means one full palette cycle per full rotation.
    # Increase for rapid color cycling as fish change direction; set to 0 to
    # remove heading contribution entirely.
    vel_color_heading_weight: float = 1.2

    # How strongly forward speed shifts the hue.  At weight=1 a sardine at
    # max speed offsets hue by roughly ¼ of the palette wheel.  Increase for
    # stronger speed-to-color mapping; useful for identifying fast-fleeing fish.
    vel_color_speed_weight: float = 1.2

    # How strongly turning rate desaturates the color.  Higher values make
    # sharply-turning fish appear washed-out, highlighting panic maneuvers.
    # Keep ≤ 2.0 to avoid all fish looking white during normal swimming.
    vel_color_turn_weight: float = 1.4

    # Exponential moving average coefficient for the per-fish turn-rate
    # estimate used by vel_color_turn_weight.  Range [0, 0.99].  Values
    # close to 1.0 give heavy smoothing (slow response); values near 0 make
    # turn color react immediately but look jittery.  0.84 is a good balance.
    vel_color_smoothing: float = 0.84

    # Age normalization window in seconds for the "age" color driver.  A
    # sardine exactly this old maps to a normalized scalar of 1.0.  Fish
    # older than this clamp to 1.0.  Decrease to see generational turnover
    # faster; increase to stretch the gradient over longer lifespans.
    sardine_age_gradient_seconds: float = 60.0

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH MOVEMENT
    # ═══════════════════════════════════════════════════════════════════

    # Maximum cruise speed for sailfish in pixels per second.  Real sailfish
    # reach ~3× sardine sprint speed; 250 px/s (≈ 1.67× sardine max) keeps
    # pursuits visible and competitive without making sardines impossible to
    # catch.  Increasing this accelerates repositioning during orbit and
    # makes herding pressure more effective; too high causes sailfish to
    # overshoot baitballs and miss strikes.
    # Interaction: should remain ≥ 1.3× sardine_max_speed for reliable prey
    # acquisition.  During burst attacks this is further multiplied by
    # sailfish_burst_speed_mult.
    sailfish_max_speed: float = 250.0

    # Minimum swim speed for sailfish in pixels per second.  Prevents full
    # stops that would break flocking physics.  Keep at roughly 30–35% of
    # sailfish_max_speed.  If set too high sailfish cannot slow down for
    # tight baitball orbit work and will overshoot their slots.
    sailfish_min_speed: float = 80.0

    # Maximum steering force for sailfish in pixels per second squared.
    # force / speed = 0.48 < sardine's 1.0, giving sailfish wider, less
    # snappy turning arcs — more like a large predatory fish and less like a
    # nimble sardine.  Increasing this allows tighter turns during orbit but
    # can produce jittery heading changes.  During burst attacks this is
    # multiplied by sailfish_burst_force_mult.
    sailfish_max_force: float = 120.0

    # Detection radius within which a sailfish can see sardines and begin
    # tracking the school center-of-mass (pixels).  This is the outermost
    # awareness boundary.  All sardines within this range are visible to the
    # sailfish and contribute to COM calculations and baitball detection.
    # Increasing it causes predators to engage schools earlier and remain
    # active longer; decreasing it creates more patrol time between hunts.
    # Performance note: this is the radius used for the sardine-grid query
    # inside each sailfish's _compute_forces — large values scan more cells.
    radius_sailfish_awareness: float = 350.0

    # Force weight on the long-range COM-pursuit steer applied in
    # TRACK_SCHOOL state.  This is the "heading toward the school" force.
    # Higher values make sailfish converge on schools faster; lower values
    # give a more casual approach.  During ORBIT_BAITBALL a reduced fraction
    # (0.35×) of this is still applied as a loose centering bias.
    weight_pursuit: float = 1.6

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH STATE MACHINE — INDIVIDUAL TARGET LOCK
    # ═══════════════════════════════════════════════════════════════════

    # Radius within which a sailfish locks onto an individual sardine for
    # predictive interception pursuit (pixels).  Must be smaller than
    # radius_sailfish_awareness.  When baitball herding is active, lock
    # acquisition is suppressed unless the target is on the baitball edge
    # (controlled by sailfish_target_lock_suppressed_near_baitball).
    # Smaller values make individual lock harder to acquire, keeping sailfish
    # in COM-pursuit mode longer; larger values cause early lock-on before
    # the fish is really close, which can be less effective than baitball
    # herding when a dense school is present.
    sailfish_target_acquire_radius: float = 150.0

    # Distance at which an existing individual lock is released (pixels).
    # If the locked sardine drifts beyond this radius the lock breaks and the
    # sailfish returns to TRACK_SCHOOL.  Should be meaningfully larger than
    # sailfish_target_acquire_radius (ideally 1.5–2× so targets are not
    # immediately re-acquired after a brief escape).
    sailfish_target_drop_radius: float = 260.0

    # Maximum duration (seconds) of a single individual target lock before
    # the sailfish automatically releases and re-evaluates.  Prevents stale
    # lock-on when a sardine leads the predator on a long chase without
    # getting close enough for a burst.  The sailfish may immediately
    # re-acquire the same or a different target on the next evaluation.
    sailfish_target_lock_time: float = 4.0

    # Look-ahead cap (seconds) for predictive interception in TARGET_LOCK
    # and BURST_ATTACK states.  The intercept point is computed as the
    # sardine's current position + velocity × min(dist/sailfish_speed,
    # prediction_time).  A larger cap allows longer-range prediction but
    # overshoots when the sardine changes direction sharply.  0.25 s is
    # conservative enough to handle most evasive maneuvers while still
    # leading the target.
    sailfish_prediction_time: float = 0.25

    # Force weight on the individual-target predictive pursuit steer.
    # Slightly higher than weight_pursuit so close-in chasing is more
    # decisive than the long-range COM approach.  Values around 2.0 produce
    # tight, intentional pursuit; much higher values cause overshooting.
    weight_target_pursuit: float = 2.0

    # When True, suppresses new individual TARGET_LOCK acquisition while a
    # dense baitball is active, unless the candidate sardine is in the
    # outer edge band (see sailfish_baitball_edge_band_width).  This prevents
    # sailfish from abandoning the cooperative herding formation to chase
    # individual sardines deep in the school.  Set False to allow individual
    # lock regardless of baitball state (reverts to pre-baitball behavior).
    sailfish_target_lock_suppressed_near_baitball: bool = True

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH STATE MACHINE — BURST ATTACK
    # ═══════════════════════════════════════════════════════════════════

    # Distance (pixels) from a locked target that triggers a burst sprint.
    # When the target enters this radius the sailfish switches from sustained
    # pursuit to a short-duration high-speed burst.  Smaller values delay the
    # burst until the fish is very close (precise but harder to trigger);
    # larger values start the burst early (wastes speed on a longer sprint).
    sailfish_burst_trigger_radius: float = 90.0

    # Duration of a single burst in seconds.  Long enough to close on typical
    # evasive sardines but short enough that the energy cost (recovery period)
    # is meaningful.  Set to 0.0 to disable bursts entirely — the sailfish
    # will still pursue in TARGET_LOCK but never escalate to BURST_ATTACK.
    sailfish_burst_duration: float = 0.6

    # Cooldown between consecutive bursts (seconds).  After a burst ends the
    # sailfish enters RECOVER for sailfish_recovery_duration, then must wait
    # an additional burst_cooldown before the next burst can fire.  Longer
    # cooldowns space out attacks realistically; too short allows continuous
    # sprinting which makes sailfish nearly impossible to escape.
    sailfish_burst_cooldown: float = 5.0

    # Speed multiplier during a burst attack, applied on top of
    # sailfish_max_speed.  1.45× gives a 45% speed boost — enough to
    # decisively close on a fleeing sardine.  Values above ~1.8 risk
    # overshooting the target within a single frame step.
    sailfish_burst_speed_mult: float = 1.45

    # Force multiplier during a burst attack, applied on top of
    # sailfish_max_force.  Higher values allow tighter turning at burst speed,
    # which is important for tracking evasive sardines.  Tune alongside
    # burst_speed_mult: high speed with low force means the sailfish will
    # overshoot turns; matched values (both ~1.5×) produce clean interception.
    sailfish_burst_force_mult: float = 1.6

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH STATE MACHINE — RECOVERY
    # ═══════════════════════════════════════════════════════════════════

    # Duration of the RECOVER state after a burst or baitball strike ends
    # (seconds).  During recovery the sailfish moves at normal speed toward
    # the school COM but cannot lock a new target or trigger another burst.
    # This models post-sprint fatigue and creates a natural rhythm of attack
    # → rest → reposition.  If a baitball is still active when recovery ends,
    # the sailfish preferentially returns to ORBIT_BAITBALL.
    sailfish_recovery_duration: float = 1.5

    # Time (seconds) a sailfish can stay in TRACK_SCHOOL without seeing any
    # sardines before falling back to PATROL wander.  This handles the case
    # where sardines move out of awareness range mid-chase.  Shorter values
    # produce more alert-then-patrol cycling; longer values make sailfish
    # persist in the pursuit mindset even when the school is temporarily
    # hidden.
    sailfish_no_sardines_timeout: float = 5.0

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL DETECTION
    # ═══════════════════════════════════════════════════════════════════
    #
    # The baitball detector scans the sardine spatial hash each frame to find
    # dense clusters.  It is O(occupied cells × neighborhood), not O(n²), so
    # it scales safely with large sardine populations.  Detected baitballs are
    # pushed to every sailfish via set_baitball_context() before their update.

    # Master toggle for baitball detection and all cooperative sailfish
    # behavior that depends on it (herding, orbit, baitball strikes).
    # Disabling this reverts sailfish to fully independent individual hunters.
    sailfish_baitball_enabled: bool = True

    # Spatial radius (pixels) used by the baitball detector to scan for
    # sardine clusters.  Each occupied grid cell is tested against its
    # neighbors within this radius to count local sardine density.  Larger
    # values detect more diffuse clusters; too large and nearly any school
    # qualifies as a "baitball," making the cooperative herding trigger too
    # frequently.  Should be tuned so only genuinely dense schools trigger.
    sailfish_baitball_detection_radius: float = 120.0

    # Minimum number of sardines that must be present within the detection
    # radius to register a baitball.  Prevents very small groups from
    # triggering cooperative herding, which would waste all sailfish on a
    # handful of fish.  Increase for more selective baitball detection.
    sailfish_baitball_min_sardines: int = 24

    # Minimum density score (sardines / radius) for a cluster to be
    # recognized as a baitball.  The score normalizes for cluster size, so a
    # small tight ball can qualify while a large diffuse school cannot.
    # Typical range: 0.1 (loose) to 0.5 (very tight).  Increase to require
    # tighter packing before sailfish cooperate; decrease to trigger herding
    # on more diffuse schools.
    sailfish_baitball_density_threshold: float = 0.24

    # How long (seconds) a sailfish remembers a previously detected baitball
    # after it disappears from the current frame's detection results.  This
    # handles transient detection gaps when sardines rearrange.  Set to 0.0
    # to require continuous detection (more reactive but less stable orbits).
    sailfish_baitball_memory_seconds: float = 1.8

    # ── Post-detection candidate merging ──────────────────────────────────
    #
    # The per-cell detector generates one candidate baitball per occupied
    # grid cell, so a single large school can produce many overlapping
    # candidates.  The merge pass collapses them into one stable baitball.
    #
    # Two candidates A and B are merged when either condition is true:
    #   center_dist < merge_distance_multiplier × max(r_A, r_B)   [near]
    #   center_dist < (r_A + r_B) × merge_overlap_threshold        [overlap]
    # The merged baitball uses count-weighted center and the smallest
    # enclosing radius from that center.  Merge is greedy, strongest
    # candidate (density × count) first.

    # Multiplier applied to the larger radius to form the "near" merge
    # distance test.  1.2 means "merge if centers are within 1.2 × max
    # radius."  Raise to merge more aggressively (good for elongated schools);
    # lower to keep slightly separated clusters as distinct baitballs.
    sailfish_baitball_merge_distance_multiplier: float = 0.8

    # Overlap fraction for the second merge test: merge if the circles
    # overlap by more than this fraction of their combined radii.  0.55
    # means the centers must be within 55 % of (r_A + r_B), i.e. more than
    # 45 % overlap.  Raise toward 1.0 to merge all touching circles; lower
    # to require substantial overlap before merging.
    sailfish_baitball_merge_overlap_threshold: float = 0.2

    # Maximum number of distinct baitballs returned after merging.  Replaces
    # the old hardcoded cap of 4.  Raise to support large worlds with many
    # genuinely separate dense schools; lower to keep sailfish cooperation
    # focused on fewer targets.
    sailfish_baitball_max_final_baitballs: int = 6

    # Maximum radius (pixels) a merged baitball may grow to before it is
    # locked from accepting further candidates.  Once a merged baitball
    # reaches this size it stays segmented — subsequent candidates that would
    # push it larger instead start a new baitball (if the cap allows) or are
    # discarded.  This prevents a giant map-scale school from collapsing into
    # one enormous entity and keeps sailfish behavior localized to distinct
    # sub-clusters.  Set to 0 to disable the limit (no maximum radius).
    sailfish_baitball_max_merge_radius: float = 400.0

    # When True, suppresses new individual TARGET_LOCK acquisition while a
    # dense baitball is active, unless the candidate sardine is in the
    # outer edge band (see sailfish_baitball_edge_band_width).  This prevents
    # sailfish from abandoning the cooperative herding formation to chase
    # individual sardines deep in the school.  Set False to allow individual
    # lock regardless of baitball state (reverts to pre-baitball behavior).
    sailfish_target_lock_suppressed_near_baitball: bool = True

    # ═══════════════════════════════════════════════════════════════════
    # PROTO-COLLABORATION / TURN-TAKING
    # ═══════════════════════════════════════════════════════════════════
    #
    # Sailfish take turns striking rather than all attacking simultaneously.
    # A frame-level count of active baitball attackers (injected by
    # Simulation into each sailfish) gates new strike attempts.

    # Orbit patience: minimum seconds in ORBIT_BAITBALL before a sailfish
    # may attempt any strike (center pass-through or edge target).  Enforces
    # patient positioning before attacking — prevents fish from immediately
    # striking on entering orbit.  Each fish randomizes this by adding a
    # jitter value (see sailfish_baitball_patience_jitter_seconds).
    sailfish_baitball_patience_seconds: float = 3.5

    # Hard minimum orbit time that cannot be reduced by negative jitter
    # (seconds).  Even the most impatient fish (maximum negative jitter) will
    # orbit for at least this long before striking.
    sailfish_baitball_min_orbit_seconds: float = 2.5

    # Per-fish patience randomization range (seconds).  Each fish's actual
    # patience goal = patience_seconds + uniform(−jitter, +jitter), clamped
    # to min_orbit_seconds.  Higher jitter causes fish to strike at different
    # times, creating the staggered turn-taking pattern.
    sailfish_baitball_patience_jitter_seconds: float = 1.2

    # Group-level strike cooldown (seconds).  After any sailfish launches a
    # baitball strike, all sailfish see this cooldown and cannot initiate a
    # new group attack until it expires.  This is the primary mechanism that
    # enforces turn-taking across the whole group.  Increase for longer pauses
    # between any two consecutive attacks; decrease for more frequent strikes.
    # Interaction: should be longer than sailfish_recovery_duration so the
    # attacker has time to recover before the next fish strikes.
    sailfish_baitball_strike_group_cooldown_seconds: float = 4.5

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL STRIKE — EDGE TARGET MODE
    # ═══════════════════════════════════════════════════════════════════
    #
    # After sufficient orbit patience, a sailfish may switch to TARGET_LOCK
    # on a sardine in the outer band of the baitball.  This is the more
    # common strike type; center pass-throughs are rarer.

    # Per-second probability that an orbit-ready sailfish launches an edge
    # target strike (given all cooldowns are clear).  Converted to a
    # per-frame probability internally.  0.14/s means a ready sailfish will
    # attempt an edge strike roughly every 7 seconds on average once patience
    # is satisfied.  Increase for more frequent edge strikes.
    sailfish_baitball_edge_target_chance: float = 0.14

    # Per-second probability that an orbit-ready sailfish launches a center
    # pass-through strike.  Should be lower than edge target chance to make
    # center passes a relatively rare dramatic event (roughly 1:2 ratio with
    # edge strikes in practice).
    sailfish_baitball_center_strike_chance: float = 0.06

    # Width of the "edge band" around the baitball perimeter from which edge
    # targets are selected (pixels).  Only sardines at a distance from the
    # baitball center between (baitball.radius − edge_band_width) and
    # baitball.radius are eligible.  Increasing this widens the zone and
    # makes more sardines eligible; decreasing it restricts attacks to the
    # outermost shell only.  Should be smaller than the baitball radius to
    # prevent targets from the school interior being selected.
    sailfish_baitball_edge_band_width: float = 18.0

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL ORBIT FORCE MODEL
    # ═══════════════════════════════════════════════════════════════════

    # Orbit radius multiplier used specifically in ORBIT_BAITBALL state
    # (separate from the deprecated sailfish_herding_radius_multiplier).
    # Values > 1.0 place sailfish outside the school perimeter.  1.95×
    # positions fish at 95% beyond the school edge — enough standoff that
    # the school has breathing room and sardines are less likely to scatter
    # from fleet radius alone, keeping baitballs looser and more stable.
    sailfish_baitball_orbit_radius_multiplier: float = 1.95

    # Force weight on the tangential (circling) component of orbit motion.
    # Positive values make sailfish circle the baitball perimeter; higher
    # values increase the circling speed relative to the slot-hold force.
    # Setting to 0.0 makes fish sit statically at their slots (no orbit).
    # The tangential direction is determined per-fish by _orbit_dir.
    sailfish_baitball_orbit_tangent_strength: float = 0.95

    # Base inward pressure force weight.  Applies a mild push toward the
    # baitball center from the orbit position, helping compress the school.
    # This is the PRIMARY tuning dial for inward pressure — set it to the
    # desired force level.  It is then capped by sailfish_baitball_max_inward_pressure
    # (the absolute safety ceiling) and tapered by the inner falloff.
    # Accessible via the config menu "Orbit inward" control.
    # Keep low (0.10–0.25) to avoid translating the whole baitball across
    # the screen.  Most containment should come from the orbit slot force.
    sailfish_baitball_inward_pressure_strength: float = 0.12

    # Random angular jitter applied to each fish's orbit angle each frame
    # (scaled by this value, in radians).  Adds micro-variation to the orbit
    # position, preventing a rigid geometric ring.  Higher values produce
    # more organic spread; too high causes fish to drift away from their
    # intended slot entirely.
    sailfish_baitball_slot_jitter: float = 0.22

    # Base angular velocity of the orbit ring (radians per second).  Each
    # sailfish's actual orbit speed is this value multiplied by its personal
    # _orbit_speed_mult (set at spawn).  Slower values give more patient,
    # stationary containment; faster values produce visible rotation of the
    # whole group (which looks like flocking, not hunting).  Keep ≤ 0.5 for
    # hunting realism; 0.0 disables orbit rotation for static containment.
    sailfish_baitball_orbit_rotation_speed: float = 0.38

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL HERDING / COOPERATIVE ORBIT — SLOT ASSIGNMENT
    # ═══════════════════════════════════════════════════════════════════
    #
    # When a baitball is detected, sailfish enter ORBIT_BAITBALL and are
    # assigned stable angular slots around the perimeter.  The slot system
    # distributes sailfish evenly by default but each fish has per-fish
    # variation (speed, radius, direction) to prevent conga-line orbiting.

    # Master toggle for cooperative herding / slot-orbit behavior.
    # When False, sailfish ignore baitballs and hunt individually.
    sailfish_herding_enabled: bool = True

    # DEPRECATED — not referenced by sailfish.py.  The effective orbit radius
    # multiplier is sailfish_baitball_orbit_radius_multiplier (BAITBALL ORBIT
    # FORCE MODEL section).  This field is retained only for backward
    # compatibility with saved boids_overrides.json files that include it;
    # changing its value has no effect on simulation behaviour.
    sailfish_herding_radius_multiplier: float = 1.35

    # Slot-hold force weight (dimensionless multiplier on the steer toward
    # the assigned orbit slot target position).  This is the primary force
    # keeping each sailfish at its designated angular position around the
    # baitball.  Increase for tighter slot adherence; decrease for a looser,
    # more drifting orbit.  Excessively high values cause fish to snap
    # rigidly to their slots and negate the per-fish variation system.
    sailfish_herding_pressure_strength: float = 0.95

    # Random angular offset applied to each sailfish's assigned slot (radians,
    # scaled by this value).  Breaks perfect symmetry in the evenly-spaced
    # ring.  Higher values allow adjacent fish to drift closer together,
    # creating a more organic (less geometric) distribution.
    sailfish_herding_slot_jitter: float = 0.35

    # Minimum number of sailfish required before cooperative herding
    # activates.  Below this count, individual hunting is more efficient.
    # Keep ≥ 2; a single herding sailfish cannot create meaningful pressure.
    sailfish_herding_min_sailfish: int = 3

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL STRIKE — CENTER PASS-THROUGH MODE
    # ═══════════════════════════════════════════════════════════════════
    #
    # The less common strike type: a sailfish at high speed straight through
    # the baitball center on a fixed exit line, eating any sardines contacted
    # along the way.  Uses BAITBALL_STRIKE state (distinct from TARGET_LOCK).

    # Master toggle for center pass-through strikes.  When False, sailfish
    # can only use edge target strikes and never enter BAITBALL_STRIKE state.
    sailfish_baitball_strike_enabled: bool = True

    # Minimum baitball density score required to permit a center pass-through
    # strike.  Higher than the detection threshold — only very dense, compact
    # schools are worth a high-speed center pass.  Increase for more
    # selective center strikes.
    sailfish_baitball_strike_density_threshold: float = 0.34

    # Legacy field — formerly used as the single strike-chance probability.
    # Now superseded by sailfish_baitball_center_strike_chance (per-second
    # probability for center passes) and sailfish_baitball_edge_target_chance
    # (per-second probability for edge target locks).  Retained for backward
    # compatibility with any code paths that still reference it.
    sailfish_baitball_strike_chance: float = 0.08

    # Speed multiplier during a center pass-through strike.  Applied on top
    # of sailfish_max_speed for the duration of the BAITBALL_STRIKE state.
    # Higher values produce a more dramatic burst through the school; too
    # high risks overshooting the exit point within a single frame step.
    sailfish_baitball_strike_speed_multiplier: float = 1.55

    # Duration of the BAITBALL_STRIKE state (seconds).  If the sailfish
    # reaches its exit point before this timer expires, it exits early.  If
    # the timer runs out first, it also exits.  0.75 s is long enough to
    # cross a typical baitball at strike speed.
    sailfish_baitball_strike_duration_seconds: float = 0.75

    # Per-fish cooldown after a center pass-through strike (seconds).  During
    # this time the sailfish cannot initiate another center strike or baitball
    # attack.  Interacts with the group cooldown; the effective wait is
    # max(individual_cooldown, group_cooldown).
    sailfish_baitball_strike_cooldown_seconds: float = 5.0

    # Distance beyond the baitball center the sailfish aims for as its exit
    # point (pixels).  The exit point is computed as baitball_center +
    # approach_direction × (baitball.radius + exit_distance).  Larger values
    # push the exit point deeper past the school, producing a longer transit;
    # smaller values exit sooner (shorter pass, fewer contacts).
    sailfish_baitball_strike_exit_distance: float = 36.0

    # ═══════════════════════════════════════════════════════════════════
    # ANTI-CONGA-LINE ORBIT VARIATION
    # ═══════════════════════════════════════════════════════════════════
    #
    # Each sailfish receives per-fish random multipliers at spawn that
    # persist for its lifetime.  These break the synchronized ring rotation
    # that produces a conga-line effect where all fish follow the same path
    # one after another.

    # Range of per-fish orbit speed variation.  Each fish's orbit speed
    # is multiplied by a random value drawn from [1 - jitter, 1 + jitter].
    # 0.55 means individual fish orbit at 45%–155% of the base orbit speed,
    # causing them to naturally drift apart over time.  Set to 0.0 for
    # perfectly synchronized rotation (conga-line behavior).
    sailfish_orbit_speed_jitter: float = 0.55

    # Range of per-fish orbit radius variation.  Each fish's orbit radius is
    # multiplied by a random value drawn from [1 - jitter, 1 + jitter].
    # 0.22 means individual fish orbit at 78%–122% of the target radius,
    # giving the ring a loose radial spread rather than a perfect circle.
    sailfish_orbit_radius_jitter: float = 0.22

    # Fraction of sailfish assigned counter-clockwise orbit direction at
    # spawn.  The remainder orbit clockwise.  0.35 means roughly 35% CCW and
    # 65% CW.  Having a mix of directions is the single most effective way to
    # prevent a conga line — fish orbiting in opposite directions cannot form
    # a synchronized following chain.  0.5 = half each way; 0.0 = all CW.
    sailfish_orbit_ccw_fraction: float = 0.35

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH LATERALIZATION (stable per-fish attack side preference)
    # ═══════════════════════════════════════════════════════════════════

    # Master toggle for per-fish lateralization.  When True each sailfish
    # receives a stable attack_side preference (LEFT = -1 or RIGHT = +1) and
    # a lateralization_bias strength at spawn.  These persist for the fish's
    # lifetime and bias: (a) which side of the baitball the fish approaches
    # from when selecting edge targets, and (b) which side of center the
    # pass-through corridor is offset toward during center strikes.  The bias
    # does not force behavior 100%; it scores candidates and offsets geometry.
    sailfish_lateralization_enabled: bool = True

    # Overall strength of the lateralization bias, scaled by each fish's
    # personal lateralization_bias value (a per-fish random in [0.35, 1.0]).
    # The effective bias per fish = sailfish_lateralization_strength ×
    # fish.lateralization_bias.  At 0.65 a fully biased fish (bias=1.0)
    # scores preferred-side edge targets roughly 65% of the baitball radius
    # higher than same-distance opposite-side targets.  At 0.0 lateralization
    # is disabled even if the toggle is on.
    sailfish_lateralization_strength: float = 0.65

    # ═══════════════════════════════════════════════════════════════════
    # PROTO-COLLABORATION — PER-FISH COOLDOWNS & CAPS
    # ═══════════════════════════════════════════════════════════════════

    # Maximum number of sailfish allowed to be in a baitball attack
    # simultaneously (BAITBALL_STRIKE, or TARGET_LOCK/BURST_ATTACK launched
    # from orbit).  Set to 1 for strict turn-taking; 2 allows occasional
    # paired attacks while still preventing full group rushes.  Values above
    # ~3 produce behavior indistinguishable from uncoordinated simultaneous
    # strikes.
    sailfish_max_simultaneous_baitball_strikes: int = 2

    # Per-fish cooldown (seconds) before the same sailfish can launch another
    # baitball attack after completing one.  Distinct from the group-level
    # cooldown (sailfish_baitball_strike_group_cooldown_seconds) which gates
    # all sailfish together.  This per-fish cooldown ensures individual fish
    # spend time recovering and repositioning before striking again, creating
    # a natural rotation of attackers over time.
    sailfish_individual_strike_cooldown_seconds: float = 7.0

    # ═══════════════════════════════════════════════════════════════════
    # BAITBALL ANCHOR / DRIFT DAMPING
    # ═══════════════════════════════════════════════════════════════════

    # Master toggle for drift damping.  When True, each orbiting sailfish
    # estimates the baitball's frame-to-frame movement and applies a counter-
    # force proportional to the drift velocity.  This counteracts the tendency
    # for multiple sailfish applying inward pressure from one side to
    # translate the whole school across the screen.  The forces from multiple
    # sailfish on different sides naturally cancel when the baitball is
    # stationary, but damping corrects residual drift when they don't.
    sailfish_baitball_anchor_enabled: bool = True

    # Force weight on the drift-damping counter-force (dimensionless steer
    # multiplier).  Higher values more aggressively oppose baitball drift;
    # too high can cause oscillation where the baitball bounces back and
    # forth.  0.60 provides firm anchoring that counteracts residual drift
    # from the reduced inward pressure.  Only triggers when detected drift
    # speed exceeds 3 px/s to filter position noise.
    sailfish_baitball_drift_damping_strength: float = 0.60

    # Absolute safety ceiling on the inward-pressure force per sailfish per
    # frame (dimensionless steer weight).  The effective inward force is
    # min(sailfish_baitball_inward_pressure_strength, this value), so this
    # cap only bites if strength is set higher than the ceiling.  Prevents
    # runaway compression if the strength dial is turned up too far.
    # The cap is applied before the falloff taper.
    sailfish_baitball_max_inward_pressure: float = 0.16

    # Radial falloff threshold as a fraction of the orbit radius.  When a
    # sailfish is inside this fraction of its orbit radius (measured from the
    # baitball center), inward pressure is linearly tapered to zero.  A fish
    # at the baitball center applies zero inward force; one at the falloff
    # boundary applies full force.  This prevents fish that drift inside the
    # school from applying inward force in the wrong direction and pushing
    # sardines outward.  Range [0.0, 1.0]; 0.85 means the inner 85% of the
    # orbit ring is a "soft zone" with reduced inward pressure, further
    # reducing compression on fish at typical orbit distances.
    sailfish_baitball_inner_pressure_falloff: float = 0.85

    # ═══════════════════════════════════════════════════════════════════
    # ATTACK PRESSURE SUPPRESSION
    # ═══════════════════════════════════════════════════════════════════

    # Scale factor [0.0–1.0] applied to all herding forces for a sailfish
    # that is currently performing a baitball attack (BAITBALL_STRIKE state,
    # or TARGET_LOCK/BURST_ATTACK launched from orbit).  At 0.08, an
    # attacking sailfish contributes only 8% of its normal herding pressure.
    # This prevents the attacker from simultaneously compressing the school
    # AND chasing a target — the two behaviors would interfere.  Non-attacking
    # sailfish maintain full containment while the attacker focuses on its
    # target.  Set to 1.0 to revert to the old behavior where all sailfish
    # always apply full herding regardless of attack state.
    sailfish_attack_pressure_suppression: float = 0.08

    # ═══════════════════════════════════════════════════════════════════
    # INTER-SAILFISH SPACING DURING ORBIT
    # ═══════════════════════════════════════════════════════════════════
    #
    # An additional lateral separation force applied between neighboring
    # sailfish during baitball orbit.  This is distinct from the general
    # sailfish-to-sailfish flocking separation (weight_sailfish_separation)
    # and specifically targets the conga-line pattern where fish follow
    # directly behind each other along the orbit path.

    # Master toggle for the orbit spacing force.  When False, only the
    # general sailfish flocking separation applies during orbit (which is
    # radially oriented and does not specifically break conga lines).
    sailfish_spacing_enabled: bool = True

    # Radius within which sailfish push each other apart during orbit
    # (pixels).  Sailfish within this distance of each other experience the
    # spacing separation force.  Should be larger than
    # radius_sailfish_separation (52 px) to catch fish that are not quite
    # within the general separation radius but are still close enough to
    # form a visual chain.  85 px ≈ 7 sailfish body-lengths.
    sailfish_spacing_radius: float = 85.0

    # Force weight on the lateral orbit spacing push.  Deliberately weaker
    # than weight_sailfish_separation (3.0) so it does not dominate movement
    # or break orbit slot adherence.  Its purpose is to create a gentle
    # spreading tendency, not strong repulsion.
    sailfish_spacing_strength: float = 1.5

    # Force weight on the following-penalty counter-tangential impulse.
    # When sailfish B is ahead of sailfish A in the orbit direction, A
    # receives a small force opposing its tangential motion.  This slows A's
    # orbital progress relative to B, causing them to spread apart over time
    # rather than following at a fixed distance.  Set to 0.0 to disable
    # following-penalty (only spatial separation applies).
    sailfish_following_penalty_strength: float = 0.7

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH VULNERABLE-TARGET SCORING
    # ═══════════════════════════════════════════════════════════════════
    #
    # When a sailfish has multiple candidates within the acquire radius it
    # selects the most "vulnerable" one using a weighted score.  Each weight
    # is dimensionless; set to 0.0 to disable that factor entirely.

    # Proximity score weight.  Prefers sardines that are already closer to
    # the sailfish (less distance to cover = lower interception cost).
    # Normalized within the acquire radius so the nearest possible sardine
    # scores 1.0 and one at the boundary scores 0.0.
    sailfish_target_dist_weight: float = 1.0

    # Isolation score weight.  Prefers sardines that are far from the local
    # sardine school center-of-mass — edge fish are less protected by
    # neighbors and harder for the school to shield.  This encourages
    # realistic predation of peripheral, straggling fish.
    sailfish_target_isolation_weight: float = 1.4

    # Slowness score weight.  Prefers sardines moving slower than
    # sardine_max_speed.  Slow fish are easier to intercept and may indicate
    # injured, old, or disoriented individuals.  Normalized by max_speed^2.
    sailfish_target_speed_weight: float = 0.8

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH STATE MACHINE — SCHOOL-PRESSURE ORBIT (non-baitball)
    # ═══════════════════════════════════════════════════════════════════
    #
    # When a sailfish is in TRACK_SCHOOL without an individual target and the
    # school is dense (≥ density_threshold sardines visible), it adds a
    # tangential orbit force around the school COM rather than diving straight
    # in.  This produces circling behavior even without a detected baitball.

    # Force weight on the tangential orbit steer around the school COM.
    # 0.0 disables orbiting entirely (sailfish always dive straight at COM).
    # Values around 0.8 give a loose circling feel without breaking pursuit.
    sailfish_school_pressure_strength: float = 0.8

    # Minimum number of visible sardines required to activate the school-
    # pressure orbit bias.  Below this threshold the sailfish dives straight
    # at the COM.  Prevents the orbit from activating on lone or sparse
    # sardines where circling would be counterproductive.
    sailfish_school_pressure_density_threshold: int = 11

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH FLOCKING (sailfish-to-sailfish interactions)
    # ═══════════════════════════════════════════════════════════════════
    #
    # Sailfish flock loosely with one another but their primary driver is
    # prey pursuit.  Separation spreads them around a school; alignment adds
    # light heading coordination; cohesion is off because pursuit already
    # groups them near prey.  These forces are applied in addition to all
    # prey-hunting forces every frame.

    # Separation weight for sailfish avoiding each other.  Deliberately
    # stronger than sardine separation (3.0 vs 2.0) to spread them apart
    # around prey rather than stacking.  Increase for more spread; decrease
    # to allow tight packs near a school (which tends to overwhelm prey too
    # quickly).
    weight_sailfish_separation: float = 3.0

    # Alignment weight for sailfish heading coordination.  Set to 0.0 by
    # default because prey pursuit already provides a shared heading signal.
    # Small non-zero values (0.2–0.5) add a subtle "pack" feeling where
    # sailfish vaguely face the same direction.
    weight_sailfish_alignment: float = 0.0

    # Cohesion weight for sailfish grouping.  Off by default — prey pursuit
    # clusters sailfish naturally near sardine schools.  A non-zero value
    # would pull all sailfish toward the group center even when hunting in
    # different directions, which can interfere with independent baitball
    # pressure zones.
    weight_sailfish_cohesion: float = 0.0

    # Separation perception radius for sailfish-to-sailfish avoidance
    # (pixels).  Roughly 2 sailfish body-lengths at size=12.  Must be smaller
    # than radius_sailfish_alignment and radius_sailfish_cohesion.
    radius_sailfish_separation: float = 52.0

    # Alignment perception radius for sailfish-to-sailfish heading matching
    # (pixels).  All sailfish within this range contribute to the average
    # heading used by alignment.
    radius_sailfish_alignment: float = 100.0

    # Cohesion perception radius for sailfish grouping (pixels).  Kept equal
    # to radius_sailfish_alignment so every cohesion neighbor is also an
    # alignment neighbor — avoids attraction to fish whose heading you cannot
    # see.
    radius_sailfish_cohesion: float = 150.0

    # Half-length of a sailfish body triangle in pixels.  At 12 px (2×
    # sardine_size) the rendered body spans ~36 px, making sailfish clearly
    # visible as the larger predator.  Does not affect any physics radius.
    sailfish_size: float = 12.0

    # Color scheme for all sailfish.  All sailfish share one color; there is
    # no per-fish color variation.  Cycle with ← / → in the SAILFISH —
    # FLOCKING config menu section.
    #   "default" — vivid orange-red (high contrast against deep-water blue)
    #   "white"   — silvery white (similar to sardines but larger)
    #   "red"     — bright red
    #   "blue"    — electric blue
    #   "black"   — near-black (still visible against the dark background)
    sailfish_color_mode: str = "white"

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE FLEE / PREDATOR AWARENESS
    # ═══════════════════════════════════════════════════════════════════

    # Radius within which sardines detect and flee from sailfish (pixels).
    # This is the range for the flee force computation: each sardine queries
    # the sailfish spatial hash for predators within this radius and applies
    # a weighted escape steer away from each one.  Larger values create
    # earlier, stronger school scatter; smaller values let predators approach
    # more closely before the school reacts, making dense baitballs more
    # stable (prey don't scatter at a distance).
    # Interaction: increasing this relative to sailfish_baitball_orbit_radius
    # may cause the school to disperse before sailfish can orbit it.
    radius_flee: float = 140.0

    # Force weight on the sardine flee steer.  Must substantially exceed
    # weight_separation (2.0) so flight from predators overrides normal
    # personal-space rules.  Values around 15–25 produce strong but not
    # instant escape reactions.  Too high causes the entire school to warp
    # away from any nearby sailfish, preventing baitball formation.
    weight_flee: float = 20.0

    # Cell size for the sailfish-only spatial hash used by sardine flee
    # queries and sailfish-to-sailfish flocking.  Should be ≥ radius_flee
    # for a one-ring lookup, but sailfish are few so a larger cell is
    # acceptable (fewer cells, cheaper rebuild).
    sailfish_cell_size: float = 120.0

    # ═══════════════════════════════════════════════════════════════════
    # ALGAE MOVEMENT
    # ═══════════════════════════════════════════════════════════════════

    # Starting number of algae particles spawned at simulation launch.  Algae
    # are the base food source for sardines.  More algae → more food →
    # sardines reproduce faster and starve less.  Very high counts increase
    # rendering cost (each algae is a drawn shape) more than CPU cost (algae
    # flocking is cheap due to low count and simple rules).
    num_algae: int = 800

    # Maximum drift speed for algae (pixels per second).  Algae move much
    # slower than sardines (~6×) to simulate passive phytoplankton drift.
    # Increasing this causes algae to redistribute faster across the world;
    # decreasing it makes algae stay in roughly the same regions for longer.
    algae_max_speed: float = 25.0

    # Maximum steering force for algae (pixels per second squared).
    # The force/speed ratio of 0.48 gives gentle arcs.  Keeping force low
    # prevents algae from making sharp direction changes, preserving the
    # smooth drift feel.
    algae_max_force: float = 12.0

    # ═══════════════════════════════════════════════════════════════════
    # ALGAE FLOCKING
    # ═══════════════════════════════════════════════════════════════════
    #
    # Algae use the same Reynolds rules as sardines but are tuned for loose,
    # current-like drift rather than tight schooling.

    # Algae separation weight.  Prevents individual algae from stacking on
    # top of each other.  Slightly higher than cohesion (0.3 vs 0.1) to
    # maintain a healthy spread:cohesion ratio.
    weight_algae_separation: float = 0.3

    # Algae alignment weight.  The dominant force — all nearby algae drift in
    # roughly the same direction, simulating a slow ocean current.  The high
    # relative value (1.5 vs 0.1 cohesion) is what creates the "current
    # stream" feel rather than scattered random movement.
    weight_algae_alignment: float = 1.5

    # Algae cohesion weight.  Very weak — algae form loose aggregates but do
    # not cluster tightly.  Increasing above 0.2 tends to create dense clumps
    # that sardines exhaust quickly.
    weight_algae_cohesion: float = 0.1

    # Small random noise applied to algae each frame.  Same purpose as
    # sardine weight_noise but even lower since algae drift should be smooth.
    weight_algae_noise: float = 0.05

    # ═══════════════════════════════════════════════════════════════════
    # ALGAE PERCEPTION RADII
    # ═══════════════════════════════════════════════════════════════════
    #
    # Same constraint as sardines: r_sep < r_ali ≤ r_coh ≤ algae_cell_size.

    # Algae separation radius (pixels).  The minimum personal space around
    # each algae particle.  Raised from 12 to 18 px for a healthier
    # separation:cohesion ratio (1:3) that prevents dense clumping without
    # scattering particles too far apart.
    radius_algae_separation: float = 18.0

    # Algae alignment radius (pixels).  All algae within this range
    # contribute to the average drift direction.  Kept equal to cohesion
    # radius so every cohesion neighbor also aligns — smoother current
    # patches and fewer isolated drifters.
    radius_algae_alignment: float = 55.0

    # Algae cohesion radius (pixels).  Equal to alignment radius for the
    # reason described above.  Algae cell_size is set larger than this to
    # maintain a clean 3×3-ring lookup.
    radius_algae_cohesion: float = 55.0

    # Cell size for the algae-only spatial hash (pixels).  Must be ≥
    # radius_algae_cohesion for efficient single-ring queries.  Algae are
    # few enough that a slightly larger cell size has negligible cost.
    algae_cell_size: float = 60.0

    # Visual size of each algae particle — smaller than sardines to give them
    # a background-layer feel.  Does not affect any physics radius.
    algae_size: float = 4.0

    # ═══════════════════════════════════════════════════════════════════
    # ALGAE ECOLOGY
    # ═══════════════════════════════════════════════════════════════════

    # Contact radius for sardine–algae eating (pixels).  A sardine whose
    # center is within this distance of an algae center eats it.  Must be
    # larger than algae_size (4 px) to reliably register contact at 60 fps
    # when a sardine passes through the algae in a single frame step.  Too
    # large makes eating too easy and deplete algae too fast.
    algae_eat_radius: float = 14.0

    # Steady-state algae spawn rate (algae per second).  Uses a fractional
    # accumulator so non-integer rates work without bias.  The effective
    # ecosystem carrying capacity depends on the balance between this rate
    # and the rate at which sardines consume algae.  Set to 0.0 for a
    # finite-resource scenario (no respawn — useful for isolation tests).
    algae_respawn_rate: float = 150.0

    # Soft cap on the total algae population.  Respawn is paused while
    # len(algae) ≥ algae_max_count.  Must be ≥ num_algae to allow the
    # initial population to exist.  A lower cap combined with a high
    # sardine population creates food scarcity and drives starvation dynamics.
    algae_max_count: int = 2400

    # ═══════════════════════════════════════════════════════════════════
    # SARDINE ECOLOGY — POPULATION CAP, REPRODUCTION & STARVATION
    # ═══════════════════════════════════════════════════════════════════

    # Soft cap on the total sardine population.  Reproduction is blocked while
    # len(sardines) ≥ sardine_max_count.  0 = no cap (unlimited growth).
    # Manual keyboard spawns (↑ key) bypass this cap.  Setting this below
    # num_sardines prevents reproduction from the very start.
    sardine_max_count: int = 3000

    # Number of algae a sardine must eat before it reproduces (splits into
    # two sardines).  Each eaten algae increments the sardine's food_counter
    # by 1; reaching this threshold triggers reproduction and decrements the
    # counter so leftover progress carries forward.  Set to 0 to disable
    # sardine reproduction entirely.  Lower values → faster population
    # growth; higher values → slower reproduction, more starvation pressure.
    sardine_eat_threshold: int = 2

    # How long (seconds) a sardine can go without eating before it dies.
    # Set to 0.0 to disable sardine starvation entirely.  Longer windows
    # allow sardines to survive sparse algae patches; shorter windows create
    # more dramatic starvation die-offs when algae is depleted.
    # Interaction: with sardine_initial_hunger_time set appropriately,
    # starvation_time determines the maximum chase duration a sardine can
    # survive without feeding.
    sardine_starvation_time: float = 30.0

    # Initial hunger_timer value at sardine spawn (seconds).  Represents how
    # long the newly-spawned fish can survive before needing to eat.  Must be
    # strictly less than sardine_starvation_time.  Setting this lower
    # (e.g., 5 s) creates immediate feeding urgency; setting it near
    # starvation_time gives new fish an almost-full lifetime runway.
    sardine_initial_hunger_time: float = 15.0

    # ═══════════════════════════════════════════════════════════════════
    # SAILFISH ECOLOGY — POPULATION CAP, EATING, REPRODUCTION & STARVATION
    # ═══════════════════════════════════════════════════════════════════

    # Soft cap on the total sailfish population.  Reproduction is blocked while
    # len(sailfish) ≥ sailfish_max_count.  0 = no cap.  Manual keyboard
    # spawns (→ key) bypass this cap.
    sailfish_max_count: int = 9

    # Contact radius for sailfish eating a sardine (pixels).  A sailfish whose
    # center is within this distance of a sardine center eats it (one sardine
    # per sailfish per frame, preventing mass-kill sweeps).  Must be large
    # enough to register contact reliably at full burst speed; too large makes
    # eating trivially easy and can wipe out schools too quickly.
    sailfish_eat_radius: float = 25.0

    # How long (seconds) a sailfish can go without eating before it dies.
    # Apex predators need a longer runway than sardines (sardines starve in
    # 30 s; sailfish in 14 s at default — tuned for the active override
    # scenario where sardines are plentiful).  Set to 0.0 to disable
    # sailfish starvation.  Shorter values put more predation pressure on the
    # simulation to maintain ecosystem balance.
    sailfish_starvation_time: float = 14.0

    # Initial hunger_timer for a newly-spawned sailfish (seconds).  Must be
    # strictly less than sailfish_starvation_time.  Staggers earliest-possible
    # starvation deaths across the initial population so they don't all die
    # simultaneously on the first hunt failure.
    sailfish_initial_hunger_time: float = 7.0

    # Number of sardines a sailfish must eat before reproducing (producing one
    # offspring).  Set to 0 to disable sailfish reproduction.  With the
    # default of 40 and typical kill rates of ~1 sardine per hunt cycle,
    # reproduction takes tens of minutes, keeping the sailfish population
    # growth slow and ecosystem-stable.
    sailfish_eat_threshold: int = 40

    # Runtime toggle for sailfish starvation.  When False the starvation
    # ecology pass is skipped entirely each frame, regardless of hunger_timer
    # values.  Useful for studying predation without population collapse.
    sailfish_starvation_enabled: bool = True

    # Runtime toggle for sailfish reproduction.  When False no offspring are
    # produced even when eat_threshold is reached.  Useful for studying fixed-
    # population predation dynamics.
    sailfish_reproduction_enabled: bool = True

    # ═══════════════════════════════════════════════════════════════════
    # MOUSE PREDATOR
    # ═══════════════════════════════════════════════════════════════════

    # Master toggle for the mouse-as-predator feature.  When True, holding
    # the left mouse button makes sardines treat the cursor as a predator
    # using the same flee logic as real sailfish.  Useful for interactively
    # testing flee response and manually herding sardines into groups.
    # Toggle in the config menu (MOUSE PREDATOR section).
    mouse_predator_enabled: bool = True

    # Detection radius for the mouse predator (pixels).  Sardines within this
    # distance of the cursor apply a flee force when the mouse button is held.
    # Defaults to the same value as radius_flee for a consistent feel with
    # real sailfish.
    mouse_predator_radius: float = 100.0

    # Force multiplier for mouse-predator flee force, applied on top of the
    # base weight_flee value.  1.0 = same intensity as a real sailfish.
    # Increase for more dramatic scatter effects; decrease if the mouse
    # predator feels too aggressive relative to sailfish.
    mouse_predator_flee_weight: float = 1.0

    # ═══════════════════════════════════════════════════════════════════
    # HUD / OVERLAY
    # ═══════════════════════════════════════════════════════════════════

    # Whether the HUD (heads-up diagnostic display) is visible at startup.
    # Toggle at runtime with the H key.  The HUD shows per-state sailfish
    # counts, baitball metrics, population figures, and FPS.  It is
    # rendered at 4 Hz to reduce string-formatting overhead.
    show_hud: bool = True

    # ═══════════════════════════════════════════════════════════════════
    # DEBUG VISUALIZATION — BAITBALL / HERDING OVERLAY
    # ═══════════════════════════════════════════════════════════════════
    #
    # These toggles enable optional in-world overlays for diagnosing
    # sailfish baitball / herding behavior.  All are disabled by default
    # for performance.  Toggle individually via the DEBUG menu (TAB →
    # scroll to DEBUG VISUALIZATION section) or in boids_overrides.json.
    #
    # Performance note: all overlays read data already computed by the
    # simulation (baitball list, sailfish state vars) — no extra O(n²)
    # work is performed.  When every toggle is False the draw path adds
    # zero overhead (single branch check per frame).

    # Master switch for all baitball/herding debug visuals.  When False,
    # every sub-toggle below is also suppressed regardless of its value.
    # This lets you turn off the whole overlay without changing sub-settings.
    debug_baitballs_enabled: bool = False

    # Draw a translucent filled circle and center dot for each detected
    # baitball.  Opacity varies slightly with density score so denser
    # schools appear more vivid.  Requires debug_baitballs_enabled.
    debug_baitball_shading_enabled: bool = True

    # Show a small text label near each baitball center with sardine count,
    # density score, and baitball radius.  Requires debug_baitballs_enabled.
    # Has a minor text-rendering cost per baitball; keep off in large
    # populations unless actively tuning detection parameters.
    debug_baitball_labels_enabled: bool = True

    # Draw a line from each sailfish to the baitball it is currently
    # associated with (orbiting, herding, striking toward, or targeting
    # from orbit).  Line style varies by state:
    #   orbit/herd  — thin white-blue line
    #   strike-ready — brighter cyan line
    #   active strike — bright yellow line
    # Requires debug_baitballs_enabled.
    debug_sailfish_baitball_links_enabled: bool = True

    # Draw a small cross marker at each sailfish's computed orbit slot
    # (the target position the herding force steers toward).  Also draws a
    # short line from the sailfish to its slot.  Useful for diagnosing
    # conga-line orbiting and slot spacing.  Requires debug_baitballs_enabled.
    debug_sailfish_orbit_slots_enabled: bool = True

    # Label each sailfish with its current state machine state (small text
    # above the fish body).  States shown: PATROL, TRACK, ORBIT, LOCK,
    # BURST, STRIKE, RECOVER.  Has a text-rendering cost per sailfish;
    # disable in large fleets.  Does NOT require debug_baitballs_enabled —
    # state labels are useful independent of baitball visualization.
    debug_sailfish_state_labels_enabled: bool = False

    # During a center pass-through baitball strike, draw the planned strike
    # line from the sailfish to its computed exit point.  During an edge-
    # target strike (TARGET_LOCK launched from orbit), highlight the locked
    # sardine target.  Requires debug_baitballs_enabled.
    debug_sailfish_strike_paths_enabled: bool = True

    # ═══════════════════════════════════════════════════════════════════
    # PERFORMANCE / FRAME TIMING (optional diagnostics)
    # ═══════════════════════════════════════════════════════════════════

    # Enable per-subsystem frame-timing metrics on the HUD.
    # When True, the simulation measures milliseconds spent in:
    #   sardine force computation, baitball detection, sailfish update,
    #   algae update.
    # The cost of measurement itself is negligible (one perf_counter() call
    # per subsystem boundary), but the HUD rendering cost of additional text
    # lines is non-zero — disable in performance-critical sessions.
    # Toggle in DEBUG VISUALIZATION config section or via boids_overrides.json.
    debug_frame_timing_enabled: bool = False

    # How many frames to stagger baitball detection across.
    # Detection runs on frame N % baitball_detect_interval == 0; between
    # detection frames the cached result is reused.
    # • 1  = detect every frame (original behavior, highest quality)
    # • 3  = detect every 3rd frame (50 ms lag at 60 fps, imperceptible)
    # • 5  = detect every 5th frame (83 ms lag, light visual stutter possible)
    # Lower values favour quality; higher values favour performance.
    # Default 3 gives ~67% reduction in baitball detection cost.
    baitball_detect_interval: int = 3


# Module-level default instance — import this everywhere.
CONFIG = SimConfig()
