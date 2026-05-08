"""
sim/simulation.py
=================
Simulation controller — owns all agents and advances them each frame.

Responsibilities
----------------
* Spawn and store all Sardine (and later Sailfish) instances.
* Rebuild the spatial hash every frame.
* Call each agent's ``update(dt)`` method.
* Provide read-only access to agent lists for the renderer.

The Simulation intentionally knows nothing about rendering; it only
manages world state.
"""

from __future__ import annotations

import collections
import dataclasses
import math
from typing import TYPE_CHECKING

import numpy as np

from config.config import SimConfig
from sim.spatial_hash import SpatialHash

if TYPE_CHECKING:
    from sim.boid import Boid


class _MouseStimulus:
    """Shared mutable container for the mouse-as-predator stimulus.

    One instance lives on ``Simulation`` and is shared by reference across
    **all** sardines.  The simulation updates ``.active``, ``.x``, ``.y``
    when mouse events arrive; sardines read the same object in
    ``_compute_forces()`` — no back-reference to ``Simulation`` needed.

    ``.radius`` / ``.radius_sq`` are refreshed by ``apply_config()``
    whenever ``mouse_predator_radius`` changes so each sardine always sees
    the current detection radius without a per-frame config lookup.
    """

    __slots__ = ("active", "x", "y", "radius", "radius_sq", "flee_weight_mult")

    def __init__(self, radius: float, flee_weight_mult: float) -> None:
        self.active:           bool  = False
        self.x:                float = 0.0
        self.y:                float = 0.0
        self.radius:           float = radius
        self.radius_sq:        float = radius * radius
        self.flee_weight_mult: float = flee_weight_mult


@dataclasses.dataclass(frozen=True)
class _BaitballInfo:
    """Lightweight dense-school summary used by sailfish cooperation logic."""

    bid: int
    cx: float
    cy: float
    radius: float
    count: int
    density: float


class Simulation:
    """Top-level world state container.

    Parameters
    ----------
    config:
        Frozen simulation config shared across all modules.
    """

    def __init__(self, config: SimConfig) -> None:
        self.config: SimConfig = config
        self.sardines: list[Boid] = []
        self.sailfish: list[Boid] = []
        self.algae:    list[Boid] = []

        # Sardine-only spatial hash — queried by both sardines (flocking) and
        # sailfish (pursuit).  Sailfish are NOT inserted here so sardines
        # never treat them as flocking neighbours.
        self._grid = SpatialHash(
            cell_size=config.cell_size,
            world_width=float(config.window_width),
            world_height=float(config.window_height),
        )

        # Sailfish-only spatial hash — queried by sardines for predator
        # awareness.  Sardines are NOT inserted here.
        self._sailfish_grid = SpatialHash(
            cell_size=config.sailfish_cell_size,
            world_width=float(config.window_width),
            world_height=float(config.window_height),
        )

        # Algae-only spatial hash — queried by algae for flocking.
        # Sardines and sailfish are never inserted here.
        self._algae_grid = SpatialHash(
            cell_size=config.algae_cell_size,
            world_width=float(config.window_width),
            world_height=float(config.window_height),
        )

        # ── Mouse predator stimulus ───────────────────────────────────
        # Shared mutable object passed to every sardine at creation.
        # main.py updates .active / .x / .y via set_mouse_predator() /
        # clear_mouse_predator() in response to pyglet mouse events.
        self._mouse_stimulus = _MouseStimulus(
            radius=config.mouse_predator_radius,
            flee_weight_mult=config.mouse_predator_flee_weight,
        )

        self._spawn_sardines()
        self._spawn_sailfish()
        self._spawn_algae()

        # ── Ecology statistics ────────────────────────────────────────
        # Monotonically increasing totals; never reset.  Read by HUD/tests.
        self.stats_algae_eaten: int = 0
        self.stats_sardine_births: int = 0
        self.stats_sardines_eaten: int = 0

        # Set of id() values for algae objects eaten during the most recent
        # update() call.  Renderer.draw() reads this to delete the
        # corresponding Triangle shapes before syncing.  Cleared at the
        # START of each update() so tests can inspect it after update().
        self._algae_eaten_ids: set[int] = set()

        # List of newly born Sardine objects from the reproduction pass in the
        # most recent update() call.  Renderer.draw() reads this to allocate
        # new Triangle shapes for the offspring.  Cleared at the START of
        # each update() so tests can inspect it after update().
        self._sardine_born: list = []

        # Set of id() values for sardines eaten by sailfish during the most
        # recent update() call.  Renderer.draw() reads this to delete the
        # corresponding Triangle shapes before syncing.  Cleared at the
        # START of each update() so tests can inspect it after update().
        self._sardines_eaten_ids: set[int] = set()

        # Sardines that died of starvation this frame (Stage 8.5).
        # Kept separate from _sardines_eaten_ids so tests can distinguish cause.
        self._sardines_starved_ids: set[int] = set()

        # Sailfish that died of starvation this frame (Stage 8.7).
        self._sailfish_died_ids: set[int] = set()

        # List of newly born Sailfish objects from the reproduction pass in
        # the most recent update() call.  Renderer.draw() reads this to
        # allocate new Triangle shapes for the offspring.  Cleared at the
        # START of each update() so tests can inspect it after update().
        self._sailfish_born: list = []

        # Starvation death totals (Stage 8.5 / 8.7).
        self.stats_sardine_starvations: int = 0
        self.stats_sailfish_starvations: int = 0

        # Monotonically increasing count of sailfish born by reproduction.
        self.stats_sailfish_births: int = 0

        # ── Algae respawn side-channel ─────────────────────────────────
        # New Algae objects spawned by the respawn pass this frame.
        # Renderer.draw() reads this to allocate matching Triangle shapes.
        # Cleared at the START of each update() (mirrors _sardine_born).
        self._algae_born: list = []
        # Fractional accumulator for sub-frame respawn: avoids integer-
        # rounding bias when algae_respawn_rate is not a whole number.
        self._algae_spawn_accumulator: float = 0.0
        # Running total of algae created by respawn (not initial _spawn_algae).
        self.stats_algae_spawned: int = 0
        # Persistent entropy-seeded RNG for algae respawn positions — avoids
        # creating a new np.random.default_rng() on every _spawn_algae_batch()
        # call (which respawns algae several times per second).
        self._respawn_rng = np.random.default_rng()

        # ── Phase 4A: batched sardine state arrays ─────────────────────
        # Contiguous (N, 2) float64 arrays backing position, velocity, and
        # acceleration for all sardines.  Populated at the start of each
        # update() frame (after grid rebuild, when _gx/_gy/_gvx/_gvy are
        # fresh) and used by _batch_integrate_sardines() for vectorised
        # Euler integration, speed clamping, and boundary wrapping.
        #
        # Each sardine's _idx attribute is set to its row index here so
        # Phase 4C can address sardine rows directly during force computation.
        #
        # Capacity grows by 2× when more sardines exist than current cap.
        # Using (0,2) as the initial shape so the first update() call
        # triggers an allocation-grow before any sardines exist.
        self._s_pos: np.ndarray = np.zeros((0, 2), dtype=np.float64)
        self._s_vel: np.ndarray = np.zeros((0, 2), dtype=np.float64)
        self._s_acc: np.ndarray = np.zeros((0, 2), dtype=np.float64)
        self._s_n:   int        = 0  # number of live sardines in batch arrays
        # Reused integer index buffers for hot-path neighbour/owner gathers.
        self._sf_neigh_buf: np.ndarray = np.empty(64, dtype=np.int32)
        self._sf_owner_buf: np.ndarray = np.empty(64, dtype=np.int32)
        self._bb_neigh_buf: np.ndarray = np.empty(64, dtype=np.int32)

        # ── Optional WebGPU sardine force backend ─────────────────────
        # Initialized lazily on first use when sardine_use_gpu is True.
        # _gpu_init_tried prevents re-attempting after a failed init.
        # On success: _gpu_backend is a SardineGPUBackend instance.
        # On failure or wgpu missing: _gpu_backend stays None, CPU used.
        self._gpu_backend = None
        self._gpu_init_tried: bool = False

        # ── Population history (for 'd'-key graph overlay) ────────────
        # Sampled at ~1 Hz of simulated time; deque cap prevents unbounded
        # growth.  Each entry is (sim_time, n_sardines, n_sailfish, n_algae).
        self.sim_time: float = 0.0
        self._pop_history_accum: float = 0.0
        self.pop_history: collections.deque = collections.deque(maxlen=3600)
        self.pop_history.append(
            (0.0, len(self.sardines), len(self.sailfish), len(self.algae))
        )

        # ── Cooperative sailfish debug side-channel ───────────────────
        self._baitballs: list[_BaitballInfo] = []
        self.debug_baitball_count: int = 0
        self.debug_baitball_max_density: float = 0.0
        self.debug_herding_sailfish: int = 0
        self.debug_orbit_sailfish: int = 0
        self.debug_group_strike_cooldown: float = 0.0
        self.stats_center_strikes: int = 0
        self.stats_edge_strikes: int = 0
        self._baitball_group_strike_cooldown_timer: float = 0.0

        # ── Baitball detection throttle ───────────────────────────────
        # Running _detect_baitballs() every frame is expensive (~2ms at 1500
        # sardines).  Stagger detection to run once every N frames and reuse
        # the cached result in between.  Baitballs move slowly relative to
        # frame rate so a 3-frame stagger (50ms at 60fps) is imperceptible.
        # Initialize at interval-1 so detection fires on the very first frame.
        self._baitball_detect_counter: int = max(0, max(1, int(config.baitball_detect_interval)) - 1)
        self._baitball_detect_interval: int = max(1, int(config.baitball_detect_interval))

        # ── Per-subsystem timing (ms per frame, optional profiling HUD) ──
        # Updated every frame when debug_frame_timing_enabled is True.
        # Cost when disabled: one bool check per frame.
        self.dbg_ms_sardine_forces:    float = 0.0
        self.dbg_ms_baitball_detect:   float = 0.0
        self.dbg_ms_sailfish_update:   float = 0.0
        self.dbg_ms_algae_update:      float = 0.0
        self.dbg_ms_grid_rebuild:      float = 0.0

    # ── Initialisation ────────────────────────────────────────────────

    def _spawn_sardines(self) -> None:
        """Populate the world with Sardine boids at random positions and headings."""
        from sim.sardine import Sardine

        rng = np.random.default_rng(42)  # fixed seed for reproducible spawning
        W   = float(self.config.window_width)
        H   = float(self.config.window_height)
        n   = self.config.num_sardines

        angles = rng.uniform(0.0, 2.0 * np.pi, n)
        speed  = self.config.sardine_min_speed

        for i in range(n):
            x  = float(rng.uniform(0.0, W))
            y  = float(rng.uniform(0.0, H))
            vx = float(np.cos(angles[i])) * speed
            vy = float(np.sin(angles[i])) * speed
            self.sardines.append(
                Sardine(x, y, vx, vy, self.config, self._grid,
                        self._sailfish_grid, self._mouse_stimulus)
            )

    # ── Initialisation (continued) ────────────────────────────────────

    def _spawn_algae(self) -> None:
        """Populate the world with Algae boids at random positions."""
        from sim.algae import Algae

        rng = np.random.default_rng(256)  # distinct seed from sardines (42) and sailfish (123)
        W   = float(self.config.window_width)
        H   = float(self.config.window_height)
        n   = self.config.num_algae

        angles = rng.uniform(0.0, 2.0 * np.pi, n)
        # Spawn at 30 % of max speed so the first frame is visually calm.
        speed  = self.config.algae_max_speed * 0.3

        for i in range(n):
            x  = float(rng.uniform(0.0, W))
            y  = float(rng.uniform(0.0, H))
            vx = float(np.cos(angles[i])) * speed
            vy = float(np.sin(angles[i])) * speed
            self.algae.append(
                Algae(x, y, vx, vy, self.config, self._algae_grid)
            )

    def _spawn_sailfish(self) -> None:
        """Populate the world with Sailfish predators at random positions."""
        from sim.sailfish import Sailfish

        rng = np.random.default_rng(123)  # different seed from sardines (42)
        W   = float(self.config.window_width)
        H   = float(self.config.window_height)
        n   = self.config.num_sailfish

        angles = rng.uniform(0.0, 2.0 * np.pi, n)
        speed  = self.config.sailfish_min_speed

        for i in range(n):
            x  = float(rng.uniform(0.0, W))
            y  = float(rng.uniform(0.0, H))
            vx = float(np.cos(angles[i])) * speed
            vy = float(np.sin(angles[i])) * speed
            # Sailfish receive the sardine grid (query prey) and the sailfish
            # grid (sailfish-to-sailfish flocking).  They are never inserted
            # into the sardine grid.
            self.sailfish.append(
                Sailfish(x, y, vx, vy, self.config, self._grid,
                         sailfish_grid=self._sailfish_grid)
            )

    # ── Update loop ───────────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """Advance the simulation by *dt* seconds.

        Called once per rendered frame by main.py / pyglet scheduler.

        Steps
        -----
        1. Rebuild the sardine-only spatial hash from current sardine positions.
           Sailfish are intentionally excluded so sardines treat them as
           obstacles, not flocking neighbours.
        2. Let each agent query neighbours and compute forces.
        3. Integrate each agent's state.
        """
        # Guard against frame-drop spikes: Euler integration diverges if dt
        # is too large (e.g. first frame, system sleep, window drag).
        dt = min(dt, 1 / 20)  # cap at 50 ms ≈ 3 dropped frames

        # Accumulate simulated time and sample population history at ~1 Hz.
        self.sim_time += dt
        self._pop_history_accum += dt
        if self._pop_history_accum >= 1.0:
            self._pop_history_accum -= 1.0
            self.pop_history.append(
                (self.sim_time, len(self.sardines), len(self.sailfish), len(self.algae))
            )

        # Clear eaten IDs from the previous frame so stale entries never
        # accumulate and the renderer always sees only this frame's removals.
        self._algae_eaten_ids.clear()
        self._sardine_born.clear()
        self._algae_born.clear()
        self._sardines_eaten_ids.clear()
        self._sardines_starved_ids.clear()
        self._sailfish_died_ids.clear()
        self._sailfish_born.clear()

        # 1 — rebuild sardine grid (sardines only)
        # Bind .insert as a local so the inner loop avoids two LOAD_ATTR
        # operations (self._grid + .insert) on every iteration.
        _grid = self._grid
        _grid.clear()
        _grid_insert = _grid.insert
        for s in self.sardines:
            _grid_insert(s)

        # 1b — rebuild sailfish grid (sailfish only)
        _sf_grid = self._sailfish_grid
        _sf_grid.clear()
        _sf_insert = _sf_grid.insert
        for sf in self.sailfish:
            _sf_insert(sf)

        # 1c — rebuild algae grid (algae only)
        _algae_grid = self._algae_grid
        _algae_grid.clear()
        _algae_insert = _algae_grid.insert
        for a in self.algae:
            _algae_insert(a)

        # Dense-school detector for cooperative sailfish logic.
        # Throttled: recompute only every _baitball_detect_interval frames to
        # avoid running the expensive O(cells × neighbors) detection every frame.
        # Between detection frames the cached _baitballs list is reused as-is.
        _timing_enabled = self.config.debug_frame_timing_enabled
        if _timing_enabled:
            import time as _time
            _t_bb = _time.perf_counter()
        self._baitball_detect_counter += 1
        if self._baitball_detect_counter >= self._baitball_detect_interval:
            self._baitball_detect_counter = 0
            self._baitballs = self._detect_baitballs()
        if _timing_enabled:
            self.dbg_ms_baitball_detect = (_time.perf_counter() - _t_bb) * 1000.0
        self.debug_baitball_count = len(self._baitballs)
        if self._baitballs:
            self.debug_baitball_max_density = max(b.density for b in self._baitballs)
        else:
            self.debug_baitball_max_density = 0.0

        if self._baitball_group_strike_cooldown_timer > 0.0:
            self._baitball_group_strike_cooldown_timer = max(
                0.0,
                self._baitball_group_strike_cooldown_timer - dt,
            )
        self.debug_group_strike_cooldown = self._baitball_group_strike_cooldown_timer

        # Feed this frame's baitball snapshots to each sailfish.
        # Count active baitball attackers from the previous frame so sailfish
        # can gate new strikes via the proto-collaboration limit.
        sf_count = len(self.sailfish)
        active_bb_strikes = sum(
            1 for sf in self.sailfish
            if sf._is_baitball_attacker
            and sf.state in ("baitball_strike", "target_lock", "burst_attack")
        )
        for sf in self.sailfish:
            sf.set_baitball_context(
                self._baitballs,
                sf_count,
                self._baitball_group_strike_cooldown_timer,
                active_bb_strikes,
            )

        # 2 — Phase 4A: populate sardine batch arrays and assign indices.
        # _gx/_gy/_gvx/_gvy are already Python floats set by insert() above —
        # no extra numpy reads.  _idx is needed by Phase 4C to address rows.
        n = len(self.sardines)
        self._s_n = n
        if n > 0:
            self._ensure_sardine_capacity(n)
            _sp = self._s_pos
            _sv = self._s_vel
            for i, s in enumerate(self.sardines):
                s._idx = i
                _sp[i, 0] = s._gx   # pre-cached Python float from insert()
                _sp[i, 1] = s._gy
                _sv[i, 0] = s._gvx  # pre-cached Python float from insert()
                _sv[i, 1] = s._gvy

        # 3 — compute forces for all agents.
        # Sardines: Phase 4C batch computation directly into _s_acc.
        # Sailfish and algae: full update() unchanged (not batched yet).
        if n > 0:
            if _timing_enabled:
                _t_sf_forces = _time.perf_counter()
            self._batch_compute_sardine_forces()
            if _timing_enabled:
                self.dbg_ms_sardine_forces = (_time.perf_counter() - _t_sf_forces) * 1000.0
        if _timing_enabled:
            _t_sailfish = _time.perf_counter()
        for agent in self.sailfish:
            agent.update(dt)
        # Arm group strike cooldown after all sailfish have made decisions.
        # This keeps edge/center strike launches visibly staggered.
        group_triggered = False
        for sf in self.sailfish:
            if sf._pending_group_strike_trigger:
                group_triggered = True
                if sf._pending_group_strike_kind == "center":
                    self.stats_center_strikes += 1
                elif sf._pending_group_strike_kind == "edge":
                    self.stats_edge_strikes += 1
                sf._pending_group_strike_trigger = False
                sf._pending_group_strike_kind = ""
        if group_triggered:
            self._baitball_group_strike_cooldown_timer = max(
                self._baitball_group_strike_cooldown_timer,
                self.config.sailfish_baitball_strike_group_cooldown_seconds,
            )
            self.debug_group_strike_cooldown = self._baitball_group_strike_cooldown_timer

        if _timing_enabled:
            self.dbg_ms_sailfish_update = (_time.perf_counter() - _t_sailfish) * 1000.0
            _t_algae = _time.perf_counter()
        for agent in self.algae:
            agent.update(dt)
        if _timing_enabled:
            self.dbg_ms_algae_update = (_time.perf_counter() - _t_algae) * 1000.0

        # Lightweight debug count for HUD: how many sailfish are currently
        # applying cooperative herding pressure.
        self.debug_herding_sailfish = sum(1 for sf in self.sailfish if sf._is_herding)
        self.debug_orbit_sailfish = sum(1 for sf in self.sailfish if sf.state == "orbit_baitball")

        # 4 — Phase 4B/4C: batch integrate all sardines.
        # _s_acc is already populated by _batch_compute_sardine_forces() above.
        # _batch_integrate_sardines() integrates vel, clamps speed, integrates
        # pos, wraps/clamps boundary, scatters back to sardine objects.
        if n > 0:
            self._batch_integrate_sardines(dt)

        # ── Ecology pass: sardine–algae eating ───────────────────────
        # Runs AFTER all agent.update() calls to avoid mutating lists mid-
        # iteration.  Uses the _algae_grid built at the start of this frame
        # (algae positions lag by one frame — negligible at algae speeds).
        #
        # Each sardine earns food_counter credit for every algae it touches
        # this frame; the same algae may be touched by multiple sardines
        # (both get credit — benign double-count for now) but is only
        # removed once (set deduplication).  hunger_timer is reset to 0.0
        # when any algae is eaten; the timer continues for sardines that
        # didn't eat this frame.
        if self.sardines:
            # 1 — increment hunger timer for every sardine regardless of eating.
            for sardine in self.sardines:
                sardine.hunger_timer += dt
                sardine.age += dt  # increment age for all sardines (used in Stage 8.6 for max age death)

            # 2 — sardine-algae contact eating.
            if self.algae:
                eat_r = self.config.algae_eat_radius
                to_eat: set = set()
                for sardine in self.sardines:
                    for alga in self._algae_grid.query(sardine, eat_r):
                        to_eat.add(alga)
                        sardine.food_counter += 1
                        sardine.hunger_timer = 0.0

                if to_eat:
                    self._algae_eaten_ids.update(map(id, to_eat))
                    self.algae = [a for a in self.algae if a not in to_eat]
                    self.stats_algae_eaten += len(to_eat)

            # 3 — sardine starvation.
            # Runs after eating so a sardine that just ate (hunger_timer=0)
            # can never starve the same frame.  Runs before reproduction so
            # dying sardines do not first produce an offspring.
            # starvation_time=0.0 acts as a disable flag (tests / tuning).
            starvation_time = self.config.sardine_starvation_time
            if starvation_time > 0.0:
                starving: set = {
                    s for s in self.sardines if s.hunger_timer > starvation_time
                }
                if starving:
                    self._sardines_starved_ids.update(map(id, starving))
                    for _s in starving:
                        _s.is_alive = False   # notify sailfish of target death
                    self.sardines = [s for s in self.sardines if s not in starving]
                    self.stats_sardine_starvations += len(starving)

            # 4 — sardine reproduction.
            # Runs after starvation so dying sardines don't also reproduce.
            # threshold=0 disables reproduction (tests/debugging).
            threshold = self.config.sardine_eat_threshold
            if threshold > 0 and self.sardines:
                self._reproduce_sardines(threshold)

        # ── Ecology pass: sailfish hunger timer + eating + starvation ────────
        # Sailfish hunger timer was deferred from Stage 8.6; incremented here
        # (Stage 8.7) before the eating pass so a sailfish that eats this
        # frame has its timer reset AFTER the increment (net = reset to 0).
        if self.sailfish:
            for sf in self.sailfish:
                sf.hunger_timer += dt
                sf.age += dt
                # Decrement individual-target lock timer (counts down to 0, then lock releases).
                if sf._target_lock_timer > 0.0:
                    sf._target_lock_timer = max(0.0, sf._target_lock_timer - dt)
                # Per-fish baitball strike cooldown (proto-collaboration).
                if sf._individual_strike_cooldown > 0.0:
                    sf._individual_strike_cooldown = max(0.0, sf._individual_strike_cooldown - dt)
                # Clear attacker flag when individual cooldown expires (or fish
                # returns to non-attack state).
                if sf._individual_strike_cooldown <= 0.0 and sf.state not in (
                    "baitball_strike", "target_lock", "burst_attack"
                ):
                    sf._is_baitball_attacker = False
                # Burst timers: count down burst; start cooldown + enter RECOVER
                # when burst expires.
                if sf._burst_timer > 0.0:
                    sf._burst_timer = max(0.0, sf._burst_timer - dt)
                    if sf._burst_timer == 0.0:
                        sf._burst_cooldown_timer = sf._config.sailfish_burst_cooldown
                        # State machine: BURST_ATTACK → RECOVER.
                        if sf.state == "burst_attack":
                            sf.sm_to_recover()
                            sf._recovery_timer = sf._config.sailfish_recovery_duration
                elif sf._burst_cooldown_timer > 0.0:
                    sf._burst_cooldown_timer = max(0.0, sf._burst_cooldown_timer - dt)

                # Baitball strike timers: count down strike runtime and move to
                # RECOVER when complete; maintain separate cooldown.
                if sf._baitball_strike_timer > 0.0:
                    sf._baitball_strike_timer = max(0.0, sf._baitball_strike_timer - dt)
                    if sf._baitball_strike_timer == 0.0:
                        sf._baitball_strike_cooldown_timer = (
                            sf._config.sailfish_baitball_strike_cooldown_seconds
                        )
                        if sf.state == "baitball_strike":
                            sf.sm_to_recover()
                            sf._recovery_timer = sf._config.sailfish_recovery_duration
                elif sf._baitball_strike_cooldown_timer > 0.0:
                    sf._baitball_strike_cooldown_timer = max(
                        0.0, sf._baitball_strike_cooldown_timer - dt
                    )

                # Recovery timer: counts down while in RECOVER; when it reaches
                # 0 pick the next state (TARGET_LOCK if prev target valid, else
                # TRACK_SCHOOL; TRACK_SCHOOL will time out to PATROL on its own).
                if sf._recovery_timer > 0.0:
                    sf._recovery_timer = max(0.0, sf._recovery_timer - dt)
                    if sf._recovery_timer == 0.0 and sf.state == "recover":
                        tgt = sf._target_sardine
                        if sf._baitball is not None and sf._config.sailfish_herding_enabled:
                            sf.sm_to_orbit_baitball()
                            sf._arm_orbit_patience()
                        elif tgt is not None and tgt.is_alive:
                            sf.sm_to_target_lock()
                            sf._target_lock_timer = sf._config.sailfish_target_lock_time
                        else:
                            sf.sm_to_track_school()

                # No-sardines timeout: TRACK_SCHOOL → PATROL when sardines have
                # been absent for too long.
                if (
                    sf.state == "track_school"
                    and sf._no_sardines_timer >= sf._config.sailfish_no_sardines_timeout
                ):
                    sf._no_sardines_timer = 0.0
                    sf.sm_to_patrol()

        # Sailfish-sardine eating: one kill per sailfish per tick.
        # Runs after sardine reproduction so newly born sardines are not in
        # _grid (built at frame start) and cannot be immediately eaten.
        if self.sailfish and self.sardines:
            eat_r = self.config.sailfish_eat_radius
            to_remove: set = set()
            for sf in self.sailfish:
                prey = self._grid.query_first(sf, eat_r)  # one kill per sailfish per tick
                if prey is not None:
                    to_remove.add(prey)
                    sf.food_counter += 1
                    sf.hunger_timer = 0.0
            if to_remove:
                self._sardines_eaten_ids.update(map(id, to_remove))
                for _s in to_remove:
                    _s.is_alive = False   # notify sailfish of target death
                self.sardines = [s for s in self.sardines if s not in to_remove]
                self.stats_sardines_eaten += len(to_remove)

        # Sailfish starvation: runs after eating so a sailfish that just killed
        # (hunger_timer=0) can never starve the same frame.
        # starvation_time=0.0 or sailfish_starvation_enabled=False disables.
        if self.sailfish:
            starvation_time = self.config.sailfish_starvation_time
            if self.config.sailfish_starvation_enabled and starvation_time > 0.0:
                dying: set = {
                    sf for sf in self.sailfish if sf.hunger_timer > starvation_time
                }
                if dying:
                    self._sailfish_died_ids.update(map(id, dying))
                    self.sailfish = [sf for sf in self.sailfish if sf not in dying]
                    self.stats_sailfish_starvations += len(dying)

        # Sailfish reproduction: runs after starvation so dying sailfish
        # don't also reproduce.  threshold=0 or sailfish_reproduction_enabled=False
        # disables reproduction.
        threshold = self.config.sailfish_eat_threshold
        if self.config.sailfish_reproduction_enabled and threshold > 0 and self.sailfish:
            self._reproduce_sailfish(threshold)

        # ── Algae respawn ─────────────────────────────────────────────
        # Fractional-accumulator pattern: algae_respawn_rate new algae per
        # simulated second, spawned in whole-unit batches.  Pauses when the
        # pool reaches algae_max_count.  algae_respawn_rate=0.0 disables.
        respawn_rate = self.config.algae_respawn_rate
        max_algae    = self.config.algae_max_count
        if respawn_rate > 0.0 and len(self.algae) < max_algae:
            self._algae_spawn_accumulator += respawn_rate * dt
            n_spawn = int(self._algae_spawn_accumulator)
            if n_spawn > 0:
                self._algae_spawn_accumulator -= n_spawn
                n_spawn = min(n_spawn, max_algae - len(self.algae))
                if n_spawn > 0:
                    self._spawn_algae_batch(n_spawn)

    # ── Dynamic population management ─────────────────────────────────

    def _reproduce_sardines(self, threshold: int) -> None:
        """Spawn offspring for every sardine whose food_counter >= threshold.

        Called from update() after the eating pass.  Appends new Sardine
        objects both to ``self.sardines`` and ``self._sardine_born`` (which
        the renderer reads to allocate matching Triangle shapes).

        Offspring placement
        -------------------
        The offspring is placed one separation-radius away from the parent,
        offset perpendicular to the parent's current heading.  This:

        * keeps the pair within each other's flocking radius immediately
        * avoids stacking two fish on top of each other (which would cause
          a spike in the separation force)
        * never places the offspring outside the world — the position is
          toroidally wrapped

        The offspring inherits the parent's velocity so it immediately
        integrates into the school on the next frame.

        food_counter reset
        ------------------
        ``food_counter`` is decremented by *threshold* rather than reset to 0.
        Any excess (e.g. if the sardine ate 7 algae at threshold=5, the
        counter becomes 2) carries forward toward the next birth — biologically
        natural and avoids discarding partial progress.
        """
        from sim.sardine import Sardine
        import math as _math

        cfg      = self.config
        W        = float(cfg.window_width)
        H        = float(cfg.window_height)
        offset_d = cfg.radius_separation  # push offspring 1 sep-radius away

        # Snapshot current list length so newly appended offspring are not
        # iterated in this same loop (avoid runaway spawning in one frame).
        max_pop   = cfg.sardine_max_count
        n_current = len(self.sardines)
        for sardine in self.sardines[:n_current]:
            if sardine.food_counter >= threshold:
                sardine.food_counter -= threshold

                # Soft cap: consume the threshold (food_counter decremented
                # above) but skip spawning when the population is at or above
                # the configured ceiling.  Mirrors algae_max_count behaviour.
                if max_pop > 0 and len(self.sardines) >= max_pop:
                    continue

                # Perpendicular offset based on current heading.
                vx = float(sardine.velocity[0])
                vy = float(sardine.velocity[1])
                mag = _math.sqrt(vx * vx + vy * vy)
                if mag > 1e-9:
                    # Perpendicular unit vector: rotate heading 90°.
                    px, py = -vy / mag, vx / mag
                else:
                    px, py = 1.0, 0.0  # no heading — offset in +x

                ox = (float(sardine.position[0]) + px * offset_d) % W
                oy = (float(sardine.position[1]) + py * offset_d) % H

                offspring = Sardine(
                    ox, oy, vx, vy,
                    cfg, self._grid, self._sailfish_grid,
                    self._mouse_stimulus,
                )
                self.sardines.append(offspring)
                self._sardine_born.append(offspring)
                self.stats_sardine_births += 1

    def _reproduce_sailfish(self, threshold: int) -> None:
        """Spawn offspring for every sailfish whose food_counter >= threshold.

        Called from update() after the starvation pass.  Appends new Sailfish
        objects both to ``self.sailfish`` and ``self._sailfish_born`` (which
        the renderer reads to allocate matching Triangle shapes).

        Mirrors ``_reproduce_sardines`` exactly: perpendicular offset by one
        separation radius, inherited velocity, food_counter decremented by
        threshold (excess carries forward).
        """
        from sim.sailfish import Sailfish
        import math as _math

        cfg      = self.config
        W        = float(cfg.window_width)
        H        = float(cfg.window_height)
        offset_d = cfg.radius_separation

        # Snapshot current list length so newly appended offspring are not
        # iterated in this same loop (avoid runaway spawning in one frame).
        max_pop   = cfg.sailfish_max_count
        n_current = len(self.sailfish)
        for sailfish in self.sailfish[:n_current]:
            if sailfish.food_counter >= threshold:
                sailfish.food_counter -= threshold

                # Soft cap: consume the threshold but skip spawning when the
                # population is at or above the configured ceiling.
                # Mirrors sardine_max_count / algae_max_count behaviour.
                if max_pop > 0 and len(self.sailfish) >= max_pop:
                    continue

                # Perpendicular offset based on current heading.
                vx = float(sailfish.velocity[0])
                vy = float(sailfish.velocity[1])
                mag = _math.sqrt(vx * vx + vy * vy)
                if mag > 1e-9:
                    px, py = -vy / mag, vx / mag
                else:
                    px, py = 1.0, 0.0  # no heading — offset in +x

                ox = (float(sailfish.position[0]) + px * offset_d) % W
                oy = (float(sailfish.position[1]) + py * offset_d) % H

                offspring = Sailfish(
                    ox, oy, vx, vy,
                    cfg, self._grid,
                    sailfish_grid=self._sailfish_grid,
                )
                self.sailfish.append(offspring)
                self._sailfish_born.append(offspring)
                self.stats_sailfish_births += 1

    def _spawn_algae_batch(self, n: int) -> None:
        """Spawn *n* new Algae objects via the respawn pass.

        Positions and headings are randomly chosen across the full world;
        the RNG is entropy-seeded (not fixed) because respawn positions are
        purely visual and don't need to be reproducible.  New Algae are
        appended to both ``self.algae`` and ``self._algae_born`` so the
        renderer can allocate Triangle shapes for them.
        """
        from sim.algae import Algae
        import math as _math

        rng    = self._respawn_rng  # reuse persistent RNG — avoids object creation each call
        W      = float(self.config.window_width)
        H      = float(self.config.window_height)
        speed  = self.config.algae_max_speed * 0.3
        xs     = rng.uniform(0.0, W, n)
        ys     = rng.uniform(0.0, H, n)
        angles = rng.uniform(0.0, 2.0 * _math.pi, n)
        for x, y, angle in zip(xs, ys, angles):
            vx   = float(_math.cos(angle)) * speed
            vy   = float(_math.sin(angle)) * speed
            alga = Algae(float(x), float(y), vx, vy, self.config, self._algae_grid)
            self.algae.append(alga)
            self._algae_born.append(alga)
        self.stats_algae_spawned += n

    # ── Batch sardine integration (Phase 4A/4B) ───────────────────────

    def _ensure_sardine_capacity(self, n: int) -> None:
        """Grow batch arrays to hold at least *n* sardines.

        Uses a 2× growth policy to amortise allocations across many small
        birth events.  Called once per frame (before array-populate) only
        when the capacity needs to increase.
        """
        if n > self._s_pos.shape[0]:
            cap = max(n, n * 2)  # at least 2× current need
            self._s_pos = np.zeros((cap, 2), dtype=np.float64)
            self._s_vel = np.zeros((cap, 2), dtype=np.float64)
            self._s_acc = np.zeros((cap, 2), dtype=np.float64)

    def _batch_compute_sardine_forces(self) -> None:
        """Phase 4C: compute all sardine forces via numpy cell-bucket vectorisation.

        Replaces the per-sardine Python loop::

            for s in self.sardines:
                s._compute_forces()

        Writes directly into ``_s_acc[:_s_n]`` so the gather loop in
        ``_batch_integrate_sardines()`` is no longer needed.

        Algorithm
        ---------
        1. **Flee** — vectorised N×S sardine-vs-sailfish pairwise computation.
        2. **Mouse flee** — vectorised against the single cursor point.
        3. **Cell-bucket flocking** — for each occupied sardine-grid cell, gather
           the (2·span+1)² neighbourhood, compute (M×K) pairwise forces via numpy
           broadcasting, accumulate sep / ali / coh into ``_s_acc``.
        4. **Noise** — one ``np.random.normal()`` call for all N sardines.

        Correctness guarantees
        ----------------------
        * Self-pair exclusion enforced by comparing owner vs neighbour indices.
        * Toroidal delta correction matches the per-sardine implementation.
        * Cohesion suppressed for sardines identified as fleeing (same rule as
          the per-sardine ``_compute_forces()``).
        * Each sardine is an *owner* in exactly one cell — no double-counting.
        * Fast-path / slow-path cell deduplication mirrors ``query()``.
        """
        n = self._s_n
        if n == 0:
            return

        sa = self._s_acc[:n]
        sa[:] = 0.0

        # Small-population fast path: per-sardine Python loop has lower per-call
        # overhead than the numpy cell-bucket ops when N is small (numpy function
        # dispatch cost dominates for (M=2, K=18)-scale arrays).
        # Crossover measured at ~500 sardines on Apple Silicon.
        if n < 500:
            for i, s in enumerate(self.sardines):
                s._compute_forces()
                acc      = s.acceleration
                sa[i, 0] = acc[0]
                sa[i, 1] = acc[1]
            return

        cfg = self.config

        # ── Optional WebGPU compute path (passes 1, 1b, 2) ───────────────────
        # Lazy-initialize on first frame that needs the GPU.  On failure the
        # backend is set to None and we fall through to the CPU path below.
        if cfg.sardine_use_gpu:
            if not self._gpu_init_tried:
                self._gpu_init_tried = True
                from sim.sardine_gpu import try_create_backend  # noqa: PLC0415
                self._gpu_backend = try_create_backend()
            if self._gpu_backend is not None:
                try:
                    gpu_acc = self._gpu_backend.compute_forces(
                        n, self._s_pos, self._s_vel,
                        self.sailfish, self._mouse_stimulus, cfg,
                    )
                    sa[:] = gpu_acc
                    # Pass 3: noise — kept on CPU (WGSL PRNG not worth complexity).
                    w_noise = cfg.weight_noise
                    if w_noise != 0.0:
                        sp = self._s_pos[:n]
                        sv = self._s_vel[:n]
                        max_speed = float(cfg.sardine_max_speed)
                        max_force = float(cfg.sardine_max_force)
                        _EPS  = 1e-9
                        noise = np.random.normal(0.0, 1.0, (n, 2))
                        mag_n = np.hypot(noise[:, 0], noise[:, 1])
                        valid_n = mag_n > _EPS
                        if valid_n.any():
                            inv_mag_n = np.where(valid_n, 1.0 / np.maximum(mag_n, _EPS), 0.0)
                            des_x = noise[:, 0] * inv_mag_n * max_speed
                            des_y = noise[:, 1] * inv_mag_n * max_speed
                            stx = des_x - sv[:, 0]
                            sty = des_y - sv[:, 1]
                            stm = np.hypot(stx, sty)
                            over_n = valid_n & (stm > max_force)
                            scale_n = np.where(over_n, max_force / np.maximum(stm, _EPS), 1.0)
                            sa[:, 0] += stx * scale_n * w_noise * valid_n
                            sa[:, 1] += sty * scale_n * w_noise * valid_n
                    return
                except Exception as _gpu_exc:  # noqa: BLE001
                    import logging as _log  # noqa: PLC0415
                    _log.getLogger(__name__).warning(
                        "sardine_gpu: frame error (%s) — disabling GPU path.", _gpu_exc
                    )
                    self._gpu_backend = None
                    sa[:] = 0.0   # reset before CPU fallback
        W   = float(cfg.window_width)
        H   = float(cfg.window_height)
        hw  = W * 0.5
        hh  = H * 0.5

        sp = self._s_pos[:n]   # (N, 2) positions  — read only
        sv = self._s_vel[:n]   # (N, 2) velocities — read only
        # sa was already zeroed above; re-bind the slice for use in pass 2/3.
        sa = self._s_acc[:n]

        max_speed = float(cfg.sardine_max_speed)
        max_force = float(cfg.sardine_max_force)
        _EPS  = 1e-9
        _EPS2 = 1e-18

        # All sardines share the same config — read cached params from [0].
        s0       = self.sardines[0]
        sep_r2   = s0._sep_r2
        ali_r2   = s0._ali_r2
        coh_r2   = s0._coh_r2
        flee_r2  = s0._flee_r2
        coh_span = s0._coh_span

        # ─── Pass 1: Flee — vectorised sardine × sailfish ─────────────────────
        is_fleeing = np.zeros(n, dtype=bool)

        if self.sailfish and cfg.radius_flee > 0.0 and cfg.weight_flee > 0.0:
            sf_x = np.array([sf._gx for sf in self.sailfish], dtype=np.float64)
            sf_y = np.array([sf._gy for sf in self.sailfish], dtype=np.float64)
            # (N, S) pairwise deltas via broadcasting.
            dx_sf = sp[:, 0:1] - sf_x   # (N, S)
            dy_sf = sp[:, 1:2] - sf_y   # (N, S)
            # Toroidal correction (in-place to reduce temporary arrays).
            _m = dx_sf > hw
            if _m.any():
                dx_sf[_m] -= W
            _m = dx_sf < -hw
            if _m.any():
                dx_sf[_m] += W
            _m = dy_sf > hh
            if _m.any():
                dy_sf[_m] -= H
            _m = dy_sf < -hh
            if _m.any():
                dy_sf[_m] += H
            d2_sf = dx_sf * dx_sf + dy_sf * dy_sf   # (N, S)
            in_flee = (d2_sf <= flee_r2) & (d2_sf > _EPS2)
            if in_flee.any():
                inv_d_sf = np.where(in_flee, 1.0 / np.sqrt(np.maximum(d2_sf, _EPS2)), 0.0)
                fvx = (dx_sf * inv_d_sf).sum(axis=1)   # (N,) sum of unit away-vecs
                fvy = (dy_sf * inv_d_sf).sum(axis=1)
                fm2 = fvx * fvx + fvy * fvy
                fleet_mask = fm2 > _EPS2
                is_fleeing |= fleet_mask
                if fleet_mask.any():
                    fm = np.sqrt(np.maximum(fm2, _EPS2))
                    des_x = fvx / fm * max_speed
                    des_y = fvy / fm * max_speed
                    stx = des_x - sv[:, 0]
                    sty = des_y - sv[:, 1]
                    stm = np.hypot(stx, sty)
                    over = fleet_mask & (stm > max_force)
                    scale = np.where(over, max_force / np.maximum(stm, _EPS), 1.0)
                    w = cfg.weight_flee
                    sa[:, 0] += stx * scale * w * fleet_mask
                    sa[:, 1] += sty * scale * w * fleet_mask

        # ─── Pass 1b: Mouse predator flee ──────────────────────────────────────
        ms = self._mouse_stimulus
        if ms is not None and ms.active and cfg.mouse_predator_enabled:
            mdx = sp[:, 0] - ms.x   # (N,)
            mdy = sp[:, 1] - ms.y
            _m = mdx > hw
            if _m.any():
                mdx[_m] -= W
            _m = mdx < -hw
            if _m.any():
                mdx[_m] += W
            _m = mdy > hh
            if _m.any():
                mdy[_m] -= H
            _m = mdy < -hh
            if _m.any():
                mdy[_m] += H
            md2 = mdx * mdx + mdy * mdy
            in_mouse = (md2 > _EPS2) & (md2 < ms.radius_sq)
            if in_mouse.any():
                is_fleeing |= in_mouse
                inv_md = np.where(in_mouse, 1.0 / np.sqrt(np.maximum(md2, _EPS2)), 0.0)
                fvx_m = mdx * inv_md
                fvy_m = mdy * inv_md
                fm_m = np.hypot(fvx_m, fvy_m)
                valid_m = in_mouse & (fm_m > _EPS)
                if valid_m.any():
                    des_x = np.where(valid_m, fvx_m / np.maximum(fm_m, _EPS) * max_speed, 0.0)
                    des_y = np.where(valid_m, fvy_m / np.maximum(fm_m, _EPS) * max_speed, 0.0)
                    stx = np.where(valid_m, des_x - sv[:, 0], 0.0)
                    sty = np.where(valid_m, des_y - sv[:, 1], 0.0)
                    stm = np.hypot(stx, sty)
                    over_m = valid_m & (stm > max_force)
                    scale_m = np.where(over_m, max_force / np.maximum(stm, _EPS), 1.0)
                    flee_w = cfg.weight_flee * ms.flee_weight_mult
                    sa[:, 0] += stx * scale_m * flee_w
                    sa[:, 1] += sty * scale_m * flee_w

        # ─── Pass 2: Cell-bucket flocking (sep + ali + coh) ───────────────────
        # For each occupied grid cell, gather sardine indices from the surrounding
        # (2·coh_span+1)² neighbourhood, then compute all pairwise forces for the
        # M *owner* sardines in that cell via numpy broadcasting over the K
        # *neighbour* sardines — avoiding any per-sardine Python inner loop.
        grid  = self._grid
        cells = grid._cells
        nc    = grid._num_cols
        nr    = grid._num_rows

        w_sep = cfg.weight_separation
        w_ali = cfg.weight_alignment
        w_coh = cfg.weight_cohesion

        # Pre-allocated neighbor index buffer — reused across cells to avoid
        # per-cell list allocations and np.array() conversions.
        if self._sf_neigh_buf.shape[0] < n:
            self._sf_neigh_buf = np.empty(max(n * 2, 64), dtype=np.int32)
        if self._sf_owner_buf.shape[0] < n:
            self._sf_owner_buf = np.empty(max(n * 2, 64), dtype=np.int32)
        neigh_buf = self._sf_neigh_buf
        owner_buf = self._sf_owner_buf
        fast_flock = nc > 2 * coh_span and nr > 2 * coh_span
        cells_get = cells.get

        for (col_c, row_c), owner_cell in cells.items():
            M = len(owner_cell)
            own_idxs = owner_buf[:M]
            for i, s in enumerate(owner_cell):
                own_idxs[i] = s._idx

            # Gather neighbour indices into the reusable buffer.
            # Fast path (large world): all cell keys are distinct after modular
            # reduction — mirrors the guarantee in SpatialHash.query().
            nb_n = 0
            if fast_flock:
                for dc in range(-coh_span, coh_span + 1):
                    col = (col_c + dc) % nc
                    for dr in range(-coh_span, coh_span + 1):
                        ncell = cells_get((col, (row_c + dr) % nr))
                        if ncell is not None:
                            for nb in ncell:
                                neigh_buf[nb_n] = nb._idx
                                nb_n += 1
            else:
                # Slow path: only hit in tiny worlds / unit-test grids.
                _vis: set = set()
                for dc in range(-coh_span, coh_span + 1):
                    col = (col_c + dc) % nc
                    for dr in range(-coh_span, coh_span + 1):
                        key = (col, (row_c + dr) % nr)
                        if key in _vis:
                            continue
                        _vis.add(key)
                        ncell = cells_get(key)
                        if ncell is not None:
                            for nb in ncell:
                                neigh_buf[nb_n] = nb._idx
                                nb_n += 1

            if nb_n == 0:
                continue

            neigh_idx = neigh_buf[:nb_n]    # view — no allocation
            P_own   = sp[own_idxs]    # (M, 2)
            P_neigh = sp[neigh_idx]   # (K, 2)
            V_neigh = sv[neigh_idx]   # (K, 2)
            V_own   = sv[own_idxs]    # (M, 2)

            # Pairwise deltas — (M, K) via broadcast; (M,1) - (K,) → (M,K).
            dx = P_own[:, 0:1] - P_neigh[:, 0]
            dy = P_own[:, 1:2] - P_neigh[:, 1]
            # Toroidal correction (in-place to reduce temporary arrays).
            _m = dx > hw
            if _m.any():
                dx[_m] -= W
            _m = dx < -hw
            if _m.any():
                dx[_m] += W
            _m = dy > hh
            if _m.any():
                dy[_m] -= H
            _m = dy < -hh
            if _m.any():
                dy[_m] += H
            d2 = dx * dx + dy * dy   # (M, K)

            # Self-pair exclusion + cohesion radius gate.
            self_pairs = own_idxs[:, None] == neigh_idx[None, :]   # (M, K)
            within_coh = ~self_pairs & (d2 <= coh_r2) & (d2 > _EPS2)
            if not within_coh.any():
                continue

            coh_count = within_coh.sum(axis=1)   # (M,)

            # ── Separation ────────────────────────────────────────────────
            if w_sep != 0.0:
                sep_mask = within_coh & (d2 < sep_r2)
                if sep_mask.any():
                    inv_d = np.where(sep_mask, 1.0 / np.sqrt(np.maximum(d2, _EPS2)), 0.0)
                    sx = (dx * inv_d).sum(axis=1)   # (M,) unit away-vec sum
                    sy = (dy * inv_d).sum(axis=1)
                    mag = np.hypot(sx, sy)
                    has_sep = (sep_mask.sum(axis=1) > 0) & (mag > _EPS)
                    if has_sep.any():
                        inv_mag = np.where(has_sep, 1.0 / np.maximum(mag, _EPS), 0.0)
                        des_x = sx * inv_mag * max_speed
                        des_y = sy * inv_mag * max_speed
                        stx = des_x - V_own[:, 0]
                        sty = des_y - V_own[:, 1]
                        stm = np.hypot(stx, sty)
                        over = has_sep & (stm > max_force)
                        scale = np.where(over, max_force / np.maximum(stm, _EPS), 1.0)
                        sa[own_idxs, 0] += stx * scale * w_sep * has_sep
                        sa[own_idxs, 1] += sty * scale * w_sep * has_sep

            # ── Alignment ─────────────────────────────────────────────────
            if w_ali != 0.0:
                ali_mask = within_coh & (d2 < ali_r2)
                if ali_mask.any():
                    alx = (V_neigh[:, 0] * ali_mask).sum(axis=1)   # (M,)
                    aly = (V_neigh[:, 1] * ali_mask).sum(axis=1)
                    mag = np.hypot(alx, aly)
                    has_ali = (ali_mask.sum(axis=1) > 0) & (mag > _EPS)
                    if has_ali.any():
                        inv_mag = np.where(has_ali, 1.0 / np.maximum(mag, _EPS), 0.0)
                        des_x = alx * inv_mag * max_speed
                        des_y = aly * inv_mag * max_speed
                        stx = des_x - V_own[:, 0]
                        sty = des_y - V_own[:, 1]
                        stm = np.hypot(stx, sty)
                        over = has_ali & (stm > max_force)
                        scale = np.where(over, max_force / np.maximum(stm, _EPS), 1.0)
                        sa[own_idxs, 0] += stx * scale * w_ali * has_ali
                        sa[own_idxs, 1] += sty * scale * w_ali * has_ali

            # ── Cohesion (suppressed while fleeing) ───────────────────────
            if w_coh != 0.0:
                coh_active = (coh_count > 0) & ~is_fleeing[own_idxs]
                if coh_active.any():
                    # cx = toward centre: -(SELF - NEIGH) = NEIGH - SELF
                    cx = -(dx * within_coh).sum(axis=1)   # (M,)
                    cy = -(dy * within_coh).sum(axis=1)
                    mag = np.hypot(cx, cy)
                    has_coh = coh_active & (mag > _EPS)
                    if has_coh.any():
                        inv_mag = np.where(has_coh, 1.0 / np.maximum(mag, _EPS), 0.0)
                        des_x = cx * inv_mag * max_speed
                        des_y = cy * inv_mag * max_speed
                        stx = des_x - V_own[:, 0]
                        sty = des_y - V_own[:, 1]
                        stm = np.hypot(stx, sty)
                        over = has_coh & (stm > max_force)
                        scale = np.where(over, max_force / np.maximum(stm, _EPS), 1.0)
                        sa[own_idxs, 0] += stx * scale * w_coh * has_coh
                        sa[own_idxs, 1] += sty * scale * w_coh * has_coh

        # ─── Pass 3: Noise ─────────────────────────────────────────────────────
        w_noise = cfg.weight_noise
        if w_noise != 0.0:
            noise = np.random.normal(0.0, 1.0, (n, 2))
            mag_n = np.hypot(noise[:, 0], noise[:, 1])
            valid_n = mag_n > _EPS
            if valid_n.any():
                inv_mag_n = np.where(valid_n, 1.0 / np.maximum(mag_n, _EPS), 0.0)
                des_x = noise[:, 0] * inv_mag_n * max_speed
                des_y = noise[:, 1] * inv_mag_n * max_speed
                stx = des_x - sv[:, 0]
                sty = des_y - sv[:, 1]
                stm = np.hypot(stx, sty)
                over_n = valid_n & (stm > max_force)
                scale_n = np.where(over_n, max_force / np.maximum(stm, _EPS), 1.0)
                sa[:, 0] += stx * scale_n * w_noise * valid_n
                sa[:, 1] += sty * scale_n * w_noise * valid_n

    def _batch_integrate_sardines(self, dt: float) -> None:
        """Vectorised Euler integration for all sardines (Phase 4B / 4C).

        Phase 4C note: the gather step that copied ``s.acceleration`` into
        ``_s_acc`` has been removed.  ``_s_acc`` is now pre-populated by
        ``_batch_compute_sardine_forces()`` before this method is called.

        Steps
        -----
        1. **Velocity integration**: ``v += a * dt`` (numpy broadcast over
           the pre-filled ``_s_acc`` rows).
        2. **Speed clamp**: vectorised equivalent of ``_clamp_speed()``.
        3. **Position integration**: ``p += v * dt`` (numpy broadcast).
        4. **Boundary wrap**: toroidal ``%`` for wrap mode; vectorised
           reflection for clamp mode.
        5. **Scatter** updated position and velocity back to sardine objects
           (needed for rendering, ecology, and query neighbours next frame).
        6. **Clear** accumulated accelerations on objects.

        Uses ``_s_pos[: _s_n]``, ``_s_vel[: _s_n]``, ``_s_acc[: _s_n]``
        views so numpy never processes stale rows beyond the live count.

        Sardine positions and velocities are the **authoritative state**
        during this call; sardine objects are re-synchronised at the end.
        """
        n    = self._s_n
        sv   = self._s_vel
        sp   = self._s_pos
        sa   = self._s_acc
        svn  = sv[:n]    # live-row views
        spn  = sp[:n]
        san  = sa[:n]

        # ── Velocity integration ──────────────────────────────────────
        # _s_acc is pre-populated by _batch_compute_sardine_forces().
        # No gather step needed.
        svn += san * dt

        # ── Speed clamping (vectorised _clamp_speed) ───────────────────
        # Computes 2-norm for each row; applies rescale where out of bounds.
        # Mirrors the logic in Boid._clamp_speed() exactly.
        max_s = self.config.sardine_max_speed
        min_s = self.config.sardine_min_speed
        _EPS  = 1e-9

        speeds = np.hypot(svn[:, 0], svn[:, 1])  # (N,)

        # Max clamp: scale rows where speed > max_s.
        over = speeds > max_s
        if over.any():
            scale_over = max_s / speeds[over]
            svn[over, 0] *= scale_over
            svn[over, 1] *= scale_over
            speeds[over]  = max_s   # update for min check below

        # Min floor: only for species with min_speed > 0 (sardines always do).
        if min_s > 0.0:
            valid  = speeds > _EPS
            under  = valid & (speeds < min_s)
            if under.any():
                scale_under = min_s / speeds[under]
                svn[under, 0] *= scale_under
                svn[under, 1] *= scale_under
            # Degenerate zero: fall back to +x heading at min_speed.
            degen = ~valid
            if degen.any():
                svn[degen, 0] = min_s
                svn[degen, 1] = 0.0

        # ── Position integration ───────────────────────────────────────
        spn += svn * dt

        # ── Boundary wrap / clamp ──────────────────────────────────────
        W = float(self.config.window_width)
        H = float(self.config.window_height)

        if self.config.boundary == "wrap":
            spn[:, 0] %= W
            spn[:, 1] %= H
        else:
            # Reflective (clamp) boundary: mirror velocity on each axis.
            out_l = spn[:, 0] < 0.0
            out_r = spn[:, 0] > W
            spn[out_l, 0] = 0.0
            spn[out_r, 0] = W
            svn[out_l | out_r, 0] *= -1.0

            out_b = spn[:, 1] < 0.0
            out_t = spn[:, 1] > H
            spn[out_b, 1] = 0.0
            spn[out_t, 1] = H
            svn[out_b | out_t, 1] *= -1.0

        # ── Scatter: write updated pos+vel back to sardine objects ──────
        # Also clears acceleration in-place (no extra array alloc needed).
        for i, s in enumerate(self.sardines):
            pos = s.position
            vel = s.velocity
            pos[0] = spn[i, 0]
            pos[1] = spn[i, 1]
            vel[0] = svn[i, 0]
            vel[1] = svn[i, 1]
            acc = s.acceleration
            acc[0] = 0.0
            acc[1] = 0.0

    # ── Cooperative baitball detection ───────────────────────────────

    def _wrap_delta(self, dx: float, dy: float, W: float, H: float) -> tuple[float, float]:
        """Return toroidally corrected vector components for world size W×H."""
        hw = W * 0.5
        hh = H * 0.5
        if dx > hw:
            dx -= W
        elif dx < -hw:
            dx += W
        if dy > hh:
            dy -= H
        elif dy < -hh:
            dy += H
        return dx, dy

    def _merge_baitball_candidates(
        self,
        candidates: "list[_BaitballInfo]",
        W: float,
        H: float,
        merge_dist_mult: float,
        overlap_thresh: float,
        max_final: int,
        max_merge_radius: float,
    ) -> "list[_BaitballInfo]":
        """Greedy merge of overlapping per-cell baitball candidates.

        The per-cell detector produces one candidate per occupied grid cell,
        so a single large school generates many overlapping candidates.  This
        pass collapses them into a small set of non-redundant baitballs.

        Algorithm
        ---------
        Sort candidates by score (density × count) descending so the
        strongest representation of each school is chosen as the "seed."
        Each subsequent candidate either merges into the nearest already-
        accepted baitball (if close enough) or starts a new one up to
        *max_final*.

        Merge condition — either test may trigger:
            center_dist < merge_dist_mult × max(r_a, r_b)       [proximity]
            center_dist < (r_a + r_b) × overlap_thresh          [overlap]

        Large-school safeguard:
            If an existing merged baitball's radius already exceeds
            *max_merge_radius* (> 0), it is skipped as a merge target.
            Candidates that would push a baitball beyond that size instead
            start a new baitball (if the cap allows) or are discarded.
            This prevents a map-scale school from collapsing into one giant
            entity and keeps sailfish behavior localized to sub-clusters.

        The merged baitball uses count-weighted center, an enclosing radius
        computed from that center (exact, no sardine re-scan needed), and
        density = combined_count / new_radius.
        """
        if not candidates:
            return []

        scored = sorted(candidates, key=lambda b: b.density * b.count, reverse=True)
        final: list = []

        for cand in scored:
            best_i = -1
            best_d2 = float("inf")

            for i, ex in enumerate(final):
                # Large-school safeguard: skip already-oversized baitballs.
                if max_merge_radius > 0.0 and ex.radius >= max_merge_radius:
                    continue

                dx = cand.cx - ex.cx
                dy = cand.cy - ex.cy
                dx, dy = self._wrap_delta(dx, dy, W, H)
                d2 = dx * dx + dy * dy

                max_r = max(cand.radius, ex.radius)
                sum_r = cand.radius + ex.radius
                thresh = max(merge_dist_mult * max_r, overlap_thresh * sum_r)
                if d2 <= thresh * thresh and d2 < best_d2:
                    best_i = i
                    best_d2 = d2

            if best_i < 0:
                # No close enough existing baitball — start a new one if cap
                # not yet reached.  Once the cap is hit, remaining candidates
                # are still checked for merging into existing baitballs (the
                # loop above runs for all of them regardless).
                if len(final) < max_final:
                    final.append(cand)
            else:
                # Merge cand into final[best_i].
                ex = final[best_i]
                total = ex.count + cand.count
                w_ex  = ex.count / total   # weight of existing
                w_c   = cand.count / total  # weight of candidate

                # Shortest-path toroidal delta from ex to cand.
                dx = cand.cx - ex.cx
                dy = cand.cy - ex.cy
                dx, dy = self._wrap_delta(dx, dy, W, H)
                d  = math.sqrt(best_d2)

                # Count-weighted center (expressed as offset from ex, then
                # wrapped back into world bounds).
                new_cx = (ex.cx + dx * w_c) % W
                new_cy = (ex.cy + dy * w_c) % H

                # Enclosing radius: the farthest sardine reachable from
                # new_cx/cy is bounded by (dist to constituent center) +
                # (constituent radius).  Distances:
                #   merged_center to ex:   d × w_c  (moves toward cand)
                #   merged_center to cand: d × w_ex (moves away from ex)
                dist_to_ex   = d * w_c
                dist_to_cand = d * w_ex
                new_radius = max(8.0,
                                 dist_to_ex   + ex.radius,
                                 dist_to_cand + cand.radius)
                new_density = total / new_radius

                final[best_i] = _BaitballInfo(
                    bid=ex.bid,
                    cx=new_cx,
                    cy=new_cy,
                    radius=new_radius,
                    count=total,
                    density=new_density,
                )

        # Assign stable bid indices to final baitballs in score order.
        return [_BaitballInfo(bid=i, cx=b.cx, cy=b.cy, radius=b.radius,
                               count=b.count, density=b.density)
                for i, b in enumerate(final)]


    def _detect_baitballs(self) -> list[_BaitballInfo]:
        """Approximate dense sardine clusters from sardine-grid neighborhoods.

        The detector iterates occupied sardine cells and evaluates only local
        neighbourhoods sized by ``sailfish_baitball_detection_radius``.  This
        avoids global pairwise clustering while still capturing dense schools.

        Performance notes
        -----------------
        * Inner distance loops are vectorised with numpy using the pre-populated
          ``_s_pos[:_s_n]`` batch array (no per-sardine attribute reads).
        * The toroidal correction is inlined via numpy where() — eliminates the
          10 M/frame Python-level ``_wrap_delta()`` calls seen in profiling.
        * Neighbor index gathering uses a pre-allocated reusable integer buffer
          (``_bb_neigh_buf``) to avoid per-cell list allocations.
        * Detection is staggered (called every N frames); the result is cached
          and reused by the caller between detection frames.
        """
        cfg = self.config
        if not cfg.sailfish_baitball_enabled:
            return []
        if not self.sardines:
            return []

        detect_r = float(cfg.sailfish_baitball_detection_radius)
        if detect_r <= 1.0:
            return []

        min_count = max(2, int(cfg.sailfish_baitball_min_sardines))
        density_threshold = float(cfg.sailfish_baitball_density_threshold)
        detect_r2 = detect_r * detect_r

        W  = float(cfg.window_width)
        H  = float(cfg.window_height)
        hw = W * 0.5
        hh = H * 0.5

        grid  = self._grid
        cells = grid._cells
        if not cells:
            return []

        span = max(1, math.ceil(detect_r / grid.cell_size))
        nc   = grid._num_cols
        nr   = grid._num_rows

        # Pre-populated sardine position batch array (N×2 float64).
        # Each sardine's row index == s._idx set in the Phase-4A gather loop.
        # If called directly (e.g. tests bypass update()), populate on demand.
        sp = self._s_pos   # shape (cap, 2); live rows are [:_s_n]
        n_s = self._s_n
        if n_s == 0 and self.sardines:
            # Fallback: populate arrays without caching (direct call from tests).
            n_s = len(self.sardines)
            self._ensure_sardine_capacity(n_s)
            sp = self._s_pos
            for i, s in enumerate(self.sardines):
                s._idx = i
                sp[i, 0] = s._gx
                sp[i, 1] = s._gy
            self._s_n = n_s

        # Reusable integer buffer for neighbor sardine indices — avoids a fresh
        # list allocation per occupied cell.  Grown lazily via 2× policy.
        if self._bb_neigh_buf.shape[0] < n_s:
            cap = max(n_s * 2, 64)
            self._bb_neigh_buf = np.empty(cap, dtype=np.int32)
        neigh_buf = self._bb_neigh_buf

        candidates: list[_BaitballInfo] = []

        fast_path = nc > 2 * span and nr > 2 * span

        for (col_c, row_c), owner_cell in cells.items():
            if not owner_cell:
                continue

            # Gather neighbor sardine indices into the reusable buffer.
            nb_n = 0
            if fast_path:
                for dc in range(-span, span + 1):
                    col = (col_c + dc) % nc
                    for dr in range(-span, span + 1):
                        ncell = cells.get((col, (row_c + dr) % nr))
                        if ncell is not None:
                            for s in ncell:
                                neigh_buf[nb_n] = s._idx
                                nb_n += 1
            else:
                visited: set[tuple[int, int]] = set()
                for dc in range(-span, span + 1):
                    col = (col_c + dc) % nc
                    for dr in range(-span, span + 1):
                        key = (col, (row_c + dr) % nr)
                        if key in visited:
                            continue
                        visited.add(key)
                        ncell = cells.get(key)
                        if ncell is not None:
                            for s in ncell:
                                neigh_buf[nb_n] = s._idx
                                nb_n += 1

            if nb_n < min_count:
                continue

            # ── Vectorised distance pass (numpy) ───────────────────────────
            # Use the pre-populated _s_pos batch array so we never call s._gx
            # or s._gy inside a Python loop.
            idxs = neigh_buf[:nb_n]          # view — no copy
            P    = sp[idxs]                  # (K, 2) neighbour positions

            anchor = owner_cell[0]
            ax = anchor._gx
            ay = anchor._gy

            dx = P[:, 0] - ax
            dy = P[:, 1] - ay

            # Toroidal correction — in-place to reduce temporary arrays.
            _m = dx > hw
            if _m.any():
                dx[_m] -= W
            _m = dx < -hw
            if _m.any():
                dx[_m] += W
            _m = dy > hh
            if _m.any():
                dy[_m] -= H
            _m = dy < -hh
            if _m.any():
                dy[_m] += H
            d2 = dx * dx + dy * dy

            mask = d2 <= detect_r2
            n_local = int(mask.sum())
            if n_local < min_count:
                continue

            dx_l = dx[mask]
            dy_l = dy[mask]

            # Count-weighted centre (relative to anchor, then added back).
            inv_n = 1.0 / n_local
            sx    = float(dx_l.sum()) * inv_n
            sy    = float(dy_l.sum()) * inv_n
            cx    = (ax + sx) % W
            cy    = (ay + sy) % H

            # Radius: max distance from centre among local sardines.
            # Use absolute positions of local sardines for second-pass delta.
            P_l   = P[mask]                  # (n_local, 2) absolute positions
            dx2   = P_l[:, 0] - cx
            dy2   = P_l[:, 1] - cy
            _m = dx2 > hw
            if _m.any():
                dx2[_m] -= W
            _m = dx2 < -hw
            if _m.any():
                dx2[_m] += W
            _m = dy2 > hh
            if _m.any():
                dy2[_m] -= H
            _m = dy2 < -hh
            if _m.any():
                dy2[_m] += H
            max_d2 = float((dx2 * dx2 + dy2 * dy2).max())

            rad     = max(8.0, math.sqrt(max_d2))
            density = n_local / rad
            if density < density_threshold:
                continue

            candidates.append(_BaitballInfo(
                bid=0,
                cx=cx,
                cy=cy,
                radius=rad,
                count=n_local,
                density=density,
            ))


        if not candidates:
            return []

        return self._merge_baitball_candidates(
            candidates,
            W,
            H,
            float(cfg.sailfish_baitball_merge_distance_multiplier),
            float(cfg.sailfish_baitball_merge_overlap_threshold),
            int(cfg.sailfish_baitball_max_final_baitballs),
            float(cfg.sailfish_baitball_max_merge_radius),
        )

    # ── Resize / config propagation ──────────────────────────────────

    def resize(self, new_w: int, new_h: int) -> None:
        """Respond to a window resize: update world dimensions in-place.

        Updates ``config``, spatial-hash grid sizes, and per-agent cached
        world dimensions.  Fish positions are toroidally clamped to the new
        bounds so nothing escapes.  Call this from the pyglet ``on_resize``
        handler; the renderer should call its own ``on_resize`` separately.

        Parameters
        ----------
        new_w, new_h:
            New window client-area dimensions in pixels.
        """
        from dataclasses import replace as dc_replace

        new_config = dc_replace(self.config, window_width=new_w, window_height=new_h)
        self.config = new_config

        W, H = float(new_w), float(new_h)
        hw, hh = W * 0.5, H * 0.5

        # Propagate new config and cached dimensions to every agent.
        for agent in self.sardines:
            agent._config = new_config
            agent._W  = W;  agent._H  = H
            agent._hw = hw; agent._hh = hh
        for agent in self.sailfish:
            agent._config = new_config
            agent._W  = W;  agent._H  = H
            agent._hw = hw; agent._hh = hh
        for agent in self.algae:
            agent._config = new_config
            agent._W  = W;  agent._H  = H
            agent._hw = hw; agent._hh = hh

        # Resize the three spatial hashes; they rebuild from scratch each frame.
        self._grid.resize(W, H)
        self._sailfish_grid.resize(W, H)
        self._algae_grid.resize(W, H)

        # Clamp all positions into the new world bounds via toroidal wrap.
        # Iterate each list separately to avoid temporary merged allocations.
        for agent in self.sardines:
            agent.position[0] %= W
            agent.position[1] %= H
        for agent in self.sailfish:
            agent.position[0] %= W
            agent.position[1] %= H
        for agent in self.algae:
            agent.position[0] %= W
            agent.position[1] %= H

    def apply_config(self, new_config: SimConfig) -> None:
        """Apply a live config change to all agents without respawning.

        Called by the config menu on every ← / → key press.  Propagates
        the new config and refreshes per-agent cached values derived from
        config fields.  Does not touch world dimensions or reallocate agents
        (use ``resize()`` for geometry changes).

        Parameters
        ----------
        new_config:
            A ``SimConfig`` produced via
            ``dataclasses.replace(self.config, **{field: value})``.
        """
        self.config = new_config
        for agent in self.sardines:
            agent._config    = new_config
            # Refresh squared-radius caches used in _compute_forces each frame.
            agent._sep_r2    = new_config.radius_separation ** 2
            agent._ali_r2    = new_config.radius_alignment  ** 2
            agent._coh_r2    = new_config.radius_cohesion   ** 2
            agent._flee_r2   = new_config.radius_flee       ** 2
            # Refresh span caches (cell_size is constant; only radius changes).
            agent._coh_span  = max(1, math.ceil(new_config.radius_cohesion / agent._grid.cell_size))
            agent._flee_span = (
                max(1, math.ceil(new_config.radius_flee / agent._sailfish_grid.cell_size))
                if agent._sailfish_grid is not None else 1
            )
            # Propagate kinematic limits — stored on Boid as instance attrs and
            # read directly by _steer_apply_xy / _clamp_speed every frame.
            agent.max_speed  = new_config.sardine_max_speed
            agent.max_force  = new_config.sardine_max_force
            agent.min_speed  = new_config.sardine_min_speed
            # Boundary mode cache — avoids attr-chain + string comparison each frame.
            agent._boundary_wrap = (new_config.boundary == "wrap")
        for agent in self.sailfish:
            agent._config   = new_config
            # Same kinematic propagation as sardines — Boid instance attrs are
            # read directly by _steer_apply_xy and _clamp_speed each frame.
            agent.max_speed = new_config.sailfish_max_speed
            agent.max_force = new_config.sailfish_max_force
            agent.min_speed = new_config.sailfish_min_speed
            agent._boundary_wrap = (new_config.boundary == "wrap")
            # Refresh sailfish-flocking squared-radius caches.
            agent._sf_sep_r2 = new_config.radius_sailfish_separation ** 2
            agent._sf_ali_r2 = new_config.radius_sailfish_alignment  ** 2
            agent._sf_coh_r2 = new_config.radius_sailfish_cohesion   ** 2
            # Clear individual target lock — acquisition/drop radii may have
            # changed so stale targets are released; they will re-acquire next frame.
            agent._target_sardine    = None
            agent._target_lock_timer = 0.0
            agent._burst_timer          = 0.0
            agent._burst_cooldown_timer = 0.0
            agent._baitball_strike_timer = 0.0
            agent._baitball_strike_cooldown_timer = 0.0
            agent._baitball_memory_timer = 0.0
            agent._baitball = None
            agent._is_herding = False
            agent._orbiting_baitball = False
            agent._orbit_timer = 0.0
            agent._orbit_patience_goal = 0.0
            agent._pending_group_strike_trigger = False
            agent._pending_group_strike_kind = ""
            # Reset proto-collaboration state; per-fish random orbit vars are
            # intentionally preserved across config changes.
            agent._individual_strike_cooldown = 0.0
            agent._is_baitball_attacker = False
            agent._active_baitball_strikes = 0
            agent._prev_baitball_cx = -1.0
            agent._prev_baitball_cy = -1.0
        for agent in self.algae:
            agent._config   = new_config
            agent._sep_r2   = new_config.radius_algae_separation ** 2
            agent._ali_r2   = new_config.radius_algae_alignment  ** 2
            agent._coh_r2   = new_config.radius_algae_cohesion   ** 2
            agent._coh_span = max(1, math.ceil(new_config.radius_algae_cohesion / agent._grid.cell_size))
            agent.max_speed = new_config.algae_max_speed
            agent.max_force = new_config.algae_max_force
            # min_speed is always 0.0 for algae — not a config field
            agent._boundary_wrap = (new_config.boundary == "wrap")
        # Propagate mouse predator params to the shared stimulus object so all
        # sardines see the updated radius / weight without being recreated.
        ms = self._mouse_stimulus
        ms.radius           = new_config.mouse_predator_radius
        ms.radius_sq        = new_config.mouse_predator_radius ** 2
        ms.flee_weight_mult = new_config.mouse_predator_flee_weight
        self._baitball_group_strike_cooldown_timer = 0.0
        self.debug_group_strike_cooldown = 0.0
        # Refresh baitball detection interval from config.
        self._baitball_detect_interval = max(1, int(new_config.baitball_detect_interval))
        # If GPU was just enabled and we haven't tried (or previously failed),
        # allow a fresh attempt so toggling the menu item takes effect.
        if new_config.sardine_use_gpu and self._gpu_backend is None:
            self._gpu_init_tried = False

    # ── Mouse predator interface ──────────────────────────────────────

    def set_mouse_predator(self, x: float, y: float) -> None:
        """Activate the mouse predator at screen position *(x, y)*.

        Called by ``main.py`` on ``on_mouse_press`` and ``on_mouse_drag``
        events (left button).  Sardines within ``mouse_predator_radius`` will
        flee away from this point on the next ``update()`` call.
        """
        ms = self._mouse_stimulus
        ms.active = True
        ms.x = x
        ms.y = y

    def clear_mouse_predator(self) -> None:
        """Deactivate the mouse predator.

        Called by ``main.py`` on ``on_mouse_release`` (left button).  The
        flee stimulus is removed immediately; sardines return to normal
        flocking on the next ``update()`` call.
        """
        self._mouse_stimulus.active = False

    def add_sardines(self, n: int) -> None:
        """Spawn *n* new sardines at random positions and append to the world.

        After this call the caller should invoke ``renderer.reset()`` so
        new Triangle shapes are allocated for the added agents.
        """
        if n <= 0:
            return
        from sim.sardine import Sardine
        rng   = np.random.default_rng()
        W, H  = float(self.config.window_width), float(self.config.window_height)
        speed = self.config.sardine_min_speed
        for _ in range(n):
            angle = float(rng.uniform(0.0, 2.0 * np.pi))
            self.sardines.append(
                Sardine(
                    float(rng.uniform(0.0, W)),
                    float(rng.uniform(0.0, H)),
                    np.cos(angle) * speed,
                    np.sin(angle) * speed,
                    self.config, self._grid, self._sailfish_grid,
                    self._mouse_stimulus,
                )
            )
        self._sync_population_counts()

    def remove_sardines(self, n: int) -> int:
        """Remove up to *n* sardines (from the end of the list).

        Returns the number actually removed.  After this call the caller
        should invoke ``renderer.reset()``.
        """
        n = min(max(n, 0), len(self.sardines))
        if n:
            for _s in self.sardines[-n:]:
                _s.is_alive = False   # release any sailfish target locks
            del self.sardines[-n:]
        self._sync_population_counts()
        return n

    def add_sailfish(self, n: int) -> None:
        """Spawn *n* new sailfish at random positions."""
        if n <= 0:
            return
        from sim.sailfish import Sailfish
        rng   = np.random.default_rng()
        W, H  = float(self.config.window_width), float(self.config.window_height)
        speed = self.config.sailfish_min_speed
        for _ in range(n):
            angle = float(rng.uniform(0.0, 2.0 * np.pi))
            self.sailfish.append(
                Sailfish(
                    float(rng.uniform(0.0, W)),
                    float(rng.uniform(0.0, H)),
                    np.cos(angle) * speed,
                    np.sin(angle) * speed,
                    self.config, self._grid,
                    sailfish_grid=self._sailfish_grid,
                )
            )
        self._sync_population_counts()

    def remove_sailfish(self, n: int) -> int:
        """Remove up to *n* sailfish (from the end of the list)."""
        n = min(max(n, 0), len(self.sailfish))
        if n:
            del self.sailfish[-n:]
        self._sync_population_counts()
        return n

    def _sync_population_counts(self) -> None:
        """Keep ``config.num_sardines`` and ``config.num_sailfish`` in sync
        with the actual live list lengths.

        Called after every add/remove so that ``save_config(simulation.config)``
        always persists the true current population rather than the startup
        value.
        """
        from dataclasses import replace as dc_replace
        live_ns = len(self.sardines)
        live_nf = len(self.sailfish)
        if (
            self.config.num_sardines != live_ns
            or self.config.num_sailfish != live_nf
        ):
            self.config = dc_replace(
                self.config,
                num_sardines=live_ns,
                num_sailfish=live_nf,
            )

    # ── Accessors ─────────────────────────────────────────────────────

    @property
    def grid(self) -> SpatialHash:
        """Expose grid so agents can query it during _compute_forces."""
        return self._grid

    def diagnostics(self) -> dict:
        """Return a lightweight snapshot of current simulation state.

        Useful for tests, periodic console logging, and HUD display.
        All values are computed fresh on each call (cheap at 300 agents).

        Returns
        -------
        dict with keys:
            count          — number of active sardines
            mean_speed     — average sardine speed (px/s)
            min_speed      — slowest sardine (px/s)
            max_speed      — fastest sardine (px/s)
            center_of_mass — naive positional mean (Vec2); note: may be
                             misleading when the school straddles the toroidal
                             wrap boundary (a known limitation of naive means
                             in toroidal spaces)
        """
        sardines = self.sardines
        if not sardines:
            return {
                "count": 0, "mean_speed": 0.0,
                "min_speed": 0.0, "max_speed": 0.0,
                "center_of_mass": np.zeros(2),
            }
        speeds    = np.array([s.speed for s in sardines])
        positions = np.array([s.position for s in sardines])
        return {
            "count":          len(sardines),
            "mean_speed":     float(speeds.mean()),
            "min_speed":      float(speeds.min()),
            "max_speed":      float(speeds.max()),
            "center_of_mass": positions.mean(axis=0),
        }
