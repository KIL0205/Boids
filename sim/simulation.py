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

        # ── Population history (for 'd'-key graph overlay) ────────────
        # Sampled at ~1 Hz of simulated time; deque cap prevents unbounded
        # growth.  Each entry is (sim_time, n_sardines, n_sailfish, n_algae).
        self.sim_time: float = 0.0
        self._pop_history_accum: float = 0.0
        self.pop_history: collections.deque = collections.deque(maxlen=3600)
        self.pop_history.append(
            (0.0, len(self.sardines), len(self.sailfish), len(self.algae))
        )

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
            self._batch_compute_sardine_forces()
        for agent in self.sailfish:
            agent.update(dt)
        for agent in self.algae:
            agent.update(dt)

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
                # Decrement individual-target lock timer (counts down to 0, then lock releases).
                if sf._target_lock_timer > 0.0:
                    sf._target_lock_timer = max(0.0, sf._target_lock_timer - dt)
                # Burst timers: count down burst; start cooldown when burst expires.
                if sf._burst_timer > 0.0:
                    sf._burst_timer = max(0.0, sf._burst_timer - dt)
                    if sf._burst_timer == 0.0:
                        sf._burst_cooldown_timer = sf._config.sailfish_burst_cooldown
                elif sf._burst_cooldown_timer > 0.0:
                    sf._burst_cooldown_timer = max(0.0, sf._burst_cooldown_timer - dt)

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
            # Toroidal correction.
            dx_sf = np.where(dx_sf >  hw, dx_sf - W, np.where(dx_sf < -hw, dx_sf + W, dx_sf))
            dy_sf = np.where(dy_sf >  hh, dy_sf - H, np.where(dy_sf < -hh, dy_sf + H, dy_sf))
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
            mdx = np.where(mdx >  hw, mdx - W, np.where(mdx < -hw, mdx + W, mdx))
            mdy = np.where(mdy >  hh, mdy - H, np.where(mdy < -hh, mdy + H, mdy))
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

        for (col_c, row_c), owner_cell in cells.items():
            M = len(owner_cell)
            own_idxs = np.empty(M, dtype=np.int32)
            for i, s in enumerate(owner_cell):
                own_idxs[i] = s._idx

            # Gather neighbour indices from (2·coh_span+1)² cells.
            # Fast path (span=1, large world): 9 keys are always distinct —
            # mirrors the same guarantee in SpatialHash.query().
            neigh_list: list = []
            if nc > 2 * coh_span and nr > 2 * coh_span:
                for dc in range(-coh_span, coh_span + 1):
                    col = (col_c + dc) % nc
                    for dr in range(-coh_span, coh_span + 1):
                        ncell = cells.get((col, (row_c + dr) % nr))
                        if ncell is not None:
                            for nb in ncell:
                                neigh_list.append(nb._idx)
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
                        ncell = cells.get(key)
                        if ncell is not None:
                            for nb in ncell:
                                neigh_list.append(nb._idx)

            if not neigh_list:
                continue

            neigh_idx = np.array(neigh_list, dtype=np.int32)   # (K,)
            P_own   = sp[own_idxs]    # (M, 2)
            P_neigh = sp[neigh_idx]   # (K, 2)
            V_neigh = sv[neigh_idx]   # (K, 2)
            V_own   = sv[own_idxs]    # (M, 2)

            # Pairwise deltas — (M, K) via broadcast; (M,1) - (K,) → (M,K).
            dx = P_own[:, 0:1] - P_neigh[:, 0]
            dy = P_own[:, 1:2] - P_neigh[:, 1]
            # Toroidal correction.
            dx = np.where(dx >  hw, dx - W, np.where(dx < -hw, dx + W, dx))
            dy = np.where(dy >  hh, dy - H, np.where(dy < -hh, dy + H, dy))
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
