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

from typing import TYPE_CHECKING

import numpy as np

from config.config import SimConfig
from sim.spatial_hash import SpatialHash

if TYPE_CHECKING:
    from sim.boid import Boid


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
                        self._sailfish_grid)
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
            # Sailfish receive the sardine grid so they can query sardine
            # positions in _compute_forces.  They are never inserted into it.
            self.sailfish.append(
                Sailfish(x, y, vx, vy, self.config, self._grid)
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
        self._grid.clear()
        for s in self.sardines:
            self._grid.insert(s)

        # 1b — rebuild sailfish grid (sailfish only)
        self._sailfish_grid.clear()
        for sf in self.sailfish:
            self._sailfish_grid.insert(sf)

        # 1c — rebuild algae grid (algae only)
        self._algae_grid.clear()
        for a in self.algae:
            self._algae_grid.insert(a)

        # 2 & 3 — update all agents (direct iteration avoids a list allocation)
        for agent in self.sardines:
            agent.update(dt)
        for agent in self.sailfish:
            agent.update(dt)
        for agent in self.algae:
            agent.update(dt)

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
                    self._algae_eaten_ids.update(id(a) for a in to_eat)
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
                    self._sardines_starved_ids.update(id(s) for s in starving)
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

        # Sailfish-sardine eating: one kill per sailfish per tick.
        # Runs after sardine reproduction so newly born sardines are not in
        # _grid (built at frame start) and cannot be immediately eaten.
        if self.sailfish and self.sardines:
            eat_r = self.config.sailfish_eat_radius
            to_remove: set = set()
            for sf in self.sailfish:
                candidates = self._grid.query(sf, eat_r)
                if candidates:
                    prey = candidates[0]          # one kill per sailfish per tick
                    to_remove.add(prey)
                    sf.food_counter += 1
                    sf.hunger_timer = 0.0
            if to_remove:
                self._sardines_eaten_ids.update(id(s) for s in to_remove)
                self.sardines = [s for s in self.sardines if s not in to_remove]
                self.stats_sardines_eaten += len(to_remove)

        # Sailfish starvation: runs after eating so a sailfish that just killed
        # (hunger_timer=0) can never starve the same frame.
        # starvation_time=0.0 disables sailfish starvation.
        if self.sailfish:
            starvation_time = self.config.sailfish_starvation_time
            if starvation_time > 0.0:
                dying: set = {
                    sf for sf in self.sailfish if sf.hunger_timer > starvation_time
                }
                if dying:
                    self._sailfish_died_ids.update(id(sf) for sf in dying)
                    self.sailfish = [sf for sf in self.sailfish if sf not in dying]
                    self.stats_sailfish_starvations += len(dying)

        # Sailfish reproduction: runs after starvation so dying sailfish
        # don't also reproduce.  threshold=0 disables reproduction.
        threshold = self.config.sailfish_eat_threshold
        if threshold > 0 and self.sailfish:
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
        n_current = len(self.sardines)
        for sardine in self.sardines[:n_current]:
            if sardine.food_counter >= threshold:
                sardine.food_counter -= threshold

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
        n_current = len(self.sailfish)
        for sailfish in self.sailfish[:n_current]:
            if sailfish.food_counter >= threshold:
                sailfish.food_counter -= threshold

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

        rng    = np.random.default_rng()  # entropy-seeded; visual-only positions
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
            # Propagate kinematic limits — stored on Boid as instance attrs and
            # read directly by _steer_apply_xy / _clamp_speed every frame.
            agent.max_speed  = new_config.sardine_max_speed
            agent.max_force  = new_config.sardine_max_force
            agent.min_speed  = new_config.sardine_min_speed
        for agent in self.sailfish:
            agent._config   = new_config
            # Same kinematic propagation as sardines — Boid instance attrs are
            # read directly by _steer_apply_xy and _clamp_speed each frame.
            agent.max_speed = new_config.sailfish_max_speed
            agent.max_force = new_config.sailfish_max_force
            agent.min_speed = new_config.sailfish_min_speed
        for agent in self.algae:
            agent._config   = new_config
            agent._sep_r2   = new_config.radius_algae_separation ** 2
            agent._ali_r2   = new_config.radius_algae_alignment  ** 2
            agent.max_speed = new_config.algae_max_speed
            agent.max_force = new_config.algae_max_force
            # min_speed is always 0.0 for algae — not a config field

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
