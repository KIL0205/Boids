"""
simulation.py
=============
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

from config import SimConfig
from spatial_hash import SpatialHash

if TYPE_CHECKING:
    from boid import Boid


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

        self._spawn_sardines()
        self._spawn_sailfish()

    # ── Initialisation ────────────────────────────────────────────────

    def _spawn_sardines(self) -> None:
        """Populate the world with Sardine boids at random positions and headings."""
        from sardine import Sardine

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

    def _spawn_sailfish(self) -> None:
        """Populate the world with Sailfish predators at random positions."""
        from sailfish import Sailfish

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

        # 1 — rebuild sardine grid (sardines only)
        self._grid.clear()
        for s in self.sardines:
            self._grid.insert(s)

        # 1b — rebuild sailfish grid (sailfish only)
        self._sailfish_grid.clear()
        for sf in self.sailfish:
            self._sailfish_grid.insert(sf)

        # 2 & 3 — update all agents (direct iteration avoids a list allocation)
        for agent in self.sardines:
            agent.update(dt)
        for agent in self.sailfish:
            agent.update(dt)

    # ── Dynamic population management ─────────────────────────────────

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

        # Resize the two spatial hashes; they rebuild from scratch each frame.
        self._grid.resize(W, H)
        self._sailfish_grid.resize(W, H)

        # Clamp all positions into the new world bounds via toroidal wrap.
        for agent in self.sardines + self.sailfish:
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

    def add_sardines(self, n: int) -> None:
        """Spawn *n* new sardines at random positions and append to the world.

        After this call the caller should invoke ``renderer.reset()`` so
        new Triangle shapes are allocated for the added agents.
        """
        if n <= 0:
            return
        from sardine import Sardine
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
        from sailfish import Sailfish
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

    # ── Internals ─────────────────────────────────────────────────────

    def _all_agents(self) -> list["Boid"]:
        """Return all active agents (sardines + sailfish) — used by tests."""
        return list(self.sardines) + list(self.sailfish)
