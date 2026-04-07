"""
sailfish.py
===========
``Sailfish`` — a pursuit predator that steers toward the centre of mass
of nearby sardines.

Phase-2 predator behaviour
---------------------------
* **Pursuit** — compute the local sardine centre of mass (COM) from all
  sardines within ``radius_sailfish_awareness`` and steer toward it.
* **Idle drift** — when no sardines are visible, coast at ``sailfish_min_speed``
  with no additional steering force applied.

All sardine-position queries go through the sardine ``SpatialHash`` grid
injected at construction time.  This is the *same* grid rebuilt by
``Simulation`` every frame, so queries always reflect current sardine
positions.

The sailfish grid (``Simulation._sailfish_grid``) is a *separate* hash
containing only sailfish, queried by sardines for flee detection.  Sailfish
themselves never query the sailfish grid.

Weights and perception radii are read from ``SimConfig`` — no magic numbers.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from boid import Boid
from config import SimConfig
from utils.vec2 import Vec2, wrap_delta

if TYPE_CHECKING:
    from spatial_hash import SpatialHash


class Sailfish(Boid):
    """Pursuit-predator sailfish boid.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config — kinematics, pursuit weights, radii.
    sardine_grid:
        The simulation's sardine-only spatial hash.  Must be the *same
        object* rebuilt by ``Simulation`` every frame so queries always
        see current sardine positions.
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        sardine_grid: "SpatialHash",
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.sailfish_max_speed,
            max_force=config.sailfish_max_force,
            config=config,
            min_speed=config.sailfish_min_speed,
        )
        self._sardine_grid: "SpatialHash" = sardine_grid

        # Cache per-instance derived values for the _compute_forces hot path.
        self._W:  float = float(config.window_width)
        self._H:  float = float(config.window_height)
        self._hw: float = self._W * 0.5
        self._hh: float = self._H * 0.5

    # ── Pursuit behaviour ─────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Steer toward the local sardine centre of mass.

        Algorithm
        ---------
        1. Query the sardine spatial hash within ``radius_sailfish_awareness``.
        2. If no sardines found → return (coast at min_speed; no force applied).
        3. Compute the toroidal COM offset using inline wrap logic (avoids
           wrap_delta function-call overhead; same correctness guarantee).
        4. ``_steer_apply_xy`` applies Reynolds pursuit in one allocation-free
           operation.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        px  = float(self.position[0])
        py  = float(self.position[1])

        sardines = self._sardine_grid.query(self, cfg.radius_sailfish_awareness)
        if not sardines:
            return  # coast at sailfish_min_speed; no steering force this frame

        # Toroidal COM offset — float accumulators avoid np.zeros(2) + divide.
        cx = cy = 0.0
        for s in sardines:
            dx = float(s.position[0]) - px
            dy = float(s.position[1]) - py
            if dx > hw:    dx -= W
            elif dx < -hw: dx += W
            if dy > hh:    dy -= H
            elif dy < -hh: dy += H
            cx += dx
            cy += dy
        n = len(sardines)
        self._steer_apply_xy(cx / n, cy / n, cfg.weight_pursuit)
