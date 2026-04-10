"""
sim/sailfish.py
===============
``Sailfish`` — a pursuit predator that steers toward the centre of mass
of nearby sardines, while also maintaining spacing with other sailfish.

Behavioural forces
------------------
* **Pursuit** — compute the local sardine centre of mass (COM) from all
  sardines within ``radius_sailfish_awareness`` and steer toward it.
* **Sailfish separation** — repel from nearby sailfish so the pack spreads
  around prey rather than stacking (``weight_sailfish_separation``).
* **Sailfish alignment** — mild heading match with nearby sailfish
  (``weight_sailfish_alignment``).
* **Sailfish cohesion** — loose pull toward sailfish COM
  (``weight_sailfish_cohesion``; off by default).
* **Idle drift** — when no sardines are visible, coast at
  ``sailfish_min_speed``; sailfish flocking still fires if sailfish_grid
  is provided.

All sardine queries use the sardine ``SpatialHash`` injected at
construction.  Sailfish-to-sailfish queries use the separate
``_sailfish_grid`` hash (also rebuild every frame before update is called).

Weights and radii are read from ``SimConfig`` — no magic numbers.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from sim.boid import Boid
from config.config import SimConfig
from utils.vec2 import Vec2, wrap_delta

if TYPE_CHECKING:
    from sim.spatial_hash import SpatialHash

_EPS2: float = 1e-18  # guards against division by zero in separation


class Sailfish(Boid):
    """Pursuit-predator sailfish boid.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config — kinematics, pursuit/flocking weights, radii.
    sardine_grid:
        The simulation's sardine-only spatial hash.  Rebuilt every frame by
        ``Simulation`` so queries always reflect current sardine positions.
    sailfish_grid:
        The simulation's sailfish-only spatial hash.  Rebuilt every frame
        before ``agent.update()`` is called.  Pass ``None`` to disable
        sailfish-to-sailfish flocking (used by unit tests that construct
        sailfish directly without a simulation).
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        sardine_grid: "SpatialHash",
        sailfish_grid: "SpatialHash | None" = None,
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.sailfish_max_speed,
            max_force=config.sailfish_max_force,
            config=config,
            min_speed=config.sailfish_min_speed,
        )
        self._sardine_grid: "SpatialHash" = sardine_grid
        self._sailfish_grid: "SpatialHash | None" = sailfish_grid

        # Cache per-instance derived values for the _compute_forces hot path.
        self._W:  float = float(config.window_width)
        self._H:  float = float(config.window_height)
        self._hw: float = self._W * 0.5
        self._hh: float = self._H * 0.5

        # Sailfish-flocking squared-radius caches (refreshed by apply_config).
        self._sf_sep_r2: float = config.radius_sailfish_separation ** 2
        self._sf_ali_r2: float = config.radius_sailfish_alignment ** 2
        self._sf_coh_r2: float = config.radius_sailfish_cohesion ** 2

        # Ecology: feeding state (Phase 8.6 / 8.7)
        self.food_counter: float = 0.0   # sardines eaten; carries toward future use
        # Every new sailfish — startup, manually added, or reproduced — starts
        # with hunger_timer = 0.  This is equivalent to "just eaten" and
        # guarantees no spawn path can produce an immediately-starving fish
        # regardless of what sailfish_starvation_time is saved in the config.
        self.hunger_timer: float = 0.0

    # ── Force computation ──────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Apply pursuit (sardine COM) and sailfish-to-sailfish flocking forces.

        Pursuit algorithm
        -----------------
        1. Query the sardine spatial hash within ``radius_sailfish_awareness``.
        2. If sardines found: compute toroidal COM offset and steer toward it.
        3. If no sardines visible: no pursuit force; sailfish drifts at
           min_speed.

        Sailfish flocking algorithm
        ---------------------------
        Runs only when ``_sailfish_grid`` is not ``None``.
        Queries the sailfish grid once using the largest of the three flocking
        radii (cohesion).  Sub-radius checks for separation and alignment are
        applied inline, matching the pattern in ``sardine._compute_forces``.
        SpatialHash.query() already excludes ``self`` from results.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        _pos = self.position
        px   = float(_pos[0])
        py   = float(_pos[1])

        # ── Pursuit: steer toward sardine centre of mass ──────────────
        sardines = self._sardine_grid.query(self, cfg.radius_sailfish_awareness)
        if sardines:
            cx = cy = 0.0
            for s in sardines:
                dx = s._gx - px
                dy = s._gy - py
                if dx >  hw: dx -= W
                elif dx < -hw: dx += W
                if dy >  hh: dy -= H
                elif dy < -hh: dy += H
                cx += dx
                cy += dy
            n = len(sardines)
            self._steer_apply_xy(cx / n, cy / n, cfg.weight_pursuit)

        # ── Sailfish flocking: separation / alignment / cohesion ──────
        if self._sailfish_grid is None:
            return
        sf_grid = self._sailfish_grid
        sep_r2  = self._sf_sep_r2
        ali_r2  = self._sf_ali_r2
        w_sep   = cfg.weight_sailfish_separation
        w_ali   = cfg.weight_sailfish_alignment
        w_coh   = cfg.weight_sailfish_cohesion

        sf_nb = sf_grid.query(self, cfg.radius_sailfish_cohesion)
        if not sf_nb:
            return

        sf_sx = sf_sy = 0.0       # separation: sum of away-unit-vectors
        sf_alx = sf_aly = 0.0     # alignment: sum of neighbour velocities
        sf_cx = sf_cy = 0.0       # cohesion: offset toward COM
        sf_sep_n = sf_ali_n = sf_coh_n = 0

        for other in sf_nb:
            dx = px - other._gx
            dy = py - other._gy
            if dx >  hw: dx -= W
            elif dx < -hw: dx += W
            if dy >  hh: dy -= H
            elif dy < -hh: dy += H
            d_sq = dx * dx + dy * dy
            if d_sq <= _EPS2:
                continue
            if w_sep != 0.0 and d_sq < sep_r2:
                inv_d = 1.0 / math.sqrt(d_sq)
                sf_sx += dx * inv_d
                sf_sy += dy * inv_d
                sf_sep_n += 1
            if w_ali != 0.0 and d_sq < ali_r2:
                sf_alx += other._gvx
                sf_aly += other._gvy
                sf_ali_n += 1
            if w_coh != 0.0:   # within coh_r2 guaranteed by query()
                sf_cx -= dx
                sf_cy -= dy
                sf_coh_n += 1

        if sf_sep_n > 0:
            self._steer_apply_xy(sf_sx, sf_sy, w_sep)
        if sf_ali_n > 0:
            n_inv = 1.0 / sf_ali_n
            self._steer_apply_xy(sf_alx * n_inv, sf_aly * n_inv, w_ali)
        if sf_coh_n > 0:
            n_inv = 1.0 / sf_coh_n
            self._steer_apply_xy(sf_cx * n_inv, sf_cy * n_inv, w_coh)
