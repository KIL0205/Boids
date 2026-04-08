"""
sim/algae.py
============
``Algae`` — a slowly drifting boid that gives the impression of algae
moving with ocean currents.

Behaviour
---------
* **Alignment** — strongly follows the average heading of nearby algae,
  creating the "current drift" visual.
* **Separation** — very weak; prevents overlap without scattering.
* **Cohesion** — very weak; keeps loose aggregates without tight schooling.
* **Noise** — moderate perturbation for an organic, drift-like feel.
  Fires even when there are no neighbours so isolated algae still drift.

No flee, no predator awareness.  ``min_speed = 0.0`` so algae can slow to
near-zero in calm regions.

All neighbour lookups go through the shared ``SpatialHash`` algae grid
injected at construction time.  The grid is rebuilt by ``Simulation`` every
frame before calling ``update(dt)``, so queries always see current positions.

Weights and perception radii are read from ``SimConfig`` — no magic numbers.
"""

from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING

import numpy as np

from sim.boid import Boid
from config.config import SimConfig

if TYPE_CHECKING:
    from sim.spatial_hash import SpatialHash

# Guard against zero-distance division (squared, in pixels²).
_EPS2: float = 1e-18


class Algae(Boid):
    """Slowly drifting algae boid.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config — kinematics, flocking weights, radii.
    grid:
        The algae-only spatial hash.  Must be the *same object* rebuilt
        by ``Simulation`` every frame so flocking queries always see current
        algae positions.  Sardines and sailfish are never inserted here.
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        grid: "SpatialHash",
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.algae_max_speed,
            max_force=config.algae_max_force,
            config=config,
            min_speed=0.0,   # algae can slow to near-stop; no min-speed floor
        )
        self._grid: "SpatialHash" = grid

        # Cache per-instance derived values for the _compute_forces hot path.
        self._W:      float = float(config.window_width)
        self._H:      float = float(config.window_height)
        self._hw:     float = self._W * 0.5
        self._hh:     float = self._H * 0.5
        self._sep_r2: float = config.radius_algae_separation ** 2
        self._ali_r2: float = config.radius_algae_alignment  ** 2

    # ── Flocking rules ────────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Apply separation, alignment, cohesion, and noise steering forces.

        Unlike sardines, algae have no flee behaviour.  The alignment weight
        dominates so they drift in loosely coordinated patches that look like
        water-current movement.

        Order of rules (all weighted equally — no priority suppression):

        1. **Separation** — very weak push away from too-close neighbours.
        2. **Alignment** — strong match of local headings (the main driver).
        3. **Cohesion** — very weak pull toward the local group centre.
        4. **Noise** — small organic perturbation.  Fires even with no
           neighbours so isolated algae still drift rather than stalling.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        px  = float(self.position[0])
        py  = float(self.position[1])

        neighbors = self._grid.query(self, cfg.radius_algae_cohesion)

        if neighbors:
            sep_r2 = self._sep_r2
            ali_r2 = self._ali_r2

            # Float-pair accumulators — avoids np.zeros(2) allocations.
            sx = sy = 0.0    # separation: weighted unit away-vectors
            alx = aly = 0.0  # alignment:  sum of neighbour velocities
            cx = cy = 0.0    # cohesion:   vector toward group centre
            sep_count = ali_count = coh_count = 0

            for other in neighbors:
                dx = px - float(other.position[0])
                dy = py - float(other.position[1])
                # Inline toroidal wrap.
                if dx > hw:    dx -= W
                elif dx < -hw: dx += W
                if dy > hh:    dy -= H
                elif dy < -hh: dy += H
                d_sq = dx * dx + dy * dy

                # ── Separation ────────────────────────────────────────
                if d_sq < sep_r2:
                    if d_sq > _EPS2:
                        inv_d = 1.0 / math.sqrt(d_sq)
                        sx += dx * inv_d
                        sy += dy * inv_d
                    sep_count += 1

                # ── Alignment ─────────────────────────────────────────
                if d_sq < ali_r2:
                    alx += float(other.velocity[0])
                    aly += float(other.velocity[1])
                    ali_count += 1

                # ── Cohesion ──────────────────────────────────────────
                cx -= dx
                cy -= dy
                coh_count += 1

            if sep_count:
                self._steer_apply_xy(sx, sy, cfg.weight_algae_separation)
            if ali_count:
                self._steer_apply_xy(alx, aly, cfg.weight_algae_alignment)
            if coh_count:
                self._steer_apply_xy(cx, cy, cfg.weight_algae_cohesion)

        # ── Noise: fires even without neighbours ──────────────────────
        # random.gauss(0, 1) returns a Python float — zero allocation.
        if cfg.weight_algae_noise > 0.0:
            self._steer_apply_xy(
                random.gauss(0.0, 1.0),
                random.gauss(0.0, 1.0),
                cfg.weight_algae_noise,
            )
