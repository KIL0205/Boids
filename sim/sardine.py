"""
sim/sardine.py
==============
``Sardine`` — a flocking boid that implements the three classic Reynolds rules:

* **Separation** — maintain personal space; push away from too-close neighbours,
  weighted by 1/distance so the closest fish exert the strongest repulsion.
* **Alignment** — match the average heading of nearby neighbours.
* **Cohesion** — steer toward the centre of mass of the local group.

All neighbour lookups go through the shared ``SpatialHash`` grid injected at
construction time.  The grid is rebuilt by ``Simulation`` every frame before
calling ``update(dt)``, so queries always reflect current positions.

Weights and perception radii are read from ``SimConfig`` — no magic numbers
here.
"""

from __future__ import annotations

import math
import random
from typing import TYPE_CHECKING

import numpy as np

from sim.boid import Boid
from config.config import SimConfig
from utils.vec2 import Vec2, wrap_delta

if TYPE_CHECKING:
    from sim.spatial_hash import SpatialHash

# Guard against zero-distance division (squared, in pixels²).
_EPS2: float = 1e-18


class Sardine(Boid):
    """Flocking sardine boid.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config — kinematics, flocking weights, radii.
    grid:
        The sardine-only spatial hash.  Must be the *same object* rebuilt
        by ``Simulation`` every frame so flocking queries always see current
        sardine positions.  Sailfish are never inserted into this grid.
    sailfish_grid:
        The sailfish-only spatial hash used for predator detection.  When
        provided, sardines flee any sailfish within ``config.radius_flee``.
        Pass ``None`` (default) to disable flee behaviour — useful for
        Phase-1 tests or standalone flocking experiments.
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        grid: "SpatialHash",
        sailfish_grid: "SpatialHash | None" = None,
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.sardine_max_speed,
            max_force=config.sardine_max_force,
            config=config,
            min_speed=config.sardine_min_speed,
        )
        self._grid: "SpatialHash" = grid
        self._sailfish_grid: "SpatialHash | None" = sailfish_grid

        # Cache per-instance derived values — avoids repeated config lookups and
        # float() conversions in the per-frame _compute_forces hot path.
        self._W:      float = float(config.window_width)
        self._H:      float = float(config.window_height)
        self._hw:     float = self._W * 0.5   # half-world for toroidal wrap
        self._hh:     float = self._H * 0.5
        self._sep_r2: float = config.radius_separation ** 2
        self._ali_r2: float = config.radius_alignment  ** 2

    # ── Flocking rules ────────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Apply flee, separation, alignment, and cohesion steering forces.

        Order of priority (highest to lowest):

        1. **Flee** — if any sailfish is within *radius_flee*, steer directly
           away from the combined threat and suppress cohesion so the school
           can split.  Fires even when there are no sardine neighbours.
        2. **Separation** — maintain personal space.
        3. **Alignment** — match local headings (kept during flee so
           sub-groups stay coordinated while escaping).
        4. **Cohesion** — steer toward group centre.  Zeroed while fleeing;
           resumes automatically once the sailfish exits *radius_flee*.
        5. **Noise** — small organic perturbation.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        # Cache position as Python floats once — avoids repeated numpy scalar
        # extraction inside the neighbour loops below.
        px  = float(self.position[0])
        py  = float(self.position[1])

        # ── 1. Flee (runs first; fires even with no sardine neighbours) ──
        fleeing = False
        if self._sailfish_grid is not None:
            predators = self._sailfish_grid.query(self, cfg.radius_flee)
            if predators:
                fvx = fvy = 0.0
                for sf in predators:
                    dx = px - float(sf.position[0])
                    dy = py - float(sf.position[1])
                    # Inline toroidal wrap (saves wrap_delta function-call overhead).
                    if dx > hw:    dx -= W
                    elif dx < -hw: dx += W
                    if dy > hh:    dy -= H
                    elif dy < -hh: dy += H
                    d2 = dx * dx + dy * dy
                    if d2 > _EPS2:
                        inv_d = 1.0 / math.sqrt(d2)
                        fvx += dx * inv_d
                        fvy += dy * inv_d
                if fvx * fvx + fvy * fvy > _EPS2:
                    self._steer_apply_xy(fvx, fvy, cfg.weight_flee)
                    fleeing = True

        # ── 2–4. Flocking rules ───────────────────────────────────────────
        sep_r2 = self._sep_r2
        ali_r2 = self._ali_r2

        neighbors = self._grid.query(self, cfg.radius_cohesion)
        if not neighbors:
            return

        # Float-pair accumulators avoid 3× np.zeros(2) allocations per frame
        # and save numpy iadd overhead in the alignment accumulation.
        sx = sy = 0.0    # separation: weighted unit away-vectors
        alx = aly = 0.0  # alignment:  sum of neighbour velocities
        cx = cy = 0.0    # cohesion:   vector toward group centre
        sep_count = ali_count = coh_count = 0

        for other in neighbors:
            dx = px - float(other.position[0])
            dy = py - float(other.position[1])
            if dx > hw:    dx -= W
            elif dx < -hw: dx += W
            if dy > hh:    dy -= H
            elif dy < -hh: dy += H
            d_sq = dx * dx + dy * dy

            # ── Separation: unit away-vector weighted by 1/distance ──────
            if d_sq < sep_r2:
                if d_sq > _EPS2:
                    inv_d = 1.0 / math.sqrt(d_sq)
                    sx += dx * inv_d
                    sy += dy * inv_d
                sep_count += 1

            # ── Alignment: accumulate neighbour velocities ────────────────
            if d_sq < ali_r2:
                alx += float(other.velocity[0])
                aly += float(other.velocity[1])
                ali_count += 1

            # ── Cohesion: accumulate toward-neighbour vectors ─────────────
            cx -= dx
            cy -= dy
            coh_count += 1

        # _steer_apply_xy: zero-allocation Reynolds steer + accumulate.
        if sep_count:
            self._steer_apply_xy(sx, sy, cfg.weight_separation)
        if ali_count:
            self._steer_apply_xy(alx, aly, cfg.weight_alignment)
        # Cohesion zeroed while fleeing — allows sub-groups to split and
        # scatter.  Resumes automatically once the sailfish exits radius_flee.
        if coh_count and not fleeing:
            self._steer_apply_xy(cx, cy, cfg.weight_cohesion)

        # ── 5. Noise: small random perturbation ──────────────────────────
        if cfg.weight_noise > 0.0:
            # random.gauss(0, 1) returns a Python float — zero allocation,
            # no numpy dispatch.  Replaces the previous np.random.randn(2)
            # which allocated a (2,) array per sardine per frame.
            self._steer_apply_xy(
                random.gauss(0.0, 1.0),
                random.gauss(0.0, 1.0),
                cfg.weight_noise,
            )
