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

        # Individual prey targeting state.
        # _target_sardine: the locked individual sardine (or None).
        # _target_lock_timer: seconds remaining on the current lock.  Decremented
        #   by Simulation each frame alongside hunger_timer.
        self._target_sardine: "object | None" = None
        self._target_lock_timer: float = 0.0

        # Burst attack state (Phase 2).
        # _burst_timer:          seconds remaining in current burst (0 = not bursting).
        # _burst_cooldown_timer: seconds until next burst allowed (0 = ready).
        self._burst_timer:          float = 0.0
        self._burst_cooldown_timer: float = 0.0

        # Ecology: feeding state (Phase 8.6 / 8.7)
        self.food_counter: float = 0.0   # sardines eaten; carries toward future use
        # Every new sailfish — startup, manually added, or reproduced — starts
        # with hunger_timer = 0.  This is equivalent to "just eaten" and
        # guarantees no spawn path can produce an immediately-starving fish
        # regardless of what sailfish_starvation_time is saved in the config.
        self.hunger_timer: float = 0.0

    # ── Burst speed override ───────────────────────────────────────────

    def update(self, dt: float) -> None:
        """Apply burst speed/force caps while bursting, then delegate to Boid.update."""
        cfg = self._config
        if self._burst_timer > 0.0:
            self.max_speed = cfg.sailfish_max_speed * cfg.sailfish_burst_speed_mult
            self.max_force = cfg.sailfish_max_force * cfg.sailfish_burst_force_mult
        else:
            # Restore base values so they reflect any live config changes.
            self.max_speed = cfg.sailfish_max_speed
            self.max_force = cfg.sailfish_max_force
        super().update(dt)

    # ── Force computation ──────────────────────────────────────────────

    def _compute_forces(self) -> None:
        """Apply pursuit and sailfish-to-sailfish flocking forces.

        Hunting algorithm (two-stage)
        ------------------------------
        **Long range** — when no individual target is locked, query the sardine
        grid and steer toward the sardine school centre of mass (COM).

        **Close range** — when a sardine enters
        ``sailfish_target_acquire_radius``, lock onto the nearest one and
        switch to predictive interception: steer toward

            predicted_pos = target.pos + target.vel * t

        where ``t = min(dist / max_speed, 0.5)`` is a capped look-ahead.
        This steers slightly *ahead* of the prey rather than directly at its
        current position, giving smooth, realistic interception curves.

        The lock is released when:
          * The target sardine is no longer alive (eaten or starved).
          * The target drifts beyond ``sailfish_target_drop_radius``.
          * ``_target_lock_timer`` reaches 0 (decremented by simulation.py).

        Sailfish flocking
        -----------------
        Runs after pursuit; unchanged from the previous implementation.
        """
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        _pos = self.position
        px   = float(_pos[0])
        py   = float(_pos[1])

        # ── Individual target lock validation ─────────────────────────
        tgt = self._target_sardine
        if tgt is not None:
            # Drop if sardine was eaten/starved or lock timer expired.
            if not tgt.is_alive or self._target_lock_timer <= 0.0:
                tgt = None
                self._target_sardine = None
            else:
                # Drop if target has drifted beyond the drop radius (toroidal).
                dx = tgt._gx - px
                dy = tgt._gy - py
                if dx >  hw: dx -= W
                elif dx < -hw: dx += W
                if dy >  hh: dy -= H
                elif dy < -hh: dy += H
                drop_r = cfg.sailfish_target_drop_radius
                if dx * dx + dy * dy > drop_r * drop_r:
                    tgt = None
                    self._target_sardine = None

        # ── Pursuit: individual target or school COM ───────────────────
        if tgt is not None:
            # ── Predictive interception toward locked individual sardine ──
            dx = tgt._gx - px
            dy = tgt._gy - py
            if dx >  hw: dx -= W
            elif dx < -hw: dx += W
            if dy >  hh: dy -= H
            elif dy < -hh: dy += H
            dist = math.sqrt(dx * dx + dy * dy)
            # Burst trigger: target is close, burst not active, and not cooling down.
            if (dist < cfg.sailfish_burst_trigger_radius
                    and self._burst_timer <= 0.0
                    and self._burst_cooldown_timer <= 0.0):
                self._burst_timer = cfg.sailfish_burst_duration
            if dist > 1.0:
                t   = min(dist / self.max_speed, 0.5)   # cap look-ahead at 0.5 s
                pdx = dx + tgt._gvx * t
                pdy = dy + tgt._gvy * t
            else:
                pdx, pdy = dx, dy
            self._steer_apply_xy(pdx, pdy, cfg.weight_target_pursuit)

        else:
            # ── No locked target: query sardines for acquisition or COM ──
            sardines = self._sardine_grid.query(self, cfg.radius_sailfish_awareness)
            if sardines:
                # Single pass: accumulate COM and find nearest within acq_r.
                acq_r2  = cfg.sailfish_target_acquire_radius ** 2
                best_d2 = 1e18
                best_s  = None
                alt_d2  = 1e18
                alt_s   = None
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
                    d2 = dx * dx + dy * dy
                    if d2 < acq_r2:
                        if d2 < best_d2:
                            alt_d2, alt_s = best_d2, best_s
                            best_d2, best_s = d2, s
                        elif d2 < alt_d2:
                            alt_d2, alt_s = d2, s

                if best_s is not None:
                    # Shared-target reduction: soft preference to avoid multiple
                    # sailfish locking the same sardine when an alternative exists.
                    if self._sailfish_grid is not None and alt_s is not None:
                        sf_nb = self._sailfish_grid.query(
                            self, cfg.sailfish_target_acquire_radius
                        )
                        if any(other._target_sardine is best_s for other in sf_nb):
                            best_s = alt_s
                    # Lock onto chosen sardine; apply predictive pursuit.
                    self._target_sardine    = best_s
                    self._target_lock_timer = cfg.sailfish_target_lock_time
                    dx = best_s._gx - px
                    dy = best_s._gy - py
                    if dx >  hw: dx -= W
                    elif dx < -hw: dx += W
                    if dy >  hh: dy -= H
                    elif dy < -hh: dy += H
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist > 1.0:
                        t   = min(dist / self.max_speed, 0.5)
                        pdx = dx + best_s._gvx * t
                        pdy = dy + best_s._gvy * t
                    else:
                        pdx, pdy = dx, dy
                    self._steer_apply_xy(pdx, pdy, cfg.weight_target_pursuit)

                else:
                    # No sardine close enough to lock — steer toward school COM.
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
