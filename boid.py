"""
boid.py
=======
Base ``Boid`` class shared by all agents (sardines, sailfish, etc.).

Responsibilities
----------------
* Store position, velocity, acceleration as numpy float64 arrays.
* Apply steering forces up to ``max_force``.
* Advance state one timestep with Euler integration (delta-time based).
* Enforce world-boundary rule (wrap or clamp — configured per instance).
* Expose helper properties (speed, heading) used by subclasses / renderer.

This class does NOT implement flocking rules — those live in subclasses
(Sardine, Sailfish) so each agent type controls its own behaviour.
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

from config import SimConfig
from utils.vec2 import Vec2, limit, normalize, wrap_position

# Epsilon used in _steer_apply_xy to guard against degenerate zero direction.
_EPS_F: float = 1e-9


class Boid:
    """Abstract-ish base for all simulation agents.

    Parameters
    ----------
    x, y:
        Initial position in world coordinates (pixels).
    vx, vy:
        Initial velocity components (pixels/second).
    max_speed:
        Maximum speed cap (pixels/second).
    max_force:
        Maximum steering-force magnitude (pixels/second²).
    config:
        Simulation configuration for world dimensions & boundary rule.
    min_speed:
        Minimum speed floor (pixels/second).  Defaults to ``0.0`` (no
        floor).  Set this for species that must not stall (e.g. sardines).
    """

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        max_speed: float,
        max_force: float,
        config: SimConfig,
        min_speed: float = 0.0,
    ) -> None:
        self.position: Vec2     = np.array([x, y],   dtype=np.float64)
        self.velocity: Vec2     = np.array([vx, vy], dtype=np.float64)
        self.acceleration: Vec2 = np.zeros(2,         dtype=np.float64)
        self.max_speed: float   = max_speed
        self.min_speed: float   = min_speed
        self.max_force: float   = max_force
        self._config: SimConfig = config

    # ── Public helpers ────────────────────────────────────────────────

    @property
    def speed(self) -> float:
        """Current scalar speed (pixels/second)."""
        v = self.velocity
        return math.sqrt(v[0]*v[0] + v[1]*v[1])

    @property
    def heading(self) -> Vec2:
        """Unit vector in the direction of current velocity."""
        return normalize(self.velocity, fallback=np.array([1.0, 0.0]))

    # ── Steering helpers (called by subclasses) ───────────────────────

    def apply_force(self, force: Vec2) -> None:
        """Accumulate a steering force into acceleration this frame."""
        self.acceleration += force

    def steer_toward(self, target_velocity: Vec2) -> Vec2:
        """Return Reynolds steering force toward *target_velocity*.

        Clamps desired velocity to ``max_speed``, then subtracts current
        velocity for a smooth, force-limited turn.
        """
        desired = normalize(target_velocity) * self.max_speed
        steer   = desired - self.velocity
        return limit(steer, self.max_force)

    def _steer_apply_xy(self, tx: float, ty: float, weight: float) -> None:
        """Compute steer_toward((tx, ty)) and add weight * result to acceleration.

        Zero-allocation fast path used by Sardine and Sailfish hot loops.
        Equivalent to ``apply_force(steer_toward([tx,ty]) * weight)`` but
        avoids all intermediate numpy array creation.
        """
        mag = math.sqrt(tx * tx + ty * ty)
        if mag < _EPS_F:
            return
        inv_mag = self.max_speed / mag
        sx = tx * inv_mag - float(self.velocity[0])
        sy = ty * inv_mag - float(self.velocity[1])
        fmag = math.sqrt(sx * sx + sy * sy)
        if fmag > self.max_force:
            inv_fmag = self.max_force / fmag
            sx *= inv_fmag
            sy *= inv_fmag
        self.acceleration[0] += sx * weight
        self.acceleration[1] += sy * weight

    # ── Update ────────────────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """Advance position and velocity by one timestep *dt* (seconds).

        Called once per frame by ``Simulation``.  Subclasses control their
        own steering by overriding ``_compute_forces()``, which is invoked
        here before integration.
        """
        self._compute_forces()

        # Euler integration
        self.velocity  += self.acceleration * dt
        self._clamp_speed()
        self.position  += self.velocity * dt

        # Reset acceleration every frame
        self.acceleration[:] = 0.0

        self._apply_boundary()

    def _compute_forces(self) -> None:
        """Override in subclasses to add steering forces each frame.

        Default: no forces.
        """

    # ── Internals ─────────────────────────────────────────────────────

    def _clamp_speed(self) -> None:
        """Clamp velocity magnitude to ``[min_speed, max_speed]``."""
        self.velocity = limit(self.velocity, self.max_speed)
        if self.min_speed > 0.0:
            v   = self.velocity
            spd = math.sqrt(v[0]*v[0] + v[1]*v[1])
            if spd < self.min_speed:
                # Preserve heading; fall back to +x if nearly stationary.
                self.velocity = (
                    normalize(self.velocity, fallback=np.array([1.0, 0.0]))
                    * self.min_speed
                )

    def _apply_boundary(self) -> None:
        """Enforce world boundaries according to config.boundary."""
        W = float(self._config.window_width)
        H = float(self._config.window_height)

        if self._config.boundary == "wrap":
            wrap_position(self.position, W, H)
        else:
            # "clamp" — reflect off edges
            if self.position[0] < 0.0:
                self.position[0] = 0.0
                self.velocity[0] *= -1.0
            elif self.position[0] > W:
                self.position[0] = W
                self.velocity[0] *= -1.0
            if self.position[1] < 0.0:
                self.position[1] = 0.0
                self.velocity[1] *= -1.0
            elif self.position[1] > H:
                self.position[1] = H
                self.velocity[1] *= -1.0
