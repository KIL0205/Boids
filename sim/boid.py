"""
sim/boid.py
===========
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

from config.config import SimConfig
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
        # Cached world dimensions — avoids re-reading config on every boundary
        # check.  Updated by Simulation.resize() and Simulation.apply_config().
        self._W: float = float(config.window_width)
        self._H: float = float(config.window_height)
        # Pre-computed boundary mode flag: avoids an attribute chain + string
        # comparison on every agent every frame.  Updated by apply_config().
        self._boundary_wrap: bool = (config.boundary == "wrap")
        # Grid-cached position and velocity — written by SpatialHash.insert()
        # once per agent per frame (during grid rebuild, before any agent
        # update runs).  Storing Python floats here lets ALL downstream
        # neighbour-loop arithmetic stay in CPython's native float path instead
        # of going through numpy's scalar type-dispatch machinery on every
        # opos[0] / ovel[0] access.  Query inner loops and species
        # _compute_forces() inner loops all read these instead of accessing
        # other.position[0] / other.velocity[0] directly.
        self._gx:  float = 0.0   # position x  (set by insert() each frame)
        self._gy:  float = 0.0   # position y
        self._gvx: float = 0.0   # velocity vx
        self._gvy: float = 0.0   # velocity vy
        # Batch-array index — set by Simulation at the start of each frame
        # during the Phase 4A array-populate pass.  Used by _batch_integrate_*
        # to address this agent's row in the shared position/velocity arrays.
        # Default -1 means "not yet assigned".
        self._idx:  int  = -1

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

        Attribute caching
        -----------------
        ``self.velocity``, ``self.max_force``, and ``self.acceleration`` are
        each read once into a local name.  This eliminates 2–3 repeated
        LOAD_ATTR + array-index ops per call that occurred when the same
        attribute was read twice (before vs. after the magnitude clamp).
        With 3–5 calls per sardine per frame, the savings compound quickly.
        """
        mag = math.sqrt(tx * tx + ty * ty)
        if mag < _EPS_F:
            return
        vel     = self.velocity           # one LOAD_ATTR covers both vel[0] + vel[1]
        max_f   = self.max_force          # cache: avoids double-lookup in clamp branch
        inv_mag = self.max_speed / mag
        # float() converts numpy array elements to Python floats so all subsequent
        # arithmetic (sx*sx, fmag comparison, sx*weight) runs in CPython's native
        # float path instead of numpy's scalar dispatch machinery (2–4× faster).
        # Caching vel= above still saves one LOAD_ATTR vs reading self.velocity twice.
        sx = tx * inv_mag - float(vel[0])
        sy = ty * inv_mag - float(vel[1])
        fmag = math.sqrt(sx * sx + sy * sy)
        if fmag > max_f:
            inv_fmag = max_f / fmag
            sx *= inv_fmag
            sy *= inv_fmag
        acc     = self.acceleration       # one LOAD_ATTR covers both acc[0] + acc[1]
        acc[0] += sx * weight
        acc[1] += sy * weight

    # ── Update ────────────────────────────────────────────────────────

    def update(self, dt: float) -> None:
        """Advance position and velocity by one timestep *dt* (seconds).

        Called once per frame by ``Simulation``.  Subclasses control their
        own steering by overriding ``_compute_forces()``, which is invoked
        here before integration.
        """
        self._compute_forces()

        # Euler integration — element-wise scalar ops avoid two temporary
        # (2,) float64 array allocations that `v += a*dt` / `p += v*dt`
        # would create.  At 2 000 agents × 60 fps that saves ~240 000 small
        # heap allocations per second with no change in numerical behaviour.
        acc = self.acceleration
        vel = self.velocity
        vel[0] += acc[0] * dt
        vel[1] += acc[1] * dt
        self._clamp_speed()
        pos = self.position
        pos[0] += vel[0] * dt
        pos[1] += vel[1] * dt

        # Reset acceleration in-place (element assignment, no alloc).
        acc[0] = 0.0
        acc[1] = 0.0

        self._apply_boundary()

    def _compute_forces(self) -> None:
        """Override in subclasses to add steering forces each frame.

        Default: no forces.
        """

    # ── Internals ─────────────────────────────────────────────────────

    def _clamp_speed(self) -> None:
        """Clamp velocity magnitude to [min_speed, max_speed] in one pass.

        Computes the magnitude once and adjusts in-place when needed,
        avoiding both the double-sqrt that the previous limit() + min-speed
        check required and the temporary numpy array that limit() allocated
        whenever clamping was triggered.
        """
        v    = self.velocity
        vx   = float(v[0])
        vy   = float(v[1])
        spd  = math.sqrt(vx * vx + vy * vy)
        maxs = self.max_speed
        mins = self.min_speed
        if spd > maxs and spd > _EPS_F:
            scale = maxs / spd
            v[0]  = vx * scale
            v[1]  = vy * scale
        elif mins > 0.0 and spd < mins:
            if spd < _EPS_F:
                # Degenerate zero velocity — fall back to +x heading.
                v[0] = mins
                v[1] = 0.0
            else:
                scale = mins / spd
                v[0]  = vx * scale
                v[1]  = vy * scale

    def _apply_boundary(self) -> None:
        """Enforce world boundaries according to config.boundary."""
        W = self._W
        H = self._H

        if self._boundary_wrap:
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
