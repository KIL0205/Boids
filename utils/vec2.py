"""
utils/vec2.py
=============
Pure numpy 2-D vector helpers used throughout the simulation.

All functions operate on ``numpy.ndarray`` of shape ``(2,)`` with dtype
``float64``.  They are free functions (no class) so they can be called with
minimal overhead from inner loops.

Public API
----------
normalize(v, fallback)  — safe unit vector
limit(v, max_mag)       — clamp magnitude
distance(a, b)          — Euclidean distance
distance_sq(a, b)       — squared distance (cheaper for radius comparisons)
wrap_delta(delta, size) — shortest signed distance across a toroidal axis
wrap_position(pos, w, h)— fold a position back into [0, w) x [0, h)
"""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

# Type alias used throughout the project.
Vec2 = npt.NDArray[np.float64]

# A small epsilon used to guard against division by near-zero vectors.
_EPS: float = 1e-9


# ── Core helpers ──────────────────────────────────────────────────────


def normalize(v: Vec2, fallback: Vec2 | None = None) -> Vec2:
    """Return the unit vector of *v*.

    Parameters
    ----------
    v:
        Input 2-D vector.
    fallback:
        Value returned when *v* has near-zero magnitude.  Defaults to the
        zero vector ``[0, 0]``.

    Returns
    -------
    Vec2
        Unit vector, or *fallback* if ``|v| < 1e-9``.

    Examples
    --------
    >>> normalize(np.array([3.0, 4.0]))
    array([0.6, 0.8])
    >>> normalize(np.zeros(2))
    array([0., 0.])
    """
    mag = math.sqrt(v[0]*v[0] + v[1]*v[1])
    if mag < _EPS:
        return fallback.copy() if fallback is not None else np.zeros(2, dtype=np.float64)
    return v / mag


def limit(v: Vec2, max_mag: float) -> Vec2:
    """Return *v* scaled so its magnitude does not exceed *max_mag*.

    If ``|v| <= max_mag`` the original vector is returned unchanged
    (same object, no copy).

    Parameters
    ----------
    v:
        Input 2-D vector.
    max_mag:
        Maximum allowed magnitude.  Must be non-negative.

    Returns
    -------
    Vec2
        Vector clamped to *max_mag*, or the original if already within bounds.

    Examples
    --------
    >>> limit(np.array([3.0, 4.0]), 2.5)  # magnitude 5 → 2.5
    array([1.5, 2. ])
    """
    mag = math.sqrt(v[0]*v[0] + v[1]*v[1])
    if mag > max_mag and mag > _EPS:
        return v * (max_mag / mag)
    return v


def distance(a: Vec2, b: Vec2) -> float:
    """Return the Euclidean distance between points *a* and *b*.

    Parameters
    ----------
    a, b:
        2-D position vectors.

    Returns
    -------
    float
        ``|a - b|``
    """
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return math.sqrt(dx * dx + dy * dy)


def distance_sq(a: Vec2, b: Vec2) -> float:
    """Return the **squared** Euclidean distance between *a* and *b*.

    Faster than ``distance()`` when you only need to compare against a
    radius threshold (avoids the square root).

    Parameters
    ----------
    a, b:
        2-D position vectors.

    Returns
    -------
    float
        ``|a - b|²``
    """
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return dx * dx + dy * dy


# ── Toroidal / boundary helpers ───────────────────────────────────────


def wrap_delta(delta: float, world_size: float) -> float:
    """Return the shortest signed displacement along one toroidal axis.

    Converts a raw coordinate difference into the minimum-image difference
    for a world that wraps at *world_size*.

    Parameters
    ----------
    delta:
        Raw difference ``(a_coord - b_coord)`` along one axis.
    world_size:
        Length of the world along that axis (pixels).

    Returns
    -------
    float
        Corrected delta in ``(-world_size/2, world_size/2]``.

    Examples
    --------
    >>> wrap_delta(750.0, 800.0)   # agent near right edge, target near left
    -50.0
    """
    half = world_size * 0.5
    if delta > half:
        return delta - world_size
    if delta < -half:
        return delta + world_size
    return delta


def wrap_position(pos: Vec2, world_width: float, world_height: float) -> Vec2:
    """Fold *pos* back into the toroidal world ``[0, world_width) × [0, world_height)``.

    Modifies *pos* **in place** and also returns it for convenience.

    Parameters
    ----------
    pos:
        Mutable 2-D position array.
    world_width, world_height:
        World dimensions (pixels).

    Returns
    -------
    Vec2
        The same array with coordinates wrapped.
    """
    pos[0] %= world_width
    pos[1] %= world_height
    return pos


def toroidal_displacement(a: Vec2, b: Vec2, world_width: float, world_height: float) -> Vec2:
    """Return the shortest-path vector from *b* to *a* in a toroidal world.

    Equivalent to ``a - b`` but with each axis independently wrapped to
    the minimum-image convention.

    Parameters
    ----------
    a, b:
        2-D position vectors.
    world_width, world_height:
        World dimensions (pixels).

    Returns
    -------
    Vec2
        Displacement vector of shape ``(2,)`` pointing from *b* toward *a*.
    """
    dx = wrap_delta(float(a[0]) - float(b[0]), world_width)
    dy = wrap_delta(float(a[1]) - float(b[1]), world_height)
    return np.array([dx, dy], dtype=np.float64)
