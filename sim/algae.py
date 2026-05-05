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
        self._sep_r2:   float = config.radius_algae_separation ** 2
        self._ali_r2:   float = config.radius_algae_alignment  ** 2
        self._coh_r2:   float = config.radius_algae_cohesion   ** 2
        self._coh_span: int   = max(1, math.ceil(config.radius_algae_cohesion / grid.cell_size))

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
        # Cache array ref once, then convert to Python floats for fast native
        # float arithmetic in the neighbour loop (same rationale as sardine.py).
        _pos = self.position
        px   = float(_pos[0])
        py   = float(_pos[1])

        # ── Flocking rules — inline cell traversal ─────────────────────────
        # Same optimisation as sardine._compute_forces(): walk the spatial grid
        # cells directly so we avoid allocating a neighbour list and avoid a
        # second pass that recomputes dx/dy/d_sq for every accepted neighbour.
        # Slow path (tiny worlds / tests) falls back to query() + loop.
        sep_r2 = self._sep_r2
        ali_r2 = self._ali_r2
        coh_r2 = self._coh_r2

        # Float-pair accumulators — avoids np.zeros(2) allocations.
        sx = sy = 0.0    # separation: weighted unit away-vectors
        alx = aly = 0.0  # alignment:  sum of neighbour velocities
        cx = cy = 0.0    # cohesion:   vector toward group centre
        sep_count = ali_count = coh_count = 0

        _grid  = self._grid
        _cells = _grid._cells
        _cs    = _grid.cell_size
        _nc    = _grid._num_cols
        _nr    = _grid._num_rows
        _span  = self._coh_span
        _col_c = int(px // _cs)
        _row_c = int(py // _cs)

        if _nc > 2 * _span and _nr > 2 * _span:
            for _dc in range(-_span, _span + 1):
                _col = (_col_c + _dc) % _nc
                for _dr in range(-_span, _span + 1):
                    _cell = _cells.get((_col, (_row_c + _dr) % _nr))
                    if _cell is None:
                        continue
                    for other in _cell:
                        if other is self:
                            continue
                        dx = px - other._gx    # pre-cached Python float from insert()
                        dy = py - other._gy
                        # Inline toroidal wrap.
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        d_sq = dx * dx + dy * dy
                        if d_sq > coh_r2:
                            continue

                        # ── Separation ────────────────────────────────────────
                        if d_sq < sep_r2:
                            if d_sq > _EPS2:
                                inv_d = 1.0 / math.sqrt(d_sq)
                                sx += dx * inv_d
                                sy += dy * inv_d
                            sep_count += 1

                        # ── Alignment ─────────────────────────────────────────
                        if d_sq < ali_r2:
                            alx += other._gvx   # pre-cached velocity from insert()
                            aly += other._gvy
                            ali_count += 1

                        # ── Cohesion ──────────────────────────────────────────
                        cx -= dx
                        cy -= dy
                        coh_count += 1
        else:
            # Slow path (tiny worlds / tests) — use query() for correctness.
            for other in _grid.query(self, cfg.radius_algae_cohesion):
                dx = px - other._gx
                dy = py - other._gy
                if dx > hw:    dx -= W
                elif dx < -hw: dx += W
                if dy > hh:    dy -= H
                elif dy < -hh: dy += H
                d_sq = dx * dx + dy * dy

                if d_sq < sep_r2:
                    if d_sq > _EPS2:
                        inv_d = 1.0 / math.sqrt(d_sq)
                        sx += dx * inv_d
                        sy += dy * inv_d
                    sep_count += 1
                if d_sq < ali_r2:
                    alx += other._gvx
                    aly += other._gvy
                    ali_count += 1
                cx -= dx
                cy -= dy
                coh_count += 1

        if coh_count:
            # Inline Reynolds steer for all three flocking directions.
            # Cache velocity / force limits once; reuse across all directions.
            _vel = self.velocity
            _vx  = float(_vel[0])
            _vy  = float(_vel[1])
            _ms  = self.max_speed
            _mf  = self.max_force
            _acc = self.acceleration

            if sep_count:
                _mag = math.sqrt(sx * sx + sy * sy)
                if _mag > 1e-9:
                    _inv = _ms / _mag
                    _stx = sx * _inv - _vx
                    _sty = sy * _inv - _vy
                    _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                    if _fm > _mf:
                        _k = _mf / _fm
                        _stx *= _k
                        _sty *= _k
                    _w = cfg.weight_algae_separation
                    _acc[0] += _stx * _w
                    _acc[1] += _sty * _w

            if ali_count:
                _mag = math.sqrt(alx * alx + aly * aly)
                if _mag > 1e-9:
                    _inv = _ms / _mag
                    _stx = alx * _inv - _vx
                    _sty = aly * _inv - _vy
                    _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                    if _fm > _mf:
                        _k = _mf / _fm
                        _stx *= _k
                        _sty *= _k
                    _w = cfg.weight_algae_alignment
                    _acc[0] += _stx * _w
                    _acc[1] += _sty * _w

            _mag = math.sqrt(cx * cx + cy * cy)
            if _mag > 1e-9:
                _inv = _ms / _mag
                _stx = cx * _inv - _vx
                _sty = cy * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_algae_cohesion
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w

        # ── Noise: fires even without neighbours ────────────────────────────
        # random.gauss(0, 1) returns a Python float — zero allocation.
        if cfg.weight_algae_noise > 0.0:
            _nx = random.gauss(0.0, 1.0)
            _ny = random.gauss(0.0, 1.0)
            _mag = math.sqrt(_nx * _nx + _ny * _ny)
            if _mag > 1e-9:
                _vel = self.velocity
                _vx  = float(_vel[0])
                _vy  = float(_vel[1])
                _ms  = self.max_speed
                _mf  = self.max_force
                _inv = _ms / _mag
                _stx = _nx * _inv - _vx
                _sty = _ny * _inv - _vy
                _fm  = math.sqrt(_stx * _stx + _sty * _sty)
                if _fm > _mf:
                    _k = _mf / _fm
                    _stx *= _k
                    _sty *= _k
                _w = cfg.weight_algae_noise
                _acc = self.acceleration
                _acc[0] += _stx * _w
                _acc[1] += _sty * _w
