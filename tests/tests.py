"""
tests/tests.py
==============
Unit and integration tests for the sardine/sailfish boid simulation.

Run with:
    python tests/tests.py
        or
    python -m pytest tests/

No external test dependencies — uses the standard-library ``unittest`` module.
"""

import unittest
import math

import numpy as np

from config.config import CONFIG, SimConfig
from utils.vec2 import (
    distance,
    distance_sq,
    limit,
    normalize,
    toroidal_displacement,
    wrap_delta,
    wrap_position,
)
from sim.boid import Boid
from sim.algae import Algae
from sim.spatial_hash import SpatialHash
from sim.simulation import Simulation


# ── Helpers ───────────────────────────────────────────────────────────────────

def _boid_at(x: float, y: float, vx: float = 1.0, vy: float = 0.0,
             max_speed: float = 100.0, max_force: float = 50.0,
             min_speed: float = 0.0, cfg: SimConfig = CONFIG) -> Boid:
    return Boid(x, y, vx, vy, max_speed, max_force, cfg, min_speed=min_speed)


# ── vec2 helpers ──────────────────────────────────────────────────────────────

class TestVec2(unittest.TestCase):
    """Tests for utils/vec2.py."""

    def test_normalize_unit_length(self):
        u = normalize(np.array([3.0, 4.0]))
        self.assertAlmostEqual(float(np.linalg.norm(u)), 1.0)

    def test_normalize_direction(self):
        u = normalize(np.array([3.0, 4.0]))
        self.assertAlmostEqual(float(u[0]), 0.6)
        self.assertAlmostEqual(float(u[1]), 0.8)

    def test_normalize_zero_returns_zeros(self):
        u = normalize(np.zeros(2))
        np.testing.assert_array_equal(u, np.zeros(2))

    def test_normalize_zero_custom_fallback(self):
        fallback = np.array([1.0, 0.0])
        u = normalize(np.zeros(2), fallback=fallback)
        np.testing.assert_array_equal(u, fallback)

    def test_limit_above_max_clamps_magnitude(self):
        v = np.array([3.0, 4.0])          # magnitude 5
        lv = limit(v, 2.5)
        self.assertAlmostEqual(float(np.linalg.norm(lv)), 2.5)

    def test_limit_at_max_unchanged(self):
        v = np.array([3.0, 4.0])
        np.testing.assert_array_almost_equal(limit(v, 5.0), v)

    def test_limit_below_max_unchanged(self):
        v = np.array([1.0, 0.0])
        np.testing.assert_array_almost_equal(limit(v, 10.0), v)

    def test_distance(self):
        self.assertAlmostEqual(
            distance(np.array([0.0, 0.0]), np.array([3.0, 4.0])), 5.0
        )

    def test_distance_sq(self):
        self.assertAlmostEqual(
            distance_sq(np.array([0.0, 0.0]), np.array([3.0, 4.0])), 25.0
        )

    def test_wrap_delta_no_wrap_needed(self):
        self.assertAlmostEqual(wrap_delta(100.0, 800.0), 100.0)

    def test_wrap_delta_positive_overshoot(self):
        # 750 > 400 (half of 800) → wrapped to −50
        self.assertAlmostEqual(wrap_delta(750.0, 800.0), -50.0)

    def test_wrap_delta_negative_overshoot(self):
        # −750 < −400 → wrapped to +50
        self.assertAlmostEqual(wrap_delta(-750.0, 800.0), 50.0)

    def test_wrap_delta_at_positive_half(self):
        # Exactly at half should return +half, not wrapped negatively
        self.assertAlmostEqual(wrap_delta(400.0, 800.0), 400.0)

    def test_wrap_position_in_bounds_unchanged(self):
        pos = np.array([100.0, 200.0])
        result = wrap_position(pos.copy(), 1400.0, 700.0)
        np.testing.assert_array_equal(result, np.array([100.0, 200.0]))

    def test_wrap_position_x_overflow(self):
        pos = np.array([1500.0, 300.0])
        wrap_position(pos, 1400.0, 700.0)
        self.assertAlmostEqual(pos[0], 100.0)

    def test_wrap_position_y_underflow(self):
        pos = np.array([100.0, -50.0])
        wrap_position(pos, 1400.0, 700.0)
        self.assertAlmostEqual(pos[1], 650.0)

    def test_toroidal_displacement_no_wrap(self):
        a = np.array([200.0, 300.0])
        b = np.array([100.0, 100.0])
        d = toroidal_displacement(a, b, 1400.0, 700.0)
        np.testing.assert_array_almost_equal(d, np.array([100.0, 200.0]))

    def test_toroidal_displacement_wraps_x(self):
        # a near right edge, b near left edge — shortest path wraps
        a = np.array([1390.0, 350.0])
        b = np.array([10.0,   350.0])
        d = toroidal_displacement(a, b, 1400.0, 700.0)
        # Toroidal distance is 20px (1390→1400→10 = 20), not 1380px
        self.assertAlmostEqual(float(np.linalg.norm(d)), 20.0)


# ── Boid kinematics ───────────────────────────────────────────────────────────

class TestBoidKinematics(unittest.TestCase):
    """Tests for Boid base class: force accumulation, speed clamping, integration."""

    def test_speed_property(self):
        b = _boid_at(0, 0, vx=30.0, vy=40.0)
        self.assertAlmostEqual(b.speed, 50.0)

    def test_heading_is_unit_vector(self):
        b = _boid_at(0, 0, vx=3.0, vy=4.0)
        self.assertAlmostEqual(float(np.linalg.norm(b.heading)), 1.0)

    def test_max_speed_clamped_after_update(self):
        b = _boid_at(100, 100, vx=0, vy=0, max_speed=50.0, max_force=9999.0)
        b.apply_force(np.array([9999.0, 0.0]))
        b.update(1 / 60)
        self.assertLessEqual(b.speed, 50.0 + 1e-9)

    def test_min_speed_floor_enforced(self):
        b = _boid_at(100, 100, vx=60.0, vy=0, max_speed=100.0,
                     max_force=9999.0, min_speed=20.0)
        b.apply_force(np.array([-9999.0, 0.0]))
        b.update(1 / 60)
        self.assertGreaterEqual(b.speed, 20.0 - 1e-9)

    def test_steer_toward_clamped_to_max_force(self):
        b = _boid_at(0, 0, vx=0, vy=0, max_speed=50.0, max_force=30.0)
        force = b.steer_toward(np.array([9999.0, 0.0]))
        self.assertLessEqual(float(np.linalg.norm(force)), 30.0 + 1e-9)

    def test_apply_force_accumulates(self):
        b = _boid_at(0, 0)
        b.apply_force(np.array([1.0, 0.0]))
        b.apply_force(np.array([2.0, 3.0]))
        np.testing.assert_array_almost_equal(b.acceleration, np.array([3.0, 3.0]))

    def test_acceleration_zeroed_after_update(self):
        b = _boid_at(100, 100)
        b.apply_force(np.array([10.0, 5.0]))
        b.update(1 / 60)
        np.testing.assert_array_almost_equal(b.acceleration, np.zeros(2))

    def test_boundary_wrap(self):
        # Position just outside the right edge should fold back in
        cfg = SimConfig(window_width=200, window_height=100, boundary="wrap")
        b = Boid(195.0, 50.0, 20.0, 0.0, 100.0, 50.0, cfg)
        b.update(1.0)        # large dt: crosses boundary
        self.assertGreaterEqual(float(b.position[0]), 0.0)
        self.assertLess(float(b.position[0]), 200.0)


# ── Spatial hash ──────────────────────────────────────────────────────────────

class TestSpatialHash(unittest.TestCase):
    """Tests for SpatialHash insert / query / clear / toroidal correctness."""

    W, H, CS = 1400.0, 700.0, 80.0

    def _grid(self) -> SpatialHash:
        return SpatialHash(cell_size=self.CS, world_width=self.W,
                           world_height=self.H)

    def test_hit_within_radius(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(130.0, 100.0)   # 30 px away
        g.insert(a); g.insert(b)
        self.assertIn(b, g.query(a, radius=50.0))

    def test_miss_beyond_radius(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(300.0, 100.0)   # 200 px away
        g.insert(a); g.insert(b)
        self.assertNotIn(b, g.query(a, radius=50.0))

    def test_query_excludes_self(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        g.insert(a)
        self.assertNotIn(a, g.query(a, radius=50.0))

    def test_hit_exactly_on_radius_boundary(self):
        # Neighbor exactly at radius should be included (≤ check)
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(150.0, 100.0)   # exactly 50 px away
        g.insert(a); g.insert(b)
        self.assertIn(b, g.query(a, radius=50.0))

    def test_clear_removes_all(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(130.0, 100.0)
        g.insert(a); g.insert(b)
        g.clear()
        self.assertEqual(g.query(a, radius=50.0), [])

    def test_multiple_neighbors_all_returned(self):
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        close = [_boid_at(200.0 + i * 5, 200.0) for i in range(1, 6)]
        g.insert(a)
        for n in close:
            g.insert(n)
        result = g.query(a, radius=60.0)
        for n in close:
            self.assertIn(n, result)

    def test_toroidal_x_wrap(self):
        """Agent near right edge sees neighbor near left edge across the wrap."""
        g = self._grid()
        # 20 px apart toroidally along x (1390 → 1400 → 0 → 10 = 20 px)
        a = _boid_at(self.W - 10.0, 350.0)
        b = _boid_at(10.0,          350.0)
        g.insert(a); g.insert(b)
        result = g.query(a, radius=50.0)
        self.assertIn(b, result,
            "Toroidal x-wrap: neighbor across right→left boundary not found")

    def test_toroidal_y_wrap(self):
        """Agent near bottom edge sees neighbor near top edge across the wrap."""
        g = self._grid()
        a = _boid_at(700.0, self.H - 10.0)
        b = _boid_at(700.0, 10.0)
        g.insert(a); g.insert(b)
        result = g.query(a, radius=50.0)
        self.assertIn(b, result,
            "Toroidal y-wrap: neighbor across bottom→top boundary not found")

    def test_no_duplicates_in_result(self):
        # An agent should appear at most once even when spanning multiple cells
        g = self._grid()
        a = _boid_at(80.0, 80.0)   # on a cell boundary
        b = _boid_at(81.0, 80.0)
        g.insert(a); g.insert(b)
        result = g.query(a, radius=50.0)
        # 'b' should appear exactly once, not per-cell
        self.assertEqual(result.count(b), 1)

    def test_query_position_hit(self):
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        g.insert(a)
        result = g.query_position(220.0, 200.0, radius=50.0)
        self.assertIn(a, result)

    def test_query_position_miss(self):
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        g.insert(a)
        result = g.query_position(500.0, 500.0, radius=50.0)
        self.assertNotIn(a, result)


# ── Simulation integration ────────────────────────────────────────────────────

class TestSimulationIntegrity(unittest.TestCase):
    """Integration tests: spawn, 60-frame stability, diagnostics."""

    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(CONFIG)
        for _ in range(60):
            cls.sim.update(1 / 60)   # run 1 simulated second

    def test_sardine_count(self):
        # Sardine count may decrease due to predation (Stage 8.6) and increase
        # due to reproduction (Stage 8.4).  Only assert it is positive.
        self.assertGreater(len(self.sim.sardines), 0)

    def test_all_speeds_in_bounds(self):
        lo, hi = CONFIG.sardine_min_speed - 1e-6, CONFIG.sardine_max_speed + 1e-6
        for s in self.sim.sardines:
            self.assertGreaterEqual(s.speed, lo)
            self.assertLessEqual(s.speed, hi)

    def test_all_positions_in_world(self):
        W = float(CONFIG.window_width)
        H = float(CONFIG.window_height)
        for s in self.sim.sardines:
            self.assertGreaterEqual(s.position[0], 0.0)
            self.assertLess(s.position[0], W)
            self.assertGreaterEqual(s.position[1], 0.0)
            self.assertLess(s.position[1], H)

    def test_dt_spike_safety(self):
        # A 2-second dt must be clamped — no explosion or out-of-bounds
        cfg = SimConfig(num_sardines=10)
        sim = Simulation(cfg)
        sim.update(2.0)
        W, H = float(cfg.window_width), float(cfg.window_height)
        for s in sim.sardines:
            self.assertLessEqual(s.speed, cfg.sardine_max_speed + 1e-6)
            self.assertGreaterEqual(s.position[0], 0.0)
            self.assertLess(s.position[0], W)

    # ── diagnostics() ─────────────────────────────────────────────────

    def test_diagnostics_has_required_keys(self):
        d = self.sim.diagnostics()
        for key in ("count", "mean_speed", "min_speed", "max_speed", "center_of_mass"):
            self.assertIn(key, d)

    def test_diagnostics_count(self):
        d = self.sim.diagnostics()
        # Population may differ from CONFIG default due to ecology (predation,
        # reproduction); just verify the value equals the live list length.
        self.assertEqual(d["count"], len(self.sim.sardines))

    def test_diagnostics_speed_range_valid(self):
        d = self.sim.diagnostics()
        self.assertGreaterEqual(d["min_speed"], CONFIG.sardine_min_speed - 1e-6)
        self.assertLessEqual(d["max_speed"],    CONFIG.sardine_max_speed + 1e-6)
        self.assertGreater(d["mean_speed"], 0.0)
        self.assertLessEqual(d["min_speed"], d["mean_speed"])
        self.assertLessEqual(d["mean_speed"], d["max_speed"])

    def test_diagnostics_center_of_mass_shape(self):
        d = self.sim.diagnostics()
        com = d["center_of_mass"]
        self.assertEqual(len(com), 2)

    def test_diagnostics_empty_sim(self):
        # Diagnostics on a sim with no sardines should not raise
        cfg = SimConfig(num_sardines=0)
        sim = Simulation(cfg)
        d = sim.diagnostics()
        self.assertEqual(d["count"], 0)


# ── Phase 2: sailfish spawn + sardine flee ────────────────────────────────────

class TestPhase2(unittest.TestCase):
    """Tests for sailfish spawning and sardine predator-awareness / flee."""

    # ── helpers ───────────────────────────────────────────────────────

    def _make_grids(self, cfg: SimConfig):
        sardine_grid = SpatialHash(
            cell_size=cfg.cell_size,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        sailfish_grid = SpatialHash(
            cell_size=cfg.sailfish_cell_size,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        return sardine_grid, sailfish_grid

    # ── sailfish spawn ─────────────────────────────────────────────────

    def test_sailfish_spawn_count(self):
        sim = Simulation(CONFIG)
        self.assertEqual(len(sim.sailfish), CONFIG.num_sailfish)

    def test_sailfish_speed_bounds(self):
        sim = Simulation(CONFIG)
        for _ in range(60):
            sim.update(1 / 60)
        lo = CONFIG.sailfish_min_speed - 1e-6
        hi = CONFIG.sailfish_max_speed + 1e-6
        for sf in sim.sailfish:
            self.assertGreaterEqual(sf.speed, lo)
            self.assertLessEqual(sf.speed, hi)

    def test_sailfish_not_in_sardine_grid(self):
        """Sailfish must never appear in the sardine spatial hash."""
        sim = Simulation(CONFIG)
        sim.update(1 / 60)
        all_in_grid = []
        for cell in sim._grid._cells.values():
            all_in_grid.extend(cell)
        for sf in sim.sailfish:
            self.assertNotIn(sf, all_in_grid)

    # ── sardine flee ───────────────────────────────────────────────────

    def test_sardine_flee_fires(self):
        """Sailfish inside radius_flee → non-zero acceleration on the sardine."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(num_sardines=1, num_sailfish=1,
                        radius_flee=100.0, weight_flee=4.0,
                        sailfish_cell_size=120.0)
        sardine_grid, sailfish_grid = self._make_grids(cfg)

        # Sailfish 50 px to the right — well inside radius_flee=100
        sf = Sailfish(750.0, 350.0, 80.0, 0.0, cfg, sardine_grid)
        sailfish_grid.insert(sf)

        s = Sardine(700.0, 350.0, 50.0, 0.0, cfg, sardine_grid, sailfish_grid)
        s._compute_forces()

        self.assertGreater(float(np.linalg.norm(s.acceleration)), 0.0,
                           "Flee force should produce non-zero acceleration")

    def test_sardine_no_flee_without_sailfish_grid(self):
        """sailfish_grid=None → no flee force even when a sailfish would be nearby."""
        from sim.sardine import Sardine

        cfg = SimConfig(num_sardines=1, num_sailfish=0,
                        radius_flee=100.0, weight_flee=4.0,
                        sailfish_cell_size=120.0)
        sardine_grid = SpatialHash(cell_size=cfg.cell_size,
                                   world_width=float(cfg.window_width),
                                   world_height=float(cfg.window_height))

        # No sailfish_grid injected (defaults to None) → no flee block runs
        s = Sardine(700.0, 350.0, 50.0, 0.0, cfg, sardine_grid)
        # No sardine neighbours either → flocking block returns early
        s._compute_forces()
        np.testing.assert_array_equal(s.acceleration, np.zeros(2))

    def test_sardine_cohesion_zeroed_when_fleeing(self):
        """Cohesion is suppressed when a sailfish is within radius_flee.

        Layout (sep/ali/noise zeroed to isolate flee vs cohesion):
          - Sardine A  at (700, 350)
          - Neighbour B at (700, 410)  → 60 px below; cohesion would pull +y
          - Sailfish   at (780, 350)   → 80 px right; flee pushes −x
        If cohesion fires:    accel.y > 0 (toward neighbour)
        If cohesion zeroed:   accel.y ≈ 0, accel.x < 0 (flee only)
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(num_sardines=2, num_sailfish=1,
                        weight_separation=0.0, weight_alignment=0.0,
                        weight_cohesion=1.0, weight_noise=0.0,
                        radius_flee=100.0, weight_flee=4.0,
                        sailfish_cell_size=120.0)
        sardine_grid, sailfish_grid = self._make_grids(cfg)

        neighbor = Sardine(700.0, 410.0, 50.0, 0.0, cfg, sardine_grid)
        sardine_grid.insert(neighbor)
        sf = Sailfish(780.0, 350.0, 80.0, 0.0, cfg, sardine_grid)
        sailfish_grid.insert(sf)

        s = Sardine(700.0, 350.0, 50.0, 0.0, cfg, sardine_grid, sailfish_grid)
        s._compute_forces()

        self.assertLess(float(s.acceleration[0]), 0.0,
                        "Flee should push sardine in −x direction")
        self.assertAlmostEqual(float(s.acceleration[1]), 0.0, places=6,
                               msg="Cohesion (pulls +y) must be zeroed when fleeing")

    def test_sardine_cohesion_fires_when_safe(self):
        """Cohesion fires normally when sailfish is outside radius_flee.

        Same layout but sailfish is 200 px away (> radius_flee=100).
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(num_sardines=2, num_sailfish=1,
                        weight_separation=0.0, weight_alignment=0.0,
                        weight_cohesion=1.0, weight_noise=0.0,
                        radius_flee=100.0, weight_flee=4.0,
                        sailfish_cell_size=120.0)
        sardine_grid, sailfish_grid = self._make_grids(cfg)

        neighbor = Sardine(700.0, 410.0, 50.0, 0.0, cfg, sardine_grid)
        sardine_grid.insert(neighbor)
        # Sailfish 200 px away — outside radius_flee=100
        sf = Sailfish(900.0, 350.0, 80.0, 0.0, cfg, sardine_grid)
        sailfish_grid.insert(sf)

        # Sardine moving in +y so steer_toward(coh_offset=(0,+60)) stays on y-axis
        s = Sardine(700.0, 350.0, 0.0, 50.0, cfg, sardine_grid, sailfish_grid)
        s._compute_forces()

        self.assertGreater(float(s.acceleration[1]), 0.0,
                           "Cohesion should pull sardine toward neighbour (+y)")
        self.assertAlmostEqual(float(s.acceleration[0]), 0.0, places=6,
                               msg="No flee force when sailfish is outside radius_flee")


# ── Phase 8: algae entity ─────────────────────────────────────────────────────

def _algae_grid(cfg: SimConfig = CONFIG) -> "SpatialHash":
    """Return a fresh SpatialHash sized for algae using *cfg*."""
    return SpatialHash(
        cell_size=cfg.algae_cell_size,
        world_width=float(cfg.window_width),
        world_height=float(cfg.window_height),
    )


def _algae_at(
    x: float,
    y: float,
    vx: float = 5.0,
    vy: float = 0.0,
    cfg: SimConfig = CONFIG,
) -> "Algae":
    """Convenience factory: return an Algae boid with its own fresh grid."""
    return Algae(x, y, vx, vy, cfg, _algae_grid(cfg))


class TestAlgaeKinematics(unittest.TestCase):
    """Tests for Algae: creation, movement, boundary, grid, and simulation integration."""

    # ── Basic kinematics ──────────────────────────────────────────────

    def test_algae_moves_after_update(self):
        """position changes when algae has non-zero velocity."""
        a = _algae_at(100.0, 200.0, vx=10.0, vy=0.0)
        pos_before = a.position.copy()
        a.update(1 / 60)
        self.assertFalse(
            np.allclose(a.position, pos_before),
            "Algae with non-zero velocity should move after update",
        )

    def test_algae_respects_max_speed(self):
        """Algae speed never exceeds algae_max_speed even under large forces."""
        cfg = SimConfig(algae_max_speed=30.0, algae_max_force=15.0)
        a = _algae_at(100.0, 100.0, vx=0.0, vy=0.0, cfg=cfg)
        a.apply_force(np.array([9999.0, 0.0]))
        a.update(1 / 60)
        self.assertLessEqual(a.speed, cfg.algae_max_speed + 1e-9)

    def test_algae_min_speed_is_zero(self):
        """Algae must have min_speed=0 so they can stop completely."""
        a = _algae_at(100.0, 100.0, vx=0.0, vy=0.0)
        self.assertAlmostEqual(a.min_speed, 0.0)

    def test_algae_can_have_zero_speed(self):
        """With zero initial velocity and noise disabled, speed stays at 0.

        Verifies that algae has no min_speed floor (unlike sardines which
        always maintain sardine_min_speed).  Noise is disabled here so no
        steering force is accumulated that would raise speed above zero.
        """
        cfg = SimConfig(weight_algae_noise=0.0)
        a = _algae_at(100.0, 100.0, vx=0.0, vy=0.0, cfg=cfg)
        a.update(1 / 60)   # no neighbours, no noise → no force → speed stays 0
        self.assertAlmostEqual(a.speed, 0.0, places=6)

    def test_algae_boundary_wrap(self):
        """Algae position folds back into world bounds after crossing an edge."""
        cfg = SimConfig(window_width=200, window_height=100, boundary="wrap")
        grid = SpatialHash(cell_size=55.0, world_width=200.0, world_height=100.0)
        a = Algae(195.0, 50.0, 20.0, 0.0, cfg, grid)
        a.update(1.0)    # large dt: crosses the right boundary
        self.assertGreaterEqual(float(a.position[0]), 0.0)
        self.assertLess(float(a.position[0]), 200.0)

    # ── Spatial hash integration ──────────────────────────────────────

    def test_algae_grid_insertion_and_query(self):
        """Algae can be inserted into a SpatialHash and found by a neighbour query."""
        grid = _algae_grid()
        a = Algae(100.0, 100.0, 5.0, 0.0, CONFIG, grid)
        b = Algae(110.0, 100.0, 5.0, 0.0, CONFIG, grid)
        grid.insert(a)
        grid.insert(b)
        neighbours = grid.query(a, radius=30.0)
        self.assertIn(b, neighbours)

    def test_algae_grid_no_false_positive(self):
        """Algae outside the query radius must not appear in the result."""
        grid = _algae_grid()
        a = Algae(100.0, 100.0, 5.0, 0.0, CONFIG, grid)
        b = Algae(300.0, 300.0, 5.0, 0.0, CONFIG, grid)   # well outside 30 px
        grid.insert(a)
        grid.insert(b)
        neighbours = grid.query(a, radius=30.0)
        self.assertNotIn(b, neighbours)

    # ── Force computation ─────────────────────────────────────────────

    def test_algae_alignment_force_applied_with_different_heading_neighbour(self):
        """Neighbour heading perpendicular to self → non-zero alignment force."""
        cfg = SimConfig(
            weight_algae_alignment=1.5,
            weight_algae_separation=0.0,
            weight_algae_cohesion=0.0,
            weight_algae_noise=0.0,
            radius_algae_alignment=80.0,
            radius_algae_cohesion=80.0,
        )
        grid = SpatialHash(
            cell_size=80.0,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        a = Algae(100.0, 100.0, 10.0, 0.0, cfg, grid)    # heading +x
        b = Algae(120.0, 100.0,  0.0, 10.0, cfg, grid)   # heading +y, 20 px away
        grid.insert(a)
        grid.insert(b)
        a._compute_forces()
        self.assertGreater(
            float(np.linalg.norm(a.acceleration)), 0.0,
            "Alignment with a differently-headed neighbour should apply a force",
        )

    def test_algae_no_force_without_noise_or_neighbours(self):
        """With noise=0 and no neighbours, acceleration stays zero."""
        cfg = SimConfig(
            weight_algae_alignment=1.5,
            weight_algae_separation=0.2,
            weight_algae_cohesion=0.2,
            weight_algae_noise=0.0,         # noise disabled
        )
        a = _algae_at(100.0, 100.0, cfg=cfg)   # grid is empty
        a._compute_forces()
        np.testing.assert_array_almost_equal(
            a.acceleration, np.zeros(2),
            err_msg="No neighbours + noise disabled → zero acceleration",
        )

    # ── Simulation integration ────────────────────────────────────────

    def test_algae_spawn_count_in_simulation(self):
        """Simulation spawns exactly num_algae algae at startup."""
        cfg = SimConfig(num_algae=100, num_sardines=0, num_sailfish=0)
        sim = Simulation(cfg)
        self.assertEqual(len(sim.algae), 100)

    def test_algae_speed_bounded_in_simulation(self):
        """All algae stay within algae_max_speed after 60 frames."""
        cfg = SimConfig(num_algae=50, num_sardines=0, num_sailfish=0,
                        algae_max_speed=30.0)
        sim = Simulation(cfg)
        for _ in range(60):
            sim.update(1 / 60)
        for a in sim.algae:
            self.assertLessEqual(a.speed, cfg.algae_max_speed + 1e-6)

    def test_algae_positions_in_world_bounds(self):
        """All algae remain inside world bounds after a large dt update."""
        cfg = SimConfig(num_algae=50, num_sardines=0, num_sailfish=0)
        sim = Simulation(cfg)
        sim.update(2.0)
        W, H = float(cfg.window_width), float(cfg.window_height)
        for a in sim.algae:
            self.assertGreaterEqual(float(a.position[0]), 0.0)
            self.assertLess(float(a.position[0]), W)
            self.assertGreaterEqual(float(a.position[1]), 0.0)
            self.assertLess(float(a.position[1]), H)

    def test_algae_not_in_sardine_grid(self):
        """Algae must never appear in the sardine spatial hash."""
        cfg = SimConfig(num_algae=20, num_sardines=50, num_sailfish=0)
        sim = Simulation(cfg)
        sim.update(1 / 60)
        all_in_sardine_grid: list = []
        for cell in sim._grid._cells.values():
            all_in_sardine_grid.extend(cell)
        for agent in all_in_sardine_grid:
            self.assertNotIsInstance(
                agent, Algae, "Algae must not be inserted into the sardine grid"
            )

    def test_algae_not_in_sailfish_grid(self):
        """Algae must never appear in the sailfish spatial hash."""
        cfg = SimConfig(num_algae=20, num_sardines=0, num_sailfish=4)
        sim = Simulation(cfg)
        sim.update(1 / 60)
        all_in_sailfish_grid: list = []
        for cell in sim._sailfish_grid._cells.values():
            all_in_sailfish_grid.extend(cell)
        for agent in all_in_sailfish_grid:
            self.assertNotIsInstance(
                agent, Algae, "Algae must not be inserted into the sailfish grid"
            )

    def test_zero_algae_simulation(self):
        """Simulation with num_algae=0 runs without errors."""
        cfg = SimConfig(num_algae=0, num_sardines=10, num_sailfish=0)
        sim = Simulation(cfg)
        self.assertEqual(len(sim.algae), 0)
        sim.update(1 / 60)   # must not raise

    # ── Tuning-validation tests ───────────────────────────────────────

    def test_algae_separation_force_applied(self):
        """Two overlapping algae push apart; acceleration of 'a' points away from 'b'.

        Isolation: alignment, cohesion, and noise are zeroed so only separation
        contributes.  'b' is 5 px to the +x side of 'a', so separation should
        accumulate a −x component on 'a'.
        """
        cfg = SimConfig(
            weight_algae_separation=0.3,
            weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,
            weight_algae_noise=0.0,
            radius_algae_separation=20.0,
            radius_algae_cohesion=20.0,
        )
        grid = SpatialHash(
            cell_size=25.0,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        a = Algae(100.0, 100.0, 0.0, 0.0, cfg, grid)
        b = Algae(105.0, 100.0, 0.0, 0.0, cfg, grid)   # 5 px to the +x of a
        grid.insert(a)
        grid.insert(b)
        a._compute_forces()
        self.assertLess(
            float(a.acceleration[0]), 0.0,
            "Separation should push 'a' in the −x direction (away from 'b' at +x)",
        )

    def test_algae_noise_drives_movement_when_isolated(self):
        """Isolated algae with noise > 0 gains non-zero speed (noise fires unconditionally).

        Verifies the design invariant: noise accumulates velocity even when
        there are no neighbours.  60 independent Gaussian samples are
        statistically certain to produce a non-zero net velocity.
        """
        cfg = SimConfig(
            weight_algae_noise=0.05,
            weight_algae_alignment=0.0,
            weight_algae_separation=0.0,
            weight_algae_cohesion=0.0,
        )
        a = Algae(300.0, 300.0, 0.0, 0.0, cfg, _algae_grid(cfg))
        for _ in range(60):
            a.update(1 / 60)
        self.assertGreater(
            a.speed, 0.0,
            "Isolated algae with noise>0 should accumulate non-zero speed over 60 frames",
        )

    def test_algae_long_run_stability(self):
        """Algae positions and speeds stay bounded and finite over 300 frames.

        Guards against numerical blow-up (Euler divergence) or boundary-wrap
        failures under the tuned parameter set.  NaN in position would cause
        the bounds assertions to fail (NaN comparisons return False).
        """
        cfg = SimConfig(num_algae=50, num_sardines=0, num_sailfish=0)
        sim = Simulation(cfg)
        for _ in range(300):
            sim.update(1 / 60)
        W, H = float(cfg.window_width), float(cfg.window_height)
        for a in sim.algae:
            px, py = float(a.position[0]), float(a.position[1])
            self.assertGreaterEqual(px, 0.0, "algae x out of bounds (or NaN)")
            self.assertLess(px, W,           "algae x out of bounds (or inf)")
            self.assertGreaterEqual(py, 0.0, "algae y out of bounds (or NaN)")
            self.assertLess(py, H,           "algae y out of bounds (or inf)")
            self.assertLessEqual(
                a.speed, cfg.algae_max_speed + 1e-6,
                "algae speed must not exceed max_speed after 300 frames",
            )

    def test_algae_coexist_with_sardines_and_sailfish(self):
        """All three species run together for 60 frames without errors or cross-contamination.

        With algae_respawn_rate enabled (default) algae can grow above the
        starting population; test only checks reasonable bounds and that at
        least some algae survive.
        """
        cfg = SimConfig(num_algae=30, num_sardines=20, num_sailfish=2)
        sim = Simulation(cfg)
        for _ in range(60):
            sim.update(1 / 60)
        self.assertGreater(len(sim.algae), 0,
                           "algae must not be fully depleted")
        self.assertLessEqual(len(sim.algae), cfg.algae_max_count,
                             "algae must not exceed algae_max_count")
        # Sardines may be eaten by sailfish (Stage 8.6); count can only shrink.
        self.assertLessEqual(len(sim.sardines), 20)
        self.assertGreater(len(sim.sardines), 0, "At least some sardines should survive")
        self.assertEqual(len(sim.sailfish),  2)
        # Spot-check grid isolation after the last update
        all_in_sardine_grid: list = []
        for cell in sim._grid._cells.values():
            all_in_sardine_grid.extend(cell)
        for agent in all_in_sardine_grid:
            self.assertNotIsInstance(agent, Algae, "Algae must not appear in sardine grid")


# ── Stage 8.3: sardine-algae eating ecology pass ──────────────────────────────

def _make_ecology_sim(
    *,
    eat_radius: float = 20.0,
    sardine_pos: tuple[float, float] = (100.0, 100.0),
    algae_pos: tuple[float, float] = (105.0, 100.0),
) -> "Simulation":
    """Return a minimal Simulation with one sardine and one algae.

    Both are manually appended at fixed positions so tests are
    deterministic and independent of the PRNG-based spawners.
    """
    from sim.sardine import Sardine

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        algae_eat_radius=eat_radius,
        algae_respawn_rate=0.0,         # disable respawn so tests stay deterministic
        # Disable all flocking forces so agents stay near their initial position.
        weight_separation=0.0,
        weight_alignment=0.0,
        weight_cohesion=0.0,
        weight_noise=0.0,
        weight_algae_separation=0.0,
        weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0,
        weight_algae_noise=0.0,
        # Disable starvation and start hunger at 0 for deterministic timer tests.
        sardine_starvation_time=0.0,
        sardine_initial_hunger_time=0.0,
    )
    sim = Simulation(cfg)
    # Sardine: stationary at sardine_pos.
    sardine = Sardine(
        sardine_pos[0], sardine_pos[1], 0.0, 0.0,
        cfg, sim._grid, sim._sailfish_grid,
    )
    sim.sardines.append(sardine)
    # Algae: stationary at algae_pos.
    alga = Algae(algae_pos[0], algae_pos[1], 0.0, 0.0, cfg, sim._algae_grid)
    sim.algae.append(alga)
    return sim


class TestEcologyContactDetection(unittest.TestCase):
    """Tests for Stage 8.3: sardine food_counter, hunger_timer, and algae removal."""

    # ── Sardine initial state ─────────────────────────────────────────

    def test_sardine_food_counter_initial_zero(self):
        """Sardine.food_counter starts at 0."""
        from sim.sardine import Sardine
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0)
        sim = Simulation(cfg)
        s = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        self.assertEqual(s.food_counter, 0.0)

    def test_sardine_hunger_timer_initialized_from_config(self):
        """Sardine.hunger_timer is initialized from sardine_initial_hunger_time."""
        from sim.sardine import Sardine
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0,
                        sardine_initial_hunger_time=7.5)
        sim = Simulation(cfg)
        s = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        self.assertAlmostEqual(s.hunger_timer, 7.5, places=9)

    # ── Contact eating ────────────────────────────────────────────────

    def test_sardine_eats_algae_within_radius(self):
        """Sardine within eat_radius of algae gains food_counter == 1."""
        # sardine at (100,100), algae at (105,100): distance = 5 px < eat_radius=20
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        sardine = sim.sardines[0]
        sim.update(1 / 60)
        self.assertEqual(sardine.food_counter, 1.0,
                         "Sardine within eat_radius should increment food_counter by 1")

    def test_algae_removed_after_eaten(self):
        """The eaten algae is removed from simulation.algae after update()."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 0,
                         "Eaten algae must be removed from simulation.algae")

    def test_algae_outside_radius_not_eaten(self):
        """Algae outside eat_radius is left intact."""
        # sardine at (100,100), algae at (200,100): distance = 100 px > eat_radius=20
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(200.0, 100.0))
        sardine = sim.sardines[0]
        sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 1,
                         "Algae outside eat_radius must not be removed")
        self.assertEqual(sardine.food_counter, 0.0,
                         "food_counter must not increment when no contact")

    def test_hunger_timer_increments_without_eating(self):
        """hunger_timer grows when no algae are nearby."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(200.0, 100.0))   # out of range
        sardine = sim.sardines[0]
        dt = 1 / 60
        sim.update(dt)
        self.assertAlmostEqual(sardine.hunger_timer, dt, places=9,
                               msg="hunger_timer should increment by dt when sardine does not eat")

    def test_hunger_timer_resets_on_eat(self):
        """hunger_timer is 0 immediately after a sardine eats."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))   # within eat_radius
        sardine = sim.sardines[0]
        sim.update(1 / 60)
        self.assertEqual(sardine.hunger_timer, 0.0,
                         "hunger_timer must be reset to 0 when sardine eats")

    def test_stats_algae_eaten_increments(self):
        """simulation.stats_algae_eaten reflects total eaten across frames."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_eat_radius=20.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        # Two algae both within eat_radius.
        sim.algae.append(Algae(105.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid))
        sim.algae.append(Algae(110.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid))
        sim.update(1 / 60)
        self.assertEqual(sim.stats_algae_eaten, 2,
                         "stats_algae_eaten must equal the total algae consumed")

    def test_algae_eaten_ids_populated(self):
        """_algae_eaten_ids contains the id() of each eaten algae after update()."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        eaten_algae = sim.algae[0]
        sim.update(1 / 60)
        self.assertIn(id(eaten_algae), sim._algae_eaten_ids,
                      "_algae_eaten_ids must hold the id() of the consumed algae")

    def test_algae_eaten_ids_cleared_next_frame(self):
        """_algae_eaten_ids is cleared at the start of the following update()."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        sim.update(1 / 60)           # frame 1: algae eaten, ids populated
        self.assertTrue(sim._algae_eaten_ids)
        sim.update(1 / 60)           # frame 2: ids cleared at start — nothing to eat
        self.assertFalse(sim._algae_eaten_ids,
                         "_algae_eaten_ids must be empty after the next update()")

    def test_no_double_removal_of_same_algae(self):
        """One algae touched by two sardines is removed exactly once."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_eat_radius=20.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        # Two sardines, both within eat_radius of the same algae.
        s1 = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(102.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([s1, s2])
        alga = Algae(105.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)
        sim.update(1 / 60)
        # Algae removed exactly once regardless of how many sardines touched it.
        self.assertEqual(len(sim.algae), 0)
        self.assertEqual(sim.stats_algae_eaten, 1,
                         "stats_algae_eaten must be 1 (single algae removed once)")

    def test_sardine_count_unchanged_after_eating(self):
        """Eating algae does not alter the sardine count."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 1,
                         "Sardine count must not change during algae eating")

    def test_zero_algae_ecology_pass_no_error(self):
        """Ecology pass with no algae runs without errors."""
        cfg = SimConfig(num_sardines=50, num_sailfish=0, num_algae=0,
                        algae_eat_radius=20.0)
        sim = Simulation(cfg)
        sim.update(1 / 60)   # must not raise
        self.assertEqual(sim.stats_algae_eaten, 0)

    def test_zero_sardines_ecology_pass_no_error(self):
        """Ecology pass with no sardines runs without errors."""
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=50,
                        algae_eat_radius=20.0)
        sim = Simulation(cfg)
        sim.update(1 / 60)   # must not raise
        self.assertEqual(len(sim.algae), 50, "Algae must survive with no sardines")

    def test_no_algae_in_sardine_or_sailfish_grid_after_eating(self):
        """Eaten algae leave no references in sardine or sailfish grids."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(105.0, 100.0))
        sim.update(1 / 60)
        all_in_sardine_grid: list = []
        for cell in sim._grid._cells.values():
            all_in_sardine_grid.extend(cell)
        for agent in all_in_sardine_grid:
            self.assertNotIsInstance(agent, Algae,
                                     "No algae in sardine grid after eating")
        all_in_sailfish_grid: list = []
        for cell in sim._sailfish_grid._cells.values():
            all_in_sailfish_grid.extend(cell)
        for agent in all_in_sailfish_grid:
            self.assertNotIsInstance(agent, Algae,
                                     "No algae in sailfish grid after eating")

    def test_hunger_timer_accumulates_across_frames_without_eating(self):
        """hunger_timer grows monotonically over multiple frames with no food."""
        sim = _make_ecology_sim(eat_radius=20.0,
                                sardine_pos=(100.0, 100.0),
                                algae_pos=(500.0, 500.0))   # far away
        sardine = sim.sardines[0]
        dt = 1 / 60
        for _ in range(5):
            sim.update(dt)
        self.assertAlmostEqual(sardine.hunger_timer, dt * 5, places=9,
                               msg="hunger_timer must accumulate dt each frame")


# ── Stage 8.4: sardine reproduction ──────────────────────────────────────────

def _make_reproduction_sim(
    *,
    threshold: int = 5,
    sardine_pos: tuple[float, float] = (700.0, 350.0),
    initial_food: float = 0.0,
) -> "Simulation":
    """Return a bare Simulation with one sardine at a fixed position.

    Algae are NOT pre-added — tests that need eating should add algae
    manually so they control the exact count.
    Forces are disabled so the sardine stays near its spawn point.
    """
    from sim.sardine import Sardine

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        sardine_eat_threshold=threshold,
        algae_eat_radius=20.0,
        algae_respawn_rate=0.0,         # disable respawn — tests manage algae manually
        # Disable flocking forces so positions stay predictable.
        weight_separation=0.0,
        weight_alignment=0.0,
        weight_cohesion=0.0,
        weight_noise=0.0,
        weight_algae_separation=0.0,
        weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0,
        weight_algae_noise=0.0,
    )
    sim = Simulation(cfg)
    sardine = Sardine(
        sardine_pos[0], sardine_pos[1],
        10.0, 0.0,        # small +x velocity so heading is well-defined
        cfg, sim._grid, sim._sailfish_grid,
    )
    sardine.food_counter = initial_food
    sim.sardines.append(sardine)
    return sim


class TestSardineReproduction(unittest.TestCase):
    """Tests for Stage 8.4: sardine food_counter → offspring spawning."""

    # ── Threshold logic ───────────────────────────────────────────────

    def test_no_reproduction_below_threshold(self):
        """food_counter < threshold → no offspring spawned."""
        sim = _make_reproduction_sim(threshold=5, initial_food=4.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 1,
                         "No offspring when food_counter < threshold")
        self.assertEqual(sim.stats_sardine_births, 0)

    def test_reproduction_at_exact_threshold(self):
        """food_counter == threshold → exactly one offspring spawned."""
        sim = _make_reproduction_sim(threshold=5, initial_food=5.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 2,
                         "Exactly one offspring when food_counter == threshold")
        self.assertEqual(sim.stats_sardine_births, 1)

    def test_reproduction_above_threshold(self):
        """food_counter > threshold → still exactly one offspring per update."""
        sim = _make_reproduction_sim(threshold=5, initial_food=8.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 2,
                         "One offspring per update, regardless of excess food")
        self.assertEqual(sim.stats_sardine_births, 1)

    def test_food_counter_decremented_by_threshold(self):
        """After reproduction food_counter is decremented by threshold (not reset to 0)."""
        threshold = 5
        initial_food = 7.0
        sim = _make_reproduction_sim(threshold=threshold, initial_food=initial_food)
        parent = sim.sardines[0]
        sim.update(1 / 60)
        self.assertAlmostEqual(
            parent.food_counter, initial_food - threshold, places=9,
            msg="food_counter -= threshold preserves leftover progress",
        )

    def test_food_counter_reset_exactly_at_threshold(self):
        """food_counter becomes 0 when it was exactly threshold."""
        sim = _make_reproduction_sim(threshold=5, initial_food=5.0)
        parent = sim.sardines[0]
        sim.update(1 / 60)
        self.assertAlmostEqual(parent.food_counter, 0.0, places=9)

    def test_threshold_zero_disables_reproduction(self):
        """threshold=0 → no reproduction even with high food_counter."""
        sim = _make_reproduction_sim(threshold=0, initial_food=100.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 1,
                         "threshold=0 should disable reproduction entirely")
        self.assertEqual(sim.stats_sardine_births, 0)

    # ── Offspring validity ────────────────────────────────────────────

    def test_offspring_in_world_bounds(self):
        """Offspring position is inside [0, W) × [0, H)."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sardines[1]
        W = float(sim.config.window_width)
        H = float(sim.config.window_height)
        px, py = float(offspring.position[0]), float(offspring.position[1])
        self.assertGreaterEqual(px, 0.0)
        self.assertLess(px, W)
        self.assertGreaterEqual(py, 0.0)
        self.assertLess(py, H)

    def test_offspring_has_non_nan_velocity(self):
        """Offspring velocity components are finite numbers."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sardines[1]
        vx = float(offspring.velocity[0])
        vy = float(offspring.velocity[1])
        self.assertTrue(math.isfinite(vx), "offspring vx must be finite")
        self.assertTrue(math.isfinite(vy), "offspring vy must be finite")

    def test_offspring_different_position_from_parent(self):
        """Offspring is not stacked exactly on top of the parent."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        parent = sim.sardines[0]
        sim.update(1 / 60)
        offspring = sim.sardines[1]
        dx = float(offspring.position[0]) - float(parent.position[0])
        dy = float(offspring.position[1]) - float(parent.position[1])
        # Toroidal distance: take the minimum of direct and wrapped
        W = float(sim.config.window_width)
        H = float(sim.config.window_height)
        if dx > W / 2:  dx -= W
        if dx < -W / 2: dx += W
        if dy > H / 2:  dy -= H
        if dy < -H / 2: dy += H
        d = math.sqrt(dx * dx + dy * dy)
        self.assertGreater(d, 1.0, "Offspring must not be stacked exactly on parent")

    def test_offspring_is_sardine_instance(self):
        """Newly born boid is a Sardine instance."""
        from sim.sardine import Sardine
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        self.assertIsInstance(sim.sardines[1], Sardine)

    def test_offspring_food_counter_zero(self):
        """Offspring starts with food_counter = 0."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sardines[1]
        self.assertEqual(offspring.food_counter, 0.0)

    def test_offspring_hunger_timer_initialized_from_config(self):
        """Offspring hunger_timer is initialized from sardine_initial_hunger_time."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sardines[1]
        self.assertAlmostEqual(
            offspring.hunger_timer, sim.config.sardine_initial_hunger_time, places=9,
            msg="Newborn sardine hunger_timer must equal sardine_initial_hunger_time",
        )

    # ── sardine_born lifecycle ────────────────────────────────────────

    def test_sardine_born_populated_on_birth_frame(self):
        """_sardine_born contains the offspring immediately after update()."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim._sardine_born), 1)
        self.assertIs(sim._sardine_born[0], sim.sardines[1])

    def test_sardine_born_cleared_next_frame(self):
        """_sardine_born is empty at the start of the next update()."""
        sim = _make_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)   # birth frame
        self.assertTrue(sim._sardine_born)
        sim.update(1 / 60)   # next frame — cleared at top of update()
        self.assertFalse(sim._sardine_born,
                         "_sardine_born must be cleared at the start of each update()")

    def test_stats_sardine_births_cumulative(self):
        """stats_sardine_births accumulates across multiple frames."""
        # threshold=1, food pre-loaded so each frame triggers a birth up to
        # n_current cap; two separate sardines both at threshold → 2 births.
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=1,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        for _ in range(2):
            s = Sardine(700.0, 350.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
            s.food_counter = 1.0
            sim.sardines.append(s)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sardine_births, 2)

    # ── Integration / stability ───────────────────────────────────────

    def test_repeated_reproduction_does_not_corrupt_simulation(self):
        """30 frames of rapid reproduction keeps population bounded and valid."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=5,
            num_sailfish=0,
            num_algae=200,
            sardine_eat_threshold=1,   # very fast reproduction
            algae_eat_radius=60.0,     # wide contact so eating is inevitable
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        for _ in range(30):
            sim.update(1 / 60)
        W = float(cfg.window_width)
        H = float(cfg.window_height)
        for s in sim.sardines:
            px, py = float(s.position[0]), float(s.position[1])
            self.assertTrue(math.isfinite(px) and math.isfinite(py),
                            "sardine position must be finite after rapid reproduction")
            self.assertGreaterEqual(px, 0.0)
            self.assertLess(px, W)
            self.assertGreaterEqual(py, 0.0)
            self.assertLess(py, H)

    def test_reproduction_via_eating_increments_count(self):
        """Sardines that eat enough algae to reproduce increase the sardine count."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=3,
            algae_eat_radius=20.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        # Place 3 algae at the sardine's position — all eaten on the first frame.
        for _ in range(3):
            sim.algae.append(Algae(100.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid))
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 2,
                         "Sardine that ate enough algae in one frame should reproduce")
        self.assertEqual(sim.stats_sardine_births, 1)


# ── Stage 8.6: sailfish eating sardines ──────────────────────────────────────

def _make_sailfish_ecology_sim(
    *,
    eat_radius: float = 20.0,
    sailfish_pos: tuple = (100.0, 100.0),
    sardine_pos: tuple  = (105.0, 100.0),
) -> "Simulation":
    """Return a minimal Simulation with one sailfish and one sardine.

    All forces are disabled and min_speed set to 0 so agents stay at their
    initial positions, making contact-detection tests deterministic.
    """
    from sim.sardine import Sardine
    from sim.sailfish import Sailfish

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        sailfish_eat_radius=eat_radius,
        sardine_eat_threshold=0,       # disable reproduction
        sardine_min_speed=0.0,         # allow stationary sardines
        sailfish_min_speed=0.0,        # allow stationary sailfish
        algae_respawn_rate=0.0,        # disable respawn — no algae in this sim
        weight_separation=0.0, weight_alignment=0.0,
        weight_cohesion=0.0,   weight_noise=0.0,
        weight_algae_separation=0.0, weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
        weight_pursuit=0.0,
        weight_flee=0.0,
        # Disable starvation and start hunger at 0 for deterministic timer tests.
        sardine_starvation_time=0.0,
        sardine_initial_hunger_time=0.0,
        sailfish_starvation_time=0.0,
        sailfish_initial_hunger_time=0.0,
    )
    sim = Simulation(cfg)
    sf = Sailfish(sailfish_pos[0], sailfish_pos[1], 0.0, 0.0, cfg, sim._grid)
    sim.sailfish.append(sf)
    sardine = Sardine(
        sardine_pos[0], sardine_pos[1], 0.0, 0.0,
        cfg, sim._grid, sim._sailfish_grid,
    )
    sim.sardines.append(sardine)
    return sim


class TestSailfishEcology(unittest.TestCase):
    """Tests for Stage 8.6: sailfish food_counter, hunger_timer, sardine removal."""

    # ── Sailfish initial state ─────────────────────────────────────────

    def test_sailfish_food_counter_initial_zero(self):
        """Sailfish.food_counter starts at 0."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0)
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        self.assertEqual(sf.food_counter, 0.0)

    def test_sailfish_hunger_timer_starts_at_zero(self):
        """New Sailfish always start with hunger_timer = 0 regardless of config."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0,
                        sailfish_initial_hunger_time=7.5)
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                               msg="hunger_timer must always start at 0")

    # ── Contact eating ─────────────────────────────────────────────────

    def test_sailfish_eats_sardine_within_radius(self):
        """Sailfish within eat_radius of a sardine removes it and increments food_counter."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        sf = sim.sailfish[0]
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 0,
                         "Sardine within eat_radius must be removed after sailfish eats it")
        self.assertEqual(sf.food_counter, 1.0,
                         "Sailfish food_counter must increment when it eats a sardine")

    def test_sardine_outside_radius_not_eaten(self):
        """Sardine outside eat_radius is not eaten."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(200.0, 100.0),   # 100 px away
        )
        sf = sim.sailfish[0]
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 1,
                         "Sardine outside eat_radius must not be removed")
        self.assertEqual(sf.food_counter, 0.0,
                         "Sailfish food_counter must not increment when no contact")

    def test_sailfish_hunger_timer_resets_on_eat(self):
        """Sailfish hunger_timer is 0 after eating a sardine."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        sf = sim.sailfish[0]
        sim.update(1 / 60)
        self.assertEqual(sf.hunger_timer, 0.0,
                         "Sailfish hunger_timer must reset to 0 after eating")

    def test_stats_sardines_eaten_increments(self):
        """simulation.stats_sardines_eaten is 1 after one sailfish eats one sardine."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        # Two sardines both within eat_radius, but sailfish eats only one per tick.
        s1 = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(108.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([s1, s2])
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sardines_eaten, 1,
                         "One sailfish eats exactly one sardine per tick")
        self.assertEqual(len(sim.sardines), 1)

    def test_sardines_eaten_ids_populated(self):
        """_sardines_eaten_ids contains the id() of the eaten sardine after update()."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        eaten_sardine = sim.sardines[0]
        sim.update(1 / 60)
        self.assertIn(id(eaten_sardine), sim._sardines_eaten_ids,
                      "_sardines_eaten_ids must contain the id() of the eaten sardine")

    def test_sardines_eaten_ids_cleared_next_frame(self):
        """_sardines_eaten_ids is cleared at the start of the following update()."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        sim.update(1 / 60)   # sardine eaten; ids populated
        self.assertTrue(sim._sardines_eaten_ids)
        sim.update(1 / 60)   # sardine gone; ids cleared at start
        self.assertFalse(sim._sardines_eaten_ids,
                         "_sardines_eaten_ids must be empty after the next update()")

    def test_one_sardine_eaten_per_sailfish_per_tick(self):
        """A single sailfish eats at most one sardine per frame even with many nearby."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=50.0,   # large so all 5 sardines are in range
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        for i in range(5):
            s = Sardine(100.0 + i * 5, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
            sim.sardines.append(s)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 4,
                         "One sailfish must eat exactly one sardine per tick regardless of crowd size")
        self.assertEqual(sf.food_counter, 1.0)

    def test_multiple_sailfish_eat_simultaneously(self):
        """Two sailfish near separate sardines can each eat one per tick."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf1 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf2 = Sailfish(500.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.extend([sf1, sf2])
        s1 = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(505.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([s1, s2])
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 0,
                         "Both sardines should be eaten (one per sailfish per tick)")
        self.assertEqual(sf1.food_counter, 1.0)
        self.assertEqual(sf2.food_counter, 1.0)

    def test_sailfish_count_unchanged_after_eating(self):
        """Eating sardines does not alter the sailfish count."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 1,
                         "Sailfish count must not change during predation")

    def test_zero_sardines_no_predation_error(self):
        """Sailfish ecology pass with no sardines runs without errors."""
        cfg = SimConfig(num_sardines=0, num_sailfish=4, num_algae=0,
                        sailfish_eat_radius=20.0)
        sim = Simulation(cfg)
        sim.update(1 / 60)   # must not raise
        self.assertEqual(sim.stats_sardines_eaten, 0)

    def test_zero_sailfish_no_predation_error(self):
        """Predation ecology pass with no sailfish runs without errors."""
        cfg = SimConfig(num_sardines=50, num_sailfish=0, num_algae=0,
                        sailfish_eat_radius=20.0)
        sim = Simulation(cfg)
        sim.update(1 / 60)   # must not raise
        self.assertEqual(sim.stats_sardines_eaten, 0)

    def test_stats_sardines_eaten_cumulative(self):
        """stats_sardines_eaten accumulates correctly across multiple frames."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        # Two sardines 5 px apart; sailfish eats one per frame.
        s1 = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([s1, s2])
        sim.update(1 / 60)   # eats first sardine
        sim.update(1 / 60)   # eats second sardine
        self.assertEqual(sim.stats_sardines_eaten, 2,
                         "stats_sardines_eaten must accumulate across frames")
        self.assertEqual(sf.food_counter, 2.0,
                         "food_counter must increment each time sailfish eats")
        self.assertEqual(len(sim.sardines), 0)

    def test_no_broken_references_after_predation(self):
        """Eaten sardine is absent from simulation.sardines; next update runs cleanly."""
        sim = _make_sailfish_ecology_sim(
            eat_radius=20.0,
            sailfish_pos=(100.0, 100.0),
            sardine_pos=(105.0, 100.0),
        )
        eaten = sim.sardines[0]
        sim.update(1 / 60)   # sardine eaten and removed
        self.assertNotIn(eaten, sim.sardines,
                         "Eaten sardine must not remain in simulation.sardines")
        # A second update with an empty sardine list must not raise.
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 0)

    def test_predation_does_not_affect_algae(self):
        """Eating a sardine does not alter the algae population."""
        from sim.algae import Algae
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
            algae_eat_radius=0.1,   # negligible so sardine doesn't eat algae
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        s = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        # Algae far from everything.
        a = Algae(600.0, 400.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(a)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 0, "Sardine should be eaten")
        self.assertEqual(len(sim.algae),    1, "Algae must be unaffected by predation")

    def test_two_sailfish_both_satiated_by_single_sardine_kill(self):
        """Two sailfish within eat_radius of the same sardine both get food credit.

        When both sailfish pick the same prey, the sardine is removed exactly
        once (set deduplication in to_remove), but each sailfish's food_counter
        is incremented and hunger_timer is reset.  stats_sardines_eaten reflects
        the one unique kill, not two.
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=30.0,
            sardine_starvation_time=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        # Both sailfish within eat_radius=30 of the sardine at (105, 100).
        sf1 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf2 = Sailfish(110.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.extend([sf1, sf2])
        sardine = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)

        sim.update(1 / 60)

        # The sardine must be removed exactly once (set deduplication).
        self.assertEqual(len(sim.sardines), 0,
                         "The sardine must be eaten once, not duplicated")
        self.assertEqual(sim.stats_sardines_eaten, 1,
                         "stats_sardines_eaten must be 1 for one unique sardine removed")
        # Both sailfish are satiated by the shared kill.
        self.assertEqual(sf1.food_counter, 1.0,
                         "sf1 must receive food credit for the shared kill")
        self.assertEqual(sf2.food_counter, 1.0,
                         "sf2 must receive food credit for the shared kill")
        self.assertAlmostEqual(sf1.hunger_timer, 0.0, places=9,
                               msg="sf1 hunger_timer must reset to 0 after eating")
        self.assertAlmostEqual(sf2.hunger_timer, 0.0, places=9,
                               msg="sf2 hunger_timer must reset to 0 after eating")


# ── Stage 8.5: sardine starvation ────────────────────────────────────────────

def _make_starvation_sim(
    *,
    sardine_initial_hunger: float = 0.0,
    starvation_time: float = 5.0,
    sardine_pos: tuple[float, float] = (100.0, 100.0),
) -> "tuple[Simulation, object]":
    """Return a minimal Simulation with one sardine for starvation tests.

    Starvation is enabled with a controlled threshold.  Reproduction,
    flocking, and algae eating are all disabled.
    """
    from sim.sardine import Sardine

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        sardine_initial_hunger_time=sardine_initial_hunger,
        sardine_starvation_time=starvation_time,
        sardine_eat_threshold=0,        # disable reproduction
        algae_respawn_rate=0.0,         # CRITICAL: spawned algae would reset hunger_timer
        sailfish_starvation_time=0.0,   # disable sailfish starvation
        sardine_min_speed=0.0,
        weight_separation=0.0, weight_alignment=0.0,
        weight_cohesion=0.0, weight_noise=0.0,
        weight_algae_separation=0.0, weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0, weight_algae_noise=0.0,
    )
    sim = Simulation(cfg)
    sardine = Sardine(
        sardine_pos[0], sardine_pos[1], 0.0, 0.0,
        cfg, sim._grid, sim._sailfish_grid,
    )
    sim.sardines.append(sardine)
    return sim, sardine


class TestSardineStarvation(unittest.TestCase):
    """Tests for Stage 8.5: sardine starvation mechanic."""

    # ── Initial state ─────────────────────────────────────────────────

    def test_sardine_initial_hunger_time_set_from_config(self):
        """Sardine.hunger_timer is initialized from sardine_initial_hunger_time config."""
        sim, sardine = _make_starvation_sim(sardine_initial_hunger=8.0)
        self.assertAlmostEqual(sardine.hunger_timer, 8.0, places=9)

    # ── Survival ──────────────────────────────────────────────────────

    def test_sardine_survives_below_threshold(self):
        """Sardine survives when hunger_timer < starvation_time."""
        sim, sardine = _make_starvation_sim(starvation_time=5.0)
        sim.update(1 / 60)   # hunger_timer ≈ 0.0167 << 5.0
        self.assertIn(sardine, sim.sardines,
                      "Sardine must survive when hunger_timer is below starvation_time")

    def test_sardine_removed_when_starved(self):
        """Sardine is removed once hunger_timer > starvation_time."""
        # hunger_timer starts above threshold so any positive dt triggers starvation
        # (note: update() caps dt at 1/20 s, so we cannot rely on large dt values).
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=5.1, starvation_time=5.0
        )
        sim.update(1 / 60)
        self.assertNotIn(sardine, sim.sardines,
                         "Sardine must be removed when hunger_timer exceeds starvation_time")

    def test_starvation_disabled_when_time_is_zero(self):
        """starvation_time=0.0 disables the starvation mechanic."""
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=999.0, starvation_time=0.0
        )
        sim.update(1 / 60)
        self.assertIn(sardine, sim.sardines,
                      "Sardine must never starve when starvation_time=0.0")

    def test_multiple_sardines_starve_together(self):
        """All sardines past the threshold are removed in one pass."""
        from sim.sardine import Sardine

        cfg = sim_cfg = _make_starvation_sim(starvation_time=5.0)[0].config
        sim2 = Simulation(cfg)
        sardines = []
        for i in range(3):
            s = Sardine(100.0 + i * 10, 100.0, 0.0, 0.0, cfg,
                        sim2._grid, sim2._sailfish_grid)
            s.hunger_timer = 5.1   # already past threshold
            sim2.sardines.append(s)
            sardines.append(s)
        sim2.update(1 / 60)
        self.assertEqual(len(sim2.sardines), 0,
                         "All sardines past the threshold must be removed together")

    # ── Side-channels ─────────────────────────────────────────────────

    def test_sardines_starved_ids_populated(self):
        """_sardines_starved_ids contains the id() of each starved sardine on death frame."""
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=5.1, starvation_time=5.0
        )
        sim.update(1 / 60)
        self.assertIn(id(sardine), sim._sardines_starved_ids,
                      "_sardines_starved_ids must contain the starved sardine's id()")

    def test_sardines_starved_ids_cleared_next_frame(self):
        """_sardines_starved_ids is empty at the start of the following update()."""
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=5.1, starvation_time=5.0
        )
        sim.update(1 / 60)   # sardine starves; ids populated
        self.assertTrue(sim._sardines_starved_ids)
        sim.update(1 / 60)   # next frame: ids cleared
        self.assertFalse(sim._sardines_starved_ids,
                         "_sardines_starved_ids must be empty after the following update()")

    def test_sardines_starved_ids_separate_from_eaten_ids(self):
        """Starvation deaths land in _sardines_starved_ids, not _sardines_eaten_ids."""
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=5.1, starvation_time=5.0
        )
        sim.update(1 / 60)
        self.assertIn(id(sardine), sim._sardines_starved_ids)
        self.assertNotIn(id(sardine), sim._sardines_eaten_ids,
                         "Starved sardines must not appear in the predation side-channel")

    # ── Stats ─────────────────────────────────────────────────────────

    def test_stats_sardine_starvations_increments(self):
        """stats_sardine_starvations accumulates the total number of starvation deaths."""
        sim, sardine = _make_starvation_sim(
            sardine_initial_hunger=5.1, starvation_time=5.0
        )
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sardine_starvations, 1,
                         "stats_sardine_starvations must increment for each starvation death")

    def test_stats_multiple_starvations_cumulative(self):
        """stats_sardine_starvations accumulates correctly across frames."""
        from sim.sardine import Sardine

        cfg = _make_starvation_sim(starvation_time=5.0)[0].config
        sim = Simulation(cfg)
        # Frame 1: s1 starves.
        s1 = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s1.hunger_timer = 5.1   # already above threshold
        sim.sardines.append(s1)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sardine_starvations, 1)
        # Frame 2: add s2 with hunger above threshold; it also starves.
        s2 = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2.hunger_timer = 5.1
        sim.sardines.append(s2)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sardine_starvations, 2)

    # ── Interaction with other mechanics ─────────────────────────────

    def test_starvation_does_not_kill_eating_sardine(self):
        """A sardine that eats on the same frame it would starve survives."""
        from sim.sardine import Sardine

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_eat_radius=20.0,
            sardine_initial_hunger_time=4.9,
            sardine_starvation_time=5.0,
            sardine_eat_threshold=0,
            sailfish_starvation_time=0.0,
            sardine_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        # Algae within eat_radius; sardine will eat it this frame (dt=0.2).
        alga = Algae(105.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)
        sim.update(0.2)   # hunger before eating = 4.9 + 0.2 = 5.1 > 5.0
        # But eating ran FIRST so hunger_timer was reset to 0 before starvation check.
        self.assertIn(sardine, sim.sardines,
                      "Sardine that ate on the starvation frame must survive")
        self.assertEqual(sardine.hunger_timer, 0.0)

    def test_starvation_runs_before_reproduction(self):
        """A starving sardine does not produce offspring."""
        from sim.sardine import Sardine

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_initial_hunger_time=5.1,
            sardine_starvation_time=5.0,
            sardine_eat_threshold=1,     # would reproduce after just 1 algae
            sailfish_starvation_time=0.0,
            sardine_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sardine.food_counter = 1.0   # enough to trigger reproduction
        sim.sardines.append(sardine)
        sim.update(1 / 60)   # hunger already > threshold → starves before reproduction
        self.assertEqual(len(sim.sardines), 0,
                         "Starving sardine must be removed before reproduction fires")


# ── Stage 8.7: sailfish starvation ────────────────────────────────────────────

def _make_sailfish_starvation_sim(
    *,
    sailfish_initial_hunger: float = 0.0,
    starvation_time: float = 5.0,
    sailfish_pos: tuple[float, float] = (100.0, 100.0),
) -> "tuple[Simulation, object]":
    """Return a minimal Simulation with one sailfish for starvation tests."""
    from sim.sailfish import Sailfish

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        sailfish_initial_hunger_time=sailfish_initial_hunger,
        sailfish_starvation_time=starvation_time,
        sardine_starvation_time=0.0,    # disable sardine starvation
        sardine_eat_threshold=0,
        algae_respawn_rate=0.0,         # no algae in this sim; disable respawn
        sardine_min_speed=0.0,
        sailfish_min_speed=0.0,
        weight_separation=0.0, weight_alignment=0.0,
        weight_cohesion=0.0, weight_noise=0.0,
        weight_algae_separation=0.0, weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        weight_pursuit=0.0, weight_flee=0.0,
    )
    sim = Simulation(cfg)
    sf = Sailfish(sailfish_pos[0], sailfish_pos[1], 0.0, 0.0, cfg, sim._grid)
    sim.sailfish.append(sf)
    return sim, sf


class TestSailfishStarvation(unittest.TestCase):
    """Tests for Stage 8.7: sailfish starvation mechanic."""

    # ── Initial state ─────────────────────────────────────────────────

    def test_sailfish_hunger_timer_starts_at_zero(self):
        """New sailfish always start with hunger_timer = 0, ignoring sailfish_initial_hunger_time."""
        sim, sf = _make_sailfish_starvation_sim(sailfish_initial_hunger=3.0, starvation_time=5.0)
        self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                               msg="hunger_timer must always start at 0")

    def test_sailfish_hunger_timer_increments_each_frame(self):
        """Sailfish hunger_timer grows by dt each frame when it does not eat."""
        sim, sf = _make_sailfish_starvation_sim(
            sailfish_initial_hunger=0.0, starvation_time=0.0
        )
        dt = 1 / 60
        sim.update(dt)
        self.assertAlmostEqual(sf.hunger_timer, dt, places=9,
                               msg="hunger_timer must increment by dt each frame without eating")

    # ── Survival ──────────────────────────────────────────────────────

    def test_sailfish_survives_below_threshold(self):
        """Sailfish survives when hunger_timer < starvation_time."""
        sim, sf = _make_sailfish_starvation_sim(starvation_time=5.0)
        sim.update(1 / 60)
        self.assertIn(sf, sim.sailfish,
                      "Sailfish must survive when hunger_timer is below starvation_time")

    def test_sailfish_removed_when_starved(self):
        """Sailfish is removed once hunger_timer > starvation_time."""
        # hunger_timer is set above threshold so any positive dt triggers starvation.
        sim, sf = _make_sailfish_starvation_sim(starvation_time=5.0)
        sf.hunger_timer = 5.1   # manually place above threshold
        sim.update(1 / 60)
        self.assertNotIn(sf, sim.sailfish,
                         "Sailfish must be removed when hunger_timer exceeds starvation_time")

    def test_sailfish_starvation_disabled_by_zero(self):
        """sailfish_starvation_time=0.0 disables the mechanic."""
        sim, sf = _make_sailfish_starvation_sim(
            sailfish_initial_hunger=999.0, starvation_time=0.0
        )
        sim.update(1 / 60)
        self.assertIn(sf, sim.sailfish,
                      "Sailfish must never starve when sailfish_starvation_time=0.0")

    # ── Side-channels ─────────────────────────────────────────────────

    def test_sailfish_starved_ids_populated(self):
        """_sailfish_died_ids contains the id() of each starved sailfish on death frame."""
        sim, sf = _make_sailfish_starvation_sim(starvation_time=5.0)
        sf.hunger_timer = 5.1   # manually place above threshold
        sim.update(1 / 60)
        self.assertIn(id(sf), sim._sailfish_died_ids,
                      "_sailfish_died_ids must contain the starved sailfish's id()")

    def test_sailfish_starved_ids_cleared_next_frame(self):
        """_sailfish_died_ids is cleared at the start of the following update()."""
        sim, sf = _make_sailfish_starvation_sim(starvation_time=5.0)
        sf.hunger_timer = 5.1   # manually place above threshold
        sim.update(1 / 60)   # sailfish starves; ids populated
        self.assertTrue(sim._sailfish_died_ids)
        sim.update(1 / 60)   # next frame: ids cleared
        self.assertFalse(sim._sailfish_died_ids,
                         "_sailfish_died_ids must be empty after the following update()")

    # ── Stats ─────────────────────────────────────────────────────────

    def test_stats_sailfish_starvations_increments(self):
        """stats_sailfish_starvations increments for each sailfish starvation death."""
        sim, sf = _make_sailfish_starvation_sim(starvation_time=5.0)
        sf.hunger_timer = 5.1   # manually place above threshold
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sailfish_starvations, 1)

    def test_stats_sailfish_starvations_cumulative(self):
        """stats_sailfish_starvations accumulates across frames."""
        from sim.sailfish import Sailfish

        cfg = _make_sailfish_starvation_sim(starvation_time=5.0)[0].config
        sim = Simulation(cfg)
        # Frame 1: sf1 starves.
        sf1 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf1.hunger_timer = 5.1   # already above threshold
        sim.sailfish.append(sf1)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sailfish_starvations, 1)
        # Frame 2: add sf2 with hunger above threshold; it also starves.
        sf2 = Sailfish(200.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf2.hunger_timer = 5.1
        sim.sailfish.append(sf2)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sailfish_starvations, 2)

    # ── Interaction with sardines ─────────────────────────────────────

    def test_eating_prevents_sailfish_starvation(self):
        """A sailfish that eats a sardine on the same frame it would starve survives."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sailfish_starvation_time=5.0,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 4.9   # manually set near-starvation state
        sim.sailfish.append(sf)
        # Sardine within eat_radius so sailfish eats it on this frame.
        sardine = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        sim.update(0.2)   # hunger before eating = 4.9 + 0.2 = 5.1 > 5.0
        # Eating resets hunger_timer to 0 before starvation check.
        self.assertIn(sf, sim.sailfish,
                      "Sailfish that ate on the starvation frame must survive")
        self.assertEqual(sf.hunger_timer, 0.0)

    def test_sardine_count_unchanged_by_sailfish_starvation(self):
        """Sailfish starvation does not alter the sardine count."""
        from sim.sardine import Sardine

        cfg = _make_sailfish_starvation_sim(starvation_time=5.0)[0].config
        sim = Simulation(cfg)
        sf = sim.sailfish[0] if sim.sailfish else None
        # start with a fresh sim; add sailfish and sardines manually
        from sim.sailfish import Sailfish
        cfg2 = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_initial_hunger_time=5.1,  # already above threshold
            sailfish_starvation_time=5.0,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_eat_radius=1.0,     # tiny: sardine is far away, won't be eaten
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim2 = Simulation(cfg2)
        sf2 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg2, sim2._grid)
        sf2.hunger_timer = 5.1   # manually place above threshold (constructor clamps initial >= starvation)
        sim2.sailfish.append(sf2)
        sardine = Sardine(500.0, 500.0, 0.0, 0.0, cfg2, sim2._grid, sim2._sailfish_grid)
        sim2.sardines.append(sardine)
        sim2.update(1 / 60)   # sailfish starves; sardine untouched
        self.assertEqual(len(sim2.sailfish), 0, "Sailfish must have starved")
        self.assertEqual(len(sim2.sardines), 1,
                         "Sardine count must be unchanged when sailfish starves")


# ── Stage 8 tuning: algae respawn ─────────────────────────────────────────────

def _make_respawn_sim(
    *,
    respawn_rate: float = 5.0,
    max_count: int = 100,
) -> "Simulation":
    """Return a Simulation with NO initial agents but algae respawn enabled.

    No sardines, no sailfish, and no initial algae so we can isolate the
    respawn mechanic without any eating or starvation noise.
    """
    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        algae_respawn_rate=respawn_rate,
        algae_max_count=max_count,
        sardine_starvation_time=0.0,
        sailfish_starvation_time=0.0,
        weight_algae_separation=0.0,
        weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0,
        weight_algae_noise=0.0,
    )
    return Simulation(cfg)


class TestAlgaeRespawn(unittest.TestCase):
    """Tests for algae respawn mechanic (ecosystem tuning, Stage 8 continuation)."""

    def test_algae_respawn_adds_algae(self):
        """After enough simulated time, algae count grows from 0."""
        sim = _make_respawn_sim(respawn_rate=5.0, max_count=100)
        # 1 full second at 60 fps = 60 updates; rate=5 → >=5 algae should appear
        for _ in range(60):
            sim.update(1 / 60)
        self.assertGreater(len(sim.algae), 0,
                           "Algae must appear after respawn rate * time > 1.0")

    def test_algae_respawn_rate_controls_count(self):
        """Higher respawn_rate produces proportionally more algae in the same time."""
        sim_low  = _make_respawn_sim(respawn_rate=2.0,  max_count=1000)
        sim_high = _make_respawn_sim(respawn_rate=10.0, max_count=1000)
        dt = 1 / 60
        for _ in range(60):
            sim_low.update(dt)
            sim_high.update(dt)
        self.assertGreater(
            len(sim_high.algae), len(sim_low.algae),
            "Higher respawn_rate must produce more algae in the same time window",
        )

    def test_algae_capped_at_max_count(self):
        """Algae count never exceeds algae_max_count even with high respawn rate."""
        max_count = 20
        sim = _make_respawn_sim(respawn_rate=1000.0, max_count=max_count)
        for _ in range(60):
            sim.update(1 / 60)
        self.assertLessEqual(len(sim.algae), max_count,
                             "Algae must not exceed algae_max_count")

    def test_algae_respawn_disabled_at_zero(self):
        """respawn_rate=0.0 produces no new algae."""
        sim = _make_respawn_sim(respawn_rate=0.0, max_count=100)
        for _ in range(120):
            sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 0,
                         "No algae must appear when algae_respawn_rate=0.0")

    def test_algae_born_populated_on_spawn_frame(self):
        """_algae_born is non-empty on a frame where algae are spawned."""
        # rate=1000/s guarantees spawn on the very first frame (dt=1/60 → 16.7 algae)
        sim = _make_respawn_sim(respawn_rate=1000.0, max_count=500)
        sim.update(1 / 60)
        self.assertGreater(len(sim._algae_born), 0,
                           "_algae_born must contain new algae on a spawn frame")

    def test_algae_born_cleared_next_frame(self):
        """_algae_born is cleared at the start of the following update()."""
        # Use rate=600/s with dt=1/60 so exactly 10 algae spawn on frame 1
        # (600 * 1/60 = 10.0).  max_count=10 means frame 2 guard fails
        # (len==max) so the respawn pass is skipped and _algae_born stays
        # empty after the frame-start clear.
        sim = _make_respawn_sim(respawn_rate=600.0, max_count=10)
        sim.update(1 / 60)   # spawns 10 algae, populates _algae_born
        self.assertGreater(len(sim._algae_born), 0, "First frame must populate _algae_born")
        sim.update(1 / 60)   # cap already reached; respawn skipped; _algae_born stays clear
        self.assertEqual(len(sim._algae_born), 0,
                         "_algae_born must be cleared at the start of each update()")

    def test_stats_algae_spawned_increments(self):
        """stats_algae_spawned accumulates the total respawned algae count."""
        sim = _make_respawn_sim(respawn_rate=5.0, max_count=1000)
        for _ in range(60):
            sim.update(1 / 60)
        # After 1 second at rate=5, at least 4 should have been spawned
        # (accumulator starts at 0; floating-point may give 4 or 5 depending
        # on exact dt; assertGreater(0) is safe).
        self.assertGreater(sim.stats_algae_spawned, 0,
                           "stats_algae_spawned must reflect spawned algae count")
        self.assertEqual(sim.stats_algae_spawned, len(sim.algae),
                         "stats_algae_spawned must equal total algae when starting from 0")

    def test_ecosystem_config_invariants(self):
        """Default SimConfig: initial hunger times must be strictly less than starvation times."""
        cfg = SimConfig()
        self.assertLess(
            cfg.sardine_initial_hunger_time,
            cfg.sardine_starvation_time,
            "sardine_initial_hunger_time must be less than sardine_starvation_time",
        )
        self.assertLess(
            cfg.sailfish_initial_hunger_time,
            cfg.sailfish_starvation_time,
            "sailfish_initial_hunger_time must be less than sailfish_starvation_time",
        )

    def test_algae_respawn_resumes_after_sardines_eat_below_cap(self):
        """Respawn restarts once the algae count falls back below algae_max_count.

        Verifies that hitting the cap does not permanently disable the respawn
        accumulator — clearing algae (as sardines would do) allows the next
        update() call to spawn new algae immediately.
        """
        sim = _make_respawn_sim(respawn_rate=1000.0, max_count=5)
        # Grow to cap.
        for _ in range(10):
            sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 5, "Algae must reach the cap before depletion")
        # Simulate sardines eating all algae by clearing the list directly.
        sim.algae.clear()
        # One more update: count is below cap so respawn must resume.
        sim.update(1 / 60)
        self.assertGreater(len(sim.algae), 0,
                           "Algae respawn must resume after count drops below cap")


# ── HUD ecosystem metrics ─────────────────────────────────────────────────────

class TestHudEcosystem(unittest.TestCase):
    """Verify the simulation data that the HUD reads is correct.

    The HUD (Hud class in rendering/renderer.py) reads directly from
    Simulation attributes every _REFRESH = 0.25 s and formats them into text.
    These tests exercise the underlying Simulation state only, so no pyglet
    display context is required.

    Coverage
    --------
    * all stats start at zero (nothing to display until events occur)
    * average sardine hunger_timer rises by dt per frame without food
    * average sardine hunger_timer resets to 0 on the frame a sardine eats
    * average sailfish hunger_timer mirrors sardine behaviour
    * sardine count identity: initial + births − (eaten by sailfish + starved)
    * stats are unaffected by simulation.resize()
    """

    # ── Initial state ─────────────────────────────────────────────────

    def test_all_stats_start_at_zero(self):
        """Every ecology stat counter is 0 on a freshly constructed Simulation."""
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
        )
        sim = Simulation(cfg)
        self.assertEqual(sim.stats_algae_eaten,         0)
        self.assertEqual(sim.stats_algae_spawned,       0)
        self.assertEqual(sim.stats_sardine_births,      0)
        self.assertEqual(sim.stats_sardines_eaten,      0)
        self.assertEqual(sim.stats_sardine_starvations, 0)
        self.assertEqual(sim.stats_sailfish_starvations,0)

    # ── Average sardine hunger ────────────────────────────────────────

    def test_avg_sardine_hunger_increases_by_dt(self):
        """Average sardine hunger_timer grows by dt each frame without eating."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
            sardine_starvation_time=0.0,  # disable starvation
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        s1 = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(200.0, 200.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s1.hunger_timer = 0.0
        s2.hunger_timer = 4.0
        sim.sardines.extend([s1, s2])

        dt = 1 / 60
        avg_before = (s1.hunger_timer + s2.hunger_timer) / 2  # 2.0
        sim.update(dt)
        avg_after = sum(s.hunger_timer for s in sim.sardines) / len(sim.sardines)
        self.assertAlmostEqual(avg_after, avg_before + dt, places=9,
                               msg="avg hunger_timer must increase by exactly dt per frame")

    def test_avg_sardine_hunger_resets_on_eating(self):
        """Sardine hunger_timer becomes 0 on the frame it contacts algae."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_eat_radius=20.0,
            algae_respawn_rate=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            sardine_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sardine.hunger_timer = 15.0          # set elevated hunger before eating
        sim.sardines.append(sardine)
        alga = Algae(102.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)

        sim.update(1 / 60)                   # sardine eats alga → hunger_timer = 0
        self.assertAlmostEqual(sardine.hunger_timer, 0.0, places=9,
                               msg="hunger_timer must reset to 0 when sardine eats")

    # ── Average sailfish hunger ───────────────────────────────────────

    def test_avg_sailfish_hunger_increases_by_dt(self):
        """Average sailfish hunger_timer grows by dt each frame without eating."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
            sailfish_starvation_time=0.0,   # disable starvation
            sailfish_initial_hunger_time=0.0,
        )
        sim = Simulation(cfg)
        sf1 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf2 = Sailfish(200.0, 200.0, 0.0, 0.0, cfg, sim._grid)
        sf1.hunger_timer = 5.0
        sf2.hunger_timer = 10.0
        sim.sailfish.extend([sf1, sf2])

        dt = 1 / 60
        avg_before = (sf1.hunger_timer + sf2.hunger_timer) / 2  # 7.5
        sim.update(dt)
        avg_after = sum(sf.hunger_timer for sf in sim.sailfish) / len(sim.sailfish)
        self.assertAlmostEqual(avg_after, avg_before + dt, places=9,
                               msg="avg sailfish hunger_timer must increase by dt per frame")

    def test_avg_sailfish_hunger_resets_on_eating(self):
        """Sailfish hunger_timer becomes 0 on the frame it eats a sardine."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
            sailfish_eat_radius=20.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0,
            sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 30.0               # elevated hunger before eating
        sim.sailfish.append(sf)
        sardine = Sardine(105.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)

        sim.update(1 / 60)                   # sailfish eats sardine → hunger_timer = 0
        self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                               msg="sailfish hunger_timer must reset to 0 when eating")

    # ── Population identity ───────────────────────────────────────────

    def test_sardine_count_consistent_with_births_and_deaths(self):
        """len(sardines) == initial + births − eaten_by_sailfish − starved.

        Runs one frame with four sardines and exercises all three death/birth
        paths at once: one sardine in sailfish range (eaten), one that has
        already exceeded starvation_time (starves), one whose food_counter
        equals the threshold (reproduces), and one that simply idles.
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
            algae_eat_radius=2.0,          # very tight — no accidental algae eating
            sailfish_eat_radius=20.0,
            sardine_starvation_time=5.0,
            sardine_initial_hunger_time=0.0,  # all sardines start at hunger 0
            sardine_eat_threshold=2,
            sardine_min_speed=0.0,
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,  # disable sailfish starvation
            sailfish_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)

        # 1. Sardine eaten by sailfish (placed within sailfish's eat_radius).
        prey = Sardine(500.0, 500.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        # 2. Sardine that starves (hunger already past starvation_time).
        starved = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        starved.hunger_timer = 5.1
        # 3. Sardine that reproduces this frame (food_counter == threshold).
        reproducer = Sardine(300.0, 300.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        reproducer.food_counter = 2         # threshold=2 → will split
        # 4. Idle sardine — nothing happens to it.
        idle = Sardine(700.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([prey, starved, reproducer, idle])

        # Sailfish at (500, 510): 10 px from prey → within eat_radius=20.
        sf = Sailfish(500.0, 510.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)

        initial = 4
        sim.update(1 / 60)

        births      = sim.stats_sardine_births
        eaten       = sim.stats_sardines_eaten
        starved_cnt = sim.stats_sardine_starvations
        n_current   = len(sim.sardines)
        self.assertEqual(
            n_current,
            initial + births - eaten - starved_cnt,
            f"current={n_current} != initial({initial}) + births({births})"
            f" - eaten({eaten}) - starved({starved_cnt})",
        )

    # ── Resize stability ──────────────────────────────────────────────

    def test_stats_unchanged_after_resize(self):
        """Ecology stats survive simulation.resize() without being reset."""
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_eat_radius=20.0,
            algae_respawn_rate=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            sardine_eat_threshold=0,
            sardine_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sardine = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        alga = Algae(102.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)

        sim.update(1 / 60)                   # sardine eats alga → stats_algae_eaten = 1
        self.assertEqual(sim.stats_algae_eaten, 1, "Setup: expected one algae eaten")

        sim.resize(1600, 900)                # window resize must not reset stats

        self.assertEqual(sim.stats_algae_eaten, 1,
                         "stats_algae_eaten must be unchanged after resize()")


# ── Live apply_config for ecosystem parameters ───────────────────────────────

class TestConfigMenuEcosystem(unittest.TestCase):
    """Verify that ecosystem config fields take effect immediately via apply_config.

    These tests exercise ``Simulation.apply_config()`` and the ecology pass in
    ``Simulation.update()`` without requiring a pyglet display context.  Each
    test mutates one ecosystem field through apply_config and asserts the change
    is honoured on the very next update() call.

    Coverage
    --------
    * sardine starvation_time lowered  → sardine starves next frame
    * sardine_eat_threshold lowered    → reproduction fires next frame
    * algae_eat_radius increased       → sardine eats algae it just couldn't reach
    * sailfish_starvation_time lowered → sailfish starves next frame
    * algae_respawn_rate zeroed        → no new algae after the change
    * int fields (sardine_eat_threshold, algae_max_count) stay int after apply_config
    * save_config / load_overrides round-trip preserves int field types and values
    """

    # ── Helper ────────────────────────────────────────────────────────

    @staticmethod
    def _minimal_cfg(**overrides) -> "SimConfig":
        """Base SimConfig with all forces and starvation disabled for isolation."""
        base = dict(
            num_sardines=0, num_sailfish=0, num_algae=0,
            algae_respawn_rate=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            sailfish_starvation_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        base.update(overrides)
        return SimConfig(**base)

    # ── sardine starvation_time ───────────────────────────────────────

    def test_sardine_starvation_time_lowered_triggers_starvation(self):
        """Lowering starvation_time via apply_config causes a sardine to starve
        on the next update() call when its hunger_timer already exceeds the new
        threshold.
        """
        import dataclasses
        from sim.sardine import Sardine

        cfg = self._minimal_cfg(
            sardine_starvation_time=10.0,
            sardine_initial_hunger_time=8.0,  # hunger_timer starts at 8.0
        )
        sim = Simulation(cfg)
        s = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)

        # Lower starvation_time to 5.0; sardine's hunger_timer (8.0) now exceeds it.
        sim.apply_config(dataclasses.replace(cfg, sardine_starvation_time=5.0))

        sim.update(1 / 60)   # hunger_timer becomes ~8.017 > 5.0 → sardine starves

        self.assertEqual(len(sim.sardines), 0,
                         "Sardine should have starved after starvation_time was lowered")
        self.assertEqual(sim.stats_sardine_starvations, 1)

    # ── sardine_eat_threshold ─────────────────────────────────────────

    def test_eat_threshold_lowered_enables_reproduction(self):
        """Lowering sardine_eat_threshold via apply_config allows a sardine with
        sufficient food_counter to reproduce on the next update().
        """
        import dataclasses
        from sim.sardine import Sardine

        cfg = self._minimal_cfg(sardine_eat_threshold=3)
        sim = Simulation(cfg)
        s = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s.food_counter = 2   # below original threshold of 3
        sim.sardines.append(s)

        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 1, "No reproduction with food_counter=2 < 3")

        # Lower threshold to 2 — sardine's food_counter is now sufficient.
        sim.apply_config(dataclasses.replace(sim.config, sardine_eat_threshold=2))
        self.assertIsInstance(sim.config.sardine_eat_threshold, int,
                              "sardine_eat_threshold must remain int after apply_config")

        sim.update(1 / 60)
        self.assertGreater(len(sim.sardines), 1,
                           "Sardine should have reproduced after threshold was lowered")

    # ── algae_eat_radius ──────────────────────────────────────────────

    def test_algae_eat_radius_increased_enables_eating(self):
        """Increasing algae_eat_radius via apply_config allows a sardine to eat
        an alga that was just outside the old radius on the following update().
        """
        import dataclasses

        # sardine at (100, 100), alga at (115, 100): distance = 15 px
        cfg = self._minimal_cfg(algae_eat_radius=10.0)   # alga is outside (15 > 10)
        sim = Simulation(cfg)
        from sim.sardine import Sardine
        s = Sardine(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        alga = Algae(115.0, 100.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)

        sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 1, "Algae outside radius should not be eaten")

        # Increase radius to 20 px — alga is now within range.
        sim.apply_config(dataclasses.replace(sim.config, algae_eat_radius=20.0))

        sim.update(1 / 60)
        self.assertEqual(len(sim.algae), 0,
                         "Algae should have been eaten after radius increase")
        self.assertEqual(s.food_counter, 1)

    # ── sailfish starvation_time ──────────────────────────────────────

    def test_sailfish_starvation_time_lowered_triggers_starvation(self):
        """Lowering sailfish_starvation_time via apply_config causes a sailfish
        to starve on the next update() when its hunger_timer already exceeds the
        new threshold.
        """
        import dataclasses
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg(
            sailfish_starvation_time=10.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 8.0   # manually set above the new starvation threshold
        sim.sailfish.append(sf)

        # Lower starvation_time below the current hunger_timer.
        sim.apply_config(dataclasses.replace(sim.config, sailfish_starvation_time=5.0))

        sim.update(1 / 60)   # hunger_timer ~8.017 > 5.0 → sailfish starves

        self.assertEqual(len(sim.sailfish), 0,
                         "Sailfish should have starved after starvation_time was lowered")
        self.assertEqual(sim.stats_sailfish_starvations, 1)

    # ── algae_respawn_rate ────────────────────────────────────────────

    def test_algae_respawn_rate_zeroed_halts_new_spawns(self):
        """Setting algae_respawn_rate=0 via apply_config stops new algae from
        spawning on subsequent update() calls.
        """
        import dataclasses

        cfg = self._minimal_cfg(
            algae_respawn_rate=30.0, algae_max_count=500, num_algae=10,
        )
        sim = Simulation(cfg)

        # Run a few frames to confirm spawning is active; count may stay
        # bounded by algae_max_count but should not decrease.
        for _ in range(5):
            sim.update(1 / 60)
        count_before = len(sim.algae)

        # Zero out the respawn rate.
        sim.apply_config(dataclasses.replace(sim.config, algae_respawn_rate=0.0))
        count_after_change = len(sim.algae)

        # Run more frames; count must not increase.
        for _ in range(30):
            sim.update(1 / 60)

        self.assertLessEqual(
            len(sim.algae), count_after_change,
            "No new algae should spawn after algae_respawn_rate is set to 0"
        )

    # ── int field type safety ─────────────────────────────────────────

    def test_int_fields_remain_int_after_apply_config(self):
        """sardine_eat_threshold and algae_max_count must be int (not float)
        after a round-trip through apply_config.  ConfigMenu._adjust_value()
        casts back to int before calling dataclasses.replace(); these assertions
        confirm the type contract is preserved end-to-end.
        """
        import dataclasses

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=3,
            algae_max_count=1200,
        )
        sim = Simulation(cfg)

        # Simulate what _adjust_value does: cast to int before replace.
        new_threshold = int(round(3 + 1, 6))   # = 4
        new_max       = int(round(1200 + 50, 6))  # = 1250
        sim.apply_config(dataclasses.replace(
            sim.config,
            sardine_eat_threshold=new_threshold,
            algae_max_count=new_max,
        ))

        self.assertIsInstance(sim.config.sardine_eat_threshold, int,
                              "sardine_eat_threshold must stay int")
        self.assertIsInstance(sim.config.algae_max_count, int,
                              "algae_max_count must stay int")
        self.assertEqual(sim.config.sardine_eat_threshold, 4)
        self.assertEqual(sim.config.algae_max_count, 1250)

    # ── persistence round-trip ────────────────────────────────────────

    def test_persistence_round_trip_for_int_fields(self):
        """save_config followed by load_overrides restores int fields with the
        correct Python type (int, not float) and exact value.

        The test writes to a temporary file so it never touches the real
        boids_overrides.json during the test run.
        """
        import dataclasses
        import tempfile
        import pathlib
        import config.config_io as config_io

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=7,
            algae_max_count=800,
        )

        orig_file = config_io.OVERRIDE_FILE
        try:
            with tempfile.TemporaryDirectory() as tmp:
                config_io.OVERRIDE_FILE = pathlib.Path(tmp) / "test_overrides.json"
                config_io.save_config(cfg)
                overrides = config_io.load_overrides()
        finally:
            config_io.OVERRIDE_FILE = orig_file

        self.assertIn("sardine_eat_threshold", overrides)
        self.assertIn("algae_max_count", overrides)
        self.assertIsInstance(overrides["sardine_eat_threshold"], int,
                              "sardine_eat_threshold must be int after load_overrides()")
        self.assertIsInstance(overrides["algae_max_count"], int,
                              "algae_max_count must be int after load_overrides()")
        self.assertEqual(overrides["sardine_eat_threshold"], 7)
        self.assertEqual(overrides["algae_max_count"], 800)

    # ── algae radii cache ─────────────────────────────────────────────

    def test_apply_config_updates_algae_cached_radii(self):
        """apply_config() refreshes _sep_r2 and _ali_r2 on all live algae.

        Simulation.apply_config() must propagate radius changes to algae's
        cached squared-radius values used by _compute_forces each frame.
        """
        import dataclasses

        cfg = self._minimal_cfg(
            radius_algae_separation=18.0,
            radius_algae_alignment=55.0,
        )
        sim = Simulation(cfg)
        alga = Algae(200.0, 200.0, 0.0, 0.0, cfg, sim._algae_grid)
        sim.algae.append(alga)

        new_sep = 30.0
        new_ali = 70.0
        sim.apply_config(dataclasses.replace(
            sim.config,
            radius_algae_separation=new_sep,
            radius_algae_alignment=new_ali,
        ))

        self.assertAlmostEqual(
            alga._sep_r2, new_sep ** 2, places=9,
            msg="_sep_r2 must update when radius_algae_separation changes via apply_config",
        )
        self.assertAlmostEqual(
            alga._ali_r2, new_ali ** 2, places=9,
            msg="_ali_r2 must update when radius_algae_alignment changes via apply_config",
        )


# ── Stage 8.8: sailfish reproduction ─────────────────────────────────────────

def _make_sailfish_reproduction_sim(
    *,
    threshold: int = 5,
    sailfish_pos: tuple[float, float] = (700.0, 350.0),
    initial_food: float = 0.0,
) -> "Simulation":
    """Return a bare Simulation with one sailfish at a fixed position.

    No sardines, no algae — tests that need eating should manage fish manually.
    Forces are disabled so the sailfish stays near its spawn point.
    Starvation is disabled so food_counter tests are not confounded.
    """
    from sim.sailfish import Sailfish

    cfg = SimConfig(
        num_sardines=0,
        num_sailfish=0,
        num_algae=0,
        sailfish_eat_threshold=threshold,
        sailfish_min_speed=0.0,
        algae_respawn_rate=0.0,
        # Disable starvation so tests are purely about reproduction.
        sardine_starvation_time=0.0,
        sailfish_starvation_time=0.0,
        sailfish_initial_hunger_time=0.0,
        sardine_initial_hunger_time=0.0,
        # Disable all flocking / pursuit forces.
        weight_separation=0.0, weight_alignment=0.0,
        weight_cohesion=0.0, weight_noise=0.0,
        weight_algae_separation=0.0, weight_algae_alignment=0.0,
        weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        weight_pursuit=0.0, weight_flee=0.0,
    )
    sim = Simulation(cfg)
    sf = Sailfish(
        sailfish_pos[0], sailfish_pos[1],
        10.0, 0.0,   # small +x velocity so heading is well-defined
        cfg, sim._grid,
    )
    sf.food_counter = initial_food
    sim.sailfish.append(sf)
    return sim


class TestSailfishReproduction(unittest.TestCase):
    """Tests for Stage 8.8: sailfish food_counter → offspring spawning."""

    # ── Threshold logic ───────────────────────────────────────────────

    def test_no_reproduction_below_threshold(self):
        """food_counter < threshold → no offspring spawned."""
        sim = _make_sailfish_reproduction_sim(threshold=5, initial_food=4.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 1,
                         "No offspring when food_counter < threshold")
        self.assertEqual(sim.stats_sailfish_births, 0)

    def test_reproduction_at_exact_threshold(self):
        """food_counter == threshold → exactly one offspring spawned."""
        sim = _make_sailfish_reproduction_sim(threshold=5, initial_food=5.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2,
                         "Exactly one offspring when food_counter == threshold")
        self.assertEqual(sim.stats_sailfish_births, 1)

    def test_reproduction_above_threshold(self):
        """food_counter > threshold → still exactly one offspring per update."""
        sim = _make_sailfish_reproduction_sim(threshold=5, initial_food=8.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2,
                         "One offspring per update, regardless of excess food")
        self.assertEqual(sim.stats_sailfish_births, 1)

    def test_food_counter_decremented_by_threshold(self):
        """After reproduction food_counter is decremented by threshold (not reset to 0)."""
        threshold = 5
        initial_food = 7.0
        sim = _make_sailfish_reproduction_sim(threshold=threshold, initial_food=initial_food)
        parent = sim.sailfish[0]
        sim.update(1 / 60)
        self.assertAlmostEqual(
            parent.food_counter, initial_food - threshold, places=9,
            msg="food_counter -= threshold preserves leftover progress",
        )

    def test_food_counter_reset_exactly_at_threshold(self):
        """food_counter becomes 0 when it was exactly threshold."""
        sim = _make_sailfish_reproduction_sim(threshold=5, initial_food=5.0)
        parent = sim.sailfish[0]
        sim.update(1 / 60)
        self.assertAlmostEqual(parent.food_counter, 0.0, places=9)

    def test_threshold_zero_disables_reproduction(self):
        """threshold=0 → no reproduction even with high food_counter."""
        sim = _make_sailfish_reproduction_sim(threshold=0, initial_food=100.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 1,
                         "threshold=0 should disable reproduction entirely")
        self.assertEqual(sim.stats_sailfish_births, 0)

    # ── Offspring validity ────────────────────────────────────────────

    def test_offspring_in_world_bounds(self):
        """Offspring position is inside [0, W) × [0, H)."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sailfish[1]
        W = float(sim.config.window_width)
        H = float(sim.config.window_height)
        px, py = float(offspring.position[0]), float(offspring.position[1])
        self.assertGreaterEqual(px, 0.0)
        self.assertLess(px, W)
        self.assertGreaterEqual(py, 0.0)
        self.assertLess(py, H)

    def test_offspring_has_non_nan_velocity(self):
        """Offspring velocity components are finite numbers."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sailfish[1]
        vx = float(offspring.velocity[0])
        vy = float(offspring.velocity[1])
        self.assertTrue(math.isfinite(vx), "offspring vx must be finite")
        self.assertTrue(math.isfinite(vy), "offspring vy must be finite")

    def test_offspring_different_position_from_parent(self):
        """Offspring is not stacked exactly on top of the parent."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        parent = sim.sailfish[0]
        sim.update(1 / 60)
        offspring = sim.sailfish[1]
        dx = float(offspring.position[0]) - float(parent.position[0])
        dy = float(offspring.position[1]) - float(parent.position[1])
        W = float(sim.config.window_width)
        H = float(sim.config.window_height)
        if dx > W / 2:  dx -= W
        if dx < -W / 2: dx += W
        if dy > H / 2:  dy -= H
        if dy < -H / 2: dy += H
        d = math.sqrt(dx * dx + dy * dy)
        self.assertGreater(d, 1.0, "Offspring must not be stacked exactly on parent")

    def test_offspring_is_sailfish_instance(self):
        """Newly born boid is a Sailfish instance."""
        from sim.sailfish import Sailfish
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        self.assertIsInstance(sim.sailfish[1], Sailfish)

    def test_offspring_food_counter_zero(self):
        """Offspring starts with food_counter = 0."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sailfish[1]
        self.assertEqual(offspring.food_counter, 0.0)

    def test_offspring_hunger_timer_starts_at_zero(self):
        """Reproduction offspring always start with hunger_timer = 0."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        offspring = sim.sailfish[1]
        self.assertAlmostEqual(
            offspring.hunger_timer, 0.0, places=9,
            msg="Newborn sailfish must start with hunger_timer = 0",
        )

    # ── _sailfish_born lifecycle ──────────────────────────────────────

    def test_sailfish_born_populated_on_birth_frame(self):
        """_sailfish_born contains the offspring immediately after update()."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim._sailfish_born), 1)
        self.assertIs(sim._sailfish_born[0], sim.sailfish[1])

    def test_sailfish_born_cleared_next_frame(self):
        """_sailfish_born is empty at the start of the next update()."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)   # birth frame
        self.assertTrue(sim._sailfish_born)
        sim.update(1 / 60)   # next frame — cleared at top of update()
        self.assertFalse(sim._sailfish_born,
                         "_sailfish_born must be cleared at the start of each update()")

    def test_stats_sailfish_births_cumulative(self):
        """stats_sailfish_births accumulates across multiple births."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        for _ in range(2):
            sf = Sailfish(700.0, 350.0, 10.0, 0.0, cfg, sim._grid)
            sf.food_counter = 1.0
            sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertEqual(sim.stats_sailfish_births, 2)

    # ── Integration / stability ───────────────────────────────────────

    def test_repeated_reproduction_does_not_corrupt_simulation(self):
        """30 frames after one burst of reproduction keep positions bounded and finite."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0,
            num_sailfish=3,
            num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        # Pre-load food once so reproduction triggers only on the first frame.
        # After that food_counter = 0 on all sailfish; no further births.
        for sf in sim.sailfish:
            sf.food_counter = 1.0
        for _ in range(30):
            sim.update(1 / 60)
        W = float(cfg.window_width)
        H = float(cfg.window_height)
        for sf in sim.sailfish:
            px, py = float(sf.position[0]), float(sf.position[1])
            self.assertTrue(math.isfinite(px) and math.isfinite(py),
                            "sailfish position must be finite after reproduction")
            self.assertGreaterEqual(px, 0.0)
            self.assertLess(px, W)
            self.assertGreaterEqual(py, 0.0)
            self.assertLess(py, H)


# ── Sailfish hunger initialization ───────────────────────────────────────────

class TestSailfishSpawnInit(unittest.TestCase):
    """Verify that every sailfish spawn path always starts with hunger_timer = 0."""

    def test_startup_sailfish_hunger_zero(self):
        """Startup-created sailfish all begin with hunger_timer = 0."""
        cfg = SimConfig(
            num_sailfish=5,
            sailfish_initial_hunger_time=40.0,  # ignored: hunger is always 0
            sailfish_starvation_time=30.0,
        )
        sim = Simulation(cfg)
        for sf in sim.sailfish:
            self.assertAlmostEqual(
                sf.hunger_timer, 0.0, places=9,
                msg=f"startup sailfish hunger_timer must be 0.0, got {sf.hunger_timer}",
            )

    def test_manually_added_sailfish_hunger_zero(self):
        """Sailfish added via add_sailfish() always start with hunger_timer = 0."""
        cfg = SimConfig(
            num_sailfish=0,
            sailfish_initial_hunger_time=40.0,
            sailfish_starvation_time=30.0,
            sailfish_starvation_enabled=True,
        )
        sim = Simulation(cfg)
        sim.add_sailfish(3)
        for sf in sim.sailfish:
            self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                                   msg="add_sailfish() must produce sailfish with hunger_timer = 0")

    def test_reproduction_offspring_hunger_zero(self):
        """Reproduction-created sailfish always start with hunger_timer = 0."""
        sim = _make_sailfish_reproduction_sim(threshold=1, initial_food=1.0)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2, "Expected one offspring")
        offspring = sim.sailfish[1]
        self.assertAlmostEqual(offspring.hunger_timer, 0.0, places=9,
                               msg="Reproduction offspring must start with hunger_timer = 0")

    def test_hunger_zero_regardless_of_initial_hunger_time_config(self):
        """hunger_timer is always 0 regardless of sailfish_initial_hunger_time in config."""
        from sim.sailfish import Sailfish
        for initial_hunger in (0.0, 10.0, 30.0, 50.0, 100.0):
            cfg = SimConfig(
                num_sardines=0, num_sailfish=0, num_algae=0,
                sailfish_initial_hunger_time=initial_hunger,
                sailfish_starvation_time=30.0,
                sailfish_starvation_enabled=True,
            )
            sim = Simulation(cfg)
            sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
            self.assertAlmostEqual(
                sf.hunger_timer, 0.0, places=9,
                msg=f"hunger_timer must be 0 even when sailfish_initial_hunger_time={initial_hunger}",
            )

    def test_fresh_sailfish_survive_starvation_enabled(self):
        """Fresh sailfish (hunger=0) do not die immediately even with starvation enabled."""
        cfg = SimConfig(
            num_sailfish=5,
            sailfish_starvation_time=30.0,
            sailfish_starvation_enabled=True,
            sardine_starvation_time=0.0,
            num_sardines=0,
            num_algae=0,
        )
        sim = Simulation(cfg)
        initial_count = len(sim.sailfish)
        # Run 30 frames (0.5 s simulated) — well below the 30 s starvation time.
        for _ in range(30):
            sim.update(1 / 60)
        self.assertEqual(
            len(sim.sailfish), initial_count,
            "Fresh sailfish must not die in the first 0.5 s with starvation_time=30 s",
        )

    def test_all_spawn_paths_consistent(self):
        """Startup, add_sailfish(), and reproduction all produce hunger_timer = 0."""
        cfg = SimConfig(
            num_sailfish=2,
            sailfish_initial_hunger_time=40.0,
            sailfish_starvation_time=0.0,   # disable starvation so only reproduction is active
            sailfish_eat_threshold=1,
            sailfish_reproduction_enabled=True,
            num_sardines=0, num_algae=0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
            sailfish_min_speed=0.0,
        )
        sim = Simulation(cfg)
        # Path 1: startup
        for sf in sim.sailfish:
            self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                                   msg="startup sailfish: hunger must be 0")
        # Path 2: add_sailfish
        sim.add_sailfish(2)
        for sf in sim.sailfish[2:]:
            self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                                   msg="add_sailfish sailfish: hunger must be 0")
        # Path 3: reproduction (food_counter >= threshold)
        sim.sailfish[0].food_counter = 1.0
        sim.update(1 / 60)
        born = sim._sailfish_born
        self.assertTrue(born, "Expected at least one reproduced offspring")
        for sf in born:
            self.assertAlmostEqual(sf.hunger_timer, 0.0, places=9,
                                   msg="reproduced sailfish: hunger must be 0")


# ── Sailfish ecology toggles ──────────────────────────────────────────────────

class TestSailfishToggles(unittest.TestCase):
    """Tests for sailfish_starvation_enabled and sailfish_reproduction_enabled toggles."""

    # ── sailfish_starvation_enabled ───────────────────────────────────

    def test_starvation_toggle_false_prevents_death(self):
        """sailfish_starvation_enabled=False: sailfish with hunger above threshold survives."""
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_initial_hunger_time=0.0,
            sailfish_starvation_time=5.0,
            sailfish_starvation_enabled=False,  # disabled
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 5.1   # above threshold — would die if starvation were on
        sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertIn(sf, sim.sailfish,
                      "Sailfish must survive when sailfish_starvation_enabled=False")
        self.assertEqual(sim.stats_sailfish_starvations, 0)

    def test_starvation_toggle_true_allows_death(self):
        """sailfish_starvation_enabled=True (default): sailfish above threshold dies."""
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_initial_hunger_time=0.0,
            sailfish_starvation_time=5.0,
            sailfish_starvation_enabled=True,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 5.1
        sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertNotIn(sf, sim.sailfish,
                         "Sailfish must die when sailfish_starvation_enabled=True and hunger exceeds threshold")
        self.assertEqual(sim.stats_sailfish_starvations, 1)

    def test_starvation_toggle_via_apply_config(self):
        """Toggle starvation off via apply_config() takes effect on the next update()."""
        import dataclasses
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_initial_hunger_time=0.0,
            sailfish_starvation_time=5.0,
            sailfish_starvation_enabled=True,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sf.hunger_timer = 5.1
        sim.sailfish.append(sf)

        sim.apply_config(dataclasses.replace(cfg, sailfish_starvation_enabled=False))
        sim.update(1 / 60)
        self.assertIn(sf, sim.sailfish,
                      "apply_config(sailfish_starvation_enabled=False) must disable starvation immediately")

    # ── sailfish_reproduction_enabled ────────────────────────────────

    def test_reproduction_toggle_false_prevents_birth(self):
        """sailfish_reproduction_enabled=False: no offspring even when food_counter >= threshold."""
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_reproduction_enabled=False,  # disabled
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(700.0, 350.0, 10.0, 0.0, cfg, sim._grid)
        sf.food_counter = 1.0   # at threshold, would reproduce if enabled
        sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 1,
                         "No offspring when sailfish_reproduction_enabled=False")
        self.assertEqual(sim.stats_sailfish_births, 0)

    def test_reproduction_toggle_true_allows_birth(self):
        """sailfish_reproduction_enabled=True (default): offspring spawned at threshold."""
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_reproduction_enabled=True,
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(700.0, 350.0, 10.0, 0.0, cfg, sim._grid)
        sf.food_counter = 1.0
        sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2,
                         "Offspring must spawn when sailfish_reproduction_enabled=True")
        self.assertEqual(sim.stats_sailfish_births, 1)

    def test_reproduction_toggle_via_apply_config(self):
        """Toggle reproduction on via apply_config() takes effect on the next update()."""
        import dataclasses
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_reproduction_enabled=False,
            sailfish_min_speed=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sardine_initial_hunger_time=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(700.0, 350.0, 10.0, 0.0, cfg, sim._grid)
        sf.food_counter = 1.0
        sim.sailfish.append(sf)

        # Confirm disabled: no birth.
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 1, "No birth while reproduction is disabled")

        # Re-enable and prime food_counter again.
        sim.apply_config(dataclasses.replace(sim.config, sailfish_reproduction_enabled=True))
        sim.sailfish[0].food_counter = 1.0
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2,
                         "apply_config(sailfish_reproduction_enabled=True) must enable reproduction immediately")

    # ── Toggle persistence round-trip ────────────────────────────────

    def test_toggle_persistence_round_trip(self):
        """Both bool toggles survive a save_config / load_overrides round-trip."""
        import dataclasses
        import tempfile
        import pathlib
        import config.config_io as config_io

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_starvation_enabled=False,
            sailfish_reproduction_enabled=False,
        )

        orig_file = config_io.OVERRIDE_FILE
        try:
            with tempfile.TemporaryDirectory() as tmp:
                config_io.OVERRIDE_FILE = pathlib.Path(tmp) / "test_overrides.json"
                config_io.save_config(cfg)
                overrides = config_io.load_overrides()
        finally:
            config_io.OVERRIDE_FILE = orig_file

        self.assertIn("sailfish_starvation_enabled", overrides)
        self.assertIn("sailfish_reproduction_enabled", overrides)
        self.assertIs(overrides["sailfish_starvation_enabled"], False,
                      "sailfish_starvation_enabled=False must round-trip correctly")
        self.assertIs(overrides["sailfish_reproduction_enabled"], False,
                      "sailfish_reproduction_enabled=False must round-trip correctly")


# ── Mouse predator ─────────────────────────────────────────────────────────

class TestMousePredator(unittest.TestCase):
    """Tests for the mouse-as-predator interaction feature."""

    # ── Helper ───────────────────────────────────────────────────────

    @staticmethod
    def _make_isolated_sardine(
        sardine_x: float,
        sardine_y: float,
        *,
        mouse_predator_enabled: bool = True,
        mouse_predator_radius: float = 100.0,
        mouse_predator_flee_weight: float = 1.0,
        weight_flee: float = 10.0,
        weight_noise: float = 0.0,
    ):
        """Return (sim, sardine) with a single sardine and no other agents.

        Forces and noise are disabled except flee, so the only velocity
        change on update() would come from the mouse predator.
        """
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            mouse_predator_enabled=mouse_predator_enabled,
            mouse_predator_radius=mouse_predator_radius,
            mouse_predator_flee_weight=mouse_predator_flee_weight,
            weight_flee=weight_flee,
            weight_noise=weight_noise,
            weight_separation=0.0, weight_alignment=0.0, weight_cohesion=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0,
            sardine_starvation_time=0.0,
            sardine_min_speed=0.0,
        )
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        sim = Simulation(cfg)
        sardine = Sardine(
            sardine_x, sardine_y, 0.0, 0.0,
            cfg, sim._grid, None, sim._mouse_stimulus,
        )
        sim.sardines.append(sardine)
        return sim, sardine

    # ── Stimulus object ──────────────────────────────────────────────

    def test_simulation_has_mouse_stimulus(self):
        """Simulation always creates a _mouse_stimulus object."""
        sim = Simulation(SimConfig(num_sardines=0, num_sailfish=0, num_algae=0))
        self.assertTrue(hasattr(sim, "_mouse_stimulus"),
                        "Simulation must have a _mouse_stimulus attribute")

    def test_mouse_stimulus_initially_inactive(self):
        """Mouse predator starts inactive."""
        sim = Simulation(SimConfig(num_sardines=0, num_sailfish=0, num_algae=0))
        self.assertFalse(sim._mouse_stimulus.active,
                         "Mouse predator must be inactive at startup")

    def test_set_mouse_predator_activates(self):
        """set_mouse_predator() marks the stimulus active and sets coordinates."""
        sim = Simulation(SimConfig(num_sardines=0, num_sailfish=0, num_algae=0))
        sim.set_mouse_predator(300.0, 400.0)
        ms = sim._mouse_stimulus
        self.assertTrue(ms.active)
        self.assertAlmostEqual(ms.x, 300.0)
        self.assertAlmostEqual(ms.y, 400.0)

    def test_clear_mouse_predator_deactivates(self):
        """clear_mouse_predator() marks the stimulus inactive."""
        sim = Simulation(SimConfig(num_sardines=0, num_sailfish=0, num_algae=0))
        sim.set_mouse_predator(100.0, 200.0)
        sim.clear_mouse_predator()
        self.assertFalse(sim._mouse_stimulus.active,
                         "Mouse predator must become inactive after clear")

    def test_radius_cached_in_stimulus(self):
        """_mouse_stimulus.radius_sq matches mouse_predator_radius ** 2."""
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0,
                        mouse_predator_radius=150.0)
        sim = Simulation(cfg)
        ms = sim._mouse_stimulus
        self.assertAlmostEqual(ms.radius, 150.0)
        self.assertAlmostEqual(ms.radius_sq, 150.0 ** 2)

    # ── Sardine response ─────────────────────────────────────────────

    def test_sardine_flees_active_mouse_predator(self):
        """A sardine south of the mouse cursor (Y below) flees southward."""
        # Sardine at (400, 300); mouse at (400, 380) — 80 px north.
        # After one update the sardine must have a southward velocity (vy < 0).
        sim, sardine = self._make_isolated_sardine(400.0, 300.0)
        sim.set_mouse_predator(400.0, 380.0)
        sim.update(1 / 60)
        vy = float(sardine.velocity[1])
        self.assertLess(vy, 0.0,
                        f"Sardine south of mouse must flee southward; vy={vy:.3f}")

    def test_sardine_flees_left_of_mouse(self):
        """A sardine left of the mouse cursor flees leftward (vx < 0)."""
        # Sardine at (300, 350); mouse at (380, 350) — 80 px to the right.
        sim, sardine = self._make_isolated_sardine(300.0, 350.0)
        sim.set_mouse_predator(380.0, 350.0)
        sim.update(1 / 60)
        vx = float(sardine.velocity[0])
        self.assertLess(vx, 0.0,
                        f"Sardine left of mouse must flee leftward; vx={vx:.3f}")

    def test_sardine_ignores_inactive_mouse(self):
        """Sardine does not flee when mouse predator is inactive."""
        sim, sardine = self._make_isolated_sardine(400.0, 300.0)
        # Do not activate mouse predator.
        sim.update(1 / 60)
        vx = float(sardine.velocity[0])
        vy = float(sardine.velocity[1])
        # With all forces including noise disabled, velocity stays at (0, 0).
        # min_speed=0 so clamping won't introduce motion either.
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 0.0, places=6)

    def test_sardine_ignores_mouse_beyond_radius(self):
        """Sardine does not flee when mouse is outside mouse_predator_radius."""
        # radius=100; mouse 200 px away.
        sim, sardine = self._make_isolated_sardine(
            400.0, 300.0, mouse_predator_radius=100.0
        )
        sim.set_mouse_predator(600.0, 300.0)  # 200 px east — outside radius
        sim.update(1 / 60)
        vx = float(sardine.velocity[0])
        vy = float(sardine.velocity[1])
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 0.0, places=6)

    def test_mouse_predator_disabled_config_suppresses_flee(self):
        """When mouse_predator_enabled=False sardines do not flee the cursor."""
        sim, sardine = self._make_isolated_sardine(
            400.0, 300.0, mouse_predator_enabled=False
        )
        sim.set_mouse_predator(400.0, 380.0)  # 80 px north, inside radius
        sim.update(1 / 60)
        vx = float(sardine.velocity[0])
        vy = float(sardine.velocity[1])
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 0.0, places=6)

    def test_releasing_mouse_stops_effect(self):
        """Clearing the mouse predator mid-run stops the flee force."""
        sim, sardine = self._make_isolated_sardine(400.0, 300.0)
        sim.set_mouse_predator(400.0, 380.0)
        sim.update(1 / 60)
        # Confirm it was fleeing.
        self.assertLess(float(sardine.velocity[1]), 0.0)
        # Zero out velocity to isolate the next frame.
        sardine.velocity[0] = 0.0
        sardine.velocity[1] = 0.0
        # Release mouse.
        sim.clear_mouse_predator()
        sim.update(1 / 60)
        # With no forces and noise=0 velocity should not change.
        self.assertAlmostEqual(float(sardine.velocity[0]), 0.0, places=6)
        self.assertAlmostEqual(float(sardine.velocity[1]), 0.0, places=6)

    # ── Config propagation ───────────────────────────────────────────

    def test_apply_config_updates_radius(self):
        """apply_config() propagates mouse_predator_radius to _mouse_stimulus."""
        import dataclasses
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0,
                        mouse_predator_radius=80.0)
        sim = Simulation(cfg)
        new_cfg = dataclasses.replace(cfg, mouse_predator_radius=200.0)
        sim.apply_config(new_cfg)
        ms = sim._mouse_stimulus
        self.assertAlmostEqual(ms.radius, 200.0)
        self.assertAlmostEqual(ms.radius_sq, 200.0 ** 2)

    def test_apply_config_updates_flee_weight_mult(self):
        """apply_config() propagates mouse_predator_flee_weight to _mouse_stimulus."""
        import dataclasses
        cfg = SimConfig(num_sardines=0, num_sailfish=0, num_algae=0,
                        mouse_predator_flee_weight=1.0)
        sim = Simulation(cfg)
        new_cfg = dataclasses.replace(cfg, mouse_predator_flee_weight=2.5)
        sim.apply_config(new_cfg)
        self.assertAlmostEqual(sim._mouse_stimulus.flee_weight_mult, 2.5)

    def test_config_radius_change_affects_detection(self):
        """Reducing radius below the mouse distance stops the flee effect."""
        import dataclasses
        # Sardine 150 px south of mouse.  Default radius=200 → should flee.
        sim, sardine = self._make_isolated_sardine(
            400.0, 200.0, mouse_predator_radius=200.0
        )
        sim.set_mouse_predator(400.0, 350.0)  # 150 px north
        sim.update(1 / 60)
        self.assertLess(float(sardine.velocity[1]), 0.0, "Should flee at radius=200")
        # Now shrink radius to 100 px (below 150 px distance).
        sardine.velocity[0] = 0.0
        sardine.velocity[1] = 0.0
        sim.apply_config(dataclasses.replace(sim.config, mouse_predator_radius=100.0))
        sim.update(1 / 60)
        self.assertAlmostEqual(float(sardine.velocity[1]), 0.0, places=6,
                               msg="Should not flee after radius shrinks below distance")

    # ── Offspring and coexistence ────────────────────────────────────

    def test_reproduced_sardine_gets_mouse_stimulus(self):
        """Sardine offspring created by reproduction share the mouse stimulus."""
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=1,
            mouse_predator_enabled=True,
            mouse_predator_radius=100.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=10.0,
            sardine_starvation_time=0.0, sardine_min_speed=0.0,
        )
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        sim = Simulation(cfg)
        # Parent sardine already fed enough to reproduce.
        parent = Sardine(400.0, 300.0, 10.0, 0.0, cfg, sim._grid,
                         None, sim._mouse_stimulus)
        parent.food_counter = 1.0
        sim.sardines.append(parent)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sardines), 2, "Expected one offspring")
        offspring = sim.sardines[1]
        self.assertIs(offspring._mouse_stimulus, sim._mouse_stimulus,
                      "Offspring must share the simulation's mouse stimulus")

    def test_sailfish_flee_coexists_with_mouse_predator(self):
        """Sailfish predator flee still works when mouse predator is also active."""
        from sim.sailfish import Sailfish
        from sim.sardine import Sardine
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            radius_flee=200.0, weight_flee=10.0,
            mouse_predator_enabled=True,
            mouse_predator_radius=100.0,
            mouse_predator_flee_weight=1.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_pursuit=0.0,
            sardine_starvation_time=0.0,
            sailfish_starvation_time=0.0,
            sardine_min_speed=0.0,
            sailfish_min_speed=0.0,
        )
        sim = Simulation(cfg)
        # Place sardine at (400, 300).  Sailfish 150 px to the right.
        sardine = Sardine(400.0, 300.0, 0.0, 0.0, cfg, sim._grid,
                          sim._sailfish_grid, sim._mouse_stimulus)
        sim.sardines.append(sardine)
        sailfish = Sailfish(550.0, 300.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sailfish)
        # Mouse is 80 px above — should also trigger flee.
        sim.set_mouse_predator(400.0, 380.0)
        sim.update(1 / 60)
        # Sardine should flee leftward (from sailfish) and/or southward (from mouse).
        vx = float(sardine.velocity[0])
        vy = float(sardine.velocity[1])
        self.assertTrue(vx < 0.0 or vy < 0.0,
                        f"Sardine should flee from sailfish or mouse; vx={vx:.3f} vy={vy:.3f}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
