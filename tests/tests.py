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
        self.assertEqual(len(self.sim.sardines), CONFIG.num_sardines)

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
        self.assertEqual(d["count"], CONFIG.num_sardines)

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
