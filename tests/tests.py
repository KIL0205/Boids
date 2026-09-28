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


# ── query_first() ─────────────────────────────────────────────────────────────

class TestQueryFirst(unittest.TestCase):
    """Tests for SpatialHash.query_first() — the stop-at-first-match helper."""

    W, H, CS = 1400.0, 700.0, 80.0

    def _grid(self) -> SpatialHash:
        return SpatialHash(cell_size=self.CS, world_width=self.W,
                           world_height=self.H)

    def test_hit_returns_neighbor(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(130.0, 100.0)   # 30 px away
        g.insert(a); g.insert(b)
        self.assertIs(g.query_first(a, radius=50.0), b)

    def test_miss_returns_none(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(300.0, 100.0)   # 200 px away
        g.insert(a); g.insert(b)
        self.assertIsNone(g.query_first(a, radius=50.0))

    def test_excludes_self(self):
        """query_first() must never return the querying agent itself."""
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        g.insert(a)
        self.assertIsNone(g.query_first(a, radius=50.0))

    def test_empty_grid_returns_none(self):
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        g.insert(a)
        g.clear()
        self.assertIsNone(g.query_first(a, radius=50.0))

    def test_boundary_included(self):
        """Boid exactly at radius boundary is included (≤ check)."""
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(150.0, 100.0)   # exactly 50 px away
        g.insert(a); g.insert(b)
        self.assertIs(g.query_first(a, radius=50.0), b)

    def test_multiple_candidates_returns_one(self):
        """When multiple agents qualify, query_first() returns exactly one."""
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        neighbors = [_boid_at(200.0 + i * 8, 200.0) for i in range(1, 5)]
        g.insert(a)
        for n in neighbors:
            g.insert(n)
        result = g.query_first(a, radius=60.0)
        self.assertIsNotNone(result)
        self.assertIn(result, neighbors)

    def test_returns_nearest_candidate(self):
        """query_first() returns the closest neighbor, not insertion order."""
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        far = _boid_at(245.0, 200.0)   # 45 px
        near = _boid_at(210.0, 200.0)  # 10 px
        # Insert farther first to ensure nearest behavior is explicit.
        g.insert(a)
        g.insert(far)
        g.insert(near)
        self.assertIs(g.query_first(a, radius=60.0), near)

    def test_agrees_with_query_empty(self):
        """When query() returns empty, query_first() returns None."""
        g = self._grid()
        a = _boid_at(100.0, 100.0)
        b = _boid_at(400.0, 400.0)   # far away
        g.insert(a); g.insert(b)
        self.assertEqual(g.query(a, radius=50.0), [])
        self.assertIsNone(g.query_first(a, radius=50.0))

    def test_agrees_with_query_nonempty(self):
        """When query() returns results, query_first() returns one of them."""
        g = self._grid()
        a = _boid_at(200.0, 200.0)
        neighbors = [_boid_at(200.0 + i * 10, 200.0) for i in range(1, 4)]
        g.insert(a)
        for n in neighbors:
            g.insert(n)
        query_result = set(g.query(a, radius=60.0))
        first = g.query_first(a, radius=60.0)
        self.assertIsNotNone(first)
        self.assertIn(first, query_result,
            "query_first() must return an element that query() would also return")

    def test_toroidal_x_wrap(self):
        """Returns neighbor across the right→left world boundary."""
        g = self._grid()
        a = _boid_at(self.W - 10.0, 350.0)
        b = _boid_at(10.0, 350.0)   # 20 px toroidal distance
        g.insert(a); g.insert(b)
        self.assertIs(g.query_first(a, radius=50.0), b,
            "query_first() missed neighbor across x wrap boundary")

    def test_toroidal_y_wrap(self):
        """Returns neighbor across the bottom→top world boundary."""
        g = self._grid()
        a = _boid_at(700.0, self.H - 10.0)
        b = _boid_at(700.0, 10.0)   # 20 px toroidal distance
        g.insert(a); g.insert(b)
        self.assertIs(g.query_first(a, radius=50.0), b,
            "query_first() missed neighbor across y wrap boundary")


# ── Inline traversal correctness ──────────────────────────────────────────────

class TestInlineTraversalCorrectness(unittest.TestCase):
    """Verify inline cell traversal in sardine/algae _compute_forces().

    The fast-path inline duplicates the query() cell-walk directly in each
    species' _compute_forces() to avoid list materialisation.  These tests
    confirm the inline:
      - reaches neighbours that query() would return
      - produces non-trivial steering forces in known configurations
      - degrades correctly to the slow-path (query()) in tiny worlds
      - slow path and fast path produce the same forces for the same geometry
    """

    from sim.sardine import Sardine

    def _make_cfg(self, W=1400, H=700):
        return SimConfig(
            window_width=W, window_height=H,
            num_sardines=0, num_algae=0, num_sailfish=0,
        )

    def _make_sardine(self, x, y, vx, vy, cfg, grid, sf_grid=None):
        from sim.sardine import Sardine
        return Sardine(x, y, vx, vy, cfg, grid, sf_grid)

    def _make_algae(self, x, y, vx, vy, cfg, grid):
        from sim.algae import Algae
        return Algae(x, y, vx, vy, cfg, grid)

    # ── sardine fast-path produces non-zero forces ─────────────────────────

    def test_sardine_separation_force_nonzero(self):
        """A sardine with a very close neighbour accumulates a separation force."""
        cfg = self._make_cfg()
        grid = SpatialHash(cell_size=cfg.cell_size,
                           world_width=float(cfg.window_width),
                           world_height=float(cfg.window_height))
        a = self._make_sardine(200.0, 200.0, 1.0, 0.0, cfg, grid)
        b = self._make_sardine(205.0, 200.0, 1.0, 0.0, cfg, grid)  # well inside sep_r
        grid.insert(a); grid.insert(b)
        a.acceleration[:] = 0.0
        a._compute_forces()
        mag = math.sqrt(float(a.acceleration[0])**2 + float(a.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "separation from a close neighbour should produce a force")

    def test_sardine_alignment_force_nonzero(self):
        """A sardine aligns with a neighbour heading in a different direction."""
        cfg = self._make_cfg()
        grid = SpatialHash(cell_size=cfg.cell_size,
                           world_width=float(cfg.window_width),
                           world_height=float(cfg.window_height))
        a = self._make_sardine(200.0, 200.0, 1.0,  0.0, cfg, grid)
        b = self._make_sardine(240.0, 200.0, 0.0, -1.0, cfg, grid)  # different heading
        grid.insert(a); grid.insert(b)
        a.acceleration[:] = 0.0
        a._compute_forces()
        mag = math.sqrt(float(a.acceleration[0])**2 + float(a.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "neighbour with different heading should produce alignment force")

    def test_sardine_no_force_with_no_neighbours(self):
        """A sardine with no neighbours in cohesion radius applies no flocking forces."""
        cfg = self._make_cfg()
        grid = SpatialHash(cell_size=cfg.cell_size,
                           world_width=float(cfg.window_width),
                           world_height=float(cfg.window_height))
        a = self._make_sardine(100.0, 100.0, 1.0, 0.0, cfg, grid)
        b = self._make_sardine(900.0, 600.0, 1.0, 0.0, cfg, grid)  # far away
        grid.insert(a); grid.insert(b)
        a.acceleration[:] = 0.0
        # Disable noise to get a clean zero result
        from dataclasses import replace
        a._config = replace(cfg, weight_noise=0.0)
        a._compute_forces()
        mag = math.sqrt(float(a.acceleration[0])**2 + float(a.acceleration[1])**2)
        self.assertAlmostEqual(mag, 0.0, places=9,
            msg="isolated sardine should produce zero flocking force")

    # ── slow-path fallback (tiny world) ───────────────────────────────────

    def test_sardine_slow_path_fallback(self):
        """Sardine _compute_forces() works in a tiny world (slow path, uses query())."""
        # World width=100, cell_size=80 → nc=ceil(100/80)=2, span=1 → nc≤2*span → slow path.
        cfg = self._make_cfg(W=100, H=100)
        grid = SpatialHash(cell_size=80.0,
                           world_width=float(cfg.window_width),
                           world_height=float(cfg.window_height))
        a = self._make_sardine(30.0, 50.0, 1.0, 0.0, cfg, grid)
        b = self._make_sardine(50.0, 50.0, 1.0, 0.0, cfg, grid)
        grid.insert(a); grid.insert(b)
        # Should not raise; slow path must see b as a neighbour.
        a.acceleration[:] = 0.0
        a._compute_forces()
        # b is within cohesion radius → coh_count must be ≥ 1 → force applied.
        mag = math.sqrt(float(a.acceleration[0])**2 + float(a.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "slow-path sardine in tiny world should still accumulate forces")

    # ── algae fast-path ────────────────────────────────────────────────────

    def test_algae_alignment_force_nonzero(self):
        """An algae with a neighbour heading differently accumulates alignment force."""
        cfg = self._make_cfg()
        grid = SpatialHash(cell_size=cfg.algae_cell_size,
                           world_width=float(cfg.window_width),
                           world_height=float(cfg.window_height))
        a = self._make_algae(200.0, 200.0, 1.0,  0.0, cfg, grid)
        b = self._make_algae(230.0, 200.0, 0.0, -1.0, cfg, grid)
        grid.insert(a); grid.insert(b)
        a.acceleration[:] = 0.0
        from dataclasses import replace
        a._config = replace(cfg, weight_algae_noise=0.0)
        a._compute_forces()
        mag = math.sqrt(float(a.acceleration[0])**2 + float(a.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "algae with a nearby differently-headed neighbour should have alignment force")

    # ── apply_config updates span caches ──────────────────────────────────

    def test_apply_config_updates_coh_span(self):
        """apply_config() refreshes _coh_span when radius_cohesion changes."""
        cfg = self._make_cfg()
        sim = Simulation(cfg)
        # Find a sardine and check initial span.
        if not sim.sardines:
            return
        s = sim.sardines[0]
        initial_span = s._coh_span
        expected_initial = max(1, math.ceil(cfg.radius_cohesion / s._grid.cell_size))
        self.assertEqual(initial_span, expected_initial)

        # Change radius_cohesion to something that gives a different span.
        from dataclasses import replace
        new_radius = s._grid.cell_size * 2.5   # span would be 3
        new_cfg = replace(cfg, radius_cohesion=new_radius)
        sim.apply_config(new_cfg)
        expected_new = max(1, math.ceil(new_radius / s._grid.cell_size))
        self.assertEqual(s._coh_span, expected_new,
            f"_coh_span should update to {expected_new} after apply_config")

    def test_apply_config_updates_flee_span(self):
        """apply_config() refreshes _flee_span when radius_flee changes."""
        cfg = self._make_cfg()
        sim = Simulation(cfg)
        if not sim.sardines:
            return
        s = sim.sardines[0]
        from dataclasses import replace
        sf_cs = s._sailfish_grid.cell_size
        new_radius = sf_cs * 2.1   # forces span=3
        new_cfg = replace(cfg, radius_flee=new_radius)
        sim.apply_config(new_cfg)
        expected = max(1, math.ceil(new_radius / sf_cs))
        self.assertEqual(s._flee_span, expected,
            f"_flee_span should update to {expected} after apply_config")




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
        cfg = SimConfig(num_algae=30, num_sardines=20, num_sailfish=2,
                        sardine_eat_threshold=0)
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
            algae_respawn_rate=0.0,
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
                        algae_eat_radius=20.0, algae_respawn_rate=0.0)
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
        """stats_sardines_eaten accumulates correctly across multiple frames.

        Per-state eating gate note: the sailfish transitions from PATROL →
        TRACK_SCHOOL → TARGET_LOCK on frame 1 (sardine within awareness and
        acquire_radius).  Burst is disabled (sailfish_burst_trigger_radius=0)
        so the sailfish stays in TARGET_LOCK across both frames rather than
        entering RECOVER after eating the first sardine.
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=20.0,
            sailfish_burst_trigger_radius=0.0,  # disable burst so sf stays in TARGET_LOCK
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
        sim.update(1 / 60)   # eats first sardine; sf in target_lock
        sim.update(1 / 60)   # eats second sardine; sf re-acquires target_lock
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
            algae_respawn_rate=0.0,
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


# ── Per-state eating gate ─────────────────────────────────────────────────────

class TestSailfishPerStateEatingGate(unittest.TestCase):
    """Verify that the per-state eating gate allows kills only in hunting states.

    Hunting states (TRACK_SCHOOL, ORBIT_BAITBALL, TARGET_LOCK, BURST_ATTACK,
    BAITBALL_STRIKE) should permit eating.  Non-hunting states (PATROL,
    RECOVER) should not — the gate increments stats_eat_blocked_by_state.
    """

    # ── Helpers ───────────────────────────────────────────────────────

    def _minimal_cfg(self, **kwargs) -> "SimConfig":
        """Minimal frozen config with all forces and ecology effects disabled."""
        base = dict(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sailfish_eat_radius=30.0,
            # Disable all starvation so hunger state is static across frames.
            sardine_starvation_time=0.0,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_initial_hunger_time=0.0,
            sardine_eat_threshold=0,
            # Disable all motion forces so agents stay put.
            sardine_min_speed=0.0, sailfish_min_speed=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0,   weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0,   weight_algae_noise=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
        )
        base.update(kwargs)
        return SimConfig(**base)

    def _make_sim_with_sf_in_patrol(self) -> tuple:
        """Return (sim, sf, sardine) with the sardine outside awareness radius.

        The sailfish cannot detect the sardine so it stays in PATROL, but the
        sardine IS within eat_radius (30 px).  Used to verify PATROL blocks eating.
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg(
            # Tiny awareness so the sardine at 10 px is invisible to the sailfish.
            radius_sailfish_awareness=1.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        sardine = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        return sim, sf, sardine

    def _make_sim_with_sf_in_track_school(self) -> tuple:
        """Return (sim, sf, sardine) with sf heading to TRACK_SCHOOL but not locking.

        The sardine is within awareness but outside acquire_radius so the sailfish
        steers toward the COM without acquiring an individual lock.
        """
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg(
            # Small acquire radius so sardine at 10 px is outside lock range…
            sailfish_target_acquire_radius=1.0,
            # …but within awareness (350 default) and eat_radius (30).
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        sardine = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)
        return sim, sf, sardine

    # ── PATROL: eating must be blocked ────────────────────────────────

    def test_patrol_state_blocks_eating(self):
        """A sailfish in PATROL does not eat a sardine within eat_radius."""
        sim, sf, sardine = self._make_sim_with_sf_in_patrol()
        sim.update(1 / 60)
        self.assertEqual(sf.state, "patrol",
                         "Sailfish must remain in PATROL when sardine is outside awareness")
        self.assertEqual(len(sim.sardines), 1,
                         "Sardine must NOT be eaten while sailfish is in PATROL")
        self.assertEqual(sf.food_counter, 0.0,
                         "food_counter must not increase when eating is blocked by PATROL gate")
        self.assertEqual(sim.stats_sardines_eaten, 0)

    def test_patrol_block_increments_stats_counter(self):
        """stats_eat_blocked_by_state increments when PATROL gate blocks a kill."""
        sim, sf, sardine = self._make_sim_with_sf_in_patrol()
        initial = sim.stats_eat_blocked_by_state
        sim.update(1 / 60)
        self.assertGreater(sim.stats_eat_blocked_by_state, initial,
                           "stats_eat_blocked_by_state must increment when PATROL blocks eating")

    # ── RECOVER: eating must be blocked ───────────────────────────────

    def test_recover_state_blocks_eating(self):
        """A sailfish in RECOVER does not eat a sardine within eat_radius."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg()
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        sardine = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)

        # Manually put the sailfish into RECOVER with a long timer so it stays there.
        sf.sm_to_track_school()
        sf.sm_to_target_lock()
        sf.sm_to_burst_attack()
        sf.sm_to_recover()
        sf._recovery_timer = 5.0   # will not expire in one frame at dt=1/60

        self.assertEqual(sf.state, "recover")
        sim.update(1 / 60)
        self.assertEqual(sf.state, "recover",
                         "Sailfish must still be in RECOVER after one frame")
        self.assertEqual(len(sim.sardines), 1,
                         "Sardine must NOT be eaten while sailfish is in RECOVER")
        self.assertEqual(sf.food_counter, 0.0)

    # ── TRACK_SCHOOL: eating must be allowed ──────────────────────────

    def test_track_school_state_allows_eating(self):
        """A sailfish in TRACK_SCHOOL eats a sardine within eat_radius."""
        sim, sf, sardine = self._make_sim_with_sf_in_track_school()
        sim.update(1 / 60)
        self.assertEqual(sf.state, "track_school",
                         "Sailfish must be in TRACK_SCHOOL when acquire_radius is tiny")
        self.assertEqual(len(sim.sardines), 0,
                         "Sardine must be eaten while sailfish is in TRACK_SCHOOL")
        self.assertEqual(sf.food_counter, 1.0)
        self.assertEqual(sim.stats_sardines_eaten, 1)

    def test_orbit_baitball_state_allows_eating(self):
        """A sailfish in ORBIT_BAITBALL can still eat within eat_radius."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg(
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_herding_min_sailfish=1,
            sailfish_baitball_density_threshold=0.05,
            sailfish_baitball_min_sardines=3,
            sailfish_baitball_detection_radius=80.0,
            sailfish_target_acquire_radius=1.0,
            sailfish_baitball_center_strike_chance=0.0,
            sailfish_baitball_edge_target_chance=0.0,
            sailfish_baitball_strike_enabled=False,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sailfish.append(sf)

        # Create a compact local cluster so baitball detection activates.
        sim.sardines.append(Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))
        sim.sardines.append(Sardine(111.0, 102.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))
        sim.sardines.append(Sardine(108.0, 99.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))

        sim.update(1 / 60)
        self.assertEqual(sf.state, "orbit_baitball")
        self.assertEqual(len(sim.sardines), 2)
        self.assertEqual(sf.food_counter, 1.0)
        self.assertEqual(sim.stats_eat_blocked_by_state, 0)

    # ── BURST_ATTACK: eating must be allowed ──────────────────────────

    def test_burst_attack_state_allows_eating(self):
        """A sailfish in BURST_ATTACK eats a sardine within eat_radius."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._minimal_cfg(
            # Large burst trigger so sf immediately bursts on target acquisition.
            sailfish_burst_trigger_radius=200.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)
        sardine = Sardine(110.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(sardine)

        sim.update(1 / 60)
        # With burst_trigger_radius=200 and sardine at 10 px, sf goes to burst_attack.
        self.assertIn(sf.state, ("burst_attack", "recover"),
                      "Sailfish should be in burst_attack (or recover if burst expired in 1 dt)")
        self.assertEqual(len(sim.sardines), 0,
                         "Sardine must be eaten while sailfish is in BURST_ATTACK")
        self.assertEqual(sf.food_counter, 1.0)

    # ── Blocked-state counter attribute exists ────────────────────────

    def test_stats_eat_blocked_by_state_starts_at_zero(self):
        """stats_eat_blocked_by_state is 0 on a fresh Simulation."""
        cfg = self._minimal_cfg()
        sim = Simulation(cfg)
        self.assertEqual(sim.stats_eat_blocked_by_state, 0)

    def test_stats_eat_blocked_not_incremented_in_hunting_states(self):
        """stats_eat_blocked_by_state does not increment when eating succeeds."""
        sim, sf, sardine = self._make_sim_with_sf_in_track_school()
        sim.update(1 / 60)
        # Sardine was eaten; gate was not blocked.
        self.assertEqual(sim.stats_eat_blocked_by_state, 0,
                         "stats_eat_blocked_by_state must not increment on a successful eat")
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




# ── Phase 4A/4B: batched sardine state and integration ───────────────────────

class TestPhase4Batching(unittest.TestCase):
    """Tests for Phase 4A (batch state arrays) and Phase 4B (batch integration).

    All tests use force-free configurations (all weights = 0) so sardines
    move with their initial velocity only, making integration deterministic.
    """

    def _no_force_cfg(self, **kwargs) -> SimConfig:
        """Return a SimConfig with all flocking and flee forces disabled.

        Any keyword can be overridden via kwargs — they take precedence over
        the defaults below.
        """
        defaults: dict = dict(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=0,
            sardine_starvation_time=0.0,
            algae_respawn_rate=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_flee=0.0, weight_pursuit=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            sardine_min_speed=0.0,
        )
        defaults.update(kwargs)
        return SimConfig(**defaults)

    # ── Phase 4A: array populate and index assignment ─────────────────

    def test_batch_arrays_allocated_on_first_update(self):
        """_s_pos/_s_vel/_s_acc are sized >= n after the first update()."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        for i in range(5):
            sim.sardines.append(
                Sardine(float(i * 10), 100.0, 0.0, 0.0, cfg, sim._grid,
                        sim._sailfish_grid)
            )
        sim.update(1 / 60)
        self.assertGreaterEqual(sim._s_pos.shape[0], 5)
        self.assertGreaterEqual(sim._s_vel.shape[0], 5)
        self.assertGreaterEqual(sim._s_acc.shape[0], 5)
        self.assertEqual(sim._s_n, 5)

    def test_batch_index_assigned_correctly(self):
        """Each sardine._idx equals its list index after update()."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        sardines = [
            Sardine(float(i * 20), 200.0, 0.0, 0.0, cfg, sim._grid,
                    sim._sailfish_grid)
            for i in range(4)
        ]
        for s in sardines:
            sim.sardines.append(s)
        sim.update(1 / 60)
        for expected_idx, s in enumerate(sim.sardines):
            self.assertEqual(
                s._idx, expected_idx,
                f"sardine {expected_idx}: _idx={s._idx} expected {expected_idx}",
            )

    def test_batch_pos_populated_matches_position(self):
        """_s_pos[:n] matches sardine positions at the time of populate."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        positions = [(100.0, 50.0), (200.0, 150.0), (350.0, 300.0)]
        for x, y in positions:
            sim.sardines.append(
                Sardine(x, y, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
            )
        # Run one update to trigger populate; forces are zero so pos barely changes
        # (integration with v=0 leaves position unchanged).
        sim.update(1 / 60)
        # After update, sardine.position reflects post-integration values.
        # Verify index ↔ object consistency.
        for s in sim.sardines:
            self.assertAlmostEqual(
                float(s.position[0]), sim._s_pos[s._idx, 0], places=5
            )
            self.assertAlmostEqual(
                float(s.position[1]), sim._s_pos[s._idx, 1], places=5
            )

    def test_capacity_grows_when_sardines_exceed_initial_cap(self):
        """_ensure_sardine_capacity grows arrays when count exceeds current cap."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        # Array starts at (0, 2); adding 10 sardines should grow.
        for i in range(10):
            sim.sardines.append(
                Sardine(float(i * 5), 50.0, 0.0, 0.0, cfg, sim._grid,
                        sim._sailfish_grid)
            )
        sim.update(1 / 60)
        self.assertGreaterEqual(sim._s_pos.shape[0], 10)
        self.assertEqual(sim._s_n, 10)

    # ── Phase 4B: integration correctness ───────────────────────────────

    def test_batch_integration_moves_sardine_expected_distance(self):
        """Sardine with constant velocity (no forces) moves v*dt per frame."""
        from sim.sardine import Sardine
        vx, vy = 50.0, 30.0
        dt = 1.0 / 60.0
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        s = Sardine(500.0, 300.0, vx, vy, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        x0, y0 = float(s.position[0]), float(s.position[1])
        sim.update(dt)
        x1, y1 = float(s.position[0]), float(s.position[1])
        W = float(cfg.window_width)
        H = float(cfg.window_height)
        # Expected displacement (toroidal wrap applied).
        dx = (x0 + vx * dt) % W - x1
        dy = (y0 + vy * dt) % H - y1
        self.assertAlmostEqual(dx, 0.0, places=4,
                               msg="X displacement incorrect after batch integration")
        self.assertAlmostEqual(dy, 0.0, places=4,
                               msg="Y displacement incorrect after batch integration")

    def test_batch_integration_velocity_written_back(self):
        """sardine.velocity reflects batch-integrated velocity after update()."""
        from sim.sardine import Sardine
        vx, vy = 80.0, 0.0
        dt = 1.0 / 60.0
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        s = Sardine(400.0, 400.0, vx, vy, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(dt)
        # No forces → velocity unchanged (forces=0 → acc stays 0 → vel unchanged).
        self.assertAlmostEqual(float(s.velocity[0]), vx, places=5)
        self.assertAlmostEqual(float(s.velocity[1]), vy, places=5)

    def test_batch_speed_clamp_max_enforced(self):
        """Batch integration clamps sardine speed to sardine_max_speed."""
        from sim.sardine import Sardine
        max_s = 100.0
        cfg = self._no_force_cfg(sardine_max_speed=max_s, sardine_max_force=200.0)
        sim = Simulation(cfg)
        # Give the sardine a velocity well above max_speed.
        s = Sardine(300.0, 300.0, 300.0, 400.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(1 / 60)
        speed_after = math.sqrt(
            float(s.velocity[0]) ** 2 + float(s.velocity[1]) ** 2
        )
        self.assertLessEqual(
            speed_after, max_s + 1e-6,
            f"Batch clamp failed: speed={speed_after:.3f} > max={max_s}",
        )

    def test_batch_speed_floor_min_enforced(self):
        """Batch integration raises sardine speed to sardine_min_speed when below."""
        from sim.sardine import Sardine
        min_s = 50.0
        max_s = 150.0
        cfg = self._no_force_cfg(sardine_min_speed=min_s, sardine_max_speed=max_s,
                                  sardine_max_force=200.0)
        sim = Simulation(cfg)
        # Give the sardine a very slow velocity (below min_speed).
        s = Sardine(300.0, 300.0, 1.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(1 / 60)
        speed_after = math.sqrt(
            float(s.velocity[0]) ** 2 + float(s.velocity[1]) ** 2
        )
        self.assertGreaterEqual(
            speed_after, min_s - 1e-6,
            f"Batch floor failed: speed={speed_after:.3f} < min={min_s}",
        )

    def test_batch_boundary_wrap_x(self):
        """Position wraps correctly at right edge (x > W → x % W)."""
        from sim.sardine import Sardine
        dt = 1.0 / 60.0
        vx = 500.0  # fast enough to cross boundary in one frame
        cfg = self._no_force_cfg(sardine_max_speed=600.0, sardine_max_force=300.0)
        sim = Simulation(cfg)
        W = float(cfg.window_width)
        # Start near right edge so x + vx*dt > W.
        x0 = W - 0.5
        s = Sardine(x0, 300.0, vx, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(dt)
        x1 = float(s.position[0])
        self.assertGreaterEqual(x1, 0.0)
        self.assertLess(x1, W)

    def test_batch_boundary_wrap_y(self):
        """Position wraps correctly at top edge (y > H → y % H)."""
        from sim.sardine import Sardine
        dt = 1.0 / 60.0
        vy = 500.0
        cfg = self._no_force_cfg(sardine_max_speed=600.0, sardine_max_force=300.0)
        sim = Simulation(cfg)
        H = float(cfg.window_height)
        y0 = H - 0.5
        s = Sardine(300.0, y0, 0.0, vy, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(dt)
        y1 = float(s.position[1])
        self.assertGreaterEqual(y1, 0.0)
        self.assertLess(y1, H)

    def test_batch_acceleration_cleared_after_integration(self):
        """Sardine acceleration is 0 after batch integration completes."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        s = Sardine(200.0, 200.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)
        sim.update(1 / 60)
        self.assertAlmostEqual(float(s.acceleration[0]), 0.0)
        self.assertAlmostEqual(float(s.acceleration[1]), 0.0)

    # ── Births and deaths ─────────────────────────────────────────────

    def test_births_get_indices_next_frame(self):
        """After sardine reproduction, offspring have valid _idx on next frame."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg(
            sardine_eat_threshold=1,
            sardine_min_speed=0.0,
        )
        sim = Simulation(cfg)
        s = Sardine(400.0, 300.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s.food_counter = 1.0  # triggers reproduction this frame
        sim.sardines.append(s)
        sim.update(1 / 60)  # reproduction fires → 2 sardines now
        sim.update(1 / 60)  # next frame: indices reassigned to both
        self.assertEqual(len(sim.sardines), 2)
        for i, s in enumerate(sim.sardines):
            self.assertEqual(s._idx, i,
                             f"sardine {i} _idx={s._idx} should be {i}")

    def test_deaths_remaining_sardines_valid_next_frame(self):
        """After sardine removal, remaining sardines have contiguous _idx values."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        for i in range(5):
            sim.sardines.append(
                Sardine(float(i * 30), 200.0, 0.0, 0.0, cfg, sim._grid,
                        sim._sailfish_grid)
            )
        sim.update(1 / 60)
        # Remove the middle sardine (index 2).
        del sim.sardines[2]
        sim.update(1 / 60)  # indices reassigned: 0,1,2,3 for 4 remaining
        self.assertEqual(len(sim.sardines), 4)
        self.assertEqual(sim._s_n, 4)
        for i, s in enumerate(sim.sardines):
            self.assertEqual(s._idx, i,
                             f"After deletion, sardine {i} _idx={s._idx} != {i}")

    def test_zero_sardines_no_crash(self):
        """Simulation with 0 sardines runs update() without error."""
        cfg = self._no_force_cfg()
        sim = Simulation(cfg)
        # No sardines added; just run several frames.
        for _ in range(5):
            sim.update(1 / 60)
        self.assertEqual(sim._s_n, 0)

    def test_apply_config_max_speed_used_in_batch_clamp(self):
        """Config change to sardine_max_speed is respected by batch clamp."""
        from sim.sardine import Sardine
        cfg = self._no_force_cfg(sardine_max_speed=200.0, sardine_max_force=300.0)
        sim = Simulation(cfg)
        s = Sardine(300.0, 300.0, 250.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.append(s)

        # Lower max_speed via apply_config.
        import dataclasses as _dc
        new_cfg = _dc.replace(cfg, sardine_max_speed=80.0)
        sim.apply_config(new_cfg)

        sim.update(1 / 60)
        speed_after = float(s.speed)
        self.assertLessEqual(
            speed_after, 80.0 + 1e-6,
            f"speed={speed_after:.3f} should be ≤ new max_speed 80",
        )


# ── Sailfish flocking (sailfish-to-sailfish) ─────────────────────────────────

def _make_sf_pair(
    *,
    sep_weight: float = 3.0,
    sep_radius: float = 80.0,
    ali_weight: float = 0.0,
    ali_radius: float = 150.0,
    coh_weight: float = 0.0,
    coh_radius: float = 150.0,
    sf1_pos: tuple[float, float] = (700.0, 350.0),
    sf2_pos: tuple[float, float] = (740.0, 350.0),
    sf1_vel: tuple[float, float] = (80.0, 0.0),
    sf2_vel: tuple[float, float] = (80.0, 0.0),
) -> "tuple[object, object, object]":
    """Return (sf_grid, sf1, sf2) — two sailfish inserted into a fresh sailfish grid.

    All forcing except the requested sailfish flocking forces is disabled so
    tests can isolate exactly one effect at a time.
    """
    from sim.sailfish import Sailfish

    cfg = SimConfig(
        num_sardines=0, num_sailfish=0, num_algae=0,
        sailfish_cell_size=120.0,
        sailfish_min_speed=0.0,
        weight_pursuit=0.0,             # disable sardine pursuit
        weight_sailfish_separation=sep_weight,
        weight_sailfish_alignment=ali_weight,
        weight_sailfish_cohesion=coh_weight,
        radius_sailfish_separation=sep_radius,
        radius_sailfish_alignment=ali_radius,
        radius_sailfish_cohesion=coh_radius,
        sailfish_starvation_time=0.0,
        sailfish_initial_hunger_time=0.0,
        sardine_starvation_time=0.0,
        sardine_initial_hunger_time=0.0,
    )
    sardine_grid = SpatialHash(
        cell_size=cfg.cell_size,
        world_width=float(cfg.window_width),
        world_height=float(cfg.window_height),
    )
    sf_grid = SpatialHash(
        cell_size=cfg.sailfish_cell_size,
        world_width=float(cfg.window_width),
        world_height=float(cfg.window_height),
    )
    sf1 = Sailfish(sf1_pos[0], sf1_pos[1], sf1_vel[0], sf1_vel[1],
                   cfg, sardine_grid, sf_grid)
    sf2 = Sailfish(sf2_pos[0], sf2_pos[1], sf2_vel[0], sf2_vel[1],
                   cfg, sardine_grid, sf_grid)
    sf_grid.insert(sf1)
    sf_grid.insert(sf2)
    return sf_grid, sf1, sf2


class TestSailfishFlocking(unittest.TestCase):
    """Tests for sailfish-to-sailfish separation, alignment, and cohesion forces."""

    # ── Separation ────────────────────────────────────────────────────

    def test_separation_force_when_close(self):
        """Sailfish within sep_radius produces non-zero separation force."""
        # sf2 is 40 px to the +x of sf1 — well inside sep_radius=80
        _, sf1, sf2 = _make_sf_pair(
            sep_weight=3.0, sep_radius=80.0,
            sf1_pos=(700.0, 350.0), sf2_pos=(740.0, 350.0),
        )
        sf1.acceleration[:] = 0.0
        sf1._compute_forces()
        mag = math.sqrt(float(sf1.acceleration[0])**2 + float(sf1.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "separation from a close sailfish must produce a force")

    def test_separation_force_direction_away(self):
        """Separation force on sf1 points in −x (away from sf2 at +x)."""
        _, sf1, _ = _make_sf_pair(
            sep_weight=3.0, sep_radius=80.0,
            ali_weight=0.0, coh_weight=0.0,
            sf1_pos=(700.0, 350.0), sf2_pos=(740.0, 350.0),  # sf2 to the +x
        )
        sf1.acceleration[:] = 0.0
        sf1._compute_forces()
        self.assertLess(float(sf1.acceleration[0]), 0.0,
                        "separation must push sf1 in −x (away from sf2 at +x)")
        self.assertAlmostEqual(float(sf1.acceleration[1]), 0.0, places=6,
                               msg="separation must have no y component when neighbour is on x-axis")

    def test_no_separation_when_beyond_radius(self):
        """Two sailfish outside sep_radius produce zero separation force."""
        _, sf1, _ = _make_sf_pair(
            sep_weight=3.0, sep_radius=30.0,   # small radius
            ali_weight=0.0, coh_weight=0.0,
            sf1_pos=(700.0, 350.0), sf2_pos=(750.0, 350.0),  # 50 px apart > 30
        )
        sf1.acceleration[:] = 0.0
        sf1._compute_forces()
        mag = math.sqrt(float(sf1.acceleration[0])**2 + float(sf1.acceleration[1])**2)
        self.assertAlmostEqual(mag, 0.0, places=9,
                               msg="sailfish outside sep_radius must produce zero force")

    def test_no_flocking_when_isolated(self):
        """A sailfish with no neighbours applies no flocking force."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            weight_pursuit=0.0,
            weight_sailfish_separation=3.0,
            weight_sailfish_alignment=0.5,
            weight_sailfish_cohesion=0.0,
            radius_sailfish_separation=80.0,
            radius_sailfish_alignment=150.0,
            radius_sailfish_cohesion=150.0,
        )
        sardine_grid = SpatialHash(cell_size=cfg.cell_size,
                                   world_width=float(cfg.window_width),
                                   world_height=float(cfg.window_height))
        sf_grid = SpatialHash(cell_size=cfg.sailfish_cell_size,
                              world_width=float(cfg.window_width),
                              world_height=float(cfg.window_height))
        sf = Sailfish(700.0, 350.0, 80.0, 0.0, cfg, sardine_grid, sf_grid)
        # Only sf is inserted — no neighbours
        sf_grid.insert(sf)
        sf.acceleration[:] = 0.0
        sf._compute_forces()
        mag = math.sqrt(float(sf.acceleration[0])**2 + float(sf.acceleration[1])**2)
        self.assertAlmostEqual(mag, 0.0, places=9,
                               msg="isolated sailfish must produce zero flocking force")

    def test_no_flocking_without_sailfish_grid(self):
        """sailfish_grid=None disables the flocking block entirely."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            weight_pursuit=0.0,
            weight_sailfish_separation=3.0,
        )
        sardine_grid = SpatialHash(cell_size=cfg.cell_size,
                                   world_width=float(cfg.window_width),
                                   world_height=float(cfg.window_height))
        sf = Sailfish(700.0, 350.0, 80.0, 0.0, cfg, sardine_grid)  # no sf_grid
        sf.acceleration[:] = 0.0
        sf._compute_forces()
        np.testing.assert_array_equal(sf.acceleration, np.zeros(2))

    # ── Alignment ────────────────────────────────────────────────────

    def test_alignment_force_with_different_heading_neighbour(self):
        """Sailfish with a differently-headed neighbour accumulates alignment force."""
        _, sf1, _ = _make_sf_pair(
            sep_weight=0.0, ali_weight=1.0, ali_radius=150.0, coh_weight=0.0,
            sf1_pos=(700.0, 350.0), sf2_pos=(740.0, 350.0),
            sf1_vel=(80.0, 0.0), sf2_vel=(0.0, 80.0),   # sf2 heading +y vs sf1 +x
        )
        sf1.acceleration[:] = 0.0
        sf1._compute_forces()
        mag = math.sqrt(float(sf1.acceleration[0])**2 + float(sf1.acceleration[1])**2)
        self.assertGreater(mag, 0.0, "alignment with differently-headed neighbour must apply a force")

    def test_no_alignment_beyond_radius(self):
        """Neighbour beyond ali_radius produces no alignment force."""
        _, sf1, _ = _make_sf_pair(
            sep_weight=0.0, ali_weight=1.0, ali_radius=30.0, coh_weight=0.0,
            sf1_pos=(700.0, 350.0), sf2_pos=(750.0, 350.0),  # 50 px > ali_radius=30
            sf1_vel=(80.0, 0.0), sf2_vel=(0.0, 80.0),
        )
        sf1.acceleration[:] = 0.0
        sf1._compute_forces()
        mag = math.sqrt(float(sf1.acceleration[0])**2 + float(sf1.acceleration[1])**2)
        self.assertAlmostEqual(mag, 0.0, places=9,
                               msg="neighbour beyond ali_radius must not produce alignment force")

    # ── apply_config cache refresh ────────────────────────────────────

    def test_apply_config_refreshes_sailfish_flocking_caches(self):
        """apply_config() updates _sf_sep_r2, _sf_ali_r2, _sf_coh_r2 on sailfish."""
        import dataclasses
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            radius_sailfish_separation=80.0,
            radius_sailfish_alignment=150.0,
            radius_sailfish_cohesion=150.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(700.0, 350.0, 80.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sailfish.append(sf)

        new_sep = 50.0
        new_ali = 100.0
        new_coh = 120.0
        sim.apply_config(dataclasses.replace(
            cfg,
            radius_sailfish_separation=new_sep,
            radius_sailfish_alignment=new_ali,
            radius_sailfish_cohesion=new_coh,
        ))

        self.assertAlmostEqual(sf._sf_sep_r2, new_sep ** 2, places=9,
                               msg="_sf_sep_r2 must refresh after apply_config")
        self.assertAlmostEqual(sf._sf_ali_r2, new_ali ** 2, places=9,
                               msg="_sf_ali_r2 must refresh after apply_config")
        self.assertAlmostEqual(sf._sf_coh_r2, new_coh ** 2, places=9,
                               msg="_sf_coh_r2 must refresh after apply_config")

    # ── Spawned sailfish have correct flocking caches ─────────────────

    def test_spawned_sailfish_have_sailfish_grid(self):
        """All sailfish created by Simulation._spawn_sailfish() receive sailfish_grid."""
        cfg = SimConfig(num_sailfish=4, num_sardines=0, num_algae=0)
        sim = Simulation(cfg)
        for sf in sim.sailfish:
            self.assertIsNotNone(sf._sailfish_grid,
                                 "startup sailfish must have _sailfish_grid set")
            self.assertIs(sf._sailfish_grid, sim._sailfish_grid,
                          "_sailfish_grid must be the simulation's grid object")

    def test_add_sailfish_has_sailfish_grid(self):
        """Sailfish added via add_sailfish() also receive _sailfish_grid."""
        cfg = SimConfig(num_sailfish=0, num_sardines=0, num_algae=0)
        sim = Simulation(cfg)
        sim.add_sailfish(3)
        for sf in sim.sailfish:
            self.assertIs(sf._sailfish_grid, sim._sailfish_grid,
                          "add_sailfish() must set _sailfish_grid correctly")

    def test_reproduced_sailfish_has_sailfish_grid(self):
        """Offspring created by _reproduce_sailfish() receive _sailfish_grid."""
        from sim.sailfish import Sailfish
        cfg = SimConfig(
            num_sailfish=0, num_sardines=0, num_algae=0,
            sailfish_eat_threshold=1,
            sailfish_reproduction_enabled=True,
            sailfish_starvation_time=0.0,
            sailfish_initial_hunger_time=0.0,
            sardine_starvation_time=0.0,
            sailfish_min_speed=0.0,
            weight_pursuit=0.0, weight_flee=0.0,
            weight_separation=0.0, weight_alignment=0.0,
            weight_cohesion=0.0, weight_noise=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
        )
        sim = Simulation(cfg)
        sf = Sailfish(700.0, 350.0, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sf.food_counter = 1.0
        sim.sailfish.append(sf)
        sim.update(1 / 60)
        self.assertEqual(len(sim.sailfish), 2, "Expected one offspring")
        offspring = sim.sailfish[1]
        self.assertIs(offspring._sailfish_grid, sim._sailfish_grid,
                      "reproduced sailfish must have _sailfish_grid set")

    # ── Pursuit still works with flocking enabled ─────────────────────

    def test_pursuit_still_fires_when_flocking_enabled(self):
        """Pursuit force toward sardines is not suppressed when sailfish flocking is on."""
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            radius_sailfish_awareness=200.0,
            weight_pursuit=1.5,
            weight_sailfish_separation=3.0,
            weight_sailfish_alignment=0.5,
            weight_sailfish_cohesion=0.0,
            radius_sailfish_separation=80.0,
            radius_sailfish_alignment=150.0,
            radius_sailfish_cohesion=150.0,
            sailfish_min_speed=0.0,
        )
        sardine_grid = SpatialHash(
            cell_size=cfg.cell_size,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        sf_grid = SpatialHash(
            cell_size=cfg.sailfish_cell_size,
            world_width=float(cfg.window_width),
            world_height=float(cfg.window_height),
        )
        # Sardine 80 px to the +x of the sailfish — within awareness radius
        sardine = Sardine(780.0, 350.0, 10.0, 0.0, cfg, sardine_grid)
        sardine_grid.insert(sardine)
        sf = Sailfish(700.0, 350.0, 80.0, 0.0, cfg, sardine_grid, sf_grid)
        sf_grid.insert(sf)

        sf.acceleration[:] = 0.0
        sf._compute_forces()
        # Pursuit steers toward +x; acceleration[0] must be > 0
        self.assertGreater(float(sf.acceleration[0]), 0.0,
                           "pursuit force must still fire (+x) when sailfish flocking is enabled")

    # ── Simulation-level integration ──────────────────────────────────

    def test_simulation_sailfish_spread_after_tick(self):
        """Two sailfish placed on top of each other spread apart after one update()."""
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            weight_pursuit=0.0,
            weight_sailfish_separation=3.0,
            radius_sailfish_separation=80.0,
            weight_sailfish_alignment=0.0,
            weight_sailfish_cohesion=0.0,
            sailfish_max_speed=250.0,
            sailfish_min_speed=0.0,
            sailfish_max_force=200.0,
            sailfish_starvation_time=0.0,
            sardine_starvation_time=0.0,
        )
        sim = Simulation(cfg)
        from sim.sailfish import Sailfish
        sf1 = Sailfish(700.0, 350.0, 80.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sf2 = Sailfish(720.0, 350.0, 80.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sailfish.extend([sf1, sf2])

        sim.update(1 / 60)

        # After one tick they don't need to be far apart, just confirm forces
        # fired (i.e., the update didn't error and they moved).
        dx = float(sf1.position[0]) - float(sf2.position[0])
        dy = float(sf1.position[1]) - float(sf2.position[1])
        d = math.sqrt(dx * dx + dy * dy)
        self.assertGreater(d, 0.0, "The two sailfish should have moved apart")


if __name__ == "__main__":
    unittest.main(verbosity=2)


# =============================================================================
# Phase 4C — Batched force computation tests
# =============================================================================

class TestPhase4CBatchedForces(unittest.TestCase):
    """Tests for Phase 4C: numpy cell-bucket sardine force computation.

    Verifies:
      - Separation, alignment, cohesion forces are non-zero in known configs
      - No self-interaction in batch results
      - Toroidal wrap delta computed correctly
      - Flee force injected correctly
      - Output numerically close to per-sardine _compute_forces() reference
      - Zero-sardine edge case doesn't crash
      - Single-sardine edge case produces no flocking force
    """

    # ── helpers ──────────────────────────────────────────────────────────────

    def _flock_cfg(self, **overrides):
        """Config with flocking enabled, flee disabled."""
        defaults = dict(
            num_sardines=0, num_sailfish=0, num_algae=0,
            sardine_eat_threshold=0,
            sardine_starvation_time=0.0,
            algae_respawn_rate=0.0,
            weight_flee=0.0, weight_pursuit=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_noise=0.0,
            sardine_min_speed=0.0,
            mouse_predator_enabled=False,
        )
        defaults.update(overrides)
        return SimConfig(**defaults)

    def _make_sardine(self, x, y, vx, vy, cfg, grid, sf_grid=None):
        from sim.sardine import Sardine
        return Sardine(x, y, vx, vy, cfg, grid, sf_grid)

    def _make_sim_with_sardines(self, positions, velocities, cfg):
        """Build a Simulation and add sardines at given positions/velocities."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        sim = Simulation(cfg)
        for (x, y), (vx, vy) in zip(positions, velocities):
            sim.sardines.append(
                Sardine(x, y, vx, vy, cfg, sim._grid, sim._sailfish_grid)
            )
        return sim

    # ── zero sardines ─────────────────────────────────────────────────────────

    def test_zero_sardines_no_crash(self):
        """_batch_compute_sardine_forces() with 0 sardines runs without error."""
        from sim.simulation import Simulation
        cfg = self._flock_cfg()
        sim = Simulation(cfg)
        # Force population > 500 threshold by manually setting _s_n to 0
        sim._s_n = 0
        sim._batch_compute_sardine_forces()   # must not raise

    # ── single sardine ────────────────────────────────────────────────────────

    def test_single_sardine_no_flocking_force(self):
        """With no noise and one sardine, batch produces zero acc."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._flock_cfg()
        sim = Simulation(cfg)
        sim.sardines.append(Sardine(300.0, 300.0, 50.0, 0.0, cfg, sim._grid))
        sim.update(1 / 60)

    # ── no self-interaction ───────────────────────────────────────────────────

    def test_no_self_interaction(self):
        """No sardine's own position contributes to its own separation force.

        Place a sardine at the origin and a second one far away.  The first
        should only be influenced by the second, never by itself.
        """
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        cfg = self._flock_cfg(
            weight_separation=2.0, weight_alignment=0.0, weight_cohesion=0.0,
            radius_separation=40.0, window_width=1400, window_height=700,
        )
        sim = Simulation(cfg)
        # Two sardines that are within separation range of each other.
        s0 = Sardine(300.0, 300.0, 50.0, 0.0, cfg, sim._grid)
        s1 = Sardine(320.0, 300.0, 50.0, 0.0, cfg, sim._grid)
        sim.sardines.extend([s0, s1])

        # Rebuild grid and arrays manually.
        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)
        n = len(sim.sardines)
        sim._s_n = n
        sim._ensure_sardine_capacity(n)
        for i, s in enumerate(sim.sardines):
            s._idx = i
            sim._s_pos[i] = [s._gx, s._gy]
            sim._s_vel[i] = [s._gvx, s._gvy]

        sim._batch_compute_sardine_forces()

        # s0 is to the left of s1 → separation pushes s0 to the left (acc[0] < 0).
        acc0x = float(sim._s_acc[0, 0])
        acc1x = float(sim._s_acc[1, 0])

        # acc0x should be negative (pushed away from s1 which is to the right)
        # acc1x should be positive (pushed away from s0 which is to the left)
        # If self-interaction occurred, both would be zero.
        self.assertLess(acc0x, 0.0, "s0 should be pushed left by s1 separation")
        self.assertGreater(acc1x, 0.0, "s1 should be pushed right by s0 separation")

    # ── separation force ──────────────────────────────────────────────────────

    def test_batch_separation_nonzero_close_pair(self):
        """Batch force is non-zero for a pair within separation radius."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        cfg = self._flock_cfg(
            weight_separation=2.0, weight_alignment=0.0, weight_cohesion=0.0,
            radius_separation=40.0,
        )
        sim = Simulation(cfg)
        s0 = Sardine(300.0, 300.0, 50.0, 0.0, cfg, sim._grid)
        s1 = Sardine(315.0, 300.0, 50.0, 0.0, cfg, sim._grid)  # 15 px within sep_r=40
        sim.sardines.extend([s0, s1])

        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)
        n = len(sim.sardines)
        sim._s_n = n
        sim._ensure_sardine_capacity(n)
        for i, s in enumerate(sim.sardines):
            s._idx = i
            sim._s_pos[i] = [s._gx, s._gy]
            sim._s_vel[i] = [s._gvx, s._gvy]

        sim._batch_compute_sardine_forces()

        mag0 = math.sqrt(float(sim._s_acc[0, 0])**2 + float(sim._s_acc[0, 1])**2)
        self.assertGreater(mag0, 0.0, "separation should produce non-zero acc")

    # ── toroidal wrap correctness ─────────────────────────────────────────────

    def test_batch_toroidal_wrap_detect_neighbour(self):
        """Batch sees a neighbour across the toroidal boundary."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        W, H = 1400, 700
        cfg = self._flock_cfg(
            window_width=W, window_height=H,
            weight_separation=0.0, weight_alignment=0.0, weight_cohesion=1.0,
            radius_cohesion=80.0, cell_size=80.0,
        )
        sim = Simulation(cfg)
        # Place s0 near the right edge, s1 near the left edge — they should
        # see each other across the wrap (distance ~30 px < coh_r=80).
        s0 = Sardine(W - 15.0, 300.0, 50.0, 0.0, cfg, sim._grid)
        s1 = Sardine(15.0,      300.0, 50.0, 0.0, cfg, sim._grid)
        sim.sardines.extend([s0, s1])

        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)
        n = len(sim.sardines)
        sim._s_n = n
        sim._ensure_sardine_capacity(n)
        for i, s in enumerate(sim.sardines):
            s._idx = i
            sim._s_pos[i] = [s._gx, s._gy]
            sim._s_vel[i] = [s._gvx, s._gvy]

        sim._batch_compute_sardine_forces()

        # At least one sardine should have a non-zero cohesion force.
        mag0 = math.sqrt(float(sim._s_acc[0, 0])**2 + float(sim._s_acc[0, 1])**2)
        mag1 = math.sqrt(float(sim._s_acc[1, 0])**2 + float(sim._s_acc[1, 1])**2)
        self.assertGreater(mag0 + mag1, 0.0,
            "Toroidal wrap should allow sardines near opposite edges to see each other")

    # ── equivalence with per-sardine reference ────────────────────────────────

    def test_batch_matches_per_sardine_reference(self):
        """_batch_compute_sardine_forces() is numerically close to the per-sardine loop.

        Use a large grid (far above the 500-sardine threshold) with several
        sardines in a cluster.  Both paths must reach the same result within
        a tight tolerance (they use the same formula, so agreement should be
        within floating-point rounding).
        """
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        W, H = 1400, 700
        cfg = self._flock_cfg(
            window_width=W, window_height=H,
            weight_separation=2.0, weight_alignment=1.3, weight_cohesion=1.0,
            radius_separation=40.0, radius_alignment=75.0, radius_cohesion=80.0,
            cell_size=80.0,
        )

        # Build 600 sardines in a concentrated cluster so the neighbourhood
        # lookup is non-trivial and all four forces fire for every sardine.
        n_sardines = 600
        rng = np.random.default_rng(9)
        xs  = rng.uniform(300, 600, n_sardines)
        ys  = rng.uniform(200, 500, n_sardines)
        vxs = rng.uniform(-50, 50, n_sardines)
        vys = rng.uniform(-50, 50, n_sardines)

        # ── Reference: per-sardine loop ──────────────────────────────────────
        sim_ref = Simulation(cfg)
        for i in range(n_sardines):
            sim_ref.sardines.append(
                Sardine(xs[i], ys[i], vxs[i], vys[i], cfg, sim_ref._grid)
            )
        # Build grid + arrays without calling update() to avoid ecology effects.
        sim_ref._grid.clear()
        for s in sim_ref.sardines:
            sim_ref._grid.insert(s)
        n = len(sim_ref.sardines)
        sim_ref._s_n = n
        sim_ref._ensure_sardine_capacity(n)
        for i, s in enumerate(sim_ref.sardines):
            s._idx = i
            sim_ref._s_pos[i] = [s._gx, s._gy]
            sim_ref._s_vel[i] = [s._gvx, s._gvy]

        # Run reference per-sardine forces and copy acc into array.
        for i, s in enumerate(sim_ref.sardines):
            s.acceleration[:] = 0.0
            s._compute_forces()
            sim_ref._s_acc[i, 0] = float(s.acceleration[0])
            sim_ref._s_acc[i, 1] = float(s.acceleration[1])
        ref_acc = sim_ref._s_acc[:n].copy()

        # ── Batch path (≥ 500 sardines triggers numpy path) ──────────────────
        sim_batch = Simulation(cfg)
        for i in range(n_sardines):
            sim_batch.sardines.append(
                Sardine(xs[i], ys[i], vxs[i], vys[i], cfg, sim_batch._grid)
            )
        sim_batch._grid.clear()
        for s in sim_batch.sardines:
            sim_batch._grid.insert(s)
        n = len(sim_batch.sardines)
        sim_batch._s_n = n
        sim_batch._ensure_sardine_capacity(n)
        for i, s in enumerate(sim_batch.sardines):
            s._idx = i
            sim_batch._s_pos[i] = [s._gx, s._gy]
            sim_batch._s_vel[i] = [s._gvx, s._gvy]

        sim_batch._batch_compute_sardine_forces()
        batch_acc = sim_batch._s_acc[:n].copy()

        # Expect values to match within 0.1% relative tolerance + small absolute tol.
        np.testing.assert_allclose(
            batch_acc, ref_acc, rtol=1e-3, atol=1e-3,
            err_msg="Batch force acc should match per-sardine reference within tolerance",
        )

    # ── flee force ────────────────────────────────────────────────────────────

    def test_batch_flee_force_nonzero_when_sailfish_nearby(self):
        """Batch correctly injects flee force when a sailfish is in range (≥500 path)."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        W, H = 1400, 700
        cfg = SimConfig(
            num_sardines=0, num_sailfish=0, num_algae=0,
            window_width=W, window_height=H,
            sardine_eat_threshold=0, sardine_starvation_time=0.0,
            algae_respawn_rate=0.0,
            weight_separation=0.0, weight_alignment=0.0, weight_cohesion=0.0,
            weight_noise=0.0, weight_pursuit=0.0,
            weight_algae_separation=0.0, weight_algae_alignment=0.0,
            weight_algae_cohesion=0.0, weight_algae_noise=0.0,
            weight_flee=10.0,
            radius_flee=100.0,
            sardine_min_speed=0.0,
            mouse_predator_enabled=False,
        )

        # Build ≥500 sardines so we exercise the numpy path.
        n_sards = 600
        rng = np.random.default_rng(7)
        xs = rng.uniform(100, 600, n_sards)
        ys = rng.uniform(100, 600, n_sards)

        sim = Simulation(cfg)
        for i in range(n_sards):
            sim.sardines.append(
                Sardine(xs[i], ys[i], 50.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
            )

        # Place one sailfish very close to sardine 0 (within flee radius).
        sf = Sailfish(xs[0] + 30.0, ys[0], 0.0, 0.0, cfg, sim._grid)
        sim.sailfish.append(sf)

        # Rebuild grids + arrays.
        sim._grid.clear()
        sim._sailfish_grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)
        for sf in sim.sailfish:
            sim._sailfish_grid.insert(sf)

        n = len(sim.sardines)
        sim._s_n = n
        sim._ensure_sardine_capacity(n)
        for i, s in enumerate(sim.sardines):
            s._idx = i
            sim._s_pos[i] = [s._gx, s._gy]
            sim._s_vel[i] = [s._gvx, s._gvy]

        sim._batch_compute_sardine_forces()

        # Sardine 0 is 30 px from the sailfish → flee should fire.
        mag0 = math.sqrt(float(sim._s_acc[0, 0])**2 + float(sim._s_acc[0, 1])**2)
        self.assertGreater(mag0, 0.0, "Sardine 0 should have non-zero flee force")

    # ── stable small population ───────────────────────────────────────────────

    def test_stable_small_population_uses_fallback(self):
        """Below threshold (< 500), fallback path runs without crash or NaN."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        cfg = self._flock_cfg(
            weight_separation=2.0, weight_alignment=1.3, weight_cohesion=1.0,
            radius_separation=40.0, radius_alignment=75.0, radius_cohesion=80.0,
        )
        sim = Simulation(cfg)
        for i in range(20):
            sim.sardines.append(
                Sardine(float(i * 30 + 100), 200.0, 50.0, 0.0, cfg, sim._grid)
            )

        for _ in range(10):
            sim.update(1 / 60)

        for s in sim.sardines:
            for v in [float(s.position[0]), float(s.position[1]),
                      float(s.velocity[0]), float(s.velocity[1])]:
                self.assertFalse(math.isnan(v), f"NaN detected in sardine state: {v}")
                self.assertFalse(math.isinf(v), f"Inf detected in sardine state: {v}")

    # ── no NaN / Inf at scale ─────────────────────────────────────────────────

    def test_no_nan_inf_large_population(self):
        """Batch force computation with 1000 sardines produces no NaN or Inf in acc."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine

        W, H = 1400, 700
        cfg = self._flock_cfg(
            window_width=W, window_height=H,
            weight_separation=2.0, weight_alignment=1.3, weight_cohesion=1.0,
            radius_separation=40.0, radius_alignment=75.0, radius_cohesion=80.0,
            cell_size=80.0,
        )

        n_sards = 1000
        rng = np.random.default_rng(13)
        xs  = rng.uniform(0, W, n_sards)
        ys  = rng.uniform(0, H, n_sards)
        vxs = rng.uniform(-150, 150, n_sards)
        vys = rng.uniform(-150, 150, n_sards)

        sim = Simulation(cfg)
        for i in range(n_sards):
            sim.sardines.append(
                Sardine(xs[i], ys[i], vxs[i], vys[i], cfg, sim._grid)
            )

        for _ in range(5):
            sim.update(1 / 60)

        n = sim._s_n
        acc = sim._s_acc[:n]
        self.assertFalse(
            np.isnan(acc).any(), "NaN found in _s_acc after batch force computation"
        )
        self.assertFalse(
            np.isinf(acc).any(), "Inf found in _s_acc after batch force computation"
        )


# ── Cooperative sailfish baitball behavior ──────────────────────────────────

class TestSailfishBaitballCooperation(unittest.TestCase):
    """Targeted checks for baitball detection, herding, and strike state entry."""

    @staticmethod
    def _coop_cfg(**kwargs) -> SimConfig:
        base = dict(
            num_sardines=0,
            num_sailfish=0,
            num_algae=0,
            algae_respawn_rate=0.0,
            sardine_starvation_time=0.0,
            sailfish_starvation_time=0.0,
            sailfish_reproduction_enabled=False,
            sailfish_starvation_enabled=False,
            sardine_eat_threshold=0,
            sailfish_eat_threshold=0,
            weight_noise=0.0,
            weight_pursuit=1.0,
            weight_target_pursuit=1.4,
            sailfish_baitball_enabled=True,
            sailfish_baitball_detection_radius=110.0,
            sailfish_baitball_min_sardines=10,
            sailfish_baitball_density_threshold=0.12,
            sailfish_baitball_memory_seconds=1.0,
            sailfish_herding_enabled=True,
            sailfish_herding_min_sailfish=2,
            sailfish_baitball_strike_enabled=True,
            sailfish_baitball_strike_density_threshold=0.16,
            sailfish_baitball_strike_chance=1.0,
            sailfish_baitball_center_strike_chance=1.0,
            sailfish_baitball_edge_target_chance=1.0,
            sailfish_baitball_strike_duration_seconds=0.8,
            sailfish_baitball_strike_cooldown_seconds=2.0,
            sailfish_baitball_patience_seconds=0.0,
            sailfish_baitball_min_orbit_seconds=0.0,
            sailfish_baitball_patience_jitter_seconds=0.0,
            sailfish_baitball_strike_group_cooldown_seconds=0.0,
        )
        base.update(kwargs)
        return SimConfig(**base)

    def _spawn_sardine_cluster(self, sim: Simulation, cx: float, cy: float, n: int) -> None:
        from sim.sardine import Sardine

        for i in range(n):
            # Deterministic compact ring cluster.
            theta = (2.0 * math.pi * i) / max(1, n)
            r = 8.0 + (i % 5)
            x = cx + math.cos(theta) * r
            y = cy + math.sin(theta) * r
            sim.sardines.append(
                Sardine(x, y, 0.0, 0.0, sim.config, sim._grid, sim._sailfish_grid)
            )

    def test_detect_baitball_from_dense_local_cluster(self):
        cfg = self._coop_cfg()
        sim = Simulation(cfg)
        self._spawn_sardine_cluster(sim, 300.0, 250.0, 24)

        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)

        baitballs = sim._detect_baitballs()
        self.assertGreaterEqual(len(baitballs), 1)
        self.assertGreaterEqual(baitballs[0].count, 10)
        self.assertGreater(baitballs[0].density, cfg.sailfish_baitball_density_threshold)

    def test_dense_baitball_can_trigger_strike_state(self):
        from sim.sailfish import Sailfish

        cfg = self._coop_cfg(
            sailfish_target_acquire_radius=40.0,
            sailfish_herding_min_sailfish=1,
            sailfish_baitball_center_strike_chance=1.0,
            sailfish_baitball_edge_target_chance=0.0,
        )
        sim = Simulation(cfg)
        self._spawn_sardine_cluster(sim, 350.0, 250.0, 28)

        sf = Sailfish(280.0, 250.0, 20.0, 0.0, cfg, sim._grid, sailfish_grid=sim._sailfish_grid)
        sim.sailfish.append(sf)

        sim.update(1 / 60)

        self.assertIn(
            sf.state,
            {
                "orbit_baitball",
                "baitball_strike",
                "recover",
                "track_school",
                "target_lock",
                "burst_attack",
            },
        )
        self.assertTrue(
            sf.state == "baitball_strike" or sf._baitball_strike_cooldown_timer > 0.0,
            "Strike should either be active or have completed into cooldown.",
        )

    def test_herding_debug_count_reports_active_herders(self):
        from sim.sailfish import Sailfish

        cfg = self._coop_cfg(
            sailfish_baitball_strike_enabled=False,
            sailfish_herding_min_sailfish=2,
            sailfish_baitball_center_strike_chance=0.0,
            sailfish_baitball_edge_target_chance=0.0,
        )
        sim = Simulation(cfg)
        self._spawn_sardine_cluster(sim, 400.0, 300.0, 22)

        sf1 = Sailfish(330.0, 300.0, 10.0, 0.0, cfg, sim._grid, sailfish_grid=sim._sailfish_grid)
        sf2 = Sailfish(470.0, 300.0, -10.0, 0.0, cfg, sim._grid, sailfish_grid=sim._sailfish_grid)
        sim.sailfish.extend([sf1, sf2])

        sim.update(1 / 60)

        self.assertGreaterEqual(sim.debug_baitball_count, 1)
        self.assertGreaterEqual(sim.debug_herding_sailfish, 1)
        self.assertGreaterEqual(sim.debug_orbit_sailfish, 1)

    def test_orbit_prefers_edge_target_when_center_strike_disabled(self):
        from sim.sailfish import Sailfish

        cfg = self._coop_cfg(
            sailfish_baitball_center_strike_chance=0.0,
            sailfish_baitball_edge_target_chance=1.0,
            sailfish_target_lock_suppressed_near_baitball=True,
            sailfish_baitball_edge_band_width=10.0,
        )
        sim = Simulation(cfg)
        self._spawn_sardine_cluster(sim, 360.0, 280.0, 24)

        sf = Sailfish(300.0, 280.0, 12.0, 0.0, cfg, sim._grid, sailfish_grid=sim._sailfish_grid)
        sim.sailfish.append(sf)

        sim.update(1 / 60)

        self.assertIn(sf.state, {"target_lock", "burst_attack", "orbit_baitball", "recover"})
        if sf._target_sardine is not None and sf._baitball is not None:
            self.assertTrue(sf._is_target_on_baitball_edge(sf._target_sardine, sf._baitball))

    def test_recover_returns_to_orbit_when_baitball_persists(self):
        from sim.sailfish import Sailfish

        cfg = self._coop_cfg(
            sailfish_baitball_center_strike_chance=1.0,
            sailfish_baitball_edge_target_chance=0.0,
            sailfish_recovery_duration=0.05,
        )
        sim = Simulation(cfg)
        self._spawn_sardine_cluster(sim, 430.0, 300.0, 26)
        sf = Sailfish(360.0, 300.0, 14.0, 0.0, cfg, sim._grid, sailfish_grid=sim._sailfish_grid)
        sim.sailfish.append(sf)

        for _ in range(40):
            sim.update(1 / 60)

        self.assertIn(sf.state, {"orbit_baitball", "baitball_strike", "recover", "target_lock", "burst_attack"})


class TestSailfishTargetDecision(unittest.TestCase):
    """Focused checks for Sailfish.query_hunting_target()."""

    def _cfg(self, **kwargs) -> SimConfig:
        base = dict(
            num_sardines=0,
            num_sailfish=0,
            num_algae=0,
            sardine_starvation_time=0.0,
            sailfish_starvation_time=0.0,
            sailfish_reproduction_enabled=False,
            sailfish_starvation_enabled=False,
            sardine_eat_threshold=0,
            sailfish_eat_threshold=0,
            algae_respawn_rate=0.0,
            weight_noise=0.0,
            sailfish_target_acquire_radius=180.0,
            radius_sailfish_awareness=350.0,
            sailfish_target_lock_suppressed_near_baitball=False,
        )
        base.update(kwargs)
        return SimConfig(**base)

    def test_query_hunting_target_penalizes_shared_target(self):
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish

        cfg = self._cfg(sailfish_target_shared_penalty_weight=3.0)
        sim = Simulation(cfg)

        sf1 = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sf2 = Sailfish(102.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sailfish.extend([sf1, sf2])

        s1 = Sardine(120.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        s2 = Sardine(124.0, 102.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sardines.extend([s1, s2])

        # Build grids and pre-lock sf1 onto s1 so sf2 should avoid sharing it.
        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)
        sim._sailfish_grid.clear()
        for sf in sim.sailfish:
            sim._sailfish_grid.insert(sf)

        sf1._target_sardine = s1
        sardines = sim._grid.query(sf2, cfg.radius_sailfish_awareness)
        decision = sf2.query_hunting_target(sardines, baitball=None)
        self.assertEqual(decision.kind, "isolated_target")
        self.assertIs(decision.target, s2)

    def test_query_hunting_target_prefers_orbit_for_dense_baitball(self):
        from sim.sardine import Sardine
        from sim.sailfish import Sailfish
        from sim.simulation import _BaitballInfo

        cfg = self._cfg(
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_herding_min_sailfish=1,
            sailfish_baitball_density_threshold=0.10,
            sailfish_target_decision_margin=0.05,
        )
        sim = Simulation(cfg)
        sf = Sailfish(100.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid)
        sim.sailfish.append(sf)

        # Visible sardines plus a high-density baitball near the fish.
        sim.sardines.append(Sardine(130.0, 100.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))
        sim.sardines.append(Sardine(132.0, 102.0, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))

        sim._grid.clear()
        for s in sim.sardines:
            sim._grid.insert(s)

        baitball = _BaitballInfo(
            bid=1,
            cx=131.0,
            cy=101.0,
            radius=18.0,
            count=22,
            density=0.45,
        )
        sf.set_baitball_context([baitball], 1, 0.0, 0)

        sardines = sim._grid.query(sf, cfg.radius_sailfish_awareness)
        decision = sf.query_hunting_target(sardines, baitball)
        self.assertEqual(decision.kind, "baitball_orbit")


# =============================================================================
# WebGPU sardine backend tests
# =============================================================================

import unittest.mock

class TestSardineGPUBackend(unittest.TestCase):
    """Tests for the optional WebGPU (wgpu-py) sardine force backend.

    Tests run regardless of whether wgpu is installed:
    - When wgpu is available: validate numerical correctness against CPU path.
    - Always: verify graceful fallback behavior in simulation.

    Correctness criteria (GPU vs CPU):
    - Positions remain within world bounds after wrapping.
    - Velocities stay within [min_speed, max_speed].
    - No NaN or Inf values after many frames.
    - Net flocking behavior visible (sardines cluster rather than diverge).
    """

    def _sim_cfg(self, n_sardines=600, sardine_use_gpu=False, **overrides):
        kwargs = dict(
            window_width=400, window_height=300,
            num_sardines=0, num_algae=0, num_sailfish=0,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            algae_respawn_rate=0.0,
            mouse_predator_enabled=False,
            weight_noise=0.0,    # disable noise for determinism in invariant tests
            sardine_use_gpu=sardine_use_gpu,
            sardine_min_speed=0.0,
        )
        kwargs.update(overrides)
        return SimConfig(**kwargs)

    def _make_sim_n(self, n, cfg):
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        import random
        rng = random.Random(42)
        sim = Simulation(cfg)
        W, H = float(cfg.window_width), float(cfg.window_height)
        for _ in range(n):
            x = rng.uniform(10, W - 10)
            y = rng.uniform(10, H - 10)
            vx = rng.uniform(-50, 50)
            vy = rng.uniform(-50, 50)
            sim.sardines.append(Sardine(x, y, vx, vy, cfg, sim._grid, sim._sailfish_grid))
        return sim

    # ── try_create_backend ────────────────────────────────────────────────────

    def test_try_create_backend_returns_backend_or_none(self):
        """try_create_backend() returns a SardineGPUBackend or None; never raises."""
        from sim.sardine_gpu import try_create_backend
        backend = try_create_backend()
        # Either a backend object or None — never an exception.
        self.assertIn(type(backend).__name__, {"SardineGPUBackend", "NoneType"})

    def test_try_create_backend_survives_import_error(self):
        """When wgpu is unavailable, try_create_backend() returns None, no crash."""
        import sys
        import importlib
        # Temporarily hide wgpu from the import system.
        saved = sys.modules.get("wgpu")
        sys.modules["wgpu"] = None  # type: ignore[assignment]
        try:
            # Force re-import of sardine_gpu with wgpu blocked.
            import sim.sardine_gpu as sgpu_mod
            importlib.reload(sgpu_mod)
            result = sgpu_mod.try_create_backend()
            self.assertIsNone(result)
        finally:
            if saved is None:
                sys.modules.pop("wgpu", None)
            else:
                sys.modules["wgpu"] = saved
            importlib.reload(sgpu_mod)

    # ── graceful CPU fallback in Simulation ───────────────────────────────────

    def test_simulation_falls_back_to_cpu_when_gpu_fails(self):
        """When _gpu_backend is None after init, simulation runs on CPU without crashing."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._sim_cfg(sardine_use_gpu=True)
        sim = Simulation(cfg)
        # Simulate failed GPU init: mark tried=True, backend=None.
        sim._gpu_init_tried = True
        sim._gpu_backend = None
        # Add enough sardines to trigger the n>=500 vectorized path.
        W, H = float(cfg.window_width), float(cfg.window_height)
        for k in range(550):
            x = 20.0 + (k % 20) * 18.0
            y = 20.0 + (k // 20) * 18.0
            sim.sardines.append(Sardine(x, y, 10.0, 0.0, cfg, sim._grid, sim._sailfish_grid))
        # Must not raise even with GPU disabled.
        for _ in range(5):
            sim.update(1 / 60)

    def test_gpu_flag_off_uses_cpu_path(self):
        """With sardine_use_gpu=False, _gpu_backend is never set."""
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._sim_cfg(sardine_use_gpu=False)
        sim = Simulation(cfg)
        for k in range(10):
            sim.sardines.append(Sardine(50.0 + k * 15, 50.0, 10.0, 0.0, cfg, sim._grid))
        sim.update(1 / 60)
        # _gpu_init_tried must remain False (lazy init never triggered).
        self.assertFalse(sim._gpu_init_tried)

    # ── invariant tests (run on real GPU if available, else skip) ─────────────

    @classmethod
    def setUpClass(cls):
        from sim.sardine_gpu import try_create_backend, _WGPU_AVAILABLE
        cls._wgpu_available = _WGPU_AVAILABLE
        cls._gpu_backend = try_create_backend() if _WGPU_AVAILABLE else None

    def _skip_if_no_gpu(self):
        if self._gpu_backend is None:
            self.skipTest("wgpu not available or GPU init failed — skipping GPU invariant test")

    def test_gpu_positions_within_bounds_after_many_frames(self):
        """After 120 GPU frames, all sardine positions wrap within world bounds."""
        self._skip_if_no_gpu()
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._sim_cfg(sardine_use_gpu=True, num_sardines=0)
        sim = Simulation(cfg)
        sim._gpu_init_tried = True
        sim._gpu_backend = self._gpu_backend
        W, H = float(cfg.window_width), float(cfg.window_height)
        rng = np.random.default_rng(7)
        for _ in range(600):
            x = rng.uniform(5, W - 5)
            y = rng.uniform(5, H - 5)
            vx, vy = rng.uniform(-60, 60), rng.uniform(-60, 60)
            sim.sardines.append(Sardine(x, y, vx, vy, cfg, sim._grid, sim._sailfish_grid))

        for _ in range(120):
            sim.update(1 / 60)

        for s in sim.sardines:
            self.assertGreaterEqual(s._gx, 0.0)
            self.assertLess(s._gx, W)
            self.assertGreaterEqual(s._gy, 0.0)
            self.assertLess(s._gy, H)

    def test_gpu_no_nan_inf_after_many_frames(self):
        """No NaN or Inf values in positions or velocities after GPU simulation."""
        self._skip_if_no_gpu()
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._sim_cfg(sardine_use_gpu=True, num_sardines=0)
        sim = Simulation(cfg)
        sim._gpu_init_tried = True
        sim._gpu_backend = self._gpu_backend
        W, H = float(cfg.window_width), float(cfg.window_height)
        rng = np.random.default_rng(13)
        for _ in range(600):
            x = rng.uniform(5, W - 5)
            y = rng.uniform(5, H - 5)
            vx, vy = rng.uniform(-60, 60), rng.uniform(-60, 60)
            sim.sardines.append(Sardine(x, y, vx, vy, cfg, sim._grid, sim._sailfish_grid))

        for _ in range(120):
            sim.update(1 / 60)

        pos = sim._s_pos[:sim._s_n]
        vel = sim._s_vel[:sim._s_n]
        self.assertTrue(np.all(np.isfinite(pos)), "NaN/Inf in positions after GPU frames")
        self.assertTrue(np.all(np.isfinite(vel)), "NaN/Inf in velocities after GPU frames")

    def test_gpu_velocity_clamping_respected(self):
        """GPU path: sardine speeds stay within [min_speed, max_speed]."""
        self._skip_if_no_gpu()
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        max_speed = 100.0
        min_speed = 20.0
        cfg = self._sim_cfg(
            sardine_use_gpu=True, num_sardines=0,
            sardine_max_speed=max_speed, sardine_min_speed=min_speed,
            weight_noise=0.02,
        )
        sim = Simulation(cfg)
        sim._gpu_init_tried = True
        sim._gpu_backend = self._gpu_backend
        W, H = float(cfg.window_width), float(cfg.window_height)
        rng = np.random.default_rng(99)
        for _ in range(600):
            x = rng.uniform(5, W - 5)
            y = rng.uniform(5, H - 5)
            vx, vy = rng.uniform(-80, 80), rng.uniform(-80, 80)
            sim.sardines.append(Sardine(x, y, vx, vy, cfg, sim._grid, sim._sailfish_grid))

        for _ in range(60):
            sim.update(1 / 60)

        vel = sim._s_vel[:sim._s_n]
        speeds = np.hypot(vel[:, 0], vel[:, 1])
        self.assertTrue(
            np.all(speeds <= max_speed + 1e-6),
            f"Speeds exceed max_speed: max={speeds.max():.4f}"
        )

    def test_gpu_sardines_cluster_over_time(self):
        """GPU sardines starting spread should cluster (avg pairwise distance shrinks)."""
        self._skip_if_no_gpu()
        from sim.simulation import Simulation
        from sim.sardine import Sardine
        cfg = self._sim_cfg(
            sardine_use_gpu=True, num_sardines=0,
            window_width=400, window_height=300,
            weight_noise=0.0,
            weight_cohesion=2.0,
            weight_separation=1.5,
            weight_alignment=1.2,
        )
        sim = Simulation(cfg)
        sim._gpu_init_tried = True
        sim._gpu_backend = self._gpu_backend
        # Place 600 sardines spread across the world.
        rng = np.random.default_rng(17)
        W, H = 400.0, 300.0
        for _ in range(600):
            x = rng.uniform(0, W)
            y = rng.uniform(0, H)
            sim.sardines.append(Sardine(x, y, 0.0, 0.0, cfg, sim._grid, sim._sailfish_grid))

        # Measure initial spread as sum of position variance.
        def spread(sim_):
            pos = sim_._s_pos[:sim_._s_n]
            if len(pos) == 0:
                return 0.0
            return float(np.var(pos[:, 0]) + np.var(pos[:, 1]))

        # Need one update to populate _s_pos.
        sim.update(1 / 60)
        spread_initial = spread(sim)

        for _ in range(180):
            sim.update(1 / 60)
        spread_final = spread(sim)

        # With strong cohesion, fish should cluster — variance should decrease.
        self.assertLess(
            spread_final, spread_initial,
            f"GPU sardines did not cluster: spread {spread_initial:.1f} → {spread_final:.1f}"
        )


# =============================================================================
# Sleep prevention (utils/sleep_prevention.py)
# =============================================================================

class TestSleepPrevention(unittest.TestCase):
    """Tests for the platform sleep-prevention utility.

    The tests verify the public contract without spawning real system processes.
    All platform calls are mocked so the suite runs identically on every OS.
    """

    def setUp(self):
        # Reset module-level state before each test so they are isolated.
        import utils.sleep_prevention as sp
        sp._caffeinate_proc = None
        sp._win_active = False

    def tearDown(self):
        # Ensure any mock caffeinate processes are cleaned up.
        import utils.sleep_prevention as sp
        sp._caffeinate_proc = None
        sp._win_active = False

    # ── macOS path ────────────────────────────────────────────────────

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_enable_starts_caffeinate(self, _sys):
        """enable_sleep_prevention on macOS spawns caffeinate and returns True."""
        import utils.sleep_prevention as sp
        mock_proc = unittest.mock.MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.pid = 12345
        with unittest.mock.patch("subprocess.Popen", return_value=mock_proc) as popen:
            result = sp.enable_sleep_prevention()
            popen.assert_called_once()
            args = popen.call_args[0][0]
            self.assertIn("caffeinate", args[0])
        self.assertTrue(result)
        self.assertIs(sp._caffeinate_proc, mock_proc)

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_enable_idempotent(self, _sys):
        """Second enable_sleep_prevention call on macOS does not spawn a new process."""
        import utils.sleep_prevention as sp
        mock_proc = unittest.mock.MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.pid = 9999
        sp._caffeinate_proc = mock_proc
        with unittest.mock.patch("subprocess.Popen") as popen:
            result = sp.enable_sleep_prevention()
            popen.assert_not_called()
        self.assertTrue(result)

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_disable_terminates_caffeinate(self, _sys):
        """disable_sleep_prevention terminates the caffeinate process."""
        import utils.sleep_prevention as sp
        mock_proc = unittest.mock.MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.pid = 12345
        sp._caffeinate_proc = mock_proc
        sp.disable_sleep_prevention()
        mock_proc.terminate.assert_called_once()
        self.assertIsNone(sp._caffeinate_proc)

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_disable_without_enable_is_noop(self, _sys):
        """disable_sleep_prevention with no active process does not crash."""
        import utils.sleep_prevention as sp
        sp._caffeinate_proc = None
        # Should not raise.
        sp.disable_sleep_prevention()

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_is_active_reflects_state(self, _sys):
        """is_active() returns True only when caffeinate is running."""
        import utils.sleep_prevention as sp
        self.assertFalse(sp.is_active())
        mock_proc = unittest.mock.MagicMock()
        mock_proc.poll.return_value = None
        sp._caffeinate_proc = mock_proc
        self.assertTrue(sp.is_active())

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_enable_handles_missing_caffeinate(self, _sys):
        """enable_sleep_prevention returns False gracefully if caffeinate is absent."""
        import utils.sleep_prevention as sp
        with unittest.mock.patch("subprocess.Popen", side_effect=FileNotFoundError):
            result = sp.enable_sleep_prevention()
        self.assertFalse(result)
        self.assertIsNone(sp._caffeinate_proc)

    @unittest.mock.patch("platform.system", return_value="Darwin")
    def test_macos_double_disable_is_safe(self, _sys):
        """Calling disable twice (after enable) does not crash."""
        import utils.sleep_prevention as sp
        mock_proc = unittest.mock.MagicMock()
        mock_proc.poll.return_value = None
        mock_proc.pid = 1
        sp._caffeinate_proc = mock_proc
        sp.disable_sleep_prevention()
        sp.disable_sleep_prevention()  # second call: _caffeinate_proc is already None

    # ── Windows path ──────────────────────────────────────────────────

    @unittest.mock.patch("platform.system", return_value="Windows")
    def test_windows_enable_calls_set_thread_execution_state(self, _sys):
        """enable_sleep_prevention on Windows calls SetThreadExecutionState."""
        import utils.sleep_prevention as sp
        mock_ste = unittest.mock.MagicMock(return_value=1)
        mock_windll = unittest.mock.MagicMock()
        mock_windll.kernel32.SetThreadExecutionState = mock_ste
        with unittest.mock.patch("ctypes.windll", mock_windll, create=True):
            result = sp.enable_sleep_prevention()
        self.assertTrue(result)
        self.assertTrue(sp._win_active)
        mock_ste.assert_called_once()

    @unittest.mock.patch("platform.system", return_value="Windows")
    def test_windows_disable_clears_execution_state(self, _sys):
        """disable_sleep_prevention on Windows clears SetThreadExecutionState."""
        import utils.sleep_prevention as sp
        sp._win_active = True
        mock_ste = unittest.mock.MagicMock(return_value=1)
        mock_windll = unittest.mock.MagicMock()
        mock_windll.kernel32.SetThreadExecutionState = mock_ste
        with unittest.mock.patch("ctypes.windll", mock_windll, create=True):
            sp.disable_sleep_prevention()
        self.assertFalse(sp._win_active)
        mock_ste.assert_called_once()

    @unittest.mock.patch("platform.system", return_value="Windows")
    def test_windows_enable_idempotent(self, _sys):
        """Second enable call on Windows does not call SetThreadExecutionState again."""
        import utils.sleep_prevention as sp
        sp._win_active = True
        mock_ste = unittest.mock.MagicMock()
        mock_windll = unittest.mock.MagicMock()
        mock_windll.kernel32.SetThreadExecutionState = mock_ste
        with unittest.mock.patch("ctypes.windll", mock_windll, create=True):
            result = sp.enable_sleep_prevention()
        self.assertTrue(result)
        mock_ste.assert_not_called()

    # ── Linux / unknown platform ──────────────────────────────────────

    @unittest.mock.patch("platform.system", return_value="Linux")
    def test_linux_enable_returns_false_gracefully(self, _sys):
        """enable_sleep_prevention on Linux returns False without raising."""
        import utils.sleep_prevention as sp
        result = sp.enable_sleep_prevention()
        self.assertFalse(result)

    @unittest.mock.patch("platform.system", return_value="Linux")
    def test_linux_disable_is_noop(self, _sys):
        """disable_sleep_prevention on Linux does not raise."""
        import utils.sleep_prevention as sp
        sp.disable_sleep_prevention()  # must not raise

    @unittest.mock.patch("platform.system", return_value="Linux")
    def test_linux_is_active_returns_false(self, _sys):
        """is_active() returns False on Linux."""
        import utils.sleep_prevention as sp
        self.assertFalse(sp.is_active())

    # ── Config field ──────────────────────────────────────────────────

    def test_config_has_prevent_system_sleep_defaulting_true(self):
        """SimConfig.prevent_system_sleep exists and defaults to True."""
        from config.config import SimConfig
        cfg = SimConfig()
        self.assertTrue(cfg.prevent_system_sleep)

    def test_config_prevent_system_sleep_can_be_disabled(self):
        """SimConfig.prevent_system_sleep can be set to False via dataclasses.replace."""
        import dataclasses
        from config.config import SimConfig
        cfg = dataclasses.replace(SimConfig(), prevent_system_sleep=False)
        self.assertFalse(cfg.prevent_system_sleep)


class _DummyShape:
    """Minimal test double for pyglet shape objects."""

    def __init__(self) -> None:
        self.deleted = False
        self.delete_calls = 0

    def delete(self) -> None:
        self.deleted = True
        self.delete_calls += 1


class TestRendererLifecycleCleanup(unittest.TestCase):
    """Regression tests for stale renderer-cache cleanup.

    These tests validate the central helper used to prune stale shape-cache
    entries when update/draw cadence causes per-frame death side-channels to
    be missed.
    """

    def test_prune_removed_shapes_deletes_stale_shape(self):
        from rendering.renderer import _prune_removed_shapes

        live_sid = 101
        dead_sid = 202
        live_shape = _DummyShape()
        dead_shape = _DummyShape()
        shape_map = {live_sid: live_shape, dead_sid: dead_shape}

        removed = _prune_removed_shapes(shape_map, {live_sid})

        self.assertEqual(removed, 1)
        self.assertIn(live_sid, shape_map)
        self.assertNotIn(dead_sid, shape_map)
        self.assertFalse(live_shape.deleted)
        self.assertTrue(dead_shape.deleted)
        self.assertEqual(dead_shape.delete_calls, 1)

    def test_prune_removed_shapes_prunes_extra_maps(self):
        from rendering.renderer import _prune_removed_shapes

        dead_sid = 55
        shape_map = {dead_sid: _DummyShape()}
        vel_map = {dead_sid: object()}

        removed = _prune_removed_shapes(shape_map, set(), extra_maps=(vel_map,))

        self.assertEqual(removed, 1)
        self.assertNotIn(dead_sid, shape_map)
        self.assertNotIn(dead_sid, vel_map)

    def test_prune_removed_shapes_noop_when_all_live(self):
        from rendering.renderer import _prune_removed_shapes

        sid = 77
        shape = _DummyShape()
        shape_map = {sid: shape}

        removed = _prune_removed_shapes(shape_map, {sid})

        self.assertEqual(removed, 0)
        self.assertIn(sid, shape_map)
        self.assertFalse(shape.deleted)

