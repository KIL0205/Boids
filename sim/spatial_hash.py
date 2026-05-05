"""
sim/spatial_hash.py
===================
Uniform-grid spatial hash for O(1) average-case neighbour lookups.

Design
------
The world is divided into square cells of side ``cell_size``.  Each
agent is bucketed into the cell that contains its position.  A radius
query walks the (at most) 3x3 ring of overlapping cells and returns
every agent whose distance is within the requested radius.

Choosing ``cell_size >= max_perception_radius`` guarantees that a
single-ring lookup (the 8 immediate neighbours + center cell) is
always sufficient — no wider search is needed.

Usage
-----
    grid = SpatialHash(cell_size=80.0, world_width=1400, world_height=700)

    # Rebuild every frame:
    grid.clear()
    for boid in boids:
        grid.insert(boid)

    # Query:
    neighbours = grid.query(boid, radius=60.0)
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sim.boid import Boid

# Cell key is a pair of ints (col, row)
CellKey = tuple[int, int]

# Shared sentinel returned by query() / query_position() when no neighbours
# are found.  Callers must NEVER mutate this list — all existing call sites
# only iterate or test truthiness.  Avoids a fresh [] allocation for every
# empty query (⩾ 1 500 calls/frame when sardines query the sailfish grid
# and few or no sailfish are visible).
_EMPTY_LIST: list = []


class SpatialHash:
    """Uniform-grid spatial hash.

    Parameters
    ----------
    cell_size:
        Side length of each grid cell (pixels).  Should be >= the largest
        perception radius used in the simulation.
    world_width, world_height:
        World dimensions used for toroidal (wrap) distance calculations.
    """

    def __init__(
        self,
        cell_size: float,
        world_width: float,
        world_height: float,
    ) -> None:
        if cell_size <= 0:
            raise ValueError("cell_size must be positive")
        self.cell_size: float    = cell_size
        self.world_width: float  = world_width
        self.world_height: float = world_height

        # Grid dimensions (number of columns / rows).
        # ceil() handles worlds whose width is not an exact multiple of cell_size.
        # Modular-wrapping cell keys against these values is what makes toroidal
        # edge-crossing queries correct: an agent at x≈W-1 must find agents at x≈0.
        self._num_cols: int = max(1, math.ceil(world_width / cell_size))
        self._num_rows: int = max(1, math.ceil(world_height / cell_size))

        # Pre-computed half-dimensions for inline toroidal wrap in query.
        self._half_w: float = world_width  * 0.5
        self._half_h: float = world_height * 0.5

        # Maps cell key → list of Boid objects in that cell.
        # Plain dict; conditional insert avoids per-miss factory allocation.
        self._cells: dict[CellKey, list["Boid"]] = {}

    # ── Mutation ──────────────────────────────────────────────────────

    def resize(self, world_width: float, world_height: float) -> None:
        """Update grid dimensions for a new world size.

        Call this after the window is resized.  Clears all current buckets
        (the grid will be rebuilt on the next ``update()`` frame anyway).
        """
        self.world_width  = world_width
        self.world_height = world_height
        self._num_cols = max(1, math.ceil(world_width  / self.cell_size))
        self._num_rows = max(1, math.ceil(world_height / self.cell_size))
        self._half_w   = world_width  * 0.5
        self._half_h   = world_height * 0.5
        self._cells.clear()

    def clear(self) -> None:
        """Remove all agents from every cell (call once per frame)."""
        self._cells.clear()

    def insert(self, agent: "Boid") -> None:
        """Insert *agent* into the cell that contains its position.

        Pre-caches position and velocity as Python floats on the agent.
        ----------------------------------------------------------------
        ``query()`` and every species ``_compute_forces()`` inner loop need
        each neighbour's position and velocity.  Computing those with numpy
        array access (``other.position[0]``, etc.) routes through numpy's
        scalar type-dispatch machinery on each candidate pair — the dominant
        cost in ``query()``.  Instead, we compute them here once per agent
        per frame (``insert()`` runs during the grid-rebuild pass, before any
        agent update), store them as Python floats on the agent, and let all
        downstream loops read ``other._gx / other._gy / other._gvx /
        other._gvy`` — a single LOAD_ATTR returning a Python float that keeps
        the arithmetic in CPython's native float path.
        """
        apos         = agent.position
        ax           = float(apos[0])
        ay           = float(apos[1])
        agent._gx    = ax           # pre-cache: read by query() + inner loops
        agent._gy    = ay
        avel         = agent.velocity
        agent._gvx   = float(avel[0])   # pre-cache velocity for alignment loops
        agent._gvy   = float(avel[1])
        col = int(ax // self.cell_size) % self._num_cols
        row = int(ay // self.cell_size) % self._num_rows
        key = (col, row)
        cell = self._cells.get(key)
        if cell is None:
            self._cells[key] = [agent]
        else:
            cell.append(agent)

    # ── Query ─────────────────────────────────────────────────────────

    def query(self, agent: "Boid", radius: float) -> list["Boid"]:
        """Return all agents (excluding *agent* itself) within *radius*.

        Uses toroidal distance so agents near a wrapped edge still see
        neighbours on the opposite side correctly.

        Fast path (normal worlds)
        --------------------------
        When ``nc > 2*span`` and ``nr > 2*span`` — true for any world
        larger than three cells in each dimension, which covers every
        real simulation frame — all (2*span+1)² cell keys in the loop
        are guaranteed to be distinct after modular reduction.  No
        deduplication set is needed, so the per-query ``set()``
        allocation and its O(1)-but-not-free hash operations are
        eliminated entirely.

        Slow path (tiny worlds / very large radii)
        -------------------------------------------
        Falls back to the original visited-set deduplication.  Only hit
        during tests that use worlds smaller than three cells.
        """
        results: list["Boid"] = []
        r2     = radius * radius
        apos   = agent.position          # cache: one attr lookup, not two
        ax     = float(apos[0])
        ay     = float(apos[1])
        W      = self.world_width
        H      = self.world_height
        hw     = self._half_w
        hh     = self._half_h
        cs     = self.cell_size
        nc     = self._num_cols
        nr     = self._num_rows
        span   = max(1, math.ceil(radius / cs))
        col_c  = int(ax // cs)
        row_c  = int(ay // cs)

        if nc > 2 * span and nr > 2 * span:
            # Fast path: all cell keys in the (2*span+1)² window are distinct
            # after modular reduction → no visited-set needed.
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    cell = self._cells.get((col, (row_c + dr) % nr))
                    if cell is None:
                        continue
                    for other in cell:
                        if other is agent:
                            continue
                        # Use pre-cached Python floats set by insert() —
                        # eliminates LOAD_ATTR(position) + numpy subscript +
                        # numpy scalar arithmetic on every candidate pair.
                        dx = ax - other._gx
                        dy = ay - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            results.append(other)
        else:
            # Slow path: small world or very large radius — use visited set
            # to deduplicate cell keys that collide under modular reduction.
            visited: set[tuple[int, int]] = set()
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    key = (col, (row_c + dr) % nr)
                    if key in visited:
                        continue
                    visited.add(key)
                    cell = self._cells.get(key)
                    if cell is None:
                        continue
                    for other in cell:
                        if other is agent:
                            continue
                        dx = ax - other._gx
                        dy = ay - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            results.append(other)
        return results if results else _EMPTY_LIST

    def query_position(
        self, x: float, y: float, radius: float
    ) -> list["Boid"]:
        """Like ``query`` but searches from an arbitrary point (x, y).

        Applies the same fast-path / slow-path split as ``query()``.
        """
        results: list["Boid"] = []
        r2    = radius * radius
        W     = self.world_width
        H     = self.world_height
        hw    = self._half_w
        hh    = self._half_h
        cs    = self.cell_size
        nc    = self._num_cols
        nr    = self._num_rows
        span  = max(1, math.ceil(radius / cs))
        col_c = int(x // cs)
        row_c = int(y // cs)

        if nc > 2 * span and nr > 2 * span:
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    cell = self._cells.get((col, (row_c + dr) % nr))
                    if cell is None:
                        continue
                    for other in cell:
                        dx = x - other._gx
                        dy = y - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            results.append(other)
        else:
            visited: set[tuple[int, int]] = set()
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    key = (col, (row_c + dr) % nr)
                    if key in visited:
                        continue
                    visited.add(key)
                    cell = self._cells.get(key)
                    if cell is None:
                        continue
                    for other in cell:
                        dx = x - other._gx
                        dy = y - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            results.append(other)
        return results if results else _EMPTY_LIST

    def query_first(self, agent: "Boid", radius: float) -> "Boid | None":
        """Return the first neighbour within *radius*, or ``None``.

        Stops as soon as one qualifying neighbour is found — avoids building
        a full result list when only the first match is needed (e.g. the
        sailfish one-kill-per-tick ecology pass).

        Applies the same fast-path / slow-path split as ``query()``.
        """
        r2    = radius * radius
        apos  = agent.position
        ax    = float(apos[0])
        ay    = float(apos[1])
        W     = self.world_width
        H     = self.world_height
        hw    = self._half_w
        hh    = self._half_h
        cs    = self.cell_size
        nc    = self._num_cols
        nr    = self._num_rows
        span  = max(1, math.ceil(radius / cs))
        col_c = int(ax // cs)
        row_c = int(ay // cs)

        if nc > 2 * span and nr > 2 * span:
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    cell = self._cells.get((col, (row_c + dr) % nr))
                    if cell is None:
                        continue
                    for other in cell:
                        if other is agent:
                            continue
                        dx = ax - other._gx
                        dy = ay - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            return other
        else:
            visited: set[tuple[int, int]] = set()
            for dc in range(-span, span + 1):
                col = (col_c + dc) % nc
                for dr in range(-span, span + 1):
                    key = (col, (row_c + dr) % nr)
                    if key in visited:
                        continue
                    visited.add(key)
                    cell = self._cells.get(key)
                    if cell is None:
                        continue
                    for other in cell:
                        if other is agent:
                            continue
                        dx = ax - other._gx
                        dy = ay - other._gy
                        if dx > hw:    dx -= W
                        elif dx < -hw: dx += W
                        if dy > hh:    dy -= H
                        elif dy < -hh: dy += H
                        if dx * dx + dy * dy <= r2:
                            return other
        return None

    def _candidate_cells(
        self, x: float, y: float, radius: float
    ) -> set[CellKey]:
        """Return the deduplicated set of cell keys near (x, y) within *radius*.

        Uses a set for O(1) deduplication; the previous list-based
        implementation used an O(n) ``not in`` scan per key.
        """
        cs    = self.cell_size
        span  = max(1, math.ceil(radius / cs))
        col_c = int(x // cs)
        row_c = int(y // cs)
        nc    = self._num_cols
        nr    = self._num_rows
        keys: set[CellKey] = set()
        for dc in range(-span, span + 1):
            col = (col_c + dc) % nc
            for dr in range(-span, span + 1):
                keys.add((col, (row_c + dr) % nr))
        return keys
