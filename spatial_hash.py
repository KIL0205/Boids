"""
spatial_hash.py
===============
Uniform-grid spatial hash for O(1) average-case neighbour lookups.

Design
------
The world is divided into square cells of side ``cell_size``.  Each
agent is bucketed into the cell that contains its position.  A radius
query walks the (at most) 3×3 ring of overlapping cells and returns
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
    from boid import Boid

# Cell key is a pair of ints (col, row)
CellKey = tuple[int, int]


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
        """Insert *agent* into the cell that contains its position."""
        col = int(agent.position[0] // self.cell_size) % self._num_cols
        row = int(agent.position[1] // self.cell_size) % self._num_rows
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
        """
        results: list["Boid"] = []
        r2 = radius * radius
        ax = float(agent.position[0])
        ay = float(agent.position[1])
        W  = self.world_width
        H  = self.world_height
        hw = self._half_w
        hh = self._half_h

        for key in self._candidate_cells(ax, ay, radius):
            cell = self._cells.get(key)
            if cell is None:
                continue
            for other in cell:
                if other is agent:
                    continue
                dx = ax - float(other.position[0])
                dy = ay - float(other.position[1])
                # Inline toroidal correction (avoids wrap_delta function-call overhead).
                if dx > hw:   dx -= W
                elif dx < -hw: dx += W
                if dy > hh:   dy -= H
                elif dy < -hh: dy += H
                if dx * dx + dy * dy <= r2:
                    results.append(other)
        return results

    def query_position(
        self, x: float, y: float, radius: float
    ) -> list["Boid"]:
        """Like ``query`` but searches from an arbitrary point (x, y)."""
        results: list["Boid"] = []
        r2 = radius * radius
        W  = self.world_width
        H  = self.world_height
        hw = self._half_w
        hh = self._half_h
        for key in self._candidate_cells(x, y, radius):
            cell = self._cells.get(key)
            if cell is None:
                continue
            for other in cell:
                dx = x - float(other.position[0])
                dy = y - float(other.position[1])
                if dx > hw:   dx -= W
                elif dx < -hw: dx += W
                if dy > hh:   dy -= H
                elif dy < -hh: dy += H
                if dx * dx + dy * dy <= r2:
                    results.append(other)
        return results

    # ── Internals ─────────────────────────────────────────────────────

    def _cell_key(self, x: float, y: float) -> CellKey:
        col = int(x // self.cell_size) % self._num_cols
        row = int(y // self.cell_size) % self._num_rows
        return (col, row)

    def _candidate_cells(
        self, x: float, y: float, radius: float
    ) -> list[CellKey]:
        """Return all cell keys whose cells overlap a circle of *radius*.

        Keys are modulo-wrapped so queries that extend past a world edge
        automatically include cells on the opposite side, making toroidal
        edge-crossing neighbor lookups correct.  A set is used internally
        to deduplicate keys that fold to the same cell (can happen when
        the query radius is comparable to the world size).
        """
        cs      = self.cell_size
        inv_cs  = 1.0 / cs
        min_col = int(math.floor((x - radius) * inv_cs))
        max_col = int(math.floor((x + radius) * inv_cs))
        min_row = int(math.floor((y - radius) * inv_cs))
        max_row = int(math.floor((y + radius) * inv_cs))
        nc = self._num_cols
        nr = self._num_rows
        seen: set[CellKey] = set()
        for col in range(min_col, max_col + 1):
            col_w = col % nc
            for row in range(min_row, max_row + 1):
                seen.add((col_w, row % nr))
        return list(seen)
