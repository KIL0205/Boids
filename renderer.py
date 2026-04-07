"""
renderer.py
===========
Pyglet 2.x drawing layer for the sardine / sailfish simulation.

Draws each sardine as a heading-oriented filled triangle inside a single
pyglet Batch.  The batch is submitted to the GPU with one draw call per
frame, so this layer has minimal overhead for up to ~500 agents.

Interface
---------
* ``Renderer(config, window)``  — create once **after** the pyglet Window
  (the GL context must exist before ``glClearColor`` is called).
* ``renderer.reset(sardines)``  — call once after ``Simulation`` is created,
  and again if the population ever changes (Phase 2 spawn / despawn).
* ``renderer.draw(simulation)`` — call every frame inside ``on_draw``,
  **after** ``window.clear()``.

This module reads only the ``position`` and ``heading`` attributes from
each ``Boid``; it imports nothing from ``simulation.py`` or ``sardine.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import time

import numpy as np
import pyglet
import pyglet.gl as gl

if TYPE_CHECKING:
    from boid import Boid
    from config import SimConfig
    from simulation import Simulation


# Sardine fill colour — cool silvery blue-white against deep-water background.
_SARDINE_COLOR: tuple[int, int, int, int] = (210, 240, 230, 255)
# Sailfish fill colour — vivid orange-red so predators stand out clearly.
_SAILFISH_COLOR: tuple[int, int, int, int] = (220, 80, 40, 255)


class Hud:
    """Live diagnostics text overlay, refreshed every 0.25 s.

    Displays FPS, frame / update / render timing, agent counts, and world
    dimensions.  The text label is rebuilt at most 4 times per second to
    avoid per-frame string-allocation cost.
    """

    _REFRESH: float = 0.25

    def __init__(self, window_height: int) -> None:
        self._label = pyglet.text.Label(
            "",
            font_name=["Menlo", "Consolas", "Courier New"],
            font_size=12,
            x=8,
            y=window_height - 8,
            width=460,
            multiline=True,
            anchor_y="top",
            color=(220, 220, 220, 210),
        )
        self._next_refresh: float = 0.0
        # Manual FPS tracking — pyglet.clock.get_fps() was removed in pyglet 2.x.
        self._fps: float = 0.0
        self._frame_count: int = 0
        self._window_start: float = time.perf_counter()

    def update(
        self,
        simulation: "Simulation",
        update_ms: float,
        render_ms: float,
    ) -> None:
        """Rebuild the label text (rate-limited to _REFRESH seconds)."""
        self._frame_count += 1
        now = time.perf_counter()
        if now < self._next_refresh:
            return
        # Compute FPS from frames drawn since last refresh window.
        elapsed = now - self._window_start
        self._fps = self._frame_count / elapsed if elapsed > 0 else 0.0
        self._frame_count = 0
        self._window_start = now
        self._next_refresh = now + self._REFRESH
        fps = self._fps
        frame_ms = 1000.0 / fps if fps > 0.5 else 0.0
        try:
            n_s = len(simulation.sardines)
            n_f = len(simulation.sailfish)
            W = simulation.config.window_width
            H = simulation.config.window_height
            counts = f"Sardines {n_s}  |  Sailfish {n_f}  |  Total {n_s + n_f}"
            dims = f"World {W} \u00d7 {H} px"
        except Exception:
            counts = "Sardines n/a  |  Sailfish n/a"
            dims = "World n/a"
        self._label.text = (
            f"FPS {fps:5.1f}  |  Frame {frame_ms:4.1f} ms\n"
            f"Update {update_ms:5.1f} ms  |  Render {render_ms:4.1f} ms\n"
            f"{counts}\n"
            f"{dims}\n"
            f"\u2191/\u2193 sardines (+/-50)  \u2190/\u2192 sailfish (+/-1)  F fullscreen  H HUD  TAB config"
        )

    def reposition(self, window_height: int) -> None:
        """Move the label to the top-left of a newly sized window."""
        self._label.y = window_height - 8

    def draw(self) -> None:
        """Render the label for this frame."""
        self._label.draw()


class Renderer:
    """Batch renderer for all simulation agents.

    Parameters
    ----------
    config:
        Shared simulation config; reads ``sardine_size`` and
        ``background_color``.
    window:
        The active pyglet window.  Must be created before this class is
        instantiated so the OpenGL context is current.
    """

    def __init__(self, config: SimConfig, window: pyglet.window.Window) -> None:
        self._config = config
        self._window = window
        self._batch: pyglet.graphics.Batch = pyglet.graphics.Batch()
        self._sardine_shapes: list[pyglet.shapes.Triangle] = []
        self._sailfish_shapes: list[pyglet.shapes.Triangle] = []

        # Set window clear colour using the config's background value.
        # Requires an active GL context, which the Window constructor provides.
        r, g, b, a = config.background_color
        gl.glClearColor(r / 255.0, g / 255.0, b / 255.0, a / 255.0)

        self._last_render_ms: float = 0.0
        self._hud: Hud | None = Hud(window.height) if config.show_hud else None

    # ── One-time setup ────────────────────────────────────────────────

    def reset(self, sardines: list["Boid"], sailfish: list["Boid"] | None = None) -> None:
        """Build one Triangle shape per agent and register with the batch.

        Safe to call more than once: existing shapes are deleted first so
        this can be used for population changes (spawn / despawn).

        Parameters
        ----------
        sardines:
            The ``Simulation.sardines`` list.
        sailfish:
            The ``Simulation.sailfish`` list.  Defaults to ``None`` (empty)
            so Phase-1-only callers require no change.
        """
        for shape in self._sardine_shapes:
            shape.delete()
        self._sardine_shapes.clear()

        for shape in self._sailfish_shapes:
            shape.delete()
        self._sailfish_shapes.clear()

        self._sardine_shapes = self._make_shapes(
            sardines, float(self._config.sardine_size), _SARDINE_COLOR
        )
        self._sailfish_shapes = self._make_shapes(
            sailfish or [], float(self._config.sailfish_size), self._config.sailfish_color
        )

    # ── Per-frame draw ────────────────────────────────────────────────

    def draw(self, simulation: "Simulation", *, update_ms: float = 0.0) -> None:
        """Sync triangle vertices with current agent positions and draw.

        Call this inside the ``on_draw`` event handler after
        ``window.clear()``.  *update_ms* is the simulation step time
        (milliseconds) measured by the caller and forwarded to the HUD.
        """
        t0 = time.perf_counter()
        _sync_shapes(
            self._sardine_shapes, simulation.sardines,
            float(self._config.sardine_size),
        )
        _sync_shapes(
            self._sailfish_shapes, simulation.sailfish,
            float(self._config.sailfish_size),
        )
        self._batch.draw()
        self._last_render_ms = (time.perf_counter() - t0) * 1000.0

        if self._hud is not None:
            self._hud.update(simulation, update_ms, self._last_render_ms)
            self._hud.draw()

    def toggle_hud(self) -> None:
        """Show the HUD if hidden; hide it if shown."""
        if self._hud is None:
            self._hud = Hud(self._window.height)
        else:
            self._hud = None

    def on_resize(self, width: int, height: int) -> None:
        """React to a window resize: reposition the HUD label.

        Call this from the pyglet ``on_resize`` event handler after
        ``simulation.resize()`` has already been called.
        """
        if self._hud is not None:
            self._hud.reposition(height)

    # ── Private helpers ───────────────────────────────────────────────

    def _make_shapes(
        self,
        agents: list["Boid"],
        size: float,
        color: tuple[int, int, int, int],
    ) -> list[pyglet.shapes.Triangle]:
        """Allocate one Triangle per agent in ``self._batch``."""
        shapes = []
        for agent in agents:
            tip, left, right = _boid_verts(agent, size)
            shapes.append(
                pyglet.shapes.Triangle(
                    float(tip[0]),   float(tip[1]),
                    float(left[0]),  float(left[1]),
                    float(right[0]), float(right[1]),
                    color=color,
                    batch=self._batch,
                )
            )
        return shapes


# ── Module-level vertex helpers ──────────────────────────────────────────────

def _sync_shapes(
    shapes: list[pyglet.shapes.Triangle],
    agents: list["Boid"],
    size: float,
) -> None:
    """Write current vertex positions into *shapes* for the next draw call.

    All six vertex coordinates are pushed in a single ``_update_vertices()``
    call (rather than six separate property-setter calls) to minimise
    redundant buffer traffic.

    Implementation note: ``_x`` / ``_y`` is the first vertex **and** the GPU
    translation origin; ``_x2``, ``_y2``, ``_x3``, ``_y3`` are stored as
    absolute world coordinates (the GPU shader adds the translation back).
    """
    size2 = size * 2.0
    for tri, agent in zip(shapes, agents):
        px, py = float(agent.position[0]), float(agent.position[1])
        vx, vy = float(agent.velocity[0]), float(agent.velocity[1])
        mag = (vx * vx + vy * vy) ** 0.5
        if mag > 1e-9:
            hx, hy = vx / mag, vy / mag
        else:
            hx, hy = 1.0, 0.0
        # tip: nose; left/right: rear corners using perp = (-hy, hx)
        tri._x,  tri._y  = px + hx * size2,           py + hy * size2
        tri._x2, tri._y2 = px - hx * size - hy * size, py - hy * size + hx * size
        tri._x3, tri._y3 = px - hx * size + hy * size, py - hy * size - hx * size
        tri._update_translation()
        tri._update_vertices()


def _boid_verts(
    boid: Boid,
    size: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the three triangle vertices for *boid* at the current frame.

    The triangle points in the direction of ``boid.heading``:

    * **tip**   — nose: ``size * 2`` pixels ahead of position.
    * **left**  — rear-left corner: ``size`` pixels behind, ``size`` left.
    * **right** — rear-right corner: symmetric to left.

    Parameters
    ----------
    boid:
        Any object with a ``position`` (float64 ``(2,)`` array) and a
        ``heading`` (unit float64 ``(2,)`` array) attribute.
    size:
        Comes from ``SimConfig.sardine_size`` (pixels).

    Returns
    -------
    tip, left, right:
        Three ``(2,)`` float64 arrays in world coordinates.
    """
    h    = boid.heading                               # unit direction vector
    perp = np.array([-h[1], h[0]], dtype=np.float64)  # 90° left of heading
    tip   = boid.position + h    * (size * 2.0)
    left  = boid.position - h    *  size + perp * size
    right = boid.position - h    *  size - perp * size
    return tip, left, right
