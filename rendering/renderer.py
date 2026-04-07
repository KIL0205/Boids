"""
rendering/renderer.py
=====================
Pyglet 2.x drawing layer for the sardine / sailfish simulation.

Draws each sardine as a heading-oriented filled triangle inside a single
pyglet Batch.  The batch is submitted to the GPU with one draw call per
frame, so this layer has minimal overhead for up to ~500 agents.

Velocity-based coloring
-----------------------
When ``config.use_velocity_coloring`` is ``True``, sardine color is derived
each frame from two per-fish quantities:

* **Forward speed** (normalised to [0, 1] relative to ``sardine_max_speed``)
  maps to HSV **hue** in the blue-cyan-green range (0.5–0.75 in [0,1]).
  Slow fish appear deeper blue; fast fish shift toward cyan/teal.

* **Turn rate** (angular velocity, estimated from the heading-change over
  consecutive frames) maps to HSV **saturation**.  Straight-swimming fish
  get saturated, vivid hues; fish in tight turns desaturate toward grey-white
  to give a "stress" signal.  A 0.9-weight exponential smoother on the
  per-fish heading tames frame-to-frame noise without adding lag.

Result: fish schooling together share speed and heading, so they converge
to similar colours.  Fleeing / scattered fish diverge visibly.

Interface
---------
* ``Renderer(config, window)``  — create once **after** the pyglet Window.
* ``renderer.reset(sardines)``  — call once after ``Simulation`` is created,
  and again if the population ever changes.
* ``renderer.draw(simulation)`` — call every frame inside ``on_draw``.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

import numpy as np
import pyglet
import pyglet.gl as gl

if TYPE_CHECKING:
    from sim.boid import Boid
    from config.config import SimConfig
    from sim.simulation import Simulation


# Sardine fill colour — cool silvery blue-white against deep-water background.
_SARDINE_COLOR: tuple[int, int, int, int] = (210, 240, 230, 255)
# Sailfish fill colour — vivid orange-red so predators stand out clearly.
_SAILFISH_COLOR: tuple[int, int, int, int] = (220, 80, 40, 255)

# HSV hue range for velocity-coloring mode (blue → cyan → teal).
_VEL_HUE_MIN: float = 0.50   # blue   (slow fish)
_VEL_HUE_MAX: float = 0.80   # teal   (fast fish)
# Saturation range — high sat for straight swimming, lower for tight turns.
_VEL_SAT_MIN: float = 0.30   # desaturated (high turn rate)
_VEL_SAT_MAX: float = 0.95   # vivid       (straight swim)
# Value (brightness) — kept constant so the school is always visible.
_VEL_VAL:     float = 0.95

# Exponential smoothing coefficient for turn-rate estimates (0 = no smooth).
# 0.85 gives ~6-frame time constant at 60 fps — damps noise while responding
# to genuine turns within ~0.1 s.
_TURN_SMOOTH: float = 0.85

# How many frames since heading capture before initialising (avoids t=0 jump).
_HEADING_INIT_FRAMES: int = 1


def _hsv_to_rgb(h: float, s: float, v: float) -> tuple[int, int, int]:
    """Convert HSV (each in [0, 1]) to integer RGB tuple."""
    if s == 0.0:
        c = int(v * 255)
        return c, c, c
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i %= 6
    if   i == 0: r, g, b = v, t, p
    elif i == 1: r, g, b = q, v, p
    elif i == 2: r, g, b = p, v, t
    elif i == 3: r, g, b = p, q, v
    elif i == 4: r, g, b = t, p, v
    else:        r, g, b = v, p, q
    return int(r * 255), int(g * 255), int(b * 255)


class _VelColorState:
    """Per-fish state needed to estimate turn rate between frames.

    Stored externally (in Renderer) so Boid subclasses stay rendering-free.
    """

    __slots__ = ("prev_angle", "smooth_turn", "frames")

    def __init__(self) -> None:
        self.prev_angle:  float = 0.0
        self.smooth_turn: float = 0.0   # EMA of |angular velocity| in rad/s
        self.frames:      int   = 0


class Hud:
    """Live diagnostics text overlay, refreshed every 0.25 s."""

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
            vel_str = " [VEL COLOR]" if simulation.config.use_velocity_coloring else ""
            counts = f"Sardines {n_s}  |  Sailfish {n_f}  |  Total {n_s + n_f}{vel_str}"
            dims = f"World {W} \u00d7 {H} px"
        except Exception:
            counts = "Sardines n/a  |  Sailfish n/a"
            dims = "World n/a"
        self._label.text = (
            f"FPS {fps:5.1f}  |  Frame {frame_ms:4.1f} ms\n"
            f"Update {update_ms:5.1f} ms  |  Render {render_ms:4.1f} ms\n"
            f"{counts}\n"
            f"{dims}\n"
            f"\u2191/\u2193 sardines (+/-50)  \u2190/\u2192 sailfish (+/-1)  "
            f"F fullscreen  H HUD  V vel-color  TAB config"
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
        Shared simulation config; reads ``sardine_size``, ``background_color``,
        and ``use_velocity_coloring``.
    window:
        The active pyglet window.  Must be created before this class is
        instantiated so the OpenGL context is current.
    """

    def __init__(self, config: "SimConfig", window: pyglet.window.Window) -> None:
        self._config = config
        self._window = window
        self._batch: pyglet.graphics.Batch = pyglet.graphics.Batch()
        self._sardine_shapes: list[pyglet.shapes.Triangle] = []
        self._sailfish_shapes: list[pyglet.shapes.Triangle] = []

        # Per-fish velocity-color state (rebuilt in reset()).
        self._vel_states: list[_VelColorState] = []

        r, g, b, a = config.background_color
        gl.glClearColor(r / 255.0, g / 255.0, b / 255.0, a / 255.0)

        self._last_render_ms: float = 0.0
        self._hud: Hud | None = Hud(window.height) if config.show_hud else None

    # ── One-time setup ────────────────────────────────────────────────

    def reset(self, sardines: list["Boid"], sailfish: list["Boid"] | None = None) -> None:
        """Build one Triangle shape per agent and register with the batch.

        Safe to call more than once: existing shapes are deleted first.
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

        # Allocate (or resize) velocity-color state for each sardine.
        self._vel_states = [_VelColorState() for _ in sardines]

    # ── Per-frame draw ────────────────────────────────────────────────

    def draw(self, simulation: "Simulation", *, update_ms: float = 0.0) -> None:
        """Sync triangle vertices with current agent positions and draw.

        Call inside the ``on_draw`` event handler after ``window.clear()``.
        """
        t0 = time.perf_counter()

        cfg = simulation.config
        if cfg.use_velocity_coloring:
            _sync_shapes_vel_color(
                self._sardine_shapes,
                simulation.sardines,
                float(cfg.sardine_size),
                float(cfg.sardine_max_speed),
                self._vel_states,
            )
        else:
            _sync_shapes(
                self._sardine_shapes, simulation.sardines,
                float(cfg.sardine_size),
            )
            # Reset vel states so they initialise cleanly if mode is toggled on.
            for vs in self._vel_states:
                vs.frames = 0

        _sync_shapes(
            self._sailfish_shapes, simulation.sailfish,
            float(cfg.sailfish_size),
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
        """React to a window resize: reposition the HUD label."""
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


# ── Module-level vertex / color helpers ──────────────────────────────────────

def _sync_shapes(
    shapes: list[pyglet.shapes.Triangle],
    agents: list["Boid"],
    size: float,
) -> None:
    """Write current vertex positions into *shapes* for the next draw call."""
    size2 = size * 2.0
    for tri, agent in zip(shapes, agents):
        px, py = float(agent.position[0]), float(agent.position[1])
        vx, vy = float(agent.velocity[0]), float(agent.velocity[1])
        mag = (vx * vx + vy * vy) ** 0.5
        if mag > 1e-9:
            hx, hy = vx / mag, vy / mag
        else:
            hx, hy = 1.0, 0.0
        tri._x,  tri._y  = px + hx * size2,           py + hy * size2
        tri._x2, tri._y2 = px - hx * size - hy * size, py - hy * size + hx * size
        tri._x3, tri._y3 = px - hx * size + hy * size, py - hy * size - hx * size
        tri._update_translation()
        tri._update_vertices()


def _sync_shapes_vel_color(
    shapes: list[pyglet.shapes.Triangle],
    agents: list["Boid"],
    size: float,
    max_speed: float,
    vel_states: list[_VelColorState],
) -> None:
    """Like ``_sync_shapes`` but also updates each triangle's colour.

    Color mapping
    -------------
    * Hue:        linear speed / max_speed  → [HUE_MIN, HUE_MAX] (blue→teal)
    * Saturation: turn rate (EMA-smoothed)  → [SAT_MAX, SAT_MIN]
                  (vivid=straight, grey=turning)
    * Value:      constant _VEL_VAL (keeps fish visible at all times)

    Turn rate is estimated as |Δangle| / dt.  Because dt is not forwarded here
    we use a simple frame-delta approximation: the EMA coefficient already
    encodes the smoothing time constant.  For 60 fps the distinction between
    using dt or frame-count is negligible.
    """
    size2    = size * 2.0
    inv_mspd = 1.0 / max(max_speed, 1.0)

    for tri, agent, vs in zip(shapes, agents, vel_states):
        px, py = float(agent.position[0]), float(agent.position[1])
        vx, vy = float(agent.velocity[0]), float(agent.velocity[1])
        mag = math.sqrt(vx * vx + vy * vy)

        if mag > 1e-9:
            hx, hy = vx / mag, vy / mag
        else:
            hx, hy = 1.0, 0.0

        # ── Velocity positions ─────────────────────────────────────────
        tri._x,  tri._y  = px + hx * size2,           py + hy * size2
        tri._x2, tri._y2 = px - hx * size - hy * size, py - hy * size + hx * size
        tri._x3, tri._y3 = px - hx * size + hy * size, py - hy * size - hx * size
        tri._update_translation()
        tri._update_vertices()

        # ── Colour computation ─────────────────────────────────────────
        # --- Hue from forward speed ---
        speed_norm = min(mag * inv_mspd, 1.0)
        hue = _VEL_HUE_MIN + speed_norm * (_VEL_HUE_MAX - _VEL_HUE_MIN)

        # --- Saturation from turn rate ---
        cur_angle = math.atan2(hy, hx)
        if vs.frames >= _HEADING_INIT_FRAMES:
            # Δangle with wrap-around correction to [-π, π]
            da = cur_angle - vs.prev_angle
            if da >  math.pi: da -= 2.0 * math.pi
            if da < -math.pi: da += 2.0 * math.pi
            # |da| in radians/frame; smooth with EMA.
            vs.smooth_turn = (
                _TURN_SMOOTH * vs.smooth_turn + (1.0 - _TURN_SMOOTH) * abs(da)
            )
        vs.prev_angle = cur_angle
        vs.frames += 1

        # Map turn rate: 0 → SAT_MAX (vivid/straight), π/6 rad/frame → SAT_MIN.
        # π/6 ≈ 30°/frame is an aggressive full-turn rate; most flocking turns
        # are much smaller so saturation typically stays near SAT_MAX.
        turn_norm = min(vs.smooth_turn / (math.pi / 6.0), 1.0)
        sat = _VEL_SAT_MAX - turn_norm * (_VEL_SAT_MAX - _VEL_SAT_MIN)

        r, g, b = _hsv_to_rgb(hue, sat, _VEL_VAL)
        tri.color = (r, g, b, 255)


def _boid_verts(
    boid: "Boid",
    size: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the three triangle vertices for *boid* at the current frame."""
    h    = boid.heading
    perp = np.array([-h[1], h[0]], dtype=np.float64)
    tip   = boid.position + h    * (size * 2.0)
    left  = boid.position - h    *  size + perp * size
    right = boid.position - h    *  size - perp * size
    return tip, left, right
