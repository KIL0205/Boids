"""
rendering/renderer.py
=====================
Pyglet 2.x drawing layer for the sardine / sailfish simulation.

Draws each sardine as a heading-oriented filled triangle inside a single
pyglet Batch.  The batch is submitted to the GPU with one draw call per
frame, so this layer has minimal overhead for up to ~500 agents.

Dynamic palette coloring
-----------------------
Sardine color is derived each frame from three per-fish kinematic channels:

* **Heading direction** — maps the fish's travel angle (0-360°) to the
  current palette.  Fish swimming in the same direction share the same base
  hue, so an aligned school forms a tight color cluster.

* **Linear speed** — adds a fractional hue offset (up to 1/4 wheel) so faster
  fish within the same school shift visibly, giving a speed-cohort signal.

* **Turn rate (rotational velocity)** — reduces saturation as the fish turns.
  Straight-swimming fish are vivid; tight turners desaturate toward grey.
  A configurable EMA smoother tames frame-to-frame jitter.

The heading contribution is computed as ``atan2(vy, vx) / (2π) % 1.0`` which
maps direction to a ``[0, 1)`` parameter.  For **rainbow** mode this parameter
is used directly as the HSV hue (hue 0 ≡ hue 1 = red, so the full wheel is
seamlessly cyclic).  For **restricted palettes** (cool / warm / ocean / sunset)
a cosine mapping oscillates smoothly within the palette band::

    hue = (hue_center + band_half * cos(raw_hue * 2π)) % 1.0

This guarantees that ``raw_hue = 0`` and ``raw_hue = 1`` produce identical
colors, eliminating the flicker that a linear ramp would cause whenever the
heading + speed sum wraps through 1.0.

All three contributions are weighted independently via ``SimConfig`` fields
(``vel_color_heading_weight``, ``vel_color_speed_weight``,
``vel_color_turn_weight``), so the balance can be tuned at runtime.

Toggle-off bug fix
------------------
The previous implementation let stale velocity colors persist on triangles
when the mode was disabled because ``_sync_shapes()`` updates vertex
positions but never touches ``tri.color``.  The fix: ``Renderer`` tracks
the previous ``use_velocity_coloring`` value.  On the first frame after
transitioning to OFF, all sardine triangles are explicitly reset to
``_SARDINE_COLOR`` in a single O(N) pass before the normal position sync.

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


# Sardine default fill colour — silvery blue-white on deep-water background.
_SARDINE_COLOR: tuple[int, int, int, int] = (210, 240, 230, 255)

# Algae fixed fill colour — reddish-brown, distinct from silver sardines and orange sailfish.
_ALGAE_COLOR: tuple[int, int, int, int] = (170, 50, 20, 255)

# ── Sailfish per-mode fixed colors ───────────────────────────────────────────
# Keyed by ``config.sailfish_color_mode``.  All RGBA.
_SAILFISH_COLORS: dict[str, tuple[int, int, int, int]] = {
    "default": (220,  80,  40, 255),   # vivid orange-red (high contrast)
    "white":   (240, 245, 248, 255),   # silvery white
    "red":     (215,  30,  30, 255),   # bright red
    "blue":    ( 40,  90, 215, 255),   # electric blue
    "black":   ( 25,  30,  40, 255),   # near-black (still visible on dark bg)
}
# Fallback when an unknown mode appears at runtime.
_SAILFISH_COLOR_DEFAULT: tuple[int, int, int, int] = _SAILFISH_COLORS["default"]

# ── Sardine palette system ────────────────────────────────────────────────────
# Each entry: (hue_min, hue_max, brightness_value)
# The motion-derived raw_hue in [0,1] is remapped into [hue_min, hue_max].
# hue_max - hue_min >= 1.0 means full wheel ("rainbow" mode).
# hue_max > 1.0 wraps via `% 1.0`, allowing palettes that span the 0 degree boundary.
# "mono" is handled as a special case (brightness encodes speed; sat near 0).
_PALETTE_BANDS: dict[str, tuple[float, float, float]] = {
    "rainbow": (0.00,  1.00, 0.95),   # full HSV wheel
    "cool":    (0.50,  0.80, 0.90),   # teal -> blue -> purple
    "warm":    (0.02,  0.17, 0.95),   # red -> orange -> yellow
    "ocean":   (0.47,  0.62, 0.82),   # cyan -> teal -> blue (deeper/muted)
    "sunset":  (0.74,  1.08, 0.90),   # purple -> magenta -> pink-orange (wraps)
}

# HSV constants shared by all dynamic palette modes.
_VEL_SAT_MIN: float = 0.15           # saturation floor (no fully-gray fish)
_VEL_SPEED_HUE_RANGE: float = 0.25   # max hue shift from speed at w_speed=1
_VEL_TURN_REF: float = math.pi / 6.0 # reference turn rate for saturation scaling
_TWO_PI: float = 2.0 * math.pi


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
            width=520,
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
        """Rebuild the label text (rate-limited to _REFRESH seconds).

        Throttling
        ----------
        Population counts, cumulative stats, and average hunger values are
        all recomputed at most once per _REFRESH interval (default 0.25 s =
        4 Hz).  The avg-hunger sums iterate over the live agent lists —
        O(N_sardines + N_sailfish) — but only run 4 times per second, so
        the cost is negligible (≈1 200 attribute reads/s at N=300).
        """
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
            cfg      = simulation.config
            sardines = simulation.sardines
            sailfish = simulation.sailfish
            n_s = len(sardines)
            n_f = len(sailfish)
            n_a = len(simulation.algae)

            # ── Average hunger (O(N) but only recomputed at 4 Hz) ────────
            # Shows avg time since last meal vs. current population, giving
            # an at-a-glance read on how stressed each trophic level is.
            if sardines:
                avg_sar = sum(s.hunger_timer for s in sardines) / n_s
                sar_hunger = f"{avg_sar:.1f}s"
            else:
                sar_hunger = "---"
            if sailfish:
                avg_sf = sum(sf.hunger_timer for sf in sailfish) / n_f
                sf_hunger = f"{avg_sf:.1f}s"
            else:
                sf_hunger = "---"

            color_mode = cfg.sardine_color_mode
            color_str  = f"  [{color_mode.upper()}]" if color_mode != "white" else ""
            sf_flags = ""
            if not cfg.sailfish_starvation_enabled:   sf_flags += "  [STARV OFF]"
            if not cfg.sailfish_reproduction_enabled: sf_flags += "  [REPRO OFF]"
            W = cfg.window_width
            H = cfg.window_height
        except Exception:
            self._label.text = "HUD data unavailable"
            return

        self._label.text = (
            f"FPS {fps:5.1f}  |  Frame {frame_ms:4.1f} ms  |  "
            f"Update {update_ms:4.1f} ms  |  Render {render_ms:4.1f} ms\n"
            f"Sardines {n_s}  |  Sailfish {n_f}  |  Algae {n_a}"
            f"  |  Total {n_s + n_f + n_a}\n"
            f"Algae: eaten\u2191{simulation.stats_algae_eaten}"
            f"  spawned\u2191{simulation.stats_algae_spawned}\n"
            f"Sardines: born\u2191{simulation.stats_sardine_births}"
            f"  starved\u2191{simulation.stats_sardine_starvations}"
            f"  eaten\u2191{simulation.stats_sardines_eaten}"
            f"  avg hunger {sar_hunger}\n"
            f"Sailfish: born\u2191{simulation.stats_sailfish_births}"
            f"  starved\u2191{simulation.stats_sailfish_starvations}"
            f"  avg hunger {sf_hunger}{sf_flags}{color_str}\n"
            f"World {W} \u00d7 {H} px\n"
            f"\u2191/\u2193 sardines (+/-50)  \u2190/\u2192 sailfish (+/-1)  "
            f"F fullscreen  H HUD  V color-mode  TAB config"
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
        self._algae_shapes:   list[pyglet.shapes.Circle]   = []
        self._sardine_shapes: list[pyglet.shapes.Triangle] = []
        self._sailfish_shapes: list[pyglet.shapes.Triangle] = []

        # Maps id(Algae boid) → Circle so eaten algae shapes can be removed
        # in O(K) without rebuilding the whole algae shape list each frame.
        self._algae_shape_map: dict[int, pyglet.shapes.Circle] = {}

        # Maps id(Sardine boid) → Triangle and _VelColorState so sardines eaten
        # by sailfish or starved can be removed in O(K) without rebuilding.
        self._sardine_shape_map: dict[int, pyglet.shapes.Triangle] = {}
        self._sardine_vel_map:   dict[int, _VelColorState]         = {}

        # Maps id(Sailfish boid) → Triangle so starved sailfish shapes can be
        # removed in O(K) without rebuilding the whole sailfish shape list.
        self._sailfish_shape_map: dict[int, pyglet.shapes.Triangle] = {}

        # Per-fish velocity-color state (rebuilt in reset()).
        self._vel_states: list[_VelColorState] = []

        # Track previous color modes to detect transitions.
        # On sardine-mode change: reset EMA state; restore white if going to "white".
        # On sailfish-mode change: repaint all sailfish triangles in one O(N) pass.
        self._prev_sardine_mode: str = config.sardine_color_mode
        self._prev_sailfish_mode: str = config.sailfish_color_mode

        r, g, b, a = config.background_color
        gl.glClearColor(r / 255.0, g / 255.0, b / 255.0, a / 255.0)

        self._last_render_ms: float = 0.0
        self._hud: Hud | None = Hud(window.height) if config.show_hud else None

        # Optional mouse-predator radius indicator (drawn outside the batch).
        # Created lazily on the first active frame; deleted on reset().
        self._mouse_indicator: pyglet.shapes.Circle | None = None

    # ── One-time setup ────────────────────────────────────────────────

    def reset(self, sardines: list["Boid"], sailfish: list["Boid"] | None = None, algae: list["Boid"] | None = None) -> None:
        """Build one Triangle shape per agent and register with the batch.

        Safe to call more than once: existing shapes are deleted first.

        Algae shapes are created first so they render behind sardines and
        sailfish in the batch draw order.
        """
        for shape in self._algae_shapes:
            shape.delete()
        self._algae_shapes.clear()

        for shape in self._sardine_shapes:
            shape.delete()
        self._sardine_shapes.clear()

        for shape in self._sailfish_shapes:
            shape.delete()
        self._sailfish_shapes.clear()

        if self._mouse_indicator is not None:
            self._mouse_indicator.delete()
            self._mouse_indicator = None

        # Algae first — drawn behind sardines and sailfish.
        self._algae_shapes = self._make_algae_shapes(
            algae or [], float(self._config.algae_size), _ALGAE_COLOR
        )
        # Build identity→shape map for O(K) per-frame removal of eaten algae.
        self._algae_shape_map = {
            id(a): s for a, s in zip(algae or [], self._algae_shapes)
        }
        self._sardine_shapes = self._make_shapes(
            sardines, float(self._config.sardine_size), _SARDINE_COLOR
        )
        sailfish_color = _SAILFISH_COLORS.get(
            self._config.sailfish_color_mode, _SAILFISH_COLOR_DEFAULT
        )
        self._sailfish_shapes = self._make_shapes(
            sailfish or [], float(self._config.sailfish_size), sailfish_color
        )

        # Allocate (or resize) velocity-color state for each sardine.
        self._vel_states = [_VelColorState() for _ in sardines]
        self._sardine_shape_map = {
            id(s): shape for s, shape in zip(sardines, self._sardine_shapes)
        }
        self._sardine_vel_map = {
            id(s): vs for s, vs in zip(sardines, self._vel_states)
        }
        # Build sailfish identity→shape map for O(K) starvation removal.
        self._sailfish_shape_map = {
            id(sf): shape for sf, shape in zip(sailfish or [], self._sailfish_shapes)
        }

    # ── Per-frame draw ────────────────────────────────────────────────

    def draw(self, simulation: "Simulation", *, update_ms: float = 0.0) -> None:
        """Sync triangle vertices with current agent positions and draw.

        Call inside the ``on_draw`` event handler after ``window.clear()``.
        """
        t0 = time.perf_counter()

        cfg           = simulation.config
        sardine_mode  = cfg.sardine_color_mode
        sailfish_mode = cfg.sailfish_color_mode
        # Cache size floats once — avoids repeated float() coercions on the
        # frozen-dataclass fields during every call to _sync_shapes*.
        sardine_size_f  = float(cfg.sardine_size)
        sailfish_size_f = float(cfg.sailfish_size)

        # ── Handle sardine births (reproduction) ─────────────────────────
        # _sardine_born contains new Sardine objects appended to
        # simulation.sardines this frame.  Allocate a Triangle + VelColorState
        # for each and append — keeps _sardine_shapes in 1-to-1 order with
        # simulation.sardines without a full reset().
        # Guard: skip if a shape already exists for this offspring.  This
        # prevents duplicate triangles when update() runs twice before draw()
        # (reentrant idle() on macOS during resize/fullscreen transitions).
        born = simulation._sardine_born
        if born:
            size_f = float(cfg.sardine_size)
            for offspring in born:
                if id(offspring) not in self._sardine_shape_map:
                    tri = self._make_shapes([offspring], size_f, _SARDINE_COLOR)[0]
                    vs  = _VelColorState()
                    self._sardine_shapes.append(tri)
                    self._vel_states.append(vs)
                    self._sardine_shape_map[id(offspring)] = tri
                    self._sardine_vel_map[id(offspring)]   = vs

        # ── Handle sardine deaths (eaten by sailfish OR starved) ───────────────
        # Both _sardines_eaten_ids (predation) and _sardines_starved_ids
        # (starvation) are processed here so the renderer always stays in
        # 1-to-1 alignment with simulation.sardines.
        eaten_sardine_ids = simulation._sardines_eaten_ids | simulation._sardines_starved_ids
        if eaten_sardine_ids:
            for sid in eaten_sardine_ids:
                shape = self._sardine_shape_map.pop(sid, None)
                if shape is not None:
                    shape.delete()
                self._sardine_vel_map.pop(sid, None)
            # Lazy rescue: if a sardine exists in simulation.sardines but is
            # missing from the map (born-notification was cleared by a
            # reentrant update() before draw() processed it), create its
            # triangle now so the rebuild never raises KeyError.
            s_size_f = float(cfg.sardine_size)
            self._sardine_shapes = []
            self._vel_states = []
            for s in simulation.sardines:
                tri = self._sardine_shape_map.get(id(s))
                if tri is None:
                    tri = self._make_shapes([s], s_size_f, _SARDINE_COLOR)[0]
                    vs  = _VelColorState()
                    self._sardine_shape_map[id(s)] = tri
                    self._sardine_vel_map[id(s)]   = vs
                self._sardine_shapes.append(tri)
                self._vel_states.append(self._sardine_vel_map[id(s)])

        # ── Detect sardine color-mode transitions ─────────────────────────
        if sardine_mode != self._prev_sardine_mode:
            # Reset per-fish EMA state so the first frame after a switch has
            # no stale turn-rate estimate regardless of which mode we enter.
            for vs in self._vel_states:
                vs.frames = 0
            if sardine_mode == "white":
                # One-shot restore: _sync_shapes only writes vertex positions,
                # never colors, so without this pass triangles would keep
                # whatever palette colors they had from the previous mode.
                for tri in self._sardine_shapes:
                    tri.color = _SARDINE_COLOR
            self._prev_sardine_mode = sardine_mode

        if sardine_mode == "white":
            _sync_shapes(
                self._sardine_shapes, simulation.sardines,
                sardine_size_f,
            )
        else:
            _sync_shapes_vel_color(
                self._sardine_shapes,
                simulation.sardines,
                sardine_size_f,
                float(cfg.sardine_max_speed),
                self._vel_states,
                cfg.vel_color_heading_weight,
                cfg.vel_color_speed_weight,
                cfg.vel_color_turn_weight,
                cfg.vel_color_smoothing,
                sardine_mode,
            )

        # ── Detect sailfish color-mode transitions ────────────────────────
        if sailfish_mode != self._prev_sailfish_mode:
            color = _SAILFISH_COLORS.get(sailfish_mode, _SAILFISH_COLOR_DEFAULT)
            for tri in self._sailfish_shapes:
                tri.color = color
            self._prev_sailfish_mode = sailfish_mode

        # ── Handle sailfish births (reproduction) ─────────────────────────
        # _sailfish_born contains new Sailfish objects appended to
        # simulation.sailfish this frame.  Allocate a Triangle for each and
        # append — keeps _sailfish_shapes in 1-to-1 order with
        # simulation.sailfish without a full reset().
        # Guard: skip if a shape already exists for this offspring (protects
        # against double-processing on reentrant idle() during resize).
        sf_born = simulation._sailfish_born
        if sf_born:
            sf_size_f = float(cfg.sailfish_size)
            sf_color  = _SAILFISH_COLORS.get(cfg.sailfish_color_mode,
                                              _SAILFISH_COLOR_DEFAULT)
            for offspring in sf_born:
                if id(offspring) not in self._sailfish_shape_map:
                    tri = self._make_shapes([offspring], sf_size_f, sf_color)[0]
                    self._sailfish_shapes.append(tri)
                    self._sailfish_shape_map[id(offspring)] = tri

        # ── Handle sailfish deaths (starvation) ────────────────────────────
        # _sailfish_died_ids holds id() values of sailfish that starved this
        # frame.  Delete their Triangle shapes and rebuild _sailfish_shapes so
        # _sync_shapes stays in 1-to-1 alignment with simulation.sailfish.
        if simulation._sailfish_died_ids:
            for sfid in simulation._sailfish_died_ids:
                shape = self._sailfish_shape_map.pop(sfid, None)
                if shape is not None:
                    shape.delete()
            # Lazy rescue: create triangles for any sailfish missing from the
            # map (defensive — sailfish have no birth side-channel but keeps
            # the pattern consistent with sardines and algae).
            sf_size_f = float(cfg.sailfish_size)
            sf_color  = _SAILFISH_COLORS.get(cfg.sailfish_color_mode,
                                              _SAILFISH_COLOR_DEFAULT)
            self._sailfish_shapes = []
            for sf in simulation.sailfish:
                tri = self._sailfish_shape_map.get(id(sf))
                if tri is None:
                    tri = self._make_shapes([sf], sf_size_f, sf_color)[0]
                    self._sailfish_shape_map[id(sf)] = tri
                self._sailfish_shapes.append(tri)
        _sync_shapes(
            self._sailfish_shapes, simulation.sailfish,
            sailfish_size_f,
        )
        # Algae — fixed colour, position-only sync.
        # Handle newly respawned algae first (mirrors the _sardine_born birth
        # pattern): allocate a Circle for each new Algae object so the
        # shapes list stays in 1-to-1 alignment with simulation.algae BEFORE
        # we process eaten ids.
        # Guard: skip if a circle already exists (protects against double-
        # processing when update() runs twice before draw() on macOS during
        # fullscreen/resize transitions — see renderer crash-fix notes in
        # PROJECT_LOG.md).
        algae_size_f = float(cfg.algae_size)
        algae_born = simulation._algae_born
        if algae_born:
            for alga in algae_born:
                if id(alga) not in self._algae_shape_map:
                    circle = self._make_algae_shapes([alga], algae_size_f,
                                                     _ALGAE_COLOR)[0]
                    self._algae_shapes.append(circle)
                    self._algae_shape_map[id(alga)] = circle

        # First remove Circle shapes for any algae eaten this frame so the
        # shapes list stays in 1-to-1 alignment with simulation.algae.
        eaten_ids = simulation._algae_eaten_ids
        if eaten_ids:
            for boid_id in eaten_ids:
                shape = self._algae_shape_map.pop(boid_id, None)
                if shape is not None:
                    shape.delete()
            # Rebuild with lazy rescue: if a born-notification was cleared by
            # a reentrant update() before draw() processed it, the alga is
            # already in simulation.algae but has no circle yet.  Create one
            # now so the list comprehension never raises KeyError.
            new_shapes = []
            for a in simulation.algae:
                shape = self._algae_shape_map.get(id(a))
                if shape is None:
                    shape = self._make_algae_shapes([a], algae_size_f,
                                                    _ALGAE_COLOR)[0]
                    self._algae_shape_map[id(a)] = shape
                new_shapes.append(shape)
            self._algae_shapes = new_shapes
        _sync_circle_positions(
            self._algae_shapes, simulation.algae,
        )
        self._batch.draw()
        self._last_render_ms = (time.perf_counter() - t0) * 1000.0

        # ── Mouse predator visual indicator ───────────────────────────────
        # Draw a faint red filled circle at the cursor when the mouse
        # predator is active.  Drawn outside the batch so it appears on top
        # of all fish, and so it never interferes with the agent shape maps.
        ms = simulation._mouse_stimulus
        if cfg.mouse_predator_enabled and ms.active:
            if self._mouse_indicator is None:
                self._mouse_indicator = pyglet.shapes.Circle(
                    ms.x, ms.y, ms.radius,
                    color=(220, 40, 40, 40),
                )
            else:
                self._mouse_indicator.x      = ms.x
                self._mouse_indicator.y      = ms.y
                self._mouse_indicator.radius = ms.radius
            self._mouse_indicator.draw()

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

    def _make_algae_shapes(
        self,
        agents: list["Boid"],
        radius: float,
        color: tuple[int, int, int, int],
    ) -> list[pyglet.shapes.Circle]:
        """Allocate one Circle per algae agent in ``self._batch``."""
        shapes = []
        for agent in agents:
            shapes.append(
                pyglet.shapes.Circle(
                    float(agent.position[0]),
                    float(agent.position[1]),
                    radius,
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
        mag = math.sqrt(vx * vx + vy * vy)
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
    w_heading: float,
    w_speed: float,
    w_turn: float,
    smoothing: float,
    mode: str = "rainbow",
) -> None:
    """Like ``_sync_shapes`` but also updates each triangle's colour.

    Palette modes
    -------------
    *mode* is one of ``"rainbow"``, ``"cool"``, ``"warm"``, ``"ocean"``,
    ``"sunset"``, or ``"mono"``.

    **Rainbow** — full HSV wheel; motion-derived raw_hue used directly.

    **Restricted palettes** (cool / warm / ocean / sunset) — raw_hue drives
    a cyclic cosine mapping within the allowed hue band::

        hue = (hue_center + band_half * cos(raw_hue * 2π)) % 1.0

    where ``hue_center = (hue_min + hue_max) / 2`` and
    ``band_half = (hue_max - hue_min) / 2``.  Because cos(0) = cos(2π),
    ``raw_hue = 0`` and ``raw_hue = 1`` produce exactly the same hue —
    eliminating the jump that the old linear ramp produced whenever the
    cumulative heading + speed sum wrapped through 1.0 (previously visible
    as a purple→teal flash for ``cool``, etc.).

    School coherence is preserved: each palette oscillates smoothly between
    ``hue_min`` and ``hue_max`` as direction rotates.  Two schools pointing
    in exactly opposite directions land at the same palette position (cosine
    symmetry), but in practice opposing schools diverge spatially and remain
    visually distinct near the palette midpoint.

    **Mono** — hue is irrelevant; saturation is near zero; value
    (brightness) is derived from speed so fast fish are bright.

    Color channels (shared by all non-mono modes)
    -----------------------------------------------
    * Hue   = raw_hue (rainbow) or
              ``(hue_center + band_half * cos(raw_hue * 2π)) % 1.0``
      where ``raw_hue = (heading_01 * w_heading
                         + speed_norm * _VEL_SPEED_HUE_RANGE * w_speed) % 1.0``
    * Sat   = ``max(_VEL_SAT_MIN, 1.0 - clamp(smooth_turn / _VEL_TURN_REF, 0,1)
                   * w_turn * (1 - _VEL_SAT_MIN))``
    * Value = brightness from ``_PALETTE_BANDS`` (0.82–0.95 depending on palette)

    Emergent school coherence: fish sharing heading + speed share the same
    raw_hue and therefore map to the same part of whatever palette is active.
    """
    band = _PALETTE_BANDS.get(mode, _PALETTE_BANDS["rainbow"])
    hue_min, hue_max, vel_val = band
    band_width    = hue_max - hue_min
    full_wheel    = band_width >= 1.0   # skip remap for rainbow
    is_mono       = (mode == "mono")
    size2         = size * 2.0
    inv_mspd      = 1.0 / max(max_speed, 1.0)
    sat_range     = 1.0 - _VEL_SAT_MIN   # = 0.85
    # Pre-computed cyclic palette geometry for restricted palettes.
    # Cached outside the fish loop; negligible when full_wheel or is_mono.
    hue_center    = (hue_min + hue_max) * 0.5
    band_half     = band_width * 0.5

    for tri, agent, vs in zip(shapes, agents, vel_states):
        px, py = float(agent.position[0]), float(agent.position[1])
        vx, vy = float(agent.velocity[0]), float(agent.velocity[1])
        mag = math.sqrt(vx * vx + vy * vy)

        if mag > 1e-9:
            hx, hy = vx / mag, vy / mag
        else:
            hx, hy = 1.0, 0.0

        # ── Vertex positions (same for every mode) ─────────────────────
        tri._x,  tri._y  = px + hx * size2,             py + hy * size2
        tri._x2, tri._y2 = px - hx * size - hy * size,  py - hy * size + hx * size
        tri._x3, tri._y3 = px - hx * size + hy * size,  py - hy * size - hx * size
        tri._update_translation()
        tri._update_vertices()

        # ── Per-fish turn-rate EMA ────────────────────────────────────
        angle = math.atan2(hy, hx)
        if vs.frames >= 1:
            da = angle - vs.prev_angle
            if da >  math.pi: da -= _TWO_PI
            if da < -math.pi: da += _TWO_PI
            vs.smooth_turn = smoothing * vs.smooth_turn + (1.0 - smoothing) * abs(da)
        vs.prev_angle = angle
        vs.frames += 1

        # ── Color computation ─────────────────────────────────────────
        speed_norm = min(mag * inv_mspd, 1.0)
        turn_norm  = min(vs.smooth_turn / _VEL_TURN_REF, 1.0)

        if is_mono:
            # Grayscale: brightness encodes speed; near-zero saturation.
            value = max(0.30, min(0.95, 0.35 + speed_norm * 0.55 * w_speed))
            sat   = 0.0
            hue   = 0.58   # irrelevant but defined; slight blue-grey at low sat
        else:
            heading_01 = angle / _TWO_PI % 1.0
            raw_hue = (heading_01 * w_heading
                       + speed_norm * _VEL_SPEED_HUE_RANGE * w_speed) % 1.0
            if full_wheel:
                hue = raw_hue
            else:
                # Cyclic cosine mapping — cos(0) = cos(2π) so raw_hue=0
                # and raw_hue=1 produce identical colors.  No jump at any
                # heading or speed wrap boundary.
                hue = (hue_center + band_half * math.cos(raw_hue * _TWO_PI)) % 1.0
            sat   = max(_VEL_SAT_MIN, 1.0 - turn_norm * w_turn * sat_range)
            value = vel_val

        r, g, b = _hsv_to_rgb(hue, sat, value)
        tri.color = (r, g, b, 255)


def _sync_circle_positions(
    shapes: list[pyglet.shapes.Circle],
    agents: list["Boid"],
) -> None:
    """Write current positions into *shapes* (circles) for the next draw call."""
    for circle, agent in zip(shapes, agents):
        circle.x = float(agent.position[0])
        circle.y = float(agent.position[1])


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
