"""
rendering/debug_overlay.py
==========================
Optional in-world debug overlay for baitball / herding visualisation.

All drawing is done **outside** the main Pyglet batch, using immediate
``.draw()`` calls on short-lived shapes and labels, exactly like the mouse
predator indicator in renderer.py.  This means the overlay always appears
on top of every fish sprite and adds zero cost to the batch submission.

The entire overlay is gated by ``cfg.debug_baitballs_enabled``.  When that
flag is False the ``draw()`` method returns after a single branch check.

Exposed visuals (each individually toggled in SimConfig):
  1. Baitball shading    — translucent filled circle + center dot per baitball.
  2. Baitball labels     — sardine count / density / radius text near center.
  3. SF-BB links         — line from each sailfish to its baitball (color
                           varies by state: orbit=cyan-dim, strike-ready=cyan,
                           active-strike=yellow).
  4. Orbit slot markers  — cross + stub line showing each sailfish's assigned
                           orbit target position (only when slot is valid).
  5. Strike path lines   — planned exit point for center-pass strikes; target
                           highlight for edge-target (TARGET_LOCK) strikes.
  6. State labels        — small text label above each sailfish body showing
                           its current FSM state.  Does NOT require the master
                           toggle; gated by its own flag.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pyglet
import pyglet.shapes
import pyglet.text

if TYPE_CHECKING:
    from config.config import SimConfig
    from sim.simulation import Simulation

# ── Position helper ────────────────────────────────────────────────────────────

def _pos(boid: object) -> tuple[float, float]:
    """Return world (x, y) from any Boid whose position is a numpy array.

    ``boid.position`` is a ``Vec2`` (numpy float64 array) — ``position[0]`` is
    x and ``position[1]`` is y.  This helper keeps the overlay code readable
    and avoids attribute errors if the API ever changes.
    """
    return float(boid.position[0]), float(boid.position[1])  # type: ignore[union-attr]

# ── Color constants ────────────────────────────────────────────────────────────
# All tuples are (R, G, B, A) in 0-255 range.

_BB_FILL      = (  80, 180, 255,  28)   # translucent sky-blue baitball zone
_BB_CENTER    = ( 100, 200, 255, 180)   # brighter center dot
_BB_EDGE      = (  80, 200, 255, 100)   # baitball perimeter ring
_BB_LABEL_CLR = ( 180, 230, 255, 230)

_LINK_ORBIT   = (  80, 190, 210, 120)   # dim cyan — orbiting/herding
_LINK_READY   = (  60, 230, 255, 200)   # bright cyan — strike-ready (target_lock)
_LINK_STRIKE  = ( 255, 220,  30, 220)   # yellow — active strike

_SLOT_CLR     = (  80, 255, 140, 200)   # green cross marker
_SLOT_LINE    = (  60, 200, 100, 120)   # dim green stub line

_STRIKE_PATH  = ( 255, 220,  30, 180)   # yellow strike exit line
_TARGET_HIGH  = ( 255,  80,  80, 200)   # red target highlight (edge strike)

_STATE_CLR    = ( 230, 230, 230, 200)

# Half-length of the cross arms at orbit slot markers (pixels).
_CROSS_ARM = 5

# State label abbreviations — keep short so they don't crowd at high density.
_STATE_ABBR: dict[str, str] = {
    "patrol":          "PTRL",
    "track_school":    "TRCK",
    "orbit_baitball":  "ORBT",
    "target_lock":     "LOCK",
    "burst_attack":    "BRST",
    "baitball_strike": "STRK",
    "recover":         "RCVR",
}


class BaitballDebugOverlay:
    """Draws optional baitball / sailfish-state debug visuals each frame.

    Usage::

        overlay = BaitballDebugOverlay()
        # --- inside Renderer.draw(), after self._batch.draw() ---
        overlay.draw(simulation, cfg)
    """

    def draw(self, simulation: "Simulation", cfg: "SimConfig") -> None:
        """Draw all enabled debug overlays.  Call once per frame after batch.draw()."""

        # ── Master gate ────────────────────────────────────────────────────
        master = cfg.debug_baitballs_enabled

        # State labels don't require the master toggle.
        if cfg.debug_sailfish_state_labels_enabled:
            self._draw_state_labels(simulation)

        if not master:
            return

        baitballs = simulation._baitballs
        sailfish  = simulation.sailfish

        if cfg.debug_baitball_shading_enabled:
            self._draw_baitball_shading(baitballs)

        if cfg.debug_baitball_labels_enabled:
            self._draw_baitball_labels(baitballs)

        if cfg.debug_sailfish_baitball_links_enabled:
            self._draw_sailfish_baitball_links(sailfish)

        if cfg.debug_sailfish_orbit_slots_enabled:
            self._draw_orbit_slot_markers(sailfish)

        if cfg.debug_sailfish_strike_paths_enabled:
            self._draw_strike_paths(sailfish)

    # ── Sub-renderers ──────────────────────────────────────────────────────

    def _draw_baitball_shading(self, baitballs: list) -> None:
        """Translucent filled circle + outline + center dot for each baitball."""
        for bb in baitballs:
            # Density-scaled opacity: more vivid for denser schools.
            alpha_fill = int(max(15, min(55, _BB_FILL[3] + (bb.density - 4.0) * 3)))
            alpha_edge = int(max(60, min(180, _BB_EDGE[3] + (bb.density - 4.0) * 8)))

            fill = pyglet.shapes.Circle(
                bb.cx, bb.cy, bb.radius,
                color=(*_BB_FILL[:3], alpha_fill),
            )
            fill.draw()

            # Perimeter arc (thin circle at radius edge).
            outline = pyglet.shapes.Circle(
                bb.cx, bb.cy, bb.radius,
                color=(*_BB_EDGE[:3], alpha_edge),
            )
            outline.draw()

            # Small solid center dot.
            dot = pyglet.shapes.Circle(
                bb.cx, bb.cy, 4,
                color=_BB_CENTER,
            )
            dot.draw()

    def _draw_baitball_labels(self, baitballs: list) -> None:
        """Small text labels: count / density / radius."""
        for bb in baitballs:
            label_text = (
                f"n={bb.count}  d={bb.density:.1f}  r={bb.radius:.0f}"
            )
            label = pyglet.text.Label(
                label_text,
                font_name="monospace",
                font_size=9,
                x=bb.cx,
                y=bb.cy + bb.radius + 8,
                anchor_x="center",
                anchor_y="bottom",
                color=_BB_LABEL_CLR,
            )
            label.draw()

    def _draw_sailfish_baitball_links(self, sailfish: list) -> None:
        """Line from each herding/striking sailfish to its baitball center."""
        for sf in sailfish:
            bb = sf._baitball
            if bb is None:
                continue
            state = sf.state
            if state in ("baitball_strike", "burst_attack"):
                color = _LINK_STRIKE
            elif state == "target_lock":
                color = _LINK_READY
            elif state in ("orbit_baitball", "track_school"):
                color = _LINK_ORBIT
            else:
                continue
            sx, sy = _pos(sf)
            line = pyglet.shapes.Line(
                sx, sy,
                bb.cx, bb.cy,
                thickness=1,
                color=color,
            )
            line.draw()

    def _draw_orbit_slot_markers(self, sailfish: list) -> None:
        """Small cross at each sailfish's computed orbit slot + stub line."""
        for sf in sailfish:
            slot_x = sf.debug_orbit_slot_x
            slot_y = sf.debug_orbit_slot_y
            if slot_x < 0:
                # Slot not active this frame.
                continue

            fx, fy = _pos(sf)
            # Stub line from fish body to slot.
            stub = pyglet.shapes.Line(
                fx, fy, slot_x, slot_y,
                thickness=1,
                color=_SLOT_LINE,
            )
            stub.draw()

            # Horizontal arm of the cross.
            h_arm = pyglet.shapes.Line(
                slot_x - _CROSS_ARM, slot_y, slot_x + _CROSS_ARM, slot_y,
                thickness=2,
                color=_SLOT_CLR,
            )
            h_arm.draw()

            # Vertical arm.
            v_arm = pyglet.shapes.Line(
                slot_x, slot_y - _CROSS_ARM, slot_x, slot_y + _CROSS_ARM,
                thickness=2,
                color=_SLOT_CLR,
            )
            v_arm.draw()

    def _draw_strike_paths(self, sailfish: list) -> None:
        """Strike exit path for center strikes; target highlight for edge strikes."""
        for sf in sailfish:
            if sf.state == "baitball_strike":
                # Center-pass strike: draw line to computed exit point.
                ex = sf._baitball_strike_exit_x
                ey = sf._baitball_strike_exit_y
                if ex != 0.0 or ey != 0.0:
                    fx, fy = _pos(sf)
                    line = pyglet.shapes.Line(
                        fx, fy, ex, ey,
                        thickness=2,
                        color=_STRIKE_PATH,
                    )
                    line.draw()

            elif sf.state == "target_lock":
                # Edge-target strike: highlight the locked sardine.
                # Guard against stale reference (sardine may have been eaten).
                target = sf._target_sardine
                if target is not None and hasattr(target, 'position'):
                    tx, ty = _pos(target)
                    highlight = pyglet.shapes.Circle(
                        tx, ty, 6,
                        color=_TARGET_HIGH,
                    )
                    highlight.draw()
                    fx, fy = _pos(sf)
                    link = pyglet.shapes.Line(
                        fx, fy, tx, ty,
                        thickness=2,
                        color=_TARGET_HIGH,
                    )
                    link.draw()

    def _draw_state_labels(self, simulation: "Simulation") -> None:
        """Small state abbreviation above each sailfish body."""
        for sf in simulation.sailfish:
            abbr = _STATE_ABBR.get(sf.state, sf.state[:4].upper())
            fx, fy = _pos(sf)
            label = pyglet.text.Label(
                abbr,
                font_name="monospace",
                font_size=8,
                x=fx,
                y=fy + 10,
                anchor_x="center",
                anchor_y="bottom",
                color=_STATE_CLR,
            )
            label.draw()
