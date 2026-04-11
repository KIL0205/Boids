"""
config/config_menu.py
=====================
Translucent in-app config panel for the sardine/sailfish simulation.

Toggle with TAB.  While the panel is visible:
  ↑ / ↓          — move cursor between rows
  ← / →          — edit value (parameter rows) / activate Save (Save row)
  Shift + ← / →  — coarse edit (parameter rows only)
  Enter / Return  — activate Save (when cursor is on the Save row)
  S              — save from any row
  ESC or TAB     — close the menu (simulation keeps running)

The simulation continues to run while the menu is open.

Draw the overlay by calling ``menu.draw()`` after ``Renderer.draw()``; it
is a no-op when the menu is not visible so there is no branch needed at the
call site.
"""

from __future__ import annotations

import dataclasses
import time
from typing import TYPE_CHECKING

import pyglet
import pyglet.shapes
import pyglet.text
import pyglet.window.key as Key

from config.config_io import save_config

if TYPE_CHECKING:
    from sim.simulation import Simulation

# ── Layout constants ──────────────────────────────────────────────────────

_PANEL_W:   int = 320   # panel width (px)
_PANEL_X:   int = 20    # distance from left window edge
_PANEL_TOP: int = 20    # distance from top window edge
_HEADER_H:  int = 28    # title bar height (px)
_FOOTER_H:  int = 22    # footer hint strip height (px)
_ROW_H:     int = 22    # height of each parameter row (px)
_PAD_V:     int = 8     # vertical padding above and below the row list
_FONT: list[str] = ["Menlo", "Consolas", "Courier New"]
_FONT_SZ:   int = 13

# Maximum number of rows visible at once.  Panel height is fixed to this
# count so the overlay stays compact on any window size.  If the total row
# count exceeds this, the panel scrolls automatically as you navigate.
_VISIBLE_ROWS: int = 16

# ── Colours ───────────────────────────────────────────────────────────────

_C_BG_RGB:       tuple[int, int, int] = (10,  18,  35)
_C_BG_ALPHA:     int = 185    # ~73 % opaque

_C_HDR_RGB:      tuple[int, int, int] = (15,  28,  55)
_C_HDR_ALPHA:    int = 215

_C_FTR_RGB:      tuple[int, int, int] = (12,  22,  45)
_C_FTR_ALPHA:    int = 200

_C_CURSOR_RGB:   tuple[int, int, int] = (45, 100, 195)
_C_CURSOR_ALPHA: int = 140    # cursor highlight bar

_C_HDR_TEXT:     tuple[int, int, int, int] = (170, 200, 255, 255)
_C_HDR_SAVE:     tuple[int, int, int, int] = (120, 240, 140, 255)   # green "✓ saved"
_C_HDR_ERR:      tuple[int, int, int, int] = (240,  90,  70, 255)   # red   "✗ error"
_C_ROW_SEL:      tuple[int, int, int, int] = (215, 235, 255, 255)   # bright white-blue
_C_ROW_DIM:      tuple[int, int, int, int] = (115, 145, 175, 185)   # muted grey-blue
_C_FTR_TEXT:     tuple[int, int, int, int] = (100, 130, 165, 160)   # very muted
_C_GROUP_TEXT:   tuple[int, int, int, int] = ( 80, 108, 150, 165)   # dim section headers

# How long to show "✓ saved" / "✗ error" in the header (seconds).
_SAVE_FEEDBACK_SECS: float = 1.5
_SAVE_ERR_SECS:      float = 2.5


# ── Menu item definition ──────────────────────────────────────────────────

@dataclasses.dataclass(frozen=True)
class _MenuItem:
    """Metadata for one editable numeric config row."""

    label:   str    # display label (max 16 chars for column alignment)
    field:   str    # SimConfig attribute name
    step:    float  # fine step applied by ← / →
    coarse:  float  # Shift step (typically 10 × fine step)
    min_val: float  # inclusive lower bound
    max_val: float  # inclusive upper bound


@dataclasses.dataclass(frozen=True)
class _ToggleItem:
    """A boolean config field toggled by ← / → or Enter."""

    label: str   # display label (max 16 chars)
    field: str   # SimConfig attribute name (must be bool)


@dataclasses.dataclass(frozen=True)
class _CycleItem:
    """A string config field that cycles through a fixed ordered list of options.

    ← / → (or Enter) step backward / forward through ``options``.
    """

    label:   str            # display label (max 16 chars)
    field:   str            # SimConfig attribute name (must be str)
    options: tuple[str, ...]  # ordered cycle list (non-empty)


@dataclasses.dataclass(frozen=True)
class _SaveRow:
    """A non-editable menu row that activates the save-to-disk action."""
    label: str = "Save to file"


@dataclasses.dataclass(frozen=True)
class _GroupRow:
    """Non-navigable section-header row rendered as a dim divider."""
    label: str


# ── Row definitions ───────────────────────────────────────────────────────
#
# Structure:
#   SARDINES           — movement, flocking weights+radii, flee, color
#   MOUSE PREDATOR     — toggle, radius, flee wt
#   SAILFISH—FLOCKING  — movement, sailfish-to-sailfish flocking, color
#   SAILFISH—HUNTING   — COM pursuit, individual targeting, vulnerability
#                        scoring, school-pressure/orbit
#   SAILFISH—BURST     — burst trigger, duration, cooldown, multipliers
#   ALGAE              — drift kinematics, flocking weights, respawn/ecology
#   ECOLOGY            — sardine + sailfish reproduction/starvation
#   Save

_ROWS: list[_MenuItem | _ToggleItem | _CycleItem | _SaveRow | _GroupRow] = [
    # ── Sardines ──────────────────────────────────────────────────────
    _GroupRow("SARDINES"),
    # Movement
    _MenuItem("Sardine speed",   "sardine_max_speed",       10.0,  50.0,  50.0,  500.0),
    _MenuItem("Sardine force",   "sardine_max_force",       10.0,  50.0,  10.0,  500.0),
    # Flocking weights
    _MenuItem("Sep weight",      "weight_separation",        0.1,   1.0,   0.0,   10.0),
    _MenuItem("Ali weight",      "weight_alignment",         0.1,   1.0,   0.0,   10.0),
    _MenuItem("Coh weight",      "weight_cohesion",          0.1,   1.0,   0.0,   10.0),
    _MenuItem("Noise weight",    "weight_noise",             0.01,  0.1,   0.0,    0.5),
    # Perception radii
    _MenuItem("Sep radius",      "radius_separation",        2.0,  20.0,   5.0,  100.0),
    _MenuItem("Ali radius",      "radius_alignment",         2.0,  20.0,  10.0,  150.0),
    _MenuItem("Coh radius",      "radius_cohesion",          2.0,  20.0,  10.0,  150.0),
    # Flee / predator avoidance
    _MenuItem("Flee weight",     "weight_flee",              0.5,   5.0,   0.0,   30.0),
    _MenuItem("Flee radius",     "radius_flee",              5.0,  50.0,  20.0,  300.0),
    # Color
    _CycleItem("Color mode",     "sardine_color_mode",
               ("white", "rainbow", "cool", "warm", "ocean", "sunset", "mono")),
    _MenuItem("Vel heading wt",  "vel_color_heading_weight", 0.1,   0.5,   0.0,    2.0),
    _MenuItem("Vel speed wt",    "vel_color_speed_weight",   0.1,   0.5,   0.0,    2.0),
    _MenuItem("Vel turn wt",     "vel_color_turn_weight",    0.1,   0.5,   0.0,    2.0),
    _MenuItem("Vel smoothing",   "vel_color_smoothing",      0.05,  0.1,   0.0,    0.99),

    # ── Mouse predator ────────────────────────────────────────────────
    _GroupRow("MOUSE PREDATOR"),
    _ToggleItem("Mouse predator", "mouse_predator_enabled"),
    _MenuItem("Mouse radius",    "mouse_predator_radius",    5.0,  50.0,  20.0,  400.0),
    _MenuItem("Mouse flee wt",   "mouse_predator_flee_weight", 0.1, 0.5,   0.0,    5.0),

    # ── Sailfish: movement + sailfish-to-sailfish flocking ────────────
    _GroupRow("SAILFISH — FLOCKING"),
    # Movement
    _MenuItem("Sailfish speed",  "sailfish_max_speed",      10.0,  50.0,  50.0,  600.0),
    _MenuItem("Sailfish force",  "sailfish_max_force",      10.0,  50.0,  10.0,  600.0),
    # Sailfish-to-sailfish flocking
    _MenuItem("SF sep weight",   "weight_sailfish_separation",  0.1,   1.0,   0.0,   10.0),
    _MenuItem("SF sep radius",   "radius_sailfish_separation",  2.0,  20.0,   5.0,  200.0),
    _MenuItem("SF ali weight",   "weight_sailfish_alignment",   0.1,   1.0,   0.0,   10.0),
    _MenuItem("SF ali radius",   "radius_sailfish_alignment",   5.0,  50.0,  20.0,  400.0),
    _MenuItem("SF coh weight",   "weight_sailfish_cohesion",    0.1,   1.0,   0.0,   10.0),
    _MenuItem("SF coh radius",   "radius_sailfish_cohesion",    5.0,  50.0,  20.0,  400.0),
    # Color
    _CycleItem("Color mode",     "sailfish_color_mode",
               ("default", "white", "red", "blue", "black")),

    # ── Sailfish: hunting, targeting, and school-pressure ─────────────
    _GroupRow("SAILFISH — HUNTING"),
    # Long-range COM pursuit
    _MenuItem("Pursuit wt",      "weight_pursuit",           0.1,   1.0,   0.0,   10.0),
    _MenuItem("Pursuit radius",  "radius_sailfish_awareness",5.0,  50.0,  50.0,  400.0),
    # Individual target acquisition
    _MenuItem("Acquire radius",  "sailfish_target_acquire_radius", 5.0, 50.0, 20.0, 400.0),
    _MenuItem("Drop radius",     "sailfish_target_drop_radius",    5.0, 50.0, 20.0, 600.0),
    _MenuItem("Lock time",       "sailfish_target_lock_time",      0.5,  1.0,  0.0,  30.0),
    _MenuItem("Target wt",       "weight_target_pursuit",          0.1,  1.0,  0.0,  10.0),
    # Vulnerability scoring: higher weight → stronger preference
    _MenuItem("Tgt dist wt",     "sailfish_target_dist_weight",    0.1,  0.5,  0.0,   5.0),
    _MenuItem("Tgt iso wt",      "sailfish_target_isolation_weight",0.1, 0.5,  0.0,   5.0),
    _MenuItem("Tgt spd wt",      "sailfish_target_speed_weight",   0.1,  0.5,  0.0,   5.0),
    # School-pressure / orbit bias (activates when no target, school dense)
    _MenuItem("Pressure str",    "sailfish_school_pressure_strength",        0.1, 0.5, 0.0, 5.0),
    _MenuItem("Pressure dens",   "sailfish_school_pressure_density_threshold", 1.0, 5.0, 1.0, 100.0),

    # ── Sailfish: burst attack ─────────────────────────────────────────
    _GroupRow("SAILFISH — BURST"),
    _MenuItem("Burst trig r",    "sailfish_burst_trigger_radius",  5.0, 50.0, 10.0, 300.0),
    _MenuItem("Burst duration",  "sailfish_burst_duration",        0.1,  0.5,  0.0,   5.0),
    _MenuItem("Burst cooldown",  "sailfish_burst_cooldown",        0.5,  1.0,  0.0,  15.0),
    _MenuItem("Burst spd mult",  "sailfish_burst_speed_mult",      0.05, 0.1,  1.0,   3.0),
    _MenuItem("Burst frc mult",  "sailfish_burst_force_mult",      0.1,  0.5,  1.0,   4.0),

    # ── Algae ──────────────────────────────────────────────────────────
    _GroupRow("ALGAE"),
    # Drift kinematics and flocking weights
    _MenuItem("Algae speed",     "algae_max_speed",          1.0,   5.0,   1.0,  100.0),
    _MenuItem("Algae align wt",  "weight_algae_alignment",   0.1,   0.5,   0.0,    5.0),
    _MenuItem("Algae sep wt",    "weight_algae_separation",  0.1,   0.5,   0.0,    5.0),
    # Ecology
    _MenuItem("Algae respawn",   "algae_respawn_rate",       2.0,   5.0,   0.0,  200.0),
    _MenuItem("Algae max",       "algae_max_count",         50.0, 200.0, 100.0, 5000.0),
    _MenuItem("Algae eat r",     "algae_eat_radius",         1.0,   5.0,   4.0,   50.0),

    # ── Ecology: reproduction and starvation ───────────────────────────
    _GroupRow("ECOLOGY"),
    # Sardine
    _MenuItem("Sardine repro",   "sardine_eat_threshold",    1.0,   5.0,   0.0,   20.0),
    _MenuItem("Sardine starve",  "sardine_starvation_time",  1.0,  10.0,   0.0,  300.0),
    _MenuItem("Sardine max",     "sardine_max_count",       50.0, 200.0,   0.0, 3000.0),
    # Sailfish
    _MenuItem("Sailfish eat r",  "sailfish_eat_radius",      1.0,   5.0,  10.0,   80.0),
    _MenuItem("Sailfish repro",  "sailfish_eat_threshold",   1.0,   5.0,   0.0,   75.0),
    _MenuItem("Sailfish starve", "sailfish_starvation_time", 1.0,  30.0,   0.0,  600.0),
    _MenuItem("Sailfish max",    "sailfish_max_count",       1.0,   5.0,   0.0,  200.0),
    _ToggleItem("SF starvation", "sailfish_starvation_enabled"),
    _ToggleItem("SF repro",      "sailfish_reproduction_enabled"),

    _SaveRow(),
]

_NUM_ROWS: int = len(_ROWS)


# ── Helpers ───────────────────────────────────────────────────────────────

def _fmt_value(val: object) -> str:
    """Format a SimConfig value for display in the menu."""
    if isinstance(val, bool):
        return "ON" if val else "OFF"
    if isinstance(val, float):
        # Show 2 decimal places for values < 1 so 0.04 isn't just "0.0".
        return f"{val:.2f}" if val < 1.0 else f"{val:.1f}"
    return str(val)


def _row_text(
    item: "_MenuItem | _ToggleItem | _CycleItem | _SaveRow",
    cfg: object,
    is_selected: bool,
    save_state: str = "",
) -> str:
    """Build the fixed-width monospace display string for a row.

    Format: ``▶ <label>      <value>`` (selected)
            ``  <label>      <value>`` (unselected)
    Columns: 2-char prefix · 16-char label · 8-char right-aligned value.

    Note: ``_GroupRow`` items are rendered directly in ``_build``/``_refresh_rows``
    and are never passed to this function.
    """
    prefix = "\u25b6 " if is_selected else "  "   # ▶ or spaces
    if isinstance(item, _SaveRow):
        val_str = (
            "\u2713 saved" if save_state == "saved" else   # ✓ saved  (7 chars)
            "\u2717 error" if save_state == "error" else   # ✗ error  (7 chars)
            "[S/Ret]"                                       # idle     (7 chars)
        )
        return f"{prefix}{item.label:<16}{val_str:>8}"
    val_str = _fmt_value(getattr(cfg, item.field))
    return f"{prefix}{item.label:<16}{val_str:>8}"


def _footer_text(item: "_MenuItem | _ToggleItem | _CycleItem | _SaveRow | _GroupRow") -> str:
    """One-line hint for the currently selected row."""
    if isinstance(item, _GroupRow):
        return ""
    if isinstance(item, _SaveRow):
        return "  \u21b5 Enter / \u2190\u2192  \u00b7  saves to boids_overrides.json"
    if isinstance(item, (_ToggleItem, _CycleItem)):
        opts_str = " / ".join(item.options) if isinstance(item, _CycleItem) else "on / off"
        return f"  \u2190\u2192 or \u21b5 Enter  \u00b7  {opts_str}"
    s  = _fmt_value(item.step)
    cs = _fmt_value(item.coarse)
    lo = _fmt_value(item.min_val)
    hi = _fmt_value(item.max_val)
    return f"  \u2190\u2192 \u00b1{s}  \u00b7  Shift \u00b1{cs}  \u00b7  [{lo}\u2013{hi}]"


# ── Main class ────────────────────────────────────────────────────────────

class ConfigMenu:
    """Translucent overlay for live config inspection and editing.

    Values are changed immediately and propagated to all agents via
    ``Simulation.apply_config()``.  The simulation runs throughout.

    Create once after both the pyglet ``Window`` and ``Simulation`` exist.
    """

    def __init__(
        self,
        window:     pyglet.window.Window,
        simulation: "Simulation",
    ) -> None:
        self._window     = window
        self._simulation = simulation

        self.visible:           bool  = False
        self._selected:         int   = next(
            i for i, r in enumerate(_ROWS) if not isinstance(r, _GroupRow)
        )
        self._dirty:            bool  = False
        self._save_until:       float = 0.0
        self._save_error_until: float = 0.0
        self._scroll_offset:    int   = 0

        # Pyglet objects — None until _build() is called.
        self._bg:         pyglet.shapes.Rectangle | None = None
        self._hdr_bg:     pyglet.shapes.Rectangle | None = None
        self._ftr_bg:     pyglet.shapes.Rectangle | None = None
        self._cursor_bar: pyglet.shapes.Rectangle | None = None
        self._sel_bar:    pyglet.shapes.Rectangle | None = None
        self._hdr_label:  pyglet.text.Label | None = None
        self._ftr_label:  pyglet.text.Label | None = None
        self._row_labels: list[pyglet.text.Label] = []

        self._build(window.width, window.height)

    # ── Geometry ──────────────────────────────────────────────────────

    def _panel_h(self) -> int:
        """Total panel height in pixels (fixed; rows scroll within this window)."""
        return _HEADER_H + _PAD_V + _VISIBLE_ROWS * _ROW_H + _PAD_V + _FOOTER_H

    def _panel_origin(self, win_h: int) -> tuple[int, int]:
        """Bottom-left corner of the panel (pyglet y-up coordinates)."""
        py = win_h - _PANEL_TOP - self._panel_h()
        return _PANEL_X, max(0, py)

    def _row_center_y(self, panel_y: int, index: int) -> int:
        """Vertical centre of row *index* in pyglet coords."""
        body_top = panel_y + self._panel_h() - _HEADER_H - _PAD_V
        return body_top - index * _ROW_H - _ROW_H // 2

    def _row_bottom_y(self, panel_y: int, index: int) -> int:
        """Bottom edge of row *index* in pyglet coords."""
        body_top = panel_y + self._panel_h() - _HEADER_H - _PAD_V
        return body_top - (index + 1) * _ROW_H

    def _ensure_visible(self) -> None:
        """Clamp _scroll_offset so _selected is always in the visible window."""
        if self._selected < self._scroll_offset:
            self._scroll_offset = self._selected
        elif self._selected >= self._scroll_offset + _VISIBLE_ROWS:
            self._scroll_offset = self._selected - _VISIBLE_ROWS + 1

    # ── Build / teardown ──────────────────────────────────────────────

    def _delete_all(self) -> None:
        """Release all pyglet objects so _build can start clean."""
        for obj in (
            self._bg, self._hdr_bg, self._ftr_bg,
            self._cursor_bar, self._sel_bar,
            self._hdr_label, self._ftr_label,
        ):
            if obj is not None:
                obj.delete()
        for lbl in self._row_labels:
            lbl.delete()
        self._row_labels.clear()
        self._bg = self._hdr_bg = self._ftr_bg = None
        self._cursor_bar = self._sel_bar = None
        self._hdr_label  = self._ftr_label = None

    def _build(self, win_w: int, win_h: int) -> None:
        """Allocate all shapes and labels for the given window dimensions."""
        self._delete_all()

        px, py = self._panel_origin(win_h)
        ph     = self._panel_h()

        # ── Backgrounds ───────────────────────────────────────────────
        self._bg = pyglet.shapes.Rectangle(
            px, py, _PANEL_W, ph, color=_C_BG_RGB,
        )
        self._bg.opacity = _C_BG_ALPHA

        self._hdr_bg = pyglet.shapes.Rectangle(
            px, py + ph - _HEADER_H, _PANEL_W, _HEADER_H, color=_C_HDR_RGB,
        )
        self._hdr_bg.opacity = _C_HDR_ALPHA

        self._ftr_bg = pyglet.shapes.Rectangle(
            px, py, _PANEL_W, _FOOTER_H, color=_C_FTR_RGB,
        )
        self._ftr_bg.opacity = _C_FTR_ALPHA

        # ── Cursor highlight bar + left-edge accent ────────────────────
        _visible_sel = self._selected - self._scroll_offset
        _cursor_y = (
            self._row_bottom_y(py, _visible_sel)
            if 0 <= _visible_sel < _VISIBLE_ROWS
            else -9999
        )
        self._cursor_bar = pyglet.shapes.Rectangle(
            px, _cursor_y, _PANEL_W, _ROW_H, color=_C_CURSOR_RGB,
        )
        self._cursor_bar.opacity = _C_CURSOR_ALPHA

        self._sel_bar = pyglet.shapes.Rectangle(
            px, _cursor_y, 3, _ROW_H, color=(100, 160, 255),
        )
        self._sel_bar.opacity = 230

        # ── Header label ───────────────────────────────────────────────
        self._hdr_label = pyglet.text.Label(
            self._hdr_text(),
            font_name=_FONT,
            font_size=11,
            weight="bold",
            x=px + 10,
            y=py + ph - _HEADER_H // 2,
            anchor_y="center",
            color=_C_HDR_TEXT,
        )

        # ── Footer label ───────────────────────────────────────────────
        self._ftr_label = pyglet.text.Label(
            _footer_text(_ROWS[self._selected]),
            font_name=_FONT,
            font_size=10,
            x=px + 4,
            y=py + _FOOTER_H // 2,
            anchor_y="center",
            color=_C_FTR_TEXT,
        )

        # ── Row labels ──────────────────────────────────────────────────
        cfg        = self._simulation.config
        save_state = self._save_row_state()
        self._row_labels = []
        for i, item in enumerate(_ROWS):
            sel = (i == self._selected)
            if isinstance(item, _GroupRow):
                lbl_text  = item.label
                lbl_color = _C_GROUP_TEXT
                fs        = 10
            elif isinstance(item, _SaveRow):
                lbl_text  = _row_text(item, cfg, sel, save_state)
                lbl_color = (
                    _C_HDR_SAVE if save_state == "saved" else
                    _C_HDR_ERR  if save_state == "error" else
                    _C_ROW_SEL  if sel else _C_ROW_DIM
                )
                fs        = _FONT_SZ
            else:
                lbl_text  = _row_text(item, cfg, sel)
                lbl_color = _C_ROW_SEL if sel else _C_ROW_DIM
                fs        = _FONT_SZ
            visible_idx = i - self._scroll_offset
            row_y = (
                self._row_center_y(py, visible_idx)
                if 0 <= visible_idx < _VISIBLE_ROWS
                else -9999
            )
            self._row_labels.append(
                pyglet.text.Label(
                    lbl_text,
                    font_name=_FONT,
                    font_size=fs,
                    x=px + 10,
                    y=row_y,
                    anchor_y="center",
                    color=lbl_color,
                )
            )

        self._dirty = False

    # ── Refresh ───────────────────────────────────────────────────────

    def _save_row_state(self) -> str:
        """Return 'saved', 'error', or '' (idle) for the Save row display."""
        now = time.perf_counter()
        if self._save_until       and now < self._save_until:
            return "saved"
        if self._save_error_until and now < self._save_error_until:
            return "error"
        return ""

    def _hdr_text(self) -> str:
        """Return normal key-guide, save-feedback, or error header string."""
        state = self._save_row_state()
        if state == "saved":
            return "CONFIG   \u2713 saved"
        if state == "error":
            return "CONFIG   \u2717 write error"
        return "CONFIG  \u2191\u2193 nav  \u2190\u2192 edit  S save  ESC"

    def _refresh_rows(self) -> None:
        """Sync label text, colours, cursor bar, and footer text with state."""
        _, py = self._panel_origin(self._window.height)
        cfg   = self._simulation.config

        # Cursor bar + left-edge accent (hidden when selection scrolls off-screen)
        visible_sel = self._selected - self._scroll_offset
        if 0 <= visible_sel < _VISIBLE_ROWS:
            row_y = self._row_bottom_y(py, visible_sel)
            self._cursor_bar.y = row_y
            self._sel_bar.y    = row_y
        else:
            self._cursor_bar.y = -9999
            self._sel_bar.y    = -9999

        # Header (may show save feedback or error)
        save_state = self._save_row_state()
        self._hdr_label.text  = self._hdr_text()
        self._hdr_label.color = (
            _C_HDR_SAVE if save_state == "saved" else
            _C_HDR_ERR  if save_state == "error" else
            _C_HDR_TEXT
        )

        # Footer hint: selected-row hint + optional scroll indicator
        can_up   = self._scroll_offset > 0
        can_down = self._scroll_offset + _VISIBLE_ROWS < _NUM_ROWS
        row_hint = _footer_text(_ROWS[self._selected])
        if can_up and can_down:
            scroll_hint = "  \u25b2\u25bc"
        elif can_up:
            scroll_hint = "  \u25b2"
        elif can_down:
            scroll_hint = "  \u25bc"
        else:
            scroll_hint = ""
        self._ftr_label.text = row_hint + scroll_hint

        # Row labels — update text, colour, and y position (scroll may shift rows)
        for i, item in enumerate(_ROWS):
            sel = (i == self._selected)
            lbl = self._row_labels[i]
            visible_idx = i - self._scroll_offset
            lbl.y = (
                self._row_center_y(py, visible_idx)
                if 0 <= visible_idx < _VISIBLE_ROWS
                else -9999
            )
            if isinstance(item, _GroupRow):
                pass  # group headers are static — text and colour never change
            elif isinstance(item, _SaveRow):
                lbl.text  = _row_text(item, cfg, sel, save_state)
                lbl.color = (
                    _C_HDR_SAVE if save_state == "saved" else
                    _C_HDR_ERR  if save_state == "error" else
                    _C_ROW_SEL  if sel else _C_ROW_DIM
                )
            else:
                lbl.text  = _row_text(item, cfg, sel)
                lbl.color = _C_ROW_SEL if sel else _C_ROW_DIM

        self._dirty = False

    # ── Public API ────────────────────────────────────────────────────

    def toggle(self) -> None:
        """Open the menu if closed; close it if open."""
        self.visible = not self.visible
        if self.visible:
            self._dirty = True   # refresh values on open

    def handle_key(self, symbol: int, modifiers: int) -> bool:
        """Process a key press; return True if consumed."""
        if not self.visible:
            return False

        if symbol == Key.UP:
            new = (self._selected - 1) % _NUM_ROWS
            while isinstance(_ROWS[new], _GroupRow):
                new = (new - 1) % _NUM_ROWS
            self._selected = new
            self._ensure_visible()
            self._dirty = True
            return True

        if symbol == Key.DOWN:
            new = (self._selected + 1) % _NUM_ROWS
            while isinstance(_ROWS[new], _GroupRow):
                new = (new + 1) % _NUM_ROWS
            self._selected = new
            self._ensure_visible()
            self._dirty = True
            return True

        if symbol in (Key.LEFT, Key.RIGHT):
            item = _ROWS[self._selected]
            if isinstance(item, _SaveRow):
                self._save()
            elif isinstance(item, _ToggleItem):
                self._toggle_bool(item)
            elif isinstance(item, _CycleItem):
                self._cycle_value(item, symbol)
            else:
                self._adjust_value(symbol, modifiers)
            return True

        if symbol in (Key.ENTER, Key.RETURN):
            item = _ROWS[self._selected]
            if isinstance(item, _SaveRow):
                self._save()
                return True
            if isinstance(item, _ToggleItem):
                self._toggle_bool(item)
                return True
            if isinstance(item, _CycleItem):
                self._cycle_value(item, Key.RIGHT)
                return True
            return False

        if symbol == Key.S:
            self._save()
            return True

        if symbol in (Key.ESCAPE, Key.TAB):
            self.visible = False
            return True

        # H, F, and other global keys fall through unchanged.
        return False

    def on_resize(self, width: int, height: int) -> None:
        """Rebuild the panel geometry after a window resize or fullscreen toggle."""
        self._build(width, height)

    def draw(self) -> None:
        """Draw the overlay on top of the current frame.  No-op if not visible."""
        if not self.visible:
            return

        # Let the save-feedback timers expire naturally between frames.
        now = time.perf_counter()
        if self._save_until and now >= self._save_until:
            self._save_until = 0.0
            self._dirty = True
        if self._save_error_until and now >= self._save_error_until:
            self._save_error_until = 0.0
            self._dirty = True

        if self._dirty:
            self._refresh_rows()

        # Draw order: bg → header strip → footer strip → cursor bar → accent → text
        self._bg.draw()
        self._hdr_bg.draw()
        self._ftr_bg.draw()
        self._cursor_bar.draw()
        self._sel_bar.draw()
        self._hdr_label.draw()
        for lbl in self._row_labels:
            lbl.draw()
        self._ftr_label.draw()

    # ── Private helpers ───────────────────────────────────────────────

    def _adjust_value(self, symbol: int, modifiers: int) -> None:
        """Change the selected numeric config field by one step and propagate live."""
        item      = _ROWS[self._selected]
        cfg       = self._simulation.config
        cur_typed = getattr(cfg, item.field)
        cur       = float(cur_typed)
        step      = item.coarse if (modifiers & Key.MOD_SHIFT) else item.step
        delta     = step if symbol == Key.RIGHT else -step

        new_val = round(max(item.min_val, min(item.max_val, cur + delta)), 6)

        if new_val == cur:
            return   # already at boundary

        # Cast back to int for int config fields (avoids TypeError on frozen dataclass).
        if isinstance(cur_typed, int):
            new_val = int(new_val)

        new_cfg = dataclasses.replace(cfg, **{item.field: new_val})
        self._simulation.apply_config(new_cfg)
        self._dirty = True

    def _toggle_bool(self, item: "_ToggleItem") -> None:
        """Flip a boolean config field and propagate live."""
        cfg     = self._simulation.config
        cur_val = bool(getattr(cfg, item.field))
        new_cfg = dataclasses.replace(cfg, **{item.field: not cur_val})
        self._simulation.apply_config(new_cfg)
        self._dirty = True

    def _cycle_value(self, item: "_CycleItem", symbol: int) -> None:
        """Step a string enum field forward (RIGHT) or backward (LEFT)."""
        cfg  = self._simulation.config
        cur  = str(getattr(cfg, item.field))
        opts = list(item.options)
        try:
            idx = opts.index(cur)
        except ValueError:
            idx = 0
        delta   = 1 if symbol == Key.RIGHT else -1
        new_idx = (idx + delta) % len(opts)
        new_cfg = dataclasses.replace(cfg, **{item.field: opts[new_idx]})
        self._simulation.apply_config(new_cfg)
        self._dirty = True

    def _save(self) -> None:
        """Write current config to disk; show brief header and row feedback."""
        try:
            save_config(self._simulation.config)
            self._save_until       = time.perf_counter() + _SAVE_FEEDBACK_SECS
            self._save_error_until = 0.0
        except OSError:
            self._save_error_until = time.perf_counter() + _SAVE_ERR_SECS
            self._save_until       = 0.0
        self._dirty = True
