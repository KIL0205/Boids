"""
config/config_io.py
===================
Config persistence for the sardine/sailfish boid simulation.

Provides two public functions:
    load_overrides()  ->  dict   Read boids_overrides.json and return a
                                 validated, safe-to-splat dict of overrides.
    save_config(cfg)  ->  None   Persist all SimConfig fields to disk.

This module is intentionally free of rendering, simulation, and UI imports
so it can be exercised in unit tests or future CLI tools without a display.

Override file
-------------
``boids_overrides.json`` lives next to ``main.py`` at the project root.
Format: a flat JSON object whose keys match ``SimConfig`` field names exactly.
Only fields you want to change need to be present — defaults for the rest
come from the code.  The file is created on the first ``save_config()`` call.

Startup contract
----------------
``window_width`` and ``window_height`` are always set from the live window
at runtime and are never read back from the file; stale screen dimensions
from a previous session are silently dropped.

Invalid entries
---------------
Any of the following are silently skipped so a partial or hand-edited file
never crashes the app:
  * Absent or unreadable file          → return {}
  * Top-level JSON is not an object    → return {}
  * Unknown field name                  → skip that key
  * Value has incompatible type         → skip that key (code default is used)
  * ``window_width`` / ``window_height``-> always skipped
"""

from __future__ import annotations

import dataclasses
import json
import pathlib
import typing
from typing import Any

from config.config import SimConfig

# Override file lives at the project root (one level up from this package).
OVERRIDE_FILE = pathlib.Path(__file__).parent.parent / "boids_overrides.json"

# Fields that must be derived from the live window at startup.
# Loading stale values from a previous session would break world geometry.
_RUNTIME_FIELDS: frozenset[str] = frozenset({"window_width", "window_height"})


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _type_hints() -> dict[str, Any]:
    """Return fully resolved type hints for all ``SimConfig`` fields."""
    return typing.get_type_hints(SimConfig)


def _coerce(field_name: str, value: Any, hint: Any) -> Any:
    """Coerce *value* to the type described by *hint*.

    Raises ``ValueError`` or ``TypeError`` for incompatible inputs; the
    caller catches these and skips the entry rather than crashing.
    """
    origin = getattr(hint, "__origin__", None)

    # bool must come before int because bool is a subclass of int in Python.
    if hint is bool:
        if not isinstance(value, bool):
            raise TypeError(
                f"'{field_name}': expected bool, got {type(value).__name__}"
            )
        return value

    if hint is int:
        if isinstance(value, bool):
            raise TypeError(f"'{field_name}': bool not acceptable for int field")
        # JSON makes no distinction between 1 and 1.0; accept whole-number floats.
        if isinstance(value, float) and not value.is_integer():
            raise ValueError(
                f"'{field_name}': non-integer float {value!r} for int field"
            )
        return int(value)

    if hint is float:
        if isinstance(value, bool):
            raise TypeError(f"'{field_name}': bool not acceptable for float field")
        # int literals from JSON (e.g. "radius_separation": 40) are fine.
        return float(value)

    if hint is str:
        if not isinstance(value, str):
            raise TypeError(
                f"'{field_name}': expected str, got {type(value).__name__}"
            )
        return value

    # Handles tuple[int, int, int, int] (RGBA colour fields).
    if origin is tuple:
        if not isinstance(value, (list, tuple)):
            raise TypeError(
                f"'{field_name}': expected list, got {type(value).__name__}"
            )
        args = hint.__args__  # e.g. (int, int, int, int)
        if len(value) != len(args):
            raise ValueError(
                f"'{field_name}': expected {len(args)}-element tuple, "
                f"got {len(value)}"
            )
        return tuple(t(v) for t, v in zip(args, value))

    # Unknown annotation type — pass through unchanged (future-proof).
    return value


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def load_overrides() -> dict[str, Any]:
    """Read saved overrides from ``boids_overrides.json``.

    Returns a ``dict`` of ``{field_name: value}`` pairs that are safe to
    splat directly into ``SimConfig(**overrides)``.

    The file is optional — an empty dict is returned when it is absent,
    unreadable, or empty.  Invalid or unrecognised entries are silently
    dropped so a hand-edited file never stops the simulation from starting.
    """
    if not OVERRIDE_FILE.exists():
        return {}

    try:
        raw = json.loads(OVERRIDE_FILE.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        # Corrupt or unreadable file — run with code defaults.
        return {}

    if not isinstance(raw, dict):
        # Top-level value is not an object (e.g. someone saved a JSON array).
        return {}

    hints = _type_hints()
    result: dict[str, Any] = {}

    for key, value in raw.items():
        if key in _RUNTIME_FIELDS:
            # Always set from live window; ignore whatever was saved.
            continue
        if key not in hints:
            # Field was removed or renamed since the file was saved.
            continue
        try:
            result[key] = _coerce(key, value, hints[key])
        except (ValueError, TypeError):
            # Type mismatch — fall back to the code default for this field.
            continue

    return result


def save_config(config: SimConfig) -> None:
    """Persist all ``SimConfig`` fields to ``boids_overrides.json``.

    The file is a flat, human-readable JSON object.  Colour tuples are
    written as JSON arrays (``[R, G, B, A]``).  The write is performed via
    a temporary file that is then renamed, which is atomic on POSIX systems
    (macOS / Linux) so a crash mid-write never corrupts the saved config.

    ``window_width`` and ``window_height`` *are* written so that the file
    records a full snapshot; they are simply ignored on the next load (see
    ``_RUNTIME_FIELDS``).
    """
    data = dataclasses.asdict(config)
    text = json.dumps(data, indent=2)
    # Atomic replace: write to sibling temp file, then rename over target.
    tmp = OVERRIDE_FILE.with_suffix(".json.tmp")
    try:
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(OVERRIDE_FILE)
    except OSError:
        # If the rename fails for any reason, clean up and re-raise so the
        # caller (menu save button) can surface a visible error.
        tmp.unlink(missing_ok=True)
        raise
