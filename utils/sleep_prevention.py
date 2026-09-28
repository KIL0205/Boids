"""
utils/sleep_prevention.py
==========================
Lightweight, cross-platform sleep-prevention utility.

Prevents the host computer from sleeping, idling to system suspend, or
dimming/turning off the display while the simulation window is open.

Usage
-----
Call ``enable_sleep_prevention()`` once at app startup and
``disable_sleep_prevention()`` once at app shutdown.  Both calls are
idempotent and fail gracefully (logged, not raised) on unsupported platforms.

Platform support
----------------
macOS  : Spawns a single persistent ``caffeinate -d -i -m`` subprocess.
         ``caffeinate`` is a standard macOS system binary (OS X 10.8+).
         The subprocess is terminated on ``disable_sleep_prevention()``.
         Using a subprocess avoids the need for ctypes/IOKit and keeps the
         implementation minimal.  The process is deliberately *not* kept
         alive across ``enable``/``disable`` cycles — each enable starts a
         fresh subprocess.

Windows: Calls ``SetThreadExecutionState`` via ``ctypes.windll`` with
         ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED.
         Cleared with ES_CONTINUOUS alone on disable.

Linux  : Not currently supported (fails gracefully with a log message).
         systemd-inhibit or org.freedesktop.PowerManagement D-Bus calls
         could be added in future if needed.

Thread safety
-------------
Not thread-safe; intended to be called only from the main thread during
app lifecycle transitions.
"""

from __future__ import annotations

import logging
import platform
import subprocess
import sys

_log = logging.getLogger(__name__)

# ── Internal state ────────────────────────────────────────────────────────────

# macOS: the caffeinate process handle (or None when not active).
_caffeinate_proc: subprocess.Popen | None = None  # type: ignore[type-arg]

# Windows: whether SetThreadExecutionState is currently active.
_win_active: bool = False


# ── Public API ────────────────────────────────────────────────────────────────

def enable_sleep_prevention() -> bool:
    """Prevent the system from sleeping or dimming the display.

    Returns True if sleep prevention was successfully enabled, False on
    failure or unsupported platform.  Calling this more than once is safe —
    redundant calls are ignored.
    """
    _sys = platform.system()
    if _sys == "Darwin":
        return _enable_macos()
    elif _sys == "Windows":
        return _enable_windows()
    else:
        _log.warning("sleep_prevention: platform %r not supported; skipping.", _sys)
        return False


def disable_sleep_prevention() -> None:
    """Release the sleep-prevention assertion/process.

    Safe to call multiple times or without a prior ``enable_sleep_prevention``
    call.
    """
    _sys = platform.system()
    if _sys == "Darwin":
        _disable_macos()
    elif _sys == "Windows":
        _disable_windows()
    # Linux / other: nothing to release.


def is_active() -> bool:
    """Return True if sleep prevention is currently active."""
    _sys = platform.system()
    if _sys == "Darwin":
        return _caffeinate_proc is not None and _caffeinate_proc.poll() is None
    elif _sys == "Windows":
        return _win_active
    return False


# ── macOS implementation ──────────────────────────────────────────────────────

def _enable_macos() -> bool:
    global _caffeinate_proc

    if _caffeinate_proc is not None and _caffeinate_proc.poll() is None:
        # Already running — don't spawn a second process.
        return True

    try:
        # -d  prevent display sleep
        # -i  prevent idle system sleep
        # -m  prevent disk sleep
        # No -t timeout: runs until we terminate it explicitly.
        _caffeinate_proc = subprocess.Popen(
            ["/usr/bin/caffeinate", "-d", "-i", "-m"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            close_fds=True,
        )
        _log.info(
            "sleep_prevention: caffeinate started (pid %d).", _caffeinate_proc.pid
        )
        return True
    except FileNotFoundError:
        _log.warning(
            "sleep_prevention: /usr/bin/caffeinate not found; sleep prevention disabled."
        )
        _caffeinate_proc = None
        return False
    except OSError as exc:
        _log.warning("sleep_prevention: failed to start caffeinate: %s", exc)
        _caffeinate_proc = None
        return False


def _disable_macos() -> None:
    global _caffeinate_proc

    if _caffeinate_proc is None:
        return
    proc = _caffeinate_proc
    _caffeinate_proc = None
    if proc.poll() is not None:
        # Already exited on its own (shouldn't happen normally).
        return
    proc.terminate()
    try:
        proc.wait(timeout=2.0)
        _log.info("sleep_prevention: caffeinate terminated (pid %d).", proc.pid)
    except subprocess.TimeoutExpired:
        proc.kill()
        _log.warning(
            "sleep_prevention: caffeinate did not terminate in time; killed (pid %d).",
            proc.pid,
        )


# ── Windows implementation ────────────────────────────────────────────────────

# SetThreadExecutionState flags
_ES_CONTINUOUS:       int = 0x80000000
_ES_SYSTEM_REQUIRED:  int = 0x00000001
_ES_DISPLAY_REQUIRED: int = 0x00000002


def _enable_windows() -> bool:
    global _win_active

    if _win_active:
        return True
    try:
        import ctypes
        result = ctypes.windll.kernel32.SetThreadExecutionState(  # type: ignore[attr-defined]
            _ES_CONTINUOUS | _ES_SYSTEM_REQUIRED | _ES_DISPLAY_REQUIRED
        )
        if result == 0:
            _log.warning(
                "sleep_prevention: SetThreadExecutionState returned 0 (error)."
            )
            return False
        _win_active = True
        _log.info("sleep_prevention: SetThreadExecutionState enabled (Windows).")
        return True
    except Exception as exc:
        _log.warning("sleep_prevention: Windows sleep prevention failed: %s", exc)
        return False


def _disable_windows() -> None:
    global _win_active

    if not _win_active:
        return
    try:
        import ctypes
        ctypes.windll.kernel32.SetThreadExecutionState(  # type: ignore[attr-defined]
            _ES_CONTINUOUS
        )
        _log.info("sleep_prevention: SetThreadExecutionState cleared (Windows).")
    except Exception as exc:
        _log.warning(
            "sleep_prevention: failed to clear Windows sleep prevention: %s", exc
        )
    finally:
        _win_active = False
