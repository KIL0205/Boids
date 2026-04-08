"""
main.py
=======
Entry point for the sardine/sailfish boid simulation.

Run with:
    python main.py

Wires together Config → Simulation → Renderer and hands control to
the pyglet event loop.  All simulation logic lives in simulation.py;
all drawing lives in renderer.py.

Dependencies:
    pip install -r requirements.txt
"""

from __future__ import annotations

import time

import pyglet

from config.config import SimConfig
from config.config_io import load_overrides
from config.config_menu import ConfigMenu
from sim.simulation import Simulation
from rendering.renderer import Renderer


def main() -> None:
    """Bootstrap the simulation and start the pyglet event loop."""
    # Detect screen dimensions; fall back to safe defaults if unavailable.
    try:
        screen = pyglet.canvas.get_display().get_default_screen()
        W, H = screen.width, screen.height
    except Exception:
        W, H = 1400, 700

    # Create the window FIRST — on macOS the OS may adjust dims for the menu
    # bar / dock, so we read back window.width/height as the true client area.
    # resizable=True enables the on_resize handler below.
    window = pyglet.window.Window(
        width=W,
        height=H,
        caption="Sardine Schooling & Sailfish Hunt",
        resizable=True,
    )

    config = SimConfig(
        window_width=window.width,
        window_height=window.height,
        **load_overrides(),   # saved user prefs layered over code defaults
    )
    simulation = Simulation(config)

    # Renderer must be created after the window so the GL context is active.
    renderer = Renderer(config, window)
    renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)
    menu = ConfigMenu(window, simulation)

    # Mutable cell shared between closures to pass update timing to on_draw.
    _update_ms: list[float] = [0.0]
    # Guard: ignore on_resize events that fire before everything is ready.
    _ready: list[bool] = [False]

    @window.event
    def on_draw() -> None:
        window.clear()
        renderer.draw(simulation, update_ms=_update_ms[0])
        menu.draw()   # no-op when menu is not visible

    def update(dt: float) -> None:
        t0 = time.perf_counter()
        simulation.update(dt)
        _update_ms[0] = (time.perf_counter() - t0) * 1000.0

    @window.event
    def on_resize(width: int, height: int) -> None:
        if not _ready[0]:
            return
        # Enforce sensible minimum to avoid degenerate spatial hash / rendering.
        if width < 200 or height < 150:
            return
        simulation.resize(width, height)
        renderer.on_resize(width, height)
        menu.on_resize(width, height)
        # Full renderer resync after every geometry change.  Resize/fullscreen
        # events are rare (not 60 fps), so the O(N) shape rebuild is acceptable.
        # This guarantees that _algae_shape_map, _sardine_shape_map, and
        # _sailfish_shape_map are perfectly consistent with simulation state
        # regardless of any reentrant idle() calls that macOS may fire during
        # the transition (see PROJECT_LOG.md renderer crash-fix notes).
        renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)

    @window.event
    def on_key_press(symbol: int, modifiers: int) -> None:
        # Menu gets first crack at every key when open (modal guard).
        # handle_key() returns True if the key was consumed — skip all other
        # handlers in that case so e.g. ESC closes the menu rather than quitting.
        if menu.handle_key(symbol, modifiers):
            return
        # TAB opens the menu when it is closed (handle_key returned False above).
        if symbol == pyglet.window.key.TAB:
            menu.toggle()
            return
        if symbol == pyglet.window.key.ESCAPE:
            window.close()
        elif symbol == pyglet.window.key.H:
            renderer.toggle_hud()
        elif symbol == pyglet.window.key.F:
            # Toggle fullscreen; pyglet fires on_resize automatically afterwards.
            window.set_fullscreen(not window.fullscreen)
        elif symbol == pyglet.window.key.V:
            import dataclasses
            _SARDINE_MODES = ("white", "rainbow", "cool", "warm", "ocean", "sunset", "mono")
            cur  = simulation.config.sardine_color_mode
            idx  = _SARDINE_MODES.index(cur) if cur in _SARDINE_MODES else 0
            next_mode = _SARDINE_MODES[(idx + 1) % len(_SARDINE_MODES)]
            simulation.apply_config(
                dataclasses.replace(
                    simulation.config,
                    sardine_color_mode=next_mode,
                )
            )
        # ── Dynamic population controls ───────────────────────────────
        # ↑/↓ : add/remove 50 sardines   ←/→ : remove/add 1 sailfish
        elif symbol == pyglet.window.key.UP:
            simulation.add_sardines(50)
            renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)
        elif symbol == pyglet.window.key.DOWN:
            simulation.remove_sardines(50)
            renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)
        elif symbol == pyglet.window.key.RIGHT:
            simulation.add_sailfish(1)
            renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)
        elif symbol == pyglet.window.key.LEFT:
            simulation.remove_sailfish(1)
            renderer.reset(simulation.sardines, simulation.sailfish, simulation.algae)

    # Mark as ready only after all closures are registered, so on_resize
    # won't fire on any initial geometry event before the simulation exists.
    _ready[0] = True

    # Advance simulation at the target frame rate; pyglet redraws via on_draw.
    pyglet.clock.schedule_interval(update, 1.0 / config.target_fps)

    pyglet.app.run()


if __name__ == "__main__":
    main()
