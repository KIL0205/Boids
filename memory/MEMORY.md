# MEMORY.md — Stable Project Memory

Use this file for durable conventions and quick-reference facts.

Authoritative current-state system behavior: [PROJECT_LOG.md](../PROJECT_LOG.md)
Historical milestones and completed phases: [ARCHIVE.md](ARCHIVE.md)
Implementation tactics and patterns: [DEV_NOTES.md](DEV_NOTES.md)

---

## Stable Conventions

- Simulation data flow: rebuild grids -> update agents -> ecology pass -> render.
- Three-grid model is intentional and should be preserved.
- Config source of truth is [config/config.py](../config/config.py) `SimConfig`.
- Runtime persistence is [config/config_io.py](../config/config_io.py) with migration-safe loading and atomic save.
- Config menu should modify live state through `Simulation.apply_config()` only.
- Do not add duplicate logging/docs systems; keep active state in PROJECT_LOG and references in this memory folder.

---

## Sardine Color Architecture (Durable Rule)

- Keep sardine coloring split into:
  - driver: `sardine_color_driver`
  - palette: `sardine_color_palette`
- Driver computes scalar; palette maps scalar.
- Do not reintroduce age-as-palette/theme coupling.

---

## Reliability Rules

- Keep renderer shape-map lifecycle and lazy-rescue logic intact for births/deaths.
- Preserve modal behavior of config menu key handling.
- Preserve `Simulation.apply_config()` cache refresh paths when adding new cached fields.

---

## Validation Commands

- Tests: `python -m unittest tests.tests`
- Benchmarks: `python -m tests.bench`
