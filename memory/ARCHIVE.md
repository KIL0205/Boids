# ARCHIVE.md — Historical Milestones and Completed Phase Notes

Purpose:
- Preserve completed milestones and historical context that is no longer required in the day-to-day active log.
- Keep current-state authority in [PROJECT_LOG.md](../PROJECT_LOG.md).

Scope:
- chronology of major phases
- legacy architecture decisions and migrations
- completed feature milestones

For low-level implementation tactics and profiling patterns, see [DEV_NOTES.md](DEV_NOTES.md).

---

## Timeline (Chronological)

### Phase 1 — Core Boid Foundation
What changed:
- Introduced base simulation architecture: `SimConfig`, `Boid`, `Sardine`, `SpatialHash`, `Simulation`, `Renderer`.
- Established toroidal world update/integration pipeline.

Why:
- Build a stable baseline for flocking simulation and future species/ecology expansion.

Where:
- [config/config.py](../config/config.py)
- [sim/boid.py](../sim/boid.py)
- [sim/sardine.py](../sim/sardine.py)
- [sim/spatial_hash.py](../sim/spatial_hash.py)
- [sim/simulation.py](../sim/simulation.py)
- [rendering/renderer.py](../rendering/renderer.py)

### Phase 2 — Sailfish Predator + Sardine Flee
What changed:
- Added sailfish predator behavior and sardine flee mechanics.
- Formalized multi-grid pattern (`_grid`, `_sailfish_grid`).

Why:
- Introduce predator-prey dynamics while preserving flocking cohesion.

Where:
- [sim/sailfish.py](../sim/sailfish.py)
- [sim/sardine.py](../sim/sardine.py)
- [sim/simulation.py](../sim/simulation.py)

### Phase 3 — HUD and Runtime Stability Fixes
What changed:
- Added runtime HUD metrics.
- Fixed pyglet-2 compatibility issues and stabilized draw/update overlays.

Why:
- Improve observability and runtime debugging confidence.

Where:
- [rendering/renderer.py](../rendering/renderer.py)
- [main.py](../main.py)

### Phases 4A–4C — Performance Optimization Pass
What changed:
- Moved sardine updates toward batched numpy-assisted paths.
- Added float caching and inline traversal optimizations.
- Added benchmark harness and thresholds.

Why:
- Raise sustainable population limits and reduce frame cost in heavy scenarios.

Where:
- [sim/simulation.py](../sim/simulation.py)
- [sim/spatial_hash.py](../sim/spatial_hash.py)
- [sim/sardine.py](../sim/sardine.py)
- [tests/bench.py](../tests/bench.py)

Note:
- Detailed pass mechanics and profiling lessons now maintained in [DEV_NOTES.md](DEV_NOTES.md).

### Phase 5 — Resize/Fullscreen and Input Remap
What changed:
- Added resize-safe world updates and fullscreen support.
- Refined keyboard controls and HUD toggles.

Why:
- Ensure runtime geometry correctness and predictable controls.

Where:
- [main.py](../main.py)
- [sim/simulation.py](../sim/simulation.py)
- [rendering/renderer.py](../rendering/renderer.py)

### Phase 6 — Runtime Config Menu + Persistence
What changed:
- Added config menu with live apply/edit/save behavior.
- Added robust override load/save with migration and validation.

Why:
- Enable live tuning without code edits and preserve reproducible sessions.

Where:
- [config/config_menu.py](../config/config_menu.py)
- [config/config_io.py](../config/config_io.py)
- [sim/simulation.py](../sim/simulation.py)

### Phase 8 — Ecosystem Expansion
What changed:
- Added algae species, feeding chain, starvation/reproduction loops, ecology counters.
- Added menu + HUD ecology controls and metrics.

Why:
- Shift from single-flock simulation to ecosystem-level dynamics.

Where:
- [sim/algae.py](../sim/algae.py)
- [sim/simulation.py](../sim/simulation.py)
- [config/config.py](../config/config.py)
- [config/config_menu.py](../config/config_menu.py)
- [rendering/renderer.py](../rendering/renderer.py)

### 2026-04-13 — Sardine Color Architecture Refactor
What changed:
- Replaced legacy combined sardine color mode with split driver/palette model.
- Added config-IO migration from legacy fields.

Why:
- Decouple scalar source from palette mapping and support future drivers cleanly.

Where:
- [rendering/renderer.py](../rendering/renderer.py)
- [config/config.py](../config/config.py)
- [config/config_io.py](../config/config_io.py)
- [config/config_menu.py](../config/config_menu.py)

---

## Legacy Notes (Retained for Traceability)

- `use_velocity_coloring` and combined `sardine_color_mode` are historical-only concepts.
  Current logic uses `sardine_color_driver` + `sardine_color_palette`.
- Old color evolution sections (linear/cosine palette tuning) were retained in intent,
  but authoritative behavior is now documented in [PROJECT_LOG.md](../PROJECT_LOG.md).
- Early section numbering style (`Section 24`, `Section 25`, etc.) is deprecated in this file.
  Timeline is now phase/milestone ordered.

---

## Archive Boundary

This file is intentionally historical.
Do not duplicate current-state definitions here; update [PROJECT_LOG.md](../PROJECT_LOG.md) first, then add concise archive entries after milestones are complete.
