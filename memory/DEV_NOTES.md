# DEV_NOTES.md — Implementation Patterns and Engineering Notes

Purpose:
- Keep practical engineering detail out of the active project log while preserving useful implementation knowledge.
- Store patterns, pitfalls, and checklist-style notes for maintenance work.

Current-state behavior remains authoritative in [PROJECT_LOG.md](../PROJECT_LOG.md).

---

## Simulation and Grid Patterns

Three-grid contract:
- `_grid`: sardines only (sardine flocking + sailfish prey query)
- `_sailfish_grid`: sailfish only (sardine flee query)
- `_algae_grid`: algae only (algae flocking + sardine food query)

Update sequencing pattern in [sim/simulation.py](../sim/simulation.py):
1. Rebuild all grids.
2. Update agent state.
3. Run ecology passes (feeding, starvation, reproduction, respawn).
4. Expose side-channels consumed by renderer in same frame.

Do not mutate population lists during per-agent force loops.

---

## Sardine Color Driver/Palette Pipeline

Current split:
- driver: `sardine_color_driver` (`velocity` or `age`)
- palette: `sardine_color_palette` (`white`, `rainbow`, `cool`, `warm`, `ocean`, `sunset`, `mono`)

Required pipeline order in [rendering/renderer.py](../rendering/renderer.py):
1. Compute normalized scalar from driver.
2. Apply palette mapping to scalar.

Migration rules live in [config/config_io.py](../config/config_io.py):
- `use_velocity_coloring` -> velocity + rainbow
- `sardine_color_mode` -> split fields

---

## apply_config Cache Checklist

When adding cached per-agent derived values, update [sim/simulation.py](../sim/simulation.py) `apply_config()`.

Current caches refreshed there:
- Sardine: radius squares, span caches, min/max speed/force, boundary flag
- Sailfish: kinematic limits, flocking radius squares, target lock reset paths
- Algae: radius squares, span cache, kinematic limits, boundary flag
- Mouse stimulus: radius/radius_sq/flee multiplier

Rule:
- If a cache depends on config and is read in hot paths, it must be refreshed in apply_config.

---

## Renderer Shape-Map Safety Pattern

Side-channel + shape-map pattern in [rendering/renderer.py](../rendering/renderer.py):
- id-based maps for algae/sardines/sailfish
- birth guard: skip allocation if id already exists
- death cleanup: delete shape, then rebuild visible list alignment
- lazy rescue path: if an entity exists without cached shape, allocate on demand

Use this pattern for any future side-channel entity lifecycle additions.

---

## Performance Notes (Condensed)

High-impact established patterns:
- use cached Python floats in hot neighbor loops
- avoid repeated numpy scalar dispatch in scalar arithmetic paths
- avoid per-frame temporary list/array allocations in inner loops
- prefer batched/vectorized paths only above measured crossover points

Bench command:
- `python -m tests.bench`

Test command:
- `python -m unittest tests.tests`

---

## Documentation Discipline

For documentation updates:
- update [PROJECT_LOG.md](../PROJECT_LOG.md) with current-state changes first
- add completed milestone summary to [ARCHIVE.md](ARCHIVE.md)
- keep this file focused on actionable engineering detail, not full changelog prose
