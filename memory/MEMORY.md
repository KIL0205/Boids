# Project Memory — Boids Ecosystem Simulation

## Project
Real-time 2D boid ecosystem. `python main.py` to run. `python -m unittest tests.tests` for tests (268). `python -m tests.bench` for benchmarks (4 scenarios).

## Key Locations
- `sim/spatial_hash.py` — SpatialHash: insert/query/query_first/query_position
- `sim/sardine.py` — Sardine(Boid): flee + flocking, all inline cell traversal
- `sim/algae.py` — Algae(Boid): flocking, inline cell traversal
- `sim/sailfish.py` — Sailfish(Boid): COM pursuit
- `sim/simulation.py` — Simulation: grid rebuild, update loop, ecology pass, `_batch_integrate_sardines()`
- `sim/boid.py` — Boid base class: _gx/_gy/_gvx/_gvy cached floats; `_idx` for batch array row
- `bench.py` — headless benchmark, --profile for cProfile, --test for CI thresholds
- `tests/tests.py` — 268 tests, 23 suites

## Optimization History
- **Phase 3C**: Added `_gx/_gy/_gvx/_gvy` Python-float cache on each boid in `insert()`. Eliminated numpy scalar dispatch in all inner loops. ~50% speedup.
- **Phase 3D**: Inlined cell traversal in sardine flocking + flee + algae flocking. Eliminates: (1) `results=[]` allocation, (2) N `list.append` calls, (3) second loop re-computing dx/dy/d_sq. Added `query_first()` for sailfish eating. Cached `_coh_r2/_flee_r2/_coh_span/_flee_span`. ~14–22% additional speedup.
- **Phase 4A**: Added `_s_pos`, `_s_vel`, `_s_acc`, `_s_n` arrays to `Simulation`. At start of each `update()`, populate arrays from `_gx/_gy/_gvx/_gvy` caches and assign `s._idx` per sardine. Capacity 2× growth policy.
- **Phase 4B**: Split per-sardine `agent.update(dt)` into `s._compute_forces()` + `_batch_integrate_sardines(dt)`. Vectorised Euler, speed clamp, boundary wrap. Eliminated `boid.update()` dispatch for sardines; net −5% profiled frame time.
- **Phase 4C (current)**: `_batch_compute_sardine_forces()` replaces per-sardine Python loop for N≥500. Three vectorised passes: (1) sardine–sailfish flee, (2) per-cell pairwise sep/ali/coh via numpy broadcasting, (3) batch noise. Fallback to per-sardine loop for N<500. +2.27× at heavy, +2.89× at extreme.

## Benchmarks (Phase 4C, Apple Silicon Python 3.13)
| Scenario | ms/frame |
|---|---|
| light (300s, 600a, 5sf) | ~3.6 |
| medium (800s, 800a, 8sf) | ~8.4 |
| heavy (1500s, 1200a, 10sf) | ~11.6 |
| extreme (2000s, 1200a, 10sf) | ~14.9 |

## Current Bottleneck (post-4C)
Cell-bucket Python sweep in `_batch_compute_sardine_forces()` — `dict.get` + `list.append`
building `neigh_list` per cell. Phase 4D: pre-build CSR flat sardine-index array during grid
rebuild so neighbour rows can be gathered with a single slice.
- `Simulation._s_pos/vel/acc`: (N,2) float64 arrays, capacity 2× growth
- `Simulation._s_n`: live sardine count (slice bound for all numpy ops)
- `Boid._idx`: row index in batch arrays, set at start of each update() frame
- `_gx/_gy/_gvx/_gvy` are used as source for populate pass (already Python floats from insert())
- `apply_config()` does NOT need to update batch arrays — they're repopulated every frame; `sardine_max_speed` read from `self.config` inside `_batch_integrate_sardines()` automatically

## Test Patterns
- SpatialHash tests use W=1400, H=700, CS=80 (fast path, nc=17, nr=8)
- Tiny world for slow path: W=100, H=100, CS=80 → nc=2, span=1 → nc≤2*span → slow path
- `_boid_at(x, y, vx, vy)` helper creates a bare Boid for grid tests
- Phase 4 tests use `_no_force_cfg(**kwargs)` helper (defaults merge with kwargs)

## Architecture Notes
- Three grids: `_grid` (sardines), `_sailfish_grid` (sailfish), `_algae_grid` (algae)
- Sardines query sailfish_grid for flee, sardine grid for flocking
- Sailfish query sardine grid for COM pursuit
- Ecology pass runs AFTER all agent.update() calls
- `query()` fast path: nc>2*span AND nr>2*span (always true in production)
- `_EMPTY_LIST` sentinel for empty query() results (never mutate it)

## apply_config() Caches to Update
When adding a new radius-based cache to a species, always refresh it in `Simulation.apply_config()` for that species' loop. Current caches maintained there:
- Sardine: `_sep_r2`, `_ali_r2`, `_coh_r2`, `_flee_r2`, `_coh_span`, `_flee_span`, `max_speed`, `max_force`, `min_speed`, `_boundary_wrap`
- Algae: `_sep_r2`, `_ali_r2`, `_coh_r2`, `_coh_span`, `max_speed`, `max_force`, `_boundary_wrap`
- Sailfish: `_config`, `max_speed`, `max_force`, `min_speed`, `_boundary_wrap`
