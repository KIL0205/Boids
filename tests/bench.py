"""bench.py — Headless performance benchmark for the Boids simulation.

Usage
-----
  python -m tests.bench                         # run all 8 benchmark scenarios
  python -m tests.bench --scenario baitball_detect  # run one scenario by name
  python -m tests.bench --frames 200            # override timed frame count
  python -m tests.bench --warmup 120            # override warmup frame count
  python -m tests.bench --verbose               # print per-subsystem timing breakdown
  python -m tests.bench --profile               # cProfile the baitball_detect scenario
  python -m tests.bench --profile-scenario heavy  # cProfile a regression scenario
  python -m tests.bench --test                  # assert regression thresholds (CI guard)
  python -m tests.bench --csv results.csv       # export all results to CSV

Each scenario runs WARMUP frames (not timed) so the simulation reaches
ecological steady-state before measurement begins.  Then FRAMES timed frames
are collected individually with time.perf_counter(), giving per-frame
distributions (avg/p95/p99/min/max) rather than just a single total.

Subsystem timing is collected via sim.dbg_ms_* fields (enabled by setting
debug_frame_timing_enabled=True in each scenario's config), providing
separate sardine / sailfish / baitball-detection / algae breakdowns.

Headless limitation
-------------------
Pyglet rendering is unavailable in headless mode.  Scenarios 6 (debug_off)
and 7 (debug_on) therefore measure simulation update cost only; the actual
cost of drawing debug shapes and labels is not captured.  Compare them to
verify that configuring debug flags adds zero simulation overhead.

Scenarios
---------
  baseline        — sardines only, no ecology, no predators
  standard_ecology — full ecology (sardines + algae + sailfish)
  sailfish_stress  — high sailfish count, all FSM states active
  baitball_detect  — dense sardine clusters, baitball detection every frame
  coop_herding     — cooperative orbit/herding/lateralization/spacing/strike
  debug_off        — full debug config, master toggle OFF
  debug_on         — full debug config, master toggle ON
  worst_case       — maximum load: high populations + all features

Regression scenarios (--test mode)
-----------------------------------
  light    < 6 s  total for 300 frames
  medium   < 12 s
  heavy    < 24 s
  extreme  < 40 s
"""

from __future__ import annotations

import argparse
import cProfile
import csv
import dataclasses
import io
import pstats
import sys
import time

import numpy as np

# ── Bootstrap: load config.config without pyglet ─────────────────────────────
# config/__init__.py imports ConfigMenu which depends on pyglet; we bypass it
# by loading config.config directly so the benchmark runs headlessly.
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location("config.config", "config/config.py")
_mod  = _ilu.module_from_spec(_spec)
sys.modules["config.config"] = _mod
_spec.loader.exec_module(_mod)
SimConfig = _mod.SimConfig  # noqa: N816

from sim.simulation import Simulation  # noqa: E402

# ── Global constants ──────────────────────────────────────────────────────────
WARMUP  = 60         # warmup frames before measurement
FRAMES  = 300        # default timed frames per scenario
DT      = 1.0 / 60.0

# Standard world resolution used by benchmark scenarios.  Larger than the
# default 1400×700 to exercise realistic spatial-hash cell counts without
# being as extreme as the override file's 3024×1898.
_W = 1920
_H = 1080


# ─────────────────────────────────────────────────────────────────────────────
# Result container
# ─────────────────────────────────────────────────────────────────────────────

@dataclasses.dataclass
class BenchResult:
    """Collected timing data for one benchmark scenario.

    frame_times_ms holds raw per-frame wall-clock times so callers can
    compute any percentile they need.  subsystem_avgs maps subsystem name
    to average milliseconds per frame (populated when collect_subsystem=True).
    """

    name:              str
    description:       str
    frames:            int
    frame_times_ms:    list   # raw per-frame times (ms), len == frames
    pop_start:         tuple  # (sardines, algae, sailfish) just before timing
    pop_end:           tuple  # same triple after timing run
    subsystem_avgs:    dict   # {subsystem: avg_ms}; empty when not collected
    baitball_count:    int    # sim.debug_baitball_count at end (0 if N/A)
    note:              str    # human-readable limitation note (may be empty)

    # ── Derived statistics ─────────────────────────────────────────────

    @property
    def avg_ms(self) -> float:
        return float(np.mean(self.frame_times_ms))

    @property
    def p95_ms(self) -> float:
        return float(np.percentile(self.frame_times_ms, 95))

    @property
    def p99_ms(self) -> float:
        return float(np.percentile(self.frame_times_ms, 99))

    @property
    def min_ms(self) -> float:
        return float(np.min(self.frame_times_ms))

    @property
    def max_ms(self) -> float:
        return float(np.max(self.frame_times_ms))

    @property
    def fps(self) -> float:
        avg = self.avg_ms
        return 1000.0 / avg if avg > 0 else 0.0

    @property
    def elapsed_s(self) -> float:
        return sum(self.frame_times_ms) / 1000.0


# ─────────────────────────────────────────────────────────────────────────────
# Scenario definitions
# ─────────────────────────────────────────────────────────────────────────────

@dataclasses.dataclass
class Scenario:
    """Definition of one benchmark run.

    config_kwargs is passed directly to SimConfig(**config_kwargs) so any
    SimConfig field can be overridden per scenario.

    collect_subsystem: if True, per-frame dbg_ms_* values are gathered and
    averaged; requires debug_frame_timing_enabled=True in config_kwargs.

    note: optional string printed below the result for headless limitations
    or other context.
    """

    name:             str
    description:      str
    config_kwargs:    dict
    collect_subsystem: bool = False
    note:             str = ""


def _base(**overrides) -> dict:
    """Standard benchmark world (1920×1080) with subsystem timing enabled."""
    return {"window_width": _W, "window_height": _H,
            "debug_frame_timing_enabled": True, **overrides}


# ── Benchmark scenario catalogue ──────────────────────────────────────────────

BENCHMARK_SCENARIOS: list[Scenario] = [

    # ── 1. Baseline movement ───────────────────────────────────────────────────
    # Purpose: isolate the raw sardine flocking + spatial-hash cost.
    # Catches regressions in sardine force computation, Phase 4A/4C batching,
    # or spatial-hash insert/query loops.  No ecology or predators to confound.
    Scenario(
        name="baseline",
        description="Pure sardine flocking: no predators, no algae, no ecology",
        config_kwargs=_base(
            num_sardines=1500,
            num_algae=0,
            algae_max_count=0,             # suppress respawn — no algae in this scenario
            num_sailfish=0,
            sardine_starvation_time=0.0,  # disabled — keep count stable
            sardine_eat_threshold=0,       # disabled — no reproduction
            sailfish_baitball_enabled=False,
        ),
        collect_subsystem=True,
    ),

    # ── 2. Standard ecology ────────────────────────────────────────────────────
    # Purpose: measure typical real-world gameplay overhead with a live
    # three-tier ecosystem.  Catches regressions in the ecology passes
    # (eating, starvation, reproduction, algae respawn) and entity-count drift.
    Scenario(
        name="standard_ecology",
        description="Full ecosystem: sardines + algae + sailfish with reproduction and starvation",
        config_kwargs=_base(
            num_sardines=800,
            num_algae=800,
            num_sailfish=6,
            # All ecology features use defaults (enabled).
        ),
        collect_subsystem=True,
    ),

    # ── 3. Sailfish state-machine stress ──────────────────────────────────────
    # Purpose: measure per-sailfish FSM update cost across all seven states
    # (PATROL, TRACK_SCHOOL, ORBIT_BAITBALL, TARGET_LOCK, BURST_ATTACK,
    # BAITBALL_STRIKE, RECOVER).  High sailfish count drives sustained state
    # transition activity.  Ecology disabled to stabilise population.
    Scenario(
        name="sailfish_stress",
        description="15 sailfish with full FSM: baitball detection, herding, orbit, and strike",
        config_kwargs=_base(
            num_sardines=800,
            num_algae=0,
            algae_max_count=0,
            num_sailfish=15,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_starvation_enabled=False,
            sailfish_reproduction_enabled=False,
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_baitball_strike_enabled=True,
        ),
        collect_subsystem=True,
    ),

    # ── 4. Baitball detection stress ──────────────────────────────────────────
    # Purpose: stress-test the baitball candidate generation and merge pipeline
    # independently of sailfish behavior.  Uses a small world so sardines
    # cluster tightly, maximising candidate count and merge work.
    # baitball_detect_interval=1 disables the stagger so detection runs every
    # frame.  Catches regressions in _detect_baitballs vectorisation and
    # _merge_baitball_candidates.
    Scenario(
        name="baitball_detect",
        description="2000 sardines in a 900×600 world; baitball detection every frame (no stagger)",
        config_kwargs={
            "window_width":  900,
            "window_height": 600,
            "num_sardines":  2000,
            "num_algae":     0,
            "algae_max_count": 0,
            "num_sailfish":  0,
            "sardine_starvation_time": 0.0,
            "sardine_eat_threshold":   0,
            "sailfish_baitball_enabled":   True,
            "baitball_detect_interval":    1,  # every frame — no stagger
            "debug_frame_timing_enabled":  True,
        },
        collect_subsystem=True,
    ),

    # ── 5. Cooperative herding ────────────────────────────────────────────────
    # Purpose: measure the coordinated-orbit cost stack: slot assignment, per-
    # fish orbit angle accumulation, lateralization bias, inter-sailfish
    # spacing force, drift damping, and strike scheduling.  All cooperation
    # features enabled simultaneously to stress the interaction of those paths.
    Scenario(
        name="coop_herding",
        description="12 sailfish with full cooperative orbit: lateralization, spacing, strike scheduling",
        config_kwargs=_base(
            num_sardines=1000,
            num_algae=0,
            algae_max_count=0,
            num_sailfish=12,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_starvation_enabled=False,
            sailfish_reproduction_enabled=False,
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_baitball_strike_enabled=True,
            sailfish_lateralization_enabled=True,
            sailfish_spacing_enabled=True,
            baitball_detect_interval=1,
        ),
        collect_subsystem=True,
    ),

    # ── 6. Debug overlay disabled ──────────────────────────────────────────────
    # Purpose: verify that disabled debug systems add near-zero simulation
    # overhead.  All sub-flags are set to True but the master toggle is False;
    # the overlay's draw() method returns after one branch check.
    # Note: in headless mode the pyglet draw call itself is never made, so
    # only simulation-side cost is measured.  Compare result with debug_on to
    # confirm identical simulation times (the difference, if any, is noise).
    Scenario(
        name="debug_off",
        description="Debug config with master toggle OFF — verifies idle debug overhead is zero",
        config_kwargs=_base(
            num_sardines=1000,
            num_algae=800,
            num_sailfish=6,
            sailfish_baitball_enabled=True,
            debug_baitballs_enabled=False,           # master OFF
            debug_baitball_shading_enabled=True,     # sub-flags on but suppressed
            debug_baitball_labels_enabled=True,
            debug_sailfish_baitball_links_enabled=True,
            debug_sailfish_orbit_slots_enabled=True,
            debug_sailfish_state_labels_enabled=True,
            debug_sailfish_strike_paths_enabled=True,
        ),
        collect_subsystem=True,
        note="Headless: pyglet shape/label drawing not measured. "
             "Compare with debug_on — simulation times should be within noise.",
    ),

    # ── 7. Debug overlay enabled ───────────────────────────────────────────────
    # Purpose: measure simulation update cost with all debug flags active.
    # In headless mode the BaitballDebugOverlay.draw() call is never made, so
    # only simulation-side state preparation (baitball detection, debug slot
    # writes) is captured.  A real rendering overhead measurement requires a
    # display; see PROJECT_LOG for previously recorded overlay profiling data.
    Scenario(
        name="debug_on",
        description="Debug config with all visualization toggles ON",
        config_kwargs=_base(
            num_sardines=1000,
            num_algae=800,
            num_sailfish=6,
            sailfish_baitball_enabled=True,
            debug_baitballs_enabled=True,            # master ON
            debug_baitball_shading_enabled=True,
            debug_baitball_labels_enabled=True,
            debug_sailfish_baitball_links_enabled=True,
            debug_sailfish_orbit_slots_enabled=True,
            debug_sailfish_state_labels_enabled=True,
            debug_sailfish_strike_paths_enabled=True,
        ),
        collect_subsystem=True,
        note="Headless: pyglet shape/label drawing not measured. "
             "Verifies debug_baitballs_enabled adds zero simulation overhead.",
    ),

    # ── 8. Worst case ─────────────────────────────────────────────────────────
    # Purpose: practical upper-bound on frame time.  Exercises every major
    # subsystem simultaneously: sardine batch forces, sailfish FSM, baitball
    # detection (every frame), full ecology, and debug state preparation.
    # Use this scenario to set realistic frame-budget expectations on target
    # hardware and to catch any catastrophic performance regressions.
    Scenario(
        name="worst_case",
        description="Maximum load: 2500 sardines, 1200 algae, 10 sailfish, all features, no stagger",
        config_kwargs=_base(
            num_sardines=2500,
            num_algae=1200,
            num_sailfish=10,
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_baitball_strike_enabled=True,
            baitball_detect_interval=1,
            debug_baitballs_enabled=True,
            debug_baitball_shading_enabled=True,
            debug_baitball_labels_enabled=True,
            debug_sailfish_baitball_links_enabled=True,
            debug_sailfish_orbit_slots_enabled=True,
            debug_sailfish_state_labels_enabled=True,
            debug_sailfish_strike_paths_enabled=True,
        ),
        collect_subsystem=True,
        note="Headless: rendering overhead excluded.",
    ),
]

# ── GPU comparison scenario catalogue ─────────────────────────────────────────
# Run via --scenario gpu_baseline / gpu_worst_case, or via --compare-gpu.
# Each mirrors a CPU scenario but adds sardine_use_gpu=True.
# Requires wgpu-py (pip install wgpu); silently falls back to CPU if unavailable.
GPU_SCENARIOS: list[Scenario] = [

    # ── 9. GPU baseline ────────────────────────────────────────────────────────
    Scenario(
        name="gpu_baseline",
        description="GPU sardine forces: 1500 sardines, no predators (mirrors 'baseline')",
        config_kwargs=_base(
            num_sardines=1500,
            num_algae=0,
            algae_max_count=0,
            num_sailfish=0,
            sardine_starvation_time=0.0,
            sardine_eat_threshold=0,
            sailfish_baitball_enabled=False,
            sardine_use_gpu=True,
        ),
        collect_subsystem=True,
        note="WebGPU backend via wgpu-py (Metal on macOS). Falls back to CPU if unavailable.",
    ),

    # ── 10. GPU worst case ─────────────────────────────────────────────────────
    Scenario(
        name="gpu_worst_case",
        description="GPU sardine forces: 2500 sardines, full ecology (mirrors 'worst_case')",
        config_kwargs=_base(
            num_sardines=2500,
            num_algae=1200,
            num_sailfish=10,
            sailfish_baitball_enabled=True,
            sailfish_herding_enabled=True,
            sailfish_baitball_strike_enabled=True,
            baitball_detect_interval=1,
            sardine_use_gpu=True,
        ),
        collect_subsystem=True,
        note="WebGPU backend via wgpu-py (Metal on macOS). Falls back to CPU if unavailable.",
    ),
]

# Index for fast name lookup
_SCENARIO_BY_NAME: dict[str, Scenario] = {
    s.name: s for s in BENCHMARK_SCENARIOS + GPU_SCENARIOS
}


# ── Regression scenarios — used only by --test ─────────────────────────────────
# These match the pre-existing light/medium/heavy/extreme workloads so
# --test remains a stable CI guard across benchmark refactors.

_REGRESSION: list[tuple[str, dict]] = [
    ("light",   dict(num_sardines=300,  num_algae=600,  num_sailfish=5)),
    ("medium",  dict(num_sardines=800,  num_algae=800,  num_sailfish=8)),
    ("heavy",   dict(num_sardines=1500, num_algae=1200, num_sailfish=10)),
    ("extreme", dict(num_sardines=2000, num_algae=1200, num_sailfish=10)),
]

# Thresholds: total wall-clock seconds for FRAMES=300 timed frames.
_THRESHOLDS_S: dict[str, float] = {
    "light":   6.0,
    "medium":  12.0,
    "heavy":   24.0,
    "extreme": 40.0,
}


# ─────────────────────────────────────────────────────────────────────────────
# Core execution helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_sim(kwargs: dict) -> Simulation:
    """Construct a Simulation from a config_kwargs dict."""
    return Simulation(SimConfig(**kwargs))


def _warmup(sim: Simulation, n: int) -> None:
    """Advance *n* frames without recording timings."""
    for _ in range(n):
        sim.update(DT)


def run_scenario(scenario: Scenario, frames: int, warmup: int) -> BenchResult:
    """Run one scenario and return a BenchResult.

    Parameters
    ----------
    scenario:
        Scenario definition.
    frames:
        Number of timed frames to collect.
    warmup:
        Number of untimed warmup frames before measurement.
    """
    sim = _make_sim(scenario.config_kwargs)

    pop_start = (len(sim.sardines), len(sim.algae), len(sim.sailfish))
    _warmup(sim, warmup)

    # Per-subsystem accumulation buffers (filled only if collect_subsystem).
    ss_sardine:  list[float] = []
    ss_sailfish: list[float] = []
    ss_baitball: list[float] = []
    ss_algae:    list[float] = []

    frame_times: list[float] = []
    collect = scenario.collect_subsystem

    for _ in range(frames):
        t0 = time.perf_counter()
        sim.update(DT)
        frame_times.append((time.perf_counter() - t0) * 1000.0)

        if collect:
            ss_sardine.append(sim.dbg_ms_sardine_forces)
            ss_sailfish.append(sim.dbg_ms_sailfish_update)
            ss_baitball.append(sim.dbg_ms_baitball_detect)
            ss_algae.append(sim.dbg_ms_algae_update)

    pop_end = (len(sim.sardines), len(sim.algae), len(sim.sailfish))

    subsystem_avgs: dict[str, float] = {}
    if collect:
        subsystem_avgs = {
            "sardine_ms":  float(np.mean(ss_sardine)),
            "sailfish_ms": float(np.mean(ss_sailfish)),
            "baitball_ms": float(np.mean(ss_baitball)),
            "algae_ms":    float(np.mean(ss_algae)),
        }

    return BenchResult(
        name=scenario.name,
        description=scenario.description,
        frames=frames,
        frame_times_ms=frame_times,
        pop_start=pop_start,
        pop_end=pop_end,
        subsystem_avgs=subsystem_avgs,
        baitball_count=sim.debug_baitball_count,
        note=scenario.note,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Output helpers
# ─────────────────────────────────────────────────────────────────────────────

def print_result(r: BenchResult, verbose: bool = False) -> None:
    """Print a single result row (+ optional subsystem breakdown)."""
    ps, as_, fs = r.pop_start
    pe, ae, fe  = r.pop_end

    bb_str = f"  bb={r.baitball_count}" if r.baitball_count > 0 else ""

    print(
        f"  {r.name:<20}  {r.avg_ms:6.2f} ms  "
        f"p95 {r.p95_ms:5.2f}  p99 {r.p99_ms:5.2f}  "
        f"({r.fps:5.1f} fps)"
        f"  s {ps:4d}→{pe:4d}  a {as_:4d}→{ae:4d}  sf {fs:2d}→{fe:2d}"
        f"{bb_str}"
    )

    if verbose and r.subsystem_avgs:
        sa = r.subsystem_avgs
        parts = []
        if sa.get("sardine_ms", 0) > 0.01:
            parts.append(f"sardine {sa['sardine_ms']:.2f}")
        if sa.get("sailfish_ms", 0) > 0.01:
            parts.append(f"sailfish {sa['sailfish_ms']:.2f}")
        if sa.get("baitball_ms", 0) > 0.01:
            parts.append(f"baitball {sa['baitball_ms']:.2f}")
        if sa.get("algae_ms", 0) > 0.01:
            parts.append(f"algae {sa['algae_ms']:.2f}")
        if parts:
            print(f"    subsystem (ms):  {',  '.join(parts)}")

    if r.note:
        print(f"    note: {r.note}")


def print_header(frames: int, warmup: int) -> None:
    """Print column headers above the result table."""
    print(
        f"\nBoids benchmark — {frames} timed frames (+{warmup} warmup), "
        f"DT={DT:.4f}, world {_W}×{_H}\n"
    )
    print(
        f"  {'scenario':<20}  {'avg':>6}     {'p95':>5}   {'p99':>5}   "
        f"{'fps':>8}  populations"
    )
    print("  " + "-" * 90)


def export_csv(results: list[BenchResult], path: str) -> None:
    """Write benchmark results to a CSV file for longitudinal comparison."""
    fieldnames = [
        "name", "description", "frames",
        "avg_ms", "p95_ms", "p99_ms", "min_ms", "max_ms", "fps",
        "elapsed_s",
        "sardines_start", "algae_start", "sailfish_start",
        "sardines_end",   "algae_end",   "sailfish_end",
        "baitball_count",
        "sardine_subsys_ms", "sailfish_subsys_ms",
        "baitball_subsys_ms", "algae_subsys_ms",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            sa = r.subsystem_avgs
            ps, as_, fs = r.pop_start
            pe, ae, fe  = r.pop_end
            w.writerow({
                "name":        r.name,
                "description": r.description,
                "frames":      r.frames,
                "avg_ms":      f"{r.avg_ms:.4f}",
                "p95_ms":      f"{r.p95_ms:.4f}",
                "p99_ms":      f"{r.p99_ms:.4f}",
                "min_ms":      f"{r.min_ms:.4f}",
                "max_ms":      f"{r.max_ms:.4f}",
                "fps":         f"{r.fps:.2f}",
                "elapsed_s":   f"{r.elapsed_s:.4f}",
                "sardines_start": ps, "algae_start": as_, "sailfish_start": fs,
                "sardines_end":   pe, "algae_end":   ae,  "sailfish_end":   fe,
                "baitball_count": r.baitball_count,
                "sardine_subsys_ms":  f"{sa.get('sardine_ms',  0):.4f}",
                "sailfish_subsys_ms": f"{sa.get('sailfish_ms', 0):.4f}",
                "baitball_subsys_ms": f"{sa.get('baitball_ms', 0):.4f}",
                "algae_subsys_ms":    f"{sa.get('algae_ms',    0):.4f}",
            })
    print(f"\n  Results exported → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# cProfile support
# ─────────────────────────────────────────────────────────────────────────────

def run_profile(scenario_name: str, frames: int, warmup: int) -> None:
    """Run *scenario_name* under cProfile and print top-25 hot functions.

    Warms up before profiling to avoid one-time initialisation noise.
    """
    if scenario_name in _SCENARIO_BY_NAME:
        sc = _SCENARIO_BY_NAME[scenario_name]
        sim = _make_sim(sc.config_kwargs)
    else:
        # Fall back to a regression scenario by name.
        reg = {n: d for n, d in _REGRESSION}
        if scenario_name not in reg:
            raise SystemExit(f"Unknown scenario '{scenario_name}' for --profile.")
        sim = _make_sim(reg[scenario_name])

    print(f"\nProfiling '{scenario_name}' ({frames} frames after {warmup} warmup)…")
    _warmup(sim, warmup)

    pr = cProfile.Profile()
    pr.enable()
    for _ in range(frames):
        sim.update(DT)
    pr.disable()

    buf = io.StringIO()
    ps  = pstats.Stats(pr, stream=buf).sort_stats("tottime")
    ps.print_stats(25)
    print(f"\n--- cProfile: '{scenario_name}', {frames} frames ---")
    print(buf.getvalue())


# ─────────────────────────────────────────────────────────────────────────────
# Regression test (--test)
# ─────────────────────────────────────────────────────────────────────────────

def run_regression_test(frames: int, warmup: int) -> None:
    """Run the four regression scenarios and assert elapsed-time thresholds.

    These scenarios are intentionally minimal (no subsystem timing overhead)
    to produce stable wall-clock totals suitable for CI assertions.
    """
    print(f"\nRegression test — {frames} frames (+{warmup} warmup)\n")
    print(f"  {'scenario':<10}  {'elapsed':>8}  {'threshold':>10}  result")
    print("  " + "-" * 45)

    failures: list[str] = []

    for name, counts in _REGRESSION:
        sc = Scenario(
            name=name,
            description=name,
            config_kwargs=counts,
            collect_subsystem=False,
        )
        r = run_scenario(sc, frames, warmup)
        threshold = _THRESHOLDS_S[name]
        passed = r.elapsed_s <= threshold
        status = "PASS" if passed else "FAIL"
        print(
            f"  {name:<10}  {r.elapsed_s:7.2f}s  <{threshold:7.1f}s  {status}"
        )
        if not passed:
            failures.append(
                f"  FAIL {name}: {r.elapsed_s:.2f}s > {threshold}s threshold"
            )

    print()
    if failures:
        for msg in failures:
            print(msg)
        raise SystemExit(1)
    print("  All regression scenarios within thresholds. PASS")


# ─────────────────────────────────────────────────────────────────────────────
# GPU vs CPU comparison (--compare-gpu)
# ─────────────────────────────────────────────────────────────────────────────

# CPU/GPU scenario pairs: (cpu_name, gpu_name)
_GPU_COMPARE_PAIRS: list[tuple[str, str]] = [
    ("baseline",   "gpu_baseline"),
    ("worst_case", "gpu_worst_case"),
]


def run_gpu_comparison(frames: int, warmup: int, *, verbose: bool = False) -> None:
    """Run CPU vs WebGPU head-to-head comparison and print a delta table."""
    print(f"\nCPU vs WebGPU comparison — {frames} frames (+{warmup} warmup)\n")
    col = f"  {'scenario':<18}  {'avg ms':>8}  {'sardine ms':>10}  {'note'}"
    print(col)
    print("  " + "-" * 65)

    for cpu_name, gpu_name in _GPU_COMPARE_PAIRS:
        sc_cpu = _SCENARIO_BY_NAME[cpu_name]
        sc_gpu = _SCENARIO_BY_NAME[gpu_name]

        r_cpu = run_scenario(sc_cpu, frames, warmup)
        r_gpu = run_scenario(sc_gpu, frames, warmup)

        delta_avg = r_gpu.avg_ms - r_cpu.avg_ms
        sign = "+" if delta_avg >= 0 else ""
        delta_str = f"{sign}{delta_avg:.2f} ms"

        def sf_ms(r: BenchResult) -> str:
            v = r.subsystem_avgs.get("sardine_ms")
            return f"{v:.2f}" if v is not None else "   —  "

        print(
            f"  {cpu_name:<18}  {r_cpu.avg_ms:>7.2f}   {sf_ms(r_cpu):>10}  CPU"
        )
        print(
            f"  {gpu_name:<18}  {r_gpu.avg_ms:>7.2f}   {sf_ms(r_gpu):>10}  GPU"
        )
        print(
            f"  {'  delta':<18}  {delta_str:>8}"
            + f"  ({r_cpu.avg_ms / r_gpu.avg_ms:.2f}× speedup)"
            if r_gpu.avg_ms > 0 else ""
        )
        print()

        if verbose:
            print("  CPU detail:")
            print_result(r_cpu, verbose=True)
            print("  GPU detail:")
            print_result(r_gpu, verbose=True)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Boids headless benchmark",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--frames", type=int, default=FRAMES,
        help=f"Timed frames per scenario (default: {FRAMES})",
    )
    parser.add_argument(
        "--warmup", type=int, default=WARMUP,
        help=f"Warmup frames before measurement (default: {WARMUP})",
    )
    parser.add_argument(
        "--scenario", metavar="NAME",
        help="Run only the named benchmark scenario (e.g. baitball_detect)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Print per-subsystem timing breakdown below each result",
    )
    parser.add_argument(
        "--profile", action="store_true",
        help="Profile baitball_detect scenario with cProfile (or use --profile-scenario)",
    )
    parser.add_argument(
        "--profile-scenario", metavar="NAME", default="baitball_detect",
        help="Scenario to profile when --profile is set (default: baitball_detect)",
    )
    parser.add_argument(
        "--test", action="store_true",
        help="Assert regression thresholds (light/medium/heavy/extreme); exit 1 on failure",
    )
    parser.add_argument(
        "--csv", metavar="PATH",
        help="Export results to a CSV file for longitudinal comparison",
    )
    parser.add_argument(
        "--compare-gpu", action="store_true",
        help="Run CPU vs WebGPU head-to-head comparison (baseline and worst_case)",
    )
    args = parser.parse_args()

    # ── cProfile mode ──────────────────────────────────────────────────────────
    if args.profile:
        run_profile(args.profile_scenario, args.frames, args.warmup)
        return

    # ── Regression-test mode ───────────────────────────────────────────────────
    if args.test:
        run_regression_test(args.frames, args.warmup)
        return

    # ── CPU vs GPU comparison mode ─────────────────────────────────────────────
    if args.compare_gpu:
        run_gpu_comparison(args.frames, args.warmup, verbose=args.verbose)
        return

    # ── Benchmark mode ─────────────────────────────────────────────────────────
    if args.scenario:
        all_scenarios = BENCHMARK_SCENARIOS + GPU_SCENARIOS
        if args.scenario not in _SCENARIO_BY_NAME:
            names = ", ".join(s.name for s in all_scenarios)
            raise SystemExit(
                f"Unknown scenario '{args.scenario}'.  "
                f"Available: {names}"
            )
        scenarios_to_run = [_SCENARIO_BY_NAME[args.scenario]]
    else:
        scenarios_to_run = BENCHMARK_SCENARIOS

    print_header(args.frames, args.warmup)

    results: list[BenchResult] = []
    for sc in scenarios_to_run:
        r = run_scenario(sc, args.frames, args.warmup)
        results.append(r)
        print_result(r, verbose=args.verbose)

    if args.csv:
        export_csv(results, args.csv)


if __name__ == "__main__":
    main()
