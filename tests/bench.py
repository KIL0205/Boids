"""bench.py — Headless performance benchmark for the Boids simulation.

Usage
-----
  python bench.py                  # run all scenarios, print ms/frame summary
  python bench.py --profile        # run heavy scenario under cProfile
  python bench.py --test           # assert conservative timing thresholds (CI guard)
  python bench.py --frames 100     # override default timed-frame count

Each scenario runs WARMUP frames (not timed) to reach thermal/JIT steady state,
then FRAMES timed frames.  Wall-clock time is measured with time.perf_counter.

Scenarios
---------
  light   — 300 sardines, 600 algae, 5 sailfish
  medium  — 800 sardines, 800 algae, 8 sailfish
  heavy   — 1500 sardines, 1200 algae, 10 sailfish
  extreme — 2000 sardines, 1200 algae, 10 sailfish

Thresholds (--test mode, FRAMES=300 wall-clock seconds)
---------------------------------------------------------
  light    < 6 s
  medium   < 12 s
  heavy    < 24 s
  extreme  < 40 s
"""

import argparse
import cProfile
import pstats
import io
import time

from boids.config.config import SimConfig
from sim.simulation import Simulation

# ── Constants ─────────────────────────────────────────────────────────────────
WARMUP = 60
DT     = 1.0 / 60.0

SCENARIOS = [
    ("light",   dict(num_sardines=300,  num_algae=600,  num_sailfish=5)),
    ("medium",  dict(num_sardines=800,  num_algae=800,  num_sailfish=8)),
    ("heavy",   dict(num_sardines=1500, num_algae=1200, num_sailfish=10)),
    ("extreme", dict(num_sardines=2000, num_algae=1200, num_sailfish=10)),
]

# Conservative thresholds in seconds for FRAMES=300 timed frames
THRESHOLDS = {
    "light":   6.0,
    "medium":  12.0,
    "heavy":   24.0,
    "extreme": 40.0,
}


def _make_sim(counts: dict) -> Simulation:
    """Build a headless Simulation from population counts."""
    cfg = SimConfig(
        num_sardines=counts["num_sardines"],
        num_algae=counts["num_algae"],
        num_sailfish=counts["num_sailfish"],
    )
    return Simulation(cfg)


def _run_frames(sim: Simulation, n: int) -> None:
    """Advance *n* frames without timing."""
    for _ in range(n):
        sim.update(DT)


def run_scenario(name: str, counts: dict, frames: int) -> dict:
    """Run one scenario; return a result dict."""
    sim = _make_sim(counts)
    pop_start = (
        len(sim.sardines),
        len(sim.algae),
        len(sim.sailfish),
    )

    # Warmup — not timed
    _run_frames(sim, WARMUP)

    # Timed frames
    t0 = time.perf_counter()
    _run_frames(sim, frames)
    elapsed = time.perf_counter() - t0

    pop_end = (
        len(sim.sardines),
        len(sim.algae),
        len(sim.sailfish),
    )

    ms_per_frame = elapsed / frames * 1000.0
    fps_equiv    = frames / elapsed

    return {
        "name":         name,
        "frames":       frames,
        "elapsed_s":    elapsed,
        "ms_per_frame": ms_per_frame,
        "fps_equiv":    fps_equiv,
        "pop_start":    pop_start,
        "pop_end":      pop_end,
    }


def print_result(r: dict) -> None:
    name = r["name"]
    ms   = r["ms_per_frame"]
    fps  = r["fps_equiv"]
    ps, as_, fs = r["pop_start"]
    pe, ae, fe  = r["pop_end"]
    print(
        f"  {name:<8}  {ms:6.2f} ms/frame  ({fps:6.1f} fps-equiv)"
        f"  sardines {ps:4d}→{pe:4d}  algae {as_:4d}→{ae:4d}"
        f"  sailfish {fs:2d}→{fe:2d}"
    )


def run_profile(frames: int) -> None:
    """Run the heavy scenario under cProfile and print top 20 functions."""
    name, counts = SCENARIOS[2]  # "heavy"
    sim = _make_sim(counts)
    _run_frames(sim, WARMUP)

    pr = cProfile.Profile()
    pr.enable()
    _run_frames(sim, frames)
    pr.disable()

    buf = io.StringIO()
    ps  = pstats.Stats(pr, stream=buf).sort_stats("tottime")
    ps.print_stats(20)
    print(f"\n--- cProfile: '{name}' scenario, {frames} frames ---")
    print(buf.getvalue())


def main() -> None:
    parser = argparse.ArgumentParser(description="Boids headless benchmark")
    parser.add_argument(
        "--frames", type=int, default=300,
        help="Number of timed frames per scenario (default: 300)"
    )
    parser.add_argument(
        "--profile", action="store_true",
        help="Run heavy scenario under cProfile instead of all scenarios"
    )
    parser.add_argument(
        "--test", action="store_true",
        help="Assert conservative timing thresholds (exit 1 on failure)"
    )
    args = parser.parse_args()

    if args.profile:
        run_profile(args.frames)
        return

    print(f"\nBoids benchmark — {args.frames} timed frames per scenario "
          f"(+{WARMUP} warmup), DT={DT:.4f}\n")

    failures = []
    for name, counts in SCENARIOS:
        r = run_scenario(name, counts, args.frames)
        print_result(r)

        if args.test:
            threshold = THRESHOLDS[name]
            if r["elapsed_s"] > threshold:
                failures.append(
                    f"  FAIL {name}: {r['elapsed_s']:.2f}s > {threshold}s threshold"
                )

    if args.test:
        print()
        if failures:
            for msg in failures:
                print(msg)
            raise SystemExit(1)
        else:
            print("  All scenarios within thresholds. PASS")


if __name__ == "__main__":
    main()
