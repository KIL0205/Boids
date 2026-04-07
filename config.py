"""
config.py
=========
Central configuration for the sardine/sailfish boid simulation.

All tunable parameters live here as a frozen dataclass so they can be
imported by any module.  Change values here; do NOT scatter magic
numbers throughout the codebase.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimConfig:
    """Immutable simulation configuration.

    Instantiate once in main.py and pass into Simulation / Renderer.
    """

    # ── Window ────────────────────────────────────────────────────────
    window_width: int = 1400
    window_height: int = 700
    target_fps: int = 60

    # ── Population ────────────────────────────────────────────────────
    num_sardines: int = 900

    # ── Sardine kinematics ────────────────────────────────────────────
    sardine_max_speed: float = 150.0    # pixels / second (~1.5 m/s if 1 px ≈ 1 cm)
    sardine_min_speed: float = 50.0     # prevent stall; sardines always swim
    # Rule of thumb: max_force / max_speed ≈ 1.0 gives a natural turning arc.
    # Lower → sluggish; higher → jittery snap-turns.
    sardine_max_force: float = 150.0    # steering budget per tick (px/s²)

    # ── Sardine flocking weights ──────────────────────────────────────
    # Per-frame velocity contribution ≈ weight × max_force × dt (at 60 fps):
    #   sep: 2.0 × 150 / 60 ≈ 5.0 px/s   ali: 1.3 × 150 / 60 ≈ 3.3 px/s
    # Priority order: separation > alignment > cohesion (Reynolds convention).
    weight_separation: float = 2.0      # highest: prevent overlap above all else
    weight_alignment: float  = 1.3      # strong: heading coherence is key to schooling
    weight_cohesion: float   = 1.0      # baseline: gentle pull toward group centre
    # Small random nudge each frame — prevents rigid lock-in and adds organic feel.
    # Keep ≤ 0.05 so it never overwhelms a real force; set to 0.0 to disable.
    weight_noise: float      = 0.04

    # ── Sardine perception radii ──────────────────────────────────────
    # Radii must satisfy: r_separation < r_alignment ≤ r_cohesion ≤ cell_size.
    # At 600 boids and ≈1920×1080 world: avg spacing ≈ 59 px — similar to the
    # original 300-boid 1400×700 tuning.  r_sep=40 → ~1.4 expected sep-neighbours.
    # At smaller screens (e.g. 1280×800): ≈ 41 px avg spacing, r_sep fires more
    # often — school spreads naturally; no instability observed.
    radius_separation: float = 40.0    # personal space (~3 body-lengths at size=6)
    radius_alignment: float  = 75.0    # match headings; broader window → tighter school
    radius_cohesion: float   = 80.0    # group attraction; must equal cell_size or less

    # ── Spatial hash grid ─────────────────────────────────────────────
    # For best performance cell_size >= radius_cohesion, ensuring a single
    # 3×3 ring lookup suffices.  _candidate_cells handles larger radii
    # correctly by scanning more cells, but at a proportional CPU cost.
    cell_size: float = 80.0

    # ── World boundary ────────────────────────────────────────────────
    # "wrap" = toroidal (recommended for open-sea schooling)
    boundary: str = "wrap"

    # ── Visual ────────────────────────────────────────────────────────
    sardine_size: float = 6.0          # half-length of body triangle (px); body ≈ 18 px
    background_color: tuple[int, int, int, int] = (0, 20, 50, 255)   # deep-water blue

    # ── Sailfish (Phase 2) ────────────────────────────────────────────
    num_sailfish: int = 8

    # Sailfish kinematics — real sailfish reach ~3× sardine sprint speed;
    # 250 px/s (≈ 1.67× sardine max) keeps pursuits visible and winnable.
    # Force/speed ratio = 0.8 (< sardine's 1.0) → wider, less snappy turns.
    sailfish_max_speed: float = 250.0   # px/s
    sailfish_min_speed: float = 80.0    # always swimming; no stall
    sailfish_max_force: float = 200.0   # steering budget (px/s²)

    # How far a sailfish can see sardines.
    # 200 px > cell_size=80, so the spatial hash generates a 5×5 candidate
    # ring — handled correctly by _candidate_cells (not limited to 3×3).
    radius_sailfish_awareness: float = 200.0  # px

    # Multiplier on the Reynolds pursuit steer force.
    # 1.0 means the force is already at max_force; increase to make the
    # sailfish feel more "driven", or decrease for a lazier glide.
    weight_pursuit: float = 1.5

    sailfish_size: float = 12.0                                        # 2× sardine; body ≈ 36 px
    sailfish_color: tuple[int, int, int, int] = (220, 80, 40, 255)    # orange-red predator

    # ── Sardine flee / predator-awareness ─────────────────────────────
    # Contribution at 60 fps ≈ weight_flee × max_force / 60 ≈ 10 px/s —
    # exceeds separation (5 px/s) so escape dominates crowding instincts.
    # Cohesion is zeroed while fleeing so sub-groups can split and scatter;
    # it resumes automatically once the sailfish exits radius_flee.
    radius_flee: float = 100.0      # px; sailfish within this trigger escape
    weight_flee: float = 10.0        # must exceed weight_separation (2.0)

    # Cell size for the sailfish-only spatial hash.  For best performance,
    # keep this >= max(radius_flee, radius_sailfish_awareness) so a single
    # 3×3 ring lookup always suffices.  Larger radii work correctly but cost
    # proportionally more per frame.
    sailfish_cell_size: float = 120.0

    # ── Overlay / HUD ─────────────────────────────────────────────────
    # Toggle with the H key at runtime (via Renderer.toggle_hud()).
    show_hud: bool = True


# Module-level default instance — import this everywhere.
CONFIG = SimConfig()
