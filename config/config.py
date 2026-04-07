"""
config/config.py
================
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

    # ── Sardine color mode ────────────────────────────────────────────
    # Selects which color palette is applied to sardines each frame.
    # Valid modes (cycle with ← / → in the VISUAL config menu section):
    #
    #   "white"   — static silvery white (default; no per-frame color work)
    #   "rainbow" — full HSV color wheel keyed to heading + speed + turn rate
    #   "cool"    — blues / teals / purples (hue band constrained)
    #   "warm"    — reds / oranges / yellows (hue band constrained)
    #   "ocean"   — deep aquatic blue-teal-green (slightly dimmer)
    #   "sunset"  — purples / magentas / pinks with orange wrap
    #   "mono"    — grayscale; brightness encodes speed
    #
    # All dynamic modes (everything except "white") still respect the
    # vel_color_* weighting fields below.
    sardine_color_mode: str = "white"

    # Per-channel kinematic weights for all dynamic sardine color modes.
    # Dimensionless multipliers; 0.0 = that channel is silent.
    #
    # Heading weight (0–2):  scales how much heading cycles through the palette.
    # Speed weight (0–2):    hue offset from forward speed (≤¼ wheel at w=1).
    # Turn weight (0–2):     how aggressively turning desaturates the color.
    # Smoothing (0–0.99):    EMA coefficient for per-fish turn-rate estimate.
    vel_color_heading_weight: float = 1.0
    vel_color_speed_weight:   float = 0.5
    vel_color_turn_weight:    float = 1.0
    vel_color_smoothing:      float = 0.85

    # ── Sailfish (Phase 2) ────────────────────────────────────────────
    num_sailfish: int = 8

    # Sailfish kinematics — real sailfish reach ~3× sardine sprint speed;
    # 250 px/s (≈ 1.67× sardine max) keeps pursuits visible and winnable.
    # Force/speed ratio = 0.8 (< sardine's 1.0) → wider, less snappy turns.
    sailfish_max_speed: float = 250.0   # px/s
    sailfish_min_speed: float = 80.0    # always swimming; no stall
    sailfish_max_force: float = 200.0   # steering budget (px/s²)

    # How far a sailfish can see sardines.
    radius_sailfish_awareness: float = 200.0  # px

    # Multiplier on the Reynolds pursuit steer force.
    weight_pursuit: float = 1.5

    sailfish_size: float = 12.0     # 2× sardine; body ≈ 36 px

    # Sailfish color mode — selects a fixed color for all sailfish.
    # Valid modes (cycle with ← / → in the VISUAL config menu section):
    #   "default" — vivid orange-red (high contrast against dark background)
    #   "white"   — silvery white
    #   "red"     — bright red
    #   "blue"    — electric blue
    #   "black"   — near-black (still visible against the dark background)
    sailfish_color_mode: str = "default"

    # ── Sardine flee / predator-awareness ─────────────────────────────
    radius_flee: float = 100.0      # px; sailfish within this trigger escape
    weight_flee: float = 10.0        # must exceed weight_separation (2.0)

    # Cell size for the sailfish-only spatial hash.
    sailfish_cell_size: float = 120.0

    # ── Overlay / HUD ─────────────────────────────────────────────────
    # Toggle with the H key at runtime (via Renderer.toggle_hud()).
    show_hud: bool = True


# Module-level default instance — import this everywhere.
CONFIG = SimConfig()
