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
    # Reduced from 900 to 300 for ecology balance: 900 sardines deplete 500
    # algae in seconds, leaving no time for reproduction or starvation
    # dynamics to play out.  300 sardines with respawn creates a sustainable
    # food cycle.  Increase with ↑ key once you want a pure-flocking view.
    num_sardines: int = 300

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
    weight_flee: float = 10.0       # must exceed weight_separation (2.0)

    # Cell size for the sailfish-only spatial hash.
    sailfish_cell_size: float = 120.0

    # ── Algae (Phase 8) ───────────────────────────────────────────────
    # Increased from 500 to 800 for higher initial food density so sardines
    # can eat before starvation clocks tick down.
    num_algae: int = 800

    # Algae kinematics — algae drift slowly, driven mostly by alignment.
    # min_speed is always 0.0 (not a config field); algae can stop completely.
    # max_force / max_speed ≈ 0.48 — gentle arc, less snappy than sardines (1.0).
    algae_max_speed: float = 25.0     # px/s; slow drift (~6× slower than sardines)
    algae_max_force: float = 12.0     # px/s²; gentle steering budget

    # Algae flocking weights — alignment-dominant gives the "current drift" feel.
    # Tuned so algae form loose drifting streams rather than tight schools:
    #   - alignment dominant (drives the current)
    #   - cohesion very weak (loose aggregates; reduced from 0.2 to avoid clumping)
    #   - separation slightly stronger than cohesion for healthy sep:coh radius ratio
    #   - noise kept low so drift is smooth, not chaotic
    weight_algae_separation: float = 0.3
    weight_algae_alignment:  float = 1.5
    weight_algae_cohesion:   float = 0.1
    weight_algae_noise:      float = 0.05

    # Algae perception radii (px).
    # r_sep < r_ali == r_coh ≤ algae_cell_size.
    # sep radius raised (12→18) so sep:coh ratio is 1:3 instead of 1:4.6 —
    # prevents dense clumping without scattering.
    # Alignment and cohesion share the same radius (55 px) so every cohesion
    # neighbour also aligns, producing smoother current patches.
    radius_algae_separation: float = 18.0
    radius_algae_alignment:  float = 55.0
    radius_algae_cohesion:   float = 55.0

    # Cell size for the algae-only spatial hash.
    # Kept > both r_ali and r_coh for a clean 3×3-ring neighbour lookup.
    algae_cell_size: float = 60.0

    # Visual size — smaller than sardines for the background-layer feel.
    algae_size: float = 4.0

    # Contact radius used by the ecology pass to detect sardine–algae eating.
    # A sardine whose centre is within this distance of an algae centre eats it.
    # Larger than algae_size (4 px) to register contact reliably at 60 fps
    # even when the sardine passes through the algae in one frame step.
    algae_eat_radius: float = 14.0

    # Steady-state rate at which new algae are spawned into the world.
    # Uses a fractional accumulator so non-integer rates (e.g. 3.7/s) work
    # correctly without bias.  Set to 0.0 to disable respawn entirely
    # (useful for tests / debugging a finite-resource scenario).
    algae_respawn_rate: float = 8.0

    # Soft cap on the total algae population.  Respawn is paused while
    # len(algae) >= algae_max_count so the pool never grows unboundedly
    # when sardines are absent.  Must be >= num_algae for a sensible
    # starting state.
    algae_max_count: int = 1200

    # Number of algae a sardine must consume before it reproduces (splits).
    # food_counter is decremented by this amount on reproduction so any
    # "leftover" eating already in progress carries forward.
    # Lowered from 5 → 3: with 300 sardines and limited algae density,
    # threshold 5 made reproduction extremely rare.  3 algae is achievable
    # within a school pass through a well-stocked area.
    # Set to 0 to disable reproduction entirely (useful for tests/debugging).
    sardine_eat_threshold: int = 3

    # ── Sardine ecology: starvation (Phase 8.5) ──────────────────────
    # How long (seconds) a sardine can go without eating before it dies.
    # Set to 0.0 to disable sardine starvation entirely (useful for tests /
    # tuning algae or reproduction in isolation).
    # Increased from 20 → 30 s: gives sardines a full 30-second window to
    # find algae.  At world size 1400×700 with 800 algae and 300 sardines,
    # an average sardine should cross an algae patch within ~15 seconds.
    sardine_starvation_time: float = 30.0

    # Initial value for sardine.hunger_timer at spawn.  Must be strictly less
    # than sardine_starvation_time so freshly spawned sardines don't die at
    # t = starvation_time − initial_hunger_time.  Also staggers die-off
    # timing when sardines go without algae from startup.
    # Increased from 10 → 15 s to pair with the 30 s starvation time,
    # giving each sardine 15 seconds of initial runway.
    sardine_initial_hunger_time: float = 15.0

    # ── Sailfish ecology (Phase 8.6) ──────────────────────────────────
    # Contact radius for sailfish-sardine eating.  A sailfish whose centre is
    # within this distance of a sardine centre eats it.  One sardine per
    # sailfish per frame (prevents mass-kill when a school is discovered).
    # Widened from 20 → 25 px: at sardine speed ~100 px/s, a 20 px window
    # means a sardine can pass entirely through the kill zone in one frame;
    # 25 px provides a reliable contact zone without making prey too easy.
    sailfish_eat_radius: float = 25.0

    # ── Sailfish ecology: starvation (Phase 8.7) ──────────────────────
    # How long (seconds) a sailfish can go without eating before it dies.
    # Longer than sardines — apex predators need more runway between meals.
    # Set to 0.0 to disable sailfish starvation entirely.
    # Increased from 45 → 90 s: with a sardine school that may evade for
    # extended periods, 45 s was too tight.  90 s gives sailfish a realistic
    # survival window even when sardines are sparse.
    sailfish_starvation_time: float = 90.0

    # Initial value for sailfish.hunger_timer at spawn.  Must be strictly less
    # than sailfish_starvation_time.  Staggers the earliest possible death.
    # Increased from 20 → 40 s to match the longer starvation threshold.
    sailfish_initial_hunger_time: float = 40.0

    # ── Sailfish ecology: reproduction (Phase 8.8) ────────────────────
    # Number of sardines a sailfish must eat before producing one offspring.
    # Set to 0 to disable sailfish reproduction entirely.
    # Default of 8: each sardine eaten increments food_counter by 1; with
    # 8 sailfish hunting actively, at ~1 kill per sailfish every few seconds
    # a single sailfish reaches threshold roughly every 30–60 seconds.
    sailfish_eat_threshold: int = 8

    # ── Overlay / HUD ─────────────────────────────────────────────────
    # Toggle with the H key at runtime (via Renderer.toggle_hud()).
    show_hud: bool = True


# Module-level default instance — import this everywhere.
CONFIG = SimConfig()
