"""
sim/sailfish.py
===============
``Sailfish`` — a pursuit predator driven by a state machine built
with the ``transitions`` library.

State machine
-------------
PATROL          No sardines visible.  Sailfish flocks with other sailfish
                and drifts.  Transitions to TRACK_SCHOOL when sardines enter
                ``radius_sailfish_awareness``.

TRACK_SCHOOL    Sardines are visible.  Sailfish steers toward the sardine
                school centre-of-mass (COM) and applies the school-pressure
                orbit bias.  Transitions to TARGET_LOCK when a sardine
                enters ``sailfish_target_acquire_radius``.  Returns to
                PATROL after ``sailfish_no_sardines_timeout`` seconds
                without any visible sardine.

ORBIT_BAITBALL  Cooperative pressure state around a detected dense school.
                Sailfish maintain distributed slots around the baitball,
                apply tangential orbit motion, and only attempt deliberate
                strikes after patience/cooldown gates.

TARGET_LOCK     One sardine has been chosen and locked.  Sailfish steers
                toward its predicted future position (predictive
                interception).  Lock is released when the target dies,
                escapes beyond ``sailfish_target_drop_radius``, or the
                lock timer expires.  Transitions to BURST_ATTACK when the
                target is close enough and burst cooldown is ready.

BURST_ATTACK    High-speed sprint toward the locked target using the same
                predictive interception as TARGET_LOCK, but at a
                multiplied max_speed / max_force (managed in update()).
                Transitions to RECOVER when the burst timer expires
                (managed by Simulation ecology pass).  Transitions to
                TRACK_SCHOOL if the target is lost mid-burst.

BAITBALL_STRIKE Fast pass through a detected dense sardine baitball.
                Sailfish chooses a strike line that crosses the cluster
                instead of chasing one sardine.  Ends on timer/exit and
                transitions to RECOVER.

RECOVER         Short rest after a burst/strike during which high-speed
                attack modes are unavailable. Sailfish pursues the school
                COM if sardines are visible; otherwise drifts. Timer managed
                by Simulation; when it expires the machine evaluates the
                next state (TARGET_LOCK if previous target still valid,
                else TRACK_SCHOOL).

Behavioural forces (preserved and extended)
-------------------------------------------
* Predictive interception toward locked individual sardine.
* School COM steer when no individual target is held.
* School-pressure tangential orbit when school density is high.
* Cooperative herding slots around dense baitballs.
* Sailfish-to-sailfish separation / alignment / cohesion.

All tunables read from SimConfig -- no magic numbers.
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from transitions import Machine

from sim.boid import Boid
from config.config import SimConfig

if TYPE_CHECKING:
    from sim.simulation import _BaitballInfo
    from sim.spatial_hash import SpatialHash

# Suppress transitions DEBUG noise; keep WARNING and above.
logging.getLogger("transitions").setLevel(logging.WARNING)

_EPS2: float = 1e-18  # division-by-zero guard in separation force

_DECISION_ISOLATED: str = "isolated_target"
_DECISION_ORBIT: str = "baitball_orbit"
_DECISION_STRIKE: str = "baitball_strike"
_DECISION_COM: str = "track_com"
_DECISION_NONE: str = "no_target"


@dataclass(frozen=True)
class _HuntingDecision:
    """Structured output from query_hunting_target()."""

    kind: str
    target: object | None = None
    baitball: "_BaitballInfo | None" = None


class Sailfish(Boid):
    """Pursuit-predator sailfish boid with a transitions state machine.

    Parameters
    ----------
    x, y:
        Initial position (pixels).
    vx, vy:
        Initial velocity (pixels/second).
    config:
        Shared simulation config.
    sardine_grid:
        The simulation's sardine-only spatial hash.  Rebuilt every frame by
        ``Simulation`` so queries always reflect current sardine positions.
    sailfish_grid:
        The simulation's sailfish-only spatial hash.  Pass ``None`` to
        disable sailfish-to-sailfish flocking (used by unit tests).
    """

    # ------------------------------------------------------------------ #
    # State machine definition                                             #
    # ------------------------------------------------------------------ #

    STATES = [
        "patrol",
        "track_school",
        "orbit_baitball",
        "target_lock",
        "burst_attack",
        "baitball_strike",
        "recover",
    ]

    _NEXT_ID: int = 0

    _TRANSITIONS = [
        # Any non-patrol state can move to TRACK_SCHOOL.
        {
            "trigger": "sm_to_track_school",
            "source":  [
                "patrol",
                "target_lock",
                "burst_attack",
                "baitball_strike",
                "recover",
                "orbit_baitball",
            ],
            "dest":    "track_school",
        },
        # TRACK_SCHOOL / TARGET_LOCK / RECOVER can shift to cooperative orbit.
        {
            "trigger": "sm_to_orbit_baitball",
            "source":  ["track_school", "target_lock", "recover"],
            "dest":    "orbit_baitball",
        },
        # ORBIT_BAITBALL can fall back when no useful baitball exists.
        {
            "trigger": "sm_orbit_to_track_school",
            "source":  "orbit_baitball",
            "dest":    "track_school",
        },
        # ORBIT_BAITBALL can launch an edge-target strike mode.
        {
            "trigger": "sm_orbit_to_target_lock",
            "source":  "orbit_baitball",
            "dest":    "target_lock",
        },
        # TRACK_SCHOOL / RECOVER can fall back to PATROL when no sardines
        # are seen for too long.
        {
            "trigger": "sm_to_patrol",
            "source":  ["track_school", "recover"],
            "dest":    "patrol",
        },
        # TRACK_SCHOOL / RECOVER can acquire an individual target.
        {
            "trigger": "sm_to_target_lock",
            "source":  ["track_school", "recover"],
            "dest":    "target_lock",
        },
        # ORBIT_BAITBALL can branch into a baitball strike.
        {
            "trigger": "sm_to_baitball_strike",
            "source":  ["orbit_baitball"],
            "dest":    "baitball_strike",
        },
        # Baitball strike can be canceled if the cluster disappears.
        {
            "trigger": "sm_baitball_to_track_school",
            "source":  "baitball_strike",
            "dest":    "track_school",
        },
        # Only TARGET_LOCK can trigger a burst.
        {
            "trigger": "sm_to_burst_attack",
            "source":  "target_lock",
            "dest":    "burst_attack",
        },
        # BURST_ATTACK and BAITBALL_STRIKE both exit via RECOVER.
        {
            "trigger": "sm_to_recover",
            "source":  ["burst_attack", "baitball_strike"],
            "dest":    "recover",
        },
    ]

    # ------------------------------------------------------------------ #
    # Constructor                                                          #
    # ------------------------------------------------------------------ #

    def __init__(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        config: SimConfig,
        sardine_grid: "SpatialHash",
        sailfish_grid: "SpatialHash | None" = None,
    ) -> None:
        super().__init__(
            x, y, vx, vy,
            max_speed=config.sailfish_max_speed,
            max_force=config.sailfish_max_force,
            config=config,
            min_speed=config.sailfish_min_speed,
        )
        self._sardine_grid: "SpatialHash" = sardine_grid
        self._sailfish_grid: "SpatialHash | None" = sailfish_grid

        # Stable per-instance id used for deterministic cooperative slotting.
        self._sid: int = Sailfish._NEXT_ID
        Sailfish._NEXT_ID += 1

        # World dimension caches.
        self._W:  float = float(config.window_width)
        self._H:  float = float(config.window_height)
        self._hw: float = self._W * 0.5
        self._hh: float = self._H * 0.5

        # Sailfish-flocking squared-radius caches (refreshed by apply_config).
        self._sf_sep_r2: float = config.radius_sailfish_separation ** 2
        self._sf_ali_r2: float = config.radius_sailfish_alignment  ** 2
        self._sf_coh_r2: float = config.radius_sailfish_cohesion   ** 2

        # ── State machine ─────────────────────────────────────────────
        # Machine attaches .state (str) and sm_to_*() trigger methods to self.
        # ignore_invalid_triggers=True: returns False silently for disallowed
        # transitions instead of raising InvalidTransition.
        Machine(
            model=self,
            states=Sailfish.STATES,
            transitions=Sailfish._TRANSITIONS,
            initial="patrol",
            auto_transitions=False,
            ignore_invalid_triggers=True,
        )

        # ── Individual prey targeting ─────────────────────────────────
        # _target_sardine:    the locked sardine (or None).
        # _target_lock_timer: seconds remaining; decremented by Simulation.
        self._target_sardine: "object | None" = None
        self._target_lock_timer: float = 0.0

        # ── Burst attack timers ───────────────────────────────────────
        # _burst_timer:          seconds remaining in current burst.
        # _burst_cooldown_timer: seconds until next burst is allowed.
        self._burst_timer:          float = 0.0
        self._burst_cooldown_timer: float = 0.0

        # ── Baitball strike timers/context ────────────────────────────
        self._baitball_strike_timer: float = 0.0
        self._baitball_strike_cooldown_timer: float = 0.0
        self._baitball_memory_timer: float = 0.0
        self._baitball: "_BaitballInfo | None" = None
        self._baitball_strike_exit_x: float = 0.0
        self._baitball_strike_exit_y: float = 0.0

        # Runtime context injected by Simulation each frame.
        self._baitballs_frame: list["_BaitballInfo"] = []
        self._sailfish_count: int = 0
        self._group_strike_cooldown_timer: float = 0.0
        self._is_herding: bool = False
        self._orbiting_baitball: bool = False

        # Cooperative baitball pacing state.
        self._orbit_timer: float = 0.0
        self._orbit_patience_goal: float = 0.0
        self._pending_group_strike_trigger: bool = False
        self._pending_group_strike_kind: str = ""

        # ── State machine timing ──────────────────────────────────────
        # _no_sardines_timer: seconds in TRACK_SCHOOL/RECOVER without visible
        #   sardines; triggers PATROL when >= sailfish_no_sardines_timeout.
        # _recovery_timer:    seconds remaining in RECOVER state; decremented
        #   by Simulation; reaching 0 triggers next-state selection.
        # _dt:                dt cached in update() for use in _compute_forces.
        self._no_sardines_timer: float = 0.0
        self._recovery_timer:    float = 0.0
        self._dt:                float = 1.0 / 60.0

        # ── Ecology ───────────────────────────────────────────────────
        # hunger_timer starts at 0.0 ("just eaten") so no spawn path can
        # produce an immediately-starving fish regardless of config value.
        self.food_counter: float = 0.0
        self.hunger_timer: float = 0.0
        self.age:          float = 0.0

        # ── Lateralization (stable per-fish attack side preference) ───
        # attack_side: +1 = prefer right flank of baitball, -1 = left.
        # lateralization_bias: per-fish strength in [0.35, 1.0].
        # Both persist for the lifetime of the fish.
        self.attack_side:       int   = random.choice([-1, 1])
        self.lateralization_bias: float = random.uniform(0.35, 1.0)

        # ── Per-fish orbit variation (anti-conga-line) ─────────────────
        # Each fish accumulates its own orbit angle at its own speed and
        # direction so fish naturally spread out instead of forming a ring
        # that rotates together.
        self._orbit_angle_accum: float = random.uniform(0.0, 2.0 * math.pi)
        _spd_j = config.sailfish_orbit_speed_jitter
        self._orbit_speed_mult: float = max(
            0.10, 1.0 + random.uniform(-_spd_j, _spd_j)
        )
        self._orbit_dir: int = (
            -1 if random.random() < config.sailfish_orbit_ccw_fraction else 1
        )
        _rad_j = config.sailfish_orbit_radius_jitter
        self._orbit_radius_mult: float = max(
            0.60, min(1.50, 1.0 + random.uniform(-_rad_j, _rad_j))
        )

        # ── Proto-collaboration (turn-taking) ──────────────────────────
        # _active_baitball_strikes: count injected by Simulation each frame.
        # _individual_strike_cooldown: per-fish cooldown before next strike.
        # _is_baitball_attacker: True while this fish is executing a baitball
        #   attack (used by Simulation to count concurrent attackers).
        self._active_baitball_strikes:      int   = 0
        self._individual_strike_cooldown:   float = 0.0
        self._is_baitball_attacker:         bool  = False

        # Drift-damping: track previous baitball center to estimate velocity.
        self._prev_baitball_cx: float = -1.0
        self._prev_baitball_cy: float = -1.0

        # ── Debug / visualization cache (written each frame by _apply_herding_pressure)
        # These expose internal per-fish rendering data without requiring the
        # renderer to re-derive it from first principles.
        # debug_orbit_slot_x/y: world-space position of this fish's assigned
        #   orbit slot (the target point the herding force steers toward).
        #   Set to (-1, -1) when not orbiting or no baitball is active.
        self.debug_orbit_slot_x: float = -1.0
        self.debug_orbit_slot_y: float = -1.0

        # Last hunting-target decision category made this frame.
        # One of: isolated_target, baitball_orbit, baitball_strike,
        # track_com, no_target.
        self.debug_last_target_decision: str = _DECISION_NONE

    # ------------------------------------------------------------------ #
    # External context                                                      #
    # ------------------------------------------------------------------ #

    def set_baitball_context(
        self,
        baitballs: list["_BaitballInfo"],
        sailfish_count: int,
        group_strike_cooldown_timer: float,
        active_baitball_strikes: int = 0,
    ) -> None:
        """Inject this frame's baitball snapshots from Simulation."""
        self._baitballs_frame = baitballs
        self._sailfish_count = sailfish_count
        self._group_strike_cooldown_timer = group_strike_cooldown_timer
        self._active_baitball_strikes = active_baitball_strikes

    # ------------------------------------------------------------------ #
    # Speed/force override                                                 #
    # ------------------------------------------------------------------ #

    def update(self, dt: float) -> None:
        """Apply state-specific speed caps and delegate to Boid integration."""
        self._dt = dt
        cfg = self._config
        if self._burst_timer > 0.0 and self.state == "burst_attack":
            self.max_speed = cfg.sailfish_max_speed * cfg.sailfish_burst_speed_mult
            self.max_force = cfg.sailfish_max_force * cfg.sailfish_burst_force_mult
        elif self._baitball_strike_timer > 0.0 and self.state == "baitball_strike":
            self.max_speed = (
                cfg.sailfish_max_speed * cfg.sailfish_baitball_strike_speed_multiplier
            )
            self.max_force = cfg.sailfish_max_force
        else:
            # Restore base values so live config changes propagate.
            self.max_speed = cfg.sailfish_max_speed
            self.max_force = cfg.sailfish_max_force
        super().update(dt)

    # ------------------------------------------------------------------ #
    # Force computation                                                    #
    # ------------------------------------------------------------------ #

    def _compute_forces(self) -> None:
        """State-machine-driven pursuit + cooperative herding + flocking forces."""
        cfg = self._config
        W   = self._W
        H   = self._H
        hw  = self._hw
        hh  = self._hh
        px  = float(self.position[0])
        py  = float(self.position[1])
        dt  = self._dt

        self._is_herding = False
        self.debug_orbit_slot_x = -1.0
        self.debug_orbit_slot_y = -1.0
        self.debug_last_target_decision = _DECISION_NONE
        baitball = self._select_active_baitball(dt)

        # ── 1. Validate individual target ─────────────────────────────
        tgt = self._target_sardine
        if tgt is not None:
            if not tgt.is_alive or self._target_lock_timer <= 0.0:
                tgt = None
                self._target_sardine = None
            else:
                dx = tgt._gx - px
                dy = tgt._gy - py
                if dx >  hw: dx -= W
                elif dx < -hw: dx += W
                if dy >  hh: dy -= H
                elif dy < -hh: dy += H
                drop_r = cfg.sailfish_target_drop_radius
                if dx * dx + dy * dy > drop_r * drop_r:
                    tgt = None
                    self._target_sardine = None

        # ── 2. Query sardines ──────────────────────────────────────────
        sardines = self._sardine_grid.query(self, cfg.radius_sailfish_awareness)
        has_sardines = bool(sardines)

        state = self.state

        if state == "patrol":
            if has_sardines:
                self._no_sardines_timer = 0.0
                self.sm_to_track_school()
                state = self.state

        if state == "track_school":
            if not has_sardines:
                self._no_sardines_timer += dt
            else:
                self._no_sardines_timer = 0.0
                decision = self.query_hunting_target(sardines, baitball)
                self.debug_last_target_decision = decision.kind

                if decision.kind == _DECISION_ORBIT and decision.baitball is not None:
                    self.sm_to_orbit_baitball()
                    self._arm_orbit_patience()
                    state = self.state
                    baitball = decision.baitball
                elif decision.kind == _DECISION_ISOLATED and decision.target is not None:
                    self._target_sardine = decision.target
                    self._target_lock_timer = cfg.sailfish_target_lock_time
                    tgt = decision.target
                    self.sm_to_target_lock()
                    state = self.state
                else:
                    com_x, com_y = self._local_school_com(sardines, px, py)
                    self._steer_apply_xy(com_x, com_y, cfg.weight_pursuit)
                    if len(sardines) >= cfg.sailfish_school_pressure_density_threshold:
                        self._apply_school_pressure(com_x, com_y)

                # Herding pressure supplements pursuit when a baitball exists,
                # including frames where this fish is still in TRACK_SCHOOL.
                self._apply_herding_pressure(baitball, px, py)

        if state == "orbit_baitball":
            if baitball is None or not has_sardines:
                self._is_baitball_attacker = False
                self.sm_orbit_to_track_school()
                state = self.state
            else:
                self._orbiting_baitball = True
                self._is_herding = True
                self._no_sardines_timer = 0.0
                self._orbit_timer += dt

                com_x, com_y = self._local_school_com(sardines, px, py)
                # Keep loose school COM bias while orbiting to avoid drift.
                self._steer_apply_xy(com_x, com_y, cfg.weight_pursuit * 0.35)
                self._apply_herding_pressure(baitball, px, py)

                decision = self.query_hunting_target(sardines, baitball)
                self.debug_last_target_decision = decision.kind
                if decision.kind == _DECISION_STRIKE and baitball is not None:
                    self._begin_baitball_strike(baitball, px, py)
                    self.sm_to_baitball_strike()
                    self._mark_group_strike("center")
                    state = self.state
                elif decision.kind == _DECISION_ISOLATED and decision.target is not None:
                    if (
                        self._group_strike_cooldown_timer <= 0.0
                        and self._baitball_strike_cooldown_timer <= 0.0
                        and self._individual_strike_cooldown <= 0.0
                        and self._active_baitball_strikes < cfg.sailfish_max_simultaneous_baitball_strikes
                    ):
                        self._target_sardine = decision.target
                        self._target_lock_timer = cfg.sailfish_target_lock_time
                        self.sm_orbit_to_target_lock()
                        self._mark_group_strike("edge")
                        state = self.state

        if state in ("target_lock", "burst_attack"):
            if (
                state == "target_lock"
                and baitball is not None
                and has_sardines
            ):
                decision = self.query_hunting_target(sardines, baitball)
                self.debug_last_target_decision = decision.kind
                if decision.kind == _DECISION_ORBIT:
                    self._target_sardine = None
                    self._target_lock_timer = 0.0
                    self.sm_to_orbit_baitball()
                    self._arm_orbit_patience()
                    state = self.state
                    tgt = None

        if state in ("target_lock", "burst_attack"):
            if tgt is None:
                if state == "burst_attack":
                    self._burst_timer = 0.0
                    self._burst_cooldown_timer = cfg.sailfish_burst_cooldown
                    self._recovery_timer = cfg.sailfish_recovery_duration
                    self.sm_to_recover()
                    state = self.state
                else:
                    self.sm_to_track_school()
                    state = self.state
            else:
                dx = tgt._gx - px
                dy = tgt._gy - py
                if dx > hw:
                    dx -= W
                elif dx < -hw:
                    dx += W
                if dy > hh:
                    dy -= H
                elif dy < -hh:
                    dy += H
                dist = math.sqrt(dx * dx + dy * dy)

                if (
                    state == "target_lock"
                    and dist < cfg.sailfish_burst_trigger_radius
                    and self._burst_timer <= 0.0
                    and self._burst_cooldown_timer <= 0.0
                ):
                    self._burst_timer = cfg.sailfish_burst_duration
                    self.sm_to_burst_attack()
                    state = self.state

                pred_t = min(
                    dist / max(self.max_speed, 1.0),
                    cfg.sailfish_prediction_time,
                )
                if dist > 1.0:
                    pdx = dx + tgt._gvx * pred_t
                    pdy = dy + tgt._gvy * pred_t
                else:
                    pdx, pdy = dx, dy
                self._steer_apply_xy(pdx, pdy, cfg.weight_target_pursuit)

        if state == "baitball_strike":
            if (
                baitball is None
                and self._baitball_strike_timer > cfg.sailfish_baitball_strike_duration_seconds * 0.75
            ):
                self._baitball_strike_timer = 0.0
                self.sm_baitball_to_track_school()
                state = self.state
            else:
                dx = self._baitball_strike_exit_x - px
                dy = self._baitball_strike_exit_y - py
                if dx > hw:
                    dx -= W
                elif dx < -hw:
                    dx += W
                if dy > hh:
                    dy -= H
                elif dy < -hh:
                    dy += H
                self._steer_apply_xy(
                    dx,
                    dy,
                    max(cfg.weight_target_pursuit, cfg.weight_pursuit) * 1.25,
                )
                if (
                    dx * dx + dy * dy
                    <= cfg.sailfish_baitball_strike_exit_distance
                    * cfg.sailfish_baitball_strike_exit_distance
                ):
                    self._baitball_strike_timer = 0.0
                    self._baitball_strike_cooldown_timer = (
                        cfg.sailfish_baitball_strike_cooldown_seconds
                    )
                    self._recovery_timer = cfg.sailfish_recovery_duration
                    self.sm_to_recover()
                    state = self.state

        if state == "recover":
            if has_sardines:
                self._no_sardines_timer = 0.0
                if self._should_enter_orbit_baitball(baitball):
                    self.sm_to_orbit_baitball()
                    self._arm_orbit_patience()
                    state = self.state
                if state == "orbit_baitball":
                    # Fall through to orbit handling in the next frame.
                    pass
                n = len(sardines)
                cx = cy = 0.0
                for s in sardines:
                    dx = s._gx - px
                    dy = s._gy - py
                    if dx > hw:
                        dx -= W
                    elif dx < -hw:
                        dx += W
                    if dy > hh:
                        dy -= H
                    elif dy < -hh:
                        dy += H
                    cx += dx
                    cy += dy
                com_x = cx / n
                com_y = cy / n
                self._steer_apply_xy(com_x, com_y, cfg.weight_pursuit)
                self._apply_herding_pressure(baitball, px, py)
            else:
                self._no_sardines_timer += dt

        # ── Sailfish flocking: separation / alignment / cohesion ──────
        if self._sailfish_grid is None:
            return
        sf_grid = self._sailfish_grid
        sep_r2 = self._sf_sep_r2
        ali_r2 = self._sf_ali_r2
        w_sep = cfg.weight_sailfish_separation
        w_ali = cfg.weight_sailfish_alignment
        w_coh = cfg.weight_sailfish_cohesion

        sf_nb = sf_grid.query(self, cfg.radius_sailfish_cohesion)
        if not sf_nb:
            return

        sf_sx = sf_sy = 0.0
        sf_alx = sf_aly = 0.0
        sf_cx = sf_cy = 0.0
        sf_sep_n = sf_ali_n = sf_coh_n = 0

        for other in sf_nb:
            dx = px - other._gx
            dy = py - other._gy
            if dx > hw:
                dx -= W
            elif dx < -hw:
                dx += W
            if dy > hh:
                dy -= H
            elif dy < -hh:
                dy += H
            d_sq = dx * dx + dy * dy
            if d_sq <= _EPS2:
                continue
            if w_sep != 0.0 and d_sq < sep_r2:
                inv_d = 1.0 / math.sqrt(d_sq)
                sf_sx += dx * inv_d
                sf_sy += dy * inv_d
                sf_sep_n += 1
            if w_ali != 0.0 and d_sq < ali_r2:
                sf_alx += other._gvx
                sf_aly += other._gvy
                sf_ali_n += 1
            if w_coh != 0.0:
                sf_cx -= dx
                sf_cy -= dy
                sf_coh_n += 1

        if sf_sep_n > 0:
            self._steer_apply_xy(sf_sx, sf_sy, w_sep)
        if sf_ali_n > 0:
            n_inv = 1.0 / sf_ali_n
            self._steer_apply_xy(sf_alx * n_inv, sf_aly * n_inv, w_ali)
        if sf_coh_n > 0:
            n_inv = 1.0 / sf_coh_n
            self._steer_apply_xy(sf_cx * n_inv, sf_cy * n_inv, w_coh)

    # ------------------------------------------------------------------ #
    # Private helpers                                                      #
    # ------------------------------------------------------------------ #

    def _local_school_com(
        self,
        sardines: list,
        px: float,
        py: float,
    ) -> tuple[float, float]:
        """Return local sardine COM as a toroidal-relative vector from self."""
        cx = cy = 0.0
        for s in sardines:
            dx = s._gx - px
            dy = s._gy - py
            if dx > self._hw:
                dx -= self._W
            elif dx < -self._hw:
                dx += self._W
            if dy > self._hh:
                dy -= self._H
            elif dy < -self._hh:
                dy += self._H
            cx += dx
            cy += dy
        inv_n = 1.0 / max(1, len(sardines))
        return cx * inv_n, cy * inv_n

    def _hunger_aggression(self) -> float:
        """Return hunger-driven aggression in [0, 1]."""
        starve_t = max(1e-6, float(self._config.sailfish_starvation_time))
        return max(0.0, min(1.0, float(self.hunger_timer) / starve_t))

    def query_hunting_target(
        self,
        sardines: list,
        baitball: "_BaitballInfo | None",
    ) -> _HuntingDecision:
        """Score local options and return one structured hunting decision.

        This function is intentionally separate from SpatialHash.query_first(),
        which remains the ecology eating helper.  Here we evaluate whether this
        fish should prefer isolated individual pursuit, cooperative baitball
        pressure/strike, COM tracking, or no target.
        """
        if not sardines:
            return _HuntingDecision(_DECISION_NONE)

        cfg = self._config
        px = float(self.position[0])
        py = float(self.position[1])
        com_x, com_y = self._local_school_com(sardines, px, py)
        hunger = self._hunger_aggression()

        acq_r = cfg.sailfish_target_acquire_radius
        acq_r2 = acq_r * acq_r
        candidates: list[tuple[float, float, float, object]] = []
        for s in sardines:
            dx = s._gx - px
            dy = s._gy - py
            if dx > self._hw:
                dx -= self._W
            elif dx < -self._hw:
                dx += self._W
            if dy > self._hh:
                dy -= self._H
            elif dy < -self._hh:
                dy += self._H
            d2 = dx * dx + dy * dy
            if d2 <= acq_r2:
                candidates.append((dx, dy, d2, s))

        best_s = None
        best_individual_score = -1e18
        if candidates:
            shared_counts: dict[int, int] = {}
            if self._sailfish_grid is not None:
                sf_nb = self._sailfish_grid.query(self, cfg.sailfish_target_acquire_radius)
                for other in sf_nb:
                    ot = other._target_sardine
                    if ot is None:
                        continue
                    k = id(ot)
                    shared_counts[k] = shared_counts.get(k, 0) + 1
            for dx, dy, d2, s in candidates:
                score = self._score_individual_candidate(
                    sardine=s,
                    dx=dx,
                    dy=dy,
                    d2=d2,
                    com_x=com_x,
                    com_y=com_y,
                    baitball=baitball,
                    hunger=hunger,
                    shared_count=shared_counts.get(id(s), 0),
                )
                if score > best_individual_score:
                    best_individual_score = score
                    best_s = s

        baitball_score = -1e18
        can_orbit = self._should_enter_orbit_baitball(baitball)
        if can_orbit and baitball is not None:
            baitball_score = self._score_baitball_choice(baitball, hunger)

        # ORBIT_BAITBALL is chosen when cooperative pressure clearly dominates.
        if (
            can_orbit
            and baitball is not None
            and baitball_score >= best_individual_score + cfg.sailfish_target_decision_margin
        ):
            if (
                self.state == "orbit_baitball"
                and self._orbit_timer >= self._orbit_patience_goal
                and self._should_start_baitball_strike(baitball)
            ):
                return _HuntingDecision(_DECISION_STRIKE, baitball=baitball)
            return _HuntingDecision(_DECISION_ORBIT, baitball=baitball)

        # Individual lock is allowed unless cooperative suppression is active
        # and this target is not in the baitball edge band.
        if best_s is not None:
            if (
                baitball is not None
                and cfg.sailfish_target_lock_suppressed_near_baitball
                and self._should_enter_orbit_baitball(baitball)
                and not self._is_target_on_baitball_edge(best_s, baitball)
            ):
                return _HuntingDecision(_DECISION_ORBIT, baitball=baitball)
            return _HuntingDecision(_DECISION_ISOLATED, target=best_s, baitball=baitball)

        # If sardines are visible but no acquirable candidate exists, continue
        # tracking school COM (or orbit if a strong baitball exists).
        if can_orbit and baitball is not None:
            return _HuntingDecision(_DECISION_ORBIT, baitball=baitball)
        return _HuntingDecision(_DECISION_COM, baitball=baitball)

    def _score_individual_candidate(
        self,
        sardine: object,
        dx: float,
        dy: float,
        d2: float,
        com_x: float,
        com_y: float,
        baitball: "_BaitballInfo | None",
        hunger: float,
        shared_count: int,
    ) -> float:
        """Score one sardine target (higher is better)."""
        cfg = self._config
        acq_r2 = max(1e-6, cfg.sailfish_target_acquire_radius ** 2)
        awa_r = max(1e-6, cfg.radius_sailfish_awareness)
        max_spd2 = max(1e-6, cfg.sardine_max_speed * cfg.sardine_max_speed)

        dist_norm = 1.0 - min(d2 / acq_r2, 1.0)
        iso_dist = math.sqrt((dx - com_x) ** 2 + (dy - com_y) ** 2)
        iso_norm = min(1.0, iso_dist / awa_r)
        spd2 = sardine._gvx * sardine._gvx + sardine._gvy * sardine._gvy
        vuln_speed = 1.0 - min(spd2 / max_spd2, 1.0)

        edge_norm = 0.0
        if baitball is not None and baitball.radius > 1e-6:
            bdx = float(sardine._gx) - baitball.cx
            bdy = float(sardine._gy) - baitball.cy
            if bdx > self._hw:
                bdx -= self._W
            elif bdx < -self._hw:
                bdx += self._W
            if bdy > self._hh:
                bdy -= self._H
            elif bdy < -self._hh:
                bdy += self._H
            edge_norm = min(1.0, math.sqrt(bdx * bdx + bdy * bdy) / baitball.radius)

        return (
            cfg.sailfish_target_dist_weight * dist_norm
            + cfg.sailfish_target_isolation_weight * iso_norm
            + cfg.sailfish_target_speed_weight * vuln_speed
            + cfg.sailfish_target_edge_weight * edge_norm
            + cfg.sailfish_target_hunger_weight * hunger
            - cfg.sailfish_target_shared_penalty_weight * shared_count
        )

    def _score_baitball_choice(self, baitball: "_BaitballInfo", hunger: float) -> float:
        """Score whether cooperative baitball pressure is preferable."""
        cfg = self._config
        px = float(self.position[0])
        py = float(self.position[1])
        dx = baitball.cx - px
        dy = baitball.cy - py
        if dx > self._hw:
            dx -= self._W
        elif dx < -self._hw:
            dx += self._W
        if dy > self._hh:
            dy -= self._H
        elif dy < -self._hh:
            dy += self._H

        edge_dist = max(0.0, math.sqrt(dx * dx + dy * dy) - baitball.radius)
        edge_near = 1.0 - min(1.0, edge_dist / max(1.0, cfg.radius_sailfish_awareness))
        dens_rel = max(0.0, baitball.density - cfg.sailfish_baitball_density_threshold)
        dens_norm = min(1.0, dens_rel / max(0.05, cfg.sailfish_baitball_density_threshold))
        radius_norm = min(1.0, baitball.radius / max(1.0, cfg.sailfish_target_drop_radius))

        available = max(0.0, float(self._sailfish_count - self._active_baitball_strikes))
        coop_norm = min(1.0, available / max(1.0, float(cfg.sailfish_herding_min_sailfish)))
        if self._group_strike_cooldown_timer > 0.0:
            coop_norm *= 0.5

        return (
            cfg.sailfish_baitball_score_edge_weight * edge_near
            + cfg.sailfish_baitball_score_density_weight * dens_norm
            + cfg.sailfish_baitball_score_radius_weight * radius_norm
            + cfg.sailfish_baitball_score_coop_weight * coop_norm
            + cfg.sailfish_target_hunger_weight * (hunger * 0.5)
        )

    def _select_active_baitball(self, dt: float) -> "_BaitballInfo | None":
        """Choose nearest baitball this frame with short memory fallback.

        Baitballs are ranked by *edge* distance (center_dist - baitball.radius)
        rather than center distance.  This correctly handles the case where a
        large baitball (high radius) is actually closer at its perimeter than a
        smaller one despite having a more distant center.
        """
        if not self._config.sailfish_baitball_enabled:
            self._baitball = None
            self._baitball_memory_timer = 0.0
            return None

        px = float(self.position[0])
        py = float(self.position[1])
        best = None
        best_edge_dist = float("inf")
        for b in self._baitballs_frame:
            dx = b.cx - px
            dy = b.cy - py
            if dx > self._hw:
                dx -= self._W
            elif dx < -self._hw:
                dx += self._W
            if dy > self._hh:
                dy -= self._H
            elif dy < -self._hh:
                dy += self._H
            # Edge distance: how far this sailfish is from the baitball perimeter.
            # Negative when the sailfish is inside the baitball (treated as 0).
            edge_dist = math.sqrt(dx * dx + dy * dy) - b.radius
            if edge_dist < best_edge_dist:
                best_edge_dist = edge_dist
                best = b

        if best is not None:
            self._baitball = best
            self._baitball_memory_timer = self._config.sailfish_baitball_memory_seconds
            return best

        if self._baitball_memory_timer > 0.0 and self._baitball is not None:
            self._baitball_memory_timer = max(0.0, self._baitball_memory_timer - dt)
            return self._baitball

        self._baitball = None
        self._baitball_memory_timer = 0.0
        return None

    def _should_take_individual_target(self, baitball: "_BaitballInfo | None") -> bool:
        """Suppress eager target-lock while cooperative baitball herding is active.

        Returns True when individual target acquisition is permitted, False when
        the cooperative orbit sequence should handle the attack instead.

        When a dense baitball is active and herding conditions are met in
        TRACK_SCHOOL, ``_should_enter_orbit_baitball`` will already redirect the
        fish to ORBIT_BAITBALL on the same frame — so this method's suppression
        path is only reached in the rare case where that redirect was skipped
        (e.g. orbit conditions check failed mid-transition).  Explicitly return
        False here rather than testing ``_orbit_timer >= _orbit_patience_goal``
        (which is always False in TRACK_SCHOOL because the timer is only
        incremented inside the ORBIT_BAITBALL block).
        """
        cfg = self._config
        if not cfg.sailfish_herding_enabled or baitball is None:
            return True
        if self._sailfish_count < cfg.sailfish_herding_min_sailfish:
            return True
        if baitball.density < cfg.sailfish_baitball_density_threshold:
            return True
        if not cfg.sailfish_target_lock_suppressed_near_baitball:
            return True
        # All cooperative-orbit conditions are met: suppress individual lock so
        # the baitball orbit sequence handles the attack.  The old code here
        # tested _orbit_timer >= _orbit_patience_goal, which was always False in
        # TRACK_SCHOOL (timer not incremented there), making this effectively
        # the same as returning False.  Made explicit for clarity.
        return False

    def _should_enter_orbit_baitball(self, baitball: "_BaitballInfo | None") -> bool:
        """Return True when cooperative orbit should be preferred."""
        cfg = self._config
        if baitball is None:
            return False
        if not cfg.sailfish_herding_enabled:
            return False
        if self._sailfish_count < cfg.sailfish_herding_min_sailfish:
            return False
        return baitball.density >= cfg.sailfish_baitball_density_threshold

    def _arm_orbit_patience(self) -> None:
        """Reset orbit patience window with deterministic per-sailfish jitter."""
        cfg = self._config
        self._orbit_timer = 0.0
        # Deterministic pseudo-random jitter in [-1, 1] by stable id.
        unit = ((self._sid * 2654435761) & 0xFFFF) / 65535.0
        signed = unit * 2.0 - 1.0
        jitter = signed * cfg.sailfish_baitball_patience_jitter_seconds
        base = max(cfg.sailfish_baitball_patience_seconds, cfg.sailfish_baitball_min_orbit_seconds)
        self._orbit_patience_goal = max(0.0, base + jitter)

    def _should_start_baitball_strike(self, baitball: "_BaitballInfo") -> bool:
        """Probabilistic gate for deliberate center pass-through strikes."""
        cfg = self._config
        if not cfg.sailfish_baitball_strike_enabled:
            return False
        if baitball.density < cfg.sailfish_baitball_strike_density_threshold:
            return False
        if self._baitball_strike_cooldown_timer > 0.0:
            return False
        if self._baitball_strike_timer > 0.0:
            return False
        if self._group_strike_cooldown_timer > 0.0:
            return False
        if self._orbit_timer < self._orbit_patience_goal:
            return False
        # Proto-collaboration gates.
        if self._individual_strike_cooldown > 0.0:
            return False
        if self._active_baitball_strikes >= cfg.sailfish_max_simultaneous_baitball_strikes:
            return False

        # Convert a per-second chance into a frame-probability.
        chance = max(0.0, min(1.0, cfg.sailfish_baitball_center_strike_chance))
        frame_p = 1.0 - ((1.0 - chance) ** max(self._dt, 0.0))
        return random.random() < frame_p

    def _mark_group_strike(self, kind: str) -> None:
        """Notify Simulation that a group-level strike cooldown should be armed."""
        self._pending_group_strike_trigger = True
        self._pending_group_strike_kind = kind
        # Start per-fish cooldown so this sailfish cannot immediately strike again.
        self._individual_strike_cooldown = (
            self._config.sailfish_individual_strike_cooldown_seconds
        )
        # Flag this fish as an active baitball attacker (read by Simulation).
        self._is_baitball_attacker = True

    def _is_target_on_baitball_edge(
        self,
        sardine: object,
        baitball: "_BaitballInfo",
    ) -> bool:
        """Edge-target filter: reject targets deep inside the baitball core."""
        cfg = self._config
        dx = float(sardine._gx) - baitball.cx
        dy = float(sardine._gy) - baitball.cy
        if dx > self._hw:
            dx -= self._W
        elif dx < -self._hw:
            dx += self._W
        if dy > self._hh:
            dy -= self._H
        elif dy < -self._hh:
            dy += self._H
        d = math.sqrt(dx * dx + dy * dy)
        edge_floor = max(0.0, baitball.radius - cfg.sailfish_baitball_edge_band_width)
        return d >= edge_floor

    def _select_edge_target(
        self,
        sardines: list,
        baitball: "_BaitballInfo",
    ) -> "object | None":
        """Pick a sardine near the baitball perimeter for deliberate edge strikes.

        Biases selection toward the sailfish's preferred attack side when
        lateralization is enabled, so individual fish consistently approach
        from a stable flank rather than randomly.

        Vectorised with numpy to avoid per-sardine math.sqrt calls.
        """
        if not sardines:
            return None

        cfg = self._config
        edge_floor = max(0.0, baitball.radius - cfg.sailfish_baitball_edge_band_width)
        lat_enabled  = cfg.sailfish_lateralization_enabled
        lat_strength = cfg.sailfish_lateralization_strength * self.lateralization_bias

        # Compute a lateral axis (perpendicular to fish→baitball vector) once.
        lat_x = lat_y = 0.0
        if lat_enabled and lat_strength > 0.0:
            fbx = baitball.cx - float(self.position[0])
            fby = baitball.cy - float(self.position[1])
            if fbx > self._hw: fbx -= self._W
            elif fbx < -self._hw: fbx += self._W
            if fby > self._hh: fby -= self._H
            elif fby < -self._hh: fby += self._H
            fb_len = math.sqrt(fbx * fbx + fby * fby)
            if fb_len > 1.0:
                inv_fb = 1.0 / fb_len
                lat_x = -fby * inv_fb   # perpendicular (CCW rotate)
                lat_y =  fbx * inv_fb

        # ── Vectorised scoring ─────────────────────────────────────────
        # Gather sardine positions and velocities into numpy arrays.
        n_s = len(sardines)
        gxs  = np.empty(n_s, dtype=np.float64)
        gys  = np.empty(n_s, dtype=np.float64)
        gvxs = np.empty(n_s, dtype=np.float64)
        gvys = np.empty(n_s, dtype=np.float64)
        for i, s in enumerate(sardines):
            gxs[i]  = s._gx
            gys[i]  = s._gy
            gvxs[i] = s._gvx
            gvys[i] = s._gvy

        cx, cy = baitball.cx, baitball.cy
        W, H   = self._W, self._H
        hw, hh = self._hw, self._hh

        dx = gxs - cx
        dy = gys - cy
        # Toroidal correction.
        dx = np.where(dx >  hw, dx - W, np.where(dx < -hw, dx + W, dx))
        dy = np.where(dy >  hh, dy - H, np.where(dy < -hh, dy + H, dy))
        dists = np.sqrt(dx * dx + dy * dy)

        # Filter by edge floor.
        edge_mask = dists >= edge_floor
        if not edge_mask.any():
            return None

        # Base score: outer-edge bias + slower = more exposed.
        speeds  = np.sqrt(gvxs * gvxs + gvys * gvys)
        scores  = dists - 0.08 * speeds

        # Lateralization bias.
        if lat_enabled and lat_strength > 0.0 and (lat_x != 0.0 or lat_y != 0.0):
            safe_d = np.maximum(dists, 0.5)
            side_dot = (dx * lat_x + dy * lat_y) / safe_d   # in [-1, 1]
            scores += self.attack_side * side_dot * lat_strength * baitball.radius

        # Mask out non-edge candidates and find best.
        scores[~edge_mask] = -1e18
        best_i = int(scores.argmax())
        if scores[best_i] <= -1e17:
            return None
        return sardines[best_i]

    def _begin_baitball_strike(self, baitball: "_BaitballInfo", px: float, py: float) -> None:
        """Set strike line endpoints and arm the strike timer.

        The exit point is computed as a straight line from the fish through
        the baitball center and out the other side.  When lateralization is
        enabled the corridor is shifted slightly to the fish's preferred flank
        so each fish makes a slightly different angled pass.
        """
        cfg = self._config
        dx = baitball.cx - px
        dy = baitball.cy - py
        if dx > self._hw:
            dx -= self._W
        elif dx < -self._hw:
            dx += self._W
        if dy > self._hh:
            dy -= self._H
        elif dy < -self._hh:
            dy += self._H

        d2 = dx * dx + dy * dy
        if d2 <= 1e-9:
            vx = float(self.velocity[0])
            vy = float(self.velocity[1])
            vm2 = vx * vx + vy * vy
            if vm2 > 1e-9:
                inv = 1.0 / math.sqrt(vm2)
                ux = vx * inv
                uy = vy * inv
            else:
                ux, uy = 1.0, 0.0
        else:
            inv = 1.0 / math.sqrt(d2)
            ux = dx * inv
            uy = dy * inv

        # Lateralized exit: shift the exit anchor perpendicular to approach.
        # Each fish gets a unique corridor through the baitball based on its
        # stable attack_side preference.
        if cfg.sailfish_lateralization_enabled and self.lateralization_bias > 0.05:
            lat_offset = (
                self.attack_side
                * self.lateralization_bias
                * baitball.radius
                * 0.28
            )
            # Perpendicular to approach direction.
            perp_x = -uy
            perp_y =  ux
            exit_cx = baitball.cx + perp_x * lat_offset
            exit_cy = baitball.cy + perp_y * lat_offset
        else:
            exit_cx = baitball.cx
            exit_cy = baitball.cy

        strike_len = baitball.radius + cfg.sailfish_baitball_strike_exit_distance
        self._baitball_strike_exit_x = (exit_cx + ux * strike_len) % self._W
        self._baitball_strike_exit_y = (exit_cy + uy * strike_len) % self._H
        self._baitball_strike_timer = cfg.sailfish_baitball_strike_duration_seconds
        self._target_sardine = None
        self._target_lock_timer = 0.0

    def _apply_herding_pressure(
        self,
        baitball: "_BaitballInfo | None",
        px: float,
        py: float,
    ) -> None:
        """Push sailfish onto distributed pressure slots around dense schools.

        Anti-conga-line changes vs original implementation
        --------------------------------------------------
        * Each sailfish accumulates its own orbit angle at its own speed and
          direction (``_orbit_speed_mult``, ``_orbit_dir``), so fish naturally
          drift apart around the perimeter instead of rotating together.
        * Per-fish radius jitter (``_orbit_radius_mult``) adds loose radial
          spread so the ring is not perfectly uniform.
        * Inward pressure is capped (``sailfish_baitball_max_inward_pressure``)
          and tapers to zero when the fish is inside the orbit ring (falloff),
          preventing fish from pushing the baitball across the screen.
        * Drift damping opposes the detected baitball movement direction.
        * Attacking sailfish have their herding contribution scaled to near-zero
          (``sailfish_attack_pressure_suppression``) so they don't simultaneously
          herd AND strike.
        * Inter-sailfish spacing force breaks the conga-line following pattern.
        """
        cfg = self._config
        if baitball is None or not cfg.sailfish_herding_enabled:
            return
        if self._sailfish_count < cfg.sailfish_herding_min_sailfish:
            return
        if baitball.density < cfg.sailfish_baitball_density_threshold:
            return

        self._is_herding = True
        self._orbiting_baitball = True

        dt = self._dt

        # ── Attack pressure suppression ────────────────────────────────
        # Sailfish in a baitball attack apply near-zero herding so they don't
        # simultaneously push inward AND sprint at a target.
        if self._is_baitball_attacker and self.state in (
            "baitball_strike", "target_lock", "burst_attack"
        ):
            suppress = cfg.sailfish_attack_pressure_suppression
        else:
            suppress = 1.0
        if suppress <= 0.0:
            return

        # ── Per-fish orbit angle (anti-conga-line) ─────────────────────
        self._orbit_angle_accum += (
            cfg.sailfish_baitball_orbit_rotation_speed
            * self._orbit_speed_mult
            * float(self._orbit_dir)
            * dt
        )
        # Keep in [-100, 100] range to prevent float precision drift.
        if self._orbit_angle_accum > 100.0 or self._orbit_angle_accum < -100.0:
            self._orbit_angle_accum = math.fmod(self._orbit_angle_accum, 2.0 * math.pi)

        slot_count = max(cfg.sailfish_herding_min_sailfish, self._sailfish_count)
        slot = (self._sid + baitball.bid * 7) % slot_count
        base_theta = (2.0 * math.pi * slot) / slot_count
        jitter = math.sin(self._sid * 1.73) * cfg.sailfish_baitball_slot_jitter
        theta = base_theta + self._orbit_angle_accum + jitter

        # Per-fish orbit radius variation.
        pressure_r = (
            baitball.radius
            * cfg.sailfish_baitball_orbit_radius_multiplier
            * self._orbit_radius_mult
        )
        tx = (baitball.cx + math.cos(theta) * pressure_r) % self._W
        ty = (baitball.cy + math.sin(theta) * pressure_r) % self._H

        # Cache slot position for debug rendering.
        self.debug_orbit_slot_x = tx
        self.debug_orbit_slot_y = ty

        dxp = tx - px
        dyp = ty - py
        if dxp > self._hw: dxp -= self._W
        elif dxp < -self._hw: dxp += self._W
        if dyp > self._hh: dyp -= self._H
        elif dyp < -self._hh: dyp += self._H

        # Slot-hold force keeps each sailfish at a stable orbit position.
        self._steer_apply_xy(dxp, dyp, cfg.sailfish_herding_pressure_strength * suppress)

        # ── Vector from fish to baitball center ───────────────────────
        dx_center = baitball.cx - px
        dy_center = baitball.cy - py
        if dx_center > self._hw: dx_center -= self._W
        elif dx_center < -self._hw: dx_center += self._W
        if dy_center > self._hh: dy_center -= self._H
        elif dy_center < -self._hh: dy_center += self._H
        d2_center = dx_center * dx_center + dy_center * dy_center
        dist_center = math.sqrt(d2_center) if d2_center > 1.0 else 0.0

        # Tangential and inward forces require a valid center vector.
        tangent_x = tangent_y = 0.0
        if dist_center > 1.0:
            inv = 1.0 / dist_center
            nx = dx_center * inv
            ny = dy_center * inv

            # ── Tangential force (direction set by per-fish orbit direction) ─
            # CW (+1): tangent = (+ny, -nx);  CCW (-1): tangent = (-ny, +nx)
            if self._orbit_dir >= 0:
                tangent_x = -ny
                tangent_y =  nx
            else:
                tangent_x =  ny
                tangent_y = -nx
            self._steer_apply_xy(
                tangent_x,
                tangent_y,
                cfg.sailfish_baitball_orbit_tangent_strength * suppress,
            )

            # ── Inward pressure with falloff + cap ──────────────────────
            # sailfish_baitball_inward_pressure_strength is the nominal base
            # value (tunable via the config menu "Orbit inward" control).
            # sailfish_baitball_max_inward_pressure is the absolute safety
            # ceiling — the effective force never exceeds it even if strength
            # is set higher.
            inward_str = min(
                cfg.sailfish_baitball_inward_pressure_strength,
                cfg.sailfish_baitball_max_inward_pressure,
            )
            falloff_r  = pressure_r * cfg.sailfish_baitball_inner_pressure_falloff
            if falloff_r > 0.0 and dist_center < falloff_r:
                # Linear taper: zero at baitball center, full at falloff_r.
                inward_str *= dist_center / falloff_r
            if inward_str > 1e-6:
                self._steer_apply_xy(
                    dx_center, dy_center,
                    inward_str * suppress,
                )

        # ── Drift damping: estimate baitball velocity from frame-to-frame ─
        # We track the baitball center per-fish each frame.  If the baitball
        # has moved, we push against the drift to anchor it in place.
        if cfg.sailfish_baitball_anchor_enabled and cfg.sailfish_baitball_drift_damping_strength > 0.0:
            if self._prev_baitball_cx >= 0.0 and dt > 0.0:
                ddx = baitball.cx - self._prev_baitball_cx
                ddy = baitball.cy - self._prev_baitball_cy
                # Toroidal correction for the delta.
                if ddx > self._hw: ddx -= self._W
                elif ddx < -self._hw: ddx += self._W
                if ddy > self._hh: ddy -= self._H
                elif ddy < -self._hh: ddy += self._H
                drift_spd2 = (ddx * ddx + ddy * ddy) / (dt * dt)
                # Only dampen if drift is substantial (> 3 px/s) to avoid noise.
                if drift_spd2 > 9.0:
                    drift_vx = ddx / dt
                    drift_vy = ddy / dt
                    self._steer_apply_xy(
                        -drift_vx, -drift_vy,
                        cfg.sailfish_baitball_drift_damping_strength * suppress,
                    )
            self._prev_baitball_cx = baitball.cx
            self._prev_baitball_cy = baitball.cy

        # ── Inter-sailfish spacing during orbit (anti-conga-line) ──────
        # Push nearby sailfish apart tangentially to prevent them from
        # following directly behind one another in a line.
        if (
            cfg.sailfish_spacing_enabled
            and self._sailfish_grid is not None
            and suppress > 0.0
        ):
            sp_r  = cfg.sailfish_spacing_radius
            sp_r2 = sp_r * sp_r
            sp_w  = cfg.sailfish_spacing_strength * suppress
            fol_w = cfg.sailfish_following_penalty_strength * suppress
            sf_nb = self._sailfish_grid.query(self, sp_r)
            if sf_nb:
                ssx = ssy = 0.0
                for other in sf_nb:
                    ddx = px - other._gx
                    ddy = py - other._gy
                    if ddx > self._hw: ddx -= self._W
                    elif ddx < -self._hw: ddx += self._W
                    if ddy > self._hh: ddy -= self._H
                    elif ddy < -self._hh: ddy += self._H
                    dd2 = ddx * ddx + ddy * ddy
                    if dd2 <= _EPS2 or dd2 > sp_r2:
                        continue
                    inv_d = 1.0 / math.sqrt(dd2)
                    ssx += ddx * inv_d
                    ssy += ddy * inv_d
                    # Following penalty: if other is ahead in the orbit direction,
                    # apply a small counter-tangential push to spread fish out.
                    if fol_w > 0.0 and tangent_x != 0.0:
                        to_other_x = -ddx
                        to_other_y = -ddy
                        dot = to_other_x * tangent_x + to_other_y * tangent_y
                        if dot > 0.0:
                            falloff = 1.0 - math.sqrt(dd2) / sp_r
                            self._steer_apply_xy(
                                -tangent_x, -tangent_y,
                                fol_w * falloff,
                            )
                if ssx != 0.0 or ssy != 0.0:
                    self._steer_apply_xy(ssx, ssy, sp_w)

    def _apply_school_pressure(self, com_x: float, com_y: float) -> None:
        """Add a tangential orbit force around the school COM.

        Reinforces whichever circular direction is already set by the
        current velocity so sailfish circle rather than always diving in.
        """
        cfg    = self._config
        com_d2 = com_x * com_x + com_y * com_y
        if com_d2 <= 1.0:
            return
        com_inv = 1.0 / math.sqrt(com_d2)
        com_nx  = com_x * com_inv
        com_ny  = com_y * com_inv
        vx = float(self.velocity[0])
        vy = float(self.velocity[1])
        cross = com_nx * vy - com_ny * vx
        if cross >= 0.0:
            pnx = -com_ny;  pny =  com_nx   # CCW tangent
        else:
            pnx =  com_ny;  pny = -com_nx   # CW tangent
        self._steer_apply_xy(pnx, pny, cfg.sailfish_school_pressure_strength)
