"""Configuration for the V2 /sort pipeline (g1_dashboard_v2.py).

Three groups, deliberately separate:

1. HardwareLimits   -- facts about the robot taken from the repository
                       (sdk_wrapper.py, grasp_pipeline.py, hand_pose_navigation/*).
                       Frozen; not adjustable at runtime.
2. SafetyThresholds -- conservative limits the V2 controller never exceeds.
                       Frozen; changing one is a code review, not a UI action.
3. SortTuning       -- performance parameters.  Defaults come from values the
                       V1 pipeline already ran with on the robot; the dashboard
                       exposes a subset, and every setter clamps against (1)/(2).

Units: metres, radians, seconds.  Dex3 tactile values are raw sensor counts:
they are NOT calibrated forces, and every tactile threshold below is a change
relative to a per-element baseline recorded just before closing.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, replace
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class HardwareLimits:
    # Quick distance gate before IK.  The URDF palm link reaches at most
    # 0.4318 m from the shoulder (measured over the joint space, see
    # reach_map_v2.MODEL_MAX_PALM_REACH_M); 0.46 m adds the 3 cm vision
    # tolerance V1 used.  Points that pass still need IK to be reachable.
    arm_reach_m: float = 0.46
    # sdk_wrapper: Dex3 stall detection (error / speed / confirmation time).
    dex3_stall_error_rad: float = 0.16
    dex3_stall_speed_rad_s: float = 0.12
    # Dex3 motor temperatures (hottest of the two readings): warn, and cool
    # down limp before the next grasp.  Measured 42-50 C after three picks.
    dex3_temp_warn_c: float = 60.0
    dex3_temp_limit_c: float = 70.0
    # sdk_wrapper.MAX_KD applies to every arm command.
    arm_max_kd: float = 10.0
    # Dex3 tactile sentinel for "no reading" (grasp_pipeline.TACTILE_INVALID_VALUE).
    tactile_invalid_raw: float = 30000.0
    # rt/arm_sdk weight slot used by engage/release (sdk_wrapper).
    arm_sdk_weight_index: int = 29


@dataclass(frozen=True)
class SafetyThresholds:
    # Generic table clearance for the wrist and elbow on every move that is not
    # the final near-table grasp segment (grasp_pipeline.TABLE_COLLISION_MARGIN_M).
    table_margin_m: float = 0.10
    # Final grasp segment: the whole hand model (see HandGeometry) must keep at
    # least this much clearance from the table plane.
    grasp_hand_table_clearance_m: float = 0.015
    # Carried object: its bottom keeps this clearance above the table and
    # above any obstacle it passes over.
    carried_object_clearance_m: float = 0.03
    # Torso: ReachabilityChecker rejects < 0.03 m from the 0.18 m cylinder.
    torso_keep_out_radius_m: float = 0.23
    # Hard cap of the arm joint speed for any V2 motion (sdk_wrapper move_ll_joint
    # defaults to 0.45 rad/s; grasp_pipeline ABSOLUTE_MAX_JOINT_SPEED_RAD_S 0.60).
    max_joint_speed_rad_s: float = 0.60
    # Per-joint bound of the gravity-sag integral (command minus reference).
    # At kp=60 this is at most 60 * 0.20 = 12 Nm of extra effort per joint
    # (0.15 saturated carrying the bottle at the far end of the transport).
    max_sag_compensation_rad: float = 0.20
    # Reference-vs-measured joint error that means the arm cannot follow
    # (blocked, in contact, or sagging beyond compensation).
    max_tracking_error_rad: float = 0.25
    # Dex3 finger command step limit per control tick and the squeeze past a
    # blocked finger (stays below the stall error so no stall fires on hold).
    max_finger_squeeze_rad: float = 0.15
    # Carried mass the controller accepts at all (grasp_pipeline.MAX_OBJECT_MASS_KG
    # is 5 kg for the UI; a Dex3 lateral grip is far below that).
    max_carry_mass_kg: float = 0.5
    # Largest joint change one planner step (<= ~1 cm of hand travel) may ask
    # for; a bigger jump means the IK switched configuration branch and the
    # solution is never commanded.  Smooth motion near the edge of the reach
    # needs up to ~0.21 rad per 7 mm step (measured in simulation for a cup
    # straight ahead at x=0.41 m), so the limit sits above that.
    max_ik_step_jump_rad: float = 0.30


@dataclass(frozen=True)
class HandGeometry:
    """Conservative Dex3 hand model in the hand (wrist) frame.

    Collision points run from the wrist along the fingers (hand x).  The hand
    extends below those points by `low_extent_m` when the palm is level: in
    the 2026-10-26 16:39 test the hand rubbed the table with the wrist 7.6 cm
    above it and the palm tilted ~6 deg, i.e. the lowest point sat >= ~6 cm
    below the wrist axis.  This value is an estimate and must be calibrated on
    the robot (checklist step 3).
    """
    collision_offsets_m: Tuple[float, ...] = (0.0, 0.045, 0.09, 0.13)
    collision_radius_m: float = 0.04
    # Slim model of the open fingers, used to keep them off the target object
    # while the hand comes down behind it (the body model above is too coarse
    # for that: it would push the pre-grasp ~15 cm behind every object).
    finger_radius_m: float = 0.02
    low_extent_m: float = 0.065
    palm_offset_m: float = 0.045
    # Object centre relative to the wrist that the fingers can enclose: along
    # the fingers between palm and fingertips, small lateral/vertical slack.
    capture_forward_m: Tuple[float, float] = (0.03, 0.12)
    capture_lateral_m: float = 0.035
    capture_vertical_m: float = 0.06
    # When the default grasp (object centre ~8 cm ahead of the palm link) is
    # beyond the arm's reach, the object may sit further toward the
    # fingertips, up to this, and the grip may rise to within
    # min_grip_below_top_m of the object's top.  The arm's reach drops fast
    # below shoulder height (palm link ~0.35 m forward at 17 cm below it).
    max_grasp_standoff_m: float = 0.10
    min_grip_below_top_m: float = 0.025


@dataclass
class SortTuning:
    # Arm controller (one continuous publisher for the whole sort).  kp/kd are
    # those of V1's approach executor (ArmExecutor default); V1's hold/lift ran
    # at kp=30, which let the extended arm sag several cm.
    control_rate_hz: float = 50.0
    arm_kp: float = 60.0
    arm_kd: float = 2.0
    sag_integral_gain: float = 0.8          # 1/s, joint-space integral on (ref - measured)
    sag_integral_active_rad: float = 0.12   # integrate only while |ref - measured| is below this
    # Errors below this are not integrated: with joint friction a pure
    # integral hunts around the reference (seen as a slow elbow shake).
    sag_integral_deadband_rad: float = 0.004
    # IK iteration cap per solver tier (a failing solve otherwise costs
    # hundreds of numerical-Jacobian iterations on the Jetson).
    ik_max_iter: int = 80
    max_joint_speed_rad_s: float = 0.45
    # Cartesian moves (straight lines, IK per tick, palm kept level).
    planner_rate_hz: float = 10.0
    approach_speed_m_s: float = 0.03
    transport_speed_m_s: float = 0.05
    lift_speed_m_s: float = 0.03
    retract_speed_m_s: float = 0.05
    move_tolerance_m: float = 0.012
    move_settle_s: float = 0.3
    # Approach / grasp gate.
    grasp_position_tolerance_m: float = 0.02
    grasp_tilt_tolerance_rad: float = math.radians(5.0)
    # Finger direction (hand x, horizontal) vs the plan: a level-palm IK leaves
    # yaw free, and 20 deg of yaw put the object 5 cm beside the fingers.
    grasp_yaw_tolerance_rad: float = math.radians(8.0)
    grasp_gate_ticks: int = 2
    pregrasp_backoff_m: float = 0.06
    # Side approach: the hand comes down beside the object on the side away
    # from the palm (the robot's right for the right hand), then moves along
    # the palm normal onto the grasp.  The object edge stays this far clear of
    # the hand body (collision_radius_m) while the hand comes down.
    side_approach_margin_m: float = 0.02
    # Last IK tier: a level-palm solution whose wrist misses the target by at
    # most this much (the edge of the arm's level-palm workspace, e.g. a cup
    # straight ahead at x~0.41 m).  Every solution is still validated on its
    # own FK, and closing still requires the measured grasp gate.
    ik_reach_tolerance_m: float = 0.015
    hover_clearance_m: float = 0.05
    # Progress / blocking detection for every Cartesian or joint move.
    progress_window_s: float = 3.0
    progress_min_m: float = 0.005
    # Transport / release / retreat / recovery moves that settle this close
    # to the goal (loaded arm sagging) count as reached.
    short_move_accept_m: float = 0.03
    max_consecutive_rejections: int = 8
    max_consecutive_ik_failures: int = 8
    move_timeout_s: float = 30.0
    # Detection (phase 1).
    detection_passes: int = 3
    detection_min_consistent: int = 2
    detection_consistency_m: float = 0.03
    detection_timeout_s: float = 8.0
    min_detection_score: float = 0.4
    blacklist_radius_m: float = 0.04
    max_attempts_per_object: int = 3
    # Dex3 closure (phase 3).
    finger_close_step: float = 0.05           # closure fraction per step (0 open .. 1 closed)
    finger_step_s: float = 0.06
    finger_kp: float = 1.8                    # V1 DEFAULT_TACTILE_CLOSE_KP
    finger_kd: float = 0.08
    finger_tau: float = 0.06
    finger_squeeze_rad: float = 0.12
    finger_min_contact_closure: float = 0.10  # blocked nearer open than this = jam
    tactile_baseline_s: float = 0.3
    tactile_contact_delta_raw: float = 1000.0  # idle per-element noise measured <= 8 raw (std)
    tactile_firm_delta_raw: float = 3000.0
    tactile_min_contact_sensors: int = 2
    grip_confirm_s: float = 0.3
    held_min_block_rad: float = 0.035          # finger still short of its squeeze target = blocked by object
    # Stale finger: commanded at least this far but moved less than this (the
    # motor did not follow at all, unlike a finger stopped by an object).
    finger_stale_min_command_rad: float = 0.30
    finger_stale_max_motion_rad: float = 0.05
    # Free-space finger check before every grasp (partial close and open),
    # with the stale-finger recovery; the closure fraction it closes to.
    finger_check_before_grasp: bool = True
    finger_check_closure: float = 0.35
    finger_cooldown_max_s: float = 120.0
    # Lift (phase 4) and transport (phase 5).
    lift_m: float = 0.08
    lift_tolerance_m: float = 0.015
    transport_extra_clearance_m: float = 0.03
    # Place / retract (phases 6-7).  The release height itself is configured
    # per drop-off pattern in the /sort UI; V2 raises it when the carried
    # object hangs lower (SafetyThresholds.carried_object_clearance_m).
    release_open_s: float = 1.0
    # Gentle placing: below the release pose, lower slowly toward where the
    # object's bottom meets the spot (its hang below the wrist at the grasp)
    # and up to touchdown_overshoot_m further, stopping as soon as the descent
    # stalls (the object rests on the table), then open.
    place_touchdown: bool = True
    touchdown_speed_m_s: float = 0.01
    touchdown_overshoot_m: float = 0.015
    touchdown_window_s: float = 0.5
    touchdown_stall_ratio: float = 0.3        # measured descent < this x commanded = resistance
    # After opening on the spot (and after a recovery set-down): move the
    # palm away from the object along the palm normal before lifting, so the
    # open fingers do not catch a lid or rim.  The approach's side offset is
    # used when larger.  0 disables.
    release_slide_m: float = 0.05
    retreat_m: float = 0.06
    prep_tolerance_rad: float = 0.06
    # From a hanging arm, the first motion swings the hand out to the side
    # (shoulder roll, outward, absolute) so the fingers clear the thigh before
    # anything rises.  The extend recipe itself uses 0.50 rad.
    prep_abduction_rad: float = 0.45
    prep_abduction_speed_rad_s: float = 0.3
    # Joint speed of the rise beside the body after swinging out (and of the
    # descent back down when parking): one smooth joint move each way.
    prep_rise_speed_rad_s: float = 0.35
    placement_confirm_timeout_s: float = 6.0
    # Finish (phase 8).
    release_arms_at_end: bool = True
    final_hand_tolerance_rad: float = 0.25

    # Bounds for every value an operator may change at runtime: (min, max).
    ADJUSTABLE = {
        "approach_speed_m_s": (0.01, 0.05),
        "transport_speed_m_s": (0.01, 0.08),
        "lift_speed_m_s": (0.01, 0.05),
        "retract_speed_m_s": (0.01, 0.08),
        "max_joint_speed_rad_s": (0.05, 0.60),
        "grasp_position_tolerance_m": (0.008, 0.03),
        "side_approach_margin_m": (0.01, 0.06),
        "tactile_contact_delta_raw": (300.0, 20000.0),
        "tactile_firm_delta_raw": (500.0, 50000.0),
        "lift_m": (0.04, 0.15),
        "max_attempts_per_object": (1, 5),
        "release_arms_at_end": (False, True),
        "finger_check_before_grasp": (False, True),
        "place_touchdown": (False, True),
        "touchdown_speed_m_s": (0.005, 0.02),
        "touchdown_overshoot_m": (0.0, 0.02),
        "release_slide_m": (0.0, 0.10),
    }

    def update(self, values: Dict[str, Any], safety: SafetyThresholds) -> Dict[str, Any]:
        """Apply operator changes, clamped to ADJUSTABLE and the safety caps;
        returns {name: applied value}.  Unknown/non-adjustable names raise."""
        applied: Dict[str, Any] = {}
        for name, raw in values.items():
            if name not in self.ADJUSTABLE:
                raise ValueError(f"{name!r} is not an adjustable sort parameter")
            lo, hi = self.ADJUSTABLE[name]
            current = getattr(self, name)
            if isinstance(current, bool):
                value: Any = bool(raw)
            elif isinstance(current, int):
                value = int(max(lo, min(hi, int(float(raw)))))
            else:
                value = float(max(lo, min(hi, float(raw))))
            if name == "max_joint_speed_rad_s":
                value = min(value, safety.max_joint_speed_rad_s)
            setattr(self, name, value)
            applied[name] = value
        return applied

    def as_dict(self) -> Dict[str, Any]:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass
class SortV2Config:
    hardware: HardwareLimits = field(default_factory=HardwareLimits)
    safety: SafetyThresholds = field(default_factory=SafetyThresholds)
    hand: HandGeometry = field(default_factory=HandGeometry)
    tuning: SortTuning = field(default_factory=SortTuning)

    def grasp_wrist_floor_m(self) -> float:
        """Lowest wrist height above the table for a level-palm grasp: the
        hand's low extent, its required clearance, and the fingertip drop at
        the allowed palm tilt."""
        tilt_drop = max(self.hand.collision_offsets_m) * math.sin(self.tuning.grasp_tilt_tolerance_rad)
        return self.hand.low_extent_m + self.safety.grasp_hand_table_clearance_m + tilt_drop

    def describe(self) -> Dict[str, Any]:
        return {"hardware": asdict(self.hardware), "safety": asdict(self.safety), "hand": asdict(self.hand),
                "tuning": self.tuning.as_dict(), "adjustable": {k: list(v) for k, v in SortTuning.ADJUSTABLE.items()},
                "grasp_wrist_floor_m": self.grasp_wrist_floor_m()}


def default_config() -> SortV2Config:
    return SortV2Config()


# Corrections ik_calibration.py may propose, with the bounds this loader enforces.
CALIBRATION_BOUNDS = {
    "hand.max_grasp_standoff_m": (0.06, 0.12),          # capture zone ends at 0.12 m
    "tuning.grasp_tilt_tolerance_rad": (math.radians(2.0), math.radians(12.0)),
    "tuning.lift_m": SortTuning.ADJUSTABLE["lift_m"],
    "camera_correction_m": 0.08,                         # largest accepted camera translation fix
}


def load_calibration(cfg: SortV2Config, path: Any) -> Tuple[Optional[Dict[str, float]], List[str]]:
    """Apply a sort_v2_calibration.json (from ik_calibration.py analyze --apply)
    to `cfg` in place.  Returns (camera extrinsic override or None, notes).
    Only the bounded fields above are read; safety thresholds never change."""
    import json
    from pathlib import Path as _Path
    notes: List[str] = []
    path = _Path(path)
    if not path.exists():
        return None, notes
    data = json.loads(path.read_text())
    camera = data.get("camera_extrinsic")
    correction = float(np.linalg.norm(data.get("camera_correction_m", [0.0, 0.0, 0.0]))) if camera else 0.0
    if camera and correction > CALIBRATION_BOUNDS["camera_correction_m"]:
        notes.append(f"camera correction {correction:.3f} m exceeds the bound; ignored")
        camera = None
    if camera:
        camera = {k: float(camera[k]) for k in ("x", "y", "z", "roll", "pitch", "yaw")}
        notes.append(f"camera extrinsic from {path.name} (correction {correction * 100:.1f} cm)")
    standoff = (data.get("hand") or {}).get("max_grasp_standoff_m")
    if standoff is not None:
        lo, hi = CALIBRATION_BOUNDS["hand.max_grasp_standoff_m"]
        value = float(min(hi, max(lo, float(standoff))))
        cfg.hand = replace(cfg.hand, max_grasp_standoff_m=value)
        notes.append(f"max_grasp_standoff_m = {value:.3f}")
    lift = (data.get("tuning") or {}).get("lift_m")
    if lift is not None:
        lo, hi = SortTuning.ADJUSTABLE["lift_m"]
        cfg.tuning.lift_m = float(min(hi, max(lo, float(lift))))
        notes.append(f"lift_m = {cfg.tuning.lift_m:.3f}")
    tilt = (data.get("tuning") or {}).get("grasp_tilt_tolerance_rad")
    if tilt is not None:
        lo, hi = CALIBRATION_BOUNDS["tuning.grasp_tilt_tolerance_rad"]
        cfg.tuning.grasp_tilt_tolerance_rad = float(min(hi, max(lo, float(tilt))))
        notes.append(f"grasp_tilt_tolerance_rad = {cfg.tuning.grasp_tilt_tolerance_rad:.3f}")
    return camera, notes


__all__ = ["HardwareLimits", "SafetyThresholds", "HandGeometry", "SortTuning", "SortV2Config", "default_config"]
