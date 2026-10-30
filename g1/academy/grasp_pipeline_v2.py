"""V2 manipulation layer for /sort: one arm controller, validated straight-line
moves, whole-hand table geometry and a feedback-driven Dex3 grip.

Built on grasp_pipeline.GraspPipeline (camera, perception, table and object
segmentation, grasp-pose construction, overlays), whose motion code is left
untouched for V1.  Root causes this module addresses (see the V2 summary in
g1_dashboard_v2.py for the evidence):

* Near-table approach rejected forever ("hand would clip table z=0.087 <
  0.014 + 0.080"): V1's HandApproach took capped *joint-space* fractions of
  each IK step.  Partial joint steps of an extending arm leave the straight
  Cartesian line and dip below both endpoints; with the wrist planned 1 cm
  above the reduced 8 cm wrist-point margin, every tick produced the same
  rejected step.  Dynamic target corrections (not frozen until 6 cm) kept
  resetting the stall window, so the loop never ended.  V2 interpolates the
  hand along the straight line, solves IK per point, validates the predicted
  physical path with a whole-hand model, and fails on bounded rejection /
  progress counts.
* Incomplete lift / arm drooping after the grasp: V1 approached at kp=60
  (ArmExecutor default) but held and lifted at kp=30 (hold executor,
  lift_tuning_for_mass), so the extended arm sagged at every handover and a
  5 cm Cartesian bias over 4 rounds could not recover it.  V2 keeps one
  publisher with one gain set for the whole sort and removes steady-state sag
  with a bounded joint-space integral.
* Flat tactile readings: V1 compared the global maximum over all pads with
  an absolute target.  Idle offsets differ by up to ~1700 raw between
  elements (one static element dominated the maximum) while per-element
  noise is <10 raw, so real contacts on other pads were invisible.  V2 uses
  per-element deltas from a baseline recorded just before closing, per sensor.
"""
from __future__ import annotations

import functools
import math
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

import grasp_pipeline as gp1
from grasp_pipeline import (
    DEFAULT_DIRECT_MAX_REACH_M, TABLE_COLLISION_XY_MARGIN_M, GraspPipeline, TableObject, base_from_camera,
    direct_grasp_hand_pose, level_rotation, palm_visual_point,
)
from hand_pose_navigation.arm_fk import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, SHOULDER_IN_BASE, ArmFK
from hand_pose_navigation.arm_ik import ArmIK
from hand_pose_navigation.obstacle_checker import Obstacles, check_swept_path
from hand_pose_navigation.reachability_checker import ReachabilityChecker
from hand_pose_navigation.table_estimator import TablePlane
from sdk_wrapper import HAND_CLOSED, HAND_JOINT_NAMES, HAND_SAFE_OPEN
from sort_v2_config import SortV2Config, default_config
from sorting_state_machine import Failure

# Violation texts of a pose that is too low (table / floor / minimum height,
# or a carried object too close to an obstacle under it): a rising move that
# does not go lower may still run.
LOW_POSE_MARKERS = ("below", "too close to the table", "carried object would hit obstacle")
# Waist yaw joint and the gains sdk_wrapper.engage_arms holds the waist with.
WAIST_YAW_JOINT = 12
WAIST_KP = 480.0
WAIST_KD = 10.0
# A tactile baseline younger than this (taken at the pre-grasp, hand clear of
# the object) is kept when closing.
GRIP_BASELINE_MAX_AGE_S = 15.0
THUMB_JOINTS = ("thumb_1", "thumb_2")
FINGER_JOINTS = ("middle_0", "middle_1", "index_0", "index_1")


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def interpolate_pose(T0: np.ndarray, T1: np.ndarray, alpha: float) -> np.ndarray:
    """Straight-line position, geodesic rotation (axis-angle) blend."""
    alpha = float(min(1.0, max(0.0, alpha)))
    T = np.eye(4)
    T[:3, 3] = T0[:3, 3] + (T1[:3, 3] - T0[:3, 3]) * alpha
    R_rel = T0[:3, :3].T @ T1[:3, :3]
    angle = math.acos(float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-6:
        T[:3, :3] = T1[:3, :3] if alpha >= 1.0 else T0[:3, :3]
        return T
    axis = np.array([R_rel[2, 1] - R_rel[1, 2], R_rel[0, 2] - R_rel[2, 0], R_rel[1, 0] - R_rel[0, 1]])
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:  # ~180 deg: no stable axis, snap at the end
        T[:3, :3] = T1[:3, :3] if alpha >= 0.5 else T0[:3, :3]
        return T
    k = axis / norm
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    a = angle * alpha
    T[:3, :3] = T0[:3, :3] @ (np.eye(3) + math.sin(a) * K + (1.0 - math.cos(a)) * (K @ K))
    return T


def palm_tilt(T_a: np.ndarray, T_b: np.ndarray) -> float:
    return math.acos(float(np.clip(np.dot(T_a[:3, 2], T_b[:3, 2]), -1.0, 1.0)))


def yaw_error(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """Angle between the horizontal finger directions (hand x) of two poses."""
    a, b = T_a[:2, 0], T_b[:2, 0]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return math.acos(float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)))


@dataclass
class CarriedObject:
    """The held object, hanging below the wrist."""
    offset_hand: np.ndarray      # object centre from the wrist, hand frame
    hang_m: float                # how far its bottom sits below the wrist
    radius_m: float


@dataclass
class MotionContext:
    """What a move must stay clear of."""
    table: Optional[TablePlane]
    objects: Sequence[TableObject] = ()
    # "general": wrist and elbow keep SafetyThresholds.table_margin_m (V1 rule).
    # "grasp": the whole hand model keeps grasp_hand_table_clearance_m; the elbow
    #          keeps the general margin.  Only the descent to and the final
    #          approach of a grasp use it.
    hand_mode: str = "general"
    # Vertical moves that start inside the general margin (a lift from the
    # grasp height): the wrist may not drop below this instead.
    min_wrist_z_m: Optional[float] = None
    carried: Optional[CarriedObject] = None
    # The target object while the hand is not yet allowed to enclose it
    # (clearance, descent to the pre-grasp): fingers must stay off it.
    keep_clear: Sequence[TableObject] = ()
    # Setting the carried object down on the drop spot: its bottom may reach
    # the table and the hand may use its table clearance band (touch-down).
    placing: bool = False
    label: str = ""


@dataclass
class MoveResult:
    ok: bool
    failure: Optional[Failure] = None
    detail: str = ""
    error_m: float = float("nan")
    tilt_rad: float = float("nan")
    data: Dict[str, Any] = field(default_factory=dict)


class MotionValidator:
    """Existing workspace / torso / opposite-arm checks (ReachabilityChecker,
    check_swept_path) plus V2's table, hand and carried-object geometry."""

    def __init__(self, arm: str, cfg: SortV2Config, opposite_wrist: Callable[[], Optional[np.ndarray]]) -> None:
        self.arm = arm
        self.cfg = cfg
        self.fk = ArmFK(arm=arm, backend="urdf")
        self.checker = ReachabilityChecker(arm=arm, max_reach_m=DEFAULT_DIRECT_MAX_REACH_M)
        self.opposite_arm = "left" if arm == "right" else "right"
        self.opposite_wrist = opposite_wrist

    def hand_points(self, T: np.ndarray) -> List[np.ndarray]:
        return [T[:3, 3] + T[:3, :3] @ np.array([o, 0.0, 0.0]) for o in self.cfg.hand.collision_offsets_m]

    @staticmethod
    def _over_table(table: TablePlane, p: np.ndarray) -> bool:
        m = TABLE_COLLISION_XY_MARGIN_M
        return (table.x_range_m[0] - m <= p[0] <= table.x_range_m[1] + m
                and table.y_range_m[0] - m <= p[1] <= table.y_range_m[1] + m)

    def pose_violations(self, T: np.ndarray, elbow: Optional[np.ndarray], ctx: MotionContext,
                        clearance_slack_m: float = 0.0) -> List[str]:
        """Geometric checks of one hand pose (no IK/workspace)."""
        s, h = self.cfg.safety, self.cfg.hand
        out: List[str] = []
        table = ctx.table
        if table is not None:
            general_floor = table.z_base_m + s.table_margin_m - clearance_slack_m
            if elbow is not None and self._over_table(table, elbow) and elbow[2] < general_floor:
                out.append(f"elbow z={elbow[2]:.3f}m below table {table.z_base_m:.3f}+{s.table_margin_m:.3f}m")
            if ctx.hand_mode == "grasp":
                down = T[:3, 2] if T[2, 2] > 0.0 else np.array([0.0, 0.0, 1.0])
                # Setting an object down may use the clearance band: the slow
                # descent stops at the first resistance (object or hand).
                clearance = self.cfg.place_hand_clearance_m() if ctx.placing else self.cfg.grasp_hand_clearance_m()
                floor = table.z_base_m + clearance - clearance_slack_m
                for i, p in enumerate(self.hand_points(T)):
                    low = p - down * h.low_extent_m
                    if self._over_table(table, low) and low[2] < floor:
                        out.append(f"hand point {i} lowest z={low[2]:.3f}m below table {table.z_base_m:.3f}"
                                   f"+{clearance:.3f}m")
                        break
            else:
                wrist_floor = general_floor if ctx.min_wrist_z_m is None else min(general_floor, ctx.min_wrist_z_m)
                if self._over_table(table, T[:3, 3]) and T[2, 3] < wrist_floor:
                    out.append(f"wrist z={T[2, 3]:.3f}m below {wrist_floor:.3f}m")
        for p in self.hand_points(T):
            for i, obj in enumerate(ctx.objects):
                reach = obj.radius_m + h.collision_radius_m + gp1.OBJECT_OBSTACLE_MARGIN_M - clearance_slack_m
                if (float(np.linalg.norm(p[:2] - obj.center_xy)) < reach
                        and p[2] - h.collision_radius_m < obj.top_z_m + gp1.OBJECT_OBSTACLE_MARGIN_M - clearance_slack_m):
                    out.append(f"hand would hit obstacle {i + 1} at ({obj.center_xy[0]:+.2f}, {obj.center_xy[1]:+.2f})")
                    break
        for obj in ctx.keep_clear:
            # The slack (measured poses only) also applies sideways: the plan is
            # checked without it, so execution tolerates small tracking deviations.
            reach = obj.radius_m + h.finger_radius_m + 0.01 - clearance_slack_m
            for i, p in enumerate(self.hand_points(T)):
                if float(np.linalg.norm(p[:2] - obj.center_xy)) < reach and \
                        p[2] - h.finger_radius_m < obj.top_z_m + 0.01 - clearance_slack_m:
                    out.append(f"hand point {i} would come down on the target object (top z={obj.top_z_m:.3f}m)")
                    break
        carried = ctx.carried
        if carried is not None:
            centre = T[:3, 3] + T[:3, :3] @ carried.offset_hand
            bottom = float(T[2, 3] - carried.hang_m)
            # Placing: the bottom is computed from the hang at the grasp, which
            # is wrong once the object slides in the hand on touching the table
            # (seen: "bottom z=-0.057" while the cup stood on the table).  The
            # set-down's own floor and the hand's table check apply instead.
            floor = table.z_base_m + s.carried_object_clearance_m if table is not None else 0.0
            if table is not None and not ctx.placing and self._over_table(table, centre) and \
                    bottom < floor - clearance_slack_m:
                out.append(f"carried object bottom z={bottom:.3f}m too close to the table")
            for i, obj in enumerate(ctx.objects):
                if float(np.linalg.norm(centre[:2] - obj.center_xy)) < obj.radius_m + carried.radius_m + 0.02 and \
                        bottom < obj.top_z_m + s.carried_object_clearance_m - clearance_slack_m:
                    out.append(f"carried object would hit obstacle {i + 1}")
                    break
        return out

    def check_q(self, q: np.ndarray, ctx: MotionContext, T_desired: Optional[np.ndarray] = None) -> List[str]:
        T = self.fk.compute_arm(q)
        reach = self.checker.check(q, T_desired if T_desired is not None else T)
        out = [f"workspace: {r}" for r in reach.reasons] if not reach.safe else []
        return out + self.pose_violations(T, self.fk.compute_arm_partial(q, 4)[:3, 3], ctx)

    def check_segment(self, q0: np.ndarray, q1: np.ndarray, ctx: MotionContext,
                      T_desired: Optional[np.ndarray] = None, samples: int = 4) -> List[str]:
        wrist = self.opposite_wrist()
        sweep = check_swept_path(self.fk, q0, q1, Obstacles(
            table=None, opposite_shoulder_base=SHOULDER_IN_BASE[self.opposite_arm] if wrist is not None else None,
            opposite_wrist_base=wrist), n_samples=samples)
        if not sweep.safe:
            return list(sweep.reasons)
        z_start = float(self.fk.compute_arm(q0)[2, 3])
        for i in range(1, samples + 1):
            a = i / samples
            q = (1.0 - a) * q0 + a * q1
            reasons = self.check_q(q, ctx, T_desired if i == samples else None) if i == samples else \
                self.pose_violations(self.fk.compute_arm(q), self.fk.compute_arm_partial(q, 4)[:3, 3], ctx)
            # An arm hanging below ReachabilityChecker's minimum wrist height
            # (released arms) may still rise out of it, and may go back down to
            # an explicit floor (parking at the recorded hanging pose).
            z = float(self.fk.compute_arm(q)[2, 3])
            if reasons and (z >= z_start - 1e-3 or (ctx.min_wrist_z_m is not None and z >= ctx.min_wrist_z_m)):
                reasons = [r for r in reasons if not r.startswith("workspace: Wrist z=")]
            if reasons:
                return [f"[alpha={a:.2f}] {r}" for r in reasons]
        return []


# ---------------------------------------------------------------------------
# Arm controller: the only rt/arm_sdk publisher during a V2 sort
# ---------------------------------------------------------------------------

class ArmController:
    """Publishes reference + sag integral at a fixed rate with one gain set.

    The reference is what the planner validated (the intended physical pose).
    The command is reference + integral, where the integral removes the
    steady-state gravity sag of the position loop.  It learns only while the
    reference is still (tracking lag during motion is not sag and would wind
    it up) and the arm tracks closely, keeps its value through the next move
    (gravity changes slowly with the configuration), is bounded per joint, and
    the command itself is speed-limited, so a blocked arm is detected instead
    of pushed harder.
    """

    STATIC_AFTER_S = 0.25

    def __init__(self, robot: Any, arm: str, cfg: SortV2Config, read_arm_q: Callable[[str], np.ndarray]) -> None:
        self.robot = robot
        self.arm = arm
        self.cfg = cfg
        self.read_arm_q = read_arm_q
        self.joints = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._q_ref = np.zeros(7)
        self._q_cmd = np.zeros(7)
        self._integral = np.zeros(7)
        self._q_meas = np.zeros(7)
        # False freezes the sag integral at its current value (holding against
        # the table or in an error hold: no windup); every V2 move resets it.
        self.integrate = True
        self._ref_changed_ts = 0.0
        self.last_error = ""
        self.loop_errors = 0
        self.read_failures = 0
        self.last_read_ts = 0.0
        # Waist yaw (joint 12): None = not commanded by this controller (the
        # arm_sdk message keeps what engage wrote).  Once set, the command is
        # ramped at waist_yaw_speed_rad_s and published with the arm joints.
        self._waist_target: Optional[float] = None
        self._waist_cmd: Optional[float] = None

    def set_waist_yaw(self, target: float, current: float) -> None:
        """Ramp the waist yaw command from `current` (measured) to `target`,
        bounded by SafetyThresholds.max_waist_yaw_rad."""
        lim = self.cfg.safety.max_waist_yaw_rad
        with self._lock:
            if self._waist_cmd is None:
                self._waist_cmd = float(current)
            self._waist_target = float(max(-lim, min(lim, target)))

    def waist_command(self) -> Optional[float]:
        with self._lock:
            return self._waist_cmd

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        q = np.asarray(self.read_arm_q(self.arm), dtype=np.float64)
        with self._lock:
            self._q_ref, self._q_cmd, self._q_meas = q.copy(), q.copy(), q.copy()
            self._integral = np.zeros(7)
            self.last_read_ts = time.time()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=f"sort-v2-arm-{self.arm}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        thread = self._thread
        self._stop.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._thread = None

    def set_reference(self, q: np.ndarray) -> None:
        q = np.asarray(q, dtype=np.float64)
        with self._lock:
            if float(np.max(np.abs(q - self._q_ref))) > 1e-5:
                self._ref_changed_ts = time.time()
            self._q_ref = q.copy()

    def reference(self) -> np.ndarray:
        with self._lock:
            return self._q_ref.copy()

    def mark_free(self) -> None:
        """Remember the free-arm state (command-minus-measured gap and the
        sag integral) while nothing touches the hand: relax() restores it."""
        with self._lock:
            self._free = (self._q_ref - self._q_meas, self._integral.copy())

    def relax(self) -> bool:
        """Hold the arm where it measures now without pressing on whatever
        stopped it: integral back to the last free value (a wound-up integral
        kept pushing into the table: "arm not following" on every move after),
        reference = measured + the free gap (the measured pose alone drops a
        sagging arm by that gap), integral frozen until the next move.  False
        if no free state was marked."""
        with self._lock:
            free = getattr(self, "_free", None)
            self.integrate = False
            if free is None:
                return False
            gap, integral = free
            self._integral = integral.copy()
            self._q_ref = self._q_meas + gap
            self._ref_changed_ts = time.time()
            return True

    def measured(self) -> np.ndarray:
        with self._lock:
            return self._q_meas.copy()

    def lag_rad(self) -> float:
        """How far the speed-limited command still trails the reference."""
        with self._lock:
            return float(np.max(np.abs(self._q_ref + self._integral - self._q_cmd)))

    def status(self) -> Dict[str, Any]:
        with self._lock:
            # Compensated command vs measured: sag beyond the integral, a
            # blocked joint, or a lost position loop (not the planner lead).
            err = (self._q_cmd - self._integral) - self._q_meas
            sat = np.abs(self._integral) >= self.cfg.safety.max_sag_compensation_rad - 1e-6
            return {"running": self.running, "tracking_error_rad": float(np.max(np.abs(err))),
                    "command_lag_rad": float(np.max(np.abs(self._q_ref + self._integral - self._q_cmd))),
                    "sag_compensation_rad": float(np.max(np.abs(self._integral))),
                    "sag_saturated": bool(np.any(sat)), "read_failures": self.read_failures,
                    "loop_errors": self.loop_errors, "last_error": self.last_error,
                    "state_age_s": time.time() - self.last_read_ts}

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._run()
                return
            except Exception as exc:  # keep publishing; the movers see the error count
                self.loop_errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._stop.wait(0.02)

    def _run(self) -> None:
        t = self.cfg.tuning
        s = self.cfg.safety
        dt = 1.0 / max(5.0, t.control_rate_hz)
        kd = min(t.arm_kd, self.cfg.hardware.arm_max_kd)
        last_tick = time.time()
        while not self._stop.is_set():
            t0 = time.time()
            # Real elapsed time (a loaded CPU delays ticks), bounded so a long
            # stall can never turn into a jump.
            step_dt = min(0.1, max(1e-3, t0 - last_tick))
            last_tick = t0
            speed = min(t.max_joint_speed_rad_s, s.max_joint_speed_rad_s)
            try:
                q_meas = np.asarray(self.read_arm_q(self.arm), dtype=np.float64)
                with self._lock:
                    self._q_meas = q_meas
                    self.last_read_ts = t0
            except Exception:
                self.read_failures += 1
                q_meas = None
            with self._lock:
                static = t0 - self._ref_changed_ts >= self.STATIC_AFTER_S
                if q_meas is not None and self.integrate and static:
                    err = self._q_ref - q_meas
                    if float(np.max(np.abs(err))) < t.sag_integral_active_rad:
                        err = np.sign(err) * np.maximum(np.abs(err) - t.sag_integral_deadband_rad, 0.0)
                        self._integral = np.clip(self._integral + t.sag_integral_gain * err * step_dt,
                                                 -s.max_sag_compensation_rad, s.max_sag_compensation_rad)
                target = self._q_ref + self._integral
                delta = target - self._q_cmd
                biggest = float(np.max(np.abs(delta)))
                cap = speed * step_dt
                if biggest > cap:
                    delta *= cap / biggest
                self._q_cmd = self._q_cmd + delta
                q_cmd = self._q_cmd.copy()
                if self._waist_target is not None and self._waist_cmd is not None:
                    step = t.waist_yaw_speed_rad_s * step_dt
                    self._waist_cmd += max(-step, min(step, self._waist_target - self._waist_cmd))
                waist = self._waist_cmd
            targets = {int(j): float(q_cmd[i]) for i, j in enumerate(self.joints)}
            if waist is None:
                self.robot._arm_sdk_client().write(targets, kp=float(t.arm_kp), kd=float(kd))
            else:
                # Waist gains as sdk_wrapper.engage_arms holds the waist.
                targets[WAIST_YAW_JOINT] = float(waist)
                self.robot._arm_sdk_client().write(targets, kp=float(t.arm_kp), kd=float(kd),
                                                   waist_kp={WAIST_YAW_JOINT: WAIST_KP}, waist_kd={WAIST_YAW_JOINT: WAIST_KD})
            remaining = dt - (time.time() - t0)
            if remaining > 0:
                self._stop.wait(remaining)


# ---------------------------------------------------------------------------
# Dex3 grip
# ---------------------------------------------------------------------------

@dataclass
class GripResult:
    evidence: str                 # "tactile" | "kinematic" | "none" | "jam" | "no_feedback" | "cancelled"
    verified: bool
    closure: float = 0.0
    blocked: List[str] = field(default_factory=list)
    contact_sensors: List[int] = field(default_factory=list)
    max_delta_raw: float = 0.0
    hold_target: List[float] = field(default_factory=list)
    detail: str = ""
    # Joints that never moved while closing (stale motor suspected).
    suspects: List[str] = field(default_factory=list)


def _hand_serialized(method: Callable[..., Any]) -> Callable[..., Any]:
    """Dex3Grip command sequences run one at a time (Dex3Grip.cmd_lock)."""
    @functools.wraps(method)
    def wrapper(self: "Dex3Grip", *args: Any, **kwargs: Any) -> Any:
        with self.cmd_lock:
            return method(self, *args, **kwargs)
    return wrapper


class Dex3Grip:
    """Incremental close with per-joint blockage and per-element tactile
    deltas; a hold stream keeps the grip commanded while carrying."""

    def __init__(self, robot: Any, side: str, cfg: SortV2Config, diag: Callable[..., None]) -> None:
        self.robot = robot
        self.side = side
        self.cfg = cfg
        self.diag = diag
        self.open_t = list(HAND_SAFE_OPEN[side])
        self.closed_t = list(HAND_CLOSED[side])
        self.movable = [i for i in range(len(HAND_JOINT_NAMES)) if abs(self.closed_t[i] - self.open_t[i]) > 1e-3]
        self.baseline: Optional[List[Optional[np.ndarray]]] = None
        self.baseline_ts = 0.0
        self.last_grip: Optional[GripResult] = None
        self._hold_cmd: Optional[List[float]] = None
        self._hold_thread: Optional[threading.Thread] = None
        self._hold_stop = threading.Event()
        # Structured event sink (kind, message); the pipeline forwards it to the sort log.
        self.event: Callable[[str, str], None] = lambda kind, message: None
        # One hand command sequence at a time (sort, manual buttons, recovery):
        # two threads streaming different targets make the fingers stutter.
        self.cmd_lock = threading.RLock()

    # -- sensing ----------------------------------------------------------
    def snapshot(self, max_age_s: float = 0.5) -> Optional[Dict[str, Any]]:
        try:
            snap = self.robot.get_dex3_hand_sensors(self.side)
        except Exception:
            return None
        if not isinstance(snap, dict) or "error" in snap or snap.get("timestamp") is None:
            return None
        if time.time() - float(snap["timestamp"]) > max_age_s:
            return None
        try:
            snap["q"] = [float(snap["positions"][n]) for n in HAND_JOINT_NAMES]
            snap["dq"] = [float(snap["velocities"][n]) for n in HAND_JOINT_NAMES]
        except (KeyError, TypeError, ValueError):
            return None
        return snap

    def _pressures(self, snap: Dict[str, Any]) -> Optional[List[np.ndarray]]:
        raw = snap.get("tactile_pressures") or []
        return [np.asarray(s, dtype=np.float64) for s in raw] if raw else None

    def record_baseline(self, duration_s: float) -> Optional[List[Optional[np.ndarray]]]:
        """Per-element mean over `duration_s`; invalid elements are NaN."""
        samples: List[List[np.ndarray]] = []
        deadline = time.time() + max(0.05, duration_s)
        while time.time() < deadline:
            snap = self.snapshot()
            pressures = None if snap is None else self._pressures(snap)
            if pressures:
                samples.append(pressures)
            time.sleep(0.03)
        if not samples:
            self.baseline = None
            return None
        invalid = self.cfg.hardware.tactile_invalid_raw
        baseline: List[Optional[np.ndarray]] = []
        for i in range(len(samples[-1])):
            stack = np.array([s[i] for s in samples if i < len(s) and len(s[i]) == len(samples[-1][i])])
            if stack.size == 0:
                baseline.append(None)
                continue
            mean = stack.mean(axis=0)
            mean[np.any(np.abs(stack - invalid) < 0.5, axis=0)] = np.nan
            baseline.append(mean)
        self.baseline = baseline if any(b is not None and np.isfinite(b).any() for b in baseline) else None
        self.baseline_ts = time.time() if self.baseline is not None else 0.0
        return self.baseline

    def tactile_deltas(self, snap: Optional[Dict[str, Any]]) -> Optional[List[float]]:
        """Per-sensor max increase over the baseline (None = unavailable)."""
        if snap is None or self.baseline is None:
            return None
        pressures = self._pressures(snap)
        if not pressures:
            return None
        invalid = self.cfg.hardware.tactile_invalid_raw
        out = []
        for i, base in enumerate(self.baseline):
            if base is None or i >= len(pressures) or len(pressures[i]) != len(base):
                out.append(0.0)
                continue
            cur = pressures[i]
            ok = np.isfinite(base) & (np.abs(cur - invalid) >= 0.5)
            out.append(float(np.max(cur[ok] - base[ok])) if ok.any() else 0.0)
        return out

    def closure_of(self, j: int, q: float) -> float:
        o, c = self.open_t[j], self.closed_t[j]
        return (q - o) / (c - o) if abs(c - o) > 1e-6 else 0.0

    # -- commands ----------------------------------------------------------
    def _stream(self, cmd: List[float], duration_s: float) -> None:
        t = self.cfg.tuning
        deadline = time.time() + max(0.0, duration_s)
        while True:
            self.robot.write_dex3_pose(self.side, cmd, kp=t.finger_kp, kd=t.finger_kd, tau=t.finger_tau)
            if time.time() >= deadline:
                return
            time.sleep(0.02)

    def start_hold(self, cmd: List[float]) -> None:
        self.stop_hold()
        self._hold_cmd = list(cmd)
        self._hold_stop.clear()

        def loop() -> None:
            t = self.cfg.tuning
            while not self._hold_stop.is_set():
                self.robot.write_dex3_pose(self.side, self._hold_cmd, kp=t.finger_kp, kd=t.finger_kd, tau=t.finger_tau)
                self._hold_stop.wait(0.05)

        self._hold_thread = threading.Thread(target=loop, name=f"sort-v2-grip-{self.side}", daemon=True)
        self._hold_thread.start()

    def stop_hold(self) -> None:
        self._hold_stop.set()
        thread = self._hold_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._hold_thread = None

    @property
    def holding(self) -> bool:
        return self._hold_thread is not None and self._hold_thread.is_alive()

    def _staged(self, first: List[str], target: List[float], duration_s: float, kp: float, kd: float):
        """Move the `first` joints to `target` while the others hold, then
        everything: from a closed fist the thumb otherwise swings into the
        curled fingers (and, closing, folds under fingers already closed) and
        stalls (thumb_1 jams seen on the robot).  SDK stall protection stays on."""
        snap = self.snapshot()
        current = list(snap["q"]) if snap is not None else list(target)
        stage = [target[i] if HAND_JOINT_NAMES[i] in first else current[i] for i in range(len(target))]
        half = max(0.2, duration_s / 2.0)
        result = self.robot.hand_pose(stage, hand=self.side, hold_s=half + 0.2, rate_hz=50.0, kp=kp, kd=kd, tau=0.0,
                                      ramp_s=half)
        if isinstance(result, dict) and result.get("reason") == "collision_or_stall":
            return result
        return self.robot.hand_pose(target, hand=self.side, hold_s=half + 0.3, rate_hz=50.0, kp=kp, kd=kd, tau=0.0,
                                    ramp_s=half)

    # -- health / stale fingers ------------------------------------------------
    def motor_health(self, snap: Optional[Dict[str, Any]]) -> Tuple[List[str], List[str]]:
        """(faults, warnings): motors at the temperature limit are faults;
        warm motors, non-zero motor status words and hand errors are
        warnings (reported with every finger failure).  Empty when the
        state carries no health fields."""
        hw = self.cfg.hardware
        faults: List[str] = []
        warns: List[str] = []
        if snap is None:
            return faults, warns
        if any(snap.get("hand_error") or []):
            warns.append(f"hand error {snap['hand_error']}")
        temps = snap.get("motor_temperatures") or {}
        states = snap.get("motor_states") or {}
        for name in HAND_JOINT_NAMES:
            temp = temps.get(name)
            if temp is not None and temp >= hw.dex3_temp_limit_c:
                faults.append(f"{name} {temp:.0f} C")
            elif temp is not None and temp >= hw.dex3_temp_warn_c:
                warns.append(f"{name} {temp:.0f} C")
            if states.get(name):
                warns.append(f"{name} motor state {states[name]}")
        hottest = max([v for v in temps.values() if v is not None], default=None)
        if hottest is not None:
            self.diag(finger_max_temperature_c=hottest)
        return faults, warns

    def stale_joints(self, q_before: Sequence[float], target: Sequence[float], q_after: Sequence[float]) -> List[int]:
        """Joints commanded a clear distance that did not move at all."""
        t = self.cfg.tuning
        return [j for j in self.movable if abs(target[j] - q_before[j]) >= t.finger_stale_min_command_rad
                and abs(q_after[j] - q_before[j]) < t.finger_stale_max_motion_rad]

    def _reengage(self, duration_s: float = 0.3) -> None:
        """Position control at the measured pose (clears a release/timeout mode)."""
        snap = self.snapshot()
        if snap is not None:
            self._stream(list(snap["q"]), duration_s)

    def _unstick(self, step: str, joints: List[int], target: Sequence[float]) -> None:
        if step == "re-engaged":
            self._reengage()
        elif step == "released and re-engaged":
            self.release()
            time.sleep(0.5)
            self._reengage()
        elif step == "wiggled":
            snap = self.snapshot()
            if snap is None:
                return
            cmd = list(snap["q"])
            for j in joints:
                cmd[j] = snap["q"][j] - math.copysign(0.15, target[j] - snap["q"][j])
            self._stream(cmd, 0.4)
            self._reengage()

    @_hand_serialized
    def _move_verified(self, target: List[float], orders: Tuple[Tuple[str, ...], Tuple[str, ...]], duration_s: float,
                       what: str, before_retry: Optional[Callable[[], Any]] = None) -> Tuple[bool, str]:
        """Staged move to `target`, verified joint by joint.  A stalled or
        stale finger (a motor that does not follow at all) is recovered step
        by step: re-engaged and moved in the other order, released (limp) and
        re-engaged and moved slowly, wiggled away from the target and back.
        Fails, naming the joints and the motor health, only when none helps."""
        self.stop_hold()
        tol = self.cfg.tuning.final_hand_tolerance_rad
        steps = (("", orders[0], duration_s), ("re-engaged", orders[1], duration_s),
                 ("released and re-engaged", orders[0], 2.0 * duration_s), ("wiggled", orders[1], 2.0 * duration_s))
        tries: List[str] = []
        problem: List[int] = []
        for k, (step, first, dur) in enumerate(steps):
            if k:
                self._unstick(step, problem, target)
                if before_retry is not None:
                    before_retry()
            before = self.snapshot()
            if before is None:
                return False, f"{what}: no Dex3 feedback"
            result = self._staged(list(first), list(target), dur, kp=1.0, kd=0.1)
            after = self.snapshot()
            if after is None:
                return False, f"{what}: no Dex3 feedback after the move"
            errors = [abs(a - b) for a, b in zip(after["q"], target)]
            self.diag(finger_positions=after["q"], finger_targets=list(target))
            stalled = isinstance(result, dict) and result.get("reason") == "collision_or_stall"
            stale = self.stale_joints(before["q"], target, after["q"])
            if stalled:
                # A stall can end the move in its first stage, before the other
                # joints were commanded: judge only the joints that were.
                stale = [j for j in stale if HAND_JOINT_NAMES[j] in first]
            off = [j for j in self.movable if errors[j] > tol]
            label = f"after {step} " if step else ""
            if not stalled and not off and not stale:
                if tries:
                    self.event("warn", f"{what}: fingers recovered {label.strip()} ({'; '.join(tries)})")
                return True, f"{what} ok (max error {max(errors):.2f} rad)" + (f" {label.strip()}" if tries else "")
            problem = stale or off or ([HAND_JOINT_NAMES.index(result["joint"])]
                                       if stalled and result.get("joint") in HAND_JOINT_NAMES else [])
            names = ", ".join(HAND_JOINT_NAMES[j] for j in problem) or "?"
            kind = "stale (did not move)" if stale else "stalled" if stalled else "off target"
            tries.append(f"{label}{names} {kind}, max error {max(errors):.2f} rad")
            self.event("warn", f"{what}: {names} {kind}" + (f" {label.strip()}" if step else "")
                       + ("; recovering" if k + 1 < len(steps) else ""))
        faults, warns = self.motor_health(self.snapshot())
        health = "; motor health: " + ", ".join(faults + warns) if faults or warns else "; motor health reports no fault"
        return False, f"{what} failed after recovery ({'; '.join(tries)}){health}"

    def open(self, duration_s: float) -> Tuple[bool, str]:
        """Open to the safe open pose, fingers before the thumb, verified,
        with the stale-finger recovery."""
        return self._move_verified(list(self.open_t), (FINGER_JOINTS, THUMB_JOINTS), duration_s, "opening")

    def close_neutral(self) -> Tuple[bool, str]:
        """Neutral closed pose, thumb before the fingers, verified.  Retries
        open the fingers first (thumb-first can block a finger under it)."""
        return self._move_verified(
            list(self.closed_t), (THUMB_JOINTS, FINGER_JOINTS), 1.0, "neutral close",
            before_retry=lambda: self._staged(list(FINGER_JOINTS), list(self.open_t), 1.0, kp=1.0, kd=0.1))

    @_hand_serialized
    def cool_down(self, cancel: threading.Event) -> Tuple[bool, str]:
        """Motors at the temperature limit: fingers limp (open) until all are
        below the warning temperature, at most finger_cooldown_max_s."""
        hw, t = self.cfg.hardware, self.cfg.tuning
        faults, _w = self.motor_health(self.snapshot())
        if not faults:
            return True, ""
        self.event("warn", f"Dex3 motors hot ({', '.join(faults)}): fingers limp to cool down "
                           f"(up to {t.finger_cooldown_max_s:.0f} s)")
        self.release()
        deadline = time.time() + t.finger_cooldown_max_s
        while time.time() < deadline and not cancel.is_set():
            cancel.wait(2.0)
            snap = self.snapshot()
            temps = [v for v in ((snap or {}).get("motor_temperatures") or {}).values() if v is not None]
            if temps and max(temps) < hw.dex3_temp_warn_c:
                self.event("info", f"Dex3 motors cooled to {max(temps):.0f} C")
                return True, ""
        faults, _w = self.motor_health(self.snapshot())
        return (not faults), ("" if not faults else f"Dex3 motors still hot after cooling: {', '.join(faults)}")

    @_hand_serialized
    def finger_check(self, cancel: threading.Event) -> Tuple[bool, str]:
        """Free-space check before a grasp: cool down if hot, close part way
        and open again, every joint verified (stale-finger recovery
        included)."""
        ok, detail = self.cool_down(cancel)
        if not ok:
            return False, detail
        frac = self.cfg.tuning.finger_check_closure
        partial = [o + frac * (c - o) for o, c in zip(self.open_t, self.closed_t)]
        ok, detail = self._move_verified(partial, (THUMB_JOINTS, FINGER_JOINTS), 0.8, "finger check close")
        if not ok:
            return False, detail
        ok, detail = self.open(0.8)
        if not ok:
            return False, detail
        return True, "fingers ok"

    def release(self) -> None:
        self.stop_hold()
        try:
            self.robot.release_dex3_fingers(hand=self.side, hold_s=0.2, rate_hz=50.0, persistent=False)
        except Exception:
            pass

    # -- grasp ------------------------------------------------------------
    def _evidence(self, snap: Dict[str, Any], frozen: Dict[int, float],
                  exclude: Sequence[int] = ()) -> Tuple[bool, bool, List[int], float]:
        """(tactile ok, kinematic ok, contact sensors, max delta); `exclude`:
        joints that never moved (stale suspects), no kinematic evidence."""
        t = self.cfg.tuning
        deltas = self.tactile_deltas(snap)
        contacts = [] if deltas is None else [i for i, d in enumerate(deltas) if d >= t.tactile_contact_delta_raw]
        max_delta = 0.0 if not deltas else max(deltas)
        tactile = deltas is not None and (len(contacts) >= t.tactile_min_contact_sensors
                                          or max_delta >= t.tactile_firm_delta_raw)
        blocked_names = {HAND_JOINT_NAMES[j] for j, hold in frozen.items()
                         if abs(hold - snap["q"][j]) >= t.held_min_block_rad and j not in exclude}
        kinematic = any(n in blocked_names for n in THUMB_JOINTS) and any(n in blocked_names for n in FINGER_JOINTS)
        return tactile, kinematic, contacts, max_delta

    @_hand_serialized
    def close(self, cancel: threading.Event) -> GripResult:
        t, hw, s = self.cfg.tuning, self.cfg.hardware, self.cfg.safety
        snap = self.snapshot()
        if snap is None:
            return GripResult("no_feedback", False, detail="no fresh Dex3 state")
        # Keep a baseline taken moments ago with the open hand clear of the
        # object (at the pre-grasp): here a sensor may already touch it.
        if self.baseline is None or time.time() - self.baseline_ts > GRIP_BASELINE_MAX_AGE_S:
            self.record_baseline(t.tactile_baseline_s)
        tactile_available = self.baseline is not None
        q0 = list(snap["q"])
        cmd = list(q0)
        frozen: Dict[int, float] = {}
        near_open: List[str] = []
        blocked_ticks: Dict[int, int] = defaultdict(int)
        fraction = max(0.0, min(1.0, max(self.closure_of(j, q0[j]) for j in self.movable)))
        tactile = kinematic = False
        contacts: List[int] = []
        max_delta = 0.0
        while True:
            if cancel.is_set():
                self.start_hold(cmd)
                return GripResult("cancelled", False, detail="operator stop during closing", hold_target=cmd)
            fraction = min(1.0, fraction + t.finger_close_step)
            for j in self.movable:
                if j not in frozen:
                    cmd[j] = self.open_t[j] + fraction * (self.closed_t[j] - self.open_t[j])
            self._stream(cmd, t.finger_step_s)
            snap = self.snapshot()
            if snap is None:
                self.start_hold(cmd)
                return GripResult("no_feedback", False, closure=fraction, detail="Dex3 state lost while closing")
            for j in self.movable:
                if j in frozen:
                    continue
                err = abs(cmd[j] - snap["q"][j])
                stalled = err >= hw.dex3_stall_error_rad and abs(snap["dq"][j]) <= hw.dex3_stall_speed_rad_s
                blocked_ticks[j] = blocked_ticks[j] + 1 if stalled else 0
                if blocked_ticks[j] >= 2:
                    if self.closure_of(j, snap["q"][j]) < t.finger_min_contact_closure:
                        # Blocked right at the open pose: the object touches this
                        # finger first, or it is jammed.  Hold it and keep closing
                        # the others; it is a jam only if no grip results.
                        near_open.append(HAND_JOINT_NAMES[j])
                    squeeze = max(-s.max_finger_squeeze_rad, min(s.max_finger_squeeze_rad,
                                                                 min(t.finger_squeeze_rad, abs(self.closed_t[j] - snap["q"][j]))
                                                                 * math.copysign(1.0, self.closed_t[j] - snap["q"][j])))
                    frozen[j] = snap["q"][j] + squeeze
                    cmd[j] = frozen[j]
            tactile, kinematic, contacts, max_delta = self._evidence(snap, frozen)
            self.diag(finger_positions=snap["q"], finger_targets=list(cmd), grip_fraction=fraction,
                      tactile_deltas=self.tactile_deltas(snap), grasp_status="closing")
            if tactile and (kinematic or fraction >= 1.0):
                # Enough contact: stop every finger where it is (plus the
                # bounded squeeze) instead of pushing further.
                for j in self.movable:
                    if j not in frozen:
                        d = self.closed_t[j] - snap["q"][j]
                        frozen[j] = snap["q"][j] + math.copysign(min(t.finger_squeeze_rad / 2.0, abs(d)), d)
                        cmd[j] = frozen[j]
                break
            free = [j for j in self.movable if j not in frozen]
            if not free:
                break
            if fraction >= 1.0 and all(abs(cmd[j] - snap["q"][j]) < hw.dex3_stall_error_rad for j in free):
                break
        # A joint frozen without having moved at all may be a stale motor, not
        # a finger on the object: it is no kinematic evidence.
        snap_now = self.snapshot() or snap
        suspects = [j for j in frozen if abs(snap_now["q"][j] - q0[j]) < t.finger_stale_max_motion_rad]
        if suspects:
            # Stale motor or a finger touching the object right at the open
            # pose (the bottle against the fingers as the palm arrives)?  A
            # live motor follows a small command back toward open.
            alive = self._probe_alive(cmd, suspects)
            if alive:
                self.event("info", f"closing: {', '.join(HAND_JOINT_NAMES[j] for j in alive)} did not move but "
                                   "respond: touching the object at the open pose")
            suspects = [j for j in suspects if j not in alive]
        if suspects:
            self.event("warn", f"closing: {', '.join(HAND_JOINT_NAMES[j] for j in suspects)} did not move and do not "
                               "respond (stale motor): not counted as grip evidence")
        # Confirmation: the evidence has to persist while the grip is held.
        self.start_hold(cmd)
        confirm_deadline = time.time() + t.grip_confirm_s
        held_tactile = held_kinematic = True
        while time.time() < confirm_deadline:
            time.sleep(0.05)
            snap = self.snapshot()
            if snap is None:
                return GripResult("no_feedback", False, closure=fraction, hold_target=cmd,
                                  detail="Dex3 state lost while confirming the grip")
            tac, kin, contacts, max_delta = self._evidence(snap, frozen, suspects)
            held_tactile &= tac
            held_kinematic &= kin
        blocked = [HAND_JOINT_NAMES[j] for j in frozen]
        if held_tactile:
            evidence = "tactile"
        elif held_kinematic:
            evidence = "kinematic"
        elif near_open:
            self.release()
            return GripResult("jam", False, closure=fraction, blocked=near_open,
                              detail=f"{', '.join(near_open)} blocked near the open pose and no grip formed (jam); "
                                     "fingers released")
        else:
            evidence = "none"
        detail = (f"{evidence} evidence; blocked: {', '.join(blocked) or 'none'}; contact sensors: "
                  f"{contacts or 'none'} (max delta {max_delta:.0f} raw"
                  + ("" if tactile_available else ", tactile unavailable") + ")")
        result = GripResult(evidence, evidence in ("tactile", "kinematic"), closure=fraction, blocked=blocked,
                            contact_sensors=contacts, max_delta_raw=max_delta, hold_target=list(cmd), detail=detail,
                            suspects=[HAND_JOINT_NAMES[j] for j in suspects])
        self.last_grip = result
        self._frozen = dict(frozen)
        self._suspects = list(suspects)
        self.diag(grasp_status=detail)
        return result

    def load_reference(self) -> Optional[Dict[str, Any]]:
        """Finger positions and tactile deltas now, to tell later whether the
        load on the grip changed (the table taking the object's weight)."""
        snap = self.snapshot()
        if snap is None:
            return None
        return {"q": list(snap["q"]), "tactile": self.tactile_deltas(snap)}

    def load_change(self, ref: Optional[Dict[str, Any]]) -> str:
        """Why the grip load changed since `ref` ("" if not): a held finger
        moved (the object slipping or pushed up in the hand) or a contact
        sensor's reading changed clearly."""
        snap = self.snapshot()
        if ref is None or snap is None:
            return ""
        t = self.cfg.tuning
        for j in getattr(self, "_frozen", {}):
            moved = abs(snap["q"][j] - ref["q"][j])
            if moved >= t.touchdown_finger_shift_rad:
                return f"{HAND_JOINT_NAMES[j]} shifted {moved:.2f} rad (object pushed in the hand)"
        now, before = self.tactile_deltas(snap), ref.get("tactile")
        if now and before:
            for i, (a, b) in enumerate(zip(now, before)):
                if max(a, b) >= t.tactile_contact_delta_raw and \
                        abs(a - b) >= max(t.touchdown_tactile_change_raw, 0.4 * max(a, b)):
                    return f"contact sensor {i} changed {b:.0f} -> {a:.0f} raw"
        return ""

    def _probe_alive(self, cmd: List[float], joints: List[int]) -> List[int]:
        """Of `joints`, those whose motor follows a small command back toward
        open (then the grip command is restored)."""
        t = self.cfg.tuning
        before = self.snapshot()
        if before is None:
            return []
        probe = list(cmd)
        for j in joints:
            probe[j] = before["q"][j] - math.copysign(t.finger_probe_rad, self.closed_t[j] - self.open_t[j])
        self._stream(probe, 0.3)
        after = self.snapshot()
        self._stream(list(cmd), 0.3)
        if after is None:
            return []
        return [j for j in joints if abs(after["q"][j] - before["q"][j]) >= t.finger_stale_max_motion_rad * 0.8]

    def retained(self) -> Tuple[Optional[bool], str]:
        """Is the object still in the hand?  None when it cannot be told."""
        grip = self.last_grip
        snap = self.snapshot()
        if grip is None or snap is None:
            return None, "no grip / no Dex3 feedback"
        tac, kin, contacts, max_delta = self._evidence(snap, getattr(self, "_frozen", {}), getattr(self, "_suspects", ()))
        self.diag(finger_positions=snap["q"], tactile_deltas=self.tactile_deltas(snap))
        if grip.evidence == "tactile":
            if tac or kin:
                return True, f"tactile contacts {contacts}" + (" + blocked fingers" if kin else "")
            return False, f"tactile contact lost (max delta {max_delta:.0f} raw) and fingers not blocked"
        if kin:
            return True, "fingers still blocked by the object"
        return False, "fingers closed past the contact point (object slipped)"


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

class GraspPipelineV2(GraspPipeline):
    """GraspPipeline perception + V2 control.  V1 motion methods remain for
    the chat agent; any of them stops the V2 controller before publishing
    (stop_active_hold), so the arm always has exactly one controller."""

    def __init__(self, robot: Any, rgbd_endpoint: str, cfg: Optional[SortV2Config] = None, arm: str = "right",
                 **kwargs: Any) -> None:
        super().__init__(robot, rgbd_endpoint, **kwargs)
        self.cfg = cfg or default_config()
        self.sort_arm = arm
        self.v2_active = False
        self._diag: Dict[str, Any] = {}
        self._diag_lock = threading.Lock()
        self._build_arm(arm)
        # Chat hand moves on the other arm (see arm_motion).
        self._other_arms: Dict[str, "SecondArmMotion"] = {}
        # Structured event sink (kind, message); SorterV2 points it at its log.
        self.event: Callable[[str, str], None] = lambda kind, message: self.log(f"[sort-v2] {kind}: {message}")
        # Looked up at call time: SorterV2 re-points self.event at its log.
        self.grip.event = lambda kind, message: self.event(kind, message)

    def _build_arm(self, arm: str) -> None:
        """Everything a validated move needs for one arm."""
        self.controller = ArmController(self.robot, arm, self.cfg, self.read_current_arm_q)
        self.validator = MotionValidator(arm, self.cfg, self._opposite_wrist)
        self.grip = Dex3Grip(self.robot, arm, self.cfg, self.diag)
        self.fk = ArmFK(arm=arm, backend="urdf")
        n = self.cfg.tuning.ik_max_iter
        # Level-palm tiers: precise, normal, and the bounded reach-edge residual.
        self.ik_tiers = [
            ("fine", ArmIK(arm=arm, solver="level_dls", max_iter=n, tol_pos_m=gp1.GRASP_FINE_IK_TOL_POS_M,
                           tol_rot_rad=gp1.GRASP_FINE_IK_TOL_ROT_RAD)),
            ("loose", ArmIK(arm=arm, solver="level_dls", max_iter=n, tol_pos_m=0.008,
                            tol_rot_rad=gp1.GRASP_PALM_LEVEL_TOL_RAD)),
            ("reach-edge", ArmIK(arm=arm, solver="level_dls", max_iter=n, tol_pos_m=self.cfg.tuning.ik_reach_tolerance_m,
                                 tol_rot_rad=gp1.GRASP_PALM_LEVEL_TOL_RAD)),
        ]
        # Full 6D target (position, level palm, finger direction).  Its
        # position-only fallback is never accepted (see solve_pose).
        self.ik_6d = ArmIK(arm=arm, solver="dls", max_iter=n, tol_pos_m=0.004, tol_rot_rad=math.radians(2.0))
        # Orientation-free rises (from the hanging arm): minimal joint change.
        self.ik_position = ArmIK(arm=arm, solver="position_dls", max_iter=n, tol_pos_m=0.004, tol_rot_rad=math.pi)
        self.last_ik_detail = ""

    def arm_motion(self, side: str) -> "GraspPipelineV2":
        """The object whose move_linear / move_joint / grip drive this arm."""
        if side == self.sort_arm:
            return self
        with self.lock:
            other = self._other_arms.get(side)
            if other is None:
                other = self._other_arms[side] = SecondArmMotion(self, side)
        return other

    # -- diagnostics -----------------------------------------------------------
    def diag(self, **values: Any) -> None:
        with self._diag_lock:
            self._diag.update(values)
            self._diag["ts"] = time.time()

    def diagnostics(self) -> Dict[str, Any]:
        with self._diag_lock:
            out = dict(self._diag)
        out["controller"] = self.controller.status()
        return out

    def clear_diag(self) -> None:
        with self._diag_lock:
            self._diag = {}

    # -- controller ownership --------------------------------------------------
    def stop_active_hold(self) -> None:
        super().stop_active_hold()
        if self.controller.running and not self.v2_active:
            self.controller.stop()
        if not self.v2_active:
            for other in self._other_arms.values():
                other.controller.stop()

    def busy(self) -> bool:
        return self.v2_active or super().busy()

    def _opposite_wrist(self) -> Optional[np.ndarray]:
        opposite = "left" if self.sort_arm == "right" else "right"
        T = self.hand_T(opposite)
        return None if T is None else T[:3, 3].copy()

    def measured_hand_T(self) -> np.ndarray:
        return self.fk.compute_arm(self.read_current_arm_q(self.sort_arm))

    def reference_hand_T(self) -> np.ndarray:
        return self.fk.compute_arm(self.controller.reference())

    # -- scene (overridden by the simulator) ----------------------------------
    def current_table(self) -> Optional[TablePlane]:
        with self.lock:
            cam = dict(self.camera_extrinsic)
        return self.estimate_obstacles(cam).table

    def obstacle_objects(self, T_base_object: np.ndarray, since_ts: float) -> List[TableObject]:
        return self.grab_object_obstacles(self.sort_arm, T_base_object, since_ts)

    def grasp_pose(self, det: gp1.Detection, T_base_object: np.ndarray, table: Optional[TablePlane]) -> np.ndarray:
        """V1 grasp construction (object top, robot-side approach, level
        palm) with the wrist floor taken from the hand model."""
        obstacles = Obstacles(table=table)
        T = direct_grasp_hand_pose(det, T_base_object, self.sort_arm, obstacles)
        T[:3, :3] = level_rotation(T[:3, :3])
        if table is not None:
            T[2, 3] = max(float(T[2, 3]), table.z_base_m + self.cfg.grasp_wrist_floor_m() + 0.005)
        return T

    def in_capture(self, T_hand: np.ndarray, p_object: np.ndarray) -> Tuple[bool, np.ndarray]:
        h = self.cfg.hand
        d = T_hand[:3, :3].T @ (np.asarray(p_object, dtype=np.float64) - T_hand[:3, 3])
        ok = (h.capture_forward_m[0] <= d[0] <= h.capture_forward_m[1] and abs(d[1]) <= h.capture_lateral_m
              and abs(d[2]) <= h.capture_vertical_m)
        return bool(ok), d

    # -- IK --------------------------------------------------------------------
    def solve_level(self, T: np.ndarray, q_init: np.ndarray) -> Optional[np.ndarray]:
        """Level palm, yaw free; last_ik_detail says which tier solved or why
        none did (best residual position / tilt)."""
        best = ""
        for name, ik in self.ik_tiers:
            q, info = ik.solve(T, q_init=q_init)
            if q is not None:
                self.last_ik_detail = f"{name} tier"
                return q
            best = f"residual {info['error_pos_m'] * 1000:.0f} mm / tilt {math.degrees(info['error_rot_rad']):.1f} deg"
        self.last_ik_detail = f"no level-palm solution ({best} after the {self.ik_tiers[-1][0]} tier)"
        return None

    def reach_residual_m(self, p: np.ndarray, q_seed: np.ndarray) -> float:
        """How far the palm link stays from point p at best (position-only
        IK, any orientation): 0 when reachable, else the shortfall."""
        T = np.eye(4)
        T[:3, 3] = np.asarray(p, dtype=np.float64)
        q, info = self.ik_position.solve(T, q_init=q_seed)
        return 0.0 if q is not None else float(info["error_pos_m"])

    def solve_step(self, T: np.ndarray, q_prev: np.ndarray, level: bool, yaw_free: bool,
                   yaw_tol: Optional[float] = None) -> Optional[np.ndarray]:
        """IK for one planner step; None also when the solution would jump
        configuration (SafetyThresholds.max_ik_step_jump_rad)."""
        if not level:
            q = self.ik_position.solve(T, q_init=q_prev)[0]
        elif yaw_free:
            q = self.solve_level(T, q_prev)
        else:
            q = self.solve_pose(T, q_prev, yaw_tol)
        if q is None:
            if not level:
                self.last_ik_detail = "no position-only solution"
            return None
        jump = float(np.max(np.abs(q - q_prev)))
        if jump > self.cfg.safety.max_ik_step_jump_rad:
            self.last_ik_detail = (f"configuration jump {jump:.2f} rad > {self.cfg.safety.max_ik_step_jump_rad:.2f} "
                                   "(IK switched branch)")
            return None
        return q

    def solve_pose(self, T: np.ndarray, q_init: np.ndarray, yaw_tol: Optional[float] = None) -> Optional[np.ndarray]:
        """The full 6D target: 6D IK first; at the reach edge, a level-palm
        solution whose finger direction is within yaw_tol (default
        grasp_yaw_tolerance_rad)."""
        tol = self.cfg.tuning.grasp_yaw_tolerance_rad if yaw_tol is None else yaw_tol
        q, info = self.ik_6d.solve(T, q_init=q_init)
        if q is not None and not info.get("fallback"):
            self.last_ik_detail = "6D"
            return q
        q = self.solve_level(T, q_init)
        if q is None:
            return None
        yaw = yaw_error(self.fk.compute_arm(q), T)
        if yaw > tol:
            self.last_ik_detail = (f"finger direction {math.degrees(yaw):.1f} deg off the planned yaw "
                                   f"(max {math.degrees(tol):.0f})")
            return None
        return q

    # -- moves -----------------------------------------------------------------
    def move_linear(self, T_goal: np.ndarray, ctx: MotionContext, speed_m_s: float, label: str,
                    cancel: threading.Event, tolerance_m: Optional[float] = None, max_tilt_rad: Optional[float] = None,
                    gate: Optional[Callable[[np.ndarray], bool]] = None,
                    monitor: Optional[Callable[[], Optional[Tuple[Failure, str]]]] = None,
                    level: bool = True, timeout_s: Optional[float] = None,
                    gate_reason: Optional[Callable[[np.ndarray], str]] = None, yaw_free: bool = False,
                    accept_short_m: Optional[float] = None, integrate: bool = True,
                    q_path: Optional[Sequence[np.ndarray]] = None, yaw_tol: Optional[float] = None) -> MoveResult:
        """Straight-line hand move from the current reference to T_goal.

        Each planner tick advances the reference one step along the line,
        solves IK for that point, and validates the segment from the previous
        reference (the predicted physical path).  A rejected or unsolvable
        step is retried, not skipped; the move fails after a bounded number of
        consecutive rejections / IK failures, when the measured error stops
        improving, or on timeout.  `gate` (measured hand pose -> bool) may end
        the move early once it holds for grasp_gate_ticks ticks.  With
        `yaw_free` (carrying, retracting) the palm stays level but the finger
        direction may turn; grasp moves keep the planned yaw.  With
        `accept_short_m` (transport, release, retreat: no precision needed) a
        move whose whole reference was sent but that settles that close to
        the goal (the loaded arm sagging beyond compensation) counts as reached.
        `integrate=False` keeps the sag integral frozen for the move (moves
        that end in contact: it would wind up pressing).  `q_path`: the joint
        path planned for this move (one configuration per step, ending at
        T_goal); its steps are followed instead of solving IK every tick (a
        stretched arm's per-tick IK wandered and the descent crawled to a
        stop).  Every step is still validated, the measured pose checked.
        """
        t = self.cfg.tuning
        tol = t.move_tolerance_m if tolerance_m is None else tolerance_m
        timeout = t.move_timeout_s if timeout_s is None else timeout_s
        rate = max(2.0, t.planner_rate_hz)
        dt = 1.0 / rate
        self.controller.integrate = integrate
        q_ref = self.controller.reference()
        T_start = self.fk.compute_arm(q_ref)
        distance = float(np.linalg.norm(T_goal[:3, 3] - T_start[:3, 3]))
        angle = palm_tilt(T_start, T_goal) if level else 0.0
        steps = max(1, int(math.ceil(max(distance / max(1e-3, speed_m_s * dt), angle / math.radians(3.0)))))
        path: Optional[List[np.ndarray]] = None
        if q_path:
            path = [np.asarray(q, dtype=np.float64) for q in q_path]
            end_err = float(np.linalg.norm(self.fk.compute_arm(path[-1])[:3, 3] - T_goal[:3, 3]))
            if end_err > 0.01:
                path = None            # not this move's path: solve per step
            else:
                # Planned at 5 mm; executed at the move's speed by streaming
                # interpolated references between the planned configurations.
                dense: List[np.ndarray] = []
                q_prev_p = q_ref
                per = max(1, int(round(steps / len(path))))
                for q_p in path:
                    dense.extend(q_prev_p + (q_p - q_prev_p) * (k / per) for k in range(1, per + 1))
                    q_prev_p = q_p
                path = dense
                steps = len(path)
        # A move that rises out of a pose already too low (hand sagged onto the
        # table, wrist under its floor) may run as long as it does not go lower
        # than where it started: the height checks would otherwise refuse
        # every step and the measured pose, trapping the hand there.
        rising = float(T_goal[2, 3]) > float(T_start[2, 3]) + 0.005
        try:
            z_low = min(float(T_start[2, 3]), float(self.fk.compute_arm(self.read_current_arm_q(self.sort_arm))[2, 3]))
        except Exception:
            z_low = float(T_start[2, 3])

        def escape(reasons: List[str], z: float) -> List[str]:
            if not rising or z < z_low - 0.005:
                return reasons
            return [r for r in reasons if not any(m in r for m in LOW_POSE_MARKERS)]

        index = 0
        rejections = ik_failures = 0
        last_reject = ""
        gate_ticks = 0
        best_err, best_t = float("inf"), time.time()
        last_lag = float("inf")
        settled_since: Optional[float] = None
        t0 = time.time()
        err = tilt = float("nan")
        self.diag(waypoint=label, desired_ee=[float(v) for v in T_goal[:3, 3]], collision="ok", move_progress=0.0,
                  move_elapsed_s=0.0, move_timeout_s=timeout)
        g, a = T_goal[:3, 3], T_start[:3, 3]
        self.event("info", f"{label}: {steps} EE poses from ({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f}) to "
                           f"({g[0]:+.3f}, {g[1]:+.3f}, {g[2]:+.3f}) m, {distance * 1000:.0f} mm / "
                           f"{math.degrees(angle):.0f} deg, ~{steps * dt:.1f} s"
                           + ("" if level else ", orientation free") + (", yaw free" if yaw_free else ""))
        last_report = time.time()

        def finish(r: MoveResult) -> MoveResult:
            self.event("info" if r.ok else "warn", r.detail + ("" if r.ok else f" [{r.failure.value}]"))
            return r

        while True:
            tick = time.time()
            if cancel.is_set():
                return finish(MoveResult(False, Failure.OPERATOR_STOP, f"{label}: operator stop", err, tilt))
            # Issue the next validated reference only once the command has
            # reached the previous one, so the command never cuts across
            # unvalidated joint space.
            if index < steps and self.controller.lag_rad() <= 0.02:
                if path is not None:
                    q_next = path[index]
                    T_next = self.fk.compute_arm(q_next)
                else:
                    T_next = interpolate_pose(T_start, T_goal, (index + 1) / steps)
                    q_next = self.solve_step(T_next, q_ref, level, yaw_free, yaw_tol)
                if q_next is None:
                    ik_failures += 1
                    self.diag(collision=f"IK failure x{ik_failures}")
                    pn = T_next[:3, 3]
                    self.event("warn", f"{label}: IK failed for pose {index + 1}/{steps} "
                                       f"({pn[0]:+.3f}, {pn[1]:+.3f}, {pn[2]:+.3f}) m: {self.last_ik_detail}")
                    if ik_failures >= t.max_consecutive_ik_failures:
                        p = T_next[:3, 3]
                        return finish(MoveResult(False, Failure.IK_FAILURE, f"{label}: no level-palm IK for "
                                          f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m after {ik_failures} tries", err, tilt))
                else:
                    reasons = escape(self.validator.check_segment(q_ref, q_next, ctx, T_next), float(T_next[2, 3]))
                    if reasons:
                        rejections += 1
                        last_reject = reasons[0]
                        self.diag(collision=f"rejected x{rejections}: {last_reject}")
                        self.event("warn", f"{label}: pose {index + 1}/{steps} rejected by the safety check: {last_reject}")
                        if rejections >= t.max_consecutive_rejections:
                            return finish(MoveResult(False, Failure.COLLISION_REJECTED,
                                              f"{label}: next step rejected {rejections}x in a row: {last_reject}",
                                              err, tilt, {"violations": reasons}))
                    else:
                        rejections = ik_failures = 0
                        q_ref = q_next
                        self.controller.set_reference(q_ref)
                        index += 1
                        self.diag(collision="ok")
            # Measured pose: progress, safety and the optional early gate.
            try:
                q_meas = self.read_current_arm_q(self.sort_arm)
            except Exception as exc:
                return finish(MoveResult(False, Failure.SENSOR_UNAVAILABLE, f"{label}: {exc}", err, tilt))
            T_meas = self.fk.compute_arm(q_meas)
            err = float(np.linalg.norm(T_goal[:3, 3] - T_meas[:3, 3]))
            tilt = palm_tilt(T_meas, T_goal)
            self.diag(measured_ee=[float(v) for v in T_meas[:3, 3]], ee_error_m=err, palm_tilt_rad=tilt,
                      move_progress=index / steps, move_elapsed_s=tick - t0)
            unsafe = escape(self.validator.pose_violations(T_meas, self.fk.compute_arm_partial(q_meas, 4)[:3, 3], ctx,
                                                    clearance_slack_m=0.01), float(T_meas[2, 3]))
            if unsafe:
                return finish(MoveResult(False, Failure.COLLISION_REJECTED, f"{label}: measured pose unsafe: {unsafe[0]}",
                                  err, tilt, {"violations": unsafe}))
            status = self.controller.status()
            if not status["running"]:
                return finish(MoveResult(False, Failure.INTERNAL, f"{label}: arm controller is not running "
                                  f"({status['last_error'] or 'stopped'})", err, tilt))
            if status["tracking_error_rad"] > self.cfg.safety.max_tracking_error_rad:
                return finish(MoveResult(False, Failure.TRACKING_FAILURE,
                                  f"{label}: arm not following (tracking error {status['tracking_error_rad']:.2f} rad)",
                                  err, tilt))
            if status["state_age_s"] > 0.5:
                return finish(MoveResult(False, Failure.SENSOR_UNAVAILABLE, f"{label}: joint state stale", err, tilt))
            if monitor is not None:
                problem = monitor()
                if problem is not None:
                    return finish(MoveResult(False, problem[0], f"{label}: {problem[1]}", err, tilt))
            if tick - last_report >= 2.0:
                last_report = tick
                self.event("info", f"{label}: pose {index}/{steps}, EE error {err * 1000:.0f} mm, tilt "
                                   f"{math.degrees(tilt):.1f} deg, command lag {self.controller.lag_rad():.3f} rad, "
                                   f"tracking {status['tracking_error_rad']:.3f} rad, sag comp "
                                   f"{status['sag_compensation_rad']:.3f} rad, rejections {rejections}, "
                                   f"IK failures {ik_failures}")
                if gate is not None and gate_reason is not None and index >= steps:
                    why = gate_reason(T_meas)
                    if why:       # empty = satisfied, waiting for the confirming tick
                        self.event("info", f"{label}: grasp gate not yet satisfied: {why}")
            if gate is not None:
                gate_ticks = gate_ticks + 1 if gate(T_meas) else 0
                if gate_ticks >= t.grasp_gate_ticks:
                    return finish(MoveResult(True, None, f"{label}: grasp gate satisfied", err, tilt))
            tilt_ok = max_tilt_rad is None or tilt <= max_tilt_rad
            if index >= steps and err <= tol and tilt_ok:
                settled_since = settled_since or tick
                if gate is None and tick - settled_since >= t.move_settle_s:
                    return finish(MoveResult(True, None, f"{label}: reached (error {err * 1000:.0f} mm)", err, tilt))
            else:
                settled_since = None
            if gate is not None and settled_since is not None and tick - settled_since >= t.move_settle_s + 0.5:
                why = gate_reason(T_meas) if gate_reason is not None else "gate not satisfied"
                return finish(MoveResult(False, Failure.NOT_REACHED, f"{label}: arrived (error {err * 1000:.0f} mm) but {why}",
                                  err, tilt))
            lag = self.controller.lag_rad()
            if err < best_err - t.progress_min_m or (lag > 0.02 and lag < last_lag - 1e-3):
                # Measured progress, or the command still visibly catching up
                # to a validated reference (a slow controller tick is not a block).
                best_err, best_t = min(err, best_err), tick
            last_lag = lag
            if tick - best_t > t.progress_window_s:
                if index < steps:
                    why = (f"blocked at step {index}/{steps}" + (f" ({last_reject})" if last_reject else "")
                           + f"; IK failures {ik_failures}, rejections {rejections}, "
                             f"command lag {self.controller.lag_rad():.3f} rad, controller errors "
                             f"{status['loop_errors']} {status['last_error']}")
                    kind = (Failure.COLLISION_REJECTED if last_reject else
                            Failure.IK_FAILURE if ik_failures else Failure.NOT_REACHED)
                else:
                    sat = " (sag compensation saturated)" if status["sag_saturated"] else ""
                    if accept_short_m is not None and err <= accept_short_m and tilt_ok:
                        self.event("warn", f"{label}: settled {err * 1000:.0f} mm short{sat}; accepted "
                                           f"(within {accept_short_m * 1000:.0f} mm)")
                        return finish(MoveResult(True, None, f"{label}: reached within {err * 1000:.0f} mm{sat}",
                                                 err, tilt))
                    why = f"stopped {err * 1000:.0f} mm short{sat}"
                    kind = Failure.TRACKING_FAILURE if status["sag_saturated"] else Failure.NOT_REACHED
                return finish(MoveResult(False, kind, f"{label}: no progress for {t.progress_window_s:.0f} s, {why}", err, tilt))
            if tick - t0 > timeout:
                return finish(MoveResult(False, Failure.NOT_REACHED, f"{label}: timeout after {timeout:.0f} s "
                                  f"(error {err * 1000:.0f} mm)", err, tilt))
            remaining = dt - (time.time() - tick)
            if remaining > 0:
                cancel.wait(remaining)

    def touch_down(self, z_goal: float, ctx: MotionContext, speed_m_s: float, label: str,
                   cancel: threading.Event, load_cue: Optional[Callable[[], str]] = None) -> MoveResult:
        """Lower straight down toward z_goal and stop at the first resistance:
        once the reference is descending steadily, a measured descent below
        touchdown_stall_ratio of the commanded one over touchdown_window_s
        means the carried object rests on the table.  So does a change of the
        grip load (`load_cue`: a finger shifting or a tactile change, also
        when the object slides up in the hand instead of stopping it), and
        the measured wrist reaching z_goal ends it in any case (the object
        slid in the hand: seen on the cup).  The reference is then
        frozen at the measured pose, so the arm does not keep pressing (the
        sag integral would otherwise wind up against the table).  Reaching
        z_goal without resistance also ends it (ok, noted)."""
        t = self.cfg.tuning
        T_goal = self.reference_hand_T()
        z_start = float(T_goal[2, 3])
        T_goal[2, 3] = z_goal
        hist: List[Tuple[float, float, float]] = []      # (time, measured z, reference z)
        t0 = time.time()
        contact = {"why": ""}
        # Free-arm state before anything touches (relax() returns to it).
        self.controller.mark_free()
        # How far the hand trails above its command while descending freely,
        # taken once the descent is steady (first second).
        lag_free: Dict[str, Optional[float]] = {"lag": None}

        def gate(T_meas: np.ndarray) -> bool:
            now = time.time()
            z_ref = float(self.reference_hand_T()[2, 3])
            hist.append((now, float(T_meas[2, 3]), z_ref))
            while len(hist) > 2 and now - hist[1][0] >= t.touchdown_window_s:
                hist.pop(0)
            # Held up: the hand stays above its command by more than it trails
            # in free descent, also while the command itself is not moving (an
            # IK stall kept the stall test below from firing while the hand
            # was already pressed onto the cup).
            lag = float(T_meas[2, 3]) - z_ref
            if now - t0 < 1.0:
                lag_free["lag"] = lag
                return False
            held_up = lag - (lag_free["lag"] or 0.0)
            if z_start - z_ref >= 0.005 and held_up >= t.touchdown_push_m:
                contact["why"] = f"hand held {held_up * 1000:.0f} mm above its command at z={T_meas[2, 3]:.3f} m"
                return True
            span = hist[-1][0] - hist[0][0]
            if now - t0 < 1.0 or z_start - z_ref < 0.01 or span < 0.8 * t.touchdown_window_s:
                return False
            if float(T_meas[2, 3]) <= z_goal + 0.002:
                contact["why"] = f"measured wrist reached the set-down floor z={z_goal:.3f} m"
                return True
            if load_cue is not None and z_start - z_ref >= 0.005:
                why = load_cue()
                if why:
                    contact["why"] = f"grip load changed at z={T_meas[2, 3]:.3f} m: {why}"
                    return True
            descent = hist[0][1] - hist[-1][1]
            # What the reference actually did in the window (not the nominal
            # speed): a reference held back by the validator or at its goal is
            # no resistance.
            expected = hist[0][2] - hist[-1][2]
            # The command itself no longer descends although the target does:
            # the arm reaches no lower here (the reach-edge IK tier accepts the
            # residual).  Release here instead of failing on "no progress".
            if now - t0 >= 1.5 and z_ref > z_goal + 0.005 and expected < 0.2 * speed_m_s * span:
                contact["why"] = (f"the arm reaches no lower here (edge of its reach) at z={T_meas[2, 3]:.3f} m, "
                                  f"{(z_ref - z_goal) * 1000:.0f} mm above the set-down floor")
                return True
            if expected >= 0.5 * speed_m_s * span and descent < t.touchdown_stall_ratio * expected:
                contact["why"] = (f"resistance at z={T_meas[2, 3]:.3f} m: descended {descent * 1000:.1f} mm in "
                                  f"{span:.1f} s for {expected * 1000:.1f} mm commanded")
                return True
            return False

        # Sag integral frozen for the whole descent (it wound up to its cap
        # against the cup while IK stalled), finger direction free within the
        # level palm (a fixed yaw stalled the IK near the reach edge).
        r = self.move_linear(T_goal, ctx, speed_m_s, label, cancel, gate=gate,
                             gate_reason=lambda _T: "no resistance yet", timeout_s=30.0, yaw_free=True,
                             integrate=False)
        if r.ok and contact["why"]:
            r = MoveResult(True, None, f"{label}: {contact['why']}", r.error_m, r.tilt_rad)
        elif not r.ok and r.failure == Failure.NOT_REACHED and "arrived" in r.detail:
            r = MoveResult(True, None, f"{label}: reached z={z_goal:.3f} m without resistance", r.error_m, r.tilt_rad)
        elif not r.ok and r.failure in (Failure.NOT_REACHED, Failure.TRACKING_FAILURE) and "no progress" in r.detail:
            # The hand stopped going down (reach edge, or resting on something
            # the other cues missed): release here, the object is over the spot.
            z_now = float(self.measured_hand_T()[2, 3])
            r = MoveResult(True, None, f"{label}: the arm goes no lower at z={z_now:.3f} m "
                                       f"({(z_now - z_goal) * 1000:.0f} mm above the set-down floor); releasing here",
                           r.error_m, r.tilt_rad)
        # Hold where the hand is without pressing (also after a failure: the
        # recovery starts from a relaxed arm).
        self.controller.relax()
        if r.ok:
            self.event("info", r.detail)
        return r

    def waist_yaw(self) -> float:
        return self.read_joint_position(WAIST_YAW_JOINT)

    def turn_waist(self, target: float, cancel: threading.Event, label: str = "waist") -> MoveResult:
        """Turn the torso (waist yaw) to `target` through the arm controller
        (the only rt/arm_sdk publisher); the arm joints hold.  The camera
        turns with the torso, so everything measured before is stale."""
        t = self.cfg.tuning
        try:
            start = self.waist_yaw()
        except Exception as exc:
            return MoveResult(False, Failure.SENSOR_UNAVAILABLE, f"{label}: waist yaw not readable ({exc})")
        lim = self.cfg.safety.max_waist_yaw_rad
        target = max(-lim, min(lim, float(target)))
        self.controller.set_waist_yaw(target, start)
        self.event("info", f"{label}: waist yaw {math.degrees(start):+.1f} -> {math.degrees(target):+.1f} deg")
        deadline = time.time() + abs(target - start) / max(0.05, t.waist_yaw_speed_rad_s) + 4.0
        settled_since: Optional[float] = None
        while True:
            if cancel.is_set():
                try:
                    self.controller.set_waist_yaw(self.waist_yaw(), start)
                except Exception:
                    pass
                return MoveResult(False, Failure.OPERATOR_STOP, f"{label}: operator stop")
            try:
                yaw = self.waist_yaw()
            except Exception as exc:
                return MoveResult(False, Failure.SENSOR_UNAVAILABLE, f"{label}: waist yaw not readable ({exc})")
            cmd = self.controller.waist_command()
            if abs(yaw - target) <= 0.02 and cmd is not None and abs(cmd - target) < 1e-3:
                settled_since = settled_since or time.time()
                if time.time() - settled_since >= 0.3:
                    return MoveResult(True, None, f"{label}: waist at {math.degrees(yaw):+.1f} deg")
            else:
                settled_since = None
            if time.time() > deadline:
                return MoveResult(False, Failure.NOT_REACHED, f"{label}: waist stopped at {math.degrees(yaw):+.1f} deg "
                                  f"(target {math.degrees(target):+.1f})")
            cancel.wait(0.05)

    def move_joint(self, q_goal: np.ndarray, ctx: MotionContext, label: str, cancel: threading.Event,
                   tolerance_rad: Optional[float] = None, speed_rad_s: Optional[float] = None) -> MoveResult:
        """Joint-space move (validated per step) to a recorded configuration."""
        return self.move_joint_path([q_goal], ctx, label, cancel, tolerance_rad, speed_rad_s)

    def plan_vertical_path(self, q_from: np.ndarray, z_to: float, step_m: float = 0.01) -> Optional[List[np.ndarray]]:
        """Joint waypoints for a straight vertical hand move from FK(q_from)
        to height z_to, orientation free: position-only IK every step_m, each
        seeded from the previous one (one consistent branch, solved before
        moving).  None if a point has no solution."""
        T = self.fk.compute_arm(q_from)
        z0 = float(T[2, 3])
        n = max(1, int(math.ceil(abs(z_to - z0) / step_m)))
        path: List[np.ndarray] = []
        q = np.asarray(q_from, dtype=np.float64)
        for k in range(1, n + 1):
            T_k = T.copy()
            T_k[2, 3] = z0 + (z_to - z0) * (k / n)
            q_k, _info = self.ik_position.solve(T_k, q_init=q)
            if q_k is None or float(np.linalg.norm(self.fk.compute_arm(q_k)[:3, 3] - T_k[:3, 3])) > 0.006:
                return None
            path.append(q_k)
            q = q_k
        return path

    def move_joint_path(self, path: Sequence[np.ndarray], ctx: MotionContext, label: str, cancel: threading.Event,
                        tolerance_rad: Optional[float] = None, speed_rad_s: Optional[float] = None) -> MoveResult:
        """Joint-space move through `path` without stopping at its waypoints:
        every segment is validated first, then the reference is streamed at a
        constant joint speed (no IK while moving)."""
        t = self.cfg.tuning
        tol = t.prep_tolerance_rad if tolerance_rad is None else tolerance_rad
        speed = min(speed_rad_s or t.max_joint_speed_rad_s, self.cfg.safety.max_joint_speed_rad_s)
        dt = 1.0 / max(2.0, t.planner_rate_hz)
        self.controller.integrate = True
        q_prev = self.controller.reference()
        refs: List[np.ndarray] = []
        for q_next in path:
            q_next = np.asarray(q_next, dtype=np.float64)
            n = max(1, int(math.ceil(float(np.max(np.abs(q_next - q_prev))) / max(1e-3, speed * dt))))
            reasons = self.validator.check_segment(q_prev, q_next, ctx, samples=max(4, min(20, n)))
            if reasons:
                return MoveResult(False, Failure.COLLISION_REJECTED, f"{label}: path rejected: {reasons[0]}",
                                  data={"violations": reasons})
            refs.extend(q_prev + (q_next - q_prev) * (k / n) for k in range(1, n + 1))
            q_prev = q_next
        q_goal = q_prev
        steps = len(refs)
        self.diag(waypoint=label, desired_ee=[float(v) for v in self.fk.compute_arm(q_goal)[:3, 3]], collision="ok")
        t0 = time.time()
        best, best_t = float("inf"), t0
        index = 0
        settled_since: Optional[float] = None
        while True:
            tick = time.time()
            if cancel.is_set():
                return MoveResult(False, Failure.OPERATOR_STOP, f"{label}: operator stop")
            if index < steps:
                self.controller.set_reference(refs[index])
                index += 1
            q_meas = self.read_current_arm_q(self.sort_arm)
            err = float(np.max(np.abs(q_goal - q_meas)))
            T_meas = self.fk.compute_arm(q_meas)
            self.diag(measured_ee=[float(v) for v in T_meas[:3, 3]], joint_error_rad=err, move_progress=index / steps,
                      move_elapsed_s=tick - t0, move_timeout_s=t.move_timeout_s)
            status = self.controller.status()
            if status["tracking_error_rad"] > self.cfg.safety.max_tracking_error_rad and index >= steps:
                return MoveResult(False, Failure.TRACKING_FAILURE,
                                  f"{label}: arm not following (tracking error {status['tracking_error_rad']:.2f} rad)")
            if index >= steps and err <= tol:
                settled_since = settled_since or tick
                if tick - settled_since >= t.move_settle_s:
                    return MoveResult(True, None, f"{label}: reached (max joint error {err:.3f} rad)")
            else:
                settled_since = None
            if err < best - 0.01:
                best, best_t = err, tick
            elif tick - best_t > t.progress_window_s and index >= steps:
                return MoveResult(False, Failure.NOT_REACHED, f"{label}: stopped {err:.3f} rad from the goal")
            if tick - t0 > t.move_timeout_s:
                return MoveResult(False, Failure.NOT_REACHED, f"{label}: timeout")
            remaining = dt - (time.time() - tick)
            if remaining > 0:
                cancel.wait(remaining)


class SecondArmMotion(GraspPipelineV2):
    """The validated moves for the arm the pipeline does not sort with: its own
    controller, FK, IK and validator; scene, joint reader, config and log come
    from the primary pipeline.  No perception of its own, so the V1 pipeline
    constructor is deliberately skipped."""

    def __init__(self, primary: GraspPipelineV2, arm: str) -> None:
        self.primary = primary
        self.robot = primary.robot
        self.cfg = primary.cfg
        self.sort_arm = arm
        self.v2_active = False
        self.read_current_arm_q = primary.read_current_arm_q
        self.hand_T = primary.hand_T
        self._build_arm(arm)
        self.grip.event = self._event

    def _event(self, kind: str, message: str) -> None:
        self.primary.event(kind, f"{self.sort_arm} arm: {message}")

    def event(self, kind: str, message: str) -> None:
        self._event(kind, message)

    def diag(self, **values: Any) -> None:
        pass

    def current_table(self) -> Optional[TablePlane]:
        return self.primary.current_table()

    def stop_active_hold(self) -> None:
        self.controller.stop()


__all__ = ["yaw_error", "GraspPipelineV2", "SecondArmMotion", "ArmController", "MotionValidator", "MotionContext", "CarriedObject", "MoveResult",
           "Dex3Grip", "GripResult", "interpolate_pose", "palm_tilt", "THUMB_JOINTS", "FINGER_JOINTS"]
