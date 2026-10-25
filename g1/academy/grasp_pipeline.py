"""RGB-D perception and the Dex3 grab pipeline of recognition_app_v4 on sdk_wrapper.G1.

Used by the dashboard's /sort tab (academy/sorting.py).  This is a port of the
parts of hand_pose_navigation/recognition_app_v4.py that sorting_app.py builds
on, together with the pieces of direct_nav.DirectHandPoseNav,
tracking_loop.TrackingLoop and arm_executor.ArmExecutor that its grab uses --
those import modules/sdk_client.py, so they are re-implemented here against
sdk_wrapper.G1.  Only the dependency-free kinematics/perception helpers of
hand_pose_navigation are reused (arm_fk, arm_ik, grasp_planner, segmentation,
table_estimator, obstacle_checker, reachability_checker, vision_detector).

Behaviour, constants and log messages follow v4: a grab extends the arm with
the fixed table-aware recipe, opens the hand, approaches the object in
bounded joint steps (optionally correcting the target from fresh detections),
closes the hand under tactile feedback scaled by the object mass, and the lift
raises the hand vertically before moving to the WBC "right arm forward"
stable pose.
"""
from __future__ import annotations

import json
import math
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

_G1_ROOT = Path(__file__).resolve().parent.parent
if str(_G1_ROOT) not in sys.path:
    sys.path.insert(0, str(_G1_ROOT))

from hand_pose_navigation.arm_fk import ArmFK, JOINT_LIMITS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, SHOULDER_IN_BASE  # noqa: E402
from hand_pose_navigation.arm_ik import ArmIK  # noqa: E402
from hand_pose_navigation.grasp_planner import GraspPlanner  # noqa: E402
from hand_pose_navigation.obstacle_checker import Obstacles, check_swept_path  # noqa: E402
from hand_pose_navigation.reachability_checker import ReachabilityChecker  # noqa: E402
from hand_pose_navigation.segmentation import (  # noqa: E402
    GraspSurfaceEstimate, estimate_grasp_surface, mask_from_box_depth, pose_from_mask,
)
from hand_pose_navigation.table_estimator import TablePlane, estimate_table_plane  # noqa: E402
from hand_pose_navigation.target_detector import CameraIntrinsics  # noqa: E402
from hand_pose_navigation.vision_detector import parse_prompt_to_classes  # noqa: E402
from sdk_wrapper import HAND_CLOSED, HAND_JOINT_NAMES, HAND_SAFE_OPEN  # noqa: E402

# Unitree G1 depth-camera drawing: optical ray 42 deg from vertical.
_UNITREE_CAMERA_X_M = 47.64571478 / 1000.0
_UNITREE_CAMERA_Z_M = 462.68178553 / 1000.0
_UNITREE_CAMERA_DOWN_PITCH_RAD = math.radians(90.0 - 42.0)
DEFAULT_CAMERA_EXTRINSIC = {
    "x": _UNITREE_CAMERA_X_M, "y": 0.0, "z": _UNITREE_CAMERA_Z_M,
    "roll": -math.pi / 2.0 - _UNITREE_CAMERA_DOWN_PITCH_RAD, "pitch": 0.0, "yaw": -math.pi / 2.0,
}
DEFAULT_MAX_JOINT_STEP_RAD = 0.08
ABSOLUTE_MAX_JOINT_STEP_RAD = 0.10
DEFAULT_MAX_JOINT_SPEED_RAD_S = 0.30
ABSOLUTE_MAX_JOINT_SPEED_RAD_S = 0.60
DEFAULT_OBJECT_MASS_KG = 0.30
MAX_OBJECT_MASS_KG = 5.0
WAIST_YAW_JOINT_INDEX = 12
BASE_K = CameraIntrinsics()
KNOWN_INTRINSICS = {
    (424, 240): CameraIntrinsics(fx=302.1, fy=302.5, cx=214.3, cy=121.4, width=424, height=240),
    (640, 480): CameraIntrinsics(fx=604.2, fy=605.1, cx=324.7, cy=242.9, width=640, height=480),
}
STALE_AFTER_S = 4.0
DETECTION_HOLD_S = 1.5
DEFAULT_GRASP_STANDOFF_M = 0.08
DEFAULT_VISION_GRASP_LIFT_M = 0.04
DEFAULT_VISION_TABLE_CLEARANCE_M = 0.14
TABLE_COLLISION_MARGIN_M = 0.10
TABLE_COLLISION_XY_MARGIN_M = 0.12
DEFAULT_DIRECT_MAX_REACH_M = 0.52
DEFAULT_SLOW_GRAB_RATE_HZ = 3.0
DEFAULT_SLOW_GRAB_TIMEOUT_S = 180.0
DEFAULT_VISION_GRAB_CONVERGENCE_POS_M = 0.030
# Pre-grab reach limit for the planned hand target.  The URDF arm reaches at
# most ~0.432 m from the shoulder, so a target farther than that plus the
# vision convergence tolerance (0.030 m) cannot converge: with the 0.52 m
# executor limit such targets passed the check and the approach IK stalled
# 5-7 cm short until the timeout.
GRAB_TARGET_MAX_REACH_M = 0.46
# Vision grabs keep the palm level: the Dex3 fingers close in the palm x-y
# plane, so palm z must stay vertical or closing sweeps the fingers into the
# table.  IK accepts up to this palm-normal tilt; the approach converges below
# the second value (yaw is free).
GRASP_PALM_LEVEL_TOL_RAD = math.radians(6.0)
GRASP_PALM_LEVEL_CONVERGENCE_RAD = math.radians(10.0)
DEFAULT_VISION_STALL_CLOSE_POS_M = 0.075
DYNAMIC_TARGET_GATE_M = 0.06
DYNAMIC_TARGET_FILTER_ALPHA = 0.35
DYNAMIC_TARGET_DEADBAND_M = 0.008
DYNAMIC_TARGET_MAX_STEP_M = 0.015
DYNAMIC_TARGET_MAX_TOTAL_M = 0.08
DYNAMIC_TARGET_FREEZE_ERR_M = 0.06
DYNAMIC_TARGET_HAND_EXCLUSION_M = 0.10
DEFAULT_HAND_COMMAND_HOLD_S = 1.0
DEFAULT_HAND_RECOVERY_HOLD_S = 1.6
DEFAULT_HAND_TARGET_TOL_RAD = 0.12
DEX3_STALL_ERROR_RAD = 0.16
DEX3_STALL_SPEED_RAD_S = 0.12
DEX3_STALL_CONFIRM_S = 0.16
DEFAULT_EXTEND_STAGE_DURATION_S = 1.2
DEFAULT_CENTER_WAIST_MAX_YAW_RAD = 0.25
DEFAULT_CENTER_WAIST_DEADBAND_RAD = 0.025
DEFAULT_CENTER_WAIST_SPEED_RAD_S = 0.35
DEFAULT_POST_GRASP_LIFT_M = 0.08
DEFAULT_BOARD_PLACE_RELEASE_M = 0.12
DEFAULT_POST_GRASP_LIFT_DURATION_S = 2.5
DEFAULT_SURFACE_CLEARANCE_M = 0.03
MAX_SURFACE_STANDOFF_M = 0.16
WBC_SAVED_POSES_PATH = _G1_ROOT / "WBC" / "saved_ik_pose_cli_v3_poses.json"
WBC_RIGHT_ARM_FORWARD_POSE_NAME = "right arm forward"
# ArmFK's hand frame is the palm mount; this nudges display/exclusion points
# to the visible palm centre.
_PALM_VISUAL_OFFSET_M = 0.045

# Dex3 tactile raw values: 30000 is the "no reading" sentinel, >=100000 a
# confirmed contact.
TACTILE_INVALID_VALUE = 30000.0
TACTILE_VALID_THRESHOLD = 100000.0
DEFAULT_TACTILE_BASE_TARGET_RAW = 120000.0
DEFAULT_TACTILE_PER_KG_RAW = 60000.0
DEFAULT_TACTILE_MAX_TARGET_RAW = 450000.0
DEFAULT_TACTILE_CLOSE_STEPS = 10
DEFAULT_TACTILE_STEP_HOLD_S = 0.12
DEFAULT_TACTILE_STEP_SETTLE_S = 0.06

# Post-grasp lift stiffness/speed/duration scale with the object mass.
DEFAULT_LIFT_KP = 30.0
DEFAULT_LIFT_KD = 1.5
MAX_LIFT_KP = 48.0
MAX_LIFT_KD = 3.0
LIFT_KP_PER_KG = 5.0
LIFT_KD_PER_KG = 0.25
LIFT_DURATION_PER_KG = 0.35
MAX_LIFT_DURATION_S = 6.0
LIFT_SPEED_SLOWDOWN_PER_KG = 0.12

# Perception/camera threads stop this long after the last viewer or task.
IDLE_STOP_S = 30.0


@dataclass
class Detection:
    id: str
    label: str
    source: str  # "vision"
    score: float
    T_camera_object: np.ndarray
    box: Optional[Tuple[int, int, int, int]] = None
    grasp_surface: Optional[GraspSurfaceEstimate] = None


def copy_detection(det: Detection) -> Detection:
    return Detection(id=det.id, label=det.label, source=det.source, score=float(det.score),
                     T_camera_object=det.T_camera_object.copy(), box=None if det.box is None else tuple(det.box),
                     grasp_surface=det.grasp_surface)


def make_transform(xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = np.array(xyz, dtype=np.float64)
    return T


def base_from_camera(cam: Dict[str, float]) -> np.ndarray:
    return make_transform((cam["x"], cam["y"], cam["z"]), (cam["roll"], cam["pitch"], cam["yaw"]))


def palm_visual_point(T_base_hand: np.ndarray) -> np.ndarray:
    return T_base_hand[:3, 3] + T_base_hand[:3, :3] @ np.array([_PALM_VISUAL_OFFSET_M, 0.0, 0.0])


def _smooth_step(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def colorize_depth(depth_m: np.ndarray, max_m: float = 3.0) -> np.ndarray:
    clipped = np.clip(depth_m, 0.0, max_m)
    return cv2.applyColorMap((clipped / max_m * 255.0).astype(np.uint8), cv2.COLORMAP_JET)


def placeholder(text: str) -> np.ndarray:
    img = np.full((240, 320, 3), 30, dtype=np.uint8)
    cv2.putText(img, text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    return img


def _is_invalid_tactile_value(value: float) -> bool:
    return abs(float(value) - TACTILE_INVALID_VALUE) < 0.5


def grip_target_pressure_raw(mass_kg: float) -> float:
    """Heavier objects need a firmer grip before the hand stops closing."""
    target = DEFAULT_TACTILE_BASE_TARGET_RAW + DEFAULT_TACTILE_PER_KG_RAW * max(0.0, float(mass_kg))
    return float(min(DEFAULT_TACTILE_MAX_TARGET_RAW, target))


def lift_tuning_for_mass(mass_kg: float) -> Tuple[float, float, float, float]:
    """Stiffer, slower and longer post-grasp lifts for heavier objects."""
    mass_kg = max(0.0, float(mass_kg))
    kp = min(MAX_LIFT_KP, DEFAULT_LIFT_KP + LIFT_KP_PER_KG * mass_kg)
    kd = min(MAX_LIFT_KD, DEFAULT_LIFT_KD + LIFT_KD_PER_KG * mass_kg)
    duration_s = min(MAX_LIFT_DURATION_S, DEFAULT_POST_GRASP_LIFT_DURATION_S * (1.0 + LIFT_DURATION_PER_KG * mass_kg))
    return kp, kd, duration_s, 1.0 / (1.0 + LIFT_SPEED_SLOWDOWN_PER_KG * mass_kg)


def grasp_standoff_for_surface(grasp_surface: Optional[GraspSurfaceEstimate]) -> float:
    """Size the wrist standoff from the object's estimated near surface."""
    if grasp_surface is None or grasp_surface.confidence < 0.15:
        return DEFAULT_GRASP_STANDOFF_M
    surface_standoff = grasp_surface.near_surface_offset_m + DEFAULT_SURFACE_CLEARANCE_M
    return float(min(MAX_SURFACE_STANDOFF_M, max(DEFAULT_GRASP_STANDOFF_M, surface_standoff)))


def stable_vision_object_pose(T_base_object: np.ndarray, table: Optional[TablePlane] = None) -> np.ndarray:
    """Synthetic object frame: measured position, robot-side approach, palm
    kept above the table (mask-PCA orientation is too noisy to use)."""
    T = np.eye(4, dtype=np.float64)
    pos = T_base_object[:3, 3].copy()
    min_z = pos[2] + DEFAULT_VISION_GRASP_LIFT_M
    if table is not None:
        min_z = max(min_z, float(table.z_base_m) + DEFAULT_VISION_TABLE_CLEARANCE_M)
    pos[2] = min_z
    z_axis = np.array([-1.0, 0.0, 0.0])
    y_axis = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= max(1e-9, np.linalg.norm(x_axis))
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= max(1e-9, np.linalg.norm(y_axis))
    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T[:3, 3] = pos
    return T


def level_grasp_rotation(arm: str, p_object: np.ndarray) -> np.ndarray:
    """Palm z up (fingers close parallel to the table), fingers pointing
    horizontally from the shoulder toward the object."""
    d = np.asarray(p_object, dtype=np.float64)[:2] - SHOULDER_IN_BASE[arm][:2]
    yaw = math.atan2(float(d[1]), float(d[0]))
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def direct_grasp_hand_pose(det: Detection, T_base_object: np.ndarray, arm: str,
                           obstacles: Optional[Obstacles] = None, fixed_R_base: Optional[np.ndarray] = None) -> np.ndarray:
    T_object = (stable_vision_object_pose(T_base_object, table=obstacles.table if obstacles is not None else None)
                if det.source == "vision" else T_base_object)
    if fixed_R_base is None and det.source == "vision":
        fixed_R_base = level_grasp_rotation(arm, T_base_object[:3, 3])
    T_base_hand = GraspPlanner(arm=arm, standoff_m=grasp_standoff_for_surface(det.grasp_surface)).compute(T_object)
    if fixed_R_base is not None:
        T_base_hand = T_base_hand.copy()
        T_base_hand[:3, :3] = np.asarray(fixed_R_base, dtype=np.float64)
    return T_base_hand


def extend_arm_forward_joint_waypoints(arm: str) -> List[Tuple[str, np.ndarray]]:
    """Fixed, reviewed extend-arm recipe, independent of the current pose:
    [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll,
    wrist_pitch, wrist_yaw] radians."""
    side = str(arm).strip().lower()
    if side not in ("left", "right"):
        raise ValueError("arm must be 'left' or 'right'")
    sign = 1.0 if side == "left" else -1.0
    raw = [
        ("preconfigured_clearance", [-0.10, 0.50 * sign, 0.0, -0.25, 0.0, -0.10, 0.0]),
        ("preconfigured_forward", [-0.35, 0.30 * sign, 0.0, -0.70, 0.25, -0.28, -0.20]),
    ]
    result = []
    for name, values in raw:
        q = np.asarray(values, dtype=np.float64)
        for i, (lo, hi) in enumerate(JOINT_LIMITS[side]):
            q[i] = max(lo, min(hi, q[i]))
        result.append((name, q))
    return result


# ---------------------------------------------------------------------------
# Arm execution (port of arm_executor.ArmExecutor on G1's rt/arm_sdk client)
# ---------------------------------------------------------------------------

class ArmExecutor:
    """Interpolated, step/speed-capped arm commands with a safety gate.

    execute() runs one blocking smooth move; start_continuous() keeps
    republishing the newest submit_target() so slow perception/IK never
    leaves rt/arm_sdk silent (which lets the default controller reclaim the
    arm and snap it).
    """

    def __init__(self, robot: Any, arm: str = "right", kp: float = 60.0, kd: float = 2.0, rate_hz: float = 50.0,
                 safety_gate: bool = True, max_reach_m: float = 0.42, max_joint_step_rad: float = 0.1,
                 max_joint_speed_rad_s: float = 0.15, read_arm_q: Optional[Callable[[str], Optional[np.ndarray]]] = None) -> None:
        self.robot = robot
        self.arm = arm
        self.kp = kp
        self.kd = kd
        self.rate_hz = rate_hz
        self.safety_gate = safety_gate
        self.max_joint_step_rad = max(0.0, float(max_joint_step_rad))
        self.max_joint_speed_rad_s = max(0.0, float(max_joint_speed_rad_s))
        self._joint_indices = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
        self._read_arm_q = read_arm_q
        # URDF-exact FK for the swept-path obstacle check.
        self._fk = ArmFK(arm=arm, backend="urdf")
        self._checker = ReachabilityChecker(arm=arm, max_reach_m=max_reach_m)
        self._last_command_q: Optional[np.ndarray] = None
        self._continuous_thread: Optional[threading.Thread] = None
        self._continuous_stop: Optional[threading.Event] = None
        self._target_queue: Optional["queue.Queue[np.ndarray]"] = None

    def validate(self, q_arm_desired: np.ndarray, *, T_base_desired: Optional[np.ndarray] = None,
                 q_arm_start: Optional[np.ndarray] = None, obstacles: Optional[Obstacles] = None) -> Dict:
        if not self.safety_gate:
            return {"safe": True}
        result = self._checker.check(q_arm_desired, T_base_desired)
        if not result.safe:
            return {"safe": False, "reason": "safety_gate", "violations": result.reasons}
        if obstacles is not None and q_arm_start is not None:
            sweep = check_swept_path(self._fk, q_arm_start, q_arm_desired, obstacles)
            if not sweep.safe:
                return {"safe": False, "reason": "obstacle_gate", "violations": sweep.reasons}
        return {"safe": True}

    def start_continuous(self, rate_hz: Optional[float] = None) -> None:
        if self._continuous_thread is not None:
            return
        seed = self._last_command_q if self._last_command_q is not None else self._current_q()
        if seed is None:
            raise RuntimeError("cannot start continuous execution: joint state unavailable")
        self._last_command_q = seed.copy()
        self._target_queue = queue.Queue(maxsize=1)
        self._continuous_stop = threading.Event()
        thread = threading.Thread(target=self._continuous_loop, name=f"arm-exec-{self.arm}",
                                  args=(self._target_queue, self._continuous_stop, max(1.0, float(rate_hz or self.rate_hz))),
                                  daemon=True)
        self._continuous_thread = thread
        thread.start()

    def stop_continuous(self) -> None:
        thread, stop_event = self._continuous_thread, self._continuous_stop
        self._continuous_thread = self._continuous_stop = self._target_queue = None
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def submit_target(self, q_arm_desired: np.ndarray) -> None:
        """Replace any pending target so the thread ramps to the newest one."""
        target_queue = self._target_queue
        if target_queue is None:
            raise RuntimeError("start_continuous() was not called")
        q = np.asarray(q_arm_desired, dtype=np.float64).copy()
        try:
            target_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            target_queue.put_nowait(q)
        except queue.Full:
            pass

    def _continuous_loop(self, target_queue: "queue.Queue[np.ndarray]", stop_event: threading.Event, rate_hz: float) -> None:
        dt = 1.0 / rate_hz
        q_target = self._last_command_q.copy()
        while not stop_event.is_set():
            t0 = time.time()
            try:
                while True:
                    q_target = target_queue.get_nowait()
            except queue.Empty:
                pass
            q_cmd, _ = self._cap_command_step(q_target, dt)
            self._send_command(q_cmd)
            remaining = dt - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)

    def execute(self, q_arm_desired: np.ndarray, duration_s: float = 2.0, q_arm_start: Optional[np.ndarray] = None,
                T_base_desired: Optional[np.ndarray] = None, stop_event: Optional[threading.Event] = None,
                obstacles: Optional[Obstacles] = None) -> Dict:
        if q_arm_start is None:
            q_arm_start = self._current_q()
            if q_arm_start is None:
                return {"success": False, "reason": "joint_state_unavailable", "duration_s": 0.0, "steps": 0}
        q_arm_start = np.asarray(q_arm_start, dtype=np.float64).copy()
        q_arm_desired = np.asarray(q_arm_desired, dtype=np.float64).copy()
        validation = self.validate(q_arm_desired, T_base_desired=T_base_desired, q_arm_start=q_arm_start, obstacles=obstacles)
        if not validation.get("safe"):
            return {"success": False, "reason": validation.get("reason"), "violations": validation.get("violations"),
                    "duration_s": 0.0, "steps": 0}
        if self._last_command_q is None:
            self._last_command_q = q_arm_start.copy()
        requested_duration_s = max(0.0, float(duration_s))
        duration_s = self._duration_for_speed_limit(q_arm_start, q_arm_desired, requested_duration_s)
        steps = max(1, int(duration_s * self.rate_hz))
        dt = duration_s / steps
        final_q = q_arm_start.copy()
        capped_samples = 0
        for i in range(steps):
            if stop_event is not None and stop_event.is_set():
                return {"success": False, "reason": "stopped", "duration_s": duration_s, "steps": i, "final_q": final_q}
            alpha = _smooth_step((i + 1) / steps)
            q_cmd, was_capped = self._cap_command_step((1 - alpha) * q_arm_start + alpha * q_arm_desired, dt)
            capped_samples += int(was_capped)
            self._send_command(q_cmd)
            final_q = q_cmd
            time.sleep(dt)
        return {"success": True, "duration_s": duration_s, "requested_duration_s": requested_duration_s, "steps": steps,
                "final_q": final_q, "final_error_rad": float(np.max(np.abs(q_arm_desired - final_q))) if final_q.size else 0.0,
                "capped_samples": capped_samples}

    def _current_q(self) -> Optional[np.ndarray]:
        try:
            return self._read_arm_q(self.arm) if self._read_arm_q is not None else None
        except Exception:
            return None

    def _cap_command_step(self, q_arm: np.ndarray, dt: float) -> Tuple[np.ndarray, bool]:
        """Hard-cap every published sample against the previous one."""
        q_arm = np.asarray(q_arm, dtype=np.float64)
        caps = []
        if self.max_joint_step_rad > 0.0:
            caps.append(self.max_joint_step_rad)
        if self.max_joint_speed_rad_s > 0.0 and dt > 0.0:
            caps.append(self.max_joint_speed_rad_s * float(dt))
        if not caps or self._last_command_q is None:
            self._last_command_q = q_arm.copy()
            return q_arm, False
        cap = min(caps)
        delta = q_arm - self._last_command_q
        max_abs_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
        if max_abs_delta <= cap:
            self._last_command_q = q_arm.copy()
            return q_arm, False
        capped = self._last_command_q + delta * (cap / max_abs_delta)
        self._last_command_q = capped.copy()
        return capped, True

    def _duration_for_speed_limit(self, q_start: np.ndarray, q_desired: np.ndarray, requested_duration_s: float) -> float:
        duration_s = max(0.02, float(requested_duration_s))
        delta = np.asarray(q_desired, dtype=np.float64) - np.asarray(q_start, dtype=np.float64)
        max_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
        if max_delta <= 1e-9:
            return duration_s
        # Smooth-step reaches about 1.5x average velocity at mid-trajectory.
        if self.max_joint_speed_rad_s > 0.0:
            duration_s = max(duration_s, 2.0 * max_delta / self.max_joint_speed_rad_s)
        if self.max_joint_step_rad > 0.0 and self.rate_hz > 0.0:
            duration_s = max(duration_s, 2.0 * max_delta / (self.max_joint_step_rad * self.rate_hz))
        return duration_s

    def _send_command(self, q_arm: np.ndarray) -> None:
        """Publish this arm's joints on rt/arm_sdk; other joints keep their last command."""
        targets = {int(j): float(q_arm[i]) for i, j in enumerate(self._joint_indices)}
        self.robot._arm_sdk_client().write(targets, kp=float(self.kp), kd=float(self.kd))


# ---------------------------------------------------------------------------
# Closed-loop approach (DirectHandPoseNav + TrackingLoop, fixed target only)
# ---------------------------------------------------------------------------

class HandApproach:
    """Re-solve IK every tick from the measured joints toward a desired hand
    pose, in capped joint increments, until converged, stalled or timed out.

    The target can be corrected mid-run (update_target); a correction restarts
    the stall window.  When the loop converges or stalls its continuous
    executor keeps holding the last command until shutdown().
    """

    _STALL_WINDOW = 30
    _STALL_PROGRESS_TOL_M = 0.001

    def __init__(self, pipeline: "GraspPipeline", T_base_hand_desired: np.ndarray, config: Dict[str, Any],
                 obstacles: Optional[Obstacles]) -> None:
        arm = config["arm"]
        self.pipeline = pipeline
        self.arm = arm
        self.config = dict(config)
        self._target = np.asarray(T_base_hand_desired, dtype=np.float64).copy()
        self._target_lock = threading.Lock()
        self._target_revision = 0
        self.fk = ArmFK(arm=arm)
        self.ik = ArmIK(arm=arm, solver=config["ik_solver"], tol_pos_m=config["ik_tol_pos_m"],
                        tol_rot_rad=config["ik_tol_rot_rad"])
        self.checker = ReachabilityChecker(arm=arm, max_reach_m=config["max_reach_m"])
        self.executor = ArmExecutor(pipeline.robot, arm=arm, max_reach_m=config["max_reach_m"],
                                    max_joint_step_rad=config["max_joint_step_rad"],
                                    max_joint_speed_rad_s=config["max_joint_speed_rad_s"],
                                    read_arm_q=pipeline.read_current_arm_q)
        self.rate_hz = float(config["rate_hz"])
        step = float(config["max_joint_step_rad"])
        if self.rate_hz > 0.0 and config["max_joint_speed_rad_s"] > 0.0:
            step = min(step, float(config["max_joint_speed_rad_s"]) / self.rate_hz)
        self.max_joint_step_rad = max(0.0, step)
        self.convergence_pos_m = float(config["convergence_pos_m"])
        self.convergence_rot_rad = float(config["convergence_rot_rad"])
        self.timeout_s = float(config["timeout_s"])
        self.static_obstacles = obstacles
        self._joint_indices = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
        self._opposite_arm = "left" if arm == "right" else "right"
        self._opposite_fk = ArmFK(arm=self._opposite_arm, backend="urdf")
        self._stop_event = threading.Event()
        self.status: Dict[str, Any] = {"running": True, "converged": False, "stalled": False, "iteration": 0,
                                       "last_error_pos_m": None, "last_error_rot_rad": None, "ik_failures": 0,
                                       "safety_rejections": 0, "detection_failures": 0, "log": []}
        self._thread = threading.Thread(target=self._run, name=f"hand-approach-{arm}", daemon=True)
        self._thread.start()

    def _record(self, msg: str) -> None:
        log = self.status["log"]
        log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        del log[:-200]

    def update_target(self, T_base_hand_desired: np.ndarray) -> None:
        with self._target_lock:
            self._target = np.asarray(T_base_hand_desired, dtype=np.float64).copy()
            self._target_revision += 1

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self.executor.stop_continuous()

    def status_snapshot(self) -> Dict[str, Any]:
        out = dict(self.status)
        out["log"] = list(self.status["log"][-80:])
        return out

    def _run(self) -> None:
        try:
            self.executor.start_continuous()
        except Exception as exc:
            self._record(f"[Step 9] Could not start continuous executor: {exc}")
            self.status["running"] = False
            return
        try:
            self._loop()
        except Exception as exc:
            self._record(f"[Step 10] Approach loop error: {exc}")
        finally:
            if not (self.status["converged"] or self.status["stalled"]):
                self.executor.stop_continuous()
            self.status["running"] = False

    def _loop(self) -> None:
        dt = 1.0 / self.rate_hz
        start_t = time.time()
        q_arm_commanded: Optional[np.ndarray] = None
        err_history: Deque[float] = deque(maxlen=self._STALL_WINDOW)
        seen_revision = self._target_revision
        while not self._stop_event.is_set():
            t0 = time.time()
            self.status["iteration"] += 1
            with self._target_lock:
                T_base_desired = self._target.copy()
                revision = self._target_revision
            if revision != seen_revision:
                seen_revision = revision
                err_history.clear()
                self._record("[Step 5] Target corrected - stall window restarted.")
            elapsed = t0 - start_t
            if self.timeout_s > 0 and elapsed > self.timeout_s:
                self._record(f"[Step 10] Timeout after {elapsed:.1f}s - stopping.")
                break
            q_full = self.pipeline.read_q_full()
            if q_full is None:
                self.status["safety_rejections"] += 1
                self._record("[Step 4] Joint state unavailable - skipping command.")
                self._sleep(dt, t0)
                continue
            T_base_hand = self.fk.compute(q_full)
            q_arm_cur = q_full[self._joint_indices]
            reach_check = self.checker.check_target_reachable(T_base_desired)
            if not reach_check.safe:
                self.status["safety_rejections"] += 1
                self._record(f"[Step 8] Workspace fail: {reach_check.reasons}")
                self._record("[Step 8] Fixed selected target is unreachable - stopping grab.")
                break
            err_pos, err_rot = self._pose_error_scalars(T_base_desired, T_base_hand, level=self.ik.solver == "level_dls")
            self.status["last_error_pos_m"], self.status["last_error_rot_rad"] = err_pos, err_rot
            if err_pos < self.convergence_pos_m and err_rot < self.convergence_rot_rad:
                self.status["converged"] = True
                self._record(f"[Step 10] Converged! pos_err={err_pos:.4f}m  rot_err={err_rot:.4f}rad  "
                             f"iter={self.status['iteration']}")
                break
            err_history.append(err_pos)
            if len(err_history) == err_history.maxlen and err_history[-1] >= err_history[0] - self._STALL_PROGRESS_TOL_M:
                self.status["stalled"] = True
                delta = T_base_desired[:3, 3] - T_base_hand[:3, 3]
                self._record(f"[Step 10] Stalled: target not getting closer over last {err_history.maxlen} EE-pose "
                             f"increments ({err_history[0]:.4f}m -> {err_history[-1]:.4f}m); "
                             f"delta_xyz=({delta[0]:+.3f},{delta[1]:+.3f},{delta[2]:+.3f}) - aborting.")
                break
            q_arm_desired, ik_info = self.ik.solve(T_base_desired, q_init=q_arm_cur)
            if q_arm_desired is None:
                self.status["ik_failures"] += 1
                p = T_base_desired[:3, 3]
                self._record(f"[Step 7] IK failed: pos_err={ik_info['error_pos_m']:.4f} "
                             f"target=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})")
                self._sleep(dt, t0)
                continue
            safety = self.checker.check(q_arm_desired, T_base_desired)
            if not safety.safe:
                self.status["safety_rejections"] += 1
                self._record(f"[Step 8] Safety fail: {safety.reasons}")
                self._sleep(dt, t0)
                continue
            q_start = q_arm_commanded if q_arm_commanded is not None else q_arm_cur
            q_next = q_arm_desired.copy()
            if self.max_joint_step_rad > 0.0:
                delta_q = q_arm_desired - q_start
                max_abs = float(np.max(np.abs(delta_q))) if delta_q.size else 0.0
                if max_abs > self.max_joint_step_rad:
                    q_next = q_start + delta_q * (self.max_joint_step_rad / max_abs)
            obstacles = None
            if self.static_obstacles is not None:
                so = self.static_obstacles
                obstacles = Obstacles(
                    table=so.table, opposite_shoulder_base=SHOULDER_IN_BASE[self._opposite_arm],
                    opposite_wrist_base=self._opposite_fk.compute(q_full)[:3, 3], torso_radius_m=so.torso_radius_m,
                    torso_z=so.torso_z, self_collision_radius_m=so.self_collision_radius_m,
                    table_margin_m=so.table_margin_m, table_xy_margin_m=so.table_xy_margin_m,
                )
            validation = self.executor.validate(q_next, T_base_desired=T_base_desired, q_arm_start=q_start, obstacles=obstacles)
            if not validation.get("safe"):
                self.status["safety_rejections"] += 1
                label = "[Step 8b] Obstacle in swept path" if validation.get("reason") == "obstacle_gate" else "[Step 9] Command rejected"
                self._record(f"{label}: {validation.get('violations') or validation.get('reason')}")
                self._sleep(dt, t0)
                continue
            self.executor.submit_target(q_next)
            q_arm_commanded = q_next.copy()
            self._record(f"[Step 10] it={self.status['iteration']:4d}  pos_err={err_pos:.4f}m  "
                         f"rot_err={err_rot:.3f}rad  ik_iter={ik_info['iterations']}")
            self._sleep(dt, t0)

    @staticmethod
    def _pose_error_scalars(T_des: np.ndarray, T_cur: np.ndarray, level: bool = False) -> Tuple[float, float]:
        pos_err = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
        if level:
            # Level-palm grasp: only the palm-normal tilt counts, yaw is free.
            return pos_err, float(np.arccos(np.clip(np.dot(T_des[:3, 2], T_cur[:3, 2]), -1.0, 1.0)))
        R = T_des[:3, :3] @ T_cur[:3, :3].T
        rot_err = float(np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5)
        return pos_err, rot_err

    @staticmethod
    def _sleep(dt: float, t_start: float) -> None:
        remaining = dt - (time.time() - t_start)
        if remaining > 0:
            time.sleep(remaining)


class GrabTargetCorrector:
    """Steer a running approach toward fresh detections of its object.

    A measurement is used only if it lies within the gate of the current
    estimate; it is low-pass filtered, applied in bounded steps, capped in
    total, ignored near the hand, and correction freezes once the hand is
    close (the hand then contaminates the object's depth mask).
    """

    def __init__(self, pipeline: "GraspPipeline", det: Detection, arm: str, T_base_camera: np.ndarray,
                 T_base_object: np.ndarray, obstacles: Optional[Obstacles],
                 candidates: Optional[Callable[[Detection], Tuple[float, List[np.ndarray]]]] = None) -> None:
        self.pipeline = pipeline
        self.det = det
        self.arm = arm
        self.T_base_camera = T_base_camera.copy()
        self.T_base_object = T_base_object.copy()
        self.obstacles = obstacles
        self.candidates = candidates
        self.p_initial = T_base_object[:3, 3].copy()
        self.p_filtered = self.p_initial.copy()
        self.p_commanded = self.p_initial.copy()
        with pipeline.lock:
            self.last_detection_ts = pipeline.last_detection_ts
        self.frozen = False
        self.updates = 0

    def _candidate_poses(self) -> Tuple[float, List[np.ndarray]]:
        if self.candidates is not None:
            return self.candidates(self.det)
        p = self.pipeline
        with p.lock:
            return p.last_detection_ts, [d.T_camera_object.copy() for d in p.detections.values()
                                         if d.source == self.det.source and d.label == self.det.label]

    def _measure(self) -> Optional[np.ndarray]:
        detection_ts, candidates = self._candidate_poses()
        if detection_ts <= self.last_detection_ts or (time.time() - detection_ts) > STALE_AFTER_S:
            return None
        self.last_detection_ts = detection_ts
        best, best_dist = None, DYNAMIC_TARGET_GATE_M
        for T_camera_object in candidates:
            p_base = (self.T_base_camera @ T_camera_object)[:3, 3]
            dist = float(np.linalg.norm(p_base - self.p_filtered))
            if dist <= best_dist:
                best, best_dist = p_base, dist
        if best is None:
            return None
        T_base_hand = self.pipeline.hand_T(self.arm)
        if T_base_hand is not None and float(np.linalg.norm(palm_visual_point(T_base_hand) - best)) < DYNAMIC_TARGET_HAND_EXCLUSION_M:
            return None
        return best

    def step(self, approach: HandApproach, pos_err: Optional[float]) -> Optional[str]:
        """Apply at most one bounded correction; return a log line if any."""
        if self.frozen:
            return None
        if pos_err is not None and pos_err < DYNAMIC_TARGET_FREEZE_ERR_M:
            self.frozen = True
            total = float(np.linalg.norm(self.p_commanded - self.p_initial))
            return (f"dynamic target frozen near the object (pos_err={pos_err:.3f}m) after "
                    f"{self.updates} correction(s), total {total * 100:.1f} cm.")
        p_meas = self._measure()
        if p_meas is None:
            return None
        a = DYNAMIC_TARGET_FILTER_ALPHA
        self.p_filtered = (1.0 - a) * self.p_filtered + a * p_meas
        offset = self.p_filtered - self.p_initial
        offset_norm = float(np.linalg.norm(offset))
        if offset_norm > DYNAMIC_TARGET_MAX_TOTAL_M:
            offset *= DYNAMIC_TARGET_MAX_TOTAL_M / offset_norm
        delta = (self.p_initial + offset) - self.p_commanded
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm < DYNAMIC_TARGET_DEADBAND_M:
            return None
        if delta_norm > DYNAMIC_TARGET_MAX_STEP_M:
            delta *= DYNAMIC_TARGET_MAX_STEP_M / delta_norm
        p_new = self.p_commanded + delta
        T_base_object = self.T_base_object.copy()
        T_base_object[:3, 3] = p_new
        approach.update_target(direct_grasp_hand_pose(self.det, T_base_object, self.arm, self.obstacles))
        self.p_commanded = p_new
        self.updates += 1
        total = p_new - self.p_initial
        return (f"dynamic target correction #{self.updates}: object base xyz=({p_new[0]:+.3f}, {p_new[1]:+.3f}, "
                f"{p_new[2]:+.3f}) m (step {np.linalg.norm(delta) * 100:.1f} cm, total "
                f"{total[0] * 100:+.1f}/{total[1] * 100:+.1f}/{total[2] * 100:+.1f} cm)")


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

class GraspPipeline:
    """Shared state, camera/perception threads and grab/lift/place motions.

    Attribute and method names follow recognition_app_v4's STATE and
    functions so the two can be compared side by side.
    """

    def __init__(self, robot: Any, rgbd_endpoint: str, vision_model: str = "yolov8s-world.pt",
                 stable_pose_file: Path = WBC_SAVED_POSES_PATH, perception_rate_hz: float = 3.0,
                 camera_rate_hz: float = 15.0) -> None:
        self.robot = robot
        self.endpoint = str(rgbd_endpoint)
        self.vision_model = str(vision_model)
        self.stable_pose_file = Path(stable_pose_file)
        self.perception_rate_hz = float(perception_rate_hz)
        self.camera_rate_hz = float(camera_rate_hz)
        self.lock = threading.RLock()
        self.arm_lock = threading.Lock()          # ARM_CONTROL_LOCK
        self.cancel = threading.Event()           # ARM_CANCEL_EVENT
        self.nav_lock = threading.Lock()          # ACTIVE_NAV_LOCK
        self.active_nav: Optional[HandApproach] = None
        self.hold_executor: Optional[ArmExecutor] = None
        self._authority_engaged = False
        # Optional matcher for dynamic target correction (set by the sorter).
        self.correction_candidates: Optional[Callable[[Detection], Tuple[float, List[np.ndarray]]]] = None
        self.K = CameraIntrinsics()
        self._hand_fk: Dict[str, ArmFK] = {}
        self.vision: Any = None
        self.vision_status = "vision model not loaded"
        self._vision_prompt = "coffee cup"
        self.vision_classes = parse_prompt_to_classes(self._vision_prompt)
        self.rgb_bgr: Optional[np.ndarray] = None
        self.depth_m: Optional[np.ndarray] = None
        self.detection_rgb_bgr: Optional[np.ndarray] = None
        self.detections: Dict[str, Detection] = {}
        self.boxes_for_draw: List[Dict] = []
        self.masks_for_draw: List[Tuple[str, str, np.ndarray]] = []
        self.max_joint_step_rad = DEFAULT_MAX_JOINT_STEP_RAD
        self.max_joint_speed_rad_s = DEFAULT_MAX_JOINT_SPEED_RAD_S
        self.selected_id: Optional[str] = None
        self.camera_extrinsic = dict(DEFAULT_CAMERA_EXTRINSIC)
        self.dynamic_target_correction = True
        self.obstacle_status = "obstacles: not checked yet"
        self.status_msg = "camera idle"
        self.camera_status = "camera idle"
        self.grab_log: List[str] = []
        self.grab_running = False
        self.arm_motion_running = False
        self.arm_motion_label = ""
        self.last_frame_ts = 0.0
        self.last_detection_ts = 0.0
        self.frame_seq = 0
        self.extended_hand_reference_q: Dict[str, np.ndarray] = {}
        self.extended_hand_reference_T_base: Dict[str, np.ndarray] = {}
        self.object_mass_kg = DEFAULT_OBJECT_MASS_KG
        self.last_grip_result: Dict[str, Any] = {}
        self.lift_confirmation_pending = False
        self.lift_arm: Optional[str] = None
        self.lift_unverified = False
        self.holding_object = False
        self.holding_arm: Optional[str] = None
        self.keep_alive = 0          # >0 while a task needs perception
        self.last_viewer = 0.0
        self._threads: Dict[str, threading.Thread] = {}
        self._threads_lock = threading.Lock()

    # -- logging / lifecycle -------------------------------------------------

    def log(self, line: str) -> None:
        with self.lock:
            self.grab_log.append(f"{time.strftime('%H:%M:%S')} {line}")
            del self.grab_log[:-300]

    def touch(self) -> None:
        """Keep camera + perception running (viewer present or task active)."""
        self.last_viewer = time.monotonic()
        with self._threads_lock:
            for name, target in (("camera", self._camera_loop), ("perception", self._perception_loop)):
                thread = self._threads.get(name)
                if thread is None or not thread.is_alive():
                    thread = threading.Thread(target=target, name=f"grasp-{name}", daemon=True)
                    self._threads[name] = thread
                    thread.start()

    def _wanted(self) -> bool:
        return (self.keep_alive > 0 or self.grab_running or self.arm_motion_running
                or time.monotonic() - self.last_viewer < IDLE_STOP_S)

    def set_prompt(self, text: str) -> List[str]:
        with self.lock:
            self._vision_prompt = text
            vision = self.vision
        classes = vision.set_prompt(text) if vision is not None else parse_prompt_to_classes(text)
        with self.lock:
            self.vision_classes = classes
        return classes

    # -- camera + perception -------------------------------------------------

    def _update_intrinsics(self, rgb: np.ndarray) -> None:
        h, w = rgb.shape[:2]
        if w == self.K.width and h == self.K.height:
            return
        if (w, h) in KNOWN_INTRINSICS:
            self.K = KNOWN_INTRINSICS[(w, h)]
        else:
            sx, sy = float(w) / BASE_K.width, float(h) / BASE_K.height
            self.K = CameraIntrinsics(fx=BASE_K.fx * sx, fy=BASE_K.fy * sy, cx=BASE_K.cx * sx, cy=BASE_K.cy * sy,
                                      width=w, height=h)

    def camera_matrix(self) -> np.ndarray:
        K = self.K
        return np.array([[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    def _camera_loop(self) -> None:
        """Newest RGB-D frame from realsense_rgbd_zmq_stream.py (JPEG, PNG depth, scale)."""
        try:
            import zmq
        except ImportError:
            self.camera_status = "camera needs pyzmq"
            return
        context = zmq.Context.instance()
        sock = context.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVHWM, 2)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.endpoint)
        period = 1.0 / max(1.0, self.camera_rate_hz)
        self.camera_status = f"waiting for RGB-D frames on {self.endpoint}"
        try:
            while self._wanted():
                t0 = time.time()
                if not sock.poll(1000):
                    self.camera_status = (f"waiting for RGB-D frames on {self.endpoint} "
                                          "(start visualizations/realsense_rgbd_zmq_stream.py)")
                    continue
                parts = sock.recv_multipart()
                while sock.poll(0):  # detection is slower than the publisher: keep the newest
                    parts = sock.recv_multipart()
                try:
                    if len(parts) >= 4:
                        parts = parts[-3:]
                    rgb = cv2.imdecode(np.frombuffer(parts[0], dtype=np.uint8), cv2.IMREAD_COLOR)
                    raw = (cv2.imdecode(np.frombuffer(parts[1], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                           if len(parts) > 1 and parts[1] != b"0" else None)
                    if rgb is None or raw is None:
                        raise ValueError("frame without decodable RGB and depth")
                    if raw.ndim == 3:
                        raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
                    scale = 0.001
                    if len(parts) >= 3 and len(parts[2]) >= 4:
                        scale = float(struct.unpack("f", parts[2][:4])[0]) or 0.001
                    depth = raw.astype(np.float32) * scale
                    if depth.shape[:2] != rgb.shape[:2]:
                        depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                    self._update_intrinsics(rgb)
                    with self.lock:
                        self.rgb_bgr = rgb
                        self.depth_m = depth
                        self.last_frame_ts = time.time()
                        self.frame_seq += 1
                    self.camera_status = f"RGB-D {rgb.shape[1]}x{rgb.shape[0]} from {self.endpoint}"
                except Exception as exc:
                    self.camera_status = f"camera error: {exc}"
                time.sleep(max(0.0, period - (time.time() - t0)))
        finally:
            sock.close(0)
            self.camera_status = "camera paused (no viewers)"

    def _load_vision(self) -> None:
        self.vision_status = f"loading {self.vision_model}..."
        try:
            from hand_pose_navigation.vision_detector import VisionDetector
            vision = VisionDetector(model_name=self.vision_model)
        except Exception as exc:
            self.vision_status = f"vision model unavailable: {exc}"
            return
        with self.lock:
            prompt = self._vision_prompt
        classes = vision.set_prompt(prompt)
        with self.lock:
            self.vision = vision
            self.vision_classes = classes
        self.vision_status = "on" if vision.available else f"off ({vision.error})"

    def _perception_loop(self) -> None:
        if self.vision is None:
            self._load_vision()
        vision = self.vision
        period = 1.0 / max(0.5, self.perception_rate_hz)
        last_seq = -1
        held_detections: Dict[str, Tuple[Detection, float]] = {}
        held_boxes: Dict[str, Tuple[Dict, float]] = {}
        held_masks: Dict[str, Tuple[Tuple[str, str, np.ndarray], float]] = {}

        # Short motion blur or one slow model call must not blank the list;
        # hold detections briefly, still clearing them before they go stale.
        def merge_and_prune(held: Dict, now: float) -> Dict:
            for key in [k for k, (_v, ts) in held.items() if now - ts > DETECTION_HOLD_S]:
                del held[key]
            return {k: v for k, (v, _ts) in held.items()}

        while self._wanted():
            t0 = time.time()
            with self.lock:
                seq = self.frame_seq
                rgb = None if self.rgb_bgr is None else self.rgb_bgr.copy()
                depth = None if self.depth_m is None else self.depth_m.copy()
                selected_id = self.selected_id
            if rgb is None or depth is None or seq == last_seq:
                time.sleep(min(0.05, period))
                continue
            last_seq = seq
            K = self.K
            vis_dets = vision.detect(rgb) if vision is not None and vision.available else []
            detections: Dict[str, Detection] = {}
            boxes: List[Dict] = []
            masks: List[Tuple[str, str, np.ndarray]] = []
            for i, vd in enumerate(vis_dets):
                mask = mask_from_box_depth(depth, vd.box_xyxy)
                pose = pose_from_mask(mask, depth, K)
                det_id = f"vision:{i}"
                boxes.append({"id": det_id, "box": vd.box_xyxy, "label": vd.label, "score": vd.score,
                              "selected": det_id == selected_id})
                if pose is None:
                    continue
                detections[det_id] = Detection(id=det_id, label=vd.label, source="vision", score=vd.score,
                                               T_camera_object=pose.T_camera_object, box=vd.box_xyxy,
                                               grasp_surface=estimate_grasp_surface(mask, depth, K))
                masks.append((det_id, vd.label, mask))
            now = time.time()
            for det_id, det in detections.items():
                held_detections[det_id] = (det, now)
            for box in boxes:
                held_boxes[box["id"]] = (box, now)
            for entry in masks:
                held_masks[entry[0]] = (entry, now)
            detections = merge_and_prune(held_detections, now)
            boxes = list(merge_and_prune(held_boxes, now).values())
            masks = list(merge_and_prune(held_masks, now).values())
            with self.lock:
                self.detection_rgb_bgr = rgb
                self.detections = detections
                self.boxes_for_draw = boxes
                self.masks_for_draw = masks
                self.last_detection_ts = time.time()
                state = "on" if vision is not None and vision.available else self.vision_status
                self.status_msg = f"{len(detections)} detections  |  vision model: {state}  |  frame #{seq}"
            time.sleep(max(0.0, period - (time.time() - t0)))

    # -- joint state / FK ----------------------------------------------------

    def read_q_full(self) -> Optional[np.ndarray]:
        snap = self.robot.get_joint_positions()
        if snap is None:
            return None
        q = np.zeros(30, dtype=np.float64)
        for j, value in snap[0].items():
            q[int(j)] = float(value)
        return q

    def read_current_arm_q(self, arm: str) -> np.ndarray:
        joints = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
        snap = self.robot.get_joint_positions(joints, max_age_s=STALE_AFTER_S)
        if snap is None:
            raise RuntimeError(f"cannot read current {arm} arm joints from lowstate")
        return np.asarray([snap[0][j] for j in joints], dtype=np.float64)

    def read_joint_position(self, joint_index: int) -> float:
        snap = self.robot.get_joint_positions([int(joint_index)], max_age_s=STALE_AFTER_S)
        if snap is None:
            raise RuntimeError(f"cannot read joint index {joint_index} from lowstate")
        return float(snap[0][int(joint_index)])

    def hand_T(self, arm: str) -> Optional[np.ndarray]:
        q_full = self.read_q_full()
        if q_full is None:
            return None
        fk = self._hand_fk.get(arm)
        if fk is None:
            fk = self._hand_fk[arm] = ArmFK(arm)
        return fk.compute(q_full)

    def project_camera_point(self, point_camera: np.ndarray) -> Optional[Tuple[int, int]]:
        x, y, z = (float(v) for v in point_camera[:3])
        if z <= 0.05:
            return None
        K = self.K
        u, v = int(round(K.fx * x / z + K.cx)), int(round(K.fy * y / z + K.cy))
        if u < -200 or u > K.width + 200 or v < -200 or v > K.height + 200:
            return None
        return u, v

    # -- obstacles / reach ---------------------------------------------------

    def estimate_obstacles(self, cam: Dict[str, float]) -> Obstacles:
        """Fit the table plane from the current depth frame for swept-path checks."""
        with self.lock:
            depth = None if self.depth_m is None else self.depth_m.copy()
        table: Optional[TablePlane] = None
        if depth is not None:
            try:
                table = estimate_table_plane(depth, self.camera_matrix(), base_from_camera(cam))
            except Exception as exc:
                self.log(f"[grasp] table plane estimation failed: {exc}")
        if table is not None:
            status = (f"obstacles: table z={table.z_base_m:+.3f}m x=[{table.x_range_m[0]:+.2f},{table.x_range_m[1]:+.2f}] "
                      f"y=[{table.y_range_m[0]:+.2f},{table.y_range_m[1]:+.2f}] confidence={table.confidence:.2f} "
                      f"(n={table.inlier_count})")
        else:
            status = "obstacles: no confident table plane found in current depth frame"
        self.log(f"[grasp] {status}")
        with self.lock:
            self.obstacle_status = status
        # A palm is not a point and the table fit has cm-level uncertainty.
        return Obstacles(table=table, table_margin_m=TABLE_COLLISION_MARGIN_M, table_xy_margin_m=TABLE_COLLISION_XY_MARGIN_M)

    def resolve_arm_and_base_pose(self, det: Detection, cam: Dict[str, float], arm_override: str) -> Tuple[str, np.ndarray, np.ndarray]:
        T_base_camera = base_from_camera(cam)
        T_base_object = T_base_camera @ det.T_camera_object
        arm = arm_override
        if arm == "auto":
            arm = "left" if T_base_object[1, 3] > 0 else "right"
        return arm, T_base_camera, T_base_object

    def reach_preview(self, det: Detection, cam: Dict[str, float], arm_override: str,
                      obstacles: Optional[Obstacles] = None) -> Tuple[str, float, float, float]:
        arm, _T, T_base_object = self.resolve_arm_and_base_pose(det, cam, arm_override)
        T_base_desired = direct_grasp_hand_pose(det, T_base_object, arm, obstacles)
        shoulder = np.array([0.0, 0.10 if arm == "left" else -0.10, 0.292])
        reach = float(np.linalg.norm(T_base_desired[:3, 3] - shoulder))
        return arm, reach, GRAB_TARGET_MAX_REACH_M, max(0.0, reach - GRAB_TARGET_MAX_REACH_M)

    # -- arm authority, holds and motions -------------------------------------

    def ensure_arm_authority(self, duration_s: float) -> None:
        """Ramp arm_sdk authority to 1.0 only if it isn't already engaged.

        Re-ramping while engaged would drop the weight to 0 for a moment and
        hand the arm to the default controller (a snap).  The last weight
        this process published is checked too, so a release elsewhere in the
        dashboard (/control, /wbc) re-arms the ramp.  Hold arm_lock.
        """
        weight = float(self.robot._arm_sdk_client().msg.motor_cmd[29].q)
        if self._authority_engaged and weight >= 0.999:
            self.log("[grasp] Arm authority already engaged; skipping re-ramp.")
            return
        self.log("[grasp] Re-engaging arm SDK control.")
        result = self.robot.engage_arms(duration_s=duration_s)
        self.log(f"[grasp] engage_arms() done: {result}")
        self._authority_engaged = True

    def stop_active_hold(self) -> None:
        executor, self.hold_executor = self.hold_executor, None
        if executor is not None:
            try:
                executor.stop_continuous()
            except Exception:
                pass

    def hold_current_arm_pose(self, arm: str) -> None:
        """Keep publishing the measured current joints (no authority gap)."""
        self.stop_active_hold()
        q_current = self.read_current_arm_q(arm)
        executor = ArmExecutor(self.robot, arm=arm, kp=30.0, kd=1.5, max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
                               max_joint_step_rad=DEFAULT_MAX_JOINT_STEP_RAD,
                               max_joint_speed_rad_s=DEFAULT_MAX_JOINT_SPEED_RAD_S, read_arm_q=self.read_current_arm_q)
        executor.start_continuous()
        executor.submit_target(q_current)
        self.hold_executor = executor
        self.log(f"[grasp] Holding current {arm} arm joints.")

    def shutdown_nav(self, nav: Optional[HandApproach], context: str) -> None:
        if nav is None:
            return
        try:
            nav.shutdown()
        except Exception as exc:
            self.log(f"[grasp] {context} approach shutdown failed: {exc}")
        finally:
            with self.nav_lock:
                if self.active_nav is nav:
                    self.active_nav = None

    def _executor(self, arm: str, step: float, speed: float, kp: float = 30.0, kd: float = 1.5) -> ArmExecutor:
        return ArmExecutor(self.robot, arm=arm, kp=kp, kd=kd, max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
                           max_joint_step_rad=step, max_joint_speed_rad_s=speed, read_arm_q=self.read_current_arm_q)

    def move_waist(self, target_yaw: float, max_speed_rad_s: float) -> Dict[str, Any]:
        speed = min(DEFAULT_CENTER_WAIST_SPEED_RAD_S, max(0.05, float(max_speed_rad_s)))
        return self.robot.move_upper_body_joint(WAIST_YAW_JOINT_INDEX, float(target_yaw), max_joint_speed=speed)

    def run_extend_arm_ee_ik(self, arm: str, step: float, speed: float, cam: Dict[str, float]) -> Dict[str, Any]:
        self.ensure_arm_authority(duration_s=0.4)
        q_cur = self.read_current_arm_q(arm)
        static_obstacles = self.estimate_obstacles(cam)
        opposite_arm = "left" if arm == "right" else "right"
        fk = ArmFK(arm=arm, backend="urdf")
        executor = self._executor(arm, step, speed)
        results = []

        def stage_obstacles(table_margin_m: float = TABLE_COLLISION_MARGIN_M) -> Obstacles:
            T_opposite = self.hand_T(opposite_arm)
            return Obstacles(table=static_obstacles.table, table_margin_m=table_margin_m,
                             table_xy_margin_m=TABLE_COLLISION_XY_MARGIN_M,
                             opposite_shoulder_base=SHOULDER_IN_BASE[opposite_arm] if T_opposite is not None else None,
                             opposite_wrist_base=T_opposite[:3, 3].copy() if T_opposite is not None else None)

        # A low robot-side rest pose could sweep through the desk on the way
        # into the recipe: lift the current EE above the table first.
        if static_obstacles.table is not None:
            T_current = fk.compute_arm(q_cur)
            clearance_z = float(static_obstacles.table.z_base_m + TABLE_COLLISION_MARGIN_M + 0.10)
            if float(T_current[2, 3]) < clearance_z:
                T_preclear = T_current.copy()
                T_preclear[2, 3] = clearance_z
                q_preclear, info = ArmIK(arm=arm, solver="position_dls", tol_pos_m=0.012,
                                         tol_rot_rad=math.pi).solve(T_preclear, q_init=q_cur)
                if q_preclear is None:
                    return {"success": False, "reason": "preclear_ik_failed", "ik_info": info, "stage_results": results}
                # The preclear runs because the hand starts low, often inside
                # the table buffer: checked against the full buffer its first
                # sample (the current pose) is rejected.  As for the post-grasp
                # lift, keep 1.5 cm below the start clearance from then on.
                start_clearance_m = float(T_current[2, 3] - static_obstacles.table.z_base_m)
                preclear_margin_m = min(TABLE_COLLISION_MARGIN_M, max(0.03, start_clearance_m - 0.015))
                if preclear_margin_m < TABLE_COLLISION_MARGIN_M:
                    self.log(f"[grasp] preclear starts inside the table buffer; using {preclear_margin_m:.3f}m "
                             f"path margin from {start_clearance_m:.3f}m clearance.")
                result = executor.execute(q_preclear, duration_s=DEFAULT_EXTEND_STAGE_DURATION_S, q_arm_start=q_cur,
                                          T_base_desired=T_preclear, stop_event=self.cancel,
                                          obstacles=stage_obstacles(preclear_margin_m))
                results.append({"stage": "vertical_table_preclear", "execute": result})
                self.log(f"[grasp] extend vertical_table_preclear: success={result.get('success')} reason={result.get('reason', '')}")
                if not result.get("success"):
                    return {"success": False, "reason": result.get("reason"), "stage_results": results}
                q_cur = np.asarray(result.get("final_q", q_preclear), dtype=np.float64)
        for stage_name, q_stage in extend_arm_forward_joint_waypoints(arm):
            if self.cancel.is_set():
                return {"success": False, "reason": "cancelled", "stage_results": results}
            result = executor.execute(q_stage, duration_s=DEFAULT_EXTEND_STAGE_DURATION_S, q_arm_start=q_cur,
                                      T_base_desired=fk.compute_arm(q_stage), stop_event=self.cancel,
                                      obstacles=stage_obstacles())
            results.append({"stage": stage_name, "execute": result})
            self.log(f"[grasp] extend stage {stage_name}: success={result.get('success')} "
                     f"reason={result.get('reason', '')} duration={result.get('duration_s', 0.0):.2f}s "
                     f"final_err={result.get('final_error_rad', 0.0):.4f}rad violations={result.get('violations', [])}")
            if not result.get("success"):
                return {"success": False, "reason": result.get("reason"), "stage_results": results}
            q_cur = np.asarray(result.get("final_q", q_stage), dtype=np.float64)
        return {"success": True, "arm": arm, "controller": "ee_ik", "final_q": q_cur.copy(),
                "stages": [item["stage"] for item in results]}

    def save_extend_arm_reference(self, arm: str, q_ref: np.ndarray, source: str) -> None:
        q_ref = np.asarray(q_ref, dtype=np.float64).copy()
        T_ref = ArmFK(arm=arm, backend="urdf").compute_arm(q_ref)
        with self.lock:
            self.extended_hand_reference_q[arm] = q_ref.copy()
            self.extended_hand_reference_T_base[arm] = T_ref.copy()
        self.log(f"[grasp] saved {arm} extend-arm reference from {source}: "
                 f"xyz=({T_ref[0, 3]:+.3f}, {T_ref[1, 3]:+.3f}, {T_ref[2, 3]:+.3f}) m.")

    def forget_extend_reference(self, arm: str) -> None:
        with self.lock:
            self.extended_hand_reference_q.pop(arm, None)
            self.extended_hand_reference_T_base.pop(arm, None)

    def reset_to_extend_arm_reference(self, arm: str, step: float, speed: float) -> Dict[str, Any]:
        with self.lock:
            q_ref = self.extended_hand_reference_q.get(arm)
            T_ref = self.extended_hand_reference_T_base.get(arm)
        if q_ref is None or T_ref is None:
            return {"success": False, "reason": "no_extend_arm_reference", "arm": arm}
        result = self._executor(arm, step, speed).execute(
            q_ref, duration_s=DEFAULT_EXTEND_STAGE_DURATION_S, q_arm_start=self.read_current_arm_q(arm),
            T_base_desired=T_ref, stop_event=self.cancel)
        result["controller"] = "saved_extend_arm_reference"
        return result

    def wbc_right_arm_forward_q(self) -> np.ndarray:
        """The reviewed stable right-arm pose maintained by the WBC tool."""
        try:
            poses = json.loads(self.stable_pose_file.read_text(encoding="utf-8")).get("poses", [])
        except Exception as exc:
            raise RuntimeError(f"cannot read WBC pose file {self.stable_pose_file}: {exc}") from exc
        for pose in poses:
            if str(pose.get("name", "")).strip().lower() != WBC_RIGHT_ARM_FORWARD_POSE_NAME:
                continue
            joints = pose.get("arm_joints", {})
            try:
                return np.asarray([float(joints[str(i)]) for i in RIGHT_ARM_JOINTS], dtype=np.float64)
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(f"WBC pose {WBC_RIGHT_ARM_FORWARD_POSE_NAME!r} lacks right arm joints: {exc}") from exc
        raise RuntimeError(f"WBC pose {WBC_RIGHT_ARM_FORWARD_POSE_NAME!r} was not found in {self.stable_pose_file}")

    def move_to_wbc_right_arm_forward(self, step: float, speed: float, obstacles: Optional[Obstacles]) -> Dict[str, Any]:
        q_target = self.wbc_right_arm_forward_q()
        result = self._executor("right", step, speed).execute(
            q_target, duration_s=DEFAULT_POST_GRASP_LIFT_DURATION_S, q_arm_start=self.read_current_arm_q("right"),
            T_base_desired=ArmFK(arm="right", backend="urdf").compute_arm(q_target), stop_event=self.cancel,
            obstacles=obstacles)
        result["controller"] = "WBC right arm forward"
        return result

    def move_to_wbc_right_arm_forward_continuous(self, executor: ArmExecutor, speed: float,
                                                 obstacles: Optional[Obstacles]) -> Dict[str, Any]:
        """Retarget an already-publishing hold executor without a control gap."""
        q_target = self.wbc_right_arm_forward_q()
        q_current = self.read_current_arm_q("right")
        validation = executor.validate(q_target, T_base_desired=ArmFK(arm="right", backend="urdf").compute_arm(q_target),
                                       q_arm_start=q_current, obstacles=obstacles)
        if not validation.get("safe"):
            return {"success": False, "reason": validation.get("reason"), "violations": validation.get("violations")}
        executor.submit_target(q_target)
        duration_s = max(DEFAULT_POST_GRASP_LIFT_DURATION_S,
                         float(np.max(np.abs(q_target - q_current))) / max(0.05, float(speed)) + 0.5)
        deadline = time.time() + duration_s
        while time.time() < deadline:
            if self.cancel.is_set():
                return {"success": False, "reason": "cancelled"}
            time.sleep(0.05)
        return {"success": True, "controller": "continuous WBC right arm forward", "duration_s": duration_s}

    def move_ee_to_position(self, arm: str, target_position: np.ndarray, step: float, speed: float,
                            obstacles: Optional[Obstacles], label: str) -> Dict[str, Any]:
        """Position-first Cartesian move; wrist orientation stays reachable."""
        q_current = self.read_current_arm_q(arm)
        T_target = ArmFK(arm=arm, backend="urdf").compute_arm(q_current)
        T_target[:3, 3] = np.asarray(target_position, dtype=np.float64)
        q_target, info = ArmIK(arm=arm, solver="position_dls", tol_pos_m=0.012, tol_rot_rad=math.pi).solve(T_target, q_init=q_current)
        if q_target is None:
            return {"success": False, "reason": "ik_failed", "ik_info": info, "label": label}
        result = self._executor(arm, step, speed).execute(q_target, duration_s=DEFAULT_POST_GRASP_LIFT_DURATION_S,
                                                          q_arm_start=q_current, T_base_desired=T_target,
                                                          stop_event=self.cancel, obstacles=obstacles)
        result.update({"label": label, "ik_info": info})
        return result

    def lift_end_effector_z_slowly(self, arm: str, step: float, speed: float, lift_m: float = DEFAULT_POST_GRASP_LIFT_M,
                                   mass_kg: float = 0.0, obstacles: Optional[Obstacles] = None) -> Dict[str, Any]:
        if lift_m <= 0.0:
            return {"success": True, "reason": "lift_disabled"}
        kp, kd, duration_s, speed_scale = lift_tuning_for_mass(mass_kg)
        q_cur = self.read_current_arm_q(arm)
        T_lift = ArmFK(arm=arm, backend="urdf").compute_arm(q_cur)
        T_lift[2, 3] += float(lift_m)
        q_solution, info = ArmIK(arm=arm, solver="dls", tol_pos_m=0.010, tol_rot_rad=0.20).solve(T_lift, q_init=q_cur)
        if q_solution is None:
            return {"success": False, "reason": "ik_failed", "ik_info": info}
        result = self._executor(arm, step, speed * speed_scale, kp=kp, kd=kd).execute(
            q_solution, duration_s=duration_s, q_arm_start=q_cur, T_base_desired=T_lift, stop_event=self.cancel,
            obstacles=obstacles)
        result.update({"ik_info": info, "lift_m": float(lift_m), "mass_kg": float(mass_kg)})
        return result

    def lift_end_effector_z_with_continuous_executor(self, arm: str, executor: ArmExecutor,
                                                     lift_m: float = DEFAULT_POST_GRASP_LIFT_M, mass_kg: float = 0.0,
                                                     obstacles: Optional[Obstacles] = None) -> Dict[str, Any]:
        if lift_m <= 0.0:
            return {"success": True, "reason": "lift_disabled"}
        kp, kd, duration_s, speed_scale = lift_tuning_for_mass(mass_kg)
        # The executor reads kp/kd/speed live at publish time.
        executor.kp, executor.kd = kp, kd
        executor.max_joint_speed_rad_s *= speed_scale
        q_cur = self.read_current_arm_q(arm)
        T_start = ArmFK(arm=arm, backend="urdf").compute_arm(q_cur)
        T_lift = T_start.copy()
        T_lift[2, 3] += float(lift_m)
        q_solution, info = ArmIK(arm=arm, solver="dls", tol_pos_m=0.010, tol_rot_rad=0.20).solve(T_lift, q_init=q_cur)
        if q_solution is None:
            return {"success": False, "reason": "ik_failed", "ik_info": info}
        # A confirmed grasp may start just inside the conservative table
        # margin; for this vertical move only, keep >=1.5 cm below the start
        # clearance and still check every later sample.
        lift_obstacles = obstacles
        if obstacles is not None and obstacles.table is not None:
            start_clearance_m = float(T_start[2, 3] - obstacles.table.z_base_m)
            margin = min(float(obstacles.table_margin_m), max(0.03, start_clearance_m - 0.015))
            if margin < float(obstacles.table_margin_m):
                lift_obstacles = replace(obstacles, table_margin_m=margin)
                self.log(f"[grasp] vertical lift starts inside the table buffer; using {margin:.3f}m lift-path "
                         f"margin from {start_clearance_m:.3f}m measured clearance.")
        validation = executor.validate(q_solution, T_base_desired=T_lift, q_arm_start=q_cur, obstacles=lift_obstacles)
        if not validation.get("safe"):
            return {"success": False, "reason": validation.get("reason"), "violations": validation.get("violations"),
                    "ik_info": info}
        executor.submit_target(q_solution)
        t0 = time.time()
        while time.time() - t0 < duration_s:
            if self.cancel.is_set():
                return {"success": False, "reason": "stopped", "duration_s": time.time() - t0}
            time.sleep(0.05)
        return {"success": True, "controller": "continuous_executor", "duration_s": time.time() - t0,
                "lift_m": float(lift_m), "mass_kg": float(mass_kg)}

    def run_confirm_lift(self) -> None:
        """Lift vertically, then move from the held grasp to the WBC stable pose."""
        with self.lock:
            pending, arm = self.lift_confirmation_pending, self.lift_arm
            cam = dict(self.camera_extrinsic)
            speed = self.max_joint_speed_rad_s
            mass_kg = self.object_mass_kg
        if not pending or arm not in ("left", "right"):
            self.log("[grasp] Lift ignored: no held object is awaiting a lift.")
            return
        if not self.arm_lock.acquire(blocking=False):
            self.log("[grasp] Lift ignored: arm controller is busy.")
            return
        try:
            self._busy("lift to stable pose")
            with self.nav_lock:
                nav = self.active_nav
            # Hold the measured pose *before* stopping the approach loop so
            # there is no arm_sdk authority gap (the arm sagged toward the desk).
            self.hold_current_arm_pose(arm)
            self.shutdown_nav(nav, "before confirmed lift")
            obstacles = self.estimate_obstacles(cam)
            hold = self.hold_executor
            if hold is None:
                raise RuntimeError("could not establish continuous current-pose hold")
            lift = self.lift_end_effector_z_with_continuous_executor(arm, hold, mass_kg=mass_kg, obstacles=obstacles)
            if not lift.get("success"):
                raise RuntimeError(f"vertical clearance lift failed: {lift}")
            if arm != "right":
                raise RuntimeError("WBC stable forward pose is currently defined for the right arm only")
            result = self.move_to_wbc_right_arm_forward_continuous(hold, speed, obstacles)
            self.log(f"[grasp] lift to WBC right arm forward: success={result.get('success')} reason={result.get('reason', '')}")
            with self.lock:
                if result.get("success"):
                    self.lift_confirmation_pending = False
                    self.lift_arm = None
                    self.lift_unverified = False
                    self.status_msg = "Lift complete: object moved to WBC right arm forward pose."
                else:
                    self.status_msg = f"Lift to WBC forward pose blocked/failed: {result.get('reason')}"
        except Exception as exc:
            self.log(f"[grasp] lift to WBC forward pose failed: {exc}")
            with self.lock:
                self.status_msg = f"Lift to WBC forward pose failed: {exc}"
        finally:
            self._idle()
            self.arm_lock.release()

    def _busy(self, label: str) -> None:
        with self.lock:
            self.arm_motion_running = True
            self.arm_motion_label = label

    def _idle(self) -> None:
        with self.lock:
            self.arm_motion_running = False
            self.arm_motion_label = ""

    def clear_hold_flags(self) -> None:
        with self.lock:
            self.lift_confirmation_pending = False
            self.lift_arm = None
            self.lift_unverified = False
            self.holding_object = False
            self.holding_arm = None

    # -- hands ---------------------------------------------------------------

    def hand_snapshot(self, side: str, max_age_s: float = 1.0) -> Optional[Dict[str, Any]]:
        try:
            snap = self.robot.get_dex3_hand_sensors(side)
        except Exception:
            return None
        if not isinstance(snap, dict) or "error" in snap or snap.get("timestamp") is None:
            return None
        return snap if time.time() - float(snap["timestamp"]) <= max_age_s else None

    def tactile_pressures(self, side: str) -> Optional[List[List[float]]]:
        snap = self.hand_snapshot(side, max_age_s=2.0)
        return None if snap is None else [list(sensor) for sensor in snap.get("tactile_pressures") or []]

    def max_tactile_pressure(self, side: str) -> Optional[float]:
        sensors = self.tactile_pressures(side)
        if not sensors:
            return None
        valid = [float(v) for sensor in sensors for v in sensor if not _is_invalid_tactile_value(v)]
        return max(valid) if valid else 0.0

    def tactile_status_text(self) -> str:
        lines = []
        for side in ("left", "right"):
            sensors = self.tactile_pressures(side)
            if sensors is None:
                lines.append(f"{side}: waiting for Dex3 state")
                continue
            parts = []
            for index, values in enumerate(sensors):
                valid = [float(v) for v in values if not _is_invalid_tactile_value(v)]
                if not valid:
                    parts.append(f"S{index}=--")
                    continue
                peak = max(valid)
                parts.append(f"S{index}={peak:.0f}({'CONTACT' if peak >= TACTILE_VALID_THRESHOLD else 'idle'})")
            overall = self.max_tactile_pressure(side)
            lines.append(f"{side}: peak={'--' if overall is None else f'{overall:.0f}'} raw | "
                         + "  ".join(parts or ["no pressure sensors"]))
        return "\n".join(lines)

    def _hand_target_error(self, side: str, target: List[float]) -> Optional[float]:
        snap = self.hand_snapshot(side)
        if snap is None:
            return None
        try:
            actual = [float(snap["positions"][name]) for name in HAND_JOINT_NAMES]
        except (KeyError, TypeError, ValueError):
            return None
        return max(abs(goal - cur) for goal, cur in zip(target, actual))

    def hand_stalled_joints(self, side: str, target: List[float], start: Optional[List[float]] = None) -> List[str]:
        """Joints asked to move far, still far from the command, and not moving."""
        snap = self.hand_snapshot(side)
        if snap is None:
            return []
        try:
            actual = [float(snap["positions"][n]) for n in HAND_JOINT_NAMES]
            speed = [float(snap["velocities"][n]) for n in HAND_JOINT_NAMES]
        except (KeyError, TypeError, ValueError):
            return []
        if start is None or len(start) != len(actual):
            start = list(actual)
        return [name for name, req, cur, vel, initial in zip(HAND_JOINT_NAMES, target, actual, speed, start)
                if abs(float(req) - cur) >= DEX3_STALL_ERROR_RAD and abs(vel) <= DEX3_STALL_SPEED_RAD_S
                and abs(float(req) - float(initial)) >= DEX3_STALL_ERROR_RAD]

    def release_stalled_fingers(self, side: str, joints: List[str], context: str) -> None:
        """Send Dex3's release bit (zero gains) after a confirmed jam."""
        try:
            self.robot.release_dex3_fingers(hand=side, hold_s=DEX3_STALL_CONFIRM_S, rate_hz=50.0, persistent=False)
            self.log(f"[grasp] DEX3 stall during {context}: released {side} fingers (stalled joints: {', '.join(joints)}).")
        except Exception as exc:
            self.log(f"[grasp] DEX3 stall release failed during {context} ({side}): {exc}")

    def send_hand_action(self, action: str, side: str, hold_s: Optional[float] = None) -> bool:
        target = list(HAND_SAFE_OPEN[side] if action == "open" else HAND_CLOSED[side])
        feedback = self.hand_snapshot(side) is not None
        if not feedback:
            self.log(f"[grasp] {side} Dex3 state feedback unavailable; sending hand_{action} anyway.")
        command_hold_s = DEFAULT_HAND_COMMAND_HOLD_S if hold_s is None else float(hold_s)
        if action == "open":
            result = self.robot.open_dex3_hand(hand=side, hold_s=command_hold_s, ramp_s=0.25)
        else:
            result = self.robot.close_dex3_hand(hand=side, hold_s=command_hold_s, ramp_s=0.25 if hold_s is None else 0.20)
        if isinstance(result, dict) and result.get("reason") == "collision_or_stall":
            self.log(f"[grasp] hand_{action}({side}) stalled at {result.get('joint')}; fingers released.")
        if action == "open" or hold_s is None:
            err = self._hand_target_error(side, target)
            if err is not None and err > DEFAULT_HAND_TARGET_TOL_RAD:
                stalled = self.hand_stalled_joints(side, target)
                if stalled:
                    self.release_stalled_fingers(side, stalled, f"hand_{action} recovery")
                self.log(f"[grasp] hand_{action}({side}) missed target (max_err={err:.3f}rad); retrying explicit pose.")
                self.robot.hand_pose(target, hand=side, hold_s=DEFAULT_HAND_RECOVERY_HOLD_S, rate_hz=50.0,
                                     kp=1.8, kd=0.12, tau=0.08, ramp_s=0.25)
                retry_err = self._hand_target_error(side, target)
                if retry_err is not None and retry_err > DEFAULT_HAND_TARGET_TOL_RAD:
                    self.log(f"[grasp] hand_{action}({side}) still off target (max_err={retry_err:.3f}rad).")
        return feedback

    def run_hand_action(self, action: str, side: str = "right") -> None:
        try:
            feedback = self.send_hand_action(action, side)
            with self.lock:
                if action == "open" and self.holding_arm == side:
                    self.holding_object = False
                    self.holding_arm = None
                    self.lift_confirmation_pending = False
                    self.lift_unverified = False
                self.status_msg = f"hand_{action}({side}) sent{'' if feedback else ' (feedback unavailable)'}."
        except Exception as exc:
            with self.lock:
                self.status_msg = f"hand_{action}({side}) failed: {exc}"

    def tactile_guided_hand_close(self, side: str, mass_kg: float) -> Dict[str, Any]:
        """Close in small steps and stop once the mass-scaled fingertip
        pressure is reached; one fixed close when tactile data is missing."""
        open_t, closed_t = list(HAND_SAFE_OPEN[side]), list(HAND_CLOSED[side])
        if self.tactile_pressures(side) is None:
            self.log(f"[grasp] Tactile-guided close unavailable for {side}; falling back to fixed close.")
            self.send_hand_action("close", side, hold_s=0.6)
            return {"success": True, "method": "fixed_close", "side": side}
        target_pressure = grip_target_pressure_raw(mass_kg)
        self.log(f"[grasp] Tactile-guided close ({side}): object_mass={mass_kg:.2f}kg target_pressure={target_pressure:.0f} raw.")
        start_snap = self.hand_snapshot(side)
        close_start: Optional[List[float]] = None
        if start_snap is not None:
            try:
                close_start = [float(start_snap["positions"][n]) for n in HAND_JOINT_NAMES]
            except (KeyError, TypeError, ValueError):
                close_start = None
        steps = DEFAULT_TACTILE_CLOSE_STEPS
        last_pressure = last_fraction = 0.0
        result: Dict[str, Any]
        for step in range(1, steps + 1):
            if self.cancel.is_set():
                self.log(f"[grasp] Tactile-guided close cancelled at step {step}/{steps}.")
                return {"success": False, "method": "tactile", "reason": "cancelled", "side": side}
            fraction = step / float(steps)
            target_positions = [o + fraction * (c - o) for o, c in zip(open_t, closed_t)]
            try:
                command = self.robot.hand_pose(target_positions, hand=side, hold_s=DEFAULT_TACTILE_STEP_HOLD_S,
                                               rate_hz=50.0, kp=1.4, kd=0.06, tau=0.06, ramp_s=0.06)
            except Exception as exc:
                self.log(f"[grasp] Tactile-guided close step {step} command failed: {exc}")
                break
            time.sleep(DEFAULT_TACTILE_STEP_SETTLE_S)
            pressure = self.max_tactile_pressure(side)
            last_fraction = fraction
            if pressure is not None:
                last_pressure = pressure
            self.log(f"[grasp] grip step {step}/{steps} fraction={fraction:.2f} "
                     f"pressure={'n/a' if pressure is None else f'{pressure:.0f}'} raw target={target_pressure:.0f} raw")
            if pressure is not None and pressure >= target_pressure:
                self.log(f"[grasp] Grip target reached ({side}): fraction={fraction:.2f} pressure={pressure:.0f} raw.")
                result = {"success": True, "method": "tactile", "side": side, "fraction": fraction,
                          "pressure": pressure, "target_pressure": target_pressure, "mass_kg": mass_kg}
                with self.lock:
                    self.last_grip_result = dict(result)
                return result
            # Without tactile confirmation, a finger that stopped well short
            # of its command is jammed: release it instead of pushing harder.
            stalled = self.hand_stalled_joints(side, target_positions, close_start)
            if isinstance(command, dict) and command.get("reason") == "collision_or_stall" and not stalled:
                stalled = [str(command.get("joint"))]
            if stalled:
                self.release_stalled_fingers(side, stalled, "tactile-guided close")
                result = {"success": False, "method": "dex3_stall_release", "reason": "collision_or_stall", "side": side,
                          "stalled_joints": stalled, "fraction": fraction, "pressure": pressure,
                          "target_pressure": target_pressure, "mass_kg": mass_kg}
                with self.lock:
                    self.last_grip_result = dict(result)
                return result
        self.log(f"[grasp] Tactile-guided close reached full closure without hitting target pressure ({side}); "
                 f"last_pressure={last_pressure:.0f} raw target={target_pressure:.0f} raw.")
        # A fully closed hand with no measured contact cannot confirm a grasp.
        result = {"success": False, "method": "tactile_full_close_no_contact", "side": side, "fraction": last_fraction,
                  "pressure": last_pressure, "target_pressure": target_pressure, "mass_kg": mass_kg}
        with self.lock:
            self.last_grip_result = dict(result)
        return result

    # -- grab ----------------------------------------------------------------

    def prepare_hand_visibility_for_grab(self, arm: str, cam: Dict[str, float], step: float, speed: float) -> bool:
        with self.lock:
            q_ref = self.extended_hand_reference_q.get(arm)
        if q_ref is not None:
            self.log("[grasp] Configured extend pose is already active; opening hand before grab.")
            self.send_hand_action("open", arm)
            return True
        self.log("[grasp] Arm is not at the configured extend pose; extending before grab.")
        if not self.arm_lock.acquire(blocking=False):
            self.log("[grasp] Waiting for current arm motion before pre-grab extend.")
            self.arm_lock.acquire()
        try:
            if self.cancel.is_set():
                self.log("[grasp] Pre-grab extend cancelled before control handoff.")
                return False
            self._busy("pre-grab extend")
            result = self.run_extend_arm_ee_ik(arm, step, speed, cam)
            if not result.get("success"):
                self.log(f"[grasp] Pre-grab extend failed: {result.get('reason')}")
                return False
            self.save_extend_arm_reference(arm, result["final_q"], "pre-grab extend")
            self.send_hand_action("open", arm)
            self.log(f"[grasp] Opened {arm} hand after pre-grab extend.")
            return True
        finally:
            self._idle()
            self.arm_lock.release()

    def run_direct_grab_inline(self, det: Detection, arm: str, cam: Dict[str, float], T_base_object: np.ndarray,
                               step: float, speed: float, mass_kg: float = 0.0) -> None:
        """Approach the object and close the hand; on success the approach
        loop keeps holding the grasp pose and lift_confirmation_pending is set."""
        if not self.arm_lock.acquire(blocking=False):
            self.log("[grasp] Waiting for current arm motion to finish before grab control.")
            self.arm_lock.acquire()
        try:
            if self.cancel.is_set():
                self.log("[grasp] Direct grab cancelled before control handoff.")
                return
            self.stop_active_hold()
            with self.lock:
                self.arm_motion_running = True
                self.arm_motion_label = "direct grab"
                self.lift_confirmation_pending = False
                self.lift_arm = arm
                self.lift_unverified = False
                self.holding_object = False
                self.holding_arm = None
            obstacles = self.estimate_obstacles(cam)
            _arm, reach, max_reach, excess = self.reach_preview(det, cam, arm, obstacles)
            T_base_camera = base_from_camera(cam)
            if excess > 0.0:
                self.log(f"[grasp] target outside arm workspace: reach_dist={reach:.3f} m max={max_reach:.3f} m "
                         f"excess={excess:.3f} m; move closer.")
                return
            # The extend pose is a safe starting posture, not an orientation
            # constraint.  Vision grabs keep the palm level (yaw free) so the
            # fingers close parallel to the table instead of into it.
            T_base_hand = direct_grasp_hand_pose(det, T_base_object, arm, obstacles)
            vision = det.source == "vision"
            config = {
                "arm": arm, "ik_solver": "level_dls" if vision else "dls", "rate_hz": DEFAULT_SLOW_GRAB_RATE_HZ,
                "timeout_s": DEFAULT_SLOW_GRAB_TIMEOUT_S, "ik_tol_pos_m": 0.040 if vision else 0.003,
                "ik_tol_rot_rad": GRASP_PALM_LEVEL_TOL_RAD if vision else 0.01,
                "convergence_pos_m": DEFAULT_VISION_GRAB_CONVERGENCE_POS_M if vision else 0.015,
                "convergence_rot_rad": GRASP_PALM_LEVEL_CONVERGENCE_RAD if vision else 0.05, "max_joint_step_rad": step,
                "max_joint_speed_rad_s": speed, "max_reach_m": DEFAULT_DIRECT_MAX_REACH_M,
            }
            p_obj, p_hand = T_base_object[:3, 3], T_base_hand[:3, 3]
            self.log(f"[grasp] object_base_xyz=({p_obj[0]:+.3f}, {p_obj[1]:+.3f}, {p_obj[2]:+.3f}) m")
            self.log(f"[grasp] desired hand xyz=({p_hand[0]:+.3f}, {p_hand[1]:+.3f}, {p_hand[2]:+.3f}) m "
                     "with stable robot-side approach; " + ("palm kept level (fingers close parallel to the table)."
                                                            if vision else "full 6-D hand pose."))
            if vision:
                # Fail in ~1 s instead of a timed-out approach when no level
                # hand pose reaches the target.
                _q, level_info = ArmIK(arm=arm, solver="level_dls", tol_pos_m=config["ik_tol_pos_m"],
                                       tol_rot_rad=GRASP_PALM_LEVEL_TOL_RAD).solve(
                    T_base_hand, q_init=self.read_current_arm_q(arm))
                if not level_info.get("success"):
                    self.log(f"[grasp] no level-palm hand pose reaches the target (best pos_err="
                             f"{level_info['error_pos_m']:.3f} m, tilt={math.degrees(level_info['error_rot_rad']):.1f} deg); "
                             "move the object or the robot closer.")
                    return
            with self.lock:
                dynamic = self.dynamic_target_correction and vision
            self.log("[grasp] fresh detections of the same object will correct the target during the approach "
                     "(bounded, frozen near the object)." if dynamic else
                     "[grasp] camera detections will not retarget the arm mid-grab.")
            self.log(f"[grasp] slow grab config: rate={config['rate_hz']:.1f}Hz max_joint_step={step:.3f}rad "
                     f"max_joint_speed={speed:.3f}rad/s timeout={config['timeout_s']:.1f}s")
            try:
                self.ensure_arm_authority(duration_s=0.5)
            except Exception as exc:
                self.log(f"[grasp] engage_arms() before grab failed: {exc}")
            nav = HandApproach(self, T_base_hand, config, obstacles)
            with self.nav_lock:
                self.active_nav = nav
            corrector = (GrabTargetCorrector(self, det, arm, T_base_camera, T_base_object, obstacles,
                                             candidates=self.correction_candidates) if dynamic else None)
            ok = stalled = False
            last_status: Dict[str, Any] = {}
            last_log_t = 0.0
            try:
                deadline = time.time() + float(config["timeout_s"]) + 2.0
                while time.time() < deadline:
                    if self.cancel.is_set():
                        self.log("[grasp] Direct grab cancelled by safety command.")
                        break
                    status = nav.status_snapshot()
                    last_status = status
                    if time.time() - last_log_t >= 1.0:
                        last_log_t = time.time()
                        loop_log = status.get("log") or []
                        self.log(f"[grasp] approach: iter={status.get('iteration')} converged={status.get('converged')} "
                                 f"pos_err={status.get('last_error_pos_m')} ik_failures={status.get('ik_failures')} "
                                 f"safety_rejections={status.get('safety_rejections')} last={loop_log[-1] if loop_log else ''}")
                    if status.get("converged"):
                        ok = True
                        break
                    if status.get("stalled"):
                        stalled = True
                        self.log("[grasp] Direct grab stalled: target has not gotten closer over the last several "
                                 "EE-pose increments - checking whether the hand is close enough to grasp.")
                        break
                    if not status.get("running", True):
                        self.log(f"[grasp] approach stopped: {(status.get('log') or [''])[-1]}")
                        break
                    if corrector is not None:
                        try:
                            pos_err = status.get("last_error_pos_m")
                            message = corrector.step(nav, None if pos_err is None else float(pos_err))
                            if message:
                                self.log(f"[grasp] {message}")
                        except Exception as exc:
                            self.log(f"[grasp] dynamic target correction disabled after error: {exc}")
                            corrector = None
                    time.sleep(0.2)
            finally:
                nav_holding = bool(ok or stalled)
                if not nav_holding:
                    self.shutdown_nav(nav, "direct grab")
            if stalled:
                try:
                    pos_err = float(last_status.get("last_error_pos_m"))
                except (TypeError, ValueError):
                    pos_err = float("inf")
                if vision and pos_err <= DEFAULT_VISION_STALL_CLOSE_POS_M:
                    self.log(f"[grasp] Direct grab stalled at visually close target (pos_err={pos_err:.4f}m); "
                             "attempting tactile-verified close.")
                    ok = True
                else:
                    self.log("[grasp] Direct grab stalled without a close-enough pose; not closing the hand and "
                             "returning to the extend pose.")
            if stalled and not ok:
                self.shutdown_nav(nav, "direct grab before stall reset")
                if self.cancel.is_set():
                    return
                try:
                    reset = self.reset_to_extend_arm_reference(arm, step, speed)
                    self.log(f"[grasp] stall reset to saved extend-arm reference: success={reset.get('success')} "
                             f"reason={reset.get('reason', '')}")
                except Exception as exc:
                    self.log(f"[grasp] stall reset to saved extend-arm reference failed: {exc}")
                return
            if not ok:
                self.log("[grasp] Direct grab did not converge; not closing hand.")
                self.shutdown_nav(nav, "direct grab")
                return
            self.log("[grasp] Direct grab reached an accepted close pose; holding it while closing the hand (tactile-guided).")
            try:
                grip = self.tactile_guided_hand_close(arm, mass_kg)
                closed = bool(grip.get("success"))
                unverified = grip.get("method") == "tactile_full_close_no_contact"
                self.log(f"[grasp] Closed {arm} hand: {grip}")
                if not closed and not unverified:
                    self.log("[grasp] Hand close was unavailable; not lifting.")
                    self.shutdown_nav(nav, "direct grab")
                    return
            except Exception as exc:
                self.log(f"[grasp] Hand close failed: {exc}")
                self.shutdown_nav(nav, "direct grab")
                return
            if self.cancel.is_set():
                self.log("[grasp] Lift skipped because the grab was cancelled.")
                self.shutdown_nav(nav, "direct grab")
                return
            # The approach loop's continuous executor keeps holding the grasp
            # pose until run_confirm_lift() takes over.
            with self.lock:
                self.lift_confirmation_pending = True
                self.lift_arm = arm
                self.lift_unverified = unverified
                self.holding_object = True
                self.holding_arm = arm
                self.status_msg = ("Hand closed without tactile confirmation." if unverified else "Object grasped.")
            self.log(f"[grasp] Hand closed (tactile_verified={not unverified}).")
        finally:
            self._idle()
            self.arm_lock.release()

    # -- safety actions ------------------------------------------------------

    def _acquire_for_safety(self, label: str) -> None:
        if not self.arm_lock.acquire(timeout=3.0):
            with self.lock:
                self.status_msg = f"{label} waiting: arm controller is busy"
            self.arm_lock.acquire()

    def run_release_arms(self) -> None:
        self.cancel.set()
        self.stop_active_hold()
        with self.nav_lock:
            nav = self.active_nav
        self.shutdown_nav(nav, "release arms")
        self._acquire_for_safety("release_arms()")
        try:
            self._busy("release arms")
            result = self.robot.release_arms()
            self._authority_engaged = False
            with self.lock:
                self.grab_running = False
                self.extended_hand_reference_q.clear()
                self.extended_hand_reference_T_base.clear()
                self.status_msg = f"release_arms() done: {result}"
        except Exception as exc:
            with self.lock:
                self.status_msg = f"release_arms() failed: {exc}"
        finally:
            self.clear_hold_flags()
            self._idle()
            self.cancel.clear()
            self.arm_lock.release()

    def run_engage_arms(self) -> None:
        if not self.arm_lock.acquire(blocking=False):
            with self.lock:
                self.status_msg = "engage_arms() skipped: arm controller is busy"
            return
        try:
            self.cancel.clear()
            self._busy("re-engage arms")
            result = self.robot.engage_arms()
            self._authority_engaged = True
            with self.lock:
                self.status_msg = f"engage_arms() done: {result}"
        except Exception as exc:
            with self.lock:
                self.status_msg = f"engage_arms() failed: {exc}"
        finally:
            self._idle()
            self.arm_lock.release()

    def run_damp(self) -> None:
        self.cancel.set()
        self.stop_active_hold()
        with self.nav_lock:
            nav = self.active_nav
        self.shutdown_nav(nav, "damp")
        self._acquire_for_safety("damp()")
        try:
            self._busy("damp")
            self.robot.damp_mode()
            self._authority_engaged = False
            with self.lock:
                self.status_msg = "damp() sent."
        except Exception as exc:
            with self.lock:
                self.status_msg = f"damp() failed: {exc}"
        finally:
            self._idle()
            self.cancel.clear()
            self.arm_lock.release()

    def run_stop_grabbing(self, hold_arm: str = "right") -> None:
        self.cancel.set()
        self.log("[grasp] Stop grabbing requested.")
        with self.nav_lock:
            nav = self.active_nav
        self.shutdown_nav(nav, "stop grabbing")
        self._acquire_for_safety("stop grabbing")
        try:
            with self.lock:
                self.grab_running = False
            self._busy("hold current EE pose")
            self.hold_current_arm_pose(hold_arm)
            # The held pose is not the extend pose; the next grab re-extends.
            self.forget_extend_reference(hold_arm)
            with self.lock:
                self.status_msg = "grab stopped; holding current end-effector pose (arms remain engaged)."
        except Exception as exc:
            self.log(f"[grasp] stop grabbing failed: {exc}")
            with self.lock:
                self.status_msg = f"stop grabbing failed: {exc}"
        finally:
            self._idle()
            self.cancel.clear()
            self.arm_lock.release()

    def run_retract_stable_pose(self) -> None:
        """Seamless return of the right arm to the WBC stable forward pose."""
        if not self.arm_lock.acquire(blocking=False):
            self.log("[grasp] Stable retract ignored: arm controller is busy.")
            return
        try:
            with self.lock:
                cam = dict(self.camera_extrinsic)
                speed = self.max_joint_speed_rad_s
            self._busy("retract to stable pose")
            self.ensure_arm_authority(duration_s=0.5)
            with self.nav_lock:
                nav = self.active_nav
            self.hold_current_arm_pose("right")
            self.shutdown_nav(nav, "before manual stable retract")
            obstacles = self.estimate_obstacles(cam)
            hold = self.hold_executor
            if hold is None:
                raise RuntimeError("could not establish continuous current-pose hold")
            result = self.move_to_wbc_right_arm_forward_continuous(hold, speed, obstacles)
            self.log(f"[grasp] manual retract to WBC stable pose: success={result.get('success')} reason={result.get('reason', '')}")
            with self.lock:
                self.status_msg = ("Right arm at WBC stable forward pose." if result.get("success")
                                   else f"Stable retract blocked/failed: {result.get('reason')}")
        except Exception as exc:
            self.log(f"[grasp] manual stable retract failed: {exc}")
            with self.lock:
                self.status_msg = f"Stable retract failed: {exc}"
        finally:
            self._idle()
            self.arm_lock.release()

    def busy(self) -> bool:
        with self.lock:
            return self.grab_running or self.arm_motion_running

    def yield_arms(self) -> None:
        """Stop an idle approach/pose hold so another rt/arm_sdk controller can
        take over; nothing happens while a motion is running."""
        if self.busy():
            return
        with self.nav_lock:
            nav = self.active_nav
        self.shutdown_nav(nav, "another controller took the arms")
        self.stop_active_hold()
        self._authority_engaged = False

    # -- views ---------------------------------------------------------------

    def draw_desk_obstacle_segmentation(self, rgb: np.ndarray, depth_m: Optional[np.ndarray], cam: Dict[str, float],
                                        object_masks: List[Tuple[str, str, np.ndarray]]) -> np.ndarray:
        """Green = fitted desk plane, orange = detected object masks."""
        out = (rgb.astype(np.float32) * 0.30).astype(np.uint8)
        if depth_m is None:
            cv2.putText(out, "No depth for desk segmentation", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 255), 2, cv2.LINE_AA)
            return out
        T_base_camera = base_from_camera(cam)
        try:
            table = estimate_table_plane(depth_m, self.camera_matrix(), T_base_camera)
        except Exception:
            table = None
        if table is None:
            cv2.putText(out, "No confident desk plane", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 255), 2, cv2.LINE_AA)
            return out
        K = self.K
        h, w = depth_m.shape[:2]
        ys, xs = np.indices((h, w), dtype=np.float64)
        valid = np.isfinite(depth_m) & (depth_m > 0.05)
        x_cam = (xs - K.cx) * depth_m / K.fx
        y_cam = (ys - K.cy) * depth_m / K.fy
        T = T_base_camera
        z_base = T[2, 0] * x_cam + T[2, 1] * y_cam + T[2, 2] * depth_m + T[2, 3]
        x_base = T[0, 0] * x_cam + T[0, 1] * y_cam + T[0, 2] * depth_m + T[0, 3]
        y_base = T[1, 0] * x_cam + T[1, 1] * y_cam + T[1, 2] * depth_m + T[1, 3]
        desk = valid & (np.abs(z_base - table.z_base_m) <= 0.020)
        desk &= (x_base >= table.x_range_m[0] - 0.04) & (x_base <= table.x_range_m[1] + 0.04)
        desk &= (y_base >= table.y_range_m[0] - 0.04) & (y_base <= table.y_range_m[1] + 0.04)
        out[desk] = (55, 190, 55)
        for _det_id, _label, mask in object_masks:
            if mask.shape == desk.shape:
                out[mask] = (0, 145, 255)
        cv2.putText(out, "desk: green | detected object: orange", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (245, 245, 245), 2, cv2.LINE_AA)
        return out
