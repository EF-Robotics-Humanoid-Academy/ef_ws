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
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

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
# Objects shorter than the clearance above (a coffee cup) put the fingers over
# the lid: grip this far below the measured object top instead, but keep the
# hand at least GRASP_MIN_TABLE_CLEARANCE_M above the table.  The grasp
# approach is checked against GRASP_APPROACH_TABLE_MARGIN_M (just under that
# floor) instead of TABLE_COLLISION_MARGIN_M.
GRASP_BELOW_TOP_M = 0.04
GRASP_MIN_TABLE_CLEARANCE_M = 0.09
GRASP_APPROACH_TABLE_MARGIN_M = 0.08
# The hand first reaches the grasp height this far short of the object along
# the finger direction, then moves straight in, so the fingers never come down
# onto the object.
GRASP_PREGRASP_BACKOFF_M = 0.06
GRASP_PREGRASP_TIMEOUT_S = 60.0
# ReachabilityChecker rejects a wrist within 0.03 m of the 0.18 m torso
# cylinder; waypoints (pre-grasp, hover) keep the wrist at least this far from
# the torso axis, shortening the pre-grasp backoff when needed.
TORSO_KEEP_OUT_RADIUS_M = 0.23
# Other objects on the table are obstacles for the grab: each is a vertical
# cylinder from the table to its measured top; the hand is a chain of spheres
# from the wrist (hand frame) along the fingers.  When the way to the
# pre-grasp crosses an obstacle the hand first hovers this far above the
# tallest one.
HAND_COLLISION_OFFSETS_M = (0.0, 0.045, 0.09, 0.13)
HAND_COLLISION_RADIUS_M = 0.04
OBJECT_OBSTACLE_MARGIN_M = 0.02
GRASP_HOVER_CLEARANCE_M = 0.05
# Table objects this close to the target (horizontally) are the target, and
# those whose visible surface lies this close to the forearm/hand (elbow to
# fingertips) are the arm itself.
OBJECT_OBSTACLE_TARGET_GAP_M = 0.04
OBJECT_OBSTACLE_ARM_GAP_M = 0.08
# Objects farther than this from the shoulder (horizontally, to their near
# edge) are out of the hand's reach (0.46 m grasp reach + fingers) and not obstacles.
OBJECT_OBSTACLE_MAX_REACH_M = 0.60
TABLE_OBJECTS_WAIT_S = 2.0
# Sorting-view overlay of the grab plan (BGR); the plan stays visible this
# long after the grab ends.
GRAB_PLAN_SHOW_S = 15.0
EE_COLOR_BGR = (255, 0, 255)
PLAN_COLOR_BGR = (255, 255, 0)
BLOCKED_COLOR_BGR = (60, 60, 255)
OBSTACLE_COLOR_BGR = (0, 140, 255)
TARGET_COLOR_BGR = (80, 255, 80)
PATH_CHECK_STEP_M = 0.01
TABLE_COLLISION_MARGIN_M = 0.10
TABLE_COLLISION_XY_MARGIN_M = 0.12
DEFAULT_DIRECT_MAX_REACH_M = 0.52
DEFAULT_SLOW_GRAB_RATE_HZ = 3.0
DEFAULT_SLOW_GRAB_TIMEOUT_S = 180.0
DEFAULT_VISION_GRAB_CONVERGENCE_POS_M = 0.015
# Pre-grab reach limit for the planned hand target.  The URDF arm reaches at
# most ~0.432 m from the shoulder, so a target farther than that plus a
# vision convergence tolerance (0.030 m) cannot converge: with the 0.52 m
# executor limit such targets passed the check and the approach IK stalled
# 5-7 cm short until the timeout.
GRAB_TARGET_MAX_REACH_M = 0.46
# Vision grabs keep the palm level: the Dex3 fingers close in the palm x-y
# plane, so palm z must stay vertical or closing sweeps the fingers into the
# table.  IK accepts up to this palm-normal tilt; the approach converges below
# the second value (yaw is free).
GRASP_PALM_LEVEL_TOL_RAD = math.radians(6.0)
GRASP_PALM_LEVEL_CONVERGENCE_RAD = math.radians(4.0)
# The level IK above returns as soon as it is inside its loose tolerance, so
# the approach settled ~3-4 cm low and ~6 deg tilted.  It is first solved to
# these tolerances (falling back to the loose ones), and the remaining
# gravity sag is removed by a position bias integrated while progress stalls
# within APPROACH_SAG_ACTIVE_M of the target.
GRASP_FINE_IK_TOL_POS_M = 0.004
GRASP_FINE_IK_TOL_ROT_RAD = math.radians(1.0)
APPROACH_SAG_GAIN = 0.5
APPROACH_SAG_MAX_M = 0.04
APPROACH_SAG_ACTIVE_M = 0.06
# Closed-loop Cartesian moves on the hold executor (lift, placement): re-aim
# at the measured error for up to this many rounds until within the tolerance.
HOLD_MOVE_ROUNDS = 4
HOLD_MOVE_TOL_M = 0.01
HOLD_MOVE_ACCEPT_M = 0.025
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

# Dex3 tactile raw values: 30000 is the "no reading" sentinel.  The Unitree
# docs call >=100000 a contact, but these pads idle at ~101000, so contact
# and the grip target start at 103000.
TACTILE_INVALID_VALUE = 30000.0
TACTILE_VALID_THRESHOLD = 103000.0
DEFAULT_TACTILE_BASE_TARGET_RAW = 103000.0
DEFAULT_TACTILE_PER_KG_RAW = 60000.0
DEFAULT_TACTILE_MAX_TARGET_RAW = 450000.0
DEFAULT_TACTILE_CLOSE_STEPS = 10
DEFAULT_TACTILE_STEP_HOLD_S = 0.12
DEFAULT_TACTILE_STEP_SETTLE_S = 0.06
# kp 1.0-1.4 cannot track the close steps even in free air (fingers lag the
# command by more than the stall threshold), so the close runs at the
# recovery-move stiffness.
DEFAULT_TACTILE_CLOSE_KP = 1.8
DEFAULT_TACTILE_CLOSE_KD = 0.08
DEFAULT_TACTILE_CLOSE_TAU = 0.06
# A suspected finger stall is re-checked while the step target keeps being
# commanded; tracking lag clears within this window, a blocked finger does not.
DEX3_GRIP_STALL_CONFIRM_S = 0.25
DEX3_GRIP_STREAM_DT_S = 0.02
# A confirmed stall with a stalled finger measured at least this far
# (open=0, closed=1) means the fingers met the object (the pads often miss
# side contact): hold there with a small squeeze instead of releasing.  The
# squeeze stays below DEX3_STALL_ERROR_RAD so no stall detector fires on the
# hold.  Index/middle joints that had not stalled keep closing to the closed
# pose so they wrap the object too.  Stalls right at the open pose are
# treated as jams.
DEX3_GRIP_CONTACT_MIN_CLOSURE = 0.10
DEX3_GRIP_SQUEEZE_RAD = 0.15
DEX3_GRIP_SQUEEZE_HOLD_S = 0.4
# After lifting a stall-contact grasp, a stalled finger still at least this
# far short of its squeeze target is blocked by the object; one that reached
# the target closed on nothing (the object slipped or the stall was the table).
DEX3_GRIP_HELD_MIN_ERR_RAD = 0.035

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

# Depth-based object candidates for the OpenAI detector: connected regions
# standing this high above the fitted table plane, inside its footprint.
OBJECT_PROPOSAL_MIN_HEIGHT_M = 0.02
OBJECT_PROPOSAL_MAX_HEIGHT_M = 0.40
OBJECT_PROPOSAL_FOOTPRINT_MARGIN_M = 0.05
OBJECT_PROPOSAL_MIN_AREA_FRAC = 0.0005   # of the image (150 px at 640x480)
OBJECT_PROPOSAL_PAD_PX = 4

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
    # Height of the object's top above its centroid (base frame), from depth.
    top_offset_m: Optional[float] = None


@dataclass
class TableObject:
    """Something standing on the table, from depth alone (base frame)."""
    box: Tuple[int, int, int, int]
    center_xy: np.ndarray
    radius_m: float
    top_z_m: float
    table_z_m: float
    # Mean of the visible surface points; on the arm itself when the
    # "object" is the robot's own hand/forearm over the desk.
    visible_centroid: Optional[np.ndarray] = None


def _draw_overlay_label(image: np.ndarray, text: str, origin: Tuple[int, int], color: Tuple[int, int, int]) -> None:
    x, y = int(origin[0]), int(origin[1])
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def hand_collision_points(T_base_hand: np.ndarray) -> List[np.ndarray]:
    """Sphere centres from the wrist along the fingers (hand x)."""
    return [T_base_hand[:3, 3] + T_base_hand[:3, :3] @ np.array([offset, 0.0, 0.0])
            for offset in HAND_COLLISION_OFFSETS_M]


def object_collisions(T_base_hand: np.ndarray, objects: Sequence[TableObject]) -> List[str]:
    reasons = []
    for point in hand_collision_points(T_base_hand):
        for i, obj in enumerate(objects):
            reach = obj.radius_m + HAND_COLLISION_RADIUS_M + OBJECT_OBSTACLE_MARGIN_M
            if (float(np.linalg.norm(point[:2] - obj.center_xy)) < reach
                    and point[2] - HAND_COLLISION_RADIUS_M < obj.top_z_m + OBJECT_OBSTACLE_MARGIN_M):
                reasons.append(f"hand would hit obstacle {i + 1} at ({obj.center_xy[0]:+.2f}, {obj.center_xy[1]:+.2f}) "
                               f"top z={obj.top_z_m:.3f}m")
    return reasons


def path_collisions(T_from: np.ndarray, T_to: np.ndarray, objects: Sequence[TableObject]) -> List[str]:
    """Straight-line hand path (positions interpolated, T_to's orientation)."""
    if not objects:
        return []
    p0, p1 = T_from[:3, 3], T_to[:3, 3]
    steps = max(1, int(math.ceil(float(np.linalg.norm(p1 - p0)) / PATH_CHECK_STEP_M)))
    T = T_to.copy()
    for i in range(steps + 1):
        T[:3, 3] = p0 + (p1 - p0) * (i / steps)
        reasons = object_collisions(T, objects)
        if reasons:
            return reasons
    return []


def object_top_offset(mask: np.ndarray, depth_m: np.ndarray, K: CameraIntrinsics, R_base_camera: np.ndarray,
                      centroid_camera: np.ndarray) -> Optional[float]:
    """Height of the masked object's top (95th percentile) above its
    centroid, in the base frame's vertical."""
    ys, xs = np.nonzero(mask)
    d = depth_m[ys, xs]
    keep = d > 0.05
    if int(np.count_nonzero(keep)) < 30:
        return None
    xs, ys, d = xs[keep], ys[keep], d[keep].astype(np.float64)
    pts = np.stack([(xs - K.cx) * d / K.fx, (ys - K.cy) * d / K.fy, d], axis=1)
    z = pts @ R_base_camera[2]
    return float(np.percentile(z, 95) - float(R_base_camera[2] @ np.asarray(centroid_camera, dtype=np.float64)))


def copy_detection(det: Detection) -> Detection:
    return Detection(id=det.id, label=det.label, source=det.source, score=float(det.score),
                     T_camera_object=det.T_camera_object.copy(), box=None if det.box is None else tuple(det.box),
                     grasp_surface=det.grasp_surface, top_offset_m=det.top_offset_m)


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


def stable_vision_object_pose(T_base_object: np.ndarray, table: Optional[TablePlane] = None,
                              top_offset_m: Optional[float] = None) -> np.ndarray:
    """Synthetic object frame: measured position, robot-side approach, palm
    kept above the table (mask-PCA orientation is too noisy to use)."""
    T = np.eye(4, dtype=np.float64)
    pos = T_base_object[:3, 3].copy()
    min_z = pos[2] + DEFAULT_VISION_GRASP_LIFT_M
    if table is not None:
        min_z = max(min_z, float(table.z_base_m) + DEFAULT_VISION_TABLE_CLEARANCE_M)
        if top_offset_m is not None:
            below_top = pos[2] + float(top_offset_m) - GRASP_BELOW_TOP_M
            min_z = max(float(table.z_base_m) + GRASP_MIN_TABLE_CLEARANCE_M, min(min_z, below_top))
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
    T_object = (stable_vision_object_pose(T_base_object, table=obstacles.table if obstacles is not None else None,
                                          top_offset_m=det.top_offset_m)
                if det.source == "vision" else T_base_object)
    if fixed_R_base is None and det.source == "vision":
        fixed_R_base = level_grasp_rotation(arm, T_base_object[:3, 3])
    T_base_hand = GraspPlanner(arm=arm, standoff_m=grasp_standoff_for_surface(det.grasp_surface)).compute(T_object)
    if fixed_R_base is not None:
        T_base_hand = T_base_hand.copy()
        T_base_hand[:3, :3] = np.asarray(fixed_R_base, dtype=np.float64)
    return T_base_hand


def level_rotation(R: np.ndarray) -> np.ndarray:
    """R's yaw with the palm normal (z) exactly vertical, as for the grasp."""
    x = np.array([R[0, 0], R[1, 0], 0.0])
    norm = float(np.linalg.norm(x))
    x = x / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    z = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def palm_tilt_correction(z_target: np.ndarray, z_measured: np.ndarray, max_rad: float = math.radians(12.0)) -> np.ndarray:
    """Rotation turning the measured palm normal onto the target one (angle
    capped); pre-multiplied onto an aim pose it cancels the sag tilt."""
    axis = np.cross(z_measured, z_target)
    s, c = float(np.linalg.norm(axis)), float(np.dot(z_measured, z_target))
    angle = min(max_rad, math.atan2(s, c))
    if s < 1e-9 or angle < 1e-6:
        return np.eye(3)
    k = axis / s
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def _status_pos_err(status: Dict[str, Any]) -> float:
    try:
        return float(status.get("last_error_pos_m"))
    except (TypeError, ValueError):
        return float("inf")


def pregrasp_hand_pose(T_base_hand: np.ndarray) -> np.ndarray:
    """The grasp pose pulled back along the finger direction (hand x), by
    less than GRASP_PREGRASP_BACKOFF_M when that would bring the wrist
    within TORSO_KEEP_OUT_RADIUS_M of the torso axis."""
    T = np.asarray(T_base_hand, dtype=np.float64).copy()
    forward = T[:3, 0].copy()
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm <= 1e-6:
        return T
    forward /= norm
    grasp_xy = T[:2, 3].copy()
    backoff = GRASP_PREGRASP_BACKOFF_M
    while backoff > 0.0 and float(np.linalg.norm(grasp_xy - forward[:2] * backoff)) < TORSO_KEEP_OUT_RADIUS_M:
        backoff -= 0.005
    T[:3, 3] -= forward * max(0.0, backoff)
    return T


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
                 max_joint_speed_rad_s: float = 0.15, read_arm_q: Optional[Callable[[str], Optional[np.ndarray]]] = None,
                 on_takeover: Optional[Callable[[], None]] = None) -> None:
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
        # Called once a validated execute() is about to publish, so another
        # publisher (an idle pose hold) stops first: two rt/arm_sdk writers
        # alternating targets make the joints chatter.
        self._on_takeover = on_takeover
        # URDF-exact FK for the swept-path obstacle check.
        self._fk = ArmFK(arm=arm, backend="urdf")
        self._checker = ReachabilityChecker(arm=arm, max_reach_m=max_reach_m)
        self._last_command_q: Optional[np.ndarray] = None
        self._continuous_thread: Optional[threading.Thread] = None
        self._continuous_stop: Optional[threading.Event] = None
        self._target_queue: Optional["queue.Queue[np.ndarray]"] = None

    def validate(self, q_arm_desired: np.ndarray, *, T_base_desired: Optional[np.ndarray] = None,
                 q_arm_start: Optional[np.ndarray] = None, obstacles: Optional[Obstacles] = None,
                 objects: Sequence[TableObject] = ()) -> Dict:
        if not self.safety_gate:
            return {"safe": True}
        result = self._checker.check(q_arm_desired, T_base_desired)
        if not result.safe:
            return {"safe": False, "reason": "safety_gate", "violations": result.reasons}
        if obstacles is not None and q_arm_start is not None:
            sweep = check_swept_path(self._fk, q_arm_start, q_arm_desired, obstacles)
            if not sweep.safe:
                return {"safe": False, "reason": "obstacle_gate", "violations": sweep.reasons}
        if objects and q_arm_start is not None:
            q_start, q_goal = np.asarray(q_arm_start, dtype=np.float64), np.asarray(q_arm_desired, dtype=np.float64)
            for i in range(9):
                alpha = i / 8.0
                reasons = object_collisions(self._fk.compute_arm((1.0 - alpha) * q_start + alpha * q_goal), objects)
                if reasons:
                    return {"safe": False, "reason": "object_gate", "violations": [f"[alpha={alpha:.2f}] {reasons[0]}"]}
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
        if self._on_takeover is not None:
            self._on_takeover()
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
                 obstacles: Optional[Obstacles], objects: Sequence[TableObject] = ()) -> None:
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
        self.ik_fine = (ArmIK(arm=arm, solver="level_dls", tol_pos_m=GRASP_FINE_IK_TOL_POS_M,
                              tol_rot_rad=GRASP_FINE_IK_TOL_ROT_RAD) if config["ik_solver"] == "level_dls" else None)
        self._sag_bias = np.zeros(3)
        self._sag_tilt = np.eye(3)
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
        self.objects: Tuple[TableObject, ...] = tuple(objects)
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
            T_ik = T_base_desired
            if self.ik_fine is not None:
                settled = len(err_history) >= 2 and abs(err_history[-2] - err_history[-1]) < 0.003
                if err_pos < APPROACH_SAG_ACTIVE_M and settled:
                    bias = self._sag_bias + APPROACH_SAG_GAIN * (T_base_desired[:3, 3] - T_base_hand[:3, 3])
                    norm = float(np.linalg.norm(bias))
                    self._sag_bias = bias * (APPROACH_SAG_MAX_M / norm) if norm > APPROACH_SAG_MAX_M else bias
                    z_half = T_base_hand[:3, 2] + T_base_desired[:3, 2]
                    z_half /= max(1e-9, float(np.linalg.norm(z_half)))
                    self._sag_tilt = palm_tilt_correction(T_base_desired[:3, 2], z_half) @ self._sag_tilt
                T_ik = T_base_desired.copy()
                T_ik[:3, :3] = self._sag_tilt @ T_base_desired[:3, :3]
                T_ik[:3, 3] += self._sag_bias
                q_arm_desired, ik_info = self.ik_fine.solve(T_ik, q_init=q_arm_cur)
                if q_arm_desired is None:
                    q_arm_desired, ik_info = self.ik.solve(T_ik, q_init=q_arm_cur)
            else:
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
            validation = self.executor.validate(q_next, T_base_desired=T_base_desired, q_arm_start=q_start,
                                                obstacles=obstacles, objects=self.objects)
            if not validation.get("safe"):
                self.status["safety_rejections"] += 1
                label = {"obstacle_gate": "[Step 8b] Obstacle in swept path",
                         "object_gate": "[Step 8c] Object in swept path"}.get(validation.get("reason"),
                                                                              "[Step 9] Command rejected")
                self._record(f"{label}: {validation.get('violations') or validation.get('reason')}")
                self._sleep(dt, t0)
                continue
            self.executor.submit_target(q_next)
            q_arm_commanded = q_next.copy()
            self._record(f"[Step 10] it={self.status['iteration']:4d}  pos_err={err_pos:.4f}m  "
                         f"rot_err={err_rot:.3f}rad  ik_iter={ik_info['iterations']}"
                         + (f"  sag_bias={np.linalg.norm(self._sag_bias) * 100:.1f}cm" if self.ik_fine is not None else ""))
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
                 camera_rate_hz: float = 15.0, vision_backend: str = "yolo") -> None:
        self.robot = robot
        self.endpoint = str(rgbd_endpoint)
        self.vision_model = str(vision_model)
        self.vision_backend = str(vision_backend)  # "yolo" (local YOLO-World) or "openai"
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
        # Depth-only objects on the table (obstacles) and the current grab
        # plan, both drawn on the sorting view.
        self.table_objects: List[TableObject] = []
        self.table_objects_ts = 0.0
        self.grab_plan: Dict[str, Any] = {}
        self.last_grab_blocked = ""
        self.held_object: Dict[str, Any] = {}
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
            if self.vision_backend == "openai":
                from openai_vision import OpenAIVisionDetector
                vision = OpenAIVisionDetector(model_name=self.vision_model)
            else:
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
        self.vision_status = (f"on ({self.vision_backend}: {self.vision_model})" if vision.available
                              else f"off ({vision.error})")

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
            vis_dets = []
            with self.lock:
                cam = dict(self.camera_extrinsic)
            R_base_camera = base_from_camera(cam)[:3, :3]
            table_objects = self.table_objects_from_depth(depth, cam)
            with self.lock:
                self.table_objects = table_objects or []
                self.table_objects_ts = time.time()
            if vision is not None and vision.available:
                if getattr(vision, "uses_proposals", False):
                    vis_dets = vision.detect(rgb, proposals=None if table_objects is None
                                             else [obj.box for obj in table_objects])
                else:
                    vis_dets = vision.detect(rgb)
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
                                               grasp_surface=estimate_grasp_surface(mask, depth, K),
                                               top_offset_m=object_top_offset(mask, depth, K, R_base_camera,
                                                                              pose.T_camera_object[:3, 3]))
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
                state = self.vision_status
                if vision is not None and vision.available and getattr(vision, "error", ""):
                    state += f" - last error: {vision.error}"
                self.status_msg = f"{len(detections)} detections  |  vision model: {state}  |  frame #{seq}"
            time.sleep(max(0.0, period - (time.time() - t0)))

    def table_objects_from_depth(self, depth_m: np.ndarray, cam: Dict[str, float]) -> Optional[List[TableObject]]:
        """Everything standing on the table, from depth alone: pixel box plus
        a vertical base-frame cylinder.  None when no table plane is found."""
        T = base_from_camera(cam)
        K = self.camera_matrix()
        try:
            table = estimate_table_plane(depth_m, K, T)
        except Exception:
            table = None
        if table is None:
            return None
        h, w = depth_m.shape[:2]
        ys, xs = np.indices((h, w), dtype=np.float32)
        x_cam = (xs - K[0, 2]) * depth_m / K[0, 0]
        y_cam = (ys - K[1, 2]) * depth_m / K[1, 1]
        x_base = T[0, 0] * x_cam + T[0, 1] * y_cam + T[0, 2] * depth_m + T[0, 3]
        y_base = T[1, 0] * x_cam + T[1, 1] * y_cam + T[1, 2] * depth_m + T[1, 3]
        height = T[2, 0] * x_cam + T[2, 1] * y_cam + T[2, 2] * depth_m + T[2, 3] - table.z_base_m
        margin = OBJECT_PROPOSAL_FOOTPRINT_MARGIN_M
        above = (np.isfinite(depth_m) & (depth_m > 0.15)
                 & (height >= OBJECT_PROPOSAL_MIN_HEIGHT_M) & (height <= OBJECT_PROPOSAL_MAX_HEIGHT_M)
                 & (x_base >= table.x_range_m[0] - margin) & (x_base <= table.x_range_m[1] + margin)
                 & (y_base >= table.y_range_m[0] - margin) & (y_base <= table.y_range_m[1] + margin))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        above = cv2.morphologyEx(above.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(above, connectivity=8)
        min_area = OBJECT_PROPOSAL_MIN_AREA_FRAC * h * w
        pad = OBJECT_PROPOSAL_PAD_PX
        objects = []
        for index in range(1, count):
            x, y, bw, bh, area = (int(v) for v in stats[index])
            if area < min_area:
                continue
            sel = labels[y:y + bh, x:x + bw] == index
            px, py = x_base[y:y + bh, x:x + bw][sel], y_base[y:y + bh, x:x + bw][sel]
            # The camera only sees the near half: the radius is half the
            # extent across the line of sight, and the centre sits that far
            # behind the nearest visible point along it.
            pts = np.stack([px, py], axis=1).astype(np.float64)
            sight = np.median(pts, axis=0) - T[:2, 3]
            sight /= max(1e-9, float(np.linalg.norm(sight)))
            across = np.array([-sight[1], sight[0]])
            along_p, across_p = pts @ sight, pts @ across
            a_lo, a_hi = np.percentile(across_p, 2), np.percentile(across_p, 98)
            radius = max(0.01, float(a_hi - a_lo) / 2.0)
            centre = sight * (float(np.percentile(along_p, 2)) + radius) + across * float(a_lo + a_hi) / 2.0
            objects.append(TableObject(
                box=(max(0, x - pad), max(0, y - pad), min(w - 1, x + bw + pad), min(h - 1, y + bh + pad)),
                center_xy=centre, radius_m=radius,
                top_z_m=float(table.z_base_m + np.percentile(height[y:y + bh, x:x + bw][sel], 98)),
                table_z_m=float(table.z_base_m),
                visible_centroid=np.array([float(np.mean(px)), float(np.mean(py)),
                                           float(table.z_base_m + np.mean(height[y:y + bh, x:x + bw][sel]))])))
        return objects

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
        """Blocking-move executor; it stops the idle pose hold when it starts publishing."""
        return ArmExecutor(self.robot, arm=arm, kp=kp, kd=kd, max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
                           max_joint_step_rad=step, max_joint_speed_rad_s=speed, read_arm_q=self.read_current_arm_q,
                           on_takeover=self.stop_active_hold)

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

    def _vertical_lift_obstacles(self, obstacles: Optional[Obstacles], T_start: np.ndarray) -> Optional[Obstacles]:
        """A grasp or a low stable pose may start inside the conservative
        table margin; for a vertical lift only, keep >=1.5 cm below the start
        clearance and still check every later sample."""
        if obstacles is None or obstacles.table is None:
            return obstacles
        start_clearance_m = float(T_start[2, 3] - obstacles.table.z_base_m)
        margin = min(float(obstacles.table_margin_m), max(0.03, start_clearance_m - 0.015))
        if margin >= float(obstacles.table_margin_m):
            return obstacles
        self.log(f"[grasp] vertical lift starts inside the table buffer; using {margin:.3f}m lift-path "
                 f"margin from {start_clearance_m:.3f}m measured clearance.")
        return replace(obstacles, table_margin_m=margin)

    def move_hand_on_hold(self, arm: str, executor: ArmExecutor, T_target: np.ndarray,
                          obstacles: Optional[Obstacles], label: str) -> Dict[str, Any]:
        """Closed-loop Cartesian move on a continuous executor with the palm
        kept level: after each move the measured hand error (gravity sag under
        the soft gains) is added to the aim point and the move repeated."""
        fk = ArmFK(arm=arm, backend="urdf")
        ik_fine = ArmIK(arm=arm, solver="level_dls", tol_pos_m=GRASP_FINE_IK_TOL_POS_M,
                        tol_rot_rad=GRASP_FINE_IK_TOL_ROT_RAD)
        ik_loose = ArmIK(arm=arm, solver="level_dls", tol_pos_m=0.012, tol_rot_rad=GRASP_PALM_LEVEL_TOL_RAD)
        target = T_target[:3, 3].copy()
        bias = np.zeros(3)
        R_tilt = np.eye(3)
        err = float("inf")
        for attempt in range(HOLD_MOVE_ROUNDS):
            q_cur = self.read_current_arm_q(arm)
            T_aim = T_target.copy()
            T_aim[:3, :3] = R_tilt @ T_target[:3, :3]
            T_aim[:3, 3] = target + bias
            q_sol, info = ik_fine.solve(T_aim, q_init=q_cur)
            if q_sol is None:
                q_sol, info = ik_loose.solve(T_aim, q_init=q_cur)
            if q_sol is None:
                return {"success": False, "reason": "ik_failed", "label": label,
                        "ik_error_pos_m": info.get("error_pos_m"), "attempt": attempt}
            if attempt == 0:
                validation = executor.validate(q_sol, T_base_desired=T_target, q_arm_start=q_cur, obstacles=obstacles)
                if not validation.get("safe"):
                    return {"success": False, "reason": validation.get("reason"), "label": label,
                            "violations": validation.get("violations")}
            executor.submit_target(q_sol)
            wait_s = float(np.max(np.abs(q_sol - q_cur))) / max(0.05, executor.max_joint_speed_rad_s) * 2.0 + 0.6
            deadline = time.time() + wait_s
            while time.time() < deadline:
                if self.cancel.is_set():
                    return {"success": False, "reason": "stopped", "label": label}
                time.sleep(0.05)
            T_now = fk.compute_arm(self.read_current_arm_q(arm))
            delta = target - T_now[:3, 3]
            err = float(np.linalg.norm(delta))
            tilt = math.acos(float(np.clip(np.dot(T_now[:3, 2], T_target[:3, 2]), -1.0, 1.0)))
            if err < HOLD_MOVE_TOL_M and tilt < GRASP_PALM_LEVEL_CONVERGENCE_RAD / 2.0:
                break
            R_tilt = palm_tilt_correction(T_target[:3, 2], T_now[:3, 2]) @ R_tilt
            bias = bias + delta
            norm = float(np.linalg.norm(bias))
            if norm > 0.05:
                bias *= 0.05 / norm
        return {"success": err < HOLD_MOVE_ACCEPT_M, "reason": "" if err < HOLD_MOVE_ACCEPT_M else "not_reached",
                "label": label, "pos_err_m": err, "tilt_rad": tilt, "sag_bias_m": float(np.linalg.norm(bias))}

    def lift_end_effector_z_with_continuous_executor(self, arm: str, executor: ArmExecutor,
                                                     lift_m: float = DEFAULT_POST_GRASP_LIFT_M, mass_kg: float = 0.0,
                                                     obstacles: Optional[Obstacles] = None) -> Dict[str, Any]:
        """Vertical lift keeping the palm level, closed-loop on the hand's
        measured height (the soft lift gains sag under the object)."""
        if lift_m <= 0.0:
            return {"success": True, "reason": "lift_disabled"}
        kp, kd, _duration_s, speed_scale = lift_tuning_for_mass(mass_kg)
        # The executor reads kp/kd/speed live at publish time.
        executor.kp, executor.kd = kp, kd
        executor.max_joint_speed_rad_s *= speed_scale
        fk = ArmFK(arm=arm, backend="urdf")
        T_start = fk.compute_arm(self.read_current_arm_q(arm))
        T_lift = T_start.copy()
        T_lift[:3, :3] = level_rotation(T_start[:3, :3])
        T_lift[2, 3] += float(lift_m)
        result = self.move_hand_on_hold(arm, executor, T_lift, self._vertical_lift_obstacles(obstacles, T_start),
                                        "vertical lift")
        lifted = float(fk.compute_arm(self.read_current_arm_q(arm))[2, 3] - T_start[2, 3])
        self.log(f"[grasp] vertical lift: {lifted * 100:.1f} of {lift_m * 100:.1f} cm, palm tilt "
                 f"{math.degrees(result.get('tilt_rad', float('nan'))):.1f} deg (pos_err={result.get('pos_err_m', float('nan')):.3f}m, "
                 f"sag correction {result.get('sag_bias_m', 0.0) * 100:.1f} cm).")
        result.update({"controller": "continuous_executor", "lift_m": float(lift_m), "lifted_m": lifted,
                       "mass_kg": float(mass_kg)})
        return result

    def _lift_to_stable_pose_on_hold(self, arm: str, cam: Dict[str, float], speed: float,
                                     mass_kg: float) -> Dict[str, Any]:
        """Rise vertically from a pose near the table, then go to the WBC
        stable pose, all on the running hold executor (no authority gap and no
        second publisher).  Hold arm_lock with hold_current_arm_pose() active."""
        obstacles = self.estimate_obstacles(cam)
        hold = self.hold_executor
        if hold is None:
            raise RuntimeError("could not establish continuous current-pose hold")
        lift = self.lift_end_effector_z_with_continuous_executor(arm, hold, mass_kg=mass_kg, obstacles=obstacles)
        if not lift.get("success"):
            raise RuntimeError(f"vertical clearance lift failed: {lift}")
        if arm != "right":
            raise RuntimeError("WBC stable forward pose is currently defined for the right arm only")
        # The WBC stable pose carries the hand low; moving there from a low
        # grasp would set the object back down on the table.
        fk = ArmFK(arm=arm, backend="urdf")
        z_now = float(fk.compute_arm(self.read_current_arm_q(arm))[2, 3])
        z_stable = float(fk.compute_arm(self.wbc_right_arm_forward_q())[2, 3])
        if z_stable < z_now - 0.01:
            self.log(f"[grasp] WBC stable pose is lower (hand z={z_stable:.3f}m) than the lifted hand ({z_now:.3f}m); "
                     "holding the lifted pose instead.")
            return {"success": True, "controller": "lifted pose hold", "reason": "stable pose lower than lift"}
        return self.move_to_wbc_right_arm_forward_continuous(hold, speed, obstacles)

    def run_release_and_retract(self, arm: str) -> bool:
        """Open the hand at the grasp pose, rise clear of the table and return
        to the WBC stable pose.  A direct retract from the grasp pose starts
        inside the table margin, so the obstacle gate rejects it."""
        try:
            self.send_hand_action("open", arm)
        except Exception as exc:
            self.log(f"[grasp] Opening {arm} hand before retract failed: {exc}")
        self.clear_hold_flags()
        if not self.arm_lock.acquire(timeout=10.0):
            self.log("[grasp] Release-and-retract skipped: arm controller is busy.")
            return False
        try:
            with self.lock:
                cam = dict(self.camera_extrinsic)
                speed = self.max_joint_speed_rad_s
            self._busy("release and retract")
            with self.nav_lock:
                nav = self.active_nav
            self.hold_current_arm_pose(arm)
            self.shutdown_nav(nav, "before release retract")
            result = self._lift_to_stable_pose_on_hold(arm, cam, speed, 0.0)
            self.log(f"[grasp] release retract to WBC stable pose: success={result.get('success')} "
                     f"reason={result.get('reason', '')}")
            return bool(result.get("success"))
        except Exception as exc:
            self.log(f"[grasp] release retract failed (holding the current pose): {exc}")
            return False
        finally:
            self._idle()
            self.arm_lock.release()

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
            result = self._lift_to_stable_pose_on_hold(arm, cam, speed, mass_kg)
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

    def stream_hand_pose(self, side: str, target: List[float], duration_s: float) -> None:
        """Command `target` for `duration_s` without the SDK's stall release."""
        deadline = time.monotonic() + max(0.0, duration_s)
        while True:
            self.robot.write_dex3_pose(side, target, kp=DEFAULT_TACTILE_CLOSE_KP, kd=DEFAULT_TACTILE_CLOSE_KD,
                                       tau=DEFAULT_TACTILE_CLOSE_TAU)
            if time.monotonic() >= deadline:
                return
            time.sleep(DEX3_GRIP_STREAM_DT_S)

    def confirm_hand_stall(self, side: str, target: List[float], start: Optional[List[float]]) -> List[str]:
        """Keep commanding `target` and return the joints still stalled after
        DEX3_GRIP_STALL_CONFIRM_S; a finger that was only lagging catches up."""
        stalled = self.hand_stalled_joints(side, target, start)
        deadline = time.monotonic() + DEX3_GRIP_STALL_CONFIRM_S
        while stalled and time.monotonic() < deadline:
            self.stream_hand_pose(side, target, 0.0)
            time.sleep(DEX3_GRIP_STREAM_DT_S)
            stalled = self.hand_stalled_joints(side, target, start)
        return stalled

    def hold_grip_contact(self, side: str, stalled: List[str], open_t: List[float],
                          closed_t: List[float]) -> Optional[Tuple[float, List[float]]]:
        """Hold the fingers where they met the object, squeezed slightly
        further toward closed; None when no stalled finger has closed far
        enough for the stall to be contact rather than a jam."""
        snap = self.hand_snapshot(side)
        if snap is None:
            return None
        try:
            actual = [float(snap["positions"][n]) for n in HAND_JOINT_NAMES]
        except (KeyError, TypeError, ValueError):
            return None
        closure = max([(cur - o) / (c - o) for name, cur, o, c in zip(HAND_JOINT_NAMES, actual, open_t, closed_t)
                       if name in stalled and abs(c - o) > 1e-3] or [0.0])
        if closure < DEX3_GRIP_CONTACT_MIN_CLOSURE:
            return None
        # Only the index/middle fingers wrap fully: a free thumb joint driven
        # to the closed pose curls into the object or the fingers and jams.
        squeeze = [closed if name not in stalled and not name.startswith("thumb")
                   else cur + max(-DEX3_GRIP_SQUEEZE_RAD, min(DEX3_GRIP_SQUEEZE_RAD, closed - cur))
                   for name, cur, closed in zip(HAND_JOINT_NAMES, actual, closed_t)]
        self.stream_hand_pose(side, squeeze, DEX3_GRIP_SQUEEZE_HOLD_S)
        return closure, squeeze

    def grip_contact_holds(self, side: str) -> Optional[bool]:
        """After a stall-contact grasp has been lifted: True while a stalled
        finger is still blocked short of its squeeze target (the object is in
        the hand), False when they all reached it, None without feedback."""
        with self.lock:
            grip = dict(self.last_grip_result)
        target, stalled = grip.get("hold_target"), set(grip.get("stalled_joints") or [])
        snap = self.hand_snapshot(side)
        if grip.get("method") != "stall_contact" or not target or not stalled or snap is None:
            return None
        try:
            actual = [float(snap["positions"][n]) for n in HAND_JOINT_NAMES]
        except (KeyError, TypeError, ValueError):
            return None
        errors = {name: abs(goal - cur) for name, goal, cur in zip(HAND_JOINT_NAMES, target, actual) if name in stalled}
        self.log("[grasp] grip check after lift: " + ", ".join(f"{n}={e:.3f}rad" for n, e in errors.items())
                 + f" short of the squeeze target (held >= {DEX3_GRIP_HELD_MIN_ERR_RAD:.3f}rad).")
        return any(err >= DEX3_GRIP_HELD_MIN_ERR_RAD for err in errors.values())

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
                                               rate_hz=50.0, kp=DEFAULT_TACTILE_CLOSE_KP, kd=DEFAULT_TACTILE_CLOSE_KD,
                                               tau=DEFAULT_TACTILE_CLOSE_TAU, ramp_s=0.06)
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
            # A finger short of its step target may only be lagging: re-check
            # while still commanding the step before calling it a stall.
            sdk_stall = isinstance(command, dict) and command.get("reason") == "collision_or_stall"
            stalled = self.confirm_hand_stall(side, target_positions, close_start)
            if sdk_stall and not stalled:
                stalled = [str(command.get("joint"))]
            if not stalled:
                continue
            pressure = self.max_tactile_pressure(side)
            if pressure is not None and pressure >= target_pressure:
                self.log(f"[grasp] Grip target reached ({side}) while confirming a stall: fraction={fraction:.2f} "
                         f"pressure={pressure:.0f} raw.")
                result = {"success": True, "method": "tactile", "side": side, "fraction": fraction,
                          "pressure": pressure, "target_pressure": target_pressure, "mass_kg": mass_kg}
                with self.lock:
                    self.last_grip_result = dict(result)
                return result
            # Fingers blocked partway through the close met the object even
            # when the pads read nothing (side contact): hold the contact
            # instead of releasing it.  The grasp stays tactile-unverified.
            hold = self.hold_grip_contact(side, stalled, open_t, closed_t)
            if hold is not None:
                closure = hold[0]
                self.log(f"[grasp] Fingers stopped on the object at fraction={fraction:.2f} closure={closure:.2f} "
                         f"(stalled joints: {', '.join(stalled)}); holding the contact with a "
                         f"{DEX3_GRIP_SQUEEZE_RAD:.2f}rad squeeze and wrapping the other fingers (pressure={'n/a' if pressure is None else f'{pressure:.0f}'} raw, "
                         "no tactile confirmation).")
                result = {"success": True, "method": "stall_contact", "side": side, "stalled_joints": stalled,
                          "fraction": fraction, "closure": closure, "pressure": pressure, "target_pressure": target_pressure,
                          "mass_kg": mass_kg, "hold_target": hold[1]}
                with self.lock:
                    self.last_grip_result = dict(result)
                return result
            # A stall before the fingers could reach the object is a jam:
            # release it instead of pushing harder.
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

    def grab_object_obstacles(self, arm: str, T_base_object: np.ndarray, since_ts: float) -> List[TableObject]:
        """Table objects other than the target and the arm itself, segmented
        from a depth frame taken after `since_ts` (the arm has stopped)."""
        deadline = time.time() + TABLE_OBJECTS_WAIT_S
        while True:
            with self.lock:
                depth, ts, cam = self.depth_m, self.last_frame_ts, dict(self.camera_extrinsic)
            if ts > since_ts or time.time() >= deadline or self.cancel.is_set():
                break
            time.sleep(0.05)
        if ts <= since_ts:
            self.log("[grasp] no fresh depth frame for the obstacle snapshot; using the last one.")
        objects = (self.table_objects_from_depth(depth, cam) if depth is not None else None) or []
        with self.lock:
            self.table_objects, self.table_objects_ts = list(objects), time.time()
        target_xy = T_base_object[:2, 3]
        q_arm = self.read_current_arm_q(arm)
        fk = ArmFK(arm=arm, backend="urdf")
        elbow = fk.compute_arm_partial(q_arm, 4)[:3, 3]
        tip = hand_collision_points(fk.compute_arm(q_arm))[-1]
        shoulder_xy = np.asarray(SHOULDER_IN_BASE[arm][:2], dtype=np.float64)
        kept = []
        for obj in objects:
            if float(np.linalg.norm(obj.center_xy - shoulder_xy)) - obj.radius_m > OBJECT_OBSTACLE_MAX_REACH_M:
                continue
            if float(np.linalg.norm(obj.center_xy - target_xy)) < obj.radius_m + OBJECT_OBSTACLE_TARGET_GAP_M:
                continue
            centre = (obj.visible_centroid if obj.visible_centroid is not None else
                      np.array([obj.center_xy[0], obj.center_xy[1], (obj.top_z_m + obj.table_z_m) / 2.0]))
            seg = tip - elbow
            t = float(np.clip(np.dot(centre - elbow, seg) / max(1e-9, float(np.dot(seg, seg))), 0.0, 1.0))
            if float(np.linalg.norm(centre - (elbow + t * seg))) < OBJECT_OBSTACLE_ARM_GAP_M:
                continue
            kept.append(obj)
        return kept

    def hand_reach_m(self, arm: str, point: np.ndarray) -> float:
        return float(np.linalg.norm(np.asarray(point, dtype=np.float64)[:3] - SHOULDER_IN_BASE[arm]))

    def place_hand_targets(self, arm: str, centre: np.ndarray, normal: np.ndarray, release_height_m: float,
                           approach_extra_m: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """Hand poses (palm level, current yaw) that put the held object over
        `centre`: (approach, release, release height used).  The release is
        raised so the object's bottom clears the spot by >= 2 cm.  With nothing
        held the object is assumed a grasp standoff ahead of the wrist."""
        T_now = ArmFK(arm=arm, backend="urdf").compute_arm(self.read_current_arm_q(arm))
        with self.lock:
            held = dict(self.held_object) if self.holding_object else {}
        if "offset_hand" in held:
            offset = T_now[:3, :3] @ np.asarray(held["offset_hand"], dtype=np.float64)
        else:
            offset = np.append(np.asarray(centre, dtype=np.float64)[:2] - SHOULDER_IN_BASE[arm][:2], 0.0)
            offset *= DEFAULT_GRASP_STANDOFF_M / max(1e-9, float(np.linalg.norm(offset)))
        offset[2] = 0.0
        release_h = max(float(release_height_m), float(held.get("hang_m", 0.0)) + 0.02)
        T_now[:3, :3] = level_rotation(T_now[:3, :3])
        T_release, T_approach = T_now.copy(), T_now.copy()
        T_release[:3, 3] = centre + normal * release_h - offset
        T_approach[:3, 3] = centre + normal * (release_h + approach_extra_m) - offset
        return T_approach, T_release, release_h

    def plan_grasp_waypoints(self, arm: str, T_grasp: np.ndarray,
                             objects: Sequence[TableObject]) -> Tuple[List[Tuple[str, np.ndarray]], str]:
        """[hover ->] pre-grasp -> grasp, or ([], reason) when the object
        cannot be reached without touching another one."""
        T_pre = pregrasp_hand_pose(T_grasp)
        reasons = object_collisions(T_grasp, objects) or path_collisions(T_pre, T_grasp, objects)
        if reasons:
            return [], f"moving in to the grasp: {reasons[0]}"
        waypoints = [("pre-grasp", T_pre), ("approach", T_grasp)]
        T_now = self.hand_T(arm)
        if T_now is None or not objects:
            return waypoints, ""
        T_start = T_pre.copy()
        T_start[:3, 3] = T_now[:3, 3]
        if not path_collisions(T_start, T_pre, objects):
            return waypoints, ""
        T_hover = T_pre.copy()
        T_hover[2, 3] = max(float(T_pre[2, 3]), max(obj.top_z_m for obj in objects)
                            + GRASP_HOVER_CLEARANCE_M + HAND_COLLISION_RADIUS_M)
        reasons = path_collisions(T_start, T_hover, objects) or path_collisions(T_hover, T_pre, objects)
        if reasons:
            return [], f"going around the other objects: {reasons[0]}"
        return [("hover", T_hover)] + waypoints, ""

    def set_grab_plan(self, arm: str, T_base_object: np.ndarray, T_grasp: np.ndarray,
                      waypoints: List[Tuple[str, np.ndarray]], objects: Sequence[TableObject], blocked: str) -> None:
        with self.lock:
            self.grab_plan = {"ts": time.time(), "arm": arm, "target": T_base_object[:3, 3].copy(),
                              "grasp": palm_visual_point(T_grasp), "objects": list(objects), "blocked": blocked,
                              "waypoints": [(label, palm_visual_point(T)) for label, T in waypoints]}

    def _wait_for_approach(self, nav: HandApproach, corrector: Optional["GrabTargetCorrector"], timeout_s: float,
                           label: str) -> Tuple[bool, bool, Dict[str, Any]]:
        """Follow one approach until it converges, stalls, stops or times
        out; returns (converged, stalled, last status).  The approach keeps
        holding its last command on convergence or stall."""
        ok = stalled = False
        last_status: Dict[str, Any] = {}
        last_log_t = 0.0
        deadline = time.time() + float(timeout_s) + 2.0
        while time.time() < deadline:
            if self.cancel.is_set():
                self.log("[grasp] Direct grab cancelled by safety command.")
                break
            status = nav.status_snapshot()
            last_status = status
            if time.time() - last_log_t >= 1.0:
                last_log_t = time.time()
                loop_log = status.get("log") or []
                self.log(f"[grasp] {label}: iter={status.get('iteration')} converged={status.get('converged')} "
                         f"pos_err={status.get('last_error_pos_m')} ik_failures={status.get('ik_failures')} "
                         f"safety_rejections={status.get('safety_rejections')} last={loop_log[-1] if loop_log else ''}")
            if status.get("converged"):
                ok = True
                break
            if status.get("stalled"):
                stalled = True
                self.log(f"[grasp] {label} stalled: target has not gotten closer over the last several "
                         "EE-pose increments - checking whether the hand is close enough.")
                break
            if not status.get("running", True):
                self.log(f"[grasp] {label} stopped: {(status.get('log') or [''])[-1]}")
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
        return ok, stalled, last_status

    def _reset_after_failed_approach(self, arm: str, step: float, speed: float) -> None:
        try:
            reset = self.reset_to_extend_arm_reference(arm, step, speed)
            self.log(f"[grasp] stall reset to saved extend-arm reference: success={reset.get('success')} "
                     f"reason={reset.get('reason', '')}")
        except Exception as exc:
            self.log(f"[grasp] stall reset to saved extend-arm reference failed: {exc}")

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
                self.last_grab_blocked = ""
                self.grab_plan = {}
            grab_t0 = time.time()
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
            # Vision grasps may sit closer to the table than the general
            # collision margin (short objects); the approach is checked
            # against the grasp floor instead.
            approach_obstacles = (replace(obstacles, table_margin_m=GRASP_APPROACH_TABLE_MARGIN_M)
                                  if vision and obstacles.table is not None else obstacles)
            objects: List[TableObject] = []
            waypoints: List[Tuple[str, np.ndarray]] = []
            if vision:
                objects = self.grab_object_obstacles(arm, T_base_object, grab_t0)
                waypoints, blocked = self.plan_grasp_waypoints(arm, T_base_hand, objects)
                self.set_grab_plan(arm, T_base_object, T_base_hand, waypoints, objects, blocked)
                if blocked:
                    self.log(f"[grasp] no collision-free way to the object: {blocked}; not approaching it.")
                    with self.lock:
                        self.last_grab_blocked = blocked
                    return
                self.log(f"[grasp] {len(objects)} other object(s) on the table treated as obstacles; path: "
                         + " -> ".join(label for label, _T in waypoints) + ".")
            # Every waypoint but the last; the approach to the grasp pose
            # follows with dynamic target correction.  Each next approach
            # starts before the previous one stops so rt/arm_sdk is never silent.
            prev_nav: Optional[HandApproach] = None
            for label, T_waypoint in waypoints[:-1]:
                p_way = T_waypoint[:3, 3]
                self.log(f"[grasp] {label}: hand to ({p_way[0]:+.3f}, {p_way[1]:+.3f}, {p_way[2]:+.3f}) m.")
                way_nav = HandApproach(self, T_waypoint, dict(config, timeout_s=GRASP_PREGRASP_TIMEOUT_S),
                                       approach_obstacles, objects)
                with self.nav_lock:
                    self.active_nav = way_nav
                if prev_nav is not None:
                    self.shutdown_nav(prev_nav, f"{label} handover")
                prev_nav = way_nav
                way_ok, way_stalled, way_status = self._wait_for_approach(way_nav, None, GRASP_PREGRASP_TIMEOUT_S, label)
                way_err = _status_pos_err(way_status)
                if not way_ok and not (way_stalled and way_err <= DEFAULT_VISION_STALL_CLOSE_POS_M):
                    self.log(f"[grasp] {label} pose not reached; not approaching the object.")
                    self.shutdown_nav(way_nav, label)
                    if way_stalled and not self.cancel.is_set():
                        self._reset_after_failed_approach(arm, step, speed)
                    return
                self.log(f"[grasp] {label} reached (pos_err={way_err:.3f}m).")
            nav = HandApproach(self, T_base_hand, config, approach_obstacles, objects)
            with self.nav_lock:
                self.active_nav = nav
            if prev_nav is not None:
                self.shutdown_nav(prev_nav, "grasp approach handover")
            corrector = (GrabTargetCorrector(self, det, arm, T_base_camera, T_base_object, obstacles,
                                             candidates=self.correction_candidates) if dynamic else None)
            ok = stalled = False
            last_status: Dict[str, Any] = {}
            try:
                ok, stalled, last_status = self._wait_for_approach(nav, corrector, float(config["timeout_s"]), "approach")
            finally:
                nav_holding = bool(ok or stalled)
                if not nav_holding:
                    self.shutdown_nav(nav, "direct grab")
            if stalled:
                pos_err = _status_pos_err(last_status)
                if vision and pos_err <= DEFAULT_VISION_STALL_CLOSE_POS_M:
                    self.log(f"[grasp] Direct grab stalled at visually close target (pos_err={pos_err:.4f}m); "
                             "attempting tactile-verified close.")
                    ok = True
                else:
                    self.log("[grasp] Direct grab stalled without a close-enough pose; not closing the hand and "
                             "returning to the extend pose.")
            if stalled and not ok:
                self.shutdown_nav(nav, "direct grab before stall reset")
                if not self.cancel.is_set():
                    self._reset_after_failed_approach(arm, step, speed)
                return
            if not ok:
                self.log("[grasp] Direct grab did not converge; not closing hand.")
                self.shutdown_nav(nav, "direct grab")
                return
            self.log("[grasp] Direct grab reached an accepted close pose; holding it while closing the hand (tactile-guided).")
            try:
                grip = self.tactile_guided_hand_close(arm, mass_kg)
                closed = bool(grip.get("success"))
                unverified = grip.get("method") in ("tactile_full_close_no_contact", "stall_contact")
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
            # Where the object sits relative to the wrist, and how far it hangs
            # below it, so placement can put the object (not the wrist) on a spot.
            T_hand_now = ArmFK(arm=arm, backend="urdf").compute_arm(self.read_current_arm_q(arm))
            held = {"offset_hand": T_hand_now[:3, :3].T @ (T_base_object[:3, 3] - T_hand_now[:3, 3]),
                    "hang_m": (float(T_hand_now[2, 3] - obstacles.table.z_base_m)
                               if obstacles.table is not None else DEFAULT_GRASP_STANDOFF_M)}
            # The approach loop's continuous executor keeps holding the grasp
            # pose until run_confirm_lift() takes over.
            with self.lock:
                self.held_object = held
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

    def draw_grasp_overlay(self, image: np.ndarray, cam: Dict[str, float], arm: str = "right") -> None:
        """Draw in place: the FK hand (wrist -> palm point), the obstacles and
        target of the current grab, and its planned palm path."""
        T_camera_base = np.linalg.inv(base_from_camera(cam))

        def uv(point_base: np.ndarray) -> Optional[Tuple[int, int]]:
            return self.project_camera_point((T_camera_base @ np.append(np.asarray(point_base, dtype=np.float64), 1.0))[:3])

        with self.lock:
            plan = dict(self.grab_plan)
            active = self.grab_running or self.arm_motion_running or self.lift_confirmation_pending
        if plan and (active or time.time() - plan["ts"] < GRAB_PLAN_SHOW_S):
            for i, obj in enumerate(plan["objects"]):
                angles = np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False)
                r = obj.radius_m + OBJECT_OBSTACLE_MARGIN_M
                rings = []
                for z in (obj.table_z_m, obj.top_z_m):
                    ring = [uv(np.array([obj.center_xy[0] + r * math.cos(a), obj.center_xy[1] + r * math.sin(a), z]))
                            for a in angles]
                    if all(q is not None for q in ring):
                        rings.append(np.array(ring, dtype=np.int32))
                for ring in rings:
                    cv2.polylines(image, [ring], True, OBSTACLE_COLOR_BGR, 2, cv2.LINE_AA)
                if len(rings) == 2:
                    for side in (np.argmin(rings[1][:, 0]), np.argmax(rings[1][:, 0])):
                        cv2.line(image, tuple(int(v) for v in rings[0][side]), tuple(int(v) for v in rings[1][side]), OBSTACLE_COLOR_BGR, 1, cv2.LINE_AA)
                    _draw_overlay_label(image, f"obstacle {i + 1}", tuple(int(v) for v in rings[1][np.argmin(rings[1][:, 1])]),
                                        OBSTACLE_COLOR_BGR)
            target_uv = uv(plan["target"])
            if target_uv is not None:
                cv2.drawMarker(image, target_uv, TARGET_COLOR_BGR, cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            T_hand = self.hand_T(plan["arm"])
            path = ([("", palm_visual_point(T_hand))] if T_hand is not None else []) + list(plan["waypoints"])
            pts = [(label, uv(point)) for label, point in path]
            pts = [(label, q) for label, q in pts if q is not None]
            color = BLOCKED_COLOR_BGR if plan["blocked"] else PLAN_COLOR_BGR
            for (_a, q0), (_b, q1) in zip(pts, pts[1:]):
                cv2.line(image, q0, q1, color, 2, cv2.LINE_AA)
            for label, q in pts:
                if label:
                    cv2.circle(image, q, 5, color, -1, cv2.LINE_AA)
                    _draw_overlay_label(image, label, (q[0] + 7, q[1] + 4), color)
            if plan["blocked"]:
                grasp_uv = uv(plan["grasp"])
                if grasp_uv is not None:
                    cv2.drawMarker(image, grasp_uv, BLOCKED_COLOR_BGR, cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
                    _draw_overlay_label(image, "blocked", (grasp_uv[0] + 9, grasp_uv[1] + 4), BLOCKED_COLOR_BGR)
        T_hand = self.hand_T(arm)
        if T_hand is None:
            return
        wrist_uv, palm_uv = uv(T_hand[:3, 3]), uv(palm_visual_point(T_hand))
        if wrist_uv is not None and palm_uv is not None:
            cv2.line(image, wrist_uv, palm_uv, EE_COLOR_BGR, 2, cv2.LINE_AA)
            cv2.circle(image, wrist_uv, 4, EE_COLOR_BGR, 1, cv2.LINE_AA)
        if palm_uv is not None:
            cv2.circle(image, palm_uv, 7, EE_COLOR_BGR, -1, cv2.LINE_AA)
            cv2.circle(image, palm_uv, 7, (255, 255, 255), 1, cv2.LINE_AA)
            p = palm_visual_point(T_hand)
            _draw_overlay_label(image, f"EE (FK palm) z={p[2]:.3f}m", (palm_uv[0] + 10, palm_uv[1] - 8), EE_COLOR_BGR)

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
