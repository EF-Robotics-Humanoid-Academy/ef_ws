#!/usr/bin/env python3
"""
Recognition layer — sensors-to-UI web app for the G1 + Dex3 grasp pipeline.

Shows the RGB-D feed, a segmentation overlay, and a labeled-detections overlay
(open-vocabulary vision model, NL-prompted). Pick a detected object from the
list, hit Grab, and direct_nav.DirectHandPoseNav drives the arm to it (no ROS
2) and closes the hand. Release Arms and Damp are also exposed as one-click
safety controls (sdk_client.Robot.release_arms() / .damp()).

The ROS 2/TF backend (hand_pose_nav_node.HandPoseNavNode via grab_ros2.py)
was removed from this app: it was never exercised past the initial scaffold
and its camera extrinsic was disconnected from the one calibrated/drawn here
(see camera_tf_publisher.py's own hardcoded default). Use
run_hand_pose_nav.py --use-ros-tf directly if you need that pipeline.

v3 additions over recognition_app_old.py:
  - Grasp surface estimation (segmentation.estimate_grasp_surface): each
    vision detection's mask/depth point cloud is used to estimate the
    object's in-view footprint and how far its near surface sits in front
    of the point-cloud centroid, and the grasp standoff is sized from that
    instead of one fixed distance for every object.
  - Dex3 tactile sensor feedback (sdk_client.Robot.get_tactile_pressures(),
    same raw-value semantics as modules/scripts/test_dex3_tactile.py):
    hand closing is driven by measured fingertip pressure instead of a
    single fixed close command, and current pressures are shown in the UI.
  - Object mass input in the web UI: the target grip pressure and the
    post-grasp lift stiffness/speed/duration all scale with the entered
    mass, so light and heavy objects are gripped and lifted differently.

Run:
    python3 recognition_app_v3.py [--iface eth0] [--domain-id 0] [--mock]
Then open http://<host>:8060 in a browser on the same network.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# ── path setup ────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
_G1_DIR = os.path.abspath(os.path.join(_DIR, ".."))
for _p in (_G1_DIR, os.path.join(_G1_DIR, "modules")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from hand_pose_navigation.target_detector import (
    CameraIntrinsics, DetectionResult,
)
from hand_pose_navigation.segmentation import (
    mask_from_box_depth, pose_from_mask,
    draw_detection_boxes, draw_segmentation_overlay,
    estimate_grasp_surface, GraspSurfaceEstimate,
)
from hand_pose_navigation.vision_detector import VisionDetector
from hand_pose_navigation.direct_nav import DirectHandPoseNav, _make_transform
from hand_pose_navigation.grasp_planner import GraspPlanner
from hand_pose_navigation.reachability_checker import ReachabilityChecker
from hand_pose_navigation.arm_fk import ArmFK
from hand_pose_navigation.arm_ik import ArmIK
from hand_pose_navigation.arm_executor import ArmExecutor
from hand_pose_navigation.arm_fk import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, JOINT_LIMITS, SHOULDER_IN_BASE
from hand_pose_navigation.table_estimator import estimate_table_plane, TablePlane
from hand_pose_navigation.obstacle_checker import Obstacles
from hand_pose_navigation.vla_planner import DEFAULT_VLA_POLICY

import dash
from dash import dcc, html, Input, Output, State, callback_context, no_update, ALL
import dash_bootstrap_components as dbc

try:
    from sdk_client import Robot
    _ROBOT_AVAILABLE = True
except ImportError:
    Robot = None
    _ROBOT_AVAILABLE = False

try:
    from sdk_hand import hand_closed_targets, hand_open_targets
    _HAND_TARGETS_AVAILABLE = True
except ImportError:
    hand_closed_targets = None
    hand_open_targets = None
    _HAND_TARGETS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    id: str
    label: str
    source: str  # "vision"
    score: float
    T_camera_object: np.ndarray
    box: Optional[Tuple[int, int, int, int]] = None
    grasp_surface: Optional[GraspSurfaceEstimate] = None


# Unitree G1 depth-camera drawing:
# optical ray is 42 deg from vertical -> 48 deg below horizontal.
_UNITREE_CAMERA_X_M = 47.64571478 / 1000.0
_UNITREE_CAMERA_Z_M = 462.68178553 / 1000.0
_UNITREE_CAMERA_DOWN_PITCH_RAD = math.radians(90.0 - 42.0)
DEFAULT_CAMERA_EXTRINSIC = {
    "x": _UNITREE_CAMERA_X_M,
    "y": 0.0,
    "z": _UNITREE_CAMERA_Z_M,
    "roll": -math.pi / 2.0 - _UNITREE_CAMERA_DOWN_PITCH_RAD,
    "pitch": 0.0,
    "yaw": -math.pi / 2.0,
}
DEFAULT_MAX_JOINT_STEP_RAD = 0.08
ABSOLUTE_MAX_JOINT_STEP_RAD = 0.10
DEFAULT_MAX_JOINT_SPEED_RAD_S = 0.30
ABSOLUTE_MAX_JOINT_SPEED_RAD_S = 0.60
DEFAULT_OBJECT_MASS_KG = 0.30
MAX_OBJECT_MASS_KG = 5.0
WAIST_YAW_JOINT_INDEX = 12
BODY_JOINT_INDEX_BY_LABEL = {
    "left_arm.shoulder_pitch": 15,
    "left_arm.shoulder_roll": 16,
    "left_arm.shoulder_yaw": 17,
    "left_arm.elbow": 18,
    "left_arm.wrist_roll": 19,
    "left_arm.wrist_pitch": 20,
    "left_arm.wrist_yaw": 21,
    "right_arm.shoulder_pitch": 22,
    "right_arm.shoulder_roll": 23,
    "right_arm.shoulder_yaw": 24,
    "right_arm.elbow": 25,
    "right_arm.wrist_roll": 26,
    "right_arm.wrist_pitch": 27,
    "right_arm.wrist_yaw": 28,
}


def _copy_detection(det: Detection) -> Detection:
    return Detection(
        id=det.id,
        label=det.label,
        source=det.source,
        score=float(det.score),
        T_camera_object=det.T_camera_object.copy(),
        box=None if det.box is None else tuple(det.box),
        grasp_surface=det.grasp_surface,  # frozen dataclass of floats; safe to share
    )


def _joint_entry_index(label: str, data: Dict[str, Any]) -> int:
    if "index" in data:
        try:
            return int(data.get("index", -1))
        except Exception:
            return -1
    return int(BODY_JOINT_INDEX_BY_LABEL.get(str(label), -1))


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.rgb_bgr: Optional[np.ndarray] = None
        self.depth_m: Optional[np.ndarray] = None
        self.detection_rgb_bgr: Optional[np.ndarray] = None
        self.detection_depth_m: Optional[np.ndarray] = None
        self.detections: Dict[str, Detection] = {}
        self.boxes_for_draw: List[Dict] = []
        self.masks_for_draw: List[Tuple[str, str, np.ndarray]] = []
        self.max_visible_detections = 1
        self.max_joint_step_rad = DEFAULT_MAX_JOINT_STEP_RAD
        self.max_joint_speed_rad_s = DEFAULT_MAX_JOINT_SPEED_RAD_S
        self.selected_id: Optional[str] = None
        self.camera_extrinsic = dict(DEFAULT_CAMERA_EXTRINSIC)
        self.camera_tf_status = "camera TF: Unitree drawing fallback"
        self.arm_override = "right"
        self.auto_step_base = False
        self.planning_mode = "direct"  # "direct" | "vla" (vla is scaffolding-only, see vla_planner.py)
        self.obstacle_status = "obstacles: not checked yet"
        self.vision_classes: List[str] = []
        self.status_msg = "starting…"
        self.grab_log: List[str] = []
        self.grab_running = False
        self.arm_motion_running = False
        self.arm_motion_label = ""
        self.last_frame_ts: float = 0.0
        self.last_detection_ts: float = 0.0
        self.frame_seq: int = 0
        self.hand_fk_base: Dict[str, np.ndarray] = {}
        self.hand_fk_T_base: Dict[str, np.ndarray] = {}
        self.hand_fk_ts: float = 0.0
        self.hand_fk_status: str = "FK hand: starting"
        self.extended_hand_R_base: Dict[str, np.ndarray] = {}
        self.extended_hand_reference_q: Dict[str, np.ndarray] = {}
        self.extended_hand_reference_T_base: Dict[str, np.ndarray] = {}
        self.object_mass_kg: float = DEFAULT_OBJECT_MASS_KG
        self.tactile_status: str = "tactile: starting…"
        self.last_grip_result: Dict[str, Any] = {}


STATE = SharedState()
ARM_CONTROL_LOCK = threading.Lock()
ARM_CANCEL_EVENT = threading.Event()
ACTIVE_NAV_LOCK = threading.Lock()
ACTIVE_DIRECT_NAV = None

# unrelease_arms() always ramps arm_sdk_weight 0 -> 1 over its duration,
# *regardless* of the weight it's already at. Calling it again while
# authority is already engaged (e.g. Grab starting right after Extend arm)
# drops weight to 0 and ramps back up, handing the arm to the low-level
# default controller for a moment -> the "snappy transition" between
# controllers. All arm-motion entry points funnel through
# _ensure_arm_authority() instead of calling unrelease_arms() directly, so
# the ramp only ever runs when authority isn't already known-engaged.
# Only ever touched while ARM_CONTROL_LOCK is held, so no separate lock.
_ARM_AUTHORITY_ENGAGED = False
BASE_K = CameraIntrinsics()  # default 640x480 RealSense-ish intrinsics
K = CameraIntrinsics()
KNOWN_INTRINSICS = {
    (424, 240): CameraIntrinsics(
        fx=302.1,
        fy=302.5,
        cx=214.3,
        cy=121.4,
        width=424,
        height=240,
    ),
    (640, 480): CameraIntrinsics(
        fx=604.2,
        fy=605.1,
        cx=324.7,
        cy=242.9,
        width=640,
        height=480,
    ),
}

# The object list must reflect what the robot can see *right now*, not the
# last thing it happened to see. If the perception loop hasn't produced a
# fresh frame+detection pass within this window, the UI treats the list as
# stale rather than silently keeping possibly-gone objects selectable.
STALE_AFTER_S = 4.0
DEFAULT_GRASP_STANDOFF_M = 0.08
DEFAULT_VISION_GRASP_LIFT_M = 0.08
DEFAULT_VISION_TABLE_CLEARANCE_M = 0.14
DEFAULT_MAX_BASE_STEP_M = 0.30
DEFAULT_REACH_MARGIN_M = 0.04
DEFAULT_DIRECT_MAX_REACH_M = 0.52
DEFAULT_SLOW_GRAB_RATE_HZ = 3.0
DEFAULT_SLOW_GRAB_TIMEOUT_S = 180.0
DEFAULT_VISION_GRAB_CONVERGENCE_POS_M = 0.065
DEFAULT_VISION_CLOSE_ON_STALL_POS_M = 0.085
DEFAULT_HAND_COMMAND_HOLD_S = 1.0
DEFAULT_HAND_RECOVERY_HOLD_S = 1.6
DEFAULT_HAND_TARGET_TOL_RAD = 0.12
DEFAULT_EXTEND_STAGE_DURATION_S = 1.2
DEFAULT_EXTEND_FINAL_Y_DECREASE_M = 0.09
DEFAULT_CENTER_WAIST_MAX_YAW_RAD = 0.25
DEFAULT_CENTER_WAIST_DEADBAND_RAD = 0.025
DEFAULT_CENTER_WAIST_SPEED_RAD_S = 0.35
DEFAULT_CENTER_WAIST_SETTLE_S = 0.4
DEFAULT_POST_GRASP_LIFT_M = 0.08
DEFAULT_POST_GRASP_LIFT_DURATION_S = 2.5
DEFAULT_DETECTION_PROMPT = "coffee cup"
DEFAULT_SURFACE_CLEARANCE_M = 0.03
MAX_SURFACE_STANDOFF_M = 0.16

# Dex3 tactile (rt/dex3/{side}/state press_sensor_state) raw-value semantics
# mirrored from modules/scripts/test_dex3_tactile.py: 30000 is the
# "no reading" sentinel, and the Unitree docs call >=100000 a confirmed
# contact reading. Read via sdk_client.Robot.get_tactile_pressures(), which
# subscribes to the same topic this app already uses for hand joint state,
# rather than opening a second independent DDS subscription to it.
TACTILE_INVALID_VALUE = 30000.0
TACTILE_VALID_THRESHOLD = 100000.0
DEFAULT_TACTILE_BASE_TARGET_RAW = 120000.0
DEFAULT_TACTILE_PER_KG_RAW = 60000.0
DEFAULT_TACTILE_MAX_TARGET_RAW = 450000.0
DEFAULT_TACTILE_CLOSE_STEPS = 10
DEFAULT_TACTILE_STEP_HOLD_S = 0.12
DEFAULT_TACTILE_STEP_SETTLE_S = 0.06

# Post-grasp lift arm stiffness/speed/duration scale with the entered object
# mass: heavier objects get a stiffer hold (higher kp/kd), a slower joint
# speed (less likely to shake the object loose), and more time to get there.
DEFAULT_LIFT_KP = 30.0
DEFAULT_LIFT_KD = 1.5
MAX_LIFT_KP = 48.0
MAX_LIFT_KD = 3.0
LIFT_KP_PER_KG = 5.0
LIFT_KD_PER_KG = 0.25
LIFT_DURATION_PER_KG = 0.35
MAX_LIFT_DURATION_S = 6.0
LIFT_SPEED_SLOWDOWN_PER_KG = 0.12


class _MockRobot:
    """Frame source + no-op safety calls, used with --mock."""

    def get_rgbd(self, timeout: float = 2.0) -> Dict[str, Any]:
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(rgb, "MOCK CAMERA", (160, 240), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2, cv2.LINE_AA)
        depth = np.full((480, 640), 0.8, dtype=np.float32)
        return {"rgb_bgr": rgb, "depth_m": depth}

    def release_arms(self, **kw) -> Dict:
        time.sleep(0.5)
        return {"mock": True}

    def unrelease_arms(self, **kw) -> Dict:
        time.sleep(0.2)
        return {"mock": True}

    def damp(self) -> None:
        pass


class _ZmqRgbdRobot:
    """Persistent RGB-D ZMQ frame source used by the recognition UI."""

    def __init__(self, host: str, port: int, topic: str = "") -> None:
        import zmq  # type: ignore

        self.host = str(host)
        self.port = int(port)
        self.topic = str(topic)
        self._zmq = zmq
        self._ctx = zmq.Context()
        self._socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, self.topic.encode("utf-8"))
        self._socket.setsockopt(zmq.RCVTIMEO, 100)
        self._socket.setsockopt(zmq.RCVHWM, 2)
        self._socket.connect(f"tcp://{self.host}:{self.port}")

    def get_rgbd(self, timeout: float = 2.0) -> Dict[str, Any]:
        import struct

        deadline = time.time() + max(0.2, float(timeout))
        last_error = ""
        while time.time() < deadline:
            try:
                parts = self._socket.recv_multipart()
                # Detection can be slower than the RGB-D publisher. Drain any
                # queued multipart frames so the UI works from the newest one.
                while True:
                    try:
                        parts = self._socket.recv_multipart(flags=self._zmq.NOBLOCK)
                    except self._zmq.Again:
                        break
            except self._zmq.Again:
                continue
            except Exception as exc:
                last_error = str(exc)
                continue
            if len(parts) >= 4:
                parts = parts[-3:]
            if len(parts) < 2:
                last_error = f"expected RGBD multipart frame, got {len(parts)} part(s)"
                continue

            rgb_jpeg = bytes(parts[0])
            depth_png = bytes(parts[1])
            depth_scale = 0.001
            if len(parts) >= 3 and len(parts[2]) >= 4:
                try:
                    depth_scale = float(struct.unpack("f", parts[2][:4])[0])
                except Exception:
                    depth_scale = 0.001

            rgb = cv2.imdecode(np.frombuffer(rgb_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            depth_raw = cv2.imdecode(np.frombuffer(depth_png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if rgb is None:
                last_error = "failed to decode RGB JPEG"
                continue
            if depth_raw is None:
                last_error = "failed to decode depth PNG"
                continue
            if depth_raw.ndim == 3:
                depth_raw = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)
            depth_m = depth_raw.astype("float32") * float(depth_scale)
            return {
                "source": f"zmq://{self.host}:{self.port}",
                "rgb_jpeg": rgb_jpeg,
                "depth_png": depth_png,
                "rgb_bgr": rgb,
                "rgb_rgb": cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB),
                "depth_raw": depth_raw,
                "depth_m": depth_m,
                "depth_scale_m_per_unit": float(depth_scale),
                "valid_depth_fraction": float((depth_raw > 0).mean()) if depth_raw.size else 0.0,
            }

        detail = f" Last error: {last_error}" if last_error else ""
        raise RuntimeError(
            f"No RGBD frames received from tcp://{self.host}:{self.port} "
            f"within {timeout:.1f}s.{detail}"
        )

    def release_arms(self, **kw) -> Dict:
        return {"rgbd_only": True, "message": "release_arms unavailable in RGBD-only UI mode"}

    def unrelease_arms(self, **kw) -> Dict:
        return {"rgbd_only": True, "message": "unrelease_arms unavailable in RGBD-only UI mode"}

    def damp(self) -> None:
        return None


# ---------------------------------------------------------------------------
# Background perception loop
# ---------------------------------------------------------------------------

def _update_intrinsics_for_frame(rgb: np.ndarray) -> None:
    global K
    h, w = rgb.shape[:2]
    if w == K.width and h == K.height:
        return
    if (w, h) in KNOWN_INTRINSICS:
        K = KNOWN_INTRINSICS[(w, h)]
    else:
        sx = float(w) / float(BASE_K.width)
        sy = float(h) / float(BASE_K.height)
        K = CameraIntrinsics(
            fx=BASE_K.fx * sx,
            fy=BASE_K.fy * sy,
            cx=BASE_K.cx * sx,
            cy=BASE_K.cy * sy,
            width=w,
            height=h,
        )


def _camera_loop(robot, rate_hz: float) -> None:
    period = 1.0 / max(1.0, rate_hz)
    while True:
        t0 = time.time()
        try:
            frame = robot.get_rgbd(timeout=2.0)
            rgb = frame["rgb_bgr"]
            depth = frame["depth_m"]
            _update_intrinsics_for_frame(rgb)
        except Exception as exc:
            with STATE.lock:
                STATE.status_msg = f"camera error: {exc}"
            time.sleep(period)
            continue

        with STATE.lock:
            STATE.rgb_bgr = rgb
            STATE.depth_m = depth
            STATE.last_frame_ts = time.time()
            STATE.frame_seq += 1

        dt = time.time() - t0
        time.sleep(max(0.0, period - dt))


# Perception runs at its own, slower rate_hz while the
# UI polls every 500ms — short bursts of motion blur, one slow vision-model
# call, or one missed camera read used to blank the overlay/list and make the
# selected object feel lost. Hold detections briefly, while still clearing
# them before the stale window when the object is genuinely gone.
DETECTION_HOLD_S = 1.5


def _perception_loop(vision: VisionDetector, rate_hz: float) -> None:
    period = 1.0 / max(0.5, rate_hz)
    last_processed_seq = -1
    held_detections: Dict[str, Tuple[Detection, float]] = {}
    held_boxes: Dict[str, Tuple[Dict, float]] = {}
    held_masks: Dict[str, Tuple[Tuple[str, str, np.ndarray], float]] = {}

    def _merge_and_prune(held: Dict, now: float) -> Dict:
        for k in list(held.keys()):
            if now - held[k][1] > DETECTION_HOLD_S:
                del held[k]
        return {k: v for k, (v, _ts) in held.items()}

    while True:
        t0 = time.time()
        with STATE.lock:
            seq = STATE.frame_seq
            rgb = None if STATE.rgb_bgr is None else STATE.rgb_bgr.copy()
            depth = None if STATE.depth_m is None else STATE.depth_m.copy()
        if rgb is None or depth is None or seq == last_processed_seq:
            time.sleep(min(0.05, period))
            continue
        last_processed_seq = seq

        _update_intrinsics_for_frame(rgb)
        vis_dets = vision.detect(rgb) if vision.available else []

        detections: Dict[str, Detection] = {}
        boxes_for_draw: List[Dict] = []
        masks_for_draw: List[Tuple[str, str, np.ndarray]] = []
        with STATE.lock:
            selected_id = STATE.selected_id
        for i, vd in enumerate(vis_dets):
            mask = mask_from_box_depth(depth, vd.box_xyxy)
            pose = pose_from_mask(mask, depth, K)
            det_id = f"vision:{i}"
            boxes_for_draw.append({
                "id": det_id, "box": vd.box_xyxy, "label": vd.label, "score": vd.score,
                "selected": det_id == selected_id,
            })
            if pose is None:
                continue
            grasp_surface = estimate_grasp_surface(mask, depth, K)
            detections[det_id] = Detection(
                id=det_id, label=vd.label, source="vision", score=vd.score,
                T_camera_object=pose.T_camera_object, box=vd.box_xyxy,
                grasp_surface=grasp_surface,
            )
            masks_for_draw.append((det_id, vd.label, mask))

        now = time.time()
        for det_id, det in detections.items():
            held_detections[det_id] = (det, now)
        for b in boxes_for_draw:
            held_boxes[b["id"]] = (b, now)
        for mask_entry in masks_for_draw:
            held_masks[mask_entry[0]] = (mask_entry, now)

        detections = _merge_and_prune(held_detections, now)
        boxes_for_draw = list(_merge_and_prune(held_boxes, now).values())
        for b in boxes_for_draw:
            b["selected"] = b["id"] == selected_id
        masks_for_draw = list(_merge_and_prune(held_masks, now).values())

        with STATE.lock:
            STATE.detection_rgb_bgr = rgb
            STATE.detection_depth_m = depth
            STATE.detections = detections
            STATE.boxes_for_draw = boxes_for_draw
            STATE.masks_for_draw = masks_for_draw
            STATE.last_detection_ts = time.time()
            n_vision = "on" if vision.available else f"off ({vision.error})"
            STATE.status_msg = (
                f"{len(detections)} detections  |  vision model: {n_vision}  |  frame #{seq}"
            )

        dt = time.time() - t0
        time.sleep(max(0.0, period - dt))


def _hand_fk_loop(iface: str, domain_id: int) -> None:
    try:
        from dds_env import ensure_channel_factory_initialized
        from sdk_sensors import LatestSubscriber, resolve_lowstate_type, lowstate_snapshot_from_msg

        ensure_channel_factory_initialized(int(domain_id), iface)
        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("Could not resolve Unitree LowState_ message type.")
        sub = LatestSubscriber("rt/lowstate", lowstate_type)
        sub.start(queue_len=10)
        fk = {"left": ArmFK("left"), "right": ArmFK("right")}
    except Exception as exc:
        with STATE.lock:
            STATE.hand_fk_status = f"FK hand unavailable: {exc}"
        return

    while True:
        msg, ts = sub.get_latest()
        if msg is None:
            with STATE.lock:
                STATE.hand_fk_status = "FK hand: waiting for rt/lowstate"
            time.sleep(0.2)
            continue
        try:
            snap = lowstate_snapshot_from_msg(msg)
            q_full = np.zeros(30, dtype=np.float64)
            n = min(len(snap.joint_positions), q_full.size)
            q_full[:n] = np.asarray(snap.joint_positions[:n], dtype=np.float64)
            hand_T = {
                side: fk[side].compute(q_full).copy()
                for side in ("left", "right")
            }
            hands = {
                side: T_base_hand[:3, 3].copy()
                for side, T_base_hand in hand_T.items()
            }
            with STATE.lock:
                STATE.hand_fk_base = hands
                STATE.hand_fk_T_base = hand_T
                STATE.hand_fk_ts = float(ts or time.time())
                STATE.hand_fk_status = "FK hand: rt/lowstate"
        except Exception as exc:
            with STATE.lock:
                STATE.hand_fk_status = f"FK hand error: {exc}"
        time.sleep(0.1)


def _init_unitree_dds_once(iface: str, domain_id: int) -> str:
    try:
        from dds_env import ensure_channel_factory_initialized
        ensure_channel_factory_initialized(int(domain_id), iface)
        return f"Unitree DDS initialized: iface={iface} domain_id={domain_id}"
    except Exception as exc:
        return f"Unitree DDS init failed: {exc}"


def _make_control_robot(iface: str, domain_id: int):
    if not _ROBOT_AVAILABLE:
        return None, "sdk_client.Robot unavailable"
    try:
        robot = Robot(
            iface=iface,
            domain_id=domain_id,
            auto_start_sensors=True,
        )
        return robot, "control robot ready"
    except Exception as exc:
        return None, f"control robot unavailable: {exc!r}"


def _ensure_arm_authority(robot, duration_s: float) -> None:
    """Ramp arm_sdk authority to 1.0 only if it isn't already engaged.

    Call this instead of robot.unrelease_arms() directly from any
    arm-motion entry point (Extend arm, Grab, ...). Must be called while
    ARM_CONTROL_LOCK is held.
    """
    global _ARM_AUTHORITY_ENGAGED
    if _ARM_AUTHORITY_ENGAGED:
        _log("[recognition_app] Arm authority already engaged; skipping re-ramp.")
        return
    if robot is None or not hasattr(robot, "unrelease_arms"):
        return
    _log("[recognition_app] Re-engaging arm SDK control.")
    result = robot.unrelease_arms(duration_s=duration_s)
    _log(f"[recognition_app] unrelease_arms() done: {result}")
    _ARM_AUTHORITY_ENGAGED = True


# ---------------------------------------------------------------------------
# Image encoding helpers
# ---------------------------------------------------------------------------

def _encode_jpeg_src(bgr: np.ndarray, quality: int = 85) -> str:
    ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return ""
    payload = base64.b64encode(enc.tobytes()).decode("ascii")
    return f"data:image/jpeg;base64,{payload}"


def _placeholder_src(text: str) -> str:
    img = np.full((240, 320, 3), 30, dtype=np.uint8)
    cv2.putText(img, text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (200, 200, 200), 1, cv2.LINE_AA)
    return _encode_jpeg_src(img)


def _colorize_depth(depth_m: np.ndarray, max_m: float = 3.0) -> np.ndarray:
    clipped = np.clip(depth_m, 0.0, max_m)
    scaled = (clipped / max_m * 255.0).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_JET)


def _draw_pose_axis(
    image: np.ndarray,
    T_camera_pose: np.ndarray,
    label: str,
    color: Tuple[int, int, int],
    axis_len_m: float = 0.05,
) -> bool:
    cam_mat = np.array(
        [[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist = np.zeros((4,), dtype=np.float64)
    R = T_camera_pose[:3, :3]
    t = T_camera_pose[:3, 3].reshape(3, 1)
    if float(t[2, 0]) <= 0.05:
        return False
    try:
        rvec, _ = cv2.Rodrigues(R)
        cv2.drawFrameAxes(image, cam_mat, dist, rvec, t, float(axis_len_m))
        uv, _ = cv2.projectPoints(
            np.zeros((1, 3), dtype=np.float64),
            rvec,
            t,
            cam_mat,
            dist,
        )
        u, v = (int(x) for x in uv.reshape(-1)[:2])
        cv2.putText(
            image,
            label,
            (u + 8, v + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
            cv2.LINE_AA,
        )
        return True
    except Exception:
        return False


def _project_camera_point(point_camera: np.ndarray) -> Optional[Tuple[int, int]]:
    x, y, z = (float(v) for v in point_camera[:3])
    if z <= 0.05:
        return None
    u = int(round(K.fx * x / z + K.cx))
    v = int(round(K.fy * y / z + K.cy))
    if u < -200 or u > K.width + 200 or v < -200 or v > K.height + 200:
        return None
    return u, v


def _draw_base_polyline(
    image: np.ndarray,
    T_camera_base: np.ndarray,
    points_base: List[np.ndarray],
    color: Tuple[int, int, int],
    thickness: int = 2,
) -> None:
    projected: List[Tuple[int, int]] = []
    for point in points_base:
        p_h = np.ones(4, dtype=np.float64)
        p_h[:3] = np.asarray(point, dtype=np.float64)[:3]
        uv = _project_camera_point((T_camera_base @ p_h)[:3])
        if uv is not None:
            projected.append(uv)
    for a, b in zip(projected, projected[1:]):
        cv2.line(image, a, b, color, thickness, cv2.LINE_AA)
    for uv in projected:
        cv2.circle(image, uv, 3, color, -1, cv2.LINE_AA)


def _palm_visual_point(T_base_hand: np.ndarray) -> np.ndarray:
    return T_base_hand[:3, 3] + T_base_hand[:3, :3] @ np.array(
        [_PALM_VISUAL_OFFSET_M, 0.0, 0.0],
        dtype=np.float64,
    )


def _stable_vision_object_pose(
    T_base_object: np.ndarray,
    table: Optional[TablePlane] = None,
) -> np.ndarray:
    """Return a synthetic object frame for vision detections.

    Mask-PCA orientation is noisy for cups/bottles and can make the standoff
    offset move downward or sideways as the mask jitters. Use the measured
    object position, but make the approach come from the robot side in base
    coordinates and keep the palm comfortably above the table.
    """
    T = np.eye(4, dtype=np.float64)
    pos = T_base_object[:3, 3].copy()
    min_z = pos[2] + DEFAULT_VISION_GRASP_LIFT_M
    if table is not None:
        min_z = max(min_z, float(table.z_base_m) + DEFAULT_VISION_TABLE_CLEARANCE_M)
    pos[2] = min_z

    z_axis = np.array([-1.0, 0.0, 0.0], dtype=np.float64)
    y_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= max(1e-9, np.linalg.norm(x_axis))
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= max(1e-9, np.linalg.norm(y_axis))

    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T[:3, 3] = pos
    return T


def _direct_grasp_object_pose(
    det: Detection,
    T_base_object: np.ndarray,
    obstacles: Optional[Obstacles] = None,
) -> np.ndarray:
    if det.source == "vision":
        return _stable_vision_object_pose(
            T_base_object,
            table=obstacles.table if obstacles is not None else None,
        )
    return T_base_object


def _grasp_standoff_for_surface(grasp_surface: Optional[GraspSurfaceEstimate]) -> float:
    """Size the wrist standoff from the object's estimated near surface
    instead of using one fixed distance for every object. A standoff based
    on the point-cloud centroid alone would drive the wrist into a bulky
    object's near surface before it "arrives"; low-confidence or missing
    estimates fall back to the original fixed default.
    """
    if grasp_surface is None or grasp_surface.confidence < 0.15:
        return DEFAULT_GRASP_STANDOFF_M
    surface_standoff = grasp_surface.near_surface_offset_m + DEFAULT_SURFACE_CLEARANCE_M
    return float(min(MAX_SURFACE_STANDOFF_M, max(DEFAULT_GRASP_STANDOFF_M, surface_standoff)))


def _grasp_standoff_for_det(det: Detection) -> float:
    return _grasp_standoff_for_surface(det.grasp_surface)


def _direct_grasp_hand_pose(
    det: Detection,
    T_base_object: np.ndarray,
    arm: str,
    obstacles: Optional[Obstacles] = None,
    fixed_R_base: Optional[np.ndarray] = None,
) -> np.ndarray:
    T_base_hand = GraspPlanner(
        arm=arm,
        standoff_m=_grasp_standoff_for_det(det),
    ).compute(_direct_grasp_object_pose(det, T_base_object, obstacles))
    if fixed_R_base is not None:
        T_base_hand = T_base_hand.copy()
        T_base_hand[:3, :3] = np.asarray(fixed_R_base, dtype=np.float64)
    return T_base_hand


def _grasp_offset_transform(arm: str, standoff_m: float = DEFAULT_GRASP_STANDOFF_M) -> np.ndarray:
    return GraspPlanner(
        arm=arm,
        standoff_m=standoff_m,
    ).compute(np.eye(4, dtype=np.float64))


def _direct_nav_target_frame_from_hand_pose(
    T_base_hand_desired: np.ndarray,
    arm: str,
    standoff_m: float = DEFAULT_GRASP_STANDOFF_M,
) -> np.ndarray:
    return T_base_hand_desired @ np.linalg.inv(_grasp_offset_transform(arm, standoff_m))


def _direct_nav_target_frame(
    det: Detection,
    T_base_object: np.ndarray,
    arm: str,
    obstacles: Optional[Obstacles] = None,
    fixed_R_base: Optional[np.ndarray] = None,
) -> np.ndarray:
    return _direct_nav_target_frame_from_hand_pose(
        _direct_grasp_hand_pose(det, T_base_object, arm, obstacles, fixed_R_base),
        arm,
        _grasp_standoff_for_det(det),
    )


# ArmFK's "hand" frame is hand_palm_link, i.e. the fixed wrist_yaw -> palm
# mount flange (see arm_fk.py's _HAND_PALM_OFFSET). For the camera overlay,
# nudge the gizmo a little farther along the palm so it lands near the visible
# palm center instead of the wrist mount. Display only; control/IK still use
# the FK hand frame.
_PALM_VISUAL_OFFSET_M = 0.045


def _draw_detection_gizmos(
    image: np.ndarray,
    selected: Optional[Detection],
    cam: Dict[str, float],
    arm_override: str,
    hand_fk_T_base: Dict[str, np.ndarray],
    extended_hand_R_base: Dict[str, np.ndarray],
) -> np.ndarray:
    out = image.copy()
    T_base_camera = _make_transform(
        xyz=(cam["x"], cam["y"], cam["z"]),
        rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
    )
    T_camera_base = np.linalg.inv(T_base_camera)
    arm = arm_override
    if selected is not None:
        arm, _T_base_camera, _T_base_object = _resolve_arm_and_base_pose(
            selected,
            cam,
            arm_override,
        )
        _draw_pose_axis(out, selected.T_camera_object, "object", (0, 220, 255), axis_len_m=0.055)
    if arm == "auto":
        arm = "right"
    T_base_hand = hand_fk_T_base.get(arm)
    drew_hand = False
    if T_base_hand is not None:
        T_base_palm = T_base_hand.copy()
        T_base_palm[:3, 3] += T_base_hand[:3, :3] @ np.array(
            [_PALM_VISUAL_OFFSET_M, 0.0, 0.0]
        )
        drew_hand = _draw_pose_axis(
            out,
            T_camera_base @ T_base_palm,
            f"{arm} hand",
            (255, 255, 80),
            axis_len_m=0.06,
        )
    if selected is not None:
        try:
            _arm, _T_base_camera, T_base_object = _resolve_arm_and_base_pose(
                selected,
                cam,
                arm_override,
            )
            T_base_plan = _direct_grasp_hand_pose(
                selected,
                T_base_object,
                arm,
                fixed_R_base=extended_hand_R_base.get(arm),
            )
            plan_color = (80, 255, 255)
            if T_base_hand is not None:
                _draw_base_polyline(
                    out,
                    T_camera_base,
                    [_palm_visual_point(T_base_hand), _palm_visual_point(T_base_plan)],
                    plan_color,
                    thickness=2,
                )
            T_base_plan_visual = T_base_plan.copy()
            T_base_plan_visual[:3, 3] = _palm_visual_point(T_base_plan)
            _draw_pose_axis(
                out,
                T_camera_base @ T_base_plan_visual,
                "planned hand",
                plan_color,
                axis_len_m=0.055,
            )
        except Exception as exc:
            cv2.putText(
                out,
                f"plan unavailable: {exc}",
                (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (80, 80, 255),
                2,
                cv2.LINE_AA,
            )
    return out


def _resolve_arm_and_base_pose(
    det: Detection,
    cam: Dict[str, float],
    arm_override: str,
) -> Tuple[str, np.ndarray, np.ndarray]:
    T_base_camera = _make_transform(
        xyz=(cam["x"], cam["y"], cam["z"]),
        rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
    )
    T_base_object = T_base_camera @ det.T_camera_object
    arm = arm_override
    if arm == "auto":
        arm = "left" if T_base_object[1, 3] > 0 else "right"
    return arm, T_base_camera, T_base_object


def _reach_preview(
    det: Detection,
    cam: Dict[str, float],
    arm_override: str,
    obstacles: Optional[Obstacles] = None,
) -> Tuple[str, float, float, float, float]:
    arm, _T_base_camera, T_base_object = _resolve_arm_and_base_pose(det, cam, arm_override)
    checker = ReachabilityChecker(arm=arm, max_reach_m=DEFAULT_DIRECT_MAX_REACH_M)
    T_base_desired = _direct_grasp_hand_pose(det, T_base_object, arm, obstacles)
    shoulder_y = 0.10 if arm == "left" else -0.10
    shoulder = np.array([0.0, shoulder_y, 0.292], dtype=np.float64)
    reach_dist = float(np.linalg.norm(T_base_desired[:3, 3] - shoulder))
    excess_m = max(0.0, reach_dist - checker.max_reach_m)
    suggested_step_m = min(
        max(0.0, excess_m + DEFAULT_REACH_MARGIN_M),
        DEFAULT_MAX_BASE_STEP_M,
    )
    return arm, reach_dist, checker.max_reach_m, excess_m, suggested_step_m


# ---------------------------------------------------------------------------
# Dash app
# ---------------------------------------------------------------------------

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.DARKLY])
app.title = "G1 Recognition Layer"


def _instructions_card() -> dbc.Card:
    return dbc.Card(dbc.CardBody([
        html.H5("Vision setup"),
        html.Small(
            "Use the prompt below to choose the target class. The app estimates "
            "object pose from the vision box, segmentation mask, and depth frame.",
            className="text-muted",
        ),
    ]), className="mb-3")


def _options_card() -> dbc.Card:
    return dbc.Card(dbc.CardBody([
        html.H5("Motion options"),
        dbc.Label("Maximum visible detections", className="mt-2"),
        dbc.Input(
            id="max-visible-detections",
            type="number",
            min=1,
            step=1,
            value=STATE.max_visible_detections,
            className="mb-2",
        ),
        dbc.Label("Max arm joint increment per command (rad)", className="mt-2"),
        dbc.Input(
            id="max-joint-step-rad",
            type="number",
            min=0.005,
            max=ABSOLUTE_MAX_JOINT_STEP_RAD,
            step=0.005,
            value=STATE.max_joint_step_rad,
            className="mb-2",
        ),
        dbc.Label("Max arm joint speed (rad/s)", className="mt-2"),
        dbc.Input(
            id="max-joint-speed-rad-s",
            type="number",
            min=0.02,
            max=ABSOLUTE_MAX_JOINT_SPEED_RAD_S,
            step=0.01,
            value=STATE.max_joint_speed_rad_s,
            className="mb-2",
        ),
        html.Small(
            "Camera projection uses the built-in fixed base_link<-camera_color_optical_frame transform.",
            className="text-muted",
        ),
        html.Div(id="camera-tf-status", className="mt-2", style={"fontSize": "12px"}),
    ]), className="mb-3")


def _image_panel(panel_id: str, title: str) -> dbc.Col:
    return dbc.Col(dbc.Card([
        dbc.CardHeader(title, style={"fontSize": "13px"}),
        html.Img(id=panel_id, style={"width": "100%"}),
    ]), width=6, className="mb-3")


def _view_panel() -> dbc.Card:
    return dbc.Card([
        dbc.CardHeader([
            dbc.Row([
                dbc.Col(html.Div("Live view", style={"fontSize": "13px"}), width=3),
                dbc.Col(dcc.Dropdown(
                    id="view-select",
                    options=[
                        {"label": "RGB raw", "value": "rgb"},
                        {"label": "Depth", "value": "depth"},
                        {"label": "Vision detections", "value": "detections"},
                        {"label": "Segmentation", "value": "segmentation"},
                    ],
                    value="detections",
                    clearable=False,
                    style={"color": "#111"},
                ), width=9),
            ], align="center"),
        ]),
        dbc.CardBody(html.Img(id="panel-view", style={"width": "100%"})),
    ], className="mb-3")


def _logs_card() -> dbc.Card:
    return dbc.Card([
        dbc.CardHeader("Grab log", style={"fontSize": "13px"}),
        dbc.CardBody(html.Pre(id="grab-log", style={
            "maxHeight": "460px", "overflow": "auto", "fontSize": "11px",
            "background": "#111", "padding": "8px", "marginBottom": 0,
        })),
    ], className="mb-3")


def _limit_visible_detections(
    detections: Dict[str, Detection],
    boxes: List[Dict],
    masks: List[Tuple[str, str, np.ndarray]],
    max_count: int,
) -> Tuple[Dict[str, Detection], List[Dict], List[Tuple[str, str, np.ndarray]]]:
    try:
        limit = max(1, int(max_count))
    except Exception:
        limit = 5
    if len(detections) <= limit:
        return detections, boxes, masks

    ranked = sorted(
        detections.items(),
        key=lambda item: (-float(item[1].score), item[0]),
    )
    allowed_ids = {det_id for det_id, _det in ranked[:limit]}
    limited_detections = {
        det_id: det
        for det_id, det in detections.items()
        if det_id in allowed_ids
    }
    limited_boxes = [
        box for box in boxes
        if str(box.get("id", "")) in allowed_ids
    ]
    limited_masks = [
        mask_item for mask_item in masks
        if mask_item[0] in allowed_ids
    ]
    return limited_detections, limited_boxes, limited_masks


app.layout = dbc.Container([
    html.H3("G1 Recognition Layer", className="mt-3"),
    html.Div(id="global-status", className="mb-2", style={"fontSize": "13px", "color": "#9c9"}),

    dbc.Row([
        dbc.Col(_instructions_card(), width=6),
        dbc.Col(_options_card(), width=6),
    ]),

    dbc.Row([
        dbc.Col([
            dbc.Label("Describe what to look for"),
            dbc.InputGroup([
                dbc.Input(
                    id="nl-prompt",
                    placeholder="e.g. red mug, soda can",
                    value=DEFAULT_DETECTION_PROMPT,
                ),
                dbc.Button("Set prompt", id="set-prompt-btn", color="primary"),
            ]),
            html.Div(id="prompt-status", className="mt-1", style={"fontSize": "12px"}),
        ], width=8),
    ], className="mb-3"),

    dbc.Row([
        dbc.Col(_view_panel(), width=6),
        dbc.Col(_logs_card(), width=6),
    ]),

    dbc.Row([
        dbc.Col([
            html.H5("Detections — select one"),
            html.Div(id="detection-list"),
        ], width=6),
        dbc.Col([
            html.H5("Grab"),
            dbc.RadioItems(
                id="arm-select",
                options=[
                    {"label": "Auto (pick by target side)", "value": "auto"},
                    {"label": "Left", "value": "left"},
                    {"label": "Right", "value": "right"},
                ],
                value="right", inline=True, className="mb-2",
            ),
            dbc.RadioItems(
                id="planning-mode-select",
                options=[
                    {"label": "Direct (analytic IK)", "value": "direct"},
                    {
                        "label": "VLA — obstacle-aware trajectory planning (experimental, no model wired in)",
                        "value": "vla",
                    },
                ],
                value="direct", inline=False, className="mb-2",
            ),
            dbc.Checkbox(
                id="auto-step-base",
                label="Step base closer if target is outside arm reach",
                value=False,
                className="mb-2",
            ),
            dbc.Label("Object mass (kg) — sets grip force target and lift stiffness", className="mt-1"),
            dbc.Input(
                id="object-mass-kg",
                type="number",
                min=0,
                max=MAX_OBJECT_MASS_KG,
                step=0.05,
                value=DEFAULT_OBJECT_MASS_KG,
                className="mb-2",
            ),
            dbc.Button("Grab selected object", id="grab-btn", color="success",
                        className="mb-2", disabled=True),
            html.Div(id="grab-selected-label", className="mb-2", style={"fontSize": "13px"}),
            html.Div(id="obstacle-status", className="mb-2", style={"fontSize": "12px", "color": "#aad"}),
            html.Div(id="tactile-status", className="mb-2", style={"fontSize": "12px", "color": "#ada"}),
            html.Hr(),
            dbc.ButtonGroup([
                dbc.Button("Prepare", id="prepare-btn", color="secondary"),
                dbc.Button("Walk", id="walk-btn", color="secondary"),
                dbc.Button("Stop moving", id="stop-moving-btn", color="dark"),
                dbc.Button("Extend arm", id="extend-arm-btn", color="info"),
                dbc.Button("Stop grabbing", id="stop-grabbing-btn", color="danger"),
                dbc.Button("Release arms", id="release-arms-btn", color="warning"),
                dbc.Button("Re-engage arms", id="unrelease-arms-btn", color="primary"),
                dbc.Button("Open hand", id="hand-open-btn", color="info"),
                dbc.Button("Close hand", id="hand-close-btn", color="info"),
                dbc.Button("Damp", id="damp-btn", color="danger"),
            ]),
            html.Div(id="safety-status", className="mt-2", style={"fontSize": "12px"}),
        ], width=6),
    ]),

    dcc.Store(id="selected-store"),
    dcc.Interval(id="refresh-interval", interval=500, n_intervals=0),
], fluid=True)


# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------

@app.callback(
    Output("panel-view", "src"),
    Output("detection-list", "children"),
    Output("global-status", "children"),
    Output("camera-tf-status", "children"),
    Output("grab-log", "children"),
    Output("grab-btn", "disabled"),
    Output("grab-selected-label", "children"),
    Output("obstacle-status", "children"),
    Output("tactile-status", "children"),
    Input("refresh-interval", "n_intervals"),
    Input("view-select", "value"),
)
def _refresh(_n, view_name):
    with STATE.lock:
        rgb = None if STATE.rgb_bgr is None else STATE.rgb_bgr.copy()
        depth = None if STATE.depth_m is None else STATE.depth_m.copy()
        detection_rgb = None if STATE.detection_rgb_bgr is None else STATE.detection_rgb_bgr.copy()
        detection_depth = None if STATE.detection_depth_m is None else STATE.detection_depth_m.copy()
        detections = dict(STATE.detections)
        boxes = list(STATE.boxes_for_draw)
        masks = list(STATE.masks_for_draw)
        selected_id = STATE.selected_id
        status_msg = STATE.status_msg
        grab_log = list(STATE.grab_log[-100:])
        grab_running = STATE.grab_running
        arm_motion_running = STATE.arm_motion_running
        arm_motion_label = STATE.arm_motion_label
        last_frame_ts = STATE.last_frame_ts
        last_detection_ts = STATE.last_detection_ts
        cam = dict(STATE.camera_extrinsic)
        arm_override = STATE.arm_override
        hand_fk_T_base = {k: v.copy() for k, v in STATE.hand_fk_T_base.items()}
        extended_hand_R_base = {
            k: v.copy() for k, v in STATE.extended_hand_R_base.items()
        }
        camera_tf_status = STATE.camera_tf_status
        max_visible_detections = STATE.max_visible_detections
        obstacle_status = STATE.obstacle_status

    tactile_status = _tactile_status_text()

    if rgb is None:
        ph = _placeholder_src("waiting for camera…")
        return (
            ph, "No detections yet.", status_msg, camera_tf_status,
            "\n".join(grab_log), True, "", obstacle_status, tactile_status,
        )

    # A frozen/erroring camera feed must not keep offering stale objects as
    # if the robot could still see them — the list only reflects detections
    # from a genuinely recent perception pass.
    now = time.time()
    frame_is_stale = (now - last_frame_ts) > STALE_AFTER_S
    detections_are_stale = (
        last_detection_ts <= 0.0 or (now - last_detection_ts) > STALE_AFTER_S
    )
    is_stale = frame_is_stale or detections_are_stale
    if is_stale:
        detections = {}
        boxes = []
        masks = []
        if frame_is_stale:
            status_msg = f"STALE — no fresh frame in >{STALE_AFTER_S:.0f}s. {status_msg}"
        else:
            status_msg = f"DETECTING — latest detection is >{STALE_AFTER_S:.0f}s old. {status_msg}"
    else:
        total_detections = len(detections)
        detections, boxes, masks = _limit_visible_detections(
            detections, boxes, masks, max_visible_detections,
        )
        if len(detections) < total_detections:
            status_msg = (
                f"{status_msg} | showing {len(detections)}/{total_detections} "
                f"detections"
            )
    view_name = view_name or "detections"
    if view_name == "rgb":
        view_src = _encode_jpeg_src(rgb)
    elif view_name == "depth":
        view_src = _encode_jpeg_src(_colorize_depth(depth)) if depth is not None else _placeholder_src("no depth")
    elif view_name == "segmentation":
        base = detection_rgb if detection_rgb is not None else rgb
        view_src = _encode_jpeg_src(draw_segmentation_overlay(base, masks)) if masks else _placeholder_src("no masks")
    else:
        base = detection_rgb if detection_rgb is not None else rgb
        selected = detections.get(selected_id or "")
        boxed = draw_detection_boxes(base, boxes)
        view_src = _encode_jpeg_src(_draw_detection_gizmos(
            boxed,
            selected,
            cam,
            arm_override,
            hand_fk_T_base,
            extended_hand_R_base,
        ))

    items = []
    for det_id, det in sorted(detections.items()):
        active = det_id == selected_id
        items.append(dbc.ListGroupItem(
            [
                dbc.Badge(det.source, color="primary", className="me-2"),
                f"{det.label}  (score={det.score:.2f})",
            ],
            id={"type": "det-item", "index": det_id},
            action=True, active=active, n_clicks=0,
        ))
    if items:
        det_list = dbc.ListGroup(items)
    elif is_stale:
        det_list = html.Div(
            "Camera feed is stale — the robot isn't reporting fresh frames, "
            "so no objects can be confirmed as currently visible.",
            className="text-danger",
        )
    else:
        det_list = html.Div(
            "No detections. Set an NL prompt and keep the object in the RGB-D view.",
            className="text-muted",
        )

    grab_disabled = (selected_id not in detections) or grab_running or arm_motion_running
    selected_label = ""
    if selected_id in detections:
        det = detections[selected_id]
        with STATE.lock:
            cam = dict(STATE.camera_extrinsic)
            arm_override = STATE.arm_override
            auto_step_base = STATE.auto_step_base
        arm, reach_dist, max_reach, excess_m, suggested_step_m = _reach_preview(
            det, cam, arm_override,
        )
        selected_label = (
            f"Selected: {det.label} ({det.source}) | arm={arm} | "
            f"wrist reach={reach_dist:.3f}/{max_reach:.3f} m"
        )
        if det.grasp_surface is not None:
            gs = det.grasp_surface
            standoff_m = _grasp_standoff_for_surface(gs)
            selected_label += (
                f" | est. size {gs.width_m * 100:.1f}x{gs.length_m * 100:.1f} cm "
                f"(conf={gs.confidence:.2f}) | standoff={standoff_m * 100:.1f} cm"
            )
        if excess_m > 0.0:
            if auto_step_base:
                selected_label += f" | base step planned ~{suggested_step_m:.3f} m"
            else:
                selected_label += (
                    f" | outside arm reach by {excess_m:.3f} m; "
                    "enable base step or move closer"
                )
        if arm_motion_running:
            selected_label += f" | arm busy: {arm_motion_label}"
    elif selected_id is not None:
        selected_label = "Selected object is no longer visible."

    return (
        view_src, det_list, status_msg, camera_tf_status,
        "\n".join(grab_log), grab_disabled, selected_label, obstacle_status,
        tactile_status,
    )


@app.callback(
    Output("prompt-status", "children"),
    Input("set-prompt-btn", "n_clicks"),
    State("nl-prompt", "value"),
    prevent_initial_call=True,
)
def _set_prompt(_n, text):
    text = text or ""
    classes = _VISION.set_prompt(text)
    with STATE.lock:
        STATE.vision_classes = classes
    if not _VISION.available:
        return f"Vision model unavailable: {_VISION.error}"
    return f"Looking for: {', '.join(classes)}"


@app.callback(
    Output("selected-store", "data"),
    Input({"type": "det-item", "index": ALL}, "n_clicks"),
    State({"type": "det-item", "index": ALL}, "id"),
    prevent_initial_call=True,
)
def _select_detection(_clicks, ids):
    ctx = callback_context
    if not ctx.triggered:
        return no_update
    triggered_id = ctx.triggered[0]["prop_id"].split(".")[0]
    try:
        det_id = json.loads(triggered_id)["index"]
    except Exception:
        return no_update
    with STATE.lock:
        STATE.selected_id = det_id
    return det_id


@app.callback(
    Output("arm-select", "value"),
    Input("arm-select", "value"),
)
def _sync_arm(value):
    with STATE.lock:
        STATE.arm_override = value
    return value


@app.callback(
    Output("auto-step-base", "value"),
    Input("auto-step-base", "value"),
)
def _sync_auto_step(value):
    enabled = bool(value)
    with STATE.lock:
        STATE.auto_step_base = enabled
    return enabled


@app.callback(
    Output("object-mass-kg", "value"),
    Input("object-mass-kg", "value"),
)
def _sync_object_mass(value):
    try:
        mass_kg = float(value)
    except Exception:
        mass_kg = DEFAULT_OBJECT_MASS_KG
    mass_kg = max(0.0, min(MAX_OBJECT_MASS_KG, mass_kg))
    with STATE.lock:
        STATE.object_mass_kg = mass_kg
    return mass_kg


@app.callback(
    Output("max-visible-detections", "value"),
    Input("max-visible-detections", "value"),
)
def _sync_max_visible_detections(value):
    try:
        limit = max(1, int(value))
    except Exception:
        limit = 5
    with STATE.lock:
        STATE.max_visible_detections = limit
    return limit


@app.callback(
    Output("max-joint-step-rad", "value"),
    Input("max-joint-step-rad", "value"),
)
def _sync_max_joint_step_rad(value):
    try:
        step = float(value)
    except Exception:
        step = DEFAULT_MAX_JOINT_STEP_RAD
    step = max(0.005, min(ABSOLUTE_MAX_JOINT_STEP_RAD, step))
    with STATE.lock:
        STATE.max_joint_step_rad = step
    return step


@app.callback(
    Output("max-joint-speed-rad-s", "value"),
    Input("max-joint-speed-rad-s", "value"),
)
def _sync_max_joint_speed_rad_s(value):
    try:
        speed = float(value)
    except Exception:
        speed = DEFAULT_MAX_JOINT_SPEED_RAD_S
    speed = max(0.02, min(ABSOLUTE_MAX_JOINT_SPEED_RAD_S, speed))
    with STATE.lock:
        STATE.max_joint_speed_rad_s = speed
    return speed


@app.callback(
    Output("planning-mode-select", "value"),
    Input("planning-mode-select", "value"),
)
def _sync_planning_mode(value):
    mode = value if value in ("direct", "vla") else "direct"
    with STATE.lock:
        STATE.planning_mode = mode
        if mode == "vla":
            STATE.obstacle_status = f"obstacles: VLA mode selected — {DEFAULT_VLA_POLICY.unavailable_reason}"
    return mode


@app.callback(
    Output("safety-status", "children"),
    Input("grab-btn", "n_clicks"),
    Input("prepare-btn", "n_clicks"),
    Input("walk-btn", "n_clicks"),
    Input("stop-moving-btn", "n_clicks"),
    Input("extend-arm-btn", "n_clicks"),
    Input("stop-grabbing-btn", "n_clicks"),
    Input("release-arms-btn", "n_clicks"),
    Input("unrelease-arms-btn", "n_clicks"),
    Input("hand-open-btn", "n_clicks"),
    Input("hand-close-btn", "n_clicks"),
    Input("damp-btn", "n_clicks"),
    prevent_initial_call=True,
)
def _buttons(
    grab_clicks,
    prepare_clicks,
    walk_clicks,
    stop_moving_clicks,
    extend_clicks,
    stop_grabbing_clicks,
    release_clicks,
    unrelease_clicks,
    hand_open_clicks,
    hand_close_clicks,
    damp_clicks,
):
    ctx = callback_context
    if not ctx.triggered:
        return no_update
    trigger = ctx.triggered[0]["prop_id"].split(".")[0]

    if trigger == "grab-btn":
        _log("[recognition_app] Grab button clicked.")
        threading.Thread(target=_run_grab, daemon=True).start()
        return "Grab started — see log below."
    if trigger == "prepare-btn":
        threading.Thread(target=_run_prepare, daemon=True).start()
        return "Prepare sent..."
    if trigger == "walk-btn":
        threading.Thread(target=_run_walk, daemon=True).start()
        return "Walk sent..."
    if trigger == "stop-moving-btn":
        threading.Thread(target=_run_stop_moving, daemon=True).start()
        return "Stop moving sent..."
    if trigger == "extend-arm-btn":
        threading.Thread(target=_run_extend_arm, daemon=True).start()
        return "Extending arm..."
    if trigger == "stop-grabbing-btn":
        threading.Thread(target=_run_stop_grabbing, daemon=True).start()
        return "Stopping grab..."
    if trigger == "release-arms-btn":
        threading.Thread(target=_run_release_arms, daemon=True).start()
        return "Releasing arms…"
    if trigger == "unrelease-arms-btn":
        threading.Thread(target=_run_unrelease_arms, daemon=True).start()
        return "Re-engaging arms..."
    if trigger == "hand-open-btn":
        threading.Thread(target=_run_hand_open, daemon=True).start()
        return "Opening hand..."
    if trigger == "hand-close-btn":
        threading.Thread(target=_run_hand_close, daemon=True).start()
        return "Closing hand..."
    if trigger == "damp-btn":
        threading.Thread(target=_run_damp, daemon=True).start()
        return "Damping…"
    return no_update


# ---------------------------------------------------------------------------
# Background actions
# ---------------------------------------------------------------------------

def _run_release_arms() -> None:
    ARM_CANCEL_EVENT.set()
    acquired = ARM_CONTROL_LOCK.acquire(timeout=3.0)
    if not acquired:
        with STATE.lock:
            STATE.status_msg = "release_arms() waiting: arm controller is busy"
        ARM_CONTROL_LOCK.acquire()
    try:
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "release arms"
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        result = _CONTROL_ROBOT.release_arms()
        global _ARM_AUTHORITY_ENGAGED
        _ARM_AUTHORITY_ENGAGED = False
        with STATE.lock:
            STATE.status_msg = f"release_arms() done: {result}"
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"release_arms() failed: {exc}"
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CANCEL_EVENT.clear()
        ARM_CONTROL_LOCK.release()


def _run_unrelease_arms() -> None:
    if not ARM_CONTROL_LOCK.acquire(blocking=False):
        with STATE.lock:
            STATE.status_msg = "unrelease_arms() skipped: arm controller is busy"
        return
    try:
        ARM_CANCEL_EVENT.clear()
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "re-engage arms"
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        if not hasattr(_CONTROL_ROBOT, "unrelease_arms"):
            raise AttributeError("control robot has no unrelease_arms()")
        result = _CONTROL_ROBOT.unrelease_arms()
        global _ARM_AUTHORITY_ENGAGED
        _ARM_AUTHORITY_ENGAGED = True
        with STATE.lock:
            STATE.status_msg = f"unrelease_arms() done: {result}"
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"unrelease_arms() failed: {exc}"
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CONTROL_LOCK.release()


def _hand_side_from_override(arm_override: str) -> str:
    side = str(arm_override).strip().lower()
    if side not in ("left", "right"):
        side = "right"
    return side


def _run_hand_open() -> None:
    _run_hand_action("open")


def _run_hand_close() -> None:
    _run_hand_action("close")


def _hand_action_target(action: str, side: str) -> Optional[List[float]]:
    if not _HAND_TARGETS_AVAILABLE:
        return None
    try:
        if action == "open" and hand_open_targets is not None:
            return [float(v) for v in hand_open_targets(side)]
        if action == "close" and hand_closed_targets is not None:
            return [float(v) for v in hand_closed_targets(side)]
    except Exception as exc:
        _log(f"[recognition_app] cannot build hand_{action} target for {side}: {exc}")
    return None


def _hand_snapshot(side: str) -> Optional[Dict[str, Any]]:
    if _CONTROL_ROBOT is None:
        return None
    try:
        snapshot = _CONTROL_ROBOT.get_hand_state_snapshot(side)
    except Exception:
        snapshot = None
    if snapshot is not None:
        return snapshot
    try:
        controller = _CONTROL_ROBOT._get_hand(side)
        if hasattr(controller, "get_state_snapshot"):
            state = controller.get_state_snapshot(max_age=1.0)
            if state is not None:
                positions = state.get("positions")
                if isinstance(positions, list):
                    from sdk_hand import HAND_JOINT_NAMES
                    state["positions"] = {
                        HAND_JOINT_NAMES[idx]: positions[idx]
                        for idx in range(min(len(positions), len(HAND_JOINT_NAMES)))
                    }
                return state
    except Exception:
        return None
    return None


def _hand_target_error(side: str, target: List[float]) -> Optional[float]:
    snapshot = _hand_snapshot(side)
    if snapshot is None:
        return None
    positions = snapshot.get("positions", {})
    if not isinstance(positions, dict):
        return None
    try:
        from sdk_hand import HAND_JOINT_NAMES
        actual = [float(positions[name]) for name in HAND_JOINT_NAMES]
    except Exception:
        return None
    return max(abs(goal - current) for goal, current in zip(target, actual))


def _reset_hand_controller(side: str) -> None:
    if _CONTROL_ROBOT is None:
        return
    hands = getattr(_CONTROL_ROBOT, "_hands", None)
    if not isinstance(hands, dict):
        return
    controller = hands.pop(side, None)
    if controller is not None and hasattr(controller, "stop_release_fingers"):
        try:
            controller.stop_release_fingers()
        except Exception:
            pass
    _log(f"[recognition_app] reset {side} Dex3 controller instance.")


def _send_hand_action(action: str, side: str, hold_s: Optional[float] = None) -> bool:
    if _CONTROL_ROBOT is None:
        raise RuntimeError(_CONTROL_STATUS)
    target = _hand_action_target(action, side)
    snapshot = _hand_snapshot(side)
    if snapshot is None:
        feedback_available = False
        _log(
            f"[recognition_app] {side} Dex3 state feedback unavailable "
            f"(no rt/dex3/{side}/state messages); sending hand_{action} anyway."
        )
    else:
        feedback_available = True

    command_hold_s = DEFAULT_HAND_COMMAND_HOLD_S if hold_s is None else float(hold_s)
    if action == "open":
        _CONTROL_ROBOT.hand_open(hand=side, hold_s=command_hold_s, ramp_s=0.25)
    elif hold_s is None:
        _CONTROL_ROBOT.hand_close(hand=side, hold_s=command_hold_s, ramp_s=0.25)
    else:
        _CONTROL_ROBOT.hand_close(hand=side, hold_s=command_hold_s, ramp_s=0.20)

    should_recover = target is not None and (action == "open" or hold_s is None)
    if should_recover:
        err = _hand_target_error(side, target)
        if err is not None and err > DEFAULT_HAND_TARGET_TOL_RAD:
            _log(
                f"[recognition_app] hand_{action}({side}) missed target "
                f"(max_err={err:.3f}rad); retrying explicit pose."
            )
            if hasattr(_CONTROL_ROBOT, "hand_pose"):
                _CONTROL_ROBOT.hand_pose(
                    target,
                    hand=side,
                    hold_s=DEFAULT_HAND_RECOVERY_HOLD_S,
                    rate_hz=50.0,
                    kp=1.8,
                    kd=0.12,
                    tau=0.08,
                    ramp_s=0.25,
                )
            else:
                if action == "open":
                    _CONTROL_ROBOT.hand_open(hand=side, hold_s=DEFAULT_HAND_RECOVERY_HOLD_S, ramp_s=0.25)
                else:
                    _CONTROL_ROBOT.hand_close(hand=side, hold_s=DEFAULT_HAND_RECOVERY_HOLD_S, ramp_s=0.25)
            retry_err = _hand_target_error(side, target)
            if retry_err is not None and retry_err > DEFAULT_HAND_TARGET_TOL_RAD:
                _log(
                    f"[recognition_app] hand_{action}({side}) still off target "
                    f"(max_err={retry_err:.3f}rad); recreating hand controller and retrying."
                )
                _reset_hand_controller(side)
                if hasattr(_CONTROL_ROBOT, "hand_pose"):
                    _CONTROL_ROBOT.hand_pose(
                        target,
                        hand=side,
                        hold_s=DEFAULT_HAND_RECOVERY_HOLD_S,
                        rate_hz=50.0,
                        kp=1.8,
                        kd=0.12,
                        tau=0.08,
                        ramp_s=0.25,
                    )
                elif action == "open":
                    _CONTROL_ROBOT.hand_open(hand=side, hold_s=DEFAULT_HAND_RECOVERY_HOLD_S, ramp_s=0.25)
                else:
                    _CONTROL_ROBOT.hand_close(hand=side, hold_s=DEFAULT_HAND_RECOVERY_HOLD_S, ramp_s=0.25)
    return feedback_available


def _run_hand_action(action: str) -> None:
    with STATE.lock:
        arm_override = STATE.arm_override
    side = _hand_side_from_override(arm_override)
    try:
        feedback_available = _send_hand_action(action, side)
        with STATE.lock:
            suffix = "" if feedback_available else " (feedback unavailable)"
            STATE.status_msg = f"hand_{action}({side}) sent{suffix}."
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"hand_{action}({side}) failed: {exc}"


# ---------------------------------------------------------------------------
# Dex3 tactile feedback + mass-aware grip control
# ---------------------------------------------------------------------------

def _is_invalid_tactile_value(value: float) -> bool:
    return abs(float(value) - TACTILE_INVALID_VALUE) < 0.5


def _max_tactile_pressure(side: str) -> Optional[float]:
    """Highest current raw fingertip pressure reading for one hand, or None
    if tactile feedback isn't available (mock mode, RGBD-only control robot,
    or no rt/dex3/{side}/state messages yet).
    """
    if _CONTROL_ROBOT is None or not hasattr(_CONTROL_ROBOT, "get_tactile_pressures"):
        return None
    try:
        sensors = _CONTROL_ROBOT.get_tactile_pressures(side)
    except Exception:
        return None
    if not sensors:
        return None
    valid_values = [
        float(value)
        for sensor_values in sensors
        for value in sensor_values
        if not _is_invalid_tactile_value(value)
    ]
    if not valid_values:
        return 0.0
    return max(valid_values)


def _tactile_status_text() -> str:
    if _CONTROL_ROBOT is None or not hasattr(_CONTROL_ROBOT, "get_tactile_pressures"):
        return "tactile: unavailable (no Dex3 state feed)"
    parts = []
    for side in ("left", "right"):
        pressure = _max_tactile_pressure(side)
        if pressure is None:
            parts.append(f"{side}=n/a")
        else:
            contact = "contact" if pressure >= TACTILE_VALID_THRESHOLD else "no contact"
            parts.append(f"{side}={pressure:.0f} raw ({contact})")
    return "tactile: " + "  ".join(parts)


def _grip_target_pressure_raw(mass_kg: float) -> float:
    """Heavier objects need a firmer grip before the hand stops closing —
    scale the target fingertip pressure with the entered object mass.
    """
    mass_kg = max(0.0, float(mass_kg))
    target = DEFAULT_TACTILE_BASE_TARGET_RAW + DEFAULT_TACTILE_PER_KG_RAW * mass_kg
    return float(min(DEFAULT_TACTILE_MAX_TARGET_RAW, target))


def _tactile_guided_hand_close(arm: str, mass_kg: float) -> Dict[str, Any]:
    """Close the hand in small steps, checking measured fingertip pressure
    after each one, and stop as soon as the mass-scaled target pressure is
    reached instead of always running the fixed hand_close() sequence to
    completion. Falls back to the original one-shot close when tactile
    feedback or per-joint hand_pose() targets aren't available (mock mode,
    RGBD-only control robot, sdk_hand target helpers missing).
    """
    side = arm if arm in ("left", "right") else _hand_side_from_override(arm)
    open_t = _hand_action_target("open", side)
    closed_t = _hand_action_target("close", side)
    tactile_ready = (
        open_t is not None
        and closed_t is not None
        and len(open_t) == len(closed_t)
        and _CONTROL_ROBOT is not None
        and hasattr(_CONTROL_ROBOT, "hand_pose")
        and hasattr(_CONTROL_ROBOT, "get_tactile_pressures")
    )
    if not tactile_ready:
        _log(f"[recognition_app] Tactile-guided close unavailable for {side}; falling back to fixed close.")
        _send_hand_action("close", side, hold_s=0.6)
        return {"success": True, "method": "fixed_close", "side": side}

    target_pressure = _grip_target_pressure_raw(mass_kg)
    _log(
        f"[recognition_app] Tactile-guided close ({side}): object_mass={mass_kg:.2f}kg "
        f"target_pressure={target_pressure:.0f} raw."
    )
    steps = DEFAULT_TACTILE_CLOSE_STEPS
    last_pressure = 0.0
    last_fraction = 0.0
    for step in range(1, steps + 1):
        if ARM_CANCEL_EVENT.is_set():
            _log(f"[recognition_app] Tactile-guided close cancelled at step {step}/{steps}.")
            return {"success": False, "method": "tactile", "reason": "cancelled", "side": side}
        fraction = step / float(steps)
        target_positions = [o + fraction * (c - o) for o, c in zip(open_t, closed_t)]
        try:
            _CONTROL_ROBOT.hand_pose(
                target_positions, hand=side,
                hold_s=DEFAULT_TACTILE_STEP_HOLD_S, rate_hz=50.0,
                kp=1.4, kd=0.06, tau=0.06, ramp_s=0.06,
            )
        except Exception as exc:
            _log(f"[recognition_app] Tactile-guided close step {step} command failed: {exc}")
            break
        time.sleep(DEFAULT_TACTILE_STEP_SETTLE_S)
        pressure = _max_tactile_pressure(side)
        last_fraction = fraction
        if pressure is not None:
            last_pressure = pressure
        _log(
            f"[recognition_app] grip step {step}/{steps} fraction={fraction:.2f} "
            f"pressure={'n/a' if pressure is None else f'{pressure:.0f}'} raw "
            f"target={target_pressure:.0f} raw"
        )
        if pressure is not None and pressure >= target_pressure:
            _log(
                f"[recognition_app] Grip target reached ({side}): "
                f"fraction={fraction:.2f} pressure={pressure:.0f} raw."
            )
            result = {
                "success": True, "method": "tactile", "side": side,
                "fraction": fraction, "pressure": pressure,
                "target_pressure": target_pressure, "mass_kg": mass_kg,
            }
            with STATE.lock:
                STATE.last_grip_result = dict(result)
            return result

    _log(
        f"[recognition_app] Tactile-guided close reached full closure without hitting "
        f"target pressure ({side}); last_pressure={last_pressure:.0f} raw "
        f"target={target_pressure:.0f} raw (object may be thinner/lighter than expected, "
        "or tactile feedback is unavailable)."
    )
    result = {
        "success": True, "method": "tactile_full_close", "side": side,
        "fraction": last_fraction, "pressure": last_pressure,
        "target_pressure": target_pressure, "mass_kg": mass_kg,
    }
    with STATE.lock:
        STATE.last_grip_result = dict(result)
    return result


def _lift_tuning_for_mass(mass_kg: float) -> Tuple[float, float, float, float]:
    """Post-grasp lift stiffness (kp/kd), duration, and speed scale for the
    entered object mass: heavier objects get a stiffer hold, more time to
    get there, and a slower joint speed so the lift is less likely to shake
    the object loose.
    """
    mass_kg = max(0.0, float(mass_kg))
    kp = min(MAX_LIFT_KP, DEFAULT_LIFT_KP + LIFT_KP_PER_KG * mass_kg)
    kd = min(MAX_LIFT_KD, DEFAULT_LIFT_KD + LIFT_KD_PER_KG * mass_kg)
    duration_s = min(
        MAX_LIFT_DURATION_S,
        DEFAULT_POST_GRASP_LIFT_DURATION_S * (1.0 + LIFT_DURATION_PER_KG * mass_kg),
    )
    speed_scale = 1.0 / (1.0 + LIFT_SPEED_SLOWDOWN_PER_KG * mass_kg)
    return kp, kd, duration_s, speed_scale


def _run_damp() -> None:
    ARM_CANCEL_EVENT.set()
    acquired = ARM_CONTROL_LOCK.acquire(timeout=3.0)
    if not acquired:
        with STATE.lock:
            STATE.status_msg = "damp() waiting: arm controller is busy"
        ARM_CONTROL_LOCK.acquire()
    try:
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "damp"
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        _CONTROL_ROBOT.damp()
        global _ARM_AUTHORITY_ENGAGED
        _ARM_AUTHORITY_ENGAGED = False
        with STATE.lock:
            STATE.status_msg = "damp() sent."
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"damp() failed: {exc}"
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CANCEL_EVENT.clear()
        ARM_CONTROL_LOCK.release()


def _run_prepare() -> None:
    try:
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        if hasattr(_CONTROL_ROBOT, "prepare"):
            _CONTROL_ROBOT.prepare()
        else:
            client = getattr(_CONTROL_ROBOT, "_client", None)
            if client is None or not hasattr(client, "SetFsmId"):
                raise AttributeError("control robot has no prepare or SetFsmId")
            client.SetFsmId(4)
        with STATE.lock:
            STATE.status_msg = "prepare FSM 4 sent."
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"prepare failed: {exc}"


def _run_walk() -> None:
    try:
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        if hasattr(_CONTROL_ROBOT, "walk_mode"):
            _CONTROL_ROBOT.walk_mode()
        else:
            client = getattr(_CONTROL_ROBOT, "_client", None)
            if client is None or not hasattr(client, "SetFsmId"):
                raise AttributeError("control robot has no walk_mode or SetFsmId")
            client.SetFsmId(501)
        with STATE.lock:
            STATE.status_msg = "walk FSM 501 sent."
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"walk failed: {exc}"


def _run_stop_moving() -> None:
    try:
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        if hasattr(_CONTROL_ROBOT, "loco_move"):
            _CONTROL_ROBOT.loco_move(0.0, 0.0, 0.0)
        else:
            client = getattr(_CONTROL_ROBOT, "_client", None)
            if client is None or not hasattr(client, "Move"):
                raise AttributeError("control robot has no loco_move or Move")
            client.Move(0.0, 0.0, 0.0, continous_move=False)
        with STATE.lock:
            STATE.status_msg = "loco velocity zero sent."
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"stop moving failed: {exc}"


def _run_stop_grabbing() -> None:
    ARM_CANCEL_EVENT.set()
    _log("[recognition_app] Stop grabbing requested.")
    with ACTIVE_NAV_LOCK:
        nav = ACTIVE_DIRECT_NAV
    if nav is not None:
        try:
            nav.shutdown()
            _log("[recognition_app] Active direct nav stopped.")
        except Exception as exc:
            _log(f"[recognition_app] Active direct nav stop failed: {exc}")

    acquired = ARM_CONTROL_LOCK.acquire(timeout=3.0)
    if not acquired:
        with STATE.lock:
            STATE.status_msg = "stop grabbing waiting: arm controller is busy"
        ARM_CONTROL_LOCK.acquire()
    try:
        with STATE.lock:
            STATE.grab_running = False
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "stop grabbing"
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        if not hasattr(_CONTROL_ROBOT, "release_arms"):
            raise AttributeError("control robot has no release_arms()")
        result = _CONTROL_ROBOT.release_arms(duration_s=0.5)
        global _ARM_AUTHORITY_ENGAGED
        _ARM_AUTHORITY_ENGAGED = False
        _log(f"[recognition_app] stop grabbing release_arms() done: {result}")
        with STATE.lock:
            STATE.status_msg = "grab stopped; arms released."
    except Exception as exc:
        _log(f"[recognition_app] stop grabbing failed: {exc}")
        with STATE.lock:
            STATE.status_msg = f"stop grabbing failed: {exc}"
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CANCEL_EVENT.clear()
        ARM_CONTROL_LOCK.release()


def _read_current_arm_q(robot, arm: str) -> np.ndarray:
    js = robot.get_joint_states()
    joints = js.get("joints", {}) if js else {}
    joint_indices = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
    seen_indices = set()
    q_full = np.zeros(30, dtype=np.float64)
    for name, data in joints.items():
        idx = _joint_entry_index(name, data)
        if 0 <= idx < q_full.size and data.get("position") is not None:
            q_full[idx] = float(data.get("position"))
            seen_indices.add(idx)
    missing = [idx for idx in joint_indices if idx not in seen_indices]
    if missing:
        raise RuntimeError(f"cannot read current {arm} arm joints from lowstate; missing indices {missing}")
    return q_full[joint_indices]


def _read_joint_position(robot, joint_index: int) -> float:
    js = robot.get_joint_states()
    joints = js.get("joints", {}) if js else {}
    for name, data in joints.items():
        idx = _joint_entry_index(name, data)
        if idx == int(joint_index) and data.get("position") is not None:
            return float(data.get("position"))
    raise RuntimeError(f"cannot read joint index {joint_index} from lowstate")


def _center_object_with_waist_yaw(
    det: Detection,
    selected_id: Optional[str],
    max_joint_speed_rad_s: float,
) -> Detection:
    if _CONTROL_ROBOT is None:
        raise RuntimeError(_CONTROL_STATUS)
    p_cam = det.T_camera_object[:3, 3]
    z = float(p_cam[2])
    if z <= 0.05:
        _log("[recognition_app] Waist centering skipped: invalid object depth.")
        return det
    yaw_delta = -math.atan2(float(p_cam[0]), z)
    yaw_delta = max(
        -DEFAULT_CENTER_WAIST_MAX_YAW_RAD,
        min(DEFAULT_CENTER_WAIST_MAX_YAW_RAD, yaw_delta),
    )
    if abs(yaw_delta) < DEFAULT_CENTER_WAIST_DEADBAND_RAD:
        _log(
            "[recognition_app] Waist centering skipped: "
            f"object already near camera center (yaw_delta={yaw_delta:+.3f}rad)."
        )
        return det
    if not ARM_CONTROL_LOCK.acquire(blocking=False):
        _log("[recognition_app] Waiting for current arm motion before waist centering.")
        ARM_CONTROL_LOCK.acquire()
    try:
        if ARM_CANCEL_EVENT.is_set():
            _log("[recognition_app] Waist centering cancelled before control handoff.")
            return det
        ARM_CANCEL_EVENT.clear()
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "waist center"
        _ensure_arm_authority(_CONTROL_ROBOT, duration_s=0.3)
        current_yaw = _read_joint_position(_CONTROL_ROBOT, WAIST_YAW_JOINT_INDEX)
        target_yaw = current_yaw + yaw_delta
        speed = min(
            DEFAULT_CENTER_WAIST_SPEED_RAD_S,
            max(0.05, float(max_joint_speed_rad_s)),
        )
        _log(
            "[recognition_app] Centering object with waist yaw: "
            f"current={current_yaw:+.3f}rad target={target_yaw:+.3f}rad "
            f"delta={yaw_delta:+.3f}rad."
        )
        result = _CONTROL_ROBOT.move_upper_body_joint(
            joint_index=WAIST_YAW_JOINT_INDEX,
            target=target_yaw,
            max_speed_rad_s=speed,
            timeout=2.0,
        )
        _log(f"[recognition_app] waist yaw center result: {result}")
    except Exception as exc:
        _log(f"[recognition_app] Waist centering failed; continuing with original target: {exc}")
        return det
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CONTROL_LOCK.release()

    min_detection_ts = time.time()
    deadline = min_detection_ts + max(DEFAULT_CENTER_WAIST_SETTLE_S, STALE_AFTER_S)
    refreshed: Optional[Detection] = None
    while time.time() < deadline:
        time.sleep(0.1)
        with STATE.lock:
            candidate = STATE.detections.get(selected_id) if selected_id else None
            age_s = time.time() - STATE.last_detection_ts if STATE.last_detection_ts else float("inf")
            detection_ts = STATE.last_detection_ts
        if (
            candidate is not None
            and detection_ts >= min_detection_ts
            and age_s <= STALE_AFTER_S
        ):
            refreshed = _copy_detection(candidate)
            break
    if refreshed is None:
        _log("[recognition_app] No fresh detection after waist centering; using original target.")
        return det
    p_new = refreshed.T_camera_object[:3, 3]
    _log(
        "[recognition_app] Refreshed target after waist centering: "
        f"camera xyz=({p_new[0]:+.3f}, {p_new[1]:+.3f}, {p_new[2]:+.3f}) m."
    )
    return refreshed


def _extend_arm_forward_joint_waypoints(q_start: np.ndarray, arm: str) -> List[Tuple[str, np.ndarray]]:
    side = str(arm).strip().lower()
    if side not in ("left", "right"):
        raise ValueError("arm must be 'left' or 'right'")
    limits = JOINT_LIMITS[side]

    def clamp_i(i: int, value: float) -> float:
        lo, hi = limits[i]
        return max(lo, min(hi, float(value)))

    roll_delta = 0.50 if side == "left" else -0.50
    pitch_delta = -0.35
    restore_fraction = 0.45

    start = np.asarray(q_start, dtype=np.float64).copy()
    roll_pose = start.copy()
    roll_pose[1] = clamp_i(1, start[1] + roll_delta)

    pitch_pose = roll_pose.copy()
    pitch_pose[0] = clamp_i(0, roll_pose[0] + pitch_delta)

    restored_roll_pose = pitch_pose.copy()
    restored_roll_pose[1] = clamp_i(1, roll_pose[1] - (roll_delta * restore_fraction))

    target_pose = restored_roll_pose.copy()
    target_pose[3] = clamp_i(3, start[3] - 0.90)
    target_pose[4] = clamp_i(4, start[4] + 0.40)
    target_pose[5] = clamp_i(5, start[5] - 0.40)

    final_pose = target_pose.copy()
    final_pose[0] = clamp_i(0, target_pose[0] - 0.08)
    final_pose[1] = clamp_i(1, target_pose[1] - np.copysign(0.10, roll_delta))
    final_pose[3] = clamp_i(3, target_pose[3] - 0.10)
    final_pose[5] = clamp_i(5, target_pose[5] - 0.08)

    return [
        ("shoulder_roll_clearance", roll_pose),
        ("shoulder_pitch_forward", pitch_pose),
        ("partial_shoulder_roll_restore", restored_roll_pose),
        ("elbow_and_wrist_pitch", target_pose),
        ("final_forward_up_and_in", final_pose),
    ]


def _run_extend_arm_ee_ik(
    robot,
    arm: str,
    max_joint_step_rad: float,
    max_joint_speed_rad_s: float,
    cam: Dict[str, float],
) -> Dict[str, Any]:
    _ensure_arm_authority(robot, duration_s=0.4)
    q_cur = _read_current_arm_q(robot, arm)
    static_obstacles = _estimate_obstacles(cam)
    opposite_arm = "left" if arm == "right" else "right"
    fk = ArmFK(arm=arm, backend="urdf")
    ik = ArmIK(
        arm=arm,
        solver="dls",
        tol_pos_m=0.012,
        tol_rot_rad=0.20,
    )
    executor = ArmExecutor(
        robot,
        arm=arm,
        kp=30.0,
        kd=1.5,
        max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
        max_joint_step_rad=max_joint_step_rad,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
    )
    results = []
    for stage_name, q_stage_nominal in _extend_arm_forward_joint_waypoints(q_cur, arm):
        if ARM_CANCEL_EVENT.is_set():
            return {
                "success": False,
                "reason": "cancelled",
                "stage_results": results,
                "final_q": q_cur.copy() if results else None,
            }
        T_desired = fk.compute_arm(q_stage_nominal)
        if stage_name == "final_forward_up_and_in":
            T_desired = T_desired.copy()
            T_desired[1, 3] += DEFAULT_EXTEND_FINAL_Y_DECREASE_M
        q_solution, info = ik.solve(T_desired, q_init=q_cur)
        if q_solution is None:
            return {
                "success": False,
                "reason": f"ik failed at {stage_name}",
                "ik_info": info,
                "stage_results": results,
            }
        with STATE.lock:
            T_opposite = STATE.hand_fk_T_base.get(opposite_arm)
        stage_obstacles = Obstacles(
            table=static_obstacles.table,
            opposite_shoulder_base=SHOULDER_IN_BASE[opposite_arm] if T_opposite is not None else None,
            opposite_wrist_base=T_opposite[:3, 3].copy() if T_opposite is not None else None,
        )
        result = executor.execute(
            q_solution,
            duration_s=DEFAULT_EXTEND_STAGE_DURATION_S,
            q_arm_start=q_cur,
            T_base_desired=T_desired,
            stop_event=ARM_CANCEL_EVENT,
            obstacles=stage_obstacles,
        )
        results.append({"stage": stage_name, "ik": info, "execute": result})
        _log(
            "[recognition_app] extend stage "
            f"{stage_name}: success={result.get('success')} "
            f"reason={result.get('reason', '')} "
            f"duration={result.get('duration_s', 0.0):.2f}s "
            f"requested={result.get('requested_duration_s', DEFAULT_EXTEND_STAGE_DURATION_S):.2f}s "
            f"final_err={result.get('final_error_rad', 0.0):.4f}rad "
            f"capped_samples={result.get('capped_samples', 0)} "
            f"violations={result.get('violations', [])}"
        )
        if not result.get("success"):
            return {
                "success": False,
                "reason": result.get("reason"),
                "stage_results": results,
                "final_q": q_cur.copy() if len(results) > 1 else None,
            }
        q_cur = np.asarray(result.get("final_q", q_solution), dtype=np.float64)
    return {
        "success": True,
        "arm": arm,
        "controller": "ee_ik",
        "stages": [item["stage"] for item in results],
        "final_q": q_cur.copy(),
    }


def _save_extend_arm_reference(arm: str, q_ref: np.ndarray, source: str) -> None:
    q_ref = np.asarray(q_ref, dtype=np.float64).copy()
    T_ref = ArmFK(arm=arm, backend="urdf").compute_arm(q_ref)
    with STATE.lock:
        STATE.extended_hand_R_base[arm] = T_ref[:3, :3].copy()
        STATE.extended_hand_reference_q[arm] = q_ref.copy()
        STATE.extended_hand_reference_T_base[arm] = T_ref.copy()
    _log(
        f"[recognition_app] saved {arm} extend-arm reference from {source}: "
        f"xyz=({T_ref[0, 3]:+.3f}, {T_ref[1, 3]:+.3f}, {T_ref[2, 3]:+.3f}) m."
    )


def _run_reset_to_extend_arm_reference(
    robot,
    arm: str,
    max_joint_step_rad: float,
    max_joint_speed_rad_s: float,
) -> Dict[str, Any]:
    with STATE.lock:
        q_ref = (
            None
            if arm not in STATE.extended_hand_reference_q
            else STATE.extended_hand_reference_q[arm].copy()
        )
        T_ref = (
            None
            if arm not in STATE.extended_hand_reference_T_base
            else STATE.extended_hand_reference_T_base[arm].copy()
        )
    if q_ref is None or T_ref is None:
        return {"success": False, "reason": "no_extend_arm_reference", "arm": arm}
    q_cur = _read_current_arm_q(robot, arm)
    executor = ArmExecutor(
        robot,
        arm=arm,
        kp=30.0,
        kd=1.5,
        max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
        max_joint_step_rad=max_joint_step_rad,
        max_joint_speed_rad_s=max_joint_speed_rad_s,
    )
    result = executor.execute(
        q_ref,
        duration_s=DEFAULT_EXTEND_STAGE_DURATION_S,
        q_arm_start=q_cur,
        T_base_desired=T_ref,
        stop_event=ARM_CANCEL_EVENT,
    )
    result["arm"] = arm
    result["controller"] = "saved_extend_arm_reference"
    return result


def _run_extend_arm() -> None:
    if not ARM_CONTROL_LOCK.acquire(blocking=False):
        with STATE.lock:
            STATE.status_msg = "extend arm skipped: arm controller is busy"
        return
    try:
        ARM_CANCEL_EVENT.clear()
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "extend arm"
        if _CONTROL_ROBOT is None:
            raise RuntimeError(_CONTROL_STATUS)
        with STATE.lock:
            arm_override = STATE.arm_override
            selected_id = STATE.selected_id
            det = STATE.detections.get(selected_id) if selected_id else None
            cam = dict(STATE.camera_extrinsic)
            max_joint_step_rad = STATE.max_joint_step_rad
            max_joint_speed_rad_s = STATE.max_joint_speed_rad_s
        arm = arm_override
        if arm == "auto":
            if det is not None:
                arm, _T_base_camera, _T_base_object = _resolve_arm_and_base_pose(det, cam, arm_override)
            else:
                arm = "right"
        result = _run_extend_arm_ee_ik(
            _CONTROL_ROBOT,
            arm,
            max_joint_step_rad,
            max_joint_speed_rad_s,
            cam,
        )
        if result.get("success") and result.get("final_q") is not None:
            _save_extend_arm_reference(arm, result["final_q"], "extend arm")
        with STATE.lock:
            if result.get("success"):
                STATE.status_msg = f"extend arm EE IK ({arm}) done: {result}"
            else:
                STATE.status_msg = f"extend arm EE IK ({arm}) failed: {result}"
    except Exception as exc:
        with STATE.lock:
            STATE.status_msg = f"extend arm failed: {exc}"
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CONTROL_LOCK.release()


def _log(line: str) -> None:
    with STATE.lock:
        STATE.grab_log.append(line)
        STATE.grab_log = STATE.grab_log[-300:]


def _shutdown_direct_nav(nav, context: str) -> None:
    global ACTIVE_DIRECT_NAV
    if nav is None:
        return
    try:
        nav.shutdown()
    except Exception as exc:
        _log(f"[recognition_app] {context} nav shutdown failed: {exc}")
    finally:
        with ACTIVE_NAV_LOCK:
            if ACTIVE_DIRECT_NAV is nav:
                ACTIVE_DIRECT_NAV = None


def _estimate_obstacles(cam: Dict[str, float]) -> Obstacles:
    """Fit a table plane from the current depth frame for obstacle-aware
    trajectory checking (Step 8b — see obstacle_checker.py). Table is None
    when no confident horizontal plane is found; the returned Obstacles is
    still valid and still gates torso/self-collision in that case.
    """
    with STATE.lock:
        depth = None if STATE.depth_m is None else STATE.depth_m.copy()
    table: Optional[TablePlane] = None
    if depth is not None:
        T_base_camera = _make_transform(
            xyz=(cam["x"], cam["y"], cam["z"]),
            rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
        )
        cam_mat = np.array(
            [[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        try:
            table = estimate_table_plane(depth, cam_mat, T_base_camera)
        except Exception as exc:
            _log(f"[recognition_app] table plane estimation failed: {exc}")
            table = None

    if table is not None:
        status = (
            f"obstacles: table z={table.z_base_m:+.3f}m "
            f"x=[{table.x_range_m[0]:+.2f},{table.x_range_m[1]:+.2f}] "
            f"y=[{table.y_range_m[0]:+.2f},{table.y_range_m[1]:+.2f}] "
            f"confidence={table.confidence:.2f} (n={table.inlier_count})"
        )
    else:
        status = "obstacles: no confident table plane found in current depth frame"
    _log(f"[recognition_app] {status}")
    with STATE.lock:
        STATE.obstacle_status = status
    return Obstacles(table=table)


def _hand_fk_visible_now(arm: str, cam: Dict[str, float]) -> bool:
    with STATE.lock:
        T_base_hand = (
            None
            if arm not in STATE.hand_fk_T_base
            else STATE.hand_fk_T_base[arm].copy()
        )
        hand_fk_ts = STATE.hand_fk_ts
    if T_base_hand is None or (time.time() - hand_fk_ts) > STALE_AFTER_S:
        return False
    T_base_camera = _make_transform(
        xyz=(cam["x"], cam["y"], cam["z"]),
        rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
    )
    p_h = np.ones(4, dtype=np.float64)
    p_h[:3] = _palm_visual_point(T_base_hand)
    p_camera = (np.linalg.inv(T_base_camera) @ p_h)[:3]
    x, y, z = (float(v) for v in p_camera)
    if z <= 0.05:
        return False
    u = K.fx * x / z + K.cx
    v = K.fy * y / z + K.cy
    return -10.0 <= u <= K.width + 10.0 and -10.0 <= v <= K.height + 10.0


def _prepare_hand_visibility_for_grab(
    arm: str,
    cam: Dict[str, float],
    max_joint_step_rad: float,
    max_joint_speed_rad_s: float,
) -> bool:
    if _hand_fk_visible_now(arm, cam):
        _log("[recognition_app] FK hand pose projects inside camera; opening hand before grab.")
        _send_hand_action("open", arm)
        _log(f"[recognition_app] Opened {arm} hand before grab.")
        return True
    if _CONTROL_ROBOT is None:
        raise RuntimeError(_CONTROL_STATUS)
    _log("[recognition_app] FK hand pose is not visible in camera; extending arm before grab.")
    if not ARM_CONTROL_LOCK.acquire(blocking=False):
        _log("[recognition_app] Waiting for current arm motion before pre-grab extend.")
        ARM_CONTROL_LOCK.acquire()
    try:
        if ARM_CANCEL_EVENT.is_set():
            _log("[recognition_app] Pre-grab extend cancelled before control handoff.")
            return False
        ARM_CANCEL_EVENT.clear()
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "pre-grab extend"
        result = _run_extend_arm_ee_ik(
            _CONTROL_ROBOT,
            arm,
            max_joint_step_rad,
            max_joint_speed_rad_s,
            cam,
        )
        if not result.get("success"):
            _log(f"[recognition_app] Pre-grab extend failed: {result}")
            return False
        if result.get("final_q") is not None:
            _save_extend_arm_reference(arm, result["final_q"], "pre-grab extend")
        _send_hand_action("open", arm)
        _log(f"[recognition_app] Opened {arm} hand after pre-grab extend.")
        return True
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CONTROL_LOCK.release()


def _lift_end_effector_z_slowly(
    robot,
    arm: str,
    max_joint_step_rad: float,
    max_joint_speed_rad_s: float,
    lift_m: float = DEFAULT_POST_GRASP_LIFT_M,
    mass_kg: float = 0.0,
) -> Dict[str, Any]:
    if lift_m <= 0.0:
        return {"success": True, "reason": "lift_disabled"}
    kp, kd, duration_s, speed_scale = _lift_tuning_for_mass(mass_kg)
    q_cur = _read_current_arm_q(robot, arm)
    fk = ArmFK(arm=arm, backend="urdf")
    T_start = fk.compute_arm(q_cur)
    T_lift = T_start.copy()
    T_lift[2, 3] += float(lift_m)
    ik = ArmIK(
        arm=arm,
        solver="dls",
        tol_pos_m=0.010,
        tol_rot_rad=0.20,
    )
    q_solution, info = ik.solve(T_lift, q_init=q_cur)
    if q_solution is None:
        return {"success": False, "reason": "ik_failed", "ik_info": info}
    executor = ArmExecutor(
        robot,
        arm=arm,
        kp=kp,
        kd=kd,
        max_reach_m=DEFAULT_DIRECT_MAX_REACH_M,
        max_joint_step_rad=max_joint_step_rad,
        max_joint_speed_rad_s=max_joint_speed_rad_s * speed_scale,
    )
    result = executor.execute(
        q_solution,
        duration_s=duration_s,
        q_arm_start=q_cur,
        T_base_desired=T_lift,
        stop_event=ARM_CANCEL_EVENT,
    )
    result["ik_info"] = info
    result["lift_m"] = float(lift_m)
    result["mass_kg"] = float(mass_kg)
    return result


def _lift_end_effector_z_with_continuous_executor(
    robot,
    arm: str,
    executor: ArmExecutor,
    lift_m: float = DEFAULT_POST_GRASP_LIFT_M,
    mass_kg: float = 0.0,
) -> Dict[str, Any]:
    if lift_m <= 0.0:
        return {"success": True, "reason": "lift_disabled"}
    kp, kd, duration_s, speed_scale = _lift_tuning_for_mass(mass_kg)
    # ArmExecutor reads kp/kd/max_joint_speed_rad_s live at publish time (see
    # ArmExecutor._send_command), so bumping them here stiffens/slows this
    # lift without needing a new executor instance.
    executor.kp = kp
    executor.kd = kd
    executor.max_joint_speed_rad_s = executor.max_joint_speed_rad_s * speed_scale
    q_cur = _read_current_arm_q(robot, arm)
    fk = ArmFK(arm=arm, backend="urdf")
    T_start = fk.compute_arm(q_cur)
    T_lift = T_start.copy()
    T_lift[2, 3] += float(lift_m)
    ik = ArmIK(
        arm=arm,
        solver="dls",
        tol_pos_m=0.010,
        tol_rot_rad=0.20,
    )
    q_solution, info = ik.solve(T_lift, q_init=q_cur)
    if q_solution is None:
        return {"success": False, "reason": "ik_failed", "ik_info": info}
    validation = executor.validate(
        q_solution,
        T_base_desired=T_lift,
        q_arm_start=q_cur,
    )
    if not validation.get("safe"):
        return {
            "success": False,
            "reason": validation.get("reason"),
            "violations": validation.get("violations"),
            "ik_info": info,
        }
    executor.submit_target(q_solution)
    t0 = time.time()
    while time.time() - t0 < duration_s:
        if ARM_CANCEL_EVENT.is_set():
            return {
                "success": False,
                "reason": "stopped",
                "duration_s": time.time() - t0,
                "ik_info": info,
                "lift_m": float(lift_m),
                "mass_kg": float(mass_kg),
            }
        time.sleep(0.05)
    return {
        "success": True,
        "controller": "continuous_executor",
        "duration_s": time.time() - t0,
        "ik_info": info,
        "lift_m": float(lift_m),
        "mass_kg": float(mass_kg),
    }


def _run_direct_grab_inline(
    det: Detection,
    arm: str,
    cam: Dict[str, float],
    auto_step_base: bool,
    T_base_object: np.ndarray,
    max_joint_step_rad: float,
    max_joint_speed_rad_s: float,
    mass_kg: float = 0.0,
) -> None:
    global ACTIVE_DIRECT_NAV
    if _CONTROL_ROBOT is None:
        raise RuntimeError(_CONTROL_STATUS)

    if not ARM_CONTROL_LOCK.acquire(blocking=False):
        _log("[recognition_app] Waiting for current arm motion to finish before grab control.")
        ARM_CONTROL_LOCK.acquire()
    try:
        if ARM_CANCEL_EVENT.is_set():
            _log("[recognition_app] Direct grab cancelled before control handoff.")
            return
        ARM_CANCEL_EVENT.clear()
        with STATE.lock:
            STATE.arm_motion_running = True
            STATE.arm_motion_label = "direct grab"
            fixed_R_base = (
                None
                if arm not in STATE.extended_hand_R_base
                else STATE.extended_hand_R_base[arm].copy()
            )

        obstacles = _estimate_obstacles(cam)

        _arm, reach_dist, max_reach, excess_m, suggested_step_m = _reach_preview(
            det, cam, arm, obstacles,
        )
        T_base_camera = _make_transform(
            xyz=(cam["x"], cam["y"], cam["z"]),
            rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
        )
        base_step_m = 0.0
        if excess_m > 0.0:
            _log(
                "[recognition_app] target outside arm workspace: "
                f"reach_dist={reach_dist:.3f} m max={max_reach:.3f} m "
                f"excess={excess_m:.3f} m"
            )
            if not auto_step_base:
                _log("[recognition_app] Auto-step disabled; move closer or enable base step.")
                return
            base_step_m = suggested_step_m
            if base_step_m <= 0.0:
                _log("[recognition_app] Auto-step requested, but allowed step distance is zero.")
                return
            speed_m_s = 1.5
            duration_s = base_step_m / speed_m_s
            _log(
                f"[recognition_app] stepping base forward {base_step_m:.3f} m "
                f"at {speed_m_s:.3f} m/s for {duration_s:.2f} s"
            )
            _CONTROL_ROBOT.move_for(duration=duration_s, vx=speed_m_s, vy=0.0, vyaw=0.0)
            T_base_object = T_base_object.copy()
            T_base_object[0, 3] -= base_step_m

        T_base_direct_hand = _direct_grasp_hand_pose(
            det,
            T_base_object,
            arm,
            obstacles,
            fixed_R_base=fixed_R_base,
        )
        grasp_standoff_m = _grasp_standoff_for_det(det)
        T_base_direct_target = _direct_nav_target_frame_from_hand_pose(
            T_base_direct_hand,
            arm,
            grasp_standoff_m,
        )
        T_camera_direct_target = np.linalg.inv(T_base_camera) @ T_base_direct_target

        fixed_result = DetectionResult(
            T_camera_object=T_camera_direct_target,
            confidence=float(det.score),
            method="fixed",
        )
        config = {
            "arm": arm,
            "detection_method": "fixed",
            # Must match the standoff used above to build T_base_direct_target:
            # DirectHandPoseNav re-applies GraspPlanner(standoff_m=...) to this
            # "fixed" target internally (see direct_nav.py), so a mismatch here
            # would double-apply (or under-apply) the surface-aware standoff.
            "standoff_m": grasp_standoff_m,
            "rate_hz": DEFAULT_SLOW_GRAB_RATE_HZ,
            "timeout_s": DEFAULT_SLOW_GRAB_TIMEOUT_S,
            "ik_solver": "dls",
            "iface": _ARGS.iface,
            "domain_id": _ARGS.domain_id,
            "mock": _ARGS.mock,
            "camera_x": float(cam.get("x", 0.0)),
            "camera_y": float(cam.get("y", 0.0)),
            "camera_z": float(cam.get("z", 0.0)),
            "camera_roll": float(cam.get("roll", 0.0)),
            "camera_pitch": float(cam.get("pitch", 0.0)),
            "camera_yaw": float(cam.get("yaw", 0.0)),
            "ik_tol_pos_m": 0.10 if det.source == "vision" else 0.003,
            "ik_tol_rot_rad": 3.14 if det.source == "vision" else 0.01,
            "convergence_pos_m": DEFAULT_VISION_GRAB_CONVERGENCE_POS_M if det.source == "vision" else 0.015,
            "convergence_rot_rad": 3.14 if det.source == "vision" else 0.05,
            "max_joint_step_rad": max_joint_step_rad,
            "max_joint_speed_rad_s": max_joint_speed_rad_s,
            "max_reach_m": DEFAULT_DIRECT_MAX_REACH_M,
        }
        _log(
            "[recognition_app] object_base_xyz="
            f"({T_base_object[0, 3]:+.3f}, {T_base_object[1, 3]:+.3f}, {T_base_object[2, 3]:+.3f}) m "
            f"base_step={base_step_m:.3f} m"
        )
        if det.source == "vision":
            p_hand = T_base_direct_hand[:3, 3]
            _log(
                "[recognition_app] vision desired hand xyz="
                f"({p_hand[0]:+.3f}, {p_hand[1]:+.3f}, {p_hand[2]:+.3f}) m "
                "with stable robot-side approach"
            )
        if fixed_R_base is not None:
            _log(
                "[recognition_app] direct hand orientation fixed from last "
                f"extend arm ({arm}); moving XYZ only."
            )
        _log(
            "[recognition_app] using fixed selected direct target for this grab; "
            "camera detections will not retarget the arm mid-grab."
        )
        _log(
            "[recognition_app] slow grab config: "
            f"rate={config['rate_hz']:.1f}Hz "
            f"max_joint_step={config['max_joint_step_rad']:.3f}rad "
            f"max_joint_speed={config['max_joint_speed_rad_s']:.3f}rad/s "
            f"timeout={config['timeout_s']:.1f}s"
        )
        try:
            _ensure_arm_authority(_CONTROL_ROBOT, duration_s=0.5)
        except Exception as exc:
            _log(f"[recognition_app] unrelease_arms() before grab failed: {exc}")
        nav = DirectHandPoseNav(
            config, fixed_result=fixed_result, robot=_CONTROL_ROBOT, obstacles=obstacles,
        )
        with ACTIVE_NAV_LOCK:
            ACTIVE_DIRECT_NAV = nav
        ok = False
        stalled = False
        last_nav_status: Dict[str, Any] = {}
        last_status_log_t = 0.0
        nav_holding = False
        try:
            deadline = time.time() + float(config["timeout_s"]) + 2.0
            while time.time() < deadline:
                if ARM_CANCEL_EVENT.is_set():
                    _log("[recognition_app] Direct grab cancelled by safety command.")
                    break
                status = nav.status_snapshot()
                last_nav_status = dict(status)
                now = time.time()
                if now - last_status_log_t >= 1.0:
                    last_status_log_t = now
                    loop_log = status.get("log") or []
                    last_loop_msg = loop_log[-1] if loop_log else ""
                    _log(
                        "[recognition_app] direct nav status: "
                        f"running={status.get('running')} "
                        f"iter={status.get('iteration')} "
                        f"converged={status.get('converged')} "
                        f"pos_err={status.get('last_error_pos_m')} "
                        f"ik_failures={status.get('ik_failures')} "
                        f"safety_rejections={status.get('safety_rejections')} "
                        f"detection_failures={status.get('detection_failures')} "
                        f"last={last_loop_msg}"
                    )
                if status.get("converged"):
                    ok = True
                    break
                if status.get("stalled"):
                    stalled = True
                    _log(
                        "[recognition_app] Direct grab stalled: target has not "
                        "gotten closer over the last several EE-pose increments "
                        "(object likely moved) — checking whether EE is close enough to grasp."
                    )
                    break
                if not status.get("running", True):
                    _log(f"[recognition_app] direct nav stopped: {status}")
                    break
                time.sleep(0.2)
        finally:
            nav_holding = bool(ok or stalled)
            if not nav_holding:
                _shutdown_direct_nav(nav, "direct grab")

        if stalled:
            pos_err = None
            try:
                raw_pos_err = last_nav_status.get("last_error_pos_m")
                pos_err = None if raw_pos_err is None else float(raw_pos_err)
            except Exception:
                pos_err = None
            close_on_stall_m = (
                DEFAULT_VISION_CLOSE_ON_STALL_POS_M
                if det.source == "vision"
                else max(0.030, float(config["convergence_pos_m"]))
            )
            if pos_err is not None and pos_err <= close_on_stall_m:
                _log(
                    "[recognition_app] Direct grab stalled near target "
                    f"(pos_err={pos_err:.4f}m <= {close_on_stall_m:.4f}m); "
                    "closing hand instead of resetting."
                )
                ok = True
            else:
                _log(
                    "[recognition_app] Direct grab stalled away from target "
                    f"(pos_err={pos_err}); resetting to saved extend-arm reference."
                )
        if stalled and not ok:
            if ARM_CANCEL_EVENT.is_set():
                _log("[recognition_app] Stall reset skipped: grab was cancelled.")
                if nav_holding:
                    _shutdown_direct_nav(nav, "direct grab")
                return
            if nav_holding:
                _shutdown_direct_nav(nav, "direct grab before stall reset")
                nav_holding = False
            try:
                reset_result = _run_reset_to_extend_arm_reference(
                    _CONTROL_ROBOT,
                    arm,
                    max_joint_step_rad,
                    max_joint_speed_rad_s,
                )
                _log(f"[recognition_app] stall reset to saved extend-arm reference result: {reset_result}")
            except Exception as exc:
                _log(f"[recognition_app] stall reset to saved extend-arm reference failed: {exc}")
            return

        if not ok:
            _log("[recognition_app] Direct grab did not converge; not closing hand.")
            if nav_holding:
                _shutdown_direct_nav(nav, "direct grab")
            return
        _log("[recognition_app] Direct grab converged.")
        try:
            closed_hand = False
            if hasattr(_CONTROL_ROBOT, "hand_close"):
                _log("[recognition_app] Holding EE pose while closing hand (tactile-guided).")
                grip_result = _tactile_guided_hand_close(arm, mass_kg)
                closed_hand = bool(grip_result.get("success"))
                _log(f"[recognition_app] Closed {arm} hand: {grip_result}")
            if not closed_hand:
                _log("[recognition_app] Hand close unavailable; skipping post-grasp lift.")
                if nav_holding:
                    _shutdown_direct_nav(nav, "direct grab")
                return
        except Exception as exc:
            _log(f"[recognition_app] Hand close failed: {exc}")
            if nav_holding:
                _shutdown_direct_nav(nav, "direct grab")
            return
        if ARM_CANCEL_EVENT.is_set():
            _log("[recognition_app] Post-grasp lift skipped because grab was cancelled.")
            if nav_holding:
                _shutdown_direct_nav(nav, "direct grab")
            return
        try:
            with STATE.lock:
                STATE.arm_motion_label = "post-grasp lift"
            _log(
                "[recognition_app] Raising end effector in base Z by "
                f"{DEFAULT_POST_GRASP_LIFT_M:.3f} m."
            )
            hold_executor = getattr(nav, "_executor_obj", None) if nav_holding else None
            if hold_executor is not None:
                _log("[recognition_app] Lifting with the continuous hold executor.")
                lift_result = _lift_end_effector_z_with_continuous_executor(
                    _CONTROL_ROBOT,
                    arm,
                    hold_executor,
                    mass_kg=mass_kg,
                )
            else:
                lift_result = _lift_end_effector_z_slowly(
                    _CONTROL_ROBOT,
                    arm,
                    max_joint_step_rad,
                    max_joint_speed_rad_s,
                    mass_kg=mass_kg,
                )
            _log(f"[recognition_app] post-grasp lift result: {lift_result}")
        except Exception as exc:
            _log(f"[recognition_app] post-grasp lift failed: {exc}")
        finally:
            if nav_holding:
                _shutdown_direct_nav(nav, "direct grab after post-grasp lift")
    finally:
        with STATE.lock:
            STATE.arm_motion_running = False
            STATE.arm_motion_label = ""
        ARM_CONTROL_LOCK.release()


def _run_grab_impl() -> None:
    with STATE.lock:
        selected_id = STATE.selected_id
        live_det = STATE.detections.get(selected_id) if selected_id else None
        det = _copy_detection(live_det) if live_det is not None else None
        detection_age_s = time.time() - STATE.last_detection_ts if STATE.last_detection_ts else float("inf")
        cam = dict(STATE.camera_extrinsic)
        arm_override = STATE.arm_override
        auto_step_base = STATE.auto_step_base
        max_joint_step_rad = STATE.max_joint_step_rad
        max_joint_speed_rad_s = STATE.max_joint_speed_rad_s
        planning_mode = STATE.planning_mode
        mass_kg = STATE.object_mass_kg
        STATE.grab_running = True

    if det is None or detection_age_s > STALE_AFTER_S:
        if det is None:
            _log("[recognition_app] No object selected — aborting grab.")
        else:
            _log(
                "[recognition_app] Selected detection is stale "
                f"({detection_age_s:.1f}s old) — aborting grab."
            )
        with STATE.lock:
            STATE.grab_running = False
        return

    det = _center_object_with_waist_yaw(
        det,
        selected_id,
        max_joint_speed_rad_s,
    )
    with STATE.lock:
        cam = dict(STATE.camera_extrinsic)
    arm, _T_base_camera, T_base_object = _resolve_arm_and_base_pose(
        det, cam, arm_override,
    )
    _arm, reach_dist, max_reach, excess_m, suggested_step_m = _reach_preview(
        det, cam, arm_override,
    )
    if excess_m > 0.0 and not auto_step_base:
        _log(
            "[recognition_app] Selected target is outside arm-only reach: "
            f"reach_dist={reach_dist:.3f} m max={max_reach:.3f} m "
            f"excess={excess_m:.3f} m. Enable 'Step base closer' "
            f"or move the object/robot about {suggested_step_m:.3f} m closer."
        )

    p_cam = det.T_camera_object[:3, 3]
    p_base = T_base_object[:3, 3]
    _log(
        "[recognition_app] Grab target coordinate snapshot: "
        f"id={det.id!r} label={det.label!r} source={det.source} "
        f"confidence={det.score:.2f}"
    )
    _log(
        f"[recognition_app] arm={arm} label={det.label!r} "
        f"auto_step_base={auto_step_base} planning_mode={planning_mode} "
        f"object_mass={mass_kg:.2f}kg"
    )
    _log(
        "[recognition_app] object camera xyz="
        f"({p_cam[0]:+.3f}, {p_cam[1]:+.3f}, {p_cam[2]:+.3f}) m  "
        "base xyz="
        f"({p_base[0]:+.3f}, {p_base[1]:+.3f}, {p_base[2]:+.3f}) m"
    )

    if planning_mode == "vla":
        obstacles = _estimate_obstacles(cam)
        try:
            q_cur = _read_current_arm_q(_CONTROL_ROBOT, arm) if _CONTROL_ROBOT is not None else None
        except Exception:
            q_cur = None
        with STATE.lock:
            rgb = None if STATE.rgb_bgr is None else STATE.rgb_bgr.copy()
            depth = None if STATE.depth_m is None else STATE.depth_m.copy()
        plan = DEFAULT_VLA_POLICY.plan(
            rgb_bgr=rgb, depth_m=depth, instruction=det.label,
            q_arm_current=q_cur, arm=arm, obstacles=obstacles,
        )
        _log(f"[recognition_app] VLA plan: success={plan.success} reason={plan.reason}")
        with STATE.lock:
            STATE.status_msg = f"VLA grab not executed: {plan.reason}"
            STATE.grab_running = False
        return

    try:
        prepared = _prepare_hand_visibility_for_grab(
            arm,
            cam,
            max_joint_step_rad,
            max_joint_speed_rad_s,
        )
        if not prepared:
            _log("[recognition_app] Grab aborted during hand visibility preflight.")
            return
        _run_direct_grab_inline(
            det,
            arm,
            cam,
            auto_step_base,
            T_base_object,
            max_joint_step_rad,
            max_joint_speed_rad_s,
            mass_kg=mass_kg,
        )
    finally:
        with STATE.lock:
            STATE.grab_running = False


def _run_grab() -> None:
    try:
        _run_grab_impl()
    except Exception as exc:
        _log(f"[recognition_app] Grab worker failed before launch: {exc!r}")
        with STATE.lock:
            STATE.grab_running = False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="G1 recognition layer web app.")
    p.add_argument("--iface", default="eth0")
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--rgbd-host", default="10.34.0.47")
    p.add_argument("--rgbd-port", type=int, default=5555)
    p.add_argument("--rgbd-topic", default="")
    p.add_argument(
        "--sdk-rgbd",
        action="store_true",
        help="Use sdk_client.Robot.get_rgbd() instead of the persistent ZMQ receiver.",
    )
    p.add_argument("--rate-hz", type=float, default=3.0, help="perception loop rate")
    p.add_argument("--camera-rate-hz", type=float, default=15.0, help="raw RGB-D capture loop rate")
    p.add_argument("--vision-model", default="yolov8s-world.pt")
    p.add_argument("--mock", action="store_true", help="use a synthetic camera feed")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8060)
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


_ARGS = _parse_args()

_DDS_STATUS = "Unitree DDS skipped in mock mode"
if not _ARGS.mock:
    _DDS_STATUS = _init_unitree_dds_once(_ARGS.iface, _ARGS.domain_id)
with STATE.lock:
    STATE.hand_fk_status = _DDS_STATUS

if _ARGS.mock:
    _ROBOT = _MockRobot()
elif _ARGS.sdk_rgbd and _ROBOT_AVAILABLE:
    _ROBOT = Robot(
        iface=_ARGS.iface,
        domain_id=_ARGS.domain_id,
        auto_start_sensors=True,
        rgbd_host=_ARGS.rgbd_host,
        rgbd_port=_ARGS.rgbd_port,
    )
else:
    _ROBOT = _ZmqRgbdRobot(
        host=_ARGS.rgbd_host,
        port=_ARGS.rgbd_port,
        topic=_ARGS.rgbd_topic,
    )

_CONTROL_ROBOT = _ROBOT if (Robot is not None and isinstance(_ROBOT, Robot)) else None
_CONTROL_STATUS = "control robot unavailable"
if _ARGS.mock:
    _CONTROL_ROBOT = _ROBOT
    _CONTROL_STATUS = "mock control robot"
elif _DDS_STATUS.startswith("Unitree DDS initialized"):
    if _CONTROL_ROBOT is None:
        _CONTROL_ROBOT, _CONTROL_STATUS = _make_control_robot(_ARGS.iface, _ARGS.domain_id)
    else:
        _CONTROL_STATUS = "control robot ready"
else:
    _CONTROL_STATUS = _DDS_STATUS
with STATE.lock:
    if _CONTROL_ROBOT is None:
        STATE.status_msg = _CONTROL_STATUS

_VISION = VisionDetector(model_name=_ARGS.vision_model)
with STATE.lock:
    STATE.vision_classes = _VISION.set_prompt(DEFAULT_DETECTION_PROMPT)

threading.Thread(
    target=_camera_loop, args=(_ROBOT, _ARGS.camera_rate_hz),
    daemon=True,
).start()

threading.Thread(
    target=_perception_loop, args=(_VISION, _ARGS.rate_hz),
    daemon=True,
).start()

if not _ARGS.mock and _DDS_STATUS.startswith("Unitree DDS initialized"):
    threading.Thread(
        target=_hand_fk_loop, args=(_ARGS.iface, _ARGS.domain_id),
        daemon=True,
    ).start()

if __name__ == "__main__":
    app.run(host=_ARGS.host, port=_ARGS.port, debug=_ARGS.debug)
