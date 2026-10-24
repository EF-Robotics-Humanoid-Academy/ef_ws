"""
Non-ROS direct runtime for hand_pose_navigation.

This keeps the perception, IK, reachability, execution, tracking loop, and
web monitor usable when ROS 2 DDS cannot coexist with the Unitree SDK DDS in
one Python process. TF is replaced by a local static T_base_camera transform.
"""
from __future__ import annotations
from .tracking_loop import TrackingLoop
from .target_detector import DetectionResult, TargetDetector
from .reachability_checker import ReachabilityChecker
from .grasp_planner import GraspPlanner
from .arm_ik import ArmIK
from .arm_fk import ArmFK
from .arm_executor import ArmExecutor
from .obstacle_checker import Obstacles

import math
import os
import sys
from typing import Dict, Optional, Tuple

import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
_MODULES = os.path.join(_DIR, "..", "modules")
if _MODULES not in sys.path:
    sys.path.insert(0, _MODULES)

try:
    from sdk_client import Robot
    _ROBOT_AVAILABLE = True
except ImportError:
    Robot = None
    _ROBOT_AVAILABLE = False


class DirectPosePublisher:
    def __init__(self) -> None:
        self.last_result: Optional[DetectionResult] = None

    def update(self, result: DetectionResult) -> None:
        self.last_result = result


class DirectTFUtils:
    def __init__(
        self,
        pose_publisher: DirectPosePublisher,
        T_base_camera: np.ndarray,
    ) -> None:
        self._pose_publisher = pose_publisher
        self._T_base_camera = T_base_camera

    def base_to_target(
        self,
        timeout_s: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        result = self._pose_publisher.last_result
        if result is None:
            return None
        return self._T_base_camera @ result.T_camera_object


class DirectHandPoseNav:
    def __init__(
        self,
        config: Dict,
        fixed_result: Optional[DetectionResult] = None,
        robot=None,
        obstacles: Optional[Obstacles] = None,
    ) -> None:
        self._config = dict(config)
        arm = config.get("arm", "right")
        detection_method = config.get("detection_method", "aruco")

        self._pose_pub = DirectPosePublisher()
        self._detector = TargetDetector(
            method=detection_method,
            aruco_id=config.get("aruco_id", 0),
            marker_size_m=config.get("marker_size_m", 0.05),
            fixed_result=fixed_result,
        )
        self._fk = ArmFK(arm=arm)
        self._tf_utils = DirectTFUtils(
            self._pose_pub,
            _make_transform(
                xyz=(
                    config.get("camera_x", 0.0),
                    config.get("camera_y", 0.0),
                    config.get("camera_z", 0.0),
                ),
                rpy=(
                    config.get("camera_roll", 0.0),
                    config.get("camera_pitch", 0.0),
                    config.get("camera_yaw", 0.0),
                ),
            ),
        )
        self._grasp_planner = GraspPlanner(
            arm=arm,
            standoff_m=config.get("standoff_m", 0.08),
        )
        self._ik = ArmIK(
            arm=arm,
            solver=config.get("ik_solver", "dls"),
            tol_pos_m=config.get("ik_tol_pos_m", 0.003),
            tol_rot_rad=config.get("ik_tol_rot_rad", 0.01),
        )
        max_reach_m = config.get("max_reach_m", 0.42)
        self._checker = ReachabilityChecker(arm=arm, max_reach_m=max_reach_m)

        self._robot_mode = "mock"
        self._sdk_error = ""
        if robot is not None:
            self._robot = robot
            self._robot_mode = "sdk"
        elif config.get("mock", False) or not _ROBOT_AVAILABLE:
            self._robot = _MockRobot()
            if not _ROBOT_AVAILABLE and not config.get("mock", False):
                self._sdk_error = "sdk_client import failed"
        else:
            try:
                self._robot = Robot(
                    iface=config.get("iface", "eth0"),
                    domain_id=config.get("domain_id", 0),
                    auto_start_sensors=True,
                    rgbd_host=config.get("rgbd_host", "192.168.2.41"),
                    rgbd_port=config.get("rgbd_port", 5555),
                )
                self._robot_mode = "sdk"
            except Exception as exc:
                self._robot = _MockRobot()
                self._sdk_error = repr(exc)

        self._executor_obj = ArmExecutor(
            self._robot,
            arm=arm,
            max_reach_m=max_reach_m,
            max_joint_step_rad=config.get("max_joint_step_rad", 0.08),
            max_joint_speed_rad_s=config.get("max_joint_speed_rad_s", 0.15),
        )
        control_rate_hz = float(config.get("rate_hz", 10.0))
        max_joint_step_rad = float(config.get("max_joint_step_rad", 0.08))
        max_joint_speed_rad_s = float(config.get("max_joint_speed_rad_s", 0.15))
        if control_rate_hz > 0.0 and max_joint_speed_rad_s > 0.0:
            max_joint_step_rad = min(max_joint_step_rad, max_joint_speed_rad_s / control_rate_hz)
        self._tracking_loop = TrackingLoop(
            robot=self._robot,
            detector=self._detector,
            pose_publisher=self._pose_pub,
            tf_utils=self._tf_utils,
            fk=self._fk,
            grasp_planner=self._grasp_planner,
            ik=self._ik,
            checker=self._checker,
            executor=self._executor_obj,
            arm=arm,
            rate_hz=control_rate_hz,
            convergence_pos_m=config.get("convergence_pos_m", 0.015),
            convergence_rot_rad=config.get("convergence_rot_rad", 0.05),
            max_joint_step_rad=max_joint_step_rad,
            timeout_s=config.get("timeout_s", 30.0),
            static_obstacles=obstacles,
        )
        self._tracking_loop.start(blocking=False)

    def update_fixed_target(self, T_camera_target: np.ndarray, confidence: float = 1.0) -> None:
        """Replace the "fixed" target while the tracking loop is running.

        The loop reads it on its next iteration and re-solves IK toward it
        from the measured joints, so the approach bends toward the corrected
        target instead of restarting.
        """
        self._detector.set_fixed_result(DetectionResult(
            T_camera_object=np.asarray(T_camera_target, dtype=np.float64).copy(),
            confidence=float(confidence),
            method="fixed",
        ))
        self._tracking_loop.notify_target_updated()

    def shutdown(self) -> None:
        self._tracking_loop.stop()

    def status_snapshot(self) -> Dict:
        status = self._tracking_loop.status.to_dict()
        status.update(
            {
                "node": "direct_hand_pose_nav",
                "robot_mode": self._robot_mode,
                "sdk_error": self._sdk_error,
                "config": dict(self._config),
            }
        )
        return status


class _MockRobot:
    def get_rgbd(self, timeout=2.0):
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 0.8, dtype=np.float32)
        return {"rgb_bgr": rgb, "depth_m": depth}

    def get_joint_states(self):
        return {
            "joints": {
                f"j{i}": {"index": i, "position": 0.0}
                for i in range(30)
            }
        }

    def move_upper_body_joint(self, **kw):
        pass


def _make_transform(
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float],
) -> np.ndarray:
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
