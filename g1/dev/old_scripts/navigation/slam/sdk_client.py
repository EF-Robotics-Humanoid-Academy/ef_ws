"""
sdk_client.py
=============

SDK-native Robot wrapper for Unitree G1.

The implementation is intentionally local to `modules/` and avoids imports
from `../scripts`. Script-backed workflows were replaced with direct SDK
helpers or removed from the core wrapper.
"""
from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from dds_env import ensure_cyclonedds_environment
from sdk_audio import RobotAudio
from sdk_boot import create_loco_client, rpc_get_int
from sdk_hand import Dex3HandController
from secure_boot import force_normal_gait, secure_boot
from sdk_sensors import (
    LatestSubscriber,
    LidarImu_,
    LowStateSnapshot,
    Odometry_,
    decode_video_frame_bgr,
    load_video_client_type,
    lowstate_snapshot_from_msg,
    odom_pose_from_msg,
    resolve_lowstate_type,
)
from sdk_slam import SlamInfoSubscriber, SlamOdomSubscriber, SlamOperateClient, SlamResponse

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_, SportModeState_
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
    )
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


DEFAULT_SPORT_TOPIC = "rt/odommodestate"
DEFAULT_LIDAR_MAP_TOPIC = "rt/utlidar/map_state"
DEFAULT_LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_deskewed"
DEFAULT_LIDAR_CLOUD_FALLBACK_TOPIC = "rt/utlidar/cloud_livox_mid360"
DEFAULT_RGBD_HOST = os.environ.get("G1_RGBD_HOST", "10.34.0.83")
DEFAULT_RGBD_PORT = int(os.environ.get("G1_RGBD_PORT", "5555"))
DEFAULT_RGBD_TOPIC = os.environ.get("G1_RGBD_TOPIC", "")
HAND_STATE_TOPIC_BY_SIDE = {
    "left": "rt/dex3/left/state",
    "right": "rt/dex3/right/state",
}
HAND_JOINT_NAMES = [
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "middle_0",
    "middle_1",
    "index_0",
    "index_1",
]
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
WAIST_JOINTS = [12, 13, 14]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_SDK_NOT_USED_IDX = 29
WAIST_HOLD_KP = 240.0
WAIST_HOLD_KD = 12.0
HL_ARM_ACTION_RELEASE = "release arm"
HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S = 2.0
HL_ARM_ACTIONS = {
    "release arm": 99,
    "two-hand kiss": 11,
    "left kiss": 12,
    "right kiss": 13,
    "hands up": 15,
    "clap": 17,
    "high five": 18,
    "hug": 19,
    "heart": 20,
    "right heart": 21,
    "reject": 22,
    "right hand up": 23,
    "x-ray": 24,
    "face wave": 25,
    "high wave": 26,
    "shake hand": 27,
}
HL_ARM_ACTION_ALIASES = {
    "release": "release arm",
    "two hand kiss": "two-hand kiss",
    "lefthand kiss": "left kiss",
    "left hand kiss": "left kiss",
    "righthand kiss": "right kiss",
    "right hand kiss": "right kiss",
    "xray": "x-ray",
    "x ray": "x-ray",
}
BODY_JOINT_LAYOUT: list[tuple[str, int, str]] = [
    ("left_leg", 0, "hip_pitch"),
    ("left_leg", 1, "hip_roll"),
    ("left_leg", 2, "hip_yaw"),
    ("left_leg", 3, "knee"),
    ("left_leg", 4, "ankle_pitch"),
    ("left_leg", 5, "ankle_roll"),
    ("right_leg", 6, "hip_pitch"),
    ("right_leg", 7, "hip_roll"),
    ("right_leg", 8, "hip_yaw"),
    ("right_leg", 9, "knee"),
    ("right_leg", 10, "ankle_pitch"),
    ("right_leg", 11, "ankle_roll"),
    ("waist", 12, "yaw"),
    ("waist", 13, "roll"),
    ("waist", 14, "pitch"),
    ("left_arm", 15, "shoulder_pitch"),
    ("left_arm", 16, "shoulder_roll"),
    ("left_arm", 17, "shoulder_yaw"),
    ("left_arm", 18, "elbow"),
    ("left_arm", 19, "wrist_roll"),
    ("left_arm", 20, "wrist_pitch"),
    ("left_arm", 21, "wrist_yaw"),
    ("right_arm", 22, "shoulder_pitch"),
    ("right_arm", 23, "shoulder_roll"),
    ("right_arm", 24, "shoulder_yaw"),
    ("right_arm", 25, "elbow"),
    ("right_arm", 26, "wrist_roll"),
    ("right_arm", 27, "wrist_pitch"),
    ("right_arm", 28, "wrist_yaw"),
]
BODY_JOINT_NAME_BY_INDEX = {
    index: f"{group}.{name}" for group, index, name in BODY_JOINT_LAYOUT
}
BODY_JOINT_INDEX_BY_NAME = {
    f"{group}.{name}": index for group, index, name in BODY_JOINT_LAYOUT
}

try:
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
except Exception:
    HandState_ = None  # type: ignore[assignment]


@dataclass
class ImuData:
    rpy: tuple[float, float, float]
    gyro: tuple[float, float, float] | None
    acc: tuple[float, float, float] | None
    quat: tuple[float, float, float, float] | None
    temp: float | None


class _ArmSdkPublisher:
    def __init__(self, iface: str, domain_id: int) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        # Use the SDK Python CRC when the native architecture library is missing.
        if not getattr(CRC, "_wbc_python_crc_fallback", False):
            _crc_sdk_init = CRC.__init__
            def _crc_init_python_fallback(self):
                try:
                    _crc_sdk_init(self)
                except OSError as exc:
                    if "crc_" not in str(exc) or "No such file" not in str(exc):
                        raise
                    self.platform = "Python"
            CRC.__init__ = _crc_init_python_fallback
            CRC._wbc_python_crc_fallback = True


        ChannelFactoryInitialize(int(domain_id), str(iface))
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_NOT_USED_IDX].q = 1.0

    def publish_targets(
        self,
        joint_targets: dict[int, float],
        *,
        kp: float = 30.0,
        kd: float = 1.5,
        kp_by_joint: dict[int, float] | None = None,
        kd_by_joint: dict[int, float] | None = None,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        for joint_index, target in joint_targets.items():
            idx = int(joint_index)
            mc = self._cmd.motor_cmd[int(joint_index)]
            mc.mode = 1
            mc.q = float(target)
            mc.dq = float(dq)
            mc.tau = float(tau)
            mc.kp = float(kp_by_joint.get(idx, kp) if kp_by_joint is not None else kp)
            mc.kd = float(kd_by_joint.get(idx, kd) if kd_by_joint is not None else kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def publish_arm_sdk_weight(self, weight: float) -> None:
        self._cmd.motor_cmd[ARM_SDK_NOT_USED_IDX].q = max(0.0, min(1.0, float(weight)))
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class Robot:
    """End-user wrapper around common G1 SDK workflows."""

    def __init__(
        self,
        iface: str = "eth0",
        domain_id: int = 0,
        safety_boot: bool = False,
        auto_start_sensors: bool = True,
        sport_topic: str = DEFAULT_SPORT_TOPIC,
        lidar_map_topic: str = DEFAULT_LIDAR_MAP_TOPIC,
        lidar_cloud_topic: str = DEFAULT_LIDAR_CLOUD_TOPIC,
        slam_info_topic: str = "rt/slam_info",
        slam_key_topic: str = "rt/slam_key_info",
        rgbd_host: str = DEFAULT_RGBD_HOST,
        rgbd_port: int = DEFAULT_RGBD_PORT,
        rgbd_topic: str = DEFAULT_RGBD_TOPIC,
    ) -> None:
        self.iface = iface
        self.domain_id = int(domain_id)
        self.sport_topic = sport_topic
        self.lidar_map_topic = lidar_map_topic
        self.lidar_cloud_topic = lidar_cloud_topic
        self.slam_info_topic = slam_info_topic
        self.slam_key_topic = slam_key_topic
        self.rgbd_host = str(rgbd_host)
        self.rgbd_port = int(rgbd_port)
        self.rgbd_topic = str(rgbd_topic)
        self.lidar_cloud_topics = list(
            dict.fromkeys(
                [
                    str(self.lidar_cloud_topic),
                    DEFAULT_LIDAR_CLOUD_FALLBACK_TOPIC,
                    DEFAULT_LIDAR_CLOUD_TOPIC,
                ]
            )
        )

        self._lock = threading.Lock()
        self._sport: SportModeState_ | None = None
        self._lidar_map: HeightMap_ | None = None
        self._lidar_cloud: PointCloud2_ | None = None
        self._lidar_cloud_by_topic: dict[str, PointCloud2_ | None] = {
            topic: None for topic in self.lidar_cloud_topics
        }
        self._last_sport_ts = 0.0
        self._last_lidar_map_ts = 0.0
        self._last_lidar_cloud_ts = 0.0
        self._last_lidar_cloud_ts_by_topic: dict[str, float] = {
            topic: 0.0 for topic in self.lidar_cloud_topics
        }

        self._sport_sub: ChannelSubscriber | None = None
        self._lidar_map_sub: ChannelSubscriber | None = None
        self._lidar_cloud_subs: dict[str, ChannelSubscriber] = {}
        self._lowstate_sub: LatestSubscriber | None = None
        self._odom_sub: LatestSubscriber | None = None
        self._lidar_imu_sub: LatestSubscriber | None = None
        self._slam_info_sub: SlamInfoSubscriber | None = None
        self._slam_odom_sub: SlamOdomSubscriber | None = None
        self._hand_state_subs: dict[str, LatestSubscriber] = {}

        self._path_points: list[tuple[float, float, float]] = []
        self._slam_client: SlamOperateClient | None = None
        self._audio: RobotAudio | None = None
        self._video_client: Any = None
        self._arm_sdk: _ArmSdkPublisher | None = None
        self._arm_action_client: Any = None
        self._hands: dict[str, Dex3HandController] = {}
        self._usb_controller_thread: threading.Thread | None = None
        self._usb_controller_stop = threading.Event()
        self.slam_is_running = False

        if safety_boot:
            self._client = secure_boot(iface=self.iface, domain_id=self.domain_id)
        else:
            self._client = create_loco_client(domain_id=self.domain_id, iface=self.iface)
            force_normal_gait(self._client)

        if auto_start_sensors:
            self.start_sensors()

    def _get_slam_client(self) -> SlamOperateClient:
        if self._slam_client is None:
            self._slam_client = SlamOperateClient()
            self._slam_client.Init()
            self._slam_client.SetTimeout(10.0)
        return self._slam_client

    def _get_audio(self) -> RobotAudio:
        if self._audio is None:
            self._audio = RobotAudio()
        return self._audio

    def _get_video_client(self) -> Any:
        if self._video_client is None:
            video_client_cls = load_video_client_type()
            self._video_client = video_client_cls()
            self._video_client.SetTimeout(2.0)
            self._video_client.Init()
        return self._video_client

    def _get_hand(self, hand: str = "right") -> Dex3HandController:
        side = str(hand).strip().lower()
        if side not in self._hands:
            self._hands[side] = Dex3HandController(side, iface=self.iface, domain_id=self.domain_id)
        return self._hands[side]

    def _get_arm_sdk(self) -> _ArmSdkPublisher:
        if self._arm_sdk is None:
            self._arm_sdk = _ArmSdkPublisher(iface=self.iface, domain_id=self.domain_id)
        return self._arm_sdk

    def _get_arm_action_client(self) -> Any:
        if self._arm_action_client is None:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

            self._arm_action_client = G1ArmActionClient()
            self._arm_action_client.SetTimeout(10.0)
            self._arm_action_client.Init()
        return self._arm_action_client

    def _ensure_slam_info_subscriber(self) -> SlamInfoSubscriber:
        if self._slam_info_sub is None:
            self._slam_info_sub = SlamInfoSubscriber(self.slam_info_topic, self.slam_key_topic)
            self._slam_info_sub.start()
        return self._slam_info_sub

    def _ensure_slam_odom_subscriber(self) -> SlamOdomSubscriber:
        if self._slam_odom_sub is None:
            self._slam_odom_sub = SlamOdomSubscriber()
            self._slam_odom_sub.start()
        return self._slam_odom_sub

    # ------------------------------------------------------------------
    # Sensor subscriptions
    # ------------------------------------------------------------------

    def start_sensors(self) -> None:
        if self._sport_sub is None:
            self._sport_sub = ChannelSubscriber(self.sport_topic, SportModeState_)
            self._sport_sub.Init(self._sport_cb, 10)
        if self._lidar_map_sub is None:
            self._lidar_map_sub = ChannelSubscriber(self.lidar_map_topic, HeightMap_)
            self._lidar_map_sub.Init(self._lidar_map_cb, 10)
        for topic in self.lidar_cloud_topics:
            if topic in self._lidar_cloud_subs:
                continue
            sub = ChannelSubscriber(topic, PointCloud2_)
            sub.Init(self._make_lidar_cloud_cb(topic), 10)
            self._lidar_cloud_subs[topic] = sub
        lowstate_type = resolve_lowstate_type()
        if lowstate_type is not None and self._lowstate_sub is None:
            self._lowstate_sub = LatestSubscriber("rt/lowstate", lowstate_type)
            self._lowstate_sub.start()
        if Odometry_ is not None and self._odom_sub is None:
            self._odom_sub = LatestSubscriber("rt/odom", Odometry_)
            self._odom_sub.start()
        if LidarImu_ is not None and self._lidar_imu_sub is None:
            self._lidar_imu_sub = LatestSubscriber("rt/utlidar/imu_livox_mid360", LidarImu_)
            self._lidar_imu_sub.start()
        if HandState_ is not None:
            for side, topic in HAND_STATE_TOPIC_BY_SIDE.items():
                if side in self._hand_state_subs:
                    continue
                sub = LatestSubscriber(topic, HandState_)
                sub.start(queue_len=20)
                self._hand_state_subs[side] = sub

    def _sport_cb(self, msg: SportModeState_) -> None:
        with self._lock:
            self._sport = msg
            self._last_sport_ts = time.time()

    def _lidar_map_cb(self, msg: HeightMap_) -> None:
        with self._lock:
            self._lidar_map = msg
            self._last_lidar_map_ts = time.time()

    def _make_lidar_cloud_cb(self, topic: str):
        def _lidar_cloud_cb(msg: PointCloud2_) -> None:
            with self._lock:
                self._lidar_cloud = msg
                self._lidar_cloud_by_topic[topic] = msg
                now = time.time()
                self._last_lidar_cloud_ts = now
                self._last_lidar_cloud_ts_by_topic[topic] = now

        return _lidar_cloud_cb

    def _lidar_cloud_cb(self, msg: PointCloud2_) -> None:
        with self._lock:
            self._lidar_cloud = msg
            self._last_lidar_cloud_ts = time.time()

    # ------------------------------------------------------------------
    # Generic state helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _read_attr(obj: Any, *path: str) -> Any:
        cur = obj
        for name in path:
            if cur is None or not hasattr(cur, name):
                return None
            cur = getattr(cur, name)
        return cur

    @staticmethod
    def _vector3_from(value: Any) -> tuple[float, float, float] | None:
        try:
            if value is None:
                return None
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                return (float(value[0]), float(value[1]), float(value[2]))
        except Exception:
            return None
        return None

    def get_sport_state(self) -> SportModeState_ | None:
        with self._lock:
            return self._sport

    def get_lidar_map(self) -> HeightMap_ | None:
        with self._lock:
            return self._lidar_map

    def get_lidar_cloud(self) -> dict[str, Any] | None:
        msg, topic, ts = self._get_latest_lidar_cloud_msg()
        if msg is None:
            return None
        points = self._extract_xyz_from_cloud(msg, max_points=20000, as_dict=True)
        return {
            "topic": topic,
            "timestamp": ts,
            "width": int(getattr(msg, "width", 0) or 0),
            "height": int(getattr(msg, "height", 0) or 0),
            "point_step": int(getattr(msg, "point_step", 0) or 0),
            "frame_id": self._read_attr(msg, "header", "frame_id"),
            "point_count": len(points),
            "points": points,
        }

    def get_lidar_cloud_msg(self) -> PointCloud2_ | None:
        return self._get_latest_lidar_cloud_msg()[0]

    def get_sensor_timestamps(self) -> dict[str, float]:
        with self._lock:
            timestamps = {
                "sport": float(self._last_sport_ts),
                "lidar_map": float(self._last_lidar_map_ts),
                "lidar_cloud": float(self._last_lidar_cloud_ts),
            }
            for topic, ts in self._last_lidar_cloud_ts_by_topic.items():
                timestamps[f"lidar_cloud[{topic}]"] = float(ts)
        if self._lowstate_sub is not None:
            timestamps["lowstate"] = float(self._lowstate_sub.get_latest()[1])
        if self._odom_sub is not None:
            timestamps["odom"] = float(self._odom_sub.get_latest()[1])
        if self._lidar_imu_sub is not None:
            timestamps["lidar_imu"] = float(self._lidar_imu_sub.get_latest()[1])
        if self._slam_odom_sub is not None:
            timestamps["slam_odom"] = float(self._slam_odom_sub.get_latest()[1])
        for side, sub in self._hand_state_subs.items():
            timestamps[f"{side}_hand_state"] = float(sub.get_latest()[1])
        return timestamps

    def sensors_stale(self, max_age: float = 1.0) -> dict[str, bool]:
        now = time.time()
        return {
            name: (ts <= 0.0) or ((now - ts) > max_age)
            for name, ts in self.get_sensor_timestamps().items()
        }

    def wait_for_sport_state(self, timeout: float = 2.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < max(0.0, timeout):
            if self.get_sport_state() is not None:
                return True
            time.sleep(0.05)
        return self.get_sport_state() is not None

    def wait_for_low_state(self, timeout: float = 2.0) -> bool:
        if self._lowstate_sub is None:
            return False
        t0 = time.time()
        while time.time() - t0 < max(0.0, timeout):
            if self._lowstate_sub.get_latest()[0] is not None:
                return True
            time.sleep(0.05)
        return self._lowstate_sub.get_latest()[0] is not None

    def get_mode(self) -> int | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        value = self._read_attr(msg, "mode")
        try:
            return int(value)
        except Exception:
            return None

    def get_gait(self) -> int | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        for key in ("gait_type", "gaitType", "gait"):
            value = self._read_attr(msg, key)
            if value is None:
                continue
            try:
                return int(value)
            except Exception:
                continue
        return None

    def get_body_height(self) -> float | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        for key in ("body_height", "bodyHeight", "stand_height", "standHeight"):
            value = self._read_attr(msg, key)
            if value is None:
                continue
            try:
                return float(value)
            except Exception:
                continue
        return None

    def get_position(self) -> tuple[float, float, float] | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        for key in ("position", "pos", "position_w"):
            vec = self._vector3_from(self._read_attr(msg, key))
            if vec is not None:
                return vec
        return None

    def get_velocity(self) -> tuple[float, float, float] | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        for key in ("velocity", "vel", "velocity_w"):
            vec = self._vector3_from(self._read_attr(msg, key))
            if vec is not None:
                return vec
        return None

    def get_low_state_msg(self) -> Any | None:
        if self._lowstate_sub is None:
            return None
        return self._lowstate_sub.get_latest()[0]

    def get_low_state(self) -> dict[str, Any] | None:
        joint_state = self.get_joint_states()
        if joint_state is None:
            return None
        joint_positions = [entry["position"] for entry in joint_state["joints"].values() if entry["position"] is not None]
        joint_velocities = [entry["velocity"] for entry in joint_state["joints"].values() if entry["velocity"] is not None]
        joint_torques = [entry["torque"] for entry in joint_state["joints"].values() if entry["torque"] is not None]
        return {
            "timestamp": joint_state["timestamp"],
            "joint_count": len(joint_state["joints"]),
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "joint_torques": joint_torques,
            "imu": joint_state["imu"],
            "joints": joint_state["joints"],
            "sources": joint_state["sources"],
        }

    def get_low_state_snapshot(self) -> LowStateSnapshot | None:
        msg = self.get_low_state_msg()
        if msg is None:
            return None
        return lowstate_snapshot_from_msg(msg)

    def get_joint_positions(self) -> dict[str, float | None]:
        state = self.get_joint_states()
        if state is None:
            return {}
        return {name: values["position"] for name, values in state["joints"].items()}

    def get_joint_velocities(self) -> dict[str, float | None]:
        state = self.get_joint_states()
        if state is None:
            return {}
        return {name: values["velocity"] for name, values in state["joints"].items()}

    def get_joint_torques(self) -> dict[str, float | None]:
        state = self.get_joint_states()
        if state is None:
            return {}
        return {name: values["torque"] for name, values in state["joints"].items()}

    def get_joint_position(self, joint_index: int | str) -> float | None:
        positions = self.get_joint_positions()
        key = self._resolve_joint_lookup_key(joint_index)
        if key is None:
            return None
        value = positions.get(key)
        return None if value is None else float(value)

    def _read_joint_positions_or_raise(
        self,
        joint_indices: list[int],
        *,
        timeout: float = 3.0,
    ) -> dict[int, float]:
        if not self.wait_for_low_state(timeout=max(0.1, float(timeout))):
            raise TimeoutError("Timed out waiting for rt/lowstate joint positions.")
        values: dict[int, float] = {}
        for joint_index in joint_indices:
            value = self.get_joint_position(joint_index)
            if value is None:
                name = BODY_JOINT_NAME_BY_INDEX.get(int(joint_index), str(joint_index))
                raise RuntimeError(f"Joint position for {name} is unavailable.")
            values[int(joint_index)] = float(value)
        return values

    @staticmethod
    def _with_upper_body_hold(
        joint_targets: dict[int, float],
        upper_body_positions: dict[int, float],
    ) -> dict[int, float]:
        targets = {
            int(joint_index): float(upper_body_positions[int(joint_index)])
            for joint_index in UPPER_BODY_JOINTS
        }
        for joint_index, value in joint_targets.items():
            targets[int(joint_index)] = float(value)
        return targets

    def _publish_with_upper_body_hold(
        self,
        joint_targets: dict[int, float],
        upper_body_positions: dict[int, float],
        *,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
    ) -> None:
        targets = self._with_upper_body_hold(joint_targets, upper_body_positions)
        waist_gains = {joint_index: float(waist_kp) for joint_index in WAIST_JOINTS}
        waist_damping = {joint_index: float(waist_kd) for joint_index in WAIST_JOINTS}
        self._get_arm_sdk().publish_targets(
            targets,
            kp=kp,
            kd=kd,
            kp_by_joint=waist_gains,
            kd_by_joint=waist_damping,
        )

    @staticmethod
    def _normalize_hl_arm_action_name(action: str) -> str:
        key = " ".join(str(action).strip().lower().replace("_", " ").split())
        key = HL_ARM_ACTION_ALIASES.get(key, key)
        if key in HL_ARM_ACTIONS:
            return key
        raise ValueError(
            "Unknown high-level arm action. Use one of: "
            + ", ".join(sorted(HL_ARM_ACTIONS))
        )

    @staticmethod
    def list_arm_actions() -> dict[str, int]:
        """Return the SDK high-level arm action names supported by this wrapper."""
        return dict(HL_ARM_ACTIONS)

    def get_arm_action_list(self) -> tuple[int, Any]:
        """Read the action list from the robot's high-level arm service."""
        code, actions = self._get_arm_action_client().GetActionList()
        return int(code), actions

    def execute_arm_action(
        self,
        action: str | int,
        *,
        release_after_s: float | None = None,
    ) -> int:
        """Execute a high-level G1 arm action through the SDK arm service.

        `action` may be an SDK action id or one of the names from
        :meth:`list_arm_actions`. For gestures that the SDK example releases
        after a pause, pass `release_after_s`; convenience methods do this by
        default where the example does.
        """
        if isinstance(action, str):
            action_name = self._normalize_hl_arm_action_name(action)
            action_id = HL_ARM_ACTIONS[action_name]
        else:
            action_id = int(action)

        client = self._get_arm_action_client()
        code = int(client.ExecuteAction(int(action_id)))
        if release_after_s is not None:
            time.sleep(max(0.0, float(release_after_s)))
            release_code = int(client.ExecuteAction(HL_ARM_ACTIONS[HL_ARM_ACTION_RELEASE]))
            return release_code if code == 0 else code
        return code

    def execute_hl_arm_action(
        self,
        action: str | int,
        *,
        release_after_s: float | None = None,
    ) -> int:
        return self.execute_arm_action(action, release_after_s=release_after_s)

    def release_arm(self) -> int:
        return self.execute_arm_action(HL_ARM_ACTION_RELEASE)

    def release_arms(
        self,
        *,
        duration_s: float = 3.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        """Gradually release DDS arm_sdk control from the current pose.

        The release needs continuous pose commands while authority is fading;
        otherwise a fresh `rt/arm_sdk` command can leave the non-weight joints
        at their default values and the final handoff can feel abrupt.
        """
        positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        arm_sdk = self._get_arm_sdk()
        base_kp = float(kp)
        base_kd = float(kd)
        base_waist_kp = float(waist_kp)
        base_waist_kd = float(waist_kd)
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            # Smoothstep avoids a jerk at the beginning and end of the handoff.
            fade = ratio * ratio * (3.0 - 2.0 * ratio)
            authority = 1.0 - fade
            waist_gains = {
                joint_index: base_waist_kp * authority for joint_index in WAIST_JOINTS
            }
            waist_damping = {
                joint_index: base_waist_kd * authority for joint_index in WAIST_JOINTS
            }
            arm_sdk.publish_targets(
                positions,
                kp=base_kp * authority,
                kd=base_kd * authority,
                kp_by_joint=waist_gains,
                kd_by_joint=waist_damping,
            )
            arm_sdk.publish_arm_sdk_weight(authority)
            time.sleep(dt)
        return {
            "duration_s": float(duration_s),
            "command_rate_hz": float(command_rate_hz),
            "final_arm_sdk_weight": 0.0,
            "joint_count": len(positions),
        }

    def unrelease_arms(
        self,
        *,
        duration_s: float = 1.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        """Re-enable DDS arm_sdk control while holding the current pose."""
        positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        arm_sdk = self._get_arm_sdk()
        waist_gains = {joint_index: float(waist_kp) for joint_index in WAIST_JOINTS}
        waist_damping = {joint_index: float(waist_kd) for joint_index in WAIST_JOINTS}
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            arm_sdk.publish_targets(
                positions,
                kp=kp,
                kd=kd,
                kp_by_joint=waist_gains,
                kd_by_joint=waist_damping,
            )
            arm_sdk.publish_arm_sdk_weight(ratio)
            time.sleep(dt)
        return {
            "duration_s": float(duration_s),
            "command_rate_hz": float(command_rate_hz),
            "final_arm_sdk_weight": 1.0,
            "joint_count": len(positions),
        }

    def shake_hand_action(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("shake hand", release_after_s=release_after_s)

    def shake_hand(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.shake_hand_action(release_after_s=release_after_s)

    def arm_shake_hand(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.shake_hand_action(release_after_s=release_after_s)

    def high_five(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("high five", release_after_s=release_after_s)

    def hug(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("hug", release_after_s=release_after_s)

    def high_wave(self) -> int:
        return self.execute_arm_action("high wave")

    def clap(self) -> int:
        return self.execute_arm_action("clap")

    def face_wave(self) -> int:
        return self.execute_arm_action("face wave")

    def left_kiss(self) -> int:
        return self.execute_arm_action("left kiss")

    def heart(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("heart", release_after_s=release_after_s)

    def right_heart(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("right heart", release_after_s=release_after_s)

    def hands_up(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("hands up", release_after_s=release_after_s)

    def x_ray(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("x-ray", release_after_s=release_after_s)

    def right_hand_up(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("right hand up", release_after_s=release_after_s)

    def reject(self, *, release_after_s: float | None = HL_ARM_ACTION_DEFAULT_RELEASE_DELAY_S) -> int:
        return self.execute_arm_action("reject", release_after_s=release_after_s)

    def right_kiss(self) -> int:
        return self.execute_arm_action("right kiss")

    def two_hand_kiss(self) -> int:
        return self.execute_arm_action("two-hand kiss")

    def move_upper_body_joint(
        self,
        joint_index: int,
        target: float,
        *,
        command_rate_hz: float = 50.0,
        max_speed_rad_s: float = 0.45,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        joint = int(joint_index)
        if joint not in UPPER_BODY_JOINTS:
            raise ValueError("joint_index must be a waist or arm joint.")
        positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        start = float(positions[joint])
        stop = float(target)
        steps = max(
            1,
            int(abs(stop - start) / max(0.01, float(max_speed_rad_s)) * max(1.0, float(command_rate_hz))),
        )
        dt = 1.0 / max(1.0, float(command_rate_hz))
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            value = start + (stop - start) * alpha
            self._publish_with_upper_body_hold(
                {joint: value},
                positions,
                kp=kp,
                kd=kd,
                waist_kp=waist_kp,
                waist_kd=waist_kd,
            )
            time.sleep(dt)
        return {
            "joint_index": joint,
            "joint_name": BODY_JOINT_NAME_BY_INDEX[joint],
            "start": start,
            "target": stop,
            "command_rate_hz": float(command_rate_hz),
            "max_speed_rad_s": float(max_speed_rad_s),
        }

    def extend_arm_forward(
        self,
        *,
        arm: str = "right",
        duration_s: float = 4.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
        shoulder_roll_delta: float = 0.50,
        shoulder_roll_restore_fraction: float = 0.45,
        shoulder_pitch_delta: float = 0.35,
        elbow_delta: float = 0.9,
        wrist_roll_delta: float = 0.12,
        wrist_pitch_delta: float = 0.4,
    ) -> dict[str, Any]:
        side = str(arm).strip().lower()
        if side not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'.")
        arm_joints = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        roll_delta = abs(float(shoulder_roll_delta)) if side == "left" else -abs(float(shoulder_roll_delta))
        pitch_delta = -abs(float(shoulder_pitch_delta))
        elbow_delta_signed = -abs(float(elbow_delta))
        wrist_roll_delta_signed = abs(float(wrist_roll_delta))
        wrist_pitch_delta_signed = -abs(float(wrist_pitch_delta))
        joint_limits = {
            arm_joints[0]: (-3.0892, 2.6704),
            arm_joints[1]: (-1.5882, 2.2515) if side == "left" else (-2.2515, 1.5882),
            arm_joints[3]: (-1.0472, 2.0944),
            arm_joints[4]: (-1.9722, 1.9722),
            arm_joints[5]: (-1.6144, 1.6144),
        }

        def clamp_joint(joint_index: int, value: float) -> float:
            lo, hi = joint_limits[int(joint_index)]
            return max(lo, min(hi, float(value)))

        initial_positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        start_pose = [initial_positions[joint_index] for joint_index in arm_joints]

        steps = max(1, int(max(0.02, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        stage_1_steps = max(1, steps // 4)
        stage_2_steps = max(1, steps // 4)
        stage_3_steps = max(1, steps // 4)
        stage_4_steps = max(1, steps - stage_1_steps - stage_2_steps - stage_3_steps)

        roll_pose = list(start_pose)
        roll_pose[1] = clamp_joint(arm_joints[1], float(start_pose[1]) + roll_delta)

        pitch_pose = list(roll_pose)
        pitch_pose[0] = clamp_joint(arm_joints[0], float(start_pose[0]) + pitch_delta)

        restored_roll_pose = list(pitch_pose)
        restore_fraction = max(0.0, min(1.0, float(shoulder_roll_restore_fraction)))
        restored_roll_pose[1] = clamp_joint(
            arm_joints[1],
            float(roll_pose[1]) - (roll_delta * restore_fraction),
        )

        target_pose = list(restored_roll_pose)
        target_pose[3] = clamp_joint(arm_joints[3], float(start_pose[3]) + elbow_delta_signed)
        target_pose[4] = clamp_joint(arm_joints[4], float(start_pose[4]) + wrist_roll_delta_signed)
        target_pose[5] = clamp_joint(arm_joints[5], float(start_pose[5]) + wrist_pitch_delta_signed)
        stages = [
            (start_pose, roll_pose, stage_1_steps, "shoulder_roll_clearance"),
            (roll_pose, pitch_pose, stage_2_steps, "shoulder_pitch_forward"),
            (pitch_pose, restored_roll_pose, stage_3_steps, "partial_shoulder_roll_restore"),
            (restored_roll_pose, target_pose, stage_4_steps, "elbow_and_wrist_pitch"),
        ]

        for stage_start, stage_target, stage_steps, _stage_name in stages:
            for step_idx in range(1, stage_steps + 1):
                alpha = float(step_idx) / float(stage_steps)
                arm_pose = [
                    (1.0 - alpha) * float(start_q) + alpha * float(target_q)
                    for start_q, target_q in zip(stage_start, stage_target)
                ]
                joint_targets = {
                    joint_index: pose_value
                    for joint_index, pose_value in zip(arm_joints, arm_pose)
                }
                self._publish_with_upper_body_hold(
                    joint_targets,
                    initial_positions,
                    kp=kp,
                    kd=kd,
                    waist_kp=waist_kp,
                    waist_kd=waist_kd,
                )
                time.sleep(dt)

        return {
            "arm": side,
            "start_pose": start_pose,
            "clearance_pose": roll_pose,
            "forward_pose": pitch_pose,
            "restored_roll_pose": restored_roll_pose,
            "target_pose": target_pose,
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint_index] for joint_index in arm_joints],
            "stages": [stage_name for _start, _target, _steps, stage_name in stages],
            "command_rate_hz": float(command_rate_hz),
            "duration_s": float(duration_s),
        }

    def extend_right_arm_forward(
        self,
        *,
        duration_s: float = 4.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        return self.extend_arm_forward(
            arm="right",
            duration_s=duration_s,
            command_rate_hz=command_rate_hz,
            kp=kp,
            kd=kd,
            timeout=timeout,
        )

    def retract_arm_forward(
        self,
        *,
        arm: str = "right",
        duration_s: float = 4.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
        shoulder_roll_delta: float = 0.50,
        shoulder_roll_restore_fraction: float = 0.45,
        shoulder_pitch_delta: float = 0.35,
        elbow_delta: float = 1,
        wrist_roll_delta: float = 0.16,
        wrist_pitch_delta: float = 0.4,
    ) -> dict[str, Any]:
        """Best-effort inverse of :meth:`extend_arm_forward`.

        This exactly reverses an `extend_arm_forward` call when the forward
        motion did not hit joint limits and the same delta parameters are used.
        """
        side = str(arm).strip().lower()
        if side not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'.")
        arm_joints = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        roll_delta = abs(float(shoulder_roll_delta)) if side == "left" else -abs(float(shoulder_roll_delta))
        pitch_delta = -abs(float(shoulder_pitch_delta))
        elbow_delta_signed = -abs(float(elbow_delta))
        wrist_roll_delta_signed = abs(float(wrist_roll_delta))
        wrist_pitch_delta_signed = -abs(float(wrist_pitch_delta))
        restore_fraction = max(0.0, min(1.0, float(shoulder_roll_restore_fraction)))
        joint_limits = {
            arm_joints[0]: (-3.0892, 2.6704),
            arm_joints[1]: (-1.5882, 2.2515) if side == "left" else (-2.2515, 1.5882),
            arm_joints[3]: (-1.0472, 2.0944),
            arm_joints[4]: (-1.9722, 1.9722),
            arm_joints[5]: (-1.6144, 1.6144),
        }

        def clamp_joint(joint_index: int, value: float) -> float:
            lo, hi = joint_limits[int(joint_index)]
            return max(lo, min(hi, float(value)))

        initial_positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        start_pose = [initial_positions[joint_index] for joint_index in arm_joints]

        steps = max(1, int(max(0.02, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        stage_1_steps = max(1, steps // 4)
        stage_2_steps = max(1, steps // 4)
        stage_3_steps = max(1, steps // 4)
        stage_4_steps = max(1, steps - stage_1_steps - stage_2_steps - stage_3_steps)

        wrist_and_elbow_retracted_pose = list(start_pose)
        wrist_and_elbow_retracted_pose[3] = clamp_joint(arm_joints[3], float(start_pose[3]) - elbow_delta_signed)
        wrist_and_elbow_retracted_pose[4] = clamp_joint(arm_joints[4], float(start_pose[4]) - wrist_roll_delta_signed)
        wrist_and_elbow_retracted_pose[5] = clamp_joint(arm_joints[5], float(start_pose[5]) - wrist_pitch_delta_signed)

        clearance_roll_pose = list(wrist_and_elbow_retracted_pose)
        clearance_roll_pose[1] = clamp_joint(
            arm_joints[1],
            float(start_pose[1]) + (roll_delta * restore_fraction),
        )

        shoulder_pitch_retracted_pose = list(clearance_roll_pose)
        shoulder_pitch_retracted_pose[0] = clamp_joint(arm_joints[0], float(start_pose[0]) - pitch_delta)

        target_pose = list(shoulder_pitch_retracted_pose)
        target_pose[1] = clamp_joint(
            arm_joints[1],
            float(start_pose[1]) - (roll_delta * (1.0 - restore_fraction)),
        )
        stages = [
            (start_pose, wrist_and_elbow_retracted_pose, stage_4_steps, "undo_elbow_and_wrist_pitch"),
            (wrist_and_elbow_retracted_pose, clearance_roll_pose, stage_3_steps, "restore_shoulder_roll_clearance"),
            (clearance_roll_pose, shoulder_pitch_retracted_pose, stage_2_steps, "shoulder_pitch_back"),
            (shoulder_pitch_retracted_pose, target_pose, stage_1_steps, "shoulder_roll_home"),
        ]

        for stage_start, stage_target, stage_steps, _stage_name in stages:
            for step_idx in range(1, stage_steps + 1):
                alpha = float(step_idx) / float(stage_steps)
                arm_pose = [
                    (1.0 - alpha) * float(start_q) + alpha * float(target_q)
                    for start_q, target_q in zip(stage_start, stage_target)
                ]
                joint_targets = {
                    joint_index: pose_value
                    for joint_index, pose_value in zip(arm_joints, arm_pose)
                }
                self._publish_with_upper_body_hold(
                    joint_targets,
                    initial_positions,
                    kp=kp,
                    kd=kd,
                    waist_kp=waist_kp,
                    waist_kd=waist_kd,
                )
                time.sleep(dt)

        return {
            "arm": side,
            "start_pose": start_pose,
            "wrist_and_elbow_retracted_pose": wrist_and_elbow_retracted_pose,
            "clearance_roll_pose": clearance_roll_pose,
            "shoulder_pitch_retracted_pose": shoulder_pitch_retracted_pose,
            "target_pose": target_pose,
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint_index] for joint_index in arm_joints],
            "stages": [stage_name for _start, _target, _steps, stage_name in stages],
            "command_rate_hz": float(command_rate_hz),
            "duration_s": float(duration_s),
        }

    def retract_right_arm_forward(
        self,
        *,
        duration_s: float = 4.0,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        return self.retract_arm_forward(
            arm="right",
            duration_s=duration_s,
            command_rate_hz=command_rate_hz,
            kp=kp,
            kd=kd,
            timeout=timeout,
        )

    def get_odom(self) -> Any | None:
        if self._odom_sub is None:
            return None
        return self._odom_sub.get_latest()[0]

    def get_odom_pose(self) -> tuple[float, float, float] | None:
        msg = self.get_odom()
        if msg is None:
            return None
        return odom_pose_from_msg(msg)

    def get_lidar_imu(self) -> Any | None:
        if self._lidar_imu_sub is None:
            return None
        return self._lidar_imu_sub.get_latest()[0]

    def get_yaw(self) -> float | None:
        imu = self.get_imu()
        if imu is None:
            return None
        return float(imu.rpy[2])

    def is_moving(self, linear_eps: float = 0.03, yaw_eps: float = 0.08) -> bool:
        velocity = self.get_velocity()
        if velocity is None:
            return False
        vx, vy, vz = velocity
        return math.hypot(vx, vy) > linear_eps or abs(vz) > yaw_eps

    def get_robot_state(self) -> dict[str, Any]:
        return {
            "fsm": self.get_fsm(),
            "mode": self.get_mode(),
            "gait": self.get_gait(),
            "body_height": self.get_body_height(),
            "position": self.get_position(),
            "velocity": self.get_velocity(),
            "yaw": self.get_yaw(),
            "is_moving": self.is_moving(),
            "imu": self.get_imu(),
            "odom_pose": self.get_odom_pose(),
            "slam_pose": self.get_slam_pose(),
            "joint_count": len(self.get_joint_positions()),
            "sensor_timestamps": self.get_sensor_timestamps(),
            "sensor_stale": self.sensors_stale(),
            "slam_is_running": bool(self.slam_is_running),
            "queued_path_points": len(self._path_points),
        }

    # ------------------------------------------------------------------
    # Locomotion + FSM
    # ------------------------------------------------------------------

    @staticmethod
    def _normalize_sdk_status(result: Any) -> int:
        # Some SDK bindings return None for successful non-blocking motion calls.
        if result is None:
            return 0
        return int(result)

    def loco_move(self, vx: float, vy: float, vyaw: float) -> int:
        result = self._client.Move(float(vx), float(vy), float(vyaw), continous_move=True)
        return self._normalize_sdk_status(result)

    def move_for(
        self,
        duration: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
    ) -> int:
        duration_s = float(duration)
        if duration_s < 0.0:
            raise ValueError("duration must be >= 0.0")
        result = self.loco_move(vx, vy, vyaw)
        try:
            time.sleep(duration_s)
        finally:
            self.stop()
        return result

    def stop_moving(self) -> None:
        if hasattr(self._client, "StopMove"):
            self._client.StopMove()
            return
        self._client.Move(0.0, 0.0, 0.0, continous_move=False)

    def stop(self) -> None:
        self.stop_moving()

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        dz = min(0.99, max(0.0, float(deadzone)))
        sample = float(value)
        if abs(sample) < dz:
            return 0.0
        sign = 1.0 if sample > 0.0 else -1.0
        return sign * (abs(sample) - dz) / (1.0 - dz)

    def zero_torque(self) -> None:
        self.fsm_0_zt()

    def damp(self) -> None:
        self.fsm_1_damp()

    def enter_walking_ready_mode(self) -> None:
        if hasattr(self._client, "Damp"):
            self._client.Damp()
            time.sleep(0.5)
        if hasattr(self._client, "Squat2StandUp"):
            self._client.Squat2StandUp()
            time.sleep(1.0)
        if hasattr(self._client, "Start"):
            self._client.Start()
            return
        self.balanced_stand()

    def _usb_controller_loop(
        self,
        *,
        joy_index: int,
        send_hz: float,
        max_vx: float,
        max_vy: float,
        max_vyaw: float,
        deadzone: float,
    ) -> None:
        try:
            import pygame
        except ModuleNotFoundError as exc:
            raise SystemExit(
                "The 'pygame' package is required for USB controller support.\n"
                "Install with: pip install pygame"
            ) from exc

        btn_a = 0
        btn_b = 1
        btn_x = 2
        btn_y = 3
        btn_start = 7
        axis_lx = 0
        axis_ly = 1
        axis_rx = 3

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            pygame.quit()
            raise RuntimeError("No joystick detected. Connect a USB gamepad and retry.")
        if joy_index < 0 or joy_index >= pygame.joystick.get_count():
            pygame.quit()
            raise IndexError(f"Joystick index {joy_index} is out of range.")

        joy = pygame.joystick.Joystick(int(joy_index))
        joy.init()

        active = True
        dt = 1.0 / max(1.0, float(send_hz))
        try:
            while not self._usb_controller_stop.is_set():
                pygame.event.pump()

                if joy.get_numbuttons() > btn_y and joy.get_button(btn_y):
                    self.zero_torque()
                    active = False
                    time.sleep(0.5)
                    continue

                if joy.get_numbuttons() > btn_a and joy.get_button(btn_a):
                    self.damp()
                    active = False
                    time.sleep(0.5)
                    continue

                if joy.get_numbuttons() > btn_x and joy.get_button(btn_x):
                    self.enter_walking_ready_mode()
                    active = True
                    time.sleep(0.5)
                    continue

                if joy.get_numbuttons() > btn_b and joy.get_button(btn_b):
                    self.balanced_stand()
                    active = True
                    time.sleep(0.5)
                    continue

                if joy.get_numbuttons() > btn_start and joy.get_button(btn_start):
                    self.stop()
                    time.sleep(0.2)
                    continue

                if active:
                    lx = self._apply_deadzone(joy.get_axis(axis_lx), deadzone)
                    ly = self._apply_deadzone(joy.get_axis(axis_ly), deadzone)
                    rx = self._apply_deadzone(joy.get_axis(axis_rx), deadzone)

                    vx = -ly * float(max_vx)
                    vy = -lx * float(max_vy)
                    vyaw = -rx * float(max_vyaw)
                    self.loco_move(vx=vx, vy=vy, vyaw=vyaw)

                time.sleep(dt)
        finally:
            self.stop()
            joy.quit()
            pygame.joystick.quit()
            pygame.quit()

    def start_usb_controller(
        self,
        joy_index: int = 0,
        send_hz: float = 10.0,
        max_vx: float = 0.5,
        max_vy: float = 0.3,
        max_vyaw: float = 0.8,
        deadzone: float = 0.1,
    ) -> threading.Thread:
        thread = self._usb_controller_thread
        if thread is not None and thread.is_alive():
            raise RuntimeError("USB controller loop is already running.")

        try:
            import pygame
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "The 'pygame' package is required for USB controller support. "
                "Install it with: pip install pygame"
            ) from exc

        pygame.init()
        pygame.joystick.init()
        try:
            if pygame.joystick.get_count() == 0:
                raise RuntimeError("No joystick detected. Connect a USB gamepad and retry.")
            if joy_index < 0 or joy_index >= pygame.joystick.get_count():
                raise IndexError(f"Joystick index {joy_index} is out of range.")
        finally:
            pygame.joystick.quit()
            pygame.quit()

        self._usb_controller_stop = threading.Event()
        self._usb_controller_thread = threading.Thread(
            target=self._usb_controller_loop,
            kwargs={
                "joy_index": int(joy_index),
                "send_hz": float(send_hz),
                "max_vx": float(max_vx),
                "max_vy": float(max_vy),
                "max_vyaw": float(max_vyaw),
                "deadzone": float(deadzone),
            },
            name=f"usb-controller-{int(joy_index)}",
            daemon=True,
        )
        self._usb_controller_thread.start()
        return self._usb_controller_thread

    def stop_usb_controller(self, join_timeout: float = 1.0) -> None:
        self._usb_controller_stop.set()
        thread = self._usb_controller_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, float(join_timeout)))
        self._usb_controller_thread = None

    def walk_mode(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(501)
            return
        raise AttributeError("Current locomotion client does not support FSM mode setting API.")

    def run_mode(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(802)
            return
        raise AttributeError("Current locomotion client does not support FSM mode setting API.")

    def dev_mode(self) -> None:
        if hasattr(self._client, "SetGaitType"):
            self._client.SetGaitType(3)
            return
        if hasattr(self._client, "SetBalanceMode"):
            self._client.SetBalanceMode(3)
            return
        raise AttributeError("Current locomotion client does not support dev mode gait API.")

    def balanced_stand(self, mode: int = 0) -> None:
        if hasattr(self._client, "BalanceStand"):
            self._client.BalanceStand(int(mode))
            return
        if hasattr(self._client, "SetFsmId") and int(mode) == 0:
            self._client.SetFsmId(500)
            return
        raise AttributeError("Current locomotion client does not support balanced stand API.")

    def _rpc_get_int(self, api_id: int) -> Optional[int]:
        return rpc_get_int(self._client, api_id)

    def get_fsm(self) -> dict[str, Optional[int]]:
        return {
            "id": self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_ID),
            "mode": self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_MODE),
        }

    def fsm_0_zt(self) -> None:
        if hasattr(self._client, "ZeroTorque"):
            self._client.ZeroTorque()
        elif hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(0)

    def fsm_1_damp(self) -> None:
        if hasattr(self._client, "Damp"):
            self._client.Damp()
        elif hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(1)

    def fsm_2_airborne(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(2)

    def fsm_2_squat(self) -> None:
        self.fsm_2_squat_placeholder()

    def fsm_2_squat_placeholder(self) -> None:
        self.fsm_2_airborne()

    def fsm_dev_mode(self) -> None:
        if hasattr(self._client, "Start"):
            self._client.Start()
        elif hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(500)

    # ------------------------------------------------------------------
    # IMU + lidar getters
    # ------------------------------------------------------------------

    def get_imu(self) -> ImuData | None:
        with self._lock:
            msg = self._sport
        if msg is None:
            return None

        rpy = (0.0, 0.0, 0.0)
        gyro = acc = quat = None
        temp = None

        try:
            rpy = tuple(float(msg.imu_state.rpy[i]) for i in range(3))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            gyro = tuple(float(msg.imu_state.gyroscope[i]) for i in range(3))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            acc = tuple(float(msg.imu_state.accelerometer[i]) for i in range(3))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            quat = tuple(float(msg.imu_state.quaternion[i]) for i in range(4))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            temp = float(msg.imu_state.temperature)
        except Exception:
            pass

        return ImuData(rpy=rpy, gyro=gyro, acc=acc, quat=quat, temp=temp)

    @staticmethod
    def _extract_xyz_from_cloud(
        msg: PointCloud2_,
        max_points: int | None = None,
        *,
        as_dict: bool = False,
    ) -> list[Any]:
        try:
            width = int(msg.width)
            height = int(msg.height)
            point_step = int(msg.point_step)
            raw = bytes(msg.data)
        except Exception:
            return []

        if point_step <= 0:
            return []

        x_off, y_off, z_off = 0, 4, 8
        try:
            fields = list(msg.fields)
            name_to_off = {str(field.name).lower(): int(field.offset) for field in fields}
            x_off = name_to_off.get("x", x_off)
            y_off = name_to_off.get("y", y_off)
            z_off = name_to_off.get("z", z_off)
        except Exception:
            pass

        total = max(0, width * height)
        if max_points is not None:
            total = min(total, max_points)

        points: list[Any] = []
        for idx in range(total):
            base = idx * point_step
            try:
                x = struct.unpack_from("<f", raw, base + x_off)[0]
                y = struct.unpack_from("<f", raw, base + y_off)[0]
                z = struct.unpack_from("<f", raw, base + z_off)[0]
            except Exception:
                break
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                if as_dict:
                    points.append({"x": float(x), "y": float(y), "z": float(z)})
                else:
                    points.append((float(x), float(y), float(z)))
        return points

    def get_lidar_points(self, max_points: int | None = 20000) -> list[dict[str, float]]:
        msg, _topic, _ts = self._get_latest_lidar_cloud_msg()
        if msg is None:
            return []
        return self._extract_xyz_from_cloud(msg, max_points=max_points, as_dict=True)

    def get_camera_image_jpeg(self) -> bytes:
        code, data = self._get_video_client().GetImageSample()
        if int(code) != 0:
            raise RuntimeError(f"GetImageSample failed with code={code}")
        return bytes(data)

    def get_rgb_jpeg(self, timeout: float | None = None) -> bytes:
        _ = timeout
        return self.get_camera_image_jpeg()

    def get_camera_frame_bgr(self):
        return decode_video_frame_bgr(self.get_camera_image_jpeg())

    def get_camera_frame_rgb(self):
        import cv2

        frame = self.get_camera_frame_bgr()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def get_rgbd(self, timeout: float = 2.0) -> dict[str, Any]:
        rgb_jpeg, depth_png, depth_scale, timestamp = self._recv_rgbd_payload(timeout=timeout)
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            raise RuntimeError(f"RGBD decoding requires cv2 and numpy: {exc}") from exc

        rgb = cv2.imdecode(np.frombuffer(rgb_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        depth_raw = cv2.imdecode(np.frombuffer(depth_png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if rgb is None:
            raise RuntimeError("Failed to decode RGB JPEG from RGBD stream.")
        if depth_raw is None:
            raise RuntimeError("Failed to decode depth PNG from RGBD stream.")
        if depth_raw.ndim == 3:
            depth_raw = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)

        depth_m = depth_raw.astype("float32") * float(depth_scale)
        valid = depth_raw > 0
        h, w = depth_raw.shape[:2]
        center_size = max(8, min(w, h) // 12)
        cx = w // 2
        cy = h // 2
        center = depth_m[
            max(0, cy - center_size) : min(h, cy + center_size),
            max(0, cx - center_size) : min(w, cx + center_size),
        ]
        roi = depth_m[int(h * 0.25) : int(h * 0.70), int(w * 0.30) : int(w * 0.70)]
        center_valid = center[center > 0]
        center_depth_m = float(__import__("numpy").median(center_valid)) if center_valid.size else None
        near_coverage_1m = float(__import__("numpy").mean((roi > 0) & (roi <= 1.0))) if roi.size else None

        return {
            "source": f"zmq://{self.rgbd_host}:{self.rgbd_port}",
            "topic": self.rgbd_topic,
            "timestamp": float(timestamp),
            "rgb_jpeg": rgb_jpeg,
            "depth_png": depth_png,
            "rgb_bgr": rgb,
            "rgb_rgb": cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB),
            "depth_raw": depth_raw,
            "depth_m": depth_m,
            "depth_scale_m_per_unit": float(depth_scale),
            "center_depth_m": center_depth_m,
            "near_coverage_1m": near_coverage_1m,
            "valid_depth_fraction": float(valid.mean()) if valid.size else 0.0,
        }

    # ------------------------------------------------------------------
    # SLAM + navigation
    # ------------------------------------------------------------------

    def start_slam(self, slam_type: str = "indoor") -> int:
        response = self._get_slam_client().start_mapping(slam_type=slam_type)
        self.slam_is_running = response.code == 0
        return int(response.code)

    def stop_slam(self, save_path: str | None = None) -> int:
        client = self._get_slam_client()
        response = client.end_mapping(save_path) if save_path else client.close_slam()
        self.slam_is_running = False
        return int(response.code)

    def set_path_point(self, x: float, y: float, yaw: float = 0.0) -> None:
        self._path_points.append((float(x), float(y), float(yaw)))

    def get_path_points(self) -> list[tuple[float, float, float]]:
        return list(self._path_points)

    def clear_path_points(self) -> None:
        self._path_points.clear()

    def _run_pose_nav(self, x: float, y: float, yaw: float = 0.0) -> int:
        client = self._get_slam_client()
        qz = math.sin(float(yaw) * 0.5)
        qw = math.cos(float(yaw) * 0.5)
        response = client.pose_nav(float(x), float(y), 0.0, 0.0, 0.0, qz, qw, mode=1)
        return int(response.code)

    @staticmethod
    def _format_pose_debug(pose: tuple[float, float, float] | None) -> str:
        if pose is None:
            return "None"
        return f"({float(pose[0]):.3f}, {float(pose[1]):.3f}, {float(pose[2]):.3f})"

    def _trace_nav_result(
        self,
        *,
        step_idx: int,
        target: tuple[float, float, float],
        before_slam: tuple[float, float, float] | None,
        trace_duration_s: float = 2.0,
        sample_period_s: float = 0.5,
    ) -> None:
        target_x, target_y, target_yaw = target
        t0 = time.time()
        deadline = t0 + max(0.0, float(trace_duration_s))
        sample_idx = 0
        while True:
            now = time.time()
            slam_pose = self.get_slam_pose(timeout_s=0.15)
            odom_pose = self.get_odom_pose()
            dist = None
            if slam_pose is not None:
                dist = math.hypot(float(target_x) - float(slam_pose[0]), float(target_y) - float(slam_pose[1]))
            moved = None
            if before_slam is not None and slam_pose is not None:
                moved = math.hypot(float(slam_pose[0]) - float(before_slam[0]), float(slam_pose[1]) - float(before_slam[1]))
            extra = []
            if dist is not None:
                extra.append(f"dist_to_target={dist:.3f}m")
            if moved is not None:
                extra.append(f"slam_delta={moved:.3f}m")
            print(
                f"[navigate_path] trace step={step_idx} sample={sample_idx} "
                f"target={self._format_pose_debug((target_x, target_y, target_yaw))} "
                f"slam_pose={self._format_pose_debug(slam_pose)} "
                f"odom_pose={self._format_pose_debug(odom_pose)}"
                + (f" {' '.join(extra)}" if extra else "")
            )
            if now >= deadline:
                break
            sample_idx += 1
            time.sleep(max(0.05, float(sample_period_s)))

    def navigate_path(self, clear_on_finish: bool = True) -> bool:
        if not self._path_points:
            raise RuntimeError("No path points queued. Call set_path_point(...) first.")

        if not self.slam_is_running:
            print("[navigate_path] SLAM is not running; pose_nav requests are expected to fail.")
            return False

        slam_status = self.get_slam_pose_status(timeout_s=0.40)
        if not bool(slam_status.get("usable")):
            print(
                "[navigate_path] SLAM pose is not usable for navigation: "
                f"reason={slam_status.get('reason')} "
                f"slam_pose={slam_status.get('pose')} "
                f"sport_pose={slam_status.get('sport_pose')} "
                f"sport_vs_slam_xy_gap_m={slam_status.get('sport_vs_slam_xy_gap_m')}"
            )
            return False

        try:
            self.walk_mode()
        except Exception as exc:
            print(f"[navigate_path] warning: failed to enter walk mode ({exc})")

        ok = True
        try:
            for idx, (x, y, yaw) in enumerate(self._path_points, start=1):
                pos = self.get_position()
                slam_pos = self.get_slam_pose(timeout_s=0.20)
                odom_pose = self.get_odom_pose()
                target_pose = (float(x), float(y), float(yaw))
                frame_gap = None
                if pos is not None and slam_pos is not None:
                    frame_gap = math.hypot(float(pos[0]) - float(slam_pos[0]), float(pos[1]) - float(slam_pos[1]))
                print(
                    f"[navigate_path] step={idx} target={self._format_pose_debug(target_pose)} "
                    f"sport_pose={self._format_pose_debug(pos)} "
                    f"slam_pose={self._format_pose_debug(slam_pos)} "
                    f"odom_pose={self._format_pose_debug(odom_pose)}"
                    + (f" sport_vs_slam_xy_gap={frame_gap:.3f}m" if frame_gap is not None else "")
                )
                if pos is not None:
                    dxy = math.hypot(float(x) - float(pos[0]), float(y) - float(pos[1]))
                    # pose_nav commonly rejects goals that are already effectively reached.
                    if dxy <= 0.20:
                        print(f"[navigate_path] step={idx} skipped: sport_pose already within {dxy:.3f}m of target.")
                        continue
                rc = self._run_pose_nav(x, y, yaw)
                print(f"[navigate_path] step={idx} pose_nav rc={rc}")
                ref = slam_pos if slam_pos is not None else pos
                if rc == 4 and ref is not None:
                    dxy = math.hypot(float(x) - float(ref[0]), float(y) - float(ref[1]))
                    print(
                        "[navigate_path] pose_nav rc=4 likely frame/relocalization mismatch or planner rejection; "
                        f"reference_dist={dxy:.3f}m slam_pose={slam_pos} odom_pose={pos} goal=({x:.3f},{y:.3f})"
                    )
                self._trace_nav_result(step_idx=idx, target=target_pose, before_slam=slam_pos)
                if rc != 0:
                    print(f"[navigate_path] failed at point {idx}: ({x:.3f},{y:.3f},{yaw:.3f}) rc={rc}")
                    ok = False
                    break
        finally:
            if clear_on_finish:
                self._path_points.clear()
        return ok

    def get_slam_info(self) -> str | None:
        return self._ensure_slam_info_subscriber().get_info()

    def get_slam_key(self) -> str | None:
        return self._ensure_slam_info_subscriber().get_key()

    def get_slam_pose(self, timeout_s: float = 0.4) -> tuple[float, float, float] | None:
        sub = self._ensure_slam_info_subscriber()
        t0 = time.time()
        while time.time() - t0 < max(0.05, float(timeout_s)):
            pose = sub.get_pose()
            if pose is not None:
                return pose
            time.sleep(0.03)
        return None

    @staticmethod
    def _is_origin_like_pose(
        pose: tuple[float, float, float] | None,
        *,
        xy_eps: float = 0.05,
        yaw_eps: float = 0.15,
    ) -> bool:
        if pose is None:
            return False
        return math.hypot(float(pose[0]), float(pose[1])) <= float(xy_eps) and abs(float(pose[2])) <= float(yaw_eps)

    def get_slam_pose_status(self, timeout_s: float = 0.4) -> dict[str, Any]:
        pose = self.get_slam_pose(timeout_s=timeout_s)
        sport_pose = self.get_position()
        status: dict[str, Any] = {
            "pose": pose,
            "sport_pose": sport_pose,
            "slam_running": bool(self.slam_is_running),
            "usable": pose is not None,
            "reason": "ok" if pose is not None else "no_pose",
            "sport_vs_slam_xy_gap_m": None,
        }

        if pose is not None and sport_pose is not None:
            gap = math.hypot(float(sport_pose[0]) - float(pose[0]), float(sport_pose[1]) - float(pose[1]))
            status["sport_vs_slam_xy_gap_m"] = float(gap)
            sport_radius = math.hypot(float(sport_pose[0]), float(sport_pose[1]))
            if self._is_origin_like_pose(pose) and sport_radius > 0.50 and gap > 0.50:
                status["usable"] = False
                status["reason"] = "origin_like_pose_but_robot_not_near_origin"
        return status

    def get_slam_odom_pose(self) -> tuple[float, float, float] | None:
        return self._ensure_slam_odom_subscriber().get_pose()

    def debug_api(
        self,
        save_path: str = "/home/unitree/test1.pcd",
        load_path: str = "/home/unitree/test1.pcd",
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        goal_yaw: float = 0.0,
        pause: bool = False,
        resume: bool = False,
        wait_task_result: bool = False,
    ) -> None:
        def print_resp(label: str, req: dict[str, Any], resp: SlamResponse) -> None:
            print(f"\n[{label}]")
            print("request:", json.dumps(req, indent=2))
            print(f"response: code={resp.code} raw={resp.raw}")

        def wait_task(sub: SlamInfoSubscriber, timeout: float = 10.0) -> None:
            t0 = time.time()
            while time.time() - t0 < timeout:
                key = sub.get_key()
                if key:
                    try:
                        payload = json.loads(key)
                        if payload.get("type") == "task_result":
                            print("task_result:", json.dumps(payload, indent=2))
                            return
                    except Exception:
                        pass
                time.sleep(0.05)
            print("task_result: timeout")

        info_sub = SlamInfoSubscriber(self.slam_info_topic, self.slam_key_topic)
        info_sub.start()

        client = self._get_slam_client()

        req = {"data": {"slam_type": "indoor"}}
        print_resp("start_mapping (1801)", req, client.start_mapping("indoor"))

        req = {"data": {"address": save_path}}
        print_resp("end_mapping (1802)", req, client.end_mapping(save_path))

        req = {
            "data": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "q_x": 0.0,
                "q_y": 0.0,
                "q_z": 0.0,
                "q_w": 1.0,
                "address": load_path,
            }
        }
        print_resp("init_pose (1804)", req, client.init_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, load_path))

        qz = math.sin(float(goal_yaw) * 0.5)
        qw = math.cos(float(goal_yaw) * 0.5)
        req = {
            "data": {
                "targetPose": {
                    "x": float(goal_x),
                    "y": float(goal_y),
                    "z": 0.0,
                    "q_x": 0.0,
                    "q_y": 0.0,
                    "q_z": qz,
                    "q_w": qw,
                },
                "mode": 1,
            }
        }
        print_resp("pose_nav (1102)", req, client.pose_nav(float(goal_x), float(goal_y), 0.0, 0.0, 0.0, qz, qw, mode=1))

        if pause:
            print_resp("pause_nav (1201)", {"data": {}}, client.pause_nav())
        if resume:
            print_resp("resume_nav (1202)", {"data": {}}, client.resume_nav())
        if wait_task_result:
            wait_task(info_sub)

        print_resp("close_slam (1901)", {"data": {}}, client.close_slam())

    # ------------------------------------------------------------------
    # Safety + audio
    # ------------------------------------------------------------------

    def hanged_boot(
        self,
        step: float = 0.02,
        max_height: float = 0.5,
        max_attempts: int = 3,
        require_confirmation: bool = True,
        interactive_retry: bool | None = None,
    ) -> None:
        self._client = secure_boot(
            iface=self.iface,
            domain_id=self.domain_id,
            step=step,
            max_height=max_height,
            max_attempts=max_attempts,
            require_confirmation=require_confirmation,
            interactive_retry=interactive_retry,
        )

    def hanging_boot_placeholder(self) -> None:
        self.hanged_boot()

    def hanging_boot(self) -> None:
        self.hanging_boot_placeholder()

    def say(self, text: str = "what would you like me to say?", volume: int | None = None) -> int:
        return self._get_audio().speak(text, volume=volume)

    def play_wav(self, wav_path: str, volume: int | None = None) -> int:
        return self._get_audio().play_wav(wav_path, volume=volume)

    def headlight(
        self,
        color: str = "white",
        intensity: int = 100,
        duration: float | None = None,
    ) -> int:
        return self._get_audio().set_headlight(color=color, intensity=intensity, duration=duration)

    def hand_open(
        self,
        hand: str = "right",
        hold_s: float = 0.6,
        rate_hz: float = 50.0,
        ramp_s: float | None = None,
    ) -> None:
        self._get_hand(hand).open(hold_s=hold_s, rate_hz=rate_hz, ramp_s=ramp_s)

    def hand_close(
        self,
        hand: str = "right",
        hold_s: float = 0.6,
        rate_hz: float = 50.0,
        ramp_s: float | None = None,
    ) -> None:
        self._get_hand(hand).close(hold_s=hold_s, rate_hz=rate_hz, ramp_s=ramp_s)

    def release_fingers(
        self,
        hand: str = "right",
        hold_s: float = 0.5,
        rate_hz: float = 50.0,
        persistent: bool = True,
    ) -> None:
        side = str(hand).strip().lower()
        if side == "both":
            for each_hand in ("left", "right"):
                self._get_hand(each_hand).release_fingers(
                    hold_s=hold_s,
                    rate_hz=rate_hz,
                    persistent=persistent,
                )
            return
        self._get_hand(side).release_fingers(hold_s=hold_s, rate_hz=rate_hz, persistent=persistent)

    def stop_release_fingers(self, hand: str = "both") -> None:
        side = str(hand).strip().lower()
        if side == "both":
            for each_hand in ("left", "right"):
                self._get_hand(each_hand).stop_release_fingers()
            return
        self._get_hand(side).stop_release_fingers()

    def hand_pose(
        self,
        targets: list[float],
        hand: str = "right",
        hold_s: float = 0.6,
        rate_hz: float = 50.0,
        kp: float = 1.2,
        kd: float = 0.05,
        tau: float = 0.05,
        ramp_s: float | None = None,
    ) -> None:
        self._get_hand(hand).set_targets(
            targets,
            hold_s=hold_s,
            rate_hz=rate_hz,
            kp=kp,
            kd=kd,
            tau=tau,
            ramp_s=ramp_s,
        )

    def hand_move_finger(
        self,
        finger_name: str,
        hand: str = "right",
        hold_s: float = 1.0,
        settle_s: float = 0.6,
        rate_hz: float = 50.0,
    ) -> None:
        self._get_hand(hand).move_finger(finger_name, hold_s=hold_s, settle_s=settle_s, rate_hz=rate_hz)

    def _get_latest_lidar_cloud_msg(self) -> tuple[PointCloud2_ | None, str | None, float]:
        with self._lock:
            best_topic = None
            best_msg = None
            best_ts = 0.0
            for topic in self.lidar_cloud_topics:
                ts = float(self._last_lidar_cloud_ts_by_topic.get(topic, 0.0))
                msg = self._lidar_cloud_by_topic.get(topic)
                if msg is not None and ts >= best_ts:
                    best_topic = topic
                    best_msg = msg
                    best_ts = ts
        return best_msg, best_topic, best_ts

    @staticmethod
    def _series_value(values: list[float], index: int) -> float | None:
        if index < 0 or index >= len(values):
            return None
        return float(values[index])

    def _get_hand_state_msg(self, hand: str) -> tuple[Any | None, float]:
        sub = self._hand_state_subs.get(str(hand).strip().lower())
        if sub is None:
            return None, 0.0
        return sub.get_latest()

    @staticmethod
    def _extract_hand_joint_series(msg: Any) -> tuple[list[float | None], list[float | None], list[float | None]]:
        positions: list[float | None] = []
        velocities: list[float | None] = []
        torques: list[float | None] = []
        motor_state = list(getattr(msg, "motor_state", []) or [])
        for idx in range(7):
            motor = motor_state[idx] if idx < len(motor_state) else None
            try:
                positions.append(float(getattr(motor, "q")))
            except Exception:
                positions.append(None)
            try:
                velocities.append(float(getattr(motor, "dq")))
            except Exception:
                velocities.append(None)
            try:
                torques.append(float(getattr(motor, "tau_est")))
            except Exception:
                torques.append(None)
        return positions, velocities, torques

    def _resolve_joint_lookup_key(self, joint_index: int | str) -> str | None:
        if isinstance(joint_index, str):
            key = joint_index.strip()
            if key in BODY_JOINT_INDEX_BY_NAME:
                return key
            if key.startswith("left_hand.") or key.startswith("right_hand."):
                return key
            if key in HAND_JOINT_NAMES:
                return f"right_hand.{key}"
            return None
        idx = int(joint_index)
        return BODY_JOINT_NAME_BY_INDEX.get(idx)

    def get_joint_states(self) -> dict[str, Any] | None:
        snap = self.get_low_state_snapshot()
        if snap is None:
            return None

        joints: dict[str, dict[str, float | None | str]] = {}
        for group, index, name in BODY_JOINT_LAYOUT:
            label = f"{group}.{name}"
            joints[label] = {
                "position": self._series_value(snap.joint_positions, index),
                "velocity": self._series_value(snap.joint_velocities, index),
                "torque": self._series_value(snap.joint_torques, index),
                "source": "lowstate",
                "group": group,
            }

        sources: dict[str, Any] = {"body": "rt/lowstate", "hands": {}}
        timestamp = float(snap.stamp)
        for side in ("left", "right"):
            hand_msg, hand_ts = self._get_hand_state_msg(side)
            if hand_msg is None:
                continue
            positions, velocities, torques = self._extract_hand_joint_series(hand_msg)
            for idx, joint_name in enumerate(HAND_JOINT_NAMES):
                label = f"{side}_hand.{joint_name}"
                joints[label] = {
                    "position": positions[idx],
                    "velocity": velocities[idx],
                    "torque": torques[idx],
                    "source": HAND_STATE_TOPIC_BY_SIDE[side],
                    "group": f"{side}_hand",
                }
            sources["hands"][side] = HAND_STATE_TOPIC_BY_SIDE[side]
            timestamp = max(timestamp, float(hand_ts))

        imu = {
            "rpy": snap.imu_rpy,
            "gyro": snap.imu_gyro,
            "acc": snap.imu_acc,
        }
        return {
            "timestamp": timestamp,
            "imu": imu,
            "joints": joints,
            "sources": sources,
        }

    def _recv_rgbd_payload(self, timeout: float = 2.0) -> tuple[bytes, bytes, float, float]:
        try:
            import zmq
        except Exception as exc:
            raise RuntimeError(f"RGBD access requires pyzmq: {exc}") from exc

        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, self.rgbd_topic.encode("utf-8"))
        socket.setsockopt(zmq.RCVTIMEO, 250)
        socket.connect(f"tcp://{self.rgbd_host}:{self.rgbd_port}")
        deadline = time.time() + max(0.2, float(timeout))
        try:
            time.sleep(0.1)
            while time.time() < deadline:
                try:
                    parts = socket.recv_multipart()
                except zmq.Again:
                    continue
                if len(parts) >= 4:
                    parts = parts[-3:]
                if len(parts) < 2:
                    continue
                rgb_jpeg = bytes(parts[0])
                depth_png = bytes(parts[1])
                depth_scale = 0.001
                if len(parts) >= 3 and len(parts[2]) >= 4:
                    try:
                        depth_scale = float(struct.unpack("f", parts[2][:4])[0])
                    except Exception:
                        depth_scale = 0.001
                return rgb_jpeg, depth_png, depth_scale, time.time()
        finally:
            try:
                socket.close(0)
                context.term()
            except Exception:
                pass
        raise RuntimeError(f"No RGBD frames received from tcp://{self.rgbd_host}:{self.rgbd_port} within {timeout:.1f}s.")


__all__ = ["Robot", "ImuData"]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Smoke test for sdk_client Robot wrapper")
    parser.add_argument("--iface", default="eth0")
    parser.add_argument(
        "--robot-ip",
        "--rgbd-host",
        dest="rgbd_host",
        default=DEFAULT_RGBD_HOST,
        help="Robot RGBD publisher IP/host.",
    )
    parser.add_argument(
        "--safety-boot",
        action="store_true",
        help="Run the hanged safety boot sequence during initialization.",
    )
    args = parser.parse_args()

    bot = Robot(iface=args.iface, safety_boot=args.safety_boot, rgbd_host=args.rgbd_host)
    time.sleep(0.6)
    print("FSM:", bot.get_fsm())
    print("IMU:", bot.get_imu())
