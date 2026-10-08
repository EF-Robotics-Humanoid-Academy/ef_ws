"""
sdk_client.py
=============

SDK-native Robot wrapper for Unitree G1.

The implementation is intentionally local to `modules/` and avoids imports
from `../scripts`. Script-backed workflows were replaced with direct SDK
helpers or removed from the core wrapper.
"""
from __future__ import annotations

import base64
import csv
import importlib
import json
import math
import numpy as np
import os
import pickle
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Optional

from dds_env import (
    default_dds_iface,
    ensure_channel_factory_initialized,
    ensure_cyclonedds_environment,
)
from sdk_audio import RobotAudio
from sdk_boot import create_loco_client, rpc_get_int
from sdk_hand import Dex3HandController
from secure_boot import secure_boot
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
DEFAULT_SLAM_POINTS_TOPIC = "rt/unitree/slam_mapping/points"
DEFAULT_SLAM_RELOCATION_POINTS_TOPIC = "rt/unitree/slam_relocation/points"
DEFAULT_LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_deskewed"
DEFAULT_LIDAR_CLOUD_FALLBACK_TOPIC = "rt/utlidar/cloud_livox_mid360"
DEFAULT_RGBD_HOST = os.environ.get("G1_RGBD_HOST", "10.34.0.83")
DEFAULT_RGBD_PORT = int(os.environ.get("G1_RGBD_PORT", "5555"))
DEFAULT_RGBD_TOPIC = os.environ.get("G1_RGBD_TOPIC", "")
DEFAULT_OLLAMA_URL = "http://127.0.0.1:11434"
DEFAULT_CHAT_MODEL = "qwen3.5:9b"
DEFAULT_VISION_MODEL = "qwen2.5vl:7b"
VISION_MODEL_CANDIDATES = (
    "qwen2.5vl:7b",
    "qwen2.5vl",
    "llava:latest",
    "llava",
    "minicpm-v:latest",
    "minicpm-v",
)
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
G1_NUM_MOTOR = 29
LEFT_LEG_JOINTS = [0, 1, 2, 3, 4, 5]
RIGHT_LEG_JOINTS = [6, 7, 8, 9, 10, 11]
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
WAIST_JOINTS = [12, 13, 14]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
LEG_JOINTS = LEFT_LEG_JOINTS + RIGHT_LEG_JOINTS
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
BODY_JOINTS = list(range(G1_NUM_MOTOR))
ARM_SDK_NOT_USED_IDX = 29
WAIST_HOLD_KP = 480.0
WAIST_HOLD_KD = 12.0
LOWCMD_DEFAULT_KP = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
]
LOWCMD_DEFAULT_KD = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]
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
PBD_ARM_JOINTS = {
    "left": list(LEFT_ARM_JOINTS),
    "right": list(RIGHT_ARM_JOINTS),
    "both": list(LEFT_ARM_JOINTS) + list(RIGHT_ARM_JOINTS),
}
PBD_HAND_JOINT_LABELS = {
    "left": [f"left_hand.{name}" for name in HAND_JOINT_NAMES],
    "right": [f"right_hand.{name}" for name in HAND_JOINT_NAMES],
}


def _decode_depth_scale(payload: bytes, default: float = 0.001) -> float:
    """Decode one little-endian float from RGB-D metadata safely."""
    if len(payload) < 4:
        return default
    try:
        scale = float(struct.unpack("<f", payload[:4])[0])
    except (struct.error, TypeError, ValueError):
        return default
    return scale if math.isfinite(scale) and scale > 0.0 else default


def _normalize_arm_selection(arm: str) -> str:
    side = str(arm).strip().lower()
    if side not in PBD_ARM_JOINTS:
        raise ValueError("arm must be 'left', 'right', or 'both'.")
    return side


def _load_pbd_motion_file(path: str) -> dict[str, np.ndarray]:
    if not path:
        raise ValueError("motion file path is empty")
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    ext = os.path.splitext(path)[1].lower()
    if ext == ".npz":
        with np.load(path, allow_pickle=True) as data:
            return {k: np.asarray(data[k]) for k in data.files}
    if ext == ".csv":
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                raise ValueError(f"CSV has no header: {path}")
            ts_key = next((k for k in ("t_s", "ts", "time_s", "time")
                          if k in reader.fieldnames), None)
            if ts_key is None:
                raise ValueError(f"CSV must include one time column (t_s/ts/time_s/time): {path}")
            joint_cols: list[tuple[int, str]] = []
            for name in reader.fieldnames:
                match = re.fullmatch(r"j(\d+)", str(name).strip().lower())
                if match:
                    joint_cols.append((int(match.group(1)), name))
            if not joint_cols:
                raise ValueError(f"CSV must include joint columns like j22,j23,...: {path}")
            ts_vals: list[float] = []
            q_rows: list[list[float]] = []
            for row in reader:
                if not row:
                    continue
                raw_t = row.get(ts_key)
                if raw_t is None or str(raw_t).strip() == "":
                    continue
                ts_vals.append(float(raw_t))
                q_rows.append([float(row[col_name]) for _, col_name in joint_cols])
            if not ts_vals or not q_rows:
                raise ValueError(f"CSV has no data rows: {path}")
            return {
                "joints": np.asarray([joint for joint, _ in joint_cols], dtype=int),
                "ts": np.asarray(ts_vals, dtype=float),
                "qs": np.asarray(q_rows, dtype=float),
            }
    if ext in (".pkl", ".pickle"):
        with open(path, "rb") as handle:
            obj = pickle.load(handle)
        if not isinstance(obj, dict):
            raise ValueError(f"Pickle motion file must contain a dict, got: {type(obj).__name__}")
        return {str(k): np.asarray(v) for k, v in obj.items()}
    raise ValueError(
        f"Unsupported motion file format for '{path}'. "
        "Use .npz, .csv (t_s + jXX columns), or .pkl/.pickle dict."
    )


def _interp_motion_row(ts: np.ndarray, qs: np.ndarray, t: float) -> np.ndarray:
    if t <= float(ts[0]):
        return qs[0]
    if t >= float(ts[-1]):
        return qs[-1]
    hi = int(np.searchsorted(ts, t, side="right"))
    lo = max(0, hi - 1)
    t0 = float(ts[lo])
    t1 = float(ts[hi])
    if t1 <= t0:
        return qs[hi]
    alpha = (t - t0) / (t1 - t0)
    return qs[lo] * (1.0 - alpha) + qs[hi] * alpha


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
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        ensure_channel_factory_initialized(int(domain_id), str(iface))
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


class _LowCmdPublisher:
    def __init__(self, iface: str, domain_id: int) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        ensure_channel_factory_initialized(int(domain_id), str(iface))
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0

    def publish_targets(
        self,
        joint_targets: dict[int, float],
        *,
        mode_machine: int = 0,
        kp: float | dict[int, float] = 40.0,
        kd: float | dict[int, float] = 1.0,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine)
        for motor_idx in range(G1_NUM_MOTOR):
            mc = self._cmd.motor_cmd[motor_idx]
            mc.mode = 0
            mc.q = 0.0
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        for joint_index, target in joint_targets.items():
            idx = int(joint_index)
            if idx < 0 or idx >= G1_NUM_MOTOR:
                raise ValueError(f"Invalid body joint index for rt/lowcmd: {idx}")
            mc = self._cmd.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(target)
            mc.dq = float(dq)
            mc.tau = float(tau)
            mc.kp = float(kp.get(idx, 40.0) if isinstance(kp, dict) else kp)
            mc.kd = float(kd.get(idx, 1.0) if isinstance(kd, dict) else kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class Robot:
    """End-user wrapper around common G1 SDK workflows."""

    def __init__(
        self,
        iface: str | None = None,
        domain_id: int = 0,
        safety_boot: bool = False,
        recover_dev_mode_on_init: bool = True,
        auto_start_sensors: bool = True,
        sport_topic: str = DEFAULT_SPORT_TOPIC,
        lidar_map_topic: str = DEFAULT_LIDAR_MAP_TOPIC,
        lidar_cloud_topic: str = DEFAULT_LIDAR_CLOUD_TOPIC,
        slam_points_topic: str = DEFAULT_SLAM_POINTS_TOPIC,
        slam_info_topic: str = "rt/slam_info",
        slam_key_topic: str = "rt/slam_key_info",
        rgbd_host: str = DEFAULT_RGBD_HOST,
        rgbd_port: int = DEFAULT_RGBD_PORT,
        rgbd_topic: str = DEFAULT_RGBD_TOPIC,
        ollama_url: str = DEFAULT_OLLAMA_URL,
        chat_model: str = DEFAULT_CHAT_MODEL,
        vision_model: str = DEFAULT_VISION_MODEL,
    ) -> None:
        self.iface = str(iface) if iface is not None else default_dds_iface("eth0")
        self.domain_id = int(domain_id)
        self.sport_topic = sport_topic
        self.lidar_map_topic = lidar_map_topic
        self.lidar_cloud_topic = lidar_cloud_topic
        self.slam_points_topic = str(slam_points_topic)
        self.slam_info_topic = slam_info_topic
        self.slam_key_topic = slam_key_topic
        self.rgbd_host = str(rgbd_host)
        self.rgbd_port = int(rgbd_port)
        self.rgbd_topic = str(rgbd_topic)
        self.ollama_url = str(ollama_url).rstrip("/")
        self.chat_model = str(chat_model)
        self.vision_model = str(vision_model)
        self.point_cloud_topics = list(
            dict.fromkeys(
                [
                    self.slam_points_topic,
                    DEFAULT_SLAM_RELOCATION_POINTS_TOPIC,
                    str(self.lidar_cloud_topic),
                    DEFAULT_LIDAR_CLOUD_FALLBACK_TOPIC,
                    DEFAULT_LIDAR_CLOUD_TOPIC,
                ]
            )
        )
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
            topic: None for topic in self.point_cloud_topics
        }
        self._last_sport_ts = 0.0
        self._last_lidar_map_ts = 0.0
        self._last_lidar_cloud_ts = 0.0
        self._last_lidar_cloud_ts_by_topic: dict[str, float] = {
            topic: 0.0 for topic in self.point_cloud_topics
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
        self._lowcmd: _LowCmdPublisher | None = None
        self._robot_state_client: Any = None
        self._ai_sport_enabled_estimate: bool | None = None
        self._arm_action_client: Any = None
        self._hands: dict[str, Dex3HandController] = {}
        self._usb_controller_thread: threading.Thread | None = None
        self._usb_controller_stop = threading.Event()
        self._chat_process: subprocess.Popen[str] | None = None
        self.slam_is_running = False

        if safety_boot:
            self._client = secure_boot(iface=self.iface, domain_id=self.domain_id)
        else:
            self._client = create_loco_client(domain_id=self.domain_id, iface=self.iface)
            # if recover_dev_mode_on_init:
            # self.leave_dev_mode(restart_wait_s=1.0)

        if auto_start_sensors:
            self.start_sensors()

    @staticmethod
    def _motion_switcher_result_code(result: Any) -> Any:
        if isinstance(result, tuple):
            return result[0]
        return result

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

    def _get_lowcmd(self) -> _LowCmdPublisher:
        if self._lowcmd is None:
            self._lowcmd = _LowCmdPublisher(iface=self.iface, domain_id=self.domain_id)
        return self._lowcmd

    def _get_robot_state_client(self, *, timeout: float = 3.0) -> Any:
        if self._robot_state_client is not None:
            if hasattr(self._robot_state_client, "SetTimeout"):
                self._robot_state_client.SetTimeout(float(timeout))
            return self._robot_state_client

        errors: list[str] = []
        for module_name in (
            "unitree_sdk2py.b2.robot_state.robot_state_client",
            "unitree_sdk2py.go2.robot_state.robot_state_client",
        ):
            try:
                module = importlib.import_module(module_name)
                client_type = module.RobotStateClient
                break
            except ModuleNotFoundError as exc:
                errors.append(f"{module_name}: {exc}")
            except ImportError as exc:
                errors.append(f"{module_name}: {exc}")
        else:
            details = "\n  ".join(errors) if errors else "no candidate modules found"
            raise RuntimeError(f"RobotStateClient could not be imported:\n  {details}")

        ensure_channel_factory_initialized(self.domain_id, self.iface)
        client = client_type()
        if hasattr(client, "SetTimeout"):
            client.SetTimeout(float(timeout))
        client.Init()
        self._robot_state_client = client
        return client

    def _get_arm_action_client(self) -> Any:
        if self._arm_action_client is None:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

            self._arm_action_client = G1ArmActionClient()
            self._arm_action_client.SetTimeout(10.0)
            self._arm_action_client.Init()
        return self._arm_action_client

    def _ollama_request(
        self,
        path: str,
        body: dict[str, Any] | None = None,
        *,
        timeout: float = 30.0,
        method: str | None = None,
    ) -> dict[str, Any]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            f"{self.ollama_url}{path}",
            data=data,
            headers={"Content-Type": "application/json"} if body is not None else {},
            method=method or ("POST" if body is not None else "GET"),
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                raw = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Ollama HTTP {exc.code}: {detail}") from exc
        return json.loads(raw) if raw else {}

    def _ollama_ready(self, *, timeout: float = 1.5) -> bool:
        try:
            payload = self._ollama_request("/api/tags", timeout=timeout, method="GET")
        except Exception:
            return False
        return isinstance(payload, dict) and "models" in payload

    def _ensure_ollama_running(
        self,
        *,
        command: str = "ollama",
        start_timeout: float = 20.0,
        log_path: str = "/tmp/ollama_sdk_client_server.log",
    ) -> bool:
        if self._ollama_ready(timeout=1.5):
            return True
        if shutil.which(command) is None and not os.path.exists(command):
            return False
        with open(log_path, "a", encoding="utf-8") as log_handle:
            log_handle.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] starting ollama serve\n")
            log_handle.flush()
            proc = subprocess.Popen(
                [command, "serve"],
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        deadline = time.time() + max(1.0, float(start_timeout))
        while time.time() < deadline:
            if self._ollama_ready(timeout=1.0):
                return True
            if proc.poll() is not None:
                return False
            time.sleep(0.5)
        return False

    def _ollama_model_names(self) -> set[str]:
        payload = self._ollama_request("/api/tags", timeout=5.0, method="GET")
        models = payload.get("models", []) if isinstance(payload, dict) else []
        names: set[str] = set()
        for item in models:
            if isinstance(item, dict) and item.get("name"):
                names.add(str(item["name"]))
                names.add(str(item["name"]).split(":", 1)[0])
        return names

    @staticmethod
    def _ollama_model_available(model: str, names: set[str]) -> bool:
        selected = str(model)
        return selected in names or selected.split(":", 1)[0] in names

    def _select_available_vision_model(self, names: set[str]) -> str | None:
        candidates = (self.vision_model, DEFAULT_VISION_MODEL, *VISION_MODEL_CANDIDATES)
        seen: set[str] = set()
        for candidate in candidates:
            selected = str(candidate)
            if selected in seen:
                continue
            seen.add(selected)
            if self._ollama_model_available(selected, names):
                return selected
        return None

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
        for topic in self.point_cloud_topics:
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

    def get_mic(
        self,
        topic: str = "/audio_msg",
        *,
        duration_s: float = 5.0,
        max_messages: int | None = None,
        print_messages: bool = True,
        use_cli: bool = True,
    ) -> list[dict[str, Any]]:
        """Listen to the ROS ASR/microphone topic and return received messages.

        The default topic matches chat.py. Messages are stored as dictionaries
        with timestamp, topic, raw text, and parsed JSON payload when available.
        """
        if use_cli:
            return self._get_mic_via_ros2_cli(
                topic,
                duration_s=duration_s,
                max_messages=max_messages,
                print_messages=print_messages,
            )

        old_ros_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.node import Node
            from std_msgs.msg import String
        except Exception as exc:
            raise RuntimeError(f"get_mic requires ROS 2 rclpy and std_msgs: {exc}") from exc

        context = Context()
        node: Any = None
        messages: list[dict[str, Any]] = []

        def _callback(msg: String) -> None:
            raw = str(msg.data)
            parsed: Any
            try:
                parsed = json.loads(raw)
            except Exception:
                parsed = None
            record = {
                "timestamp": time.time(),
                "topic": topic,
                "raw": raw,
                "payload": parsed,
                "text": str(parsed.get("text", "")) if isinstance(parsed, dict) else raw,
            }
            messages.append(record)
            if print_messages:
                print(f"[{topic}] {raw}")

        try:
            rclpy.init(context=context)
            node = Node("robot_sdk_mic_listener", context=context)
            node.create_subscription(String, topic, _callback, 10)
            deadline = time.time() + max(0.0, float(duration_s))
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if max_messages is not None and len(messages) >= int(max_messages):
                    break
        except Exception as exc:
            print(f"rclpy get_mic failed ({exc}); falling back to `ros2 topic echo`.")
            return self._get_mic_via_ros2_cli(
                topic,
                duration_s=duration_s,
                max_messages=max_messages,
                print_messages=print_messages,
                rclpy_error=str(exc),
            )
        finally:
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
            if old_ros_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = old_ros_domain
        return messages

    def _get_mic_via_ros2_cli(
        self,
        topic: str,
        *,
        duration_s: float,
        max_messages: int | None,
        print_messages: bool,
        rclpy_error: str | None = None,
    ) -> list[dict[str, Any]]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.domain_id)
        env.pop("CYCLONEDDS_URI", None)
        env.pop("CYCLONEDDS_HOME", None)
        command = [
            "timeout",
            f"{max(0.1, float(duration_s))}s",
            "ros2",
            "topic",
            "echo",
            "--qos-reliability",
            "reliable",
            "--full-length",
            str(topic),
            "std_msgs/msg/String",
        ]
        proc = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
        )
        messages: list[dict[str, Any]] = []
        count = 0
        blocks = proc.stdout.split("---")
        for block in blocks:
            lines = [line.strip() for line in block.splitlines() if line.strip()]
            if not lines:
                continue
            data_lines = [line for line in lines if line.startswith("data:")]
            line = data_lines[0].split(":", 1)[1].strip() if data_lines else " ".join(lines)
            if line.startswith("'") and line.endswith("'"):
                line = line[1:-1]
            if line.startswith('"') and line.endswith('"'):
                line = line[1:-1]
            parsed: Any = None
            try:
                parsed = json.loads(line)
            except Exception:
                parsed = None
            record = {
                "timestamp": time.time(),
                "topic": topic,
                "raw": line,
                "payload": parsed,
                "text": str(parsed.get("text", "")) if isinstance(parsed, dict) else line,
                "source": "ros2 topic echo",
            }
            if rclpy_error:
                record["rclpy_error"] = rclpy_error
            messages.append(record)
            count += 1
            if print_messages:
                print(f"[{topic}] {line}")
            if max_messages is not None and count >= int(max_messages):
                break
        return messages

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
        joint_positions = [entry["position"]
                           for entry in joint_state["joints"].values() if entry["position"] is not None]
        joint_velocities = [entry["velocity"]
                            for entry in joint_state["joints"].values() if entry["velocity"] is not None]
        joint_torques = [entry["torque"]
                         for entry in joint_state["joints"].values() if entry["torque"] is not None]
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
        timeout: float = 10.0,
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

    def _read_lowcmd_state_or_raise(
        self,
        *,
        timeout: float = 10.0,
    ) -> tuple[dict[int, float], int]:
        if not self.wait_for_low_state(timeout=max(0.1, float(timeout))):
            raise TimeoutError("Timed out waiting for rt/lowstate joint positions.")
        msg = self.get_low_state_msg()
        snapshot = self.get_low_state_snapshot()
        if msg is None or snapshot is None:
            raise RuntimeError("rt/lowstate is unavailable.")
        if len(snapshot.joint_positions) < G1_NUM_MOTOR:
            raise RuntimeError(
                f"rt/lowstate has {len(snapshot.joint_positions)} joints, expected at least {G1_NUM_MOTOR}."
            )
        mode_machine = getattr(msg, "mode_machine", 0)
        try:
            mode_machine_int = int(mode_machine)
        except Exception:
            mode_machine_int = 0
        return {
            joint_index: float(snapshot.joint_positions[joint_index])
            for joint_index in BODY_JOINTS
        }, mode_machine_int

    @staticmethod
    def _smoothstep(ratio: float) -> float:
        x = max(0.0, min(1.0, float(ratio)))
        return x * x * (3.0 - 2.0 * x)

    @staticmethod
    def _resolve_body_joint_selection(joints: Any = "arms", *, arm: str = "both") -> list[int]:
        groups = {
            "left_leg": LEFT_LEG_JOINTS,
            "left leg": LEFT_LEG_JOINTS,
            "right_leg": RIGHT_LEG_JOINTS,
            "right leg": RIGHT_LEG_JOINTS,
            "legs": LEG_JOINTS,
            "leg": LEG_JOINTS,
            "waist": WAIST_JOINTS,
            "left_arm": LEFT_ARM_JOINTS,
            "left arm": LEFT_ARM_JOINTS,
            "right_arm": RIGHT_ARM_JOINTS,
            "right arm": RIGHT_ARM_JOINTS,
            "arms": list(PBD_ARM_JOINTS[_normalize_arm_selection(arm)]),
            "arm": list(PBD_ARM_JOINTS[_normalize_arm_selection(arm)]),
            "upper_body": UPPER_BODY_JOINTS,
            "upper body": UPPER_BODY_JOINTS,
            "body": BODY_JOINTS,
            "all": BODY_JOINTS,
        }
        if isinstance(joints, str):
            full_key = str(joints).strip().lower().replace("-", "_")
            if full_key in groups:
                return list(groups[full_key])
            raw_items = [item.strip() for item in re.split(r"[,\s]+", joints) if item.strip()]
            if len(raw_items) == 1:
                key = raw_items[0].lower().replace("-", "_")
                if key in groups:
                    return list(groups[key])
            items: list[Any] = raw_items
        else:
            try:
                items = list(joints)
            except TypeError:
                items = [joints]

        resolved: list[int] = []
        for item in items:
            if isinstance(item, str):
                token = item.strip()
                key = token.lower().replace("-", "_")
                if key in groups:
                    resolved.extend(groups[key])
                    continue
                if key in BODY_JOINT_INDEX_BY_NAME:
                    resolved.append(BODY_JOINT_INDEX_BY_NAME[key])
                    continue
                if key.startswith("j") and key[1:].isdigit():
                    resolved.append(int(key[1:]))
                    continue
                if key.isdigit():
                    resolved.append(int(key))
                    continue
                raise ValueError(f"Unknown body joint selector: {item!r}")
            resolved.append(int(item))

        unique = list(dict.fromkeys(resolved))
        invalid = [joint_index for joint_index in unique if joint_index < 0 or joint_index >= G1_NUM_MOTOR]
        if invalid:
            raise ValueError(f"Invalid body joint indices: {invalid}")
        if not unique:
            raise ValueError("At least one body joint must be selected.")
        return unique

    @staticmethod
    def _default_lowcmd_gains(joint_indices: list[int]) -> tuple[dict[int, float], dict[int, float]]:
        return (
            {joint_index: float(LOWCMD_DEFAULT_KP[joint_index]) for joint_index in joint_indices},
            {joint_index: float(LOWCMD_DEFAULT_KD[joint_index]) for joint_index in joint_indices},
        )

    def _service_status(self, service_name: str, *, timeout: float = 3.0) -> int | None:
        client = self._get_robot_state_client(timeout=timeout)
        if not hasattr(client, "ServiceList"):
            return None
        code, service_states = client.ServiceList()
        if int(code) != 0:
            raise RuntimeError(f"ServiceList failed: code={int(code)}")
        for state in service_states or []:
            name = str(getattr(state, "name", "")).strip()
            if name != service_name:
                continue
            status = getattr(state, "status", None)
            try:
                return None if status is None else int(status)
            except Exception:
                return None
        return None

    def switch_service(self, service_name: str, enabled: bool, *, timeout: float = 3.0) -> int:
        """Switch a robot service using the robot_state ServiceSwitch API."""
        client = self._get_robot_state_client(timeout=timeout)
        if not hasattr(client, "ServiceSwitch"):
            raise AttributeError("RobotStateClient does not support ServiceSwitch().")
        return int(client.ServiceSwitch(str(service_name), bool(enabled)))

    def _set_ai_sport_service(self, enabled: bool, *, timeout: float = 10.0, wait: bool = True) -> int:
        if self._ai_sport_enabled_estimate == bool(enabled):
            return 0
        expected_status = 0 if enabled else 1
        try:
            current_status = self._service_status("ai_sport", timeout=timeout)
        except Exception:
            current_status = None
        if current_status == expected_status:
            self._ai_sport_enabled_estimate = bool(enabled)
            return 0

        code = self.switch_service("ai_sport", bool(enabled), timeout=timeout)
        if code != 0:
            try:
                current_status = self._service_status("ai_sport", timeout=timeout)
            except Exception:
                current_status = None
            if current_status == expected_status:
                self._ai_sport_enabled_estimate = bool(enabled)
                return 0
            raise RuntimeError(f"ServiceSwitch('ai_sport', {bool(enabled)}) failed: code={code}")
        if not wait:
            self._ai_sport_enabled_estimate = bool(enabled)
            return code
        deadline = time.time() + max(0.0, float(timeout))
        while time.time() < deadline:
            status = self._service_status("ai_sport", timeout=timeout)
            if status is None or status == expected_status:
                self._ai_sport_enabled_estimate = bool(enabled)
                return code
            time.sleep(0.1)
        raise TimeoutError(
            f"Timed out waiting for ai_sport service to turn {'on' if enabled else 'off'}."
        )

    def enter_lowcmd_dev_mode(self, *, timeout: float = 10.0) -> int:
        """Enter lowcmd developer mode by turning the ai_sport service off."""
        return self._set_ai_sport_service(False, timeout=timeout)

    def leave_lowcmd_dev_mode(self, *, timeout: float = 10.0) -> int:
        """Leave lowcmd developer mode by turning the ai_sport service on."""
        return self._set_ai_sport_service(True, timeout=timeout)


    def _read_upper_body_hold_pose(self, *, timeout: float = 3.0) -> dict[int, float]:
        """Capture the live upper-body pose to hold during arm_sdk handoff."""
        return self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)

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
        positions = self._read_upper_body_hold_pose(timeout=timeout)
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
        """Re-enable DDS arm_sdk control while holding the current pose.

        The pose is captured from `rt/lowstate` before the ramp starts, then
        published unchanged while arm_sdk authority returns.
        """
        positions = self._read_upper_body_hold_pose(timeout=timeout)
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

    def hold_arm_pose(
        self,
        arm_targets: dict[int, float],
        *,
        speed_rad_s: float = 0.2,
        max_step_rad: float = 0.2,
        command_rate_hz: float = 50.0,
        kp: float = 30.0,
        kd: float = 1.5,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        tolerance_rad: float = 0.002,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        """Ramp the given arm joints to arm_targets and hold there.

        Joints not present in arm_targets (e.g. the waist) stay at their
        current reading. Assumes arm_sdk authority is already engaged
        (see unrelease_arms()/_ensure_arm_authority). Mirrors the ramp in
        WBC/to_stable_hold.py so speed/step limits behave the same way.
        """
        start = self._read_upper_body_hold_pose(timeout=timeout)
        target = dict(start)
        target.update(arm_targets)
        rate_hz = max(1.0, float(command_rate_hz))
        dt = 1.0 / rate_hz
        per_tick_delta = min(max(0.001, float(max_step_rad)), max(0.001, float(speed_rad_s)) / rate_hz)
        arm_sdk = self._get_arm_sdk()
        waist_gains = {joint_index: float(waist_kp) for joint_index in WAIST_JOINTS}
        waist_damping = {joint_index: float(waist_kd) for joint_index in WAIST_JOINTS}
        current = dict(start)
        steps = 0
        while max(abs(float(target[j]) - float(current[j])) for j in target) > float(tolerance_rad):
            stepped = dict(current)
            for joint_index, target_q in target.items():
                cur_q = float(current[joint_index])
                delta = float(target_q) - cur_q
                if abs(delta) <= per_tick_delta:
                    stepped[joint_index] = float(target_q)
                else:
                    stepped[joint_index] = cur_q + math.copysign(per_tick_delta, delta)
            current = stepped
            arm_sdk.publish_targets(current, kp=kp, kd=kd, kp_by_joint=waist_gains, kd_by_joint=waist_damping)
            time.sleep(dt)
            steps += 1
        return {"steps": steps, "joint_count": len(target)}

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

    def _publish_pbd_teach_hold(
        self,
        arm_joints: list[int],
        arm_positions: dict[int, float],
        waist_positions: dict[int, float],
        *,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
    ) -> None:
        targets = dict(waist_positions)
        arm_kp = {int(joint_index): 0.0 for joint_index in arm_joints}
        arm_kd = {int(joint_index): 0.0 for joint_index in arm_joints}
        for joint_index in arm_joints:
            targets[int(joint_index)] = float(arm_positions[int(joint_index)])
        waist_gains = {int(joint_index): float(waist_kp) for joint_index in waist_positions}
        waist_damping = {int(joint_index): float(waist_kd) for joint_index in waist_positions}
        self._get_arm_sdk().publish_targets(
            targets,
            kp=0.0,
            kd=0.0,
            kp_by_joint={**arm_kp, **waist_gains},
            kd_by_joint={**arm_kd, **waist_damping},
        )

    def _capture_current_upper_body_pose(
        self,
        *,
        duration_s: float = 0.6,
        command_rate_hz: float = 50.0,
        kp: float = 20.0,
        kd: float = 1.0,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        positions = self._read_upper_body_hold_pose(timeout=timeout)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        arm_sdk = self._get_arm_sdk()
        waist_gains = {joint_index: float(waist_kp) for joint_index in WAIST_JOINTS}
        waist_damping = {joint_index: float(waist_kd) for joint_index in WAIST_JOINTS}
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            smooth = ratio * ratio * (3.0 - 2.0 * ratio)
            arm_sdk.publish_targets(
                positions,
                kp=kp * smooth,
                kd=kd * smooth,
                kp_by_joint=waist_gains,
                kd_by_joint=waist_damping,
            )
            arm_sdk.publish_arm_sdk_weight(smooth)
            time.sleep(dt)
        return {
            "duration_s": float(duration_s),
            "command_rate_hz": float(command_rate_hz),
            "final_arm_sdk_weight": 1.0,
            "joint_count": len(positions),
        }

    def dev_mode_teach(
        self,
        *,
        joints: Any = "arms",
        arm: str = "both",
        out: str = "/tmp/pbd_motion_lowcmd.npz",
        log_path: str | None = None,
        duration_s: float = 0.0,
        poll_s: float = 0.01,
        record_hands: bool = True,
        include_legs_and_waist: bool = True,
        zero_after_teach_s: float = 0.2,
        ensure_dev_mode: bool = False,
        timeout: float = 10.0,
    ) -> dict[str, Any]:
        """Record a body joint demonstration while publishing zero-gain rt/lowcmd.

        `joints` may be a group name (`"arms"`, `"legs"`, `"waist"`,
        `"upper_body"`, `"all"`), a joint name, or a list of indices/names.
        By default this assumes `ai_sport` is already off. The leg and waist
        joints are also included so they are recorded with zero gains and
        replayed later. Leg joints are low-level only; use this with the robot
        externally supported.
        """
        selected_joints = self._resolve_body_joint_selection(joints, arm=arm)
        if include_legs_and_waist:
            selected_joints = sorted(dict.fromkeys(selected_joints + LEG_JOINTS + WAIST_JOINTS))
        done_event = threading.Event()

        def _wait_for_enter() -> None:
            try:
                input("Press Enter when the dev-mode teach motion is complete...")
            except EOFError:
                return
            done_event.set()

        if ensure_dev_mode:
            self.enter_lowcmd_dev_mode(timeout=timeout)
        self._get_lowcmd()
        positions, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
        selected_positions = {joint: positions[joint] for joint in selected_joints}
        self._get_lowcmd().publish_targets(
            selected_positions,
            mode_machine=mode_machine,
            kp={joint: 0.0 for joint in selected_joints},
            kd={joint: 0.0 for joint in selected_joints},
        )
        record_hands_enabled = bool(record_hands)
        hands_zero_torqued = False
        if record_hands_enabled:
            self.zero_torque_fingers("both")
            hands_zero_torqued = True
            left_hand_snapshot = self.get_hand_state_snapshot("left")
            right_hand_snapshot = self.get_hand_state_snapshot("right")
            if left_hand_snapshot is None or right_hand_snapshot is None:
                print("Hand state is unavailable; continuing body-only teach.")
                record_hands_enabled = False

        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        resolved_log_path = log_path or f"{os.path.splitext(out)[0]}.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        sample_period = max(1e-3, float(poll_s))
        duration_limit = max(0.0, float(duration_s))
        timestamps: list[float] = []
        samples: list[list[float]] = []
        left_hand_samples: list[list[float]] = []
        right_hand_samples: list[list[float]] = []
        start = time.time()
        next_tick = start
        prompt_thread = threading.Thread(
            target=_wait_for_enter,
            name="dev-mode-teach-enter-confirm",
            daemon=True,
        )
        prompt_thread.start()

        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["phase", "t_s", "joint_indices", "target_positions", "actual_positions"])
            try:
                while True:
                    now = time.time()
                    if now < next_tick:
                        time.sleep(min(0.02, next_tick - now))
                        continue
                    next_tick += sample_period
                    if done_event.is_set():
                        break
                    if duration_limit > 0.0 and (now - start) >= duration_limit:
                        break

                    positions, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
                    row = [float(positions[joint]) for joint in selected_joints]
                    zero_targets = {
                        joint: float(row[idx])
                        for idx, joint in enumerate(selected_joints)
                    }
                    self._get_lowcmd().publish_targets(
                        zero_targets,
                        mode_machine=mode_machine,
                        kp={joint: 0.0 for joint in selected_joints},
                        kd={joint: 0.0 for joint in selected_joints},
                    )

                    left_hand_row: list[float] = []
                    right_hand_row: list[float] = []
                    if record_hands_enabled:
                        left_hand_snapshot = self.get_hand_state_snapshot("left")
                        right_hand_snapshot = self.get_hand_state_snapshot("right")
                        if left_hand_snapshot is None or right_hand_snapshot is None:
                            record_hands_enabled = False
                        else:
                            left_hand_row = [
                                float(left_hand_snapshot["positions"][joint_name])
                                for joint_name in HAND_JOINT_NAMES
                            ]
                            right_hand_row = [
                                float(right_hand_snapshot["positions"][joint_name])
                                for joint_name in HAND_JOINT_NAMES
                            ]
                    if not record_hands_enabled:
                        left_hand_row = []
                        right_hand_row = []

                    t_rel = now - start
                    timestamps.append(t_rel)
                    samples.append(row)
                    if record_hands_enabled:
                        left_hand_samples.append(left_hand_row)
                        right_hand_samples.append(right_hand_row)
                    writer.writerow(
                        [
                            "dev_mode_teach",
                            f"{t_rel:.6f}",
                            " ".join(
                                [str(joint) for joint in selected_joints]
                                + (PBD_HAND_JOINT_LABELS["left"] + PBD_HAND_JOINT_LABELS["right"] if record_hands_enabled else [])
                            ),
                            " ".join(
                                [f"{value:.6f}" for value in row]
                                + ([f"{value:.6f}" for value in left_hand_row] + [f"{value:.6f}" for value in right_hand_row] if record_hands_enabled else [])
                            ),
                            " ".join(
                                [f"{value:.6f}" for value in row]
                                + ([f"{value:.6f}" for value in left_hand_row] + [f"{value:.6f}" for value in right_hand_row] if record_hands_enabled else [])
                            ),
                        ]
                    )
                    handle.flush()
                    print(
                        f"[dev_mode_teach] t={t_rel:.3f}s joints={selected_joints} "
                        f"actual={[round(value, 4) for value in row]}"
                    )
            except KeyboardInterrupt:
                pass

        if not timestamps:
            raise RuntimeError("No samples recorded. Is rt/lowstate publishing?")

        save_kwargs: dict[str, Any] = {
            "joints": np.asarray(selected_joints, dtype=np.int32),
            "joint_names": np.asarray([BODY_JOINT_NAME_BY_INDEX[joint] for joint in selected_joints], dtype="<U32"),
            "ts": np.asarray(timestamps, dtype=np.float32),
            "qs": np.asarray(samples, dtype=np.float32),
            "poll_s": np.asarray([sample_period], dtype=np.float32),
            "representation": np.asarray(["joint_space"], dtype="<U16"),
            "control_topic": np.asarray(["rt/lowcmd"], dtype="<U16"),
            "targeted_joints": np.asarray(["body"], dtype="<U16"),
        }
        if record_hands_enabled:
            save_kwargs.update(
                {
                    "left_hand_joints": np.asarray(PBD_HAND_JOINT_LABELS["left"], dtype="<U32"),
                    "right_hand_joints": np.asarray(PBD_HAND_JOINT_LABELS["right"], dtype="<U32"),
                    "left_hand_qs": np.asarray(left_hand_samples, dtype=np.float32),
                    "right_hand_qs": np.asarray(right_hand_samples, dtype=np.float32),
                }
            )
        np.savez(out, **save_kwargs)

        final_targets = {
            joint: float(samples[-1][idx])
            for idx, joint in enumerate(selected_joints)
        }
        zero_steps = max(0, int(max(0.0, float(zero_after_teach_s)) / sample_period))
        for _ in range(zero_steps + 1):
            _, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
            self._get_lowcmd().publish_targets(
                final_targets,
                mode_machine=mode_machine,
                kp={joint: 0.0 for joint in selected_joints},
                kd={joint: 0.0 for joint in selected_joints},
            )
            if zero_steps > 0:
                time.sleep(sample_period)
        if hands_zero_torqued:
            self.stop_release_fingers("both")
        return {
            "joint_count": len(selected_joints),
            "sample_count": len(timestamps),
            "duration_s": float(timestamps[-1]) if timestamps else 0.0,
            "poll_s": sample_period,
            "out": os.path.abspath(out),
            "log_path": os.path.abspath(resolved_log_path),
            "control_topic": "rt/lowcmd",
            "targeted_joints": list(selected_joints),
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint] for joint in selected_joints],
            "recorded_hands": bool(record_hands_enabled),
        }

    def dev_mode_repeat(
        self,
        *,
        motion_file: str = "/tmp/pbd_motion_lowcmd.npz",
        joints: Any | None = None,
        arm: str = "both",
        log_path: str | None = None,
        speed: float = 1.0,
        command_rate_hz: float = 50.0,
        start_ramp_s: float = 0.8,
        final_hold_s: float = 0.8,
        kp: float | None = None,
        kd: float | None = None,
        replay_hands: bool = True,
        zero_gains_on_exit: bool = False,
        ensure_dev_mode: bool = False,
        timeout: float = 10.0,
    ) -> dict[str, Any]:
        """Replay a saved body joint trajectory through rt/lowcmd.

        By default this assumes `ai_sport` is already off. Pass
        `ensure_dev_mode=True` only if this method should turn it off first.
        """
        data = _load_pbd_motion_file(motion_file)
        if "joints" not in data or "ts" not in data or "qs" not in data:
            raise ValueError("Motion file must contain 'joints', 'ts', and 'qs'.")
        recorded_joints = [int(joint) for joint in np.asarray(data["joints"]).astype(int).tolist()]
        ts = np.asarray(data["ts"], dtype=float)
        qs = np.asarray(data["qs"], dtype=float)
        if ts.size == 0 or qs.size == 0:
            raise ValueError("No samples in motion file.")
        if qs.shape[0] != ts.shape[0]:
            raise ValueError("Invalid motion file: ts and qs length mismatch.")
        if qs.shape[1] != len(recorded_joints):
            raise ValueError("Invalid motion file: joints and qs width mismatch.")

        selected_joints = (
            list(recorded_joints)
            if joints is None
            else self._resolve_body_joint_selection(joints, arm=arm)
        )
        joint_to_col = {joint: idx for idx, joint in enumerate(recorded_joints)}
        missing = [joint for joint in selected_joints if joint not in joint_to_col]
        if missing:
            raise ValueError(f"Motion file missing requested body joints: {missing}.")
        active_cols = [joint_to_col[joint] for joint in selected_joints]
        active_qs = qs[:, active_cols]
        left_hand_qs = np.asarray(data.get("left_hand_qs", np.empty((0, 7))), dtype=float)
        right_hand_qs = np.asarray(data.get("right_hand_qs", np.empty((0, 7))), dtype=float)
        use_hands = (
            replay_hands
            and left_hand_qs.shape == (ts.shape[0], 7)
            and right_hand_qs.shape == (ts.shape[0], 7)
        )

        if ensure_dev_mode:
            self.enter_lowcmd_dev_mode(timeout=timeout)
        self._get_lowcmd()
        resolved_log_path = log_path or f"{os.path.splitext(motion_file)[0]}_dev_repeat.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        rate_hz = max(1.0, float(command_rate_hz))
        dt = 1.0 / rate_hz
        replay_ts = ts / max(1e-6, float(speed))
        t_final = float(replay_ts[-1])
        start_positions, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
        first_targets = {
            joint: float(active_qs[0, idx])
            for idx, joint in enumerate(selected_joints)
        }
        kp_map, kd_map = self._default_lowcmd_gains(selected_joints)
        if kp is not None:
            kp_map = {joint: float(kp) for joint in selected_joints}
        if kd is not None:
            kd_map = {joint: float(kd) for joint in selected_joints}

        ramp_steps = max(1, int(max(0.0, float(start_ramp_s)) * rate_hz))
        if float(start_ramp_s) > 0.0:
            for step_idx in range(1, ramp_steps + 1):
                ratio = self._smoothstep(float(step_idx) / float(ramp_steps))
                targets = {
                    joint: float(start_positions[joint]) + (first_targets[joint] - float(start_positions[joint])) * ratio
                    for joint in selected_joints
                }
                _, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
                self._get_lowcmd().publish_targets(targets, mode_machine=mode_machine, kp=kp_map, kd=kd_map)
                time.sleep(dt)
        else:
            self._get_lowcmd().publish_targets(first_targets, mode_machine=mode_machine, kp=kp_map, kd=kd_map)

        if use_hands:
            self.stop_release_fingers("both")
            self._get_hand("left").set_targets(
                left_hand_qs[0].tolist(),
                hold_s=max(0.8, float(start_ramp_s)),
                rate_hz=rate_hz,
                kp=0.55,
                kd=0.05,
                tau=0.015,
                ramp_s=max(0.8, float(start_ramp_s)),
            )
            self._get_hand("right").set_targets(
                right_hand_qs[0].tolist(),
                hold_s=max(0.8, float(start_ramp_s)),
                rate_hz=rate_hz,
                kp=0.55,
                kd=0.05,
                tau=0.015,
                ramp_s=max(0.8, float(start_ramp_s)),
            )

        started = time.time()
        final_targets = first_targets
        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["phase", "t_s", "joint_indices", "target_positions", "actual_positions"])
            while True:
                elapsed = time.time() - started
                if elapsed > t_final:
                    break
                desired_row = np.asarray(_interp_motion_row(replay_ts, active_qs, elapsed), dtype=float)
                final_targets = {
                    joint: float(desired_row[idx])
                    for idx, joint in enumerate(selected_joints)
                }
                positions, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
                self._get_lowcmd().publish_targets(final_targets, mode_machine=mode_machine, kp=kp_map, kd=kd_map)
                if use_hands:
                    left_hand_desired = np.asarray(_interp_motion_row(replay_ts, left_hand_qs, elapsed), dtype=float)
                    right_hand_desired = np.asarray(_interp_motion_row(replay_ts, right_hand_qs, elapsed), dtype=float)
                    self._get_hand("left").write_targets_once(left_hand_desired.tolist(), kp=0.8, kd=0.05, tau=0.02)
                    self._get_hand("right").write_targets_once(right_hand_desired.tolist(), kp=0.8, kd=0.05, tau=0.02)
                actual_row = [float(positions[joint]) for joint in selected_joints]
                target_row = [float(final_targets[joint]) for joint in selected_joints]
                writer.writerow(
                    [
                        "dev_mode_repeat",
                        f"{elapsed:.6f}",
                        " ".join(str(joint) for joint in selected_joints),
                        " ".join(f"{value:.6f}" for value in target_row),
                        " ".join(f"{value:.6f}" for value in actual_row),
                    ]
                )
                handle.flush()
                print(
                    f"[dev_mode_repeat] t={elapsed:.3f}s joints={selected_joints} "
                    f"target={[round(value, 4) for value in target_row]} "
                    f"actual={[round(value, 4) for value in actual_row]}"
                )
                time.sleep(dt)

            hold_deadline = time.time() + max(0.0, float(final_hold_s))
            while time.time() < hold_deadline:
                positions, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
                self._get_lowcmd().publish_targets(final_targets, mode_machine=mode_machine, kp=kp_map, kd=kd_map)
                writer.writerow(
                    [
                        "dev_mode_repeat_final_hold",
                        f"{time.time() - started:.6f}",
                        " ".join(str(joint) for joint in selected_joints),
                        " ".join(f"{float(final_targets[joint]):.6f}" for joint in selected_joints),
                        " ".join(f"{float(positions[joint]):.6f}" for joint in selected_joints),
                    ]
                )
                handle.flush()
                time.sleep(dt)

        if zero_gains_on_exit:
            _, mode_machine = self._read_lowcmd_state_or_raise(timeout=timeout)
            self._get_lowcmd().publish_targets(
                final_targets,
                mode_machine=mode_machine,
                kp={joint: 0.0 for joint in selected_joints},
                kd={joint: 0.0 for joint in selected_joints},
            )
        return {
            "motion_file": os.path.abspath(motion_file),
            "joint_count": len(selected_joints),
            "sample_count": int(ts.shape[0]),
            "command_rate_hz": rate_hz,
            "speed": max(1e-6, float(speed)),
            "duration_s": t_final,
            "final_hold_s": max(0.0, float(final_hold_s)),
            "log_path": os.path.abspath(resolved_log_path),
            "control_topic": "rt/lowcmd",
            "targeted_joints": list(selected_joints),
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint] for joint in selected_joints],
            "replayed_hands": bool(use_hands),
        }

    def teach(
        self,
        *,
        arm: str = "both",
        out: str = "/tmp/pbd_motion.npz",
        log_path: str | None = None,
        duration_s: float = 0.0,
        poll_s: float = 0.01,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        side = _normalize_arm_selection(arm)
        arm_joints = list(PBD_ARM_JOINTS[side])
        done_event = threading.Event()

        def _wait_for_enter() -> None:
            try:
                input("Press Enter when the teach motion is complete...")
            except EOFError:
                return
            done_event.set()

        prompt_thread = threading.Thread(
            target=_wait_for_enter,
            name="teach-enter-confirm",
            daemon=True,
        )
        self.release_arms(timeout=timeout)
        self.unrelease_arms(timeout=timeout)
        waist_positions = self._read_joint_positions_or_raise(WAIST_JOINTS, timeout=timeout)
        arm_positions = self._read_joint_positions_or_raise(arm_joints, timeout=timeout)
        self._publish_pbd_teach_hold(
            arm_joints,
            arm_positions,
            waist_positions,
            waist_kp=waist_kp,
            waist_kd=waist_kd,
        )
        self.zero_torque_fingers("both")
        left_hand_snapshot = self.get_hand_state_snapshot("left")
        right_hand_snapshot = self.get_hand_state_snapshot("right")
        if left_hand_snapshot is None or right_hand_snapshot is None:
            raise RuntimeError("Hand state is unavailable. Are rt/dex3/*/state topics publishing?")

        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        resolved_log_path = log_path or f"{os.path.splitext(out)[0]}.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        sample_period = max(1e-3, float(poll_s))
        duration_limit = max(0.0, float(duration_s))
        timestamps: list[float] = []
        samples: list[list[float]] = []
        left_hand_samples: list[list[float]] = []
        right_hand_samples: list[list[float]] = []
        start = time.time()
        next_tick = start
        duration_notice_sent = False
        prompt_thread.start()

        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "phase",
                    "t_s",
                    "joint_indices",
                    "target_positions",
                    "actual_positions",
                ]
            )
            try:
                while True:
                    now = time.time()
                    if now < next_tick:
                        time.sleep(min(0.02, next_tick - now))
                        continue
                    next_tick += sample_period
                    if done_event.is_set():
                        break
                    if duration_limit > 0.0 and (now - start) >= duration_limit:
                        if not duration_notice_sent:
                            print("Teach duration limit reached. Press Enter to finish recording.")
                            duration_notice_sent = True

                    commanded_row = [float(arm_positions[int(joint_index)])
                                     for joint_index in arm_joints]
                    self._publish_pbd_teach_hold(
                        arm_joints,
                        arm_positions,
                        waist_positions,
                        waist_kp=waist_kp,
                        waist_kd=waist_kd,
                    )
                    snapshot = self.get_low_state_snapshot()
                    if snapshot is None:
                        continue
                    left_hand_snapshot = self.get_hand_state_snapshot("left")
                    right_hand_snapshot = self.get_hand_state_snapshot("right")
                    if left_hand_snapshot is None or right_hand_snapshot is None:
                        continue

                    row: list[float] = []
                    for joint_index in arm_joints:
                        q_val = snapshot.joint_positions[int(joint_index)]
                        if q_val is None:
                            raise RuntimeError(
                                f"Joint position for {BODY_JOINT_NAME_BY_INDEX.get(int(joint_index), joint_index)} is unavailable."
                            )
                        q_float = float(q_val)
                        arm_positions[int(joint_index)] = q_float
                        row.append(q_float)
                    left_hand_row = [
                        float(left_hand_snapshot["positions"][joint_name])
                        for joint_name in HAND_JOINT_NAMES
                    ]
                    right_hand_row = [
                        float(right_hand_snapshot["positions"][joint_name])
                        for joint_name in HAND_JOINT_NAMES
                    ]

                    t_rel = now - start
                    timestamps.append(t_rel)
                    samples.append(row)
                    left_hand_samples.append(left_hand_row)
                    right_hand_samples.append(right_hand_row)
                    writer.writerow(
                        [
                            "teach",
                            f"{t_rel:.6f}",
                            " ".join(
                                [str(joint_index) for joint_index in arm_joints]
                                + PBD_HAND_JOINT_LABELS["left"]
                                + PBD_HAND_JOINT_LABELS["right"]
                            ),
                            " ".join(
                                [f"{value:.6f}" for value in commanded_row]
                                + [f"{value:.6f}" for value in left_hand_row]
                                + [f"{value:.6f}" for value in right_hand_row]
                            ),
                            " ".join(
                                [f"{value:.6f}" for value in row]
                                + [f"{value:.6f}" for value in left_hand_row]
                                + [f"{value:.6f}" for value in right_hand_row]
                            ),
                        ]
                    )
                    handle.flush()
                    print(
                        f"[teach] t={t_rel:.3f}s joints={arm_joints + PBD_HAND_JOINT_LABELS['left'] + PBD_HAND_JOINT_LABELS['right']} "
                        f"target={[round(value, 4) for value in commanded_row]} "
                        f"actual={[round(value, 4) for value in row + left_hand_row + right_hand_row]}"
                    )
            except KeyboardInterrupt:
                pass

        if not timestamps:
            raise RuntimeError("No samples recorded. Is rt/lowstate publishing?")

        np.savez(
            out,
            joints=np.asarray(arm_joints, dtype=np.int32),
            ts=np.asarray(timestamps, dtype=np.float32),
            qs=np.asarray(samples, dtype=np.float32),
            left_hand_joints=np.asarray(PBD_HAND_JOINT_LABELS["left"], dtype="<U32"),
            right_hand_joints=np.asarray(PBD_HAND_JOINT_LABELS["right"], dtype="<U32"),
            left_hand_qs=np.asarray(left_hand_samples, dtype=np.float32),
            right_hand_qs=np.asarray(right_hand_samples, dtype=np.float32),
            poll_s=np.asarray([sample_period], dtype=np.float32),
            representation=np.asarray(["joint_space"], dtype="<U16"),
        )
        self.stop_release_fingers("both")
        self._capture_current_upper_body_pose(
            duration_s=0.8,
            command_rate_hz=max(50.0, 1.0 / max(1e-3, sample_period)),
            kp=18.0,
            kd=0.9,
            waist_kp=waist_kp,
            waist_kd=waist_kd,
            timeout=timeout,
        )
        self.release_arms(timeout=timeout)
        return {
            "arm": side,
            "joint_count": len(arm_joints),
            "sample_count": len(timestamps),
            "duration_s": float(timestamps[-1]) if timestamps else 0.0,
            "poll_s": sample_period,
            "out": os.path.abspath(out),
            "log_path": os.path.abspath(resolved_log_path),
            "waist_hold_joints": list(WAIST_JOINTS),
        }

    def repeat(
        self,
        *,
        motion_file: str = '/tmp/pbd_motion.npz',
        arm: str = "both",
        log_path: str | None = None,
        speed: float = 1.0,
        command_rate_hz: float = 50.0,
        start_ramp_s: float = 0.8,
        final_hold_s: float = 0.8,
        kp: float = 40.0,
        kd: float = 1.0,
        waist_kp: float = WAIST_HOLD_KP,
        waist_kd: float = WAIST_HOLD_KD,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        side = _normalize_arm_selection(arm)
        data = _load_pbd_motion_file(motion_file)
        if "joints" not in data or "ts" not in data or "qs" not in data:
            raise ValueError("Motion file must contain 'joints', 'ts', and 'qs'.")
        recorded_joints = [int(joint_index)
                           for joint_index in np.asarray(data["joints"]).astype(int).tolist()]
        ts = np.asarray(data["ts"], dtype=float)
        qs = np.asarray(data["qs"], dtype=float)
        if ts.size == 0 or qs.size == 0:
            raise ValueError("No samples in motion file.")
        if qs.shape[0] != ts.shape[0]:
            raise ValueError("Invalid motion file: ts and qs length mismatch.")
        if qs.shape[1] != len(recorded_joints):
            raise ValueError("Invalid motion file: joints and qs width mismatch.")

        requested_joints = list(PBD_ARM_JOINTS[side])
        joint_to_col = {joint_index: idx for idx, joint_index in enumerate(recorded_joints)}
        missing = [joint_index for joint_index in requested_joints if joint_index not in joint_to_col]
        if missing:
            raise ValueError(f"Motion file missing required joints for arm={side}: {missing}.")
        active_cols = [joint_to_col[joint_index] for joint_index in requested_joints]
        active_qs = qs[:, active_cols]
        left_hand_qs = np.asarray(data.get("left_hand_qs", np.empty((0, 7))), dtype=float)
        right_hand_qs = np.asarray(data.get("right_hand_qs", np.empty((0, 7))), dtype=float)
        replay_hands = left_hand_qs.shape == (
            ts.shape[0], 7) and right_hand_qs.shape == (ts.shape[0], 7)
        resolved_log_path = log_path or f"{os.path.splitext(motion_file)[0]}_repeat.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        self.release_arms(timeout=timeout)
        self.unrelease_arms(timeout=timeout)
        self._publish_pbd_teach_hold(
            requested_joints,
            self._read_joint_positions_or_raise(requested_joints, timeout=timeout),
            self._read_joint_positions_or_raise(WAIST_JOINTS, timeout=timeout),
            waist_kp=waist_kp,
            waist_kd=waist_kd,
        )
        self.zero_torque_fingers("both")
        self._capture_current_upper_body_pose(
            duration_s=max(0.6, min(1.2, float(start_ramp_s))),
            command_rate_hz=command_rate_hz,
            kp=min(20.0, float(kp) * 0.5),
            kd=min(1.0, float(kd)),
            waist_kp=waist_kp,
            waist_kd=waist_kd,
            timeout=timeout,
        )
        if replay_hands:
            self.stop_release_fingers("both")
            self._get_hand("left").set_targets(
                left_hand_qs[0].tolist(),
                hold_s=max(0.8, float(start_ramp_s)),
                rate_hz=command_rate_hz,
                kp=0.55,
                kd=0.05,
                tau=0.015,
                ramp_s=max(0.8, float(start_ramp_s)),
            )
            self._get_hand("right").set_targets(
                right_hand_qs[0].tolist(),
                hold_s=max(0.8, float(start_ramp_s)),
                rate_hz=command_rate_hz,
                kp=0.55,
                kd=0.05,
                tau=0.015,
                ramp_s=max(0.8, float(start_ramp_s)),
            )
        positions = self._read_joint_positions_or_raise(UPPER_BODY_JOINTS, timeout=timeout)
        first_targets = {
            int(joint_index): float(active_qs[0, idx])
            for idx, joint_index in enumerate(requested_joints)
        }
        current_targets = self._with_upper_body_hold(first_targets, positions)
        steps = max(1, int(max(0.0, float(start_ramp_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        if float(start_ramp_s) > 0.0:
            for step_idx in range(1, steps + 1):
                alpha = float(step_idx) / float(steps)
                blended = {}
                for joint_index, target in current_targets.items():
                    start_q = float(positions[int(joint_index)])
                    blended[int(joint_index)] = start_q + (float(target) - start_q) * alpha
                self._publish_with_upper_body_hold(
                    {joint_index: blended[joint_index] for joint_index in requested_joints},
                    blended,
                    kp=kp,
                    kd=kd,
                    waist_kp=waist_kp,
                    waist_kd=waist_kd,
                )
                time.sleep(dt)
        else:
            self._publish_with_upper_body_hold(
                first_targets,
                positions,
                kp=kp,
                kd=kd,
                waist_kp=waist_kp,
                waist_kd=waist_kd,
            )

        replay_ts = ts / max(1e-6, float(speed))
        t_final = float(replay_ts[-1])
        start = time.time()
        previous_desired_row = np.asarray(active_qs[0], dtype=float)
        commanded_targets = {
            int(joint_index): float(active_qs[0, idx])
            for idx, joint_index in enumerate(requested_joints)
        }
        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "phase",
                    "t_s",
                    "joint_indices",
                    "target_positions",
                    "actual_positions",
                ]
            )
            while True:
                elapsed = time.time() - start
                if elapsed > t_final:
                    break
                latest_positions = self._read_joint_positions_or_raise(
                    UPPER_BODY_JOINTS, timeout=timeout)
                desired_row = np.asarray(_interp_motion_row(
                    replay_ts, active_qs, elapsed), dtype=float)
                row_delta = desired_row - previous_desired_row
                for idx, joint_index in enumerate(requested_joints):
                    joint_key = int(joint_index)
                    commanded_targets[joint_key] = float(
                        commanded_targets[joint_key] + row_delta[idx])
                self._publish_with_upper_body_hold(
                    commanded_targets,
                    latest_positions,
                    kp=kp,
                    kd=kd,
                    waist_kp=waist_kp,
                    waist_kd=waist_kd,
                )
                if replay_hands:
                    left_hand_desired = np.asarray(_interp_motion_row(
                        replay_ts, left_hand_qs, elapsed), dtype=float)
                    right_hand_desired = np.asarray(_interp_motion_row(
                        replay_ts, right_hand_qs, elapsed), dtype=float)
                    self._get_hand("left").write_targets_once(
                        left_hand_desired.tolist(),
                        kp=0.8,
                        kd=0.05,
                        tau=0.02,
                    )
                    self._get_hand("right").write_targets_once(
                        right_hand_desired.tolist(),
                        kp=0.8,
                        kd=0.05,
                        tau=0.02,
                    )
                    left_hand_actual = self.get_hand_state_snapshot("left")
                    right_hand_actual = self.get_hand_state_snapshot("right")
                else:
                    left_hand_desired = right_hand_desired = None
                    left_hand_actual = right_hand_actual = None
                actual_row = [float(latest_positions[int(joint_index)])
                              for joint_index in requested_joints]
                target_row = [float(commanded_targets[int(joint_index)])
                              for joint_index in requested_joints]
                writer.writerow(
                    [
                        "repeat",
                        f"{elapsed:.6f}",
                        " ".join(
                            [str(joint_index) for joint_index in requested_joints]
                            + (PBD_HAND_JOINT_LABELS["left"] +
                               PBD_HAND_JOINT_LABELS["right"] if replay_hands else [])
                        ),
                        " ".join(
                            [f"{value:.6f}" for value in target_row]
                            + (
                                [f"{value:.6f}" for value in left_hand_desired.tolist()]
                                + [f"{value:.6f}" for value in right_hand_desired.tolist()]
                                if replay_hands
                                else []
                            )
                        ),
                        " ".join(
                            [f"{value:.6f}" for value in actual_row]
                            + (
                                [f"{float(left_hand_actual['positions'][joint_name]):.6f}" for joint_name in HAND_JOINT_NAMES]
                                + [f"{float(right_hand_actual['positions'][joint_name]):.6f}" for joint_name in HAND_JOINT_NAMES]
                                if replay_hands and left_hand_actual is not None and right_hand_actual is not None
                                else []
                            )
                        ),
                    ]
                )
                handle.flush()
                print(
                    f"[repeat] t={elapsed:.3f}s joints={requested_joints + (PBD_HAND_JOINT_LABELS['left'] + PBD_HAND_JOINT_LABELS['right'] if replay_hands else [])} "
                    f"target={[round(value, 4) for value in target_row] + ([round(value, 4) for value in left_hand_desired.tolist()] + [round(value, 4) for value in right_hand_desired.tolist()] if replay_hands else [])} "
                    f"actual={[round(value, 4) for value in actual_row] + ([round(float(left_hand_actual['positions'][joint_name]), 4) for joint_name in HAND_JOINT_NAMES] + [round(float(right_hand_actual['positions'][joint_name]), 4) for joint_name in HAND_JOINT_NAMES] if replay_hands and left_hand_actual is not None and right_hand_actual is not None else [])}"
                )
                previous_desired_row = desired_row
                time.sleep(dt)

            final_targets = {
                int(joint_index): float(active_qs[-1, idx])
                for idx, joint_index in enumerate(requested_joints)
            }
            hold_deadline = time.time() + max(0.0, float(final_hold_s))
            while True:
                final_positions = self._read_joint_positions_or_raise(
                    UPPER_BODY_JOINTS, timeout=timeout)
                self._publish_with_upper_body_hold(
                    final_targets,
                    final_positions,
                    kp=kp,
                    kd=kd,
                    waist_kp=waist_kp,
                    waist_kd=waist_kd,
                )
                if replay_hands:
                    self._get_hand("left").write_targets_once(
                        left_hand_qs[-1].tolist(),
                        kp=0.8,
                        kd=0.05,
                        tau=0.02,
                    )
                    self._get_hand("right").write_targets_once(
                        right_hand_qs[-1].tolist(),
                        kp=0.8,
                        kd=0.05,
                        tau=0.02,
                    )
                    left_hand_actual = self.get_hand_state_snapshot("left")
                    right_hand_actual = self.get_hand_state_snapshot("right")
                else:
                    left_hand_actual = right_hand_actual = None
                actual_row = [float(final_positions[int(joint_index)])
                              for joint_index in requested_joints]
                target_row = [float(final_targets[int(joint_index)])
                              for joint_index in requested_joints]
                writer.writerow(
                    [
                        "repeat_final_hold",
                        f"{time.time() - start:.6f}",
                        " ".join(
                            [str(joint_index) for joint_index in requested_joints]
                            + (PBD_HAND_JOINT_LABELS["left"] +
                               PBD_HAND_JOINT_LABELS["right"] if replay_hands else [])
                        ),
                        " ".join(
                            [f"{value:.6f}" for value in target_row]
                            + (
                                [f"{value:.6f}" for value in left_hand_qs[-1].tolist()]
                                + [f"{value:.6f}" for value in right_hand_qs[-1].tolist()]
                                if replay_hands
                                else []
                            )
                        ),
                        " ".join(
                            [f"{value:.6f}" for value in actual_row]
                            + (
                                [f"{float(left_hand_actual['positions'][joint_name]):.6f}" for joint_name in HAND_JOINT_NAMES]
                                + [f"{float(right_hand_actual['positions'][joint_name]):.6f}" for joint_name in HAND_JOINT_NAMES]
                                if replay_hands and left_hand_actual is not None and right_hand_actual is not None
                                else []
                            )
                        ),
                    ]
                )
                handle.flush()
                print(
                    f"[repeat_final_hold] joints={requested_joints + (PBD_HAND_JOINT_LABELS['left'] + PBD_HAND_JOINT_LABELS['right'] if replay_hands else [])} "
                    f"target={[round(value, 4) for value in target_row] + ([round(value, 4) for value in left_hand_qs[-1].tolist()] + [round(value, 4) for value in right_hand_qs[-1].tolist()] if replay_hands else [])} "
                    f"actual={[round(value, 4) for value in actual_row] + ([round(float(left_hand_actual['positions'][joint_name]), 4) for joint_name in HAND_JOINT_NAMES] + [round(float(right_hand_actual['positions'][joint_name]), 4) for joint_name in HAND_JOINT_NAMES] if replay_hands and left_hand_actual is not None and right_hand_actual is not None else [])}"
                )
                if time.time() >= hold_deadline:
                    break
                time.sleep(dt)
        self.release_arms(timeout=timeout)
        return {
            "arm": side,
            "motion_file": os.path.abspath(motion_file),
            "joint_count": len(requested_joints),
            "sample_count": int(ts.shape[0]),
            "command_rate_hz": float(command_rate_hz),
            "speed": max(1e-6, float(speed)),
            "duration_s": t_final,
            "final_hold_s": max(0.0, float(final_hold_s)),
            "log_path": os.path.abspath(resolved_log_path),
            "waist_hold_joints": list(WAIST_JOINTS),
        }

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
            int(abs(stop - start) / max(0.01, float(max_speed_rad_s))
                * max(1.0, float(command_rate_hz))),
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
        wrist_roll_delta: float = 0.4,
        wrist_pitch_delta: float = 0.4,
        final_shoulder_pitch_delta: float = 0.08,
        final_shoulder_roll_toward_body_delta: float = 0.10,
        final_elbow_delta: float = 0.10,
        final_wrist_pitch_delta: float = 0.08,
    ) -> dict[str, Any]:
        side = str(arm).strip().lower()
        if side not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'.")
        arm_joints = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        roll_delta = abs(float(shoulder_roll_delta)) if side == "left" else - \
            abs(float(shoulder_roll_delta))
        pitch_delta = -abs(float(shoulder_pitch_delta))
        elbow_delta_signed = -abs(float(elbow_delta))
        wrist_roll_delta_signed = abs(float(wrist_roll_delta))
        wrist_pitch_delta_signed = -abs(float(wrist_pitch_delta))
        final_pitch_delta = -abs(float(final_shoulder_pitch_delta))
        final_roll_delta = -math.copysign(
            abs(float(final_shoulder_roll_toward_body_delta)),
            roll_delta,
        )
        final_elbow_delta_signed = -abs(float(final_elbow_delta))
        final_wrist_pitch_delta_signed = -abs(float(final_wrist_pitch_delta))
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
        stage_4_steps = max(1, steps // 5)
        stage_5_steps = max(1, steps - stage_1_steps - stage_2_steps - stage_3_steps - stage_4_steps)

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

        final_pose = list(target_pose)
        final_pose[0] = clamp_joint(arm_joints[0], float(target_pose[0]) + final_pitch_delta)
        final_pose[1] = clamp_joint(arm_joints[1], float(target_pose[1]) + final_roll_delta)
        final_pose[3] = clamp_joint(arm_joints[3], float(target_pose[3]) + final_elbow_delta_signed)
        final_pose[5] = clamp_joint(
            arm_joints[5],
            float(target_pose[5]) + final_wrist_pitch_delta_signed,
        )
        stages = [
            (start_pose, roll_pose, stage_1_steps, "shoulder_roll_clearance"),
            (roll_pose, pitch_pose, stage_2_steps, "shoulder_pitch_forward"),
            (pitch_pose, restored_roll_pose, stage_3_steps, "partial_shoulder_roll_restore"),
            (restored_roll_pose, target_pose, stage_4_steps, "elbow_and_wrist_pitch"),
            (target_pose, final_pose, stage_5_steps, "final_forward_up_and_in"),
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
            "final_pose": final_pose,
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint_index] for joint_index in arm_joints],
            "stages": [stage_name for _start, _target, _steps, stage_name in stages],
            "command_rate_hz": float(command_rate_hz),
            "duration_s": float(duration_s),
        }

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
        wrist_roll_delta: float = 0.2,
        wrist_pitch_delta: float = 0.4,
        final_shoulder_pitch_delta: float = 0.08,
        final_shoulder_roll_toward_body_delta: float = 0.10,
        final_elbow_delta: float = 0.10,
        final_wrist_pitch_delta: float = 0.08,
    ) -> dict[str, Any]:
        """Best-effort inverse of :meth:`extend_arm_forward`.

        This exactly reverses an `extend_arm_forward` call when the forward
        motion did not hit joint limits and the same delta parameters are used.
        """
        side = str(arm).strip().lower()
        if side not in ("left", "right"):
            raise ValueError("arm must be 'left' or 'right'.")
        arm_joints = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        roll_delta = abs(float(shoulder_roll_delta)) if side == "left" else - \
            abs(float(shoulder_roll_delta))
        pitch_delta = -abs(float(shoulder_pitch_delta))
        elbow_delta_signed = -abs(float(elbow_delta))
        wrist_roll_delta_signed = abs(float(wrist_roll_delta))
        wrist_pitch_delta_signed = -abs(float(wrist_pitch_delta))
        final_pitch_delta = -abs(float(final_shoulder_pitch_delta))
        final_roll_delta = -math.copysign(
            abs(float(final_shoulder_roll_toward_body_delta)),
            roll_delta,
        )
        final_elbow_delta_signed = -abs(float(final_elbow_delta))
        final_wrist_pitch_delta_signed = -abs(float(final_wrist_pitch_delta))
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
        stage_4_steps = max(1, steps // 5)
        stage_5_steps = max(1, steps - stage_1_steps - stage_2_steps - stage_3_steps - stage_4_steps)

        final_adjustment_retracted_pose = list(start_pose)
        final_adjustment_retracted_pose[0] = clamp_joint(
            arm_joints[0], float(start_pose[0]) - final_pitch_delta)
        final_adjustment_retracted_pose[1] = clamp_joint(
            arm_joints[1], float(start_pose[1]) - final_roll_delta)
        final_adjustment_retracted_pose[3] = clamp_joint(
            arm_joints[3], float(start_pose[3]) - final_elbow_delta_signed)
        final_adjustment_retracted_pose[5] = clamp_joint(
            arm_joints[5], float(start_pose[5]) - final_wrist_pitch_delta_signed)

        wrist_and_elbow_retracted_pose = list(final_adjustment_retracted_pose)
        wrist_and_elbow_retracted_pose[3] = clamp_joint(
            arm_joints[3], float(final_adjustment_retracted_pose[3]) - elbow_delta_signed)
        wrist_and_elbow_retracted_pose[4] = clamp_joint(
            arm_joints[4], float(final_adjustment_retracted_pose[4]) - wrist_roll_delta_signed)
        wrist_and_elbow_retracted_pose[5] = clamp_joint(
            arm_joints[5], float(final_adjustment_retracted_pose[5]) - wrist_pitch_delta_signed)

        clearance_roll_pose = list(wrist_and_elbow_retracted_pose)
        clearance_roll_pose[1] = clamp_joint(
            arm_joints[1],
            float(final_adjustment_retracted_pose[1]) + (roll_delta * restore_fraction),
        )

        shoulder_pitch_retracted_pose = list(clearance_roll_pose)
        shoulder_pitch_retracted_pose[0] = clamp_joint(
            arm_joints[0], float(final_adjustment_retracted_pose[0]) - pitch_delta)

        target_pose = list(shoulder_pitch_retracted_pose)
        target_pose[1] = clamp_joint(
            arm_joints[1],
            float(final_adjustment_retracted_pose[1]) - (roll_delta * (1.0 - restore_fraction)),
        )
        stages = [
            (start_pose, final_adjustment_retracted_pose, stage_5_steps, "undo_final_forward_up_and_in"),
            (
                final_adjustment_retracted_pose,
                wrist_and_elbow_retracted_pose,
                stage_4_steps,
                "undo_elbow_and_wrist_pitch",
            ),
            (wrist_and_elbow_retracted_pose, clearance_roll_pose,
             stage_3_steps, "restore_shoulder_roll_clearance"),
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
            "final_adjustment_retracted_pose": final_adjustment_retracted_pose,
            "wrist_and_elbow_retracted_pose": wrist_and_elbow_retracted_pose,
            "clearance_roll_pose": clearance_roll_pose,
            "shoulder_pitch_retracted_pose": shoulder_pitch_retracted_pose,
            "target_pose": target_pose,
            "joint_names": [BODY_JOINT_NAME_BY_INDEX[joint_index] for joint_index in arm_joints],
            "stages": [stage_name for _start, _target, _steps, stage_name in stages],
            "command_rate_hz": float(command_rate_hz),
            "duration_s": float(duration_s),
        }

    def get_odom(self) -> Any | None:
        if self._odom_sub is not None:
            msg = self._odom_sub.get_latest()[0]
            if msg is not None:
                return msg
        return self.get_sport_state()

    def get_odom_pose(self) -> tuple[float, float, float] | None:
        msg = self.get_odom()
        pose = odom_pose_from_msg(msg) if msg is not None else None
        if pose is not None:
            return pose
        return odom_pose_from_msg(self.get_sport_state())

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

    def prepare(self) -> None:
        self.fsm_4_prepare()

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
        # FSM 500, not 501: this academy's G1 units have the waist LOCKED
        # (only WaistYaw free); 501 is the unlocked 3-DOF-waist variant's id.
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(500)
            return
        raise AttributeError("Current locomotion client does not support FSM mode setting API.")

    def run_mode(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(802)
            return
        raise AttributeError("Current locomotion client does not support FSM mode setting API.")

    def enter_dev_mode(self, *, timeout: float = 10.0) -> int:
        return self.enter_lowcmd_dev_mode(timeout=timeout)

    def dev_mode(self, *, timeout: float = 10.0) -> int:
        return self.enter_dev_mode(timeout=timeout)

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

    def fsm_4_prepare(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(4)

    def fsm_2_squat(self) -> None:
        self.fsm_2_squat_placeholder()

    def fsm_2_squat_placeholder(self) -> None:
        self.fsm_2_airborne()

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
            gyro = tuple(float(msg.imu_state.gyroscope[i])
                         for i in range(3))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            acc = tuple(float(msg.imu_state.accelerometer[i])
                        for i in range(3))  # type: ignore[assignment]
        except Exception:
            pass
        try:
            quat = tuple(float(msg.imu_state.quaternion[i])
                         for i in range(4))  # type: ignore[assignment]
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

        if point_step <= 0 or not raw:
            return []
        try:
            fields = {str(field.name).lower(): field for field in list(msg.fields)}
            if "x" not in fields or "y" not in fields or "z" not in fields:
                return []
            dtype = np.dtype(
                {
                    "names": ["x", "y", "z"],
                    "formats": ["<f4", "<f4", "<f4"],
                    "offsets": [
                        int(fields["x"].offset),
                        int(fields["y"].offset),
                        int(fields["z"].offset),
                    ],
                    "itemsize": point_step,
                }
            )
        except Exception:
            return []

        total = max(0, min(width * height, len(raw) // point_step))
        if total <= 0:
            return []

        try:
            arr = np.frombuffer(raw, dtype=dtype, count=total)
            pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32, copy=False)
        except Exception:
            return []

        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.size == 0:
            return []

        if max_points is not None and max_points > 0 and pts.shape[0] > max_points:
            indices = np.linspace(0, pts.shape[0] - 1, num=max_points, dtype=np.int64)
            pts = pts[indices]

        points: list[Any] = []
        for x, y, z in pts:
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

    def get_detection_image_jpeg(self, timeout: float = 2.0) -> tuple[bytes, str]:
        """Return an RGB JPEG for vision-model detection.

        Prefer the Unitree VideoClient. If that path is unavailable, fall back
        to the RGBD ZMQ stream used by the notebook demos.
        """
        try:
            return self.get_camera_image_jpeg(), "video_client"
        except Exception as video_exc:
            try:
                frame = self.get_rgbd(timeout=timeout)
                return bytes(frame["rgb_jpeg"]), "rgbd"
            except Exception as rgbd_exc:
                raise RuntimeError(
                    "Could not get an RGB image from VideoClient or RGBD stream. "
                    f"VideoClient error: {video_exc}; RGBD error: {rgbd_exc}"
                ) from rgbd_exc

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
            max(0, cy - center_size): min(h, cy + center_size),
            max(0, cx - center_size): min(w, cx + center_size),
        ]
        roi = depth_m[int(h * 0.25): int(h * 0.70), int(w * 0.30): int(w * 0.70)]
        center_valid = center[center > 0]
        center_depth_m = float(__import__("numpy").median(
            center_valid)) if center_valid.size else None
        near_coverage_1m = float(__import__("numpy").mean(
            (roi > 0) & (roi <= 1.0))) if roi.size else None

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

    @staticmethod
    def _extract_json_object(text: str) -> dict[str, Any]:
        cleaned = str(text).strip()
        if cleaned.startswith("```"):
            cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned, flags=re.IGNORECASE)
            cleaned = re.sub(r"\s*```$", "", cleaned)
        try:
            value = json.loads(cleaned)
            return value if isinstance(value, dict) else {}
        except Exception:
            pass
        start = cleaned.find("{")
        end = cleaned.rfind("}")
        if start >= 0 and end > start:
            try:
                value = json.loads(cleaned[start:end + 1])
                return value if isinstance(value, dict) else {}
            except Exception:
                return {}
        return {}

    @staticmethod
    def _normalize_bbox(value: Any) -> tuple[float, float, float, float] | None:
        if value is None:
            return None
        if isinstance(value, dict):
            seq = [value.get(key) for key in ("x1", "y1", "x2", "y2")]
        else:
            seq = list(value) if isinstance(value, (list, tuple)) else []
        if len(seq) != 4:
            return None
        try:
            x1, y1, x2, y2 = [float(item) for item in seq]
        except Exception:
            return None
        if max(abs(x1), abs(y1), abs(x2), abs(y2)) > 1.5:
            return None
        x1, x2 = sorted((max(0.0, min(1.0, x1)), max(0.0, min(1.0, x2))))
        y1, y2 = sorted((max(0.0, min(1.0, y1)), max(0.0, min(1.0, y2))))
        if (x2 - x1) <= 0.01 or (y2 - y1) <= 0.01:
            return None
        return (x1, y1, x2, y2)

    @staticmethod
    def _resize_jpeg_for_ollama(
        rgb_jpeg: bytes,
        *,
        max_side_px: int = 512,
        quality: int = 85,
    ) -> bytes:
        if int(max_side_px) <= 0:
            return rgb_jpeg
        try:
            import cv2

            frame = decode_video_frame_bgr(rgb_jpeg)
            height, width = frame.shape[:2]
            longest = max(height, width)
            if longest > int(max_side_px):
                scale = float(max_side_px) / float(longest)
                frame = cv2.resize(
                    frame,
                    (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), max(40, min(95, int(quality)))],
            )
            return bytes(encoded) if ok else rgb_jpeg
        except Exception:
            return rgb_jpeg

    def setup_ollama_vision_model(
        self,
        model: str | None = None,
        *,
        pull: bool = False,
        ollama_command: str = "ollama",
        start_server: bool = True,
        start_timeout: float = 20.0,
    ) -> dict[str, Any]:
        """Prepare the Ollama vision model used by detect().

        If pull=True and the model is not installed, this runs
        `ollama pull <model>`. Otherwise it reports whether the model is
        already available.
        """
        requested = str(model) if model is not None else None
        selected = requested or str(self.vision_model)

        server_ready = self._ollama_ready(timeout=1.5)
        if not server_ready and start_server:
            server_ready = self._ensure_ollama_running(
                command=ollama_command,
                start_timeout=start_timeout,
            )
        if not server_ready:
            return {
                "ok": False,
                "server_ready": False,
                "model": selected,
                "installed": False,
                "message": "Ollama server is not reachable.",
            }

        names = self._ollama_model_names()
        if requested is None:
            available = self._select_available_vision_model(names)
            if available is not None:
                selected = available
                self.vision_model = selected
        installed = self._ollama_model_available(selected, names)
        pull_result: dict[str, Any] | None = None
        if not installed and pull:
            if shutil.which(ollama_command) is None and not os.path.exists(ollama_command):
                raise FileNotFoundError(f"Ollama executable not found: {ollama_command}")
            proc = subprocess.run(
                [ollama_command, "pull", selected],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=1800,
            )
            pull_result = {"returncode": int(proc.returncode), "output": proc.stdout}
            if proc.returncode != 0:
                return {
                    "ok": False,
                    "server_ready": True,
                    "model": selected,
                    "installed": False,
                    "pull": pull_result,
                    "message": f"ollama pull failed for {selected}.",
                }
            names = self._ollama_model_names()
            installed = self._ollama_model_available(selected, names)
            if installed:
                self.vision_model = selected
        return {
            "ok": bool(installed),
            "server_ready": True,
            "model": selected,
            "installed": bool(installed),
            "pull": pull_result,
            "message": "ready" if installed else f"Model is not installed. Run: ollama pull {selected}",
        }

    def detect(
        self,
        object: str = "human",
        *,
        img: bool = False,
        model: str | None = None,
        confidence_threshold: float = 0.0,
        timeout: float = 120.0,
        image_max_side_px: int = 512,
        image_quality: int = 85,
    ) -> float | dict[str, Any]:
        """Detect whether an object is visible in the current RGB camera view.

        Returns a confidence float when img=False. When img=True, returns a
        dictionary containing confidence, normalized bbox, pixel bbox, raw model
        response, and an RGB numpy image with the bbox drawn when available.
        """
        setup = self.setup_ollama_vision_model(model=model, pull=False)
        if not setup.get("ok"):
            raise RuntimeError(str(setup.get("message", "Ollama vision model is not ready.")))
        selected_model = str(setup.get("model") or model or self.vision_model)

        target = str(object).strip() or "object"
        rgb_jpeg, image_source = self.get_detection_image_jpeg(timeout=min(5.0, float(timeout)))
        prompt = (
            "You are a visual object detector. Inspect the image and answer only valid JSON. "
            f"Target object: {target!r}. "
            "Return this exact schema: "
            "{\"object\":\"target name\",\"present\":true|false,\"confidence\":0.0,"
            "\"bbox\":[x1,y1,x2,y2],\"reason\":\"short\"}. "
            "bbox must be normalized image coordinates from 0 to 1 around the most likely target, "
            "or null if the target is not visible. Confidence must be between 0 and 1."
        )
        retry_sides = [
            int(image_max_side_px),
            min(384, int(image_max_side_px)),
            min(256, int(image_max_side_px)),
        ]
        last_error: Exception | None = None
        result: dict[str, Any] | None = None
        used_side = retry_sides[0]
        for side in dict.fromkeys(max(64, value) for value in retry_sides):
            used_side = int(side)
            ollama_jpeg = self._resize_jpeg_for_ollama(
                rgb_jpeg,
                max_side_px=used_side,
                quality=int(image_quality),
            )
            body = {
                "model": selected_model,
                "messages": [
                    {
                        "role": "user",
                        "content": prompt,
                        "images": [base64.b64encode(ollama_jpeg).decode("ascii")],
                    }
                ],
                "stream": False,
                "keep_alive": "0",
                "think": False,
                "options": {
                    "temperature": 0.0,
                    "num_predict": 120,
                    "num_ctx": 1024,
                    "num_batch": 128,
                },
            }
            try:
                result = self._ollama_request("/api/chat", body, timeout=timeout)
                last_error = None
                break
            except Exception as exc:
                last_error = exc
                if "llama runner process has terminated" not in str(exc) and "timed out" not in str(exc).lower():
                    raise
                time.sleep(1.0)
        if result is None:
            raise RuntimeError(f"Ollama vision request failed after retries: {last_error}") from last_error
        raw_text = str(result.get("message", {}).get("content", "")).strip()
        parsed = self._extract_json_object(raw_text)
        try:
            confidence = float(parsed.get("confidence", 0.0))
        except Exception:
            confidence = 0.0
        confidence = max(0.0, min(1.0, confidence))
        if not bool(parsed.get("present", confidence > 0.0)):
            confidence = 0.0
        bbox_norm = self._normalize_bbox(parsed.get("bbox"))

        if not img:
            return confidence

        import cv2

        frame_bgr = decode_video_frame_bgr(rgb_jpeg)
        height, width = frame_bgr.shape[:2]
        bbox_px: tuple[int, int, int, int] | None = None
        if bbox_norm is not None and confidence >= float(confidence_threshold):
            x1, y1, x2, y2 = bbox_norm
            bbox_px = (
                int(round(x1 * width)),
                int(round(y1 * height)),
                int(round(x2 * width)),
                int(round(y2 * height)),
            )
            cv2.rectangle(frame_bgr, (bbox_px[0], bbox_px[1]), (bbox_px[2], bbox_px[3]), (0, 255, 0), 2)
            label = f"{target} {confidence:.2f}"
            cv2.putText(
                frame_bgr,
                label,
                (bbox_px[0], max(20, bbox_px[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        return {
            "object": target,
            "confidence": confidence,
            "present": confidence > float(confidence_threshold),
            "bbox": bbox_norm,
            "bbox_pixels": bbox_px,
            "image_rgb": image_rgb,
            "raw_response": raw_text,
            "model": selected_model,
            "image_source": image_source,
            "image_max_side_px": used_side,
            "rgb_jpeg": rgb_jpeg,
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
                dist = math.hypot(float(target_x) -
                                  float(slam_pose[0]), float(target_y) - float(slam_pose[1]))
            moved = None
            if before_slam is not None and slam_pose is not None:
                moved = math.hypot(
                    float(slam_pose[0]) - float(before_slam[0]), float(slam_pose[1]) - float(before_slam[1]))
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
                    frame_gap = math.hypot(
                        float(pos[0]) - float(slam_pos[0]), float(pos[1]) - float(slam_pos[1]))
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
                        print(
                            f"[navigate_path] step={idx} skipped: sport_pose already within {dxy:.3f}m of target.")
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
                    print(
                        f"[navigate_path] failed at point {idx}: ({x:.3f},{y:.3f},{yaw:.3f}) rc={rc}")
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
            gap = math.hypot(float(sport_pose[0]) - float(pose[0]),
                             float(sport_pose[1]) - float(pose[1]))
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
        print_resp("init_pose (1804)", req, client.init_pose(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, load_path))

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
        print_resp("pose_nav (1102)", req, client.pose_nav(
            float(goal_x), float(goal_y), 0.0, 0.0, 0.0, qz, qw, mode=1))

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

    def chat(
        self,
        text: str | None = None,
        *,
        model: str | None = None,
        system_prompt: str | None = None,
        temperature: float = 0.4,
        num_predict: int = 80,
        timeout: float = 30.0,
        speak: bool = False,
        start_robot_chat: bool = False,
        extra_args: list[str] | None = None,
    ) -> str | subprocess.Popen[str]:
        """Chat with Ollama, or launch the ASR-backed chat.py node.

        Pass text for a single Ollama reply. Pass start_robot_chat=True, or
        call with text=None, to start scripts/chat.py in a subprocess.
        """
        selected_model = str(model or self.chat_model)
        if start_robot_chat or text is None:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            script = os.path.join(base_dir, "chat.py")
            if not os.path.exists(script):
                script = os.path.join(base_dir, "scripts", "chat.py")
            command = [
                sys.executable,
                script,
                "--iface",
                str(self.iface),
                "--domain-id",
                str(self.domain_id),
                "--ollama-url",
                str(self.ollama_url),
                "--model",
                selected_model,
            ]
            if extra_args:
                command.extend([str(item) for item in extra_args])
            log_path = "/tmp/robot_sdk_chat.log"
            pid_path = "/tmp/robot_sdk_chat.pid"
            log_handle = open(log_path, "a", encoding="utf-8")
            log_handle.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] starting {' '.join(command)}\n")
            log_handle.flush()
            proc = subprocess.Popen(
                command,
                text=True,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            log_handle.close()
            self._chat_process = proc
            with open(pid_path, "w", encoding="utf-8") as handle:
                handle.write(str(proc.pid))
            return proc

        if not self._ollama_ready(timeout=1.5):
            self._ensure_ollama_running()
        prompt = system_prompt or (
            "You are the voice of a Unitree humanoid robot. "
            "Answer naturally, concisely, and do not mention hidden reasoning."
        )
        body = {
            "model": selected_model,
            "messages": [
                {"role": "system", "content": prompt},
                {"role": "user", "content": str(text)},
            ],
            "stream": False,
            "think": False,
            "options": {
                "temperature": float(temperature),
                "num_predict": int(num_predict),
            },
        }
        result = self._ollama_request("/api/chat", body, timeout=timeout)
        reply = str(result.get("message", {}).get("content", "")).strip()
        reply = " ".join(reply.split())
        if speak and reply:
            self.say(reply)
        return reply

    def stop_chat(self, *, timeout: float = 3.0) -> bool:
        """Stop a chat.py process launched by Robot.chat()."""
        pid_path = "/tmp/robot_sdk_chat.pid"
        pids: list[int] = []
        proc = self._chat_process
        if proc is not None and proc.poll() is None:
            pids.append(int(proc.pid))
        try:
            with open(pid_path, "r", encoding="utf-8") as handle:
                value = handle.read().strip()
            if value:
                pids.append(int(value))
        except Exception:
            pass

        stopped = False
        for pid in sorted(set(pids)):
            try:
                os.kill(pid, 15)
                stopped = True
            except ProcessLookupError:
                continue
            except Exception:
                continue

        deadline = time.time() + max(0.0, float(timeout))
        for pid in sorted(set(pids)):
            while time.time() < deadline:
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                try:
                    os.kill(pid, 9)
                    stopped = True
                except Exception:
                    pass
        try:
            os.unlink(pid_path)
        except Exception:
            pass
        self._chat_process = None
        return stopped

    def hanged_boot(
        self,
        step: float = 0.02,
        max_height: float = 0.22,
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

    def say(
        self,
        text: str = "what would you like me to say?",
        volume: int | None = None,
        language: str | None = None,
        voice_model: str | None = None,
        speaker: int | None = None,
    ) -> int:
        return self._get_audio().speak(text, volume=volume, model=voice_model, language=language, speaker=speaker)

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

    def unrelease_fingers(
        self,
        hand: str = "both",
        hold_s: float = 0.4,
        rate_hz: float = 50.0,
        ramp_s: float | None = 0.2,
    ) -> None:
        side = str(hand).strip().lower()
        if side == "both":
            for each_hand in ("left", "right"):
                self._get_hand(each_hand).stop_release_fingers()
                self._get_hand(each_hand).open(hold_s=hold_s, rate_hz=rate_hz, ramp_s=ramp_s)
            return
        self._get_hand(side).stop_release_fingers()
        self._get_hand(side).open(hold_s=hold_s, rate_hz=rate_hz, ramp_s=ramp_s)

    def zero_torque_fingers(
        self,
        hand: str = "both",
        *,
        rate_hz: float = 50.0,
        persistent: bool = True,
    ) -> None:
        side = str(hand).strip().lower()
        if side == "both":
            for each_hand in ("left", "right"):
                controller = self._get_hand(each_hand)
                current_targets = list(
                    controller._last_targets) if controller._last_targets is not None else None
                if current_targets is not None:
                    controller.set_targets(
                        current_targets,
                        hold_s=0.15,
                        rate_hz=rate_hz,
                        kp=1.0,
                        kd=0.05,
                        tau=0.02,
                    )
                controller.release_fingers(
                    hold_s=0.2,
                    rate_hz=rate_hz,
                    persistent=persistent,
                )
            return
        controller = self._get_hand(side)
        current_targets = list(
            controller._last_targets) if controller._last_targets is not None else None
        if current_targets is not None:
            controller.set_targets(
                current_targets,
                hold_s=0.15,
                rate_hz=rate_hz,
                kp=1.0,
                kd=0.05,
                tau=0.02,
            )
        controller.release_fingers(
            hold_s=0.2,
            rate_hz=rate_hz,
            persistent=persistent,
        )

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
        self._get_hand(hand).move_finger(finger_name, hold_s=hold_s,
                                         settle_s=settle_s, rate_hz=rate_hz)

    def _get_latest_lidar_cloud_msg(self) -> tuple[PointCloud2_ | None, str | None, float]:
        with self._lock:
            best_topic = None
            best_msg = None
            best_ts = 0.0
            for topic in self.point_cloud_topics:
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

    @staticmethod
    def _extract_hand_tactile_series(msg: Any) -> dict[str, list[Any]]:
        pressures: list[list[float]] = []
        temperatures: list[list[float]] = []
        lost: list[int] = []
        for sensor in list(getattr(msg, "press_sensor_state", []) or []):
            try:
                pressures.append([float(value)
                                 for value in list(getattr(sensor, "pressure", []) or [])])
            except Exception:
                pressures.append([])
            try:
                temperatures.append([float(value)
                                    for value in list(getattr(sensor, "temperature", []) or [])])
            except Exception:
                temperatures.append([])
            try:
                lost.append(int(getattr(sensor, "lost", 0)))
            except Exception:
                lost.append(0)
        return {
            "pressures": pressures,
            "temperatures": temperatures,
            "lost": lost,
        }

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

    def get_hand_state_snapshot(self, hand: str = "right") -> dict[str, Any] | None:
        side = str(hand).strip().lower()
        hand_msg, hand_ts = self._get_hand_state_msg(side)
        if hand_msg is None:
            return None
        positions, velocities, torques = self._extract_hand_joint_series(hand_msg)
        tactile = self._extract_hand_tactile_series(hand_msg)
        return {
            "hand": side,
            "source": HAND_STATE_TOPIC_BY_SIDE.get(side),
            "timestamp": float(hand_ts),
            "positions": {
                HAND_JOINT_NAMES[idx]: positions[idx]
                for idx in range(min(len(positions), len(HAND_JOINT_NAMES)))
            },
            "velocities": {
                HAND_JOINT_NAMES[idx]: velocities[idx]
                for idx in range(min(len(velocities), len(HAND_JOINT_NAMES)))
            },
            "torques": {
                HAND_JOINT_NAMES[idx]: torques[idx]
                for idx in range(min(len(torques), len(HAND_JOINT_NAMES)))
            },
            "tactile_pressures": tactile["pressures"],
            "tactile_temperatures": tactile["temperatures"],
            "tactile_lost": tactile["lost"],
        }

    def get_tactile_pressures(self, hand: str = "right") -> list[list[float]] | None:
        snapshot = self.get_hand_state_snapshot(hand)
        if snapshot is None:
            return None
        return snapshot["tactile_pressures"]

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
                depth_scale = (
                    _decode_depth_scale(bytes(parts[2]))
                    if len(parts) >= 3
                    else 0.001
                )
                return rgb_jpeg, depth_png, depth_scale, time.time()
        finally:
            try:
                socket.close(0)
                context.term()
            except Exception:
                pass
        raise RuntimeError(
            f"No RGBD frames received from tcp://{self.rgbd_host}:{self.rgbd_port} within {timeout:.1f}s.")


__all__ = ["Robot", "ImuData"]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Smoke test for sdk_client Robot wrapper")
    parser.add_argument("--iface", default=default_dds_iface("eth0"))
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
