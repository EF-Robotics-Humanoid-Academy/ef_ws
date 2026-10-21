import audioop
import importlib
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import wave
from pathlib import Path

import numpy as np

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID, ROBOT_API_ID_LOCO_GET_FSM_MODE
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_
from unitree_sdk2py.rpc.client import Client
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


FSM_IDS = {"zero_torque": 0, "damp": 1, "prepare": 4, "walk": 501, "run": 802}
SERVICE_CATALOG = {
    "ai_sport": "Main Motion Control Service",
    "basic_service": "Basic Service",
    "g1_arm_example": "Upper Limb Motion Service",
    "vui_service": "Audio and Lighting Control Service",
    "unitree_slam": "Navigation Service",
}
HL_ARM_ACTIONS = {
    "release arm": 99, "two-hand kiss": 11, "left kiss": 12, "right kiss": 13, "hands up": 15,
    "clap": 17, "high five": 18, "hug": 19, "heart": 20, "right heart": 21, "reject": 22,
    "right hand up": 23, "x-ray": 24, "face wave": 25, "high wave": 26, "shake hand": 27,
}
HL_ARM_ACTION_ALIASES = {
    "release": "release arm", "two hand kiss": "two-hand kiss", "lefthand kiss": "left kiss",
    "left hand kiss": "left kiss", "righthand kiss": "right kiss", "right hand kiss": "right kiss",
    "xray": "x-ray", "x ray": "x-ray",
}
LEFT_LEG_JOINTS = [0, 1, 2, 3, 4, 5]
RIGHT_LEG_JOINTS = [6, 7, 8, 9, 10, 11]
WAIST_JOINTS = [12, 13, 14]
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
LOWCMD_JOINTS = LEFT_LEG_JOINTS + RIGHT_LEG_JOINTS + WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
HAND_JOINT_NAMES = ["thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"]
HAND_CMD_TOPICS = {"left": "rt/dex3/left/cmd", "right": "rt/dex3/right/cmd"}
HAND_STATE_TOPICS = {"left": "rt/dex3/left/state", "right": "rt/dex3/right/state"}
HAND_MAX = {"left": [1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0], "right": [1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75]}
HAND_MIN = {"left": [-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75], "right": [-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0]}
HAND_THUMB0 = {"left": -0.09927542507648468, "right": -0.03510913997888565}
HAND_CLOSED = {
    "left": [HAND_THUMB0["left"], HAND_MAX["left"][1], HAND_MAX["left"][2], HAND_MIN["left"][3], HAND_MIN["left"][4], HAND_MIN["left"][5], HAND_MIN["left"][6]],
    "right": [HAND_THUMB0["right"], HAND_MIN["right"][1], HAND_MIN["right"][2], HAND_MAX["right"][3], HAND_MAX["right"][4], HAND_MAX["right"][5], HAND_MAX["right"][6]],
}
HAND_OPEN = {
    side: [closed[0]] + [hi if abs(v - lo) < abs(v - hi) else lo for v, lo, hi in zip(closed[1:], HAND_MIN[side][1:], HAND_MAX[side][1:])]
    for side, closed in HAND_CLOSED.items()
}
BMS_TOPICS = ["rt/lf/bmsstate", "rt/lf/agvbmsstate", "rt/bmsstate", "rt/agvbmsstate"]
INSPIRE_CONFIGS = {"right": ("192.168.123.210", 6000, 1), "left": ("192.168.123.211", 6000, 1)}
INSPIRE_OPEN = [1000, 1000, 1000, 1000, 1000, 250]
INSPIRE_CLOSE = [500, 500, 500, 500, 500, 250]
_NAMED_COLORS = {
    "white": (255, 255, 255), "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
    "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
    "orange": (255, 165, 0), "purple": (128, 0, 128), "pink": (255, 105, 180),
}
_PIPER_VOICES = {
    "en": "en_US-lessac-medium", "en_us": "en_US-lessac-medium", "english": "en_US-lessac-medium",
    "de": "de_DE-thorsten-medium", "de_de": "de_DE-thorsten-medium", "german": "de_DE-thorsten-medium",
    "fr": "fr_FR-siwis-medium", "fr_fr": "fr_FR-siwis-medium", "french": "fr_FR-siwis-medium",
    "es": "es_ES-davefx-medium", "es_es": "es_ES-davefx-medium", "spanish": "es_ES-davefx-medium",
    "ar": "ar_JO-kareem-medium", "ar_jo": "ar_JO-kareem-medium", "arabic": "ar_JO-kareem-medium",
}
SLAM_POINT_TOPICS = [
    "rt/unitree/slam_mapping/points",
    "rt/unitree/slam_relocation/points",
    "rt/unitree/slam_relocation/global_map",
    "rt/unitree/slam_relocation/web_points",
]
_factory = None

_ROOT_DIR = Path(__file__).resolve().parent.parent
if str(_ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(_ROOT_DIR))

try:
    from hand_pose_navigation.arm_fk import ArmFK, JOINT_LIMITS
    from hand_pose_navigation.arm_ik import ArmIK
except Exception:
    ArmFK = None
    ArmIK = None
    JOINT_LIMITS = {}


def _ensure_factory(domain_id, iface):
    global _factory
    cfg = (int(domain_id), str(iface))
    if _factory is None:
        ChannelFactoryInitialize(cfg[0], cfg[1])
        _factory = cfg
    elif _factory != cfg:
        raise RuntimeError(f"ChannelFactoryInitialize already called with {_factory}, got {cfg}")


def _rpc_get_int(client, api_id):
    code, data = client._Call(int(api_id), "{}")
    if int(code) != 0 or not data:
        return None
    value = json.loads(data).get("data")
    return None if value is None else int(value)


def _normalize_action(name):
    key = " ".join(str(name).strip().lower().replace("_", " ").split())
    return HL_ARM_ACTION_ALIASES.get(key, key)


def _normalize_side(hand):
    side = str(hand).strip().lower()
    if side in ("left", "l"):
        return "left"
    if side in ("right", "r"):
        return "right"
    raise ValueError("hand must be left/right")


def _clamp_hand(side, targets):
    return [max(float(lo), min(float(hi), float(v))) for v, lo, hi in zip(targets, HAND_MIN[side], HAND_MAX[side])]


def _parse_color(value):
    if isinstance(value, tuple) and len(value) == 3:
        return tuple(int(max(0, min(255, v))) for v in value)
    value = str(value).strip().lower()
    if value in _NAMED_COLORS:
        return _NAMED_COLORS[value]
    if re.fullmatch(r"#?[0-9a-fA-F]{6}", value):
        value = value.lstrip("#")
        return (int(value[0:2], 16), int(value[2:4], 16), int(value[4:6], 16))
    if re.fullmatch(r"\d{1,3},\d{1,3},\d{1,3}", value):
        return tuple(max(0, min(255, int(x))) for x in value.split(","))
    raise ValueError("color must be a name, #RRGGBB, or R,G,B")


def _scale_color(rgb, intensity):
    scale = max(0, min(100, int(intensity))) / 100.0
    return tuple(int(x * scale) for x in rgb)


def _read(obj, name, default=None):
    if obj is None or not hasattr(obj, name):
        return default
    value = getattr(obj, name)
    if callable(value):
        try:
            return value()
        except TypeError:
            return value
    return value


def _result_code(result):
    if isinstance(result, tuple):
        return int(result[0])
    return int(result)


def _mode_name(data):
    if not isinstance(data, dict):
        return ""
    for key in ("name", "mode", "alias"):
        value = data.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _vector3(value):
    try:
        if value is None:
            return None
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return None


def _rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


_ROT_BY_AXIS = (_rot_x, _rot_y, _rot_z)


def _clamp_arm_q(q, side):
    if not JOINT_LIMITS or side not in JOINT_LIMITS:
        return q
    lo = np.array([lim[0] for lim in JOINT_LIMITS[side]], dtype=np.float64)
    hi = np.array([lim[1] for lim in JOINT_LIMITS[side]], dtype=np.float64)
    return np.clip(q, lo, hi)


def _solve_position_shoulder_elbow(fk, side, target_T, q_init, selected_axis=None):
    """Position-only DLS with wrist joints held fixed, matching ik_pose_cli_v3."""
    q = _clamp_arm_q(q_init.copy(), side)
    lam = 0.05
    eps = 1e-5
    max_iter = 64
    tol = 0.005
    active = (0, 1, 2, 3)
    best_q = q.copy()
    best_err_pos = float("inf")
    best_axis_err = float("inf")

    for iteration in range(max_iter):
        T_cur = fk.compute_arm(q)
        pos_err = target_T[:3, 3] - T_cur[:3, 3]
        err_pos = float(np.linalg.norm(pos_err))
        axis_err = abs(float(pos_err[selected_axis])) if selected_axis is not None else err_pos
        if (err_pos, axis_err) < (best_err_pos, best_axis_err):
            best_q = q.copy()
            best_err_pos = err_pos
            best_axis_err = axis_err
        if err_pos < tol:
            return q, {
                "success": True,
                "error_pos_m": err_pos,
                "error_rot_rad": 0.0,
                "iterations": iteration,
                "mode": "pos_shoulder_elbow",
            }

        J = np.zeros((3, len(active)), dtype=np.float64)
        p0 = T_cur[:3, 3]
        for col, idx in enumerate(active):
            q1 = q.copy()
            q1[idx] += eps
            T1 = fk.compute_arm(q1)
            J[:, col] = (T1[:3, 3] - p0) / eps

        JJT = J @ J.T
        dq_active = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(3), pos_err)
        norm_dq = float(np.linalg.norm(dq_active))
        if norm_dq > 0.3:
            dq_active *= 0.3 / norm_dq

        q_next = q.copy()
        for col, idx in enumerate(active):
            q_next[idx] += dq_active[col]
        q = _clamp_arm_q(q_next, side)
        q[4:] = q_init[4:]

    T_cur = fk.compute_arm(best_q)
    err = target_T[:3, 3] - T_cur[:3, 3]
    err_pos = float(np.linalg.norm(err))
    axis_err = abs(float(err[selected_axis])) if selected_axis is not None else err_pos
    if selected_axis is not None and axis_err < 0.006 and err_pos < 0.040:
        return best_q, {
            "success": True,
            "error_pos_m": err_pos,
            "error_rot_rad": 0.0,
            "iterations": max_iter,
            "mode": "pos_axis_clamped",
            "axis_error_m": axis_err,
        }
    return None, {
        "success": False,
        "error_pos_m": err_pos,
        "error_rot_rad": 0.0,
        "iterations": max_iter,
        "mode": "pos_shoulder_elbow",
    }


def _decode_json_text(raw):
    try:
        return json.loads(raw)
    except Exception:
        return None


def _parse_slam_pose(raw):
    payload = _decode_json_text(raw)
    if not isinstance(payload, dict) or int(payload.get("errorCode", 0)) != 0:
        return None
    cur = payload.get("data", {}).get("currentPose", {})
    try:
        x = float(cur.get("x", 0.0))
        y = float(cur.get("y", 0.0))
        z = float(cur.get("z", 0.0))
        if {"q_x", "q_y", "q_z", "q_w"} <= set(cur):
            qx = float(cur.get("q_x", 0.0))
            qy = float(cur.get("q_y", 0.0))
            qz = float(cur.get("q_z", 0.0))
            qw = float(cur.get("q_w", 1.0))
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        else:
            yaw = float(cur.get("yaw", 0.0))
    except Exception:
        return None
    if abs(x) < 1e-5 and abs(y) < 1e-5 and abs(z) < 1e-5 and abs(yaw) < 1e-5:
        return None
    return (x, y, yaw)


class _Latest:
    def __init__(self, topic, msg_type, queue_len=10):
        self.msg = None
        self.ts = 0.0
        self.sub = ChannelSubscriber(topic, msg_type)
        self.sub.Init(self._cb, int(queue_len))
    def _cb(self, msg):
        self.msg = msg
        self.ts = time.time()
    def get(self):
        return self.msg, self.ts


class _HeadlightThread(threading.Thread):
    def __init__(self, client, rgb, duration, interval, stop_event):
        super().__init__(daemon=False)
        self.client = client
        self.rgb = rgb
        self.duration = max(0.0, float(duration))
        self.interval = max(0.0, float(interval))
        self.stop_event = stop_event
        self.last_code = 0
    def run(self):
        end_time = time.monotonic() + self.duration
        next_call = time.monotonic() + self.interval
        try:
            while not self.stop_event.is_set() and time.monotonic() < end_time:
                remaining = next_call - time.monotonic()
                if remaining > 0 and self.stop_event.wait(remaining):
                    break
                self.last_code = int(self.client.LedControl(*self.rgb))
                if self.last_code != 0:
                    self.stop_event.set()
                    break
                next_call += self.interval
        finally:
            try:
                self.client.LedControl(0, 0, 0)
            except Exception:
                pass


class _Dex3:
    def __init__(self, side):
        self.side = side
        self.pub = ChannelPublisher(HAND_CMD_TOPICS[side], HandCmd_)
        self.pub.Init()
        self.state = _Latest(HAND_STATE_TOPICS[side], HandState_, 20)
    def _write(self, targets, kp=0.8, kd=0.05, tau=0.02):
        msg = unitree_hg_msg_dds__HandCmd_()
        for i, q in enumerate(_clamp_hand(self.side, targets)):
            cmd = msg.motor_cmd[i]
            cmd.mode = (i & 0x0F) | (1 << 4)
            cmd.q = float(q)
            cmd.dq = 0.0
            cmd.tau = float(tau)
            cmd.kp = float(kp)
            cmd.kd = float(kd)
        self.pub.Write(msg)
    def move(self, targets, hold_s=0.6, rate_hz=50.0):
        dt = 1.0 / max(1.0, float(rate_hz))
        for _ in range(max(1, int(max(0.0, float(hold_s)) * max(1.0, float(rate_hz))))):
            self._write(targets)
            time.sleep(dt)
    def snapshot(self):
        msg, ts = self.state.get()
        if msg is None:
            return None
        out = {"hand": self.side, "timestamp": ts, "positions": {}, "velocities": {}, "torques": {}, "tactile_pressures": [], "tactile_temperatures": [], "tactile_lost": []}
        motors = list(getattr(msg, "motor_state", []) or [])
        for i, name in enumerate(HAND_JOINT_NAMES):
            motor = motors[i] if i < len(motors) else None
            out["positions"][name] = None if motor is None else float(getattr(motor, "q"))
            out["velocities"][name] = None if motor is None else float(getattr(motor, "dq"))
            out["torques"][name] = None if motor is None else float(getattr(motor, "tau_est"))
        for sensor in list(getattr(msg, "press_sensor_state", []) or []):
            out["tactile_pressures"].append([float(x) for x in list(getattr(sensor, "pressure", []) or [])])
            out["tactile_temperatures"].append([float(x) for x in list(getattr(sensor, "temperature", []) or [])])
            out["tactile_lost"].append(int(getattr(sensor, "lost", 0)))
        return out


class _LowCmd:
    def __init__(self):
        self.crc = CRC()
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
    def write(self, q, mode_machine, kp=None, kd=None, dq=0.0, tau=0.0):
        self.msg.mode_machine = int(mode_machine)
        kp = kp or [60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40]
        kd = kd or [1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
        for i in range(len(LOWCMD_JOINTS)):
            cmd = self.msg.motor_cmd[i]
            cmd.mode = 1
            cmd.q = float(q[i])
            cmd.dq = float(dq)
            cmd.tau = float(tau)
            cmd.kp = float(kp[i])
            cmd.kd = float(kd[i])
        self.msg.crc = self.crc.Crc(self.msg)
        self.pub.Write(self.msg)


class _ArmSdk:
    def __init__(self):
        self.crc = CRC()
        self.pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.pub.Init()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = 0
        self.msg.motor_cmd[29].q = 1.0
    def write(self, targets, weight=None, kp=30.0, kd=1.5, waist_kp=None, waist_kd=None, dq=0.0, tau=0.0):
        for i, q in targets.items():
            cmd = self.msg.motor_cmd[int(i)]
            cmd.mode = 1
            cmd.q = float(q)
            cmd.dq = float(dq)
            cmd.tau = float(tau)
            cmd.kp = float(waist_kp[int(i)] if waist_kp and int(i) in waist_kp else kp)
            cmd.kd = float(waist_kd[int(i)] if waist_kd and int(i) in waist_kd else kd)
        if weight is not None:
            self.msg.motor_cmd[29].q = max(0.0, min(1.0, float(weight)))
        self.msg.crc = self.crc.Crc(self.msg)
        self.pub.Write(self.msg)


class _SlamClient(Client):
    def __init__(self):
        super().__init__("slam_operate", False)
        self._RegistApi(1801, 0)
        self._RegistApi(1802, 0)
        self._RegistApi(1804, 0)
        self._RegistApi(1102, 0)
        self._RegistApi(1901, 0)
        self._SetApiVerson("1.0.0.1")
    def _call_json(self, api_id, payload):
        code, data = self._Call(api_id, json.dumps(payload, ensure_ascii=True))
        return int(code), data
    def start_mapping(self, slam_type="indoor"):
        return self._call_json(1801, {"data": {"slam_type": slam_type}})
    def stop_mapping(self, save_path=None):
        if save_path:
            return self._call_json(1802, {"data": {"address": save_path}})
        return self._call_json(1901, {"data": {}})
    def init_pose(self, x, y, yaw=0.0, address=""):
        qz = math.sin(float(yaw) * 0.5)
        qw = math.cos(float(yaw) * 0.5)
        return self._call_json(1804, {"data": {"x": float(x), "y": float(y), "z": 0.0, "q_x": 0.0, "q_y": 0.0, "q_z": qz, "q_w": qw, "address": address}})
    def pose_nav(self, x, y, yaw=0.0):
        qz = math.sin(float(yaw) * 0.5)
        qw = math.cos(float(yaw) * 0.5)
        return self._call_json(1102, {"data": {"targetPose": {"x": float(x), "y": float(y), "z": 0.0, "q_x": 0.0, "q_y": 0.0, "q_z": qz, "q_w": qw}, "mode": 1}})


def _modbus_move(host, port, unit_id, values, timeout=2.0):
    sock = socket.create_connection((host, port), timeout)
    try:
        sock.settimeout(timeout)
        tid = 1
        def request(pdu):
            nonlocal tid
            sock.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit_id) + pdu)
            tid += 1
            head = sock.recv(7)
            remaining = struct.unpack(">HHHB", head)[2] - 1
            body = b""
            while len(body) < remaining:
                chunk = sock.recv(remaining - len(body))
                if not chunk:
                    raise RuntimeError("socket closed")
                body += chunk
            if body[0] & 0x80:
                raise RuntimeError(f"modbus error {body[1] if len(body) > 1 else None}")
        request(struct.pack(">BHH", 6, 1004, 1))
        for reg in (1522, 1498):
            payload = struct.pack(">" + "H" * 6, *([200] * 6))
            request(struct.pack(">BHHB", 16, reg, 6, len(payload)) + payload)
        payload = struct.pack(">" + "H" * 6, *[int(v) & 0xFFFF for v in values])
        request(struct.pack(">BHHB", 16, 1486, 6, len(payload)) + payload)
    finally:
        sock.close()


class G1:
    def __init__(self, iface, domain_id=0):
        self.iface = str(iface)
        self.domain_id = int(domain_id)
        _ensure_factory(self.domain_id, self.iface)
        self._client = LocoClient()
        self._client.SetTimeout(10.0)
        self._client.Init()
        self._motion = None
        self._robot_state = None
        self._audio = None
        self._arm_action = None
        self._video = None
        self._slam = None
        self._lowcmd = None
        self._arm_sdk = None
        self._arm_fk = {}
        self._arm_ik = {}
        self._headlight_stop = None
        self._headlight_thread = None
        self._path_points = []
        self._dex3 = {}
        self._last_slam_pose = None
        self._initial_slam_pose = None
        self._gait_override = None
        self._slam_map_path = os.environ.get("G1_SLAM_MAP_PATH", "/home/unitree/test.pcd")
        self._lowstate = self._latest("rt/lowstate", self._load_type("unitree_sdk2py.idl.unitree_hg.msg.dds_", "LowState_", "unitree_sdk2py.idl.unitree_go.msg.dds_"))
        self._sport = self._latest("rt/odommodestate", SportModeState_)
        self._sport_alt = self._latest("rt/sportmodestate", SportModeState_)
        self._odom = self._latest("rt/odom", Odometry_)
        self._slam_odom = self._latest("rt/unitree/slam_mapping/odom", Odometry_)
        self._clouds = [(topic, self._latest(topic, PointCloud2_)) for topic in SLAM_POINT_TOPICS]
        self._slam_info = self._latest("rt/slam_info", String_)
        self._slam_key = self._latest("rt/slam_key_info", String_)
        self._audio_msg = self._latest("rt/audio_msg", String_)
        self._bms = []
        for topic in BMS_TOPICS:
            for msg_type in self._bms_types():
                self._bms.append((topic, self._latest(topic, msg_type)))

    def _load_type(self, primary_module, name, fallback_module=None):
        module = importlib.import_module(primary_module)
        if hasattr(module, name):
            return getattr(module, name)
        if fallback_module:
            module = importlib.import_module(fallback_module)
            return getattr(module, name)
        raise RuntimeError(f"Could not load {name}")

    def _bms_types(self):
        out = []
        for module_name in ("unitree_sdk2py.idl.unitree_hg.msg.dds_", "unitree_sdk2py.idl.unitree_go.msg.dds_"):
            module = importlib.import_module(module_name)
            if hasattr(module, "BmsState_"):
                out.append(module.BmsState_)
        return out

    def _latest(self, topic, msg_type, queue_len=10):
        return _Latest(topic, msg_type, queue_len)

    def _robot_state_client(self):
        if self._robot_state is None:
            try:
                mod = importlib.import_module("unitree_sdk2py.b2.robot_state.robot_state_client")
            except Exception:
                mod = importlib.import_module("unitree_sdk2py.go2.robot_state.robot_state_client")
            self._robot_state = mod.RobotStateClient()
            if hasattr(self._robot_state, "SetTimeout"):
                self._robot_state.SetTimeout(2.0)
            self._robot_state.Init()
        return self._robot_state

    def _motion_client(self):
        if self._motion is None:
            self._motion = MotionSwitcherClient()
            self._motion.SetTimeout(5.0)
            self._motion.Init()
        return self._motion

    def _audio_client(self):
        if self._audio is None:
            self._audio = AudioClient()
            self._audio.SetTimeout(5.0)
            self._audio.Init()
        return self._audio

    def _arm_action_client(self):
        if self._arm_action is None:
            self._arm_action = G1ArmActionClient()
            self._arm_action.SetTimeout(10.0)
            self._arm_action.Init()
        return self._arm_action

    def _video_client(self):
        if self._video is None:
            try:
                mod = importlib.import_module("unitree_sdk2py.g1.video.video_client")
            except Exception:
                mod = importlib.import_module("unitree_sdk2py.go2.video.video_client")
            self._video = mod.VideoClient()
            self._video.SetTimeout(2.0)
            self._video.Init()
        return self._video

    def _slam_client(self):
        if self._slam is None:
            self._slam = _SlamClient()
            self._slam.SetTimeout(10.0)
        return self._slam

    def _lowcmd_client(self):
        if self._lowcmd is None:
            self._lowcmd = _LowCmd()
        return self._lowcmd

    def _arm_sdk_client(self):
        if self._arm_sdk is None:
            self._arm_sdk = _ArmSdk()
        return self._arm_sdk

    def _arm_fk_solver(self, side):
        side = _normalize_side(side)
        if ArmFK is None:
            raise RuntimeError("hand_pose_navigation ArmFK is unavailable; cannot run DLS IK")
        if side not in self._arm_fk:
            self._arm_fk[side] = ArmFK(side, "urdf")
        return self._arm_fk[side]

    def _arm_ik_solver(self, side):
        side = _normalize_side(side)
        if ArmIK is None:
            raise RuntimeError("hand_pose_navigation ArmIK is unavailable; cannot run DLS IK")
        if side not in self._arm_ik:
            self._arm_ik[side] = ArmIK(side, "dls", max_iter=24, tol_pos_m=0.005, tol_rot_rad=0.02)
        return self._arm_ik[side]

    def _dex3_hand(self, side):
        side = _normalize_side(side)
        if side not in self._dex3:
            self._dex3[side] = _Dex3(side)
        return self._dex3[side]

    def _lowstate_msg(self):
        return self._lowstate.get()[0]

    def _sport_msg(self):
        primary = self._sport.get()
        secondary = self._sport_alt.get()
        if secondary[0] is not None and secondary[1] >= primary[1]:
            return secondary[0]
        return primary[0]

    def _string_data(self, msg):
        data = _read(msg, "data")
        return None if data is None else str(data)

    def _latest_cloud_msg(self):
        best_msg = None
        best_ts = 0.0
        for topic, latest in self._clouds:
            msg, ts = latest.get()
            if msg is not None and ts >= best_ts:
                best_msg = (topic, msg, ts)
                best_ts = ts
        return best_msg

    def _slam_pose(self):
        pose = _parse_slam_pose(self.get_slam_info())
        if pose is not None:
            self._last_slam_pose = pose
            return pose
        return self._last_slam_pose

    def _hand_error(self, hand, action, exc=None):
        out = {"hand": hand, "ok": False, "action": action, "error": "hand not detected"}
        if exc is not None:
            out["detail"] = str(exc)
        return out

    def _current_q_mode(self, timeout=3.0):
        deadline = time.time() + max(0.1, float(timeout))
        while time.time() < deadline:
            msg = self._lowstate_msg()
            if msg is not None:
                return [float(msg.motor_state[i].q) for i in range(len(LOWCMD_JOINTS))], int(getattr(msg, "mode_machine", 0))
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate")

    def _upper_body_pose(self, timeout=3.0):
        q, _ = self._current_q_mode(timeout=timeout)
        return {i: float(q[i]) for i in UPPER_BODY_JOINTS}

    def _odom_pose(self, msg):
        if msg is None:
            return None
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            x, y = float(pos.x), float(pos.y)
            qx, qy, qz, qw = float(ori.x), float(ori.y), float(ori.z), float(ori.w)
        except Exception:
            try:
                pos = msg.position()
                quat = msg.imu_state().quaternion()
                x, y = float(pos[0]), float(pos[1])
                qw, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            except Exception:
                return None
        return (x, y, math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    def _exec_arm_action(self, action, release_after_s=None):
        if isinstance(action, str):
            action = HL_ARM_ACTIONS[_normalize_action(action)]
        client = self._arm_action_client()
        code = int(client.ExecuteAction(int(action)))
        if release_after_s is not None:
            time.sleep(max(0.0, float(release_after_s)))
            release = int(client.ExecuteAction(HL_ARM_ACTIONS["release arm"]))
            return release if code == 0 else code
        return code

    def get_state(self):
        motion_code, motion_raw = self._motion_client().CheckMode()
        return {
            "id": _rpc_get_int(self._client, ROBOT_API_ID_LOCO_GET_FSM_ID),
            "mode": _rpc_get_int(self._client, ROBOT_API_ID_LOCO_GET_FSM_MODE),
            "motion_mode": _mode_name(motion_raw),
            "motion_code": int(motion_code),
            "gait": self.get_gait(),
            "battery": self.get_battery(),
            "lowstate": self.get_lowstate(),
            "services": self.get_service(),
            "slam_info": self.get_slam_info(),
        }

    def get_service(self, service=None):
        client = self._robot_state_client()
        if not hasattr(client, "ServiceList"):
            rows = [{"name": name, "description": desc, "status": None, "protected": None} for name, desc in SERVICE_CATALOG.items()]
        else:
            code, service_states = client.ServiceList()
            if int(code) != 0:
                raise RuntimeError(f"ServiceList failed: {code}")
            rows = []
            for state in service_states or []:
                name = str(getattr(state, "name", "")).strip()
                if name:
                    rows.append({"name": name, "description": SERVICE_CATALOG.get(name, ""), "status": None if getattr(state, "status", None) is None else int(getattr(state, "status")), "protected": None if getattr(state, "protect", None) is None else bool(int(getattr(state, "protect")))})
            known = {row["name"] for row in rows}
            for name, desc in SERVICE_CATALOG.items():
                if name not in known:
                    rows.append({"name": name, "description": desc, "status": None, "protected": None})
        if service is None:
            return rows
        service = str(service).strip().lower()
        for row in rows:
            if row["name"].lower() == service:
                return row
        return None

    def toggle_service(self, service):
        row = self.get_service(service)
        if row is None:
            raise ValueError(f"Unknown service: {service}")
        enabled = row.get("status") != 0
        code = int(self._robot_state_client().ServiceSwitch(row["name"], bool(enabled)))
        return {"name": row["name"], "previous_status": row.get("status"), "enabled": enabled, "code": code}

    def zero_torque_mode(self):
        return self._client.SetFsmId(FSM_IDS["zero_torque"])

    def damp_mode(self):
        return self._client.SetFsmId(FSM_IDS["damp"])

    def prepare_mode(self):
        return self._client.SetFsmId(FSM_IDS["prepare"])

    def walk_mode(self):
        return self._client.SetFsmId(FSM_IDS["walk"])

    def run_mode(self):
        return self._client.SetFsmId(FSM_IDS["run"])

    def toggle_dev_mode(self):
        return self.toggle_service("ai_sport")

    def get_gait(self):
        msg = self._sport_msg()
        if msg is None:
            return self._gait_override
        for key in ("gait_type", "gaitType", "gait"):
            value = _read(msg, key)
            if value is not None:
                value = int(value)
                if self._gait_override is not None and value == 0:
                    return self._gait_override
                return value
        return self._gait_override

    def toggle_gait(self):
        current = self.get_gait() or 0
        target = 0 if current else 1
        codes = []
        if target:
            for method_name in ("SetBalanceMode", "SetGaitType"):
                if hasattr(self._client, method_name):
                    try:
                        code = int(getattr(self._client, method_name)(1))
                    except Exception:
                        continue
                    codes.append((method_name, code))
                    if code == 0:
                        self._gait_override = 1
                        return {"gait": 1, "method": method_name, "code": code}
        else:
            if hasattr(self._client, "BalanceStand"):
                try:
                    codes.append(("BalanceStand", int(self._client.BalanceStand(0))))
                except Exception:
                    pass
            for method_name in ("SetBalanceMode", "SetGaitType"):
                if hasattr(self._client, method_name):
                    try:
                        code = int(getattr(self._client, method_name)(0))
                    except Exception:
                        continue
                    codes.append((method_name, code))
            if hasattr(self._client, "SetFsmId"):
                try:
                    codes.append(("SetFsmId", int(self._client.SetFsmId(FSM_IDS["walk"]))))
                except Exception:
                    pass
            if any(code == 0 for _, code in codes):
                self._gait_override = 0
                return {"gait": 0, "codes": codes}
        return {"gait": target, "codes": codes}

    def get_lowstate(self):
        msg = self._lowstate_msg()
        if msg is None:
            return None
        motors = list(getattr(msg, "motor_state", []) or [])
        imu = getattr(msg, "imu_state", None)
        return {
            "timestamp": time.time(),
            "joint_positions": [float(getattr(m, "q", 0.0)) for m in motors],
            "joint_velocities": [float(getattr(m, "dq", 0.0)) for m in motors],
            "joint_torques": [float(getattr(m, "tau_est", 0.0)) for m in motors],
            "imu": None if imu is None else {"rpy": [float(imu.rpy[i]) for i in range(3)], "gyro": [float(imu.gyroscope[i]) for i in range(3)], "acc": [float(imu.accelerometer[i]) for i in range(3)]},
            "raw": msg,
        }

    def say(self, text="", language="EN", volume=100):
        piper = os.environ.get("G1_PIPER_BIN") or os.environ.get("PIPER_BIN") or "piper"
        if subprocess.call(["/usr/bin/env", "which", piper], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) != 0:
            local_piper = Path.home() / ".local" / "bin" / "piper"
            if local_piper.exists():
                piper = str(local_piper)
            else:
                raise RuntimeError("piper not found")
        lang = str(language or "EN").strip().lower().replace("-", "_")
        voice = _PIPER_VOICES.get(lang)
        if voice is None:
            raise ValueError(f"unsupported Piper language {language!r}")
        model = Path.home() / ".local" / "share" / "piper" / "voices" / voice / f"{voice}.onnx"
        if not model.exists():
            raise FileNotFoundError(f"Piper model does not exist: {model}")
        with tempfile.TemporaryDirectory(prefix="g1_say_") as td:
            wav_path = Path(td) / "speech.wav"
            robot_wav = Path(td) / "speech_robot.wav"
            subprocess.run([piper, "--model", str(model), "--output-file", str(wav_path)], input=str(text), text=True, check=True)
            with wave.open(str(wav_path), "rb") as wf:
                channels, sample_width, frame_rate = wf.getnchannels(), wf.getsampwidth(), wf.getframerate()
                pcm = wf.readframes(wf.getnframes())
            if channels == 2:
                pcm = audioop.tomono(pcm, sample_width, 0.5, 0.5)
                channels = 1
            if sample_width != 2:
                pcm = audioop.lin2lin(pcm, sample_width, 2)
                sample_width = 2
            if frame_rate != 16000:
                pcm, _ = audioop.ratecv(pcm, sample_width, channels, frame_rate, 16000, None)
            with wave.open(str(robot_wav), "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(pcm)
            client = self._audio_client()
            client.SetVolume(int(volume))
            with wave.open(str(robot_wav), "rb") as wf:
                pcm = wf.readframes(wf.getnframes())
            code, _ = client.PlayStream("sdk_wrapper_v3", "sdk-wrapper-v3", pcm)
            return int(code)

    def set_headlight(self, color="green", intensity=100, duration_s=3):
        rgb = _scale_color(_parse_color(color), intensity)
        client = self._audio_client()
        if self._headlight_thread is not None and self._headlight_thread.is_alive():
            self._headlight_stop.set()
            self._headlight_thread.join()
        code = int(client.LedControl(*rgb))
        if code != 0 or float(duration_s) <= 0:
            return code
        self._headlight_stop = threading.Event()
        self._headlight_thread = _HeadlightThread(client, rgb, duration_s, 0.2, self._headlight_stop)
        self._headlight_thread.start()
        return code

    def release_arms(self):
        positions = self._upper_body_pose()
        arm_sdk = self._arm_sdk_client()
        for i in range(151):
            ratio = float(i) / 150.0
            fade = ratio * ratio * (3.0 - 2.0 * ratio)
            weight = 1.0 - fade
            arm_sdk.write(positions, weight=weight, kp=30.0 * weight, kd=1.5 * weight, waist_kp={j: 480.0 * weight for j in WAIST_JOINTS}, waist_kd={j: 12.0 * weight for j in WAIST_JOINTS})
            time.sleep(0.02)
        return {"final_arm_sdk_weight": 0.0, "joint_count": len(positions)}

    def engage_arms(self):
        positions = self._upper_body_pose()
        arm_sdk = self._arm_sdk_client()
        for i in range(51):
            weight = float(i) / 50.0
            arm_sdk.write(positions, weight=weight, kp=30.0, kd=1.5, waist_kp={j: 480.0 for j in WAIST_JOINTS}, waist_kd={j: 12.0 for j in WAIST_JOINTS})
            time.sleep(0.02)
        return {"final_arm_sdk_weight": 1.0, "joint_count": len(positions)}

    def loco_move(self, vx, vy, vyaw, duration_s=2):
        code = int(self._client.Move(float(vx), float(vy), float(vyaw), continous_move=True) or 0)
        try:
            time.sleep(max(0.0, float(duration_s)))
        finally:
            self.loco_stop()
        return code

    def loco_stop(self):
        if hasattr(self._client, "StopMove"):
            return self._client.StopMove()
        return self._client.Move(0.0, 0.0, 0.0, continous_move=False)

    def get_rgbd(self):
        endpoints = ["tcp://0.0.0.0:5555", "tcp://127.0.0.1:5555", "tcp://localhost:5555"]
        last_error = None
        for endpoint in endpoints:
            try:
                import zmq

                ctx = zmq.Context.instance()
                sock = ctx.socket(zmq.SUB)
                sock.setsockopt(zmq.SUBSCRIBE, b"")
                sock.setsockopt(zmq.RCVTIMEO, 3000)
                sock.connect(endpoint)
                try:
                    parts = sock.recv_multipart()
                finally:
                    sock.close(0)
                if len(parts) < 3:
                    continue
                scale = None
                if parts[2] != b"0" and len(parts[2]) == 4:
                    scale = float(struct.unpack("f", parts[2])[0])
                return {
                    "endpoint": endpoint,
                    "rgb_jpeg": bytes(parts[0]),
                    "depth_png": None if parts[1] == b"0" else bytes(parts[1]),
                    "depth_scale": scale,
                }
            except Exception as exc:
                last_error = exc
        if last_error is not None:
            raise last_error
        return None

    def get_point_cloud(self):
        latest = self._latest_cloud_msg()
        if latest is None:
            return []
        _, msg, _ = latest
        try:
            fields = {str(f.name).lower(): f for f in list(msg.fields)}
            x_off, y_off, z_off = int(fields["x"].offset), int(fields["y"].offset), int(fields["z"].offset)
            point_step, width, height, raw = int(msg.point_step), int(msg.width), int(msg.height), bytes(msg.data)
        except Exception:
            return []
        total = max(0, min(width * height, len(raw) // max(1, point_step)))
        step = max(1, total // 20000) if total > 20000 else 1
        points = []
        for idx in range(0, total, step):
            base = idx * point_step
            try:
                x = struct.unpack_from("<f", raw, base + x_off)[0]
                y = struct.unpack_from("<f", raw, base + y_off)[0]
                z = struct.unpack_from("<f", raw, base + z_off)[0]
            except Exception:
                continue
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append({"x": float(x), "y": float(y), "z": float(z)})
        return points

    def get_slam_info(self):
        msg, _ = self._slam_info.get()
        if msg is not None:
            return self._string_data(msg)
        msg, _ = self._slam_key.get()
        return None if msg is None else self._string_data(msg)

    def move_ll_joint(self, joint_id, q, dq, kp, kd, tau, arm_sdk=True):
        joint_id = int(joint_id)
        if arm_sdk:
            if joint_id not in UPPER_BODY_JOINTS:
                raise ValueError("arm_sdk=True only supports joints 12-28")
            pose = self._upper_body_pose()
            pose[joint_id] = float(q)
            self._arm_sdk_client().write(pose, kp=float(kp), kd=float(kd), dq=float(dq), tau=float(tau), waist_kp={j: 480.0 for j in WAIST_JOINTS}, waist_kd={j: 12.0 for j in WAIST_JOINTS})
            return {"joint_id": joint_id, "arm_sdk": True}
        if joint_id not in LOWCMD_JOINTS:
            raise ValueError("arm_sdk=False only supports lowcmd body joints 0-28")
        q_all, mode_machine = self._current_q_mode()
        q_all[joint_id] = float(q)
        kp_all = [60,60,60,100,40,40,60,60,60,100,40,40,60,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40]
        kd_all = [1,1,1,2,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
        kp_all[joint_id] = float(kp)
        kd_all[joint_id] = float(kd)
        self._lowcmd_client().write(q_all, mode_machine, kp=kp_all, kd=kd_all, dq=float(dq), tau=float(tau))
        return {"joint_id": joint_id, "arm_sdk": False, "topic": "rt/lowcmd"}

    def get_odom(self):
        msg = self._sport_msg()
        if msg is not None:
            return {
                "timestamp": time.time(),
                "topic": "rt/odommodestate",
                "position": _vector3(_read(msg, "position") or _read(msg, "pos") or _read(msg, "position_w")),
                "velocity": _vector3(_read(msg, "velocity") or _read(msg, "vel")),
                "imu": self.get_imus(),
                "mode": None if _read(msg, "mode") is None else int(_read(msg, "mode")),
                "gait": self.get_gait(),
                "raw": msg,
            }
        msg, ts = self._odom.get()
        if msg is not None:
            return {"timestamp": ts, "topic": "rt/odom", "pose": self._odom_pose(msg), "raw": msg}
        msg, ts = self._slam_odom.get()
        if msg is not None:
            return {"timestamp": ts, "topic": "rt/unitree/slam_mapping/odom", "pose": self._odom_pose(msg), "raw": msg}
        return None

    def get_imus(self):
        msg = self._lowstate_msg() or self._sport_msg()
        if msg is None:
            return None
        imu = _read(msg, "imu_state")
        if imu is None:
            return {"rpy": None, "gyro": None, "acc": None, "quat": None, "temp": None}
        out = {"rpy": None, "gyro": None, "acc": None, "quat": None, "temp": None}
        try:
            out["rpy"] = tuple(float(imu.rpy[i]) for i in range(3))
        except Exception:
            pass
        try:
            out["gyro"] = tuple(float(imu.gyroscope[i]) for i in range(3))
        except Exception:
            pass
        try:
            out["acc"] = tuple(float(imu.accelerometer[i]) for i in range(3))
        except Exception:
            pass
        try:
            out["quat"] = tuple(float(imu.quaternion[i]) for i in range(4))
        except Exception:
            pass
        try:
            out["temp"] = float(imu.temperature)
        except Exception:
            pass
        return out

    def get_battery(self):
        for topic, latest in self._bms:
            msg, ts = latest.get()
            if msg is None or (time.time() - ts) > 3.0:
                continue
            return {
                "timestamp": time.time(),
                "source": topic,
                "power_v": None,
                "power_a": None,
                "bit_flag": None,
                "battery_timeout": None,
                "bms": {
                    "status": None if getattr(msg, "status", None) is None else int(getattr(msg, "status")),
                    "soc": None if getattr(msg, "soc", None) is None else int(getattr(msg, "soc")),
                    "soh": None if getattr(msg, "soh", None) is None else int(getattr(msg, "soh")),
                    "current": None if getattr(msg, "current", None) is None else int(getattr(msg, "current")),
                    "cycle": None if getattr(msg, "cycle", None) is None else int(getattr(msg, "cycle")),
                    "version_high": None if getattr(msg, "version_high", None) is None else int(getattr(msg, "version_high")),
                    "version_low": None if getattr(msg, "version_low", None) is None else int(getattr(msg, "version_low")),
                    "bq_ntc": [int(x) for x in list(getattr(msg, "bq_ntc", []) or [])],
                    "mcu_ntc": [int(x) for x in list(getattr(msg, "mcu_ntc", []) or [])],
                    "temperature": [int(x) for x in list(getattr(msg, "temperature", []) or [])],
                    "cell_vol": [int(x) for x in list(getattr(msg, "cell_vol", []) or [])],
                    "bmsvoltage": [int(x) for x in list(getattr(msg, "bmsvoltage", []) or [])],
                    "bmsstate": [int(x) for x in list(getattr(msg, "bmsstate", []) or [])],
                },
            }
        msg = self._lowstate_msg()
        if msg is None:
            return None
        bms = getattr(msg, "bms_state", None)
        bit_flag = getattr(msg, "bit_flag", None)
        return {
            "timestamp": time.time(),
            "power_v": None if getattr(msg, "power_v", None) is None else float(getattr(msg, "power_v")),
            "power_a": None if getattr(msg, "power_a", None) is None else float(getattr(msg, "power_a")),
            "bit_flag": None if bit_flag is None else int(bit_flag),
            "battery_timeout": None if bit_flag is None else bool(int(bit_flag) & 0x10),
            "bms": None if bms is None else {
                "status": None if getattr(bms, "status", None) is None else int(getattr(bms, "status")),
                "soc": None if getattr(bms, "soc", None) is None else int(getattr(bms, "soc")),
                "current": None if getattr(bms, "current", None) is None else int(getattr(bms, "current")),
                "cycle": None if getattr(bms, "cycle", None) is None else int(getattr(bms, "cycle")),
                "version_high": None if getattr(bms, "version_high", None) is None else int(getattr(bms, "version_high")),
                "version_low": None if getattr(bms, "version_low", None) is None else int(getattr(bms, "version_low")),
                "bq_ntc": [int(x) for x in list(getattr(bms, "bq_ntc", []) or [])],
                "mcu_ntc": [int(x) for x in list(getattr(bms, "mcu_ntc", []) or [])],
                "cell_vol": [int(x) for x in list(getattr(bms, "cell_vol", []) or [])],
            },
        }

    def open_dex3_hand(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {}
        for side in sides:
            try:
                self._dex3_hand(side).move(HAND_OPEN[side])
                out[side] = {"hand": side, "ok": True, "action": "open_dex3_hand"}
            except Exception as exc:
                out[side] = self._hand_error(side, "open_dex3_hand", exc)
        return out if len(sides) > 1 else out[sides[0]]

    def close_dex3_hand(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {}
        for side in sides:
            try:
                self._dex3_hand(side).move(HAND_CLOSED[side])
                out[side] = {"hand": side, "ok": True, "action": "close_dex3_hand"}
            except Exception as exc:
                out[side] = self._hand_error(side, "close_dex3_hand", exc)
        return out if len(sides) > 1 else out[sides[0]]

    def get_dex3_hand_sensors(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {}
        for side in sides:
            try:
                snap = self._dex3_hand(side).snapshot()
                out[side] = self._hand_error(side, "get_dex3_hand_sensors") if snap is None else snap
            except Exception as exc:
                out[side] = self._hand_error(side, "get_dex3_hand_sensors", exc)
        return out if len(sides) > 1 else out[sides[0]]

    def get_inspire_hand_sensors(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {side: self._hand_error(side, "get_inspire_hand_sensors") for side in sides}
        return out if len(sides) > 1 else out[sides[0]]

    def open_inspire_hand(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {}
        for side in sides:
            try:
                host, port, unit_id = INSPIRE_CONFIGS[side]
                _modbus_move(host, port, unit_id, INSPIRE_OPEN)
                out[side] = {"hand": side, "ok": True, "action": "open_inspire_hand"}
            except Exception as exc:
                out[side] = self._hand_error(side, "open_inspire_hand", exc)
        return out if len(sides) > 1 else out[sides[0]]

    def close_inspire_hand(self, hand="both"):
        sides = ("left", "right") if str(hand).strip().lower() == "both" else (_normalize_side(hand),)
        out = {}
        for side in sides:
            try:
                host, port, unit_id = INSPIRE_CONFIGS[side]
                _modbus_move(host, port, unit_id, INSPIRE_CLOSE)
                out[side] = {"hand": side, "ok": True, "action": "close_inspire_hand"}
            except Exception as exc:
                out[side] = self._hand_error(side, "close_inspire_hand", exc)
        return out if len(sides) > 1 else out[sides[0]]

    def start_mapping(self):
        self._initial_slam_pose = self._slam_pose()
        code, raw = self._slam_client().start_mapping("indoor")
        return {"code": code, "raw": raw}

    def stop_mapping(self):
        code, raw = self._slam_client().stop_mapping()
        return {"code": code, "raw": raw}

    def relocate(self):
        pose = self._slam_pose() or self._last_slam_pose or self._initial_slam_pose or (0.0, 0.0, 0.0)
        code, resp = self._slam_client().init_pose(pose[0], pose[1], yaw=pose[2], address=self._slam_map_path)
        if int(code) == 0:
            self._last_slam_pose = pose
            self._initial_slam_pose = pose
        return {"code": code, "raw": resp}

    def add_map_pose(self):
        pose = self._slam_pose() or self._last_slam_pose or self._initial_slam_pose
        if pose is None:
            raise RuntimeError("No valid SLAM pose available")
        self._path_points.append(pose)
        return pose

    def navigate(self, map_pose):
        if map_pose is not None:
            self._path_points.append((float(map_pose[0]), float(map_pose[1]), float(map_pose[2])))
        if not self._path_points:
            raise RuntimeError("No path points queued")
        out = []
        for x, y, yaw in self._path_points:
            code, raw = self._slam_client().pose_nav(x, y, yaw=yaw)
            out.append({"target": (x, y, yaw), "code": code, "raw": raw})
            if code != 0:
                break
        self._path_points.clear()
        return out

    def get_mic_input(self, duration_s=0.0, poll_s=0.05):
        deadline = time.time() + max(0.0, float(duration_s))
        seen = set()
        messages = []
        while True:
            msg, ts = self._audio_msg.get()
            raw = None if msg is None else self._string_data(msg)
            if raw:
                key = (float(ts), raw)
                if key not in seen:
                    seen.add(key)
                    payload = _decode_json_text(raw)
                    messages.append({
                        "timestamp": ts,
                        "topic": "rt/audio_msg",
                        "raw": raw,
                        "payload": payload,
                        "text": str(payload.get("text", "")) if isinstance(payload, dict) else raw,
                    })
            if float(duration_s) <= 0.0 or time.time() >= deadline:
                return messages
            time.sleep(max(0.01, float(poll_s)))

    def clap(self):
        return self._exec_arm_action("clap")

    def face_wave(self):
        return self._exec_arm_action("face wave")

    def high_wave(self):
        return self._exec_arm_action("high wave")

    def shake_hand(self):
        return self._exec_arm_action("shake hand", release_after_s=2.0)

    def hug(self):
        return self._exec_arm_action("hug", release_after_s=2.0)

    def left_kiss(self):
        return self._exec_arm_action("left kiss")

    def ik_move_ee(self, hand="right", delta_q=None, pose_increment=None, max_speed=0.35, max_dq=0.2, rate_hz=50.0, position_only=False):
        side = _normalize_side(hand)
        inc = pose_increment if pose_increment is not None else delta_q
        if inc is None:
            raise ValueError("pose_increment is required")
        inc = list(inc)
        if len(inc) == 3:
            inc = inc + [0.0, 0.0, 0.0]
        if len(inc) != 6:
            raise ValueError("pose_increment must have 3 or 6 elements: [dx, dy, dz, droll, dpitch, dyaw]")
        dx, dy, dz, droll, dpitch, dyaw = [float(x) for x in inc]
        joints = RIGHT_ARM_JOINTS if side == "right" else LEFT_ARM_JOINTS
        current = self._upper_body_pose()
        q_init = np.array([current[j] for j in joints], dtype=np.float64)
        fk = self._arm_fk_solver(side)
        target_T = fk.compute_arm(q_init).copy()

        target_T[0, 3] += dx
        target_T[1, 3] += dy if side == "left" else -dy
        target_T[2, 3] += dz
        if not position_only:
            rotations = (
                -droll if side == "left" else droll,
                -dpitch,
                -dyaw if side == "left" else dyaw,
            )
            for axis, value in enumerate(rotations):
                if value:
                    target_T[:3, :3] = _ROT_BY_AXIS[axis](value) @ target_T[:3, :3]

        selected_axis = None
        xyz_nonzero = [idx for idx, value in enumerate((dx, dy, dz)) if abs(value) > 1e-12]
        if position_only and len(xyz_nonzero) == 1:
            selected_axis = xyz_nonzero[0]
        if position_only:
            q_sol, info = _solve_position_shoulder_elbow(
                fk,
                side,
                target_T,
                q_init,
                selected_axis=selected_axis,
            )
        else:
            q_sol, info = self._arm_ik_solver(side).solve(target_T, q_init=q_init)
        if q_sol is None:
            return {
                "hand": side,
                "success": False,
                "pose_increment": inc,
                "position_only": bool(position_only),
                "ik": info,
                "steps": 0,
            }

        delta = np.clip(np.asarray(q_sol, dtype=np.float64) - q_init, -float(max_dq), float(max_dq))
        q_apply = _clamp_arm_q(q_init + delta, side)
        target = dict(current)
        for i, joint_id in enumerate(joints):
            target[joint_id] = float(q_apply[i])
        step_limit = max(1e-4, float(max_speed) / max(1.0, float(rate_hz)))
        remaining = max(abs(target[j] - current[j]) for j in joints)
        steps = max(1, int(math.ceil(remaining / step_limit)))
        arm_sdk = self._arm_sdk_client()
        for step in range(1, steps + 1):
            ratio = float(step) / float(steps)
            ramp = ratio * ratio * (3.0 - 2.0 * ratio)
            frame = dict(current)
            for joint_id in joints:
                frame[joint_id] = current[joint_id] + (target[joint_id] - current[joint_id]) * ramp
            arm_sdk.write(frame, kp=30.0, kd=1.5, waist_kp={12: 200.0, 13: 200.0, 14: 480.0}, waist_kd={j: 12.0 for j in WAIST_JOINTS})
            time.sleep(1.0 / max(1.0, float(rate_hz)))
        final_T = fk.compute_arm(q_apply)
        return {
            "hand": side,
            "success": True,
            "pose_increment": inc,
            "position_only": bool(position_only),
            "ik": info,
            "joint_targets": {joint_id: target[joint_id] for joint_id in joints},
            "ee_pose": {
                "x": float(final_T[0, 3]),
                "y": float(final_T[1, 3]),
                "z": float(final_T[2, 3]),
            },
            "steps": steps,
        }

    def extend_arm(self, hand="right", dx=0.08, dy=0.08, dz=0.04, steps=3, max_speed=0.25, max_dq=0.15, rate_hz=50.0):
        side = _normalize_side(hand)
        step_count = max(1, int(steps))
        inc = [float(dx) / step_count, float(dy) / step_count, float(dz) / step_count, 0.0, 0.0, 0.0]
        out = []
        for _ in range(step_count):
            out.append(self.ik_move_ee(hand=side, pose_increment=inc, max_speed=max_speed, max_dq=max_dq, rate_hz=rate_hz, position_only=True))
        return {"hand": side, "steps": step_count, "step_increment": inc, "position_only": True, "results": out}

    def teach(self, arm="both"):
        raise NotImplementedError

    def repeat(self):
        raise NotImplementedError
