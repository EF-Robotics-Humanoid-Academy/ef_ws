"""lib_solved.py — the G1 Academy Bonus toolbox, consolidated.

Every function/class below was built, one small piece at a time, across the 12
hands-on notebooks `solved/task2_*.ipynb` .. `solved/task13_*.ipynb` (Task 1,
`task1_sdkwrapper_usage.ipynb`, only *uses* the finished `sdk_wrapper.G1` and
builds nothing new, so it has no section here). This file exists so a later
task's slide can say "this is already available" and mean it: import it and
call it, instead of re-pasting the previous task's code cell into a new
notebook.

Section banners below are numbered by the task that introduces that piece, in
the same dependency order the course teaches them — each section only calls
functions/classes defined in an *earlier* section (or the `unitree_sdk2py`/
`util`/`hand_pose_navigation` modules directly), the same rule the tasks
themselves follow. By the time you reach Task 13's `AcademyRobot`, every
method it needs already exists above it in this file.

Two deliberate differences from copy-pasting the notebook cells verbatim:

1. Every task notebook calls `ensure_channel_factory(0, "eth0")` immediately
   at import time, hardcoding the interface. A shared library must not do
   that — the caller may be on a different interface/domain, or want to
   `import lib_solved` before deciding. Call `lib_solved.init(domain_id, iface)`
   **once**, exactly where a notebook would have run its Task-2 setup cell,
   before calling anything else in this module.
2. Per-task publishers/clients (`rt/arm_sdk`, `rt/lowcmd`, Dex3 hands,
   `LocoClient`, `AudioClient`, `G1ArmActionClient`, the SLAM RPC client) are
   constructed lazily on first use and cached, the same pattern
   `sdk_wrapper.G1` itself uses (`_audio_client()`, `_arm_sdk_client()`, ...) —
   so importing this module, or calling `init()`, never opens a publisher a
   given session never ends up using.

Compare any function here against the matching method on `sdk_wrapper.G1` —
that comparison *is* the point of this bonus track (see Task 13's "Reflect"
note).
"""
from __future__ import annotations

import importlib
import json
import math
import os
import re
import struct
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
from unitree_sdk2py.rpc.client import Client
from unitree_sdk2py.utils.crc import CRC

from util import HAND_CLOSED, HAND_OPEN, play_piper_text

try:
    from hand_pose_navigation.arm_fk import ArmFK
    from hand_pose_navigation.arm_ik import ArmIK
except Exception:  # pragma: no cover - only available where hand_pose_navigation is on sys.path
    ArmFK = None
    ArmIK = None


# ======================================================================
# Task 2 — necessary DDS init + pub/sub helpers
# ======================================================================
# The two foundational helpers every later section depends on: a kernel-wide
# ChannelFactory guard, and a generic "latest message" subscriber cache.

_factory_config = None


def ensure_channel_factory(domain_id, interface):
    """Initialize the DDS ChannelFactory exactly once per process.

    ChannelFactoryInitialize has global process state: call it before
    constructing any ChannelSubscriber/ChannelPublisher, always with the same
    (domain_id, interface). A conflicting re-call raises instead of silently
    reconnecting — restart the kernel/process to point at a different one.
    """
    global _factory_config
    config = (int(domain_id), str(interface))
    if _factory_config is None:
        ChannelFactoryInitialize(*config)
        _factory_config = config
    elif _factory_config != config:
        raise RuntimeError(f"ChannelFactory already initialized as {_factory_config}; restart kernel for {config}.")
    return _factory_config


class Latest:
    """A ChannelSubscriber that keeps only the newest message and its receipt time.

    A callback runs asynchronously, so `.message` can legitimately be `None`
    (nothing received yet) or stale (publisher died, wrong topic, wrong
    domain/interface) — `.fresh()` turns that check into a one-liner instead
    of repeating it at every call site.
    """

    def __init__(self, topic, message_type, queue_len=10):
        self.message = None
        self.timestamp = 0.0
        self.subscriber = ChannelSubscriber(topic, message_type)
        self.subscriber.Init(self._callback, queue_len)

    def _callback(self, message):
        self.message = message
        self.timestamp = time.time()

    def fresh(self, max_age_s=0.5):
        return self.message is not None and time.time() - self.timestamp <= max_age_s


def make_publisher(topic, message_type):
    """Every native publisher (`rt/arm_sdk`, `rt/lowcmd`, `rt/dex3/*/cmd`, ...)
    follows this same construct-then-Init() pattern."""
    publisher = ChannelPublisher(topic, message_type)
    publisher.Init()
    return publisher


def diagnose(latest, name, max_age_s=0.5):
    """Turn a Latest subscriber into a human-readable freshness report — useful
    the first time a topic name, domain id, or interface turns out to be wrong.

    | Symptom                          | Likely cause                              | Check                         |
    |-----------------------------------|--------------------------------------------|--------------------------------|
    | `message is None` forever         | topic name typo, wrong domain/interface     | `ros2 topic echo <topic>`      |
    | `message` present, `fresh()` False| publisher stopped, robot in a mode that stops publishing | timestamp age, robot mode |
    | `fresh()` flips True/False        | queue too short, bursty publisher           | raise `queue_len`               |
    """
    if latest.message is None:
        return f"{name}: no message received yet (publisher not running / topic name wrong / domain-id or interface mismatch)"
    age = time.time() - latest.timestamp
    return f"{name}: last message {age:.2f}s ago ({'fresh' if age <= max_age_s else 'STALE'})"


# ======================================================================
# Task 3 — say() and set_headlight() from scratch
# ======================================================================

_NAMED_COLORS = {
    "white": (255, 255, 255), "red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255),
    "yellow": (255, 255, 0), "cyan": (0, 255, 255), "magenta": (255, 0, 255),
    "orange": (255, 165, 0), "purple": (128, 0, 128), "pink": (255, 105, 180),
}


def parse_color(value):
    """Accepts a name (`"green"`), `#RRGGBB`, or `"R,G,B"`; returns a clamped (r, g, b)."""
    if isinstance(value, tuple) and len(value) == 3:
        return tuple(int(max(0, min(255, v))) for v in value)
    value = str(value).strip().lower()
    if value in _NAMED_COLORS:
        return _NAMED_COLORS[value]
    if re.fullmatch(r"#?[0-9a-fA-F]{6}", value):
        value = value.lstrip("#")
        return tuple(int(value[i:i + 2], 16) for i in (0, 2, 4))
    if re.fullmatch(r"\d{1,3},\d{1,3},\d{1,3}", value):
        return tuple(max(0, min(255, int(x))) for x in value.split(","))
    raise ValueError("color must be a name, #RRGGBB, or R,G,B")


def scale_color(rgb, intensity):
    scale = max(0, min(100, int(intensity))) / 100.0
    return tuple(int(x * scale) for x in rgb)


class HeadlightThread(threading.Thread):
    """Refreshes LedControl(*rgb) every interval_s until stop_event fires or
    duration_s elapses; always attempts to turn the light off in `finally`."""

    def __init__(self, audio_client, rgb, duration_s, interval_s, stop_event):
        super().__init__(daemon=False)
        self.audio_client, self.rgb = audio_client, rgb
        self.duration_s = max(0.0, float(duration_s))
        self.interval_s = max(0.0, float(interval_s))
        self.stop_event = stop_event

    def run(self):
        end_time = time.monotonic() + self.duration_s
        try:
            while not self.stop_event.is_set() and time.monotonic() < end_time:
                code = int(self.audio_client.LedControl(*self.rgb))
                if code != 0:
                    self.stop_event.set()
                    break
                self.stop_event.wait(self.interval_s)
        finally:
            try:
                self.audio_client.LedControl(0, 0, 0)
            except Exception:
                pass


_headlight_stop = None
_headlight_thread = None


def say(text, language="en", volume=100):
    """Synthesize `text` with Piper and play it through AudioClient.PlayStream."""
    return play_piper_text(_audio_client(), text, language=language, volume=volume)


def set_headlight(color="green", intensity=100, duration_s=3):
    """Set/hold the headlight color. duration_s <= 0 sends one LedControl call
    and returns; duration_s > 0 starts a cancellable background refresh thread
    (a new call always stops+joins any earlier one first)."""
    global _headlight_stop, _headlight_thread
    rgb = scale_color(parse_color(color), intensity)
    client = _audio_client()
    if _headlight_thread is not None and _headlight_thread.is_alive():
        _headlight_stop.set()
        _headlight_thread.join()
    code = int(client.LedControl(*rgb))
    if code != 0 or float(duration_s) <= 0:
        return code
    _headlight_stop = threading.Event()
    _headlight_thread = HeadlightThread(client, rgb, duration_s, 0.2, _headlight_stop)
    _headlight_thread.start()
    return code


# ======================================================================
# Task 4 — robot-state observation (dictionary-based get_* interface)
# ======================================================================
# Every get_* function below wraps a native subscriber/client and normalizes
# the result into a plain dict, so later tasks never touch raw DDS layouts.
#
# | Function              | Topic / client                          | Notes                              |
# |------------------------|-------------------------------------------|-------------------------------------|
# | get_lowstate()          | `rt/lowstate`                             | joints + IMU                       |
# | get_odommodestate()     | `rt/odommodestate`                        | position/velocity/gait, defensive getattr |
# | get_battery()           | `rt/lf/bmsstate` etc., falls back to lowstate | 3s freshness window            |
# | get_slam_info()         | `rt/slam_info`, falls back to `rt/slam_key_info` | JSON string               |
# | get_occupancygrid()     | *unconfirmed — raises NotImplementedError* | see `academy/todo.txt`            |
# | get_rgbd()               | ZMQ SUB `tcp://127.0.0.1:5555`            | `rgbd_server_service`              |
# | get_services()           | `RobotStateClient.ServiceList()`          | annotated with `SERVICE_CATALOG`   |


def get_lowstate():
    sub = _get_lowstate_sub()
    msg = sub.message
    if msg is None:
        return None
    motors = list(msg.motor_state)
    imu = msg.imu_state
    return {
        "timestamp": sub.timestamp,
        "joint_positions": [float(m.q) for m in motors],
        "joint_velocities": [float(m.dq) for m in motors],
        "joint_torques": [float(m.tau_est) for m in motors],
        "imu": {
            "rpy": [float(imu.rpy[i]) for i in range(3)],
            "gyro": [float(imu.gyroscope[i]) for i in range(3)],
            "acc": [float(imu.accelerometer[i]) for i in range(3)],
        },
    }


def _first_attr(obj, names, default=None):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return default


def get_odommodestate():
    sub = _get_odom_sub()
    msg = sub.message
    if msg is None:
        return None
    position = _first_attr(msg, ("position", "pos", "position_w"))
    velocity = _first_attr(msg, ("velocity", "vel"))
    gait = _first_attr(msg, ("gait_type", "gaitType", "gait"))
    return {
        "timestamp": sub.timestamp,
        "position": None if position is None else tuple(float(x) for x in position),
        "velocity": None if velocity is None else tuple(float(x) for x in velocity),
        "mode": None if getattr(msg, "mode", None) is None else int(msg.mode),
        "gait_type": None if gait is None else int(gait),
    }


BMS_TOPICS = ["rt/lf/bmsstate", "rt/lf/agvbmsstate", "rt/bmsstate", "rt/agvbmsstate"]
SERVICE_CATALOG = {
    "ai_sport": "Main Motion Control Service", "basic_service": "Basic Service",
    "g1_arm_example": "Upper Limb Motion Service", "vui_service": "Audio and Lighting Control Service",
    "unitree_slam": "Navigation Service",
}


def _bms_types():
    types = []
    for module_name in ("unitree_sdk2py.idl.unitree_hg.msg.dds_", "unitree_sdk2py.idl.unitree_go.msg.dds_"):
        module = importlib.import_module(module_name)
        if hasattr(module, "BmsState_"):
            types.append(module.BmsState_)
    return types


def get_battery():
    for topic, sub in _get_bms_subs():
        if sub.fresh(max_age_s=3.0):
            msg = sub.message
            return {"source": topic, "timestamp": sub.timestamp, "soc": int(msg.soc), "current": int(msg.current), "cycle": int(msg.cycle)}
    msg = _get_lowstate_sub().message
    if msg is None:
        return None
    return {
        "source": "rt/lowstate",
        "timestamp": _get_lowstate_sub().timestamp,
        "power_v": None if getattr(msg, "power_v", None) is None else float(msg.power_v),
        "power_a": None if getattr(msg, "power_a", None) is None else float(msg.power_a),
    }


def get_slam_info():
    info_sub, key_sub = _get_slam_info_subs()
    if info_sub.message is not None:
        return info_sub.message.data
    if key_sub.message is not None:
        return key_sub.message.data
    return None


def get_occupancygrid():
    """`academy/todo.txt`: "Add a direct occupancy-grid subscriber once the
    deployed topic and message type are confirmed." Guessing a wrong topic/type
    pair and silently returning empty data is worse than refusing outright."""
    raise NotImplementedError(
        "Confirm the deployed occupancy-grid topic/message type against the running SLAM stack "
        "(e.g. `ros2 topic list` / `ros2 topic info -v`) before implementing this; do not guess."
    )


def get_rgbd(endpoints=("tcp://127.0.0.1:5555", "tcp://localhost:5555")):
    import zmq
    ctx = zmq.Context.instance()
    last_error = None
    for endpoint in endpoints:
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVTIMEO, 3000)
        try:
            sock.connect(endpoint)
            parts = sock.recv_multipart()
            if len(parts) < 3:
                continue
            scale = None
            if parts[2] != b"0" and len(parts[2]) == 4:
                candidate = float(struct.unpack("<f", parts[2])[0])
                if math.isfinite(candidate) and candidate > 0.0:
                    scale = candidate
            return {
                "timestamp": time.time(), "endpoint": endpoint, "rgb_jpeg": bytes(parts[0]),
                "depth_png": None if parts[1] == b"0" else bytes(parts[1]), "depth_scale": scale,
            }
        except Exception as exc:
            last_error = exc
        finally:
            sock.close(0)
    if last_error is not None:
        raise last_error
    return None


def get_services():
    client = _robot_state_client()
    code, service_states = client.ServiceList()
    if int(code) != 0:
        raise RuntimeError(f"ServiceList failed: {code}")
    return [
        {"name": s.name, "description": SERVICE_CATALOG.get(s.name, ""), "status": int(s.status), "protected": bool(s.protect)}
        for s in service_states
    ]


# ======================================================================
# Task 5 — services and mode switching
# ======================================================================

# "walk" is 500, not 501: this academy's G1 units run with the waist LOCKED
# (only WaistYaw is a free joint; WaistRoll/WaistPitch are absent/invalid —
# see WAIST_JOINTS). FSM 501 is the balanced-stand/walk id for the unlocked
# 3-DOF waist variant and is not valid on this hardware.
FSM_IDS = {"zero_torque": 0, "damp": 1, "prepare": 4, "walk": 500, "run": 802}
_gait_override = None
_custom_modes = {
    # greet is presentation-only: never take ownership of the robot FSM.
    "greet": {"fsm": None, "announce": "Entering greet mode."},
    "patrol": {"fsm": "walk", "announce": "Entering patrol mode."},
}
_custom_mode_state = {"active": None, "stop": None, "thread": None}


def damp_mode():
    """The always-available safe fallback: bounded joint damping, no
    locomotion. Call this before an emergency stop, or before releasing
    controller ownership of any other publisher."""
    return _rpc_code(_loco_client().SetFsmId(FSM_IDS["damp"]))


def _rpc_code(value):
    return 0 if value is None else int(value)


def get_service(name):
    code, rows = _robot_state_client().ServiceList()
    if int(code) != 0:
        raise RuntimeError(f"ServiceList failed: {code}")
    target = str(name).strip().lower()
    return next((r for r in rows if str(r.name).strip().lower() == target), None)


def toggle_service(name):
    row = get_service(name)
    if row is None:
        raise ValueError(f"Unknown service: {name}")
    enable = int(row.status) != 0  # status 0 is ON, status 1 is OFF
    return set_service(row.name, enable)


def set_service(name, enabled):
    row = get_service(name)
    if row is None:
        raise ValueError(f"Unknown service: {name}")
    code = _rpc_code(_robot_state_client().ServiceSwitch(row.name, bool(enabled)))
    return {"name": row.name, "previous_status": int(row.status), "enabled": bool(enabled), "code": code}


def toggle_gait():
    global _gait_override
    loco = _loco_client()
    target = 0 if (_gait_override or 0) else 1
    codes = []
    if target:
        for method_name in ("SetBalanceMode", "SetGaitType"):
            if hasattr(loco, method_name):
                code = _rpc_code(getattr(loco, method_name)(1))
                codes.append((method_name, code))
                if code == 0:
                    _gait_override = 1
                    return {"gait": 1, "codes": codes}
    else:
        if hasattr(loco, "BalanceStand"):
            codes.append(("BalanceStand", _rpc_code(loco.BalanceStand(0))))
        for method_name in ("SetBalanceMode", "SetGaitType"):
            if hasattr(loco, method_name):
                codes.append((method_name, _rpc_code(getattr(loco, method_name)(0))))
        if hasattr(loco, "SetFsmId"):
            codes.append(("SetFsmId", _rpc_code(loco.SetFsmId(FSM_IDS["walk"]))))
        if any(code == 0 for _, code in codes):
            _gait_override = 0
    return {"gait": target, "codes": codes}


def _exit_custom_mode():
    state = _custom_mode_state
    if state["thread"] is not None:
        state["stop"].set()
        state["thread"].join()
    left = state["active"]
    # Do not change FSM state while leaving presentation-only greet mode.
    if left != "greet":
        damp_mode()
    state.update(active=None, stop=None, thread=None)
    return {"exited": left}


def toggle_custom_mode(mode_name, language="en", voice=None, headlight_color="green"):
    """Announce a custom mode and hold its headlight. Motion-owning modes may
    switch FSM; greet deliberately never does. Exiting greet only stops/off."""
    if _custom_mode_state["active"] == mode_name:
        return _exit_custom_mode()
    if _custom_mode_state["active"] is not None:
        _exit_custom_mode()
    spec = _custom_modes[mode_name]
    say(spec["announce"], language=language)
    fsm_code = None if spec["fsm"] is None else _rpc_code(_loco_client().SetFsmId(FSM_IDS[spec["fsm"]]))
    rgb = parse_color(headlight_color)
    stop_event = threading.Event()

    def worker():
        client = _audio_client()
        while not stop_event.is_set():
            client.LedControl(*rgb)
            stop_event.wait(0.2)
        client.LedControl(0, 0, 0)

    thread = threading.Thread(target=worker, daemon=False)
    thread.start()
    _custom_mode_state.update(active=mode_name, stop=stop_event, thread=thread)
    return {"mode": mode_name, "fsm_code": fsm_code}


# ======================================================================
# Task 6 — basic locomotion helpers
# ======================================================================

_locomotion_lock = threading.Lock()


def _odom_pose():
    msg = _get_odom_sub().message
    if msg is None:
        return None
    return (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))


def loco_stop():
    loco = _loco_client()
    if hasattr(loco, "StopMove"):
        return loco.StopMove()
    return loco.Move(0.0, 0.0, 0.0, continous_move=False)


def loco_move(vx, vy, vyaw, duration_s=2.0):
    """A continuous velocity command held for duration_s, always cancelled in
    `finally` so an exception mid-sleep can never leave the robot moving."""
    if not _locomotion_lock.acquire(blocking=False):
        raise RuntimeError("Another locomotion command is already in progress.")
    try:
        code = int(_loco_client().Move(float(vx), float(vy), float(vyaw), continous_move=True) or 0)
        try:
            time.sleep(max(0.0, float(duration_s)))
        finally:
            loco_stop()
        return code
    finally:
        _locomotion_lock.release()


def odom_move(target_dx, target_dy, target_dyaw, pos_tol_m=0.05, yaw_tol_rad=0.05, timeout_s=15.0, poll_s=0.2):
    """One non-continuous relative Move, closing the loop on odometry instead
    of trusting the RPC return code alone as proof of arrival."""
    if not _locomotion_lock.acquire(blocking=False):
        raise RuntimeError("Another locomotion command is already in progress.")
    try:
        start = _odom_pose()
        if start is None:
            raise RuntimeError("No fresh odometry; cannot monitor arrival.")
        code = int(_loco_client().Move(float(target_dx), float(target_dy), float(target_dyaw), continous_move=False) or 0)
        if code != 0:
            return {"code": code, "arrived": False, "reason": "move request rejected"}
        deadline = time.time() + timeout_s
        pose = None
        while time.time() < deadline:
            pose = _odom_pose()
            if pose is not None:
                dx, dy, dyaw = pose[0] - start[0], pose[1] - start[1], pose[2] - start[2]
                err_pos = ((target_dx - dx) ** 2 + (target_dy - dy) ** 2) ** 0.5
                err_yaw = abs(((target_dyaw - dyaw) + math.pi) % (2 * math.pi) - math.pi)
                if err_pos <= pos_tol_m and err_yaw <= yaw_tol_rad:
                    return {"code": code, "arrived": True, "moved": (dx, dy, dyaw)}
            time.sleep(poll_s)
        loco_stop()
        moved = None if pose is None else (pose[0] - start[0], pose[1] - start[1], pose[2] - start[2])
        return {"code": code, "arrived": False, "reason": "timeout", "moved": moved}
    finally:
        _locomotion_lock.release()


# ======================================================================
# Task 7 — SLAM operation and map visualization
# ======================================================================
# Deployed/verified RPC ids: 1801 start mapping, 1802/1901 stop mapping,
# 1804 relocate/init-pose, 1102 single-pose navigation.

_map_path = "/home/unitree/test.pcd"
_points_path = Path("slam_points.json")
_points = {}
SLAM_POINT_TOPICS = [
    "rt/unitree/slam_mapping/points", "rt/unitree/slam_relocation/points",
    "rt/unitree/slam_relocation/global_map", "rt/unitree/slam_relocation/web_points",
]
_map_snapshot_dir = Path("slam_map_snapshots")
_last_slam_notice = None


def _parse_slam_status(raw):
    """errorCode/info/is_arrived/obsInfo envelope, beyond the bare pose
    current_pose() extracts. `data.obsInfo.state` is the obstacle-blocked
    flag; `data.is_arrived` is what keyDemo.cpp's taskLoopFun waits on.
    Verified against a live rt/slam_info capture (dev/slam_viz_in_jupyter.ipynb,
    Inspire_hands/topics.md). Every field optional; never raises."""
    try:
        payload = json.loads(raw) if raw else None
    except Exception:
        return None
    if not isinstance(payload, dict):
        return None
    data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
    obs_info = data.get("obsInfo") if isinstance(data.get("obsInfo"), dict) else {}
    return {
        "type": payload.get("type"),
        "error_code": int(payload.get("errorCode", 0) or 0),
        "info": payload.get("info"),
        "is_arrived": data.get("is_arrived"),
        "target_node_name": data.get("targetNodeName"),
        "obstacle_blocked": bool(obs_info["state"]) if "state" in obs_info else None,
    }


class SlamRpc(Client):
    def __init__(self):
        super().__init__("slam_operate", False)
        for api_id in (1801, 1802, 1804, 1102, 1901):
            self._RegistApi(api_id, 0)
        self._SetApiVerson("1.0.0.1")

    def _call_json(self, api_id, payload):
        code, data = self._Call(api_id, json.dumps(payload))
        return int(code), data

    def start_mapping(self, slam_type="indoor"):
        return self._call_json(1801, {"data": {"slam_type": slam_type}})

    def stop_mapping(self, save_path=None):
        if save_path:
            return self._call_json(1802, {"data": {"address": save_path}})
        return self._call_json(1901, {"data": {}})

    def init_pose(self, x, y, yaw, address):
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        return self._call_json(1804, {"data": {"x": x, "y": y, "z": 0.0, "q_x": 0.0, "q_y": 0.0, "q_z": qz, "q_w": qw, "address": address}})

    def pose_nav(self, x, y, yaw):
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        return self._call_json(1102, {"data": {"targetPose": {"x": x, "y": y, "z": 0.0, "q_x": 0.0, "q_y": 0.0, "q_z": qz, "q_w": qw}, "mode": 1}})


def current_pose():
    """An all-zero pose means "no valid pose yet", not "robot at the origin" —
    treated as absent. As a side effect, also refreshes _last_slam_notice
    (errorCode/info/is_arrived/obsInfo) from whichever sub has a message,
    so wait_for_arrival()/navigate_path() can see obstacle/arrival status
    without a second subscription."""
    global _last_slam_notice
    result = None
    for sub in _get_slam_info_subs():
        if sub.message is None:
            continue
        raw = sub.message.data
        notice = _parse_slam_status(raw)
        if notice is not None:
            _last_slam_notice = {**notice, "stamp": sub.timestamp}
        if result is not None:
            continue
        try:
            payload = json.loads(raw)
            cur = payload.get("data", {}).get("currentPose", {})
            x, y = float(cur["x"]), float(cur["y"])
            qz, qw = float(cur.get("q_z", 0.0)), float(cur.get("q_w", 1.0))
            yaw = math.atan2(2 * qw * qz, 1 - 2 * qz * qz)
        except Exception:
            continue
        if abs(x) < 1e-5 and abs(y) < 1e-5 and abs(yaw) < 1e-5:
            continue
        result = (x, y, yaw)
    return result


def wait_for_arrival(point_name, tolerance_m=0.35, timeout_s=120.0):
    """Waits for point_name to be reached, preferring the robot's own
    `is_arrived` confirmation (_last_slam_notice, refreshed by current_pose())
    over the xy-distance check -- keyDemo.cpp's taskLoopFun waits on that
    same flag rather than computing distance itself. Distance remains a
    fallback for firmware that never publishes it, and to bound the wait.
    Returns the last notice seen (e.g. obstacle_blocked) either way."""
    target = _points[point_name]
    send_ts, deadline = time.time(), time.time() + timeout_s
    last_notice = None
    while time.time() < deadline:
        pose = current_pose()
        if _last_slam_notice is not None and _last_slam_notice.get("stamp", 0.0) >= send_ts:
            last_notice = _last_slam_notice
            if last_notice.get("is_arrived") is True:
                return {"arrived": True, "pose": pose, "notice": last_notice}
        if pose is not None and math.hypot(pose[0] - target[0], pose[1] - target[1]) <= tolerance_m:
            return {"arrived": True, "pose": pose, "notice": last_notice}
        time.sleep(0.2)
    return {"arrived": False, "pose": current_pose(), "notice": last_notice}


def start_mapping():
    return _slam_rpc().start_mapping("indoor")


def stop_mapping(save_path=None):
    return _slam_rpc().stop_mapping(save_path or _map_path)


def relocate():
    pose = current_pose()
    if pose is None:
        return {
            "ok": False,
            "error": "No SLAM pose received yet. Start mapping/relocation, or pass an explicit pose in the notebook helper.",
            "topics": ["rt/slam_info", "rt/slam_key_info"],
        }
    return _slam_rpc().init_pose(*pose, address=_map_path)


def _load_points():
    global _points
    _points = json.loads(_points_path.read_text()) if _points_path.exists() else {}
    return _points


def _save_points():
    tmp = _points_path.with_suffix(".tmp")
    tmp.write_text(json.dumps(_points, indent=2))
    import os
    os.replace(tmp, _points_path)


def add_point(point_name):
    pose = current_pose()
    if pose is None:
        raise RuntimeError("No valid SLAM pose; relocate first.")
    _points[point_name] = pose
    _save_points()
    return pose


def remove_point(point_name):
    _points.pop(point_name, None)
    _save_points()


def navigate_to_point(point_name):
    x, y, yaw = _points[point_name]
    return _slam_rpc().pose_nav(x, y, yaw)


def navigate_path(point_names, native_path_callback=None, tolerance_m=0.35, timeout_s=120.0):
    """Visits point_names in order.

    There is no deployed multi-point "path" RPC -- the verified SLAM RPC ids
    are start/stop mapping, relocate, and single-pose navigation (1102). So
    by default this calls navigate_to_point()+wait_for_arrival() once per
    point, stopping at the first that fails to send or isn't reached. This
    is not a fallback hack: it's the same sequential approach Unitree's own
    reference SLAM demo uses (unitree_sdk2/example keyDemo.cpp's
    taskLoopFun loops over pose_nav the same way). Pass native_path_callback
    only to override this with a genuine single-RPC path API, if one is
    ever verified to exist."""
    if native_path_callback is not None:
        return native_path_callback([_points[name] for name in point_names])
    results = []
    for name in point_names:
        code, raw = navigate_to_point(name)
        entry = {"name": name, "code": code, "raw": raw}
        if int(code) == 0:
            entry["arrival"] = wait_for_arrival(name, tolerance_m=tolerance_m, timeout_s=timeout_s)
        results.append(entry)
        if int(code) != 0 or not entry.get("arrival", {}).get("arrived", True):
            break
    return results


def _decode_xy(msg, max_points=20000):
    fields = {f.name.lower(): f for f in msg.fields}
    x_off, y_off = fields["x"].offset, fields["y"].offset
    raw, step = bytes(msg.data), msg.point_step
    total = min(msg.width * msg.height, len(raw) // max(1, step))
    stride = max(1, total // max_points)
    xs, ys = [], []
    for i in range(0, total, stride):
        base = i * step
        xs.append(struct.unpack_from("<f", raw, base + x_off)[0])
        ys.append(struct.unpack_from("<f", raw, base + y_off)[0])
    return xs, ys


def capture_map_snapshot():
    """Call periodically while start_mapping() is active — once a map is
    saved on the mainboard it may no longer be accessible for visualization,
    so this saves 2-D scatter PNGs *during* mapping instead."""
    import matplotlib.pyplot as plt
    _map_snapshot_dir.mkdir(exist_ok=True)
    subs = _get_cloud_subs()
    candidates = [(t, s) for t, s in subs if s.message is not None]
    if not candidates:
        raise RuntimeError("No SLAM point-cloud message received yet; is mapping running?")
    topic, sub = max(candidates, key=lambda pair: pair[1].timestamp)
    xs, ys = _decode_xy(sub.message)
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.scatter(xs, ys, s=0.5)
    ax.set_title(f"SLAM snapshot ({topic})")
    ax.set_aspect("equal")
    path = _map_snapshot_dir / f"map_{int(time.time())}.png"
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def view_map():
    snapshots = sorted(_map_snapshot_dir.glob("map_*.png")) if _map_snapshot_dir.exists() else []
    return snapshots[-1] if snapshots else None


# ======================================================================
# Task 8 — high-level arm gestures, teach/repeat, low-level pose interpolation
# ======================================================================

HL_ARM_ACTIONS = {"release arm": 99, "clap": 17, "face wave": 25, "high wave": 26, "shake hand": 27, "hug": 19}
WAIST_JOINTS = (12, 13, 14)
LEFT_ARM_JOINTS = list(range(15, 22))
RIGHT_ARM_JOINTS = list(range(22, 29))
ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
UPPER_BODY_JOINTS = list(WAIST_JOINTS) + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
_MODULE_DIR = Path(__file__).resolve().parent
_ll_pose_store_path = Path(
    os.environ.get("G1_LL_POSE_PATH", str(_MODULE_DIR / "ll_poses.json"))
).expanduser()
_ll_poses = {}
_sequences_path = Path(
    os.environ.get("G1_ARM_SEQUENCE_PATH", str(_MODULE_DIR / "arm_sequences.json"))
).expanduser()
_sequences = {}


def _finite_float(value, name, *, minimum=None, maximum=None):
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    if minimum is not None and parsed < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    if maximum is not None and parsed > maximum:
        raise ValueError(f"{name} must be <= {maximum}")
    return parsed


def _positive_steps(value, name="steps"):
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _atomic_write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def exec_arm_action(name, release_after_s=None):
    """G1ArmActionClient.ExecuteAction(action_id) — a documented, firmware-baked
    gesture. Must never run concurrently with a low-level rt/arm_sdk stream."""
    code = int(_arm_action_client().ExecuteAction(HL_ARM_ACTIONS[name]))
    if release_after_s is not None:
        time.sleep(release_after_s)
        return int(_arm_action_client().ExecuteAction(HL_ARM_ACTIONS["release arm"]))
    return code


def clap():
    return exec_arm_action("clap")


def face_wave():
    return exec_arm_action("face wave")


def shake_hand():
    return exec_arm_action("shake hand", release_after_s=2.0)


def current_upper_body_pose(timeout_s=3.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _get_lowstate_sub().message is not None:
            return {j: float(_get_lowstate_sub().message.motor_state[j].q) for j in UPPER_BODY_JOINTS}
        time.sleep(0.02)
    raise TimeoutError("No fresh rt/lowstate.")


def write_arm_sdk_pose(targets, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0):
    """Writes a complete rt/arm_sdk LowCmd_ frame, including the weight byte
    at motor_cmd[29] that arbitrates between the default controller and this
    publisher (0 = released, 1 = this publisher owns the arm/waist joints)."""
    weight = _finite_float(weight, "weight", minimum=0.0, maximum=1.0)
    kp = _finite_float(kp, "kp", minimum=0.0)
    kd = _finite_float(kd, "kd", minimum=0.0)
    waist_kp = _finite_float(waist_kp, "waist_kp", minimum=0.0)
    waist_kd = _finite_float(waist_kd, "waist_kd", minimum=0.0)
    normalized_targets = {}
    for joint, q in targets.items():
        joint = int(joint)
        if joint not in UPPER_BODY_JOINTS:
            raise ValueError(f"arm_sdk target joint is outside the upper body: {joint}")
        normalized_targets[joint] = _finite_float(q, f"joint {joint} target")

    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = 0
    msg.motor_cmd[29].q = weight
    for joint, q in normalized_targets.items():
        cmd = msg.motor_cmd[joint]
        cmd.mode = 1
        cmd.q = float(q)
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.kp = waist_kp if joint in WAIST_JOINTS else kp
        cmd.kd = waist_kd if joint in WAIST_JOINTS else kd
    msg.crc = _arm_sdk_crc().Crc(msg)
    _arm_sdk_pub().Write(msg)


_zero_stiffness_state = {
    "thread": None,
    "stop": None,
    "arm": None,
    "frames": 0,
    "error": None,
}


def _stop_zero_stiffness_stream():
    state = _zero_stiffness_state
    if state["stop"] is not None:
        state["stop"].set()
        thread = state["thread"]
        if thread is not None:
            thread.join(timeout=4.0)
            if thread.is_alive():
                raise RuntimeError("Zero-stiffness worker did not stop; refusing a competing arm command")
    result = {"arm": state["arm"], "frames": state["frames"], "error": state["error"]}
    state.update(thread=None, stop=None, arm=None, frames=0, error=None)
    return result


def arms_restore_stiffness():
    """Stop free-move mode and hold the current upper-body pose normally."""
    result = _stop_zero_stiffness_stream()
    pose = current_upper_body_pose()
    write_arm_sdk_pose(pose, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0)
    result["final_pose"] = pose
    return result


def arms_zero_stiffness(rate_hz=50.0, arm="both", handoff=True):
    """Keep one or both arms backdrivable until arms_restore_stiffness().

    This matches ``modules.sdk_client.Robot.teach``: arm joints use zero
    gains, while the waist keeps its normal hold gains for stability.
    """
    state = _zero_stiffness_state
    rate_hz = _finite_float(rate_hz, "rate_hz", minimum=1.0, maximum=200.0)
    arm = str(arm).lower()
    joints = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS, "both": ARM_JOINTS}.get(arm)
    if joints is None:
        raise ValueError("arm must be 'left', 'right', or 'both'")
    if state["thread"] is not None and state["thread"].is_alive():
        if state["arm"] != arm:
            raise RuntimeError("Free-move mode is already active for a different arm selection")
        return {"active": True, "arm": arm, "frames": state["frames"]}
    if handoff:
        release_arms()
        engage_arms()
    waist_hold = current_upper_body_pose()
    stop = threading.Event()
    interval_s = 1.0 / rate_hz
    state.update(stop=stop, arm=arm, frames=0, error=None)

    def worker():
        try:
            while not stop.is_set():
                pose = current_upper_body_pose()
                if stop.is_set():
                    break
                targets = {joint: waist_hold[joint] for joint in WAIST_JOINTS}
                targets.update({joint: pose[joint] for joint in joints})
                write_arm_sdk_pose(
                    targets, weight=1.0, kp=0.0, kd=0.0,
                    waist_kp=480.0, waist_kd=12.0,
                )
                state["frames"] += 1
                stop.wait(interval_s)
        except Exception as exc:
            state["error"] = str(exc)
            stop.set()

    state["thread"] = threading.Thread(target=worker, name="arms-zero-stiffness", daemon=True)
    state["thread"].start()
    return {"active": True, "arm": arm, "joints": joints, "kp": 0.0, "kd": 0.0, "waist_kp": 480.0}


def _ease(ratio):
    return ratio * ratio * (3 - 2 * ratio)


def release_arms(steps=150, rate_hz=50.0):
    """Ramp the arm_sdk weight 1 -> 0 on an ease curve, handing control back
    to the default controller. Run this before this publisher stops owning
    rt/arm_sdk."""
    steps = _positive_steps(steps)
    rate_hz = _finite_float(rate_hz, "rate_hz", minimum=1.0, maximum=200.0)
    # A zero-stiffness worker must never publish during this ownership ramp.
    _stop_zero_stiffness_stream()
    pose = current_upper_body_pose()
    for i in range(steps + 1):
        weight = 1.0 - _ease(i / steps)
        write_arm_sdk_pose(pose, weight=weight, kp=30.0 * weight, kd=1.5 * weight, waist_kp=480.0 * weight, waist_kd=12.0 * weight)
        time.sleep(1.0 / rate_hz)
    return {"final_weight": 0.0}


def engage_arms(steps=50, rate_hz=50.0):
    """Ramp the arm_sdk weight 0 -> 1, taking ownership of the arm/waist joints."""
    steps = _positive_steps(steps)
    rate_hz = _finite_float(rate_hz, "rate_hz", minimum=1.0, maximum=200.0)
    pose = current_upper_body_pose()
    for i in range(steps + 1):
        write_arm_sdk_pose(pose, weight=i / steps)
        time.sleep(1.0 / rate_hz)
    return {"final_weight": 1.0}


def _load_ll_poses():
    global _ll_poses
    _ll_poses = json.loads(_ll_pose_store_path.read_text()) if _ll_pose_store_path.exists() else {}
    return _ll_poses


def _save_ll_poses():
    _atomic_write_json(_ll_pose_store_path, _ll_poses)


def save_current_ll_pose(name):
    pose = current_upper_body_pose()
    _ll_poses[str(name)] = pose
    _save_ll_poses()
    return pose


def interpolate_to_ll_pose(name_or_pose, duration_s=4.0, steps=150):
    """Smoothly interpolate from the current pose to a saved (by name) or
    literal upper-body pose, using the same ease curve as release/engage_arms."""
    target = _ll_poses[name_or_pose] if isinstance(name_or_pose, str) else name_or_pose
    target = {int(j): float(q) for j, q in target.items()}
    start = current_upper_body_pose()
    for step in range(1, steps + 1):
        smooth = _ease(step / steps)
        frame = {j: start[j] + (target[j] - start[j]) * smooth for j in target}
        write_arm_sdk_pose(frame)
        time.sleep(duration_s / steps)
    return {"target": target, "steps": steps}


def extend_arm_forward(side="right", duration_s=4.0):
    return interpolate_to_ll_pose(f"extended_{side}", duration_s=duration_s)


def _load_sequences():
    global _sequences
    _sequences = json.loads(_sequences_path.read_text()) if _sequences_path.exists() else {}
    if not isinstance(_sequences, dict):
        raise ValueError(f"Arm sequence store must contain a JSON object: {_sequences_path}")
    return _sequences


def _save_sequences():
    _atomic_write_json(_sequences_path, _sequences)


def _arm_joints(arm):
    joints = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS, "both": ARM_JOINTS}.get(str(arm).lower())
    if joints is None:
        raise ValueError("arm must be 'left', 'right', or 'both'")
    return joints


def _sequence_name(value):
    name = str(value).strip()
    if not name or len(name) > 128 or any(ord(char) < 32 for char in name):
        raise ValueError("sequence_name must be 1-128 printable characters")
    return name


def _validate_trajectory(sequence):
    if not isinstance(sequence, dict) or sequence.get("format") != "trajectory_v1":
        raise ValueError("Re-record this sequence with teach(); legacy waypoint sequences are not safe to replay.")
    arm = str(sequence.get("arm", "")).lower()
    expected_joints = list(_arm_joints(arm))
    try:
        joints = [int(joint) for joint in sequence["joints"]]
        timestamps = [float(timestamp) for timestamp in sequence["timestamps"]]
        raw_frames = sequence["frames"]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Recorded sequence has invalid trajectory fields") from exc
    if not isinstance(raw_frames, list):
        raise ValueError("Recorded frames must be a list")
    if joints != expected_joints:
        raise ValueError(f"Recorded joints do not match the declared {arm!r} arm selection")
    if not timestamps or len(raw_frames) != len(timestamps):
        raise ValueError("Recorded sequence has invalid frames or timestamps")
    if any(not math.isfinite(timestamp) or timestamp < 0.0 for timestamp in timestamps):
        raise ValueError("Recorded timestamps must be finite and non-negative")
    if any(current <= previous for previous, current in zip(timestamps, timestamps[1:])):
        raise ValueError("Recorded timestamps must be strictly increasing")
    first_timestamp = timestamps[0]
    timestamps = [timestamp - first_timestamp for timestamp in timestamps]
    frames = []
    for frame_index, frame in enumerate(raw_frames):
        if not isinstance(frame, dict):
            raise ValueError(f"Recorded frame {frame_index} must be an object")
        normalized = {}
        for joint in joints:
            raw_value = frame.get(str(joint), frame.get(joint))
            if raw_value is None:
                raise ValueError(f"Recorded frame {frame_index} is missing joint {joint}")
            normalized[joint] = _finite_float(
                raw_value,
                f"recorded frame {frame_index} joint {joint}",
            )
        frames.append(normalized)
    return arm, joints, timestamps, frames


def teach(sequence_name, reset=False, arm="both", rate_hz=50.0):
    """Record until Enter is pressed, while the selected arms stay backdrivable."""
    sequence_name = _sequence_name(sequence_name)
    arm = str(arm).lower()
    joints = _arm_joints(arm)
    rate_hz = _finite_float(rate_hz, "rate_hz", minimum=1.0, maximum=200.0)
    if sequence_name in _sequences and not reset:
        raise ValueError("Sequence already exists; pass reset=True to replace it")
    arms_zero_stiffness(arm=arm)
    done = threading.Event()

    def wait_for_enter():
        try:
            input("Move the arm, then press Enter to finish recording... ")
        except EOFError:
            pass
        done.set()

    threading.Thread(target=wait_for_enter, name="teach-enter", daemon=True).start()
    interval_s = 1.0 / rate_hz
    start = time.monotonic()
    next_report = start + 1.0
    timestamps, frames = [], []
    try:
        while not done.is_set():
            elapsed = time.monotonic() - start
            pose = current_upper_body_pose()
            timestamps.append(elapsed)
            frames.append({joint: pose[joint] for joint in joints})
            if time.monotonic() >= next_report:
                print(f"[teach] {len(frames)} frames, {elapsed:.1f}s saved")
                next_report += 1.0
            done.wait(interval_s)
        if not frames:
            raise RuntimeError("No teach frames were captured.")
        _sequences[sequence_name] = {
            "format": "trajectory_v1", "arm": arm, "joints": joints,
            "timestamps": timestamps, "frames": frames,
        }
        _save_sequences()
        duration_s = timestamps[-1]
        print(f"[teach] complete: {len(frames)} frames over {duration_s:.1f}s")
    finally:
        release_arms()
    return {
        "sequence": sequence_name, "frames": len(frames), "duration_s": duration_s,
        "arm": arm, "released_to_ai": True,
    }


def repeat(sequence_name, speed=1.0, rate_hz=50.0, start_ramp_s=0.8, final_hold_s=0.8, max_joint_speed=0.45):
    """Replay a recorded trajectory with a safe ramp, speed limit, final hold, and release."""
    sequence_name = _sequence_name(sequence_name)
    if sequence_name not in _sequences:
        raise KeyError(f"Unknown arm sequence: {sequence_name}")
    arm, joints, timestamps, frames = _validate_trajectory(_sequences[sequence_name])
    speed = _finite_float(speed, "speed", minimum=1e-3)
    rate_hz = _finite_float(rate_hz, "rate_hz", minimum=1.0, maximum=200.0)
    start_ramp_s = _finite_float(start_ramp_s, "start_ramp_s", minimum=0.0)
    final_hold_s = _finite_float(final_hold_s, "final_hold_s", minimum=0.0, maximum=60.0)
    max_joint_speed = _finite_float(
        max_joint_speed,
        "max_joint_speed",
        minimum=1e-3,
        maximum=2.0,
    )
    dt = 1.0 / rate_hz
    release_arms()
    try:
        engage_arms()
        start_pose = current_upper_body_pose()
        waist_hold = {j: start_pose[j] for j in WAIST_JOINTS}
        first = frames[0]
        max_delta = max(abs(first[j] - start_pose[j]) for j in joints)
        # Smoothstep's peak slope is 1.5, so use that factor to make the
        # configured joint-speed limit a true peak limit, not only an average.
        ramp_s = max(start_ramp_s, 1.5 * max_delta / max_joint_speed)
        steps = max(1, math.ceil(ramp_s * rate_hz))
        for step in range(1, steps + 1):
            smooth = (step / steps) ** 2 * (3.0 - 2.0 * step / steps)
            target = dict(start_pose)
            target.update({j: start_pose[j] + (first[j] - start_pose[j]) * smooth for j in joints})
            write_arm_sdk_pose(target, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0)
            time.sleep(dt)
        started = time.monotonic()
        previous = {j: first[j] for j in joints}
        index = 0
        while True:
            elapsed = (time.monotonic() - started) * speed
            if elapsed >= timestamps[-1]:
                break
            while index + 1 < len(timestamps) and timestamps[index + 1] <= elapsed:
                index += 1
            next_index = min(index + 1, len(timestamps) - 1)
            span = max(1e-6, timestamps[next_index] - timestamps[index])
            alpha = 0.0 if next_index == index else (elapsed - timestamps[index]) / span
            desired = {j: frames[index][j] + (frames[next_index][j] - frames[index][j]) * alpha for j in joints}
            target = current_upper_body_pose()
            target.update(waist_hold)
            max_step = max_joint_speed * dt
            for j in joints:
                previous[j] += max(-max_step, min(max_step, desired[j] - previous[j]))
                target[j] = previous[j]
            write_arm_sdk_pose(target, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0)
            time.sleep(dt)
        final = frames[-1]
        # Reach the final frame through the same velocity limiter. Writing it
        # directly here could otherwise snap after a speed-limited replay.
        while True:
            target = current_upper_body_pose()
            target.update(waist_hold)
            reached = True
            for j in joints:
                delta = final[j] - previous[j]
                if abs(delta) > 1e-9:
                    reached = False
                previous[j] += max(-max_step, min(max_step, delta))
                target[j] = previous[j]
            write_arm_sdk_pose(target, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0)
            if reached:
                break
            time.sleep(dt)
        deadline = time.monotonic() + final_hold_s
        while time.monotonic() < deadline:
            target = current_upper_body_pose()
            target.update(waist_hold)
            target.update({j: previous[j] for j in joints})
            write_arm_sdk_pose(target, weight=1.0, kp=30.0, kd=1.5, waist_kp=480.0, waist_kd=12.0)
            time.sleep(dt)
    finally:
        release_arms()
    return {"sequence": sequence_name, "frames": len(frames), "duration_s": timestamps[-1] / speed, "arm": arm}


# ======================================================================
# Task 9 — low-level joint control (rt/lowcmd)
# ======================================================================

LOWCMD_JOINTS = list(range(0, 29))  # left_leg 0-5, right_leg 6-11, waist 12-14, left_arm 15-21, right_arm 22-28
DEFAULT_KP = [60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40]
DEFAULT_KD = [1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]


class LowCmdPublisher:
    def __init__(self):
        self.crc = CRC()
        self.pub = make_publisher("rt/lowcmd", LowCmd_)
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0

    def write(self, q, mode_machine, kp=None, kd=None, dq=0.0, tau=0.0):
        self.msg.mode_machine = int(mode_machine)
        kp = kp or DEFAULT_KP
        kd = kd or DEFAULT_KD
        for i in LOWCMD_JOINTS:
            cmd = self.msg.motor_cmd[i]
            cmd.mode = 1
            cmd.q = float(q[i])
            cmd.dq = float(dq)
            cmd.tau = float(tau)
            cmd.kp = float(kp[i])
            cmd.kd = float(kd[i])
        self.msg.crc = self.crc.Crc(self.msg)
        self.pub.Write(self.msg)


def require_fresh(sub, name, max_age_s=0.5):
    """Reusable freshness policy: every write below must refuse to command
    joints from a stale/absent lowstate snapshot instead of holding the
    last-known q forever."""
    if not sub.fresh(max_age_s=max_age_s):
        raise RuntimeError(f"{name} is not fresh; refusing to command.")


def current_q_mode(timeout_s=3.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _get_lowstate_sub().fresh(max_age_s=1.0):
            msg = _get_lowstate_sub().message
            return [float(msg.motor_state[i].q) for i in LOWCMD_JOINTS], int(msg.mode_machine)
        time.sleep(0.02)
    raise TimeoutError("Timed out waiting for fresh rt/lowstate.")


def move_ll_joint(joint_id, q, dq=0.0, kp=40.0, kd=1.0, tau=0.0):
    """Command a single joint while holding every other joint at its
    currently observed position — the same pattern sdk_wrapper.G1.move_ll_joint
    uses for its arm_sdk=False path."""
    require_fresh(_get_lowstate_sub(), "rt/lowstate")
    q_all, mode_machine = current_q_mode()
    kp_all, kd_all = list(DEFAULT_KP), list(DEFAULT_KD)
    q_all[joint_id] = float(q)
    kp_all[joint_id] = float(kp)
    kd_all[joint_id] = float(kd)
    _lowcmd_pub().write(q_all, mode_machine, kp=kp_all, kd=kd_all, dq=dq, tau=tau)


# ======================================================================
# Task 10 — end-effector IK control
# ======================================================================


def fk_solver(side):
    cache = _state["fk_cache"]
    if side not in cache:
        cache[side] = ArmFK(side, "urdf")
    return cache[side]


def ik_solver(side):
    cache = _state["ik_cache"]
    if side not in cache:
        cache[side] = ArmIK(side, "dls", max_iter=24, tol_pos_m=0.005, tol_rot_rad=0.02)
    return cache[side]


def ik_move_ee(hand, dx=0.0, dy=0.0, dz=0.0, max_speed=0.25, max_dq=0.15, rate_hz=50.0):
    """Position-only DLS IK step: solve a small Cartesian offset from the
    current end-effector pose, clip the joint delta to max_dq, then ramp to
    it in speed-limited, eased steps — never jump straight to the solved pose."""
    side = "right" if str(hand).lower().startswith("r") else "left"
    joints = RIGHT_ARM_JOINTS if side == "right" else LEFT_ARM_JOINTS
    current = current_upper_body_pose()
    q_init = np.array([current[j] for j in joints])
    fk = fk_solver(side)
    target_T = fk.compute_arm(q_init).copy()
    target_T[0, 3] += dx
    target_T[1, 3] += dy if side == "left" else -dy
    target_T[2, 3] += dz
    q_sol, info = ik_solver(side).solve(target_T, q_init=q_init)
    if q_sol is None:
        return {"success": False, "ik": info}
    delta = np.clip(np.asarray(q_sol) - q_init, -max_dq, max_dq)
    target_q = q_init + delta
    target = dict(current)
    for i, j in enumerate(joints):
        target[j] = float(target_q[i])
    remaining = max(abs(target[j] - current[j]) for j in joints)
    steps = max(1, int(np.ceil(remaining / max(1e-4, max_speed / rate_hz))))
    for step in range(1, steps + 1):
        smooth = _ease(step / steps)
        frame = dict(current)
        for j in joints:
            frame[j] = current[j] + (target[j] - current[j]) * smooth
        write_arm_sdk_pose(frame)
        time.sleep(1.0 / rate_hz)
    ee = tuple(float(x) for x in fk.compute_arm(target_q)[:3, 3])
    return {"success": True, "ik": info, "ee": ee, "steps": steps}


def current_palm_xyz(side="right"):
    joints = RIGHT_ARM_JOINTS if side == "right" else LEFT_ARM_JOINTS
    current = current_upper_body_pose()
    q = np.array([current[j] for j in joints])
    return tuple(float(x) for x in fk_solver(side).compute_arm(q)[:3, 3])


def ik_increment(dx, dy, dz, side="right"):
    """The exact 3-argument shape `DeliveryPipeline.execute_incremental_ik`
    (Task 12/13) expects for its `ik_increment` callback."""
    return ik_move_ee(side, dx=dx, dy=dy, dz=dz)


# ======================================================================
# Task 11 — Dex3 hands: write_hand, tactile baseline, gradual close/open
# ======================================================================

HAND_CMD_TOPICS = {"left": "rt/dex3/left/cmd", "right": "rt/dex3/right/cmd"}
HAND_STATE_TOPICS = {"left": "rt/dex3/left/state", "right": "rt/dex3/right/state"}
_last_hand_targets = {}


def write_hand(targets, side="right", kp=0.8, kd=0.05, tau=0.02):
    msg = unitree_hg_msg_dds__HandCmd_()
    for i, q in enumerate(targets):
        cmd = msg.motor_cmd[i]
        cmd.mode = (i & 0x0F) | (1 << 4)
        cmd.q = float(q)
        cmd.dq = 0.0
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd
    _hand_pubs()[side].Write(msg)
    _last_hand_targets[side] = list(targets)


def open_hand(side="right"):
    return write_hand(HAND_OPEN[side], side=side)


def close_hand(side="right"):
    return write_hand(HAND_CLOSED[side], side=side)


def tactile_samples(side="right"):
    msg = _hand_state_subs()[side].message
    if msg is None:
        return []
    return [float(v) for sensor in msg.press_sensor_state for v in sensor.pressure]


def measure_baseline(side="right", duration_s=2.0):
    """Record no-object noise before picking any grasp-contact threshold —
    thresholds are empirical per installation/object class, never a constant
    copied from another robot."""
    samples = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        samples.extend(tactile_samples(side))
        time.sleep(0.02)
    if not samples:
        raise RuntimeError("No tactile samples received; is the hand publishing state?")
    return {"max": max(samples), "mean": sum(samples) / len(samples), "n": len(samples)}


def gradual_close(threshold, side="right", steps=40, delay_s=0.05):
    start = _last_hand_targets.get(side, HAND_OPEN[side])
    for step in range(1, steps + 1):
        a = step / steps
        frame = [x + (y - x) * a for x, y in zip(start, HAND_CLOSED[side])]
        write_hand(frame, side=side)
        time.sleep(delay_s)
        samples = tactile_samples(side)
        if samples and max(samples) >= threshold:
            return {"contact": True, "step": step, "side": side}
    return {"contact": False, "step": steps, "side": side}


def gradual_open(side="right", steps=40, delay_s=0.05):
    """Symmetric release: ramp from whatever pose is currently commanded
    (possibly a partial grasp) back to HAND_OPEN. No tactile early-stop —
    opening should be gentle, not stop-on-contact."""
    start = _last_hand_targets.get(side, HAND_CLOSED[side])
    for step in range(1, steps + 1):
        a = step / steps
        frame = [x + (y - x) * a for x, y in zip(start, HAND_OPEN[side])]
        write_hand(frame, side=side)
        time.sleep(delay_s)
    return {"opened": True, "side": side}


# ======================================================================
# Task 12 — perception + pose-estimation pipeline (usage, not implementation)
# ======================================================================
# `util.DeliveryPipeline` is supplied, calibrated infrastructure — OpenAI
# vision detection, ArUco pose estimation, camera-to-base / wrist-to-palm
# transforms. This task is about validating its inputs/outputs, not
# rebuilding the algorithms.


def make_pipeline(camera_matrix, distortion, camera_to_base, wrist_to_palm):
    from util import DeliveryPipeline
    return DeliveryPipeline(camera_matrix, distortion, camera_to_base, wrist_to_palm)


def validated_target(pipeline, frame, marker_length_m, min_confidence=0.5, max_age_s=1.0, prompt="the delivery package"):
    """Perception output must never drive motion unchecked: reject a stale
    frame and a low-confidence detection instead of quietly acting on a guess."""
    import cv2
    if time.time() - frame.get("timestamp", time.time()) > max_age_s:
        raise RuntimeError("RGB-D frame is stale.")
    rgb = cv2.imdecode(np.frombuffer(frame["rgb_jpeg"], dtype=np.uint8), cv2.IMREAD_COLOR)
    detection = pipeline.detect_object(frame["rgb_jpeg"], prompt)
    if detection.confidence < min_confidence:
        raise RuntimeError(f"Low-confidence detection ({detection.confidence}); refusing to act.")
    marker_pose = pipeline.aruco_pose(rgb, marker_length_m)
    return pipeline.marker_to_palm_target(marker_pose)


# ======================================================================
# Task 13 — AcademyRobot: everything above, one consolidated object
# ======================================================================
# Every method below is a thin wrapper around a Task 2-12 function already
# defined in this file — proof that, piece by piece, this file is a from-
# scratch rebuild of the shape of sdk_wrapper.G1.


class AcademyRobot:
    def __init__(self, iface="eth0", domain_id=0, calibration=None):
        init(domain_id, iface)
        self.pipeline = make_pipeline(*calibration) if calibration is not None else None
        _state["academy_robot"] = self

    # -- state ------------------------------------------------------------
    upper_body_pose = staticmethod(current_upper_body_pose)
    current_pose = staticmethod(current_pose)

    # -- arm ----------------------------------------------------------------
    write_arm_sdk_pose = staticmethod(write_arm_sdk_pose)
    arms_zero_stiffness = staticmethod(arms_zero_stiffness)
    arms_restore_stiffness = staticmethod(arms_restore_stiffness)
    engage_arms = staticmethod(engage_arms)
    release_arms = staticmethod(release_arms)
    interpolate_to_ll_pose = staticmethod(interpolate_to_ll_pose)
    extend_arm_forward = staticmethod(extend_arm_forward)
    ik_move_ee = staticmethod(ik_move_ee)
    ik_increment = staticmethod(ik_increment)
    current_palm_xyz = staticmethod(current_palm_xyz)

    # -- hand -----------------------------------------------------------------
    write_hand = staticmethod(write_hand)
    gradual_close = staticmethod(gradual_close)
    gradual_open = staticmethod(gradual_open)

    # -- slam -----------------------------------------------------------
    navigate_to_point = staticmethod(navigate_to_point)


def deliver(pickup_point, dropdown_point, side="right", marker_length_m=0.04, grasp_threshold=0.5, prompt="the delivery package"):
    """The 8-step flow from `notes.txt` section 11, entirely built from
    functions already defined above in this file:

    1. extend_arm_forward  -> a saved pre-grasp pose            (Task 8)
    2. gradual_open                                              (Task 11)
    3. perception + pose estimation                              (Task 12)
    4. ik_increment in small steps via execute_incremental_ik     (Task 10)
    5. gradual_close at the grip pose                             (Task 11)
    6. interpolate_to_ll_pose("stable_hold_pose")                 (Task 8)
    7. navigate_to_point(dropdown_point)                          (Task 7)
    8. IK to the drop target, gradual_open, extend_arm_forward, release_arms
    """
    import cv2
    robot = _state["academy_robot"]
    if robot is None or robot.pipeline is None:
        raise RuntimeError("Construct an AcademyRobot with calibration before calling deliver().")

    extend_arm_forward(side=side)
    gradual_open(side=side)

    frame = get_rgbd()
    if time.time() - frame["timestamp"] > 1.0:
        raise RuntimeError("RGB-D frame is stale.")
    rgb = cv2.imdecode(np.frombuffer(frame["rgb_jpeg"], dtype=np.uint8), cv2.IMREAD_COLOR)
    detection = robot.pipeline.detect_object(frame["rgb_jpeg"], prompt)
    if detection.confidence < 0.5:
        raise RuntimeError(f"Low-confidence detection ({detection.confidence}); aborting.")
    marker_pose = robot.pipeline.aruco_pose(rgb, marker_length_m)
    pickup_target = robot.pipeline.marker_to_palm_target(marker_pose)

    robot.pipeline.execute_incremental_ik(ik_increment, current_palm_xyz(side), pickup_target, side=side)
    gradual_close(grasp_threshold, side=side)
    interpolate_to_ll_pose("stable_hold_pose", duration_s=3.0)
    navigate_to_point(dropdown_point)

    frame = get_rgbd()
    rgb = cv2.imdecode(np.frombuffer(frame["rgb_jpeg"], dtype=np.uint8), cv2.IMREAD_COLOR)
    dropdown_marker = robot.pipeline.aruco_pose(rgb, marker_length_m)
    dropdown_target = robot.pipeline.marker_to_palm_target(dropdown_marker)
    robot.pipeline.execute_incremental_ik(ik_increment, current_palm_xyz(side), dropdown_target, side=side)
    gradual_open(side=side)
    extend_arm_forward(side=side)
    release_arms()
    return {"pickup": pickup_point, "dropdown": dropdown_point, "side": side}


# ======================================================================
# Lazy client/publisher/subscriber cache + init()
# ======================================================================
# Everything above reads/writes through this one dict of module state
# instead of loose globals, so `init()` has one obvious place to populate
# and every accessor has one obvious place to look.

_state = {
    "lowstate_sub": None,
    "odom_sub": None,
    "slam_info_subs": None,
    "bms_subs": None,
    "cloud_subs": None,
    "hand_pubs": None,
    "hand_state_subs": None,
    "fk_cache": {},
    "ik_cache": {},
    "loco": None,
    "switcher": None,
    "audio": None,
    "arm_action": None,
    "robot_state": None,
    "arm_sdk_pub": None,
    "arm_sdk_crc": None,
    "lowcmd_pub": None,
    "slam_rpc": None,
    "academy_robot": None,
}


def init(domain_id=0, iface="eth0"):
    """Call once per kernel/process before using anything else in this module
    — the equivalent of running every task notebook's Task-2-style setup cell.
    Safe to call again with the same (domain_id, iface): later calls are a
    no-op once `rt/lowstate`/`rt/odommodestate` subscribers already exist."""
    ensure_channel_factory(domain_id, iface)
    if _state["lowstate_sub"] is None:
        _state["lowstate_sub"] = Latest("rt/lowstate", LowState_)
    if _state["odom_sub"] is None:
        _state["odom_sub"] = Latest("rt/odommodestate", SportModeState_)
    _load_points()
    _load_ll_poses()
    _load_sequences()
    return _state


def _require_init():
    if _state["lowstate_sub"] is None:
        raise RuntimeError("Call lib_solved.init(domain_id, iface) first.")


def _get_lowstate_sub():
    _require_init()
    return _state["lowstate_sub"]


def _get_odom_sub():
    _require_init()
    return _state["odom_sub"]


def _get_slam_info_subs():
    _require_init()
    if _state["slam_info_subs"] is None:
        _state["slam_info_subs"] = (Latest("rt/slam_info", String_), Latest("rt/slam_key_info", String_))
    return _state["slam_info_subs"]


def _get_bms_subs():
    _require_init()
    if _state["bms_subs"] is None:
        _state["bms_subs"] = [(topic, Latest(topic, msg_type)) for topic in BMS_TOPICS for msg_type in _bms_types()]
    return _state["bms_subs"]


def _get_cloud_subs():
    _require_init()
    if _state["cloud_subs"] is None:
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        _state["cloud_subs"] = [(topic, Latest(topic, PointCloud2_)) for topic in SLAM_POINT_TOPICS]
    return _state["cloud_subs"]


def _hand_pubs():
    _require_init()
    if _state["hand_pubs"] is None:
        _state["hand_pubs"] = {side: make_publisher(topic, HandCmd_) for side, topic in HAND_CMD_TOPICS.items()}
    return _state["hand_pubs"]


def _hand_state_subs():
    _require_init()
    if _state["hand_state_subs"] is None:
        _state["hand_state_subs"] = {side: Latest(topic, HandState_, 20) for side, topic in HAND_STATE_TOPICS.items()}
    return _state["hand_state_subs"]


def _loco_client():
    _require_init()
    if _state["loco"] is None:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        client = LocoClient()
        client.SetTimeout(5.0)
        client.Init()
        _state["loco"] = client
    return _state["loco"]


def _audio_client():
    _require_init()
    if _state["audio"] is None:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
        client = AudioClient()
        client.SetTimeout(5.0)
        client.Init()
        _state["audio"] = client
    return _state["audio"]


def _arm_action_client():
    _require_init()
    if _state["arm_action"] is None:
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        client = G1ArmActionClient()
        client.SetTimeout(10.0)
        client.Init()
        _state["arm_action"] = client
    return _state["arm_action"]


def _robot_state_client():
    _require_init()
    if _state["robot_state"] is None:
        try:
            from unitree_sdk2py.b2.robot_state.robot_state_client import RobotStateClient
        except ImportError:
            from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        client = RobotStateClient()
        client.SetTimeout(5.0)
        client.Init()
        _state["robot_state"] = client
    return _state["robot_state"]


def _arm_sdk_pub():
    _require_init()
    if _state["arm_sdk_pub"] is None:
        _state["arm_sdk_pub"] = make_publisher("rt/arm_sdk", LowCmd_)
        _state["arm_sdk_crc"] = CRC()
    return _state["arm_sdk_pub"]


def _arm_sdk_crc():
    _arm_sdk_pub()
    return _state["arm_sdk_crc"]


def _lowcmd_pub():
    _require_init()
    if _state["lowcmd_pub"] is None:
        _state["lowcmd_pub"] = LowCmdPublisher()
    return _state["lowcmd_pub"]


def _slam_rpc():
    _require_init()
    if _state["slam_rpc"] is None:
        client = SlamRpc()
        client.SetTimeout(10.0)
        _state["slam_rpc"] = client
    return _state["slam_rpc"]
