"""Off-robot simulator for g1_dashboard.py (``--off-robot``).

Pure Python (stdlib only): no unitree_sdk2py, CycloneDDS, OpenAI, numpy or
OpenCV. ``install_stubs()`` registers stand-ins for the robot/hardware
modules g1_cli imports, so g1_cli and the dashboard load unchanged. SimRobot
implements the sdk_wrapper.G1 surface the dashboard uses: a small room with
obstacles, simulated LiDAR clouds, SLAM mapping/relocation/navigation, FSM
modes, services, Dex3 tactile data and teleop kinematics. SimCamera renders
RGB and depth by ray casting the same room and outlines the obstacles with
convex hulls, as PNG frames.
"""
from __future__ import annotations

import json
import math
import random
import struct
import sys
import threading
import time
import types
import zlib
from pathlib import Path
from types import SimpleNamespace
from typing import Any

# Values copied from sdk_wrapper.py so g1_cli's topic list matches the robot.
BMS_TOPICS = ["rt/lf/bmsstate", "rt/lf/agvbmsstate", "rt/bmsstate", "rt/agvbmsstate"]
SLAM_POINT_TOPICS = [
    "rt/unitree/slam_mapping/points",
    "rt/unitree/slam_relocation/points",
    "rt/unitree/slam_relocation/global_map",
    "rt/unitree/slam_relocation/web_points",
]
HAND_CMD_TOPICS = {"left": "rt/dex3/right/cmd", "right": "rt/dex3/left/cmd"}
HAND_STATE_TOPICS = {"left": "rt/dex3/right/state", "right": "rt/dex3/left/state"}
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
WAIST_JOINTS = [12, 13, 14]
API_FSM_ID, API_FSM_MODE = 7001, 7002
WALKING_FSMS = {500, 501, 801, 802}
SERVICES = [("ai_sport", "Main Motion Control Service", True), ("basic_service", "Basic Service", True),
            ("g1_arm_example", "Upper Limb Motion Service", False), ("vui_service", "Audio and Lighting Control Service", False),
            ("unitree_slam", "Navigation Service", False)]

ROOM = (-1.0, 9.0, -3.0, 4.0, 2.6)  # x0, x1, y0, y1, ceiling
# (cx, cy, width_x, depth_y, height, rgb)
BOXES = [
    (3.0, 1.0, 1.2, 0.8, 0.75, (176, 122, 72)),   # table
    (5.5, -1.5, 0.6, 0.6, 0.5, (70, 140, 220)),   # crate
    (6.5, 2.2, 0.4, 0.4, 2.4, (200, 200, 210)),   # pillar
    (1.2, 3.55, 2.0, 0.4, 1.8, (120, 90, 160)),   # shelf
    (2.0, -2.0, 0.5, 0.5, 0.9, (220, 80, 80)),    # chair
]


def install_stubs() -> None:
    """Register stand-ins for dds_env, sdk_wrapper and the agent package."""
    def module(name: str, **attrs: Any) -> types.ModuleType:
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
        return mod

    class _Msg:  # placeholder IDL types
        pass

    def rpc_get_int(client: Any, api_id: int) -> Any:
        return client.rpc_get_int(api_id)

    def mode_name(data: Any) -> str:
        if isinstance(data, dict):
            return str(data.get("name", ""))
        return str(data or "")

    def result_code(result: Any) -> int:
        if isinstance(result, tuple):
            return int(result[0])
        if isinstance(result, dict):
            return int(result.get("code", 0))
        return int(result)

    module("dds_env", ensure_cyclonedds_environment=lambda: None)
    module("sdk_wrapper", BMS_TOPICS=BMS_TOPICS, SLAM_POINT_TOPICS=SLAM_POINT_TOPICS, HAND_CMD_TOPICS=HAND_CMD_TOPICS,
           HAND_STATE_TOPICS=HAND_STATE_TOPICS, LEFT_ARM_JOINTS=LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS=RIGHT_ARM_JOINTS,
           WAIST_JOINTS=WAIST_JOINTS, ROBOT_API_ID_LOCO_GET_FSM_ID=API_FSM_ID, ROBOT_API_ID_LOCO_GET_FSM_MODE=API_FSM_MODE,
           PointCloud2_=_Msg, LowCmd_=_Msg, HandCmd_=_Msg, _rpc_get_int=rpc_get_int, _mode_name=mode_name,
           _result_code=result_code, _factory=None, G1=None)

    class OpenAIPlanner:
        DEFAULT_MODEL = "off-robot"

    class LifecycleState:
        AWAKE = "awake"

    def build_agent(_args: Any) -> Any:
        raise RuntimeError("build_agent is not available in --off-robot mode")

    module("agent")
    module("agent.cli")
    module("agent.cli.__main__", build_agent=build_agent)
    module("agent.planner", OpenAIPlanner=OpenAIPlanner)
    module("agent.models", LifecycleState=LifecycleState)


# -- agent ----------------------------------------------------------------------

class _Settings:
    def __init__(self) -> None:
        self.values: dict[str, Any] = {}

    def set(self, key: str, value: Any, persist: bool = False) -> None:
        self.values[key] = value


class SimAgent:
    """Stands in for the OpenAI intent agent; exercises reply and confirm flows."""

    def __init__(self, robot: "SimRobot") -> None:
        self.robot = robot
        self.settings = _Settings()
        self._confirm_prompt = lambda skill, policy: True

    def handle_cli_text(self, text: str) -> Any:
        time.sleep(0.6)  # visible "thinking" state
        lowered = text.lower()
        skills: list[tuple[str, Any]] = []
        if any(word in lowered for word in ("wave", "clap", "hug")):
            skill = next(word for word in ("wave", "clap", "hug") if word in lowered)
            policy = SimpleNamespace(reason=f"{skill} moves both arms (simulated).")
            approved = self._confirm_prompt(skill, policy)
            skills.append((skill, SimpleNamespace(message="done (simulated)" if approved else "declined by user")))
            reply = f"Okay, {skill} {'finished' if approved else 'cancelled'}."
        elif "battery" in lowered:
            reply = f"Battery is at {self.robot.soc:.0f}% (simulated)."
        elif "where" in lowered or "pose" in lowered:
            reply = f"I am at x {self.robot.x:.2f} m, y {self.robot.y:.2f} m, heading {math.degrees(self.robot.yaw):.0f}°."
        else:
            reply = (f"[off-robot simulator] You said: “{text}”. Try asking about the battery, my pose, "
                     "or ask me to wave to test the confirmation dialog.")
        decision = SimpleNamespace(response_text=reply)
        return SimpleNamespace(decision=decision, skill_outcomes=skills)


# -- robot ----------------------------------------------------------------------

class SimReader:
    """Mimics sdk_wrapper._Latest: get() -> (message, timestamp)."""

    def __init__(self, producer: Any = None, period: float = 0.2) -> None:
        self.producer = producer
        self.period = period
        self._cache: tuple[Any, float] = (None, 0.0)

    def get(self) -> tuple[Any, float]:
        if self.producer is None:
            return None, 0.0
        now = time.time()
        if now - self._cache[1] >= self.period:
            msg = self.producer()
            self._cache = (msg, now if msg is not None else 0.0)
        return self._cache


def _cloud_msg(points: list[tuple[float, float, float]]) -> Any:
    flat = [v for p in points for v in p]
    fields = [SimpleNamespace(name=n, offset=o) for n, o in (("x", 0), ("y", 4), ("z", 8))]
    return SimpleNamespace(fields=fields, point_step=12, width=len(points), height=1,
                           data=struct.pack(f"<{len(flat)}f", *flat))


def _room_points() -> list[tuple[float, float, float]]:
    x0, x1, y0, y1, top = ROOM
    pts: list[tuple[float, float, float]] = []
    def frange(a: float, b: float, step: float) -> list[float]:
        n = max(1, int((b - a) / step))
        return [a + (b - a) * i / n for i in range(n + 1)]
    for z in frange(0.05, top, 0.18):
        for x in frange(x0, x1, 0.12):
            pts += [(x, y0, z), (x, y1, z)]
        for y in frange(y0, y1, 0.12):
            pts += [(x0, y, z), (x1, y, z)]
    for x in frange(x0, x1, 0.35):
        for y in frange(y0, y1, 0.35):
            pts.append((x, y, random.uniform(-0.01, 0.01)))  # floor (removed by the height filter)
    for cx, cy, w, d, hgt, _rgb in BOXES:
        bx0, bx1, by0, by1 = cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2
        for z in frange(0.04, hgt, 0.08):
            for x in frange(bx0, bx1, 0.08):
                pts += [(x, by0, z), (x, by1, z)]
            for y in frange(by0, by1, 0.08):
                pts += [(bx0, y, z), (bx1, y, z)]
        for x in frange(bx0, bx1, 0.1):
            for y in frange(by0, by1, 0.1):
                pts.append((x, y, hgt))
    return pts


def _blocked(x: float, y: float, margin: float = 0.28) -> bool:
    x0, x1, y0, y1, _top = ROOM
    if not (x0 + margin < x < x1 - margin and y0 + margin < y < y1 - margin):
        return True
    return any(abs(x - cx) < w / 2 + margin and abs(y - cy) < d / 2 + margin for cx, cy, w, d, _h, _c in BOXES)


class _Client:
    def __init__(self, robot: "SimRobot") -> None:
        self.robot = robot

    def SetTimeout(self, _seconds: float) -> None:
        pass

    def SetFsmId(self, fsm_id: int) -> int:
        return self.robot._set_fsm(int(fsm_id))

    def Sit(self) -> int:
        return self.robot._set_fsm(3)

    def rpc_get_int(self, api_id: int) -> int:
        return self.robot.fsm_id if api_id == API_FSM_ID else int(self.robot.moving)


class _Motion:
    def __init__(self, robot: "SimRobot") -> None:
        self.robot = robot

    def CheckMode(self) -> tuple[int, dict[str, str]]:
        return 0, {"name": "" if self.robot.dev_mode else "ai_sport"}

    def SelectMode(self, name: str) -> int:
        if name == "ai_sport":
            self.robot.dev_mode = False
        return 0


class _StateClient:
    def __init__(self, robot: "SimRobot") -> None:
        self.robot = robot

    def SetTimeout(self, _seconds: float) -> None:
        pass

    def ServiceSwitch(self, name: str, enabled: bool) -> int:
        return int(self.robot.set_service(name, enabled)["code"])


class _ArmSdk:
    def __init__(self, robot: "SimRobot") -> None:
        self.robot = robot

    def write(self, targets: dict[int, float], **_kwargs: Any) -> None:
        with self.robot.lock:
            self.robot.arm_q.update({int(k): float(v) for k, v in targets.items()})


class _Hand:
    def __init__(self, robot: "SimRobot", side: str) -> None:
        self.state = SimReader(lambda: SimpleNamespace(
            motor_state=[SimpleNamespace(q=round(robot.grip[side] * 1.2, 3)) for _ in range(7)]), 0.1)


class SimRobot:
    """Implements the sdk_wrapper.G1 methods g1_cli and g1_dashboard call."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.x, self.y, self.yaw = 0.5, 0.0, 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.fsm_id = 500
        self.dev_mode = False
        self.services = {name: {"name": name, "status": 0 if on else 1, "protected": name == "ai_sport", "description": desc}
                         for name, desc, on in SERVICES}
        self.soc = 86.0
        self.phase = 0.0
        self.arm_q = {j: 0.0 for j in WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS}
        self.arm_q.update({16: 0.25, 23: -0.25, 18: 0.9, 25: 0.9})
        self.grip = {"left": 0.0, "right": 0.0}
        self.mapping = False
        self.relocated = False
        self.saved_map: set[int] | None = None
        self.seen: set[int] = set()
        self.nav_target: tuple[float, float, float] | None = None
        self.nav_paused = False
        self.notice: dict[str, Any] | None = None
        self._slam_map_path = "/home/unitree/test.pcd"
        self.room = _room_points()
        self._client = _Client(self)
        self._lowstate = SimReader(lambda: SimpleNamespace(kind="lowstate"), 0.02)
        self._odom = SimReader(lambda: SimpleNamespace(kind="odom"), 0.02)
        self._sport = SimReader(lambda: SimpleNamespace(kind="sport"), 0.02)
        self._sport_alt = SimReader(None)
        self._slam_info = SimReader(self._slam_info_msg, 0.5)
        self._slam_key = SimReader(self._slam_key_msg, 0.5)
        self._audio_msg = SimReader(lambda: SimpleNamespace(data=json.dumps({"asr": "", "sim": True})), 2.0)
        self._bms = [(BMS_TOPICS[2], SimReader(lambda: SimpleNamespace(status=1, soc=int(self.soc), soh=98,
                                                                       current=self._current(), cycle=37), 1.0))]
        self._clouds = [
            (SLAM_POINT_TOPICS[0], SimReader(self._mapping_cloud, 0.5)),
            (SLAM_POINT_TOPICS[1], SimReader(None)),
            (SLAM_POINT_TOPICS[2], SimReader(self._global_map, 2.0)),
            (SLAM_POINT_TOPICS[3], SimReader(None)),
        ]
        self._live = SimReader(self._live_cloud, 0.2)
        self._hands = {side: _Hand(self, side) for side in ("left", "right")}
        self._stop = threading.Event()
        threading.Thread(target=self._physics, name="sim-physics", daemon=True).start()

    # -- simulation loop ---------------------------------------------------------
    @property
    def moving(self) -> bool:
        return any(abs(v) > 1e-3 for v in self.velocity)

    def _can_walk(self) -> bool:
        return self.fsm_id in WALKING_FSMS and not self.dev_mode

    def _physics(self) -> None:
        dt = 0.05
        while not self._stop.wait(dt):
            with self.lock:
                vx = vyaw = 0.0
                if self.nav_target is not None and not self.nav_paused and self._can_walk():
                    tx, ty, tyaw = self.nav_target
                    dist = math.hypot(tx - self.x, ty - self.y)
                    if dist < 0.12:
                        err = math.atan2(math.sin(tyaw - self.yaw), math.cos(tyaw - self.yaw))
                        vyaw = max(-0.8, min(0.8, 2.0 * err))
                        if abs(err) < 0.05:
                            self.nav_target = None
                            self._set_notice("arrived", is_arrived=True)
                    else:
                        heading = math.atan2(ty - self.y, tx - self.x)
                        err = math.atan2(math.sin(heading - self.yaw), math.cos(heading - self.yaw))
                        vyaw = max(-0.8, min(0.8, 2.0 * err))
                        vx = 0.35 if abs(err) < 0.5 else 0.0
                elif self._can_walk():
                    vx, _vy, vyaw = self.cmd
                nx = self.x + vx * math.cos(self.yaw) * dt
                ny = self.y + vx * math.sin(self.yaw) * dt
                if vx and _blocked(nx, ny):
                    vx = 0.0
                    if self.nav_target is not None:
                        self._set_notice("path blocked by obstacle", obstacle_blocked=True)
                else:
                    self.x, self.y = nx, ny
                self.yaw = math.atan2(math.sin(self.yaw + vyaw * dt), math.cos(self.yaw + vyaw * dt))
                self.velocity = (vx, 0.0, vyaw)
                if vx or vyaw:
                    self.phase += dt * 6.0
                self.soc = max(5.0, self.soc - dt * (0.004 if self.moving else 0.0015))
                if self.mapping:
                    rx, ry = self.x, self.y
                    self.seen.update(i for i, p in enumerate(self.room) if (p[0] - rx) ** 2 + (p[1] - ry) ** 2 < 25.0)

    def _set_notice(self, info: str, is_arrived: bool = False, obstacle_blocked: bool = False) -> None:
        self.notice = {"type": "nav", "error_code": 0, "info": info, "is_arrived": is_arrived,
                       "target_node_name": None, "obstacle_blocked": obstacle_blocked, "stamp": time.time()}

    def _current(self) -> int:
        return -2600 if self.moving else -950

    # -- clouds ------------------------------------------------------------------
    def _near(self, radius: float) -> list[int]:
        r2 = radius * radius
        return [i for i, p in enumerate(self.room) if (p[0] - self.x) ** 2 + (p[1] - self.y) ** 2 < r2]

    def _mapping_cloud(self) -> Any:
        if not self.mapping:
            return None
        return _cloud_msg([self.room[i] for i in self._near(6.0)])

    def _global_map(self) -> Any:
        if not self.relocated or not self.saved_map:
            return None
        return _cloud_msg([self.room[i] for i in sorted(self.saved_map)])

    def _live_cloud(self) -> Any:
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        out = []
        for i in self._near(5.5):
            px, py, pz = self.room[i]
            dx, dy = px - self.x, py - self.y
            out.append((c * dx - s * dy + random.gauss(0, 0.01), s * dx + c * dy + random.gauss(0, 0.01), pz))
        return _cloud_msg(out)

    def _latest(self, topic: str, _msg_type: Any = None, _queue_len: int = 10) -> SimReader:
        if topic in ("rt/utlidar/cloud_deskewed", "rt/utlidar/cloud_livox_mid360"):
            return self._live
        return SimReader(None)

    def _slam_info_msg(self) -> Any:
        q = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        return SimpleNamespace(data=json.dumps({"type": "pos_info", "errorCode": 0, "data": {"currentPose": {
            "x": round(self.x, 3), "y": round(self.y, 3), "z": 0.0, "q_x": q[0], "q_y": q[1], "q_z": q[2], "q_w": q[3]}}}))

    def _slam_key_msg(self) -> Any:
        return None if self.notice is None else SimpleNamespace(data=json.dumps({"type": "nav", "errorCode": 0, "data": {
            "is_arrived": self.notice["is_arrived"], "obsInfo": {"state": self.notice["obstacle_blocked"]}}}))

    # -- telemetry ---------------------------------------------------------------
    def get_battery(self) -> dict[str, Any]:
        return {"timestamp": time.time(), "source": BMS_TOPICS[2], "power_v": 50.4 - (86 - self.soc) * 0.05, "power_a": None,
                "bms": {"status": 1, "soc": int(self.soc), "soh": 98, "current": self._current(), "cycle": 37}}

    def get_imus(self) -> dict[str, Any]:
        t = time.time()
        sway = 0.03 if self.moving else 0.005
        return {"rpy": (sway * math.sin(self.phase), 0.02 + sway * math.cos(self.phase * 2), self.yaw),
                "gyro": (0.01 * math.sin(t), 0.01 * math.cos(t), self.velocity[2]), "acc": (0.0, 0.0, 9.81),
                "quat": (math.cos(self.yaw / 2), 0.0, 0.0, math.sin(self.yaw / 2)), "temp": 41.0}

    def get_odom(self) -> dict[str, Any]:
        vx, _vy, vyaw = self.velocity
        return {"timestamp": time.time(), "topic": "rt/odommodestate", "position": (round(self.x, 3), round(self.y, 3), 0.78),
                "velocity": (round(vx * math.cos(self.yaw), 3), round(vx * math.sin(self.yaw), 3), 0.0),
                "mode": int(self.moving), "gait": 1 if self.moving else 0}

    def get_slam_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def get_slam_odom_pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def get_slam_notice(self) -> dict[str, Any] | None:
        return None if self.notice is None else dict(self.notice)

    def get_lowstate(self) -> dict[str, Any]:
        swing = 0.35 if self.moving else 0.0
        p = self.phase
        legs = []
        for side in (0, math.pi):
            legs += [-0.3 + swing * math.sin(p + side), 0.02, 0.0, 0.65 + swing * max(0.0, math.sin(p + side + 0.5)),
                     -0.35 + 0.5 * swing * math.sin(p + side), 0.0]
        with self.lock:
            upper = [self.arm_q[j] for j in WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS]
        q = legs + upper
        return {"timestamp": time.time(), "joint_positions": q,
                "joint_velocities": [swing * 3.0 * math.cos(p + i) if i < 12 else 0.0 for i in range(29)],
                "joint_torques": [round(random.uniform(-2, 2) + (12 * math.sin(p + i) if i < 12 and swing else 0.0), 2) for i in range(29)],
                "imu": self.get_imus()}

    # -- services and modes ------------------------------------------------------
    def get_service(self, service: str | None = None) -> Any:
        rows = [dict(row) for row in self.services.values()]
        return rows if service is None else next((r for r in rows if r["name"] == service), None)

    def set_service(self, service: str, enabled: bool) -> dict[str, Any]:
        row = self.services.get(service)
        if row is None:
            return {"code": 5201}
        row["status"] = 0 if enabled else 1
        if service == "ai_sport":
            self.dev_mode = not enabled
            self.cmd = (0.0, 0.0, 0.0)
        return {"code": 0}

    def _robot_state_client(self) -> _StateClient:
        return _StateClient(self)

    def _motion_client(self) -> _Motion:
        return _Motion(self)

    def _set_fsm(self, fsm_id: int) -> int:
        if self.dev_mode and fsm_id != 0:
            return 3104
        self.fsm_id = fsm_id
        if fsm_id not in WALKING_FSMS:
            self.cmd = (0.0, 0.0, 0.0)
        return 0

    def zero_torque_mode(self) -> int:
        return self._set_fsm(0)

    def damp_mode(self) -> int:
        return self._set_fsm(1)

    def prepare_mode(self) -> int:
        return self._set_fsm(4)

    def walk_mode(self, waist_locked: bool = True) -> int:
        return self._set_fsm(500 if waist_locked else 501)

    def run_mode(self) -> int:
        return self._set_fsm(802)

    # -- locomotion, arms and hands ----------------------------------------------
    def loco_move(self, vx: float, vy: float, vyaw: float, duration_s: Any = None) -> int:
        with self.lock:
            self.nav_target = None
            self.cmd = (float(vx), float(vy), float(vyaw))
        return 0

    def loco_stop(self) -> int:
        with self.lock:
            self.cmd = (0.0, 0.0, 0.0)
        return 0

    def _arm_sdk_client(self) -> _ArmSdk:
        return _ArmSdk(self)

    def _upper_body_pose(self, timeout: float = 3.0) -> dict[int, float]:
        with self.lock:
            return dict(self.arm_q)

    def release_arms(self, duration_s: float = 3.0, rate_hz: float = 50.0) -> int:
        time.sleep(0.6)
        return 0

    def engage_arms(self, duration_s: float = 1.0, rate_hz: float = 50.0) -> int:
        time.sleep(0.4)
        return 0

    def ik_move_ee(self, hand: str = "right", pose_increment: Any = None, stop_event: Any = None, **_kwargs: Any) -> dict[str, Any]:
        joints = LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS
        dx, dy, dz = (list(pose_increment or [0, 0, 0]) + [0, 0, 0])[:3]
        if stop_event is not None and stop_event.wait(0.35):
            raise RuntimeError("IK step cancelled")
        with self.lock:
            sign = 1.0 if hand == "left" else -1.0
            self.arm_q[joints[0]] -= 6.0 * dx + 4.0 * dz   # shoulder pitch
            self.arm_q[joints[1]] += sign * 6.0 * dy        # shoulder roll
            self.arm_q[joints[3]] -= 5.0 * dz               # elbow
            targets = {j: self.arm_q[j] for j in joints}
        return {"hand": hand, "ok": True, "command_targets": targets}

    def _dex3_hand(self, side: str) -> _Hand:
        return self._hands["right" if side == "right" else "left"]

    def get_dex3_hand_sensors(self, hand: str = "both") -> Any:
        def snap(side: str) -> dict[str, Any]:
            grip = self.grip[side]
            pads = []
            for pad in range(9):
                level = grip * (120000 + 40000 * math.sin(pad + time.time())) + random.uniform(0, 3000)
                pads.append([30000.0] * 2 + [round(level + random.uniform(-500, 500), 1) for _ in range(10)])
            return {"hand": side, "ok": True, "timestamp": time.time(), "positions": [grip * 1.2] * 7, "tactile_pressures": pads}
        if hand == "both":
            return {"left": snap("left"), "right": snap("right")}
        return snap(hand)

    def _grip(self, hand: str, value: float) -> dict[str, Any]:
        time.sleep(0.4)
        for side in (("left", "right") if hand == "both" else (hand,)):
            self.grip[side] = value
        return {"hand": hand, "ok": True}

    def open_dex3_hand(self, hand: str = "both", **_kwargs: Any) -> dict[str, Any]:
        return self._grip(hand, 0.0)

    def close_dex3_hand(self, hand: str = "both", **_kwargs: Any) -> dict[str, Any]:
        return self._grip(hand, 1.0)

    # -- SLAM ------------------------------------------------------------------
    @staticmethod
    def _rpc(error_code: int = 0, info: str = "success") -> dict[str, Any]:
        return {"code": 0, "raw": json.dumps({"errorCode": error_code, "info": info})}

    def start_mapping(self, slam_type: str = "indoor") -> dict[str, Any]:
        with self.lock:
            self.mapping, self.relocated = True, False
            self.seen = set()
        return self._rpc(info=f"{slam_type} mapping started")

    def stop_mapping(self, save_path: str | None = None, save: bool = True) -> dict[str, Any]:
        with self.lock:
            if not save:
                self.mapping = self.relocated = False
                self.nav_target = None
                return self._rpc(info="slam closed")
            if not self.mapping:
                return self._rpc(7, "mapping is not running")
            self.mapping = False
            self.saved_map = set(self.seen)
            if save_path:
                self._slam_map_path = str(save_path)
        return self._rpc(info=f"map saved to {self._slam_map_path} ({len(self.saved_map)} points)")

    def relocate(self, map_path: str | None = None, pose: Any = None) -> dict[str, Any]:
        if not self.saved_map:
            return self._rpc(4, "no saved map; map and save first")
        self.relocated = True
        return self._rpc(info="relocation succeeded")

    def pose_nav(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        if not (self.relocated or self.mapping):
            return self._rpc(5, "relocate before navigating")
        if _blocked(float(x), float(y), margin=0.2):
            return self._rpc(6, "target is inside an obstacle")
        with self.lock:
            self.nav_target = (float(x), float(y), float(yaw))
            self.nav_paused = False
            self.cmd = (0.0, 0.0, 0.0)
            self._set_notice("navigating")
        return self._rpc(info="navigation started")

    def pause_nav(self) -> dict[str, Any]:
        self.nav_paused = True
        return self._rpc(info="paused")

    def resume_nav(self) -> dict[str, Any]:
        self.nav_paused = False
        return self._rpc(info="resumed")

    def add_point(self, name: str, points_path: str = "slam_points.json") -> list[float]:
        path = Path(points_path)
        points = json.loads(path.read_text()) if path.exists() else {}
        points[str(name)] = [self.x, self.y, self.yaw]
        path.write_text(json.dumps(points, indent=2))
        return points[str(name)]

    def remove_point(self, name: str, points_path: str = "slam_points.json") -> Any:
        path = Path(points_path)
        points = json.loads(path.read_text()) if path.exists() else {}
        removed = points.pop(str(name), None)
        path.write_text(json.dumps(points, indent=2))
        return removed

    def close(self) -> None:
        self._stop.set()


# -- camera -----------------------------------------------------------------------

def _png(width: int, height: int, rgb: bytearray) -> bytes:
    row = width * 3
    raw = b"".join(b"\x00" + bytes(rgb[y * row:(y + 1) * row]) for y in range(height))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


def _turbo(t: float) -> tuple[int, int, int]:
    t = max(0.0, min(1.0, t))
    r = 34.61 + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05))))
    g = 23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56))))
    b = 27.2 + t * (3211.1 - t * (15327.97 - t * (27814 - t * (22569.18 - t * 6838.66))))
    return tuple(max(0, min(255, int(v))) for v in (r, g, b))  # type: ignore[return-value]


TURBO = [_turbo(i / 255) for i in range(256)]


def _hull(points: list[tuple[int, int]]) -> list[tuple[int, int]]:
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    def cross(o: Any, a: Any, b: Any) -> int:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower: list[tuple[int, int]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[int, int]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


class SimCamera:
    """Ray-cast RGB-D view of the simulated room from the robot's head camera."""

    W, H = 192, 144
    FX = FY = 604.2 * 192 / 640
    CX, CY = 96.0, 72.0
    PITCH = 0.42  # camera tilted down

    def __init__(self, robot: SimRobot) -> None:
        self.robot = robot
        self._key: Any = None
        self._frame: dict[str, Any] | None = None
        self.depth: list[float] = []
        self.rays = [((u + 0.5 - self.CX) / self.FX, (v + 0.5 - self.CY) / self.FY) for v in range(self.H) for u in range(self.W)]

    def depth_at(self, u: float, v: float) -> dict[str, Any]:
        if not self.depth:
            return {"m": None}
        x, y = int(min(self.W - 1, max(0, u * self.W))), int(min(self.H - 1, max(0, v * self.H)))
        d = self.depth[y * self.W + x]
        return {"m": d if d > 0 else None, "x": x, "y": y}

    def render(self, params: dict[str, Any]) -> dict[str, Any]:
        r = self.robot
        key = (round(r.x, 3), round(r.y, 3), round(r.yaw, 3), tuple(sorted(params.items())))
        if key == self._key and self._frame is not None:
            return self._frame
        self._key = key
        self._frame = self._render(r.x, r.y, r.yaw, params)
        return self._frame

    def _render(self, rx: float, ry: float, ryaw: float, params: dict[str, Any]) -> dict[str, Any]:
        W, H = self.W, self.H
        cy, sy, cp, sp = math.cos(ryaw), math.sin(ryaw), math.cos(self.PITCH), math.sin(self.PITCH)
        ox, oy, oz = rx + 0.08 * cy, ry + 0.08 * sy, 1.25
        F = (cp * cy, cp * sy, -sp)
        R = (sy, -cy, 0.0)
        D = (-sp * cy, -sp * sy, -cp)
        x0, x1, y0, y1, top = ROOM
        boxes = [(cx - w / 2, cx + w / 2, cy_ - d / 2, cy_ + d / 2, h, rgb) for cx, cy_, w, d, h, rgb in BOXES]
        rgb = bytearray(W * H * 3)
        depth = [0.0] * (W * H)
        hit_id = [-1] * (W * H)
        floor_mask = [False] * (W * H)
        light = (0.45, 0.3, 0.84)
        for i, (a, b) in enumerate(self.rays):
            dx = R[0] * a + D[0] * b + F[0]
            dy = R[1] * a + D[1] * b + F[1]
            dz = R[2] * a + D[2] * b + F[2]
            # exit point of the room box (camera is inside)
            best, axis = 1e9, 0
            for t, ax in (((x1 - ox) / dx if dx > 0 else (x0 - ox) / dx if dx < 0 else 1e9, 0),
                          ((y1 - oy) / dy if dy > 0 else (y0 - oy) / dy if dy < 0 else 1e9, 1),
                          ((top - oz) / dz if dz > 0 else -oz / dz if dz < 0 else 1e9, 2)):
                if 0 < t < best:
                    best, axis = t, ax
            obj = -1
            normal_axis = axis
            for k, (bx0, bx1, by0, by1, bh, _c) in enumerate(boxes):
                tmin, tmax, nax = 0.0, best, -1
                ok = True
                for o, d, lo, hi, ax in ((ox, dx, bx0, bx1, 0), (oy, dy, by0, by1, 1), (oz, dz, 0.0, bh, 2)):
                    if abs(d) < 1e-9:
                        if not lo <= o <= hi:
                            ok = False
                            break
                        continue
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    if t1 > t2:
                        t1, t2 = t2, t1
                    if t1 > tmin:
                        tmin, nax = t1, ax
                    tmax = min(tmax, t2)
                    if tmin > tmax:
                        ok = False
                        break
                if ok and nax >= 0 and tmin < best:
                    best, obj, normal_axis = tmin, k, nax
            hx, hy, hz = ox + dx * best, oy + dy * best, oz + dz * best
            normal = [0.0, 0.0, 0.0]
            normal[normal_axis] = -1.0 if (dx, dy, dz)[normal_axis] > 0 else 1.0
            shade = 0.45 + 0.55 * max(0.0, normal[0] * light[0] + normal[1] * light[1] + normal[2] * light[2])
            if obj >= 0:
                base = boxes[obj][5]
            elif axis == 2 and dz < 0:
                base = (150, 150, 155) if (int(math.floor(hx * 2)) + int(math.floor(hy * 2))) % 2 else (125, 125, 132)
                floor_mask[i] = True
            elif axis == 2:
                base = (210, 210, 215)
            else:
                base = (226, 214, 190) if int(hz * 4) % 2 else (216, 204, 180)
            fog = max(0.55, 1.0 - best * 0.05)
            p = i * 3
            rgb[p], rgb[p + 1], rgb[p + 2] = (int(min(255, c * shade * fog)) for c in base)
            depth[i] = best if best < 6.0 else 0.0  # beyond range reads invalid, like the RealSense
            hit_id[i] = obj
        self.depth = depth
        floor_ok = bool(params.get("floor", True))
        min_m, max_m = float(params.get("min_m", 0.25)), float(params.get("max_m", 3.0))
        min_px = float(params.get("min_area", 0.003)) * W * H
        obstacles = []
        pixels: dict[int, list[int]] = {}
        for i, k in enumerate(hit_id):
            if (k >= 0 or (not floor_ok and floor_mask[i])) and min_m < depth[i] < max_m:
                pixels.setdefault(k, []).append(i)
        for k, idx in pixels.items():
            if len(idx) < min_px:
                continue
            rows: dict[int, list[int]] = {}
            for i in idx:
                rows.setdefault(i // W, []).append(i % W)
            edge = [(min(xs), v) for v, xs in rows.items()] + [(max(xs), v) for v, xs in rows.items()]
            hull = _hull(edge)
            ds = sorted(depth[i] for i in idx)
            dist = ds[len(ds) // 2]
            us = [i % W for i in idx]
            vs = [i // W for i in idx]
            u, v = sum(us) / len(us), sum(vs) / len(vs)
            bw, bh = max(us) - min(us) + 1, max(vs) - min(vs) + 1
            obstacles.append({"distance": round(dist, 3), "area": round(len(idx) / (W * H), 4),
                              "center": [round((u - self.CX) * dist / self.FX, 3), round((v - self.CY) * dist / self.FY, 3), round(dist, 3)],
                              "size": [round(bw * dist / self.FX, 3), round(bh * dist / self.FY, 3)],
                              "bbox": [min(us), min(vs), bw, bh], "hull": hull})
        obstacles.sort(key=lambda o: o["distance"])
        for n, item in enumerate(obstacles, start=1):
            item["id"] = n
        if params.get("floor_tint", True) and floor_ok:
            for i, f in enumerate(floor_mask):
                if f:
                    p = i * 3
                    rgb[p], rgb[p + 1], rgb[p + 2] = (int(0.6 * rgb[p] + 0.4 * 80), int(0.6 * rgb[p + 1] + 0.4 * 200), int(0.6 * rgb[p + 2] + 0.4 * 120))
        depth_rgb = bytearray(W * H * 3)
        dmax = float(params.get("depth_max", 4.0))
        for i, d in enumerate(depth):
            c = TURBO[int(255 * (1.0 - min(1.0, d / dmax)))] if d > 0 else (18, 20, 24)
            p = i * 3
            depth_rgb[p], depth_rgb[p + 1], depth_rgb[p + 2] = c
        for item in obstacles:
            color = (255, 90, 90) if item["distance"] < 0.8 else (255, 190, 60) if item["distance"] < 1.5 else (90, 220, 200)
            self._polyline(rgb, item["hull"], color, thick=True)
            self._polyline(depth_rgb, item["hull"], (255, 255, 255), thick=False)
        return {"rgb": _png(W, H, rgb), "depth": _png(W, H, depth_rgb), "mime": "image/png", "size": [W, H],
                "obstacles": [{k: v for k, v in o.items() if k != "hull"} for o in obstacles], "floor": floor_ok}

    def _polyline(self, buf: bytearray, hull: list[tuple[int, int]], color: tuple[int, int, int], thick: bool) -> None:
        W, H = self.W, self.H
        def plot(x: int, y: int) -> None:
            if 0 <= x < W and 0 <= y < H:
                p = (y * W + x) * 3
                buf[p], buf[p + 1], buf[p + 2] = color
        n = len(hull)
        for j in range(n if n > 2 else n - 1):
            (xa, ya), (xb, yb) = hull[j], hull[(j + 1) % n]
            steps = max(abs(xb - xa), abs(yb - ya), 1)
            for s in range(steps + 1):
                x = round(xa + (xb - xa) * s / steps)
                y = round(ya + (yb - ya) * s / steps)
                plot(x, y)
                if thick:
                    plot(x + 1, y)
                    plot(x, y + 1)
