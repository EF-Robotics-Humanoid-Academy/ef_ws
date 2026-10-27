"""Off-robot simulation for the V2 /sort pipeline (tests and --off-robot UI).

A kinematic stand-in, NOT a physics model: it exercises sequencing, sensing
and recovery logic, never real-robot safety or grasp success.

* SimRobotV2  -- the sdk_wrapper.G1 surface the V2 code uses: rt/arm_sdk
  writes (position loop with gravity sag ~ 1/kp, authority weight), lowstate
  joint positions, engage/release, Dex3 commands and sensor snapshots.
* SimHandV2   -- Dex3 fingers that stop on an object between them, per-element
  tactile pads (offsets, noise, invalid sentinels) and optional jams.
* SimSceneV2  -- table, objects and ArUco spots; publishes detections and
  pattern poses into a GraspPipelineV2/SorterV2 pair at a fixed rate.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

import grasp_pipeline as gp1
from grasp_pipeline import DEFAULT_CAMERA_EXTRINSIC, Detection, TableObject, base_from_camera
from grasp_pipeline_v2 import GraspPipelineV2
from hand_pose_navigation.arm_fk import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, ArmFK
from hand_pose_navigation.table_estimator import TablePlane
from sdk_wrapper import HAND_CLOSED, HAND_JOINT_NAMES, HAND_SAFE_OPEN
from sort_v2_config import SortV2Config
from sorting_v2 import SorterV2

N_JOINTS = 29
WAIST_YAW = 12
# Gravity torque per right-arm joint (Nm) for the sag model: sag = tau / kp.
DEFAULT_GRAVITY_NM = np.array([3.0, 0.4, 0.0, 1.5, 0.0, 0.3, 0.0])
HAND_SPEED_RAD_S = 6.0


class _MotorCmd:
    def __init__(self) -> None:
        self.q = 0.0


class _Msg:
    def __init__(self) -> None:
        self.motor_cmd = [_MotorCmd() for _ in range(35)]


class SimArmSdk:
    def __init__(self, robot: "SimRobotV2") -> None:
        self.robot = robot
        self.msg = _Msg()
        self.msg.motor_cmd[29].q = 0.0
        self.writes = 0
        self.writers: Dict[str, int] = {}

    def write(self, targets: Dict[int, float], weight: Optional[float] = None, kp: float = 30.0, kd: float = 1.5,
              **_: Any) -> None:
        with self.robot.lock:
            for j, q in targets.items():
                self.robot.q_cmd[int(j)] = float(q)
                self.robot.kp[int(j)] = float(kp)
            if weight is not None:
                self.msg.motor_cmd[29].q = max(0.0, min(1.0, float(weight)))
            self.writes += 1
            name = threading.current_thread().name
            self.writers[name] = self.writers.get(name, 0) + 1


class SimHandV2:
    def __init__(self, side: str, seed: int = 0) -> None:
        rng = np.random.default_rng(seed)
        self.side = side
        self.open_t = np.array(HAND_SAFE_OPEN[side])
        self.closed_t = np.array(HAND_CLOSED[side])
        self.q = self.closed_t.copy()
        self.dq = np.zeros(7)
        self.cmd = self.q.copy()
        self.limp = False
        # 9 sensors x 12 elements, 4 valid per sensor, offsets up to ~1.7k raw.
        self.valid = np.zeros((9, 12), dtype=bool)
        self.valid[:, :4] = True
        self.base = 100600.0 + rng.uniform(0.0, 1800.0, size=(9, 12))
        self.rng = rng
        self.tactile_works = True
        self.contact_sensors = (2, 5, 7)
        self.raw_per_rad = 25000.0
        self.jammed: Dict[int, float] = {}          # joint -> stuck position
        # Stale motors: ignore commands until released (limp) and re-engaged,
        # as a Dex3 motor that dropped out of position control.
        self.stale: set = set()
        self._released_since_stale = False
        self.temps = np.full(7, 40.0)
        self.motor_states = np.zeros(7, dtype=int)
        self.contact_closure = {1: 0.45, 2: 0.45, 3: 0.5, 4: 0.5, 5: 0.5, 6: 0.5}
        self.in_contact = False

    def closure(self, j: int, q: float) -> float:
        o, c = self.open_t[j], self.closed_t[j]
        return (q - o) / (c - o) if abs(c - o) > 1e-6 else 0.0

    def step(self, dt: float) -> None:
        prev = self.q.copy()
        if not self.limp:
            for j in range(7):
                if j in self.jammed:
                    self.q[j] = self.jammed[j]
                    continue
                if j in self.stale:
                    continue
                target = self.cmd[j]
                if self.in_contact and j in self.contact_closure and \
                        self.closure(j, target) > self.contact_closure[j]:
                    o, c = self.open_t[j], self.closed_t[j]
                    target = o + self.contact_closure[j] * (c - o)
                delta = target - self.q[j]
                cap = HAND_SPEED_RAD_S * dt
                self.q[j] += max(-cap, min(cap, delta))
        self.dq = (self.q - prev) / dt

    def blocked_both(self) -> bool:
        def at_contact(j: int) -> bool:
            return abs(self.closure(j, self.q[j]) - self.contact_closure[j]) < 0.05 and \
                self.closure(j, self.cmd[j]) > self.contact_closure[j]
        return self.in_contact and any(at_contact(j) for j in (1, 2)) and any(at_contact(j) for j in (3, 4, 5, 6))

    def pressures(self) -> List[List[float]]:
        values = self.base + self.rng.normal(0.0, 5.0, size=self.base.shape)
        if self.tactile_works and self.in_contact:
            squeeze = max(0.0, max(self.closure(j, self.cmd[j]) - self.contact_closure[j]
                                   for j in self.contact_closure) * abs(self.closed_t[4] - self.open_t[4]))
            for s in self.contact_sensors:
                values[s, :4] += min(20000.0, squeeze * self.raw_per_rad)
        values[~self.valid] = 30000.0
        return values.tolist()


@dataclass
class SimObject:
    label: str
    xy: np.ndarray
    height: float
    radius: float
    z_bottom: float = 0.0
    attached: bool = False
    offset_hand: Optional[np.ndarray] = None
    position: Optional[np.ndarray] = None

    def centre(self) -> np.ndarray:
        if self.position is not None:
            return self.position
        return np.array([self.xy[0], self.xy[1], self.z_bottom + self.height / 2.0])


class SimRobotV2:
    def __init__(self, gravity_nm: Optional[np.ndarray] = None, rate_hz: float = 200.0) -> None:
        self.lock = threading.RLock()
        self.q = np.zeros(N_JOINTS)
        # Arms hanging at the sides (as after release).
        self.q[RIGHT_ARM_JOINTS] = [0.25, -0.15, 0.0, 0.9, 0.0, 0.0, 0.0]
        self.q[LEFT_ARM_JOINTS] = [0.25, 0.15, 0.0, 0.9, 0.0, 0.0, 0.0]
        self.q_cmd = self.q.copy()
        self.kp = np.full(N_JOINTS, 30.0)
        self.gravity = np.zeros(N_JOINTS)
        self.gravity[RIGHT_ARM_JOINTS] = DEFAULT_GRAVITY_NM if gravity_nm is None else gravity_nm
        self.sdk = SimArmSdk(self)
        self.hands = {"right": SimHandV2("right", 1), "left": SimHandV2("left", 2)}
        self.released = True
        self.release_calls = 0
        self.scene: Optional["SimSceneV2"] = None
        self._stop = threading.Event()
        self._dt = 1.0 / rate_hz
        self._thread = threading.Thread(target=self._loop, name="sim-robot", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            with self.lock:
                weight = self.sdk.msg.motor_cmd[29].q
                for j in RIGHT_ARM_JOINTS + LEFT_ARM_JOINTS:
                    if weight > 0.5:
                        goal = self.q_cmd[j] - self.gravity[j] / max(5.0, self.kp[j])
                    else:
                        goal = self.q[j] - 0.002 * np.sign(self.q[j])  # limp: drifting, unpowered
                    self.q[j] += (goal - self.q[j]) * min(1.0, self._dt / 0.05)
                # Waist yaw follows its command (arm_sdk engaged), rate-limited.
                if weight > 0.5:
                    dw = self.q_cmd[WAIST_YAW] - self.q[WAIST_YAW]
                    self.q[WAIST_YAW] += max(-0.5 * self._dt, min(0.5 * self._dt, dw))
                for hand in self.hands.values():
                    hand.step(self._dt)
            if self.scene is not None:
                self.scene.physics()
            time.sleep(self._dt)

    # -- sdk_wrapper.G1 surface ------------------------------------------------
    def _arm_sdk_client(self) -> SimArmSdk:
        return self.sdk

    def get_joint_positions(self, joints=None, max_age_s=None):
        with self.lock:
            idx = range(N_JOINTS) if joints is None else [int(j) for j in joints]
            return {j: float(self.q[j]) for j in idx}, time.time()

    def engage_arms(self, duration_s: float = 1.0, rate_hz: float = 50.0) -> Dict[str, Any]:
        with self.lock:
            for j in RIGHT_ARM_JOINTS + LEFT_ARM_JOINTS:
                self.q_cmd[j] = self.q[j]
                self.kp[j] = 30.0
            self.sdk.msg.motor_cmd[29].q = 1.0
            self.released = False
        return {"final_arm_sdk_weight": 1.0, "joint_count": 17}

    def release_arms(self, duration_s: float = 3.0, rate_hz: float = 50.0) -> Dict[str, Any]:
        with self.lock:
            self.sdk.msg.motor_cmd[29].q = 0.0
            self.released = True
            self.release_calls += 1
        return {"final_arm_sdk_weight": 0.0, "joint_count": 17}

    def get_dex3_hand_sensors(self, hand: str = "right") -> Dict[str, Any]:
        h = self.hands[hand]
        with self.lock:
            return {"hand": hand, "timestamp": time.time(),
                    "positions": {n: float(h.q[i]) for i, n in enumerate(HAND_JOINT_NAMES)},
                    "velocities": {n: float(h.dq[i]) for i, n in enumerate(HAND_JOINT_NAMES)},
                    "torques": {n: 0.0 for n in HAND_JOINT_NAMES}, "tactile_pressures": h.pressures(),
                    "tactile_temperatures": [], "tactile_lost": [0] * 9,
                    "motor_temperatures": {n: float(h.temps[i]) for i, n in enumerate(HAND_JOINT_NAMES)},
                    "motor_states": {n: int(h.motor_states[i]) for i, n in enumerate(HAND_JOINT_NAMES)},
                    "motor_modes": {n: 1 for n in HAND_JOINT_NAMES}, "hand_error": [0, 0]}

    def write_dex3_pose(self, hand: str, targets, kp: float = 1.2, kd: float = 0.05, tau: float = 0.05) -> None:
        with self.lock:
            h = self.hands[hand]
            if h.limp and h._released_since_stale:
                h.stale.clear()                      # re-engaged after a release
                h._released_since_stale = False
            h.limp = False
            h.cmd = np.asarray(targets, dtype=np.float64).copy()

    def hand_pose(self, targets, hand: str = "right", hold_s: float = 0.6, rate_hz: float = 50.0, kp: float = 1.2,
                  kd: float = 0.05, tau: float = 0.05, ramp_s=None) -> Dict[str, Any]:
        self.write_dex3_pose(hand, targets)
        time.sleep(min(1.5, max(0.05, float(hold_s))))  # the SDK call blocks for hold_s
        h = self.hands[hand]
        with self.lock:
            err = np.abs(h.cmd - h.q)
            stuck = [j for j in range(7) if err[j] >= 0.16 and abs(h.dq[j]) <= 0.12]
            if stuck:
                h.cmd = h.q.copy()
                h.limp = True
                return {"ok": False, "reason": "collision_or_stall", "joint": HAND_JOINT_NAMES[stuck[0]]}
        return {"ok": True}

    def open_dex3_hand(self, hand: str = "both", hold_s: float = 0.6, rate_hz: float = 50.0, ramp_s=None):
        return self.hand_pose(HAND_SAFE_OPEN[hand], hand=hand, hold_s=hold_s)

    def close_dex3_hand(self, hand: str = "both", hold_s: float = 0.6, rate_hz: float = 50.0, ramp_s=None):
        return self.hand_pose(HAND_CLOSED[hand], hand=hand, hold_s=hold_s)

    def release_dex3_fingers(self, hand: str = "both", hold_s: float = 0.5, rate_hz: float = 50.0,
                             persistent: bool = False) -> None:
        with self.lock:
            h = self.hands[hand]
            h.cmd = h.q.copy()
            h.limp = True
            h._released_since_stale = bool(h.stale)


class SimSceneV2:
    """Objects, table and spots; physics hook runs in the robot thread."""

    def __init__(self, robot: SimRobotV2, table_z: float = 0.02) -> None:
        self.robot = robot
        robot.scene = self
        self.table = TablePlane(z_base_m=table_z, x_range_m=(0.27, 0.90), y_range_m=(-0.45, 0.45),
                                inlier_count=3000, confidence=0.65)
        self.objects: List[SimObject] = []
        self.spots: List[np.ndarray] = []
        self.fk = ArmFK(arm="right", backend="urdf")
        self.pipeline: Optional[GraspPipelineV2] = None
        self.sorter: Optional[SorterV2] = None
        self.hidden_labels: set = set()
        self.slip_requested = False
        self.score = 0.95
        self.lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def add_object(self, label: str, x: float, y: float, height: float, radius: float) -> SimObject:
        obj = SimObject(label, np.array([x, y]), height, radius, z_bottom=self.table.z_base_m)
        self.objects.append(obj)
        return obj

    def torso_T(self) -> np.ndarray:
        """World (= base at zero waist yaw) from the turned torso."""
        with self.robot.lock:
            yaw = float(self.robot.q[WAIST_YAW])
        T = np.eye(4)
        T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        return T

    def hand_T(self) -> np.ndarray:
        """Hand in the world frame (the arm hangs off the turned torso)."""
        with self.robot.lock:
            q = self.robot.q[RIGHT_ARM_JOINTS].copy()
        return self.torso_T() @ self.fk.compute_arm(q)

    # -- physics (robot thread) ------------------------------------------------
    def physics(self) -> None:
        hand = self.robot.hands["right"]
        T = self.hand_T()
        with self.lock:
            contact = False
            for obj in self.objects:
                if obj.attached:
                    if self.slip_requested:
                        self._drop(obj)
                        self.slip_requested = False
                        continue
                    if all(hand.closure(j, hand.cmd[j]) < hand.contact_closure[j] - 0.1 for j in (1, 3, 5)):
                        self._drop(obj)
                        continue
                    obj.position = T[:3, 3] + T[:3, :3] @ obj.offset_hand
                    contact = True
                    continue
                d = T[:3, :3].T @ (obj.centre() - T[:3, 3])
                if 0.03 <= d[0] <= 0.12 + obj.radius and abs(d[1]) <= obj.radius + 0.02 and abs(d[2]) <= 0.06:
                    contact = True
                    with self.robot.lock:
                        hand.in_contact = True
                    if hand.blocked_both():
                        obj.attached = True
                        obj.offset_hand = d.copy()
            with self.robot.lock:
                hand.in_contact = contact

    def _drop(self, obj: SimObject) -> None:
        centre = obj.centre()
        obj.attached = False
        obj.offset_hand = None
        obj.xy = centre[:2].copy()
        obj.position = None
        obj.z_bottom = self.table.z_base_m

    # -- perception (own thread) ----------------------------------------------
    def bind(self, pipeline: GraspPipelineV2, sorter: Optional[SorterV2] = None) -> None:
        self.pipeline = pipeline
        self.sorter = sorter
        if self._thread is None:
            self._thread = threading.Thread(target=self._perception_loop, name="sim-scene", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()

    def table_objects(self) -> List[TableObject]:
        """As the head camera sees them: in the (turned) torso frame."""
        out = []
        T_torso_world = np.linalg.inv(self.torso_T())
        with self.lock:
            for obj in self.objects:
                c = (T_torso_world @ np.append(obj.centre(), 1.0))[:3]
                bottom = c[2] - obj.height / 2.0
                out.append(TableObject(box=(0, 0, 1, 1), center_xy=c[:2].copy(), radius_m=obj.radius,
                                       top_z_m=float(bottom + obj.height), table_z_m=self.table.z_base_m,
                                       visible_centroid=c.copy()))
        return out

    def _perception_loop(self) -> None:
        import cv2
        p = self.pipeline
        T_cam_torso = np.linalg.inv(base_from_camera(DEFAULT_CAMERA_EXTRINSIC))
        while not self._stop.is_set():
            now = time.time()
            # The camera rides on the torso: world points seen through the turn.
            T_cam_base = T_cam_torso @ np.linalg.inv(self.torso_T())
            image = np.full((480, 640, 3), 40, dtype=np.uint8)
            detections: Dict[str, Detection] = {}
            with self.lock:
                for i, obj in enumerate(self.objects):
                    if obj.label in self.hidden_labels:
                        continue
                    T = np.eye(4)
                    T[:3, 3] = obj.centre()
                    T_cam = T_cam_base @ T
                    uv = p.project_camera_point(T_cam[:3, 3])
                    box = None
                    if uv is not None:
                        r = int(max(8, 600 * obj.radius / max(0.2, T_cam[2, 3])))
                        box = (uv[0] - r, uv[1] - 2 * r, uv[0] + r, uv[1] + 2 * r)
                        cv2.rectangle(image, box[:2], box[2:], (200, 200, 200), 1)
                    detections[f"vision:{i}"] = Detection(id=f"vision:{i}", label=obj.label, source="vision",
                                                          score=self.score, T_camera_object=T_cam, box=box,
                                                          top_offset_m=obj.height / 2.0)
            with p.lock:
                p.rgb_bgr = image
                p.depth_m = np.zeros((480, 640), dtype=np.float32)
                p.detection_rgb_bgr = image
                p.detections = detections
                p.last_frame_ts = now
                p.last_detection_ts = now
                p.table_objects = self.table_objects()
                p.table_objects_ts = now
                p.camera_status = "simulated camera"
                p.vision_status = "on (simulated)"
            sorter = self.sorter
            if sorter is not None:
                with sorter.lock:
                    for i, centre in enumerate(self.spots):
                        pattern = sorter.patterns[i]
                        T_pattern = np.eye(4)
                        T_pattern[:3, 3] = centre - np.array([pattern.width_m / 2.0, pattern.height_m / 2.0, 0.0])
                        sorter.pattern_T_camera[i] = T_cam_base @ T_pattern
                        sorter.pattern_ts[i] = now
                        sorter.pattern_epoch[i] = sorter.view_epoch
                        sorter.pattern_status[i] = "visible (simulated)"
            self._stop.wait(0.1)


class SimPipelineV2(GraspPipelineV2):
    def __init__(self, robot: SimRobotV2, scene: SimSceneV2, cfg: Optional[SortV2Config] = None) -> None:
        super().__init__(robot, "sim://", cfg=cfg)
        self.scene = scene

    def touch(self) -> None:
        self.last_viewer = time.monotonic()

    def current_table(self) -> Optional[TablePlane]:
        return self.scene.table

    def estimate_obstacles(self, cam: Dict[str, float]) -> gp1.Obstacles:
        return gp1.Obstacles(table=self.scene.table, table_margin_m=gp1.TABLE_COLLISION_MARGIN_M,
                             table_xy_margin_m=gp1.TABLE_COLLISION_XY_MARGIN_M)

    def obstacle_objects(self, T_base_object: np.ndarray, since_ts: float) -> List[TableObject]:
        target = T_base_object[:2, 3]
        return [o for o in self.scene.table_objects()
                if float(np.linalg.norm(o.center_xy - target)) >= o.radius_m + gp1.OBJECT_OBSTACLE_TARGET_GAP_M]


class SimSorterV2(SorterV2):
    def touch(self) -> None:
        self.p.touch()


def build_sim(cfg: Optional[SortV2Config] = None, gravity_nm: Optional[np.ndarray] = None, default_scene: bool = True):
    """(robot, scene, pipeline, sorter) wired together; the sorter keeps its
    default object/pattern configuration (coffee cup -> A, domol bottle -> B)."""
    robot = SimRobotV2(gravity_nm=gravity_nm)
    scene = SimSceneV2(robot)
    pipeline = SimPipelineV2(robot, scene, cfg)
    sorter = SimSorterV2(pipeline)
    if default_scene:
        # Right-arm layout: both spots on the robot's right (a level-palm right
        # hand cannot carry far to the left at transport height).
        scene.add_object("coffee cup", 0.33, -0.20, 0.11, 0.045)
        scene.add_object("domol bottle", 0.33, -0.02, 0.20, 0.03)
        scene.spots = [np.array([0.27, -0.37, scene.table.z_base_m]), np.array([0.41, -0.10, scene.table.z_base_m])]
    scene.bind(pipeline, sorter)
    return robot, scene, pipeline, sorter
