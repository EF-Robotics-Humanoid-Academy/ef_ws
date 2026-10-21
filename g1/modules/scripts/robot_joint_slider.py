#!/usr/bin/env python3
from __future__ import annotations
from sdk_hand import (
    Dex3HandController,
    HAND_MAX_LIMITS,
    HAND_MIN_LIMITS,
    TOPIC_HAND_BY_SIDE,
    hand_open_targets,
)
from dds_env import ensure_cyclonedds_environment

import argparse
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)


ensure_cyclonedds_environment()

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QScrollArea,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit("PyQt5 not found. Install it with: pip install PyQt5") from exc

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, LowCmd_
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

except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


SLIDER_SCALE = 1000
DEFAULT_SOFT_RANGE_RAD = 1.0
ABS_RANGE_LIMIT_RAD = 3.14
NOT_USED_IDX = 29
BODY_COMMAND_TOPIC = "rt/lowcmd"
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

# Limits follow the published Unitree G1 29-DOF joint table.
JOINT_LAYOUT: list[tuple[str, int, str, float, float]] = [
    ("left_leg", 0, "hip_pitch", -2.5307, 2.8798),
    ("left_leg", 1, "hip_roll", -0.5236, 2.9671),
    ("left_leg", 2, "hip_yaw", -2.7576, 2.7576),
    ("left_leg", 3, "knee", -0.087267, 2.8798),
    ("left_leg", 4, "ankle_pitch", -0.87267, 0.5236),
    ("left_leg", 5, "ankle_roll", -0.2618, 0.2618),
    ("right_leg", 6, "hip_pitch", -2.5307, 2.8798),
    ("right_leg", 7, "hip_roll", -2.9671, 0.5236),
    ("right_leg", 8, "hip_yaw", -2.7576, 2.7576),
    ("right_leg", 9, "knee", -0.087267, 2.8798),
    ("right_leg", 10, "ankle_pitch", -0.87267, 0.5236),
    ("right_leg", 11, "ankle_roll", -0.2618, 0.2618),
    ("waist", 12, "yaw", -2.618, 2.618),
    ("waist", 13, "roll", -0.52, 0.52),
    ("waist", 14, "pitch", -0.52, 0.52),
    ("left_arm", 15, "shoulder_pitch", -3.0892, 2.6704),
    ("left_arm", 16, "shoulder_roll", -1.5882, 2.2515),
    ("left_arm", 17, "shoulder_yaw", -2.618, 2.618),
    ("left_arm", 18, "elbow", -1.0472, 2.0944),
    ("left_arm", 19, "wrist_roll", -1.9722, 1.9722),
    ("left_arm", 20, "wrist_pitch", -1.6144, 1.6144),
    ("left_arm", 21, "wrist_yaw", -1.6144, 1.6144),
    ("right_arm", 22, "shoulder_pitch", -3.0892, 2.6704),
    ("right_arm", 23, "shoulder_roll", -2.2515, 1.5882),
    ("right_arm", 24, "shoulder_yaw", -2.618, 2.618),
    ("right_arm", 25, "elbow", -1.0472, 2.0944),
    ("right_arm", 26, "wrist_roll", -1.9722, 1.9722),
    ("right_arm", 27, "wrist_pitch", -1.6144, 1.6144),
    ("right_arm", 28, "wrist_yaw", -1.6144, 1.6144),
]

BODY_GROUP_ORDER = ["left_leg", "right_leg", "waist", "left_arm", "right_arm"]
BODY_GROUP_TITLES = {
    "left_leg": "Left Leg",
    "right_leg": "Right Leg",
    "waist": "Waist",
    "left_arm": "Left Arm",
    "right_arm": "Right Arm",
}


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="PyQt whole-robot joint and Dex3 finger slider.")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument(
        "--speed-rad-s",
        type=float,
        default=0.5,
        help="Maximum commanded joint transition speed.",
    )
    parser.add_argument("--kp", type=float, default=30.0, help="Body joint proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Body joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Body joint feed-forward torque.")
    parser.add_argument("--hand-kp", type=float, default=0.8, help="Dex3 finger proportional gain.")
    parser.add_argument("--hand-kd", type=float, default=0.05, help="Dex3 finger derivative gain.")
    parser.add_argument("--hand-tau", type=float, default=0.0,
                        help="Dex3 finger feed-forward torque.")
    args, remaining = parser.parse_known_args()
    return args, [sys.argv[0], *remaining]


def resolve_lowstate_type() -> type | None:
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            module = __import__(module_path, fromlist=["LowState_"])
        except Exception:
            continue
        if hasattr(module, "LowState_"):
            return getattr(module, "LowState_")
    return None


@dataclass(frozen=True)
class JointSpec:
    group: str
    motor_index: int
    name: str
    limit_min: float
    limit_max: float


@dataclass
class JointControl:
    spec: JointSpec
    current_label: QLabel
    desired_label: QLabel
    slider: QSlider
    min_box: QDoubleSpinBox
    max_box: QDoubleSpinBox
    reset_button: QPushButton


@dataclass
class HandJointControl:
    hand: str
    joint_index: int
    name: str
    current_label: QLabel
    desired_label: QLabel
    slider: QSlider
    min_box: QDoubleSpinBox
    max_box: QDoubleSpinBox
    reset_button: QPushButton


JOINT_SPECS = [JointSpec(*item) for item in JOINT_LAYOUT]
JOINTS_BY_GROUP: dict[str, list[JointSpec]] = {group: [] for group in BODY_GROUP_ORDER}
for spec in JOINT_SPECS:
    JOINTS_BY_GROUP.setdefault(spec.group, []).append(spec)


class RobotStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: dict[int, float] = {}
        self._timestamp = 0.0
        self._mode_machine = 0

        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        self._sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            positions = {joint: float(msg.motor_state[joint].q) for joint in self.joints}
            mode_machine = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()
            self._mode_machine = mode_machine

    def snapshot(self) -> tuple[dict[int, float], float, int] | None:
        with self._lock:
            if not self._positions:
                return None
            return dict(self._positions), float(self._timestamp), int(self._mode_machine)


class HandStateSubscriber:
    def __init__(self, hand: str) -> None:
        self.hand = str(hand)
        self._lock = threading.Lock()
        self._positions = hand_open_targets(self.hand)
        self._timestamp = 0.0
        self._sub = ChannelSubscriber(HAND_STATE_TOPIC_BY_SIDE[self.hand], HandState_)
        self._sub.Init(self._callback, 50)

    def _callback(self, msg: Any) -> None:
        try:
            positions = [float(msg.motor_state[idx].q) for idx in range(7)]
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[list[float], float] | None:
        with self._lock:
            return list(self._positions), float(self._timestamp)


class RobotPoseController:
    def __init__(self, joints: list[int], *, iface: str, domain_id: int) -> None:
        self.joints = [int(j) for j in joints]
        self._crc = CRC()
        ChannelFactoryInitialize(int(domain_id), str(iface))
        self._pub = ChannelPublisher(BODY_COMMAND_TOPIC, LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        for joint in self.joints:
            self._cmd.motor_cmd[joint].mode = 1

    def write_targets_once(
        self,
        targets_by_joint: dict[int, float],
        *,
        kp: float,
        kd: float,
        tau: float,
        mode_machine: int,
    ) -> None:
        self._cmd.mode_machine = int(mode_machine)
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(targets_by_joint[joint])
            mc.dq = 0.0
            mc.kp = float(kp)
            mc.kd = float(kd)
            mc.tau = float(tau)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_zero_gains_once(self, hold_positions: dict[int, float], *, mode_machine: int) -> None:
        self._cmd.mode_machine = int(mode_machine)
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(hold_positions[joint])
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class RobotJointSliderApp(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.tau = float(args.tau)
        self.hand_kp = float(args.hand_kp)
        self.hand_kd = float(args.hand_kd)
        self.hand_tau = float(args.hand_tau)

        self.joint_specs = list(JOINT_SPECS)
        self.all_joints = [spec.motor_index for spec in self.joint_specs]
        self.current_targets = {joint: 0.0 for joint in self.all_joints}
        self.desired_targets = dict(self.current_targets)
        self.latest_positions = dict(self.current_targets)
        self.latest_state_time = 0.0
        self.mode_machine = 0
        self.controls_by_joint: dict[int, JointControl] = {}

        self.hand_sides = ["left", "right"]
        self.hand_current_targets = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_desired_targets = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_latest_positions = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_latest_state_time = {hand: 0.0 for hand in self.hand_sides}
        self.hand_controls: dict[tuple[str, int], HandJointControl] = {}

        self._updating_controls = False
        self.last_tick_s = time.monotonic()
        self.seeded_from_state = False
        self.status_text = "Initializing..."

        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.state_sub = RobotStateSubscriber(self.all_joints)
        self.controller = RobotPoseController(
            self.all_joints,
            iface=self.iface,
            domain_id=self.domain_id,
        )
        self.hand_subs = {hand: HandStateSubscriber(hand) for hand in self.hand_sides}
        self.hand_controllers: dict[str, Dex3HandController | None] = {}
        for hand in self.hand_sides:
            try:
                self.hand_controllers[hand] = Dex3HandController(
                    hand=hand, iface=self.iface, domain_id=self.domain_id)
            except Exception:
                self.hand_controllers[hand] = None

        self._build_ui()
        self._seed_from_state_if_available()

        self.status_text = f"Publishing body on {BODY_COMMAND_TOPIC}; hands on Dex3 topics via {self.iface}"
        self._refresh_status_label()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(max(10, int(round(1000.0 / self.rate_hz))))

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        gains = QFormLayout()
        self.speed_box = self._make_spinbox(0.01, 5.0, self.speed_rad_s, 0.05)
        self.speed_box.valueChanged.connect(
            lambda value: setattr(self, "speed_rad_s", float(value)))
        gains.addRow("Ramp speed rad/s", self.speed_box)

        self.kp_box = self._make_spinbox(0.0, 100.0, self.kp, 0.5)
        self.kp_box.valueChanged.connect(lambda value: setattr(self, "kp", float(value)))
        gains.addRow("Body kp", self.kp_box)

        self.kd_box = self._make_spinbox(0.0, 20.0, self.kd, 0.1)
        self.kd_box.valueChanged.connect(lambda value: setattr(self, "kd", float(value)))
        gains.addRow("Body kd", self.kd_box)

        self.hand_kp_box = self._make_spinbox(0.0, 5.0, self.hand_kp, 0.05)
        self.hand_kp_box.valueChanged.connect(lambda value: setattr(self, "hand_kp", float(value)))
        gains.addRow("Hand kp", self.hand_kp_box)

        self.hand_kd_box = self._make_spinbox(0.0, 1.0, self.hand_kd, 0.01)
        self.hand_kd_box.valueChanged.connect(lambda value: setattr(self, "hand_kd", float(value)))
        gains.addRow("Hand kd", self.hand_kd_box)
        root.addLayout(gains)

        buttons = QHBoxLayout()
        self.sync_button = QPushButton("Sync All To Current", self)
        self.sync_button.clicked.connect(self._sync_desired_to_current)
        buttons.addWidget(self.sync_button)

        self.zero_button = QPushButton("Zero Body Gains", self)
        self.zero_button.clicked.connect(self._zero_gains_once)
        buttons.addWidget(self.zero_button)

        self.open_hands_button = QPushButton("Open Hands", self)
        self.open_hands_button.clicked.connect(self._open_hands)
        buttons.addWidget(self.open_hands_button)
        root.addLayout(buttons)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll_contents = QWidget()
        scroll_layout = QVBoxLayout()

        for group_name in BODY_GROUP_ORDER:
            specs = JOINTS_BY_GROUP.get(group_name, [])
            if specs:
                scroll_layout.addWidget(self._build_body_group_box(group_name, specs))

        for hand in self.hand_sides:
            scroll_layout.addWidget(self._build_hand_group_box(hand))

        scroll_contents.setLayout(scroll_layout)
        scroll.setWidget(scroll_contents)
        root.addWidget(scroll)

        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        self.setLayout(root)
        self.setWindowTitle("Robot Joint Pose Slider")
        self.resize(1380, 980)

    def _build_body_group_box(self, group_name: str, specs: list[JointSpec]) -> QGroupBox:
        box = QGroupBox(BODY_GROUP_TITLES.get(group_name, group_name))
        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Current"), 0, 1)
        grid.addWidget(QLabel("Desired"), 0, 2)
        grid.addWidget(QLabel("Min"), 0, 3)
        grid.addWidget(QLabel("Slider"), 0, 4)
        grid.addWidget(QLabel("Max"), 0, 5)
        grid.addWidget(QLabel(""), 0, 6)

        for row, spec in enumerate(specs, start=1):
            control = self._make_joint_row(spec)
            grid.addWidget(QLabel(f"{spec.motor_index}: {spec.name}"), row, 0)
            grid.addWidget(control.current_label, row, 1)
            grid.addWidget(control.desired_label, row, 2)
            grid.addWidget(control.min_box, row, 3)
            grid.addWidget(control.slider, row, 4)
            grid.addWidget(control.max_box, row, 5)
            grid.addWidget(control.reset_button, row, 6)

        box.setLayout(grid)
        return box

    def _build_hand_group_box(self, hand: str) -> QGroupBox:
        box = QGroupBox(f"{hand.title()} Dex3 Fingers")
        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Current"), 0, 1)
        grid.addWidget(QLabel("Desired"), 0, 2)
        grid.addWidget(QLabel("Min"), 0, 3)
        grid.addWidget(QLabel("Slider"), 0, 4)
        grid.addWidget(QLabel("Max"), 0, 5)
        grid.addWidget(QLabel(""), 0, 6)

        for row, joint_index in enumerate(range(7), start=1):
            control = self._make_hand_joint_row(hand, joint_index)
            grid.addWidget(QLabel(f"{joint_index}: {control.name}"), row, 0)
            grid.addWidget(control.current_label, row, 1)
            grid.addWidget(control.desired_label, row, 2)
            grid.addWidget(control.min_box, row, 3)
            grid.addWidget(control.slider, row, 4)
            grid.addWidget(control.max_box, row, 5)
            grid.addWidget(control.reset_button, row, 6)

        box.setLayout(grid)
        return box

    def _make_joint_row(self, spec: JointSpec) -> JointControl:
        current_label = QLabel(" -- ", self)
        desired_label = QLabel(" -- ", self)

        min_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD,
                                     ABS_RANGE_LIMIT_RAD, spec.limit_min, 0.05)
        max_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD,
                                     ABS_RANGE_LIMIT_RAD, spec.limit_max, 0.05)
        min_box.valueChanged.connect(
            lambda _value, joint=spec.motor_index: self._update_slider_range(joint))
        max_box.valueChanged.connect(
            lambda _value, joint=spec.motor_index: self._update_slider_range(joint))

        slider = QSlider(Qt.Horizontal, self)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.valueChanged.connect(
            lambda raw, joint=spec.motor_index: self._on_slider_changed(joint, raw))

        reset_button = QPushButton("Current", self)
        reset_button.clicked.connect(
            lambda _checked=False, joint=spec.motor_index: self._set_desired_to_current(joint))

        control = JointControl(
            spec=spec,
            current_label=current_label,
            desired_label=desired_label,
            slider=slider,
            min_box=min_box,
            max_box=max_box,
            reset_button=reset_button,
        )
        self.controls_by_joint[spec.motor_index] = control
        self._update_slider_range(spec.motor_index)
        self._update_joint_labels(spec.motor_index)
        return control

    def _make_hand_joint_row(self, hand: str, joint_index: int) -> HandJointControl:
        current_label = QLabel(" -- ", self)
        desired_label = QLabel(" -- ", self)

        min_default = float(HAND_MIN_LIMITS[hand][joint_index])
        max_default = float(HAND_MAX_LIMITS[hand][joint_index])
        min_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD, ABS_RANGE_LIMIT_RAD, min_default, 0.05)
        max_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD, ABS_RANGE_LIMIT_RAD, max_default, 0.05)
        min_box.valueChanged.connect(lambda _value, side=hand,
                                     idx=joint_index: self._update_hand_slider_range(side, idx))
        max_box.valueChanged.connect(lambda _value, side=hand,
                                     idx=joint_index: self._update_hand_slider_range(side, idx))

        slider = QSlider(Qt.Horizontal, self)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.valueChanged.connect(lambda raw, side=hand,
                                    idx=joint_index: self._on_hand_slider_changed(side, idx, raw))

        reset_button = QPushButton("Current", self)
        reset_button.clicked.connect(lambda _checked=False, side=hand,
                                     idx=joint_index: self._set_hand_desired_to_current(side, idx))

        control = HandJointControl(
            hand=hand,
            joint_index=joint_index,
            name=HAND_JOINT_NAMES[joint_index],
            current_label=current_label,
            desired_label=desired_label,
            slider=slider,
            min_box=min_box,
            max_box=max_box,
            reset_button=reset_button,
        )
        self.hand_controls[(hand, joint_index)] = control
        self._update_hand_slider_range(hand, joint_index)
        self._update_hand_joint_labels(hand, joint_index)
        return control

    @staticmethod
    def _make_spinbox(minimum: float, maximum: float, value: float, step: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setDecimals(3)
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def _seed_from_state_if_available(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, timestamp, mode_machine = snapshot
            self.seeded_from_state = True
            self.latest_positions = positions
            self.current_targets = dict(positions)
            self.desired_targets = dict(positions)
            self.latest_state_time = timestamp
            self.mode_machine = mode_machine
            self._updating_controls = True
            try:
                for joint, control in self.controls_by_joint.items():
                    spec = control.spec
                    center = float(positions[joint])
                    lo = max(spec.limit_min, center - DEFAULT_SOFT_RANGE_RAD)
                    hi = min(spec.limit_max, center + DEFAULT_SOFT_RANGE_RAD)
                    if hi <= lo:
                        lo = max(spec.limit_min, center - 0.5)
                        hi = min(spec.limit_max, center + 0.5)
                    control.min_box.setValue(lo)
                    control.max_box.setValue(hi)
                    self._update_slider_range(joint)
            finally:
                self._updating_controls = False
            self._refresh_all_body_labels()

        for hand in self.hand_sides:
            snapshot_hand = self.hand_subs[hand].snapshot()
            if snapshot_hand is None:
                continue
            positions, timestamp = snapshot_hand
            self.hand_latest_positions[hand] = positions
            self.hand_current_targets[hand] = list(positions)
            self.hand_desired_targets[hand] = list(positions)
            self.hand_latest_state_time[hand] = timestamp
            self._updating_controls = True
            try:
                for joint_index in range(7):
                    control = self.hand_controls[(hand, joint_index)]
                    lo = max(float(HAND_MIN_LIMITS[hand][joint_index]),
                             positions[joint_index] - DEFAULT_SOFT_RANGE_RAD)
                    hi = min(float(HAND_MAX_LIMITS[hand][joint_index]),
                             positions[joint_index] + DEFAULT_SOFT_RANGE_RAD)
                    if hi <= lo:
                        lo = float(HAND_MIN_LIMITS[hand][joint_index])
                        hi = float(HAND_MAX_LIMITS[hand][joint_index])
                    control.min_box.setValue(lo)
                    control.max_box.setValue(hi)
                    self._update_hand_slider_range(hand, joint_index)
            finally:
                self._updating_controls = False
            self._refresh_all_hand_labels(hand)

    def _joint_limits(self, joint: int) -> tuple[float, float]:
        control = self.controls_by_joint[joint]
        spec = control.spec
        lo = max(spec.limit_min, float(control.min_box.value()))
        hi = min(spec.limit_max, float(control.max_box.value()))
        if hi <= lo:
            lo = spec.limit_min
            hi = spec.limit_max
        return lo, hi

    def _hand_joint_limits(self, hand: str, joint_index: int) -> tuple[float, float]:
        control = self.hand_controls[(hand, joint_index)]
        lo = max(float(HAND_MIN_LIMITS[hand][joint_index]), float(control.min_box.value()))
        hi = min(float(HAND_MAX_LIMITS[hand][joint_index]), float(control.max_box.value()))
        if hi <= lo:
            lo = float(HAND_MIN_LIMITS[hand][joint_index])
            hi = float(HAND_MAX_LIMITS[hand][joint_index])
        return lo, hi

    def _update_slider_range(self, joint: int) -> None:
        control = self.controls_by_joint[joint]
        lo, hi = self._joint_limits(joint)
        desired = max(lo, min(hi, float(self.desired_targets[joint])))
        self.desired_targets[joint] = desired

        self._updating_controls = True
        try:
            control.slider.setMinimum(int(round(lo * SLIDER_SCALE)))
            control.slider.setMaximum(int(round(hi * SLIDER_SCALE)))
            control.slider.setTickInterval(max(1, int(round((hi - lo) * SLIDER_SCALE / 10.0))))
            control.slider.setValue(int(round(desired * SLIDER_SCALE)))
        finally:
            self._updating_controls = False
        self._update_joint_labels(joint)

    def _update_hand_slider_range(self, hand: str, joint_index: int) -> None:
        control = self.hand_controls[(hand, joint_index)]
        lo, hi = self._hand_joint_limits(hand, joint_index)
        desired = max(lo, min(hi, float(self.hand_desired_targets[hand][joint_index])))
        self.hand_desired_targets[hand][joint_index] = desired

        self._updating_controls = True
        try:
            control.slider.setMinimum(int(round(lo * SLIDER_SCALE)))
            control.slider.setMaximum(int(round(hi * SLIDER_SCALE)))
            control.slider.setTickInterval(max(1, int(round((hi - lo) * SLIDER_SCALE / 10.0))))
            control.slider.setValue(int(round(desired * SLIDER_SCALE)))
        finally:
            self._updating_controls = False
        self._update_hand_joint_labels(hand, joint_index)

    def _set_desired_to_current(self, joint: int) -> None:
        self.desired_targets[joint] = float(
            self.latest_positions.get(joint, self.current_targets[joint]))
        self._update_slider_range(joint)

    def _set_hand_desired_to_current(self, hand: str, joint_index: int) -> None:
        self.hand_desired_targets[hand][joint_index] = float(
            self.hand_latest_positions.get(hand, self.hand_current_targets[hand])[joint_index]
        )
        self._update_hand_slider_range(hand, joint_index)

    def _sync_desired_to_current(self) -> None:
        for joint in self.all_joints:
            self.desired_targets[joint] = float(
                self.latest_positions.get(joint, self.current_targets[joint]))
            self._update_slider_range(joint)
        for hand in self.hand_sides:
            positions = self.hand_latest_positions.get(hand, self.hand_current_targets[hand])
            self.hand_desired_targets[hand] = list(positions)
            for joint_index in range(7):
                self._update_hand_slider_range(hand, joint_index)

    def _open_hands(self) -> None:
        for hand in self.hand_sides:
            self.hand_desired_targets[hand] = hand_open_targets(hand)
            for joint_index in range(7):
                self._update_hand_slider_range(hand, joint_index)

    def _zero_gains_once(self) -> None:
        self.controller.write_zero_gains_once(self.current_targets, mode_machine=self.mode_machine)
        self.status_text = f"Zero-gain body stop sent on {BODY_COMMAND_TOPIC}"
        self._refresh_status_label()

    def _on_slider_changed(self, joint: int, raw_value: int) -> None:
        if self._updating_controls:
            return
        lo, hi = self._joint_limits(joint)
        self.desired_targets[joint] = max(lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self._update_joint_labels(joint)

    def _on_hand_slider_changed(self, hand: str, joint_index: int, raw_value: int) -> None:
        if self._updating_controls:
            return
        lo, hi = self._hand_joint_limits(hand, joint_index)
        self.hand_desired_targets[hand][joint_index] = max(
            lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self._update_hand_joint_labels(hand, joint_index)

    def _update_joint_labels(self, joint: int) -> None:
        control = self.controls_by_joint[joint]
        current = float(self.latest_positions.get(joint, self.current_targets[joint]))
        desired = float(self.desired_targets[joint])
        lo, hi = self._joint_limits(joint)
        control.current_label.setText(f"{current: .3f} rad")
        control.desired_label.setText(f"{desired: .3f} rad")
        control.slider.setToolTip(
            f"{BODY_GROUP_TITLES.get(control.spec.group, control.spec.group)} {control.spec.name} motor {joint} | "
            f"current {current:.3f} | desired {desired:.3f} | software range [{lo:.3f}, {hi:.3f}] | "
            f"hard limits [{control.spec.limit_min:.3f}, {control.spec.limit_max:.3f}]"
        )

    def _update_hand_joint_labels(self, hand: str, joint_index: int) -> None:
        control = self.hand_controls[(hand, joint_index)]
        current = float(self.hand_latest_positions.get(
            hand, self.hand_current_targets[hand])[joint_index])
        desired = float(self.hand_desired_targets[hand][joint_index])
        lo, hi = self._hand_joint_limits(hand, joint_index)
        control.current_label.setText(f"{current: .3f} rad")
        control.desired_label.setText(f"{desired: .3f} rad")
        control.slider.setToolTip(
            f"{hand} Dex3 {control.name} | current {current:.3f} | desired {desired:.3f} | "
            f"software range [{lo:.3f}, {hi:.3f}]"
        )

    def _refresh_all_body_labels(self) -> None:
        for joint in self.all_joints:
            self._update_joint_labels(joint)

    def _refresh_all_hand_labels(self, hand: str) -> None:
        for joint_index in range(7):
            self._update_hand_joint_labels(hand, joint_index)

    def _refresh_status_label(self) -> None:
        body_age = "n/a" if self.latest_state_time <= 0.0 else f"{max(0.0, time.time() - self.latest_state_time):.2f}s"
        left_age = self.hand_latest_state_time["left"]
        right_age = self.hand_latest_state_time["right"]
        left_text = "n/a" if left_age <= 0.0 else f"{max(0.0, time.time() - left_age):.2f}s"
        right_text = "n/a" if right_age <= 0.0 else f"{max(0.0, time.time() - right_age):.2f}s"
        self.status_label.setText(
            f"{self.status_text} | body state {body_age} | left hand {left_text} | right hand {right_text}"
        )

    def _tick(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, timestamp, mode_machine = snapshot
            self.latest_positions = positions
            self.latest_state_time = timestamp
            self.mode_machine = mode_machine
            if not self.seeded_from_state:
                self._seed_from_state_if_available()

        for hand in self.hand_sides:
            snapshot_hand = self.hand_subs[hand].snapshot()
            if snapshot_hand is not None:
                positions, timestamp = snapshot_hand
                self.hand_latest_positions[hand] = positions
                self.hand_latest_state_time[hand] = timestamp

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, min(0.2, now - self.last_tick_s))
        self.last_tick_s = now
        max_delta = self.speed_rad_s * dt

        next_targets = dict(self.current_targets)
        body_changed = False
        for joint in self.all_joints:
            current = float(self.current_targets[joint])
            desired = float(self.desired_targets[joint])
            error = desired - current
            if abs(error) <= max_delta:
                next_value = desired
            else:
                next_value = current + max_delta * (1.0 if error > 0.0 else -1.0)
            if abs(next_value - current) > 1e-6:
                body_changed = True
            next_targets[joint] = next_value
        self.current_targets = next_targets
        self.controller.write_targets_once(
            self.current_targets,
            kp=self.kp,
            kd=self.kd,
            tau=self.tau,
            mode_machine=self.mode_machine,
        )

        hand_changed = False
        for hand in self.hand_sides:
            current_list = list(self.hand_current_targets[hand])
            desired_list = list(self.hand_desired_targets[hand])
            next_list = list(current_list)
            for joint_index in range(7):
                current = float(current_list[joint_index])
                desired = float(desired_list[joint_index])
                error = desired - current
                if abs(error) <= max_delta:
                    next_value = desired
                else:
                    next_value = current + max_delta * (1.0 if error > 0.0 else -1.0)
                if abs(next_value - current) > 1e-6:
                    hand_changed = True
                next_list[joint_index] = next_value
            self.hand_current_targets[hand] = next_list
            controller = self.hand_controllers.get(hand)
            if controller is not None:
                controller.write_targets_once(
                    next_list,
                    kp=self.hand_kp,
                    kd=self.hand_kd,
                    tau=self.hand_tau,
                    timeout=0,
                )

        if body_changed or snapshot is not None:
            self._refresh_all_body_labels()
        for hand in self.hand_sides:
            if hand_changed or self.hand_latest_state_time[hand] > 0.0:
                self._refresh_all_hand_labels(hand)

        self._refresh_status_label()


def main() -> int:
    args, qt_argv = parse_args()
    ChannelFactoryInitialize(int(args.domain_id), str(args.iface))
    app = QApplication(qt_argv)
    window = RobotJointSliderApp(args)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
