#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

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
        QLineEdit,
        QListWidget,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSlider,
        QSplitter,
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

from sdk_hand import Dex3HandController, HAND_MAX_LIMITS, HAND_MIN_LIMITS, hand_open_targets


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_IDX = [12, 13, 14]
ARM_GROUPS = {
    "waist": WAIST_IDX,
    "left_arm": LEFT_ARM_IDX,
    "right_arm": RIGHT_ARM_IDX,
}
ARM_GROUP_TITLES = {
    "waist": "Waist",
    "left_arm": "Left Arm",
    "right_arm": "Right Arm",
}
WAIST_JOINT_NAMES = [
    "yaw",
    "roll",
    "pitch",
]
ARM_JOINT_NAMES = [
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
]
ARM_LIMITS = {
    12: (-2.618, 2.618),
    13: (-0.52, 0.52),
    14: (-0.52, 0.52),
    15: (-3.0892, 2.6704),
    16: (-1.5882, 2.2515),
    17: (-2.618, 2.618),
    18: (-1.0472, 2.0944),
    19: (-1.9722, 1.9722),
    20: (-1.6144, 1.6144),
    21: (-1.6144, 1.6144),
    22: (-3.0892, 2.6704),
    23: (-2.2515, 1.5882),
    24: (-2.618, 2.618),
    25: (-1.0472, 2.0944),
    26: (-1.9722, 1.9722),
    27: (-1.6144, 1.6144),
    28: (-1.6144, 1.6144),
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

NOT_USED_IDX = 29
SLIDER_SCALE = 1000
DEFAULT_SOFT_RANGE_RAD = 1.0
ABS_RANGE_LIMIT_RAD = 3.14
DEFAULT_POSE_FILE = os.path.join(SCRIPT_DIR, "saved_arm_hand_poses.json")
MAX_JOINT_INCREMENT_RAD = 0.005


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="PyQt arm and Dex3 hand pose teaching UI with sliders, pose save, zero torque, and replay."
    )
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--file", default=DEFAULT_POSE_FILE, help="Saved arm+hand pose JSON file.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument("--speed-rad-s", type=float, default=0.4, help="Maximum arm joint transition speed.")
    parser.add_argument("--hand-speed-rad-s", type=float, default=0.6, help="Maximum finger joint transition speed.")
    parser.add_argument("--kp", type=float, default=30.0, help="Arm joint proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Arm joint feed-forward torque.")
    parser.add_argument("--hand-kp", type=float, default=0.5, help="Dex3 finger proportional gain.")
    parser.add_argument("--hand-kd", type=float, default=0.1, help="Dex3 finger derivative gain.")
    parser.add_argument("--hand-tau", type=float, default=0.0, help="Dex3 finger feed-forward torque.")
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


@dataclass
class ArmJointControl:
    group_name: str
    motor_index: int
    joint_name: str
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
    joint_name: str
    current_label: QLabel
    desired_label: QLabel
    slider: QSlider
    min_box: QDoubleSpinBox
    max_box: QDoubleSpinBox
    reset_button: QPushButton


class ArmStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(joint) for joint in joints]
        self._lock = threading.Lock()
        self._positions: dict[int, float] = {}
        self._timestamp = 0.0

        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        self._sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            positions = {joint: float(msg.motor_state[joint].q) for joint in self.joints}
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[dict[int, float], float] | None:
        with self._lock:
            if not self._positions:
                return None
            return dict(self._positions), float(self._timestamp)


class HandStateSubscriber:
    TOPIC_BY_SIDE = {
        "left": "rt/dex3/left/state",
        "right": "rt/dex3/right/state",
    }

    def __init__(self, hand: str) -> None:
        self.hand = str(hand)
        self._lock = threading.Lock()
        self._positions = hand_open_targets(self.hand)
        self._timestamp = 0.0
        self._sub = ChannelSubscriber(self.TOPIC_BY_SIDE[self.hand], HandState_)
        self._sub.Init(self._callback, 50)

    def _callback(self, msg: Any) -> None:
        try:
            positions = [float(msg.motor_state[idx].q) for idx in range(7)]
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[list[float], float]:
        with self._lock:
            return list(self._positions), float(self._timestamp)


class ArmPoseController:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(joint) for joint in joints]
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        for joint in self.joints:
            self._cmd.motor_cmd[joint].mode = 1

    def write_targets_once(self, targets_by_joint: dict[int, float], *, kp: float, kd: float, tau: float) -> None:
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

    def write_zero_gains_once(self, hold_positions: dict[int, float]) -> None:
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


class ArmHandPoseTeachApp(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.pose_path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.hand_speed_rad_s = max(0.01, float(args.hand_speed_rad_s))
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.tau = float(args.tau)
        self.hand_kp = float(args.hand_kp)
        self.hand_kd = float(args.hand_kd)
        self.hand_tau = float(args.hand_tau)

        self.arm_joints = list(WAIST_IDX) + list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX)
        self.hand_sides = ["left", "right"]

        self.arm_current_targets = {joint: 0.0 for joint in self.arm_joints}
        self.arm_desired_targets = dict(self.arm_current_targets)
        self.arm_latest_positions = dict(self.arm_current_targets)
        self.arm_latest_state_time = 0.0

        self.hand_current_targets = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_desired_targets = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_latest_positions = {hand: hand_open_targets(hand) for hand in self.hand_sides}
        self.hand_latest_state_time = {hand: 0.0 for hand in self.hand_sides}

        self.saved_poses: list[dict[str, Any]] = []
        self.arm_controls: dict[int, ArmJointControl] = {}
        self.hand_controls: dict[tuple[str, int], HandJointControl] = {}
        self._updating_controls = False
        self._seeded = False
        self.last_tick_s = time.monotonic()
        self.control_enabled = True
        self.status_text = "Initializing..."

        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.arm_state_sub = ArmStateSubscriber(self.arm_joints)
        self.arm_controller = ArmPoseController(self.arm_joints)
        self.hand_state_subs = {hand: HandStateSubscriber(hand) for hand in self.hand_sides}
        self.hand_controllers: dict[str, Dex3HandController | None] = {}
        for hand in self.hand_sides:
            try:
                self.hand_controllers[hand] = Dex3HandController(hand=hand, iface=self.iface, domain_id=self.domain_id)
            except Exception:
                self.hand_controllers[hand] = None

        self._build_ui()
        self._load_saved_poses()
        self._seed_from_live_state()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(max(10, int(round(1000.0 / self.rate_hz))))

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        gains = QFormLayout()
        self.speed_box = self._make_spinbox(0.01, 5.0, self.speed_rad_s, 0.05)
        self.speed_box.valueChanged.connect(lambda value: setattr(self, "speed_rad_s", float(value)))
        gains.addRow("Arm speed rad/s", self.speed_box)

        self.hand_speed_box = self._make_spinbox(0.01, 5.0, self.hand_speed_rad_s, 0.05)
        self.hand_speed_box.valueChanged.connect(lambda value: setattr(self, "hand_speed_rad_s", float(value)))
        gains.addRow("Hand speed rad/s", self.hand_speed_box)

        self.kp_box = self._make_spinbox(0.0, 100.0, self.kp, 0.5)
        self.kp_box.valueChanged.connect(lambda value: setattr(self, "kp", float(value)))
        gains.addRow("Arm kp", self.kp_box)

        self.kd_box = self._make_spinbox(0.0, 20.0, self.kd, 0.1)
        self.kd_box.valueChanged.connect(lambda value: setattr(self, "kd", float(value)))
        gains.addRow("Arm kd", self.kd_box)

        self.hand_kp_box = self._make_spinbox(0.0, 5.0, self.hand_kp, 0.05)
        self.hand_kp_box.valueChanged.connect(lambda value: setattr(self, "hand_kp", float(value)))
        gains.addRow("Hand kp", self.hand_kp_box)

        self.hand_kd_box = self._make_spinbox(0.0, 1.0, self.hand_kd, 0.01)
        self.hand_kd_box.valueChanged.connect(lambda value: setattr(self, "hand_kd", float(value)))
        gains.addRow("Hand kd", self.hand_kd_box)
        root.addLayout(gains)

        action_row = QHBoxLayout()
        self.sync_button = QPushButton("Sync Desired To Current", self)
        self.sync_button.clicked.connect(self._sync_all_to_current)
        action_row.addWidget(self.sync_button)

        self.zero_torque_button = QPushButton("Zero Torque", self)
        self.zero_torque_button.clicked.connect(self._send_zero_torque)
        action_row.addWidget(self.zero_torque_button)

        self.control_toggle_button = QPushButton("Disable Slider Control", self)
        self.control_toggle_button.clicked.connect(self._toggle_slider_control)
        action_row.addWidget(self.control_toggle_button)

        self.replay_button = QPushButton("Replay Saved Pose", self)
        self.replay_button.clicked.connect(self._replay_selected_pose)
        action_row.addWidget(self.replay_button)
        root.addLayout(action_row)

        splitter = QSplitter(Qt.Horizontal, self)

        left_panel = QWidget(self)
        left_layout = QVBoxLayout()
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll_contents = QWidget()
        scroll_layout = QVBoxLayout()
        for group_name, joints in ARM_GROUPS.items():
            scroll_layout.addWidget(self._build_joint_group_box(group_name, joints))
        for hand in self.hand_sides:
            scroll_layout.addWidget(self._build_hand_group_box(hand))
        scroll_contents.setLayout(scroll_layout)
        scroll.setWidget(scroll_contents)
        left_layout.addWidget(scroll)
        left_panel.setLayout(left_layout)
        splitter.addWidget(left_panel)

        right_panel = QWidget(self)
        right_layout = QVBoxLayout()
        pose_form = QFormLayout()
        self.pose_file_edit = QLineEdit(str(self.pose_path), self)
        self.pose_file_edit.editingFinished.connect(self._on_pose_file_changed)
        pose_form.addRow("Pose file", self.pose_file_edit)

        self.pose_name_edit = QLineEdit(self)
        self.pose_name_edit.setPlaceholderText("pose name")
        pose_form.addRow("Pose name", self.pose_name_edit)
        right_layout.addLayout(pose_form)

        pose_buttons = QHBoxLayout()
        self.save_button = QPushButton("Save Current Desired Pose", self)
        self.save_button.clicked.connect(self._save_current_pose)
        pose_buttons.addWidget(self.save_button)

        self.delete_button = QPushButton("Delete Selected", self)
        self.delete_button.clicked.connect(self._delete_selected_pose)
        pose_buttons.addWidget(self.delete_button)
        right_layout.addLayout(pose_buttons)

        self.pose_list = QListWidget(self)
        self.pose_list.itemDoubleClicked.connect(lambda _item: self._replay_selected_pose())
        right_layout.addWidget(self.pose_list)

        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.status_label.setWordWrap(True)
        right_layout.addWidget(self.status_label)

        right_panel.setLayout(right_layout)
        splitter.addWidget(right_panel)
        splitter.setSizes([1100, 360])

        root.addWidget(splitter)
        self.setLayout(root)
        self.setWindowTitle("Arm And Hand Pose Teach UI")
        self.resize(1600, 980)

    def _joint_names_for_group(self, group_name: str) -> list[str]:
        if group_name == "waist":
            return WAIST_JOINT_NAMES
        return ARM_JOINT_NAMES

    def _build_joint_group_box(self, group_name: str, joints: list[int]) -> QGroupBox:
        box = QGroupBox(ARM_GROUP_TITLES[group_name])
        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Current"), 0, 1)
        grid.addWidget(QLabel("Desired"), 0, 2)
        grid.addWidget(QLabel("Min"), 0, 3)
        grid.addWidget(QLabel("Slider"), 0, 4)
        grid.addWidget(QLabel("Max"), 0, 5)
        grid.addWidget(QLabel(""), 0, 6)
        joint_names = self._joint_names_for_group(group_name)
        for row, joint in enumerate(joints, start=1):
            control = self._make_arm_joint_row(group_name, joint, joint_names[row - 1])
            grid.addWidget(QLabel(f"{joint}: {control.joint_name}"), row, 0)
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
            grid.addWidget(QLabel(f"{joint_index}: {control.joint_name}"), row, 0)
            grid.addWidget(control.current_label, row, 1)
            grid.addWidget(control.desired_label, row, 2)
            grid.addWidget(control.min_box, row, 3)
            grid.addWidget(control.slider, row, 4)
            grid.addWidget(control.max_box, row, 5)
            grid.addWidget(control.reset_button, row, 6)
        box.setLayout(grid)
        return box

    def _make_arm_joint_row(self, group_name: str, motor_index: int, joint_name: str) -> ArmJointControl:
        current_label = QLabel(" -- ", self)
        desired_label = QLabel(" -- ", self)
        hard_min, hard_max = ARM_LIMITS[motor_index]

        min_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD, ABS_RANGE_LIMIT_RAD, hard_min, 0.05)
        max_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD, ABS_RANGE_LIMIT_RAD, hard_max, 0.05)
        min_box.valueChanged.connect(lambda _value, joint=motor_index: self._update_arm_slider_range(joint))
        max_box.valueChanged.connect(lambda _value, joint=motor_index: self._update_arm_slider_range(joint))

        slider = QSlider(Qt.Horizontal, self)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.valueChanged.connect(lambda raw, joint=motor_index: self._on_arm_slider_changed(joint, raw))

        reset_button = QPushButton("Current", self)
        reset_button.clicked.connect(lambda _checked=False, joint=motor_index: self._set_arm_desired_to_current(joint))

        control = ArmJointControl(
            group_name=group_name,
            motor_index=motor_index,
            joint_name=joint_name,
            current_label=current_label,
            desired_label=desired_label,
            slider=slider,
            min_box=min_box,
            max_box=max_box,
            reset_button=reset_button,
        )
        self.arm_controls[motor_index] = control
        self._update_arm_slider_range(motor_index)
        return control

    def _make_hand_joint_row(self, hand: str, joint_index: int) -> HandJointControl:
        current_label = QLabel(" -- ", self)
        desired_label = QLabel(" -- ", self)

        min_box = self._make_spinbox(
            -ABS_RANGE_LIMIT_RAD,
            ABS_RANGE_LIMIT_RAD,
            float(HAND_MIN_LIMITS[hand][joint_index]),
            0.05,
        )
        max_box = self._make_spinbox(
            -ABS_RANGE_LIMIT_RAD,
            ABS_RANGE_LIMIT_RAD,
            float(HAND_MAX_LIMITS[hand][joint_index]),
            0.05,
        )
        min_box.valueChanged.connect(lambda _value, side=hand, idx=joint_index: self._update_hand_slider_range(side, idx))
        max_box.valueChanged.connect(lambda _value, side=hand, idx=joint_index: self._update_hand_slider_range(side, idx))

        slider = QSlider(Qt.Horizontal, self)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.valueChanged.connect(lambda raw, side=hand, idx=joint_index: self._on_hand_slider_changed(side, idx, raw))

        reset_button = QPushButton("Current", self)
        reset_button.clicked.connect(
            lambda _checked=False, side=hand, idx=joint_index: self._set_hand_desired_to_current(side, idx)
        )

        control = HandJointControl(
            hand=hand,
            joint_index=joint_index,
            joint_name=HAND_JOINT_NAMES[joint_index],
            current_label=current_label,
            desired_label=desired_label,
            slider=slider,
            min_box=min_box,
            max_box=max_box,
            reset_button=reset_button,
        )
        self.hand_controls[(hand, joint_index)] = control
        self._update_hand_slider_range(hand, joint_index)
        return control

    @staticmethod
    def _make_spinbox(minimum: float, maximum: float, value: float, step: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setDecimals(3)
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def _arm_joint_limits(self, joint: int) -> tuple[float, float]:
        control = self.arm_controls[joint]
        hard_min, hard_max = ARM_LIMITS[joint]
        lo = max(hard_min, float(control.min_box.value()))
        hi = min(hard_max, float(control.max_box.value()))
        if hi <= lo:
            lo, hi = hard_min, hard_max
        return lo, hi

    def _hand_joint_limits(self, hand: str, joint_index: int) -> tuple[float, float]:
        control = self.hand_controls[(hand, joint_index)]
        lo = max(float(HAND_MIN_LIMITS[hand][joint_index]), float(control.min_box.value()))
        hi = min(float(HAND_MAX_LIMITS[hand][joint_index]), float(control.max_box.value()))
        if hi <= lo:
            lo = float(HAND_MIN_LIMITS[hand][joint_index])
            hi = float(HAND_MAX_LIMITS[hand][joint_index])
        return lo, hi

    def _seed_from_live_state(self) -> None:
        arm_snapshot = self.arm_state_sub.snapshot()
        if arm_snapshot is not None:
            positions, timestamp = arm_snapshot
            self.arm_latest_positions = positions
            self.arm_current_targets = dict(positions)
            self.arm_desired_targets = dict(positions)
            self.arm_latest_state_time = timestamp

            self._updating_controls = True
            try:
                for joint in self.arm_joints:
                    control = self.arm_controls[joint]
                    center = float(positions[joint])
                    hard_min, hard_max = ARM_LIMITS[joint]
                    lo = max(hard_min, center - DEFAULT_SOFT_RANGE_RAD)
                    hi = min(hard_max, center + DEFAULT_SOFT_RANGE_RAD)
                    control.min_box.setValue(lo)
                    control.max_box.setValue(hi)
                    self._update_arm_slider_range(joint)
            finally:
                self._updating_controls = False

        for hand in self.hand_sides:
            positions, timestamp = self.hand_state_subs[hand].snapshot()
            self.hand_latest_positions[hand] = list(positions)
            self.hand_current_targets[hand] = list(positions)
            self.hand_desired_targets[hand] = list(positions)
            self.hand_latest_state_time[hand] = timestamp

            self._updating_controls = True
            try:
                for joint_index in range(7):
                    control = self.hand_controls[(hand, joint_index)]
                    lo = max(float(HAND_MIN_LIMITS[hand][joint_index]), positions[joint_index] - DEFAULT_SOFT_RANGE_RAD)
                    hi = min(float(HAND_MAX_LIMITS[hand][joint_index]), positions[joint_index] + DEFAULT_SOFT_RANGE_RAD)
                    control.min_box.setValue(lo)
                    control.max_box.setValue(hi)
                    self._update_hand_slider_range(hand, joint_index)
            finally:
                self._updating_controls = False

        self._seeded = arm_snapshot is not None
        self._refresh_all_labels()

    def _refresh_all_labels(self) -> None:
        for joint in self.arm_joints:
            self._update_arm_labels(joint)
        for hand in self.hand_sides:
            for joint_index in range(7):
                self._update_hand_labels(hand, joint_index)
        self._refresh_status_label()

    def _update_arm_slider_range(self, joint: int) -> None:
        control = self.arm_controls[joint]
        lo, hi = self._arm_joint_limits(joint)
        desired = max(lo, min(hi, float(self.arm_desired_targets[joint])))
        self.arm_desired_targets[joint] = desired

        self._updating_controls = True
        try:
            control.slider.setMinimum(int(round(lo * SLIDER_SCALE)))
            control.slider.setMaximum(int(round(hi * SLIDER_SCALE)))
            control.slider.setTickInterval(max(1, int(round((hi - lo) * SLIDER_SCALE / 10.0))))
            control.slider.setValue(int(round(desired * SLIDER_SCALE)))
        finally:
            self._updating_controls = False
        self._update_arm_labels(joint)

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
        self._update_hand_labels(hand, joint_index)

    def _update_arm_labels(self, joint: int) -> None:
        control = self.arm_controls[joint]
        current = float(self.arm_latest_positions.get(joint, self.arm_current_targets[joint]))
        desired = float(self.arm_desired_targets[joint])
        lo, hi = self._arm_joint_limits(joint)
        hard_min, hard_max = ARM_LIMITS[joint]
        control.current_label.setText(f"{current: .3f}")
        control.desired_label.setText(f"{desired: .3f}")
        control.slider.setToolTip(
            f"{control.group_name} joint {joint} {control.joint_name} | current {current:.3f} | "
            f"desired {desired:.3f} | range [{lo:.3f}, {hi:.3f}] | hard [{hard_min:.3f}, {hard_max:.3f}]"
        )

    def _update_hand_labels(self, hand: str, joint_index: int) -> None:
        control = self.hand_controls[(hand, joint_index)]
        current = float(self.hand_latest_positions[hand][joint_index])
        desired = float(self.hand_desired_targets[hand][joint_index])
        lo, hi = self._hand_joint_limits(hand, joint_index)
        control.current_label.setText(f"{current: .3f}")
        control.desired_label.setText(f"{desired: .3f}")
        control.slider.setToolTip(
            f"{hand} hand joint {joint_index} {control.joint_name} | current {current:.3f} | "
            f"desired {desired:.3f} | range [{lo:.3f}, {hi:.3f}]"
        )

    def _on_arm_slider_changed(self, joint: int, raw_value: int) -> None:
        if self._updating_controls:
            return
        lo, hi = self._arm_joint_limits(joint)
        self.arm_desired_targets[joint] = max(lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self._update_arm_labels(joint)

    def _on_hand_slider_changed(self, hand: str, joint_index: int, raw_value: int) -> None:
        if self._updating_controls:
            return
        lo, hi = self._hand_joint_limits(hand, joint_index)
        self.hand_desired_targets[hand][joint_index] = max(lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self._update_hand_labels(hand, joint_index)

    def _set_arm_desired_to_current(self, joint: int) -> None:
        self.arm_desired_targets[joint] = float(self.arm_latest_positions.get(joint, self.arm_current_targets[joint]))
        self._update_arm_slider_range(joint)

    def _set_hand_desired_to_current(self, hand: str, joint_index: int) -> None:
        self.hand_desired_targets[hand][joint_index] = float(self.hand_latest_positions[hand][joint_index])
        self._update_hand_slider_range(hand, joint_index)

    def _sync_all_to_current(self) -> None:
        for joint in self.arm_joints:
            self.arm_desired_targets[joint] = float(self.arm_latest_positions.get(joint, self.arm_current_targets[joint]))
            self._update_arm_slider_range(joint)
        for hand in self.hand_sides:
            for joint_index in range(7):
                self.hand_desired_targets[hand][joint_index] = float(self.hand_latest_positions[hand][joint_index])
                self._update_hand_slider_range(hand, joint_index)
        self.status_text = "Desired targets synced to current live state."
        self._refresh_status_label()

    def _seed_command_state_from_live(self) -> None:
        self.arm_current_targets = {
            joint: float(self.arm_latest_positions.get(joint, self.arm_current_targets[joint]))
            for joint in self.arm_joints
        }
        for hand in self.hand_sides:
            self.hand_current_targets[hand] = [float(value) for value in self.hand_latest_positions[hand]]

    def _send_zero_torque(self) -> None:
        self._seed_command_state_from_live()
        self.arm_controller.write_zero_gains_once(self.arm_latest_positions)
        for hand in self.hand_sides:
            controller = self.hand_controllers.get(hand)
            if controller is not None:
                controller.write_targets_once(
                    self.hand_latest_positions[hand],
                    kp=0.0,
                    kd=0.0,
                    tau=0.0,
                    timeout=1,
                    first_write_timeout_s=1.0,
                )
        self.control_enabled = False
        self.control_toggle_button.setText("Enable Slider Control")
        self.status_text = "Zero torque sent to arms and both hands. Slider control is now disabled."
        self._refresh_status_label()

    def _toggle_slider_control(self) -> None:
        self.control_enabled = not self.control_enabled
        if self.control_enabled:
            self._seed_command_state_from_live()
            self.control_toggle_button.setText("Disable Slider Control")
            self.status_text = "Slider control enabled. Command state re-seeded from live joints before publishing."
        else:
            self.control_toggle_button.setText("Enable Slider Control")
            self.status_text = "Slider control disabled. No arm or hand targets are being published."
        self._refresh_status_label()

    def _pose_payload(self) -> dict[str, Any]:
        return {
            "name": self.pose_name_edit.text().strip(),
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {str(joint): float(self.arm_desired_targets[joint]) for joint in self.arm_joints},
            "waist_joints": {str(joint): float(self.arm_desired_targets[joint]) for joint in WAIST_IDX},
            "hands": {
                hand: [float(value) for value in self.hand_desired_targets[hand]]
                for hand in self.hand_sides
            },
        }

    def _normalized_pose_name(self) -> str:
        name = self.pose_name_edit.text().strip()
        if name:
            return name
        return f"pose_{len(self.saved_poses) + 1:03d}"

    def _load_saved_poses(self) -> None:
        self.saved_poses = []
        if not self.pose_path.exists():
            self._refresh_pose_list()
            return
        try:
            payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.status_text = f"Could not read pose file: {exc}"
            self._refresh_pose_list()
            return
        poses = payload.get("poses", [])
        if isinstance(poses, list):
            self.saved_poses = [pose for pose in poses if isinstance(pose, dict)]
        self._refresh_pose_list()

    def _write_saved_poses(self) -> None:
        payload = {"poses": self.saved_poses}
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _refresh_pose_list(self) -> None:
        selected_row = self.pose_list.currentRow()
        self.pose_list.clear()
        for idx, pose in enumerate(self.saved_poses):
            name = str(pose.get("name", f"pose_{idx}"))
            saved_at = str(pose.get("saved_at", "?"))
            self.pose_list.addItem(f"{idx}: {name} | {saved_at}")
        if self.saved_poses:
            self.pose_list.setCurrentRow(min(max(selected_row, 0), len(self.saved_poses) - 1))
        self._refresh_status_label()

    def _on_pose_file_changed(self) -> None:
        self.pose_path = Path(os.path.abspath(os.path.expanduser(self.pose_file_edit.text().strip() or str(self.pose_path))))
        self._load_saved_poses()
        self.status_text = f"Pose file set to {self.pose_path}"
        self._refresh_status_label()

    def _save_current_pose(self) -> None:
        pose = self._pose_payload()
        pose["name"] = self._normalized_pose_name()
        self.saved_poses.append(pose)
        self._write_saved_poses()
        self._refresh_pose_list()
        self.pose_name_edit.clear()
        self.status_text = f"Saved pose '{pose['name']}' to {self.pose_path}"
        self._refresh_status_label()

    def _selected_pose(self) -> dict[str, Any] | None:
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            return None
        return self.saved_poses[row]

    def _apply_pose_to_desired_targets(self, pose: dict[str, Any]) -> None:
        arm_joints = pose.get("arm_joints")
        waist_joints = pose.get("waist_joints")
        hands = pose.get("hands")
        if not isinstance(arm_joints, dict) or not isinstance(hands, dict):
            raise ValueError("Pose is missing arm_joints or hands.")

        if isinstance(waist_joints, dict):
            for joint in WAIST_IDX:
                key = str(joint)
                if key in waist_joints:
                    self.arm_desired_targets[joint] = float(waist_joints[key])
                    self._update_arm_slider_range(joint)

        for joint in self.arm_joints:
            key = str(joint)
            if key in arm_joints:
                self.arm_desired_targets[joint] = float(arm_joints[key])
                self._update_arm_slider_range(joint)

        for hand in self.hand_sides:
            values = hands.get(hand)
            if isinstance(values, list) and len(values) == 7:
                for joint_index in range(7):
                    self.hand_desired_targets[hand][joint_index] = float(values[joint_index])
                    self._update_hand_slider_range(hand, joint_index)

    def _replay_selected_pose(self) -> None:
        pose = self._selected_pose()
        if pose is None:
            QMessageBox.warning(self, "No Pose", "Select a saved pose first.")
            return
        try:
            self._apply_pose_to_desired_targets(pose)
        except Exception as exc:
            QMessageBox.warning(self, "Invalid Pose", str(exc))
            return
        name = str(pose.get("name", "<unnamed>"))
        self._seed_command_state_from_live()
        self.control_enabled = True
        self.control_toggle_button.setText("Disable Slider Control")
        self.status_text = f"Replaying saved pose '{name}' from the current live joint state."
        self._refresh_status_label()

    def _delete_selected_pose(self) -> None:
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            QMessageBox.warning(self, "No Pose", "Select a saved pose first.")
            return
        name = str(self.saved_poses[row].get("name", f"pose_{row}"))
        del self.saved_poses[row]
        self._write_saved_poses()
        self._refresh_pose_list()
        self.status_text = f"Deleted pose '{name}'."
        self._refresh_status_label()

    def _refresh_status_label(self) -> None:
        arm_age = "n/a" if self.arm_latest_state_time <= 0.0 else f"{max(0.0, time.time() - self.arm_latest_state_time):.2f}s"
        left_age = self.hand_latest_state_time["left"]
        right_age = self.hand_latest_state_time["right"]
        left_text = "n/a" if left_age <= 0.0 else f"{max(0.0, time.time() - left_age):.2f}s"
        right_text = "n/a" if right_age <= 0.0 else f"{max(0.0, time.time() - right_age):.2f}s"
        self.status_label.setText(
            f"{self.status_text}\n"
            f"Pose file: {self.pose_path}\n"
            f"Slider control: {'enabled' if self.control_enabled else 'disabled'}\n"
            f"Saved poses: {len(self.saved_poses)} | arm state age {arm_age} | left hand age {left_text} | right hand age {right_text}"
        )

    def _tick(self) -> None:
        arm_snapshot = self.arm_state_sub.snapshot()
        if arm_snapshot is not None:
            positions, timestamp = arm_snapshot
            self.arm_latest_positions = positions
            self.arm_latest_state_time = timestamp
            if not self._seeded:
                self._seed_from_live_state()

        for hand in self.hand_sides:
            positions, timestamp = self.hand_state_subs[hand].snapshot()
            self.hand_latest_positions[hand] = positions
            self.hand_latest_state_time[hand] = timestamp

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, min(0.2, now - self.last_tick_s))
        self.last_tick_s = now

        if not self.control_enabled:
            self._refresh_all_labels()
            return

        arm_step = min(MAX_JOINT_INCREMENT_RAD, self.speed_rad_s * dt)
        next_arm_targets = dict(self.arm_current_targets)
        for joint in self.arm_joints:
            current = float(self.arm_current_targets[joint])
            desired = float(self.arm_desired_targets[joint])
            error = desired - current
            if abs(error) <= arm_step:
                next_arm_targets[joint] = desired
            else:
                next_arm_targets[joint] = current + arm_step * (1.0 if error > 0.0 else -1.0)
        self.arm_current_targets = next_arm_targets
        self.arm_controller.write_targets_once(self.arm_current_targets, kp=self.kp, kd=self.kd, tau=self.tau)

        hand_step = min(MAX_JOINT_INCREMENT_RAD, self.hand_speed_rad_s * dt)
        for hand in self.hand_sides:
            controller = self.hand_controllers.get(hand)
            if controller is None:
                continue
            next_targets = list(self.hand_current_targets[hand])
            for idx, current in enumerate(self.hand_current_targets[hand]):
                desired = float(self.hand_desired_targets[hand][idx])
                error = desired - current
                if abs(error) <= hand_step:
                    next_targets[idx] = desired
                else:
                    next_targets[idx] = current + hand_step * (1.0 if error > 0.0 else -1.0)
            self.hand_current_targets[hand] = next_targets
            controller.write_targets_once(
                next_targets,
                kp=self.hand_kp,
                kd=self.hand_kd,
                tau=self.hand_tau,
                timeout=0,
                first_write_timeout_s=None,
            )

        self._refresh_all_labels()


def main() -> int:
    args, qt_argv = parse_args()
    app = QApplication(qt_argv)
    window = ArmHandPoseTeachApp(args)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
