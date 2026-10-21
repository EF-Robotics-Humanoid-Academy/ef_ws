#!/usr/bin/env python3
from __future__ import annotations
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

except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
NOT_USED_IDX = 29
JOINT_NAMES = [
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
    "wrist_yaw",
]
SLIDER_SCALE = 1000
DEFAULT_SOFT_RANGE_RAD = 1.0
ABS_RANGE_LIMIT_RAD = 3.14


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="PyQt whole-arm joint pose slider.")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="Low-level command publish rate.")
    parser.add_argument(
        "--speed-rad-s",
        type=float,
        default=0.5,
        help="Maximum commanded joint transition speed.",
    )
    parser.add_argument("--kp", type=float, default=30.0, help="Joint proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Feed-forward torque.")
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


def selected_joint_map(arm: str) -> dict[str, list[int]]:
    side = str(arm).strip().lower()
    if side == "left":
        return {"left": list(LEFT_ARM_IDX)}
    if side == "right":
        return {"right": list(RIGHT_ARM_IDX)}
    if side == "both":
        return {"left": list(LEFT_ARM_IDX), "right": list(RIGHT_ARM_IDX)}
    raise ValueError(f"Unsupported arm selection '{arm}'.")


@dataclass
class JointControl:
    arm_name: str
    joint_offset: int
    motor_index: int
    name: str
    current_label: QLabel
    desired_label: QLabel
    slider: QSlider
    min_box: QDoubleSpinBox
    max_box: QDoubleSpinBox
    reset_button: QPushButton


class ArmStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
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


class ArmPoseController:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0

    def write_targets_once(
        self,
        targets_by_joint: dict[int, float],
        *,
        kp: float,
        kd: float,
        tau: float,
    ) -> None:
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
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
            mc.q = float(hold_positions[joint])
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class ArmJointSliderApp(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.arm = str(args.arm)
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.tau = float(args.tau)

        self.arm_to_joints = selected_joint_map(self.arm)
        self.all_joints = [joint for joints in self.arm_to_joints.values() for joint in joints]
        self.current_targets = {joint: 0.0 for joint in self.all_joints}
        self.desired_targets = dict(self.current_targets)
        self.latest_positions = dict(self.current_targets)
        self.latest_state_time = 0.0
        self.controls_by_joint: dict[int, JointControl] = {}
        self._updating_controls = False
        self.last_tick_s = time.monotonic()
        self.seeded_from_state = False

        self.state_sub = ArmStateSubscriber(self.all_joints)
        self.controller = ArmPoseController(self.all_joints)

        self._build_ui()
        self._seed_from_state_if_available()

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
        gains.addRow("kp", self.kp_box)

        self.kd_box = self._make_spinbox(0.0, 20.0, self.kd, 0.1)
        self.kd_box.valueChanged.connect(lambda value: setattr(self, "kd", float(value)))
        gains.addRow("kd", self.kd_box)
        root.addLayout(gains)

        buttons = QHBoxLayout()
        self.sync_button = QPushButton("Sync Desired To Current", self)
        self.sync_button.clicked.connect(self._sync_desired_to_current)
        buttons.addWidget(self.sync_button)

        self.zero_button = QPushButton("Zero Gains", self)
        self.zero_button.clicked.connect(self._zero_gains_once)
        buttons.addWidget(self.zero_button)
        root.addLayout(buttons)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll_contents = QWidget()
        scroll_layout = QVBoxLayout()

        for arm_name, joints in self.arm_to_joints.items():
            box = QGroupBox(f"{arm_name.title()} Arm")
            grid = QGridLayout()
            grid.addWidget(QLabel("Joint"), 0, 0)
            grid.addWidget(QLabel("Current"), 0, 1)
            grid.addWidget(QLabel("Desired"), 0, 2)
            grid.addWidget(QLabel("Min"), 0, 3)
            grid.addWidget(QLabel("Slider"), 0, 4)
            grid.addWidget(QLabel("Max"), 0, 5)
            grid.addWidget(QLabel(""), 0, 6)

            for row, (joint_offset, motor_index) in enumerate(enumerate(joints), start=1):
                control = self._make_joint_row(arm_name, joint_offset, motor_index)
                self.controls_by_joint[motor_index] = control

                grid.addWidget(QLabel(f"{joint_offset}: {control.name} ({motor_index})"), row, 0)
                grid.addWidget(control.current_label, row, 1)
                grid.addWidget(control.desired_label, row, 2)
                grid.addWidget(control.min_box, row, 3)
                grid.addWidget(control.slider, row, 4)
                grid.addWidget(control.max_box, row, 5)
                grid.addWidget(control.reset_button, row, 6)

            box.setLayout(grid)
            scroll_layout.addWidget(box)

        scroll_contents.setLayout(scroll_layout)
        scroll.setWidget(scroll_contents)
        root.addWidget(scroll)

        self.status_label = QLabel("Waiting for rt/lowstate arm state...", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        self.setLayout(root)
        self.setWindowTitle(f"Arm Joint Pose Slider ({self.arm})")
        self.resize(1200, 620)

    def _make_joint_row(self, arm_name: str, joint_offset: int, motor_index: int) -> JointControl:
        current_label = QLabel(" -- ", self)
        desired_label = QLabel(" -- ", self)

        min_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD,
                                     ABS_RANGE_LIMIT_RAD, -DEFAULT_SOFT_RANGE_RAD, 0.05)
        max_box = self._make_spinbox(-ABS_RANGE_LIMIT_RAD,
                                     ABS_RANGE_LIMIT_RAD, DEFAULT_SOFT_RANGE_RAD, 0.05)
        min_box.valueChanged.connect(
            lambda _value, joint=motor_index: self._update_slider_range(joint))
        max_box.valueChanged.connect(
            lambda _value, joint=motor_index: self._update_slider_range(joint))

        slider = QSlider(Qt.Horizontal, self)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.valueChanged.connect(
            lambda raw, joint=motor_index: self._on_slider_changed(joint, raw))

        reset_button = QPushButton("Current", self)
        reset_button.clicked.connect(
            lambda _checked=False, joint=motor_index: self._set_desired_to_current(joint))

        control = JointControl(
            arm_name=arm_name,
            joint_offset=joint_offset,
            motor_index=motor_index,
            name=JOINT_NAMES[joint_offset],
            current_label=current_label,
            desired_label=desired_label,
            slider=slider,
            min_box=min_box,
            max_box=max_box,
            reset_button=reset_button,
        )
        self.controls_by_joint[motor_index] = control
        self._update_slider_range(motor_index)
        self._update_joint_labels(motor_index)
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
        if snapshot is None:
            return

        positions, _timestamp = snapshot
        self.seeded_from_state = True
        self.latest_positions = positions
        self.current_targets = dict(positions)
        self.desired_targets = dict(positions)
        self._updating_controls = True
        try:
            for joint, control in self.controls_by_joint.items():
                center = float(positions[joint])
                lo = max(-ABS_RANGE_LIMIT_RAD, center - DEFAULT_SOFT_RANGE_RAD)
                hi = min(ABS_RANGE_LIMIT_RAD, center + DEFAULT_SOFT_RANGE_RAD)
                if hi <= lo:
                    lo = max(-ABS_RANGE_LIMIT_RAD, center - 0.5)
                    hi = min(ABS_RANGE_LIMIT_RAD, center + 0.5)
                control.min_box.setValue(lo)
                control.max_box.setValue(hi)
                self._update_slider_range(joint)
        finally:
            self._updating_controls = False

        self._refresh_all_labels()
        self.status_label.setText(f"Connected: {self.arm} arm(s) on {self.iface}")

    def _joint_limits(self, joint: int) -> tuple[float, float]:
        control = self.controls_by_joint[joint]
        lo = float(control.min_box.value())
        hi = float(control.max_box.value())
        if hi <= lo:
            hi = lo + 0.001
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

    def _set_desired_to_current(self, joint: int) -> None:
        self.desired_targets[joint] = float(
            self.latest_positions.get(joint, self.current_targets[joint]))
        self._update_slider_range(joint)

    def _sync_desired_to_current(self) -> None:
        for joint in self.all_joints:
            self.desired_targets[joint] = float(
                self.latest_positions.get(joint, self.current_targets[joint]))
            self._update_slider_range(joint)

    def _zero_gains_once(self) -> None:
        self.controller.write_zero_gains_once(self.current_targets)
        self.status_label.setText("Zero-gain stop sent on rt/arm_sdk")

    def _on_slider_changed(self, joint: int, raw_value: int) -> None:
        if self._updating_controls:
            return
        lo, hi = self._joint_limits(joint)
        self.desired_targets[joint] = max(lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self._update_joint_labels(joint)

    def _update_joint_labels(self, joint: int) -> None:
        control = self.controls_by_joint[joint]
        current = float(self.latest_positions.get(joint, self.current_targets[joint]))
        desired = float(self.desired_targets[joint])
        lo, hi = self._joint_limits(joint)
        control.current_label.setText(f"{current: .3f} rad")
        control.desired_label.setText(f"{desired: .3f} rad")
        control.slider.setToolTip(
            f"{control.arm_name} {control.name} motor {joint} | current {current:.3f} | desired {desired:.3f} | "
            f"software range [{lo:.3f}, {hi:.3f}]"
        )

    def _refresh_all_labels(self) -> None:
        for joint in self.all_joints:
            self._update_joint_labels(joint)

    def _tick(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, timestamp = snapshot
            self.latest_positions = positions
            self.latest_state_time = timestamp
            if not self.seeded_from_state:
                self._seed_from_state_if_available()

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, min(0.2, now - self.last_tick_s))
        self.last_tick_s = now
        max_delta = self.speed_rad_s * dt

        next_targets = dict(self.current_targets)
        changed = False
        for joint in self.all_joints:
            current = float(self.current_targets[joint])
            desired = float(self.desired_targets[joint])
            error = desired - current
            if abs(error) <= max_delta:
                next_value = desired
            else:
                next_value = current + max_delta * (1.0 if error > 0.0 else -1.0)
            if abs(next_value - current) > 1e-6:
                changed = True
            next_targets[joint] = next_value

        self.current_targets = next_targets
        self.controller.write_targets_once(
            self.current_targets,
            kp=self.kp,
            kd=self.kd,
            tau=self.tau,
        )

        if self.latest_state_time > 0.0:
            age_s = max(0.0, time.time() - self.latest_state_time)
            self.status_label.setText(
                f"Publishing {len(self.all_joints)} joints on rt/arm_sdk via {self.iface} | "
                f"state age {age_s:.2f}s"
            )
        else:
            self.status_label.setText("Publishing commands; waiting for rt/lowstate arm state...")

        if changed or snapshot is not None:
            self._refresh_all_labels()


def main() -> int:
    args, qt_argv = parse_args()
    ChannelFactoryInitialize(int(args.domain_id), str(args.iface))
    app = QApplication(qt_argv)
    window = ArmJointSliderApp(args)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
