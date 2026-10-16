#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QApplication,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QPushButton,
        QSlider,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit("PyQt5 not found. Install it with: pip install PyQt5") from exc

from sdk_hand import Dex3HandController, HAND_MAX_LIMITS, HAND_MIN_LIMITS, hand_open_targets


JOINT_NAMES = [
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "index_0",
    "index_1",
    "middle_0",
    "middle_1",
]

SLIDER_SCALE = 1000


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="PyQt Dex3 joint pose slider.")
    parser.add_argument("--hand", choices=("right", "left"), default="right")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Low-level command publish rate.")
    parser.add_argument(
        "--speed-rad-s",
        type=float,
        default=0.45,
        help="Maximum commanded joint transition speed.",
    )
    parser.add_argument("--kp", type=float, default=0.5, help="Joint proportional gain.")
    parser.add_argument("--kd", type=float, default=0.1, help="Joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Feed-forward torque.")
    args, remaining = parser.parse_known_args()
    return args, [sys.argv[0], *remaining]


class Dex3JointSliderApp(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.kp = float(args.kp)
        self.kd = float(args.kd)
        self.tau = float(args.tau)

        self.hand = str(args.hand)
        self.controller: Dex3HandController | None = None
        self.current_targets = hand_open_targets(self.hand)
        self.desired_targets = list(self.current_targets)
        self.last_tick_s = time.monotonic()
        self.matched_once = False
        self._updating_controls = False

        self._build_ui()
        self._set_controller(self.hand)
        self._sync_joint_controls()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._publish_step)
        self.timer.start(int(round(1000.0 / self.rate_hz)))

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        selectors = QFormLayout()
        self.hand_combo = QComboBox(self)
        self.hand_combo.addItems(["right", "left"])
        self.hand_combo.setCurrentText(self.hand)
        self.hand_combo.currentTextChanged.connect(self._on_hand_changed)
        selectors.addRow("Hand", self.hand_combo)

        self.joint_combo = QComboBox(self)
        self.joint_combo.addItems([f"{idx}: {name}" for idx, name in enumerate(JOINT_NAMES)])
        self.joint_combo.currentIndexChanged.connect(self._sync_joint_controls)
        selectors.addRow("Joint", self.joint_combo)
        root.addLayout(selectors)

        slider_box = QGroupBox("Selected Joint Pose")
        slider_layout = QVBoxLayout()
        self.pose_label = QLabel("", self)
        self.pose_label.setAlignment(Qt.AlignCenter)
        slider_layout.addWidget(self.pose_label)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(100)
        self.slider.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider)
        slider_box.setLayout(slider_layout)
        root.addWidget(slider_box)

        gains = QFormLayout()
        self.speed_box = self._make_spinbox(0.01, 5.0, self.speed_rad_s, 0.05)
        self.speed_box.valueChanged.connect(lambda value: setattr(self, "speed_rad_s", float(value)))
        gains.addRow("Ramp speed rad/s", self.speed_box)

        self.kp_box = self._make_spinbox(0.0, 5.0, self.kp, 0.1)
        self.kp_box.valueChanged.connect(lambda value: setattr(self, "kp", float(value)))
        gains.addRow("kp", self.kp_box)

        self.kd_box = self._make_spinbox(0.0, 1.0, self.kd, 0.01)
        self.kd_box.valueChanged.connect(lambda value: setattr(self, "kd", float(value)))
        gains.addRow("kd", self.kd_box)
        root.addLayout(gains)

        buttons = QHBoxLayout()
        self.open_button = QPushButton("Open Hand", self)
        self.open_button.clicked.connect(self._open_hand)
        buttons.addWidget(self.open_button)

        self.stop_button = QPushButton("Zero Gains", self)
        self.stop_button.clicked.connect(self._zero_gains_once)
        buttons.addWidget(self.stop_button)
        root.addLayout(buttons)

        self.status_label = QLabel("", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        self.setLayout(root)
        self.setWindowTitle("Dex3 Joint Pose Slider")
        self.setMinimumWidth(520)

    @staticmethod
    def _make_spinbox(minimum: float, maximum: float, value: float, step: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setDecimals(3)
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def _set_controller(self, hand: str) -> None:
        try:
            self.controller = Dex3HandController(
                hand=hand,
                iface=self.iface,
                domain_id=self.domain_id,
            )
            self.status_label.setText(f"Connected: {hand} on {self.iface}")
        except Exception as exc:
            self.controller = None
            self.status_label.setText(f"Controller init failed: {exc}")

    def _on_hand_changed(self, hand: str) -> None:
        self.hand = str(hand)
        self.current_targets = hand_open_targets(self.hand)
        self.desired_targets = list(self.current_targets)
        self.matched_once = False
        self._set_controller(self.hand)
        self._sync_joint_controls()

    def _joint_bounds(self, joint_idx: int) -> tuple[float, float]:
        return (
            float(HAND_MIN_LIMITS[self.hand][joint_idx]),
            float(HAND_MAX_LIMITS[self.hand][joint_idx]),
        )

    def _sync_joint_controls(self) -> None:
        joint_idx = self.joint_combo.currentIndex()
        if joint_idx < 0:
            return

        lo, hi = self._joint_bounds(joint_idx)
        value = self.desired_targets[joint_idx]
        self._updating_controls = True
        try:
            self.slider.setMinimum(int(round(lo * SLIDER_SCALE)))
            self.slider.setMaximum(int(round(hi * SLIDER_SCALE)))
            self.slider.setValue(int(round(value * SLIDER_SCALE)))
            self.slider.setTickInterval(max(1, int(round((hi - lo) * SLIDER_SCALE / 10.0))))
        finally:
            self._updating_controls = False
        self._update_pose_label()

    def _on_slider_changed(self, raw_value: int) -> None:
        if self._updating_controls:
            return

        joint_idx = self.joint_combo.currentIndex()
        if joint_idx < 0:
            return

        lo, hi = self._joint_bounds(joint_idx)
        value = max(lo, min(hi, float(raw_value) / SLIDER_SCALE))
        self.desired_targets[joint_idx] = value
        self._update_pose_label()

    def _update_pose_label(self) -> None:
        joint_idx = self.joint_combo.currentIndex()
        if joint_idx < 0:
            return
        lo, hi = self._joint_bounds(joint_idx)
        self.pose_label.setText(
            f"{self.hand} {JOINT_NAMES[joint_idx]} | "
            f"current {self.current_targets[joint_idx]: .3f} rad | "
            f"desired {self.desired_targets[joint_idx]: .3f} rad | "
            f"limits [{lo: .3f}, {hi: .3f}]"
        )

    def _open_hand(self) -> None:
        self.desired_targets = hand_open_targets(self.hand)
        self._sync_joint_controls()

    def _zero_gains_once(self) -> None:
        if self.controller is None:
            return
        ok = self.controller.write_targets_once(
            self.current_targets,
            kp=0.0,
            kd=0.0,
            tau=0.0,
            timeout=1,
            first_write_timeout_s=1.0,
        )
        self.status_label.setText(f"Zero-gain stop sent: {ok}")

    def _publish_step(self) -> None:
        if self.controller is None:
            return

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, min(0.2, now - self.last_tick_s))
        self.last_tick_s = now
        max_delta = self.speed_rad_s * dt

        changed = False
        next_targets = list(self.current_targets)
        for idx, (current, desired) in enumerate(zip(self.current_targets, self.desired_targets)):
            error = desired - current
            if abs(error) <= max_delta:
                next_value = desired
            else:
                next_value = current + max_delta * (1.0 if error > 0.0 else -1.0)
            if abs(next_value - current) > 1e-6:
                changed = True
            next_targets[idx] = next_value

        self.current_targets = next_targets
        ok = self.controller.write_targets_once(
            self.current_targets,
            kp=self.kp,
            kd=self.kd,
            tau=self.tau,
            timeout=0,
            first_write_timeout_s=None if self.matched_once else 1.0,
        )
        self.matched_once = self.matched_once or ok

        if changed:
            self._update_pose_label()
        if not ok:
            self.status_label.setText("DDS command subscriber not matched")


def main() -> int:
    args, qt_argv = parse_args()
    app = QApplication(qt_argv)
    window = Dex3JointSliderApp(args)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
