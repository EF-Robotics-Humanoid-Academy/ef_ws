#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

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
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMessageBox,
        QPushButton,
        QTextEdit,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit("PyQt5 is required. Install it with: pip install PyQt5") from exc

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

from sdk_boot import hanger_boot_sequence


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
NOT_USED_IDX = 29

COMMAND_NAMES = ["forward", "backward", "left", "right", "up", "down"]
COMMAND_TO_INDEX = {name: idx for idx, name in enumerate(COMMAND_NAMES)}

KEY_TO_COMMAND = {
    Qt.Key_Up: "forward",
    Qt.Key_Down: "backward",
    Qt.Key_Left: "left",
    Qt.Key_Right: "right",
    Qt.Key_PageUp: "up",
    Qt.Key_PageDown: "down",
}


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


def selected_joint_indices(arm: str) -> list[int]:
    side = str(arm).strip().lower()
    if side == "left":
        return list(LEFT_ARM_IDX)
    if side == "right":
        return list(RIGHT_ARM_IDX)
    if side == "both":
        return list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX)
    raise ValueError(f"Unsupported arm selection '{arm}'.")


def command_feature(command_name: str) -> np.ndarray:
    feature = np.zeros(len(COMMAND_NAMES), dtype=np.float32)
    feature[COMMAND_TO_INDEX[command_name]] = 1.0
    return feature


def fit_ridge_regression(
    features: np.ndarray,
    targets: np.ndarray,
    ridge_lambda: float,
) -> tuple[np.ndarray, np.ndarray]:
    x = np.asarray(features, dtype=np.float64)
    y = np.asarray(targets, dtype=np.float64)
    x_aug = np.concatenate([x, np.ones((x.shape[0], 1), dtype=np.float64)], axis=1)
    reg = np.eye(x_aug.shape[1], dtype=np.float64) * max(0.0, float(ridge_lambda))
    reg[-1, -1] = 0.0
    lhs = x_aug.T @ x_aug + reg
    rhs = x_aug.T @ y
    try:
        theta = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        theta = np.linalg.pinv(lhs) @ rhs
    coef = theta[:-1, :]
    bias = theta[-1, :]
    return coef.astype(np.float32), bias.astype(np.float32)


def predict_pose(coef: np.ndarray, bias: np.ndarray, command_name: str) -> np.ndarray:
    x = command_feature(command_name).astype(np.float32)
    return x @ coef + bias


@dataclass
class Sample:
    t_s: float
    command_name: str
    command_index: int
    feature: np.ndarray
    joint_positions: np.ndarray


class ArmStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: list[float] | None = None
        self._timestamp = 0.0

        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        self._sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            positions = [float(msg.motor_state[j].q) for j in self.joints]
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[np.ndarray, float] | None:
        with self._lock:
            if self._positions is None:
                return None
            return np.asarray(self._positions, dtype=np.float32), float(self._timestamp)


class ArmTeachController:
    def __init__(self, joints: list[int], kp: float, kd: float) -> None:
        self.joints = [int(j) for j in joints]
        self.kp = float(kp)
        self.kd = float(kd)
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        self._last_pose = {joint: 0.0 for joint in self.joints}

    def write_zero_torque(self) -> None:
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
            mc.q = self._last_pose.get(joint, 0.0)
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def seed_pose(self, joint_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.joints, joint_positions):
            self._last_pose[joint] = float(q_val)

    def write_pose(self, joint_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.joints, joint_positions):
            mc = self._cmd.motor_cmd[joint]
            mc.q = float(q_val)
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
            self._last_pose[joint] = float(q_val)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def ramp_to_pose(self, target_positions: np.ndarray, duration_s: float, rate_hz: float = 50.0) -> None:
        target = np.asarray(target_positions, dtype=np.float32)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(rate_hz))))
        if steps <= 1:
            self.write_pose(target)
            return

        start = np.asarray([self._last_pose[joint] for joint in self.joints], dtype=np.float32)
        dt = 1.0 / max(1.0, float(rate_hz))
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            pose = start + (target - start) * alpha
            self.write_pose(pose)
            time.sleep(dt)


class RegressionArmMotionWindow(QWidget):
    def __init__(
        self,
        *,
        arm: str,
        sample_hz: float,
        ridge_lambda: float,
        output_dir: str,
        kp: float,
        kd: float,
    ) -> None:
        super().__init__()
        self.arm = str(arm)
        self.joints = selected_joint_indices(self.arm)
        self.sample_hz = max(1.0, float(sample_hz))
        self.ridge_lambda = max(0.0, float(ridge_lambda))
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.state_sub = ArmStateSubscriber(self.joints)
        self.arm_ctrl = ArmTeachController(self.joints, kp=kp, kd=kd)

        self.samples: list[Sample] = []
        self.command_counts = {name: 0 for name in COMMAND_NAMES}
        self.active_command: str | None = None
        self.teach_enabled = True
        self.model_coef: np.ndarray | None = None
        self.model_bias: np.ndarray | None = None
        self.model_rmse: float | None = None
        self.session_started_at = time.time()
        self.dataset_prefix = datetime.now().strftime("arm_regression_%Y%m%d_%H%M%S")
        self.command_buttons: dict[str, QPushButton] = {}

        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start(max(10, int(round(1000.0 / self.sample_hz))))
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()
        self._append_log(
            "Manual teaching active. Hold one command button or key while physically moving the selected arm joints."
        )

    def _build_ui(self) -> None:
        self.setWindowTitle("Regression Arm Motion Trainer")
        self.setMinimumWidth(900)

        root = QVBoxLayout()

        info = QLabel(
            "Controls: Up/Down/Left/Right/PageUp/PageDown. "
            "Hold one command while guiding the arm by hand. "
            "Only the selected arm joints are kept limp via rt/arm_sdk."
        )
        info.setWordWrap(True)
        root.addWidget(info)

        self.status_label = QLabel()
        root.addWidget(self.status_label)

        command_group = QGroupBox("Command Inputs")
        command_layout = QGridLayout()
        button_specs = [
            ("forward", "Forward\n[Up]", 0, 1),
            ("left", "Left\n[Left]", 1, 0),
            ("right", "Right\n[Right]", 1, 2),
            ("backward", "Backward\n[Down]", 2, 1),
            ("up", "Up\n[PgUp]", 0, 3),
            ("down", "Down\n[PgDn]", 2, 3),
        ]
        for command_name, label, row, col in button_specs:
            button = QPushButton(label)
            button.setCheckable(True)
            button.setMinimumHeight(70)
            button.pressed.connect(lambda name=command_name: self._set_active_command(name))
            button.released.connect(lambda name=command_name: self._clear_active_command(name))
            self.command_buttons[command_name] = button
            command_layout.addWidget(button, row, col)
        command_group.setLayout(command_layout)
        root.addWidget(command_group)

        controls_group = QGroupBox("Training")
        controls_layout = QHBoxLayout()

        self.teach_button = QPushButton("Pause Teaching")
        self.teach_button.clicked.connect(self._toggle_teaching)
        controls_layout.addWidget(self.teach_button)

        self.command_selector = QComboBox()
        self.command_selector.addItems(COMMAND_NAMES)
        controls_layout.addWidget(self.command_selector)

        self.ramp_spin = QDoubleSpinBox()
        self.ramp_spin.setRange(0.0, 5.0)
        self.ramp_spin.setSingleStep(0.1)
        self.ramp_spin.setValue(1.0)
        self.ramp_spin.setPrefix("Ramp ")
        self.ramp_spin.setSuffix(" s")
        controls_layout.addWidget(self.ramp_spin)

        self.train_button = QPushButton("Train Model")
        self.train_button.clicked.connect(self._train_model)
        controls_layout.addWidget(self.train_button)

        self.apply_button = QPushButton("Apply Prediction")
        self.apply_button.clicked.connect(self._apply_prediction)
        controls_layout.addWidget(self.apply_button)

        self.save_button = QPushButton("Save Dataset")
        self.save_button.clicked.connect(self._save_dataset_and_model)
        controls_layout.addWidget(self.save_button)

        controls_group.setLayout(controls_layout)
        root.addWidget(controls_group)

        self.summary_label = QLabel()
        self.summary_label.setWordWrap(True)
        root.addWidget(self.summary_label)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        root.addWidget(self.log_view)

        self.setLayout(root)
        self._refresh_summary()

    def _append_log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"[{stamp}] {text}")

    def _set_active_command(self, command_name: str) -> None:
        self.active_command = str(command_name)
        self._refresh_button_states()
        self._refresh_summary()

    def _clear_active_command(self, command_name: str) -> None:
        if self.active_command == str(command_name):
            self.active_command = None
        self._refresh_button_states()
        self._refresh_summary()

    def _refresh_button_states(self) -> None:
        for name, button in self.command_buttons.items():
            button.blockSignals(True)
            button.setChecked(name == self.active_command)
            button.blockSignals(False)

    def _toggle_teaching(self) -> None:
        self.teach_enabled = not self.teach_enabled
        self.teach_button.setText("Pause Teaching" if self.teach_enabled else "Resume Teaching")
        state = "enabled" if self.teach_enabled else "paused"
        self._append_log(f"Teaching {state}.")
        self._refresh_summary()

    def _refresh_summary(self) -> None:
        total_samples = len(self.samples)
        active = self.active_command or "none"
        counts = ", ".join(f"{name}:{self.command_counts[name]}" for name in COMMAND_NAMES)
        model_state = "not trained"
        if self.model_coef is not None and self.model_bias is not None:
            rmse_text = "n/a" if self.model_rmse is None else f"{self.model_rmse:.4f}"
            model_state = f"trained (rmse={rmse_text})"
        self.status_label.setText(
            f"Arm={self.arm}  joints={self.joints}  teaching={'on' if self.teach_enabled else 'off'}  "
            f"active_command={active}"
        )
        self.summary_label.setText(
            f"Samples: {total_samples} | Counts: {counts} | Model: {model_state} | Output dir: {self.output_dir}"
        )

    def _on_timer(self) -> None:
        if self.teach_enabled:
            self.arm_ctrl.write_zero_torque()

        snap = self.state_sub.snapshot()
        if snap is None:
            self._refresh_summary()
            return

        joint_positions, snap_ts = snap
        self.arm_ctrl.seed_pose(joint_positions)

        if self.teach_enabled and self.active_command is not None:
            command_name = self.active_command
            command_idx = COMMAND_TO_INDEX[command_name]
            sample = Sample(
                t_s=float(snap_ts - self.session_started_at),
                command_name=command_name,
                command_index=command_idx,
                feature=command_feature(command_name),
                joint_positions=joint_positions.copy(),
            )
            self.samples.append(sample)
            self.command_counts[command_name] += 1

        self._refresh_summary()

    def _build_dataset_arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self.samples:
            raise RuntimeError("No samples recorded yet.")
        ts = np.asarray([sample.t_s for sample in self.samples], dtype=np.float32)
        x = np.asarray([sample.feature for sample in self.samples], dtype=np.float32)
        y = np.asarray([sample.joint_positions for sample in self.samples], dtype=np.float32)
        cmd_idx = np.asarray([sample.command_index for sample in self.samples], dtype=np.int32)
        return ts, x, y, cmd_idx

    def _train_model(self) -> None:
        try:
            _, features, targets, _ = self._build_dataset_arrays()
        except RuntimeError as exc:
            QMessageBox.warning(self, "No Data", str(exc))
            return

        self.model_coef, self.model_bias = fit_ridge_regression(features, targets, self.ridge_lambda)
        predictions = features @ self.model_coef + self.model_bias
        self.model_rmse = float(np.sqrt(np.mean((predictions - targets) ** 2)))
        self._append_log(
            f"Model trained on {len(self.samples)} samples with ridge_lambda={self.ridge_lambda:.4f}. "
            f"Train RMSE={self.model_rmse:.6f}"
        )
        self._refresh_summary()
        self._save_dataset_and_model(show_message=False)

    def _apply_prediction(self) -> None:
        if self.model_coef is None or self.model_bias is None:
            QMessageBox.warning(self, "Model Missing", "Train the model before applying a prediction.")
            return

        command_name = self.command_selector.currentText()
        target = predict_pose(self.model_coef, self.model_bias, command_name)
        snap = self.state_sub.snapshot()
        if snap is not None:
            self.arm_ctrl.seed_pose(snap[0])

        self.teach_enabled = False
        self.teach_button.setText("Resume Teaching")
        self._append_log(f"Applying predicted pose for command '{command_name}'.")
        self.arm_ctrl.ramp_to_pose(target, duration_s=float(self.ramp_spin.value()))
        self._refresh_summary()

    def _save_dataset_and_model(self, show_message: bool = True) -> None:
        if not self.samples:
            if show_message:
                QMessageBox.warning(self, "No Data", "No samples recorded yet.")
            return

        ts, features, targets, cmd_idx = self._build_dataset_arrays()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = self.output_dir / f"{self.dataset_prefix}_{stamp}"
        dataset_npz = prefix.with_suffix(".npz")
        dataset_csv = prefix.with_suffix(".csv")
        model_npz = prefix.with_name(prefix.name + "_model").with_suffix(".npz")

        np.savez(
            dataset_npz,
            ts=ts,
            commands=np.asarray(COMMAND_NAMES, dtype="<U16"),
            command_indices=cmd_idx,
            features=features,
            joints=np.asarray(self.joints, dtype=np.int32),
            joint_positions=targets,
            arm=np.asarray([self.arm], dtype="<U16"),
        )

        with dataset_csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ["t_s", "command_name", "command_index"]
                + [f"input_{name}" for name in COMMAND_NAMES]
                + [f"joint_{joint}" for joint in self.joints]
            )
            for sample in self.samples:
                writer.writerow(
                    [f"{sample.t_s:.6f}", sample.command_name, sample.command_index]
                    + [f"{float(value):.6f}" for value in sample.feature]
                    + [f"{float(value):.6f}" for value in sample.joint_positions]
                )

        if self.model_coef is not None and self.model_bias is not None:
            np.savez(
                model_npz,
                commands=np.asarray(COMMAND_NAMES, dtype="<U16"),
                joints=np.asarray(self.joints, dtype=np.int32),
                coef=self.model_coef,
                bias=self.model_bias,
                ridge_lambda=np.asarray([self.ridge_lambda], dtype=np.float32),
                train_rmse=np.asarray(
                    [0.0 if self.model_rmse is None else self.model_rmse],
                    dtype=np.float32,
                ),
            )

        self._append_log(f"Saved dataset to {dataset_npz} and {dataset_csv}.")
        if self.model_coef is not None and self.model_bias is not None:
            self._append_log(f"Saved model to {model_npz}.")
        if show_message:
            QMessageBox.information(self, "Saved", f"Artifacts written under:\n{prefix.parent}")

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            event.ignore()
            return
        command_name = KEY_TO_COMMAND.get(event.key())
        if command_name is not None:
            self._set_active_command(command_name)
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            event.ignore()
            return
        command_name = KEY_TO_COMMAND.get(event.key())
        if command_name is not None:
            self._clear_active_command(command_name)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._timer.stop()
        self.arm_ctrl.write_zero_torque()
        super().closeEvent(event)


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Teach arm poses from six discrete input commands and fit a regression model."
    )
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both", help="which arm(s) to teach")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="recording rate while a command is held")
    parser.add_argument("--ridge-lambda", type=float, default=1e-3, help="ridge regularization strength")
    parser.add_argument("--kp", type=float, default=25.0, help="arm replay kp for predicted poses")
    parser.add_argument("--kd", type=float, default=1.0, help="arm replay kd for predicted poses")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(SCRIPT_DIR, "regression_arm_outputs"),
        help="where recorded datasets and model files are stored",
    )
    parser.add_argument(
        "--no-safety-boot",
        action="store_true",
        help="skip hanger boot sequence if the robot is already standing safely",
    )
    args, remaining = parser.parse_known_args()
    return args, [sys.argv[0], *remaining]


def main() -> int:
    args, qt_argv = parse_args()

    if not args.no_safety_boot:
        try:
            hanger_boot_sequence(iface=args.iface, domain_id=args.domain_id)
        except Exception as exc:
            print(f"Failed during safety boot: {exc}")
            return 1
    else:
        ChannelFactoryInitialize(int(args.domain_id), args.iface)

    app = QApplication(qt_argv)
    window = RegressionArmMotionWindow(
        arm=args.arm,
        sample_hz=args.sample_hz,
        ridge_lambda=args.ridge_lambda,
        output_dir=args.output_dir,
        kp=args.kp,
        kd=args.kd,
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
