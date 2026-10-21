#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

try:
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtWidgets import (
        QApplication,
        QFileDialog,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
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

try:
    from sdk_boot import hanger_boot_sequence
except ImportError:
    hanger_boot_sequence = None


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_IDX = [12, 13, 14]
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
DEFAULT_OUTPUT = os.path.join(SCRIPT_DIR, "saved_arm_poses.json")


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="PyQt arm pose capture tool with waist lock and zero-torque teaching."
    )
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument("--waist-kp", type=float, default=30.0, help="Waist lock proportional gain.")
    parser.add_argument("--waist-kd", type=float, default=1.5, help="Waist lock derivative gain.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="JSON file used to store captured poses.")
    parser.add_argument(
        "--run-hanged-boot",
        action="store_true",
        help="Run the hanger boot sequence before starting the GUI.",
    )
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


class RobotStateSubscriber:
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


class TeachController:
    def __init__(
        self,
        arm_joints: list[int],
        waist_joints: list[int],
        *,
        waist_kp: float,
        waist_kd: float,
    ) -> None:
        self.arm_joints = [int(j) for j in arm_joints]
        self.waist_joints = [int(j) for j in waist_joints]
        self.waist_kp = float(waist_kp)
        self.waist_kd = float(waist_kd)
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0

        self._waist_hold = {joint: 0.0 for joint in self.waist_joints}
        self._arm_hold = {joint: 0.0 for joint in self.arm_joints}
        for joint in self.arm_joints + self.waist_joints:
            self._cmd.motor_cmd[joint].mode = 1

    def seed_from_positions(self, positions: dict[int, float]) -> None:
        for joint in self.arm_joints:
            if joint in positions:
                self._arm_hold[joint] = float(positions[joint])
        for joint in self.waist_joints:
            if joint in positions:
                self._waist_hold[joint] = float(positions[joint])

    def update_arm_hold(self, positions: dict[int, float]) -> None:
        for joint in self.arm_joints:
            if joint in positions:
                self._arm_hold[joint] = float(positions[joint])

    def write_teach_mode(self) -> None:
        for joint in self.arm_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._arm_hold[joint]
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0

        for joint in self.waist_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._waist_hold[joint]
            mc.dq = 0.0
            mc.kp = self.waist_kp
            mc.kd = self.waist_kd
            mc.tau = 0.0

        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class ArmPoseCaptureApp(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.arm = str(args.arm)
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.output_path = Path(os.path.abspath(os.path.expanduser(str(args.output))))

        self.arm_to_joints = selected_joint_map(self.arm)
        self.arm_joints = [joint for joints in self.arm_to_joints.values() for joint in joints]
        self.state_joints = self.arm_joints + list(WAIST_IDX)

        self.latest_positions = {joint: 0.0 for joint in self.state_joints}
        self.latest_state_time = 0.0
        self.teach_enabled = False
        self.pose_count = 0

        self.state_sub = RobotStateSubscriber(self.state_joints)
        self.controller = TeachController(
            self.arm_joints,
            list(WAIST_IDX),
            waist_kp=float(args.waist_kp),
            waist_kd=float(args.waist_kd),
        )

        self._build_ui()
        self._load_pose_count()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(max(10, int(round(1000.0 / self.rate_hz))))

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        instructions = QLabel(
            "1. Wait for state.\n"
            "2. Click Enable Teach to lock the waist and release the selected arm joints.\n"
            "3. Move the arm by hand.\n"
            "4. Enter a pose name and click Save Pose.",
            self,
        )
        instructions.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        root.addWidget(instructions)

        form = QFormLayout()
        self.output_edit = QLineEdit(str(self.output_path), self)
        form.addRow("Output file", self.output_edit)
        root.addLayout(form)

        button_row = QHBoxLayout()
        self.browse_button = QPushButton("Browse", self)
        self.browse_button.clicked.connect(self._browse_output)
        button_row.addWidget(self.browse_button)

        self.enable_button = QPushButton("Enable Teach", self)
        self.enable_button.clicked.connect(self._enable_teach)
        button_row.addWidget(self.enable_button)

        self.disable_button = QPushButton("Disable Teach", self)
        self.disable_button.clicked.connect(self._disable_teach)
        button_row.addWidget(self.disable_button)
        root.addLayout(button_row)

        save_row = QHBoxLayout()
        self.pose_name_edit = QLineEdit(self)
        self.pose_name_edit.setPlaceholderText("pose name")
        save_row.addWidget(self.pose_name_edit)

        self.save_button = QPushButton("Save Pose", self)
        self.save_button.clicked.connect(self._save_pose)
        save_row.addWidget(self.save_button)
        root.addLayout(save_row)

        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Current (rad)"), 0, 1)

        self.joint_labels: dict[int, QLabel] = {}
        row = 1
        for arm_name, joints in self.arm_to_joints.items():
            box = QGroupBox(f"{arm_name.title()} Arm")
            box_grid = QGridLayout()
            box_grid.addWidget(QLabel("Joint"), 0, 0)
            box_grid.addWidget(QLabel("Current (rad)"), 0, 1)
            for idx, joint in enumerate(joints):
                box_grid.addWidget(QLabel(f"{JOINT_NAMES[idx]} ({joint})"), idx + 1, 0)
                value_label = QLabel("--", self)
                box_grid.addWidget(value_label, idx + 1, 1)
                self.joint_labels[joint] = value_label
            box.setLayout(box_grid)
            root.addWidget(box)
            row += len(joints)

        waist_box = QGroupBox("Waist Lock")
        waist_grid = QGridLayout()
        waist_grid.addWidget(QLabel("Joint"), 0, 0)
        waist_grid.addWidget(QLabel("Current (rad)"), 0, 1)
        waist_names = ["yaw", "roll", "pitch"]
        for idx, joint in enumerate(WAIST_IDX):
            waist_grid.addWidget(QLabel(f"{waist_names[idx]} ({joint})"), idx + 1, 0)
            value_label = QLabel("--", self)
            waist_grid.addWidget(value_label, idx + 1, 1)
            self.joint_labels[joint] = value_label
        waist_box.setLayout(waist_grid)
        root.addWidget(waist_box)

        self.status_label = QLabel("Waiting for rt/lowstate...", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.status_label)

        self.log_box = QPlainTextEdit(self)
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(200)
        root.addWidget(self.log_box)

        self.setLayout(root)
        self.setWindowTitle(f"Arm Pose Capture ({self.arm})")
        self.resize(720, 760)

    def _browse_output(self) -> None:
        selected, _ = QFileDialog.getSaveFileName(
            self,
            "Select pose output file",
            str(self.output_edit.text().strip() or self.output_path),
            "JSON Files (*.json);;All Files (*)",
        )
        if not selected:
            return
        self.output_path = Path(selected)
        self.output_edit.setText(str(self.output_path))
        self._load_pose_count()

    def _append_log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_box.appendPlainText(f"[{stamp}] {message}")

    def _load_pose_count(self) -> None:
        self.output_path = Path(os.path.abspath(os.path.expanduser(self.output_edit.text().strip() or str(self.output_path))))
        if not self.output_path.exists():
            self.pose_count = 0
            return
        try:
            payload = json.loads(self.output_path.read_text(encoding="utf-8"))
            poses = payload.get("poses", [])
            self.pose_count = len(poses) if isinstance(poses, list) else 0
        except Exception:
            self.pose_count = 0

    def _enable_teach(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is None:
            QMessageBox.warning(self, "No State", "No rt/lowstate data yet.")
            return
        positions, _timestamp = snapshot
        self.latest_positions = positions
        self.controller.seed_from_positions(positions)
        self.teach_enabled = True
        self._append_log("Teach enabled: waist locked, selected arm joints set to zero torque.")

    def _disable_teach(self) -> None:
        self.teach_enabled = False
        self._append_log("Teach disabled.")

    def _save_pose(self) -> None:
        if not self.teach_enabled:
            QMessageBox.warning(self, "Teach Disabled", "Enable teach mode before saving a pose.")
            return

        snapshot = self.state_sub.snapshot()
        if snapshot is None:
            QMessageBox.warning(self, "No State", "No rt/lowstate data available.")
            return

        pose_name = self.pose_name_edit.text().strip()
        if not pose_name:
            pose_name = f"pose_{self.pose_count + 1:03d}"

        positions, timestamp = snapshot
        self.latest_positions = positions
        state_age_s = max(0.0, time.time() - float(timestamp))

        pose_entry = {
            "name": pose_name,
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "state_timestamp_s": float(timestamp),
            "state_age_s": state_age_s,
            "arm_selection": self.arm,
            "arm_joints": {
                str(joint): float(positions[joint]) for joint in self.arm_joints
            },
            "waist_joints": {
                str(joint): float(positions[joint]) for joint in WAIST_IDX
            },
            "arms": {
                arm_name: {
                    JOINT_NAMES[idx]: float(positions[joint]) for idx, joint in enumerate(joints)
                }
                for arm_name, joints in self.arm_to_joints.items()
            },
        }

        payload = {"arm_selection": self.arm, "poses": []}
        if self.output_path.exists():
            try:
                payload = json.loads(self.output_path.read_text(encoding="utf-8"))
            except Exception:
                reply = QMessageBox.question(
                    self,
                    "Overwrite File",
                    f"{self.output_path} is not valid JSON. Overwrite it with a new pose file?",
                )
                if reply != QMessageBox.Yes:
                    return
                payload = {"arm_selection": self.arm, "poses": []}

        poses = payload.get("poses")
        if not isinstance(poses, list):
            poses = []
            payload["poses"] = poses
        payload["arm_selection"] = self.arm
        duplicate_count = sum(1 for existing_pose in poses if str(existing_pose.get("name", "")) == pose_name)
        poses.append(pose_entry)

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        self.pose_count = len(poses)
        self.pose_name_edit.clear()
        if duplicate_count > 0:
            self._append_log(
                f"Pose name '{pose_name}' already existed {duplicate_count} time(s); appended a new entry."
            )
        arm_summary = ", ".join(
            f"{joint}={float(positions[joint]):+.4f}" for joint in self.arm_joints
        )
        waist_summary = ", ".join(
            f"{joint}={float(positions[joint]):+.4f}" for joint in WAIST_IDX
        )
        self._append_log(
            f"Saved pose '{pose_name}' to {self.output_path} | state age {state_age_s:.3f}s"
        )
        self._append_log(f"Saved arm joints: {arm_summary}")
        self._append_log(f"Saved waist joints: {waist_summary}")

    def _refresh_joint_labels(self) -> None:
        for joint, label in self.joint_labels.items():
            label.setText(f"{self.latest_positions.get(joint, 0.0): .4f}")

    def _tick(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, timestamp = snapshot
            self.latest_positions = positions
            self.latest_state_time = timestamp
            self.controller.update_arm_hold(positions)
            self._refresh_joint_labels()

        if self.teach_enabled:
            self.controller.write_teach_mode()

        if self.latest_state_time > 0.0:
            age_s = max(0.0, time.time() - self.latest_state_time)
            mode = "teach enabled" if self.teach_enabled else "teach disabled"
            self.status_label.setText(
                f"{mode} | arm={self.arm} | poses={self.pose_count} | state age {age_s:.2f}s"
            )
        else:
            self.status_label.setText("Waiting for rt/lowstate...")


def main() -> int:
    args, qt_argv = parse_args()

    if args.run_hanged_boot:
        if hanger_boot_sequence is None:
            raise SystemExit("sdk_boot.py not available, cannot run hanger boot sequence.")
        hanger_boot_sequence(iface=args.iface, domain_id=args.domain_id)

    ChannelFactoryInitialize(int(args.domain_id), str(args.iface))
    app = QApplication(qt_argv)
    window = ArmPoseCaptureApp(args)
    window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
