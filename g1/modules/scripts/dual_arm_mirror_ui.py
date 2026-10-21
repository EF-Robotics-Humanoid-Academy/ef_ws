#!/usr/bin/env python3
from __future__ import annotations
from sdk_client import Robot
from dds_env import ensure_cyclonedds_environment

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


ensure_cyclonedds_environment()

try:
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (
        QAbstractItemView,
        QApplication,
        QDoubleSpinBox,
        QFormLayout,
        QHBoxLayout,
        QLabel,
        QComboBox,
        QLineEdit,
        QListWidget,
        QMessageBox,
        QPushButton,
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


SLIDER_SCALE = 1000
ARM_SDK_WEIGHT_INDEX = 29
WAIST_HOLD_KP = 480.0
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
DEFAULT_POSE_FILE = os.path.join(SCRIPT_DIR, "saved_dual_arm_mirror_poses.json")
INACTIVE_TRANSITION_KP = 300.0
TRANSITION_EPSILON_RAD = 1e-4

WAIST_JOINTS = [12, 13, 14]
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS


@dataclass(frozen=True)
class JointSpec:
    name: str
    left_index: int
    right_index: int
    left_min: float
    left_max: float
    right_min: float
    right_max: float
    right_sign: float

    @property
    def slider_min(self) -> float:
        if self.right_sign > 0.0:
            return max(self.left_min, self.right_min)
        mirrored_right_min = -self.right_max
        return max(self.left_min, mirrored_right_min)

    @property
    def slider_max(self) -> float:
        if self.right_sign > 0.0:
            return min(self.left_max, self.right_max)
        mirrored_right_max = -self.right_min
        return min(self.left_max, mirrored_right_max)


JOINT_SPECS = [
    JointSpec("shoulder_pitch", 15, 22, -3.0892, 2.6704, -3.0892, 2.6704, 1.0),
    JointSpec("shoulder_roll", 16, 23, -1.5882, 2.2515, -2.2515, 1.5882, -1.0),
    JointSpec("shoulder_yaw", 17, 24, -2.6180, 2.6180, -2.6180, 2.6180, -1.0),
    JointSpec("elbow", 18, 25, -1.0472, 2.0944, -1.0472, 2.0944, 1.0),
    JointSpec("wrist_roll", 19, 26, -1.9722, 1.9722, -1.9722, 1.9722, -1.0),
    JointSpec("wrist_pitch", 20, 27, -1.6144, 1.6144, -1.6144, 1.6144, 1.0),
    JointSpec("wrist_yaw", 21, 28, -1.6144, 1.6144, -1.6144, 1.6144, -1.0),
]
JOINT_SPEC_BY_NAME = {spec.name: spec for spec in JOINT_SPECS}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mirror both G1 arms from a single joint slider UI.")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--file", default=DEFAULT_POSE_FILE, help="Saved pose JSON file.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument("--speed-rad-s", type=float, default=0.1,
                        help="Initial max ramp increment in rad/s.")
    parser.add_argument("--kp", type=float, default=DEFAULT_ARM_KP, help="Arm proportional gain.")
    parser.add_argument("--kd", type=float, default=DEFAULT_ARM_KD, help="Arm derivative gain.")
    return parser.parse_args()


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


class UpperBodyStateSubscriber:
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


class UpperBodyPoseController:
    def __init__(self, *, iface: str, domain_id: int) -> None:
        ChannelFactoryInitialize(int(domain_id), str(iface))
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0

    def write_upper_body(
        self,
        targets: dict[int, float],
        *,
        arm_kp: float,
        arm_kd: float,
        waist_kp: float,
        waist_kd: float,
        joint_kp_overrides: dict[int, float] | None = None,
    ) -> None:
        kp_overrides = joint_kp_overrides or {}
        for joint_index in UPPER_BODY_JOINTS:
            cmd = self._cmd.motor_cmd[joint_index]
            cmd.mode = 1
            cmd.q = float(targets[joint_index])
            cmd.dq = 0.0
            cmd.tau = 0.0
            if joint_index in WAIST_JOINTS:
                cmd.kp = float(kp_overrides.get(joint_index, waist_kp))
                cmd.kd = float(waist_kd)
            else:
                cmd.kp = float(kp_overrides.get(joint_index, arm_kp))
                cmd.kd = float(arm_kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_zero_gains(self, hold_targets: dict[int, float]) -> None:
        for joint_index in UPPER_BODY_JOINTS:
            cmd = self._cmd.motor_cmd[joint_index]
            cmd.mode = 1
            cmd.q = float(hold_targets[joint_index])
            cmd.dq = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class DualArmMirrorWindow(QWidget):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.pose_path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.max_speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.arm_kp = float(args.kp)
        self.arm_kd = float(args.kd)
        self.waist_kp = float(WAIST_HOLD_KP)
        self.waist_kd = float(WAIST_HOLD_KD)

        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self.controller = UpperBodyPoseController(iface=self.iface, domain_id=self.domain_id)
        self.robot = Robot(iface=self.iface, domain_id=self.domain_id, auto_start_sensors=True)

        self.latest_positions = {joint: 0.0 for joint in UPPER_BODY_JOINTS}
        self.current_targets = dict(self.latest_positions)
        self.desired_targets = dict(self.latest_positions)
        self.saved_poses: list[dict[str, Any]] = []
        self.sequence_pose_indices: list[int] = []
        self.selected_joint_name = JOINT_SPECS[0].name
        self._updating_widgets = False
        self.last_tick_s = time.monotonic()
        self.seeded_from_state = False
        self.control_enabled = True
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.transition_joint_indices: set[int] = set()

        self._build_ui()
        self._load_saved_poses()
        self._seed_from_state_if_available()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(max(10, int(round(1000.0 / self.rate_hz))))

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        controls = QFormLayout()

        self.joint_combo = QComboBox(self)
        for spec in JOINT_SPECS:
            self.joint_combo.addItem(spec.name, spec.name)
        self.joint_combo.currentIndexChanged.connect(self._on_joint_selected)
        controls.addRow("Arm joint", self.joint_combo)

        self.speed_box = self._make_spinbox(0.01, 2.0, self.max_speed_rad_s, 0.01)
        self.speed_box.valueChanged.connect(self._on_speed_changed)
        controls.addRow("Ramp limit rad/s", self.speed_box)

        self.slider_value_box = self._make_spinbox(-3.14, 3.14, 0.0, 0.01)
        self.slider_value_box.valueChanged.connect(self._on_value_box_changed)
        controls.addRow("Target value", self.slider_value_box)

        root.addLayout(controls)

        slider_row = QHBoxLayout()
        self.slider_min_label = QLabel("", self)
        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.slider_max_label = QLabel("", self)
        slider_row.addWidget(self.slider_min_label)
        slider_row.addWidget(self.slider, 1)
        slider_row.addWidget(self.slider_max_label)
        root.addLayout(slider_row)

        self.mapping_label = QLabel("", self)
        self.mapping_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.mapping_label)

        self.current_label = QLabel("", self)
        self.current_label.setAlignment(Qt.AlignCenter)
        root.addWidget(self.current_label)

        buttons = QHBoxLayout()
        self.sync_button = QPushButton("Sync To Current", self)
        self.sync_button.clicked.connect(self._sync_desired_to_current)
        buttons.addWidget(self.sync_button)

        self.release_button = QPushButton("Release Arms", self)
        self.release_button.clicked.connect(self._release_arms)
        buttons.addWidget(self.release_button)

        self.reengage_button = QPushButton("Reengage Arms", self)
        self.reengage_button.clicked.connect(self._reengage_arms)
        buttons.addWidget(self.reengage_button)

        self.zero_button = QPushButton("Zero Gains Once", self)
        self.zero_button.clicked.connect(self._zero_gains_once)
        buttons.addWidget(self.zero_button)
        root.addLayout(buttons)

        pose_form = QFormLayout()
        self.pose_file_edit = QLineEdit(str(self.pose_path), self)
        self.pose_file_edit.editingFinished.connect(self._on_pose_file_changed)
        pose_form.addRow("Pose file", self.pose_file_edit)

        self.pose_name_edit = QLineEdit(self)
        self.pose_name_edit.setPlaceholderText("pose name")
        pose_form.addRow("Pose name", self.pose_name_edit)
        root.addLayout(pose_form)

        pose_buttons = QHBoxLayout()
        self.save_pose_button = QPushButton("Save Current Pose", self)
        self.save_pose_button.clicked.connect(self._save_current_pose)
        pose_buttons.addWidget(self.save_pose_button)

        self.load_pose_button = QPushButton("Load Selected Pose", self)
        self.load_pose_button.clicked.connect(self._load_selected_pose)
        pose_buttons.addWidget(self.load_pose_button)

        self.delete_pose_button = QPushButton("Remove Selected Pose", self)
        self.delete_pose_button.clicked.connect(self._remove_selected_poses)
        pose_buttons.addWidget(self.delete_pose_button)
        root.addLayout(pose_buttons)

        self.pose_list = QListWidget(self)
        self.pose_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.pose_list.itemDoubleClicked.connect(lambda _item: self._load_selected_pose())
        root.addWidget(self.pose_list)

        sequence_form = QFormLayout()
        self.sequence_gap_box = self._make_spinbox(0.0, 30.0, 2.0, 0.1)
        sequence_form.addRow("Sequence gap s", self.sequence_gap_box)
        root.addLayout(sequence_form)

        sequence_buttons = QHBoxLayout()
        self.sequence_add_button = QPushButton("Add Selected To Sequence", self)
        self.sequence_add_button.clicked.connect(self._add_selected_poses_to_sequence)
        sequence_buttons.addWidget(self.sequence_add_button)

        self.sequence_remove_button = QPushButton("Remove Sequence Step", self)
        self.sequence_remove_button.clicked.connect(self._remove_selected_sequence_steps)
        sequence_buttons.addWidget(self.sequence_remove_button)
        root.addLayout(sequence_buttons)

        sequence_order_buttons = QHBoxLayout()
        self.sequence_up_button = QPushButton("Move Step Up", self)
        self.sequence_up_button.clicked.connect(self._move_sequence_step_up)
        sequence_order_buttons.addWidget(self.sequence_up_button)

        self.sequence_down_button = QPushButton("Move Step Down", self)
        self.sequence_down_button.clicked.connect(self._move_sequence_step_down)
        sequence_order_buttons.addWidget(self.sequence_down_button)

        self.sequence_run_button = QPushButton("Run Sequence", self)
        self.sequence_run_button.clicked.connect(self._start_sequence)
        sequence_order_buttons.addWidget(self.sequence_run_button)

        self.sequence_stop_button = QPushButton("Stop Sequence", self)
        self.sequence_stop_button.clicked.connect(self._stop_sequence)
        sequence_order_buttons.addWidget(self.sequence_stop_button)
        root.addLayout(sequence_order_buttons)

        self.sequence_list = QListWidget(self)
        root.addWidget(self.sequence_list)

        self.status_label = QLabel("Waiting for rt/lowstate upper-body state...", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)

        self.setLayout(root)
        self.setWindowTitle("Dual Arm Mirror Joint UI")
        self.resize(920, 560)
        self._refresh_joint_selection_widgets()

    @staticmethod
    def _make_spinbox(minimum: float, maximum: float, value: float, step: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setDecimals(3)
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def _seed_from_state_if_available(self) -> None:
        deadline = time.monotonic() + 2.0
        snapshot = self.state_sub.snapshot()
        while snapshot is None and time.monotonic() < deadline:
            QApplication.processEvents()
            time.sleep(0.02)
            snapshot = self.state_sub.snapshot()
        if snapshot is None:
            return

        positions, _timestamp = snapshot
        self.seeded_from_state = True
        self.latest_positions = dict(positions)
        self.current_targets = dict(positions)
        self.desired_targets = dict(positions)
        self._refresh_joint_selection_widgets()
        self.status_label.setText(f"Connected on {self.iface}; waist pinned at current pose")

    def _selected_spec(self) -> JointSpec:
        return JOINT_SPEC_BY_NAME[self.selected_joint_name]

    def _selected_slider_value(self) -> float:
        spec = self._selected_spec()
        return float(self.desired_targets[spec.left_index])

    def _set_selected_slider_value(self, value: float) -> None:
        spec = self._selected_spec()
        clamped = max(spec.slider_min, min(spec.slider_max, float(value)))
        self.desired_targets[spec.left_index] = clamped
        self.desired_targets[spec.right_index] = clamped * spec.right_sign

    def _refresh_joint_selection_widgets(self) -> None:
        spec = self._selected_spec()
        slider_value = max(spec.slider_min, min(spec.slider_max, self._selected_slider_value()))
        self._set_selected_slider_value(slider_value)

        self._updating_widgets = True
        try:
            self.slider.setMinimum(int(round(spec.slider_min * SLIDER_SCALE)))
            self.slider.setMaximum(int(round(spec.slider_max * SLIDER_SCALE)))
            self.slider.setTickInterval(
                max(1, int(round((spec.slider_max - spec.slider_min) * SLIDER_SCALE / 10.0))))
            self.slider.setValue(int(round(slider_value * SLIDER_SCALE)))
            self.slider_value_box.setRange(spec.slider_min, spec.slider_max)
            self.slider_value_box.setValue(slider_value)
        finally:
            self._updating_widgets = False

        self.slider_min_label.setText(f"{spec.slider_min: .3f}")
        self.slider_max_label.setText(f"{spec.slider_max: .3f}")
        sign_text = "+" if spec.right_sign > 0.0 else "-"
        self.mapping_label.setText(
            f"Left {spec.name} = x, Right {spec.name} = {sign_text}x"
        )
        self._refresh_labels()

    def _refresh_labels(self) -> None:
        spec = self._selected_spec()
        left_current = float(self.latest_positions.get(
            spec.left_index, self.current_targets[spec.left_index]))
        right_current = float(self.latest_positions.get(
            spec.right_index, self.current_targets[spec.right_index]))
        left_target = float(self.desired_targets[spec.left_index])
        right_target = float(self.desired_targets[spec.right_index])
        self.current_label.setText(
            f"Current L/R: {left_current: .3f} / {right_current: .3f} rad    "
            f"Target L/R: {left_target: .3f} / {right_target: .3f} rad"
        )

    def _normalized_pose_name(self) -> str:
        name = self.pose_name_edit.text().strip()
        if not name:
            raise ValueError("Pose name is required. Auto-generated pose names are disabled.")
        return name

    def _pose_payload(self) -> dict[str, Any]:
        source = self.latest_positions if self.seeded_from_state else self.current_targets
        return {
            "name": self.pose_name_edit.text().strip(),
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {
                str(joint): float(source[joint])
                for joint in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
            },
            "waist_joints": {
                str(joint): float(source[joint])
                for joint in WAIST_JOINTS
            },
        }

    def _load_saved_poses(self) -> None:
        self.saved_poses = []
        if not self.pose_path.exists():
            self._refresh_pose_list()
            return
        try:
            payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.status_label.setText(f"Could not read pose file: {exc}")
            self._refresh_pose_list()
            return
        poses = payload.get("poses", [])
        if isinstance(poses, list):
            self.saved_poses = [
                pose
                for pose in poses
                if isinstance(pose, dict) and not self._is_generic_pose_name(pose.get("name"))
            ]
        self.sequence_pose_indices = [
            idx for idx in self.sequence_pose_indices
            if 0 <= idx < len(self.saved_poses)
        ]
        self._refresh_pose_list()
        self._refresh_sequence_list()

    def _write_saved_poses(self) -> None:
        payload = {"poses": self.saved_poses}
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(json.dumps(
            payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _refresh_pose_list(self) -> None:
        selected_row = self.pose_list.currentRow() if hasattr(self, "pose_list") else -1
        if not hasattr(self, "pose_list"):
            return
        self.pose_list.clear()
        for idx, pose in enumerate(self.saved_poses):
            name = str(pose.get("name", f"pose_{idx}"))
            saved_at = str(pose.get("saved_at", "?"))
            self.pose_list.addItem(f"{idx}: {name} | {saved_at}")
        if self.saved_poses:
            self.pose_list.setCurrentRow(min(max(selected_row, 0), len(self.saved_poses) - 1))

    @staticmethod
    def _is_generic_pose_name(name: Any) -> bool:
        if name is None:
            return True
        text = str(name).strip().lower()
        if not text:
            return True
        if not text.startswith("pose_"):
            return False
        suffix = text[5:]
        return suffix.isdigit()

    def _on_pose_file_changed(self) -> None:
        self.pose_path = Path(os.path.abspath(os.path.expanduser(
            self.pose_file_edit.text().strip() or str(self.pose_path))))
        self._load_saved_poses()
        self.status_label.setText(f"Pose file set to {self.pose_path}")

    def _save_current_pose(self) -> None:
        try:
            pose_name = self._normalized_pose_name()
        except ValueError as exc:
            QMessageBox.warning(self, "Pose Name Required", str(exc))
            return
        pose = self._pose_payload()
        pose["name"] = pose_name
        self.saved_poses.append(pose)
        self._write_saved_poses()
        self._refresh_pose_list()
        self.pose_name_edit.clear()
        self.status_label.setText(f"Saved pose '{pose['name']}' to {self.pose_path}")

    def _selected_pose(self) -> dict[str, Any] | None:
        row = self.pose_list.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            return None
        return self.saved_poses[row]

    def _apply_pose_to_desired_targets(self, pose: dict[str, Any]) -> None:
        previous_targets = dict(self.desired_targets)
        arm_joints = pose.get("arm_joints")
        waist_joints = pose.get("waist_joints")
        if not isinstance(arm_joints, dict):
            raise ValueError("Pose is missing arm_joints.")
        if isinstance(waist_joints, dict):
            for joint in WAIST_JOINTS:
                key = str(joint)
                if key in waist_joints:
                    self.desired_targets[joint] = float(waist_joints[key])
        for joint in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            key = str(joint)
            if key in arm_joints:
                self.desired_targets[joint] = float(arm_joints[key])
        self.transition_joint_indices = {
            joint for joint in UPPER_BODY_JOINTS
            if abs(float(self.desired_targets[joint]) - float(previous_targets[joint])) > TRANSITION_EPSILON_RAD
        }
        self._refresh_joint_selection_widgets()

    def _load_selected_pose(self) -> None:
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
        if self.control_enabled:
            self.status_label.setText(f"Loaded pose '{name}' into desired targets for live ramping")
        else:
            self.status_label.setText(
                f"Loaded pose '{name}' into desired targets; publishing is still disabled")

    def _selected_pose_rows(self) -> list[int]:
        rows = sorted({index.row() for index in self.pose_list.selectedIndexes()})
        return [row for row in rows if 0 <= row < len(self.saved_poses)]

    def _remove_selected_poses(self) -> None:
        rows = self._selected_pose_rows()
        if not rows:
            QMessageBox.warning(self, "No Pose", "Select one or more saved poses first.")
            return

        removed_names = [str(self.saved_poses[row].get("name", f"pose_{row}")) for row in rows]
        removed_set = set(rows)

        new_sequence_pose_indices: list[int] = []
        for pose_index in self.sequence_pose_indices:
            if pose_index in removed_set:
                continue
            shift = sum(1 for row in rows if row < pose_index)
            new_sequence_pose_indices.append(pose_index - shift)

        for row in reversed(rows):
            del self.saved_poses[row]

        self.sequence_pose_indices = new_sequence_pose_indices
        if self.sequence_running:
            self._stop_sequence()
        self._write_saved_poses()
        self._refresh_pose_list()
        self._refresh_sequence_list()

        if len(removed_names) == 1:
            self.status_label.setText(f"Removed pose '{removed_names[0]}' from {self.pose_path}")
        else:
            self.status_label.setText(f"Removed {len(removed_names)} poses from {self.pose_path}")

    def _refresh_sequence_list(self) -> None:
        selected_row = self.sequence_list.currentRow() if hasattr(self, "sequence_list") else -1
        if not hasattr(self, "sequence_list"):
            return
        self.sequence_list.clear()
        for order, pose_index in enumerate(self.sequence_pose_indices, start=1):
            if pose_index < 0 or pose_index >= len(self.saved_poses):
                continue
            pose = self.saved_poses[pose_index]
            name = str(pose.get("name", f"pose_{pose_index}"))
            self.sequence_list.addItem(f"{order}: {name}")
        if self.sequence_pose_indices:
            self.sequence_list.setCurrentRow(
                min(max(selected_row, 0), len(self.sequence_pose_indices) - 1))

    def _add_selected_poses_to_sequence(self) -> None:
        rows = self._selected_pose_rows()
        if not rows:
            QMessageBox.warning(self, "No Pose", "Select one or more saved poses first.")
            return
        self.sequence_pose_indices.extend(rows)
        self._refresh_sequence_list()
        self.status_label.setText(f"Added {len(rows)} pose(s) to the sequence")

    def _remove_selected_sequence_steps(self) -> None:
        rows = sorted({index.row() for index in self.sequence_list.selectedIndexes()}, reverse=True)
        if not rows:
            QMessageBox.warning(self, "No Sequence Step",
                                "Select one or more sequence steps first.")
            return
        for row in rows:
            if 0 <= row < len(self.sequence_pose_indices):
                del self.sequence_pose_indices[row]
        self._refresh_sequence_list()
        self.status_label.setText("Removed selected sequence step(s)")

    def _move_sequence_step_up(self) -> None:
        row = self.sequence_list.currentRow()
        if row <= 0 or row >= len(self.sequence_pose_indices):
            return
        self.sequence_pose_indices[row - 1], self.sequence_pose_indices[row] = (
            self.sequence_pose_indices[row],
            self.sequence_pose_indices[row - 1],
        )
        self._refresh_sequence_list()
        self.sequence_list.setCurrentRow(row - 1)

    def _move_sequence_step_down(self) -> None:
        row = self.sequence_list.currentRow()
        if row < 0 or row >= len(self.sequence_pose_indices) - 1:
            return
        self.sequence_pose_indices[row + 1], self.sequence_pose_indices[row] = (
            self.sequence_pose_indices[row],
            self.sequence_pose_indices[row + 1],
        )
        self._refresh_sequence_list()
        self.sequence_list.setCurrentRow(row + 1)

    def _start_sequence(self) -> None:
        if not self.control_enabled:
            QMessageBox.warning(self, "Slider Control Disabled",
                                "Reengage arms before running a sequence.")
            return
        if not self.sequence_pose_indices:
            QMessageBox.warning(self, "Empty Sequence",
                                "Add one or more poses to the sequence first.")
            return
        self.sequence_running = True
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.status_label.setText("Sequence started")

    def _stop_sequence(self) -> None:
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.status_label.setText("Sequence stopped")

    def _advance_sequence_if_due(self, now_s: float) -> None:
        if not self.sequence_running:
            return
        if self.sequence_step_index >= len(self.sequence_pose_indices):
            self.sequence_running = False
            self.status_label.setText("Sequence completed")
            return
        if self.sequence_next_time_s < 0.0:
            if self.transition_joint_indices:
                return
            self.sequence_next_time_s = now_s + max(0.0, float(self.sequence_gap_box.value()))
            return
        if self.sequence_next_time_s > 0.0 and now_s < self.sequence_next_time_s:
            return
        pose_index = self.sequence_pose_indices[self.sequence_step_index]
        if pose_index < 0 or pose_index >= len(self.saved_poses):
            self.sequence_running = False
            self.status_label.setText("Sequence stopped due to missing pose entry")
            return
        pose = self.saved_poses[pose_index]
        self._apply_pose_to_desired_targets(pose)
        self.sequence_list.setCurrentRow(self.sequence_step_index)
        self.sequence_step_index += 1
        self.sequence_next_time_s = -1.0 if self.transition_joint_indices else now_s + \
            max(0.0, float(self.sequence_gap_box.value()))
        name = str(pose.get("name", "<unnamed>"))
        self.status_label.setText(
            f"Sequence step {self.sequence_step_index}/{len(self.sequence_pose_indices)}: {name}")

    def _on_joint_selected(self, index: int) -> None:
        name = self.joint_combo.itemData(index)
        if not name:
            return
        self.selected_joint_name = str(name)
        self._refresh_joint_selection_widgets()

    def _on_speed_changed(self, value: float) -> None:
        self.max_speed_rad_s = max(0.01, float(value))

    def _on_slider_changed(self, raw_value: int) -> None:
        if self._updating_widgets:
            return
        self._set_selected_slider_value(float(raw_value) / SLIDER_SCALE)
        self._refresh_joint_selection_widgets()

    def _on_value_box_changed(self, value: float) -> None:
        if self._updating_widgets:
            return
        self._set_selected_slider_value(float(value))
        self._refresh_joint_selection_widgets()

    def _sync_desired_to_current(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, _timestamp = snapshot
            self.latest_positions = dict(positions)
        self.current_targets = dict(self.latest_positions)
        self.desired_targets = dict(self.latest_positions)
        self.transition_joint_indices.clear()
        self._refresh_joint_selection_widgets()
        self.status_label.setText("Desired targets synced to current upper-body pose")

    def _release_arms(self) -> None:
        try:
            if not self.robot.wait_for_low_state(timeout=2.0):
                raise TimeoutError("Robot helper did not receive rt/lowstate in time.")
            self.robot.release_arms()
        except Exception as exc:
            self.status_label.setText(f"Release arms failed: {exc}")
            return
        self.control_enabled = False
        self.status_label.setText(
            "Arms released through sdk_client.Robot.release_arms(); slider publishing disabled")

    def _reengage_arms(self) -> None:
        try:
            if not self.robot.wait_for_low_state(timeout=2.0):
                raise TimeoutError("Robot helper did not receive rt/lowstate in time.")
            self.robot.unrelease_arms()
        except Exception as exc:
            self.status_label.setText(f"Reengage arms failed: {exc}")
            return
        self.control_enabled = True
        self._sync_desired_to_current()
        self.status_label.setText(
            "Arms reengaged through sdk_client.Robot.unrelease_arms(); publishing resumed from live pose")

    def _zero_gains_once(self) -> None:
        self.controller.write_zero_gains(self.current_targets)
        self.status_label.setText("Zero-gain hold sent on rt/arm_sdk")

    def _step_toward_targets(self, dt: float) -> None:
        max_step = max(1e-6, self.max_speed_rad_s * dt)
        for joint_index in UPPER_BODY_JOINTS:
            current = float(self.current_targets[joint_index])
            desired = float(self.desired_targets[joint_index])
            delta = desired - current
            if abs(delta) <= max_step:
                self.current_targets[joint_index] = desired
            else:
                self.current_targets[joint_index] = current + \
                    max_step * (1.0 if delta > 0.0 else -1.0)

        if self.transition_joint_indices and all(
            abs(float(self.current_targets[joint]) -
                float(self.desired_targets[joint])) <= TRANSITION_EPSILON_RAD
            for joint in self.transition_joint_indices
        ):
            self.transition_joint_indices.clear()

    def _inactive_transition_kp_overrides(self) -> dict[int, float]:
        if not self.transition_joint_indices:
            return {}
        return {
            joint: max(INACTIVE_TRANSITION_KP,
                       self.waist_kp if joint in WAIST_JOINTS else self.arm_kp)
            for joint in UPPER_BODY_JOINTS
            if joint not in self.transition_joint_indices
        }

    def _tick(self) -> None:
        snapshot = self.state_sub.snapshot()
        if snapshot is not None:
            positions, _timestamp = snapshot
            self.latest_positions = positions
            if not self.seeded_from_state:
                self.seeded_from_state = True
                self.current_targets = dict(positions)
                self.desired_targets = dict(positions)
                self._refresh_joint_selection_widgets()

        if not self.seeded_from_state:
            self.status_label.setText("Waiting for rt/lowstate upper-body state...")
            return

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, now - self.last_tick_s)
        self.last_tick_s = now

        if not self.control_enabled:
            self._refresh_labels()
            return

        self._advance_sequence_if_due(now)
        self._step_toward_targets(dt)
        self.controller.write_upper_body(
            self.current_targets,
            arm_kp=self.arm_kp,
            arm_kd=self.arm_kd,
            waist_kp=self.waist_kp,
            waist_kd=self.waist_kd,
            joint_kp_overrides=self._inactive_transition_kp_overrides(),
        )
        self._refresh_labels()
        self.status_label.setText(
            f"Publishing {self.rate_hz: .1f} Hz, ramp limit {self.max_speed_rad_s: .2f} rad/s, waist pinned"
        )


def main() -> None:
    args = parse_args()
    app = QApplication(sys.argv)
    window = DualArmMirrorWindow(args)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
