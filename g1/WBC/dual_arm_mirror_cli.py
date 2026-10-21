#!/usr/bin/env python3
"""
Terminal (curses) equivalent of dual_arm_mirror_ui_with_waist.py.

All features of the PyQt5 GUI are replicated:
  - Joint picker with mirror mapping (arms) and single-axis (waist)
  - Live value bar + current / target readout
  - Speed ramp limit and per-key step size
  - Save / load / delete named poses (same JSON format)
  - Sequence builder: add poses, reorder, toggle waist, set gap, run/stop
  - Sync-to-current, release/reengage arms, zero-gains-once

Key bindings (also shown in footer)
────────────────────────────────────
  ↑ / ↓  or  j / k     change selected joint
  ← / →  or  - / +     adjust target by step
  <  /  >               halve / double step size
  s                     set ramp speed (prompt)
  y                     sync targets → live pose
  r                     release arms
  e                     reengage arms
  z                     zero-gain send once
  m                     cycle arm control: both → left → right
  Tab                   cycle focus: joint → poses → sequence
  (poses focus)
    p                   save pose (name prompt)
    l / Enter           load selected pose
    d                   delete selected pose
    a                   add selected pose to sequence
  (sequence focus)
    x / Delete          remove selected step
    u                   move step up
    n                   move step down
    w                   toggle include-waist for added steps
    g                   set sequence gap (prompt)
    R                   run sequence
    S                   stop sequence
  q / Esc               quit
"""

from __future__ import annotations

import argparse
import curses
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
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
MODULES_DIR = os.path.join(ROOT_DIR, "modules")
for _p in (ROOT_DIR, MODULES_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)


from dds_env import ensure_cyclonedds_environment
from sdk_client import Robot


ensure_cyclonedds_environment()

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
        "unitree_sdk2py not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


# ── Constants (identical to the GUI source) ───────────────────────────────────
SLIDER_SCALE = 1000
ARM_SDK_WEIGHT_INDEX = 29
WAIST_HOLD_KP = 480.0
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
INACTIVE_TRANSITION_KP = 300.0
TRANSITION_EPSILON_RAD = 1e-4

WAIST_JOINTS = [12, 13, 14]
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_CONTROL_MODES = ("both", "left", "right")

WAIST_JOINT_NAMES = {12: "waist_yaw", 13: "waist_roll", 14: "waist_pitch"}

DEFAULT_POSE_FILE = os.path.join(
    os.path.dirname(os.path.abspath(
        os.path.join(MODULES_DIR, "scripts", "saved_dual_arm_mirror_poses.json")
    )),
    "scripts", "saved_dual_arm_mirror_poses.json",
)

# ── Joint data structures (identical to GUI) ──────────────────────────────────


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


JOINT_SPECS = [
    JointSpec("shoulder_pitch", 15, 22, -3.0892, 2.6704, -3.0892, 2.6704, 1.0),
    JointSpec("shoulder_roll", 16, 23, -1.5882, 2.2515, -2.2515, 1.5882, -1.0),
    JointSpec("shoulder_yaw", 17, 24, -2.6180, 2.6180, -2.6180, 2.6180, -1.0),
    JointSpec("elbow", 18, 25, -1.0472, 2.0944, -1.0472, 2.0944, 1.0),
    JointSpec("wrist_roll", 19, 26, -1.9722, 1.9722, -1.9722, 1.9722, -1.0),
    JointSpec("wrist_pitch", 20, 27, -1.6144, 1.6144, -1.6144, 1.6144, 1.0),
    JointSpec("wrist_yaw", 21, 28, -1.6144, 1.6144, -1.6144, 1.6144, -1.0),
]


@dataclass(frozen=True)
class JointSelection:
    name: str
    label: str
    joint_type: str
    left_index: int
    right_index: int | None
    left_min: float
    left_max: float
    right_min: float | None
    right_max: float | None
    right_sign: float

    @property
    def slider_min(self) -> float:
        if self.right_index is None:
            return self.left_min
        if self.right_sign > 0.0:
            return max(self.left_min, float(self.right_min))
        return max(self.left_min, -float(self.right_max))

    @property
    def slider_max(self) -> float:
        if self.right_index is None:
            return self.left_max
        if self.right_sign > 0.0:
            return min(self.left_max, float(self.right_max))
        return min(self.left_max, -float(self.right_min))


JOINT_SELECTIONS: list[JointSelection] = [
    JointSelection(
        name=WAIST_JOINT_NAMES[j],
        label=f"waist: {WAIST_JOINT_NAMES[j]}",
        joint_type="waist",
        left_index=j, right_index=None,
        left_min=-2.5, left_max=2.5,
        right_min=None, right_max=None, right_sign=1.0,
    )
    for j in WAIST_JOINTS
] + [
    JointSelection(
        name=spec.name, label=f"arms: {spec.name}",
        joint_type="arm",
        left_index=spec.left_index, right_index=spec.right_index,
        left_min=spec.left_min, left_max=spec.left_max,
        right_min=spec.right_min, right_max=spec.right_max,
        right_sign=spec.right_sign,
    )
    for spec in JOINT_SPECS
]
JOINT_SELECTION_BY_NAME = {s.name: s for s in JOINT_SELECTIONS}

# ── Robot helpers (identical to GUI) ─────────────────────────────────────────


def _resolve_lowstate_type():
    for path in ("unitree_sdk2py.idl.unitree_hg.msg.dds_",
                 "unitree_sdk2py.idl.unitree_go.msg.dds_"):
        try:
            mod = __import__(path, fromlist=["LowState_"])
            if hasattr(mod, "LowState_"):
                return getattr(mod, "LowState_")
        except Exception:
            pass
    return None


class UpperBodyStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: dict[int, float] = {}
        self._timestamp = 0.0
        t = _resolve_lowstate_type()
        if t is None:
            raise RuntimeError("LowState_ not found in unitree_sdk2py.")
        self._sub = ChannelSubscriber("rt/lowstate", t)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            pos = {j: float(msg.motor_state[j].q) for j in self.joints}
        except Exception:
            return
        with self._lock:
            self._positions = pos
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

    def write_upper_body(self, targets: dict[int, float], *, arm_kp, arm_kd,
                         waist_kp, waist_kd, joint_kp_overrides=None) -> None:
        ov = joint_kp_overrides or {}
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(targets[j])
            c.dq = 0.0
            c.tau = 0.0
            if j in WAIST_JOINTS:
                c.kp = float(ov.get(j, waist_kp))
                c.kd = float(waist_kd)
            else:
                c.kp = float(ov.get(j, arm_kp))
                c.kd = float(arm_kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_zero_gains(self, hold: dict[int, float]) -> None:
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(hold[j])
            c.dq = 0.0
            c.kp = 0.0
            c.kd = 0.0
            c.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


# ── Focus constants ───────────────────────────────────────────────────────────
FOCUS_JOINT = 0
FOCUS_POSES = 1
FOCUS_SEQUENCE = 2

# ── Colour pair indices ───────────────────────────────────────────────────────
C_NORMAL = 0
C_GREEN = 1   # connected / active
C_YELLOW = 2   # warning / step size
C_RED = 3   # released / error
C_CYAN = 4   # title / header
C_SEL = 5   # selected list row  (black on white)
C_FOCUS = 6   # focused panel header (black on cyan)
C_RUNNING = 7   # sequence running (white on blue)


# ── Main application ──────────────────────────────────────────────────────────
class DualArmMirrorCLI:
    def __init__(self, args: argparse.Namespace) -> None:
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.pose_path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.max_speed = max(0.01, float(args.speed_rad_s))
        self.arm_kp = float(args.kp)
        self.arm_kd = float(args.kd)
        self.waist_kp = float(WAIST_HOLD_KP)
        self.waist_kd = float(WAIST_HOLD_KD)
        self.arm_control_mode = str(args.arm_control)

        # Joint selection
        self.joint_idx = 0          # index into JOINT_SELECTIONS
        self.adjust_step = 0.05       # rad per key-press

        # Robot state mirrors (same names as GUI)
        self.latest_positions: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.seeded_from_state = False
        self.control_enabled = True
        self.transition_joint_indices: set[int] = set()

        # Sequence state
        self.saved_poses: list[dict[str, Any]] = []
        self.sequence_steps: list[dict[str, Any]] = []
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.sequence_gap_s = 2.0
        self.include_waist_new = True

        # UI cursor state
        self.focus = FOCUS_JOINT
        self.pose_cursor = 0
        self.seq_cursor = 0
        self.status = "Waiting for rt/lowstate upper-body state..."

        # Timing
        self.last_tick_s = time.monotonic()
        self._running = True

        # Init robot
        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self.controller = UpperBodyPoseController(iface=self.iface, domain_id=self.domain_id)
        self.robot = Robot(iface=self.iface, domain_id=self.domain_id, auto_start_sensors=True)

        self._load_saved_poses()
        self._seed_from_state()

    # ── Boot helpers ─────────────────────────────────────────────────────────

    def _seed_from_state(self) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self.seeded_from_state = True
                self.status = f"Connected on {self.iface}"
                return
            time.sleep(0.02)

    # ── Pose file I/O ─────────────────────────────────────────────────────────

    def _load_saved_poses(self) -> None:
        self.saved_poses = []
        if not self.pose_path.exists():
            return
        try:
            payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.status = f"Could not read pose file: {exc}"
            return
        poses = payload.get("poses", [])
        if isinstance(poses, list):
            self.saved_poses = [
                p for p in poses
                if isinstance(p, dict) and not self._is_generic_name(p.get("name"))
            ]
        self.sequence_steps = [
            s for s in self.sequence_steps
            if 0 <= int(s.get("pose_index", -1)) < len(self.saved_poses)
        ]
        self.pose_cursor = min(self.pose_cursor, max(0, len(self.saved_poses) - 1))

    def _write_saved_poses(self) -> None:
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(
            json.dumps({"poses": self.saved_poses}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _is_generic_name(name: Any) -> bool:
        if name is None:
            return True
        t = str(name).strip().lower()
        return not t or (t.startswith("pose_") and t[5:].isdigit())

    def _pose_payload(self) -> dict[str, Any]:
        src = self.latest_positions if self.seeded_from_state else self.current_targets
        return {
            "name": "",
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {str(j): float(src[j]) for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS},
            "waist_joints": {str(j): float(src[j]) for j in WAIST_JOINTS},
        }

    def _apply_pose(self, pose: dict[str, Any], *, include_waist: bool = True) -> None:
        prev = dict(self.desired_targets)
        arm_j = pose.get("arm_joints")
        if not isinstance(arm_j, dict):
            raise ValueError("Pose missing arm_joints.")
        if include_waist:
            for j in WAIST_JOINTS:
                k = str(j)
                if k in (pose.get("waist_joints") or {}):
                    self.desired_targets[j] = float(pose["waist_joints"][k])
        if self.arm_control_mode == "left":
            active_arm_joints = LEFT_ARM_JOINTS
        elif self.arm_control_mode == "right":
            active_arm_joints = RIGHT_ARM_JOINTS
        else:
            active_arm_joints = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
        for j in active_arm_joints:
            k = str(j)
            if k in arm_j:
                self.desired_targets[j] = float(arm_j[k])
        self.transition_joint_indices = {
            j for j in UPPER_BODY_JOINTS
            if abs(self.desired_targets[j] - prev[j]) > TRANSITION_EPSILON_RAD
        }

    # ── Joint helpers ─────────────────────────────────────────────────────────

    @property
    def _joint(self) -> JointSelection:
        return JOINT_SELECTIONS[self.joint_idx]

    def _joint_slider_range(self, sel: JointSelection | None = None) -> tuple[float, float]:
        sel = sel or self._joint
        if sel.right_index is None or self.arm_control_mode == "both":
            return sel.slider_min, sel.slider_max
        if self.arm_control_mode == "left":
            return sel.left_min, sel.left_max
        return float(sel.right_min), float(sel.right_max)

    def _cycle_arm_control_mode(self) -> None:
        idx = ARM_CONTROL_MODES.index(self.arm_control_mode)
        self.arm_control_mode = ARM_CONTROL_MODES[(idx + 1) % len(ARM_CONTROL_MODES)]
        self.status = f"Arm control set to {self.arm_control_mode}"

    def _set_joint_value(self, value: float) -> None:
        sel = self._joint
        vmin, vmax = self._joint_slider_range(sel)
        clamped = max(vmin, min(vmax, value))
        if sel.right_index is None:
            self.desired_targets[sel.left_index] = clamped
        elif self.arm_control_mode == "left":
            self.desired_targets[sel.left_index] = clamped
        elif self.arm_control_mode == "right":
            self.desired_targets[sel.right_index] = clamped
        else:
            self.desired_targets[sel.left_index] = clamped
            self.desired_targets[sel.right_index] = clamped * sel.right_sign

    def _get_joint_value(self) -> float:
        sel = self._joint
        if sel.right_index is not None and self.arm_control_mode == "right":
            return float(self.desired_targets[sel.right_index])
        return float(self.desired_targets[sel.left_index])

    # ── Robot tick (same logic as GUI _tick + _step_toward_targets) ───────────

    def _step_toward_targets(self, dt: float) -> None:
        step = max(1e-6, self.max_speed * dt)
        for j in UPPER_BODY_JOINTS:
            cur = float(self.current_targets[j])
            des = float(self.desired_targets[j])
            d = des - cur
            if abs(d) <= step:
                self.current_targets[j] = des
            else:
                self.current_targets[j] = cur + step * (1.0 if d > 0 else -1.0)
        if self.transition_joint_indices and all(
            abs(self.current_targets[j] - self.desired_targets[j]) <= TRANSITION_EPSILON_RAD
            for j in self.transition_joint_indices
        ):
            self.transition_joint_indices.clear()

    def _transition_kp_overrides(self) -> dict[int, float]:
        if not self.transition_joint_indices:
            return {}
        return {
            j: max(INACTIVE_TRANSITION_KP, self.waist_kp if j in WAIST_JOINTS else self.arm_kp)
            for j in UPPER_BODY_JOINTS
            if j not in self.transition_joint_indices
        }

    def _advance_sequence(self, now: float) -> None:
        if not self.sequence_running:
            return
        if self.sequence_step_index >= len(self.sequence_steps):
            self.sequence_running = False
            self.status = "Sequence completed"
            return
        if self.sequence_next_time_s < 0.0:
            if self.transition_joint_indices:
                return
            self.sequence_next_time_s = now + max(0.0, self.sequence_gap_s)
            return
        if self.sequence_next_time_s > 0.0 and now < self.sequence_next_time_s:
            return
        step = self.sequence_steps[self.sequence_step_index]
        pi = int(step.get("pose_index", -1))
        if not (0 <= pi < len(self.saved_poses)):
            self.sequence_running = False
            self.status = "Sequence stopped: missing pose"
            return
        pose = self.saved_poses[pi]
        iw = bool(step.get("include_waist", True))
        try:
            self._apply_pose(pose, include_waist=iw)
        except Exception as exc:
            self.sequence_running = False
            self.status = f"Sequence error: {exc}"
            return
        self.seq_cursor = self.sequence_step_index
        self.sequence_step_index += 1
        self.sequence_next_time_s = (
            -1.0 if self.transition_joint_indices
            else now + max(0.0, self.sequence_gap_s)
        )
        name = str(pose.get("name", "<unnamed>"))
        waist_text = "waist on" if iw else "arms only"
        self.status = (
            f"Seq step {self.sequence_step_index}/{len(self.sequence_steps)}: "
            f"{name} ({waist_text})"
        )

    def tick(self) -> None:
        snap = self.state_sub.snapshot()
        if snap is not None:
            pos, _ = snap
            self.latest_positions = pos
            if not self.seeded_from_state:
                self.seeded_from_state = True
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)

        if not self.seeded_from_state:
            return

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, now - self.last_tick_s)
        self.last_tick_s = now

        if not self.control_enabled:
            return

        self._advance_sequence(now)
        self._step_toward_targets(dt)
        self.controller.write_upper_body(
            self.current_targets,
            arm_kp=self.arm_kp, arm_kd=self.arm_kd,
            waist_kp=self.waist_kp, waist_kd=self.waist_kd,
            joint_kp_overrides=self._transition_kp_overrides(),
        )
        if self.seeded_from_state and self.control_enabled:
            self.status = (
                f"Publishing {self.rate_hz:.0f} Hz  ramp {self.max_speed:.3f} rad/s  "
                f"step {self.adjust_step:.3f} rad  arm:{self.arm_control_mode}"
                + ("  [SEQUENCE RUNNING]" if self.sequence_running else "")
                + ("" if self.control_enabled else "  [RELEASED]")
            )

    # ── Curses drawing helpers ────────────────────────────────────────────────

    @staticmethod
    def _safe_addstr(win, y: int, x: int, text: str, attr: int = 0) -> None:
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass

    @staticmethod
    def _safe_addnstr(win, y: int, x: int, text: str, n: int, attr: int = 0) -> None:
        try:
            win.addnstr(y, x, text, n, attr)
        except curses.error:
            pass

    def _cp(self, pair: int) -> int:
        return curses.color_pair(pair) if curses.has_colors() else 0

    def _draw_bar(self, win, y: int, x: int, width: int,
                  value: float, vmin: float, vmax: float) -> None:
        if width <= 0 or vmax <= vmin:
            return
        frac = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
        filled = int(round(frac * width))
        bar = "█" * filled + "░" * (width - filled)
        self._safe_addnstr(win, y, x, bar, width,
                           self._cp(C_GREEN) | curses.A_BOLD)

    # ── Section drawers ───────────────────────────────────────────────────────

    def _draw_header(self, win, h: int, w: int) -> None:
        sel = self._joint
        display_index = (
            sel.right_index
            if sel.right_index is not None and self.arm_control_mode == "right"
            else sel.left_index
        )
        cur_val = float(self.latest_positions.get(
            display_index, self.current_targets[display_index]))
        tgt_val = float(self.desired_targets[display_index])
        smin, smax = self._joint_slider_range(sel)

        # Row 0 — title
        conn_attr = self._cp(C_GREEN if self.seeded_from_state else C_RED) | curses.A_BOLD
        armed_attr = self._cp(C_GREEN if self.control_enabled else C_RED) | curses.A_BOLD
        title = "Dual Arm Mirror CLI"
        self._safe_addstr(win, 0, 0, "─" * w, self._cp(C_CYAN))
        self._safe_addstr(win, 0, max(0, (w - len(title)) // 2), title,
                          self._cp(C_CYAN) | curses.A_BOLD)
        conn_text = "CONNECTED" if self.seeded_from_state else "WAITING"
        armed_text = "ARMED" if self.control_enabled else "RELEASED"
        self._safe_addstr(win, 0, w - 22, f"[{conn_text}]", conn_attr)
        self._safe_addstr(win, 0, w - 12, f"[{armed_text}]", armed_attr)

        # Row 1 — joint name + index
        jname_attr = self._cp(C_CYAN) | curses.A_BOLD
        focus_joint = (self.focus == FOCUS_JOINT)
        focus_mark = "▶ " if focus_joint else "  "
        jinfo = (f"{focus_mark}Joint {self.joint_idx + 1}/{len(JOINT_SELECTIONS)}: "
                 f"{sel.label}  [↑/↓ j/k change]")
        self._safe_addnstr(win, 1, 0, jinfo, w, jname_attr)

        # Row 2 — mirror mapping + speed
        if sel.right_index is None:
            map_txt = f"  Waist: {sel.name} = x"
        elif self.arm_control_mode == "left":
            map_txt = f"  Arms [{self.arm_control_mode}]: left {sel.name} = x"
        elif self.arm_control_mode == "right":
            map_txt = f"  Arms [{self.arm_control_mode}]: right {sel.name} = x"
        else:
            sign = "+" if sel.right_sign > 0 else "-"
            map_txt = f"  Arms [{self.arm_control_mode}]: left {sel.name} = x   right {sel.name} = {sign}x"
        self._safe_addnstr(win, 2, 0, map_txt, w - 30)
        speed_txt = f"Speed: {self.max_speed:.3f} r/s [s]"
        self._safe_addstr(win, 2, max(0, w - len(speed_txt) - 2), speed_txt,
                          self._cp(C_YELLOW))

        # Row 3 — range + step
        range_txt = (f"  Range [{smin:+.3f} … {smax:+.3f}]"
                     f"   Step: {self.adjust_step:.4f} rad  [< >]")
        self._safe_addnstr(win, 3, 0, range_txt, w)

        # Row 4 — target bar
        bar_lpad = 10
        bar_rpad = 12
        bar_width = max(10, w - bar_lpad - bar_rpad)
        self._safe_addstr(win, 4, 0, f"  {smin:+.3f} ")
        self._draw_bar(win, 4, bar_lpad, bar_width, tgt_val, smin, smax)
        val_txt = f" {tgt_val:+.4f}"
        self._safe_addstr(win, 4, bar_lpad + bar_width, val_txt, curses.A_BOLD)
        self._safe_addstr(win, 4, bar_lpad + bar_width + len(val_txt),
                          f" {smax:+.3f}")
        self._safe_addstr(win, 4, w - 14, "[← → + -]", self._cp(C_YELLOW))

        # Row 5 — current actual values
        if sel.right_index is None:
            cr_txt = (f"  Current: {cur_val:+.4f} rad   "
                      f"Target: {tgt_val:+.4f} rad")
        else:
            cl = float(self.latest_positions.get(sel.left_index,
                                                 self.current_targets[sel.left_index]))
            clt = float(self.desired_targets[sel.left_index])
            cr = float(self.latest_positions.get(sel.right_index,
                                                 self.current_targets[sel.right_index]))
            crt = float(self.desired_targets[sel.right_index])
            cr_txt = (f"  Current L/R: {cl:+.4f} / {cr:+.4f} rad   "
                      f"Target L/R: {clt:+.4f} / {crt:+.4f} rad")
        self._safe_addnstr(win, 5, 0, cr_txt, w)

        # Row 6 — divider
        self._safe_addstr(win, 6, 0, "─" * w, self._cp(C_CYAN))

    def _draw_panels(self, win, h: int, w: int) -> None:
        panel_top = 7
        panel_bot = h - 4
        panel_rows = max(0, panel_bot - panel_top)
        mid = w // 2

        # ── Poses panel header (left) ─────────────────────────────────────
        poses_focus = (self.focus == FOCUS_POSES)
        ph_attr = (self._cp(C_FOCUS) | curses.A_BOLD) if poses_focus else curses.A_BOLD
        ph_text = f" Poses ({len(self.saved_poses)}) [Tab focus] [p]save [l/⏎]load [d]del [a]→seq "
        self._safe_addnstr(win, panel_top, 0, ph_text[:mid], mid, ph_attr)

        # ── Sequence panel header (right) ─────────────────────────────────
        seq_focus = (self.focus == FOCUS_SEQUENCE)
        sh_attr = (self._cp(C_FOCUS) | curses.A_BOLD) if seq_focus else curses.A_BOLD
        waist_ind = "W" if self.include_waist_new else "w"
        sh_text = (f" Seq ({len(self.sequence_steps)}) gap:{self.sequence_gap_s:.1f}s "
                   f"[{waist_ind}]waist [R]run [S]stop [x]rem [u/n]↕ ")
        seq_x = mid + 1
        self._safe_addstr(win, panel_top, mid, "│", self._cp(C_CYAN))
        self._safe_addnstr(win, panel_top, seq_x, sh_text[:w - seq_x], w - seq_x, sh_attr)

        # ── Poses list ────────────────────────────────────────────────────
        for row in range(panel_rows):
            y = panel_top + 1 + row
            if y >= panel_bot:
                break
            pidx = row
            self._safe_addstr(win, y, mid, "│", self._cp(C_CYAN))
            if pidx < len(self.saved_poses):
                pose = self.saved_poses[pidx]
                name = str(pose.get("name", f"pose_{pidx}"))
                saved = str(pose.get("saved_at", ""))[:19]
                text = f" {pidx}: {name}  {saved}"
                is_sel = (pidx == self.pose_cursor and poses_focus)
                attr = (self._cp(C_SEL) | curses.A_BOLD) if is_sel else 0
                cur_mark = "▶" if pidx == self.pose_cursor else " "
                line = f"{cur_mark}{text}"
                self._safe_addnstr(win, y, 0, line[:mid], mid, attr)
            else:
                self._safe_addstr(win, y, 0, " " * mid)

        # ── Sequence list ─────────────────────────────────────────────────
        for row in range(panel_rows):
            y = panel_top + 1 + row
            if y >= panel_bot:
                break
            sidx = row
            if sidx >= len(self.sequence_steps):
                continue
            step = self.sequence_steps[sidx]
            pi = int(step.get("pose_index", -1))
            if 0 <= pi < len(self.saved_poses):
                sname = str(self.saved_poses[pi].get("name", f"pose_{pi}"))
            else:
                sname = "<missing>"
            wt = "waist on" if step.get("include_waist", True) else "waist off"
            text = f" {sidx + 1}: {sname} [{wt}]"
            is_sel_seq = (sidx == self.seq_cursor and seq_focus)
            is_active = (self.sequence_running and sidx == self.sequence_step_index - 1)
            if is_sel_seq:
                attr = self._cp(C_SEL) | curses.A_BOLD
            elif is_active:
                attr = self._cp(C_RUNNING) | curses.A_BOLD
            else:
                attr = 0
            cur_mark = "▶" if sidx == self.seq_cursor else " "
            line = f"{cur_mark}{text}"
            self._safe_addnstr(win, y, seq_x, line[:w - seq_x], w - seq_x, attr)

        # ── Bottom divider ─────────────────────────────────────────────────
        if panel_bot < h:
            self._safe_addstr(win, panel_bot, 0, "─" * w, self._cp(C_CYAN))

    def _draw_footer(self, win, h: int, w: int) -> None:
        hints1 = ("y:sync  r:release  e:reengage  z:zero  m:arm mode  Tab:focus  "
                  "a:→seq  x:rem  u:↑  n:↓  R:run  S:stop")
        hints2 = "q:quit  s:speed  g:gap  w:waist  < >:step  ← →:adjust  j k:joint"
        self._safe_addnstr(win, h - 3, 0, hints1[:w], w, self._cp(C_YELLOW))
        self._safe_addnstr(win, h - 2, 0, hints2[:w], w, self._cp(C_YELLOW))

        st_attr = self._cp(C_GREEN if self.control_enabled and self.seeded_from_state else C_RED)
        self._safe_addnstr(win, h - 1, 0, f" {self.status}"[:w], w, st_attr)

    def draw(self, win, h: int, w: int) -> None:
        if h < 14 or w < 60:
            self._safe_addstr(win, 0, 0, f"Terminal too small ({w}x{h}). Need 60x14.")
            return
        try:
            self._draw_header(win, h, w)
            self._draw_panels(win, h, w)
            self._draw_footer(win, h, w)
        except curses.error:
            pass

    # ── Inline prompt ─────────────────────────────────────────────────────────

    def _prompt(self, win, h: int, w: int, label: str) -> str:
        """Read a line from the user at the bottom of the screen."""
        curses.curs_set(1)
        win.timeout(-1)   # blocking during input
        buf: list[str] = []
        while True:
            win.move(h - 1, 0)
            win.clrtoeol()
            display = f"{label}: {''.join(buf)}▌"
            self._safe_addnstr(win, h - 1, 0, display[:w], w, curses.A_BOLD)
            win.refresh()
            key = win.getch()
            if key in (curses.KEY_ENTER, 10, 13):
                break
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                if buf:
                    buf.pop()
            elif key == 27:  # Esc → cancel
                buf = []
                break
            elif 32 <= key <= 126:
                buf.append(chr(key))
        curses.curs_set(0)
        win.timeout(20)
        return "".join(buf).strip()

    # ── Key handler ───────────────────────────────────────────────────────────

    def handle_key(self, key: int, win, h: int, w: int) -> None:  # noqa: C901
        # ── Quit ─────────────────────────────────────────────────────────
        if key in (ord("q"), 27):
            self._running = False
            return

        # ── Focus cycle ───────────────────────────────────────────────────
        if key == 9:  # Tab
            self.focus = (self.focus + 1) % 3
            return

        # ── Arm control mode ─────────────────────────────────────────────
        if key == ord("m"):
            self._cycle_arm_control_mode()
            return

        # ── Joint navigation (always available) ───────────────────────────
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_JOINT:
            self.joint_idx = max(0, self.joint_idx - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_JOINT:
            self.joint_idx = min(len(JOINT_SELECTIONS) - 1, self.joint_idx + 1)
            return

        # ── Pose list navigation ──────────────────────────────────────────
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_POSES:
            self.pose_cursor = max(0, self.pose_cursor - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_POSES:
            self.pose_cursor = min(max(0, len(self.saved_poses) - 1), self.pose_cursor + 1)
            return

        # ── Sequence list navigation ──────────────────────────────────────
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_SEQUENCE:
            self.seq_cursor = max(0, self.seq_cursor - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_SEQUENCE:
            self.seq_cursor = min(max(0, len(self.sequence_steps) - 1), self.seq_cursor + 1)
            return

        # ── Value adjustment (works regardless of focus) ──────────────────
        if key in (curses.KEY_LEFT, ord("-")):
            self._set_joint_value(self._get_joint_value() - self.adjust_step)
            return
        if key in (curses.KEY_RIGHT, ord("+")):
            self._set_joint_value(self._get_joint_value() + self.adjust_step)
            return

        # ── Step size ─────────────────────────────────────────────────────
        if key == ord("<"):
            self.adjust_step = max(0.001, self.adjust_step / 2.0)
            return
        if key == ord(">"):
            self.adjust_step = min(0.5, self.adjust_step * 2.0)
            return

        # ── Speed prompt ──────────────────────────────────────────────────
        if key == ord("s"):
            val = self._prompt(win, h, w, f"Speed rad/s [{self.max_speed:.3f}]")
            try:
                self.max_speed = max(0.01, float(val))
                self.status = f"Speed set to {self.max_speed:.3f} rad/s"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid speed: {val!r}"
            return

        # ── Gap prompt ────────────────────────────────────────────────────
        if key == ord("g"):
            val = self._prompt(win, h, w, f"Sequence gap s [{self.sequence_gap_s:.1f}]")
            try:
                self.sequence_gap_s = max(0.0, float(val))
                self.status = f"Sequence gap set to {self.sequence_gap_s:.1f} s"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid gap: {val!r}"
            return

        # ── Sync to current ───────────────────────────────────────────────
        if key == ord("y"):
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
            self.current_targets = dict(self.latest_positions)
            self.desired_targets = dict(self.latest_positions)
            self.transition_joint_indices.clear()
            self.status = "Desired targets synced to current pose"
            return

        # ── Release / reengage / zero ─────────────────────────────────────
        if key == ord("r"):
            try:
                self.robot.wait_for_low_state(timeout=2.0)
                self.robot.release_arms()
                self.control_enabled = False
                self.status = "Arms released"
            except Exception as exc:
                self.status = f"Release failed: {exc}"
            return

        if key == ord("e"):
            try:
                self.robot.wait_for_low_state(timeout=2.0)
                self.robot.unrelease_arms()
                self.control_enabled = True
                snap = self.state_sub.snapshot()
                if snap:
                    pos, _ = snap
                    self.latest_positions = dict(pos)
                self.current_targets = dict(self.latest_positions)
                self.desired_targets = dict(self.latest_positions)
                self.transition_joint_indices.clear()
                self.status = "Arms reengaged; synced to live pose"
            except Exception as exc:
                self.status = f"Reengage failed: {exc}"
            return

        if key == ord("z"):
            self.controller.write_zero_gains(self.current_targets)
            self.status = "Zero-gain hold sent on rt/arm_sdk"
            return

        # ── Toggle include-waist for new sequence steps ───────────────────
        if key == ord("w"):
            self.include_waist_new = not self.include_waist_new
            self.status = f"New seq steps will {'include' if self.include_waist_new else 'exclude'} waist"
            return

        # ── Save pose ─────────────────────────────────────────────────────
        if key == ord("p"):
            name = self._prompt(win, h, w, "Pose name")
            if not name:
                self.status = "Save cancelled (empty name)"
                return
            pose = self._pose_payload()
            pose["name"] = name
            self.saved_poses.append(pose)
            self._write_saved_poses()
            self.pose_cursor = len(self.saved_poses) - 1
            self.status = f"Saved pose '{name}'"
            return

        # ── Load pose ─────────────────────────────────────────────────────
        if key in (ord("l"), curses.KEY_ENTER, 10) and self.focus == FOCUS_POSES:
            if 0 <= self.pose_cursor < len(self.saved_poses):
                pose = self.saved_poses[self.pose_cursor]
                try:
                    self._apply_pose(pose)
                    name = str(pose.get("name", "<unnamed>"))
                    self.status = f"Loaded pose '{name}'"
                except Exception as exc:
                    self.status = f"Load failed: {exc}"
            else:
                self.status = "No pose selected"
            return

        # ── Delete pose ───────────────────────────────────────────────────
        if key == ord("d") and self.focus == FOCUS_POSES:
            row = self.pose_cursor
            if 0 <= row < len(self.saved_poses):
                name = str(self.saved_poses[row].get("name", f"pose_{row}"))
                new_seq = []
                for step in self.sequence_steps:
                    pi = int(step.get("pose_index", -1))
                    if pi == row:
                        continue
                    new_seq.append({"pose_index": pi - (1 if pi > row else 0),
                                    "include_waist": bool(step.get("include_waist", True))})
                self.sequence_steps = new_seq
                del self.saved_poses[row]
                if self.sequence_running:
                    self.sequence_running = False
                self._write_saved_poses()
                self.pose_cursor = min(self.pose_cursor, max(0, len(self.saved_poses) - 1))
                self.seq_cursor = min(self.seq_cursor, max(0, len(self.sequence_steps) - 1))
                self.status = f"Deleted pose '{name}'"
            else:
                self.status = "No pose selected"
            return

        # ── Add pose to sequence ──────────────────────────────────────────
        if key == ord("a") and self.focus == FOCUS_POSES:
            row = self.pose_cursor
            if 0 <= row < len(self.saved_poses):
                self.sequence_steps.append(
                    {"pose_index": row, "include_waist": self.include_waist_new}
                )
                self.seq_cursor = len(self.sequence_steps) - 1
                name = str(self.saved_poses[row].get("name", f"pose_{row}"))
                waist_text = "with waist" if self.include_waist_new else "arms only"
                self.status = f"Added '{name}' to sequence ({waist_text})"
            else:
                self.status = "No pose selected"
            return

        # ── Remove sequence step ──────────────────────────────────────────
        if key in (ord("x"), curses.KEY_DC) and self.focus == FOCUS_SEQUENCE:
            row = self.seq_cursor
            if 0 <= row < len(self.sequence_steps):
                del self.sequence_steps[row]
                self.seq_cursor = min(self.seq_cursor, max(0, len(self.sequence_steps) - 1))
                if self.sequence_running:
                    self.sequence_running = False
                self.status = f"Removed sequence step {row + 1}"
            return

        # ── Move sequence step up / down ──────────────────────────────────
        if key == ord("u") and self.focus == FOCUS_SEQUENCE:
            row = self.seq_cursor
            if 1 <= row < len(self.sequence_steps):
                self.sequence_steps[row - 1], self.sequence_steps[row] = (
                    self.sequence_steps[row], self.sequence_steps[row - 1])
                self.seq_cursor -= 1
            return

        if key == ord("n") and self.focus == FOCUS_SEQUENCE:
            row = self.seq_cursor
            if 0 <= row < len(self.sequence_steps) - 1:
                self.sequence_steps[row + 1], self.sequence_steps[row] = (
                    self.sequence_steps[row], self.sequence_steps[row + 1])
                self.seq_cursor += 1
            return

        # ── Run / stop sequence ───────────────────────────────────────────
        if key == ord("R"):
            if not self.control_enabled:
                self.status = "Reengage arms before running a sequence"
            elif not self.sequence_steps:
                self.status = "Add poses to the sequence first"
            else:
                self.sequence_running = True
                self.sequence_step_index = 0
                self.sequence_next_time_s = 0.0
                self.status = "Sequence started"
            return

        if key == ord("S"):
            self.sequence_running = False
            self.sequence_step_index = 0
            self.sequence_next_time_s = 0.0
            self.status = "Sequence stopped"
            return

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self) -> None:
        curses.wrapper(self._curses_main)

    def _curses_main(self, stdscr) -> None:
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_GREEN, curses.COLOR_GREEN, -1)
            curses.init_pair(C_YELLOW, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_RED, curses.COLOR_RED, -1)
            curses.init_pair(C_CYAN, curses.COLOR_CYAN, -1)
            curses.init_pair(C_SEL, curses.COLOR_BLACK, curses.COLOR_WHITE)
            curses.init_pair(C_FOCUS, curses.COLOR_BLACK, curses.COLOR_CYAN)
            curses.init_pair(C_RUNNING, curses.COLOR_WHITE, curses.COLOR_BLUE)

        curses.curs_set(0)
        stdscr.timeout(20)   # 50 fps; also sets tick cadence

        last_tick = 0.0
        dt_target = 1.0 / self.rate_hz

        while self._running:
            h, w = stdscr.getmaxyx()
            try:
                stdscr.erase()
                self.draw(stdscr, h, w)
                stdscr.refresh()
            except curses.error:
                pass

            key = stdscr.getch()
            if key != -1:
                self.handle_key(key, stdscr, h, w)

            now = time.monotonic()
            if now - last_tick >= dt_target:
                self.tick()
                last_tick = now

        # Clean shutdown
        try:
            self.robot.release_arms()
        except Exception:
            pass


# ── Argument parsing & main ───────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Terminal mirror of dual_arm_mirror_ui_with_waist.py"
    )
    p.add_argument("--iface", default="eth0", help="Network interface for DDS")
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--file", default=DEFAULT_POSE_FILE, help="Saved pose JSON file")
    p.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate")
    p.add_argument("--speed-rad-s", type=float, default=0.1, help="Initial ramp limit rad/s")
    p.add_argument("--kp", type=float, default=DEFAULT_ARM_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_ARM_KD)
    p.add_argument(
        "--arm-control",
        choices=ARM_CONTROL_MODES,
        default="both",
        help="Initial arm target mode for manual edits, pose loads, and sequences",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    app = DualArmMirrorCLI(args)
    app.run()


if __name__ == "__main__":
    main()
