#!/usr/bin/env python3
"""
6D end-effector TUI for Unitree G1 arm — Cartesian jogging via coupled joints.

Translation axes (approximate straight-line end-effector motions):
  X  forward / back    shoulder_pitch + elbow          decrease both = forward
  Y  away / toward     shoulder_roll  + wrist_roll      right arm: both decrease = away
  Z  up / down         elbow + wrist_pitch             decrease elbow + increase wrist_pitch = up

Rotation axes (wrist only, no coupling):
  Roll                 wrist_roll
  Pitch                wrist_pitch
  Yaw                  wrist_yaw

Key bindings
────────────
  W / S         +X / -X   (forward / back)
  A / D         +Y / -Y   (away from body / toward)
  Q / E         +Z / -Z   (up / down)
  I / K         Roll+ / Roll-
  J / L         Pitch+ / Pitch-
  U / O         Yaw+ / Yaw-
  < / >         halve / double step size
  s             set ramp speed (prompt)
  y             sync targets → live pose
  r             release arms
  e             reengage arms
  z             zero gains once
  m             cycle arm: right → left → both
  ↑ / ↓         navigate pose list
  p             save pose (name prompt)
  l / Enter     load selected pose
  d             delete selected pose
  q / Esc       quit
"""

from __future__ import annotations
from sdk_client import Robot
from dds_env import ensure_cyclonedds_environment

import argparse
import curses
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
MODULES_DIR = os.path.join(ROOT_DIR, "modules")
for _p in (ROOT_DIR, MODULES_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
    )
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
        "unitree_sdk2py not installed.\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


# ── Constants ──────────────────────────────────────────────────────────────────
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

LEFT_BASE = 15   # joint index of left shoulder_pitch
RIGHT_BASE = 22   # joint index of right shoulder_pitch

# Offsets within each arm: 0=shl_pitch, 1=shl_roll, 2=shl_yaw,
#                          3=elbow, 4=wrs_roll, 5=wrs_pitch, 6=wrs_yaw
JOINT_NAMES = [
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw",
]

# (min, max) for the displayed/controlled arm
# Right arm limits (independent control, not mirror)
RIGHT_LIMITS = [
    (-3.0892, 2.6704),  # shoulder_pitch
    (-2.2515, 1.5882),  # shoulder_roll
    (-2.6180, 2.6180),  # shoulder_yaw
    (-1.0472, 2.0944),  # elbow
    (-1.9722, 1.9722),  # wrist_roll
    (-1.6144, 1.6144),  # wrist_pitch
    (-1.6144, 1.6144),  # wrist_yaw
]
# Left arm limits
LEFT_LIMITS = [
    (-3.0892, 2.6704),  # shoulder_pitch
    (-1.5882, 2.2515),  # shoulder_roll
    (-2.6180, 2.6180),  # shoulder_yaw
    (-1.0472, 2.0944),  # elbow
    (-1.9722, 1.9722),  # wrist_roll
    (-1.6144, 1.6144),  # wrist_pitch
    (-1.6144, 1.6144),  # wrist_yaw
]

ARM_MODES = ["right", "left", "both"]

# ── 6D axis → joint coupling ───────────────────────────────────────────────────
# Each entry: (joint_offset, scale)
# Positive axis command → joint_value += step * scale
# Right-arm conventions:
#   X+  forward:      shoulder_pitch and elbow both decrease
#   Y+  away (right): shoulder_roll and wrist_roll both decrease
#   Z+  up:           elbow decreases, wrist_pitch increases
AXIS_COUPLINGS_RIGHT: dict[str, list[tuple[int, float]]] = {
    "X": [(0, -1.0), (3, -1.0)],   # shl_pitch -, elbow -
    "Y": [(1, -1.0), (4, -1.0)],   # shl_roll -,  wrs_roll -
    "Z": [(3, -1.0), (5, +1.0)],   # elbow -,     wrs_pitch +
    "Roll": [(4, +1.0)],
    "Pitch": [(5, +1.0)],
    "Yaw": [(6, +1.0)],
}
# Left arm: Y motion is mirrored (shoulder_roll sign flips)
AXIS_COUPLINGS_LEFT: dict[str, list[tuple[int, float]]] = {
    "X": [(0, -1.0), (3, -1.0)],
    "Y": [(1, +1.0), (4, +1.0)],   # mirrored: left arm away = increase shl_roll
    "Z": [(3, -1.0), (5, +1.0)],
    "Roll": [(4, +1.0)],
    "Pitch": [(5, +1.0)],
    "Yaw": [(6, +1.0)],
}

# Human-readable coupling summary for display
AXIS_COUPLING_DESC: dict[str, str] = {
    "X": "shl_pitch + elbow",
    "Y": "shl_roll + wrs_roll",
    "Z": "elbow + wrs_pitch",
    "Roll": "wrist_roll",
    "Pitch": "wrist_pitch",
    "Yaw": "wrist_yaw",
}

# key → (axis, sign)
KEY_AXIS_MAP: dict[int, tuple[str, float]] = {
    ord("w"): ("X", +1.0),
    ord("s"): ("X", -1.0),
    ord("a"): ("Y", +1.0),
    ord("d"): ("Y", -1.0),
    ord("q"): ("Z", +1.0),
    ord("e"): ("Z", -1.0),
    ord("i"): ("Roll", +1.0),
    ord("k"): ("Roll", -1.0),
    ord("j"): ("Pitch", +1.0),
    ord("l"): ("Pitch", -1.0),
    ord("u"): ("Yaw", +1.0),
    ord("o"): ("Yaw", -1.0),
}

FLASH_DURATION = 0.25  # seconds to highlight moved joints

# ── Colour pair indices ────────────────────────────────────────────────────────
C_GREEN = 1
C_YELLOW = 2
C_RED = 3
C_CYAN = 4
C_SEL = 5
C_FOCUS = 6


# ── LowState subscriber ────────────────────────────────────────────────────────
def _resolve_lowstate_type():
    for path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
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

    def snapshot(self) -> dict[int, float] | None:
        with self._lock:
            return dict(self._positions) if self._positions else None


class UpperBodyPoseController:
    def __init__(self) -> None:
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0

    def write(self, targets: dict[int, float], *,
              arm_kp, arm_kd, waist_kp, waist_kd,
              kp_overrides: dict[int, float] | None = None) -> None:
        ov = kp_overrides or {}
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


# ── Main app ───────────────────────────────────────────────────────────────────

class Arm6DCLI:
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
        self.arm_mode = str(args.arm_mode)
        self.adjust_step = 0.05

        self.latest_positions: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.seeded_from_state = False
        self.control_enabled = True
        self.transition_indices: set[int] = set()

        self.saved_poses: list[dict[str, Any]] = []
        self.pose_cursor = 0
        self.status = "Waiting for rt/lowstate..."
        self._running = True
        self.last_tick_s = time.monotonic()

        self._flash_joints: set[int] = set()
        self._flash_time = 0.0

        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self.controller = UpperBodyPoseController()
        self.robot = Robot(iface=self.iface, domain_id=self.domain_id,
                           auto_start_sensors=True)
        self._load_poses()
        self._seed_from_state()

    # ── Init helpers ───────────────────────────────────────────────────────────

    def _seed_from_state(self) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snap = self.state_sub.snapshot()
            if snap:
                self.latest_positions = snap
                self.current_targets = dict(snap)
                self.desired_targets = dict(snap)
                self.seeded_from_state = True
                self.status = f"Connected — {self.iface}"
                return
            time.sleep(0.02)

    # ── Pose file I/O ──────────────────────────────────────────────────────────

    def _load_poses(self) -> None:
        self.saved_poses = []
        if not self.pose_path.exists():
            return
        try:
            data = json.loads(self.pose_path.read_text(encoding="utf-8"))
            self.saved_poses = [
                p for p in data.get("poses", [])
                if isinstance(p, dict)
            ]
        except Exception as exc:
            self.status = f"Pose file read error: {exc}"

    def _save_poses(self) -> None:
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(
            json.dumps({"poses": self.saved_poses}, indent=2) + "\n",
            encoding="utf-8",
        )

    def _pose_snapshot(self) -> dict[str, Any]:
        src = self.latest_positions if self.seeded_from_state else self.current_targets
        return {
            "name": "",
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {
                str(j): float(src[j])
                for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
            },
        }

    def _apply_pose(self, pose: dict[str, Any]) -> None:
        joints = pose.get("arm_joints", {})
        if not joints:
            raise ValueError("pose missing arm_joints")
        prev = dict(self.desired_targets)
        if self.arm_mode in ("right", "both"):
            for j in RIGHT_ARM_JOINTS:
                k = str(j)
                if k in joints:
                    self.desired_targets[j] = float(joints[k])
        if self.arm_mode in ("left", "both"):
            for j in LEFT_ARM_JOINTS:
                k = str(j)
                if k in joints:
                    self.desired_targets[j] = float(joints[k])
        self.transition_indices = {
            j for j in UPPER_BODY_JOINTS
            if abs(self.desired_targets[j] - prev[j]) > TRANSITION_EPSILON_RAD
        }

    # ── 6D motion ──────────────────────────────────────────────────────────────

    def _apply_axis(self, axis: str, sign: float) -> None:
        step = self.adjust_step * sign
        moved: set[int] = set()

        if self.arm_mode in ("right", "both"):
            for offset, scale in AXIS_COUPLINGS_RIGHT[axis]:
                j = RIGHT_BASE + offset
                lim = RIGHT_LIMITS[offset]
                self.desired_targets[j] = max(
                    lim[0], min(lim[1], self.desired_targets[j] + step * scale)
                )
                moved.add(j)

        if self.arm_mode in ("left", "both"):
            for offset, scale in AXIS_COUPLINGS_LEFT[axis]:
                j = LEFT_BASE + offset
                lim = LEFT_LIMITS[offset]
                self.desired_targets[j] = max(
                    lim[0], min(lim[1], self.desired_targets[j] + step * scale)
                )
                moved.add(j)

        self._flash_joints = moved
        self._flash_time = time.monotonic()

    # ── Robot tick ─────────────────────────────────────────────────────────────

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
        if self.transition_indices and all(
            abs(self.current_targets[j] - self.desired_targets[j]) <= TRANSITION_EPSILON_RAD
            for j in self.transition_indices
        ):
            self.transition_indices.clear()

    def _kp_overrides(self) -> dict[int, float]:
        if not self.transition_indices:
            return {}
        return {
            j: max(
                INACTIVE_TRANSITION_KP,
                self.waist_kp if j in WAIST_JOINTS else self.arm_kp,
            )
            for j in UPPER_BODY_JOINTS
            if j not in self.transition_indices
        }

    def tick(self) -> None:
        snap = self.state_sub.snapshot()
        if snap:
            self.latest_positions = snap
            if not self.seeded_from_state:
                self.seeded_from_state = True
                self.current_targets = dict(snap)
                self.desired_targets = dict(snap)
        if not self.seeded_from_state or not self.control_enabled:
            return
        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, now - self.last_tick_s)
        self.last_tick_s = now
        self._step_toward_targets(dt)
        self.controller.write(
            self.current_targets,
            arm_kp=self.arm_kp, arm_kd=self.arm_kd,
            waist_kp=self.waist_kp, waist_kd=self.waist_kd,
            kp_overrides=self._kp_overrides(),
        )

    # ── Drawing helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _safe_add(win, y: int, x: int, text: str, attr: int = 0) -> None:
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass

    @staticmethod
    def _safe_addn(win, y: int, x: int, text: str, n: int, attr: int = 0) -> None:
        try:
            win.addnstr(y, x, text, max(0, n), attr)
        except curses.error:
            pass

    def _cp(self, pair: int) -> int:
        return curses.color_pair(pair) if curses.has_colors() else 0

    def _draw_bar(self, win, y: int, x: int, width: int,
                  value: float, vmin: float, vmax: float, attr: int = 0) -> None:
        if width <= 0 or vmax <= vmin:
            return
        frac = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
        filled = int(round(frac * width))
        bar = "█" * filled + "░" * (width - filled)
        try:
            win.addnstr(y, x, bar, width, attr or (self._cp(C_GREEN) | curses.A_BOLD))
        except curses.error:
            pass

    # ── Section drawers ────────────────────────────────────────────────────────

    def _draw_header(self, win, h: int, w: int) -> int:
        conn_attr = self._cp(C_GREEN if self.seeded_from_state else C_RED) | curses.A_BOLD
        armed_attr = self._cp(C_GREEN if self.control_enabled else C_RED) | curses.A_BOLD
        title = "6D End-Effector Arm Controller"
        self._safe_add(win, 0, 0, "─" * w, self._cp(C_CYAN))
        self._safe_add(win, 0, max(0, (w - len(title)) // 2), title,
                       self._cp(C_CYAN) | curses.A_BOLD)
        conn_txt = "CONNECTED" if self.seeded_from_state else "WAITING"
        armed_txt = "ARMED" if self.control_enabled else "RELEASED"
        self._safe_add(win, 0, w - 23, f"[{conn_txt}]", conn_attr)
        self._safe_add(win, 0, w - 12, f"[{armed_txt}]", armed_attr)

        cfg = (f" arm:{self.arm_mode}  step:{self.adjust_step:.4f}rad  "
               f"speed:{self.max_speed:.3f}rad/s  kp:{self.arm_kp:.0f}  "
               f"[<>]step [s]speed [m]arm")
        self._safe_addn(win, 1, 0, cfg, w, self._cp(C_YELLOW))
        self._safe_add(win, 2, 0, "─" * w, self._cp(C_CYAN))
        return 3  # next row

    def _draw_axes(self, win, top: int, w: int) -> int:
        mid = w // 2
        # Headers
        self._safe_addn(win, top, 0,
                        " TRANSLATION    [W/S:X]  [A/D:Y]  [Q/E:Z]",
                        mid, curses.A_BOLD)
        self._safe_add(win, top, mid, "│", self._cp(C_CYAN))
        self._safe_addn(win, top, mid + 1,
                        " ROTATION    [I/K:roll]  [J/L:pitch]  [U/O:yaw]",
                        w - mid - 1, curses.A_BOLD)

        rows_t = [
            ("X", "W", "S", "fwd/bk"),
            ("Y", "A", "D", "away/twrd"),
            ("Z", "Q", "E", "up/dn"),
        ]
        rows_r = [
            ("Roll", "I", "K"),
            ("Pitch", "J", "L"),
            ("Yaw", "U", "O"),
        ]
        for i, (axis, kp, kn, desc) in enumerate(rows_t):
            y = top + 1 + i
            txt = (f"  {axis} ({desc}): {kp}/{kn} → {AXIS_COUPLING_DESC[axis]}")
            self._safe_addn(win, y, 0, txt, mid)
            self._safe_add(win, y, mid, "│", self._cp(C_CYAN))
        for i, (axis, kp, kn) in enumerate(rows_r):
            y = top + 1 + i
            txt = f"  {axis}: {kp}/{kn} → {AXIS_COUPLING_DESC[axis]}"
            self._safe_addn(win, y, mid + 1, txt, w - mid - 1)

        next_row = top + 4
        self._safe_add(win, next_row, 0, "─" * w, self._cp(C_CYAN))
        return next_row + 1

    def _draw_joints(self, win, top: int, w: int) -> int:
        now = time.monotonic()
        flash_active = (now - self._flash_time) < FLASH_DURATION

        # Determine which arm to display (if both, show right)
        if self.arm_mode == "left":
            base = LEFT_BASE
            limits = LEFT_LIMITS
        else:
            base = RIGHT_BASE
            limits = RIGHT_LIMITS

        label_w = 22
        val_w = 28  # "cur: +0.0000  tgt: +0.0000"
        bar_w = max(8, w - label_w - val_w - 2)

        for i, name in enumerate(JOINT_NAMES):
            y = top + i
            ji = base + i
            lim = limits[i]
            cur = float(self.latest_positions.get(ji, self.current_targets[ji]))
            tgt = float(self.desired_targets[ji])
            in_flash = flash_active and ji in self._flash_joints
            label_attr = (self._cp(C_YELLOW) | curses.A_BOLD) if in_flash else curses.A_BOLD
            label_txt = f" {name:<20}[{ji:02d}]"
            self._safe_addn(win, y, 0, label_txt, label_w, label_attr)
            val_txt = f" c:{cur:+.4f} t:{tgt:+.4f}"
            self._safe_addn(win, y, label_w, val_txt, val_w)
            bar_attr = (self._cp(C_YELLOW) | curses.A_BOLD) if in_flash else 0
            self._draw_bar(win, y, label_w + val_w, bar_w, tgt, lim[0], lim[1], bar_attr)

        next_row = top + len(JOINT_NAMES)
        self._safe_add(win, next_row, 0, "─" * w, self._cp(C_CYAN))
        return next_row + 1

    def _draw_poses(self, win, top: int, h: int, w: int) -> None:
        footer_rows = 3
        avail = max(0, h - footer_rows - top - 1)
        hdr = f" Poses ({len(self.saved_poses)}) [p]save [l/⏎]load [d]del [↑↓]nav"
        self._safe_addn(win, top, 0, hdr, w, curses.A_BOLD)
        for row in range(avail):
            y = top + 1 + row
            pidx = row
            if pidx >= len(self.saved_poses):
                break
            pose = self.saved_poses[pidx]
            name = str(pose.get("name", f"pose_{pidx}"))
            ts = str(pose.get("saved_at", ""))[:19]
            mark = "▶" if pidx == self.pose_cursor else " "
            is_sel = (pidx == self.pose_cursor)
            attr = (self._cp(C_SEL) | curses.A_BOLD) if is_sel else 0
            self._safe_addn(win, y, 0, f"{mark} {pidx}: {name:<24} {ts}", w, attr)
        self._safe_add(win, h - footer_rows, 0, "─" * w, self._cp(C_CYAN))

    def _draw_footer(self, win, h: int, w: int) -> None:
        hints1 = " W/S:X  A/D:Y  Q/E:Z  I/K:roll  J/L:pitch  U/O:yaw  m:arm  </>:step"
        hints2 = " y:sync  r:release  e:reengage  z:zero  s:speed  p:save  l:load  q:quit"
        self._safe_addn(win, h - 2, 0, hints1, w, self._cp(C_YELLOW))
        self._safe_addn(win, h - 1, 0, hints2, w, self._cp(C_YELLOW))

    def _draw_status(self, win, h: int, w: int) -> None:
        conn = self.seeded_from_state and self.control_enabled
        attr = self._cp(C_GREEN if conn else C_RED)
        self._safe_addn(win, h - 3, 0, f" {self.status}", w, attr)

    def draw(self, win, h: int, w: int) -> None:
        if h < 22 or w < 80:
            self._safe_add(win, 0, 0, f"Terminal too small ({w}x{h}). Need 80x22.")
            return
        try:
            row = self._draw_header(win, h, w)   # rows 0-2, returns 3
            row = self._draw_axes(win, row, w)    # rows 3-7, returns 8
            row = self._draw_joints(win, row, w)  # rows 8-15 (7 joints + sep), returns 16
            self._draw_poses(win, row, h, w)
            self._draw_footer(win, h, w)
            self._draw_status(win, h, w)
        except curses.error:
            pass

    # ── Inline prompt ──────────────────────────────────────────────────────────

    def _prompt(self, win, h: int, w: int, label: str) -> str:
        curses.curs_set(1)
        win.timeout(-1)
        buf: list[str] = []
        while True:
            win.move(h - 3, 0)
            win.clrtoeol()
            self._safe_addn(win, h - 3, 0,
                            f" {label}: {''.join(buf)}▌", w, curses.A_BOLD)
            win.refresh()
            key = win.getch()
            if key in (curses.KEY_ENTER, 10, 13):
                break
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                if buf:
                    buf.pop()
            elif key == 27:
                buf = []
                break
            elif 32 <= key <= 126:
                buf.append(chr(key))
        curses.curs_set(0)
        win.timeout(20)
        return "".join(buf).strip()

    # ── Key handler ────────────────────────────────────────────────────────────

    def handle_key(self, key: int, win, h: int, w: int) -> None:  # noqa: C901
        if key in (ord("q"), 27):
            self._running = False
            return

        # 6D axis motion
        if key in KEY_AXIS_MAP:
            axis, sign = KEY_AXIS_MAP[key]
            self._apply_axis(axis, sign)
            self.status = (
                f"{'↑' if sign > 0 else '↓'}{axis}  "
                f"step={self.adjust_step:.4f}  arm:{self.arm_mode}"
            )
            return

        # Step size
        if key == ord("<"):
            self.adjust_step = max(0.001, self.adjust_step / 2.0)
            self.status = f"Step → {self.adjust_step:.4f} rad"
            return
        if key == ord(">"):
            self.adjust_step = min(0.5, self.adjust_step * 2.0)
            self.status = f"Step → {self.adjust_step:.4f} rad"
            return

        # Arm mode cycle
        if key == ord("m"):
            idx = ARM_MODES.index(self.arm_mode)
            self.arm_mode = ARM_MODES[(idx + 1) % len(ARM_MODES)]
            self.status = f"Arm mode → {self.arm_mode}"
            return

        # Speed prompt
        if key == ord("s"):
            val = self._prompt(win, h, w, f"Speed rad/s [{self.max_speed:.3f}]")
            try:
                self.max_speed = max(0.01, float(val))
                self.status = f"Speed → {self.max_speed:.3f} rad/s"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid speed: {val!r}"
            return

        # Sync
        if key == ord("y"):
            snap = self.state_sub.snapshot()
            if snap:
                self.latest_positions = snap
            self.current_targets = dict(self.latest_positions)
            self.desired_targets = dict(self.latest_positions)
            self.transition_indices.clear()
            self.status = "Targets synced to live pose"
            return

        # Release / reengage / zero
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
                    self.latest_positions = snap
                self.current_targets = dict(self.latest_positions)
                self.desired_targets = dict(self.latest_positions)
                self.transition_indices.clear()
                self.status = "Arms reengaged; synced"
            except Exception as exc:
                self.status = f"Reengage failed: {exc}"
            return

        if key == ord("z"):
            self.controller.write_zero_gains(self.current_targets)
            self.status = "Zero-gain hold sent"
            return

        # Pose list navigation
        if key in (curses.KEY_UP,):
            self.pose_cursor = max(0, self.pose_cursor - 1)
            return
        if key in (curses.KEY_DOWN,):
            self.pose_cursor = min(max(0, len(self.saved_poses) - 1),
                                   self.pose_cursor + 1)
            return

        # Save pose
        if key == ord("p"):
            name = self._prompt(win, h, w, "Pose name")
            if not name:
                self.status = "Save cancelled"
                return
            pose = self._pose_snapshot()
            pose["name"] = name
            self.saved_poses.append(pose)
            self._save_poses()
            self.pose_cursor = len(self.saved_poses) - 1
            self.status = f"Saved '{name}'"
            return

        # Load pose
        if key in (ord("l"), curses.KEY_ENTER, 10, 13):
            if 0 <= self.pose_cursor < len(self.saved_poses):
                try:
                    self._apply_pose(self.saved_poses[self.pose_cursor])
                    name = str(self.saved_poses[self.pose_cursor].get("name", ""))
                    self.status = f"Loaded '{name}'"
                except Exception as exc:
                    self.status = f"Load failed: {exc}"
            else:
                self.status = "No pose selected"
            return

        # Delete pose
        if key == ord("d"):
            row = self.pose_cursor
            if 0 <= row < len(self.saved_poses):
                name = str(self.saved_poses[row].get("name", f"pose_{row}"))
                del self.saved_poses[row]
                self._save_poses()
                self.pose_cursor = min(self.pose_cursor, max(0, len(self.saved_poses) - 1))
                self.status = f"Deleted '{name}'"
            else:
                self.status = "No pose selected"
            return

    # ── Curses main loop ───────────────────────────────────────────────────────

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
        curses.curs_set(0)
        stdscr.timeout(20)

        dt_target = 1.0 / self.rate_hz
        last_tick = 0.0

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

        try:
            self.robot.release_arms()
        except Exception:
            pass


# ── Argument parsing & main ────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="6D end-effector TUI — Cartesian jogging for Unitree G1 arm."
    )
    p.add_argument("--iface", default="eth0",
                   help="Network interface for DDS")
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--file", default="saved_6d_poses.json",
                   help="JSON file for saved poses")
    p.add_argument("--rate-hz", type=float, default=50.0,
                   help="Command publish rate (Hz)")
    p.add_argument("--speed-rad-s", type=float, default=0.1,
                   help="Initial ramp limit (rad/s)")
    p.add_argument("--kp", type=float, default=DEFAULT_ARM_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_ARM_KD)
    p.add_argument("--arm-mode", choices=ARM_MODES, default="right",
                   help="Initial arm control target")
    return p.parse_args()


def main() -> None:
    app = Arm6DCLI(parse_args())
    app.run()


if __name__ == "__main__":
    main()
