#!/usr/bin/env python3
"""
6D End-Effector IK Pose Control TUI.

Controls the absolute 6D Cartesian pose (x, y, z, roll, pitch, yaw) of the
chosen hand end-effector (left / right / both) by running damped-least-squares
IK over the 7 arm DOFs (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw).

Each key press:
  1. Increments the EE target by the configured step.
  2. Runs warm-started DLS IK from the previous joint solution.
  3. Clamps each resulting joint delta to ±max_dq (default 0.2 rad).
  4. Queues the clamped solution as the ramp target.
  5. The control loop ramps current joints smoothly toward the target at
     max_speed r/s with uniform servo gains throughout.

If IK fails the EE target is rolled back, joints do not move.

Key bindings
────────────
  ↑ / ↓  or  k / j    select DOF (x y z roll pitch yaw)
  ← / →  or  - / +    decrement / increment selected DOF by step
  < / >                halve / double EE step for selected DOF class
  [ / ]                halve / double max joint-delta per IK step
  m                    cycle arm mode: both → left → right
  y                    sync EE targets → current FK pose (resync)
  r                    release arms (zero servo gains)
  e                    reengage arms and resync to live pose
  z                    send zero-gain hold packet once
  s                    set ramp speed rad/s (prompt)
  d                    set max joint-delta rad (prompt)
  q / Esc              quit
"""
from __future__ import annotations

import argparse
import curses
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
MODULES_DIR = os.path.join(ROOT_DIR, "modules")
for _p in (ROOT_DIR, MODULES_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from hand_pose_navigation_copy.arm_ik import ArmIK
    from hand_pose_navigation_copy.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
        _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
    )
except ModuleNotFoundError:
    from hand_pose_navigation.arm_ik import ArmIK
    from hand_pose_navigation.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
        _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
    )

from sdk_client import Robot
from dds_env import ensure_cyclonedds_environment

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
    raise SystemExit("unitree_sdk2py not installed.") from exc


# FK/IK package at <ROOT_DIR>/hand_pose_navigation_copy/

# ── Joint indices ─────────────────────────────────────────────────────────────
WAIST_JOINTS = [12, 13, 14]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

ARM_SDK_WEIGHT_INDEX = 29
WAIST_HOLD_KP = 480.0
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5

ARM_CONTROL_MODES = ("both", "left", "right")
ARM_JOINTS: Dict[str, List[int]] = {
    "left": LEFT_ARM_JOINTS,
    "right": RIGHT_ARM_JOINTS,
}
JOINT_LABELS = ("sh_p", "sh_r", "sh_y", "elbow", "wr_r", "wr_p", "wr_y")

_SHOULDER_ORIGIN: Dict[str, np.ndarray] = {
    "left": _LEFT_SHOULDER_IN_BASE,
    "right": _RIGHT_SHOULDER_IN_BASE,
}

# ── DOF table ─────────────────────────────────────────────────────────────────
DOF_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
DOF_UNITS = ("m", "m", "m", "rad", "rad", "rad")
N_DOFS = 6

# ── Colour pairs ──────────────────────────────────────────────────────────────
C_GREEN = 1
C_YELLOW = 2
C_RED = 3
C_CYAN = 4
C_SEL = 5
C_BOLD = 6


# ── Rotation helpers ──────────────────────────────────────────────────────────

def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


_ROT_BY_AXIS = (_Rx, _Ry, _Rz)   # axis index 0/1/2 = roll/pitch/yaw


def _rpy_from_R(R: np.ndarray) -> Tuple[float, float, float]:
    """ZYX Euler angles (roll, pitch, yaw) from 3×3 rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


# ── Home EE pose (after reengage) ─────────────────────────────────────────────
# Expressed in the arm-centric frame: x=forward, y=outward, z=up.
# y=outward maps to -y_base for right arm, +y_base for left arm.
_HOME_EE_LOCAL = (0.0, 0.0, -0.4, 0.0, -math.pi / 2, 0.0)  # x,y,z,roll,pitch,yaw


def _make_home_T(arm: str) -> np.ndarray:
    """4×4 home EE pose in base_link: 0.4 m below shoulder, hand pointing forward."""
    x, y, z, roll, pitch, yaw = _HOME_EE_LOCAL
    y_sign = -1.0 if arm == "right" else 1.0
    pos = _SHOULDER_ORIGIN[arm] + np.array([x, y_sign * y, z])
    R = _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


# ── Robot infrastructure ──────────────────────────────────────────────────────

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
    def __init__(self, joints: List[int]) -> None:
        self._joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._pos: Dict[int, float] = {}
        self._ts = 0.0
        t = _resolve_lowstate_type()
        if t is None:
            raise RuntimeError("LowState_ not found in unitree_sdk2py.")
        sub = ChannelSubscriber("rt/lowstate", t)
        sub.Init(self._on_msg, 200)

    def _on_msg(self, msg: Any) -> None:
        try:
            pos = {j: float(msg.motor_state[j].q) for j in self._joints}
        except Exception:
            return
        with self._lock:
            self._pos = pos
            self._ts = time.time()

    def snapshot(self) -> Optional[Tuple[Dict[int, float], float]]:
        with self._lock:
            if not self._pos:
                return None
            return dict(self._pos), float(self._ts)


class ArmSDKPublisher:
    """Publishes upper-body targets via rt/arm_sdk."""

    def __init__(self) -> None:
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0

    def publish(
        self,
        targets: Dict[int, float],
        *,
        arm_kp: float,
        arm_kd: float,
        waist_kp: float,
        waist_kd: float,
    ) -> None:
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(targets[j])
            c.dq = 0.0
            c.tau = 0.0
            if j in WAIST_JOINTS:
                c.kp = float(waist_kp)
                c.kd = float(waist_kd)
            else:
                c.kp = float(arm_kp)
                c.kd = float(arm_kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def publish_zero_gains(self, hold: Dict[int, float]) -> None:
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


# ── Main TUI ──────────────────────────────────────────────────────────────────

class IKPoseCLI:
    def __init__(self, args: argparse.Namespace) -> None:
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.max_speed = max(0.01, float(args.speed_rad_s))
        self.arm_kp = float(args.kp)
        self.arm_kd = float(args.kd)
        self.waist_kp = float(WAIST_HOLD_KP)
        self.waist_kd = float(WAIST_HOLD_KD)
        self.waist_enabled = True   # False = waist released (kp/kd = 0)
        self.arm_control_mode = str(args.arm_control)

        # EE step sizes (per key-press)
        self.pos_step = 0.01    # metres
        self.rot_step = 0.05    # radians

        # Maximum joint angle change applied per IK solve (safety clamp)
        self.max_dq = float(args.max_dq)  # default 0.1 rad

        # ── Robot joint state mirrors ─────────────────────────────────────
        # latest_positions : most recent feedback from rt/lowstate
        # current_targets  : what is actively being sent (ramped)
        # desired_targets  : goal the ramp is tracking toward
        self.latest_positions: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}

        self.seeded = False
        self.armed = True    # False while arms are released
        self._running = True
        self.status = "Waiting for rt/lowstate…"

        # DOF navigation
        self.dof_idx = 0   # 0–5: x, y, z, roll, pitch, yaw

        # ── EE target poses — absolute hand pose in base_link frame ───────
        self.target_T: Dict[str, np.ndarray] = {
            "left": np.eye(4, dtype=np.float64),
            "right": np.eye(4, dtype=np.float64),
        }
        self._home_T: Dict[str, np.ndarray] = {
            arm: _make_home_T(arm) for arm in ("left", "right")
        }

        # ── FK / IK ───────────────────────────────────────────────────────
        # IK tolerances tuned for fast warm-start solves (~3 ms each)
        self._fk: Dict[str, ArmFK] = {
            "left": ArmFK("left", "urdf"),
            "right": ArmFK("right", "urdf"),
        }
        self._ik: Dict[str, ArmIK] = {
            "left": ArmIK("left", "dls", max_iter=10, tol_pos_m=0.005, tol_rot_rad=0.02),
            "right": ArmIK("right", "dls", max_iter=10, tol_pos_m=0.005, tol_rot_rad=0.02),
        }
        self.ik_info: Dict[str, Dict] = {
            "left": {"success": None, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0},
            "right": {"success": None, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0},
        }

        # ── Robot objects ─────────────────────────────────────────────────
        ChannelFactoryInitialize(self.domain_id, self.iface)
        self.state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self.pub = ArmSDKPublisher()
        self.robot = Robot(iface=self.iface, domain_id=self.domain_id, auto_start_sensors=True)

        self._seed_from_state()

    # ── Initialisation ────────────────────────────────────────────────────────

    def _seed_from_state(self) -> None:
        """Block up to 2 s waiting for the first lowstate packet."""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self.seeded = True
                self._sync_ee_from_joints()
                self.status = f"Connected on {self.iface}"
                return
            time.sleep(0.02)

    def _sync_ee_from_joints(self) -> None:
        """Recompute EE target poses via FK from current desired joint angles."""
        for arm, joints in ARM_JOINTS.items():
            q = np.array([self.desired_targets[j] for j in joints])
            self.target_T[arm] = self._fk[arm].compute_arm(q).copy()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _active_arms(self) -> List[str]:
        return ["left", "right"] if self.arm_control_mode == "both" else [self.arm_control_mode]

    def _display_arm(self) -> str:
        return "right" if self.arm_control_mode == "right" else "left"

    def _ee_step(self, idx: Optional[int] = None) -> float:
        i = self.dof_idx if idx is None else idx
        return self.pos_step if i < 3 else self.rot_step

    def _set_ee_step(self, value: float) -> None:
        if self.dof_idx < 3:
            self.pos_step = value
        else:
            self.rot_step = value

    def _fk_live(self, arm: str) -> np.ndarray:
        """FK from the most recent feedback joint positions."""
        joints = ARM_JOINTS[arm]
        q = np.array([self.latest_positions.get(j, self.current_targets[j]) for j in joints])
        return self._fk[arm].compute_arm(q)

    # ── IK and joint application ──────────────────────────────────────────────

    def _adjust_dof(self, delta: float) -> None:
        """Increment the selected DOF on each active arm's EE target, then solve IK."""
        for arm in self._active_arms():
            T_prev = self.target_T[arm].copy()
            T_new = T_prev.copy()

            arm_delta = delta
            if self.dof_idx == 1:
                # y: outward = +y for left arm, -y for right arm.
                arm_delta = delta if arm == "left" else -delta
            elif self.dof_idx in (3, 5):
                # roll, yaw: left arm geometry is mirrored → negate for left arm.
                arm_delta = -delta if arm == "left" else delta
            elif self.dof_idx == 4:
                # pitch: both arms use -delta (both pitched the same visual direction).
                arm_delta = -delta

            if self.dof_idx < 3:                    # Cartesian position
                T_new[self.dof_idx, 3] += arm_delta
            else:                                    # world-frame rotation
                axis = self.dof_idx - 3             # 0=roll, 1=pitch, 2=yaw
                T_new[:3, :3] = _ROT_BY_AXIS[axis](arm_delta) @ T_new[:3, :3]

            self.target_T[arm] = T_new
            ok = self._apply_ik(arm, T_prev)
            if not ok:
                self.target_T[arm] = T_prev         # rollback: EE target stays reachable

    def _apply_ik(self, arm: str, T_prev: np.ndarray) -> bool:
        """
        Solve IK for arm toward self.target_T[arm].

        On success: write max_dq-clamped joint deltas into desired_targets.
        On failure: return False (caller reverts target_T).
        """
        joints = ARM_JOINTS[arm]
        q_init = np.array([self.desired_targets[j] for j in joints])

        q_sol, info = self._ik[arm].solve(self.target_T[arm], q_init=q_init)
        self.ik_info[arm] = info

        if q_sol is None:
            return False

        # Clamp each joint's delta to ±max_dq per key-press
        delta = q_sol - q_init
        delta = np.clip(delta, -self.max_dq, self.max_dq)
        q_apply = q_init + delta

        for i, j in enumerate(joints):
            self.desired_targets[j] = float(q_apply[i])
        return True

    # ── Control loop ──────────────────────────────────────────────────────────

    def _ramp_step(self, dt: float) -> None:
        """Advance current_targets toward desired_targets at max_speed r/s."""
        step = max(1e-9, self.max_speed * dt)
        for j in UPPER_BODY_JOINTS:
            cur = float(self.current_targets[j])
            des = float(self.desired_targets[j])
            d = des - cur
            if abs(d) <= step:
                self.current_targets[j] = des
            else:
                self.current_targets[j] = cur + math.copysign(step, d)

    def tick(self) -> None:
        snap = self.state_sub.snapshot()
        if snap is not None:
            pos, _ = snap
            self.latest_positions = pos
            if not self.seeded:
                self.seeded = True
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self._sync_ee_from_joints()

        if not self.seeded or not self.armed:
            return

        now = time.monotonic()
        dt = max(1.0 / self.rate_hz, now - self._last_tick)
        self._last_tick = now

        self._ramp_step(dt)
        self.pub.publish(
            self.current_targets,
            arm_kp=self.arm_kp,
            arm_kd=self.arm_kd,
            waist_kp=self.waist_kp if self.waist_enabled else 0.0,
            waist_kd=self.waist_kd if self.waist_enabled else 0.0,
        )
        self.status = (
            f"Publishing {self.rate_hz:.0f} Hz  "
            f"ramp {self.max_speed:.3f} r/s  "
            f"max_dq {self.max_dq:.3f} rad  "
            f"arm:{self.arm_control_mode}"
        )

    # ── Drawing ───────────────────────────────────────────────────────────────

    @staticmethod
    def _addnstr(win, y: int, x: int, text: str, n: int, attr: int = 0) -> None:
        try:
            win.addnstr(y, x, text, n, attr)
        except curses.error:
            pass

    @staticmethod
    def _addstr(win, y: int, x: int, text: str, attr: int = 0) -> None:
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass

    def _cp(self, pair: int) -> int:
        return curses.color_pair(pair) if curses.has_colors() else 0

    def draw(self, win, h: int, w: int) -> None:
        if h < 18 or w < 74:
            self._addstr(win, 0, 0, f"Terminal too small ({w}×{h}). Need ≥74×18.")
            return
        try:
            self._draw_all(win, h, w)
        except curses.error:
            pass

    def _draw_all(self, win, h: int, w: int) -> None:  # noqa: C901
        row = 0

        # ── Title ─────────────────────────────────────────────────────────
        title = "6D EE IK Pose Control"
        conn_attr = self._cp(C_GREEN if self.seeded else C_RED) | curses.A_BOLD
        armed_attr = self._cp(C_GREEN if self.armed else C_RED) | curses.A_BOLD
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        self._addstr(win, row, max(0, (w - len(title)) // 2), title,
                     self._cp(C_CYAN) | curses.A_BOLD)
        conn_txt = "CONNECTED" if self.seeded else "WAITING"
        armed_txt = "ARMED" if self.armed else "RELEASED"
        self._addstr(win, row, w - 22, f"[{conn_txt}]", conn_attr)
        self._addstr(win, row, w - 12, f"[{armed_txt}]", armed_attr)
        row += 1

        # ── Parameter bar ─────────────────────────────────────────────────
        waist_lbl = "ON" if self.waist_enabled else "OFF"
        arm_txt = f"  Arm: [{self.arm_control_mode.upper()}]  (m)  Waist: [{waist_lbl}]  (w)"
        param_txt = (f"ramp {self.max_speed:.3f} r/s (s)  "
                     f"max_dq {self.max_dq:.3f} rad (d/[/])")
        self._addnstr(win, row, 0, arm_txt, w // 2)
        self._addnstr(win, row, w - len(param_txt) - 2, param_txt, w,
                      self._cp(C_YELLOW))
        row += 1

        # ── Divider ───────────────────────────────────────────────────────
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        row += 1

        # ── DOF table ─────────────────────────────────────────────────────
        disp = self._display_arm()
        T_cur = self._fk_live(disp) if self.seeded else np.eye(4)
        T_tgt = self.target_T[disp]
        cur_rpy = _rpy_from_R(T_cur[:3, :3])
        tgt_rpy = _rpy_from_R(T_tgt[:3, :3])

        hdr = (f"  {'DOF':<9}{'Live FK':<19}{'Target':<19}"
               f"{'Step':<15}  ({disp} arm)")
        self._addnstr(win, row, 0, hdr, w, curses.A_BOLD)
        row += 1

        for i in range(N_DOFS):
            sel = (i == self.dof_idx)
            mark = "▶" if sel else " "
            step = self._ee_step(i)
            unit = DOF_UNITS[i]
            if i < 3:
                cur_v = float(T_cur[i, 3])
                tgt_v = float(T_tgt[i, 3])
            else:
                cur_v = cur_rpy[i - 3]
                tgt_v = tgt_rpy[i - 3]
            line = (f"{mark} {DOF_NAMES[i]:<9}"
                    f"{cur_v:+.4f} {unit:<5}"
                    f"   {tgt_v:+.4f} {unit:<5}"
                    f"   {step:.4f} {unit}")
            attr = (self._cp(C_SEL) | curses.A_BOLD) if sel else 0
            self._addnstr(win, row, 0, line, w, attr)
            row += 1

        # ── Divider ───────────────────────────────────────────────────────
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        row += 1

        # ── IK status ─────────────────────────────────────────────────────
        for arm in ("left", "right"):
            if row >= h - 6:
                break
            if self.arm_control_mode not in ("both", arm):
                continue
            info = self.ik_info[arm]
            ok = info.get("success")
            if ok is None:
                txt = f"  IK {arm:<5}: pending"
                attr = 0
            elif ok:
                txt = (f"  IK {arm:<5}: OK  "
                       f"pos={info['error_pos_m']:.4f}m  "
                       f"rot={info['error_rot_rad']:.4f}rad  "
                       f"{info['iterations']}it")
                attr = self._cp(C_GREEN)
            else:
                txt = (f"  IK {arm:<5}: FAIL — target rolled back  "
                       f"(pos={info['error_pos_m']:.4f}m  "
                       f"rot={info['error_rot_rad']:.4f}rad)")
                attr = self._cp(C_RED)
            self._addnstr(win, row, 0, txt, w, attr)
            row += 1

        # ── Divider ───────────────────────────────────────────────────────
        if row < h - 6:
            self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
            row += 1

        # ── Joint readout ─────────────────────────────────────────────────
        if row < h - 6:
            self._addnstr(win, row, 0,
                          "  Joint targets (rad):                    "
                          "  Live feedback (rad):",
                          w, curses.A_BOLD)
            row += 1

        for arm, joints in ARM_JOINTS.items():
            if row >= h - 6:
                break
            if self.arm_control_mode not in ("both", arm):
                continue
            prefix = f"  {arm.upper():<5}: "
            lbl = "  ".join(f"{n:<5}" for n in JOINT_LABELS)
            tgt_v = "  ".join(f"{self.desired_targets[j]:+.3f}" for j in joints)
            liv_v = "  ".join(
                f"{self.latest_positions.get(j, self.current_targets[j]):+.3f}"
                for j in joints
            )
            self._addnstr(win, row, 0, prefix + lbl, w, self._cp(C_CYAN))
            row += 1
            if row < h - 6:
                half = w // 2
                self._addnstr(win, row, 0, prefix + tgt_v, half)
                self._addnstr(win, row, half, prefix + liv_v, w - half,
                              self._cp(C_YELLOW))
                row += 1

        # ── Footer ────────────────────────────────────────────────────────
        self._addstr(win, h - 5, 0, "─" * w, self._cp(C_CYAN))
        hn1 = "  ↑/↓ j/k: DOF   ← →/- +: adjust   < >: EE step   [ ]: max_dq   m: arm"
        hn2 = "  y: sync   w: waist   r: release   e: reengage   z: zero-gain   s: speed   d: max_dq   q: quit"
        self._addnstr(win, h - 4, 0, hn1, w, self._cp(C_YELLOW))
        self._addnstr(win, h - 3, 0, hn2, w, self._cp(C_YELLOW))
        self._addstr(win, h - 2, 0, "─" * w, self._cp(C_CYAN))
        st_attr = self._cp(C_GREEN if self.armed and self.seeded else C_RED)
        self._addnstr(win, h - 1, 0, f"  {self.status}", w, st_attr)

    # ── Inline prompt ─────────────────────────────────────────────────────────

    def _prompt(self, win, h: int, w: int, label: str) -> str:
        curses.curs_set(1)
        win.timeout(-1)
        buf: List[str] = []
        while True:
            win.move(h - 1, 0)
            win.clrtoeol()
            self._addnstr(win, h - 1, 0, f"{label}: {''.join(buf)}▌"[:w], w, curses.A_BOLD)
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

    # ── Key handler ───────────────────────────────────────────────────────────

    def handle_key(self, key: int, win, h: int, w: int) -> None:  # noqa: C901
        # ── Quit ──────────────────────────────────────────────────────────
        if key in (ord("q"), 27):
            self._running = False
            return

        # ── DOF navigation ────────────────────────────────────────────────
        if key in (curses.KEY_UP, ord("k")):
            self.dof_idx = max(0, self.dof_idx - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")):
            self.dof_idx = min(N_DOFS - 1, self.dof_idx + 1)
            return

        # ── EE increment / decrement ──────────────────────────────────────
        if key in (curses.KEY_LEFT, ord("-")):
            if self.seeded and self.armed:
                self._adjust_dof(-self._ee_step())
            return
        if key in (curses.KEY_RIGHT, ord("+")):
            if self.seeded and self.armed:
                self._adjust_dof(+self._ee_step())
            return

        # ── EE step size: < > ─────────────────────────────────────────────
        if key == ord("<"):
            self._set_ee_step(max(0.0001, self._ee_step() / 2.0))
            return
        if key == ord(">"):
            hi = 1.0 if self.dof_idx < 3 else math.pi
            self._set_ee_step(min(hi, self._ee_step() * 2.0))
            return

        # ── Max joint delta: [ ] ─────────────────────────────────────────
        if key == ord("["):
            self.max_dq = max(0.005, self.max_dq / 2.0)
            self.status = f"max_dq → {self.max_dq:.4f} rad"
            return
        if key == ord("]"):
            self.max_dq = min(math.pi, self.max_dq * 2.0)
            self.status = f"max_dq → {self.max_dq:.4f} rad"
            return

        # ── Arm mode ──────────────────────────────────────────────────────
        if key == ord("m"):
            idx = ARM_CONTROL_MODES.index(self.arm_control_mode)
            self.arm_control_mode = ARM_CONTROL_MODES[(idx + 1) % len(ARM_CONTROL_MODES)]
            self.status = f"Arm mode → {self.arm_control_mode}"
            return

        # ── Sync EE targets to current FK pose ────────────────────────────
        if key == ord("y"):
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
            self.current_targets = dict(self.latest_positions)
            self.desired_targets = dict(self.latest_positions)
            self._sync_ee_from_joints()
            self.status = "EE targets resynced to current hand pose"
            return

        # ── Waist toggle ──────────────────────────────────────────────────
        if key == ord("w"):
            self.waist_enabled = not self.waist_enabled
            self.status = f"Waist {'ENABLED (held)' if self.waist_enabled else 'DISABLED (free)'}"
            return

        # ── Release arms ──────────────────────────────────────────────────
        if key == ord("r"):
            try:
                self.robot.wait_for_low_state(timeout=2.0)
                self.robot.release_arms()
                self.armed = False
                self.status = "Arms released — move freely, press e to reengage"
            except Exception as exc:
                self.status = f"Release failed: {exc}"
            return

        # ── Reengage arms ─────────────────────────────────────────────────
        if key == ord("e"):
            try:
                self.robot.wait_for_low_state(timeout=2.0)
                self.robot.unrelease_arms()
                self.armed = True
                snap = self.state_sub.snapshot()
                if snap:
                    pos, _ = snap
                    self.latest_positions = dict(pos)
                # current_targets = physical position, ramp starts from here
                self.current_targets = dict(self.latest_positions)
                self.desired_targets = dict(self.latest_positions)
                # Solve IK for the home EE pose and set as desired target
                for arm in ("left", "right"):
                    self.target_T[arm] = self._home_T[arm].copy()
                    joints = ARM_JOINTS[arm]
                    q_init = np.array([self.current_targets[j] for j in joints])
                    q_sol, info = self._ik[arm].solve(self._home_T[arm], q_init=q_init)
                    self.ik_info[arm] = info
                    if q_sol is not None:
                        for i, j in enumerate(joints):
                            self.desired_targets[j] = float(q_sol[i])
                self.status = "Reengaged — homing to initial EE pose"
            except Exception as exc:
                self.status = f"Reengage failed: {exc}"
            return

        # ── Zero-gain hold ────────────────────────────────────────────────
        if key == ord("z"):
            self.pub.publish_zero_gains(self.current_targets)
            self.status = "Zero-gain hold sent on rt/arm_sdk"
            return

        # ── Ramp speed prompt ─────────────────────────────────────────────
        if key == ord("s"):
            val = self._prompt(win, h, w, f"Ramp speed r/s [{self.max_speed:.4f}]")
            try:
                self.max_speed = max(0.01, float(val))
                self.status = f"Ramp speed → {self.max_speed:.4f} r/s"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

        # ── Max joint delta prompt ─────────────────────────────────────────
        if key == ord("d"):
            val = self._prompt(win, h, w, f"Max joint delta rad [{self.max_dq:.4f}]")
            try:
                self.max_dq = max(0.005, min(math.pi, float(val)))
                self.status = f"max_dq → {self.max_dq:.4f} rad"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

    # ── Main curses loop ──────────────────────────────────────────────────────

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
            curses.init_pair(C_BOLD, curses.COLOR_BLACK, curses.COLOR_CYAN)

        curses.curs_set(0)
        stdscr.timeout(20)   # 50 fps

        self._last_tick = time.monotonic()
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
            if now - self._last_tick >= dt_target:
                self.tick()

        try:
            self.robot.release_arms()
        except Exception:
            pass


# ── CLI entry point ───────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="6D end-effector IK pose control TUI for the G1 arms"
    )
    p.add_argument("--iface", default="eth0", help="DDS network interface")
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--rate-hz", type=float, default=25.0, help="Publish rate Hz")
    p.add_argument("--speed-rad-s", type=float, default=0.2,
                   help="Joint ramp speed rad/s (how fast joints move to new target)")
    p.add_argument("--max-dq", type=float, default=0.2,
                   help="Max joint change applied per IK key-press (rad, default 0.2)")
    p.add_argument("--kp", type=float, default=DEFAULT_ARM_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_ARM_KD)
    p.add_argument(
        "--arm-control",
        choices=ARM_CONTROL_MODES,
        default="right",
        help="Which arm(s) to control (default: right)",
    )
    return p.parse_args()


def main() -> None:
    IKPoseCLI(_parse_args()).run()


if __name__ == "__main__":
    main()
