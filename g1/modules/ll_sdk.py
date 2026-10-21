"""
ll_sdk.py — Low-Level SDK for the Unitree G1 (rt/lowcmd, developer mode)
=========================================================================
Publishes to rt/lowcmd (all 29 body joints) rather than rt/arm_sdk.
Requires the MotionSwitcher to be released first — call enter_dev_mode().

Quick start:
    from ll_sdk import LLSdk
    sdk = LLSdk(iface="eth0", domain_id=0)
    sdk.enter_dev_mode()                              # once per session

    sdk.move_ll_joint({15: 0.3, 16: -0.1})           # hold others at current
    sdk.ik_move_EE([0.02, 0, 0, 0, 0, 0])   # right arm EE +2 cm X
    sdk.ik_move_EE_leg([0, 0, -0.03, 0, 0, 0], leg="left")

All move_* functions are single-shot: they send one command packet and
return.  Call them inside your own control loop at whatever rate you need.
"""
from __future__ import annotations
from hand_pose_navigation.arm_ik import ArmIK
from hand_pose_navigation.arm_fk import (
    ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, JOINT_LIMITS,
    _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
)
from dds_env import (
    default_dds_iface,
    ensure_channel_factory_initialized,
    ensure_cyclonedds_environment,
)

import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_MODULES_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.abspath(os.path.join(_MODULES_DIR, ".."))
for _p in (_ROOT_DIR, _MODULES_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
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

    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
except ImportError as exc:
    raise SystemExit("unitree_sdk2py not installed.") from exc


# ── Joint layout ──────────────────────────────────────────────────────────────
G1_NUM_MOTOR = 29

LEFT_LEG_JOINTS = [0, 1, 2, 3, 4, 5]
RIGHT_LEG_JOINTS = [6, 7, 8, 9, 10, 11]
WAIST_JOINTS = [12, 13, 14]

# Default gains from the Unitree g1_low_level_example
_DEFAULT_KP: List[float] = [
    60, 60, 60, 100, 40, 40,       # left leg
    60, 60, 60, 100, 40, 40,       # right leg
    60, 40, 40,                    # waist
    40, 40, 40, 40, 40, 40, 40,    # left arm
    40, 40, 40, 40, 40, 40, 40,    # right arm
]
_DEFAULT_KD: List[float] = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

# IK joint weights: higher = more costly to move = prefers to stay still.
# shoulder_p/r/y, elbow, wrist_r/p/y
_DEFAULT_ARM_IK_WEIGHTS = np.array([4.0, 4.0, 4.0, 2.0, 1.0, 1.0, 1.0], dtype=np.float64)
# hip_p/r/y, knee, ankle_p/r
_DEFAULT_LEG_IK_WEIGHTS = np.array([4.0, 4.0, 4.0, 2.0, 1.0, 1.0], dtype=np.float64)

# Leg joint limits [rad] from URDF / low_level_commands.py
_LEG_LIMITS: Dict[str, List[Tuple[float, float]]] = {
    "left": [
        (-2.5307, 2.8798),   # hip_pitch
        (-0.5236, 2.9671),   # hip_roll
        (-2.7576, 2.7576),   # hip_yaw
        (-0.087267, 2.8798),  # knee
        (-0.87267, 0.5236),  # ankle_pitch
        (-0.2618, 0.2618),  # ankle_roll
    ],
    "right": [
        (-2.5307, 2.8798),
        (-2.9671, 0.5236),
        (-2.7576, 2.7576),
        (-0.087267, 2.8798),
        (-0.87267, 0.5236),
        (-0.2618, 0.2618),
    ],
}

# ── Leg kinematics (URDF-exact chain, sourced from g1_29dof_with_hand_rev_1_0_pkg.urdf)
# Each entry: (xyz_in_parent, rpy_of_joint_frame, joint_axis)
_LEFT_LEG_CHAIN = [
    ([0.0, 0.064452, -0.1027], [0.0, 0.0, 0.0], [0, 1, 0]),  # hip_pitch
    ([0.0, 0.052, -0.030465], [0.0, -0.1749, 0.0], [1, 0, 0]),  # hip_roll
    ([0.025001, 0.0, -0.12412], [0.0, 0.0, 0.0], [0, 0, 1]),  # hip_yaw
    ([-0.078273, 0.0021489, -0.17734], [0.0, 0.1749, 0.0], [0, 1, 0]),  # knee
    ([0.0, -9.4445e-05, -0.30001], [0.0, 0.0, 0.0], [0, 1, 0]),  # ankle_pitch
    ([0.0, 0.0, -0.017558], [0.0, 0.0, 0.0], [1, 0, 0]),  # ankle_roll
]
_RIGHT_LEG_CHAIN = [
    ([0.0, -0.064452, -0.1027], [0.0, 0.0, 0.0], [0, 1, 0]),
    ([0.0, -0.052, -0.030465], [0.0, -0.1749, 0.0], [1, 0, 0]),
    ([0.025001, 0.0, -0.12412], [0.0, 0.0, 0.0], [0, 0, 1]),
    ([-0.078273, -0.0021489, -0.17734], [0.0, 0.1749, 0.0], [0, 1, 0]),
    ([0.0, 9.4445e-05, -0.30001], [0.0, 0.0, 0.0], [0, 1, 0]),
    ([0.0, 0.0, -0.017558], [0.0, 0.0, 0.0], [1, 0, 0]),
]
# Foot centre in ankle_roll frame (from URDF visual origin)
_FOOT_EE_OFFSET = np.array([0.026505, 0.0, -0.016425], dtype=np.float64)

_LEG_CHAINS = {"left": _LEFT_LEG_CHAIN, "right": _RIGHT_LEG_CHAIN}
_LEG_JOINTS = {"left": LEFT_LEG_JOINTS, "right": RIGHT_LEG_JOINTS}


# ── Small math helpers ────────────────────────────────────────────────────────

def _T_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    """4×4 homogeneous transform: translate xyz, rotate RPY (URDF Rz@Ry@Rx)."""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T[:3, 3] = [float(v) for v in xyz]
    return T


def _rot_axis(axis, q: float) -> np.ndarray:
    """3×3 rotation around a unit axis vector by q radians (Rodrigues)."""
    ax, ay, az = float(axis[0]), float(axis[1]), float(axis[2])
    c, s = math.cos(q), math.sin(q)
    t = 1.0 - c
    return np.array([
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c],
    ], dtype=np.float64)


def _pose_error(T_des: np.ndarray, T_cur: np.ndarray) -> np.ndarray:
    """6-D error [pos(3), rot(3)] — same convention as arm_ik.py."""
    pos_err = T_des[:3, 3] - T_cur[:3, 3]
    R_err = T_des[:3, :3] @ T_cur[:3, :3].T
    rot_err = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) * 0.5
    return np.concatenate([pos_err, rot_err])


def _clamp_q(q: np.ndarray, limits: List[Tuple[float, float]]) -> np.ndarray:
    lo = np.array([lim[0] for lim in limits], dtype=np.float64)
    hi = np.array([lim[1] for lim in limits], dtype=np.float64)
    return np.clip(q, lo, hi)


# ── Leg FK ────────────────────────────────────────────────────────────────────

class LegFK:
    """Forward kinematics for one G1 leg (6-DOF).  Returns 4×4 foot pose in base_link."""

    def __init__(self, leg: str) -> None:
        if leg not in ("left", "right"):
            raise ValueError(f"leg must be 'left' or 'right', got {leg!r}")
        self.leg = leg
        self.chain = _LEG_CHAINS[leg]

    def compute(self, q: np.ndarray) -> np.ndarray:
        """q: 6-element joint angles [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]."""
        T = np.eye(4, dtype=np.float64)
        for (xyz, rpy, axis), qi in zip(self.chain, q):
            T = T @ _T_from_xyz_rpy(xyz, rpy)
            T_rot = np.eye(4, dtype=np.float64)
            T_rot[:3, :3] = _rot_axis(axis, float(qi))
            T = T @ T_rot
        T_ee = np.eye(4, dtype=np.float64)
        T_ee[:3, 3] = _FOOT_EE_OFFSET
        return T @ T_ee


# ── Leg IK (DLS) ──────────────────────────────────────────────────────────────

class LegIK:
    """Damped least-squares IK for one G1 leg."""

    def __init__(
        self,
        leg: str,
        max_iter: int = 64,
        tol_pos_m: float = 0.005,
        tol_rot_rad: float = 0.02,
        damping: float = 0.05,
    ) -> None:
        self.leg = leg
        self.max_iter = max_iter
        self.tol_pos = tol_pos_m
        self.tol_rot = tol_rot_rad
        self.damping = damping
        self._fk = LegFK(leg)
        self._limits = _LEG_LIMITS[leg]

    def solve(
        self,
        T_des: np.ndarray,
        q_init: Optional[np.ndarray] = None,
    ) -> Tuple[Optional[np.ndarray], Dict]:
        q = _clamp_q(
            np.zeros(6, dtype=np.float64) if q_init is None else q_init.copy(),
            self._limits,
        )
        lam = self.damping
        eps = 1e-5
        for it in range(self.max_iter):
            T_cur = self._fk.compute(q)
            err = _pose_error(T_des, T_cur)
            ep = float(np.linalg.norm(err[:3]))
            er = float(np.linalg.norm(err[3:]))
            if ep < self.tol_pos and er < self.tol_rot:
                return q, {"success": True, "error_pos_m": ep, "error_rot_rad": er, "iterations": it}

            # Numerical Jacobian (6 rows × 6 cols)
            J = np.zeros((6, 6), dtype=np.float64)
            p0 = T_cur[:3, 3]
            R0 = T_cur[:3, :3]
            for col in range(6):
                q1 = q.copy()
                q1[col] += eps
                T1 = self._fk.compute(q1)
                J[:3, col] = (T1[:3, 3] - p0) / eps
                dR = T1[:3, :3] @ R0.T
                J[3:, col] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] -
                                      dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * eps)

            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), err)
            norm_dq = float(np.linalg.norm(dq))
            if norm_dq > 0.3:
                dq *= 0.3 / norm_dq
            q = _clamp_q(q + dq, self._limits)

        T_cur = self._fk.compute(q)
        err = _pose_error(T_des, T_cur)
        return None, {
            "success": False,
            "error_pos_m": float(np.linalg.norm(err[:3])),
            "error_rot_rad": float(np.linalg.norm(err[3:])),
            "iterations": self.max_iter,
        }


# ── Low-state subscriber ──────────────────────────────────────────────────────

class _LowStateReader:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: Optional[List[float]] = None
        self._mode_machine: int = 0
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._cb, 200)

    def _cb(self, msg: LowState_) -> None:
        try:
            q = [float(msg.motor_state[i].q) for i in range(G1_NUM_MOTOR)]
            mm = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        with self._lock:
            self._q = q
            self._mode_machine = mm

    def snapshot(self) -> Optional[Tuple[List[float], int]]:
        with self._lock:
            if self._q is None:
                return None
            return list(self._q), self._mode_machine

    def wait(self, timeout: float = 3.0) -> Tuple[List[float], int]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snap = self.snapshot()
            if snap is not None:
                return snap
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


# ── Rotation helpers for pose increment ──────────────────────────────────────

def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _apply_pose_increment(T: np.ndarray, inc: np.ndarray) -> np.ndarray:
    """Apply [dx,dy,dz,droll,dpitch,dyaw] increment to a 4×4 pose."""
    dx, dy, dz, dr, dp, dyw = inc
    T_new = T.copy()
    T_new[:3, 3] += np.array([dx, dy, dz])
    T_new[:3, :3] = _Rz(dyw) @ _Ry(dp) @ _Rx(dr) @ T_new[:3, :3]
    return T_new


def _mirror_inc(inc: np.ndarray) -> np.ndarray:
    """Flip y, roll, yaw — apply to right arm/leg for bilateral symmetric motion.

    Left is the reference: positive dy = both spread outward, etc.
    """
    m = inc.copy()
    m[1] = -inc[1]
    m[3] = -inc[3]
    m[5] = -inc[5]
    return m


def _solve_dls(
    fk_fn,
    T_des: np.ndarray,
    q_init: np.ndarray,
    limits,
    *,
    fixed: frozenset = frozenset(),
    weights: Optional[np.ndarray] = None,
    max_iter: int = 64,
    damping: float = 0.05,
    tol_pos: float = 0.005,
    tol_rot: float = 0.02,
) -> Tuple[Optional[np.ndarray], Dict]:
    """Weighted DLS IK with optional fixed joints.

    weights : per-joint cost vector — higher means the joint is penalised for
              moving, so the solver preferentially moves cheaper (outer) joints.
              Uses the weighted pseudoinverse:
                  dq = W⁻¹ Jᵀ (J W⁻¹ Jᵀ + λ²I)⁻¹ err
    fixed   : local joint indices held at q_init throughout the solve.
    """
    n = len(q_init)
    q = _clamp_q(q_init.copy(), limits)
    W_inv = np.ones(n, dtype=np.float64) if weights is None else 1.0 / \
        np.asarray(weights, dtype=np.float64)
    eps = 1e-5
    for it in range(max_iter):
        T_cur = fk_fn(q)
        err = _pose_error(T_des, T_cur)
        ep = float(np.linalg.norm(err[:3]))
        er = float(np.linalg.norm(err[3:]))
        if ep < tol_pos and er < tol_rot:
            return q, {"success": True, "error_pos_m": ep, "error_rot_rad": er, "iterations": it}
        J = np.zeros((6, n), dtype=np.float64)
        p0 = T_cur[:3, 3]
        R0 = T_cur[:3, :3]
        for col in range(n):
            if col in fixed:
                continue
            q1 = q.copy()
            q1[col] += eps
            T1 = fk_fn(q1)
            J[:3, col] = (T1[:3, 3] - p0) / eps
            dR = T1[:3, :3] @ R0.T
            J[3:, col] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] -
                                  dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * eps)
        # Weighted step: dq = W⁻¹ Jᵀ (J W⁻¹ Jᵀ + λ²I)⁻¹ err
        A = J * W_inv[np.newaxis, :]          # J @ diag(W_inv), shape (6, n)
        dq = W_inv * (J.T @ np.linalg.solve(A @ J.T + damping**2 * np.eye(6), err))
        norm_dq = float(np.linalg.norm(dq))
        if norm_dq > 0.3:
            dq *= 0.3 / norm_dq
        q = _clamp_q(q + dq, limits)
        for i in fixed:
            q[i] = float(q_init[i])
    T_cur = fk_fn(q)
    err = _pose_error(T_des, T_cur)
    return None, {
        "success": False,
        "error_pos_m": float(np.linalg.norm(err[:3])),
        "error_rot_rad": float(np.linalg.norm(err[3:])),
        "iterations": max_iter,
    }


# ── Main SDK ──────────────────────────────────────────────────────────────────

class LLSdk:
    """
    Low-level G1 controller via rt/lowcmd.

    Parameters
    ----------
    iface : str
        Network interface (e.g. "eth0").
    domain_id : int
        DDS domain ID (0 for real robot, 1 for sim).
    arm_ik_max_iter : int
        Max DLS iterations for arm IK.
    leg_ik_max_iter : int
        Max DLS iterations for leg IK.
    """

    def __init__(
        self,
        iface: str | None = None,
        domain_id: int = 0,
        arm_ik_max_iter: int = 24,
        leg_ik_max_iter: int = 64,
    ) -> None:
        self.iface = str(iface) if iface is not None else default_dds_iface("eth0")
        self.domain_id = int(domain_id)

        ensure_channel_factory_initialized(self.domain_id, self.iface)

        self._crc = CRC()
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0   # PR (series pitch/roll) mode

        self._state = _LowStateReader()

        self._arm_fk: Dict[str, ArmFK] = {
            "left": ArmFK("left", "urdf"),
            "right": ArmFK("right", "urdf"),
        }
        self._arm_ik: Dict[str, ArmIK] = {
            "left": ArmIK("left", "dls", max_iter=arm_ik_max_iter, tol_pos_m=0.005, tol_rot_rad=0.02),
            "right": ArmIK("right", "dls", max_iter=arm_ik_max_iter, tol_pos_m=0.005, tol_rot_rad=0.02),
        }
        self._leg_fk: Dict[str, LegFK] = {
            "left": LegFK("left"),
            "right": LegFK("right"),
        }
        self._leg_ik: Dict[str, LegIK] = {
            "left": LegIK("left", max_iter=leg_ik_max_iter),
            "right": LegIK("right", max_iter=leg_ik_max_iter),
        }

        self._msc: Optional[MotionSwitcherClient] = None

    # ── Developer mode ────────────────────────────────────────────────────────

    def enter_dev_mode(self, timeout: float = 10.0) -> None:
        """Release the MotionSwitcher so rt/lowcmd commands are accepted.

        Must be called once before any move_* function. The robot will go
        limp briefly while switching — ensure it is supported or lying down.
        """
        self._msc = MotionSwitcherClient()
        self._msc.SetTimeout(5.0)
        self._msc.Init()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            code, data = self._msc.CheckMode()
            if code != 0:
                raise RuntimeError(f"MotionSwitcherClient.CheckMode() failed: code={code}")
            if not (data or {}).get("name"):
                return
            self._msc.ReleaseMode()
            time.sleep(0.5)
        raise TimeoutError("Could not release MotionSwitcher within timeout.")

    # ── Core publisher ────────────────────────────────────────────────────────

    def _publish(
        self,
        q_full: List[float],
        mode_machine: int,
        *,
        kp: Optional[List[float]] = None,
        kd: Optional[List[float]] = None,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> None:
        kp = kp or _DEFAULT_KP
        kd = kd or _DEFAULT_KD
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine)
        for i in range(G1_NUM_MOTOR):
            mc = self._cmd.motor_cmd[i]
            mc.mode = 1
            mc.q = float(q_full[i])
            mc.dq = float(dq)
            mc.tau = float(tau)
            mc.kp = float(kp[i])
            mc.kd = float(kd[i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def _ramp_publish(
        self,
        q_start: List[float],
        q_target: List[float],
        mode_machine: int,
        *,
        speed_rad_s: float,
        rate_hz: float,
        kp: Optional[List[float]] = None,
        kd: Optional[List[float]] = None,
        dq: float = 0.0,
        tau: float = 0.0,
    ) -> Dict[str, float | int]:
        speed = float(speed_rad_s)
        rate = max(1.0, float(rate_hz))
        if speed <= 0.0:
            self._publish(q_target, mode_machine, kp=kp, kd=kd, dq=dq, tau=tau)
            return {"steps": 1, "speed_rad_s": speed, "rate_hz": rate}

        max_delta = max(abs(float(dst) - float(src)) for src, dst in zip(q_start, q_target))
        steps = max(1, int(math.ceil(max_delta / max(1e-6, speed / rate))))
        if steps <= 1:
            self._publish(q_target, mode_machine, kp=kp, kd=kd, dq=dq, tau=tau)
            return {"steps": 1, "speed_rad_s": speed, "rate_hz": rate}

        dt = 1.0 / rate
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            q_step = [
                float(src) + alpha * (float(dst) - float(src))
                for src, dst in zip(q_start, q_target)
            ]
            self._publish(q_step, mode_machine, kp=kp, kd=kd, dq=dq, tau=tau)
            if step_idx < steps:
                time.sleep(dt)
        return {"steps": steps, "speed_rad_s": speed, "rate_hz": rate}

    # ── Public API ────────────────────────────────────────────────────────────

    def move_ll_joint(
        self,
        targets: Dict[int, float],
        *,
        dq: float = 0.0,
        kp: Optional[Dict[int, float]] = None,
        kd: Optional[Dict[int, float]] = None,
        tau: float = 0.0,
        ramp_speed_rad_s: float = 0.0,
        ramp_rate_hz: float = 50.0,
        timeout: float = 3.0,
    ) -> None:
        """Send one rt/lowcmd packet with the specified joint targets.

        Parameters
        ----------
        targets : dict {joint_index: q_rad}
            Joints to move.  All unspecified joints are held at their current
            lowstate position — never left at zero.
        kp, kd : dict {joint_index: gain}, optional
            Per-joint gain overrides; default gains are used for unspecified joints.
        dq, tau : float
            Applied uniformly to every joint in this packet.
        """
        q_full, mm = self._state.wait(timeout)
        q_start = list(q_full)
        for idx, val in targets.items():
            q_full[int(idx)] = float(val)

        kp_list = list(_DEFAULT_KP)
        kd_list = list(_DEFAULT_KD)
        if kp:
            for idx, val in kp.items():
                kp_list[int(idx)] = float(val)
        if kd:
            for idx, val in kd.items():
                kd_list[int(idx)] = float(val)

        self._ramp_publish(
            q_start,
            q_full,
            mm,
            speed_rad_s=float(ramp_speed_rad_s),
            rate_hz=float(ramp_rate_hz),
            kp=kp_list,
            kd=kd_list,
            dq=dq,
            tau=tau,
        )

    def ik_move_EE(
        self,
        pose_increment: "np.ndarray | List[float]",
        arm: str = "right",
        *,
        mirror: bool = True,
        fixed_joints: "Optional[List[int]]" = None,
        joint_weights: "Optional[np.ndarray]" = None,
        max_dq: float = 0.2,
        ramp_speed_rad_s: float = 0.35,
        ramp_rate_hz: float = 50.0,
        timeout: float = 3.0,
    ) -> Dict:
        """Move one (or both) arm end-effector(s) by a Cartesian increment.

        Parameters
        ----------
        pose_increment : array-like [dx, dy, dz, droll, dpitch, dyaw]
            Increment in base_link frame (metres / radians).
        arm : "left" | "right" | "both"
        mirror : bool
            When arm="both", flip y/roll/yaw for the right arm so the motion
            is bilaterally symmetric (left is the reference). Default True.
        fixed_joints : list of int, optional
            Local joint indices (0-6) to hold fixed during the IK solve.
            e.g. [4, 5, 6] locks all three wrist joints.
        joint_weights : array-like of shape (7,), optional
            Per-joint cost weights.  Higher weight → joint moves less.
            Default: [4,4,4,2,1,1,1] — shoulder costly, wrist cheap.
            Pass np.ones(7) for uniform (classic DLS) behaviour.
        max_dq : float
            Max joint angle change per call (rad) — safety clamp.

        Returns
        -------
        dict with keys: success, error_pos_m, error_rot_rad, iterations
        (for "both" arms, returns the result for the last arm solved)
        """
        inc = np.asarray(pose_increment, dtype=np.float64)
        arms = ["left", "right"] if arm == "both" else [str(arm)]
        fixed = frozenset(int(i) for i in fixed_joints) if fixed_joints else frozenset()
        w = _DEFAULT_ARM_IK_WEIGHTS if joint_weights is None else np.asarray(
            joint_weights, dtype=np.float64)

        q_full, mm = self._state.wait(timeout)
        q_start = list(q_full)
        info: Dict = {"success": False, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0}

        for a in arms:
            arm_inc = _mirror_inc(inc) if (mirror and arm == "both" and a == "right") else inc
            joints = LEFT_ARM_JOINTS if a == "left" else RIGHT_ARM_JOINTS
            limits = JOINT_LIMITS[a]
            q_arm = np.array([q_full[j] for j in joints], dtype=np.float64)
            T_cur = self._arm_fk[a].compute_arm(q_arm)
            T_des = _apply_pose_increment(T_cur, arm_inc)

            q_sol, info = _solve_dls(
                self._arm_fk[a].compute_arm, T_des, q_arm, limits,
                fixed=fixed, weights=w,
                max_iter=self._arm_ik[a].max_iter,
                damping=self._arm_ik[a].damping,
                tol_pos=self._arm_ik[a].tol_pos_m,
                tol_rot=self._arm_ik[a].tol_rot_rad,
            )
            if q_sol is None:
                continue

            delta = np.clip(q_sol - q_arm, -max_dq, max_dq)
            q_new = np.clip(q_arm + delta,
                            np.array([limit[0] for limit in limits]),
                            np.array([limit[1] for limit in limits]))
            for i, j in enumerate(joints):
                q_full[j] = float(q_new[i])

        info["ramp"] = self._ramp_publish(
            q_start,
            q_full,
            mm,
            speed_rad_s=float(ramp_speed_rad_s),
            rate_hz=float(ramp_rate_hz),
        )
        return info

    def ik_move_EE_leg(
        self,
        pose_increment: "np.ndarray | List[float]",
        leg: str = "right",
        *,
        mirror: bool = True,
        fixed_joints: "Optional[List[int]]" = None,
        joint_weights: "Optional[np.ndarray]" = None,
        max_dq: float = 0.15,
        ramp_speed_rad_s: float = 0.35,
        ramp_rate_hz: float = 50.0,
        timeout: float = 3.0,
    ) -> Dict:
        """Move one (or both) leg foot end-effector(s) by a Cartesian increment.

        Parameters
        ----------
        pose_increment : array-like [dx, dy, dz, droll, dpitch, dyaw]
            Increment in base_link frame (metres / radians).
        leg : "left" | "right" | "both"
        mirror : bool
            When leg="both", flip y/roll/yaw for the right leg so the motion
            is bilaterally symmetric (left is the reference). Default True.
        fixed_joints : list of int, optional
            Local joint indices (0-5) to hold fixed during the IK solve.
            e.g. [4, 5] locks ankle pitch and roll.
        joint_weights : array-like of shape (6,), optional
            Per-joint cost weights.  Higher weight → joint moves less.
            Default: [4,4,4,2,1,1] — hip costly, ankle cheap.
            Pass np.ones(6) for uniform (classic DLS) behaviour.
        max_dq : float
            Max joint angle change per call (rad) — safety clamp.

        Returns
        -------
        dict with keys: success, error_pos_m, error_rot_rad, iterations

        WARNING
        -------
        Leg IK via rt/lowcmd bypasses the loco controller balance stack.
        Only use this when the robot is supported externally or seated.
        """
        inc = np.asarray(pose_increment, dtype=np.float64)
        legs = ["left", "right"] if leg == "both" else [str(leg)]
        fixed = frozenset(int(i) for i in fixed_joints) if fixed_joints else frozenset()
        w = _DEFAULT_LEG_IK_WEIGHTS if joint_weights is None else np.asarray(
            joint_weights, dtype=np.float64)

        q_full, mm = self._state.wait(timeout)
        q_start = list(q_full)
        info: Dict = {"success": False, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0}

        for lg in legs:
            leg_inc = _mirror_inc(inc) if (mirror and leg == "both" and lg == "right") else inc
            joints = _LEG_JOINTS[lg]
            limits = _LEG_LIMITS[lg]
            q_leg = np.array([q_full[j] for j in joints], dtype=np.float64)
            T_cur = self._leg_fk[lg].compute(q_leg)
            T_des = _apply_pose_increment(T_cur, leg_inc)

            q_sol, info = _solve_dls(
                self._leg_fk[lg].compute, T_des, q_leg, limits,
                fixed=fixed, weights=w,
                max_iter=self._leg_ik[lg].max_iter,
                damping=self._leg_ik[lg].damping,
                tol_pos=self._leg_ik[lg].tol_pos,
                tol_rot=self._leg_ik[lg].tol_rot,
            )
            if q_sol is None:
                continue

            delta = np.clip(q_sol - q_leg, -max_dq, max_dq)
            q_new = np.clip(q_leg + delta,
                            np.array([limit[0] for limit in limits]),
                            np.array([limit[1] for limit in limits]))
            for i, j in enumerate(joints):
                q_full[j] = float(q_new[i])

        info["ramp"] = self._ramp_publish(
            q_start,
            q_full,
            mm,
            speed_rad_s=float(ramp_speed_rad_s),
            rate_hz=float(ramp_rate_hz),
        )
        return info

    # ── Convenience helpers ───────────────────────────────────────────────────

    def get_joint_positions(self, timeout: float = 3.0) -> List[float]:
        """Return the current 29-element joint position vector from lowstate."""
        q, _ = self._state.wait(timeout)
        return q

    def get_arm_ee_pose(self, arm: str = "right", timeout: float = 3.0) -> np.ndarray:
        """Return the current 4×4 EE pose in base_link for the given arm."""
        q_full, _ = self._state.wait(timeout)
        joints = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
        q_arm = np.array([q_full[j] for j in joints])
        return self._arm_fk[arm].compute_arm(q_arm)

    def get_foot_pose(self, leg: str = "right", timeout: float = 3.0) -> np.ndarray:
        """Return the current 4×4 foot pose in base_link for the given leg."""
        q_full, _ = self._state.wait(timeout)
        joints = _LEG_JOINTS[leg]
        q_leg = np.array([q_full[j] for j in joints])
        return self._leg_fk[leg].compute(q_leg)
