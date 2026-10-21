"""
arm_sdk.py — Arm SDK wrapper for the Unitree G1 (rt/arm_sdk)
=============================================================
Controls the upper body (waist + arms, joints 12-28) via rt/arm_sdk.
Does NOT require developer mode or MotionSwitcher release.

Quick start:
    from arm_sdk import ArmSdk
    sdk = ArmSdk(iface="eth0", domain_id=0)

    # Direct joint command (upper body only):
    sdk.move_joint({15: 0.3, 16: -0.1})

    # Cartesian EE increment (right arm, +2 cm along X):
    info = sdk.ik_move_EE([0.02, 0, 0, 0, 0, 0], arm="right")

    # Resync EE targets to current measured pose (call after physical disturbance):
    sdk.resync()
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_NUMPY_COMPAT_ALIASES = {
    "bool": bool,
    "complex": complex,
    "float": float,
    "int": int,
    "object": object,
    "str": str,
    "typeDict": np.sctypeDict,
}
for _name, _value in _NUMPY_COMPAT_ALIASES.items():
    if _name not in np.__dict__:
        setattr(np, _name, _value)

# ── Path bootstrap ────────────────────────────────────────────────────────────
_MODULES_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.abspath(os.path.join(_MODULES_DIR, ".."))
for _p in (_ROOT_DIR, _MODULES_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from hand_pose_navigation_copy.arm_ik import ArmIK
    from hand_pose_navigation_copy.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, JOINT_LIMITS,
    )
except ModuleNotFoundError:
    from hand_pose_navigation.arm_ik import ArmIK
    from hand_pose_navigation.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, JOINT_LIMITS,
    )

from dds_env import (
    default_dds_iface,
    ensure_channel_factory_initialized,
    ensure_cyclonedds_environment,
)

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber,
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


# ── Constants ─────────────────────────────────────────────────────────────────
WAIST_JOINTS = [12, 13, 14]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_SDK_WEIGHT_IDX = 29

_ARM_JOINTS: Dict[str, List[int]] = {
    "left": LEFT_ARM_JOINTS,
    "right": RIGHT_ARM_JOINTS,
}

DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
DEFAULT_WAIST_KP = 200.0
DEFAULT_WAIST_KD = 12.0

# IK joint weights: higher = more costly to move = prefers to stay still.
# Outer joints (wrist) are cheap; inner joints (shoulder) are expensive.
# shoulder_p/r/y, elbow, wrist_r/p/y
_DEFAULT_ARM_IK_WEIGHTS = np.array([4.0, 4.0, 4.0, 2.0, 1.0, 1.0, 1.0], dtype=np.float64)
_POSITION_IK_TOL_M = 0.005
_POSITION_IK_AXIS_TOL_M = 0.006
_POSITION_IK_SOFT_LIMIT_M = 0.040
_POSITION_IK_ACTIVE_IDXS = (0, 1, 2, 3)


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


def _apply_pose_increment(T: np.ndarray, inc: np.ndarray) -> np.ndarray:
    """Apply [dx, dy, dz, droll, dpitch, dyaw] to a 4×4 pose."""
    T_new = T.copy()
    T_new[:3, 3] += inc[:3]
    T_new[:3, :3] = _Rz(inc[5]) @ _Ry(inc[4]) @ _Rx(inc[3]) @ T_new[:3, :3]
    return T_new


def _mirror_inc(inc: np.ndarray) -> np.ndarray:
    """Flip y, roll, yaw — apply to right arm for bilateral symmetric motion.

    Follows the ik_pose_cli_v3 convention where left is the reference arm:
      y    → negated  (right arm moves outward when left moves outward)
      roll → negated  (wrists mirror each other)
      yaw  → negated  (wrists mirror each other)
    """
    m = inc.copy()
    m[1] = -inc[1]
    m[3] = -inc[3]
    m[5] = -inc[5]
    return m


def _clamp_q(q: np.ndarray, limits) -> np.ndarray:
    lo = np.array([limit[0] for limit in limits], dtype=np.float64)
    hi = np.array([limit[1] for limit in limits], dtype=np.float64)
    return np.clip(q, lo, hi)


def _pose_error(T_des: np.ndarray, T_cur: np.ndarray) -> np.ndarray:
    pos_err = T_des[:3, 3] - T_cur[:3, 3]
    R_err = T_des[:3, :3] @ T_cur[:3, :3].T
    rot_err = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) * 0.5
    return np.concatenate([pos_err, rot_err])


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


def _solve_position_shoulder_elbow(
    fk_fn,
    T_des: np.ndarray,
    q_init: np.ndarray,
    limits,
    *,
    selected_axis: Optional[int] = None,
    max_iter: int = 64,
    damping: float = 0.05,
) -> Tuple[Optional[np.ndarray], Dict]:
    """Position-only DLS matching ik_pose_cli_v3's free-orientation mode."""
    q = _clamp_q(q_init.copy(), limits)
    eps = 1e-5
    best_q = q.copy()
    best_err_pos = float("inf")
    best_axis_err = float("inf")

    for iteration in range(max_iter):
        T_cur = fk_fn(q)
        pos_err = T_des[:3, 3] - T_cur[:3, 3]
        err_pos = float(np.linalg.norm(pos_err))
        axis_err = abs(float(pos_err[selected_axis])) if selected_axis is not None else err_pos
        if (err_pos, axis_err) < (best_err_pos, best_axis_err):
            best_q = q.copy()
            best_err_pos = err_pos
            best_axis_err = axis_err
        if err_pos < _POSITION_IK_TOL_M:
            return q, {
                "success": True,
                "error_pos_m": err_pos,
                "error_rot_rad": 0.0,
                "iterations": iteration,
                "mode": "pos_shoulder_elbow",
            }

        J = np.zeros((3, len(_POSITION_IK_ACTIVE_IDXS)), dtype=np.float64)
        p0 = T_cur[:3, 3]
        for col, idx in enumerate(_POSITION_IK_ACTIVE_IDXS):
            q1 = q.copy()
            q1[idx] += eps
            T1 = fk_fn(q1)
            J[:, col] = (T1[:3, 3] - p0) / eps

        dq_active = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), pos_err)
        norm_dq = float(np.linalg.norm(dq_active))
        if norm_dq > 0.3:
            dq_active *= 0.3 / norm_dq

        q_next = q.copy()
        for col, idx in enumerate(_POSITION_IK_ACTIVE_IDXS):
            q_next[idx] += dq_active[col]
        q = _clamp_q(q_next, limits)
        q[4:] = q_init[4:]

    T_cur = fk_fn(best_q)
    err_pos = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
    axis_err = abs(float(T_des[selected_axis, 3] - T_cur[selected_axis, 3])
                   ) if selected_axis is not None else err_pos
    if (
        selected_axis is not None
        and axis_err < _POSITION_IK_AXIS_TOL_M
        and err_pos < _POSITION_IK_SOFT_LIMIT_M
    ):
        return best_q, {
            "success": True,
            "error_pos_m": err_pos,
            "error_rot_rad": 0.0,
            "iterations": max_iter,
            "mode": "pos_axis_clamped",
            "axis_error_m": axis_err,
        }
    return None, {
        "success": False,
        "error_pos_m": err_pos,
        "error_rot_rad": 0.0,
        "iterations": max_iter,
        "mode": "pos_shoulder_elbow",
    }


# ── Low-state subscriber (upper body only) ────────────────────────────────────

class _UpperBodyStateReader:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: Optional[Dict[int, float]] = None
        try:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as _LS
        except ImportError:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as _LS
        sub = ChannelSubscriber("rt/lowstate", _LS)
        sub.Init(self._cb, 200)

    def _cb(self, msg) -> None:
        try:
            q = {j: float(msg.motor_state[j].q) for j in UPPER_BODY_JOINTS}
        except Exception:
            return
        with self._lock:
            self._q = q

    def snapshot(self) -> Optional[Dict[int, float]]:
        with self._lock:
            return dict(self._q) if self._q is not None else None

    def wait(self, timeout: float = 3.0) -> Dict[int, float]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snap = self.snapshot()
            if snap is not None:
                return snap
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


# ── rt/arm_sdk publisher ──────────────────────────────────────────────────────

class _ArmSdkPublisher:
    def __init__(self) -> None:
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_IDX].q = 1.0

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
            mc = self._cmd.motor_cmd[j]
            mc.mode = 1
            mc.q = float(targets[j])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(waist_kp if j in WAIST_JOINTS else arm_kp)
            mc.kd = float(waist_kd if j in WAIST_JOINTS else arm_kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


# ── Main SDK ──────────────────────────────────────────────────────────────────

class ArmSdk:
    """
    Upper-body controller via rt/arm_sdk.

    Parameters
    ----------
    iface : str
        Network interface (e.g. "eth0").
    domain_id : int
        DDS domain ID.
    arm_kp, arm_kd : float
        Servo gains for arm joints.
    waist_kp, waist_kd : float
        Servo gains for waist joints.
    arm_ik_max_iter : int
        DLS IK iteration limit.
    """

    def __init__(
        self,
        iface: str | None = None,
        domain_id: int = 0,
        arm_kp: float = DEFAULT_ARM_KP,
        arm_kd: float = DEFAULT_ARM_KD,
        waist_kp: float = DEFAULT_WAIST_KP,
        waist_kd: float = DEFAULT_WAIST_KD,
        arm_ik_max_iter: int = 24,
    ) -> None:
        iface = str(iface) if iface is not None else default_dds_iface("eth0")
        self.arm_kp = float(arm_kp)
        self.arm_kd = float(arm_kd)
        self.waist_kp = float(waist_kp)
        self.waist_kd = float(waist_kd)

        ensure_channel_factory_initialized(int(domain_id), str(iface))

        self._state = _UpperBodyStateReader()
        self._pub = _ArmSdkPublisher()

        self._fk: Dict[str, ArmFK] = {
            "left": ArmFK("left", "urdf"),
            "right": ArmFK("right", "urdf"),
        }
        self._ik: Dict[str, ArmIK] = {
            "left": ArmIK("left", "dls", max_iter=arm_ik_max_iter, tol_pos_m=0.005, tol_rot_rad=0.02),
            "right": ArmIK("right", "dls", max_iter=arm_ik_max_iter, tol_pos_m=0.005, tol_rot_rad=0.02),
        }

        # Last commanded joint targets — seeded on first use (needed by move_joint).
        self._desired: Optional[Dict[int, float]] = None

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _seed(self, timeout: float = 3.0) -> None:
        """Seed desired joint targets from current lowstate if not yet done."""
        if self._desired is not None:
            return
        q = self._state.wait(timeout)
        self._desired = dict(q)

    def _publish(self) -> None:
        assert self._desired is not None
        self._pub.publish(
            self._desired,
            arm_kp=self.arm_kp,
            arm_kd=self.arm_kd,
            waist_kp=self.waist_kp,
            waist_kd=self.waist_kd,
        )

    def _ramp_to_desired(
        self,
        start: Dict[int, float],
        target: Dict[int, float],
        *,
        speed_rad_s: float,
        rate_hz: float,
    ) -> Dict[str, float | int]:
        """Ramp desired joint targets before publishing the final target."""
        speed = float(speed_rad_s)
        rate = max(1.0, float(rate_hz))
        if speed <= 0.0:
            self._desired = dict(target)
            self._publish()
            return {"steps": 1, "speed_rad_s": speed, "rate_hz": rate}

        max_delta = max(
            abs(float(target.get(j, start.get(j, 0.0))) - float(start.get(j, 0.0)))
            for j in target
        )
        steps = max(1, int(math.ceil(max_delta / max(1e-6, speed / rate))))
        if steps <= 1:
            self._desired = dict(target)
            self._publish()
            return {"steps": 1, "speed_rad_s": speed, "rate_hz": rate}

        dt = 1.0 / rate
        keys = list(target)
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            self._desired = {
                j: float(start.get(j, target[j])) + alpha * (float(target[j]) - float(start.get(j, target[j])))
                for j in keys
            }
            self._publish()
            if step_idx < steps:
                time.sleep(dt)
        self._desired = dict(target)
        return {"steps": steps, "speed_rad_s": speed, "rate_hz": rate}

    # ── Public API ────────────────────────────────────────────────────────────

    def resync(self, timeout: float = 3.0) -> None:
        """Resync desired joint targets to the current measured state.

        Call this after a physical disturbance or after loading a pose, so that
        subsequent move_joint calls hold the correct positions for unset joints.
        """
        q = self._state.wait(timeout)
        self._desired = dict(q)

    def move_joint(
        self,
        targets: Dict[int, float],
        *,
        timeout: float = 3.0,
    ) -> None:
        """Send one rt/arm_sdk packet with explicit joint targets.

        Parameters
        ----------
        targets : dict {joint_index: q_rad}
            Upper-body joints to command (indices 12-28).
            Unspecified joints are held at the last desired position.
        """
        self._seed(timeout)
        assert self._desired is not None
        for idx, val in targets.items():
            self._desired[int(idx)] = float(val)
        # Resync EE targets for any arm whose joints changed.
        for arm, joints in _ARM_JOINTS.items():
            if any(j in targets for j in joints):
                q_arm = self._q_arm(arm)
                self._target_T[arm] = self._fk[arm].compute_arm(q_arm).copy()
        self._publish()

    def ik_move_EE(
        self,
        pose_increment: "np.ndarray | List[float]",
        arm: str = "right",
        *,
        mirror: bool = True,
        fixed_joints: "Optional[List[int]]" = None,
        joint_weights: "Optional[np.ndarray]" = None,
        position_only: bool = False,
        selected_axis: "Optional[int]" = None,
        max_dq: float = 0.2,
        ramp_speed_rad_s: float = 0.35,
        ramp_rate_hz: float = 50.0,
        timeout: float = 3.0,
    ) -> Dict:
        """Move one (or both) arm end-effector(s) by a Cartesian increment.

        Warm-started from the last commanded joint state so repeated small
        increments accumulate correctly even if lowstate lags the command.

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
            Default: [4,4,4,2,1,1,1] — shoulder/hip costly, wrist/ankle cheap.
            Pass np.ones(7) for uniform (classic DLS) behaviour.
        position_only : bool
            If true, solve position with wrist joints held fixed and do not
            constrain end-effector orientation. This matches ik_pose_cli_v3's
            orientation-free x/y/z move mode.
        selected_axis : int, optional
            Position axis index used for soft axis-clamped success near limits.
        max_dq : float
            Per-joint safety clamp on the IK step (rad).
        ramp_speed_rad_s : float
            Maximum joint-target ramp speed after IK succeeds. Set <= 0 for a
            single publish, but the notebook and VLA paths should leave this on.
        ramp_rate_hz : float
            Publish rate used while ramping to the solved joint target.

        Returns
        -------
        dict with keys: success, error_pos_m, error_rot_rad, iterations.
        For "both" arms, returns the result of the last arm solved.
        """
        inc = np.asarray(pose_increment, dtype=np.float64)
        if inc.shape != (6,):
            raise ValueError(f"pose_increment must have 6 elements, got {inc.shape}")

        arms = ["left", "right"] if arm == "both" else [str(arm)]
        fixed = frozenset(int(i) for i in fixed_joints) if fixed_joints else frozenset()
        w = _DEFAULT_ARM_IK_WEIGHTS if joint_weights is None else np.asarray(
            joint_weights, dtype=np.float64)

        self._seed(timeout)
        assert self._desired is not None
        start_desired = dict(self._desired)
        target_desired = dict(self._desired)
        q_state = self._state.wait(timeout)  # fresh measured state for IK init

        info: Dict = {"success": False, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0}

        for a in arms:
            arm_inc = _mirror_inc(inc) if (mirror and arm == "both" and a == "right") else inc
            joints = _ARM_JOINTS[a]
            limits = JOINT_LIMITS[a]
            q_arm = np.array([q_state[j] for j in joints])
            T_cur = self._fk[a].compute_arm(q_arm)

            scales = (1.0, 0.5, 0.25, 0.1)
            solved = False
            for scale in scales:
                T_scaled = _apply_pose_increment(T_cur, arm_inc * scale)
                if position_only:
                    q_sol, info = _solve_position_shoulder_elbow(
                        self._fk[a].compute_arm,
                        T_scaled,
                        q_arm,
                        limits,
                        selected_axis=selected_axis,
                        max_iter=self._ik[a].max_iter,
                        damping=self._ik[a].damping,
                    )
                else:
                    q_sol, info = _solve_dls(
                        self._fk[a].compute_arm, T_scaled, q_arm, limits,
                        fixed=fixed, weights=w,
                        max_iter=self._ik[a].max_iter,
                        damping=self._ik[a].damping,
                        tol_pos=self._ik[a].tol_pos_m,
                        tol_rot=self._ik[a].tol_rot_rad,
                    )
                if q_sol is None:
                    continue

                delta = np.clip(q_sol - q_arm, -max_dq, max_dq)
                q_new = np.clip(q_arm + delta,
                                np.array([limit[0] for limit in limits]),
                                np.array([limit[1] for limit in limits]))
                for i, j in enumerate(joints):
                    target_desired[j] = float(q_new[i])
                solved = True
                break

        self._desired = dict(target_desired)
        ramp_info = self._ramp_to_desired(
            start_desired,
            target_desired,
            speed_rad_s=float(ramp_speed_rad_s),
            rate_hz=float(ramp_rate_hz),
        )
        info["ramp"] = ramp_info
        return info

    def get_ee_pose(self, arm: str = "right", timeout: float = 3.0) -> np.ndarray:
        """Return the current measured 4×4 EE pose in base_link."""
        q_state = self._state.wait(timeout)
        q_arm = np.array([q_state[j] for j in _ARM_JOINTS[arm]])
        return self._fk[arm].compute_arm(q_arm)

    def get_joint_targets(self) -> Dict[int, float]:
        """Return the current desired upper-body joint targets."""
        self._seed()
        assert self._desired is not None
        return dict(self._desired)
