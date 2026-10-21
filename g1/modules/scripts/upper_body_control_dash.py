#!/usr/bin/env python3
"""
Upper-body Dash control panel for the Unitree G1.

Wraps the 6D EE IK control in a web UI with:
  - Release / Unrelease arms (toggle)
  - Arm selection (right / left / both)
  - Orientation-lock toggle
  - Extend arm forward (preset motion)
  - +/- nudge buttons for x, y, z, roll, pitch, yaw
"""
from __future__ import annotations
from hand_pose_navigation_copy.arm_fk import JOINT_LIMITS
from hand_pose_navigation_copy.arm_ik import ArmIK
from hand_pose_navigation_copy.arm_fk import (
    ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
    _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
)
from sdk_client import Robot
from dds_env import ensure_cyclonedds_environment

import argparse
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import dash
    import dash_bootstrap_components as dbc
    from dash import Input, Output, State, dcc, html
except ImportError as exc:
    raise SystemExit(
        "Dash and dash-bootstrap-components are required.\n"
        "  pip install dash dash-bootstrap-components"
    ) from exc

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODULES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
ROOT_DIR = os.path.abspath(os.path.join(MODULES_DIR, ".."))
WBC_DIR = os.path.join(ROOT_DIR, "WBC")
for _p in (ROOT_DIR, MODULES_DIR, WBC_DIR):
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
    raise SystemExit("unitree_sdk2py not installed.") from exc


# ── Constants ─────────────────────────────────────────────────────────────────
WAIST_JOINTS = [12, 13, 14]
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
ARM_SDK_WEIGHT_INDEX = 29
WAIST_HOLD_KP = 480.0
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
DEFAULT_WAIST_PR_KP = 200.0

ARM_JOINTS: Dict[str, List[int]] = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}
DOF_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
DOF_UNITS = ("m", "m", "m", "rad", "rad", "rad")
N_DOFS = 6

SHOULDER_ELBOW_IDXS = (0, 1, 2, 3)
POSITION_IK_TOL_M = 0.005
POSITION_IK_AXIS_TOL_M = 0.006
POSITION_IK_SOFT_LIMIT_M = 0.040

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


_ROT_BY_AXIS = (_Rx, _Ry, _Rz)


def _rpy_from_R(R: np.ndarray) -> Tuple[float, float, float]:
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


def _clamp_q(q: np.ndarray, arm: str) -> np.ndarray:
    limits = JOINT_LIMITS[arm]
    lo = np.array([lim[0] for lim in limits], dtype=np.float64)
    hi = np.array([lim[1] for lim in limits], dtype=np.float64)
    return np.clip(q, lo, hi)

# ── DDS subscriber (mirrors ik_pose_cli_v3) ───────────────────────────────────


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
        t = _resolve_lowstate_type()
        if t is None:
            raise RuntimeError("LowState_ IDL type not found.")
        sub = ChannelSubscriber("rt/lowstate", t)
        sub.Init(self._on_msg, 200)

    def _on_msg(self, msg: Any) -> None:
        try:
            pos = {j: float(msg.motor_state[j].q) for j in self._joints}
        except Exception:
            return
        with self._lock:
            self._pos = pos

    def snapshot(self) -> Optional[Dict[int, float]]:
        with self._lock:
            return dict(self._pos) if self._pos else None


class ArmSDKPublisher:
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
        waist_pr_kp: float,
        waist_y_kp: float,
        waist_kd: float,
    ) -> None:
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(targets[j])
            c.dq = 0.0
            c.tau = 0.0
            if j in (12, 13):
                c.kp = float(waist_pr_kp)
                c.kd = float(waist_kd)
            elif j == 14:
                c.kp = float(waist_y_kp)
                c.kd = float(waist_kd)
            else:
                c.kp = float(arm_kp)
                c.kd = float(arm_kd)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


# ── Arm Controller ────────────────────────────────────────────────────────────
class ArmController:
    """Thread-safe controller running a background publish loop."""

    def __init__(
        self,
        iface: str = "eth0",
        domain_id: int = 0,
        rate_hz: float = 25.0,
        max_speed: float = 0.2,
        arm_kp: float = DEFAULT_ARM_KP,
        arm_kd: float = DEFAULT_ARM_KD,
        waist_pr_kp: float = DEFAULT_WAIST_PR_KP,
        pos_step: float = 0.02,
        rot_step: float = 0.05,
        max_dq: float = 0.2,
    ) -> None:
        self.iface = iface
        self.rate_hz = rate_hz
        self.max_speed = max_speed
        self.arm_kp = arm_kp
        self.arm_kd = arm_kd
        self.waist_pr_kp = waist_pr_kp
        self.waist_y_kp = WAIST_HOLD_KP
        self.waist_kd = WAIST_HOLD_KD
        self.pos_step = pos_step
        self.rot_step = rot_step
        self.max_dq = max_dq

        self._lock = threading.RLock()
        self.arm_control_mode = "right"
        self.orient_stiff = True
        self.armed = True
        self.seeded = False
        self.status = "Waiting for rt/lowstate…"

        self.latest_positions: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}

        self.target_T: Dict[str, np.ndarray] = {
            "left": np.eye(4, dtype=np.float64),
            "right": np.eye(4, dtype=np.float64),
        }
        self.ik_info: Dict[str, Dict] = {
            "left": {"success": None, "error_pos_m": 0.0, "iterations": 0},
            "right": {"success": None, "error_pos_m": 0.0, "iterations": 0},
        }

        self._fk: Dict[str, ArmFK] = {
            "left": ArmFK("left", "urdf"),
            "right": ArmFK("right", "urdf"),
        }
        self._ik: Dict[str, ArmIK] = {
            "left": ArmIK("left", "dls", max_iter=10, tol_pos_m=0.005, tol_rot_rad=0.02),
            "right": ArmIK("right", "dls", max_iter=10, tol_pos_m=0.005, tol_rot_rad=0.02),
        }

        ChannelFactoryInitialize(domain_id, iface)
        self._state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self._pub = ArmSDKPublisher()
        self.robot = Robot(iface=iface, domain_id=domain_id, auto_start_sensors=True)

        # Seed from live state before starting loop
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            pos = self._state_sub.snapshot()
            if pos:
                self.latest_positions = pos
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self.seeded = True
                self._sync_ee_from_joints()
                self.status = f"Connected on {iface}"
                break
            time.sleep(0.02)

        self._last_tick = time.monotonic()
        threading.Thread(target=self._control_loop, daemon=True, name="arm-ctrl").start()

    # ── Internal helpers ──────────────────────────────────────────────────────
    def _sync_ee_from_joints(self) -> None:
        for arm, joints in ARM_JOINTS.items():
            q = np.array([self.desired_targets[j] for j in joints])
            self.target_T[arm] = self._fk[arm].compute_arm(q).copy()

    def _sync_targets_to_live(self) -> None:
        pos = self._state_sub.snapshot()
        if pos:
            self.latest_positions = pos
        self.current_targets = dict(self.latest_positions)
        self.desired_targets = dict(self.latest_positions)
        self._sync_ee_from_joints()

    def _ramp_step(self, dt: float) -> None:
        step = max(1e-9, self.max_speed * dt)
        for j in UPPER_BODY_JOINTS:
            cur = float(self.current_targets[j])
            des = float(self.desired_targets[j])
            d = des - cur
            if abs(d) <= step:
                self.current_targets[j] = des
            else:
                self.current_targets[j] = cur + math.copysign(step, d)

    def _targets_reached(self, eps: float = 0.02) -> bool:
        return all(
            abs(float(self.current_targets[j]) - float(self.desired_targets[j])) <= eps
            for j in UPPER_BODY_JOINTS
        )

    def _control_loop(self) -> None:
        dt_target = 1.0 / self.rate_hz
        while True:
            now = time.monotonic()
            pos = self._state_sub.snapshot()
            if pos:
                with self._lock:
                    self.latest_positions = pos
                    if not self.seeded:
                        self.seeded = True
                        self.current_targets = dict(pos)
                        self.desired_targets = dict(pos)
                        self._sync_ee_from_joints()
                        self.status = f"Connected on {self.iface}"

            with self._lock:
                if self.seeded and self.armed:
                    dt = max(dt_target, now - self._last_tick)
                    self._last_tick = now
                    self._ramp_step(dt)
                    self._pub.publish(
                        self.current_targets,
                        arm_kp=self.arm_kp,
                        arm_kd=self.arm_kd,
                        waist_pr_kp=self.waist_pr_kp,
                        waist_y_kp=self.waist_y_kp,
                        waist_kd=self.waist_kd,
                    )

            elapsed = time.monotonic() - now
            rem = dt_target - elapsed
            if rem > 0:
                time.sleep(rem)

    # ── IK internals ──────────────────────────────────────────────────────────
    def _active_arms(self) -> List[str]:
        return ["left", "right"] if self.arm_control_mode == "both" else [self.arm_control_mode]

    def _fk_desired(self, arm: str) -> np.ndarray:
        q = np.array([self.desired_targets[j] for j in ARM_JOINTS[arm]])
        return self._fk[arm].compute_arm(q)

    def _solve_position_shoulder_elbow(
        self,
        arm: str,
        T_des: np.ndarray,
        q_init: np.ndarray,
        selected_axis: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], Dict]:
        q = _clamp_q(q_init.copy(), arm)
        lam, eps_fd = 0.05, 1e-5
        best_q, best_err = q.copy(), float("inf")

        for iteration in range(64):
            T_cur = self._fk[arm].compute_arm(q)
            pos_err = T_des[:3, 3] - T_cur[:3, 3]
            err_pos = float(np.linalg.norm(pos_err))
            if err_pos < best_err:
                best_q, best_err = q.copy(), err_pos
            if err_pos < POSITION_IK_TOL_M:
                return q, {"success": True, "error_pos_m": err_pos, "error_rot_rad": 0.0, "iterations": iteration}

            J = np.zeros((3, len(SHOULDER_ELBOW_IDXS)), dtype=np.float64)
            p0 = T_cur[:3, 3]
            for col, idx in enumerate(SHOULDER_ELBOW_IDXS):
                q1 = q.copy()
                q1[idx] += eps_fd
                J[:, col] = (self._fk[arm].compute_arm(q1)[:3, 3] - p0) / eps_fd
            dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), pos_err)
            norm = float(np.linalg.norm(dq))
            if norm > 0.3:
                dq *= 0.3 / norm
            q_next = q.copy()
            for col, idx in enumerate(SHOULDER_ELBOW_IDXS):
                q_next[idx] += dq[col]
            q = _clamp_q(q_next, arm)
            q[4:] = q_init[4:]

        T_cur = self._fk[arm].compute_arm(best_q)
        err_pos = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
        axis_err = (
            abs(float(T_des[selected_axis, 3] - T_cur[selected_axis, 3]))
            if selected_axis is not None else err_pos
        )
        if selected_axis is not None and axis_err < POSITION_IK_AXIS_TOL_M and err_pos < POSITION_IK_SOFT_LIMIT_M:
            return best_q, {"success": True, "error_pos_m": err_pos, "error_rot_rad": 0.0, "iterations": 64}
        return None, {"success": False, "error_pos_m": err_pos, "error_rot_rad": 0.0, "iterations": 64}

    def _apply_ik(
        self,
        arm: str,
        T_prev: np.ndarray,
        *,
        shoulder_elbow_only: bool = False,
        selected_axis: Optional[int] = None,
    ) -> bool:
        joints = ARM_JOINTS[arm]
        q_init = np.array([self.desired_targets[j] for j in joints])
        if shoulder_elbow_only:
            q_sol, info = self._solve_position_shoulder_elbow(
                arm, self.target_T[arm], q_init, selected_axis=selected_axis)
        else:
            q_sol, info = self._ik[arm].solve(self.target_T[arm], q_init=q_init)
        self.ik_info[arm] = info
        if q_sol is None:
            return False
        q_apply = q_init + np.clip(q_sol - q_init, -self.max_dq, self.max_dq)
        for i, j in enumerate(joints):
            self.desired_targets[j] = float(q_apply[i])
        self.target_T[arm] = self._fk[arm].compute_arm(q_apply).copy()
        return True

    def _adjust_dof(self, dof_idx: int, delta: float) -> bool:
        any_ok = False
        for arm in self._active_arms():
            T_prev = self.target_T[arm].copy()
            arm_delta = delta
            if dof_idx == 1:
                arm_delta = delta if arm == "left" else -delta
            elif dof_idx in (3, 5):
                arm_delta = -delta if arm == "left" else delta
            elif dof_idx == 4:
                arm_delta = -delta

            shoulder_elbow_only = dof_idx < 3 and not self.orient_stiff
            for scale in (1.0, 0.5, 0.25, 0.1):
                T_new = T_prev.copy()
                if dof_idx < 3:
                    if not self.orient_stiff:
                        T_new[:3, :3] = self._fk_desired(arm)[:3, :3]
                    T_new[dof_idx, 3] += arm_delta * scale
                else:
                    axis = dof_idx - 3
                    T_new[:3, :3] = _ROT_BY_AXIS[axis](arm_delta * scale) @ T_new[:3, :3]
                self.target_T[arm] = T_new
                if self._apply_ik(
                    arm, T_prev,
                    shoulder_elbow_only=shoulder_elbow_only,
                    selected_axis=dof_idx if shoulder_elbow_only else None,
                ):
                    any_ok = True
                    break
            else:
                self.target_T[arm] = T_prev
        return any_ok

    # ── Public API ────────────────────────────────────────────────────────────
    def nudge(self, dof_idx: int, direction: float) -> str:
        if not self.seeded or not self.armed:
            return "Not ready."
        step = self.pos_step if dof_idx < 3 else self.rot_step
        with self._lock:
            ok = self._adjust_dof(dof_idx, direction * step)
        sign = "+" if direction > 0 else "−"
        return f"{DOF_NAMES[dof_idx]} {sign}{step:.3f} {DOF_UNITS[dof_idx]} — IK {'OK' if ok else 'FAILED'}"

    def extend_arm_forward(self) -> str:
        if not self.seeded or not self.armed:
            return "Not ready."

        def _run() -> None:
            with self._lock:
                self.orient_stiff = False
                self.status = "Extending: orient lock OFF, y +0.15 m…"
                self._adjust_dof(1, +0.15)
            _wait_ramp(self)
            with self._lock:
                self.status = "Extending: z +1.0 m…"
                self._adjust_dof(2, +1.0)
            _wait_ramp(self)
            with self._lock:
                self.status = "Extending: x +0.8 m…"
                self._adjust_dof(0, +0.8)
            with self._lock:
                self.status = "Extend arm forward complete."

        threading.Thread(target=_run, daemon=True, name="extend-arm").start()
        return "Extend arm forward sequence started…"

    def release(self) -> str:
        try:
            self.robot.wait_for_low_state(timeout=2.0)
            # Disarm the publish loop first so it stops fighting the release ramp.
            with self._lock:
                self.armed = False
            self.robot.release_arms()
            return "Arms released — move freely."
        except Exception as exc:
            with self._lock:
                self.armed = True  # restore on failure
            return f"Release failed: {exc}"

    def unrelease(self) -> str:
        try:
            self.robot.wait_for_low_state(timeout=2.0)
            self.robot.unrelease_arms()
            with self._lock:
                self.armed = True
                self._sync_targets_to_live()
            return "Arms reengaged, synced to live pose."
        except Exception as exc:
            return f"Reengage failed: {exc}"

    def set_arm_mode(self, mode: str) -> str:
        with self._lock:
            self.arm_control_mode = mode
        return f"Arm mode → {mode}"

    def toggle_orient_stiff(self) -> str:
        with self._lock:
            self.orient_stiff = not self.orient_stiff
        return f"Orient lock {'ON — rotation held during x/y/z moves' if self.orient_stiff else 'OFF — position-only IK'}"

    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            disp = "right" if self.arm_control_mode != "left" else "left"
            T = self.target_T[disp]
            rpy = _rpy_from_R(T[:3, :3])
            return {
                "seeded": self.seeded,
                "armed": self.armed,
                "arm_mode": self.arm_control_mode,
                "orient_stiff": self.orient_stiff,
                "status": self.status,
                "ee": {
                    "x": float(T[0, 3]),
                    "y": float(T[1, 3]),
                    "z": float(T[2, 3]),
                    "roll": float(rpy[0]),
                    "pitch": float(rpy[1]),
                    "yaw": float(rpy[2]),
                },
                "ik_info": dict(self.ik_info),
            }


def _wait_ramp(ctrl: ArmController, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with ctrl._lock:
            if ctrl._targets_reached():
                return
        time.sleep(0.05)


# ── Dash app ──────────────────────────────────────────────────────────────────
def make_app(ctrl: ArmController) -> dash.Dash:
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        title="G1 Upper Body Control",
    )

    def _dof_row(dof: str) -> dbc.Row:
        return dbc.Row(
            [
                dbc.Col(
                    html.Span(dof.upper(), className="fw-semibold text-uppercase"),
                    width=2,
                    className="d-flex align-items-center justify-content-center text-muted",
                ),
                dbc.Col(
                    html.Div("—", id=f"val-{dof}", className="font-monospace text-center"),
                    width=4,
                    className="d-flex align-items-center justify-content-center",
                ),
                dbc.Col(
                    dbc.Button(
                        "−", id=f"btn-dof-{dof}-dec",
                        color="secondary", n_clicks=0,
                        className="w-100 fw-bold fs-5",
                    ),
                    width=3,
                ),
                dbc.Col(
                    dbc.Button(
                        "+", id=f"btn-dof-{dof}-inc",
                        color="primary", n_clicks=0,
                        className="w-100 fw-bold fs-5",
                    ),
                    width=3,
                ),
            ],
            className="mb-1 g-1",
        )

    app.layout = html.Div(
        [
            dcc.Interval(id="state-interval", interval=500, n_intervals=0),
            dcc.Store(id="event-log-store", data=[]),
            dbc.Container(
                dbc.Row(
                    dbc.Col(
                        [
                            html.H4("G1 Upper Body IK Control", className="text-center mb-3 mt-4"),

                            # ── Connection status ─────────────────────────
                            html.Div(id="status-display",
                                     className="text-center text-muted mb-3 small"),

                            # ── Release / Arm selection ───────────────────
                            dbc.Row(
                                [
                                    dbc.Col(
                                        dbc.Button(
                                            "Release Arms",
                                            id="btn-arm-release",
                                            color="danger",
                                            n_clicks=0,
                                            className="w-100 py-2 fw-semibold",
                                            size="lg",
                                        ),
                                        xs=12, sm=5, className="mb-2",
                                    ),
                                    dbc.Col(
                                        [
                                            html.Span(
                                                "Arm:", className="fw-semibold me-2 text-muted"),
                                            dbc.RadioItems(
                                                id="radio-arm-mode",
                                                options=[
                                                    {"label": " Right", "value": "right"},
                                                    {"label": " Left", "value": "left"},
                                                    {"label": " Both", "value": "both"},
                                                ],
                                                value="right",
                                                inline=True,
                                                className="d-inline-flex gap-3",
                                            ),
                                        ],
                                        xs=12, sm=7,
                                        className="mb-2 d-flex align-items-center",
                                    ),
                                ],
                                className="mb-3 g-2",
                            ),

                            # ── Orient lock / Extend forward ──────────────
                            dbc.Row(
                                [
                                    dbc.Col(
                                        dbc.Button(
                                            "Orient Lock: ON",
                                            id="btn-orient-lock",
                                            color="info",
                                            n_clicks=0,
                                            className="w-100 py-2 fw-semibold",
                                            size="lg",
                                        ),
                                        xs=12, sm=5, className="mb-2",
                                    ),
                                    dbc.Col(
                                        dbc.Button(
                                            "Extend Arm Forward",
                                            id="btn-extend-forward",
                                            color="success",
                                            n_clicks=0,
                                            className="w-100 py-2 fw-semibold",
                                            size="lg",
                                        ),
                                        xs=12, sm=7, className="mb-2",
                                    ),
                                ],
                                className="mb-3 g-2",
                            ),

                            # ── DOF header ────────────────────────────────
                            dbc.Row(
                                [
                                    dbc.Col(html.Small("DOF", className="text-muted fw-bold"),
                                            width=2, className="text-center"),
                                    dbc.Col(html.Small("Target", className="text-muted fw-bold"),
                                            width=4, className="text-center"),
                                    dbc.Col(html.Small("−", className="text-muted fw-bold"),
                                            width=3, className="text-center"),
                                    dbc.Col(html.Small("+", className="text-muted fw-bold"),
                                            width=3, className="text-center"),
                                ],
                                className="mb-1",
                            ),

                            # ── DOF rows ──────────────────────────────────
                            *[_dof_row(dof) for dof in DOF_NAMES],

                            # ── Command result & log ──────────────────────
                            html.Div(
                                id="command-result",
                                className="text-center mt-3 fw-semibold",
                                style={"minHeight": "1.4rem"},
                            ),
                            html.Pre(id="event-log", className="state-box mt-2"),
                        ],
                        xs=12, sm=10, md=8, lg=6,
                    ),
                    className="justify-content-center min-vh-100",
                ),
                fluid=True,
            ),
        ]
    )

    app.index_string = """
<!DOCTYPE html>
<html>
  <head>
    {%metas%}
    <title>{%title%}</title>
    {%favicon%}
    {%css%}
    <style>
      body { background: #f7f7f8; }
      .state-box {
        background: #111827;
        border-radius: 6px;
        color: #e5e7eb;
        font-size: 0.78rem;
        margin: 0;
        max-height: 8rem;
        overflow: auto;
        padding: 0.6rem 0.75rem;
        white-space: pre-wrap;
      }
    </style>
  </head>
  <body>
    {%app_entry%}
    <footer>
      {%config%}
      {%scripts%}
      {%renderer%}
    </footer>
  </body>
</html>
"""

    # ── Outputs ───────────────────────────────────────────────────────────────
    _outputs = [
        Output("status-display", "children"),
        Output("command-result", "children"),
        Output("event-log", "children"),
        Output("event-log-store", "data"),
        Output("btn-arm-release", "children"),
        Output("btn-arm-release", "color"),
        Output("btn-orient-lock", "children"),
        Output("btn-orient-lock", "color"),
        Output("btn-extend-forward", "disabled"),
    ]
    for _dof in DOF_NAMES:
        _outputs.append(Output(f"val-{_dof}", "children"))
    for _dof in DOF_NAMES:
        _outputs.append(Output(f"btn-dof-{_dof}-dec", "disabled"))
        _outputs.append(Output(f"btn-dof-{_dof}-inc", "disabled"))

    # ── Inputs ────────────────────────────────────────────────────────────────
    _inputs = [
        Input("state-interval", "n_intervals"),
        Input("btn-arm-release", "n_clicks"),
        Input("radio-arm-mode", "value"),
        Input("btn-orient-lock", "n_clicks"),
        Input("btn-extend-forward", "n_clicks"),
    ]
    for _dof in DOF_NAMES:
        _inputs.append(Input(f"btn-dof-{_dof}-dec", "n_clicks"))
        _inputs.append(Input(f"btn-dof-{_dof}-inc", "n_clicks"))

    @app.callback(
        _outputs,
        _inputs,
        State("command-result", "children"),
        State("event-log-store", "data"),
        prevent_initial_call=False,
    )
    def update(_n_intervals: int, *args: Any) -> tuple:  # noqa: C901
        prior_result = args[-2] or ""
        event_log = list(args[-1]) if isinstance(args[-1], list) else []
        result = prior_result
        trigger = dash.ctx.triggered_id

        # ── Handle button / radio events ──────────────────────────────────
        if trigger == "btn-arm-release":
            st = ctrl.get_state()
            if st["armed"]:
                result = ctrl.release()
            else:
                result = ctrl.unrelease()
            event_log = [f"{time.strftime('%H:%M:%S')} {result}", *event_log][:30]

        elif trigger == "radio-arm-mode":
            mode = args[1]  # second arg after n_intervals is radio value
            result = ctrl.set_arm_mode(mode)
            event_log = [f"{time.strftime('%H:%M:%S')} {result}", *event_log][:30]

        elif trigger == "btn-orient-lock":
            result = ctrl.toggle_orient_stiff()
            event_log = [f"{time.strftime('%H:%M:%S')} {result}", *event_log][:30]

        elif trigger == "btn-extend-forward":
            result = ctrl.extend_arm_forward()
            event_log = [f"{time.strftime('%H:%M:%S')} {result}", *event_log][:30]

        elif isinstance(trigger, str) and trigger.startswith("btn-dof-"):
            # btn-dof-{dof}-dec or btn-dof-{dof}-inc
            parts = trigger.split("-")  # ["btn", "dof", dof_name, dir]
            dof_name = parts[2]
            direction = -1.0 if parts[3] == "dec" else +1.0
            dof_idx = DOF_NAMES.index(dof_name)
            result = ctrl.nudge(dof_idx, direction)
            event_log = [f"{time.strftime('%H:%M:%S')} {result}", *event_log][:30]

        # ── Collect display state ─────────────────────────────────────────
        st = ctrl.get_state()

        conn_tag = "[CONNECTED]" if st["seeded"] else "[WAITING]"
        arm_tag = "[ARMED]" if st["armed"] else "[RELEASED]"
        orient_tag = "orient:locked" if st["orient_stiff"] else "orient:free"
        status_line = f"{conn_tag}  {arm_tag}  arm:{st['arm_mode']}  {orient_tag}  {st['status']}"

        release_label = "Reengage Arms" if not st["armed"] else "Release Arms"
        release_color = "success" if not st["armed"] else "danger"

        orient_label = f"Orient Lock: {'ON' if st['orient_stiff'] else 'OFF'}"
        orient_color = "info" if st["orient_stiff"] else "warning"

        not_ready = not st["seeded"] or not st["armed"]

        ee = st["ee"]
        val_children = [
            f"{ee[dof]:+.4f} {DOF_UNITS[i]}" for i, dof in enumerate(DOF_NAMES)
        ]

        disabled_list: List[bool] = []
        for _ in DOF_NAMES:
            disabled_list.append(not_ready)  # dec
            disabled_list.append(not_ready)  # inc

        return (
            status_line,
            result,
            "\n".join(event_log),
            event_log,
            release_label,
            release_color,
            orient_label,
            orient_color,
            not_ready,
            *val_children,
            *disabled_list,
        )

    return app


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Dash upper-body IK control panel for the Unitree G1.")
    p.add_argument("--iface", default=os.environ.get("G1_IFACE", "eth0"))
    p.add_argument("--domain-id", type=int, default=int(os.environ.get("G1_DOMAIN_ID", "0")))
    p.add_argument("--host", default=os.environ.get("UB_CONTROL_HOST", "0.0.0.0"))
    p.add_argument("--port", type=int,
                   default=int(os.environ.get("UB_CONTROL_PORT", "8052")))
    p.add_argument("--rate-hz", type=float, default=25.0, help="Control loop publish rate")
    p.add_argument("--speed", type=float, default=0.2, help="Joint ramp speed rad/s")
    p.add_argument("--pos-step", type=float, default=0.02, help="Position nudge step (m)")
    p.add_argument("--rot-step", type=float, default=0.05, help="Rotation nudge step (rad)")
    p.add_argument("--max-dq", type=float, default=0.2,
                   help="Max joint delta per IK solve (rad)")
    p.add_argument("--debug", action="store_true")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ctrl = ArmController(
        iface=args.iface,
        domain_id=args.domain_id,
        rate_hz=args.rate_hz,
        max_speed=args.speed,
        pos_step=args.pos_step,
        rot_step=args.rot_step,
        max_dq=args.max_dq,
    )
    app = make_app(ctrl)
    app.run(host=args.host, port=args.port, debug=bool(args.debug))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
