"""
Step 7 — Inverse kinematics: desired pose -> q_arm_desired
============================================================
Solves for the 7-DOF arm joint angles that achieve a desired
end-effector pose T_base_hand_desired.

Two solvers are available:
    "pin"   — pinocchio Levenberg-Marquardt IK (preferred)
    "num"   — pure-numpy damped least-squares Jacobian iteration

Both respect joint limits defined in arm_fk.JOINT_LIMITS.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
try:
    from scipy.optimize import minimize  # type: ignore
except ImportError:  # DLS solvers do not require SciPy
    minimize = None  # type: ignore[assignment]

from .arm_fk import ArmFK, JOINT_LIMITS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, _fk_dh, _DH_RIGHT


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def _clamp(q: np.ndarray, limits: List[Tuple[float, float]]) -> np.ndarray:
    lo = np.array([lim[0] for lim in limits])
    hi = np.array([lim[1] for lim in limits])
    return np.clip(q, lo, hi)


def _pose_error(T_desired: np.ndarray, T_current: np.ndarray) -> np.ndarray:
    """
    6-D error vector [pos_err (3), rot_err (3)] in base frame.
    Rotation error uses the skew-symmetric approach (small-angle valid near solution).
    """
    pos_err = T_desired[:3, 3] - T_current[:3, 3]
    R_err = T_desired[:3, :3] @ T_current[:3, :3].T
    # Extract axis-angle from rotation error matrix
    rot_err = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) * 0.5
    return np.concatenate([pos_err, rot_err])


def _numerical_jacobian(
    q: np.ndarray,
    fk: ArmFK,
    eps: float = 1e-5,
) -> np.ndarray:
    """Finite-difference 6×7 Jacobian for DH-based FK."""
    J = np.zeros((6, 7), dtype=np.float64)
    T0 = fk.compute_arm(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    for i in range(7):
        q1 = q.copy()
        q1[i] += eps
        T1 = fk.compute_arm(q1)
        J[:3, i] = (T1[:3, 3] - p0) / eps
        dR = T1[:3, :3] @ R0.T
        J[3:, i] = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * eps)
    return J


# ---------------------------------------------------------------------------
# Main solver
# ---------------------------------------------------------------------------

class ArmIK:
    """
    Step 7: Solve IK for the G1 arm.

    Args:
        arm:          "left" | "right"
        solver:       "dls" | "scipy" | "pin"
        max_iter:     maximum iterations for iterative solvers
        tol_pos_m:    position convergence tolerance (metres)
        tol_rot_rad:  rotation convergence tolerance (radians)
        damping:      DLS damping factor λ
    """

    def __init__(
        self,
        arm: str = "right",
        solver: str = "dls",
        max_iter: int = 200,
        tol_pos_m: float = 0.003,
        tol_rot_rad: float = 0.01,
        damping: float = 0.05,
    ) -> None:
        self.arm = arm
        self.solver = solver
        self.max_iter = max_iter
        self.tol_pos_m = tol_pos_m
        self.tol_rot_rad = tol_rot_rad
        self.damping = damping
        self._fk = ArmFK(arm=arm, backend="urdf")
        self._limits = JOINT_LIMITS[arm]
        self._joint_indices = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS

    # ------------------------------------------------------------------
    def solve(
        self,
        T_base_desired: np.ndarray,
        q_init: Optional[np.ndarray] = None,
    ) -> Tuple[Optional[np.ndarray], Dict]:
        """
        Solve IK.

        Args:
            T_base_desired: 4×4 desired end-effector pose in base frame
            q_init:         7-element initial arm joint angles (radians);
                            if None, uses zeros

        Returns:
            (q_arm, info) where q_arm is length-7 or None on failure.
            info dict contains: "success", "error_pos_m", "error_rot_rad", "iterations"
        """
        if q_init is None:
            q_init = np.zeros(7)
        q = _clamp(q_init.copy(), self._limits)

        if self.solver == "dls":
            q_sol, info = self._solve_dls(T_base_desired, q)
            if q_sol is not None:
                return q_sol, info
            q_pos, pos_info = self._solve_position_dls(T_base_desired, q)
            if q_pos is not None:
                pos_info["fallback"] = "position_dls"
                return q_pos, pos_info
            info["fallback_error_pos_m"] = pos_info["error_pos_m"]
            return None, info
        elif self.solver == "position_dls":
            # For an RGB-D box/mask target, position is measured while full
            # wrist orientation is only a grasp preference.  This mode lets
            # the redundant wrist joints (especially wrist pitch) find the
            # reachable position rather than forcing a brittle 6-D pose.
            return self._solve_position_dls(T_base_desired, q)
        elif self.solver == "level_dls":
            return self._solve_level_dls(T_base_desired, q)
        elif self.solver == "scipy":
            return self._solve_scipy(T_base_desired, q)
        elif self.solver == "pin":
            return self._solve_pin(T_base_desired, q)
        else:
            raise ValueError(f"Unknown solver: {self.solver!r}")

    # ------------------------------------------------------------------
    # Damped Least Squares (primary solver — fast, no external deps)
    # ------------------------------------------------------------------

    def _solve_dls(
        self, T_des: np.ndarray, q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Dict]:
        lam = self.damping
        for iteration in range(self.max_iter):
            T_cur = self._fk.compute_arm(q)
            err = _pose_error(T_des, T_cur)

            err_pos = float(np.linalg.norm(err[:3]))
            err_rot = float(np.linalg.norm(err[3:]))

            if err_pos < self.tol_pos_m and err_rot < self.tol_rot_rad:
                return q, {
                    "success": True,
                    "error_pos_m": err_pos,
                    "error_rot_rad": err_rot,
                    "iterations": iteration,
                }

            J = _numerical_jacobian(q, self._fk)
            # DLS: dq = J^T (J J^T + λ²I)^-1 err
            JJT = J @ J.T
            dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), err)

            norm_dq = float(np.linalg.norm(dq))
            if norm_dq > 0.3:
                dq *= 0.3 / norm_dq
            q = _clamp(q + dq, self._limits)

        T_cur = self._fk.compute_arm(q)
        err = _pose_error(T_des, T_cur)
        return None, {
            "success": False,
            "error_pos_m": float(np.linalg.norm(err[:3])),
            "error_rot_rad": float(np.linalg.norm(err[3:])),
            "iterations": self.max_iter,
        }

    def _solve_position_dls(
        self, T_des: np.ndarray, q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Dict]:
        """DLS fallback that reaches the wrist position and ignores orientation.

        Vision detections only provide a coarse PCA orientation. Requiring the
        wrist to match that full 6-DoF pose can reject reachable object
        positions, so this fallback is intentionally position-first.
        """
        lam = self.damping
        target_pos = T_des[:3, 3]
        for iteration in range(self.max_iter):
            T_cur = self._fk.compute_arm(q)
            pos_err = target_pos - T_cur[:3, 3]
            err_pos = float(np.linalg.norm(pos_err))
            if err_pos < self.tol_pos_m:
                return q, {
                    "success": True,
                    "error_pos_m": err_pos,
                    "error_rot_rad": 0.0,
                    "iterations": iteration,
                }

            J_pos = _numerical_jacobian(q, self._fk)[:3, :]
            JJT = J_pos @ J_pos.T
            dq = J_pos.T @ np.linalg.solve(JJT + lam**2 * np.eye(3), pos_err)

            norm_dq = float(np.linalg.norm(dq))
            if norm_dq > 0.3:
                dq *= 0.3 / norm_dq
            q = _clamp(q + dq, self._limits)

        T_cur = self._fk.compute_arm(q)
        err_pos = float(np.linalg.norm(target_pos - T_cur[:3, 3]))
        return None, {
            "success": False,
            "error_pos_m": err_pos,
            "error_rot_rad": 0.0,
            "iterations": self.max_iter,
        }

    def _solve_level_dls(
        self, T_des: np.ndarray, q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Dict]:
        """DLS on position plus the palm normal (EE z axis); yaw is free.

        The Dex3 finger joints all rotate about the palm z axis, so the
        fingers close in the palm x-y plane.  Aligning palm z with T_des's
        z axis (world up for a side grasp) keeps that closing plane parallel
        to the table, while rotation about the axis stays free for reach.
        error_rot_rad is the angle between the two axes.
        """
        lam = self.damping
        target_pos = T_des[:3, 3]
        z_des = T_des[:3, 2] / max(1e-9, float(np.linalg.norm(T_des[:3, 2])))

        def errors(T_cur: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, float]:
            pos_err = target_pos - T_cur[:3, 3]
            z_cur = T_cur[:3, 2]
            axis_err = np.cross(z_cur, z_des)  # rotates z_cur onto z_des
            angle = float(np.arccos(np.clip(np.dot(z_cur, z_des), -1.0, 1.0)))
            return pos_err, axis_err, float(np.linalg.norm(pos_err)), angle

        for iteration in range(self.max_iter):
            T_cur = self._fk.compute_arm(q)
            pos_err, axis_err, err_pos, err_rot = errors(T_cur)
            if err_pos < self.tol_pos_m and err_rot < self.tol_rot_rad:
                return q, {
                    "success": True,
                    "error_pos_m": err_pos,
                    "error_rot_rad": err_rot,
                    "iterations": iteration,
                }

            J = _numerical_jacobian(q, self._fk)
            # Drop the angular-rate component about the palm axis: yaw is free.
            z_cur = T_cur[:3, 2]
            J_rot = (np.eye(3) - np.outer(z_cur, z_cur)) @ J[3:, :]
            J_level = np.vstack([J[:3, :], J_rot])
            err = np.concatenate([pos_err, axis_err])
            dq = J_level.T @ np.linalg.solve(J_level @ J_level.T + lam**2 * np.eye(6), err)

            norm_dq = float(np.linalg.norm(dq))
            if norm_dq > 0.3:
                dq *= 0.3 / norm_dq
            q = _clamp(q + dq, self._limits)

        _pos, _axis, err_pos, err_rot = errors(self._fk.compute_arm(q))
        return None, {
            "success": False,
            "error_pos_m": err_pos,
            "error_rot_rad": err_rot,
            "iterations": self.max_iter,
        }

    # ------------------------------------------------------------------
    # SciPy SLSQP (accurate, slower — use as fallback)
    # ------------------------------------------------------------------

    def _solve_scipy(
        self, T_des: np.ndarray, q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Dict]:
        if minimize is None:
            raise RuntimeError("SciPy is required for the scipy IK solver")
        lo = [lim[0] for lim in self._limits]
        hi = [lim[1] for lim in self._limits]

        def objective(q_):
            T_cur = self._fk.compute_arm(q_)
            err = _pose_error(T_des, T_cur)
            return float(np.dot(err, err))

        result = minimize(
            objective, q,
            method="SLSQP",
            bounds=list(zip(lo, hi)),
            options={"maxiter": self.max_iter, "ftol": 1e-8},
        )
        q_sol = _clamp(result.x, self._limits)
        T_cur = self._fk.compute_arm(q_sol)
        err = _pose_error(T_des, T_cur)
        success = result.success and np.linalg.norm(err[:3]) < self.tol_pos_m
        return (q_sol if success else None), {
            "success": success,
            "error_pos_m": float(np.linalg.norm(err[:3])),
            "error_rot_rad": float(np.linalg.norm(err[3:])),
            "iterations": result.nit,
        }

    # ------------------------------------------------------------------
    # Pinocchio IK (best accuracy, requires pip install pin)
    # ------------------------------------------------------------------

    def _solve_pin(
        self, T_des: np.ndarray, q: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Dict]:
        try:
            import pinocchio as pin  # type: ignore
            from .arm_fk import _pin_model, _pin_data, _pin_ee_frame_id, _URDF_PATH
            if _pin_model is None:
                raise ImportError("pinocchio model not loaded")

            frame_id = _pin_ee_frame_id.get(self.arm)
            if frame_id is None:
                raise ValueError(f"No frame ID for arm {self.arm!r}")

            q_pin = pin.neutral(_pin_model)
            for i, arm_idx in enumerate(self._joint_indices):
                if arm_idx < len(q_pin):
                    q_pin[arm_idx] = q[i]

            goal = pin.SE3(T_des[:3, :3], T_des[:3, 3])
            success = False
            for it in range(self.max_iter):
                pin.forwardKinematics(_pin_model, _pin_data, q_pin)
                pin.updateFramePlacements(_pin_model, _pin_data)
                err_se3 = goal.actInv(_pin_data.oMf[frame_id])
                err = pin.log(err_se3).vector
                if np.linalg.norm(err[:3]) < self.tol_pos_m and \
                   np.linalg.norm(err[3:]) < self.tol_rot_rad:
                    success = True
                    break
                J = pin.computeFrameJacobian(
                    _pin_model, _pin_data, q_pin, frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )
                lam = self.damping
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), err)
                q_pin = pin.integrate(_pin_model, q_pin, dq * 0.5)

            q_sol = np.array([q_pin[i] for i in self._joint_indices])
            q_sol = _clamp(q_sol, self._limits)
            T_cur = self._fk.compute_arm(q_sol)
            err_vec = _pose_error(T_des, T_cur)
            return (q_sol if success else None), {
                "success": success,
                "error_pos_m": float(np.linalg.norm(err_vec[:3])),
                "error_rot_rad": float(np.linalg.norm(err_vec[3:])),
                "iterations": it,
            }
        except Exception as exc:
            return self._solve_dls(T_des, q)

    # ------------------------------------------------------------------
    def extract_arm_q(self, q_full: np.ndarray) -> np.ndarray:
        """Extract 7-element arm-only q from 30-element full joint array."""
        return q_full[self._joint_indices].copy()

    def inject_arm_q(self, q_full: np.ndarray, q_arm: np.ndarray) -> np.ndarray:
        """Write 7-element q_arm back into q_full at correct indices."""
        q_out = q_full.copy()
        q_out[self._joint_indices] = q_arm
        return q_out
