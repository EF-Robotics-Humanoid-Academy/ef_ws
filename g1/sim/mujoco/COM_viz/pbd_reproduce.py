#!/usr/bin/env python3
"""
pbd_reproduce.py
===============

Reproduce a recorded arm joint trajectory from a .npz file created by
pbd_demonstrate.py.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import threading
import time
from typing import Dict, List

import numpy as np
import pickle
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

except Exception as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


NOT_USED_IDX = 29  # enable arm sdk
LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_YAW_IDX = 12


def _rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _mat_to_rotvec(r: np.ndarray) -> np.ndarray:
    tr = float(np.trace(r))
    cos_theta = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-9:
        return np.zeros(3, dtype=float)
    denom = 2.0 * np.sin(theta)
    rx = (r[2, 1] - r[1, 2]) / denom
    ry = (r[0, 2] - r[2, 0]) / denom
    rz = (r[1, 0] - r[0, 1]) / denom
    axis = np.array([rx, ry, rz], dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.zeros(3, dtype=float)
    return axis / n * theta


def _pose_to_vec(p: np.ndarray, r: np.ndarray) -> np.ndarray:
    return np.concatenate([p, _mat_to_rotvec(r)], axis=0)


def _arm_fk(arm: str, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # Lightweight 7-DoF arm model for trajectory reproduction.
    # Link dimensions are approximate but internally consistent for FK->IK.
    if arm == "right":
        sign = 1.0
    elif arm == "left":
        sign = -1.0
    else:
        raise ValueError(f"unknown arm '{arm}'")

    upper_arm = 0.23
    forearm = 0.24
    wrist = 0.10
    shoulder_offset_y = 0.20 * sign
    shoulder_offset_z = 0.27

    p = np.array([0.0, shoulder_offset_y, shoulder_offset_z], dtype=float)
    r = np.eye(3, dtype=float)

    # shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_roll, wrist_yaw
    r = r @ _rot_y(float(q[0]))
    r = r @ _rot_x(float(q[1]))
    r = r @ _rot_z(float(q[2]))
    p = p + r @ np.array([upper_arm, 0.0, 0.0], dtype=float)
    r = r @ _rot_y(float(q[3]))
    p = p + r @ np.array([forearm, 0.0, 0.0], dtype=float)
    r = r @ _rot_y(float(q[4]))
    r = r @ _rot_x(float(q[5]))
    r = r @ _rot_z(float(q[6]))
    p = p + r @ np.array([wrist, 0.0, 0.0], dtype=float)
    return p, r


def _ik_solve_arm(
    arm: str,
    q_init: np.ndarray,
    p_target: np.ndarray,
    r_target: np.ndarray,
    max_iter: int = 40,
    damping: float = 1e-2,
) -> np.ndarray:
    q = q_init.astype(float).copy()
    dof = q.shape[0]
    w = np.array([1.0, 1.0, 1.0, 0.35, 0.35, 0.35], dtype=float)
    eps = 1e-4

    for _ in range(max_iter):
        p_cur, r_cur = _arm_fk(arm, q)
        e_pos = p_target - p_cur
        e_rot = _mat_to_rotvec(r_target @ r_cur.T)
        e = np.concatenate([e_pos, e_rot], axis=0)
        e_w = e * w
        if np.linalg.norm(e_w) < 1e-4:
            break

        j = np.zeros((6, dof), dtype=float)
        for i in range(dof):
            dq = np.zeros(dof, dtype=float)
            dq[i] = eps
            p_plus, r_plus = _arm_fk(arm, q + dq)
            p_minus, r_minus = _arm_fk(arm, q - dq)
            v_plus = _pose_to_vec(p_plus, r_plus)
            v_minus = _pose_to_vec(p_minus, r_minus)
            j[:, i] = (v_plus - v_minus) / (2.0 * eps)

        jw = j * w[:, None]
        a = jw.T @ jw + (damping ** 2) * np.eye(dof, dtype=float)
        b = jw.T @ e_w
        try:
            dq = np.linalg.solve(a, b)
        except np.linalg.LinAlgError:
            break
        q = q + dq
        q = np.clip(q, -3.0, 3.0)

    return q


def _col_map(joints: List[int]) -> Dict[int, int]:
    return {int(j): i for i, j in enumerate(joints)}


def _try_get_arm_cols(joints: List[int], arm: str) -> List[int]:
    idx = RIGHT_ARM_IDX if arm == "right" else LEFT_ARM_IDX
    c_map = _col_map(joints)
    cols = []
    for j in idx:
        if j not in c_map:
            return []
        cols.append(c_map[j])
    return cols


def _compute_ik_qs(joints: List[int], qs: np.ndarray, verbose: bool = True) -> np.ndarray:
    out = qs.copy().astype(float)
    left_cols = _try_get_arm_cols(joints, "left")
    right_cols = _try_get_arm_cols(joints, "right")

    if not left_cols and not right_cols:
        if verbose:
            print("IK: no complete 7-DoF arm set found in motion file; using recorded joints.")
        return out

    if verbose:
        active = []
        if left_cols:
            active.append("left")
        if right_cols:
            active.append("right")
        print(f"IK: solving for arm(s): {', '.join(active)}")

    n = out.shape[0]
    q_prev_left = out[0, left_cols].copy() if left_cols else None
    q_prev_right = out[0, right_cols].copy() if right_cols else None

    for i in range(n):
        if left_cols:
            q_demo = out[i, left_cols]
            p_t, r_t = _arm_fk("left", q_demo)
            q_init = q_prev_left if q_prev_left is not None else q_demo
            q_sol = _ik_solve_arm("left", q_init=q_init, p_target=p_t, r_target=r_t)
            out[i, left_cols] = q_sol
            q_prev_left = q_sol

        if right_cols:
            q_demo = out[i, right_cols]
            p_t, r_t = _arm_fk("right", q_demo)
            q_init = q_prev_right if q_prev_right is not None else q_demo
            q_sol = _ik_solve_arm("right", q_init=q_init, p_target=p_t, r_target=r_t)
            out[i, right_cols] = q_sol
            q_prev_right = q_sol

        if verbose and (i % max(1, n // 10) == 0):
            print(f"IK progress: {i + 1}/{n}")

    if verbose:
        print("IK: solve complete.")
    return out


def _select_replay_qs(joints: List[int], qs: np.ndarray, arm: str) -> tuple[List[int], np.ndarray]:
    c_map = _col_map(joints)
    selected: List[int] = []
    required: List[int] = []
    if arm in ("left", "both"):
        required.extend(LEFT_ARM_IDX)
        selected.extend(LEFT_ARM_IDX)
    if arm in ("right", "both"):
        required.extend(RIGHT_ARM_IDX)
        selected.extend(RIGHT_ARM_IDX)

    missing = [j for j in required if j not in c_map]
    if missing:
        raise SystemExit(
            f"Motion file missing required joints for --arm={arm}: {missing}. "
            "Record with pbd_demonstrate.py --arm both (default)."
        )

    if WAIST_YAW_IDX in c_map:
        selected.append(WAIST_YAW_IDX)

    cols = [c_map[j] for j in selected]
    return selected, qs[:, cols]


def _resolve_lowstate_type():
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            mod = __import__(module_path, fromlist=["LowState_"])
            if hasattr(mod, "LowState_"):
                return getattr(mod, "LowState_")
        except Exception:
            continue
    return None


class LowCmdController:
    def __init__(self, iface: str, joints: List[int], kp: float, kd: float) -> None:
        self._joints = [int(j) for j in joints]
        self._kp = float(kp)
        self._kd = float(kd)
        self._crc = CRC()
        self._cmd_q: Dict[int, float] = {j: 0.0 for j in self._joints}
        self._joint_cur: Dict[int, float] = {}
        self._state_ready = threading.Event()

        ChannelFactoryInitialize(0, iface)
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()

        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1

        LowState_ = _resolve_lowstate_type()
        self._sub = None
        if LowState_ is not None:
            def _ls_cb(msg):
                got_any = False
                for j_idx in self._joints:
                    try:
                        self._joint_cur[j_idx] = float(msg.motor_state[j_idx].q)
                        got_any = True
                    except Exception:
                        pass
                if got_any:
                    self._state_ready.set()

            self._sub = ChannelSubscriber("rt/lowstate", LowState_)
            self._sub.Init(_ls_cb, 200)

    def seed_from_lowstate(self, timeout_s: float = 0.8) -> bool:
        self._state_ready.wait(timeout=max(0.0, timeout_s))
        if not self._joint_cur:
            return False
        for j_idx, q_val in self._joint_cur.items():
            if j_idx in self._cmd_q:
                self._cmd_q[j_idx] = float(q_val)
        return True

    def write(self, q_by_joint: Dict[int, float]) -> None:
        for j_idx, q_val in q_by_joint.items():
            mc = self._cmd.motor_cmd[int(j_idx)]
            mc.q = float(q_val)
            mc.kp = self._kp
            mc.kd = self._kd
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)
        self._cmd_q.update(q_by_joint)

    def ramp_to(self, target_by_joint: Dict[int, float], duration_s: float, cmd_hz: float) -> None:
        duration_s = max(0.0, float(duration_s))
        if duration_s <= 0:
            self.write(target_by_joint)
            return

        steps = max(1, int(duration_s * max(1.0, cmd_hz)))
        dt = 1.0 / max(1.0, cmd_hz)
        start = {j: self._cmd_q.get(j, 0.0) for j in target_by_joint}

        for i in range(1, steps + 1):
            a = i / steps
            cur = {j: start[j] + (target_by_joint[j] - start[j]) * a for j in target_by_joint}
            self.write(cur)
            time.sleep(dt)


def _resolve_replay_qs(data: np.lib.npyio.NpzFile, mode: str) -> np.ndarray:
    qs = data["qs"].astype(float)
    if mode == "joint":
        return qs
    if mode == "fk":
        if "fk_qs" in data:
            return data["fk_qs"].astype(float)
        # Backward-compatible path for older recordings.
        return qs
    if mode == "ik":
        if "ik_qs" in data:
            return data["ik_qs"].astype(float)
        joints = data["joints"].astype(int).tolist()
        print("IK mode: no precomputed 'ik_qs' in file, solving IK from FK targets now.")
        return _compute_ik_qs(joints=joints, qs=qs, verbose=True)
    raise ValueError(f"Unsupported mode: {mode}")


def _interp_row(ts: np.ndarray, qs: np.ndarray, t: float) -> np.ndarray:
    if t <= float(ts[0]):
        return qs[0]
    if t >= float(ts[-1]):
        return qs[-1]
    hi = int(np.searchsorted(ts, t, side="right"))
    lo = max(0, hi - 1)
    t0 = float(ts[lo])
    t1 = float(ts[hi])
    if t1 <= t0:
        return qs[hi]
    a = (t - t0) / (t1 - t0)
    return qs[lo] * (1.0 - a) + qs[hi] * a


def _load_motion_file(path: str) -> Dict[str, np.ndarray]:
    if not path:
        raise ValueError("motion file path is empty")
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    ext = os.path.splitext(path)[1].lower()
    if ext == ".npz":
        with np.load(path, allow_pickle=True) as data:
            return {k: np.asarray(data[k]) for k in data.files}
    if ext == ".csv":
        with open(path, "r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"CSV has no header: {path}")

            ts_key = None
            for k in ("t_s", "ts", "time_s", "time"):
                if k in reader.fieldnames:
                    ts_key = k
                    break
            if ts_key is None:
                raise ValueError(
                    f"CSV must include one time column (t_s/ts/time_s/time): {path}"
                )

            joint_cols: List[tuple[int, str]] = []
            for name in reader.fieldnames:
                m = re.fullmatch(r"j(\d+)", str(name).strip().lower())
                if m:
                    joint_cols.append((int(m.group(1)), name))
            if not joint_cols:
                raise ValueError(f"CSV must include joint columns like j22,j23,...: {path}")

            ts_vals: List[float] = []
            q_rows: List[List[float]] = []
            for row in reader:
                if not row:
                    continue
                t_raw = row.get(ts_key)
                if t_raw is None or str(t_raw).strip() == "":
                    continue
                ts_vals.append(float(t_raw))
                q_rows.append([float(row[col_name]) for _, col_name in joint_cols])

            if not ts_vals or not q_rows:
                raise ValueError(f"CSV has no data rows: {path}")

            return {
                "joints": np.asarray([j for j, _ in joint_cols], dtype=int),
                "ts": np.asarray(ts_vals, dtype=float),
                "qs": np.asarray(q_rows, dtype=float),
            }
    if ext in (".pkl", ".pickle"):
        with open(path, "rb") as f:
            obj = pickle.load(f)
        if not isinstance(obj, dict):
            raise ValueError(f"Pickle motion file must contain a dict, got: {type(obj).__name__}")
        return {str(k): np.asarray(v) for k, v in obj.items()}

    # Fallback: try NPZ first, then pickle.
    try:
        with np.load(path, allow_pickle=True) as data:
            return {k: np.asarray(data[k]) for k in data.files}
    except Exception:
        try:
            with open(path, "rb") as f:
                obj = pickle.load(f)
            if not isinstance(obj, dict):
                raise ValueError(f"Unsupported motion file format: {path}")
            return {str(k): np.asarray(v) for k, v in obj.items()}
        except Exception as exc:
            raise ValueError(
                f"Unsupported motion file format for '{path}'. "
                "Use .npz, .csv (t_s + jXX columns), or .pkl/.pickle dict."
            ) from exc


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded arm motion.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--file", default="/tmp/pbd_motion.npz", help="input motion file (.npz or .pkl/.pickle)")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both", help="which arm(s) to replay")
    parser.add_argument("--mode", choices=["joint", "fk", "ik"], default="fk", help="trajectory replay space")
    parser.add_argument("--speed", type=float, default=1.0, help="time scale (1.0=real-time)")
    parser.add_argument("--cmd-hz", type=float, default=50.0, help="command rate (Hz)")
    parser.add_argument("--seed-timeout", type=float, default=0.8, help="seconds to wait for lowstate seeding")
    parser.add_argument("--start-ramp", type=float, default=0.8, help="seconds to ramp to first target pose")
    parser.add_argument("--kp", type=float, default=40.0, help="arm joint kp")
    parser.add_argument("--kd", type=float, default=1.0, help="arm joint kd")
    args = parser.parse_args()

    hanger_boot_sequence(iface=args.iface)
    data = _load_motion_file(args.file)
    joints = data["joints"].astype(int).tolist()
    ts = data["ts"].astype(float)
    qs = _resolve_replay_qs(data, args.mode)

    if len(ts) == 0 or len(qs) == 0:
        raise SystemExit("No samples in motion file.")
    if qs.shape[0] != len(ts):
        raise SystemExit("Invalid motion file: ts and qs length mismatch.")
    if qs.shape[1] != len(joints):
        raise SystemExit("Invalid motion file: joints and qs width mismatch.")
    joints, qs = _select_replay_qs(joints, qs, arm=args.arm)

    ctrl = LowCmdController(iface=args.iface, joints=joints, kp=args.kp, kd=args.kd)
    seeded = ctrl.seed_from_lowstate(timeout_s=args.seed_timeout)
    print(f"Lowstate seed: {'ok' if seeded else 'not available'}")

    dt = 1.0 / max(1e-6, args.cmd_hz)
    t_final = float(ts[-1]) / max(1e-6, args.speed)
    first_map = {int(j_idx): float(qs[0, j_i]) for j_i, j_idx in enumerate(joints)}
    ctrl.ramp_to(first_map, duration_s=max(0.0, args.start_ramp), cmd_hz=args.cmd_hz)

    print(
        f"Replaying {len(ts)} samples in {args.mode} mode "
        f"@ {args.cmd_hz} Hz (speed={args.speed})"
    )
    start = time.time()
    tick = 0
    try:
        while True:
            now = time.time() - start
            if now > t_final:
                break

            q_row = _interp_row(ts / max(1e-6, args.speed), qs, now)
            q_map = {int(j_idx): float(q_row[j_i]) for j_i, j_idx in enumerate(joints)}
            ctrl.write(q_map)

            if tick % max(1, int(args.cmd_hz // 5)) == 0:
                print(f"t={now:.2f}s -> " + ",".join([f"j{j}={q_map[j]:.3f}" for j in joints]))
            tick += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    print("Replay complete.")


if __name__ == "__main__":
    main()
