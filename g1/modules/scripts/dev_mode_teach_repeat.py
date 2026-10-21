#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import pickle
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODULES_DIR = os.path.abspath(os.path.join(_HERE, ".."))
for _path in (_HERE, _MODULES_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from dds_env import ensure_cyclonedds_environment

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
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

except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


G1_NUM_MOTOR = 29
ARM_SDK_WEIGHT_INDEX = 29
LEFT_ARM_JOINTS = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_JOINTS = [22, 23, 24, 25, 26, 27, 28]
PBD_ARM_JOINTS = {
    "left": list(LEFT_ARM_JOINTS),
    "right": list(RIGHT_ARM_JOINTS),
    "both": list(LEFT_ARM_JOINTS) + list(RIGHT_ARM_JOINTS),
}
BODY_JOINT_NAME_BY_INDEX = {
    15: "left_arm.shoulder_pitch",
    16: "left_arm.shoulder_roll",
    17: "left_arm.shoulder_yaw",
    18: "left_arm.elbow",
    19: "left_arm.wrist_roll",
    20: "left_arm.wrist_pitch",
    21: "left_arm.wrist_yaw",
    22: "right_arm.shoulder_pitch",
    23: "right_arm.shoulder_roll",
    24: "right_arm.shoulder_yaw",
    25: "right_arm.elbow",
    26: "right_arm.wrist_roll",
    27: "right_arm.wrist_pitch",
    28: "right_arm.wrist_yaw",
}
ARM_LIMITS = {
    15: (-3.0892, 2.6704),
    16: (-1.5882, 2.2515),
    17: (-2.618, 2.618),
    18: (-1.0472, 2.0944),
    19: (-1.9722, 1.9722),
    20: (-1.6144, 1.6144),
    21: (-1.6144, 1.6144),
    22: (-3.0892, 2.6704),
    23: (-2.2515, 1.5882),
    24: (-2.618, 2.618),
    25: (-1.0472, 2.0944),
    26: (-1.9722, 1.9722),
    27: (-1.6144, 1.6144),
    28: (-1.6144, 1.6144),
}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def smoothstep(ratio: float) -> float:
    x = clamp(ratio, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def normalize_arm_selection(arm: str) -> str:
    side = str(arm).strip().lower()
    if side not in PBD_ARM_JOINTS:
        raise ValueError("arm must be 'left', 'right', or 'both'.")
    return side


def load_pbd_motion_file(path: str) -> dict[str, np.ndarray]:
    if not path:
        raise ValueError("motion file path is empty")
    if not os.path.exists(path):
        raise FileNotFoundError(path)

    ext = os.path.splitext(path)[1].lower()
    if ext == ".npz":
        with np.load(path, allow_pickle=True) as data:
            return {k: np.asarray(data[k]) for k in data.files}
    if ext == ".csv":
        with open(path, "r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                raise ValueError(f"CSV has no header: {path}")
            ts_key = next((k for k in ("t_s", "ts", "time_s", "time") if k in reader.fieldnames), None)
            if ts_key is None:
                raise ValueError(f"CSV must include one time column (t_s/ts/time_s/time): {path}")
            joint_cols: list[tuple[int, str]] = []
            for name in reader.fieldnames:
                match = re.fullmatch(r"j(\d+)", str(name).strip().lower())
                if match:
                    joint_cols.append((int(match.group(1)), name))
            if not joint_cols:
                raise ValueError(f"CSV must include joint columns like j22,j23,...: {path}")
            ts_vals: list[float] = []
            q_rows: list[list[float]] = []
            for row in reader:
                if not row:
                    continue
                raw_t = row.get(ts_key)
                if raw_t is None or str(raw_t).strip() == "":
                    continue
                ts_vals.append(float(raw_t))
                q_rows.append([float(row[col_name]) for _, col_name in joint_cols])
            if not ts_vals or not q_rows:
                raise ValueError(f"CSV has no data rows: {path}")
            return {
                "joints": np.asarray([joint for joint, _ in joint_cols], dtype=int),
                "ts": np.asarray(ts_vals, dtype=float),
                "qs": np.asarray(q_rows, dtype=float),
            }
    if ext in (".pkl", ".pickle"):
        with open(path, "rb") as handle:
            obj = pickle.load(handle)
        if not isinstance(obj, dict):
            raise ValueError(f"Pickle motion file must contain a dict, got: {type(obj).__name__}")
        return {str(k): np.asarray(v) for k, v in obj.items()}
    raise ValueError(
        f"Unsupported motion file format for '{path}'. "
        "Use .npz, .csv (t_s + jXX columns), or .pkl/.pickle dict."
    )


def interp_motion_row(ts: np.ndarray, qs: np.ndarray, t: float) -> np.ndarray:
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
    alpha = (float(t) - t0) / (t1 - t0)
    return qs[lo] * (1.0 - alpha) + qs[hi] * alpha


@dataclass(frozen=True)
class BodySnapshot:
    positions: list[float]
    mode_machine: int


class LatestBodyState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: BodySnapshot | None = None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._cb, 200)

    def _cb(self, msg: LowState_) -> None:
        try:
            positions = [float(msg.motor_state[idx].q) for idx in range(G1_NUM_MOTOR)]
            mode_machine = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        with self._lock:
            self._latest = BodySnapshot(positions=positions, mode_machine=mode_machine)

    def snapshot(self) -> BodySnapshot | None:
        with self._lock:
            if self._latest is None:
                return None
            return BodySnapshot(positions=list(self._latest.positions), mode_machine=self._latest.mode_machine)

    def wait(self, timeout_s: float) -> BodySnapshot:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            latest = self.snapshot()
            if latest is not None:
                return latest
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


class ArmOnlyLowCmd:
    """rt/lowcmd publisher that enables PD control only for selected arm joints."""

    def __init__(self, joint_indices: list[int]) -> None:
        self.joint_indices = [int(joint) for joint in joint_indices]
        illegal = [joint for joint in self.joint_indices if joint not in ARM_LIMITS]
        if illegal:
            raise ValueError(f"Only arm joints are allowed, got: {illegal}")
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0

    def write(
        self,
        targets: dict[int, float],
        *,
        mode_machine: int,
        kp: float,
        kd: float,
        tau: float = 0.0,
    ) -> None:
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine)
        for motor_idx in range(G1_NUM_MOTOR):
            mc = self._cmd.motor_cmd[motor_idx]
            mc.mode = 0
            mc.q = 0.0
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        for joint in self.joint_indices:
            if joint not in targets:
                continue
            lo, hi = ARM_LIMITS[joint]
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = clamp(float(targets[joint]), lo, hi)
            mc.dq = 0.0
            mc.kp = float(kp)
            mc.kd = float(kd)
            mc.tau = float(tau)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_zero_gains(self, targets: dict[int, float], *, mode_machine: int) -> None:
        self.write(targets, mode_machine=mode_machine, kp=0.0, kd=0.0, tau=0.0)


class ArmSdkZeroTorque:
    """rt/arm_sdk publisher that gives arm_sdk authority with zero arm gains."""

    def __init__(self, joint_indices: list[int]) -> None:
        self.joint_indices = [int(joint) for joint in joint_indices]
        illegal = [joint for joint in self.joint_indices if joint not in ARM_LIMITS]
        if illegal:
            raise ValueError(f"Only arm joints are allowed, got: {illegal}")
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0

    def write_zero_gains(self, targets: dict[int, float]) -> None:
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0
        for joint in self.joint_indices:
            if joint not in targets:
                continue
            lo, hi = ARM_LIMITS[joint]
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = clamp(float(targets[joint]), lo, hi)
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class DevModeTeachRepeat:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.side = normalize_arm_selection(args.arm)
        self.arm_joints = list(PBD_ARM_JOINTS[self.side])
        self.rate_hz = max(1.0, float(getattr(args, "command_rate_hz", getattr(args, "rate_hz", 50.0))))
        self.dt = 1.0 / self.rate_hz
        self.state: LatestBodyState | None = None
        self.pub: ArmOnlyLowCmd | None = None
        self.arm_sdk_pub: ArmSdkZeroTorque | None = None
        self.motion: MotionSwitcherClient | None = None

    def setup(self, *, use_motion_switcher: bool = True, use_lowcmd: bool = True, use_arm_sdk: bool = False) -> None:
        ChannelFactoryInitialize(int(self.args.domain_id), str(self.args.iface))
        if use_motion_switcher:
            self.motion = MotionSwitcherClient()
            self.motion.SetTimeout(float(self.args.timeout))
            self.motion.Init()
        self.state = LatestBodyState()
        if use_lowcmd:
            self.pub = ArmOnlyLowCmd(self.arm_joints)
        if use_arm_sdk:
            self.arm_sdk_pub = ArmSdkZeroTorque(self.arm_joints)

    @staticmethod
    def current_motion_name(data: Any) -> str:
        if not isinstance(data, dict):
            return ""
        name = data.get("name")
        return "" if name is None else str(name)

    def enter_dev_mode(self) -> None:
        assert self.motion is not None
        code, data = self.motion.CheckMode()
        print(f"motion before: code={int(code)} data={data}")
        deadline = time.time() + max(0.0, float(self.args.timeout))
        while int(code) == 0 and self.current_motion_name(data):
            release_code, _ = self.motion.ReleaseMode()
            print(f"ReleaseMode(): code={int(release_code)}")
            time.sleep(0.5)
            if time.time() > deadline:
                raise TimeoutError("Timed out releasing MotionSwitcher mode.")
            code, data = self.motion.CheckMode()
            print(f"motion check: code={int(code)} data={data}")
        if int(code) != 0:
            raise RuntimeError(f"MotionSwitcher CheckMode failed with code={int(code)} data={data}")

    def snapshot(self) -> BodySnapshot:
        assert self.state is not None
        return self.state.wait(float(self.args.timeout))

    def read_arm_positions(self) -> dict[int, float]:
        snap = self.snapshot()
        values: dict[int, float] = {}
        for joint in self.arm_joints:
            q_val = snap.positions[joint]
            lo, hi = ARM_LIMITS[joint]
            values[joint] = clamp(float(q_val), lo, hi)
        return values

    def write_arm_targets(self, targets: dict[int, float], *, kp: float, kd: float) -> None:
        assert self.pub is not None
        mode_machine = self.snapshot().mode_machine
        self.pub.write(targets, mode_machine=mode_machine, kp=kp, kd=kd)

    def zero_arm_gains(self, targets: dict[int, float] | None = None) -> None:
        assert self.pub is not None
        snap = self.snapshot()
        safe_targets = targets if targets is not None else {
            joint: clamp(snap.positions[joint], *ARM_LIMITS[joint])
            for joint in self.arm_joints
        }
        self.pub.write_zero_gains(safe_targets, mode_machine=snap.mode_machine)

    def zero_arm_sdk_gains(self, targets: dict[int, float] | None = None) -> None:
        assert self.arm_sdk_pub is not None
        if targets is None:
            snap = self.snapshot()
            targets = {
                joint: clamp(snap.positions[joint], *ARM_LIMITS[joint])
                for joint in self.arm_joints
            }
        self.arm_sdk_pub.write_zero_gains(targets)

    def ramp_to(
        self,
        start_targets: dict[int, float],
        end_targets: dict[int, float],
        *,
        duration_s: float,
        kp: float,
        kd: float,
    ) -> None:
        steps = max(1, int(max(0.0, float(duration_s)) * self.rate_hz))
        for step_idx in range(1, steps + 1):
            ratio = smoothstep(float(step_idx) / float(steps))
            targets = {
                joint: float(start_targets[joint]) + (float(end_targets[joint]) - float(start_targets[joint])) * ratio
                for joint in self.arm_joints
            }
            self.write_arm_targets(targets, kp=kp, kd=kd)
            time.sleep(self.dt)

    def hold(self, targets: dict[int, float], *, duration_s: float, kp: float, kd: float) -> None:
        steps = max(1, int(max(0.0, float(duration_s)) * self.rate_hz))
        for _ in range(steps):
            self.write_arm_targets(targets, kp=kp, kd=kd)
            time.sleep(self.dt)

    def teach(self) -> dict[str, Any]:
        if not self.args.yes:
            print("Dry run only. Re-run with --yes to publish zero-gain arm commands on rt/arm_sdk.")
            for joint in self.arm_joints:
                lo, hi = ARM_LIMITS[joint]
                print(f"arm {joint:02d} {BODY_JOINT_NAME_BY_INDEX[joint]} limits=[{lo:+.4f}, {hi:+.4f}]")
            return {"dry_run": True, "arm": self.side, "joint_count": len(self.arm_joints)}

        self.setup(use_motion_switcher=False, use_lowcmd=False, use_arm_sdk=True)
        arm_positions = self.read_arm_positions()
        self.zero_arm_sdk_gains(arm_positions)

        done_event = threading.Event()

        def wait_for_enter() -> None:
            try:
                input("Press Enter when the teach motion is complete...")
            except EOFError:
                return
            done_event.set()

        prompt_thread = threading.Thread(target=wait_for_enter, name="dev-mode-teach-enter", daemon=True)
        prompt_thread.start()

        out = str(self.args.out)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        resolved_log_path = self.args.log_path or f"{os.path.splitext(out)[0]}.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        sample_period = max(1e-3, float(self.args.poll_s))
        duration_limit = max(0.0, float(self.args.duration_s))
        timestamps: list[float] = []
        samples: list[list[float]] = []
        start = time.time()
        next_tick = start
        duration_notice_sent = False

        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["phase", "t_s", "joint_indices", "target_positions", "actual_positions"])
            try:
                while True:
                    now = time.time()
                    if now < next_tick:
                        time.sleep(min(0.02, next_tick - now))
                        continue
                    next_tick += sample_period
                    if done_event.is_set():
                        break
                    if duration_limit > 0.0 and (now - start) >= duration_limit:
                        if not duration_notice_sent:
                            print("Teach duration limit reached. Press Enter to finish recording.")
                            duration_notice_sent = True

                    snap = self.snapshot()
                    row = [
                        clamp(float(snap.positions[joint]), *ARM_LIMITS[joint])
                        for joint in self.arm_joints
                    ]
                    zero_targets = {
                        joint: float(row[idx])
                        for idx, joint in enumerate(self.arm_joints)
                    }
                    self.zero_arm_sdk_gains(zero_targets)
                    t_rel = now - start
                    timestamps.append(t_rel)
                    samples.append(row)
                    writer.writerow(
                        [
                            "teach",
                            f"{t_rel:.6f}",
                            " ".join(str(joint) for joint in self.arm_joints),
                            " ".join(f"{value:.6f}" for value in row),
                            " ".join(f"{value:.6f}" for value in row),
                        ]
                    )
                    handle.flush()
                    print(
                        f"[teach] t={t_rel:.3f}s joints={self.arm_joints} "
                        f"actual={[round(value, 4) for value in row]}"
                    )
            except KeyboardInterrupt:
                pass

        if not timestamps:
            raise RuntimeError("No samples recorded. Is rt/lowstate publishing?")

        np.savez(
            out,
            joints=np.asarray(self.arm_joints, dtype=np.int32),
            ts=np.asarray(timestamps, dtype=np.float32),
            qs=np.asarray(samples, dtype=np.float32),
            poll_s=np.asarray([sample_period], dtype=np.float32),
            representation=np.asarray(["joint_space"], dtype="<U16"),
            control_topic=np.asarray(["rt/arm_sdk"], dtype="<U16"),
            targeted_joints=np.asarray(["arms_only"], dtype="<U16"),
        )

        final_targets = {
            joint: float(samples[-1][idx])
            for idx, joint in enumerate(self.arm_joints)
        }
        zero_after = max(0.0, float(self.args.zero_after_teach_s))
        if zero_after > 0.0:
            steps = max(1, int(zero_after / sample_period))
            for _ in range(steps):
                self.zero_arm_sdk_gains(final_targets)
                time.sleep(sample_period)
        self.zero_arm_sdk_gains(final_targets)
        return {
            "arm": self.side,
            "joint_count": len(self.arm_joints),
            "sample_count": len(timestamps),
            "duration_s": float(timestamps[-1]) if timestamps else 0.0,
            "poll_s": sample_period,
            "out": os.path.abspath(out),
            "log_path": os.path.abspath(resolved_log_path),
            "targeted_joints": list(self.arm_joints),
        }

    def repeat(self) -> dict[str, Any]:
        data = load_pbd_motion_file(str(self.args.motion_file))
        if "joints" not in data or "ts" not in data or "qs" not in data:
            raise ValueError("Motion file must contain 'joints', 'ts', and 'qs'.")
        recorded_joints = [int(joint) for joint in np.asarray(data["joints"]).astype(int).tolist()]
        ts = np.asarray(data["ts"], dtype=float)
        qs = np.asarray(data["qs"], dtype=float)
        if ts.size == 0 or qs.size == 0:
            raise ValueError("No samples in motion file.")
        if qs.shape[0] != ts.shape[0]:
            raise ValueError("Invalid motion file: ts and qs length mismatch.")
        if qs.shape[1] != len(recorded_joints):
            raise ValueError("Invalid motion file: joints and qs width mismatch.")

        joint_to_col = {joint: idx for idx, joint in enumerate(recorded_joints)}
        missing = [joint for joint in self.arm_joints if joint not in joint_to_col]
        if missing:
            raise ValueError(f"Motion file missing required joints for arm={self.side}: {missing}.")
        active_cols = [joint_to_col[joint] for joint in self.arm_joints]
        active_qs = qs[:, active_cols]

        if not self.args.yes:
            print("Dry run only. Re-run with --yes to release motion switcher and publish rt/lowcmd.")
            print(f"motion_file={os.path.abspath(str(self.args.motion_file))}")
            print(f"arm={self.side} joints={self.arm_joints} samples={ts.shape[0]}")
            return {"dry_run": True, "arm": self.side, "joint_count": len(self.arm_joints)}

        self.setup(use_motion_switcher=True)
        self.enter_dev_mode()
        resolved_log_path = self.args.log_path or f"{os.path.splitext(str(self.args.motion_file))[0]}_dev_repeat.csv"
        os.makedirs(os.path.dirname(os.path.abspath(resolved_log_path)), exist_ok=True)

        start_positions = self.read_arm_positions()
        first_targets = {
            joint: clamp(float(active_qs[0, idx]), *ARM_LIMITS[joint])
            for idx, joint in enumerate(self.arm_joints)
        }
        self.ramp_to(
            start_positions,
            first_targets,
            duration_s=max(0.0, float(self.args.start_ramp_s)),
            kp=float(self.args.kp),
            kd=float(self.args.kd),
        )

        replay_ts = ts / max(1e-6, float(self.args.speed))
        t_final = float(replay_ts[-1])
        started = time.time()
        with open(resolved_log_path, "w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["phase", "t_s", "joint_indices", "target_positions", "actual_positions"])
            while True:
                elapsed = time.time() - started
                if elapsed > t_final:
                    break
                desired_row = np.asarray(interp_motion_row(replay_ts, active_qs, elapsed), dtype=float)
                targets = {
                    joint: clamp(float(desired_row[idx]), *ARM_LIMITS[joint])
                    for idx, joint in enumerate(self.arm_joints)
                }
                self.write_arm_targets(targets, kp=float(self.args.kp), kd=float(self.args.kd))
                snap = self.snapshot()
                actual_row = [
                    clamp(float(snap.positions[joint]), *ARM_LIMITS[joint])
                    for joint in self.arm_joints
                ]
                target_row = [float(targets[joint]) for joint in self.arm_joints]
                writer.writerow(
                    [
                        "repeat",
                        f"{elapsed:.6f}",
                        " ".join(str(joint) for joint in self.arm_joints),
                        " ".join(f"{value:.6f}" for value in target_row),
                        " ".join(f"{value:.6f}" for value in actual_row),
                    ]
                )
                handle.flush()
                print(
                    f"[repeat] t={elapsed:.3f}s joints={self.arm_joints} "
                    f"target={[round(value, 4) for value in target_row]} "
                    f"actual={[round(value, 4) for value in actual_row]}"
                )
                time.sleep(self.dt)

            final_targets = {
                joint: clamp(float(active_qs[-1, idx]), *ARM_LIMITS[joint])
                for idx, joint in enumerate(self.arm_joints)
            }
            hold_deadline = time.time() + max(0.0, float(self.args.final_hold_s))
            while True:
                self.write_arm_targets(final_targets, kp=float(self.args.kp), kd=float(self.args.kd))
                snap = self.snapshot()
                actual_row = [
                    clamp(float(snap.positions[joint]), *ARM_LIMITS[joint])
                    for joint in self.arm_joints
                ]
                target_row = [float(final_targets[joint]) for joint in self.arm_joints]
                writer.writerow(
                    [
                        "repeat_final_hold",
                        f"{time.time() - started:.6f}",
                        " ".join(str(joint) for joint in self.arm_joints),
                        " ".join(f"{value:.6f}" for value in target_row),
                        " ".join(f"{value:.6f}" for value in actual_row),
                    ]
                )
                handle.flush()
                print(
                    f"[repeat_final_hold] joints={self.arm_joints} "
                    f"target={[round(value, 4) for value in target_row]} "
                    f"actual={[round(value, 4) for value in actual_row]}"
                )
                if time.time() >= hold_deadline:
                    break
                time.sleep(self.dt)

        if self.args.zero_gains_on_exit:
            self.zero_arm_gains(final_targets)
        return {
            "arm": self.side,
            "motion_file": os.path.abspath(str(self.args.motion_file)),
            "joint_count": len(self.arm_joints),
            "sample_count": int(ts.shape[0]),
            "command_rate_hz": self.rate_hz,
            "speed": max(1e-6, float(self.args.speed)),
            "duration_s": t_final,
            "final_hold_s": max(0.0, float(self.args.final_hold_s)),
            "log_path": os.path.abspath(resolved_log_path),
            "targeted_joints": list(self.arm_joints),
        }


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--iface", default="eth0", help="DDS network interface.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain ID.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for SDK RPC/state.")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both", help="Arm joints to target.")
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Actually publish robot commands.",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Programming-by-demonstration for G1 arm joints. "
            "Teach records with zero-gain rt/arm_sdk arm authority; repeat replays through rt/lowcmd."
        )
    )
    sub = parser.add_subparsers(dest="command", required=True)

    teach_parser = sub.add_parser("teach", help="Record selected arm joint poses from rt/lowstate.")
    add_common_args(teach_parser)
    teach_parser.add_argument("--out", default="/tmp/pbd_motion.npz", help="Output .npz path.")
    teach_parser.add_argument("--log-path", default=None, help="Optional CSV log path.")
    teach_parser.add_argument("--duration-s", type=float, default=0.0, help="Optional duration limit; 0 waits for Enter.")
    teach_parser.add_argument("--poll-s", type=float, default=0.01, help="Recording poll period.")
    teach_parser.add_argument(
        "--zero-after-teach-s",
        type=float,
        default=0.2,
        help="Continue publishing zero-gain rt/arm_sdk arm packets briefly after recording.",
    )
    teach_parser.set_defaults(func="teach")

    repeat_parser = sub.add_parser("repeat", help="Replay a saved arm joint trajectory through rt/lowcmd.")
    add_common_args(repeat_parser)
    repeat_parser.add_argument("--motion-file", default="/tmp/pbd_motion.npz", help="Input .npz/.csv/.pkl motion file.")
    repeat_parser.add_argument("--log-path", default=None, help="Optional CSV log path.")
    repeat_parser.add_argument("--speed", type=float, default=1.0, help="Replay speed multiplier.")
    repeat_parser.add_argument("--command-rate-hz", type=float, default=50.0, help="Lowcmd publish rate.")
    repeat_parser.add_argument("--start-ramp-s", type=float, default=0.8, help="Ramp from current pose to first sample.")
    repeat_parser.add_argument("--final-hold-s", type=float, default=0.8, help="Hold final sample before exit.")
    repeat_parser.add_argument("--kp", type=float, default=40.0, help="Arm kp during replay.")
    repeat_parser.add_argument("--kd", type=float, default=1.0, help="Arm kd during replay.")
    repeat_parser.add_argument("--zero-gains-on-exit", action="store_true", help="Release selected arm gains after replay.")
    repeat_parser.set_defaults(func="repeat")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    runner = DevModeTeachRepeat(args)
    result = runner.teach() if args.func == "teach" else runner.repeat()
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
