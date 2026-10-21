#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

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

try:
    from sdk_boot import hanger_boot_sequence
except ImportError:
    hanger_boot_sequence = None


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_IDX = [12, 13, 14]
NOT_USED_IDX = 29
DEFAULT_POSE_FILE = os.path.join(SCRIPT_DIR, "saved_arm_poses.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a saved arm pose through rt/arm_sdk and hold it there."
    )
    parser.add_argument("--file", default=DEFAULT_POSE_FILE, help="Saved JSON pose file.")
    parser.add_argument("--pose", default=None, help="Pose name to replay.")
    parser.add_argument("--index", type=int, default=None, help="Pose index to replay if no name is given.")
    parser.add_argument("--list", action="store_true", help="List poses in the file and exit.")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument(
        "--speed-rad-s",
        type=float,
        default=0.2,
        help="Maximum arm joint speed while ramping to the target pose.",
    )
    parser.add_argument("--kp", type=float, default=30.0, help="Arm hold proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm hold derivative gain.")
    parser.add_argument("--waist-kp", type=float, default=30.0, help="Waist hold proportional gain.")
    parser.add_argument("--waist-kd", type=float, default=1.5, help="Waist hold derivative gain.")
    parser.add_argument(
        "--use-saved-waist",
        action="store_true",
        help="Replay the saved waist joints from the pose file instead of holding the current waist pose.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="How long to hold after reaching the pose. Use 0 to hold until Ctrl-C.",
    )
    parser.add_argument(
        "--tolerance-rad",
        type=float,
        default=0.05,
        help="Measured joint error threshold used to decide whether the target pose was actually reached.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print detailed joint diagnostics before ramping, after the ramp, and while holding.",
    )
    parser.add_argument(
        "--debug-interval",
        type=float,
        default=1.0,
        help="Seconds between live diagnostic prints while holding when --debug is enabled.",
    )
    parser.add_argument(
        "--stall-window",
        type=float,
        default=1.5,
        help="Seconds used to judge whether a joint is stalled while large error remains.",
    )
    parser.add_argument(
        "--stall-progress-rad",
        type=float,
        default=0.01,
        help="Minimum motion over the stall window that counts as making progress.",
    )
    parser.add_argument(
        "--run-hanged-boot",
        action="store_true",
        help="Run the hanger boot sequence before starting replay.",
    )
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


def arm_selection_to_joints(arm: str) -> list[int]:
    side = str(arm).strip().lower()
    if side == "left":
        return list(LEFT_ARM_IDX)
    if side == "right":
        return list(RIGHT_ARM_IDX)
    if side == "both":
        return list(LEFT_ARM_IDX) + list(RIGHT_ARM_IDX)
    raise ValueError(f"Unsupported arm selection '{arm}'.")


def load_pose_file(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"Pose file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Pose file is not valid JSON: {path}: {exc}") from exc


def list_poses(payload: dict[str, Any]) -> list[dict[str, Any]]:
    poses = payload.get("poses")
    if not isinstance(poses, list):
        raise SystemExit("Pose file does not contain a 'poses' list.")
    return poses


def choose_pose(poses: list[dict[str, Any]], *, pose_name: str | None, pose_index: int | None) -> dict[str, Any]:
    if pose_name:
        matches = [
            (idx, pose)
            for idx, pose in enumerate(poses)
            if str(pose.get("name", "")) == pose_name
        ]
        if matches:
            if len(matches) > 1:
                match_indexes = [idx for idx, _pose in matches]
                print(
                    f"Pose name '{pose_name}' appears multiple times at indices {match_indexes}; "
                    f"using the most recent entry at index {matches[-1][0]}."
                )
            else:
                print(f"Selected pose '{pose_name}' at index {matches[0][0]}.")
            return matches[-1][1]
        raise SystemExit(f"Pose named '{pose_name}' not found.")

    if pose_index is not None:
        if pose_index < 0 or pose_index >= len(poses):
            raise SystemExit(f"Pose index {pose_index} out of range [0, {len(poses) - 1}].")
        print(f"Selected pose index {pose_index}.")
        return poses[pose_index]

    if len(poses) == 1:
        print("Selected the only pose in the file.")
        return poses[0]

    raise SystemExit("Choose a pose with --pose NAME or --index N, or use --list.")


def parse_arm_targets(pose: dict[str, Any]) -> tuple[str, list[int], np.ndarray, np.ndarray | None]:
    arm_selection = str(pose.get("arm_selection", "")).strip().lower()
    arm_joints = arm_selection_to_joints(arm_selection)

    raw_targets = pose.get("arm_joints")
    if not isinstance(raw_targets, dict):
        raise SystemExit("Pose entry does not contain an 'arm_joints' mapping.")

    target_positions = []
    for joint in arm_joints:
        key = str(joint)
        if key not in raw_targets:
            raise SystemExit(f"Pose entry is missing arm joint {joint}.")
        target_positions.append(float(raw_targets[key]))

    waist_targets_raw = pose.get("waist_joints")
    waist_targets: np.ndarray | None = None
    if isinstance(waist_targets_raw, dict):
        waist_targets = np.asarray(
            [float(waist_targets_raw[str(joint)]) for joint in WAIST_IDX],
            dtype=np.float32,
        )

    return arm_selection, arm_joints, np.asarray(target_positions, dtype=np.float32), waist_targets


class RobotStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: list[float] | None = None
        self._timestamp = 0.0

        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        self._sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            positions = [float(msg.motor_state[j].q) for j in self.joints]
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[np.ndarray, float] | None:
        with self._lock:
            if self._positions is None:
                return None
            return np.asarray(self._positions, dtype=np.float32), float(self._timestamp)

    def wait_for_snapshot(self, timeout_s: float = 3.0) -> np.ndarray:
        deadline = time.time() + max(0.1, float(timeout_s))
        while time.time() < deadline:
            snap = self.snapshot()
            if snap is not None:
                return snap[0]
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate joint data.")


def format_joint_diagnostics(
    joints: list[int],
    *,
    current: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray | None = None,
) -> list[str]:
    lines: list[str] = []
    for idx, joint in enumerate(joints):
        start_val = float(current[idx])
        target_val = float(target[idx])
        delta_val = target_val - start_val
        if actual is None:
            lines.append(
                f"  joint {joint}: start={start_val:+.4f} target={target_val:+.4f} delta={delta_val:+.4f}"
            )
            continue
        actual_val = float(actual[idx])
        error_val = target_val - actual_val
        lines.append(
            "  joint "
            f"{joint}: start={start_val:+.4f} target={target_val:+.4f} actual={actual_val:+.4f} "
            f"cmd_delta={delta_val:+.4f} err={error_val:+.4f}"
        )
    return lines


def print_joint_diagnostics(
    title: str,
    joints: list[int],
    *,
    current: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray | None = None,
) -> None:
    print(title)
    for line in format_joint_diagnostics(joints, current=current, target=target, actual=actual):
        print(line)


def print_tracking_summary(
    joints: list[int],
    start_positions: np.ndarray,
    target_positions: np.ndarray,
    actual_positions: np.ndarray,
    *,
    tolerance_rad: float,
    label: str,
) -> float:
    errors = target_positions - actual_positions
    abs_errors = np.abs(errors)
    max_error = float(np.max(abs_errors)) if abs_errors.size else 0.0
    mean_error = float(np.mean(abs_errors)) if abs_errors.size else 0.0
    worst_index = int(np.argmax(abs_errors)) if abs_errors.size else 0
    worst_joint = joints[worst_index] if joints else -1
    status = "OK" if max_error <= float(tolerance_rad) else "WARN"
    print(
        f"{label}: {status} max_abs_err={max_error:.4f} rad mean_abs_err={mean_error:.4f} rad "
        f"worst_joint={worst_joint} tolerance={float(tolerance_rad):.4f} rad"
    )
    print_joint_diagnostics(
        f"{label} per-joint diagnostics:",
        joints,
        current=start_positions,
        target=target_positions,
        actual=actual_positions,
    )
    return max_error


def print_stall_summary(
    joints: list[int],
    *,
    earlier_positions: np.ndarray,
    latest_positions: np.ndarray,
    target_positions: np.ndarray,
    elapsed_s: float,
    tolerance_rad: float,
    min_progress_rad: float,
) -> None:
    progress = latest_positions - earlier_positions
    remaining_error = target_positions - latest_positions
    stalled_lines: list[str] = []
    for idx, joint in enumerate(joints):
        abs_err = abs(float(remaining_error[idx]))
        abs_progress = abs(float(progress[idx]))
        if abs_err > float(tolerance_rad) and abs_progress < float(min_progress_rad):
            stalled_lines.append(
                f"  joint {joint}: moved={float(progress[idx]):+.4f} rad in {elapsed_s:.2f}s, "
                f"remaining_err={float(remaining_error[idx]):+.4f} rad"
            )
    if stalled_lines:
        print(
            f"Potential stall detected over {elapsed_s:.2f}s "
            f"(progress threshold {float(min_progress_rad):.4f} rad):"
        )
        for line in stalled_lines:
            print(line)


class ArmReplayController:
    def __init__(
        self,
        arm_joints: list[int],
        waist_joints: list[int],
        *,
        kp: float,
        kd: float,
        waist_kp: float,
        waist_kd: float,
    ) -> None:
        self.arm_joints = [int(j) for j in arm_joints]
        self.waist_joints = [int(j) for j in waist_joints]
        self.kp = float(kp)
        self.kd = float(kd)
        self.waist_kp = float(waist_kp)
        self.waist_kd = float(waist_kd)

        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0

        managed_joints = self.arm_joints + self.waist_joints
        self._last_pose = {joint: 0.0 for joint in managed_joints}
        for joint in managed_joints:
            self._cmd.motor_cmd[joint].mode = 1

    def seed_pose(self, joint_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.arm_joints + self.waist_joints, joint_positions):
            self._last_pose[joint] = float(q_val)

    def set_waist_target(self, waist_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.waist_joints, waist_positions):
            self._last_pose[joint] = float(q_val)

    def write_pose(self, arm_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.arm_joints, arm_positions):
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(q_val)
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
            self._last_pose[joint] = float(q_val)

        for joint in self.waist_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._last_pose[joint]
            mc.dq = 0.0
            mc.kp = self.waist_kp
            mc.kd = self.waist_kd
            mc.tau = 0.0

        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def ramp_to_pose(self, target_positions: np.ndarray, *, speed_rad_s: float, rate_hz: float) -> None:
        target = np.asarray(target_positions, dtype=np.float32)
        start = np.asarray([self._last_pose[joint] for joint in self.arm_joints], dtype=np.float32)
        max_delta = float(np.max(np.abs(target - start))) if target.size else 0.0
        if max_delta <= 1e-6:
            self.write_pose(target)
            return

        duration_s = max(0.1, max_delta / max(0.01, float(speed_rad_s)))
        steps = max(1, int(duration_s * max(1.0, float(rate_hz))))
        dt = 1.0 / max(1.0, float(rate_hz))
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            pose = start + (target - start) * alpha
            self.write_pose(pose)
            time.sleep(dt)

    def hold_pose(self, arm_positions: np.ndarray, *, stop_event: threading.Event, rate_hz: float) -> None:
        dt = 1.0 / max(1.0, float(rate_hz))
        while not stop_event.is_set():
            self.write_pose(arm_positions)
            time.sleep(dt)


def main() -> int:
    args = parse_args()
    pose_path = Path(os.path.abspath(os.path.expanduser(args.file)))
    payload = load_pose_file(pose_path)
    poses = list_poses(payload)

    if args.list:
        for idx, pose in enumerate(poses):
            name = str(pose.get("name", f"pose_{idx}"))
            arm_selection = str(pose.get("arm_selection", "?"))
            saved_at = str(pose.get("saved_at", "?"))
            print(f"{idx}: name={name} arm={arm_selection} saved_at={saved_at}")
        return 0

    pose = choose_pose(poses, pose_name=args.pose, pose_index=args.index)
    arm_selection, arm_joints, target_positions, saved_waist_positions = parse_arm_targets(pose)

    if args.run_hanged_boot:
        if hanger_boot_sequence is None:
            raise SystemExit("sdk_boot.py not available, cannot run hanger boot sequence.")
        hanger_boot_sequence(iface=args.iface, domain_id=args.domain_id)

    ChannelFactoryInitialize(int(args.domain_id), args.iface)

    state_joints = arm_joints + list(WAIST_IDX)
    state_sub = RobotStateSubscriber(state_joints)
    controller = ArmReplayController(
        arm_joints,
        list(WAIST_IDX),
        kp=float(args.kp),
        kd=float(args.kd),
        waist_kp=float(args.waist_kp),
        waist_kd=float(args.waist_kd),
    )

    current_positions = state_sub.wait_for_snapshot()
    controller.seed_pose(current_positions)
    current_arm_positions = current_positions[: len(arm_joints)].copy()
    current_waist_positions = current_positions[len(arm_joints):].copy()
    active_waist_positions = current_waist_positions.copy()
    if args.use_saved_waist and saved_waist_positions is not None:
        active_waist_positions = saved_waist_positions.copy()
        controller.set_waist_target(active_waist_positions)

    print(f"Replaying pose '{pose.get('name', '<unnamed>')}' for arm selection '{arm_selection}'.")
    print(f"Pose saved_at: {pose.get('saved_at', '?')}")
    print(f"Target arm joints: {arm_joints}")
    print(f"Target values: {[round(float(v), 4) for v in target_positions.tolist()]}")
    print(f"Start arm values: {[round(float(v), 4) for v in current_arm_positions.tolist()]}")
    print(f"Start waist values: {[round(float(v), 4) for v in current_waist_positions.tolist()]}")
    if saved_waist_positions is not None:
        print(f"Saved waist values: {[round(float(v), 4) for v in saved_waist_positions.tolist()]}")
        print(
            f"Waist delta saved-current: "
            f"{[round(float(v), 4) for v in (saved_waist_positions - current_waist_positions).tolist()]}"
        )
    else:
        print("Saved waist values: <not present in pose file>")
    print(
        f"Ramping with speed limit {float(args.speed_rad_s):.3f} rad/s and holding waist joints {WAIST_IDX} "
        f"at {'their saved pose' if args.use_saved_waist and saved_waist_positions is not None else 'their current pose'}."
    )
    if args.debug:
        print_joint_diagnostics(
            "Pre-ramp diagnostics:",
            arm_joints,
            current=current_arm_positions,
            target=target_positions,
        )

    stop_event = threading.Event()

    def _handle_signal(_signum: int, _frame: Any) -> None:
        stop_event.set()

    previous_sigint = signal.signal(signal.SIGINT, _handle_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, _handle_signal)
    try:
        controller.ramp_to_pose(
            target_positions,
            speed_rad_s=float(args.speed_rad_s),
            rate_hz=float(args.rate_hz),
        )
        time.sleep(min(0.25, 1.0 / max(1.0, float(args.rate_hz))))
        post_ramp_snapshot = state_sub.wait_for_snapshot()
        post_ramp_arm = post_ramp_snapshot[: len(arm_joints)].copy()
        max_error = print_tracking_summary(
            arm_joints,
            current_arm_positions,
            target_positions,
            post_ramp_arm,
            tolerance_rad=float(args.tolerance_rad),
            label="Post-ramp tracking",
        )
        if max_error <= float(args.tolerance_rad):
            print("Measured arm state is within tolerance. Holding...")
        else:
            print("Measured arm state is outside tolerance. Holding and continuing to report live error...")

        stall_reference_positions = post_ramp_arm.copy()
        stall_reference_time = time.time()

        if float(args.hold_seconds) > 0.0:
            deadline = time.time() + float(args.hold_seconds)
            dt = 1.0 / max(1.0, float(args.rate_hz))
            next_debug_time = time.time()
            while not stop_event.is_set() and time.time() < deadline:
                controller.write_pose(target_positions)
                if args.debug and time.time() >= next_debug_time:
                    snap = state_sub.snapshot()
                    if snap is not None:
                        live_positions, live_timestamp = snap
                        live_arm = live_positions[: len(arm_joints)].copy()
                        live_age = max(0.0, time.time() - live_timestamp)
                        print_tracking_summary(
                            arm_joints,
                            current_arm_positions,
                            target_positions,
                            live_arm,
                            tolerance_rad=float(args.tolerance_rad),
                            label=f"Hold tracking @ age={live_age:.3f}s",
                        )
                        stall_elapsed = time.time() - stall_reference_time
                        if stall_elapsed >= max(0.25, float(args.stall_window)):
                            print_stall_summary(
                                arm_joints,
                                earlier_positions=stall_reference_positions,
                                latest_positions=live_arm,
                                target_positions=target_positions,
                                elapsed_s=stall_elapsed,
                                tolerance_rad=float(args.tolerance_rad),
                                min_progress_rad=float(args.stall_progress_rad),
                            )
                            stall_reference_positions = live_arm.copy()
                            stall_reference_time = time.time()
                    next_debug_time = time.time() + max(0.1, float(args.debug_interval))
                time.sleep(dt)
        else:
            dt = 1.0 / max(1.0, float(args.rate_hz))
            next_debug_time = time.time()
            while not stop_event.is_set():
                controller.write_pose(target_positions)
                if args.debug and time.time() >= next_debug_time:
                    snap = state_sub.snapshot()
                    if snap is not None:
                        live_positions, live_timestamp = snap
                        live_arm = live_positions[: len(arm_joints)].copy()
                        live_age = max(0.0, time.time() - live_timestamp)
                        print_tracking_summary(
                            arm_joints,
                            current_arm_positions,
                            target_positions,
                            live_arm,
                            tolerance_rad=float(args.tolerance_rad),
                            label=f"Hold tracking @ age={live_age:.3f}s",
                        )
                        stall_elapsed = time.time() - stall_reference_time
                        if stall_elapsed >= max(0.25, float(args.stall_window)):
                            print_stall_summary(
                                arm_joints,
                                earlier_positions=stall_reference_positions,
                                latest_positions=live_arm,
                                target_positions=target_positions,
                                elapsed_s=stall_elapsed,
                                tolerance_rad=float(args.tolerance_rad),
                                min_progress_rad=float(args.stall_progress_rad),
                            )
                            stall_reference_positions = live_arm.copy()
                            stall_reference_time = time.time()
                    next_debug_time = time.time() + max(0.1, float(args.debug_interval))
                time.sleep(dt)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
