#!/usr/bin/env python3
from __future__ import annotations
from dds_env import ensure_cyclonedds_environment

import argparse
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)


ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
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
JOINT_NAMES = [
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_pitch",
    "wrist_roll",
    "wrist_yaw",
]
MIRROR_SIGNS = np.asarray([1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0], dtype=np.float32)
DEFAULT_TRAJECTORY_FILE = os.path.join(SCRIPT_DIR, "saved_shake_hand_trajectories.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record the loco ShakeHand right-arm trajectory and replay raise/drop phases on either arm."
    )
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--file", default=DEFAULT_TRAJECTORY_FILE, help="JSON trajectory file.")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="Capture and command publish rate.")
    parser.add_argument(
        "--run-hanged-boot",
        action="store_true",
        help="Run the hanger boot sequence before the selected command.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser(
        "record", help="Call LocoClient.ShakeHand() and save the right arm trajectory.")
    record.add_argument("--seconds", type=float, default=4.0,
                        help="Seconds to capture after ShakeHand().")
    record.add_argument("--name", default=None, help="Optional recording name.")
    record.add_argument("--yes", action="store_true", help="Skip the operator confirmation prompt.")

    replay = subparsers.add_parser(
        "replay", help="Replay a saved trajectory phase on the selected arm.")
    replay.add_argument("--arm", choices=("left", "right"), required=True, help="Arm to replay on.")
    replay.add_argument("--phase", choices=("raise", "drop"),
                        required=True, help="Trajectory phase to replay.")
    replay.add_argument("--index", type=int, default=None,
                        help="Recording index. Defaults to latest.")
    replay.add_argument("--name", default=None,
                        help="Recording name. Uses the newest matching entry.")
    replay.add_argument(
        "--max-increment-rad",
        type=float,
        default=0.01,
        help="Maximum per-joint command change per publish tick.",
    )
    replay.add_argument("--kp", type=float, default=30.0, help="Arm replay proportional gain.")
    replay.add_argument("--kd", type=float, default=1.5, help="Arm replay derivative gain.")
    replay.add_argument("--waist-kp", type=float, default=30.0,
                        help="Waist hold proportional gain.")
    replay.add_argument("--waist-kd", type=float, default=1.5, help="Waist hold derivative gain.")
    replay.add_argument("--hold-seconds", type=float, default=1.0,
                        help="Hold final command before exiting.")
    replay.add_argument("--hold-forever", action="store_true",
                        help="Hold final command until Ctrl-C.")

    subparsers.add_parser("list", help="List saved trajectory recordings.")
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


def selected_arm_joints(arm: str) -> list[int]:
    if arm == "left":
        return list(LEFT_ARM_IDX)
    if arm == "right":
        return list(RIGHT_ARM_IDX)
    raise ValueError(f"Unsupported arm '{arm}'.")


def step_towards(current: np.ndarray, target: np.ndarray, max_delta: float) -> np.ndarray:
    delta = target - current
    if max_delta <= 0.0:
        return target.astype(np.float32)
    clipped = np.clip(delta, -float(max_delta), float(max_delta))
    next_value = current + clipped
    close_mask = np.abs(delta) <= float(max_delta)
    next_value[close_mask] = target[close_mask]
    return next_value.astype(np.float32)


def split_index_for_samples(samples: list[dict[str, Any]]) -> int:
    if len(samples) < 2:
        return 0
    start = np.asarray(samples[0]["right_arm"], dtype=np.float32)
    distances = [
        float(np.linalg.norm(np.asarray(sample["right_arm"], dtype=np.float32) - start))
        for sample in samples
    ]
    return int(np.argmax(distances))


def phase_samples(record: dict[str, Any], phase: str) -> list[dict[str, Any]]:
    samples = record.get("samples", [])
    if not isinstance(samples, list) or not samples:
        return []
    split_index = int(record.get("split_index", split_index_for_samples(samples)))
    split_index = max(0, min(split_index, len(samples) - 1))
    if phase == "raise":
        return samples[: split_index + 1]
    if phase == "drop":
        return samples[split_index:]
    raise ValueError(f"Unsupported phase '{phase}'.")


def map_right_trajectory_to_arm(samples: list[dict[str, Any]], arm: str) -> list[np.ndarray]:
    mapped: list[np.ndarray] = []
    for sample in samples:
        values = np.asarray(sample["right_arm"], dtype=np.float32)
        if values.shape != (7,):
            raise ValueError("Trajectory sample does not contain 7 right arm joints.")
        if arm == "left":
            values = values * MIRROR_SIGNS
        mapped.append(values.astype(np.float32))
    return mapped


def load_payload(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"records": []}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Trajectory file is not valid JSON: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise SystemExit(f"Trajectory file root must be a JSON object: {path}")
    if not isinstance(payload.get("records"), list):
        payload["records"] = []
    return payload


def load_records(path: Path) -> list[dict[str, Any]]:
    payload = load_payload(path)
    return [record for record in payload.get("records", []) if isinstance(record, dict)]


def save_record(path: Path, record: dict[str, Any]) -> int:
    payload = load_payload(path)
    records = payload.get("records")
    if not isinstance(records, list):
        records = []
        payload["records"] = records
    records.append(record)
    payload["latest_record_index"] = len(records) - 1
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return len(records) - 1


def choose_record(records: list[dict[str, Any]], *, name: str | None, index: int | None) -> tuple[int, dict[str, Any]]:
    if not records:
        raise SystemExit("No saved recordings. Run the record command first.")
    if name is not None:
        matches = [(idx, record) for idx, record in enumerate(
            records) if str(record.get("name", "")) == name]
        if not matches:
            raise SystemExit(f"No recording named '{name}'.")
        return matches[-1]
    if index is not None:
        if index < 0 or index >= len(records):
            raise SystemExit(f"Recording index {index} out of range [0, {len(records) - 1}].")
        return index, records[index]
    return len(records) - 1, records[-1]


class RobotStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: dict[int, float] = {}
        self._timestamp = 0.0

        lowstate_type = resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        self._sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        self._sub.Init(self._callback, 200)

    def _callback(self, msg: Any) -> None:
        try:
            positions = {joint: float(msg.motor_state[joint].q) for joint in self.joints}
        except Exception:
            return
        with self._lock:
            self._positions = positions
            self._timestamp = time.time()

    def snapshot(self) -> tuple[dict[int, float], float] | None:
        with self._lock:
            if not self._positions:
                return None
            return dict(self._positions), float(self._timestamp)

    def wait_for_snapshot(self, timeout_s: float = 3.0) -> tuple[dict[int, float], float]:
        deadline = time.time() + max(0.1, float(timeout_s))
        while time.time() < deadline:
            snap = self.snapshot()
            if snap is not None:
                return snap
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate joint data.")


class ArmTrajectoryController:
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
        self.waist_hold = {joint: 0.0 for joint in self.waist_joints}

        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        for joint in self.arm_joints + self.waist_joints:
            self._cmd.motor_cmd[joint].mode = 1

    def seed_waist_hold(self, positions: dict[int, float]) -> None:
        for joint in self.waist_joints:
            if joint in positions:
                self.waist_hold[joint] = float(positions[joint])

    def write_pose(self, arm_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.arm_joints, arm_positions):
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(q_val)
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0

        for joint in self.waist_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self.waist_hold[joint]
            mc.dq = 0.0
            mc.kp = self.waist_kp
            mc.kd = self.waist_kd
            mc.tau = 0.0

        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


def run_record(args: argparse.Namespace) -> int:
    if not args.yes:
        input("Press Enter to call LocoClient.ShakeHand() and record the right arm trajectory...")

    state = RobotStateSubscriber(list(WAIST_IDX) + list(RIGHT_ARM_IDX))
    state.wait_for_snapshot()

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    if not hasattr(loco, "ShakeHand"):
        raise AttributeError("Current locomotion client does not support ShakeHand().")

    seconds = max(0.2, float(args.seconds))
    rate_hz = max(1.0, float(args.rate_hz))
    dt = 1.0 / rate_hz
    samples: list[dict[str, Any]] = []

    start_mono = time.monotonic()
    positions, timestamp = state.wait_for_snapshot()
    samples.append(
        {
            "t": 0.0,
            "state_timestamp_s": float(timestamp),
            "right_arm": [float(positions[joint]) for joint in RIGHT_ARM_IDX],
            "waist": [float(positions[joint]) for joint in WAIST_IDX],
        }
    )

    print("Sending ShakeHand() and recording...")
    loco.ShakeHand()
    next_print_s = 0.0
    while True:
        rel_t = time.monotonic() - start_mono
        if rel_t >= seconds:
            break
        snap = state.snapshot()
        if snap is not None:
            positions, timestamp = snap
            samples.append(
                {
                    "t": float(rel_t),
                    "state_timestamp_s": float(timestamp),
                    "right_arm": [float(positions[joint]) for joint in RIGHT_ARM_IDX],
                    "waist": [float(positions[joint]) for joint in WAIST_IDX],
                }
            )
        if rel_t >= next_print_s:
            print(f"  recording {rel_t:.1f}/{seconds:.1f}s samples={len(samples)}")
            next_print_s += 1.0
        time.sleep(dt)

    if len(samples) < 2:
        raise RuntimeError("Fewer than two samples captured.")

    split_index = split_index_for_samples(samples)
    record_name = args.name or f"shake_hand_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    record = {
        "name": record_name,
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "source_motion": "LocoClient.ShakeHand",
        "source_arm": "right",
        "joint_names": JOINT_NAMES,
        "right_arm_joints": RIGHT_ARM_IDX,
        "left_arm_joints": LEFT_ARM_IDX,
        "mirror_signs_for_left": [float(value) for value in MIRROR_SIGNS.tolist()],
        "rate_hz": rate_hz,
        "record_seconds": seconds,
        "split_index": split_index,
        "samples": samples,
    }

    path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
    index = save_record(path, record)
    print(
        f"Saved recording index {index} '{record_name}' to {path}\n"
        f"  samples={len(samples)} raise_samples={split_index + 1} drop_samples={len(samples) - split_index}"
    )
    return 0


def run_replay(args: argparse.Namespace) -> int:
    path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
    records = load_records(path)
    record_index, record = choose_record(records, name=args.name, index=args.index)
    samples = phase_samples(record, args.phase)
    if not samples:
        raise RuntimeError(f"Recording {record_index} has no samples for phase '{args.phase}'.")

    arm_joints = selected_arm_joints(args.arm)
    targets = map_right_trajectory_to_arm(samples, args.arm)
    sample_t0 = float(samples[0].get("t", 0.0))
    sample_times = [max(0.0, float(sample.get("t", 0.0)) - sample_t0) for sample in samples]

    state = RobotStateSubscriber(list(WAIST_IDX) + arm_joints)
    positions, _timestamp = state.wait_for_snapshot()
    current_command = np.asarray([positions[joint] for joint in arm_joints], dtype=np.float32)

    controller = ArmTrajectoryController(
        arm_joints,
        list(WAIST_IDX),
        kp=float(args.kp),
        kd=float(args.kd),
        waist_kp=float(args.waist_kp),
        waist_kd=float(args.waist_kd),
    )
    controller.seed_waist_hold(positions)

    stop_event = threading.Event()

    def _handle_signal(_signum: int, _frame: Any) -> None:
        stop_event.set()

    previous_sigint = signal.signal(signal.SIGINT, _handle_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, _handle_signal)

    rate_hz = max(1.0, float(args.rate_hz))
    dt = 1.0 / rate_hz
    max_increment = max(0.0001, float(args.max_increment_rad))
    started_s = time.monotonic()
    final_target = targets[-1]

    print(
        f"Replaying recording {record_index} '{record.get('name', '<unnamed>')}' "
        f"phase={args.phase} arm={args.arm} samples={len(samples)} "
        f"max_increment={max_increment:.4f} rad/tick"
    )
    try:
        while not stop_event.is_set():
            elapsed_s = time.monotonic() - started_s
            target_index = 0
            for idx, sample_time in enumerate(sample_times):
                if sample_time <= elapsed_s:
                    target_index = idx
                else:
                    break

            target = targets[target_index]
            current_command = step_towards(current_command, target, max_increment)
            controller.write_pose(current_command)

            if elapsed_s >= sample_times[-1] and np.max(np.abs(final_target - current_command)) <= max_increment:
                current_command = final_target.copy()
                controller.write_pose(current_command)
                break
            time.sleep(dt)

        if stop_event.is_set():
            print("Replay interrupted.")
            return 1

        print("Replay reached final target.")
        hold_started_s = time.monotonic()
        while not stop_event.is_set():
            controller.write_pose(final_target)
            if not args.hold_forever and time.monotonic() - hold_started_s >= max(0.0, float(args.hold_seconds)):
                break
            time.sleep(dt)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    print("Replay complete.")
    return 0


def run_list(args: argparse.Namespace) -> int:
    path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
    records = load_records(path)
    if not records:
        print(f"No recordings in {path}")
        return 0
    for idx, record in enumerate(records):
        samples = record.get("samples", [])
        split_index = int(record.get("split_index", 0))
        print(
            f"{idx}: name={record.get('name', '<unnamed>')} "
            f"recorded_at={record.get('recorded_at', '?')} "
            f"samples={len(samples) if isinstance(samples, list) else '?'} "
            f"raise={split_index + 1} drop={max(0, len(samples) - split_index) if isinstance(samples, list) else '?'}"
        )
    return 0


def main() -> int:
    args = parse_args()

    if args.run_hanged_boot:
        if hanger_boot_sequence is None:
            raise SystemExit("sdk_boot.py not available, cannot run hanger boot sequence.")
        hanger_boot_sequence(iface=str(args.iface), domain_id=int(args.domain_id))

    ChannelFactoryInitialize(int(args.domain_id), str(args.iface))

    if args.command == "record":
        return run_record(args)
    if args.command == "replay":
        return run_replay(args)
    if args.command == "list":
        return run_list(args)
    raise SystemExit(f"Unsupported command '{args.command}'.")


if __name__ == "__main__":
    raise SystemExit(main())
