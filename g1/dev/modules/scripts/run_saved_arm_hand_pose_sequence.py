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

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

import numpy as np

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, LowCmd_
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

from sdk_hand import Dex3HandController, hand_open_targets


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
ALL_ARM_JOINTS = LEFT_ARM_IDX + RIGHT_ARM_IDX
NOT_USED_IDX = 29
DEFAULT_POSE_FILE = os.path.join(SCRIPT_DIR, "saved_arm_hand_poses.json")
MAX_JOINT_INCREMENT_RAD = 0.005


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Execute saved arm and hand poses in sequence with a pause between poses."
    )
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument("--file", default=DEFAULT_POSE_FILE, help="Saved arm and hand pose JSON file.")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Command publish rate.")
    parser.add_argument("--speed-rad-s", type=float, default=0.4, help="Maximum arm joint transition speed.")
    parser.add_argument("--hand-speed-rad-s", type=float, default=0.6, help="Maximum finger joint transition speed.")
    parser.add_argument("--kp", type=float, default=30.0, help="Arm joint proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Arm joint feed-forward torque.")
    parser.add_argument("--hand-kp", type=float, default=0.5, help="Dex3 finger proportional gain.")
    parser.add_argument("--hand-kd", type=float, default=0.1, help="Dex3 finger derivative gain.")
    parser.add_argument("--hand-tau", type=float, default=0.0, help="Dex3 finger feed-forward torque.")
    parser.add_argument("--sleep-between", type=float, default=1.0, help="Seconds to hold each pose before moving to the next.")
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


def load_poses(path: Path) -> list[dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"Pose file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Pose file is not valid JSON: {path}: {exc}") from exc
    poses = payload.get("poses")
    if not isinstance(poses, list):
        raise SystemExit("Pose file does not contain a 'poses' list.")
    if not poses:
        raise SystemExit("Pose file does not contain any poses.")
    return [pose for pose in poses if isinstance(pose, dict)]


class ArmStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: np.ndarray | None = None

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
            self._positions = np.asarray(positions, dtype=np.float32)

    def wait_for_snapshot(self, timeout_s: float = 3.0) -> np.ndarray:
        deadline = time.time() + max(0.1, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                if self._positions is not None:
                    return self._positions.copy()
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for arm state.")


class HandStateSubscriber:
    TOPIC_BY_SIDE = {
        "left": "rt/dex3/left/state",
        "right": "rt/dex3/right/state",
    }

    def __init__(self, hand: str) -> None:
        self.hand = str(hand)
        self._lock = threading.Lock()
        self._positions = np.asarray(hand_open_targets(self.hand), dtype=np.float32)
        self._sub = ChannelSubscriber(self.TOPIC_BY_SIDE[self.hand], HandState_)
        self._sub.Init(self._callback, 50)

    def _callback(self, msg: Any) -> None:
        try:
            positions = [float(msg.motor_state[idx].q) for idx in range(7)]
        except Exception:
            return
        with self._lock:
            self._positions = np.asarray(positions, dtype=np.float32)

    def wait_for_snapshot(self, timeout_s: float = 3.0) -> np.ndarray:
        deadline = time.time() + max(0.1, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                if self._positions is not None:
                    return self._positions.copy()
            time.sleep(0.02)
        raise TimeoutError(f"Timed out waiting for {self.hand} hand state.")


class ArmPoseController:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        for joint in self.joints:
            self._cmd.motor_cmd[joint].mode = 1

    def write_targets_once(self, targets_by_joint: dict[int, float], *, kp: float, kd: float, tau: float) -> None:
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(targets_by_joint[joint])
            mc.dq = 0.0
            mc.kp = float(kp)
            mc.kd = float(kd)
            mc.tau = float(tau)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


def parse_pose(pose: dict[str, Any]) -> tuple[dict[int, float], dict[str, np.ndarray]]:
    raw_arm = pose.get("arm_joints")
    raw_hands = pose.get("hands")
    if not isinstance(raw_arm, dict) or not isinstance(raw_hands, dict):
        raise ValueError("Pose is missing arm_joints or hands.")

    arm_targets: dict[int, float] = {}
    for joint in ALL_ARM_JOINTS:
        key = str(joint)
        if key not in raw_arm:
            raise ValueError(f"Pose is missing arm joint {joint}.")
        arm_targets[joint] = float(raw_arm[key])

    hand_targets: dict[str, np.ndarray] = {}
    for hand in ("left", "right"):
        values = raw_hands.get(hand)
        if not isinstance(values, list) or len(values) != 7:
            raise ValueError(f"Pose is missing 7 values for {hand} hand.")
        hand_targets[hand] = np.asarray([float(v) for v in values], dtype=np.float32)
    return arm_targets, hand_targets


def step_towards(current: np.ndarray, target: np.ndarray, max_delta: float) -> np.ndarray:
    delta = target - current
    if max_delta <= 0.0:
        return target.copy()
    clipped = np.clip(delta, -max_delta, max_delta)
    next_value = current + clipped
    close_mask = np.abs(delta) <= max_delta
    next_value[close_mask] = target[close_mask]
    return next_value.astype(np.float32)


def main() -> int:
    args = parse_args()
    pose_path = Path(os.path.abspath(os.path.expanduser(args.file)))
    poses = load_poses(pose_path)

    ChannelFactoryInitialize(int(args.domain_id), str(args.iface))
    arm_state = ArmStateSubscriber(ALL_ARM_JOINTS)
    hand_state = {hand: HandStateSubscriber(hand) for hand in ("left", "right")}
    arm_controller = ArmPoseController(ALL_ARM_JOINTS)
    hand_controller = {
        hand: Dex3HandController(hand=hand, iface=args.iface, domain_id=args.domain_id)
        for hand in ("left", "right")
    }

    current_arm = arm_state.wait_for_snapshot()
    current_hands = {hand: hand_state[hand].wait_for_snapshot() for hand in ("left", "right")}

    rate_hz = max(1.0, float(args.rate_hz))
    dt = 1.0 / rate_hz
    arm_step = min(MAX_JOINT_INCREMENT_RAD, max(0.01, float(args.speed_rad_s)) * dt)
    hand_step = min(MAX_JOINT_INCREMENT_RAD, max(0.01, float(args.hand_speed_rad_s)) * dt)
    sleep_between = max(0.0, float(args.sleep_between))

    print(
        f"Using per-update joint increment guard of {MAX_JOINT_INCREMENT_RAD:.3f} rad "
        f"(arm_step={arm_step:.3f}, hand_step={hand_step:.3f})."
    )

    stop_event = threading.Event()

    def _handle_signal(_signum: int, _frame: Any) -> None:
        stop_event.set()

    previous_sigint = signal.signal(signal.SIGINT, _handle_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, _handle_signal)
    try:
        for idx, pose in enumerate(poses):
            if stop_event.is_set():
                break

            name = str(pose.get("name", f"pose_{idx}"))
            arm_target_map, hand_targets = parse_pose(pose)
            target_arm = np.asarray([arm_target_map[joint] for joint in ALL_ARM_JOINTS], dtype=np.float32)
            print(f"[{idx + 1}/{len(poses)}] Moving to pose '{name}'")

            while not stop_event.is_set():
                next_arm = step_towards(current_arm, target_arm, arm_step)
                next_hands = {
                    hand: step_towards(current_hands[hand], hand_targets[hand], hand_step)
                    for hand in ("left", "right")
                }

                arm_controller.write_targets_once(
                    {joint: float(value) for joint, value in zip(ALL_ARM_JOINTS, next_arm)},
                    kp=float(args.kp),
                    kd=float(args.kd),
                    tau=float(args.tau),
                )
                for hand in ("left", "right"):
                    hand_controller[hand].write_targets_once(
                        next_hands[hand].tolist(),
                        kp=float(args.hand_kp),
                        kd=float(args.hand_kd),
                        tau=float(args.hand_tau),
                        timeout=0,
                        first_write_timeout_s=None,
                    )

                current_arm = next_arm
                current_hands = next_hands
                arm_done = bool(np.allclose(current_arm, target_arm, atol=max(arm_step, 1e-4)))
                hands_done = all(
                    bool(np.allclose(current_hands[hand], hand_targets[hand], atol=max(hand_step, 1e-4)))
                    for hand in ("left", "right")
                )
                if arm_done and hands_done:
                    break
                time.sleep(dt)

            if stop_event.is_set():
                break

            if sleep_between > 0.0 and idx < len(poses) - 1:
                print(f"Holding pose '{name}' for {sleep_between:.1f}s before the next pose.")
                deadline = time.time() + sleep_between
                while not stop_event.is_set() and time.time() < deadline:
                    arm_controller.write_targets_once(
                        {joint: float(value) for joint, value in zip(ALL_ARM_JOINTS, current_arm)},
                        kp=float(args.kp),
                        kd=float(args.kd),
                        tau=float(args.tau),
                    )
                    for hand in ("left", "right"):
                        hand_controller[hand].write_targets_once(
                            current_hands[hand].tolist(),
                            kp=float(args.hand_kp),
                            kd=float(args.hand_kd),
                            tau=float(args.hand_tau),
                            timeout=0,
                            first_write_timeout_s=None,
                        )
                    time.sleep(dt)
        if not stop_event.is_set():
            print("Finished executing all saved poses.")
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
