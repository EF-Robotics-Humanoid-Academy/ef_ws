#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
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

from sdk_boot import hanger_boot_sequence


LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
WAIST_IDX = [12, 13, 14]
NOT_USED_IDX = 29
FORWARD_ELBOW_TARGET_RAD = 0.03
STARTUP_HOLD_SECONDS = 10.0
DEFAULT_RAMP_DURATION_SECONDS = 12.0
DEFAULT_COMMAND_RATE_HZ = 50.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extend one arm forward into a grip-ready pose and hold it there."
    )
    hand_group = parser.add_mutually_exclusive_group()
    hand_group.add_argument(
        "--hand",
        choices=("left", "right"),
        default=None,
        help="Which arm/hand to extend. Defaults to right.",
    )
    hand_group.add_argument("--left", "--left-hand", action="store_const", const="left", dest="hand", help="Extend the left hand.")
    hand_group.add_argument("--right", "--right-hand", action="store_const", const="right", dest="hand", help="Extend the right hand.")
    parser.add_argument("--iface", default="eth0", help="Network interface for DDS traffic.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id.")
    parser.add_argument(
        "--run-hanged-boot",
        action="store_true",
        help="Run the hanger boot sequence before arm control. Disabled by default.",
    )
    parser.add_argument(
        "--ramp-duration",
        type=float,
        default=DEFAULT_RAMP_DURATION_SECONDS,
        help="Seconds used to gradually move from the current arm pose to the target pose.",
    )
    args = parser.parse_args()
    if args.hand is None:
        args.hand = "right"
    return args


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


def selected_joint_indices(hand: str) -> list[int]:
    side = str(hand).strip().lower()
    if side == "left":
        return list(LEFT_ARM_IDX)
    if side == "right":
        return list(RIGHT_ARM_IDX)
    raise ValueError(f"Unsupported hand '{hand}'.")


class ArmStateSubscriber:
    def __init__(self, joints: list[int]) -> None:
        self.joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._positions: list[float] | None = None

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

    def wait_for_snapshot(self, timeout_s: float = 3.0) -> np.ndarray:
        deadline = time.time() + max(0.1, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                if self._positions is not None:
                    return np.asarray(self._positions, dtype=np.float32)
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for arm joint state.")


class ArmPoseController:
    def __init__(
        self,
        joints: list[int],
        kp: float = 30.0,
        kd: float = 1.5,
        hold_joints: list[int] | None = None,
    ) -> None:
        self.joints = [int(j) for j in joints]
        self.hold_joints = [int(j) for j in (hold_joints or [])]
        self.kp = float(kp)
        self.kd = float(kd)
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        managed_joints = set(self.joints + self.hold_joints)
        self._last_pose = {joint: 0.0 for joint in managed_joints}
        for joint in managed_joints:
            self._cmd.motor_cmd[joint].mode = 1

    def seed_pose(self, joint_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.joints + self.hold_joints, joint_positions):
            self._last_pose[joint] = float(q_val)

    def write_pose(self, joint_positions: np.ndarray) -> None:
        for joint, q_val in zip(self.joints, joint_positions):
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = float(q_val)
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
            self._last_pose[joint] = float(q_val)
        for joint in self.hold_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._last_pose[joint]
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_hold_pose(self) -> None:
        for joint in self.joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._last_pose[joint]
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
        for joint in self.hold_joints:
            mc = self._cmd.motor_cmd[joint]
            mc.mode = 1
            mc.q = self._last_pose[joint]
            mc.dq = 0.0
            mc.kp = self.kp
            mc.kd = self.kd
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def hold_countdown(
        self,
        duration_s: float,
        stop_event: threading.Event,
        rate_hz: float = 50.0,
    ) -> None:
        dt = 1.0 / max(1.0, float(rate_hz))
        deadline = time.time() + max(0.0, float(duration_s))
        last_remaining: int | None = None
        while not stop_event.is_set():
            remaining_s = deadline - time.time()
            if remaining_s <= 0.0:
                break

            remaining_int = int(np.ceil(remaining_s))
            if remaining_int != last_remaining:
                print(f"Hold countdown: {remaining_int}")
                last_remaining = remaining_int

            self.write_hold_pose()
            time.sleep(dt)

    def ramp_to_pose(self, target_positions: np.ndarray, duration_s: float = 1.2, rate_hz: float = 50.0) -> None:
        target = np.asarray(target_positions, dtype=np.float32)
        start = np.asarray([self._last_pose[joint] for joint in self.joints], dtype=np.float32)
        steps = max(1, int(max(0.1, float(duration_s)) * max(1.0, float(rate_hz))))
        dt = 1.0 / max(1.0, float(rate_hz))
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            pose = start + (target - start) * alpha
            self.write_pose(pose)
            time.sleep(dt)

    def hold_pose(self, joint_positions: np.ndarray, stop_event: threading.Event, rate_hz: float = 50.0) -> None:
        dt = 1.0 / max(1.0, float(rate_hz))
        while not stop_event.is_set():
            self.write_pose(joint_positions)
            time.sleep(dt)


def main() -> int:
    args = parse_args()
    arm_joints = selected_joint_indices(args.hand)
    hold_joints = list(WAIST_IDX)
    state_joints = arm_joints + hold_joints
    elbow_offset = 3  # shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_pitch, wrist_roll, wrist_yaw

    if args.run_hanged_boot:
        print("Running hanger boot sequence before arm control.")
        hanger_boot_sequence(iface=args.iface, domain_id=args.domain_id)
    else:
        print("Skipping hanger boot sequence.")
    ChannelFactoryInitialize(int(args.domain_id), args.iface)

    state_sub = ArmStateSubscriber(state_joints)
    arm_ctrl = ArmPoseController(
        arm_joints,
        hold_joints=hold_joints,
    )
    current_pose = state_sub.wait_for_snapshot()
    arm_ctrl.seed_pose(current_pose)

    stop_event = threading.Event()

    def _handle_signal(signum: int, _frame: Any) -> None:
        print(f"Received signal {signum}; stopping pose hold.")
        stop_event.set()

    for signame in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, signame, None)
        if sig is not None:
            signal.signal(sig, _handle_signal)

    print(f"Holding the current arm and waist pose for {STARTUP_HOLD_SECONDS:.0f} seconds.")
    arm_ctrl.hold_countdown(
        STARTUP_HOLD_SECONDS,
        stop_event=stop_event,
        rate_hz=DEFAULT_COMMAND_RATE_HZ,
    )
    if stop_event.is_set():
        return 1

    current_pose = state_sub.wait_for_snapshot()
    arm_ctrl.seed_pose(current_pose)
    target_pose = current_pose[: len(arm_joints)].copy()
    target_pose[elbow_offset] = np.float32(FORWARD_ELBOW_TARGET_RAD)

    print(
        f"Extending {args.hand} hand forward by moving elbow joint {arm_joints[elbow_offset]} "
        f"from {float(current_pose[elbow_offset]):.3f} rad to {float(target_pose[elbow_offset]):.3f} rad."
    )
    print(f"Ramping to the target pose over {float(args.ramp_duration):.1f} seconds.")
    print("Holding the arm in that pose. Press Ctrl-C to stop publishing the hold command.")

    arm_ctrl.ramp_to_pose(
        target_pose,
        duration_s=max(1.0, float(args.ramp_duration)),
        rate_hz=DEFAULT_COMMAND_RATE_HZ,
    )
    arm_ctrl.hold_pose(target_pose, stop_event=stop_event, rate_hz=DEFAULT_COMMAND_RATE_HZ)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
