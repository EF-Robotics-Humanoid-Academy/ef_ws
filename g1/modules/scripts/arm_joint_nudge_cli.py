#!/usr/bin/env python3
from __future__ import annotations
from dds_env import ensure_cyclonedds_environment

import argparse
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)


ensure_cyclonedds_environment()

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


MAX_NUDGE_RAD = 0.1
NOT_USED_IDX = 29


@dataclass(frozen=True)
class ArmJoint:
    label: str
    motor_index: int
    lo: float
    hi: float


ARM_JOINTS: list[ArmJoint] = [
    ArmJoint("left_arm.shoulder_pitch", 15, -3.0892, 2.6704),
    ArmJoint("left_arm.shoulder_roll", 16, -1.5882, 2.2515),
    ArmJoint("left_arm.shoulder_yaw", 17, -2.618, 2.618),
    ArmJoint("left_arm.elbow", 18, -1.0472, 2.0944),
    ArmJoint("left_arm.wrist_roll", 19, -1.9722, 1.9722),
    ArmJoint("left_arm.wrist_pitch", 20, -1.6144, 1.6144),
    ArmJoint("left_arm.wrist_yaw", 21, -1.6144, 1.6144),
    ArmJoint("right_arm.shoulder_pitch", 22, -3.0892, 2.6704),
    ArmJoint("right_arm.shoulder_roll", 23, -2.2515, 1.5882),
    ArmJoint("right_arm.shoulder_yaw", 24, -2.618, 2.618),
    ArmJoint("right_arm.elbow", 25, -1.0472, 2.0944),
    ArmJoint("right_arm.wrist_roll", 26, -1.9722, 1.9722),
    ArmJoint("right_arm.wrist_pitch", 27, -1.6144, 1.6144),
    ArmJoint("right_arm.wrist_yaw", 28, -1.6144, 1.6144),
]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


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


class ArmStateSubscriber:
    def __init__(self, joints: list[ArmJoint]) -> None:
        self.joints = list(joints)
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
            positions = {joint.motor_index: float(
                msg.motor_state[joint.motor_index].q) for joint in self.joints}
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

    def wait(self, timeout_s: float) -> dict[int, float]:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            snapshot = self.snapshot()
            if snapshot is not None:
                positions, _timestamp = snapshot
                return positions
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate arm joint data.")


class ArmPosePublisher:
    def __init__(self, joints: list[ArmJoint]) -> None:
        self.joints = list(joints)
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1.0
        for joint in self.joints:
            self._cmd.motor_cmd[joint.motor_index].mode = 1

    def write_targets_once(self, targets: dict[int, float], *, kp: float, kd: float, tau: float) -> None:
        for joint in self.joints:
            motor_index = joint.motor_index
            mc = self._cmd.motor_cmd[motor_index]
            mc.mode = 1
            mc.q = float(targets[motor_index])
            mc.dq = 0.0
            mc.kp = float(kp)
            mc.kd = float(kd)
            mc.tau = float(tau)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def write_zero_gains_once(self, targets: dict[int, float]) -> None:
        for joint in self.joints:
            motor_index = joint.motor_index
            mc = self._cmd.motor_cmd[motor_index]
            mc.mode = 1
            mc.q = float(targets[motor_index])
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class ArmJointNudgeCli:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.dt = 1.0 / self.rate_hz
        self.speed_rad_s = max(0.01, float(args.speed_rad_s))
        self.step_rad = min(MAX_NUDGE_RAD, max(0.001, abs(float(args.step_rad))))
        self.joints = list(ARM_JOINTS)
        self.selected_index = 0
        self.targets: dict[int, float] = {}
        self.state_sub: ArmStateSubscriber | None = None
        self.publisher: ArmPosePublisher | None = None

    def setup(self) -> None:
        ChannelFactoryInitialize(int(self.args.domain_id), str(self.args.iface))
        self.state_sub = ArmStateSubscriber(self.joints)
        self.publisher = ArmPosePublisher(self.joints)
        positions = self.state_sub.wait(float(self.args.timeout))
        self.targets = {
            joint.motor_index: clamp(positions[joint.motor_index], joint.lo, joint.hi)
            for joint in self.joints
        }
        self.publisher.write_targets_once(
            self.targets,
            kp=float(self.args.kp),
            kd=float(self.args.kd),
            tau=float(self.args.tau),
        )

    def print_help(self) -> None:
        print(
            "\nCommands:\n"
            "  list | l                show arm joints and current targets\n"
            "  select <index> | s <n>  select a joint from the list\n"
            "  + [rad]                 increase selected joint, capped at +0.1 rad\n"
            "  - [rad]                 decrease selected joint, capped at -0.1 rad\n"
            "  set <rad>               set selected joint target, ramped safely\n"
            "  step <rad>              set default +/- step, capped at 0.1 rad\n"
            "  sync                    replace targets with latest measured state\n"
            "  zero                    send one zero-gain command\n"
            "  help | h                show this help\n"
            "  quit | q                exit\n"
        )

    def print_joint_list(self) -> None:
        assert self.state_sub is not None
        snapshot = self.state_sub.snapshot()
        measured = snapshot[0] if snapshot is not None else {}
        print("")
        for idx, joint in enumerate(self.joints):
            prefix = "*" if idx == self.selected_index else " "
            motor_index = joint.motor_index
            current = measured.get(motor_index)
            current_text = "n/a" if current is None else f"{current:+.3f}"
            target = self.targets.get(motor_index, 0.0)
            print(
                f"{prefix} {idx:2d} motor {motor_index:02d} {joint.label:28s} "
                f"current={current_text:>7s} target={target:+.3f} limits=[{joint.lo:+.3f}, {joint.hi:+.3f}]"
            )
        print("")

    def selected_joint(self) -> ArmJoint:
        return self.joints[self.selected_index]

    def select_joint(self, raw: str) -> None:
        try:
            index = int(raw)
        except ValueError:
            print(f"Invalid joint index: {raw!r}")
            return
        if not 0 <= index < len(self.joints):
            print(f"Joint index must be between 0 and {len(self.joints) - 1}.")
            return
        self.selected_index = index
        joint = self.selected_joint()
        print(f"Selected {index}: motor {joint.motor_index} {joint.label}")

    def ramp_selected_to(self, target: float) -> None:
        assert self.publisher is not None
        joint = self.selected_joint()
        motor_index = joint.motor_index
        start = float(self.targets[motor_index])
        target = clamp(float(target), joint.lo, joint.hi)
        delta = target - start
        if abs(delta) <= 1e-6:
            print(f"{joint.label} already at {start:+.4f} rad")
            return

        max_step_per_tick = self.speed_rad_s * self.dt
        steps = max(1, int(abs(delta) / max(1e-6, max_step_per_tick) + 0.999))
        print(
            f"{joint.label} motor {motor_index}: {start:+.4f} -> {target:+.4f} rad "
            f"({steps} publish steps)"
        )
        for step_idx in range(1, steps + 1):
            alpha = float(step_idx) / float(steps)
            self.targets[motor_index] = start + delta * alpha
            self.publisher.write_targets_once(
                self.targets,
                kp=float(self.args.kp),
                kd=float(self.args.kd),
                tau=float(self.args.tau),
            )
            time.sleep(self.dt)
        self.targets[motor_index] = target

    def nudge_selected(self, direction: float, amount: float | None) -> None:
        requested = self.step_rad if amount is None else abs(float(amount))
        increment = min(MAX_NUDGE_RAD, requested) * (1.0 if direction >= 0.0 else -1.0)
        joint = self.selected_joint()
        current_target = float(self.targets[joint.motor_index])
        self.ramp_selected_to(current_target + increment)

    def sync_targets_to_state(self) -> None:
        assert self.state_sub is not None
        positions = self.state_sub.wait(float(self.args.timeout))
        for joint in self.joints:
            self.targets[joint.motor_index] = clamp(
                positions[joint.motor_index], joint.lo, joint.hi)
        print("Synced targets to latest rt/lowstate arm positions.")

    def run(self) -> int:
        self.setup()
        print("Arm joint nudge CLI publishing on rt/arm_sdk.")
        print(
            f"Default step is {self.step_rad:.3f} rad; every +/- command is capped at {MAX_NUDGE_RAD:.3f} rad.")
        self.print_joint_list()
        self.print_help()

        try:
            while True:
                joint = self.selected_joint()
                prompt = f"{self.selected_index}:{joint.label} target={self.targets[joint.motor_index]:+.3f}> "
                try:
                    line = input(prompt).strip()
                except EOFError:
                    print("")
                    break
                except KeyboardInterrupt:
                    print("")
                    break
                if not line:
                    continue

                parts = line.split()
                cmd = parts[0].lower()
                try:
                    if cmd in ("q", "quit", "exit"):
                        break
                    if cmd in ("h", "help", "?"):
                        self.print_help()
                    elif cmd in ("l", "list"):
                        self.print_joint_list()
                    elif cmd in ("s", "select"):
                        if len(parts) < 2:
                            print("Usage: select <index>")
                        else:
                            self.select_joint(parts[1])
                    elif cmd == "+":
                        self.nudge_selected(1.0, float(parts[1]) if len(parts) >= 2 else None)
                    elif cmd == "-":
                        self.nudge_selected(-1.0, float(parts[1]) if len(parts) >= 2 else None)
                    elif cmd == "set":
                        if len(parts) < 2:
                            print("Usage: set <rad>")
                        else:
                            self.ramp_selected_to(float(parts[1]))
                    elif cmd == "step":
                        if len(parts) < 2:
                            print(f"Current step: {self.step_rad:.3f} rad")
                        else:
                            self.step_rad = min(MAX_NUDGE_RAD, max(0.001, abs(float(parts[1]))))
                            print(f"Default step set to {self.step_rad:.3f} rad")
                    elif cmd == "sync":
                        self.sync_targets_to_state()
                    elif cmd == "zero":
                        assert self.publisher is not None
                        self.publisher.write_zero_gains_once(self.targets)
                        print("Sent one zero-gain command on rt/arm_sdk.")
                    else:
                        print(f"Unknown command: {cmd!r}. Type 'help' for commands.")
                except ValueError as exc:
                    print(f"Invalid numeric value: {exc}")
        finally:
            if self.args.zero_gains_on_exit and self.publisher is not None and self.targets:
                self.publisher.write_zero_gains_once(self.targets)
                print("Sent one zero-gain command on exit.")
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactive CLI to select a G1 arm joint and smoothly increase/decrease its pose."
    )
    parser.add_argument("--iface", default="eth0", help="DDS network interface.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain ID.")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="Seconds to wait for rt/lowstate.")
    parser.add_argument("--rate-hz", type=float, default=50.0,
                        help="rt/arm_sdk publish rate while ramping.")
    parser.add_argument("--speed-rad-s", type=float, default=0.25,
                        help="Maximum ramp speed in rad/s.")
    parser.add_argument("--step-rad", type=float, default=0.05,
                        help="Default +/- nudge amount, capped at 0.1 rad.")
    parser.add_argument("--kp", type=float, default=30.0, help="Arm joint proportional gain.")
    parser.add_argument("--kd", type=float, default=1.5, help="Arm joint derivative gain.")
    parser.add_argument("--tau", type=float, default=0.0, help="Feed-forward torque.")
    parser.add_argument("--zero-gains-on-exit", action="store_true",
                        help="Send one zero-gain command before exiting.")
    return parser.parse_args()


def main() -> int:
    return ArmJointNudgeCli(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
