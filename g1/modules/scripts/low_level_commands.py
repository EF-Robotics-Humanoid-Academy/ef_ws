#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from typing import Any

try:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
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


BODY_JOINTS: list[tuple[str, int, float, float]] = [
    ("left_leg.hip_pitch", 0, -2.5307, 2.8798),
    ("left_leg.hip_roll", 1, -0.5236, 2.9671),
    ("left_leg.hip_yaw", 2, -2.7576, 2.7576),
    ("left_leg.knee", 3, -0.087267, 2.8798),
    ("left_leg.ankle_pitch", 4, -0.87267, 0.5236),
    ("left_leg.ankle_roll", 5, -0.2618, 0.2618),
    ("right_leg.hip_pitch", 6, -2.5307, 2.8798),
    ("right_leg.hip_roll", 7, -2.9671, 0.5236),
    ("right_leg.hip_yaw", 8, -2.7576, 2.7576),
    ("right_leg.knee", 9, -0.087267, 2.8798),
    ("right_leg.ankle_pitch", 10, -0.87267, 0.5236),
    ("right_leg.ankle_roll", 11, -0.2618, 0.2618),
    ("waist.yaw", 12, -2.618, 2.618),
    ("waist.roll", 13, -0.52, 0.52),
    ("waist.pitch", 14, -0.52, 0.52),
    ("left_arm.shoulder_pitch", 15, -3.0892, 2.6704),
    ("left_arm.shoulder_roll", 16, -1.5882, 2.2515),
    ("left_arm.shoulder_yaw", 17, -2.618, 2.618),
    ("left_arm.elbow", 18, -1.0472, 2.0944),
    ("left_arm.wrist_roll", 19, -1.9722, 1.9722),
    ("left_arm.wrist_pitch", 20, -1.6144, 1.6144),
    ("left_arm.wrist_yaw", 21, -1.6144, 1.6144),
    ("right_arm.shoulder_pitch", 22, -3.0892, 2.6704),
    ("right_arm.shoulder_roll", 23, -2.2515, 1.5882),
    ("right_arm.shoulder_yaw", 24, -2.618, 2.618),
    ("right_arm.elbow", 25, -1.0472, 2.0944),
    ("right_arm.wrist_roll", 26, -1.9722, 1.9722),
    ("right_arm.wrist_pitch", 27, -1.6144, 1.6144),
    ("right_arm.wrist_yaw", 28, -1.6144, 1.6144),
]

HAND_JOINT_NAMES = [
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "middle_0",
    "middle_1",
    "index_0",
    "index_1",
]

HAND_MIN_LIMITS = {
    "left": [-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75],
    "right": [-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0],
}

HAND_MAX_LIMITS = {
    "left": [1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0],
    "right": [1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75],
}

HAND_CMD_TOPIC = {
    "left": "rt/dex3/left/cmd",
    "right": "rt/dex3/right/cmd",
}

HAND_STATE_TOPIC = {
    "left": "rt/dex3/left/state",
    "right": "rt/dex3/right/state",
}

HAND_CHOICES = ("both", "left", "right", "none")

BODY_KP = [60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40] + [40] * 14
BODY_KD = [1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1] + [1] * 14


def clamp(value: float, lo: float, hi: float) -> float:
    return max(float(lo), min(float(hi), float(value)))


def lerp(start: float, stop: float, ratio: float) -> float:
    return float(start) + (float(stop) - float(start)) * float(ratio)


def smoothstep(ratio: float) -> float:
    x = clamp(float(ratio), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def pack_ris_mode(motor_id: int, status: int = 1, timeout: int = 0) -> int:
    return (
        (int(motor_id) & 0x0F)
        | ((int(status) & 0x07) << 4)
        | ((int(timeout) & 0x01) << 7)
    )


@dataclass(frozen=True)
class BodySnapshot:
    positions: list[float]
    mode_machine: int


class LatestBodyState:
    def __init__(self) -> None:
        self.latest: BodySnapshot | None = None
        self.ts = 0.0
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._cb, 50)

    def _cb(self, msg: LowState_) -> None:
        try:
            positions = [float(msg.motor_state[idx].q) for _, idx, _, _ in BODY_JOINTS]
            mode_machine = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        self.latest = BodySnapshot(positions=positions, mode_machine=mode_machine)
        self.ts = time.time()

    def wait(self, timeout_s: float) -> BodySnapshot:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            if self.latest is not None:
                return self.latest
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


class LatestHandState:
    def __init__(self, hand: str) -> None:
        self.hand = hand
        self.latest: list[float] | None = None
        self.ts = 0.0
        self.sub = ChannelSubscriber(HAND_STATE_TOPIC[hand], HandState_)
        self.sub.Init(self._cb, 20)

    def _cb(self, msg: HandState_) -> None:
        try:
            self.latest = [float(msg.motor_state[idx].q) for idx in range(7)]
            self.ts = time.time()
        except Exception:
            return

    def wait(self, timeout_s: float) -> list[float]:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            if self.latest is not None:
                return list(self.latest)
            time.sleep(0.02)
        raise TimeoutError(f"Timed out waiting for {self.hand} Dex3 state.")


class LowLevelJointExample:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.dt = 1.0 / self.rate_hz
        self.crc = CRC()
        self.motion: MotionSwitcherClient | None = None
        self.body_state: LatestBodyState | None = None
        self.hand_states: dict[str, LatestHandState] = {}
        self.body_pub: ChannelPublisher | None = None
        self.hand_pubs: dict[str, ChannelPublisher] = {}
        self.body_cmd = unitree_hg_msg_dds__LowCmd_()
        self.hand_cmds = {
            "left": unitree_hg_msg_dds__HandCmd_(),
            "right": unitree_hg_msg_dds__HandCmd_(),
        }

    def setup_dds(self) -> None:
        ChannelFactoryInitialize(int(self.args.domain_id), str(self.args.iface))
        self.motion = MotionSwitcherClient()
        self.motion.SetTimeout(float(self.args.timeout))
        self.motion.Init()

        self.body_state = LatestBodyState()
        hands = self.selected_hands()
        self.hand_states = {hand: LatestHandState(hand) for hand in hands}
        self.body_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.body_pub.Init()
        self.hand_pubs = {
            hand: ChannelPublisher(topic, HandCmd_)
            for hand, topic in HAND_CMD_TOPIC.items()
            if hand in hands
        }
        for pub in self.hand_pubs.values():
            pub.Init()

    @staticmethod
    def current_motion_name(data: Any) -> str:
        if not isinstance(data, dict):
            return ""
        name = data.get("name")
        return "" if name is None else str(name)

    def selected_hands(self) -> list[str]:
        value = str(self.args.hands)
        if value == "none":
            return []
        if value == "both":
            return ["left", "right"]
        return [value]

    def enter_dev_mode(self) -> None:
        assert self.motion is not None
        code, data = self.motion.CheckMode()
        print(f"motion before: code={int(code)} data={data}")
        while int(code) == 0 and self.current_motion_name(data):
            release_code, _ = self.motion.ReleaseMode()
            print(f"ReleaseMode(): code={int(release_code)}")
            time.sleep(0.5)
            code, data = self.motion.CheckMode()
            print(f"motion check: code={int(code)} data={data}")
        if int(code) != 0:
            raise RuntimeError(f"MotionSwitcher CheckMode failed with code={int(code)} data={data}")

    def write_body(self, targets: list[float], mode_machine: int, *, kp_scale: float = 1.0) -> None:
        assert self.body_pub is not None
        self.body_cmd.mode_pr = 0
        self.body_cmd.mode_machine = int(mode_machine)
        for local_idx, (_, motor_idx, _, _) in enumerate(BODY_JOINTS):
            cmd = self.body_cmd.motor_cmd[motor_idx]
            cmd.mode = 1
            cmd.tau = 0.0
            cmd.q = float(targets[local_idx])
            cmd.dq = 0.0
            cmd.kp = float(BODY_KP[local_idx]) * float(kp_scale)
            cmd.kd = float(BODY_KD[local_idx])
        self.body_cmd.crc = self.crc.Crc(self.body_cmd)
        self.body_pub.Write(self.body_cmd)

    def write_hand(self, hand: str, targets: list[float], *, kp: float | None = None) -> None:
        assert hand in self.hand_pubs
        msg = self.hand_cmds[hand]
        for idx, value in enumerate(targets):
            cmd = msg.motor_cmd[idx]
            cmd.mode = pack_ris_mode(idx)
            cmd.tau = 0.0
            cmd.q = float(value)
            cmd.dq = 0.0
            cmd.kp = float(self.args.hand_kp if kp is None else kp)
            cmd.kd = float(self.args.hand_kd)
        self.hand_pubs[hand].Write(msg)

    def hold_all(self, body_targets: list[float], hand_targets: dict[str, list[float]], mode_machine: int, seconds: float) -> None:
        steps = max(1, int(max(0.0, float(seconds)) * self.rate_hz))
        for _ in range(steps):
            self.write_body(body_targets, mode_machine)
            for hand, targets in hand_targets.items():
                self.write_hand(hand, targets)
            time.sleep(self.dt)

    def exercise_body_joint(
        self,
        name: str,
        local_idx: int,
        motor_idx: int,
        lo: float,
        hi: float,
        body_targets: list[float],
        hand_targets: dict[str, list[float]],
        mode_machine: int,
    ) -> None:
        start = float(body_targets[local_idx])
        target = clamp(start + float(self.args.offset), lo, hi)
        if math.isclose(start, target, abs_tol=1e-6):
            print(f"skip body {motor_idx:02d} {name}: start {start:.3f} already at limit")
            return
        print(f"body {motor_idx:02d} {name}: {start:+.3f} -> {target:+.3f}")
        steps = max(1, int(float(self.args.duration_per_joint) * self.rate_hz))
        for step in range(steps + 1):
            ratio = smoothstep(step / steps)
            body_targets[local_idx] = lerp(start, target, ratio)
            self.write_body(body_targets, mode_machine)
            for hand, targets in hand_targets.items():
                self.write_hand(hand, targets)
            time.sleep(self.dt)
        if self.args.return_each_joint:
            for step in range(steps + 1):
                ratio = smoothstep(step / steps)
                body_targets[local_idx] = lerp(target, start, ratio)
                self.write_body(body_targets, mode_machine)
                for hand, targets in hand_targets.items():
                    self.write_hand(hand, targets)
                time.sleep(self.dt)

    def exercise_hand_joint(
        self,
        hand: str,
        joint_idx: int,
        hand_targets: dict[str, list[float]],
        body_targets: list[float],
        mode_machine: int,
    ) -> None:
        lo = HAND_MIN_LIMITS[hand][joint_idx]
        hi = HAND_MAX_LIMITS[hand][joint_idx]
        start = float(hand_targets[hand][joint_idx])
        target = clamp(start + float(self.args.offset), lo, hi)
        if math.isclose(start, target, abs_tol=1e-6):
            print(
                f"skip {hand}_hand {joint_idx} {HAND_JOINT_NAMES[joint_idx]}: start {start:.3f} already at limit")
            return
        print(
            f"{hand}_hand {joint_idx} {HAND_JOINT_NAMES[joint_idx]}: {start:+.3f} -> {target:+.3f}")
        steps = max(1, int(float(self.args.duration_per_joint) * self.rate_hz))
        for step in range(steps + 1):
            ratio = smoothstep(step / steps)
            hand_targets[hand][joint_idx] = lerp(start, target, ratio)
            self.write_body(body_targets, mode_machine)
            for side, targets in hand_targets.items():
                self.write_hand(side, targets)
            time.sleep(self.dt)
        if self.args.return_each_joint:
            for step in range(steps + 1):
                ratio = smoothstep(step / steps)
                hand_targets[hand][joint_idx] = lerp(target, start, ratio)
                self.write_body(body_targets, mode_machine)
                for side, targets in hand_targets.items():
                    self.write_hand(side, targets)
                time.sleep(self.dt)

    def run(self) -> int:
        print("This example uses true low-level body control on rt/lowcmd.")
        print("The G1 body has 29 valid motors: 6 per leg, 3 waist, 7 per arm.")
        print("Together with two 7-DOF Dex3 hands, this script exercises 43 joints total.")
        if not self.args.yes:
            print("Dry run only. Re-run with --yes to release motion switcher and send commands.")
            for name, idx, _, _ in BODY_JOINTS:
                print(f"body {idx:02d} {name}")
            for hand in self.selected_hands():
                for idx, name in enumerate(HAND_JOINT_NAMES):
                    print(f"{hand}_hand {idx} {name}")
            return 0

        self.setup_dds()
        self.enter_dev_mode()
        assert self.body_state is not None
        body_snapshot = self.body_state.wait(self.args.timeout)
        body_targets = list(body_snapshot.positions)
        mode_machine = int(body_snapshot.mode_machine)
        hand_targets: dict[str, list[float]] = {}
        for hand in self.selected_hands():
            try:
                hand_targets[hand] = self.hand_states[hand].wait(self.args.timeout)
            except TimeoutError as exc:
                if self.args.require_hands:
                    raise
                print(f"warning: {exc}; skipping {hand} hand")
        for hand in list(hand_targets):
            hand_targets[hand] = [
                clamp(value, lo, hi)
                for value, lo, hi in zip(hand_targets[hand], HAND_MIN_LIMITS[hand], HAND_MAX_LIMITS[hand])
            ]

        print(
            f"mode_machine={mode_machine}; holding current pose for {self.args.initial_hold_s:.1f}s")
        self.hold_all(body_targets, hand_targets, mode_machine, self.args.initial_hold_s)

        try:
            for local_idx, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
                self.exercise_body_joint(name, local_idx, motor_idx, lo, hi,
                                         body_targets, hand_targets, mode_machine)
            for hand in list(hand_targets):
                for joint_idx in range(7):
                    self.exercise_hand_joint(hand, joint_idx, hand_targets,
                                             body_targets, mode_machine)
            print(f"done; holding final pose for {self.args.final_hold_s:.1f}s")
            self.hold_all(body_targets, hand_targets, mode_machine, self.args.final_hold_s)
        finally:
            if self.args.zero_gains_on_exit:
                print("zeroing gains on exit")
                self.write_body(body_targets, mode_machine, kp_scale=0.0)
                for hand, targets in hand_targets.items():
                    self.write_hand(hand, targets, kp=0.0)
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Example: release G1 motion switcher and slowly move all 43 body+Dex3 joints by an offset."
    )
    parser.add_argument("--iface", default="eth0", help="DDS network interface.")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain ID.")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="Seconds to wait for SDK RPC/state.")
    parser.add_argument("--offset", type=float, default=0.4, help="Joint offset in radians.")
    parser.add_argument("--duration-per-joint", type=float,
                        default=8.0, help="Ramp duration per joint.")
    parser.add_argument("--initial-hold-s", type=float, default=2.0,
                        help="Hold current pose before moving.")
    parser.add_argument("--final-hold-s", type=float, default=2.0,
                        help="Hold final pose before exit.")
    parser.add_argument("--rate-hz", type=float, default=250.0, help="Low-level publish rate.")
    parser.add_argument("--hand-kp", type=float, default=0.5, help="Dex3 hand kp.")
    parser.add_argument("--hand-kd", type=float, default=0.1, help="Dex3 hand kd.")
    parser.add_argument("--hands", choices=HAND_CHOICES, default="both",
                        help="Which Dex3 hand(s) to include.")
    parser.add_argument("--require-hands", action="store_true",
                        help="Fail if selected Dex3 hand state is unavailable.")
    parser.add_argument("--return-each-joint", action="store_true",
                        help="Return each joint to its start before moving on.")
    parser.add_argument("--zero-gains-on-exit", action="store_true",
                        help="Send one zero-gain command on exit.")
    parser.add_argument("--yes", action="store_true",
                        help="Actually release motion switcher and send low-level commands.")
    return parser.parse_args()


def main() -> int:
    return LowLevelJointExample(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
