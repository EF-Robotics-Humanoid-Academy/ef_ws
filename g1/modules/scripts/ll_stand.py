#!/usr/bin/env python3
"""
Experimental G1 low-level stand controller.

This publishes all 29 body joints on rt/lowcmd. It uses rt/lowstate for joint
position, joint velocity, mode_machine, and fallback IMU attitude. It also
subscribes to rt/odommodestate for attitude/velocity when available.

Run only with the robot externally supported or where a fall cannot cause
damage. Low-level control bypasses the locomotion balance controller.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_SCRIPTS_DIR = Path(__file__).resolve().parent
_MODULES_DIR = _SCRIPTS_DIR.parent
if str(_MODULES_DIR) not in sys.path:
    sys.path.insert(0, str(_MODULES_DIR))

from dds_env import default_dds_iface, ensure_channel_factory_initialized, ensure_cyclonedds_environment

ensure_cyclonedds_environment()


N_JOINTS = 29
LOWSTATE_TOPIC = "rt/lowstate"
LOWCMD_TOPIC = "rt/lowcmd"
ODOM_MODE_STATE_TOPIC = "rt/odommodestate"

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

DEFAULT_KP = [
    55, 55, 35, 85, 38, 28,
    55, 55, 35, 85, 38, 28,
    35, 30, 30,
    22, 22, 18, 20, 10, 10, 10,
    22, 22, 18, 20, 10, 10, 10,
]
DEFAULT_KD = [
    2.0, 2.0, 1.0, 3.0, 1.2, 1.0,
    2.0, 2.0, 1.0, 3.0, 1.2, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 0.8, 0.8, 0.5, 0.5, 0.5,
    1.0, 1.0, 0.8, 0.8, 0.5, 0.5, 0.5,
]

# Captured from scripts/dev_stand_snapshot.json, phase "walk" (FSM 501).
STAND_TARGET = [
    -0.30892667174339294,
    -0.021620607003569603,
    0.02119937166571617,
    0.6701621413230896,
    -0.32765069603919983,
    0.03816698119044304,
    -0.30685579776763916,
    0.02323128655552864,
    -0.01849420741200447,
    0.6826469898223877,
    -0.3303791880607605,
    -0.014181708917021751,
    0.0013391895918175578,
    0.003294900292530656,
    -0.015183044597506523,
    0.29694512486457825,
    0.1248396709561348,
    -0.025837989524006844,
    0.9889981746673584,
    0.08142082393169403,
    0.05089700594544411,
    0.0011864382540807128,
    0.2926907241344452,
    -0.12313791364431381,
    0.02828277088701725,
    0.9849954843521118,
    -0.15000654757022858,
    0.07590807974338531,
    0.019222697243094444,
]


@dataclass
class LowStateSample:
    q: list[float]
    dq: list[float]
    tau_est: list[float]
    mode_machine: int
    imu_rpy: tuple[float, float, float] | None
    timestamp: float


@dataclass
class OdomModeSample:
    rpy: tuple[float, float, float] | None
    velocity: tuple[float, float, float] | None
    timestamp: float


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(float(value), float(lo)), float(hi))


def smoothstep(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _as_vec3(value: Any) -> tuple[float, float, float] | None:
    try:
        if value is not None and len(value) >= 3:
            return float(value[0]), float(value[1]), float(value[2])
    except Exception:
        return None
    return None


def _attr_path(obj: Any, *names: str) -> Any:
    cur = obj
    for name in names:
        if cur is None or not hasattr(cur, name):
            return None
        cur = getattr(cur, name)
    return cur


def _rpy_from_msg(msg: Any) -> tuple[float, float, float] | None:
    for value in (
        _attr_path(msg, "rpy"),
        _attr_path(msg, "imu_state", "rpy"),
    ):
        rpy = _as_vec3(value)
        if rpy is not None:
            return rpy

    quat = _attr_path(msg, "imu_state", "quaternion")
    if quat is None:
        quat = _attr_path(msg, "quaternion")
    try:
        if quat is not None and len(quat) >= 4:
            return quat_to_rpy(float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0]))
    except Exception:
        pass

    ori = _attr_path(msg, "pose", "pose", "orientation")
    if ori is not None:
        try:
            return quat_to_rpy(float(ori.x), float(ori.y), float(ori.z), float(ori.w))
        except Exception:
            return None
    return None


def _velocity_from_msg(msg: Any) -> tuple[float, float, float] | None:
    for name in ("velocity", "vel", "velocity_w", "linear_velocity"):
        vec = _as_vec3(getattr(msg, name, None))
        if vec is not None:
            return vec
    lin = _attr_path(msg, "twist", "twist", "linear")
    if lin is not None:
        try:
            return float(lin.x), float(lin.y), float(lin.z)
        except Exception:
            return None
    return None


class LowStateReader:
    def __init__(self, msg_type: type) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber

        self._lock = threading.Lock()
        self._latest: LowStateSample | None = None
        self._sub = ChannelSubscriber(LOWSTATE_TOPIC, msg_type)
        self._sub.Init(self._cb, 200)

    def _cb(self, msg: Any) -> None:
        try:
            motor_state = list(getattr(msg, "motor_state"))
            q = [float(motor_state[i].q) for i in range(N_JOINTS)]
            dq = [float(getattr(motor_state[i], "dq", 0.0)) for i in range(N_JOINTS)]
            tau = [float(getattr(motor_state[i], "tau_est", 0.0)) for i in range(N_JOINTS)]
            mode_machine = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        sample = LowStateSample(
            q=q,
            dq=dq,
            tau_est=tau,
            mode_machine=mode_machine,
            imu_rpy=_rpy_from_msg(msg),
            timestamp=time.monotonic(),
        )
        with self._lock:
            self._latest = sample

    def latest(self) -> LowStateSample | None:
        with self._lock:
            return self._latest

    def wait(self, timeout: float) -> LowStateSample:
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            latest = self.latest()
            if latest is not None:
                return latest
            time.sleep(0.02)
        raise TimeoutError(f"Timed out waiting for {LOWSTATE_TOPIC}.")


class OdomModeStateReader:
    def __init__(self, msg_type: type) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber

        self._lock = threading.Lock()
        self._latest: OdomModeSample | None = None
        self._sub = ChannelSubscriber(ODOM_MODE_STATE_TOPIC, msg_type)
        self._sub.Init(self._cb, 50)

    def _cb(self, msg: Any) -> None:
        sample = OdomModeSample(
            rpy=_rpy_from_msg(msg),
            velocity=_velocity_from_msg(msg),
            timestamp=time.monotonic(),
        )
        with self._lock:
            self._latest = sample

    def latest(self) -> OdomModeSample | None:
        with self._lock:
            return self._latest


class LowCmdPublisher:
    def __init__(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
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


        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._pub = ChannelPublisher(LOWCMD_TOPIC, LowCmd_)
        self._pub.Init()

    def write(
        self,
        q: list[float],
        mode_machine: int,
        *,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        kp_scale: float = 1.0,
    ) -> None:
        kp_values = DEFAULT_KP if kp is None else kp
        kd_values = DEFAULT_KD if kd is None else kd
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine)
        for i in range(N_JOINTS):
            mc = self._cmd.motor_cmd[i]
            mc.mode = 1
            mc.q = float(q[i])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(kp_values[i]) * float(kp_scale)
            mc.kd = float(kd_values[i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


def resolve_lowstate_type() -> type:
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            module = __import__(module_path, fromlist=["LowState_"])
            return getattr(module, "LowState_")
        except Exception:
            continue
    raise RuntimeError("LowState_ type not found in unitree_sdk2py.")


def resolve_odom_mode_state_type() -> type:
    for module_path, type_name in (
        ("unitree_sdk2py.idl.unitree_go.msg.dds_", "SportModeState_"),
        ("unitree_sdk2py.idl.unitree_hg.msg.dds_", "SportModeState_"),
    ):
        try:
            module = __import__(module_path, fromlist=[type_name])
            return getattr(module, type_name)
        except Exception:
            continue
    raise RuntimeError("SportModeState_ type not found in unitree_sdk2py.")


def enter_lowcmd_dev_mode(timeout: float) -> None:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    client = MotionSwitcherClient()
    client.SetTimeout(5.0)
    client.Init()
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        code, data = client.CheckMode()
        if int(code) != 0:
            raise RuntimeError(f"MotionSwitcherClient.CheckMode() failed: code={code}")
        if not (data or {}).get("name"):
            return
        client.ReleaseMode()
        time.sleep(0.5)
    raise TimeoutError("Could not release MotionSwitcher within timeout.")


def limited_step(prev: list[float], target: list[float], max_step: float) -> list[float]:
    return [
        float(p) + clamp(float(t) - float(p), -max_step, max_step)
        for p, t in zip(prev, target)
    ]


def clamp_to_limits(q: list[float]) -> list[float]:
    out = list(q)
    for _, idx, lo, hi in BODY_JOINTS:
        out[idx] = clamp(out[idx], lo, hi)
    return out


def build_stabilized_target(
    base: list[float],
    rpy: tuple[float, float, float] | None,
    args: argparse.Namespace,
) -> list[float]:
    q = list(base)
    if rpy is None:
        return q

    roll, pitch, _ = rpy
    pitch_corr = clamp(-float(args.pitch_gain) * pitch, -float(args.max_correction), float(args.max_correction))
    roll_corr = clamp(-float(args.roll_gain) * roll, -float(args.max_correction), float(args.max_correction))

    if args.invert_pitch:
        pitch_corr = -pitch_corr
    if args.invert_roll:
        roll_corr = -roll_corr

    q[0] += 0.55 * pitch_corr
    q[6] += 0.55 * pitch_corr
    q[4] += -0.80 * pitch_corr
    q[10] += -0.80 * pitch_corr
    q[14] += 0.35 * pitch_corr

    q[1] += 0.55 * roll_corr
    q[7] += 0.55 * roll_corr
    q[5] += -0.80 * roll_corr
    q[11] += -0.80 * roll_corr
    q[13] += 0.25 * roll_corr

    return clamp_to_limits(q)


def select_attitude(low: LowStateSample, odom: OdomModeSample | None, max_odom_age: float) -> tuple[tuple[float, float, float] | None, str]:
    now = time.monotonic()
    if odom is not None and odom.rpy is not None and (now - odom.timestamp) <= max_odom_age:
        return odom.rpy, ODOM_MODE_STATE_TOPIC
    if low.imu_rpy is not None:
        return low.imu_rpy, LOWSTATE_TOPIC
    return None, "none"


def print_pose_summary(q: list[float]) -> None:
    print(f"{'idx':>3}  {'joint':<28}  {'target rad':>10}")
    for name, idx, _, _ in BODY_JOINTS:
        print(f"{idx:>3}  {name:<28}  {q[idx]:>10.5f}")


def run(args: argparse.Namespace) -> int:
    if not args.yes and not args.dry_run:
        print("Refusing to publish low-level commands without --yes.")
        print("Use --dry-run to inspect the target pose, or --yes when the robot is safely supported.")
        return 2

    ensure_channel_factory_initialized(int(args.domain), str(args.iface))

    lowstate = LowStateReader(resolve_lowstate_type())
    odom_state = OdomModeStateReader(resolve_odom_mode_state_type())

    print(f"Waiting for {LOWSTATE_TOPIC} ...")
    first = lowstate.wait(float(args.timeout))
    print(f"Initial mode_machine={first.mode_machine}")
    attitude, source = select_attitude(first, odom_state.latest(), float(args.max_odom_age))
    if attitude is None:
        print("No attitude estimate yet; continuing with pose ramp only.")
    else:
        print(
            f"Initial attitude from {source}: "
            f"roll={math.degrees(attitude[0]):+.2f} deg "
            f"pitch={math.degrees(attitude[1]):+.2f} deg"
        )

    if args.print_target:
        print_pose_summary(STAND_TARGET)

    if args.dry_run:
        print("--dry-run: DDS subscribers initialized, not entering developer mode or publishing.")
        return 0

    print("Releasing MotionSwitcher for rt/lowcmd developer control ...")
    enter_lowcmd_dev_mode(float(args.switch_timeout))

    pub = LowCmdPublisher()
    rate_hz = max(5.0, float(args.rate_hz))
    dt = 1.0 / rate_hz
    max_step = max(0.001, float(args.max_step_rad))
    ramp_s = max(0.1, float(args.ramp_s))
    hold_s = max(0.0, float(args.hold_s))
    max_tilt = math.radians(float(args.abort_tilt_deg))
    started = time.monotonic()
    last_status = 0.0
    q_cmd = list(first.q)

    print(
        f"Publishing {LOWCMD_TOPIC} at {rate_hz:.1f} Hz: "
        f"ramp_s={ramp_s:.1f}, hold_s={hold_s:.1f}, max_step={max_step:.4f} rad"
    )
    aborted = False
    try:
        while True:
            now = time.monotonic()
            low = lowstate.latest()
            if low is None or (now - low.timestamp) > float(args.max_lowstate_age):
                raise RuntimeError(f"{LOWSTATE_TOPIC} is stale.")

            attitude, source = select_attitude(low, odom_state.latest(), float(args.max_odom_age))
            if attitude is not None:
                roll, pitch, _ = attitude
                if abs(roll) > max_tilt or abs(pitch) > max_tilt:
                    raise RuntimeError(
                        f"Abort tilt exceeded: roll={math.degrees(roll):+.1f} deg "
                        f"pitch={math.degrees(pitch):+.1f} deg"
                    )

            elapsed = now - started
            alpha = smoothstep(elapsed / ramp_s)
            base = [
                float(low.q[i]) * (1.0 - alpha) + float(STAND_TARGET[i]) * alpha
                for i in range(N_JOINTS)
            ]
            target = build_stabilized_target(base, attitude, args)
            q_cmd = limited_step(q_cmd, target, max_step)
            pub.write(q_cmd, low.mode_machine)

            if now - last_status >= float(args.status_period_s):
                last_status = now
                if attitude is not None:
                    print(
                        f"t={elapsed:5.1f}s alpha={alpha:.2f} attitude={source} "
                        f"roll={math.degrees(attitude[0]):+5.1f} "
                        f"pitch={math.degrees(attitude[1]):+5.1f}"
                    )
                else:
                    print(f"t={elapsed:5.1f}s alpha={alpha:.2f} attitude=none")

            if elapsed >= ramp_s + hold_s:
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        aborted = True
        print("\nInterrupted by user.")
    except Exception as exc:
        aborted = True
        print(f"\nAborted: {exc}")
    finally:
        latest = lowstate.latest()
        if latest is not None:
            for _ in range(5):
                pub.write(q_cmd, latest.mode_machine, kp_scale=0.0)
                time.sleep(0.02)

    print("Done." if not aborted else "Stopped.")
    return 1 if aborted else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bring the G1 toward a captured standing pose using rt/lowcmd, rt/lowstate, and rt/odommodestate.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iface", default=default_dds_iface("eth0"))
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--switch-timeout", type=float, default=10.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--ramp-s", type=float, default=8.0)
    parser.add_argument("--hold-s", type=float, default=5.0)
    parser.add_argument("--max-step-rad", type=float, default=0.012)
    parser.add_argument("--pitch-gain", type=float, default=0.18)
    parser.add_argument("--roll-gain", type=float, default=0.14)
    parser.add_argument("--max-correction", type=float, default=0.10)
    parser.add_argument("--abort-tilt-deg", type=float, default=22.0)
    parser.add_argument("--max-lowstate-age", type=float, default=0.25)
    parser.add_argument("--max-odom-age", type=float, default=0.35)
    parser.add_argument("--status-period-s", type=float, default=0.5)
    parser.add_argument("--invert-pitch", action="store_true")
    parser.add_argument("--invert-roll", action="store_true")
    parser.add_argument("--print-target", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Required before any low-level command is published.")
    return parser.parse_args()


def main() -> None:
    raise SystemExit(run(parse_args()))


if __name__ == "__main__":
    main()
