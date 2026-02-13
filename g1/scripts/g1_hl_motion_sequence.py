#!/usr/bin/env python3
"""
Simple high-level multi-step motion sequence for G1.

Sequence:
  1) Walk forward N meters
  2) Wave right hand
  3) Turn in place (degrees)
  4) Walk forward N meters

Connects over DDS using --iface (default: eth0).
Uses hanger_boot_sequence for safe FSM-200 initialization.
"""
from __future__ import annotations

import argparse
import importlib
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Any

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence

# Allow importing low_level helpers without package install.
_SCRIPTS_DIR = os.path.dirname(__file__)
_LOW_LEVEL_DIR = os.path.join(_SCRIPTS_DIR, "low_level")
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
if _LOW_LEVEL_DIR not in sys.path:
    sys.path.insert(0, _LOW_LEVEL_DIR)


def _command_velocity(client: LocoClient, vx: float, vy: float, vyaw: float) -> None:
    if hasattr(client, "SetVelocity"):
        client.SetVelocity(float(vx), float(vy), float(vyaw))
    else:
        client.Move(float(vx), float(vy), float(vyaw))


def _stop(client: LocoClient) -> None:
    if hasattr(client, "StopMove"):
        client.StopMove()
    else:
        client.Move(0.0, 0.0, 0.0)


def _try_import(module_path: str):
    try:
        return importlib.import_module(module_path)
    except Exception:
        return None


def _resolve_lowstate_type() -> Optional[type]:
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        module = _try_import(module_path)
        if module and hasattr(module, "LowState_"):
            return getattr(module, "LowState_")
    return None


class ImuCache:
    def __init__(self) -> None:
        self._last: Optional[list[float]] = None

    def cb(self, msg: Any) -> None:
        try:
            imu = msg.imu_state
            self._last = [float(imu.rpy[0]), float(imu.rpy[1]), float(imu.rpy[2])]
        except Exception:
            return

    def get(self) -> Optional[list[float]]:
        return self._last


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


@dataclass
class SportState:
    ts: float
    position: list[float]


class SportCache:
    def __init__(self) -> None:
        self._last: Optional[SportState] = None

    def cb(self, msg: Any) -> None:
        try:
            pos = [float(v) for v in msg.position]
        except Exception:
            return
        self._last = SportState(ts=time.time(), position=pos)

    def get(self) -> Optional[SportState]:
        return self._last


def _send_for_duration(
    client: LocoClient,
    vx: float,
    vy: float,
    vyaw: float,
    duration: float,
    cmd_hz: float,
    imu_cache: Optional[ImuCache] = None,
    yaw_kp: float = 0.8,
    yaw_max: float = 0.6,
    hold_yaw: bool = True,
) -> None:
    dt = 1.0 / max(1e-6, cmd_hz)
    end_time = time.monotonic() + max(0.0, duration)
    next_cmd = time.monotonic()
    yaw0: Optional[float] = None
    while time.monotonic() < end_time:
        now = time.monotonic()
        if now >= next_cmd:
            vyaw_cmd = vyaw
            if imu_cache is not None and hold_yaw:
                rpy = imu_cache.get()
                if rpy is not None:
                    if yaw0 is None:
                        yaw0 = rpy[2]
                    yaw_err = _wrap_angle(rpy[2] - yaw0)
                    vyaw_cmd -= max(-yaw_max, min(yaw_max, yaw_kp * yaw_err))
            _command_velocity(client, vx, vy, vyaw_cmd)
            next_cmd += dt
        else:
            time.sleep(min(0.005, next_cmd - now))


def _ramp_scale(elapsed: float, remaining: float, ramp_time: float) -> float:
    if ramp_time <= 0.0:
        return 1.0
    up = min(1.0, max(0.0, elapsed / ramp_time))
    down = min(1.0, max(0.0, remaining / ramp_time))
    return min(up, down)


def _send_until_distance(
    client: LocoClient,
    distance_m: float,
    speed_mps: float,
    cmd_hz: float,
    sport_cache: SportCache,
    imu_cache: Optional[ImuCache],
    yaw_kp: float,
    yaw_max: float,
    ramp_time: float,
    timeout: float,
) -> None:
    dt = 1.0 / max(1e-6, cmd_hz)
    start_time = time.monotonic()
    next_cmd = start_time

    start_pos: Optional[list[float]] = None
    yaw0: Optional[float] = None
    while time.monotonic() - start_time < max(0.0, timeout):
        now = time.monotonic()
        state = sport_cache.get()
        if state and len(state.position) >= 2 and start_pos is None:
            start_pos = list(state.position)
        if start_pos is not None and state is not None:
            dx = state.position[0] - start_pos[0]
            dy = state.position[1] - start_pos[1]
            traveled = math.hypot(dx, dy)
            remaining = max(0.0, abs(distance_m) - traveled)
            if remaining <= 0.0:
                break
        else:
            remaining = abs(distance_m)

        if now >= next_cmd:
            elapsed = now - start_time
            scale = _ramp_scale(elapsed, remaining / max(1e-6, abs(speed_mps)), ramp_time)
            vx_cmd = math.copysign(abs(speed_mps) * scale, distance_m)
            vyaw_cmd = 0.0
            if imu_cache is not None:
                rpy = imu_cache.get()
                if rpy is not None:
                    if yaw0 is None:
                        yaw0 = rpy[2]
                    yaw_err = _wrap_angle(rpy[2] - yaw0)
                    vyaw_cmd -= max(-yaw_max, min(yaw_max, yaw_kp * yaw_err))
            _command_velocity(client, vx_cmd, 0.0, vyaw_cmd)
            next_cmd += dt
        else:
            time.sleep(min(0.005, next_cmd - now))


def _send_until_yaw(
    client: LocoClient,
    target_deg: float,
    yaw_rate: float,
    cmd_hz: float,
    imu_cache: ImuCache,
    ramp_time: float,
    yaw_kp: float,
    yaw_min_rate: float,
    yaw_tol_deg: float,
    timeout: float,
) -> None:
    dt = 1.0 / max(1e-6, cmd_hz)
    start_time = time.monotonic()
    next_cmd = start_time
    yaw0: Optional[float] = None
    target_rad = math.radians(target_deg)
    tol_rad = math.radians(abs(yaw_tol_deg))
    while time.monotonic() - start_time < max(0.0, timeout):
        now = time.monotonic()
        rpy = imu_cache.get()
        if rpy is not None and yaw0 is None:
            yaw0 = rpy[2]
        if rpy is not None and yaw0 is not None:
            yaw_err = _wrap_angle(rpy[2] - yaw0)
            remaining = abs(target_rad) - abs(yaw_err)
            if remaining <= tol_rad:
                break
        else:
            remaining = abs(target_rad)

        if now >= next_cmd:
            elapsed = now - start_time
            scale = _ramp_scale(elapsed, remaining / max(1e-6, abs(yaw_rate)), ramp_time)
            # Proportional slowing as we approach target to reduce overshoot.
            if rpy is not None and yaw0 is not None:
                yaw_err = _wrap_angle(rpy[2] - yaw0)
                err = abs(target_rad) - abs(yaw_err)
                raw = min(abs(yaw_rate), max(0.0, yaw_kp * err))
            else:
                raw = abs(yaw_rate)
            vyaw_mag = max(0.0, min(abs(yaw_rate), raw))
            if vyaw_mag > 0.0:
                vyaw_mag = max(vyaw_mag, abs(yaw_min_rate))
            vyaw_cmd = math.copysign(vyaw_mag * scale, target_rad)
            _command_velocity(client, 0.0, 0.0, vyaw_cmd)
            next_cmd += dt
        else:
            time.sleep(min(0.005, next_cmd - now))


class _WaveHelper:
    def __init__(self) -> None:
        self._client = None
        self._mode_set = False
        self._init_attempted = False

    def _try_init(self) -> None:
        if self._init_attempted:
            return
        self._init_attempted = True
        # Try a few likely client classes and paths.
        candidates = [
            ("unitree_sdk2py.g1.arm.g1_arm_client", "G1ArmClient"),
            ("unitree_sdk2py.g1.arm.g1_arm_client", "ArmClient"),
            ("unitree_sdk2py.g1.hand.g1_hand_client", "G1HandClient"),
            ("unitree_sdk2py.g1.hand.g1_hand_client", "HandClient"),
        ]
        for mod, cls in candidates:
            try:
                module = __import__(mod, fromlist=[cls])
                client_cls = getattr(module, cls)
                client = client_cls()
                if hasattr(client, "SetTimeout"):
                    client.SetTimeout(5.0)
                if hasattr(client, "Init"):
                    client.Init()
                self._client = client
                return
            except Exception:
                continue

    def wave_right(self) -> bool:
        self._try_init()
        if self._client is None:
            return False

        # Prefer a built-in wave if it exists.
        for name in ["WaveRight", "Wave", "WaveHand", "WaveRightHand"]:
            if hasattr(self._client, name):
                getattr(self._client, name)()
                return True

        # Try a minimal joint-motion style API if available.
        # These names are intentionally conservative to avoid unexpected behavior.
        if hasattr(self._client, "SetArmMode") and not self._mode_set:
            try:
                self._client.SetArmMode(1)
                self._mode_set = True
            except Exception:
                pass

        # If there is a generic "Pose" or "Move" call, we skip to avoid guessing.
        return False


def _run_low_level_wave(
    iface: str,
    steps_path: str,
    easing: str,
    cmd_hz_override: float,
    kp_override: float,
    kd_override: float,
    no_seed: bool,
) -> bool:
    try:
        from arm_motion import (
            ArmSdkController,
            _build_pose,
            _load_steps,
            _parse_descriptions,
            _pose_to_indexed,
        )
    except Exception:
        return False

    if not os.path.isabs(steps_path):
        steps_path = os.path.join(_LOW_LEVEL_DIR, steps_path)

    data = _load_steps(steps_path)
    arm = (data.get("arm") or "right").lower()
    cmd_hz = float(cmd_hz_override or data.get("cmd_hz") or 50.0)
    kp = float(kp_override or data.get("kp") or 40.0)
    kd = float(kd_override or data.get("kd") or 1.0)

    steps = data.get("steps") or []
    if not steps:
        raise SystemExit("Wave steps.json has no steps")

    arm_ctrl = ArmSdkController(iface, arm, cmd_hz, kp, kd)
    if not no_seed:
        arm_ctrl.seed_from_lowstate()

    current_pose_deg: dict[str, float] = {}
    for idx, step in enumerate(steps, start=1):
        name = step.get("name") or f"step_{idx}"
        duration = float(step.get("duration", 1.5))
        hold = float(step.get("hold", 0.0))

        angles = step.get("angles") or {}
        descriptions = step.get("descriptions") or []
        if descriptions:
            angles_from_desc = _parse_descriptions(descriptions, arm)
            angles.update(angles_from_desc)
        if not angles:
            raise SystemExit(f"Step '{name}' has no angles or descriptions")

        current_pose_deg = _build_pose(arm, angles, current_pose_deg)
        indexed_pose = _pose_to_indexed(arm, current_pose_deg)

        print(f"Wave: {name} (duration={duration}s hold={hold}s)")
        arm_ctrl.ramp_to_pose(indexed_pose, duration, easing)
        arm_ctrl.hold_pose(indexed_pose, hold)

    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 simple multi-step motion sequence.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--walk-m", type=float, default=1.0, help="walk distance (meters)")
    parser.add_argument("--walk-v", type=float, default=0.3, help="walk speed (m/s)")
    parser.add_argument("--turn-deg", type=float, default=180.0, help="turn angle (degrees)")
    parser.add_argument("--turn-vyaw", type=float, default=0.8, help="turn rate (rad/s)")
    parser.add_argument("--cmd-hz", type=float, default=20.0, help="command rate (Hz)")
    parser.add_argument("--yaw-kp", type=float, default=0.8, help="IMU yaw correction gain")
    parser.add_argument("--yaw-max", type=float, default=0.6, help="max yaw correction (rad/s)")
    parser.add_argument("--no-imu", action="store_true", help="disable IMU yaw correction")
    parser.add_argument("--ramp-time", type=float, default=0.4, help="seconds for speed ramp up/down")
    parser.add_argument("--walk-timeout", type=float, default=0.0, help="max seconds for each walk (0=auto)")
    parser.add_argument("--turn-timeout", type=float, default=0.0, help="max seconds for turn (0=auto)")
    parser.add_argument("--turn-kp", type=float, default=2.0, help="turn slowing gain (rad/s per rad)")
    parser.add_argument("--turn-min-rate", type=float, default=0.15, help="min yaw rate near target (rad/s)")
    parser.add_argument("--turn-tol-deg", type=float, default=2.0, help="stop when within this tolerance (deg)")
    parser.add_argument("--no-wave", action="store_true", help="skip right-hand wave")
    parser.add_argument("--wave-steps", default="", help="use low_level wave.json (path) instead of HL wave")
    parser.add_argument("--wave-easing", choices=["linear", "smooth"], default="smooth", help="wave easing profile")
    parser.add_argument("--wave-cmd-hz", type=float, default=0.0, help="override wave command rate (Hz)")
    parser.add_argument("--wave-kp", type=float, default=0.0, help="override wave joint kp")
    parser.add_argument("--wave-kd", type=float, default=0.0, help="override wave joint kd")
    parser.add_argument("--wave-no-seed", action="store_true", help="skip wave seeding from lowstate")
    args = parser.parse_args()

    loco = hanger_boot_sequence(iface=args.iface)

    wave = _WaveHelper()
    imu_cache: Optional[ImuCache] = None
    if not args.no_imu:
        LowState = _resolve_lowstate_type()
        if LowState is not None:
            imu_cache = ImuCache()
            imu_sub = ChannelSubscriber("rt/lowstate", LowState)
            imu_sub.Init(imu_cache.cb, 10)
        else:
            print("WARN: LowState_ type not found; IMU correction disabled.")

    sport_cache = SportCache()
    sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sport_sub.Init(sport_cache.cb, 10)

    walk_time = abs(args.walk_m) / max(1e-6, abs(args.walk_v))
    turn_rad = math.radians(args.turn_deg)
    turn_time = abs(turn_rad) / max(1e-6, abs(args.turn_vyaw))
    turn_dir = 1.0 if turn_rad >= 0.0 else -1.0
    walk_timeout = args.walk_timeout if args.walk_timeout > 0.0 else max(2.0, walk_time * 2.0)
    turn_timeout = args.turn_timeout if args.turn_timeout > 0.0 else max(2.0, turn_time * 2.0)

    try:
        # 1) Walk forward
        _send_until_distance(
            loco,
            distance_m=args.walk_m,
            speed_mps=abs(args.walk_v),
            cmd_hz=args.cmd_hz,
            sport_cache=sport_cache,
            imu_cache=imu_cache,
            yaw_kp=args.yaw_kp,
            yaw_max=args.yaw_max,
            ramp_time=args.ramp_time,
            timeout=walk_timeout,
        )
        _stop(loco)
        time.sleep(0.5)

        # 2) Wave right hand
        if not args.no_wave:
            if args.wave_steps:
                ok = _run_low_level_wave(
                    iface=args.iface,
                    steps_path=args.wave_steps,
                    easing=args.wave_easing,
                    cmd_hz_override=args.wave_cmd_hz,
                    kp_override=args.wave_kp,
                    kd_override=args.wave_kd,
                    no_seed=args.wave_no_seed,
                )
                if not ok:
                    print("Wave: low-level wave steps failed; falling back to HL wave.")
                    ok = wave.wave_right()
                time.sleep(1.0)
            else:
                ok = wave.wave_right()
                if not ok:
                    print("Wave: right-hand wave not supported by available SDK; skipping.")
                time.sleep(1.0)

        # 3) Turn in place
        if imu_cache is None:
            _send_for_duration(
                loco,
                vx=0.0,
                vy=0.0,
                vyaw=turn_dir * abs(args.turn_vyaw),
                duration=turn_time,
                cmd_hz=args.cmd_hz,
                imu_cache=None,
                hold_yaw=False,
            )
        else:
            _send_until_yaw(
                loco,
                target_deg=args.turn_deg,
                yaw_rate=abs(args.turn_vyaw),
                cmd_hz=args.cmd_hz,
                imu_cache=imu_cache,
                ramp_time=args.ramp_time,
                yaw_kp=args.turn_kp,
                yaw_min_rate=args.turn_min_rate,
                yaw_tol_deg=args.turn_tol_deg,
                timeout=turn_timeout,
            )
        _stop(loco)
        time.sleep(0.5)

        # 4) Walk forward
        _send_until_distance(
            loco,
            distance_m=args.walk_m,
            speed_mps=abs(args.walk_v),
            cmd_hz=args.cmd_hz,
            sport_cache=sport_cache,
            imu_cache=imu_cache,
            yaw_kp=args.yaw_kp,
            yaw_max=args.yaw_max,
            ramp_time=args.ramp_time,
            timeout=walk_timeout,
        )
        _stop(loco)
    finally:
        _stop(loco)


if __name__ == "__main__":
    main()
