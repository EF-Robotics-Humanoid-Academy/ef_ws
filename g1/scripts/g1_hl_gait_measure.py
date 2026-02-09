#!/usr/bin/env python3
"""
Measure steps and distance for G1 high-level locomotion.

Subscribes to rt/sportmodestate for position + foot_force, and optionally
commands high-level gait via LocoClient. When commanding motion, the
hanger boot sequence is used to ensure a safe FSM-200 state.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
import importlib
from dataclasses import dataclass
from typing import Any, Iterable, Optional

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


@dataclass
class SportState:
    ts: float
    position: list[float]
    gait_type: Optional[int]
    foot_force: list[float]


class SportCache:
    def __init__(self) -> None:
        self._last: Optional[SportState] = None

    def cb(self, msg: Any) -> None:
        try:
            pos = [float(v) for v in msg.position]
        except Exception:
            pos = []
        gait = None
        if hasattr(msg, "gait_type"):
            try:
                gait = int(msg.gait_type)
            except Exception:
                gait = None
        forces: list[float] = []
        if hasattr(msg, "foot_force"):
            try:
                forces = [float(v) for v in msg.foot_force]
            except Exception:
                forces = []
        self._last = SportState(ts=time.time(), position=pos, gait_type=gait, foot_force=forces)

    def get(self) -> Optional[SportState]:
        return self._last


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


@dataclass
class ImuState:
    ts: float
    rpy: list[float]


class ImuCache:
    def __init__(self) -> None:
        self._last: Optional[ImuState] = None

    def cb(self, msg: Any) -> None:
        try:
            imu = msg.imu_state
            rpy = [float(imu.rpy[0]), float(imu.rpy[1]), float(imu.rpy[2])]
        except Exception:
            return
        self._last = ImuState(ts=time.time(), rpy=rpy)

    def get(self) -> Optional[ImuState]:
        return self._last


class StepCounter:
    def __init__(self, threshold: float) -> None:
        self.threshold = threshold
        self.prev_contact: list[bool] = []
        self.counts: list[int] = []
        self.total: int = 0

    def update(self, forces: Iterable[float]) -> None:
        values = list(forces)
        if not values:
            return
        if not self.prev_contact:
            self.prev_contact = [v > self.threshold for v in values]
            self.counts = [0 for _ in values]
            return
        if len(values) != len(self.prev_contact):
            self.prev_contact = [v > self.threshold for v in values]
            self.counts = [0 for _ in values]
            self.total = 0
            return
        contacts = [v > self.threshold for v in values]
        for i, (prev, now) in enumerate(zip(self.prev_contact, contacts)):
            if (not prev) and now:
                self.counts[i] += 1
                self.total += 1
        self.prev_contact = contacts


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


def _wrap_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure steps + distance during G1 high-level gait.")
    parser.add_argument("--iface", default="enp1s0", help="network interface for DDS")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to measure")
    parser.add_argument("--vx", type=float, default=0.3, help="forward velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="lateral velocity (m/s)")
    parser.add_argument("--vyaw", type=float, default=0.0, help="yaw rate (rad/s)")
    parser.add_argument("--target-m", type=float, default=0.0, help="stop after reaching displacement (m)")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="sample rate (Hz)")
    parser.add_argument("--cmd-hz", type=float, default=20.0, help="command rate (Hz)")
    parser.add_argument("--force-threshold", type=float, default=5.0, help="foot force contact threshold")
    parser.add_argument("--stride-m", type=float, default=0.5, help="stride length for fallback steps")
    parser.add_argument("--yaw-kp", type=float, default=0.8, help="IMU yaw correction gain")
    parser.add_argument("--yaw-max", type=float, default=0.6, help="max yaw correction (rad/s)")
    parser.add_argument("--lat-kp", type=float, default=0.3, help="lateral correction gain (m/s per m)")
    parser.add_argument("--lat-max", type=float, default=0.3, help="max lateral correction (m/s)")
    parser.add_argument("--no-imu", action="store_true", help="disable IMU-based yaw correction")
    parser.add_argument("--no-command", action="store_true", help="only measure, do not command gait")
    parser.add_argument("--csv", help="optional CSV log path")
    args = parser.parse_args()

    # If commanding motion, use the safety boot sequence.
    loco = None
    if not args.no_command:
        loco = hanger_boot_sequence(iface=args.iface)
    else:
        ChannelFactoryInitialize(0, args.iface)

    cache = SportCache()
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(cache.cb, 10)

    imu_cache = ImuCache()
    if not args.no_imu:
        LowState = _resolve_lowstate_type()
        if LowState is not None:
            imu_sub = ChannelSubscriber("rt/lowstate", LowState)
            imu_sub.Init(imu_cache.cb, 10)
        else:
            print("WARN: LowState_ type not found; IMU correction disabled.")
            args.no_imu = True

    dt = 1.0 / max(1e-6, args.sample_hz)
    cmd_dt = 1.0 / max(1e-6, args.cmd_hz)

    step_counter = StepCounter(args.force_threshold)
    total_dist = 0.0
    start_pos: Optional[list[float]] = None
    last_pos: Optional[list[float]] = None
    gait_samples = 0
    gait_active_samples = 0
    yaw0: Optional[float] = None
    y0: Optional[float] = None

    csv_file = None
    writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "t",
            "x",
            "y",
            "z",
            "gait_type",
            "total_dist",
            "step_total",
            "foot_force",
        ])

    start_time = time.monotonic()
    next_sample = start_time
    next_cmd = start_time

    try:
        while True:
            now = time.monotonic()
            if now - start_time >= args.duration:
                break
            if args.target_m > 0.0 and start_pos is not None and last_pos is not None:
                if math.hypot(last_pos[0] - start_pos[0], last_pos[1] - start_pos[1]) >= args.target_m:
                    break
            if now >= next_cmd and loco is not None:
                vy_cmd = args.vy
                vyaw_cmd = args.vyaw
                if start_pos is not None and y0 is not None:
                    lat_err = (last_pos[1] - y0) if last_pos is not None else 0.0
                    vy_cmd -= max(-args.lat_max, min(args.lat_max, args.lat_kp * lat_err))
                if not args.no_imu:
                    imu_state = imu_cache.get()
                    if imu_state is not None:
                        if yaw0 is None:
                            yaw0 = imu_state.rpy[2]
                        yaw_err = _wrap_angle(imu_state.rpy[2] - yaw0)
                        vyaw_cmd -= max(-args.yaw_max, min(args.yaw_max, args.yaw_kp * yaw_err))
                _command_velocity(loco, args.vx, vy_cmd, vyaw_cmd)
                next_cmd += cmd_dt
            if now < next_sample:
                time.sleep(min(0.005, next_sample - now))
                continue
            next_sample += dt

            state = cache.get()
            if state is None or len(state.position) < 2:
                continue

            if start_pos is None:
                start_pos = list(state.position)
                last_pos = list(state.position)
                y0 = start_pos[1]

            if last_pos is not None:
                dx = state.position[0] - last_pos[0]
                dy = state.position[1] - last_pos[1]
                total_dist += math.hypot(dx, dy)
            last_pos = list(state.position)

            if state.gait_type is not None:
                gait_samples += 1
                if state.gait_type != 0:
                    gait_active_samples += 1

            if state.foot_force:
                step_counter.update(state.foot_force)

            if writer is not None:
                writer.writerow([
                    time.time(),
                    state.position[0],
                    state.position[1],
                    state.position[2] if len(state.position) > 2 else 0.0,
                    state.gait_type if state.gait_type is not None else "",
                    total_dist,
                    step_counter.total,
                    list(state.foot_force),
                ])
    finally:
        if loco is not None:
            _stop(loco)
        if csv_file is not None:
            csv_file.close()

    displacement = 0.0
    if start_pos is not None and last_pos is not None:
        displacement = math.hypot(last_pos[0] - start_pos[0], last_pos[1] - start_pos[1])

    print("=== G1 High-Level Gait Measurement ===")
    print(f"Duration: {args.duration:.2f}s")
    if args.target_m > 0.0:
        print(f"Target displacement: {args.target_m:.3f} m")
    print(f"Path length: {total_dist:.3f} m")
    print(f"Displacement: {displacement:.3f} m")

    if step_counter.total > 0:
        print(f"Step count (contact events): {step_counter.total}")
        print(f"Per-foot counts: {step_counter.counts}")
    else:
        est_steps = total_dist / max(1e-6, args.stride_m)
        print("Foot force not available; using stride-length estimate.")
        print(f"Estimated steps: {est_steps:.1f} (stride={args.stride_m:.3f} m)")

    if gait_samples > 0:
        ratio = 100.0 * gait_active_samples / gait_samples
        print(f"Gait active samples: {gait_active_samples}/{gait_samples} ({ratio:.1f}%)")
    else:
        print("Gait type not available in sport state.")


if __name__ == "__main__":
    main()
