#!/usr/bin/env python3
"""
Low-level wave using steps JSON (rt/arm_sdk).

Uses the same step parsing approach as arm_motion.py but defaults to wave.json.
"""
from __future__ import annotations

import argparse
import os

from arm_motion import (
    ArmSdkController,
    _build_pose,
    _load_steps,
    _parse_descriptions,
    _pose_to_indexed,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Wave using low-level arm SDK and steps JSON.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--steps", default="wave.json", help="path to steps.json (default: wave.json)")
    parser.add_argument("--arm", choices=["left", "right"], default="", help="override arm in file")
    parser.add_argument("--cmd-hz", type=float, default=0.0, help="override command rate (Hz)")
    parser.add_argument("--kp", type=float, default=0.0, help="override joint kp")
    parser.add_argument("--kd", type=float, default=0.0, help="override joint kd")
    parser.add_argument("--easing", choices=["linear", "smooth"], default="smooth", help="easing profile")
    parser.add_argument("--no-seed", action="store_true", help="skip seeding from lowstate")
    parser.add_argument("--dry-run", action="store_true", help="print resolved steps without commanding")
    args = parser.parse_args()

    steps_path = args.steps
    if not os.path.isabs(steps_path):
        steps_path = os.path.join(os.path.dirname(__file__), steps_path)

    data = _load_steps(steps_path)
    arm = (args.arm or data.get("arm") or "right").lower()
    cmd_hz = float(args.cmd_hz or data.get("cmd_hz") or 50.0)
    kp = float(args.kp or data.get("kp") or 40.0)
    kd = float(args.kd or data.get("kd") or 1.0)

    steps = data.get("steps") or []
    if not steps:
        raise SystemExit("No steps found in steps.json")

    arm_ctrl = ArmSdkController(args.iface, arm, cmd_hz, kp, kd)
    if not args.no_seed:
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

        if args.dry_run:
            print(f"[{name}] duration={duration}s hold={hold}s angles={current_pose_deg}")
            continue

        print(f"Executing {name} (duration={duration}s hold={hold}s)")
        arm_ctrl.ramp_to_pose(indexed_pose, duration, args.easing)
        arm_ctrl.hold_pose(indexed_pose, hold)


if __name__ == "__main__":
    main()
