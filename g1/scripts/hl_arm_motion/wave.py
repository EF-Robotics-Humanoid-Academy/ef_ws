#!/usr/bin/env python3
"""
G1 high-level wave gesture.

Uses G1ArmActionClient to execute the built-in "high wave" action (ID 26).
The robot must already be standing (FSM-200) before running this script.

Usage:
    python wave.py --iface eth0
    python wave.py --iface eth0 --hold 5.0
"""
from __future__ import annotations

import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 high-level wave gesture.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--hold", type=float, default=4.0,
                        help="seconds to hold the wave before releasing")
    parser.add_argument("--no-release", action="store_true",
                        help="skip releasing the arm after the gesture")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    arm = G1ArmActionClient()
    arm.SetTimeout(10.0)
    arm.Init()

    print("Executing wave gesture (action 26) ...")
    code = arm.ExecuteAction(26)  # high wave
    if code != 0:
        print(f"ExecuteAction failed: code={code}")
        return

    time.sleep(args.hold)

    if not args.no_release:
        print("Releasing arm ...")
        arm.ExecuteAction(99)  # release arm
        time.sleep(1.0)

    print("Done.")


if __name__ == "__main__":
    main()
