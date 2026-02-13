#!/usr/bin/env python3
import argparse
import signal
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from go2_step_sequence_with_imu import StepSequence


def main():
    parser = argparse.ArgumentParser(
        description="Simulate go2_step_sequence_with_imu using unitree_mujoco (simulate_python)."
    )
    parser.add_argument(
        "--iface",
        default="lo",
        help="Network interface for simulation (default: lo)",
    )
    parser.add_argument(
        "--domain",
        type=int,
        default=1,
        help="DDS domain id for simulation (default: 1)",
    )
    args = parser.parse_args()

    print("Simulation mode: ensure unitree_mujoco simulate_python is running.")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(args.domain, args.iface)

    seq = StepSequence()
    seq.Init()
    seq.Start()

    def _sigint(_signum, _frame):
        print("\nStopping...")
        try:
            seq.Stop()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    last_print = time.time()
    while True:
        now = time.time()
        if now - last_print > 1.0:
            last_print = now
            if seq.has_imu:
                print(
                    f"IMU r/p/y (deg): "
                    f"{seq.roll*180/3.14159:+6.1f}, {seq.pitch*180/3.14159:+6.1f}, {seq.yaw*180/3.14159:+6.1f}"
                )
            else:
                print("IMU not available in LowState (no stabilization applied).")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
