#!/usr/bin/env python3
"""
pbd_reproduce.py
===============

Reproduce a recorded arm joint trajectory from a .npz file created by
pbd_demonstrate.py.
"""
from __future__ import annotations

import argparse
import time
from typing import Dict, List

import numpy as np

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except Exception as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


WAIST_YAW_IDX = 12
NOT_USED_IDX = 29  # enable arm sdk


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded arm motion.")
    parser.add_argument("--iface", default="enp1s0", help="network interface for DDS")
    parser.add_argument("--file", default="/tmp/pbd_motion.npz", help="input .npz file")
    parser.add_argument("--speed", type=float, default=1.0, help="time scale (1.0=real-time)")
    parser.add_argument("--cmd-hz", type=float, default=50.0, help="command rate (Hz)")
    parser.add_argument("--kp", type=float, default=40.0, help="arm joint kp")
    parser.add_argument("--kd", type=float, default=1.0, help="arm joint kd")
    args = parser.parse_args()

    hanger_boot_sequence(iface=args.iface)
    ChannelFactoryInitialize(0, args.iface)

    data = np.load(args.file)
    joints = data["joints"].astype(int).tolist()
    ts = data["ts"].astype(float)
    qs = data["qs"].astype(float)

    if len(ts) == 0 or len(qs) == 0:
        raise SystemExit("No samples in motion file.")

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.motor_cmd[NOT_USED_IDX].q = 1
    crc = CRC()

    dt = 1.0 / max(1e-6, args.cmd_hz)
    start = time.time()
    idx = 0
    total = len(ts)

    print(f"Replaying {total} samples @ {args.cmd_hz} Hz (speed={args.speed})")
    try:
        while idx < total:
            now = time.time() - start
            target_t = ts[idx] / max(1e-6, args.speed)
            if now < target_t:
                time.sleep(min(dt, target_t - now))
                continue

            for j_i, j_idx in enumerate(joints):
                mc = cmd.motor_cmd[int(j_idx)]
                mc.q = float(qs[idx, j_i])
                mc.kp = float(args.kp)
                mc.kd = float(args.kd)
                mc.tau = 0.0

            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            idx += 1

            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    print("Replay complete.")


if __name__ == "__main__":
    main()
