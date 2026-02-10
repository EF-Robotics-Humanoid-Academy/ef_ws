#!/usr/bin/env python3
"""
pbd_demonstrate.py
==================

Record arm joint trajectories (PBD) while the user moves the arms.
Puts robot into balanced stand (FSM-200) using safety/hanger_boot_sequence.py,
then subscribes to LowState and logs joint positions over time.
"""
from __future__ import annotations

import argparse
import os
import time
from typing import Dict, List

import numpy as np

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
except Exception as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
WAIST_YAW_IDX = 12


def _resolve_lowstate_type():
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            mod = __import__(module_path, fromlist=["LowState_"])
            if hasattr(mod, "LowState_"):
                return getattr(mod, "LowState_")
        except Exception:
            continue
    return None


class Recorder:
    def __init__(self, joints: List[int]) -> None:
        self.joints = joints
        self.t0 = time.time()
        self.ts: List[float] = []
        self.qs: List[List[float]] = []
        self.last_update = 0.0

    def cb(self, msg):
        try:
            q = [float(msg.motor_state[j].q) for j in self.joints]
        except Exception:
            return
        now = time.time()
        self.ts.append(now - self.t0)
        self.qs.append(q)
        self.last_update = now


def main() -> None:
    parser = argparse.ArgumentParser(description="Record arm joint trajectories (PBD).")
    parser.add_argument("--iface", default="enp1s0", help="network interface for DDS")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="right", help="which arm(s) to record")
    parser.add_argument("--duration", type=float, default=15.0, help="seconds to record (0=until Ctrl+C)")
    parser.add_argument("--out", default="/tmp/pbd_motion.npz", help="output file (.npz)")
    args = parser.parse_args()

    # Ensure balanced stand (FSM-200)
    hanger_boot_sequence(iface=args.iface)
    ChannelFactoryInitialize(0, args.iface)

    LowState_ = _resolve_lowstate_type()
    if LowState_ is None:
        raise SystemExit("LowState_ type not found in unitree_sdk2py.")

    joints: List[int] = []
    if args.arm in ("left", "both"):
        joints.extend(LEFT_ARM_IDX)
    if args.arm in ("right", "both"):
        joints.extend(RIGHT_ARM_IDX)
    joints.append(WAIST_YAW_IDX)

    recorder = Recorder(joints)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(recorder.cb, 200)

    print(f"Recording joints {joints} for {args.duration}s (Ctrl+C to stop)...")
    t0 = time.time()
    try:
        while True:
            time.sleep(0.02)
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                break
    except KeyboardInterrupt:
        pass

    if not recorder.ts:
        raise SystemExit("No samples recorded. Is LowState publishing?")

    np.savez(
        args.out,
        joints=np.array(joints, dtype=np.int32),
        ts=np.array(recorder.ts, dtype=np.float32),
        qs=np.array(recorder.qs, dtype=np.float32),
    )
    print(f"Saved {len(recorder.ts)} samples to {args.out}")


if __name__ == "__main__":
    main()
