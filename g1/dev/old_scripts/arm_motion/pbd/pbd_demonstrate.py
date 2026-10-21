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
import threading
import time
from typing import List, Optional, Tuple

import numpy as np

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

except Exception as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


RIGHT_ARM_IDX = [22, 23, 24, 25, 26, 27, 28]
LEFT_ARM_IDX = [15, 16, 17, 18, 19, 20, 21]
WAIST_YAW_IDX = 12
NOT_USED_IDX = 29  # enable arm sdk
ARM_HAND_ZERO_TORQUE_IDX = LEFT_ARM_IDX + RIGHT_ARM_IDX


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
        self._lock = threading.Lock()
        self._latest_q: Optional[List[float]] = None
        self._latest_update = 0.0

    def cb(self, msg):
        try:
            q = [float(msg.motor_state[j].q) for j in self.joints]
        except Exception:
            return
        with self._lock:
            self._latest_q = q
            self._latest_update = time.time()

    def snapshot(self) -> Optional[Tuple[List[float], float]]:
        with self._lock:
            if self._latest_q is None:
                return None
            return list(self._latest_q), self._latest_update


class ArmZeroTorqueController:
    def __init__(self, joints: List[int]) -> None:
        self._joints = [int(j) for j in joints]
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1

    def write_zero_torque(self) -> None:
        for j_idx in self._joints:
            mc = self._cmd.motor_cmd[j_idx]
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


def main() -> None:
    parser = argparse.ArgumentParser(description="Record arm joint trajectories (PBD).")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both", help="which arm(s) to record")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to record (0=until Ctrl+C)")
    parser.add_argument("--poll-s", type=float, default=0.02, help="sample period in seconds")
    parser.add_argument("--out", default="/tmp/pbd_motion.npz", help="output file (.npz)")
    parser.add_argument(
        "--log",
        default="",
        help="CSV log path (default: <out>.csv)",
    )
    args = parser.parse_args()

    # Ensure balanced stand (FSM-200), then zero arm/hand torque for teaching.
    _ = hanger_boot_sequence(iface=args.iface)
    ChannelFactoryInitialize(0, args.iface)
    arm_ctrl = ArmZeroTorqueController(ARM_HAND_ZERO_TORQUE_IDX)
    for _ in range(5):
        arm_ctrl.write_zero_torque()
        time.sleep(0.02)

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

    print("Balanced stand active. Arm/hand joints set to zero torque.")
    print("Move joints freely to demonstrate.")
    print("Press <Enter> when finished (Ctrl+C also stops).")
    stop_event = threading.Event()

    def _wait_for_enter() -> None:
        try:
            input()
        except Exception:
            pass
        stop_event.set()

    threading.Thread(target=_wait_for_enter, daemon=True).start()

    log_path = args.log or f"{os.path.splitext(args.out)[0]}.csv"
    poll_s = max(1e-3, float(args.poll_s))

    print(f"Recording joints {joints} (duration limit: {args.duration}s, poll_s={poll_s})")
    t0 = time.time()
    ts: List[float] = []
    qs: List[List[float]] = []
    last_wait_log = 0.0

    with open(log_path, "w", encoding="utf-8") as f_log:
        f_log.write("t_s," + ",".join([f"j{j}" for j in joints]) + "\n")
        next_tick = t0
        try:
            while True:
                now = time.time()
                if now < next_tick:
                    time.sleep(min(0.02, next_tick - now))
                    continue
                next_tick += poll_s

                if stop_event.is_set():
                    break
                if args.duration > 0 and (time.time() - t0) >= args.duration:
                    break

                arm_ctrl.write_zero_torque()
                snap = recorder.snapshot()
                if snap is None:
                    if time.time() - last_wait_log >= 1.0:
                        print("Waiting for rt/lowstate...")
                        last_wait_log = time.time()
                    continue

                q, _ = snap
                t_rel = time.time() - t0
                ts.append(t_rel)
                qs.append(q)

                row = f"{t_rel:.6f}," + ",".join([f"{v:.6f}" for v in q])
                f_log.write(row + "\n")
                f_log.flush()
                print(f"[{len(ts):04d}] {row}")
        except KeyboardInterrupt:
            pass

    if not ts:
        raise SystemExit("No samples recorded. Is LowState publishing?")

    np.savez(
        args.out,
        joints=np.array(joints, dtype=np.int32),
        ts=np.array(ts, dtype=np.float32),
        qs=np.array(qs, dtype=np.float32),
        fk_qs=np.array(qs, dtype=np.float32),
        poll_s=np.array([poll_s], dtype=np.float32),
        representation=np.array(["joint_space"], dtype="<U16"),
    )
    print(f"Saved {len(ts)} samples to {args.out}")
    print(f"Logged sampled poses to {log_path}")


if __name__ == "__main__":
    main()
