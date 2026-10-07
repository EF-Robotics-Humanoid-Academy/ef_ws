#!/usr/bin/env python3
"""G1 boxing demonstration over DDS.

The default target is the MuJoCo simulator over loopback DDS.  Physical-robot
mode is deliberately gated: use only while the G1 is secured in its support
frame, and pass ``--target robot --confirm-harnessed`` with the robot-facing
network interface.
"""

import argparse
import math
import os
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


MOTOR_COUNT = 29
DT = 0.004
LEFT_ARM = (15, 16, 17, 18, 19, 20, 21)
RIGHT_ARM = (22, 23, 24, 25, 26, 27, 28)

latest_q = None
latest_mode_machine = None
last_low_state_time = 0.0
state_lock = threading.Lock()


def on_low_state(message: LowState_):
    """Capture the current pose so robot mode never starts from the MJCF pose."""
    global latest_q, latest_mode_machine, last_low_state_time
    with state_lock:
        latest_q = [message.motor_state[index].q for index in range(MOTOR_COUNT)]
        latest_mode_machine = message.mode_machine
        last_low_state_time = time.monotonic()


def require_selected_cyclonedds():
    """Ensure the Python DDS binding and its C library came from one install."""
    home = os.environ.get("CYCLONEDDS_HOME")
    if not home:
        raise RuntimeError("CYCLONEDDS_HOME is unset; use ./run_g1_boxing.sh")
    expected = str((Path(home) / "lib" / "libddsc.so").resolve())
    maps = Path("/proc/self/maps").read_text()
    if not any(expected in line for line in maps.splitlines() if "libddsc.so" in line):
        raise RuntimeError("The selected CycloneDDS library is not loaded; use ./run_g1_boxing.sh")


def model_home(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    return np.array([
        model.qpos0[int(model.jnt_qposadr[int(model.actuator_trnid[index, 0])])]
        for index in range(MOTOR_COUNT)
    ])


class JointRamp:
    def __init__(self, q_start, speed):
        self.q = np.asarray(q_start, dtype=float).copy()
        self.speed = speed

    def update(self, target):
        maximum_step = self.speed * DT
        self.q += np.clip(np.asarray(target) - self.q, -maximum_step, maximum_step)
        return self.q


def set_arm(target, indices, shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll):
    target[indices[0]] = shoulder_pitch
    target[indices[1]] = shoulder_roll
    target[indices[2]] = shoulder_yaw
    target[indices[3]] = elbow
    target[indices[4]] = wrist_roll


def boxing_pose(home, t):
    """Guard pose plus alternating 0.9-second jabs in joint space."""
    q = home.copy()
    # A compact guard: elbows bent and hands in front of the torso.
    set_arm(q, LEFT_ARM, 0.38, 0.22, 0.00, 0.95, 0.0)
    set_arm(q, RIGHT_ARM, 0.38, -0.22, 0.00, 0.95, 0.0)

    cycle = t % 1.8
    # Smooth 0 -> 1 -> 0 jab envelope; left then right.
    side_phase = cycle / 0.9 if cycle < 0.9 else (cycle - 0.9) / 0.9
    jab = math.sin(math.pi * side_phase) ** 2
    left_jab = cycle < 0.9
    if left_jab:
        set_arm(q, LEFT_ARM, 0.38 - 0.55 * jab, 0.22, -0.10 * jab, 0.95 - 0.72 * jab, 0.15 * jab)
        q[12] = 0.10 * jab  # waist yaw counters the punch
    else:
        set_arm(q, RIGHT_ARM, 0.38 - 0.55 * jab, -0.22, 0.10 * jab, 0.95 - 0.72 * jab, -0.15 * jab)
        q[12] = -0.10 * jab
    return q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("simulator", "robot"), default="simulator",
                        help="where to publish commands (default: simulator)")
    parser.add_argument("--interface", default="lo", help="DDS interface (simulator default: lo)")
    parser.add_argument("--domain-id", default=1, type=int, help="DDS domain ID (default: 1)")
    parser.add_argument("--max-joint-speed", type=float,
                        help="per-joint q-command rate limit in rad/s")
    parser.add_argument("--confirm-harnessed", action="store_true",
                        help="required acknowledgement before publishing to a physical robot")
    args = parser.parse_args()

    require_selected_cyclonedds()
    project = Path(__file__).resolve().parents[2]
    if args.target == "robot":
        if not args.confirm_harnessed:
            parser.error("robot mode requires --confirm-harnessed; use only with the G1 secured in its support frame")
        if args.interface == "lo":
            parser.error("robot mode needs the robot-facing network interface, not lo")
        max_joint_speed = args.max_joint_speed if args.max_joint_speed is not None else 0.15
    else:
        max_joint_speed = args.max_joint_speed if args.max_joint_speed is not None else 1.4

    ChannelFactoryInitialize(args.domain_id, args.interface)
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    if args.target == "robot":
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(on_low_state, 10)
        print("Waiting up to 15 seconds for the physical G1 low state...")
        deadline = time.monotonic() + 15.0
        while latest_q is None and time.monotonic() < deadline:
            time.sleep(0.01)
        if latest_q is None:
            raise RuntimeError("No rt/lowstate received. Check the interface, DDS domain, robot mode, and cable.")
        with state_lock:
            home = np.asarray(latest_q, dtype=float).copy()
            mode_machine = latest_mode_machine
    else:
        home = model_home(project / "unitree_robots" / "g1" / "scene.xml")
        mode_machine = 5

    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_machine = mode_machine
    for index, motor in enumerate(command.motor_cmd):
        motor.mode = 0x01
        motor.q = float(home[index]) if index < MOTOR_COUNT else 0.0
        motor.dq = motor.tau = 0.0
        motor.kp, motor.kd = (180.0, 5.0) if index < 15 else (35.0, 1.5)
        if index in (20, 21, 27, 28):
            motor.kp, motor.kd = (10.0, 0.8)
        if args.target == "robot":
            motor.kp, motor.kd = (30.0, 1.0) if index < 15 else (15.0, 0.6)
            if index in (20, 21, 27, 28):
                motor.kp, motor.kd = (8.0, 0.5)

    ramp = JointRamp(home, max_joint_speed)
    crc = CRC()
    start = time.monotonic()
    print(f"G1 {args.target} boxing demo active on {args.interface}, domain {args.domain_id}; "
          f"joint commands limited to {max_joint_speed:.2f} rad/s. Press Ctrl+C to stop.")
    try:
        while True:
            if args.target == "robot":
                with state_lock:
                    state_age = time.monotonic() - last_low_state_time
                if state_age > 0.25:
                    raise RuntimeError("rt/lowstate became stale; stopping command publication")
            target = boxing_pose(home, time.monotonic() - start)
            q = ramp.update(target)
            for index in range(MOTOR_COUNT):
                command.motor_cmd[index].q = float(q[index])
            command.crc = crc.Crc(command)
            publisher.Write(command)
            time.sleep(DT)
    except KeyboardInterrupt:
        print("Stopped. The receiver will release commands after its DDS timeout.")


if __name__ == "__main__":
    main()
