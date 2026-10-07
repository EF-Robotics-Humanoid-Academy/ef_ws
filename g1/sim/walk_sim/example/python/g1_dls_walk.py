#!/usr/bin/env python3
"""G1 harness-supported walking demonstration using model-based DLS IK.

This adapts the damped-least-squares method documented in ``~/ef_ws/g1/WBC``.
Rather than maintaining a second, hard-coded kinematic chain, it asks MuJoCo for
the G1 model Jacobians.  The controller solves Cartesian trajectories for both
ankles and wrists, then publishes the resulting targets on ``rt/lowcmd``.

It is deliberately a *kinematic* gait demo: the elastic band should remain on
while using it.  It is not a balance controller and must not be used to make a
free-standing robot walk on the ground.  The optional real-robot mode is
deliberately gated and requires a support frame plus current ``rt/lowstate``.
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
CONTROL_DT = 0.004
LEFT_LEG = list(range(0, 6))
RIGHT_LEG = list(range(6, 12))
LEFT_ARM = list(range(15, 22))
RIGHT_ARM = list(range(22, 29))


def require_selected_cyclonedds():
    """Avoid publishing with a different DDS library than the Python binding."""
    try:
        maps = Path("/proc/self/maps").read_text()
    except OSError:
        return
    cyclonedds_home = os.environ.get("CYCLONEDDS_HOME")
    if not cyclonedds_home:
        raise RuntimeError("CYCLONEDDS_HOME is unset; use ./run_g1_dls_walk.sh")
    expected_library = str((Path(cyclonedds_home) / "lib" / "libddsc.so").resolve())
    loaded_ddsc = [line for line in maps.splitlines() if "libddsc.so" in line]
    if not any(expected_library in line for line in loaded_ddsc):
        raise RuntimeError(
            f"Expected CycloneDDS library {expected_library}, but it is not loaded. "
            "Start this controller with ./run_g1_dls_walk.sh."
        )


class G1DlsKinematics:
    """Damped least-squares position IK, warm-started from the prior solution."""

    def __init__(self, model_path: Path, q_reference):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.q_reference = np.asarray(q_reference, dtype=float).copy()
        self.q = self.q_reference.copy()
        self.qpos_addresses = []
        self.dof_addresses = []
        self.lower = []
        self.upper = []
        for motor_index in range(MOTOR_COUNT):
            joint_id = int(self.model.actuator_trnid[motor_index, 0])
            self.qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            self.dof_addresses.append(int(self.model.jnt_dofadr[joint_id]))
            self.lower.append(float(self.model.jnt_range[joint_id, 0]))
            self.upper.append(float(self.model.jnt_range[joint_id, 1]))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        self.body_ids = {
            "left_foot": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link"),
            "right_foot": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link"),
            "left_hand": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link"),
            "right_hand": mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link"),
        }
        if min(self.body_ids.values()) < 0:
            raise RuntimeError("Expected G1 ankle/wrist bodies are absent from the MJCF")
        self._forward()
        self.home_positions = {name: self.data.xpos[body_id].copy() for name, body_id in self.body_ids.items()}

    def _forward(self):
        for index, qpos_address in enumerate(self.qpos_addresses):
            self.data.qpos[qpos_address] = self.q[index]
        mujoco.mj_forward(self.model, self.data)

    def solve_position(self, body_name, motor_indices, target, iterations=20, damping=0.05):
        """Solve a 3-D task using WBC-style Jᵀ(JJᵀ+λ²I)⁻¹e updates.

        A small posture term resolves the redundant DOFs toward the captured
        start pose, keeping knees, wrists, and hip yaw from drifting.
        """
        body_id = self.body_ids[body_name]
        motor_indices = np.asarray(motor_indices, dtype=int)
        dofs = np.asarray([self.dof_addresses[i] for i in motor_indices], dtype=int)
        posture_gain = 0.01
        for _ in range(iterations):
            self._forward()
            error = np.asarray(target) - self.data.xpos[body_id]
            if np.linalg.norm(error) < 0.002:
                break
            jacobian = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacobian, None, body_id)
            task_jacobian = jacobian[:, dofs]
            posture_error = self.q_reference[motor_indices] - self.q[motor_indices]
            # The first line is the DLS update used by WBC.  The second,
            # stacked term is weak null-space posture regularisation.
            augmented_jacobian = np.vstack((task_jacobian, math.sqrt(posture_gain) * np.eye(len(dofs))))
            augmented_error = np.concatenate((error, math.sqrt(posture_gain) * posture_error))
            update = augmented_jacobian.T @ np.linalg.solve(
                augmented_jacobian @ augmented_jacobian.T + damping * damping * np.eye(augmented_jacobian.shape[0]),
                augmented_error,
            )
            update = np.clip(update, -0.12, 0.12)
            self.q[motor_indices] = np.clip(
                self.q[motor_indices] + update,
                self.lower[motor_indices],
                self.upper[motor_indices],
            )
        return self.q.copy()


latest_q = None
latest_mode_machine = None
last_low_state_time = 0.0
state_lock = threading.Lock()


def on_low_state(message: LowState_):
    global latest_q, latest_mode_machine, last_low_state_time
    with state_lock:
        latest_q = [message.motor_state[index].q for index in range(MOTOR_COUNT)]
        latest_mode_machine = message.mode_machine
        last_low_state_time = time.monotonic()


class JointRamp:
    """Limits every commanded joint increment, independent of IK output."""

    def __init__(self, start, max_speed):
        self.q = np.asarray(start, dtype=float).copy()
        self.max_speed = max_speed

    def update(self, target, dt):
        maximum_step = self.max_speed * dt
        self.q += np.clip(np.asarray(target) - self.q, -maximum_step, maximum_step)
        return self.q


def make_command(q_home, physical_robot=False, mode_machine=5):
    command = unitree_hg_msg_dds__LowCmd_()
    command.mode_machine = mode_machine
    for index, motor in enumerate(command.motor_cmd):
        motor.mode = 0x01
        motor.q = q_home[index] if index < MOTOR_COUNT else 0.0
        motor.dq = motor.tau = 0.0
        motor.kp, motor.kd = (220.0, 6.0) if index < 12 else (100.0, 3.0)
        if index >= 15:
            motor.kp, motor.kd = (35.0, 1.5)
        if index in (20, 21, 27, 28):
            motor.kp, motor.kd = (10.0, 0.8)
        if physical_robot:
            # The support frame carries the body; these conservative gains and
            # the position-rate limiter make this a slow range-of-motion test.
            motor.kp, motor.kd = (60.0, 2.0) if index < 12 else (30.0, 1.0)
            if index >= 15:
                motor.kp, motor.kd = (15.0, 0.6)
    return command


def gait_reference(q_home):
    """A modest crouch removes the straight-leg IK singularity before walking."""
    q = np.asarray(q_home, dtype=float).copy()
    for hip_pitch, knee, ankle_pitch in ((0, 3, 4), (6, 9, 10)):
        q[hip_pitch] = -0.30
        q[knee] = 0.60
        q[ankle_pitch] = -0.30
    return q


def targets(kinematics, phase, stride, lift, arm_swing):
    """Alternating feet plus opposite-phase arm swings, all relative to home."""
    result = {}
    for side, phase_offset in (("left", 0.0), ("right", math.pi)):
        foot_phase = phase + phase_offset
        foot = kinematics.home_positions[f"{side}_foot"].copy()
        # In stance the foot traces backwards beneath the torso; in swing it
        # comes forward and lifts.  This is the familiar treadmill-frame gait.
        foot[0] += 0.5 * stride * math.sin(foot_phase)
        foot[2] += lift * max(0.0, math.sin(foot_phase))
        result[f"{side}_foot"] = foot

        hand = kinematics.home_positions[f"{side}_hand"].copy()
        hand_phase = foot_phase + math.pi
        hand[0] += arm_swing * math.sin(hand_phase)
        hand[2] += 0.35 * arm_swing * math.cos(hand_phase)
        result[f"{side}_hand"] = hand
    return result


def dry_run(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    q_home = [model.qpos0[int(model.jnt_qposadr[int(model.actuator_trnid[i, 0])])] for i in range(MOTOR_COUNT)]
    ik = G1DlsKinematics(model_path, gait_reference(q_home))
    desired = targets(ik, 0.7, 0.05, 0.025, 0.035)
    for body, motors in (("left_foot", LEFT_LEG), ("right_foot", RIGHT_LEG), ("left_hand", LEFT_ARM), ("right_hand", RIGHT_ARM)):
        ik.solve_position(body, motors, desired[body])
    ik._forward()
    errors = {name: float(np.linalg.norm(desired[name] - ik.data.xpos[ik.body_ids[name]])) for name in desired}
    print("DLS model check completed; endpoint errors (m):", {name: round(error, 4) for name, error in errors.items()})


def model_home(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    return np.array([
        model.qpos0[int(model.jnt_qposadr[int(model.actuator_trnid[index, 0])])]
        for index in range(MOTOR_COUNT)
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=("simulator", "robot"), default="simulator",
                        help="where to publish commands (default: simulator)")
    parser.add_argument("--interface", default="lo", help="DDS interface (simulator default: lo)")
    parser.add_argument("--domain-id", default=1, type=int, help="DDS domain ID (default: 1)")
    parser.add_argument("--frequency", type=float, help="gait frequency in Hz")
    parser.add_argument("--stride", type=float, help="foot fore/aft travel in metres")
    parser.add_argument("--lift", type=float, help="swing-foot lift in metres")
    parser.add_argument("--arm-swing", type=float, help="wrist fore/aft travel in metres")
    parser.add_argument("--posture-time", type=float, help="seconds used to enter the bent-knee posture")
    parser.add_argument("--max-joint-speed", type=float, help="per-joint q-command rate limit in rad/s")
    parser.add_argument("--confirm-harnessed", action="store_true",
                        help="required acknowledgement before publishing to a physical robot")
    parser.add_argument("--dry-run", action="store_true", help="validate the IK without DDS or a running simulator")
    args = parser.parse_args()
    model_path = Path(__file__).resolve().parents[2] / "unitree_robots" / "g1" / "scene.xml"
    if args.dry_run:
        dry_run(model_path)
        return

    require_selected_cyclonedds()

    if args.target == "robot":
        if not args.confirm_harnessed:
            parser.error("robot mode requires --confirm-harnessed; use only with the G1 secured in its support frame")
        if args.interface == "lo":
            parser.error("robot mode needs the robot-facing network interface, not lo")
        defaults = dict(frequency=0.20, stride=0.02, lift=0.01, arm_swing=0.015,
                        posture_time=8.0, max_joint_speed=0.15)
    else:
        defaults = dict(frequency=0.55, stride=0.05, lift=0.025, arm_swing=0.035,
                        posture_time=2.0, max_joint_speed=1.2)
    for name, value in defaults.items():
        if getattr(args, name) is None:
            setattr(args, name, value)

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
            q_home = np.asarray(latest_q, dtype=float).copy()
            mode_machine = latest_mode_machine
    else:
        # The C++ simulator's low-state publisher is optional; its initial MJCF
        # pose is sufficient to start this visualisation immediately.
        q_home = model_home(model_path)
        mode_machine = 5
    q_gait = gait_reference(q_home)
    ik = G1DlsKinematics(model_path, q_gait)
    command = make_command(
        q_home,
        physical_robot=args.target == "robot",
        mode_machine=mode_machine,
    )
    crc = CRC()
    start = time.monotonic()
    limiter = JointRamp(q_home, args.max_joint_speed)
    print(f"Entering gait posture over {args.posture_time:.1f}s; joint commands limited to {args.max_joint_speed:.2f} rad/s.")
    try:
        while True:
            elapsed = time.monotonic() - start
            if args.target == "robot":
                with state_lock:
                    state_age = time.monotonic() - last_low_state_time
                if state_age > 0.25:
                    raise RuntimeError("rt/lowstate became stale; stopping command publication")
            # Enter the gait posture gently before the first swing step.
            ramp = min(elapsed / args.posture_time, 1.0)
            if ramp < 1.0:
                desired_q = (1.0 - ramp) * np.asarray(q_home) + ramp * q_gait
                commanded_q = limiter.update(desired_q, CONTROL_DT)
                for index in range(MOTOR_COUNT):
                    command.motor_cmd[index].q = float(commanded_q[index])
                command.crc = crc.Crc(command)
                publisher.Write(command)
                time.sleep(CONTROL_DT)
                continue
            phase = 2.0 * math.pi * args.frequency * (elapsed - args.posture_time)
            desired = targets(ik, phase, args.stride, args.lift, args.arm_swing)
            ik.solve_position("left_foot", LEFT_LEG, desired["left_foot"])
            ik.solve_position("right_foot", RIGHT_LEG, desired["right_foot"])
            ik.solve_position("left_hand", LEFT_ARM, desired["left_hand"])
            ik.solve_position("right_hand", RIGHT_ARM, desired["right_hand"])
            commanded_q = limiter.update(ik.q, CONTROL_DT)
            for index in range(MOTOR_COUNT):
                command.motor_cmd[index].q = float(commanded_q[index])
            command.crc = crc.Crc(command)
            publisher.Write(command)
            time.sleep(CONTROL_DT)
    except KeyboardInterrupt:
        print("Stopped. Restart the simulator or run another controller to take over.")


if __name__ == "__main__":
    main()
