import argparse
import os
import time

import numpy as np

from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# Disable CycloneDDS tracing to avoid config print crash
channel_module.ChannelConfigHasInterface = """<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface name=\"$__IF_NAME__$\" priority=\"default\" multicast=\"default\"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""
channel_module.ChannelConfigAutoDetermine = """<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS>
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""
os.environ.setdefault(
    "CYCLONEDDS_URI",
    "<CycloneDDS><Domain><Tracing><Category>none</Category></Tracing></Domain></CycloneDDS>",
)

G1_NUM_MOTOR = 29

# Conservative gains from the example
Kp = [
    60, 60, 60, 100, 40, 40,      # legs
    60, 60, 60, 100, 40, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]

Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms
]

# Joint limits from g1_29dof.xml (radians)
JOINT_LIMITS = [
    (-2.5307, 2.8798),   # LeftHipPitch
    (-0.5236, 2.9671),   # LeftHipRoll
    (-2.7576, 2.7576),   # LeftHipYaw
    (-0.087267, 2.8798), # LeftKnee
    (-0.87267, 0.5236),  # LeftAnklePitch
    (-0.2618, 0.2618),   # LeftAnkleRoll
    (-2.5307, 2.8798),   # RightHipPitch
    (-2.9671, 0.5236),   # RightHipRoll
    (-2.7576, 2.7576),   # RightHipYaw
    (-0.087267, 2.8798), # RightKnee
    (-0.87267, 0.5236),  # RightAnklePitch
    (-0.2618, 0.2618),   # RightAnkleRoll
    (-2.618, 2.618),     # WaistYaw
    (-0.52, 0.52),       # WaistRoll
    (-0.52, 0.52),       # WaistPitch
    (-3.0892, 2.6704),   # LeftShoulderPitch
    (-1.5882, 2.2515),   # LeftShoulderRoll
    (-2.618, 2.618),     # LeftShoulderYaw
    (-1.0472, 2.0944),   # LeftElbow
    (-1.97222, 1.97222), # LeftWristRoll
    (-1.61443, 1.61443), # LeftWristPitch
    (-1.61443, 1.61443), # LeftWristYaw
    (-3.0892, 2.6704),   # RightShoulderPitch
    (-2.2515, 1.5882),   # RightShoulderRoll
    (-2.618, 2.618),     # RightShoulderYaw
    (-1.0472, 2.0944),   # RightElbow
    (-1.97222, 1.97222), # RightWristRoll
    (-1.61443, 1.61443), # RightWristPitch
    (-1.61443, 1.61443), # RightWristYaw
]

class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28


class Mode:
    PR = 0
    AB = 1


class WakeUpController:
    def __init__(self, control_dt=0.002):
        self.control_dt_ = control_dt
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.mode_machine_ = 0
        self.have_mode_machine_ = False
        self.crc = CRC()
        self.time_ = 0.0
        self.phase_start_ = 0.0
        self.phase_index_ = 0
        self.phase_plan_ = []
        self.phase_from_ = None
        self.last_cmd_q_ = None
        self.ramp_time_ = 5.0

        # Smoother, safer actuation limits
        self.max_speed_rad_s_ = 0.25
        self.soft_limit_margin_ = 0.05
        self.kp_scale_ = 0.25
        self.kd_scale_ = 0.25
        self.arm_enable_phase_ = 2  # hold arms until crouched stand phase
        self.arm_indices_ = list(range(G1JointIndex.LeftShoulderPitch, G1_NUM_MOTOR))

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        if not self.have_mode_machine_:
            self.mode_machine_ = msg.mode_machine
            self.have_mode_machine_ = True

    def build_plan(self, base_q, scale=1.0):
        # Joint-space waypoints. Tune values as needed for your robot.
        # All poses start from current joint positions, then override key joints.
        def pose(overrides):
            q = np.array(base_q, dtype=float)
            for idx, val in overrides.items():
                q[idx] = val * scale
            return q

        # Phase 1: bring knees under body and put feet on ground.
        phase1 = pose({
            G1JointIndex.LeftHipPitch: 0.4,
            G1JointIndex.RightHipPitch: 0.4,
            G1JointIndex.LeftKnee: -0.9,
            G1JointIndex.RightKnee: -0.9,
            G1JointIndex.LeftAnklePitch: 0.5,
            G1JointIndex.RightAnklePitch: 0.5,
        })

        # Phase 2: place hands for support.
        phase2 = pose({
            G1JointIndex.LeftShoulderPitch: 0.6,
            G1JointIndex.RightShoulderPitch: 0.6,
            G1JointIndex.LeftElbow: -0.8,
            G1JointIndex.RightElbow: -0.8,
        })

        # Phase 3: push up to a crouched stand.
        phase3 = pose({
            G1JointIndex.LeftHipPitch: 0.2,
            G1JointIndex.RightHipPitch: 0.2,
            G1JointIndex.LeftKnee: -0.6,
            G1JointIndex.RightKnee: -0.6,
            G1JointIndex.LeftAnklePitch: 0.3,
            G1JointIndex.RightAnklePitch: 0.3,
            G1JointIndex.LeftShoulderPitch: 0.2,
            G1JointIndex.RightShoulderPitch: 0.2,
            G1JointIndex.LeftElbow: -0.4,
            G1JointIndex.RightElbow: -0.4,
        })

        # Phase 4: balanced stand (neutral-ish upper body).
        phase4 = pose({
            G1JointIndex.LeftHipPitch: 0.0,
            G1JointIndex.RightHipPitch: 0.0,
            G1JointIndex.LeftKnee: 0.0,
            G1JointIndex.RightKnee: 0.0,
            G1JointIndex.LeftAnklePitch: 0.0,
            G1JointIndex.RightAnklePitch: 0.0,
            G1JointIndex.LeftShoulderPitch: 0.0,
            G1JointIndex.RightShoulderPitch: 0.0,
            G1JointIndex.LeftElbow: 0.0,
            G1JointIndex.RightElbow: 0.0,
        })

        self.phase_plan_ = [
            (5.0, phase1),
            (5.0, phase2),
            (5.0, phase3),
            (5.0, phase4),
        ]

    def _apply_limits(self, q):
        limited = np.array(q, dtype=float)
        for i in range(G1_NUM_MOTOR):
            qmin, qmax = JOINT_LIMITS[i]
            margin = self.soft_limit_margin_
            qmin = qmin + margin
            qmax = qmax - margin
            if qmin < qmax:
                limited[i] = float(np.clip(limited[i], qmin, qmax))
        return limited

    def _rate_limit(self, desired_q, current_q):
        if self.last_cmd_q_ is None:
            self.last_cmd_q_ = np.array(current_q, dtype=float)
        max_step = self.max_speed_rad_s_ * self.control_dt_
        delta = desired_q - self.last_cmd_q_
        delta = np.clip(delta, -max_step, max_step)
        self.last_cmd_q_ = self.last_cmd_q_ + delta
        return self.last_cmd_q_

    def start(self):
        self.thread = RecurrentThread(
            interval=self.control_dt_, target=self.step, name="wake_up"
        )
        self.thread.Start()

    def step(self):
        if self.low_state is None:
            return
        if not self.have_mode_machine_:
            return

        if not self.phase_plan_:
            base_q = [self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)]
            self.build_plan(base_q)
            self.phase_start_ = self.time_
            self.phase_from_ = np.array(base_q, dtype=float)
            self.last_cmd_q_ = np.array(base_q, dtype=float)

        self.time_ += self.control_dt_

        # Determine current phase
        elapsed = self.time_ - self.phase_start_
        phase_dur, phase_target = self.phase_plan_[self.phase_index_]
        if elapsed > phase_dur:
            self.phase_index_ = min(self.phase_index_ + 1, len(self.phase_plan_) - 1)
            self.phase_start_ = self.time_
            self.phase_from_ = np.array(
                [self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)],
                dtype=float,
            )
            elapsed = 0.0
            phase_dur, phase_target = self.phase_plan_[self.phase_index_]

        # Smooth interpolation using tanh (similar to stand_go2 example).
        # About 1.2s is a good rise time; scale based on phase duration.
        rise = max(0.6, min(1.2, phase_dur * 0.8))
        alpha = np.tanh(elapsed / rise)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        if self.phase_from_ is None:
            self.phase_from_ = np.array(
                [self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)],
                dtype=float,
            )
        desired_q = self.phase_from_ * (1.0 - alpha) + phase_target * alpha
        desired_q = self._apply_limits(desired_q)
        current_q = np.array(
            [self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)],
            dtype=float,
        )
        if self.phase_index_ < self.arm_enable_phase_:
            for idx in self.arm_indices_:
                desired_q[idx] = current_q[idx]
        desired_q = self._rate_limit(desired_q, current_q)

        # Gain ramp at start to reduce snap.
        gain_alpha = float(np.clip(self.time_ / self.ramp_time_, 0.0, 1.0))
        kp_scale = self.kp_scale_ * gain_alpha
        kd_scale = self.kd_scale_ * gain_alpha

        self.low_cmd.mode_pr = Mode.PR
        self.low_cmd.mode_machine = self.mode_machine_
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].q = float(desired_q[i])
            self.low_cmd.motor_cmd[i].dq = 0.0
            if self.phase_index_ < self.arm_enable_phase_ and i in self.arm_indices_:
                self.low_cmd.motor_cmd[i].kp = 0.0
                self.low_cmd.motor_cmd[i].kd = 0.0
            else:
                self.low_cmd.motor_cmd[i].kp = Kp[i] * kp_scale
                self.low_cmd.motor_cmd[i].kd = Kd[i] * kd_scale

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)


def main():
    parser = argparse.ArgumentParser(description="Wake the robot up using joint-space IK waypoints.")
    parser.add_argument("--iface", default=None, help="Network interface (robot: enp1s0, sim: lo)")
    parser.add_argument("--domain_id", type=int, default=None, help="DDS domain id (robot: 0, sim: 1)")
    args = parser.parse_args()

    if args.iface is not None:
        domain_id = 0 if args.domain_id is None else args.domain_id
        ChannelFactoryInitialize(domain_id, args.iface)
    else:
        domain_id = 1 if args.domain_id is None else args.domain_id
        ChannelFactoryInitialize(domain_id)

    ctl = WakeUpController()
    ctl.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
    ctl.lowcmd_publisher_.Init()

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(ctl.LowStateHandler, 10)

    ctl.start()
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
