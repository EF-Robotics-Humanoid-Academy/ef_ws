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
os.environ.setdefault(
    "CYCLONEDDS_URI",
    "<CycloneDDS><Domain><Tracing><Category>none</Category></Tracing></Domain></CycloneDDS>",
)

G1_NUM_MOTOR = 29

class G1JointIndex:
    LeftHipPitch = 0
    LeftKnee = 3
    LeftAnklePitch = 4
    RightHipPitch = 6
    RightKnee = 9
    RightAnklePitch = 10


class HipBowController:
    def __init__(self, args):
        self.args = args
        self.control_dt = 0.002
        self.crc = CRC()
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.mode_machine = 0
        self.have_mode_machine = False
        self.t = 0.0

        self.q_start = None
        self.q_target = None
        self.q_last = None

        # Conservative gains and speed to avoid contact-induced bounce.
        self.kp_hold = 10.0
        self.kd_hold = 2.0
        self.kp_bow = 14.0
        self.kd_bow = 2.8
        self.max_speed = 0.12  # rad/s
        self.gain_ramp_time = 4.0
        self.safe_mode = False

    def lowstate_handler(self, msg: LowState_):
        self.low_state = msg
        if not self.have_mode_machine:
            self.mode_machine = msg.mode_machine
            self.have_mode_machine = True

    def _smoothstep(self, x: float) -> float:
        x = float(np.clip(x, 0.0, 1.0))
        return x * x * (3.0 - 2.0 * x)

    def _build_target(self, q0: np.ndarray) -> np.ndarray:
        q = np.array(q0, dtype=float)
        # Clamp target posture around current pose to avoid large transients.
        hip_t = np.clip(float(self.args.hip_pitch), q0[G1JointIndex.LeftHipPitch] - 0.35, q0[G1JointIndex.LeftHipPitch] + 0.35)
        knee_t = np.clip(float(self.args.knee), q0[G1JointIndex.LeftKnee] - 0.45, q0[G1JointIndex.LeftKnee] + 0.45)
        ankle_t = np.clip(float(self.args.ankle), q0[G1JointIndex.LeftAnklePitch] - 0.35, q0[G1JointIndex.LeftAnklePitch] + 0.35)
        q[G1JointIndex.LeftHipPitch] = float(hip_t)
        q[G1JointIndex.RightHipPitch] = float(hip_t)
        q[G1JointIndex.LeftKnee] = float(knee_t)
        q[G1JointIndex.RightKnee] = float(knee_t)
        q[G1JointIndex.LeftAnklePitch] = float(ankle_t)
        q[G1JointIndex.RightAnklePitch] = float(ankle_t)
        return q

    def _rate_limit(self, q_des: np.ndarray) -> np.ndarray:
        if self.q_last is None:
            self.q_last = q_des.copy()
        max_step = self.max_speed * self.control_dt
        delta = np.clip(q_des - self.q_last, -max_step, max_step)
        self.q_last = self.q_last + delta
        return self.q_last

    def _desired_q(self) -> np.ndarray:
        q_now = np.array([self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=float)
        if self.q_start is None:
            self.q_start = q_now.copy()
            self.q_target = self._build_target(q_now)
            self.q_last = q_now.copy()

        t1 = max(0.2, float(self.args.bow_time))
        t2 = t1 + max(0.0, float(self.args.hold_time))
        t3 = t2 + max(0.2, float(self.args.return_time))

        if self.t <= t1:
            a = self._smoothstep(self.t / t1)
            q_des = (1.0 - a) * self.q_start + a * self.q_target
        elif self.t <= t2:
            q_des = self.q_target
        elif self.t <= t3:
            a = self._smoothstep((self.t - t2) / max(1e-6, float(self.args.return_time)))
            q_des = (1.0 - a) * self.q_target + a * self.q_start
        else:
            q_des = self.q_start

        return self._rate_limit(q_des)

    def step(self):
        if self.low_state is None or not self.have_mode_machine:
            return

        self.t += self.control_dt
        q_now = np.array([self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)], dtype=float)

        # Safety: if torso tilt gets too high, freeze to current posture.
        try:
            roll = float(self.low_state.imu_state.rpy[0])
            pitch = float(self.low_state.imu_state.rpy[1])
            if abs(roll) > float(self.args.max_tilt) or abs(pitch) > float(self.args.max_tilt):
                self.safe_mode = True
        except Exception:
            pass

        if self.safe_mode:
            q_des = q_now
        else:
            q_des = self._desired_q()

        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine

        bow_ids = {
            G1JointIndex.LeftHipPitch,
            G1JointIndex.RightHipPitch,
            G1JointIndex.LeftKnee,
            G1JointIndex.RightKnee,
            G1JointIndex.LeftAnklePitch,
            G1JointIndex.RightAnklePitch,
        }

        gain_alpha = float(np.clip(self.t / self.gain_ramp_time, 0.0, 1.0))
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].q = float(q_des[i])
            self.low_cmd.motor_cmd[i].dq = 0.0
            if i in bow_ids:
                self.low_cmd.motor_cmd[i].kp = self.kp_bow * gain_alpha
                self.low_cmd.motor_cmd[i].kd = self.kd_bow * gain_alpha
            else:
                self.low_cmd.motor_cmd[i].kp = self.kp_hold * gain_alpha
                self.low_cmd.motor_cmd[i].kd = self.kd_hold * gain_alpha

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def run(self):
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.lowstate_handler, 10)

        self.thread = RecurrentThread(interval=self.control_dt, target=self.step, name="hip_bow_pickup")
        self.thread.Start()


def main():
    parser = argparse.ArgumentParser(description="Bow forward using hip joints for pickup posture")
    parser.add_argument("--iface", default="lo", help="DDS interface (sim usually lo)")
    parser.add_argument("--domain_id", type=int, default=1, help="DDS domain id (sim usually 1)")
    parser.add_argument("--hip_pitch", type=float, default=0.30, help="Target hip pitch rad")
    parser.add_argument("--knee", type=float, default=-0.35, help="Target knee rad")
    parser.add_argument("--ankle", type=float, default=0.20, help="Target ankle pitch rad")
    parser.add_argument("--bow_time", type=float, default=6.0, help="Seconds to bow down")
    parser.add_argument("--hold_time", type=float, default=1.5, help="Seconds to hold pickup pose")
    parser.add_argument("--return_time", type=float, default=6.0, help="Seconds to return upright")
    parser.add_argument("--max_tilt", type=float, default=0.35, help="Abort motion if |roll| or |pitch| exceeds this rad")
    args = parser.parse_args()

    ChannelFactoryInitialize(args.domain_id, args.iface)

    ctl = HipBowController(args)
    ctl.run()

    print(
        "hip_bow_pickup running (safe profile). Note: DISABLE_HIP_JOINTS must be False in mujoco/COM_viz/config.py",
        flush=True,
    )
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
