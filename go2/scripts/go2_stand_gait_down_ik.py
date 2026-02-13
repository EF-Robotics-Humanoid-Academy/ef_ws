import time
import argparse
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2


class StandGaitDownIK:
    def __init__(self):
        self.Kp_stand = 40.0
        self.Kd_stand = 3.0
        self.Kp_gait = 25.0
        self.Kd_gait = 1.6
        self.dt = 0.005

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None

        # Stand poses.
        self.pos_low = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                        -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self.pos_high = [0.0, 0.85, -1.65, 0.0, 0.85, -1.65,
                         0.0, 0.85, -1.65, 0.0, 0.85, -1.65]

        # IK parameters (meters). Tune if needed.
        self.hip_offset_x = 0.1934
        self.hip_offset_y = 0.0465
        self.hip_offset_z = 0.0
        self.thigh_len = 0.213
        self.calf_len = 0.213

        # Gait params.
        self.period = 2.8
        self.swing_ratio = 0.22
        self.step_amp = 0.32
        self.stance_amp = 0.28
        self.lift_amp = 0.55

        # IMU stabilization and bias.
        self.k_roll = 0.15
        self.k_pitch = 0.15
        self.k_roll_rate = 0.01
        self.k_pitch_rate = 0.01
        self.support_roll_bias = 0.08
        self.forward_bias = 0.05
        self.diag_knee_bias = 0.10
        self.fall_pitch = 0.8
        self.fall_roll = 0.8

        self.last_imu_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.last_imu_gyro = np.zeros(3)

        # Command smoothing
        self.prev_cmd = None
        self.cmd_smooth_alpha = 0.2
        self.max_step = 0.08
        self.dq_max = 3.0

        self.legs = ["FL", "FR", "RR", "RL"]
        self.phase_offsets = {
            "FR": 0.0,
            "RL": 0.0,
            "FL": 0.5,
            "RR": 0.5,
        }
        self.motor_index = {
            "FR": [go2.LegID["FR_0"], go2.LegID["FR_1"], go2.LegID["FR_2"]],
            "FL": [go2.LegID["FL_0"], go2.LegID["FL_1"], go2.LegID["FL_2"]],
            "RR": [go2.LegID["RR_0"], go2.LegID["RR_1"], go2.LegID["RR_2"]],
            "RL": [go2.LegID["RL_0"], go2.LegID["RL_1"], go2.LegID["RL_2"]],
        }

        # State machine.
        self.state = "stand_up"
        self.state_t = 0.0
        self.stand_up_time = 6.0
        self.gait_time = 3.0
        self.gait_ramp_time = 1.5
        self.hold_before_gait = 0.8
        self.stand_down_time = 6.0
        self.stand_ramp_time = 1.0

        self.startPos = [0.0] * 12
        self.firstRun = True
        self.prev_q = None

        self.lowCmdWriteThreadPtr = None
        self.crc = CRC()

    def Init(self):
        self.InitLowCmd()
        self.lowcmd_publisher = ChannelPublisher('rt/lowcmd', LowCmd_)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.dt, target=self.LowCmdWrite, name='writebasiccmd'
        )
        self.lowCmdWriteThreadPtr.Start()

    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        try:
            self.last_imu_quat = np.array([
                msg.imu_state.quaternion[0],
                msg.imu_state.quaternion[1],
                msg.imu_state.quaternion[2],
                msg.imu_state.quaternion[3],
            ], dtype=np.float64)
            self.last_imu_gyro = np.array([
                msg.imu_state.gyroscope[0],
                msg.imu_state.gyroscope[1],
                msg.imu_state.gyroscope[2],
            ], dtype=np.float64)
        except Exception:
            pass

    def wait_for_low_state(self, timeout_s=5.0):
        start = time.time()
        while self.low_state is None and (time.time() - start) < timeout_s:
            time.sleep(0.05)
        return self.low_state is not None

    def _quat_to_rpy(self, quat):
        q = np.array(quat, dtype=np.float64)
        if np.linalg.norm(q) > 0:
            q = q / np.linalg.norm(q)
        w, x, y, z = q
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * (np.pi / 2.0)
        else:
            pitch = np.arcsin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([roll, pitch, yaw], dtype=np.float64)

    def _phase(self, leg, t):
        return (t / self.period + self.phase_offsets[leg]) % 1.0

    def _swing_profile(self, s):
        lift = np.sin(np.pi * s) * self.lift_amp
        forward = np.sin(2.0 * np.pi * s) * self.step_amp
        return lift, forward

    def _stance_profile(self, s):
        return (0.5 - s) * 2.0 * self.stance_amp

    def _hip_position(self, leg):
        x = self.hip_offset_x if leg in ("FL", "FR") else -self.hip_offset_x
        y = self.hip_offset_y if leg in ("FL", "RL") else -self.hip_offset_y
        z = self.hip_offset_z
        return np.array([x, y, z], dtype=np.float64)

    def _ik_leg(self, leg, foot_body):
        # foot_body: desired foot pos in body frame (meters)
        hip = self._hip_position(leg)
        p = foot_body - hip
        # Abduction angle about x-axis
        q0 = np.arctan2(p[1], -p[2])
        # Rotate into sagittal plane
        cy = np.cos(-q0)
        sy = np.sin(-q0)
        y = p[1] * cy - p[2] * sy
        z = p[1] * sy + p[2] * cy
        x = p[0]
        # Planar IK in x-z
        L1 = self.thigh_len
        L2 = self.calf_len
        d = np.sqrt(x * x + z * z)
        d = max(min(d, L1 + L2 - 1e-6), abs(L1 - L2) + 1e-6)
        cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)
        q2 = np.pi - np.arccos(cos_knee)  # knee flexion
        cos_hip = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
        hip_ang = np.arccos(cos_hip)
        q1 = np.arctan2(-z, x) - hip_ang
        return np.array([q0, q1, q2], dtype=np.float64)

    def _gait_targets(self, t):
        q_targets = {leg: [0.0, 0.0, 0.0] for leg in self.legs}
        base_height = -0.27  # nominal foot height in body frame (z negative)
        base_x = 0.0

        roll, pitch, _ = self._quat_to_rpy(self.last_imu_quat)
        roll_rate = self.last_imu_gyro[0]
        pitch_rate = self.last_imu_gyro[1]
        roll_base = self.k_roll * roll + self.k_roll_rate * roll_rate
        pitch_base = self.k_pitch * pitch + self.k_pitch_rate * pitch_rate

        swing_legs = [leg for leg in self.legs if self._phase(leg, t) < self.swing_ratio]
        swing_leg = swing_legs[0] if len(swing_legs) == 1 else None

        for leg in self.legs:
            phase = self._phase(leg, t)
            swinging = phase < self.swing_ratio

            # Foot target in body frame (x forward, z down).
            if swinging:
                s = phase / self.swing_ratio
                lift, forward = self._swing_profile(s)
                foot_x = base_x + forward
                foot_z = base_height + lift
            else:
                s = (phase - self.swing_ratio) / (1.0 - self.swing_ratio)
                back = self._stance_profile(s)
                foot_x = base_x + back
                foot_z = base_height

            # Simple pitch/roll compensation into foot target.
            foot_z += -0.05 * pitch_base
            if leg in ("FL", "RL"):
                foot_z += -0.05 * roll_base
            else:
                foot_z += 0.05 * roll_base

            # IK to joint targets.
            foot_body = np.array([foot_x, self.hip_offset_y * (1 if leg in ("FL", "RL") else -1), foot_z])
            q = self._ik_leg(leg, foot_body)

            # Diagonal knee support bias when a leg swings.
            if swing_leg is not None and not swinging:
                diag_map = {"FR": "RL", "FL": "RR", "RR": "FL", "RL": "FR"}
                if leg == diag_map.get(swing_leg):
                    q[2] += -self.diag_knee_bias

            q_targets[leg] = q

        return q_targets

    def _apply_cmd_filter(self):
        if self.prev_cmd is None:
            self.prev_cmd = [self.low_cmd.motor_cmd[i].q for i in range(12)]
        for i in range(12):
            q_raw = self.low_cmd.motor_cmd[i].q
            q_prev = self.prev_cmd[i]
            delta = q_raw - q_prev
            if delta > self.max_step:
                q_raw = q_prev + self.max_step
            elif delta < -self.max_step:
                q_raw = q_prev - self.max_step

            q_cmd = (1 - self.cmd_smooth_alpha) * q_prev + self.cmd_smooth_alpha * q_raw
            dq_cmd = (q_cmd - q_prev) / self.dt
            if dq_cmd > self.dq_max:
                dq_cmd = self.dq_max
            elif dq_cmd < -self.dq_max:
                dq_cmd = -self.dq_max

            self.low_cmd.motor_cmd[i].q = q_cmd
            self.low_cmd.motor_cmd[i].dq = dq_cmd
            self.prev_cmd[i] = q_cmd

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        roll, pitch, _ = self._quat_to_rpy(self.last_imu_quat)
        if abs(roll) > self.fall_roll or abs(pitch) > self.fall_pitch:
            self.state = "stand_down"
            self.state_t = 0.0

        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            # initialize filters from current state to avoid startup jump
            self.prev_cmd = [self.low_state.motor_state[i].q for i in range(12)]
            self.prev_q = [self.low_state.motor_state[i].q for i in range(12)]
            self.firstRun = False

        smooth_alpha = 0.15
        kp_scale = min(self.state_t / self.stand_ramp_time, 1.0)

        if self.state == "stand_up":
            alpha = min(self.state_t / self.stand_up_time, 1.0)
            src = self.startPos
            dst = self.pos_high
            for i in range(12):
                q_des = (1 - alpha) * src[i] + alpha * dst[i]
                if self.prev_q is None:
                    self.prev_q = [q_des] * 12
                q_smooth = (1 - smooth_alpha) * self.prev_q[i] + smooth_alpha * q_des
                self.prev_q[i] = q_smooth
                self.low_cmd.motor_cmd[i].q = q_smooth
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp_stand * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.Kd_stand * kp_scale
                self.low_cmd.motor_cmd[i].tau = 0
            if alpha >= 1.0:
                self.state = "hold_high"
                self.state_t = 0.0
                self.prev_cmd = None
        elif self.state == "hold_high":
            # hold high to settle before gait
            for i in range(12):
                q_des = self.pos_high[i]
                if self.prev_q is None:
                    self.prev_q = [q_des] * 12
                q_smooth = (1 - smooth_alpha) * self.prev_q[i] + smooth_alpha * q_des
                self.prev_q[i] = q_smooth
                self.low_cmd.motor_cmd[i].q = q_smooth
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp_stand * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.Kd_stand * kp_scale
                self.low_cmd.motor_cmd[i].tau = 0
            if self.state_t >= self.hold_before_gait:
                self.state = "gait"
                self.state_t = 0.0
                self.prev_cmd = None
        elif self.state == "gait":
            gait_scale = min(self.state_t / self.gait_ramp_time, 1.0)
            q_targets = self._gait_targets(self.state_t)
            for leg in q_targets:
                base_idx = 0 if leg=="FR" else 3 if leg=="FL" else 6 if leg=="RR" else 9
                base = self.pos_high[base_idx:base_idx+3]
                q_targets[leg] = [base[i] + gait_scale * (q_targets[leg][i] - base[i]) for i in range(3)]
            for leg, idxs in self.motor_index.items():
                for j, midx in enumerate(idxs):
                    self.low_cmd.motor_cmd[midx].q = float(q_targets[leg][j])
                    self.low_cmd.motor_cmd[midx].dq = 0
                    self.low_cmd.motor_cmd[midx].kp = self.Kp_gait
                    self.low_cmd.motor_cmd[midx].kd = self.Kd_gait
                    self.low_cmd.motor_cmd[midx].tau = 0
            if self.state_t >= self.gait_time:
                self.state = "stand_down"
                self.state_t = 0.0
        elif self.state == "stand_down":
            alpha = min(self.state_t / self.stand_down_time, 1.0)
            src = self.pos_high
            dst = self.pos_low
            for i in range(12):
                q_des = (1 - alpha) * src[i] + alpha * dst[i]
                if self.prev_q is None:
                    self.prev_q = [q_des] * 12
                q_smooth = (1 - smooth_alpha) * self.prev_q[i] + smooth_alpha * q_des
                self.prev_q[i] = q_smooth
                self.low_cmd.motor_cmd[i].q = q_smooth
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp_stand * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.Kd_stand * kp_scale
                self.low_cmd.motor_cmd[i].tau = 0
            if alpha >= 1.0:
                self.state = "hold_low"
                self.state_t = 0.0
        elif self.state == "hold_low":
            for i in range(12):
                q_des = self.pos_low[i]
                if self.prev_q is None:
                    self.prev_q = [q_des] * 12
                q_smooth = (1 - smooth_alpha) * self.prev_q[i] + smooth_alpha * q_des
                self.prev_q[i] = q_smooth
                self.low_cmd.motor_cmd[i].q = q_smooth
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp_stand * kp_scale
                self.low_cmd.motor_cmd[i].kd = self.Kd_stand * kp_scale
                self.low_cmd.motor_cmd[i].tau = 0
        else:
            return

        self._apply_cmd_filter()
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

        self.state_t += self.dt


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('iface', nargs='?', default='lo')
    parser.add_argument('--domain_id', type=int, default=None)
    parser.add_argument('--sim', action='store_true', help='Use MuJoCo defaults (domain_id=1)')
    args = parser.parse_args()

    if args.domain_id is None:
        args.domain_id = 1 if args.sim else 0

    print('WARNING: Please ensure there are no obstacles around the robot while running this example.')
    input('Press Enter to continue...')

    ChannelFactoryInitialize(args.domain_id, args.iface)

    custom = StandGaitDownIK()
    custom.Init()
    if not custom.wait_for_low_state(timeout_s=5.0):
        print('No lowstate received. Check domain_id/interface or start the simulator.')
        print('For MuJoCo, try: --sim or --domain_id 1')
        raise SystemExit(1)
    custom.Start()

    while True:
        if custom.state == 'hold_low':
            time.sleep(1)
            print('Done!')
            raise SystemExit(0)
        time.sleep(1)
