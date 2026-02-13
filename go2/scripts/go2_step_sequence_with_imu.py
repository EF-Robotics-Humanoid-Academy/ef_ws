#!/usr/bin/env python3
import sys
import time
import math
import signal

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient


# -------------------------
# Small math helpers
# -------------------------
def lerp(a, b, t):
    return a + (b - a) * t


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def quat_to_rpy_wxyz(qw, qx, qy, qz):
    """
    Convert quaternion (w,x,y,z) to roll/pitch/yaw (rad).
    Convention: intrinsic ZYX (yaw-pitch-roll).
    """
    # roll (x)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# -------------------------
# Joint targets (SDK example)
# -------------------------
POS_LOW = [
    0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
    -0.2, 1.36, -2.65, 0.2, 1.36, -2.65
]
POS_MAX = [
    0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, 0.0, 0.67, -1.3
]
LOWER_RATIO = 0.18
POS_MID = [lerp(POS_MAX[i], POS_LOW[i], LOWER_RATIO) for i in range(12)]

# Control loop
DT = 0.002  # 500 Hz
KP = 40.0
KD = 3.0

# Step tuning (small offsets).
HIP_FWD = 0.20
THIGH_LIFT = -0.20
CALF_LIFT = 0.40

# Stage durations (seconds)
T_STAND_MAX = 2.0
T_LOWER = 2.0
T_LIFT = 0.4
T_SWING = 0.4
T_DOWN = 0.4

# Joint indices for each leg: (hip, thigh, calf)
LEG_INDEX = {
    "FR": (0, 1, 2),
    "FL": (3, 4, 5),
    "RR": (6, 7, 8),
    "RL": (9, 10, 11),
}
LEG_ORDER = ["FR", "FL", "RR", "RL"]

# -------------------------
# IMU stabilization tuning
# -------------------------
# Conservative attitude stabilization gains.
# If correction goes the wrong way, flip sign by negating K_* or swapping +/- in application.
K_ROLL_P = 0.15
K_ROLL_D = 0.03
K_PITCH_P = 0.15
K_PITCH_D = 0.03

# Map "extension command" -> joint deltas (heuristic).
# Increase magnitude only after verifying stability.
EXT_THIGH_GAIN = 0.35
EXT_CALF_GAIN = -0.55

# Hard clamps
MAX_EXT_CMD = 0.20         # max stabilization "extension command"
MAX_JOINT_DELTA = 0.35     # absolute clamp of added delta per joint (rad)
MAX_ROLL = 0.70            # rad (~40 deg) failsafe threshold
MAX_PITCH = 0.70           # rad (~40 deg) failsafe threshold

# If tilt exceeds threshold, you can either:
# (A) hold POS_MID, or (B) stand down and stop sending low-level commands.
FAILSAFE_MODE = "HOLD"  # "HOLD" or "STOP"


class StepSequence:
    def __init__(self):
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None

        self.crc = CRC()
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)

        self.thread = None
        self.running = False

        self.stage = "stand_max"
        self.stage_t = 0.0
        self.leg_idx = 0
        self.done = False

        # IMU derived state
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0
        self.has_imu = False
        self.failsafe_triggered = False

    def Init(self):
        self.pub.Init()
        self.sub.Init(self.LowStateHandler, 10)
        self._init_lowcmd()

    def Start(self):
        self.running = True
        self.thread = RecurrentThread(interval=DT, target=self._loop, name="step_sequence")
        self.thread.Start()

    def Stop(self):
        self.running = False
        try:
            if self.thread:
                self.thread.Stop()
        except Exception:
            pass

    # -------- IMU/state callback --------
    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        # Try common field names across bindings
        imu = None
        if hasattr(msg, "imu_state"):
            imu = msg.imu_state
        elif hasattr(msg, "imu"):
            imu = msg.imu

        if imu is None:
            self.has_imu = False
            return

        q = getattr(imu, "quaternion", None)
        g = getattr(imu, "gyroscope", None)

        if q is None or g is None:
            self.has_imu = False
            return

        # Most Unitree examples use quaternion order [w, x, y, z]
        qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        self.roll, self.pitch, self.yaw = quat_to_rpy_wxyz(qw, qx, qy, qz)

        self.gx, self.gy, self.gz = float(g[0]), float(g[1]), float(g[2])
        self.has_imu = True

    # -------- LowCmd init --------
    def _init_lowcmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0

        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # position/servo mode
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0

    # -------- Write desired joint positions --------
    def _write_q(self, q):
        for i in range(12):
            self.low_cmd.motor_cmd[i].q = float(q[i])
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = KP
            self.low_cmd.motor_cmd[i].kd = KD
            self.low_cmd.motor_cmd[i].tau = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    # -------- Pose helpers --------
    def _interp_pose(self, src, dst, t):
        return [lerp(src[i], dst[i], t) for i in range(12)]

    def _leg_target(self, base, leg_name, hip_delta, thigh_delta, calf_delta):
        q = list(base)
        hip, thigh, calf = LEG_INDEX[leg_name]
        q[hip] = base[hip] + hip_delta
        q[thigh] = base[thigh] + thigh_delta
        q[calf] = base[calf] + calf_delta
        return q

    # -------- IMU stabilization overlay --------
    def _apply_imu_stabilization(self, q_in):
        """
        Add a small attitude-stabilization correction on top of the current joint targets.
        Uses roll/pitch (from IMU quaternion) and gx/gy (from gyro) as PD feedback.

        Strategy:
          - pitch error -> front vs rear leg "extension"
          - roll error  -> left vs right leg "extension"
        """
        if not self.has_imu:
            return list(q_in)

        roll = self.roll
        pitch = self.pitch

        # Failsafe on extreme tilt
        if abs(roll) > MAX_ROLL or abs(pitch) > MAX_PITCH:
            self.failsafe_triggered = True
            if FAILSAFE_MODE.upper() == "STOP":
                return list(POS_MID)  # last command while main thread stops
            return list(POS_MID)      # HOLD

        # PD controls (gyro: x ~ roll rate, y ~ pitch rate)
        u_roll = -(K_ROLL_P * roll + K_ROLL_D * self.gx)
        u_pitch = -(K_PITCH_P * pitch + K_PITCH_D * self.gy)

        u_roll = clamp(u_roll, -MAX_EXT_CMD, MAX_EXT_CMD)
        u_pitch = clamp(u_pitch, -MAX_EXT_CMD, MAX_EXT_CMD)

        q = list(q_in)

        front_legs = ("FR", "FL")
        rear_legs = ("RR", "RL")
        left_legs = ("FL", "RL")
        right_legs = ("FR", "RR")

        def apply_leg_ext(leg, ext):
            hip, thigh, calf = LEG_INDEX[leg]
            d_thigh = EXT_THIGH_GAIN * ext
            d_calf = EXT_CALF_GAIN * ext

            d_thigh = clamp(d_thigh, -MAX_JOINT_DELTA, MAX_JOINT_DELTA)
            d_calf = clamp(d_calf, -MAX_JOINT_DELTA, MAX_JOINT_DELTA)

            q[thigh] += d_thigh
            q[calf] += d_calf

        # Pitch: front +u_pitch, rear -u_pitch
        for leg in front_legs:
            apply_leg_ext(leg, +u_pitch)
        for leg in rear_legs:
            apply_leg_ext(leg, -u_pitch)

        # Roll: left +u_roll, right -u_roll
        for leg in left_legs:
            apply_leg_ext(leg, +u_roll)
        for leg in right_legs:
            apply_leg_ext(leg, -u_roll)

        return q

    # -------- Main 500 Hz loop --------
    def _loop(self):
        if not self.running:
            return

        if self.low_state is None:
            return

        self.stage_t += DT

        # Optional: if failsafe triggered and STOP is selected, stop loop
        if self.failsafe_triggered and FAILSAFE_MODE.upper() == "STOP":
            # Send one more safe pose
            self._write_q(POS_MID)
            self.done = True
            self.Stop()
            return

        if self.stage == "stand_max":
            t = min(self.stage_t / T_STAND_MAX, 1.0)
            q = self._interp_pose(POS_LOW, POS_MAX, t)
            q = self._apply_imu_stabilization(q)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "lower"
                self.stage_t = 0.0

        elif self.stage == "lower":
            t = min(self.stage_t / T_LOWER, 1.0)
            q = self._interp_pose(POS_MAX, POS_MID, t)
            q = self._apply_imu_stabilization(q)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "lift"
                self.stage_t = 0.0

        elif self.stage == "lift":
            leg = LEG_ORDER[self.leg_idx]
            t = min(self.stage_t / T_LIFT, 1.0)
            q_lift = self._leg_target(POS_MID, leg, 0.0, THIGH_LIFT, CALF_LIFT)
            q = self._interp_pose(POS_MID, q_lift, t)
            q = self._apply_imu_stabilization(q)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "swing"
                self.stage_t = 0.0

        elif self.stage == "swing":
            leg = LEG_ORDER[self.leg_idx]
            t = min(self.stage_t / T_SWING, 1.0)
            q_lift = self._leg_target(POS_MID, leg, 0.0, THIGH_LIFT, CALF_LIFT)
            q_fwd = self._leg_target(POS_MID, leg, HIP_FWD, THIGH_LIFT, CALF_LIFT)
            q = self._interp_pose(q_lift, q_fwd, t)
            q = self._apply_imu_stabilization(q)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "down"
                self.stage_t = 0.0

        elif self.stage == "down":
            leg = LEG_ORDER[self.leg_idx]
            t = min(self.stage_t / T_DOWN, 1.0)
            q_fwd = self._leg_target(POS_MID, leg, HIP_FWD, THIGH_LIFT, CALF_LIFT)
            q = self._interp_pose(q_fwd, POS_MID, t)
            q = self._apply_imu_stabilization(q)
            self._write_q(q)
            if t >= 1.0:
                self.leg_idx = (self.leg_idx + 1) % len(LEG_ORDER)
                self.stage = "lift"
                self.stage_t = 0.0


def main():
    print("WARNING: Low-level control. Ensure clear space + stable ground. Keep E-stop ready.")
    input("Press Enter to continue...")

    iface = sys.argv[1] if len(sys.argv) > 1 else "enp2s0"
    if len(sys.argv) <= 1:
        print("No interface provided; using default: enp2s0")
    print(f"Using interface: {iface}")

    ChannelFactoryInitialize(0, iface)

    # Release any active high-level mode before low-level control.
    sport = SportClient()
    sport.SetTimeout(5.0)
    sport.Init()

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    code, result = msc.CheckMode()
    while result and result.get("name"):
        print(f"Releasing active mode: {result.get('name')}")
        sport.StandDown()
        msc.ReleaseMode()
        time.sleep(1.0)
        code, result = msc.CheckMode()

    seq = StepSequence()
    seq.Init()
    seq.Start()

    # Ctrl-C handler: stop thread + try to stand down via high-level (best effort)
    def _sigint(_signum, _frame):
        print("\nStopping...")
        try:
            seq.Stop()
        except Exception:
            pass
        try:
            sport.StandDown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    last_print = time.time()
    while True:
        if seq.done:
            print("Done.")
            sys.exit(0)

        # Print IMU status occasionally
        now = time.time()
        if now - last_print > 1.0:
            last_print = now
            if seq.has_imu:
                print(f"IMU r/p/y (deg): "
                      f"{seq.roll*180/math.pi:+6.1f}, {seq.pitch*180/math.pi:+6.1f}, {seq.yaw*180/math.pi:+6.1f} | "
                      f"gyro x/y (rad/s): {seq.gx:+.3f}, {seq.gy:+.3f}")
            else:
                print("IMU not available in LowState (no stabilization applied).")

        time.sleep(0.1)


if __name__ == "__main__":
    main()
