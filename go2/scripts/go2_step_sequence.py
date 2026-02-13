import sys
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient


def lerp(a, b, t):
    return a + (b - a) * t


# Joint targets from SDK example (low and max height).
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

DT = 0.002
KP = 40.0
KD = 3.0

# Step tuning (small offsets).
HIP_FWD = 0.20
THIGH_LIFT = -0.20
CALF_LIFT = 0.40

T_STAND_MAX = 2.0
T_LOWER = 2.0
T_LIFT = 0.4
T_SWING = 0.4
T_DOWN = 0.4

LEG_INDEX = {
    "FR": (0, 1, 2),
    "FL": (3, 4, 5),
    "RR": (6, 7, 8),
    "RL": (9, 10, 11),
}
LEG_ORDER = ["FR", "FL", "RR", "RL"]


class StepSequence:
    def __init__(self):
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.thread = None

        self.stage = "stand_max"
        self.stage_t = 0.0
        self.leg_idx = 0
        self.done = False

    def Init(self):
        self.pub.Init()
        self.sub.Init(self.LowStateHandler, 10)
        self._init_lowcmd()

    def Start(self):
        self.thread = RecurrentThread(interval=DT, target=self._loop, name="step_sequence")
        self.thread.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

    def _init_lowcmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = 0.0
            self.low_cmd.motor_cmd[i].kd = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0

    def _write_q(self, q):
        for i in range(12):
            self.low_cmd.motor_cmd[i].q = q[i]
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].kp = KP
            self.low_cmd.motor_cmd[i].kd = KD
            self.low_cmd.motor_cmd[i].tau = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def _interp_pose(self, src, dst, t):
        return [lerp(src[i], dst[i], t) for i in range(12)]

    def _leg_target(self, base, leg_name, hip_delta, thigh_delta, calf_delta):
        q = list(base)
        hip, thigh, calf = LEG_INDEX[leg_name]
        q[hip] = base[hip] + hip_delta
        q[thigh] = base[thigh] + thigh_delta
        q[calf] = base[calf] + calf_delta
        return q

    def _loop(self):
        if self.low_state is None:
            return

        self.stage_t += DT

        if self.stage == "stand_max":
            t = min(self.stage_t / T_STAND_MAX, 1.0)
            q = self._interp_pose(POS_LOW, POS_MAX, t)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "lower"
                self.stage_t = 0.0

        elif self.stage == "lower":
            t = min(self.stage_t / T_LOWER, 1.0)
            q = self._interp_pose(POS_MAX, POS_MID, t)
            self._write_q(q)
            if t >= 1.0:
                self.stage = "lift"
                self.stage_t = 0.0

        elif self.stage == "lift":
            leg = LEG_ORDER[self.leg_idx]
            t = min(self.stage_t / T_LIFT, 1.0)
            q_lift = self._leg_target(POS_MID, leg, 0.0, THIGH_LIFT, CALF_LIFT)
            q = self._interp_pose(POS_MID, q_lift, t)
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
            self._write_q(q)
            if t >= 1.0:
                self.stage = "down"
                self.stage_t = 0.0

        elif self.stage == "down":
            leg = LEG_ORDER[self.leg_idx]
            t = min(self.stage_t / T_DOWN, 1.0)
            q_fwd = self._leg_target(POS_MID, leg, HIP_FWD, THIGH_LIFT, CALF_LIFT)
            q = self._interp_pose(q_fwd, POS_MID, t)
            self._write_q(q)
            if t >= 1.0:
                self.leg_idx += 1
                if self.leg_idx >= len(LEG_ORDER):
                    self.leg_idx = 0
                self.stage = "lift"
                self.stage_t = 0.0


def main():
    print("WARNING: Ensure the robot has clear space and is on stable ground.")
    input("Press Enter to continue...")

    iface = sys.argv[1] if len(sys.argv) > 1 else "enp2s0"
    if len(sys.argv) <= 1:
        print("No interface provided; using default: enp2s0")

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
        sport.StandDown()
        msc.ReleaseMode()
        time.sleep(1.0)
        code, result = msc.CheckMode()

    seq = StepSequence()
    seq.Init()
    seq.Start()

    while True:
        if seq.done:
            print("Done.")
            sys.exit(0)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
