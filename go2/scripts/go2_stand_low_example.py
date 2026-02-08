import time
import argparse

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2


class LowStand:
    def __init__(self):
        self.Kp = 60.0
        self.Kd = 5.0
        self.dt = 0.002

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None

        # Stand poses (same as original example).
        self.pos_low = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                        -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self.pos_mid = [0.0, 1.015, -1.975, 0.0, 1.015, -1.975,
                        -0.1, 1.015, -1.975, 0.1, 1.015, -1.975]
        self.pos_high = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                         0.0, 0.67, -1.3, 0.0, 0.67, -1.3]

        self.startPos = [0.0] * 12
        self.stage = 0
        self.stage_progress = 0.0
        self.firstRun = True
        self.done = False

        self.duration_up = 800   # low -> high
        self.duration_hold = 800 # hold
        self.duration_down = 800 # high -> low

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


    def wait_for_low_state(self, timeout_s=5.0):
        start = time.time()
        while self.low_state is None and (time.time() - start) < timeout_s:
            time.sleep(0.05)
        return self.low_state is not None

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        if self.stage == 0:
            # low -> high
            self.stage_progress += 1.0 / self.duration_up
            self.stage_progress = min(self.stage_progress, 1)
            src = self.startPos
            dst = self.pos_high
        elif self.stage == 1:
            # hold
            self.stage_progress += 1.0 / self.duration_hold
            self.stage_progress = min(self.stage_progress, 1)
            src = self.pos_high
            dst = self.pos_high
        elif self.stage == 2:
            # high -> low
            self.stage_progress += 1.0 / self.duration_down
            self.stage_progress = min(self.stage_progress, 1)
            src = self.pos_high
            dst = self.pos_low
        else:
            self.done = True
            return

        for i in range(12):
            self.low_cmd.motor_cmd[i].q = (
                (1 - self.stage_progress) * src[i] + self.stage_progress * dst[i]
            )
            self.low_cmd.motor_cmd[i].dq = 0
            self.low_cmd.motor_cmd[i].kp = self.Kp
            self.low_cmd.motor_cmd[i].kd = self.Kd
            self.low_cmd.motor_cmd[i].tau = 0

        if self.stage_progress >= 1.0:
            self.stage += 1
            self.stage_progress = 0.0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


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

    custom = LowStand()
    custom.Init()

    if not custom.wait_for_low_state(timeout_s=5.0):
        print('No lowstate received. Check domain_id/interface or start the simulator.')
        print('For MuJoCo, try: --sim or --domain_id 1')
        raise SystemExit(1)
    custom.Start()

    while True:
        if custom.done:
            time.sleep(1)
            print('Done!')
            raise SystemExit(0)
        time.sleep(1)
