import argparse
import os
import sys
import time

from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

import unitree_legged_const as go2


TARGETS = {
    # Height 0: lower stance
    0: [
        0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
        0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    ],
    # Height 1: nominal stand
    # Matches sim/test_unitree_sdk2.py stand_up_joint_pos (more stable in MuJoCo).
    1: [
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
    ],
    # Height 2: taller/extended stance
    2: [
        -0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
        -0.5, 1.36, -2.65, 0.5, 1.36, -2.65,
    ],
}


class StandHeight:
    def __init__(self, height_idx: int, dt: float = 0.002, duration_s: float = 2.0):
        self.height_idx = height_idx
        self.dt = dt
        self.duration_steps = max(1, int(duration_s / dt))

        # Tuned for MuJoCo stability; closer to sim/test_unitree_sdk2.py.
        self.Kp = 50.0
        self.Kd = 3.5

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None

        self.target = TARGETS[height_idx]
        self.start_pos = [0.0] * 12
        self.progress = 0.0
        self.first_run = True

        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.lowcmd_thread = None

        self.crc = CRC()
        self.sim_mode = False

    def Init(self):
        self._init_low_cmd()

        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._low_state_handler, 10)

        start = time.time()
        while self.low_state is None and (time.time() - start) < 2.0:
            time.sleep(0.01)
        if self.low_state is None:
            raise RuntimeError("No low_state received (rt/lowstate). Check DDS config/interface.")

        if not self.sim_mode:
            self.sc = SportClient()
            self.sc.SetTimeout(5.0)
            self.sc.Init()

            self.msc = MotionSwitcherClient()
            self.msc.SetTimeout(5.0)
            self.msc.Init()

            status, result = self.msc.CheckMode()
            while result and result.get("name"):
                self.sc.StandDown()
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)

            if result is None:
                print("WARNING: MotionSwitcher CheckMode failed; continuing without mode release.")
        else:
            print("Sim mode: skipping SportClient/MotionSwitcher.")

    def Start(self):
        self.lowcmd_thread = RecurrentThread(
            interval=self.dt, target=self._write_low_cmd, name="go2-stand-height"
        )
        self.lowcmd_thread.Start()

    def _init_low_cmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # PMSM
            self.low_cmd.motor_cmd[i].q = go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def _low_state_handler(self, msg: LowState_):
        self.low_state = msg

    def _write_low_cmd(self):
        if self.low_state is None:
            return

        if self.first_run:
            for i in range(12):
                self.start_pos[i] = self.low_state.motor_state[i].q
            self.first_run = False

        if self.progress < 1.0:
            self.progress = min(1.0, self.progress + 1.0 / self.duration_steps)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (
                    (1 - self.progress) * self.start_pos[i]
                    + self.progress * self.target[i]
                )
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0
        else:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self.target[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Stand Go2 at a selected height.")
    parser.add_argument(
        "--height",
        type=int,
        default=1,
        choices=[0, 1, 2],
        help="Height index: 0=low, 1=normal, 2=tall",
    )
    parser.add_argument(
        "--iface",
        type=str,
        default="lo",
        help="Network interface for CycloneDDS (default: lo for Mujoco).",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Sim mode: skip SportClient/MotionSwitcher (auto-enabled for iface=lo).",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args(sys.argv[1:])
    sim_mode = args.sim or args.iface == "lo"

    print(
        "WARNING: Please ensure there are no obstacles around the robot while running this example."
    )

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

    domain_id = 1 if args.iface == "lo" else 0
    ChannelFactoryInitialize(domain_id, args.iface)

    print(f"Starting stand height={args.height} on iface={args.iface} domain_id={domain_id}")

    controller = StandHeight(args.height)
    controller.sim_mode = sim_mode
    controller.Init()
    controller.Start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting.")
