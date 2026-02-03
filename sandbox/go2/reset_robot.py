import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Tunable timings (seconds)
STAND_DOWN_WAIT = 4.0
DAMP_WAIT = 1.5
BALANCE_WAIT = 2.0
LOWLEVEL_STOP_WAIT = 3.0


def try_stop_lowlevel(duration_sec: float):
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x00
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
    crc = CRC()
    start = time.time()
    while time.time() - start < duration_sec:
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        time.sleep(0.02)


def release_active_mode(motion_switcher: MotionSwitcherClient, sport: SportClient):
    code, result = motion_switcher.CheckMode()
    if code != 0 or result is None:
        return
    while result.get("name"):
        sport.StandDown()
        motion_switcher.ReleaseMode()
        time.sleep(1.0)
        code, result = motion_switcher.CheckMode()
        if code != 0 or result is None:
            break


def main():
    print("WARNING: Ensure the robot has clear space and is on stable ground.")
    input("Press Enter to continue...")
    parser = argparse.ArgumentParser()
    parser.add_argument("iface", nargs="?", default="enp2s0")
    parser.add_argument("--force-lowlevel", action="store_true")
    parser.add_argument("--lowlevel-seconds", type=float, default=LOWLEVEL_STOP_WAIT)
    args = parser.parse_args()
    if args.iface == "enp2s0":
        print("Using default interface: enp2s0")

    ChannelFactoryInitialize(0, args.iface)

    print("Attempting to neutralize low-level control...")
    try_stop_lowlevel(args.lowlevel_seconds if args.force_lowlevel else LOWLEVEL_STOP_WAIT)
    print("Low-level neutralization attempt done.")

    sport = SportClient()
    sport.SetTimeout(10.0)
    sport.Init()

    motion_switcher = MotionSwitcherClient()
    motion_switcher.SetTimeout(5.0)
    motion_switcher.Init()

    release_active_mode(motion_switcher, sport)

    print("Stand down (slow settle)...")
    ret = sport.StandDown()
    print(f"StandDown ret: {ret}")
    time.sleep(STAND_DOWN_WAIT)

    print("Damping mode...")
    ret = sport.Damp()
    print(f"Damp ret: {ret}")
    if ret != 0:
        print("Damp failed; trying StopMove + RecoveryStand...")
        sport.StopMove()
        time.sleep(1.0)
        sport.RecoveryStand()
    time.sleep(DAMP_WAIT)

    print("Balanced stand...")
    ret = sport.BalanceStand()
    print(f"BalanceStand ret: {ret}")
    time.sleep(BALANCE_WAIT)

    print("Done.")


if __name__ == "__main__":
    main()
