import argparse
import math
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
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

import unitree_legged_const as go2


STAND_UP_TARGET = [
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
]


def quat_to_euler(q):
    qw, qx, qy, qz = q
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class StandController:
    def __init__(self, dt=0.002, duration_s=2.0):
        self.dt = dt
        self.duration_steps = max(1, int(duration_s / dt))
        self.Kp = 50.0
        self.Kd = 3.5

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.start_pos = [0.0] * 12
        self.progress = 0.0
        self.first_run = True
        self.crc = CRC()

        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.lowcmd_thread = None

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

    def Start(self):
        self.lowcmd_thread = RecurrentThread(
            interval=self.dt, target=self._write_low_cmd, name="go2-stand-seq"
        )
        self.lowcmd_thread.Start()

    def Stop(self):
        if self.lowcmd_thread is not None:
            # RecurrentThread exposes Wait() to signal quit and join.
            self.lowcmd_thread.Wait()

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
                    + self.progress * STAND_UP_TARGET[i]
                )
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0
        else:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = STAND_UP_TARGET[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


class StateCache:
    def __init__(self):
        self.low_state = None
        self.sport_state = None

    def low_state_cb(self, msg: LowState_):
        self.low_state = msg

    def sport_state_cb(self, msg: SportModeState_):
        self.sport_state = msg

    def imu_rpy(self):
        if self.low_state is None:
            return None
        q = self.low_state.imu_state.quaternion
        return quat_to_euler(q)

    def position(self):
        if self.sport_state is None:
            return None
        return list(self.sport_state.position)


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Execute a motion sequence with IMU checks.")
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


def wait_for_state(cache: StateCache, timeout_s: float = 2.0):
    start = time.time()
    while (cache.low_state is None or cache.sport_state is None) and (time.time() - start) < timeout_s:
        time.sleep(0.01)
    if cache.low_state is None:
        raise RuntimeError("No low_state received (rt/lowstate).")
    if cache.sport_state is None:
        raise RuntimeError("No sportmodestate received (rt/sportmodestate).")


def main(argv):
    args = parse_args(argv)
    sim_mode = args.sim or args.iface == "lo"

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")

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
    print(f"Starting move sequence on iface={args.iface} domain_id={domain_id}")

    cache = StateCache()
    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init(cache.low_state_cb, 10)
    sportstate_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sportstate_sub.Init(cache.sport_state_cb, 10)

    wait_for_state(cache)

    stand = StandController()
    stand.Init()

    sc = SportClient()
    sc.SetTimeout(2.0)
    sc.Init()
    # Wait for sport RPC server to be discoverable.
    for _ in range(10):
        try:
            code, version = sc.GetServerApiVersion()
            if code == 0:
                print(f"Sport server API version: {version}")
                break
        except Exception:
            pass
        time.sleep(0.2)

    msc = None
    if not sim_mode:
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()

        status, result = msc.CheckMode()
        while result and result.get("name"):
            sc.StandDown()
            msc.ReleaseMode()
            status, result = msc.CheckMode()
            time.sleep(1)
    else:
        print("Sim mode: MotionSwitcher disabled. SportClient enabled for sim bridge.")

    sequence = [
        {"type": "stand_up", "duration_s": 2.0, "roll_pitch_tol": 0.25},
        {"type": "move", "vx": 1.0, "vy": 0.0, "vyaw": 0.0, "duration_s": 1.0},
        {"type": "turn", "yaw_deg": 90.0, "vyaw": math.radians(90.0), "duration_s": 1.0},
        {"type": "move", "vx": 1.0, "vy": 0.0, "vyaw": 0.0, "duration_s": 1.0},
    ]

    for idx, step in enumerate(sequence, start=1):
        print(f"Step {idx}: {step}")
        if step["type"] == "stand_up":
            stand.Start()
            time.sleep(step["duration_s"])
            rpy = cache.imu_rpy()
            if rpy is None:
                print("IMU not available for stand check.")
            else:
                roll, pitch, _ = rpy
                if abs(roll) > step["roll_pitch_tol"] or abs(pitch) > step["roll_pitch_tol"]:
                    print(f"Stand check failed: roll={roll:.3f} pitch={pitch:.3f}")
                else:
                    print(f"Stand check ok: roll={roll:.3f} pitch={pitch:.3f}")
        elif step["type"] == "move":
            start_pos = cache.position()
            sc.Move(step["vx"], step["vy"], step["vyaw"])
            time.sleep(step["duration_s"])
            sc.StopMove()
            end_pos = cache.position()
            if start_pos is not None and end_pos is not None:
                dx = end_pos[0] - start_pos[0]
                dy = end_pos[1] - start_pos[1]
                dist = math.hypot(dx, dy)
                print(f"Move distance: {dist:.3f} m")
        elif step["type"] == "turn":
            rpy0 = cache.imu_rpy()
            sc.Move(0.0, 0.0, step["vyaw"])
            time.sleep(step["duration_s"])
            sc.StopMove()
            rpy1 = cache.imu_rpy()
            if rpy0 and rpy1:
                yaw0 = rpy0[2]
                yaw1 = rpy1[2]
                dyaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
                print(f"Turned yaw: {math.degrees(dyaw):.1f} deg")
        else:
            print(f"Unknown step type: {step['type']}")

    stand.Stop()
    print("Sequence complete.")


if __name__ == "__main__":
    main(sys.argv[1:])
