import time
import os
import numpy as np
from unitree_sdk2py.core import channel as channel_module
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


def HighStateHandler(msg: SportModeState_):
    print("Position: ", msg.position)
    #print("Velocity: ", msg.velocity)


def LowStateHandler(msg: LowState_):
    print("IMU state: ", msg.imu_state)
    # print("motor[0] state: ", msg.motor_state[0])

stand_up_joint_pos = np.array(
    [
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
    ],
    dtype=float,
)

stand_down_joint_pos = np.array(
    [
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
    ],
    dtype=float,
)

dt = 0.002
running_time = 0.0


if __name__ == "__main__":
    channel_module.ChannelConfigHasInterface = """<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>"""
    os.environ.setdefault(
        "CYCLONEDDS_URI",
        "<CycloneDDS><Domain><Tracing><Category>none</Category></Tracing></Domain></CycloneDDS>",
    )
    ChannelFactoryInitialize(1, "lo")
    hight_state_suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)

    hight_state_suber.Init(HighStateHandler, 10)
    low_state_suber.Init(LowStateHandler, 10)

    low_cmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_puber.Init()
    crc = CRC()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0]=0xFE
    cmd.head[1]=0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q= 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
        
    while True:
        step_start = time.perf_counter()
        running_time += dt

        # Ramp into balanced stand and hold.
        phase = np.tanh(min(running_time, 2.0) / 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                1 - phase
            ) * stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)

        #Publish message
        low_cmd_puber.Write(cmd)
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
