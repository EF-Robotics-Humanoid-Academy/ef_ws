# Unitree SDK2 Python Overview

This document summarizes the most important Unitree SDK2 Python imports and the patterns used in this repository.

## 1) Core DDS Initialization
Most scripts start by initializing DDS on the interface connected to the robot.

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

ChannelFactoryInitialize(0, "enp1s0")  # domain_id=0, robot NIC
```

Important:
- Call once before creating clients/subscribers/publishers.
- Correct `iface` is required (`ip link` to inspect NIC names).

## 2) Subscribing to Robot State
For locomotion state, pose, and IMU context:

```python
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

def cb(msg: SportModeState_):
    print(msg.position, msg.imu_state.rpy)

sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
sub.Init(cb, 10)
```

Used in:
- `g1/scripts/basic/g1_hl_gait_measure.py`
- `g1/scripts/dev/ef_client.py`

## 3) Locomotion Control (`LocoClient`)
Main locomotion client for move/stop/gait/FSM operations.

```python
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()

loco.Move(0.2, 0.0, 0.0, continous_move=True)
loco.StopMove()
```

Common calls:
- `Move(vx, vy, vyaw, continous_move=True)`
- `StopMove()`
- `SetGaitType(0|1)` (supported firmware)
- `SetFsmId(...)` / related FSM routines

See:
- `g1/scripts/basic/g1_loco_client_example.py`
- `g1/scripts/basic/g1_hl_motion_sequence.py`
- `g1/scripts/dev/ef_client.py`

## 4) Arm/Low-Level Motion (Publish `LowCmd_`)
For direct arm or full-body low-level command streams:

```python
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
pub.Init()

cmd = unitree_hg_msg_dds__LowCmd_()
# fill cmd.motor_cmd[...] fields
cmd.crc = CRC().Crc(cmd)
pub.Write(cmd)
```

Typical topics:
- `rt/arm_sdk` for arm SDK style control.
- `rt/lowcmd` for lower-level full command streams.

See:
- `g1/scripts/arm_motion/g1_arm7_sdk_dds_example.py`
- `g1/scripts/arm_motion/pbd/pbd_demonstrate.py`
- `g1/scripts/arm_motion/pbd/pbd_reproduce.py`

## 5) Hand Control (`HandCmd_`)
Dex3 hand command topic pattern:

```python
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
```

Topics:
- `rt/dex3/left/cmd`
- `rt/dex3/right/cmd`

See:
- `g1/scripts/basic/test_finger_motion.py`

## 6) Camera Access
Unitree SDK RPC snapshot pattern:

```python
from unitree_sdk2py.go2.video.video_client import VideoClient

vc = VideoClient()
vc.SetTimeout(3.0)
vc.Init()
code, data = vc.GetImageSample()
```

See:
- `g1/scripts/obj_detection/soda_can_detect.py`
- `g1/scripts/sensors/rgbd_cam.py`

For full RGB+depth network streaming, see manual streaming tools:
- `g1/scripts/sensors/manual_streaming/*.py`

## 7) SLAM RPC Client (`slam_operate`)
Repository wrapper for SLAM service API:

```python
# typically run from within g1/scripts/navigation/obstacle_avoidance
from slam_service import SlamOperateClient

client = SlamOperateClient()
client.Init()
resp = client.start_mapping("indoor")
```

See:
- `g1/scripts/navigation/obstacle_avoidance/slam_service.py`
- `g1/scripts/dev/ef_client.py` (high-level wrappers)

## 8) High-Level Wrapper: `ef_client.Robot`
If you do not need raw SDK calls, use:

```python
# typically run from within g1/scripts
from dev.ef_client import Robot

robot = Robot(iface="enp1s0")
robot.walk(0.2, 0.0, 0.0)
robot.set_gait_type(1)
robot.run(0.4, 0.0, 0.0)
robot.stop()
```

Wrapper covers:
- locomotion and gait/FSM helpers,
- sensor caches (pose/IMU/lidar),
- SLAM start/stop and path navigation,
- arm joint convenience APIs.

## 9) Safety Boot Pattern
Many scripts rely on safe startup before locomotion:

```python
from safety.hanger_boot_sequence import hanger_boot_sequence
loco = hanger_boot_sequence(iface="enp1s0")
```

Used broadly in `g1/scripts/basic`, `g1/scripts/arm_motion/pbd`, and `g1/scripts/usecases`.

## 10) Where To Read Next
- Root overview: `README.md`
- Script index: `g1/scripts/README.md`
- `ef_client` details: `g1/scripts/dev/sdk_details_ef_client.md`
- G1 docs index: `g1/docs/index.md`
- RealSense docs: `g1/docs/quick_start.md`, `g1/docs/how_it_works.md`
