# Day 2: Motion Control and Locomotion

---

## Day 2 outline
- High Level Gait
- Using IMU Data
- Lunch break
- Multi Step motion sequence
- Troubleshooting

Beginner focus:
- You will build confidence with short, low-speed walking commands.

---

## High Level Gait
![Basic motor control overview](https://doc-cdn.unitree.com/static/2024/9/19/66d93f622b6a4ce2a962be2dc2c91054_830x986.jpg)

Measurement: define target speed, distance, time, or number of steps before commanding motion.

Beginner notes:
- Start with low speed and short duration (e.g., 0.1–0.2 m/s for 2–3 seconds).
- Always send a stop command after each motion segment.
- No obstacle avoidance is active in low-level velocity control.

**Parametrized walk example**
```python
from unitree_sdk2py.rpc import LocoClient
from unitree_sdk2py.dds import ChannelFactory

ChannelFactory.Instance().Init(0, "enp3s0")

loco = LocoClient()
loco.Init()

# Walk forward at 0.2 m/s for 3 seconds
loco.SetVelocity(0.2, 0.0, 0.0, 3.0)
loco.StopMove()
```

**Video: Basic motor control (Day 2)**
![Basic motor control walkthrough](https://doc-cdn.unitree.com/static/2024/9/19/66d93f622b6a4ce2a962be2dc2c91054_830x986.jpg)
Video file: https://doc-cdn.unitree.com/static/2024/9/23/982715b4258a4acda666792ac8c964f6.mp4

**Parametrized turn example**
```python
from unitree_sdk2py.rpc import LocoClient
from unitree_sdk2py.dds import ChannelFactory

ChannelFactory.Instance().Init(0, "enp3s0")

loco = LocoClient()
loco.Init()

# Turn in place (yaw rate) for 2 seconds
loco.SetVelocity(0.0, 0.0, 0.6, 2.0)
loco.StopMove()
```

---

## Using IMU Data
![Basic motor control overview](https://doc-cdn.unitree.com/static/2024/9/19/66d93f622b6a4ce2a962be2dc2c91054_830x986.jpg)

- Use IMU data to correct drift and improve motion accuracy in parametrized scripts.
- Visualize stability state during gait tests to validate balance.

Beginner notes:
- The IMU tells you if the robot is tilting or drifting.
- If you see large pitch/roll changes, reduce speed and shorten the step duration.

---

## Multi Step motion sequence
![Basic motor control overview](https://doc-cdn.unitree.com/static/2024/9/19/66d93f622b6a4ce2a962be2dc2c91054_830x986.jpg)

```python
from unitree_sdk2py.rpc import LocoClient, ArmActionClient
from unitree_sdk2py.dds import ChannelFactory

ChannelFactory.Instance().Init(0, "enp3s0")

loco = LocoClient()
loco.Init()

arm = ArmActionClient()
arm.Init()

# Walk forward
loco.SetVelocity(0.2, 0.0, 0.0, 3.0)

# Turn 90 degrees
loco.SetVelocity(0.0, 0.0, 0.6, 1.5)

# Walk forward
loco.SetVelocity(0.2, 0.0, 0.0, 3.0)

# Wave arm
arm.DoAction(26)

# Turn 90 degrees
loco.SetVelocity(0.0, 0.0, 0.6, 1.5)

# Walk forward
loco.SetVelocity(0.2, 0.0, 0.0, 3.0)

loco.StopMove()
```

Beginner notes:
- Sequence each action clearly: walk, stop, turn, stop, walk.
- Insert short pauses if the robot looks unstable.
- Keep arm actions short and simple while walking.

---

## Troubleshooting
![Debug mode on controller](https://doc-cdn.unitree.com/static/2024/9/29/236fa93a8fae4eaa8815004f42e87ede_1065x1419.jpg)

**Recovery procedure (balanced stand)**
```python
from unitree_sdk2py.rpc import LocoClient
from unitree_sdk2py.dds import ChannelFactory

ChannelFactory.Instance().Init(0, "enp3s0")

loco = LocoClient()
loco.Init()

# Bring robot to a safe stop and balanced stand
loco.StopMove()
```

**Intentional error handling**
- Try an invalid parameter range and observe errors.
- Correct parameter ranges and retry.
- Confirm debug mode before low-level commands.

Beginner notes:
- If the robot does not move, verify network interface and debug mode.
- If motion is jerky, reduce speed and duration first, then retry.
