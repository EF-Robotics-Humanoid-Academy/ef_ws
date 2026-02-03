# Day 2: Motion Control and Locomotion

---

## Day 2 outline
- High Level Gait
- Using IMU Data
- Lunch break
- Multi Step motion sequence
- Troubleshooting

---

## High Level Gait
![Placeholder](./placeholder.jpg)

Measurement: define target speed, distance, time, or number of steps before commanding motion.

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
![Placeholder](./placeholder.jpg)

- Use IMU data to correct drift and improve motion accuracy in parametrized scripts.
- Visualize stability state during gait tests to validate balance.

---

## Multi Step motion sequence
![Placeholder](./placeholder.jpg)

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

---

## Troubleshooting
![Placeholder](./placeholder.jpg)

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
