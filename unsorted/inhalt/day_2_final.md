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

## Schedule (Tag 2 – Di 17.02.2026)
- 08.00 Crew
- 08:30 Einfinden der Teilnehmer
- 09:00 Schulungsblock 1
- 10:30 Kaffee- und Teepause
- 10:45 Schulungsblock 2
- 12:00 Mittagessen
- 13:00 Schulungsblock 3
- 14:30 Kaffee- und Teepause mit Snacks
- 15:00 Schulungsblock 4
- 17:00 Ende Tag 2
- 17:00 – 17:30 Podcast / Testimonials
- 17:30 – 17:45 Puffer
- 17:45 – 18:15 Tages‑Recap
- 18:15 – 18:30 Tagesabschluss

---

## High Level Gait
![Basic motor control overview](https://doc-cdn.unitree.com/static/2024/9/19/66d93f622b6a4ce2a962be2dc2c91054_830x986.jpg)

Measurement: define target speed, distance, time, or number of steps before commanding motion.

Beginner notes:
- Start with low speed and short duration (e.g., 0.1–0.2 m/s for 2–3 seconds).
- Always send a stop command after each motion segment.
- No obstacle avoidance is active in low-level velocity control.

### Implementation details (from repo scripts)
- `g1/scripts/basic/g1_hl_gait_measure.py` subscribes to `rt/sportmodestate`, samples position and `foot_force`, and computes total path length plus step count. Contact steps are detected by thresholding each foot force signal and counting rising edges.
- The script supports `--no-command` for measurement-only runs, `--force-threshold` to tune contact detection, and `--csv` to log time, pose, gait type, and foot force for analysis.

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
![Basic motor control walkthrough](./placeholder.jpg)
Video file:
[https://doc-cdn.unitree.com/static/2024/9/23/982715b4258a4acda666792ac8c964f6.mp4](https://doc-cdn.unitree.com/static/2024/9/23/982715b4258a4acda666792ac8c964f6.mp4)

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
![Basic motor control overview](./placeholder.jpg)

- Use IMU data to correct drift and improve motion accuracy in parametrized scripts.
- Visualize stability state during gait tests to validate balance.

Beginner notes:
- The IMU tells you if the robot is tilting or drifting.
- If you see large pitch/roll changes, reduce speed and shorten the step duration.

### Implementation details (from repo scripts)
- `g1/scripts/sensors/g1_stability_view.py` subscribes to `rt/lowstate` for IMU `rpy` and to `rt/odom` for position, then plots roll/pitch/yaw, tilt magnitude, and a stability score in real time.
- The stability score is computed as `1.0 - (tilt_deg / max_tilt_deg)` and can be tuned with `--max-tilt-deg`. Use `--history` to control the rolling window length.

---

## Multi Step motion sequence
![Basic motor control overview](./placeholder.jpg)

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

### Implementation details (from repo scripts)
- `g1/scripts/basic/g1_hl_motion_sequence.py` computes walk and turn durations from `--walk-m`, `--walk-v`, `--turn-deg`, and `--turn-vyaw`, then sends velocities at a fixed `--cmd-hz` rate using a `Move/SetVelocity` loop with explicit `StopMove` between segments.
- The script attempts a right-hand wave via whichever arm/hand client exists and skips the wave if no supported method is present.

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
