# Unitree G1 SDK Basics with `ef_client.Robot`

This guide mirrors `sdk_details.md`, but uses the high-level wrapper `ef_client.Robot`.

---

## 1) Minimal Imports

```python
from ef_client import Robot
```

---

## 2) Network / DDS Initialization

`Robot` initializes DDS/client internals for you.

```python
iface = "enp1s0"
domain_id = 0

robot = Robot(iface=iface, domain_id=domain_id)
```

---

## 3) Create Robot Client

```python
robot = Robot("enp1s0")
```

Default behavior uses the safety boot path and starts sensor subscriptions.

---

## 4) Read IMU + Pose Data

```python
imu = robot.get_imu()
pos = robot.get_position()

if imu is not None:
    roll, pitch, yaw = imu.rpy
    print("rpy:", roll, pitch, yaw)

if pos is not None:
    x, y, z = pos
    print("pos:", x, y, z)
```

### IMU Data Flowchart

![IMU correction loop](/tmp/imu_correction_loop.png)

---

## 5) Basic Motion Commands

```python
import time

robot.walk(0.2, 0.0, 0.0)   # forward command (balanced gait)
time.sleep(1.0)
robot.stop()
```

Notes:
- `vx`: forward/backward (m/s)
- `vy`: lateral (m/s)
- `vyaw`: yaw rate (rad/s)

---

## 6) Safe Startup via `hanger_boot_sequence` (through `Robot`)

`Robot(...)` already uses `hanger_boot_sequence` by default (`safety_boot=True`), so you normally do not need manual FSM calls.

If you need to re-run the safe boot flow later:

```python
robot.hanged_boot()
```

---

## 7) Gait Type Control

```python
robot.set_gait_type(0)      # walk gait
robot.loco_move(0.2, 0.0, 0.0)

robot.set_gait_type(1)      # run gait
robot.loco_move(0.4, 0.0, 0.0)
```

Or use wrappers:

```python
robot.walk(0.2, 0.0, 0.0)
robot.run(0.4, 0.0, 0.0)
```

---

## 8) Full Minimal Example

```python
import time
from ef_client import Robot

robot = Robot(iface="enp1s0", domain_id=0)
time.sleep(0.5)  # allow first sensor callbacks

robot.walk(0.2, 0.0, 0.0)
time.sleep(1.5)
robot.stop()

imu = robot.get_imu()
pos = robot.get_position()
print("yaw:", None if imu is None else imu.rpy[2])
print("pos:", pos)
```

---

## 9) Practical Safety Notes

- Start with `Robot(...)` default safety boot.
- Keep `robot.stop()` reachable in your control flow.
- Validate `robot.get_position()` / `robot.get_imu()` before feedback-controlled motion.
- Use a tether/spotter for early tests.

---

## 10) Extra: `walk_for` Example

Move a relative distance with IMU/pose feedback correction:

```python
ok = robot.walk_for(1.0)   # move ~1 meter forward
print("walk_for ok:", ok)
```

With tighter controls:

```python
ok = robot.walk_for(
    distance=0.6,
    max_vx=0.2,
    max_vyaw=0.4,
    pos_tolerance=0.03,
    timeout=12.0,
)
print("walk_for ok:", ok)
```

---

## 11) Extra: `rotate_joint` Example

Rotate a single arm joint:

```python
rc = robot.rotate_joint("elbow", 20)   # right arm by default
print("rotate_joint rc:", rc)
```

Left arm example:

```python
rc = robot.rotate_joint("wrist_roll", 15, arm="left", duration=1.2, hold=0.2)
print("rotate_joint rc:", rc)
```
