# Unitree G1 SDK Basics (No HL Wrapper)

This guide shows the low-level pattern for controlling G1 directly with `unitree_sdk2py`, without `ef_client` or `hanger_boot_sequence`.

---

## 1) Minimal Imports

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
```

---

## 2) Network / DDS Initialization

Initialize DDS once before creating clients/subscribers.

```python
iface = "enp1s0"   # your robot NIC
domain_id = 0      # usually 0

ChannelFactoryInitialize(domain_id, iface)
```

---

## 3) Create Loco Client

```python
loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()
```

After this, you can call locomotion APIs (`Move`, `StopMove`, `Damp`, `BalanceStand`, etc.).

---

## 4) Read IMU + Pose Data (DDS Subscription)

`SportModeState_` carries IMU, pose, velocity, and other runtime state.

```python
import time

latest_sport = {"msg": None, "ts": 0.0}

def sport_cb(msg: SportModeState_):
    latest_sport["msg"] = msg
    latest_sport["ts"] = time.time()

sport_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
sport_sub.Init(sport_cb, 10)

# wait for first message
while latest_sport["msg"] is None:
    time.sleep(0.05)

msg = latest_sport["msg"]

# IMU RPY
roll = float(msg.imu_state.rpy[0])
pitch = float(msg.imu_state.rpy[1])
yaw = float(msg.imu_state.rpy[2])

# Pose (world)
x = float(msg.position[0])
y = float(msg.position[1])
z = float(msg.position[2])
```

---

## 5) Basic Motion Commands

```python
# Move(vx, vy, vyaw, continous_move=True)
loco.Move(0.2, 0.0, 0.0, continous_move=True)   # forward
time.sleep(1.0)
loco.StopMove()
```

Notes:
- `vx`: forward/backward (m/s)
- `vy`: lateral (m/s)
- `vyaw`: yaw rate (rad/s)

---

## 6) Safe Startup via `hanger_boot_sequence`

Instead of manually chaining FSM transitions, use the safety helper:

```python
from safety.hanger_boot_sequence import hanger_boot_sequence

loco = hanger_boot_sequence(iface="enp1s0")
```

What it does:
- Initializes DDS/client safely.
- Checks current FSM/mode first.
- If already in balanced stand (FSM-200), it returns immediately.
- Otherwise it runs the required transition sequence to reach motion-ready balanced stand.

Why use it:
- Avoids fragile manual timing between `Damp`, stand-up, and balance calls.
- Gives one consistent entry point before sending motion commands.

Required sequence inside `hanger_boot_sequence` (from `other/safety/hanger_boot_sequence.py`):

1. DDS + client init:
   - `ChannelFactoryInitialize(0, iface)`
   - `LocoClient().Init()`
2. Early-out check:
   - read FSM id + mode via RPC
   - if `fsm_id == 200` and `mode != 2` (feet loaded), skip full sequence and return
3. `Damp()`
4. `SetFsmId(4)` (stand-up helper state)
5. Stand-height sweep loop:
   - increment `SetStandHeight(height)` from `0` to `max_height` by `step`
   - keep checking `fsm_mode`
   - success condition: `fsm_mode == 0` (feet loaded), then continue
   - if still unloaded after max height, reset to `SetStandHeight(0.0)`, prompt operator to adjust hanger, retry
6. `BalanceStand(0)`
7. Re-apply the final loaded height (`SetStandHeight(height)`)
8. `Start()` to enter FSM-200 balance controller
9. Final normalize call: `BalanceStand(0)` best-effort

Mode meaning used by the sequence:
- `mode == 2`: feet unloaded (still hanging / not grounded)
- `mode == 0`: feet loaded (ground contact established)

---

## 7) Gait Type Control

If firmware supports gait switching:

```python
loco.SetGaitType(0)   # walk / balanced gait
loco.SetGaitType(1)   # run / continuous gait
```

Recommended pattern:

```python
# walk-style command
loco.SetGaitType(0)
loco.Move(0.2, 0.0, 0.0, continous_move=True)

# run-style command
loco.SetGaitType(1)
loco.Move(0.4, 0.0, 0.0, continous_move=True)
```

---

## 8) Full Minimal Example

```python
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

iface = "enp1s0"
domain_id = 0

ChannelFactoryInitialize(domain_id, iface)

loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()

latest = {"msg": None}
def cb(msg: SportModeState_):
    latest["msg"] = msg

sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
sub.Init(cb, 10)

while latest["msg"] is None:
    time.sleep(0.05)

loco.BalanceStand(0)
loco.SetGaitType(0)
loco.Move(0.2, 0.0, 0.0, continous_move=True)
time.sleep(1.5)
loco.StopMove()

msg = latest["msg"]
print("yaw:", float(msg.imu_state.rpy[2]))
print("pos:", float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
```

---

## 9) Practical Safety Notes

- Run on a tether or with a spotter for first tests.
- Never call `ZeroTorque()` unless you explicitly intend to drop support torque.
- Always keep a `StopMove()` path reachable (keyboard/controller emergency action).
- Validate stale sensor data before control loops (timestamp checks).
