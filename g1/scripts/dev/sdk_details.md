# Unitree G1 SDK Grundlagen (ohne High-Level-Wrapper)

Diese Anleitung zeigt das Low-Level-Muster zur Steuerung des G1 direkt mit `unitree_sdk2py`, ohne `ef_client`.

---

## 1) Minimale Imports

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
```

---

## 2) Netzwerk- / DDS-Initialisierung

DDS einmalig initialisieren, bevor Clients/Subscribers erzeugt werden.

```python
iface = "enp1s0"   # Netzwerk-Interface zum Roboter
domain_id = 0      # in der Regel 0

ChannelFactoryInitialize(domain_id, iface)
```

---

## 3) Loco-Client erzeugen

```python
loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()
```

Danach sind Locomotion-APIs wie `Move`, `StopMove`, `BalanceStand` nutzbar.

---

## 4) IMU- und Pose-Daten lesen (DDS-Subscription)

`SportModeState_` enthaelt IMU, Pose, Geschwindigkeit und weitere Laufzeitdaten.

```python
import time

latest_sport = {"msg": None, "ts": 0.0}

def sport_cb(msg: SportModeState_):
    latest_sport["msg"] = msg
    latest_sport["ts"] = time.time()

sport_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
sport_sub.Init(sport_cb, 10)

# Auf erste Nachricht warten
while latest_sport["msg"] is None:
    time.sleep(0.05)

msg = latest_sport["msg"]

# IMU RPY
roll = float(msg.imu_state.rpy[0])
pitch = float(msg.imu_state.rpy[1])
yaw = float(msg.imu_state.rpy[2])

# Pose (Weltkoordinaten)
x = float(msg.position[0])
y = float(msg.position[1])
z = float(msg.position[2])
```

---

## 5) Grundlegende Bewegungsbefehle

```python
# Move(vx, vy, vyaw, continous_move=True)
loco.Move(0.2, 0.0, 0.0, continous_move=True)   # vorwaerts
time.sleep(1.0)
loco.StopMove()
```

Hinweise:
- `vx`: vor/zurueck (m/s)
- `vy`: seitwaerts (m/s)
- `vyaw`: Giergeschwindigkeit (rad/s)

---

## 6) Sicherer Start ueber `hanger_boot_sequence`

Statt eine FSM-Sequenz manuell zu verketten, den Safety-Helper nutzen:

```python
from safety.hanger_boot_sequence import hanger_boot_sequence

loco = hanger_boot_sequence(iface="enp1s0")
```

Was der Helper macht:
- Initialisiert DDS und Client sicher.
- Prueft zuerst den aktuellen FSM-/Mode-Zustand.
- Wenn der Roboter bereits stabil im Balanced-Stand ist (FSM-200), wird sofort zurueckgegeben.
- Sonst wird die notwendige Startsequenz ausgefuehrt.

Exakte Reihenfolge aus `other/safety/hanger_boot_sequence.py`:

1. DDS + Client init:
   - `ChannelFactoryInitialize(0, iface)`
   - `LocoClient().Init()`
2. Frueher Ruecksprung:
   - FSM-ID und Mode per RPC lesen
   - falls `fsm_id == 200` und `mode != 2`, Sequenz ueberspringen
3. `Damp()`
4. `SetFsmId(4)` (Stand-up-Helferzustand)
5. Standhoehen-Sweep:
   - `SetStandHeight(height)` in Schritten von `0` bis `max_height`
   - laufend `fsm_mode` pruefen
   - Erfolg: `fsm_mode == 0` (Fuesse tragen Last)
   - bei Misserfolg: auf `0.0` zuruecksetzen, Bediener passt Haenger an, Sweep erneut
6. `BalanceStand(0)`
7. finale Standhoehe erneut setzen (`SetStandHeight(height)`)
8. `Start()` (FSM-200 aktivieren)
9. abschliessend best effort `BalanceStand(0)`

Bedeutung der Modes in der Sequenz:
- `mode == 2`: Fuesse unbelastet
- `mode == 0`: Fuesse belastet

---

## 7) Gait-Type-Steuerung

Wenn Firmware Gait-Umschaltung unterstuetzt:

```python
loco.SetGaitType(0)   # Gehen / Balanced Gait
loco.SetGaitType(1)   # Laufen / Continuous Gait
```

Empfohlenes Muster:

```python
# Gehmodus
loco.SetGaitType(0)
loco.Move(0.2, 0.0, 0.0, continous_move=True)

# Laufmodus
loco.SetGaitType(1)
loco.Move(0.4, 0.0, 0.0, continous_move=True)
```

---

## 8) Vollstaendiges Minimalbeispiel

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

## 9) Praktische Sicherheitshinweise

- Erste Tests nur mit Sicherung/Spotter.
- `ZeroTorque()` nur bewusst einsetzen.
- Immer einen schnell erreichbaren `StopMove()`-Pfad vorsehen.
- In Regelkreisen veraltete Sensordaten pruefen (Zeitstempel).
