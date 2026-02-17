# Unitree G1 SDK Grundlagen mit `ef_client.Robot`

Diese Anleitung entspricht inhaltlich der Low-Level-Variante, verwendet aber den High-Level-Wrapper `ef_client.Robot`.

---

## 1) Minimaler Import

```python
from ef_client import Robot
```

---

## 2) Netzwerk- / DDS-Initialisierung

`Robot` initialisiert die benoetigten Komponenten intern.

```python
iface = "enp1s0"
domain_id = 0

robot = Robot(iface=iface, domain_id=domain_id)
```

---

## 3) Robot-Client erzeugen

```python
robot = Robot("enp1s0")
```

Standardverhalten:
- Safety-Boot aktiv
- Sensorsubscriptions werden gestartet

---

## 4) IMU- und Pose-Daten lesen

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

### IMU-Flowchart

![IMU correction loop](/tmp/imu_correction_loop.png)

---

## 5) Grundlegende Bewegungsbefehle

```python
import time

robot.walk(0.2, 0.0, 0.0)   # vorwaerts im Balanced-Gait
time.sleep(1.0)
robot.stop()
```

Hinweise:
- `vx`: vor/zurueck (m/s)
- `vy`: seitwaerts (m/s)
- `vyaw`: Giergeschwindigkeit (rad/s)

---

## 6) Sicherer Start via `hanger_boot_sequence` (ueber `Robot`)

`Robot(...)` nutzt standardmaessig bereits `hanger_boot_sequence` (`safety_boot=True`).

Wenn die sichere Sequenz spaeter erneut gestartet werden soll:

```python
robot.hanged_boot()
```

---

## 7) Gait-Type-Steuerung

```python
robot.set_gait_type(0)      # Gehen
robot.loco_move(0.2, 0.0, 0.0)

robot.set_gait_type(1)      # Laufen
robot.loco_move(0.4, 0.0, 0.0)
```

Oder direkt ueber Wrapper:

```python
robot.walk(0.2, 0.0, 0.0)
robot.run(0.4, 0.0, 0.0)
```

---

## 8) Vollstaendiges Minimalbeispiel

```python
import time
from ef_client import Robot

robot = Robot(iface="enp1s0", domain_id=0)
time.sleep(0.5)  # kurze Wartezeit bis erste Sensordaten verfuegbar sind

robot.walk(0.2, 0.0, 0.0)
time.sleep(1.5)
robot.stop()

imu = robot.get_imu()
pos = robot.get_position()
print("yaw:", None if imu is None else imu.rpy[2])
print("pos:", pos)
```

---

## 9) Praktische Sicherheitshinweise

- Fuer erste Tests immer absichern/spotten.
- `robot.stop()` schnell erreichbar halten.
- Vor Feedback-Reglern `robot.get_position()` und `robot.get_imu()` auf Verfuegbarkeit pruefen.

---

## 10) Zusatz: `walk_for` Beispiel

Relative Strecke mit IMU-/Pose-Feedback:

```python
ok = robot.walk_for(1.0)   # ca. 1 m vorwaerts
print("walk_for ok:", ok)
```

Mit feineren Parametern:

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

## 11) Zusatz: `rotate_joint` Beispiel

Einzelnes Armgelenk rotieren:

```python
rc = robot.rotate_joint("elbow", 20)   # standardmaessig rechter Arm
print("rotate_joint rc:", rc)
```

Beispiel linker Arm:

```python
rc = robot.rotate_joint("wrist_roll", 15, arm="left", duration=1.2, hold=0.2)
print("rotate_joint rc:", rc)
```
