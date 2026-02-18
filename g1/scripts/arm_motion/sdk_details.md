# G1 SDK Details: Arme und Wrist (nur Low-Level)

Diese Datei beschreibt ausschliesslich die Low-Level-Steuerung fuer G1-Arm- und Wrist-Gelenke.
High-Level-Loco/Arm-Task-Ansatze sind hier bewusst ausgeschlossen.

## 1. Wichtige Einordnung

- G1 nutzt `idl/unitree_hg` (nicht `unitree_go`).
- Fuer Armkontrolle werden direkt Gelenkkommandos gesetzt: `q`, `dq`, `kp`, `kd`, `tau`.
- Relevante Topics:
  - `rt/arm_sdk` (arm-spezifischer Low-Level-Kanal)
  - `rt/lowcmd` (voller Low-Level-Pfad)
  - `rt/lowstate` (Rueckmeldung)

## 2. Schnellstart und Laufvoraussetzungen

- Netzwerkinterface mitgeben, z. B.:
  - `python3 g1_arm5_sdk_dds_example.py enp3s0`
  - `python3 g1_arm7_sdk_dds_example.py enp3s0`
- DDS initialisieren:
  - `ChannelFactoryInitialize(0, sys.argv[1])`
- Vor dem Senden immer erst gueltigen `LowState` empfangen.

## 3. Low-Level Steuerung fuer Arme/Wrist

Es gibt zwei Wege:

- `rt/arm_sdk`: gezielte Armsteuerung mit Enable-Flag auf Index `29`
- `rt/lowcmd`: voller Roboterpfad; aktive Modi vorher freigeben

## 4. Joint-Indizes und Konstanten

### Arm/Waist Indizes

- `WaistYaw = 12`
- `WaistRoll = 13` (bei waist-locked ungueltig)
- `WaistPitch = 14` (bei waist-locked ungueltig)
- `LeftShoulderPitch = 15`
- `LeftShoulderRoll = 16`
- `LeftShoulderYaw = 17`
- `LeftElbow = 18`
- `LeftWristRoll = 19`
- `LeftWristPitch = 20` (ungueltig bei G1 23DOF)
- `LeftWristYaw = 21` (ungueltig bei G1 23DOF)
- `RightShoulderPitch = 22`
- `RightShoulderRoll = 23`
- `RightShoulderYaw = 24`
- `RightElbow = 25`
- `RightWristRoll = 26`
- `RightWristPitch = 27` (ungueltig bei G1 23DOF)
- `RightWristYaw = 28` (ungueltig bei G1 23DOF)

### Schluesselkonstanten

- `G1_NUM_MOTOR = 29` (voller `lowcmd` Pfad)
- `kNotUsedJoint = 29` (`arm_sdk` Enable-Channel)
  - `motor_cmd[29].q = 1` -> `arm_sdk` aktiv
  - `motor_cmd[29].q = 0` -> `arm_sdk` freigeben/deaktivieren

## 5. Notwendige Imports fuer `rt/arm_sdk`

```python
import time
import sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
```

### Minimalmuster (`rt/arm_sdk`)

```python
import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

kp, kd = 60.0, 1.5
low_cmd = unitree_hg_msg_dds__LowCmd_()
crc = CRC()

pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
sub = ChannelSubscriber("rt/lowstate", LowState_)
pub.Init()

latest_state = {"msg": None}
def on_state(msg):
    latest_state["msg"] = msg

sub.Init(on_state, 10)
while latest_state["msg"] is None:
    time.sleep(0.01)

arm_joints = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 12]
low_cmd.motor_cmd[29].q = 1.0  # arm_sdk enable

for j in arm_joints:
    low_cmd.motor_cmd[j].tau = 0.0
    low_cmd.motor_cmd[j].q = latest_state["msg"].motor_state[j].q
    low_cmd.motor_cmd[j].dq = 0.0
    low_cmd.motor_cmd[j].kp = kp
    low_cmd.motor_cmd[j].kd = kd

low_cmd.crc = crc.Crc(low_cmd)
pub.Write(low_cmd)
```

## 6. Notwendige Imports fuer `rt/lowcmd`

```python
import time
import sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
```

Wichtiger Unterschied:

- Bei `rt/lowcmd` musst du vor Kontrolle aktive Modi freigeben (`MotionSwitcherClient.CheckMode/ReleaseMode`).
- Bei `rt/arm_sdk` uebernimmst du gezielt Armkontrolle mit dem Enable-Wert auf Index `29`.

## 7. Sicherheitsregeln

- Arbeitsraum freihalten, keine Personen im Schwenkbereich.
- Roboter stabil aufstellen (fester, rutschfester Boden).
- Immer weich einblenden:
  - Start mit Interpolation vom Istzustand (`ratio` von 0 auf 1).
  - Keine Spruenge in `q`.
- Erst steuern, wenn gueltiger `LowState` empfangen wurde.
- Moderate Gains fuer Einstieg:
  - Typisch `kp=60`, `kd=1.5` (arm_sdk).
- Kontrollrate sinnvoll halten:
  - arm_sdk-Beispiel: `control_dt=0.02` (50 Hz)
  - full lowcmd Beispiel: `control_dt=0.002` (500 Hz)
- Immer sauber freigeben:
  - am Ende `motor_cmd[29].q -> 0`.
- Not-Aus und Fernbedienung griffbereit halten.
- Bei unbekanntem DOF-Setup (23DOF/29DOF):
  - `WristPitch/WristYaw` sowie ggf. `WaistRoll/WaistPitch` nicht blind anfahren.

## 8. Wichtige lokale Dateien

- `g1_arm5_sdk_dds_example.py`
- `g1_arm7_sdk_dds_example.py`
- `../low_level/g1_low_level_example.py`
