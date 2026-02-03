# Day 2: Motion Control and Locomotion

## Day outline
- [Bewegungssteuerung & Laufen](#bewegungssteuerung--laufen)
- [Dynamisches Laufen & Balance](#dynamisches-laufen--balance)
- [Regelungs-Konzepte & Tuning](#regelungs-konzepte--tuning)
- [Stabilität praxisnah testen](#stabilität-praxisnah-testen)
- [Übung: Bewegungssequenzen definieren](#übung-bewegungssequenzen-definieren)

---

## Bewegungssteuerung & Laufen
![Control layers](docs/generated_charts/day_2_chart_01.png)

![Gait state machine](docs/generated_charts/day_2_chart_02.png)

![LocoClient common calls](docs/generated_charts/day_2_chart_03.png)

**RPC CLI example**
```bash
# Service name: "sport" if ai_sport >= 8.2.0.0, otherwise "loco"
./g1_loco_client --network_interface=enp3s0 --set_velocity="0.5 0 0 1"
```

---

## Dynamisches Laufen & Balance
![Balance fundamentals](docs/generated_charts/day_2_chart_04.png)

![FSM IDs (sport service)](docs/generated_charts/day_2_chart_05.png)

---

## Regelungs-Konzepte & Tuning
![DDS topics (500 Hz)](docs/generated_charts/day_2_chart_06.png)

**Low-level safety rule**: enter debug mode (L2 + R2) before sending `rt/lowcmd` to avoid conflicts with the motion controller.

![Ankle/waist control modes (basic motion routine)](docs/generated_charts/day_2_chart_07.png)

![23 DOF motor order (LowCmd_.mode_machine == 1)](docs/generated_charts/day_2_chart_08.png)

![29 DOF motor order (LowCmd_.mode_machine == 2)](docs/generated_charts/day_2_chart_09.png)

---

## Stabilität praxisnah testen
![Remote data format](docs/generated_charts/day_2_chart_10.png)

```cpp
unitree_go::msg::dds_::LowState_ dds_low_state;
xRockerBtnDataStruct remote_key_data;
memcpy(&remote_key_data, &dds_low_state.wireless_remote()[0], 40);
```

![Safe tuning loop](docs/generated_charts/day_2_chart_11.png)

![Stability checks during runs](docs/generated_charts/day_2_chart_12.png)

---

## Übung: Bewegungssequenzen definieren
**Goal**: walk 5 steps at low speed, then wave the right arm.

```python
from unitree_sdk2py.rpc import LocoClient
from unitree_sdk2py.dds import ChannelFactory
from unitree_sdk2py.arm import ArmSdk  # Replace with actual SDK module

ChannelFactory.Instance().Init(0, "enp3s0")

loco = LocoClient()
loco.Init()

# Walk 5 steps at low speed (duration-based for a safe stop)
loco.SetVelocity(0.2, 0.0, 0.0, 5.0)

# Right-arm wave (either ArmAction or /arm_sdk)
arm = ArmSdk(side="right")
arm.Wave(repetitions=3, speed=0.8)

loco.StopMove()
```

### AI CLI agents for scripting
**Codex**
![Persona: Senior robotics engineer, safety-first.](docs/generated_charts/day_2_chart_13.png)

**Claude Code**
![You are a cautious robotics integrator. Use unitree_sdk2_python APIs and include](docs/generated_charts/day_2_chart_14.png)

**Gemini CLI**
![Role: Robotics controls engineer for Unitree G1.](docs/generated_charts/day_2_chart_15.png)
