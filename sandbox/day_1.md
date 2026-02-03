# Day 1: Introduction and Fundamentals

## Day outline
- [Einführung EF Robotics & Academy Setup](#einführung-ef-robotics--academy-setup)
- [Überblick Unitree G1 Hardware & Software](#überblick-unitree-g1-hardware--software)
- [Sicherheitskonzept & Inbetriebnahme](#sicherheitskonzept--inbetriebnahme)
- [Erste Steuerung: G1 bewegen & verstehen](#erste-steuerung-g1-bewegen--verstehen)

---

## Einführung EF Robotics & Academy Setup
![Lab workflow](docs/generated_charts/day_1_chart_01.png)

**Networking and access**
![Default IPs (wired)](docs/generated_charts/day_1_chart_02.png)

```bash
# Set your laptop to 192.168.123.222 or 192.168.123.99
ip addr
ping 192.168.123.161
ping 192.168.123.164

# Find the network interface name for SDK examples
ip addr | rg "192.168.123" -n
```

```bash
ssh unitree@192.168.123.164
```

**WiFi setup (FAQ)**
```bash
# AP mode on PC2 (example)
sudo create_ap wlan0 eth0 G1_AP <password>

# STA mode using NetworkManager (example)
sudo nmcli dev wifi connect <ssid> password <password>
```

---

## Überblick Unitree G1 Hardware & Software
![Unitree G1 MJCF render](docs/repos/unitree_g1_vibes/RL-shenanigans/unitree_mujoco/unitree_robots/g1/images/g1_29dof.png)

**Structure and DOF**
![G1 structure](docs/generated_charts/day_1_chart_03.png)

![G1 system architecture](https://doc-cdn.unitree.com/static/2024/9/18/5da5c8fdc8f84b59aa3f2f5d45add0e4_8000x6106.jpg)

![Onboard compute roles](docs/generated_charts/day_1_chart_04.png)

**Key software notes**
- DDS is the native middleware, and the IDL is ROS 2 compatible.
- SDKs support both publish/subscribe and request/response patterns.
- G1 software does not currently support GStreamer video streaming.
- Cloud connectivity is available only after authorization.

### Sensors and perception hardware
![Livox MID-360](https://doc-cdn.unitree.com/static/2024/12/18/b68f35fca9724bfcae513610179e0bed_356x393.png)

![Intel RealSense D435i](https://doc-cdn.unitree.com/static/2024/7/26/ddc4f187060d434587e2f08ea0045e3c_1042x718.png)

![Primary sensors](docs/generated_charts/day_1_chart_05.png)

### Waist fastener (EDU 29 DOF)
![Waist fastener positions](https://doc-cdn.unitree.com/static/2024/10/12/4b8f516b51164efda1bca73908a7e33c_3160x2374.png)

![Waist lock checklist](docs/generated_charts/day_1_chart_06.png)

---

## Sicherheitskonzept & Inbetriebnahme
![R3 remote layout](https://doc-cdn.unitree.com/static/2024/1/8/5a0057f38e464df3a1bdfb806edb1334_1600x819.png)

![Modes and behavior](docs/generated_charts/day_1_chart_07.png)

![LED strip colors](docs/generated_charts/day_1_chart_08.png)

![Critical button sequences](docs/generated_charts/day_1_chart_09.png)

**Safety notes**
- Avoid stair climbing; current gait does not support it.
- Starting from lying/squatting is only for flat hard ground.
- When using the SDK, always confirm debug mode to avoid command conflicts.

![Debug prerequisites](docs/generated_charts/day_1_chart_10.png)

### Status and error codes (quick reference)
![Device status (bitmask)](docs/generated_charts/day_1_chart_11.png)

![Motor status (bitmask)](docs/generated_charts/day_1_chart_12.png)

---

## Erste Steuerung: G1 bewegen & verstehen
**Clone the SDKs**
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

**Build C++ SDK and examples**
```bash
cd unitree_sdk2
cmake -B build
cmake --build build
```

**DDS pub/sub skeleton (C++)**
```cpp
unitree::robot::ChannelFactory::Instance()->Init(0, "enp3s0");
unitree::robot::ChannelPublisher<MSG> pub("rt/lowcmd");
unitree::robot::ChannelSubscriber<MSG> sub("rt/lowstate");
```

**Run the low-level ankle swing example (robot must be suspended)**
```bash
./build/bin/g1_ankle_swing_example <network_interface_name>
```

**Run the high-level locomotion example (RPC)**
```bash
./build/bin/g1_loco_client_example --network_interface=<network_interface_name> --set_velocity="0.2 0 0 1"
```

**Python examples (G1)**
```bash
cd unitree_sdk2_python
python3 example/g1/high_level/g1_loco_client_example.py
python3 example/g1/high_level/g1_arm_action_example.py
python3 example/g1/low_level/g1_low_level_example.py
```

### Voice assistant and audio playback
![Voice assistant (firmware >= 1.3.0)](docs/generated_charts/day_1_chart_13.png)

![Audio playback constraints](docs/generated_charts/day_1_chart_14.png)

```bash
python3 example/g1/audio/g1_audio_client_play_wav.py --file example/g1/audio/test.wav
```
