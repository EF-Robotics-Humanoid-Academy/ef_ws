# Day 1: Introduction and Fundamentals

---

## Intro
- EF Robotics: Overview of EF Robotics, lab environment, and safety-first operating culture.
- G1 Masterclass: What you will learn and how hands-on sessions are structured.
- Course design: Each day builds from simple, safe commands to more advanced behaviors so new learners can program the G1 with confidence.
- What you will do by the end of Day 1: connect to the robot, send a basic motion command, and understand the safety modes.

**Course Schedule**
- 08:00-09:00 Welcome and setup
- 09:00-12:00 Morning sessions (Intro, G1 Basics)
- 12:00-13:00 Lunch break
- 13:00-17:00 Afternoon sessions (Workspace Setup, SDK Basics)

**Requirements (Laptop)**
- RJ45 (Ethernet) or a docking station with RJ45
- 8-16 GB RAM
- 250 GB SSD free space

---

## Day 1 outline
- Intro
- G1 Basics
- Lunch break
- Workspace Setup
- SDK Basics

Beginner focus:
- Today is about safe setup and small, repeatable commands.

---

## G1 Basics

### Hardware
![G1 component overview](https://www.docs.quadruped.de/projects/g1/html/_images/g1_component_overview.png)

**Actors**

| Actor | Role | Notes |
| --- | --- | --- |
| Legs (hip/knee/ankle) | Locomotion | Supports multiple gaits and balance modes |
| Arms/Hands | Manipulation | Depends on arm/hand package installed |
| Waist | Upper-body orientation | Check waist fastener before use |

Beginner notes:
- Actors are what move (motors/joints). Sensors are what measure (orientation, depth, distance).
- Most beginner scripts control only a few joints at a time or use the built-in walking controller.
- Keep arm motion slow at first so balance and timing are easy to see.

**Sensors**

| Sensor | Role | Notes |
| --- | --- | --- |
| IMU | Balance and stability | Used in gait control loops |
| LiDAR | Mapping and navigation | MID-360 supported |
| Depth camera | Perception | RealSense D435i supported |

Beginner notes:
- IMU is the "balance sense"; LiDAR and camera are for mapping and perception.
- For Day 1, you do not need to process sensor data yet; we only verify they are detected.

### Software
![G1 system architecture](https://doc-cdn.unitree.com/static/2024/9/18/5da5c8fdc8f84b59aa3f2f5d45add0e4_8000x6106.jpg)

Beginner notes:
- You will use high-level RPC clients first (`LocoClient`, `ArmActionClient`) before any low-level control.
- The robot can keep balancing while you send arm commands, but we start in safe, static poses.

### Safety checklist
![Debug mode on controller](https://doc-cdn.unitree.com/static/2024/9/29/236fa93a8fae4eaa8815004f42e87ede_1065x1419.jpg)

Beginner notes:
- Always start in a clear area with hard, flat ground and no obstacles within 1–2 meters.
- For low-level commands, confirm `Debug mode` is enabled before sending any motion.
- The robot does not stop for obstacles when you send low-level velocity commands.
- Know how long a command will run (duration is part of the command, not automatic).

**Modes and meaning**

| State/Mode | Description |
| --- | --- |
| Zero-Torque | Motors unpowered for safe handling |
| Damping | Soft resistance for safe positioning |
| Balanced Stand | Stable idle stance for transitions |

**Joint limits and constraints**

| Item | Description |
| --- | --- |
| Joint limits | Keep motions inside safe mechanical ranges |
| Starting posture | Use flat hard ground for transitions |
| Debug mode | Required before sending low-level commands |

### Remote Controller
![Remote controller and debug mode](https://marketing.unitree.com/article/en/G1/Remote_Control.html)

| Keybinding | Description |
| --- | --- |
| L2 + R2 | Enter debug mode for low-level control |
| Start/Stop | Safe motion start/stop sequence |
| Mode toggle | Switch between motion modes |

Beginner notes:
- Practice entering and exiting debug mode before running scripts.
- Keep one person focused on the controller while another runs code.

### Unitree App
![Unitree app overview](https://marketing.unitree.com/article/en/G1/Remote_Control.html)

Beginner notes:
- The app is used for quick checks (camera preview, status, battery).
- We use the app for verification, not for programming.

---

## Workspace Setup
- Ubuntu 24.04: Standard OS for labs and SDK tools.
- Unitree repos: `unitree_sdk2_python`, `unitree_sdk2`, `unitree_mujoco`, `unitree_ros`.
- CycloneDDS setup: Configure DDS for reliable pub/sub on the LAN.
- Python environment: Create a dedicated venv or conda env for SDKs.
- AI CLIs: Use `codex`, `claude code`, `gemini cli` for scripting support.

Beginner steps:
- Verify the robot and laptop are on the same subnet.
- Identify your network interface name (e.g., `enp3s0`) before running SDK examples.
- Start with small test scripts first, then expand.

---

## SDK Basics
- Repo structure: `unitree_sdk2` for C++, `unitree_sdk2_python` for Python.
- Ethernet connection: Robot IP and interface name are required for SDK examples.
- Basic example: `g1_loco_client_example.py` for first motion test.

Beginner steps:
- Put the robot in balanced stand before any motion command.
- Run a short, low-speed test (e.g., 0.1–0.2 m/s for 2–3 seconds).
- Stop the robot explicitly after each motion command.

### Motion Switcher Service Interface
MotionSwitcherClient lets you release and switch motion control modes via RPC and enter your own debug mode.

**Include (C++)**
```cpp
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
```

**Core functions**

| Function | Prototype | Summary |
| --- | --- | --- |
| `CheckMode` | `int32_t CheckMode(std::string& form, std::string& name)` | Detects current form and motion control mode. |
| `SelectMode` | `int32_t SelectMode(const std::string& name)` | Selects a motion control mode. |
| `ReleaseMode` | `int32_t ReleaseMode()` | Releases current motion mode. |

Notes:
- `form` is `"0"` for Standard Form, `"1"` for Wheel-Foot Form.
- `name` is the motion control mode name as defined by the system.

**Interface error codes**

| Error number | Description | Remarks |
| --- | --- | --- |
| 7001 | Request parameter error | Server return |
| 7002 | Service busy, retry again please | Server return |
| 7004 | Unsupport mode name | Server return |
| 7005 | Internal command execute error | Server return |
| 7006 | Check command execute error | Server return |
| 7007 | Switch command execute error | Server return |
| 7008 | Release command execute error | Server return |
| 7009 | Customize config set error | Server return |

**Usage example (C++)**
```cpp
MotionSwitcherClient msc;
msc.Init();

std::string form;
std::string mode;
msc.CheckMode(form, mode);

msc.SelectMode("stand");
// ...
msc.ReleaseMode();
```
