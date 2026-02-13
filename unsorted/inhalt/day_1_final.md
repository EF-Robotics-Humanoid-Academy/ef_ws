# Day 1: Introduction and Fundamentals

---

## Intro
- EF Robotics: Overview of EF Robotics, lab environment, and safety-first operating culture.
- G1 Masterclass: What you will learn and how hands-on sessions are structured.
- Course design: Each day builds from simple, safe commands to more advanced behaviors so new learners can program the G1 with confidence.
- What you will do by the end of Day 1: connect to the robot, send a basic motion command, and understand the safety modes.

### Technical Specifications (Overview)
![G1 overview run](https://docs.quadruped.de/projects/g1/html/_images/g1_run.png)

**Body and Dimensions**

| Item | Value |
| --- | --- |
| Weight | 35 kg |
| Height | 1270 mm |
| Total Degrees of Freedom | Up to 43 |
| Max Joint Torque | 120 N.m |

**Course Schedule (Tag 1 – Mo 16.02.2026)**
- 07:30 – 09:30 Aufbau / Technik
- 09:30 Doors open Registrierung
- 10:00 – 11:30 Schulungsblock
- 11:30 – 11:45 Pause
- 11:45 – 13:00 Schulungsblock
- 13:00 – 14:00 Mittagessen
- 14:00 – 15:30 Schulungsblock
- 15:30 – 15:45 Pause
- 15:45 – 17:00 Schulungsblock
- 17:00 – 17:30 Podcast / Testimonials
- 17:30 – 17:45 Puffer
- 17:45 – 18:15 Tages‑Recap
- 18:15 – 18:30 Tagesabschluss

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

![G1 sensor overview 1](https://doc-cdn.unitree.com/static/2024/10/12/b7aab82da80940faa773f213baf13e32_13364x6401.png)
![G1 sensor overview 2](https://doc-cdn.unitree.com/static/2024/7/30/4afc81d7c48c452aaa4fc078f90a859f_627x206.png)
![G1 sensor overview 3](https://doc-cdn.unitree.com/static/2024/7/30/31b7d70c4ec1463a8143af70c43f33b9_592x897.png)

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

Controller documentation: [https://marketing.unitree.com/article/en/G1/Remote_Control.html](https://marketing.unitree.com/article/en/G1/Remote_Control.html)

### Unitree App
![Unitree app overview](https://marketing.unitree.com/article/en/G1/Remote_Control.html)

Beginner notes:
- The app is used for quick checks (camera preview, status, battery).
- We use the app for verification, not for programming.

App documentation: [https://www.unitree.com/app/g1](https://www.unitree.com/app/g1)

---

## Workspace Setup
- Ubuntu 24.04: Standard OS for labs and SDK tools.
- Unitree repos: `unitree_sdk2_python`, `unitree_sdk2`, `unitree_mujoco`, `unitree_ros`, [https://github.com/ahmedgalaief/ef_ws.git](https://github.com/ahmedgalaief/ef_ws.git).
- CycloneDDS setup: Configure DDS for reliable pub/sub on the LAN.
- Python environment: Create a dedicated venv or conda env for SDKs.
- AI CLIs: Use `codex`, `claude code`, `gemini cli` for scripting support.

Beginner steps:
- Verify the robot and laptop are on the same subnet.
- Identify your network interface name (e.g., `enp3s0`) before running SDK examples.
- Start with small test scripts first, then expand.

### AI CLI Agents (Codex and Friends)
![Codex CLI](./codex.jpg)

Use AI CLI agents to speed up repetitive work: generate small scripts, explain SDK snippets, and draft troubleshooting steps.

Beginner tips:
- Ask for tiny, specific tasks (e.g., "write a 3-second walk command").
- Verify the output before running it on the robot.
- Keep a human in the loop for safety checks and parameter review.

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

---

## Implementation details (from repo scripts)
- `g1/scripts/safety/hanger_boot_sequence.py` implements a repeatable boot path from hanger to balanced stand. It calls `ChannelFactoryInitialize(0, iface)`, creates a `LocoClient`, then sequences `Damp` → `SetFsmId(4)` → `SetStandHeight` sweep until `FSM mode == 0` (feet loaded) → `BalanceStand` → `Start`. If the robot is already in FSM 200 with loaded feet, it returns early to avoid re-running the sequence.
- The same script polls `ROBOT_API_ID_LOCO_GET_FSM_ID` and `ROBOT_API_ID_LOCO_GET_FSM_MODE` to confirm the current state and to decide whether another height sweep is needed.
- `g1/scripts/safety/keyboard_controller.py` provides a minimal teleop loop with `pynput` for key-hold detection and sends `Move(vx, vy, omega, continous_move=True)` at 10 Hz. Key mapping is W/S for forward/back, A/D for yaw, Q/E for lateral, Space for stop, `Z` for `Damp`, and `Esc` for `StopMove` + `ZeroTorque`.
- `g1/scripts/unsorted/g1_loco_client_example.py` is the Day 1 interactive test harness. It initializes DDS, then lets you trigger actions like `Damp`, `Move`, `WaveHand`, `ZeroTorque`, and posture changes via a simple CLI menu.
