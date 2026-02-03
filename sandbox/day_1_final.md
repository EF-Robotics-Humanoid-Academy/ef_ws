# Day 1: Introduction and Fundamentals

---

## Intro
- EF Robotics: Overview of EF Robotics, lab environment, and safety-first operating culture.
- G1 Masterclass: What you will learn and how hands-on sessions are structured.

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

---

## G1 Basics

### Hardware
![Placeholder](./placeholder.jpg)

**Actors**

| Actor | Role | Notes |
| --- | --- | --- |
| Legs (hip/knee/ankle) | Locomotion | Supports multiple gaits and balance modes |
| Arms/Hands | Manipulation | Depends on arm/hand package installed |
| Waist | Upper-body orientation | Check waist fastener before use |

**Sensors**

| Sensor | Role | Notes |
| --- | --- | --- |
| IMU | Balance and stability | Used in gait control loops |
| LiDAR | Mapping and navigation | MID-360 supported |
| Depth camera | Perception | RealSense D435i supported |

### Software
![G1 system architecture](https://doc-cdn.unitree.com/static/2024/9/18/5da5c8fdc8f84b59aa3f2f5d45add0e4_8000x6106.jpg)

### Safety checklist
![Placeholder](./placeholder.jpg)

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
![Placeholder](./placeholder.jpg)

| Keybinding | Description |
| --- | --- |
| L2 + R2 | Enter debug mode for low-level control |
| Start/Stop | Safe motion start/stop sequence |
| Mode toggle | Switch between motion modes |

### Unitree App
![Placeholder](./placeholder.jpg)

---

## Workspace Setup
- Ubuntu 24.04: Standard OS for labs and SDK tools.
- Unitree repos: `unitree_sdk2_python`, `unitree_sdk2`, `unitree_mujoco`, `unitree_ros`.
- CycloneDDS setup: Configure DDS for reliable pub/sub on the LAN.
- Python environment: Create a dedicated venv or conda env for SDKs.
- AI CLIs: Use `codex`, `claude code`, `gemini cli` for scripting support.

---

## SDK Basics
- Repo structure: `unitree_sdk2` for C++, `unitree_sdk2_python` for Python.
- Ethernet connection: Robot IP and interface name are required for SDK examples.
- Basic example: `g1_loco_client_example.py` for first motion test.
