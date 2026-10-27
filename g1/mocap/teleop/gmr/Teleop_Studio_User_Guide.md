# Teleop Studio User Guide

> Teleoperation Console — Moretion MoCap / GMR Retargeting / SONIC Whole-Body Control

Teleop Studio is a desktop GUI application built on PySide6. It retargets human motion-capture data in real time and uses SONIC to control either a MuJoCo simulator or a Unitree G1 humanoid robot. The entire pipeline — *Moretion MoCap Stream → GMR Motion Retargeting → SONIC Whole-Body Control* — is unified into a single control panel. The operator only needs to fill in parameters, start retargeting, connect SONIC, and start / pause tracking.

---

## Table of Contents

1. [System Architecture & How It Works](#1-system-architecture--how-it-works)
2. [Runtime Environment & Hardware Setup](#2-runtime-environment--hardware-setup)
3. [Installation & Launch](#3-installation--launch)
4. [UI Overview](#4-ui-overview)
5. [Parameter Reference](#5-parameter-reference)
6. [Status Indicators](#6-status-indicators)
7. [Control Buttons](#7-control-buttons)
8. [Log Panels](#8-log-panels)
9. [Standard Workflow](#9-standard-workflow)
10. [Safety & Emergency Stop](#10-safety--emergency-stop)
11. [Frequently Asked Questions](#11-frequently-asked-questions)

---

## 1. System Architecture & How It Works

Teleop Studio orchestrates three sub-systems into a single real-time pipeline:

```
┌────────────────┐    ┌────────────────────┐   ZMQ (50 Hz)   ┌────────────────────────┐
│ Moretion MoCap │ → │ GMR Retargeting    │ ──────────────→ │ SONIC WBC              │
│ Server         │    │ (local sub-process)│   pose topic    │ (local or remote host) │
└────────────────┘    │ moretion_stream_   │                 │ deploy.sh              │
 TCP/UDP :9999        │ to_sonic.py        │                 └────────────────────────┘
                      └────────────────────┘                              │
                                                                          ▼
                                                               MuJoCo / Unitree G1
```

- **Moretion MoCap Server** — Streams human skeleton pose per frame (joint Euler angles + root position) via TCP or UDP to the local machine.
- **GMR Retargeting (local sub-process)** — Receives MoCap frames, runs IK retargeting to produce robot joint angles, and publishes the `pose` topic via ZMQ PUB at 50 Hz.
- **SONIC Whole-Body Control (deployment host)** — Runs `deploy.sh` on the local or remote Linux host, subscribes to the ZMQ `pose` topic, and controls either the MuJoCo simulator or the physical robot.

The Teleop Studio UI itself only handles *parameter input + start/stop + status monitoring* — it does not directly participate in data transfer.

**Key point:** A remote SONIC deployment host must be able to reach the operator PC's ZMQ port (default 5556). Therefore, *Local Network Interface* must be set to a NIC that the remote host can reach. Local deployment always uses `localhost`.

---

## 2. Runtime Environment & Hardware Setup

### 2.1 Operating System

- Linux / Ubuntu (recommended; local deploy mode is Linux-only)
- Windows (remote SSH deploy mode only)

### 2.2 Python Environment

A conda environment named `gmr` (dev mode) or a bundled conda-pack environment (packaged mode) is required. The launch scripts auto-detect and activate the correct one — no manual activation is needed.

### 2.3 Hardware List

| Device | Notes |
|--------|-------|
| Humanoid robot | Unitree G1 (29 DOF). Default SSH address `192.168.3.9`, user `unitree` |
| MoCap server | Moretion. Default IP `192.168.2.132`, port `9999` |
| Operator PC | The machine running Teleop Studio. Must reach the MoCap server and, as applicable, the remote host or robot network |

### 2.4 Network

- The operator PC must reach the MoCap server. For remote deployment, it must also reach the remote Linux host.
- By default, the app prefers a NIC on `192.168.3.*` (a common robot segment) as the ZMQ Host, so the remote deployment host can connect back to the operator PC.
- In Remote SSH mode, the operator PC must be able to SSH into the configured SSH host.

---

## 3. Installation & Launch

### 3.1 Linux / Ubuntu

```bash
bash scripts/run_teleop_studio.sh
```

The launch script auto-detects the running mode:

- **Packaged mode** — an `env/` directory (conda-pack environment) and a `gmr/` directory (repo) exist next to the script. First run auto-executes `conda-unpack`, then activates and starts the app.
- **Dev mode** — no sibling `env/` directory is found; it falls back to activating the conda `gmr` environment and uses the parent directory as the repo.

The script automatically prepends PySide6's bundled Qt6 library directory to `LD_LIBRARY_PATH`, preventing the system Qt6 from being loaded first and causing missing-symbol errors.

### 3.2 Windows

```powershell
./scripts/run_teleop_studio.ps1
```

Same packaged / dev mode support. In dev mode, conda is auto-detected (PATH / registry / common install locations), preferring the `gmr` env and falling back to `base`.

Override via environment variables:

- `CONDA_ROOT` — conda install directory
- `CONDA_ENV` (Windows) — conda environment name, default `gmr`

Once launched, a window opens with the English title `Teleop Studio - moretion / GMR / SONIC Teleoperation` and an initial size of 1100×760.

---

## 4. UI Overview

The UI is organized top-to-bottom into 4 groups:

1. **Run Parameters** — GMR, network, and deployment configuration
2. **Status Indicators** — 4 colored dot LEDs
3. **Control Buttons** — Retargeting, Robot, and Tracking buttons
4. **Log Panels** — Two read-only text boxes for GMR output and SONIC output

Use the *Language* menu in the menu bar to switch instantly between *中文* and *English*. Switching does not clear any entered parameters. The selected language is saved and restored the next time the application starts.

---

## 5. Parameter Reference

### 5.1 Retargeting (GMR) Parameters

| Control | Description | Default |
|---------|-------------|---------|
| GMR Python | Absolute path to the Python executable (conda env) used to run the retargeting script | Current `sys.executable` |
| GMR Repo Dir | Root directory of the GMR repo, used to locate `scripts/moretion_stream_to_sonic.py` | Auto-detected from the location of `teleop_studio/` |
| Moretion Address | IP of the MoCap server | `192.168.2.132` |
| Moretion Port | MoCap stream port (1–65535) | `9999` |
| UDP | If checked, receive MoCap data over UDP instead of TCP | Off |
| Keep Retargeting Window | If checked, keep the MuJoCo visualization window (useful to observe retargeting output) | On |

### 5.2 Network Interface

| Control | Description |
|---------|-------------|
| Local Network Interface | Drop-down listing all IPv4 NICs (including WSL), with loopback and link-local filtered. A *Refresh* button on the right re-enumerates them |
| SONIC ZMQ Host | Auto-filled from the IP of the selected NIC; fixed to `localhost` in local deploy mode |
| SONIC ZMQ Port | ZMQ publisher port, default `5556` |

> Tip: By default, the `192.168.3.*` segment (a common robot segment) is pre-selected. If not, manually select a NIC that the remote SONIC deployment host can reach.

### 5.3 Deployment and Control

Two options in the drop-down:

- **Remote SSH (deploy SONIC on a remote host)** — SSH into a Linux host and run `deploy.sh` there. Available on both Windows and Linux.
- **Local Deploy (SONIC on this machine, wired to robot)** — Run `deploy.sh` locally via PTY. Linux-only.

The deployment location is independent from the runtime target. Select **MuJoCo Simulation** (default) when SONIC controls the simulator, including a simulator started over SSH from Windows. Select **Unitree G1 Robot** only for real hardware. Both targets are available in local and remote deployment modes.

| Control | Options and effect | Default |
|---------|--------------------|---------|
| Runtime Target | **MuJoCo Simulation** passes `sim` to `deploy.sh`; **Unitree G1 Robot** passes `real` | MuJoCo Simulation |
| Control Model | **Default Model** uses SONIC's default policy; **Low-latency Model** uses `policy/low_latency/model` and its `observation_config.yaml` | Default Model |
| Motion Preview Delay | Measured GMR preview followed by pose holds for unknown future samples; `-1` selects full preview (920 / 200 ms) | Default model: 400 ms; low-latency: 100 ms |

Switching the model resets the delay to its compromise default. Both settings can
be changed while only GMR retargeting is running, without restarting its viewer.
They lock once deployment starts, so sender and controller remain matched.
Each packet retains one past frame, the target, and the model's future reference
window; `--batch 2` counts only the past frame and target. See `SONIC_LATENCY.md`.

### 5.4 SSH / Remote Deploy (enabled in Remote SSH mode only)

| Control | Default |
|---------|---------|
| SSH Host | `192.168.3.9` |
| SSH User | `unitree` |
| SSH Password | (empty — password mode) |
| SSH Private Key | Optional. If left empty, password login is used |
| Deploy Dir | Remote: `/home/unitree/workspace/GR00T-WholeBodyControl/gear_sonic_deploy`; Local: `/home/zpf/workspace/GR00T-WholeBodyControl/gear_sonic_deploy` |

---

## 6. Status Indicators

Four colored dot LEDs are placed in the middle of the UI:

| LED | Color Meaning |
|-----|---------------|
| GMR Retargeting | Gray = idle, Blue = running |
| SSH Connection / Local Deploy | Gray = idle, Green = connected |
| Control State | Orange = waiting for control, Green = in control, Gray = unknown |
| Tracking State | Green = tracking, Orange = paused, Gray = unknown |

*Control State* and *Tracking State* are parsed automatically from keyword matches in the SONIC deploy log — no manual input is required.

---

## 7. Control Buttons

Three buttons. Their colors change with state. During transitions they turn gray and are disabled to prevent accidental clicks.

| Button | State-dependent display and meaning |
|--------|-------------------------------------|
| Retargeting | Green *Start* → (running) Red *Stop* |
| Robot | Green *Connect* → (in control) Red *E-Stop* |
| Tracking | Green *Start\nTracking* → (tracking) Red *Pause\nTracking*. **Enabled only when the robot is connected and in control** |

The Tracking button is a large chamfered square spanning two rows, so the operator can click it quickly.

---

## 8. Log Panels

Two read-only text boxes, capped at 5000 lines, using a monospace font:

- **GMR Output** — stdout / stderr of the retargeting sub-process, auto-scrolled in real time.
- **SONIC Output** — Output from the remote SSH shell or local PTY, auto-scrolled in real time.

Logs are the primary diagnostic resource. When something goes wrong, look at these two panels first.

---

## 9. Standard Workflow

A complete teleoperation session follows these steps.

### Step 0: Prepare the Moretion MoCap Server

- Start the MoCap server streaming on the Moretion server.
- Use either live motion capture or prerecorded motion playback as the input.

### Step 1: Launch the App

```bash
bash scripts/run_teleop_studio.sh     # Linux                
./scripts/run_teleop_studio.ps1       # Windows PowerShell
```

### Step 2: Verify / Fill in Run Parameters

- Check that GMR Python and GMR Repo Dir are correct.
- Fill in the Moretion MoCap server IP and port (confirm the server has already started streaming).
- Select the correct Local Network Interface (the remote SONIC deployment host must be able to reach its IP).
- Choose a deployment mode:
  - Remote SSH: fill in SSH host / user / password-or-key and deploy directory.
  - Local Deploy: confirm the deploy directory is correct.
- Choose the runtime target: keep **MuJoCo Simulation** for Sonic Sim; use **Unitree G1 Robot** only for hardware deployment.
- Choose the control model: normally keep **Default Model**; select **Low-latency Model** when the low-latency policy is required.

### Step 3: Start GMR Retargeting

1. Ensure the MoCap server is ready and streaming.
2. Click the green *Start* retargeting button.
3. The app launches `moretion_stream_to_sonic.py` as a sub-process:
   - Connects to the Moretion server, waits for the first frame, and auto-detects the human height.
   - Initializes the IK retargeter and (optionally) the MuJoCo visualization window.
   - Publishes the `pose` topic at 50 Hz via ZMQ to `tcp://*:5556`.
4. The retargeting LED turns blue (running), the button turns red (*Stop*), and GMR Output starts scrolling.

### Step 3.1: Complete Pre-Hardware Validation

Before connecting the physical robot, complete the following checks in order. Do not proceed to the next stage if any abnormality is found:

1. Keep *Keep Retargeting Window* enabled and verify that the retargeted reference motion closely follows the human motion. Pay particular attention to left/right mapping, joint directions, motion range, and pose continuity. Confirm that there are no sudden jumps, reversed motions, or obvious distortions.
2. Set the runtime target to **MuJoCo Simulation** and run the complete motion planned for the physical robot. Confirm that the robot pose, balance, and joint motion are normal, and verify that Pause Tracking and E-Stop work correctly.
3. Only after the simulation behaves normally may you switch the runtime target to **Unitree G1 Robot**. Start the first hardware test with small, slow motions and a short duration, then gradually increase the motion range and duration after confirming stable operation.

### Step 4: Connect SONIC / the Robot

1. Click the green *Connect* robot button.
   - Remote SSH mode: opens an interactive SSH shell via paramiko and sends the deploy command:
     ```
     cd <deploy_dir> && bash deploy.sh [low-latency-model options] --input-type zmq \
       --zmq-host <local-WiFi-IP> --zmq-port <sonic_port> \
       --zmq-topic pose <sim|real>
     ```
     With **Low-latency Model**, `[low-latency-model options]` is `--cp policy/low_latency/model --obs-config policy/low_latency/observation_config.yaml`. The default model omits both options.
   - Local Deploy mode: runs the same deploy command via a local PTY.

2. The app auto-handles `deploy.sh` interaction prompts (auto-answers *yes* and sends `]` to enter the control system).
3. When the log shows `transitioning to control state`, the Control State LED turns green (in control) and the robot button turns red (*E-Stop*).

### Step 5: Start Tracking

1. Confirm the robot has entered control state (Control State LED is green).
2. The Tracking button is now enabled — click the large green *Start\nTracking* button.
3. The app sends the Enter key to the deploy shell, toggling the SONIC planner. When the log shows `planner initialized successfully`, the Tracking State LED turns green (tracking) and the button turns red (*Pause\nTracking*).
4. MoCap data is now streamed to the selected runtime target in real time, starting teleoperation or simulation.

### Step 6: Pause / Resume Tracking

- Click the red *Pause\nTracking* while tracking to pause the planner (log shows `disabling planner`, Tracking State LED turns orange *paused*).
- Click the green button again to resume.

### Step 7: End the Session

1. Click *Pause\nTracking* first to stop tracking.
2. Click the red *E-Stop* to stop SONIC control (sends the `O` key to trigger emergency stop).
3. Click the red *Stop* to stop the GMR retargeting sub-process:
   - Linux: sends SIGTERM with a 3-second timeout fallback to `kill`.
   - Windows: sends CTRL_BREAK_EVENT.
   - Before exiting, the sub-process sends a `stop` command and closes the receiver / viewer.

---

## 10. Safety & Emergency Stop

> **Warning:** Physical-robot teleoperation carries risks of falling, loss of control, and unexpected motion. Software limits and E-Stop do not replace on-site safety measures. Do not start hardware tracking until all checks in this section have been completed.

### 10.1 Suspension and Work-Area Safety

- During routine commissioning and teleoperation, suspend the robot from a support frame or safety tether with an adequate rated load to limit its fall range. Before each use, inspect the frame, suspension points, straps, and connectors, and ensure that the suspension does not restrict normal motion or pull on cables.
- Never stand beneath a suspended robot. Establish a safety zone around the robot and remove furniture, cables, and other obstacles. Operators and bystanders must remain at a safe distance and outside any area the robot could fall into or sweep with its limbs.
- Assign at least one person to continuously monitor the robot and its surroundings during hardware tests. Keep the E-Stop control and robot power-disconnect device immediately accessible and unobstructed.
- Before startup, inspect the robot body, power supply, communication cables, and fasteners. Do not start or continue testing if the robot status is abnormal, the suspension is unreliable, the network is unstable, or anyone enters the safety zone.

### 10.2 Three-Stage Pre-Hardware Validation

Always follow this sequence: **inspect the retargeted reference motion → validate in MuJoCo simulation → test on the physical robot**.

1. **Inspect the retargeted reference motion:** Enable *Keep Retargeting Window*. Confirm that the reference motion closely follows the human motion, with correct left/right mapping, joint directions, and motion range, and without pose jumps, jitter, or obvious distortion.
2. **Validate in simulation:** Select **MuJoCo Simulation** and execute the complete planned motion. Check balance, foot contact, joint movement, and motion continuity, and explicitly test the Pause Tracking and E-Stop procedures.
3. **Test on the physical robot:** Switch to **Unitree G1 Robot** only after the first two stages pass. Keep the robot suspended, clear the safety zone, and begin with small, slow, short-duration motions. Expand the test range gradually only after confirming normal operation.

If the motion does not match the operator, a joint moves in the wrong direction, the pose jumps or continuously jitters, foot contact is abnormal, or the robot shows signs of losing balance, pause immediately and investigate. Never carry a known abnormality into the next validation stage.

### 10.3 Operation and Incident Response

- **Emergency Stop (E-Stop):** While the robot is in control state, the Robot button reads red *E-Stop*. Clicking it sends the uppercase `O` key to the deploy process's `keyboard_handler` and triggers an emergency stop. Before testing, make sure the operator knows where the E-Stop is and how to use it.
- **Maintain supervision:** The operator must not leave the console while tracking is active. The safety observer must continuously watch the robot, suspension system, and safety zone. Do not treat status indicators or software limits as substitutes for direct supervision.
- **Respond to abnormalities:** If unexpected motion, severe jitter, loss of balance, a communication failure, or a person entering the safety zone occurs, pause tracking immediately. If the robot does not stop or could continue moving, trigger E-Stop and, when necessary, disconnect robot power according to the site procedure. Resume testing only after the robot has completely stopped and the cause has been resolved.
- **Normal shutdown order:** When ending a session, always pause tracking first, then E-Stop the robot, and finally stop retargeting. This avoids cutting upstream data while the robot is still moving.
- **Safety parameters do not replace physical safeguards:** The SONIC configuration includes velocity limits (arms 25 rad/s, lower body 20 rad/s), waist-pitch angle limits, and joint safety modes (`kill` / `freeze`). Suspension, isolation, and staged validation are still mandatory.

---

## 11. Frequently Asked Questions

**Q1: The launcher reports permission denied or says the script is not signed?**
A: On Linux, run `chmod +x scripts/run_teleop_studio.sh` before executing the script directly. The documented `bash scripts/run_teleop_studio.sh` command does not require the executable bit. In Windows PowerShell, if script execution is blocked or the script is reported as unsigned, run `Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned`, then remove the downloaded-file block with `Unblock-File -Path scripts/run_teleop_studio.ps1`.

**Q2: On launch I see Qt symbol errors like `undefined symbol _ZN14QObjectPrivateC2Ei`?**
A: The system Qt6 is being loaded before PySide6's bundled Qt6. The launch script already prepends PySide6's Qt6 lib directory to `LD_LIBRARY_PATH`. If you still see this, make sure you launch via `scripts/run_teleop_studio.sh` rather than directly with `python -m teleop_studio.app`.

**Q3: The NIC I need isn't in the network-interface dropdown?**
A: Click *Refresh* to re-enumerate. The app filters out loopback (127.x) and link-local (169.254.x) addresses but keeps WSL virtual interfaces. If it's still missing, verify that the NIC has been assigned an IPv4 address.

**Q4: After starting retargeting, GMR Output is stuck waiting for the first frame?**
A: Verify that Moretion address / port are correct, the MoCap server is actually streaming, and the protocol (TCP/UDP) matches (TCP is default; UDP must be explicitly checked). You can temporarily use a network utility to confirm the local machine can reach the MoCap server.

**Q5: After connecting the robot, the Control State LED stays orange "waiting for control"?**
A: Look at the SONIC Output log. The app parses state from log keywords: `transitioning to control state` → in control; `planner initialized successfully` → tracking. It will stall if `deploy.sh`'s confirmation prompt was not auto-passed, or if the deploy directory / ZMQ host / port are misconfigured. Make sure `--zmq-host` points to an IP on the operator PC that the SONIC deployment host can reach, and `--zmq-port` matches the SONIC ZMQ port.

**Q6: The Tracking button is grayed out and un-clickable?**
A: The Tracking button is only enabled when the robot is connected AND in control state. Finish Step 4 first and wait until the Control State LED turns green.

**Q7: The "Local Deploy" option is missing on Windows?**
A: Local deploy mode requires Linux PTY support. On Windows, only Remote SSH deploy mode is supported — the option is hidden / disabled.

For a Windows-to-Linux simulation, use **Remote SSH** with runtime target **MuJoCo Simulation**. Remote SSH does not imply a real-robot target.

**Q8: How do I switch the robot model?**
A: Currently SONIC publishing supports G1 (29 DOF) only. The `--robot` argument of `moretion_stream_to_sonic.py` accepts `unitree_g1`, `unitree_g1_with_hands`, `unitree_h1_2`, `Q1`, `X1`, but the UI is defaulted for G1 29-DOF. Switching models requires verifying the corresponding configuration and joint-order mapping.

**Q9: The MuJoCo window doesn't close after stopping retargeting?**
A: If *Keep Retargeting Window* is checked, the window is left open for inspection. Uncheck it, and the window will auto-close when the sub-process exits.

---

> This document is derived from the `teleop_studio/` source tree and the `scripts/run_teleop_studio.*` launchers. Please keep it in sync whenever the UI or parameters change.
