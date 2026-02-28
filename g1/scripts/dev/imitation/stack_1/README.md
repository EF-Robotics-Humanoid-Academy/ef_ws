# stack\_1 — Human-to-G1 Motion Imitation

`stack_1` is a three-stage pipeline that records a human's full-body motion from an Intel RealSense RGBD camera, maps the detected pose to the Unitree G1's joint space, and reproduces the motion on the robot over DDS.

```
┌──────────────────┐      JSON       ┌──────────────┐      NPZ       ┌───────────────┐
│  vision_module   │ ──────────────► │    mapper    │ ─────────────► │   reproduce   │
│                  │                 │              │                 │               │
│  RealSense RGBD  │                 │  Geometric   │                 │  DDS LowCmd   │
│  + MediaPipe     │                 │  IK mapping  │                 │  rt/arm_sdk   │
│  33 landmarks    │                 │  29 G1 DOF   │                 │  rt/lowcmd    │
└──────────────────┘                 └──────────────┘                 └───────────────┘
```

---

## Contents

| File | Purpose | Lines |
|---|---|---|
| `vision_module.py` | Capture + record human pose landmarks from RealSense | 584 |
| `mapper.py` | Map recorded pose to G1 joint angles, save NPZ | 724 |
| `reproduce.py` | Replay NPZ on the physical G1 via DDS low-level control | 902 |

---

## Quick-start

```bash
# 1. Install dependencies (once)
pip install pyrealsense2 mediapipe opencv-python numpy

# pip install -e <path_to_unitree_sdk2_python>   # for reproduce.py

# 2. Record a 10-second human wave
python3 vision_module.py --duration 10 --output wave.json

# 3. Convert pose to G1 joint angles (23-DOF safe)
python3 mapper.py --input wave.json --output g1_wave.npz --dof23

# 4. Play back on the robot (arms + waist, balance controller active)
python3 reproduce.py --iface enp1s0 --file g1_wave.npz --dof23
```

---

## Prerequisites

### Hardware

| Component | Role |
|---|---|
| Intel RealSense D4xx (e.g. D435i) | RGBD source for `vision_module.py` |
| Unitree G1 on the same LAN | Target robot for `reproduce.py` |
| Development host | Runs all three scripts |

The host running `vision_module.py` must have **USB 3 access** to the RealSense camera. If the camera is on the robot's Jetson and the host receives the stream remotely, use the GStreamer pipeline from `sensors/manual_streaming/jetson_realsense_stream.py` and adapt `vision_module.py` to read from the GStreamer receiver instead of directly from `pyrealsense2`.

### Software

| Package | Required by |
|---|---|
| `pyrealsense2` | `vision_module.py` |
| `mediapipe` | `vision_module.py` |
| `opencv-python` | `vision_module.py` |
| `numpy` | all three |
| `unitree_sdk2py` | `reproduce.py` |

```bash
pip install pyrealsense2 mediapipe opencv-python numpy
pip install -e /path/to/unitree_sdk2_python
```

---

## Stage 1 — `vision_module.py`

### What it does

1. Opens a RealSense pipeline with depth aligned to colour (following `sensors/manual_streaming/stream_realsense.py`). Applies spatial + temporal filters to improve depth quality.
2. Runs **MediaPipe Pose** on every colour frame to detect all **33 body landmarks**.
3. For each landmark stores two independent 3-D measurements:
   - `world_xyz` — MediaPipe's own metric world coordinates (hip-centred, metres; `x=right y=up z=toward-viewer`). This is used by `mapper.py` for angle computation.
   - `depth_xyz` — pixel coordinates `(px, py)` plus `Z = depth_raw[py,px] × depth_scale` from the aligned RealSense depth frame. Used as a fallback when `world_xyz` is unavailable.
4. Records samples for `--duration` seconds then saves to disk.
5. Displays a live preview: Canny edge overlay (cyan) + colour-coded skeleton + depth panel.

### All 33 tracked landmarks

| Group | Names (MediaPipe index) |
|---|---|
| Face | nose (0), left/right eye inner/eye/eye outer (1–6), left/right ear (7–8), mouth left/right (9–10) |
| Arms & hands | left/right shoulder (11–12), elbow (13–14), wrist (15–16), pinky (17–18), index (19–20), thumb (21–22) |
| Hips | left/right hip (23–24) |
| Legs & feet | left/right knee (25–26), ankle (27–28), heel (29–30), foot index (31–32) |

### Output format

Default: **JSON** (`pose_recording.json`).
Optional: NumPy archive (`.npy` / `.npz`) — flat `float32` array with shape `[N, 1 + 33×7]` (timestamp + 7 values per landmark: `wx wy wz vis dx dy dz`).

JSON structure:
```json
{
  "format": "vision_module_v2",
  "joint_order": ["left_ankle", "left_ear", ...],
  "joint_ids":   {"left_ankle": 27, ...},
  "samples": [
    {
      "t": 0.0333,
      "joints": {
        "left_shoulder": {
          "px": 312, "py": 148,
          "vis": 0.994,
          "world_xyz": [0.152, 0.301, 0.087],
          "depth_xyz": [312.0, 148.0, 0.843]
        }
      }
    }
  ]
}
```

### CLI reference

```
python3 vision_module.py [OPTIONS]

  --width      INT    RealSense stream width          (default: 640)
  --height     INT    RealSense stream height         (default: 480)
  --fps        INT    Frame rate                      (default: 30)
  --duration   FLOAT  Recording length in seconds     (default: 5.0)
  --output     PATH   Output file (.json / .npy/.npz) (default: pose_recording.json)
  --no-display        Disable OpenCV preview window (headless / SSH)
```

---

## Stage 2 — `mapper.py`

### What it does

Converts the JSON pose recording into a G1-compatible NPZ trajectory by computing joint angles geometrically from 3-D landmark positions.

For each frame:

1. **Extracts 3-D positions** — uses `world_xyz` (preferred) or `depth_xyz` as fallback.
2. **Builds reference frames** from the landmark cloud:
   - *Torso frame*: origin = shoulder midpoint; `x=body-right`, `y=body-forward`, `z=body-up` (spine axis).
   - *Pelvis frame*: same convention anchored to the hip midpoint.
3. **Computes joint angles** by expressing each limb segment in its parent frame and decomposing using `atan2` projections:

| G1 joint | Computation method |
|---|---|
| ShoulderPitch | `atan2(v_fwd, -v_down)` of upper-arm vector in torso frame |
| ShoulderRoll | `atan2(±v_lat, -v_down)` of upper-arm vector |
| ShoulderYaw | Forearm deviation from the natural elbow-drop plane (cross-product sign) |
| Elbow | `π − interior_angle_at_elbow` (0 = extended, π/2 = 90° bent) |
| WristRoll | Index-to-pinky span projected perpendicular to forearm axis |
| WristPitch | Hand-index deviation from forearm direction |
| WaistYaw | Signed horizontal rotation between shoulder line and hip line |
| WaistPitch | Forward tilt of spine vector |
| WaistRoll | Lateral tilt of spine vector |
| HipPitch/Roll/Yaw | Thigh vector decomposed in pelvis frame |
| Knee | `π − interior_angle_at_knee` |
| AnklePitch/Roll | Shin + foot vectors in pelvis frame |

4. **Clamps** every angle to the per-joint safe range (see Joint Limits table).
5. Writes a **pbd\_reproduce.py-compatible NPZ**.

> **Accuracy note.** ShoulderYaw, WristPitch, and WristYaw are inherently hard to estimate from an RGB-D skeleton (they require hand-pose or forearm IMU). The mapper produces reasonable approximations; expect ±10–20° error on those DOFs. All others are geometrically well-constrained.

### Output NPZ keys

| Key | dtype | Shape | Description |
|---|---|---|---|
| `joints` | `int32` | `[J]` | G1 joint indices in column order |
| `ts` | `float32` | `[N]` | Timestamps (seconds from recording start) |
| `qs` | `float32` | `[N, J]` | Joint angles in radians |
| `fk_qs` | `float32` | `[N, J]` | Copy of `qs` (replay-mode compatibility) |
| `poll_s` | `float32` | scalar | Mean inter-sample interval |
| `representation` | str | — | `"joint_space"` |

This file is **directly loadable by `pbd_reproduce.py`** using `--mode joint`.

### CLI reference

```
python3 mapper.py [OPTIONS]

  --input    PATH  pose_recording.json from vision_module.py  (default: pose_recording.json)
  --output   PATH  Output NPZ for reproduce.py / pbd_reproduce.py (default: g1_motion.npz)
  --no-legs        Skip all leg joints (0–11); arm + waist only
  --no-waist       Skip waist joints (12–14)
  --dof23          23-DOF safe mode: skip WaistRoll (13), WaistPitch (14),
                   LeftWristPitch (20), LeftWristYaw (21),
                   RightWristPitch (27), RightWristYaw (28)
  --verbose        Print per-frame angle summaries to stdout
```

---

## Stage 3 — `reproduce.py`

### What it does

Loads the mapper NPZ and replays the joint trajectory on the physical G1 over Unitree SDK2 DDS.

### Control channels

Two modes, selectable at runtime:

#### Default — arms + waist (`rt/arm_sdk`, safe)

The standard safe path used by all PBD scripts in this repository.

- **Topic**: `rt/arm_sdk`
- **Joints commanded**: 12–28 (waist + both arms, 17 DOF)
- **Rate**: 50 Hz (`control_dt = 0.02 s`)
- **Gains**: `kp = 60`, `kd = 1.5` (configurable)
- **Balance controller**: remains active; the G1 continues to manage its own legs
- **Enable**: `motor_cmd[29].q = 1`; release: `motor_cmd[29].q = 0`

The robot can stand freely during this mode. The human motion is reproduced in the upper body only.

#### Full-body — all joints (`rt/lowcmd`, hanger required)

For experiments that include leg motion.

- **Topic**: `rt/lowcmd`
- **Joints commanded**: 0–28 (all 29 DOF)
- **Rate**: 500 Hz (`control_dt = 0.002 s`)
- **Gains**: legs `kp=120 kd=2.0`, arms/waist `kp=60 kd=1.5` (all configurable)
- **Balance controller**: released via `MotionSwitcherClient.ReleaseMode()` before playback, restored on exit
- **Requires**: robot hanging from a hanger frame — it **will not** balance itself

### Stability layer

Every control tick applies the following checks in order:

| Check | Mechanism |
|---|---|
| **LowState seed** | Waits up to `--seed-wait` seconds for `rt/lowstate`; primes all initial commanded positions from actual current joint angles. No jump on start. |
| **Soft ramp-in** | Linearly interpolates from current robot pose to the first trajectory frame over `--start-ramp` seconds (default 1.5 s). |
| **Velocity limiter** | Per-joint `δq` is clamped so angular velocity ≤ `MAX_VEL` (rad/s). Conservative defaults: arms 3 rad/s, hip 2.5 rad/s, knee 3 rad/s, ankle 1.5–2.0 rad/s. |
| **IMU watchdog** | Reads `imu_state.rpy` from `rt/lowstate` each tick. Triggers immediate e-stop + arm\_sdk disable if `|roll|` or `|pitch| > --imu-limit` (default 0.35 rad ≈ 20°). |
| **Joint re-clamp** | All commanded angles are re-clamped to the same ranges as `mapper.py` before every DDS write. |
| **Foot-contact gate** | In `--with-legs` mode: reads `SportModeState.mode` via `rt/odommodestate`. If `mode == 2` (feet unloaded), leg joint commands are frozen at the last safe pose. Arms continue playing. |

### Shutdown behaviour

| Mode | On exit / Ctrl-C |
|---|---|
| Arms only | `motor_cmd[29].q = 0` — arm\_sdk released cleanly |
| Full-body | `MotionSwitcherClient.SelectMode(original_mode)` — balance controller restored |

### CLI reference

```
python3 reproduce.py [OPTIONS]

  --iface      STR    DDS network interface (e.g. enp1s0, eth0) (default: enp1s0)
  --file       PATH   Input NPZ from mapper.py                   (default: g1_motion.npz)

  Motion subset
  --with-legs         Include leg joints via rt/lowcmd (HANGER REQUIRED)
  --no-waist          Skip waist joints (12–14)
  --dof23             23-DOF safe: skip WaistRoll/Pitch + WristPitch/Yaw

  Timing
  --speed      FLOAT  Playback speed multiplier (0.5 = half, 2.0 = double) (default: 1.0)
  --start-ramp FLOAT  Ramp-in duration in seconds                          (default: 1.5)
  --seed-wait  FLOAT  Seconds to wait for initial rt/lowstate              (default: 2.0)
  --cmd-hz     FLOAT  Command rate Hz (overridden to 500 in --with-legs)   (default: 50.0)

  Gains
  --arm-kp     FLOAT  Position stiffness for arms/waist   (default: 60.0)
  --arm-kd     FLOAT  Damping for arms/waist              (default: 1.5)
  --leg-kp     FLOAT  Position stiffness for legs         (default: 120.0)
  --leg-kd     FLOAT  Damping for legs                    (default: 2.0)

  Safety
  --imu-limit  FLOAT  IMU roll/pitch e-stop threshold, radians (default: 0.35 ≈ 20°)
```

---

## Full-pipeline workflows

### Workflow A — upper-body imitation (default, safe)

Suitable for waving, gesturing, arm-reach demonstrations. Robot stands freely.

```bash
# Step 1: record (stand 1–3 m in front of the camera)
python3 vision_module.py --duration 10 --output demo.json

# Step 2: map (23-DOF safe, skip legs)
python3 mapper.py --input demo.json --output demo.npz --dof23 --no-legs

# Step 3: reproduce on robot
python3 reproduce.py --iface enp1s0 --file demo.npz --dof23 --no-legs
```

### Workflow B — upper-body with waist rotation (29-DOF waist model)

Only for robots with an unlocked 3-DOF waist.

```bash
python3 mapper.py  --input demo.json --output demo_waist.npz
python3 reproduce.py --iface enp1s0 --file demo_waist.npz
```

### Workflow C — full-body motion on hanger

For lower-body gestures (squats, lunges, etc.) where the robot is secured.

```bash
# Map with legs included
python3 mapper.py --input demo.json --output demo_full.npz

# Reproduce — will prompt for hanger confirmation
python3 reproduce.py --iface enp1s0 --file demo_full.npz --with-legs --speed 0.5
```

### Workflow D — inspect then replay with pbd\_reproduce.py

The NPZ produced by `mapper.py` is also directly compatible with the existing `pbd_reproduce.py` script if you only need arm joints.

```bash
python3 ../../arm_motion/pbd/pbd_reproduce.py \
    --iface enp1s0 \
    --file demo.npz \
    --arm both \
    --mode joint \
    --speed 0.8
```

---

## G1 joint reference

```
Index  Name                  Group   Limits (rad)      Notes
─────────────────────────────────────────────────────────────────────
  0    LeftHipPitch          leg     −1.57  +1.57
  1    LeftHipRoll           leg     −0.52  +0.52
  2    LeftHipYaw            leg     −0.52  +0.52
  3    LeftKnee              leg      0.00  +2.30
  4    LeftAnklePitch        leg     −0.52  +0.52
  5    LeftAnkleRoll         leg     −0.30  +0.30
  6    RightHipPitch         leg     −1.57  +1.57
  7    RightHipRoll          leg     −0.52  +0.52
  8    RightHipYaw           leg     −0.52  +0.52
  9    RightKnee             leg      0.00  +2.30
 10    RightAnklePitch       leg     −0.52  +0.52
 11    RightAnkleRoll        leg     −0.30  +0.30
 12    WaistYaw              waist   −1.00  +1.00
 13    WaistRoll             waist   −0.35  +0.35   INVALID on 23-DOF
 14    WaistPitch            waist   −0.52  +0.52   INVALID on 23-DOF
 15    LeftShoulderPitch     arm     −1.57  +2.40
 16    LeftShoulderRoll      arm     −0.20  +2.40
 17    LeftShoulderYaw       arm     −1.40  +1.40
 18    LeftElbow             arm      0.00  +2.40
 19    LeftWristRoll         arm     −1.40  +1.40
 20    LeftWristPitch        arm     −0.70  +0.70   INVALID on 23-DOF
 21    LeftWristYaw          arm     −0.44  +0.44   INVALID on 23-DOF
 22    RightShoulderPitch    arm     −1.57  +2.40
 23    RightShoulderRoll     arm     −2.40  +0.20
 24    RightShoulderYaw      arm     −1.40  +1.40
 25    RightElbow            arm      0.00  +2.40
 26    RightWristRoll        arm     −1.40  +1.40
 27    RightWristPitch       arm     −0.70  +0.70   INVALID on 23-DOF
 28    RightWristYaw         arm     −0.44  +0.44   INVALID on 23-DOF
 29    (ArmSdkEnable)        —        —      —      motor_cmd[29].q=1 enables rt/arm_sdk
```

Use `--dof23` in both `mapper.py` and `reproduce.py` to automatically exclude all INVALID joints.

---

## Safety

> **Read before running `reproduce.py` on hardware.**

### General rules (from `arm_motion/sdk_details.md`)

- Always keep a **spotter** present and maintain a **clear workspace** around the robot.
- Never run `reproduce.py` until the robot is **in a stable balanced stand** (FSM 200) or is **secured on a hanger**.
- The script waits for a valid `rt/lowstate` before sending any command. If no state is received within `--seed-wait` seconds it continues with unseeded (zero) positions — verify the network interface with `ip link`.
- Every first command is a **smooth ramp** from the robot's actual current joint angles. Never skip or shorten `--start-ramp`.

### IMU watchdog

The IMU watchdog reads `roll` and `pitch` from `rt/lowstate` every control tick. If either exceeds `--imu-limit` (default 0.35 rad ≈ 20°) the script:

1. Prints an e-stop message with the offending angles.
2. Immediately disables `arm_sdk` (`motor_cmd[29].q = 0`) or stops commanding `lowcmd`.
3. Exits the playback loop.

Reduce `--imu-limit` for a more sensitive watchdog; 0.25 rad (≈14°) is a good starting point for cautious testing.

### `--with-legs` safety checklist

- [ ] Robot is hanging from a secure hanger frame with feet clear of the ground.
- [ ] Workspace is clear of people and obstacles.
- [ ] You have typed `YES` at the confirmation prompt.
- [ ] You are prepared to hit **Ctrl-C** or the hardware emergency stop at any time.
- [ ] `--speed` is set to ≤ 0.5 for the first test.
- [ ] `--imu-limit` is set to a tight value (e.g. 0.25) so any unexpected tilt stops the replay.

### Gain tuning guidance

| Scenario | Recommendation |
|---|---|
| First run, unfamiliar motion | `--arm-kp 30 --arm-kd 1.0 --speed 0.3` |
| Normal wave / gesture | `--arm-kp 60 --arm-kd 1.5 --speed 1.0` (defaults) |
| Fast dynamic motion | `--arm-kp 80 --arm-kd 2.0` — monitor for oscillation |
| Leg motion on hanger | `--leg-kp 100 --leg-kd 2.0 --speed 0.4` for first test |

---

## Known limitations

| Limitation | Detail |
|---|---|
| **ShoulderYaw accuracy** | Estimated from forearm-deviation heuristic. ±10–20° typical error. Requires IMU on the forearm for precision. |
| **WristPitch / WristYaw** | Estimated from finger landmark positions; highly sensitive to hand occlusion. Use `--dof23` to skip. |
| **Depth at occluded landmarks** | MediaPipe may report landmarks outside the camera frustum. `depth_xyz` falls back to `None`; `mapper.py` uses `world_xyz` where possible. |
| **Single person only** | MediaPipe Pose detects one body. Multi-person disambiguation is not implemented. |
| **Leg imitation fidelity** | Human leg DOF and link lengths differ significantly from the G1. Hip and knee angles are geometrically correct but absolute end-effector positions will differ. |
| **No feedforward torque** | All commands are position-PD only (`tau = 0`). For high-speed or loaded motions this may cause lag. |
| **No online loop-closure** | There is no feedback from the robot's actual joint positions back into the mapper. Accumulated drift is not corrected mid-motion. |

---

## File relationships

```
stack_1/
├── vision_module.py          # Stage 1: capture
├── mapper.py                 # Stage 2: map
├── reproduce.py              # Stage 3: replay
└── README.md                 # This file

Related project files:
  ../../../sensors/manual_streaming/stream_realsense.py   # RealSense patterns used in vision_module
  ../../../sensors/manual_streaming/sdk_details.md        # RGBD pipeline documentation
  ../../arm_motion/g1_arm7_sdk_dds_example.py             # G1 joint indices + arm_sdk pattern
  ../../arm_motion/sdk_details.md                         # Low-level DDS documentation
  ../../arm_motion/pbd/pbd_demonstrate.py                 # PBD recording reference
  ../../arm_motion/pbd/pbd_reproduce.py                   # NPZ replay reference
  ../../arm_motion/pbd/pbd_docs.md                        # NPZ format specification
  ../../arm_motion/safety/hanger_boot_sequence.py         # Safe boot helper used by reproduce.py
```
