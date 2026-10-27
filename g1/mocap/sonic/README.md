# G1-EDU Moretion / GMR / SONIC Deployment

## 1. Runtime boundary

This package runs the SONIC controller either on the G1-EDU onboard Orin or on
an x86_64 Ubuntu computer. On x86_64, the same controller can connect either to
a real G1 or to an externally provided Unitree DDS simulator. Moretion parsing
and GMR retargeting remain on the operator side.

```text
Moretion motion-capture server
  -> TCP/UDP :9999
Operator computer: GMR/scripts/moretion_stream_to_sonic.py
  -> GMR IK retargeting to G1 29-DOF in IsaacLab joint order
  -> ZMQ PUB :5556, topic=pose, 1280-byte header, Protocol v1
G1 controller package (onboard aarch64 or Ubuntu x86_64)
  -> ZMQ SUB -> SONIC TensorRT encoder/decoder -> Unitree DDS
  -> real G1 motors (`real`) or external simulator (`sim` on x86_64)
```

Keep these components on the operator computer: `general_motion_retargeting/`,
`assets/unitree_g1/`, `scripts/moretion_stream_to_sonic.py`, and the associated
Python environment. They are intentionally not included in the onboard package.

The universal package contains the minimal SONIC controller source, both the
default `release` and `low_latency` encoder/decoder ONNX model pairs, their
matching observation configurations, aarch64 and x86_64 Unitree SDK2 libraries and
headers, build and launch scripts, and one small seed motion. The seed motion is
required by the current controller constructor and is replaced by the live ZMQ
motion after streaming is enabled.

The package excludes training code, Isaac Lab, the external MuJoCo simulator,
visualization, ROS 2,
the motion planner, MotionBricks, Decoupled WBC, data processing, tests, and
TensorRT `.trt` caches. TensorRT engines
are tied to the GPU, TensorRT version, and model. They must be generated from ONNX
on the deployment computer.

## 2. Platform prerequisites

### G1-EDU onboard aarch64

- Unitree G1-EDU onboard Orin running aarch64 Linux.
- JetPack 6.
- TensorRT **10.7**. This is the version validated by the upstream deployment
  documentation; another version can produce incorrect motion output.
- A Unitree DDS network interface configured on the `192.168.123.x` subnet.
- A second interface or Wi-Fi route from the onboard computer to the GMR operator
  computer.
- Internet access and `sudo` for the dependency installation step.
- The robot suspended for the first hardware run, with an operator ready to use
  the physical emergency stop.

### Ubuntu x86_64

- Ubuntu 20.04, 22.04, or 24.04 on x86_64.
- An NVIDIA GPU and a compatible CUDA 12 installation.
- TensorRT **10.13**, installed separately and exposed with `TensorRT_ROOT` when
  it is not in a system search path.
- For `real`, a wired interface on the `192.168.123.x` Unitree DDS subnet.
- Network reachability to the GMR ZMQ publisher. GMR and SONIC may run on the same
  x86_64 computer, in which case use `localhost`.
- For `sim`, an external simulator that implements the Unitree SDK2 DDS
  LowCmd/LowState loop on the same DDS domain. It is intentionally not bundled.
- Internet access and `sudo` for dependency installation.
- The same suspension and physical emergency-stop precautions as onboard use.

## 3. Transfer and unpack

From the complete GR00T repository, generate the universal package:

```bash
cd gear_sonic_deploy
bash onboard/package_g1_edu.sh

# Transfer the same archive to either deployment platform
scp dist/g1-edu-sonic-moretion.tar.gz \
  unitree@G1_IP:/home/unitree/
```

`g1-edu-sonic-moretion.tar.gz` supports both the G1-EDU Orin and Ubuntu x86_64.
It contains one copy of the common source, models, reference data, and documentation,
plus the relatively small architecture-specific Unitree SDK library directories.

On the selected deployment computer, unpack the same universal archive. For example:

```bash
cd /home/unitree
tar -xzf g1-edu-sonic-moretion.tar.gz
cd g1-edu-sonic-moretion
sha256sum -c SHA256SUMS
```

The packaging script uses an explicit file allowlist and rejects ONNX files that
are still Git LFS pointers.

## 4. Install, verify, and build

Run these commands on the selected deployment computer:

```bash
./install_onboard_deps.sh
./check_onboard.sh
./build_onboard.sh
```

The scripts detect `aarch64` or `x86_64` with `uname -m`. `install_onboard_deps.sh`
installs the required C++ development packages and
downloads the matching ONNX Runtime 1.16.3 build (`aarch64` or `x64`) into the
package. It does not install or modify JetPack, CUDA, or TensorRT.

`check_onboard.sh` verifies that the host is a supported architecture. It requires
Jetson plus TensorRT 10.7 for aarch64, or Ubuntu plus TensorRT 10.13 for x86_64,
and checks both model profiles, Unitree SDK libraries, and system headers. Do not
bypass a TensorRT version failure for physical robot operation.

`build_onboard.sh` creates `bin/g1_deploy_onnx_ref`. The first controller launch
builds TensorRT engines from the selected ONNX models and can take considerably
longer than later launches. Do not enter motor control while engine generation is
in progress.

## 5. Start the GMR publisher

On the operator computer, start the Moretion-to-SONIC bridge. Bind the publisher
to all interfaces so the onboard computer can connect:

```bash
python scripts/moretion_stream_to_sonic.py \
  --server_ip MORETION_IP \
  --port 9999 \
  --sonic_host '*' \
  --sonic_port 5556
```

Allow inbound TCP port 5556 through the operator computer firewall. The onboard
ZMQ subscriber connects to the operator computer's publisher, so test network
reachability in that direction.

Match the sender profile to the deployed model: use `--model_profile default`
with `release`, or `--model_profile low_latency` with `low_latency`. Omitting
`--preview_delay_ms` preserves full measured preview (920 / 200 ms at 50 Hz).
Teleop Studio defaults to compromise preview (400 / 100 ms). Sender packets
include `live_frame_index`, one past frame, the target, and the full reference
window; the controller aligns to the target rather than replaying startup backlog.
Pass `--zmq-conflate` to `deploy.sh` (already enabled by Teleop Studio) to consume
the latest pose message. Rebuild the controller from this package to enable
target-frame alignment; replacing the Python sender alone is insufficient.

## 6. Select simulation or real hardware

The final argument accepted by `deploy.sh` is a runtime target, not a build
target. Both modes execute the same `g1_deploy_onnx_ref` binary:

```bash
# External simulator must already be running on the loopback DDS interface.
./deploy.sh --input-type zmq --zmq-host localhost --zmq-port 5556 sim

# Real G1; the DDS interface is auto-detected from 192.168.123.x.
./deploy.sh --input-type zmq --zmq-host 192.168.3.10 --zmq-port 5556 real
```

`sim` selects `lo` and adds `--disable-crc-check`, because the external simulator
does not produce the hardware CRC. `real` selects the G1 network interface and
never adds that flag. The package does not start, install, or constrain the
external simulator; it only supplies the controller side of the DDS loop.

Simulation is enabled only on x86_64. The aarch64 package path remains dedicated
to onboard real-robot control.

## 7. Manual hardware launch

The first argument is the Unitree DDS interface. The second is the operator
computer IP that hosts the ZMQ publisher.

Default SONIC release model:

```bash
./run_onboard.sh eth0 192.168.3.10 5556 release
```

Low-latency model:

```bash
./run_onboard.sh eth0 192.168.3.10 5556 low_latency
```

After the controller has initialized:

1. Keep the G1 suspended and wait for LowState plus encoder/decoder initialization.
2. Press `]` to enter motor control.
3. Verify the default posture and motor state, then press `Enter` to enable live
   ZMQ motion tracking.
4. Press `O` for the software emergency stop if anything is abnormal, and use the
   physical emergency stop when required.

Never pass `--disable-crc-check` on hardware. That option is only for MuJoCo.

## 8. Teleop Studio deployment

In the GMR Teleop Studio UI, configure:

- Deployment mode: **Remote SSH**.
- Runtime target: **Unitree G1 Robot**.
- Deployment directory: `/home/unitree/g1-edu-sonic-moretion`.
- Model: **Default Model** or **Low-latency Model**.
- SONIC ZMQ host: the operator computer IP reachable from the onboard computer.
- SONIC ZMQ port: `5556` unless intentionally changed on both sides.

For x86_64, **Local Deploy** can be used when Teleop Studio and SONIC run on the
same Ubuntu computer. Select **MuJoCo Simulation** when an external simulator is
running, or **Unitree G1 Robot** for hardware. Set the extracted package as the
deployment directory and use `localhost` as the ZMQ host. Remote SSH supports the
same target selection on a separate x86_64 control computer.

The packaged `deploy.sh` accepts the command generated by Teleop Studio. The
default model maps to `policy/release`; `--cp policy/low_latency/model` maps to
`policy/low_latency`. Each profile always uses its matching observation config.

`deploy.sh` automatically selects the interface with a `192.168.123.x` address.
To select it explicitly in the onboard shell environment:

```bash
export ROBOT_INTERFACE=eth0
```

Teleop Studio uses an SSH pseudo-terminal to answer the
`Proceed with deployment?` prompt and inject `]`, `Enter`, and `O`. The standalone
`zmq` input mode subscribes to the `pose` topic. Although the GMR publisher also
sends a `command` topic, motor-control and tracking transitions remain controlled
by these terminal keys or by Teleop Studio.

## 9. Network and troubleshooting

- `robot_network_interface` is the Unitree DDS interface, normally on
  `192.168.123.x`.
- Simulation uses the loopback interface `lo`; start the external DDS simulator
  before launching the controller.
- `gmr_operator_ip` is the operator computer address reachable from the onboard
  computer. It is not the Unitree DDS address.
- If no pose data arrives, verify that the GMR publisher uses `--sonic_host '*'`,
  TCP 5556 is open, and the operator IP is reachable from the G1.
- If preflight reports a small ONNX file, the source repository contained a Git
  LFS pointer. Run `git lfs pull` and rebuild the package.
- Do not copy `.trt` files generated on another computer or with another TensorRT
  version into either policy directory.

Physical robot deployment is inherently hazardous. Keep a trained operator in
control of the physical emergency stop and maintain a clear workspace.
