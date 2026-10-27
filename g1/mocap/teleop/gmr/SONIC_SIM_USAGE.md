# SONIC MuJoCo Simulator Package

Chinese documentation: [README.zh-CN.md](README.zh-CN.md)

## 1. Package Contents

This archive contains a relocatable Ubuntu x86_64 SONIC MuJoCo simulation environment with:

- A relocatable Python 3.11 runtime
- MuJoCo and the G1 43-DoF scene model
- The SONIC simulation loop, CycloneDDS 0.10.2, and Unitree DDS message definitions
- The `run_sonic_sim.sh` launcher

The archive does not include `gear_sonic_deploy`, WBC policy files, the GMR retargeting environment, or real-robot deployment software. The simulator communicates with a separately deployed SONIC WBC process over CycloneDDS.

## 2. System Requirements

- Ubuntu Desktop 20.04 or newer
- x86_64 CPU
- glibc 2.28 or newer
- A working X11/Wayland session and OpenGL driver for onscreen rendering
- Use `--no-onscreen` on a headless server

Check the architecture and glibc version:

```bash
uname -m
getconf GNU_LIBC_VERSION
```

## 3. Verify and Extract

Place the archive and its `.sha256` file in the same directory:

```bash
sha256sum -c SonicSim-linux-x86_64-<version>.tar.gz.sha256
mkdir -p SonicSim
tar -xzf SonicSim-linux-x86_64-<version>.tar.gz -C SonicSim
cd SonicSim
chmod +x run_sonic_sim.sh
```

Extract into an empty directory. On the first launch, the launcher runs `conda-unpack` once to repair paths after relocation.

## 4. Start the Simulator

Start with the MuJoCo window on Ubuntu Desktop:

```bash
./run_sonic_sim.sh
```

Start in headless mode:

```bash
./run_sonic_sim.sh --no-onscreen
```

Display all options:

```bash
./run_sonic_sim.sh --help
```

Press `Ctrl+C` to stop the simulator.

## 5. Connect SONIC WBC

For a same-host setup, run both the simulator and WBC over the loopback interface:

```bash
# Terminal 1: simulator
./run_sonic_sim.sh --interface sim

# Terminal 2: WBC from a separately installed gear_sonic_deploy directory
cd /path/to/gear_sonic_deploy
bash deploy.sh --input-type zmq \
  --zmq-host localhost --zmq-port 5556 --zmq-topic pose sim
```

`gear_sonic_deploy` and its policy files must be provided separately. Waiting for DDS data is expected when no WBC process is publishing commands.

For a cross-host setup, select a local interface name or IP address reachable from the WBC host:

```bash
./run_sonic_sim.sh --interface enp3s0
# or
./run_sonic_sim.sh --interface 192.168.1.20
```

The simulator and WBC must use the same DDS Domain ID and mutually reachable interfaces. Allow CycloneDDS UDP unicast and multicast traffic through the firewall. Do not pass `--interface real` to the simulator.

## 6. Common Options

| Option | Default | Description |
| --- | --- | --- |
| `--interface sim` | `sim` | Use the local loopback interface |
| `--interface <name-or-IP>` | | Use a reachable interface for cross-host DDS |
| `--sim-frequency 200` | `200` | MuJoCo simulation frequency in Hz |
| `--control-frequency 50` | `50` | Control frequency in Hz |
| `--no-onscreen` | disabled | Disable the desktop viewer |
| `--offscreen` | disabled | Enable offscreen rendering |
| `--enable-image-publish` | disabled | Publish camera images over ZMQ; requires `--offscreen` |
| `--camera-port 5555` | `5555` | Camera image publication port |

## 7. Troubleshooting

- **No display session warning**: run inside an Ubuntu Desktop session or add `--no-onscreen`.
- **DDS peer not found**: check the interface, Domain ID, subnet, firewall, and CycloneDDS versions on both hosts.
- **glibc requirement failure**: use Ubuntu 20.04 or newer and do not replace the bundled Python environment.
- **Viewer opens but the robot does not move**: verify that `gear_sonic_deploy` is running and has entered control mode. Waiting is normal without WBC commands.
- **First launch is slow**: `conda-unpack` runs once on the first launch and is skipped afterward.
