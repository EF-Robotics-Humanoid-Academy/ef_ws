# ef_ws

EF workspace for Unitree robots (G1, GO2) with scripts, simulation assets, and supporting docs.

**Repository layout**
- `g1/`: G1 scripts, docs, and MuJoCo assets.
- `go2/`: GO2 scripts, maps, and MuJoCo assets.
- `config_ubuntu.bash`: Full provisioning script for Ubuntu hosts.
- `conf_raspi.bash`: Provisioning variant for Raspberry Pi.
- `links.md`: External repos referenced by the setup scripts.

**Quick start (provisioning script)**
If you want the fastest, repeatable setup on Ubuntu, use the provisioning script and provide Unitree repo URLs:

```bash
export UNITREE_SDK2_REPO=https://.../unitree_sdk2.git
export UNITREE_SDK_PY_REPO=https://.../unitree_sdk2_python.git
sudo bash config_ubuntu.bash
```

---

**Environment setup checklist (core)**
This checklist covers the core development environment, CycloneDDS build, and the Unitree Python SDK in a `uv` Python 3.10 venv.

- [ ] System packages
  - `sudo apt update`
  - `sudo apt install -y build-essential cmake ninja-build pkg-config git curl ca-certificates python3-dev mesa-utils libgl1-mesa-dev libssl-dev tmux`

- [ ] Install `uv` (Astral)
  - `curl -LsSf https://astral.sh/uv/install.sh | sh`
  - `uv --version`

- [ ] Create a Python 3.10 environment (recommended path used by scripts)
  - `uv venv --python 3.10 ~/.vens/python310`

- [ ] Build and install CycloneDDS
  - `git clone https://github.com/eclipse-cyclonedds/cyclonedds.git ~/cyclonedds`
  - `cd ~/cyclonedds`
  - `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/cyclonedds/install`
  - `cmake --build build`
  - `cmake --install build`

- [ ] Build and install Unitree SDK2 (C/C++)
  - `git clone <UNITREE_SDK2_REPO> ~/unitree_sdk2`
  - `cd ~/unitree_sdk2`
  - `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=~/unitree_sdk2/install`
  - `cmake --build build`
  - `cmake --install build`

- [ ] Install Unitree Python SDK wrapper into the `uv` env
  - `git clone <UNITREE_SDK_PY_REPO> ~/unitree_sdk_python`
  - `uv pip install --python ~/.vens/python310/bin/python -e ~/unitree_sdk_python`

---

**Environment setup checklist (Livox dependencies)**
Use this if you plan to run LiDAR-related scripts (Livox MID-360, SLAM, point-cloud viewers).

- [ ] Build and install Livox-SDK2 (required by `livox2_python.py` and live LiDAR tools)
  - `git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2`
  - `cd ~/Livox-SDK2 && mkdir -p build && cd build`
  - `cmake .. -DCMAKE_BUILD_TYPE=Release`
  - `make -j$(nproc)`
  - `sudo make install`
  - `sudo ldconfig`

- [ ] Optional: Build and install Livox-SDK (SDK1 fallback)
  - `git clone https://github.com/Livox-SDK/Livox-SDK.git ~/Livox-SDK`
  - `cd ~/Livox-SDK && mkdir -p build && cd build`
  - `cmake .. -DCMAKE_BUILD_TYPE=Release`
  - `make -j$(nproc)`
  - `sudo make install`
  - `sudo ldconfig`

- [ ] Install OpenPyLivox (Python helper)
  - `git clone https://github.com/Livox-SDK/openpylivox.git ~/openpylivox`
  - `uv pip install --python ~/.vens/python310/bin/python -e ~/openpylivox`

---

**Environment setup checklist (MuJoCo dependencies)**
Use this if you plan to run the MuJoCo simulators under `g1/sim/mujoco` or `go2/sim/mujoco`.

- [ ] Ensure OpenGL / X11 dependencies are installed
  - `sudo apt install -y mesa-utils libgl1-mesa-dev`

- [ ] Install MuJoCo Python package in the `uv` env
  - `uv pip install --python ~/.vens/python310/bin/python mujoco`

---

**Basic examples (from commands.txt files)**
These examples are pulled directly from the command snippets in this repo. Update `--iface` and any absolute paths to match your machine.

**G1 gait measure**
Source: `g1/scripts/basic/commands.txt`

```bash
python3 /home/ag/academy/academy_content/ef_ws/g1/scripts/g1_hl_gait_measure.py \
  --iface enp1s0 \
  --vx 0.25 \
  --target-m 1.0 \
  --duration 20.0
```

**G1 motion sequence**
Source: `g1/scripts/basic/commands.txt`

```bash
python3 /home/ag/academy/academy_content/ef_ws/g1/scripts/g1_hl_motion_sequence.py \
  --iface enp1s0 \
  --walk-m 1.0 \
  --walk-v 0.3 \
  --turn-deg 180 \
  --turn-vyaw 0.8 \
  --ramp-time 0.4
```

**G1 polling / troubleshooting**
Source: `g1/scripts/troubleshooting/command.txt`

```bash
py g1_poll_all.py --config g1_topics.example.yaml --profile g1_basic,g1_odom,g1_lidar --iface enp1s0
py g1_poll_all.py --config g1_topics.example.yaml --profile g1_basic,g1_odom,g1_lidar --iface enp1s0 --poll 5 --df-rows 10
```

---

**Docs and references**
- G1 docs: `g1/docs/index.md`
- Livox cheat sheet: `g1/docs/lidar_cheatsheet.html`
- Livox deep dive: `g1/docs/lidar_docs.html`
- Quick start: `g1/docs/quick_start.md`
- External links: `links.md`
