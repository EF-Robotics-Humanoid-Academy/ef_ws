# Day 4: Perception and Project Challenge

## Day outline
- [Perception & Projekt-Challenge](#perception--projekt-challenge)
- [Einführung in Wahrnehmung & Objekterkennung](#einführung-in-wahrnehmung--objekterkennung)
- [Kleine Teams: Mini-Applikation umsetzen](#kleine-teams-mini-applikation-umsetzen)
- [Projekt-Präsentationen & Feedback](#projekt-präsentationen--feedback)
- [Zertifikatsvergabe & nächster Schritt](#zertifikatsvergabe--nächster-schritt)

---

## Perception & Projekt-Challenge
![G1 arm and hand](https://doc-cdn.unitree.com/static/2025/1/24/c5b9464015cd4ee49314f634ebf2b3d0_1540x998.png)

![Project stack](docs/generated_charts/day_4_chart_01.png)

---

## Einführung in Wahrnehmung & Objekterkennung
![Perception pipeline](docs/generated_charts/day_4_chart_02.png)

![Evaluation metrics](docs/generated_charts/day_4_chart_03.png)

---

## Kleine Teams: Mini-Applikation umsetzen
![Arm control paths](docs/generated_charts/day_4_chart_04.png)

![Upper limb indices (rt/arm_sdk)](docs/generated_charts/day_4_chart_05.png)

![RobotStateClient service switch](docs/generated_charts/day_4_chart_06.png)

**Rule of thumb**
- Disable `g1_arm_example` before sending custom `rt/arm_sdk` targets.
- Keep `ai_sport` running for locomotion unless you fully take over low-level control.

### ArmAction RPC (gesture library)
![Common action IDs](docs/generated_charts/day_4_chart_07.png)

```python
from unitree_sdk2py.rpc import ArmActionClient

arm_action = ArmActionClient()
arm_action.Init()
arm_action.DoAction(26)  # wave high
```

```bash
python3 example/g1/high_level/g1_arm5_sdk_dds_example.py
python3 example/g1/high_level/g1_arm7_sdk_dds_example.py
```

### Dexterous hands (Dex3-1, Inspire, BrainCo)
![Dex3-1 DDS topics](docs/generated_charts/day_4_chart_08.png)

![Dex3-1 hand motor order](docs/generated_charts/day_4_chart_09.png)

![Inspire/DFX hand wiring](docs/generated_charts/day_4_chart_10.png)

![BrainCo hand topics](docs/generated_charts/day_4_chart_11.png)

![BrainCo finger order](docs/generated_charts/day_4_chart_12.png)

**BrainCo notes**
- Finger position and speed are normalized to [0, 1].
- Recommended to set finger speed to 1.0.

### Advanced custom example: pick and place (known pose)
![Pick and place pipeline](docs/generated_charts/day_4_chart_13.png)

```python
from unitree_sdk2py.arm import ArmSdk  # Replace with actual SDK module

arm = ArmSdk(side="right")

pick_pose = {"x": 0.45, "y": -0.10, "z": 0.35, "roll": 0.0, "pitch": 1.57, "yaw": 0.0}
place_pose = {"x": 0.30, "y": 0.20, "z": 0.40, "roll": 0.0, "pitch": 1.57, "yaw": 0.0}

arm.MoveToPose(pick_pose)
arm.CloseGripper()
arm.MoveToPose(place_pose)
arm.OpenGripper()
```

---

## Projekt-Präsentationen & Feedback
![Demo checklist](docs/generated_charts/day_4_chart_14.png)

![Feedback focus](docs/generated_charts/day_4_chart_15.png)

---

## Zertifikatsvergabe & nächster Schritt
![Isaac Gym training](https://doc-cdn.unitree.com/static/2023/9/11/3ab2cbb2083b489da41f7aecc6a8a299_715x448.png)

![Terrain generation (MuJoCo)](docs/repos/unitree_g1_vibes/RL-shenanigans/unitree_mujoco/doc/terrain.png)

![Simulation options](docs/generated_charts/day_4_chart_16.png)

**Isaac Gym environment (from RL routine)**
```bash
conda create -n rl-g1 python=3.8
conda activate rl-g1
pip3 install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
pip3 install numpy==1.23.5
```

```bash
git clone https://github.com/leggedrobotics/rsl_rl
cd rsl_rl
git checkout v1.0.2
pip install -e .
```

```bash
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
cd unitree_rl_gym/legged_gym/scripts
python3 train.py --task=g1
python3 play.py --task=g1
```

**MuJoCo and MJCF assets**
![MJCF paths](docs/generated_charts/day_4_chart_17.png)

```bash
# Render MJCF previews locally
python3 scripts/render_g1_models.py --output-dir docs/rendered
```

![Sim-to-real loop](docs/generated_charts/day_4_chart_18.png)
