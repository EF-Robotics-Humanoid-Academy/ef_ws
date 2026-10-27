# Repository Guidelines

## Project Structure & Module Organization

Core retargeting logic lives in `general_motion_retargeting/`; `motion_retarget.py` coordinates IK, while `kinematics_model.py`, format adapters under `utils/`, and JSON mappings under `ik_configs/` support it. Executable workflows are in `scripts/` (conversion, streaming, visualization, simulation, and packaging). `teleop_studio/` contains the Qt desktop application, and `sonic_sim/` contains simulator integration. Robot XML/URDF files, meshes, and sample motions belong in `assets/`. Treat `third_party/` as vendored code and avoid broad cleanup there.

## Environment Setup

Use Python 3.10+, the version documented and exercised by the project:

```bash
conda create -n gmr python=3.10 -y
conda activate gmr
pip install -e .
```

## Test, and Development Commands

Open the GUI application:

```bash
bash scripts/run_teleop_studio.sh
```

Activate the local environment before running any python commands.

```bash
source ~/.holosoma_deps/miniconda3/bin/activate gmr
python scripts/smplx_to_robot.py --smplx_file INPUT --robot unitree_g1 --save_path output.pkl
python scripts/vis_robot_motion.py --robot unitree_g1 --robot_motion_path output.pkl
python scripts/run_sonic_sim.py
```

The GMR environment supplies runtime dependencies. Conversion commands produce retargeted pickle files; the viewer provides a visual correctness check. Use `python scripts/run_sonic_sim.py` for Mujoco simulation and `python scripts/run_sonic_sim.py --no-onscreen` for headless simulator work.

## Coding Style & Naming Conventions

Follow existing Python conventions: four-space indentation, `snake_case` for modules/functions/variables, `PascalCase` for classes, and uppercase names for module constants. Keep imports grouped as standard library, external packages, then local modules. Prefer type hints on new public APIs and concise comments for coordinate frames, quaternion order, or non-obvious IK math. No formatter or linter is configured, so keep patches focused and match surrounding code.

## Testing Guidelines

There is no first-party automated test suite or coverage threshold. Exercise the exact script and robot/input format changed, then inspect the saved motion with `vis_robot_motion.py`. Use the known AMASS and LAFAN1 cases in `TEST_MOTIONS.md` for regression checks. Tests under `third_party/` cover vendored libraries only; run them only when modifying those libraries. Name new pytest files `test_<feature>.py` and keep fixtures small.

## Commit & Pull Request Guidelines

Recent commits use short imperative subjects, commonly prefixed with GitHub emoji codes such as `:bug:`, `:sparkles:`, or `:speech_balloon:`. Keep each commit scoped to one behavior. Pull requests should identify affected robots and input formats, list commands used for validation, link relevant issues, and include screenshots or short videos for viewer, simulator, or Teleop Studio changes. Do not commit licensed SMPL-X body models, private motion captures, credentials, or machine-specific paths.
