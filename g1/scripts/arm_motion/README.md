# arm_motion

Arm control scripts for Unitree G1, including direct DDS command examples and programming-by-demonstration (PBD) workflows.

## Contents
- `g1_arm7_sdk_dds_example.py`
  - Low-level DDS arm command example on `rt/arm_sdk`.
  - Shows joint indexing, CRC handling, and staged motion playback.
- `pbd/`
  - `pbd_demonstrate.py`: record arm trajectories from manual demonstration.
  - `pbd_reproduce.py`: replay recorded trajectories (CSV/NPZ).
  - `pbd_docs.md`: file format and usage notes.
  - `motion_databse/*.csv`: sample trajectories.
- `safety/`
  - `hanger_boot_sequence.py`: safe startup helper.
  - keyboard/controller helpers used by motion and navigation scripts.
- `sdk_details.md`
  - arm SDK notes and setup details.

## Recommended Workflow
1. Ensure safe boot / balanced stand.
2. Record a motion with `pbd/pbd_demonstrate.py` (optional).
3. Replay via `pbd/pbd_reproduce.py`.
4. Move to direct DDS authoring with `g1_arm7_sdk_dds_example.py` when needed.

## Typical Commands
```bash
# Record a demonstration
python3 pbd/pbd_demonstrate.py --iface enp1s0 --arm both --out /tmp/pbd_motion.npz

# Replay a demonstration
python3 pbd/pbd_reproduce.py --iface enp1s0 --file /tmp/pbd_motion.npz --arm both

# Run DDS arm control example
python3 g1_arm7_sdk_dds_example.py enp1s0
```

## Notes
- Keep a spotter and clear workspace for all arm motion tests.
- Joint index validity depends on robot variant (23 DoF vs 29 DoF / hand configuration).
- Many scripts assume `unitree_sdk2py` is installed editable and CycloneDDS is available.
