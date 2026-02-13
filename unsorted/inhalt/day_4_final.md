# Day 4: Perception and Project Challenge

---

## Day 4 outline
- Object Detection
- Decision making
- Lunch break
- Reinforcement Learning
- Final task

Beginner focus:
- Start with simple perception and a small decision loop, then scale up.

---

## Schedule (Tag 4 – Do 19.02.2026)
- 08.00 Uhr Crew
- 08:30 Einfinden der Teilnehmer
- 09:00 Schulungsblock 1
- 10:30 Kaffee- und Teepause
- 10:45 Schulungsblock 2
- 12:00 Mittagessen
- 13:00 Schulungsblock 3
- 14:30 Kaffee- und Teepause mit Snacks
- 15:00 Schulungsblock 4
- 17:00 Ende Tag 4
- Ab 17:00 Abbau (nur Tag 4)
- Abbau & Verladung
- Abschluss‑Recap

---

## Object Detection
![SLAM overview](./placeholder.jpg)

- Basics: input preprocessing, model inference, and binary classification.
- Target: detect if an object is a soda can.

Beginner notes:
- Start with a small, well-lit dataset (positive and negative examples).
- Use a single object class first, then add more classes later.

### Implementation details (from repo scripts)
- `g1/scripts/obj_detection/soda_can_detect.py` captures a JPEG frame from the robot via `VideoClient.GetImageSample()`, decodes it with OpenCV, then runs CLIP zero-shot classification with labels `["a soda can", "no soda can, an empty scene"]`.
- The script exposes `--threshold` (default 0.6), `--timeout` for the RPC, and `--save` or `--show` to visualize results. It runs fully on the development laptop over DDS, not on the robot.

---

## Decision making
![Basic motor control overview](./placeholder.jpg)

- Basics: simple state machine to connect perception to actions.
- Trigger pick-and-place when detection confidence exceeds a threshold.

Beginner notes:
- Keep the state machine small: `search` → `approach` → `pick` → `place`.
- Add timeouts so the robot can recover if a step fails.

### Implementation details (from repo scripts)
- `g1/scripts/obstacle_parcour/pattern_1.py` is the reference decision loop. It executes a multi-phase pipeline: navigate to point B, wave, navigate to point C, run soda-can detection, then conditionally run pick-and-place.
- Each phase is isolated so you can set per-phase timeouts and handle failures independently. Navigation phases use the same A* + replanning logic as `obstacle_avoidance/navigate.py`.

---

## Reinforcement Learning
![RL and simulators](https://doc-cdn.unitree.com/static/2024/6/20/ad293ca6260b4c09b915e4adc39637e6_1590x924.png)

- Short theory: policy, reward, and sim-to-real transfer.
- IsaacLab/Isaac Gym intro and example workflow for G1.

Beginner notes:
- Treat RL as an advanced option; start with scripted behaviors first.
- Use simulation to test many variations safely before trying on the real robot.

---

## Final task
![G1 component overview](./placeholder.jpg)

- Necessary modules: locomotion, perception, arm control, and basic decision logic.
- Suggested task: obstacle parcours with object detection and a pick action.

Beginner notes:
- Break the final task into small checklists and verify each step.
- Keep goals short and repeatable for consistent demos.

### Implementation details (from repo scripts)
- `g1/scripts/pick_and_place/g1_pick_place_hardcoded.py` publishes low-level arm and hand commands to `rt/arm_sdk` and `rt/dex3/right/cmd`, interpolating joint targets over time and applying CRC checks for each frame.
- The script uses fixed joint pose dictionaries (`POSES`) and soft-grip parameters (low `kp` and small `tau`) to reduce the chance of crushing the object during grasp.
