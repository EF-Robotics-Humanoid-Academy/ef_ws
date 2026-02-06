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

## Object Detection
![SLAM overview](./placeholder.jpg)

- Basics: input preprocessing, model inference, and binary classification.
- Target: detect if an object is a soda can.

Beginner notes:
- Start with a small, well-lit dataset (positive and negative examples).
- Use a single object class first, then add more classes later.

---

## Decision making
![Basic motor control overview](./placeholder.jpg)

- Basics: simple state machine to connect perception to actions.
- Trigger pick-and-place when detection confidence exceeds a threshold.

Beginner notes:
- Keep the state machine small: `search` → `approach` → `pick` → `place`.
- Add timeouts so the robot can recover if a step fails.

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
