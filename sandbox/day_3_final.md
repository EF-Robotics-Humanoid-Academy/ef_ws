# Day 3: Navigation and Environment Awareness

---

## Day 3 outline
- Complex motion
- SLAM
- Lunch break
- Path planning
- Obstacle avoidance

---

## Complex motion
![Placeholder](./placeholder.jpg)

**Hardcoded pick and place (known pose)**
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

## SLAM
![Placeholder](./placeholder.jpg)

**Create and save map**
```json
{"api_id": 1801, "parameter": ""}
```

```json
{"api_id": 1802, "parameter": "map_name"}
```

---

## Path planning
![Placeholder](./placeholder.jpg)

**Example goal**
```json
{"api_id": 1102, "parameter": {"target_pos": [2.0, 1.0, 0.0]}}
```

---

## Obstacle avoidance
![Placeholder](./placeholder.jpg)

- Enable obstacle avoidance in the same navigation task after map initialization.
- Use the navigation stack to replan when new obstacles appear.
