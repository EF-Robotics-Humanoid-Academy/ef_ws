# Day 3: Navigation and Environment Awareness

---

## Day 3 outline
- Complex motion
- SLAM
- Lunch break
- Path planning
- Obstacle avoidance

Beginner focus:
- We keep navigation goals small and predictable before attempting complex routes.

---

## Complex motion
![G1 component overview](https://www.docs.quadruped.de/projects/g1/html/_images/g1_component_overview.png)

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

Beginner notes:
- Start with a known object pose and slow arm motion.
- Keep the robot in a stable stand during the pick-and-place.
- Use small, testable changes to poses (5–10 cm at a time).

---

## SLAM
![SLAM overview](https://doc-cdn.unitree.com/static/2025/7/23/ea38cdd7418e4b81852c819a55e7aa2e_1164x1000.jpg)

**Create and save map**
```json
{"api_id": 1801, "parameter": ""}
```

```json
{"api_id": 1802, "parameter": "map_name"}
```

Beginner notes:
- Start mapping in a simple room before trying corridors or clutter.
- Keep the robot speed low while building the map.
- Save the map immediately after a clean loop.

---

## Path planning
![SLAM overview](https://doc-cdn.unitree.com/static/2025/7/23/ea38cdd7418e4b81852c819a55e7aa2e_1164x1000.jpg)

**Example goal**
```json
{"api_id": 1102, "parameter": {"target_pos": [2.0, 1.0, 0.0]}}
```

Beginner notes:
- Use short, easy goals first (1–2 meters).
- Confirm localization is stable before sending a goal.
- If planning fails, remap or reduce map size.

---

## Obstacle avoidance
![SLAM overview](https://doc-cdn.unitree.com/static/2025/7/23/ea38cdd7418e4b81852c819a55e7aa2e_1164x1000.jpg)

- Enable obstacle avoidance in the same navigation task after map initialization.
- Use the navigation stack to replan when new obstacles appear.

Beginner notes:
- Place one or two obstacles first, then increase complexity.
- If avoidance oscillates, slow down and increase obstacle clearance.
