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

## Schedule (Tag 3 – Mi 18.02.2026)
- 08.00 Uhr Crew
- 08:30 Einfinden der Teilnehmer
- 09:00 Schulungsblock 1
- 10:30 Kaffee- und Teepause
- 10:45 Schulungsblock 2
- 12:00 Mittagessen
- 13:00 Schulungsblock 3
- 14:30 Kaffee- und Teepause mit Snacks
- 15:00 Schulungsblock 4
- 17:00 Ende Tag 3
- 17:00 – 17:30 Podcast / Testimonials
- 17:30 – 17:45 Puffer
- 17:45 – 18:15 Tages‑Recap
- 18:15 – 18:30 Tagesabschluss

---

## Complex motion
[https://www.unitree.com/images/7e51cf20dc6145cf99ae0d0b6ea4d2c5.mp4](https://www.unitree.com/images/7e51cf20dc6145cf99ae0d0b6ea4d2c5.mp4)

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
![SLAM overview](./placeholder.jpg)

### SLAM Navigation Service Interface (Beginner Flow)
Before you start:
- Ensure your PC and the robot are on the same LAN segment.
- Turn on `unitree_slam` and `lidar_driver` in the App.
- Do not use App navigation at the same time as API calls.

**Coordinate frame (important)**
- Origin: MID360-IMU coordinate system.
- +X forward, +Z up.

**Recommended scope**
- Static indoor, flat scenes with rich features.
- X/Y map size under 45 m.
- Avoid violent movements to prevent localization loss.

### Network check (quick)
1. Set your PC to `192.168.123.XXX` (no IP conflicts).
2. Test connections:
```bash
ssh unitree@192.168.123.164
ping 192.168.123.120
```

### Core API IDs
| Action | API ID | Purpose |
| --- | --- | --- |
| Start mapping | 1801 | Begin SLAM map creation |
| End mapping | 1802 | Save the map (PCD file) |
| Initialize pose | 1804 | Load map + set start pose |
| Pose navigation | 1102 | Navigate to a target pose |
| Pause navigation | 1201 | Pause current nav |
| Resume navigation | 1202 | Resume current nav |
| Close SLAM | 1901 | Stop SLAM service |

### Create a map (start → explore → save)
**1) Start mapping**
```json
{
  "api_id": 1801,
  "data": {
    "slam_type": "indoor"
  }
}
```

**2) Explore slowly**
- Walk the robot around the space at low speed.
- Try to close a loop for a clean map.

**3) End mapping and save**
```json
{
  "api_id": 1802,
  "data": {
    "address": "/home/unitree/test1.pcd"
  }
}
```
Note:
- Reuse file names like `test1.pcd` to `test10.pcd` to avoid filling disk.

### Use a saved map (localize → navigate)
**1) Initialize pose with the map**
```json
{
  "api_id": 1804,
  "data": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "q_x": 0.0,
    "q_y": 0.0,
    "q_z": 0.0,
    "q_w": 1.0,
    "address": "/home/unitree/test1.pcd"
  }
}
```

**2) Send a navigation goal**
```json
{
  "api_id": 1102,
  "data": {
    "targetPose": {
      "x": 2.0,
      "y": 0.0,
      "z": 0.0,
      "q_x": 0.0,
      "q_y": 0.0,
      "q_z": 0.0,
      "q_w": 1.0
    },
    "mode": 1
  }
}
```
Notes:
- Target distance must be within 10 m.
- Robot moves in a straight line.
- Obstacles should be at least 50 cm tall.

### Topics to watch (optional, for debugging)
| Topic | Data |
| --- | --- |
| `rt/unitree/slam_mapping/points` | Mapping point cloud |
| `rt/unitree/slam_mapping/odom` | Mapping odometry |
| `rt/unitree/slam_relocation/points` | Relocation point cloud |
| `rt/unitree/slam_relocation/odom` | Relocation odometry |
| `rt/slam_info` | Status broadcast (JSON) |
| `rt/slam_key_info` | Task result (JSON) |

Beginner notes:
- Start in a simple room before trying corridors or clutter.
- Keep speed low while building the map.
- Save the map immediately after a clean loop.

---

## Path planning
![SLAM overview](./placeholder.jpg)

**Example goal**
```json
{
  "api_id": 1102,
  "data": {
    "targetPose": {
      "x": 2.0,
      "y": 1.0,
      "z": 0.0,
      "q_x": 0.0,
      "q_y": 0.0,
      "q_z": 0.0,
      "q_w": 1.0
    },
    "mode": 1
  }
}
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

---

## Implementation details (from repo scripts)
- `g1/scripts/obstacle_avoidance/navigate.py` is the orchestration entry point. It initializes DDS, starts `LocoClient`, spins up `ObstacleDetector`, creates or loads an `OccupancyGrid`, then repeatedly plans (A*), smooths, and executes waypoints. It replans when `front_blocked()` is true or when a waypoint times out.
- `g1/scripts/obstacle_avoidance/obstacle_detection.py` subscribes to `rt/sportmodestate` and reads `range_obstacle[4]` plus `imu_state.rpy` to compute pose and obstacle directions (front, right, rear, left). It exposes thread-safe getters and a stale-data check.
- `g1/scripts/obstacle_avoidance/create_map.py` implements a 2D grid with world-to-grid conversion, obstacle marking from range data, and inflation (via binary dilation) for safe planning margins.
- `g1/scripts/obstacle_avoidance/path_planner.py` runs A* on the inflated grid, then smooths the path using line-of-sight checks and converts it into world-coordinate waypoints spaced by `--spacing` (default 0.5 m).
- `g1/scripts/obstacle_avoidance/locomotion.py` provides a proportional controller: turn-in-place until heading error is below `yaw_tolerance`, then move forward with bounded `vx` and `vyaw`. It accepts a `check_obstacle` callback so the planner can abort and replan cleanly.
- `g1/scripts/obstacle_avoidance/map_viewer.py` renders a live OpenCV map (optional `--viz`) showing inflated cells, planned path, waypoints, and sensor rays for debug sessions.
