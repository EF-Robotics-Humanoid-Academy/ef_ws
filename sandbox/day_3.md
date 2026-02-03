# Day 3: Navigation and Environment Awareness

## Day outline
- [Navigation & Umgebung](#navigation--umgebung)
- [Mapping & Lokalisierung im Szenario](#mapping--lokalisierung-im-szenario)
- [Routenplanung & Hindernisvermeidung](#routenplanung--hindernisvermeidung)
- [Arbeiten mit Sensordaten (Kamera/LiDAR)](#arbeiten-mit-sensordaten-kameralidar)
- [Übung: Navigationsaufgabe im Parcours](#übung-navigationsaufgabe-im-parcours)

---

## Navigation & Umgebung
![Navigation pipeline](docs/generated_charts/day_3_chart_01.png)

![Odometry topics](docs/generated_charts/day_3_chart_02.png)

---

## Mapping & Lokalisierung im Szenario
![Mapping and nav example](https://doc-cdn.unitree.com/static/2025/7/23/ea38cdd7418e4b81852c819a55e7aa2e_1164x1000.jpg)

![Required services](docs/generated_charts/day_3_chart_03.png)

**Prerequisites**
- Be on the same LAN segment as the robot.
- Enable `unitree_slam` and `lidar_driver` via the app before calling APIs.

![SLAM topics to subscribe](docs/generated_charts/day_3_chart_04.png)

![Mapping constraints](docs/generated_charts/day_3_chart_05.png)

**Operational note**: do not use App navigation while calling the SLAM API.

```json
// Start mapping (API 1801)
{"api_id": 1801, "parameter": ""}
```

```json
// End mapping and save PCD (API 1802)
{"api_id": 1802, "parameter": "map_name"}
```

```json
// Initialize pose (API 1804)
{"api_id": 1804, "parameter": {"map_name": "map_name", "init_pos": [0.0, 0.0, 0.0]}}
```

---

## Routenplanung & Hindernisvermeidung
![Planner tradeoffs](docs/generated_charts/day_3_chart_06.png)

```json
// Navigate to pose (API 1102)
{"api_id": 1102, "parameter": {"target_pos": [2.0, 1.0, 0.0]}}
```

```json
// Pause/Resume/Close
{"api_id": 1201, "parameter": ""}
{"api_id": 1202, "parameter": ""}
{"api_id": 1901, "parameter": ""}
```

![Safety checks during navigation](docs/generated_charts/day_3_chart_07.png)

---

## Arbeiten mit Sensordaten (Kamera/LiDAR)
![Livox MID-360 mount](https://doc-cdn.unitree.com/static/2024/12/18/b68f35fca9724bfcae513610179e0bed_356x393.png)

![RealSense D435i](https://doc-cdn.unitree.com/static/2024/7/26/ddc4f187060d434587e2f08ea0045e3c_1042x718.png)

![LiDAR placement (from lidar services)](docs/generated_charts/day_3_chart_08.png)

![LiDAR topics](docs/generated_charts/day_3_chart_09.png)

![LiDAR network](docs/generated_charts/day_3_chart_10.png)

![Depth camera essentials](docs/generated_charts/day_3_chart_11.png)

```bash
realsense-viewer
```

![ROS 2 + DDS integration](docs/generated_charts/day_3_chart_12.png)

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

---

## Übung: Navigationsaufgabe im Parcours
![Workflow](docs/generated_charts/day_3_chart_13.png)

```bash
rviz2
ros2 run tf2_tools view_frames
```

```bash
# Example goal command via ROS 2 nav stack (if bridged)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
