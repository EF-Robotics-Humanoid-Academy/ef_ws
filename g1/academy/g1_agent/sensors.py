"""Topic freshness for the agent's state, navigation and SLAM preflight.

Replaces sdk_client.Robot.start_sensors() / get_sensor_timestamps() /
sensors_stale().  The keys are the legacy ones ("lowstate", "sport",
"odom", "slam_odom", "lidar_map", "lidar_cloud", "lidar_cloud[<topic>]",
"lidar_imu", "<side>_hand_state") because agent.state, agent.navigation and
the SLAM preflight interpret them by name.  G1's own readers are reused; only
topics G1 does not subscribe to (lidar cloud/IMU/height map) get readers here,
created on first use.
"""
from __future__ import annotations

import importlib
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

LIDAR_CLOUD_TOPICS = ("rt/utlidar/cloud_livox_mid360", "rt/utlidar/cloud_deskewed")
LIDAR_IMU_TOPIC = "rt/utlidar/imu_livox_mid360"
LIDAR_MAP_TOPIC = "rt/utlidar/map_state"


def _msg_type(module: str, name: str) -> Any:
    try:
        return getattr(importlib.import_module(module), name)
    except Exception:
        return None


class SensorWatch:
    def __init__(self, robot: Any) -> None:
        self.robot = robot
        self._lock = threading.Lock()
        self._started = False
        self._extra: List[Tuple[str, Any]] = []
        self.errors: Dict[str, str] = {}

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._started = True
            cloud = _msg_type("unitree_sdk2py.idl.sensor_msgs.msg.dds_", "PointCloud2_")
            imu = _msg_type("unitree_sdk2py.idl.sensor_msgs.msg.dds_", "Imu_")
            height_map = _msg_type("unitree_sdk2py.idl.unitree_go.msg.dds_", "HeightMap_")
            wanted = [(f"lidar_cloud[{t}]", t, cloud) for t in LIDAR_CLOUD_TOPICS]
            wanted += [("lidar_imu", LIDAR_IMU_TOPIC, imu), ("lidar_map", LIDAR_MAP_TOPIC, height_map)]
            for key, topic, msg_type in wanted:
                if msg_type is None:
                    self.errors[key] = f"message type for {topic} unavailable"
                    continue
                try:
                    self._extra.append((key, self.robot._latest(topic, msg_type)))
                except Exception as exc:
                    self.errors[key] = f"{topic}: {exc}"

    def timestamps(self) -> Dict[str, float]:
        self.start()
        robot = self.robot
        out: Dict[str, float] = {
            "lowstate": robot._lowstate.get()[1],
            "sport": max(robot._sport.get()[1], robot._sport_alt.get()[1]),
            "odom": robot._odom.get()[1],
            "slam_odom": robot._slam_odom.get()[1],
        }
        clouds = [(f"lidar_cloud[{topic}]", reader) for topic, reader in getattr(robot, "_clouds", [])]
        for key, reader in clouds + list(self._extra):
            out[key] = reader.get()[1]
        out["lidar_cloud"] = max([ts for key, ts in out.items() if key.startswith("lidar_cloud[")] or [0.0])
        for side in ("left", "right"):
            dex3 = getattr(robot, "_dex3", {}).get(side)
            if dex3 is not None:
                out[f"{side}_hand_state"] = dex3.state.get()[1]
        return {key: float(ts or 0.0) for key, ts in out.items()}

    def stale(self, max_age: float = 1.0) -> Dict[str, bool]:
        now = time.time()
        return {name: ts <= 0.0 or now - ts > max_age for name, ts in self.timestamps().items()}

    def age(self, key: str) -> Optional[float]:
        ts = self.timestamps().get(key, 0.0)
        return None if ts <= 0.0 else max(0.0, time.time() - ts)
