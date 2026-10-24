"""SLAM skills on sdk_wrapper.G1 (replaces agent.slam.SlamBackend, which
drives modules/scripts/slam_web_app.SlamWebState -> sdk_slam).

Same public methods and result strings ("<label> ok|failed: <raw>") as the
agent's SlamBackend, so the skill registry and NavigationAdapter work
unchanged.  SlamWebState's operating state (task queue, selected pose,
relocation flag, last action) lives here; the map visualisation it also had
is the dashboard's /control SLAM view.  Named points keep the agent's own file
(G1_SLAM_POINTS_FILE, default ~/.g1_agent/slam_points.json).
"""
from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
import time
from dataclasses import dataclass
from difflib import SequenceMatcher
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .sensors import SensorWatch

NAV_REACHED_DISTANCE_M = 0.35
NAV_TARGET_TIMEOUT_S = 120.0
NAV_POLL_INTERVAL_S = 0.5
TOPIC_NAMES = {
    "rt/unitree/slam_mapping/points": "slam_mapping",
    "rt/unitree/slam_relocation/points": "slam_relocation",
    "rt/unitree/slam_relocation/global_map": "slam_global_map",
    "rt/unitree/slam_relocation/web_points": "slam_web_points",
    "rt/utlidar/cloud_deskewed": "deskewed",
    "rt/utlidar/cloud_livox_mid360": "livox",
}


@dataclass
class PoseTarget:
    x: float
    y: float
    yaw: float = 0.0
    z: float = 0.0

    def xy_distance_to(self, other: "PoseTarget") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def as_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw}


def _response(result: Dict[str, Any]) -> Dict[str, Any]:
    raw = result.get("raw")
    try:
        raw = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
    except Exception:
        pass
    ok = int(result.get("code", -1)) == 0
    if isinstance(raw, dict):
        ok = ok and int(raw.get("errorCode", 0)) == 0 and bool(raw.get("succeed", True))
    return {"code": int(result.get("code", -1)), "ok": bool(ok), "raw": raw}


class G1SlamBackend:
    def __init__(self, robot: Any, sensors: SensorWatch, *, map_path: Optional[str] = None,
                 points_file: Optional[str] = None) -> None:
        self.robot = robot
        self.sensors = sensors
        self.map_path = map_path or getattr(robot, "_slam_map_path", "/home/unitree/test.pcd")
        self.points_file = Path(points_file or os.environ.get("G1_SLAM_POINTS_FILE", "~/.g1_agent/slam_points.json")).expanduser()
        self.tasks: List[PoseTarget] = []
        self.initial_pose: Optional[PoseTarget] = None
        self.last_valid_pose: Optional[PoseTarget] = None
        self.selected_pose: Optional[PoseTarget] = None
        self.slam_running = False
        self.relocation_ready = False
        self.last_action: Dict[str, Any] = {"label": "startup", "ok": True, "raw": "ready"}
        self._lock = threading.RLock()

    # -- state -------------------------------------------------------------------

    def current_pose(self) -> Optional[PoseTarget]:
        pose = self.robot.get_slam_pose()
        if pose is None:
            return None
        target = PoseTarget(float(pose[0]), float(pose[1]), float(pose[2]))
        self.last_valid_pose = target
        return target

    def _record(self, label: str, result: Dict[str, Any]) -> Dict[str, Any]:
        self.last_action = {"label": label, **result, "stamp": time.time()}
        return self.last_action

    @staticmethod
    def _format(result: Dict[str, Any]) -> str:
        return f"{result.get('label', 'slam')} {'ok' if result.get('ok') else 'failed'}: {result.get('raw')}"

    # -- operations (SlamBackend API) ----------------------------------------------

    def start_mapping(self, slam_type: str = "indoor") -> str:
        before = self.preflight()
        with self._lock:
            self.tasks.clear()
            self.initial_pose = self.current_pose()
            result = _response(self.robot.start_mapping(slam_type))
            self.slam_running = result["ok"]
            self.relocation_ready = False
            result = self._record("start_mapping", result)
        message = self._format(result)
        if not result["ok"]:
            message = self._append_slam_data_hint(message, result)
            message += f"; preflight={json.dumps(before, ensure_ascii=False, sort_keys=True)}"
        return message

    def save_map(self, path: Optional[str] = None) -> str:
        with self._lock:
            result = _response(self.robot.stop_mapping(save_path=path or self.map_path))
            if result["ok"]:
                self.map_path = getattr(self.robot, "_slam_map_path", path or self.map_path)
            return self._format(self._record("end_mapping", result))

    def start_relocation(self, path: Optional[str] = None) -> str:
        with self._lock:
            self.map_path = path or self.map_path
            pose = self.current_pose() or self.last_valid_pose or self.initial_pose
            used_origin_fallback = pose is None
            if pose is None:
                # No non-zero /slam_info pose yet: relocate at the map origin.
                pose = PoseTarget(0.0, 0.0, 0.0)
            result = _response(self.robot.relocate(map_path=self.map_path, pose=(pose.x, pose.y, pose.yaw)))
            self.relocation_ready = result["ok"]
            if result["ok"]:
                self.initial_pose = pose
            result = self._record("init_pose", result)
        message = self._format(result)
        if used_origin_fallback:
            message += ("; fallback=used explicit map-origin pose because no non-zero /slam_info pose was available. "
                        "If this fails, move the robot near the saved map origin or a distinctive mapped area and retry.")
        message = self._append_slam_data_hint(message, result)
        if not result["ok"] and self._error_code(result) == 509:
            message += ("; hint=relocation scan matching confidence is low. Move the robot to a distinctive area "
                        "already present in the saved map, verify /utlidar/cloud_livox_mid360 and "
                        "/utlidar/imu_livox_mid360 are fresh, confirm the selected map path is correct, then retry relocation.")
        return message

    def add_current_pose(self) -> str:
        with self._lock:
            pose = self.current_pose()
            if pose is None:
                return self._format(self._record("add_current_pose", {"code": 1, "ok": False, "raw": "No current SLAM pose available."}))
            self.tasks.append(pose)
            self.selected_pose = pose
            return self._format(self._record("add_current_pose", {"code": 0, "ok": True,
                                                                  "raw": {**pose.as_dict(), "task_count": len(self.tasks)}}))

    def _pose_nav(self, target: PoseTarget) -> Dict[str, Any]:
        return _response(self.robot.pose_nav(target.x, target.y, target.yaw))

    def go_to_selected_pose(self) -> str:
        with self._lock:
            target = self.selected_pose
            if target is None:
                return self._format(self._record("go_to_selected_pose", {"code": 1, "ok": False, "raw": "No selected pose. Add a task point first."}))
            if not self.relocation_ready:
                return self._format(self._record("go_to_selected_pose", {"code": 1, "ok": False, "raw": "Relocation is not active."}))
            result = self._pose_nav(target)
            result["target"] = target.as_dict()
            return self._format(self._record("go_to_selected_pose", result))

    def execute_tasks(self) -> str:
        with self._lock:
            tasks = list(self.tasks)
            if not tasks:
                return self._format(self._record("execute_tasks", {"code": 1, "ok": False, "raw": "No task points queued."}))
            if not self.relocation_ready:
                return self._format(self._record("execute_tasks", {"code": 1, "ok": False, "raw": "Relocation is not active."}))
        results, ok = [], True
        for idx, target in enumerate(tasks, start=1):
            result = self._pose_nav(target)
            result.update(target_index=idx, target={"x": target.x, "y": target.y, "yaw": target.yaw})
            if result["ok"]:
                reached, final_pose, elapsed = self._wait_for_target(target)
                result.update(reached=reached, elapsed_s=round(elapsed, 2))
                if final_pose is not None:
                    result["final_pose"] = {"x": final_pose.x, "y": final_pose.y, "yaw": final_pose.yaw}
                    result["final_distance_m"] = round(final_pose.xy_distance_to(target), 3)
                if not reached:
                    result.update(ok=False, code=1, raw=f"Timed out waiting for task {idx} to be reached.")
            results.append(result)
            if not result["ok"]:
                ok = False
                break
        return self._format(self._record("execute_tasks", {"code": 0 if ok else 1, "ok": ok, "raw": results}))

    def _wait_for_target(self, target: PoseTarget) -> Tuple[bool, Optional[PoseTarget], float]:
        start = time.time()
        last_pose: Optional[PoseTarget] = None
        while time.time() - start < NAV_TARGET_TIMEOUT_S:
            pose = self.current_pose()
            if pose is not None:
                last_pose = pose
                if pose.xy_distance_to(target) <= NAV_REACHED_DISTANCE_M:
                    return True, pose, time.time() - start
            time.sleep(NAV_POLL_INTERVAL_S)
        return False, last_pose, time.time() - start

    def pause(self) -> str:
        return self._format(self._record("pause_nav", _response(self.robot.pause_nav())))

    def resume(self) -> str:
        return self._format(self._record("resume_nav", _response(self.robot.resume_nav())))

    def stop_slam(self) -> str:
        with self._lock:
            result = _response(self.robot.stop_mapping(save=False))
            self.slam_running = False
            self.relocation_ready = False
            return self._format(self._record("close_slam", result))

    def _topic_rows(self) -> List[Dict[str, Any]]:
        stamps = self.sensors.timestamps()
        now = time.time()
        rows = []
        for topic, name in TOPIC_NAMES.items():
            ts = stamps.get(f"lidar_cloud[{topic}]", 0.0)
            rows.append({"name": name, "topic": topic, "age_s": None if ts <= 0 else round(now - ts, 2),
                         "fresh": ts > 0 and now - ts < 2.0})
        return rows

    def status(self) -> str:
        pose = self.current_pose()
        compact = {
            "slam_running": self.slam_running,
            "relocation_ready": self.relocation_ready,
            "pose": None if pose is None else {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "last_action": self.last_action,
            "fresh_topics": [row["name"] for row in self._topic_rows() if row["fresh"]],
        }
        return json.dumps(compact, ensure_ascii=False, sort_keys=True, default=str)

    # -- named points ---------------------------------------------------------------

    def save_current_point(self, name: str) -> str:
        clean = self._clean_point_name(name)
        if not clean:
            return "add_current_point failed: point name was empty"
        pose = self.current_pose()
        if pose is None:
            return "add_current_point failed: no current non-zero SLAM pose is available"
        points = self._load_points()
        points[clean] = pose.as_dict()
        self._save_points(points)
        return f"point saved: {json.dumps({'name': clean, **points[clean], 'point_count': len(points)}, ensure_ascii=False, sort_keys=True)}"

    def list_points(self) -> str:
        points = self._load_points()
        if not points:
            return "no saved SLAM points"
        return json.dumps({"map_path": self.map_path, "points_file": str(self.points_file), "points": points},
                          ensure_ascii=False, sort_keys=True)

    def clear_points(self) -> str:
        count = len(self._load_points())
        self._save_points({})
        return f"cleared {count} saved SLAM points"

    def go_to_point(self, name: str, *, auto_relocate: bool = True) -> str:
        point_name, target, score = self._find_point(name)
        if target is None or point_name is None:
            return f"navigate_named_point failed: no saved point named {name!r}; match_score={score:.2f}"
        with self._lock:
            self.selected_pose = PoseTarget(float(target["x"]), float(target["y"]), float(target.get("yaw", 0.0)),
                                            float(target.get("z", 0.0)))
            if auto_relocate and not self.relocation_ready:
                relocation = self.start_relocation(self.map_path)
                if not self.relocation_ready:
                    return f"navigate_named_point failed: relocation failed before navigation: {relocation}"
            result = self._pose_nav(self.selected_pose)
            result.update(point=point_name, match_score=round(score, 3), target=target)
            return self._format(self._record("go_to_selected_pose", result))

    def _load_points(self) -> Dict[str, Dict[str, float]]:
        if not self.points_file.exists():
            return {}
        try:
            data = json.loads(self.points_file.read_text(encoding="utf-8"))
        except Exception:
            return {}
        raw_points = data.get("points", data) if isinstance(data, dict) else {}
        points: Dict[str, Dict[str, float]] = {}
        for name, raw in (raw_points.items() if isinstance(raw_points, dict) else []):
            try:
                clean = self._clean_point_name(str(name))
                point = {"x": float(raw["x"]), "y": float(raw["y"]), "z": float(raw.get("z", 0.0)),
                         "yaw": float(raw.get("yaw", 0.0))}
                if clean and all(math.isfinite(v) for v in point.values()):
                    points[clean] = point
            except Exception:
                continue
        return points

    def _save_points(self, points: Dict[str, Dict[str, float]]) -> None:
        payload = {"map_path": self.map_path, "updated": time.time(), "points": dict(sorted(points.items()))}
        self.points_file.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=f".{self.points_file.name}.", suffix=".tmp", dir=str(self.points_file.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True)
                handle.write("\n")
            os.replace(temporary, self.points_file)
        except Exception:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise

    def _find_point(self, requested: str) -> Tuple[Optional[str], Optional[Dict[str, float]], float]:
        wanted = self._clean_point_name(requested)
        if not wanted:
            return None, None, 0.0
        points = self._load_points()
        if wanted in points:
            return wanted, points[wanted], 1.0
        best_name, best_score = None, 0.0
        wanted_tokens = set(wanted.split()) - {"a", "an", "and", "at", "go", "navigate", "point", "to"}
        for name in points:
            score = max(SequenceMatcher(None, wanted, name).ratio(),
                        SequenceMatcher(None, wanted.replace(" ", ""), name.replace(" ", "")).ratio())
            if wanted_tokens and wanted_tokens <= set(name.split()):
                score = max(score, 0.92)
            if score > best_score:
                best_name, best_score = name, score
        if best_name is None or best_score < 0.62:
            return None, None, best_score
        return best_name, points[best_name], best_score

    @staticmethod
    def _clean_point_name(text: str) -> str:
        text = re.sub(r"^(call it|name it|save it as|save as|called|named)\s+", "", str(text).strip().lower()).strip()
        text = re.sub(r"[^a-z0-9 _-]+", "", text)
        return re.sub(r"\s+", " ", text).strip(" -_")

    # -- diagnostics -------------------------------------------------------------------

    @staticmethod
    def _error_code(result: Dict[str, Any]) -> int:
        raw = result.get("raw")
        for source, key in ((raw, "errorCode"), (result, "errorCode"), (result, "code")):
            try:
                if isinstance(source, dict) and source.get(key) is not None:
                    return int(source[key])
            except Exception:
                pass
        return -1

    def _append_slam_data_hint(self, message: str, result: Dict[str, Any]) -> str:
        if result.get("ok") or self._error_code(result) != 501:
            return message
        return (message + "; hint=Unitree SLAM returned 501 Lack of lidar or imu data. The API call reached the SLAM "
                "service, but the service is not receiving accepted lidar+IMU data. Check the robot-side lidar driver "
                "and SLAM prerequisites, especially DDS topics rt/utlidar/cloud_livox_mid360 and "
                "rt/utlidar/imu_livox_mid360, plus matching --iface/--domain-id.")

    def preflight(self) -> Dict[str, Any]:
        diagnostics: Dict[str, Any] = {}
        try:
            stale = self.sensors.stale(max_age=2.0)
            diagnostics["sensor_stale"] = stale
            diagnostics["fresh_lidar_cloud"] = any(name.startswith("lidar_cloud") and not s for name, s in stale.items())
            diagnostics["fresh_lidar_imu"] = not stale.get("lidar_imu", True)
            diagnostics["fresh_lowstate"] = not stale.get("lowstate", True)
            diagnostics["fresh_slam_odom"] = not stale.get("slam_odom", True)
        except Exception as exc:
            diagnostics["sensor_error"] = str(exc)
        try:
            row = self.robot.get_service("unitree_slam")
            diagnostics["unitree_slam_service_status"] = None if row is None else row.get("status")
        except Exception as exc:
            diagnostics["unitree_slam_service_error"] = str(exc)
        try:
            diagnostics["slam_info_available"] = bool(self.robot.get_slam_info())
        except Exception as exc:
            diagnostics["slam_info_error"] = str(exc)
        diagnostics["web_fresh_topics"] = [row["name"] for row in self._topic_rows() if row["fresh"]]
        fresh_cloud = bool(diagnostics.get("fresh_lidar_cloud"))
        fresh_imu = bool(diagnostics.get("fresh_lidar_imu"))
        slam_status = diagnostics.get("unitree_slam_service_status")
        if slam_status == 0 and not fresh_cloud and not fresh_imu:
            diagnostics["root_cause"] = ("unitree_slam is running, but neither the required lidar point cloud nor lidar IMU "
                                         "has produced fresh samples for this process. Unitree SLAM API 1801 will return "
                                         "errorCode=501 until both are live.")
        elif slam_status == 0 and not fresh_cloud:
            diagnostics["root_cause"] = "unitree_slam is running, but no fresh lidar point-cloud samples are visible to the SLAM client."
        elif slam_status == 0 and not fresh_imu:
            diagnostics["root_cause"] = "unitree_slam is running, but no fresh lidar IMU samples are visible to the SLAM client."
        elif slam_status not in (None, 0):
            diagnostics["root_cause"] = "unitree_slam service is not ON/healthy according to robot_state service status."
        else:
            diagnostics["root_cause"] = "SLAM prerequisites look partially available; if 501 persists, inspect lidar_driver logs."
        diagnostics["next_checks"] = [
            "ros2 topic hz /utlidar/cloud_livox_mid360",
            "ros2 topic hz /utlidar/imu_livox_mid360",
            "ros2 topic hz /utlidar/cloud_deskewed",
            "restart/check lidar_driver if all lidar topics are discovered but publish no samples",
            "keep unitree_slam ON after lidar/IMU samples are fresh, then retry start_mapping",
        ]
        return diagnostics
