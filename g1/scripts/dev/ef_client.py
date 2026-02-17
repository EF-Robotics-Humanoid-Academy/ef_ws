"""
ef_client.py
============

Pragmatic high-level client for Unitree G1.

Highlights:
- Locomotion/FSM helpers
- Cached IMU + lidar subscriptions
- Local RGBD GST viewer launcher
- Local SLAM launcher (live_slam_save.py)
- Path point queue + navigation execution
- SLAM API debug routine
- Audio/headlight convenience wrappers
"""
from __future__ import annotations

import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------

THIS_FILE = Path(__file__).resolve()
DEV_OTHER_DIR = THIS_FILE.parent
SCRIPTS_ROOT = DEV_OTHER_DIR.parent
DEV_OTHER_SAFETY_DIR = DEV_OTHER_DIR / "other" / "safety"

for p in (str(DEV_OTHER_DIR), str(SCRIPTS_ROOT), str(DEV_OTHER_SAFETY_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

# ---------------------------------------------------------------------------
# Unitree SDK imports
# ---------------------------------------------------------------------------

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, HeightMap_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

try:
    from safety.hanger_boot_sequence import hanger_boot_sequence
except Exception:
    from hanger_boot_sequence import hanger_boot_sequence  # type: ignore

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_SPORT_TOPIC = "rt/odommodestate"
DEFAULT_LIDAR_MAP_TOPIC = "rt/utlidar/map_state"
DEFAULT_LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_deskewed"
DEFAULT_NAV_OVERLAY_PLAN_FILE = "/tmp/g1_nav_overlay_plan.json"

ARM_JOINT_ALIASES = {
    "shoulder": "shoulder_pitch",
    "shoulder_pitch": "shoulder_pitch",
    "shoulder_roll": "shoulder_roll",
    "shoulder_yaw": "shoulder_yaw",
    "elbow": "elbow",
    "wrist": "wrist_pitch",
    "wrist_pitch": "wrist_pitch",
    "wrist_roll": "wrist_roll",
    "wrist_yaw": "wrist_yaw",
    "waist_yaw": "waist_yaw",
    "waist": "waist_yaw",
}

LEG_JOINT_ALIASES = {
    "hip_pitch": "hip_pitch",
    "hip_roll": "hip_roll",
    "hip_yaw": "hip_yaw",
    "knee": "knee",
    "ankle_pitch": "ankle_pitch",
    "ankle_roll": "ankle_roll",
    "ankle_a": "ankle_roll",
    "ankle_b": "ankle_pitch",
}

G1_LEG_JOINT_INDEX = {
    "left_hip_pitch": 0,
    "left_hip_roll": 1,
    "left_hip_yaw": 2,
    "left_knee": 3,
    "left_ankle_pitch": 4,
    "left_ankle_roll": 5,
    "right_hip_pitch": 6,
    "right_hip_roll": 7,
    "right_hip_yaw": 8,
    "right_knee": 9,
    "right_ankle_pitch": 10,
    "right_ankle_roll": 11,
}


@dataclass
class ImuData:
    rpy: tuple[float, float, float]
    gyro: tuple[float, float, float] | None
    acc: tuple[float, float, float] | None
    quat: tuple[float, float, float, float] | None
    temp: float | None


class _RgbdDepthGuard:
    """
    Lightweight RGBD depth guard using the GST receive pipeline.

    The incoming depth stream is expected to be PLASMA color-mapped depth
    (as produced by jetson_realsense_stream.py).
    """

    def __init__(
        self,
        depth_port: int,
        width: int,
        height: int,
        fps: int,
        near_distance_m: float = 0.75,
        min_coverage: float = 0.18,
        required_hits: int = 2,
    ) -> None:
        self.depth_port = int(depth_port)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))
        self.near_distance_m = float(near_distance_m)
        self.min_coverage = float(min_coverage)
        self.required_hits = max(1, int(required_hits))

        self._available = False
        self._err: str | None = None
        self._blocked = False
        self._hits = 0
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def available(self) -> bool:
        return bool(self._available)

    @property
    def error(self) -> str | None:
        return self._err

    def is_blocked(self) -> bool:
        with self._lock:
            return bool(self._blocked)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            import cv2
            import gi
            import numpy as np

            gi.require_version("Gst", "1.0")
            gi.require_version("GstApp", "1.0")
            from gi.repository import Gst
        except Exception as exc:
            self._err = f"RGBD depth guard unavailable: {exc}"
            self._available = False
            return

        try:
            Gst.init(None)
            pipeline = Gst.parse_launch(
                f"udpsrc port={self.depth_port} caps=application/x-rtp,media=video,encoding-name=H264,payload=97 ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=true sync=false drop=true"
            )
            sink = pipeline.get_by_name("sink")
            if sink is None:
                raise RuntimeError("appsink not found")
            pipeline.set_state(Gst.State.PLAYING)
            self._available = True

            # Build PLASMA lookup in BGR, index 0..255 -> distance 0..6m.
            cmap = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_PLASMA)
            cmap = cmap.reshape(256, 3).astype(np.int16)
            near_idx = int(max(0.0, min(255.0, (self.near_distance_m / 6.0) * 255.0)))

            wait_ns = int(Gst.SECOND // self.fps)

            while self._running:
                sample = sink.emit("try-pull-sample", wait_ns)
                if not sample:
                    time.sleep(0.01)
                    continue
                buf = sample.get_buffer()
                if buf is None:
                    continue
                raw = np.frombuffer(buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
                expected = self.width * self.height * 3
                if raw.size != expected:
                    continue
                depth_bgr = raw.reshape((self.height, self.width, 3))

                # Forward ROI: center horizontally, upper-middle vertically to reduce floor hits.
                x0 = int(self.width * 0.30)
                x1 = int(self.width * 0.70)
                y0 = int(self.height * 0.25)
                y1 = int(self.height * 0.70)
                roi = depth_bgr[y0:y1, x0:x1]
                if roi.size == 0:
                    continue
                pix = roi.reshape(-1, 3).astype(np.int16)

                # Approximate inverse-colormap by nearest BGR entry.
                diff = pix[:, None, :] - cmap[None, :, :]
                dist2 = (diff * diff).sum(axis=2)
                idx = np.argmin(dist2, axis=1)
                near_cov = float(np.mean(idx <= near_idx))
                blocked_now = near_cov >= self.min_coverage

                with self._lock:
                    if blocked_now:
                        self._hits += 1
                    else:
                        self._hits = 0
                    self._blocked = self._hits >= self.required_hits
        except Exception as exc:
            self._err = str(exc)
            self._available = False
        finally:
            try:
                pipeline.set_state(Gst.State.NULL)  # type: ignore[name-defined]
            except Exception:
                pass


class Robot:
    """End-user wrapper around common G1 workflows."""

    def __init__(
        self,
        iface: str = "eth0",
        domain_id: int = 0,
        safety_boot: bool = True,
        auto_start_sensors: bool = True,
        sport_topic: str = DEFAULT_SPORT_TOPIC,
        lidar_map_topic: str = DEFAULT_LIDAR_MAP_TOPIC,
        lidar_cloud_topic: str = DEFAULT_LIDAR_CLOUD_TOPIC,
        slam_info_topic: str = "rt/slam_info",
        slam_key_topic: str = "rt/slam_key_info",
        rgb_port: int = 5600,
        depth_port: int = 5602,
        rgb_width: int = 640,
        rgb_height: int = 480,
        rgb_fps: int = 30,
        nav_map: str | None = None,
        nav_extra_args: str = "--smooth --no-viz --no-live-map --inflation 1",
        nav_use_external_astar: bool = False,
        rgbd_obs_near_m: float = 0.75,
        rgbd_obs_min_coverage: float = 0.18,
        lidar_ignore_near_m: float = 0.0,
        yaw_cmd_sign: float = 1.0,
        imu_debug: bool = False,
        imu_debug_interval: float = 0.5,
    ) -> None:
        self.iface = iface
        self.domain_id = int(domain_id)
        self.sport_topic = sport_topic
        self.lidar_map_topic = lidar_map_topic
        self.lidar_cloud_topic = lidar_cloud_topic
        self.slam_info_topic = slam_info_topic
        self.slam_key_topic = slam_key_topic

        self.rgb_port = int(rgb_port)
        self.depth_port = int(depth_port)
        self.rgb_width = int(rgb_width)
        self.rgb_height = int(rgb_height)
        self.rgb_fps = int(rgb_fps)

        self.nav_map = nav_map
        self.nav_extra_args = nav_extra_args
        self.nav_use_external_astar = bool(nav_use_external_astar)
        self.rgbd_obs_near_m = float(rgbd_obs_near_m)
        self.rgbd_obs_min_coverage = float(rgbd_obs_min_coverage)
        self.lidar_ignore_near_m = max(0.0, float(lidar_ignore_near_m))
        self.yaw_cmd_sign = -1.0 if float(yaw_cmd_sign) < 0.0 else 1.0
        self.imu_debug = bool(imu_debug)
        self.imu_debug_interval = max(0.05, float(imu_debug_interval))

        self._lock = threading.Lock()
        self._sport: SportModeState_ | None = None
        self._lidar_map: HeightMap_ | None = None
        self._lidar_cloud: PointCloud2_ | None = None
        self._last_sport_ts = 0.0
        self._last_lidar_map_ts = 0.0
        self._last_lidar_cloud_ts = 0.0

        self._sport_sub: ChannelSubscriber | None = None
        self._lidar_map_sub: ChannelSubscriber | None = None
        self._lidar_cloud_sub: ChannelSubscriber | None = None

        self._rgbd_proc: subprocess.Popen | None = None
        self._slam_proc: subprocess.Popen | None = None
        self._slam_log_fp: Any | None = None
        self._usb_proc: subprocess.Popen | None = None
        self.slam_is_running = False
        self._slam_save_dir: str | None = None

        self._path_points: list[tuple[float, float, float]] = []
        self._slam_service_windows: list[Any] = []
        self._nav_overlay_plan_file = Path(DEFAULT_NAV_OVERLAY_PLAN_FILE)
        self._slam_info_sub: Any | None = None

        if safety_boot:
            self._client = hanger_boot_sequence(iface=self.iface)
        else:
            ChannelFactoryInitialize(self.domain_id, self.iface)
            self._client = LocoClient()
            self._client.SetTimeout(10.0)
            self._client.Init()
        self._ensure_balanced_gait_mode()

        if auto_start_sensors:
            self.start_sensors()

    def set_imu_debug(self, enabled: bool = True, interval: float = 0.5) -> None:
        """Enable/disable IMU control-loop diagnostics."""
        self.imu_debug = bool(enabled)
        self.imu_debug_interval = max(0.05, float(interval))

    def _imu_log(self, message: str) -> None:
        print(f"[imu_ctrl] {message}")

    def _ensure_balanced_gait_mode(self) -> None:
        try:
            if hasattr(self._client, "BalanceStand"):
                self._client.BalanceStand(0)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Sensor subscriptions
    # ------------------------------------------------------------------

    def start_sensors(self) -> None:
        if self._sport_sub is None:
            self._sport_sub = ChannelSubscriber(self.sport_topic, SportModeState_)
            self._sport_sub.Init(self._sport_cb, 10)

        if self._lidar_map_sub is None:
            self._lidar_map_sub = ChannelSubscriber(self.lidar_map_topic, HeightMap_)
            self._lidar_map_sub.Init(self._lidar_map_cb, 10)

        if self._lidar_cloud_sub is None:
            self._lidar_cloud_sub = ChannelSubscriber(self.lidar_cloud_topic, PointCloud2_)
            self._lidar_cloud_sub.Init(self._lidar_cloud_cb, 10)

    def _sport_cb(self, msg: SportModeState_) -> None:
        with self._lock:
            self._sport = msg
            self._last_sport_ts = time.time()

    def _lidar_map_cb(self, msg: HeightMap_) -> None:
        with self._lock:
            self._lidar_map = msg
            self._last_lidar_map_ts = time.time()

    def _lidar_cloud_cb(self, msg: PointCloud2_) -> None:
        with self._lock:
            self._lidar_cloud = msg
            self._last_lidar_cloud_ts = time.time()

    # ------------------------------------------------------------------
    # Generic state helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _read_attr(obj: Any, *path: str) -> Any:
        cur = obj
        for name in path:
            if cur is None or not hasattr(cur, name):
                return None
            cur = getattr(cur, name)
        return cur

    @staticmethod
    def _vector3_from(value: Any) -> tuple[float, float, float] | None:
        try:
            if value is None:
                return None
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                return (float(value[0]), float(value[1]), float(value[2]))
        except Exception:
            return None
        return None

    def get_sport_state(self) -> SportModeState_ | None:
        with self._lock:
            return self._sport

    def get_lidar_map(self) -> HeightMap_ | None:
        with self._lock:
            return self._lidar_map

    def get_lidar_cloud(self) -> PointCloud2_ | None:
        with self._lock:
            return self._lidar_cloud

    def get_sensor_timestamps(self) -> dict[str, float]:
        with self._lock:
            return {
                "sport": float(self._last_sport_ts),
                "lidar_map": float(self._last_lidar_map_ts),
                "lidar_cloud": float(self._last_lidar_cloud_ts),
            }

    def sensors_stale(self, max_age: float = 1.0) -> dict[str, bool]:
        now = time.time()
        ts = self.get_sensor_timestamps()
        out: dict[str, bool] = {}
        for k, v in ts.items():
            out[k] = (v <= 0.0) or ((now - v) > max_age)
        return out

    def wait_for_sport_state(self, timeout: float = 2.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < max(0.0, timeout):
            if self.get_sport_state() is not None:
                return True
            time.sleep(0.05)
        return self.get_sport_state() is not None

    def get_mode(self) -> int | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        value = self._read_attr(msg, "mode")
        try:
            return int(value)
        except Exception:
            return None

    def get_gait(self) -> int | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        # field name differs across SDK revisions
        for key in ("gait_type", "gaitType", "gait"):
            value = self._read_attr(msg, key)
            if value is None:
                continue
            try:
                return int(value)
            except Exception:
                continue
        return None

    def get_body_height(self) -> float | None:
        msg = self.get_sport_state()
        if msg is None:
            return None
        for key in ("body_height", "bodyHeight", "stand_height", "standHeight"):
            value = self._read_attr(msg, key)
            if value is None:
                continue
            try:
                return float(value)
            except Exception:
                continue
        return None

    def get_position(self) -> tuple[float, float, float] | None:
        msg = self.get_sport_state()
        if msg is None:
            return None

        for key in ("position", "pos", "position_w"):
            vec = self._vector3_from(self._read_attr(msg, key))
            if vec is not None:
                return vec
        return None

    def get_velocity(self) -> tuple[float, float, float] | None:
        msg = self.get_sport_state()
        if msg is None:
            return None

        for key in ("velocity", "vel", "velocity_w"):
            vec = self._vector3_from(self._read_attr(msg, key))
            if vec is not None:
                return vec
        return None

    def get_yaw(self) -> float | None:
        imu = self.get_imu()
        if imu is None:
            return None
        return self._normalize_yaw_rad(float(imu.rpy[2]))

    def is_moving(self, linear_eps: float = 0.03, yaw_eps: float = 0.08) -> bool:
        v = self.get_velocity()
        if v is None:
            return False
        vx, vy, vz = v
        planar = math.hypot(vx, vy)
        return planar > linear_eps or abs(vz) > yaw_eps

    def get_robot_state(self) -> dict[str, Any]:
        """
        Consolidated state snapshot from cached DDS data + locomotion RPC.
        """
        return {
            "fsm": self.get_fsm(),
            "mode": self.get_mode(),
            "gait": self.get_gait(),
            "body_height": self.get_body_height(),
            "position": self.get_position(),
            "velocity": self.get_velocity(),
            "yaw": self.get_yaw(),
            "is_moving": self.is_moving(),
            "imu": self.get_imu(),
            "sensor_timestamps": self.get_sensor_timestamps(),
            "sensor_stale": self.sensors_stale(),
            "slam_is_running": bool(self.slam_is_running),
            "queued_path_points": len(self._path_points),
        }

    # ------------------------------------------------------------------
    # Locomotion + FSM
    # ------------------------------------------------------------------

    def loco_move(self, vx: float, vy: float, vyaw: float) -> int:
        return self._client.Move(float(vx), float(vy), float(vyaw), continous_move=True)

    def walk(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> int:
        """Balanced gait (type 0) then apply locomotion command."""
        self.set_gait_type(0)
        return int(self.loco_move(float(vx), float(vy), float(vyaw)))

    def run(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> int:
        """Continuous gait (type 1) then apply locomotion command."""
        self.set_gait_type(1)
        return int(self.loco_move(float(vx), float(vy), float(vyaw)))

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _normalize_yaw_rad(yaw_value: float) -> float:
        """
        Normalize IMU yaw to radians.
        Some SDK variants expose rpy in degrees.
        """
        y = float(yaw_value)
        if not math.isfinite(y):
            return 0.0
        if abs(y) > (2.0 * math.pi + 0.2):
            y = math.radians(y)
        return Robot._wrap_angle(y)

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _wait_for_live_pose_and_yaw(
        self,
        timeout: float = 2.0,
        tick: float = 0.05,
    ) -> tuple[tuple[float, float, float], float] | None:
        """
        Wait briefly for live sport-state pose + IMU yaw.
        Returns (position, yaw) when available, otherwise None.
        """
        t0 = time.time()
        self.wait_for_sport_state(timeout=min(max(0.0, float(timeout)), 0.5))
        while (time.time() - t0) <= max(0.0, float(timeout)):
            pos = self.get_position()
            yaw = self.get_yaw()
            if pos is not None and yaw is not None:
                return (pos, float(yaw))
            time.sleep(max(0.01, float(tick)))
        return None

    def _move_for_feedback(
        self,
        distance: float,
        gait_type: int,
        max_vx: float,
        max_vyaw: float,
        pos_tolerance: float,
        yaw_tolerance: float,
        timeout: float,
        tick: float,
        kp_lin: float,
        kp_yaw: float,
        feedback_wait: float = 2.0,
        debug: bool | None = None,
        debug_interval: float | None = None,
    ) -> bool:
        log_enabled = self.imu_debug if debug is None else bool(debug)
        log_interval = self.imu_debug_interval if debug_interval is None else max(0.05, float(debug_interval))

        live = self._wait_for_live_pose_and_yaw(timeout=feedback_wait, tick=tick)
        if live is None:
            ts = self.get_sensor_timestamps()
            stale = self.sensors_stale()
            if log_enabled:
                self._imu_log(
                    "startup failed: no live pose/yaw "
                    f"(sport_ts={ts.get('sport', 0.0):.3f}, sport_stale={stale.get('sport', True)})"
                )
            raise RuntimeError(
                "walk_for/run_for requires live position and IMU yaw "
                f"(sport_ts={ts.get('sport', 0.0):.3f}, sport_stale={stale.get('sport', True)})."
            )
        pos0, yaw0 = live

        d = float(distance)
        if abs(d) <= float(pos_tolerance):
            self.stop()
            return True

        sign = 1.0 if d >= 0.0 else -1.0
        yaw_ref = float(yaw0)
        target_x = float(pos0[0]) + d * math.cos(float(yaw0))
        target_y = float(pos0[1]) + d * math.sin(float(yaw0))
        heading_hold_radius = max(0.20, 4.0 * float(pos_tolerance))

        if log_enabled:
            self._imu_log(
                "start move "
                f"distance={d:+.3f}m gait={int(gait_type)} "
                f"start=({float(pos0[0]):+.3f},{float(pos0[1]):+.3f}) yaw0={float(yaw0):+.3f}rad "
                f"target=({target_x:+.3f},{target_y:+.3f}) "
                f"kp_lin={float(kp_lin):.2f} kp_yaw={float(kp_yaw):.2f} "
                f"max_vx={float(max_vx):.2f} max_vyaw={float(max_vyaw):.2f}"
            )

        self.set_gait_type(int(gait_type))
        t0 = time.time()
        ok = False
        last_dist = float("nan")
        last_heading_err = float("nan")
        missing_count = 0
        next_log_ts = t0
        try:
            while (time.time() - t0) <= max(0.1, float(timeout)):
                now = time.time()
                pos = self.get_position()
                yaw = self.get_yaw()
                if pos is None or yaw is None:
                    missing_count += 1
                    if log_enabled and now >= next_log_ts:
                        ts = self.get_sensor_timestamps()
                        stale = self.sensors_stale()
                        self._imu_log(
                            "feedback missing "
                            f"count={missing_count} sport_ts={ts.get('sport', 0.0):.3f} "
                            f"sport_stale={stale.get('sport', True)}"
                        )
                        next_log_ts = now + log_interval
                    time.sleep(max(0.01, float(tick)))
                    continue

                dx = target_x - float(pos[0])
                dy = target_y - float(pos[1])
                dist = math.hypot(dx, dy)
                last_dist = dist
                if dist <= float(pos_tolerance):
                    ok = True
                    last_heading_err = 0.0
                    break

                target_heading_to_goal = math.atan2(dy, dx)
                if dist < heading_hold_radius:
                    # Near the target, hold the original travel heading to avoid
                    # atan2 jitter that can trigger rotate-in-place behavior.
                    blend = self._clamp(dist / heading_hold_radius, 0.0, 1.0)
                    d_heading = self._wrap_angle(target_heading_to_goal - yaw_ref)
                    target_heading = self._wrap_angle(yaw_ref + blend * d_heading)
                else:
                    target_heading = target_heading_to_goal
                heading_err = self._wrap_angle(target_heading - float(yaw))
                last_heading_err = heading_err

                yaw_gate_on = (abs(heading_err) > float(yaw_tolerance)) and (dist >= heading_hold_radius)
                if yaw_gate_on:
                    vx_cmd = 0.0
                else:
                    vx_cmd = sign * self._clamp(float(kp_lin) * dist, 0.0, max(0.0, float(max_vx)))
                vyaw_cmd = self._clamp(
                    float(kp_yaw) * heading_err,
                    -max(0.0, float(max_vyaw)),
                    max(0.0, float(max_vyaw)),
                )
                vyaw_cmd *= self.yaw_cmd_sign
                self.loco_move(vx_cmd, 0.0, vyaw_cmd)
                if log_enabled and now >= next_log_ts:
                    elapsed = now - t0
                    self._imu_log(
                        f"t={elapsed:5.2f}s pos=({float(pos[0]):+.3f},{float(pos[1]):+.3f}) yaw={float(yaw):+.3f} "
                        f"dx={dx:+.3f} dy={dy:+.3f} dist={dist:.3f} "
                        f"heading_err={heading_err:+.3f}rad({math.degrees(heading_err):+.1f}deg) "
                        f"cmd_vx={vx_cmd:+.3f} cmd_vyaw={vyaw_cmd:+.3f} "
                        f"yaw_gate={'ON' if yaw_gate_on else 'off'}"
                    )
                    next_log_ts = now + log_interval
                time.sleep(max(0.01, float(tick)))
        finally:
            self.stop()
            if log_enabled:
                elapsed = time.time() - t0
                self._imu_log(
                    "end move "
                    f"ok={ok} elapsed={elapsed:.2f}s "
                    f"final_dist={last_dist:.3f} final_heading_err={last_heading_err:+.3f}rad "
                    f"missing_feedback_count={missing_count}"
                )
        return ok

    def walk_for(
        self,
        distance: float,
        max_vx: float = 0.25,
        max_vyaw: float = 0.5,
        pos_tolerance: float = 0.05,
        yaw_tolerance: float = 0.20,
        timeout: float = 20.0,
        tick: float = 0.05,
        kp_lin: float = 0.9,
        kp_yaw: float = 1.6,
        debug: bool | None = None,
        debug_interval: float | None = None,
    ) -> bool:
        """
        Balanced gait (type 0), move for a relative distance (meters) with
        IMU/pose feedback correction toward the target pose.
        """
        return self._move_for_feedback(
            distance=distance,
            gait_type=0,
            max_vx=max_vx,
            max_vyaw=max_vyaw,
            pos_tolerance=pos_tolerance,
            yaw_tolerance=yaw_tolerance,
            timeout=timeout,
            tick=tick,
            kp_lin=kp_lin,
            kp_yaw=kp_yaw,
            feedback_wait=2.0,
            debug=debug,
            debug_interval=debug_interval,
        )

    def run_for(
        self,
        distance: float,
        max_vx: float = 0.45,
        max_vyaw: float = 0.8,
        pos_tolerance: float = 0.07,
        yaw_tolerance: float = 0.25,
        timeout: float = 15.0,
        tick: float = 0.05,
        kp_lin: float = 1.0,
        kp_yaw: float = 1.8,
        debug: bool | None = None,
        debug_interval: float | None = None,
    ) -> bool:
        """
        Continuous gait (type 1), move for a relative distance (meters) with
        IMU/pose feedback correction toward the target pose.
        """
        return self._move_for_feedback(
            distance=distance,
            gait_type=1,
            max_vx=max_vx,
            max_vyaw=max_vyaw,
            pos_tolerance=pos_tolerance,
            yaw_tolerance=yaw_tolerance,
            timeout=timeout,
            tick=tick,
            kp_lin=kp_lin,
            kp_yaw=kp_yaw,
            feedback_wait=2.0,
            debug=debug,
            debug_interval=debug_interval,
        )

    def turn_for(
        self,
        angle_deg: float,
        max_vyaw: float = 0.30,
        min_vyaw: float = 0.06,
        yaw_tolerance_deg: float = 1.5,
        timeout: float = 10.0,
        tick: float = 0.05,
        kp_yaw: float = 0.95,
        kd_yaw: float = 0.12,
        settle_cycles: int = 4,
        gait_type: int = 0,
        debug: bool | None = None,
        debug_interval: float | None = None,
    ) -> bool:
        """
        Turn in place by a relative angle in degrees (can be negative).
        Uses IMU yaw feedback to converge accurately to the target heading.
        """
        log_enabled = self.imu_debug if debug is None else bool(debug)
        log_interval = self.imu_debug_interval if debug_interval is None else max(0.05, float(debug_interval))

        live = self._wait_for_live_pose_and_yaw(timeout=2.0, tick=tick)
        if live is None:
            ts = self.get_sensor_timestamps()
            stale = self.sensors_stale()
            if log_enabled:
                self._imu_log(
                    "turn startup failed: no live yaw "
                    f"(sport_ts={ts.get('sport', 0.0):.3f}, sport_stale={stale.get('sport', True)})"
                )
            raise RuntimeError(
                "turn_for requires live IMU yaw "
                f"(sport_ts={ts.get('sport', 0.0):.3f}, sport_stale={stale.get('sport', True)})."
            )
        _, yaw0 = live

        delta = math.radians(float(angle_deg))
        tol = math.radians(max(0.1, float(yaw_tolerance_deg)))
        if abs(delta) <= tol:
            self.stop()
            return True

        target = self._wrap_angle(float(yaw0) + delta)
        self.set_gait_type(int(gait_type))

        if log_enabled:
            self._imu_log(
                "start turn "
                f"delta={float(angle_deg):+.2f}deg yaw0={float(yaw0):+.3f}rad "
                f"target={target:+.3f}rad tol={tol:.3f}rad({math.degrees(tol):.2f}deg) "
                f"kp_yaw={float(kp_yaw):.2f} kd_yaw={float(kd_yaw):.2f} "
                f"max_vyaw={float(max_vyaw):.2f} min_vyaw={max(0.0, float(min_vyaw)):.2f} "
                f"settle_cycles={max(1, int(settle_cycles))}"
            )

        t0 = time.time()
        ok = False
        last_err = float("nan")
        stable_hits = 0
        last_err_abs = float("inf")
        no_progress_hits = 0
        missing_count = 0
        next_log_ts = t0
        stalled_limit = max(3, int(0.4 / max(0.01, float(tick))))
        try:
            while (time.time() - t0) <= max(0.1, float(timeout)):
                now = time.time()
                yaw = self.get_yaw()
                if yaw is None:
                    missing_count += 1
                    if log_enabled and now >= next_log_ts:
                        ts = self.get_sensor_timestamps()
                        stale = self.sensors_stale()
                        self._imu_log(
                            "turn feedback missing "
                            f"count={missing_count} sport_ts={ts.get('sport', 0.0):.3f} "
                            f"sport_stale={stale.get('sport', True)}"
                        )
                        next_log_ts = now + log_interval
                    time.sleep(max(0.01, float(tick)))
                    continue
                err = self._wrap_angle(target - float(yaw))
                last_err = err
                imu = self.get_imu()
                yaw_rate = 0.0
                if imu is not None and imu.gyro is not None:
                    try:
                        yaw_rate = float(imu.gyro[2])
                        if not math.isfinite(yaw_rate):
                            yaw_rate = 0.0
                    except Exception:
                        yaw_rate = 0.0

                err_abs = abs(err)
                within_tol = err_abs <= tol
                if within_tol and abs(yaw_rate) <= math.radians(6.0):
                    stable_hits += 1
                else:
                    stable_hits = 0
                if err_abs < (last_err_abs - math.radians(0.10)):
                    no_progress_hits = 0
                else:
                    no_progress_hits += 1
                last_err_abs = err_abs

                if stable_hits >= max(1, int(settle_cycles)):
                    ok = True
                    break
                if no_progress_hits >= stalled_limit and err_abs <= max(1.5 * tol, math.radians(2.5)):
                    ok = True
                    break

                # Conservative near-target profile to avoid overshoot.
                if err_abs >= math.radians(30.0):
                    max_cmd = max(0.0, float(max_vyaw))
                elif err_abs >= math.radians(10.0):
                    k = (err_abs - math.radians(10.0)) / math.radians(20.0)
                    max_cmd = 0.12 + k * (max(0.0, float(max_vyaw)) - 0.12)
                else:
                    max_cmd = min(max(0.0, float(max_vyaw)), 0.12)

                # PD heading controller (uses IMU yaw rate damping when available).
                vyaw_raw = float(kp_yaw) * err - float(kd_yaw) * yaw_rate
                vyaw_cmd = self._clamp(vyaw_raw, -max_cmd, max_cmd)

                # Adaptive floor to overcome actuator deadband without forcing overshoot near tolerance.
                min_cmd = min(max(0.0, float(min_vyaw)), max_cmd)
                needs_deadband_boost = (err_abs >= (2.0 * tol)) or (no_progress_hits >= stalled_limit)
                if (not within_tol) and needs_deadband_boost and abs(vyaw_cmd) < min_cmd:
                    vyaw_cmd = min_cmd if vyaw_cmd >= 0.0 else -min_cmd

                vyaw_cmd *= self.yaw_cmd_sign
                self.loco_move(0.0, 0.0, vyaw_cmd)
                if log_enabled and now >= next_log_ts:
                    elapsed = now - t0
                    self._imu_log(
                        f"turn t={elapsed:5.2f}s yaw={float(yaw):+.3f} err={err:+.3f}rad({math.degrees(err):+.1f}deg) "
                        f"yaw_rate={yaw_rate:+.3f} within_tol={within_tol} "
                        f"stable={stable_hits}/{max(1, int(settle_cycles))} no_progress={no_progress_hits} "
                        f"max_cmd={max_cmd:.3f} raw_vyaw={vyaw_raw:+.3f} cmd_vyaw={vyaw_cmd:+.3f}"
                    )
                    next_log_ts = now + log_interval
                time.sleep(max(0.01, float(tick)))
        finally:
            self.stop()
            if log_enabled:
                elapsed = time.time() - t0
                self._imu_log(
                    "end turn "
                    f"ok={ok} elapsed={elapsed:.2f}s final_err={last_err:+.3f}rad "
                    f"missing_feedback_count={missing_count}"
                )
        return ok

    def stop(self) -> None:
        """Stop locomotion motion commands."""
        if hasattr(self._client, "StopMove"):
            self._client.StopMove()
        else:
            self._client.Move(0.0, 0.0, 0.0, continous_move=False)

    def stop_moving(self) -> None:
        """Backward-compatible alias for stop()."""
        self.stop()

    @staticmethod
    def _normalize_gait_type(gait_type: int | str) -> int:
        if isinstance(gait_type, str):
            key = gait_type.strip().lower().replace("-", "_").replace(" ", "_")
            alias = {
                "normal": 0,
                "balanced": 0,
                "balance": 0,
                "static": 0,
                "stand": 0,
                "continuous": 1,
                "walk": 1,
                "walking": 1,
                "dynamic": 1,
            }
            if key not in alias:
                raise ValueError(f"Unknown gait_type '{gait_type}'.")
            return int(alias[key])
        return int(gait_type)

    def set_lidar_ignore_near(self, meters: float = 0.0) -> float:
        """
        Ignore lidar/map obstacles within this radius (meters) around robot pose
        when preparing the navigation occupancy map.
        """
        self.lidar_ignore_near_m = max(0.0, float(meters))
        return self.lidar_ignore_near_m

    def set_gait_type(self, gait_type: int | str = 0) -> int:
        """
        Set locomotion gait/balance mode.

        Accepts integer mode or aliases:
          - 0: normal/balanced/static stand
          - 1: continuous walking mode
        """
        mode = self._normalize_gait_type(gait_type)
        if hasattr(self._client, "SetGaitType"):
            return int(self._client.SetGaitType(mode))
        if hasattr(self._client, "SetBalanceMode"):
            return int(self._client.SetBalanceMode(mode))
        raise AttributeError("Current locomotion client does not support gait mode setting API.")

    def _rpc_get_int(self, api_id: int) -> Optional[int]:
        try:
            code, data = self._client._Call(api_id, "{}")  # type: ignore[attr-defined]
            if code != 0 or not data:
                return None
            return int(json.loads(data).get("data"))
        except Exception:
            return None

    def get_fsm(self) -> dict[str, Optional[int]]:
        try:
            from unitree_sdk2py.g1.loco.g1_loco_api import (
                ROBOT_API_ID_LOCO_GET_FSM_ID,
                ROBOT_API_ID_LOCO_GET_FSM_MODE,
            )
        except Exception:
            return {"id": None, "mode": None}

        return {
            "id": self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_ID),
            "mode": self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_MODE),
        }

    def fsm_0_zt(self) -> None:
        if hasattr(self._client, "ZeroTorque"):
            self._client.ZeroTorque()
        elif hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(0)

    def fsm_1_damp(self) -> None:
        if hasattr(self._client, "Damp"):
            self._client.Damp()
        elif hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(1)

    def fsm_2_squat(self) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(2)

    # ------------------------------------------------------------------
    # IMU + lidar getters
    # ------------------------------------------------------------------

    def get_imu(self) -> ImuData | None:
        with self._lock:
            msg = self._sport
        if msg is None:
            return None

        rpy = (0.0, 0.0, 0.0)
        gyro = acc = quat = None
        temp = None

        try:
            rpy = (
                float(msg.imu_state.rpy[0]),
                float(msg.imu_state.rpy[1]),
                float(msg.imu_state.rpy[2]),
            )
        except Exception:
            pass
        try:
            gyro = (
                float(msg.imu_state.gyroscope[0]),
                float(msg.imu_state.gyroscope[1]),
                float(msg.imu_state.gyroscope[2]),
            )
        except Exception:
            pass
        try:
            acc = (
                float(msg.imu_state.accelerometer[0]),
                float(msg.imu_state.accelerometer[1]),
                float(msg.imu_state.accelerometer[2]),
            )
        except Exception:
            pass
        try:
            quat = (
                float(msg.imu_state.quaternion[0]),
                float(msg.imu_state.quaternion[1]),
                float(msg.imu_state.quaternion[2]),
                float(msg.imu_state.quaternion[3]),
            )
        except Exception:
            pass
        try:
            temp = float(msg.imu_state.temperature)
        except Exception:
            pass

        return ImuData(rpy=rpy, gyro=gyro, acc=acc, quat=quat, temp=temp)

    @staticmethod
    def _extract_xyz_from_cloud(msg: PointCloud2_, max_points: int | None = None) -> list[tuple[float, float, float]]:
        try:
            width = int(msg.width)
            height = int(msg.height)
            point_step = int(msg.point_step)
            raw = bytes(msg.data)
        except Exception:
            return []

        if point_step <= 0:
            return []

        x_off, y_off, z_off = 0, 4, 8
        try:
            fields = list(msg.fields)
            name_to_off = {str(f.name).lower(): int(f.offset) for f in fields}
            x_off = name_to_off.get("x", x_off)
            y_off = name_to_off.get("y", y_off)
            z_off = name_to_off.get("z", z_off)
        except Exception:
            pass

        total = max(0, width * height)
        if max_points is not None:
            total = min(total, max_points)

        import struct

        out: list[tuple[float, float, float]] = []
        for i in range(total):
            base = i * point_step
            try:
                x = struct.unpack_from("<f", raw, base + x_off)[0]
                y = struct.unpack_from("<f", raw, base + y_off)[0]
                z = struct.unpack_from("<f", raw, base + z_off)[0]
            except Exception:
                break
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                out.append((float(x), float(y), float(z)))
        return out

    def get_lidar_points(self, max_points: int | None = 20000) -> list[tuple[float, float, float]]:
        with self._lock:
            msg = self._lidar_cloud
        if msg is None:
            return []
        return self._extract_xyz_from_cloud(msg, max_points=max_points)

    # ------------------------------------------------------------------
    # RGBD GST helpers
    # ------------------------------------------------------------------

    def get_rgbd_gst(self, detect: str = "human") -> subprocess.Popen:
        """
        Start a local RGBD GST viewer.

        detect="" (or "none") => plain RGBD viewer.
        detect="human" (or any text) => CLIP detector viewer with custom prompt.
        """
        if self._rgbd_proc is not None and self._rgbd_proc.poll() is None:
            return self._rgbd_proc

        plain_script = SCRIPTS_ROOT / "sensors" / "manual_streaming" / "receive_realsense_gst.py"
        clip_script = SCRIPTS_ROOT / "sensors" / "manual_streaming" / "receive_realsense_gst_clip_can.py"

        det = (detect or "").strip().lower()
        if det in ("", "none", "off"):
            cmd = [sys.executable, str(plain_script)]
        else:
            prompt = f"a photo of a {det}"
            negative = f"a photo without a {det}"
            cmd = [
                sys.executable,
                str(clip_script),
                "--positive",
                prompt,
                "--negative",
                negative,
                "--threshold",
                "0.55",
            ]
            cmd.extend(
                [
                    "--rgb-port",
                    str(self.rgb_port),
                    "--depth-port",
                    str(self.depth_port),
                    "--width",
                    str(self.rgb_width),
                    "--height",
                    str(self.rgb_height),
                    "--fps",
                    str(self.rgb_fps),
                ]
            )

        self._rgbd_proc = subprocess.Popen(cmd, cwd=str(SCRIPTS_ROOT))
        return self._rgbd_proc

    # ------------------------------------------------------------------
    # Local SLAM helpers (live_slam_save.py)
    # ------------------------------------------------------------------

    def start_slam(
        self,
        save_folder: str = "./maps",
        save_every: int = 1,
        save_latest: bool = True,
        save_prefix: str = "live_slam_latest",
        viz: bool = False,
    ) -> subprocess.Popen:
        if self._slam_proc is not None and self._slam_proc.poll() is None:
            self.slam_is_running = True
            return self._slam_proc

        if viz:
            slam_script = DEV_OTHER_DIR / "other" / "slam_viz_nav_overlay_runner.py"
            slam_cwd = SCRIPTS_ROOT / "navigation" / "obstacle_avoidance"
        else:
            slam_script = DEV_OTHER_DIR / "other" / "slam_headless_save_runner.py"
            slam_cwd = slam_script.parent
        save_dir = Path(save_folder)
        if not save_dir.is_absolute():
            save_dir = (slam_cwd / save_dir).resolve()
        save_dir.mkdir(parents=True, exist_ok=True)

        cmd = [
            sys.executable,
            str(slam_script),
            "--save-dir",
            str(save_dir),
            "--save-every",
            str(max(1, int(save_every))),
            "--save-prefix",
            save_prefix,
        ]
        if save_latest:
            cmd.append("--save-latest")
        if viz:
            cmd.extend(
                [
                    "--iface",
                    self.iface,
                    "--domain-id",
                    str(self.domain_id),
                    "--overlay-plan-file",
                    str(self._nav_overlay_plan_file),
                ]
            )

        log_path = save_dir / "slam_runtime.log"
        self._slam_log_fp = open(log_path, "a", encoding="utf-8", buffering=1)
        self._slam_log_fp.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}] start viz={bool(viz)} cmd={cmd}\n")
        self._slam_proc = subprocess.Popen(
            cmd,
            cwd=str(slam_cwd),
            stdin=subprocess.DEVNULL,
            stdout=self._slam_log_fp,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        startup_deadline = time.time() + (5.0 if viz else 0.35)
        while time.time() < startup_deadline and self._slam_proc.poll() is None:
            time.sleep(0.1)

        if self._slam_proc.poll() is not None:
            tail = ""
            try:
                with open(log_path, "r", encoding="utf-8") as fp:
                    lines = fp.readlines()
                tail = "".join(lines[-40:]).strip()
            except Exception:
                pass
            tail_lower = tail.lower()
            viz_init_failed = (
                "failed to initialize glew" in tail_lower
                or "glfw error" in tail_lower
                or "failed to initialize gtk" in tail_lower
            )
            self.slam_is_running = False
            self._slam_proc = None
            if self._slam_log_fp is not None:
                try:
                    self._slam_log_fp.close()
                except Exception:
                    pass
                self._slam_log_fp = None
            if viz and viz_init_failed:
                # Typical on headless/Wayland sessions where Open3D GUI cannot be created.
                print("[start_slam] Visualization init failed; falling back to headless SLAM runner.")
                return self.start_slam(
                    save_folder=str(save_dir),
                    save_every=save_every,
                    save_latest=save_latest,
                    save_prefix=save_prefix,
                    viz=False,
                )
            detail = f"\nSLAM log tail:\n{tail}" if tail else ""
            raise RuntimeError(f"Failed to start SLAM process (viz={bool(viz)}).{detail}")
        self._slam_save_dir = str(save_dir)
        if save_latest:
            self.nav_map = str(save_dir / f"{save_prefix}_latest.pcd")
        self.slam_is_running = True
        return self._slam_proc

    def stop_slam(self, save_folder: str = "./maps") -> None:
        """
        Stop local SLAM subprocess started by start_slam().
        save_folder is kept for API compatibility with local live_slam_save usage.
        """
        _ = save_folder  # intentionally accepted even when process already has its own save-dir

        proc = self._slam_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5.0)
            except Exception:
                try:
                    proc.terminate()
                    proc.wait(timeout=3.0)
                except Exception:
                    proc.kill()
        self._slam_proc = None
        self._clear_nav_overlay_plan()
        if self._slam_log_fp is not None:
            try:
                self._slam_log_fp.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] stop\n")
                self._slam_log_fp.close()
            except Exception:
                pass
            self._slam_log_fp = None
        self.slam_is_running = False

    # ------------------------------------------------------------------
    # Single-call convenience API
    # ------------------------------------------------------------------

    def slam_start(
        self,
        viz: bool = False,
        save_folder: str = "./maps",
        save_every: int = 1,
        save_latest: bool = True,
        save_prefix: str = "live_slam_latest",
    ) -> bool:
        self.start_slam(
            save_folder=save_folder,
            save_every=save_every,
            save_latest=save_latest,
            save_prefix=save_prefix,
            viz=viz,
        )
        return bool(self.slam_is_running)

    def slam_stop(self, save_folder: str = "./maps") -> None:
        self.stop_slam(save_folder=save_folder)

    def slam_add_pose(
        self,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_slam_pose: bool = True,
    ) -> tuple[float, float, float]:
        if x is None or y is None:
            pose = self.get_slam_pose(timeout_s=0.35) if bool(use_slam_pose) and self.slam_is_running else None
            if pose is None:
                pos = self.get_position()
                if pos is None:
                    raise RuntimeError("No current pose available from SLAM or sport state.")
                x = float(pos[0])
                y = float(pos[1])
                if yaw is None:
                    yv = self.get_yaw()
                    yaw = float(yv) if yv is not None else 0.0
            else:
                x = float(pose[0])
                y = float(pose[1])
                if yaw is None:
                    yaw = float(pose[2])
        if yaw is None:
            yaw = 0.0
        self.set_path_point(float(x), float(y), float(yaw))
        return (float(x), float(y), float(yaw))

    def slam_nav_pose(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        obs_avoid: bool = False,
        use_dynamic_fallback: bool = True,
        use_rgbd_depth_guard: bool = True,
    ) -> int:
        """
        Navigate to a single pose.
        - obs_avoid=True: use local dynamic planner.
        - obs_avoid=False: try robot pose_nav API first.
        """
        try:
            self.set_gait_type(0)
        except Exception:
            pass

        if bool(obs_avoid):
            return int(self._run_dynamic_nav(float(x), float(y), use_rgbd_depth_guard=bool(use_rgbd_depth_guard)))

        rc = int(self._run_pose_nav(float(x), float(y), float(yaw)))
        if rc != 0 and bool(use_dynamic_fallback):
            print(f"[nav_pose] pose_nav rejected (rc={rc}); trying local dynamic nav fallback.")
            return int(self._run_dynamic_nav(float(x), float(y), use_rgbd_depth_guard=bool(use_rgbd_depth_guard)))
        return rc

    def slam_nav_path(
        self,
        points: list[tuple[float, float] | tuple[float, float, float]],
        obs_avoid: bool = True,
        clear_on_finish: bool = True,
        append: bool = False,
    ) -> bool:
        """
        Convenience wrapper to navigate a path in one call.
        points accepts:
          - (x, y)
          - (x, y, yaw)
        """
        if not bool(append):
            self.clear_path_points()
        for p in points:
            if len(p) == 2:
                x, y = p
                yaw = 0.0
            elif len(p) == 3:
                x, y, yaw = p
            else:
                raise ValueError(f"Invalid path point {p!r}; expected (x,y) or (x,y,yaw).")
            self.set_path_point(float(x), float(y), float(yaw))
        return bool(self.navigate_path(obs_avoid=bool(obs_avoid), clear_on_finish=bool(clear_on_finish)))

    # ------------------------------------------------------------------
    # Path points + navigation
    # ------------------------------------------------------------------

    def slam_set_path_point(self, x: float, y: float, yaw: float = 0.0) -> None:
        if not self.slam_is_running:
            raise RuntimeError("set_path_point is only allowed while slam_is_running=True")
        self._path_points.append((float(x), float(y), float(yaw)))

    def get_path_points(self) -> list[tuple[float, float, float]]:
        return list(self._path_points)

    def clear_path_points(self) -> None:
        self._path_points.clear()

    def _clear_nav_overlay_plan(self) -> None:
        try:
            if self._nav_overlay_plan_file.exists():
                self._nav_overlay_plan_file.unlink()
        except Exception:
            pass

    def _write_nav_overlay_plan(
        self,
        start_xy: tuple[float, float] | None,
        goal_xy: tuple[float, float] | None,
        path_xy: list[tuple[float, float]] | None = None,
        source: str = "preview",
    ) -> None:
        if goal_xy is None:
            return
        payload: dict[str, Any] = {
            "source": str(source),
            "timestamp": time.time(),
            "goal": {"x": float(goal_xy[0]), "y": float(goal_xy[1])},
            "path": [],
        }
        if start_xy is not None:
            payload["start"] = {"x": float(start_xy[0]), "y": float(start_xy[1])}
        pts = path_xy or []
        if not pts and start_xy is not None:
            pts = [start_xy, goal_xy]
        payload["path"] = [{"x": float(px), "y": float(py)} for px, py in pts]
        try:
            tmp = self._nav_overlay_plan_file.with_suffix(".tmp")
            tmp.write_text(json.dumps(payload, ensure_ascii=True), encoding="utf-8")
            tmp.replace(self._nav_overlay_plan_file)
        except Exception:
            pass

    def _ensure_slam_info_sub(self) -> Any | None:
        if self._slam_info_sub is not None:
            return self._slam_info_sub
        try:
            from navigation.obstacle_avoidance.slam_map import SlamInfoSubscriber
            sub = SlamInfoSubscriber(self.slam_info_topic, self.slam_key_topic)
            sub.start()
            self._slam_info_sub = sub
            return sub
        except Exception:
            return None

    @staticmethod
    def _parse_slam_pose_payload(payload_raw: str | None) -> tuple[float, float, float] | None:
        if not payload_raw:
            return None
        try:
            payload = json.loads(payload_raw)
        except Exception:
            return None
        if not isinstance(payload, dict):
            return None
        data = payload.get("data")
        if not isinstance(data, dict):
            return None
        cur = data.get("currentPose")
        if not isinstance(cur, dict):
            return None
        try:
            x = float(cur.get("x"))
            y = float(cur.get("y"))
        except Exception:
            return None
        # Prefer yaw from quaternion when available.
        yaw = 0.0
        try:
            qx = float(cur.get("q_x", 0.0))
            qy = float(cur.get("q_y", 0.0))
            qz = float(cur.get("q_z", 0.0))
            qw = float(cur.get("q_w", 1.0))
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            try:
                yaw = float(cur.get("yaw", 0.0))
            except Exception:
                yaw = 0.0
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return None
        return (x, y, yaw)

    def get_slam_pose(self, timeout_s: float = 0.4) -> tuple[float, float, float] | None:
        sub = self._ensure_slam_info_sub()
        if sub is None:
            return None
        t0 = time.time()
        while time.time() - t0 < max(0.05, float(timeout_s)):
            for payload in (sub.get_info(), sub.get_key()):
                pose = self._parse_slam_pose_payload(payload)
                if pose is not None:
                    return pose
            time.sleep(0.03)
        return None

    @staticmethod
    def _heightmap_to_occupancy(
        height_map_msg: HeightMap_,
        height_threshold: float = 0.15,
        ignore_near_center_xy: tuple[float, float] | None = None,
        ignore_near_m: float = 0.0,
    ) -> Any | None:
        try:
            import numpy as np
            from navigation.obstacle_avoidance.create_map import OccupancyGrid
        except Exception:
            return None
        try:
            width = int(height_map_msg.width)
            height = int(height_map_msg.height)
            resolution = float(height_map_msg.resolution)
            data = np.array(list(height_map_msg.data), dtype=float)
            if width <= 0 or height <= 0 or resolution <= 0:
                return None
            if data.size != width * height:
                return None
            heights = data.reshape((height, width))
            occ = (heights >= float(height_threshold)).astype(np.int8)
            origin_x = -width * resolution / 2.0
            origin_y = -height * resolution / 2.0
            out = OccupancyGrid(
                width_m=width * resolution,
                height_m=height * resolution,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
            )
            out.grid = occ
            if ignore_near_center_xy is not None and float(ignore_near_m) > 0.0:
                cx, cy = float(ignore_near_center_xy[0]), float(ignore_near_center_xy[1])
                rr = max(1, int(float(ignore_near_m) / float(resolution)))
                r0, c0 = out.world_to_grid(cx, cy)
                import numpy as np
                y, x = np.ogrid[-rr : rr + 1, -rr : rr + 1]
                mask = (x * x + y * y) <= (rr * rr)
                r_lo = max(0, r0 - rr)
                r_hi = min(out.height_cells - 1, r0 + rr)
                c_lo = max(0, c0 - rr)
                c_hi = min(out.width_cells - 1, c0 + rr)
                sub = out.grid[r_lo : r_hi + 1, c_lo : c_hi + 1]
                mr0 = rr - (r0 - r_lo)
                mc0 = rr - (c0 - c_lo)
                sub[mask[mr0 : mr0 + sub.shape[0], mc0 : mc0 + sub.shape[1]]] = 0
            return out
        except Exception:
            return None

    def _filter_nav_map_near_points(
        self,
        map_path: str,
        start_xy: tuple[float, float] | None,
    ) -> str:
        radius_m = float(self.lidar_ignore_near_m)
        if radius_m <= 0.0 or start_xy is None:
            return str(map_path)
        try:
            from navigation.obstacle_avoidance.create_map import OccupancyGrid, load_from_point_cloud
            import numpy as np
        except Exception:
            return str(map_path)
        try:
            lower = str(map_path).lower()
            if lower.endswith((".pcd", ".ply", ".xyz", ".xyzn", ".xyzrgb")):
                occ = load_from_point_cloud(
                    str(map_path),
                    resolution=0.1,
                    padding_m=0.5,
                    height_threshold=0.15,
                    max_height=None,
                    origin_centered=True,
                )
            else:
                occ = OccupancyGrid.load(str(map_path))
            rr = max(1, int(radius_m / float(occ.resolution)))
            r0, c0 = occ.world_to_grid(float(start_xy[0]), float(start_xy[1]))
            y, x = np.ogrid[-rr : rr + 1, -rr : rr + 1]
            mask = (x * x + y * y) <= (rr * rr)
            r_lo = max(0, r0 - rr)
            r_hi = min(occ.height_cells - 1, r0 + rr)
            c_lo = max(0, c0 - rr)
            c_hi = min(occ.width_cells - 1, c0 + rr)
            sub = occ.grid[r_lo : r_hi + 1, c_lo : c_hi + 1]
            mr0 = rr - (r0 - r_lo)
            mc0 = rr - (c0 - c_lo)
            sub[mask[mr0 : mr0 + sub.shape[0], mc0 : mc0 + sub.shape[1]]] = 0
            with tempfile.NamedTemporaryFile(prefix="g1_nav_filtered_", suffix=".npz", delete=False) as tf:
                out = tf.name
            occ.save(out)
            return out
        except Exception:
            return str(map_path)

    def _preview_path_from_map(
        self,
        map_path: str,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        inflation: int = 1,
        allow_diagonal: bool = False,
        smooth: bool = True,
    ) -> list[tuple[float, float]] | None:
        try:
            from navigation.obstacle_avoidance.create_map import OccupancyGrid, load_from_point_cloud
            from navigation.obstacle_avoidance.path_planner import astar, smooth_path
        except Exception:
            return None
        try:
            lower = str(map_path).lower()
            if lower.endswith((".pcd", ".ply", ".xyz", ".xyzn", ".xyzrgb")):
                occ = load_from_point_cloud(
                    str(map_path),
                    resolution=0.1,
                    padding_m=0.5,
                    height_threshold=0.15,
                    max_height=None,
                    origin_centered=True,
                )
            else:
                occ = OccupancyGrid.load(str(map_path))
            plan_grid = occ.inflate(max(0, int(inflation))) if inflation > 0 else occ.grid.copy()
            s_rc = occ.world_to_grid(float(start_xy[0]), float(start_xy[1]))
            g_rc = occ.world_to_grid(float(goal_xy[0]), float(goal_xy[1]))
            path = astar(plan_grid, s_rc, g_rc, allow_diagonal=bool(allow_diagonal))
            if not path:
                return None
            if smooth:
                path = smooth_path(path, plan_grid, max_skip=5)
            return [occ.grid_to_world(r, c) for r, c in path]
        except Exception:
            return None

    def _snapshot_live_slam_map_npz(self, timeout_s: float = 2.0) -> str | None:
        """
        Snapshot current rt/utlidar map_state into a temporary .npz map file.
        This keeps planner map frame aligned with robot pose frame.
        """
        # Fast path: use already cached HeightMap_ from this process.
        slam_pose = self.get_slam_pose(timeout_s=0.20)
        pos = self.get_position()
        near_center = (
            (float(slam_pose[0]), float(slam_pose[1])) if slam_pose is not None
            else ((float(pos[0]), float(pos[1])) if pos is not None else None)
        )
        with self._lock:
            hm = self._lidar_map
            hm_ts = self._last_lidar_map_ts
        if hm is not None and (time.time() - float(hm_ts)) <= max(0.2, float(timeout_s) + 0.5):
            occ = self._heightmap_to_occupancy(
                hm,
                height_threshold=0.15,
                ignore_near_center_xy=near_center,
                ignore_near_m=self.lidar_ignore_near_m,
            )
            if occ is not None:
                try:
                    with tempfile.NamedTemporaryFile(prefix="g1_live_nav_", suffix=".npz", delete=False) as tf:
                        out = tf.name
                    occ.save(out)
                    return out
                except Exception:
                    pass

        try:
            from navigation.obstacle_avoidance.slam_map import SlamMapSubscriber
        except Exception:
            return None

        sub = SlamMapSubscriber(self.lidar_map_topic)
        sub.start()
        t0 = time.time()
        while time.time() - t0 < max(0.2, float(timeout_s)):
            occ, _meta = sub.to_occupancy(height_threshold=0.15, max_height=None, origin_centered=True)
            if occ is not None:
                try:
                    if near_center is not None and self.lidar_ignore_near_m > 0.0:
                        rr = max(1, int(float(self.lidar_ignore_near_m) / float(occ.resolution)))
                        r0, c0 = occ.world_to_grid(float(near_center[0]), float(near_center[1]))
                        import numpy as np
                        y, x = np.ogrid[-rr : rr + 1, -rr : rr + 1]
                        mask = (x * x + y * y) <= (rr * rr)
                        r_lo = max(0, r0 - rr)
                        r_hi = min(occ.height_cells - 1, r0 + rr)
                        c_lo = max(0, c0 - rr)
                        c_hi = min(occ.width_cells - 1, c0 + rr)
                        subm = occ.grid[r_lo : r_hi + 1, c_lo : c_hi + 1]
                        mr0 = rr - (r0 - r_lo)
                        mc0 = rr - (c0 - c_lo)
                        subm[mask[mr0 : mr0 + subm.shape[0], mc0 : mc0 + subm.shape[1]]] = 0
                    with tempfile.NamedTemporaryFile(prefix="g1_live_nav_", suffix=".npz", delete=False) as tf:
                        out = tf.name
                    occ.save(out)
                    return out
                except Exception:
                    return None
            time.sleep(0.05)
        return None

    def _run_dynamic_nav(self, x: float, y: float, use_rgbd_depth_guard: bool = False) -> int:
        nav_script = SCRIPTS_ROOT / "navigation" / "obstacle_avoidance" / "real_time_path_steps_dynamic.py"

        map_for_run: str | None = self._snapshot_live_slam_map_npz(timeout_s=3.0)
        if map_for_run is None:
            print("[_run_dynamic_nav] live SLAM snapshot unavailable; using fallback map source.")
            map_for_run = self.nav_map

        if not map_for_run:
            candidates: list[Path] = []
            if self._slam_save_dir:
                save_dir = Path(self._slam_save_dir)
                candidates.extend(
                    sorted(save_dir.glob("*latest*.pcd"), key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)
                )
                candidates.extend(
                    sorted(save_dir.glob("*.pcd"), key=lambda p: p.stat().st_mtime if p.exists() else 0.0, reverse=True)
                )
            # common legacy fallback map path
            candidates.append(SCRIPTS_ROOT / "navigation" / "obstacle_avoidance" / "maps" / "live_slam_latest.pcd")
            for c in candidates:
                if c.exists():
                    map_for_run = str(c)
                    break
        if not map_for_run:
            print("[_run_dynamic_nav] no map path resolved for dynamic navigation.")
            return 97

        nav_map_path = Path(map_for_run)
        if not nav_map_path.exists():
            # map file can appear shortly after SLAM starts; wait briefly.
            t0 = time.time()
            while time.time() - t0 < 3.0 and not nav_map_path.exists():
                time.sleep(0.1)
            if not nav_map_path.exists():
                print(f"[_run_dynamic_nav] map file does not exist: {nav_map_path}")
                return 97

        slam_pose = self.get_slam_pose(timeout_s=0.25)
        start = slam_pose if slam_pose is not None else self.get_position()
        start_xy = (float(start[0]), float(start[1])) if start is not None else None
        map_for_run = self._filter_nav_map_near_points(str(map_for_run), start_xy)

        cmd = [
            sys.executable,
            str(nav_script),
            "--map",
            str(map_for_run),
            "--iface",
            self.iface,
            "--domain-id",
            str(self.domain_id),
            "--goal-x",
            str(float(x)),
            "--goal-y",
            str(float(y)),
        ]
        if start is not None:
            cmd.extend(["--start-x", str(float(start[0])), "--start-y", str(float(start[1]))])
        extra_args = shlex.split(self.nav_extra_args) if self.nav_extra_args else []
        sanitized: list[str] = []
        i = 0
        while i < len(extra_args):
            tok = extra_args[i]
            if tok in ("--use-live-map", "--allow-diagonal"):
                i += 1
                continue
            if tok == "--inflation" and i + 1 < len(extra_args):
                sanitized.extend([tok, extra_args[i + 1]])
                i += 2
                continue
            sanitized.append(tok)
            i += 1
        cmd.extend(sanitized)
        if "--no-live-map" not in cmd:
            cmd.append("--no-live-map")
        if "--no-viz" not in cmd:
            cmd.append("--no-viz")
        if "--inflation" not in cmd:
            cmd.extend(["--inflation", "1"])
        # Preview path into SLAM overlay (same map window).
        goal_xy = (float(x), float(y))
        preview = None
        if start_xy is not None:
            preview = self._preview_path_from_map(
                str(map_for_run),
                start_xy=start_xy,
                goal_xy=goal_xy,
                inflation=1,
                allow_diagonal=False,
                smooth=True,
            )
        self._write_nav_overlay_plan(start_xy=start_xy, goal_xy=goal_xy, path_xy=preview, source="dynamic_nav")

        guard: _RgbdDepthGuard | None = None
        if use_rgbd_depth_guard:
            guard = _RgbdDepthGuard(
                depth_port=self.depth_port,
                width=self.rgb_width,
                height=self.rgb_height,
                fps=self.rgb_fps,
                near_distance_m=self.rgbd_obs_near_m,
                min_coverage=self.rgbd_obs_min_coverage,
            )
            guard.start()
            # Give guard a brief moment to initialise pipeline.
            time.sleep(0.15)
            if not guard.available and guard.error:
                print(f"[_run_dynamic_nav] RGBD depth guard disabled: {guard.error}")
                guard = None

        def _exec_nav(run_cmd: list[str]) -> tuple[int, bool]:
            proc = subprocess.Popen(run_cmd, cwd=str(nav_script.parent), start_new_session=True)
            blocked_abort = False
            while proc.poll() is None:
                if guard is not None and guard.is_blocked():
                    blocked_abort = True
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    except Exception:
                        try:
                            proc.terminate()
                        except Exception:
                            pass
                    break
                time.sleep(0.05)
            return int(proc.wait()), blocked_abort

        print(f"[_run_dynamic_nav] start map={map_for_run} goal=({float(x):+.3f},{float(y):+.3f})")
        try:
            rc, blocked_abort = _exec_nav(cmd)
            if blocked_abort:
                print("[_run_dynamic_nav] aborted due to RGBD depth obstacle in front ROI.")
                return 98
            if rc != 0 and "--inflation" in cmd:
                retry = list(cmd)
                idx = retry.index("--inflation")
                if idx + 1 < len(retry) and retry[idx + 1] != "0":
                    retry[idx + 1] = "0"
                    print("[_run_dynamic_nav] retrying with --inflation 0 due to planning failure.")
                    rc2, blocked_abort2 = _exec_nav(retry)
                    if blocked_abort2:
                        print("[_run_dynamic_nav] aborted due to RGBD depth obstacle in front ROI.")
                        return 98
                    rc = rc2
            return rc
        finally:
            if guard is not None:
                guard.stop()

    def _wait_pose_nav_arrival(self, timeout_s: float = 90.0) -> bool:
        from navigation.obstacle_avoidance.slam_map import SlamInfoSubscriber

        sub = SlamInfoSubscriber(self.slam_info_topic, self.slam_key_topic)
        sub.start()
        t0 = time.time()
        while time.time() - t0 < max(0.1, float(timeout_s)):
            for payload in (sub.get_key(), sub.get_info()):
                if not payload:
                    continue
                try:
                    msg = json.loads(payload)
                except Exception:
                    continue
                data = msg.get("data") if isinstance(msg, dict) else None
                if not isinstance(data, dict):
                    continue
                if data.get("is_arrived") is True:
                    return True
                if data.get("is_abort") is True or data.get("is_failed") is True:
                    return False
            time.sleep(0.05)
        return False

    def _run_pose_nav(self, x: float, y: float, yaw: float = 0.0, wait_timeout_s: float = 90.0) -> int:
        from navigation.obstacle_avoidance.slam_service import SlamOperateClient

        client = SlamOperateClient()
        client.Init()
        client.SetTimeout(10.0)

        qz = math.sin(float(yaw) * 0.5)
        qw = math.cos(float(yaw) * 0.5)
        slam_pose = self.get_slam_pose(timeout_s=0.25)
        if slam_pose is not None:
            self._write_nav_overlay_plan(
                start_xy=(float(slam_pose[0]), float(slam_pose[1])),
                goal_xy=(float(x), float(y)),
                path_xy=None,
                source="pose_nav",
            )
        resp = client.pose_nav(float(x), float(y), 0.0, 0.0, 0.0, qz, qw, mode=1)
        if int(resp.code) != 0:
            return int(resp.code)
        return 0 if self._wait_pose_nav_arrival(timeout_s=wait_timeout_s) else -1

    def navigate_path(self, obs_avoid: bool = True, clear_on_finish: bool = True) -> bool:
        if not self._path_points:
            raise RuntimeError("No path points queued. Call set_path_point(...) first.")
        try:
            self.set_gait_type(0)
        except Exception as exc:
            print(f"[navigate_path] warning: failed to set gait_type=0 ({exc})")

        ok = True
        try:
            for idx, (x, y, yaw) in enumerate(self._path_points, start=1):
                pos = self.get_position()
                slam_pos = self.get_slam_pose(timeout_s=0.20)
                if pos is not None:
                    dxy = math.hypot(float(x) - float(pos[0]), float(y) - float(pos[1]))
                    # pose_nav often rejects already-reached goals with rc=4.
                    if dxy <= 0.20:
                        continue
                if obs_avoid:
                    rc = self._run_dynamic_nav(x, y, use_rgbd_depth_guard=True)
                else:
                    # Robot-side planning/execution through SLAM pose_nav.
                    rc = self._run_pose_nav(x, y, yaw)
                    ref = slam_pos if slam_pos is not None else pos
                    if rc == 4 and ref is not None:
                        dxy = math.hypot(float(x) - float(ref[0]), float(y) - float(ref[1]))
                        if dxy <= 0.30:
                            rc = 0
                        else:
                            print(
                                "[navigate_path] pose_nav rc=4 likely frame/relocalization mismatch; "
                                f"slam_pose={slam_pos} odom_pose={pos} goal=({x:.3f},{y:.3f})"
                            )
                    if rc != 0:
                        print(f"[navigate_path] pose_nav rejected (rc={rc}); trying local dynamic nav fallback.")
                        rc = self._run_dynamic_nav(x, y, use_rgbd_depth_guard=True)
                if rc != 0:
                    print(f"[navigate_path] failed at point {idx}: ({x:.3f},{y:.3f},{yaw:.3f}) rc={rc}")
                    ok = False
                    break
        finally:
            if clear_on_finish:
                self._path_points.clear()
            self._clear_nav_overlay_plan()
        return ok

    # ------------------------------------------------------------------
    # Unified SLAM service GUI
    # ------------------------------------------------------------------

    def _keyboard_controller_path(self) -> Path:
        candidates = [
            SCRIPTS_ROOT / "basic" / "safety" / "keyboard_controller.py",
            DEV_OTHER_SAFETY_DIR / "keyboard_controller.py",
        ]
        for p in candidates:
            if p.exists():
                return p
        raise FileNotFoundError("keyboard_controller.py not found in basic/safety or dev/other/safety")

    def _usb_controller_path(self) -> Path:
        candidates = [
            DEV_OTHER_SAFETY_DIR / "usb_controller.py",
            SCRIPTS_ROOT / "basic" / "safety" / "usb_controller.py",
        ]
        for p in candidates:
            if p.exists():
                return p
        raise FileNotFoundError("usb_controller.py not found in dev/other/safety or basic/safety")

    def _launch_teleop(self, input_mode: str = "curses") -> subprocess.Popen:
        script = self._keyboard_controller_path()
        base_cmd = [sys.executable, str(script), "--iface", self.iface, "--input", input_mode]

        if input_mode == "curses":
            term_launchers = [
                ["x-terminal-emulator", "-e"] if shutil.which("x-terminal-emulator") else None,
                ["gnome-terminal", "--"] if shutil.which("gnome-terminal") else None,
                ["konsole", "-e"] if shutil.which("konsole") else None,
                ["xterm", "-e"] if shutil.which("xterm") else None,
            ]
            for prefix in term_launchers:
                if not prefix:
                    continue
                try:
                    return subprocess.Popen(prefix + base_cmd, start_new_session=True)
                except Exception:
                    continue

        return subprocess.Popen(base_cmd, start_new_session=True)

    def enable_usb_controller(self, open_terminal: bool = True, joy: int = 0) -> subprocess.Popen:
        """
        Launch USB gamepad controller.
        While enabled, set the robot headlight to red.
        """
        if self._usb_proc is not None and self._usb_proc.poll() is None:
            return self._usb_proc

        try:
            self.headlight({"color": "red", "intensity": 100})
        except Exception:
            pass

        script = self._usb_controller_path()
        base_cmd = [sys.executable, str(script), "--iface", self.iface, "--joy", str(int(joy))]

        if open_terminal:
            term_launchers = [
                ["x-terminal-emulator", "-e"] if shutil.which("x-terminal-emulator") else None,
                ["gnome-terminal", "--"] if shutil.which("gnome-terminal") else None,
                ["konsole", "-e"] if shutil.which("konsole") else None,
                ["xterm", "-e"] if shutil.which("xterm") else None,
            ]
            for prefix in term_launchers:
                if not prefix:
                    continue
                try:
                    self._usb_proc = subprocess.Popen(prefix + base_cmd, start_new_session=True)
                    return self._usb_proc
                except Exception:
                    continue

        self._usb_proc = subprocess.Popen(base_cmd, start_new_session=True)
        return self._usb_proc

    def slam_service(
        self,
        save_folder: str = "./maps",
        save_every: int = 1,
        save_latest: bool = True,
        save_prefix: str = "live_slam_latest",
        viz: bool = False,
    ) -> int:
        try:
            from PyQt5 import QtCore, QtWidgets
        except Exception:
            try:
                from PySide6 import QtCore, QtWidgets  # type: ignore
            except Exception as exc:
                raise RuntimeError(
                    "PyQt is required for slam_service(). Install PyQt5 or PySide6."
                ) from exc

        robot = self

        class _SlamServiceWindow(QtWidgets.QWidget):  # type: ignore[misc]
            def __init__(self) -> None:
                super().__init__()
                self.setWindowTitle("SLAM Service")
                self.resize(620, 520)

                self._teleop_proc: subprocess.Popen | None = None
                self._follow_thread: threading.Thread | None = None
                self._follow_result: tuple[bool, str] | None = None

                self._status = QtWidgets.QLabel("idle")
                self._path_list = QtWidgets.QListWidget()
                self._obs_avoid = QtWidgets.QCheckBox("Obstacle Avoid Navigation")
                self._obs_avoid.setChecked(True)
                self._clear_on_finish = QtWidgets.QCheckBox("Clear Path On Finish")
                self._clear_on_finish.setChecked(True)
                self._lidar_ignore_spin = QtWidgets.QDoubleSpinBox()
                self._lidar_ignore_spin.setRange(0.0, 3.0)
                self._lidar_ignore_spin.setDecimals(2)
                self._lidar_ignore_spin.setSingleStep(0.05)
                self._lidar_ignore_spin.setSuffix(" m")
                self._lidar_ignore_spin.setValue(float(getattr(robot, "lidar_ignore_near_m", 0.0)))
                self._lidar_apply_btn = QtWidgets.QPushButton("Apply Lidar Ignore")

                self._start_btn = QtWidgets.QPushButton("Start SLAM")
                self._stop_btn = QtWidgets.QPushButton("Stop SLAM")
                self._teleop_start_btn = QtWidgets.QPushButton("Start Teleop (curses)")
                self._teleop_stop_btn = QtWidgets.QPushButton("Stop Teleop")
                self._add_pose_btn = QtWidgets.QPushButton("Add Current Pose")
                self._clear_btn = QtWidgets.QPushButton("Clear Path")
                self._follow_btn = QtWidgets.QPushButton("Follow Path")

                btn_row1 = QtWidgets.QHBoxLayout()
                btn_row1.addWidget(self._start_btn)
                btn_row1.addWidget(self._stop_btn)
                btn_row2 = QtWidgets.QHBoxLayout()
                btn_row2.addWidget(self._teleop_start_btn)
                btn_row2.addWidget(self._teleop_stop_btn)
                btn_row_cfg = QtWidgets.QHBoxLayout()
                btn_row_cfg.addWidget(QtWidgets.QLabel("Lidar Ignore Near"))
                btn_row_cfg.addWidget(self._lidar_ignore_spin)
                btn_row_cfg.addWidget(self._lidar_apply_btn)
                btn_row3 = QtWidgets.QHBoxLayout()
                btn_row3.addWidget(self._add_pose_btn)
                btn_row3.addWidget(self._clear_btn)
                btn_row3.addWidget(self._follow_btn)

                layout = QtWidgets.QVBoxLayout()
                layout.addWidget(self._status)
                layout.addLayout(btn_row1)
                layout.addLayout(btn_row2)
                layout.addLayout(btn_row_cfg)
                layout.addWidget(self._obs_avoid)
                layout.addWidget(self._clear_on_finish)
                layout.addLayout(btn_row3)
                layout.addWidget(QtWidgets.QLabel("Queued Path Points (x, y, yaw):"))
                layout.addWidget(self._path_list)
                self.setLayout(layout)

                self._start_btn.clicked.connect(self._start_slam)
                self._stop_btn.clicked.connect(self._stop_slam)
                self._teleop_start_btn.clicked.connect(self._start_teleop)
                self._teleop_stop_btn.clicked.connect(self._stop_teleop)
                self._lidar_apply_btn.clicked.connect(self._apply_lidar_ignore_near)
                self._add_pose_btn.clicked.connect(self._add_pose)
                self._clear_btn.clicked.connect(self._clear_path)
                self._follow_btn.clicked.connect(self._follow_path)

                self._timer = QtCore.QTimer(self)
                self._timer.setInterval(400)
                self._timer.timeout.connect(self._refresh)
                self._timer.start()
                self._refresh()

            def _refresh(self) -> None:
                follow_alive = self._follow_thread is not None and self._follow_thread.is_alive()
                self._follow_btn.setEnabled(not follow_alive)

                pos = robot.get_position()
                yaw = robot.get_yaw()
                pose_txt = "pose unavailable"
                if pos is not None:
                    pose_txt = f"x={pos[0]:.3f} y={pos[1]:.3f} yaw={(yaw if yaw is not None else 0.0):.3f}"
                self._status.setText(
                    "slam_running="
                    f"{robot.slam_is_running} | queued={len(robot.get_path_points())} | "
                    f"lidar_ignore_near={float(getattr(robot, 'lidar_ignore_near_m', 0.0)):.2f}m | {pose_txt}"
                )

                self._path_list.clear()
                for x, y, yyaw in robot.get_path_points():
                    self._path_list.addItem(f"{x:.3f}, {y:.3f}, {yyaw:.3f}")

                if not follow_alive and self._follow_result is not None:
                    ok, err = self._follow_result
                    self._follow_result = None
                    if err:
                        QtWidgets.QMessageBox.critical(self, "Follow Path Failed", err)
                    elif not ok:
                        QtWidgets.QMessageBox.warning(self, "Follow Path", "Path execution did not complete.")

            def _start_slam(self) -> None:
                try:
                    robot.start_slam(
                        save_folder=save_folder,
                        save_every=save_every,
                        save_latest=save_latest,
                        save_prefix=save_prefix,
                        viz=viz,
                    )
                except Exception as exc:
                    QtWidgets.QMessageBox.critical(self, "Start SLAM Failed", str(exc))
                self._refresh()

            def _stop_slam(self) -> None:
                try:
                    robot.stop_slam(save_folder=save_folder)
                except Exception as exc:
                    QtWidgets.QMessageBox.critical(self, "Stop SLAM Failed", str(exc))
                self._refresh()

            def _start_teleop(self) -> None:
                if self._teleop_proc is not None and self._teleop_proc.poll() is None:
                    return
                try:
                    self._teleop_proc = robot._launch_teleop(input_mode="curses")
                except Exception as exc:
                    QtWidgets.QMessageBox.critical(self, "Teleop Failed", str(exc))

            def _stop_teleop(self) -> None:
                if self._teleop_proc is None or self._teleop_proc.poll() is not None:
                    return
                try:
                    os.killpg(os.getpgid(self._teleop_proc.pid), signal.SIGINT)
                except Exception:
                    try:
                        self._teleop_proc.terminate()
                    except Exception:
                        pass

            def _apply_lidar_ignore_near(self) -> None:
                try:
                    val = float(self._lidar_ignore_spin.value())
                    robot.set_lidar_ignore_near(val)
                except Exception as exc:
                    QtWidgets.QMessageBox.warning(self, "Lidar Config Failed", str(exc))
                self._refresh()

            def _add_pose(self) -> None:
                slam_pose = robot.get_slam_pose(timeout_s=0.35) if robot.slam_is_running else None
                if slam_pose is not None:
                    pos = (slam_pose[0], slam_pose[1], 0.0)
                    yaw = slam_pose[2]
                else:
                    pos = robot.get_position()
                    yaw = robot.get_yaw()
                if pos is None:
                    QtWidgets.QMessageBox.warning(self, "Pose Unavailable", "No current pose from sensors.")
                    return
                try:
                    robot.set_path_point(float(pos[0]), float(pos[1]), float(yaw if yaw is not None else 0.0))
                except Exception as exc:
                    QtWidgets.QMessageBox.warning(self, "Add Pose Failed", str(exc))
                self._refresh()

            def _clear_path(self) -> None:
                robot.clear_path_points()
                self._refresh()

            def _follow_path(self) -> None:
                if self._follow_thread is not None and self._follow_thread.is_alive():
                    return

                def _run() -> None:
                    ok = False
                    err = ""
                    try:
                        ok = robot.navigate_path(
                            obs_avoid=bool(self._obs_avoid.isChecked()),
                            clear_on_finish=bool(self._clear_on_finish.isChecked()),
                        )
                    except Exception as exc:
                        err = str(exc)
                    self._follow_result = (ok, err)

                self._follow_thread = threading.Thread(target=_run, daemon=True)
                self._follow_thread.start()

            def closeEvent(self, event: Any) -> None:  # noqa: N802
                try:
                    self._timer.stop()
                except Exception:
                    pass
                super().closeEvent(event)

        app = QtWidgets.QApplication.instance()
        own_app = app is None
        if app is None:
            app = QtWidgets.QApplication(sys.argv)

        window = _SlamServiceWindow()
        self._slam_service_windows.append(window)
        window.destroyed.connect(lambda *_: self._slam_service_windows.remove(window) if window in self._slam_service_windows else None)
        window.show()

        if own_app:
            return int(app.exec())
        return 0

    # ------------------------------------------------------------------
    # SLAM debug API (mirrors api_util.py sequence)
    # ------------------------------------------------------------------

    def debug_api(
        self,
        save_path: str = "/home/unitree/test1.pcd",
        load_path: str = "/home/unitree/test1.pcd",
        goal_x: float = 1.0,
        goal_y: float = 0.0,
        goal_yaw: float = 0.0,
        pause: bool = False,
        resume: bool = False,
        wait_task_result: bool = False,
    ) -> None:
        from navigation.obstacle_avoidance.slam_map import SlamInfoSubscriber
        from navigation.obstacle_avoidance.slam_service import SlamOperateClient, SlamResponse

        def _print_resp(label: str, req: dict[str, Any], resp: SlamResponse) -> None:
            print(f"\\n[{label}]")
            print("request:", json.dumps(req, indent=2))
            print(f"response: code={resp.code} raw={resp.raw}")

        def _wait_task(sub: SlamInfoSubscriber, timeout: float = 10.0) -> None:
            t0 = time.time()
            while time.time() - t0 < timeout:
                key = sub.get_key()
                if key:
                    try:
                        payload = json.loads(key)
                        if payload.get("type") == "task_result":
                            print("task_result:", json.dumps(payload, indent=2))
                            return
                    except Exception:
                        pass
                time.sleep(0.05)
            print("task_result: timeout")

        info_sub = SlamInfoSubscriber(self.slam_info_topic, self.slam_key_topic)
        info_sub.start()

        client = SlamOperateClient()
        client.Init()
        client.SetTimeout(10.0)

        req = {"data": {"slam_type": "indoor"}}
        _print_resp("start_mapping (1801)", req, client.start_mapping("indoor"))

        req = {"data": {"address": save_path}}
        _print_resp("end_mapping (1802)", req, client.end_mapping(save_path))

        req = {
            "data": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "q_x": 0.0,
                "q_y": 0.0,
                "q_z": 0.0,
                "q_w": 1.0,
                "address": load_path,
            }
        }
        _print_resp("init_pose (1804)", req, client.init_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, load_path))

        qz = math.sin(float(goal_yaw) * 0.5)
        qw = math.cos(float(goal_yaw) * 0.5)
        req = {
            "data": {
                "targetPose": {
                    "x": float(goal_x),
                    "y": float(goal_y),
                    "z": 0.0,
                    "q_x": 0.0,
                    "q_y": 0.0,
                    "q_z": qz,
                    "q_w": qw,
                },
                "mode": 1,
            }
        }
        _print_resp(
            "pose_nav (1102)",
            req,
            client.pose_nav(float(goal_x), float(goal_y), 0.0, 0.0, 0.0, qz, qw, mode=1),
        )

        if pause:
            _print_resp("pause_nav (1201)", {"data": {}}, client.pause_nav())
        if resume:
            _print_resp("resume_nav (1202)", {"data": {}}, client.resume_nav())
        if wait_task_result:
            _wait_task(info_sub)

        _print_resp("close_slam (1901)", {"data": {}}, client.close_slam())

    # ------------------------------------------------------------------
    # Safety / audio / lights
    # ------------------------------------------------------------------

    def balanced_stand(self, mode: int = 0) -> None:
        """Command balanced stand (default mode=0)."""
        if hasattr(self._client, "BalanceStand"):
            self._client.BalanceStand(int(mode))
        else:
            self._ensure_balanced_gait_mode()

    def hanging_boot(self) -> None:
        """Backward-compatible alias; use balanced_stand()."""
        self.balanced_stand(0)

    def hanged_boot(self) -> None:
        """Backward-compatible alias; use balanced_stand()."""
        self.balanced_stand(0)

    @staticmethod
    def _normalize_arm_joint_name(name: str) -> str:
        key = str(name).strip().lower().replace("-", " ").replace("_", " ")
        key = re.sub(r"\s+", " ", key)
        key = key.replace(" ", "_")
        return ARM_JOINT_ALIASES.get(key, key)

    @staticmethod
    def _normalize_leg_joint_name(name: str) -> str:
        key = str(name).strip().lower().replace("-", " ").replace("_", " ")
        key = re.sub(r"\s+", " ", key)
        key = key.replace(" ", "_")
        return LEG_JOINT_ALIASES.get(key, key)

    def _rotate_lowcmd_joint(
        self,
        joint_idx: int,
        delta_deg: float,
        duration: float,
        hold: float,
        cmd_hz: float,
        kp: float,
        kd: float,
        easing: str,
    ) -> int:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

        joint_idx = int(joint_idx)
        if joint_idx < 0 or joint_idx > 28:
            raise ValueError(f"Invalid joint index: {joint_idx}")

        ChannelFactoryInitialize(self.domain_id, self.iface)
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        try:
            _status, result = msc.CheckMode()
            tries = 0
            while isinstance(result, dict) and result.get("name") and tries < 3:
                msc.ReleaseMode()
                time.sleep(0.5)
                _status, result = msc.CheckMode()
                tries += 1
        except Exception:
            pass

        state_box: dict[str, Any] = {"msg": None}
        state_ready = threading.Event()

        def _ls_cb(msg: Any) -> None:
            state_box["msg"] = msg
            state_ready.set()

        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(_ls_cb, 10)
        if not state_ready.wait(timeout=1.0):
            return 2
        ls = state_box.get("msg")
        if ls is None:
            return 2

        q_cur: list[float] = []
        for i in range(29):
            try:
                q_cur.append(float(ls.motor_state[i].q))
            except Exception:
                q_cur.append(0.0)

        q0 = float(q_cur[joint_idx])
        q1 = float(q0 + math.radians(float(delta_deg)))

        pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        pub.Init()
        crc = CRC()
        cmd = unitree_hg_msg_dds__LowCmd_()

        hz = max(1.0, float(cmd_hz))
        dt = 1.0 / hz
        steps = max(1, int(hz * max(0.05, float(duration))))

        for step in range(1, steps + 1):
            alpha = step / steps
            if str(easing).lower() == "smooth":
                alpha = 0.5 - 0.5 * math.cos(math.pi * alpha)
            q_tar = q0 + (q1 - q0) * alpha
            for i in range(29):
                mc = cmd.motor_cmd[i]
                mc.q = float(q_cur[i])
                mc.dq = 0.0
                mc.kp = 0.0
                mc.kd = 0.0
                mc.tau = 0.0
            tgt = cmd.motor_cmd[joint_idx]
            tgt.q = float(q_tar)
            tgt.dq = 0.0
            tgt.kp = float(kp)
            tgt.kd = float(kd)
            tgt.tau = 0.0
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            time.sleep(dt)

        hold_steps = max(0, int(hz * max(0.0, float(hold))))
        for _ in range(hold_steps):
            for i in range(29):
                mc = cmd.motor_cmd[i]
                mc.q = float(q_cur[i])
                mc.dq = 0.0
                mc.kp = 0.0
                mc.kd = 0.0
                mc.tau = 0.0
            tgt = cmd.motor_cmd[joint_idx]
            tgt.q = float(q1)
            tgt.dq = 0.0
            tgt.kp = float(kp)
            tgt.kd = float(kd)
            tgt.tau = 0.0
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            time.sleep(dt)
        return 0

    def rotate_joint(
        self,
        joint_name: str,
        angle_deg: float,
        arm: str | None = None,
        duration: float = 1.0,
        hold: float = 0.0,
        cmd_hz: float = 50.0,
        kp: float = 40.0,
        kd: float = 1.0,
        easing: str = "smooth",
    ) -> int:
        """
        Rotate a single arm joint using the same arm_sdk path as arm_motion.py.

        Examples:
            rotate_joint("elbow", 30)                  # defaults to right arm
            rotate_joint("left shoulder_pitch", -20)   # arm inferred from name
            rotate_joint("wrist_roll", 15, arm="left")
        """
        raw = str(joint_name).strip()
        if not raw:
            raise ValueError("joint_name cannot be empty")

        inferred_arm = None
        inferred_side = None
        lower = raw.lower()
        if lower.startswith("left "):
            inferred_arm = "left"
            inferred_side = "left"
            raw = raw[5:].strip()
        elif lower.startswith("right "):
            inferred_arm = "right"
            inferred_side = "right"
            raw = raw[6:].strip()

        # Try arm-style joint name first.
        arm_name = str(arm or inferred_arm or "right").strip().lower()
        if arm_name not in ("left", "right"):
            arm_name = "right"

        joint = self._normalize_arm_joint_name(raw)
        valid = {
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_pitch",
            "wrist_roll",
            "wrist_yaw",
            "waist_yaw",
        }
        if joint not in valid:
            # Fall back to leg-joint route.
            side = str(arm or inferred_side or "left").strip().lower()
            if side not in ("left", "right"):
                raise ValueError("For leg joints, provide side with arm='left'/'right' or prefix joint with left/right.")
            leg_joint = self._normalize_leg_joint_name(raw)
            leg_key = f"{side}_{leg_joint}"
            if leg_key not in G1_LEG_JOINT_INDEX:
                raise ValueError(f"Unknown joint_name '{joint_name}'")
            return int(
                self._rotate_lowcmd_joint(
                    joint_idx=G1_LEG_JOINT_INDEX[leg_key],
                    delta_deg=float(angle_deg),
                    duration=max(0.05, float(duration)),
                    hold=max(0.0, float(hold)),
                    cmd_hz=max(1.0, float(cmd_hz)),
                    kp=float(kp),
                    kd=float(kd),
                    easing=str(easing),
                )
            )

        steps = {
            "arm": arm_name,
            "cmd_hz": float(cmd_hz),
            "kp": float(kp),
            "kd": float(kd),
            "steps": [
                {
                    "name": f"rotate_{joint}",
                    "duration": max(0.05, float(duration)),
                    "hold": max(0.0, float(hold)),
                    "angles": {joint: float(angle_deg)},
                }
            ],
        }

        script = SCRIPTS_ROOT / "arm_motion" / "arm_motion.py"
        if not script.exists():
            raise FileNotFoundError(f"arm motion script not found: {script}")

        with tempfile.NamedTemporaryFile(prefix="g1_rotate_joint_", suffix=".json", delete=False) as tf:
            tf.write(json.dumps(steps, ensure_ascii=True).encode("utf-8"))
            steps_path = tf.name
        try:
            cmd = [
                sys.executable,
                str(script),
                "--iface",
                self.iface,
                "--steps",
                str(steps_path),
                "--arm",
                arm_name,
                "--easing",
                str(easing),
            ]
            rc = subprocess.run(cmd, cwd=str(script.parent), check=False).returncode
            return int(rc)
        finally:
            try:
                os.unlink(steps_path)
            except Exception:
                pass

    def say(self, text: str = "what would you like me to say?") -> None:
        audio_dir = SCRIPTS_ROOT / "basic" / "audio"
        tts_script = audio_dir / "text_to_wav.py"
        greet_script = audio_dir / "greeting.py"

        with tempfile.TemporaryDirectory(prefix="g1_say_") as td:
            wav = Path(td) / "speech.wav"
            subprocess.run([sys.executable, str(tts_script), text, "-o", str(wav)], check=True)
            subprocess.run(
                [sys.executable, str(greet_script), "--robot", "--iface", self.iface, "--file", str(wav)],
                check=True,
            )

    def headlight(
        self,
        args: dict[str, Any] | list[str] | str | None = None,
        duration: float | None = None,
    ) -> int:
        """
        Wrapper over basic/headlight_client/headlight.py.

        Examples:
            robot.headlight({"color": "yellow", "intensity": 70})
            robot.headlight("--color red --intensity 40")
            robot.headlight(["--color", "green", "--intensity", "90"])
            robot.headlight({"color": "white", "intensity": 80}, duration=2.5)
        """
        script = SCRIPTS_ROOT / "basic" / "headlight_client" / "headlight.py"
        cmd = [sys.executable, str(script), "--iface", self.iface]

        if isinstance(args, dict):
            for k, v in args.items():
                key = f"--{str(k).replace('_', '-')}"
                if isinstance(v, bool):
                    if v:
                        cmd.append(key)
                elif v is not None:
                    cmd.extend([key, str(v)])
        elif isinstance(args, str) and args.strip():
            cmd.extend(shlex.split(args))
        elif isinstance(args, list):
            cmd.extend([str(x) for x in args])

        if duration is not None:
            cmd.extend(["--duration", str(float(duration))])

        result = subprocess.run(cmd, check=False)
        return int(result.returncode)

    def huddle(self, args: dict[str, Any] | list[str] | str | None = None) -> int:
        """
        Wrapper over dev/other/huddle/huddle.py.

        Examples:
            robot.huddle()
            robot.huddle({"arm": "right", "volume": 80, "brightness": 40})
            robot.huddle("--arm left --file huddle.wav --volume 90")
        """
        script = SCRIPTS_ROOT / "dev" / "other" / "huddle" / "huddle.py"
        cmd = [sys.executable, str(script), "--iface", self.iface]

        if isinstance(args, dict):
            for k, v in args.items():
                key = f"--{str(k).replace('_', '-')}"
                if isinstance(v, bool):
                    if v:
                        cmd.append(key)
                elif v is not None:
                    cmd.extend([key, str(v)])
        elif isinstance(args, str) and args.strip():
            cmd.extend(shlex.split(args))
        elif isinstance(args, list):
            cmd.extend([str(x) for x in args])

        result = subprocess.run(cmd, cwd=str(script.parent), check=False)
        return int(result.returncode)


__all__ = ["Robot", "ImuData"]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Smoke test for ef_client Robot wrapper")
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--no-safety", action="store_true")
    args = parser.parse_args()

    bot = Robot(iface=args.iface, safety_boot=not args.no_safety)
    time.sleep(0.6)
    print("FSM:", bot.get_fsm())
    print("IMU:", bot.get_imu())
