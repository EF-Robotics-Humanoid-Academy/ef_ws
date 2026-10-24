"""Semantic RobotStateSnapshot from sdk_wrapper.G1 (replaces agent.state's
SdkClientRobotStateSource, which reads sdk_client.Robot).

Same snapshot contract: posture from the sport-state mode and velocity,
stability/faults from stale watched topics (alias topics tolerated when one
source is fresh), lowstate and battery summaries.  The RPC part of the read
runs with a timeout so a blocked locomotion RPC (e.g. in developer mode)
degrades the snapshot instead of freezing the agent.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Dict, Optional

from agent.models import RobotStateSnapshot
from agent.state import RobotStateSource

from .sensors import SensorWatch

SOURCE = "sdk_wrapper.G1"


class G1RobotStateSource(RobotStateSource):
    def __init__(self, robot: Any, sensors: SensorWatch, *,
                 arm_control_state_hint: Optional[Callable[[], str]] = None) -> None:
        self._robot = robot
        self._sensors = sensors
        self._arm_control_state_hint = arm_control_state_hint
        self._read_lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._read_result: Optional[Dict[str, Any]] = None
        self._read_error: Optional[BaseException] = None

    def read(self) -> RobotStateSnapshot:
        now = time.time()
        try:
            raw = self._read_raw_state_with_timeout(timeout_s=1.0)
        except Exception as exc:
            return RobotStateSnapshot(timestamp=now, posture="unknown", stability="unknown",
                                      active_faults=[f"state_read_error: {exc}"], arm_control_state="unknown",
                                      source=f"{SOURCE} (read failed)")
        lowstate = self._read_lowstate_summary()
        battery = self._read_battery_summary()
        stale = raw.get("sensor_stale") or {}
        active_faults = self._active_faults_from_stale(stale, lowstate=lowstate, raw=raw)
        if lowstate is None and "lowstate" not in active_faults:
            active_faults.append("lowstate")
        posture = "moving" if raw.get("is_moving") else "standing"
        try:
            if raw.get("mode") is not None and int(raw["mode"]) in (0, 1):  # zero torque / damp
                posture = "damped_or_zero_torque"
        except (TypeError, ValueError):
            pass
        arm_control_state = "unknown"
        if self._arm_control_state_hint is not None:
            try:
                arm_control_state = self._arm_control_state_hint()
            except Exception:
                arm_control_state = "unknown"
        return RobotStateSnapshot(
            timestamp=now, posture=posture, battery_pct=battery.get("soc") if battery else None,
            charging=battery.get("charging") if battery else None,
            stability="stable" if not active_faults else "degraded", active_faults=active_faults,
            arm_control_state=arm_control_state, lowstate=lowstate, battery=battery, sensor_stale=stale,
            sensor_timestamps=raw.get("sensor_timestamps") or {},
            stale_sensor_topics=[name for name, is_stale in stale.items() if is_stale], source=SOURCE,
        )

    def _raw_state(self) -> Dict[str, Any]:
        robot = self._robot
        odom = robot.get_odom() or {}
        velocity = odom.get("velocity")
        moving = False
        if velocity is not None:
            vx, vy, vz = velocity
            moving = math.hypot(vx, vy) > 0.03 or abs(vz) > 0.08
        return {
            "fsm": robot.get_fsm(),
            "mode": odom.get("mode"),
            "position": odom.get("position"),
            "velocity": velocity,
            "is_moving": moving,
            "odom_pose": odom.get("pose") or robot.get_slam_odom_pose(),
            "slam_pose": robot.get_slam_pose(),
            "sensor_timestamps": self._sensors.timestamps(),
            "sensor_stale": self._sensors.stale(),
        }

    def _read_raw_state_with_timeout(self, *, timeout_s: float) -> Dict[str, Any]:
        with self._read_lock:
            thread = self._read_thread
            if thread is not None and thread.is_alive():
                raise TimeoutError("previous G1 state read is still pending")
            self._read_result = None
            self._read_error = None

            def worker() -> None:
                try:
                    self._read_result = self._raw_state()
                except BaseException as exc:  # re-raised on the caller thread
                    self._read_error = exc

            thread = threading.Thread(target=worker, name="g1-agent-state-read", daemon=True)
            self._read_thread = thread
            thread.start()
        thread.join(timeout=max(0.05, float(timeout_s)))
        if thread.is_alive():
            raise TimeoutError("G1 state read timed out")
        if self._read_error is not None:
            raise self._read_error
        return dict(self._read_result or {})

    @staticmethod
    def _active_faults_from_stale(stale: Dict[str, Any], *, lowstate: Optional[Dict[str, Any]],
                                  raw: Dict[str, Any]) -> list:
        """Stale watched topics become faults, except aliases of a fresh source."""
        fresh_cloud = any(name.startswith("lidar_cloud") and not is_stale for name, is_stale in stale.items())
        has_pose = any(raw.get(key) is not None for key in ("position", "odom_pose", "slam_pose"))
        faults = []
        for name, is_stale in stale.items():
            if not is_stale:
                continue
            if name.startswith("lidar_cloud") and fresh_cloud:
                continue
            if name in {"lidar_map", "odom", "slam_odom"} and has_pose:
                continue
            if name == "lowstate" and lowstate is not None:
                continue
            if name in {"left_hand_state", "right_hand_state"}:
                continue
            faults.append(name)
        return faults

    def _read_lowstate_summary(self) -> Optional[Dict[str, Any]]:
        try:
            low = self._robot.get_lowstate()
        except Exception:
            return None
        if low is None:
            return None
        imu = low.get("imu") or {}
        return {
            "timestamp": low.get("timestamp"),
            "joint_count": len(low.get("joint_positions") or []),
            "joint_positions": low.get("joint_positions") or [],
            "joint_velocities": low.get("joint_velocities") or [],
            "joint_torques": low.get("joint_torques") or [],
            "imu": {"rpy": imu.get("rpy"), "gyro": imu.get("gyro"), "acc": imu.get("acc")},
            "source": "/lowstate",
        }

    def _read_battery_summary(self) -> Optional[Dict[str, Any]]:
        try:
            battery = self._robot.get_battery()
        except Exception as exc:
            return {"soc": None, "charging": None, "source": "battery", "error": str(exc)}
        if not battery:
            return None
        bms = battery.get("bms") or {}
        current_ma = bms.get("current")
        power_a = battery.get("power_a")
        current_a = power_a if power_a is not None else (None if current_ma is None else current_ma / 1000.0)
        return {
            "soc": bms.get("soc"), "soh": bms.get("soh"), "status": bms.get("status"), "current_ma": current_ma,
            "current_a": current_a, "cycle": bms.get("cycle"), "power_v": battery.get("power_v"), "power_a": power_a,
            "charging": None if current_a is None else current_a > 0.05,
            "source": battery.get("source") or "/lowstate.bms_state",
        }
