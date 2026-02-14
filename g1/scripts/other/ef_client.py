"""
ef_client.py
============

Lightweight robot client wrapper for Unitree G1 troubleshooting.

Provides:
  - safe boot / FSM control via hanger_boot_sequence
  - cached sensor data (IMU, sport state, lidar map, lidar cloud)
  - simple motion helpers

This is intentionally conservative: it does not attempt autonomous motion
or planning; it only exposes basic SDK calls and cached telemetry.
"""
from __future__ import annotations

import importlib
import json
import os
import sys
import threading
import time
from dataclasses import dataclass, is_dataclass, asdict
from typing import Any, Dict, Iterable, Optional, Set, Tuple

# Ensure scripts dir is on sys.path so we can import safety helpers.
_SCRIPTS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_, HeightMap_
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from safety.hanger_boot_sequence import hanger_boot_sequence


TOPIC_SPORT = "rt/sportmodestate"
TOPIC_LIDAR_STATE = "rt/utlidar/map_state"
TOPIC_LIDAR_CLOUD = "rt/utlidar/cloud_deskewed"
TOPIC_LIDAR_SWITCH = "rt/utlidar/switch"
RGBD_TOPIC_KEYWORDS = ("rgbd", "depth", "rgb", "color", "image", "camera")


@dataclass
class ImuData:
    rpy: tuple[float, float, float]
    gyro: tuple[float, float, float] | None
    acc: tuple[float, float, float] | None
    quat: tuple[float, float, float, float] | None
    temp: float | None


class TopicStats:
    def __init__(self) -> None:
        self.sample_count: int = 0
        self.last_sample: str = ""
        self.last_ts: float = 0.0
        self.last_error: str = ""

    def update_sample(self, sample: Any) -> None:
        self.sample_count += 1
        self.last_sample = _truncate(_format_sample(sample), 400)
        self.last_ts = time.time()
        self.last_error = ""

    def update_error(self, exc: Exception) -> None:
        self.last_error = str(exc)
        self.last_ts = time.time()


def _truncate(value: str, limit: int) -> str:
    if len(value) <= limit:
        return value
    return value[: limit - 3] + "..."


def _format_sample(sample: Any) -> str:
    try:
        if is_dataclass(sample):
            return json.dumps(asdict(sample), ensure_ascii=True)
    except Exception:
        pass
    try:
        if hasattr(sample, "to_dict"):
            return json.dumps(sample.to_dict(), ensure_ascii=True)
    except Exception:
        pass
    try:
        return repr(sample)
    except Exception:
        return "<unprintable sample>"


def _type_name_to_idl_module(type_name: str) -> Optional[Tuple[str, str]]:
    if not type_name or "::" not in type_name:
        return None
    parts = type_name.split("::")
    if len(parts) < 2:
        return None
    class_name = parts[-1]
    namespace = parts[:-1]
    module_path = "unitree_sdk2py.idl." + ".".join(namespace) + "._" + class_name
    return module_path, class_name


def _resolve_type(type_path: str) -> Any:
    if ":" in type_path:
        module_path, class_name = type_path.split(":", 1)
    else:
        module_path, class_name = type_path.rsplit(".", 1)
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def _type_path_candidates(type_path: str) -> list[str]:
    candidates: list[str] = []
    if "::" not in type_path:
        return [type_path]
    mapped = _type_name_to_idl_module(type_path)
    if mapped:
        module_path, class_name = mapped
        candidates.append(f"{module_path}:{class_name}")
    if type_path.startswith("sensor_msgs::msg::dds_::"):
        class_name = type_path.split("::")[-1]
        candidates.append(f"unitree_sdk2py.idl.ros2._{class_name}:{class_name}")
    candidates.append(type_path)
    return candidates


class DdsDiscovery:
    def __init__(self, domain_id: int) -> None:
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsTopic, BuiltinTopicDcpsPublication

        self.participant = DomainParticipant(domain_id)
        self.topic_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsTopic)
        self.pub_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsPublication)

        self.seen_topics: Dict[str, str] = {}
        self.seen_publications: Set[Tuple[str, str]] = set()

    def poll(self) -> list[Tuple[str, str]]:
        discovered: list[Tuple[str, str]] = []
        try:
            topics = self.topic_reader.read(64)
        except Exception:
            topics = []

        for t in topics:
            try:
                topic_name = t.topic_name
                type_name = t.type_name
            except Exception:
                continue
            if topic_name not in self.seen_topics:
                self.seen_topics[topic_name] = type_name
                discovered.append((topic_name, type_name))

        try:
            pubs = self.pub_reader.read(64)
        except Exception:
            pubs = []

        for p in pubs:
            try:
                key = (p.topic_name, p.type_name)
            except Exception:
                continue
            if key not in self.seen_publications:
                self.seen_publications.add(key)
        return discovered


class Robot:
    """Minimal, safe wrapper around Unitree SDK2 for troubleshooting."""

    def __init__(self, iface: str, domain_id: int = 0, safety_boot: bool = True) -> None:
        self.iface = iface
        self.domain_id = domain_id
        self._lock = threading.Lock()

        self._sport: SportModeState_ | None = None
        self._lidar_state: Any | None = None
        self._lidar_cloud: PointCloud2_ | None = None
        self._last_sport_ts: float = 0.0
        self._last_lidar_ts: float = 0.0
        self._last_cloud_ts: float = 0.0

        self._sport_sub: ChannelSubscriber | None = None
        self._lidar_state_sub: ChannelSubscriber | None = None
        self._lidar_cloud_sub: ChannelSubscriber | None = None
        self._lidar_switch_pub: ChannelPublisher | None = None
        self._extra_subs: Dict[str, ChannelSubscriber] = {}
        self._topic_stats: Dict[str, TopicStats] = {}
        self._discovery: DdsDiscovery | None = None
        self._discovery_thread: threading.Thread | None = None
        self._discovery_stop = threading.Event()

        # SDK init + safe boot (preferred)
        if safety_boot:
            self._client: LocoClient = hanger_boot_sequence(iface=iface)
        else:
            ChannelFactoryInitialize(domain_id, iface)
            self._client = LocoClient()
            self._client.SetTimeout(10.0)
            self._client.Init()

        # Start subscribers lazily; caller can invoke start_sensors() / start_extra_sensors().

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def start_sensors(self) -> None:
        """Start DDS subscriptions for sport state and lidar topics."""
        if self._sport_sub is None:
            self._sport_sub = ChannelSubscriber(TOPIC_SPORT, SportModeState_)
            self._sport_sub.Init(self._sport_cb, 10)

        if self._lidar_state_sub is None:
            self._lidar_state_sub = ChannelSubscriber(TOPIC_LIDAR_STATE, HeightMap_)
            self._lidar_state_sub.Init(self._lidar_state_cb, 10)

        if self._lidar_cloud_sub is None:
            self._lidar_cloud_sub = ChannelSubscriber(TOPIC_LIDAR_CLOUD, PointCloud2_)
            self._lidar_cloud_sub.Init(self._lidar_cloud_cb, 10)

        if self._lidar_switch_pub is None:
            self._lidar_switch_pub = ChannelPublisher(TOPIC_LIDAR_SWITCH, String_)
            self._lidar_switch_pub.Init()

    def start_extra_sensors(self, topics: Iterable[Tuple[str, str]]) -> None:
        """
        Start additional DDS subscriptions using type discovery helpers.

        topics: iterable of (topic_name, type_path_or_dds_type_name)
        """
        for topic_name, type_path in topics:
            if topic_name in self._extra_subs:
                continue
            msg_type = None
            last_exc: Exception | None = None
            for candidate in _type_path_candidates(type_path):
                try:
                    msg_type = _resolve_type(candidate)
                    break
                except Exception as exc:
                    last_exc = exc
            if msg_type is None:
                if last_exc:
                    raise last_exc
                raise RuntimeError(f"Failed to resolve type for {topic_name}: {type_path}")
            sub = ChannelSubscriber(topic_name, msg_type)
            sub.Init(lambda msg, t=topic_name: self._generic_cb(t, msg), 10)
            self._extra_subs[topic_name] = sub
            self._topic_stats.setdefault(topic_name, TopicStats())

    def start_discovery(self, poll_interval: float = 0.2) -> None:
        """
        Start DDS discovery loop and auto-subscribe to new topics when type mapping works.
        Requires cyclonedds.
        """
        if self._discovery_thread is not None:
            return

        try:
            self._discovery = DdsDiscovery(self.domain_id)
        except Exception as exc:
            raise RuntimeError("DDS discovery unavailable (cyclonedds required)") from exc

        self._discovery_stop.clear()

        def _loop() -> None:
            while not self._discovery_stop.is_set():
                if self._discovery is None:
                    break
                for topic_name, type_name in self._discovery.poll():
                    if topic_name in self._extra_subs:
                        continue
                    if any(key in topic_name.lower() for key in RGBD_TOPIC_KEYWORDS):
                        pass
                    try:
                        self.start_extra_sensors([(topic_name, type_name)])
                    except Exception:
                        continue
                time.sleep(poll_interval)

        self._discovery_thread = threading.Thread(target=_loop, name="g1_dds_discovery", daemon=True)
        self._discovery_thread.start()

    def stop_discovery(self) -> None:
        if self._discovery_thread is None:
            return
        self._discovery_stop.set()
        self._discovery_thread.join(timeout=1.0)
        self._discovery_thread = None

    def _sport_cb(self, msg: SportModeState_) -> None:
        with self._lock:
            self._sport = msg
            self._last_sport_ts = time.time()

    def _lidar_state_cb(self, msg: Any) -> None:
        with self._lock:
            self._lidar_state = msg
            self._last_lidar_ts = time.time()

    def _lidar_cloud_cb(self, msg: PointCloud2_) -> None:
        with self._lock:
            self._lidar_cloud = msg
            self._last_cloud_ts = time.time()

    def _generic_cb(self, topic_name: str, msg: Any) -> None:
        with self._lock:
            stat = self._topic_stats.setdefault(topic_name, TopicStats())
            stat.update_sample(msg)

    # ------------------------------------------------------------------
    # Sensor getters (best-effort)
    # ------------------------------------------------------------------

    def get_sport_state(self) -> SportModeState_ | None:
        with self._lock:
            return self._sport

    def get_imu_data(self) -> ImuData | None:
        with self._lock:
            msg = self._sport
        if msg is None:
            return None

        rpy = (0.0, 0.0, 0.0)
        gyro = acc = quat = None
        temp = None

        try:
            rpy = (float(msg.imu_state.rpy[0]), float(msg.imu_state.rpy[1]), float(msg.imu_state.rpy[2]))
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

    def get_lidar_map(self) -> HeightMap_ | None:
        with self._lock:
            return self._lidar_state

    def get_lidar_cloud(self) -> PointCloud2_ | None:
        with self._lock:
            return self._lidar_cloud

    def lidar_switch(self, on: bool) -> int:
        if self._lidar_switch_pub is None:
            return -1
        self._lidar_switch_pub.Write(String_("ON" if on else "OFF"))
        return 0

    def sensors_stale(self, max_age: float = 1.0) -> bool:
        with self._lock:
            ts = self._last_sport_ts
        if ts == 0.0:
            return True
        return (time.time() - ts) > max_age

    def get_topic_stats(self, topic_name: str) -> TopicStats | None:
        with self._lock:
            return self._topic_stats.get(topic_name)

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------

    def move(self, vx: float, vy: float, vyaw: float, continuous: bool = True) -> int:
        return self._client.Move(float(vx), float(vy), float(vyaw), continous_move=continuous)

    def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        if hasattr(self._client, "SetVelocity"):
            self._client.SetVelocity(float(vx), float(vy), float(vyaw))
        else:
            self.move(vx, vy, vyaw, continuous=True)

    def stop(self) -> None:
        if hasattr(self._client, "StopMove"):
            self._client.StopMove()
        else:
            self.move(0.0, 0.0, 0.0, continuous=False)

    def emergency_stop(self) -> None:
        try:
            self.stop()
        finally:
            if hasattr(self._client, "ZeroTorque"):
                self._client.ZeroTorque()

    def damp(self) -> None:
        if hasattr(self._client, "Damp"):
            self._client.Damp()

    # ------------------------------------------------------------------
    # FSM helpers (best-effort)
    # ------------------------------------------------------------------

    def set_fsm_id(self, fsm_id: int) -> None:
        if hasattr(self._client, "SetFsmId"):
            self._client.SetFsmId(int(fsm_id))

    def start(self) -> None:
        if hasattr(self._client, "Start"):
            self._client.Start()

    def balance_stand(self, mode: int = 0) -> None:
        if hasattr(self._client, "BalanceStand"):
            self._client.BalanceStand(int(mode))

    # Low-level RPC access (same as hanger_boot_sequence)
    def _rpc_get_int(self, api_id: int) -> Optional[int]:
        try:
            code, data = self._client._Call(api_id, "{}")  # type: ignore[attr-defined]
            if code == 0 and data:
                import json

                return json.loads(data).get("data")
        except Exception:
            pass
        return None

    def fsm_id(self) -> Optional[int]:
        try:
            from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
        except Exception:
            return None
        return self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_ID)

    def fsm_mode(self) -> Optional[int]:
        try:
            from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_MODE
        except Exception:
            return None
        return self._rpc_get_int(ROBOT_API_ID_LOCO_GET_FSM_MODE)


__all__ = ["Robot", "ImuData"]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Quick smoke-test for ef_client Robot wrapper")
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--no-safety", action="store_true", help="Do not run safety boot sequence")
    args = parser.parse_args()

    bot = Robot(args.iface, safety_boot=not args.no_safety)
    bot.start_sensors()

    print("Waiting for sport state...")
    time.sleep(1.0)

    imu = bot.get_imu_data()
    print("IMU:", imu)

    print("FSM id:", bot.fsm_id(), "mode:", bot.fsm_mode())
