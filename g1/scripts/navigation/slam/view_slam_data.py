#!/usr/bin/env python3
"""
view_slam_data.py
=================

Probe key G1 SLAM DDS topics and report whether data is arriving.

Default topics checked:
- rt/unitree/slam_mapping/points (PointCloud2)
- rt/unitree/slam_mapping/odom (Odometry)
- rt/unitree/slam_relocation/points (PointCloud2)
- rt/unitree/slam_relocation/odom (Odometry)
- rt/slam_info (String JSON)
- rt/slam_key_info (String JSON)
- rt/unitree/slam_relocation/global_map (PointCloud2)
"""
from __future__ import annotations

import argparse
import importlib
import time
from dataclasses import dataclass, field
from threading import Event, Lock
from typing import Any, List, Optional

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber


@dataclass
class ProbeState:
    name: str
    type_name: str
    received: Event = field(default_factory=Event)
    count: int = 0
    last_string: Optional[str] = None
    error: Optional[str] = None
    _lock: Lock = field(default_factory=Lock)

    def on_msg(self, msg: Any) -> None:
        with self._lock:
            self.count += 1
            if self.last_string is not None:
                return
            if hasattr(msg, "data"):
                try:
                    self.last_string = str(msg.data)
                except Exception:
                    self.last_string = None
        self.received.set()


def _resolve_type(path: str) -> Any:
    if "::" in path:
        parts = [p for p in path.split("::") if p]
        if len(parts) < 2:
            raise ValueError(path)
        module = importlib.import_module(".".join(parts[:-1]))
        return getattr(module, parts[-1])
    if ":" in path:
        m, c = path.split(":", 1)
    else:
        m, c = path.rsplit(".", 1)
    module = importlib.import_module(m)
    return getattr(module, c)


def _resolve_first(candidates: List[str]) -> Optional[type]:
    for c in candidates:
        try:
            return _resolve_type(c)
        except Exception:
            pass
    return None


def _pc2_candidates() -> List[str]:
    return [
        "sensor_msgs::msg::dds_::PointCloud2_",
        "unitree_sdk2py.idl.sensor_msgs.msg.dds_:PointCloud2_",
        "unitree_sdk2py.idl.sensor_msgs.msg.dds_.PointCloud2_",
    ]


def _odom_candidates() -> List[str]:
    return [
        "nav_msgs::msg::dds_::Odometry_",
        "unitree_sdk2py.idl.nav_msgs.msg.dds_:Odometry_",
        "unitree_sdk2py.idl.nav_msgs.msg.dds_.Odometry_",
    ]


def _string_candidates() -> List[str]:
    return [
        "std_msgs::msg::dds_::String_",
        "unitree_sdk2py.idl.std_msgs.msg.dds_:String_",
        "unitree_sdk2py.idl.std_msgs.msg.dds_.String_",
    ]


#TOPICS = [
#    ("rt/unitree/slam_mapping/points", _pc2_candidates()),
#    ("rt/unitree/slam_mapping/odom", _odom_candidates()),
#    ("rt/unitree/slam_relocation/points", _pc2_candidates()),
#    ("rt/unitree/slam_relocation/odom", _odom_candidates()),
#    ("rt/slam_info", _string_candidates()),
#    ("rt/slam_key_info", _string_candidates()),
#    ("rt/unitree/slam_relocation/global_map", _pc2_candidates()),
#]

TOPICS = [
    ("rt/slam_mapping/points", _pc2_candidates()),
    ("rt/slam_mapping/odom", _odom_candidates()),
    ("rt/slam_relocation/points", _pc2_candidates()),
    ("rt/slam_relocation/odom", _odom_candidates()),
    ("rt/slam_info", _string_candidates()),
    ("rt/slam_key_info", _string_candidates()),
    ("rt/slam_relocation/global_map", _pc2_candidates()),

]

def main() -> None:
    parser = argparse.ArgumentParser(description="Probe G1 SLAM DDS topics and report accessibility.")
    parser.add_argument("--iface", default="eth0", help="DDS network interface")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for first message")
    args = parser.parse_args()

    ChannelFactoryInitialize(args.domain_id, args.iface)

    subs = []
    probes: List[ProbeState] = []

    for name, candidates in TOPICS:
        msg_type = _resolve_first(candidates)
        if msg_type is None:
            probes.append(ProbeState(name=name, type_name="UNRESOLVED", error="type_not_found"))
            continue
        probe = ProbeState(name=name, type_name=str(msg_type))
        probes.append(probe)
        try:
            sub = ChannelSubscriber(name, msg_type)
            sub.Init(probe.on_msg, 10)
            subs.append(sub)
        except Exception as exc:
            probe.error = f"subscribe_failed: {exc}"

    start = time.time()
    while (time.time() - start) < args.timeout:
        if all(p.received.is_set() or p.error or p.type_name == "UNRESOLVED" for p in probes):
            break
        time.sleep(0.05)

    print("\nSLAM DDS topic probe:\n")
    for p in probes:
        status = "OK" if p.count > 0 else "NO_DATA"
        if p.type_name == "UNRESOLVED":
            status = "TYPE_MISSING"
        if p.error:
            status = f"ERROR ({p.error})"
        print(f"- {p.name}")
        print(f"  type: {p.type_name}")
        print(f"  status: {status}")
        if p.last_string:
            preview = p.last_string.replace("\n", " ")
            if len(preview) > 120:
                preview = preview[:120] + "..."
            print(f"  sample: {preview}")

    _ = subs


if __name__ == "__main__":
    main()
