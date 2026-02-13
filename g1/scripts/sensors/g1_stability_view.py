from __future__ import annotations

import argparse
import importlib
import logging
import math
import threading
import time
from collections import deque
from typing import Any, List, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_ODOM = "rt/odom"

LOG = logging.getLogger("g1_stability_view")


def _configure_logging(level: int) -> None:
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def _try_import(module_path: str):
    try:
        return importlib.import_module(module_path)
    except Exception:
        LOG.debug("Failed to import %s", module_path, exc_info=True)
        return None


def _resolve_type(type_path: str) -> Any:
    """Resolve a python type from a dotted path like package.module:ClassName or package.module.ClassName."""
    if ":" in type_path:
        module_path, class_name = type_path.split(":", 1)
    else:
        module_path, class_name = type_path.rsplit(".", 1)
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def _type_path_candidates(type_path: str) -> List[str]:
    """
    Return candidate python type paths for a given DDS type name or python path.
    """
    if "::" not in type_path:
        return [type_path]

    parts = type_path.split("::")
    class_name = parts[-1]
    namespace = parts[:-1]
    candidates = [
        f"unitree_sdk2py.idl.{'.'.join(namespace)}._{class_name}:{class_name}",
        f"unitree_sdk2py.idl.{'.'.join(namespace)}:{class_name}",
    ]

    # Common ROS2 IDL location in unitree_sdk2py
    if namespace[:1] in (["sensor_msgs"], ["nav_msgs"]):
        candidates.append(f"unitree_sdk2py.idl.ros2._{class_name}:{class_name}")

    candidates.append(type_path)
    return candidates


def _resolve_lowstate_type() -> Optional[type]:
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        module = _try_import(module_path)
        if module and hasattr(module, "LowState_"):
            return getattr(module, "LowState_")
    return None


def _resolve_odom_type(type_path: str) -> Optional[type]:
    last_exc: Optional[Exception] = None
    for candidate in _type_path_candidates(type_path):
        try:
            return _resolve_type(candidate)
        except Exception as exc:
            last_exc = exc
    if last_exc:
        LOG.debug("Odom type resolve failed for %s", type_path, exc_info=last_exc)
    return None


class StabilityStream:
    def __init__(self, history_seconds: float, max_tilt_deg: float):
        self.history_seconds = float(history_seconds)
        self.max_tilt_deg = float(max_tilt_deg)
        self.lock = threading.Lock()

        self.last_lowstate = None
        self.last_odom = None

        self.ts = deque()
        self.roll = deque()
        self.pitch = deque()
        self.yaw = deque()
        self.tilt = deque()
        self.stability = deque()

        self.odom_x = deque()
        self.odom_y = deque()

    def lowstate_cb(self, msg: Any):
        now = time.time()
        imu = msg.imu_state
        roll = float(imu.rpy[0])
        pitch = float(imu.rpy[1])
        yaw = float(imu.rpy[2])

        tilt = math.degrees(math.sqrt(roll * roll + pitch * pitch))
        stability = max(0.0, 1.0 - (tilt / self.max_tilt_deg))

        with self.lock:
            self.last_lowstate = msg
            self.ts.append(now)
            self.roll.append(math.degrees(roll))
            self.pitch.append(math.degrees(pitch))
            self.yaw.append(math.degrees(yaw))
            self.tilt.append(tilt)
            self.stability.append(stability)
            self._trim(now)

    def odom_cb(self, msg: Any):
        with self.lock:
            self.last_odom = msg
            try:
                self.odom_x.append(float(msg.pose.pose.position.x))
                self.odom_y.append(float(msg.pose.pose.position.y))
            except Exception:
                pass

    def snapshot(self):
        with self.lock:
            if not self.ts:
                return None
            return {
                "t0": self.ts[0],
                "ts": list(self.ts),
                "roll": list(self.roll),
                "pitch": list(self.pitch),
                "yaw": list(self.yaw),
                "tilt": list(self.tilt),
                "stability": list(self.stability),
                "odom_x": list(self.odom_x),
                "odom_y": list(self.odom_y),
            }

    def _trim(self, now):
        cutoff = now - self.history_seconds
        while self.ts and self.ts[0] < cutoff:
            self.ts.popleft()
            self.roll.popleft()
            self.pitch.popleft()
            self.yaw.popleft()
            self.tilt.popleft()
            self.stability.popleft()
        max_odom = max(1, int(self.history_seconds * 10))
        while len(self.odom_x) > max_odom:
            self.odom_x.popleft()
            self.odom_y.popleft()


def main():
    parser = argparse.ArgumentParser(
        description="Stream Unitree G1 IMU + odom and visualize stability."
    )
    parser.add_argument("--iface", default="eth0", help="Network interface")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--history", type=float, default=30.0, help="History length (seconds)")
    parser.add_argument(
        "--max-tilt-deg",
        type=float,
        default=20.0,
        help="Tilt (deg) that maps to stability=0.0",
    )
    parser.add_argument(
        "--lowstate-topic",
        default=TOPIC_LOWSTATE,
        help="Lowstate topic name",
    )
    parser.add_argument(
        "--odom-topic",
        default=TOPIC_ODOM,
        help="Odometry topic name",
    )
    parser.add_argument(
        "--odom-type",
        default="nav_msgs::msg::dds_::Odometry_",
        help="Odometry DDS type or python path",
    )
    parser.add_argument("--log-level", type=str, default="INFO", help="Log level (DEBUG, INFO, WARNING)")
    args = parser.parse_args()

    level = getattr(logging, args.log_level.upper(), logging.INFO)
    _configure_logging(level)

    chan = _try_import("unitree_sdk2py.core.channel")
    if not chan or not hasattr(chan, "ChannelFactoryInitialize") or not hasattr(chan, "ChannelSubscriber"):
        LOG.error("unitree_sdk2py.core.channel not available; cannot start subscribers")
        return 2

    LowState = _resolve_lowstate_type()
    if LowState is None:
        LOG.error("LowState_ type not found in unitree_sdk2py (unitree_hg or unitree_go)")
        return 2

    LOG.info("Connecting via iface=%s domain_id=%s", args.iface, args.domain_id)
    chan.ChannelFactoryInitialize(args.domain_id, args.iface)

    stream = StabilityStream(args.history, args.max_tilt_deg)

    lowstate_sub = chan.ChannelSubscriber(args.lowstate_topic, LowState)
    lowstate_sub.Init(stream.lowstate_cb, 10)

    odom_type = _resolve_odom_type(args.odom_type)
    if odom_type:
        odom_sub = chan.ChannelSubscriber(args.odom_topic, odom_type)
        odom_sub.Init(stream.odom_cb, 10)
        LOG.info("Subscribed to odom: %s (%s)", args.odom_topic, args.odom_type)
    else:
        odom_sub = None
        LOG.warning(
            "Odom type not resolved for %s. Skipping odom subscriber.",
            args.odom_type,
        )

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.canvas.manager.set_window_title("G1 Stability")

    ax_rpy = axes[0]
    ax_stab = axes[1]

    ax_rpy.set_title("Roll/Pitch/Yaw (deg)")
    ax_rpy.set_xlabel("Time (s)")
    ax_rpy.set_ylabel("Degrees")
    line_roll, = ax_rpy.plot([], [], label="roll")
    line_pitch, = ax_rpy.plot([], [], label="pitch")
    line_yaw, = ax_rpy.plot([], [], label="yaw")
    line_tilt, = ax_rpy.plot([], [], label="tilt")
    ax_rpy.legend(loc="upper right")

    ax_stab.set_title("Stability (1.0=level)")
    ax_stab.set_xlabel("Time (s)")
    ax_stab.set_ylabel("Score")
    line_stab, = ax_stab.plot([], [], color="tab:green")
    ax_stab.set_ylim(-0.05, 1.05)

    status = ax_stab.text(
        0.01,
        0.92,
        "Waiting for rt/lowstate...",
        transform=ax_stab.transAxes,
    )

    def _update(_frame):
        snap = stream.snapshot()
        if snap is None:
            return line_roll, line_pitch, line_yaw, line_tilt, line_stab, status

        t0 = snap["t0"]
        ts = [t - t0 for t in snap["ts"]]

        line_roll.set_data(ts, snap["roll"])
        line_pitch.set_data(ts, snap["pitch"])
        line_yaw.set_data(ts, snap["yaw"])
        line_tilt.set_data(ts, snap["tilt"])
        line_stab.set_data(ts, snap["stability"])

        ax_rpy.set_xlim(max(0.0, ts[-1] - args.history), ts[-1] + 0.1)
        all_vals = snap["roll"] + snap["pitch"] + snap["yaw"] + snap["tilt"]
        vmin = min(all_vals) - 5.0
        vmax = max(all_vals) + 5.0
        ax_rpy.set_ylim(vmin, vmax)
        ax_stab.set_xlim(max(0.0, ts[-1] - args.history), ts[-1] + 0.1)

        status_text = (
            f"tilt={snap['tilt'][-1]:.2f} deg  stability={snap['stability'][-1]:.2f}"
        )
        if snap["odom_x"] and snap["odom_y"]:
            status_text += f"  odom=({snap['odom_x'][-1]:.2f}, {snap['odom_y'][-1]:.2f})"
        status.set_text(status_text)
        return line_roll, line_pitch, line_yaw, line_tilt, line_stab, status

    anim = FuncAnimation(fig, _update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        LOG.info("Interrupted; exiting")
    finally:
        # Keep reference alive through show() for some backends
        _ = anim
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
