"""
obstacle_detection.py
=====================

Real-time obstacle detection via DDS subscription to ``SportModeState_``.

Reads the ``range_obstacle[4]`` field (front / right / rear / left distances
in metres) and the robot's pose (``position``, ``imu_state.rpy``) so that
the orchestrator can mark obstacles on the occupancy grid and the locomotion
module can read the robot's current pose without a separate subscriber.

**Important**: ``ChannelFactoryInitialize`` must be called *before*
constructing an ``ObstacleDetector``.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed.  Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


class ObstacleDetector:
    """Subscribe to ``rt/sportmodestate`` and expose obstacle / pose queries.

    All public ``get_*`` / ``is_*`` methods are thread-safe.
    """

    # Indices into range_obstacle[4].
    # Verify empirically: walk the robot toward a known wall from each side
    # and check which index decreases.
    FRONT = 0
    RIGHT = 1
    REAR = 2
    LEFT = 3

    _DIRECTION_OFFSETS = {
        0: 0.0,            # FRONT
        1: -math.pi / 2,   # RIGHT
        2: math.pi,         # REAR
        3: math.pi / 2,     # LEFT
    }

    def __init__(
        self,
        warn_distance: float = 0.8,
        stop_distance: float = 0.4,
        topic: str = "rt/sportmodestate",
    ):
        """
        Args:
            warn_distance: Distance (m) at which an obstacle is flagged "near".
            stop_distance: Distance (m) at which an obstacle is flagged "blocked".
        """
        self.warn_distance = warn_distance
        self.stop_distance = stop_distance
        self.topic = topic

        self._lock = threading.Lock()
        self._range_obstacle: list[float] = [float("inf")] * 4
        self._position: list[float] = [0.0, 0.0, 0.0]
        self._yaw: float = 0.0
        self._last_update: float = 0.0
        self._sub: ChannelSubscriber | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Create and initialise the DDS subscriber."""
        self._sub = ChannelSubscriber(self.topic, SportModeState_)
        self._sub.Init(self._callback, 10)

    # ------------------------------------------------------------------
    # DDS callback (runs on SDK thread)
    # ------------------------------------------------------------------

    def _callback(self, msg: Any) -> None:
        with self._lock:
            self._range_obstacle = [float(v) for v in msg.range_obstacle]
            self._position = [float(v) for v in msg.position]
            # rpy may be a list or array -- index 2 is yaw
            try:
                self._yaw = float(msg.imu_state.rpy[2])
            except (AttributeError, IndexError, TypeError):
                pass
            self._last_update = time.time()

    # ------------------------------------------------------------------
    # Thread-safe queries
    # ------------------------------------------------------------------

    def get_ranges(self) -> list[float]:
        """Return a copy of the latest ``range_obstacle[4]``."""
        with self._lock:
            return list(self._range_obstacle)

    def get_pose(self) -> tuple[float, float, float]:
        """Return ``(x, y, yaw)`` from the latest ``SportModeState_``."""
        with self._lock:
            return (self._position[0], self._position[1], self._yaw)

    def is_blocked(self, direction: int = 0) -> bool:
        """True if ``range_obstacle[direction] < stop_distance``."""
        with self._lock:
            val = self._range_obstacle[direction]
        return 0.01 < val < self.stop_distance

    def is_near(self, direction: int = 0) -> bool:
        """True if ``range_obstacle[direction] < warn_distance``."""
        with self._lock:
            val = self._range_obstacle[direction]
        return 0.01 < val < self.warn_distance

    def front_blocked(self) -> bool:
        """Shorthand for ``is_blocked(FRONT)``."""
        return self.is_blocked(self.FRONT)

    def any_blocked(self) -> bool:
        """True if any of the 4 directions is below ``stop_distance``."""
        with self._lock:
            ranges = list(self._range_obstacle)
        return any(0.01 < r < self.stop_distance for r in ranges)

    def get_obstacle_world_positions(self) -> list[tuple[float, float]]:
        """For each direction where range < warn_distance, compute world (x, y).

        Returns a list of ``(x, y)`` tuples suitable for
        ``OccupancyGrid.set_obstacle_world()``.
        """
        with self._lock:
            ranges = list(self._range_obstacle)
            rx, ry = self._position[0], self._position[1]
            yaw = self._yaw

        positions: list[tuple[float, float]] = []
        for idx, offset in self._DIRECTION_OFFSETS.items():
            dist = ranges[idx]
            if dist <= 0.01 or dist >= 5.0:
                continue
            if dist >= self.warn_distance:
                continue
            angle = yaw + offset
            ox = rx + dist * math.cos(angle)
            oy = ry + dist * math.sin(angle)
            positions.append((ox, oy))
        return positions

    def is_stale(self, max_age: float = 1.0) -> bool:
        """True if last callback was more than *max_age* seconds ago."""
        with self._lock:
            ts = self._last_update
        if ts == 0.0:
            return True
        return (time.time() - ts) > max_age


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Listen to SportModeState_ and print ranges/pose.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--sport-topic", default="rt/odommodestate", help="SportModeState topic name")
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(args.domain_id, args.iface)

    detector = ObstacleDetector(topic=args.sport_topic)
    detector.start()

    print("Listening for SportModeState_ ... (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(0.5)
            if detector.is_stale():
                print("  (no data yet)")
                continue
            ranges = detector.get_ranges()
            x, y, yaw = detector.get_pose()
            print(
                f"  pos=({x:+.2f}, {y:+.2f})  yaw={math.degrees(yaw):+.1f} deg  "
                f"ranges=[F:{ranges[0]:.2f} R:{ranges[1]:.2f} "
                f"B:{ranges[2]:.2f} L:{ranges[3]:.2f}]  "
                f"front_blocked={detector.front_blocked()}"
            )
    except KeyboardInterrupt:
        print("\nStopped.")
