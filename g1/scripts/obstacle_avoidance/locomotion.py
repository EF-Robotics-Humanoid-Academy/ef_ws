"""
locomotion.py
=============

Proportional-control locomotion wrapper for the Unitree G1.

Given a target world-coordinate waypoint, drives the robot toward it using
heading-then-forward proportional control.  Reads the robot's live pose
from an ``ObstacleDetector`` (which subscribes to ``rt/sportmodestate``).

The ``walk_to`` method accepts an optional *check_obstacle* callback so
the orchestrator can inject obstacle-detection logic without coupling
this module to the map or planner.
"""
from __future__ import annotations

import math
import time
from typing import Callable

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed.  Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from obstacle_detection import ObstacleDetector


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Locomotion controller
# ---------------------------------------------------------------------------

class Locomotion:
    """Drive the G1 toward waypoints using proportional velocity control.

    Requires an initialised ``LocoClient`` (robot in FSM-200) and a running
    ``ObstacleDetector`` for pose feedback.
    """

    # Proportional gains (tune on real hardware)
    Kp_lin: float = 0.8   # distance -> vx
    Kp_yaw: float = 1.5   # heading error -> vyaw

    def __init__(
        self,
        loco_client: LocoClient,
        obstacle_detector: ObstacleDetector,
        max_vx: float = 0.3,
        max_vyaw: float = 0.5,
        position_tolerance: float = 0.2,
        yaw_tolerance: float = 0.15,
    ):
        """
        Args:
            loco_client:        Initialised ``LocoClient`` in FSM-200.
            obstacle_detector:  Running ``ObstacleDetector`` (for pose via
                                ``get_pose()``).
            max_vx:             Maximum forward speed (m/s).
            max_vyaw:           Maximum yaw rate (rad/s).
            position_tolerance: Distance (m) to consider "arrived".
            yaw_tolerance:      Heading error (rad) below which the robot
                                starts moving forward (above this it turns
                                in place first).
        """
        self.client = loco_client
        self.detector = obstacle_detector
        self.max_vx = max_vx
        self.max_vyaw = max_vyaw
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self._aborted = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def walk_to(
        self,
        target_x: float,
        target_y: float,
        final_yaw: float | None = None,
        timeout: float = 30.0,
        tick: float = 0.1,
        check_obstacle: Callable[[], bool] | None = None,
        stale_timeout: float = 1.0,
    ) -> bool:
        """Walk the robot to ``(target_x, target_y)`` in world coordinates.

        Control loop (runs at ``1/tick`` Hz):

        1. Read pose from ``ObstacleDetector.get_pose()``.
        2. Compute heading error and distance to target.
        3. If ``|heading_error| > yaw_tolerance``: turn in place.
        4. Otherwise: move forward with heading correction.
        5. Call ``check_obstacle()`` each tick -- if True, stop and return
           ``False`` so the orchestrator can replan.

        Args:
            target_x:       Goal x (metres).
            target_y:       Goal y (metres).
            final_yaw:      Optional heading to face after arriving (radians).
            timeout:        Max time for this walk (seconds).
            tick:           Control loop period (seconds).
            check_obstacle: Optional callable.  Return ``True`` to abort.

        Returns:
            ``True`` if waypoint reached, ``False`` if aborted.
        """
        self._aborted = False
        t0 = time.time()

        while not self._aborted:
            elapsed = time.time() - t0
            if elapsed > timeout:
                self.client.StopMove()
                return False
            if self.detector.is_stale(max_age=stale_timeout):
                self.client.StopMove()
                return False

            cx, cy, cyaw = self.detector.get_pose()
            dx = target_x - cx
            dy = target_y - cy
            distance = math.hypot(dx, dy)

            # Arrived?
            if distance < self.position_tolerance:
                self.client.StopMove()
                if final_yaw is not None:
                    self._turn_to(final_yaw, timeout=max(1.0, timeout - elapsed),
                                  tick=tick)
                return True

            # Heading toward target
            target_heading = math.atan2(dy, dx)
            heading_error = _wrap_angle(target_heading - cyaw)

            if abs(heading_error) > self.yaw_tolerance:
                # Turn in place
                vyaw = _clamp(self.Kp_yaw * heading_error,
                              -self.max_vyaw, self.max_vyaw)
                vx = 0.0
            else:
                # Move forward with heading correction
                vx = _clamp(self.Kp_lin * distance, 0.0, self.max_vx)
                vyaw = _clamp(self.Kp_yaw * heading_error,
                              -self.max_vyaw, self.max_vyaw)

            self.client.Move(vx, 0.0, vyaw, continous_move=True)

            # Obstacle check
            if check_obstacle is not None and check_obstacle():
                self.client.StopMove()
                return False

            time.sleep(tick)

        # _aborted was set externally
        self.client.StopMove()
        return False

    def stop(self) -> None:
        """Immediately stop the robot."""
        self.client.StopMove()

    def abort(self) -> None:
        """Set abort flag so ``walk_to`` exits on next tick."""
        self._aborted = True

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _turn_to(self, target_yaw: float, timeout: float = 10.0,
                 tick: float = 0.1) -> None:
        """Turn in place until facing *target_yaw* (radians)."""
        t0 = time.time()
        while (time.time() - t0) < timeout:
            _, _, cyaw = self.detector.get_pose()
            err = _wrap_angle(target_yaw - cyaw)
            if abs(err) < self.yaw_tolerance:
                break
            vyaw = _clamp(self.Kp_yaw * err, -self.max_vyaw, self.max_vyaw)
            self.client.Move(0.0, 0.0, vyaw, continous_move=True)
            time.sleep(tick)
        self.client.StopMove()


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Locomotion quick test (walk 1m forward).")
    parser.add_argument("--iface", default="enp1s0", help="network interface for DDS")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--sport-topic", default="rt/odommodestate", help="SportModeState topic name")
    parser.add_argument("--stale-timeout", type=float, default=1.0, help="stop if sensor data is stale (s)")
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(args.domain_id, args.iface)

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    detector = ObstacleDetector(topic=args.sport_topic)
    detector.start()
    time.sleep(0.5)

    if detector.is_stale():
        sys.exit("No SportModeState_ data.  Is the robot connected?")

    walker = Locomotion(loco, detector, max_vx=0.2)

    x0, y0, yaw0 = detector.get_pose()
    target_x = x0 + 1.0 * math.cos(yaw0)
    target_y = y0 + 1.0 * math.sin(yaw0)

    print(f"Walking 1m forward: ({x0:.2f},{y0:.2f}) -> ({target_x:.2f},{target_y:.2f})")
    try:
        ok = walker.walk_to(target_x, target_y, timeout=15.0, stale_timeout=args.stale_timeout)
        print("Reached!" if ok else "Aborted / timed out.")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping.")
    finally:
        walker.stop()
