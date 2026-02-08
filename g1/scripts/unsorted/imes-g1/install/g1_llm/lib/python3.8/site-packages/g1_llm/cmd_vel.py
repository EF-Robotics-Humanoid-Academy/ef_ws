#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BaseCmdVel:
    """
    Minimal helper to drive base via geometry_msgs/Twist on a cmd_vel-like topic.
    Provides: drive(vx, vy, wz, duration_s), stop().
    """

    def __init__(self, node: Node,
                 topic: str = None,
                 rate_hz: float = None,
                 max_vx: float = None,
                 max_vy: float = None,
                 max_wz: float = None):
        self.node = node

        # Config via env with sane defaults
        self.topic = topic or os.getenv("CMD_VEL_TOPIC", "unfiltered/steamdeck_joy_teleop/cmd_vel")
        self.rate_hz = rate_hz or float(os.getenv("CMD_VEL_RATE_HZ", "20"))
        self.max_vx = max_vx if max_vx is not None else float(os.getenv("CMD_VEL_MAX_VX", "1.0"))   # m/s
        self.max_vy = max_vy if max_vy is not None else float(os.getenv("CMD_VEL_MAX_VY", "1.0"))   # m/s 
        self.max_wz = max_wz if max_wz is not None else float(os.getenv("CMD_VEL_MAX_WZ", "1.0"))   # rad/s

        self.pub = self.node.create_publisher(Twist, self.topic, 10)

        # Threaded runner state
        self._runner_lock = threading.Lock()
        self._runner = None
        self._stop_flag = False

        self.node.get_logger().info(self.node.colorize(
            f"BaseCmdVel on {self.topic} @ {self.rate_hz} Hz (limits vx={self.max_vx}, vy={self.max_vy}, wz={self.max_wz})",
            "blue"
        ))

    # ---------------------------- Public API ----------------------------------

    def drive(self, vx: float, vy: float, wz: float, duration_s: float):
        """
        Starts a background thread that publishes the given twist for duration_s,
        then sends a zero twist. If a previous drive is active, it is stopped first.
        """
        duration_s = max(0.0, float(duration_s))
        vx, vy, wz = self._clamp(vx, vy, wz)

        with self._runner_lock:
            self._kill_runner_unlocked()
            self._stop_flag = False
            self._runner = threading.Thread(
                target=self._run_profile, args=(vx, vy, wz, duration_s), daemon=True
            )
            self._runner.start()

        self.node.get_logger().info(self.node.colorize(
            f"cmd_vel RUN → vx={vx:.3f} m/s, vy={vy:.3f} m/s, wz={wz:.3f} rad/s for {duration_s:.2f} s",
            "yellow"
        ))

    def stop(self):
        """
        Stops any ongoing drive and publishes a single zero twist.
        """
        with self._runner_lock:
            self._kill_runner_unlocked()
        self._publish_zero()
        self.node.get_logger().info(self.node.colorize("cmd_vel STOP", "magenta"))

    # --------------------------- Internals -------------------------------------

    def _clamp(self, vx, vy, wz):
        vx = max(-self.max_vx, min(self.max_vx, float(vx)))
        vy = max(-self.max_vy, min(self.max_vy, float(vy)))
        wz = max(-self.max_wz, min(self.max_wz, float(wz)))
        return vx, vy, wz

    def _publish(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def _publish_zero(self):
        self._publish(0.0, 0.0, 0.0)

    def _run_profile(self, vx, vy, wz, duration_s):
        period = 1.0 / max(1e-3, self.rate_hz)
        t_end = time.monotonic() + duration_s

        try:
            while not self._stop_flag and time.monotonic() < t_end:
                self._publish(vx, vy, wz)
                time.sleep(period)
        except Exception as e:
            self.node.get_logger().error(f"cmd_vel runner error: {e}")
        finally:
            # always stop at the end
            self._publish_zero()

    def _kill_runner_unlocked(self):
        if self._runner and self._runner.is_alive():
            self._stop_flag = True
            self._runner.join(timeout=1.0)
        self._runner = None
        self._stop_flag = False
