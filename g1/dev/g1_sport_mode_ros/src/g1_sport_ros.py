#!/usr/bin/env python3
"""ROS 2 ``cmd_vel`` bridge for the Unitree G1 locomotion client.

The ROS graph and the native Unitree SDK intentionally use different DDS
domains. Set ROS_DOMAIN_ID for ROS (normally 1); ``sdk_domain_id`` remains 0
unless the robot setup explicitly requires another native SDK domain.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class G1SportRosNode(Node):
    def __init__(self) -> None:
        super().__init__("g1_sport_ros_node")

        self.declare_parameter("interface", "eth0")
        self.declare_parameter("sdk_domain_id", 0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("deadband", 0.02)

        self.interface = str(self.get_parameter("interface").value)
        self.sdk_domain_id = int(self.get_parameter("sdk_domain_id").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self._validate_parameters()

        self.message_count = 0
        self.log_every_n_messages = 100
        self.last_command_monotonic = time.monotonic()
        self.motion_active = False
        self.client: LocoClient | None = None

        self.get_logger().info(
            f"Using network interface={self.interface}, SDK DDS domain={self.sdk_domain_id}"
        )
        ChannelFactoryInitialize(self.sdk_domain_id, self.interface)
        self.client = LocoClient()
        self.client.SetTimeout(10.0)
        self.client.Init()
        self.get_logger().info("G1 SDK initialized successfully")

        self.cmd_vel_subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.watchdog_timer = self.create_timer(
            min(0.1, self.command_timeout_sec / 2.0), self.watchdog_callback
        )
        self.get_logger().info(
            f"Subscribed to /cmd_vel; command watchdog={self.command_timeout_sec:.2f}s"
        )

    def _validate_parameters(self) -> None:
        if not 0 <= self.sdk_domain_id <= 232:
            raise ValueError("sdk_domain_id must be between 0 and 232")
        if not math.isfinite(self.command_timeout_sec) or self.command_timeout_sec <= 0:
            raise ValueError("command_timeout_sec must be finite and greater than zero")
        if not math.isfinite(self.max_linear_speed) or self.max_linear_speed <= 0:
            raise ValueError("max_linear_speed must be finite and greater than zero")
        if not math.isfinite(self.max_angular_speed) or self.max_angular_speed <= 0:
            raise ValueError("max_angular_speed must be finite and greater than zero")
        if not math.isfinite(self.deadband) or self.deadband < 0:
            raise ValueError("deadband must be finite and non-negative")

    def _sanitize(self, value: float, limit: float) -> float:
        value = float(value)
        if not math.isfinite(value) or abs(value) <= self.deadband:
            return 0.0
        return max(-limit, min(limit, value))

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Send a bounded command and refresh the command watchdog."""
        if self.client is None:
            return

        vx = self._sanitize(msg.linear.x, self.max_linear_speed)
        vy = self._sanitize(msg.linear.y, self.max_linear_speed)
        omega = self._sanitize(msg.angular.z, self.max_angular_speed)
        self.last_command_monotonic = time.monotonic()
        self.message_count += 1

        try:
            self.client.Move(vx, vy, omega)
            self.motion_active = any((vx, vy, omega))
            if self.message_count % self.log_every_n_messages == 0:
                self.get_logger().info(
                    f"Processed {self.message_count} commands; latest "
                    f"vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to send movement command: {exc}")

    def watchdog_callback(self) -> None:
        if self.client is None or not self.motion_active:
            return
        if time.monotonic() - self.last_command_monotonic <= self.command_timeout_sec:
            return
        try:
            self.client.StopMove()
            self.motion_active = False
            self.get_logger().warning("/cmd_vel timed out; robot movement stopped")
        except Exception as exc:
            self.get_logger().error(f"Failed to stop after /cmd_vel timeout: {exc}")

    def shutdown(self) -> None:
        """Stop locomotion exactly once during node shutdown."""
        if self.client is None:
            return
        try:
            self.client.StopMove()
            self.motion_active = False
            self.get_logger().info("G1 movement stopped")
        except Exception as exc:
            self.get_logger().error(f"Error during shutdown: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node: G1SportRosNode | None = None
    try:
        node = G1SportRosNode()
        node.get_logger().info("G1 Sport ROS node started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as exc:
        print(f"Error in G1 Sport ROS node: {exc}")
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
