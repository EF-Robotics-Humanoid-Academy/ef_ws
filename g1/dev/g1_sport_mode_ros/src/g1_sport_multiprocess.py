#!/usr/bin/env python3
"""ROS2 bridge for the G1 sport-mode SDK using a separate SDK process.

The ROS graph and the native Unitree SDK use separate DDS domains. Commands
are bounded and stale commands are discarded so a busy ROS publisher cannot
build up a queue of old motion requests.
"""

import math
import multiprocessing as mp
import signal
import time
from multiprocessing import Process, Queue
from queue import Empty, Full

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def sdk_process(interface, sdk_domain_id, command_queue, status_queue):
    """Run Unitree SDK calls outside the ROS process."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    client = None
    running = True
    initialized = False

    def signal_handler(signum, frame):
        del signum, frame
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        ChannelFactoryInitialize(sdk_domain_id, interface)
        client = LocoClient()
        client.SetTimeout(10.0)
        client.Init()
        initialized = True
        status_queue.put(('initialized', True))

        while running:
            try:
                command = command_queue.get(timeout=0.1)
            except Empty:
                continue

            kind = command[0]
            if kind == 'move':
                _, vx, vy, omega = command
                client.Move(vx, vy, omega)
            elif kind == 'stop':
                client.StopMove()
            elif kind == 'shutdown':
                break
    except Exception as exc:
        status_queue.put(('error' if initialized else 'init_error', str(exc)))
    finally:
        if client is not None:
            try:
                client.StopMove()
            except Exception:
                pass


class G1SportRosNode(Node):
    """Translate safe, bounded ROS velocity commands into SDK commands."""

    def __init__(self):
        super().__init__('g1_sport_ros_node')

        self.declare_parameter('interface', 'eth0')
        self.declare_parameter('sdk_domain_id', 0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('deadband', 0.02)

        self.interface = str(self.get_parameter('interface').value)
        self.sdk_domain_id = int(self.get_parameter('sdk_domain_id').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self._validate_parameters()

        self.command_queue = Queue(maxsize=2)
        self.status_queue = Queue()
        self.sdk_process = None
        self.sdk_ready = False
        self.sdk_exit_logged = False
        self.motion_active = False
        self.last_command_time = time.monotonic()

        self.start_sdk_process()
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.status_timer = self.create_timer(0.1, self.check_sdk_status)
        self.watchdog_timer = self.create_timer(0.1, self.check_command_watchdog)

        self.get_logger().info(
            f'Using interface {self.interface}; SDK domain {self.sdk_domain_id}; '
            f'command timeout {self.command_timeout_sec:.2f}s'
        )

    def _validate_parameters(self):
        if not 0 <= self.sdk_domain_id <= 232:
            raise ValueError('sdk_domain_id must be between 0 and 232')
        numeric_values = (
            ('command_timeout_sec', self.command_timeout_sec),
            ('max_linear_speed', self.max_linear_speed),
            ('max_angular_speed', self.max_angular_speed),
        )
        for name, value in numeric_values:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be a finite positive number')
        if not math.isfinite(self.deadband) or self.deadband < 0:
            raise ValueError('deadband must be a finite non-negative number')

    def start_sdk_process(self):
        self.sdk_process = Process(
            target=sdk_process,
            args=(
                self.interface,
                self.sdk_domain_id,
                self.command_queue,
                self.status_queue,
            ),
        )
        self.sdk_process.start()

    def check_sdk_status(self):
        while True:
            try:
                status_type, data = self.status_queue.get_nowait()
            except Empty:
                break

            if status_type == 'initialized':
                self.sdk_ready = True
                self.get_logger().info('SDK process initialized successfully')
            elif status_type == 'init_error':
                self.get_logger().error(f'SDK initialization failed: {data}')
            elif status_type == 'error':
                self.get_logger().error(f'SDK error: {data}')

        if (
            self.sdk_process is not None
            and not self.sdk_process.is_alive()
            and not self.sdk_exit_logged
        ):
            self.sdk_exit_logged = True
            if self.sdk_ready:
                self.sdk_ready = False
                self.motion_active = False
                self.get_logger().error('SDK process exited; motion commands are disabled')
            else:
                self.get_logger().error('SDK process exited before becoming ready')

    def _sanitize(self, value, limit):
        if not math.isfinite(value) or abs(value) < self.deadband:
            return 0.0
        return max(-limit, min(limit, value))

    def _enqueue_latest(self, command):
        try:
            self.command_queue.put_nowait(command)
            return True
        except Full:
            try:
                self.command_queue.get_nowait()
                self.command_queue.put_nowait(command)
                return True
            except (Empty, Full):
                self.get_logger().warning('Dropping command because SDK queue is full')
                return False

    def cmd_vel_callback(self, msg):
        if not self.sdk_ready:
            return

        vx = self._sanitize(float(msg.linear.x), self.max_linear_speed)
        vy = self._sanitize(float(msg.linear.y), self.max_linear_speed)
        omega = self._sanitize(float(msg.angular.z), self.max_angular_speed)
        if self._enqueue_latest(('move', vx, vy, omega)):
            self.last_command_time = time.monotonic()
            self.motion_active = any((vx, vy, omega))
        else:
            self.motion_active = False
            self._enqueue_latest(('stop',))

    def check_command_watchdog(self):
        if (
            self.sdk_ready
            and self.motion_active
            and time.monotonic() - self.last_command_time > self.command_timeout_sec
        ):
            if self._enqueue_latest(('stop',)):
                self.motion_active = False
                self.get_logger().warning('cmd_vel timed out; stopping the robot')

    def shutdown(self):
        try:
            if self.sdk_process is not None and self.sdk_process.is_alive():
                self._enqueue_latest(('stop',))
                time.sleep(0.1)
                self._enqueue_latest(('shutdown',))
                self.sdk_process.join(timeout=2.0)
                if self.sdk_process.is_alive():
                    self.sdk_process.terminate()
                    self.sdk_process.join(timeout=1.0)
            self.sdk_ready = False
            self.get_logger().info('Shutdown complete')
        except Exception as exc:
            self.get_logger().error(f'Error during shutdown: {exc}')


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = G1SportRosNode()
        node.get_logger().info('G1 Sport ROS node started (multiprocess mode)')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Error in main: {exc}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    main()
