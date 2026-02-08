import cv2
import zmq
import time
import struct
from collections import deque
import numpy as np
import logging_mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data

logger_mp = logging_mp.get_logger(__name__, level=logging_mp.DEBUG)


class ROS2ImageSubscriber(Node):
    def __init__(self, topic_name="/g1_unit_5849/d435/image_raw"):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Subscribe to the image topic
        self.sub = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            qos_profile_sensor_data
        )

        logger_mp.info(
            f"[ROS2 Image Server] Subscribed to topic: {topic_name}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            logger_mp.error(f"[ROS2 Image Server] Error converting image: {e}")

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None


class ROS2ImageServer:
    def __init__(self, config, port=5555, Unit_Test=False):
        """
        config example:
        {
            'fps': 30,                                                        # frame per second
            'topic_name': '/g1_unit_5849/d435/image_raw',                    # ROS2 topic name
        }
        """
        logger_mp.info(config)
        self.fps = config.get('fps', 30)
        self.topic_name = config.get(
            'topic_name', '/g1_unit_5849/d435/image_raw')

        self.port = port
        self.Unit_Test = Unit_Test

        # Initialize ROS2
        rclpy.init()
        self.ros2_subscriber = ROS2ImageSubscriber(self.topic_name)

        # Start ROS2 spinning in a separate thread
        self.ros2_thread = threading.Thread(
            target=self._ros2_spin_thread, daemon=True)
        self.ros2_thread.start()

        # Set ZeroMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.port}")

        if self.Unit_Test:
            self._init_performance_metrics()

        logger_mp.info(
            f"[ROS2 Image Server] Server started on port {self.port}, subscribing to {self.topic_name}")
        logger_mp.info(
            "[ROS2 Image Server] Waiting for ROS2 image messages and client connections...")

    def _ros2_spin_thread(self):
        """Thread function to spin ROS2 node"""
        try:
            rclpy.spin(self.ros2_subscriber)
        except Exception as e:
            logger_mp.error(f"[ROS2 Image Server] ROS2 spin error: {e}")

    def _init_performance_metrics(self):
        self.frame_count = 0  # Total frames sent
        self.time_window = 1.0  # Time window for FPS calculation (in seconds)
        self.frame_times = deque()  # Timestamps of frames sent within the time window
        self.start_time = time.time()  # Start time of the streaming

    def _update_performance_metrics(self, current_time):
        # Add current time to frame times deque
        self.frame_times.append(current_time)
        # Remove timestamps outside the time window
        while self.frame_times and self.frame_times[0] < current_time - self.time_window:
            self.frame_times.popleft()
        # Increment frame count
        self.frame_count += 1

    def _print_performance_metrics(self, current_time):
        if self.frame_count % 30 == 0:
            elapsed_time = current_time - self.start_time
            real_time_fps = len(self.frame_times) / self.time_window
            logger_mp.info(
                f"[ROS2 Image Server] Real-time FPS: {real_time_fps:.2f}, Total frames sent: {self.frame_count}, Elapsed time: {elapsed_time:.2f} sec")

    def _close(self):
        # Shutdown ROS2
        self.ros2_subscriber.destroy_node()
        rclpy.shutdown()

        # Close ZMQ
        self.socket.close()
        self.context.term()
        logger_mp.info("[ROS2 Image Server] The server has been closed.")

    def send_process(self):
        try:
            # Calculate sleep time to maintain desired FPS
            sleep_time = 1.0 / self.fps

            while True:
                loop_start = time.time()

                # Get frame from ROS2 subscriber
                frame = self.ros2_subscriber.get_frame()

                if frame is None:
                    logger_mp.warning(
                        "[ROS2 Image Server] No frame received from ROS2 topic yet...")
                    time.sleep(0.1)  # Wait a bit before trying again
                    continue

                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame)
                if not ret:
                    logger_mp.error(
                        "[ROS2 Image Server] Frame imencode failed.")
                    continue

                jpg_bytes = buffer.tobytes()

                if self.Unit_Test:
                    timestamp = time.time()
                    frame_id = self.frame_count
                    # 8-byte double, 4-byte unsigned int
                    header = struct.pack('dI', timestamp, frame_id)
                    message = header + jpg_bytes
                else:
                    message = jpg_bytes

                self.socket.send(message)

                if self.Unit_Test:
                    current_time = time.time()
                    self._update_performance_metrics(current_time)
                    self._print_performance_metrics(current_time)

                # Control frame rate
                loop_end = time.time()
                elapsed = loop_end - loop_start
                if elapsed < sleep_time:
                    time.sleep(sleep_time - elapsed)

        except KeyboardInterrupt:
            logger_mp.warning("[ROS2 Image Server] Interrupted by user.")
        finally:
            self._close()


if __name__ == "__main__":
    config = {
        'fps': 30,
        'topic_name': '/g1_unit_5849/d435/image_raw',
    }

    server = ROS2ImageServer(config, Unit_Test=False)
    server.send_process()
