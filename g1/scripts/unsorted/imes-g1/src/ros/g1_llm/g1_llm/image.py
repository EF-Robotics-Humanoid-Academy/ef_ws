#!/usr/bin/env python3
import os
import time
import base64
import threading
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy

class ImageSubscriber:
    """
    Receives frames and caches latest JPEG with timestamps, plus detailed logs.
    """
    def __init__(self, node, topic='d435/image_raw', queue_size=10):
        self._node = node
        self._topic = topic
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._latest_stamp_wall = 0.0         # wall time when cached
        self._latest_msg_header_stamp = None  # ROS stamp (sec,nsec) if available
        self._latest_shape = None             # (h, w, channels)
        self._latest_encoding = None
        self._frames_received = 0
        self._last_log_t = 0.0

        # Match camera publishers (SensorDataQoS): best-effort, volatile, small depth
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._sub = node.create_subscription(
            Image, self._topic, self._cb, qos
        )
        node.get_logger().info(f"[ImageSubscriber] Subscribed to {self._topic} with SensorDataQoS")

        # Watchdog: warn if no frames for > 3s
        self._watchdog_period = 1.0
        self._no_frame_warn_after_s = float(os.getenv("IMAGE_NO_FRAME_WARN_S", "3.0"))
        self._watchdog_timer = node.create_timer(self._watchdog_period, self._watchdog_tick)

    def _cb(self, msg: Image):
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                self._node.get_logger().warn("[ImageSubscriber] JPEG encode failed")
                return

            with self._lock:
                self._latest_jpeg = bytes(buf)
                self._latest_stamp_wall = time.time()
                self._latest_shape = getattr(cv_img, 'shape', None)
                self._latest_encoding = msg.encoding if hasattr(msg, "encoding") else "unknown"
                self._latest_msg_header_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec) if msg and msg.header else None
                self._frames_received += 1

            # Periodic concise log (every 2s)
            now = time.time()
            if now - self._last_log_t > 2.0 or self._frames_received == 1:
                h, w = (self._latest_shape[0], self._latest_shape[1]) if self._latest_shape else (-1, -1)
                # self._node.get_logger().info(
                #     f"[ImageSubscriber] frame#{self._frames_received} {w}x{h} enc={self._latest_encoding} "
                #     f"ros_stamp={self._latest_msg_header_stamp}"
                # )
                self._last_log_t = now

        except Exception as e:
            self._node.get_logger().error(f"[ImageSubscriber] Callback error: {e}")

    def _watchdog_tick(self):
        with self._lock:
            age = time.time() - self._latest_stamp_wall if self._latest_stamp_wall else float('inf')
            frames = self._frames_received
        if frames == 0 and age > self._no_frame_warn_after_s:
            self._node.get_logger().warn(
                f"[ImageSubscriber] No frames received for {age:.1f}s on {self._topic}. "
                "Check topic name and QoS."
            )

    def get_latest_jpeg(self, max_age_s: float = 2.0):
        """
        Returns (jpeg_bytes or None, age_seconds).
        """
        with self._lock:
            data = self._latest_jpeg
            stamp = self._latest_stamp_wall
        if not data:
            return None, float("inf")
        age = time.time() - stamp
        if max_age_s > 0 and age > max_age_s:
            return None, age
        return data, age

    def debug_snapshot(self) -> str:
        with self._lock:
            frames = self._frames_received
            age = time.time() - self._latest_stamp_wall if self._latest_stamp_wall else float('inf')
            shape = self._latest_shape
            enc = self._latest_encoding
            stamp = self._latest_msg_header_stamp
            size = len(self._latest_jpeg) if self._latest_jpeg else 0
        return (f"[ImageSubscriber] frames={frames} age={age:.2f}s size={size}B "
                f"shape={shape} enc={enc} ros_stamp={stamp}")


class ImageExplainer:
    """
    Wraps the vision caption call to the LLM using the latest image.
    """
    def __init__(self, node, image_subscriber, openai_client, model=None):
        self._node = node
        self._sub = image_subscriber
        self._client = openai_client
        self._model = model or os.getenv("OPENAI_VISION_MODEL", os.getenv("OPENAI_MODEL", "gpt-4o-mini"))

    def describe(self, focus: str = "") -> str:
        jpeg, age = self._sub.get_latest_jpeg(max_age_s=float(os.getenv("IMAGE_MAX_AGE_S", "2.0")))
        if jpeg is None:
            dbg = self._sub.debug_snapshot()
            self._node.get_logger().warn(f"[ImageExplainer] No fresh frame. {dbg}")
            return "I do not have a fresh camera frame yet."

        self._node.get_logger().info(f"[ImageExplainer] Using frame age={age:.2f}s bytes={len(jpeg)} focus='{focus}'")

        # Prepare data URI (works with OpenAI Chat Completions image inputs)
        b64 = base64.b64encode(jpeg).decode("ascii")
        data_url = f"data:image/jpeg;base64,{b64}"

        user_text = ("Describe this scene clearly and concisely in as few words as possible in 2 sentences."
                     " Mention key objects, spatial layout, and notable details.")
        if focus:
            user_text += f" Focus specifically on: {focus}."

        try:
            resp = self._client.chat.completions.create(
                model=self._model,
                max_tokens=int(os.getenv("VISION_MAX_TOKENS", "50")),
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": user_text},
                            {"type": "image_url", "image_url": {"url": data_url}},
                        ],
                    }
                ],
            )
            text = (resp.choices[0].message.content or "").strip()
            return text or "I analyzed the image, but no description was produced."
        except Exception as e:
            self._node.get_logger().error(f"[ImageExplainer] Vision call failed: {e}")
            return "I could not analyze the image due to an error."
