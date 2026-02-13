#!/usr/bin/env python3
"""
rgbd_cam.py
===========

ROS2 camera viewer for the G1. Displays RGB (and optional depth) using OpenCV.

Defaults to /frontvideostream for RGB. Provide --depth-topic if available.
"""
from __future__ import annotations

import argparse
import os
from typing import Optional

try:
    import cv2  # type: ignore
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Missing ROS2 Python deps. Ensure rclpy, sensor_msgs, cv_bridge, and OpenCV are installed."
    ) from exc


class RGBDViewer(Node):
    def __init__(self, rgb_topic: str, depth_topic: Optional[str]) -> None:
        super().__init__("g1_rgbd_viewer")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._bridge = CvBridge()
        self._rgb = None
        self._depth = None

        self._rgb_sub = self.create_subscription(Image, rgb_topic, self._rgb_cb, qos)
        self._depth_sub = None
        if depth_topic:
            self._depth_sub = self.create_subscription(Image, depth_topic, self._depth_cb, qos)

        cv2.namedWindow("G1 RGB", cv2.WINDOW_NORMAL)
        if depth_topic:
            cv2.namedWindow("G1 Depth", cv2.WINDOW_NORMAL)

        self._timer = self.create_timer(0.03, self._render)

    def _rgb_cb(self, msg: Image) -> None:
        try:
            self._rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self._rgb = None

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg)
            self._depth = depth
        except Exception:
            self._depth = None

    def _render(self) -> None:
        if self._rgb is not None:
            cv2.imshow("G1 RGB", self._rgb)

        if self._depth is not None:
            depth = self._depth
            if depth.dtype != np.uint8:
                dmin = np.nanmin(depth)
                dmax = np.nanmax(depth)
                if dmax > dmin:
                    disp = (255.0 * (depth - dmin) / (dmax - dmin)).astype(np.uint8)
                else:
                    disp = np.zeros_like(depth, dtype=np.uint8)
            else:
                disp = depth
            disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
            cv2.imshow("G1 Depth", disp)

        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 RGBD viewer.")
    parser.add_argument("--rgb-topic", default="/frontvideostream", help="RGB Image topic")
    parser.add_argument("--depth-topic", default="", help="Depth Image topic (optional)")
    parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID")
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = RGBDViewer(args.rgb_topic, args.depth_topic or None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
