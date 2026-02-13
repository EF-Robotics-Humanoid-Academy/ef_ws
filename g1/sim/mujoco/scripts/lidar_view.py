import argparse
import os
import sys
import time
import struct

import numpy as np

try:
    import cv2
except Exception as exc:
    print("OpenCV (cv2) is required for this script.")
    print(f"Import error: {exc}")
    sys.exit(1)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if SIM_DIR not in sys.path:
    sys.path.insert(0, SIM_DIR)

import config

TOPIC_LIDAR_POINTS = "rt/utlidar/cloud"

POINTFIELD_FLOAT32 = 7


def _get_point_offsets(fields):
    offsets = {}
    for f in fields:
        offsets[f.name] = (f.offset, f.datatype)
    return offsets


def _extract_points_xy(msg: PointCloud2_, max_points: int = 4000):
    offsets = _get_point_offsets(msg.fields)
    if "x" not in offsets or "y" not in offsets:
        return []
    x_off, x_type = offsets["x"]
    y_off, y_type = offsets["y"]
    if x_type != POINTFIELD_FLOAT32 or y_type != POINTFIELD_FLOAT32:
        return []

    total = int(msg.width) * int(msg.height)
    if total <= 0:
        return []
    step = max(1, total // max_points)
    data = bytes(msg.data)
    endian = ">" if msg.is_bigendian else "<"
    pts = []
    for i in range(0, total, step):
        base = i * msg.point_step
        try:
            x = struct.unpack_from(endian + "f", data, base + x_off)[0]
            y = struct.unpack_from(endian + "f", data, base + y_off)[0]
        except struct.error:
            break
        pts.append((x, y))
    return pts


def _make_lidar_image(points, size=512, scale=20.0):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cx = size // 2
    cy = size // 2
    if not points:
        cv2.putText(img, "No LiDAR data", (20, size // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 1)
        return img
    for x, y in points:
        ix = int(cx + x * scale)
        iy = int(cy - y * scale)
        if 0 <= ix < size and 0 <= iy < size:
            img[iy, ix] = (0, 255, 0)
    cv2.line(img, (cx, 0), (cx, size - 1), (60, 60, 60), 1)
    cv2.line(img, (0, cy), (size - 1, cy), (60, 60, 60), 1)
    return img


class LidarSubscriber:
    def __init__(self):
        self.last_msg = None
        self.last_ts = 0.0

    def cb(self, msg: PointCloud2_):
        self.last_msg = msg
        self.last_ts = time.time()


def run_lidar(iface: str, domain_id: int):
    ChannelFactoryInitialize(domain_id, iface)

    lidar = LidarSubscriber()
    lidar_sub = ChannelSubscriber(TOPIC_LIDAR_POINTS, PointCloud2_)
    lidar_sub.Init(lidar.cb, 10)

    cv2.namedWindow("LiDAR")

    while True:
        pts = []
        if lidar.last_msg is not None:
            pts = _extract_points_xy(lidar.last_msg)
        lidar_vis = _make_lidar_image(pts)

        cv2.imshow("LiDAR", lidar_vis)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Show LiDAR point cloud data.")
    parser.add_argument("--iface", default="lo", help="Network interface (use 'lo' for sim)")
    parser.add_argument("--domain_id", type=int, default=1, help="DDS domain id")
    args = parser.parse_args()

    run_lidar(args.iface, args.domain_id)


if __name__ == "__main__":
    main()
