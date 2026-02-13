#!/usr/bin/env python3
"""
lidar_points.py
===============

DDS PointCloud2 viewer for the G1 lidar (rt/utlidar/cloud_livox_mid360).
Displays a simple top-down XY view using OpenCV.
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Optional

try:
    import cv2  # type: ignore
    import numpy as np
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Missing deps. Ensure unitree_sdk2py, numpy, and OpenCV are installed."
    ) from exc


_DATATYPE_MAP = {
    1: ("int8", 1),
    2: ("uint8", 1),
    3: ("int16", 2),
    4: ("uint16", 2),
    5: ("int32", 4),
    6: ("uint32", 4),
    7: ("float32", 4),
    8: ("float64", 8),
}


class LidarViewer:
    def __init__(self, size: int, range_m: float, z_min: float, z_max: float, stride: int) -> None:
        self.size = int(size)
        self.range_m = float(range_m)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.stride = max(1, int(stride))
        self._latest: Optional[PointCloud2_] = None
        self._last_ts = 0.0

        cv2.namedWindow("G1 Lidar (top-down)", cv2.WINDOW_NORMAL)

    def cb(self, msg: PointCloud2_) -> None:
        self._latest = msg
        self._last_ts = time.time()

    def _decode_xyz(self, msg: PointCloud2_) -> Optional[np.ndarray]:
        fields = {f.name: f for f in msg.fields}
        if "x" not in fields or "y" not in fields or "z" not in fields:
            return None

        point_step = int(msg.point_step)
        if point_step <= 0:
            return None

        data = bytes(msg.data)
        if not data:
            return None

        # Build a structured dtype for x/y/z only.
        dtype = []
        for name in ("x", "y", "z"):
            f = fields[name]
            dt, size = _DATATYPE_MAP.get(int(f.datatype), ("float32", 4))
            dtype.append((name, dt))

        try:
            arr = np.frombuffer(data, dtype=np.dtype(dtype), count=len(data) // point_step)
            xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
            return xyz
        except Exception:
            return None

    def render(self) -> None:
        if self._latest is None:
            return

        msg = self._latest
        xyz = self._decode_xyz(msg)
        if xyz is None:
            return

        if self.stride > 1:
            xyz = xyz[:: self.stride]

        img = np.zeros((self.size, self.size, 3), dtype=np.uint8)
        center = self.size // 2
        scale = self.size / (2.0 * self.range_m)

        xs = xyz[:, 0]
        ys = xyz[:, 1]
        zs = xyz[:, 2]

        mask = (
            (zs >= self.z_min)
            & (zs <= self.z_max)
            & (np.abs(xs) <= self.range_m)
            & (np.abs(ys) <= self.range_m)
        )
        xs = xs[mask]
        ys = ys[mask]
        zs = zs[mask]

        for x, y, z in zip(xs, ys, zs):
            px = int(center + x * scale)
            py = int(center - y * scale)
            if 0 <= px < self.size and 0 <= py < self.size:
                t = (z - self.z_min) / max(1e-6, (self.z_max - self.z_min))
                col = (0, int(255 * t), int(255 * (1.0 - t)))
                img[py, px] = col

        cv2.line(img, (center - 10, center), (center + 10, center), (0, 255, 0), 1)
        cv2.line(img, (center, center - 10), (center, center + 10), (0, 255, 0), 1)

        cv2.imshow("G1 Lidar (top-down)", img)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="G1 DDS Lidar PointCloud2 viewer.")
    parser.add_argument("--iface", default="enp1s0", help="network interface for DDS")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--topic", default="rt/utlidar/cloud_livox_mid360", help="DDS pointcloud topic")
    parser.add_argument("--size", type=int, default=800, help="display size (pixels)")
    parser.add_argument("--range-m", type=float, default=10.0, help="display range in meters")
    parser.add_argument("--z-min", type=float, default=-1.0, help="min z height to show")
    parser.add_argument("--z-max", type=float, default=2.0, help="max z height to show")
    parser.add_argument("--stride", type=int, default=2, help="subsample points (1=all)")
    args = parser.parse_args()

    ChannelFactoryInitialize(args.domain_id, args.iface)
    viewer = LidarViewer(args.size, args.range_m, args.z_min, args.z_max, args.stride)

    sub = ChannelSubscriber(args.topic, PointCloud2_)
    sub.Init(viewer.cb, 10)

    print(f"Listening on {args.topic} (domain={args.domain_id}, iface={args.iface})")
    try:
        while True:
            viewer.render()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
