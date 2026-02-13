#!/usr/bin/env python3
"""
rgbd_cam.py
===========

Laptop-side Unitree camera viewer.
- DDS subscribe path (if a camera topic is published)
- Optional automatic RPC fallback via videohub GetImageSample()

No ROS2 and no local librealsense build required.
"""
from __future__ import annotations

import argparse
import importlib
import time
from threading import Lock
from typing import Any, List, Optional

import cv2  # type: ignore
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber

try:
    from unitree_sdk2py.go2.video.video_client import VideoClient
except Exception:
    VideoClient = None  # type: ignore

RPC_ERR_CLIENT_SEND = 3102
RPC_ERR_CLIENT_API_TIMEOUT = 3104


def _resolve_type(path: str) -> Any:
    if "::" in path:
        parts = [p for p in path.split("::") if p]
        if len(parts) < 2:
            raise ValueError(path)
        module = importlib.import_module(".".join(parts[:-1]))
        return getattr(module, parts[-1])
    if ":" in path:
        m, c = path.split(":", 1)
    else:
        m, c = path.rsplit(".", 1)
    module = importlib.import_module(m)
    return getattr(module, c)


def _resolve_first(candidates: List[str]) -> Optional[type]:
    for c in candidates:
        try:
            return _resolve_type(c)
        except Exception:
            pass
    return None


def _rgb_type_candidates(user_type: str) -> List[str]:
    return [
        user_type,
        "unitree_go::msg::dds_::Go2FrontVideoData_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_:Go2FrontVideoData_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_.Go2FrontVideoData_",
        "sensor_msgs::msg::dds_::Image_",
        "unitree_sdk2py.idl.sensor_msgs.msg.dds_:Image_",
    ]


def _depth_type_candidates(user_type: str) -> List[str]:
    return [
        user_type,
        "sensor_msgs::msg::dds_::Image_",
        "unitree_sdk2py.idl.sensor_msgs.msg.dds_:Image_",
    ]


def _bytes_from_seq(data: Any) -> bytes:
    if isinstance(data, bytes):
        return data
    if isinstance(data, bytearray):
        return bytes(data)
    return bytes(bytearray(data))


def _decode_ros_image(msg: Any) -> Optional[np.ndarray]:
    try:
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)
        enc = str(getattr(msg, "encoding", "")).lower()
        buf = _bytes_from_seq(msg.data)
    except Exception:
        return None

    if h <= 0 or w <= 0 or not buf:
        return None

    if enc in ("bgr8", "rgb8"):
        dtype, ch = np.uint8, 3
    elif enc in ("bgra8", "rgba8"):
        dtype, ch = np.uint8, 4
    elif enc in ("mono8", "8uc1"):
        dtype, ch = np.uint8, 1
    elif enc in ("mono16", "16uc1", "z16"):
        dtype, ch = np.uint16, 1
    else:
        if len(buf) == h * w * 3:
            dtype, ch, step = np.uint8, 3, w * 3
        elif len(buf) == h * w * 2:
            dtype, ch, step = np.uint16, 1, w * 2
        else:
            return None

    elem = np.dtype(dtype).itemsize
    min_step = w * ch * elem
    if step < min_step:
        step = min_step
    if len(buf) < h * step:
        return None

    if ch == 1:
        img = np.ndarray((h, w), dtype=dtype, buffer=buf, strides=(step, elem)).copy()
    else:
        img = np.ndarray((h, w, ch), dtype=dtype, buffer=buf, strides=(step, ch * elem, elem)).copy()

    if enc == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif enc == "rgba8":
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    elif enc == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def _decode_go2_front(msg: Any, pref: str) -> Optional[np.ndarray]:
    for field in [pref, "video720p", "video360p", "video180p"]:
        payload = getattr(msg, field, None)
        if payload is None:
            continue
        try:
            arr = np.frombuffer(_bytes_from_seq(payload), dtype=np.uint8)
        except Exception:
            continue
        if arr.size == 0:
            continue
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            return img
    return None


def _decode_any(msg: Any, pref: str) -> Optional[np.ndarray]:
    if hasattr(msg, "height") and hasattr(msg, "width") and hasattr(msg, "data"):
        return _decode_ros_image(msg)
    if hasattr(msg, "video720p") or hasattr(msg, "video360p") or hasattr(msg, "video180p"):
        return _decode_go2_front(msg, pref)
    return None


def _depth_to_colormap(depth: np.ndarray) -> Optional[np.ndarray]:
    if depth.dtype == np.uint8:
        disp = depth
    else:
        d = depth.astype(np.float32, copy=False)
        valid = np.isfinite(d)
        if not np.any(valid):
            return None
        vals = d[valid]
        dmin = float(np.min(vals))
        dmax = float(np.max(vals))
        if dmax <= dmin:
            disp = np.zeros_like(d, dtype=np.uint8)
        else:
            disp = np.clip(255.0 * (d - dmin) / (dmax - dmin), 0, 255).astype(np.uint8)
    return cv2.applyColorMap(disp, cv2.COLORMAP_JET)


class RGBDViewer:
    def __init__(self, video_field: str) -> None:
        self._lock = Lock()
        self._rgb: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._last_rgb_ts = 0.0
        self._rgb_count = 0
        self._depth_count = 0
        self._video_field = video_field
        self._rpc_last_code: Optional[int] = None

        cv2.namedWindow("G1 RGB", cv2.WINDOW_NORMAL)
        cv2.namedWindow("G1 Depth", cv2.WINDOW_NORMAL)

    def rgb_cb(self, msg: Any) -> None:
        img = _decode_any(msg, self._video_field)
        if img is None:
            return
        with self._lock:
            self._rgb = img
            self._last_rgb_ts = time.time()
            self._rgb_count += 1

    def depth_cb(self, msg: Any) -> None:
        img = _decode_any(msg, self._video_field)
        if img is None:
            return
        with self._lock:
            self._depth = img
            self._depth_count += 1

    def set_rpc_result(self, code: int, img: Optional[np.ndarray]) -> None:
        with self._lock:
            self._rpc_last_code = code
            if code == 0 and img is not None:
                self._rgb = img
                self._last_rgb_ts = time.time()

    def state(self):
        with self._lock:
            return {
                "rgb": None if self._rgb is None else self._rgb.copy(),
                "depth": None if self._depth is None else self._depth.copy(),
                "last_rgb_ts": self._last_rgb_ts,
                "rgb_count": self._rgb_count,
                "depth_count": self._depth_count,
                "rpc_last_code": self._rpc_last_code,
            }

    def render(self, source_label: str) -> None:
        s = self.state()
        rgb = s["rgb"]
        depth = s["depth"]

        if rgb is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for RGB...", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 220), 2)
            cv2.putText(blank, f"source: {source_label}", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)
            code = s["rpc_last_code"]
            if code not in (None, 0):
                cv2.putText(blank, f"RPC code: {code}", (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2)
            cv2.imshow("G1 RGB", blank)
        else:
            cv2.imshow("G1 RGB", rgb)

        if depth is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for Depth...", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 180, 0), 2)
            cv2.imshow("G1 Depth", blank)
        else:
            dcol = _depth_to_colormap(depth)
            if dcol is not None:
                cv2.imshow("G1 Depth", dcol)

        cv2.waitKey(1)


def _rpc_hint(code: int) -> str:
    if code == RPC_ERR_CLIENT_SEND:
        return "RPC_ERR_CLIENT_SEND (3102): videohub service not reachable or not running"
    if code == RPC_ERR_CLIENT_API_TIMEOUT:
        return "RPC_ERR_CLIENT_API_TIMEOUT (3104): request timed out"
    return f"RPC error code {code}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Unitree RGBD viewer (DDS + optional videohub fallback).")
    parser.add_argument("--iface", default="eth0", help="DDS network interface")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--mode", choices=["auto", "dds", "rpc"], default="auto", help="camera source mode")
    parser.add_argument("--rgb-topic", default="rt/frontvideostream", help="DDS RGB topic")
    parser.add_argument("--depth-topic", default="", help="DDS depth topic (optional)")
    parser.add_argument("--rgb-type", default="unitree_go::msg::dds_::Go2FrontVideoData_", help="DDS RGB type")
    parser.add_argument("--depth-type", default="sensor_msgs::msg::dds_::Image_", help="DDS depth type")
    parser.add_argument("--video-field", default="video720p", choices=["video720p", "video360p", "video180p"])
    parser.add_argument("--rpc-timeout", type=float, default=2.0, help="videohub RPC timeout seconds")
    parser.add_argument("--dds-wait", type=float, default=2.5, help="seconds to wait for DDS RGB before RPC fallback in auto mode")
    args = parser.parse_args()

    ChannelFactoryInitialize(args.domain_id, args.iface)
    viewer = RGBDViewer(args.video_field)

    rgb_sub = None
    depth_sub = None
    rgb_type = None
    depth_type = None

    if args.mode in ("auto", "dds"):
        rgb_type = _resolve_first(_rgb_type_candidates(args.rgb_type))
        if rgb_type is None:
            print("Could not resolve any RGB DDS type candidate.")
        else:
            rgb_sub = ChannelSubscriber(args.rgb_topic, rgb_type)
            rgb_sub.Init(viewer.rgb_cb, 10)
            print(f"DDS RGB subscribe: topic={args.rgb_topic}, type={rgb_type}")

        if args.depth_topic:
            depth_type = _resolve_first(_depth_type_candidates(args.depth_type))
            if depth_type is None:
                print("Depth type unresolved. Depth DDS disabled.")
            else:
                depth_sub = ChannelSubscriber(args.depth_topic, depth_type)
                depth_sub.Init(viewer.depth_cb, 10)
                print(f"DDS Depth subscribe: topic={args.depth_topic}, type={depth_type}")

    rpc_client = None
    if args.mode in ("auto", "rpc"):
        if VideoClient is None:
            print("VideoClient unavailable in this unitree_sdk2py build.")
        else:
            rpc_client = VideoClient()
            rpc_client.SetTimeout(float(args.rpc_timeout))
            rpc_client.Init()
            print("RPC videohub client initialized.")

    start = time.time()
    warned_auto = False

    try:
        while True:
            now = time.time()
            st = viewer.state()
            source = "dds"

            use_rpc = False
            if args.mode == "rpc":
                use_rpc = True
                source = "rpc"
            elif args.mode == "auto":
                if st["rgb_count"] == 0 and (now - start) > float(args.dds_wait):
                    use_rpc = True
                    source = "rpc"
                    if not warned_auto:
                        warned_auto = True
                        print(
                            f"No DDS RGB frames from '{args.rgb_topic}' after {args.dds_wait:.1f}s. "
                            "Falling back to RPC videohub."
                        )

            if use_rpc and rpc_client is not None:
                code, data = rpc_client.GetImageSample()
                if code == 0:
                    arr = np.frombuffer(bytes(data), dtype=np.uint8)
                    img = cv2.imdecode(arr, cv2.IMREAD_COLOR) if arr.size else None
                    viewer.set_rpc_result(code, img)
                else:
                    viewer.set_rpc_result(code, None)
                    if code in (RPC_ERR_CLIENT_SEND, RPC_ERR_CLIENT_API_TIMEOUT):
                        print(_rpc_hint(code))

            viewer.render(source)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if rgb_sub is not None:
            del rgb_sub
        if depth_sub is not None:
            del depth_sub
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
