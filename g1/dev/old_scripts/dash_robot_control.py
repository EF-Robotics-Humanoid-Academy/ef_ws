#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import base64
from dataclasses import dataclass
import json
import logging
import os
from pathlib import Path
import platform
import struct
import threading
import time
from typing import Any
import sys

import dash
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
from dash import Input, Output, State, dcc, html

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(SCRIPT_DIR, "dash_robot_control.log")
LOGGER = logging.getLogger("dash_robot_control")
LOGGER.setLevel(logging.INFO)
if not LOGGER.handlers:
    _formatter = logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s")
    _stream_handler = logging.StreamHandler()
    _stream_handler.setFormatter(_formatter)
    LOGGER.addHandler(_stream_handler)
    try:
        _file_handler = logging.FileHandler(LOG_PATH)
    except OSError:
        LOG_PATH = os.path.join("/tmp", f"dash_robot_control_{os.getpid()}.log")
        _file_handler = logging.FileHandler(LOG_PATH)
    _file_handler.setFormatter(_formatter)
    LOGGER.addHandler(_file_handler)
    LOGGER.info("Dash robot control log path: %s", LOG_PATH)

from dev.sdk_client import Robot


ROBOT_LOCK = threading.Lock()
ROBOT_INSTANCE: Robot | None = None
ROBOT_INIT_ERR: str | None = None
ROBOT_IFACE = "eth0"
ROBOT_LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_livox_mid360"
RGBD_HOST = os.environ.get("G1_RGBD_HOST", "10.34.0.83")
RGBD_PORT = int(os.environ.get("G1_RGBD_PORT", "5555"))
RGBD_TOPIC = os.environ.get("G1_RGBD_TOPIC", "")
IMU_HISTORY: deque[tuple[float, float, float, float]] = deque(maxlen=300)
DEPTH_LOCK = threading.Lock()
DEPTH_PREVIEW: "_DepthPreviewReceiver | None" = None
RGB_LOCK = threading.Lock()
RGB_PREVIEW: "_RgbPreviewReceiver | None" = None
RGBD_LOCK = threading.Lock()
RGBD_PREVIEW: "_ZmqRgbdPreviewReceiver | None" = None
LIVOX_LOCK = threading.Lock()
LIVOX_PREVIEW: "_LivoxPointsReceiver | None" = None

LOWLEVEL_NOT_USED_IDX = 29
LOWLEVEL_COMMAND_TOPIC = "rt/lowcmd"
LOWLEVEL_JOINT_LAYOUT: list[tuple[str, int, str, float, float]] = [
    ("left_leg", 0, "hip_pitch", -2.5307, 2.8798),
    ("left_leg", 1, "hip_roll", -0.5236, 2.9671),
    ("left_leg", 2, "hip_yaw", -2.7576, 2.7576),
    ("left_leg", 3, "knee", -0.087267, 2.8798),
    ("left_leg", 4, "ankle_pitch", -0.87267, 0.5236),
    ("left_leg", 5, "ankle_roll", -0.2618, 0.2618),
    ("right_leg", 6, "hip_pitch", -2.5307, 2.8798),
    ("right_leg", 7, "hip_roll", -2.9671, 0.5236),
    ("right_leg", 8, "hip_yaw", -2.7576, 2.7576),
    ("right_leg", 9, "knee", -0.087267, 2.8798),
    ("right_leg", 10, "ankle_pitch", -0.87267, 0.5236),
    ("right_leg", 11, "ankle_roll", -0.2618, 0.2618),
    ("waist", 12, "yaw", -2.618, 2.618),
    ("waist", 13, "roll", -0.52, 0.52),
    ("waist", 14, "pitch", -0.52, 0.52),
    ("left_arm", 15, "shoulder_pitch", -3.0892, 2.6704),
    ("left_arm", 16, "shoulder_roll", -1.5882, 2.2515),
    ("left_arm", 17, "shoulder_yaw", -2.618, 2.618),
    ("left_arm", 18, "elbow", -1.0472, 2.0944),
    ("left_arm", 19, "wrist_roll", -1.9722, 1.9722),
    ("left_arm", 20, "wrist_pitch", -1.6144, 1.6144),
    ("left_arm", 21, "wrist_yaw", -1.6144, 1.6144),
    ("right_arm", 22, "shoulder_pitch", -3.0892, 2.6704),
    ("right_arm", 23, "shoulder_roll", -2.2515, 1.5882),
    ("right_arm", 24, "shoulder_yaw", -2.618, 2.618),
    ("right_arm", 25, "elbow", -1.0472, 2.0944),
    ("right_arm", 26, "wrist_roll", -1.9722, 1.9722),
    ("right_arm", 27, "wrist_pitch", -1.6144, 1.6144),
    ("right_arm", 28, "wrist_yaw", -1.6144, 1.6144),
]


@dataclass(frozen=True)
class LowLevelJointSpec:
    group: str
    motor_index: int
    name: str
    limit_min: float
    limit_max: float

    @property
    def label(self) -> str:
        return f"{self.motor_index}: {self.group} {self.name}"


LOWLEVEL_JOINT_SPECS = [LowLevelJointSpec(*item) for item in LOWLEVEL_JOINT_LAYOUT]
LOWLEVEL_JOINT_BY_INDEX = {spec.motor_index: spec for spec in LOWLEVEL_JOINT_SPECS}
LOWLEVEL_JOINT_OPTIONS = [
    {"label": spec.label, "value": spec.motor_index} for spec in LOWLEVEL_JOINT_SPECS
]


def _resolve_lowstate_type() -> type | None:
    for module_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            module = __import__(module_path, fromlist=["LowState_"])
        except Exception:
            continue
        if hasattr(module, "LowState_"):
            return getattr(module, "LowState_")
    return None


class _LowLevelJointController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._initialized = False
        self._iface = ""
        self._domain_id = 0
        self._pub: Any = None
        self._cmd: Any = None
        self._crc: Any = None
        self._positions = [0.0] * 29
        self._mode_machine = 0
        self._state_ts = 0.0
        self._thread: threading.Thread | None = None
        self._active_joint: int | None = None
        self._target = 0.0
        self._params = (0.0, 0.0, 30.0, 1.5)
        self._status = "Low-level controller idle."
        self._error: str | None = None

    def _ensure(self, iface: str, domain_id: int = 0) -> None:
        if self._initialized and self._iface == str(iface) and self._domain_id == int(domain_id):
            return
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        # Use the SDK Python CRC when the native architecture library is missing.
        if not getattr(CRC, "_wbc_python_crc_fallback", False):
            _crc_sdk_init = CRC.__init__
            def _crc_init_python_fallback(self):
                try:
                    _crc_sdk_init(self)
                except OSError as exc:
                    if "crc_" not in str(exc) or "No such file" not in str(exc):
                        raise
                    self.platform = "Python"
            CRC.__init__ = _crc_init_python_fallback
            CRC._wbc_python_crc_fallback = True


        lowstate_type = _resolve_lowstate_type()
        if lowstate_type is None:
            raise RuntimeError("LowState_ type not found in unitree_sdk2py.")

        ChannelFactoryInitialize(int(domain_id), str(iface))
        pub = ChannelPublisher(LOWLEVEL_COMMAND_TOPIC, LowCmd_)
        pub.Init()
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = 0
        cmd.motor_cmd[LOWLEVEL_NOT_USED_IDX].q = 1.0
        for idx in range(29):
            cmd.motor_cmd[idx].mode = 1

        def _lowstate_cb(msg: Any) -> None:
            try:
                positions = [float(msg.motor_state[idx].q) for idx in range(29)]
                mode_machine = int(getattr(msg, "mode_machine", 0))
            except Exception:
                return
            with self._lock:
                self._positions = positions
                self._mode_machine = mode_machine
                self._state_ts = time.time()
            self._ready.set()

        sub = ChannelSubscriber("rt/lowstate", lowstate_type)
        sub.Init(_lowstate_cb, 200)
        with self._lock:
            self._pub = pub
            self._sub = sub
            self._cmd = cmd
            self._crc = CRC()
            self._iface = str(iface)
            self._domain_id = int(domain_id)
            self._initialized = True
            self._status = f"Low-level controller ready on {LOWLEVEL_COMMAND_TOPIC}."
            self._error = None

    def start_move(
        self,
        *,
        joint_index: int,
        target: float,
        max_increment: float,
        dq: float,
        tau: float,
        pk: float,
        pd: float,
        iface: str,
        domain_id: int = 0,
    ) -> str:
        spec = LOWLEVEL_JOINT_BY_INDEX.get(int(joint_index))
        if spec is None:
            return f"Invalid low-level joint index: {joint_index}"
        target = max(spec.limit_min, min(spec.limit_max, float(target)))
        max_increment = max(0.0005, abs(float(max_increment or 0.01)))
        try:
            self._ensure(iface, domain_id)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._status = f"Low-level init failed: {exc}"
            return self._status

        if not self._ready.wait(timeout=1.0):
            with self._lock:
                self._status = "Waiting for rt/lowstate before sending low-level command."
            return self._status

        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                if self._active_joint == spec.motor_index:
                    self._target = target
                    self._params = (float(dq), float(tau), float(pk), float(pd))
                    self._status = f"Updated {spec.label} target to {target:.3f} rad."
                    return self._status
                self._status = "Low-level move already running on another joint; wait for it to finish."
                return self._status
            self._status = f"Moving {spec.label} to {target:.3f} rad."
            self._error = None
            self._active_joint = spec.motor_index
            self._target = target
            self._params = (float(dq), float(tau), float(pk), float(pd))
            self._thread = threading.Thread(
                target=self._run_move,
                args=(spec, max_increment),
                daemon=True,
            )
            self._thread.start()
            return self._status

    def _run_move(
        self,
        spec: LowLevelJointSpec,
        max_increment: float,
    ) -> None:
        try:
            joint = spec.motor_index
            with self._lock:
                commanded = float(self._positions[joint])
            while True:
                with self._lock:
                    target = float(self._target)
                    dq, tau, pk, pd = self._params
                error = float(target) - commanded
                if abs(error) <= 1e-6:
                    break
                if abs(error) <= max_increment:
                    commanded = float(target)
                else:
                    commanded += max_increment * (1.0 if error > 0.0 else -1.0)
                self._publish_joint(joint, commanded, dq=dq, tau=tau, pk=pk, pd=pd)
                with self._lock:
                    self._status = f"Moving {spec.label}: command {commanded:.3f} / target {target:.3f} rad."
                time.sleep(0.02)
            with self._lock:
                self._status = f"Low-level move complete for {spec.label}: {target:.3f} rad."
                self._error = None
                self._active_joint = None
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._status = f"Low-level move failed: {exc}"
                self._active_joint = None

    def _publish_joint(self, joint: int, q: float, *, dq: float, tau: float, pk: float, pd: float) -> None:
        with self._lock:
            positions = list(self._positions)
            mode_machine = int(self._mode_machine)
            cmd = self._cmd
            pub = self._pub
            crc = self._crc
        if cmd is None or pub is None or crc is None:
            raise RuntimeError("Low-level publisher is not initialized.")
        cmd.mode_machine = mode_machine
        for idx in range(29):
            mc = cmd.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(positions[idx])
            mc.dq = 0.0
            mc.kp = 0.0
            mc.kd = 0.0
            mc.tau = 0.0
        tgt = cmd.motor_cmd[int(joint)]
        tgt.q = float(q)
        tgt.dq = float(dq)
        tgt.kp = float(pk)
        tgt.kd = float(pd)
        tgt.tau = float(tau)
        cmd.motor_cmd[LOWLEVEL_NOT_USED_IDX].q = 1.0
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    def snapshot(self) -> tuple[str, str | None, float, bool]:
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return self._status, self._error, self._state_ts, running

    def current_position(self, joint_index: int) -> float | None:
        with self._lock:
            if self._state_ts <= 0.0:
                return None
            return float(self._positions[int(joint_index)])


LOWLEVEL_CONTROLLER = _LowLevelJointController()


def _load_hand_sdk() -> tuple[type, list[float], list[float]]:
    try:
        from sdk_hand import Dex3HandController, HAND_CLOSED, HAND_OPEN

        return Dex3HandController, list(HAND_OPEN), list(HAND_CLOSED)
    except Exception:
        modules_dir = Path(__file__).resolve().parents[1] / "modules"
        if str(modules_dir) not in sys.path:
            sys.path.insert(0, str(modules_dir))
        from sdk_hand import Dex3HandController, HAND_CLOSED, HAND_OPEN

        return Dex3HandController, list(HAND_OPEN), list(HAND_CLOSED)


class _GripController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._controllers: dict[str, Any] = {}
        self._thread: threading.Thread | None = None
        self._active_hand = "right"
        self._current = 100.0
        self._target = 100.0
        self._max_increment = 2.0
        self._status = "Grip controller idle."
        self._error: str | None = None

    def start_move(self, *, hand: str, percent: float, max_increment: float, iface: str, domain_id: int = 0) -> str:
        side = str(hand or "right").strip().lower()
        if side not in {"left", "right"}:
            side = "right"
        target = max(0.0, min(100.0, float(percent)))
        step = max(0.1, abs(float(max_increment or 2.0)))
        with self._lock:
            self._active_hand = side
            self._target = target
            self._max_increment = step
            if self._thread is not None and self._thread.is_alive():
                self._status = f"Updated {side} grip target to {target:.1f}%."
                return self._status
            self._status = f"Moving {side} grip to {target:.1f}%."
            self._error = None
            self._thread = threading.Thread(target=self._run, args=(side, str(iface), int(domain_id)), daemon=True)
            self._thread.start()
            return self._status

    def _controller_for(self, hand: str, iface: str, domain_id: int) -> Any:
        key = f"{hand}:{iface}:{domain_id}"
        if key not in self._controllers:
            cls, _open, _closed = _load_hand_sdk()
            self._controllers[key] = cls(hand=hand, iface=iface, domain_id=domain_id)
        return self._controllers[key]

    def _run(self, hand: str, iface: str, domain_id: int) -> None:
        try:
            controller = self._controller_for(hand, iface, domain_id)
            _cls, hand_open, hand_closed = _load_hand_sdk()
            published = False
            while True:
                with self._lock:
                    target = float(self._target)
                    step = float(self._max_increment)
                    if self._active_hand != hand:
                        hand = self._active_hand
                        controller = self._controller_for(hand, iface, domain_id)
                    current = float(self._current)
                error = target - current
                if abs(error) <= 1e-6:
                    if not published:
                        alpha = target / 100.0
                        targets = [start + (stop - start) * alpha for start, stop in zip(hand_open, hand_closed)]
                        controller.write_targets_once(targets, kp=1.2, kd=0.05, tau=0.05, timeout=0)
                        published = True
                    break
                if abs(error) <= step:
                    current = target
                else:
                    current += step * (1.0 if error > 0.0 else -1.0)
                alpha = current / 100.0
                targets = [start + (stop - start) * alpha for start, stop in zip(hand_open, hand_closed)]
                controller.write_targets_once(targets, kp=1.2, kd=0.05, tau=0.05, timeout=0)
                published = True
                with self._lock:
                    self._current = current
                    self._status = f"{hand.title()} grip command {current:.1f}% / target {target:.1f}%."
                time.sleep(0.02)
            with self._lock:
                self._status = f"{hand.title()} grip reached {target:.1f}%."
                self._error = None
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._status = f"Grip command failed: {exc}"

    def snapshot(self) -> tuple[str, str | None, bool]:
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return self._status, self._error, running


GRIP_CONTROLLER = _GripController()


class _RgbPreviewReceiver:
    def __init__(self, rgb_port: int, width: int, height: int, fps: int) -> None:
        self.rgb_port = int(rgb_port)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))

        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._latest_ts = 0.0
        self._error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self) -> tuple[bytes | None, float, str | None]:
        with self._lock:
            return (self._latest_jpeg, self._latest_ts, self._error)

    def _run(self) -> None:
        pipeline = None
        try:
            import cv2
            import gi
            import numpy as np

            gi.require_version("Gst", "1.0")
            gi.require_version("GstApp", "1.0")
            from gi.repository import Gst
        except Exception as exc:
            with self._lock:
                self._error = f"RGB receiver unavailable: {exc}"
            return

        try:
            Gst.init(None)
            pipeline = Gst.parse_launch(
                f"udpsrc port={self.rgb_port} caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=true sync=false drop=true"
            )
            sink = pipeline.get_by_name("sink")
            if sink is None:
                raise RuntimeError("appsink not found")
            pipeline.set_state(Gst.State.PLAYING)

            wait_ns = int(Gst.SECOND // self.fps)

            while self._running:
                sample = sink.emit("try-pull-sample", wait_ns)
                if not sample:
                    time.sleep(0.01)
                    continue
                buf = sample.get_buffer()
                if buf is None:
                    continue
                raw = np.frombuffer(buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
                expected = self.width * self.height * 3
                if raw.size != expected:
                    continue
                bgr = raw.reshape((self.height, self.width, 3))
                ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if not ok:
                    continue
                with self._lock:
                    self._latest_jpeg = enc.tobytes()
                    self._latest_ts = time.time()
                    self._error = None
        except Exception as exc:
            with self._lock:
                self._error = f"RGB stream error: {exc}"
        finally:
            try:
                if pipeline is not None:
                    pipeline.set_state(Gst.State.NULL)  # type: ignore[name-defined]
            except Exception:
                pass


class _DepthPreviewReceiver:
    def __init__(self, depth_port: int, width: int, height: int, fps: int) -> None:
        self.depth_port = int(depth_port)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1, int(fps))

        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._latest_ts = 0.0
        self._latest_center_depth_m: float | None = None
        self._latest_near_coverage: float | None = None
        self._error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self) -> tuple[bytes | None, float, float | None, float | None, str | None]:
        with self._lock:
            return (
                self._latest_jpeg,
                self._latest_ts,
                self._latest_center_depth_m,
                self._latest_near_coverage,
                self._error,
            )

    def _run(self) -> None:
        pipeline = None
        try:
            import cv2
            import gi
            import numpy as np

            gi.require_version("Gst", "1.0")
            gi.require_version("GstApp", "1.0")
            from gi.repository import Gst
        except Exception as exc:
            with self._lock:
                self._error = f"Depth receiver unavailable: {exc}"
            return

        try:
            Gst.init(None)
            pipeline = Gst.parse_launch(
                f"udpsrc port={self.depth_port} caps=application/x-rtp,media=video,encoding-name=H264,payload=97 ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink name=sink emit-signals=true sync=false drop=true"
            )
            sink = pipeline.get_by_name("sink")
            if sink is None:
                raise RuntimeError("appsink not found")
            pipeline.set_state(Gst.State.PLAYING)

            cmap = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(256, 1), cv2.COLORMAP_PLASMA)
            cmap = cmap.reshape(256, 3).astype(np.int16)
            wait_ns = int(Gst.SECOND // self.fps)

            while self._running:
                sample = sink.emit("try-pull-sample", wait_ns)
                if not sample:
                    time.sleep(0.01)
                    continue
                buf = sample.get_buffer()
                if buf is None:
                    continue
                raw = np.frombuffer(buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
                expected = self.width * self.height * 3
                if raw.size != expected:
                    continue
                depth_bgr = raw.reshape((self.height, self.width, 3))
                ok, enc = cv2.imencode(".jpg", depth_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    continue

                center_size = max(8, min(self.width, self.height) // 12)
                cx = self.width // 2
                cy = self.height // 2
                x0 = max(0, cx - center_size)
                x1 = min(self.width, cx + center_size)
                y0 = max(0, cy - center_size)
                y1 = min(self.height, cy + center_size)
                center = depth_bgr[y0:y1, x0:x1]
                roi = depth_bgr[int(self.height * 0.25) : int(self.height * 0.70), int(self.width * 0.30) : int(self.width * 0.70)]
                center_depth_m: float | None = None
                near_cov: float | None = None
                if center.size > 0 and roi.size > 0:
                    center_pix = center.reshape(-1, 3).astype(np.int16)
                    diff = center_pix[:, None, :] - cmap[None, :, :]
                    dist2 = (diff * diff).sum(axis=2)
                    center_idx = np.argmin(dist2, axis=1)
                    center_depth_m = float(np.median(center_idx) / 255.0 * 6.0)

                    roi_pix = roi.reshape(-1, 3).astype(np.int16)
                    roi_diff = roi_pix[:, None, :] - cmap[None, :, :]
                    roi_dist2 = (roi_diff * roi_diff).sum(axis=2)
                    roi_idx = np.argmin(roi_dist2, axis=1)
                    near_cov = float(np.mean(roi_idx <= int((1.0 / 6.0) * 255.0)))

                with self._lock:
                    self._latest_jpeg = enc.tobytes()
                    self._latest_ts = time.time()
                    self._latest_center_depth_m = center_depth_m
                    self._latest_near_coverage = near_cov
                    self._error = None
        except Exception as exc:
            with self._lock:
                self._error = f"Depth stream error: {exc}"
        finally:
            try:
                if pipeline is not None:
                    pipeline.set_state(Gst.State.NULL)  # type: ignore[name-defined]
            except Exception:
                pass


class _ZmqRgbdPreviewReceiver:
    def __init__(self, host: str, port: int, topic: str = "", fps: int = 8) -> None:
        self.host = str(host)
        self.port = int(port)
        self.topic = str(topic)
        self.fps = max(1, int(fps))

        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest_rgb_jpeg: bytes | None = None
        self._latest_depth_jpeg: bytes | None = None
        self._latest_ts = 0.0
        self._latest_center_depth_m: float | None = None
        self._latest_near_coverage: float | None = None
        self._error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self) -> tuple[bytes | None, bytes | None, float, float | None, float | None, str | None]:
        with self._lock:
            return (
                self._latest_rgb_jpeg,
                self._latest_depth_jpeg,
                self._latest_ts,
                self._latest_center_depth_m,
                self._latest_near_coverage,
                self._error,
            )

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np
            import zmq
        except Exception as exc:
            with self._lock:
                self._error = f"RGBD receiver unavailable: {exc}"
            return

        context = None
        socket = None
        try:
            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            socket.setsockopt(zmq.SUBSCRIBE, self.topic.encode("utf-8"))
            socket.setsockopt(zmq.RCVTIMEO, 500)
            socket.connect(f"tcp://{self.host}:{self.port}")
            min_dt = 1.0 / float(self.fps)
            last_update = 0.0

            while self._running:
                try:
                    parts = socket.recv_multipart()
                except zmq.Again:
                    with self._lock:
                        if self._latest_ts <= 0.0:
                            self._error = f"Waiting for RGBD frames on tcp://{self.host}:{self.port}"
                    continue
                except Exception as exc:
                    with self._lock:
                        self._error = f"RGBD receive error: {exc}"
                    time.sleep(0.25)
                    continue

                if len(parts) >= 4:
                    parts = parts[-3:]
                if len(parts) < 2:
                    continue
                now = time.time()
                if now - last_update < min_dt:
                    continue
                last_update = now

                rgb_jpeg = bytes(parts[0])
                depth_png = bytes(parts[1])
                depth_scale = 0.001
                if len(parts) >= 3 and len(parts[2]) >= 4:
                    try:
                        depth_scale = float(struct.unpack("f", parts[2][:4])[0])
                    except Exception:
                        depth_scale = 0.001

                depth_raw = cv2.imdecode(np.frombuffer(depth_png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                if depth_raw is None:
                    continue
                if depth_raw.ndim == 3:
                    depth_raw = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)

                valid = depth_raw > 0
                max_depth_m = 4.0
                depth_m = depth_raw.astype(np.float32) * float(depth_scale)
                depth_norm = np.zeros(depth_raw.shape, dtype=np.uint8)
                depth_norm[valid] = np.clip((depth_m[valid] / max_depth_m) * 255.0, 0, 255).astype(np.uint8)
                depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_PLASMA)
                depth_vis[~valid] = (0, 0, 0)

                h, w = depth_raw.shape[:2]
                center_size = max(8, min(w, h) // 12)
                cx = w // 2
                cy = h // 2
                center = depth_m[
                    max(0, cy - center_size) : min(h, cy + center_size),
                    max(0, cx - center_size) : min(w, cx + center_size),
                ]
                roi = depth_m[int(h * 0.25) : int(h * 0.70), int(w * 0.30) : int(w * 0.70)]
                center_valid = center[center > 0]
                roi_valid = roi[roi > 0]
                center_depth_m = float(np.median(center_valid)) if center_valid.size else None
                near_cov = float(np.mean((roi > 0) & (roi <= 1.0))) if roi.size else None

                ok, depth_enc = cv2.imencode(".jpg", depth_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
                if not ok:
                    continue

                with self._lock:
                    self._latest_rgb_jpeg = rgb_jpeg
                    self._latest_depth_jpeg = depth_enc.tobytes()
                    self._latest_ts = now
                    self._latest_center_depth_m = center_depth_m
                    self._latest_near_coverage = near_cov
                    self._error = None
        except Exception as exc:
            with self._lock:
                self._error = f"RGBD stream error: {exc}"
        finally:
            try:
                if socket is not None:
                    socket.close(0)
                if context is not None:
                    context.term()
            except Exception:
                pass


class _LivoxPointsReceiver:
    """
    Fallback point receiver using the same Livox SDK wrappers as
    /home/ag/ef_ws/g1/scripts/sensors/live_points.py.
    """

    def __init__(self) -> None:
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._frames_xyz: deque[Any] = deque(maxlen=15)
        self._latest_ts = 0.0
        self._error: str | None = None
        self._mount = os.environ.get("LIVOX_MOUNT", "upside_down").lower()
        if self._mount not in {"normal", "upside_down"}:
            self._mount = "upside_down"

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def snapshot(self) -> tuple[Any | None, float, str | None]:
        with self._lock:
            if not self._frames_xyz:
                return None, self._latest_ts, self._error
            try:
                import numpy as np

                merged = np.concatenate(list(self._frames_xyz), axis=0)
            except Exception as exc:
                self._error = f"Livox frame merge failed: {exc}"
                return None, self._latest_ts, self._error
            return merged, self._latest_ts, self._error

    def _run(self) -> None:
        try:
            import numpy as np
        except Exception as exc:
            with self._lock:
                self._error = f"numpy unavailable: {exc}"
            return

        sensors_dir = Path("/home/ag/ef_ws/g1/scripts/sensors")
        if str(sensors_dir) not in sys.path:
            sys.path.insert(0, str(sensors_dir))

        base_cls = None
        sdk2 = False
        try:
            from livox2_python import Livox2 as base_cls  # type: ignore[assignment]

            sdk2 = True
        except Exception:
            try:
                from livox_python import Livox as base_cls  # type: ignore[assignment]
            except Exception as exc:
                with self._lock:
                    self._error = f"Livox wrapper import failed: {exc}"
                return

        receiver = self

        class _DashLivox(base_cls):  # type: ignore[misc, valid-type]
            def __init__(self) -> None:
                if sdk2:
                    cfg = sensors_dir / "mid360_config.json"
                    if not cfg.exists():
                        raise RuntimeError(f"Missing Livox config: {cfg}")
                    host_ip = os.environ.get("HOST_IP", "192.168.123.222")
                    super().__init__(str(cfg), host_ip=host_ip)
                else:
                    super().__init__()

            def handle_points(self, xyz: Any) -> None:  # type: ignore[override]
                arr = np.asarray(xyz, dtype=np.float32)
                if arr.ndim != 2 or arr.shape[1] < 3:
                    return
                arr = arr[:, :3]
                if receiver._mount == "upside_down":
                    arr = arr * np.array([1.0, -1.0, -1.0], dtype=np.float32)
                if arr.shape[0] > 100_000:
                    step = max(1, arr.shape[0] // 100_000)
                    arr = arr[::step]
                with receiver._lock:
                    receiver._frames_xyz.append(arr)
                    receiver._latest_ts = time.time()
                    receiver._error = None

        lidar = None
        try:
            lidar = _DashLivox()
            while self._running:
                time.sleep(0.02)
        except Exception as exc:
            with self._lock:
                self._error = f"Livox stream error: {exc}"
        finally:
            try:
                if lidar is not None:
                    lidar.shutdown()
            except Exception:
                pass


def get_livox_preview() -> _LivoxPointsReceiver:
    global LIVOX_PREVIEW
    with LIVOX_LOCK:
        if LIVOX_PREVIEW is None:
            LIVOX_PREVIEW = _LivoxPointsReceiver()
        return LIVOX_PREVIEW


def get_depth_preview(robot: Robot) -> _DepthPreviewReceiver:
    global DEPTH_PREVIEW
    with DEPTH_LOCK:
        if (
            DEPTH_PREVIEW is None
            or DEPTH_PREVIEW.depth_port != int(robot.depth_port)
            or DEPTH_PREVIEW.width != int(robot.rgb_width)
            or DEPTH_PREVIEW.height != int(robot.rgb_height)
            or DEPTH_PREVIEW.fps != int(robot.rgb_fps)
        ):
            if DEPTH_PREVIEW is not None:
                DEPTH_PREVIEW.stop()
            DEPTH_PREVIEW = _DepthPreviewReceiver(
                depth_port=int(robot.depth_port),
                width=int(robot.rgb_width),
                height=int(robot.rgb_height),
                fps=int(robot.rgb_fps),
            )
        return DEPTH_PREVIEW


def get_rgb_preview(robot: Robot) -> _RgbPreviewReceiver:
    global RGB_PREVIEW
    with RGB_LOCK:
        if (
            RGB_PREVIEW is None
            or RGB_PREVIEW.rgb_port != int(robot.rgb_port)
            or RGB_PREVIEW.width != int(robot.rgb_width)
            or RGB_PREVIEW.height != int(robot.rgb_height)
            or RGB_PREVIEW.fps != int(robot.rgb_fps)
        ):
            if RGB_PREVIEW is not None:
                RGB_PREVIEW.stop()
            RGB_PREVIEW = _RgbPreviewReceiver(
                rgb_port=int(robot.rgb_port),
                width=int(robot.rgb_width),
                height=int(robot.rgb_height),
                fps=int(robot.rgb_fps),
            )
        return RGB_PREVIEW


def get_rgbd_preview() -> _ZmqRgbdPreviewReceiver:
    global RGBD_PREVIEW
    with RGBD_LOCK:
        if (
            RGBD_PREVIEW is None
            or RGBD_PREVIEW.host != RGBD_HOST
            or RGBD_PREVIEW.port != RGBD_PORT
            or RGBD_PREVIEW.topic != RGBD_TOPIC
        ):
            if RGBD_PREVIEW is not None:
                RGBD_PREVIEW.stop()
            RGBD_PREVIEW = _ZmqRgbdPreviewReceiver(host=RGBD_HOST, port=RGBD_PORT, topic=RGBD_TOPIC)
        return RGBD_PREVIEW


def get_robot() -> Robot | None:
    global ROBOT_INSTANCE, ROBOT_INIT_ERR, ROBOT_IFACE, ROBOT_LIDAR_CLOUD_TOPIC
    with ROBOT_LOCK:
        if ROBOT_INSTANCE is not None:
            return ROBOT_INSTANCE
        if ROBOT_INIT_ERR is not None:
            return None
        try:
            ROBOT_INSTANCE = Robot(
                iface=ROBOT_IFACE,
                lidar_cloud_topic=ROBOT_LIDAR_CLOUD_TOPIC,
                safety_boot=False,
            )
            return ROBOT_INSTANCE
        except Exception as exc:
            ROBOT_INIT_ERR = str(exc)
            return None


class _BootSequenceController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._confirm = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = "idle"
        self._message = "Boot sequence idle."
        self._error: str | None = None

    def start(self, iface: str, domain_id: int = 0) -> str:
        global ROBOT_INIT_ERR, ROBOT_INSTANCE
        LOGGER.info("Boot start requested iface=%s domain_id=%s", iface, domain_id)
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                LOGGER.info("Boot start ignored because thread is already alive: %s", self._message)
                return self._message
            with ROBOT_LOCK:
                ROBOT_INSTANCE = None
                ROBOT_INIT_ERR = None
            self._confirm.clear()
            self._state = "running"
            self._message = "Starting secure boot sequence."
            self._error = None
            self._thread = threading.Thread(target=self._run, args=(iface, int(domain_id)), daemon=True)
            self._thread.start()
            LOGGER.info("Boot thread started name=%s", self._thread.name)
            return self._message

    def confirm(self) -> str:
        LOGGER.info("Boot confirm requested")
        with self._lock:
            if self._state != "waiting":
                self._confirm.set()
                self._message = f"Dashboard confirmation queued while boot state is {self._state}."
                LOGGER.info("Boot confirm queued state=%s", self._state)
                return self._message
            self._message = "Dashboard confirmation received; continuing boot sequence."
            self._state = "running"
            self._confirm.set()
            LOGGER.info("Boot confirmation accepted")
            return self._message

    def snapshot(self) -> tuple[str, str, str | None, bool]:
        with self._lock:
            running = self._thread is not None and self._thread.is_alive()
            return self._state, self._message, self._error, running

    def _set(self, state: str, message: str, error: str | None = None) -> None:
        if error:
            LOGGER.error("Boot state=%s message=%s error=%s", state, message, error)
        else:
            LOGGER.info("Boot state=%s message=%s", state, message)
        with self._lock:
            self._state = state
            self._message = message
            self._error = error

    def _wait_for_confirm(self, message: str) -> None:
        LOGGER.info("Boot waiting for dashboard confirmation: %s", message)
        self._set("waiting", message)
        while not self._confirm.wait(timeout=0.2):
            pass
        self._confirm.clear()
        LOGGER.info("Boot dashboard confirmation received")

    @staticmethod
    def _is_balanced_stand_state(fsm_id: Any, fsm_mode_value: Any) -> bool:
        try:
            fsm_i = int(fsm_id)
        except Exception:
            return False
        if fsm_i == 501:
            return True
        try:
            mode_i = int(fsm_mode_value)
        except Exception:
            mode_i = -1
        return fsm_i == 200 and mode_i == 0

    def _run(self, iface: str, domain_id: int) -> None:
        global ROBOT_INIT_ERR, ROBOT_INSTANCE
        try:
            LOGGER.info("Boot worker importing SDK helpers")
            from hanger_boot_sequence import _fsm_id, _fsm_mode
            from secure_boot import force_normal_gait
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

            LOGGER.info("Boot worker initializing channel iface=%s domain_id=%s", iface, domain_id)
            ChannelFactoryInitialize(domain_id, iface)
            bot = LocoClient()
            bot.SetTimeout(10.0)
            LOGGER.info("Boot calling LocoClient.Init()")
            bot.Init()
            cur_id, cur_mode = _fsm_id(bot), _fsm_mode(bot)
            LOGGER.info("Boot initial FSM id=%s mode=%s", cur_id, cur_mode)
            if self._is_balanced_stand_state(cur_id, cur_mode):
                LOGGER.info("Boot already balanced; forcing normal gait")
                force_normal_gait(bot)
                with ROBOT_LOCK:
                    ROBOT_INSTANCE = None
                    ROBOT_INIT_ERR = None
                self._set("done", f"Robot is already in balanced stand (FSM {cur_id}, mode {cur_mode}).")
                return

            self._set("running", "Damping robot before stand-up.")
            LOGGER.info("Boot calling Damp()")
            bot.Damp()
            time.sleep(0.1)
            self._set("running", "Switching to stand-up FSM.")
            LOGGER.info("Boot calling SetFsmId(4)")
            bot.SetFsmId(4)
            time.sleep(0.1)

            height = 0.0
            while True:
                height = 0.0
                while height < 0.5:
                    height += 0.02
                    bot.SetStandHeight(height)
                    self._set("running", f"Raising stand height to {height:.2f} m.")
                    time.sleep(0.05)
                    cur_id, mode = _fsm_id(bot), _fsm_mode(bot)
                    LOGGER.info("Boot height=%.2f fsm=%s mode=%s", height, cur_id, mode)
                    if self._is_balanced_stand_state(cur_id, mode) and height > 0.2:
                        break

                cur_id, mode = _fsm_id(bot), _fsm_mode(bot)
                LOGGER.info("Boot sweep complete height=%.2f fsm=%s mode=%s", height, cur_id, mode)
                if self._is_balanced_stand_state(cur_id, mode):
                    break

                self._set("running", "Feet still unloaded; resetting stand height.")
                try:
                    LOGGER.info("Boot calling SetStandHeight(0.0)")
                    bot.SetStandHeight(0.0)
                except Exception:
                    LOGGER.exception("Boot failed to reset stand height")
                self._wait_for_confirm(
                    "Adjust the hanger height, then press Confirm balanced stand in the dashboard."
                )

            self._wait_for_confirm("Robot appears loaded. Press Confirm balanced stand to command BalanceStand.")
            LOGGER.info("Boot calling BalanceStand(0)")
            bot.BalanceStand(0)
            LOGGER.info("Boot calling SetStandHeight(%.2f)", height)
            bot.SetStandHeight(height)
            LOGGER.info("Boot calling Start()")
            bot.Start()
            LOGGER.info("Boot calling force_normal_gait()")
            force_normal_gait(bot)
            with ROBOT_LOCK:
                ROBOT_INSTANCE = None
                ROBOT_INIT_ERR = None
            self._set("done", "Secure boot complete; robot is in balanced stand/start mode.")
        except Exception as exc:
            LOGGER.exception("Boot worker failed")
            self._set("error", f"Hanger boot sequence failed: {exc}", str(exc))


BOOT_SEQUENCE = _BootSequenceController()


def empty_lidar_figure(title: str = "LiDAR stream") -> go.Figure:
    fig = go.Figure()
    fig.update_layout(
        template="plotly_dark",
        title=title,
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        margin={"l": 30, "r": 20, "t": 45, "b": 35},
        height=500,
    )
    return fig


def empty_imu_figure(title: str = "IMU orientation (RPY)") -> go.Figure:
    fig = go.Figure()
    fig.update_layout(
        template="plotly_dark",
        title=title,
        xaxis_title="Time (s, recent window)",
        yaxis_title="Angle (rad)",
        margin={"l": 30, "r": 20, "t": 45, "b": 35},
        height=320,
    )
    return fig


def empty_slam_cloud_figure(title: str = "SLAM 3D LiDAR cloud") -> go.Figure:
    fig = go.Figure()
    fig.update_layout(
        template="plotly_dark",
        title=title,
        scene={
            "xaxis_title": "X (m)",
            "yaxis_title": "Y (m)",
            "zaxis_title": "Z (m)",
            "aspectmode": "data",
        },
        margin={"l": 0, "r": 0, "t": 45, "b": 0},
        height=560,
    )
    return fig


def empty_slam_map_figure(title: str = "SLAM map") -> go.Figure:
    fig = go.Figure()
    fig.update_layout(
        template="plotly_dark",
        title=title,
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        margin={"l": 35, "r": 20, "t": 45, "b": 35},
        height=560,
        clickmode="event+select",
    )
    return fig


def _heightmap_to_arrays(height_map: Any) -> tuple[Any, float, float, float] | None:
    try:
        import numpy as np

        width = int(height_map.width)
        height = int(height_map.height)
        resolution = float(height_map.resolution)
        data = np.asarray(list(height_map.data), dtype=float)
        if width <= 0 or height <= 0 or resolution <= 0 or data.size != width * height:
            return None
        grid = data.reshape((height, width))
        origin_x = -width * resolution / 2.0
        origin_y = -height * resolution / 2.0
        return grid, resolution, origin_x, origin_y
    except Exception:
        return None


def _make_slam_map_figure(height_map: Any, target: tuple[float, float] | None, pose: tuple[float, float, float] | None) -> go.Figure:
    converted = _heightmap_to_arrays(height_map)
    if converted is None:
        return empty_slam_map_figure("SLAM map unavailable")
    grid, resolution, origin_x, origin_y = converted
    height, width = grid.shape
    xs = [origin_x + col * resolution for col in range(width)]
    ys = [origin_y + row * resolution for row in range(height)]
    fig = go.Figure(
        data=[
            go.Heatmap(
                x=xs,
                y=ys,
                z=grid,
                colorscale="Viridis",
                colorbar={"title": "height"},
                name="map",
            )
        ]
    )
    if pose is not None:
        fig.add_trace(
            go.Scatter(
                x=[float(pose[0])],
                y=[float(pose[1])],
                mode="markers",
                marker={"size": 12, "color": "#00d084", "symbol": "triangle-up"},
                name="robot",
            )
        )
    if target is not None:
        fig.add_trace(
            go.Scatter(
                x=[float(target[0])],
                y=[float(target[1])],
                mode="markers",
                marker={"size": 13, "color": "#ff4136", "symbol": "x"},
                name="target",
            )
        )
    fig.update_layout(
        template="plotly_dark",
        title="SLAM map",
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        margin={"l": 35, "r": 20, "t": 45, "b": 35},
        height=560,
        clickmode="event+select",
        yaxis={"scaleanchor": "x", "scaleratio": 1},
    )
    return fig


def _edge_runtime_dir() -> Path:
    script_path = Path(SCRIPT_DIR).resolve()
    candidates = [
        script_path.parents[1] / "edge_runtime",
        script_path.parent / "edge_runtime",
        script_path.parent / "modules" / "edge_runtime",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _load_json_file(path: Path) -> dict[str, Any]:
    try:
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}
    except Exception:
        LOGGER.exception("Failed to load JSON file %s", path)
        return {}


def _load_runtime_defaults(path: Path) -> dict[str, Any]:
    try:
        if not path.exists():
            return {}
        try:
            import yaml  # type: ignore
        except ImportError:
            LOGGER.warning("PyYAML is unavailable; skipping runtime config %s", path)
            return {}
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return data if isinstance(data, dict) else {}
    except Exception:
        LOGGER.exception("Failed to load runtime config %s", path)
        return {}


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    if hasattr(value, "tolist"):
        try:
            return _json_safe(value.tolist())
        except Exception:
            pass
    if hasattr(value, "__dict__"):
        try:
            return _json_safe(vars(value))
        except Exception:
            pass
    return repr(value)


def build_altegro_info() -> dict[str, Any]:
    edge_dir = _edge_runtime_dir()
    fingerprint_path = edge_dir / "device_identity" / "hardware_fingerprint.json"
    config_path = edge_dir / "config" / "runtime_config.yaml"
    fingerprint = _load_json_file(fingerprint_path)
    runtime_defaults = _load_runtime_defaults(config_path)

    robot = get_robot()
    robot_state: dict[str, Any] = {}
    robot_error: str | None = None
    if robot is None:
        robot_error = ROBOT_INIT_ERR or "Robot is not initialized."
    else:
        try:
            robot_state = robot.get_robot_state()
        except Exception as exc:
            LOGGER.exception("Failed to collect robot state for Info tab")
            robot_error = str(exc)

    system_info: dict[str, Any] = {"psutil_available": False}
    try:
        import psutil  # type: ignore

        net_io = psutil.net_io_counters()
        system_info = {
            "psutil_available": True,
            "cpu_usage": psutil.cpu_percent(interval=None),
            "memory_usage": psutil.virtual_memory().percent,
            "disk_usage": psutil.disk_usage(os.path.abspath(os.sep)).percent,
            "network_bytes_sent": net_io.bytes_sent,
            "network_bytes_recv": net_io.bytes_recv,
        }
    except Exception as exc:
        system_info["error"] = str(exc)

    device_id = str(fingerprint.get("device_id") or "G1_Robot_001")
    battery_level = fingerprint.get("battery_level", fingerprint.get("battery_capacity"))
    skill_settings = runtime_defaults.get("skills", {}) if isinstance(runtime_defaults.get("skills"), dict) else {}
    updates = runtime_defaults.get("updates", {}) if isinstance(runtime_defaults.get("updates"), dict) else {}
    network = runtime_defaults.get("network", {}) if isinstance(runtime_defaults.get("network"), dict) else {}
    domain_id = getattr(robot, "domain_id", 0) if robot is not None else 0

    hardware = dict(fingerprint)
    hardware.setdefault("manufacturer", "Unitree")
    hardware.setdefault("model", "G1")
    hardware["network_interface"] = ROBOT_IFACE
    hardware["domain_id"] = domain_id

    software = {
        "runtime": "altegro_client",
        "runtime_version": "0.1.0",
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "os_version": fingerprint.get("os_version"),
        "firmware_version": fingerprint.get("firmware_version"),
        "skills_repository_path": skill_settings.get("repository_path"),
        "max_concurrent_skills": skill_settings.get("max_concurrent_skills"),
        "default_skill_timeout": skill_settings.get("default_timeout"),
        "fallback_version": updates.get("fallback_version"),
        "network_timeout": network.get("timeout"),
    }

    registration = dict(fingerprint)
    registration.setdefault("manufacturer", "Unitree")
    registration.setdefault("model", "G1")
    registration.setdefault("device_id", device_id)
    registration["network_interface"] = ROBOT_IFACE
    registration["domain_id"] = domain_id
    registration["runtime"] = "altegro_client"

    telemetry = {
        "timestamp": int(time.time()),
        "device_id": device_id,
        "runtime": "altegro_client",
        "system": system_info,
        "robot": {
            "fsm": robot_state.get("fsm"),
            "mode": robot_state.get("mode"),
            "gait": robot_state.get("gait"),
            "body_height": robot_state.get("body_height"),
            "position": robot_state.get("position"),
            "velocity": robot_state.get("velocity"),
            "yaw": robot_state.get("yaw"),
            "imu": robot_state.get("imu"),
            "odom_pose": robot_state.get("odom_pose"),
            "slam_pose": robot_state.get("slam_pose"),
            "is_moving": robot_state.get("is_moving"),
            "sensor_stale": robot_state.get("sensor_stale"),
            "sensor_timestamps": robot_state.get("sensor_timestamps"),
            "slam_is_running": robot_state.get("slam_is_running"),
            "queued_path_points": robot_state.get("queued_path_points"),
            "joint_count": robot_state.get("joint_count"),
            "battery": battery_level,
        },
    }

    heartbeat = {
        "status": "active" if robot_error is None else "unavailable",
        "timestamp": int(time.time()),
        "robot_connected": robot_error is None,
        "fsm": robot_state.get("fsm"),
        "mode": robot_state.get("mode"),
        "is_moving": robot_state.get("is_moving"),
    }

    return _json_safe(
        {
            "source": "altegro_client compatible dashboard snapshot",
            "paths": {
                "edge_runtime": str(edge_dir),
                "hardware_fingerprint": str(fingerprint_path),
                "runtime_config": str(config_path),
            },
            "robot_error": robot_error,
            "device_registration": registration,
            "telemetry": telemetry,
            "heartbeat": heartbeat,
            "skill_inventory": {"hardware": hardware, "software": software},
            "raw_robot_state": robot_state,
        }
    )


app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.title = "Robot Control"
TAB_STYLE = {
    "color": "#1f2933",
    "backgroundColor": "#eef2f6",
    "border": "1px solid #b8c2cc",
    "fontWeight": 600,
}
ACTIVE_TAB_STYLE = {
    "color": "#0b2545",
    "backgroundColor": "#ffffff",
    "border": "1px solid #b8c2cc",
    "borderBottomColor": "#ffffff",
    "fontWeight": 700,
}
TAB_LABEL_STYLE = {"color": "#1f2933"}
ACTIVE_TAB_LABEL_STYLE = {"color": "#0b2545"}
app.index_string = """
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            .nav-tabs { border-bottom: 1px solid #b8c2cc; }
            .nav-tabs .nav-link {
                color: #1f2933 !important;
                background: #eef2f6;
                border: 1px solid #b8c2cc !important;
                margin-right: 4px;
                font-weight: 600;
            }
            .nav-tabs .nav-link.active {
                color: #0b2545 !important;
                background: #ffffff !important;
                border-bottom-color: #ffffff !important;
            }
            .joystick-pad {
                width: min(320px, 80vw);
                aspect-ratio: 1;
                border: 1px solid #9aa6b2;
                background: #eef2f6;
                position: relative;
                touch-action: none;
                user-select: none;
                margin-top: 8px;
            }
            .joystick-pad::before,
            .joystick-pad::after {
                content: "";
                position: absolute;
                background: #b8c2cc;
            }
            .joystick-pad::before { left: 50%; top: 8%; width: 1px; height: 84%; }
            .joystick-pad::after { left: 8%; top: 50%; width: 84%; height: 1px; }
            .joystick-knob {
                width: 54px;
                height: 54px;
                border-radius: 50%;
                background: #0d6efd;
                position: absolute;
                left: 50%;
                top: 50%;
                transform: translate(-50%, -50%);
                z-index: 1;
            }
            #logs-content,
            #info-content {
                white-space: pre-wrap;
                font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
                font-size: 12px;
                max-height: 70vh;
                overflow: auto;
                background: #0f172a;
                color: #e5e7eb;
                padding: 12px;
            }
            .nav-tabs .nav-item:nth-child(1) .nav-link:empty::before { content: "Control"; }
            .nav-tabs .nav-item:nth-child(2) .nav-link:empty::before { content: "LowLevel"; }
            .nav-tabs .nav-item:nth-child(3) .nav-link:empty::before { content: "Locomotion"; }
            .nav-tabs .nav-item:nth-child(4) .nav-link:empty::before { content: "SLAM"; }
            .nav-tabs .nav-item:nth-child(5) .nav-link:empty::before { content: "Info"; }
            .nav-tabs .nav-item:nth-child(6) .nav-link:empty::before { content: "Skills"; }
            .nav-tabs .nav-item:nth-child(7) .nav-link:empty::before { content: "Logs"; }
            .nav-tabs .nav-item:nth-child(8) .nav-link:empty::before { content: "Sensors"; }
            .nav-tabs .nav-item:nth-child(9) .nav-link:empty::before { content: "Settings"; }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
            <script>
            (function () {
                window.__dashJoystick = window.__dashJoystick || {vx: 0, vy: 0, vyaw: 0, seq: 0};
                function updateJoystick(values) {
                    window.__dashJoystick = Object.assign({}, window.__dashJoystick, values);
                    window.__dashJoystick.seq = (window.__dashJoystick.seq || 0) + 1;
                }
                function setDashValue(id, value) {
                    var el = document.getElementById(id);
                    if (!el) return;
                    var text = Number(value).toFixed(3);
                    var setter = Object.getOwnPropertyDescriptor(window.HTMLInputElement.prototype, "value").set;
                    setter.call(el, text);
                    el.dispatchEvent(new Event("input", {bubbles: true}));
                    el.dispatchEvent(new Event("change", {bubbles: true}));
                }
                function setReadout(id, text) {
                    var el = document.getElementById(id);
                    if (el) el.textContent = text;
                }
                function attachJoystick(id) {
                    var pad = document.getElementById(id);
                    if (!pad || pad.dataset.joystickReady === "1") return;
                    pad.dataset.joystickReady = "1";
                    var knob = pad.querySelector(".joystick-knob");
                    var active = false;
                    function apply(clientX, clientY) {
                        var rect = pad.getBoundingClientRect();
                        var x = Math.max(-1, Math.min(1, ((clientX - rect.left) / rect.width - 0.5) * 2));
                        var y = Math.max(-1, Math.min(1, ((clientY - rect.top) / rect.height - 0.5) * 2));
                        if (knob) {
                            knob.style.left = ((x + 1) * 50) + "%";
                            knob.style.top = ((y + 1) * 50) + "%";
                        }
                        if (id === "linear-joystick") {
                            var vx = -y * 0.6;
                            var vy = x * 0.4;
                            updateJoystick({vx: vx, vy: vy});
                            setDashValue("nav-vx", vx);
                            setDashValue("nav-vy", vy);
                            setReadout("linear-joystick-readout", "vx " + vx.toFixed(2) + " m/s | vy " + vy.toFixed(2) + " m/s");
                        } else {
                            var vyaw = x * 1.0;
                            updateJoystick({vyaw: vyaw});
                            setDashValue("nav-vyaw", vyaw);
                            setReadout("angular-joystick-readout", "vyaw " + vyaw.toFixed(2) + " rad/s");
                        }
                    }
                    function reset() {
                        active = false;
                        if (knob) {
                            knob.style.left = "50%";
                            knob.style.top = "50%";
                        }
                        if (id === "linear-joystick") {
                            updateJoystick({vx: 0, vy: 0});
                            setDashValue("nav-vx", 0);
                            setDashValue("nav-vy", 0);
                            setReadout("linear-joystick-readout", "vx 0.00 m/s | vy 0.00 m/s");
                        } else {
                            updateJoystick({vyaw: 0});
                            setDashValue("nav-vyaw", 0);
                            setReadout("angular-joystick-readout", "vyaw 0.00 rad/s");
                        }
                    }
                    pad.addEventListener("pointerdown", function (ev) {
                        active = true;
                        pad.setPointerCapture(ev.pointerId);
                        apply(ev.clientX, ev.clientY);
                    });
                    pad.addEventListener("pointermove", function (ev) {
                        if (active) apply(ev.clientX, ev.clientY);
                    });
                    pad.addEventListener("pointerup", reset);
                    pad.addEventListener("pointercancel", reset);
                    pad.addEventListener("lostpointercapture", reset);
                }
                setInterval(function () {
                    attachJoystick("linear-joystick");
                    attachJoystick("angular-joystick");
                }, 500);
            })();
            </script>
        </footer>
    </body>
</html>
"""

app.layout = dbc.Container(
    [
        html.H3("Robot Control Dashboard", className="mt-3 mb-3"),
        dbc.Alert(id="status-alert", color="secondary", children="Ready", className="mb-3"),
        dbc.Tabs(
            [
                dbc.Tab(
                    label="Control",
                    tab_id="control",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(dbc.Button("Damp", id="btn-damp", color="warning", className="w-100"), md=4),
                                dbc.Col(dbc.Button("Zero Torque", id="btn-zero", color="danger", className="w-100"), md=4),
                                dbc.Col(dbc.Button("Stop", id="btn-stop", color="secondary", className="w-100"), md=4),
                            ],
                            className="g-2 mt-2",
                        ),
                        dbc.Row(
                            [
                                dbc.Col(
                                    dbc.Button(
                                        "Start secure boot sequence",
                                        id="btn-hanged-boot",
                                        color="primary",
                                        className="w-100 mt-2",
                                    ),
                                    md=6,
                                ),
                                dbc.Col(
                                    dbc.Button(
                                        "Confirm balanced stand",
                                        id="btn-boot-enter",
                                        color="success",
                                        className="w-100 mt-2",
                                    ),
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="boot-status", className="mt-2"),
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("Gait Type", className="mt-3 mb-1"),
                                        dbc.RadioItems(
                                            id="gait-toggle",
                                            options=[
                                                {"label": "Walk", "value": "walk"},
                                                {"label": "Run", "value": "run"},
                                            ],
                                            value="walk",
                                            inline=True,
                                        ),
                                    ],
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
                        dbc.InputGroup(
                            [
                                dbc.Input(id="say-text", placeholder="Type text to speak", type="text"),
                                dbc.Button("Say", id="btn-say", color="success"),
                            ],
                            className="mt-3",
                        ),
                        html.Div(id="say-result", className="mt-2"),
                        dbc.Row(
                            [
                                dbc.Col([html.Div("Headlight color", className="mt-3 mb-1"), dbc.Input(id="headlight-color", type="text", value="white")], md=3),
                                dbc.Col([html.Div("Intensity", className="mt-3 mb-1"), dbc.Input(id="headlight-intensity", type="number", value=80, min=0, max=100, step=1)], md=3),
                                dbc.Col([html.Div("Duration (s)", className="mt-3 mb-1"), dbc.Input(id="headlight-duration", type="number", value=2.0, min=0, step=0.1)], md=3),
                                dbc.Col(dbc.Button("Apply Headlight", id="btn-headlight", color="primary", className="w-100 mt-4"), md=3),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="headlight-result", className="mt-2"),
                        html.Div(id="control-result", className="mt-3"),
                    ],
                ),
                dbc.Tab(
                    label="LowLevel",
                    tab_id="lowlevel",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("Joint", className="mt-3 mb-1"),
                                        dcc.Dropdown(
                                            id="lowlevel-joint",
                                            options=LOWLEVEL_JOINT_OPTIONS,
                                            value=LOWLEVEL_JOINT_SPECS[0].motor_index,
                                            clearable=False,
                                        ),
                                    ],
                                    md=6,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("Max increment (rad)", className="mt-3 mb-1"),
                                        dbc.Input(id="lowlevel-max-inc", type="number", value=0.01, step=0.001, min=0.0005),
                                    ],
                                    md=3,
                                ),
                                dbc.Col(
                                    dbc.Button("Enable LowLevel", id="btn-lowlevel-toggle", color="danger", className="w-100 mt-4"),
                                    md=3,
                                ),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="lowlevel-selected", className="mt-2"),
                        html.Div("Target pose (rad)", className="mt-3 mb-1"),
                        dcc.Slider(
                            id="lowlevel-target",
                            min=LOWLEVEL_JOINT_SPECS[0].limit_min,
                            max=LOWLEVEL_JOINT_SPECS[0].limit_max,
                            step=0.001,
                            value=0.0,
                            tooltip={"placement": "bottom", "always_visible": True},
                            marks=None,
                        ),
                        dbc.Row(
                            [
                                dbc.Col([html.Div("dq", className="mt-3 mb-1"), dbc.Input(id="lowlevel-dq", type="number", value=0.0, step=0.01)], md=3),
                                dbc.Col([html.Div("tau", className="mt-3 mb-1"), dbc.Input(id="lowlevel-tau", type="number", value=0.0, step=0.01)], md=3),
                                dbc.Col([html.Div("pk", className="mt-3 mb-1"), dbc.Input(id="lowlevel-pk", type="number", value=30.0, step=0.5)], md=3),
                                dbc.Col([html.Div("pd", className="mt-3 mb-1"), dbc.Input(id="lowlevel-pd", type="number", value=1.5, step=0.1)], md=3),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="lowlevel-status", className="mt-3"),
                        html.Hr(),
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("Hand", className="mt-3 mb-1"),
                                        dcc.Dropdown(
                                            id="grip-hand",
                                            options=[
                                                {"label": "Right", "value": "right"},
                                                {"label": "Left", "value": "left"},
                                            ],
                                            value="right",
                                            clearable=False,
                                        ),
                                    ],
                                    md=4,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("Grip max increment (%)", className="mt-3 mb-1"),
                                        dbc.Input(id="grip-max-inc", type="number", value=2.0, step=0.5, min=0.1),
                                    ],
                                    md=3,
                                ),
                            ],
                            className="g-2",
                        ),
                        html.Div("Grip open to close (%)", className="mt-3 mb-1"),
                        dcc.Slider(
                            id="grip-target",
                            min=0,
                            max=100,
                            step=1,
                            value=100,
                            marks={0: "Open", 100: "Closed"},
                            tooltip={"placement": "bottom", "always_visible": True},
                        ),
                        html.Div(id="grip-status", className="mt-3"),
                    ],
                ),
                dbc.Tab(
                    label="Locomotion",
                    tab_id="locomotion",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("Left joystick linear velocity", className="mt-3 mb-2"),
                                        html.Div(
                                            [html.Div(className="joystick-knob")],
                                            id="linear-joystick",
                                            className="joystick-pad",
                                        ),
                                        html.Div("vx 0.00 m/s | vy 0.00 m/s", id="linear-joystick-readout", className="mt-2"),
                                    ],
                                    md=6,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("Right joystick angular velocity", className="mt-3 mb-2"),
                                        html.Div(
                                            [html.Div(className="joystick-knob")],
                                            id="angular-joystick",
                                            className="joystick-pad",
                                        ),
                                        html.Div("vyaw 0.00 rad/s", id="angular-joystick-readout", className="mt-2"),
                                    ],
                                    md=6,
                                ),
                            ],
                            className="g-3",
                        ),
                        dcc.Input(id="nav-vx", type="hidden", value=0.0),
                        dcc.Input(id="nav-vy", type="hidden", value=0.0),
                        dcc.Input(id="nav-vyaw", type="hidden", value=0.0),
                        dbc.Row(
                            [
                                dbc.Col(
                                    dbc.Button("Handshake", id="btn-handshake", color="primary", className="w-100 mt-3"),
                                    md=6,
                                ),
                                dbc.Col(
                                    dbc.Button("Stop Move", id="btn-nav-stop", color="secondary", className="w-100 mt-3"),
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="nav-result", className="mt-3"),
                    ],
                ),
                dbc.Tab(
                    label="SLAM",
                    tab_id="slam",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(dbc.Button("Start Mapping", id="btn-slam-start", color="primary", className="w-100 mt-3"), md=3),
                                dbc.Col(dbc.Button("Finish Mapping", id="btn-slam-stop", color="secondary", className="w-100 mt-3"), md=3),
                                dbc.Col(dbc.Button("Navigate Target", id="btn-slam-nav", color="success", className="w-100 mt-3"), md=3),
                                dbc.Col(dbc.Button("Clear Target", id="btn-slam-clear-target", color="warning", className="w-100 mt-3"), md=3),
                            ],
                            className="g-2",
                        ),
                        dbc.Row(
                            [
                                dbc.Col([html.Div("Target X (m)", className="mt-3 mb-1"), dbc.Input(id="slam-target-x", type="number", value=0.0, step=0.05)], md=4),
                                dbc.Col([html.Div("Target Y (m)", className="mt-3 mb-1"), dbc.Input(id="slam-target-y", type="number", value=0.0, step=0.05)], md=4),
                                dbc.Col([html.Div("Target yaw (rad)", className="mt-3 mb-1"), dbc.Input(id="slam-target-yaw", type="number", value=0.0, step=0.05)], md=4),
                            ],
                            className="g-2",
                        ),
                        html.Div(id="slam-status", className="mt-3"),
                        dbc.Row(
                            [
                                dbc.Col([dcc.Graph(id="slam-cloud-graph", figure=empty_slam_cloud_figure())], md=6),
                                dbc.Col([dcc.Graph(id="slam-map-graph", figure=empty_slam_map_figure())], md=6),
                            ],
                            className="g-2",
                        ),
                    ],
                ),
                dbc.Tab(
                    label="Info",
                    tab_id="info",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Button("Refresh Info", id="btn-info-refresh", color="primary", className="mt-3 mb-2"),
                        html.Pre(id="info-content", children=""),
                    ],
                ),
                dbc.Tab(
                    label="Skills",
                    tab_id="skills",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Card(
                            dbc.CardBody(
                                [
                                    html.H4("ALTEGRO store", className="card-title"),
                                    html.Div("Skill marketplace placeholder.", className="text-muted"),
                                ]
                            ),
                            className="mt-3",
                        ),
                    ],
                ),
                dbc.Tab(
                    label="Logs",
                    tab_id="logs",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        html.Div(f"Log file: {LOG_PATH}", className="mt-3 mb-2"),
                        html.Pre(id="logs-content", children=""),
                    ],
                ),
                dbc.Tab(
                    label="Sensors",
                    tab_id="sensors",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("RGB camera feed", className="mt-3 mb-2"),
                                        html.Img(
                                            id="rgb-feed",
                                            style={
                                                "width": "100%",
                                                "border": "1px solid #444",
                                                "borderRadius": "8px",
                                            },
                                        ),
                                        html.Div(id="rgb-status", className="mb-3"),
                                    ],
                                    md=6,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("Depth camera feed (RealSense, PLASMA)", className="mt-3 mb-2"),
                                        html.Img(
                                            id="depth-feed",
                                            style={
                                                "width": "100%",
                                                "border": "1px solid #444",
                                                "borderRadius": "8px",
                                            },
                                        ),
                                        html.Div(id="depth-status", className="mb-3"),
                                    ],
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
                        html.Hr(),
                        html.Div("LiDAR stream (XY scatter from PointCloud2)", className="mt-3 mb-2"),
                        dcc.Graph(id="lidar-graph", figure=empty_lidar_figure()),
                        html.Div(id="lidar-status", className="mb-3"),
                        html.Hr(),
                        html.Div("IMU orientation (roll/pitch/yaw)", className="mt-3 mb-2"),
                        dcc.Graph(id="imu-graph", figure=empty_imu_figure()),
                        html.Div(id="imu-status", className="mb-3"),
                    ],
                ),
                dbc.Tab(
                    label="Settings",
                    tab_id="settings",
                    tab_style=TAB_STYLE,
                    active_tab_style=ACTIVE_TAB_STYLE,
                    label_style=TAB_LABEL_STYLE,
                    active_label_style=ACTIVE_TAB_LABEL_STYLE,
                    children=[
                        html.Div("Current iface", className="mt-3 mb-1"),
                        dbc.Badge(ROBOT_IFACE, id="iface-current", color="secondary"),
                        dbc.InputGroup(
                            [
                                dbc.Input(id="iface-input", placeholder="e.g. eth0, enp3s0, wlan0", type="text", value=ROBOT_IFACE),
                                dbc.Button("Apply iface", id="btn-apply-iface", color="primary"),
                            ],
                            className="mt-3",
                        ),
                        html.Div(id="settings-result", className="mt-3"),
                    ],
                ),
            ],
            active_tab="control",
            className="mb-3",
        ),
        dcc.Interval(id="lidar-interval", interval=1000, n_intervals=0),
        dcc.Interval(id="rgb-interval", interval=500, n_intervals=0),
        dcc.Interval(id="depth-interval", interval=500, n_intervals=0),
        dcc.Interval(id="boot-interval", interval=500, n_intervals=0),
        dcc.Interval(id="lowlevel-interval", interval=500, n_intervals=0),
        dcc.Interval(id="grip-interval", interval=500, n_intervals=0),
        dcc.Interval(id="logs-interval", interval=1000, n_intervals=0),
        dcc.Interval(id="slam-interval", interval=1000, n_intervals=0),
        dcc.Interval(id="info-interval", interval=2000, n_intervals=0),
        dcc.Store(id="slam-target-store", data=None),
        dcc.Interval(id="locomotion-interval", interval=100, n_intervals=0),
        dcc.Store(id="nav-command", data={"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "seq": 0}),
        dcc.Store(id="lowlevel-enabled", data=False),
    ],
    fluid=True,
)


app.clientside_callback(
    """
    function(n) {
        const v = window.__dashJoystick || {vx: 0, vy: 0, vyaw: 0, seq: 0};
        return {
            vx: Number(v.vx || 0),
            vy: Number(v.vy || 0),
            vyaw: Number(v.vyaw || 0),
            seq: Number(v.seq || 0)
        };
    }
    """,
    Output("nav-command", "data"),
    Input("locomotion-interval", "n_intervals"),
)


@app.callback(
    Output("boot-status", "children"),
    Input("boot-interval", "n_intervals"),
    Input("btn-hanged-boot", "n_clicks"),
    Input("btn-boot-enter", "n_clicks"),
)
def update_boot_status(
    _tick: int,
    _hanged_boot: int | None,
    _boot_enter_btn: int | None,
) -> str:
    trigger = dash.ctx.triggered_id
    if trigger in {"btn-hanged-boot", "btn-boot-enter"}:
        LOGGER.info(
            "Boot callback triggered=%s hanged_clicks=%s confirm_clicks=%s iface=%s",
            trigger,
            _hanged_boot,
            _boot_enter_btn,
            ROBOT_IFACE,
        )
    if trigger == "btn-hanged-boot":
        BOOT_SEQUENCE.start(ROBOT_IFACE)
    elif trigger == "btn-boot-enter":
        BOOT_SEQUENCE.confirm()

    state, message, err, running = BOOT_SEQUENCE.snapshot()
    if trigger in {"btn-hanged-boot", "btn-boot-enter"}:
        LOGGER.info("Boot callback snapshot state=%s running=%s err=%s message=%s", state, running, err, message)
    if err:
        return f"Boot: error | {message} | log={LOG_PATH}"
    if running or state != "idle":
        return f"Boot: {state} | {message} | log={LOG_PATH}"
    return ""


@app.callback(
    Output("status-alert", "children"),
    Output("status-alert", "color"),
    Output("control-result", "children"),
    Input("btn-damp", "n_clicks"),
    Input("btn-zero", "n_clicks"),
    Input("btn-stop", "n_clicks"),
    Input("gait-toggle", "value"),
    prevent_initial_call=True,
)
def on_control(
    _damp: int | None,
    _zero: int | None,
    _stop: int | None,
    gait_value: str,
) -> tuple[str, str, str]:
    trigger = dash.ctx.triggered_id

    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}", "danger", "No robot instance available."

    try:
        if trigger == "btn-damp":
            robot.fsm_1_damp()
            return "Damp command sent.", "warning", "FSM set to damp mode."
        if trigger == "btn-zero":
            robot.fsm_0_zt()
            return "Zero torque command sent.", "danger", "FSM set to zero torque."
        if trigger == "btn-stop":
            robot.stop()
            return "Stop command sent.", "secondary", "Robot motion stopped."
        if trigger == "gait-toggle":
            if gait_value == "run":
                robot.set_gait_type(1)
                return "Gait switched to run.", "primary", "Set gait type to run (1)."
            robot.set_gait_type(0)
            return "Gait switched to walk.", "primary", "Set gait type to walk (0)."
    except Exception as exc:
        LOGGER.exception("Control command failed trigger=%s", trigger)
        return f"Command failed: {exc}", "danger", str(exc)

    return "Ready", "secondary", "No action taken."


@app.callback(
    Output("nav-result", "children"),
    Input("btn-handshake", "n_clicks"),
    Input("btn-nav-stop", "n_clicks"),
    Input("nav-command", "data"),
    prevent_initial_call=True,
)
def on_navigation(
    _handshake_clicks: int | None,
    _stop_clicks: int | None,
    command: dict[str, Any] | None,
) -> str:
    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}"

    trigger = dash.ctx.triggered_id
    try:
        if trigger == "btn-handshake":
            loco = getattr(robot, "_client", None)
            if loco is None or not hasattr(loco, "ShakeHand"):
                raise AttributeError("Current locomotion client does not support ShakeHand().")
            rc = getattr(loco, "ShakeHand")()
            return f"Handshake command sent, rc={rc}."
        if trigger == "btn-nav-stop":
            robot.stop()
            return "stop() sent."
        if trigger == "nav-command":
            command = command or {}
            cmd_vx = float(command.get("vx") or 0.0)
            cmd_vy = float(command.get("vy") or 0.0)
            cmd_vyaw = float(command.get("vyaw") or 0.0)
            if hasattr(robot, "move"):
                rc = int(getattr(robot, "move")(cmd_vx, cmd_vy, cmd_vyaw))
            elif hasattr(robot, "loco_move"):
                rc = int(robot.loco_move(cmd_vx, cmd_vy, cmd_vyaw))
            else:
                rc = int(robot.walk(cmd_vx, cmd_vy, cmd_vyaw))
            return f"move(vx={cmd_vx:.3f}, vy={cmd_vy:.3f}, vyaw={cmd_vyaw:.3f}) sent, rc={rc}."
    except Exception as exc:
        LOGGER.exception("Locomotion command failed trigger=%s command=%s", trigger, command)
        return f"Locomotion command failed: {exc}"

    return "No locomotion action taken."


def read_warning_error_logs(max_lines: int = 200) -> str:
    try:
        with open(LOG_PATH, "r", encoding="utf-8", errors="replace") as fh:
            lines = fh.readlines()
    except Exception as exc:
        return f"Unable to read log file {LOG_PATH}: {exc}"
    selected = [
        line.rstrip()
        for line in lines
        if " WARNING " in line or " ERROR " in line or " CRITICAL " in line or "Traceback " in line
    ]
    if not selected:
        return f"No warnings or errors in {LOG_PATH}."
    return "\n".join(selected[-max_lines:])


@app.callback(
    Output("logs-content", "children"),
    Input("logs-interval", "n_intervals"),
)
def update_logs(_tick: int) -> str:
    return read_warning_error_logs()


@app.callback(
    Output("info-content", "children"),
    Input("info-interval", "n_intervals"),
    Input("btn-info-refresh", "n_clicks"),
)
def update_info(_tick: int, _refresh: int | None) -> str:
    try:
        return json.dumps(build_altegro_info(), indent=2, sort_keys=True)
    except Exception as exc:
        LOGGER.exception("Info tab update failed")
        return f"Info update failed: {exc}"


def _make_slam_cloud_from_points(points: list[tuple[float, float, float]], title: str) -> go.Figure:
    if not points:
        return empty_slam_cloud_figure(title)
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    zs = [float(p[2]) for p in points]
    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=xs,
                y=ys,
                z=zs,
                mode="markers",
                marker={"size": 2, "color": zs, "colorscale": "Turbo", "opacity": 0.85},
                name="LiDAR",
            )
        ]
    )
    fig.update_layout(
        template="plotly_dark",
        title=title,
        scene={
            "xaxis_title": "X (m)",
            "yaxis_title": "Y (m)",
            "zaxis_title": "Z (m)",
            "aspectmode": "data",
        },
        margin={"l": 0, "r": 0, "t": 45, "b": 0},
        height=560,
    )
    return fig


@app.callback(
    Output("slam-cloud-graph", "figure"),
    Output("slam-map-graph", "figure"),
    Output("slam-status", "children"),
    Output("slam-target-x", "value"),
    Output("slam-target-y", "value"),
    Output("slam-target-store", "data"),
    Input("slam-interval", "n_intervals"),
    Input("btn-slam-start", "n_clicks"),
    Input("btn-slam-stop", "n_clicks"),
    Input("btn-slam-nav", "n_clicks"),
    Input("btn-slam-clear-target", "n_clicks"),
    Input("slam-map-graph", "clickData"),
    State("slam-target-x", "value"),
    State("slam-target-y", "value"),
    State("slam-target-yaw", "value"),
    State("slam-target-store", "data"),
)
def update_slam_tab(
    _tick: int,
    _start: int | None,
    _stop: int | None,
    _nav: int | None,
    _clear: int | None,
    click_data: dict[str, Any] | None,
    target_x: float | None,
    target_y: float | None,
    target_yaw: float | None,
    target_store: dict[str, Any] | None,
) -> tuple[go.Figure, go.Figure, str, Any, Any, Any]:
    robot = get_robot()
    if robot is None:
        msg = f"Robot init failed: {ROBOT_INIT_ERR}"
        return empty_slam_cloud_figure("SLAM unavailable"), empty_slam_map_figure("SLAM map unavailable"), msg, dash.no_update, dash.no_update, dash.no_update

    trigger = dash.ctx.triggered_id
    status_parts: list[str] = []
    out_x: Any = dash.no_update
    out_y: Any = dash.no_update
    out_store: Any = dash.no_update
    selected_target: tuple[float, float] | None = None

    if target_store and "x" in target_store and "y" in target_store:
        selected_target = (float(target_store["x"]), float(target_store["y"]))

    try:
        if trigger == "slam-map-graph" and click_data and click_data.get("points"):
            pt = click_data["points"][0]
            selected_target = (float(pt["x"]), float(pt["y"]))
            out_x, out_y = selected_target
            out_store = {"x": selected_target[0], "y": selected_target[1]}
            status_parts.append(f"Target selected: x={out_x:.3f}, y={out_y:.3f}")
        elif trigger == "btn-slam-clear-target":
            selected_target = None
            out_x, out_y = 0.0, 0.0
            out_store = None
            status_parts.append("Target cleared.")
        elif trigger == "btn-slam-start":
            rc = robot.start_slam()
            status_parts.append(f"start_slam rc={rc}")
        elif trigger == "btn-slam-stop":
            rc = robot.stop_slam()
            status_parts.append(f"stop_slam rc={rc}")
        elif trigger == "btn-slam-nav":
            if selected_target is None and target_x is not None and target_y is not None:
                selected_target = (float(target_x), float(target_y))
                out_store = {"x": selected_target[0], "y": selected_target[1]}
            if selected_target is None:
                status_parts.append("No target selected.")
            elif hasattr(robot, "slam_nav_pose"):
                rc = int(robot.slam_nav_pose(float(selected_target[0]), float(selected_target[1]), float(target_yaw or 0.0), obs_avoid=False))
                status_parts.append(f"slam_nav_pose rc={rc}")
            elif hasattr(robot, "_run_pose_nav"):
                rc = int(getattr(robot, "_run_pose_nav")(float(selected_target[0]), float(selected_target[1]), float(target_yaw or 0.0)))
                status_parts.append(f"pose_nav rc={rc}")
            else:
                status_parts.append("Robot wrapper has no SLAM navigation method.")
    except Exception as exc:
        LOGGER.exception("SLAM tab command failed trigger=%s", trigger)
        status_parts.append(f"SLAM command failed: {exc}")

    points: list[tuple[float, float, float]] = []
    try:
        points = robot.get_lidar_points(max_points=12000)
    except Exception as exc:
        LOGGER.exception("SLAM LiDAR read failed")
        status_parts.append(f"LiDAR read failed: {exc}")
    if not points:
        try:
            live = get_livox_preview()
            live.start()
            xyz, _ts, live_err = live.snapshot()
            if xyz is not None:
                import numpy as np

                arr = np.asarray(xyz, dtype=float)
                if arr.ndim == 2 and arr.shape[1] >= 3:
                    if arr.shape[0] > 12000:
                        arr = arr[:: int(arr.shape[0] / 12000) + 1]
                    points = [(float(row[0]), float(row[1]), float(row[2])) for row in arr[:, :3]]
            elif live_err:
                status_parts.append(f"Live LiDAR error: {live_err}")
        except Exception as exc:
            LOGGER.exception("SLAM live LiDAR fallback failed")
            status_parts.append(f"Live LiDAR fallback failed: {exc}")

    cloud_fig = _make_slam_cloud_from_points(points, f"SLAM 3D LiDAR cloud ({len(points)} pts)")

    pose = None
    try:
        pose = robot.get_slam_pose(timeout_s=0.05)
    except Exception:
        pose = None
    if pose is None:
        try:
            pos = robot.get_position()
            if pos is not None:
                pose = (float(pos[0]), float(pos[1]), 0.0)
        except Exception:
            pose = None

    try:
        height_map = robot.get_lidar_map()
        map_fig = _make_slam_map_figure(height_map, selected_target, pose) if height_map is not None else empty_slam_map_figure("SLAM map unavailable")
    except Exception as exc:
        LOGGER.exception("SLAM map render failed")
        map_fig = empty_slam_map_figure("SLAM map error")
        status_parts.append(f"Map render failed: {exc}")

    ts = {}
    try:
        ts = robot.get_sensor_timestamps()
    except Exception:
        ts = {}
    map_age = max(0.0, time.time() - ts.get("lidar_map", 0.0)) if ts.get("lidar_map", 0.0) else -1.0
    cloud_age = max(0.0, time.time() - ts.get("lidar_cloud", 0.0)) if ts.get("lidar_cloud", 0.0) else -1.0
    pose_txt = "pose unavailable" if pose is None else f"pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})"
    target_txt = "target unset" if selected_target is None else f"target=({selected_target[0]:.2f}, {selected_target[1]:.2f}, yaw={float(target_yaw or 0.0):.2f})"
    status = " | ".join([*status_parts, pose_txt, target_txt, f"map_age={map_age:.2f}s", f"cloud_age={cloud_age:.2f}s"])
    return cloud_fig, map_fig, status, out_x, out_y, out_store


@app.callback(
    Output("lowlevel-target", "min"),
    Output("lowlevel-target", "max"),
    Output("lowlevel-selected", "children"),
    Input("lowlevel-joint", "value"),
)
def on_lowlevel_joint_selected(joint_index: int | None) -> tuple[float, float, str]:
    spec = LOWLEVEL_JOINT_BY_INDEX.get(int(joint_index or 0), LOWLEVEL_JOINT_SPECS[0])
    current = LOWLEVEL_CONTROLLER.current_position(spec.motor_index)
    current_text = "current unavailable" if current is None else f"current {current:.3f} rad"
    return (
        float(spec.limit_min),
        float(spec.limit_max),
        f"{spec.label} | limits [{spec.limit_min:.3f}, {spec.limit_max:.3f}] rad | {current_text}",
    )


@app.callback(
    Output("lowlevel-status", "children"),
    Input("lowlevel-interval", "n_intervals"),
    Input("lowlevel-target", "value"),
    State("lowlevel-joint", "value"),
    State("lowlevel-max-inc", "value"),
    State("lowlevel-dq", "value"),
    State("lowlevel-tau", "value"),
    State("lowlevel-pk", "value"),
    State("lowlevel-pd", "value"),
    State("lowlevel-enabled", "data"),
)
def on_lowlevel_target(
    _tick: int,
    target: float | None,
    joint_index: int | None,
    max_increment: float | None,
    dq: float | None,
    tau: float | None,
    pk: float | None,
    pd: float | None,
    lowlevel_enabled: bool,
) -> str:
    if dash.ctx.triggered_id == "lowlevel-target":
        if not bool(lowlevel_enabled):
            LOGGER.warning("LowLevel target ignored because LowLevel is disabled.")
        else:
            LOWLEVEL_CONTROLLER.start_move(
                joint_index=int(joint_index or 0),
                target=float(target or 0.0),
                max_increment=float(max_increment or 0.01),
                dq=float(dq or 0.0),
                tau=float(tau or 0.0),
                pk=float(pk if pk is not None else 30.0),
                pd=float(pd if pd is not None else 1.5),
                iface=ROBOT_IFACE,
            )
    status, err, state_ts, running = LOWLEVEL_CONTROLLER.snapshot()
    age = "state n/a" if state_ts <= 0.0 else f"state age {max(0.0, time.time() - state_ts):.2f}s"
    prefix = "running" if running else ("error" if err else "idle")
    enabled_text = "enabled" if bool(lowlevel_enabled) else "disabled"
    return f"LowLevel: {enabled_text} | {prefix} | {status} | {age}"


@app.callback(
    Output("lowlevel-enabled", "data"),
    Output("btn-lowlevel-toggle", "children"),
    Output("btn-lowlevel-toggle", "color"),
    Input("btn-lowlevel-toggle", "n_clicks"),
    State("lowlevel-enabled", "data"),
    prevent_initial_call=True,
)
def toggle_lowlevel(_clicks: int | None, enabled: bool) -> tuple[bool, str, str]:
    new_enabled = not bool(enabled)
    LOGGER.warning("LowLevel control toggled %s", "enabled" if new_enabled else "disabled")
    return new_enabled, ("Disable LowLevel" if new_enabled else "Enable LowLevel"), ("success" if new_enabled else "danger")


@app.callback(
    Output("grip-status", "children"),
    Input("grip-interval", "n_intervals"),
    Input("grip-target", "value"),
    Input("grip-hand", "value"),
    State("grip-max-inc", "value"),
)
def on_grip_target(
    _tick: int,
    percent: float | None,
    hand: str | None,
    max_increment: float | None,
) -> str:
    if dash.ctx.triggered_id in {"grip-target", "grip-hand"}:
        GRIP_CONTROLLER.start_move(
            hand=str(hand or "right"),
            percent=float(percent if percent is not None else 100.0),
            max_increment=float(max_increment or 2.0),
            iface=ROBOT_IFACE,
        )
    status, err, running = GRIP_CONTROLLER.snapshot()
    prefix = "running" if running else ("error" if err else "idle")
    return f"Grip: {prefix} | {status}"


@app.callback(
    Output("settings-result", "children"),
    Output("iface-current", "children"),
    Input("btn-apply-iface", "n_clicks"),
    State("iface-input", "value"),
    prevent_initial_call=True,
)
def on_apply_iface(_n: int | None, iface_input: str | None) -> tuple[str, str]:
    global ROBOT_INSTANCE, ROBOT_INIT_ERR, ROBOT_IFACE, DEPTH_PREVIEW, RGB_PREVIEW, RGBD_PREVIEW, LIVOX_PREVIEW
    iface = (iface_input or "").strip()
    if not iface:
        return "Interface cannot be empty.", ROBOT_IFACE

    with ROBOT_LOCK:
        ROBOT_IFACE = iface
        ROBOT_INSTANCE = None
        ROBOT_INIT_ERR = None
    with DEPTH_LOCK:
        if DEPTH_PREVIEW is not None:
            DEPTH_PREVIEW.stop()
        DEPTH_PREVIEW = None
    with RGB_LOCK:
        if RGB_PREVIEW is not None:
            RGB_PREVIEW.stop()
        RGB_PREVIEW = None
    with RGBD_LOCK:
        if RGBD_PREVIEW is not None:
            RGBD_PREVIEW.stop()
        RGBD_PREVIEW = None
    with LIVOX_LOCK:
        if LIVOX_PREVIEW is not None:
            LIVOX_PREVIEW.stop()
        LIVOX_PREVIEW = None

    return f"Iface updated to '{iface}'. Robot client will reconnect on next command.", iface


@app.callback(
    Output("say-result", "children"),
    Input("btn-say", "n_clicks"),
    State("say-text", "value"),
    prevent_initial_call=True,
)
def on_say(_n: int | None, text: str | None) -> str:
    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}"

    phrase = (text or "").strip()
    if not phrase:
        return "Enter text before pressing Say."

    try:
        robot.say(phrase)
        return f"Said: {phrase}"
    except Exception as exc:
        LOGGER.exception("Say command failed")
        return f"Say failed: {exc}"


@app.callback(
    Output("headlight-result", "children"),
    Input("btn-headlight", "n_clicks"),
    State("headlight-color", "value"),
    State("headlight-intensity", "value"),
    State("headlight-duration", "value"),
    prevent_initial_call=True,
)
def on_headlight(_n: int | None, color: str | None, intensity: int | float | None, duration: float | None) -> str:
    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}"
    color_value = str(color or "white").strip() or "white"
    intensity_value = int(max(0, min(100, float(intensity if intensity is not None else 100))))
    duration_value = None if duration is None or float(duration) <= 0 else float(duration)
    try:
        try:
            rc = int(robot.headlight(color=color_value, intensity=intensity_value, duration=duration_value))
        except TypeError:
            rc = int(robot.headlight({"color": color_value, "intensity": intensity_value}, duration=duration_value))
        return f"Headlight applied: color={color_value}, intensity={intensity_value}, duration={duration_value}, rc={rc}"
    except Exception as exc:
        LOGGER.exception("Headlight command failed")
        return f"Headlight failed: {exc}"


@app.callback(
    Output("rgb-feed", "src"),
    Output("rgb-status", "children"),
    Input("rgb-interval", "n_intervals"),
    State("rgb-feed", "src"),
)
def update_rgb_feed(_tick: int, prev_src: str | None) -> tuple[str | None, str]:
    try:
        preview = get_rgbd_preview()
        preview.start()
        jpeg, _depth_jpeg, ts, _center_depth_m, _near_cov, err = preview.snapshot()
        if err is not None:
            raise RuntimeError(err)
        if jpeg is None:
            raise RuntimeError("Waiting for RGBD frames.")
        payload = base64.b64encode(jpeg).decode("ascii")
        src = f"data:image/jpeg;base64,{payload}"
        age_s = max(0.0, time.time() - ts) if ts > 0 else -1.0
        return src, f"RGB OK (RealSense ZMQ {RGBD_HOST}:{RGBD_PORT}) | bytes: {len(jpeg)} | age_s: {age_s:.2f}"
    except Exception as exc:
        robot = get_robot()
        if robot is None:
            return prev_src, f"RGBD stream failed: {exc} | robot fallback unavailable: {ROBOT_INIT_ERR}"
        try:
            jpeg = robot.get_rgb_jpeg(timeout=2.0)
            payload = base64.b64encode(jpeg).decode("ascii")
            src = f"data:image/jpeg;base64,{payload}"
            return src, f"RGB OK (VideoClient fallback) | bytes: {len(jpeg)} | note: {exc}"
        except Exception as fallback_exc:
            return prev_src, f"RGB read failed: {exc} | fallback failed: {fallback_exc}"


@app.callback(
    Output("depth-feed", "src"),
    Output("depth-status", "children"),
    Input("depth-interval", "n_intervals"),
    State("depth-feed", "src"),
)
def update_depth_feed(_tick: int, prev_src: str | None) -> tuple[str | None, str]:
    try:
        preview = get_rgbd_preview()
        preview.start()
        _rgb_jpeg, jpeg, ts, center_depth_m, near_cov, err = preview.snapshot()
        if err is not None:
            return prev_src, err
        if jpeg is None:
            return prev_src, f"Waiting for depth frames on tcp://{RGBD_HOST}:{RGBD_PORT}."

        age_s = max(0.0, time.time() - ts) if ts > 0 else -1.0
        payload = base64.b64encode(jpeg).decode("ascii")
        src = f"data:image/jpeg;base64,{payload}"
        center_text = f"{center_depth_m:.2f}m" if center_depth_m is not None else "n/a"
        near_text = f"{near_cov * 100.0:.1f}%" if near_cov is not None else "n/a"
        return src, f"Depth OK | bytes: {len(jpeg)} | center: {center_text} | near@1m: {near_text} | age_s: {age_s:.2f}"
    except Exception as exc:
        return prev_src, f"Depth read failed: {exc}"


@app.callback(
    Output("lidar-graph", "figure"),
    Output("lidar-status", "children"),
    Output("imu-graph", "figure"),
    Output("imu-status", "children"),
    Input("lidar-interval", "n_intervals"),
)
def update_lidar(_tick: int) -> tuple[go.Figure, str, go.Figure, str]:
    robot = get_robot()
    if robot is None:
        return (
            empty_lidar_figure("LiDAR stream unavailable"),
            f"Robot init failed: {ROBOT_INIT_ERR}",
            empty_imu_figure("IMU unavailable"),
            f"Robot init failed: {ROBOT_INIT_ERR}",
        )

    try:
        pts = robot.get_lidar_points(max_points=4000)
    except Exception as exc:
        lidar_fig = empty_lidar_figure("LiDAR stream error")
        lidar_status = f"LiDAR read failed: {exc}"
    else:
        if not pts:
            live = get_livox_preview()
            live.start()
            xyz, live_ts, live_err = live.snapshot()
            if xyz is not None:
                import numpy as np

                arr = np.asarray(xyz, dtype=np.float32)
                if arr.ndim == 2 and arr.shape[1] >= 3:
                    xs_a = arr[:, 0]
                    ys_a = arr[:, 1]
                    zs_a = arr[:, 2]
                    # Mirror the top-down limits used by lidar_points.py.
                    mask = (
                        (zs_a >= -1.0)
                        & (zs_a <= 2.0)
                        & (np.abs(xs_a) <= 10.0)
                        & (np.abs(ys_a) <= 10.0)
                    )
                    xs = xs_a[mask].tolist()
                    ys = ys_a[mask].tolist()
                    zs = zs_a[mask].tolist()
                else:
                    xs, ys, zs = [], [], []

                lidar_fig = go.Figure(
                    data=[
                        go.Scattergl(
                            x=xs,
                            y=ys,
                            mode="markers",
                            marker={"size": 3, "color": zs, "colorscale": "Viridis", "showscale": True},
                            name="LiDAR",
                        )
                    ]
                )
                lidar_fig.update_layout(
                    template="plotly_dark",
                    title="LiDAR stream (live_points fallback)",
                    xaxis_title="X (m)",
                    yaxis_title="Y (m)",
                    margin={"l": 30, "r": 20, "t": 45, "b": 35},
                    height=500,
                )
                age = max(0.0, time.time() - live_ts) if live_ts > 0 else -1.0
                lidar_status = f"Points: {len(xs)} | source: live_points | age_s: {age:.2f}"
            else:
                stale = robot.sensors_stale(max_age=1.5)
                lidar_fig = empty_lidar_figure("LiDAR stream (no points yet)")
                extra = f" | live_err: {live_err}" if live_err else ""
                lidar_status = f"No LiDAR points yet. stale={stale}{extra}"
        else:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]

            lidar_fig = go.Figure(
                data=[
                    go.Scattergl(
                        x=xs,
                        y=ys,
                        mode="markers",
                        marker={"size": 3, "color": zs, "colorscale": "Viridis", "showscale": True},
                        name="LiDAR",
                    )
                ]
            )
            lidar_fig.update_layout(
                template="plotly_dark",
                title="LiDAR stream",
                xaxis_title="X (m)",
                yaxis_title="Y (m)",
                margin={"l": 30, "r": 20, "t": 45, "b": 35},
                height=500,
            )

            ts = robot.get_sensor_timestamps()
            age = max(0.0, time.time() - ts.get("lidar_cloud", 0.0)) if ts.get("lidar_cloud", 0.0) > 0 else -1.0
            lidar_status = (
                f"Points: {len(pts)} | topic: {ROBOT_LIDAR_CLOUD_TOPIC} | "
                f"lidar_cloud_age_s: {age:.2f}"
            )

    imu = robot.get_imu()
    if imu is None:
        imu_fig = empty_imu_figure("IMU orientation (no data yet)")
        imu_status = "No IMU data yet."
    else:
        now = time.time()
        roll = float(imu.rpy[0])
        pitch = float(imu.rpy[1])
        yaw = float(imu.rpy[2])
        IMU_HISTORY.append((now, roll, pitch, yaw))

        t0 = IMU_HISTORY[0][0]
        rel_t = [row[0] - t0 for row in IMU_HISTORY]
        roll_s = [row[1] for row in IMU_HISTORY]
        pitch_s = [row[2] for row in IMU_HISTORY]
        yaw_s = [row[3] for row in IMU_HISTORY]

        imu_fig = go.Figure(
            data=[
                go.Scatter(x=rel_t, y=roll_s, mode="lines", name="roll"),
                go.Scatter(x=rel_t, y=pitch_s, mode="lines", name="pitch"),
                go.Scatter(x=rel_t, y=yaw_s, mode="lines", name="yaw"),
            ]
        )
        imu_fig.update_layout(
            template="plotly_dark",
            title="IMU orientation (RPY)",
            xaxis_title="Time (s, recent window)",
            yaxis_title="Angle (rad)",
            margin={"l": 30, "r": 20, "t": 45, "b": 35},
            height=320,
        )
        imu_status = f"Latest RPY(rad): [{roll:.3f}, {pitch:.3f}, {yaw:.3f}] | samples: {len(IMU_HISTORY)}"

    return lidar_fig, lidar_status, imu_fig, imu_status


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dash robot control dashboard.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--robot-ip", "--rgbd-host", dest="rgbd_host", default=RGBD_HOST)
    parser.add_argument("--rgbd-port", type=int, default=RGBD_PORT)
    parser.add_argument("--rgbd-topic", default=RGBD_TOPIC)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    RGBD_HOST = str(args.rgbd_host)
    RGBD_PORT = int(args.rgbd_port)
    RGBD_TOPIC = str(args.rgbd_topic)
    app.run(host=str(args.host), port=int(args.port), debug=False)
