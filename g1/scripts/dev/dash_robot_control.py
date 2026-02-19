#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import base64
import os
from pathlib import Path
import threading
import time
from typing import Any
import sys

import dash
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
from dash import Input, Output, State, dcc, html

from dev.ef_client import Robot


ROBOT_LOCK = threading.Lock()
ROBOT_INSTANCE: Robot | None = None
ROBOT_INIT_ERR: str | None = None
ROBOT_IFACE = "enp1s0"
ROBOT_LIDAR_CLOUD_TOPIC = "rt/utlidar/cloud_livox_mid360"
IMU_HISTORY: deque[tuple[float, float, float, float]] = deque(maxlen=300)
DEPTH_LOCK = threading.Lock()
DEPTH_PREVIEW: "_DepthPreviewReceiver | None" = None
RGB_LOCK = threading.Lock()
RGB_PREVIEW: "_RgbPreviewReceiver | None" = None
LIVOX_LOCK = threading.Lock()
LIVOX_PREVIEW: "_LivoxPointsReceiver | None" = None


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
            )
            return ROBOT_INSTANCE
        except Exception as exc:
            ROBOT_INIT_ERR = str(exc)
            return None


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


app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.title = "Robot Control"

app.layout = dbc.Container(
    [
        html.H3("Robot Control Dashboard", className="mt-3 mb-3"),
        dbc.Alert(id="status-alert", color="secondary", children="Ready", className="mb-3"),
        dbc.Tabs(
            [
                dbc.Tab(
                    label="Control",
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
                                        "Run hanged_boot_sequence",
                                        id="btn-hanged-boot",
                                        color="primary",
                                        className="w-100 mt-2",
                                    ),
                                    md=6,
                                ),
                                dbc.Col(
                                    dbc.InputGroup(
                                        [
                                            dbc.Input(
                                                id="boot-enter-input",
                                                type="text",
                                                placeholder="Focus here, then press Enter",
                                                debounce=False,
                                            ),
                                            dbc.Button("Enter", id="btn-boot-enter", color="success"),
                                        ]
                                    ),
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
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
                                dbc.Col(
                                    dbc.Button(
                                        "Start SLAM Service (viz=True)",
                                        id="btn-slam-service",
                                        color="primary",
                                        className="mt-4 w-100",
                                    ),
                                    md=6,
                                ),
                            ],
                            className="g-2",
                        ),
                        dbc.Row(
                            [
                                dbc.Col(
                                    dbc.Button(
                                        "Start RGB Video Client",
                                        id="btn-rgb",
                                        color="info",
                                        className="mt-3 w-100",
                                    ),
                                    md=6,
                                )
                            ],
                            className="g-2",
                        ),
                        html.Div(id="control-result", className="mt-3"),
                    ],
                ),
                dbc.Tab(
                    label="Navigation",
                    children=[
                        dbc.Row(
                            [
                                dbc.Col(
                                    [
                                        html.Div("vx (m/s)", className="mt-3 mb-1"),
                                        dbc.Input(id="nav-vx", type="number", value=0.0, step=0.05),
                                    ],
                                    md=4,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("vy (m/s)", className="mt-3 mb-1"),
                                        dbc.Input(id="nav-vy", type="number", value=0.0, step=0.05),
                                    ],
                                    md=4,
                                ),
                                dbc.Col(
                                    [
                                        html.Div("vyaw (rad/s)", className="mt-3 mb-1"),
                                        dbc.Input(id="nav-vyaw", type="number", value=0.0, step=0.05),
                                    ],
                                    md=4,
                                ),
                            ],
                            className="g-2",
                        ),
                        dbc.Row(
                            [
                                dbc.Col(
                                    dbc.Button("move(vx, vy, vyaw)", id="btn-nav-move", color="primary", className="w-100 mt-3"),
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
                    label="Sensors",
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
                    label="Speech",
                    children=[
                        dbc.InputGroup(
                            [
                                dbc.Input(id="say-text", placeholder="Type text to speak", type="text"),
                                dbc.Button("Say", id="btn-say", color="success"),
                            ],
                            className="mt-3",
                        ),
                        html.Div(id="say-result", className="mt-3"),
                    ],
                ),
                dbc.Tab(
                    label="Settings",
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
            className="mb-3",
        ),
        dcc.Interval(id="lidar-interval", interval=1000, n_intervals=0),
        dcc.Interval(id="rgb-interval", interval=500, n_intervals=0),
        dcc.Interval(id="depth-interval", interval=500, n_intervals=0),
    ],
    fluid=True,
)


@app.callback(
    Output("status-alert", "children"),
    Output("status-alert", "color"),
    Output("control-result", "children"),
    Input("btn-damp", "n_clicks"),
    Input("btn-zero", "n_clicks"),
    Input("btn-stop", "n_clicks"),
    Input("btn-hanged-boot", "n_clicks"),
    Input("btn-boot-enter", "n_clicks"),
    Input("boot-enter-input", "n_submit"),
    Input("btn-rgb", "n_clicks"),
    Input("btn-slam-service", "n_clicks"),
    Input("gait-toggle", "value"),
    prevent_initial_call=True,
)
def on_control(
    _damp: int | None,
    _zero: int | None,
    _stop: int | None,
    _hanged_boot: int | None,
    _boot_enter_btn: int | None,
    _boot_enter_submit: int | None,
    _rgb: int | None,
    _slam: int | None,
    gait_value: str,
) -> tuple[str, str, str]:
    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}", "danger", "No robot instance available."

    trigger = dash.ctx.triggered_id
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
        if trigger in ("btn-hanged-boot", "btn-boot-enter", "boot-enter-input"):
            if hasattr(robot, "hanged_boot_sequence"):
                getattr(robot, "hanged_boot_sequence")()
            elif hasattr(robot, "hanged_boot"):
                robot.hanged_boot()
            elif hasattr(robot, "hanging_boot"):
                robot.hanging_boot()
            else:
                robot.balanced_stand(0)
            return (
                "hanged_boot_sequence completion sent.",
                "success",
                "Sent balanced-stand completion command (Enter equivalent).",
            )
        if trigger == "btn-rgb":
            robot.get_rgbd_gst(detect="none")
            return "RGB video client started.", "info", "Started local RGB video client process."
        if trigger == "btn-slam-service":
            def _run_slam_service() -> None:
                try:
                    robot.slam_service(viz=True)
                except Exception as exc:
                    print(f"slam_service failed: {exc}")

            threading.Thread(target=_run_slam_service, daemon=True).start()
            return "SLAM service launched.", "primary", "Started Robot.slam_service(viz=True) in background thread."
        if trigger == "gait-toggle":
            if gait_value == "run":
                robot.set_gait_type(1)
                return "Gait switched to run.", "primary", "Set gait type to run (1)."
            robot.set_gait_type(0)
            return "Gait switched to walk.", "primary", "Set gait type to walk (0)."
    except Exception as exc:
        return f"Command failed: {exc}", "danger", str(exc)

    return "Ready", "secondary", "No action taken."


@app.callback(
    Output("nav-result", "children"),
    Input("btn-nav-move", "n_clicks"),
    Input("btn-nav-stop", "n_clicks"),
    State("nav-vx", "value"),
    State("nav-vy", "value"),
    State("nav-vyaw", "value"),
    prevent_initial_call=True,
)
def on_navigation(
    _move_clicks: int | None,
    _stop_clicks: int | None,
    vx: float | None,
    vy: float | None,
    vyaw: float | None,
) -> str:
    robot = get_robot()
    if robot is None:
        return f"Robot init failed: {ROBOT_INIT_ERR}"

    trigger = dash.ctx.triggered_id
    try:
        if trigger == "btn-nav-stop":
            robot.stop()
            return "stop() sent."
        if trigger == "btn-nav-move":
            cmd_vx = float(vx or 0.0)
            cmd_vy = float(vy or 0.0)
            cmd_vyaw = float(vyaw or 0.0)
            if hasattr(robot, "move"):
                rc = int(getattr(robot, "move")(cmd_vx, cmd_vy, cmd_vyaw))
            elif hasattr(robot, "loco_move"):
                rc = int(robot.loco_move(cmd_vx, cmd_vy, cmd_vyaw))
            else:
                rc = int(robot.walk(cmd_vx, cmd_vy, cmd_vyaw))
            return f"move(vx={cmd_vx:.3f}, vy={cmd_vy:.3f}, vyaw={cmd_vyaw:.3f}) sent, rc={rc}."
    except Exception as exc:
        return f"Navigation command failed: {exc}"

    return "No navigation action taken."


@app.callback(
    Output("settings-result", "children"),
    Output("iface-current", "children"),
    Input("btn-apply-iface", "n_clicks"),
    State("iface-input", "value"),
    prevent_initial_call=True,
)
def on_apply_iface(_n: int | None, iface_input: str | None) -> tuple[str, str]:
    global ROBOT_INSTANCE, ROBOT_INIT_ERR, ROBOT_IFACE, DEPTH_PREVIEW, RGB_PREVIEW, LIVOX_PREVIEW
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
        return f"Say failed: {exc}"


@app.callback(
    Output("rgb-feed", "src"),
    Output("rgb-status", "children"),
    Input("rgb-interval", "n_intervals"),
    State("rgb-feed", "src"),
)
def update_rgb_feed(_tick: int, prev_src: str | None) -> tuple[str | None, str]:
    robot = get_robot()
    if robot is None:
        return prev_src, f"Robot init failed: {ROBOT_INIT_ERR}"

    try:
        preview = get_rgb_preview(robot)
        preview.start()
        jpeg, ts, err = preview.snapshot()
        if err is not None:
            raise RuntimeError(err)
        if jpeg is None:
            raise RuntimeError("Waiting for RGB frames on UDP stream.")
        payload = base64.b64encode(jpeg).decode("ascii")
        src = f"data:image/jpeg;base64,{payload}"
        age_s = max(0.0, time.time() - ts) if ts > 0 else -1.0
        return src, f"RGB OK (UDP) | bytes: {len(jpeg)} | age_s: {age_s:.2f}"
    except Exception as exc:
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
    robot = get_robot()
    if robot is None:
        return prev_src, f"Robot init failed: {ROBOT_INIT_ERR}"

    try:
        preview = get_depth_preview(robot)
        preview.start()
        jpeg, ts, center_depth_m, near_cov, err = preview.snapshot()
        if err is not None:
            return prev_src, err
        if jpeg is None:
            return prev_src, "Waiting for depth frames on UDP stream."

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


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000, debug=False)
