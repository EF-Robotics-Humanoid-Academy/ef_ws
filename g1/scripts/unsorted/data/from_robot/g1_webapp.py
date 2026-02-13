#!/usr/bin/env python3
"""
Web dashboard for Unitree G1 DDS topics.

- Polls DDS topics via CycloneDDS.
- Shows topic status, IMU summary, Lidar point cloud, and RGB/Depth feeds when available.
"""

from __future__ import annotations

import argparse
import io
import json
import logging
import threading
import time
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
from PIL import Image
from flask import Flask, Response, jsonify, request


LOG = logging.getLogger("g1_webapp")
RGBD_TOPIC_KEYWORDS = ("rgbd", "depth", "rgb", "color", "image", "camera")


class TopicStats:
    def __init__(self) -> None:
        self.sample_count: int = 0
        self.last_ts: float = 0.0
        self.last_error: str = ""
        self.last_sample_preview: str = ""

    def update_sample(self, sample: Any) -> None:
        self.sample_count += 1
        self.last_ts = time.time()
        self.last_error = ""
        self.last_sample_preview = _truncate(_format_sample(sample), 260)

    def update_error(self, exc: Exception) -> None:
        self.last_error = str(exc)
        self.last_ts = time.time()


class TopicReader:
    def __init__(self, participant: Any, topic_name: str, msg_type: Any, type_label: str) -> None:
        from cyclonedds.topic import Topic
        from cyclonedds.sub import DataReader

        self.topic_name = topic_name
        self.msg_type = msg_type
        self.type_label = type_label
        self.topic = Topic(participant, topic_name, msg_type)
        self.reader = DataReader(participant, self.topic)

    def read(self, max_samples: int = 8) -> Iterable[Any]:
        try:
            return self.reader.read(max_samples)
        except Exception as exc:
            raise RuntimeError(f"read failed for {self.topic_name}") from exc


class DdsDiscovery:
    def __init__(self, domain_id: int) -> None:
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsTopic, BuiltinTopicDcpsPublication

        self.participant = DomainParticipant(domain_id)
        self.topic_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsTopic)
        self.pub_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsPublication)
        self.seen_topics: Dict[str, str] = {}
        self.seen_publications: set[Tuple[str, str]] = set()

    def poll(self) -> List[Tuple[str, str]]:
        discovered: List[Tuple[str, str]] = []
        try:
            topics = self.topic_reader.read(64)
        except Exception:
            topics = []
        for t in topics:
            try:
                topic_name = t.topic_name
                type_name = t.type_name
            except Exception:
                continue
            if topic_name not in self.seen_topics:
                self.seen_topics[topic_name] = type_name
                discovered.append((topic_name, type_name))

        try:
            pubs = self.pub_reader.read(64)
        except Exception:
            pubs = []
        for p in pubs:
            try:
                key = (p.topic_name, p.type_name)
            except Exception:
                continue
            if key not in self.seen_publications:
                self.seen_publications.add(key)
        return discovered


def _load_config(path: Optional[str]) -> Dict[str, Any]:
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = f.read()
    if path.endswith(".json"):
        return json.loads(data)
    if path.endswith(".yaml") or path.endswith(".yml"):
        try:
            import yaml  # type: ignore
        except Exception as exc:
            raise RuntimeError("PyYAML not installed but YAML config provided") from exc
        return yaml.safe_load(data) or {}
    return json.loads(data)


def _type_name_to_idl_module(type_name: str) -> Optional[Tuple[str, str]]:
    if not type_name or "::" not in type_name:
        return None
    parts = type_name.split("::")
    if len(parts) < 2:
        return None
    class_name = parts[-1]
    namespace = parts[:-1]
    module_path = "unitree_sdk2py.idl." + ".".join(namespace) + "._" + class_name
    return module_path, class_name


def _resolve_type(type_path: str) -> Any:
    if ":" in type_path:
        module_path, class_name = type_path.split(":", 1)
    else:
        module_path, class_name = type_path.rsplit(".", 1)
    module = __import__(module_path, fromlist=[class_name])
    return getattr(module, class_name)


def _type_path_candidates(type_path: str) -> List[str]:
    candidates: List[str] = []
    if "::" not in type_path:
        return [type_path]
    mapped = _type_name_to_idl_module(type_path)
    if mapped:
        module_path, class_name = mapped
        candidates.append(f"{module_path}:{class_name}")
    if type_path.startswith("sensor_msgs::msg::dds_::"):
        class_name = type_path.split("::")[-1]
        candidates.append(f"unitree_sdk2py.idl.sensor_msgs.msg.dds_._{class_name}:{class_name}")
    candidates.append(type_path)
    return candidates


def _build_profiles() -> Dict[str, Dict[str, Any]]:
    return {
        "g1_basic": {
            "topics": [
                {"topic": "rt/lowstate", "type": "unitree_hg::msg::dds_::LowState_"},
                {"topic": "rt/lowcmd", "type": "unitree_hg::msg::dds_::LowCmd_"},
                {"topic": "rt/dex3/left/state", "type": "unitree_hg::msg::dds_::HandState_"},
                {"topic": "rt/dex3/right/state", "type": "unitree_hg::msg::dds_::HandState_"},
                {"topic": "rt/dex3/left/cmd", "type": "unitree_hg::msg::dds_::HandCmd_"},
                {"topic": "rt/dex3/right/cmd", "type": "unitree_hg::msg::dds_::HandCmd_"},
            ],
        },
        "g1_sport": {
            "topics": [
                {"topic": "rt/arm_sdk", "type": "unitree_hg::msg::dds_::LowCmd_"},
            ],
        },
        "g1_odom": {
            "topics": [
                {"topic": "rt/odommodestate", "type": "unitree_go::msg::dds_::SportModeState_"},
                {"topic": "rt/lf/odommodestate", "type": "unitree_go::msg::dds_::SportModeState_"},
            ],
        },
        "g1_lidar": {
            "topics": [
                {"topic": "rt/utlidar/cloud_livox_mid360", "type": "sensor_msgs::msg::dds_::PointCloud2_"},
                {"topic": "rt/utlidar/imu_livox_mid360", "type": "sensor_msgs::msg::dds_::Imu_"},
            ],
        },
    }


def _build_readers_from_config(participant: Any, config: Dict[str, Any]) -> Dict[str, TopicReader]:
    readers: Dict[str, TopicReader] = {}
    for item in config.get("topics", []) or []:
        topic = item.get("topic")
        type_path = item.get("type")
        if not topic or not type_path:
            LOG.warning("Skipping topic entry missing 'topic' or 'type': %s", item)
            continue
        try:
            last_exc: Optional[Exception] = None
            msg_type = None
            for candidate in _type_path_candidates(type_path):
                try:
                    msg_type = _resolve_type(candidate)
                    break
                except Exception as exc:
                    last_exc = exc
            if msg_type is None:
                raise last_exc or RuntimeError("No candidate types resolved")
            readers[topic] = TopicReader(participant, topic, msg_type, type_path)
            LOG.info("Subscribed (config): %s -> %s", topic, type_path)
        except Exception as exc:
            if isinstance(exc, ModuleNotFoundError):
                missing = getattr(exc, "name", None) or str(exc)
                LOG.warning(
                    "Skipping topic %s (%s). Missing module: %s. Update config or install the IDL module.",
                    topic,
                    type_path,
                    missing,
                )
            else:
                LOG.exception("Failed to subscribe (config) to %s with %s", topic, type_path)
    return readers


def _format_sample(sample: Any) -> str:
    try:
        if is_dataclass(sample):
            return json.dumps(asdict(sample), ensure_ascii=True)
    except Exception:
        pass
    try:
        if hasattr(sample, "to_dict"):
            return json.dumps(sample.to_dict(), ensure_ascii=True)
    except Exception:
        pass
    try:
        return repr(sample)
    except Exception:
        return "<unprintable sample>"


def _truncate(value: str, limit: int) -> str:
    if len(value) <= limit:
        return value
    return value[: limit - 3] + "..."


def _format_timestamp(ts: float) -> str:
    if ts <= 0.0:
        return "never"
    return time.strftime("%H:%M:%S", time.localtime(ts))


def _get_attr(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _to_bytes(data: Any) -> Optional[bytes]:
    if data is None:
        return None
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if isinstance(data, list):
        try:
            return bytes(data)
        except Exception:
            return None
    if hasattr(data, "tobytes"):
        try:
            return data.tobytes()
        except Exception:
            return None
    return None


def _jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return {"__bytes__": len(value)}
    if isinstance(value, (float, int, str, bool)) or value is None:
        return value
    if hasattr(value, "to_dict"):
        try:
            return _jsonable(value.to_dict())
        except Exception:
            return repr(value)
    return repr(value)


def _extract_imu(sample: Any) -> Optional[Dict[str, Any]]:
    if sample is None:
        return None
    imu_state = _get_attr(sample, "imu_state")
    if imu_state is None:
        return None
    return {
        "quaternion": _get_attr(imu_state, "quaternion"),
        "gyroscope": _get_attr(imu_state, "gyroscope"),
        "accelerometer": _get_attr(imu_state, "accelerometer"),
        "rpy": _get_attr(imu_state, "rpy"),
        "temperature": _get_attr(imu_state, "temperature"),
    }


def _extract_pointcloud(sample: Any, max_points: int = 5000) -> Optional[Dict[str, Any]]:
    if sample is None:
        return None
    fields = _get_attr(sample, "fields")
    data = _to_bytes(_get_attr(sample, "data"))
    height = _get_attr(sample, "height")
    width = _get_attr(sample, "width")
    point_step = _get_attr(sample, "point_step")
    if not fields or data is None or not height or not width or not point_step:
        return None

    offsets: Dict[str, Tuple[int, int]] = {}
    for f in fields:
        name = _get_attr(f, "name")
        offset = _get_attr(f, "offset")
        datatype = _get_attr(f, "datatype")
        if not name:
            continue
        offsets[str(name)] = (int(offset), int(datatype))

    def dtype_for(datatype: int) -> Optional[str]:
        return {
            1: "b",
            2: "B",
            3: "h",
            4: "H",
            5: "i",
            6: "I",
            7: "f",
            8: "d",
        }.get(datatype)

    def read_value(base: int, name: str) -> Optional[float]:
        if name not in offsets:
            return None
        offset, datatype = offsets[name]
        fmt = dtype_for(datatype)
        if fmt is None:
            return None
        try:
            import struct

            return struct.unpack_from("<" + fmt, data, base + offset)[0]
        except Exception:
            return None

    total = int(height) * int(width)
    if total <= 0:
        return None
    stride = max(1, total // max_points)

    points: List[List[float]] = []
    for idx in range(0, total, stride):
        base = idx * int(point_step)
        x = read_value(base, "x")
        y = read_value(base, "y")
        z = read_value(base, "z")
        if x is None or y is None or z is None:
            continue
        intensity = read_value(base, "intensity")
        if intensity is None:
            intensity = 0.0
        points.append([float(x), float(y), float(z), float(intensity)])
        if len(points) >= max_points:
            break

    header = _get_attr(sample, "header")
    return {
        "frame_id": _get_attr(header, "frame_id") if header else "",
        "stamp": _jsonable(_get_attr(header, "stamp")) if header else None,
        "points": points,
    }


def _decode_image(sample: Any) -> Optional[Image.Image]:
    if sample is None:
        return None
    width = _get_attr(sample, "width")
    height = _get_attr(sample, "height")
    encoding = _get_attr(sample, "encoding")
    data = _to_bytes(_get_attr(sample, "data"))
    if not width or not height or not encoding or data is None:
        return None

    enc = str(encoding).lower()
    width = int(width)
    height = int(height)

    if enc in ("rgb8", "bgr8", "rgba8", "bgra8"):
        channels = 4 if "a" in enc else 3
        arr = np.frombuffer(data, dtype=np.uint8)
        expected = width * height * channels
        if arr.size < expected:
            return None
        arr = arr[:expected].reshape((height, width, channels))
        if enc.startswith("bgr"):
            arr = arr[:, :, ::-1]
        if channels == 4:
            img = Image.fromarray(arr, mode="RGBA").convert("RGB")
        else:
            img = Image.fromarray(arr, mode="RGB")
        return img

    if enc in ("mono8", "8uc1"):
        arr = np.frombuffer(data, dtype=np.uint8)
        expected = width * height
        if arr.size < expected:
            return None
        arr = arr[:expected].reshape((height, width))
        return Image.fromarray(arr, mode="L").convert("RGB")

    if enc in ("mono16", "16uc1", "16uc"):
        arr = np.frombuffer(data, dtype=np.uint16)
        expected = width * height
        if arr.size < expected:
            return None
        arr = arr[:expected].reshape((height, width)).astype(np.float32)
        max_val = np.percentile(arr, 95) if arr.size else 1.0
        max_val = max(max_val, 1.0)
        arr = np.clip(arr / max_val * 255.0, 0, 255).astype(np.uint8)
        return Image.fromarray(arr, mode="L").convert("RGB")

    if enc in ("32fc1",):
        arr = np.frombuffer(data, dtype=np.float32)
        expected = width * height
        if arr.size < expected:
            return None
        arr = arr[:expected].reshape((height, width))
        max_val = np.percentile(arr, 95) if arr.size else 1.0
        max_val = max(max_val, 1e-6)
        arr = np.clip(arr / max_val * 255.0, 0, 255).astype(np.uint8)
        return Image.fromarray(arr, mode="L").convert("RGB")

    return None


class AppState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.readers: Dict[str, TopicReader] = {}
        self.stats: Dict[str, TopicStats] = {}
        self.latest_samples: Dict[str, Any] = {}
        self.latest_types: Dict[str, str] = {}
        self.running = True

    def update_reader(self, topic: str, reader: TopicReader) -> None:
        with self.lock:
            self.readers[topic] = reader
            self.latest_types[topic] = reader.type_label

    def update_sample(self, topic: str, sample: Any) -> None:
        with self.lock:
            stat = self.stats.setdefault(topic, TopicStats())
            stat.update_sample(sample)
            self.latest_samples[topic] = sample

    def update_error(self, topic: str, exc: Exception) -> None:
        with self.lock:
            stat = self.stats.setdefault(topic, TopicStats())
            stat.update_error(exc)

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            out = {}
            for topic, reader in self.readers.items():
                stat = self.stats.get(topic) or TopicStats()
                out[topic] = {
                    "type": reader.type_label,
                    "samples": stat.sample_count,
                    "last": _format_timestamp(stat.last_ts),
                    "error": stat.last_error,
                    "preview": stat.last_sample_preview,
                }
            return out


def _poll_loop(state: AppState, participant: Any, readers: Dict[str, TopicReader], poll_s: float, discovery: Optional[DdsDiscovery]) -> None:
    while state.running:
        if discovery:
            for topic_name, type_name in discovery.poll():
                if topic_name in readers:
                    continue
                mapped = _type_name_to_idl_module(type_name)
                if not mapped:
                    continue
                module_path, class_name = mapped
                try:
                    module = __import__(module_path, fromlist=[class_name])
                    msg_type = getattr(module, class_name)
                except Exception:
                    continue
                try:
                    reader = TopicReader(participant, topic_name, msg_type, type_name)
                    readers[topic_name] = reader
                    state.update_reader(topic_name, reader)
                    LOG.info("Subscribed (discovered): %s -> %s", topic_name, type_name)
                except Exception:
                    LOG.exception("Failed to subscribe to discovered topic %s (%s)", topic_name, type_name)

        for topic_name, reader in list(readers.items()):
            try:
                for sample in reader.read(8):
                    state.update_sample(topic_name, sample)
            except Exception as exc:
                state.update_error(topic_name, exc)
        time.sleep(poll_s)


def create_app(state: AppState) -> Flask:
    app = Flask(__name__)

    @app.get("/")
    def index() -> Response:
        return Response(_INDEX_HTML, mimetype="text/html")

    @app.get("/api/topics")
    def api_topics() -> Response:
        return jsonify(state.snapshot())

    @app.get("/api/imu")
    def api_imu() -> Response:
        with state.lock:
            candidates = [
                t
                for t in state.latest_samples.keys()
                if "lowstate" in t or "odommodestate" in t or "imu" in t
            ]
            if not candidates:
                candidates = list(state.latest_samples.keys())
            for topic in candidates:
                imu = _extract_imu(state.latest_samples.get(topic))
                if imu:
                    return jsonify({"topic": topic, "imu": imu})
        return jsonify({"topic": "", "imu": None})

    @app.get("/api/pointcloud/<path:topic>")
    def api_pointcloud(topic: str) -> Response:
        with state.lock:
            sample = state.latest_samples.get(topic)
        payload = _extract_pointcloud(sample)
        if payload is None:
            return jsonify({"points": [], "frame_id": "", "stamp": None})
        return jsonify(payload)

    @app.get("/api/image/<path:topic>")
    def api_image(topic: str) -> Response:
        with state.lock:
            sample = state.latest_samples.get(topic)
        img = _decode_image(sample)
        if img is None:
            return Response(status=404)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        return Response(buf.getvalue(), mimetype="image/jpeg")

    @app.get("/api/sample/<path:topic>")
    def api_sample(topic: str) -> Response:
        with state.lock:
            sample = state.latest_samples.get(topic)
        return jsonify(_jsonable(sample) if sample is not None else None)

    return app


_INDEX_HTML = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>G1 Sensor Console</title>
  <style>
    :root {
      --bg: #0f172a;
      --bg-2: #0b1223;
      --card: #111827;
      --card-2: #1f2937;
      --accent: #f59e0b;
      --accent-2: #22d3ee;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --danger: #f43f5e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Space Grotesk", "Sora", "IBM Plex Sans", sans-serif;
      color: var(--text);
      background: radial-gradient(circle at 20% 10%, #1e293b 0%, var(--bg) 35%, #020617 100%);
    }
    header {
      padding: 20px 28px;
      background: linear-gradient(120deg, #111827 0%, #0b1223 60%, #0f172a 100%);
      border-bottom: 1px solid #1f2937;
    }
    header h1 {
      margin: 0;
      font-size: 22px;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }
    header p {
      margin: 6px 0 0;
      color: var(--muted);
    }
    main {
      padding: 24px 28px 40px;
      display: grid;
      gap: 18px;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    }
    .card {
      background: linear-gradient(150deg, var(--card), var(--card-2));
      border: 1px solid #1f2937;
      border-radius: 16px;
      padding: 16px;
      min-height: 180px;
      box-shadow: 0 15px 35px rgba(0, 0, 0, 0.25);
    }
    .card h2 {
      margin: 0 0 10px;
      font-size: 16px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--accent);
    }
    .metrics {
      display: grid;
      gap: 8px;
      font-size: 13px;
      color: var(--muted);
    }
    .metrics span { color: var(--text); }
    .grid-2 {
      grid-column: span 2;
    }
    .full {
      grid-column: 1 / -1;
    }
    .preview {
      margin-top: 10px;
      padding: 8px;
      background: #0b1223;
      border-radius: 12px;
      font-family: "IBM Plex Mono", "JetBrains Mono", monospace;
      font-size: 12px;
      color: #a5b4fc;
      max-height: 140px;
      overflow: auto;
      white-space: pre-wrap;
    }
    canvas {
      width: 100%;
      height: 320px;
      background: #05070f;
      border-radius: 12px;
      border: 1px solid #1f2937;
    }
    .image-grid {
      display: grid;
      gap: 12px;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    }
    .image-card {
      background: #0b1223;
      border-radius: 12px;
      padding: 10px;
      border: 1px solid #1f2937;
    }
    .image-card img {
      width: 100%;
      height: 200px;
      object-fit: cover;
      border-radius: 8px;
      border: 1px solid #1f2937;
    }
    .badge {
      display: inline-block;
      margin-top: 8px;
      padding: 4px 8px;
      border-radius: 999px;
      background: #0f172a;
      border: 1px solid #1f2937;
      font-size: 11px;
      color: var(--muted);
    }
    .error { color: var(--danger); }
    @media (max-width: 900px) {
      .grid-2 { grid-column: span 1; }
      canvas { height: 260px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>G1 Sensor Console</h1>
    <p>Live DDS feeds: RGB-D, LiDAR, IMU, and topic diagnostics.</p>
  </header>
  <main>
    <section class="card grid-2">
      <h2>Lidar Point Cloud</h2>
      <canvas id="lidarCanvas" width="900" height="420"></canvas>
      <div class="badge" id="lidarMeta">No data</div>
    </section>
    <section class="card">
      <h2>IMU</h2>
      <div class="metrics" id="imuMetrics">
        <div>Source: <span>-</span></div>
        <div>Quaternion: <span>-</span></div>
        <div>Gyro: <span>-</span></div>
        <div>Accel: <span>-</span></div>
        <div>RPY: <span>-</span></div>
        <div>Temp: <span>-</span></div>
      </div>
    </section>
    <section class="card grid-2">
      <h2>RGB-D Feeds</h2>
      <div class="image-grid" id="imageGrid">
        <div class="image-card">
          <div class="badge">No image topics discovered</div>
        </div>
      </div>
    </section>
    <section class="card full">
      <h2>Topic Status</h2>
      <div id="topicList"></div>
    </section>
  </main>

  <script>
    const lidarCanvas = document.getElementById('lidarCanvas');
    const lidarCtx = lidarCanvas.getContext('2d');
    const lidarMeta = document.getElementById('lidarMeta');
    const imuMetrics = document.getElementById('imuMetrics');
    const imageGrid = document.getElementById('imageGrid');
    const topicList = document.getElementById('topicList');

    let lidarTopic = null;
    let imageTopics = [];

    function drawPointCloud(points) {
      const w = lidarCanvas.width;
      const h = lidarCanvas.height;
      lidarCtx.clearRect(0, 0, w, h);
      lidarCtx.fillStyle = '#05070f';
      lidarCtx.fillRect(0, 0, w, h);
      if (!points || points.length === 0) {
        return;
      }
      const scale = 8.0;
      const centerX = w / 2;
      const centerY = h / 2;
      for (const p of points) {
        const x = p[0] * scale;
        const y = p[1] * scale;
        const z = p[2];
        const intensity = p[3] || 0;
        const alpha = Math.min(1.0, 0.2 + intensity / 255.0);
        const color = `rgba(34, 211, 238, ${alpha})`;
        const px = centerX + x;
        const py = centerY - y;
        if (px < 0 || px > w || py < 0 || py > h) continue;
        lidarCtx.fillStyle = color;
        lidarCtx.fillRect(px, py, 2, 2);
      }
      lidarCtx.strokeStyle = '#1f2937';
      lidarCtx.beginPath();
      lidarCtx.moveTo(centerX, 0);
      lidarCtx.lineTo(centerX, h);
      lidarCtx.moveTo(0, centerY);
      lidarCtx.lineTo(w, centerY);
      lidarCtx.stroke();
    }

    function setImu(data) {
      const spans = imuMetrics.querySelectorAll('span');
      if (!data || !data.imu) {
        spans.forEach(s => s.textContent = '-');
        return;
      }
      const imu = data.imu;
      spans[0].textContent = data.topic || '-';
      spans[1].textContent = JSON.stringify(imu.quaternion || []);
      spans[2].textContent = JSON.stringify(imu.gyroscope || []);
      spans[3].textContent = JSON.stringify(imu.accelerometer || []);
      spans[4].textContent = JSON.stringify(imu.rpy || []);
      spans[5].textContent = imu.temperature ?? '-';
    }

    function renderTopics(topics) {
      const entries = Object.entries(topics);
      if (entries.length === 0) {
        topicList.textContent = 'No active subscriptions.';
        return;
      }
      topicList.innerHTML = '';
      entries.sort().forEach(([name, info]) => {
        const card = document.createElement('div');
        card.className = 'image-card';
        const error = info.error ? `<div class="error">${info.error}</div>` : '';
        card.innerHTML = `
          <strong>${name}</strong>
          <div class="metrics">
            <div>Type: <span>${info.type}</span></div>
            <div>Samples: <span>${info.samples}</span></div>
            <div>Last: <span>${info.last}</span></div>
          </div>
          ${error}
          <div class="preview">${info.preview || '<no data yet>'}</div>
        `;
        topicList.appendChild(card);
      });
    }

    async function refreshTopics() {
      const res = await fetch('/api/topics');
      const data = await res.json();
      renderTopics(data);

      if (!lidarTopic) {
        const lidarEntry = Object.keys(data).find(t => t.includes('cloud') || t.includes('lidar'));
        if (lidarEntry) {
          lidarTopic = lidarEntry;
        }
      }

      if (imageTopics.length === 0) {
        imageTopics = Object.keys(data).filter(t => {
          const lower = t.toLowerCase();
          return ['image', 'rgb', 'depth', 'color', 'camera'].some(k => lower.includes(k));
        });
        renderImageGrid();
      }
    }

    function renderImageGrid() {
      imageGrid.innerHTML = '';
      if (imageTopics.length === 0) {
        imageGrid.innerHTML = '<div class="image-card"><div class="badge">No image topics discovered</div></div>';
        return;
      }
      imageTopics.forEach(topic => {
        const card = document.createElement('div');
        card.className = 'image-card';
        card.innerHTML = `
          <div class="badge">${topic}</div>
          <img id="img-${topic.replace(/\W/g, '_')}" alt="${topic}" />
        `;
        imageGrid.appendChild(card);
      });
    }

    async function refreshLidar() {
      if (!lidarTopic) return;
      const res = await fetch(`/api/pointcloud/${encodeURIComponent(lidarTopic)}`);
      const data = await res.json();
      drawPointCloud(data.points || []);
      if (data.points && data.points.length) {
        lidarMeta.textContent = `${data.points.length} pts | ${data.frame_id || 'frame'}`;
      } else {
        lidarMeta.textContent = 'No data';
      }
    }

    async function refreshImu() {
      const res = await fetch('/api/imu');
      const data = await res.json();
      setImu(data);
    }

    function refreshImages() {
      imageTopics.forEach(topic => {
        const img = document.getElementById(`img-${topic.replace(/\W/g, '_')}`);
        if (!img) return;
        img.src = `/api/image/${encodeURIComponent(topic)}?t=${Date.now()}`;
      });
    }

    setInterval(refreshTopics, 1500);
    setInterval(refreshLidar, 200);
    setInterval(refreshImu, 500);
    setInterval(refreshImages, 1000);

    refreshTopics();
    refreshLidar();
    refreshImu();
    refreshImages();
  </script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="G1 DDS web dashboard")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (default: 0)")
    parser.add_argument("--iface", type=str, default="", help="Network interface name")
    parser.add_argument("--config", type=str, default="", help="Path to JSON/YAML config with topics list")
    parser.add_argument(
        "--profile",
        type=str,
        default="",
        help="Built-in topic profile: g1_basic, g1_sport, g1_odom, g1_lidar (comma-separated)",
    )
    parser.add_argument("--poll", type=float, default=0.05, help="Polling interval seconds")
    parser.add_argument("--no-discover", action="store_true", help="Disable DDS discovery")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Web host")
    parser.add_argument("--port", type=int, default=8000, help="Web port")
    parser.add_argument("--log-level", type=str, default="INFO", help="Log level (DEBUG, INFO, WARNING)")

    args = parser.parse_args()

    level = getattr(logging, args.log_level.upper(), logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.iface:
        try:
            import unitree_sdk2py.core.channel as channel  # type: ignore

            if hasattr(channel, "ChannelFactoryInitialize"):
                channel.ChannelFactoryInitialize(args.domain, args.iface)
                LOG.info("ChannelFactoryInitialize(domain=%s, iface=%s) OK", args.domain, args.iface)
        except Exception:
            LOG.exception("ChannelFactoryInitialize failed")

    try:
        from cyclonedds.domain import DomainParticipant
    except Exception as exc:
        LOG.error("cyclonedds is required for DDS polling: %s", exc)
        return 2

    participant = DomainParticipant(args.domain)

    discovery = None
    if not args.no_discover:
        try:
            discovery = DdsDiscovery(args.domain)
        except Exception:
            LOG.exception("DDS discovery unavailable; continuing without discovery")
            discovery = None

    config = _load_config(args.config) if args.config else {}
    profiles = _build_profiles()
    if args.profile:
        combined: Dict[str, Any] = {"topics": []}
        for name in [p.strip() for p in args.profile.split(",") if p.strip()]:
            if name not in profiles:
                LOG.warning("Unknown profile %s (available: %s)", name, ", ".join(sorted(profiles.keys())))
                continue
            combined["topics"].extend(profiles[name].get("topics", []))
        if combined["topics"]:
            if config:
                config = {"topics": (config.get("topics", []) or []) + combined["topics"]}
            else:
                config = combined

    readers = _build_readers_from_config(participant, config)

    state = AppState()
    for topic, reader in readers.items():
        state.update_reader(topic, reader)

    poll_thread = threading.Thread(
        target=_poll_loop, args=(state, participant, readers, args.poll, discovery), daemon=True
    )
    poll_thread.start()

    app = create_app(state)
    LOG.info("Web app running on http://%s:%s", args.host, args.port)
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
