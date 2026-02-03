import json
import math
import os
import threading
import time
import urllib.parse
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_, LowState_, SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient

TOPIC_LIDAR_SWITCH = "rt/utlidar/switch"
TOPIC_LIDAR_STATE = "rt/utlidar/map_state"
TOPIC_LIDAR_POINTS = "rt/utlidar/cloud"
TOPIC_ODOM = "rt/odom"
TOPIC_SPORTSTATE = "rt/sportmodestate"

HOST = "0.0.0.0"
PORT = 8010
INTERFACE = "enp2s0"
MAP_VOXEL_SIZE = 0.05

state_lock = threading.Lock()
last_lidar_state = None
last_lidar_points = None
last_lidar_points_ts = 0.0
last_odom = None
last_sportstate = None
last_imu_rpy = None
last_imu_ts = 0.0
mapping_enabled = False

lidar_switch_pub = None
sport_client = None


@dataclass
class Pose2D:
    x: float
    y: float
    roll: float
    pitch: float
    yaw: float


class MapBuilder:
    def __init__(
        self,
        resolution: float,
        width_m: float,
        height_m: float,
        min_range: float,
        max_range: float,
        z_min: float,
        z_max: float,
        decay_sec: float,
    ):
        self.resolution = float(resolution)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.width = max(1, int(round(self.width_m / self.resolution)))
        self.height = max(1, int(round(self.height_m / self.resolution)))
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self.decay_sec = float(decay_sec)
        self.grid = [-1] * (self.width * self.height)
        self.grid_age = [0.0] * (self.width * self.height)
        self.last_update = 0.0
        self.updates = 0
        self.last_points = 0

    def reset(self):
        self.grid = [-1] * (self.width * self.height)
        self.grid_age = [0.0] * (self.width * self.height)
        self.last_update = 0.0
        self.updates = 0
        self.last_points = 0

    def world_to_grid(self, x: float, y: float):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return ix, iy

    def _grid_index(self, ix: int, iy: int):
        return iy * self.width + ix

    def _mark_free(self, ix: int, iy: int):
        idx = self._grid_index(ix, iy)
        if self.grid[idx] == -1:
            self.grid[idx] = 0
        self.grid_age[idx] = time.time()

    def _mark_occ(self, ix: int, iy: int):
        idx = self._grid_index(ix, iy)
        self.grid[idx] = 100
        self.grid_age[idx] = time.time()

    def _raytrace(self, x0: int, y0: int, x1: int, y1: int):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if x == x1 and y == y1:
                break
            self._mark_free(x, y)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def update(self, pose: Pose2D, points):
        origin = self.world_to_grid(pose.x, pose.y)
        if origin is None:
            return
        ox, oy = origin
        cos_r = math.cos(pose.roll)
        sin_r = math.sin(pose.roll)
        cos_p = math.cos(pose.pitch)
        sin_p = math.sin(pose.pitch)
        cos_y = math.cos(pose.yaw)
        sin_y = math.sin(pose.yaw)
        added = 0
        for x, y, z in points:
            rng = math.hypot(x, y)
            if rng < self.min_range or rng > self.max_range:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            # Rotate point into world frame using IMU roll/pitch/yaw.
            x1 = x
            y1 = y * cos_r - z * sin_r
            z1 = y * sin_r + z * cos_r
            x2 = x1 * cos_p + z1 * sin_p
            y2 = y1
            z2 = -x1 * sin_p + z1 * cos_p
            wx = x2 * cos_y - y2 * sin_y + pose.x
            wy = x2 * sin_y + y2 * cos_y + pose.y
            cell = self.world_to_grid(wx, wy)
            if cell is None:
                continue
            ix, iy = cell
            self._raytrace(ox, oy, ix, iy)
            self._mark_occ(ix, iy)
            added += 1
        if added:
            self.last_update = time.time()
            self.updates += 1
            self.last_points = added


map_builder = None


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lidar_state_cb(msg: LidarState_):
    global last_lidar_state
    with state_lock:
        last_lidar_state = msg


def lidar_points_cb(msg: PointCloud2_):
    global last_lidar_points, last_lidar_points_ts
    with state_lock:
        last_lidar_points = msg
        last_lidar_points_ts = time.time()


def odom_cb(msg: Odometry_):
    global last_odom
    with state_lock:
        last_odom = msg


def sportstate_cb(msg: SportModeState_):
    global last_sportstate
    with state_lock:
        last_sportstate = msg


def lowstate_cb(msg: LowState_):
    global last_imu_rpy, last_imu_ts
    with state_lock:
        last_imu_rpy = [float(v) for v in msg.imu_state.rpy]
        last_imu_ts = time.time()


def _serialize_map_info():
    if map_builder is None:
        return None
    age = time.time() - map_builder.last_update if map_builder.last_update else 0.0
    return {
        "width": map_builder.width,
        "height": map_builder.height,
        "resolution": map_builder.resolution,
        "origin": [map_builder.origin_x, map_builder.origin_y, 0.0],
        "age_sec": max(0.0, age),
        "updates": map_builder.updates,
        "last_points": map_builder.last_points,
        "mapping": mapping_enabled,
        "decay_sec": map_builder.decay_sec,
    }


def _serialize_lidar(msg: LidarState_):
    if msg is None:
        return None
    return {
        "cloud_size": int(msg.cloud_size),
        "cloud_loss": float(msg.cloud_packet_loss_rate),
        "error": int(msg.error_state),
    }


def _serialize_odom(msg: Odometry_):
    if msg is None:
        return None
    return {
        "pos": [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(msg.pose.pose.position.z)],
        "lin_vel": [float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y), float(msg.twist.twist.linear.z)],
    }


def _serialize_sportstate(msg: SportModeState_):
    if msg is None:
        return None
    return {
        "mode": int(msg.mode),
        "position": [float(v) for v in msg.position],
        "velocity": [float(v) for v in msg.velocity],
    }


def _iter_points(msg: PointCloud2_, max_points: int, voxel_size: float):
    if msg is None:
        return []
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == "x":
            x_off = int(f.offset)
        elif f.name == "y":
            y_off = int(f.offset)
        elif f.name == "z":
            z_off = int(f.offset)
    if x_off is None or y_off is None or z_off is None:
        return []
    if msg.point_step <= max(x_off, y_off, z_off) + 4:
        return []
    data = bytes(msg.data)
    total_points = int(msg.width * msg.height)
    if total_points <= 0:
        return []
    stride = max(1, total_points // max_points)
    import struct
    fmt = ">f" if msg.is_bigendian else "<f"
    points = []
    vox = float(voxel_size) if voxel_size and voxel_size > 0 else 0.0
    voxels = {} if vox > 0 else None
    for i in range(0, total_points, stride):
        base = i * msg.point_step
        if base + msg.point_step > len(data):
            break
        x = struct.unpack_from(fmt, data, base + x_off)[0]
        y = struct.unpack_from(fmt, data, base + y_off)[0]
        z = struct.unpack_from(fmt, data, base + z_off)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if voxels is None:
            points.append((float(x), float(y), float(z)))
            if len(points) >= max_points:
                break
        else:
            key = (int(x / vox), int(y / vox), int(z / vox))
            if key in voxels:
                continue
            voxels[key] = (float(x), float(y), float(z))
            if len(voxels) >= max_points:
                break
    if voxels is not None:
        return list(voxels.values())
    return points


def _extract_points_xy(msg: PointCloud2_, max_points: int, voxel_size: float):
    points = _iter_points(msg, max_points, voxel_size)
    if not points:
        return None
    xy = [[x, y] for x, y, _ in points]
    return {"count": len(xy), "points": xy}


def _get_pose():
    with state_lock:
        odom = last_odom
        imu_rpy = last_imu_rpy
        sport = last_sportstate
    if odom is None:
        if sport is None:
            return None
        if len(sport.position) < 2:
            return None
        x = float(sport.position[0])
        y = float(sport.position[1])
    else:
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)
    if imu_rpy is not None:
        roll, pitch, yaw = imu_rpy
    else:
        yaw = _quat_to_yaw(odom.pose.pose.orientation) if odom is not None else 0.0
        roll = 0.0
        pitch = 0.0
    return Pose2D(x=x, y=y, roll=roll, pitch=pitch, yaw=yaw)


def map_loop(max_points: int, voxel_size: float):
    global last_lidar_points
    last_processed = 0.0
    while True:
        if not mapping_enabled:
            time.sleep(0.05)
            continue
        with state_lock:
            cloud = last_lidar_points
            cloud_ts = last_lidar_points_ts
        if cloud is None or map_builder is None:
            time.sleep(0.05)
            continue
        if cloud_ts <= last_processed:
            time.sleep(0.02)
            continue
        pose = _get_pose()
        if pose is None:
            time.sleep(0.05)
            continue
        points = _iter_points(cloud, max_points, voxel_size)
        if points:
            map_builder.update(pose, points)
            last_processed = cloud_ts
        time.sleep(0.05)


def _write_pgm(path, width, height, data):
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    pixels = bytearray(width * height)
    for y in range(height):
        src_y = height - 1 - y
        row_start = src_y * width
        for x in range(width):
            occ = data[row_start + x]
            if occ == 255:
                val = 205
            else:
                occ = max(0, min(100, int(occ)))
                val = int(round(255 - (occ * 255.0 / 100.0)))
            pixels[y * width + x] = val
    with open(path, "wb") as handle:
        handle.write(header)
        handle.write(pixels)


def _save_map_snapshot(name):
    if map_builder is None:
        return None, "map builder not initialized"
    width = map_builder.width
    height = map_builder.height
    if width <= 0 or height <= 0:
        return None, "map has invalid dimensions"
    out_dir = os.path.join(os.getcwd(), "maps")
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, f"{name}.pgm")
    yaml_path = os.path.join(out_dir, f"{name}.yaml")
    now = time.time()
    data = list(map_builder.grid)
    ages = list(map_builder.grid_age)
    occ = []
    for v, age in zip(data, ages):
        if v < 0:
            occ.append(255)
            continue
        if map_builder.decay_sec > 0 and (now - age) > map_builder.decay_sec:
            occ.append(255)
            continue
        occ.append(int(v))
    _write_pgm(pgm_path, width, height, occ)
    yaml_text = "\n".join(
        [
            f"image: {os.path.basename(pgm_path)}",
            f"resolution: {map_builder.resolution}",
            f"origin: [{map_builder.origin_x}, {map_builder.origin_y}, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
        ]
    )
    with open(yaml_path, "w", encoding="ascii") as handle:
        handle.write(yaml_text)
    return {
        "pgm": pgm_path,
        "yaml": yaml_path,
        "age_sec": max(0.0, time.time() - map_builder.last_update) if map_builder.last_update else 0.0,
        "width": width,
        "height": height,
    }, None


def _set_lidar_switch(status: str):
    global lidar_switch_pub
    if lidar_switch_pub is None:
        return -1
    msg = String_(status)
    lidar_switch_pub.Write(msg)
    return 0


def _handle_cmd(name, params):
    if name == "lidar_on":
        return {"code": _set_lidar_switch("ON")}
    if name == "lidar_off":
        return {"code": _set_lidar_switch("OFF")}
    if name == "mapping_start":
        global mapping_enabled
        mapping_enabled = True
        return {"code": 0, "mapping": True}
    if name == "mapping_stop":
        mapping_enabled = False
        return {"code": 0, "mapping": False}
    if name == "mapping_reset":
        if map_builder is None:
            return {"code": -1, "error": "map builder not initialized"}
        map_builder.reset()
        return {"code": 0}
    if sport_client is None:
        return {"code": -1, "error": "sport client not initialized"}
    if name == "stand_up":
        return {"code": sport_client.StandUp()}
    if name == "stand_down":
        return {"code": sport_client.StandDown()}
    if name == "stop":
        return {"code": sport_client.StopMove()}
    if name == "free_walk":
        return {"code": sport_client.FreeWalk()}
    if name == "move":
        vx = float(params.get("vx", 0.0))
        vy = float(params.get("vy", 0.0))
        vyaw = float(params.get("vyaw", 0.0))
        return {"code": sport_client.Move(vx, vy, vyaw)}
    return {"code": -1, "error": f"unknown cmd: {name}"}


def _downsample_grid(max_size: int):
    if map_builder is None:
        return None
    width = map_builder.width
    height = map_builder.height
    if width <= 0 or height <= 0:
        return None
    scale = max(1, int(max(width, height) / max_size))
    out_w = max(1, width // scale)
    out_h = max(1, height // scale)
    data = []
    now = time.time()
    for oy in range(out_h):
        for ox in range(out_w):
            vals = []
            ages = []
            for dy in range(scale):
                for dx in range(scale):
                    ix = ox * scale + dx
                    iy = oy * scale + dy
                    if ix >= width or iy >= height:
                        continue
                    v = map_builder.grid[iy * width + ix]
                    a = map_builder.grid_age[iy * width + ix]
                    if v >= 0:
                        vals.append(v)
                        ages.append(a)
            if not vals:
                data.append(255)
            else:
                if map_builder.decay_sec > 0 and ages:
                    newest = max(ages)
                    if now - newest > map_builder.decay_sec:
                        data.append(255)
                        continue
                occ = sum(vals) / len(vals)
                data.append(int(round(255 - (occ * 255.0 / 100.0))))
    return {"width": out_w, "height": out_h, "scale": scale, "data": data}


class SlamUiServer(BaseHTTPRequestHandler):
    def _send_json(self, payload, code=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        params = urllib.parse.parse_qs(parsed.query)
        params = {k: v[-1] for k, v in params.items()}

        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
            return
        if path == "/api/status":
            with state_lock:
                lidar = _serialize_lidar(last_lidar_state)
                odom = _serialize_odom(last_odom)
                sport = _serialize_sportstate(last_sportstate)
                map_info = _serialize_map_info()
            self._send_json(
                {
                    "lidar": lidar,
                    "odom": odom,
                    "sportstate": sport,
                    "map": map_info,
                    "topics": {
                        "lidar_state": TOPIC_LIDAR_STATE,
                        "lidar_points": TOPIC_LIDAR_POINTS,
                        "odom": TOPIC_ODOM,
                    },
                }
            )
            return
        if path == "/api/lidar_points":
            with state_lock:
                points = _extract_points_xy(last_lidar_points, 1500, MAP_VOXEL_SIZE)
            self._send_json(points or {})
            return
        if path == "/api/map_grid":
            grid = _downsample_grid(300)
            self._send_json(grid or {})
            return
        if path == "/api/cmd":
            name = params.get("name", "")
            self._send_json(_handle_cmd(name, params))
            return
        if path == "/api/save_map":
            name = params.get("name") or time.strftime("map_%Y%m%d_%H%M%S")
            result, err = _save_map_snapshot(name)
            if err:
                self._send_json({"error": err}, code=400)
            else:
                self._send_json(result)
            return

        self.send_error(404, "not found")


HTML_PAGE = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Go2 SLAM Webapp</title>
  <style>
    body { font-family: "Trebuchet MS", sans-serif; background: #0e1116; color: #e6e9ef; margin: 0; }
    header { padding: 18px 28px; background: #151a22; border-bottom: 1px solid #222a35; }
    main { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; padding: 18px 28px; }
    section { background: #121723; border: 1px solid #1f2732; border-radius: 12px; padding: 16px; }
    h1 { margin: 0; font-size: 20px; letter-spacing: 0.5px; }
    h2 { margin: 0 0 12px; font-size: 16px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; }
    button { background: #2a3241; color: #e6e9ef; border: 1px solid #3a465a; padding: 8px 12px; border-radius: 8px; cursor: pointer; }
    button.primary { background: #3178ff; border-color: #3f84ff; }
    button.warn { background: #ff6a4a; border-color: #ff7c61; }
    .kv { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px solid #1f2732; font-size: 13px; }
    .kv:last-child { border-bottom: none; }
    input { background: #0c111a; border: 1px solid #2b3342; color: #e6e9ef; padding: 8px 10px; border-radius: 8px; }
    .wide { grid-column: 1 / -1; }
    .status { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; font-size: 12px; }
  </style>
</head>
<body>
  <header>
    <h1>Go2 SLAM Webapp</h1>
  </header>
  <main>
    <section>
      <h2>SLAM Status</h2>
      <div class="kv"><span>Map Source</span><strong id="map_topic">--</strong></div>
      <div class="kv"><span>Map Info</span><strong id="map_info">--</strong></div>
      <div class="kv"><span>Map Age</span><strong id="map_age">--</strong></div>
      <div class="kv"><span>LiDAR Topic</span><strong id="lidar_topic">--</strong></div>
      <div class="kv"><span>LiDAR Points</span><strong id="lidar_points_topic">--</strong></div>
      <div class="kv"><span>LiDAR Cloud</span><strong id="lidar_cloud">--</strong></div>
      <div class="kv"><span>LiDAR Loss</span><strong id="lidar_loss">--</strong></div>
      <div class="kv"><span>Map Updates</span><strong id="map_updates">--</strong></div>
      <div class="kv"><span>Last Points</span><strong id="map_points">--</strong></div>
      <div class="kv"><span>Mapping</span><strong id="map_running">--</strong></div>
    </section>
    <section>
      <h2>Drive Controls</h2>
      <div class="row">
        <button onclick="cmd('stand_up')" class="primary">Stand Up</button>
        <button onclick="cmd('stand_down')" class="warn">Stand Down</button>
        <button onclick="cmd('stop')" class="warn">Stop</button>
        <button onclick="cmd('free_walk')" class="primary">Free Walk</button>
      </div>
      <div class="row" style="margin-top: 10px;">
        <button onclick="move(0.3,0,0)">Forward</button>
        <button onclick="move(-0.3,0,0)">Back</button>
        <button onclick="move(0,0.3,0)">Left</button>
        <button onclick="move(0,-0.3,0)">Right</button>
        <button onclick="move(0,0,0.5)">Rotate</button>
      </div>
      <div class="kv"><span>Command</span><strong id="cmd_result">--</strong></div>
    </section>
    <section>
      <h2>LiDAR</h2>
      <div class="row">
        <button onclick="cmd('lidar_on')" class="primary">LiDAR ON</button>
        <button onclick="cmd('lidar_off')" class="warn">LiDAR OFF</button>
      </div>
      <canvas id="lidar_canvas" width="360" height="260" style="width:100%; margin-top:10px; background:#0b0f18; border-radius:10px; border:1px solid #222a3a;"></canvas>
    </section>
    <section>
      <h2>Mapping</h2>
      <div class="row">
        <button onclick="cmd('mapping_start')" class="primary">Start Mapping</button>
        <button onclick="cmd('mapping_stop')" class="warn">Stop Mapping</button>
        <button onclick="cmd('mapping_reset')">Reset Map</button>
      </div>
      <canvas id="map_canvas" width="360" height="260" style="width:100%; margin-top:10px; background:#0b0f18; border-radius:10px; border:1px solid #222a3a;"></canvas>
    </section>
    <section>
      <h2>Save Map</h2>
      <div class="row">
        <input id="map_name" placeholder="map name (optional)" />
        <button onclick="saveMap()" class="primary">Save Map</button>
      </div>
      <div class="status" id="save_status">--</div>
    </section>
    <section class="wide">
      <h2>Robot State</h2>
      <div class="kv"><span>Odom Pos</span><strong id="odom_pos">--</strong></div>
      <div class="kv"><span>Odom Vel</span><strong id="odom_vel">--</strong></div>
      <div class="kv"><span>Sport Pos</span><strong id="sport_pos">--</strong></div>
      <div class="kv"><span>Sport Vel</span><strong id="sport_vel">--</strong></div>
    </section>
  </main>
  <script>
    function cmd(name) {
      fetch(`/api/cmd?name=${encodeURIComponent(name)}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d))
        .catch(() => document.getElementById('cmd_result').textContent = 'error');
    }
    function move(vx, vy, vyaw) {
      fetch(`/api/cmd?name=move&vx=${vx}&vy=${vy}&vyaw=${vyaw}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d))
        .catch(() => document.getElementById('cmd_result').textContent = 'error');
    }
    function saveMap() {
      const name = document.getElementById('map_name').value.trim();
      const url = name ? `/api/save_map?name=${encodeURIComponent(name)}` : '/api/save_map';
      fetch(url)
        .then(r => r.json())
        .then(d => document.getElementById('save_status').textContent = JSON.stringify(d))
        .catch(() => document.getElementById('save_status').textContent = 'error');
    }
    function update() {
      fetch('/api/status')
        .then(r => r.json())
        .then(d => {
          const map = d.map || {};
          const lidar = d.lidar || {};
          const odom = d.odom || {};
          const sport = d.sportstate || {};
          const topics = d.topics || {};
          document.getElementById('map_topic').textContent = 'lidar+odom+imu';
          document.getElementById('lidar_topic').textContent = topics.lidar_state || '--';
          document.getElementById('lidar_points_topic').textContent = topics.lidar_points || '--';
          document.getElementById('map_info').textContent = map.width ? `${map.width}x${map.height} @${map.resolution}` : '--';
          document.getElementById('map_age').textContent = map.age_sec ? `${map.age_sec.toFixed(1)}s` : '--';
          document.getElementById('lidar_cloud').textContent = lidar.cloud_size ?? '--';
          document.getElementById('lidar_loss').textContent = lidar.cloud_loss ?? '--';
          document.getElementById('map_updates').textContent = map.updates ?? '--';
          document.getElementById('map_points').textContent = map.last_points ?? '--';
          document.getElementById('map_running').textContent = map.mapping ? 'ON' : 'OFF';
          document.getElementById('odom_pos').textContent = odom.pos ? odom.pos.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('odom_vel').textContent = odom.lin_vel ? odom.lin_vel.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('sport_pos').textContent = sport.position ? sport.position.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('sport_vel').textContent = sport.velocity ? sport.velocity.map(v => v.toFixed(2)).join(', ') : '--';
        })
        .catch(() => {});
    }
    function drawLidar() {
      fetch('/api/lidar_points')
        .then(r => r.json())
        .then(d => {
          const canvas = document.getElementById('lidar_canvas');
          const ctx = canvas.getContext('2d');
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.fillStyle = '#0b0f18';
          ctx.fillRect(0, 0, canvas.width, canvas.height);
          const pts = d.points || [];
          if (!pts.length) return;
          const scale = Math.min(canvas.width, canvas.height) * 0.45;
          const cx = canvas.width / 2;
          const cy = canvas.height / 2;
          ctx.fillStyle = '#8bd5ff';
          for (let i = 0; i < pts.length; i++) {
            const x = pts[i][0];
            const y = pts[i][1];
            const px = cx + x * scale;
            const py = cy - y * scale;
            if (px < 0 || py < 0 || px >= canvas.width || py >= canvas.height) continue;
            ctx.fillRect(px, py, 2, 2);
          }
        })
        .catch(() => {});
    }
    function drawMap() {
      fetch('/api/map_grid')
        .then(r => r.json())
        .then(d => {
          const canvas = document.getElementById('map_canvas');
          const ctx = canvas.getContext('2d');
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.fillStyle = '#0b0f18';
          ctx.fillRect(0, 0, canvas.width, canvas.height);
          if (!d.data || !d.width || !d.height) return;
          const img = ctx.createImageData(d.width, d.height);
          for (let i = 0; i < d.data.length; i++) {
            const v = d.data[i];
            img.data[i * 4 + 0] = v;
            img.data[i * 4 + 1] = v;
            img.data[i * 4 + 2] = v;
            img.data[i * 4 + 3] = 255;
          }
          const scaleX = canvas.width / d.width;
          const scaleY = canvas.height / d.height;
          ctx.save();
          ctx.imageSmoothingEnabled = false;
          ctx.scale(scaleX, scaleY);
          ctx.putImageData(img, 0, 0);
          ctx.restore();
        })
        .catch(() => {});
    }
    setInterval(update, 500);
    setInterval(drawLidar, 200);
    setInterval(drawMap, 500);
    update();
    drawLidar();
    drawMap();
  </script>
</body>
</html>
"""


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default=INTERFACE)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--lidar-state-topic", default=TOPIC_LIDAR_STATE)
    parser.add_argument("--lidar-points-topic", default=TOPIC_LIDAR_POINTS)
    parser.add_argument("--odom-topic", default=TOPIC_ODOM)
    parser.add_argument("--sportstate-topic", default=TOPIC_SPORTSTATE)
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--map-width-m", type=float, default=20.0)
    parser.add_argument("--map-height-m", type=float, default=20.0)
    parser.add_argument("--map-max-range", type=float, default=15.0)
    parser.add_argument("--map-min-range", type=float, default=0.3)
    parser.add_argument("--map-z-min", type=float, default=-0.4)
    parser.add_argument("--map-z-max", type=float, default=1.2)
    parser.add_argument("--map-max-points", type=int, default=3000)
    parser.add_argument("--map-voxel-size", type=float, default=0.05)
    parser.add_argument("--map-decay-sec", type=float, default=8.0)
    args = parser.parse_args()

    INTERFACE = args.iface
    TOPIC_LIDAR_STATE = args.lidar_state_topic
    TOPIC_LIDAR_POINTS = args.lidar_points_topic
    TOPIC_ODOM = args.odom_topic
    TOPIC_SPORTSTATE = args.sportstate_topic
    MAP_VOXEL_SIZE = args.map_voxel_size

    ChannelFactoryInitialize(0, INTERFACE)

    lidar_sub = ChannelSubscriber(TOPIC_LIDAR_STATE, LidarState_)
    lidar_sub.Init(lidar_state_cb, 10)
    lidar_points_sub = ChannelSubscriber(TOPIC_LIDAR_POINTS, PointCloud2_)
    lidar_points_sub.Init(lidar_points_cb, 10)
    odom_sub = ChannelSubscriber(TOPIC_ODOM, Odometry_)
    odom_sub.Init(odom_cb, 10)
    sport_sub = ChannelSubscriber(TOPIC_SPORTSTATE, SportModeState_)
    sport_sub.Init(sportstate_cb, 10)
    low_sub = ChannelSubscriber("rt/lowstate", LowState_)
    low_sub.Init(lowstate_cb, 10)

    lidar_switch_pub = ChannelPublisher(TOPIC_LIDAR_SWITCH, String_)
    lidar_switch_pub.Init()

    sport_client = SportClient()
    sport_client.SetTimeout(5.0)
    sport_client.Init()

    map_builder = MapBuilder(
        resolution=args.map_resolution,
        width_m=args.map_width_m,
        height_m=args.map_height_m,
        min_range=args.map_min_range,
        max_range=args.map_max_range,
        z_min=args.map_z_min,
        z_max=args.map_z_max,
        decay_sec=args.map_decay_sec,
    )

    map_thread = threading.Thread(
        target=map_loop, args=(args.map_max_points, args.map_voxel_size), daemon=True
    )
    map_thread.start()

    server = ThreadingHTTPServer((HOST, args.port), SlamUiServer)
    print(f"Go2 SLAM webapp on http://{HOST}:{args.port} (iface={INTERFACE})")
    server.serve_forever()
