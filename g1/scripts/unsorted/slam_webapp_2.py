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
PORT = 8020
INTERFACE = "enp2s0"

state_lock = threading.Lock()
last_lidar_state = None
last_lidar_points = None
last_lidar_points_ts = 0.0
last_odom = None
last_sportstate = None
last_imu_rpy = None
last_imu_gyro = None

lidar_switch_pub = None
sport_client = None
mapping_enabled = False


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    roll: float = 0.0
    pitch: float = 0.0


class EKF2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.P = [
            [0.5, 0.0, 0.0],
            [0.0, 0.5, 0.0],
            [0.0, 0.0, 0.2],
        ]

    def predict(self, dx, dy, dyaw, q_pos=0.05, q_yaw=0.03):
        self.x += dx
        self.y += dy
        self.yaw = _wrap_angle(self.yaw + dyaw)
        self.P[0][0] += q_pos
        self.P[1][1] += q_pos
        self.P[2][2] += q_yaw

    def update(self, z_x, z_y, z_yaw, r_pos=0.1, r_yaw=0.08):
        H = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        R = [[r_pos, 0.0, 0.0], [0.0, r_pos, 0.0], [0.0, 0.0, r_yaw]]
        z = [z_x, z_y, z_yaw]
        x = [self.x, self.y, self.yaw]
        y = [z[0] - x[0], z[1] - x[1], _wrap_angle(z[2] - x[2])]
        S = _mat_add(self.P, R)
        K = _mat_mul(self.P, _mat_inv(S))
        dx = _mat_vec_mul(K, y)
        self.x += dx[0]
        self.y += dx[1]
        self.yaw = _wrap_angle(self.yaw + dx[2])
        I = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.P = _mat_mul(_mat_sub(I, K), self.P)

    def pose(self):
        return Pose2D(self.x, self.y, self.yaw)


class LogOddsMap:
    def __init__(
        self,
        resolution,
        width_m,
        height_m,
        l_occ=0.85,
        l_free=0.4,
        l_min=-2.0,
        l_max=3.5,
        decay_sec=10.0,
    ):
        self.resolution = float(resolution)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.width = max(1, int(round(self.width_m / self.resolution)))
        self.height = max(1, int(round(self.height_m / self.resolution)))
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.decay_sec = float(decay_sec)
        self.log_odds = [0.0] * (self.width * self.height)
        self.age = [0.0] * (self.width * self.height)
        self.updates = 0
        self.last_points = 0
        self.last_update = 0.0

    def reset(self):
        self.log_odds = [0.0] * (self.width * self.height)
        self.age = [0.0] * (self.width * self.height)
        self.updates = 0
        self.last_points = 0
        self.last_update = 0.0

    def _idx(self, ix, iy):
        return iy * self.width + ix

    def world_to_grid(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return ix, iy

    def _update_cell(self, ix, iy, delta):
        idx = self._idx(ix, iy)
        val = self.log_odds[idx] + delta
        if val > self.l_max:
            val = self.l_max
        if val < self.l_min:
            val = self.l_min
        self.log_odds[idx] = val
        self.age[idx] = time.time()

    def _raytrace(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if x == x1 and y == y1:
                break
            self._update_cell(x, y, -self.l_free)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def insert(self, pose, points):
        origin = self.world_to_grid(pose.x, pose.y)
        if origin is None:
            return
        ox, oy = origin
        added = 0
        for wx, wy in points:
            cell = self.world_to_grid(wx, wy)
            if cell is None:
                continue
            ix, iy = cell
            self._raytrace(ox, oy, ix, iy)
            self._update_cell(ix, iy, self.l_occ)
            added += 1
        if added:
            self.updates += 1
            self.last_points = added
            self.last_update = time.time()

    def to_image(self, max_size):
        if self.width <= 0 or self.height <= 0:
            return None
        scale = max(1, int(max(self.width, self.height) / max_size))
        out_w = max(1, self.width // scale)
        out_h = max(1, self.height // scale)
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
                        if ix >= self.width or iy >= self.height:
                            continue
                        idx = self._idx(ix, iy)
                        vals.append(self.log_odds[idx])
                        ages.append(self.age[idx])
                if not vals:
                    data.append(255)
                    continue
                newest = max(ages) if ages else 0.0
                if self.decay_sec > 0 and newest > 0 and (now - newest) > self.decay_sec:
                    data.append(255)
                    continue
                avg = sum(vals) / len(vals)
                prob = 1.0 - 1.0 / (1.0 + math.exp(avg))
                val = int(round(255 - (prob * 255.0)))
                data.append(val)
        return {"width": out_w, "height": out_h, "data": data}


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
    global last_imu_rpy, last_imu_gyro
    with state_lock:
        last_imu_rpy = [float(v) for v in msg.imu_state.rpy]
        last_imu_gyro = [float(v) for v in msg.imu_state.gyroscope]


def _wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def _mat_sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(3)] for i in range(3)]


def _mat_mul(a, b):
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _mat_vec_mul(a, v):
    return [
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    ]


def _mat_inv(a):
    det = (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )
    if abs(det) < 1e-9:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    inv_det = 1.0 / det
    return [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
        ],
    ]


def _extract_points(msg, max_points, voxel_size):
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
    vox = float(voxel_size) if voxel_size and voxel_size > 0 else 0.0
    voxels = {} if vox > 0 else None
    points = []
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


def _filter_and_level(points, min_range, max_range, z_min, z_max, roll, pitch):
    filtered = []
    cos_r = math.cos(roll)
    sin_r = math.sin(roll)
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)
    for x, y, z in points:
        rng = math.hypot(x, y)
        if rng < min_range or rng > max_range:
            continue
        if z < z_min or z > z_max:
            continue
        y1 = y * cos_r - z * sin_r
        z1 = y * sin_r + z * cos_r
        x2 = x * cos_p + z1 * sin_p
        y2 = y1
        filtered.append((x2, y2))
    return filtered


def _transform_points(points, pose):
    cos_y = math.cos(pose.yaw)
    sin_y = math.sin(pose.yaw)
    out = []
    for x, y in points:
        wx = x * cos_y - y * sin_y + pose.x
        wy = x * sin_y + y * cos_y + pose.y
        out.append((wx, wy))
    return out


def _icp_2d(src, tgt, max_iter=10, tol=1e-3):
    if not src or not tgt:
        return 0.0, 0.0, 0.0, 1e9
    src_pts = [list(p) for p in src]
    total_err = 0.0
    dx = dy = angle = 0.0
    for _ in range(max_iter):
        pairs = []
        total_err = 0.0
        for sx, sy in src_pts:
            best = None
            best_d = 1e9
            for tx, ty in tgt:
                dx = sx - tx
                dy = sy - ty
                d = dx * dx + dy * dy
                if d < best_d:
                    best_d = d
                    best = (tx, ty)
            if best is not None:
                pairs.append(((sx, sy), best))
                total_err += best_d
        if not pairs:
            break
        csx = sum(p[0][0] for p in pairs) / len(pairs)
        csy = sum(p[0][1] for p in pairs) / len(pairs)
        ctx = sum(p[1][0] for p in pairs) / len(pairs)
        cty = sum(p[1][1] for p in pairs) / len(pairs)
        sxx = sxy = syx = syy = 0.0
        for (sx, sy), (tx, ty) in pairs:
            sx -= csx
            sy -= csy
            tx -= ctx
            ty -= cty
            sxx += sx * tx
            sxy += sx * ty
            syx += sy * tx
            syy += sy * ty
        angle = math.atan2(sxy - syx, sxx + syy)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        dx = ctx - (csx * cos_a - csy * sin_a)
        dy = cty - (csx * sin_a + csy * cos_a)
        for i, (sx, sy) in enumerate(src_pts):
            x = sx * cos_a - sy * sin_a + dx
            y = sx * sin_a + sy * cos_a + dy
            src_pts[i] = [x, y]
        if total_err / max(1, len(pairs)) < tol:
            break
    rms = math.sqrt(total_err / max(1, len(src_pts)))
    # Compute the final transform from original src to aligned src_pts.
    # Use centroids for translation; angle from last iteration.
    # This is approximate but useful for correction.
    return dx, dy, angle, rms


def _serialize_lidar(msg: LidarState_):
    if msg is None:
        return None
    return {
        "cloud_size": int(msg.cloud_size),
        "cloud_loss": float(msg.cloud_packet_loss_rate),
        "error": int(msg.error_state),
    }


def _serialize_status(mapper, ekf, loop_hits, mapping):
    pose = ekf.pose() if ekf else None
    return {
        "mapping": mapping,
        "pose": [pose.x, pose.y, pose.yaw] if pose else None,
        "updates": mapper.updates if mapper else 0,
        "last_points": mapper.last_points if mapper else 0,
        "loop_hits": loop_hits,
        "age_sec": max(0.0, time.time() - mapper.last_update) if mapper and mapper.last_update else 0.0,
    }


def _save_map_snapshot(mapper, name):
    if mapper is None:
        return None, "map builder not initialized"
    out_dir = os.path.join(os.getcwd(), "maps")
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, f"{name}.pgm")
    yaml_path = os.path.join(out_dir, f"{name}.yaml")
    now = time.time()
    data = []
    for val, age in zip(mapper.log_odds, mapper.age):
        if age <= 0:
            data.append(255)
            continue
        if mapper.decay_sec > 0 and now - age > mapper.decay_sec:
            data.append(255)
            continue
        prob = 1.0 - 1.0 / (1.0 + math.exp(val))
        data.append(int(round(255 - (prob * 255.0))))
    _write_pgm(pgm_path, mapper.width, mapper.height, data)
    yaml_text = "\n".join(
        [
            f"image: {os.path.basename(pgm_path)}",
            f"resolution: {mapper.resolution}",
            f"origin: [{mapper.origin_x}, {mapper.origin_y}, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.196",
        ]
    )
    with open(yaml_path, "w", encoding="ascii") as handle:
        handle.write(yaml_text)
    return {"pgm": pgm_path, "yaml": yaml_path}, None


def _write_pgm(path, width, height, data):
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    pixels = bytearray(width * height)
    for y in range(height):
        src_y = height - 1 - y
        row_start = src_y * width
        for x in range(width):
            pixels[y * width + x] = int(data[row_start + x])
    with open(path, "wb") as handle:
        handle.write(header)
        handle.write(pixels)


class SlamServer(BaseHTTPRequestHandler):
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
            self._send_json(
                {
                    "lidar": lidar,
                    "slam": _serialize_status(MAPPER, EKF, LOOP_HITS, mapping_enabled),
                }
            )
            return
        if path == "/api/cmd":
            name = params.get("name", "")
            self._send_json(_handle_cmd(name, params))
            return
        if path == "/api/lidar_points":
            with state_lock:
                points = _extract_points(last_lidar_points, 1200, MAP_VOXEL_SIZE)
                imu = last_imu_rpy
            if imu:
                pts = _filter_and_level(points, MAP_MIN_RANGE, MAP_MAX_RANGE, MAP_Z_MIN, MAP_Z_MAX, imu[0], imu[1])
            else:
                pts = [(x, y) for x, y, _ in points]
            self._send_json({"points": pts})
            return
        if path == "/api/map_grid":
            grid = MAPPER.to_image(320) if MAPPER else None
            self._send_json(grid or {})
            return
        if path == "/api/save_map":
            name = params.get("name") or time.strftime("map_%Y%m%d_%H%M%S")
            result, err = _save_map_snapshot(MAPPER, name)
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
  <title>Go2 SLAM Webapp 2</title>
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
    canvas { width: 100%; background:#0b0f18; border-radius:10px; border:1px solid #222a3a; }
  </style>
</head>
<body>
  <header>
    <h1>Go2 SLAM Webapp 2</h1>
  </header>
  <main>
    <section>
      <h2>SLAM Status</h2>
      <div class="kv"><span>Mapping</span><strong id="slam_mapping">--</strong></div>
      <div class="kv"><span>Pose</span><strong id="slam_pose">--</strong></div>
      <div class="kv"><span>Updates</span><strong id="slam_updates">--</strong></div>
      <div class="kv"><span>Last Points</span><strong id="slam_points">--</strong></div>
      <div class="kv"><span>Loop Hits</span><strong id="slam_loop">--</strong></div>
      <div class="kv"><span>Age</span><strong id="slam_age">--</strong></div>
    </section>
    <section>
      <h2>Drive + Mapping</h2>
      <div class="row">
        <button onclick="cmd('mapping_start')" class="primary">Start Mapping</button>
        <button onclick="cmd('mapping_stop')" class="warn">Stop Mapping</button>
        <button onclick="cmd('mapping_reset')">Reset Map</button>
      </div>
      <div class="row" style="margin-top:10px;">
        <button onclick="cmd('free_walk')" class="primary">Free Walk</button>
        <button onclick="cmd('stop')" class="warn">Stop</button>
      </div>
    </section>
    <section>
      <h2>LiDAR</h2>
      <div class="row">
        <button onclick="cmd('lidar_on')" class="primary">LiDAR ON</button>
        <button onclick="cmd('lidar_off')" class="warn">LiDAR OFF</button>
      </div>
      <canvas id="lidar_canvas" width="360" height="260" style="margin-top:10px;"></canvas>
    </section>
    <section>
      <h2>Map View</h2>
      <canvas id="map_canvas" width="360" height="260" style="margin-top:10px;"></canvas>
    </section>
    <section>
      <h2>Save Map</h2>
      <div class="row">
        <input id="map_name" placeholder="map name (optional)" />
        <button onclick="saveMap()" class="primary">Save Map</button>
      </div>
      <div class="kv"><span>Result</span><strong id="save_status">--</strong></div>
    </section>
  </main>
  <script>
    function cmd(name) {
      fetch(`/api/cmd?name=${encodeURIComponent(name)}`)
        .then(r => r.json())
        .then(d => document.getElementById('save_status').textContent = JSON.stringify(d))
        .catch(() => document.getElementById('save_status').textContent = 'error');
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
          const slam = d.slam || {};
          const pose = slam.pose || [];
          document.getElementById('slam_mapping').textContent = slam.mapping ? 'ON' : 'OFF';
          document.getElementById('slam_pose').textContent = pose.length ? pose.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('slam_updates').textContent = slam.updates ?? '--';
          document.getElementById('slam_points').textContent = slam.last_points ?? '--';
          document.getElementById('slam_loop').textContent = slam.loop_hits ?? '--';
          document.getElementById('slam_age').textContent = slam.age_sec ? `${slam.age_sec.toFixed(1)}s` : '--';
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
    setInterval(drawMap, 600);
    update();
    drawLidar();
    drawMap();
  </script>
</body>
</html>
"""


def _handle_cmd(name, params):
    global mapping_enabled
    if name == "lidar_on":
        return {"code": _set_lidar_switch("ON")}
    if name == "lidar_off":
        return {"code": _set_lidar_switch("OFF")}
    if name == "mapping_start":
        mapping_enabled = True
        return {"code": 0, "mapping": True}
    if name == "mapping_stop":
        mapping_enabled = False
        return {"code": 0, "mapping": False}
    if name == "mapping_reset":
        if MAPPER:
            MAPPER.reset()
        return {"code": 0}
    if sport_client is None:
        return {"code": -1, "error": "sport client not initialized"}
    if name == "free_walk":
        return {"code": sport_client.FreeWalk()}
    if name == "stop":
        return {"code": sport_client.StopMove()}
    return {"code": -1, "error": f"unknown cmd: {name}"}


def _set_lidar_switch(status):
    if lidar_switch_pub is None:
        return -1
    msg = String_(status)
    lidar_switch_pub.Write(msg)
    return 0


def _pose_from_sources():
    with state_lock:
        odom = last_odom
        sport = last_sportstate
        imu = last_imu_rpy
    if odom is not None:
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)
        yaw = _quat_to_yaw(odom.pose.pose.orientation)
    elif sport is not None and len(sport.position) >= 2:
        x = float(sport.position[0])
        y = float(sport.position[1])
        yaw = 0.0
    else:
        return None
    roll = imu[0] if imu else 0.0
    pitch = imu[1] if imu else 0.0
    return Pose2D(x, y, yaw, roll, pitch)


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def slam_loop():
    global LOOP_HITS
    last_pose = None
    last_scan_world = None
    keyframes = []
    last_cloud_ts = 0.0
    while True:
        if not mapping_enabled:
            time.sleep(0.05)
            continue
        with state_lock:
            cloud = last_lidar_points
            cloud_ts = last_lidar_points_ts
            imu = last_imu_rpy
        if cloud is None or cloud_ts <= last_cloud_ts:
            time.sleep(0.05)
            continue
        base_pose = _pose_from_sources()
        if base_pose is None or imu is None:
            time.sleep(0.05)
            continue
        points = _extract_points(cloud, MAP_MAX_POINTS, MAP_VOXEL_SIZE)
        leveled = _filter_and_level(
            points, MAP_MIN_RANGE, MAP_MAX_RANGE, MAP_Z_MIN, MAP_Z_MAX, base_pose.roll, base_pose.pitch
        )
        if not leveled:
            last_cloud_ts = cloud_ts
            continue
        if last_pose is None:
            EKF.x, EKF.y, EKF.yaw = base_pose.x, base_pose.y, base_pose.yaw
        else:
            dx = base_pose.x - last_pose.x
            dy = base_pose.y - last_pose.y
            dyaw = _wrap_angle(base_pose.yaw - last_pose.yaw)
            EKF.predict(dx, dy, dyaw)
        pred_pose = EKF.pose()
        pred_pose.roll = base_pose.roll
        pred_pose.pitch = base_pose.pitch
        scan_world = _transform_points(leveled, pred_pose)
        if last_scan_world:
            dx, dy, dyaw, err = _icp_2d(scan_world, last_scan_world)
            if err < ICP_MAX_ERROR:
                corr_pose = Pose2D(pred_pose.x + dx, pred_pose.y + dy, _wrap_angle(pred_pose.yaw + dyaw))
                EKF.update(corr_pose.x, corr_pose.y, corr_pose.yaw)
                pred_pose = EKF.pose()
        scan_world = _transform_points(leveled, pred_pose)
        MAPPER.insert(pred_pose, scan_world)
        if len(keyframes) < MAX_KEYFRAMES or (
            last_pose and math.hypot(pred_pose.x - last_pose.x, pred_pose.y - last_pose.y) > KEYFRAME_DIST
        ):
            keyframes.append((pred_pose, scan_world))
            if len(keyframes) > MAX_KEYFRAMES:
                keyframes.pop(0)
        for k_pose, k_scan in keyframes[:-1]:
            if math.hypot(pred_pose.x - k_pose.x, pred_pose.y - k_pose.y) < LOOP_DIST:
                dx, dy, dyaw, err = _icp_2d(scan_world, k_scan)
                if err < LOOP_MAX_ERROR:
                    EKF.update(pred_pose.x + dx, pred_pose.y + dy, _wrap_angle(pred_pose.yaw + dyaw))
                    LOOP_HITS += 1
                break
        last_pose = pred_pose
        last_scan_world = scan_world
        last_cloud_ts = cloud_ts
        time.sleep(0.02)


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
    parser.add_argument("--map-width-m", type=float, default=12.0)
    parser.add_argument("--map-height-m", type=float, default=12.0)
    parser.add_argument("--map-max-range", type=float, default=8.0)
    parser.add_argument("--map-min-range", type=float, default=0.6)
    parser.add_argument("--map-z-min", type=float, default=-0.2)
    parser.add_argument("--map-z-max", type=float, default=0.7)
    parser.add_argument("--map-max-points", type=int, default=1200)
    parser.add_argument("--map-voxel-size", type=float, default=0.08)
    parser.add_argument("--map-decay-sec", type=float, default=8.0)
    parser.add_argument("--icp-max-error", type=float, default=0.6)
    parser.add_argument("--loop-dist", type=float, default=1.0)
    parser.add_argument("--loop-max-error", type=float, default=0.5)
    parser.add_argument("--max-keyframes", type=int, default=20)
    parser.add_argument("--keyframe-dist", type=float, default=0.6)
    args = parser.parse_args()

    INTERFACE = args.iface
    TOPIC_LIDAR_STATE = args.lidar_state_topic
    TOPIC_LIDAR_POINTS = args.lidar_points_topic
    TOPIC_ODOM = args.odom_topic
    TOPIC_SPORTSTATE = args.sportstate_topic

    MAP_MAX_RANGE = args.map_max_range
    MAP_MIN_RANGE = args.map_min_range
    MAP_Z_MIN = args.map_z_min
    MAP_Z_MAX = args.map_z_max
    MAP_MAX_POINTS = args.map_max_points
    MAP_VOXEL_SIZE = args.map_voxel_size
    ICP_MAX_ERROR = args.icp_max_error
    LOOP_DIST = args.loop_dist
    LOOP_MAX_ERROR = args.loop_max_error
    MAX_KEYFRAMES = args.max_keyframes
    KEYFRAME_DIST = args.keyframe_dist

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

    EKF = EKF2D()
    MAPPER = LogOddsMap(
        args.map_resolution,
        args.map_width_m,
        args.map_height_m,
        decay_sec=args.map_decay_sec,
    )
    LOOP_HITS = 0

    slam_thread = threading.Thread(target=slam_loop, daemon=True)
    slam_thread.start()

    server = ThreadingHTTPServer((HOST, args.port), SlamServer)
    print(f"Go2 SLAM webapp 2 on http://{HOST}:{args.port} (iface={INTERFACE})")
    server.serve_forever()
