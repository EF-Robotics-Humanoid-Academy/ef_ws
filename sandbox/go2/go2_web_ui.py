import json
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
    LowState_,
    SportModeState_,
    WirelessController_,
    LidarState_,
    HeightMap_,
)
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import OccupancyGrid_, Odometry_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient

TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_SPORTSTATE = "rt/sportmodestate"
TOPIC_WIRELESS = "rt/wirelesscontroller"
TOPIC_LIDAR_SWITCH = "rt/utlidar/switch"

# These topics may vary by firmware; override via CLI args if needed.
TOPIC_LIDAR_STATE = "rt/utlidar/state"
TOPIC_LIDAR_POINTS = "rt/utlidar/cloud"
TOPIC_ODOM = "rt/odom"
TOPIC_MAP = "rt/map"
TOPIC_HEIGHTMAP = "rt/heightmap"

HOST = "0.0.0.0"
PORT = 8000
INTERFACE = "enp2s0"

state_lock = threading.Lock()
last_lowstate = None
last_sportstate = None
last_wireless = None
last_lidar_state = None
last_odom = None
last_map = None
last_heightmap = None
last_lidar_points = None
last_lidar_points_ts = 0.0
last_camera_jpeg = None
last_camera_ts = 0.0
sport_client = None
lidar_switch_pub = None
low_cmd_pub = None
low_level_enabled = False
low_q = [0.0] * 20
low_kp = [0.0] * 20
low_kd = [0.0] * 20
crc = CRC()


def lowstate_cb(msg: LowState_):
    global last_lowstate
    with state_lock:
        last_lowstate = msg


def sportstate_cb(msg: SportModeState_):
    global last_sportstate
    with state_lock:
        last_sportstate = msg


def wireless_cb(msg: WirelessController_):
    global last_wireless
    with state_lock:
        last_wireless = msg


def lidar_state_cb(msg: LidarState_):
    global last_lidar_state
    with state_lock:
        last_lidar_state = msg


def odom_cb(msg: Odometry_):
    global last_odom
    with state_lock:
        last_odom = msg


def map_cb(msg: OccupancyGrid_):
    global last_map
    with state_lock:
        last_map = msg


def heightmap_cb(msg: HeightMap_):
    global last_heightmap
    with state_lock:
        last_heightmap = msg


def lidar_points_cb(msg: PointCloud2_):
    global last_lidar_points, last_lidar_points_ts
    with state_lock:
        last_lidar_points = msg
        last_lidar_points_ts = time.time()


def serialize_lowstate(msg: LowState_):
    if msg is None:
        return None
    motors = []
    for i in range(12):
        m = msg.motor_state[i]
        motors.append(
            {
                "id": i,
                "q": float(m.q),
                "dq": float(m.dq),
                "tau_est": float(m.tau_est),
                "temp": int(m.temperature),
                "mode": int(m.mode),
            }
        )
    return {
        "power_v": float(msg.power_v),
        "power_a": float(msg.power_a),
        "foot_force": [int(v) for v in msg.foot_force],
        "imu_rpy": [
            float(msg.imu_state.rpy[0]),
            float(msg.imu_state.rpy[1]),
            float(msg.imu_state.rpy[2]),
        ],
        "imu_gyro": [
            float(msg.imu_state.gyroscope[0]),
            float(msg.imu_state.gyroscope[1]),
            float(msg.imu_state.gyroscope[2]),
        ],
        "imu_acc": [
            float(msg.imu_state.accelerometer[0]),
            float(msg.imu_state.accelerometer[1]),
            float(msg.imu_state.accelerometer[2]),
        ],
        "motors": motors,
    }


def serialize_sportstate(msg: SportModeState_):
    if msg is None:
        return None
    return {
        "mode": int(msg.mode),
        "progress": float(msg.progress),
        "gait_type": int(msg.gait_type),
        "foot_raise_height": float(msg.foot_raise_height),
        "body_height": float(msg.body_height),
        "position": [float(v) for v in msg.position],
        "velocity": [float(v) for v in msg.velocity],
        "yaw_speed": float(msg.yaw_speed),
        "foot_force": [int(v) for v in msg.foot_force],
    }


def serialize_wireless(msg: WirelessController_):
    if msg is None:
        return None
    return {
        "lx": float(msg.lx),
        "ly": float(msg.ly),
        "rx": float(msg.rx),
        "ry": float(msg.ry),
        "keys": int(msg.keys),
    }


def serialize_lidar(msg: LidarState_):
    if msg is None:
        return None
    return {
        "firmware": msg.firmware_version,
        "software": msg.software_version,
        "sdk": msg.sdk_version,
        "sys_rpm": float(msg.sys_rotation_speed),
        "com_rpm": float(msg.com_rotation_speed),
        "error": int(msg.error_state),
        "cloud_freq": float(msg.cloud_frequency),
        "cloud_loss": float(msg.cloud_packet_loss_rate),
        "cloud_size": int(msg.cloud_size),
        "cloud_scan_num": int(msg.cloud_scan_num),
        "imu_freq": float(msg.imu_frequency),
        "imu_loss": float(msg.imu_packet_loss_rate),
        "imu_rpy": [float(v) for v in msg.imu_rpy],
    }


def serialize_odom(msg: Odometry_):
    if msg is None:
        return None
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    return {
        "pos": [float(p.x), float(p.y), float(p.z)],
        "quat": [float(q.x), float(q.y), float(q.z), float(q.w)],
        "lin_vel": [float(v.x), float(v.y), float(v.z)],
        "ang_vel": [float(w.x), float(w.y), float(w.z)],
        "frame": msg.child_frame_id,
    }


def serialize_map(msg: OccupancyGrid_):
    if msg is None:
        return None
    info = msg.info
    return {
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin": [
            float(info.origin.position.x),
            float(info.origin.position.y),
            float(info.origin.position.z),
        ],
        "data_len": len(msg.data),
        "frame": msg.header.frame_id,
    }


def serialize_heightmap(msg: HeightMap_):
    if msg is None:
        return None
    return {
        "resolution": float(msg.resolution),
        "width": int(msg.width),
        "height": int(msg.height),
        "origin": [float(msg.origin[0]), float(msg.origin[1])],
        "data_len": len(msg.data),
        "frame": msg.frame_id,
    }


def _extract_points_xy(msg: PointCloud2_, max_points: int = 300):
    # Best-effort extraction for float32 x/y/z PointCloud2.
    if msg is None:
        return None
    x_off = y_off = z_off = None
    for f in msg.fields:
        if f.name == "x":
            x_off = int(f.offset)
        elif f.name == "y":
            y_off = int(f.offset)
        elif f.name == "z":
            z_off = int(f.offset)
    if x_off is None or y_off is None:
        return None
    if msg.point_step <= max(x_off, y_off, z_off or 0) + 4:
        return None
    data = bytes(msg.data)
    total_points = int(msg.width * msg.height)
    if total_points <= 0:
        return None
    stride = max(1, total_points // max_points)
    pts = []
    import struct
    for i in range(0, total_points, stride):
        base = i * msg.point_step
        if base + msg.point_step > len(data):
            break
        x = struct.unpack_from("<f", data, base + x_off)[0]
        y = struct.unpack_from("<f", data, base + y_off)[0]
        pts.append([float(x), float(y)])
        if len(pts) >= max_points:
            break
    return {
        "count": total_points,
        "points": pts,
        "width": int(msg.width),
        "height": int(msg.height),
        "frame": msg.header.frame_id,
    }


def camera_loop():
    global last_camera_jpeg, last_camera_ts
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    while True:
        code, data = client.GetImageSample()
        if code == 0:
            jpeg = bytes(data)
            if jpeg:
                with state_lock:
                    last_camera_jpeg = jpeg
                    last_camera_ts = time.time()
        time.sleep(0.02)


def set_lidar_switch(status: str):
    global lidar_switch_pub
    if lidar_switch_pub is None:
        return -1
    msg = std_msgs_msg_dds__String_()
    msg.data = status
    lidar_switch_pub.Write(msg)
    return 0


def set_low_level_enabled(enable: bool):
    global low_level_enabled
    low_level_enabled = enable


def low_level_loop():
    # Low-level command streaming; use with caution.
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
    while True:
        if low_level_enabled and low_cmd_pub is not None:
            for i in range(20):
                cmd.motor_cmd[i].q = float(low_q[i])
                cmd.motor_cmd[i].kp = float(low_kp[i])
                cmd.motor_cmd[i].kd = float(low_kd[i])
            cmd.crc = crc.Crc(cmd)
            low_cmd_pub.Write(cmd)
        time.sleep(0.02)


def _toggle_action(fn, duration_sec: float):
    code = fn(True)

    def _clear():
        time.sleep(duration_sec)
        try:
            fn(False)
        except Exception:
            pass

    threading.Thread(target=_clear, daemon=True).start()
    return code


class UiServer(BaseHTTPRequestHandler):
    def _send(self, status, body, content_type="text/plain; charset=utf-8"):
        payload = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _send_json(self, data, status=200):
        self._send(status, json.dumps(data), "application/json; charset=utf-8")

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path == "/":
            self._send(200, render_html(), "text/html; charset=utf-8")
            return
        if path == "/api/state":
            with state_lock:
                data = {
                    "lowstate": serialize_lowstate(last_lowstate),
                    "sportstate": serialize_sportstate(last_sportstate),
                    "wireless": serialize_wireless(last_wireless),
                    "lidar": serialize_lidar(last_lidar_state),
                    "lidar_points": _extract_points_xy(last_lidar_points),
                    "odometry": serialize_odom(last_odom),
                    "map": serialize_map(last_map),
                    "heightmap": serialize_heightmap(last_heightmap),
                    "camera_ts": last_camera_ts,
                    "lidar_points_ts": last_lidar_points_ts,
                    "low_level": {
                        "enabled": low_level_enabled,
                        "q": list(low_q),
                        "kp": list(low_kp),
                        "kd": list(low_kd),
                    },
                    "topics": {
                        "lidar_state": TOPIC_LIDAR_STATE,
                        "lidar_points": TOPIC_LIDAR_POINTS,
                        "odometry": TOPIC_ODOM,
                        "map": TOPIC_MAP,
                        "heightmap": TOPIC_HEIGHTMAP,
                    },
                }
            self._send_json(data)
            return
        if path == "/api/cmd":
            params = urllib.parse.parse_qs(parsed.query)
            name = params.get("name", [""])[0]
            height = params.get("height", [""])[0]
            vx = params.get("vx", [""])[0]
            vy = params.get("vy", [""])[0]
            vyaw = params.get("vyaw", [""])[0]
            result = handle_command(name, height, vx, vy, vyaw)
            self._send_json(result)
            return
        if path == "/api/low/enable":
            params = urllib.parse.parse_qs(parsed.query)
            state = params.get("state", ["0"])[0]
            enable = state == "1"
            set_low_level_enabled(enable)
            self._send_json({"enabled": low_level_enabled})
            return
        if path == "/api/low/set":
            params = urllib.parse.parse_qs(parsed.query)
            idx = int(params.get("idx", ["-1"])[0])
            q = params.get("q", [None])[0]
            kp = params.get("kp", [None])[0]
            kd = params.get("kd", [None])[0]
            if 0 <= idx < 20:
                if q is not None:
                    low_q[idx] = float(q)
                if kp is not None:
                    low_kp[idx] = float(kp)
                if kd is not None:
                    low_kd[idx] = float(kd)
                self._send_json({"ok": True})
            else:
                self._send_json({"ok": False, "error": "bad idx"}, status=400)
            return
        if path == "/api/low/set_all":
            params = urllib.parse.parse_qs(parsed.query)
            kp = params.get("kp", [None])[0]
            kd = params.get("kd", [None])[0]
            for i in range(20):
                if kp is not None:
                    low_kp[i] = float(kp)
                if kd is not None:
                    low_kd[i] = float(kd)
            self._send_json({"ok": True})
            return
        if path == "/camera.mjpeg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with state_lock:
                        frame = last_camera_jpeg
                    if frame:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        self._send(404, "Not found")


def handle_command(name, height, vx, vy, vyaw):
    try:
        if sport_client is None:
            return {"code": -1, "error": "sport client not initialized"}
        if name == "damp":
            return {"code": sport_client.Damp()}
        if name == "stand_up":
            return {"code": sport_client.StandUp()}
        if name == "stand_down":
            return {"code": sport_client.StandDown()}
        if name == "balance_stand":
            return {"code": sport_client.BalanceStand()}
        if name == "recovery":
            return {"code": sport_client.RecoveryStand()}
        if name == "stop_move":
            return {"code": sport_client.StopMove()}
        if name == "free_walk":
            return {"code": sport_client.FreeWalk()}
        if name == "free_bound":
            return {"code": _toggle_action(sport_client.FreeBound, 2.0)}
        if name == "free_avoid":
            return {"code": _toggle_action(sport_client.FreeAvoid, 2.0)}
        if name == "walk_upright":
            return {"code": _toggle_action(sport_client.WalkUpright, 4.0)}
        if name == "cross_step":
            return {"code": _toggle_action(sport_client.CrossStep, 4.0)}
        if name == "hand_stand":
            return {"code": _toggle_action(sport_client.HandStand, 4.0)}
        if name == "left_flip":
            return {"code": sport_client.LeftFlip()}
        if name == "back_flip":
            return {"code": sport_client.BackFlip()}
        if name == "free_jump":
            return {"code": _toggle_action(sport_client.FreeJump, 4.0)}
        if name == "move":
            fx = float(vx or 0.0)
            fy = float(vy or 0.0)
            fz = float(vyaw or 0.0)
            return {"code": sport_client.Move(fx, fy, fz)}
        if name == "euler":
            roll = float(vx or 0.0)
            pitch = float(vy or 0.0)
            yaw = float(vyaw or 0.0)
            return {"code": sport_client.Euler(roll, pitch, yaw)}
        if name == "move_forward":
            return {"code": sport_client.Move(0.3, 0.0, 0.0)}
        if name == "move_lateral":
            return {"code": sport_client.Move(0.0, 0.3, 0.0)}
        if name == "move_rotate":
            return {"code": sport_client.Move(0.0, 0.0, 0.5)}
        if name == "sit":
            return {"code": sport_client.Sit()}
        if name == "rise_sit":
            return {"code": sport_client.RiseSit()}
        if name == "speed_level":
            level = int(vx)
            return {"code": sport_client.SpeedLevel(level)}
        if name == "hello":
            return {"code": sport_client.Hello()}
        if name == "stretch":
            return {"code": sport_client.Stretch()}
        if name == "content":
            return {"code": sport_client.Content()}
        if name == "heart":
            return {"code": sport_client.Heart()}
        if name == "pose_on":
            return {"code": sport_client.Pose(True)}
        if name == "pose_off":
            return {"code": sport_client.Pose(False)}
        if name == "scrape":
            return {"code": sport_client.Scrape()}
        if name == "front_flip":
            return {"code": sport_client.FrontFlip()}
        if name == "front_jump":
            return {"code": sport_client.FrontJump()}
        if name == "front_pounce":
            return {"code": sport_client.FrontPounce()}
        if name == "dance1":
            return {"code": sport_client.Dance1()}
        if name == "dance2":
            return {"code": sport_client.Dance2()}
        if name == "switch_joystick_on":
            return {"code": sport_client.SwitchJoystick(True)}
        if name == "switch_joystick_off":
            return {"code": sport_client.SwitchJoystick(False)}
        if name == "auto_recovery_on":
            return {"code": sport_client.AutoRecoverySet(True)}
        if name == "auto_recovery_off":
            return {"code": sport_client.AutoRecoverySet(False)}
        if name == "auto_recovery_get":
            code, enabled = sport_client.AutoRecoveryGet()
            return {"code": code, "enabled": enabled}
        if name == "classic_walk_on":
            return {"code": sport_client.ClassicWalk(True)}
        if name == "classic_walk_off":
            return {"code": sport_client.ClassicWalk(False)}
        if name == "static_walk":
            return {"code": sport_client.StaticWalk()}
        if name == "trot_run":
            return {"code": sport_client.TrotRun()}
        if name == "free_bound_on":
            return {"code": sport_client.FreeBound(True)}
        if name == "free_bound_off":
            return {"code": sport_client.FreeBound(False)}
        if name == "free_jump_on":
            return {"code": sport_client.FreeJump(True)}
        if name == "free_jump_off":
            return {"code": sport_client.FreeJump(False)}
        if name == "free_avoid_on":
            return {"code": sport_client.FreeAvoid(True)}
        if name == "free_avoid_off":
            return {"code": sport_client.FreeAvoid(False)}
        if name == "walk_upright_on":
            return {"code": sport_client.WalkUpright(True)}
        if name == "walk_upright_off":
            return {"code": sport_client.WalkUpright(False)}
        if name == "cross_step_on":
            return {"code": sport_client.CrossStep(True)}
        if name == "cross_step_off":
            return {"code": sport_client.CrossStep(False)}
        if name == "hand_stand_on":
            return {"code": sport_client.HandStand(True)}
        if name == "hand_stand_off":
            return {"code": sport_client.HandStand(False)}
        if name == "switch_avoid_mode":
            return {"code": sport_client.SwitchAvoidMode()}
        if name == "lidar_on":
            return {"code": set_lidar_switch("ON")}
        if name == "lidar_off":
            return {"code": set_lidar_switch("OFF")}
    except Exception as exc:
        return {"code": -1, "error": str(exc)}
    return {"code": -1, "error": "unknown command"}


def render_html():
    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Go2 Sport Control</title>
  <style>
    :root {{
      --bg: #0f1116;
      --panel: #151923;
      --panel-2: #1c2230;
      --accent: #f5b942;
      --accent-2: #59d0ff;
      --text: #e9edf2;
      --muted: #94a0b8;
      --danger: #ff6b6b;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Futura", "Avenir Next", "Trebuchet MS", sans-serif;
      background: radial-gradient(1200px 700px at 20% -10%, #23304a 0%, var(--bg) 45%) no-repeat,
                  radial-gradient(1000px 600px at 120% 10%, #2b1f33 0%, var(--bg) 55%) no-repeat;
      color: var(--text);
    }}
    header {{
      padding: 18px 22px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-bottom: 1px solid #1f2633;
      background: linear-gradient(90deg, rgba(245,185,66,0.15), rgba(89,208,255,0.12));
      backdrop-filter: blur(6px);
    }}
    header h1 {{
      font-size: 20px;
      margin: 0;
      letter-spacing: 1px;
      text-transform: uppercase;
    }}
    header .status {{
      font-size: 12px;
      color: var(--muted);
    }}
    .grid {{
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 16px;
      padding: 16px;
    }}
    .panel {{
      background: var(--panel);
      border: 1px solid #232a3a;
      border-radius: 14px;
      padding: 14px;
      box-shadow: 0 8px 30px rgba(0,0,0,0.35);
    }}
    .panel h2 {{
      margin: 0 0 10px 0;
      font-size: 14px;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: var(--muted);
    }}
    .camera {{
      position: relative;
      aspect-ratio: 16 / 9;
      background: #0b0f18;
      border-radius: 12px;
      overflow: hidden;
      border: 1px solid #222a3a;
    }}
    .camera img {{
      width: 100%;
      height: 100%;
      object-fit: cover;
      filter: saturate(1.1) contrast(1.05);
    }}
    .row {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
    }}
    .kv {{
      background: var(--panel-2);
      padding: 10px;
      border-radius: 10px;
      border: 1px solid #252e40;
      min-height: 72px;
    }}
    .kv span {{
      display: block;
      font-size: 11px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.8px;
    }}
    .kv strong {{
      display: block;
      font-size: 16px;
      margin-top: 6px;
    }}
    .controls {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 10px;
    }}
    button {{
      background: linear-gradient(135deg, #2b3244, #1f2433);
      color: var(--text);
      border: 1px solid #2c3448;
      padding: 10px 12px;
      border-radius: 10px;
      cursor: pointer;
      font-size: 12px;
      letter-spacing: 0.4px;
      transition: transform 0.12s ease, border 0.12s ease;
    }}
    button:hover {{ transform: translateY(-1px); border-color: var(--accent); }}
    button.primary {{ border-color: var(--accent); }}
    button.warn {{
      border-color: var(--danger);
      background: linear-gradient(135deg, #4b1f1f, #2a1414);
    }}
    .slider {{
      display: flex;
      align-items: center;
      gap: 10px;
      background: var(--panel-2);
      padding: 10px;
      border-radius: 10px;
      border: 1px solid #252e40;
    }}
    input[type="range"] {{
      width: 100%;
    }}
    .joy {{
      position: relative;
      width: 160px;
      height: 160px;
      border-radius: 50%;
      margin: 8px auto 6px auto;
      background: radial-gradient(circle at 30% 30%, #1e2535, #131825);
      border: 1px solid #2b3244;
      touch-action: none;
    }}
    .joy-dot {{
      position: absolute;
      width: 28px;
      height: 28px;
      border-radius: 50%;
      background: var(--accent-2);
      box-shadow: 0 0 18px rgba(89, 208, 255, 0.5);
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      transition: transform 0.05s linear;
    }}
    .joy-label {{
      text-align: center;
      font-size: 11px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.6px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }}
    th, td {{
      padding: 6px 8px;
      border-bottom: 1px solid #242b3b;
      text-align: right;
    }}
    th:first-child, td:first-child {{ text-align: left; }}
    .tag {{
      display: inline-block;
      padding: 4px 8px;
      border-radius: 999px;
      background: rgba(245,185,66,0.12);
      color: var(--accent);
      font-size: 11px;
      letter-spacing: 0.6px;
      text-transform: uppercase;
    }}
  </style>
</head>
<body>
  <header>
    <h1>Go2 Sport Control</h1>
    <div class="status">Interface: {INTERFACE} · Web UI</div>
  </header>
  <div class="grid">
    <div class="panel">
      <h2>Live Camera</h2>
      <div class="camera">
        <img id="camera" src="/camera.mjpeg" alt="front camera">
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Camera Status</span>
          <strong id="camera_status">--</strong>
        </div>
      </div>
    </div>
    <div class="panel">
      <h2>LiDAR Service</h2>
      <div class="controls">
        <button onclick="cmd('lidar_on')" class="primary">LiDAR ON</button>
        <button onclick="cmd('lidar_off')" class="warn">LiDAR OFF</button>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv"><span>Topic</span><strong id="lidar_topic">--</strong></div>
        <div class="kv"><span>Firmware</span><strong id="lidar_firmware">--</strong></div>
        <div class="kv"><span>SDK</span><strong id="lidar_sdk">--</strong></div>
        <div class="kv"><span>Cloud Hz</span><strong id="lidar_cloud">--</strong></div>
        <div class="kv"><span>Loss %</span><strong id="lidar_loss">--</strong></div>
        <div class="kv"><span>Error</span><strong id="lidar_error">--</strong></div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Point Cloud Topic</span>
          <strong id="lidar_points_topic">--</strong>
        </div>
        <div class="kv">
          <span>Point Cloud Count</span>
          <strong id="lidar_points_count">--</strong>
        </div>
        <div class="kv">
          <span>Point Cloud Frame</span>
          <strong id="lidar_points_frame">--</strong>
        </div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Point Cloud Preview (XY)</span>
          <canvas id="lidar_canvas" width="360" height="220" style="width:100%; background:#0b0f18; border-radius:10px; border:1px solid #222a3a;"></canvas>
        </div>
      </div>
    </div>
    <div class="panel">
      <h2>Control Panel</h2>
      <div class="controls">
        <button onclick="cmd('damp')" class="warn">Damp</button>
        <button onclick="cmd('stand_up')">Stand Up</button>
        <button onclick="cmd('stand_down')" class="warn">Stand Down</button>
        <button onclick="cmd('balance_stand')" class="primary">Balance Stand</button>
        <button onclick="cmd('recovery')">Recovery Stand</button>
        <button onclick="cmd('move_forward')" class="warn">Move Forward</button>
        <button onclick="cmd('move_lateral')" class="warn">Move Lateral</button>
        <button onclick="cmd('move_rotate')" class="warn">Move Rotate</button>
        <button onclick="cmd('stop_move')" class="warn">Stop Move</button>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Speed Level</span>
          <div class="controls">
            <button onclick="cmd('speed_level', -1)">Slow</button>
            <button onclick="cmd('speed_level', 0)" class="primary">Normal</button>
            <button onclick="cmd('speed_level', 1)" class="warn">Fast</button>
          </div>
        </div>
        <div class="kv">
          <span>Joystick Response</span>
          <div class="controls">
            <button onclick="cmd('switch_joystick_on')">Enable</button>
            <button onclick="cmd('switch_joystick_off')" class="warn">Disable</button>
          </div>
        </div>
        <div class="kv">
          <span>Auto Recovery</span>
          <div class="controls">
            <button onclick="cmd('auto_recovery_on')">Enable</button>
            <button onclick="cmd('auto_recovery_off')" class="warn">Disable</button>
            <button onclick="cmd('auto_recovery_get')">Check</button>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Euler (roll/pitch/yaw)</span>
          <div class="controls">
            <input id="roll" type="range" min="-0.75" max="0.75" step="0.01" value="0.00">
            <input id="pitch" type="range" min="-0.75" max="0.75" step="0.01" value="0.00">
            <input id="yaw" type="range" min="-0.6" max="0.6" step="0.01" value="0.00">
            <button onclick="sendEuler()">Apply</button>
            <button onclick="resetEuler()">Reset</button>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Gaits</span>
          <div class="controls">
            <button onclick="cmd('free_walk')" class="warn">Free Walk</button>
            <button onclick="cmd('static_walk')" class="warn">Static Walk</button>
            <button onclick="cmd('trot_run')" class="warn">Trot Run</button>
            <button onclick="cmd('classic_walk_on')" class="warn">Classic Walk On</button>
            <button onclick="cmd('classic_walk_off')">Classic Walk Off</button>
            <button onclick="cmd('free_bound_on')" class="warn">Free Bound On</button>
            <button onclick="cmd('free_bound_off')">Free Bound Off</button>
            <button onclick="cmd('free_jump_on')" class="warn">Free Jump On</button>
            <button onclick="cmd('free_jump_off')">Free Jump Off</button>
            <button onclick="cmd('free_avoid_on')" class="warn">Free Avoid On</button>
            <button onclick="cmd('free_avoid_off')">Free Avoid Off</button>
            <button onclick="cmd('walk_upright_on')" class="warn">Walk Upright On</button>
            <button onclick="cmd('walk_upright_off')">Walk Upright Off</button>
            <button onclick="cmd('cross_step_on')" class="warn">Cross Step On</button>
            <button onclick="cmd('cross_step_off')">Cross Step Off</button>
            <button onclick="cmd('switch_avoid_mode')" class="warn">Switch Avoid Mode</button>
          </div>
        </div>
        <div class="kv">
          <span>Special Actions</span>
          <div class="controls">
            <button onclick="cmd('sit')">Sit</button>
            <button onclick="cmd('rise_sit')">Rise Sit</button>
            <button onclick="cmd('hello')">Hello</button>
            <button onclick="cmd('stretch')">Stretch</button>
            <button onclick="cmd('content')">Content</button>
            <button onclick="cmd('heart')">Heart</button>
            <button onclick="cmd('pose_on')">Pose On</button>
            <button onclick="cmd('pose_off')">Pose Off</button>
            <button onclick="cmd('scrape')">Scrape</button>
            <button onclick="cmd('dance1')">Dance1</button>
            <button onclick="cmd('dance2')">Dance2</button>
            <button onclick="cmd('hand_stand_on')" class="warn">Hand Stand On</button>
            <button onclick="cmd('hand_stand_off')">Hand Stand Off</button>
            <button onclick="cmd('front_jump')" class="warn">Front Jump</button>
            <button onclick="cmd('front_pounce')" class="warn">Front Pounce</button>
            <button onclick="cmd('front_flip')" class="warn">Front Flip</button>
            <button onclick="cmd('left_flip')" class="warn">Left Flip</button>
            <button onclick="cmd('back_flip')" class="warn">Back Flip</button>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv"><span>Drive Command</span><strong id="drive_cmd">vx 0.00, vy 0.00, yaw 0.00</strong></div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv"><span>Command Result</span><strong id="cmd_result">--</strong></div>
        <div class="kv"><span>Mode</span><strong id="mode">--</strong></div>
        <div class="kv"><span>Body Height</span><strong id="body_height">--</strong></div>
      </div>
    </div>
    <div class="panel">
      <h2>Low-Level Actuators (20 Motors)</h2>
      <div class="row">
        <div class="kv">
          <span>Warning</span>
          <strong>Use with caution. Low-level overrides high-level control.</strong>
        </div>
        <div class="kv">
          <span>Stream</span>
          <div class="controls">
            <button onclick="lowEnable(true)" class="warn">Enable</button>
            <button onclick="lowEnable(false)">Disable</button>
          </div>
        </div>
        <div class="kv">
          <span>Global kp/kd</span>
          <div class="controls">
            <input id="kp_all" type="range" min="0" max="60" step="1" value="20">
            <input id="kd_all" type="range" min="0" max="5" step="0.1" value="3.5">
            <button onclick="setAllGains()">Apply</button>
          </div>
        </div>
      </div>
      <div id="low_table" style="margin-top:12px;"></div>
    </div>
    <div class="panel">
      <h2>Drive Joysticks</h2>
      <div class="row">
        <div class="kv">
          <span>Translation</span>
          <div class="joy" id="joy_move">
            <div class="joy-dot"></div>
          </div>
          <div class="joy-label">Forward / Back, Left / Right</div>
        </div>
        <div class="kv">
          <span>Rotation</span>
          <div class="joy" id="joy_yaw">
            <div class="joy-dot"></div>
          </div>
          <div class="joy-label">Rotate Left / Right</div>
        </div>
      </div>
    </div>
    <div class="panel">
      <h2>SLAM + Navigation</h2>
      <div class="row">
        <div class="kv"><span>Odom Topic</span><strong id="odom_topic">--</strong></div>
        <div class="kv"><span>Map Topic</span><strong id="map_topic">--</strong></div>
        <div class="kv"><span>HeightMap Topic</span><strong id="heightmap_topic">--</strong></div>
        <div class="kv"><span>Odom Pos</span><strong id="odom_pos">--</strong></div>
        <div class="kv"><span>Odom Vel</span><strong id="odom_vel">--</strong></div>
        <div class="kv"><span>Map Info</span><strong id="map_info">--</strong></div>
        <div class="kv"><span>HeightMap</span><strong id="heightmap_info">--</strong></div>
      </div>
    </div>
    <div class="panel">
      <h2>Sport State</h2>
      <div class="row">
        <div class="kv"><span>Progress</span><strong id="progress">--</strong></div>
        <div class="kv"><span>Gait</span><strong id="gait">--</strong></div>
        <div class="kv"><span>Foot Raise</span><strong id="foot_raise">--</strong></div>
        <div class="kv"><span>Position</span><strong id="position">--</strong></div>
        <div class="kv"><span>Velocity</span><strong id="velocity">--</strong></div>
        <div class="kv"><span>Yaw Speed</span><strong id="yaw_speed">--</strong></div>
      </div>
    </div>
    <div class="panel">
      <h2>Low State + Actuators</h2>
      <div class="row">
        <div class="kv"><span>Battery V</span><strong id="power_v">--</strong></div>
        <div class="kv"><span>Battery A</span><strong id="power_a">--</strong></div>
        <div class="kv"><span>Foot Force</span><strong id="foot_force">--</strong></div>
        <div class="kv"><span>IMU RPY</span><strong id="imu_rpy">--</strong></div>
      </div>
      <table style="margin-top:10px;">
        <thead>
          <tr>
            <th>Joint</th>
            <th>q</th>
            <th>dq</th>
            <th>tau</th>
            <th>temp</th>
          </tr>
        </thead>
        <tbody id="motors"></tbody>
      </table>
    </div>
  </div>
  <script>
    function cmd(name, value) {{
      const extra = (value !== undefined) ? `&vx=${{encodeURIComponent(value)}}` : '';
      fetch(`/api/cmd?name=${{encodeURIComponent(name)}}${{extra}}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d))
        .catch(() => document.getElementById('cmd_result').textContent = 'error');
    }}

    const joyState = {{
      vx: 0.0,
      vy: 0.0,
      vyaw: 0.0,
      lastSend: 0,
    }};
    const joyActive = {{
      move: false,
      yaw: false,
    }};

    function resetMove() {{
      joyState.vx = 0.0;
      joyState.vy = 0.0;
      sendMove(true);
    }}

    function resetYaw() {{
      joyState.vyaw = 0.0;
      sendMove(true);
    }}

    function sendMove(force) {{
      const now = Date.now();
      if (!force && now - joyState.lastSend < 120) return;
      joyState.lastSend = now;
      const vx = joyState.vx.toFixed(2);
      const vy = joyState.vy.toFixed(2);
      const vyaw = joyState.vyaw.toFixed(2);
      document.getElementById('drive_cmd').textContent = `vx ${{vx}}, vy ${{vy}}, yaw ${{vyaw}}`;
      fetch(`/api/cmd?name=move&vx=${{vx}}&vy=${{vy}}&vyaw=${{vyaw}}`)
        .then(r => r.json())
        .then(d => {{
          if (d.code !== 0) {{
            document.getElementById('cmd_result').textContent = JSON.stringify(d);
          }}
        }})
        .catch(() => {{}});
    }}

    function setupJoystick(rootId, onMove, onReset, flagKey) {{
      const root = document.getElementById(rootId);
      const dot = root.querySelector('.joy-dot');
      const rect = () => root.getBoundingClientRect();
      const radius = 70;
      function setDot(dx, dy) {{
        dot.style.transform = `translate(${{-50 + dx}}px, ${{-50 + dy}}px)`;
      }}
      function handle(x, y) {{
        const r = rect();
        const cx = r.left + r.width / 2;
        const cy = r.top + r.height / 2;
        let dx = x - cx;
        let dy = y - cy;
        const dist = Math.hypot(dx, dy);
        if (dist > radius) {{
          dx = dx / dist * radius;
          dy = dy / dist * radius;
        }}
        setDot(dx, dy);
        const nx = dx / radius;
        const ny = dy / radius;
        onMove(nx, ny);
      }}
      function reset() {{
        setDot(0, 0);
        joyActive[flagKey] = false;
        onReset();
      }}
      root.addEventListener('pointerdown', (e) => {{
        root.setPointerCapture(e.pointerId);
        joyActive[flagKey] = true;
        handle(e.clientX, e.clientY);
      }});
      root.addEventListener('pointermove', (e) => {{
        if (e.pressure === 0) return;
        handle(e.clientX, e.clientY);
      }});
      root.addEventListener('pointerup', reset);
      root.addEventListener('pointercancel', reset);
      root.addEventListener('pointerleave', reset);
    }}

    setupJoystick('joy_move', (nx, ny) => {{
      joyState.vx = -ny * 0.5;
      joyState.vy = nx * 0.5;
      sendMove();
    }}, resetMove, 'move');
    setupJoystick('joy_yaw', (nx) => {{
      joyState.vyaw = nx * 0.8;
      sendMove();
    }}, resetYaw, 'yaw');

    window.addEventListener('blur', () => {{
      resetMove();
      resetYaw();
    }});
    document.addEventListener('visibilitychange', () => {{
      if (document.hidden) {{
        resetMove();
        resetYaw();
      }}
    }});
    document.addEventListener('pointerup', () => {{
      if (joyActive.move || joyActive.yaw) {{
        resetMove();
        resetYaw();
      }}
    }});

    function sendEuler() {{
      const roll = document.getElementById('roll').value;
      const pitch = document.getElementById('pitch').value;
      const yaw = document.getElementById('yaw').value;
      fetch(`/api/cmd?name=euler&vx=${{roll}}&vy=${{pitch}}&vyaw=${{yaw}}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d));
    }}

    function resetEuler() {{
      document.getElementById('roll').value = 0.0;
      document.getElementById('pitch').value = 0.0;
      document.getElementById('yaw').value = 0.0;
      sendEuler();
    }}

    function drawLidar(lidarPoints) {{
      const canvas = document.getElementById('lidar_canvas');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#0b0f18';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      if (!lidarPoints || !lidarPoints.points || lidarPoints.points.length === 0) {{
        ctx.fillStyle = '#6b7280';
        ctx.fillText('no points', 10, 20);
        return;
      }}
      const pts = lidarPoints.points;
      let maxR = 0.0;
      for (let i = 0; i < pts.length; i++) {{
        const x = pts[i][0], y = pts[i][1];
        const r = Math.hypot(x, y);
        if (r > maxR) maxR = r;
      }}
      maxR = Math.max(maxR, 1.0);
      const cx = canvas.width / 2;
      const cy = canvas.height / 2;
      const scale = 0.45 * Math.min(canvas.width, canvas.height) / maxR;
      ctx.strokeStyle = '#1f2937';
      ctx.beginPath();
      ctx.arc(cx, cy, scale, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = '#59d0ff';
      for (let i = 0; i < pts.length; i++) {{
        const x = pts[i][0] * scale;
        const y = pts[i][1] * scale;
        ctx.fillRect(cx + x, cy - y, 2, 2);
      }}
    }}

    function lowEnable(enable) {{
      fetch(`/api/low/enable?state=${{enable ? 1 : 0}}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d));
    }}

    function setAllGains() {{
      const kp = document.getElementById('kp_all').value;
      const kd = document.getElementById('kd_all').value;
      fetch(`/api/low/set_all?kp=${{kp}}&kd=${{kd}}`)
        .then(r => r.json())
        .then(d => document.getElementById('cmd_result').textContent = JSON.stringify(d));
    }}

    function buildLowTable() {{
      const host = document.getElementById('low_table');
      const table = document.createElement('table');
      table.innerHTML = `
        <thead>
          <tr>
            <th>Motor</th>
            <th>q (rad)</th>
            <th>kp</th>
            <th>kd</th>
          </tr>
        </thead>
        <tbody></tbody>`;
      const body = table.querySelector('tbody');
      for (let i = 0; i < 20; i++) {{
        const row = document.createElement('tr');
        row.innerHTML = `
          <td>M${{i}}</td>
          <td>
            <input data-idx="${{i}}" data-key="q" type="range" min="-1.5" max="1.5" step="0.01" value="0.0">
            <span id="qv_${{i}}">0.00</span>
          </td>
          <td>
            <input data-idx="${{i}}" data-key="kp" type="range" min="0" max="60" step="1" value="20">
            <span id="kpv_${{i}}">20</span>
          </td>
          <td>
            <input data-idx="${{i}}" data-key="kd" type="range" min="0" max="5" step="0.1" value="3.5">
            <span id="kdv_${{i}}">3.5</span>
          </td>`;
        body.appendChild(row);
      }}
      host.appendChild(table);
      host.querySelectorAll('input[data-key]').forEach(inp => {{
        inp.addEventListener('input', (e) => {{
          const idx = e.target.getAttribute('data-idx');
          const key = e.target.getAttribute('data-key');
          const val = e.target.value;
          document.getElementById(`${{key}}v_${{idx}}`).textContent = Number(val).toFixed(2);
          fetch(`/api/low/set?idx=${{idx}}&${{key}}=${{val}}`)
            .catch(() => {{}});
        }});
      }});
    }}

    buildLowTable();

    function update() {{
      fetch('/api/state')
        .then(r => r.json())
        .then(data => {{
          const s = data.sportstate || {{}};
          const l = data.lowstate || {{}};
          const lidar = data.lidar || {{}};
          const lidarPoints = data.lidar_points || {{}};
          const odom = data.odometry || {{}};
          const map = data.map || {{}};
          const heightmap = data.heightmap || {{}};
          const topics = data.topics || {{}};
          document.getElementById('camera_status').textContent =
            data.camera_ts ? 'streaming' : 'no data';
          document.getElementById('lidar_topic').textContent = topics.lidar_state || '--';
          document.getElementById('lidar_points_topic').textContent = topics.lidar_points || '--';
          document.getElementById('lidar_firmware').textContent = lidar.firmware || '--';
          document.getElementById('lidar_sdk').textContent = lidar.sdk || '--';
          document.getElementById('lidar_cloud').textContent = lidar.cloud_freq ?? '--';
          document.getElementById('lidar_loss').textContent = lidar.cloud_loss ?? '--';
          document.getElementById('lidar_error').textContent = lidar.error ?? '--';
          document.getElementById('lidar_points_count').textContent = lidarPoints.count ?? '--';
          document.getElementById('lidar_points_frame').textContent = lidarPoints.frame || '--';
          document.getElementById('odom_topic').textContent = topics.odometry || '--';
          document.getElementById('map_topic').textContent = topics.map || '--';
          document.getElementById('heightmap_topic').textContent = topics.heightmap || '--';
          document.getElementById('odom_pos').textContent = odom.pos ? odom.pos.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('odom_vel').textContent = odom.lin_vel ? odom.lin_vel.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('map_info').textContent =
            map.width ? `${{map.width}}x${{map.height}} @${{map.resolution}}` : '--';
          document.getElementById('heightmap_info').textContent =
            heightmap.width ? `${{heightmap.width}}x${{heightmap.height}} @${{heightmap.resolution}}` : '--';
          drawLidar(lidarPoints);
          document.getElementById('mode').textContent = s.mode ?? '--';
          document.getElementById('progress').textContent = s.progress ?? '--';
          document.getElementById('gait').textContent = s.gait_type ?? '--';
          document.getElementById('foot_raise').textContent = s.foot_raise_height ?? '--';
          document.getElementById('body_height').textContent = s.body_height ?? '--';
          document.getElementById('position').textContent = s.position ? s.position.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('velocity').textContent = s.velocity ? s.velocity.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('yaw_speed').textContent = s.yaw_speed ?? '--';
          document.getElementById('power_v').textContent = l.power_v ?? '--';
          document.getElementById('power_a').textContent = l.power_a ?? '--';
          document.getElementById('foot_force').textContent = l.foot_force ? l.foot_force.join(', ') : '--';
          document.getElementById('imu_rpy').textContent = l.imu_rpy ? l.imu_rpy.map(v => v.toFixed(2)).join(', ') : '--';
          const tbody = document.getElementById('motors');
          tbody.innerHTML = '';
          if (l.motors) {{
            l.motors.forEach(m => {{
              const row = document.createElement('tr');
              row.innerHTML = `<td>J${{m.id}}</td><td>${{m.q.toFixed(3)}}</td><td>${{m.dq.toFixed(3)}}</td><td>${{m.tau_est.toFixed(3)}}</td><td>${{m.temp}}</td>`;
              tbody.appendChild(row);
            }});
          }}
        }})
        .catch(() => {{}});
    }}
    setInterval(update, 500);
    update();
  </script>
</body>
</html>
"""


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp2s0")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--lidar-state-topic", default=TOPIC_LIDAR_STATE)
    parser.add_argument("--lidar-points-topic", default=TOPIC_LIDAR_POINTS)
    parser.add_argument("--odom-topic", default=TOPIC_ODOM)
    parser.add_argument("--map-topic", default=TOPIC_MAP)
    parser.add_argument("--heightmap-topic", default=TOPIC_HEIGHTMAP)
    args = parser.parse_args()

    INTERFACE = args.iface
    TOPIC_LIDAR_STATE = args.lidar_state_topic
    TOPIC_LIDAR_POINTS = args.lidar_points_topic
    TOPIC_ODOM = args.odom_topic
    TOPIC_MAP = args.map_topic
    TOPIC_HEIGHTMAP = args.heightmap_topic
    ChannelFactoryInitialize(0, INTERFACE)

    low_sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
    low_sub.Init(lowstate_cb, 10)
    sport_sub = ChannelSubscriber(TOPIC_SPORTSTATE, SportModeState_)
    sport_sub.Init(sportstate_cb, 10)
    wireless_sub = ChannelSubscriber(TOPIC_WIRELESS, WirelessController_)
    wireless_sub.Init(wireless_cb, 10)
    lidar_sub = ChannelSubscriber(TOPIC_LIDAR_STATE, LidarState_)
    lidar_sub.Init(lidar_state_cb, 10)
    lidar_points_sub = ChannelSubscriber(TOPIC_LIDAR_POINTS, PointCloud2_)
    lidar_points_sub.Init(lidar_points_cb, 10)
    odom_sub = ChannelSubscriber(TOPIC_ODOM, Odometry_)
    odom_sub.Init(odom_cb, 10)
    map_sub = ChannelSubscriber(TOPIC_MAP, OccupancyGrid_)
    map_sub.Init(map_cb, 10)
    heightmap_sub = ChannelSubscriber(TOPIC_HEIGHTMAP, HeightMap_)
    heightmap_sub.Init(heightmap_cb, 10)

    sport_client = SportClient()
    sport_client.SetTimeout(5.0)
    sport_client.Init()

    lidar_switch_pub = ChannelPublisher(TOPIC_LIDAR_SWITCH, String_)
    lidar_switch_pub.Init()

    low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_pub.Init()

    low_thread = threading.Thread(target=low_level_loop, daemon=True)
    low_thread.start()

    camera_thread = threading.Thread(target=camera_loop, daemon=True)
    camera_thread.start()

    server = ThreadingHTTPServer((HOST, args.port), UiServer)
    print(f"Go2 web UI on http://{HOST}:{args.port} (iface={INTERFACE})")
    server.serve_forever()
