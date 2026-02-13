import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

HOST = "0.0.0.0"
PORT = 8010

TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_LOWCMD = "rt/lowcmd"

# Heuristic mapping: small offsets to avoid fighting the ground.
HEIGHT_REF_M = 0.02
THIGH_DELTA_RAD = 0.05
CALF_DELTA_RAD = -0.10

KP = 8.0
KD = 1.5
DAMP_KD = 2.0

state_lock = threading.Lock()
last_lowstate = None
base_q = None
height_m = 0.0
control_mode = "height"

crc = CRC()
low_cmd_pub = None
sport_client = None
motion_switcher = None


def lowstate_cb(msg: LowState_):
    global last_lowstate
    with state_lock:
        last_lowstate = msg


def _build_lowcmd(q_targets):
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01 if i < 12 else 0x00
        cmd.motor_cmd[i].q = float(q_targets[i]) if i < 12 else 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = KP if i < 12 else 0.0
        cmd.motor_cmd[i].kd = KD if i < 12 else 0.0
        cmd.motor_cmd[i].tau = 0.0
    cmd.crc = crc.Crc(cmd)
    return cmd


def _compute_q_targets(height_value, base):
    # Height slider is in meters; map to thigh/calf deltas.
    if base is None:
        return None
    factor = max(min(height_value / HEIGHT_REF_M, 1.0), -1.0)
    thigh_delta = factor * THIGH_DELTA_RAD
    calf_delta = factor * CALF_DELTA_RAD
    q = list(base)
    for leg in range(4):
        hip = leg * 3 + 0
        thigh = leg * 3 + 1
        calf = leg * 3 + 2
        q[hip] = base[hip]
        q[thigh] = base[thigh] + thigh_delta
        q[calf] = base[calf] + calf_delta
    return q


def lowcmd_loop():
    global base_q
    while True:
        if low_cmd_pub is None:
            time.sleep(0.02)
            continue
        with state_lock:
            ls = last_lowstate
            current_height = height_m
            mode = control_mode
        if ls is None:
            time.sleep(0.02)
            continue
        if base_q is None:
            base_q = [float(ls.motor_state[i].q) for i in range(12)]
        if mode == "damp":
            cmd = unitree_go_msg_dds__LowCmd_()
            cmd.head[0] = 0xFE
            cmd.head[1] = 0xEF
            cmd.level_flag = 0xFF
            cmd.gpio = 0
            for i in range(20):
                cmd.motor_cmd[i].mode = 0x01 if i < 12 else 0x00
                cmd.motor_cmd[i].q = 0.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kp = 0.0
                cmd.motor_cmd[i].kd = DAMP_KD if i < 12 else 0.0
                cmd.motor_cmd[i].tau = 0.0
            cmd.crc = crc.Crc(cmd)
            low_cmd_pub.Write(cmd)
        else:
            q_targets = _compute_q_targets(current_height, base_q)
            if q_targets is not None:
                cmd = _build_lowcmd(q_targets + [0.0] * 8)
                low_cmd_pub.Write(cmd)
        time.sleep(0.02)


class UiServer(BaseHTTPRequestHandler):
    def _send(self, status, body, content_type="text/plain; charset=utf-8"):
        payload = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/":
            self._send(200, render_html(), "text/html; charset=utf-8")
            return
        if parsed.path == "/api/state":
            with state_lock:
                payload = {
                    "height_m": height_m,
                    "have_lowstate": last_lowstate is not None,
                    "mode": control_mode,
                }
            self._send(200, str(payload).replace("'", '"'), "application/json; charset=utf-8")
            return
        if parsed.path == "/api/set":
            params = urllib.parse.parse_qs(parsed.query)
            value = params.get("height", ["0.0"])[0]
            set_height(float(value))
            self._send(200, '{"ok": true}', "application/json; charset=utf-8")
            return
        if parsed.path == "/api/mode":
            params = urllib.parse.parse_qs(parsed.query)
            mode = params.get("mode", ["height"])[0]
            set_mode(mode)
            self._send(200, '{"ok": true}', "application/json; charset=utf-8")
            return
        if parsed.path == "/api/unblock":
            threading.Thread(target=unblock_robot, daemon=True).start()
            self._send(200, '{"ok": true}', "application/json; charset=utf-8")
            return
        self._send(404, "Not found")


def set_height(value):
    global height_m, base_q
    with state_lock:
        height_m = value
        # Rebase to current pose for a smooth transition.
        if last_lowstate is not None:
            base_q = [float(last_lowstate.motor_state[i].q) for i in range(12)]


def set_mode(mode: str):
    global control_mode
    with state_lock:
        control_mode = mode


def send_damping(duration_sec: float):
    prev_mode = control_mode
    set_mode("damp")
    time.sleep(duration_sec)
    set_mode(prev_mode)


def unblock_robot():
    # Enter damping briefly, then release low-level control and return to balance.
    send_damping(2.0)
    set_mode("height")
    time.sleep(0.1)
    if sport_client is None or motion_switcher is None:
        return
    code, result = motion_switcher.CheckMode()
    while result and result.get("name"):
        sport_client.StandDown()
        motion_switcher.ReleaseMode()
        time.sleep(1.0)
        code, result = motion_switcher.CheckMode()
    sport_client.BalanceStand()


def render_html():
    return """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Go2 Hip Height</title>
  <style>
    :root { --bg:#0f1116; --panel:#151923; --accent:#f5b942; --text:#e9edf2; --muted:#94a0b8; }
    body { margin:0; font-family: "Futura","Avenir Next","Trebuchet MS",sans-serif; background: var(--bg); color:var(--text); }
    header { padding:18px 22px; border-bottom:1px solid #1f2633; }
    .panel { margin:20px; background:var(--panel); border:1px solid #232a3a; border-radius:14px; padding:16px; }
    .row { display:flex; gap:16px; align-items:center; }
    .label { font-size:12px; color:var(--muted); text-transform:uppercase; letter-spacing:1px; }
    input[type="range"] { width:100%; }
    .value { font-size:18px; font-weight:600; color:var(--accent); }
    .note { font-size:12px; color:var(--muted); margin-top:10px; }
  </style>
</head>
<body>
  <header>
    <h2>Go2 Hip Height (Low-Level)</h2>
  </header>
  <div class="panel">
    <div class="label">Hip Height (m, relative)</div>
    <div class="row">
      <input id="height" type="range" min="-0.02" max="0.02" step="0.001" value="0.0">
      <div class="value" id="height_val">0.000 m</div>
    </div>
    <div class="note">
      Uses low-level motor control (rt/lowcmd). Heuristic joint offsets only; no full IK.
    </div>
    <div class="row" style="margin-top:12px;">
      <button onclick="setMode('height')">Height Control</button>
      <button onclick="setMode('damp')">Damping Mode</button>
      <button onclick="unblock()">Unblock Robot</button>
    </div>
  </div>
  <script>
    const slider = document.getElementById('height');
    const label = document.getElementById('height_val');
    function send() {
      const v = parseFloat(slider.value).toFixed(3);
      label.textContent = v + ' m';
      fetch(`/api/set?height=${v}`).catch(() => {});
    }
    slider.addEventListener('input', send);
    send();

    function setMode(mode) {
      fetch(`/api/mode?mode=${mode}`).catch(() => {});
    }

    function unblock() {
      fetch('/api/unblock').catch(() => {});
    }
  </script>
</body>
</html>
"""


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp2s0")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    # Release any active high-level mode before low-level control.
    sport = SportClient()
    sport.SetTimeout(5.0)
    sport.Init()
    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()
    code, result = msc.CheckMode()
    while result and result.get("name"):
        sport.StandDown()
        msc.ReleaseMode()
        time.sleep(1.0)
        code, result = msc.CheckMode()
    sport_client = sport
    motion_switcher = msc
    sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
    sub.Init(lowstate_cb, 10)

    low_cmd_pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
    low_cmd_pub.Init()

    t = threading.Thread(target=lowcmd_loop, daemon=True)
    t.start()

    server = ThreadingHTTPServer((HOST, args.port), UiServer)
    print(f"Hip height UI on http://{HOST}:{args.port} (iface={args.iface})")
    server.serve_forever()
