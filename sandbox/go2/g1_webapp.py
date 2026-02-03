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
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

HOST = "0.0.0.0"
PORT = 8020
INTERFACE = "enp2s0"

TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_LOWCMD = "rt/lowcmd"

NUM_MOTOR = 29

state_lock = threading.Lock()
last_lowstate = None
mode_machine = 0
mode_pr = 0

low_cmd_pub = None
low_level_enabled = False
low_q = [0.0] * NUM_MOTOR
low_kp = [20.0] * NUM_MOTOR
low_kd = [1.0] * NUM_MOTOR
crc = CRC()

loco_client = None


def lowstate_cb(msg: LowState_):
    global last_lowstate, mode_machine, mode_pr
    with state_lock:
        last_lowstate = msg
        mode_machine = int(msg.mode_machine)
        mode_pr = int(msg.mode_pr)


def serialize_lowstate(msg: LowState_):
    if msg is None:
        return None
    motors = []
    for i in range(NUM_MOTOR):
        m = msg.motor_state[i]
        motors.append(
            {
                "id": i,
                "q": float(m.q),
                "dq": float(m.dq),
                "tau_est": float(m.tau_est),
                "temp": int(m.temperature),
                "mode": int(m.mode),
                "lost": int(m.lost),
                "err": int(m.reserve[0]),
                "hz": int(m.reserve[1]),
            }
        )
    return {
        "mode_machine": int(msg.mode_machine),
        "mode_pr": int(msg.mode_pr),
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
        "imu_quat": [
            float(msg.imu_state.quaternion[0]),
            float(msg.imu_state.quaternion[1]),
            float(msg.imu_state.quaternion[2]),
            float(msg.imu_state.quaternion[3]),
        ],
        "imu_temp": int(msg.imu_state.temperature),
        "motors": motors,
    }


def set_low_level_enabled(enable: bool):
    global low_level_enabled
    low_level_enabled = enable


def low_level_loop():
    cmd = unitree_hg_msg_dds__LowCmd_()
    while True:
        if low_level_enabled and low_cmd_pub is not None:
            cmd.mode_pr = mode_pr
            cmd.mode_machine = mode_machine
            for i in range(NUM_MOTOR):
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q = float(low_q[i])
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].tau = 0.0
                cmd.motor_cmd[i].kp = float(low_kp[i])
                cmd.motor_cmd[i].kd = float(low_kd[i])
            cmd.crc = crc.Crc(cmd)
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
                    "low_level": {
                        "enabled": low_level_enabled,
                        "q": list(low_q),
                        "kp": list(low_kp),
                        "kd": list(low_kd),
                    },
                }
            self._send_json(data)
            return
        if path == "/api/cmd":
            params = urllib.parse.parse_qs(parsed.query)
            name = params.get("name", [""])[0]
            vx = params.get("vx", [""])[0]
            vy = params.get("vy", [""])[0]
            vyaw = params.get("vyaw", [""])[0]
            result = handle_command(name, vx, vy, vyaw)
            self._send_json(result)
            return
        if path == "/api/low/enable":
            params = urllib.parse.parse_qs(parsed.query)
            state = params.get("state", ["0"])[0]
            set_low_level_enabled(state == "1")
            self._send_json({"enabled": low_level_enabled})
            return
        if path == "/api/low/set":
            params = urllib.parse.parse_qs(parsed.query)
            idx = int(params.get("idx", ["-1"])[0])
            q = params.get("q", [None])[0]
            kp = params.get("kp", [None])[0]
            kd = params.get("kd", [None])[0]
            if 0 <= idx < NUM_MOTOR:
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
            for i in range(NUM_MOTOR):
                if kp is not None:
                    low_kp[i] = float(kp)
                if kd is not None:
                    low_kd[i] = float(kd)
            self._send_json({"ok": True})
            return
        self._send(404, "Not found")


def handle_command(name, vx, vy, vyaw):
    try:
        if loco_client is None:
            return {"code": -1, "error": "loco client not initialized"}
        if name == "damp":
            return {"code": loco_client.Damp() or 0}
        if name == "start":
            return {"code": loco_client.Start() or 0}
        if name == "lie2stand":
            return {"code": loco_client.Lie2StandUp() or 0}
        if name == "squat2stand":
            return {"code": loco_client.Squat2StandUp() or 0}
        if name == "sit":
            return {"code": loco_client.Sit() or 0}
        if name == "stand2squat":
            return {"code": loco_client.StandUp2Squat() or 0}
        if name == "zero_torque":
            return {"code": loco_client.ZeroTorque() or 0}
        if name == "stop_move":
            return {"code": loco_client.StopMove() or 0}
        if name == "move":
            fx = float(vx or 0.0)
            fy = float(vy or 0.0)
            fz = float(vyaw or 0.0)
            return {"code": loco_client.Move(fx, fy, fz, True) or 0}
        if name == "stand_height":
            return {"code": loco_client.SetStandHeight(float(vx)) or 0}
        if name == "high_stand":
            return {"code": loco_client.HighStand() or 0}
        if name == "low_stand":
            return {"code": loco_client.LowStand() or 0}
        if name == "balance_mode":
            return {"code": loco_client.BalanceStand(int(vx)) or 0}
        if name == "wave_hand":
            return {"code": loco_client.WaveHand(True) or 0}
        if name == "shake_hand":
            return {"code": loco_client.ShakeHand() or 0}
    except Exception as exc:
        return {"code": -1, "error": str(exc)}
    return {"code": -1, "error": "unknown command"}


def render_html():
    return """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>G1 Control</title>
  <style>
    :root {
      --bg:#0f1116; --panel:#151923; --panel-2:#1c2230; --accent:#f5b942;
      --accent-2:#59d0ff; --text:#e9edf2; --muted:#94a0b8; --danger:#ff6b6b;
    }
    body { margin:0; font-family:"Futura","Avenir Next","Trebuchet MS",sans-serif; background:var(--bg); color:var(--text); }
    header { padding:18px 22px; border-bottom:1px solid #1f2633; }
    .grid { display:grid; grid-template-columns:1.2fr 1fr; gap:16px; padding:16px; }
    .panel { background:var(--panel); border:1px solid #232a3a; border-radius:14px; padding:14px; }
    .panel h2 { margin:0 0 10px 0; font-size:14px; text-transform:uppercase; letter-spacing:1px; color:var(--muted); }
    .row { display:grid; grid-template-columns:repeat(auto-fit, minmax(180px, 1fr)); gap:12px; }
    .kv { background:var(--panel-2); padding:10px; border-radius:10px; border:1px solid #252e40; min-height:72px; }
    .kv span { display:block; font-size:11px; color:var(--muted); text-transform:uppercase; letter-spacing:0.8px; }
    .kv strong { display:block; font-size:16px; margin-top:6px; }
    .controls { display:grid; grid-template-columns:repeat(auto-fit, minmax(140px, 1fr)); gap:10px; }
    button { background:linear-gradient(135deg,#2b3244,#1f2433); color:var(--text); border:1px solid #2c3448; padding:10px 12px; border-radius:10px; cursor:pointer; font-size:12px; }
    button.warn { border-color:var(--danger); background:linear-gradient(135deg,#4b1f1f,#2a1414); }
    input[type="range"] { width:100%; }
    table { width:100%; border-collapse:collapse; font-size:12px; }
    th, td { padding:6px 8px; border-bottom:1px solid #242b3b; text-align:right; }
    th:first-child, td:first-child { text-align:left; }
    .joy { position:relative; width:160px; height:160px; border-radius:50%; margin:8px auto 6px auto; background:radial-gradient(circle at 30% 30%,#1e2535,#131825); border:1px solid #2b3244; touch-action:none; }
    .joy-dot { position:absolute; width:28px; height:28px; border-radius:50%; background:var(--accent-2); left:50%; top:50%; transform:translate(-50%,-50%); }
    .joy-label { text-align:center; font-size:11px; color:var(--muted); text-transform:uppercase; letter-spacing:0.6px; }
  </style>
</head>
<body>
  <header>
    <h2>G1 Web UI (Low + Loco)</h2>
  </header>
  <div class="grid">
    <div class="panel">
      <h2>Control Panel</h2>
      <div class="controls">
        <button onclick="cmd('damp')" class="warn">Damp</button>
        <button onclick="cmd('start')">Start</button>
        <button onclick="cmd('lie2stand')">Lie→Stand</button>
        <button onclick="cmd('squat2stand')">Squat→Stand</button>
        <button onclick="cmd('sit')">Sit</button>
        <button onclick="cmd('stand2squat')">Stand→Squat</button>
        <button onclick="cmd('zero_torque')" class="warn">Zero Torque</button>
        <button onclick="cmd('stop_move')" class="warn">Stop Move</button>
        <button onclick="cmd('high_stand')">High Stand</button>
        <button onclick="cmd('low_stand')">Low Stand</button>
        <button onclick="cmd('wave_hand')">Wave Hand</button>
        <button onclick="cmd('shake_hand')">Shake Hand</button>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv">
          <span>Stand Height</span>
          <div class="controls">
            <input id="stand_height" type="range" min="0" max="1" step="0.01" value="0.5">
            <button onclick="setStandHeight()">Apply</button>
          </div>
        </div>
        <div class="kv">
          <span>Balance Mode</span>
          <div class="controls">
            <button onclick="cmd('balance_mode', 0)">Mode 0</button>
            <button onclick="cmd('balance_mode', 1)" class="warn">Mode 1</button>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="kv"><span>Drive Command</span><strong id="drive_cmd">vx 0.00, vy 0.00, yaw 0.00</strong></div>
        <div class="kv"><span>Command Result</span><strong id="cmd_result">--</strong></div>
      </div>
    </div>

    <div class="panel">
      <h2>Low-Level Actuators (29 Motors)</h2>
      <div class="row">
        <div class="kv">
          <span>Warning</span>
          <strong>Low-level overrides high-level control.</strong>
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
            <input id="kd_all" type="range" min="0" max="5" step="0.1" value="1.0">
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
      <h2>Low State</h2>
      <div class="row">
        <div class="kv"><span>Mode Machine</span><strong id="mode_machine">--</strong></div>
        <div class="kv"><span>Mode PR</span><strong id="mode_pr">--</strong></div>
        <div class="kv"><span>IMU RPY</span><strong id="imu_rpy">--</strong></div>
        <div class="kv"><span>IMU Temp</span><strong id="imu_temp">--</strong></div>
      </div>
      <table style="margin-top:10px;">
        <thead>
          <tr>
            <th>Joint</th>
            <th>q</th>
            <th>dq</th>
            <th>tau</th>
            <th>temp</th>
            <th>lost</th>
            <th>err</th>
            <th>hz</th>
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

    function setStandHeight() {{
      const v = document.getElementById('stand_height').value;
      cmd('stand_height', v);
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
      for (let i = 0; i < __NUM_MOTOR__; i++) {{
        const row = document.createElement('tr');
        row.innerHTML = `
          <td>M${{i}}</td>
          <td>
            <input data-idx="${{i}}" data-key="q" type="range" min="-2.0" max="2.0" step="0.01" value="0.0">
            <span id="qv_${{i}}">0.00</span>
          </td>
          <td>
            <input data-idx="${{i}}" data-key="kp" type="range" min="0" max="60" step="1" value="20">
            <span id="kpv_${{i}}">20</span>
          </td>
          <td>
            <input data-idx="${{i}}" data-key="kd" type="range" min="0" max="5" step="0.1" value="1.0">
            <span id="kdv_${{i}}">1.00</span>
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
          fetch(`/api/low/set?idx=${{idx}}&${{key}}=${{val}}`).catch(() => {{}});
        }});
      }});
    }}

    const joyState = {{ vx:0.0, vy:0.0, vyaw:0.0, lastSend:0 }};
    function sendMove(force) {{
      const now = Date.now();
      if (!force && now - joyState.lastSend < 120) return;
      joyState.lastSend = now;
      const vx = joyState.vx.toFixed(2);
      const vy = joyState.vy.toFixed(2);
      const vyaw = joyState.vyaw.toFixed(2);
      document.getElementById('drive_cmd').textContent = `vx ${{vx}}, vy ${{vy}}, yaw ${{vyaw}}`;
      fetch(`/api/cmd?name=move&vx=${{vx}}&vy=${{vy}}&vyaw=${{vyaw}}`).catch(() => {{}});
    }}
    function setupJoystick(rootId, onMove, onReset) {{
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
        onReset();
      }}
      root.addEventListener('pointerdown', (e) => {{
        root.setPointerCapture(e.pointerId);
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
      joyState.vx = -ny * 0.6;
      joyState.vy = nx * 0.4;
      sendMove();
    }}, () => {{
      joyState.vx = 0.0; joyState.vy = 0.0; sendMove(true);
    }});
    setupJoystick('joy_yaw', (nx) => {{
      joyState.vyaw = nx * 0.8;
      sendMove();
    }}, () => {{
      joyState.vyaw = 0.0; sendMove(true);
    }});

    function update() {{
      fetch('/api/state')
        .then(r => r.json())
        .then(data => {{
          const l = data.lowstate || {{}};
          document.getElementById('mode_machine').textContent = l.mode_machine ?? '--';
          document.getElementById('mode_pr').textContent = l.mode_pr ?? '--';
          document.getElementById('imu_rpy').textContent = l.imu_rpy ? l.imu_rpy.map(v => v.toFixed(2)).join(', ') : '--';
          document.getElementById('imu_temp').textContent = l.imu_temp ?? '--';
          const tbody = document.getElementById('motors');
          tbody.innerHTML = '';
          if (l.motors) {{
            l.motors.forEach(m => {{
              const row = document.createElement('tr');
              row.innerHTML = `<td>J${{m.id}}</td><td>${{m.q.toFixed(3)}}</td><td>${{m.dq.toFixed(3)}}</td><td>${{m.tau_est.toFixed(3)}}</td><td>${{m.temp}}</td><td>${{m.lost}}</td><td>${{m.err}}</td><td>${{m.hz}}</td>`;
              tbody.appendChild(row);
            }});
          }}
        }})
        .catch(() => {{}});
    }}

    buildLowTable();
    setInterval(update, 500);
    update();
  </script>
</body>
</html>
""".replace("__NUM_MOTOR__", str(NUM_MOTOR))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--iface", default="enp2s0")
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.iface)

    sub = ChannelSubscriber(TOPIC_LOWSTATE, LowState_)
    sub.Init(lowstate_cb, 10)

    low_cmd_pub = ChannelPublisher(TOPIC_LOWCMD, LowCmd_)
    low_cmd_pub.Init()

    loco_client = LocoClient()
    loco_client.SetTimeout(5.0)
    loco_client.Init()

    t = threading.Thread(target=low_level_loop, daemon=True)
    t.start()

    server = ThreadingHTTPServer((HOST, args.port), UiServer)
    print(f"G1 web UI on http://{HOST}:{args.port} (iface={args.iface})")
    server.serve_forever()
