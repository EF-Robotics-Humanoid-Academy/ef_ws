#!/usr/bin/env python3
"""Small Wi-Fi provisioning portal for the robot's wlan0 interface."""
import html
import os
import subprocess
import tempfile
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Unitree Wi-Fi Setup</title><style>
:root{--bg:#09121e;--card:#122033;--muted:#aebed1;--line:#29415e;--blue:#4bb3fd;--ok:#38d996;--warn:#ffd166}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 85% 0,#183b5b,transparent 38%),var(--bg);color:#eef6ff;font:16px system-ui,-apple-system,Segoe UI,sans-serif}
main{width:min(760px,calc(100% - 32px));margin:48px auto}.eyebrow{color:var(--blue);font-size:.78rem;font-weight:800;letter-spacing:.12em;text-transform:uppercase}.card{background:linear-gradient(135deg,#14263d,#101c2e);border:1px solid var(--line);border-radius:22px;padding:30px;box-shadow:0 25px 60px #0006}h1{margin:8px 0;font-size:clamp(1.8rem,5vw,2.7rem)}p{color:var(--muted);line-height:1.55}.mode{display:flex;gap:10px;align-items:center;background:#0d1929;border:1px solid var(--line);padding:14px;border-radius:13px;margin:24px 0}.dot{width:10px;height:10px;border-radius:50%;background:var(--warn);box-shadow:0 0 14px var(--warn)}form{display:grid;gap:18px;margin-top:25px}label{display:grid;gap:7px;font-weight:650}input{width:100%;border:1px solid var(--line);border-radius:10px;background:#0b1727;color:#fff;padding:13px;font:inherit}input:focus{outline:2px solid var(--blue);outline-offset:2px}.check{display:flex;align-items:center;gap:9px;color:var(--muted);font-weight:500}.check input{width:auto}button{border:0;border-radius:11px;padding:14px 18px;background:var(--blue);color:#06111f;font:800 1rem inherit;cursor:pointer}button:hover{filter:brightness(1.1)}.note{margin-top:22px;padding:13px;border-left:3px solid var(--warn);background:#302918;color:#f7e6b3;border-radius:0 9px 9px 0;font-size:.92rem}.success{border-left-color:var(--ok);background:#12352c;color:#c9f8e3}
</style></head><body><main><div class="eyebrow">Unitree G1 · Network provisioning</div><section class="card"><h1>Set up Wi-Fi</h1><p>Enter the network that the robot should join. Saving replaces the current <code>wlan0</code> configuration and reconnects it.</p><div class="mode"><span class="dot"></span><span><strong>Connection setup</strong><br><small>The current Wi-Fi connection may drop briefly after saving.</small></span></div>{{NOTICE}}<form method="post" action="/save"><label>Wi-Fi network name (SSID)<input name="ssid" required maxlength="64" placeholder="e.g. RobotLab-5G"></label><label>Wi-Fi password<input name="password" type="password" minlength="8" maxlength="63" placeholder="8–63 characters"></label><label class="check"><input name="hidden" type="checkbox"> Hidden network</label><label>Country code<input name="country" value="DE" maxlength="2" pattern="[A-Za-z]{2}"></label><button>Save and connect</button></form><p class="note">Only use this page on a trusted local network: Wi-Fi credentials are submitted over HTTP to this robot.</p></section></main></body></html>"""

CONFIG_FILE = "/etc/wpa_supplicant/wpa_supplicant-wlan0.conf"
WPA_SERVICE = "wpa_supplicant@wlan0.service"
apply_lock = threading.Lock()


def write_wifi_config(ssid, password, hidden, country):
    """Atomically install the wlan0 configuration from the submitted values."""
    def quote(value):
        return value.replace("\\", "\\\\").replace('"', '\\"')

    network = ("network={\n"
               f'    ssid="{quote(ssid)}"\n'
               f'    psk="{quote(password)}"\n'
               "    key_mgmt=WPA-PSK\n")
    if hidden:
        network += "    scan_ssid=1\n"
    network += "}\n"
    content = ("ctrl_interface=/run/wpa_supplicant\n"
               "update_config=1\n"
               f"country={country}\n\n{network}")
    os.makedirs(os.path.dirname(CONFIG_FILE), mode=0o755, exist_ok=True)
    fd, temporary_path = tempfile.mkstemp(prefix=".wlan0.", dir=os.path.dirname(CONFIG_FILE))
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w") as temporary:
            temporary.write(content)
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temporary_path, CONFIG_FILE)
    finally:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)


def reconnect(ssid, password, hidden, country):
    # Delay lets the HTTP response leave before wlan0 is intentionally reset.
    time.sleep(1)
    with apply_lock:
        try:
            write_wifi_config(ssid, password, hidden, country)
            subprocess.run(["/bin/systemctl", "restart", WPA_SERVICE],
                           check=True, timeout=30)
            subprocess.run(["/usr/sbin/dhclient", "-r", "wlan0"], timeout=15,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["/usr/sbin/dhclient", "wlan0"], timeout=60,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.SubprocessError) as error:
            # Never log passwords; the SSID is enough to diagnose connection failures.
            print(f"Wi-Fi connection attempt for {ssid!r} failed: {error}", flush=True)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.respond(PAGE.replace("{{NOTICE}}", ""))

    def do_POST(self):
        if self.path != "/save":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            fields = parse_qs(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            self.respond(PAGE.replace("{{NOTICE}}", '<p class="note">Invalid form data.</p>'), HTTPStatus.BAD_REQUEST)
            return
        ssid = fields.get("ssid", [""])[0]
        password = fields.get("password", [""])[0]
        country = fields.get("country", ["DE"])[0].upper()
        if not ssid or len(ssid.encode("utf-8")) > 32 or len(password.encode("utf-8")) not in range(8, 64) or not country.isalpha() or len(country) != 2:
            self.respond(PAGE.replace("{{NOTICE}}", '<p class="note">Enter an SSID up to 32 bytes, an 8–63 byte password, and a two-letter country code.</p>'), HTTPStatus.BAD_REQUEST)
            return
        threading.Thread(target=reconnect, args=(ssid, password, "hidden" in fields, country), daemon=True).start()
        notice = ('<p class="note success"><strong>Credentials saved.</strong> '
                  f'The robot is now connecting to “{html.escape(ssid)}”. This page may disconnect briefly.</p>')
        self.respond(PAGE.replace("{{NOTICE}}", notice))

    def respond(self, page, status=HTTPStatus.OK):
        body = page.encode()
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", 8095), Handler)
    print("Wi-Fi dashboard: http://0.0.0.0:8095/", flush=True)
    server.serve_forever()
