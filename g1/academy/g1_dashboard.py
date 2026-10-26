#!/usr/bin/env python3
"""Browser dashboard for the G1, built on g1_cli's console backend.

Run from academy/: ``python3 g1_dashboard.py`` and open http://<robot>:8765
(it binds 0.0.0.0 by default; ``--host 127.0.0.1`` keeps it local).  Every g1_cli slash
command has its own tab.  /control adds a SLAM workspace: the processed
obstacle point cloud as an orbitable 3D top view, the RGB stream with convex
hulls around depth-segmented obstacles, the colorized depth stream, and SLAM
operation tools (mapping, relocation, named points, click-to-navigate, task
queues).  ``--offline`` mirrors g1_cli: no DDS or robot actions; the camera
view still works if an RGB-D ZMQ stream is running.

Two tabs have no g1_cli counterpart.  /wbc is WBC/ik_pose_cli_v4.py: 6D
end-effector IK pose control driven by the remote controller stream or a
the /control tab's arm buttons (wbc_ik.py).  /sort is
hand_pose_navigation/sorting_app.py: pick two kinds of objects and place each
on its ArUco spot (sorting.py on grasp_pipeline.py).  Both run on
sdk_wrapper.G1 like every other tab; nothing from modules/ is used for them.

g1_cli.py is imported, never modified: the G1Console class provides the agent
turn, settings persistence, topic readers and table builders reused here.

sdk_wrapper.G1 is the only robot interface.  The chat agent is composed by
g1_agent.build_agent(), which binds the agent's skills, state, SLAM and
vision grab to G1; the dashboard additionally binds its arm-ownership rule
and shared grab pipeline to the agent.
Only one of /control, /wbc, /sort and the agent moves the robot at a time
(Dashboard.arm_conflict).
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import platform
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

# --off-robot must be known before g1_cli is imported: it replaces the robot,
# DDS and agent modules with g1_dashboard_sim stand-ins so nothing hardware
# related (unitree_sdk2py, CycloneDDS, OpenAI) is imported at all.
OFF_ROBOT = "--off-robot" in sys.argv[1:]
if OFF_ROBOT:
    import g1_dashboard_sim
    g1_dashboard_sim.install_stubs()

# Importing g1_cli performs the same sys.path and CycloneDDS environment setup
# as the console before sdk_wrapper / unitree_sdk2py are loaded.
import g1_cli  # noqa: E402
from g1_cli import COMMANDS, DEFAULT_SETTINGS, WRAPPER_TOPICS, G1Console  # noqa: E402
from agent.planner import OpenAIPlanner  # noqa: E402
import sdk_wrapper  # noqa: E402

try:
    import numpy as np
except ImportError:  # SLAM cloud processing and camera views need numpy
    np = None
try:
    import cv2
except ImportError:  # camera views need OpenCV
    cv2 = None


HERE = g1_cli.HERE
HEARTBEAT_TIMEOUT_S = 2.5
# ServiceSwitch RPC timeout, and how long /services keeps polling ServiceList
# for the new status: stopping a service such as dex3_service_l takes a few
# seconds, longer than robot_state's 2 s default.
SERVICE_SWITCH_TIMEOUT_S = 10.0
SERVICE_VERIFY_S = 8.0
CONFIRM_TIMEOUT_S = 120.0
MAX_MESSAGES = 600
LIVE_CLOUD_TOPICS = ("rt/utlidar/cloud_deskewed", "rt/utlidar/cloud_livox_mid360")

# Same fields, ranges and steps as G1Console.open_settings_menu, plus a group.
SETTINGS_FIELDS = [
    ("locomotion_speed", "Locomotion speed (m/s)", None, 0.05, 0.05, 0.8, "Locomotion"),
    ("locomotion_duration", "Locomotion nudge duration (s)", None, 0.1, 0.1, 3.0, "Locomotion"),
    ("rotation_speed", "Rotation speed (rad/s)", None, 0.02, 0.02, 0.6, "Locomotion"),
    ("rotation_duration", "Rotation nudge duration (s)", None, 0.1, 0.1, 3.0, "Locomotion"),
    ("waist_locked", "Walk FSM uses locked waist", None, None, None, None, "Locomotion"),
    ("climb_fsm_id", "Climb FSM id", None, 1.0, 1.0, 9999.0, "Locomotion"),
    ("voice_reply", "Speak agent replies", None, None, None, None, "Voice & AI"),
    ("robot_volume", "Robot speech volume (%)", None, 5.0, 0.0, 100.0, "Voice & AI"),
    ("voice_language", "Voice language", ["en", "de", "fr", "es", "ar"], None, None, None, "Voice & AI"),
    ("ai_language", "AI reply language", ["en", "de", "auto"], None, None, None, "Voice & AI"),
    ("ramping", "Low-level motion ramping", None, None, None, None, "Arms & hands"),
    ("ll_max_joint_speed", "Maximum joint speed (rad/s)", None, 0.05, 0.05, 1.5, "Arms & hands"),
    ("ll_kp", "Low-level Kp", None, 1.0, 0.0, 100.0, "Arms & hands"),
    ("ll_kd", "Low-level Kd", None, 0.1, 0.0, 10.0, "Arms & hands"),
    ("ik_step_m", "End-effector step (m)", None, 0.005, 0.005, 0.10, "Arms & hands"),
    ("ik_rotation_rad", "End-effector turn step (rad)", None, 0.02, 0.02, 0.3, "Arms & hands"),
    ("tactile_threshold", "Dex3 tactile alert threshold (raw)", None, 5000.0, 0.0, 200000.0, "Arms & hands"),
]
THEMES = ["ef", "dark", "light", "ocean", "amber"]
MODE_NAMES = ["Zero torque", "Damp", "Sit", "Squat", "Prepare / stand", "Walk", "Run", "Climb",
              "Dev mode (disable ai_sport)"]
MODE_HINTS = ["Motors limp; re-enables ai_sport first when in dev mode", "Passive damping",
              "Sit down", "Squat (FSM 2)", "Stand up and balance", "Balanced walking FSM",
              "Running FSM", "Climb FSM from settings", "Disable ai_sport for low-level control"]
WIFI_ACTIONS = [
    ("status", "Show supplicant status", ["systemctl", "status", "wpa_supplicant@wlan0.service", "--no-pager"], False),
    ("restart", "Restart supplicant", ["systemctl", "restart", "wpa_supplicant@wlan0.service"], True),
    ("start", "Start supplicant", ["systemctl", "start", "wpa_supplicant@wlan0.service"], True),
    ("stop", "Stop supplicant", ["systemctl", "stop", "wpa_supplicant@wlan0.service"], True),
    ("enable", "Enable at boot", ["systemctl", "enable", "wpa_supplicant@wlan0.service"], True),
    ("disable", "Disable at boot", ["systemctl", "disable", "wpa_supplicant@wlan0.service"], True),
    ("dhcp_renew", "Request / renew DHCP", ["dhclient", "-v", "wlan0"], True),
    ("dhcp_release", "Release DHCP lease", ["dhclient", "-r", "wlan0"], True),
    ("ip", "Show wlan0 IPv4", ["ip", "-4", "addr", "show", "dev", "wlan0"], False),
]
# RealSense intrinsics used by visualizations/recognition_app_v3.py.
CAMERA_INTRINSICS = {(640, 480): (604.2, 605.1, 324.7, 242.9), (424, 240): (302.1, 302.5, 214.3, 121.4)}
# Dashboard-only tabs, listed after g1_cli's commands.
DASHBOARD_COMMANDS = {
    "/wbc": "6D end-effector IK pose control with a gamepad (ik_pose_cli_v4)",
    "/sort": "sort two kinds of objects onto ArUco drop-off spots",
}
ALL_COMMANDS = {**COMMANDS, **DASHBOARD_COMMANDS}


class LiveRequired(RuntimeError):
    """Raised for robot-only endpoints in offline mode (HTTP 409)."""


def _jsonable(value: Any, depth: int = 0) -> Any:
    if depth > 8:
        return str(value)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _jsonable(v, depth + 1) for k, v in value.items() if k != "raw"}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v, depth + 1) for v in value]
    if np is not None and isinstance(value, np.generic):
        return _jsonable(value.item(), depth + 1)
    if np is not None and isinstance(value, np.ndarray):
        return _jsonable(value.tolist(), depth + 1)
    return str(value)


def _rpc_result(label: str, result: Any) -> dict[str, Any]:
    """Decode {code, raw}: code is transport status, raw may carry errorCode."""
    code = result.get("code") if isinstance(result, dict) else result
    raw = result.get("raw") if isinstance(result, dict) else None
    try:
        ok = int(code) == 0
    except (TypeError, ValueError):
        ok = False
    detail = None
    try:
        payload = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
        if isinstance(payload, dict):
            error_code = payload.get("errorCode")
            if error_code not in (None, 0, "0"):
                ok = False
                detail = f"errorCode {error_code}: {payload.get('info') or ''}".strip()
    except Exception:
        pass
    message = detail or ("ok" if ok else f"RPC returned {code}")
    return {"label": label, "ok": ok, "code": code, "message": message, "raw": str(raw)[:600], "stamp": time.time()}


def _sudo(command: list[str]) -> list[str]:
    # Non-interactive: a web request cannot answer a terminal password prompt.
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        return command
    return ["sudo", "-n"] + command


def decode_xyz(msg: Any, max_points: int = 40000) -> Any:
    """PointCloud2 -> (N, 3) float32 with finite rows only."""
    empty = np.empty((0, 3), dtype=np.float32)
    try:
        fields = {str(f.name).lower(): f for f in list(msg.fields)}
        point_step = int(msg.point_step)
        data = bytes(msg.data)
        if not {"x", "y", "z"} <= set(fields) or point_step <= 0 or not data:
            return empty
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4", "<f4", "<f4"],
                          "offsets": [int(fields[k].offset) for k in ("x", "y", "z")], "itemsize": point_step})
        raw = np.frombuffer(data, dtype=dtype, count=len(data) // point_step)
        pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if max_points and pts.shape[0] > max_points:
            pts = pts[np.linspace(0, pts.shape[0] - 1, max_points, dtype=np.int64)]
        return pts
    except Exception:
        return empty


def voxel_keys(pts: Any, voxel: float) -> Any:
    idx = np.clip(np.floor(pts / voxel).astype(np.int64) + (1 << 20), 0, (1 << 21) - 1)
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def voxel_downsample(pts: Any, voxel: float) -> Any:
    if pts.shape[0] == 0:
        return pts
    _, first = np.unique(voxel_keys(pts, voxel), return_index=True)
    return pts[first]


def _empty_cloud_packet(floor: float = 0.0, cell: float = 0.1) -> bytes:
    return struct.pack("<IIIff", 0, 0, 0, cell, floor)


def decode_xyz_py(msg: Any, max_points: int = 20000) -> list[tuple[float, float, float]]:
    """Pure-Python decode_xyz for machines without numpy (strided subsample)."""
    try:
        fields = {str(f.name).lower(): f for f in list(msg.fields)}
        step, data = int(msg.point_step), bytes(msg.data)
        offsets = [int(fields[k].offset) for k in ("x", "y", "z")]
    except Exception:
        return []
    count = len(data) // step if step > 0 else 0
    stride = max(1, count // max_points) if max_points else 1
    out = []
    unpack = struct.unpack_from
    for i in range(0, count, stride):
        base = i * step
        x, y, z = (unpack("<f", data, base + o)[0] for o in offsets)
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append((x, y, z))
    return out


class Dashboard(G1Console):
    """G1Console adapted to HTTP: same agent/settings/topic logic, JSON views."""

    def __init__(self, robot: Any, agent: Any, offline: bool, args: argparse.Namespace):
        self._ui_lock = threading.RLock()
        self._capture = threading.local()
        self.revision = 0
        self.messages: list[dict[str, Any]] = []
        self._message_id = 0
        self._confirm_seq = 0
        self.pending: dict[str, Any] | None = None
        super().__init__(robot, agent, offline=offline)
        self.lines = []
        self.say("system", "G1 dashboard · natural language requests are routed by the OpenAI intent agent.")
        if offline:
            self.say("system", "Offline mode: robot telemetry and actions are disabled.")
        self.rpc_lock = threading.Lock()
        self.httpd: ThreadingHTTPServer | None = None
        self.service_rows: list[dict[str, Any]] = []
        self.service_message = "Service list not loaded."
        self._topics_cache: tuple[float, Any] = (0.0, None)
        self._cpu_prev: dict[str, tuple[int, int]] = {}
        self.off_robot = bool(getattr(args, "off_robot", False))
        self.control = ControlSession(self)
        self.wbc = WbcSession(self, args)
        self.sort = SortSession(self, args)
        self.slam = SlamView(robot) if robot is not None else None
        self.vision = VisionView(args.rgbd)
        threading.Thread(target=self._ticker, name="g1-dash-ticker", daemon=True).start()

    # -- transcript --------------------------------------------------------
    def say(self, role: str, message: Any) -> None:
        if not isinstance(message, str):
            try:
                message = json.dumps(message, indent=2, ensure_ascii=False, default=str)
            except Exception:
                message = str(message)
        sink = getattr(self._capture, "sink", None)
        if sink is not None:
            sink.append({"role": role, "text": message})
            return
        with self._ui_lock:
            super().say(role, message)
            del self.lines[:-4000]
            self._message_id += 1
            self.messages.append({"id": self._message_id, "role": role, "text": message, "t": time.time()})
            del self.messages[:-MAX_MESSAGES]
            self.revision += 1

    def _table(self, title: str, rows: list[tuple[str, Any]]) -> None:
        sink = getattr(self._capture, "sink", None)
        if sink is not None:
            sink.append({"title": title, "rows": [[str(k), _jsonable(v)] for k, v in rows]})
            return
        super()._table(title, rows)

    def capture(self, fn: Any) -> list[dict[str, Any]]:
        """Run a G1Console show_* method and return its tables/messages."""
        sink: list[dict[str, Any]] = []
        self._capture.sink = sink
        try:
            fn()
        finally:
            self._capture.sink = None
        return sink

    def confirm_prompt(self, skill_name: str, policy: Any) -> bool:
        """Agent per-skill confirmation, answered by the browser modal."""
        event = threading.Event()
        answer = {"approved": False}
        with self._ui_lock:
            self._confirm_seq += 1
            entry = {"id": self._confirm_seq, "skill": skill_name, "reason": getattr(policy, "reason", "") or "",
                     "event": event, "answer": answer}
            self.pending = entry
            self.revision += 1
        if not event.wait(CONFIRM_TIMEOUT_S):
            self.say("system", f"Confirmation for {skill_name} timed out; declined.")
        with self._ui_lock:
            if self.pending is entry:
                self.pending = None
                self.revision += 1
        return bool(answer["approved"])

    def answer_confirmation(self, confirm_id: Any, approved: bool) -> dict[str, Any]:
        with self._ui_lock:
            entry = self.pending
            if entry is None or entry["id"] != int(confirm_id):
                return {"ok": False, "message": "No matching confirmation is pending."}
            entry["answer"]["approved"] = bool(approved)
            self.pending = None
            self.revision += 1
        self.say("system", f"{'Approved' if approved else 'Declined'} {entry['skill']}.")
        entry["event"].set()
        return {"ok": True}

    def _ticker(self) -> None:
        while self.running:
            try:
                self._poll_background()
            except Exception as exc:
                self.say("error", f"Background poll failed: {exc}")
            time.sleep(0.1)

    def busy(self) -> bool:
        return self.chat_future is not None and not self.chat_future.done()

    def transcript(self, since_rev: int) -> dict[str, Any]:
        with self._ui_lock:
            if since_rev == self.revision:
                return {"changed": False, "rev": self.revision, "busy": self.busy()}
            pending = None if self.pending is None else {k: self.pending[k] for k in ("id", "skill", "reason")}
            return {"changed": True, "rev": self.revision, "messages": list(self.messages),
                    "busy": self.busy(), "pending": pending}

    def chat(self, text: str) -> dict[str, Any]:
        text = (text or "").strip()
        if not text:
            return {"ok": False}
        if text.startswith("/"):
            aliases = {"/?": "/help", "/quit": "/exit", "/topic": "/topics", "/service": "/services"}
            cmd = text.split()[0].lower()
            cmd = aliases.get(cmd, cmd)
            if cmd == "/clear":
                return self.clear()
            if cmd in ALL_COMMANDS:
                return {"ok": True, "tab": cmd[1:]}
            self.say("you", text)
            self.say("error", f"Unknown command: {cmd}. Try /help.")
            return {"ok": False}
        self.execute(text)  # G1Console.execute: echoes, guards busy, submits the turn
        return {"ok": True}

    def clear(self) -> dict[str, Any]:
        with self._ui_lock:
            self.lines = []
            self.messages = []
            self.revision += 1
        return {"ok": True}

    def summary(self) -> dict[str, Any]:
        battery = None
        if self.robot is not None:
            try:
                battery = ((self.robot.get_battery() or {}).get("bms") or {}).get("soc")
            except Exception:
                battery = None
        with self._ui_lock:
            pending = None if self.pending is None else {k: self.pending[k] for k in ("id", "skill", "reason")}
            rev = self.revision
        return {"offline": self.offline, "off_robot": self.off_robot, "battery": battery, "dev_mode": self.dev_mode, "busy": self.busy(),
                "pending": pending, "rev": rev, "control": self.control.active, "theme": self.theme,
                "arms_released": self.arms_released, "wbc": self.wbc.active, "sorting": self.sort.busy()}

    def meta(self) -> dict[str, Any]:
        fields = []
        for key, label, options, step, low, high, group in SETTINGS_FIELDS:
            value = DEFAULT_SETTINGS[key]
            kind = "bool" if isinstance(value, bool) else "choice" if options else "number"
            fields.append({"key": key, "label": label, "kind": kind, "options": options, "step": step,
                           "min": low, "max": high, "group": group, "integer": isinstance(value, int) and kind == "number"})
        return {"commands": ALL_COMMANDS, "settings_fields": fields, "settings": self.settings, "themes": THEMES,
                "theme": self.theme, "offline": self.offline, "topics": WRAPPER_TOPICS, "modes": MODE_NAMES,
                "mode_hints": MODE_HINTS, "wifi_actions": [{"id": a, "label": l, "sudo": s} for a, l, _c, s in WIFI_ACTIONS],
                "hostname": socket.gethostname(), "rgbd_endpoint": self.vision.endpoint, "off_robot": self.off_robot}

    def _require_live(self) -> None:
        if self.offline or self.robot is None:
            raise LiveRequired("This view requires live robot mode (started with --offline).")

    def arm_conflict(self, me: str) -> str | None:
        """Only one of /control, /wbc, /sort and the chat agent may move the robot."""
        if me != "wbc" and self.wbc.active:
            return "The /wbc IK session owns the arms; stop it in the /wbc tab first."
        if me != "control" and self.control.active:
            return "/control teleop is active; leave the /control tab first."
        if me != "sort" and self.sort.busy():
            return "The grab pipeline (/sort or a chat grab) is moving the arm; wait for it or stop it in /sort."
        if me != "agent" and self.busy():
            return "The chat agent is running a request; wait for it to finish."
        return None

    # -- /settings and /theme ---------------------------------------------
    def update_setting(self, key: str, value: Any) -> dict[str, Any]:
        field = next((f for f in SETTINGS_FIELDS if f[0] == key), None)
        if field is None:
            raise ValueError(f"Unknown setting {key!r}")
        _key, _label, options, _step, low, high, _group = field
        current = DEFAULT_SETTINGS[key]
        if isinstance(current, bool):
            new: Any = bool(value)
        elif options:
            if value not in options:
                raise ValueError(f"{key} must be one of {options}")
            new = value
        else:
            new = type(current)(max(low, min(high, float(value))))
        self.settings[key] = new
        self._persist_settings()
        return {"ok": True, "settings": self.settings}

    def set_theme(self, theme: str) -> dict[str, Any]:
        if theme not in THEMES:
            raise ValueError(f"theme must be one of {THEMES}")
        self.theme = theme
        self.settings["theme"] = theme
        self._persist_settings()
        return {"ok": True, "theme": theme}

    # -- /topics -----------------------------------------------------------
    def topics(self) -> dict[str, Any]:
        stamp, cached = self._topics_cache
        if cached is None or time.time() - stamp > 5.0:
            out = self.capture(self.show_topics)
            table = next((item for item in out if "rows" in item), {"rows": []})
            errors = [item["text"] for item in out if item.get("role") == "error"]
            rows = []
            for topic, detail in table["rows"]:
                live = bool(detail) and not str(detail).startswith("configured")
                rows.append({"topic": topic, "live": live, "type": detail or "configured"})
            cached = {"rows": rows, "discovery_error": errors[0] if errors else None, "title": table.get("title")}
            self._topics_cache = (time.time(), cached)
        return cached

    def topic(self, name: str) -> dict[str, Any]:
        self._require_live()
        if name not in WRAPPER_TOPICS:
            raise ValueError(f"Unknown topic {name!r}")
        reader = self._topic_reader(name)
        if reader is None:
            return {"topic": name, "reader": False, "lines": [
                "No sdk_wrapper subscriber is defined for this topic.",
                "It is listed for SDK completeness but has no readable data stream."]}
        message, stamp = reader.get()
        try:
            lines = self._topic_summary(name, message)
        except Exception as exc:
            lines = [f"Summary failed: {type(exc).__name__}: {exc}"]
        out: dict[str, Any] = {"topic": name, "reader": True, "lines": lines,
                               "type": type(message).__name__ if message is not None else None,
                               "age": None if not stamp else max(0.0, time.time() - stamp)}
        if message is not None and ("points" in name or name.endswith("global_map")):
            if np is not None:
                out["points"] = np.round(decode_xyz(message, max_points=2500), 3).tolist()
            else:
                out["points"] = [[round(v, 3) for v in p] for p in decode_xyz_py(message, max_points=2500)]
        if message is not None and name == "rt/lowstate":
            low = self.robot.get_lowstate() or {}
            out["joints"] = low.get("joint_positions") or []
        return out

    # -- /services ---------------------------------------------------------
    def services(self) -> dict[str, Any]:
        self._require_live()
        with self.rpc_lock:
            try:
                found = self.robot.get_service()
                self.service_rows = found
                ai_sport = next((item for item in found if item.get("name") == "ai_sport"), None)
                if ai_sport is not None and ai_sport.get("status") in (0, 1):
                    self.dev_mode = int(ai_sport["status"]) == 1
                self.service_message = f"{len(found)} services reported by robot."
            except Exception as exc:
                # Never fabricate rows when ServiceList fails (see g1_cli).
                self.service_rows = []
                self.service_message = f"ServiceList failed ({type(exc).__name__}): {exc}."
                if "3104" in self.service_message:
                    self.service_message += " RPC timeout; check robot service availability."
        return {"rows": self.service_rows, "message": self.service_message, "ok": bool(self.service_rows)}

    def set_service(self, name: str, enabled: bool) -> dict[str, Any]:
        """Same flow as g1_cli's services menu: switch, then verify via ServiceList."""
        self._require_live()
        with self.rpc_lock:
            result = self.robot.set_service(name, bool(enabled), timeout_s=SERVICE_SWITCH_TIMEOUT_S)
            code = int(result["code"])
            if code != 0:
                # A timed-out or rejected switch may still be applied by the
                # robot (stopping dex3_service takes a few seconds), so keep
                # verifying against ServiceList instead of showing stale rows.
                self.say("error", f"{name} ServiceSwitch returned RPC {code}"
                                  f"{' (timeout)' if code == 3104 else ''}; checking the live status.")
            target_status = 0 if enabled else 1
            verified = None
            deadline = time.monotonic() + SERVICE_VERIFY_S
            while True:
                try:
                    updated = self.robot.get_service()
                    self.service_rows = updated
                    current = next((item for item in updated if item.get("name") == name), None)
                    if current is not None:
                        verified = current.get("status")
                except Exception:
                    pass
                if verified == target_status or time.monotonic() >= deadline:
                    break
                time.sleep(0.5)
        if name == "ai_sport" and verified in (0, 1):
            self.dev_mode = int(verified) == 1
        rpc = "" if code == 0 else f" (ServiceSwitch RPC {code})"
        if verified == target_status:
            message, ok = f"{name} is {'ON' if enabled else 'OFF'} (verified){rpc}.", True
        elif verified in (0, 1):
            message, ok = (f"{name} is still {'ON' if verified == 0 else 'OFF'} {SERVICE_VERIFY_S:.0f}s after the "
                           f"switch request{rpc}."), False
        else:
            message, ok = f"{name} switch sent{rpc}; refreshed status could not be verified.", False
        self.service_message = message
        self.say("system" if ok else "error", message)
        return {"ok": ok, "message": message, "rows": self.service_rows}

    # -- /state ------------------------------------------------------------
    def state(self) -> dict[str, Any]:
        self._require_live()
        robot = self.robot
        out: dict[str, Any] = {"t": time.time()}
        for key, getter in (("battery", robot.get_battery), ("imu", robot.get_imus), ("odom", robot.get_odom),
                            ("slam_pose", robot.get_slam_pose)):
            try:
                out[key] = getter()
            except Exception as exc:
                out[key] = None
                out.setdefault("errors", {})[key] = f"{type(exc).__name__}: {exc}"
        try:
            low = robot.get_lowstate() or {}
            out["joints"] = {"q": low.get("joint_positions") or [], "dq": low.get("joint_velocities") or [],
                             "tau": low.get("joint_torques") or []}
        except Exception as exc:
            out["joints"] = {"q": [], "dq": [], "tau": []}
            out.setdefault("errors", {})["lowstate"] = str(exc)
        if isinstance(out.get("odom"), dict):
            out["odom"].pop("imu", None)
        return out

    def state_details(self) -> dict[str, Any]:
        self._require_live()
        with self.rpc_lock:
            return {"tables": self.capture(self.show_state)}

    # -- /wifi -------------------------------------------------------------
    def wifi_status(self) -> dict[str, Any]:
        def run(cmd: list[str]) -> str:
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
                return (result.stdout + result.stderr).strip()
            except Exception as exc:
                return f"unavailable ({exc})"
        addr = run(["ip", "-4", "-br", "addr", "show", "dev", "wlan0"])
        link = run(["iw", "dev", "wlan0", "link"]) if shutil.which("iw") else "iw not installed"
        active = run(["systemctl", "is-active", "wpa_supplicant@wlan0.service"])
        return {"address": addr, "link": link, "supplicant": active.splitlines()[0] if active else "unknown"}

    def wifi_action(self, action: str, ssid: str = "", password: str = "") -> dict[str, Any]:
        stdin = None
        if action == "configure":
            if not ssid.strip():
                raise ValueError("SSID is required")
            if not 8 <= len(password) <= 63:
                raise ValueError("Wi-Fi password must be 8–63 characters")
            # The script reads the passphrase from stdin, so it never appears
            # in the process list or shell history.
            command = _sudo(["bash", str(HERE / "setup_wpa_supplicant_wlan0.sh"), ssid.strip()])
            stdin = password + "\n"
            label = f"Configure SSID {ssid.strip()}"
            timeout = 120
        else:
            entry = next((a for a in WIFI_ACTIONS if a[0] == action), None)
            if entry is None:
                raise ValueError(f"Unknown Wi-Fi action {action!r}")
            _id, label, command, needs_sudo = entry
            command = _sudo(command) if needs_sudo else command
            timeout = 60
        if self.off_robot and command[:2] != ["ip", "-4"] and action != "status":
            # Never touch the host's network configuration while testing the UI.
            shown = [part if part != ssid.strip() or not ssid else "<ssid>" for part in command]
            code, output = 0, "[off-robot dry run] would run: " + " ".join(shown) + (" (password via stdin)" if stdin else "")
        else:
            try:
                result = subprocess.run(command, input=stdin, capture_output=True, text=True, timeout=timeout)
                code, output = result.returncode, (result.stdout + result.stderr).strip()
            except Exception as exc:
                code, output = -1, str(exc)
        if action == "status" and code == 3:  # systemctl status reports inactive as exit 3
            code = 0
        if "a password is required" in output:
            output += "\n\nsudo needs a password. Run the dashboard with sudo or allow these commands in sudoers (NOPASSWD)."
        self.say("system" if code == 0 else "error", f"Wi-Fi · {label}: exit {code}")
        return {"ok": code == 0, "code": code, "label": label, "output": output or f"Command exited with code {code}."}

    # -- /jetson -----------------------------------------------------------
    def _cpu_usage(self) -> dict[str, Any]:
        usage: dict[str, float] = {}
        try:
            for line in Path("/proc/stat").read_text().splitlines():
                if not line.startswith("cpu"):
                    continue
                name, *values = line.split()
                numbers = [int(v) for v in values]
                idle = numbers[3] + (numbers[4] if len(numbers) > 4 else 0)
                total = sum(numbers)
                prev = self._cpu_prev.get(name)
                self._cpu_prev[name] = (idle, total)
                if prev and total > prev[1]:
                    usage[name] = 100.0 * (1.0 - (idle - prev[0]) / (total - prev[1]))
        except Exception:
            pass
        cores = [usage[k] for k in sorted((k for k in usage if k != "cpu"), key=lambda k: int(k[3:]))]
        return {"total": usage.get("cpu"), "cores": cores}

    def jetson(self) -> dict[str, Any]:
        out: dict[str, Any] = {"cpu": self._cpu_usage()}
        mem: dict[str, int] = {}
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, value = line.split(":", 1)
                mem[key] = int(value.strip().split()[0]) * 1024
        except Exception:
            pass
        if mem.get("MemTotal") and mem.get("MemAvailable") is not None:
            out["memory"] = {"used": mem["MemTotal"] - mem["MemAvailable"], "total": mem["MemTotal"]}
        if mem.get("SwapTotal"):
            out["swap"] = {"used": mem["SwapTotal"] - mem.get("SwapFree", 0), "total": mem["SwapTotal"]}
        disk = shutil.disk_usage("/")
        out["disk"] = {"used": disk.used, "total": disk.total, "free": disk.free}
        temps = []
        for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
            try:
                kind = (path.parent / "type").read_text().strip()
            except Exception:
                kind = path.parent.name
            try:
                temps.append({"name": kind, "c": float(path.read_text().strip()) / 1000})
            except Exception:
                pass
        out["thermal"] = temps
        for gpu_path in ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load"):
            try:
                out["gpu"] = float(Path(gpu_path).read_text().strip()) / 10.0
                break
            except Exception:
                pass
        try:
            out["load"] = list(os.getloadavg())
        except (AttributeError, OSError):
            out["load"] = None
        try:
            out["uptime_h"] = float(Path("/proc/uptime").read_text().split()[0]) / 3600
        except Exception:
            out["uptime_h"] = None
        try:
            tables = self.capture(self.show_jetson)
        except Exception as exc:  # e.g. unreadable /proc/device-tree/model
            tables = [{"rows": [["Board", f"unavailable ({type(exc).__name__})"], ["Hostname", socket.gethostname()],
                                ["Platform", platform.platform()], ["Python", sys.version.split()[0]],
                                ["CPU cores", os.cpu_count() or "unknown"]]}]
        rows = next((t["rows"] for t in tables if "rows" in t), [])
        out["info"] = [row for row in rows if row[0] not in ("Memory", "Root disk", "Thermal zones", "Load average", "Uptime")]
        return out

    # -- /mode -------------------------------------------------------------
    def mode_state(self) -> dict[str, Any]:
        self._require_live()
        message = "Mode state unavailable; using the last known state."
        with self.rpc_lock:
            try:
                code, data = self.robot._motion_client().CheckMode()
                if int(code) == 0:
                    if isinstance(data, str):
                        try:
                            data = json.loads(data)
                        except json.JSONDecodeError:
                            data = {}
                    active_mode = sdk_wrapper._mode_name(data).strip().lower()
                    self.dev_mode = active_mode not in ("ai_sport", "ai")
                    message = f"Motion mode: {active_mode or 'released'}"
                else:
                    raise RuntimeError(code)
            except Exception:
                try:
                    ai_sport = self.robot.get_service("ai_sport")
                    if ai_sport is not None and ai_sport.get("status") in (0, 1):
                        self.dev_mode = int(ai_sport["status"]) == 1
                        message = f"ai_sport {'OFF' if self.dev_mode else 'ON'}"
                except Exception:
                    pass
            fsm = {}
            for key, api_id in (("fsm_id", sdk_wrapper.ROBOT_API_ID_LOCO_GET_FSM_ID),
                                ("fsm_mode", sdk_wrapper.ROBOT_API_ID_LOCO_GET_FSM_MODE)):
                try:
                    fsm[key] = sdk_wrapper._rpc_get_int(self.robot._client, api_id)
                except Exception:
                    fsm[key] = None
        enabled = [True] * len(MODE_NAMES) if not self.dev_mode else [True] + [False] * 7 + [True]
        return {"message": message, "dev_mode": self.dev_mode, "enabled": enabled, **fsm}

    def select_mode(self, index: int) -> dict[str, Any]:
        """Same RPC sequence as g1_cli's /mode menu."""
        self._require_live()
        index = int(index)
        if not 0 <= index < len(MODE_NAMES):
            raise ValueError("mode index out of range")
        if self.dev_mode and 0 < index < 8:
            raise ValueError("Only zero torque is available in dev mode; select it to re-enable ai_sport.")
        robot = self.robot
        with self.rpc_lock:
            if index == 0:
                if self.dev_mode:
                    client = robot._robot_state_client()
                    client.SetTimeout(5.0)
                    code = sdk_wrapper._result_code(client.ServiceSwitch("ai_sport", True))
                    if code != 0:
                        return self._mode_fail(f"Could not re-enable ai_sport before zero torque (RPC {code}).")
                    motion_code = sdk_wrapper._result_code(robot._motion_client().SelectMode("ai_sport"))
                    if motion_code != 0:
                        return self._mode_fail(f"ai_sport service is ON, but MotionSwitcher.SelectMode returned RPC {motion_code}.")
                    time.sleep(0.2)
                    self.dev_mode = False
                result = robot.zero_torque_mode()
            elif index == 1:
                result = robot.damp_mode()
            elif index == 2:
                loco = robot._client
                result = loco.Sit() if hasattr(loco, "Sit") else loco.SetFsmId(3)
            elif index == 3:
                result = robot._client.SetFsmId(2)
            elif index == 4:
                result = robot.prepare_mode()
            elif index == 5:
                result = robot.walk_mode(waist_locked=bool(self.settings["waist_locked"]))
            elif index == 6:
                result = robot.run_mode()
            elif index == 7:
                result = robot._client.SetFsmId(int(self.settings["climb_fsm_id"]))
            else:
                client = robot._robot_state_client()
                client.SetTimeout(5.0)
                code = sdk_wrapper._result_code(client.ServiceSwitch("ai_sport", False))
                if code != 0:
                    return self._mode_fail(f"Could not enter dev mode: ai_sport switch returned RPC {code}.")
                self.dev_mode = True
                result = code
        code = (int(result[0]) if isinstance(result, tuple) else int(result) if isinstance(result, (int, bool))
                else int(result.get("code", 0)) if isinstance(result, dict) else 0)
        if code != 0:
            hint = " (RPC timeout; check DDS interface and robot service availability)" if code == 3104 else ""
            return self._mode_fail(f"{MODE_NAMES[index]} RPC returned {code}; robot did not confirm the mode change.{hint}")
        message = "ai_sport disabled; dev mode is active." if index == 8 else f"{MODE_NAMES[index]} mode selected."
        self.say("system", message)
        return {"ok": True, "message": message, "dev_mode": self.dev_mode}

    def _mode_fail(self, message: str) -> dict[str, Any]:
        self.say("error", message)
        return {"ok": False, "message": message, "dev_mode": self.dev_mode}

    # -- /exit -------------------------------------------------------------
    def shutdown(self) -> dict[str, Any]:
        self.running = False
        self.control.exit()
        self.sort.close()
        self.wbc.stop()
        if self.httpd is not None:
            threading.Thread(target=self.httpd.shutdown, daemon=True).start()
        return {"ok": True}


class ControlSession:
    """Port of G1Console.open_control's loop with browser key events.

    Keys arrive through a queue and are handled on one thread, exactly like
    the curses loop. The session ends when the dashboard stops sending
    heartbeats (tab closed or hidden), which stops locomotion and releases
    the arm pose hold just like leaving /control in the console.
    """

    HAND_CYCLE = ("right", "left", "both")
    # key -> (EE DOF, direction) for wbc_ik's jog: x, y, z as in /wbc (y is
    # mirrored between the arms there, so "y+" moves both hands outward).
    MOVES = {"w": (0, 1), "s": (0, -1), "d": (1, 1), "a": (1, -1), "q": (2, 1), "e": (2, -1)}

    def __init__(self, dash: Dashboard):
        self.dash = dash
        self.keys: queue.Queue[str] = queue.Queue()
        self.thread: threading.Thread | None = None
        self.last_seen = 0.0
        self._start_lock = threading.Lock()
        self.snapshot: dict[str, Any] = {"status": "Control inactive"}

    @property
    def active(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def enter(self) -> dict[str, Any]:
        self.dash._require_live()
        with self._start_lock:
            self.last_seen = time.monotonic()
            if not self.active:
                conflict = self.dash.arm_conflict("control")
                if conflict:
                    raise ValueError(conflict)
                self.dash.sort.yield_arms()
                self.keys = queue.Queue()
                self.thread = threading.Thread(target=self._run, name="g1-dash-control", daemon=True)
                self.thread.start()
        return self.status()

    def heartbeat(self) -> None:
        if self.active:
            self.last_seen = time.monotonic()

    def exit(self) -> dict[str, Any]:
        thread = self.thread
        if thread is not None and thread.is_alive():
            self.keys.put("__exit__")
            thread.join(timeout=1.5)
        return self.status()

    def key(self, key: str) -> dict[str, Any]:
        if not self.active:
            self.enter()
        self.last_seen = time.monotonic()
        self.keys.put(str(key))
        return {"ok": True}

    def status(self) -> dict[str, Any]:
        return {**self.snapshot, "active": self.active}

    def _run(self) -> None:
        dash, robot, settings = self.dash, self.dash.robot, self.dash.settings
        try:
            robot._client.SetTimeout(2.0)
        except Exception:
            pass
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-dash-motion")
        arm_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-dash-arm")
        motion_future: concurrent.futures.Future[Any] | None = None
        motion_active = False
        arm_future: concurrent.futures.Future[Any] | None = None
        # The arm keys drive the same IK controller as /wbc (wbc_ik, no input
        # stream): warm-started IK per jog, rollback on failure, orientation
        # lock, and a continuous rt/arm_sdk hold between jogs.  It is
        # started on the first arm command and faded out when /control ends.
        ik: Any = None
        pending_move: tuple[int, int] | None = None
        pending_arm_toggle: bool | None = None
        arm_label = ""
        hand_mode = "right"
        status = "Ready"
        tactile_data: dict[str, Any] = {"left": None, "right": None}
        dex3_available: dict[str, bool | None] = {"left": None, "right": None}
        tactile_last_poll = 0.0
        tactile_error = "Waiting for Dex3 state data…"
        orientation_locked = True
        desired = (0.0, 0.0, 0.0)
        revision = 0
        sent_revision = 0
        deadline = 0.0

        def start_ik() -> Any:
            wbc_ik = dash.wbc._module()
            try:
                return wbc_ik.IKPoseController(
                    robot, pose_path=dash.wbc.pose_file, source="none", arm_control=hand_mode,
                    hand_control="off", params={"pos_step": float(settings["ik_step_m"])})
            except wbc_ik.ControllerLockError as exc:
                raise RuntimeError(str(exc)) from exc

        def ik_action(name: str, body: dict[str, Any]) -> str:
            """One wbc_ik action; errors become the status line, not a crash."""
            try:
                ik.status = ""
                ik.action(name, body)
                return ik.status
            except Exception as exc:
                return f"IK {name} failed: {exc}"

        def jog(dof: int, direction: int) -> str:
            ik.pos_step = max(0.0001, float(settings["ik_step_m"]))
            message = ik_action("jog", {"dof": dof, "dir": direction})
            failed = [arm for arm in ik._active_arms() if ik.ik_info[arm].get("success") is False]
            if failed:
                return f"IK failed for {' + '.join(failed)} arm: target rolled back, joints not moved"
            return message or "IK step applied · pose held"

        def release_arms() -> bool:
            robot.release_arms()
            return True

        def tactile_view(side: str) -> dict[str, Any]:
            snap = tactile_data.get(side)
            age = None
            if isinstance(snap, dict) and snap.get("timestamp") is not None:
                try:
                    age = time.time() - float(snap["timestamp"])
                except (TypeError, ValueError):
                    age = None
            pads = snap.get("tactile_pressures") if isinstance(snap, dict) else None
            values = []
            if isinstance(pads, (list, tuple)):
                for taxels in pads:
                    try:
                        valid = [float(v) for v in taxels if abs(float(v) - 30000.0) >= 0.5]
                    except (TypeError, ValueError):
                        valid = []
                    values.append(max(valid) if valid else None)
            detail = ""
            if not values:
                detail = (tactile_error or "Waiting for samples…") if dex3_available.get(side) is not False else "Hand not detected"
            return {"side": side, "available": dex3_available.get(side), "age": age, "values": values, "detail": detail}

        try:
            while True:
                now = time.monotonic()
                if now - self.last_seen > HEARTBEAT_TIMEOUT_S:
                    dash.say("system", "/control session ended: dashboard heartbeat lost; locomotion stopped.")
                    break
                if now - tactile_last_poll >= 0.20:
                    tactile_last_poll = now
                    try:
                        snapshots = robot.get_dex3_hand_sensors(hand="both")
                        if isinstance(snapshots, dict):
                            for side in ("left", "right"):
                                snap = snapshots.get(side)
                                if isinstance(snap, dict) and snap.get("ok") is not False and (
                                    snap.get("tactile_pressures") or snap.get("positions")
                                ):
                                    tactile_data[side] = snap
                                    dex3_available[side] = True
                                elif isinstance(snap, dict) and snap.get("error"):
                                    tactile_data[side] = snap
                                    dex3_available[side] = False
                            if dex3_available.get(hand_mode) is False:
                                hand_mode = next((side for side in ("left", "right") if dex3_available[side]), hand_mode)
                            tactile_error = ""
                    except Exception as exc:
                        tactile_error = f"Tactile read failed: {exc}"
                if motion_future is not None and motion_future.done():
                    try:
                        motion_future.result()
                    except Exception as exc:
                        status = f"Locomotion error: {exc}"
                    motion_future = None
                if arm_future is not None and arm_future.done():
                    try:
                        result = arm_future.result()
                        if arm_label == "Engaging IK":
                            ik = result
                            dash.arms_released = False
                            ik_action("orient", {"value": orientation_locked})
                            status = "IK control engaged · pose held"
                            if pending_move is not None:
                                status = jog(*pending_move)
                        elif arm_label == "release/engage":
                            dash.arms_released = not ik.armed if ik is not None else bool(result)
                            status = "Arms released" if dash.arms_released else "Arms engaged · pose held"
                        else:
                            status = f"{arm_label} completed"
                            dash.say("tool", f"{arm_label}: {result}")
                    except Exception as exc:
                        status = f"{arm_label} failed: {exc}"
                        dash.say("error", status)
                    arm_future = None
                    pending_move = None
                    arm_label = ""
                if pending_arm_toggle is not None and arm_future is None:
                    target_released = pending_arm_toggle
                    pending_arm_toggle = None
                    arm_label = "release/engage"
                    if ik is not None:
                        arm_future = arm_executor.submit(ik.action, "release" if target_released else "engage", {})
                    elif target_released:
                        arm_future = arm_executor.submit(release_arms)
                    else:
                        # Engaging means taking the arms under IK hold control.
                        arm_label = "Engaging IK"
                        arm_future = arm_executor.submit(start_ik)
                if deadline and now >= deadline:
                    if desired != (0.0, 0.0, 0.0):
                        desired = (0.0, 0.0, 0.0)
                        revision += 1
                    deadline = 0.0
                if motion_future is None and revision != sent_revision:
                    sent_revision = revision
                    if desired == (0.0, 0.0, 0.0):
                        if motion_active:
                            motion_future = executor.submit(robot.loco_stop)
                            motion_active = False
                    else:
                        vx, vy, vyaw = desired
                        motion_future = executor.submit(
                            lambda vx=vx, vy=vy, vyaw=vyaw: robot.loco_move(vx, vy, vyaw, duration_s=None)
                        )
                        motion_active = True
                self.snapshot = {
                    "status": status, "hand_mode": hand_mode, "orientation_locked": orientation_locked,
                    "arms_released": dash.arms_released, "desired": list(desired), "motion_active": motion_active,
                    "arm_busy": arm_label or (pending_arm_toggle is not None and "release/engage") or "",
                    "ik": None if ik is None else {"armed": ik.armed, "status": ik.status,
                                                   "info": {arm: ik.ik_info[arm] for arm in ik._active_arms()}},
                    "deadline_s": max(0.0, deadline - now) if deadline else 0.0,
                    "threshold": float(settings["tactile_threshold"]),
                    "tactile": {side: tactile_view(side) for side in ("left", "right")},
                    "step": {"ik_step_m": settings["ik_step_m"], "locomotion_speed": settings["locomotion_speed"],
                             "rotation_speed": settings["rotation_speed"]},
                }
                try:
                    key = self.keys.get(timeout=0.05)
                except queue.Empty:
                    continue
                if key == "__exit__":
                    break
                if key == "stop":
                    if desired != (0.0, 0.0, 0.0) or motion_active:
                        desired = (0.0, 0.0, 0.0)
                        revision += 1
                    deadline = 0.0
                    status = "Stopped"
                    continue
                if key in ("ArrowUp", "ArrowDown"):
                    speed = float(settings["locomotion_speed"])
                    nxt = (speed if key == "ArrowUp" else -speed, 0.0, 0.0)
                    if nxt != desired:
                        desired = nxt
                        revision += 1
                    deadline = time.monotonic() + float(settings["locomotion_duration"])
                    continue
                if key in ("ArrowLeft", "ArrowRight"):
                    rate = float(settings["rotation_speed"]) * (1 if key == "ArrowLeft" else -1)
                    nxt = (0.0, 0.0, rate)
                    if nxt != desired:
                        desired = nxt
                        revision += 1
                    deadline = time.monotonic() + float(settings["rotation_duration"])
                    continue
                if key == "r":
                    # Flush queued keys and cancel an in-flight IK ramp.
                    desired = (0.0, 0.0, 0.0)
                    revision += 1
                    deadline = 0.0
                    try:
                        while True:
                            self.keys.get_nowait()
                    except queue.Empty:
                        pass
                    if pending_arm_toggle is None:
                        pending_arm_toggle = (ik.armed if ik is not None else not dash.arms_released)
                        status = "Release/engage queued…" if arm_future is not None else "Release/engage…"
                    continue
                if key == "m":
                    hand_mode = self.HAND_CYCLE[(self.HAND_CYCLE.index(hand_mode) + 1) % len(self.HAND_CYCLE)]
                    if ik is not None:
                        ik_action("arm_mode", {"mode": hand_mode})
                    status = f"Selected {hand_mode} hand control"
                    continue
                if key == "f":
                    orientation_locked = not orientation_locked
                    if ik is not None:
                        ik_action("orient", {"value": orientation_locked})
                    status = (f"EE orientation {'locked' if orientation_locked else 'free (shoulder + elbow only)'} "
                              "during position moves")
                    continue
                if key in ("o", "c"):
                    if arm_future is not None or pending_arm_toggle is not None:
                        status = "Arm busy; command not queued"
                        continue
                    selected_sides = ("left", "right") if hand_mode == "both" else (hand_mode,)
                    selected_sides = tuple(side for side in selected_sides if dex3_available.get(side) is True)
                    if not selected_sides:
                        status = f"Dex3 {hand_mode} hand unavailable; no command sent"
                        continue
                    arm_label = "Open hand" if key == "o" else "Close hand"
                    method = robot.open_dex3_hand if key == "o" else robot.close_dex3_hand
                    arm_future = arm_executor.submit(lambda m=method, sides=selected_sides: {
                        side: m(hand=side) for side in sides
                    })
                    status = f"{arm_label}…"
                    continue
                if key in self.MOVES:
                    if arm_future is not None or pending_arm_toggle is not None:
                        status = "Arm busy; command not queued"
                        continue
                    if ik is None:
                        pending_move = self.MOVES[key]
                        arm_label = "Engaging IK"
                        arm_future = arm_executor.submit(start_ik)
                        status = "Taking the arms under IK control (1 s ramp)…"
                        continue
                    if not ik.armed:
                        status = "Arms are released; press r to engage before moving"
                        continue
                    status = jog(*self.MOVES[key])
        except Exception as exc:
            dash.say("error", f"Control error: {type(exc).__name__}: {exc}")
        finally:
            try:
                if motion_future is not None and not motion_future.done():
                    motion_future.result(timeout=0.5)
            except Exception:
                pass
            if motion_active:
                try:
                    executor.submit(robot.loco_stop)
                except Exception:
                    pass
            if arm_future is not None:
                try:
                    result = arm_future.result(timeout=5.0)
                    if arm_label == "Engaging IK":
                        ik = result
                except Exception:
                    pass
            if ik is not None:
                # Fades arm authority out over 3 s and frees the IK lock.
                ik.close()
                dash.arms_released = True
            executor.shutdown(wait=False)
            arm_executor.shutdown(wait=False)
            try:
                robot._client.SetTimeout(10.0)
            except Exception:
                pass
            self.snapshot = {"status": "Control inactive", "desired": [0.0, 0.0, 0.0]}


class WbcSession:
    """Owns the /wbc IK pose controller (wbc_ik.IKPoseController).

    Unlike /control the session outlives the browser tab, because the
    gamepad -- not the page -- drives it, and stale gamepad input already
    stops locomotion.  It ends with Stop, /exit or dashboard shutdown, which
    fades arm authority out exactly like quitting ik_pose_cli_v4.py.
    """

    def __init__(self, dash: Dashboard, args: argparse.Namespace):
        self.dash = dash
        self.pose_file = Path(args.wbc_pose_file)
        self.remote_url = str(args.remote_url)
        self.controller: Any = None
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        controller = self.controller
        return controller is not None and not controller._closed

    @staticmethod
    def _module() -> Any:
        try:
            import wbc_ik
        except ImportError as exc:
            raise LiveRequired(f"/wbc needs numpy and hand_pose_navigation's arm_fk/arm_ik: {exc}") from exc
        return wbc_ik

    def start(self, body: dict[str, Any]) -> dict[str, Any]:
        self.dash._require_live()
        wbc_ik = self._module()
        with self._lock:
            if not self.active:
                conflict = self.dash.arm_conflict("wbc")
                if conflict:
                    raise ValueError(conflict)
                self.dash.sort.yield_arms()
                try:
                    self.controller = wbc_ik.IKPoseController(
                        self.dash.robot, pose_path=self.pose_file,
                        remote_url=str(body.get("url") or self.remote_url).strip(),
                        arm_control=str(body.get("arm_control") or "right"),
                        hand_control=str(body.get("hand_control") or "off"),
                        hand_grip=float(body.get("hand_grip") or 0.0),
                        params=body.get("params") if isinstance(body.get("params"), dict) else {},
                    )
                except wbc_ik.ControllerLockError as exc:
                    raise ValueError(str(exc)) from exc
                self.remote_url = self.controller.remote_url
                self.dash.say("system", f"/wbc session started ({self.controller.status}).")
        return self.status()

    def stop(self) -> dict[str, Any]:
        controller = self.controller
        if controller is not None and not controller._closed:
            # Stays registered (and owning the arms) until the fade-out is done.
            controller.close()
            self.dash.say("system", "/wbc session stopped: locomotion stopped, arm authority faded out.")
        with self._lock:
            if self.controller is controller:
                self.controller = None
        return self.status()

    def status(self) -> dict[str, Any]:
        controller = self.controller
        if controller is not None and not controller._closed:
            return controller.snapshot()
        out: dict[str, Any] = {"active": False, "remote_url": self.remote_url, "pose_file": str(self.pose_file)}
        try:
            out["defaults"] = {key: default for key, (default, _lo, _hi) in self._module().PARAMS.items()}
        except LiveRequired as exc:
            out["error"] = str(exc)
        return out

    def _controller(self) -> Any:
        controller = self.controller
        if controller is None or controller._closed:
            raise ValueError("Start the /wbc session first.")
        return controller

    def action(self, name: str, body: dict[str, Any]) -> dict[str, Any]:
        return self._controller().action(name, body)



class SortSession:
    """Lazily builds the grab pipeline and the /sort layer on first use.

    The pipeline (grasp_pipeline.GraspPipeline) is the dashboard's only one:
    /sort and the chat agent's grab/reach skills share its arm lock and pose
    hold, so they cannot command the arm at the same time.
    """

    MOTION_ACTIONS = ("sort", "retract", "engage_arms", "hand_open", "hand_close")

    def __init__(self, dash: Dashboard, args: argparse.Namespace):
        self.dash = dash
        self.args = args
        self.sorter: Any = None
        self._pipeline: Any = None
        self._lock = threading.Lock()

    def pipeline(self) -> Any:
        self.dash._require_live()
        if self.dash.off_robot:
            raise LiveRequired("The grab pipeline needs the robot's RGB-D stream, detector and arm; "
                               "it is not simulated in --off-robot mode.")
        with self._lock:
            if self._pipeline is None:
                if np is None or cv2 is None:
                    raise LiveRequired("The grab pipeline needs numpy and opencv-contrib-python.")
                try:
                    import grasp_pipeline
                except ImportError as exc:
                    raise LiveRequired(f"The grab pipeline could not load its modules: {exc}") from exc
                openai_vision = self.args.vision_backend == "openai"
                self._pipeline = grasp_pipeline.GraspPipeline(
                    self.dash.robot, self.args.rgbd, vision_backend=self.args.vision_backend,
                    vision_model=self.args.openai_vision_model if openai_vision else self.args.vision_model)
        return self._pipeline

    def get(self) -> Any:
        pipeline = self.pipeline()
        with self._lock:
            if self.sorter is None:
                try:
                    import sorting
                except ImportError as exc:
                    raise LiveRequired(f"/sort could not load its modules: {exc}") from exc
                self.sorter = sorting.Sorter(pipeline)
        return self.sorter

    def busy(self) -> bool:
        if self.sorter is not None and self.sorter.busy():
            return True
        return self._pipeline is not None and self._pipeline.busy()

    def yield_arms(self) -> None:
        if self._pipeline is not None and not self.busy():
            self._pipeline.yield_arms()

    def action(self, name: str) -> dict[str, Any]:
        sorter = self.get()
        if name in self.MOTION_ACTIONS:
            conflict = self.dash.arm_conflict("sort")
            if conflict:
                raise ValueError(conflict)
        result = sorter.action(name)
        if name == "sort":
            self.dash.say("system", "/sort started.")
        return result

    def config(self) -> dict[str, Any]:
        sorter = self.get()
        return {**sorter.config_view(), "options": sorter.options(), "meta": sorter.meta()}

    def frame(self, view: str) -> bytes | None:
        ok, jpeg = cv2.imencode(".jpg", self.get().render(view), [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpeg.tobytes() if ok else None

    def stream(self, handler: BaseHTTPRequestHandler, view: str, fps: float = 6.0) -> None:
        self.get()
        handler.close_connection = True
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Connection", "close")
        handler.end_headers()
        try:
            while self.dash.running:
                started = time.monotonic()
                image = self.frame(view)
                if image is not None:
                    handler.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                        + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                    handler.wfile.flush()
                time.sleep(max(0.02, 1.0 / fps - (time.monotonic() - started)))
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def close(self) -> None:
        if self.sorter is not None:
            self.sorter.close()


class SlamView:
    """Obstacle point cloud processing plus SLAM operations for /control.

    Map-frame SLAM clouds (sdk_wrapper.SLAM_POINT_TOPICS) are height-filtered
    against an estimated floor and accumulated into a voxel map. The live
    LiDAR scan is filtered the same way and transformed by the SLAM pose.
    The browser receives a compact int16 (centimetre) binary packet.
    """

    def __init__(self, robot: Any):
        self.robot = robot
        self.lock = threading.RLock()
        self.rpc_lock = threading.Lock()
        self.params: dict[str, Any] = {
            "voxel": 0.05, "auto_floor": True, "floor_z": 0.0, "clearance": 0.12, "max_height": 1.8,
            "cell": 0.10, "max_points": 25000, "map_cap": 90000, "live_frame": "body",
            "live_topic": LIVE_CLOUD_TOPICS[0], "accumulate_live": False,
        }
        self.map_topics = list(getattr(robot, "_clouds", []))
        self.live_readers: dict[str, Any] = {}
        for topic in LIVE_CLOUD_TOPICS:
            try:
                self.live_readers[topic] = robot._latest(topic, sdk_wrapper.PointCloud2_)
            except Exception:
                pass
        self.map_path = getattr(robot, "_slam_map_path", "/home/unitree/test.pcd")
        self.points_path = HERE / "slam_points.json"
        self.mapping = False
        self.relocated = False
        self.last_action: dict[str, Any] = {"label": "startup", "ok": True, "message": "ready", "stamp": time.time()}
        self.goal: dict[str, float] | None = None
        self.tasks: list[dict[str, Any]] = []
        self.task_state: dict[str, Any] = {"running": False}
        self._task_cancel = threading.Event()
        self._task_thread: threading.Thread | None = None
        self.last_viewer = 0.0
        self._seen: dict[str, float] = {}
        self._floor: dict[str, float] = {}
        self.stats: dict[str, Any] = {}
        self.packet = _empty_cloud_packet()
        self._packet_dirty = True
        self._reset_map()
        threading.Thread(target=self._worker, name="g1-dash-slam", daemon=True).start()

    # -- cloud processing --------------------------------------------------
    def _reset_map(self) -> None:
        with self.lock:
            self._py_map: dict[tuple[int, int, int], tuple[float, float, float, float]] = {}
            self._py_live: list[tuple[float, float, float]] = []
            if np is not None:
                self.map_keys = np.empty(0, dtype=np.int64)
                self.map_pts = np.empty((0, 3), dtype=np.float32)
                self.map_ts = np.empty(0, dtype=np.float64)
                self.live_pts = np.empty((0, 3), dtype=np.float32)
            self._floor.clear()
            self._packet_dirty = True

    def _floor_z(self, pts: Any, which: str) -> float:
        if not self.params["auto_floor"]:
            return float(self.params["floor_z"])
        estimate = float(np.percentile(pts[:, 2], 3.0))
        previous = self._floor.get(which)
        value = estimate if previous is None else 0.85 * previous + 0.15 * estimate
        self._floor[which] = value
        return value

    def _obstacles(self, pts: Any, floor: float) -> Any:
        z = pts[:, 2]
        keep = (z > floor + float(self.params["clearance"])) & (z < floor + float(self.params["max_height"]))
        return pts[keep]

    def _merge(self, pts: Any, stamp: float) -> None:
        if pts.shape[0] == 0:
            return
        voxel = float(self.params["voxel"])
        pts = voxel_downsample(pts, voxel)
        with self.lock:
            keys = np.concatenate([voxel_keys(pts, voxel), self.map_keys])
            all_pts = np.concatenate([pts, self.map_pts])
            all_ts = np.concatenate([np.full(pts.shape[0], stamp), self.map_ts])
            keys, first = np.unique(keys, return_index=True)  # first index = newest sample
            all_pts, all_ts = all_pts[first], all_ts[first]
            cap = int(self.params["map_cap"])
            if keys.shape[0] > cap:
                newest = np.argsort(all_ts)[-cap:]
                keys, all_pts, all_ts = keys[newest], all_pts[newest], all_ts[newest]
            self.map_keys, self.map_pts, self.map_ts = keys, all_pts, all_ts

    def pose(self) -> tuple[float, float, float] | None:
        pose = None
        try:
            pose = self.robot.get_slam_pose()
        except Exception:
            pose = None
        if pose is None:
            try:
                pose = self.robot.get_slam_odom_pose()
            except Exception:
                pose = None
        return None if pose is None else (float(pose[0]), float(pose[1]), float(pose[2]))

    def _worker(self) -> None:
        while True:
            time.sleep(0.2)
            watching = time.monotonic() - self.last_viewer < 30.0
            if not watching and not self.mapping:
                continue
            started = time.perf_counter()
            try:
                if np is None:
                    self._process_py()
                    if watching and self._packet_dirty:
                        self._build_packet_py()
                else:
                    self._process()
                    if watching and self._packet_dirty:
                        self._build_packet()
            except Exception as exc:
                self.stats["error"] = f"{type(exc).__name__}: {exc}"
            self.stats["proc_ms"] = round((time.perf_counter() - started) * 1000.0, 1)

    def _process(self) -> None:
        for topic, reader in self.map_topics:
            msg, stamp = reader.get()
            if msg is None or self._seen.get(topic) == stamp:
                continue
            self._seen[topic] = stamp
            pts = decode_xyz(msg)
            if pts.shape[0] < 10:
                continue
            floor = self._floor_z(pts, "map")
            self._merge(self._obstacles(pts, floor), stamp)
            self._packet_dirty = True
        topic = self.params["live_topic"]
        reader = self.live_readers.get(topic)
        if reader is None:
            return
        msg, stamp = reader.get()
        if msg is None or self._seen.get(topic) == stamp:
            return
        self._seen[topic] = stamp
        pts = decode_xyz(msg, max_points=25000)
        if pts.shape[0] < 10:
            return
        floor_live = self._floor_z(pts, "live")
        pts = self._obstacles(pts, floor_live).copy()
        pose = self.pose()
        if self.params["live_frame"] == "body" and pose is not None:
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            px, py = pts[:, 0].copy(), pts[:, 1].copy()
            pts[:, 0] = c * px - s * py + x
            pts[:, 1] = s * px + c * py + y
        # Display the live scan on the map floor so both layers share heights.
        pts[:, 2] += self._floor.get("map", floor_live) - floor_live
        pts = voxel_downsample(pts, float(self.params["voxel"]))
        with self.lock:
            self.live_pts = pts
        if self.params["accumulate_live"]:
            self._merge(pts, stamp)
        self._packet_dirty = True

    def _build_packet(self) -> None:
        with self.lock:
            map_pts, live_pts = self.map_pts, self.live_pts
        max_points = int(self.params["max_points"])
        if map_pts.shape[0] > max_points:
            map_pts = map_pts[np.linspace(0, map_pts.shape[0] - 1, max_points, dtype=np.int64)]
        cell = float(self.params["cell"])
        xy = np.concatenate([map_pts[:, :2], live_pts[:, :2]])
        cells = (np.unique(np.floor(xy / cell).astype(np.int32), axis=0) if xy.shape[0]
                 else np.empty((0, 2), dtype=np.int32))
        cells = np.clip(cells, -32767, 32767).astype("<i2")

        def centimetres(arr: Any) -> Any:
            return np.clip(np.round(arr * 100.0), -32767, 32767).astype("<i2")

        floor = self._floor.get("map", self._floor.get("live", float(self.params["floor_z"])))
        header = struct.pack("<IIIff", map_pts.shape[0], live_pts.shape[0], cells.shape[0], cell, floor)
        self.packet = header + centimetres(map_pts).tobytes() + centimetres(live_pts).tobytes() + cells.tobytes()
        self._packet_dirty = False
        self.stats.update({"map_voxels": int(self.map_pts.shape[0]), "sent_map": int(map_pts.shape[0]),
                           "live": int(live_pts.shape[0]), "cells": int(cells.shape[0]), "floor_z": floor})

    # Pure-Python equivalents of _process/_build_packet (no numpy installed).
    def _floor_py(self, pts: list[tuple[float, float, float]], which: str) -> float:
        if not self.params["auto_floor"]:
            return float(self.params["floor_z"])
        zs = sorted(p[2] for p in pts)
        estimate = zs[int(0.03 * (len(zs) - 1))]
        previous = self._floor.get(which)
        value = estimate if previous is None else 0.85 * previous + 0.15 * estimate
        self._floor[which] = value
        return value

    def _obstacles_py(self, pts: list[tuple[float, float, float]], floor: float) -> list[tuple[float, float, float]]:
        low, high = floor + float(self.params["clearance"]), floor + float(self.params["max_height"])
        return [p for p in pts if low < p[2] < high]

    def _voxel_py(self, pts: list[tuple[float, float, float]]) -> dict[tuple[int, int, int], tuple[float, float, float]]:
        voxel = float(self.params["voxel"])
        return {(math.floor(x / voxel), math.floor(y / voxel), math.floor(z / voxel)): (x, y, z) for x, y, z in pts}

    def _process_py(self) -> None:
        for topic, reader in self.map_topics:
            msg, stamp = reader.get()
            if msg is None or self._seen.get(topic) == stamp:
                continue
            self._seen[topic] = stamp
            pts = decode_xyz_py(msg)
            if len(pts) < 10:
                continue
            self._merge_py(self._obstacles_py(pts, self._floor_py(pts, "map")), stamp)
            self._packet_dirty = True
        topic = self.params["live_topic"]
        reader = self.live_readers.get(topic)
        if reader is None:
            return
        msg, stamp = reader.get()
        if msg is None or self._seen.get(topic) == stamp:
            return
        self._seen[topic] = stamp
        pts = decode_xyz_py(msg, max_points=12000)
        if len(pts) < 10:
            return
        floor_live = self._floor_py(pts, "live")
        pts = self._obstacles_py(pts, floor_live)
        shift = self._floor.get("map", floor_live) - floor_live
        pose = self.pose()
        if self.params["live_frame"] == "body" and pose is not None:
            x0, y0, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            pts = [(c * x - s * y + x0, s * x + c * y + y0, z + shift) for x, y, z in pts]
        else:
            pts = [(x, y, z + shift) for x, y, z in pts]
        live = list(self._voxel_py(pts).values())
        with self.lock:
            self._py_live = live
        if self.params["accumulate_live"]:
            self._merge_py(live, stamp)
        self._packet_dirty = True

    def _merge_py(self, pts: list[tuple[float, float, float]], stamp: float) -> None:
        with self.lock:
            for key, (x, y, z) in self._voxel_py(pts).items():
                self._py_map[key] = (x, y, z, stamp)
            cap = int(self.params["map_cap"])
            if len(self._py_map) > cap:
                newest = sorted(self._py_map.items(), key=lambda item: item[1][3])[-cap:]
                self._py_map = dict(newest)

    def _build_packet_py(self) -> None:
        with self.lock:
            map_pts = [v[:3] for v in self._py_map.values()]
            live_pts = list(self._py_live)
        max_points = int(self.params["max_points"])
        if len(map_pts) > max_points:
            stride = len(map_pts) / max_points
            map_pts = [map_pts[int(i * stride)] for i in range(max_points)]
        cell = float(self.params["cell"])
        cells = sorted({(math.floor(x / cell), math.floor(y / cell)) for x, y, _z in map_pts + live_pts})

        def clip(v: float) -> int:
            return max(-32767, min(32767, int(round(v))))

        floor = self._floor.get("map", self._floor.get("live", float(self.params["floor_z"])))
        flat = [clip(v * 100.0) for p in map_pts + live_pts for v in p] + [clip(v) for c in cells for v in c]
        header = struct.pack("<IIIff", len(map_pts), len(live_pts), len(cells), cell, floor)
        self.packet = header + struct.pack(f"<{len(flat)}h", *flat)
        self._packet_dirty = False
        self.stats.update({"map_voxels": len(self._py_map), "sent_map": len(map_pts), "live": len(live_pts),
                           "cells": len(cells), "floor_z": floor, "backend": "pure-python"})

    def cloud(self) -> bytes:
        self.last_viewer = time.monotonic()
        return self.packet

    # -- status ------------------------------------------------------------
    def load_points(self) -> dict[str, list[float]]:
        try:
            raw = json.loads(self.points_path.read_text())
        except (OSError, json.JSONDecodeError):
            return {}
        points = {}
        for name, value in (raw.items() if isinstance(raw, dict) else []):
            if isinstance(value, dict):  # slam_util.SlamUtility format
                value = [value.get("x", 0.0), value.get("y", 0.0), value.get("yaw", 0.0)]
            try:
                points[str(name)] = [float(value[0]), float(value[1]), float(value[2])]
            except (TypeError, ValueError, IndexError):
                pass
        return points

    def status(self) -> dict[str, Any]:
        self.last_viewer = time.monotonic()
        now = time.time()
        topics = []
        for topic, reader in self.map_topics + list(self.live_readers.items()):
            _msg, stamp = reader.get()
            topics.append({"topic": topic, "age": None if not stamp else now - stamp})
        for topic, reader in (("rt/slam_info", self.robot._slam_info), ("rt/slam_key_info", self.robot._slam_key)):
            _msg, stamp = reader.get()
            topics.append({"topic": topic, "age": None if not stamp else now - stamp})
        try:
            notice = self.robot.get_slam_notice()
        except Exception:
            notice = None
        with self.lock:
            return {"available": np is not None, "pose": self.pose(), "notice": notice, "mapping": self.mapping,
                    "relocated": self.relocated, "map_path": self.map_path, "last_action": self.last_action,
                    "goal": self.goal, "tasks": list(self.tasks), "task_state": dict(self.task_state),
                    "points": self.load_points(), "params": self.params, "stats": self.stats, "topics": topics,
                    "live_topics": list(self.live_readers)}

    # -- operations --------------------------------------------------------
    def _record(self, result: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            self.last_action = result
        return result

    def set_params(self, body: dict[str, Any]) -> dict[str, Any]:
        numeric = {"voxel": (0.02, 0.5), "floor_z": (-5.0, 5.0), "clearance": (0.0, 1.0), "max_height": (0.2, 4.0),
                   "cell": (0.05, 1.0), "max_points": (2000, 60000)}
        reset = False
        for key, value in body.items():
            if key in numeric:
                low, high = numeric[key]
                cast = int if key == "max_points" else float
                self.params[key] = cast(max(low, min(high, float(value))))
                reset |= key in ("voxel", "clearance", "max_height", "floor_z")
            elif key in ("auto_floor", "accumulate_live"):
                self.params[key] = bool(value)
                reset |= key == "auto_floor"
            elif key == "live_frame" and value in ("body", "world"):
                self.params[key] = value
            elif key == "live_topic" and value in self.live_readers:
                self.params[key] = value
        if reset:
            self._reset_map()
            self._seen.clear()  # re-ingest the latest map frames with the new filter
        self._packet_dirty = True
        return {"ok": True, "params": self.params}

    def action(self, body: dict[str, Any]) -> dict[str, Any]:
        name = str(body.get("action", ""))
        robot = self.robot
        if name == "clear_view":
            self._reset_map()
            self._seen.clear()
            return self._record({"label": "clear_view", "ok": True, "message": "Cloud view cleared.", "stamp": time.time()})
        if name == "add_point":
            point_name = str(body.get("name", "")).strip()
            if not point_name:
                raise ValueError("Point name is required")
            pose = robot.add_point(point_name, points_path=str(self.points_path))
            return self._record({"label": "add_point", "ok": True, "message": f"Saved {point_name} at {pose}", "stamp": time.time()})
        if name == "remove_point":
            point_name = str(body.get("name", ""))
            robot.remove_point(point_name, points_path=str(self.points_path))
            return self._record({"label": "remove_point", "ok": True, "message": f"Removed {point_name}", "stamp": time.time()})
        if name == "queue_add":
            task = {"x": float(body["x"]), "y": float(body["y"]), "yaw": float(body.get("yaw", 0.0)),
                    "name": str(body.get("name") or f"#{len(self.tasks) + 1}")}
            with self.lock:
                self.tasks.append(task)
            return {"ok": True, "message": f"Queued {task['name']}", "tasks": self.tasks}
        if name == "queue_add_current":
            pose = self.pose()
            if pose is None:
                raise ValueError("No SLAM pose available yet")
            with self.lock:
                self.tasks.append({"x": pose[0], "y": pose[1], "yaw": pose[2], "name": f"#{len(self.tasks) + 1}"})
            return {"ok": True, "message": "Queued current pose", "tasks": self.tasks}
        if name == "queue_add_point":
            point = self.load_points().get(str(body.get("name")))
            if point is None:
                raise ValueError("Unknown point")
            with self.lock:
                self.tasks.append({"x": point[0], "y": point[1], "yaw": point[2], "name": str(body.get("name"))})
            return {"ok": True, "message": f"Queued {body.get('name')}", "tasks": self.tasks}
        if name == "queue_remove":
            with self.lock:
                index = int(body.get("index", -1))
                if 0 <= index < len(self.tasks):
                    self.tasks.pop(index)
            return {"ok": True, "tasks": self.tasks}
        if name == "queue_clear":
            with self.lock:
                self.tasks.clear()
            return {"ok": True, "tasks": self.tasks}
        if name == "queue_run":
            with self.lock:
                tasks = list(self.tasks)
            if not tasks:
                raise ValueError("Queue is empty")
            return self._start_tasks(tasks, clear_queue=True)
        if name == "goto_point":
            point = self.load_points().get(str(body.get("name")))
            if point is None:
                raise ValueError("Unknown point")
            return self._start_tasks([{"x": point[0], "y": point[1], "yaw": point[2], "name": str(body.get("name"))}])
        if name == "cancel":
            self._task_cancel.set()
            with self.rpc_lock:
                result = _rpc_result("pause_nav", robot.pause_nav())
            return self._record({**result, "message": "Task sequence cancelled; " + result["message"]})
        with self.rpc_lock:
            if name == "start_mapping":
                slam_type = "outdoor" if body.get("slam_type") == "outdoor" else "indoor"
                self._reset_map()
                self._seen.clear()
                result = _rpc_result("start_mapping", robot.start_mapping(slam_type))
                self.mapping = result["ok"]
                self.relocated = False
            elif name == "save_map":
                path = str(body.get("path") or self.map_path)
                result = _rpc_result("end_mapping", robot.stop_mapping(save_path=path))
                if result["ok"]:
                    self.mapping = False
                    self.map_path = getattr(robot, "_slam_map_path", path)
            elif name == "close_slam":
                self._task_cancel.set()
                result = _rpc_result("close_slam", robot.stop_mapping(save=False))
                self.mapping = False
                self.relocated = False
            elif name == "relocate":
                path = str(body.get("path") or self.map_path)
                pose = self.pose()
                if pose is None or all(abs(v) < 1e-5 for v in pose):
                    # Refuse rather than silently relocating at the origin.
                    result = {"label": "relocate", "ok": False, "code": None, "stamp": time.time(),
                              "message": "No valid non-zero SLAM pose has been received yet."}
                else:
                    result = _rpc_result("relocate", robot.relocate(map_path=path, pose=pose))
                    self.relocated = result["ok"]
                    if result["ok"]:
                        self.map_path = path
            elif name == "pose_nav":
                x, y, yaw = float(body["x"]), float(body["y"]), float(body.get("yaw", 0.0))
                result = _rpc_result("pose_nav", robot.pose_nav(x, y, yaw))
                if result["ok"]:
                    self.goal = {"x": x, "y": y, "yaw": yaw}
            elif name == "pause":
                result = _rpc_result("pause_nav", robot.pause_nav())
            elif name == "resume":
                result = _rpc_result("resume_nav", robot.resume_nav())
            else:
                raise ValueError(f"Unknown SLAM action {name!r}")
        return self._record(result)

    def _start_tasks(self, tasks: list[dict[str, Any]], clear_queue: bool = False) -> dict[str, Any]:
        if self._task_thread is not None and self._task_thread.is_alive():
            raise ValueError("A navigation sequence is already running; cancel it first")
        self._task_cancel = threading.Event()
        if clear_queue:
            with self.lock:
                self.tasks.clear()
        self._task_thread = threading.Thread(target=self._run_tasks, args=(tasks, self._task_cancel),
                                             name="g1-dash-nav", daemon=True)
        self._task_thread.start()
        return {"ok": True, "message": f"Navigating {len(tasks)} target(s)"}

    def _run_tasks(self, tasks: list[dict[str, Any]], cancel: threading.Event, tolerance: float = 0.35,
                   timeout: float = 120.0) -> None:
        """Sequential pose_nav + arrival wait, like Unitree's keyDemo taskLoopFun."""
        total = len(tasks)
        for index, task in enumerate(tasks):
            if cancel.is_set():
                break
            self.task_state = {"running": True, "index": index, "total": total, "target": task, "phase": "sending"}
            with self.rpc_lock:
                result = _rpc_result("pose_nav", self.robot.pose_nav(task["x"], task["y"], task["yaw"]))
            self._record(result)
            if not result["ok"]:
                self.task_state = {"running": False, "index": index, "total": total, "phase": "failed",
                                   "message": f"{task['name']}: {result['message']}"}
                return
            self.goal = {"x": task["x"], "y": task["y"], "yaw": task["yaw"]}
            sent = time.time()
            deadline = sent + timeout
            arrived = False
            while time.time() < deadline and not cancel.is_set():
                try:
                    notice = self.robot.get_slam_notice()
                except Exception:
                    notice = None
                blocked = bool(notice and notice.get("stamp", 0.0) >= sent and notice.get("obstacle_blocked"))
                self.task_state = {"running": True, "index": index, "total": total, "target": task,
                                   "phase": "blocked" if blocked else "moving", "elapsed": time.time() - sent}
                if notice and notice.get("stamp", 0.0) >= sent and notice.get("is_arrived") is True:
                    arrived = True
                    break
                pose = self.pose()
                if pose is not None and math.hypot(pose[0] - task["x"], pose[1] - task["y"]) <= tolerance:
                    arrived = True
                    break
                cancel.wait(0.2)
            if not arrived:
                phase = "cancelled" if cancel.is_set() else "timeout"
                self.task_state = {"running": False, "index": index, "total": total, "phase": phase,
                                   "message": f"{task['name']}: {phase}"}
                return
        self.goal = None
        self.task_state = {"running": False, "total": total, "phase": "cancelled" if cancel.is_set() else "done",
                           "message": "Sequence cancelled" if cancel.is_set() else f"Reached {total} target(s)"}


class VisionView:
    """RGB-D frames from the RealSense ZMQ stream with obstacle hulls.

    Frames come from visualizations/realsense_rgbd_zmq_stream.py (JPEG, PNG
    depth, float32 depth scale). A RANSAC floor plane is removed, depth
    discontinuities split touching objects, and each remaining connected
    region gets a convex hull labelled with its median distance.
    """

    def __init__(self, endpoint: str):
        self.endpoint = endpoint
        self.params: dict[str, Any] = {"min_m": 0.25, "max_m": 3.0, "floor": True, "floor_tol": 0.035,
                                       "max_height": 2.0, "edge_m": 0.08, "min_area": 0.003, "depth_max": 4.0,
                                       "floor_tint": True, "max_hz": 10.0}
        self.cond = threading.Condition()
        self.frames: dict[str, tuple[int, bytes | None, str]] = {"rgb": (0, None, "image/jpeg"), "depth": (0, None, "image/jpeg")}
        self.sim: Any = None  # g1_dashboard_sim.SimCamera in --off-robot mode
        self.info: dict[str, Any] = {"ok": False, "message": "Camera idle"}
        self.obstacles: list[dict[str, Any]] = []
        self.depth_m: Any = None
        self.last_viewer = 0.0
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._plane: Any = None
        self._rng = np.random.default_rng(7) if np is not None else None
        self._fps = 0.0
        self.missing = [name for name, module in (("numpy", np), ("opencv-python", cv2)) if module is None]
        try:
            import zmq  # noqa: F401
        except ImportError:
            self.missing.append("pyzmq")

    def touch(self) -> None:
        self.last_viewer = time.monotonic()
        if self.missing and self.sim is None:
            self.info = {"ok": False, "message": "Camera view needs: " + ", ".join(self.missing)}
            return
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run, name="g1-dash-camera", daemon=True)
                self._thread.start()

    def status(self) -> dict[str, Any]:
        self.touch()
        return {**self.info, "endpoint": self.endpoint, "fps": round(self._fps, 1), "obstacles": self.obstacles,
                "params": self.params}

    def set_params(self, body: dict[str, Any]) -> dict[str, Any]:
        numeric = {"min_m": (0.05, 5.0), "max_m": (0.3, 10.0), "floor_tol": (0.005, 0.2), "max_height": (0.2, 4.0),
                   "edge_m": (0.01, 1.0), "min_area": (0.0005, 0.2), "depth_max": (0.5, 10.0), "max_hz": (1.0, 30.0)}
        for key, value in body.items():
            if key in numeric:
                self.params[key] = max(numeric[key][0], min(numeric[key][1], float(value)))
            elif key in ("floor", "floor_tint"):
                self.params[key] = bool(value)
        self._plane = None
        return {"ok": True, "params": self.params}

    def depth_at(self, u: float, v: float) -> dict[str, Any]:
        if self.sim is not None:
            return self.sim.depth_at(u, v)
        depth = self.depth_m
        if depth is None:
            return {"m": None}
        h, w = depth.shape
        x, y = int(min(w - 1, max(0, u * w))), int(min(h - 1, max(0, v * h)))
        patch = depth[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
        valid = patch[patch > 0]
        return {"m": float(np.median(valid)) if valid.size else None, "x": x, "y": y}

    def stream(self, handler: BaseHTTPRequestHandler, kind: str) -> None:
        self.touch()
        handler.close_connection = True
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Connection", "close")
        handler.end_headers()
        last = -1
        try:
            while True:
                self.touch()
                with self.cond:
                    self.cond.wait_for(lambda: self.frames[kind][0] != last, timeout=1.0)
                    seq, image, mime = self.frames[kind]
                if image is None or seq == last:
                    continue
                last = seq
                handler.wfile.write(b"--frame\r\nContent-Type: " + mime.encode() + b"\r\nContent-Length: "
                                    + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def snapshot(self, kind: str) -> tuple[bytes | None, str]:
        self.touch()
        _seq, image, mime = self.frames[kind]
        return image, mime

    def _run_sim(self) -> None:
        while time.monotonic() - self.last_viewer <= 5.0:
            started = time.monotonic()
            try:
                frame = self.sim.render(dict(self.params))
                self.obstacles = frame["obstacles"]
                self.info = {"ok": True, "message": f"{len(frame['obstacles'])} obstacle(s) · simulated camera",
                             "floor": frame["floor"], "size": frame["size"]}
                for kind in ("rgb", "depth"):
                    with self.cond:
                        if self.frames[kind][1] is not frame[kind]:
                            self.frames[kind] = (self.frames[kind][0] + 1, frame[kind], frame["mime"])
                            self.cond.notify_all()
                elapsed = time.monotonic() - started
                self._fps = 1.0 / max(elapsed, 1.0 / float(self.params["max_hz"]))
            except Exception as exc:
                self.info = {"ok": False, "message": f"Simulated camera failed: {type(exc).__name__}: {exc}"}
            time.sleep(max(0.05, 1.0 / float(self.params["max_hz"]) - (time.monotonic() - started)))
        self.info = {"ok": False, "message": "Paused (no viewers)"}

    def _run(self) -> None:
        if self.sim is not None:
            return self._run_sim()
        if self.missing:
            self.info = {"ok": False, "message": "Camera view needs: " + ", ".join(self.missing)}
            return
        import zmq
        context = zmq.Context.instance()
        sock = None
        last_frame = time.monotonic()
        try:
            while True:
                if time.monotonic() - self.last_viewer > 5.0:
                    self.info = {"ok": False, "message": "Paused (no viewers)"}
                    return
                if sock is None:
                    sock = context.socket(zmq.SUB)
                    sock.setsockopt(zmq.LINGER, 0)
                    sock.setsockopt(zmq.RCVHWM, 2)
                    sock.setsockopt(zmq.SUBSCRIBE, b"")
                    sock.connect(self.endpoint)
                if not sock.poll(1000):
                    self.info = {"ok": False, "message": f"Waiting for RGB-D frames on {self.endpoint} "
                                 "(start visualizations/realsense_rgbd_zmq_stream.py)"}
                    continue
                parts = sock.recv_multipart()
                while sock.poll(0):  # keep only the newest frame
                    parts = sock.recv_multipart()
                started = time.monotonic()
                try:
                    self._process(parts)
                    dt = max(1e-3, started - last_frame)
                    self._fps = 0.8 * self._fps + 0.2 * (1.0 / dt) if self._fps else 1.0 / dt
                    last_frame = started
                except Exception as exc:
                    self.info = {"ok": False, "message": f"Frame processing failed: {type(exc).__name__}: {exc}"}
                spare = 1.0 / float(self.params["max_hz"]) - (time.monotonic() - started)
                if spare > 0:
                    time.sleep(spare)
        finally:
            if sock is not None:
                sock.close(0)

    def _publish(self, kind: str, image: Any) -> None:
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            with self.cond:
                self.frames[kind] = (self.frames[kind][0] + 1, jpeg.tobytes(), "image/jpeg")
                self.cond.notify_all()

    def _process(self, parts: list[bytes]) -> None:
        rgb = cv2.imdecode(np.frombuffer(parts[0], dtype=np.uint8), cv2.IMREAD_COLOR)
        if rgb is None:
            raise ValueError("could not decode RGB JPEG")
        depth = None
        if len(parts) > 1 and parts[1] != b"0":
            raw = cv2.imdecode(np.frombuffer(parts[1], dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if raw is not None:
                if raw.ndim == 3:
                    raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
                scale = 0.001
                if len(parts) > 2 and len(parts[2]) >= 4:
                    scale = float(struct.unpack("f", parts[2][:4])[0]) or 0.001
                depth = raw.astype(np.float32) * scale
                if depth.shape[:2] != rgb.shape[:2]:
                    depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        self.depth_m = depth
        if depth is None:
            self.obstacles = []
            self.info = {"ok": True, "message": "RGB only (no depth in stream)"}
            self._publish("rgb", rgb)
            return
        obstacles, floor_small, factor = self._detect(depth)
        self.obstacles = [{k: v for k, v in o.items() if k != "hull"} for o in obstacles]
        self.info = {"ok": True, "message": f"{len(obstacles)} obstacle(s)", "floor": self._plane is not None,
                     "size": [int(rgb.shape[1]), int(rgb.shape[0])]}
        self._publish("rgb", self._draw_rgb(rgb, obstacles, floor_small, factor))
        self._publish("depth", self._draw_depth(depth, obstacles, factor))

    def _intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
        if (width, height) in CAMERA_INTRINSICS:
            return CAMERA_INTRINSICS[(width, height)]
        fx, fy, cx, cy = CAMERA_INTRINSICS[(640, 480)]
        sx, sy = width / 640.0, height / 480.0
        return fx * sx, fy * sy, cx * sx, cy * sy

    def _fit_floor(self, points: Any) -> Any:
        """RANSAC plane whose normal is close to the camera's vertical axis."""
        n = points.shape[0]
        if n < 200:
            return None
        sample = points[self._rng.choice(n, size=min(n, 3000), replace=False)]
        tol = float(self.params["floor_tol"])
        best, best_count = None, 0
        for a, b, c in self._rng.integers(0, sample.shape[0], size=(80, 3)):
            normal = np.cross(sample[b] - sample[a], sample[c] - sample[a])
            length = float(np.linalg.norm(normal))
            if length < 1e-6:
                continue
            normal = normal / length
            if abs(normal[1]) < 0.6:  # camera y axis points down; floor normal is near it
                continue
            offset = -float(normal @ sample[a])
            count = int((np.abs(sample @ normal + offset) < tol).sum())
            if count > best_count:
                best, best_count = (normal, offset), count
        if best is None or best_count < 0.10 * sample.shape[0]:
            return None
        normal, offset = best
        inliers = sample[np.abs(sample @ normal + offset) < tol]
        centroid = inliers.mean(axis=0)
        _u, _s, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
        normal = vt[2]
        offset = -float(normal @ centroid)
        if offset < 0:  # orient so camera-side heights are positive
            normal, offset = -normal, -offset
        if self._plane is not None:
            previous_normal, previous_offset = self._plane
            if float(previous_normal @ normal) > math.cos(math.radians(10)):
                normal = 0.7 * previous_normal + 0.3 * normal
                normal = normal / np.linalg.norm(normal)
                offset = 0.7 * previous_offset + 0.3 * offset
        return normal, offset

    def _detect(self, depth: Any) -> tuple[list[dict[str, Any]], Any, int]:
        params = self.params
        factor = 2 if depth.shape[1] >= 600 else 1
        d = depth[::factor, ::factor]
        h, w = d.shape
        fx, fy, cx, cy = self._intrinsics(depth.shape[1], depth.shape[0])
        fx, fy, cx, cy = fx / factor, fy / factor, cx / factor, cy / factor
        valid = (d > float(params["min_m"])) & (d < float(params["max_m"]))
        floor = np.zeros_like(valid)
        obstacle = valid.copy()
        if params["floor"] and int(valid.sum()) > 500:
            vs, us = np.nonzero(valid)
            z = d[vs, us]
            points = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1).astype(np.float32)
            plane = self._fit_floor(points)
            self._plane = plane
            if plane is not None:
                normal, offset = plane
                height = points @ normal + offset
                on_floor = height < float(params["floor_tol"]) * 1.5
                floor[vs[on_floor], us[on_floor]] = True
                too_high = height > float(params["max_height"])
                obstacle[vs[on_floor | too_high], us[on_floor | too_high]] = False
        filled = np.where(valid, d, 0.0).astype(np.float32)
        grad = np.abs(cv2.Sobel(filled, cv2.CV_32F, 1, 0, ksize=3)) + np.abs(cv2.Sobel(filled, cv2.CV_32F, 0, 1, ksize=3))
        obstacle &= (grad / 8.0) < float(params["edge_m"])
        mask = obstacle.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        min_pixels = float(params["min_area"]) * h * w
        obstacles = []
        for label in range(1, count):
            x, y, bw, bh, area = (int(v) for v in stats[label])
            if area < min_pixels:
                continue
            region = (labels[y:y + bh, x:x + bw] == label)
            region_depth = d[y:y + bh, x:x + bw][region & (d[y:y + bh, x:x + bw] > 0)]
            if region_depth.size == 0:
                continue
            contours = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
                                        offset=(x, y))[-2]
            if not contours:
                continue
            hull = cv2.convexHull(np.vstack(contours))
            distance = float(np.median(region_depth))
            ys, xs = np.nonzero(region)
            u, v = float(xs.mean() + x), float(ys.mean() + y)
            obstacles.append({
                "distance": round(distance, 3), "area": round(area / float(h * w), 4),
                "center": [round((u - cx) * distance / fx, 3), round((v - cy) * distance / fy, 3), round(distance, 3)],
                "size": [round(bw * distance / fx, 3), round(bh * distance / fy, 3)],
                "bbox": [x * factor, y * factor, bw * factor, bh * factor], "hull": hull,
            })
        obstacles.sort(key=lambda o: o["distance"])
        obstacles = obstacles[:16]
        for index, item in enumerate(obstacles, start=1):
            item["id"] = index
        return obstacles, floor, factor

    @staticmethod
    def _color(distance: float) -> tuple[int, int, int]:
        if distance < 0.8:
            return (90, 90, 255)
        if distance < 1.5:
            return (60, 190, 255)
        return (200, 220, 90)

    def _draw_rgb(self, rgb: Any, obstacles: list[dict[str, Any]], floor: Any, factor: int) -> Any:
        out = rgb.copy()
        overlay = rgb.copy()
        if self.params["floor_tint"] and floor is not None and floor.any():
            big = cv2.resize(floor.astype(np.uint8), (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
            overlay[big] = (0.55 * overlay[big] + 0.45 * np.array([120, 200, 80])).astype(np.uint8)
        for item in obstacles:
            cv2.fillPoly(overlay, [item["hull"] * factor], self._color(item["distance"]))
        out = cv2.addWeighted(overlay, 0.32, out, 0.68, 0)
        for item in obstacles:
            color = self._color(item["distance"])
            hull = item["hull"] * factor
            cv2.polylines(out, [hull], True, color, 2, cv2.LINE_AA)
            top = tuple(int(v) for v in hull[hull[:, 0, 1].argmin()][0])
            label = f"{item['id']}  {item['distance']:.2f} m"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x0 = max(0, min(out.shape[1] - tw - 8, top[0] - tw // 2))
            y0 = max(th + 8, top[1] - 6)
            cv2.rectangle(out, (x0, y0 - th - 6), (x0 + tw + 8, y0 + 2), (20, 20, 20), -1)
            cv2.putText(out, label, (x0 + 4, y0 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return out

    def _draw_depth(self, depth: Any, obstacles: list[dict[str, Any]], factor: int) -> Any:
        maximum = float(self.params["depth_max"])
        norm = np.clip(depth / maximum, 0.0, 1.0)
        image = ((1.0 - norm) * 255).astype(np.uint8)  # near = warm
        colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
        colored = cv2.applyColorMap(image, colormap)
        colored[depth <= 0] = (24, 20, 18)
        for item in obstacles:
            cv2.polylines(colored, [item["hull"] * factor], True, (255, 255, 255), 1, cv2.LINE_AA)
        return colored


# -- HTTP ---------------------------------------------------------------------

def _slam(d: Dashboard) -> SlamView:
    d._require_live()
    return d.slam


def _control_bundle(d: Dashboard, heartbeat: bool) -> dict[str, Any]:
    """One poll for /control (browsers cap ~6 connections; two are MJPEG streams)."""
    if heartbeat:
        d.control.heartbeat()
    out: dict[str, Any] = {}
    for key, fn in (("control", d.control.status), ("slam", lambda: _slam(d).status()), ("camera", d.vision.status)):
        try:
            out[key] = fn()
        except Exception as exc:
            out[key] = {"error": str(exc)}
    return out


ROUTES: dict[tuple[str, str], Any] = {
    ("GET", "/api/meta"): lambda d, q, b: d.meta(),
    ("GET", "/api/summary"): lambda d, q, b: d.summary(),
    ("GET", "/api/transcript"): lambda d, q, b: d.transcript(int(q.get("rev", -1))),
    ("POST", "/api/chat"): lambda d, q, b: d.chat(str(b.get("text", ""))),
    ("POST", "/api/confirm"): lambda d, q, b: d.answer_confirmation(b.get("id"), bool(b.get("approved"))),
    ("POST", "/api/clear"): lambda d, q, b: d.clear(),
    ("POST", "/api/exit"): lambda d, q, b: d.shutdown(),
    ("GET", "/api/settings"): lambda d, q, b: {"settings": d.settings},
    ("POST", "/api/settings"): lambda d, q, b: d.update_setting(str(b.get("key")), b.get("value")),
    ("POST", "/api/theme"): lambda d, q, b: d.set_theme(str(b.get("theme"))),
    ("GET", "/api/topics"): lambda d, q, b: d.topics(),
    ("GET", "/api/topic"): lambda d, q, b: d.topic(q.get("name", "")),
    ("GET", "/api/services"): lambda d, q, b: d.services(),
    ("POST", "/api/services"): lambda d, q, b: d.set_service(str(b.get("name")), bool(b.get("enabled"))),
    ("GET", "/api/state"): lambda d, q, b: d.state(),
    ("GET", "/api/state/details"): lambda d, q, b: d.state_details(),
    ("GET", "/api/wifi"): lambda d, q, b: d.wifi_status(),
    ("POST", "/api/wifi"): lambda d, q, b: d.wifi_action(str(b.get("action")), str(b.get("ssid", "")), str(b.get("password", ""))),
    ("GET", "/api/jetson"): lambda d, q, b: d.jetson(),
    ("GET", "/api/mode"): lambda d, q, b: d.mode_state(),
    ("POST", "/api/mode"): lambda d, q, b: d.select_mode(int(b.get("index", -1))),
    ("GET", "/api/control"): lambda d, q, b: (d.control.heartbeat() if q.get("hb") else None) or d.control.status(),
    ("GET", "/api/control/all"): lambda d, q, b: _control_bundle(d, bool(q.get("hb"))),
    ("POST", "/api/control/enter"): lambda d, q, b: d.control.enter(),
    ("POST", "/api/control/exit"): lambda d, q, b: d.control.exit(),
    ("POST", "/api/control/key"): lambda d, q, b: d.control.key(str(b.get("key"))),
    ("GET", "/api/slam"): lambda d, q, b: _slam(d).status(),
    ("POST", "/api/slam/action"): lambda d, q, b: _slam(d).action(b),
    ("POST", "/api/slam/params"): lambda d, q, b: _slam(d).set_params(b),
    ("GET", "/api/camera"): lambda d, q, b: d.vision.status(),
    ("POST", "/api/camera/params"): lambda d, q, b: d.vision.set_params(b),
    ("GET", "/api/camera/depth"): lambda d, q, b: d.vision.depth_at(float(q.get("u", 0)), float(q.get("v", 0))),
    ("GET", "/api/wbc"): lambda d, q, b: d.wbc.status(),
    ("POST", "/api/wbc/start"): lambda d, q, b: d.wbc.start(b),
    ("POST", "/api/wbc/stop"): lambda d, q, b: d.wbc.stop(),
    ("POST", "/api/wbc/action"): lambda d, q, b: d.wbc.action(str(b.get("action")), b),
    ("GET", "/api/sort"): lambda d, q, b: d.sort.get().status(),
    ("GET", "/api/sort/config"): lambda d, q, b: d.sort.config(),
    ("POST", "/api/sort/config"): lambda d, q, b: d.sort.get().set_config(b),
    ("POST", "/api/sort/options"): lambda d, q, b: d.sort.get().set_options(b),
    ("POST", "/api/sort/action"): lambda d, q, b: d.sort.action(str(b.get("action"))),
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "G1Dashboard/1.0"
    dash: Dashboard

    def log_message(self, fmt: str, *args: Any) -> None:
        pass

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: Any, code: int = 200) -> None:
        self._send(code, json.dumps(_jsonable(payload), default=str).encode("utf-8"), "application/json")

    def do_GET(self) -> None:
        self._dispatch("GET")

    def do_POST(self) -> None:
        self._dispatch("POST")

    def _dispatch(self, method: str) -> None:
        url = urlparse(self.path)
        query = {key: values[-1] for key, values in parse_qs(url.query).items()}
        body: dict[str, Any] = {}
        if method == "POST":
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b""
            try:
                parsed = json.loads(raw or b"{}")
                body = parsed if isinstance(parsed, dict) else {}
            except json.JSONDecodeError:
                return self._json({"error": "invalid JSON body"}, 400)
        try:
            if method == "GET" and url.path in ("/", "/index.html"):
                return self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            if method == "GET" and url.path in ("/stream/rgb", "/stream/depth"):
                return self.dash.vision.stream(self, url.path.rsplit("/", 1)[-1])
            if method == "GET" and url.path in ("/frame/rgb", "/frame/depth"):
                image, mime = self.dash.vision.snapshot(url.path.rsplit("/", 1)[-1])
                if image is None:
                    self.send_response(204)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return None
                return self._send(200, image, mime)
            if method == "GET" and url.path == "/stream/sort":
                return self.dash.sort.stream(self, query.get("view", "sort"))
            if method == "GET" and url.path == "/frame/sort":
                image = self.dash.sort.frame(query.get("view", "sort"))
                if image is None:
                    return self._json({"error": "could not encode frame"}, 500)
                return self._send(200, image, "image/jpeg")
            if method == "GET" and url.path == "/api/slam/cloud":
                data = self.dash.slam.cloud() if self.dash.slam is not None else _empty_cloud_packet()
                return self._send(200, data, "application/octet-stream")
            route = ROUTES.get((method, url.path))
            if route is None:
                return self._json({"error": f"No route for {method} {url.path}"}, 404)
            return self._json(route(self.dash, query, body))
        except LiveRequired as exc:
            return self._json({"error": str(exc)}, 409)
        except (BrokenPipeError, ConnectionResetError):
            return None
        except (ValueError, KeyError) as exc:
            return self._json({"error": str(exc)}, 400)
        except Exception as exc:
            return self._json({"error": f"{type(exc).__name__}: {exc}"}, 500)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="G1 browser dashboard (glass UI for g1_cli commands)")
    parser.add_argument("--iface", default=os.environ.get("G1_NETWORK_INTERFACE", "eth0"))
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--model", default=os.environ.get("OPENAI_CHAT_MODEL", OpenAIPlanner.DEFAULT_MODEL))
    parser.add_argument("--offline", action="store_true", help="start without robot or OpenAI; live chat is disabled")
    parser.add_argument("--off-robot", action="store_true",
                        help="UI test mode: simulated robot, SLAM, camera and agent; no robot, DDS, OpenAI or hardware modules are imported")
    parser.add_argument("--host", default=os.environ.get("G1_DASHBOARD_HOST", "0.0.0.0"),
                        help="bind address; the default 0.0.0.0 lets anyone on the network drive the robot, 127.0.0.1 keeps it local")
    parser.add_argument("--port", type=int, default=int(os.environ.get("G1_DASHBOARD_PORT", "8765")))
    parser.add_argument("--rgbd", default=os.environ.get("G1_RGBD_ENDPOINT", "tcp://127.0.0.1:5555"),
                        help="ZMQ endpoint of realsense_rgbd_zmq_stream.py")
    parser.add_argument("--remote-url", default=os.environ.get("G1_REMOTE_URL", "http://192.168.2.79:8765/events"),
                        help="/wbc default: remote controller stream URL (controller_stream.py)")
    parser.add_argument("--wbc-pose-file", default=str(HERE.parent / "WBC" / "saved_ik_pose_cli_v4_poses.json"),
                        help="/wbc saved joint poses and replay sequence (shared with ik_pose_cli_v4.py)")
    parser.add_argument("--vision-model", default=os.environ.get("G1_VISION_MODEL", "yolov8s-world.pt"),
                        help="/sort open-vocabulary YOLO-World checkpoint (--vision-backend yolo)")
    parser.add_argument("--vision-backend", choices=("openai", "yolo"), default=os.environ.get("G1_VISION_BACKEND", "openai"),
                        help="/sort object detector: OpenAI vision API (no model load) or local YOLO-World")
    parser.add_argument("--openai-vision-model", default=os.environ.get("G1_OPENAI_VISION_MODEL", "gpt-4.1"),
                        help="/sort OpenAI vision model (--vision-backend openai); gpt-4.1 gave the tightest boxes at ~1.5 s/frame")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.off_robot:
        return _main_off_robot(args)
    import g1_agent  # the agent's sdk_wrapper.G1 backend (not imported by --off-robot)

    try:
        robot = None if args.offline else sdk_wrapper.G1(iface=args.iface, domain_id=args.domain_id)
        host = None if robot is None else g1_agent.AgentHost(robot, args.rgbd)
        agent = g1_agent.build_agent(args, robot, host)
    except Exception as exc:
        print(f"Could not start g1_dashboard: {exc}", file=sys.stderr)
        return 2
    dash = Dashboard(robot, agent, offline=args.offline, args=args)
    if host is not None:
        host.bind(arm_conflict=lambda: dash.arm_conflict("agent"), grasp_pipeline=dash.sort.pipeline)
    return _serve(dash, args, robot)


def _main_off_robot(args: argparse.Namespace) -> int:
    import tempfile

    # Keep test sessions away from the real robot settings and saved points.
    sim_settings = Path(tempfile.gettempdir()) / "g1_dashboard_off_robot_settings.json"
    if not sim_settings.exists() and g1_cli.SETTINGS_PATH.exists():
        shutil.copyfile(g1_cli.SETTINGS_PATH, sim_settings)
    g1_cli.SETTINGS_PATH = sim_settings
    robot = g1_dashboard_sim.SimRobot()
    agent = g1_dashboard_sim.SimAgent(robot)
    dash = Dashboard(robot, agent, offline=False, args=args)
    dash.vision.sim = g1_dashboard_sim.SimCamera(robot)
    dash.slam.points_path = Path(tempfile.gettempdir()) / "g1_dashboard_off_robot_points.json"
    dash.say("system", "Off-robot simulator: robot, SLAM, camera and agent are simulated. "
                       f"Settings → {sim_settings}; Wi-Fi actions are dry runs.")
    return _serve(dash, args, robot)


def _serve(dash: Dashboard, args: argparse.Namespace, robot: Any) -> int:
    Handler.dash = dash
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as exc:
        print(f"Could not bind {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 2
    httpd.daemon_threads = True
    dash.httpd = httpd
    shown = socket.gethostname() if args.host in ("0.0.0.0", "::") else args.host
    kind = "off-robot simulator" if args.off_robot else "offline" if args.offline else "live robot"
    print(f"G1 dashboard on http://{shown}:{args.port}  ({kind} · Ctrl-C to quit)")
    try:
        httpd.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        dash.running = False
        dash.control.exit()
        dash.sort.close()
        dash.wbc.stop()
        httpd.server_close()
        dash.chat_executor.shutdown(wait=False)
        dash.action_executor.shutdown(wait=False)
        close = getattr(robot, "close", None)
        if callable(close):
            close()
    return 0


PAGE = r"""<!doctype html>
<html lang="en" data-theme="ef">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 Dashboard</title>
<style>
:root{
  --bg:#0d0b07;--bg2:#1c1508;--b1:#ffb300;--b2:#ff6d00;--b3:#7a5200;
  --glass:rgba(255,236,200,.07);--glass2:rgba(255,236,200,.12);--border:rgba(255,226,170,.16);
  --text:#fff5e1;--muted:rgba(255,245,225,.58);--accent:#ffc247;--accent2:#ff8f3d;
  --ok:#4ade80;--warn:#fbbf24;--err:#f87171;--shadow:0 10px 40px rgba(0,0,0,.35);
  --grid:rgba(255,236,200,.08);--input:rgba(0,0,0,.25);
}
[data-theme=dark]{--bg:#0a0c14;--bg2:#151a2c;--b1:#7c4dff;--b2:#00bcd4;--b3:#ff4081;
  --glass:rgba(220,225,255,.06);--glass2:rgba(220,225,255,.11);--border:rgba(200,210,255,.15);
  --text:#eef0ff;--muted:rgba(238,240,255,.56);--accent:#b39dff;--accent2:#4dd0e1;--grid:rgba(220,225,255,.08)}
[data-theme=ocean]{--bg:#02121f;--bg2:#062f55;--b1:#00e5ff;--b2:#2979ff;--b3:#00bfa5;
  --glass:rgba(200,240,255,.07);--glass2:rgba(200,240,255,.12);--border:rgba(170,230,255,.17);
  --text:#e8f7ff;--muted:rgba(232,247,255,.6);--accent:#4dd0e1;--accent2:#82b1ff;--grid:rgba(200,240,255,.08)}
[data-theme=light]{--bg:#dfe6f3;--bg2:#f6f8fc;--b1:#a5b4fc;--b2:#67e8f9;--b3:#f9a8d4;
  --glass:rgba(255,255,255,.55);--glass2:rgba(255,255,255,.75);--border:rgba(255,255,255,.85);
  --text:#151a2d;--muted:#56607a;--accent:#4f46e5;--accent2:#0891b2;--ok:#16a34a;--warn:#d97706;--err:#dc2626;
  --shadow:0 10px 30px rgba(60,70,110,.15);--grid:rgba(30,40,80,.09);--input:rgba(255,255,255,.7)}
/* EF Robotics (ef-robotics.de) brand pink #fa1960 / #ff4982 / #d80749 in amber's dark glass style */
[data-theme=ef]{--bg:#0c0a0b;--bg2:#1f0a12;--b1:#fa1960;--b2:#d80749;--b3:#7a0c2e;
  --glass:rgba(255,225,235,.07);--glass2:rgba(255,225,235,.12);--border:rgba(255,200,220,.16);
  --text:#fff2f6;--muted:rgba(255,242,246,.58);--accent:#ff4982;--accent2:#fa1960;--grid:rgba(255,225,235,.08)}
[data-theme=ef] body{font-family:Roboto,system-ui,-apple-system,"Segoe UI",Ubuntu,sans-serif}
*{box-sizing:border-box}
html,body{margin:0;min-height:100%}
body{background:radial-gradient(1200px 800px at 10% -10%,var(--bg2),transparent 60%),var(--bg);color:var(--text);
  font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,Ubuntu,sans-serif;overflow-x:hidden}
.blobs{position:fixed;inset:0;z-index:-1;overflow:hidden;pointer-events:none}
.blobs i{position:absolute;width:46vmax;height:46vmax;border-radius:50%;filter:blur(90px);opacity:.28}
.blobs i:nth-child(1){background:var(--b1);left:-12vmax;top:-14vmax}
.blobs i:nth-child(2){background:var(--b2);right:-16vmax;top:20vh;opacity:.2}
.blobs i:nth-child(3){background:var(--b3);left:25vw;bottom:-24vmax;opacity:.22}
.glass{background:var(--glass);border:1px solid var(--border);border-radius:18px;box-shadow:var(--shadow),inset 0 1px 0 rgba(255,255,255,.08);
  backdrop-filter:blur(18px) saturate(140%);-webkit-backdrop-filter:blur(18px) saturate(140%)}
.topbar{position:sticky;top:10px;z-index:20;margin:10px 16px 0;padding:10px 16px;display:flex;gap:16px;align-items:center;justify-content:space-between;flex-wrap:wrap}
.brand{display:flex;gap:12px;align-items:center}
.logo{width:44px;height:40px;display:grid;place-items:center;color:var(--text)}
.logo svg{width:100%;height:100%}
.brand b{display:block;font-size:15px}.brand small{color:var(--muted)}
.pills{display:flex;gap:8px;flex-wrap:wrap}
.pill{padding:5px 11px;border-radius:999px;background:var(--glass2);border:1px solid var(--border);font-size:12.5px;white-space:nowrap}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px;background:var(--muted);vertical-align:middle}
.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}.dot.warn{background:var(--warn)}.dot.err{background:var(--err)}
.tabs{margin:10px 16px 0;padding:6px;display:flex;gap:4px;overflow-x:auto;scrollbar-width:thin}
.tabs button{flex:0 0 auto;border:0;background:transparent;color:var(--muted);padding:8px 14px;border-radius:12px;cursor:pointer;font:inherit;font-weight:600}
.tabs button:hover{color:var(--text);background:var(--glass)}
.tabs button.active{color:var(--text);background:var(--glass2);box-shadow:inset 0 0 0 1px var(--border)}
.tabs button span{opacity:.55;font-weight:500}
.tabs .sep{flex:1 0 8px}
.tabs button.danger.active,.tabs button.danger:hover{color:var(--err)}
main{padding:14px 16px 40px}
.tab{display:none;animation:fade .18s ease}.tab.active{display:block}
@keyframes fade{from{opacity:0;transform:translateY(4px)}to{opacity:1}}
.panel{padding:16px;min-width:0}
.panel-head{display:flex;gap:10px;align-items:baseline;justify-content:space-between;flex-wrap:wrap;margin-bottom:12px}
h2{margin:0;font-size:17px}h3{margin:0 0 10px;font-size:14px;letter-spacing:.2px}
.muted{color:var(--muted)}.small{font-size:12px}.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.grid{display:grid;gap:14px}
.cols-2{grid-template-columns:repeat(2,minmax(0,1fr))}.cols-3{grid-template-columns:repeat(3,minmax(0,1fr))}
.cols-4{grid-template-columns:repeat(4,minmax(0,1fr))}
.span-2{grid-column:span 2}.span-3{grid-column:span 3}.span-all{grid-column:1/-1}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.row.end{justify-content:flex-end}.grow{flex:1}
.btn{border:1px solid var(--border);background:var(--glass2);color:var(--text);padding:8px 13px;border-radius:11px;cursor:pointer;font:inherit;font-weight:600;
  transition:transform .08s,background .15s;user-select:none;-webkit-user-select:none;touch-action:manipulation}
.btn:hover{background:color-mix(in srgb,var(--accent) 18%,var(--glass2))}.btn:active,.btn.flash{transform:scale(.96);background:color-mix(in srgb,var(--accent) 35%,var(--glass2))}
.btn.primary{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#14110a;border-color:transparent}
.btn.danger{color:var(--err)}.btn.ok{color:var(--ok)}.btn.ghost{background:transparent}
.btn.sm{padding:5px 9px;font-size:12.5px;border-radius:9px}
.btn:disabled{opacity:.4;cursor:not-allowed;transform:none}
.btn.on{box-shadow:inset 0 0 0 2px var(--accent)}
input,select,textarea{font:inherit;color:var(--text);background:var(--input);border:1px solid var(--border);border-radius:10px;padding:8px 10px;outline:none;min-width:0}
input:focus,select:focus{border-color:var(--accent)}
input[type=range]{padding:0;background:transparent;border:0;accent-color:var(--accent)}
select option{background:var(--bg2);color:var(--text)}
.kv{display:grid;grid-template-columns:minmax(110px,max-content) 1fr;gap:4px 14px;font-size:13px}
.kv div:nth-child(odd){color:var(--muted)}
.kv div{overflow-wrap:anywhere}
.badge{display:inline-block;padding:2px 9px;border-radius:999px;font-size:11.5px;font-weight:700;letter-spacing:.3px;background:var(--glass2);border:1px solid var(--border)}
.badge.ok{color:var(--ok)}.badge.err{color:var(--err)}.badge.warn{color:var(--warn)}
.console{background:rgba(0,0,0,.3);border-radius:12px;padding:12px;max-height:340px;overflow:auto;white-space:pre-wrap;font:12.5px/1.45 ui-monospace,Menlo,Consolas,monospace;margin:0}
[data-theme=light] .console{background:rgba(255,255,255,.6)}
.empty{padding:30px;text-align:center;color:var(--muted)}
canvas{display:block;width:100%}
.hidden{display:none!important}
/* chat */
.chat{display:flex;flex-direction:column;height:calc(100vh - 200px);min-height:420px}
.transcript{flex:1;overflow-y:auto;padding:4px 4px 12px;display:flex;flex-direction:column;gap:8px}
.msg{max-width:min(780px,92%);padding:9px 13px;border-radius:14px;background:var(--glass2);border:1px solid var(--border);white-space:pre-wrap;overflow-wrap:anywhere}
.msg .who{font-size:11px;font-weight:700;letter-spacing:.4px;text-transform:uppercase;color:var(--muted);margin-bottom:2px}
.msg.you{align-self:flex-end;background:color-mix(in srgb,var(--accent) 22%,var(--glass2))}
.msg.agent{border-color:color-mix(in srgb,var(--ok) 40%,var(--border))}
.msg.tool{font-family:ui-monospace,Menlo,monospace;font-size:12.5px;color:var(--warn)}
.msg.error{color:var(--err);border-color:color-mix(in srgb,var(--err) 40%,var(--border))}
.msg.system{align-self:center;background:transparent;border-style:dashed;color:var(--muted);font-size:12.5px;max-width:100%;font-family:ui-monospace,Menlo,monospace}
.chat-input{display:flex;gap:8px;margin-top:10px;position:relative}.chat-input input{flex:1;padding:12px 14px;border-radius:14px}
.suggest{position:absolute;bottom:52px;left:0;min-width:300px;padding:6px;z-index:5}
.suggest div{padding:6px 10px;border-radius:9px;cursor:pointer;display:flex;gap:14px;justify-content:space-between}
.suggest div.sel,.suggest div:hover{background:var(--glass2)}
.typing{align-self:flex-start;color:var(--muted);font-size:12.5px;padding:4px 8px}
.typing i{display:inline-block;width:6px;height:6px;border-radius:50%;background:var(--accent);margin-right:3px;animation:blink 1s infinite}
.typing i:nth-child(2){animation-delay:.2s}.typing i:nth-child(3){animation-delay:.4s}
@keyframes blink{50%{opacity:.2}}
/* settings */
.field{display:grid;grid-template-columns:1fr auto;gap:6px 12px;align-items:center;padding:10px 0;border-bottom:1px solid var(--grid)}
.field:last-child{border-bottom:0}.field label{font-weight:600}.field .ctl{grid-column:1/-1;display:flex;gap:10px;align-items:center}
.field .ctl input[type=range]{flex:1}.field output{font-variant-numeric:tabular-nums;color:var(--accent);font-weight:700}
.switch{position:relative;width:44px;height:24px;flex:0 0 auto}.switch input{opacity:0;width:0;height:0}
.switch span{position:absolute;inset:0;border-radius:99px;background:var(--glass2);border:1px solid var(--border);transition:.2s;cursor:pointer}
.switch span::after{content:"";position:absolute;width:18px;height:18px;left:2px;top:2px;border-radius:50%;background:var(--muted);transition:.2s}
.switch input:checked+span{background:color-mix(in srgb,var(--accent) 45%,transparent)}
.switch input:checked+span::after{transform:translateX(20px);background:var(--accent)}
.seg{display:inline-flex;background:var(--input);border:1px solid var(--border);border-radius:10px;padding:2px}
.seg button{border:0;background:transparent;color:var(--muted);padding:5px 11px;border-radius:8px;cursor:pointer;font:inherit;font-weight:600}
.seg button.on{background:var(--glass2);color:var(--text);box-shadow:inset 0 0 0 1px var(--border)}
/* topics */
.topic-list{max-height:calc(100vh - 260px);overflow:auto;display:flex;flex-direction:column;gap:3px}
.topic-item{padding:8px 10px;border-radius:10px;cursor:pointer;display:flex;gap:8px;align-items:center}
.topic-item:hover{background:var(--glass)}.topic-item.sel{background:var(--glass2);box-shadow:inset 0 0 0 1px var(--border)}
.topic-item .t{flex:1;font-family:ui-monospace,Menlo,monospace;font-size:12.5px;overflow-wrap:anywhere}
.fresh{height:5px;border-radius:5px;background:var(--glass2);overflow:hidden}.fresh i{display:block;height:100%;background:var(--ok);transition:width .3s}
/* services / modes */
.card{padding:14px;border-radius:14px;background:var(--glass);border:1px solid var(--border)}
.svc{display:flex;flex-direction:column;gap:8px}
.svc .name{font-family:ui-monospace,Menlo,monospace;font-weight:700;overflow-wrap:anywhere}
.mode{cursor:pointer;transition:transform .1s,border-color .15s;text-align:left;font:inherit;color:inherit}
.mode:hover{transform:translateY(-2px);border-color:var(--accent)}
.mode:disabled{opacity:.35;cursor:not-allowed;transform:none}
.mode .n{font-size:22px;font-weight:800;color:var(--accent)}
/* gauges */
.gauge{display:flex;gap:14px;align-items:center}
.gauge svg{width:96px;height:96px;flex:0 0 auto}
.gauge .v{font-size:22px;font-weight:800}
.bar{height:8px;border-radius:8px;background:var(--glass2);overflow:hidden}.bar i{display:block;height:100%;border-radius:8px;background:linear-gradient(90deg,var(--accent),var(--accent2))}
.mini-row{display:grid;grid-template-columns:110px 1fr 64px;gap:10px;align-items:center;font-size:12.5px;margin:5px 0}
.stat{font-size:22px;font-weight:800;font-variant-numeric:tabular-nums}.stat small{font-size:12px;color:var(--muted);font-weight:500}
/* theme */
.swatch{cursor:pointer;padding:0;overflow:hidden;font:inherit;color:inherit;text-align:left}
.swatch .pv{height:92px;position:relative}.swatch .pv i{position:absolute;width:70px;height:70px;border-radius:50%;filter:blur(18px)}
.swatch .lb{padding:10px 14px;font-weight:700;display:flex;justify-content:space-between}
.swatch.on{box-shadow:0 0 0 2px var(--accent)}
/* control */
.control-grid{display:grid;gap:14px;grid-template-columns:minmax(0,1.55fr) minmax(320px,1fr)}
.map-wrap{position:relative;border-radius:14px;overflow:hidden;background:rgba(0,0,0,.22);border:1px solid var(--border)}
[data-theme=light] .map-wrap{background:rgba(255,255,255,.45)}
#map{height:520px;touch-action:none;cursor:grab}
#map.goal{cursor:crosshair}
.map-hud{position:absolute;left:10px;top:10px;font-size:12px;pointer-events:none;display:flex;flex-direction:column;gap:4px}
.map-hud span,.cam-hud span{background:rgba(0,0,0,.45);color:#fff;padding:3px 8px;border-radius:8px;width:max-content}
.map-legend{position:absolute;right:10px;bottom:10px;font-size:11.5px;display:flex;align-items:center;gap:6px;background:rgba(0,0,0,.45);color:#fff;padding:5px 9px;border-radius:9px}
.cbar{width:110px;height:8px;border-radius:5px;background:linear-gradient(90deg,#30123b,#4686fb,#1ae4b6,#a2fc3c,#fabb39,#e4460a,#7a0403)}
.cbar.depth{background:linear-gradient(90deg,#7a0403,#e4460a,#fabb39,#a2fc3c,#1ae4b6,#4686fb,#30123b)}
.goalbar{position:absolute;left:50%;bottom:12px;transform:translateX(-50%);padding:8px 10px;display:flex;gap:8px;align-items:center}
.cams{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.cam{position:relative;border-radius:14px;overflow:hidden;background:#000;aspect-ratio:4/3;border:1px solid var(--border)}
.cam img{width:100%;height:100%;object-fit:contain;display:block;image-rendering:auto}
.cam-hud{position:absolute;left:8px;top:8px;display:flex;gap:6px;flex-wrap:wrap;font-size:12px}
.cam .ph{position:absolute;inset:0;display:grid;place-items:center;color:#bbb;text-align:center;padding:20px;font-size:13px}
.obs-list{display:flex;flex-wrap:wrap;gap:6px;margin-top:10px}
.obs{font-size:12px;padding:4px 9px;border-radius:9px;background:var(--glass2);border:1px solid var(--border);font-variant-numeric:tabular-nums}
.obs b{margin-right:4px}
.pad{display:grid;grid-template-columns:repeat(3,56px);grid-template-rows:repeat(3,48px);gap:6px}
.pad .btn{padding:0;font-size:18px;display:grid;place-items:center}
.pad .btn small{font-size:10px;display:block;opacity:.6}
kbd{font:11px ui-monospace,Menlo,monospace;padding:1px 6px;border-radius:6px;border:1px solid var(--border);background:var(--glass2)}
.tactile{display:grid;grid-template-columns:repeat(auto-fill,minmax(64px,1fr));gap:6px}
.taxel{padding:7px 6px;border-radius:9px;text-align:center;font-size:11.5px;font-variant-numeric:tabular-nums;border:1px solid var(--border)}
.taxel b{display:block;font-size:13px}
.taxel.hi{background:color-mix(in srgb,var(--ok) 30%,transparent)}.taxel.lo{background:color-mix(in srgb,var(--err) 22%,transparent)}.taxel.na{opacity:.4}
.list{display:flex;flex-direction:column;gap:5px;max-height:220px;overflow:auto}
.li{display:flex;gap:6px;align-items:center;padding:6px 8px;border-radius:10px;background:var(--glass)}
.li .grow{overflow-wrap:anywhere;font-size:12.5px}
.section-title{font-size:11.5px;text-transform:uppercase;letter-spacing:.8px;color:var(--muted);margin:14px 0 6px;font-weight:700}
.param{display:grid;grid-template-columns:118px 1fr 52px;gap:8px;align-items:center;font-size:12.5px;margin:4px 0}
.param output{text-align:right;font-variant-numeric:tabular-nums;color:var(--accent)}
.kbd-state{font-size:12px}
/* wbc & sort */
table.t tr.sel td{background:color-mix(in srgb,var(--accent) 14%,transparent)}
table.t td.num{font-variant-numeric:tabular-nums;font-family:ui-monospace,Menlo,monospace;white-space:nowrap}
.chip{display:inline-block;min-width:28px;text-align:center;padding:2px 7px;border-radius:8px;border:1px solid var(--border);font-size:12px;font-weight:700;background:var(--glass)}
.chip.on{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#14110a;border-color:transparent}
.cfg{display:grid;grid-template-columns:1fr 1fr;gap:8px 10px}
.cfg label{display:flex;flex-direction:column;gap:3px;font-size:12px;color:var(--muted);min-width:0}
.cfg .full{grid-column:1/-1}.cfg input,.cfg select{width:100%}
.wbc-off{opacity:.45;pointer-events:none}
/* modal & toast */
.modal{position:fixed;inset:0;z-index:50;display:grid;place-items:center;background:rgba(0,0,0,.35);padding:16px}
.modal-card{max-width:440px;width:100%;padding:20px}.modal-card p{color:var(--muted);white-space:pre-wrap}
#toasts{position:fixed;right:16px;bottom:16px;z-index:60;display:flex;flex-direction:column;gap:8px;max-width:min(380px,calc(100vw - 32px))}
.toast{padding:10px 14px;border-radius:12px;font-size:13px;animation:fade .2s}
.toast.err{border-color:color-mix(in srgb,var(--err) 55%,var(--border))}.toast.ok{border-color:color-mix(in srgb,var(--ok) 55%,var(--border))}
table.t{width:100%;border-collapse:collapse;font-size:13px}table.t td,table.t th{padding:6px 8px;border-bottom:1px solid var(--grid);text-align:left;vertical-align:top}
table.t th{color:var(--muted);font-weight:600}
@media (max-width:1100px){.control-grid{grid-template-columns:1fr}.cols-4{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media (max-width:760px){.cols-2,.cols-3,.cols-4{grid-template-columns:1fr}.span-2,.span-3{grid-column:auto}.cams{grid-template-columns:1fr}
  #map{height:380px}.topbar{top:0;margin:8px 8px 0}.tabs{margin:8px 8px 0}main{padding:12px 8px 30px}.chat{height:calc(100vh - 230px)}}
</style>
</head>
<body>
<div class="blobs"><i></i><i></i><i></i></div>
<header class="topbar glass">
  <div class="brand"><div class="logo"><svg aria-label="EF Robotics" role="img" viewBox="0 0 321 290" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M151.345 44.549L49.199 44.251V93.429H135.496V133.33H49.199V185.603H151.34V228.596H1.875V1.255L151.345 1.554V44.549Z" fill="currentColor"/><path d="M319.533 44.549L170.062 44.554V1.255L319.533 1.554V44.549Z" fill="currentColor"/><path d="M171.539 96.554L297.329 96.523V136.733H218.459V228.596H171.573L171.539 96.554Z" fill="currentColor"/><path d="M37.214 257.528C37.214 263.629 34.042 268.082 29.589 270.461L39.471 287.359H25.99L17.388 272.474H13.178V287.359H1.58701V242.52H21.231C29.589 242.52 37.214 248.011 37.214 257.528ZM13.178 252.465V262.53H20.133C23.244 262.53 25.441 260.7 25.441 257.528C25.441 254.356 23.245 252.465 20.133 252.465H13.178Z" fill="currentColor"/><path d="M87.905 265.031C87.905 278.391 77.839 288.152 64.54 288.152C51.241 288.152 41.176 278.391 41.176 265.031C41.176 251.55 51.181 241.727 64.54 241.727C77.899 241.727 87.905 251.549 87.905 265.031ZM52.949 265.031C52.949 272.413 57.708 277.598 64.54 277.598C71.372 277.598 76.131 272.413 76.131 265.031C76.131 257.528 71.373 252.281 64.54 252.281C57.707 252.281 52.949 257.528 52.949 265.031Z" fill="currentColor"/><path d="M126.456 254.905C126.456 258.87 124.686 261.554 122.856 263.079C127.004 264.91 129.994 268.02 129.994 273.938C129.994 282.6 123.039 287.359 114.986 287.359H93.634V242.52H112.058C120.721 242.52 126.456 247.096 126.456 254.905ZM105.226 253.075V260.151H110.473C112.913 260.151 114.682 258.87 114.682 256.613C114.682 254.356 112.913 253.075 110.473 253.075H105.226ZM105.226 268.875V276.806H113.523C116.268 276.806 118.22 275.403 118.22 272.84C118.22 270.277 116.268 268.875 113.523 268.875H105.226Z" fill="currentColor"/><path d="M179.71 265.031C179.71 278.391 169.644 288.152 156.345 288.152C143.046 288.152 132.981 278.391 132.981 265.031C132.981 251.55 142.986 241.727 156.345 241.727C169.704 241.727 179.71 251.549 179.71 265.031ZM144.754 265.031C144.754 272.413 149.513 277.598 156.345 277.598C163.177 277.598 167.936 272.413 167.936 265.031C167.936 257.528 163.178 252.281 156.345 252.281C149.512 252.281 144.754 257.528 144.754 265.031Z" fill="currentColor"/><path d="M220.701 252.709H207.036V287.359H195.445V252.709H181.78V242.521H220.701V252.709Z" fill="currentColor"/><path d="M237.413 287.359H225.822V242.52H237.413V287.359Z" fill="currentColor"/><path d="M282.982 248.621L275.722 257.04C273.708 254.478 270.72 252.281 266.51 252.281C259.372 252.281 254.919 257.528 254.919 264.91C254.919 272.292 259.372 277.598 266.51 277.598C270.72 277.598 273.708 275.402 275.722 272.839L282.92 281.258C278.345 286.321 273.099 288.151 266.51 288.151C252.845 288.151 243.146 278.146 243.146 264.909C243.146 251.672 252.845 241.726 266.51 241.726C273.83 241.726 278.529 243.984 282.982 248.62V248.621Z" fill="currentColor"/><path d="M318.301 246.425L312.322 256.003C309.76 253.929 306.527 252.281 301.768 252.281C298.474 252.281 296.887 253.38 296.887 255.514C296.887 257.039 297.986 258.137 304.025 259.174C316.166 261.676 319.52 266.312 319.52 273.145C319.52 282.173 312.932 288.152 301.829 288.152C294.814 288.152 288.286 286.017 283.711 281.991L289.934 272.352C292.801 275.524 297.803 277.598 302.196 277.598C306.162 277.598 307.93 276.439 307.93 274.243C307.93 272.657 306.649 271.559 300.61 270.461C288.531 267.899 285.237 262.592 285.237 256.125C285.237 247.95 291.642 241.728 301.83 241.728C308.48 241.728 314.519 243.498 318.302 246.425H318.301Z" fill="currentColor"/></svg></div><div><b>G1 Dashboard</b><small id="sub">connecting…</small></div></div>
  <div class="pills">
    <span class="pill" id="pill-link"><span class="dot"></span>link</span>
    <span class="pill" id="pill-batt">battery —</span>
    <span class="pill" id="pill-mode">mode —</span>
    <span class="pill" id="pill-agent">agent idle</span>
  </div>
</header>
<nav class="tabs glass" id="tabs"></nav>
<main>

<section class="tab" id="tab-chat">
  <div class="glass panel chat">
    <div class="panel-head"><h2>Chat</h2><span class="muted small">Plain-language requests go to the intent agent · slash commands open their tab · <kbd>Tab</kbd> completes</span></div>
    <div id="transcript" class="transcript"></div>
    <form id="chat-form" class="chat-input">
      <div id="suggest" class="suggest glass hidden"></div>
      <input id="chat-input" autocomplete="off" placeholder="Ask the robot something, or type / for commands">
      <button class="btn primary" type="submit">Send</button>
    </form>
  </div>
</section>

<section class="tab" id="tab-settings">
  <div class="panel-head"><h2>Settings</h2><span class="muted small">Saved to g1_cli_settings.json (shared with g1_cli) on every change</span></div>
  <div id="settings-groups" class="grid cols-3"></div>
</section>

<section class="tab" id="tab-topics">
  <div class="grid" style="grid-template-columns:minmax(260px,360px) minmax(0,1fr)" id="topics-grid">
    <div class="glass panel">
      <div class="panel-head"><h2>Topics</h2><button class="btn sm" id="topics-refresh">↻</button></div>
      <input id="topic-filter" placeholder="Filter topics" style="width:100%;margin-bottom:10px">
      <div id="topic-note" class="muted small" style="margin-bottom:8px"></div>
      <div id="topic-list" class="topic-list"></div>
    </div>
    <div class="glass panel">
      <div class="panel-head"><h2 id="ti-title" class="mono">Select a topic</h2><span id="ti-badge"></span></div>
      <div class="kv" id="ti-meta"></div>
      <div class="fresh" style="margin:12px 0"><i id="ti-fresh" style="width:0"></i></div>
      <canvas id="ti-canvas" class="hidden" style="height:280px;margin-bottom:12px"></canvas>
      <pre id="ti-lines" class="console">Pick a topic on the left to inspect live DDS data.</pre>
    </div>
  </div>
</section>

<section class="tab" id="tab-services">
  <div class="panel-head"><h2>Services</h2><div class="row"><span id="svc-msg" class="muted small"></span><button class="btn sm" id="svc-refresh">↻ Refresh</button></div></div>
  <div id="svc-grid" class="grid cols-3"></div>
</section>

<section class="tab" id="tab-state">
  <div class="grid cols-4">
    <div class="glass panel"><h3>Battery</h3><div id="st-batt" class="gauge"></div></div>
    <div class="glass panel"><h3>Attitude</h3><canvas id="st-att" style="height:150px"></canvas><div id="st-imu" class="muted small mono"></div></div>
    <div class="glass panel"><h3>Motion</h3><div id="st-motion"></div></div>
    <div class="glass panel"><h3>SLAM pose</h3><div id="st-slam"></div></div>
    <div class="glass panel span-2"><div class="panel-head"><h3>Odometry trail</h3><button class="btn sm" id="st-trail-clear">Clear</button></div><canvas id="st-trail" style="height:260px"></canvas></div>
    <div class="glass panel span-2"><div class="panel-head"><h3>Joint positions</h3><div class="seg" id="st-joint-mode"><button data-v="q" class="on">q rad</button><button data-v="dq">dq</button><button data-v="tau">τ</button></div></div><canvas id="st-joints" style="height:260px"></canvas></div>
    <div class="glass panel span-all"><div class="panel-head"><h3>Robot state (g1_cli /state table)</h3><span class="muted small">refreshes every 3 s</span></div><div id="st-details"></div></div>
  </div>
</section>

<section class="tab" id="tab-wifi">
  <div class="grid cols-3">
    <div class="glass panel"><h3>wlan0</h3><div id="wifi-status" class="kv"></div></div>
    <div class="glass panel"><h3>Actions</h3><div id="wifi-actions" class="row"></div><p class="muted small">Privileged actions run with <span class="mono">sudo -n</span>; run the dashboard with sudo or allow them in sudoers.</p></div>
    <div class="glass panel"><h3>Configure network</h3>
      <form id="wifi-form" class="grid" style="gap:8px">
        <input id="wifi-ssid" placeholder="SSID" autocomplete="off">
        <input id="wifi-pass" type="password" placeholder="Password (8–63 characters)" autocomplete="new-password">
        <button class="btn primary" type="submit">Write config &amp; start supplicant</button>
        <span class="muted small">Runs setup_wpa_supplicant_wlan0.sh. The password goes over stdin, so it never appears in the process list.</span>
      </form>
    </div>
    <div class="glass panel span-all"><div class="panel-head"><h3 id="wifi-out-title">Output</h3></div><pre id="wifi-out" class="console">No command run yet.</pre></div>
  </div>
</section>

<section class="tab" id="tab-jetson">
  <div class="grid cols-4">
    <div class="glass panel"><h3>CPU</h3><div id="j-cpu" class="gauge"></div></div>
    <div class="glass panel"><h3>Memory</h3><div id="j-mem" class="gauge"></div></div>
    <div class="glass panel"><h3>Root disk</h3><div id="j-disk" class="gauge"></div></div>
    <div class="glass panel"><h3>GPU</h3><div id="j-gpu" class="gauge"></div></div>
    <div class="glass panel span-2"><h3>CPU history</h3><canvas id="j-hist" style="height:180px"></canvas><div id="j-load" class="muted small"></div></div>
    <div class="glass panel span-2"><h3>Cores</h3><div id="j-cores"></div></div>
    <div class="glass panel span-2"><h3>Thermal zones</h3><div id="j-therm"></div></div>
    <div class="glass panel span-2"><h3>Board</h3><div id="j-info" class="kv"></div></div>
  </div>
</section>

<section class="tab" id="tab-control">
  <div class="control-grid">
    <div class="glass panel">
      <div class="panel-head">
        <h2>SLAM · obstacle cloud</h2>
        <div class="row">
          <div class="seg" id="map-view"><button data-v="top" class="on">Top</button><button data-v="3d">3D</button></div>
          <button class="btn sm on" id="map-follow">Follow</button>
          <button class="btn sm" id="map-fit">Fit</button>
          <button class="btn sm" id="map-goal">⌖ Set goal</button>
        </div>
      </div>
      <div class="map-wrap">
        <canvas id="map"></canvas>
        <div class="map-hud" id="map-hud"></div>
        <div class="map-legend"><span>floor</span><div class="cbar"></div><span id="map-hmax">1.8 m</span></div>
        <div class="goalbar glass hidden" id="goalbar">
          <span class="mono small" id="goal-text"></span>
          <button class="btn sm primary" id="goal-nav">Navigate</button>
          <button class="btn sm" id="goal-queue">Queue</button>
          <button class="btn sm ghost" id="goal-cancel">✕</button>
        </div>
      </div>
      <div class="row small muted" style="margin-top:8px">
        <label><input type="checkbox" id="ly-map" checked> map</label>
        <label><input type="checkbox" id="ly-live" checked> live scan</label>
        <label><input type="checkbox" id="ly-cells" checked> occupancy</label>
        <label><input type="checkbox" id="ly-points" checked> named points</label>
        <span class="grow"></span>
        <span>drag pan · <kbd>Shift</kbd>/right-drag orbit · wheel zoom</span>
      </div>
    </div>

    <div class="glass panel">
      <div class="panel-head"><h2>SLAM tools</h2><span id="slam-badges" class="row"></span></div>
      <div class="kv" id="slam-kv"></div>
      <div class="section-title">Mapping</div>
      <div class="row">
        <select id="slam-type"><option value="indoor">indoor</option><option value="outdoor">outdoor</option></select>
        <button class="btn sm primary" data-slam="start_mapping">Start mapping</button>
        <button class="btn sm" data-slam="save_map">Save map</button>
        <button class="btn sm danger" data-slam="close_slam">Close SLAM</button>
      </div>
      <div class="row" style="margin-top:8px">
        <input id="slam-path" class="grow mono" placeholder="/home/unitree/test.pcd">
        <button class="btn sm" data-slam="relocate">Relocate</button>
      </div>
      <div class="section-title">Navigation</div>
      <div class="row">
        <button class="btn sm" data-slam="pause">Pause</button>
        <button class="btn sm" data-slam="resume">Resume</button>
        <button class="btn sm danger" data-slam="cancel">Cancel sequence</button>
        <button class="btn sm ghost" data-slam="clear_view">Clear cloud view</button>
      </div>
      <div id="task-state" class="small" style="margin-top:8px"></div>
      <div class="section-title">Named points</div>
      <form class="row" id="point-form"><input id="point-name" class="grow" placeholder="Point name"><button class="btn sm primary">Save current pose</button></form>
      <div id="point-list" class="list" style="margin-top:8px"></div>
      <div class="section-title">Task queue</div>
      <div class="row"><button class="btn sm" data-slam="queue_add_current">+ Current pose</button><button class="btn sm primary" data-slam="queue_run">Run queue</button><button class="btn sm ghost" data-slam="queue_clear">Clear</button></div>
      <div id="task-list" class="list" style="margin-top:8px"></div>
      <div class="section-title">Last action</div>
      <div id="slam-last" class="small mono muted"></div>
    </div>

    <div class="glass panel">
      <div class="panel-head"><h2>Camera</h2><span id="cam-msg" class="muted small"></span></div>
      <div class="cams">
        <div>
          <div class="cam"><img id="cam-rgb" alt=""><div class="ph" id="cam-rgb-ph">RGB stream</div><div class="cam-hud"><span>RGB · obstacle hulls</span><span id="cam-fps"></span></div></div>
        </div>
        <div>
          <div class="cam" id="depth-box"><img id="cam-depth" alt=""><div class="ph" id="cam-depth-ph">Depth stream</div><div class="cam-hud"><span>Depth</span><span id="depth-read">hover for distance</span></div>
            <div class="map-legend"><span>near</span><div class="cbar depth"></div><span id="depth-max">4 m</span></div></div>
        </div>
      </div>
      <div class="obs-list" id="obs-list"></div>
    </div>

    <div class="glass panel">
      <div class="panel-head"><h2>Teleop</h2><span class="kbd-state" id="kbd-state"></span></div>
      <div class="row" style="align-items:flex-start;gap:18px">
        <div>
          <div class="section-title" style="margin-top:0">Locomotion</div>
          <div class="pad">
            <span></span><button class="btn hold" data-key="ArrowUp">↑</button><span></span>
            <button class="btn hold" data-key="ArrowLeft">↺</button><button class="btn danger" data-key="stop">■</button><button class="btn hold" data-key="ArrowRight">↻</button>
            <span></span><button class="btn hold" data-key="ArrowDown">↓</button><span></span>
          </div>
        </div>
        <div>
          <div class="section-title" style="margin-top:0">End effector</div>
          <div class="pad">
            <button class="btn" data-key="q">⤒<small>q z+</small></button><button class="btn" data-key="w">▲<small>w x+</small></button><button class="btn" data-key="e">⤓<small>e z−</small></button>
            <button class="btn" data-key="a">◀<small>a y−</small></button><button class="btn" data-key="m" id="hand-btn">R<small>m hand</small></button><button class="btn" data-key="d">▶<small>d y+</small></button>
            <button class="btn" data-key="f" id="orient-btn">🔒<small>f orient</small></button><button class="btn" data-key="s">▼<small>s x−</small></button><button class="btn" data-key="r" id="release-btn">⇅<small>r arms</small></button>
          </div>
        </div>
        <div class="grow" style="min-width:180px">
          <div class="section-title" style="margin-top:0">Dex3 hands</div>
          <div class="row"><button class="btn sm" data-key="o">Open <kbd>o</kbd></button><button class="btn sm" data-key="c">Close <kbd>c</kbd></button></div>
          <div class="kv" id="ctl-kv" style="margin-top:10px"></div>
        </div>
      </div>
      <div id="ctl-status" class="small" style="margin-top:10px"></div>
    </div>

    <div class="glass panel">
      <div class="panel-head"><h2>Dex3 tactile</h2><span class="muted small" id="tac-thr"></span></div>
      <div class="grid cols-2" id="tactile"></div>
    </div>

    <div class="glass panel">
      <div class="panel-head"><h2>Processing</h2><span class="muted small" id="proc-stats"></span></div>
      <div class="grid cols-2">
        <div><div class="section-title" style="margin-top:0">Obstacle cloud</div><div id="cloud-params"></div></div>
        <div><div class="section-title" style="margin-top:0">Camera hulls</div><div id="cam-params"></div></div>
      </div>
    </div>
  </div>
</section>

<section class="tab" id="tab-wbc">
  <div class="grid cols-3">
    <div class="glass panel span-2">
      <div class="panel-head"><h2>WBC · 6D end-effector IK</h2><span class="row" id="wbc-badges"></span></div>
      <div class="row">
        <span class="small muted">Remote stream</span>
        <input id="wbc-url" class="grow mono" placeholder="http://LAPTOP_IP:8765/events">
        <button class="btn sm" id="wbc-url-apply">Use</button>
      </div>
      <div class="row" style="margin-top:8px">
        <button class="btn primary" id="wbc-start">Start session</button>
        <button class="btn danger" id="wbc-stop">Stop session</button>
        <span class="grow"></span>
        <span class="wbc-live row">
          <button class="btn sm" data-wbc="release">Release arms <kbd>r</kbd></button>
          <button class="btn sm" data-wbc="engage">Engage arms <kbd>e</kbd></button>
          <button class="btn sm" data-wbc="resync">Resync EE <kbd>y</kbd></button>
          <button class="btn sm ghost danger" id="wbc-zero">Zero gains <kbd>z</kbd></button>
        </span>
      </div>
      <div class="row wbc-live" style="margin-top:8px">
        <span class="muted small">Arms <kbd>m</kbd></span><div class="seg" id="wbc-arm"><button data-v="right">right</button><button data-v="left">left</button><button data-v="both">both</button></div>
        <button class="btn sm" data-wbc="orient" id="wbc-orient">Orientation locked <kbd>f</kbd></button>
        <button class="btn sm" data-wbc="waist" id="wbc-waist">Waist held <kbd>w</kbd></button>
      </div>
      <div id="wbc-status" class="small" style="margin-top:10px"></div>
    </div>
    <div class="glass panel">
      <div class="panel-head"><h3>Controller input</h3><span id="wbc-input-badge"></span></div>
      <canvas id="wbc-pad" style="height:150px"></canvas>
      <div id="wbc-input-kv" class="kv small" style="margin-top:8px"></div>
      <p class="muted small" id="wbc-gp-hint"></p>
    </div>
    <div class="glass panel span-2 wbc-live">
      <div class="panel-head"><h3>End effector · <span id="wbc-disp-arm">right</span> arm</h3>
        <div class="row small"><span class="muted">max Δq per IK step</span><button class="btn sm" data-wbc-dq="halve">[</button><span class="mono" id="wbc-maxdq">—</span><button class="btn sm" data-wbc-dq="double">]</button></div></div>
      <table class="t" id="wbc-dofs"></table>
      <div id="wbc-ik" class="small mono" style="margin-top:8px"></div>
    </div>
    <div class="glass panel wbc-live">
      <h3>Dex3 hand</h3>
      <div class="seg" id="wbc-hand"><button data-v="off">off</button><button data-v="right">right</button><button data-v="left">left</button><button data-v="both">both</button><button data-v="follow-arm">follow arm</button></div>
      <div class="field"><label>Grip</label><output id="wbc-grip-out">0%</output><div class="ctl"><input type="range" id="wbc-grip" min="0" max="100" step="1" value="0"></div></div>
      <div class="row"><button class="btn sm" data-grip="0">Open</button><button class="btn sm" data-grip-delta="-5">−5 <kbd>{</kbd></button><button class="btn sm" data-grip-delta="5">+5 <kbd>}</kbd></button><button class="btn sm" data-grip="100">Close</button></div>
      <div id="wbc-hand-info" class="muted small" style="margin-top:8px"></div>
    </div>
    <div class="glass panel span-2"><h3>Arm joints (rad)</h3><div id="wbc-joints" class="muted small">Start a session to see joint targets and feedback.</div></div>
    <div class="glass panel"><div class="panel-head"><h3>Parameters</h3><span class="muted small" id="wbc-params-note"></span></div><div id="wbc-params"></div></div>
    <div class="glass panel wbc-live">
      <div class="panel-head"><h3>Saved poses</h3></div>
      <form class="row" id="wbc-save-form"><input id="wbc-pose-name" class="grow" placeholder="Pose name"><button class="btn sm primary">Save pose <kbd>p</kbd></button></form>
      <div id="wbc-poses" class="list" style="margin-top:8px;max-height:320px"></div>
      <div class="muted small mono" id="wbc-pose-file" style="margin-top:6px;overflow-wrap:anywhere"></div>
    </div>
    <div class="glass panel wbc-live">
      <div class="panel-head"><h3>Replay sequence</h3><div class="row"><button class="btn sm primary" data-wbc="seq_run">Run <kbd>R</kbd></button><button class="btn sm" data-wbc="seq_stop">Stop <kbd>S</kbd></button></div></div>
      <div class="row small"><button class="btn sm" data-wbc="seq_toggle_waist" id="wbc-seq-waist">New steps include waist</button><span class="muted">gap</span><input id="wbc-gap" type="number" min="0" max="60" step="0.5" style="width:80px"><span class="muted">s</span></div>
      <div id="wbc-seq" class="list" style="margin-top:8px;max-height:320px"></div>
    </div>
    <div class="glass panel"><h3>Gamepad &amp; keys</h3><table class="t small" id="wbc-help"></table></div>
  </div>
</section>

<section class="tab" id="tab-sort">
  <div class="grid cols-2">
    <div class="glass panel">
      <div class="panel-head"><h2>Sorting · live view</h2>
        <div class="seg" id="sort-view-sel"><button data-v="sort" class="on">Sorting</button><button data-v="rgb">RGB</button><button data-v="depth">Depth</button><button data-v="segmentation">Masks</button><button data-v="desk-obstacles">Desk</button></div></div>
      <div class="cam"><img id="sort-img" alt=""><div class="ph" id="sort-ph">Starting camera…</div></div>
      <div class="kv small" id="sort-kv" style="margin-top:10px"></div>
    </div>
    <div class="glass panel">
      <div class="panel-head"><h2>Sort</h2><span id="sort-badges" class="row"></span></div>
      <div class="row"><button class="btn primary" id="sort-start">Sort</button><button class="btn danger" data-sort="stop">Stop sorting</button></div>
      <div id="sort-phase" class="small" style="margin-top:10px"></div>
      <div id="sort-scene" class="list" style="margin-top:8px"></div>
      <div class="section-title">Arm &amp; hand</div>
      <div class="row"><button class="btn sm" data-sort="retract">Retract to stable pose</button><button class="btn sm" data-sort="hand_open">Open hand</button><button class="btn sm" data-sort="hand_close">Close hand</button>
        <button class="btn sm" data-sort="release_arms">Release arms</button><button class="btn sm" data-sort="engage_arms">Re-engage arms</button><button class="btn sm danger" data-sort="damp">Damp</button></div>
      <div class="section-title">Dex3 tactile</div>
      <pre id="sort-tactile" class="console" style="max-height:130px"></pre>
    </div>
    <div class="glass panel" id="sort-pat-0"></div>
    <div class="glass panel" id="sort-pat-1"></div>
    <div class="glass panel" id="sort-obj-0"></div>
    <div class="glass panel" id="sort-obj-1"></div>
    <div class="glass panel"><h3>Configuration</h3><div id="sort-config-status" class="small"></div></div>
    <div class="glass panel"><h3>Motion options</h3><div id="sort-options"></div>
      <p class="muted small">Sorting uses the right arm and never steps the base: keep all objects and both patterns within reach and in camera view.</p></div>
    <div class="glass panel span-all"><div class="panel-head"><h3>Log</h3><span class="muted small" id="sort-log-note"></span><button class="btn sm" id="sort-log-copy" onclick="copyText($('#sort-log').textContent,'Log copied')">Copy log</button></div><pre id="sort-log" class="console" style="max-height:420px"></pre></div>
  </div>
</section>

<section class="tab" id="tab-mode">
  <div class="panel-head"><h2>Robot mode</h2><div class="row"><span id="mode-msg" class="muted small"></span><button class="btn sm" id="mode-refresh">↻</button></div></div>
  <div id="mode-warn" class="glass panel hidden" style="margin-bottom:14px;border-color:var(--warn)">Dev mode is active (ai_sport off). Only zero torque, which re-enables ai_sport first, is available.</div>
  <div id="mode-grid" class="grid cols-3"></div>
  <p class="muted small">Modes can move the robot. Each selection asks for confirmation first.</p>
</section>

<section class="tab" id="tab-theme">
  <div class="panel-head"><h2>Theme</h2><span class="muted small">Same theme names as g1_cli; the choice is shared via settings</span></div>
  <div id="theme-grid" class="grid cols-4"></div>
</section>

<section class="tab" id="tab-help">
  <div class="grid cols-2">
    <div class="glass panel"><h3>Commands</h3><table class="t" id="help-cmds"></table></div>
    <div class="glass panel"><h3>/control keys</h3><table class="t" id="help-keys"></table>
      <p class="muted small">Locomotion stops after the configured nudge duration, when you release the key or button, or when the tab loses focus. Leaving /control stops motion and releases the arm pose hold.</p></div>
    <div class="glass panel span-all"><h3>/wbc gamepad &amp; keys</h3><table class="t" id="help-wbc"></table>
      <p class="muted small">The /wbc session keeps running when you leave the tab; input older than 0.5 s stops locomotion. Stop the session to fade arm authority out. Only one of /control, /wbc and /sort can command the arms at a time.</p></div>
  </div>
</section>

<section class="tab" id="tab-clear">
  <div class="glass panel" style="max-width:520px"><h2>Clear transcript</h2><p class="muted">Removes all chat and system messages from the dashboard transcript. Robot state is not affected.</p>
    <p id="clear-count" class="small"></p><button class="btn danger" id="clear-btn">Clear transcript</button></div>
</section>

<section class="tab" id="tab-exit">
  <div class="glass panel" style="max-width:520px"><h2>Exit</h2><p class="muted">Stops the dashboard server. Any active /control session stops locomotion first. Restart it with <span class="mono">python3 g1_dashboard.py</span>.</p>
    <button class="btn danger" id="exit-btn">Shut down dashboard</button></div>
</section>
</main>

<div id="modal" class="modal hidden"><div class="glass modal-card"><h3 id="modal-title"></h3><p id="modal-text"></p>
  <div class="row end"><button class="btn ghost" id="modal-no">Cancel</button><button class="btn primary" id="modal-yes">Confirm</button></div></div></div>
<div id="toasts"></div>

<script>
'use strict';
const $=(s,r=document)=>r.querySelector(s), $$=(s,r=document)=>[...r.querySelectorAll(s)];
function h(tag,attrs,...kids){const e=document.createElement(tag);for(const[k,v]of Object.entries(attrs||{})){if(k==='class')e.className=v;else if(k==='style')e.style.cssText=v;else if(k.startsWith('on'))e.addEventListener(k.slice(2),v);else if(v===true)e.setAttribute(k,'');else if(v!==false&&v!=null)e.setAttribute(k,v);}
  for(const c of kids.flat()){if(c==null||c===false)continue;e.append(c instanceof Node?c:document.createTextNode(String(c)));}return e;}
async function api(path,body){const opt=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};
  const r=await fetch(path,opt);let j={};try{j=await r.json();}catch(e){}if(!r.ok)throw new Error(j.error||`${r.status} ${r.statusText}`);return j;}
/* navigator.clipboard needs https or localhost; the dashboard is usually opened over plain http */
async function copyText(text,okMsg){try{if(navigator.clipboard&&window.isSecureContext){await navigator.clipboard.writeText(text);}
  else{const ta=h('textarea',{style:'position:fixed;opacity:0'});ta.value=text;document.body.append(ta);ta.select();const done=document.execCommand('copy');ta.remove();if(!done)throw new Error('copy blocked by the browser');}
  toast(okMsg||'Copied');}catch(e){toast('Copy failed: '+e.message,'err');}}
function toast(msg,kind){const t=h('div',{class:'toast glass '+(kind||'')},msg);$('#toasts').append(t);setTimeout(()=>t.remove(),kind==='err'?6000:3200);}
const fmt=(v,d=2)=>v==null||!isFinite(v)?'—':Number(v).toFixed(d);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const bytes=b=>b==null?'—':b>=2**30?(b/2**30).toFixed(1)+' GiB':(b/2**20).toFixed(0)+' MiB';
let modalChain=Promise.resolve();
function confirmDialog(title,text,yes='Confirm'){const p=modalChain.then(()=>new Promise(res=>{$('#modal-title').textContent=title;$('#modal-text').textContent=text;$('#modal-yes').textContent=yes;
  $('#modal').classList.remove('hidden');const done=v=>{$('#modal').classList.add('hidden');$('#modal-yes').onclick=$('#modal-no').onclick=null;res(v);};
  $('#modal-yes').onclick=()=>done(true);$('#modal-no').onclick=()=>done(false);}));modalChain=p.catch(()=>{});return p;}
function setupCanvas(c){const r=c.getBoundingClientRect(),d=window.devicePixelRatio||1;const w=Math.max(1,r.width),hh=Math.max(1,r.height);
  if(c.width!==Math.round(w*d)||c.height!==Math.round(hh*d)){c.width=Math.round(w*d);c.height=Math.round(hh*d);}const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,w,hh);return[x,w,hh];}
function ring(el,pct,value,sub){const p=Math.max(0,Math.min(100,pct||0)),R=40,C=2*Math.PI*R;const col=p>85?css('--err'):p>65?css('--warn'):css('--accent');
  el.innerHTML=`<svg viewBox="0 0 100 100"><circle cx="50" cy="50" r="${R}" fill="none" stroke="${css('--grid')}" stroke-width="10"/>
  <circle cx="50" cy="50" r="${R}" fill="none" stroke="${col}" stroke-width="10" stroke-linecap="round" stroke-dasharray="${C*p/100} ${C}" transform="rotate(-90 50 50)"/></svg>
  <div><div class="v">${value}</div><div class="muted small">${sub||''}</div></div>`;}
function turbo(t){t=Math.max(0,Math.min(1,t));const c=v=>Math.max(0,Math.min(255,Math.round(v)));
  const r=34.61+t*(1172.33-t*(10793.56-t*(33300.12-t*(38394.49-t*14825.05))));const g=23.31+t*(557.33+t*(1225.33-t*(3574.96-t*(1073.77+t*707.56))));
  const b=27.2+t*(3211.1-t*(15327.97-t*(27814-t*(22569.18-t*6838.66))));return`rgb(${c(r)},${c(g)},${c(b)})`;}
const TURBO=Array.from({length:32},(_,i)=>turbo(0.08+0.9*i/31));

let META=null, ACTIVE=null, SUMMARY={};
const hooks={};
const loops=[];
function every(tab,ms,fn){loops.push({tab,ms,fn,busy:false,last:-1e9});}
setInterval(()=>{const now=performance.now();for(const l of loops){if(l.tab!=='*'&&l.tab!==ACTIVE)continue;if(document.hidden&&l.tab!=='*')continue;
  if(l.busy||now-l.last<l.ms)continue;l.busy=true;l.last=now;Promise.resolve().then(l.fn).catch(()=>{}).finally(()=>{l.busy=false;});}},40);
const TAB_ORDER=['chat','control','wbc','sort','state','topics','services','mode','settings','jetson','wifi','theme','help','clear','exit'];
function showTab(id){if(!META||!$('#tab-'+id))id='chat';if(ACTIVE===id)return;const prev=ACTIVE;ACTIVE=id;if(prev&&hooks[prev]&&hooks[prev].leave)hooks[prev].leave();
  $$('.tab').forEach(s=>s.classList.toggle('active',s.id==='tab-'+id));$$('#tabs button').forEach(b=>b.classList.toggle('active',b.dataset.tab===id));
  if(location.hash!=='#'+id)history.replaceState(null,'','#'+id);if(hooks[id]&&hooks[id].enter)hooks[id].enter();loops.forEach(l=>{if(l.tab===id)l.last=-1e9;});}
function buildTabs(){const nav=$('#tabs');nav.innerHTML='';const cmds=Object.keys(META.commands).map(c=>c.slice(1));
  for(const id of TAB_ORDER.filter(t=>cmds.includes(t)).concat(cmds.filter(t=>!TAB_ORDER.includes(t)))){if(id==='clear')nav.append(h('span',{class:'sep'}));
    nav.append(h('button',{'data-tab':id,class:id==='exit'?'danger':'',title:META.commands['/'+id],onclick:()=>showTab(id)},h('span',{},'/'),id));}}
window.addEventListener('hashchange',()=>showTab(location.hash.slice(1)));

/* ---------- summary ---------- */
let linkOk=false;
every('*',1500,async()=>{try{SUMMARY=await api('/api/summary');linkOk=true;}catch(e){linkOk=false;}
  $('#pill-link').innerHTML=`<span class="dot ${linkOk?'ok':'err'}"></span>${linkOk?(SUMMARY.off_robot?'simulator':SUMMARY.offline?'offline mode':'live robot'):'disconnected'}`;
  const b=SUMMARY.battery;$('#pill-batt').textContent=b==null?'battery —':`battery ${b}%`;
  $('#pill-mode').textContent=SUMMARY.offline?'robot n/a':SUMMARY.dev_mode?'dev mode':'ai_sport';
  $('#pill-agent').innerHTML=SUMMARY.busy?'<span class="dot warn"></span>agent thinking':'agent idle';
  if(SUMMARY.pending)showPending(SUMMARY.pending);});
let pendingShown=null;
async function showPending(p){if(!p||pendingShown===p.id)return;pendingShown=p.id;
  const ok=await confirmDialog(`Agent wants to run ${p.skill}`,p.reason||'This skill requires confirmation.','Approve');
  try{await api('/api/confirm',{id:p.id,approved:ok});}catch(e){toast(e.message,'err');}}

/* ---------- chat ---------- */
let chatRev=-1,hist=[],histIdx=null;
const transcriptEl=$('#transcript');
every('chat',400,async()=>{const j=await api('/api/transcript?rev='+chatRev);if(j.changed===false){setTyping(j.busy);return;}chatRev=j.rev;renderTranscript(j.messages||[]);setTyping(j.busy);if(j.pending)showPending(j.pending);});
function renderTranscript(msgs){const atBottom=transcriptEl.scrollHeight-transcriptEl.scrollTop-transcriptEl.clientHeight<60;
  const have=$$('.msg',transcriptEl);if(!msgs.length||(have.length&&+have[0].dataset.id!==msgs[0].id))have.forEach(e=>e.remove());
  const existing=new Set($$('.msg',transcriptEl).map(e=>+e.dataset.id));const ty=$('.typing',transcriptEl);
  for(const m of msgs){if(existing.has(m.id))continue;const who={you:'you',agent:'g1',tool:'tool',error:'error'}[m.role]||'';
    transcriptEl.insertBefore(h('div',{class:'msg '+m.role,'data-id':m.id},who?h('div',{class:'who'},who):null,m.text),ty||null);}
  if(atBottom)transcriptEl.scrollTop=transcriptEl.scrollHeight;}
function setTyping(b){let t=$('.typing',transcriptEl);if(b&&!t){transcriptEl.append(h('div',{class:'typing'},h('i'),h('i'),h('i'),' thinking…'));transcriptEl.scrollTop=transcriptEl.scrollHeight;}else if(!b&&t)t.remove();}
const chatInput=$('#chat-input'),sugg=$('#suggest');let suggIdx=0;
function suggestions(){const v=chatInput.value;if(!v.startsWith('/')||v.includes(' '))return[];return Object.entries(META.commands).filter(([c])=>c.startsWith(v.toLowerCase()));}
function renderSuggest(){const s=suggestions();sugg.classList.toggle('hidden',!s.length);sugg.innerHTML='';suggIdx=Math.min(suggIdx,Math.max(0,s.length-1));
  s.forEach(([c,d],i)=>sugg.append(h('div',{class:i===suggIdx?'sel':'',onmousedown:e=>{e.preventDefault();chatInput.value=c;sendChat();}},h('b',{class:'mono'},c),h('span',{class:'muted small'},d))));}
chatInput.addEventListener('input',()=>{suggIdx=0;renderSuggest();histIdx=null;});
chatInput.addEventListener('keydown',e=>{const s=suggestions();
  if(e.key==='Tab'&&s.length){e.preventDefault();chatInput.value=s[suggIdx%s.length][0];suggIdx=(suggIdx+1)%s.length;renderSuggest();return;}
  if(e.key==='ArrowUp'&&!s.length&&hist.length){e.preventDefault();histIdx=histIdx==null?hist.length-1:Math.max(0,histIdx-1);chatInput.value=hist[histIdx];}
  else if(e.key==='ArrowDown'&&!s.length&&histIdx!=null){e.preventDefault();histIdx++;if(histIdx>=hist.length){histIdx=null;chatInput.value='';}else chatInput.value=hist[histIdx];}
  else if(e.key==='Escape'){sugg.classList.add('hidden');}});
$('#chat-form').addEventListener('submit',e=>{e.preventDefault();sendChat();});
async function sendChat(){const text=chatInput.value.trim();if(!text)return;chatInput.value='';sugg.classList.add('hidden');if(hist[hist.length-1]!==text)hist.push(text);histIdx=null;
  try{const j=await api('/api/chat',{text});if(j.tab){showTab(j.tab);return;}chatRev=-2;}catch(e){toast(e.message,'err');}}
hooks.chat={enter:()=>setTimeout(()=>chatInput.focus(),50)};

/* ---------- settings ---------- */
const debouncers={};
function saveSetting(key,value,delay=250){clearTimeout(debouncers[key]);debouncers[key]=setTimeout(async()=>{try{const j=await api('/api/settings',{key,value});META.settings=j.settings;toast(`Saved ${key}`,'ok');}catch(e){toast(e.message,'err');}},delay);}
function renderSettings(){const root=$('#settings-groups');root.innerHTML='';const groups={};
  for(const f of META.settings_fields)(groups[f.group]=groups[f.group]||[]).push(f);
  for(const[g,fields]of Object.entries(groups)){const card=h('div',{class:'glass panel'},h('h3',{},g));
    for(const f of fields){const v=META.settings[f.key];let ctl;
      if(f.kind==='bool'){ctl=h('label',{class:'switch'},h('input',{type:'checkbox',checked:!!v,onchange:e=>saveSetting(f.key,e.target.checked,0)}),h('span'));card.append(h('div',{class:'field'},h('label',{},f.label),ctl));continue;}
      if(f.kind==='choice'){const seg=h('div',{class:'seg'});for(const o of f.options){seg.append(h('button',{class:o===v?'on':'',onclick:()=>{$$('button',seg).forEach(b=>b.classList.toggle('on',b.textContent===o));saveSetting(f.key,o,0);}},o));}
        card.append(h('div',{class:'field'},h('label',{},f.label),seg));continue;}
      const out=h('output',{},String(v));
      const num=h('input',{type:'number',value:v,step:f.step,min:f.min,max:f.max,style:'width:96px'});
      const rng=h('input',{type:'range',min:f.min,max:f.max,step:f.step,value:v});
      rng.oninput=()=>{num.value=rng.value;out.textContent=rng.value;saveSetting(f.key,+rng.value);};num.onchange=()=>{rng.value=num.value;out.textContent=num.value;saveSetting(f.key,+num.value,0);};
      card.append(h('div',{class:'field'},h('label',{},f.label),out,h('div',{class:'ctl'},rng,num)));}
    root.append(card);}}
hooks.settings={enter:async()=>{try{META.settings=(await api('/api/settings')).settings;}catch(e){}renderSettings();}};

/* ---------- topics ---------- */
let topicRows=[],topicSel=null;
async function loadTopics(){try{const j=await api('/api/topics');topicRows=j.rows;$('#topic-note').textContent=j.discovery_error?('Live discovery: '+j.discovery_error):'● live publisher discovered via ros2 · ○ configured only';}catch(e){$('#topic-note').textContent=e.message;}renderTopicList();}
function renderTopicList(){const q=$('#topic-filter').value.toLowerCase(),list=$('#topic-list');list.innerHTML='';
  for(const r of topicRows.filter(r=>r.topic.includes(q)))list.append(h('div',{class:'topic-item'+(r.topic===topicSel?' sel':''),title:r.type,onclick:()=>{topicSel=r.topic;renderTopicList();pollTopic();}},
    h('span',{class:'dot '+(r.live?'ok':'')}),h('span',{class:'t'},r.topic)));}
$('#topic-filter').oninput=renderTopicList;$('#topics-refresh').onclick=loadTopics;
async function pollTopic(){if(!topicSel)return;let j;try{j=await api('/api/topic?name='+encodeURIComponent(topicSel));}catch(e){$('#ti-lines').textContent=e.message;return;}
  if(j.topic!==topicSel)return;$('#ti-title').textContent=j.topic;
  const age=j.age;$('#ti-badge').innerHTML=!j.reader?'<span class="badge">no reader</span>':age==null?'<span class="badge warn">waiting</span>':age<2?'<span class="badge ok">live</span>':'<span class="badge err">stale</span>';
  const kv=$('#ti-meta');kv.innerHTML='';[['Type',j.type||'unknown'],['Last message',age==null?'—':age.toFixed(2)+' s ago'],['ros2',(topicRows.find(r=>r.topic===j.topic)||{}).type||'—']].forEach(([k,v])=>kv.append(h('div',{},k),h('div',{class:'mono'},v)));
  $('#ti-fresh').style.width=age==null?'0':Math.max(0,100-age*25)+'%';$('#ti-lines').textContent=(j.lines||[]).join('\n');
  const c=$('#ti-canvas');if(j.points&&j.points.length){c.classList.remove('hidden');drawScatter(c,j.points);}else if(j.joints&&j.joints.length){c.classList.remove('hidden');drawBars(c,j.joints,Math.PI);}else c.classList.add('hidden');}
every('topics',350,pollTopic);every('topics',6000,loadTopics);
function drawScatter(c,pts){const[x,w,hh]=setupCanvas(c);let mx=1;for(const p of pts)mx=Math.max(mx,Math.abs(p[0]),Math.abs(p[1]));const s=Math.min(w,hh)/2/mx*0.92;
  let zl=Infinity,zh=-Infinity;for(const p of pts){zl=Math.min(zl,p[2]);zh=Math.max(zh,p[2]);}
  x.strokeStyle=css('--grid');x.beginPath();x.moveTo(w/2,0);x.lineTo(w/2,hh);x.moveTo(0,hh/2);x.lineTo(w,hh/2);x.stroke();
  for(const p of pts){x.fillStyle=TURBO[Math.round(31*(p[2]-zl)/Math.max(1e-3,zh-zl))];x.fillRect(w/2+p[0]*s-1,hh/2-p[1]*s-1,2,2);}
  x.fillStyle=css('--muted');x.font='11px system-ui';x.fillText(`top view · ±${mx.toFixed(1)} m · color = z`,8,14);}
const JOINT_LABELS=['L hip P','L hip R','L hip Y','L knee','L ank P','L ank R','R hip P','R hip R','R hip Y','R knee','R ank P','R ank R','waist Y','waist R','waist P',
  'L sh P','L sh R','L sh Y','L elbow','L wr R','L wr P','L wr Y','R sh P','R sh R','R sh Y','R elbow','R wr R','R wr P','R wr Y'];
function drawBars(c,vals,range,unit){const[x,w,hh]=setupCanvas(c);const n=vals.length,bw=(w-30)/n,mid=(hh-34)/2+6;let mx=range||Math.max(1e-3,...vals.map(Math.abs));
  x.strokeStyle=css('--grid');x.beginPath();x.moveTo(24,mid);x.lineTo(w,mid);x.stroke();x.fillStyle=css('--muted');x.font='10px system-ui';x.fillText('+'+mx.toFixed(1),0,12);x.fillText('−'+mx.toFixed(1),0,hh-30);
  const groups=[[0,6,'--accent'],[6,12,'--accent2'],[12,15,'--warn'],[15,22,'--ok'],[22,29,'--err']];
  vals.forEach((v,i)=>{const g=groups.find(g=>i>=g[0]&&i<g[1]);x.fillStyle=css(g?g[2]:'--accent');const bh=Math.max(-1,Math.min(1,v/mx))*(mid-6);x.fillRect(26+i*bw+1,mid-Math.max(0,bh),Math.max(1,bw-2),Math.abs(bh)||1);
    if(bw>14){x.save();x.translate(26+i*bw+bw/2+3,hh-2);x.rotate(-Math.PI/2.4);x.fillStyle=css('--muted');x.fillText(JOINT_LABELS[i]||('J'+i),0,0);x.restore();}});}
hooks.topics={enter:loadTopics};

/* ---------- services ---------- */
async function loadServices(){$('#svc-msg').textContent='Loading service list…';try{renderServices(await api('/api/services'));}catch(e){$('#svc-msg').textContent=e.message;$('#svc-grid').innerHTML='';}}
function renderServices(j){$('#svc-msg').textContent=j.message||'';const g=$('#svc-grid');g.innerHTML='';
  if(!j.rows||!j.rows.length){g.append(h('div',{class:'glass panel empty span-all'},'No service list loaded.'));return;}
  for(const r of j.rows){const st={0:['ON','ok'],1:['OFF','err'],5:['PROTECTED','warn']}[r.status]||['UNKNOWN',''];
    g.append(h('div',{class:'glass panel svc'},h('div',{class:'row'},h('span',{class:'name grow'},r.name),h('span',{class:'badge '+st[1]},st[0])),
      h('div',{class:'muted small'},r.description||'—',r.protected===true?' · protected':''),
      h('div',{class:'row'},h('button',{class:'btn sm ok',disabled:r.status===0,onclick:()=>switchSvc(r.name,true)},'Enable'),h('button',{class:'btn sm danger',disabled:r.status===1,onclick:()=>switchSvc(r.name,false)},'Disable'))));}}
async function switchSvc(name,enabled){if(!await confirmDialog(`${enabled?'Enable':'Disable'} ${name}?`,name==='ai_sport'&&!enabled?'Disabling ai_sport enters dev mode: the robot loses its balance controller.':'The switch is verified against the live service list afterwards.'))return;
  $('#svc-msg').textContent=`Switching ${name}…`;try{const j=await api('/api/services',{name,enabled});renderServices(j);toast(j.message,j.ok?'ok':'err');}catch(e){toast(e.message,'err');}}
$('#svc-refresh').onclick=loadServices;hooks.services={enter:loadServices};

/* ---------- state ---------- */
const trail=[];let jointMode='q',lastState=null;
$$('#st-joint-mode button').forEach(b=>b.onclick=()=>{jointMode=b.dataset.v;$$('#st-joint-mode button').forEach(x=>x.classList.toggle('on',x===b));if(lastState)drawJoints(lastState);});
$('#st-trail-clear').onclick=()=>{trail.length=0;};
every('state',400,async()=>{let s;try{s=await api('/api/state');}catch(e){$('#st-motion').textContent=e.message;return;}lastState=s;
  const bms=(s.battery||{}).bms||{};const soc=bms.soc;ring($('#st-batt'),soc,soc==null?'—':soc+'%',s.battery?[(s.battery.source||'lowstate'),bms.current!=null?`current ${bms.current}`:'',s.battery.power_v!=null?`${fmt(s.battery.power_v,1)} V`:''].filter(Boolean).join(' · '):'telemetry not received');
  const imu=s.imu||{};drawAttitude(imu.rpy);$('#st-imu').textContent=imu.rpy?`rpy ${imu.rpy.map(v=>fmt(v*180/Math.PI,1)).join(' / ')}°  ·  gyro ${(imu.gyro||[]).map(v=>fmt(v,2)).join(' ')}  ·  ${imu.temp!=null?fmt(imu.temp,0)+'°C':''}`:'IMU telemetry not received';
  const o=s.odom||{};const pos=o.position||(o.pose?[o.pose[0],o.pose[1],0]:null);const vel=o.velocity;
  $('#st-motion').innerHTML='';$('#st-motion').append(h('div',{class:'kv'},h('div',{},'source'),h('div',{class:'mono'},o.topic||'—'),h('div',{},'position'),h('div',{class:'mono'},pos?pos.map(v=>fmt(v)).join(', '):'—'),
    h('div',{},'velocity'),h('div',{class:'mono'},vel?vel.map(v=>fmt(v)).join(', '):'—'),h('div',{},'gait'),h('div',{},o.gait==null?'—':String(o.gait)),h('div',{},'mode'),h('div',{},o.mode==null?'—':String(o.mode))));
  const sp=s.slam_pose;$('#st-slam').innerHTML=sp?`<div class="stat">${fmt(sp[0])}, ${fmt(sp[1])} <small>m</small></div><div class="muted">yaw ${fmt(sp[2]*180/Math.PI,1)}°</div>`:'<div class="muted">not available</div>';
  if(pos&&isFinite(pos[0])){const l=trail[trail.length-1];if(!l||Math.hypot(l[0]-pos[0],l[1]-pos[1])>0.01)trail.push([pos[0],pos[1]]);if(trail.length>600)trail.shift();}drawTrail();drawJoints(s);});
every('state',3000,async()=>{try{const j=await api('/api/state/details');const root=$('#st-details');root.innerHTML='';
  for(const t of j.tables){if(!t.rows){root.append(h('div',{class:'muted'},t.text));continue;}const tb=h('table',{class:'t'});for(const[k,v]of t.rows)tb.append(h('tr',{},h('td',{class:'muted',style:'width:200px'},k),h('td',{class:'mono'},typeof v==='string'?v:JSON.stringify(v))));root.append(tb);}}catch(e){$('#st-details').textContent=e.message;}});
function drawAttitude(rpy){const[x,w,hh]=setupCanvas($('#st-att'));const r=Math.min(w,hh)/2-6,cx=w/2,cy=hh/2;x.save();x.beginPath();x.arc(cx,cy,r,0,7);x.clip();
  const roll=rpy?rpy[0]:0,pitch=rpy?rpy[1]:0;x.translate(cx,cy);x.rotate(-roll);const off=pitch*r*1.6;x.fillStyle=css('--accent2');x.globalAlpha=.35;x.fillRect(-2*r,-2*r+off,4*r,2*r);x.fillStyle=css('--accent');x.globalAlpha=.3;x.fillRect(-2*r,off,4*r,2*r);x.globalAlpha=1;
  x.strokeStyle=css('--text');x.lineWidth=1;for(let d=-30;d<=30;d+=10){if(!d)continue;const y=off-d*Math.PI/180*r*1.6;x.beginPath();x.moveTo(-r*.25,y);x.lineTo(r*.25,y);x.globalAlpha=.4;x.stroke();}x.globalAlpha=1;x.restore();
  x.strokeStyle=css('--border');x.lineWidth=2;x.beginPath();x.arc(cx,cy,r,0,7);x.stroke();x.strokeStyle=css('--warn');x.lineWidth=3;x.beginPath();x.moveTo(cx-r*.5,cy);x.lineTo(cx-r*.15,cy);x.lineTo(cx,cy+8);x.lineTo(cx+r*.15,cy);x.lineTo(cx+r*.5,cy);x.stroke();
  if(rpy){const yaw=rpy[2];x.fillStyle=css('--muted');x.font='11px system-ui';x.fillText(`yaw ${fmt(yaw*180/Math.PI,0)}°`,6,14);}}
function drawTrail(){const[x,w,hh]=setupCanvas($('#st-trail'));if(!trail.length){x.fillStyle=css('--muted');x.fillText('waiting for odometry…',12,20);return;}
  let a=Infinity,b=-Infinity,c=Infinity,d=-Infinity;for(const p of trail){a=Math.min(a,p[0]);b=Math.max(b,p[0]);c=Math.min(c,p[1]);d=Math.max(d,p[1]);}
  const span=Math.max(1,b-a,d-c)*1.2,s=Math.min(w,hh)/span,mx=(a+b)/2,my=(c+d)/2;const P=p=>[w/2+(p[0]-mx)*s,hh/2-(p[1]-my)*s];
  x.strokeStyle=css('--grid');x.lineWidth=1;const step=span>8?2:span>3?1:0.5;for(let g=Math.floor((mx-span)/step)*step;g<mx+span;g+=step){const[sx]=P([g,0]);x.beginPath();x.moveTo(sx,0);x.lineTo(sx,hh);x.stroke();}
  for(let g=Math.floor((my-span)/step)*step;g<my+span;g+=step){const[,sy]=P([0,g]);x.beginPath();x.moveTo(0,sy);x.lineTo(w,sy);x.stroke();}
  x.strokeStyle=css('--accent');x.lineWidth=2;x.beginPath();trail.forEach((p,i)=>{const[sx,sy]=P(p);i?x.lineTo(sx,sy):x.moveTo(sx,sy);});x.stroke();
  const[ex,ey]=P(trail[trail.length-1]);x.fillStyle=css('--accent2');x.beginPath();x.arc(ex,ey,5,0,7);x.fill();x.fillStyle=css('--muted');x.font='11px system-ui';x.fillText(`grid ${step} m`,8,14);}
function drawJoints(s){const v=(s.joints||{})[jointMode]||[];const c=$('#st-joints');if(!v.length){const[x]=setupCanvas(c);x.fillStyle=css('--muted');x.fillText('lowstate not received',12,20);return;}drawBars(c,v,jointMode==='q'?Math.PI:null);}

/* ---------- wifi ---------- */
function renderWifiActions(){const r=$('#wifi-actions');r.innerHTML='';for(const a of META.wifi_actions)r.append(h('button',{class:'btn sm'+(a.sudo?'':' ghost'),onclick:()=>wifi(a.id,a.label,a.sudo)},a.label));}
async function wifi(action,label,sudo,extra){if(sudo&&!await confirmDialog(label+'?','This changes the Jetson network configuration and may drop a Wi-Fi connection to the dashboard.'))return;
  $('#wifi-out-title').textContent=label+' · running…';try{const j=await api('/api/wifi',{action,...(extra||{})});$('#wifi-out-title').textContent=`${j.label} · exit ${j.code}`;$('#wifi-out').textContent=j.output;toast(`${j.label}: ${j.ok?'ok':'failed'}`,j.ok?'ok':'err');}
  catch(e){$('#wifi-out-title').textContent=label;$('#wifi-out').textContent=e.message;}loadWifi();}
async function loadWifi(){try{const j=await api('/api/wifi');const kv=$('#wifi-status');kv.innerHTML='';[['supplicant',j.supplicant],['address',j.address||'—'],['link',j.link||'—']].forEach(([k,v])=>kv.append(h('div',{},k),h('div',{class:'mono'},v)));}catch(e){}}
$('#wifi-form').addEventListener('submit',e=>{e.preventDefault();const ssid=$('#wifi-ssid').value.trim(),password=$('#wifi-pass').value;$('#wifi-pass').value='';wifi('configure',`Configure ${ssid}`,true,{ssid,password});});
every('wifi',4000,loadWifi);hooks.wifi={enter:renderWifiActions};

/* ---------- jetson ---------- */
const cpuHist=[];
every('jetson',1500,async()=>{const j=await api('/api/jetson');const cpu=j.cpu||{};
  ring($('#j-cpu'),cpu.total,cpu.total==null?'…':fmt(cpu.total,0)+'%',`${(cpu.cores||[]).length} cores`);
  if(j.memory)ring($('#j-mem'),100*j.memory.used/j.memory.total,fmt(100*j.memory.used/j.memory.total,0)+'%',`${bytes(j.memory.used)} / ${bytes(j.memory.total)}`+(j.swap?` · swap ${bytes(j.swap.used)}`:''));
  ring($('#j-disk'),100*j.disk.used/j.disk.total,fmt(100*j.disk.used/j.disk.total,0)+'%',`${bytes(j.disk.free)} free of ${bytes(j.disk.total)}`);
  ring($('#j-gpu'),j.gpu,j.gpu==null?'n/a':fmt(j.gpu,0)+'%',j.gpu==null?'no Jetson GPU load file':'GR3D load');
  if(cpu.total!=null){cpuHist.push(cpu.total);if(cpuHist.length>120)cpuHist.shift();}
  const[x,w,hh]=setupCanvas($('#j-hist'));x.strokeStyle=css('--grid');for(const p of[25,50,75]){x.beginPath();x.moveTo(0,hh-hh*p/100);x.lineTo(w,hh-hh*p/100);x.stroke();}
  if(cpuHist.length>1){const g=x.createLinearGradient(0,0,0,hh);g.addColorStop(0,css('--accent'));g.addColorStop(1,'transparent');x.beginPath();cpuHist.forEach((v,i)=>{const px=w*i/119,py=hh-hh*v/100;i?x.lineTo(px,py):x.moveTo(px,py);});
    x.strokeStyle=css('--accent');x.lineWidth=2;x.stroke();x.lineTo(w*(cpuHist.length-1)/119,hh);x.lineTo(0,hh);x.fillStyle=g;x.globalAlpha=.25;x.fill();x.globalAlpha=1;}
  $('#j-load').textContent=(j.load?'load '+j.load.map(v=>v.toFixed(2)).join(' · '):'')+(j.uptime_h!=null?`  ·  uptime ${j.uptime_h.toFixed(1)} h`:'');
  const cores=$('#j-cores');cores.innerHTML='';(cpu.cores||[]).forEach((v,i)=>cores.append(h('div',{class:'mini-row'},h('span',{class:'muted'},'cpu'+i),h('div',{class:'bar'},h('i',{style:`width:${v}%`})),h('span',{class:'mono'},fmt(v,0)+'%'))));
  if(!(cpu.cores||[]).length)cores.textContent='collecting…';
  const th=$('#j-therm');th.innerHTML='';(j.thermal||[]).forEach(t=>th.append(h('div',{class:'mini-row'},h('span',{class:'muted'},t.name),h('div',{class:'bar'},h('i',{style:`width:${Math.min(100,t.c)}%;${t.c>80?'background:var(--err)':''}`})),h('span',{class:'mono'},fmt(t.c,1)+'°C'))));
  if(!(j.thermal||[]).length)th.textContent='No thermal zones reported.';
  const info=$('#j-info');info.innerHTML='';(j.info||[]).forEach(([k,v])=>info.append(h('div',{},k),h('div',{class:'mono'},String(v))));});

/* ---------- mode ---------- */
let modeState=null;
async function loadMode(){$('#mode-msg').textContent='Checking current robot mode…';try{modeState=await api('/api/mode');$('#mode-msg').textContent=modeState.message+(modeState.fsm_id!=null?` · FSM id ${modeState.fsm_id} · FSM mode ${modeState.fsm_mode}`:'');}catch(e){$('#mode-msg').textContent=e.message;modeState=null;}renderModes();}
function renderModes(){const g=$('#mode-grid');g.innerHTML='';const dev=modeState&&modeState.dev_mode;$('#mode-warn').classList.toggle('hidden',!dev);
  META.modes.forEach((name,i)=>{const enabled=modeState?modeState.enabled[i]:false;g.append(h('button',{class:'glass panel mode',disabled:!enabled,onclick:()=>selectMode(i)},h('div',{class:'n'},String(i)),h('h3',{style:'margin:6px 0 4px'},name),h('div',{class:'muted small'},META.mode_hints[i])));});}
async function selectMode(i){if(!await confirmDialog(`Switch to ${META.modes[i]}?`,'This sends the mode change to the robot. Make sure the robot is supported (hoist or clear floor) before switching.','Send'))return;
  $('#mode-msg').textContent=`Sending ${META.modes[i]}…`;try{const j=await api('/api/mode',{index:i});toast(j.message,j.ok?'ok':'err');}catch(e){toast(e.message,'err');}loadMode();}
$('#mode-refresh').onclick=loadMode;hooks.mode={enter:loadMode};

/* ---------- theme ---------- */
const THEME_PREVIEW={ef:['#0c0a0b','#fa1960','#d80749'],dark:['#0a0c14','#7c4dff','#00bcd4'],light:['#e6ebf5','#a5b4fc','#67e8f9'],ocean:['#02121f','#00e5ff','#2979ff'],amber:['#0d0b07','#ffb300','#ff6d00']};
function applyTheme(t){document.documentElement.dataset.theme=t;}
function renderThemes(){const g=$('#theme-grid');g.innerHTML='';for(const t of META.themes){const p=THEME_PREVIEW[t]||['#111','#888','#555'];
  g.append(h('button',{class:'glass swatch'+(META.theme===t?' on':''),onclick:async()=>{try{await api('/api/theme',{theme:t});META.theme=t;applyTheme(t);renderThemes();}catch(e){toast(e.message,'err');}}},
    h('div',{class:'pv',style:`background:${p[0]}`},h('i',{style:`background:${p[1]};left:14px;top:10px`}),h('i',{style:`background:${p[2]};right:20px;top:24px`})),h('div',{class:'lb'},t,META.theme===t?h('span',{class:'badge ok'},'active'):'')));}}
hooks.theme={enter:renderThemes};

/* ---------- help / clear / exit ---------- */
const KEYS=[['↑ / ↓','walk forward / back'],['← / →','rotate left / right'],['Space / Esc','stop locomotion'],['w / s','end effector x + / −'],['d / a','end effector y outward / inward (mirrored per arm, as in /wbc)'],['q / e','end effector z + / −'],
  ['m','cycle right / left / both hands'],['f','lock / free EE orientation'],['o / c','open / close Dex3 hands'],['r','release / engage arms (IK hold)']];
hooks.help={enter:()=>{const t=$('#help-cmds');t.innerHTML='';for(const[c,d]of Object.entries(META.commands))t.append(h('tr',{},h('td',{},h('a',{href:'#'+c.slice(1),class:'mono',style:'color:var(--accent)'},c)),h('td',{},d)));
  const k=$('#help-keys');k.innerHTML='';for(const[a,b]of KEYS)k.append(h('tr',{},h('td',{},h('kbd',{},a)),h('td',{},b)));
  const w=$('#help-wbc');w.innerHTML='';for(const[a,b]of WBC_HELP)w.append(h('tr',{},h('td',{},h('kbd',{},a)),h('td',{},b)));}};
hooks.clear={enter:async()=>{try{const j=await api('/api/transcript?rev=-1');$('#clear-count').textContent=`${(j.messages||[]).length} messages in the transcript.`;}catch(e){}}};
$('#clear-btn').onclick=async()=>{if(!await confirmDialog('Clear transcript?','All dashboard messages will be removed.','Clear'))return;await api('/api/clear',{});chatRev=-2;toast('Transcript cleared','ok');hooks.clear.enter();};
$('#exit-btn').onclick=async()=>{if(!await confirmDialog('Shut down the dashboard?','The server stops. Locomotion is stopped if /control is active.','Shut down'))return;try{await api('/api/exit',{});}catch(e){}
  document.body.innerHTML='<div class="empty" style="padding-top:30vh"><h2>Dashboard stopped</h2><p>Restart with <code>python3 g1_dashboard.py</code>.</p></div>';};

/* ---------- control: teleop ---------- */
let ctlActive=false,ctl={};const holdTimers={};
async function sendKey(key){try{await api('/api/control/key',{key});}catch(e){toast(e.message,'err');}}
function flash(key){const b=$(`#tab-control [data-key="${key}"]`);if(b){b.classList.add('flash');setTimeout(()=>b.classList.remove('flash'),130);}}
$$('#tab-control [data-key]').forEach(b=>{const key=b.dataset.key;
  if(b.classList.contains('hold')){const stop=()=>{if(holdTimers[key]){clearInterval(holdTimers[key]);delete holdTimers[key];sendKey('stop');}};
    b.addEventListener('pointerdown',e=>{e.preventDefault();sendKey(key);clearInterval(holdTimers[key]);holdTimers[key]=setInterval(()=>sendKey(key),180);});
    ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,stop));}
  else b.addEventListener('click',()=>sendKey(key));});
const KEYMAP={ArrowUp:'ArrowUp',ArrowDown:'ArrowDown',ArrowLeft:'ArrowLeft',ArrowRight:'ArrowRight',' ':'stop',Escape:'stop',w:'w',s:'s',a:'a',d:'d',q:'q',e:'e',m:'m',f:'f',o:'o',c:'c',r:'r'};
const MOTION_KEYS=['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'];
document.addEventListener('keydown',e=>{if(ACTIVE!=='control'||e.ctrlKey||e.metaKey||e.altKey)return;if(e.target.closest&&e.target.closest('input,select,textarea'))return;
  const k=KEYMAP[e.key.length===1?e.key.toLowerCase():e.key];if(!k)return;e.preventDefault();if(e.repeat&&!MOTION_KEYS.includes(k))return;flash(k);sendKey(k);});
document.addEventListener('keyup',e=>{if(ACTIVE==='control'&&MOTION_KEYS.includes(e.key))sendKey('stop');});
window.addEventListener('blur',()=>{if(ACTIVE==='control'&&ctlActive)sendKey('stop');});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&ACTIVE==='control'&&ctlActive)sendKey('stop');});
every('control',300,async()=>{let b;try{b=await api('/api/control/all?hb=1');}catch(e){$('#ctl-status').textContent=e.message;return;}
  const c=b.control||{};if(c.error){$('#ctl-status').textContent=c.error;$('#kbd-state').innerHTML='<span class="dot err"></span>unavailable';}else{ctl=c;ctlActive=c.active;renderControl(c);}
  renderSlam(b.slam||{});renderCamera(b.camera||{});});
function renderControl(j){$('#kbd-state').innerHTML=j.active?'<span class="dot ok"></span>keyboard active':'<span class="dot err"></span>inactive';
  const hm=(j.hand_mode||'right');$('#hand-btn').firstChild.textContent={right:'R',left:'L',both:'LR'}[hm]||'R';$('#orient-btn').firstChild.textContent=j.orientation_locked===false?'🔓':'🔒';
  $('#release-btn').classList.toggle('on',!!j.arms_released);
  const kv=$('#ctl-kv');kv.innerHTML='';const d=j.desired||[0,0,0];
  [['hands',hm.toUpperCase()],['orientation',j.orientation_locked===false?'FREE':'LOCKED'],['arms',j.arms_released?'RELEASED':'ENGAGED'],['motion',`vx ${fmt(d[0])} · vyaw ${fmt(d[2])}`],
   ['IK',!j.ik?'starts on the first arm command':Object.entries(j.ik.info||{}).map(([a,i])=>`${a} ${i.success==null?'pending':i.success?`OK ${fmt(i.error_pos_m,4)} m`:'FAIL (rolled back)'}`).join(' · ')],
   ['steps',j.step?`EE ${j.step.ik_step_m} m · ${j.step.locomotion_speed} m/s · ${j.step.rotation_speed} rad/s`:'—']].forEach(([a,b])=>kv.append(h('div',{},a),h('div',{class:'mono'},b)));
  $('#ctl-status').innerHTML=`<span class="badge ${j.arm_busy?'warn':'ok'}">${j.arm_busy||'idle'}</span> ${j.status||''}`;
  const thr=j.threshold;$('#tac-thr').textContent=thr!=null?`threshold ${thr} · green ≥ threshold`:'';const root=$('#tactile');root.innerHTML='';
  for(const side of['left','right']){const t=(j.tactile||{})[side]||{};const head=h('div',{class:'row'},h('b',{},side.toUpperCase()+' HAND'),h('span',{class:'muted small'},t.available===false?'not detected':t.age==null?'waiting':t.age>2?`stale ${t.age.toFixed(1)}s`:'live'));
    const grid=h('div',{class:'tactile'});(t.values||[]).forEach((v,i)=>grid.append(h('div',{class:'taxel '+(v==null?'na':v>=thr?'hi':'lo')},h('b',{},v==null?'—':Math.round(v)),'S'+i)));
    root.append(h('div',{},head,(t.values||[]).length?grid:h('div',{class:'muted small',style:'margin-top:6px'},t.detail||'Waiting for samples…')));}}

/* ---------- control: SLAM map ---------- */
const mapC=$('#map');const V={cx:0,cy:0,scale:45,yaw:0,tilt:0,follow:true,floor:0,mode:'top'};
let CLOUD={map:new Int16Array(0),live:new Int16Array(0),cells:new Int16Array(0),nMap:0,nLive:0,nCells:0,cell:.1,floor:0},SLAM=null,dirty=true,tool=null,goalDraft=null,drag=null;
function project(x,y,z){const x0=x-V.cx,y0=y-V.cy,z1=z-V.floor,c=Math.cos(V.yaw),s=Math.sin(V.yaw);const x1=c*x0+s*y0,y1=-s*x0+c*y0;const y2=Math.cos(V.tilt)*y1+Math.sin(V.tilt)*z1;
  return[mapC._w/2+x1*V.scale,mapC._h/2-y2*V.scale];}
function toGround(sx,sy){const x1=(sx-mapC._w/2)/V.scale,y2=(mapC._h/2-sy)/V.scale,y1=y2/Math.cos(V.tilt),c=Math.cos(V.yaw),s=Math.sin(V.yaw);return[V.cx+c*x1-s*y1,V.cy+s*x1+c*y1];}
function drawMap(){dirty=false;const[x,w,hh]=setupCanvas(mapC);mapC._w=w;mapC._h=hh;const hmax=(SLAM&&SLAM.params?SLAM.params.max_height:1.8);
  const pose=SLAM&&SLAM.pose;if(V.follow&&pose){V.cx=pose[0];V.cy=pose[1];}V.floor=CLOUD.floor;
  // ground grid
  const [gx0,gy0]=toGround(0,0),[gx1,gy1]=toGround(w,hh),[gx2,gy2]=toGround(w,0),[gx3,gy3]=toGround(0,hh);const minX=Math.min(gx0,gx1,gx2,gx3),maxX=Math.max(gx0,gx1,gx2,gx3),minY=Math.min(gy0,gy1,gy2,gy3),maxY=Math.max(gy0,gy1,gy2,gy3);
  const step=V.scale<12?5:V.scale<30?2:1;x.lineWidth=1;x.strokeStyle=css('--grid');x.beginPath();
  if((maxX-minX)/step<400&&(maxY-minY)/step<400){for(let g=Math.floor(minX/step)*step;g<=maxX;g+=step){const a=project(g,minY,V.floor),b=project(g,maxY,V.floor);x.moveTo(a[0],a[1]);x.lineTo(b[0],b[1]);}
    for(let g=Math.floor(minY/step)*step;g<=maxY;g+=step){const a=project(minX,g,V.floor),b=project(maxX,g,V.floor);x.moveTo(a[0],a[1]);x.lineTo(b[0],b[1]);}}x.stroke();
  // origin axes
  const o=project(0,0,V.floor),ax=project(1,0,V.floor),ay=project(0,1,V.floor);x.lineWidth=2;x.strokeStyle='#ef4444';x.beginPath();x.moveTo(o[0],o[1]);x.lineTo(ax[0],ax[1]);x.stroke();x.strokeStyle='#22c55e';x.beginPath();x.moveTo(o[0],o[1]);x.lineTo(ay[0],ay[1]);x.stroke();
  // occupancy cells
  if($('#ly-cells').checked&&CLOUD.nCells){x.fillStyle=css('--accent2');x.globalAlpha=.22;const c=CLOUD.cell,cl=CLOUD.cells;x.beginPath();
    for(let i=0;i<CLOUD.nCells;i++){const X=cl[2*i]*c,Y=cl[2*i+1]*c;const p=[project(X,Y,V.floor),project(X+c,Y,V.floor),project(X+c,Y+c,V.floor),project(X,Y+c,V.floor)];
      x.moveTo(p[0][0],p[0][1]);x.lineTo(p[1][0],p[1][1]);x.lineTo(p[2][0],p[2][1]);x.lineTo(p[3][0],p[3][1]);x.closePath();}x.fill();x.globalAlpha=1;}
  // points (bucketed by height colour)
  const drawPts=(arr,n,size,alpha,mono)=>{if(!n)return;const buckets=Array.from({length:32},()=>[]);for(let i=0;i<n;i++){const X=arr[3*i]/100,Y=arr[3*i+1]/100,Z=arr[3*i+2]/100;const p=project(X,Y,Z);
      if(p[0]<-4||p[1]<-4||p[0]>w+4||p[1]>hh+4)continue;const b=mono?0:Math.max(0,Math.min(31,Math.round(31*(Z-V.floor)/hmax)));buckets[b].push(p[0],p[1]);}
    x.globalAlpha=alpha;for(let b=0;b<32;b++){const L=buckets[b];if(!L.length)continue;x.fillStyle=mono||TURBO[b];for(let i=0;i<L.length;i+=2)x.fillRect(L[i]-size/2,L[i+1]-size/2,size,size);}x.globalAlpha=1;};
  if($('#ly-map').checked)drawPts(CLOUD.map,CLOUD.nMap,V.scale>60?3:2,.85,null);
  if($('#ly-live').checked)drawPts(CLOUD.live,CLOUD.nLive,V.scale>60?3.5:2.5,.95,css('--text'));
  // named points, tasks, goal
  if(SLAM){if($('#ly-points').checked)for(const[name,p]of Object.entries(SLAM.points||{})){const s=project(p[0],p[1],V.floor);x.fillStyle=css('--warn');x.beginPath();x.arc(s[0],s[1],5,0,7);x.fill();
      x.font='600 11px system-ui';x.fillStyle=css('--text');x.fillText(name,s[0]+8,s[1]-6);}
    const tasks=SLAM.tasks||[];if(tasks.length){x.setLineDash([5,4]);x.strokeStyle=css('--accent');x.lineWidth=1.5;x.beginPath();if(pose){const s=project(pose[0],pose[1],V.floor);x.moveTo(s[0],s[1]);}
      tasks.forEach((t,i)=>{const s=project(t.x,t.y,V.floor);(i||pose)?x.lineTo(s[0],s[1]):x.moveTo(s[0],s[1]);});x.stroke();x.setLineDash([]);
      tasks.forEach((t,i)=>{const s=project(t.x,t.y,V.floor);x.fillStyle=css('--accent');x.beginPath();x.arc(s[0],s[1],9,0,7);x.fill();x.fillStyle='#111';x.font='700 10px system-ui';x.textAlign='center';x.fillText(String(i+1),s[0],s[1]+3.5);x.textAlign='left';});}
    const g=goalDraft||SLAM.goal;if(g)drawArrow(x,g.x,g.y,g.yaw,css('--ok'),goalDraft?.8:.6,true);}
  if(pose)drawArrow(x,pose[0],pose[1],pose[2],css('--accent'),1,false);
  $('#map-hud').innerHTML=`<span>${pose?`pose ${fmt(pose[0])}, ${fmt(pose[1])} · ${fmt(pose[2]*180/Math.PI,0)}°`:'no SLAM pose'}</span><span>${CLOUD.nMap} map · ${CLOUD.nLive} live · ${CLOUD.nCells} cells · grid ${step} m</span>`;
  $('#map-hmax').textContent=`+${hmax} m`;}
function drawArrow(x,X,Y,yaw,color,alpha,ring){const s=project(X,Y,V.floor),L=Math.max(14,0.5*V.scale),tip=project(X+Math.cos(yaw)*L/V.scale,Y+Math.sin(yaw)*L/V.scale,V.floor);
  const ang=Math.atan2(tip[1]-s[1],tip[0]-s[0]);x.globalAlpha=alpha;x.fillStyle=color;x.strokeStyle=color;x.lineWidth=2;
  if(ring){x.beginPath();x.arc(s[0],s[1],10,0,7);x.stroke();}x.beginPath();x.moveTo(tip[0],tip[1]);x.lineTo(s[0]+Math.cos(ang+2.5)*10,s[1]+Math.sin(ang+2.5)*10);x.lineTo(s[0]+Math.cos(ang-2.5)*10,s[1]+Math.sin(ang-2.5)*10);x.closePath();x.fill();
  if(!ring){x.beginPath();x.arc(s[0],s[1],Math.max(4,0.25*V.scale),0,7);x.globalAlpha=alpha*.25;x.fill();}x.globalAlpha=1;}
(function loop(){if(ACTIVE==='control'&&dirty)drawMap();requestAnimationFrame(loop);})();
const markDirty=()=>{dirty=true;};['ly-map','ly-live','ly-cells','ly-points'].forEach(id=>$('#'+id).onchange=markDirty);window.addEventListener('resize',markDirty);
$$('#map-view button').forEach(b=>b.onclick=()=>{V.mode=b.dataset.v;$$('#map-view button').forEach(x=>x.classList.toggle('on',x===b));V.tilt=V.mode==='3d'?0.95:0;if(V.mode==='top')V.yaw=0;dirty=true;});
$('#map-follow').onclick=()=>{V.follow=!V.follow;$('#map-follow').classList.toggle('on',V.follow);dirty=true;};
$('#map-fit').onclick=()=>{const a=CLOUD.map,n=CLOUD.nMap;if(!n)return;let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;for(let i=0;i<n;i++){const X=a[3*i]/100,Y=a[3*i+1]/100;x0=Math.min(x0,X);x1=Math.max(x1,X);y0=Math.min(y0,Y);y1=Math.max(y1,Y);}
  V.follow=false;$('#map-follow').classList.remove('on');V.cx=(x0+x1)/2;V.cy=(y0+y1)/2;V.scale=Math.max(4,Math.min(mapC._w/(x1-x0+1),mapC._h/(y1-y0+1))*.9);dirty=true;};
$('#map-goal').onclick=()=>{tool=tool==='goal'?null:'goal';$('#map-goal').classList.toggle('on',tool==='goal');mapC.classList.toggle('goal',tool==='goal');};
mapC.addEventListener('contextmenu',e=>e.preventDefault());
mapC.addEventListener('pointerdown',e=>{mapC.setPointerCapture(e.pointerId);const r=mapC.getBoundingClientRect(),sx=e.clientX-r.left,sy=e.clientY-r.top;
  if(tool==='goal'&&e.button===0){const[gx,gy]=toGround(sx,sy);goalDraft={x:gx,y:gy,yaw:0};drag={kind:'goal'};}
  else drag={kind:(e.shiftKey||e.button===2)?'orbit':'pan',sx,sy,cx:V.cx,cy:V.cy,yaw:V.yaw,tilt:V.tilt,g:toGround(sx,sy)};dirty=true;});
mapC.addEventListener('pointermove',e=>{if(!drag)return;const r=mapC.getBoundingClientRect(),sx=e.clientX-r.left,sy=e.clientY-r.top;
  if(drag.kind==='goal'){const[gx,gy]=toGround(sx,sy);if(Math.hypot(gx-goalDraft.x,gy-goalDraft.y)>0.05)goalDraft.yaw=Math.atan2(gy-goalDraft.y,gx-goalDraft.x);}
  else if(drag.kind==='pan'){V.follow=false;$('#map-follow').classList.remove('on');const save=[V.cx,V.cy];V.cx=drag.cx;V.cy=drag.cy;const g=toGround(sx,sy);V.cx=drag.cx+(drag.g[0]-g[0]);V.cy=drag.cy+(drag.g[1]-g[1]);if(!isFinite(V.cx)){[V.cx,V.cy]=save;}}
  else{V.yaw=drag.yaw-(sx-drag.sx)*0.008;V.tilt=Math.max(0,Math.min(1.25,drag.tilt+(sy-drag.sy)*0.006));}dirty=true;});
mapC.addEventListener('pointerup',()=>{if(drag&&drag.kind==='goal'&&goalDraft){$('#goalbar').classList.remove('hidden');$('#goal-text').textContent=`x ${fmt(goalDraft.x)} · y ${fmt(goalDraft.y)} · ${fmt(goalDraft.yaw*180/Math.PI,0)}°`;}drag=null;});
mapC.addEventListener('wheel',e=>{e.preventDefault();const r=mapC.getBoundingClientRect(),sx=e.clientX-r.left,sy=e.clientY-r.top;const before=toGround(sx,sy);V.scale=Math.max(3,Math.min(400,V.scale*Math.exp(-e.deltaY*0.0015)));
  if(!V.follow){const after=toGround(sx,sy);V.cx+=before[0]-after[0];V.cy+=before[1]-after[1];}dirty=true;},{passive:false});
function clearGoal(){goalDraft=null;$('#goalbar').classList.add('hidden');dirty=true;}
$('#goal-cancel').onclick=clearGoal;
$('#goal-nav').onclick=async()=>{const g=goalDraft;if(!g)return;if(!await confirmDialog('Navigate to goal?',`pose_nav to x ${fmt(g.x)}, y ${fmt(g.y)}, yaw ${fmt(g.yaw*180/Math.PI,0)}°. The robot will walk.`,'Navigate'))return;await slamAction({action:'pose_nav',...g});clearGoal();};
$('#goal-queue').onclick=async()=>{const g=goalDraft;if(!g)return;await slamAction({action:'queue_add',...g});clearGoal();};
async function slamAction(body){try{const j=await api('/api/slam/action',body);toast(`${j.label||body.action}: ${j.message||'ok'}`,j.ok===false?'err':'ok');pollSlam();return j;}catch(e){toast(e.message,'err');}}
const RISKY={start_mapping:'Start a new map? The current cloud view is cleared.',close_slam:'Close SLAM without saving the map?',relocate:'Relocate against the saved map at the current pose?',queue_run:'Run the task queue? The robot walks to each target in order.',resume:'Resume navigation?'};
$$('[data-slam]').forEach(b=>b.onclick=async()=>{const action=b.dataset.slam;if(RISKY[action]&&!await confirmDialog(b.textContent,RISKY[action]))return;
  slamAction({action,slam_type:$('#slam-type').value,path:$('#slam-path').value.trim()||undefined});});
$('#point-form').addEventListener('submit',e=>{e.preventDefault();const name=$('#point-name').value.trim();if(!name)return;$('#point-name').value='';slamAction({action:'add_point',name});});
async function pollSlam(){try{renderSlam(await api('/api/slam'));}catch(e){renderSlam({error:e.message});}}
function renderSlam(s){if(s.error){$('#slam-kv').innerHTML='';$('#slam-kv').append(h('div',{},'status'),h('div',{},s.error));return;}SLAM=s;dirty=true;
  if(!$('#slam-path').value&&s.map_path)$('#slam-path').placeholder=s.map_path;
  const n=s.notice||{};$('#slam-badges').innerHTML=`<span class="badge ${s.mapping?'ok':''}">${s.mapping?'mapping':'not mapping'}</span><span class="badge ${s.relocated?'ok':''}">${s.relocated?'relocated':'not relocated'}</span>`+(n.obstacle_blocked?'<span class="badge err">path blocked</span>':'');
  const kv=$('#slam-kv');kv.innerHTML='';const fresh=s.topics.filter(t=>t.age!=null&&t.age<3).length;
  [['pose',s.pose?`${fmt(s.pose[0])}, ${fmt(s.pose[1])} · ${fmt(s.pose[2]*180/Math.PI,0)}°`:'—'],['map file',s.map_path],['notice',n.info?`${n.info}${n.is_arrived?' · arrived':''}`:(n.type||'—')],
   ['topics',`${fresh}/${s.topics.length} fresh`],['floor',s.stats&&s.stats.floor_z!=null?fmt(s.stats.floor_z)+' m':'—']].forEach(([a,b])=>kv.append(h('div',{},a),h('div',{class:'mono',title:a==='topics'?s.topics.map(t=>`${t.topic}: ${t.age==null?'never':t.age.toFixed(1)+'s'}`).join('\n'):''},b)));
  const ts=s.task_state||{};$('#task-state').innerHTML=ts.running?`<span class="badge ${ts.phase==='blocked'?'err':'warn'}">${ts.phase}</span> target ${ts.index+1}/${ts.total} ${ts.target?ts.target.name:''} · ${fmt(ts.elapsed||0,0)} s`:ts.message?`<span class="badge ${ts.phase==='done'?'ok':'err'}">${ts.phase}</span> ${ts.message}`:'';
  const pl=$('#point-list');pl.innerHTML='';const pts=Object.entries(s.points||{});if(!pts.length)pl.append(h('div',{class:'muted small'},'No named points saved yet.'));
  for(const[name,p]of pts)pl.append(h('div',{class:'li'},h('span',{class:'grow'},h('b',{},name),' ',h('span',{class:'mono muted'},`${fmt(p[0])}, ${fmt(p[1])}`)),
    h('button',{class:'btn sm',onclick:async()=>{if(await confirmDialog(`Navigate to ${name}?`,'The robot walks to this saved point.','Go'))slamAction({action:'goto_point',name});}},'Go'),
    h('button',{class:'btn sm ghost',onclick:()=>slamAction({action:'queue_add_point',name})},'+Q'),
    h('button',{class:'btn sm ghost danger',onclick:async()=>{if(await confirmDialog(`Delete ${name}?`,'Removes it from slam_points.json.','Delete'))slamAction({action:'remove_point',name});}},'✕')));
  const tl=$('#task-list');tl.innerHTML='';if(!(s.tasks||[]).length)tl.append(h('div',{class:'muted small'},'Queue is empty. Use Set goal → Queue, or + buttons.'));
  (s.tasks||[]).forEach((t,i)=>tl.append(h('div',{class:'li'},h('span',{class:'badge'},String(i+1)),h('span',{class:'grow mono'},`${t.name} · ${fmt(t.x)}, ${fmt(t.y)}`),h('button',{class:'btn sm ghost',onclick:()=>slamAction({action:'queue_remove',index:i})},'✕'))));
  const la=s.last_action||{};$('#slam-last').textContent=`${la.label||''} · ${la.ok?'ok':'failed'} · ${la.message||''}${la.raw?'\n'+la.raw:''}`;
  const st=s.stats||{};$('#proc-stats').textContent=st.proc_ms!=null?`cloud ${st.proc_ms} ms · ${st.map_voxels||0} voxels`+(st.error?` · ${st.error}`:''):'';
  if(!paramsBuilt)buildParams(s.params,s.live_topics);}
async function pollCloud(){const r=await fetch('/api/slam/cloud');if(!r.ok)return;const buf=await r.arrayBuffer();if(buf.byteLength<20)return;const dv=new DataView(buf);
  const nMap=dv.getUint32(0,true),nLive=dv.getUint32(4,true),nCells=dv.getUint32(8,true),cell=dv.getFloat32(12,true),floor=dv.getFloat32(16,true);
  const all=new Int16Array(buf,20,(nMap+nLive)*3+nCells*2);CLOUD={map:all.subarray(0,nMap*3),live:all.subarray(nMap*3,(nMap+nLive)*3),cells:all.subarray((nMap+nLive)*3),nMap,nLive,nCells,cell,floor};dirty=true;}
every('control',350,pollCloud);

/* ---------- control: camera ---------- */
const rgbImg=$('#cam-rgb'),depthImg=$('#cam-depth');
// MJPEG-style multipart stream first; if no frame shows within 4 s (e.g. a
// browser that will not play PNG parts), poll single frames instead.
const CAMS=[[rgbImg,'rgb','#cam-rgb-ph'],[depthImg,'depth','#cam-depth-ph']];let camTimers=[];
function snap(img,kind){if(ACTIVE!=='control'||img.dataset.mode!=='snap')return;img.src=`/frame/${kind}?${Date.now()}`;}
function startStreams(){stopStreams();for(const[img,kind]of CAMS){img.dataset.mode='stream';img.src=`/stream/${kind}?${Date.now()}`;
  camTimers.push(setTimeout(()=>{if(ACTIVE==='control'&&!img.naturalWidth){img.dataset.mode='snap';snap(img,kind);}},4000));}}
function stopStreams(){camTimers.forEach(clearTimeout);camTimers=[];for(const[img]of CAMS){img.dataset.mode='off';img.removeAttribute('src');}}
for(const[img,kind,ph]of CAMS){img.onload=()=>{$(ph).classList.add('hidden');if(img.dataset.mode==='snap')setTimeout(()=>snap(img,kind),120);};
  img.onerror=()=>{if(img.dataset.mode==='snap')setTimeout(()=>snap(img,kind),1000);};}
function renderCamera(j){if(j.error){$('#cam-msg').textContent=j.error;return;}
  $('#cam-msg').textContent=j.message||'';$('#cam-fps').textContent=j.ok?`${j.fps} fps`:'';
  if(!j.ok){$('#cam-rgb-ph').textContent=$('#cam-depth-ph').textContent=j.message;$('#cam-rgb-ph').classList.remove('hidden');$('#cam-depth-ph').classList.remove('hidden');}
  $('#depth-max').textContent=`${j.params.depth_max} m`;const ol=$('#obs-list');ol.innerHTML='';
  for(const o of j.obstacles||[])ol.append(h('span',{class:'obs',style:`border-color:${o.distance<.8?'var(--err)':o.distance<1.5?'var(--warn)':'var(--ok)'}`},h('b',{},'#'+o.id),`${o.distance.toFixed(2)} m · ${o.size[0].toFixed(2)}×${o.size[1].toFixed(2)} m · x ${o.center[0].toFixed(2)}`));
  if(j.ok&&!(j.obstacles||[]).length)ol.append(h('span',{class:'muted small'},'No obstacles in range.'));
  if(!camParamsBuilt)buildCamParams(j.params);}
let depthBusy=false;
$('#depth-box').addEventListener('mousemove',e=>{if(depthBusy)return;const r=depthImg.getBoundingClientRect();if(!r.width)return;
  const iw=depthImg.naturalWidth||4,ih=depthImg.naturalHeight||3,sc=Math.min(r.width/iw,r.height/ih),ox=(r.width-iw*sc)/2,oy=(r.height-ih*sc)/2;
  const u=(e.clientX-r.left-ox)/(iw*sc),v=(e.clientY-r.top-oy)/(ih*sc);if(u<0||v<0||u>1||v>1)return;depthBusy=true;
  api(`/api/camera/depth?u=${u.toFixed(4)}&v=${v.toFixed(4)}`).then(j=>{$('#depth-read').textContent=j.m==null?'no depth here':`${j.m.toFixed(3)} m @ ${j.x},${j.y}`;}).catch(()=>{}).finally(()=>setTimeout(()=>depthBusy=false,120));});

/* ---------- control: processing params ---------- */
let paramsBuilt=false,camParamsBuilt=false;
function paramRow(label,val,min,max,step,on){const out=h('output',{},String(val));const r=h('input',{type:'range',min,max,step,value:val});r.oninput=()=>{out.textContent=r.value;on(+r.value);};return h('div',{class:'param'},h('span',{},label),r,out);}
function checkRow(label,val,on){return h('div',{class:'param'},h('span',{},label),h('label',{class:'switch'},h('input',{type:'checkbox',checked:!!val,onchange:e=>on(e.target.checked)}),h('span')),h('span'));}
const sendParams=(url,key)=>{let t;const acc={};return(k,v)=>{acc[k]=v;clearTimeout(t);t=setTimeout(async()=>{const body={...acc};for(const x in acc)delete acc[x];try{await api(url,body);}catch(e){toast(e.message,'err');}},key||300);};};
const cloudSend=sendParams('/api/slam/params'),camSend=sendParams('/api/camera/params');
function buildParams(p,liveTopics){paramsBuilt=true;const root=$('#cloud-params');root.innerHTML='';
  root.append(checkRow('auto floor',p.auto_floor,v=>cloudSend('auto_floor',v)),paramRow('floor z (manual)',p.floor_z,-2,2,.01,v=>cloudSend('floor_z',v)),paramRow('clearance m',p.clearance,0,.6,.01,v=>cloudSend('clearance',v)),
    paramRow('max height m',p.max_height,.3,3,.05,v=>cloudSend('max_height',v)),paramRow('voxel m',p.voxel,.02,.3,.01,v=>cloudSend('voxel',v)),paramRow('cell m',p.cell,.05,.5,.01,v=>cloudSend('cell',v)),
    paramRow('max points',p.max_points,2000,60000,1000,v=>cloudSend('max_points',v)),checkRow('accumulate live',p.accumulate_live,v=>cloudSend('accumulate_live',v)));
  const sel=h('select',{onchange:e=>cloudSend('live_topic',e.target.value)},...(liveTopics||[]).map(t=>h('option',{value:t,selected:t===p.live_topic},t.replace('rt/utlidar/',''))));
  const fr=h('select',{onchange:e=>cloudSend('live_frame',e.target.value)},h('option',{value:'body',selected:p.live_frame==='body'},'body → map (use pose)'),h('option',{value:'world',selected:p.live_frame==='world'},'already in map frame'));
  root.append(h('div',{class:'param'},h('span',{},'live topic'),sel,h('span')),h('div',{class:'param'},h('span',{},'live frame'),fr,h('span')));}
function buildCamParams(p){camParamsBuilt=true;const root=$('#cam-params');root.innerHTML='';
  root.append(paramRow('min range m',p.min_m,.1,2,.05,v=>camSend('min_m',v)),paramRow('max range m',p.max_m,.5,8,.1,v=>camSend('max_m',v)),checkRow('remove floor',p.floor,v=>camSend('floor',v)),
    paramRow('floor tol m',p.floor_tol,.01,.12,.005,v=>camSend('floor_tol',v)),paramRow('max height m',p.max_height,.3,3,.05,v=>camSend('max_height',v)),paramRow('edge split m',p.edge_m,.02,.5,.01,v=>camSend('edge_m',v)),
    paramRow('min area',p.min_area,.0005,.05,.0005,v=>camSend('min_area',v)),paramRow('depth colour max',p.depth_max,1,8,.5,v=>camSend('depth_max',v)),checkRow('floor tint',p.floor_tint,v=>camSend('floor_tint',v)),paramRow('max Hz',p.max_hz,1,30,1,v=>camSend('max_hz',v)));}

hooks.control={enter:async()=>{startStreams();dirty=true;try{const j=await api('/api/control/enter',{});ctlActive=j.active;}catch(e){toast(e.message,'err');$('#kbd-state').innerHTML='<span class="dot err"></span>'+e.message;}},
  leave:()=>{stopStreams();Object.keys(holdTimers).forEach(k=>{clearInterval(holdTimers[k]);delete holdTimers[k];});if(ctlActive)api('/api/control/exit',{}).catch(()=>{});ctlActive=false;}};
window.addEventListener('pagehide',()=>{if(ACTIVE==='control')navigator.sendBeacon&&navigator.sendBeacon('/api/control/exit',new Blob(['{}'],{type:'application/json'}));});

/* ---------- wbc ---------- */
const WBC_PARAMS=[['rate_hz','Publish rate (Hz)',1],['speed_rad_s','Joint ramp speed (rad/s)',0.01],['max_dq','Max joint Δ per IK step (rad)',0.005],
  ['kp','Arm kp',1],['kd','Arm kd',0.1],['waist_pr_kp','Waist pitch/roll kp',5],['ee_speed_m_s','Right stick EE x speed (m/s)',0.01],
  ['ee_y_speed_m_s','Right stick EE y speed (m/s)',0.01],['max_vx','Left stick max vx (m/s)',0.05],['max_vyaw','Left stick max yaw (rad/s)',0.05],
  ['loco_deadzone','Left stick deadzone',0.05],['pos_step','EE position step (m)',0.001],['rot_step','EE rotation step (rad)',0.01],
  ['hand_kp','Dex3 kp',0.1],['hand_kd','Dex3 kd',0.01],['hand_tau','Dex3 feed-forward τ',0.01],['hand_rate_hz','Dex3 refresh (Hz)',1]];
const WBC_HELP=[['Left stick','walk forward/back and turn (leased Move; StopMove repeated for 0.6 s on release)'],['Right stick','EE x (push/pull) and EE y (left/right)'],
  ['D-pad ↑ / ↓','EE z by the position step'],['D-pad ← / →','EE roll by the rotation step'],['A','toggle Dex3 grip open / closed'],['B','release / re-engage arms'],
  ['X','arms right → left → both'],['Y','lock / free EE orientation'],['↑ ↓ or k j','select DOF'],['← → or − +','jog the selected DOF by its step'],
  ['< / >','halve / double the selected step'],['[ / ]','halve / double max Δq'],['m','cycle arms'],['f','orientation lock'],['w','waist hold on / off'],
  ['y','resync EE targets to the live pose'],['r / e','release / engage arms'],['z','send one zero-gain packet'],['H','cycle Dex3 hand mode'],
  ['{ / }','grip −5 / +5 %'],['p','name and save the commanded pose'],['R / S','run / stop the replay sequence']];
let WBC={active:false},wbcArm='right',wbcHand='off',wbcParamsBuilt=false,wbcDofRows=null;
const fmtq=(v,d=3)=>v==null||!isFinite(v)?'—':(v>=0?'+':'')+Number(v).toFixed(d);
async function wbcAct(action,extra){try{const j=await api('/api/wbc/action',{action,...(extra||{})});if(j.status)$('#wbc-status').textContent=j.status;pollWbc();return j;}catch(e){toast(e.message,'err');}}
async function pollWbc(){let s;try{s=await api('/api/wbc');}catch(e){$('#wbc-status').textContent=e.message;return;}
  if(s.busy){$('#wbc-status').textContent=s.status||'busy…';return;}WBC=s;renderWbc(s);}
every('wbc',250,pollWbc);
function buildWbcDofs(){const t=$('#wbc-dofs');t.innerHTML='';t.append(h('tr',{},h('th',{},'DOF'),h('th',{},'live FK'),h('th',{},'target'),h('th',{},'step'),h('th',{},'jog')));
  wbcDofRows=[0,1,2,3,4,5].map(i=>{const c={name:h('b'),free:h('span',{class:'muted small'}),live:h('td',{class:'num'}),target:h('td',{class:'num'}),step:h('span',{class:'mono'})};
    c.tr=h('tr',{style:'cursor:pointer',onclick:e=>{if(!e.target.closest('button'))wbcAct('select_dof',{dof:i});}},h('td',{},c.name,c.free),c.live,c.target,
      h('td',{class:'num'},h('button',{class:'btn sm ghost',title:'halve step',onclick:()=>wbcAct('step',{dof:i,op:'halve'})},'½'),' ',c.step,' ',
        h('button',{class:'btn sm ghost',title:'double step',onclick:()=>wbcAct('step',{dof:i,op:'double'})},'×2')),
      h('td',{},h('button',{class:'btn sm',onclick:()=>wbcAct('jog',{dof:i,dir:-1})},'−'),' ',h('button',{class:'btn sm',onclick:()=>wbcAct('jog',{dof:i,dir:1})},'+')));
    t.append(c.tr);return c;});}
function renderWbcInput(inp){const[x,w,hh]=setupCanvas($('#wbc-pad'));const ax=(inp&&inp.axes)||[],bt=(inp&&inp.buttons)||[],hat=((inp&&inp.hats)||[[0,0]])[0]||[0,0];
  const r=Math.min(hh*0.34,w*0.16),cy=hh*0.42;
  const stick=(cx,sx,sy,label)=>{x.strokeStyle=css('--border');x.lineWidth=2;x.beginPath();x.arc(cx,cy,r,0,7);x.stroke();
    x.fillStyle=css('--accent');x.beginPath();x.arc(cx+(+sx||0)*r,cy+(+sy||0)*r,6,0,7);x.fill();
    x.fillStyle=css('--muted');x.font='11px system-ui';x.textAlign='center';x.fillText(label,cx,cy+r+15);x.textAlign='left';};
  stick(w*0.2,ax[0],ax[1],'walk · turn');stick(w*0.8,ax[2],ax[3],'EE x · y');
  const hx=w*0.5,hy=cy-r*0.4;[[0,-1,hat[1]>0],[0,1,hat[1]<0],[-1,0,hat[0]<0],[1,0,hat[0]>0]].forEach(([dx,dy,lit])=>{x.fillStyle=lit?css('--accent'):css('--glass2');x.fillRect(hx+dx*11-5,hy+dy*11-5,10,10);});
  [['A',0],['B',1],['X',2],['Y',3]].forEach(([n,i],k)=>{const bx=w*0.5+(k-1.5)*22,by=cy+r*0.5;x.fillStyle=bt[i]?css('--accent'):css('--glass2');x.beginPath();x.arc(bx,by,9,0,7);x.fill();
    x.fillStyle=bt[i]?'#111':css('--text');x.font='700 10px system-ui';x.textAlign='center';x.fillText(n,bx,by+3.5);x.textAlign='left';});
  const kv=$('#wbc-input-kv');kv.innerHTML='';
  if(!inp){$('#wbc-input-badge').innerHTML='';kv.append(h('div',{},'input'),h('div',{},'no session'));}
  else{$('#wbc-input-badge').innerHTML=inp.stale?'<span class="badge err">stale · locomotion stopped</span>':'<span class="badge ok">live</span>';
    [['source',inp.url],['link',inp.error||'—'],['age',inp.age==null?'—':inp.age.toFixed(2)+' s'],
     ['locomotion',`vx ${fmt(inp.loco[0])} m/s · yaw ${fmt(inp.loco[1])} rad/s`]].forEach(([a,b])=>kv.append(h('div',{},a),h('div',{class:'mono'},b)));}
  $('#wbc-gp-hint').textContent='Input from controller_stream.py on the machine with the gamepad (modules/scripts/remote_controller_stream). From the browser, use the arm buttons in /control.';}
function renderWbcParams(s){const vals=s.active?s.params:(s.defaults||{});const root=$('#wbc-params');
  if(!wbcParamsBuilt&&Object.keys(vals).length){wbcParamsBuilt=true;root.innerHTML='';
    for(const[k,label,step]of WBC_PARAMS)root.append(h('div',{class:'param',style:'grid-template-columns:1fr 96px'},h('span',{},label),
      h('input',{type:'number',step,'data-k':k,style:'width:96px',oninput:e=>{if(!WBC.active)e.target.dataset.dirty='1';},onchange:e=>{if(WBC.active)wbcAct('params',{[k]:+e.target.value});}})));}
  $('#wbc-params-note').textContent=s.active?'applied live':'used when the session starts';
  $$('#wbc-params input').forEach(i=>{if(document.activeElement!==i&&!i.dataset.dirty&&vals[i.dataset.k]!=null)i.value=+Number(vals[i.dataset.k]).toFixed(4);});}
function renderWbc(s){const on=!!s.active,inp=s.input||{};
  $$('#tab-wbc .wbc-live').forEach(e=>e.classList.toggle('wbc-off',!on));$('#wbc-start').disabled=on;$('#wbc-stop').disabled=!on;
  if(on){wbcArm=s.arm_mode;wbcHand=s.hand.mode;}
  const url=$('#wbc-url');if(document.activeElement!==url&&!url.dataset.dirty)url.value=on?inp.url:(s.remote_url||'');
  $('#wbc-badges').innerHTML=on?`<span class="badge ${s.seeded?'ok':'err'}">${s.seeded?'connected':'waiting for lowstate'}</span><span class="badge ${s.armed?'ok':'warn'}">${s.armed?'armed':'released'}</span>`
    +(s.sequence.running?'<span class="badge warn">sequence running</span>':''):'<span class="badge">no session</span>';
  $('#wbc-status').textContent=on?s.status:(s.error||'Start a session to stream rt/arm_sdk targets. Only one IK pose session (this tab or ik_pose_cli_v4.py) can run at a time.');
  $$('#wbc-arm button').forEach(b=>b.classList.toggle('on',b.dataset.v===wbcArm));$$('#wbc-hand button').forEach(b=>b.classList.toggle('on',b.dataset.v===wbcHand));
  renderWbcInput(on?inp:null);renderWbcParams(s);
  if(!on)return;
  $('#wbc-orient').firstChild.textContent=s.orient_stiff?'Orientation locked ':'Orientation free ';$('#wbc-orient').classList.toggle('on',s.orient_stiff);
  $('#wbc-waist').firstChild.textContent=s.waist_enabled?'Waist held ':'Waist free ';$('#wbc-waist').classList.toggle('on',s.waist_enabled);
  $('#wbc-disp-arm').textContent=s.display_arm;$('#wbc-maxdq').textContent=fmt(s.params.max_dq,3)+' rad';
  if(!wbcDofRows)buildWbcDofs();
  s.dofs.forEach((d,i)=>{const c=wbcDofRows[i];c.tr.classList.toggle('sel',i===s.dof_idx);c.name.textContent=d.name;c.free.textContent=d.free?' · orientation free':'';
    c.live.textContent=`${fmtq(d.live,4)} ${d.unit}`;c.target.textContent=`${fmtq(d.target,4)} ${d.unit}`;c.step.textContent=Number(d.step).toFixed(4);});
  const ik=$('#wbc-ik');ik.innerHTML='';for(const[arm,info]of Object.entries(s.ik||{})){const ok=info.success;
    ik.append(h('div',{style:`color:${ok==null?'var(--muted)':ok?'var(--ok)':'var(--err)'}`},ok==null?`IK ${arm}: pending`:ok
      ?`IK ${arm}: OK  pos=${fmt(info.error_pos_m,4)} m  rot=${fmt(info.error_rot_rad,4)} rad  ${info.iterations} it`+(info.mode==='pos_shoulder_elbow'?'  · shoulder+elbow':info.mode==='pos_axis_clamped'?`  · axis-clamped (${fmt(info.axis_error_m,4)} m)`:'')
      :`IK ${arm}: FAIL — target rolled back (pos=${fmt(info.error_pos_m,4)} m  rot=${fmt(info.error_rot_rad,4)} rad)`));}
  const grip=$('#wbc-grip');if(document.activeElement!==grip)grip.value=s.hand.grip;$('#wbc-grip-out').textContent=Math.round(s.hand.grip)+'%';
  const info=Object.entries(s.hand.info||{});$('#wbc-hand-info').textContent=info.length?info.map(([k,v])=>`${k}: ${v}`).join(' · '):'Hand control off — A, the grip buttons or H turn it on.';
  const jr=$('#wbc-joints');jr.className='';jr.innerHTML='';const tb=h('table',{class:'t small'});tb.append(h('tr',{},h('th',{},''),...s.joint_labels.map(l=>h('th',{},l))));
  for(const arm of(s.arm_mode==='both'?['left','right']:[s.arm_mode])){const j=s.joints[arm];
    tb.append(h('tr',{},h('td',{},h('b',{},arm),' target'),...j.target.map(v=>h('td',{class:'num'},fmtq(v)))),h('tr',{},h('td',{class:'muted'},'live'),...j.live.map(v=>h('td',{class:'num muted'},fmtq(v)))));}
  jr.append(tb);$('#wbc-pose-file').textContent=s.pose_file;
  const pl=$('#wbc-poses'),psig=JSON.stringify(s.poses);if(pl.dataset.sig!==psig){pl.dataset.sig=psig;pl.innerHTML='';if(!s.poses.length)pl.append(h('div',{class:'muted small'},'No saved poses yet.'));
    s.poses.forEach((p,i)=>pl.append(h('div',{class:'li'},h('span',{class:'grow'},h('b',{},p.name),' ',h('span',{class:'muted small mono'},p.saved_at.replace('T',' '))),
      h('button',{class:'btn sm',onclick:()=>wbcAct('load_pose',{index:i})},'Load'),h('button',{class:'btn sm ghost',onclick:()=>wbcAct('seq_add',{index:i})},'+Seq'),
      h('button',{class:'btn sm ghost danger',onclick:async()=>{if(await confirmDialog(`Delete pose ${p.name}?`,'It is removed from the pose file and from the sequence.','Delete'))wbcAct('delete_pose',{index:i});}},'✕'))));}
  const sq=s.sequence;$('#wbc-seq-waist').textContent=sq.include_waist_new?'New steps include waist':'New steps: arms only';
  const gap=$('#wbc-gap');if(document.activeElement!==gap)gap.value=s.params.sequence_gap_s;
  const sl=$('#wbc-seq'),ssig=JSON.stringify([sq.steps,sq.running&&sq.index]);if(sl.dataset.sig!==ssig){sl.dataset.sig=ssig;sl.innerHTML='';if(!sq.steps.length)sl.append(h('div',{class:'muted small'},'Add saved poses with +Seq.'));
    sq.steps.forEach((st,i)=>sl.append(h('div',{class:'li',style:sq.running&&i===sq.index-1?'box-shadow:inset 0 0 0 2px var(--accent)':''},h('span',{class:'badge'},String(i+1)),h('span',{class:'grow'},st.name),
      h('button',{class:'btn sm ghost',title:'toggle waist for this step',onclick:()=>wbcAct('seq_toggle_waist',{index:i})},st.include_waist?'waist':'arms'),
      h('button',{class:'btn sm ghost',onclick:()=>wbcAct('seq_move',{index:i,dir:-1})},'↑'),h('button',{class:'btn sm ghost',onclick:()=>wbcAct('seq_move',{index:i,dir:1})},'↓'),
      h('button',{class:'btn sm ghost danger',onclick:()=>wbcAct('seq_remove',{index:i})},'✕'))));}}
$('#wbc-url').oninput=e=>{e.target.dataset.dirty='1';};
$('#wbc-url-apply').onclick=()=>{const url=$('#wbc-url').value.trim();delete $('#wbc-url').dataset.dirty;if(WBC.active)wbcAct('source',{url});};
$$('#wbc-arm button').forEach(b=>b.onclick=()=>{if(WBC.active)wbcAct('arm_mode',{mode:b.dataset.v});else{wbcArm=b.dataset.v;renderWbc(WBC);}});
$$('#wbc-hand button').forEach(b=>b.onclick=()=>{if(WBC.active)wbcAct('hand_mode',{mode:b.dataset.v});else{wbcHand=b.dataset.v;renderWbc(WBC);}});
$$('#tab-wbc [data-wbc]').forEach(b=>b.onclick=()=>wbcAct(b.dataset.wbc));
$$('#tab-wbc [data-wbc-dq]').forEach(b=>b.onclick=()=>wbcAct('max_dq',{op:b.dataset.wbcDq}));
$$('#tab-wbc [data-grip]').forEach(b=>b.onclick=()=>wbcAct('grip',{percent:+b.dataset.grip}));
$$('#tab-wbc [data-grip-delta]').forEach(b=>b.onclick=()=>wbcAct('grip',{delta:+b.dataset.gripDelta}));
$('#wbc-grip').oninput=e=>{$('#wbc-grip-out').textContent=e.target.value+'%';};$('#wbc-grip').onchange=e=>wbcAct('grip',{percent:+e.target.value});
$('#wbc-gap').onchange=e=>wbcAct('params',{sequence_gap_s:+e.target.value});
$('#wbc-zero').onclick=async()=>{if(await confirmDialog('Send a zero-gain packet?','All arm and waist gains go to zero once: the arms drop until the next control tick re-commands them.','Send'))wbcAct('zero_gains');};
$('#wbc-save-form').addEventListener('submit',e=>{e.preventDefault();const name=$('#wbc-pose-name').value.trim();if(!name)return;$('#wbc-pose-name').value='';$('#wbc-pose-name').blur();wbcAct('save_pose',{name});});
$('#wbc-start').onclick=async()=>{if(!await confirmDialog('Start the /wbc session?','The dashboard takes rt/arm_sdk authority (1 s ramp) and holds both arms at their current pose. The gamepad then moves the end effector and walks the robot.','Start'))return;
  const params={};$$('#wbc-params input').forEach(i=>{if(i.value!=='')params[i.dataset.k]=+i.value;});
  $('#wbc-status').textContent='Starting… waiting for rt/lowstate and ramping arm authority.';
  try{const j=await api('/api/wbc/start',{url:$('#wbc-url').value.trim(),arm_control:wbcArm,hand_control:wbcHand,params});
    $$('#wbc-params input').forEach(i=>delete i.dataset.dirty);delete $('#wbc-url').dataset.dirty;WBC=j;renderWbc(j);toast('/wbc session started','ok');}
  catch(e){toast(e.message,'err');$('#wbc-status').textContent=e.message;}};
$('#wbc-stop').onclick=async()=>{if(!await confirmDialog('Stop the /wbc session?','Locomotion stops and arm authority fades out over 3 s, handing the arms back to the default controller.','Stop'))return;
  $('#wbc-status').textContent='Stopping… fading arm authority out.';try{const j=await api('/api/wbc/stop',{});WBC=j;renderWbc(j);}catch(e){toast(e.message,'err');}};
document.addEventListener('keydown',e=>{if(ACTIVE!=='wbc'||!WBC.active||e.ctrlKey||e.metaKey||e.altKey)return;if(e.target.closest&&e.target.closest('input,select,textarea'))return;
  const k=e.key,d=WBC.dof_idx||0,jog=['ArrowLeft','ArrowRight','-','+','='].includes(k);let call=null;
  if(k==='ArrowUp'||k==='k')call=['select_dof',{dof:Math.max(0,d-1)}];else if(k==='ArrowDown'||k==='j')call=['select_dof',{dof:Math.min(5,d+1)}];
  else if(k==='ArrowLeft'||k==='-')call=['jog',{dir:-1}];else if(k==='ArrowRight'||k==='+'||k==='=')call=['jog',{dir:1}];
  else if(k==='p'){e.preventDefault();$('#wbc-pose-name').focus();return;}
  else call={'<':['step',{op:'halve'}],'>':['step',{op:'double'}],'[':['max_dq',{op:'halve'}],']':['max_dq',{op:'double'}],m:['arm_mode',{}],f:['orient',{}],y:['resync',{}],
    r:['release',{}],e:['engage',{}],z:['zero_gains',{}],w:['waist',{}],H:['hand_mode',{}],'{':['grip',{delta:-5}],'}':['grip',{delta:5}],R:['seq_run',{}],S:['seq_stop',{}]}[k]||null;
  if(!call)return;e.preventDefault();if(e.repeat&&!jog)return;wbcAct(call[0],call[1]);});
hooks.wbc={enter:()=>{const t=$('#wbc-help');if(!t.rows.length)for(const[a,b]of WBC_HELP)t.append(h('tr',{},h('td',{},h('kbd',{},a)),h('td',{},b)));pollWbc();}};

/* ---------- sort ---------- */
let sortView='sort',sortBuilt=false,sortCfgTimer=null,sortCamTimer=null;
const sortImg=$('#sort-img');
function sortSnap(){if(ACTIVE!=='sort'||sortImg.dataset.mode!=='snap')return;sortImg.src=`/frame/sort?view=${sortView}&${Date.now()}`;}
function startSortStream(){stopSortStream();sortImg.dataset.mode='stream';sortImg.src=`/stream/sort?view=${sortView}&${Date.now()}`;
  sortCamTimer=setTimeout(()=>{if(ACTIVE==='sort'&&!sortImg.naturalWidth){sortImg.dataset.mode='snap';sortSnap();}},5000);}
function stopSortStream(){clearTimeout(sortCamTimer);sortImg.dataset.mode='off';sortImg.removeAttribute('src');}
sortImg.onload=()=>{$('#sort-ph').classList.add('hidden');if(sortImg.dataset.mode==='snap')setTimeout(sortSnap,150);};
sortImg.onerror=()=>{if(sortImg.dataset.mode==='snap')setTimeout(sortSnap,1500);};
$$('#sort-view-sel button').forEach(b=>b.onclick=()=>{sortView=b.dataset.v;$$('#sort-view-sel button').forEach(x=>x.classList.toggle('on',x===b));startSortStream();});
const sortField=(label,el,full)=>h('label',{class:full?'full':''},label,el);
function buildSortConfig(c){sortBuilt=true;const m=c.meta;
  c.patterns.forEach((p,i)=>{const num=(f,v,step,min,max)=>h('input',{type:'number','data-f':f,value:v,step,min,max});
    $('#sort-pat-'+i).replaceChildren(h('div',{class:'panel-head'},h('h3',{style:`color:${m.pattern_colors[i]}`},`Drop-off pattern ${m.letters[i]}`),h('span',{class:'small',id:`sort-pat-status-${i}`})),
      h('div',{class:'cfg','data-pattern':i},sortField('Name',h('input',{'data-f':'name',value:p.name})),
        sortField('Dictionary',h('select',{'data-f':'dict'},...m.dictionaries.map(d=>h('option',{value:d,selected:d===p.dict},d)))),
        sortField('Marker ID (first ID of a grid)',num('id',p.id,1,0)),sortField('Marker size (mm, black square)',num('size',p.size,1,5)),
        sortField('Rows (1 × 1 = single marker)',num('rows',p.rows,1,1)),sortField('Columns',num('cols',p.cols,1,1)),sortField('Separation (mm)',num('sep',p.sep,1,0)),
        sortField(`Release height above pattern (cm, ${m.release_cm[0]}–${m.release_cm[1]})`,num('height',p.height,0.5,m.release_cm[0],m.release_cm[1])),
        h('div',{class:'full muted small',id:`sort-pat-centre-${i}`})));});
  c.objects.forEach((o,i)=>$('#sort-obj-'+i).replaceChildren(h('div',{class:'panel-head'},h('h3',{style:`color:${m.object_colors[i]}`},`Object ${i+1}`),h('span',{class:'small muted',id:`sort-obj-scene-${i}`})),
    h('div',{class:'cfg','data-object':i},sortField('Recognition prompt',h('input',{'data-f':'prompt',value:o.prompt,placeholder:'e.g. cube, soda can, tennis ball'}),true),
      sortField('Colour filter',h('select',{'data-f':'color'},...m.colors.map(cn=>h('option',{value:cn,selected:cn===o.color},cn)))),
      sortField('Drop-off spot',h('select',{'data-f':'zone'},...m.letters.map((l,k)=>h('option',{value:k,selected:k===o.zone},'Pattern '+l)))),
      sortField('Mass (kg)',h('input',{type:'number','data-f':'mass',value:o.mass,step:0.05,min:0,max:m.max_mass})),
      sortField('How many',h('input',{type:'number','data-f':'count',value:o.count,step:1,min:1,max:m.max_items})))));
  $$('#tab-sort .cfg [data-f]').forEach(el=>el.addEventListener(el.tagName==='SELECT'?'change':'input',queueSortConfig));
  const o=c.options,send=sendParams('/api/sort/options');
  $('#sort-options').replaceChildren(paramRow('max joint step rad',o.max_joint_step_rad,0.005,m.max_step,0.005,v=>send('max_joint_step_rad',v)),
    paramRow('max joint speed rad/s',o.max_joint_speed_rad_s,0.02,m.max_speed,0.01,v=>send('max_joint_speed_rad_s',v)),
    h('div',{class:'small',style:'margin-top:8px'},'Correct the grab target from live detections during the approach'),checkRow('dynamic target',o.dynamic_target_correction,v=>send('dynamic_target_correction',v)),
    h('div',{class:'small'},'Turn the waist to centre each object before grabbing'),checkRow('centre waist',o.center_waist,v=>send('center_waist',v)),
    h('div',{class:'small'},'Carry objects even when tactile sensors don\'t confirm the grasp'),checkRow('unverified grasps',o.allow_unverified_grasp,v=>send('allow_unverified_grasp',v)));
  renderSortConfig(c);}
function collectSortConfig(){const read=card=>Object.fromEntries($$('[data-f]',card).map(el=>[el.dataset.f,el.type==='number'?(el.value===''?null:+el.value):el.value]));
  return{patterns:$$('#tab-sort [data-pattern]').map(read),objects:$$('#tab-sort [data-object]').map(card=>{const o=read(card);o.zone=+o.zone;return o;})};}
function queueSortConfig(){clearTimeout(sortCfgTimer);sortCfgTimer=setTimeout(async()=>{try{renderSortConfig(await api('/api/sort/config',collectSortConfig()));}catch(e){toast(e.message,'err');}},500);}
function renderSortConfig(c){const root=$('#sort-config-status');root.innerHTML='';
  if(c.running){root.append(h('div',{style:'color:var(--warn)'},c.message));return;}
  if((c.errors||[]).length)c.errors.forEach(e=>root.append(h('div',{style:'color:var(--err)'},'✕ '+e)));
  else(c.lines||[]).forEach(l=>root.append(h('div',{style:'color:var(--ok)'},l)));}
function sortUnavailable(msg){$('#sort-phase').textContent=msg;$('#sort-ph').textContent=msg;$('#sort-ph').classList.remove('hidden');$('#sort-start').disabled=true;}
async function pollSort(){let s;try{s=await api('/api/sort');}catch(e){sortUnavailable(e.message);return;}
  if(!sortBuilt){try{buildSortConfig(await api('/api/sort/config'));}catch(e){toast(e.message,'err');}}
  $('#sort-badges').innerHTML=(s.running?'<span class="badge warn">sorting</span>':'<span class="badge">idle</span>')+(s.arm_busy?`<span class="badge warn">${s.arm_busy}</span>`:'')+(s.holding?'<span class="badge ok">holding object</span>':'');
  $('#sort-start').disabled=s.sort_disabled;
  $('#sort-phase').replaceChildren(h('div',{style:'color:var(--warn)'},s.phase+(s.running?'  ·  '+s.scene.map(o=>`${o.name} ${o.sorted}/${o.count}`).join(', '):'')));
  const sc=$('#sort-scene');sc.innerHTML='';
  s.scene.forEach(o=>{sc.append(h('div',{class:'li'},h('span',{class:'badge'},String(o.object)),h('span',{class:'grow'},`${o.name} → ${o.zone}`),h('span',{class:'small'},`${o.todo} to sort · ${o.done} on its spot`)));
    const el=$('#sort-obj-scene-'+(o.object-1));if(el)el.textContent=`${o.todo} to sort · ${o.done} on its spot`;});
  if(s.ignored.length)sc.append(h('div',{class:'muted small'},'Ignored (colour filter / no match): '+s.ignored.join(', ')));
  s.patterns.forEach((p,i)=>{const st=$('#sort-pat-status-'+i);if(st){st.textContent=p.status;st.style.color=p.visible?'var(--ok)':'var(--warn)';}
    const ce=$('#sort-pat-centre-'+i);if(ce)ce.textContent=p.describe+(p.centre?` · centre base xyz (${p.centre.map(v=>fmt(v,3)).join(', ')}) m`:'');});
  $('#sort-tactile').textContent=s.tactile;
  const kv=$('#sort-kv');kv.innerHTML='';[['status',s.status_msg],['camera',s.camera],['vision',`${s.vision} · ${s.classes.join(', ')}`],['obstacles',s.obstacles]].forEach(([a,b])=>kv.append(h('div',{},a),h('div',{class:'mono'},b)));
  if(!sortImg.naturalWidth)$('#sort-ph').textContent=s.camera;
  const lg=$('#sort-log'),atBottom=lg.scrollHeight-lg.scrollTop-lg.clientHeight<40;lg.textContent=s.log.join('\n');if(atBottom)lg.scrollTop=lg.scrollHeight;}
every('sort',700,pollSort);
async function sortAction(action){try{const j=await api('/api/sort/action',{action});toast(j.message,'ok');pollSort();}catch(e){toast(e.message,'err');}}
$('#sort-start').onclick=async()=>{if(!await confirmDialog('Start sorting?','The right arm picks each configured object, lifts it to the WBC stable pose and places it on its ArUco spot. Keep the workspace clear and stay near the robot.','Sort'))return;sortAction('sort');};
const SORT_CONFIRM={release_arms:'The arms go limp over 3 s and drop anything the hand holds.',damp:'The robot enters damping: all joints go passive. Support the robot first.'};
$$('#tab-sort [data-sort]').forEach(b=>b.onclick=async()=>{const a=b.dataset.sort;if(SORT_CONFIRM[a]&&!await confirmDialog(b.textContent+'?',SORT_CONFIRM[a]))return;sortAction(a);});
hooks.sort={enter:()=>{startSortStream();pollSort();},leave:stopSortStream};

/* ---------- boot ---------- */
(async function boot(){for(let i=0;;i++){try{META=await api('/api/meta');break;}catch(e){$('#sub').textContent='waiting for server…';await new Promise(r=>setTimeout(r,1000));}}
  applyTheme(META.theme);$('#sub').textContent=`${META.hostname} · ${META.off_robot?'off-robot simulator':META.offline?'offline':'live robot'}`;buildTabs();renderWifiActions();showTab(location.hash.slice(1)||'chat');})();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    raise SystemExit(main())
