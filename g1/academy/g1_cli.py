#!/usr/bin/env python3
"""Full-screen G1 console backed by sdk_wrapper and the OpenAI intent agent.

Run from academy/: ``python3 g1_cli.py``.  Live robot mode is the default;
``--offline`` starts the agent without DDS or robot actions.

sdk_wrapper.G1 is the only robot interface; the agent's skills, state, SLAM
and vision grab are bound to it by g1_agent.build_agent().
"""
from __future__ import annotations

import argparse
import concurrent.futures
import curses
import json
import os
import platform
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
import textwrap
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
G1_ROOT = HERE.parent
# Only the robot-independent intent agent (openaiapi/agent) and its LLM
# transport (llm_client) come from modules/scripts.  modules/ itself -- the
# legacy sdk_client stack -- is deliberately not on sys.path: sdk_wrapper is
# the only robot interface, and g1_agent binds the agent's skills to it.
SCRIPTS = G1_ROOT / "modules" / "scripts"
AGENT_PACKAGE = SCRIPTS / "openaiapi"
for path in (str(SCRIPTS), str(AGENT_PACKAGE), str(HERE)):
    if path not in sys.path:
        sys.path.insert(0, path)

import sdk_wrapper  # noqa: E402  (sets up the CycloneDDS environment itself)
from agent.planner import OpenAIPlanner  # noqa: E402


COMMANDS = {
    "/help": "show commands",
    "/chat": "switch to natural language agent mode",
    "/settings": "configure robot, AI, and voice defaults",
    "/topics": "show wrapper DDS topics",
    "/services": "show robot services and status",
    "/state": "show current robot state",
    "/wifi": "manage wlan0 supplicant and DHCP",
    "/jetson": "show Jetson system information",
    "/control": "keyboard robot and end-effector control",
    "/mode": "select a robot FSM mode",
    "/theme": "change terminal colors",
    "/clear": "clear transcript",
    "/exit": "quit",
}

SETTINGS_PATH = HERE / "g1_cli_settings.json"
DEFAULT_SETTINGS = {
    "locomotion_speed": 0.2,
    "locomotion_duration": 1.0,
    "rotation_speed": 0.2,
    "rotation_duration": 1.0,
    "voice_reply": False,
    "robot_volume": 50,
    "voice_language": "en",
    "ai_language": "en",
    "theme": "amber",
    "waist_locked": False,
    "ramping": True,
    "ll_max_joint_speed": 0.45,
    "ll_kp": 30.0,
    "ll_kd": 1.5,
    "ik_step_m": 0.03,
    "ik_rotation_rad": 0.08,
    "tactile_threshold": 103000.0,
    "climb_fsm_id": int(os.environ.get("G1_CLIMB_FSM_ID", "812")),
}

WRAPPER_TOPICS = sorted(set(
    ["rt/lowstate", "rt/odom", "rt/odommodestate", "rt/sportmodestate", "rt/audio_msg",
     "rt/slam_info", "rt/slam_key_info", "rt/arm_sdk", "rt/lowcmd"]
    + list(sdk_wrapper.BMS_TOPICS)
    + list(sdk_wrapper.SLAM_POINT_TOPICS)
    + list(sdk_wrapper.HAND_CMD_TOPICS.values())
    + list(sdk_wrapper.HAND_STATE_TOPICS.values())
))


def _make_agent(args: argparse.Namespace, robot: Any = None):
    """The intent agent on sdk_wrapper.G1 (live) or offline; see g1_agent."""
    import g1_agent

    return g1_agent.build_agent(args, robot)


class G1Console:
    def __init__(self, robot: Any, agent: Any, offline: bool = False):
        self.robot = robot
        self.agent = agent
        self.offline = offline
        self.mode = "chat"
        self.lines: list[tuple[str, str]] = [
            ("system", "G1 console  ·  /help for commands  ·  Ctrl-C or /exit to quit"),
            ("system", "Natural language requests are routed by the OpenAI intent agent."),
        ]
        self.input = ""
        self.input_history: list[str] = []
        self.history_index: int | None = None
        self.history_draft = ""
        self.completion_matches: list[str] = []
        self.completion_index = 0
        self.completion_prefix = ""
        self.scroll_offset = 0
        self.running = True
        self.screen: Any = None
        self.arms_released = False
        self.dev_mode = False
        self.chat_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-chat")
        self.action_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="g1-action")
        self.menu_action_future: concurrent.futures.Future[Any] | None = None
        self.menu_action_label = ""
        self.chat_future: concurrent.futures.Future[Any] | None = None
        self.confirm_queue: queue.Queue[Any] = queue.Queue()
        self.pending_confirmation: Any = None
        self.spinner_index = 0
        self.theme = "amber"
        self._topic_readers: dict[str, Any] = {}
        self.settings = dict(DEFAULT_SETTINGS)
        try:
            saved = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                self.settings.update({k: v for k, v in saved.items() if k in DEFAULT_SETTINGS})
        except (OSError, json.JSONDecodeError):
            pass
        self.theme = self.settings.get("theme", "amber")
        if self.theme not in ("dark", "light", "ocean", "amber"):
            self.theme = "amber"
        try:
            self.agent.settings.set("interface.reply_language", self.settings["ai_language"], persist=False)
            self.agent.settings.set("announcements.audio_enabled", bool(self.settings["voice_reply"]), persist=False)
        except Exception:
            pass
        if not offline and hasattr(agent, "_confirm_prompt"):
            agent._confirm_prompt = self.confirm_prompt

    def confirm_prompt(self, skill_name: str, policy: Any) -> bool:
        """Keep the agent's per-skill confirmation inside the TUI."""
        if self.screen is None:
            return False
        event = threading.Event()
        answer = {"approved": False}
        self.confirm_queue.put((skill_name, getattr(policy, "reason", ""), event, answer))
        event.wait()
        return bool(answer["approved"])

    def say(self, role: str, message: Any) -> None:
        if not isinstance(message, str):
            try:
                message = json.dumps(message, indent=2, ensure_ascii=False, default=str)
            except Exception:
                message = str(message)
        self.lines.extend((role, line) for line in str(message).splitlines() or [""])

    def execute(self, raw: str) -> None:
        text = raw.strip()
        if not text:
            return
        self.say("you", text)
        if self.chat_future is not None and not self.chat_future.done():
            self.say("error", "The current agent turn is still running; wait for it to finish before starting another command.")
            return
        if text.startswith("/"):
            self.command(text)
            return
        self.mode = "chat"
        self.chat_future = self.chat_executor.submit(self._chat_turn, text)

    def _chat_turn(self, text: str) -> Any:
        # The agent has its own announcement audio setting, separate from the
        # CLI's optional reply voice. Keep both speech paths in sync per turn.
        try:
            self.agent.settings.set("announcements.audio_enabled", bool(self.settings["voice_reply"]), persist=False)
        except Exception:
            pass
        outcome = self.agent.handle_cli_text(text)
        decision = getattr(outcome, "decision", None)
        reply = getattr(decision, "response_text", None) if decision else None
        if reply and self.settings["voice_reply"] and self.robot is not None:
            try:
                self.robot.say(reply, language=self.settings["voice_language"],
                               volume=int(self.settings["robot_volume"]))
            except Exception as exc:
                self.say("error", f"Voice reply failed: {exc}")
        return outcome

    def _poll_background(self) -> None:
        try:
            if self.pending_confirmation is None:
                self.pending_confirmation = self.confirm_queue.get_nowait()
        except queue.Empty:
            pass
        if self.chat_future is not None and self.chat_future.done():
            try:
                outcome = self.chat_future.result()
                decision = getattr(outcome, "decision", None)
                reply = getattr(decision, "response_text", None) if decision else None
                if reply:
                    self.say("agent", reply)
                for name, result in getattr(outcome, "skill_outcomes", []) or []:
                    self.say("tool", f"{name}: {getattr(result, 'message', result)}")
                if not reply and not getattr(outcome, "skill_outcomes", []):
                    self.say("agent", "Request processed.")
            except Exception as exc:
                self.say("error", f"Agent error: {type(exc).__name__}: {exc}")
            self.chat_future = None

    def command(self, raw: str) -> None:
        parts = raw.split(maxsplit=2)
        cmd = parts[0].lower()
        arg = parts[1:]
        try:
            if cmd in ("/exit", "/quit"):
                self.running = False
            elif cmd in ("/help", "/?"):
                self.say("system", "\n".join(f"{name:<25} {desc}" for name, desc in COMMANDS.items()))
                self.say("system", "Bare text is sent to /chat. Menus: ↑/↓ select, ←/→ adjust, Enter confirm, Esc back.")
            elif cmd == "/chat":
                self.mode = "chat"
                self.say("system", "Chat mode. Enter a request in plain language.")
            elif cmd == "/clear":
                self.lines = []
            elif cmd in ("/topics", "/topic"):
                self.open_topics_menu()
            elif cmd in ("/services", "/service"):
                if self.offline:
                    self.say("system", "Services are unavailable in offline mode.")
                else:
                    self.open_services_menu()
            elif cmd == "/state":
                if self.offline:
                    self.say("system", "Robot state is unavailable in offline mode.")
                else:
                    self.open_info_view("Robot state · r refresh", self.show_state)
            elif cmd == "/settings":
                self.open_settings_menu()
            elif cmd == "/wifi":
                self.open_wifi_menu()
            elif cmd == "/jetson":
                self.open_info_view("Jetson board · r refresh", self.show_jetson)
            elif cmd == "/control":
                self.open_control()
            elif cmd == "/mode":
                self.open_mode_menu()
            elif cmd == "/theme":
                self.open_theme_menu()
            else:
                self.say("error", f"Unknown command: {cmd}. Try /help.")
        except Exception as exc:
            self.say("error", f"{cmd}: {exc}")

    def _persist_settings(self) -> None:
        SETTINGS_PATH.write_text(json.dumps(self.settings, indent=2) + "\n", encoding="utf-8")
        try:
            self.agent.settings.set("interface.reply_language", self.settings["ai_language"], persist=False)
        except Exception:
            pass

    @staticmethod
    def _bar(value: Any, maximum: float = 100.0, width: int = 24) -> str:
        try:
            ratio = max(0.0, min(1.0, float(value) / float(maximum)))
        except (TypeError, ValueError, ZeroDivisionError):
            return "[ unavailable ]"
        filled = int(round(ratio * width))
        return "[" + "█" * filled + "░" * (width - filled) + "]"

    def _table(self, title: str, rows: list[tuple[str, Any]]) -> None:
        key_width = min(24, max((len(k) for k, _ in rows), default=8))
        self.say("system", title)
        self.say("system", "─" * min(72, max(key_width + 30, 48)))
        for key, value in rows:
            self.say("system", f"{key:<{key_width}}  {value}")

    def show_services(self) -> None:
        try:
            client = self.robot._robot_state_client()
            client.SetTimeout(5.0)
            rows = self.robot.get_service()
            table = [("Service", "Status / protection / description")]
            for row in rows:
                status = row.get("status")
                label = {0: "ON", 1: "OFF", 5: "PROTECTED"}.get(status, "UNKNOWN")
                protected = "yes" if row.get("protected") is True else "no" if row.get("protected") is False else "?"
                table.append((row["name"], f"{label:<10} protected={protected:<3} {row.get('description') or ''}"))
            self._table("Robot services", table)
        except Exception as exc:
            self.say("error", f"Service list failed ({type(exc).__name__}): {exc}")

    def show_state(self) -> None:
        rows: list[tuple[str, Any]] = []
        for label, getter in (("Battery", self.robot.get_battery), ("Motion", self.robot.get_odom),
                              ("IMU", self.robot.get_imus), ("SLAM pose", self.robot.get_slam_pose),
                              ("Low state", self.robot.get_lowstate)):
            try:
                data = getter()
                if label == "Battery":
                    bms = (data or {}).get("bms") or {}
                    soc = bms.get("soc")
                    rows.append(("Battery", f"{soc}% {self._bar(soc)}" if soc is not None else "telemetry not received"))
                    rows.append(("Battery source", (data or {}).get("source", "lowstate")))
                    if bms.get("current") is not None:
                        rows.append(("Battery current", f"{bms['current']}"))
                elif label == "Motion":
                    if data is None:
                        rows.append(("Odometry", "telemetry not received"))
                    else:
                        rows.append(("Odometry topic", data.get("topic", "sport mode")))
                        rows.append(("Position", data.get("position", data.get("pose", "unavailable"))))
                        rows.append(("Velocity", data.get("velocity", "unavailable")))
                        rows.append(("Gait", data.get("gait", "unavailable")))
                elif label == "IMU":
                    rows.append(("IMU RPY", (data or {}).get("rpy", "telemetry not received")))
                    rows.append(("IMU gyro", (data or {}).get("gyro", "unavailable")))
                    rows.append(("IMU temp", (data or {}).get("temp", "unavailable")))
                elif label == "SLAM pose":
                    rows.append(("SLAM pose", data if data is not None else "not available"))
                else:
                    motors = (data or {}).get("joint_positions") or []
                    rows.append(("Low state", f"{len(motors)} joints" if motors else "telemetry not received"))
                    if motors:
                        rows.append(("Joint range", f"{min(motors):.2f} to {max(motors):.2f} rad"))
            except Exception as exc:
                rows.append((label, f"unavailable ({type(exc).__name__}: {exc})"))
        try:
            fsm_id = sdk_wrapper._rpc_get_int(self.robot._client, sdk_wrapper.ROBOT_API_ID_LOCO_GET_FSM_ID)
            fsm_mode = sdk_wrapper._rpc_get_int(self.robot._client, sdk_wrapper.ROBOT_API_ID_LOCO_GET_FSM_MODE)
            rows[:0] = [("FSM id", fsm_id if fsm_id is not None else "unavailable"),
                        ("FSM mode", fsm_mode if fsm_mode is not None else "unavailable")]
        except Exception as exc:
            rows.insert(0, ("FSM", f"unavailable ({exc})"))
        try:
            motion_code, motion_raw = self.robot._motion_client().CheckMode()
            rows.insert(0, ("Motion service", f"{sdk_wrapper._mode_name(motion_raw) or 'unknown'} (code {motion_code})"))
        except Exception as exc:
            rows.insert(0, ("Motion service", f"unavailable ({exc})"))
        self._table("Robot state", rows)

    def show_topics(self) -> None:
        try:
            result = subprocess.run(["ros2", "topic", "list", "-t"], capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                raise RuntimeError((result.stderr or result.stdout).strip() or f"ros2 exited {result.returncode}")
            live: dict[str, str] = {}
            for raw in result.stdout.splitlines():
                topic, _, type_text = raw.strip().partition("[")
                topic = topic.strip().lstrip("/")
                if topic in WRAPPER_TOPICS:
                    live[topic] = type_text.rstrip("]") if type_text else "type unknown"
            rows = [(topic, live.get(topic, "configured · no live publisher discovered")) for topic in WRAPPER_TOPICS]
            self._table("sdk_wrapper DDS topics", rows)
        except Exception as exc:
            self.say("error", f"Live topic discovery failed: {exc}")
            self._table("sdk_wrapper DDS topics (configured)", [(topic, "") for topic in WRAPPER_TOPICS])

    def _topic_reader(self, topic: str) -> Any:
        if topic in self._topic_readers:
            return self._topic_readers[topic]
        if self.robot is None:
            return None
        reader = None
        if topic == "rt/lowstate":
            reader = self.robot._lowstate
        elif topic == "rt/odom":
            reader = self.robot._odom
        elif topic == "rt/odommodestate":
            reader = self.robot._sport
        elif topic == "rt/sportmodestate":
            reader = self.robot._sport_alt
        elif topic == "rt/audio_msg":
            reader = self.robot._audio_msg
        elif topic == "rt/slam_info":
            reader = self.robot._slam_info
        elif topic == "rt/slam_key_info":
            reader = self.robot._slam_key
        else:
            for name, sub in self.robot._bms:
                if name == topic:
                    reader = sub
                    break
            if reader is None:
                for name, sub in self.robot._clouds:
                    if name == topic:
                        reader = sub
                        break
            if reader is None:
                for side, state_topic in sdk_wrapper.HAND_STATE_TOPICS.items():
                    if state_topic == topic:
                        reader = self.robot._dex3_hand(side).state
                        break
        if reader is None:
            if topic in ("rt/lowcmd", "rt/arm_sdk"):
                reader = self.robot._latest(topic, sdk_wrapper.LowCmd_)
            elif topic.endswith("/cmd"):
                reader = self.robot._latest(topic, sdk_wrapper.HandCmd_)
            else:
                return None
        self._topic_readers[topic] = reader
        return reader

    def _topic_summary(self, topic: str, message: Any) -> list[str]:
        if message is None:
            return ["Waiting for a DDS message from this topic."]
        if topic == "rt/lowstate":
            data = self.robot.get_lowstate() or {}
            joints = data.get("joint_positions") or []
            lines = [f"Joint states: {len(joints)}", f"IMU: {data.get('imu') or 'unavailable'}"]
            if joints:
                lines.append(f"Joint position range: {min(joints):.3f} … {max(joints):.3f} rad")
            return lines
        if topic in sdk_wrapper.BMS_TOPICS:
            names = ("status", "soc", "soh", "current", "cycle")
            return [f"{name}: {getattr(message, name, 'unavailable')}" for name in names]
        if "points" in topic or topic.endswith("global_map"):
            data = getattr(message, "data", b"")
            return [f"Dimensions: {getattr(message, 'width', '?')} × {getattr(message, 'height', '?')}",
                    f"Point step: {getattr(message, 'point_step', '?')} bytes",
                    f"Payload: {len(data)} bytes"]
        if topic in ("rt/lowcmd", "rt/arm_sdk"):
            motors = list(getattr(message, "motor_cmd", []) or [])
            active = [(idx, motor) for idx, motor in enumerate(motors) if float(getattr(motor, "kp", 0.0)) or float(getattr(motor, "kd", 0.0))]
            lines = [f"Motor command slots: {len(motors)}", f"Active commands: {len(active)}"]
            lines.extend(f"Joint {idx:02d}  q={getattr(motor, 'q', 0.0): .3f}  kp={getattr(motor, 'kp', 0.0):g}  kd={getattr(motor, 'kd', 0.0):g}" for idx, motor in active[:12])
            return lines
        if topic.endswith("/state") or topic.endswith("/cmd"):
            motors = list(getattr(message, "motor_state", None) or getattr(message, "motor_cmd", None) or [])
            if motors:
                return [f"Motor entries: {len(motors)}"] + [f"Joint {idx}: q={getattr(motor, 'q', '?')}" for idx, motor in enumerate(motors[:12])]
        if topic in ("rt/odom", "rt/odommodestate", "rt/sportmodestate"):
            data = self.robot.get_odom() or {}
            return [f"Position: {data.get('position', data.get('pose', 'unavailable'))}",
                    f"Velocity: {data.get('velocity', 'unavailable')}",
                    f"Gait: {data.get('gait', 'unavailable')}"]
        value = getattr(message, "data", None)
        if value is not None:
            value = str(value)
            return textwrap.wrap(value, width=100)[:20] or ["(empty message)"]
        fields = getattr(message, "__slots__", None) or getattr(message, "__dict__", {})
        if isinstance(fields, dict):
            return [f"{key}: {getattr(message, key, '?')}" for key in list(fields)[:20]]
        return [f"Message type: {type(message).__name__}", str(message)[:1500]]

    def view_topic(self, topic: str) -> None:
        screen = self.screen
        if screen is None:
            return
        reader = self._topic_reader(topic)
        screen.timeout(250)
        try:
            while True:
                screen.erase()
                height, width = screen.getmaxyx()
                screen.attron(curses.color_pair(6) | curses.A_BOLD)
                screen.addnstr(0, 0, f" Live DDS topic · {topic} ".ljust(width - 1), width - 1)
                screen.attroff(curses.color_pair(6) | curses.A_BOLD)
                now = time.time()
                if reader is None:
                    lines = ["No sdk_wrapper subscriber is defined for this topic.",
                             "It is listed for SDK completeness but has no readable data stream."]
                    stamp = None
                else:
                    message, stamp = reader.get()
                    lines = self._topic_summary(topic, message)
                age = "waiting" if not stamp else f"last received {max(0.0, now - stamp):.2f}s ago"
                screen.addnstr(1, 2, f"Type: {type(message).__name__ if reader is not None and message is not None else 'unknown'} · {age}", width - 4)
                for idx, line in enumerate(lines[:max(1, height - 5)], start=3):
                    screen.addnstr(idx, 2, line, width - 4)
                screen.addnstr(height - 1, 0, "Live refresh · Esc/q return to topic list".ljust(width - 1), width - 1)
                screen.refresh()
                key = screen.getch()
                if key in (27, ord("q"), ord("Q")):
                    return
        finally:
            screen.timeout(100)

    def open_topics_menu(self) -> None:
        if self.offline:
            self.say("system", "Topic live data requires live robot mode.")
            return
        def choose(index: int) -> None:
            self.view_topic(WRAPPER_TOPICS[index])
        self.open_menu("sdk_wrapper topics · select to inspect live data", WRAPPER_TOPICS, choose,
                       "↑/↓ select · Enter live view · Esc back", stay_open=True)

    def open_services_menu(self) -> None:
        # Populate from ServiceList on every open/refresh, as service_view does.
        # Never fabricate rows when the robot is unreachable: doing so makes an
        # action's menu index drift away from the service the user selected.
        rows: list[dict[str, Any]] = []
        last_error = "Loading service list…"

        def refresh() -> str:
            nonlocal last_error
            try:
                found = self.robot.get_service()
                rows[:] = found
                ai_sport = next((item for item in found if item.get("name") == "ai_sport"), None)
                if ai_sport is not None and ai_sport.get("status") in (0, 1):
                    self.dev_mode = int(ai_sport["status"]) == 1
                last_error = f"{len(found)} services reported by robot."
            except Exception as exc:
                rows.clear()
                last_error = f"ServiceList failed ({type(exc).__name__}): {exc}. No service list loaded."
                if "3104" in last_error:
                    last_error += " RPC timeout; check robot service availability."
            return last_error

        def entries() -> list[str]:
            lines = ["↻ Refresh service status"]
            for row in rows:
                status = {0: "ON", 1: "OFF", 5: "PROTECTED"}.get(row.get("status"), "UNKNOWN")
                lines.append(f"Enable {row['name']:<22} [{status}]")
                lines.append(f"Disable {row['name']:<21} [{status}]")
            lines.append(last_error)
            return lines

        def choose(index: int) -> Any:
            nonlocal last_error
            if index >= 1 + len(rows) * 2:
                return last_error
            if index == 0:
                return refresh()
            pair = index - 1
            row = rows[pair // 2]
            enabled = (pair % 2) == 0
            # Match visualizations/service_view.py: sdk_wrapper.G1.set_service
            # first reads the live ServiceList, then calls ServiceSwitch.
            result = self.robot.set_service(row["name"], bool(enabled))
            code = int(result["code"])
            if code != 0:
                last_error = f"{row['name']} ServiceSwitch returned RPC {code}; status was not assumed to change."
                return last_error
            target_status = 0 if enabled else 1
            verified = None
            for attempt in range(3):
                if attempt:
                    time.sleep(0.25)
                try:
                    updated = self.robot.get_service()
                    current = next((item for item in updated if item.get("name") == row["name"]), None)
                    if current is not None:
                        row.update(current)
                        verified = current.get("status")
                    if verified == target_status:
                        break
                except Exception:
                    pass
            if row["name"] == "ai_sport" and verified in (0, 1):
                self.dev_mode = int(verified) == 1
            if verified == target_status:
                last_error = f"{row['name']} is {'ON' if enabled else 'OFF'} (verified)."
            elif verified in (0, 1):
                last_error = f"{row['name']} is still {'ON' if verified == 0 else 'OFF'} after the switch request."
            else:
                row["status"] = None
                last_error = f"{row['name']} switch returned success; refreshed status could not be verified."
            return last_error

        if self.menu_action_future is None or self.menu_action_future.done():
            self.menu_action_label = "Loading service list"
            self.menu_action_future = self.action_executor.submit(refresh)
        self.open_menu("Services · choose explicit ON/OFF action", entries, choose,
                       "↑/↓ select · Enter run action · Esc back", stay_open=True, async_actions=True)

    def open_info_view(self, title: str, refresh_fn: Any) -> None:
        screen = self.screen
        if screen is None:
            refresh_fn()
            return
        body: list[str] = []
        def refresh() -> None:
            start = len(self.lines)
            refresh_fn()
            body[:] = [line for _role, line in self.lines[start:]]
            del self.lines[start:]
        refresh()
        screen.timeout(100)
        try:
            while True:
                screen.erase()
                height, width = screen.getmaxyx()
                screen.attron(curses.color_pair(6) | curses.A_BOLD)
                screen.addnstr(0, 0, f" {title} ".ljust(width - 1), width - 1)
                screen.attroff(curses.color_pair(6) | curses.A_BOLD)
                for idx, line in enumerate(body[:max(1, height - 4)], start=2):
                    screen.addnstr(idx, 1, line, width - 2)
                screen.addnstr(height - 1, 0, "r refresh · Esc/q back".ljust(width - 1), width - 1)
                screen.refresh()
                key = screen.getch()
                if key in (27, ord("q"), ord("Q")):
                    return
                if key == ord("r"):
                    refresh()
        finally:
            screen.timeout(100)

    def open_theme_menu(self) -> None:
        themes = ["dark", "light", "ocean", "amber"]
        def choose(index: int) -> None:
            self.theme = themes[index]
            self.settings["theme"] = self.theme
            self._persist_settings()
            self._init_colors()
            self.say("system", f"Theme set to {self.theme}.")
        self.open_menu("Theme", themes, choose, "↑/↓ select · Enter apply · Esc back", stay_open=True)

    def _init_colors(self) -> None:
        fg_bg = {"dark": (curses.COLOR_WHITE, curses.COLOR_BLACK),
                 "light": (curses.COLOR_BLACK, curses.COLOR_WHITE),
                 "ocean": (curses.COLOR_WHITE, curses.COLOR_BLUE),
                 "amber": (curses.COLOR_YELLOW, curses.COLOR_BLACK)}
        foreground, background = fg_bg[self.theme]
        accent = curses.COLOR_CYAN if self.theme == "ocean" else curses.COLOR_YELLOW if self.theme == "amber" else curses.COLOR_MAGENTA
        try:
            curses.init_pair(1, foreground, background)
            curses.init_pair(2, curses.COLOR_CYAN if self.theme != "amber" else curses.COLOR_YELLOW, background)
            curses.init_pair(3, curses.COLOR_GREEN, background)
            curses.init_pair(4, curses.COLOR_RED, background)
            curses.init_pair(5, curses.COLOR_YELLOW, background)
            curses.init_pair(6, accent, background)
        except curses.error:
            pass

    def open_menu(self, title: str, entries: Any, on_select: Any,
                  hint: str = "↑/↓ select · Enter choose · Esc back", stay_open: bool = False,
                  async_actions: bool = False) -> None:
        screen = self.screen
        if screen is None:
            self.say("error", f"{title} menu requires the full-screen TUI.")
            return
        selected = 0
        previous_entries: list[str] | None = None
        while True:
            if self.menu_action_future is not None and self.menu_action_future.done():
                future, label = self.menu_action_future, self.menu_action_label
                self.menu_action_future = None
                self.menu_action_label = ""
                try:
                    result = future.result()
                    if result:
                        self.say("system", str(result))
                except Exception as exc:
                    self.say("error", f"{label}: {type(exc).__name__}: {exc}")
            current_entries = entries() if callable(entries) else entries
            if not current_entries:
                current_entries = ["(no entries)"]
            if previous_entries is not None and current_entries != previous_entries:
                # Async refreshes can replace or reorder rows. Reset selection
                # so Enter cannot target a different service than expected.
                selected = 0
            previous_entries = list(current_entries)
            selected = min(selected, len(current_entries) - 1)
            screen.erase()
            height, width = screen.getmaxyx()
            screen.attron(curses.color_pair(6) | curses.A_BOLD)
            screen.addnstr(0, 0, f" {title} ".ljust(width - 1), width - 1)
            screen.attroff(curses.color_pair(6) | curses.A_BOLD)
            start = max(0, min(selected - max(0, height - 8), len(current_entries) - max(1, height - 5)))
            for idx in range(start, min(len(current_entries), start + max(1, height - 5))):
                marker = "› " if idx == selected else "  "
                attr = curses.color_pair(2) | curses.A_BOLD if idx == selected else curses.A_NORMAL
                screen.addnstr(2 + idx - start, 1, marker + current_entries[idx], max(1, width - 3), attr)
            screen.attron(curses.color_pair(6))
            footer = hint
            if self.menu_action_future is not None:
                spinner = "|/-\\"[self.spinner_index % 4]
                self.spinner_index += 1
                footer = f"{spinner} Waiting: {self.menu_action_label}… please wait"
            screen.addnstr(height - 1, 0, footer.ljust(width - 1), width - 1)
            screen.attroff(curses.color_pair(6))
            screen.refresh()
            key = screen.getch()
            if key in (27, ord("q"), ord("Q")):
                self.draw(screen)
                return
            if key in (curses.KEY_UP, ord("k")):
                selected = (selected - 1) % len(current_entries)
            elif key in (curses.KEY_DOWN, ord("j")):
                selected = (selected + 1) % len(current_entries)
            elif key in (10, 13, curses.KEY_ENTER, curses.KEY_RIGHT):
                if self.menu_action_future is not None:
                    continue
                try:
                    if async_actions:
                        self.menu_action_label = current_entries[selected]
                        self.menu_action_future = self.action_executor.submit(on_select, selected)
                    else:
                        on_select(selected)
                except Exception as exc:
                    self.say("error", f"{title}: {type(exc).__name__}: {exc}")
                if stay_open:
                    continue
                self.draw(screen)
                return

    def open_settings_menu(self) -> None:
        fields = [
            ("locomotion_speed", "Locomotion speed (m/s)", None, 0.05, 0.05, 0.8),
            ("locomotion_duration", "Locomotion nudge duration (s)", None, 0.1, 0.1, 3.0),
            ("rotation_speed", "Rotation speed (rad/s)", None, 0.02, 0.02, 0.6),
            ("rotation_duration", "Rotation nudge duration (s)", None, 0.1, 0.1, 3.0),
            ("voice_reply", "Speak agent replies", None, None, None, None),
            ("robot_volume", "Robot speech volume (%)", None, 5.0, 0.0, 100.0),
            ("voice_language", "Voice language", ["en", "de", "fr", "es", "ar"], None, None, None),
            ("ai_language", "AI reply language", ["en", "de", "auto"], None, None, None),
            ("waist_locked", "Walk FSM uses locked waist", None, None, None, None),
            ("ramping", "Low-level motion ramping", None, None, None, None),
            ("ll_max_joint_speed", "Maximum joint speed (rad/s)", None, 0.05, 0.05, 1.5),
            ("ll_kp", "Low-level Kp", None, 1.0, 0.0, 100.0),
            ("ll_kd", "Low-level Kd", None, 0.1, 0.0, 10.0),
            ("ik_step_m", "End-effector step (m)", None, 0.005, 0.005, 0.10),
            ("ik_rotation_rad", "End-effector turn step (rad)", None, 0.02, 0.02, 0.3),
            ("tactile_threshold", "Dex3 tactile alert threshold (raw)", None, 5000.0, 0.0, 200000.0),
            ("climb_fsm_id", "Climb FSM id", None, 1.0, 1.0, 9999.0),
        ]
        def display(row: tuple[Any, ...]) -> str:
            key, label, *_ = row
            value = self.settings[key]
            if isinstance(value, bool):
                suffix = "ON" if value else "OFF"
            elif isinstance(value, float):
                suffix = f"{value:g}"
            else:
                suffix = str(value)
            return f"{label:<39} {suffix}"
        cursor = 0
        while True:
            screen = self.screen
            screen.erase()
            height, width = screen.getmaxyx()
            screen.attron(curses.color_pair(6) | curses.A_BOLD)
            screen.addnstr(0, 0, " Settings · use ←/→ to adjust ".ljust(width - 1), width - 1)
            screen.attroff(curses.color_pair(6) | curses.A_BOLD)
            visible_count = max(1, height - 4)
            start = max(0, min(cursor - visible_count + 1, len(fields) - visible_count))
            for idx in range(start, min(len(fields), start + visible_count)):
                attr = curses.color_pair(2) | curses.A_BOLD if idx == cursor else curses.A_NORMAL
                screen.addnstr(idx - start + 2, 2, ("› " if idx == cursor else "  ") + display(fields[idx]), width - 4, attr)
            screen.addnstr(height - 1, 0, "↑/↓ select · ←/→ or Enter change · Esc back".ljust(width - 1), width - 1)
            screen.refresh()
            key = screen.getch()
            if key in (27, ord("q")):
                self._persist_settings()
                self.draw(screen)
                return
            if key == curses.KEY_UP:
                cursor = max(0, cursor - 1)
            elif key == curses.KEY_DOWN:
                cursor = min(len(fields) - 1, cursor + 1)
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT, 10, 13, curses.KEY_ENTER):
                key_name, _label, options, step, low, high = fields[cursor]
                value = self.settings[key_name]
                if isinstance(value, bool):
                    self.settings[key_name] = not value
                elif isinstance(options, list):
                    idx = options.index(value) if value in options else 0
                    direction = -1 if key == curses.KEY_LEFT else 1
                    self.settings[key_name] = options[(idx + direction) % len(options)]
                else:
                    direction = -1 if key == curses.KEY_LEFT else 1
                    self.settings[key_name] = type(value)(max(low, min(high, float(value) + direction * step)))
                self._persist_settings()

    def _run_external(self, command: list[str], timeout: float = 25.0) -> tuple[int, str]:
        screen = self.screen
        if screen is not None:
            curses.def_prog_mode()
            curses.endwin()
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
            return result.returncode, (result.stdout + result.stderr).strip()
        except Exception as exc:
            return -1, str(exc)
        finally:
            if screen is not None:
                curses.reset_prog_mode()
                screen.refresh()

    def open_wifi_menu(self) -> None:
        supplicant = "wpa_supplicant@wlan0.service"
        dhcp = "dhclient@wlan0.service"

        def run_sequence(steps: list[tuple[str, list[str]]], timeout: float = 90.0) -> tuple[int, str]:
            output = []
            for label, command in steps:
                code, text = self._run_external(command, timeout=timeout)
                output.append(f"{label}:\n{text or f'command exited with code {code}'}")
                if code != 0:
                    return code, "\n\n".join(output)
            return 0, "\n\n".join(output)

        def run_action(index: int) -> None:
            if index == 0:
                code, output = self._run_external(
                    ["systemctl", "status", "--no-pager", "--full", supplicant, dhcp], timeout=20)
                # systemctl status returns 3 when a listed unit is inactive;
                # its output is still the requested status report.
                if code == 3:
                    code = 0
            elif index == 1:
                code, output = run_sequence([
                    (f"Restart {supplicant}", ["sudo", "systemctl", "restart", supplicant]),
                    (f"Restart {dhcp} (waits for Wi-Fi association)", ["sudo", "systemctl", "restart", dhcp]),
                ], timeout=120)
            elif index == 2:
                code, output = run_sequence([
                    (f"Start {supplicant}", ["sudo", "systemctl", "start", supplicant]),
                    (f"Restart {dhcp} to obtain a lease after association", ["sudo", "systemctl", "restart", dhcp]),
                ], timeout=120)
            elif index == 3:
                code, output = run_sequence([
                    (f"Stop {dhcp}", ["sudo", "systemctl", "stop", dhcp]),
                    (f"Stop {supplicant}", ["sudo", "systemctl", "stop", supplicant]),
                ])
            elif index == 4:
                code, output = run_sequence([
                    (f"Enable {supplicant}", ["sudo", "systemctl", "enable", "--now", supplicant]),
                    (f"Enable {dhcp}", ["sudo", "systemctl", "enable", "--now", dhcp]),
                ], timeout=120)
            elif index == 5:
                code, output = run_sequence([
                    (f"Disable {dhcp} at boot", ["sudo", "systemctl", "disable", dhcp]),
                    (f"Disable {supplicant} at boot", ["sudo", "systemctl", "disable", supplicant]),
                ])
            elif index == 6:
                code, output = self._run_external(["sudo", "systemctl", "restart", supplicant], timeout=60)
            elif index == 7:
                code, output = self._run_external(["sudo", "systemctl", "restart", dhcp], timeout=90)
            elif index == 8:
                code, output = self._run_external(["sudo", "systemctl", "start", dhcp], timeout=90)
            elif index == 9:
                code, output = self._run_external(["sudo", "systemctl", "stop", dhcp], timeout=30)
            elif index == 10:
                code, output = self._run_external(["sudo", "wpa_cli", "-i", "wlan0", "status"], timeout=10)
            elif index == 11:
                code, output = self._run_external(["ip", "-4", "addr", "show", "dev", "wlan0"], timeout=10)
            elif index in (12, 13):
                unit = supplicant if index == 12 else dhcp
                code, output = self._run_external(["sudo", "journalctl", "-u", unit, "-n", "40", "--no-pager"], timeout=20)
            else:
                ssid = self._prompt("Wi-Fi SSID")
                if not ssid:
                    return
                code, output = self._run_external(["sudo", "bash", str(HERE / "setup_wpa_supplicant_wlan0.sh"), ssid], timeout=120)
            self.say("system" if code == 0 else "error", output or f"Command exited with code {code}.")
        self.open_menu("Wi-Fi · wlan0", [
            "Show supplicant + DHCP service status", "Restart Wi-Fi + renew DHCP (ordered)",
            "Start both services (supplicant, then obtain DHCP lease)", "Stop both services (DHCP, then supplicant)",
            "Enable both services at boot", "Disable both services at boot",
            "Restart supplicant only", "Restart DHCP client service", "Start DHCP client service",
            "Stop DHCP client service", "Show Wi-Fi association state", "Show wlan0 IPv4 address",
            "Show supplicant journal", "Show DHCP client journal", "Configure SSID (secure password prompt)",
        ], run_action, "↑/↓ select · Enter run · Esc back (DHCP waits for supplicant association)", stay_open=True)

    def _prompt(self, label: str) -> str:
        screen = self.screen
        if screen is None:
            return ""
        height, width = screen.getmaxyx()
        screen.addnstr(height - 2, 0, (label + ": ").ljust(width - 1), width - 1)
        curses.echo()
        screen.timeout(-1)
        try:
            raw = screen.getstr(height - 2, min(width - 2, len(label) + 2), max(1, width - len(label) - 4))
            return raw.decode("utf-8", "replace").strip()
        finally:
            curses.noecho()
            screen.timeout(100)

    def show_jetson(self) -> None:
        model_path = Path("/proc/device-tree/model")
        model = model_path.read_bytes().replace(b"\x00", b"").decode("utf-8", "replace") if model_path.exists() else "not reported"
        mem = {}
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                key, value = line.split(":", 1)
                mem[key] = int(value.strip().split()[0]) * 1024
        except Exception:
            pass
        total, available = mem.get("MemTotal"), mem.get("MemAvailable")
        used = total - available if total and available is not None else None
        disk = shutil.disk_usage("/")
        temps = []
        for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                temps.append(float(path.read_text().strip()) / 1000)
            except Exception:
                pass
        ips = []
        try:
            result = subprocess.run(["ip", "-br", "-4", "addr"], capture_output=True, text=True, timeout=3)
            ips = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        except Exception:
            pass
        rows = [("Board", model), ("Hostname", socket.gethostname()), ("Platform", platform.platform()),
                ("Python", sys.version.split()[0]),
                ("CPU cores", os.cpu_count() or "unknown"),
                ("Memory", f"{used / 2**30:.1f}/{total / 2**30:.1f} GiB {self._bar(used, total)}" if total and used is not None else "unavailable"),
                ("Root disk", f"{disk.free / 2**30:.1f}/{disk.total / 2**30:.1f} GiB free {self._bar(disk.used, disk.total)}"),
                ("Thermal zones", ", ".join(f"{temp:.1f}°C" for temp in temps) or "unavailable"),
                ("IPv4 interfaces", " · ".join(ips) or "unavailable")]
        try:
            uptime = float(Path("/proc/uptime").read_text().split()[0])
            rows.append(("Uptime", f"{uptime / 3600:.1f} hours"))
        except Exception:
            pass
        try:
            loads = os.getloadavg()
            rows.append(("Load average", " · ".join(f"{load:.2f}" for load in loads)))
        except (AttributeError, OSError):
            pass
        self._table("Jetson board", rows)

    def open_mode_menu(self) -> None:
        names = ["Zero torque", "Damp", "Sit", "Squat", "Prepare / stand", "Walk", "Run", "Climb", "Dev mode (disable ai_sport)"]

        def refresh_mode_state() -> str:
            try:
                motion = self.robot._motion_client()
                code, data = motion.CheckMode()
                if int(code) == 0:
                    if isinstance(data, str):
                        try:
                            data = json.loads(data)
                        except json.JSONDecodeError:
                            data = {}
                    active_mode = sdk_wrapper._mode_name(data).strip().lower()
                    self.dev_mode = active_mode not in ("ai_sport", "ai")
                    return f"Motion mode: {active_mode or 'released'}"
            except Exception:
                pass
            try:
                ai_sport = self.robot.get_service("ai_sport")
                if ai_sport is not None and ai_sport.get("status") in (0, 1):
                    self.dev_mode = int(ai_sport["status"]) == 1
                    return f"ai_sport {'OFF' if self.dev_mode else 'ON'}"
            except Exception:
                pass
            return "Mode state unavailable; using the last known state."

        def entries() -> list[str]:
            if self.dev_mode:
                return [names[0] + " (re-enable ai_sport first)"] + [
                    f"{name}  [disabled in dev mode]" for name in names[1:8]
                ]
            return names

        def select(index: int) -> None:
            if self.offline or self.robot is None:
                raise RuntimeError("Robot modes require live robot mode.")
            if self.dev_mode and index != 0:
                raise RuntimeError("Only zero torque is available in dev mode; select it to re-enable ai_sport.")
            if index == 0:
                if self.dev_mode:
                    client = self.robot._robot_state_client()
                    client.SetTimeout(5.0)
                    code = sdk_wrapper._result_code(client.ServiceSwitch("ai_sport", True))
                    if code != 0:
                        self.say("error", f"Could not re-enable ai_sport before zero torque (RPC {code}).")
                        return
                    motion_code = sdk_wrapper._result_code(self.robot._motion_client().SelectMode("ai_sport"))
                    if motion_code != 0:
                        self.say("error", f"ai_sport service is ON, but MotionSwitcher.SelectMode returned RPC {motion_code}.")
                        return
                    time.sleep(0.2)
                    self.dev_mode = False
                result = self.robot.zero_torque_mode()
            elif index == 1:
                result = self.robot.damp_mode()
            elif index == 2:
                loco = self.robot._client
                result = loco.Sit() if hasattr(loco, "Sit") else loco.SetFsmId(3)
            elif index == 3:
                result = self.robot._client.SetFsmId(2)
            elif index == 4:
                result = self.robot.prepare_mode()
            elif index == 5:
                result = self.robot.walk_mode(waist_locked=bool(self.settings["waist_locked"]))
            elif index == 6:
                result = self.robot.run_mode()
            elif index == 7:
                result = self.robot._client.SetFsmId(int(self.settings["climb_fsm_id"]))
            else:
                client = self.robot._robot_state_client()
                client.SetTimeout(5.0)
                code = sdk_wrapper._result_code(client.ServiceSwitch("ai_sport", False))
                if code != 0:
                    self.say("error", f"Could not enter dev mode: ai_sport switch returned RPC {code}.")
                    return
                self.dev_mode = True
                result = code
            code = int(result[0]) if isinstance(result, tuple) else int(result) if isinstance(result, (int, bool)) else int(result.get("code", 0)) if isinstance(result, dict) else 0
            if code != 0:
                hint = " (RPC timeout; check DDS interface and robot service availability)" if code == 3104 else ""
                self.say("error", f"{names[index]} RPC returned {code}; robot did not confirm the mode change.{hint}")
            else:
                if index == 8:
                    self.say("system", "ai_sport disabled; dev mode is active.")
                self.say("system", f"{names[index]} mode selected.")
        if self.menu_action_future is None or self.menu_action_future.done():
            self.menu_action_label = "Checking current robot mode"
            self.menu_action_future = self.action_executor.submit(refresh_mode_state)
        self.open_menu("Robot mode", entries, select, "Modes can move the robot. ↑/↓ select · Enter send · Esc back", stay_open=True, async_actions=True)

    def open_control(self) -> None:
        if self.offline or self.robot is None:
            self.say("error", "Robot control requires live mode.")
            return
        screen = self.screen
        if screen is None:
            return
        screen.timeout(50)
        try:
            self.robot._client.SetTimeout(2.0)
        except Exception:
            pass
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-motion")
        arm_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-arm")
        motion_future: concurrent.futures.Future[Any] | None = None
        motion_active = False
        arm_future: concurrent.futures.Future[Any] | None = None
        arm_cancel = threading.Event()
        hold_stop = threading.Event()
        hold_enabled = threading.Event()
        hold_targets: dict[int, float] = {}
        hold_targets_lock = threading.Lock()
        pending_arm_toggle: bool | None = None
        arm_toggle_target: bool | None = None
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
        hand_cycle = ("right", "left", "both")

        def hold_loop() -> None:
            arm_sdk = None
            while not hold_stop.wait(0.02):
                if not hold_enabled.is_set():
                    continue
                with hold_targets_lock:
                    targets = dict(hold_targets)
                if targets:
                    try:
                        if arm_sdk is None:
                            arm_sdk = self.robot._arm_sdk_client()
                        arm_sdk.write(
                            targets,
                            kp=float(self.settings["ll_kp"]),
                            kd=float(self.settings["ll_kd"]),
                            waist_kp={12: 200.0, 13: 200.0, 14: 480.0},
                            waist_kd={j: 12.0 for j in sdk_wrapper.WAIST_JOINTS},
                        )
                    except Exception as exc:
                        hold_enabled.clear()
                        status = f"Arm pose hold unavailable: {exc}"

        hold_thread = threading.Thread(target=hold_loop, name="g1-arm-hold", daemon=True)
        hold_thread.start()

        def move_selected(hand_selection: str, increment: list[float], position_only: bool) -> list[Any]:
            sides = ("right", "left") if hand_selection == "both" else (hand_selection,)
            results = []
            for side in sides:
                if arm_cancel.is_set():
                    break
                inc = list(increment)
                # ik_move_ee mirrors dy for the right arm internally. Convert
                # the logical outward direction to each side's local sign.
                if abs(inc[1]) > 0:
                    inc[1] *= -1 if side == "right" else 1
                results.append(self.robot.ik_move_ee(
                    hand=side, pose_increment=inc,
                    max_speed=float(self.settings["ll_max_joint_speed"] if self.settings["ramping"] else 0.75),
                    max_dq=0.2, kp=float(self.settings["ll_kp"]), kd=float(self.settings["ll_kd"]),
                    position_only=position_only, stop_event=arm_cancel))
            return results

        try:
            while True:
                now = time.monotonic()
                if now - tactile_last_poll >= 0.20:
                    tactile_last_poll = now
                    try:
                        snapshots = self.robot.get_dex3_hand_sensors(hand="both")
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
                        if arm_label == "release/engage":
                            self.arms_released = bool(arm_toggle_target)
                            arm_toggle_target = None
                            status = "Arms released" if self.arms_released else "Arms engaged"
                            if self.arms_released:
                                hold_enabled.clear()
                            else:
                                try:
                                    targets = self.robot._upper_body_pose(timeout=1.0)
                                    with hold_targets_lock:
                                        hold_targets.update(targets)
                                    hold_enabled.set()
                                except Exception as exc:
                                    status = f"Arms engaged; hold refresh failed: {exc}"
                        elif arm_label.startswith("Move "):
                            if isinstance(result, list):
                                with hold_targets_lock:
                                    for item in result:
                                        if isinstance(item, dict):
                                            command = item.get("command_targets") or {}
                                            if not hold_targets:
                                                hold_targets.update(command)
                                            joints = (sdk_wrapper.LEFT_ARM_JOINTS if item.get("hand") == "left"
                                                      else sdk_wrapper.RIGHT_ARM_JOINTS)
                                            hold_targets.update({joint: command[joint] for joint in joints if joint in command})
                                if hold_targets and not self.arms_released:
                                    hold_enabled.set()
                            status = f"{arm_label} completed · pose held"
                        else:
                            status = f"{arm_label} completed"
                            self.say("tool", f"{arm_label}: {result}")
                    except Exception as exc:
                        status = f"{arm_label} failed: {exc}"
                        self.say("error", status)
                    arm_future = None
                    arm_cancel = threading.Event()
                    arm_label = ""
                if pending_arm_toggle is not None and arm_future is None:
                    target_released = pending_arm_toggle
                    pending_arm_toggle = None
                    arm_toggle_target = target_released
                    arm_label = "release/engage"
                    method = self.robot.release_arms if target_released else self.robot.engage_arms
                    arm_future = arm_executor.submit(method)
                if deadline and now >= deadline:
                    if desired != (0.0, 0.0, 0.0):
                        desired = (0.0, 0.0, 0.0)
                        revision += 1
                    deadline = 0.0
                if motion_future is None and revision != sent_revision:
                    sent_revision = revision
                    if desired == (0.0, 0.0, 0.0):
                        if motion_active:
                            motion_future = executor.submit(self.robot.loco_stop)
                            motion_active = False
                    else:
                        vx, vy, vyaw = desired
                        motion_future = executor.submit(
                            lambda vx=vx, vy=vy, vyaw=vyaw: self.robot.loco_move(vx, vy, vyaw, duration_s=None)
                        )
                        motion_active = True
                screen.erase()
                h, w = screen.getmaxyx()
                if h < 14 or w < 40:
                    screen.addnstr(0, 0, "Terminal too small for /control (need 40x14). Resize and retry.", max(1, w - 1))
                    screen.addnstr(h - 1, 0, "Esc: exit control".ljust(max(1, w - 1)), max(1, w - 1))
                    screen.refresh()
                    key = screen.getch()
                    if key in (27, ord("x")):
                        return
                    continue
                screen.addnstr(0, 0, " G1 control · motion keys coalesce and stop automatically ".ljust(w - 1), w - 1, curses.A_BOLD)
                instructions = [
                    "↑/↓: forward/back · ←/→: rotate · stops after configured duration",
                    "w/s: increase/decrease x · d/a: increase/decrease y · q/e: increase/decrease z",
                    "m: cycle right/left/both hands · f: lock/unlock EE orientation",
                    "o/c: open/close Dex3 hands",
                    "r: release/unrelease arms (interrupts the active IK step)",
                    f"Hands: {hand_mode.upper()} · Orientation: {'LOCKED' if orientation_locked else 'FREE'} · Arms: {'RELEASED' if self.arms_released else 'ENGAGED'} · {status}",
                    f"Step {self.settings['ik_step_m']:g}m · move {self.settings['locomotion_speed']:g}m/s · turn {self.settings['rotation_speed']:g}rad/s",
                    "Esc: stop and exit control",
                ]
                for row, line in enumerate(instructions, 2):
                    screen.addnstr(row, 2, line, w - 4)
                threshold = float(self.settings["tactile_threshold"])
                title_row = 10
                screen.attron(curses.A_BOLD | curses.color_pair(6))
                screen.addnstr(title_row, 2, f"Dex3 tactile · sensor max (raw) · threshold {threshold:g} · red below / green above", w - 4)
                screen.attroff(curses.A_BOLD | curses.color_pair(6))
                column_width = max(18, (w - 6) // 2)
                tactile_rows = max(0, h - 14)
                for col, side in enumerate(("left", "right")):
                    x = 2 + col * column_width
                    snap = tactile_data.get(side)
                    age = None
                    if isinstance(snap, dict) and snap.get("timestamp") is not None:
                        try:
                            age = time.time() - float(snap["timestamp"])
                        except (TypeError, ValueError):
                            age = None
                    hand_label = f"{side.upper()} HAND"
                    if dex3_available.get(side) is False:
                        hand_label += " · not detected"
                    elif age is None or age > 2.0:
                        hand_label += " · waiting for data" if not isinstance(snap, dict) else f" · stale {age:.1f}s"
                    screen.addnstr(title_row + 1, x, hand_label, max(1, min(column_width - 2, w - x - 1)), curses.A_BOLD)
                    screen.addnstr(title_row + 2, x, "Sensor  Value", max(1, min(column_width, w - x - 1)), curses.A_UNDERLINE)
                    pads = snap.get("tactile_pressures") if isinstance(snap, dict) else None
                    if not isinstance(pads, (list, tuple)) or not pads:
                        detail = (tactile_error or "Waiting for samples…") if dex3_available.get(side) is not False else "Hand not detected"
                        screen.addnstr(title_row + 3, x, detail, max(1, min(column_width - 2, w - x - 1)))
                        continue
                    overflow = len(pads) > tactile_rows
                    visible_count = max(0, tactile_rows - (1 if overflow else 0))
                    for pad_index, taxels in enumerate(pads[:visible_count]):
                        try:
                            valid = [float(v) for v in taxels if abs(float(v) - 30000.0) >= 0.5]
                        except (TypeError, ValueError):
                            valid = []
                        sensor_value = max(valid) if valid else None
                        value_text = "—" if sensor_value is None else f"{sensor_value:.0f}"
                        line = f"S{pad_index:<6} {value_text}"
                        color = (curses.color_pair(3) if sensor_value >= threshold else curses.color_pair(4)) if sensor_value is not None else curses.A_DIM
                        screen.addnstr(title_row + 3 + pad_index, x, line,
                                       max(1, min(column_width, w - x - 1)), color)
                    if overflow and tactile_rows:
                        screen.addnstr(title_row + 3 + visible_count, x,
                                       f"… {len(pads) - visible_count} more sensors",
                                       max(1, min(column_width, w - x - 1)), curses.A_DIM)
                screen.addnstr(h - 1, 0, f"Motion: {desired}   Esc: stop/exit".ljust(w - 1), w - 1)
                screen.refresh()
                key = screen.getch()
                if key == -1:
                    continue
                if key in (27, ord("x")):
                    desired = (0.0, 0.0, 0.0)
                    revision += 1
                    return
                if key in (curses.KEY_UP, curses.KEY_DOWN):
                    speed = float(self.settings["locomotion_speed"])
                    vx = speed if key == curses.KEY_UP else -speed
                    nxt = (vx, 0.0, 0.0)
                    if nxt != desired:
                        desired = nxt
                        revision += 1
                    deadline = time.monotonic() + float(self.settings["locomotion_duration"])
                    continue
                if key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                    rate = float(self.settings["rotation_speed"]) * (1 if key == curses.KEY_LEFT else -1)
                    nxt = (0.0, 0.0, rate)
                    if nxt != desired:
                        desired = nxt
                        revision += 1
                    deadline = time.monotonic() + float(self.settings["rotation_duration"])
                    continue
                if key == ord("r"):
                    # Flush buffered repeats and cancel an in-flight IK ramp.
                    desired = (0.0, 0.0, 0.0)
                    revision += 1
                    deadline = 0.0
                    try:
                        curses.flushinp()
                    except curses.error:
                        pass
                    if pending_arm_toggle is None:
                        pending_arm_toggle = not self.arms_released
                        hold_enabled.clear()
                        if arm_future is not None:
                            arm_cancel.set()
                        status = "Stopping arm motion before release/engage…"
                    continue
                if key == ord("m"):
                    hand_mode = hand_cycle[(hand_cycle.index(hand_mode) + 1) % len(hand_cycle)]
                    status = f"Selected {hand_mode} hand control"
                    continue
                if key == ord("f"):
                    orientation_locked = not orientation_locked
                    status = f"EE orientation {'locked' if orientation_locked else 'free'} during position moves"
                    continue
                if key in (ord("o"), ord("c")):
                    if arm_future is not None or pending_arm_toggle is not None:
                        status = "Arm busy; command not queued"
                        continue
                    selected_sides = ("left", "right") if hand_mode == "both" else (hand_mode,)
                    selected_sides = tuple(side for side in selected_sides if dex3_available.get(side) is True)
                    if not selected_sides:
                        status = f"Dex3 {hand_mode} hand unavailable; no command sent"
                        continue
                    arm_label = "Open hand" if key == ord("o") else "Close hand"
                    method = self.robot.open_dex3_hand if key == ord("o") else self.robot.close_dex3_hand
                    arm_future = arm_executor.submit(lambda m=method, sides=selected_sides: {
                        side: m(hand=side) for side in sides
                    })
                    status = f"{arm_label}…"
                    continue
                moves = {
                    ord("w"): (1, 0, 0), ord("s"): (-1, 0, 0),
                    ord("d"): (0, 1, 0), ord("a"): (0, -1, 0),
                    ord("q"): (0, 0, 1), ord("e"): (0, 0, -1),
                }
                if key in moves:
                    if arm_future is not None or pending_arm_toggle is not None:
                        status = "Arm busy; command not queued"
                        continue
                    unit = moves[key]
                    step = float(self.settings["ik_step_m"])
                    increment = [unit[0] * step, unit[1] * step, unit[2] * step, 0, 0, 0]
                    arm_cancel.clear()
                    arm_label = f"Move {chr(key)}"
                    hold_enabled.clear()
                    arm_future = arm_executor.submit(move_selected, hand_mode, increment, not orientation_locked)
                    status = f"{arm_label}…"
        except Exception as exc:
            self.say("error", f"Control error: {type(exc).__name__}: {exc}")
        finally:
            hold_enabled.clear()
            hold_stop.set()
            hold_thread.join(timeout=0.2)
            try:
                if motion_future is not None and not motion_future.done():
                    motion_future.result(timeout=0.5)
            except Exception:
                pass
            if motion_active:
                try:
                    executor.submit(self.robot.loco_stop)
                except Exception:
                    pass
            if arm_future is not None and not arm_future.done():
                arm_cancel.set()
                try:
                    arm_future.result(timeout=0.3)
                except Exception:
                    pass
            executor.shutdown(wait=False)
            arm_executor.shutdown(wait=False)
            try:
                self.robot._client.SetTimeout(10.0)
            except Exception:
                pass
            screen.timeout(100)
            self.draw(screen)

    @staticmethod
    def _color(role: str) -> int:
        return {"you": 2, "agent": 3, "error": 4, "tool": 5, "system": 6}.get(role, 1)

    def draw(self, screen: Any) -> None:
        screen.erase()
        height, width = screen.getmaxyx()
        if height < 8 or width < 40:
            screen.addnstr(0, 0, "Terminal too small (need 40x8).", max(1, width - 1))
            screen.refresh()
            return
        title = f" G1  /  {self.mode} "
        screen.attron(curses.color_pair(6) | curses.A_BOLD)
        screen.addnstr(0, 0, title.ljust(width - 1), width - 1)
        screen.attroff(curses.color_pair(6) | curses.A_BOLD)
        content_height = height - 4
        visible: list[tuple[int, str]] = []
        for role, line in self.lines:
            prefix = {"you": "you  ", "agent": "g1   ", "error": "error", "tool": "tool ", "system": "     "}.get(role, "     ")
            wrapped = textwrap.wrap(line, width=max(10, width - 7), replace_whitespace=False, drop_whitespace=False) or [""]
            visible.extend((self._color(role), (prefix if idx == 0 else "     ") + part) for idx, part in enumerate(wrapped))
        self.scroll_offset = min(self.scroll_offset, max(0, len(visible) - content_height))
        end = len(visible) - self.scroll_offset
        start = max(0, end - content_height)
        visible = visible[start:end]
        for y, (color, line) in enumerate(visible, start=1):
            try:
                screen.attron(curses.color_pair(color))
                screen.addnstr(y, 0, line, width - 1)
                screen.attroff(curses.color_pair(color))
            except curses.error:
                pass
        screen.attron(curses.A_BOLD)
        screen.addnstr(height - 3, 0, "─" * (width - 1), width - 1)
        screen.attroff(curses.A_BOLD)
        if self.pending_confirmation is not None:
            skill_name, reason, _event, _answer = self.pending_confirmation
            prompt = f"Confirm {skill_name}? {reason} [y/N]"
        elif self.chat_future is not None and not self.chat_future.done():
            spinner = "|/-\\"[self.spinner_index % 4]
            self.spinner_index += 1
            prompt = f"{spinner} Thinking…  {self.input}"
        else:
            prompt = f"› {self.input}"
        screen.addnstr(height - 2, 0, prompt, width - 1)
        footer = f"Enter send  ·  ↑/↓ history  ·  Tab complete  ·  wheel/Ctrl+↑↓ scroll ({self.scroll_offset})  ·  /help"
        screen.attron(curses.color_pair(6))
        screen.addnstr(height - 1, 0, footer.ljust(width - 1), width - 1)
        screen.attroff(curses.color_pair(6))
        screen.move(height - 2, min(width - 1, 2 + len(self.input)))
        screen.refresh()

    def run(self, screen: Any) -> None:
        self.screen = screen
        curses.curs_set(1)
        curses.start_color()
        curses.use_default_colors()
        self._init_colors()
        screen.keypad(True)
        try:
            curses.mousemask(curses.ALL_MOUSE_EVENTS | getattr(curses, "REPORT_MOUSE_POSITION", 0))
        except curses.error:
            pass
        screen.timeout(100)
        while self.running:
            self._poll_background()
            self.draw(screen)
            key = screen.getch()
            if key in (-1,):
                continue
            if self.pending_confirmation is not None:
                skill_name, _reason, event, answer = self.pending_confirmation
                if key in (ord("y"), ord("Y")):
                    answer["approved"] = True
                    self.say("system", f"Approved {skill_name}.")
                    event.set()
                    self.pending_confirmation = None
                elif key in (ord("n"), ord("N"), 10, 13, 27, 3):
                    answer["approved"] = False
                    self.say("system", f"Declined {skill_name}.")
                    event.set()
                    self.pending_confirmation = None
                continue
            if key == curses.KEY_MOUSE:
                try:
                    _mouse_id, _mx, _my, _z, button = curses.getmouse()
                    if button & getattr(curses, "BUTTON4_PRESSED", 0):
                        self.scroll_offset += 3
                    elif button & getattr(curses, "BUTTON5_PRESSED", 0):
                        self.scroll_offset = max(0, self.scroll_offset - 3)
                except curses.error:
                    pass
                continue
            # xterm-style Ctrl+Up/Down (ESC [ 1 ; 5 A/B). These often arrive
            # as a byte sequence because curses has no portable key constants.
            if key == 27:
                screen.timeout(25)
                seq = []
                try:
                    for _ in range(6):
                        part = screen.getch()
                        if part < 0:
                            break
                        seq.append(part)
                        if part in (ord("A"), ord("B")):
                            break
                finally:
                    screen.timeout(100)
                if seq and seq[-1:] in ([ord("A")], [ord("B")]) and ord("5") in seq:
                    if seq[-1] == ord("A"):
                        self.scroll_offset += 3
                    else:
                        self.scroll_offset = max(0, self.scroll_offset - 3)
                continue
            if key in (3, 4):  # Ctrl-C / Ctrl-D
                self.running = False
            elif key == curses.KEY_PPAGE:
                self.scroll_offset = min(max(0, len(self.lines)), self.scroll_offset + max(1, screen.getmaxyx()[0] - 6))
            elif key == curses.KEY_NPAGE:
                self.scroll_offset = max(0, self.scroll_offset - max(1, screen.getmaxyx()[0] - 6))
            elif key == curses.KEY_UP:
                if self.input_history:
                    if self.history_index is None:
                        self.history_draft = self.input
                        self.history_index = len(self.input_history) - 1
                    else:
                        self.history_index = max(0, self.history_index - 1)
                    self.input = self.input_history[self.history_index]
                    self.completion_matches = []
                else:
                    self.scroll_offset += 3
            elif key == curses.KEY_DOWN:
                if self.history_index is not None:
                    self.history_index += 1
                    if self.history_index >= len(self.input_history):
                        self.history_index = None
                        self.input = self.history_draft
                    else:
                        self.input = self.input_history[self.history_index]
                    self.completion_matches = []
                else:
                    self.scroll_offset = max(0, self.scroll_offset - 3)
            elif key == curses.KEY_HOME:
                self.scroll_offset = 10**9
            elif key == curses.KEY_END:
                self.scroll_offset = 0
            elif key in (curses.KEY_ENTER, 10, 13):
                raw, self.input = self.input, ""
                if raw.strip() and (not self.input_history or self.input_history[-1] != raw.strip()):
                    self.input_history.append(raw.strip())
                self.history_index = None
                self.history_draft = ""
                self.completion_matches = []
                self.scroll_offset = 0
                self.execute(raw)
            elif key == 9:  # Tab: complete or cycle slash commands.
                head, separator, tail = self.input.partition(" ")
                if head.startswith("/"):
                    if not self.completion_matches or not head.startswith(self.completion_prefix):
                        self.completion_matches = [cmd for cmd in COMMANDS if cmd.startswith(head)]
                        self.completion_index = 0
                        self.completion_prefix = head
                    elif self.completion_matches:
                        self.completion_index = (self.completion_index + 1) % len(self.completion_matches)
                    if self.completion_matches:
                        match = self.completion_matches[self.completion_index]
                        self.input = match + (" " + tail if separator else (" " if len(self.completion_matches) == 1 else ""))
                self.history_index = None
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                self.input = self.input[:-1]
                self.completion_matches = []
                self.completion_prefix = ""
                self.history_index = None
            elif key == curses.KEY_RESIZE:
                continue
            elif 0 <= key <= 0x10FFFF:
                self.input += chr(key)
                self.completion_matches = []
                self.completion_prefix = ""
                self.history_index = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="G1 full-screen natural language console")
    parser.add_argument("--iface", default=os.environ.get("G1_NETWORK_INTERFACE", "eth0"))
    parser.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    parser.add_argument("--model", default=os.environ.get("OPENAI_CHAT_MODEL", OpenAIPlanner.DEFAULT_MODEL))
    parser.add_argument("--offline", action="store_true", help="start without robot or OpenAI; live chat is disabled")
    parser.add_argument("--no-tui", action="store_true", help="run a line based REPL (useful over SSH logging)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        robot = None if args.offline else sdk_wrapper.G1(iface=args.iface, domain_id=args.domain_id)
        agent = _make_agent(args, robot)
    except Exception as exc:
        print(f"Could not start g1_cli: {exc}", file=sys.stderr)
        return 2
    console = G1Console(robot, agent, offline=args.offline)
    if args.no_tui:
        print("G1 console · /help for commands · /exit to quit")
        while console.running:
            try:
                console.execute(input("› "))
                for role, line in console.lines[-80:]:
                    print(f"{role}: {line}")
                console.lines.clear()
            except (EOFError, KeyboardInterrupt):
                break
        console.chat_executor.shutdown(wait=False)
        console.action_executor.shutdown(wait=False)
        return 0
    try:
        curses.wrapper(console.run)
    except KeyboardInterrupt:
        pass
    finally:
        console.chat_executor.shutdown(wait=False)
        console.action_executor.shutdown(wait=False)
        close = getattr(robot, "close", None)
        if callable(close):
            close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
