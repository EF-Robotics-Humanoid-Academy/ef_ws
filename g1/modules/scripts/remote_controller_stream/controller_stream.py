"""Stream USB game-controller input over HTTP/SSE.

Run:
    python controller_stream.py

Endpoints:
    GET /             Small usage page.
    GET /snapshot     Current controller state as JSON.
    GET /events       Live Server-Sent Events stream.

The server binds to 0.0.0.0 by default so other devices on the LAN can
connect.  The implementation intentionally sends only input data; it does
not provide any endpoint that can control the laptop.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import queue
import sys
import threading
import time
from ctypes import wintypes
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import pygame


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8765


class InputState:
    """Thread-safe current state plus a set of SSE subscriber queues."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "timestamp": time.time(),
            "controllers": [],
        }
        self._subscribers: set[queue.Queue[dict[str, Any]]] = set()

    def get(self) -> dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._state))

    def update(self, state: dict[str, Any]) -> None:
        message = {"type": "state", "data": state}
        with self._lock:
            self._state = state
            subscribers = tuple(self._subscribers)
        for subscriber in subscribers:
            # A slow client must not stop controller polling.
            try:
                subscriber.put_nowait(message)
            except queue.Full:
                try:
                    subscriber.get_nowait()
                    subscriber.put_nowait(message)
                except queue.Empty:
                    pass

    def subscribe(self) -> queue.Queue[dict[str, Any]]:
        subscriber: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=2)
        with self._lock:
            self._subscribers.add(subscriber)
            subscriber.put_nowait({"type": "state", "data": self._state})
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[dict[str, Any]]) -> None:
        with self._lock:
            self._subscribers.discard(subscriber)


INPUT_STATE = InputState()


class _XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", wintypes.WORD),
        ("left_trigger", wintypes.BYTE),
        ("right_trigger", wintypes.BYTE),
        ("left_x", ctypes.c_short),
        ("left_y", ctypes.c_short),
        ("right_x", ctypes.c_short),
        ("right_y", ctypes.c_short),
    ]


class _XInputState(ctypes.Structure):
    _fields_ = [
        ("packet_number", wintypes.DWORD),
        ("gamepad", _XInputGamepad),
    ]


class XInputReader:
    """Read the first Xbox-compatible controller through Windows XInput."""

    _button_masks = {
        "A": 0x1000,
        "B": 0x2000,
        "X": 0x4000,
        "Y": 0x8000,
        "BACK": 0x0020,
        "START": 0x0010,
        "LEFT_STICK": 0x0040,
        "RIGHT_STICK": 0x0080,
        "LB": 0x0100,
        "RB": 0x0200,
        "DPAD_UP": 0x0001,
        "DPAD_DOWN": 0x0002,
        "DPAD_LEFT": 0x0004,
        "DPAD_RIGHT": 0x0008,
    }

    def __init__(self) -> None:
        if sys.platform != "win32":
            raise OSError("XInput is only available on Windows")

        self._get_state = None
        for dll_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
            try:
                dll = ctypes.WinDLL(dll_name)
                get_state = dll.XInputGetState
                get_state.argtypes = [wintypes.DWORD, ctypes.POINTER(_XInputState)]
                get_state.restype = wintypes.DWORD
                self._get_state = get_state
                break
            except (AttributeError, OSError):
                continue

        if self._get_state is None:
            raise OSError("No XInput DLL was found")

    def snapshot(self) -> dict[str, Any] | None:
        state = _XInputState()
        if self._get_state(0, ctypes.byref(state)) != 0:
            return None

        gamepad = state.gamepad
        buttons = [
            bool(gamepad.buttons & self._button_masks[name])
            for name in (
                "A", "B", "X", "Y", "BACK", "START",
                "LEFT_STICK", "RIGHT_STICK", "LB", "RB",
                "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT",
            )
        ]
        axes = [
            round(gamepad.left_x / 32767.0, 5),
            round(-gamepad.left_y / 32768.0, 5),
            round(gamepad.right_x / 32767.0, 5),
            round(-gamepad.right_y / 32768.0, 5),
            round(gamepad.left_trigger / 255.0, 5),
            round(gamepad.right_trigger / 255.0, 5),
        ]
        return {
            "id": 0,
            "name": "XInput controller",
            "guid": "xinput-0",
            "axes": axes,
            "buttons": buttons,
            "hats": [[
                int(bool(gamepad.buttons & self._button_masks["DPAD_RIGHT"]))
                - int(bool(gamepad.buttons & self._button_masks["DPAD_LEFT"])),
                int(bool(gamepad.buttons & self._button_masks["DPAD_UP"]))
                - int(bool(gamepad.buttons & self._button_masks["DPAD_DOWN"])),
            ]],
        }


def controller_snapshot(joysticks: list[pygame.joystick.Joystick]) -> dict[str, Any]:
    controllers = []
    for index, joystick in enumerate(joysticks):
        axes = [round(joystick.get_axis(axis), 5) for axis in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(button) != 0 for button in range(joystick.get_numbuttons())]
        hats = [list(joystick.get_hat(hat)) for hat in range(joystick.get_numhats())]
        controllers.append(
            {
                "id": index,
                "name": joystick.get_name(),
                "guid": joystick.get_guid(),
                "axes": axes,
                "buttons": buttons,
                "hats": hats,
            }
        )
    return {"timestamp": time.time(), "controllers": controllers}


class RequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _send_json(self, value: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        if self.path == "/snapshot":
            self._send_json(INPUT_STATE.get())
            return

        if self.path == "/events":
            self._stream_events()
            return

        if self.path in ("/", "/index.html"):
            self._send_json(
                {
                    "service": "controller-stream",
                    "endpoints": {
                        "snapshot": "/snapshot",
                        "events": "/events",
                    },
                    "event_format": "SSE data lines contain JSON: {type, data}",
                }
            )
            return

        self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _stream_events(self) -> None:
        subscriber = INPUT_STATE.subscribe()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            while True:
                try:
                    message = subscriber.get(timeout=15)
                    payload = json.dumps(message, separators=(",", ":"))
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                except queue.Empty:
                    self.wfile.write(b": heartbeat\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            INPUT_STATE.unsubscribe(subscriber)

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {format % args}")


def poll_controller(rate_hz: float) -> None:
    pygame.init()
    pygame.joystick.init()
    xinput_reader: XInputReader | None = None
    try:
        xinput_reader = XInputReader()
        print("Using Windows XInput for controller input.")
    except OSError:
        pass

    joysticks: list[pygame.joystick.Joystick] = []
    if xinput_reader is None:
        for index in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(index)
            joystick.init()
            joysticks.append(joystick)
            print(f"Controller {index}: {joystick.get_name()}")

    if xinput_reader is None and not joysticks:
        print("No controller detected yet; polling will continue.")

    delay = 1.0 / rate_hz
    previous: dict[str, Any] | None = None
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    print("Controller connected; restart the service to rescan it.")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    print("Controller disconnected; restart the service to rescan it.")

            xinput_state = xinput_reader.snapshot() if xinput_reader else None
            if xinput_state is not None:
                state = {"timestamp": time.time(), "controllers": [xinput_state]}
            else:
                state = controller_snapshot(joysticks)
            # The timestamp is intentionally refreshed for every poll, but
            # events should only be emitted when an input value changes.
            if previous is None or state["controllers"] != previous["controllers"]:
                INPUT_STATE.update(state)
                previous = state
            time.sleep(delay)
    finally:
        pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST, help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP port (default: 8765)")
    parser.add_argument("--rate", type=float, default=60.0, help="Polling rate in Hz (default: 60)")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), RequestHandler)
    print(f"Streaming on http://{args.host}:{args.port}/events")
    print("Press Ctrl+C to stop.")
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    try:
        poll_controller(args.rate)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
