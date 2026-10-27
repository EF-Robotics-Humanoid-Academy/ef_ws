"""MotionStudio Raw UDP JSON -> Inspire RH56DFX hand bridge.

The program is deliberately dry-run by default.  Add --execute only after
calibration has been checked and the hand is mechanically safe.

MotionStudio should broadcast Raw device stream JSON to UDP port 9999.
The RH56 is connected to COM7 at 115200 baud, hand ID 1.
"""

from __future__ import annotations

import argparse
import json
import socket
import statistics
import sys
import time
from pathlib import Path
from typing import Any


FINGER_NAMES = ("little", "ring", "middle", "index", "thumb1", "thumb2")
# Closed pose measured in InspireHand: Act Angle [0, 0, 0, 0, 1, 989].
# Use endpoint targets with a small readback tolerance.
RH56_CLOSED_TARGET = [0, 0, 0, 0, 0, 1000]
FINGER_PARTS = {
    "little": ("leftLittleFingerUnder", "leftLittleFingerMid", "leftLittleFingerUp"),
    "ring": ("leftRingFingerUnder", "leftRingFingerMid", "leftRingFingerUp"),
    "middle": ("leftMiddleFingerUnder", "leftMiddleFingerMid", "leftMiddleFingerUp"),
    "index": ("leftForeFingerUnder", "leftForeFingerMid", "leftForeFingerUp"),
    "thumb": ("leftThumbUnder", "leftThumbMid", "leftThumbUp"),
}


def _vec(value: Any) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) < 3:
        return (0.0, 0.0, 0.0)
    return tuple(float(value[i]) for i in range(3))


def _finger_features(device: dict[str, Any]) -> list[float]:
    """Return monotonic-ish curl features from the observed Euler data.

    The four long-finger curl features use the Z rotation of all three
    phalanges.  Thumb bend uses the total rotation magnitude.  Thumb rotation
    uses the thumb-mid Y axis and is calibrated separately when possible.
    Calibration removes the remaining sign and scale ambiguity.
    """

    result: list[float] = []
    for name in ("little", "ring", "middle", "index"):
        result.append(sum(abs(_vec(device.get(part))[2]) for part in FINGER_PARTS[name]))

    thumb_parts = [_vec(device.get(part)) for part in FINGER_PARTS["thumb"]]
    # Use the same flexion-axis measure as the long fingers. Total 3-D
    # magnitude is affected by wrist orientation and was nearly constant in
    # the captured calibration, leaving the RH56 thumb-bend channel inactive.
    thumb_bend = sum(abs(part[2]) for part in thumb_parts)
    # Thumb2 is driven by the middle thumb orientation. The thumb-under Y
    # component was nearly constant during calibration and is not useful for
    # this glove stream.
    thumb_rotation = thumb_parts[1][1]
    result.extend((thumb_bend, thumb_rotation))
    return result


class Rh56Serial:
    HEADER = (0xEB, 0x90)
    CMD_WRITE = 0x12
    ANGLE_SET = 1486
    SPEED_SET = 1522
    FORCE_SET = 1498
    ANGLE_ACTUAL = 1546
    CMD_READ = 0x11

    def __init__(self, port: str, baudrate: int, hand_id: int, debug: bool = False) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.2,
            write_timeout=0.2,
        )
        self.hand_id = hand_id
        self.debug = debug

    def close(self) -> None:
        self._serial.close()

    def _write_register(self, address: int, values: list[int]) -> bytes:
        data: list[int] = []
        for value in values:
            value = max(0, min(1000, int(value)))
            data.extend((value & 0xFF, (value >> 8) & 0xFF))
        frame = [*self.HEADER, self.hand_id, len(data) + 3, self.CMD_WRITE,
                 address & 0xFF, (address >> 8) & 0xFF, *data]
        frame.append(sum(frame[2:]) & 0xFF)
        if self.debug:
            print("RH56 TX:", bytes(frame).hex(" "), flush=True)
        self._serial.write(bytes(frame))
        self._serial.flush()
        time.sleep(0.01)
        response = bytes(self._serial.read_all())
        if self.debug:
            print("RH56 RX:", response.hex(" ") or "<no response>", flush=True)
        return response

    def set_angles(self, angles: list[int]) -> bytes:
        return self._write_register(self.ANGLE_SET, angles)

    def set_speed_and_force(self, speed: int, force: int) -> None:
        self._write_register(self.SPEED_SET, [speed] * 6)
        self._write_register(self.FORCE_SET, [force] * 6)

    def read_register(self, address: int, num_bytes: int) -> list[int] | None:
        frame = [*self.HEADER, self.hand_id, 0x04, self.CMD_READ,
                 address & 0xFF, (address >> 8) & 0xFF, num_bytes]
        frame.append(sum(frame[2:]) & 0xFF)
        self._serial.reset_input_buffer()
        if self.debug:
            print("RH56 TX:", bytes(frame).hex(" "), flush=True)
        self._serial.write(bytes(frame))
        self._serial.flush()
        deadline = time.monotonic() + 0.25
        response = bytearray()
        while time.monotonic() < deadline:
            chunk = self._serial.read_all()
            if chunk:
                response.extend(chunk)
                # A response with a valid length byte is complete.
                if len(response) >= 4 and len(response) >= 4 + response[3]:
                    break
            else:
                time.sleep(0.005)
        response = bytes(response)
        if self.debug:
            print("RH56 RX:", response.hex(" ") or "<no response>", flush=True)
        # RH56 firmware commonly returns the response header as 90 EB,
        # while requests use EB 90. Accept both byte orders.
        valid_headers = (bytes(self.HEADER), bytes(reversed(self.HEADER)))
        if len(response) < 8 or response[0:2] not in valid_headers:
            return None
        payload_length = int(response[3]) - 3
        if payload_length < num_bytes or len(response) < 7 + payload_length:
            return None
        return list(response[7:7 + num_bytes])

    def read_angles(self) -> list[int] | None:
        data = self.read_register(self.ANGLE_ACTUAL, 12)
        if data is None or len(data) < 12:
            return None
        return [data[2 * i] | (data[2 * i + 1] << 8) for i in range(6)]


class RawReceiver:
    def __init__(self, bind_ip: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))

    def receive_device(self, timeout: float | None = None) -> dict[str, Any] | None:
        self.sock.settimeout(timeout)
        data, _ = self.sock.recvfrom(65535)
        # If the producer is faster than the serial consumer, discard queued
        # stale frames and keep only the newest valid hand packet.
        latest = data
        self.sock.setblocking(False)
        try:
            while True:
                try:
                    latest, _ = self.sock.recvfrom(65535)
                except BlockingIOError:
                    break
        finally:
            self.sock.settimeout(timeout)
        packet = json.loads(latest)
        for device in packet.get("deviceList", []):
            if device.get("deviceType") == "hand":
                return device
        return None


def capture(receiver: RawReceiver, label: str, seconds: float) -> list[float]:
    print(f"{label}: hold the requested pose for {seconds:.1f} seconds...")
    samples: list[list[float]] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            device = receiver.receive_device(timeout=0.5)
        except (TimeoutError, socket.timeout, json.JSONDecodeError):
            continue
        if device is not None:
            samples.append(_finger_features(device))
    if not samples:
        raise RuntimeError("No hand packets received during calibration")
    return [statistics.median(row[i] for row in samples) for i in range(6)]


def save_calibration(path: Path, opened: list[float], closed: list[float]) -> None:
    path.write_text(json.dumps({"open": opened, "closed": closed}, indent=2), encoding="utf-8")


def load_calibration(path: Path) -> tuple[list[float], list[float]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return list(map(float, data["open"])), list(map(float, data["closed"]))


def to_angles(features: list[float], opened: list[float], closed: list[float]) -> list[int]:
    angles: list[int] = []
    for channel, (value, open_value, close_value) in enumerate(zip(features, opened, closed)):
        span = close_value - open_value
        if abs(span) < 1e-3:
            # Open/close does not calibrate thumb rotation.  Keep it neutral.
            angles.append(500)
            continue
        ratio = (value - open_value) / span
        # The RH56 Thumb2/rotation actuator moves opposite to the glove's
        # calibrated open-to-closed direction, just like the other channels.
        # Invert channel 5 as well so rotating the glove thumb toward the
        # closed pose commands the RH56 thumb toward its closed pose.
        ratio = 1.0 - ratio
        angles.append(round(max(0.0, min(1.0, ratio)) * 1000.0))
    return angles


def rate_limit(target: list[int], previous: list[int], max_step: int) -> list[int]:
    return [max(p - max_step, min(p + max_step, t)) for t, p in zip(target, previous)]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Direct left glove -> Inspire RH56DFX bridge")
    parser.add_argument("--bind-ip", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=9999)
    parser.add_argument("--serial-port", default="COM7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--hand-id", type=int, default=1)
    parser.add_argument("--calibration", type=Path, default=Path("rh56_left_glove_calibration.json"))
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--max-step", type=int, default=25,
                        help="Maximum RH56 angle units per received packet")
    parser.add_argument("--speed", type=int, default=20, help="RH56 speed, 0-100")
    parser.add_argument("--force", type=int, default=15, help="RH56 force, 0-100")
    parser.add_argument("--timeout", type=float, default=0.25,
                        help="Stop sending after this many seconds without UDP")
    parser.add_argument("--send-hz", type=float, default=10.0,
                        help="Maximum RH56 command rate; keep low for initial tests")
    parser.add_argument("--verify", action="store_true",
                        help="Read back RH56 actual angles once per second; stop on failure")
    parser.add_argument("--debug-serial", action="store_true",
                        help="Print raw RH56 serial request/response frames")
    parser.add_argument("--startup-close", action="store_true",
                        help="Safely ramp the RH56 to the fully closed target before teleoperation")
    parser.add_argument("--startup-close-seconds", type=float, default=1.0,
                        help="Duration of the startup close ramp")
    parser.add_argument("--startup-close-timeout", type=float, default=3.0,
                        help="Maximum time to wait for the RH56 to reach closed")
    parser.add_argument("--startup-close-threshold", type=int, default=25,
                        help="Maximum readback angle accepted as fully closed")
    parser.add_argument("--startup-close-target", nargs=6, type=int,
                        default=RH56_CLOSED_TARGET, metavar=("L", "R", "M", "I", "T1", "T2"),
                        help="Six RH56 target angles for the startup pose")
    parser.add_argument("--execute", action="store_true",
                        help="Actually open COM7 and command the hand")
    parser.add_argument("--recalibrate", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    receiver = RawReceiver(args.bind_ip, args.udp_port)
    print(f"Listening for Raw device JSON on UDP {args.udp_port}")

    if args.recalibrate or not args.calibration.exists():
        input("Open your left hand, then press ENTER. ")
        opened = capture(receiver, "OPEN", args.calibration_seconds)
        print("Open features:", [round(x, 2) for x in opened])
        input("Close your left hand gently, then press ENTER. ")
        closed = capture(receiver, "CLOSED", args.calibration_seconds)
        print("Closed features:", [round(x, 2) for x in closed])
        save_calibration(args.calibration, opened, closed)
        print(f"Saved calibration to {args.calibration}")
    else:
        opened, closed = load_calibration(args.calibration)
        print(f"Loaded calibration from {args.calibration}")

    if any(abs(c - o) < 1e-3 for o, c in zip(opened, closed)):
        print("Warning: one or more channels did not change during open/close calibration.")
        print("Thumb rotation is normally not calibrated by open/close and will remain at 500.")

    hand: Rh56Serial | None = None
    initial_actual: list[int] | None = None
    startup_closed = False
    if args.execute:
        hand = Rh56Serial(args.serial_port, args.baudrate, args.hand_id, args.debug_serial)
        hand.set_speed_and_force(args.speed, args.force)
        print(f"EXECUTE enabled: controlling RH56 on {args.serial_port}")
        if args.verify or args.startup_close:
            actual = hand.read_angles()
            if actual is None:
                raise RuntimeError("RH56 angle readback failed; refusing to send motion commands")
            print("Initial RH56 actual angles:", actual)
            initial_actual = actual
        if args.startup_close:
            if args.startup_close_seconds <= 0:
                raise ValueError("--startup-close-seconds must be positive")
            close_target = [max(0, min(1000, value)) for value in args.startup_close_target]
            steps = max(1, round(args.startup_close_seconds * max(1.0, args.send_hz)))
            print(f"Startup close: ramping to {close_target} in {args.startup_close_seconds:.2f}s")
            start = initial_actual or [0] * 6
            for step in range(1, steps + 1):
                alpha = step / steps
                command = [round(a + (b - a) * alpha) for a, b in zip(start, close_target)]
                hand.set_angles(command)
                time.sleep(args.startup_close_seconds / steps)
            # Keep commanding closed until the measured positions confirm it,
            # but never wait indefinitely if a finger is obstructed or faulty.
            close_deadline = time.monotonic() + args.startup_close_timeout
            last_close_actual: list[int] | None = None
            while time.monotonic() < close_deadline:
                actual = hand.read_angles()
                last_close_actual = actual
                if actual is not None and all(
                    abs(angle - target) <= args.startup_close_threshold
                    for angle, target in zip(actual, close_target)
                ):
                    break
                hand.set_angles(close_target)
                time.sleep(1.0 / max(1.0, args.send_hz))
            else:
                raise RuntimeError(
                    "RH56 startup close timed out; last actual angles: "
                    f"{last_close_actual}. Target: {close_target}"
                )
            startup_closed = True
            print(f"Startup close complete; final actual angles: {last_close_actual}")
            input("Check the physical hand. Press ENTER to start glove teleoperation, or Ctrl+C to abort. ")
    else:
        print("DRY RUN: no serial port opened. Use --execute only for a safe hand test.")

    # Start rate limiting from the measured hand position. Starting from zero
    # can create an unnecessary jump or a long artificial ramp at startup.
    previous = ([0] * 6 if startup_closed else
                (list(initial_actual) if initial_actual is not None else [0] * 6))
    last_packet = time.monotonic()
    last_command = None
    next_send_time = time.monotonic()
    last_verify_time = time.monotonic()
    try:
        while True:
            try:
                device = receiver.receive_device(timeout=0.5)
            except (TimeoutError, socket.timeout):
                if time.monotonic() - last_packet > args.timeout:
                    print("UDP timeout: no new hand command", flush=True)
                continue
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                print(f"Ignoring malformed UDP packet: {exc}", file=sys.stderr)
                continue
            if device is None:
                continue
            last_packet = time.monotonic()
            target = to_angles(_finger_features(device), opened, closed)
            command = rate_limit(target, previous, args.max_step)
            now = time.monotonic()
            if now < next_send_time:
                continue
            next_send_time = now + 1.0 / max(0.1, args.send_hz)
            previous = command
            if command == last_command:
                continue
            if hand is not None:
                hand.set_angles(command)
                if args.verify and now - last_verify_time >= 1.0:
                    actual = hand.read_angles()
                    last_verify_time = now
                    if actual is None:
                        raise RuntimeError("RH56 angle readback failed; stopping motion output")
                    print("actual:", actual, flush=True)
            last_command = command
            print("angles:", command, flush=True)
    except KeyboardInterrupt:
        print("\nStopping bridge")
    finally:
        if hand is not None:
            hand.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
