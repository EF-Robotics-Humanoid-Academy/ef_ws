#!/usr/bin/env python3
"""Play huddle.wav between two arm motions (extend → audio → lift → lower)."""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import tempfile
import wave
import sys
import time
import threading
from typing import Dict, List, Tuple, Iterable, Any

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


def _find_player() -> list[str] | None:
    for cmd in ("aplay", "paplay", "ffplay"):
        if subprocess.call(["/usr/bin/env", "which", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            return [cmd]
    return None


def _load_audio_client():
    try:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient  # type: ignore

        return AudioClient
    except Exception as exc:
        raise SystemExit(
            "unitree_sdk2py AudioClient is not available. Install unitree_sdk2_python and ensure AudioClient exists."
        ) from exc


def _parse_level(value: str) -> int:
    try:
        level = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("level must be an integer 0-100") from exc
    if not 0 <= level <= 100:
        raise argparse.ArgumentTypeError("level must be in range 0-100")
    return level


def _init_channel(iface: str) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0, iface)


def _set_volume(level: int, iface: str) -> None:
    _init_channel(iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(3.0)
    client.Init()
    code = client.SetVolume(level)
    if code != 0:
        raise SystemExit(f"SetVolume failed: code={code}")


def _set_brightness(level: int, iface: str) -> None:
    _init_channel(iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(3.0)
    client.Init()
    val = max(0, min(255, int(level * 255 / 100)))
    code = client.LedControl(val, val, val)
    if code != 0:
        raise SystemExit(f"LedControl failed: code={code}")


def _play_wav_robot(wav_path: str, iface: str, volume: int | None) -> float:
    if not os.path.exists(wav_path):
        print(f"Missing wav file: {wav_path}")
        raise SystemExit(1)

    _init_channel(iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(5.0)
    client.Init()

    if volume is not None:
        code = client.SetVolume(volume)
        if code != 0:
            raise SystemExit(f"SetVolume failed: code={code}")

    tmp_path = None
    with wave.open(wav_path, "rb") as wf:
        channels = wf.getnchannels()
        rate = wf.getframerate()
        width = wf.getsampwidth()
        if channels != 1 or rate != 16000 or width != 2:
            try:
                subprocess.run(["/usr/bin/env", "which", "ffmpeg"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                raise SystemExit(
                    "WAV must be mono 16-bit PCM at 16kHz. Install ffmpeg for auto-convert."
                )
            tmp_fd, tmp_path = tempfile.mkstemp(suffix=".wav")
            os.close(tmp_fd)
            subprocess.run(
                ["ffmpeg", "-y", "-i", wav_path, "-ac", "1", "-ar", "16000", "-f", "wav", tmp_path],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            wav_path = tmp_path
            with wave.open(wav_path, "rb") as wf2:
                pcm = wf2.readframes(wf2.getnframes())
                duration = wf2.getnframes() / float(wf2.getframerate())
        else:
            pcm = wf.readframes(wf.getnframes())
            duration = wf.getnframes() / float(rate)

    code, data = client.PlayStream("huddle", "huddle-1", pcm)
    if code != 0:
        raise SystemExit(f"PlayStream failed: code={code}, data={data}")
    if tmp_path:
        try:
            os.remove(tmp_path)
        except Exception:
            pass
    return float(duration)


JOINT_INDEX = {
    "left": {
        "shoulder_pitch": 15,
        "shoulder_roll": 16,
        "shoulder_yaw": 17,
        "elbow": 18,
        "wrist_roll": 19,
        "wrist_pitch": 20,
        "wrist_yaw": 21,
    },
    "right": {
        "shoulder_pitch": 22,
        "shoulder_roll": 23,
        "shoulder_yaw": 24,
        "elbow": 25,
        "wrist_pitch": 26,
        "wrist_roll": 27,
        "wrist_yaw": 28,
    },
}

JOINT_ALIASES = {
    "shoulder": "shoulder_pitch",
    "shoulder_pitch": "shoulder_pitch",
    "shoulder_roll": "shoulder_roll",
    "shoulder_yaw": "shoulder_yaw",
    "elbow": "elbow",
    "wrist": "wrist_pitch",
    "wrist_pitch": "wrist_pitch",
    "wrist_roll": "wrist_roll",
    "wrist_yaw": "wrist_yaw",
    "waist_yaw": "waist_yaw",
    "waist": "waist_yaw",
}

DESC_RE = re.compile(
    r"(?:(left|right)\s+)?([a-z_ ]+)\s*[:-]?\s*([+-]?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees)?",
    re.IGNORECASE,
)


def _normalize_joint_name(name: str) -> str:
    key = name.strip().lower().replace("-", " ").replace("_", " ")
    key = re.sub(r"\s+", " ", key)
    key = key.replace(" ", "_")
    return JOINT_ALIASES.get(key, key)


def _parse_descriptions(descriptions: Iterable[str], default_arm: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for line in descriptions:
        match = DESC_RE.search(line.strip())
        if not match:
            raise ValueError(f"Could not parse description: '{line}'")
        arm_raw, joint_raw, deg_raw = match.group(1), match.group(2), match.group(3)
        arm = (arm_raw or default_arm).lower()
        if arm not in JOINT_INDEX:
            raise ValueError(f"Unknown arm '{arm}' in description: '{line}'")
        if arm != default_arm:
            raise ValueError(f"Description arm '{arm}' does not match target arm '{default_arm}'")
        joint = _normalize_joint_name(joint_raw)
        out[joint] = float(deg_raw)
    return out


class ArmSdkController:
    """Simple arm SDK pose sequencer using rt/arm_sdk (LowCmd)."""

    _WAIST_YAW_IDX = 12
    _NOT_USED_IDX = 29  # enable arm sdk when q = 1

    def __init__(self, iface: str, arm: str, cmd_hz: float, kp: float, kd: float) -> None:
        self._arm = arm
        self._cmd_hz = max(1.0, cmd_hz)
        self._kp = kp
        self._kd = kd
        self._cmd_q: Dict[int, float] = {}
        self._crc = CRC()

        ChannelFactoryInitialize(0, iface)

        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()

        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.motor_cmd[self._NOT_USED_IDX].q = 1

        if arm == "left":
            self._joint_idx = [15, 16, 17, 18, 19, 20, 21]
        else:
            self._joint_idx = [22, 23, 24, 25, 26, 27, 28]

        for idx in self._joint_idx:
            self._cmd_q[idx] = 0.0
        self._cmd_q[self._WAIST_YAW_IDX] = 0.0

        self._joint_cur: Dict[int, float] = {}
        self._state_ready = threading.Event()
        self._initialised_from_state = False
        self._ls_sub = None

        threading.Thread(target=self._init_lowstate_sub, daemon=True).start()

    def _init_lowstate_sub(self) -> None:
        """Subscribe to LowState to seed initial joint angles and avoid snap."""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
        except Exception:
            return

        candidates = [
            "unitree_sdk2py.idl.unitree_hg.msg.dds_.LowState_",
            "unitree_sdk2py.idl.unitree_go.msg.dds_.LowState_",
        ]

        for dotted in candidates:
            try:
                mod_path, cls_name = dotted.rsplit(".", 1)
                mod = __import__(mod_path, fromlist=[cls_name])
                LowState_ = getattr(mod, cls_name)

                def _ls_cb(msg):
                    for j_idx in (*self._joint_idx, self._WAIST_YAW_IDX):
                        try:
                            self._joint_cur[j_idx] = msg.motor_state[j_idx].q
                        except Exception:
                            pass
                    if self._joint_cur:
                        self._state_ready.set()

                sub = ChannelSubscriber("rt/lowstate", LowState_)
                sub.Init(_ls_cb, 200)
                self._ls_sub = sub
                return
            except Exception:
                continue

    def seed_from_lowstate(self, timeout_s: float = 0.6) -> bool:
        """Seed commanded joints from measured angles to avoid a sudden jump."""
        if self._initialised_from_state:
            return True
        self._state_ready.wait(timeout=max(0.0, timeout_s))
        if not self._joint_cur:
            return False
        for j_idx, q_val in self._joint_cur.items():
            if j_idx in self._cmd_q:
                self._cmd_q[j_idx] = float(q_val)
        self._initialised_from_state = True
        return True

    def _apply_targets(self, targets: Dict[int, float]) -> None:
        for j_idx, q_val in targets.items():
            mc = self._cmd.motor_cmd[j_idx]
            mc.q = float(q_val)
            mc.kp = float(self._kp)
            mc.kd = float(self._kd)
            mc.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def ramp_to_pose(
        self,
        pose: List[Tuple[int, float]],
        duration: float,
        easing: str = "smooth",
    ) -> None:
        """Ramp the arm joints to the target pose over duration seconds."""
        target = {j: q for j, q in pose}
        start = {j: self._cmd_q.get(j, 0.0) for j in target}

        steps = max(1, int(self._cmd_hz * max(0.0, duration)))
        dt = 1.0 / self._cmd_hz

        for step in range(1, steps + 1):
            alpha = step / steps
            if easing == "smooth":
                alpha = 0.5 - 0.5 * math.cos(math.pi * alpha)
            cur = {j: start[j] + (target[j] - start[j]) * alpha for j in target}
            self._apply_targets(cur)
            time.sleep(dt)

        self._cmd_q.update(target)

    def hold_pose(self, pose: List[Tuple[int, float]], hold_s: float) -> None:
        if hold_s <= 0:
            return
        target = {j: q for j, q in pose}
        steps = max(1, int(self._cmd_hz * hold_s))
        dt = 1.0 / self._cmd_hz
        for _ in range(steps):
            self._apply_targets(target)
            time.sleep(dt)

    def hold_pose_until_done(self, pose: List[Tuple[int, float]], proc: subprocess.Popen) -> None:
        target = {j: q for j, q in pose}
        dt = 1.0 / self._cmd_hz
        while True:
            if proc.poll() is not None:
                break
            self._apply_targets(target)
            time.sleep(dt)


def _default_poses(arm: str) -> Dict[str, List[Tuple[int, float]]]:
    # These are conservative poses based on existing scripts (run_geoff_gui.py).
    # Tune as needed for your robot.
    if arm == "left":
        side = [
            (12, 0.0),
            (15, +0.211),  # shoulder pitch
            (16, +0.181),  # shoulder roll
            (17, -0.284),  # shoulder yaw
            (18, +0.672),  # elbow
            (19, -0.379),  # wrist roll
            (20, -0.852),  # wrist pitch
            (21, -0.019),  # wrist yaw
        ]
        extend = [
            (12, 0.0),
            (15, +0.380),
            (16, +0.060),
            (17, -0.240),
            (18, +0.420),
            (19, -0.379),
            (20, -0.852),
            (21, -0.050),
        ]
        lift = [
            (12, 0.0),
            (15, -0.300),
            (16, +0.080),
            (17, -0.200),
            (18, +0.520),
            (19, -0.379),
            (20, -0.852),
            (21, -0.050),
        ]
    else:
        side = [
            (12, 0.0),
            (22, +0.180),  # shoulder pitch (neutral)
            (23, -0.100),  # shoulder roll
            (24, -0.275),  # shoulder yaw
            (25, +1.273),  # elbow
            (26, -1.440),  # wrist pitch
            (27, +0.000),  # wrist roll
            (28, +0.000),  # wrist yaw
        ]
        extend = [
            (12, 0.0),
            (22, -1.000),  # shoulder up ~90 deg
            (23, -0.150),
            (24, -0.275),
            (25, +1.200),
            (26, -1.440),
            (27, +0.000),
            (28, +0.000),
        ]
        lift = [
            (12, 0.0),
            (22, -1.450),  # shoulder further up
            (23, -0.150),
            (24, -0.275),
            (25, +1.200),
            (26, -1.440),
            (27, +0.000),
            (28, +0.000),
        ]

    return {"side": side, "extend": extend, "lift": lift}


def _load_steps(path: str, arm: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    file_arm = (data.get("arm") or arm).lower()
    if file_arm != arm:
        raise SystemExit(f"steps.json arm '{file_arm}' does not match requested '{arm}'")
    return data


def _build_pose_from_angles(arm: str, angles: Dict[str, float]) -> List[Tuple[int, float]]:
    pose: List[Tuple[int, float]] = []
    idx_map = JOINT_INDEX[arm]
    for joint_name, deg in angles.items():
        key = _normalize_joint_name(joint_name)
        if key == "waist_yaw":
            pose.append((ArmSdkController._WAIST_YAW_IDX, math.radians(deg)))
            continue
        if key not in idx_map:
            raise ValueError(f"Unknown joint '{key}' for arm '{arm}'")
        pose.append((idx_map[key], math.radians(deg)))
    return pose


def main() -> None:
    parser = argparse.ArgumentParser(description="Huddle sequence: extend → audio → lift → lower.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--arm", choices=["left", "right"], default="right", help="which arm to move")
    parser.add_argument("--steps", default="", help="optional steps.json with pose descriptions")
    parser.add_argument("--file", default="huddle.wav", help="path to wav file")
    parser.add_argument("--volume", type=_parse_level, default=None, help="set robot speaker volume (0-100)")
    parser.add_argument("--brightness", type=_parse_level, default=None, help="set headlight brightness (0-100)")
    parser.add_argument("--cmd-hz", type=float, default=50.0, help="command rate for arm SDK")
    parser.add_argument("--kp", type=float, default=40.0, help="arm joint kp")
    parser.add_argument("--kd", type=float, default=1.0, help="arm joint kd")
    parser.add_argument("--extend-sec", type=float, default=3.5, help="seconds to extend hand")
    parser.add_argument("--hold-extend", type=float, default=0.3, help="seconds to hold extend pose")
    parser.add_argument("--lift-sec", type=float, default=1.5, help="seconds to lift hand")
    parser.add_argument("--hold-lift", type=float, default=0.3, help="seconds to hold lift pose")
    parser.add_argument("--lower-sec", type=float, default=3.0, help="seconds to lower back to side")
    parser.add_argument(
        "--easing",
        choices=["linear", "smooth"],
        default="smooth",
        help="easing profile for arm ramps",
    )
    parser.add_argument("--calibrate-shoulder", action="store_true",
                        help="print joint angles from lowstate for calibration")
    args = parser.parse_args()

    wav_path = args.file
    if not os.path.isabs(wav_path):
        wav_path = os.path.join(os.path.dirname(__file__), wav_path)

    poses = _default_poses(args.arm)

    arm = ArmSdkController(args.iface, args.arm, args.cmd_hz, args.kp, args.kd)
    arm.seed_from_lowstate()

    if args.calibrate_shoulder:
        print("Calibrating shoulder joint indices. Move the shoulder and watch values.")
        print("Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(0.5)
                if not arm._state_ready.is_set():
                    continue
                # Print all joint indices used by the arm plus waist yaw
                vals = []
                for j in (*arm._joint_idx, arm._WAIST_YAW_IDX):
                    v = arm._joint_cur.get(j, None)
                    if v is not None:
                        vals.append(f"{j}:{v:+.3f}")
                if vals:
                    print(" ".join(vals))
        except KeyboardInterrupt:
            return

    if args.steps:
        steps_path = args.steps
        if not os.path.isabs(steps_path):
            steps_path = os.path.join(os.path.dirname(__file__), steps_path)
        data = _load_steps(steps_path, args.arm)
        steps = data.get("steps") or []
        if not steps:
            raise SystemExit("steps.json has no steps")

        current_pose: Dict[str, float] = {}
        for i, step in enumerate(steps, start=1):
            name = step.get("name") or f"step_{i}"
            duration = float(step.get("duration", 1.5))
            hold = float(step.get("hold", 0.0))
            angles = step.get("angles") or {}
            descriptions = step.get("descriptions") or []
            if descriptions:
                angles.update(_parse_descriptions(descriptions, args.arm))
            if not angles:
                raise SystemExit(f"Step '{name}' has no angles or descriptions")
            current_pose.update({ _normalize_joint_name(k): float(v) for k, v in angles.items() })
            pose = _build_pose_from_angles(args.arm, current_pose)

            print(f"Step {i}: {name}")
            arm.ramp_to_pose(pose, duration, easing=args.easing)
            arm.hold_pose(pose, hold)

            audio = step.get("audio")
            if audio:
                audio_file = audio.get("file", wav_path)
                if not os.path.isabs(audio_file):
                    audio_file = os.path.join(os.path.dirname(__file__), audio_file)
                if args.brightness is not None:
                    _set_brightness(args.brightness, args.iface)
                volume_level = audio.get("volume", 100 if args.volume is None else args.volume)
                _set_volume(int(volume_level), args.iface)
                duration_s = _play_wav_robot(audio_file, args.iface, int(volume_level))
                arm.hold_pose(pose, duration_s)
    else:
        print("Step 1: extend hand in front (palm down).")
        arm.ramp_to_pose(poses["extend"], args.extend_sec, easing=args.easing)
        arm.hold_pose(poses["extend"], args.hold_extend)

        print("Step 2: play audio (hold palm-down pose).")
        if args.brightness is not None:
            _set_brightness(args.brightness, args.iface)
        volume_level = 100 if args.volume is None else args.volume
        _set_volume(volume_level, args.iface)
        duration = _play_wav_robot(wav_path, args.iface, volume_level)
        arm.hold_pose(poses["extend"], duration)

        print("Step 3: lift hand (rotate shoulder).")
        arm.ramp_to_pose(poses["lift"], args.lift_sec, easing=args.easing)
        arm.hold_pose(poses["lift"], args.hold_lift)

        print("Step 4: lower arm back to side (slow).")
        arm.ramp_to_pose(poses["side"], args.lower_sec, easing=args.easing)


if __name__ == "__main__":
    main()
