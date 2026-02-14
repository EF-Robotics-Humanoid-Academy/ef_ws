#!/usr/bin/env python3
"""
Low-level arm motion runner for Unitree G1 using rt/arm_sdk.

Reads a steps.json file with high-level joint descriptions (degrees) and
applies them in sequence.

Example steps.json:
{
  "arm": "right",
  "cmd_hz": 50,
  "kp": 40,
  "kd": 1,
  "steps": [
    {
      "name": "pose_1",
      "duration": 2.0,
      "hold": 0.5,
      "angles": {
        "shoulder_pitch": -90,
        "elbow": 45,
        "wrist_pitch": -30
      }
    },
    {
      "name": "pose_2",
      "duration": 1.5,
      "descriptions": [
        "right shoulder - 60 degrees",
        "right wrist - 20 degrees"
      ]
    }
  ]
}
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import time
import threading
from typing import Dict, Iterable, List, Tuple

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc


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

WAIST_YAW_IDX = 12
NOT_USED_IDX = 29  # enable arm sdk when q = 1

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


class ArmSdkController:
    """Simple arm SDK pose sequencer using rt/arm_sdk (LowCmd)."""

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
        self._cmd.motor_cmd[NOT_USED_IDX].q = 1

        self._joint_idx = list(JOINT_INDEX[arm].values())
        for idx in self._joint_idx:
            self._cmd_q[idx] = 0.0
        self._cmd_q[WAIST_YAW_IDX] = 0.0

        self._joint_cur: Dict[int, float] = {}
        self._state_ready = threading.Event()
        self._initialised_from_state = False
        self._ls_sub = None

        threading.Thread(target=self._init_lowstate_sub, daemon=True).start()

    def _init_lowstate_sub(self) -> None:
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
                    for j_idx in (*self._joint_idx, WAIST_YAW_IDX):
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

    def ramp_to_pose(self, pose: List[Tuple[int, float]], duration: float, easing: str) -> None:
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


def _load_steps(path: str) -> Dict:
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _build_pose(
    arm: str,
    angles_deg: Dict[str, float],
    current: Dict[str, float],
) -> Dict[str, float]:
    updated = dict(current)
    for joint_name, deg in angles_deg.items():
        key = _normalize_joint_name(joint_name)
        updated[key] = float(deg)
    return updated


def _pose_to_indexed(arm: str, pose_deg: Dict[str, float]) -> List[Tuple[int, float]]:
    idx_map = JOINT_INDEX[arm]
    pose: List[Tuple[int, float]] = []
    for joint_name, deg in pose_deg.items():
        if joint_name == "waist_yaw":
            pose.append((WAIST_YAW_IDX, math.radians(deg)))
            continue
        if joint_name not in idx_map:
            raise ValueError(f"Unknown joint '{joint_name}' for arm '{arm}'")
        pose.append((idx_map[joint_name], math.radians(deg)))
    return pose


def main() -> None:
    parser = argparse.ArgumentParser(description="Apply arm poses from steps.json.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--steps", default="steps.json", help="path to steps.json")
    parser.add_argument("--arm", choices=["left", "right"], default="", help="override arm in file")
    parser.add_argument("--cmd-hz", type=float, default=0.0, help="override command rate (Hz)")
    parser.add_argument("--kp", type=float, default=0.0, help="override joint kp")
    parser.add_argument("--kd", type=float, default=0.0, help="override joint kd")
    parser.add_argument("--easing", choices=["linear", "smooth"], default="smooth", help="easing profile")
    parser.add_argument("--no-seed", action="store_true", help="skip seeding from lowstate")
    parser.add_argument("--dry-run", action="store_true", help="print resolved steps without commanding")
    args = parser.parse_args()

    steps_path = args.steps
    if not os.path.isabs(steps_path):
        steps_path = os.path.join(os.path.dirname(__file__), steps_path)

    data = _load_steps(steps_path)
    arm = (args.arm or data.get("arm") or "right").lower()
    if arm not in JOINT_INDEX:
        raise SystemExit(f"Unknown arm '{arm}', expected left/right")

    cmd_hz = float(args.cmd_hz or data.get("cmd_hz") or 50.0)
    kp = float(args.kp or data.get("kp") or 40.0)
    kd = float(args.kd or data.get("kd") or 1.0)

    steps = data.get("steps") or []
    if not steps:
        raise SystemExit("No steps found in steps.json")

    arm_ctrl = ArmSdkController(args.iface, arm, cmd_hz, kp, kd)
    if not args.no_seed:
        arm_ctrl.seed_from_lowstate()

    current_pose_deg: Dict[str, float] = {}

    for idx, step in enumerate(steps, start=1):
        name = step.get("name") or f"step_{idx}"
        duration = float(step.get("duration", 1.5))
        hold = float(step.get("hold", 0.0))

        angles = step.get("angles") or {}
        descriptions = step.get("descriptions") or []
        if descriptions:
            angles_from_desc = _parse_descriptions(descriptions, arm)
            angles.update(angles_from_desc)

        if not angles:
            raise SystemExit(f"Step '{name}' has no angles or descriptions")

        current_pose_deg = _build_pose(arm, angles, current_pose_deg)
        indexed_pose = _pose_to_indexed(arm, current_pose_deg)

        if args.dry_run:
            print(f"[{name}] duration={duration}s hold={hold}s angles={current_pose_deg}")
            continue

        print(f"Executing {name} (duration={duration}s hold={hold}s)")
        arm_ctrl.ramp_to_pose(indexed_pose, duration, args.easing)
        arm_ctrl.hold_pose(indexed_pose, hold)


if __name__ == "__main__":
    main()
