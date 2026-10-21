#!/usr/bin/env python3
"""
dev_stand.py — Record and replay the G1 standup FSM sequence in developer mode.

Three sub-commands:

  record   Passively observe rt/lowstate + LocoClient while the robot boots
           normally. Snapshots joint positions at three FSM phases:
             FSM_DAMPING (1)  — robot limp, motors kd-only
             FSM_PREPARE (4)  — standup in progress
             FSM_WALK    (501) — balanced stand
           Saves to a JSON file for later replay.

  replay   Load a saved snapshot and re-execute the same pose sequence via
           rt/lowcmd (developer mode, all 29 joints). Before any command is
           sent, a safety table is shown and confirmation is required.
           Cross-checks every phase transition against ll_sdk._ramp_publish:
             RAMP>PLANNED  (yellow) — ll_sdk default ramp (0.35 rad/s) would
                           take longer than the planned ramp; this script uses
                           smoothstep interpolation instead
             VEL UNSAFE    (red)    — average velocity > 4× ll_sdk ramp speed
             LIMIT         (red)    — value outside URDF hard limits

  show     Print saved snapshot without connecting to the robot.

Typical workflow
----------------
  1. Hang robot, power on, let it sit in damp mode (FSM 1).
  2. python dev_stand.py record --output dev_stand_snapshot.json
  3. Boot robot normally (it will move through FSM 4 → FSM 501 automatically).
  4. Record exits when all three phases are captured.
  5. python dev_stand.py replay --input dev_stand_snapshot.json --dry-run
  6. python dev_stand.py replay --input dev_stand_snapshot.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SCRIPTS_DIR = Path(__file__).resolve().parent
_MODULES_DIR  = _SCRIPTS_DIR.parent
for _p in (_MODULES_DIR, _SCRIPTS_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from dds_env import ensure_channel_factory_initialized, ensure_cyclonedds_environment
ensure_cyclonedds_environment()

# ── FSM IDs (from mode_control.py / sdk_boot.py) ─────────────────────────────
FSM_ZERO_TORQUE = 0
FSM_DAMPING     = 1    # kp=0, kd only — robot limp
FSM_SIT         = 3
FSM_PREPARE     = 4    # SetFsmId(4) standup initiation
FSM_WALK        = 501  # balanced stand / locomotion

FSM_NAMES: Dict[int, str] = {
    FSM_ZERO_TORQUE: "zero_torque",
    FSM_DAMPING:     "damping",
    FSM_SIT:         "sit",
    FSM_PREPARE:     "prepare",
    FSM_WALK:        "walk",
}

# Phases to record, in expected boot order
RECORD_PHASES: List[Tuple[int, str]] = [
    (FSM_DAMPING, "damp"),
    (FSM_PREPARE, "prepare"),
    (FSM_WALK,    "walk"),
]

# ── Joint layout — matches low_level_commands.BODY_JOINTS exactly ─────────────
# (name, motor_idx, lo_rad, hi_rad)
BODY_JOINTS: List[Tuple[str, int, float, float]] = [
    ("left_leg.hip_pitch",       0,  -2.5307,   2.8798),
    ("left_leg.hip_roll",        1,  -0.5236,   2.9671),
    ("left_leg.hip_yaw",         2,  -2.7576,   2.7576),
    ("left_leg.knee",            3,  -0.087267, 2.8798),
    ("left_leg.ankle_pitch",     4,  -0.87267,  0.5236),
    ("left_leg.ankle_roll",      5,  -0.2618,   0.2618),
    ("right_leg.hip_pitch",      6,  -2.5307,   2.8798),
    ("right_leg.hip_roll",       7,  -2.9671,   0.5236),
    ("right_leg.hip_yaw",        8,  -2.7576,   2.7576),
    ("right_leg.knee",           9,  -0.087267, 2.8798),
    ("right_leg.ankle_pitch",   10,  -0.87267,  0.5236),
    ("right_leg.ankle_roll",    11,  -0.2618,   0.2618),
    ("waist.yaw",               12,  -2.618,    2.618),
    ("waist.roll",              13,  -0.52,     0.52),
    ("waist.pitch",             14,  -0.52,     0.52),
    ("left_arm.shoulder_pitch", 15,  -3.0892,   2.6704),
    ("left_arm.shoulder_roll",  16,  -1.5882,   2.2515),
    ("left_arm.shoulder_yaw",   17,  -2.618,    2.618),
    ("left_arm.elbow",          18,  -1.0472,   2.0944),
    ("left_arm.wrist_roll",     19,  -1.9722,   1.9722),
    ("left_arm.wrist_pitch",    20,  -1.6144,   1.6144),
    ("left_arm.wrist_yaw",      21,  -1.6144,   1.6144),
    ("right_arm.shoulder_pitch",22,  -3.0892,   2.6704),
    ("right_arm.shoulder_roll", 23,  -2.2515,   1.5882),
    ("right_arm.shoulder_yaw",  24,  -2.618,    2.618),
    ("right_arm.elbow",         25,  -1.0472,   2.0944),
    ("right_arm.wrist_roll",    26,  -1.9722,   1.9722),
    ("right_arm.wrist_pitch",   27,  -1.6144,   1.6144),
    ("right_arm.wrist_yaw",     28,  -1.6144,   1.6144),
]
N_JOINTS = len(BODY_JOINTS)   # 29

# PD gains from low_level_commands.py BODY_KP / BODY_KD
_KP: List[float] = [
    60, 60, 60, 100, 40, 40,        # left leg
    60, 60, 60, 100, 40, 40,        # right leg
    60, 40, 40,                      # waist
    40, 40, 40, 40, 40, 40, 40,     # left arm
    40, 40, 40, 40, 40, 40, 40,     # right arm
]
_KD: List[float] = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

# ── ll_sdk safety constants (from ll_sdk._ramp_publish / ik_move_EE defaults)
_LLSDK_RAMP_SPEED = 0.35        # rad/s — ll_sdk._ramp_publish default
_LLSDK_RAMP_HZ    = 50.0        # Hz
_LLSDK_RAMP_STEP  = _LLSDK_RAMP_SPEED / _LLSDK_RAMP_HZ   # 0.007 rad/step
_UNSAFE_VEL       = _LLSDK_RAMP_SPEED * 4.0               # 1.4 rad/s


# ── ANSI helpers ──────────────────────────────────────────────────────────────
_R = "\033[91m"; _Y = "\033[93m"; _G = "\033[92m"
_C = "\033[96m"; _B = "\033[1m";  _X = "\033[0m"


def _c(txt: str, code: str, use_color: bool) -> str:
    return f"{code}{txt}{_X}" if use_color else txt


# ── Data model ────────────────────────────────────────────────────────────────

@dataclass
class PhaseSnapshot:
    name: str                  # "damp" | "prepare" | "walk"
    fsm_id: int
    fsm_mode: Optional[int]
    mode_machine: int          # echoed from rt/lowstate at capture time
    q: List[float]             # 29 joint angles in BODY_JOINTS motor_idx order
    timestamp: str             # ISO-8601 UTC

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(d: dict) -> "PhaseSnapshot":
        return PhaseSnapshot(**d)


@dataclass
class DevStandRecord:
    recorded_at: str
    iface: str
    phases: List[PhaseSnapshot]

    def save(self, path: Path) -> None:
        path.write_text(json.dumps({
            "recorded_at": self.recorded_at,
            "iface": self.iface,
            "phases": [p.to_dict() for p in self.phases],
        }, indent=2))

    @staticmethod
    def load(path: Path) -> "DevStandRecord":
        d = json.loads(path.read_text())
        return DevStandRecord(
            recorded_at=d["recorded_at"],
            iface=d.get("iface", "eth0"),
            phases=[PhaseSnapshot.from_dict(p) for p in d["phases"]],
        )


# ── Lowstate subscriber ───────────────────────────────────────────────────────

class LowStateReader:
    """Thread-safe subscriber for rt/lowstate — returns (q[29], mode_machine)."""

    def __init__(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self._lock = threading.Lock()
        self._q: Optional[List[float]] = None
        self._mm: int = 0
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._cb, 200)

    def _cb(self, msg) -> None:
        try:
            q  = [float(msg.motor_state[idx].q) for _, idx, _, _ in BODY_JOINTS]
            mm = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        with self._lock:
            self._q  = q
            self._mm = mm

    def snapshot(self) -> Optional[Tuple[List[float], int]]:
        with self._lock:
            return (list(self._q), self._mm) if self._q is not None else None

    def wait(self, timeout: float = 5.0) -> Tuple[List[float], int]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.snapshot()
            if s is not None:
                return s
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


# ── Math ──────────────────────────────────────────────────────────────────────

def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# ── Safety analysis ───────────────────────────────────────────────────────────

def _phase_table(
    phase: PhaseSnapshot,
    prev_q: Optional[List[float]],
    ramp_s: float,
    use_color: bool,
) -> Tuple[List[str], bool, bool]:
    """
    Build a per-joint table for one phase.
    Returns (lines, has_warnings, has_errors).

    Columns:
      motor_idx | joint name | target | Δq | avg vel | ll_sdk ramp ms | status
    """
    lines: List[str] = []
    has_warnings = has_errors = False
    sep = "─" * 98
    hdr = (f"  {'Idx':>3}  {'Joint':.<28}  "
           f"{'Target':>10}  {'Δq':>10}  "
           f"{'Vel rad/s':>10}  {'Ramp@0.35':>10}  Status")
    lines += [_c(sep, _C, use_color), _c(hdr, _B, use_color), _c(sep, _C, use_color)]

    for local_i, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
        target = phase.q[local_i]
        color  = _G
        flags: List[str] = []
        d_str = v_str = r_str = "N/A"

        if prev_q is not None:
            delta    = target - prev_q[local_i]
            avg_vel  = abs(delta) / ramp_s
            ramp_ms  = abs(delta) / _LLSDK_RAMP_SPEED * 1000.0   # time at ll_sdk speed

            d_str = f"{delta:+.5f}"
            v_str = f"{avg_vel:.3f}"
            r_str = f"{ramp_ms:.1f} ms"

            if ramp_ms > ramp_s * 1000.0:
                flags.append("RAMP>PLANNED")
                has_warnings = True
                color = _Y

            if avg_vel > _UNSAFE_VEL:
                flags.append(f"VEL UNSAFE (>{_UNSAFE_VEL:.1f})")
                has_errors = True
                color = _R

        if target < lo or target > hi:
            flags.append(f"LIMIT [{lo:.3f},{hi:.3f}]")
            has_errors = True
            color = _R

        status = ", ".join(flags) if flags else "OK"
        row = (f"  {motor_idx:>3}  {name:<28}  "
               f"{target:>10.5f}  {d_str:>10}  "
               f"{v_str:>10}  {r_str:>10}  {status}")
        lines.append(_c(row, color, use_color))

    lines.append(_c(sep, _C, use_color))
    return lines, has_warnings, has_errors


def _ramp_summary(
    phase: PhaseSnapshot,
    prev_q: Optional[List[float]],
    ramp_s: float,
    use_color: bool,
) -> str:
    if prev_q is None:
        return "  (no previous phase — this is the starting pose)"
    diffs = [abs(phase.q[i] - prev_q[i]) for i in range(N_JOINTS)]
    worst = int(max(range(N_JOINTS), key=lambda i: diffs[i]))
    max_dq = diffs[worst]
    llsdk_t_s = max_dq / _LLSDK_RAMP_SPEED
    ok = llsdk_t_s <= ramp_s
    msg = (f"  max |Δq|={max_dq:.5f} rad  ({BODY_JOINTS[worst][0]})  "
           f"ll_sdk_ramp={llsdk_t_s*1000:.1f} ms  "
           f"planned_ramp={ramp_s*1000:.0f} ms  ")
    if ok:
        msg += "→ planned ramp slower than ll_sdk ramp — safe"
    else:
        msg += (f"→ ll_sdk._ramp_publish would overrun by {(llsdk_t_s - ramp_s)*1000:.1f} ms; "
                f"replay uses smoothstep (not ll_sdk ramp)")
    return _c(msg, _G if ok else _Y, use_color)


def _prompt(msg: str) -> str:
    try:
        return input(msg).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return "q"


# ── record ────────────────────────────────────────────────────────────────────

def cmd_record(args: argparse.Namespace) -> None:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.loco.g1_loco_api import (
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
    )

    use_color = not args.no_color and sys.stdout.isatty()
    out_path  = Path(args.output)

    print("Connecting …")
    ensure_channel_factory_initialized(int(args.domain), str(args.iface))

    loco = LocoClient()
    loco.SetTimeout(5.0)
    loco.Init()

    state_rdr = LowStateReader()
    print("Waiting for first lowstate …")
    state_rdr.wait(5.0)
    print("  Ready.\n")

    def poll_fsm() -> Tuple[Optional[int], Optional[int]]:
        try:
            c0, r0 = loco._Call(ROBOT_API_ID_LOCO_GET_FSM_ID,   "{}")
            c1, r1 = loco._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, "{}")
            fid  = int(json.loads(r0).get("data")) if c0 == 0 and r0 else None
            fmod = int(json.loads(r1).get("data")) if c1 == 0 and r1 else None
            return fid, fmod
        except Exception:
            return None, None

    print(_c("Recording FSM transitions.", _B, use_color))
    print("  Boot the robot normally.  This script watches passively.")
    for fid, name in RECORD_PHASES:
        print(f"    waiting for: {name.upper()} (FSM {fid})")
    print("  Press Ctrl-C to stop early.\n")

    captured: Dict[int, PhaseSnapshot] = {}
    remaining = list(RECORD_PHASES)

    try:
        while remaining:
            fid, fmod = poll_fsm()
            snap = state_rdr.snapshot()
            if snap is None:
                time.sleep(0.1)
                continue

            q, mm = snap
            target_fsm, target_name = remaining[0]

            if fid == target_fsm:
                ts = datetime.now(timezone.utc).isoformat()
                phase = PhaseSnapshot(
                    name=target_name,
                    fsm_id=target_fsm,
                    fsm_mode=fmod,
                    mode_machine=mm,
                    q=list(q),
                    timestamp=ts,
                )
                captured[target_fsm] = phase
                remaining.pop(0)

                print(_c(f"  ✓  {target_name.upper()} (FSM {target_fsm})", _G, use_color))
                print(f"     mode_machine={mm}  fsm_mode={fmod}  ts={ts}")
                print(f"     q[0:6] = {[round(v, 4) for v in q[:6]]} …")
                print()

            time.sleep(0.08)

    except KeyboardInterrupt:
        print("\nRecording stopped early.")

    if not captured:
        print("No phases captured — nothing saved.")
        return

    rec = DevStandRecord(
        recorded_at=datetime.now(timezone.utc).isoformat(),
        iface=args.iface,
        phases=list(captured.values()),
    )
    rec.save(out_path)
    n = len(captured)
    print(f"Saved {n}/{len(RECORD_PHASES)} phases → {out_path}")
    if n < len(RECORD_PHASES):
        missing = [name for fid, name in RECORD_PHASES if fid not in captured]
        print(f"  Missing: {missing}  (re-run record on next boot to complete)")


# ── show ──────────────────────────────────────────────────────────────────────

def cmd_show(args: argparse.Namespace) -> None:
    use_color = not args.no_color and sys.stdout.isatty()
    path = Path(args.input)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    rec = DevStandRecord.load(path)
    print(f"Recorded : {rec.recorded_at}")
    print(f"Interface: {rec.iface}")
    print(f"Phases   : {len(rec.phases)}\n")

    for phase in rec.phases:
        print(_c(f"── {phase.name.upper()}  (FSM {phase.fsm_id}  "
                 f"mode_machine={phase.mode_machine}  fsm_mode={phase.fsm_mode})", _B, use_color))
        print(f"   timestamp: {phase.timestamp}")
        print(f"   {'Idx':>3}  {'Joint':<28}  {'q (rad)':>11}  {'limits':>20}")
        for local_i, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
            v = phase.q[local_i]
            near = v <= lo + 0.005 or v >= hi - 0.005
            color = _Y if near else ""
            print(_c(f"   {motor_idx:>3}  {name:<28}  {v:>11.5f}  [{lo:.3f}, {hi:.3f}]",
                     color, use_color and near))
        print()


# ── replay ────────────────────────────────────────────────────────────────────

def cmd_replay(args: argparse.Namespace) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    # Use the SDK Python CRC when the native architecture library is missing.
    if not getattr(CRC, "_wbc_python_crc_fallback", False):
        _crc_sdk_init = CRC.__init__
        def _crc_init_python_fallback(self):
            try:
                _crc_sdk_init(self)
            except OSError as exc:
                if "crc_" not in str(exc) or "No such file" not in str(exc):
                    raise
                self.platform = "Python"
        CRC.__init__ = _crc_init_python_fallback
        CRC._wbc_python_crc_fallback = True

    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    use_color = not args.no_color and sys.stdout.isatty()
    in_path   = Path(args.input)
    ramp_s    = float(args.ramp_s)
    rate_hz   = float(args.rate_hz)
    dt        = 1.0 / rate_hz

    # ── Load snapshot ─────────────────────────────────────────────────────────
    if not in_path.exists():
        print(f"Snapshot not found: {in_path}", file=sys.stderr)
        sys.exit(1)
    rec = DevStandRecord.load(in_path)
    print(f"Loaded from {in_path}  (recorded {rec.recorded_at})")
    for p in rec.phases:
        print(f"  {p.name:<12}  FSM {p.fsm_id:<5}  mode_machine={p.mode_machine}")
    print()

    # ── Full pre-flight safety analysis ───────────────────────────────────────
    legend = [
        ("ll_sdk ramp speed (ik_move_EE / _ramp_publish default)",
         f"{_LLSDK_RAMP_SPEED} rad/s  step={_LLSDK_RAMP_STEP*1000:.1f} mrad @ {_LLSDK_RAMP_HZ:.0f} Hz", _C),
        ("This replay ramp",
         f"smoothstep over {ramp_s:.1f}s per phase at {rate_hz:.0f} Hz (not ll_sdk._ramp_publish)", _C),
        ("RAMP>PLANNED  (yellow)",
         "ll_sdk default ramp would overrun planned time; smoothstep used instead", _Y),
        ("VEL UNSAFE    (red)",
         f"avg vel > {_UNSAFE_VEL:.1f} rad/s (4× ll_sdk default)", _R),
        ("LIMIT         (red)",
         "joint value outside URDF hard limits", _R),
    ]
    print(_c("Safety legend", _B, use_color))
    for label, val, col in legend:
        print(f"  {_c(label, col, use_color)}: {val}")
    print()

    prev_q: Optional[List[float]] = None
    any_errors = False
    for phase in rec.phases:
        print(_c(f"Phase: {phase.name.upper()}  (FSM {phase.fsm_id} — {FSM_NAMES.get(phase.fsm_id, '?')})",
                 _B, use_color))
        table, w, e = _phase_table(phase, prev_q, ramp_s, use_color)
        for line in table:
            print(line)
        print(_ramp_summary(phase, prev_q, ramp_s, use_color))
        print()
        if e:
            any_errors = True
        prev_q = list(phase.q)

    if any_errors:
        print(_c("One or more phases have errors. Carefully review before proceeding.", _R, use_color))
    print()

    # ── Dry-run exits here ────────────────────────────────────────────────────
    if args.dry_run:
        print("--dry-run: analysis complete, not connecting.")
        return

    # ── Dev-mode warning + initial confirmation ───────────────────────────────
    print(_c("WARNING: replay publishes to rt/lowcmd (developer mode, all 29 joints).", _Y, use_color))
    print(_c("         MotionSwitcher is released — robot goes limp while switching.", _Y, use_color))
    print(_c("         Robot must be on a hanger or otherwise externally supported.", _Y, use_color))
    print()
    if _prompt("Type 'yes' to connect and begin, anything else to abort: ") != "yes":
        print("Aborted.")
        sys.exit(0)
    print()

    # ── DDS init + dev mode ───────────────────────────────────────────────────
    ChannelFactoryInitialize(int(args.domain), str(args.iface))

    crc     = CRC()
    pub     = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    cmd_msg = unitree_hg_msg_dds__LowCmd_()
    cmd_msg.mode_pr = 0

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    state_rdr = LowStateReader()

    print("Releasing MotionSwitcher …")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        code, data = msc.CheckMode()
        if code != 0:
            raise RuntimeError(f"MotionSwitcherClient.CheckMode() failed: code={code}")
        if not (data or {}).get("name"):
            break
        msc.ReleaseMode()
        time.sleep(0.5)
    else:
        raise TimeoutError("Could not release MotionSwitcher within 10 s.")

    print("Waiting for lowstate …")
    q_now, mm_now = state_rdr.wait(5.0)
    print(f"  Connected.  mode_machine={mm_now}\n")

    # ── Publisher + ramp helpers ──────────────────────────────────────────────

    def publish(q: List[float], mode_machine: int) -> None:
        cmd_msg.mode_machine = int(mode_machine)
        for local_i, (_, motor_idx, _, _) in enumerate(BODY_JOINTS):
            mc = cmd_msg.motor_cmd[motor_idx]
            mc.mode = 1
            mc.q    = float(q[local_i])
            mc.dq   = 0.0
            mc.tau  = 0.0
            mc.kp   = float(_KP[local_i])
            mc.kd   = float(_KD[local_i])
        cmd_msg.crc = crc.Crc(cmd_msg)
        pub.Write(cmd_msg)

    def ramp_phase(q_start: List[float], q_end: List[float], mm: int) -> None:
        steps = max(1, int(ramp_s * rate_hz))
        for step in range(steps + 1):
            alpha  = smoothstep(step / steps)
            q_step = [lerp(q_start[i], q_end[i], alpha) for i in range(N_JOINTS)]
            publish(q_step, mm)
            if step < steps:
                time.sleep(dt)

    # ── Phase loop ────────────────────────────────────────────────────────────
    # Seed from current measured position — avoids jump on first phase.
    current_q, _ = state_rdr.wait(3.0)
    prev_replay: List[float] = list(current_q)

    print(_c("Current joint positions (lowstate, before any command):", _B, use_color))
    print(f"  {'Idx':>3}  {'Joint':<28}  {'q (rad)':>11}")
    for local_i, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
        print(f"  {motor_idx:>3}  {name:<28}  {current_q[local_i]:>11.5f}")
    print()

    for phase_idx, phase in enumerate(rec.phases):
        fsm_label = FSM_NAMES.get(phase.fsm_id, str(phase.fsm_id))
        hdr = (f"── Phase {phase_idx + 1}/{len(rec.phases)}: "
               f"{phase.name.upper()}  (FSM {phase.fsm_id} — {fsm_label})")
        print(_c(f"\n{hdr}", _B, use_color))

        # Safety table for this transition
        table, warns, errs = _phase_table(phase, prev_replay, ramp_s, use_color)
        for line in table:
            print(line)
        print(_ramp_summary(phase, prev_replay, ramp_s, use_color))

        # Explicit rt/lowcmd preview — what will be written to the DDS topic
        print()
        print(_c(f"  rt/lowcmd commands (topic: rt/lowcmd, {N_JOINTS} joints):", _C, use_color))
        print(f"  {'motor_idx':>9}  {'mode':>5}  {'kp':>6}  {'kd':>4}  "
              f"{'target q':>11}  {'Δ from now':>12}  {'dq':>4}  {'tau':>5}")
        for local_i, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
            tgt   = phase.q[local_i]
            delta = tgt - prev_replay[local_i]
            lim_err = tgt < lo or tgt > hi
            col = _R if lim_err else (_Y if abs(delta) > 0.15 else "")
            row = (f"  [{motor_idx:>2}] {name:<24}  mode=1  "
                   f"kp={_KP[local_i]:>5.0f}  kd={_KD[local_i]:>3.0f}  "
                   f"{tgt:>11.5f}  {delta:>+12.5f}  dq=0  tau=0")
            print(_c(row, col, use_color and bool(col)))
        print()

        # Confirmation
        if errs:
            prompt_txt = _c("  ERRORS present — [y]es (unsafe!) / [s]kip / [q]uit > ", _R, use_color)
        elif warns:
            prompt_txt = _c("  Ramp warnings — [y]es / [s]kip / [q]uit > ", _Y, use_color)
        else:
            prompt_txt = "  [y]es / [s]kip / [q]uit > "

        ans = _prompt(prompt_txt)
        if ans in ("q", "quit"):
            print("Quit.")
            sys.exit(0)
        if ans in ("s", "skip"):
            print(f"  Skipped {phase.name}.")
            continue
        if ans not in ("y", "yes", ""):
            print("  Unrecognised input — skipping.")
            continue

        # Execute
        q_live, mm_live = state_rdr.wait(3.0)
        print(f"  Ramping to {phase.name.upper()} over {ramp_s:.1f} s …")
        ramp_phase(q_live, phase.q, mm_live)
        print(_c(f"  Phase complete: {phase.name.upper()}", _G, use_color))
        prev_replay = list(phase.q)

        if phase_idx < len(rec.phases) - 1:
            nxt = rec.phases[phase_idx + 1].name
            r = _prompt(f"  Press Enter to continue → {nxt.upper()}, or q to quit > ")
            if r in ("q", "quit"):
                print("Quit.")
                sys.exit(0)

    print(_c("\nAll phases complete.", _G, use_color))


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--no-color", action="store_true")

    sub = ap.add_subparsers(dest="command", required=True)

    # record
    rc = sub.add_parser("record", help="Passively record FSM phase snapshots")
    rc.add_argument("--iface",  default="eth0")
    rc.add_argument("--domain", type=int, default=0)
    rc.add_argument("--output", default="dev_stand_snapshot.json",
                    help="Output file (default: dev_stand_snapshot.json)")

    # replay
    rp = sub.add_parser("replay", help="Re-execute snapshot in developer mode via rt/lowcmd")
    rp.add_argument("--iface",    default="eth0")
    rp.add_argument("--domain",   type=int, default=0)
    rp.add_argument("--input",    default="dev_stand_snapshot.json")
    rp.add_argument("--ramp-s",   type=float, default=3.0,
                    help="Smoothstep ramp duration per phase in seconds (default 3.0)")
    rp.add_argument("--rate-hz",  type=float, default=50.0,
                    help="rt/lowcmd publish rate in Hz (default 50)")
    rp.add_argument("--dry-run",  action="store_true",
                    help="Show safety analysis without connecting")

    # show
    sh = sub.add_parser("show", help="Print saved snapshot")
    sh.add_argument("--input", default="dev_stand_snapshot.json")

    args = ap.parse_args()
    {"record": cmd_record, "replay": cmd_replay, "show": cmd_show}[args.command](args)


if __name__ == "__main__":
    main()
