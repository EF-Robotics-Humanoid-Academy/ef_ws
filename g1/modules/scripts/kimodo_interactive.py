#!/usr/bin/env python3
"""
kimodo_interactive.py — Interactive Kimodo motion generation and replay on the G1.

Pipeline:
  1. Stand     Load dev_stand_snapshot.json, enter dev mode, ramp robot to the
               saved "walk" (FSM 501) standing pose via rt/lowcmd.
  2. Prompt    Enter a REPL: type a text description of the motion you want.
  3. Generate  Kimodo generates the motion in-memory (no temp files).
  4. Review    Safety analysis: max Δq per frame, limit violations, ramp warnings.
  5. Confirm   [r]eplay / [f]rames (detailed view) / [n]ew prompt / [q]uit
  6. Replay    Execute frame-by-frame through the same LLSdk session.
  7. Return    After replay, ramp back to standing pose and loop.

Usage
-----
    python kimodo_interactive.py --snapshot dev_stand_snapshot.json [options]

Options
-------
    --snapshot PATH   dev_stand JSON snapshot (required for standup phase)
    --fps FPS         Motion playback frame rate — must match Kimodo generation (default 30)
    --num-frames N    Frames to generate per prompt (default 90 = 3s at 30fps)
    --steps N         Kimodo denoising steps — more = better quality, slower (default 50)
    --iface IFACE     DDS network interface (default eth0)
    --domain ID       DDS domain ID — 0 real robot, 1 sim (default 0)
    --ramp-s S        Standup/return ramp duration in seconds (default 3.0)
    --rate-hz HZ      rt/lowcmd publish rate (default 50)
    --no-color        Disable ANSI colour
    --dry-run         Connect to robot but skip Kimodo — useful for testing standup
    --no-robot        Skip robot connection entirely (unitree_sdk2py is never imported) —
                       useful for testing Kimodo generation and safety analysis up to the
                       replay confirmation without the SDK/robot set up
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import sys
import threading
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SCRIPTS_DIR = Path(__file__).resolve().parent
_MODULES_DIR  = _SCRIPTS_DIR.parent
for _p in (_MODULES_DIR, _SCRIPTS_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from dds_env import ensure_channel_factory_initialized, ensure_cyclonedds_environment
ensure_cyclonedds_environment()

# ── Joint layout (from low_level_commands.BODY_JOINTS / ll_sdk.py) ────────────
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

_DEFAULT_TEXT_ENCODER_URL = "http://127.0.0.1:9550/"
_LOW_RAM_TEXT_ENCODER_GIB = 20.0

_KP: List[float] = [60,60,60,100,40,40, 60,60,60,100,40,40, 60,40,40,
                     40,40,40,40,40,40,40, 40,40,40,40,40,40,40]
_KD: List[float] = [1,1,1,2,1,1, 1,1,1,2,1,1, 1,1,1,
                     1,1,1,1,1,1,1, 1,1,1,1,1,1,1]

# ── Kimodo G1Skeleton34 → robot joint mapping ─────────────────────────────────
# Key: Kimodo bone index (from bone_order_names_with_parents)
# Value: (robot motor_idx, joint_axis)
# Axis is the revolute axis in parent frame (from ll_sdk._LEG_CHAIN / arm_fk URDF chain)
_KIMODO_MAP: Dict[int, Tuple[int, Tuple[int,int,int]]] = {
    # left leg
    1:  (0,  (0,1,0)),   # hip_pitch   Y
    2:  (1,  (1,0,0)),   # hip_roll    X
    3:  (2,  (0,0,1)),   # hip_yaw     Z
    4:  (3,  (0,1,0)),   # knee        Y
    5:  (4,  (0,1,0)),   # ankle_pitch Y
    6:  (5,  (1,0,0)),   # ankle_roll  X
    # right leg
    8:  (6,  (0,1,0)),
    9:  (7,  (1,0,0)),
    10: (8,  (0,0,1)),
    11: (9,  (0,1,0)),
    12: (10, (0,1,0)),
    13: (11, (1,0,0)),
    # waist
    15: (12, (0,0,1)),   # waist_yaw   Z
    16: (13, (1,0,0)),   # waist_roll  X
    17: (14, (0,1,0)),   # waist_pitch Y
    # left arm
    18: (15, (0,1,0)),   # shoulder_pitch Y
    19: (16, (1,0,0)),   # shoulder_roll  X
    20: (17, (0,0,1)),   # shoulder_yaw   Z
    21: (18, (0,1,0)),   # elbow          Y
    22: (19, (1,0,0)),   # wrist_roll     X
    23: (20, (0,1,0)),   # wrist_pitch    Y
    24: (21, (0,0,1)),   # wrist_yaw      Z
    # right arm
    26: (22, (0,1,0)),
    27: (23, (1,0,0)),
    28: (24, (0,0,1)),
    29: (25, (0,1,0)),
    30: (26, (1,0,0)),
    31: (27, (0,1,0)),
    32: (28, (0,0,1)),
}
# Build reverse: motor_idx → local list index
_MOTOR_TO_LOCAL = {motor_idx: local_i
                   for local_i, (_, motor_idx, _, _) in enumerate(BODY_JOINTS)}

# ── ll_sdk safety constants ────────────────────────────────────────────────────
_RAMP_SPEED  = 0.35    # rad/s — ll_sdk._ramp_publish / ik_move_EE default
_UNSAFE_VEL  = _RAMP_SPEED * 4.0   # 1.4 rad/s

# ── ANSI ──────────────────────────────────────────────────────────────────────
_R="\033[91m"; _Y="\033[93m"; _G="\033[92m"
_C="\033[96m"; _B="\033[1m";  _X="\033[0m"

def _c(t:str, code:str, on:bool)->str:
    return f"{code}{t}{_X}" if on else t

def _hr(use_color:bool)->str:
    return _c("─"*80, _C, use_color)


# ── Rotation matrix → joint angle ─────────────────────────────────────────────

def _rot_to_angle(R: np.ndarray, axis: Tuple[int,int,int]) -> float:
    """Extract signed rotation angle from 3×3 matrix R around a unit axis.

    Uses the skew-symmetric part of R to recover sin(θ) and the trace for
    cos(θ), then returns atan2.  Valid for any single-axis revolute joint.
    """
    K = (R - R.T) * 0.5               # skew part = sin(θ)[axis]×
    ax, ay, az = axis
    # sin(θ) = K[2,1]*ax + K[0,2]*ay + K[1,0]*az  (verified for X, Y, Z axes)
    sin_t = K[2,1]*ax + K[0,2]*ay + K[1,0]*az
    cos_t = (np.trace(R) - 1.0) * 0.5
    return float(np.arctan2(sin_t, cos_t))


# ── Motion conversion (Kimodo output → robot joint angles) ────────────────────

def kimodo_to_joint_frames(local_rot_mats: np.ndarray) -> np.ndarray:
    """Convert Kimodo local_rot_mats to robot joint angle frames.

    Parameters
    ----------
    local_rot_mats : ndarray, shape (n_frames, 34, 3, 3)
        Per-frame, per-joint local rotation matrices from Kimodo model output.

    Returns
    -------
    ndarray, shape (n_frames, 29)
        Joint angles in robot motor order, ready for rt/lowcmd.
    """
    n_frames = local_rot_mats.shape[0]
    out = np.zeros((n_frames, N_JOINTS), dtype=np.float64)
    for kimodo_idx, (motor_idx, axis) in _KIMODO_MAP.items():
        local_i = _MOTOR_TO_LOCAL[motor_idx]
        R_seq   = local_rot_mats[:, kimodo_idx]       # (n_frames, 3, 3)
        for f in range(n_frames):
            out[f, local_i] = _rot_to_angle(R_seq[f], axis)
    return out


# ── Lowstate reader ────────────────────────────────────────────────────────────

class LowStateReader:
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
            q  = [float(msg.motor_state[idx].q) for _,idx,_,_ in BODY_JOINTS]
            mm = int(getattr(msg, "mode_machine", 0))
        except Exception:
            return
        with self._lock:
            self._q = q; self._mm = mm

    def snapshot(self) -> Optional[Tuple[List[float], int]]:
        with self._lock:
            return (list(self._q), self._mm) if self._q else None

    def wait(self, timeout:float=5.0) -> Tuple[List[float], int]:
        d = time.monotonic() + timeout
        while time.monotonic() < d:
            s = self.snapshot()
            if s: return s
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")


# ── Math helpers ───────────────────────────────────────────────────────────────

def smoothstep(x:float)->float:
    x = max(0.0, min(1.0, x))
    return x*x*(3.0-2.0*x)

def lerp(a:float, b:float, t:float)->float:
    return a + (b-a)*t


# ── Safety analysis ────────────────────────────────────────────────────────────

@dataclass
class MotionStats:
    n_frames: int
    fps: float
    duration_s: float
    max_delta: float           # max |Δq| between consecutive frames
    max_delta_joint: str       # joint name
    limit_violations: int      # frames×joints with out-of-limit values
    ramp_warnings: int         # frames where ramp>frame_interval
    unsafe_frames: int         # frames where vel > _UNSAFE_VEL


def analyse_motion(frames: np.ndarray, fps: float) -> MotionStats:
    """Quick summary stats over all frames."""
    n = len(frames)
    fi = 1.0 / fps
    max_dq = 0.0
    max_j = 0
    lim_viols = ramp_warns = unsafe_f = 0

    for f in range(n):
        frame_unsafe = False
        for local_i, (_, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
            val = frames[f, local_i]
            if val < lo or val > hi:
                lim_viols += 1
            if f > 0:
                dq = abs(val - frames[f-1, local_i])
                vel = dq / fi
                if dq > max_dq:
                    max_dq = dq; max_j = local_i
                if dq / _RAMP_SPEED * 1000 > fi * 1000:
                    ramp_warns += 1
                if vel > _UNSAFE_VEL and not frame_unsafe:
                    unsafe_f += 1; frame_unsafe = True

    return MotionStats(
        n_frames=n, fps=fps, duration_s=n/fps,
        max_delta=max_dq,
        max_delta_joint=BODY_JOINTS[max_j][0],
        limit_violations=lim_viols,
        ramp_warnings=ramp_warns,
        unsafe_frames=unsafe_f,
    )


def print_summary(stats: MotionStats, use_color: bool) -> None:
    ok = stats.limit_violations == 0 and stats.unsafe_frames == 0
    print(f"  Frames      : {stats.n_frames} @ {stats.fps:.0f} fps = {stats.duration_s:.1f} s")
    print(f"  Max |Δq|    : {stats.max_delta:.5f} rad  ({stats.max_delta_joint})")
    viols_color = _R if stats.limit_violations else _G
    print(f"  Limit viols : {_c(str(stats.limit_violations), viols_color, use_color)}")
    ramp_color  = _Y if stats.ramp_warnings else _G
    print(f"  RAMP>FRAME  : {_c(str(stats.ramp_warnings), ramp_color, use_color)}  joint-frames  "
          f"(ramp speed {_RAMP_SPEED} rad/s needs >1 frame here — playback will lag fps, ramp still enforced)")
    uf_color    = _Y if stats.unsafe_frames else _G
    print(f"  HIGH VEL    : {_c(str(stats.unsafe_frames), uf_color, use_color)}  frames  "
          f"(implied source vel > {_UNSAFE_VEL:.1f} rad/s — likely fast motion or bad data; ramp still caps it)")


def print_frame_table(frames: np.ndarray, fps: float, use_color: bool,
                      start: int = 0, end: Optional[int] = None) -> None:
    fi = 1.0 / fps
    end = end or len(frames)
    sep = "─" * 90
    hdr = f"  {'F':>5}  {'Idx':>3}  {'Joint':<28}  {'q':>10}  {'Δq':>10}  {'Vel':>8}  Status"
    print(_c(sep, _C, use_color))
    print(_c(hdr, _B, use_color))
    print(_c(sep, _C, use_color))
    for f in range(start, end):
        for local_i, (name, motor_idx, lo, hi) in enumerate(BODY_JOINTS):
            val = frames[f, local_i]
            flags = []
            color = ""
            d_str = v_str = "     N/A"
            if f > 0:
                dq  = val - frames[f-1, local_i]
                vel = abs(dq) / fi
                d_str = f"{dq:+.5f}"
                v_str = f"{vel:.3f}"
                if abs(dq) / _RAMP_SPEED * 1000 > fi * 1000:
                    flags.append("RAMP>FRAME"); color = _Y
                if vel > _UNSAFE_VEL:
                    flags.append("HIGH-VEL"); color = _Y
            if val < lo or val > hi:
                flags.append("LIMIT"); color = _R
            status = ", ".join(flags) if flags else "ok"
            row = (f"  {f+1:>5}  {motor_idx:>3}  {name:<28}  "
                   f"{val:>10.5f}  {d_str:>10}  {v_str:>8}  {status}")
            print(_c(row, color, use_color and bool(color)))
    print(_c(sep, _C, use_color))


# ── Robot publisher ────────────────────────────────────────────────────────────

class RobotSession:
    """Owns the DDS session, dev mode, and all rt/lowcmd publishing."""

    def __init__(self, iface:str, domain:int) -> None:
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

        ChannelFactoryInitialize(int(domain), str(iface))
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._msc = MotionSwitcherClient()
        self._msc.SetTimeout(5.0)
        self._msc.Init()
        self.state = LowStateReader()

    def enter_dev_mode(self) -> None:
        print("Releasing MotionSwitcher …")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            code, data = self._msc.CheckMode()
            if code != 0:
                raise RuntimeError(f"CheckMode failed: {code}")
            if not (data or {}).get("name"):
                break
            self._msc.ReleaseMode()
            time.sleep(0.5)
        else:
            raise TimeoutError("Could not release MotionSwitcher.")
        print("  Dev mode active.")

    def publish(self, q: List[float], mm: int) -> None:
        self._cmd.mode_machine = int(mm)
        for local_i, (_, motor_idx, _, _) in enumerate(BODY_JOINTS):
            mc = self._cmd.motor_cmd[motor_idx]
            mc.mode = 1; mc.q = float(q[local_i])
            mc.dq = 0.0; mc.tau = 0.0
            mc.kp = float(_KP[local_i]); mc.kd = float(_KD[local_i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def ramp(self, q_start:List[float], q_end:List[float],
             duration_s:float, rate_hz:float=50.0) -> None:
        steps = max(1, int(duration_s * rate_hz))
        dt    = 1.0 / rate_hz
        _, mm = self.state.wait(3.0)
        for step in range(steps + 1):
            alpha  = smoothstep(step / steps)
            q_step = [lerp(q_start[i], q_end[i], alpha) for i in range(N_JOINTS)]
            self.publish(q_step, mm)
            if step < steps: time.sleep(dt)

    def _ramp_publish(self, q_start: List[float], q_target: List[float], mm: int,
                       *, speed_rad_s: float, rate_hz: float) -> None:
        """Speed-limited move from q_start to q_target — mirrors ll_sdk._ramp_publish
        so Kimodo-generated motion is capped by the same safety ramp as every
        other move_* helper in this codebase, instead of jumping straight to
        each frame's raw target."""
        speed = float(speed_rad_s)
        rate  = max(1.0, float(rate_hz))
        if speed <= 0.0:
            self.publish(q_target, mm)
            return
        max_delta = max(abs(a - b) for a, b in zip(q_start, q_target))
        steps = max(1, int(math.ceil(max_delta / max(1e-6, speed / rate))))
        if steps <= 1:
            self.publish(q_target, mm)
            return
        dt = 1.0 / rate
        for step_idx in range(1, steps + 1):
            alpha  = step_idx / steps
            q_step = [a + alpha * (b - a) for a, b in zip(q_start, q_target)]
            self.publish(q_step, mm)
            if step_idx < steps:
                time.sleep(dt)

    def play_frames(self, frames: np.ndarray, fps: float, *,
                     ramp_speed_rad_s: float = _RAMP_SPEED,
                     ramp_rate_hz: float = 50.0) -> None:
        """Execute pre-converted frames (shape n_frames × 29) at fps.

        Frame-to-frame motion is speed-limited via the same ramp algorithm as
        ll_sdk._ramp_publish, so a joint can never be commanded faster than
        the ll_sdk safety ramp speed. If a frame's delta needs more time than
        1/fps, this call blocks longer and playback falls behind the source
        fps for that frame — it is never sent as an instant jump.
        """
        fi = 1.0 / fps
        q_prev, mm = self.state.wait(3.0)
        for f in range(len(frames)):
            t0 = time.monotonic()
            q_target = list(frames[f])
            self._ramp_publish(q_prev, q_target, mm,
                                speed_rad_s=ramp_speed_rad_s, rate_hz=ramp_rate_hz)
            q_prev = q_target
            elapsed = time.monotonic() - t0
            rem = fi - elapsed
            if rem > 0: time.sleep(rem)


# ── Snapshot loading (from dev_stand.py format) ────────────────────────────────

def load_walk_pose(snapshot_path: str) -> Optional[List[float]]:
    """Return the 'walk' phase q[29] from a dev_stand snapshot, or None."""
    try:
        data = json.loads(Path(snapshot_path).read_text())
        for phase in data.get("phases", []):
            if phase.get("name") == "walk":
                return list(phase["q"])
    except Exception as exc:
        print(f"  Warning: could not load snapshot ({exc})")
    return None


# ── Prompt helpers ─────────────────────────────────────────────────────────────

def _input(prompt:str) -> str:
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print(); return "q"


def _system_ram_gib() -> Optional[float]:
    """Return total system RAM in GiB without importing optional deps."""
    try:
        pages = os.sysconf("SC_PHYS_PAGES")
        page_size = os.sysconf("SC_PAGE_SIZE")
        return (pages * page_size) / (1024 ** 3)
    except (AttributeError, OSError, ValueError):
        return None


def _text_encoder_service_reachable(url: str, timeout_s: float = 1.0) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=timeout_s):
            return True
    except Exception:
        return False


def _gradio_client_installed() -> bool:
    return importlib.util.find_spec("gradio_client") is not None


def _configure_text_encoder_env(args: argparse.Namespace) -> None:
    if args.text_encoder_mode:
        os.environ["TEXT_ENCODER_MODE"] = args.text_encoder_mode
    if args.text_encoder_url:
        os.environ["TEXT_ENCODER_URL"] = args.text_encoder_url


def _guard_kimodo_text_encoder(args: argparse.Namespace) -> None:
    """Avoid Kimodo's default local 8B text-encoder fallback on low-RAM hosts."""
    if args.allow_local_text_encoder:
        return

    mode = os.environ.get("TEXT_ENCODER_MODE", "auto").lower()
    url = os.environ.get("TEXT_ENCODER_URL", _DEFAULT_TEXT_ENCODER_URL)
    has_quantized_encoder = bool(os.environ.get("LLM2VEC_QUANTIZE"))
    total_ram_gib = _system_ram_gib()
    low_ram = total_ram_gib is not None and total_ram_gib < _LOW_RAM_TEXT_ENCODER_GIB

    if mode in ("api", "auto") and not _gradio_client_installed():
        raise SystemExit(
            "Kimodo TEXT_ENCODER_MODE needs the Python package gradio_client for "
            "the TextEncoderAPI client, but it is not installed in this environment.\n\n"
            "Install it in the active venv, for example:\n"
            "  uv pip install gradio_client\n\n"
            "Then run with a reachable text encoder service:\n"
            "  TEXT_ENCODER_MODE=api TEXT_ENCODER_URL=http://HOST:9550/ "
            "python kimodo_interactive.py --no-robot"
        )

    if mode == "api":
        return
    if mode == "auto" and _text_encoder_service_reachable(url):
        return
    if has_quantized_encoder:
        return
    if not low_ram:
        return

    ram_text = f"{total_ram_gib:.1f} GiB" if total_ram_gib is not None else "unknown RAM"
    raise SystemExit(
        "Kimodo is about to use the local LLM2Vec/Llama-3-8B text encoder on "
        f"this {ram_text} system. That path typically exceeds the G1 Jetson's "
        "memory and gets killed by the kernel.\n\n"
        "Use one of these instead:\n"
        "  1. Start a text encoder service on a larger machine, then run:\n"
        "     TEXT_ENCODER_MODE=api TEXT_ENCODER_URL=http://HOST:9550/ "
        "python kimodo_interactive.py --no-robot\n"
        "  2. If you really have a quantized local Kimodo install, export "
        "LLM2VEC_QUANTIZE=nf4 (or fp4/int8) before running.\n"
        "  3. To force the risky local fallback anyway, pass "
        "--allow-local-text-encoder."
    )


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--snapshot",  default="dev_stand_snapshot.json",
                    help="dev_stand JSON snapshot for standing pose (default: dev_stand_snapshot.json)")
    ap.add_argument("--fps",       type=float, default=30.0)
    ap.add_argument("--num-frames",type=int,   default=90,
                    help="Frames to generate per prompt (default 90 = 3s @ 30fps)")
    ap.add_argument("--steps",     type=int,   default=50,
                    help="Kimodo denoising steps (default 50)")
    ap.add_argument("--iface",     default="eth0")
    ap.add_argument("--domain",    type=int, default=0)
    ap.add_argument("--ramp-s",    type=float, default=3.0,
                    help="Standup / return-to-stand ramp duration (default 3.0s)")
    ap.add_argument("--rate-hz",   type=float, default=50.0)
    ap.add_argument("--no-color",  action="store_true")
    ap.add_argument("--dry-run",   action="store_true",
                    help="Stand and prompt loop, but skip Kimodo inference and replay")
    ap.add_argument("--no-robot",  action="store_true",
                    help="Skip robot connection entirely (unitree_sdk2py is never imported). "
                         "Runs Kimodo generation and the safety analysis normally, stopping at "
                         "the replay confirmation instead of actually publishing to rt/lowcmd. "
                         "Useful for testing motion generation without the SDK/robot set up.")
    ap.add_argument("--text-encoder-mode", choices=("api", "auto", "local"),
                    help="Set Kimodo TEXT_ENCODER_MODE before loading the model. "
                         "Use 'api' to avoid the local Llama-3-8B fallback.")
    ap.add_argument("--text-encoder-url",
                    help=f"Set Kimodo TEXT_ENCODER_URL (default: {_DEFAULT_TEXT_ENCODER_URL})")
    ap.add_argument("--allow-local-text-encoder", action="store_true",
                    help="Allow Kimodo to load the local LLM2Vec/Llama-3-8B text encoder even "
                         "on low-RAM hosts. This can OOM-kill Python on the G1 Jetson.")
    args = ap.parse_args()

    uc = not args.no_color and sys.stdout.isatty()
    _configure_text_encoder_env(args)

    # ── Load Kimodo ────────────────────────────────────────────────────────────
    kimodo_model = None
    if not args.dry_run:
        print("Loading Kimodo model …")
        try:
            _guard_kimodo_text_encoder(args)
            from kimodo.model import load_model
            kimodo_model = load_model("nvidia/Kimodo-G1-SEED-v1")
            print("  Kimodo ready.\n")
        except Exception as exc:
            print(_c(f"  Could not load Kimodo: {exc}", _R, uc))
            print(_c("  Continuing in --dry-run mode (standup only).", _Y, uc))
            args.dry_run = True

    # ── Load standing pose ─────────────────────────────────────────────────────
    walk_pose = load_walk_pose(args.snapshot)
    if walk_pose:
        print(f"Standing pose: loaded from {args.snapshot}  (walk / FSM 501)")
    else:
        print(_c("No walk pose found — robot will hold its current pose as the base.", _Y, uc))

    # ── Connect to robot ───────────────────────────────────────────────────────
    bot: Optional[RobotSession] = None
    stand_q: Optional[List[float]] = None

    if args.no_robot:
        print()
        print(_c("  --no-robot: skipping robot connection (unitree_sdk2py is not imported).", _Y, uc))
        print(_c("  Generation and safety analysis will run normally; replay stops at the "
                 "confirmation prompt.", _Y, uc))
        stand_q = list(walk_pose) if walk_pose else None
    else:
        print("\nConnecting to robot …")
        print(_c("WARNING: entering developer mode (rt/lowcmd). Robot must be on hanger.", _Y, uc))
        if _input("Type 'yes' to connect: ") != "yes":
            print("Aborted."); sys.exit(0)

        bot = RobotSession(args.iface, args.domain)
        bot.enter_dev_mode()
        q_now, _ = bot.state.wait(5.0)
        print(f"  lowstate received ({N_JOINTS} joints).")

        # ── Stand up ───────────────────────────────────────────────────────────
        if walk_pose:
            print(f"\nRamping to standing pose over {args.ramp_s:.1f}s …")
            bot.ramp(q_now, walk_pose, args.ramp_s, args.rate_hz)
            print(_c("  Standing. Robot is now in developer-mode standing pose.", _G, uc))
            stand_q = list(walk_pose)
        else:
            stand_q = list(q_now)
            print(_c("  Holding current pose as standing reference.", _Y, uc))

    # ── REPL ───────────────────────────────────────────────────────────────────
    print()
    print(_c("─"*60, _C, uc))
    print(_c("  Kimodo interactive  —  type a motion description", _B, uc))
    print(_c("  Commands: q=quit  s=skip  r=return-to-stand", _C, uc))
    print(_c("─"*60, _C, uc))

    last_frames: Optional[np.ndarray] = None
    last_prompt: str = ""

    while True:
        print()
        raw = _input(_c("[motion] > ", _B, uc))
        if not raw or raw.lower() in ("q", "quit", "exit"):
            break
        if raw.lower() in ("r", "return"):
            if bot is None:
                print(_c("  [no-robot] No robot connected — nothing to return to stand.", _Y, uc))
                continue
            print(f"Returning to standing pose …")
            q_cur, _ = bot.state.wait(3.0)
            bot.ramp(q_cur, stand_q, args.ramp_s, args.rate_hz)
            print(_c("  Standing.", _G, uc)); continue

        # ── Generate ───────────────────────────────────────────────────────────
        prompt_text = raw
        if args.dry_run:
            print(_c("  [dry-run] Skipping Kimodo inference.", _Y, uc))
            continue

        print(f"  Generating: \"{prompt_text}\"  "
              f"({args.num_frames} frames @ {args.fps:.0f} fps = "
              f"{args.num_frames/args.fps:.1f}s, {args.steps} steps) …")
        t0 = time.monotonic()
        try:
            result = kimodo_model(
                text=[prompt_text],
                num_frames=args.num_frames,
                fps=args.fps,
                num_denoising_steps=args.steps,
                as_numpy=True,
            )
        except Exception as exc:
            print(_c(f"  Generation failed: {exc}", _R, uc))
            continue
        gen_s = time.monotonic() - t0

        # Unpack — model returns batched output; squeeze batch dim
        lrm = result["local_rot_mats"]
        if lrm.ndim == 5:
            lrm = lrm[0]          # (n_frames, 34, 3, 3)
        elif lrm.ndim == 4 and lrm.shape[0] == 1:
            lrm = lrm[0]

        print(f"  Generated in {gen_s:.1f}s.  Converting to joint angles …")
        frames = kimodo_to_joint_frames(lrm)   # (n_frames, 29)
        last_frames = frames
        last_prompt = prompt_text

        # ── Safety summary ─────────────────────────────────────────────────────
        stats = analyse_motion(frames, args.fps)
        print()
        print(_c(f"  Motion summary: \"{prompt_text}\"", _B, uc))
        print(_hr(uc))
        print_summary(stats, uc)
        print(_hr(uc))

        # ── Action loop ────────────────────────────────────────────────────────
        while True:
            choice = _input(
                _c("  [r]eplay  [f]rames  [n]ew prompt  [s]kip  [q]uit > ", _C, uc)
            ).lower()

            if choice in ("q", "quit"):
                print("Quit.")
                if bot is not None:
                    # ramp back to stand before exit
                    q_cur, _ = bot.state.wait(3.0)
                    bot.ramp(q_cur, stand_q, args.ramp_s, args.rate_hz)
                sys.exit(0)

            elif choice in ("n", "new", "s", "skip", ""):
                break

            elif choice.startswith("f"):
                # Optional range: "f 1 30" shows frames 1-30
                parts = choice.split()
                fr_start = int(parts[1]) - 1 if len(parts) > 1 else 0
                fr_end   = int(parts[2])     if len(parts) > 2 else min(30, len(frames))
                print(f"\n  Frames {fr_start+1}–{fr_end}:")
                print_frame_table(frames, args.fps, uc, fr_start, fr_end)

            elif choice in ("r", "replay"):
                if stats.limit_violations:
                    warn = _input(_c(
                        f"  {stats.limit_violations} limit violations present. "
                        "Proceed anyway? [y]es/[n]o > ", _R, uc)).lower()
                    if warn not in ("y", "yes"):
                        continue

                if bot is None:
                    print(_c(
                        f"  [no-robot] Reached replay confirmation for {stats.n_frames} frames "
                        f"({stats.duration_s:.1f}s) — no robot connected, skipping actual playback.",
                        _Y, uc,
                    ))
                    break

                print(f"\n  Playing {stats.n_frames} frames "
                      f"({stats.duration_s:.1f}s) …")
                bot.play_frames(frames, args.fps, ramp_rate_hz=args.rate_hz)
                print(_c("  Replay complete.", _G, uc))

                # After replay: offer return to stand
                ret = _input("  Return to standing pose? [y]es/[n]o > ").lower()
                if ret in ("y", "yes", ""):
                    q_cur, _ = bot.state.wait(3.0)
                    print(f"  Ramping back over {args.ramp_s:.1f}s …")
                    bot.ramp(q_cur, stand_q, args.ramp_s, args.rate_hz)
                    print(_c("  Standing.", _G, uc))
                break

    # ── Clean exit: return to stand ────────────────────────────────────────────
    print("\nExiting …")
    if bot is not None:
        q_cur, _ = bot.state.wait(3.0)
        if q_cur != stand_q:
            print(f"Ramping back to standing pose …")
            bot.ramp(q_cur, stand_q, args.ramp_s, args.rate_hz)
    print("Done.")


if __name__ == "__main__":
    main()
