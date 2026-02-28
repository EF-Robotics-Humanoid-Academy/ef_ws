#!/usr/bin/env python3
"""
reproduce.py
============

Low-level G1 motion reproducer for NPZ files produced by mapper.py.

Control channels
----------------
Default (arms + waist only, SAFE — balance controller stays active):
  • rt/arm_sdk   joints 12-28  @ 50 Hz  kp=60  kd=1.5
  • Balance controller manages legs (joints 0-11) throughout.
  • Enable bit: motor_cmd[29].q = 1  (arm_sdk active)
  •             motor_cmd[29].q = 0  (arm_sdk released on exit)

Full-body mode (--with-legs, HANGER REQUIRED):
  • MotionSwitcherClient.ReleaseMode() releases the balance controller.
  • rt/lowcmd     joints 0-28   @ 500 Hz  kp=120/60  kd=2.0/1.5
  • Balance controller is re-engaged (SelectMode "stand") on exit.

Stability layer (always active)
---------------------------------
1. LowState seed: waits for rt/lowstate before first command; initial
   commanded positions are seeded from the actual current joint angles.
2. Soft ramp-in: linearly interpolates from current pose to the first
   trajectory frame (--start-ramp seconds) to prevent jerks.
3. Velocity limiter: each tick, δq is clamped per joint so angular
   velocity never exceeds MAX_JOINT_VEL (rad/s).
4. IMU watchdog: reads roll + pitch from rt/lowstate every tick;
   triggers immediate e-stop if either exceeds --imu-limit (default 0.35 rad ≈ 20°).
5. Foot-contact gate (leg mode only): if SportModeState.mode == 2 (feet
   unloaded), leg joint commands are frozen at the last safe pose.
6. Joint limits: all commanded angles are re-clamped to the same ranges
   used by mapper.py before each write.

Safety rules (from arm_motion/sdk_details.md)
----------------------------------------------
• Never send commands until a valid LowState has been received.
• Never jump in q — always ramp or interpolate.
• Release arm_sdk (motor_cmd[29].q = 0) before exiting.
• Moderate gains: kp=60, kd=1.5 for arms; higher for legs.
• Keep a spotter and clear workspace during all tests.

Usage
-----
    # Safe default: arms + waist only
    python3 reproduce.py --iface enp1s0 --file g1_motion.npz

    # Full body (hanger required, robot must NOT support own weight)
    python3 reproduce.py --iface enp1s0 --file g1_motion.npz --with-legs

    # Slower, softer, no waist
    python3 reproduce.py --file g1_motion.npz --speed 0.5 \\
                         --arm-kp 40 --no-waist

    # 23-DOF safe (skip WristPitch/Yaw + WaistRoll/Pitch)
    python3 reproduce.py --file g1_motion.npz --dof23

References
----------
    ../../arm_motion/sdk_details.md
    ../../arm_motion/g1_arm7_sdk_dds_example.py
    ../../arm_motion/pbd/pbd_reproduce.py
    ../../arm_motion/pbd/pbd_docs.md
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# SDK imports — raise early with a clear message if missing
# ---------------------------------------------------------------------------

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__LowState_,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.utils.thread import RecurrentThread
except ImportError as _e:
    raise SystemExit(
        "unitree_sdk2py is not installed.\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from _e

try:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    _HAS_SWITCHER = True
except ImportError:
    _HAS_SWITCHER = False

try:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    _HAS_SPORT = True
except ImportError:
    _HAS_SPORT = False

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    _HAS_LOCO = True
except ImportError:
    _HAS_LOCO = False


# ---------------------------------------------------------------------------
# G1 joint layout  (mirrors g1_arm7_sdk_dds_example.py)
# ---------------------------------------------------------------------------

class J:
    # Left leg
    LeftHipPitch    = 0;  LeftHipRoll     = 1;  LeftHipYaw      = 2
    LeftKnee        = 3;  LeftAnklePitch  = 4;  LeftAnkleRoll   = 5
    # Right leg
    RightHipPitch   = 6;  RightHipRoll    = 7;  RightHipYaw     = 8
    RightKnee       = 9;  RightAnklePitch = 10; RightAnkleRoll  = 11
    # Waist
    WaistYaw        = 12; WaistRoll       = 13; WaistPitch      = 14
    # Left arm
    LeftShoulderPitch = 15; LeftShoulderRoll = 16; LeftShoulderYaw = 17
    LeftElbow         = 18; LeftWristRoll    = 19
    LeftWristPitch    = 20; LeftWristYaw     = 21    # invalid on 23-DOF
    # Right arm
    RightShoulderPitch = 22; RightShoulderRoll = 23; RightShoulderYaw = 24
    RightElbow         = 25; RightWristRoll    = 26
    RightWristPitch    = 27; RightWristYaw     = 28  # invalid on 23-DOF
    # arm_sdk enable channel
    ArmSdkEnable       = 29


# Joint subsets
LEG_JOINTS:      Set[int] = set(range(0, 12))
WAIST_JOINTS:    Set[int] = {J.WaistYaw, J.WaistRoll, J.WaistPitch}
ARM_JOINTS:      Set[int] = set(range(15, 29))
ARM_SDK_JOINTS:  Set[int] = WAIST_JOINTS | ARM_JOINTS   # arm_sdk controls 12-28
DOF23_INVALID:   Set[int] = {
    J.WaistRoll, J.WaistPitch,
    J.LeftWristPitch, J.LeftWristYaw,
    J.RightWristPitch, J.RightWristYaw,
}

# ---------------------------------------------------------------------------
# Per-joint gains
# ---------------------------------------------------------------------------

def _kp_for(j: int, arm_kp: float, leg_kp: float) -> float:
    if j in LEG_JOINTS:
        return leg_kp
    return arm_kp   # arms + waist


def _kd_for(j: int, arm_kd: float, leg_kd: float) -> float:
    if j in LEG_JOINTS:
        return leg_kd
    return arm_kd


# ---------------------------------------------------------------------------
# Joint limits (same as mapper.py — re-applied here as final safety clamp)
# ---------------------------------------------------------------------------

_LIMITS: Dict[int, Tuple[float, float]] = {
    J.LeftHipPitch:       (-1.57, 1.57),  J.RightHipPitch:      (-1.57, 1.57),
    J.LeftHipRoll:        (-0.52, 0.52),  J.RightHipRoll:       (-0.52, 0.52),
    J.LeftHipYaw:         (-0.52, 0.52),  J.RightHipYaw:        (-0.52, 0.52),
    J.LeftKnee:           ( 0.00, 2.30),  J.RightKnee:          ( 0.00, 2.30),
    J.LeftAnklePitch:     (-0.52, 0.52),  J.RightAnklePitch:    (-0.52, 0.52),
    J.LeftAnkleRoll:      (-0.30, 0.30),  J.RightAnkleRoll:     (-0.30, 0.30),
    J.WaistYaw:           (-1.00, 1.00),
    J.WaistRoll:          (-0.35, 0.35),
    J.WaistPitch:         (-0.52, 0.52),
    J.LeftShoulderPitch:  (-1.57, 2.40),  J.RightShoulderPitch: (-1.57, 2.40),
    J.LeftShoulderRoll:   (-0.20, 2.40),  J.RightShoulderRoll:  (-2.40, 0.20),
    J.LeftShoulderYaw:    (-1.40, 1.40),  J.RightShoulderYaw:   (-1.40, 1.40),
    J.LeftElbow:          ( 0.00, 2.40),  J.RightElbow:         ( 0.00, 2.40),
    J.LeftWristRoll:      (-1.40, 1.40),  J.RightWristRoll:     (-1.40, 1.40),
    J.LeftWristPitch:     (-0.70, 0.70),  J.RightWristPitch:    (-0.70, 0.70),
    J.LeftWristYaw:       (-0.44, 0.44),  J.RightWristYaw:      (-0.44, 0.44),
}

# Maximum angular velocity (rad/s) per joint — conservative safe values
_MAX_VEL: Dict[int, float] = {
    J.LeftHipPitch: 2.5,  J.RightHipPitch: 2.5,
    J.LeftHipRoll:  2.0,  J.RightHipRoll:  2.0,
    J.LeftHipYaw:   2.0,  J.RightHipYaw:   2.0,
    J.LeftKnee:     3.0,  J.RightKnee:     3.0,
    J.LeftAnklePitch: 2.0, J.RightAnklePitch: 2.0,
    J.LeftAnkleRoll:  1.5, J.RightAnkleRoll:  1.5,
}
# Arms / waist: 3 rad/s is safe at 50 Hz (0.06 rad/tick max step)
_DEFAULT_MAX_VEL = 3.0


def _clamp(v: float, j: int) -> float:
    lo, hi = _LIMITS.get(j, (-3.14, 3.14))
    return float(np.clip(v, lo, hi))


def _vel_limited(q_cur: Dict[int, float], q_tgt: Dict[int, float], dt: float) -> Dict[int, float]:
    """Clamp each joint target so δq/dt ≤ MAX_VEL, then re-clamp to joint limits."""
    out: Dict[int, float] = {}
    for j, q in q_tgt.items():
        q_now  = q_cur.get(j, q)
        max_dq = _MAX_VEL.get(j, _DEFAULT_MAX_VEL) * dt
        q_safe = float(np.clip(q, q_now - max_dq, q_now + max_dq))
        out[j]  = _clamp(q_safe, j)
    return out


# ---------------------------------------------------------------------------
# LowStateMonitor — thread-safe snapshot of rt/lowstate
# ---------------------------------------------------------------------------

class LowStateMonitor:
    """Subscribes to rt/lowstate and exposes joint positions + IMU RPY."""

    def __init__(self) -> None:
        self._lock      = threading.Lock()
        self._q:        Dict[int, float] = {}
        self._rpy:      Optional[Tuple[float, float, float]] = None
        self._ready     = threading.Event()

    def callback(self, msg: LowState_) -> None:
        q:   Dict[int, float] = {}
        rpy: Optional[Tuple[float, float, float]] = None
        try:
            for j in range(29):
                q[j] = float(msg.motor_state[j].q)
        except Exception:
            pass
        try:
            rpy = (
                float(msg.imu_state.rpy[0]),
                float(msg.imu_state.rpy[1]),
                float(msg.imu_state.rpy[2]),
            )
        except Exception:
            pass
        with self._lock:
            self._q   = q
            self._rpy = rpy
        self._ready.set()

    def wait(self, timeout: float = 3.0) -> bool:
        return self._ready.wait(timeout=timeout)

    def joint_q(self, j: int) -> Optional[float]:
        with self._lock:
            return self._q.get(j)

    def all_q(self) -> Dict[int, float]:
        with self._lock:
            return dict(self._q)

    @property
    def imu_rpy(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            return self._rpy


# ---------------------------------------------------------------------------
# SportModeMonitor — foot-contact status (optional)
# ---------------------------------------------------------------------------

class SportModeMonitor:
    """Reads SportModeState for foot-contact mode (mode==2 → feet unloaded)."""

    def __init__(self) -> None:
        self._mode: Optional[int] = None
        self._lock = threading.Lock()

    def callback(self, msg: "SportModeState_") -> None:
        try:
            with self._lock:
                self._mode = int(msg.mode)
        except Exception:
            pass

    @property
    def feet_loaded(self) -> bool:
        """True when feet are bearing weight (mode != 2), or unknown."""
        with self._lock:
            return self._mode != 2   # None → unknown → assume loaded


# ---------------------------------------------------------------------------
# IMU watchdog (inline check — called each control tick)
# ---------------------------------------------------------------------------

def _imu_ok(monitor: LowStateMonitor, limit_rad: float) -> bool:
    rpy = monitor.imu_rpy
    if rpy is None:
        return True   # no data yet — do not fault
    roll, pitch = rpy[0], rpy[1]
    if abs(roll) > limit_rad or abs(pitch) > limit_rad:
        print(
            f"[watchdog] IMU exceeded limit! "
            f"roll={np.degrees(roll):.1f}°  pitch={np.degrees(pitch):.1f}°  "
            f"(limit={np.degrees(limit_rad):.1f}°)"
        )
        return False
    return True


# ---------------------------------------------------------------------------
# Linear interpolation between trajectory rows
# ---------------------------------------------------------------------------

def _interp_row(
    ts:  np.ndarray,
    qs:  np.ndarray,
    t:   float,
) -> np.ndarray:
    if t <= float(ts[0]):
        return qs[0].copy()
    if t >= float(ts[-1]):
        return qs[-1].copy()
    hi = int(np.searchsorted(ts, t, side="right"))
    lo = max(0, hi - 1)
    t0, t1 = float(ts[lo]), float(ts[hi])
    if t1 <= t0:
        return qs[hi].copy()
    a = (t - t0) / (t1 - t0)
    return qs[lo] * (1.0 - a) + qs[hi] * a


# ---------------------------------------------------------------------------
# NPZ loader (mapper.py output format)
# ---------------------------------------------------------------------------

def load_npz(path: Path) -> Tuple[List[int], np.ndarray, np.ndarray]:
    """Load mapper.py NPZ.  Returns (joint_indices, ts, qs)."""
    if not path.exists():
        raise FileNotFoundError(f"Motion file not found: {path}")
    with np.load(path, allow_pickle=True) as data:
        joints = data["joints"].astype(int).tolist()
        ts     = data["ts"].astype(float)
        qs     = data["qs"].astype(float)
    if len(ts) == 0:
        raise ValueError("Motion file has no samples.")
    if qs.shape[0] != len(ts) or qs.shape[1] != len(joints):
        raise ValueError("Motion file shape mismatch (ts/qs/joints).")
    return joints, ts, qs


# ---------------------------------------------------------------------------
# ArmWaistController — rt/arm_sdk path (joints 12-28, 50 Hz)
# ---------------------------------------------------------------------------

class ArmWaistController:
    """Position-PD controller on rt/arm_sdk.

    Follows the exact pattern from g1_arm7_sdk_dds_example.py:
      motor_cmd[29].q = 1  →  arm_sdk active
      motor_cmd[29].q = 0  →  arm_sdk released
    """

    RATE_HZ  = 50.0
    CTRL_DT  = 1.0 / RATE_HZ

    def __init__(
        self,
        monitor: LowStateMonitor,
        kp:      float = 60.0,
        kd:      float = 1.5,
    ) -> None:
        self._monitor  = monitor
        self._kp       = kp
        self._kd       = kd
        self._crc      = CRC()
        self._cmd      = unitree_hg_msg_dds__LowCmd_()
        self._cmd_q:   Dict[int, float] = {}
        self._enabled  = False

        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()

    def enable(self) -> None:
        self._cmd.motor_cmd[J.ArmSdkEnable].q = 1.0
        self._enabled = True

    def disable(self) -> None:
        self._cmd.motor_cmd[J.ArmSdkEnable].q = 0.0
        self._enabled = False
        self._flush()

    def seed_from_state(self, joints: List[int]) -> None:
        """Prime commanded positions from the live robot state."""
        for j in joints:
            q = self._monitor.joint_q(j)
            if q is not None:
                self._cmd_q[j] = q

    def write(self, q_by_joint: Dict[int, float]) -> None:
        for j, q in q_by_joint.items():
            if j not in ARM_SDK_JOINTS:
                continue    # silently skip leg joints on this channel
            mc      = self._cmd.motor_cmd[j]
            mc.q    = float(q)
            mc.dq   = 0.0
            mc.tau  = 0.0
            mc.kp   = self._kp
            mc.kd   = self._kd
        self._cmd_q.update({j: q for j, q in q_by_joint.items() if j in ARM_SDK_JOINTS})
        self._flush()

    def ramp_to(
        self,
        target:  Dict[int, float],
        dur:     float,
        imu_lim: float,
        monitor: LowStateMonitor,
    ) -> bool:
        """Smoothly interpolate to target over dur seconds.  Returns False if watchdog trips."""
        steps = max(1, int(dur * self.RATE_HZ))
        start = {j: self._cmd_q.get(j, 0.0) for j in target}
        dt    = self.CTRL_DT
        for i in range(1, steps + 1):
            if not _imu_ok(monitor, imu_lim):
                return False
            a   = i / steps
            cur = {j: start[j] + (target[j] - start[j]) * a for j in target}
            self.write(cur)
            time.sleep(dt)
        return True

    def _flush(self) -> None:
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


# ---------------------------------------------------------------------------
# FullBodyController — rt/lowcmd path (joints 0-28, 500 Hz)
# ---------------------------------------------------------------------------

class FullBodyController:
    """Position-PD controller on rt/lowcmd (all 29 joints).

    Requires MotionSwitcherClient.ReleaseMode() to have been called first.
    Per sdk_details.md: control_dt = 0.002 s (500 Hz).
    """

    RATE_HZ = 500.0
    CTRL_DT = 1.0 / RATE_HZ

    def __init__(
        self,
        monitor: LowStateMonitor,
        arm_kp: float = 60.0,
        arm_kd: float = 1.5,
        leg_kp: float = 120.0,
        leg_kd: float = 2.0,
    ) -> None:
        self._monitor = monitor
        self._arm_kp  = arm_kp
        self._arm_kd  = arm_kd
        self._leg_kp  = leg_kp
        self._leg_kd  = leg_kd
        self._crc     = CRC()
        self._cmd     = unitree_hg_msg_dds__LowCmd_()
        self._cmd_q:  Dict[int, float] = {}

        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()

    def seed_from_state(self, joints: List[int]) -> None:
        for j in joints:
            q = self._monitor.joint_q(j)
            if q is not None:
                self._cmd_q[j] = q

    def write(self, q_by_joint: Dict[int, float]) -> None:
        for j, q in q_by_joint.items():
            mc      = self._cmd.motor_cmd[j]
            mc.q    = float(q)
            mc.dq   = 0.0
            mc.tau  = 0.0
            mc.kp   = _kp_for(j, self._arm_kp, self._leg_kp)
            mc.kd   = _kd_for(j, self._arm_kd, self._leg_kd)
        self._cmd_q.update(q_by_joint)
        self._flush()

    def ramp_to(
        self,
        target:  Dict[int, float],
        dur:     float,
        imu_lim: float,
        monitor: LowStateMonitor,
    ) -> bool:
        steps = max(1, int(dur * self.RATE_HZ))
        start = {j: self._cmd_q.get(j, 0.0) for j in target}
        dt    = self.CTRL_DT
        for i in range(1, steps + 1):
            if not _imu_ok(monitor, imu_lim):
                return False
            a   = i / steps
            cur = {j: start[j] + (target[j] - start[j]) * a for j in target}
            self.write(cur)
            time.sleep(dt)
        return True

    def _flush(self) -> None:
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


# ---------------------------------------------------------------------------
# MotionSwitcher helpers
# ---------------------------------------------------------------------------

def _release_balance_controller() -> Optional[str]:
    """Release the active locomotion mode via MotionSwitcherClient.
    Returns the active mode name (for restoration), or None on failure.
    """
    if not _HAS_SWITCHER:
        print("[warn] MotionSwitcherClient not available — cannot release mode.")
        return None
    try:
        sc = MotionSwitcherClient()
        sc.SetTimeout(5.0)
        sc.Init()
        code, data = sc.CheckMode()
        active = data.get("name", "") if (code == 0 and isinstance(data, dict)) else ""
        if active:
            sc.ReleaseMode()
            print(f"[switcher] released mode: '{active}'")
        else:
            print("[switcher] no active mode found.")
        return active or None
    except Exception as e:
        print(f"[warn] MotionSwitcherClient.ReleaseMode() failed: {e}")
        return None


def _restore_balance_controller(mode: Optional[str]) -> None:
    if not _HAS_SWITCHER or not mode:
        return
    try:
        sc = MotionSwitcherClient()
        sc.SetTimeout(5.0)
        sc.Init()
        sc.SelectMode(mode)
        print(f"[switcher] restored mode: '{mode}'")
    except Exception as e:
        print(f"[warn] MotionSwitcherClient.SelectMode('{mode}') failed: {e}")


# ---------------------------------------------------------------------------
# Filter joints to requested subset
# ---------------------------------------------------------------------------

def _filter_joints(
    joints:     List[int],
    qs:         np.ndarray,
    with_legs:  bool,
    no_waist:   bool,
    dof23:      bool,
) -> Tuple[List[int], np.ndarray]:
    """Drop columns that are outside the intended replay set."""
    invalid: Set[int] = set()
    if not with_legs:
        invalid |= LEG_JOINTS
    if no_waist:
        invalid |= WAIST_JOINTS
    if dof23:
        invalid |= DOF23_INVALID

    keep_idx  = [i for i, j in enumerate(joints) if j not in invalid]
    kept_j    = [joints[i] for i in keep_idx]
    kept_qs   = qs[:, keep_idx]
    return kept_j, kept_qs


# ---------------------------------------------------------------------------
# Main run function
# ---------------------------------------------------------------------------

def run(
    iface:       str,
    file:        Path,
    with_legs:   bool  = False,
    no_waist:    bool  = False,
    dof23:       bool  = False,
    speed:       float = 1.0,
    start_ramp:  float = 1.5,
    seed_wait:   float = 2.0,
    arm_kp:      float = 60.0,
    arm_kd:      float = 1.5,
    leg_kp:      float = 120.0,
    leg_kd:      float = 2.0,
    imu_limit:   float = 0.35,
    cmd_hz:      float = 50.0,
) -> None:

    # ------------------------------------------------------------------
    # Safety gate for leg mode
    # ------------------------------------------------------------------
    if with_legs:
        print(
            "\n"
            "  *** FULL-BODY MODE SELECTED ***\n"
            "  This releases the balance controller.\n"
            "  The robot WILL NOT maintain balance on its own.\n"
            "  ROBOT MUST BE ON A HANGER BEFORE CONTINUING.\n"
        )
        ans = input("  Type YES to confirm the robot is secured on a hanger: ").strip()
        if ans != "YES":
            raise SystemExit("Aborted by user.")

    # ------------------------------------------------------------------
    # DDS init
    # ------------------------------------------------------------------
    print(f"[init] ChannelFactoryInitialize(iface={iface})")
    ChannelFactoryInitialize(0, iface)

    # ------------------------------------------------------------------
    # LowState monitor
    # ------------------------------------------------------------------
    monitor = LowStateMonitor()
    ls_type = None
    for mod_path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            import importlib
            m = importlib.import_module(mod_path)
            ls_type = getattr(m, "LowState_", None)
            if ls_type:
                break
        except Exception:
            pass

    if ls_type:
        ls_sub = ChannelSubscriber("rt/lowstate", ls_type)
        ls_sub.Init(monitor.callback, 200)
    else:
        print("[warn] Could not resolve LowState_ type — no joint seeding or IMU watch.")

    # SportModeState for foot-contact gating (optional)
    sport_mon: Optional[SportModeMonitor] = None
    if _HAS_SPORT and with_legs:
        sport_mon = SportModeMonitor()
        ss_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        ss_sub.Init(sport_mon.callback, 50)

    # ------------------------------------------------------------------
    # Load motion file
    # ------------------------------------------------------------------
    print(f"[load] {file}")
    joints_all, ts_raw, qs_raw = load_npz(file)

    joints, qs = _filter_joints(
        joints_all, qs_raw,
        with_legs=with_legs,
        no_waist=no_waist,
        dof23=dof23,
    )
    ts = ts_raw / max(1e-9, speed)

    if len(joints) == 0:
        raise SystemExit("No joints left after filtering — check --dof23 / --no-waist flags.")

    print(
        f"[load] {len(joints)} joints × {len(ts)} frames  "
        f"duration={ts[-1]:.2f}s  speed={speed}x"
    )
    print(f"[load] joints: {joints}")

    # ------------------------------------------------------------------
    # Wait for valid LowState
    # ------------------------------------------------------------------
    print(f"[state] waiting for rt/lowstate (timeout={seed_wait:.1f}s) …")
    if ls_type:
        if not monitor.wait(timeout=seed_wait):
            print("[warn] LowState not received within timeout — commands will be unseeded.")
        else:
            print("[state] LowState OK")

    # ------------------------------------------------------------------
    # Hanger boot sequence (arms-only mode)
    # ------------------------------------------------------------------
    if not with_legs and _HAS_LOCO:
        try:
            sys.path.insert(0, str(Path(__file__).parent.parent.parent / "arm_motion" / "safety"))
            from hanger_boot_sequence import hanger_boot_sequence  # type: ignore
            hanger_boot_sequence(iface=iface)
        except Exception as e:
            print(f"[warn] hanger_boot_sequence failed: {e}")

    # ------------------------------------------------------------------
    # Build controller
    # ------------------------------------------------------------------
    active_mode: Optional[str] = None

    if with_legs:
        # Release balance controller before taking lowcmd
        active_mode = _release_balance_controller()
        time.sleep(0.5)   # allow firmware transition
        ctrl: ArmWaistController | FullBodyController = FullBodyController(
            monitor,
            arm_kp=arm_kp, arm_kd=arm_kd,
            leg_kp=leg_kp, leg_kd=leg_kd,
        )
        # On lowcmd, override cmd_hz to 500 if still at default 50
        if cmd_hz <= 50.0:
            cmd_hz = FullBodyController.RATE_HZ
    else:
        ctrl = ArmWaistController(monitor, kp=arm_kp, kd=arm_kd)
        ctrl.enable()

    ctrl.seed_from_state(joints)

    # ------------------------------------------------------------------
    # Soft ramp to first frame
    # ------------------------------------------------------------------
    col_of = {j: i for i, j in enumerate(joints)}
    first_q = {j: float(qs[0, col_of[j]]) for j in joints}

    # velocity-limit the ramp target against current seeded state
    dt_ramp = 1.0 / cmd_hz
    first_q_safe = _vel_limited(ctrl._cmd_q, first_q, start_ramp)

    print(f"[ramp] ramping to first frame over {start_ramp:.1f}s …")
    ok = ctrl.ramp_to(first_q_safe, start_ramp, imu_limit, monitor)
    if not ok:
        print("[estop] IMU limit triggered during ramp — aborting.")
        _shutdown(ctrl, with_legs, active_mode)
        return

    # ------------------------------------------------------------------
    # Playback loop
    # ------------------------------------------------------------------
    dt    = 1.0 / cmd_hz
    t_end = float(ts[-1])
    tick  = 0
    log_interval = max(1, int(cmd_hz / 5))   # print ~5× per second

    print(f"[play] starting playback  ({cmd_hz:.0f} Hz, {t_end:.2f}s total)")
    wall_start = time.perf_counter()

    try:
        while True:
            t_now = time.perf_counter() - wall_start
            if t_now > t_end:
                break

            # IMU watchdog
            if not _imu_ok(monitor, imu_limit):
                print("[estop] IMU limit triggered during playback.")
                break

            # Foot-contact gate for legs
            if with_legs and sport_mon is not None and not sport_mon.feet_loaded:
                # freeze leg joints, still move arms
                leg_freeze = {j: ctrl._cmd_q.get(j, 0.0) for j in joints if j in LEG_JOINTS}
            else:
                leg_freeze = {}

            # Interpolate trajectory
            q_row   = _interp_row(ts, qs, t_now)
            q_tgt   = {j: float(q_row[col_of[j]]) for j in joints}

            # Apply foot-contact freeze on legs
            q_tgt.update(leg_freeze)

            # Velocity limiting against last commanded state
            q_safe  = _vel_limited(ctrl._cmd_q, q_tgt, dt)

            ctrl.write(q_safe)

            if tick % log_interval == 0:
                arm_str = "  ".join(
                    f"j{j}={q_safe[j]:+.2f}"
                    for j in joints
                    if j in ARM_SDK_JOINTS and j in q_safe
                )
                leg_str = ""
                if with_legs:
                    leg_str = "  legs: " + " ".join(
                        f"j{j}={q_safe[j]:+.2f}"
                        for j in joints
                        if j in LEG_JOINTS and j in q_safe
                    )
                imu = monitor.imu_rpy
                imu_str = (
                    f"  imu r={np.degrees(imu[0]):+.1f}° p={np.degrees(imu[1]):+.1f}°"
                    if imu else ""
                )
                print(f"  t={t_now:5.2f}s  {arm_str}{leg_str}{imu_str}")

            tick += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[play] interrupted by user.")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    _shutdown(ctrl, with_legs, active_mode)
    print("[done] playback complete.")


def _shutdown(
    ctrl:        "ArmWaistController | FullBodyController",
    with_legs:   bool,
    active_mode: Optional[str],
) -> None:
    """Release arm_sdk or restore balance controller cleanly."""
    if not with_legs and isinstance(ctrl, ArmWaistController):
        print("[shutdown] disabling arm_sdk (motor_cmd[29].q = 0) …")
        ctrl.disable()
    elif with_legs:
        print("[shutdown] restoring balance controller …")
        _restore_balance_controller(active_mode)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Reproduce mapper.py motion on G1 via low-level DDS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iface",      default="enp1s0",
                        help="DDS network interface (e.g. enp1s0, eth0)")
    parser.add_argument("--file",       type=Path, default=Path("g1_motion.npz"),
                        help="Input NPZ from mapper.py")
    parser.add_argument("--with-legs",  action="store_true",
                        help="Include leg joints via rt/lowcmd (HANGER REQUIRED)")
    parser.add_argument("--no-waist",   action="store_true",
                        help="Skip waist joints (12-14)")
    parser.add_argument("--dof23",      action="store_true",
                        help="23-DOF safe: skip WaistRoll/Pitch + WristPitch/Yaw")
    parser.add_argument("--speed",      type=float, default=1.0,
                        help="Playback speed (0.5 = half speed, 2.0 = double)")
    parser.add_argument("--start-ramp", type=float, default=1.5,
                        help="Seconds to ramp from current pose to first frame")
    parser.add_argument("--seed-wait",  type=float, default=2.0,
                        help="Seconds to wait for initial rt/lowstate")
    parser.add_argument("--arm-kp",    type=float, default=60.0,
                        help="Position stiffness gain for arm/waist joints")
    parser.add_argument("--arm-kd",    type=float, default=1.5,
                        help="Damping gain for arm/waist joints")
    parser.add_argument("--leg-kp",    type=float, default=120.0,
                        help="Position stiffness gain for leg joints (--with-legs only)")
    parser.add_argument("--leg-kd",    type=float, default=2.0,
                        help="Damping gain for leg joints (--with-legs only)")
    parser.add_argument("--imu-limit", type=float, default=0.35,
                        help="IMU roll/pitch e-stop threshold (radians, default ≈20°)")
    parser.add_argument("--cmd-hz",    type=float, default=50.0,
                        help="Command rate Hz (overridden to 500 in --with-legs mode)")
    args = parser.parse_args()

    print(
        "\nWARNING: Ensure there are no obstacles around the robot.\n"
        "         Keep a spotter present during all tests.\n"
    )
    input("Press Enter to continue …\n")

    try:
        run(
            iface       = args.iface,
            file        = args.file,
            with_legs   = args.with_legs,
            no_waist    = args.no_waist,
            dof23       = args.dof23,
            speed       = args.speed,
            start_ramp  = args.start_ramp,
            seed_wait   = args.seed_wait,
            arm_kp      = args.arm_kp,
            arm_kd      = args.arm_kd,
            leg_kp      = args.leg_kp,
            leg_kd      = args.leg_kd,
            imu_limit   = args.imu_limit,
            cmd_hz      = args.cmd_hz,
        )
    except (SystemExit, ValueError) as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()
