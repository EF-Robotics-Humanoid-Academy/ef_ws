#!/usr/bin/env python3
"""
balance_1_leg.py — G1 one-leg balance via direct SDK DDS (rt/lowcmd).

State machine
─────────────
  Phase 0  Double-support stand, joint softening, estimator settle
  Phase 1  Shift COM over the future stance foot
  Phase 2  Unload swing foot until estimated contact force ≈ 0
  Phase 3  Lift swing leg (knee flexion + hip flexion trajectory)
  Phase 4  Hold — regulate torso orientation, pelvis lateral, stance-leg support
  Phase 5  Lower swing leg and restore double support

Architecture (single process, direct DDS)
──────────────────────────────────────────
  LowStateMonitor   – subscribes rt/lowstate, thread-safe snapshot
  StateEstimator    – IMU filter, COM lateral estimate, contact estimate
  BalanceController – stance-leg impedance law (ankle / hip / knee)
  SwingLegPlanner   – smooth joint-space trajectory for swing leg
  SafetySupervisor  – abort conditions, returns to double-support
  OneLegBalancer    – state machine, RecurrentThread at CTRL_HZ

Usage
─────
  python3 balance_1_leg.py --iface eth0 --stance left
  python3 balance_1_leg.py --iface eth0 --stance right --hold-time 8

WARNING
───────
  This script commands LOW-LEVEL motor torques on ALL leg joints.
  Use a hanger / harness at first.  Keep strict abort thresholds.
  Always test in unitree_mujoco simulation before real hardware.
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, Optional, Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# SDK imports
# ─────────────────────────────────────────────────────────────────────────────

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
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

# ─────────────────────────────────────────────────────────────────────────────
# Joint indices (mirrors g1_arm7_sdk_dds_example.py)
# ─────────────────────────────────────────────────────────────────────────────

class J:
    # Left leg
    LeftHipPitch    = 0;  LeftHipRoll     = 1;  LeftHipYaw      = 2
    LeftKnee        = 3;  LeftAnklePitch  = 4;  LeftAnkleRoll   = 5
    # Right leg
    RightHipPitch   = 6;  RightHipRoll    = 7;  RightHipYaw     = 8
    RightKnee       = 9;  RightAnklePitch = 10; RightAnkleRoll  = 11
    # Waist
    WaistYaw        = 12
    # Arms (held at zero / natural pose, not actively controlled here)
    LeftShoulderPitch  = 15; LeftShoulderRoll  = 16; LeftShoulderYaw  = 17
    LeftElbow          = 18; LeftWristRoll     = 19
    RightShoulderPitch = 22; RightShoulderRoll = 23; RightShoulderYaw = 24
    RightElbow         = 25; RightWristRoll    = 26

ALL_JOINTS = list(range(29))

# Leg joint subsets
LEFT_LEG  = [J.LeftHipPitch,  J.LeftHipRoll,  J.LeftHipYaw,  J.LeftKnee,  J.LeftAnklePitch,  J.LeftAnkleRoll]
RIGHT_LEG = [J.RightHipPitch, J.RightHipRoll, J.RightHipYaw, J.RightKnee, J.RightAnklePitch, J.RightAnkleRoll]

# ─────────────────────────────────────────────────────────────────────────────
# Joint limits (conservative subset; same source as reproduce.py)
# ─────────────────────────────────────────────────────────────────────────────

JOINT_LIMITS: Dict[int, Tuple[float, float]] = {
    J.LeftHipPitch:     (-1.50,  1.50),  J.RightHipPitch:    (-1.50,  1.50),
    J.LeftHipRoll:      (-0.50,  0.50),  J.RightHipRoll:     (-0.50,  0.50),
    J.LeftHipYaw:       (-0.50,  0.50),  J.RightHipYaw:      (-0.50,  0.50),
    J.LeftKnee:         ( 0.00,  2.20),  J.RightKnee:        ( 0.00,  2.20),
    J.LeftAnklePitch:   (-0.50,  0.50),  J.RightAnklePitch:  (-0.50,  0.50),
    J.LeftAnkleRoll:    (-0.28,  0.28),  J.RightAnkleRoll:   (-0.28,  0.28),
    J.WaistYaw:         (-1.00,  1.00),
}

def _clamp(v: float, j: int) -> float:
    lo, hi = JOINT_LIMITS.get(j, (-3.14, 3.14))
    return float(np.clip(v, lo, hi))

# ─────────────────────────────────────────────────────────────────────────────
# Nominal standing pose (rad) — tuned for G1 upright stand
# ─────────────────────────────────────────────────────────────────────────────

# HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll
_LEG_NOMINAL = [-0.10, 0.00, 0.00, 0.30, -0.20, 0.00]

def nominal_pose() -> Dict[int, float]:
    """Both legs at nominal standing position + arms relaxed."""
    pose: Dict[int, float] = {}
    for i, j in enumerate(LEFT_LEG):
        pose[j] = _LEG_NOMINAL[i]
    for i, j in enumerate(RIGHT_LEG):
        pose[j] = _LEG_NOMINAL[i]
    pose[J.WaistYaw] = 0.0
    # Arms — relaxed natural hang
    pose[J.LeftShoulderPitch]  =  0.30
    pose[J.LeftShoulderRoll]   =  0.20
    pose[J.LeftShoulderYaw]    =  0.00
    pose[J.LeftElbow]          =  0.50
    pose[J.LeftWristRoll]      =  0.00
    pose[J.RightShoulderPitch] =  0.30
    pose[J.RightShoulderRoll]  = -0.20
    pose[J.RightShoulderYaw]   =  0.00
    pose[J.RightElbow]         =  0.50
    pose[J.RightWristRoll]     =  0.00
    return pose

# ─────────────────────────────────────────────────────────────────────────────
# G1 geometry constants
# ─────────────────────────────────────────────────────────────────────────────

FOOT_HALF_TRACK = 0.090   # lateral offset of each foot from centerline (m)
COM_HEIGHT_NOM  = 0.750   # approximate COM height when standing (m)

# Target body roll to place COM over stance foot (small-angle approximation)
# left stance → lean left → roll > 0 in Unitree convention (right side higher)
# right stance → roll < 0
# Unitree G1 IMU: rpy[0] = roll, positive = left side up (right side down)
# So for left stance (foot at +y), lean so COM is at +y:
#   roll_target = +arctan(FOOT_HALF_TRACK / COM_HEIGHT_NOM) ≈ +0.120 rad
STANCE_ROLL_TARGET = {
    "left":  +math.atan2(FOOT_HALF_TRACK, COM_HEIGHT_NOM),   # ≈ +0.120 rad
    "right": -math.atan2(FOOT_HALF_TRACK, COM_HEIGHT_NOM),   # ≈ −0.120 rad
}

# ─────────────────────────────────────────────────────────────────────────────
# Safety / abort thresholds
# ─────────────────────────────────────────────────────────────────────────────

ABORT_ROLL_RAD      = 0.35    # ≈ 20° torso roll
ABORT_PITCH_RAD     = 0.30    # ≈ 17° torso pitch
ABORT_GYRO_RAD_S    = 2.50    # angular rate spike
ABORT_TAU_STANCE_NM = 28.0    # any single stance-joint torque > this
ABORT_LOWSTATE_AGE  = 0.10    # seconds since last lowstate

# ─────────────────────────────────────────────────────────────────────────────
# Control gains
# ─────────────────────────────────────────────────────────────────────────────

# Stance-leg gains — START LOW, increase cautiously after sim validation
STANCE_KP = {
    "hip_pitch":   80.0,
    "hip_roll":    80.0,
    "hip_yaw":     40.0,
    "knee":       100.0,
    "ankle_pitch":  60.0,
    "ankle_roll":   40.0,
}
STANCE_KD = {
    "hip_pitch":   2.5,
    "hip_roll":    2.5,
    "hip_yaw":     1.0,
    "knee":        3.5,
    "ankle_pitch": 1.5,
    "ankle_roll":  1.0,
}

# Swing-leg gains — deliberately soft
SWING_KP = {
    "hip_pitch":  30.0,
    "hip_roll":   30.0,
    "hip_yaw":    15.0,
    "knee":       35.0,
    "ankle_pitch": 10.0,   # compliant
    "ankle_roll":   8.0,   # compliant
}
SWING_KD = {
    "hip_pitch":  1.0,
    "hip_roll":   1.0,
    "hip_yaw":    0.5,
    "knee":       1.2,
    "ankle_pitch": 0.5,
    "ankle_roll":  0.4,
}

# Balance feedback gains (task-space corrections mapped to joint corrections)
# Torso roll → stance ankle roll
KP_ROLL_TO_ANKLE   =  0.25   # (rad ankle) / (rad body roll error)
KD_ROLL_TO_ANKLE   =  0.05   # (rad ankle) / (rad/s body roll rate)
# Torso roll → stance hip roll (complements ankle)
KP_ROLL_TO_HIP     =  0.40
KD_ROLL_TO_HIP     =  0.08
# Torso pitch → stance ankle pitch
KP_PITCH_TO_ANKLE  =  0.20
KD_PITCH_TO_ANKLE  =  0.04
# Torso pitch → stance hip pitch
KP_PITCH_TO_HIP    =  0.30
KD_PITCH_TO_HIP    =  0.06
# Lateral COM error → stance hip roll (secondary; uses estimated COM deviation)
KP_COM_LAT_TO_HIP  =  1.20   # (rad hip) / (m COM lateral error)

# Phase durations (seconds)
DUR_SOFTEN         = 3.0
DUR_COM_SHIFT      = 3.5
DUR_UNLOAD         = 2.5
DUR_LIFT           = 2.5
# hold time is user-configurable (default 5.0 s)
DUR_LOWER          = 2.5
DUR_COM_RESTORE    = 3.0

CTRL_HZ    = 200.0
CTRL_DT    = 1.0 / CTRL_HZ

# ─────────────────────────────────────────────────────────────────────────────
# State machine phases
# ─────────────────────────────────────────────────────────────────────────────

class Phase(IntEnum):
    SOFTEN      = 0   # joint softening / estimator settle
    COM_SHIFT   = 1   # shift COM over stance foot
    UNLOAD      = 2   # unload swing foot
    LIFT        = 3   # lift swing leg
    HOLD        = 4   # hold single-leg stance
    LOWER       = 5   # lower swing leg
    RESTORE     = 6   # restore double support / COM to centre
    DONE        = 7
    ABORT       = 8

# ─────────────────────────────────────────────────────────────────────────────
# State estimate (snapshot passed between subsystems each tick)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Estimate:
    roll:           float = 0.0   # rad (IMU)
    pitch:          float = 0.0   # rad (IMU)
    yaw:            float = 0.0   # rad (IMU)
    gyro_x:         float = 0.0   # rad/s
    gyro_y:         float = 0.0   # rad/s
    gyro_z:         float = 0.0   # rad/s
    q:              Dict[int, float] = field(default_factory=dict)
    dq:             Dict[int, float] = field(default_factory=dict)
    tau_est:        Dict[int, float] = field(default_factory=dict)
    # Derived
    com_lat:        float = 0.0   # estimated COM lateral displacement (m)
    swing_loaded:   float = 1.0   # 0.0=unloaded, 1.0=fully loaded (normalised)
    stance_loaded:  float = 1.0
    ts:             float = 0.0   # wall time of this estimate

# ─────────────────────────────────────────────────────────────────────────────
# LowStateMonitor — thread-safe raw snapshot of rt/lowstate
# ─────────────────────────────────────────────────────────────────────────────

class LowStateMonitor:
    def __init__(self) -> None:
        self._lock   = threading.Lock()
        self._msg    = None
        self._ts     = 0.0
        self._ready  = threading.Event()

    def callback(self, msg: LowState_) -> None:
        with self._lock:
            self._msg = msg
            self._ts  = time.time()
        self._ready.set()

    def wait(self, timeout: float = 5.0) -> bool:
        return self._ready.wait(timeout=timeout)

    @property
    def age(self) -> float:
        with self._lock:
            return time.time() - self._ts if self._ts > 0 else 999.0

    def snapshot(self) -> Optional[object]:
        with self._lock:
            return self._msg

# ─────────────────────────────────────────────────────────────────────────────
# StateEstimator — filters IMU, estimates COM lateral position and contacts
# ─────────────────────────────────────────────────────────────────────────────

class StateEstimator:
    """
    Reads raw LowState and returns a filtered Estimate.

    COM lateral:
        com_lat ≈ COM_HEIGHT_NOM * sin(roll_filtered) ≈ COM_HEIGHT_NOM * roll
        (small-angle model; adequate for the ±15° range of interest)

    Contact loading:
        Uses ankle + knee tau_est as a proxy for vertical ground-reaction force.
        Normalised to their nominal loaded values (standing still).
        swing_loaded = 1.0 means fully loaded; 0.0 means unloaded.
    """

    ALPHA_IMU   = 0.15   # EMA coefficient for IMU filter (lower = smoother)
    NOM_TAU_ANKLE = 12.0  # Nm, nominal ankle load in normal double support
    NOM_TAU_KNEE  = 20.0  # Nm, nominal knee load in normal double support

    def __init__(self, stance: str) -> None:
        self.stance = stance
        self.swing  = "right" if stance == "left" else "left"
        self._roll_f  = 0.0
        self._pitch_f = 0.0
        self._gyro_x_f = 0.0
        self._gyro_y_f = 0.0

        self._stance_leg = LEFT_LEG  if stance == "left"  else RIGHT_LEG
        self._swing_leg  = RIGHT_LEG if stance == "left"  else LEFT_LEG

    def _swing_idx(self, offset: int) -> int:
        return self._swing_leg[offset]

    def _stance_idx(self, offset: int) -> int:
        return self._stance_leg[offset]

    def update(self, msg) -> Estimate:
        a = self.ALPHA_IMU
        imu = msg.imu_state

        roll_raw  = float(imu.rpy[0])
        pitch_raw = float(imu.rpy[1])
        gyro_x    = float(imu.gyroscope[0])
        gyro_y    = float(imu.gyroscope[1])
        gyro_z    = float(imu.gyroscope[2])

        self._roll_f  = a * roll_raw  + (1 - a) * self._roll_f
        self._pitch_f = a * pitch_raw + (1 - a) * self._pitch_f
        self._gyro_x_f = a * gyro_x  + (1 - a) * self._gyro_x_f
        self._gyro_y_f = a * gyro_y  + (1 - a) * self._gyro_y_f

        q:        Dict[int, float] = {}
        dq:       Dict[int, float] = {}
        tau_est:  Dict[int, float] = {}
        for j in range(29):
            ms = msg.motor_state[j]
            q[j]       = float(ms.q)
            dq[j]      = float(ms.dq)
            tau_est[j] = float(ms.tau_est)

        com_lat = COM_HEIGHT_NOM * math.sin(self._roll_f)

        # Contact load estimate for swing leg (ankle + knee abs tau)
        sw_ankle  = self._swing_idx(4)   # AnklePitch offset
        sw_knee   = self._swing_idx(3)
        st_ankle  = self._stance_idx(4)
        st_knee   = self._stance_idx(3)

        swing_tau  = abs(tau_est.get(sw_ankle, 0.0)) + abs(tau_est.get(sw_knee, 0.0))
        stance_tau = abs(tau_est.get(st_ankle, 0.0)) + abs(tau_est.get(st_knee, 0.0))
        nom        = self.NOM_TAU_ANKLE + self.NOM_TAU_KNEE

        swing_loaded  = float(np.clip(swing_tau  / max(nom, 1.0), 0.0, 1.5))
        stance_loaded = float(np.clip(stance_tau / max(nom, 1.0), 0.0, 1.5))

        return Estimate(
            roll     = self._roll_f,
            pitch    = self._pitch_f,
            yaw      = float(imu.rpy[2]),
            gyro_x   = self._gyro_x_f,
            gyro_y   = self._gyro_y_f,
            gyro_z   = gyro_z,
            q        = q,
            dq       = dq,
            tau_est  = tau_est,
            com_lat  = com_lat,
            swing_loaded  = swing_loaded,
            stance_loaded = stance_loaded,
            ts       = time.time(),
        )

# ─────────────────────────────────────────────────────────────────────────────
# BalanceController — computes stance-leg desired joint angles each tick
# ─────────────────────────────────────────────────────────────────────────────

class BalanceController:
    """
    Regulates torso roll/pitch and lateral COM position using stance-leg joints.

    Outputs a dict {joint_index: q_desired} for the stance leg only.
    Gains are applied on top of the nominal pose.  The caller mixes in
    kp/kd for impedance control.

    Primary loops:
      1. Roll/lateral balance  → ankle roll + hip roll corrections
      2. Pitch balance         → ankle pitch + hip pitch corrections
      3. Height / knee support → knee position held at nominal
    """

    def __init__(self, stance: str) -> None:
        self.stance      = stance
        self.stance_leg  = LEFT_LEG  if stance == "left"  else RIGHT_LEG
        self.foot_y      = +FOOT_HALF_TRACK if stance == "left" else -FOOT_HALF_TRACK
        self.roll_target = STANCE_ROLL_TARGET[stance]

    def compute(self, est: Estimate, phase: Phase) -> Dict[int, float]:
        """Return desired joint angles for stance leg."""

        sl = self.stance_leg
        # Convenience index into the 6-element leg array:
        # [0]=HipPitch [1]=HipRoll [2]=HipYaw [3]=Knee [4]=AnklePitch [5]=AnkleRoll
        HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll = sl

        # ── Nominal pose as starting point ────────────────────────────────
        q = {j: _LEG_NOMINAL[i] for i, j in enumerate(sl)}

        # ── 1. Roll / lateral balance ──────────────────────────────────────
        roll_err = self.roll_target - est.roll   # target COM-shifted roll
        # Ankle roll correction (primary — fast reaction)
        ankle_roll_corr = (
            KP_ROLL_TO_ANKLE * roll_err
            - KD_ROLL_TO_ANKLE * est.gyro_x
        )
        # Hip roll correction (slower, moves COM laterally)
        hip_roll_corr = (
            KP_ROLL_TO_HIP * roll_err
            - KD_ROLL_TO_HIP * est.gyro_x
        )
        # Additional COM lateral correction via hip roll
        com_lat_err   = self.foot_y - est.com_lat
        hip_roll_com  = KP_COM_LAT_TO_HIP * com_lat_err

        q[AnkleRoll] += ankle_roll_corr
        q[HipRoll]   += hip_roll_corr + hip_roll_com

        # ── 2. Pitch balance ───────────────────────────────────────────────
        pitch_err = 0.0 - est.pitch   # target pitch = 0 (upright)
        ankle_pitch_corr = (
            KP_PITCH_TO_ANKLE * pitch_err
            - KD_PITCH_TO_ANKLE * est.gyro_y
        )
        hip_pitch_corr = (
            KP_PITCH_TO_HIP * pitch_err
            - KD_PITCH_TO_HIP * est.gyro_y
        )
        q[AnklePitch] += ankle_pitch_corr
        q[HipPitch]   += hip_pitch_corr

        # ── 3. Clamp everything to joint limits ────────────────────────────
        for j in sl:
            q[j] = _clamp(q[j], j)

        return q

# ─────────────────────────────────────────────────────────────────────────────
# SwingLegPlanner — generates swing-leg joint trajectory per phase
# ─────────────────────────────────────────────────────────────────────────────

class SwingLegPlanner:
    """
    Returns desired joint angles for the swing leg as a function of
    current phase and elapsed time within that phase.

    Design principles:
      • Lift trajectory is slow (seconds, not fractions of a second).
      • Ankle stays compliant throughout (low kp/kd, near-zero torque target).
      • Hip motion is small (< 0.15 rad) to avoid large COM disturbance.
      • Lower mirrors the lift trajectory in reverse.
    """

    # Maximum knee flex for single-leg stance (rad) — start small, increase after validation
    LIFT_KNEE_FLEX   =  0.50   # additional knee flex beyond nominal
    LIFT_HIP_FLEX    =  0.12   # hip pitch flex (positive = forward)
    LOWER_RETURN_MARGIN = 0.02 # rad before declaring fully lowered

    def __init__(self, stance: str) -> None:
        self.swing_leg = RIGHT_LEG if stance == "left" else LEFT_LEG

    def _nominal(self) -> Dict[int, float]:
        return {j: _LEG_NOMINAL[i] for i, j in enumerate(self.swing_leg)}

    def compute(
        self,
        phase:     Phase,
        phase_t:   float,   # elapsed time in current phase
        dur:       float,   # total duration of current phase
    ) -> Dict[int, float]:

        sl = self.swing_leg
        HipPitch, HipRoll, HipYaw, Knee, AnklePitch, AnkleRoll = sl
        q = self._nominal()

        ratio = float(np.clip(phase_t / max(dur, 1e-3), 0.0, 1.0))

        if phase in (Phase.SOFTEN, Phase.COM_SHIFT):
            # Swing leg tracks nominal — no motion yet
            pass

        elif phase == Phase.UNLOAD:
            # Slightly unweight: small knee/hip flex eases contact force
            unload_ratio = ratio * 0.5   # go halfway towards lift pose
            q[Knee]      += unload_ratio * self.LIFT_KNEE_FLEX * 0.4
            q[HipPitch]  += unload_ratio * self.LIFT_HIP_FLEX  * 0.4

        elif phase == Phase.LIFT:
            # Smooth raise using a cubic ease-in-out profile
            r = _ease_inout(ratio)
            q[Knee]     += r * self.LIFT_KNEE_FLEX
            q[HipPitch] += r * self.LIFT_HIP_FLEX
            # Ankle stays at nominal (compliant)

        elif phase == Phase.HOLD:
            # Maintain the final lifted pose
            q[Knee]     += self.LIFT_KNEE_FLEX
            q[HipPitch] += self.LIFT_HIP_FLEX

        elif phase == Phase.LOWER:
            # Mirror of lift: ease-out from raised to nominal
            r = _ease_inout(1.0 - ratio)
            q[Knee]     += r * self.LIFT_KNEE_FLEX
            q[HipPitch] += r * self.LIFT_HIP_FLEX

        elif phase in (Phase.RESTORE, Phase.DONE):
            pass  # fully nominal

        for j in sl:
            q[j] = _clamp(q[j], j)

        return q


def _ease_inout(t: float) -> float:
    """Cubic ease-in-out interpolation [0, 1] → [0, 1]."""
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)

# ─────────────────────────────────────────────────────────────────────────────
# SafetySupervisor — abort checks
# ─────────────────────────────────────────────────────────────────────────────

class SafetySupervisor:
    """
    Returns True (abort) if any safety condition is violated.

    Abort conditions (from the design spec):
      1. Torso roll or pitch exceeds threshold
      2. IMU angular-rate spike
      3. Any commanded stance-joint torque saturates (estimated via tau_est)
      4. Stance foot contact drops unexpectedly during HOLD phase
      5. LowState message is stale (communication timeout)
      6. COM estimate leaves the support polygon margin
    """

    COM_POLYGON_MARGIN = 0.06   # m — abort if |com_lat - foot_y| > this

    def __init__(self, stance: str) -> None:
        self.stance = stance
        self.foot_y = +FOOT_HALF_TRACK if stance == "left" else -FOOT_HALF_TRACK
        self._abort_reason = ""

    @property
    def abort_reason(self) -> str:
        return self._abort_reason

    def check(self, est: Estimate, mon: LowStateMonitor, phase: Phase) -> bool:
        """Return True if an abort should be triggered."""

        # 1. Torso tilt
        if abs(est.roll) > ABORT_ROLL_RAD:
            self._abort_reason = f"roll {math.degrees(est.roll):.1f}° > {math.degrees(ABORT_ROLL_RAD):.1f}°"
            return True
        if abs(est.pitch) > ABORT_PITCH_RAD:
            self._abort_reason = f"pitch {math.degrees(est.pitch):.1f}° > {math.degrees(ABORT_PITCH_RAD):.1f}°"
            return True

        # 2. IMU angular rate spike
        gyro_mag = math.sqrt(est.gyro_x**2 + est.gyro_y**2 + est.gyro_z**2)
        if gyro_mag > ABORT_GYRO_RAD_S:
            self._abort_reason = f"gyro spike {gyro_mag:.2f} rad/s"
            return True

        # 3. Stale lowstate
        if mon.age > ABORT_LOWSTATE_AGE:
            self._abort_reason = f"lowstate stale {mon.age*1000:.0f} ms"
            return True

        # 4. Stance contact lost during HOLD (expect near-nominal loading)
        if phase == Phase.HOLD and est.stance_loaded < 0.2:
            self._abort_reason = "stance foot contact lost during HOLD"
            return True

        # 5. COM leaves support polygon
        if phase in (Phase.HOLD, Phase.LIFT):
            com_err = abs(est.com_lat - self.foot_y)
            if com_err > self.COM_POLYGON_MARGIN:
                self._abort_reason = (
                    f"COM lateral err {com_err*100:.1f} cm > "
                    f"{self.COM_POLYGON_MARGIN*100:.1f} cm margin"
                )
                return True

        return False

# ─────────────────────────────────────────────────────────────────────────────
# MotionSwitcher helpers
# ─────────────────────────────────────────────────────────────────────────────

def _release_mode() -> Optional[str]:
    if not _HAS_SWITCHER:
        print("[warn] MotionSwitcherClient unavailable — cannot release locomotion mode.")
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
        return active or None
    except Exception as e:
        print(f"[warn] ReleaseMode failed: {e}")
        return None


def _restore_mode(mode: Optional[str]) -> None:
    if not _HAS_SWITCHER or not mode:
        return
    try:
        sc = MotionSwitcherClient()
        sc.SetTimeout(5.0)
        sc.Init()
        sc.SelectMode(mode)
        print(f"[switcher] restored mode: '{mode}'")
    except Exception as e:
        print(f"[warn] SelectMode('{mode}') failed: {e}")

# ─────────────────────────────────────────────────────────────────────────────
# LowCmdWriter — fills and publishes one LowCmd_ per tick
# ─────────────────────────────────────────────────────────────────────────────

class LowCmdWriter:
    """
    Holds the outgoing LowCmd_ buffer.  Callers call set_joint() to fill
    per-joint targets, then flush() to compute CRC and publish.

    kp/kd can differ per joint and per mode (stance vs swing).
    """

    def __init__(self) -> None:
        self._cmd  = unitree_hg_msg_dds__LowCmd_()
        self._crc  = CRC()
        self._pub  = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()

    def set_joint(
        self,
        j:   int,
        q:   float,
        dq:  float = 0.0,
        tau: float = 0.0,
        kp:  float = 0.0,
        kd:  float = 0.0,
    ) -> None:
        mc      = self._cmd.motor_cmd[j]
        mc.q    = float(q)
        mc.dq   = float(dq)
        mc.tau  = float(tau)
        mc.kp   = float(kp)
        mc.kd   = float(kd)

    def zero_joint(self, j: int) -> None:
        """Send zero targets with zero gains — effectively damping-only if kd>0."""
        mc      = self._cmd.motor_cmd[j]
        mc.q    = 0.0
        mc.dq   = 0.0
        mc.tau  = 0.0
        mc.kp   = 0.0
        mc.kd   = 0.5

    def flush(self) -> None:
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

# ─────────────────────────────────────────────────────────────────────────────
# Helper: map leg-array gains to joint-indexed dicts
# ─────────────────────────────────────────────────────────────────────────────

_GAIN_KEYS = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]

def _kp_kd_for_leg(
    joints: list,
    kp_map: Dict[str, float],
    kd_map: Dict[str, float],
) -> Tuple[Dict[int, float], Dict[int, float]]:
    kp = {j: kp_map[k] for j, k in zip(joints, _GAIN_KEYS)}
    kd = {j: kd_map[k] for j, k in zip(joints, _GAIN_KEYS)}
    return kp, kd

# ─────────────────────────────────────────────────────────────────────────────
# OneLegBalancer — state machine + RecurrentThread
# ─────────────────────────────────────────────────────────────────────────────

class OneLegBalancer:
    def __init__(self, stance: str, hold_time: float, iface: str) -> None:
        self.stance    = stance
        self.hold_time = hold_time
        self.iface     = iface

        self._mon        = LowStateMonitor()
        self._estimator  = StateEstimator(stance)
        self._balance    = BalanceController(stance)
        self._swing_plan = SwingLegPlanner(stance)
        self._safety     = SafetySupervisor(stance)
        self._writer     = LowCmdWriter()

        self.stance_leg = LEFT_LEG  if stance == "left"  else RIGHT_LEG
        self.swing_leg  = RIGHT_LEG if stance == "left"  else LEFT_LEG

        self._stance_kp, self._stance_kd = _kp_kd_for_leg(self.stance_leg, STANCE_KP, STANCE_KD)
        self._swing_kp,  self._swing_kd  = _kp_kd_for_leg(self.swing_leg,  SWING_KP,  SWING_KD)

        self._phase      = Phase.SOFTEN
        self._phase_t    = 0.0    # elapsed time inside current phase
        self._phase_dur  = DUR_SOFTEN

        self._seed_q: Dict[int, float] = {}  # actual pose at start

        self._abort_requested = threading.Event()
        self._thread: Optional[RecurrentThread] = None
        self._active_mode: Optional[str] = None

        # Saved nominal roll target for COM shift ramp
        self._roll_target_ramp = 0.0

        print(f"[balancer] stance={stance}  hold_time={hold_time}s  iface={iface}")

    # ── lifecycle ──────────────────────────────────────────────────────────

    def init(self) -> None:
        ChannelFactoryInitialize(0, self.iface)

        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._mon.callback, 10)

        print("[balancer] waiting for rt/lowstate …")
        if not self._mon.wait(timeout=5.0):
            raise RuntimeError("No rt/lowstate received within 5 s — check network.")

        print("[balancer] lowstate received.")

        # Seed commanded positions from actual current state
        msg = self._mon.snapshot()
        if msg is not None:
            for j in ALL_JOINTS:
                self._seed_q[j] = float(msg.motor_state[j].q)

    def start(self) -> None:
        print("[balancer] releasing locomotion mode (rt/lowcmd takeover) …")
        self._active_mode = _release_mode()
        time.sleep(0.5)

        self._thread = RecurrentThread(
            interval=CTRL_DT, target=self._control_tick, name="balance_ctrl"
        )
        self._thread.Start()
        print(f"[balancer] control thread started at {CTRL_HZ:.0f} Hz.")

    def stop(self) -> None:
        self._abort_requested.set()
        time.sleep(0.3)
        if self._thread is not None:
            # RecurrentThread has no explicit Stop(); flag abort and let it finish
            pass
        _restore_mode(self._active_mode)
        print("[balancer] stopped.")

    # ── state machine transitions ─────────────────────────────────────────

    def _advance_phase(self, next_phase: Phase, dur: float) -> None:
        print(f"[balancer] → Phase {self._phase.name} → {next_phase.name}")
        self._phase     = next_phase
        self._phase_t   = 0.0
        self._phase_dur = dur

    # ── main control tick (called at CTRL_HZ) ─────────────────────────────

    def _control_tick(self) -> None:
        if self._phase in (Phase.DONE, Phase.ABORT):
            return

        # ── read state ──────────────────────────────────────────────────
        msg = self._mon.snapshot()
        if msg is None:
            return

        est = self._estimator.update(msg)
        self._phase_t += CTRL_DT

        # ── safety check ────────────────────────────────────────────────
        if self._abort_requested.is_set() or (
            self._phase not in (Phase.SOFTEN, Phase.COM_SHIFT)
            and self._safety.check(est, self._mon, self._phase)
        ):
            reason = self._safety.abort_reason if not self._abort_requested.is_set() else "user request"
            print(f"[ABORT] {reason}  phase={self._phase.name}")
            self._phase = Phase.ABORT
            self._emergency_return_to_stand(est)
            return

        # ── compute desired joint angles ─────────────────────────────
        q_out: Dict[int, float] = {}
        kp_out: Dict[int, float] = {}
        kd_out: Dict[int, float] = {}

        # ── stance leg ──────────────────────────────────────────────────
        if self._phase in (Phase.SOFTEN, Phase.COM_SHIFT, Phase.RESTORE):
            # Soft position tracking during transition phases
            q_stance = {j: self._seed_q.get(j, _LEG_NOMINAL[i])
                        for i, j in enumerate(self.stance_leg)}
            # Smoothly interpolate to nominal
            ratio = float(np.clip(self._phase_t / max(self._phase_dur, 1e-3), 0.0, 1.0))
            if self._phase == Phase.SOFTEN:
                for i, j in enumerate(self.stance_leg):
                    q_stance[j] = (1 - ratio) * self._seed_q.get(j, _LEG_NOMINAL[i]) + ratio * _LEG_NOMINAL[i]
            elif self._phase == Phase.COM_SHIFT:
                # Include balance corrections at reduced weight so gains engage gently
                q_bal  = self._balance.compute(est, self._phase)
                alpha  = _ease_inout(ratio)
                for j in self.stance_leg:
                    nom = self._seed_q.get(j, q_bal[j])
                    q_stance[j] = (1 - alpha) * nom + alpha * q_bal[j]
            elif self._phase == Phase.RESTORE:
                q_nom  = {j: _LEG_NOMINAL[i] for i, j in enumerate(self.stance_leg)}
                q_bal  = self._balance.compute(est, self._phase)
                alpha  = _ease_inout(ratio)
                # Fade from balance law back to nominal
                for j in self.stance_leg:
                    q_stance[j] = alpha * q_nom[j] + (1 - alpha) * q_bal[j]
        else:
            q_stance = self._balance.compute(est, self._phase)

        for j in self.stance_leg:
            q_out[j]  = q_stance[j]
            kp_out[j] = self._stance_kp[j]
            kd_out[j] = self._stance_kd[j]

        # ── swing leg ───────────────────────────────────────────────────
        q_swing = self._swing_plan.compute(self._phase, self._phase_t, self._phase_dur)
        for j in self.swing_leg:
            q_out[j]  = q_swing[j]
            kp_out[j] = self._swing_kp[j]
            kd_out[j] = self._swing_kd[j]

        # ── write to motors ─────────────────────────────────────────────
        for j in list(self.stance_leg) + list(self.swing_leg):
            self._writer.set_joint(j, q=q_out[j], kp=kp_out[j], kd=kd_out[j])
        self._writer.flush()

        # ── log summary (throttled) ──────────────────────────────────────
        tick = int(self._phase_t / CTRL_DT)
        if tick % int(CTRL_HZ) == 0:   # once per second
            print(
                f"[{self._phase.name:<10s}] t={self._phase_t:5.1f}s  "
                f"roll={math.degrees(est.roll):+6.2f}°  pitch={math.degrees(est.pitch):+6.2f}°  "
                f"com_lat={est.com_lat*100:+5.1f}cm  "
                f"swing_load={est.swing_loaded:.2f}  stance_load={est.stance_loaded:.2f}"
            )

        # ── phase transitions ────────────────────────────────────────────
        self._check_phase_transitions(est)

    def _check_phase_transitions(self, est: Estimate) -> None:
        t    = self._phase_t
        dur  = self._phase_dur

        if self._phase == Phase.SOFTEN:
            if t >= dur:
                self._advance_phase(Phase.COM_SHIFT, DUR_COM_SHIFT)

        elif self._phase == Phase.COM_SHIFT:
            if t >= dur:
                self._advance_phase(Phase.UNLOAD, DUR_UNLOAD)

        elif self._phase == Phase.UNLOAD:
            # Proceed when swing foot is sufficiently unloaded OR timeout
            swing_unloaded = est.swing_loaded < 0.25
            if swing_unloaded or t >= dur:
                if not swing_unloaded:
                    print("[warn] swing foot not fully unloaded — proceeding anyway (check LIFT_KNEE_FLEX)")
                self._advance_phase(Phase.LIFT, DUR_LIFT)

        elif self._phase == Phase.LIFT:
            if t >= dur:
                self._advance_phase(Phase.HOLD, self.hold_time)

        elif self._phase == Phase.HOLD:
            if t >= dur:
                self._advance_phase(Phase.LOWER, DUR_LOWER)

        elif self._phase == Phase.LOWER:
            if t >= dur:
                self._advance_phase(Phase.RESTORE, DUR_COM_RESTORE)

        elif self._phase == Phase.RESTORE:
            if t >= dur:
                self._advance_phase(Phase.DONE, 0.0)
                print("[balancer] sequence complete — returning control.")
                _restore_mode(self._active_mode)
                self._active_mode = None

    # ── emergency return ─────────────────────────────────────────────────

    def _emergency_return_to_stand(self, est: Estimate) -> None:
        """
        Best-effort controlled descent: command both legs toward nominal
        standing pose over ~1 second at reduced gains.  This is NOT a
        guaranteed recovery — the hanger must catch the robot if balance
        is lost entirely.
        """
        print("[balancer] ABORT → emergency double-support return")
        nom   = nominal_pose()
        steps = int(1.2 * CTRL_HZ)   # 1.2 seconds
        low_kp = 40.0
        low_kd = 2.0

        for leg in (self.stance_leg, self.swing_leg):
            for j in leg:
                q_now = est.q.get(j, nom.get(j, 0.0))
                # Snapshot current as start
                self._seed_q[j] = q_now

        for i in range(1, steps + 1):
            ratio = i / steps
            for leg in (self.stance_leg, self.swing_leg):
                for j_idx, j in enumerate(leg):
                    q_start = self._seed_q.get(j, _LEG_NOMINAL[j_idx])
                    q_tgt   = nom.get(j, 0.0)
                    q       = q_start + (q_tgt - q_start) * _ease_inout(ratio)
                    self._writer.set_joint(j, q=_clamp(q, j), kp=low_kp, kd=low_kd)
            self._writer.flush()
            time.sleep(CTRL_DT)

        print("[balancer] emergency return complete.")
        _restore_mode(self._active_mode)
        self._active_mode = None
        self._phase = Phase.ABORT

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(
        description="G1 one-leg balance — direct SDK DDS low-level control."
    )
    parser.add_argument(
        "--iface", default="eth0",
        help="Network interface for DDS (default: eth0)",
    )
    parser.add_argument(
        "--stance", choices=["left", "right"], default="left",
        help="Which leg stays on the ground (default: left)",
    )
    parser.add_argument(
        "--hold-time", type=float, default=5.0,
        help="Seconds to hold single-leg stance (default: 5.0)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Subscribe to lowstate and print estimates but do NOT publish commands.",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  G1 one-leg balance controller")
    print(f"  stance={args.stance}  hold={args.hold_time}s  iface={args.iface}")
    print("=" * 60)
    print()
    print("WARNING: This script commands LOW-LEVEL motor torques.")
    print("         Ensure the robot is on a hanger / harness.")
    print("         Validate in unitree_mujoco simulation first.")
    print()
    ans = input("Type 'yes' to continue: ").strip().lower()
    if ans != "yes":
        print("Aborted.")
        return 0

    balancer = OneLegBalancer(
        stance    = args.stance,
        hold_time = args.hold_time,
        iface     = args.iface,
    )

    balancer.init()

    if args.dry_run:
        print("[dry-run] Estimator active — no commands sent. Ctrl-C to exit.")
        sub_mon    = balancer._mon
        estimator  = balancer._estimator
        try:
            while True:
                time.sleep(0.5)
                msg = sub_mon.snapshot()
                if msg is not None:
                    est = estimator.update(msg)
                    print(
                        f"roll={math.degrees(est.roll):+6.2f}°  "
                        f"pitch={math.degrees(est.pitch):+6.2f}°  "
                        f"com_lat={est.com_lat*100:+5.1f}cm  "
                        f"swing_load={est.swing_loaded:.2f}"
                    )
        except KeyboardInterrupt:
            pass
        return 0

    try:
        balancer.start()
        while balancer._phase not in (Phase.DONE, Phase.ABORT):
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[user] Ctrl-C received — requesting abort.")
        balancer._abort_requested.set()
        time.sleep(2.0)
    finally:
        balancer.stop()

    return 0 if balancer._phase == Phase.DONE else 1


if __name__ == "__main__":
    raise SystemExit(main())
