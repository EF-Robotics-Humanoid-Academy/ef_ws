"""
mapper.py
=========

Maps a pose recording produced by vision_module.py to G1 robot joint
angles and saves an NPZ file compatible with pbd_reproduce.py.

Algorithm
---------
For every recorded frame:

  1. Extract MediaPipe world-landmark 3-D positions (hip-centred, metres).
     Falls back to depth-derived positions when world_xyz is absent.

  2. Build two reference frames from the landmark cloud:
       - torso frame  : origin = shoulder midpoint
                        x = body-right, y = body-forward, z = body-up
       - pelvis frame : origin = hip midpoint, same axes

  3. Express each limb segment in its parent frame and decompose the
     direction vector into the corresponding G1 revolute-joint angles
     using atan2 projections.

  4. Clamp every computed angle to the G1 joint's safe range.

  5. For joints that cannot be estimated from 2-D skeleton geometry
     (shoulder yaw, wrist pitch/yaw) a zero or small heuristic value
     is used — these DOFs require hand-pose or IMU data.

G1 joint index table (from g1_arm7_sdk_dds_example.py)
-------------------------------------------------------
  Legs
    0  LeftHipPitch        6  RightHipPitch
    1  LeftHipRoll         7  RightHipRoll
    2  LeftHipYaw          8  RightHipYaw
    3  LeftKnee            9  RightKnee
    4  LeftAnklePitch     10  RightAnklePitch
    5  LeftAnkleRoll      11  RightAnkleRoll
  Waist
    12 WaistYaw
    13 WaistRoll          (invalid on 23-DOF locked-waist models)
    14 WaistPitch         (invalid on 23-DOF locked-waist models)
  Left arm                Right arm
    15 LeftShoulderPitch    22 RightShoulderPitch
    16 LeftShoulderRoll     23 RightShoulderRoll
    17 LeftShoulderYaw      24 RightShoulderYaw
    18 LeftElbow            25 RightElbow
    19 LeftWristRoll        26 RightWristRoll
    20 LeftWristPitch       27 RightWristPitch   (invalid 23-DOF)
    21 LeftWristYaw         28 RightWristYaw     (invalid 23-DOF)

Output NPZ keys (compatible with pbd_reproduce.py)
---------------------------------------------------
  joints          int32  [J]      joint indices in column order
  ts              float32 [N]     timestamps in seconds
  qs              float32 [N, J]  joint angles (radians)
  fk_qs           float32 [N, J]  copy of qs  (replay compat)
  poll_s          float32 scalar  mean dt between samples
  representation  str             "joint_space"

Usage
-----
    python3 mapper.py [OPTIONS]

    --input   PATH  pose_recording.json from vision_module.py
    --output  PATH  G1 motion NPZ (default: g1_motion.npz)
    --no-legs       Skip leg joints (arm + waist only)
    --no-waist      Skip waist joints (23-DOF safe mode)
    --dof23         Alias for --no-waist; also skips WristPitch/WristYaw
    --verbose       Print per-frame angle summaries

References
----------
    ../../arm_motion/sdk_details.md
    ../../arm_motion/g1_arm7_sdk_dds_example.py
    ../../arm_motion/pbd/pbd_docs.md
    ../../arm_motion/pbd/pbd_reproduce.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# G1 joint index constants  (mirrors g1_arm7_sdk_dds_example.py)
# ---------------------------------------------------------------------------

class G1Joint:
    # Left leg
    LeftHipPitch    = 0
    LeftHipRoll     = 1
    LeftHipYaw      = 2
    LeftKnee        = 3
    LeftAnklePitch  = 4
    LeftAnkleRoll   = 5
    # Right leg
    RightHipPitch   = 6
    RightHipRoll    = 7
    RightHipYaw     = 8
    RightKnee       = 9
    RightAnklePitch = 10
    RightAnkleRoll  = 11
    # Waist
    WaistYaw        = 12
    WaistRoll       = 13   # invalid on 23-DOF waist-locked
    WaistPitch      = 14   # invalid on 23-DOF waist-locked
    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll  = 16
    LeftShoulderYaw   = 17
    LeftElbow         = 18
    LeftWristRoll     = 19
    LeftWristPitch    = 20   # invalid on 23-DOF
    LeftWristYaw      = 21   # invalid on 23-DOF
    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll  = 23
    RightShoulderYaw   = 24
    RightElbow         = 25
    RightWristRoll     = 26
    RightWristPitch    = 27   # invalid on 23-DOF
    RightWristYaw      = 28   # invalid on 23-DOF


# ---------------------------------------------------------------------------
# Joint limits  (radians, conservative safe ranges)
# ---------------------------------------------------------------------------

JOINT_LIMITS: Dict[int, Tuple[float, float]] = {
    # Legs
    G1Joint.LeftHipPitch:      (-1.57, 1.57),
    G1Joint.LeftHipRoll:       (-0.52, 0.52),
    G1Joint.LeftHipYaw:        (-0.52, 0.52),
    G1Joint.LeftKnee:          ( 0.00, 2.30),
    G1Joint.LeftAnklePitch:    (-0.52, 0.52),
    G1Joint.LeftAnkleRoll:     (-0.30, 0.30),
    G1Joint.RightHipPitch:     (-1.57, 1.57),
    G1Joint.RightHipRoll:      (-0.52, 0.52),
    G1Joint.RightHipYaw:       (-0.52, 0.52),
    G1Joint.RightKnee:         ( 0.00, 2.30),
    G1Joint.RightAnklePitch:   (-0.52, 0.52),
    G1Joint.RightAnkleRoll:    (-0.30, 0.30),
    # Waist
    G1Joint.WaistYaw:          (-1.00, 1.00),
    G1Joint.WaistRoll:         (-0.35, 0.35),
    G1Joint.WaistPitch:        (-0.52, 0.52),
    # Left arm
    G1Joint.LeftShoulderPitch: (-1.57, 2.40),
    G1Joint.LeftShoulderRoll:  (-0.20, 2.40),
    G1Joint.LeftShoulderYaw:   (-1.40, 1.40),
    G1Joint.LeftElbow:         ( 0.00, 2.40),
    G1Joint.LeftWristRoll:     (-1.40, 1.40),
    G1Joint.LeftWristPitch:    (-0.70, 0.70),
    G1Joint.LeftWristYaw:      (-0.44, 0.44),
    # Right arm
    G1Joint.RightShoulderPitch:(-1.57, 2.40),
    G1Joint.RightShoulderRoll: (-2.40, 0.20),
    G1Joint.RightShoulderYaw:  (-1.40, 1.40),
    G1Joint.RightElbow:        ( 0.00, 2.40),
    G1Joint.RightWristRoll:    (-1.40, 1.40),
    G1Joint.RightWristPitch:   (-0.70, 0.70),
    G1Joint.RightWristYaw:     (-0.44, 0.44),
}


def clamp(value: float, joint_idx: int) -> float:
    lo, hi = JOINT_LIMITS.get(joint_idx, (-3.14, 3.14))
    return float(np.clip(value, lo, hi))


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Unsigned angle in [0, pi] between two vectors."""
    cos_a = np.clip(np.dot(normalize(a), normalize(b)), -1.0, 1.0)
    return float(np.arccos(cos_a))


def safe_atan2(y: float, x: float) -> float:
    if abs(x) < 1e-9 and abs(y) < 1e-9:
        return 0.0
    return float(np.arctan2(y, x))


# ---------------------------------------------------------------------------
# Body-frame construction
# ---------------------------------------------------------------------------

def _get(joints_world: Dict[str, np.ndarray], name: str) -> Optional[np.ndarray]:
    v = joints_world.get(name)
    return None if v is None else np.asarray(v, dtype=float)


def build_torso_frame(
    jw: Dict[str, np.ndarray],
) -> Optional[np.ndarray]:
    """Return 3x3 rotation matrix [x_right | y_fwd | z_up] for the torso.

    Uses shoulder and hip landmarks. Returns None if landmarks missing.
    MediaPipe world: x=right, y=up, z=toward_viewer.
    The returned frame uses robot conventions: x=right, y=forward, z=up.
    """
    ls = _get(jw, "left_shoulder")
    rs = _get(jw, "right_shoulder")
    lh = _get(jw, "left_hip")
    rh = _get(jw, "right_hip")
    if any(v is None for v in (ls, rs, lh, rh)):
        return None

    sh_mid = (ls + rs) / 2.0
    hp_mid = (lh + rh) / 2.0

    x = normalize(rs - ls)                  # body-right
    z = normalize(sh_mid - hp_mid)          # body-up (spine axis)
    # Orthogonalise: forward = up × right
    y = normalize(np.cross(z, x))
    x = normalize(np.cross(y, z))           # re-orthogonalise right
    return np.column_stack([x, y, z])       # [right | fwd | up]


def build_pelvis_frame(
    jw: Dict[str, np.ndarray],
) -> Optional[np.ndarray]:
    """Return 3x3 rotation matrix for the pelvis, same convention as torso."""
    lh = _get(jw, "left_hip")
    rh = _get(jw, "right_hip")
    ls = _get(jw, "left_shoulder")
    rs = _get(jw, "right_shoulder")
    if any(v is None for v in (lh, rh, ls, rs)):
        return None

    x = normalize(rh - lh)                  # pelvis-right
    # Up = rough spine direction from pelvis
    z = normalize(((ls + rs) / 2.0) - ((lh + rh) / 2.0))
    y = normalize(np.cross(z, x))
    x = normalize(np.cross(y, z))
    return np.column_stack([x, y, z])


# ---------------------------------------------------------------------------
# Arm mapping  (7 DOF each: sp, sr, sy, elbow, wr, wp, wy)
# ---------------------------------------------------------------------------

def _shoulder_angles(
    upper_arm_world: np.ndarray,   # normalize(elbow - shoulder)
    R_torso:         np.ndarray,   # 3×3 torso frame
    side:            str,          # "left" or "right"
) -> Tuple[float, float, float]:
    """ShoulderPitch, ShoulderRoll, ShoulderYaw (approximate).

    Convention (arm hanging down = all zeros):
      pitch > 0 → arm swings forward
      roll  > 0 → left arm raises laterally (left = positive, right = negative)
      yaw        → internal/external rotation (approximated as 0 here)
    """
    v = R_torso.T @ upper_arm_world   # express in torso frame
    # v[0]=right, v[1]=fwd, v[2]=up

    # Arm at rest points straight down → v ≈ (0, 0, -1)
    pitch = safe_atan2(v[1], -v[2])

    if side == "left":
        roll = safe_atan2(-v[0], -v[2])   # left-arm out to left → positive
    else:
        roll = safe_atan2( v[0], -v[2])   # right-arm out to right → positive
        roll = -roll                       # G1 right shoulder roll: T-pose = -pi/2

    # ShoulderYaw: rotation of upper arm about its own axis.
    # Estimate from where the forearm deviates out of the "pitch plane".
    # Without forearm we cannot resolve this → leave 0.
    yaw = 0.0
    return pitch, roll, yaw


def _shoulder_yaw_from_forearm(
    upper_arm_world: np.ndarray,   # normalize(elbow - shoulder)
    forearm_world:   np.ndarray,   # normalize(wrist  - elbow)
    R_torso:         np.ndarray,
) -> float:
    """Estimate shoulder yaw from how the forearm deviates from the
    natural bent-elbow plane (perpendicular to sagittal/frontal mix)."""
    # Project forearm onto plane perpendicular to upper arm
    ua = normalize(upper_arm_world)
    fa = normalize(forearm_world)
    fa_perp = normalize(fa - np.dot(fa, ua) * ua)

    # Reference: "elbow-down" direction = down projected onto plane ⊥ upper arm
    down_world = R_torso @ np.array([0.0, 0.0, -1.0])
    ref = down_world - np.dot(down_world, ua) * ua
    ref_n = np.linalg.norm(ref)
    if ref_n < 1e-9:
        return 0.0
    ref = ref / ref_n

    cos_a = np.clip(np.dot(fa_perp, ref), -1.0, 1.0)
    yaw = float(np.arccos(cos_a))
    # sign via cross product
    cross = np.cross(ref, fa_perp)
    if np.dot(cross, ua) < 0:
        yaw = -yaw
    return yaw


def map_arm(
    side:    str,
    jw:      Dict[str, np.ndarray],
    R_torso: np.ndarray,
) -> Dict[int, float]:
    """Return {joint_idx: angle_rad} for one arm's 7 DOF."""
    sh  = _get(jw, f"{side}_shoulder")
    el  = _get(jw, f"{side}_elbow")
    wr  = _get(jw, f"{side}_wrist")
    idx_sp, idx_sr, idx_sy, idx_el, idx_wr, idx_wp, idx_wy = (
        (G1Joint.LeftShoulderPitch,  G1Joint.LeftShoulderRoll,
         G1Joint.LeftShoulderYaw,    G1Joint.LeftElbow,
         G1Joint.LeftWristRoll,      G1Joint.LeftWristPitch,
         G1Joint.LeftWristYaw)
        if side == "left" else
        (G1Joint.RightShoulderPitch, G1Joint.RightShoulderRoll,
         G1Joint.RightShoulderYaw,   G1Joint.RightElbow,
         G1Joint.RightWristRoll,     G1Joint.RightWristPitch,
         G1Joint.RightWristYaw)
    )

    result: Dict[int, float] = {}

    # --- shoulder ---
    if sh is not None and el is not None:
        ua_vec = normalize(el - sh)
        sp, sr, sy = _shoulder_angles(ua_vec, R_torso, side)

        # refine yaw using forearm if available
        if wr is not None:
            fa_vec = normalize(wr - el)
            sy = _shoulder_yaw_from_forearm(ua_vec, fa_vec, R_torso)

        result[idx_sp] = clamp(sp, idx_sp)
        result[idx_sr] = clamp(sr, idx_sr)
        result[idx_sy] = clamp(sy, idx_sy)

    # --- elbow ---
    if sh is not None and el is not None and wr is not None:
        # Interior angle at elbow; G1 elbow = 0 when extended
        elbow_interior = angle_between(sh - el, wr - el)
        elbow_q = np.pi - elbow_interior        # 0 = extended, pi/2 = 90° bent
        result[idx_el] = clamp(elbow_q, idx_el)

    # --- wrist ---
    if el is not None and wr is not None:
        fa_vec = normalize(wr - el)             # forearm direction in world frame
        fa_local = R_torso.T @ fa_vec           # in torso frame

        # WristRoll: forearm roll about its own axis → use hand landmark spread
        # Approximate from index-pinky vector when available
        idx_finger  = _get(jw, f"{side}_index")
        pinky       = _get(jw, f"{side}_pinky")
        wrist_roll  = 0.0
        if idx_finger is not None and pinky is not None:
            hand_span = normalize(idx_finger - pinky)
            hs_local  = R_torso.T @ hand_span
            fa_n      = normalize(fa_local)
            hs_perp   = hs_local - np.dot(hs_local, fa_n) * fa_n
            if np.linalg.norm(hs_perp) > 1e-9:
                hs_perp_n = normalize(hs_perp)
                # reference: up direction perpendicular to forearm
                up_l  = np.array([0.0, 0.0, 1.0])
                up_perp = up_l - np.dot(up_l, fa_n) * fa_n
                up_n    = np.linalg.norm(up_perp)
                if up_n > 1e-9:
                    up_perp /= up_n
                    cos_r = np.clip(np.dot(hs_perp_n, up_perp), -1.0, 1.0)
                    wrist_roll = float(np.arccos(cos_r))
                    if np.dot(np.cross(up_perp, hs_perp_n), fa_n) < 0:
                        wrist_roll = -wrist_roll

        # WristPitch: wrist flex/extend in the forearm's bending plane
        # Approximate from how much the hand (index) deviates from forearm axis
        wrist_pitch = 0.0
        if idx_finger is not None and wr is not None:
            hand_vec = normalize(idx_finger - wr)
            hv_local = R_torso.T @ hand_vec
            fa_n     = normalize(fa_local)
            # component of hand_vec perpendicular to forearm, in the y-z (fwd/up) plane
            dev = hv_local - np.dot(hv_local, fa_n) * fa_n
            if np.linalg.norm(dev) > 1e-9:
                dev_n = normalize(dev)
                wrist_pitch = float(np.arctan2(dev_n[2], dev_n[1]))

        result[idx_wr] = clamp(wrist_roll,  idx_wr)
        result[idx_wp] = clamp(wrist_pitch, idx_wp)
        result[idx_wy] = clamp(0.0,         idx_wy)   # radial deviation not estimable

    return result


# ---------------------------------------------------------------------------
# Waist mapping  (3 DOF: yaw, roll, pitch)
# ---------------------------------------------------------------------------

def map_waist(
    jw:      Dict[str, np.ndarray],
    R_torso: np.ndarray,
) -> Dict[int, float]:
    """Return {joint_idx: angle} for the 3 waist joints."""
    ls = _get(jw, "left_shoulder")
    rs = _get(jw, "right_shoulder")
    lh = _get(jw, "left_hip")
    rh = _get(jw, "right_hip")
    if any(v is None for v in (ls, rs, lh, rh)):
        return {}

    hip_line       = normalize(rh - lh)           # pelvis x-axis
    shoulder_line  = normalize(rs - ls)            # shoulder x-axis

    # WaistYaw: horizontal rotation between pelvis and shoulder lines
    # Project both onto the horizontal (XY) plane in torso frame
    hl_t = R_torso.T @ hip_line
    sl_t = R_torso.T @ shoulder_line
    waist_yaw = safe_atan2(
        float(np.cross(hl_t[:2], sl_t[:2])),   # z-component of 2-D cross product
        float(np.dot(hl_t[:2],   sl_t[:2])),
    )

    # WaistPitch: forward tilt of spine
    spine     = normalize(((ls + rs) / 2.0) - ((lh + rh) / 2.0))
    sp_local  = R_torso.T @ spine
    # At neutral upright stance: sp_local ≈ (0, 0, 1) → pitch = 0
    waist_pitch = safe_atan2(sp_local[1], sp_local[2])   # forward lean

    # WaistRoll: lateral tilt
    waist_roll  = safe_atan2(-sp_local[0], sp_local[2])  # side lean

    return {
        G1Joint.WaistYaw:   clamp(waist_yaw,   G1Joint.WaistYaw),
        G1Joint.WaistRoll:  clamp(waist_roll,  G1Joint.WaistRoll),
        G1Joint.WaistPitch: clamp(waist_pitch, G1Joint.WaistPitch),
    }


# ---------------------------------------------------------------------------
# Leg mapping  (6 DOF each: hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
# ---------------------------------------------------------------------------

def map_leg(
    side:     str,
    jw:       Dict[str, np.ndarray],
    R_pelvis: np.ndarray,
) -> Dict[int, float]:
    """Return {joint_idx: angle} for one leg's 6 DOF."""
    hp  = _get(jw, f"{side}_hip")
    kn  = _get(jw, f"{side}_knee")
    an  = _get(jw, f"{side}_ankle")
    hl  = _get(jw, f"{side}_heel")
    fi  = _get(jw, f"{side}_foot_index")

    idx_p, idx_r, idx_y, idx_k, idx_ap, idx_ar = (
        (G1Joint.LeftHipPitch,  G1Joint.LeftHipRoll,
         G1Joint.LeftHipYaw,    G1Joint.LeftKnee,
         G1Joint.LeftAnklePitch, G1Joint.LeftAnkleRoll)
        if side == "left" else
        (G1Joint.RightHipPitch, G1Joint.RightHipRoll,
         G1Joint.RightHipYaw,   G1Joint.RightKnee,
         G1Joint.RightAnklePitch, G1Joint.RightAnkleRoll)
    )

    result: Dict[int, float] = {}

    # --- hip ---
    if hp is not None and kn is not None:
        thigh = normalize(kn - hp)               # thigh vector world frame
        v     = R_pelvis.T @ thigh               # in pelvis frame
        # Leg at rest (standing) points straight down → v ≈ (0, 0, -1)
        hip_pitch = safe_atan2(v[1], -v[2])      # forward swing > 0
        if side == "left":
            hip_roll  = safe_atan2(-v[0], -v[2]) # left-leg out → positive
        else:
            hip_roll  = safe_atan2( v[0], -v[2])
            hip_roll  = -hip_roll

        # HipYaw: compare thigh horizontal projection against pelvis-forward
        thigh_horiz = np.array([v[0], v[1], 0.0])
        if np.linalg.norm(thigh_horiz) > 1e-9:
            ref_fwd   = np.array([0.0, 1.0, 0.0])   # pelvis-forward
            hip_yaw   = safe_atan2(
                float(np.cross(ref_fwd[:2], normalize(thigh_horiz)[:2])),
                float(np.dot(  ref_fwd[:2], normalize(thigh_horiz)[:2])),
            )
        else:
            hip_yaw = 0.0

        result[idx_p] = clamp(hip_pitch, idx_p)
        result[idx_r] = clamp(hip_roll,  idx_r)
        result[idx_y] = clamp(hip_yaw,   idx_y)

    # --- knee ---
    if hp is not None and kn is not None and an is not None:
        knee_interior = angle_between(hp - kn, an - kn)
        knee_q        = np.pi - knee_interior      # 0 = extended, pi/2 = 90° bent
        result[idx_k] = clamp(knee_q, idx_k)

    # --- ankle ---
    if kn is not None and an is not None:
        shin = normalize(an - kn)                  # shin direction world
        sv   = R_pelvis.T @ shin                   # in pelvis frame

        ankle_pitch = safe_atan2(sv[1], -sv[2])    # dorsiflexion / plantarflexion
        ankle_roll  = safe_atan2(-sv[0], -sv[2])   # inversion / eversion

        # Refine pitch using foot vector if available
        if hl is not None and fi is not None:
            foot = normalize(fi - hl)
            fv   = R_pelvis.T @ foot
            ankle_pitch = safe_atan2(fv[1], fv[0])

        result[idx_ap] = clamp(ankle_pitch, idx_ap)
        result[idx_ar] = clamp(ankle_roll,  idx_ar)

    return result


# ---------------------------------------------------------------------------
# Per-frame mapping
# ---------------------------------------------------------------------------

def _extract_world_positions(
    sample_joints: Dict[str, Dict],
) -> Dict[str, np.ndarray]:
    """Pull world_xyz or depth_xyz for every landmark into a flat dict."""
    out: Dict[str, np.ndarray] = {}
    for name, data in sample_joints.items():
        xyz = data.get("world_xyz") or data.get("depth_xyz")
        if xyz is not None:
            out[name] = np.array(xyz, dtype=float)
    return out


def map_frame(
    sample:     Dict,
    include_legs:  bool = True,
    include_waist: bool = True,
    dof23:         bool = False,
) -> Dict[int, float]:
    """Map one recorded pose sample to a dict {G1_joint_idx: angle_rad}."""
    jw = _extract_world_positions(sample["joints"])

    angles: Dict[int, float] = {}

    R_torso  = build_torso_frame(jw)
    R_pelvis = build_pelvis_frame(jw)

    if R_torso is not None:
        # Arms
        angles.update(map_arm("left",  jw, R_torso))
        angles.update(map_arm("right", jw, R_torso))

        # Waist
        if include_waist:
            angles.update(map_waist(jw, R_torso))
            if dof23:
                # Remove joints invalid on 23-DOF
                for j in (G1Joint.WaistRoll, G1Joint.WaistPitch,
                          G1Joint.LeftWristPitch,  G1Joint.LeftWristYaw,
                          G1Joint.RightWristPitch, G1Joint.RightWristYaw):
                    angles.pop(j, None)

    # Legs
    if include_legs and R_pelvis is not None:
        angles.update(map_leg("left",  jw, R_pelvis))
        angles.update(map_leg("right", jw, R_pelvis))

    return angles


# ---------------------------------------------------------------------------
# Load / save
# ---------------------------------------------------------------------------

def load_recording(path: Path) -> Dict:
    raw = json.loads(path.read_text())
    # Accept both vision_module_v2 and older formats
    if "samples" not in raw:
        raise ValueError(f"Unexpected recording format in {path}")
    return raw


def save_npz(
    joint_indices: List[int],
    ts_arr:        np.ndarray,
    qs_arr:        np.ndarray,
    output:        Path,
) -> None:
    """Write NPZ compatible with pbd_reproduce.py."""
    poll_s = float(np.mean(np.diff(ts_arr))) if len(ts_arr) > 1 else 0.02
    np.savez(
        output,
        joints         = np.array(joint_indices, dtype=np.int32),
        ts             = ts_arr.astype(np.float32),
        qs             = qs_arr.astype(np.float32),
        fk_qs          = qs_arr.astype(np.float32),
        poll_s         = np.array(poll_s, dtype=np.float32),
        representation = np.array("joint_space"),
    )
    print(
        f"[mapper] saved {qs_arr.shape[0]} frames × {qs_arr.shape[1]} joints"
        f" → {output}"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(
    input_path:    Path,
    output_path:   Path,
    include_legs:  bool = True,
    include_waist: bool = True,
    dof23:         bool = False,
    verbose:       bool = False,
) -> None:
    recording = load_recording(input_path)
    samples   = recording["samples"]
    if not samples:
        raise SystemExit("Recording is empty.")

    print(f"[mapper] {len(samples)} frames from {input_path}")

    # --- First pass: collect all joint indices that appear ---------------
    # Map every frame to get which joints are actually produced
    first_angles = map_frame(
        samples[0],
        include_legs=include_legs,
        include_waist=include_waist,
        dof23=dof23,
    )
    joint_indices = sorted(first_angles.keys())
    J = len(joint_indices)
    col_of = {j: i for i, j in enumerate(joint_indices)}

    print(f"[mapper] mapping {J} G1 joints: {joint_indices}")

    ts_list: List[float] = []
    qs_rows: List[np.ndarray] = []

    for i, sample in enumerate(samples):
        angles = map_frame(
            sample,
            include_legs=include_legs,
            include_waist=include_waist,
            dof23=dof23,
        )
        row = np.zeros(J, dtype=float)
        for j_idx, angle in angles.items():
            if j_idx in col_of:
                row[col_of[j_idx]] = angle

        ts_list.append(float(sample["t"]))
        qs_rows.append(row)

        if verbose and i % max(1, len(samples) // 10) == 0:
            summary = "  ".join(
                f"j{j}={row[col_of[j]]:+.2f}" for j in joint_indices
            )
            print(f"  [{i:4d}] t={sample['t']:.3f}s  {summary}")

    ts_arr = np.array(ts_list, dtype=np.float32)
    qs_arr = np.array(qs_rows, dtype=np.float32)

    save_npz(joint_indices, ts_arr, qs_arr, output_path)

    print(
        f"[mapper] done.  Replay with:\n"
        f"  python3 ../../arm_motion/pbd/pbd_reproduce.py"
        f" --file {output_path}"
        f" --arm both --mode joint"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Map vision_module.py pose recording to G1 joint angles.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input",     type=Path, default=Path("pose_recording.json"),
                        help="Input JSON from vision_module.py")
    parser.add_argument("--output",    type=Path, default=Path("g1_motion.npz"),
                        help="Output NPZ for pbd_reproduce.py")
    parser.add_argument("--no-legs",   action="store_true",
                        help="Skip all leg joints")
    parser.add_argument("--no-waist",  action="store_true",
                        help="Skip waist joints")
    parser.add_argument("--dof23",     action="store_true",
                        help="23-DOF safe mode: skip WaistRoll/Pitch and WristPitch/Yaw")
    parser.add_argument("--verbose",   action="store_true",
                        help="Print per-frame angle summaries")
    args = parser.parse_args()

    if not args.input.exists():
        sys.exit(f"Input file not found: {args.input}")

    try:
        run(
            input_path    = args.input,
            output_path   = args.output,
            include_legs  = not args.no_legs,
            include_waist = not args.no_waist,
            dof23         = args.dof23,
            verbose       = args.verbose,
        )
    except (ValueError, SystemExit) as err:
        sys.exit(str(err))
