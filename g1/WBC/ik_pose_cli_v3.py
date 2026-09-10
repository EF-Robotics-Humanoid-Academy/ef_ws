#!/usr/bin/env python3
"""
6D End-Effector IK Pose Control TUI — v3.

Controls the absolute 6D Cartesian pose (x, y, z, roll, pitch, yaw) of the
chosen hand end-effector (left / right / both) by running damped-least-squares
IK over the 7 arm DOFs (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw).

Each key press:
  1. Increments the EE target by the configured step.
  2. Runs warm-started DLS IK from the previous joint solution.
  3. Clamps each resulting joint delta to ±max_dq (default 0.2 rad).
  4. Queues the clamped solution as the ramp target.
  5. The control loop ramps current joints smoothly toward the target at
     max_speed r/s with uniform servo gains throughout.

If IK fails the EE target is rolled back, joints do not move.

Orientation stiffness (f):
  ON  — roll/pitch/yaw are locked when adjusting x/y/z (full 6D IK, default).
  OFF — orientation is free during x/y/z moves; target rotation follows current
        FK and position IK only moves shoulder pitch/roll/yaw + elbow. Near the
        workspace boundary, the selected position axis may be accepted while the
        other position axes are clamped back to the nearest reached FK pose.

Key bindings
────────────
  ↑ / ↓  or  k / j    select DOF (x y z roll pitch yaw)
  ← / →  or  - / +    decrement / increment selected DOF by step
  < / >                halve / double EE step for selected DOF class
  [ / ]                halve / double max joint-delta per IK step
  m                    cycle arm mode: both → left → right
  f                    toggle orientation stiffness (ON/OFF)
  y                    sync EE targets → current FK pose (resync)
  r                    release arms (zero servo gains)
  e                    reengage arms and resync to live pose
  z                    send zero-gain hold packet once
  s                    set ramp speed rad/s (prompt)
  d                    set max joint-delta rad (prompt)
  W                    set waist pitch/roll kp (prompt)
  w                    toggle waist hold on/off
  H                    cycle Dex3 hand mode: off → right → left → both → follow
  { / }                open / close Dex3 hand grip by 5%
  g                    set Dex3 hand grip percent (prompt)
  Tab                  cycle focus: DOF → poses → sequence
  p                    save current upper-body joint state as a named pose
  (poses focus)
    l / Enter          load selected saved joint pose
    d                  delete selected saved joint pose
    a                  add selected pose to replay sequence
  (sequence focus)
    w                  toggle waist inclusion for newly added sequence steps
    g                  set sequence gap seconds
    x / Delete         remove selected sequence step
    u / n              move selected step up / down
    R / S              run / stop sequence
  q / Esc              quit
"""
from __future__ import annotations

import argparse
import atexit
import curses
import fcntl
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
_PATH_CANDIDATES = (
    SCRIPT_DIR,
    os.path.join(SCRIPT_DIR, "modules"),
    ROOT_DIR,
    os.path.join(ROOT_DIR, "modules"),
)
for _p in reversed(_PATH_CANDIDATES):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from hand_pose_navigation_copy.arm_fk import JOINT_LIMITS
    from hand_pose_navigation_copy.arm_ik import ArmIK
    from hand_pose_navigation_copy.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
        _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
    )
except ModuleNotFoundError:
    from hand_pose_navigation.arm_fk import JOINT_LIMITS
    from hand_pose_navigation.arm_ik import ArmIK
    from hand_pose_navigation.arm_fk import (
        ArmFK, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
        _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
    )

from sdk_hand import Dex3HandController, hand_grip_targets
from dds_env import (
    default_dds_iface,
    ensure_channel_factory_initialized,
    ensure_cyclonedds_environment,
)

ensure_cyclonedds_environment()

try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    raise SystemExit("unitree_sdk2py not installed.") from exc


# ── Joint indices ─────────────────────────────────────────────────────────────
WAIST_JOINTS = [12, 13, 14]   # roll, pitch, yaw
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

ARM_SDK_WEIGHT_INDEX = 29
WAIST_HOLD_KP = 480.0   # yaw hold kp
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
DEFAULT_WAIST_PR_KP = 200.0   # pitch + roll hold kp

ARM_CONTROL_MODES = ("both", "left", "right")
HAND_CONTROL_MODES = ("off", "right", "left", "both", "follow-arm")
ARM_JOINTS: Dict[str, List[int]] = {
    "left": LEFT_ARM_JOINTS,
    "right": RIGHT_ARM_JOINTS,
}
JOINT_LABELS = ("sh_p", "sh_r", "sh_y", "elbow", "wr_r", "wr_p", "wr_y")
SHOULDER_ELBOW_IDXS = (0, 1, 2, 3)
POSITION_IK_TOL_M = 0.005
POSITION_IK_AXIS_TOL_M = 0.006
POSITION_IK_SOFT_LIMIT_M = 0.040

_SHOULDER_ORIGIN: Dict[str, np.ndarray] = {
    "left": _LEFT_SHOULDER_IN_BASE,
    "right": _RIGHT_SHOULDER_IN_BASE,
}

# ── DOF table ─────────────────────────────────────────────────────────────────
DOF_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
DOF_UNITS = ("m", "m", "m", "rad", "rad", "rad")
N_DOFS = 6

# ── Colour pairs ──────────────────────────────────────────────────────────────
C_GREEN = 1
C_YELLOW = 2
C_RED = 3
C_CYAN = 4
C_SEL = 5
C_BOLD = 6
C_FOCUS = 7
C_RUNNING = 8

FOCUS_DOF = 0
FOCUS_POSES = 1
FOCUS_SEQUENCE = 2

DEFAULT_POSE_FILE = os.path.join(SCRIPT_DIR, "saved_ik_pose_cli_v3_poses.json")
DEFAULT_LOCK_FILE = "/tmp/ik_pose_cli_v3_rt_arm_sdk.lock"

# ── Rotation helpers ──────────────────────────────────────────────────────────

def _Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _Ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _Rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


_ROT_BY_AXIS = (_Rx, _Ry, _Rz)


def _rpy_from_R(R: np.ndarray) -> Tuple[float, float, float]:
    """ZYX Euler angles (roll, pitch, yaw) from 3×3 rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _clamp_q(q: np.ndarray, arm: str) -> np.ndarray:
    limits = JOINT_LIMITS[arm]
    lo = np.array([lim[0] for lim in limits], dtype=np.float64)
    hi = np.array([lim[1] for lim in limits], dtype=np.float64)
    return np.clip(q, lo, hi)


# ── Home EE pose (after reengage) ─────────────────────────────────────────────
_HOME_EE_LOCAL = (0.0, 0.0, -0.4, 0.0, -math.pi / 2, 0.0)


def _make_home_T(arm: str) -> np.ndarray:
    """4×4 home EE pose in base_link: 0.4 m below shoulder, hand pointing forward."""
    x, y, z, roll, pitch, yaw = _HOME_EE_LOCAL
    y_sign = -1.0 if arm == "right" else 1.0
    pos = _SHOULDER_ORIGIN[arm] + np.array([x, y_sign * y, z])
    R = _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T


# ── Robot infrastructure ──────────────────────────────────────────────────────

def _resolve_lowstate_type():
    for path in (
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
    ):
        try:
            mod = __import__(path, fromlist=["LowState_"])
            if hasattr(mod, "LowState_"):
                return getattr(mod, "LowState_")
        except Exception:
            pass
    return None


class UpperBodyStateSubscriber:
    def __init__(self, joints: List[int]) -> None:
        self._joints = [int(j) for j in joints]
        self._lock = threading.Lock()
        self._pos: Dict[int, float] = {}
        self._ts = 0.0
        t = _resolve_lowstate_type()
        if t is None:
            raise RuntimeError("LowState_ not found in unitree_sdk2py.")
        sub = ChannelSubscriber("rt/lowstate", t)
        sub.Init(self._on_msg, 200)

    def _on_msg(self, msg: Any) -> None:
        try:
            pos = {j: float(msg.motor_state[j].q) for j in self._joints}
        except Exception:
            return
        with self._lock:
            self._pos = pos
            self._ts = time.time()

    def snapshot(self) -> Optional[Tuple[Dict[int, float], float]]:
        with self._lock:
            if not self._pos:
                return None
            return dict(self._pos), float(self._ts)


class ArmSDKPublisher:
    """Publishes upper-body targets via rt/arm_sdk."""

    def __init__(self) -> None:
        self._crc = CRC()
        self._pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = 0
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = 1.0

    def publish(
        self,
        targets: Dict[int, float],
        *,
        arm_kp: float,
        arm_kd: float,
        waist_pr_kp: float,   # pitch (13) and roll (12) kp
        waist_y_kp: float,    # yaw (14) kp
        waist_kd: float,
        arm_sdk_weight: float = 1.0,
    ) -> None:
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(targets[j])
            c.dq = 0.0
            c.tau = 0.0
            if j in (12, 13):           # waist roll and pitch
                c.kp = float(waist_pr_kp)
                c.kd = float(waist_kd)
            elif j == 14:               # waist yaw
                c.kp = float(waist_y_kp)
                c.kd = float(waist_kd)
            else:
                c.kp = float(arm_kp)
                c.kd = float(arm_kd)
        self._cmd.motor_cmd[ARM_SDK_WEIGHT_INDEX].q = float(arm_sdk_weight)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def publish_zero_gains(self, hold: Dict[int, float]) -> None:
        for j in UPPER_BODY_JOINTS:
            c = self._cmd.motor_cmd[j]
            c.mode = 1
            c.q = float(hold[j])
            c.dq = 0.0
            c.kp = 0.0
            c.kd = 0.0
            c.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class ControllerLockError(RuntimeError):
    pass


class SingleControllerLock:
    """Process lock so only one TUI publishes rt/arm_sdk targets at a time."""

    def __init__(self, path: str = DEFAULT_LOCK_FILE) -> None:
        self.path = str(path)
        self._fd: Optional[int] = None

    def acquire(self) -> None:
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            # The lock is deliberately shared by all workshop accounts.  The
            # process holding it is protected by flock; write permission is
            # also needed by the next participant to update the owner record.
            # Do this explicitly because the user's umask otherwise turns a
            # newly created 0666 file into 0644.
            os.fchmod(fd, 0o666)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            owner = self._read_owner(fd)
            os.close(fd)
            raise ControllerLockError(
                "Another ik_pose_cli_v3.py session already owns rt/arm_sdk"
                + (f" ({owner})" if owner else "")
                + f". Stop that session first, or shut down Jupyter if it is stale. Lock: {self.path}"
            ) from exc
        os.ftruncate(fd, 0)
        owner = (
            f"pid={os.getpid()} "
            f"cwd={os.getcwd()} "
            f"started={datetime.now(timezone.utc).isoformat()}\n"
        )
        os.write(fd, owner.encode("ascii"))
        self._fd = fd

    @staticmethod
    def _read_owner(fd: int) -> str:
        try:
            os.lseek(fd, 0, os.SEEK_SET)
            return os.read(fd, 512).decode("utf-8", "replace").strip()
        except Exception:
            return ""

    def release(self) -> None:
        fd = self._fd
        if fd is None:
            return
        self._fd = None
        try:
            os.ftruncate(fd, 0)
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


# ── Main TUI ──────────────────────────────────────────────────────────────────

class IKPoseCLI:
    def __init__(self, args: argparse.Namespace) -> None:
        self._closed = False
        self._closing = False
        self._controller_lock = SingleControllerLock()
        self._controller_lock.acquire()
        self.iface = str(args.iface)
        self.domain_id = int(args.domain_id)
        self.pose_path = Path(os.path.abspath(os.path.expanduser(str(args.file))))
        self.rate_hz = max(1.0, float(args.rate_hz))
        self.max_speed = max(0.01, float(args.speed_rad_s))
        self.arm_kp = float(args.kp)
        self.arm_kd = float(args.kd)
        self.waist_pr_kp = float(args.waist_pr_kp)   # pitch + roll
        self.waist_y_kp = float(WAIST_HOLD_KP)      # yaw (fixed)
        self.waist_kd = float(WAIST_HOLD_KD)
        self.waist_enabled = True
        self.arm_control_mode = str(args.arm_control)
        self.hand_control_mode = str(args.hand_control)
        self.hand_grip_percent = max(0.0, min(100.0, float(args.hand_grip)))
        self.hand_kp = float(args.hand_kp)
        self.hand_kd = float(args.hand_kd)
        self.hand_tau = float(args.hand_tau)
        self.hand_rate_hz = max(0.1, float(args.hand_rate_hz))
        self.hand_write_timeout_s = max(0.0, float(args.hand_write_timeout_s))
        self._last_hand_publish_s = 0.0

        # EE step sizes (per key-press)
        self.pos_step = 0.01    # metres
        self.rot_step = 0.05    # radians

        # Maximum joint angle change applied per IK solve (safety clamp)
        self.max_dq = float(args.max_dq)

        # Orientation stiffness: when True, x/y/z moves keep rotation locked.
        # When False, target rotation follows current FK so IK is position-only.
        self.orient_stiff = True

        self.latest_positions: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}

        self.seeded = False
        self.armed = True
        self._running = True
        self.status = "Waiting for rt/lowstate…"

        self.dof_idx = 0
        self.focus = FOCUS_DOF

        self.saved_poses: List[Dict[str, Any]] = []
        self.sequence_steps: List[Dict[str, Any]] = []
        self.pose_cursor = 0
        self.seq_cursor = 0
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.sequence_gap_s = 2.0
        self.include_waist_new = True

        self.target_T: Dict[str, np.ndarray] = {
            "left": np.eye(4, dtype=np.float64),
            "right": np.eye(4, dtype=np.float64),
        }
        self._home_T: Dict[str, np.ndarray] = {
            arm: _make_home_T(arm) for arm in ("left", "right")
        }

        self._fk: Dict[str, ArmFK] = {
            "left": ArmFK("left", "urdf"),
            "right": ArmFK("right", "urdf"),
        }
        self._ik: Dict[str, ArmIK] = {
            "left": ArmIK("left", "dls", max_iter=24, tol_pos_m=0.005, tol_rot_rad=0.02),
            "right": ArmIK("right", "dls", max_iter=24, tol_pos_m=0.005, tol_rot_rad=0.02),
        }
        self.ik_info: Dict[str, Dict] = {
            "left": {"success": None, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0},
            "right": {"success": None, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0},
        }
        self.hand_info: Dict[str, str] = {"left": "off", "right": "off"}

        ensure_channel_factory_initialized(self.domain_id, self.iface)
        self.state_sub = UpperBodyStateSubscriber(UPPER_BODY_JOINTS)
        self.pub = ArmSDKPublisher()
        self.hand_controllers: Dict[str, Dex3HandController] = {}
        self._init_hand_controllers()

        self._load_pose_file()
        self._seed_from_state()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    # ── Initialisation ────────────────────────────────────────────────────────

    def _seed_from_state(self) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self.seeded = True
                self._sync_ee_from_joints()
                self.status = f"Connected on {self.iface}"
                return
            time.sleep(0.02)

    def _sync_ee_from_joints(self) -> None:
        """Recompute EE target poses via FK from current desired joint angles."""
        for arm, joints in ARM_JOINTS.items():
            q = np.array([self.desired_targets[j] for j in joints])
            self.target_T[arm] = self._fk[arm].compute_arm(q).copy()

    def _sync_targets_to_live(self) -> None:
        """Use the latest measured joints as both ramp state and EE target."""
        snap = self.state_sub.snapshot()
        if snap:
            pos, _ = snap
            self._set_targets_to_positions(pos)
        else:
            self._set_targets_to_positions(self.latest_positions)

    def _set_targets_to_positions(self, positions: Dict[int, float]) -> None:
        """Use measured joints as the held command target and EE IK target."""
        self.latest_positions = dict(positions)
        self.current_targets = dict(positions)
        self.desired_targets = dict(positions)
        self._sync_ee_from_joints()
        for arm in ("left", "right"):
            self.ik_info[arm] = {
                "success": None,
                "error_pos_m": 0.0,
                "error_rot_rad": 0.0,
                "iterations": 0,
            }

    def _wait_for_state(self, timeout: float = 2.0) -> Dict[int, float]:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            snap = self.state_sub.snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                return dict(pos)
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")

    def _release_arms(self, duration_s: float = 3.0, command_rate_hz: float = 50.0) -> None:
        positions = self._wait_for_state(timeout=3.0)
        self._set_targets_to_positions(positions)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        base_waist_pr_kp = self.waist_pr_kp if self.waist_enabled else 0.0
        base_waist_y_kp = self.waist_y_kp if self.waist_enabled else 0.0
        base_waist_kd = self.waist_kd if self.waist_enabled else 0.0
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            fade = ratio * ratio * (3.0 - 2.0 * ratio)
            authority = 1.0 - fade
            self.pub.publish(
                positions,
                arm_kp=self.arm_kp * authority,
                arm_kd=self.arm_kd * authority,
                waist_pr_kp=base_waist_pr_kp * authority,
                waist_y_kp=base_waist_y_kp * authority,
                waist_kd=base_waist_kd * authority,
                arm_sdk_weight=authority,
            )
            time.sleep(dt)

    def _unrelease_arms(self, duration_s: float = 1.0, command_rate_hz: float = 50.0) -> None:
        positions = self._wait_for_state(timeout=3.0)
        self._set_targets_to_positions(positions)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        waist_pr_kp = self.waist_pr_kp if self.waist_enabled else 0.0
        waist_y_kp = self.waist_y_kp if self.waist_enabled else 0.0
        waist_kd = self.waist_kd if self.waist_enabled else 0.0
        for step_idx in range(steps + 1):
            authority = float(step_idx) / float(steps)
            self.pub.publish(
                positions,
                arm_kp=self.arm_kp,
                arm_kd=self.arm_kd,
                waist_pr_kp=waist_pr_kp,
                waist_y_kp=waist_y_kp,
                waist_kd=waist_kd,
                arm_sdk_weight=authority,
            )
            time.sleep(dt)

    # ── Pose file and sequence helpers ────────────────────────────────────────

    def _load_pose_file(self) -> None:
        self.saved_poses = []
        self.sequence_steps = []
        if self.pose_path.exists():
            try:
                payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
                poses = payload.get("poses", [])
                if isinstance(poses, list):
                    self.saved_poses = [
                        p for p in poses
                        if isinstance(p, dict) and isinstance(p.get("arm_joints"), dict)
                    ]
                seq = payload.get("sequence", [])
                if isinstance(seq, list):
                    self.sequence_steps = [
                        {
                            "pose_index": int(s.get("pose_index", -1)),
                            "include_waist": bool(s.get("include_waist", True)),
                        }
                        for s in seq
                        if isinstance(s, dict)
                    ]
            except Exception as exc:
                self.status = f"Could not read pose file: {exc}"
        self.sequence_steps = [
            s for s in self.sequence_steps
            if 0 <= int(s.get("pose_index", -1)) < len(self.saved_poses)
        ]
        self.pose_cursor = min(self.pose_cursor, max(0, len(self.saved_poses) - 1))
        self.seq_cursor = min(self.seq_cursor, max(0, len(self.sequence_steps) - 1))

    def _write_pose_file(self) -> None:
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(
            json.dumps(
                {"poses": self.saved_poses, "sequence": self.sequence_steps},
                indent=2,
                sort_keys=True,
            ) + "\n",
            encoding="utf-8",
        )

    def _pose_payload(self, name: str) -> Dict[str, Any]:
        # Save the commanded target, not lowstate feedback. With a payload in the
        # hand, feedback can sit below the target due to compliance/sag; saving it
        # would discard any intentional lift compensation taught through IK.
        src = self.desired_targets if self.seeded else self.current_targets
        return {
            "name": name,
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {
                str(j): float(src[j])
                for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
            },
            "waist_joints": {str(j): float(src[j]) for j in WAIST_JOINTS},
        }

    def _apply_joint_pose(self, pose: Dict[str, Any], *, include_waist: bool = True) -> None:
        arm_joints = pose.get("arm_joints")
        if not isinstance(arm_joints, dict):
            raise ValueError("pose missing arm_joints")
        for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            k = str(j)
            if k in arm_joints:
                self.desired_targets[j] = float(arm_joints[k])
        waist_joints = pose.get("waist_joints")
        if include_waist and isinstance(waist_joints, dict):
            for j in WAIST_JOINTS:
                k = str(j)
                if k in waist_joints:
                    self.desired_targets[j] = float(waist_joints[k])
        self._sync_ee_from_joints()
        for arm in ("left", "right"):
            self.ik_info[arm] = {
                "success": None,
                "error_pos_m": 0.0,
                "error_rot_rad": 0.0,
                "iterations": 0,
            }

    def _targets_reached(self, eps: float = 0.01) -> bool:
        return all(
            abs(float(self.current_targets[j]) - float(self.desired_targets[j])) <= eps
            for j in UPPER_BODY_JOINTS
        )

    def _advance_sequence(self, now: float) -> None:
        if not self.sequence_running:
            return
        if self.sequence_step_index >= len(self.sequence_steps):
            self.sequence_running = False
            self.status = "Sequence completed"
            return
        if self.sequence_next_time_s < 0.0:
            if not self._targets_reached():
                return
            self.sequence_next_time_s = now + max(0.0, self.sequence_gap_s)
            return
        if self.sequence_next_time_s > 0.0 and now < self.sequence_next_time_s:
            return

        step = self.sequence_steps[self.sequence_step_index]
        pose_index = int(step.get("pose_index", -1))
        if not (0 <= pose_index < len(self.saved_poses)):
            self.sequence_running = False
            self.status = "Sequence stopped: missing pose"
            return
        pose = self.saved_poses[pose_index]
        include_waist = bool(step.get("include_waist", True))
        try:
            self._apply_joint_pose(pose, include_waist=include_waist)
        except Exception as exc:
            self.sequence_running = False
            self.status = f"Sequence error: {exc}"
            return
        self.seq_cursor = self.sequence_step_index
        self.sequence_step_index += 1
        self.sequence_next_time_s = -1.0 if not self._targets_reached() else now + max(0.0, self.sequence_gap_s)
        waist_text = "waist on" if include_waist else "arms only"
        self.status = (
            f"Seq step {self.sequence_step_index}/{len(self.sequence_steps)}: "
            f"{pose.get('name', '<unnamed>')} ({waist_text})"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _active_arms(self) -> List[str]:
        return ["left", "right"] if self.arm_control_mode == "both" else [self.arm_control_mode]

    def _active_hands(self) -> List[str]:
        if self.hand_control_mode == "off":
            return []
        if self.hand_control_mode == "both":
            return ["left", "right"]
        if self.hand_control_mode == "follow-arm":
            return self._active_arms()
        return [self.hand_control_mode]

    def _init_hand_controllers(self) -> None:
        needed = set(self._active_hands())
        for hand in ("left", "right"):
            if hand in needed and hand not in self.hand_controllers:
                try:
                    self.hand_controllers[hand] = Dex3HandController(
                        hand=hand,
                        iface=self.iface,
                        domain_id=self.domain_id,
                    )
                    self.hand_info[hand] = "ready"
                except Exception as exc:
                    self.hand_info[hand] = f"init failed: {exc}"

    def _ensure_hand_control_for_grip(self) -> None:
        """Enable intuitive grip control when the hand mode is still off."""
        if self.hand_control_mode == "off":
            self.hand_control_mode = "follow-arm"
            self._init_hand_controllers()

    def _display_arm(self) -> str:
        return "right" if self.arm_control_mode == "right" else "left"

    def _ee_step(self, idx: Optional[int] = None) -> float:
        i = self.dof_idx if idx is None else idx
        return self.pos_step if i < 3 else self.rot_step

    def _set_ee_step(self, value: float) -> None:
        if self.dof_idx < 3:
            self.pos_step = value
        else:
            self.rot_step = value

    def _fk_live(self, arm: str) -> np.ndarray:
        joints = ARM_JOINTS[arm]
        q = np.array([self.latest_positions.get(j, self.current_targets[j]) for j in joints])
        return self._fk[arm].compute_arm(q)

    def _fk_desired(self, arm: str) -> np.ndarray:
        joints = ARM_JOINTS[arm]
        q = np.array([self.desired_targets[j] for j in joints])
        return self._fk[arm].compute_arm(q)

    def _solve_position_shoulder_elbow(
        self,
        arm: str,
        T_des: np.ndarray,
        q_init: np.ndarray,
        *,
        selected_axis: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], Dict]:
        """Position-only DLS with wrist joints held fixed at q_init[4:7]."""
        q = _clamp_q(q_init.copy(), arm)
        lam = 0.05
        eps = 1e-5
        max_iter = 64
        tol = POSITION_IK_TOL_M
        active = SHOULDER_ELBOW_IDXS
        best_q = q.copy()
        best_err_pos = float("inf")
        best_axis_err = float("inf")

        for iteration in range(max_iter):
            T_cur = self._fk[arm].compute_arm(q)
            pos_err = T_des[:3, 3] - T_cur[:3, 3]
            err_pos = float(np.linalg.norm(pos_err))
            axis_err = (
                abs(float(pos_err[selected_axis]))
                if selected_axis is not None else err_pos
            )
            if (err_pos, axis_err) < (best_err_pos, best_axis_err):
                best_q = q.copy()
                best_err_pos = err_pos
                best_axis_err = axis_err
            if err_pos < tol:
                return q, {
                    "success": True,
                    "error_pos_m": err_pos,
                    "error_rot_rad": 0.0,
                    "iterations": iteration,
                    "mode": "pos_shoulder_elbow",
                }

            J = np.zeros((3, len(active)), dtype=np.float64)
            p0 = T_cur[:3, 3]
            for col, idx in enumerate(active):
                q1 = q.copy()
                q1[idx] += eps
                T1 = self._fk[arm].compute_arm(q1)
                J[:, col] = (T1[:3, 3] - p0) / eps

            JJT = J @ J.T
            dq_active = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(3), pos_err)
            norm_dq = float(np.linalg.norm(dq_active))
            if norm_dq > 0.3:
                dq_active *= 0.3 / norm_dq

            q_next = q.copy()
            for col, idx in enumerate(active):
                q_next[idx] += dq_active[col]
            q = _clamp_q(q_next, arm)
            q[4:] = q_init[4:]

        T_cur = self._fk[arm].compute_arm(best_q)
        err_pos = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
        axis_err = (
            abs(float(T_des[selected_axis, 3] - T_cur[selected_axis, 3]))
            if selected_axis is not None else err_pos
        )
        if (
            selected_axis is not None
            and axis_err < POSITION_IK_AXIS_TOL_M
            and err_pos < POSITION_IK_SOFT_LIMIT_M
        ):
            return best_q, {
                "success": True,
                "error_pos_m": err_pos,
                "error_rot_rad": 0.0,
                "iterations": max_iter,
                "mode": "pos_axis_clamped",
                "axis_error_m": axis_err,
            }
        return None, {
            "success": False,
            "error_pos_m": err_pos,
            "error_rot_rad": 0.0,
            "iterations": max_iter,
            "mode": "pos_shoulder_elbow",
        }

    # ── IK and joint application ──────────────────────────────────────────────

    def _adjust_dof(self, delta: float) -> None:
        """Increment the selected DOF on each active arm's EE target, then solve IK."""
        for arm in self._active_arms():
            T_prev = self.target_T[arm].copy()

            arm_delta = delta
            if self.dof_idx == 1:
                arm_delta = delta if arm == "left" else -delta
            elif self.dof_idx in (3, 5):
                arm_delta = -delta if arm == "left" else delta
            elif self.dof_idx == 4:
                arm_delta = -delta

            shoulder_elbow_only = self.dof_idx < 3 and not self.orient_stiff
            scales = (1.0, 0.5, 0.25, 0.1)
            for scale in scales:
                T_new = T_prev.copy()
                scaled_delta = arm_delta * scale

                if self.dof_idx < 3:
                    # Position move: optionally free the orientation target so IK
                    # only needs to satisfy position (orient_stiff = False).
                    if not self.orient_stiff:
                        T_new[:3, :3] = self._fk_desired(arm)[:3, :3]
                    T_new[self.dof_idx, 3] += scaled_delta
                else:
                    axis = self.dof_idx - 3
                    T_new[:3, :3] = _ROT_BY_AXIS[axis](scaled_delta) @ T_new[:3, :3]

                self.target_T[arm] = T_new
                if self._apply_ik(
                    arm,
                    T_prev,
                    shoulder_elbow_only=shoulder_elbow_only,
                    selected_axis=self.dof_idx if shoulder_elbow_only else None,
                ):
                    if scale < 1.0:
                        self.status = (
                            f"{arm} IK accepted {scale:.0%} step; "
                            "near limit or singularity"
                        )
                    break
            else:
                self.target_T[arm] = T_prev

    def _apply_ik(
        self,
        arm: str,
        T_prev: np.ndarray,
        *,
        shoulder_elbow_only: bool = False,
        selected_axis: Optional[int] = None,
    ) -> bool:
        """
        Solve IK for arm toward self.target_T[arm].

        On success: write max_dq-clamped joint deltas into desired_targets.
        On failure: return False (caller reverts target_T).
        """
        joints = ARM_JOINTS[arm]
        q_init = np.array([self.desired_targets[j] for j in joints])

        if shoulder_elbow_only:
            q_sol, info = self._solve_position_shoulder_elbow(
                arm,
                self.target_T[arm],
                q_init,
                selected_axis=selected_axis,
            )
        else:
            q_sol, info = self._ik[arm].solve(self.target_T[arm], q_init=q_init)
        self.ik_info[arm] = info

        if q_sol is None:
            return False

        delta = q_sol - q_init
        delta = np.clip(delta, -self.max_dq, self.max_dq)
        q_apply = q_init + delta

        for i, j in enumerate(joints):
            self.desired_targets[j] = float(q_apply[i])
        self.target_T[arm] = self._fk[arm].compute_arm(q_apply).copy()
        return True

    # ── Control loop ──────────────────────────────────────────────────────────

    def _ramp_step(self, dt: float) -> None:
        step = max(1e-9, self.max_speed * dt)
        for j in UPPER_BODY_JOINTS:
            cur = float(self.current_targets[j])
            des = float(self.desired_targets[j])
            d = des - cur
            if abs(d) <= step:
                self.current_targets[j] = des
            else:
                self.current_targets[j] = cur + math.copysign(step, d)

    def tick(self) -> None:
        snap = self.state_sub.snapshot()
        if snap is not None:
            pos, _ = snap
            self.latest_positions = pos
            if not self.seeded:
                self.seeded = True
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self._sync_ee_from_joints()

        if not self.seeded or not self.armed:
            return

        now = time.monotonic()
        dt = min(1.0 / self.rate_hz, now - self._last_tick)
        self._last_tick = now

        self._advance_sequence(now)
        self._ramp_step(dt)
        try:
            self.pub.publish(
                self.current_targets,
                arm_kp=self.arm_kp,
                arm_kd=self.arm_kd,
                waist_pr_kp=self.waist_pr_kp if self.waist_enabled else 0.0,
                waist_y_kp=self.waist_y_kp if self.waist_enabled else 0.0,
                waist_kd=self.waist_kd if self.waist_enabled else 0.0,
            )
        except Exception as exc:
            self.status = f"Publish error: {exc}"
        if (now - self._last_hand_publish_s) >= (1.0 / self.hand_rate_hz):
            self._publish_hand_targets_once()
            self._last_hand_publish_s = now
        stiff_txt = "stiff:ON" if self.orient_stiff else "stiff:OFF"
        hand_txt = (
            "hand:OFF"
            if self.hand_control_mode == "off"
            else f"hand:{self.hand_control_mode} {self.hand_grip_percent:.0f}%"
        )
        self.status = (
            f"Publishing {self.rate_hz:.0f} Hz  "
            f"ramp {self.max_speed:.3f} r/s  "
            f"max_dq {self.max_dq:.3f} rad  "
            f"arm:{self.arm_control_mode}  {stiff_txt}  {hand_txt}"
            + ("  [SEQUENCE RUNNING]" if self.sequence_running else "")
        )

    def _publish_hand_targets_once(self) -> None:
        self._init_hand_controllers()
        for hand in self._active_hands():
            controller = self.hand_controllers.get(hand)
            if controller is None:
                continue
            try:
                ok = controller.write_targets_once(
                    hand_grip_targets(hand, self.hand_grip_percent),
                    kp=self.hand_kp,
                    kd=self.hand_kd,
                    tau=self.hand_tau,
                    first_write_timeout_s=self.hand_write_timeout_s,
                )
                self.hand_info[hand] = (
                    f"{self.hand_grip_percent:.0f}%"
                    if ok else
                    "no cmd subscriber"
                )
            except Exception as exc:
                self.hand_info[hand] = f"write failed: {exc}"

    # ── Drawing ───────────────────────────────────────────────────────────────

    @staticmethod
    def _addnstr(win, y: int, x: int, text: str, n: int, attr: int = 0) -> None:
        try:
            win.addnstr(y, x, text, n, attr)
        except curses.error:
            pass

    @staticmethod
    def _addstr(win, y: int, x: int, text: str, attr: int = 0) -> None:
        try:
            win.addstr(y, x, text, attr)
        except curses.error:
            pass

    def _cp(self, pair: int) -> int:
        return curses.color_pair(pair) if curses.has_colors() else 0

    def draw(self, win, h: int, w: int) -> None:
        if h < 18 or w < 74:
            self._addstr(win, 0, 0, f"Terminal too small ({w}×{h}). Need ≥74×18.")
            return
        try:
            self._draw_all(win, h, w)
        except curses.error:
            pass

    def _draw_all(self, win, h: int, w: int) -> None:  # noqa: C901
        row = 0

        # ── Title ─────────────────────────────────────────────────────────
        title = "6D EE IK Pose Control  v3"
        conn_attr = self._cp(C_GREEN if self.seeded else C_RED) | curses.A_BOLD
        armed_attr = self._cp(C_GREEN if self.armed else C_RED) | curses.A_BOLD
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        self._addstr(win, row, max(0, (w - len(title)) // 2), title,
                     self._cp(C_CYAN) | curses.A_BOLD)
        self._addstr(win, row, w - 22, f"[{'CONNECTED' if self.seeded else 'WAITING'}]", conn_attr)
        self._addstr(win, row, w - 12, f"[{'ARMED' if self.armed else 'RELEASED'}]", armed_attr)
        row += 1

        # ── Parameter bar ─────────────────────────────────────────────────
        waist_lbl = "ON" if self.waist_enabled else "OFF"
        stiff_lbl = "ON" if self.orient_stiff else "OFF"
        hand_lbl = (
            "OFF"
            if self.hand_control_mode == "off"
            else f"{self.hand_control_mode.upper()} {self.hand_grip_percent:.0f}%"
        )
        arm_txt = (f"  Arm:[{self.arm_control_mode.upper()}](m)  "
                   f"Waist:[{waist_lbl}](w)  "
                   f"OrStiff:[{stiff_lbl}](f)  "
                   f"Hand:[{hand_lbl}](H)")
        param_txt = (f"ramp {self.max_speed:.3f} r/s (s)  "
                     f"max_dq {self.max_dq:.3f} (d/[/])")
        self._addnstr(win, row, 0, arm_txt, w // 2)
        self._addnstr(win, row, w - len(param_txt) - 2, param_txt, w, self._cp(C_YELLOW))
        row += 1

        # ── Waist kp bar ──────────────────────────────────────────────────
        waist_kp_txt = (f"  Waist roll/pitch kp:{self.waist_pr_kp:.0f} (W)  "
                        f"yaw kp:{self.waist_y_kp:.0f}  kd:{self.waist_kd:.1f}")
        self._addnstr(win, row, 0, waist_kp_txt, w, self._cp(C_YELLOW))
        row += 1

        # ── Divider ───────────────────────────────────────────────────────
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        row += 1

        # ── DOF table ─────────────────────────────────────────────────────
        disp = self._display_arm()
        T_cur = self._fk_live(disp) if self.seeded else np.eye(4)
        T_tgt = self.target_T[disp]
        cur_rpy = _rpy_from_R(T_cur[:3, :3])
        tgt_rpy = _rpy_from_R(T_tgt[:3, :3])

        dof_focus = self.focus == FOCUS_DOF
        hdr = (f" {'>' if dof_focus else ' '} {'DOF':<9}{'Live FK':<19}{'Target':<19}"
               f"{'Step':<15}  ({disp} arm)  [Tab]")
        self._addnstr(
            win,
            row,
            0,
            hdr,
            w,
            (self._cp(C_FOCUS) | curses.A_BOLD) if dof_focus else curses.A_BOLD,
        )
        row += 1

        for i in range(N_DOFS):
            sel = (i == self.dof_idx)
            mark = "▶" if sel else " "
            step = self._ee_step(i)
            unit = DOF_UNITS[i]
            if i < 3:
                cur_v = float(T_cur[i, 3])
                tgt_v = float(T_tgt[i, 3])
            else:
                cur_v = cur_rpy[i - 3]
                tgt_v = tgt_rpy[i - 3]
            # When orient_stiff is OFF and this is a rotation DOF, mark it as free
            free_mark = "" if (i >= 3 or self.orient_stiff) else " [free]"
            line = (f"{mark} {DOF_NAMES[i]:<9}"
                    f"{cur_v:+.4f} {unit:<5}"
                    f"   {tgt_v:+.4f} {unit:<5}"
                    f"   {step:.4f} {unit}{free_mark}")
            attr = (self._cp(C_SEL) | curses.A_BOLD) if sel else 0
            self._addnstr(win, row, 0, line, w, attr)
            row += 1

        # ── Divider ───────────────────────────────────────────────────────
        self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
        row += 1

        # ── IK status ─────────────────────────────────────────────────────
        for arm in ("left", "right"):
            if row >= h - 6:
                break
            if self.arm_control_mode not in ("both", arm):
                continue
            info = self.ik_info[arm]
            ok = info.get("success")
            if ok is None:
                txt = f"  IK {arm:<5}: pending"
                attr = 0
            elif ok:
                txt = (f"  IK {arm:<5}: OK  "
                       f"pos={info['error_pos_m']:.4f}m  "
                       f"rot={info['error_rot_rad']:.4f}rad  "
                       f"{info['iterations']}it")
                if info.get("mode") == "pos_shoulder_elbow":
                    txt += "  shoulder+elbow"
                elif info.get("mode") == "pos_axis_clamped":
                    txt += f"  axis-clamped ({info.get('axis_error_m', 0.0):.4f}m)"
                attr = self._cp(C_GREEN)
            else:
                txt = (f"  IK {arm:<5}: FAIL — target rolled back  "
                       f"(pos={info['error_pos_m']:.4f}m  "
                       f"rot={info['error_rot_rad']:.4f}rad)")
                attr = self._cp(C_RED)
            self._addnstr(win, row, 0, txt, w, attr)
            row += 1

        # ── Divider ───────────────────────────────────────────────────────
        if row < h - 6:
            self._addstr(win, row, 0, "─" * w, self._cp(C_CYAN))
            row += 1

        # ── Hand status ──────────────────────────────────────────────────
        if row < h - 6 and self.hand_control_mode != "off":
            hand_status = "  Dex3: " + "  ".join(
                f"{hand}={self.hand_info.get(hand, 'off')}"
                for hand in ("left", "right")
                if hand in self._active_hands()
            )
            self._addnstr(win, row, 0, hand_status, w, self._cp(C_YELLOW))
            row += 1

        # ── Joint readout ─────────────────────────────────────────────────
        if row < h - 6:
            self._addnstr(win, row, 0,
                          "  Joint targets (rad):                    "
                          "  Live feedback (rad):",
                          w, curses.A_BOLD)
            row += 1

        for arm, joints in ARM_JOINTS.items():
            if row >= h - 6:
                break
            if self.arm_control_mode not in ("both", arm):
                continue
            prefix = f"  {arm.upper():<5}: "
            lbl = "  ".join(f"{n:<5}" for n in JOINT_LABELS)
            tgt_v = "  ".join(f"{self.desired_targets[j]:+.3f}" for j in joints)
            liv_v = "  ".join(
                f"{self.latest_positions.get(j, self.current_targets[j]):+.3f}"
                for j in joints
            )
            self._addnstr(win, row, 0, prefix + lbl, w, self._cp(C_CYAN))
            row += 1
            if row < h - 6:
                half = w // 2
                self._addnstr(win, row, 0, prefix + tgt_v, half)
                self._addnstr(win, row, half, prefix + liv_v, w - half, self._cp(C_YELLOW))
                row += 1

        if row < h - 8:
            self._draw_pose_sequence_panels(win, row, h - 6, w)

        # ── Footer ────────────────────────────────────────────────────────
        self._addstr(win, h - 5, 0, "─" * w, self._cp(C_CYAN))
        hn1 = "  Tab: focus   ↑/↓ j/k: select   ← →/- +: adjust   p: save   l/Enter: load   a: add seq   R/S: run/stop"
        hn2 = "  H: hand   { }: grip   g: grip/gap   y: sync   w/W: waist   r/e: release/reengage   s/d: speed/max_dq/delete   q: quit"
        self._addnstr(win, h - 4, 0, hn1, w, self._cp(C_YELLOW))
        self._addnstr(win, h - 3, 0, hn2, w, self._cp(C_YELLOW))
        self._addstr(win, h - 2, 0, "─" * w, self._cp(C_CYAN))
        st_attr = self._cp(C_GREEN if self.armed and self.seeded else C_RED)
        self._addnstr(win, h - 1, 0, f"  {self.status}", w, st_attr)

    def _draw_pose_sequence_panels(self, win, top: int, bottom: int, w: int) -> None:
        rows = max(0, bottom - top - 1)
        if rows <= 0:
            return
        mid = max(36, w // 2)
        seq_x = min(w - 1, mid + 1)
        poses_focus = self.focus == FOCUS_POSES
        seq_focus = self.focus == FOCUS_SEQUENCE
        pose_attr = (self._cp(C_FOCUS) | curses.A_BOLD) if poses_focus else curses.A_BOLD
        seq_attr = (self._cp(C_FOCUS) | curses.A_BOLD) if seq_focus else curses.A_BOLD
        self._addstr(win, top, 0, "─" * w, self._cp(C_CYAN))
        self._addnstr(
            win,
            top,
            0,
            f" {'>' if poses_focus else ' '} Poses ({len(self.saved_poses)}) [p]save [l]load [d]del [a]->seq",
            mid,
            pose_attr,
        )
        self._addstr(win, top, mid, "│", self._cp(C_CYAN))
        waist_ind = "W" if self.include_waist_new else "w"
        self._addnstr(
            win,
            top,
            seq_x,
            f" {'>' if seq_focus else ' '} Seq ({len(self.sequence_steps)}) gap:{self.sequence_gap_s:.1f}s [{waist_ind}]waist [R]run",
            w - seq_x,
            seq_attr,
        )
        for i in range(rows):
            y = top + 1 + i
            self._addstr(win, y, mid, "│", self._cp(C_CYAN))
            if i < len(self.saved_poses):
                pose = self.saved_poses[i]
                name = str(pose.get("name", f"pose_{i}"))
                saved = str(pose.get("saved_at", ""))[:19]
                mark = ">" if i == self.pose_cursor else " "
                attr = (self._cp(C_SEL) | curses.A_BOLD) if poses_focus and i == self.pose_cursor else 0
                self._addnstr(win, y, 0, f"{mark} {i}: {name} {saved}", mid, attr)
            if i < len(self.sequence_steps):
                step = self.sequence_steps[i]
                pose_index = int(step.get("pose_index", -1))
                if 0 <= pose_index < len(self.saved_poses):
                    name = str(self.saved_poses[pose_index].get("name", f"pose_{pose_index}"))
                else:
                    name = "<missing>"
                waist = "waist" if step.get("include_waist", True) else "arms"
                mark = ">" if i == self.seq_cursor else " "
                active = self.sequence_running and i == self.sequence_step_index - 1
                if seq_focus and i == self.seq_cursor:
                    attr = self._cp(C_SEL) | curses.A_BOLD
                elif active:
                    attr = self._cp(C_RUNNING) | curses.A_BOLD
                else:
                    attr = 0
                self._addnstr(win, y, seq_x, f"{mark} {i + 1}: {name} [{waist}]", w - seq_x, attr)

    # ── Inline prompt ─────────────────────────────────────────────────────────

    def _prompt(self, win, h: int, w: int, label: str) -> str:
        try:
            curses.curs_set(1)
        except curses.error:
            pass
        win.timeout(20)
        buf: List[str] = []
        try:
            while True:
                try:
                    self.tick()
                except Exception as exc:
                    self.status = f"Tick error: {exc}"
                win.move(h - 1, 0)
                win.clrtoeol()
                self._addnstr(win, h - 1, 0, f"{label}: {''.join(buf)}▌"[:w], w, curses.A_BOLD)
                win.refresh()
                key = win.getch()
                if key == -1:
                    continue
                if key in (curses.KEY_ENTER, 10, 13):
                    break
                if key in (curses.KEY_BACKSPACE, 127, 8):
                    if buf:
                        buf.pop()
                elif key == 27:
                    buf = []
                    break
                elif 32 <= key <= 126:
                    buf.append(chr(key))
        finally:
            try:
                curses.curs_set(0)
            except curses.error:
                pass
            win.timeout(20)
        return "".join(buf).strip()

    # ── Key handler ───────────────────────────────────────────────────────────

    def handle_key(self, key: int, win, h: int, w: int) -> None:  # noqa: C901
        # ── Quit ──────────────────────────────────────────────────────────
        if key in (ord("q"), 27):
            self.request_shutdown()
            return

        if key == 9:
            self.focus = (self.focus + 1) % 3
            return

        # ── DOF navigation ────────────────────────────────────────────────
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_DOF:
            self.dof_idx = max(0, self.dof_idx - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_DOF:
            self.dof_idx = min(N_DOFS - 1, self.dof_idx + 1)
            return
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_POSES:
            self.pose_cursor = max(0, self.pose_cursor - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_POSES:
            self.pose_cursor = min(max(0, len(self.saved_poses) - 1), self.pose_cursor + 1)
            return
        if key in (curses.KEY_UP, ord("k")) and self.focus == FOCUS_SEQUENCE:
            self.seq_cursor = max(0, self.seq_cursor - 1)
            return
        if key in (curses.KEY_DOWN, ord("j")) and self.focus == FOCUS_SEQUENCE:
            self.seq_cursor = min(max(0, len(self.sequence_steps) - 1), self.seq_cursor + 1)
            return

        # ── EE increment / decrement ──────────────────────────────────────
        if key in (curses.KEY_LEFT, ord("-")):
            if self.seeded and self.armed:
                self._adjust_dof(-self._ee_step())
            return
        if key in (curses.KEY_RIGHT, ord("+")):
            if self.seeded and self.armed:
                self._adjust_dof(+self._ee_step())
            return

        # ── EE step size: < > ─────────────────────────────────────────────
        if key == ord("<"):
            self._set_ee_step(max(0.0001, self._ee_step() / 2.0))
            return
        if key == ord(">"):
            hi = 1.0 if self.dof_idx < 3 else math.pi
            self._set_ee_step(min(hi, self._ee_step() * 2.0))
            return

        # ── Max joint delta: [ ] ──────────────────────────────────────────
        if key == ord("["):
            self.max_dq = max(0.005, self.max_dq / 2.0)
            self.status = f"max_dq → {self.max_dq:.4f} rad"
            return
        if key == ord("]"):
            self.max_dq = min(math.pi, self.max_dq * 2.0)
            self.status = f"max_dq → {self.max_dq:.4f} rad"
            return

        # ── Arm mode ──────────────────────────────────────────────────────
        if key == ord("m"):
            idx = ARM_CONTROL_MODES.index(self.arm_control_mode)
            self.arm_control_mode = ARM_CONTROL_MODES[(idx + 1) % len(ARM_CONTROL_MODES)]
            self.status = f"Arm mode → {self.arm_control_mode}"
            return

        # ── Dex3 hand mode / grip ────────────────────────────────────────
        if key == ord("H"):
            idx = HAND_CONTROL_MODES.index(self.hand_control_mode)
            self.hand_control_mode = HAND_CONTROL_MODES[(idx + 1) % len(HAND_CONTROL_MODES)]
            self._init_hand_controllers()
            self.status = f"Dex3 hand mode → {self.hand_control_mode}"
            return
        if key == ord("{"):
            self._ensure_hand_control_for_grip()
            self.hand_grip_percent = max(0.0, self.hand_grip_percent - 5.0)
            self._publish_hand_targets_once()
            self._last_hand_publish_s = time.monotonic()
            self.status = f"Dex3 grip → {self.hand_grip_percent:.0f}%"
            return
        if key == ord("}"):
            self._ensure_hand_control_for_grip()
            self.hand_grip_percent = min(100.0, self.hand_grip_percent + 5.0)
            self._publish_hand_targets_once()
            self._last_hand_publish_s = time.monotonic()
            self.status = f"Dex3 grip → {self.hand_grip_percent:.0f}%"
            return
        if key == ord("g"):
            if self.focus == FOCUS_SEQUENCE:
                val = self._prompt(win, h, w, f"Sequence gap s [{self.sequence_gap_s:.1f}]")
                try:
                    self.sequence_gap_s = max(0.0, float(val))
                    self._write_pose_file()
                    self.status = f"Sequence gap -> {self.sequence_gap_s:.1f} s"
                except (ValueError, TypeError):
                    if val:
                        self.status = f"Invalid value: {val!r}"
                return
            self._ensure_hand_control_for_grip()
            val = self._prompt(win, h, w, f"Dex3 grip percent [{self.hand_grip_percent:.0f}]")
            try:
                self.hand_grip_percent = max(0.0, min(100.0, float(val)))
                self._publish_hand_targets_once()
                self._last_hand_publish_s = time.monotonic()
                self.status = f"Dex3 grip → {self.hand_grip_percent:.0f}%"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

        # ── Orientation stiffness toggle ──────────────────────────────────
        if key == ord("f"):
            self.orient_stiff = not self.orient_stiff
            self.status = (
                "Orient stiff ON — rotation locked during x/y/z moves"
                if self.orient_stiff else
                "Orient stiff OFF — x/y/z IK uses shoulder+elbow only"
            )
            return

        # ── Sync EE targets to current FK pose ────────────────────────────
        if key == ord("y"):
            self._sync_targets_to_live()
            self.status = "EE targets resynced to current hand pose"
            return

        # ── Waist toggle ──────────────────────────────────────────────────
        if key == ord("w"):
            if self.focus == FOCUS_SEQUENCE:
                self.include_waist_new = not self.include_waist_new
                self.status = (
                    "New sequence steps include waist"
                    if self.include_waist_new else
                    "New sequence steps are arms-only"
                )
                return
            self.waist_enabled = not self.waist_enabled
            if self.waist_enabled:
                # Hold waist exactly where it physically is right now so it
                # doesn't lurch to a stale desired position and disturb IK.
                for j in WAIST_JOINTS:
                    self.desired_targets[j] = self.latest_positions.get(
                        j, self.desired_targets[j]
                    )
                self.current_targets.update(
                    {j: self.desired_targets[j] for j in WAIST_JOINTS}
                )
            # Resync EE targets from current arm joints so any physical shift
            # caused by the waist change doesn't leave stale unreachable targets.
            self._sync_ee_from_joints()
            self.status = f"Waist {'ENABLED (held)' if self.waist_enabled else 'DISABLED (free)'} — EE resynced"
            return

        # ── Waist pitch/roll kp prompt ────────────────────────────────────
        if key == ord("W"):
            val = self._prompt(win, h, w, f"Waist pitch/roll kp [{self.waist_pr_kp:.1f}]")
            try:
                self.waist_pr_kp = max(0.0, float(val))
                self.status = f"Waist pitch/roll kp → {self.waist_pr_kp:.1f}"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

        # ── Release arms ──────────────────────────────────────────────────
        if key == ord("r"):
            try:
                self._release_arms()
                self.armed = False
                self.status = "Arms released — move freely, press e to reengage"
            except Exception as exc:
                self.status = f"Release failed: {exc}"
            return

        # ── Reengage arms ─────────────────────────────────────────────────
        if key == ord("e"):
            try:
                self._unrelease_arms()
                self.armed = True
                self.status = "Reengaged — holding current arm pose"
            except Exception as exc:
                self.status = f"Reengage failed: {exc}"
            return

        # ── Zero-gain hold ────────────────────────────────────────────────
        if key == ord("z"):
            self.pub.publish_zero_gains(self.current_targets)
            self.status = "Zero-gain hold sent on rt/arm_sdk"
            return

        # ── Ramp speed prompt ─────────────────────────────────────────────
        if key == ord("s"):
            val = self._prompt(win, h, w, f"Ramp speed r/s [{self.max_speed:.4f}]")
            try:
                self.max_speed = max(0.01, float(val))
                self.status = f"Ramp speed → {self.max_speed:.4f} r/s"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

        # ── Max joint delta prompt ────────────────────────────────────────
        if key == ord("d"):
            if self.focus == FOCUS_POSES:
                row = self.pose_cursor
                if 0 <= row < len(self.saved_poses):
                    name = str(self.saved_poses[row].get("name", f"pose_{row}"))
                    new_steps = []
                    for step in self.sequence_steps:
                        pose_index = int(step.get("pose_index", -1))
                        if pose_index == row:
                            continue
                        new_steps.append({
                            "pose_index": pose_index - (1 if pose_index > row else 0),
                            "include_waist": bool(step.get("include_waist", True)),
                        })
                    self.sequence_steps = new_steps
                    del self.saved_poses[row]
                    self.pose_cursor = min(self.pose_cursor, max(0, len(self.saved_poses) - 1))
                    self.seq_cursor = min(self.seq_cursor, max(0, len(self.sequence_steps) - 1))
                    self.sequence_running = False
                    self._write_pose_file()
                    self.status = f"Deleted pose '{name}'"
                else:
                    self.status = "No pose selected"
                return
            val = self._prompt(win, h, w, f"Max joint delta rad [{self.max_dq:.4f}]")
            try:
                self.max_dq = max(0.005, min(math.pi, float(val)))
                self.status = f"max_dq → {self.max_dq:.4f} rad"
            except (ValueError, TypeError):
                if val:
                    self.status = f"Invalid value: {val!r}"
            return

        # ── Save / load / sequence replay ─────────────────────────────────
        if key == ord("p"):
            name = self._prompt(win, h, w, "Pose name")
            if not name:
                self.status = "Save cancelled (empty name)"
                return
            self.saved_poses.append(self._pose_payload(name))
            self.pose_cursor = len(self.saved_poses) - 1
            self._write_pose_file()
            self.status = f"Saved pose '{name}'"
            return

        if key in (ord("l"), curses.KEY_ENTER, 10) and self.focus == FOCUS_POSES:
            if 0 <= self.pose_cursor < len(self.saved_poses):
                pose = self.saved_poses[self.pose_cursor]
                try:
                    self._apply_joint_pose(pose, include_waist=True)
                    self.status = f"Loaded pose '{pose.get('name', '<unnamed>')}'"
                except Exception as exc:
                    self.status = f"Load failed: {exc}"
            else:
                self.status = "No pose selected"
            return

        if key == ord("a") and self.focus == FOCUS_POSES:
            if 0 <= self.pose_cursor < len(self.saved_poses):
                self.sequence_steps.append({
                    "pose_index": self.pose_cursor,
                    "include_waist": self.include_waist_new,
                })
                self.seq_cursor = len(self.sequence_steps) - 1
                self._write_pose_file()
                name = self.saved_poses[self.pose_cursor].get("name", f"pose_{self.pose_cursor}")
                waist = "with waist" if self.include_waist_new else "arms only"
                self.status = f"Added '{name}' to sequence ({waist})"
            else:
                self.status = "No pose selected"
            return

        if key in (ord("x"), curses.KEY_DC) and self.focus == FOCUS_SEQUENCE:
            if 0 <= self.seq_cursor < len(self.sequence_steps):
                del self.sequence_steps[self.seq_cursor]
                self.seq_cursor = min(self.seq_cursor, max(0, len(self.sequence_steps) - 1))
                self.sequence_running = False
                self._write_pose_file()
                self.status = "Removed sequence step"
            return

        if key == ord("u") and self.focus == FOCUS_SEQUENCE:
            row = self.seq_cursor
            if 1 <= row < len(self.sequence_steps):
                self.sequence_steps[row - 1], self.sequence_steps[row] = (
                    self.sequence_steps[row], self.sequence_steps[row - 1]
                )
                self.seq_cursor -= 1
                self._write_pose_file()
            return

        if key == ord("n") and self.focus == FOCUS_SEQUENCE:
            row = self.seq_cursor
            if 0 <= row < len(self.sequence_steps) - 1:
                self.sequence_steps[row + 1], self.sequence_steps[row] = (
                    self.sequence_steps[row], self.sequence_steps[row + 1]
                )
                self.seq_cursor += 1
                self._write_pose_file()
            return

        if key == ord("R"):
            if not self.armed:
                self.status = "Reengage arms before running a sequence"
            elif not self.sequence_steps:
                self.status = "Add poses to the sequence first"
            else:
                self.sequence_running = True
                self.sequence_step_index = 0
                self.sequence_next_time_s = 0.0
                self.status = "Sequence started"
            return

        if key == ord("S"):
            self.sequence_running = False
            self.sequence_step_index = 0
            self.sequence_next_time_s = 0.0
            self.status = "Sequence stopped"
            return

    # ── Main curses loop ──────────────────────────────────────────────────────

    def run(self) -> None:
        try:
            curses.wrapper(self._curses_main)
        finally:
            self.close()

    def request_shutdown(self) -> None:
        self._running = False
        self.sequence_running = False

    def close(self) -> None:
        if self._closed or self._closing:
            return
        self._closing = True
        self.request_shutdown()
        try:
            if getattr(self, "armed", False):
                self._release_arms()
                self.armed = False
        except Exception:
            pass
        finally:
            self._closed = True
            self._closing = False
            self._controller_lock.release()

    def _curses_main(self, stdscr) -> None:
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(C_GREEN, curses.COLOR_GREEN, -1)
            curses.init_pair(C_YELLOW, curses.COLOR_YELLOW, -1)
            curses.init_pair(C_RED, curses.COLOR_RED, -1)
            curses.init_pair(C_CYAN, curses.COLOR_CYAN, -1)
            curses.init_pair(C_SEL, curses.COLOR_BLACK, curses.COLOR_WHITE)
            curses.init_pair(C_BOLD, curses.COLOR_BLACK, curses.COLOR_CYAN)
            curses.init_pair(C_FOCUS, curses.COLOR_BLACK, curses.COLOR_CYAN)
            curses.init_pair(C_RUNNING, curses.COLOR_WHITE, curses.COLOR_BLUE)

        curses.curs_set(0)
        stdscr.timeout(20)

        self._last_tick = time.monotonic()
        dt_target = 1.0 / self.rate_hz

        while self._running:
            h, w = stdscr.getmaxyx()
            try:
                stdscr.erase()
                self.draw(stdscr, h, w)
                stdscr.refresh()
            except curses.error:
                pass

            key = stdscr.getch()
            if key != -1:
                self.handle_key(key, stdscr, h, w)
                if not self._running:
                    break

            now = time.monotonic()
            if now - self._last_tick >= dt_target:
                try:
                    self.tick()
                except Exception as exc:
                    self.status = f"Tick error: {exc}"

        self.close()


# ── CLI entry point ───────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="6D end-effector IK pose control TUI for the G1 arms — v3"
    )
    p.add_argument(
        "--iface",
        default=default_dds_iface(),
        help="DDS network interface (default: auto-detected)",
    )
    p.add_argument("--domain-id", type=int, default=0)
    p.add_argument("--file", default=DEFAULT_POSE_FILE,
                   help="JSON file for saved joint poses and replay sequence")
    p.add_argument("--rate-hz", type=float, default=25.0, help="Publish rate Hz")
    p.add_argument("--speed-rad-s", type=float, default=0.2,
                   help="Joint ramp speed rad/s")
    p.add_argument("--max-dq", type=float, default=0.2,
                   help="Max joint change applied per IK key-press (rad)")
    p.add_argument("--kp", type=float, default=DEFAULT_ARM_KP)
    p.add_argument("--kd", type=float, default=DEFAULT_ARM_KD)
    p.add_argument("--waist-pr-kp", type=float, default=DEFAULT_WAIST_PR_KP,
                   help="Waist pitch+roll hold kp (default 200)")
    p.add_argument(
        "--arm-control",
        choices=ARM_CONTROL_MODES,
        default="right",
        help="Which arm(s) to control (default: right)",
    )
    p.add_argument(
        "--hand-control",
        choices=HAND_CONTROL_MODES,
        default="off",
        help="Dex3 hand(s) to control: off, left, right, both, or follow-arm (default: off)",
    )
    p.add_argument("--hand-grip", type=float, default=0.0,
                   help="Initial Dex3 grip percent, 0=open and 100=closed")
    p.add_argument("--hand-kp", type=float, default=1.2,
                   help="Dex3 hand position kp")
    p.add_argument("--hand-kd", type=float, default=0.05,
                   help="Dex3 hand position kd")
    p.add_argument("--hand-tau", type=float, default=0.05,
                   help="Dex3 hand feed-forward tau")
    p.add_argument("--hand-rate-hz", type=float, default=10.0,
                   help="Dex3 hand command refresh rate while hand control is active")
    p.add_argument("--hand-write-timeout-s", type=float, default=0.002,
                   help="Per-write timeout for Dex3 hand DDS writes")
    return p.parse_args()


def main() -> None:
    try:
        app = IKPoseCLI(_parse_args())
    except ControllerLockError as exc:
        print(f"ik_pose_cli_v3: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    atexit.register(app.close)

    def _stop(signum, _frame) -> None:
        app.request_shutdown()
        if app._closing:
            return
        app.close()
        raise SystemExit(128 + int(signum))

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, _stop)
        except (AttributeError, ValueError):
            pass

    app.run()


if __name__ == "__main__":
    main()
