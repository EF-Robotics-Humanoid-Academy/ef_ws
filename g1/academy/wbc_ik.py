"""6D end-effector IK pose control with streamed gamepad input (dashboard /wbc).

Port of WBC/ik_pose_cli_v4.py onto sdk_wrapper.G1 -- no modules/ code.  The
control behaviour is unchanged:

  - Each increment moves the chosen hand's (left / right / both) EE target,
    runs warm-started DLS IK over the 7 arm joints, clamps each joint delta to
    +-max_dq, and the control loop ramps the commanded joints toward that
    solution at max_speed rad/s, publishing rt/arm_sdk at rate_hz.
  - If IK fails the EE target is rolled back and the joints do not move.
  - Orientation stiffness ON locks roll/pitch/yaw during x/y/z moves (full 6D
    IK); OFF frees it and moves shoulder + elbow only (position IK).
  - Controller input (same mapping as v4): left stick vx / yaw (radial
    deadzone, leased Move refreshed every 0.2 s, StopMove repeated for 0.6 s
    on release), right stick EE x / y, D-pad up/down EE z, left/right EE roll,
    A grip open/close, B release / re-engage arms, X arm right->left->both,
    Y orientation lock.  Stale input (>0.5 s) stops locomotion once.

Input comes from the remote controller SSE service's /snapshot endpoint
(modules/scripts/remote_controller_stream) or from a gamepad attached to the
browser running the dashboard (Gamepad API, posted to the server).  The
legacy v4 keyboard functions (DOF jog, steps, poses, sequences, ...) are
exposed as actions for the dashboard's buttons and key bindings.

Only one process may publish rt/arm_sdk targets: the same lock file as the
CLI is used, so the dashboard and ik_pose_cli_v4.py exclude each other.
"""
from __future__ import annotations

import fcntl
import json
import math
import os
import sys
import threading
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

G1_ROOT = Path(__file__).resolve().parent.parent
if str(G1_ROOT) not in sys.path:
    sys.path.insert(0, str(G1_ROOT))

from hand_pose_navigation.arm_fk import (  # noqa: E402
    JOINT_LIMITS, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, ArmFK,
    _LEFT_SHOULDER_IN_BASE, _RIGHT_SHOULDER_IN_BASE,
)
from hand_pose_navigation.arm_ik import ArmIK  # noqa: E402

DEFAULT_POSE_FILE = G1_ROOT / "WBC" / "saved_ik_pose_cli_v4_poses.json"
DEFAULT_REMOTE_URL = "http://192.168.2.79:8765/events"
# Shared with WBC/ik_pose_cli_v3.py / v4.py so only one publisher owns rt/arm_sdk.
DEFAULT_LOCK_FILE = "/tmp/ik_pose_cli_v3_rt_arm_sdk.lock"

WAIST_JOINTS = [12, 13, 14]   # roll, pitch, yaw
UPPER_BODY_JOINTS = WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS
WAIST_HOLD_KP = 480.0   # yaw hold kp
WAIST_HOLD_KD = 12.0
DEFAULT_ARM_KP = 30.0
DEFAULT_ARM_KD = 1.5
DEFAULT_WAIST_PR_KP = 200.0   # pitch + roll hold kp

ARM_CONTROL_MODES = ("both", "left", "right")
HAND_CONTROL_MODES = ("off", "right", "left", "both", "follow-arm")
ARM_JOINTS: Dict[str, List[int]] = {"left": LEFT_ARM_JOINTS, "right": RIGHT_ARM_JOINTS}
JOINT_LABELS = ("sh_p", "sh_r", "sh_y", "elbow", "wr_r", "wr_p", "wr_y")
SHOULDER_ELBOW_IDXS = (0, 1, 2, 3)
POSITION_IK_TOL_M = 0.005
POSITION_IK_AXIS_TOL_M = 0.006
POSITION_IK_SOFT_LIMIT_M = 0.040
INPUT_STALE_S = 0.5

_SHOULDER_ORIGIN: Dict[str, np.ndarray] = {"left": _LEFT_SHOULDER_IN_BASE, "right": _RIGHT_SHOULDER_IN_BASE}

DOF_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
DOF_UNITS = ("m", "m", "m", "rad", "rad", "rad")
N_DOFS = 6

# Session options: (default, low, high).  Mirrors ik_pose_cli_v4's arguments.
PARAMS: Dict[str, Tuple[float, float, float]] = {
    "rate_hz": (25.0, 1.0, 100.0),
    "speed_rad_s": (0.2, 0.01, 2.0),
    "max_dq": (0.2, 0.005, math.pi),
    "kp": (DEFAULT_ARM_KP, 0.0, 100.0),
    "kd": (DEFAULT_ARM_KD, 0.0, 10.0),
    "waist_pr_kp": (DEFAULT_WAIST_PR_KP, 0.0, 600.0),
    "ee_speed_m_s": (0.12, 0.001, 0.5),
    "ee_y_speed_m_s": (0.12, 0.001, 0.5),
    "max_vx": (0.35, 0.0, 0.8),
    "max_vyaw": (0.7, 0.0, 1.5),
    "loco_deadzone": (0.25, 0.0, 0.9),
    "pos_step": (0.01, 0.0001, 1.0),
    "rot_step": (0.05, 0.0001, math.pi),
    "hand_kp": (1.2, 0.0, 5.0),
    "hand_kd": (0.05, 0.0, 1.0),
    "hand_tau": (0.05, 0.0, 1.0),
    "hand_rate_hz": (10.0, 0.1, 50.0),
    "sequence_gap_s": (2.0, 0.0, 60.0),
}


class ControllerLockError(RuntimeError):
    pass


class SingleControllerLock:
    """Process lock so only one controller publishes rt/arm_sdk targets."""

    def __init__(self, path: str = DEFAULT_LOCK_FILE) -> None:
        self.path = str(path)
        self._fd: Optional[int] = None

    def acquire(self) -> None:
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            # Shared by all workshop accounts: the next user must be able to
            # rewrite the owner record, which the umask would otherwise block.
            os.fchmod(fd, 0o666)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            owner = self._read_owner(fd)
            os.close(fd)
            raise ControllerLockError(
                "Another IK pose session already owns rt/arm_sdk" + (f" ({owner})" if owner else "")
                + f". Stop that session first. Lock: {self.path}"
            ) from exc
        except OSError:
            os.close(fd)
            raise
        os.ftruncate(fd, 0)
        owner = f"pid={os.getpid()} g1_dashboard /wbc started={datetime.now(timezone.utc).isoformat()}\n"
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
        fd, self._fd = self._fd, None
        if fd is None:
            return
        try:
            os.ftruncate(fd, 0)
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


class RemoteStateStream:
    """Poll the remote controller's current JSON snapshot."""

    kind = "remote"

    def __init__(self, url: str) -> None:
        # The SSE endpoint emits only on input changes, while the control loop
        # needs a fresh timestamp for held inputs. The companion snapshot
        # endpoint returns the current state on every request.
        self.url = url.rsplit("/", 1)[0] + "/snapshot"
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {}
        self.last_received_s = 0.0
        self._running = True
        self.error = "connecting"
        self._thread = threading.Thread(target=self._run, name="wbc-remote", daemon=True)
        self._thread.start()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def close(self) -> None:
        self._running = False

    def _run(self) -> None:
        while self._running:
            try:
                req = urllib.request.Request(self.url, headers={"Accept": "application/json"})
                with urllib.request.urlopen(req, timeout=1.0) as response:
                    payload = json.loads(response.read().decode("utf-8", "replace"))
                data = payload.get("data", payload)
                controllers = data.get("controllers", [])
                state = controllers[0] if controllers else {}
                if not isinstance(state, dict):
                    state = {}
                with self._lock:
                    self._state = state
                    self.last_received_s = time.monotonic()
                self.error = "connected"
                time.sleep(0.05)
            except Exception as exc:
                self.error = str(exc)
                with self._lock:
                    self._state = {}
                    self.last_received_s = 0.0
                time.sleep(1.0)


class BrowserGamepadInput:
    """Gamepad state posted by the dashboard page (standard Gamepad mapping).

    Same layout as the remote stream: axes 0/1 left stick, 2/3 right stick;
    buttons 0 A, 1 B, 2 X, 3 Y; hats[0] = [right - left, up - down].
    """

    kind = "browser"

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {}
        self.last_received_s = 0.0
        self.error = "waiting for the browser gamepad"
        self.name = ""

    def update(self, state: Dict[str, Any]) -> None:
        axes = [max(-1.0, min(1.0, float(v))) for v in list(state.get("axes") or [])[:8]]
        buttons = [bool(v) for v in list(state.get("buttons") or [])[:20]]
        hats = state.get("hats") or [[0, 0]]
        hat = hats[0] if isinstance(hats, list) and hats and isinstance(hats[0], (list, tuple)) else [0, 0]
        with self._lock:
            self._state = {"axes": axes, "buttons": buttons, "hats": [[int(hat[0]), int(hat[1])]]}
            self.last_received_s = time.monotonic()
        self.name = str(state.get("name") or "")[:80]
        self.error = "connected"

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._state)

    def close(self) -> None:
        pass


# -- Rotation helpers --------------------------------------------------------

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
    """ZYX Euler angles (roll, pitch, yaw) from a 3x3 rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        return math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0


def _clamp_q(q: np.ndarray, arm: str) -> np.ndarray:
    limits = JOINT_LIMITS[arm]
    lo = np.array([lim[0] for lim in limits], dtype=np.float64)
    hi = np.array([lim[1] for lim in limits], dtype=np.float64)
    return np.clip(q, lo, hi)


class IKPoseController:
    """ik_pose_cli_v4.IKPoseCLI without curses: a background control loop
    plus thread-safe actions and a JSON snapshot for the dashboard."""

    def __init__(self, robot: Any, *, pose_path: Path = DEFAULT_POSE_FILE, remote_url: str = DEFAULT_REMOTE_URL,
                 source: str = "remote", arm_control: str = "right", hand_control: str = "off",
                 hand_grip: float = 0.0, params: Optional[Dict[str, float]] = None,
                 lock_path: str = DEFAULT_LOCK_FILE) -> None:
        self.robot = robot
        self._lock = threading.RLock()
        self._closed = False
        self._closing = False
        self._controller_lock = SingleControllerLock(lock_path)
        self._controller_lock.acquire()
        try:
            self._init(pose_path, remote_url, source, arm_control, hand_control, hand_grip, params or {})
        except BaseException:
            self._running = False
            if getattr(self, "remote", None) is not None:
                self.remote.close()
            self._controller_lock.release()
            raise

    def _init(self, pose_path: Path, remote_url: str, source: str, arm_control: str, hand_control: str,
              hand_grip: float, params: Dict[str, float]) -> None:
        self.pose_path = Path(os.path.abspath(os.path.expanduser(str(pose_path))))
        values = {key: default for key, (default, _lo, _hi) in PARAMS.items()}
        for key, value in params.items():
            if key in PARAMS:
                low, high = PARAMS[key][1:]
                values[key] = max(low, min(high, float(value)))
        self.rate_hz = values["rate_hz"]
        self.max_speed = values["speed_rad_s"]
        self.max_dq = values["max_dq"]
        self.arm_kp = values["kp"]
        self.arm_kd = values["kd"]
        self.waist_pr_kp = values["waist_pr_kp"]   # pitch + roll
        self.waist_y_kp = float(WAIST_HOLD_KP)      # yaw (fixed)
        self.waist_kd = float(WAIST_HOLD_KD)
        self.waist_enabled = True
        self.arm_control_mode = arm_control if arm_control in ARM_CONTROL_MODES else "right"
        self.hand_control_mode = hand_control if hand_control in HAND_CONTROL_MODES else "off"
        self.hand_grip_percent = max(0.0, min(100.0, float(hand_grip)))
        self.hand_kp = values["hand_kp"]
        self.hand_kd = values["hand_kd"]
        self.hand_tau = values["hand_tau"]
        self.hand_rate_hz = values["hand_rate_hz"]
        self._last_hand_publish_s = 0.0
        self.remote_url = str(remote_url)
        self.remote: Any = BrowserGamepadInput() if source == "browser" else RemoteStateStream(self.remote_url)
        self._remote_prev_buttons: List[bool] = []
        self._remote_mode_arms = ("right", "left", "both")
        self._remote_last_loco_s = 0.0
        self._remote_loco_was_active = False
        self._remote_last_loco_cmd: Optional[Tuple[float, float]] = None
        self._remote_loco_stopped = False
        self._remote_stop_until_s = 0.0
        self._remote_dpad_last_s = 0.0
        self._remote_dpad_x_last_s = 0.0
        self._remote_ee_last_s = time.monotonic()
        self._remote_grip_closed = self.hand_grip_percent >= 50.0
        self.max_ee_speed = values["ee_speed_m_s"]
        self.max_ee_y_speed = values["ee_y_speed_m_s"]
        self.max_vx = values["max_vx"]
        self.max_vyaw = values["max_vyaw"]
        self.loco_deadzone = values["loco_deadzone"]
        self.pos_step = values["pos_step"]   # metres per increment
        self.rot_step = values["rot_step"]   # radians per increment
        # Orientation stiffness: when True, x/y/z moves keep rotation locked.
        # When False, target rotation follows current FK so IK is position-only.
        self.orient_stiff = True

        self.latest_positions: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.current_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.desired_targets: Dict[int, float] = {j: 0.0 for j in UPPER_BODY_JOINTS}
        self.seeded = False
        self.armed = False
        self._running = True
        self.status = "Waiting for rt/lowstate..."
        self.dof_idx = 0

        self.saved_poses: List[Dict[str, Any]] = []
        self.sequence_steps: List[Dict[str, Any]] = []
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.sequence_gap_s = values["sequence_gap_s"]
        self.include_waist_new = True

        self.target_T: Dict[str, np.ndarray] = {"left": np.eye(4), "right": np.eye(4)}
        self._fk: Dict[str, ArmFK] = {"left": ArmFK("left", "urdf"), "right": ArmFK("right", "urdf")}
        self._ik: Dict[str, ArmIK] = {
            arm: ArmIK(arm, "dls", max_iter=24, tol_pos_m=0.005, tol_rot_rad=0.02) for arm in ("left", "right")
        }
        self.ik_info: Dict[str, Dict] = {arm: self._empty_ik_info() for arm in ("left", "right")}
        self.hand_info: Dict[str, str] = {"left": "off", "right": "off"}
        self._load_pose_file()
        self._seed_from_state()
        # v4 starts armed and publishes at full arm_sdk weight from the first
        # lowstate sample; ramp the authority in instead so taking over from
        # the default arm controller is smooth (now, or once lowstate arrives).
        self._engage_on_seed = not self.seeded
        if self.seeded:
            self._unrelease_arms()
            self.armed = True
            self.status = "Connected · arms engaged"
        self._last_tick = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="wbc-ik", daemon=True)
        self._thread.start()

    @staticmethod
    def _empty_ik_info() -> Dict[str, Any]:
        return {"success": None, "error_pos_m": 0.0, "error_rot_rad": 0.0, "iterations": 0}

    # -- Robot I/O -------------------------------------------------------------

    def _state_snapshot(self) -> Optional[Tuple[Dict[int, float], float]]:
        return self.robot.get_joint_positions(UPPER_BODY_JOINTS)

    def _publish(self, targets: Dict[int, float], *, arm_kp: float, arm_kd: float, waist_pr_kp: float,
                 waist_y_kp: float, waist_kd: float, arm_sdk_weight: float = 1.0) -> None:
        self.robot._arm_sdk_client().write(
            {j: float(targets[j]) for j in UPPER_BODY_JOINTS}, weight=float(arm_sdk_weight),
            kp=float(arm_kp), kd=float(arm_kd),
            waist_kp={12: float(waist_pr_kp), 13: float(waist_pr_kp), 14: float(waist_y_kp)},
            waist_kd={j: float(waist_kd) for j in WAIST_JOINTS},
        )

    def _publish_zero_gains(self, hold: Dict[int, float]) -> None:
        self.robot._arm_sdk_client().write(
            {j: float(hold[j]) for j in UPPER_BODY_JOINTS}, kp=0.0, kd=0.0,
            waist_kp={j: 0.0 for j in WAIST_JOINTS}, waist_kd={j: 0.0 for j in WAIST_JOINTS},
        )

    # -- Initialisation --------------------------------------------------------

    def _seed_from_state(self) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snap = self._state_snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self.seeded = True
                self._sync_ee_from_joints()
                self.status = "Connected"
                return
            time.sleep(0.02)

    def _sync_ee_from_joints(self) -> None:
        """Recompute EE target poses via FK from the desired joint angles."""
        for arm, joints in ARM_JOINTS.items():
            q = np.array([self.desired_targets[j] for j in joints])
            self.target_T[arm] = self._fk[arm].compute_arm(q).copy()

    def _sync_targets_to_live(self) -> None:
        """Use the latest measured joints as both ramp state and EE target."""
        snap = self._state_snapshot()
        self._set_targets_to_positions(snap[0] if snap else self.latest_positions)

    def _set_targets_to_positions(self, positions: Dict[int, float]) -> None:
        self.latest_positions = dict(positions)
        self.current_targets = dict(positions)
        self.desired_targets = dict(positions)
        self._sync_ee_from_joints()
        for arm in ("left", "right"):
            self.ik_info[arm] = self._empty_ik_info()

    def _wait_for_state(self, timeout: float = 2.0) -> Dict[int, float]:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            snap = self._state_snapshot()
            if snap:
                pos, _ = snap
                self.latest_positions = dict(pos)
                return dict(pos)
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for rt/lowstate.")

    def _release_arms(self, duration_s: float = 3.0, command_rate_hz: float = 50.0) -> None:
        self._engage_on_seed = False
        positions = self._wait_for_state(timeout=3.0)
        self._set_targets_to_positions(positions)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        base_pr = self.waist_pr_kp if self.waist_enabled else 0.0
        base_y = self.waist_y_kp if self.waist_enabled else 0.0
        base_kd = self.waist_kd if self.waist_enabled else 0.0
        for step_idx in range(steps + 1):
            ratio = float(step_idx) / float(steps)
            authority = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio)
            self._publish(positions, arm_kp=self.arm_kp * authority, arm_kd=self.arm_kd * authority,
                          waist_pr_kp=base_pr * authority, waist_y_kp=base_y * authority,
                          waist_kd=base_kd * authority, arm_sdk_weight=authority)
            time.sleep(dt)

    def _unrelease_arms(self, duration_s: float = 1.0, command_rate_hz: float = 50.0) -> None:
        positions = self._wait_for_state(timeout=3.0)
        self._set_targets_to_positions(positions)
        steps = max(1, int(max(0.0, float(duration_s)) * max(1.0, float(command_rate_hz))))
        dt = 1.0 / max(1.0, float(command_rate_hz))
        for step_idx in range(steps + 1):
            self._publish(positions, arm_kp=self.arm_kp, arm_kd=self.arm_kd,
                          waist_pr_kp=self.waist_pr_kp if self.waist_enabled else 0.0,
                          waist_y_kp=self.waist_y_kp if self.waist_enabled else 0.0,
                          waist_kd=self.waist_kd if self.waist_enabled else 0.0,
                          arm_sdk_weight=float(step_idx) / float(steps))
            time.sleep(dt)

    # -- Pose file and sequence helpers ----------------------------------------

    def _load_pose_file(self) -> None:
        self.saved_poses = []
        self.sequence_steps = []
        if self.pose_path.exists():
            try:
                payload = json.loads(self.pose_path.read_text(encoding="utf-8"))
                poses = payload.get("poses", [])
                if isinstance(poses, list):
                    self.saved_poses = [p for p in poses if isinstance(p, dict) and isinstance(p.get("arm_joints"), dict)]
                seq = payload.get("sequence", [])
                if isinstance(seq, list):
                    self.sequence_steps = [
                        {"pose_index": int(s.get("pose_index", -1)), "include_waist": bool(s.get("include_waist", True))}
                        for s in seq if isinstance(s, dict)
                    ]
            except Exception as exc:
                self.status = f"Could not read pose file: {exc}"
        self.sequence_steps = [s for s in self.sequence_steps if 0 <= int(s["pose_index"]) < len(self.saved_poses)]

    def _write_pose_file(self) -> None:
        self.pose_path.parent.mkdir(parents=True, exist_ok=True)
        self.pose_path.write_text(
            json.dumps({"poses": self.saved_poses, "sequence": self.sequence_steps}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _pose_payload(self, name: str) -> Dict[str, Any]:
        # Save the commanded target, not lowstate feedback. With a payload in
        # the hand, feedback can sit below the target due to compliance/sag;
        # saving it would discard intentional lift compensation taught via IK.
        src = self.desired_targets if self.seeded else self.current_targets
        return {
            "name": name,
            "saved_at": datetime.now(timezone.utc).isoformat(),
            "arm_joints": {str(j): float(src[j]) for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS},
            "waist_joints": {str(j): float(src[j]) for j in WAIST_JOINTS},
        }

    def _apply_joint_pose(self, pose: Dict[str, Any], *, include_waist: bool = True) -> None:
        arm_joints = pose.get("arm_joints")
        if not isinstance(arm_joints, dict):
            raise ValueError("pose missing arm_joints")
        for j in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            if str(j) in arm_joints:
                self.desired_targets[j] = float(arm_joints[str(j)])
        waist_joints = pose.get("waist_joints")
        if include_waist and isinstance(waist_joints, dict):
            for j in WAIST_JOINTS:
                if str(j) in waist_joints:
                    self.desired_targets[j] = float(waist_joints[str(j)])
        self._sync_ee_from_joints()
        for arm in ("left", "right"):
            self.ik_info[arm] = self._empty_ik_info()

    def _targets_reached(self, eps: float = 0.01) -> bool:
        return all(abs(float(self.current_targets[j]) - float(self.desired_targets[j])) <= eps for j in UPPER_BODY_JOINTS)

    def _advance_sequence(self, now: float) -> None:
        if not self.sequence_running:
            return
        if self.sequence_step_index >= len(self.sequence_steps):
            self.sequence_running = False
            self.status = "Sequence completed"
            return
        if self.sequence_next_time_s < 0.0:
            if self._targets_reached():
                self.sequence_next_time_s = now + max(0.0, self.sequence_gap_s)
            return
        if self.sequence_next_time_s > 0.0 and now < self.sequence_next_time_s:
            return
        step = self.sequence_steps[self.sequence_step_index]
        pose_index = int(step.get("pose_index", -1))
        if not 0 <= pose_index < len(self.saved_poses):
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
        self.sequence_step_index += 1
        self.sequence_next_time_s = -1.0 if not self._targets_reached() else now + max(0.0, self.sequence_gap_s)
        self.status = (f"Seq step {self.sequence_step_index}/{len(self.sequence_steps)}: "
                       f"{pose.get('name', '<unnamed>')} ({'waist on' if include_waist else 'arms only'})")

    # -- Helpers -----------------------------------------------------------------

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

    def _ensure_hand_control_for_grip(self) -> None:
        """Enable intuitive grip control when the hand mode is still off."""
        if self.hand_control_mode == "off":
            self.hand_control_mode = "follow-arm"

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
        q = np.array([self.latest_positions.get(j, self.current_targets[j]) for j in ARM_JOINTS[arm]])
        return self._fk[arm].compute_arm(q)

    def _fk_desired(self, arm: str) -> np.ndarray:
        q = np.array([self.desired_targets[j] for j in ARM_JOINTS[arm]])
        return self._fk[arm].compute_arm(q)

    def _solve_position_shoulder_elbow(self, arm: str, T_des: np.ndarray, q_init: np.ndarray, *,
                                       selected_axis: Optional[int] = None) -> Tuple[Optional[np.ndarray], Dict]:
        """Position-only DLS with wrist joints held fixed at q_init[4:7]."""
        q = _clamp_q(q_init.copy(), arm)
        lam, eps, max_iter = 0.05, 1e-5, 64
        best_q = q.copy()
        best_err_pos = best_axis_err = float("inf")
        for iteration in range(max_iter):
            T_cur = self._fk[arm].compute_arm(q)
            pos_err = T_des[:3, 3] - T_cur[:3, 3]
            err_pos = float(np.linalg.norm(pos_err))
            axis_err = abs(float(pos_err[selected_axis])) if selected_axis is not None else err_pos
            if (err_pos, axis_err) < (best_err_pos, best_axis_err):
                best_q, best_err_pos, best_axis_err = q.copy(), err_pos, axis_err
            if err_pos < POSITION_IK_TOL_M:
                return q, {"success": True, "error_pos_m": err_pos, "error_rot_rad": 0.0,
                           "iterations": iteration, "mode": "pos_shoulder_elbow"}
            J = np.zeros((3, len(SHOULDER_ELBOW_IDXS)), dtype=np.float64)
            p0 = T_cur[:3, 3]
            for col, idx in enumerate(SHOULDER_ELBOW_IDXS):
                q1 = q.copy()
                q1[idx] += eps
                J[:, col] = (self._fk[arm].compute_arm(q1)[:3, 3] - p0) / eps
            dq_active = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(3), pos_err)
            norm_dq = float(np.linalg.norm(dq_active))
            if norm_dq > 0.3:
                dq_active *= 0.3 / norm_dq
            q_next = q.copy()
            for col, idx in enumerate(SHOULDER_ELBOW_IDXS):
                q_next[idx] += dq_active[col]
            q = _clamp_q(q_next, arm)
            q[4:] = q_init[4:]
        T_cur = self._fk[arm].compute_arm(best_q)
        err_pos = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
        axis_err = abs(float(T_des[selected_axis, 3] - T_cur[selected_axis, 3])) if selected_axis is not None else err_pos
        if selected_axis is not None and axis_err < POSITION_IK_AXIS_TOL_M and err_pos < POSITION_IK_SOFT_LIMIT_M:
            return best_q, {"success": True, "error_pos_m": err_pos, "error_rot_rad": 0.0, "iterations": max_iter,
                            "mode": "pos_axis_clamped", "axis_error_m": axis_err}
        return None, {"success": False, "error_pos_m": err_pos, "error_rot_rad": 0.0, "iterations": max_iter,
                      "mode": "pos_shoulder_elbow"}

    # -- IK and joint application ------------------------------------------------

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
            for scale in (1.0, 0.5, 0.25, 0.1):
                T_new = T_prev.copy()
                scaled_delta = arm_delta * scale
                if self.dof_idx < 3:
                    # Position move: optionally free the orientation target so
                    # IK only needs to satisfy position (orient_stiff = False).
                    if not self.orient_stiff:
                        T_new[:3, :3] = self._fk_desired(arm)[:3, :3]
                    T_new[self.dof_idx, 3] += scaled_delta
                else:
                    T_new[:3, :3] = _ROT_BY_AXIS[self.dof_idx - 3](scaled_delta) @ T_new[:3, :3]
                self.target_T[arm] = T_new
                if self._apply_ik(arm, shoulder_elbow_only=shoulder_elbow_only,
                                  selected_axis=self.dof_idx if shoulder_elbow_only else None):
                    if scale < 1.0:
                        self.status = f"{arm} IK accepted {scale:.0%} step; near limit or singularity"
                    break
            else:
                self.target_T[arm] = T_prev

    def _apply_ik(self, arm: str, *, shoulder_elbow_only: bool = False, selected_axis: Optional[int] = None) -> bool:
        """Solve IK toward target_T[arm]; on success write the max_dq-clamped
        joint deltas into desired_targets, on failure return False."""
        joints = ARM_JOINTS[arm]
        q_init = np.array([self.desired_targets[j] for j in joints])
        if shoulder_elbow_only:
            q_sol, info = self._solve_position_shoulder_elbow(arm, self.target_T[arm], q_init, selected_axis=selected_axis)
        else:
            q_sol, info = self._ik[arm].solve(self.target_T[arm], q_init=q_init)
        self.ik_info[arm] = info
        if q_sol is None:
            return False
        q_apply = q_init + np.clip(q_sol - q_init, -self.max_dq, self.max_dq)
        for i, j in enumerate(joints):
            self.desired_targets[j] = float(q_apply[i])
        if info.get("iterations") == 0 and np.allclose(q_apply, q_init):
            # The increment is inside the solver tolerance (a stick at 25 Hz
            # asks for ~5 mm per tick): keep the requested target so streamed
            # increments accumulate instead of resetting to the unchanged pose.
            return True
        self.target_T[arm] = self._fk[arm].compute_arm(q_apply).copy()
        return True

    # -- Control loop ------------------------------------------------------------

    def _ramp_step(self, dt: float) -> None:
        step = max(1e-9, self.max_speed * dt)
        for j in UPPER_BODY_JOINTS:
            cur, des = float(self.current_targets[j]), float(self.desired_targets[j])
            d = des - cur
            self.current_targets[j] = des if abs(d) <= step else cur + math.copysign(step, d)

    def _loop(self) -> None:
        while self._running:
            started = time.monotonic()
            with self._lock:
                if not self._running:
                    break
                try:
                    self.tick()
                except Exception as exc:
                    self.status = f"Tick error: {exc}"
            time.sleep(max(0.002, 1.0 / self.rate_hz - (time.monotonic() - started)))

    def tick(self) -> None:
        snap = self._state_snapshot()
        if snap is not None:
            pos, _ = snap
            self.latest_positions = pos
            if not self.seeded:
                self.seeded = True
                self.current_targets = dict(pos)
                self.desired_targets = dict(pos)
                self._sync_ee_from_joints()
            if self._engage_on_seed:
                self._engage_on_seed = False
                self._unrelease_arms()
                self.armed = True
                self.status = "Connected · arms engaged"
        now = time.monotonic()
        self._process_remote(now)
        if not self.seeded or not self.armed:
            self._last_tick = now
            return
        dt = min(1.0 / self.rate_hz, now - self._last_tick)
        self._last_tick = now
        self._advance_sequence(now)
        self._ramp_step(dt)
        try:
            self._publish(self.current_targets, arm_kp=self.arm_kp, arm_kd=self.arm_kd,
                          waist_pr_kp=self.waist_pr_kp if self.waist_enabled else 0.0,
                          waist_y_kp=self.waist_y_kp if self.waist_enabled else 0.0,
                          waist_kd=self.waist_kd if self.waist_enabled else 0.0)
        except Exception as exc:
            self.status = f"Publish error: {exc}"
        if now - self._last_hand_publish_s >= 1.0 / self.hand_rate_hz:
            self._publish_hand_targets_once()
            self._last_hand_publish_s = now

    @staticmethod
    def _axis_value(axes: List[Any], index: int, deadzone: float = 0.12) -> float:
        try:
            value = max(-1.0, min(1.0, float(axes[index])))
        except (IndexError, TypeError, ValueError):
            return 0.0
        if abs(value) <= deadzone:
            return 0.0
        return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)

    @staticmethod
    def _stick_value(axes: List[Any], ix: int, iy: int, deadzone: float) -> Tuple[float, float]:
        """Radial deadzone: a stick resting slightly off-centre reads as zero."""
        try:
            x = max(-1.0, min(1.0, float(axes[ix])))
            y = max(-1.0, min(1.0, float(axes[iy])))
        except (IndexError, TypeError, ValueError):
            return 0.0, 0.0
        mag = math.hypot(x, y)
        if mag <= deadzone:
            return 0.0, 0.0
        scale = min(1.0, (mag - deadzone) / (1.0 - deadzone)) / mag
        return x * scale, y * scale

    def _loco_cmd(self, axes: List[Any]) -> Tuple[float, float]:
        lx, ly = self._stick_value(axes, 0, 1, self.loco_deadzone)
        return -ly * self.max_vx, -lx * self.max_vyaw

    def _input_stale(self, now: float) -> bool:
        return not self.remote.last_received_s or now - self.remote.last_received_s > INPUT_STALE_S

    def _process_remote(self, now: float) -> None:
        state = self.remote.snapshot()
        remote_stale = self._input_stale(now)
        if remote_stale:
            state = {}
            # Flush the active locomotion command once when the stream drops.
            # Do not repeatedly enqueue zero commands while it is offline.
            if not self._remote_loco_stopped:
                try:
                    self.robot.loco_stop()
                except Exception as exc:
                    self.status = f"Remote stop error: {exc}"
                self._remote_loco_stopped = True
        axes = state.get("axes", [])
        buttons = [bool(v) for v in state.get("buttons", [])]
        hats = state.get("hats", [[0, 0]])
        hat = hats[0] if hats and isinstance(hats[0], (list, tuple)) else [0, 0]
        previous = self._remote_prev_buttons

        def pressed(index: int) -> bool:
            return index < len(buttons) and buttons[index] and not (index < len(previous) and previous[index])

        if pressed(1):  # B: release / reengage arm authority
            try:
                if self.armed:
                    self._release_arms()
                    self.armed = False
                    self.status = "Remote B: arms released"
                else:
                    self._unrelease_arms()
                    self.armed = True
                    self.status = "Remote B: arms reengaged"
            except Exception as exc:
                self.status = f"Remote arm toggle failed: {exc}"
        if pressed(3):  # Y: orientation constraint toggle
            self.orient_stiff = not self.orient_stiff
            self.status = f"Remote Y: EE orientation {'locked' if self.orient_stiff else 'free'}"
        if pressed(0):  # A: toggle grip open/closed
            self._remote_grip_closed = not self._remote_grip_closed
            self.hand_grip_percent = 100.0 if self._remote_grip_closed else 0.0
            self._ensure_hand_control_for_grip()
            self._publish_hand_targets_once()
            self.status = f"Remote A: hand {'closed' if self._remote_grip_closed else 'open'}"
        if pressed(2):  # X: right -> left -> both
            modes = self._remote_mode_arms
            idx = modes.index(self.arm_control_mode) if self.arm_control_mode in modes else 0
            self.arm_control_mode = modes[(idx + 1) % len(modes)]
            self.status = f"Remote X: arm selection {self.arm_control_mode}"

        # D-pad vertical changes EE z in discrete, repeatable increments;
        # horizontal changes EE roll.
        try:
            hat_x, hat_y = int(hat[0]), int(hat[1])
        except (IndexError, TypeError, ValueError):
            hat_x = hat_y = 0
        if hat_y and self.seeded and self.armed and now - self._remote_dpad_last_s >= 0.16:
            self.dof_idx = 2
            self._adjust_dof(math.copysign(self.pos_step, hat_y))
            self._remote_dpad_last_s = now
        if hat_x and self.seeded and self.armed and now - self._remote_dpad_x_last_s >= 0.16:
            self.dof_idx = 3
            self._adjust_dof(math.copysign(self.rot_step, hat_x))
            self._remote_dpad_x_last_s = now

        # Axes: 0/1 left stick, 2/3 right stick.  Stick vertical is inverted
        # for natural forward/up movement.
        rx = self._axis_value(axes, 2)
        ry = self._axis_value(axes, 3)
        if self.seeded and self.armed and (rx or ry):
            dt = min(0.05, max(0.0, now - self._remote_ee_last_s))
            self._remote_ee_last_s = now
            # Each stick axis is its own small Cartesian increment: diagonal
            # input stays responsive without building a backlog of large
            # target jumps.  Right X -> EE y; right Y -> EE x.
            self.dof_idx = 1
            self._adjust_dof(rx * self.max_ee_y_speed * dt)
            self.dof_idx = 0
            self._adjust_dof(-ry * self.max_ee_speed * dt)

        # Left stick drives locomotion (vx and yaw only).
        vx, vyaw = self._loco_cmd(axes)
        active = bool(vx or vyaw)
        loco_cmd = (round(vx, 3), round(vyaw, 3))
        changed = self._remote_last_loco_cmd is None or any(
            abs(a - b) >= 0.01 for a, b in zip(loco_cmd, self._remote_last_loco_cmd)
        )
        if remote_stale:
            pass
        elif active:
            # Each Move carries a ~1 s lease (non-continuous) refreshed every
            # 0.2 s, so a lost stop or a stalled loop cannot leave the robot
            # walking on an old command.
            if changed or now - self._remote_last_loco_s >= 0.2:
                try:
                    rc = self.robot.loco_move(vx, 0.0, vyaw, duration_s=None, continuous=False)
                    if rc != 0:
                        self.status = f"Remote locomotion rc={rc}"
                    self._remote_last_loco_s = now
                    self._remote_loco_was_active = True
                    self._remote_last_loco_cmd = loco_cmd
                    self._remote_loco_stopped = False
                except Exception as exc:
                    self.status = f"Remote locomotion error: {exc}"
        else:
            if self._remote_loco_was_active:
                # Stick released: repeat StopMove for a short window instead
                # of trusting a single RPC.
                self._remote_loco_was_active = False
                self._remote_stop_until_s = now + 0.6
                self._remote_last_loco_s = 0.0
            if now <= self._remote_stop_until_s and now - self._remote_last_loco_s >= 0.15:
                try:
                    self.robot.loco_stop()
                    self._remote_last_loco_s = now
                    self._remote_last_loco_cmd = loco_cmd
                except Exception as exc:
                    self.status = f"Remote stop error: {exc}"
        self._remote_prev_buttons = buttons

    def _publish_hand_targets_once(self) -> None:
        for hand in self._active_hands():
            try:
                self.robot.write_dex3_grip(hand, self.hand_grip_percent, kp=self.hand_kp, kd=self.hand_kd, tau=self.hand_tau)
                self.hand_info[hand] = f"{self.hand_grip_percent:.0f}%"
            except Exception as exc:
                self.hand_info[hand] = f"write failed: {exc}"

    # -- Actions (the v4 keyboard functions) -------------------------------------

    def action(self, name: str, body: Dict[str, Any]) -> Dict[str, Any]:
        """Run one v4 key function. Raises ValueError for bad input."""
        with self._lock:
            handler = getattr(self, f"_act_{name}", None)
            if handler is None or self._closed:
                raise ValueError(f"Unknown /wbc action {name!r}" if handler is None else "WBC session is closed")
            handler(body)
            return {"ok": True, "status": self.status}

    def _act_select_dof(self, body: Dict[str, Any]) -> None:
        self.dof_idx = max(0, min(N_DOFS - 1, int(body.get("dof", self.dof_idx))))

    def _act_jog(self, body: Dict[str, Any]) -> None:
        if "dof" in body:
            self._act_select_dof(body)
        if not (self.seeded and self.armed):
            self.status = "Arms are released; re-engage before jogging"
            return
        direction = 1.0 if float(body.get("dir", 1)) >= 0 else -1.0
        self._adjust_dof(direction * self._ee_step())

    def _act_step(self, body: Dict[str, Any]) -> None:
        if "dof" in body:
            self._act_select_dof(body)
        op = body.get("op")
        if op == "halve":
            self._set_ee_step(max(0.0001, self._ee_step() / 2.0))
        elif op == "double":
            self._set_ee_step(min(1.0 if self.dof_idx < 3 else math.pi, self._ee_step() * 2.0))
        else:
            hi = 1.0 if self.dof_idx < 3 else math.pi
            self._set_ee_step(max(0.0001, min(hi, float(body["value"]))))

    def _act_max_dq(self, body: Dict[str, Any]) -> None:
        op = body.get("op")
        if op == "halve":
            self.max_dq = max(0.005, self.max_dq / 2.0)
        elif op == "double":
            self.max_dq = min(math.pi, self.max_dq * 2.0)
        else:
            self.max_dq = max(0.005, min(math.pi, float(body["value"])))
        self.status = f"max_dq -> {self.max_dq:.4f} rad"

    def _act_arm_mode(self, body: Dict[str, Any]) -> None:
        mode = body.get("mode")
        if mode not in ARM_CONTROL_MODES:
            mode = ARM_CONTROL_MODES[(ARM_CONTROL_MODES.index(self.arm_control_mode) + 1) % len(ARM_CONTROL_MODES)]
        self.arm_control_mode = mode
        self.status = f"Arm mode -> {self.arm_control_mode}"

    def _act_hand_mode(self, body: Dict[str, Any]) -> None:
        mode = body.get("mode")
        if mode not in HAND_CONTROL_MODES:
            mode = HAND_CONTROL_MODES[(HAND_CONTROL_MODES.index(self.hand_control_mode) + 1) % len(HAND_CONTROL_MODES)]
        self.hand_control_mode = mode
        self.status = f"Dex3 hand mode -> {self.hand_control_mode}"

    def _act_grip(self, body: Dict[str, Any]) -> None:
        self._ensure_hand_control_for_grip()
        if "delta" in body:
            self.hand_grip_percent = max(0.0, min(100.0, self.hand_grip_percent + float(body["delta"])))
        else:
            self.hand_grip_percent = max(0.0, min(100.0, float(body["percent"])))
        self._remote_grip_closed = self.hand_grip_percent >= 50.0
        self._publish_hand_targets_once()
        self._last_hand_publish_s = time.monotonic()
        self.status = f"Dex3 grip -> {self.hand_grip_percent:.0f}%"

    def _act_orient(self, body: Dict[str, Any]) -> None:
        self.orient_stiff = bool(body["value"]) if "value" in body else not self.orient_stiff
        self.status = ("Orient stiff ON - rotation locked during x/y/z moves" if self.orient_stiff
                       else "Orient stiff OFF - x/y/z IK uses shoulder+elbow only")

    def _act_resync(self, _body: Dict[str, Any]) -> None:
        self._sync_targets_to_live()
        self.status = "EE targets resynced to current hand pose"

    def _act_waist(self, _body: Dict[str, Any]) -> None:
        self.waist_enabled = not self.waist_enabled
        if self.waist_enabled:
            # Hold the waist where it physically is right now so it doesn't
            # lurch to a stale desired position and disturb IK.
            for j in WAIST_JOINTS:
                self.desired_targets[j] = self.latest_positions.get(j, self.desired_targets[j])
            self.current_targets.update({j: self.desired_targets[j] for j in WAIST_JOINTS})
        # Resync EE targets so a physical shift caused by the waist change
        # doesn't leave stale unreachable targets.
        self._sync_ee_from_joints()
        self.status = f"Waist {'ENABLED (held)' if self.waist_enabled else 'DISABLED (free)'} - EE resynced"

    def _act_release(self, _body: Dict[str, Any]) -> None:
        try:
            self._release_arms()
            self.armed = False
            self.status = "Arms released - move freely, engage to take control again"
        except Exception as exc:
            self.status = f"Release failed: {exc}"

    def _act_engage(self, _body: Dict[str, Any]) -> None:
        try:
            self._unrelease_arms()
            self.armed = True
            self.status = "Reengaged - holding current arm pose"
        except Exception as exc:
            self.status = f"Reengage failed: {exc}"

    def _act_zero_gains(self, _body: Dict[str, Any]) -> None:
        self._publish_zero_gains(self.current_targets)
        self.status = "Zero-gain hold sent on rt/arm_sdk"

    def _act_params(self, body: Dict[str, Any]) -> None:
        attrs = {"speed_rad_s": "max_speed", "kp": "arm_kp", "kd": "arm_kd", "ee_speed_m_s": "max_ee_speed",
                 "ee_y_speed_m_s": "max_ee_y_speed"}
        changed = []
        for key, value in body.items():
            if key not in PARAMS:
                continue
            low, high = PARAMS[key][1:]
            setattr(self, attrs.get(key, key), max(low, min(high, float(value))))
            changed.append(key)
        if "sequence_gap_s" in changed:
            self._write_pose_file()
        self.status = "Updated " + ", ".join(changed) if changed else self.status

    def _act_source(self, body: Dict[str, Any]) -> None:
        source = body.get("source")
        url = str(body.get("url") or self.remote_url).strip()
        if source not in ("remote", "browser"):
            raise ValueError("source must be remote or browser")
        if source == self.remote.kind and (source == "browser" or url == self.remote_url):
            return
        self.remote.close()
        self.remote_url = url
        self.remote = BrowserGamepadInput() if source == "browser" else RemoteStateStream(url)
        self._remote_prev_buttons = []
        self.status = f"Input source -> {'browser gamepad' if source == 'browser' else url}"

    def gamepad(self, state: Dict[str, Any]) -> Dict[str, Any]:
        remote = self.remote
        if not isinstance(remote, BrowserGamepadInput):
            return {"ok": False, "message": "input source is not the browser gamepad"}
        remote.update(state)
        return {"ok": True}

    def _act_save_pose(self, body: Dict[str, Any]) -> None:
        name = str(body.get("name", "")).strip()
        if not name:
            raise ValueError("Pose name is required")
        self.saved_poses.append(self._pose_payload(name))
        self._write_pose_file()
        self.status = f"Saved pose '{name}'"

    def _pose_index(self, body: Dict[str, Any]) -> int:
        index = int(body.get("index", -1))
        if not 0 <= index < len(self.saved_poses):
            raise ValueError("No such pose")
        return index

    def _act_load_pose(self, body: Dict[str, Any]) -> None:
        pose = self.saved_poses[self._pose_index(body)]
        try:
            self._apply_joint_pose(pose, include_waist=True)
            self.status = f"Loaded pose '{pose.get('name', '<unnamed>')}'"
        except Exception as exc:
            self.status = f"Load failed: {exc}"

    def _act_delete_pose(self, body: Dict[str, Any]) -> None:
        row = self._pose_index(body)
        name = str(self.saved_poses[row].get("name", f"pose_{row}"))
        new_steps = []
        for step in self.sequence_steps:
            pose_index = int(step.get("pose_index", -1))
            if pose_index == row:
                continue
            new_steps.append({"pose_index": pose_index - (1 if pose_index > row else 0),
                              "include_waist": bool(step.get("include_waist", True))})
        self.sequence_steps = new_steps
        del self.saved_poses[row]
        self.sequence_running = False
        self._write_pose_file()
        self.status = f"Deleted pose '{name}'"

    def _act_seq_add(self, body: Dict[str, Any]) -> None:
        index = self._pose_index(body)
        self.sequence_steps.append({"pose_index": index, "include_waist": self.include_waist_new})
        self._write_pose_file()
        name = self.saved_poses[index].get("name", f"pose_{index}")
        self.status = f"Added '{name}' to sequence ({'with waist' if self.include_waist_new else 'arms only'})"

    def _seq_index(self, body: Dict[str, Any]) -> int:
        index = int(body.get("index", -1))
        if not 0 <= index < len(self.sequence_steps):
            raise ValueError("No such sequence step")
        return index

    def _act_seq_remove(self, body: Dict[str, Any]) -> None:
        del self.sequence_steps[self._seq_index(body)]
        self.sequence_running = False
        self._write_pose_file()
        self.status = "Removed sequence step"

    def _act_seq_move(self, body: Dict[str, Any]) -> None:
        row = self._seq_index(body)
        other = row + (1 if int(body.get("dir", 1)) > 0 else -1)
        if 0 <= other < len(self.sequence_steps):
            self.sequence_steps[row], self.sequence_steps[other] = self.sequence_steps[other], self.sequence_steps[row]
            self._write_pose_file()

    def _act_seq_toggle_waist(self, body: Dict[str, Any]) -> None:
        if "index" in body:
            step = self.sequence_steps[self._seq_index(body)]
            step["include_waist"] = not bool(step.get("include_waist", True))
            self._write_pose_file()
            return
        self.include_waist_new = not self.include_waist_new
        self.status = "New sequence steps include waist" if self.include_waist_new else "New sequence steps are arms-only"

    def _act_seq_run(self, _body: Dict[str, Any]) -> None:
        if not self.armed:
            self.status = "Reengage arms before running a sequence"
        elif not self.sequence_steps:
            self.status = "Add poses to the sequence first"
        else:
            self.sequence_running = True
            self.sequence_step_index = 0
            self.sequence_next_time_s = 0.0
            self.status = "Sequence started"

    def _act_seq_stop(self, _body: Dict[str, Any]) -> None:
        self.sequence_running = False
        self.sequence_step_index = 0
        self.sequence_next_time_s = 0.0
        self.status = "Sequence stopped"

    # -- Status ------------------------------------------------------------------

    def snapshot(self) -> Dict[str, Any]:
        if not self._lock.acquire(timeout=0.25):
            return {"busy": True, "status": self.status}
        try:
            return self._snapshot()
        finally:
            self._lock.release()

    def _snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        state = self.remote.snapshot()
        axes = state.get("axes", [])
        vx, vyaw = self._loco_cmd(axes)
        disp = self._display_arm()
        T_cur = self._fk_live(disp) if self.seeded else np.eye(4)
        T_tgt = self.target_T[disp]
        cur_rpy, tgt_rpy = _rpy_from_R(T_cur[:3, :3]), _rpy_from_R(T_tgt[:3, :3])
        dofs = []
        for i in range(N_DOFS):
            live = float(T_cur[i, 3]) if i < 3 else cur_rpy[i - 3]
            target = float(T_tgt[i, 3]) if i < 3 else tgt_rpy[i - 3]
            dofs.append({"name": DOF_NAMES[i], "unit": DOF_UNITS[i], "live": live, "target": target,
                         "step": self._ee_step(i), "free": i < 3 and not self.orient_stiff})
        joints = {}
        for arm, idxs in ARM_JOINTS.items():
            joints[arm] = {"target": [self.desired_targets[j] for j in idxs],
                           "live": [self.latest_positions.get(j, self.current_targets[j]) for j in idxs]}
        names = [str(p.get("name", f"pose_{i}")) for i, p in enumerate(self.saved_poses)]
        return {
            "active": True, "seeded": self.seeded, "armed": self.armed, "status": self.status,
            "input": {"source": self.remote.kind, "url": self.remote_url, "error": self.remote.error,
                      "stale": self._input_stale(now), "name": getattr(self.remote, "name", ""),
                      "age": None if not self.remote.last_received_s else now - self.remote.last_received_s,
                      "axes": axes, "buttons": state.get("buttons", []), "hats": state.get("hats", [[0, 0]]),
                      "loco": [vx, vyaw]},
            "arm_mode": self.arm_control_mode, "display_arm": disp, "orient_stiff": self.orient_stiff,
            "waist_enabled": self.waist_enabled, "dof_idx": self.dof_idx, "dofs": dofs,
            "ik": {arm: self.ik_info[arm] for arm in self._active_arms()},
            "hand": {"mode": self.hand_control_mode, "grip": self.hand_grip_percent,
                     "info": {h: self.hand_info.get(h, "off") for h in self._active_hands()}},
            "joints": joints, "joint_labels": list(JOINT_LABELS),
            "params": {"rate_hz": self.rate_hz, "speed_rad_s": self.max_speed, "max_dq": self.max_dq,
                       "kp": self.arm_kp, "kd": self.arm_kd, "waist_pr_kp": self.waist_pr_kp,
                       "waist_y_kp": self.waist_y_kp, "waist_kd": self.waist_kd,
                       "ee_speed_m_s": self.max_ee_speed, "ee_y_speed_m_s": self.max_ee_y_speed,
                       "max_vx": self.max_vx, "max_vyaw": self.max_vyaw, "loco_deadzone": self.loco_deadzone,
                       "pos_step": self.pos_step, "rot_step": self.rot_step, "hand_kp": self.hand_kp,
                       "hand_kd": self.hand_kd, "hand_tau": self.hand_tau, "hand_rate_hz": self.hand_rate_hz,
                       "sequence_gap_s": self.sequence_gap_s},
            "pose_file": str(self.pose_path),
            "poses": [{"name": n, "saved_at": str(p.get("saved_at", ""))[:19]} for n, p in zip(names, self.saved_poses)],
            "sequence": {"steps": [{"name": names[int(s["pose_index"])] if 0 <= int(s["pose_index"]) < len(names) else "<missing>",
                                    "include_waist": bool(s.get("include_waist", True))} for s in self.sequence_steps],
                         "running": self.sequence_running, "index": self.sequence_step_index,
                         "include_waist_new": self.include_waist_new},
        }

    # -- Shutdown ----------------------------------------------------------------

    def close(self) -> None:
        """Stop locomotion, fade arm authority out, and release the lock."""
        if self._closed or self._closing:
            return
        self._closing = True
        self._running = False
        thread = getattr(self, "_thread", None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=4.0)
        with self._lock:
            self.sequence_running = False
            try:
                self.remote.close()
            except Exception:
                pass
            try:
                self.robot.loco_stop()
            except Exception:
                pass
            try:
                if self.armed:
                    self._release_arms()
                    self.armed = False
            except Exception:
                pass
            finally:
                self._closed = True
                self._closing = False
                self._controller_lock.release()
