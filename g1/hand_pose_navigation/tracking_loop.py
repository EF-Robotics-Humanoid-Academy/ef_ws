"""
Step 10 — Continuous TF feedback tracking
==========================================
Ties all pipeline steps together in a closed-loop controller that
continuously:
    1. Grabs a fresh RGB-D frame
    2. Detects the target pose
    3. Broadcasts the TF frame
    4. Looks up T_base_target via TF
    5. Computes desired hand pose
    6. Solves IK
    7. Checks reachability
    8. Sends the arm command

The loop runs at a configurable rate and terminates when the hand
converges to within tolerance of the target, or on timeout.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional

import numpy as np

from .target_detector import TargetDetector, DetectionResult
from .detected_pose_publisher import DetectedPosePublisher
from .arm_fk import ArmFK, SHOULDER_IN_BASE
from .tf_utils import TFUtils
from .grasp_planner import GraspPlanner
from .arm_ik import ArmIK
from .reachability_checker import ReachabilityChecker
from .arm_executor import ArmExecutor
from .obstacle_checker import Obstacles

_BODY_JOINT_INDEX_BY_LABEL = {
    "left_arm.shoulder_pitch": 15,
    "left_arm.shoulder_roll": 16,
    "left_arm.shoulder_yaw": 17,
    "left_arm.elbow": 18,
    "left_arm.wrist_roll": 19,
    "left_arm.wrist_pitch": 20,
    "left_arm.wrist_yaw": 21,
    "right_arm.shoulder_pitch": 22,
    "right_arm.shoulder_roll": 23,
    "right_arm.shoulder_yaw": 24,
    "right_arm.elbow": 25,
    "right_arm.wrist_roll": 26,
    "right_arm.wrist_pitch": 27,
    "right_arm.wrist_yaw": 28,
}


# ---------------------------------------------------------------------------
# Status
# ---------------------------------------------------------------------------

@dataclass
class LoopStatus:
    running: bool = False
    converged: bool = False
    stalled: bool = False
    iteration: int = 0
    last_error_pos_m: float = float("inf")
    last_error_rot_rad: float = float("inf")
    total_elapsed_s: float = 0.0
    ik_failures: int = 0
    detection_failures: int = 0
    safety_rejections: int = 0
    log: List[str] = field(default_factory=list)

    def record(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        entry = f"[{ts}] {msg}"
        self.log.append(entry)
        if len(self.log) > 200:
            del self.log[:-200]
        print(entry)

    def to_dict(self) -> Dict:
        def finite_or_none(value: float) -> Optional[float]:
            return value if math.isfinite(value) else None

        return {
            "running": self.running,
            "converged": self.converged,
            "stalled": self.stalled,
            "iteration": self.iteration,
            "last_error_pos_m": finite_or_none(self.last_error_pos_m),
            "last_error_rot_rad": finite_or_none(self.last_error_rot_rad),
            "total_elapsed_s": finite_or_none(self.total_elapsed_s),
            "ik_failures": self.ik_failures,
            "detection_failures": self.detection_failures,
            "safety_rejections": self.safety_rejections,
            "log": list(self.log[-80:]),
        }


# ---------------------------------------------------------------------------
# Main tracking loop
# ---------------------------------------------------------------------------

class TrackingLoop:
    """
    Step 10: Closed-loop hand-to-target tracking controller.

    Args:
        robot:               Robot instance (sdk_client.Robot)
        detector:            TargetDetector instance
        pose_publisher:      DetectedPosePublisher instance
        tf_utils:            TFUtils instance
        fk:                  ArmFK instance
        grasp_planner:       GraspPlanner instance
        ik:                  ArmIK instance
        checker:             ReachabilityChecker instance
        executor:            ArmExecutor instance
        arm:                 "left" | "right"
        rate_hz:             control loop rate
        convergence_pos_m:   position tolerance to declare convergence
        convergence_rot_rad: rotation tolerance to declare convergence
        timeout_s:           maximum time before giving up (0 = run forever)
        on_converge:         optional callback when hand reaches target
    """

    # If the hand hasn't gotten meaningfully closer to the target over this
    # many consecutive EE-pose increments, the grab is considered stalled
    # (e.g. the object moved out from under the fixed target mid-grab, or the
    # arm is stuck against a workspace/obstacle limit) and the loop aborts
    # rather than continuing to chase a target that isn't converging.
    _STALL_WINDOW = 30
    _STALL_PROGRESS_TOL_M = 0.001

    def __init__(
        self,
        robot,
        detector: TargetDetector,
        pose_publisher: DetectedPosePublisher,
        tf_utils: TFUtils,
        fk: ArmFK,
        grasp_planner: GraspPlanner,
        ik: ArmIK,
        checker: ReachabilityChecker,
        executor: ArmExecutor,
        arm: str = "right",
        rate_hz: float = 10.0,
        convergence_pos_m: float = 0.015,
        convergence_rot_rad: float = 0.05,
        max_joint_step_rad: float = 0.08,
        timeout_s: float = 30.0,
        on_converge: Optional[Callable[[], None]] = None,
        static_obstacles: Optional[Obstacles] = None,
    ) -> None:
        self.robot = robot
        self.detector = detector
        self.pose_publisher = pose_publisher
        self.tf_utils = tf_utils
        self.fk = fk
        self.grasp_planner = grasp_planner
        self.ik = ik
        self.checker = checker
        self.executor = executor
        self.arm = arm
        self.rate_hz = rate_hz
        self.convergence_pos_m = convergence_pos_m
        self.convergence_rot_rad = convergence_rot_rad
        self.max_joint_step_rad = max(0.0, float(max_joint_step_rad))
        self.timeout_s = timeout_s
        self.on_converge = on_converge
        # Table plane etc. — doesn't move during a grab, computed once by the
        # caller from the frame at grab start. The opposite-arm proxy is
        # recomputed every iteration below from live joint state instead,
        # since that arm can move mid-grab.
        self.static_obstacles = static_obstacles

        self._stop_event = threading.Event()
        # Bumped by notify_target_updated() when the caller corrects a
        # "fixed" target mid-grab; the loop then restarts its stall window
        # so the jump in error isn't judged against the old target.
        self._target_revision = 0
        self._status = LoopStatus()
        self._thread: Optional[threading.Thread] = None

        self._joint_indices = (
            list(range(15, 22)) if arm == "left" else list(range(22, 29))
        )
        self._opposite_arm = "left" if arm == "right" else "right"
        self._opposite_fk = ArmFK(arm=self._opposite_arm, backend="urdf")

    # ------------------------------------------------------------------
    def start(self, blocking: bool = True) -> LoopStatus:
        """Start the tracking loop. If blocking=False, runs in a thread."""
        self._stop_event.clear()
        self._status = LoopStatus(running=True)

        if blocking:
            self._run()
        else:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return self._status

    def stop(self) -> None:
        """Signal the loop to stop cleanly."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.executor.stop_continuous()

    def notify_target_updated(self) -> None:
        """Signal that the detector's target was corrected mid-run."""
        self._target_revision += 1

    @property
    def status(self) -> LoopStatus:
        return self._status

    # ------------------------------------------------------------------
    def _run(self) -> None:
        dt = 1.0 / self.rate_hz
        start_t = time.time()
        q_arm_commanded: Optional[np.ndarray] = None
        err_history: Deque[float] = deque(maxlen=self._STALL_WINDOW)

        try:
            self.executor.start_continuous()
        except Exception as exc:
            self._status.record(f"[Step 9] Could not start continuous executor: {exc}")
            self._status.running = False
            return

        try:
            self._run_loop(dt, start_t, q_arm_commanded, err_history)
        finally:
            if not (self._status.converged or self._status.stalled):
                self.executor.stop_continuous()
            self._status.running = False

    def _run_loop(
        self,
        dt: float,
        start_t: float,
        q_arm_commanded: Optional[np.ndarray],
        err_history: Deque[float],
    ) -> None:
        seen_target_revision = self._target_revision
        while not self._stop_event.is_set():
            t0 = time.time()
            self._status.iteration += 1
            if self._target_revision != seen_target_revision:
                seen_target_revision = self._target_revision
                err_history.clear()
                self._status.record("[Step 5] Target corrected — stall window restarted.")
            elapsed = t0 - start_t
            self._status.total_elapsed_s = elapsed

            if self.timeout_s > 0 and elapsed > self.timeout_s:
                self._status.record(
                    f"[Step 10] Timeout after {elapsed:.1f}s — stopping."
                )
                break

            # ── Step 1-2: get frame + detect ──────────────────────────
            if getattr(self.detector, "method", "") == "fixed":
                rgb_bgr = np.zeros((1, 1, 3), dtype=np.uint8)
                depth_m = np.ones((1, 1), dtype=np.float32)
            else:
                try:
                    frame = self.robot.get_rgbd(timeout=0.5)
                    rgb_bgr = frame.get("rgb_bgr")
                    depth_m = frame.get("depth_m")
                    if rgb_bgr is None or depth_m is None:
                        self._status.detection_failures += 1
                        self._status.record("[Step 2] No RGBD frame — skipping.")
                        self._sleep(dt, t0)
                        continue
                except Exception as exc:
                    self._status.detection_failures += 1
                    self._status.record(f"[Step 2] get_rgbd error: {exc}")
                    self._sleep(dt, t0)
                    continue

            detection: Optional[DetectionResult] = self.detector.detect(
                rgb_bgr,
                depth_m,
            )
            if detection is None:
                self._status.detection_failures += 1
                self._status.record("[Step 2] No detection — skipping.")
                self._sleep(dt, t0)
                continue

            # ── Step 3: broadcast TF ──────────────────────────────────
            self.pose_publisher.update(detection)

            # ── Step 4: get current hand pose (FK) ────────────────────
            q_full = self._read_q_full()
            if q_full is None:
                self._status.safety_rejections += 1
                self._status.record("[Step 4] Joint state unavailable — skipping command.")
                self._sleep(dt, t0)
                continue
            T_base_hand = self.fk.compute(q_full)
            q_arm_cur = q_full[self._joint_indices]

            # ── Step 5: look up target in base frame ──────────────────
            T_base_target = self.tf_utils.base_to_target(timeout_s=0.05)
            if T_base_target is None:
                self._status.record("[Step 5] TF lookup failed — skipping.")
                self._sleep(dt, t0)
                continue

            # ── Step 6: desired hand pose ─────────────────────────────
            T_base_desired = self.grasp_planner.compute(T_base_target)

            # ── Pre-IK workspace check ────────────────────────────────
            reach_check = self.checker.check_target_reachable(T_base_desired)
            if not reach_check.safe:
                self._status.safety_rejections += 1
                self._status.record(
                    f"[Step 8] Workspace fail: {reach_check.reasons}"
                )
                if getattr(self.detector, "method", "") == "fixed":
                    self._status.record(
                        "[Step 8] Fixed selected target is unreachable — stopping grab."
                    )
                    break
                self._sleep(dt, t0)
                continue

            # ── Check if already converged ────────────────────────────
            err_pos, err_rot = self._pose_error_scalars(
                T_base_desired,
                T_base_hand,
            )
            self._status.last_error_pos_m = err_pos
            self._status.last_error_rot_rad = err_rot

            if (
                err_pos < self.convergence_pos_m
                and err_rot < self.convergence_rot_rad
            ):
                self._status.converged = True
                self._status.record(
                    f"[Step 10] Converged! pos_err={err_pos:.4f}m  "
                    f"rot_err={err_rot:.4f}rad  iter={self._status.iteration}"
                )
                if self.on_converge:
                    self.on_converge()
                break

            # ── Stall check: hand isn't getting closer to the target ──
            err_history.append(err_pos)
            if (
                len(err_history) == err_history.maxlen
                and err_history[-1] >= err_history[0] - self._STALL_PROGRESS_TOL_M
            ):
                self._status.stalled = True
                delta_xyz = T_base_desired[:3, 3] - T_base_hand[:3, 3]
                self._status.record(
                    "[Step 10] Stalled: target not getting closer over last "
                    f"{err_history.maxlen} EE-pose increments "
                    f"({err_history[0]:.4f}m -> {err_history[-1]:.4f}m); "
                    f"desired_xyz=({T_base_desired[0, 3]:+.3f},"
                    f"{T_base_desired[1, 3]:+.3f},{T_base_desired[2, 3]:+.3f}) "
                    f"current_xyz=({T_base_hand[0, 3]:+.3f},"
                    f"{T_base_hand[1, 3]:+.3f},{T_base_hand[2, 3]:+.3f}) "
                    f"delta_xyz=({delta_xyz[0]:+.3f},{delta_xyz[1]:+.3f},"
                    f"{delta_xyz[2]:+.3f}) — aborting."
                )
                break

            # ── Step 7: IK ───────────────────────────────────────────
            q_arm_desired, ik_info = self.ik.solve(
                T_base_desired,
                q_init=q_arm_cur,
            )
            if q_arm_desired is None:
                self._status.ik_failures += 1
                p_des = T_base_desired[:3, 3]
                self._status.record(
                    f"[Step 7] IK failed: pos_err={ik_info['error_pos_m']:.4f} "
                    f"target=({p_des[0]:+.3f},{p_des[1]:+.3f},{p_des[2]:+.3f})"
                )
                self._sleep(dt, t0)
                continue
            if ik_info.get("fallback"):
                self._status.record(
                    f"[Step 7] IK fallback={ik_info['fallback']} "
                    f"pos_err={ik_info['error_pos_m']:.4f}"
                )

            # ── Step 8: safety check ─────────────────────────────────
            safety = self.checker.check(q_arm_desired, T_base_desired)
            if not safety.safe:
                self._status.safety_rejections += 1
                self._status.record(f"[Step 8] Safety fail: {safety.reasons}")
                self._sleep(dt, t0)
                continue

            q_arm_start_cmd = q_arm_commanded if q_arm_commanded is not None else q_arm_cur
            q_arm_next = q_arm_desired.copy()
            if self.max_joint_step_rad > 0.0:
                delta = q_arm_desired - q_arm_start_cmd
                max_abs_delta = float(np.max(np.abs(delta))) if delta.size else 0.0
                if max_abs_delta > self.max_joint_step_rad:
                    scale = self.max_joint_step_rad / max_abs_delta
                    q_arm_next = q_arm_start_cmd + delta * scale
                    self._status.record(
                        f"[Step 9] Joint increment capped: "
                        f"max_delta={max_abs_delta:.3f}rad cap={self.max_joint_step_rad:.3f}rad"
                    )

            # ── Step 8b: swept-path obstacle check ────────────────────
            obstacles = None
            if self.static_obstacles is not None:
                T_opposite = self._opposite_fk.compute(q_full)
                obstacles = Obstacles(
                    table=self.static_obstacles.table,
                    opposite_shoulder_base=SHOULDER_IN_BASE[self._opposite_arm],
                    opposite_wrist_base=T_opposite[:3, 3],
                    torso_radius_m=self.static_obstacles.torso_radius_m,
                    torso_z=self.static_obstacles.torso_z,
                    self_collision_radius_m=self.static_obstacles.self_collision_radius_m,
                    table_margin_m=self.static_obstacles.table_margin_m,
                    table_xy_margin_m=self.static_obstacles.table_xy_margin_m,
                )

            # ── Step 9: submit to the continuous execution thread ────
            # Validation only, here — the continuous thread (started once,
            # outside this loop) owns actual publishing and keeps rt/arm_sdk
            # fed at a steady rate regardless of how long this iteration's
            # perception/IK took, so a slow iteration never leaves the wire
            # silent.
            validation = self.executor.validate(
                q_arm_next,
                T_base_desired=T_base_desired,
                q_arm_start=q_arm_start_cmd,
                obstacles=obstacles,
            )
            if not validation.get("safe") and validation.get("reason") == "obstacle_gate":
                self._status.safety_rejections += 1
                self._status.record(
                    f"[Step 8b] Obstacle in swept path: {validation.get('violations')}"
                )
                self._sleep(dt, t0)
                continue
            if not validation.get("safe"):
                self._status.safety_rejections += 1
                self._status.record(
                    f"[Step 9] Command rejected: reason={validation.get('reason')}"
                )
                self._sleep(dt, t0)
                continue
            self.executor.submit_target(q_arm_next)
            q_arm_commanded = q_arm_next.copy()

            self._status.record(
                f"[Step 10] it={self._status.iteration:4d}  "
                f"pos_err={err_pos:.4f}m  rot_err={err_rot:.3f}rad  "
                f"ik_iter={ik_info['iterations']}"
            )

            self._sleep(dt, t0)

    # ------------------------------------------------------------------
    def _read_q_full(self) -> Optional[np.ndarray]:
        try:
            js = self.robot.get_joint_states()
            if not js:
                return None
            q = np.zeros(30)
            seen_indices = set()
            for name, data in js.get("joints", {}).items():
                idx = _joint_entry_index(name, data)
                if 0 <= idx < 30:
                    q[idx] = data.get("position", 0.0)
                    seen_indices.add(int(idx))
            if not all(idx in seen_indices for idx in self._joint_indices):
                return None
            return q
        except Exception:
            return None

    @staticmethod
    def _pose_error_scalars(
        T_des: np.ndarray,
        T_cur: np.ndarray,
    ) -> tuple[float, float]:
        pos_err = float(np.linalg.norm(T_des[:3, 3] - T_cur[:3, 3]))
        R_err = T_des[:3, :3] @ T_cur[:3, :3].T
        rot_err = float(
            np.linalg.norm(
                np.array(
                    [
                        R_err[2, 1] - R_err[1, 2],
                        R_err[0, 2] - R_err[2, 0],
                        R_err[1, 0] - R_err[0, 1],
                    ]
                )
            )
            * 0.5
        )
        return pos_err, rot_err

    @staticmethod
    def _sleep(dt: float, t_start: float) -> None:
        elapsed = time.time() - t_start
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)


def _joint_entry_index(label: str, data: Dict) -> int:
    if "index" in data:
        try:
            return int(data.get("index", -1))
        except Exception:
            return -1
    return int(_BODY_JOINT_INDEX_BY_LABEL.get(str(label), -1))
