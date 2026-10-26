"""V2 /sort: an explicit, recoverable pick-and-place sequence.

Reuses sorting.Sorter for the configuration, ArUco pattern tracking, object
classification and the live view; replaces the sort sequence with
sorting_state_machine.SortStateMachine driving grasp_pipeline_v2 motions:

  PREPARING -> DETECTING -> VALIDATING_TARGET -> MOVING_TO_PREGRASP ->
  APPROACHING -> CLOSING_HAND -> VERIFYING_GRASP -> LIFTING -> VERIFYING_LIFT ->
  MOVING_TO_DROPOFF -> LOWERING -> RELEASING_OBJECT -> VERIFYING_PLACEMENT ->
  RETRACTING -> READY_FOR_NEXT_OBJECT -> (DETECTING ... | FINALIZING -> COMPLETED)

Every phase returns a PhaseResult; a failure goes to RECOVERING, whose
action depends on what the hand may be holding, and from there to the next
object, or to ERROR (a controlled hold that needs the operator).  An operator
stop goes to ABORTED: the arm holds where it is and a possibly loaded hand
is never opened automatically.  The Stop button is a software cancellation,
not an emergency stop: use the robot's physical e-stop / damping for that.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import grasp_pipeline as gp1
from grasp_pipeline import base_from_camera, palm_visual_point, pregrasp_hand_pose
from grasp_pipeline_v2 import CarriedObject, GraspPipelineV2, MotionContext, MoveResult, interpolate_pose, yaw_error
from hand_pose_navigation.arm_fk import SHOULDER_IN_BASE
from sorting import (
    OBJECT_COUNT, PATTERN_LETTERS, PLACE_APPROACH_EXTRA_M, SORT_ARM, ObjectConfig, PatternConfig, Sorter,
)
from reach_map_v2 import ReachMap
from sorting_state_machine import Failure, PhaseResult, SortStateMachine, State, TransitionError

S = State
# Finger directions (deg about the vertical through the object) tried when
# planning a grasp; 0 = pointing from the shoulder at the object.
# Turning the fingers lets the pre-grasp pull-back go sideways instead of into
# the torso keep-out (objects close to the robot need ~10 cm of pull-back).
GRASP_YAW_CANDIDATES_DEG = (0, 15, -15, 30, -30, 45, -45, 60, -60)
DROP_OUT_OF_REACH = "drop-off out of reach"
# Drop points tried on a spot: object centre at least this far inside the
# marker area, on this grid, nearest to the centre first.
DROP_EDGE_MARGIN_M = 0.015
DROP_GRID_STEP_M = 0.015
DROP_MAX_IK_CHECKS = 12


def rotate_about_vertical(T: np.ndarray, centre: np.ndarray, angle: float) -> np.ndarray:
    """T turned by `angle` about the vertical axis through `centre`."""
    c, s_ = math.cos(angle), math.sin(angle)
    Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    out = T.copy()
    out[:3, :3] = Rz @ T[:3, :3]
    rel = T[:3, 3] - np.array([centre[0], centre[1], T[2, 3]])
    out[:3, 3] = np.array([centre[0], centre[1], T[2, 3]]) + Rz @ rel
    return out




@dataclass
class Target:
    obj_index: int
    det: gp1.Detection
    p_base: np.ndarray
    score: float
    consistent: int


@dataclass
class GraspPlan:
    target: Target
    T_object: np.ndarray
    T_grasp: np.ndarray
    T_pre: np.ndarray
    T_clear: np.ndarray
    objects: List[gp1.TableObject]
    table: Any
    carried_radius_m: float
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class BlacklistEntry:
    obj_index: int
    xy: np.ndarray
    reason: str
    until: float


class SorterV2(Sorter):
    def __init__(self, pipeline: GraspPipelineV2) -> None:
        super().__init__(pipeline)
        self.p: GraspPipelineV2 = pipeline
        self.cfg = pipeline.cfg
        self.sm = SortStateMachine()
        self.prep: Optional[Dict[str, Any]] = None
        self._manual_hand_busy = False
        # The start-up chain from the hanging arm (q_start -> q_away -> q_up ->
        # recipe): played in reverse before the arms are released.
        self.home: Optional[Dict[str, np.ndarray]] = None
        self.attempts = [0] * OBJECT_COUNT
        self.confirmed = [0] * OBJECT_COUNT
        self.unconfirmed = [0] * OBJECT_COUNT
        self.finished = [False] * OBJECT_COUNT
        self.notes: List[str] = []
        self.blacklist: List[BlacklistEntry] = []
        self.current: Optional[GraspPlan] = None
        self.pick_record: Dict[str, Any] = {}
        self._arm_lock_held = False
        # Motion-level events (plans, IK failures, rejections, progress) go to
        # the same structured log as the state transitions.
        pipeline.event = lambda kind, message: self.sm.log.add(self.sm.state, kind, message)
        # Reach view: computed once per table height (separate process, disk cache).
        self.reach = ReachMap(SORT_ARM, [q for _name, q in gp1.extend_arm_forward_joint_waypoints(SORT_ARM)],
                              pick_height=self.cfg.grasp_wrist_floor_m() + 0.005, place_height=0.13,
                              standoffs=(0.08, self.cfg.hand.max_grasp_standoff_m))

    # -- logging -------------------------------------------------------------------
    def _log(self, line: str) -> None:
        super()._log(line)
        sm = getattr(self, "sm", None)  # Sorter.__init__ logs before ours runs
        if sm is not None:
            sm.info(line)

    def _set_phase(self, phase: str) -> None:
        with self.lock:
            self.phase = phase
        self.p.log(f"[sort-v2] {phase}")

    def _enter(self, state: State, message: str = "", **data: Any) -> None:
        self.sm.enter(state, message, **data)
        self._set_phase(f"{state.value}" + (f": {message}" if message else ""))
        self.p.diag(state=state.value)

    # -- actions -------------------------------------------------------------------
    def action(self, name: str) -> Dict[str, Any]:
        p = self.p
        actions = {
            "sort": (self._run_sort, "Sort V2 started - see the state and event log."),
            "stop": (self._request_stop, "Stopping: the arm holds where it is."),
            "retract": (self._manual_return_to_prep, "Returning to the preparation pose..."),
            "hand_open": (lambda: self._manual_hand("open"), "Opening right hand..."),
            "hand_close": (lambda: self._manual_hand("close"), "Closing right hand..."),
            "release_arms": (self._manual_release_arms, "Releasing arms..."),
            "engage_arms": (p.run_engage_arms, "Re-engaging arms..."),
            "damp": (self._manual_damp, "Damping..."),
        }
        if name not in actions:
            raise ValueError(f"Unknown /sort action {name!r}")
        with self.lock:
            running = self.running
        if name == "sort":
            if running:
                raise ValueError("A sort is already running")
            if self.config_errors:
                raise ValueError("Fix the configuration first: " + " ".join(self.config_errors))
        elif name != "stop" and running:
            raise ValueError("Stop the running sort first")
        if name in ("hand_open", "hand_close") and self._manual_hand_busy:
            raise ValueError("The previous hand command is still running (finger recovery can take a few seconds)")
        if name in ("release_arms", "damp"):
            self.stop_event.set()
        self.touch()
        target, message = actions[name]
        threading.Thread(target=target, name=f"sort-v2-{name}", daemon=True).start()
        return {"ok": True, "message": message}

    def _request_stop(self) -> None:
        self.stop_event.set()
        self.sm.warn("operator stop requested (software cancellation, not an emergency stop)")

    def _manual_hand(self, which: str) -> None:
        with self.p.lock:
            holding = self.p.holding_object
        if which == "open" and holding:
            self.sm.warn("Opening the hand on operator request while it may hold an object.")
        self._manual_hand_busy = True
        try:
            ok, detail = self.p.grip.open(self.cfg.tuning.release_open_s) if which == "open" else self.p.grip.close_neutral()
        finally:
            self._manual_hand_busy = False
        if which == "open" and ok:
            self.p.clear_hold_flags()
        self.sm.info(f"manual hand {which}: {'ok' if ok else 'FAILED'} ({detail})")

    def _manual_return_to_prep(self) -> None:
        if self.prep is None:
            self.sm.warn("No preparation pose recorded yet; using the V1 stable retract.")
            self.p.run_retract_stable_pose()
            return
        with self.p.lock:
            holding = self.p.holding_object
        if holding:
            self.sm.warn("Refusing to retract: the hand may hold an object. Open the hand over a safe spot first.")
            return
        if not self._acquire_arm():
            return
        self.stop_event.clear()
        try:
            self.p.v2_active = True
            self.p.ensure_arm_authority(duration_s=0.5)
            self.p.controller.start()
            result = self._return_to_prep(self.p.current_table(), [])
            self.sm.info(f"manual return to preparation pose: {result.detail}")
        finally:
            self.p.v2_active = False
            self._release_arm()

    def _manual_release_arms(self) -> None:
        self.p.controller.stop()
        self.p.grip.stop_hold()
        self.p.run_release_arms()

    def _manual_damp(self) -> None:
        self.p.controller.stop()
        self.p.grip.stop_hold()
        self.p.run_damp()

    def set_options(self, body: Dict[str, Any]) -> Dict[str, Any]:
        tuning = body.pop("tuning", None) if isinstance(body, dict) else None
        if tuning:
            applied = self.cfg.tuning.update(tuning, self.cfg.safety)
            self.sm.info(f"tuning updated: {applied}")
        return super().set_options(body)

    def options(self) -> Dict[str, Any]:
        out = super().options()
        out["tuning"] = {k: getattr(self.cfg.tuning, k) for k in self.cfg.tuning.ADJUSTABLE}
        out["tuning_bounds"] = {k: list(v) for k, v in self.cfg.tuning.ADJUSTABLE.items()}
        return out

    # -- arm ownership ---------------------------------------------------------------
    def _acquire_arm(self, timeout_s: float = 5.0) -> bool:
        if self._arm_lock_held:
            return True
        if not self.p.arm_lock.acquire(timeout=timeout_s):
            self.sm.warn("The arm controller is busy (another motion owns the arm).")
            return False
        self._arm_lock_held = True
        return True

    def _release_arm(self) -> None:
        if self._arm_lock_held:
            self._arm_lock_held = False
            self.p.arm_lock.release()

    # -- sequence ------------------------------------------------------------------
    def _run_sort(self) -> None:
        if not self.run_lock.acquire(blocking=False):
            self._log("Sort already running.")
            return
        p = self.p
        p.keep_alive += 1
        try:
            self.stop_event.clear()
            p.cancel.clear()
            with self.lock:
                objects = list(self.objects)
                errors = list(self.config_errors)
                self.running = True
                self.sorted_counts = [0] * OBJECT_COUNT
                self.summary = ""
            self.attempts = [0] * OBJECT_COUNT
            self.confirmed = [0] * OBJECT_COUNT
            self.unconfirmed = [0] * OBJECT_COUNT
            self.finished = [obj.count <= 0 for obj in objects]
            self.notes = []
            self.blacklist = []
            self.current = None
            self.pick_record = {}
            p.clear_diag()
            if errors:
                self.sm.warn("Sort refused: " + " ".join(errors))
                return
            if self.sm.state not in (S.IDLE, S.COMPLETED, S.ABORTED, S.ERROR):
                self.sm.reset()
            if self.sm.state != S.IDLE:
                self.sm.reset()
            self._enter(S.PREPARING)
            resume = self._object_at_palm(objects)
            result = self._prepare_in_place() if resume is not None else self._prepare()
            if not result.ok:
                self._recover(S.PREPARING, result)
                return
            self._enter(S.DETECTING)
            if resume is not None:
                target, obj_r = resume
                self._enter(S.VALIDATING_TARGET, f"{obj_r.name} is already at the palm: resuming from the grasp")
                planned = self._validate_resume(target, obj_r)
                if planned.ok:
                    self.current = planned.data["plan"]
                    state, result = self._pick_and_place(self.current, obj_r)
                    if not result.ok:
                        self.attempts[target.obj_index] += 1
                        if not self._recover(state, result):
                            return
                else:
                    self.sm.fail(planned)
                    self._note_target_failure(target, planned)
                if self.sm.state != S.DETECTING:
                    self._enter(S.DETECTING)
                if not self._remaining(objects):
                    self._enter(S.FINALIZING, "all requested objects handled")
                    self._finalize(objects)
                    return
            while True:
                if self.stop_event.is_set():
                    self._recover(self.sm.state, PhaseResult.fail(Failure.OPERATOR_STOP, "stop requested"))
                    return
                found = self._detect(objects)
                if not found.ok:
                    if found.failure == Failure.OPERATOR_STOP:
                        self._recover(S.DETECTING, found)
                        return
                    self.sm.fail(found)
                    self._enter(S.FINALIZING, found.detail)
                    break
                target: Target = found.data["target"]
                self._enter(S.VALIDATING_TARGET, f"{objects[target.obj_index].name} at "
                            f"({target.p_base[0]:+.3f}, {target.p_base[1]:+.3f}, {target.p_base[2]:+.3f}) m")
                planned = self._validate(target, objects[target.obj_index])
                if not planned.ok:
                    self.sm.fail(planned)
                    self._note_target_failure(target, planned)
                    if planned.failure == Failure.OPERATOR_STOP:
                        self._recover(S.VALIDATING_TARGET, planned)
                        return
                    self._enter(S.DETECTING, f"target rejected ({planned.failure.value})")
                    continue
                self.current = planned.data["plan"]
                state, result = self._pick_and_place(self.current, objects[target.obj_index])
                if result.ok:
                    if self._remaining(objects):
                        self._enter(S.DETECTING)
                        continue
                    self._enter(S.FINALIZING, "all requested objects handled")
                    break
                self.attempts[target.obj_index] += 1
                # Only a failure on the way to the object says something about
                # the object's position; a drop-off failure does not.
                if state in (S.MOVING_TO_PREGRASP, S.APPROACHING) and result.failure in (
                        Failure.COLLISION_REJECTED, Failure.IK_FAILURE, Failure.NOT_REACHED, Failure.UNREACHABLE):
                    self._blacklist(target, result.failure.value)
                if self.attempts[target.obj_index] >= self.cfg.tuning.max_attempts_per_object:
                    self.finished[target.obj_index] = True
                    self.notes.append(f"gave up on {objects[target.obj_index].name} after "
                                      f"{self.attempts[target.obj_index]} attempts ({result.failure.value})")
                if not self._recover(state, result):
                    return
                if self._remaining(objects):
                    self._enter(S.DETECTING)
                    continue
                self._enter(S.FINALIZING, "no attempts left")
                break
            self._finalize(objects)
        except TransitionError as exc:
            self.sm.log.add(self.sm.state, "error", f"state machine: {exc}")
            self._hold_error(f"internal sequencing error: {exc}")
        except Exception as exc:  # never leave the arm uncontrolled on a bug
            self.sm.log.add(self.sm.state, "error", f"unexpected {type(exc).__name__}: {exc}")
            self._hold_error(f"unexpected {type(exc).__name__}: {exc}")
        finally:
            p.v2_active = False
            self._release_arm()
            with self.lock:
                self.running = False
                self.current_target = None
                self.active_object = None
            p.keep_alive -= 1
            self.run_lock.release()

    def _remaining(self, objects: List[ObjectConfig]) -> bool:
        for i, obj in enumerate(objects):
            if not self.finished[i] and self.confirmed[i] + self.unconfirmed[i] < obj.count:
                return True
            if self.confirmed[i] + self.unconfirmed[i] >= obj.count:
                self.finished[i] = True
        return False

    # -- phase 0 -------------------------------------------------------------------
    def _object_at_palm(self, objects: List[ObjectConfig]) -> Optional[Tuple[Target, ObjectConfig]]:
        """A configured, unsorted object inside the open hand's capture box
        (and the palm level enough to grasp it): Sort then resumes from the
        grasp instead of moving to the preparation pose first."""
        p, t = self.p, self.cfg.tuning
        try:
            T_hand = p.measured_hand_T()
        except Exception:
            return None
        if math.acos(float(np.clip(T_hand[2, 2], -1.0, 1.0))) > 2 * t.grasp_tilt_tolerance_rad:
            return None
        snap = self._scene_snapshot()
        for e in self._classify(snap):
            i = e["obj"]
            if i is None or not e["sortable"] or self.finished[i]:
                continue
            inside, d = p.in_capture(T_hand, e["p_base"])
            if inside:
                self.sm.info(f"{objects[i].name} is between the fingers (hand frame {d[0]:+.3f}, {d[1]:+.3f}, "
                             f"{d[2]:+.3f} m): resuming from the grasp")
                det = e["det"]
                return Target(i, gp1.copy_detection(det), e["p_base"].copy(), float(det.score), 1), objects[i]
        return None

    def _prepare_in_place(self) -> PhaseResult:
        """Take the arm over where it is (no preparation motion)."""
        p = self.p
        if p.busy():
            return PhaseResult.fail(Failure.CONTROLLER_CONFLICT, "another arm motion is running")
        if not self._acquire_arm():
            return PhaseResult.fail(Failure.CONTROLLER_CONFLICT, "could not acquire the arm controller")
        p.v2_active = True
        with p.nav_lock:
            nav = p.active_nav
        p.shutdown_nav(nav, "sort v2 resume")
        gp1.GraspPipeline.stop_active_hold(p)
        try:
            p.ensure_arm_authority(duration_s=0.8)
        except Exception as exc:
            return PhaseResult.fail(Failure.PREPARATION_FAILURE, f"engaging arm SDK control failed: {exc}")
        p.controller.start()
        if p.current_table() is None:
            return PhaseResult.fail(Failure.SENSOR_UNAVAILABLE, "no table plane in the depth image")
        self.sm.info("arm taken over in place (object already at the palm)"
                     + ("" if self.prep is not None else "; the preparation pose is recorded after the placement"))
        return PhaseResult.success("in place")

    def _validate_resume(self, target: Target, obj: ObjectConfig) -> PhaseResult:
        """Plan only the drop-off for an object already in the hand."""
        p, t = self.p, self.cfg.tuning
        table = p.current_table()
        T_hand = p.measured_hand_T()
        with p.lock:
            cam = dict(p.camera_extrinsic)
        T_object = base_from_camera(cam) @ target.det.T_camera_object
        T_object[:3, 3] = target.p_base
        objects = p.obstacle_objects(T_object, time.time())
        own = [o for o in (p.table_objects or []) if float(np.linalg.norm(o.center_xy - target.p_base[:2])) < o.radius_m + 0.04]
        radius = own[0].radius_m if own else 0.04
        carried = CarriedObject(T_hand[:3, :3].T @ (target.p_base - T_hand[:3, 3]), float(T_hand[2, 3] - table.z_base_m),
                                radius)
        drop, drop_detail = self._drop_pose_for(obj, T_hand[:3, :3], carried)
        if drop is None:
            return PhaseResult.fail(Failure.UNREACHABLE, f"{DROP_OUT_OF_REACH}: {drop_detail}", finish_object=True)
        T_lift = T_hand.copy()
        T_lift[:3, :3] = gp1.level_rotation(T_hand[:3, :3])
        T_lift[2, 3] += t.lift_m
        transport = self._transport_route(T_lift, drop[1], carried, table, objects)
        lift_ctx = MotionContext(table, objects, min_wrist_z_m=T_hand[2, 3] - 0.015, label="lift")
        failure, why, _q = self._check_key_poses([("lift", T_lift, lift_ctx)]
                                                 + [(f"{ctx.label} end", T1, ctx) for _T0, T1, ctx, _y in transport],
                                                 p.controller.reference(), T_from=T_hand)
        if failure is not None:
            return PhaseResult.fail(failure, f"drop-off not feasible from the current grasp: {why}")
        plan = GraspPlan(target, T_object, T_hand.copy(), T_hand.copy(), T_hand.copy(), objects, table, radius,
                         {"drop": drop, "drop_detail": drop_detail, "resume": True})
        self.sm.info(f"resume plan: drop-off {drop_detail}")
        return PhaseResult.success("validated", plan=plan)

    def _prepare(self) -> PhaseResult:
        p, t = self.p, self.cfg.tuning
        if p.busy():
            return PhaseResult.fail(Failure.CONTROLLER_CONFLICT, "another arm motion is running")
        if not self._acquire_arm():
            return PhaseResult.fail(Failure.CONTROLLER_CONFLICT, "could not acquire the arm controller")
        with p.lock:
            if p.holding_object:
                return PhaseResult.fail(Failure.PREPARATION_FAILURE, "the hand may still hold an object")
        p.v2_active = True
        with p.nav_lock:
            nav = p.active_nav
        p.shutdown_nav(nav, "sort v2 preparation")
        gp1.GraspPipeline.stop_active_hold(p)       # V1 hold executor, if any
        try:
            p.ensure_arm_authority(duration_s=0.8)
        except Exception as exc:
            return PhaseResult.fail(Failure.PREPARATION_FAILURE, f"engaging arm SDK control failed: {exc}")
        p.controller.start()
        table = p.current_table()
        if table is None:
            return PhaseResult.fail(Failure.SENSOR_UNAVAILABLE, "no table plane in the depth image")
        if self.prep is not None:
            result = self._return_to_prep(table, [])
        else:
            result = self._move_to_recipe_prep(table)
        if not result.ok:
            return PhaseResult.fail(result.failure or Failure.PREPARATION_FAILURE, result.detail)
        opened, detail = p.grip.open(t.release_open_s)
        if not opened:
            return PhaseResult.fail(Failure.FINGER_FAILURE, f"hand did not open at the preparation pose: {detail}")
        q_meas = p.read_current_arm_q(SORT_ARM)
        T_meas = p.fk.compute_arm(q_meas)
        clear_z = table.z_base_m + self.cfg.safety.table_margin_m
        if T_meas[2, 3] < clear_z:
            return PhaseResult.fail(Failure.PREPARATION_FAILURE,
                                    f"preparation pose is too low (wrist z={T_meas[2, 3]:.3f} m < {clear_z:.3f} m)")
        self._record_prep()
        return PhaseResult.success("ready")

    def _record_prep(self) -> None:
        p = self.p
        q_meas = p.read_current_arm_q(SORT_ARM)
        T_meas = p.fk.compute_arm(q_meas)
        self.prep = {"q": p.controller.reference(), "q_measured": q_meas, "T": p.fk.compute_arm(p.controller.reference()),
                     "T_measured": T_meas, "ts": time.time()}
        self.sm.info(f"preparation pose recorded: wrist ({T_meas[0, 3]:+.3f}, {T_meas[1, 3]:+.3f}, {T_meas[2, 3]:+.3f}) m")

    def _move_to_recipe_prep(self, table: Any) -> MoveResult:
        """First preparation: V1's reviewed extend recipe, after a vertical
        clearance lift when the hand starts low over the table."""
        p, t = self.p, self.cfg.tuning
        q_start = p.controller.reference()
        hanging = self._hanging(table)
        away = self._move_away_from_body(table)
        if not away.ok:
            return away
        q_away = p.controller.reference()
        rise_path: List[np.ndarray] = []
        T_ref = p.reference_hand_T()
        clear_z = table.z_base_m + self.cfg.safety.table_margin_m + 0.02
        if T_ref[2, 3] < clear_z:
            T_up = T_ref.copy()
            T_up[2, 3] = clear_z + 0.01
            ctx = MotionContext(table, min_wrist_z_m=T_ref[2, 3] - 0.015, label="preparation clearance")
            # The vertical rise is solved before moving (1 cm steps, one IK
            # branch) and streamed at a constant joint speed: solving the
            # orientation-free IK every tick made the planner tick unevenly
            # (62 poses planned for 6 s took 9 s) and the rise stuttered.
            rise = p.plan_vertical_path(q_away, float(T_up[2, 3]))
            if rise is not None:
                rise_path = rise
                up = p.move_joint_path(rise, ctx, "preparation clearance", self.stop_event,
                                       speed_rad_s=t.prep_rise_speed_rad_s)
            else:
                up = p.move_linear(T_up, ctx, t.lift_speed_m_s, "preparation clearance", self.stop_event, level=False)
            if not up.ok:
                return up
        if hanging:
            self.home = {"q_start": q_start, "q_away": q_away, "q_up": p.controller.reference(), "rise": rise_path}
        result: MoveResult = MoveResult(True)
        for name, q in gp1.extend_arm_forward_joint_waypoints(SORT_ARM):
            result = p.move_joint(q, MotionContext(table, label=name), f"preparation {name}", self.stop_event)
            if not result.ok:
                return result
        return result

    def _hanging(self, table: Any) -> bool:
        """Hand low beside the body (arm hanging / released), not over the table."""
        T = self.p.reference_hand_T()
        return (float(T[2, 3]) < table.z_base_m + self.cfg.safety.table_margin_m
                and float(T[0, 3]) < table.x_range_m[0] - 0.05)

    def _move_away_from_body(self, table: Any) -> MoveResult:
        """Hanging arm (hand below the table clearance height): first swing
        the hand outward with the shoulder roll only, so the fingers come off
        the thigh before the arm rises.  No-op once the arm is up or out."""
        p, t = self.p, self.cfg.tuning
        q = p.controller.reference()
        T = p.fk.compute_arm(q)
        if float(T[2, 3]) >= table.z_base_m + self.cfg.safety.table_margin_m:
            return MoveResult(True, None, "arm already raised")
        if T[0, 3] >= table.x_range_m[0] - 0.05:
            # Low but over / at the table, not hanging beside the leg: the
            # retract rises vertically instead.
            return MoveResult(True, None, "hand is over the table, not beside the body")
        outward = -1.0 if SORT_ARM == "right" else 1.0
        if outward * float(q[1]) >= t.prep_abduction_rad - 0.02:
            return MoveResult(True, None, "arm already away from the body")
        lo, hi = gp1.JOINT_LIMITS[SORT_ARM][1]
        q_out = q.copy()
        q_out[1] = float(np.clip(outward * t.prep_abduction_rad, lo, hi))
        self.sm.info(f"moving the hand away from the body (shoulder roll {q[1]:+.2f} -> {q_out[1]:+.2f} rad)")
        return p.move_joint(q_out, MotionContext(table, label="away from body"), "move away from the body",
                            self.stop_event, speed_rad_s=t.prep_abduction_speed_rad_s)

    # -- phase 1 -------------------------------------------------------------------
    def _blacklisted(self, obj_index: int, p_base: np.ndarray) -> Optional[BlacklistEntry]:
        now = time.time()
        self.blacklist = [b for b in self.blacklist if b.until > now]
        for entry in self.blacklist:
            if entry.obj_index == obj_index and float(np.linalg.norm(entry.xy - p_base[:2])) < self.cfg.tuning.blacklist_radius_m:
                return entry
        return None

    def _blacklist(self, target: Target, reason: str, duration_s: float = 600.0) -> None:
        self.blacklist.append(BlacklistEntry(target.obj_index, target.p_base[:2].copy(), reason, time.time() + duration_s))

    def _note_target_failure(self, target: Target, result: PhaseResult) -> None:
        self.attempts[target.obj_index] += 1
        if result.failure in (Failure.UNREACHABLE, Failure.COLLISION_REJECTED, Failure.IK_FAILURE):
            self._blacklist(target, result.failure.value)
        if result.data.get("finish_object") or self.attempts[target.obj_index] >= self.cfg.tuning.max_attempts_per_object:
            self.finished[target.obj_index] = True
            with self.lock:
                name = self.objects[target.obj_index].name
            self.notes.append(f"{name}: {result.detail}")

    def _detect(self, objects: List[ObjectConfig]) -> PhaseResult:
        """Consistent detections over several fresh perception passes."""
        t = self.cfg.tuning
        wanted = [i for i in range(len(objects)) if not self.finished[i]]
        if not wanted:
            return PhaseResult.fail(Failure.NO_TARGET, "nothing left to sort")
        passes: List[List[Dict[str, Any]]] = []
        skipped: Dict[str, str] = {}
        last_ts = 0.0
        deadline = time.time() + t.detection_timeout_s
        while len(passes) < t.detection_passes and time.time() < deadline:
            if self.stop_event.is_set():
                return PhaseResult.fail(Failure.OPERATOR_STOP, "stop requested")
            snap = self._scene_snapshot()
            if snap["detection_ts"] <= last_ts:
                time.sleep(0.1)
                continue
            last_ts = snap["detection_ts"]
            entries = []
            for e in self._classify(snap):
                if e["obj"] not in wanted or not e["sortable"] or float(e["det"].score) < t.min_detection_score:
                    continue
                if not np.all(np.isfinite(e["p_base"])):
                    continue
                listed = self._blacklisted(e["obj"], e["p_base"])
                if listed is not None:
                    skipped[objects[e["obj"]].name] = listed.reason
                    continue
                entries.append(e)
            passes.append(entries)
        clusters: List[Dict[str, Any]] = []
        for pass_index, entries in enumerate(passes):
            for e in entries:
                for c in clusters:
                    if c["obj"] == e["obj"] and float(np.linalg.norm(c["p"] - e["p_base"])) <= t.detection_consistency_m \
                            and pass_index not in c["passes"]:
                        c["passes"].add(pass_index)
                        c["points"].append(e["p_base"])
                        c["p"] = np.mean(c["points"], axis=0)
                        c["score"] = max(c["score"], float(e["det"].score))
                        c["det"] = e["det"]
                        break
                else:
                    clusters.append({"obj": e["obj"], "p": e["p_base"].copy(), "points": [e["p_base"]],
                                     "passes": {pass_index}, "score": float(e["det"].score), "det": e["det"]})
        need = min(t.detection_min_consistent, max(1, len(passes)))
        good = [c for c in clusters if len(c["passes"]) >= need]
        self.p.diag(detection={"passes": len(passes), "clusters": len(clusters), "consistent": len(good)})
        if not good:
            seen = sum(len(e) for e in passes)
            if skipped and not seen:
                detail = ("only detections at positions that already failed were seen: "
                          + ", ".join(f"{name} ({reason})" for name, reason in skipped.items())
                          + "; move the object or fix the cause, then sort again")
            else:
                detail = (f"no consistent detection of {', '.join(objects[i].name for i in wanted)} over "
                          f"{len(passes)} passes" + (f" ({seen} single detections)" if seen else ""))
            for i in wanted:
                self.finished[i] = True
            self.notes.append(detail)
            return PhaseResult.fail(Failure.NO_TARGET, detail)
        best = max(good, key=lambda c: (len(c["passes"]), c["score"], -float(np.linalg.norm(c["p"][:2]))))
        target = Target(best["obj"], gp1.copy_detection(best["det"]), best["p"], best["score"], len(best["passes"]))
        self.p.diag(target={"object": objects[target.obj_index].name, "label": target.det.label,
                            "score": target.score, "xyz": [float(v) for v in target.p_base],
                            "consistent": f"{target.consistent}/{len(passes)}"})
        return PhaseResult.success("target found", target=target)

    # -- phase 1b: validation and planning ---------------------------------------------
    def _drop_pose_for(self, obj: ObjectConfig, R_hand: np.ndarray, carried: CarriedObject
                       ) -> Tuple[Optional[Tuple[np.ndarray, np.ndarray, float]], str]:
        """Release (and approach) hand poses that put the carried object on
        its spot, for a given hand rotation (made level) and grasp offset.

        The drop point is the point of the pattern nearest its centre that
        the palm can reach: candidates on a grid over the marker area (object
        centre >= DROP_EDGE_MARGIN_M inside the edges), filtered by the reach
        map when it is ready, then by a position-only reach check.  The
        release height keeps the object's bottom carried_object_clearance_m
        above the spot."""
        p = self.p
        with self.lock:
            pattern = self.patterns[obj.zone]
        T_camera_pattern = self._pattern_T_camera(obj.zone, require_current_epoch=True)
        if T_camera_pattern is None:
            return None, f"{pattern.name} is not visible"
        with p.lock:
            cam = dict(p.camera_extrinsic)
        centre, normal = self._pattern_drop_geometry(pattern, T_camera_pattern, cam)
        T_base_pattern = base_from_camera(cam) @ T_camera_pattern
        R = gp1.level_rotation(R_hand)
        offset = R @ np.asarray(carried.offset_hand, dtype=np.float64)
        offset[2] = 0.0
        release_h = max(float(pattern.release_height_m),
                        carried.hang_m + self.cfg.safety.carried_object_clearance_m + 0.005)
        # The marker plane (ArUco PnP) and the depth table plane disagree by
        # ~1-1.5 cm on the robot; the clearance check uses the table plane, so
        # the release never goes below what that check needs.
        table = p.current_table()
        min_palm_z = None if table is None else \
            table.z_base_m + carried.hang_m + self.cfg.safety.carried_object_clearance_m + 0.005
        m = DROP_EDGE_MARGIN_M
        us = np.arange(m, pattern.width_m - m + 1e-9, DROP_GRID_STEP_M)
        vs = np.arange(m, pattern.height_m - m + 1e-9, DROP_GRID_STEP_M)
        uc, vc = pattern.width_m / 2.0, pattern.height_m / 2.0
        q_seed = p.controller.reference()
        shoulder = np.asarray(SHOULDER_IN_BASE[SORT_ARM], dtype=np.float64)

        def poses_at(u: float, v: float):
            c = (T_base_pattern @ np.array([u, v, 0.0, 1.0]))[:3]
            T_release, T_approach = np.eye(4), np.eye(4)
            T_release[:3, :3] = T_approach[:3, :3] = R
            T_release[:3, 3] = c + normal * release_h - offset
            T_approach[:3, 3] = c + normal * (release_h + PLACE_APPROACH_EXTRA_M) - offset
            if min_palm_z is not None and T_release[2, 3] < min_palm_z:
                lift = min_palm_z - T_release[2, 3]
                T_release[2, 3] += lift
                T_approach[2, 3] += lift
            return c, T_approach, T_release

        def reachable(u: float, v: float) -> bool:
            _c, T_approach, T_release = poses_at(u, v)
            if max(float(np.linalg.norm(T[:3, 3] - shoulder)) for T in (T_approach, T_release)) > self.cfg.hardware.arm_reach_m:
                return False
            return p.reach_residual_m(T_release[:3, 3], q_seed) <= 0.005 and \
                p.reach_residual_m(T_approach[:3, 3], q_seed) <= 0.005

        def result(u: float, v: float, how: str):
            c, T_approach, T_release = poses_at(u, v)
            moved = float(np.linalg.norm(c[:2] - centre[:2]))
            return (T_approach, T_release, release_h), \
                (f"{pattern.name} (centre {centre[0]:+.3f}, {centre[1]:+.3f} m)"
                 + (f", dropping {moved * 100:.0f} cm from the centre at ({c[0]:+.3f}, {c[1]:+.3f}) m ({how})"
                    if moved > 0.005 else ""))

        if reachable(uc, vc):
            return result(uc, vc, "centre")
        if self.reach.state == "ready":
            # Map-filtered grid, nearest to the centre first.
            candidates = sorted(((u, v) for u in us for v in vs), key=lambda uv: math.hypot(uv[0] - uc, uv[1] - vc))
            checked = 0
            for u, v in candidates:
                c = poses_at(u, v)[0]
                info = self.reach.lookup("place", float(c[0]), float(c[1]))
                if info is not None and info["status"] != 0:
                    continue
                checked += 1
                if checked > DROP_MAX_IK_CHECKS:
                    break
                if reachable(u, v):
                    return result(u, v, "nearest reachable point on the spot, from the reach map")
        # Bisection from the centre toward the marker-area corner nearest the
        # shoulder (reach grows monotonically that way).
        corners = [(u, v) for u in (us[0], us[-1]) for v in (vs[0], vs[-1])]
        anchor = min(corners, key=lambda uv: float(np.linalg.norm(poses_at(*uv)[0] - shoulder)))
        if not reachable(*anchor):
            centre_info = self.reach.lookup("place", float(centre[0]), float(centre[1])) if self.reach.state == "ready" else None
            why = f"reach map at the centre: {centre_info['text']}" if centre_info else \
                "even its corner nearest the robot is outside the palm's reach"
            return None, f"{pattern.name} (centre {centre[0]:+.3f}, {centre[1]:+.3f} m): no reachable drop point ({why})"
        lo, hi = 0.0, 1.0   # fraction from the anchor (reachable) toward the centre (not)
        for _ in range(5):
            mid = (lo + hi) / 2.0
            if reachable(anchor[0] + (uc - anchor[0]) * mid, anchor[1] + (vc - anchor[1]) * mid):
                lo = mid
            else:
                hi = mid
        return result(anchor[0] + (uc - anchor[0]) * lo, anchor[1] + (vc - anchor[1]) * lo,
                      "nearest reachable point toward the robot")

    def _validate(self, target: Target, obj: ObjectConfig) -> PhaseResult:
        p, t, s = self.p, self.cfg.tuning, self.cfg.safety
        if self.stop_event.is_set():
            return PhaseResult.fail(Failure.OPERATOR_STOP, "stop requested")
        table = p.current_table()
        if table is None:
            return PhaseResult.fail(Failure.SENSOR_UNAVAILABLE, "no table plane; cannot validate table clearance")
        height = float(target.p_base[2] - table.z_base_m)
        if not 0.0 <= height <= 0.40:
            return PhaseResult.fail(Failure.NO_TARGET, f"object position implausible ({height:.3f} m above the table)")
        with p.lock:
            cam = dict(p.camera_extrinsic)
        T_object = base_from_camera(cam) @ target.det.T_camera_object
        T_object[:3, 3] = target.p_base
        T_grasp = p.grasp_pose(target.det, T_object, table)
        reach = float(np.linalg.norm(T_grasp[:3, 3] - SHOULDER_IN_BASE[SORT_ARM]))
        if reach > self.cfg.hardware.arm_reach_m:
            return PhaseResult.fail(Failure.UNREACHABLE, f"{obj.name} grasp is {reach:.3f} m from the shoulder "
                                    f"(max {self.cfg.hardware.arm_reach_m:.2f} m)")
        T_grasp, reach_note = self._fit_grasp_to_reach(T_grasp, target, table)
        if T_grasp is None:
            return PhaseResult.fail(Failure.UNREACHABLE, reach_note)
        if reach_note:
            self.sm.info(reach_note)
        objects = p.obstacle_objects(T_object, time.time())
        own = [o for o in (p.table_objects or []) if float(np.linalg.norm(o.center_xy - target.p_base[:2])) < o.radius_m + 0.04]
        radius = own[0].radius_m if own else 0.04
        # The default grasp points the fingers from the shoulder at the object;
        # if the arm cannot do that with a level palm, try finger directions
        # rotated about the object (it stays between the fingers either way).
        t_plan = time.time()
        first_failure: Optional[Tuple[Failure, str]] = None
        planned = None
        preferred = self._ik_preferred_yaw(T_grasp, target.p_base)
        candidates = [preferred] if preferred is not None else []
        candidates += [d for d in GRASP_YAW_CANDIDATES_DEG if d not in candidates]
        self.sm.info(f"planning the grasp of {obj.name}: object ({target.p_base[0]:+.3f}, {target.p_base[1]:+.3f}, "
                     f"{target.p_base[2]:+.3f}) m, {len(objects)} obstacle(s), finger directions to try "
                     f"{', '.join(f'{d:+d}' for d in candidates)} deg"
                     + ("" if preferred is None else f" ({preferred:+d} = what the arm reaches with a level palm)"))
        tried: List[int] = []
        for yaw_deg in candidates:
            if self.stop_event.is_set():
                return PhaseResult.fail(Failure.OPERATOR_STOP, "stop requested")
            tried.append(yaw_deg)
            T_candidate = rotate_about_vertical(T_grasp, target.p_base, math.radians(yaw_deg))
            short = p.reach_residual_m(T_candidate[:3, 3], p.controller.reference())
            if short > 0.005:
                why = f"palm position {self._pose_text(T_candidate)} is {short * 1000:.0f} mm out of reach"
                self.sm.warn(f"finger direction {yaw_deg:+d} deg skipped: {why}")
                first_failure = first_failure or (Failure.UNREACHABLE, why)
                continue
            failure, why, planned = self._plan_route(T_candidate, target, obj, table, objects, radius)
            if failure is None:
                break
            self.sm.warn(f"finger direction {yaw_deg:+d} deg rejected: {why} [{failure.value}]")
            first_failure = first_failure or (failure, why)
            planned = None
            if why.startswith(DROP_OUT_OF_REACH) or "carried object bottom" in why:
                first_failure = (failure, why)
                break  # does not depend on the finger direction
        if planned is None:
            failure, why = first_failure
            finish = failure in (Failure.UNREACHABLE,) and ("drop-off" in why or why.startswith(DROP_OUT_OF_REACH))
            return PhaseResult.fail(failure, f"{why} (finger directions tried: {', '.join(f'{d:+d}' for d in tried)} deg)",
                                    finish_object=finish)
        T_grasp, T_pre, T_clear, carried, drop, drop_detail = planned
        plan = GraspPlan(target, T_object, T_grasp, T_pre, T_clear, objects, table, carried.radius_m,
                         {"drop": drop, "drop_detail": drop_detail, "target_obj": self._planned_target_obj})
        self._log_trajectory(obj, yaw_deg, [("start", p.reference_hand_T()), ("clearance", T_clear),
                                             ("pre-grasp", T_pre), ("grasp", T_grasp)], time.time() - t_plan)
        p.set_grab_plan(SORT_ARM, T_object, T_grasp, [("clearance", T_clear), ("pre-grasp", T_pre),
                                                     ("approach", T_grasp)], objects, "")
        with self.lock:
            self.current_target = (target.obj_index, target.det.box)
            self.active_object = target.obj_index
        self.sm.info(f"drop-off: {drop_detail}; release with the hand {drop[2] * 100:.0f} cm above the spot")
        return PhaseResult.success("validated", plan=plan)

    def _fit_grasp_to_reach(self, T_grasp: np.ndarray, target: Target, table: Any
                            ) -> Tuple[Optional[np.ndarray], str]:
        """Keep the default grasp if the palm can reach it; otherwise let the
        object sit further toward the fingertips (up to max_grasp_standoff_m)
        and grip higher (up to min_grip_below_top_m under the top).  Returns
        (grasp pose or None, note)."""
        p, h = self.p, self.cfg.hand
        q_seed = p.controller.reference()
        short = p.reach_residual_m(T_grasp[:3, 3], q_seed)
        if short <= 0.005:
            return T_grasp, ""
        fwd = T_grasp[:3, 0].copy()
        fwd[2] = 0.0
        fwd /= max(1e-9, float(np.linalg.norm(fwd)))
        standoff = float(np.dot(target.p_base[:2] - T_grasp[:2, 3], fwd[:2]))
        top = float(target.p_base[2] + (target.det.top_offset_m or 0.0))
        z_max = max(float(T_grasp[2, 3]), top - h.min_grip_below_top_m)
        best = (short, None, standoff, float(T_grasp[2, 3]))
        for extra in np.arange(0.0, max(0.0, h.max_grasp_standoff_m - standoff) + 1e-9, 0.01):
            for z in np.unique(np.clip([T_grasp[2, 3], (T_grasp[2, 3] + z_max) / 2.0, z_max], T_grasp[2, 3], z_max)):
                T = T_grasp.copy()
                T[:3, 3] = T_grasp[:3, 3] - fwd * extra
                T[2, 3] = z
                miss = p.reach_residual_m(T[:3, 3], q_seed)
                if miss <= 0.005:
                    return T, (f"default grasp is {short * 1000:.0f} mm out of reach; object held "
                               f"{(standoff + extra) * 100:.0f} cm ahead of the palm (default {standoff * 100:.0f}) "
                               f"and gripped at z={z:.3f} m")
                if miss < best[0]:
                    best = (miss, T, standoff + extra, z)
        gap = best[0]
        self.reach.ensure(table.z_base_m)
        return None, (f"[{self.reach.explain('pick', target.p_base[0], target.p_base[1])}] "
                      f"{target.det.label} at ({target.p_base[0]:+.3f}, {target.p_base[1]:+.3f}) m is out of the right "
                      f"arm's reach for a palm grasp: even with the object {best[2] * 100:.0f} cm ahead of the palm and "
                      f"the grip at z={best[3]:.3f} m the palm stays {gap * 1000:.0f} mm short; move it about "
                      f"{gap * 100 + 1:.0f} cm closer to the robot")

    def _ik_preferred_yaw(self, T_grasp: np.ndarray, p_object: np.ndarray) -> Optional[int]:
        """Finger direction (deg from the default) the arm reaches with a level
        palm at this grasp: solve with yaw free, turn the grasp about the
        object to that yaw, repeat until they agree.  None if no level pose."""
        p = self.p
        total = 0.0
        q_seed = p.controller.reference()
        for _ in range(4):
            T = rotate_about_vertical(T_grasp, p_object, total)
            q = p.solve_level(T, q_seed)
            if q is None:
                return None
            x_ik, x_plan = p.fk.compute_arm(q)[:2, 0], T[:2, 0]
            delta = math.atan2(float(x_plan[0] * x_ik[1] - x_plan[1] * x_ik[0]), float(np.dot(x_plan, x_ik)))
            total += delta
            q_seed = q
            if abs(delta) < math.radians(2.0):
                break
        if abs(total) > math.radians(45.0):
            return None  # the fingers would no longer face the object from the robot side
        return int(round(math.degrees(total)))

    @staticmethod
    def _pose_text(T: np.ndarray) -> str:
        yaw = math.degrees(math.atan2(float(T[1, 0]), float(T[0, 0])))
        tilt = math.degrees(math.acos(float(np.clip(T[2, 2], -1.0, 1.0))))
        return f"({T[0, 3]:+.3f}, {T[1, 3]:+.3f}, {T[2, 3]:+.3f}) m, fingers {yaw:+.0f} deg, palm tilt {tilt:.0f} deg"

    def _log_trajectory(self, obj: ObjectConfig, yaw_deg: int, waypoints: List[Tuple[str, np.ndarray]],
                        planning_s: float) -> None:
        """One line per 6D EE waypoint plus the totals the movers will follow."""
        t = self.cfg.tuning
        speeds = {"clearance": t.transport_speed_m_s, "pre-grasp": t.approach_speed_m_s, "grasp": t.approach_speed_m_s}
        total_m = total_s = 0.0
        poses = 0
        lines = []
        for (name0, T0), (name1, T1) in zip(waypoints, waypoints[1:]):
            dist = float(np.linalg.norm(T1[:3, 3] - T0[:3, 3]))
            n = max(1, int(math.ceil(dist / (speeds[name1] / max(2.0, t.planner_rate_hz)))))
            total_m, total_s, poses = total_m + dist, total_s + dist / speeds[name1], poses + n
            lines.append(f"{name0} -> {name1}: {dist * 1000:.0f} mm, {n} EE poses")
        self.sm.info(f"EE trajectory for {obj.name} (fingers turned {yaw_deg:+d} deg about the object; planned in "
                     f"{planning_s:.1f} s): {poses} EE poses, {total_m * 1000:.0f} mm, ~{total_s:.0f} s; "
                     + "; ".join(lines))
        for name, T in waypoints:
            self.sm.info(f"  waypoint {name}: {self._pose_text(T)}")

    def _check_key_poses(self, poses: List[Tuple[str, np.ndarray, MotionContext]], q_seed: np.ndarray,
                         T_from: Optional[np.ndarray] = None) -> Tuple[Optional[Failure], str, np.ndarray]:
        """IK + the full motion validator (workspace, joint limits, torso,
        table/hand geometry, obstacles, carried object) on each key 6D pose.
        Each key pose is reached through intermediate 6D poses along the
        straight path (every 2 cm / 5 deg) so the IK follows the motion as it
        will at run time."""
        p = self.p
        q = q_seed
        T_prev = p.fk.compute_arm(q_seed) if T_from is None else T_from
        for label, T, ctx in poses:
            angle = math.degrees(math.acos(float(np.clip((np.trace(T_prev[:3, :3].T @ T[:3, :3]) - 1.0) / 2.0,
                                                         -1.0, 1.0))))
            n = max(1, int(math.ceil(max(float(np.linalg.norm(T[:3, 3] - T_prev[:3, 3])) / 0.02, angle / 5.0))))
            for k in range(1, n):
                q_mid = p.solve_pose(interpolate_pose(T_prev, T, k / n), q)
                if q_mid is None:
                    T_mid = interpolate_pose(T_prev, T, k / n)
                    return (Failure.IK_FAILURE, f"on the way to the {label} pose, at {self._pose_text(T_mid)}: "
                            f"{p.last_ik_detail}", q)
                q = q_mid
            q_next = p.solve_pose(T, q)
            if q_next is None:
                return Failure.IK_FAILURE, f"{label} pose {self._pose_text(T)}: {p.last_ik_detail}", q
            reasons = p.validator.check_q(q_next, ctx, T)
            if reasons:
                return Failure.COLLISION_REJECTED, f"{label} pose {self._pose_text(T)}: {reasons[0]}", q
            q, T_prev = q_next, T
        return None, "", q

    def _check_path_geometry(self, segments: List[Tuple[np.ndarray, np.ndarray, MotionContext]]
                             ) -> Tuple[Optional[Failure], str]:
        """The straight EE paths between key poses, every 1 cm / 3 deg, against
        the hand (and carried object) model: geometry only, no IK, plus the torso
        keep-out for the wrist (a straight path can cut in front of the body)."""
        for T0, T1, ctx in segments:
            angle = math.degrees(math.acos(float(np.clip((np.trace(T0[:3, :3].T @ T1[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))))
            n = max(1, int(math.ceil(max(float(np.linalg.norm(T1[:3, 3] - T0[:3, 3])) / 0.01, angle / 3.0))))
            for k in range(1, n + 1):
                T = interpolate_pose(T0, T1, k / n)
                reasons = self.p.validator.pose_violations(T, None, ctx)
                if not reasons and float(np.linalg.norm(T[:2, 3])) < self.cfg.safety.torso_keep_out_radius_m:
                    reasons = [f"wrist within {self.cfg.safety.torso_keep_out_radius_m:.2f} m of the torso axis"]
                if reasons:
                    return Failure.COLLISION_REJECTED, f"{ctx.label} path at {k}/{n} {self._pose_text(T)}: {reasons[0]}"
        return None, ""

    def _target_cylinder(self, target: Target, table: Any, radius: float) -> gp1.TableObject:
        """The target as a cylinder (depth segmentation when available)."""
        own = [o for o in (self.p.table_objects or [])
               if float(np.linalg.norm(o.center_xy - target.p_base[:2])) < o.radius_m + 0.04]
        top = float(target.p_base[2] + (target.det.top_offset_m or 0.05))
        if own:
            top = max(top, own[0].top_z_m)
        return gp1.TableObject(box=target.det.box or (0, 0, 1, 1), center_xy=target.p_base[:2].copy(),
                               radius_m=radius, top_z_m=top, table_z_m=table.z_base_m)

    def _pregrasp_pose(self, T_grasp: np.ndarray, p_object: np.ndarray, radius: float) -> Tuple[np.ndarray, str]:
        """Beside the object, away from the palm: the hand comes down there
        (the open fingers never pass over the object) and then moves sideways
        along the palm normal until the palm is at the contact spot.  The palm
        faces hand +y on the right hand (the object approaches from the
        robot's right), -y on the left."""
        h, t = self.cfg.hand, self.cfg.tuning
        palm = T_grasp[:3, 1].copy() * (1.0 if SORT_ARM == "right" else -1.0)
        palm[2] = 0.0
        palm /= max(1e-9, float(np.linalg.norm(palm)))
        # Object centre beyond the wrist axis on the palm side at the grasp.
        lateral = float(np.dot(p_object[:2] - T_grasp[:2, 3], palm[:2]))
        side = max(t.pregrasp_backoff_m, radius + h.collision_radius_m + t.side_approach_margin_m - lateral)
        T = T_grasp.copy()
        T[:3, 3] -= palm * side
        return T, f"side approach from {side * 100:.0f} cm beside the grasp"

    def _plan_route(self, T_grasp: np.ndarray, target: Target, obj: ObjectConfig, table: Any,
                    objects: List[gp1.TableObject], radius: float):
        """Plan one grasp: the 6D EE waypoints clearance -> pre-grasp -> grasp
        and the lift/transport/release poses with the planned carried object.
        Key poses get IK and the full validator; the straight paths between
        them get the geometry check.  Every pose of the actual motion is IK-
        solved and validated again while moving.  Returns (failure or None,
        reason, (T_grasp, T_pre, T_clear, carried, drop, drop_detail))."""
        p, t, s = self.p, self.cfg.tuning, self.cfg.safety
        target_obj = self._target_cylinder(target, table, radius)
        T_pre, pre_note = self._pregrasp_pose(T_grasp, target.p_base, radius)
        tallest = max([o.top_z_m for o in objects + [target_obj]], default=table.z_base_m)
        T_clear = T_pre.copy()
        T_clear[2, 3] = max(float(T_pre[2, 3]), table.z_base_m + s.table_margin_m + 0.02,
                            tallest + t.hover_clearance_m + self.cfg.hand.collision_radius_m)
        # Until the final approach the fingers must stay off the target too.
        general = MotionContext(table, objects, keep_clear=[target_obj], label="clearance")
        pre_ctx = MotionContext(table, objects, hand_mode="grasp", keep_clear=[target_obj], label="pre-grasp")
        grasp_ctx = MotionContext(table, objects, hand_mode="grasp", label="approach")
        failure, why, q_grasp = self._check_key_poses(
            [("clearance", T_clear, general), ("pre-grasp", T_pre, pre_ctx), ("grasp", T_grasp, grasp_ctx)],
            p.controller.reference())
        if failure is None:
            T_start = p.reference_hand_T()
            failure, why = self._check_path_geometry([(T_start, T_clear, general), (T_clear, T_pre, pre_ctx),
                                                      (T_pre, T_grasp, grasp_ctx)])
        if failure is not None:
            return failure, why + (f" ({pre_note})" if pre_note else ""), None
        self._planned_target_obj = target_obj
        hang = float(T_grasp[2, 3] - table.z_base_m)
        offset = T_grasp[:3, :3].T @ (target.p_base - T_grasp[:3, 3])
        carried = CarriedObject(offset, hang, radius)
        T_lift = T_grasp.copy()
        T_lift[2, 3] += t.lift_m
        drop, drop_detail = self._drop_pose_for(obj, T_grasp[:3, :3], carried)
        if drop is None:
            return Failure.UNREACHABLE, f"{DROP_OUT_OF_REACH}: {drop_detail}", None
        transport = self._transport_route(T_lift, drop[1], carried, table, objects)
        # Position-only reach along the drop-off route (any palm orientation):
        # if the palm cannot get there at all, no grasp variant will help.
        q_seed = p.controller.reference()
        for T0, T1, ctx, _y in transport:
            n = max(1, int(math.ceil(float(np.linalg.norm(T1[:3, 3] - T0[:3, 3])) / 0.05)))
            for k in range(1, n + 1):
                point = T0[:3, 3] + (T1[:3, 3] - T0[:3, 3]) * (k / n)
                short = p.reach_residual_m(point, q_seed)
                if short > 0.005:
                    side = "left" if point[1] > SHOULDER_IN_BASE[SORT_ARM][1] + 0.10 else "right"
                    self.reach.ensure(table.z_base_m)
                    spot = drop[1][:3, 3]
                    return (Failure.UNREACHABLE,
                            f"{DROP_OUT_OF_REACH}: {drop_detail}: the {ctx.label} path passes "
                            f"({point[0]:+.3f}, {point[1]:+.3f}, {point[2]:+.3f}) m, which the {SORT_ARM} palm cannot "
                            f"reach in any orientation ({short * 1000:.0f} mm short"
                            + (", across the body midline" if side == "left" and SORT_ARM == "right" else "")
                            + f"); at the spot itself: {self.reach.explain('place', spot[0], spot[1])}"
                            + "; move the drop-off spot toward the robot's right / closer", None)
        lift_ctx = MotionContext(table, objects, min_wrist_z_m=T_grasp[2, 3] - 0.015, label="lift")
        failure, why, _q = self._check_key_poses(
            [("lift", T_lift, lift_ctx)] + [(f"{ctx.label} end", T1, ctx) for _T0, T1, ctx, _y in transport], q_grasp,
            T_from=T_grasp)
        if failure is None:
            failure, why = self._check_path_geometry([(T_grasp, T_lift, lift_ctx)]
                                                     + [(T0, T1, ctx) for T0, T1, ctx, _y in transport])
        if failure is not None:
            return (Failure.UNREACHABLE if failure == Failure.IK_FAILURE else failure,
                    f"drop-off not feasible while carrying {obj.name}: {why}", None)
        return None, "", (T_grasp, T_pre, T_clear, carried, drop, drop_detail)

    # -- phases 2-7 ----------------------------------------------------------------
    def _pick_and_place(self, plan: GraspPlan, obj: ObjectConfig) -> Tuple[State, PhaseResult]:
        p, t = self.p, self.cfg.tuning
        cancel = self.stop_event
        table, objects = plan.table, plan.objects
        p_object = plan.T_object[:3, 3]
        self.pick_record = {"plan": plan}

        def fail(result: MoveResult) -> Tuple[State, PhaseResult]:
            failure = result.failure or Failure.INTERNAL
            if self.sm.state in (S.MOVING_TO_DROPOFF, S.LOWERING) and failure in (
                    Failure.COLLISION_REJECTED, Failure.IK_FAILURE, Failure.NOT_REACHED):
                failure = Failure.PLACEMENT_FAILURE   # the drop-off route / spot is obstructed
            return self.sm.state, PhaseResult.fail(failure, result.detail, cause=result.failure.value
                                                   if result.failure else "")

        # Phase 2: clearance -> pre-grasp -> final approach.
        keep = [plan.data["target_obj"]] if plan.data.get("target_obj") is not None else []
        if plan.data.get("resume"):
            # The object is already between the open fingers (operator or a
            # previous attempt put it there): no approach, close right away.
            self._enter(S.MOVING_TO_PREGRASP, "skipped: the object is already at the palm")
            self._enter(S.APPROACHING, "skipped: the object is already at the palm")
            r = MoveResult(True, None, "already at the object",
                           float(np.linalg.norm(p.measured_hand_T()[:3, 3] - plan.T_grasp[:3, 3])), 0.0)
        else:
            self._enter(S.MOVING_TO_PREGRASP)
            if t.finger_check_before_grasp:
                # In free space at the preparation pose: every finger must
                # follow (stale motors are recovered here, not at the object).
                ok, detail = p.grip.finger_check(cancel)
                if not ok:
                    return self.sm.state, PhaseResult.fail(Failure.FINGER_FAILURE, f"finger check: {detail}")
            r = p.move_linear(plan.T_clear, MotionContext(table, objects, keep_clear=keep, label="clearance"),
                              t.transport_speed_m_s, "clearance", cancel)
            if not r.ok:
                return fail(r)
            r = p.move_linear(plan.T_pre, MotionContext(table, objects, hand_mode="grasp", keep_clear=keep,
                                                        label="pre-grasp"),
                              t.approach_speed_m_s, "pre-grasp", cancel, max_tilt_rad=2 * t.grasp_tilt_tolerance_rad)
            if not r.ok:
                return fail(r)
            self._enter(S.APPROACHING)

        def gate_reason(T_meas: np.ndarray) -> str:
            """Empty when the hand may close: position, palm level, finger
            direction, and the object inside the fingers' capture box."""
            err = float(np.linalg.norm(T_meas[:3, 3] - plan.T_grasp[:3, 3]))
            tilt = math.acos(float(np.clip(T_meas[2, 2], -1.0, 1.0)))
            yaw = yaw_error(T_meas, plan.T_grasp)
            ok_capture, d = p.in_capture(T_meas, p_object)
            if err > t.grasp_position_tolerance_m:
                return f"EE error {err * 1000:.0f} mm > {t.grasp_position_tolerance_m * 1000:.0f} mm"
            if tilt > t.grasp_tilt_tolerance_rad:
                return f"palm tilt {math.degrees(tilt):.1f} deg"
            if yaw > t.grasp_yaw_tolerance_rad:
                return f"finger direction {math.degrees(yaw):.1f} deg off the plan"
            if not ok_capture:
                return f"object outside the fingers (hand frame {d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f} m)"
            return ""

        def gate(T_meas: np.ndarray) -> bool:
            return not gate_reason(T_meas)

        if not plan.data.get("resume"):
            r = p.move_linear(plan.T_grasp, MotionContext(table, objects, hand_mode="grasp", label="approach"),
                              t.approach_speed_m_s, "approach", cancel, tolerance_m=t.grasp_position_tolerance_m,
                              gate=gate, gate_reason=gate_reason)
            if not r.ok:
                return fail(r)
        T_meas = p.measured_hand_T()
        why = gate_reason(T_meas)
        if why:
            return self.sm.state, PhaseResult.fail(Failure.NOT_REACHED, f"approach ended outside the grasp gate: {why}")
        self.pick_record["T_grasp_measured"] = T_meas
        self.pick_record["q_grasp_ref"] = p.controller.reference()

        # Phase 3: close and verify.
        self._enter(S.CLOSING_HAND, f"EE error {r.error_m * 1000:.0f} mm")
        grip = p.grip.close(cancel)
        if grip.evidence == "jam" or (grip.suspects and grip.evidence != "cancelled"):
            # A finger blocked right at the open pose (no grip formed), or a
            # finger that never moved although the grip holds (seen: the index
            # stayed open on the bottle).  The object still rests on the table:
            # open with the stale-finger recovery and close once more.
            why = grip.detail if grip.evidence == "jam" else f"{', '.join(grip.suspects)} did not close"
            self.sm.warn(f"{why}; opening with finger recovery and closing once more")
            opened, detail = p.grip.open(t.release_open_s)
            if not opened:
                return self.sm.state, PhaseResult.fail(Failure.FINGER_FAILURE, detail)
            grip = p.grip.close(cancel)
            if grip.suspects and grip.verified:
                self.sm.warn(f"{', '.join(grip.suspects)} still did not close; the grip holds without it")
        self.pick_record["grip"] = grip
        self._enter(S.VERIFYING_GRASP, grip.detail)
        if grip.evidence == "cancelled":
            return self.sm.state, PhaseResult.fail(Failure.OPERATOR_STOP, grip.detail)
        if grip.evidence in ("jam", "no_feedback"):
            return self.sm.state, PhaseResult.fail(Failure.FINGER_FAILURE, grip.detail)
        if not grip.verified:
            return self.sm.state, PhaseResult.fail(Failure.GRASP_NOT_CONFIRMED, grip.detail)
        T_hand = p.measured_hand_T()
        hang = float(T_hand[2, 3] - table.z_base_m)
        offset = T_hand[:3, :3].T @ (p_object - T_hand[:3, 3])
        carried = CarriedObject(offset, hang, plan.carried_radius_m)
        with p.lock:
            p.held_object = {"offset_hand": offset, "hang_m": hang}
            p.holding_object = True
            p.holding_arm = SORT_ARM
        self.pick_record["carried"] = carried
        p.diag(grasp_status=f"VERIFIED ({grip.evidence})", grasp_evidence=grip.evidence)

        # Phase 4: vertical lift, closed loop on the measured hand.
        self._enter(S.LIFTING)
        z0 = float(T_hand[2, 3])
        T_lift = T_hand.copy()
        T_lift[:3, :3] = gp1.level_rotation(T_hand[:3, :3])
        T_lift[2, 3] = z0 + t.lift_m
        self.pick_record["T_lift"] = T_lift
        last_check = [0.0]

        def retained_monitor() -> Optional[Tuple[Failure, str]]:
            now = time.time()
            z_now = float(p.measured_hand_T()[2, 3])
            p.diag(lift_target_m=t.lift_m, lift_actual_m=z_now - z0)
            if now - last_check[0] < 0.3 or z_now - z0 < 0.02:
                return None
            last_check[0] = now
            held, why = p.grip.retained()
            return (Failure.OBJECT_LOST, why) if held is False else None

        r = p.move_linear(T_lift, MotionContext(table, objects, min_wrist_z_m=z0 - 0.015, label="lift"),
                          t.lift_speed_m_s, "lift", cancel, tolerance_m=t.lift_tolerance_m,
                          max_tilt_rad=2 * t.grasp_tilt_tolerance_rad, monitor=retained_monitor)
        if not r.ok:
            failure = r.failure if r.failure in (Failure.OBJECT_LOST, Failure.OPERATOR_STOP,
                                                 Failure.COLLISION_REJECTED) else Failure.LIFT_INCOMPLETE
            return self.sm.state, PhaseResult.fail(failure, r.detail)
        self._enter(S.VERIFYING_LIFT)
        lifted = float(p.measured_hand_T()[2, 3]) - z0
        p.diag(lift_target_m=t.lift_m, lift_actual_m=lifted)
        if lifted < t.lift_m - t.lift_tolerance_m:
            return self.sm.state, PhaseResult.fail(Failure.LIFT_INCOMPLETE,
                                                   f"lifted {lifted * 1000:.0f} of {t.lift_m * 1000:.0f} mm")
        held, why = p.grip.retained()
        if held is None:
            return self.sm.state, PhaseResult.fail(Failure.SENSOR_UNAVAILABLE, f"cannot verify the object is held: {why}")
        if not held:
            return self.sm.state, PhaseResult.fail(Failure.OBJECT_LOST, why)
        self.sm.info(f"lift verified: {lifted * 1000:.0f} mm, {why}")

        # Phase 5: transport above the spot at a validated clearance height.
        self._enter(S.MOVING_TO_DROPOFF)
        drop, drop_detail = self._drop_pose_for(obj, p.measured_hand_T()[:3, :3], carried)
        if drop is None:
            return self.sm.state, PhaseResult.fail(Failure.PLACEMENT_FAILURE, drop_detail)
        T_approach, T_release, release_h = drop
        route = self._transport_route(p.reference_hand_T(), T_release, carried, table, objects)
        failure, why, _q = self._check_key_poses([(f"{ctx.label} end", T1, ctx) for _T0, T1, ctx, _y in route],
                                                 p.controller.reference())
        if failure is None:
            failure, why = self._check_path_geometry([(T0, T1, ctx) for T0, T1, ctx, _y in route])
        if failure is not None:
            return self.sm.state, PhaseResult.fail(Failure.PLACEMENT_FAILURE, f"transport route: {why}")
        self.sm.info("transport: " + " -> ".join(f"{ctx.label} {self._pose_text(T1)}" for _T0, T1, ctx, _y in route))
        self.pick_record["z_transport"] = float(route[-1][0][2, 3])
        hold_monitor = self._retention_monitor()
        for T0, T1, ctx, yaw_free in route[:-1]:
            r = p.move_linear(T1, ctx, t.transport_speed_m_s, ctx.label, cancel,
                              max_tilt_rad=2 * t.grasp_tilt_tolerance_rad, monitor=hold_monitor, yaw_free=yaw_free,
                              accept_short_m=t.short_move_accept_m)
            if not r.ok:
                return fail(r)
        # The finger direction may have turned on the way: recompute the release
        # pose from the actual yaw so the object (not the wrist) is centred.
        drop, drop_detail = self._drop_pose_for(obj, p.measured_hand_T()[:3, :3], carried)
        if drop is None:
            return self.sm.state, PhaseResult.fail(Failure.PLACEMENT_FAILURE, drop_detail)
        T_approach, T_release, release_h = drop
        carry_ctx = MotionContext(table, objects, carried=carried, label="lower to release")

        # Phase 6: lower, release, rise clear.
        self._enter(S.LOWERING, f"release with the hand {release_h * 100:.0f} cm above {drop_detail}")
        r = p.move_linear(T_release, carry_ctx, t.approach_speed_m_s, "lower to release", cancel,
                          max_tilt_rad=2 * t.grasp_tilt_tolerance_rad, monitor=hold_monitor,
                          accept_short_m=t.short_move_accept_m)
        if not r.ok:
            return fail(r)
        if t.place_touchdown:
            # Set the object down instead of dropping it: its bottom meets the
            # spot with the wrist at the table + its hang at the grasp.  The
            # hand may use its table clearance band, not more.
            z_contact = table.z_base_m + carried.hang_m
            z_floor = (table.z_base_m + self.cfg.grasp_wrist_floor_m() - self.cfg.safety.grasp_hand_table_clearance_m
                       + 0.003)
            z_goal = max(z_contact - t.touchdown_overshoot_m, z_floor)
            if z_goal < p.reference_hand_T()[2, 3] - 0.005:
                r = p.touch_down(z_goal, MotionContext(table, objects, hand_mode="grasp", carried=carried, placing=True,
                                                       label="set down"),
                                 t.touchdown_speed_m_s, "set down on the spot", cancel)
                if not r.ok:
                    return fail(r)
        self._enter(S.RELEASING_OBJECT)
        opened, detail = p.grip.open(t.release_open_s)
        if not opened:
            # The finger recovery did not open every finger.  If the object
            # nevertheless dropped onto the spot, carry on (the finger check
            # before the next grasp deals with the hand).
            if self._confirm_placement(plan.target.obj_index, obj, 3.0) == "confirmed":
                self.sm.warn(f"fingers did not fully open ({detail}), but {obj.name} is on the spot")
                opened = True
        self.pick_record["released"] = opened
        if not opened:
            return self.sm.state, PhaseResult.fail(Failure.PLACEMENT_FAILURE, f"fingers did not open: {detail}")
        p.clear_hold_flags()
        r = self._slide_away_from_object(plan, table, objects, cancel)
        if r.failure == Failure.OPERATOR_STOP:
            return fail(r)
        T_rel = p.reference_hand_T()
        T_retreat = T_rel.copy()
        T_retreat[2, 3] += t.retreat_m
        # After a set-down the hand is beside the object at about the grasp
        # height: rise clear of its top before going back.
        target_obj = plan.data.get("target_obj")
        if target_obj is not None:
            T_retreat[2, 3] = max(float(T_retreat[2, 3]), float(target_obj.top_z_m) + t.hover_clearance_m)
        r = p.move_linear(T_retreat, MotionContext(table, objects, min_wrist_z_m=T_rel[2, 3] - 0.015, label="retreat"),
                          t.lift_speed_m_s, "retreat above the spot", cancel, yaw_free=True,
                          accept_short_m=t.short_move_accept_m)
        if not r.ok:
            return fail(r)
        self._enter(S.VERIFYING_PLACEMENT)
        placement = self._confirm_placement(plan.target.obj_index, obj, t.placement_confirm_timeout_s)
        self.pick_record["placement"] = placement
        if placement == "confirmed":
            self.confirmed[plan.target.obj_index] += 1
        else:
            self.unconfirmed[plan.target.obj_index] += 1
            self.sm.warn(f"placement of {obj.name} not visually confirmed ({placement}); re-checked after retracting")
        # The object just moved may have been what blocked an earlier approach
        # (e.g. the side approach to its neighbour): retry those positions.
        self.blacklist = [b for b in self.blacklist if b.reason != Failure.COLLISION_REJECTED.value]

        # Phase 7: back to the preparation pose.
        self._enter(S.RETRACTING)
        r = self._return_to_prep(table, objects)
        if not r.ok:
            return self.sm.state, PhaseResult.fail(
                Failure.OPERATOR_STOP if r.failure == Failure.OPERATOR_STOP else Failure.RETRACTION_FAILURE, r.detail)
        if placement != "confirmed":
            again = self._confirm_placement(plan.target.obj_index, obj, t.placement_confirm_timeout_s)
            if again == "confirmed":
                self.unconfirmed[plan.target.obj_index] -= 1
                self.confirmed[plan.target.obj_index] += 1
                self.sm.info(f"placement of {obj.name} confirmed from the preparation pose")
        with self.lock:
            self.sorted_counts = [c + u for c, u in zip(self.confirmed, self.unconfirmed)]
            self.current_target = None
            self.active_object = None
        self._enter(S.READY_FOR_NEXT_OBJECT, f"{obj.name}: placement {placement}")
        return self.sm.state, PhaseResult.success(placement)

    def _transport_route(self, T_from: np.ndarray, T_release: np.ndarray, carried: CarriedObject, table: Any,
                         objects: List[gp1.TableObject]) -> List[Tuple[np.ndarray, np.ndarray, MotionContext, bool]]:
        """Up to a clearance height, across, down to the release pose.  Only
        obstacles near the horizontal path raise the clearance height."""
        t, s = self.cfg.tuning, self.cfg.safety
        a, b = T_from[:2, 3], T_release[:2, 3]
        seg = b - a
        near = []
        for o in objects:
            u = float(np.clip(np.dot(o.center_xy - a, seg) / max(1e-9, float(np.dot(seg, seg))), 0.0, 1.0))
            # Same footprint overlap test the validator applies to the carried
            # object, plus margin; the hand spheres are validated separately.
            if float(np.linalg.norm(o.center_xy - (a + u * seg))) < o.radius_m + carried.radius_m + 0.04:
                near.append(o)
        z_clear = max(float(T_from[2, 3]), float(T_release[2, 3]) + 0.03,
                      (table.z_base_m if table is not None else 0.0) + s.table_margin_m,
                      max([o.top_z_m for o in near], default=-1.0) + carried.hang_m + s.carried_object_clearance_m
                      + t.transport_extra_clearance_m)
        T_up = T_from.copy()
        T_up[:3, :3] = gp1.level_rotation(T_from[:3, :3])
        T_up[2, 3] = z_clear
        T_over = T_release.copy()
        T_over[2, 3] = z_clear
        ctx = MotionContext(table, objects, carried=carried, label="transport")
        # Carried moves keep the grasp yaw: the object sits ahead of the wrist,
        # so turning the hand would swing it sideways off the validated path.
        route = []
        if z_clear > T_from[2, 3] + 0.005:
            route.append((T_from, T_up, ctx, False))
        route.append((T_up, T_over, ctx, False))
        route.append((T_over, T_release, MotionContext(table, objects, carried=carried, label="lower to release"), False))
        return route

    def _slide_away_from_object(self, plan: Optional[GraspPlan], table: Any, objects: List[gp1.TableObject],
                                cancel: threading.Event) -> MoveResult:
        """Open hand next to a just-released object: move the palm away from
        it horizontally (back along the palm normal, the way the side
        approach came in) before any lift, so the open fingers do not catch
        a lid or rim.  Refused or short: warn and leave it to the lift."""
        p, t = self.p, self.cfg.tuning
        distance = t.release_slide_m
        if plan is not None and plan.T_pre is not None:
            distance = max(distance, float(np.linalg.norm(plan.T_grasp[:2, 3] - plan.T_pre[:2, 3])))
        if t.release_slide_m <= 0.0 or distance <= 0.0:
            return MoveResult(True, None, "slide disabled")
        T_now = p.reference_hand_T()
        palm = T_now[:3, 1].copy() * (1.0 if SORT_ARM == "right" else -1.0)
        palm[2] = 0.0
        palm /= max(1e-9, float(np.linalg.norm(palm)))
        # Slide only as far as the arm keeps the finger direction (IK every
        # 1 cm, seeded along the way): far out to the side it cannot.
        q = p.controller.reference()
        reach = 0.0
        for k in range(1, int(math.ceil(distance / 0.01)) + 1):
            d = min(distance, k * 0.01)
            T_k = T_now.copy()
            T_k[:3, 3] -= palm * d
            q_k = p.solve_pose(T_k, q)
            if q_k is None:
                break
            q, reach = q_k, d
        if reach < 0.02:
            self.sm.warn("cannot move the hand away from the object sideways here; lifting straight up")
            return MoveResult(True, None, "slide not reachable")
        T_away = T_now.copy()
        T_away[:3, 3] -= palm * reach
        # At the set-down height the hand may sit in its table clearance band.
        ctx = MotionContext(table, objects, hand_mode="grasp", placing=True, label="move away from the object")
        r = p.move_linear(T_away, ctx, t.approach_speed_m_s, "move away from the object", cancel,
                          accept_short_m=t.short_move_accept_m)
        if not r.ok and r.failure != Failure.OPERATOR_STOP:
            self.sm.warn(f"could not move away from the object first ({r.detail}); lifting straight up")
        return r

    def _retention_monitor(self):
        last = [0.0]
        warned = [False]

        def monitor() -> Optional[Tuple[Failure, str]]:
            now = time.time()
            if now - last[0] < 0.3:
                return None
            last[0] = now
            held, why = self.p.grip.retained()
            if held is False:
                return Failure.OBJECT_LOST, why
            # Carrying continues on a hot motor (the object has to be put
            # down first); the finger check before the next grasp cools it.
            faults, _w = self.p.grip.motor_health(self.p.grip.snapshot())
            if faults and not warned[0]:
                warned[0] = True
                self.sm.warn(f"Dex3 motors hot while carrying: {', '.join(faults)}")
            return None

        return monitor

    def _confirm_placement(self, obj_index: int, obj: ObjectConfig, timeout_s: float) -> str:
        """'confirmed' when a fresh detection of the object lies on its own
        pattern; 'off_spot' when it is seen elsewhere; 'not_seen' otherwise."""
        deadline = time.time() + timeout_s
        start = time.time()
        seen_elsewhere = False
        while time.time() < deadline:
            snap = self._scene_snapshot()
            if snap["detection_ts"] <= start:
                time.sleep(0.1)
                continue
            for e in self._classify(snap):
                if e["obj"] != obj_index:
                    continue
                if e["zone"] == obj.zone:
                    return "confirmed"
                seen_elsewhere = True
            start = snap["detection_ts"]
            time.sleep(0.1)
        return "off_spot" if seen_elsewhere else "not_seen"

    def _return_to_prep(self, table: Any, objects: List[gp1.TableObject]) -> MoveResult:
        """Rise vertically clear, then go back to the recorded preparation joints."""
        p, t = self.p, self.cfg.tuning
        if self.prep is None or self._hanging(table):
            # From the hanging arm (also after a previous run released it):
            # the same start-up chain every time, so the park sequence can
            # play it back.
            result = self._move_to_recipe_prep(table)
            if not result.ok:
                return result
            if self.prep is None:
                self._record_prep()
                return result
            return p.move_joint(self.prep["q"], MotionContext(table, objects, label="preparation pose"),
                                "preparation pose", self.stop_event)
        away = self._move_away_from_body(table)
        if not away.ok:
            return away
        T_now = p.reference_hand_T()
        # Rise only to a safe clearance height: rising to the preparation height
        # while still far forward is where the arm sags most (seen: "retract up
        # ... sag compensation saturated"); the joint move then raises it.
        z_clear = max(float(T_now[2, 3]),
                      (table.z_base_m if table is not None else 0.0) + self.cfg.safety.table_margin_m + 0.05)
        if z_clear > T_now[2, 3] + 0.005:
            T_up = T_now.copy()
            T_up[2, 3] = z_clear
            r = p.move_linear(T_up, MotionContext(table, objects, min_wrist_z_m=T_now[2, 3] - 0.015, label="retract up"),
                              t.retract_speed_m_s, "retract up", self.stop_event, level=False,
                              accept_short_m=t.short_move_accept_m)
            if not r.ok:
                return r
        r = p.move_joint(self.prep["q"], MotionContext(table, objects, label="preparation pose"), "preparation pose",
                         self.stop_event)
        if not r.ok and r.failure == Failure.COLLISION_REJECTED:
            # Joint interpolation from far forward can dip; go above the
            # preparation wrist position first.
            T_mid = p.reference_hand_T()
            T_mid[:3, 3] = [self.prep["T"][0, 3], self.prep["T"][1, 3], z_clear]
            r2 = p.move_linear(T_mid, MotionContext(table, objects, label="retract over"), t.retract_speed_m_s,
                               "retract over the preparation pose", self.stop_event, level=False)
            if not r2.ok:
                return r2
            r = p.move_joint(self.prep["q"], MotionContext(table, objects, label="preparation pose"), "preparation pose",
                             self.stop_event)
        return r

    # -- recovery -------------------------------------------------------------------
    def _hold_error(self, reason: str) -> None:
        """Controlled hold: keep the controller (and any grip) as they are."""
        try:
            if not self.p.controller.running:
                self.p.controller.start()
        except Exception:
            pass
        self.sm.recovery_reason = reason
        self.sm.enter(S.ERROR, reason)
        self._set_phase(f"ERROR — {reason}. The arm holds its pose; operator intervention required.")

    def _recover(self, failed_state: State, result: PhaseResult) -> bool:
        """Recover according to what the hand may hold.  True: continue with
        the next object; False: the run ends (ABORTED or ERROR)."""
        p, t = self.p, self.cfg.tuning
        self.sm.fail(result)
        if result.failure == Failure.OPERATOR_STOP:
            self.sm.recovery_reason = "operator stop: holding the current pose; the hand is not opened"
            self.sm.enter(S.ABORTED, self.sm.recovery_reason)
            self._set_phase("ABORTED — operator stop. The arm holds its pose"
                            + (" and the grip is kept." if p.grip.holding else "."))
            return False
        if failed_state == S.PREPARING:
            self._hold_error(f"preparation failed: {result.detail}")
            return False
        if failed_state == S.MOVING_TO_PREGRASP and result.failure == Failure.FINGER_FAILURE:
            # The free-space finger check failed even after the recovery: every
            # grasp would fail the same way.  The arm stays at the preparation
            # pose; manual open/close runs the same recovery, then sort again.
            self._hold_error(f"Dex3 finger fault before the grasp: {result.detail}; check the hand, "
                             "use manual open/close, then sort again")
            return False
        self.sm.enter(S.RECOVERING, f"after {failed_state.value}: {result.failure.value if result.failure else ''}")
        plan = self.current
        table = plan.table if plan is not None else p.current_table()
        objects = plan.objects if plan is not None else []
        loaded_states = (S.LIFTING, S.VERIFYING_LIFT, S.MOVING_TO_DROPOFF, S.LOWERING)
        try:
            if failed_state in (S.CLOSING_HAND, S.VERIFYING_GRASP):
                # The hand is at the grasp pose and the object still rests on
                # the table: opening here cannot drop it.
                self.sm.recovery_reason = "releasing at the grasp pose (object supported by the table)"
                if not self._open_or_error("at the grasp pose"):
                    return False
            elif failed_state in loaded_states:
                if result.failure == Failure.OBJECT_LOST and failed_state in (S.MOVING_TO_DROPOFF, S.LOWERING):
                    self.sm.recovery_reason = "object lost in transport; returning over the pick location before opening"
                else:
                    self.sm.recovery_reason = "possibly loaded: returning the object to its pick location"
                if not self._return_object_to_pick(failed_state, table, objects):
                    return False
            elif failed_state == S.RELEASING_OBJECT and not self.pick_record.get("released"):
                self._hold_error(f"fingers did not open over the drop spot: {result.detail}")
                return False
            else:
                self.sm.recovery_reason = "hand empty: returning to the preparation pose"
            p.clear_hold_flags()
            r = self._return_to_prep(table, objects)
            if not r.ok:
                if r.failure == Failure.OPERATOR_STOP:
                    self.sm.enter(S.ABORTED, "operator stop during recovery")
                    return False
                self._hold_error(f"recovery could not return to the preparation pose: {r.detail}")
                return False
        except Exception as exc:
            self._hold_error(f"recovery failed: {type(exc).__name__}: {exc}")
            return False
        self.sm.enter(S.READY_FOR_NEXT_OBJECT, f"recovered ({self.sm.recovery_reason})")
        self.current = None
        return True

    def _open_or_error(self, where: str) -> bool:
        opened, detail = self.p.grip.open(self.cfg.tuning.release_open_s)
        if not opened:
            self._hold_error(f"hand did not open {where}: {detail}")
            return False
        self.p.clear_hold_flags()
        return True

    def _return_object_to_pick(self, failed_state: State, table: Any, objects: List[gp1.TableObject]) -> bool:
        """Carry a possibly held object back above where it was picked, set
        it down at the grasp height, open, rise clear."""
        p, t = self.p, self.cfg.tuning
        rec = self.pick_record
        T_grasp = rec.get("T_grasp_measured")
        if T_grasp is None:
            self._hold_error("no recorded grasp pose to return the object to")
            return False
        carried = rec.get("carried")
        z_transport = rec.get("z_transport", float(T_grasp[2, 3]) + t.lift_m)
        if failed_state in (S.MOVING_TO_DROPOFF, S.LOWERING):
            T_now = p.reference_hand_T()
            T_up = T_now.copy()
            T_up[2, 3] = max(z_transport, float(T_now[2, 3]))
            ctx = MotionContext(table, objects, carried=carried, label="recovery transport")
            # Up from where the hand actually is (a failed move leaves the
            # reference ahead of it), skipped when already at the height.
            T_meas = p.measured_hand_T()
            T_up[2, 3] = max(T_up[2, 3], float(T_meas[2, 3]))
            moves = [("recovery: over the pick location", self._at(T_grasp, T_up[2, 3]))]
            if T_up[2, 3] > T_now[2, 3] + 0.005:
                moves.insert(0, ("recovery: up", T_up))
            for label, T in moves:
                r = p.move_linear(T, ctx, t.transport_speed_m_s, label, self.stop_event,
                                  accept_short_m=t.short_move_accept_m)
                if not r.ok:
                    self._hold_error(f"{label} failed ({r.detail}); the hand may hold the object")
                    return False
        T_down = T_grasp.copy()
        T_down[2, 3] = float(T_grasp[2, 3]) + 0.005
        r = p.move_linear(T_down, MotionContext(table, objects, hand_mode="grasp", label="recovery lower"),
                          t.approach_speed_m_s, "recovery: set the object down", self.stop_event)
        if not r.ok:
            self._hold_error(f"could not set the object down ({r.detail}); the hand may hold it")
            return False
        if not self._open_or_error("after setting the object down"):
            return False
        r = self._slide_away_from_object(self.current, table, objects, self.stop_event)
        if r.failure == Failure.OPERATOR_STOP:
            self.sm.enter(S.ABORTED, "operator stop during recovery")
            return False
        T_rel = p.reference_hand_T()
        T_up = T_rel.copy()
        T_up[2, 3] += t.retreat_m
        p.move_linear(T_up, MotionContext(table, objects, min_wrist_z_m=T_rel[2, 3] - 0.015, label="recovery retreat"),
                      t.lift_speed_m_s, "recovery: rise clear", self.stop_event, yaw_free=True,
                      accept_short_m=t.short_move_accept_m)
        return True

    @staticmethod
    def _at(T: np.ndarray, z: float) -> np.ndarray:
        out = T.copy()
        out[2, 3] = z
        return out

    # -- phase 8 -------------------------------------------------------------------
    def _park_for_release(self, table: Any) -> Tuple[bool, str]:
        """The start-up chain in reverse (recipe backwards, down beside the
        body, shoulder roll back in), so the released arm only settles from
        its own hanging pose: released anywhere higher, the default
        controller swings the arm down through the table edge."""
        p, t = self.p, self.cfg.tuning
        home = self.home
        if home is None:
            return False, "no recorded hanging start pose to return to (the arm was not hanging at the start)"
        for name, q in reversed(gp1.extend_arm_forward_joint_waypoints(SORT_ARM)):
            r = p.move_joint(q, MotionContext(table, label=name), f"park: {name}", self.stop_event)
            if not r.ok:
                return False, r.detail
        r = p.move_joint(home["q_up"], MotionContext(table, label="park clearance"), "park: above the start pose",
                         self.stop_event)
        if not r.ok:
            return False, r.detail
        # Down beside the body: the rise's joint path in reverse (straight and
        # smooth), ending at the swung-out hanging pose.
        z_floor = min(float(p.fk.compute_arm(q)[2, 3]) for q in (home["q_away"], home["q_start"])) - 0.015
        descent = list(reversed(home.get("rise") or []))[1:] + [home["q_away"]]
        r = p.move_joint_path(descent, MotionContext(table, min_wrist_z_m=z_floor, label="park descent"),
                              "park: lower beside the body", self.stop_event, speed_rad_s=t.prep_rise_speed_rad_s)
        if not r.ok:
            return False, r.detail
        q_err = float(np.max(np.abs(p.read_current_arm_q(SORT_ARM) - home["q_away"])))
        if q_err > t.prep_tolerance_rad:
            return False, f"arm {q_err:.2f} rad from the hanging pose after parking"
        # Shoulder roll back in: best effort.  The hanging start pose can sit
        # inside the workspace checker's torso margin; released from the
        # outward hanging pose the arm only settles a few cm.
        r = p.move_joint(home["q_start"], MotionContext(table, min_wrist_z_m=z_floor, label="park: start pose"),
                         "park: start pose", self.stop_event, speed_rad_s=t.prep_abduction_speed_rad_s)
        if not r.ok:
            if r.failure == Failure.OPERATOR_STOP:
                return False, r.detail
            self.sm.info(f"park: staying out from the body ({r.detail})")
        return True, ""

    def _finalize(self, objects: List[ObjectConfig]) -> None:
        p, t = self.p, self.cfg.tuning
        with p.lock:
            holding = p.holding_object
        if holding or p.grip.holding:
            self._hold_error("finalizing with a possibly held object")
            return
        table = p.current_table()
        if self.prep is not None:
            q_err = float(np.max(np.abs(p.read_current_arm_q(SORT_ARM) - self.prep["q_measured"])))
            if q_err > t.prep_tolerance_rad:
                r = self._return_to_prep(table, [])
                if not r.ok:
                    self._hold_error(f"final return to the preparation pose failed: {r.detail}")
                    return
        closed, detail = p.grip.close_neutral()
        if not closed:
            self.sm.warn(f"final hand closure not confirmed: {detail}")
        released = "kept engaged"
        parked, why_not = self._park_for_release(table) if t.release_arms_at_end else (False, "")
        if t.release_arms_at_end and not parked:
            released = f"kept engaged ({why_not})"
            self.sm.warn(f"automatic arm release skipped: {why_not}; release manually from a safe pose")
        elif t.release_arms_at_end:
            p.controller.stop()
            try:
                p.robot.release_arms(duration_s=3.0)
                p._authority_engaged = False
                released = "released"
            except Exception as exc:
                p.controller.start()
                self._hold_error(f"arm release failed: {exc}")
                return
        result = ", ".join(f"{obj.name} {self.confirmed[i]}/{obj.count}"
                           + (f" (+{self.unconfirmed[i]} unconfirmed)" if self.unconfirmed[i] else "")
                           for i, obj in enumerate(objects))
        done = sum(self.confirmed)
        wanted = sum(max(0, obj.count) for obj in objects)
        summary = f"SORT COMPLETE — {done}/{wanted} objects: {result}" + (f" ({'; '.join(self.notes)})" if self.notes else "")
        summary += f"; hand {'closed' if closed else 'closure NOT confirmed'}, arms {released}"
        with self.lock:
            self.summary = summary
            self.sorted_counts = [c + u for c, u in zip(self.confirmed, self.unconfirmed)]
        self.sm.enter(S.COMPLETED, summary)
        self._set_phase(summary)

    def render(self, view: str) -> np.ndarray:
        if view != "reach":
            return super().render(view)
        self.touch()
        p = self.p
        with p.lock:
            rgb = None if p.rgb_bgr is None else p.rgb_bgr.copy()
            cam = dict(p.camera_extrinsic)
        if rgb is None:
            return gp1.placeholder(p.camera_status[:48] or "waiting for camera...")
        table = p.current_table()
        if table is not None:
            self.reach.ensure(table.z_base_m)
        T_camera_base = np.linalg.inv(base_from_camera(cam))

        def project(point: np.ndarray):
            return p.project_camera_point((T_camera_base @ np.append(np.asarray(point, dtype=np.float64), 1.0))[:3])

        marks = []
        snap = self._scene_snapshot()
        for i, pattern in enumerate(snap["patterns"]):
            T = self._pattern_T_camera(i, require_current_epoch=False)
            if T is not None:
                centre = self._pattern_drop_geometry(pattern, T, cam)[0]
                marks.append((pattern.name, "place", float(centre[0]), float(centre[1])))
        for e in self._classify(snap):
            marks.append((e["det"].label, "pick", float(e["p_base"][0]), float(e["p_base"][1])))
        out = self.reach.draw(rgb, project, marks)
        with p.lock:
            objects = list(p.table_objects or [])
        import cv2
        for obj in objects:  # live obstacles (not part of the static map)
            ring = [project(np.array([obj.center_xy[0] + obj.radius_m * math.cos(a), obj.center_xy[1] + obj.radius_m
                                      * math.sin(a), obj.top_z_m])) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
            if all(r is not None for r in ring):
                cv2.polylines(out, [np.array(ring, dtype=np.int32)], True, (0, 140, 255), 1, cv2.LINE_AA)
        return out

    def close(self) -> None:
        """Dashboard shutdown: cancel, then stop publishing (the process exits)."""
        self.stop_event.set()
        super().close()
        self.p.grip.stop_hold()
        self.p.controller.stop()

    # -- status --------------------------------------------------------------------
    def headline(self, diag: Dict[str, Any]) -> str:
        state = self.sm.state
        base = self.sm.snapshot()["headline"]
        if state in (S.APPROACHING, S.MOVING_TO_PREGRASP, S.MOVING_TO_DROPOFF, S.LOWERING, S.RETRACTING) \
                and diag.get("ee_error_m") is not None:
            return f"{base} — EE error: {diag['ee_error_m'] * 1000:.0f} mm"
        if state in (S.LIFTING, S.VERIFYING_LIFT) and diag.get("lift_actual_m") is not None:
            return f"{base} — {diag['lift_actual_m'] * 1000:.0f} / {self.cfg.tuning.lift_m * 1000:.0f} mm"
        if state == S.VERIFYING_GRASP and diag.get("grasp_status"):
            return f"{base} — {diag['grasp_status']}"
        if state == S.RECOVERING:
            return f"{base} — {self.sm.recovery_reason}"
        if state == S.COMPLETED:
            with self.lock:
                return self.summary or base
        return base

    def status(self) -> Dict[str, Any]:
        out = super().status()
        diag = self.p.diagnostics()
        with self.lock:
            objects = list(self.objects)
        out["v2"] = {
            "headline": self.headline(diag),
            "machine": self.sm.snapshot(),
            "diag": diag,
            "counts": [{"name": obj.name, "confirmed": self.confirmed[i], "unconfirmed": self.unconfirmed[i],
                        "count": obj.count, "attempts": self.attempts[i], "finished": self.finished[i]}
                       for i, obj in enumerate(objects)],
            "notes": list(self.notes),
            "prep": None if self.prep is None else [float(v) for v in self.prep["T_measured"][:3, 3]],
            "events": self.sm.log.tail(120),
            "reach": self.reach.status(),
            "config": self.cfg.describe(),
        }
        out["log"] = self.sm.log.lines(150)
        out["phase"] = out["v2"]["headline"]
        return out
