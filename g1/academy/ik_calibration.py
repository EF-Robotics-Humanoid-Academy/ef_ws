#!/usr/bin/env python3
"""Record a teleoperated pick-and-place and use it to calibrate V2 /sort.

Why: the V2 planner can call a spot or an object "unreachable" when the
robot can in fact get there.  The causes are either perception (the head
camera's mounting pose is a fixed default, so objects / spots may be seen
farther than they are) or planner assumptions (how far ahead of the palm the
object sits, how high the palm grips, how level it must stay).  Doing the
task by hand and recording both the arm kinematics and the perception lets
the analysis tell which one is wrong, and by how much.

Usage (run from academy/, with the dashboards closed -- this script owns the
arm through the same controller as the /wbc tab):

    python3 ik_calibration.py record [--iface eth0] [--remote-url http://<pc>:8765/events]
    python3 ik_calibration.py analyze calibration_sessions/<session> [--apply]

record
    The /wbc teleop (wbc_ik.IKPoseController, same mapping): right stick
    moves the hand forward/back and left/right, D-pad up/down moves it up/down
    (left/right rolls it), A toggles the grip, B releases / re-engages the
    arms, Y locks / frees the hand orientation.  Walking is disabled (the base
    must not move during a calibration).  Pick the cup up, lift it, put it on
    its ArUco spot, open the hand, move away; repeat with other positions and
    hand directions -- different finger directions between grasp and release
    are what make the camera offset observable.
    Grip close / open are detected from the finger positions and saved with
    images.  Type in the terminal:  m <note>  (marker),  s  (snapshot),
    q  (finish).  Everything goes to calibration_sessions/<timestamp>/.

analyze
    Prints a report and writes <session>/analysis.json and a proposed
    sort_v2_calibration.json in the session.  With --apply it is copied to
    academy/sort_v2_calibration.json, which g1_dashboard_v2 loads at start.
    Only bounded corrections are proposed: the camera mounting translation,
    HandGeometry.max_grasp_standoff_m and SortTuning.grasp_tilt_tolerance_rad.
    Safety thresholds are never changed.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
SESSIONS_DIR = HERE / "calibration_sessions"
CALIBRATION_FILE = HERE / "sort_v2_calibration.json"
ARM = "right"
# Finger closure (0 open .. 1 closed, mean over index/middle/thumb_1/2) that
# counts as a closed / reopened grip, with hysteresis.
GRIP_CLOSED_AT = 0.35
GRIP_OPEN_AT = 0.20
# Fingers blocked by an object stop well short of fully closed (~0.5-0.65 on a
# cup); a hold whose median closure is above this was an empty closed hand.
EMPTY_HAND_CLOSURE = 0.85
# While the hand is open, a detection this close (horizontally) to the palm
# is taken as the hand occluding / corrupting the object's depth.
HAND_EXCLUSION_M = 0.06
# A grasp/release cycle counts as a carry only if the palm moved this far
# while closed (otherwise the hand closed on nothing and reopened in place).
MIN_CARRY_M = 0.05
# After a release, an object detected this much lower than before the grasp,
# or this far from where the hand released it, fell over / rolled: its
# position is not used for the camera fit.
FELL_DROP_M = 0.012
FELL_SHIFT_M = 0.04
# A spot whose detected centre spreads more than this over the session was
# moved (or badly occluded) at some point.
SPOT_MOVED_M = 0.03
# Limits for proposed corrections (larger ones mean a wrong match, not a calibration).
MAX_CAMERA_CORRECTION_M = 0.08
MAX_PROPOSED_STANDOFF_M = 0.12
MAX_PROPOSED_TILT_RAD = math.radians(12.0)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def rpy_from_R(R: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of grasp_pipeline.make_transform's Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
    yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return roll, pitch, yaw


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def other_arm_controllers_running() -> List[str]:
    """Dashboard processes that could also publish rt/arm_sdk."""
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "g1_dashboard" in cmd and "python" in cmd:
            found.append(f"{pid}: {cmd.strip()[:120]}")
    return found


# ---------------------------------------------------------------------------
# Recorder (also usable with the off-robot simulator)
# ---------------------------------------------------------------------------

class SessionRecorder:
    """Logs arm/hand state at `rate_hz` and perception at `perception_hz`."""

    def __init__(self, robot: Any, pipeline: Any, sorter: Any, out_dir: Path, rate_hz: float = 20.0,
                 perception_hz: float = 2.0, controller_info: Optional[Any] = None) -> None:
        from hand_pose_navigation.arm_fk import SHOULDER_IN_BASE, ArmFK
        from sdk_wrapper import HAND_CLOSED, HAND_JOINT_NAMES, HAND_SAFE_OPEN
        self.robot, self.p, self.sorter = robot, pipeline, sorter
        self.out = Path(out_dir)
        (self.out / "images").mkdir(parents=True, exist_ok=True)
        self.rate_hz, self.perception_hz = rate_hz, perception_hz
        self.controller_info = controller_info
        self.fk = ArmFK(arm=ARM, backend="urdf")
        self.shoulder = np.asarray(SHOULDER_IN_BASE[ARM], dtype=np.float64)
        self.hand_names = list(HAND_JOINT_NAMES)
        self.open_t = np.array(HAND_SAFE_OPEN[ARM])
        self.closed_t = np.array(HAND_CLOSED[ARM])
        self._stop = threading.Event()
        self._files = {"samples": open(self.out / "samples.jsonl", "a"),
                       "perception": open(self.out / "perception.jsonl", "a"),
                       "events": open(self.out / "events.jsonl", "a")}
        self._file_lock = threading.Lock()
        self._grip_closed: Optional[bool] = None   # set from the first sample: no event for the start state
        self.last_sample: Dict[str, Any] = {}
        self.last_perception: Dict[str, Any] = {}
        self.events = 0
        self._threads: List[threading.Thread] = []

    def _write(self, name: str, record: Dict[str, Any]) -> None:
        with self._file_lock:
            self._files[name].write(json.dumps(_jsonable(record)) + "\n")
            self._files[name].flush()

    def write_meta(self, extra: Dict[str, Any]) -> None:
        import grasp_pipeline as gp1
        with self.p.lock:
            cam = dict(self.p.camera_extrinsic)
        meta = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "arm": ARM, "camera_extrinsic": cam,
                "default_camera_extrinsic": dict(gp1.DEFAULT_CAMERA_EXTRINSIC), "shoulder": self.shoulder,
                "intrinsics": asdict(self.p.K) if hasattr(self.p.K, "__dataclass_fields__") else None,
                "config": self.p.cfg.describe() if hasattr(self.p, "cfg") else None,
                "objects": [o.fields() for o in self.sorter.objects],
                "patterns": [p.fields() for p in self.sorter.patterns], **extra}
        (self.out / "meta.json").write_text(json.dumps(_jsonable(meta), indent=2))

    def start(self) -> None:
        for name, target in (("calib-arm", self._arm_loop), ("calib-perception", self._perception_loop)):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        with self._file_lock:
            for f in self._files.values():
                f.close()

    def mark(self, kind: str, note: str = "", images: bool = True) -> None:
        sample = dict(self.last_sample)
        record = {"t": time.time(), "kind": kind, "note": note, "palm": sample.get("palm"),
                  "q_arm": sample.get("q_arm"), "closure": sample.get("closure")}
        if images:
            record["images"] = self.save_images(f"{self.events:03d}_{kind}")
        self._write("events", record)
        self.events += 1

    def save_images(self, stem: str) -> Dict[str, str]:
        import cv2
        with self.p.lock:
            rgb = None if self.p.rgb_bgr is None else self.p.rgb_bgr.copy()
            depth = None if self.p.depth_m is None else self.p.depth_m.copy()
        out = {}
        if rgb is not None:
            path = self.out / "images" / f"{stem}_rgb.jpg"
            cv2.imwrite(str(path), rgb)
            out["rgb"] = path.name
        if depth is not None:
            path = self.out / "images" / f"{stem}_depth_mm.png"
            cv2.imwrite(str(path), np.clip(depth * 1000.0, 0, 65535).astype(np.uint16))
            out["depth"] = path.name
        return out

    def _closure(self, q_hand: np.ndarray) -> float:
        idx = [i for i, n in enumerate(self.hand_names) if n != "thumb_0"]
        span = self.closed_t[idx] - self.open_t[idx]
        return float(np.mean((q_hand[idx] - self.open_t[idx]) / np.where(np.abs(span) > 1e-6, span, 1.0)))

    def _arm_loop(self) -> None:
        dt = 1.0 / self.rate_hz
        while not self._stop.is_set():
            t0 = time.time()
            try:
                q_full = self.p.read_q_full()
                if q_full is not None:
                    from hand_pose_navigation.arm_fk import RIGHT_ARM_JOINTS
                    q_arm = q_full[RIGHT_ARM_JOINTS]
                    T = self.fk.compute_arm(q_arm)
                    rec: Dict[str, Any] = {"t": t0, "q_full": q_full, "q_arm": q_arm, "palm": T,
                                           "palm_shoulder_dist": float(np.linalg.norm(T[:3, 3] - self.shoulder))}
                    snap = self.p.hand_snapshot(ARM, max_age_s=0.5)
                    if snap is not None:
                        q_hand = np.array([float(snap["positions"][n]) for n in self.hand_names])
                        closure = self._closure(q_hand)
                        pressures = snap.get("tactile_pressures") or []
                        rec.update(q_hand=q_hand, closure=closure,
                                   tactile_max=[max([v for v in s if abs(v - 30000.0) > 0.5] or [0.0]) for s in pressures])
                        if self._grip_closed is None:
                            self._grip_closed = closure >= GRIP_CLOSED_AT
                        elif not self._grip_closed and closure >= GRIP_CLOSED_AT:
                            self._grip_closed = True
                            self.last_sample = rec
                            self.mark("grip_closed")
                        elif self._grip_closed and closure <= GRIP_OPEN_AT:
                            self._grip_closed = False
                            self.last_sample = rec
                            self.mark("grip_opened")
                    if self.controller_info is not None:
                        rec["teleop"] = self.controller_info()
                    self.last_sample = rec
                    self._write("samples", rec)
            except Exception as exc:
                self._write("samples", {"t": t0, "error": f"{type(exc).__name__}: {exc}"})
            self._stop.wait(max(0.0, dt - (time.time() - t0)))

    def _perception_loop(self) -> None:
        dt = 1.0 / self.perception_hz
        last_ts = 0.0
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self.sorter.touch()
                snap = self.sorter._scene_snapshot()
                if snap["detection_ts"] > last_ts:
                    last_ts = snap["detection_ts"]
                    cam = snap["cam"]
                    table = self.p.current_table()
                    detections = []
                    for e in self.sorter._classify(snap):
                        det = e["det"]
                        detections.append({"label": det.label, "score": float(det.score), "obj": e["obj"],
                                           "zone": e["zone"], "p_base": e["p_base"],
                                           "p_camera": det.T_camera_object[:3, 3], "top_offset_m": det.top_offset_m,
                                           "box": det.box})
                    patterns = []
                    for i, pattern in enumerate(snap["patterns"]):
                        T = self.sorter._pattern_T_camera(i, require_current_epoch=False)
                        if T is None:
                            continue
                        centre, normal = self.sorter._pattern_drop_geometry(pattern, T, cam)
                        patterns.append({"index": i, "name": pattern.name, "T_camera_pattern": T, "centre": centre,
                                         "normal": normal, "status": self.sorter.pattern_status[i]})
                    vision = getattr(self.p, "vision", None)
                    rec = {"t": t0, "detection_ts": snap["detection_ts"], "cam": cam,
                           "vision_status": self.p.vision_status, "vision_error": getattr(vision, "error", ""),
                           "table_z": None if table is None else table.z_base_m, "detections": detections,
                           "patterns": patterns}
                    self.last_perception = rec
                    self._write("perception", rec)
            except Exception as exc:
                self._write("perception", {"t": t0, "error": f"{type(exc).__name__}: {exc}"})
            self._stop.wait(max(0.0, dt - (time.time() - t0)))


# ---------------------------------------------------------------------------
# record command
# ---------------------------------------------------------------------------

def cmd_record(args: argparse.Namespace) -> int:
    others = other_arm_controllers_running()
    if others and not args.force:
        print("Close the dashboard first (it can also command the arm):\n  " + "\n  ".join(others), file=sys.stderr)
        return 2
    import grasp_pipeline_v2
    import sdk_wrapper
    import sorting_v2
    import sort_v2_config
    import wbc_ik

    robot = sdk_wrapper.G1(iface=args.iface, domain_id=args.domain_id)
    cfg = sort_v2_config.default_config()
    openai_vision = args.vision_backend == "openai"
    pipeline = grasp_pipeline_v2.GraspPipelineV2(
        robot, args.rgbd, cfg=cfg, vision_backend=args.vision_backend,
        vision_model=args.openai_vision_model if openai_vision else args.vision_model)
    sorter = sorting_v2.SorterV2(pipeline)
    pipeline.keep_alive += 1
    sorter.touch()
    out = SESSIONS_DIR / time.strftime("%Y%m%d_%H%M%S")
    controller = wbc_ik.IKPoseController(robot, remote_url=args.remote_url, arm_control=ARM, hand_control=ARM,
                                         params={"max_vx": 0.0, "max_vyaw": 0.0})

    def teleop_info() -> Dict[str, Any]:
        return {"target": controller.target_T[ARM], "grip_percent": controller.hand_grip_percent,
                "orient_locked": controller.orient_stiff, "armed": controller.armed,
                "ik": controller.ik_info.get(ARM), "status": controller.status}

    recorder = SessionRecorder(robot, pipeline, sorter, out, controller_info=teleop_info)
    recorder.write_meta({"remote_url": args.remote_url, "vision_backend": args.vision_backend})
    recorder.start()
    print(f"Recording to {out}\n"
          "Remote: right stick = hand fwd/back + left/right, D-pad up/down = hand up/down, A = grip, "
          "Y = orientation lock, B = release/re-engage arms.  Walking is disabled.\n"
          "Terminal: 'm <note>' marker, 's' snapshot, 'q' finish.\n"
          "Pick up the cup, lift it, place it on its spot, open the hand, move away.  Repeat with the hand "
          "pointing in different directions.")
    done = threading.Event()

    def status_loop() -> None:
        while not done.is_set():
            s, perc = recorder.last_sample, recorder.last_perception
            palm = s.get("palm")
            dets = ", ".join(f"{d['label']}@({d['p_base'][0]:+.2f},{d['p_base'][1]:+.2f})" for d in perc.get("detections", []))
            spots = ", ".join(f"{p['name']}@({p['centre'][0]:+.2f},{p['centre'][1]:+.2f})" for p in perc.get("patterns", []))
            if palm is not None:
                vision_error = getattr(getattr(pipeline, "vision", None), "error", "")
                line = (f" palm ({palm[0][3]:+.3f},{palm[1][3]:+.3f},{palm[2][3]:+.3f}) dist {s['palm_shoulder_dist']:.3f} "
                        f"grip {s.get('closure', float('nan')):.2f} events {recorder.events} | {dets or 'no detections'}"
                        f"{' (vision: ' + vision_error + ')' if vision_error else ''} | {spots}")
                width = shutil.get_terminal_size((120, 20)).columns - 1
                print("\r" + line[:width].ljust(width), end="", flush=True)
            done.wait(1.0)

    threading.Thread(target=status_loop, daemon=True).start()
    try:
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip()
            if cmd in ("q", "q!"):
                # Quitting fades the arm authority out and the default
                # controller swings the arm down: not while the palm is low
                # over the table (it would sweep through it).
                palm = recorder.last_sample.get("palm")
                table_z = recorder.last_perception.get("table_z")
                if cmd == "q" and palm is not None and table_z is not None and \
                        float(palm[2][3]) < float(table_z) + 0.15 and float(palm[0][3]) > 0.15:
                    print(f"\nThe palm is only {float(palm[2][3]) - float(table_z):.2f} m above the table: move the hand "
                          "up / back to the body first (the arm drops when released), or type 'q!' to quit anyway.")
                    continue
                break
            if cmd == "s":
                recorder.mark("snapshot")
            elif cmd.startswith("m"):
                recorder.mark("marker", cmd[1:].strip())
    except KeyboardInterrupt:
        pass
    finally:
        done.set()
        print("\nStopping: fading arm authority out...")
        recorder.stop()
        controller.close()
        sorter.close()
    print(f"Session saved: {out}\nAnalyse it with:  python3 ik_calibration.py analyze {out.relative_to(HERE)}")
    close = getattr(robot, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            pass
    # Skip interpreter teardown: tearing down the DDS library's background
    # threads there aborted the process ("exception not rethrown") after the
    # data was already saved.
    sys.stdout.flush()
    os._exit(0)


# ---------------------------------------------------------------------------
# analyze command
# ---------------------------------------------------------------------------

def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "error" not in row:
            rows.append(row)
    return rows


def _nearest(rows: List[Dict[str, Any]], t: float) -> Optional[Dict[str, Any]]:
    return min(rows, key=lambda r: abs(r["t"] - t)) if rows else None


def _object_track(perception: List[Dict[str, Any]], samples: List[Dict[str, Any]], labels: List[str], t0: float,
                  t1: float, near: Optional[np.ndarray] = None) -> List[np.ndarray]:
    """Detections of any of `labels` in [t0, t1] that the open hand is not
    occluding (the vision labels of one object can flip between frames)."""
    out = []
    for rec in perception:
        if not t0 <= rec["t"] <= t1:
            continue
        s = _nearest(samples, rec["t"])
        palm = None if s is None else np.array(s["palm"])[:3, 3]
        for d in rec["detections"]:
            if d["label"] not in labels:
                continue
            p = np.array(d["p_base"])
            if palm is not None and float(np.linalg.norm(p[:2] - palm[:2])) < HAND_EXCLUSION_M:
                continue
            if near is not None and float(np.linalg.norm(p[:2] - near[:2])) > 0.15:
                continue
            out.append(p)
    return out


def _median_cluster(points: List[np.ndarray], radius: float = 0.04) -> Optional[np.ndarray]:
    if not points:
        return None
    pts = np.array(points)
    med = np.median(pts, axis=0)
    keep = pts[np.linalg.norm(pts[:, :2] - med[:2], axis=1) < radius]
    return np.median(keep, axis=0) if len(keep) else med


def cmd_analyze(args: argparse.Namespace) -> int:
    session = Path(args.session)
    if not session.is_absolute() and not session.exists():
        session = HERE / session
    report, proposal = analyze(session, args.object_offset)
    print(report)
    if proposal is not None and args.apply:
        shutil.copyfile(session / "sort_v2_calibration.json", CALIBRATION_FILE)
        print(f"\nApplied: {CALIBRATION_FILE} (restart g1_dashboard_v2 to load it).")
    elif proposal is not None:
        print(f"\nProposal written to {session / 'sort_v2_calibration.json'}; re-run with --apply to use it.")
    return 0


def analyze(session: Path, object_offset: Optional[str] = None) -> Tuple[str, Optional[Dict[str, Any]]]:
    import grasp_pipeline as gp1
    from hand_pose_navigation.arm_fk import ArmFK, SHOULDER_IN_BASE
    from hand_pose_navigation.reachability_checker import ReachabilityChecker
    from reach_map_v2 import MODEL_MAX_PALM_REACH_M
    from sort_v2_config import default_config

    meta = json.loads((session / "meta.json").read_text())
    samples = _load_jsonl(session / "samples.jsonl")
    perception = _load_jsonl(session / "perception.jsonl")
    events = _load_jsonl(session / "events.jsonl")
    cfg = default_config()
    fk = ArmFK(arm=ARM, backend="urdf")
    checker = ReachabilityChecker(arm=ARM, max_reach_m=gp1.DEFAULT_DIRECT_MAX_REACH_M)
    shoulder = np.asarray(SHOULDER_IN_BASE[ARM], dtype=np.float64)
    labels = [o["prompt"] if o.get("color", "any") == "any" else f"{o['color']} {o['prompt']}" for o in meta["objects"]]
    lines = [f"Calibration session {session.name}: {len(samples)} arm samples, {len(perception)} perception passes, "
             f"{len(events)} events"]
    if not samples or not perception:
        return "\n".join(lines + ["Not enough data."]), None
    table_zs = [r["table_z"] for r in perception if r.get("table_z") is not None]
    table_z = float(np.median(table_zs)) if table_zs else None
    lines.append(f"table plane z (median of {len(table_zs)}): {table_z if table_z is None else round(table_z, 3)} m")

    # Spots that moved during the session (bumped, or occluded during PnP).
    for index in sorted({pt["index"] for r in perception for pt in r["patterns"]}):
        track = [(r["t"], np.array(pt["centre"])) for r in perception for pt in r["patterns"] if pt["index"] == index]
        centres = np.array([c for _t, c in track])
        med = np.median(centres, axis=0)
        far = [t for t, c in track if np.linalg.norm(c[:2] - med[:2]) > SPOT_MOVED_M]
        name = next(pt["name"] for r in perception for pt in r["patterns"] if pt["index"] == index)
        lines.append(f"{name}: median centre ({med[0]:+.3f}, {med[1]:+.3f}) m over {len(track)} detections"
                     + (f"; {len(far)} detection(s) more than {SPOT_MOVED_M * 100:.0f} cm away (moved or occluded) between "
                        f"{time.strftime('%H:%M:%S', time.localtime(min(far)))} and "
                        f"{time.strftime('%H:%M:%S', time.localtime(max(far)))}" if far else ""))

    # Pair grip_closed with the following grip_opened.
    cycles = []
    pending = None
    for e in events:
        if e["kind"] == "grip_closed":
            pending = e
        elif e["kind"] == "grip_opened" and pending is not None:
            cycles.append((pending, e))
            pending = None
    lines.append(f"{len(cycles)} grasp/release cycle(s)")
    vision_errors = sorted({r.get("vision_error") for r in perception if r.get("vision_error")})
    empty_passes = sum(1 for r in perception if not r["detections"])
    if empty_passes:
        lines.append(f"{empty_passes} perception pass(es) without any detection"
                     + (f"; vision errors: {'; '.join(vision_errors)[:300]}" if vision_errors else
                        " (vision status was not recorded in this session)" if not any("vision_status" in r for r in perception)
                        else ""))
    t_start, t_end = samples[0]["t"], samples[-1]["t"]
    observations = []     # for the joint (object-in-hand offset, camera offset) fit
    findings: List[str] = []
    demo = {"standoff": [], "tilt": [], "palm_height": [], "release_height": [], "palm_dist": []}
    for k, (grasp, release) in enumerate(cycles, start=1):
        Tg, Tr = np.array(grasp["palm"]), np.array(release["palm"])
        qg, qr = np.array(grasp["q_arm"]), np.array(release["q_arm"])
        carried = float(np.linalg.norm(Tr[:3, 3] - Tg[:3, 3]))
        hold = [r["closure"] for r in samples if grasp["t"] + 0.5 <= r["t"] <= release["t"] - 0.2 and "closure" in r]
        hold_closure = float(np.median(hold)) if hold else float(grasp.get("closure") or 0.0)
        lines.append(f"\n--- cycle {k}: grasp at {time.strftime('%H:%M:%S', time.localtime(grasp['t']))}, "
                     f"release at {time.strftime('%H:%M:%S', time.localtime(release['t']))}, palm moved "
                     f"{carried * 100:.0f} cm while closed")
        if hold_closure > EMPTY_HAND_CLOSURE:
            lines.append(f"  the hand was closed on nothing (median closure {hold_closure:.2f}, fingers not blocked); "
                         "not used")
            continue
        if carried < MIN_CARRY_M:
            lines.append("  not a carry (the hand reopened where it closed): a grasp attempt, not used for the fit")
            continue
        lines.append(f"  fingers blocked by an object (median closure {hold_closure:.2f})")
        prev_release = max([r["t"] for _g, r in cycles if r["t"] < grasp["t"]], default=t_start)
        next_grasp = min([g["t"] for g, _r in cycles if g["t"] > release["t"]], default=t_end)
        # Which object: the configured label detected nearest the palm just before the grasp.
        # The object: configured detections near the grasp, between the
        # previous release and this grasp (vision runs at ~0.3-2 Hz).
        pre = _object_track(perception, samples, labels, prev_release + 0.5, grasp["t"] - 0.3)
        cand = [p for p in pre if float(np.linalg.norm(p[:2] - Tg[:2, 3])) < 0.25]
        if not cand:
            lines.append(f"  no detection of {' / '.join(labels)} within 25 cm of the grasp between "
                         f"{time.strftime('%H:%M:%S', time.localtime(prev_release))} and the grasp; cycle skipped")
            continue
        p_pre = _median_cluster(cand)
        n_pre = len(cand)
        label = "object"
        post = _object_track(perception, samples, labels, release["t"] + 1.0, next_grasp - 0.3, near=Tr[:3, 3])
        p_post = _median_cluster(post)
        tilt_g = math.degrees(math.acos(max(-1.0, min(1.0, float(Tg[2, 2])))))
        tilt_r = math.degrees(math.acos(max(-1.0, min(1.0, float(Tr[2, 2])))))
        yaw_g = math.degrees(math.atan2(Tg[1, 0], Tg[0, 0]))
        yaw_r = math.degrees(math.atan2(Tr[1, 0], Tr[0, 0]))
        d_pre = Tg[:3, :3].T @ (p_pre - Tg[:3, 3])
        lines.append(f"  object '{label}' seen before the grasp at ({p_pre[0]:+.3f}, {p_pre[1]:+.3f}, {p_pre[2]:+.3f}) m "
                     f"({n_pre} detections)")
        lines.append(f"  palm at grasp ({Tg[0, 3]:+.3f}, {Tg[1, 3]:+.3f}, {Tg[2, 3]:+.3f}) m, "
                     f"{np.linalg.norm(Tg[:3, 3] - shoulder):.3f} m from the shoulder (model max "
                     f"{MODEL_MAX_PALM_REACH_M:.3f}), palm tilt {tilt_g:.1f} deg, fingers {yaw_g:+.0f} deg")
        lines.append(f"  object relative to the palm (hand frame, as perceived): forward {d_pre[0] * 100:+.1f} cm, "
                     f"sideways {d_pre[1] * 100:+.1f} cm, up {d_pre[2] * 100:+.1f} cm")
        if table_z is not None:
            lines.append(f"  palm {Tg[2, 3] - table_z:.3f} m above the table at the grasp "
                         f"(V2 floor {cfg.grasp_wrist_floor_m():.3f} m)")
            demo["palm_height"].append(Tg[2, 3] - table_z)
        demo["standoff"].append(d_pre[0])
        demo["tilt"].append(tilt_g)
        demo["palm_dist"].append(float(np.linalg.norm(Tg[:3, 3] - shoulder)))
        # What V2's checks say about the configuration the robot actually used.
        reach = checker.check(qg, fk.compute_arm(qg))
        lines.append("  V2 workspace check on the actual grasp joints: " + ("pass" if reach.safe else "; ".join(reach.reasons)))
        if tilt_g > math.degrees(cfg.tuning.grasp_tilt_tolerance_rad):
            findings.append(f"cycle {k}: you grasped with the palm tilted {tilt_g:.0f} deg; V2 requires <= "
                            f"{math.degrees(cfg.tuning.grasp_tilt_tolerance_rad):.0f} deg (level palm)")
        if d_pre[0] > cfg.hand.max_grasp_standoff_m:
            findings.append(f"cycle {k}: the object sat {d_pre[0] * 100:.0f} cm ahead of the palm; V2 allows at most "
                            f"{cfg.hand.max_grasp_standoff_m * 100:.0f} cm")
        if table_z is not None and Tg[2, 3] - table_z < cfg.grasp_wrist_floor_m():
            findings.append(f"cycle {k}: the palm was {Tg[2, 3] - table_z:.3f} m above the table at the grasp; V2's "
                            f"floor is {cfg.grasp_wrist_floor_m():.3f} m (hand model low extent)")
        # Release.
        spot = None
        rec = _nearest(perception, release["t"])
        if rec is not None and rec["patterns"]:
            spot = min(rec["patterns"], key=lambda pt: np.linalg.norm(np.array(pt["centre"])[:2] - Tr[:2, 3]))
        lines.append(f"  palm at release ({Tr[0, 3]:+.3f}, {Tr[1, 3]:+.3f}, {Tr[2, 3]:+.3f}) m, "
                     f"{np.linalg.norm(Tr[:3, 3] - shoulder):.3f} m from the shoulder, tilt {tilt_r:.1f} deg, "
                     f"fingers {yaw_r:+.0f} deg"
                     + ("" if table_z is None else f", {Tr[2, 3] - table_z:.3f} m above the table"))
        if table_z is not None:
            demo["release_height"].append(Tr[2, 3] - table_z)
        if spot is not None:
            c = np.array(spot["centre"])
            lines.append(f"  nearest spot: {spot['name']} centre ({c[0]:+.3f}, {c[1]:+.3f}) m, "
                         f"{np.linalg.norm(c - shoulder):.3f} m from the shoulder")
        # Where the hand put the object: palm at release + the in-hand offset
        # seen at the grasp (independent of what is detected afterwards).
        released = Tr[:3, 3] + Tr[:3, :3] @ d_pre
        on_spot = None
        if spot is not None:
            T_bp = gp1.base_from_camera(rec["cam"]) @ np.array(spot["T_camera_pattern"])
            uv = np.linalg.solve(T_bp, np.append(released, 1.0))
            pat = meta["patterns"][spot["index"]]
            from sorting import parse_pattern, DEFAULT_PATTERNS
            pattern_cfg, _errors = parse_pattern(pat, DEFAULT_PATTERNS[spot["index"]], "AB"[spot["index"]])
            on_spot = pattern_cfg.contains(uv)
            lines.append(f"  released at ({released[0]:+.3f}, {released[1]:+.3f}) m (palm + in-hand offset): "
                         f"{np.linalg.norm(released[:2] - np.array(spot['centre'])[:2]) * 100:.1f} cm from {spot['name']}'s "
                         f"centre, {'ON' if on_spot else 'NOT on'} its marker area")
            demo.setdefault("release_on_spot", []).append(bool(on_spot))
        # Carry height: how high the palm went while holding the object.
        carry = [np.array(r["palm"])[2][3] for r in samples if grasp["t"] <= r["t"] <= release["t"]]
        if carry and table_z is not None:
            demo.setdefault("carry_lift", []).append(float(max(carry) - Tg[2, 3]))
            lines.append(f"  carried with the palm up to {max(carry) - table_z:.3f} m above the table "
                         f"({(max(carry) - Tg[2, 3]) * 100:.0f} cm above the grasp; V2 lifts {cfg.tuning.lift_m * 100:.0f} cm "
                         "and carries at least that high)")
        observations.append((Tg, p_pre))
        if p_post is not None:
            fell = (p_pre[2] - p_post[2] > FELL_DROP_M) or float(np.linalg.norm(p_post[:2] - released[:2])) > FELL_SHIFT_M
            lines.append(f"  object seen after release at ({p_post[0]:+.3f}, {p_post[1]:+.3f}, {p_post[2]:+.3f}) m, "
                         f"{np.linalg.norm(p_post[:2] - released[:2]) * 100:.1f} cm from where it was released"
                         + (", lower than before" if p_pre[2] - p_post[2] > FELL_DROP_M else "")
                         + ("  -> it fell or moved; not used for the camera fit" if fell else ""))
            if not fell:
                observations.append((Tr, p_post))
        else:
            lines.append("  object not seen after release (occluded / vision returned nothing); grasp side only")

    # Joint fit, horizontal only: p_xy = palm_xy + R_xy @ d_xy + e_xy  (d: object
    # in the hand, e: camera offset).  With a level palm the vertical offset of
    # the object in the hand and a camera height error enter identically, so
    # the vertical part is not observable from this data.
    proposal: Optional[Dict[str, Any]] = None
    lines.append("\n=== perception vs kinematics ===")
    fit = None
    if object_offset:
        d_fixed = np.array([float(v) for v in object_offset.split(",")])
        es = [p - (T[:3, 3] + T[:3, :3] @ d_fixed) for T, p in observations]
        if es:
            e = np.median(np.array(es), axis=0)
            fit = {"d": d_fixed, "e": np.array([e[0], e[1], 0.0]), "residual": float(np.std(np.array(es)[:, :2])),
                   "how": f"object-in-hand offset fixed to {d_fixed.tolist()} (--object-offset)"}
    elif len(observations) >= 2:
        A = np.vstack([np.hstack([T[:2, :2], np.eye(2)]) for T, _p in observations])
        b = np.concatenate([p[:2] - T[:2, 3] for T, p in observations])
        yaws = [math.degrees(math.atan2(T[1, 0], T[0, 0])) for T, _p in observations]
        spread = max(yaws) - min(yaws)
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
        sv = np.linalg.svd(A, compute_uv=False)
        cond = float(sv[0] / max(1e-9, sv[-1]))
        resid = A @ x - b
        if cond < 50.0 and spread >= 20.0:
            fit = {"d": np.array([x[0], x[1], 0.0]), "e": np.array([x[2], x[3], 0.0]),
                   "residual": float(np.sqrt(np.mean(resid ** 2))),
                   "how": f"horizontal joint fit over {len(observations)} observations (finger-direction spread "
                          f"{spread:.0f} deg, condition {cond:.0f}); camera height error not observable from this data"}
        else:
            lines.append(f"  the camera offset cannot be separated from where the object sat in the hand: the finger "
                         f"direction only varied by {spread:.0f} deg (need >= 20 deg; condition {cond:.0f}).  Record "
                         "more cycles with the hand pointing in clearly different directions, or pass "
                         "--object-offset fwd,side,up (metres, hand frame) if you measured it.")
    if fit is not None:
        e, d = fit["e"], fit["d"]
        lines.append(f"  {fit['how']}")
        lines.append(f"  object in the hand: forward {d[0] * 100:+.1f} cm, sideways {d[1] * 100:+.1f} cm from the palm link")
        lines.append(f"  camera error (perceived minus kinematic, base frame): x {e[0] * 100:+.1f} cm, y {e[1] * 100:+.1f} cm "
                     f"(height: not observable); fit residual {fit['residual'] * 100:.1f} cm")
        if float(np.linalg.norm(e)) > MAX_CAMERA_CORRECTION_M:
            lines.append(f"  correction larger than {MAX_CAMERA_CORRECTION_M * 100:.0f} cm: not proposed (probably a "
                         "wrong match); check the images in the session")
        else:
            T_bc = gp1.base_from_camera(meta["camera_extrinsic"])
            T_bc[:3, 3] -= e
            roll, pitch, yaw = rpy_from_R(T_bc[:3, :3])
            corrected = {"x": float(T_bc[0, 3]), "y": float(T_bc[1, 3]), "z": float(T_bc[2, 3]),
                         "roll": roll, "pitch": pitch, "yaw": yaw}
            if float(np.linalg.norm(e)) >= 0.01:
                findings.append(f"perception: objects/spots are seen {e[0] * 100:+.1f} cm in x, {e[1] * 100:+.1f} cm in y "
                                f"from where the arm actually had them; correcting the camera mounting moves them back")
            proposal = {"camera_extrinsic": corrected, "camera_correction_m": e}
    # Planner assumptions vs what was demonstrated.
    lines.append("\n=== planner assumptions vs demonstration ===")
    hand, tuning = {}, {}
    if demo["standoff"]:
        need = max(demo["standoff"])
        lines.append(f"  object ahead of the palm: max {need * 100:.1f} cm (V2 default 8, max "
                     f"{cfg.hand.max_grasp_standoff_m * 100:.0f} cm)")
        if need > cfg.hand.max_grasp_standoff_m:
            hand["max_grasp_standoff_m"] = float(min(MAX_PROPOSED_STANDOFF_M, need + 0.005))
    if demo["tilt"]:
        need = math.radians(max(demo["tilt"]))
        lines.append(f"  palm tilt at the grasp: max {max(demo['tilt']):.1f} deg (V2 tolerance "
                     f"{math.degrees(cfg.tuning.grasp_tilt_tolerance_rad):.0f} deg)")
        if need > cfg.tuning.grasp_tilt_tolerance_rad:
            tuning["grasp_tilt_tolerance_rad"] = float(min(MAX_PROPOSED_TILT_RAD, need + math.radians(1.0)))
            if need > MAX_PROPOSED_TILT_RAD:
                findings.append("the demonstrated palm tilt exceeds what V2's level-palm grasp supports; "
                                "a tilted grasp would need planner changes, not a parameter")
    if demo["palm_dist"]:
        lines.append(f"  palm distance from the shoulder at grasps: max {max(demo['palm_dist']):.3f} m "
                     f"(model maximum {MODEL_MAX_PALM_REACH_M:.3f} m)")
    if demo["release_height"]:
        lines.append(f"  palm height above the table at release: {min(demo['release_height']):.3f}-"
                     f"{max(demo['release_height']):.3f} m")
    if demo.get("carry_lift"):
        lifted = max(demo["carry_lift"])
        lines.append(f"  lift while carrying: up to {lifted * 100:.0f} cm above the grasp (V2 lift_m "
                     f"{cfg.tuning.lift_m * 100:.0f} cm; a lower carry keeps far drop-offs reachable)")
        if lifted + 0.01 < cfg.tuning.lift_m:
            lo, _hi = cfg.tuning.ADJUSTABLE["lift_m"]
            tuning["lift_m"] = float(max(lo, round(lifted + 0.01, 3)))
            findings.append(f"you carried the object only {lifted * 100:.0f} cm above the grasp; V2's {cfg.tuning.lift_m * 100:.0f} cm "
                            "lift raises the whole transport, which shortens the reach (proposed lift_m "
                            f"{tuning['lift_m'] * 100:.0f} cm; V2 still enforces its carried-object clearance)")
    if demo.get("release_on_spot"):
        lines.append(f"  releases on the spot's marker area (by palm + in-hand offset): "
                     f"{sum(demo['release_on_spot'])}/{len(demo['release_on_spot'])}")
    if hand or tuning:
        proposal = proposal or {}
        proposal["hand"], proposal["tuning"] = hand, tuning
    lines.append("\n=== findings ===")
    lines += [f"  - {f}" for f in findings] or ["  - none: the demonstration fits V2's assumptions and the perception"]
    analysis = {"session": session.name, "cycles": len(cycles), "observations": len(observations),
                "fit": None if fit is None else {"d": fit["d"], "e": fit["e"], "residual": fit["residual"], "how": fit["how"]},
                "demonstration": demo, "findings": findings, "proposal": proposal}
    (session / "analysis.json").write_text(json.dumps(_jsonable(analysis), indent=2))
    if proposal:
        proposal = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "session": str(session), **proposal}
        (session / "sort_v2_calibration.json").write_text(json.dumps(_jsonable(proposal), indent=2))
        lines.append("\n=== proposed calibration ===\n" + json.dumps(_jsonable(proposal), indent=2))
    report = "\n".join(lines)
    (session / "report.txt").write_text(report)
    return report, proposal


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="teleoperate (/wbc controls) and record a session")
    rec.add_argument("--iface", default=os.environ.get("G1_NETWORK_INTERFACE", "eth0"))
    rec.add_argument("--domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "0")))
    rec.add_argument("--rgbd", default=os.environ.get("G1_RGBD_ENDPOINT", "tcp://127.0.0.1:5555"))
    rec.add_argument("--remote-url", default=os.environ.get("G1_REMOTE_URL", "http://192.168.2.79:8765/events"))
    rec.add_argument("--vision-backend", choices=("openai", "yolo"), default=os.environ.get("G1_VISION_BACKEND", "openai"))
    rec.add_argument("--openai-vision-model", default=os.environ.get("G1_OPENAI_VISION_MODEL", "gpt-4.1"))
    rec.add_argument("--vision-model", default=os.environ.get("G1_VISION_MODEL", "yolov8s-world.pt"))
    rec.add_argument("--force", action="store_true", help="record even if a dashboard process is running")
    ana = sub.add_parser("analyze", help="analyse a recorded session")
    ana.add_argument("session")
    ana.add_argument("--apply", action="store_true", help="install the proposal as academy/sort_v2_calibration.json")
    ana.add_argument("--object-offset", default=None,
                     help="measured object position in the hand 'fwd,side,up' (m, palm-link frame) instead of fitting it")
    args = parser.parse_args()
    return cmd_record(args) if args.command == "record" else cmd_analyze(args)


if __name__ == "__main__":
    sys.path.insert(0, str(HERE))
    raise SystemExit(main())
