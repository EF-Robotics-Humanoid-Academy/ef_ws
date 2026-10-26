"""Reachability map for the V2 /sort "Reach" camera view.

For a grid of table positions it records whether the sort arm can grasp an
object standing there (palm at the lowest grasp height) and carry / release
one there (palm at release height), and if not, why:

  OK            level palm reaches the held-object position, safety checks pass
  NOT_LEVEL     the palm gets there, but not with the palm level (fingers
                closing parallel to the table, as V2 grasps and carries)
  SAFETY        a level pose exists but fails the workspace check (torso
                clearance, workspace radius)
  JOINT_LIMIT   out of reach; the closest pose has joints pinned at their limits
  TOO_FAR       out of reach; beyond the arm's length

It is the most permissive version of what V2 needs: any finger direction,
the object held between the default and the maximum offset ahead of the palm
(HandGeometry.max_grasp_standoff_m), several IK seeds.  Live obstacles are not
part of the map (they move); the view draws them on top.

The map depends only on the arm kinematics, the configuration and the table
height, so it is computed once in a separate process (no GIL contention with
the arm controller) and cached on disk; it is recomputed only when the table
height moves by more than TABLE_TOLERANCE_M.
"""
from __future__ import annotations

import concurrent.futures as cf
import hashlib
import multiprocessing
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

OK, NOT_LEVEL, SAFETY, JOINT_LIMIT, TOO_FAR = 0, 1, 2, 3, 4
UNKNOWN = -1
STATUS_TEXT = {OK: "reachable", NOT_LEVEL: "palm cannot be level", SAFETY: "fails the safety check",
               JOINT_LIMIT: "out of reach (joint limits)", TOO_FAR: "out of reach (beyond arm length)",
               UNKNOWN: "not computed"}
# BGR overlay colours.
STATUS_COLOR = {OK: (80, 200, 80), NOT_LEVEL: (0, 215, 255), SAFETY: (0, 140, 255), JOINT_LIMIT: (60, 60, 230),
                TOO_FAR: (150, 50, 150), UNKNOWN: (120, 120, 120)}
JOINT_NAMES = ("shoulder pitch", "shoulder roll", "shoulder yaw", "elbow", "wrist roll", "wrist pitch", "wrist yaw")
TABLE_TOLERANCE_M = 0.02
# Largest palm-link distance from the shoulder the URDF arm model can reach
# (measured: 20000 random joint samples within the joint limits, then local
# refinement -> 0.4318 m; the wrist frame reaches 0.4101 m).
MODEL_MAX_PALM_REACH_M = 0.4318
CACHE_DIR = Path(__file__).resolve().parent / ".reach_cache"
LAYERS = ("pick", "place")


@dataclass(frozen=True)
class ReachSpec:
    arm: str
    table_z: float
    x_range: Tuple[float, float] = (0.10, 0.70)
    y_range: Tuple[float, float] = (-0.60, 0.40)
    step: float = 0.025
    pick_height: float = 0.10       # palm above the table when grasping (lowest grasp)
    place_height: float = 0.13      # palm above the table when releasing / carrying
    standoffs: Tuple[float, ...] = (0.08, 0.10)
    # Palm farther than this from the shoulder: out of reach without running IK
    # (model maximum + 5 mm so rounding never hides a reachable cell).
    max_dist_m: float = MODEL_MAX_PALM_REACH_M + 0.005
    level_tol_rad: float = math.radians(6.0)
    workspace_max_reach_m: float = 0.52   # ReachabilityChecker radius V2 uses (DEFAULT_DIRECT_MAX_REACH_M)

    def key(self) -> str:
        blob = json.dumps(asdict(self), sort_keys=True).encode()
        return hashlib.sha1(blob).hexdigest()[:12]

    def xs(self) -> np.ndarray:
        return np.arange(self.x_range[0], self.x_range[1] + 1e-9, self.step)

    def ys(self) -> np.ndarray:
        return np.arange(self.y_range[0], self.y_range[1] + 1e-9, self.step)


# ---------------------------------------------------------------------------
# Worker (separate process)
# ---------------------------------------------------------------------------

def _position_solve(fk: Any, q0: np.ndarray, target: np.ndarray, limits: np.ndarray, iters: int = 60
                    ) -> Tuple[np.ndarray, float, bool]:
    """Damped least squares on position only; returns the final q even on
    failure (the library solver returns nothing then)."""
    from hand_pose_navigation.arm_ik import _numerical_jacobian
    q = np.clip(np.asarray(q0, dtype=np.float64), limits[:, 0], limits[:, 1])
    err = float("inf")
    for _ in range(iters):
        e = target - fk.compute_arm(q)[:3, 3]
        err = float(np.linalg.norm(e))
        if err < 0.004:
            return q, err, True
        J = _numerical_jacobian(q, fk)[:3, :]
        dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(3), e)
        n = float(np.linalg.norm(dq))
        if n > 0.3:
            dq *= 0.3 / n
        q = np.clip(q + dq, limits[:, 0], limits[:, 1])
    return q, err, False


def compute_row(spec_dict: Dict[str, Any], x: float, seeds: List[List[float]]) -> Dict[str, Any]:
    """One grid row (fixed x) for both layers: status, residual (m), limit
    joints bitmask, standoff used."""
    from hand_pose_navigation.arm_fk import JOINT_LIMITS, SHOULDER_IN_BASE, ArmFK
    from hand_pose_navigation.arm_ik import ArmIK
    from hand_pose_navigation.reachability_checker import ReachabilityChecker

    spec = ReachSpec(**{k: tuple(v) if isinstance(v, list) else v for k, v in spec_dict.items()})
    fk = ArmFK(arm=spec.arm, backend="urdf")
    limits = np.array(JOINT_LIMITS[spec.arm], dtype=np.float64)
    shoulder = np.asarray(SHOULDER_IN_BASE[spec.arm], dtype=np.float64)
    level = ArmIK(arm=spec.arm, solver="level_dls", max_iter=60, tol_pos_m=0.008, tol_rot_rad=spec.level_tol_rad)
    checker = ReachabilityChecker(arm=spec.arm, max_reach_m=spec.workspace_max_reach_m)
    seed_list = [np.asarray(s, dtype=np.float64) for s in seeds]
    out: Dict[str, Any] = {}
    for layer, height in (("pick", spec.pick_height), ("place", spec.place_height)):
        status, residual, limit_mask, standoff_used = [], [], [], []
        prev: Optional[np.ndarray] = None
        for y in spec.ys():
            centre = np.array([x, y])
            toward = centre - shoulder[:2]
            toward /= max(1e-9, float(np.linalg.norm(toward)))
            best: Tuple[int, float, int, float] = (TOO_FAR, float("inf"), 0, spec.standoffs[0])
            for standoff in spec.standoffs:
                palm = np.array([x - toward[0] * standoff, y - toward[1] * standoff, spec.table_z + height])
                dist = float(np.linalg.norm(palm - shoulder))
                if dist > spec.max_dist_m:
                    cand = (TOO_FAR, dist - MODEL_MAX_PALM_REACH_M, 0, standoff)
                else:
                    seeds_here = ([prev] if prev is not None else []) + seed_list
                    q_best, err_best, ok = None, float("inf"), False
                    for seed in seeds_here:
                        q, err, ok = _position_solve(fk, seed, palm, limits)
                        if err < err_best:
                            q_best, err_best = q, err
                        if ok:
                            break
                    if not ok:
                        margins = np.minimum(q_best - limits[:, 0], limits[:, 1] - q_best)
                        mask = int(sum(1 << i for i, m in enumerate(margins) if m < 0.02))
                        cand = (JOINT_LIMIT if mask else TOO_FAR, err_best, mask, standoff)
                    else:
                        # Level palm, fingers pointing from the shoulder at the object
                        # (grasp_pipeline.level_grasp_rotation; yaw stays free in the IK).
                        yaw = math.atan2(float(toward[1]), float(toward[0]))
                        T = np.eye(4)
                        T[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0],
                                     [0.0, 0.0, 1.0]]
                        T[:3, 3] = palm
                        q_level = None
                        for seed in [q_best] + seeds_here:
                            q_level, _info = level.solve(T, q_init=seed)
                            if q_level is not None:
                                break
                        if q_level is None:
                            cand = (NOT_LEVEL, 0.0, 0, standoff)
                        else:
                            check = checker.check(q_level, fk.compute_arm(q_level))
                            cand = (OK, 0.0, 0, standoff) if check.safe else (SAFETY, 0.0, 0, standoff)
                            prev = q_level
                        if prev is None:
                            prev = q_best
                if cand[0] < best[0] or (cand[0] == best[0] and cand[1] < best[1]):
                    best = cand
                if best[0] == OK:
                    break
            status.append(best[0])
            residual.append(best[1] if math.isfinite(best[1]) else 1.0)
            limit_mask.append(best[2])
            standoff_used.append(best[3])
        out[layer] = {"status": status, "residual": residual, "limits": limit_mask, "standoff": standoff_used}
    return out


# ---------------------------------------------------------------------------
# Map holder (dashboard process)
# ---------------------------------------------------------------------------

class ReachMap:
    def __init__(self, arm: str, seeds: Sequence[np.ndarray], pick_height: float, place_height: float,
                 standoffs: Tuple[float, ...]) -> None:
        self.arm = arm
        self.seeds = [list(map(float, s)) for s in seeds]
        self.pick_height = pick_height
        self.place_height = place_height
        self.standoffs = standoffs
        self.spec: Optional[ReachSpec] = None
        self.grids: Dict[str, Dict[str, np.ndarray]] = {}
        self.progress = 0.0
        self.state = "idle"          # idle | computing | ready | error
        self.error = ""
        self.computed_s = 0.0
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    # -- lifecycle ------------------------------------------------------------
    def ensure(self, table_z: float) -> None:
        """Start (or load) the map for this table height; returns at once."""
        with self._lock:
            if self.state == "computing":
                return
            if self.spec is not None and self.state == "ready" and abs(self.spec.table_z - table_z) <= TABLE_TOLERANCE_M:
                return
            spec = ReachSpec(arm=self.arm, table_z=round(float(table_z), 2), pick_height=self.pick_height,
                             place_height=self.place_height, standoffs=tuple(self.standoffs))
            self.spec = spec
            if self._load(spec):
                return
            self.state, self.progress, self.error = "computing", 0.0, ""
            self._thread = threading.Thread(target=self._compute, args=(spec,), name="reach-map", daemon=True)
            self._thread.start()

    def _cache_path(self, spec: ReachSpec) -> Path:
        return CACHE_DIR / f"reach_{spec.arm}_{spec.key()}.npz"

    def _load(self, spec: ReachSpec) -> bool:
        path = self._cache_path(spec)
        if not path.exists():
            return False
        try:
            data = np.load(path)
            self.grids = {layer: {k: data[f"{layer}_{k}"] for k in ("status", "residual", "limits", "standoff")}
                          for layer in LAYERS}
            self.computed_s = float(data["computed_s"])
            self.state, self.progress = "ready", 1.0
            return True
        except Exception:
            return False

    def _compute(self, spec: ReachSpec) -> None:
        t0 = time.time()
        xs = spec.xs()
        rows: Dict[int, Dict[str, Any]] = {}
        try:
            # Spawned, not forked: the dashboard process runs DDS threads whose
            # locks a forked child would inherit.
            # A few workers, leaving cores for the dashboard and the arm controller.
            workers = max(1, min(4, (multiprocessing.cpu_count() or 2) - 2))
            with cf.ProcessPoolExecutor(max_workers=workers, mp_context=multiprocessing.get_context("spawn")) as pool:
                futures = {pool.submit(compute_row, asdict(spec), float(x), self.seeds): i for i, x in enumerate(xs)}
                for done, fut in enumerate(cf.as_completed(futures), start=1):
                    rows[futures[fut]] = fut.result()
                    self.progress = done / len(xs)
            grids = {}
            for layer in LAYERS:
                grids[layer] = {k: np.array([rows[i][layer][k] for i in range(len(xs))])
                                for k in ("status", "residual", "limits", "standoff")}
            elapsed = time.time() - t0
            CACHE_DIR.mkdir(parents=True, exist_ok=True)
            np.savez(self._cache_path(spec), computed_s=elapsed,
                     **{f"{layer}_{k}": v for layer, g in grids.items() for k, v in g.items()})
            with self._lock:
                self.grids, self.computed_s, self.state, self.progress = grids, elapsed, "ready", 1.0
        except Exception as exc:
            with self._lock:
                self.state, self.error = "error", f"{type(exc).__name__}: {exc}"

    # -- queries --------------------------------------------------------------
    def lookup(self, layer: str, x: float, y: float) -> Optional[Dict[str, Any]]:
        """Status at the nearest cell of (x, y), or None while not ready /
        outside the grid."""
        spec = self.spec
        if self.state != "ready" or spec is None:
            return None
        i = int(round((x - spec.x_range[0]) / spec.step))
        j = int(round((y - spec.y_range[0]) / spec.step))
        g = self.grids[layer]
        if not (0 <= i < g["status"].shape[0] and 0 <= j < g["status"].shape[1]):
            return {"status": TOO_FAR, "text": "outside the computed area", "residual_m": None, "limits": []}
        status = int(g["status"][i, j])
        mask = int(g["limits"][i, j])
        limits = [JOINT_NAMES[k] for k in range(7) if mask & (1 << k)]
        residual = float(g["residual"][i, j])
        text = STATUS_TEXT[status]
        if status == JOINT_LIMIT:
            text += f": {', '.join(limits)} at limit, {residual * 1000:.0f} mm short"
        elif status == TOO_FAR:
            text += f", {residual * 1000:.0f} mm too far" if residual < 0.9 else ""
        return {"status": status, "text": text, "residual_m": residual, "limits": limits,
                "standoff_m": float(g["standoff"][i, j])}

    def explain(self, layer: str, x: float, y: float) -> str:
        info = self.lookup(layer, x, y)
        if info is None:
            return f"reach map {self.state}" + (f" ({self.progress * 100:.0f}%)" if self.state == "computing" else "")
        return f"reach map ({layer}): {info['text']}"

    def status(self) -> Dict[str, Any]:
        spec = self.spec
        out = {"state": self.state, "progress": self.progress, "error": self.error, "computed_s": self.computed_s,
               "table_z": None if spec is None else spec.table_z}
        if self.state == "ready":
            for layer in LAYERS:
                s = self.grids[layer]["status"]
                out[f"{layer}_counts"] = {STATUS_TEXT[k]: int(np.sum(s == k)) for k in STATUS_TEXT if k != UNKNOWN}
        return out

    # -- drawing ----------------------------------------------------------------
    def draw(self, image: np.ndarray, project: Any, marks: Sequence[Tuple[str, str, float, float]]) -> np.ndarray:
        """Overlay on a camera image.  `project(xyz_base) -> (u, v) | None`;
        `marks`: (label, layer, x, y) of spots / objects to annotate."""
        import cv2
        out = image.copy()
        spec = self.spec
        if self.state != "ready" or spec is None:
            msg = {"computing": f"computing the reach map... {self.progress * 100:.0f}% (once; cached afterwards)",
                   "error": f"reach map failed: {self.error}", "idle": "reach map: waiting for a table plane"}[
                self.state if self.state in ("computing", "error") else "idle"]
            cv2.putText(out, msg, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(out, msg, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            return out
        overlay = out.copy()
        h = spec.step / 2.0
        z = spec.table_z
        pick = self.grids["pick"]["status"]
        place = self.grids["place"]["status"]
        for i, x in enumerate(spec.xs()):
            for j, y in enumerate(spec.ys()):
                corners = [project(np.array([x + dx, y + dy, z])) for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h))]
                if any(c is None for c in corners):
                    continue
                cv2.fillPoly(overlay, [np.array(corners, dtype=np.int32)], STATUS_COLOR[int(pick[i, j])])
                centre = project(np.array([x, y, z]))
                if centre is not None and int(place[i, j]) != int(pick[i, j]):
                    cv2.circle(overlay, centre, 3, STATUS_COLOR[int(place[i, j])], -1, cv2.LINE_AA)
        out = cv2.addWeighted(overlay, 0.45, out, 0.55, 0)
        for label, layer, x, y in marks:
            uv = project(np.array([x, y, z]))
            info = self.lookup(layer, x, y)
            if uv is None or info is None:
                continue
            color = STATUS_COLOR[info["status"]]
            cv2.drawMarker(out, uv, color, cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
            text = f"{label}: {info['text']}"
            cv2.putText(out, text, (uv[0] + 8, uv[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, text, (uv[0] + 8, uv[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        y0 = 18
        for status in (OK, NOT_LEVEL, SAFETY, JOINT_LIMIT, TOO_FAR):
            cv2.rectangle(out, (10, y0 - 10), (22, y0 + 2), STATUS_COLOR[status], -1)
            cv2.putText(out, STATUS_TEXT[status], (28, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(out, STATUS_TEXT[status], (28, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                        cv2.LINE_AA)
            y0 += 16
        note = f"cells: picking (palm {spec.pick_height * 100:.0f} cm up) | dots: placing ({spec.place_height * 100:.0f} cm)"
        cv2.putText(out, note, (10, y0 + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, note, (10, y0 + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        return out
