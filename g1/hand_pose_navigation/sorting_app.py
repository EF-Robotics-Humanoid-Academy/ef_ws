#!/usr/bin/env python3
"""
Sorting layer — pick two kinds of objects and drop each on its ArUco spot.

Built on recognition_app_v4.py, which is imported as a library: its RGB-D
capture + vision perception threads, Unitree control robot, extend/grab
pipeline (DirectHandPoseNav + tactile-guided close), lift to the WBC stable
pose, and the safety actions are all reused unchanged.  This app adds:

  - Two configurable ArUco drop-off patterns (dictionary, single marker or
    grid board, first marker ID, marker size, grid rows/cols/separation,
    release height).  Each pattern's pose is estimated with solvePnP from
    whichever of its markers are visible; the drop point is the pattern
    centre, offset along its upward normal by the release height.
  - Two object definitions (open-vocabulary recognition prompt, optional
    colour filter, mass, how many to sort, assigned drop-off pattern).  Both
    prompts are fed to the shared YOLO-World detector; the colour filter
    lets two objects of the same type but a different colour be told apart
    ("cube" red -> A, "cube" blue -> B).
  - A Sort button that repeatedly picks one object, lifts it to the stable
    pose, places it on its assigned pattern, and returns to the stable pose,
    until every requested object is sorted or no more can be found.  Objects
    already lying on their own pattern are ignored; objects lying on the
    other pattern are treated as misplaced and are picked up.

Each grab uses v4's dynamic target correction (toggle in Motion options):
the approach target follows fresh detections of the object being picked.

Sorting uses the right arm only (the WBC stable pose that lift/placement
return to is defined for the right arm).  The robot base is never stepped
during a sort, so every object and both patterns must be within reach.

Run:
    python3 sorting_app.py [--iface eth0] [--domain-id 0] [--mock] [--port 8060]
(same command-line options as recognition_app_v4.py), then open
http://<host>:<port> in a browser on the same network.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

# ── path setup ────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
_G1_DIR = os.path.abspath(os.path.join(_DIR, ".."))
for _p in (_G1_DIR, os.path.join(_G1_DIR, "modules")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Importing v4 parses the command line, connects the camera/robot and starts
# its capture, perception, FK and tactile threads.  Its own Dash app is
# created but never served.  Module globals that v4 reassigns at runtime
# (K, ACTIVE_DIRECT_NAV, ACTIVE_HOLD_EXECUTOR) are always read as rec.<name>.
from hand_pose_navigation import recognition_app_v4 as rec
from hand_pose_navigation.direct_nav import _make_transform
from hand_pose_navigation.segmentation import draw_segmentation_overlay
from hand_pose_navigation.vision_detector import parse_prompt_to_classes

import dash
from dash import dcc, html, Input, Output, State, callback_context, no_update
import dash_bootstrap_components as dbc


# ---------------------------------------------------------------------------
# Configuration model
# ---------------------------------------------------------------------------

SORT_ARM = "right"
PATTERN_COUNT = 2
OBJECT_COUNT = 2
PATTERN_LETTERS = ("A", "B")

# Predefined OpenCV ArUco dictionaries offered in the UI, with their size so
# marker IDs can be validated before detection.
ARUCO_DICTIONARIES: Dict[str, int] = {
    "DICT_4X4_50": 50,
    "DICT_4X4_100": 100,
    "DICT_4X4_250": 250,
    "DICT_5X5_50": 50,
    "DICT_5X5_100": 100,
    "DICT_5X5_250": 250,
    "DICT_6X6_50": 50,
    "DICT_6X6_100": 100,
    "DICT_6X6_250": 250,
    "DICT_7X7_50": 50,
    "DICT_7X7_250": 250,
    "DICT_ARUCO_ORIGINAL": 1024,
}

COLOR_FILTERS = ["any", "red", "orange", "yellow", "green", "blue", "purple", "black", "white", "gray"]
# Fraction of an object's mask pixels that must fall in the requested colour.
COLOR_MATCH_MIN_FRACTION = 0.30

DEFAULT_RELEASE_HEIGHT_M = rec.DEFAULT_BOARD_PLACE_RELEASE_M
MIN_RELEASE_HEIGHT_M = rec.TABLE_COLLISION_MARGIN_M
MAX_RELEASE_HEIGHT_M = 0.30
PLACE_APPROACH_EXTRA_M = 0.05
# An object whose base-frame XY lies within the pattern's half-diagonal plus
# this margin counts as lying on that pattern.
ZONE_MARGIN_M = 0.04
# With the waist fixed, a pattern pose stays valid while the robot's arm or
# the carried object occludes it; how long a remembered pose is trusted.
PATTERN_MEMORY_S = 300.0
PATTERN_WAIT_S = 4.0
VIEW_SETTLE_S = 0.4
CANDIDATE_WAIT_S = 6.0
MAX_FAILED_PICKS_PER_OBJECT = 3
MAX_ITEMS_PER_OBJECT = 10

PATTERN_COLORS_BGR = ((255, 80, 255), (80, 255, 120))
OBJECT_COLORS_BGR = ((0, 170, 255), (255, 170, 0))
SORTED_COLOR_BGR = (90, 200, 90)
IGNORED_COLOR_BGR = (150, 150, 150)


@dataclass
class PatternConfig:
    name: str
    dictionary: str
    layout: str  # "single" | "grid"
    first_id: int
    marker_length_m: float
    rows: int
    cols: int
    separation_m: float
    release_height_m: float

    @property
    def grid_rows(self) -> int:
        return 1 if self.layout == "single" else self.rows

    @property
    def grid_cols(self) -> int:
        return 1 if self.layout == "single" else self.cols

    @property
    def marker_count(self) -> int:
        return self.grid_rows * self.grid_cols

    def marker_ids(self) -> List[int]:
        return list(range(self.first_id, self.first_id + self.marker_count))

    @property
    def width_m(self) -> float:
        return self.grid_cols * self.marker_length_m + (self.grid_cols - 1) * self.separation_m

    @property
    def height_m(self) -> float:
        return self.grid_rows * self.marker_length_m + (self.grid_rows - 1) * self.separation_m

    @property
    def zone_radius_m(self) -> float:
        return 0.5 * math.hypot(self.width_m, self.height_m) + ZONE_MARGIN_M

    def marker_object_corners(self, marker_id: int) -> np.ndarray:
        """Pattern-frame corners of one marker in OpenCV corner order.

        Same convention as v4's GridBoard: origin at the outer top-left
        corner of the first marker, +X across columns, +Y down rows.
        """
        index = int(marker_id) - self.first_id
        col, row = index % self.grid_cols, index // self.grid_cols
        pitch = self.marker_length_m + self.separation_m
        x0, y0 = col * pitch, row * pitch
        s = self.marker_length_m
        return np.array(
            [[x0, y0, 0.0], [x0 + s, y0, 0.0], [x0 + s, y0 + s, 0.0], [x0, y0 + s, 0.0]],
            dtype=np.float32,
        )

    def centre_pattern(self) -> np.ndarray:
        return np.array([self.width_m / 2.0, self.height_m / 2.0, 0.0, 1.0], dtype=np.float64)

    def describe(self) -> str:
        ids = self.marker_ids()
        id_text = f"id {ids[0]}" if len(ids) == 1 else f"ids {ids[0]}-{ids[-1]}"
        layout = "single marker" if self.layout == "single" else f"{self.grid_rows}x{self.grid_cols} grid"
        return (
            f"{self.dictionary} {layout}, {id_text}, {self.marker_length_m * 1000:.0f} mm, "
            f"pattern {self.width_m * 100:.1f}x{self.height_m * 100:.1f} cm"
        )


@dataclass
class ObjectConfig:
    prompt: str
    color: str
    mass_kg: float
    count: int
    zone: int  # index into the pattern list
    classes: List[str]

    @property
    def name(self) -> str:
        return self.prompt if self.color == "any" else f"{self.color} {self.prompt}"


DEFAULT_PATTERNS = [
    PatternConfig("Spot A", "DICT_4X4_50", "single", 10, 0.050, 2, 2, 0.010, DEFAULT_RELEASE_HEIGHT_M),
    PatternConfig("Spot B", "DICT_4X4_50", "single", 11, 0.050, 2, 2, 0.010, DEFAULT_RELEASE_HEIGHT_M),
]
DEFAULT_OBJECTS = [
    ObjectConfig("cube", "red", 0.10, 1, 0, ["cube"]),
    ObjectConfig("cup", "any", 0.20, 1, 1, ["cup"]),
]


class SortState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.patterns: List[PatternConfig] = list(DEFAULT_PATTERNS)
        self.objects: List[ObjectConfig] = list(DEFAULT_OBJECTS)
        self.config_errors: List[str] = []
        self.vision_prompt = ""
        self.center_waist = False
        self.allow_unverified_grasp = False
        # Latest pose of each pattern in the camera frame and the view epoch
        # it was measured in.  The epoch increments whenever this app turns
        # the waist, which moves the camera relative to the arm base frame.
        self.pattern_T_camera: List[Optional[np.ndarray]] = [None] * PATTERN_COUNT
        self.pattern_ts: List[float] = [0.0] * PATTERN_COUNT
        self.pattern_epoch: List[int] = [-1] * PATTERN_COUNT
        self.pattern_status: List[str] = ["waiting for camera"] * PATTERN_COUNT
        self.pattern_markers: List[Tuple[List[np.ndarray], np.ndarray]] = []
        self.view_epoch = 0
        self.view_settle_until = 0.0
        self.running = False
        self.phase = "idle"
        self.sorted_counts = [0] * OBJECT_COUNT
        self.current_target: Optional[Tuple[int, Tuple[int, int, int, int]]] = None
        # Index of the object definition being grabbed, read by
        # _SortTargetCorrector; None outside a sort grab.
        self.active_object: Optional[int] = None
        self.summary = ""


SORT = SortState()
SORT_RUN_LOCK = threading.Lock()
SORT_STOP_EVENT = threading.Event()
_ARUCO_DETECTORS: Dict[str, Any] = {}


def _log(line: str) -> None:
    rec._log(f"[sorting_app] {line}")


def _set_phase(phase: str) -> None:
    with SORT.lock:
        SORT.phase = phase
    _log(phase)


def _to_int(value, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _parse_pattern(values: Dict[str, Any], fallback: PatternConfig, letter: str) -> Tuple[PatternConfig, List[str]]:
    errors: List[str] = []
    dictionary = values.get("dict") if values.get("dict") in ARUCO_DICTIONARIES else fallback.dictionary
    layout = values.get("layout") if values.get("layout") in ("single", "grid") else "single"
    pattern = PatternConfig(
        name=str(values.get("name") or f"Spot {letter}").strip() or f"Spot {letter}",
        dictionary=dictionary,
        layout=layout,
        first_id=max(0, _to_int(values.get("id"), fallback.first_id)),
        marker_length_m=_to_float(values.get("size"), fallback.marker_length_m * 1000.0) / 1000.0,
        rows=max(1, _to_int(values.get("rows"), fallback.rows)),
        cols=max(1, _to_int(values.get("cols"), fallback.cols)),
        separation_m=max(0.0, _to_float(values.get("sep"), fallback.separation_m * 1000.0) / 1000.0),
        release_height_m=_to_float(values.get("height"), fallback.release_height_m * 100.0) / 100.0,
    )
    if pattern.marker_length_m <= 0.005:
        errors.append(f"Pattern {letter}: marker size must be larger than 5 mm.")
    last_id = pattern.first_id + pattern.marker_count - 1
    if last_id >= ARUCO_DICTIONARIES[pattern.dictionary]:
        errors.append(
            f"Pattern {letter}: marker IDs up to {last_id} exceed {pattern.dictionary} "
            f"(IDs 0-{ARUCO_DICTIONARIES[pattern.dictionary] - 1})."
        )
    if not MIN_RELEASE_HEIGHT_M <= pattern.release_height_m <= MAX_RELEASE_HEIGHT_M:
        errors.append(
            f"Pattern {letter}: release height must be {MIN_RELEASE_HEIGHT_M * 100:.0f}-"
            f"{MAX_RELEASE_HEIGHT_M * 100:.0f} cm (the table collision margin is "
            f"{rec.TABLE_COLLISION_MARGIN_M * 100:.0f} cm)."
        )
    return pattern, errors


def _parse_object(values: Dict[str, Any], index: int) -> Tuple[ObjectConfig, List[str]]:
    errors: List[str] = []
    prompt = str(values.get("prompt") or "").strip()
    color = values.get("color") if values.get("color") in COLOR_FILTERS else "any"
    mass_kg = max(0.0, min(rec.MAX_OBJECT_MASS_KG, _to_float(values.get("mass"), rec.DEFAULT_OBJECT_MASS_KG)))
    count = max(1, min(MAX_ITEMS_PER_OBJECT, _to_int(values.get("count"), 1)))
    zone = _to_int(values.get("zone"), index)
    if zone not in range(PATTERN_COUNT):
        zone = index
    classes = parse_prompt_to_classes(prompt) if prompt else []
    if not prompt:
        errors.append(f"Object {index + 1}: enter a recognition prompt.")
    return ObjectConfig(prompt, color, mass_kg, count, zone, classes), errors


def _validate_pair(patterns: List[PatternConfig], objects: List[ObjectConfig]) -> List[str]:
    errors: List[str] = []
    a, b = patterns
    if a.dictionary == b.dictionary and set(a.marker_ids()) & set(b.marker_ids()):
        errors.append("Patterns A and B use overlapping marker IDs in the same dictionary.")
    o1, o2 = objects
    shared = set(o1.classes) & set(o2.classes)
    if shared and (o1.color == o2.color or "any" in (o1.color, o2.color)):
        errors.append(
            f"Objects 1 and 2 both match {', '.join(sorted(shared))!r}; give them different "
            "prompts or two different (non-'any') colour filters."
        )
    return errors


def _apply_config(patterns: List[PatternConfig], objects: List[ObjectConfig], errors: List[str]) -> None:
    classes: List[str] = []
    for obj in objects:
        for cls in obj.classes:
            if cls not in classes:
                classes.append(cls)
    prompt = ", ".join(classes)
    with SORT.lock:
        SORT.patterns = patterns
        SORT.objects = objects
        SORT.config_errors = errors
        prompt_changed = bool(prompt) and prompt != SORT.vision_prompt
        if prompt_changed:
            SORT.vision_prompt = prompt
    if prompt_changed:
        detector_classes = rec._VISION.set_prompt(prompt)
        with rec.STATE.lock:
            rec.STATE.vision_classes = detector_classes
        _log(f"Vision prompt set to: {', '.join(detector_classes)}")


# ---------------------------------------------------------------------------
# ArUco pattern detection
# ---------------------------------------------------------------------------

def _camera_matrix() -> np.ndarray:
    K = rec.K
    return np.array([[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def _detect_markers(gray: np.ndarray, dictionary_name: str) -> Tuple[List[np.ndarray], np.ndarray]:
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    if hasattr(aruco, "ArucoDetector"):
        detector = _ARUCO_DETECTORS.get(dictionary_name)
        if detector is None:
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            _ARUCO_DETECTORS[dictionary_name] = detector
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:
        corners, ids, _rejected = aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return [], np.zeros((0,), dtype=np.int32)
    return list(corners), ids.reshape(-1).astype(np.int32)


def _estimate_pattern_pose(
    pattern: PatternConfig,
    corners: List[np.ndarray],
    ids: np.ndarray,
) -> Tuple[Optional[np.ndarray], int, List[np.ndarray], List[int]]:
    wanted = set(pattern.marker_ids())
    object_points, image_points, used_corners, used_ids = [], [], [], []
    for corners_i, marker_id in zip(corners, ids):
        if int(marker_id) in wanted:
            object_points.append(pattern.marker_object_corners(int(marker_id)))
            image_points.append(np.asarray(corners_i, dtype=np.float32).reshape(4, 2))
            used_corners.append(corners_i)
            used_ids.append(int(marker_id))
    if not object_points:
        return None, 0, used_corners, used_ids
    obj = np.concatenate(object_points).astype(np.float64)
    img = np.concatenate(image_points).astype(np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, _camera_matrix(), dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(obj, img, _camera_matrix(), dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, len(used_ids), used_corners, used_ids
    R, _ = cv2.Rodrigues(rvec)
    T_camera_pattern = np.eye(4, dtype=np.float64)
    T_camera_pattern[:3, :3] = R
    T_camera_pattern[:3, 3] = tvec.reshape(3)
    if T_camera_pattern[2, 3] <= 0.05:
        return None, len(used_ids), used_corners, used_ids
    return T_camera_pattern, len(used_ids), used_corners, used_ids


def _pattern_loop(rate_hz: float) -> None:
    period = 1.0 / max(0.5, rate_hz)
    last_seq = -1
    if not hasattr(cv2, "aruco"):
        with SORT.lock:
            SORT.pattern_status = ["OpenCV was built without cv2.aruco (install opencv-contrib)"] * PATTERN_COUNT
        return
    while True:
        t0 = time.time()
        with rec.STATE.lock:
            seq = rec.STATE.frame_seq
            frame_ts = rec.STATE.last_frame_ts
            rgb = None if rec.STATE.rgb_bgr is None else rec.STATE.rgb_bgr.copy()
        if rgb is None or seq == last_seq:
            time.sleep(0.05)
            continue
        last_seq = seq
        with SORT.lock:
            patterns = list(SORT.patterns)
            epoch = SORT.view_epoch if frame_ts >= SORT.view_settle_until else -1
        try:
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            markers = {name: _detect_markers(gray, name) for name in {p.dictionary for p in patterns}}
            overlay: List[Tuple[List[np.ndarray], np.ndarray]] = []
            results = []
            for pattern in patterns:
                corners, ids = markers[pattern.dictionary]
                T, count, used_corners, used_ids = _estimate_pattern_pose(pattern, corners, ids)
                if used_corners:
                    overlay.append((used_corners, np.asarray(used_ids, dtype=np.int32).reshape(-1, 1)))
                results.append((T, count))
            with SORT.lock:
                SORT.pattern_markers = overlay
                for i, (T, count) in enumerate(results):
                    if T is None:
                        SORT.pattern_status[i] = (
                            f"not visible (0/{patterns[i].marker_count} markers)" if count == 0
                            else f"{count}/{patterns[i].marker_count} markers, pose failed"
                        )
                        continue
                    SORT.pattern_T_camera[i] = T
                    SORT.pattern_ts[i] = frame_ts
                    SORT.pattern_epoch[i] = epoch
                    SORT.pattern_status[i] = f"visible ({count}/{patterns[i].marker_count} markers)"
        except Exception as exc:
            with SORT.lock:
                SORT.pattern_status = [f"detection error: {exc}"] * PATTERN_COUNT
        time.sleep(max(0.0, period - (time.time() - t0)))


def _base_from_camera(cam: Dict[str, float]) -> np.ndarray:
    return _make_transform(
        xyz=(cam["x"], cam["y"], cam["z"]),
        rpy=(cam["roll"], cam["pitch"], cam["yaw"]),
    )


def _pattern_T_camera(index: int, require_current_epoch: bool) -> Optional[np.ndarray]:
    with SORT.lock:
        T = SORT.pattern_T_camera[index]
        age = time.time() - SORT.pattern_ts[index]
        epoch_ok = SORT.pattern_epoch[index] == SORT.view_epoch
    if T is None or age > PATTERN_MEMORY_S:
        return None
    if require_current_epoch and not epoch_ok:
        return None
    return T.copy()


def _pattern_fresh(index: int, since_ts: float) -> bool:
    with SORT.lock:
        return (
            SORT.pattern_T_camera[index] is not None
            and SORT.pattern_ts[index] >= since_ts
            and SORT.pattern_epoch[index] == SORT.view_epoch
        )


def _wait_for_pattern(index: int, since_ts: float, timeout_s: float = PATTERN_WAIT_S) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _pattern_fresh(index, since_ts):
            return True
        if SORT_STOP_EVENT.is_set():
            return False
        time.sleep(0.1)
    return _pattern_fresh(index, since_ts)


def _pattern_drop_geometry(
    pattern: PatternConfig,
    T_camera_pattern: np.ndarray,
    cam: Dict[str, float],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return the pattern centre and its upward normal in base_link."""
    T_base_pattern = _base_from_camera(cam) @ T_camera_pattern
    centre_base = (T_base_pattern @ pattern.centre_pattern())[:3]
    normal_base = T_base_pattern[:3, 2].copy()
    if float(normal_base[2]) < 0.0:
        normal_base *= -1.0
    return centre_base, normal_base


# ---------------------------------------------------------------------------
# Object classification
# ---------------------------------------------------------------------------

def _color_masks(hsv: np.ndarray) -> Dict[str, np.ndarray]:
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    chromatic = (s >= 70) & (v >= 50)
    return {
        "red": chromatic & ((h < 8) | (h >= 170)),
        "orange": chromatic & (h >= 8) & (h < 20),
        "yellow": chromatic & (h >= 20) & (h < 35),
        "green": chromatic & (h >= 35) & (h < 85),
        "blue": chromatic & (h >= 85) & (h < 130),
        "purple": chromatic & (h >= 130) & (h < 170),
        "black": v < 50,
        "white": (s < 40) & (v >= 170),
        "gray": (s < 40) & (v >= 50) & (v < 170),
    }


def _color_fraction(
    rgb: Optional[np.ndarray],
    mask: Optional[np.ndarray],
    box: Optional[Tuple[int, int, int, int]],
    color: str,
) -> float:
    if color == "any":
        return 1.0
    if rgb is None:
        return 0.0
    if mask is None or mask.shape != rgb.shape[:2] or not mask.any():
        if box is None:
            return 0.0
        # No depth mask: use the inner half of the box to skip background.
        x0, y0, x1, y1 = (int(v) for v in box)
        dx, dy = (x1 - x0) // 4, (y1 - y0) // 4
        mask = np.zeros(rgb.shape[:2], dtype=bool)
        mask[max(0, y0 + dy):max(0, y1 - dy), max(0, x0 + dx):max(0, x1 - dx)] = True
        if not mask.any():
            return 0.0
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    pixels = hsv[mask]
    return float(_color_masks(pixels[np.newaxis, ...])[color].mean())


def _scene_snapshot() -> Dict[str, Any]:
    with rec.STATE.lock:
        snap = {
            "detections": {k: rec._copy_detection(v) for k, v in rec.STATE.detections.items()},
            "masks": {det_id: mask for det_id, _label, mask in rec.STATE.masks_for_draw},
            "boxes": list(rec.STATE.boxes_for_draw),
            "rgb": None if rec.STATE.detection_rgb_bgr is None else rec.STATE.detection_rgb_bgr.copy(),
            "detection_ts": rec.STATE.last_detection_ts,
            "frame_ts": rec.STATE.last_frame_ts,
            "cam": dict(rec.STATE.camera_extrinsic),
        }
    with SORT.lock:
        snap["patterns"] = list(SORT.patterns)
        snap["objects"] = list(SORT.objects)
    return snap


def _zone_centres(patterns: List[PatternConfig], cam: Dict[str, float]) -> List[Optional[np.ndarray]]:
    centres: List[Optional[np.ndarray]] = []
    for i, pattern in enumerate(patterns):
        T = _pattern_T_camera(i, require_current_epoch=True)
        if T is None:
            T = _pattern_T_camera(i, require_current_epoch=False)
        centres.append(None if T is None else _pattern_drop_geometry(pattern, T, cam)[0])
    return centres


def _classify(snap: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Assign each detection to an object definition and drop-off zone.

    Returns one entry per detection with keys det, obj (index or None),
    zone (index of the pattern it lies on, or None), color_fraction and
    sortable (assigned and not already on its own pattern).
    """
    now = time.time()
    if snap["detection_ts"] <= 0.0 or now - snap["detection_ts"] > rec.STALE_AFTER_S:
        return []
    if now - snap["frame_ts"] > rec.STALE_AFTER_S:
        return []
    objects: List[ObjectConfig] = snap["objects"]
    patterns: List[PatternConfig] = snap["patterns"]
    cam = snap["cam"]
    T_base_camera = _base_from_camera(cam)
    centres = _zone_centres(patterns, cam)
    entries = []
    for det_id, det in sorted(snap["detections"].items()):
        best_obj, best_fraction = None, -1.0
        for obj_index, obj in enumerate(objects):
            if det.label not in obj.classes:
                continue
            fraction = _color_fraction(snap["rgb"], snap["masks"].get(det_id), det.box, obj.color)
            if obj.color != "any" and fraction < COLOR_MATCH_MIN_FRACTION:
                continue
            if fraction > best_fraction:
                best_obj, best_fraction = obj_index, fraction
        p_base = (T_base_camera @ det.T_camera_object)[:3, 3]
        zone = None
        for zone_index, centre in enumerate(centres):
            if centre is None:
                continue
            if float(np.linalg.norm(p_base[:2] - centre[:2])) <= patterns[zone_index].zone_radius_m:
                zone = zone_index
                break
        sortable = best_obj is not None and zone != objects[best_obj].zone
        entries.append({
            "det": det, "obj": best_obj, "zone": zone, "p_base": p_base,
            "color_fraction": best_fraction, "sortable": sortable,
        })
    return entries


def _pick_candidate(obj_index: int, min_detection_ts: float, prefer_centre: bool) -> Optional[Any]:
    snap = _scene_snapshot()
    if snap["detection_ts"] < min_detection_ts:
        return None
    entries = [e for e in _classify(snap) if e["obj"] == obj_index and e["sortable"]]
    if not entries:
        return None
    K = rec.K
    if prefer_centre:
        # After the waist has turned toward the target it sits nearest the
        # image's vertical centre line.
        def key(e):
            p = e["det"].T_camera_object[:3, 3]
            return abs(K.fx * float(p[0]) / max(0.05, float(p[2])))
    else:
        def key(e):
            return (-round(float(e["det"].score), 2), float(np.linalg.norm(e["p_base"][:2])))
    return min(entries, key=key)["det"]


class _SortTargetCorrector(rec._GrabTargetCorrector):
    """v4's dynamic grab target correction, matched like the sort picks.

    v4 only knows detection labels, so its corrector accepts any same-label
    detection inside the gate.  During a sort grab this one accepts only
    detections that _classify() assigns to the object being grabbed — label
    in its prompt classes, colour filter passed, and not already lying on its
    own drop-off pattern — so a same-type object of the other colour, or one
    that is already sorted, can never pull the approach toward itself.
    Gating, filtering, step limits and the near-hand freeze stay v4's.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        with SORT.lock:
            self.obj_index = SORT.active_object

    def _candidate_poses(self) -> Tuple[float, List[np.ndarray]]:
        if self.obj_index is None:
            return super()._candidate_poses()
        snap = _scene_snapshot()
        return snap["detection_ts"], [
            e["det"].T_camera_object.copy()
            for e in _classify(snap)
            if e["obj"] == self.obj_index and e["sortable"]
        ]


def _wait_for_candidate(obj_index: int, prefer_centre: bool = False) -> Optional[Any]:
    # v4 holds detections for DETECTION_HOLD_S after they disappear; wait for
    # a perception pass newer than that so objects that just moved (e.g. the
    # one that was just placed) are not reported at their old position.
    min_ts = time.time() + rec.DETECTION_HOLD_S
    deadline = min_ts + CANDIDATE_WAIT_S
    while time.time() < deadline:
        if SORT_STOP_EVENT.is_set():
            return None
        det = _pick_candidate(obj_index, min_ts, prefer_centre)
        if det is not None:
            return det
        time.sleep(0.2)
    return None


# ---------------------------------------------------------------------------
# Motion primitives (built on v4's)
# ---------------------------------------------------------------------------

def _acquire_arm_lock(timeout_s: float = 10.0) -> bool:
    if rec.ARM_CONTROL_LOCK.acquire(timeout=timeout_s):
        return True
    _log("Arm controller stayed busy; aborting this step.")
    return False


def _move_waist(target_yaw: float, max_speed_rad_s: float) -> None:
    """Turn the waist; caller holds ARM_CONTROL_LOCK.

    Turning the waist moves the camera relative to the arm base frame, so the
    view epoch is advanced and pattern poses measured before the turn (or
    during it) are no longer used for placement.
    """
    with SORT.lock:
        SORT.view_epoch += 1
        SORT.view_settle_until = float("inf")
    try:
        rec._stop_active_hold()
        rec._ensure_arm_authority(rec._CONTROL_ROBOT, duration_s=0.3)
        speed = min(rec.DEFAULT_CENTER_WAIST_SPEED_RAD_S, max(0.05, float(max_speed_rad_s)))
        result = rec._CONTROL_ROBOT.move_upper_body_joint(
            joint_index=rec.WAIST_YAW_JOINT_INDEX,
            target=float(target_yaw),
            max_speed_rad_s=speed,
            timeout=2.0,
        )
        _log(f"waist yaw move result: {result}")
    finally:
        with SORT.lock:
            SORT.view_settle_until = time.time() + VIEW_SETTLE_S


def _center_waist_on(det, max_speed_rad_s: float) -> bool:
    p_cam = det.T_camera_object[:3, 3]
    if float(p_cam[2]) <= 0.05:
        return False
    yaw_delta = -math.atan2(float(p_cam[0]), float(p_cam[2]))
    yaw_delta = max(-rec.DEFAULT_CENTER_WAIST_MAX_YAW_RAD, min(rec.DEFAULT_CENTER_WAIST_MAX_YAW_RAD, yaw_delta))
    if abs(yaw_delta) < rec.DEFAULT_CENTER_WAIST_DEADBAND_RAD:
        return False
    if not _acquire_arm_lock():
        return False
    try:
        with rec.STATE.lock:
            rec.STATE.arm_motion_running = True
            rec.STATE.arm_motion_label = "waist center"
        current = rec._read_joint_position(rec._CONTROL_ROBOT, rec.WAIST_YAW_JOINT_INDEX)
        _log(f"Centering target with waist yaw delta {yaw_delta:+.3f} rad.")
        _move_waist(current + yaw_delta, max_speed_rad_s)
        return True
    except Exception as exc:
        _log(f"Waist centering failed; continuing without it: {exc}")
        return False
    finally:
        with rec.STATE.lock:
            rec.STATE.arm_motion_running = False
            rec.STATE.arm_motion_label = ""
        rec.ARM_CONTROL_LOCK.release()


def _clear_hold_flags() -> None:
    with rec.STATE.lock:
        rec.STATE.lift_confirmation_pending = False
        rec.STATE.lift_arm = None
        rec.STATE.lift_unverified = False
        rec.STATE.holding_object = False
        rec.STATE.holding_arm = None


def _abandon_grasp() -> None:
    """Open the hand at the grasp pose and retract to the stable pose."""
    try:
        rec._send_hand_action("open", SORT_ARM)
    except Exception as exc:
        _log(f"Opening hand after rejected grasp failed: {exc}")
    _clear_hold_flags()
    rec._run_retract_stable_pose()


def _grab_object(obj_index: int, obj: ObjectConfig, det, options: Dict[str, Any]) -> str:
    """Pick one object and lift it to the WBC stable pose.

    Returns "held", "failed" (nothing in hand; try again), "unreachable",
    or "abort" (an object may be in hand or the sort was stopped).
    """
    with rec.STATE.lock:
        rec.STATE.object_mass_kg = obj.mass_kg
        rec.STATE.arm_override = SORT_ARM
        rec.STATE.selected_id = det.id
        max_step = rec.STATE.max_joint_step_rad
        max_speed = rec.STATE.max_joint_speed_rad_s
        # Placement leaves the arm at the WBC stable pose, not the extend
        # pose; forget the saved extend reference so the grab preflight runs
        # the full, table-aware extend recipe again before every pick.
        rec.STATE.extended_hand_R_base.pop(SORT_ARM, None)
        rec.STATE.extended_hand_reference_q.pop(SORT_ARM, None)
        rec.STATE.extended_hand_reference_T_base.pop(SORT_ARM, None)
        rec.STATE.grab_running = True
    with SORT.lock:
        SORT.current_target = (obj_index, det.box)
        SORT.active_object = obj_index
    try:
        if options["center_waist"] and _center_waist_on(det, max_speed):
            refreshed = _wait_for_candidate(obj_index, prefer_centre=True)
            if refreshed is None:
                _log(f"{obj.name} not re-detected after waist centering.")
                return "abort" if SORT_STOP_EVENT.is_set() else "failed"
            det = refreshed
            with SORT.lock:
                SORT.current_target = (obj_index, det.box)
        with rec.STATE.lock:
            cam = dict(rec.STATE.camera_extrinsic)
            rec.STATE.selected_id = det.id
        arm, _T_base_camera, T_base_object = rec._resolve_arm_and_base_pose(det, cam, SORT_ARM)
        _arm, reach, max_reach, excess, _step = rec._reach_preview(det, cam, SORT_ARM)
        p = T_base_object[:3, 3]
        _log(
            f"Target {obj.name}: label={det.label!r} score={det.score:.2f} "
            f"base xyz=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m reach={reach:.3f}/{max_reach:.3f} m"
        )
        if excess > 0.0:
            _log(f"{obj.name} is {excess:.3f} m outside arm reach; skipping it.")
            return "unreachable"
        if not rec._prepare_hand_visibility_for_grab(arm, cam, max_step, max_speed):
            return "abort" if SORT_STOP_EVENT.is_set() else "failed"
        if SORT_STOP_EVENT.is_set():
            return "abort"
        rec._run_direct_grab_inline(
            det, arm, cam, False, T_base_object, max_step, max_speed, mass_kg=obj.mass_kg,
        )
    finally:
        with rec.STATE.lock:
            rec.STATE.grab_running = False
        with SORT.lock:
            SORT.current_target = None
            SORT.active_object = None

    with rec.STATE.lock:
        grasped = rec.STATE.lift_confirmation_pending and rec.STATE.holding_object
        unverified = rec.STATE.lift_unverified
    if SORT_STOP_EVENT.is_set():
        return "abort"
    if not grasped:
        _log(f"Grab of {obj.name} did not end with a closed hand.")
        return "failed"
    if unverified and not options["allow_unverified_grasp"]:
        _log(f"Grasp of {obj.name} is not tactile-verified; releasing and retrying.")
        _abandon_grasp()
        return "abort" if SORT_STOP_EVENT.is_set() else "failed"

    _set_phase(f"Lifting {obj.name} to the stable pose")
    rec._run_confirm_lift()
    with rec.STATE.lock:
        lifted = not rec.STATE.lift_confirmation_pending
    if not lifted:
        _log(f"Lift of {obj.name} failed; stopping with the object held. Status: {rec.STATE.status_msg}")
        return "abort"
    return "held"


def _place_held_object(obj: ObjectConfig, pattern_index: int, pattern: PatternConfig, options: Dict[str, Any]) -> bool:
    """Place the object held in the right hand on a pattern and retract."""
    if not _acquire_arm_lock():
        return False
    try:
        with rec.STATE.lock:
            rec.STATE.arm_motion_running = True
            rec.STATE.arm_motion_label = f"place on {pattern.name}"
            max_step = rec.STATE.max_joint_step_rad
            max_speed = rec.STATE.max_joint_speed_rad_s
            cam = dict(rec.STATE.camera_extrinsic)
        robot = rec._CONTROL_ROBOT
        if robot is None:
            raise RuntimeError(rec._CONTROL_STATUS)

        start_yaw = options.get("start_waist_yaw")
        if start_yaw is not None:
            current_yaw = rec._read_joint_position(robot, rec.WAIST_YAW_JOINT_INDEX)
            if abs(current_yaw - start_yaw) >= rec.DEFAULT_CENTER_WAIST_DEADBAND_RAD:
                _log(f"Returning waist to the sort start yaw ({start_yaw:+.3f} rad) to view {pattern.name}.")
                _move_waist(start_yaw, max_speed)
                with SORT.lock:
                    settle = SORT.view_settle_until
                if not _wait_for_pattern(pattern_index, settle):
                    raise RuntimeError(f"{pattern.name} is not visible after returning the waist")

        T_camera_pattern = _pattern_T_camera(pattern_index, require_current_epoch=True)
        if T_camera_pattern is None:
            raise RuntimeError(f"no current pose for {pattern.name}; keep the pattern in camera view")
        centre, normal = _pattern_drop_geometry(pattern, T_camera_pattern, cam)
        release = centre + normal * pattern.release_height_m
        approach = centre + normal * (pattern.release_height_m + PLACE_APPROACH_EXTRA_M)
        _log(
            f"{pattern.name} centre base xyz=({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f}) m; "
            f"release {pattern.release_height_m * 100:.1f} cm above it."
        )

        with rec.ACTIVE_NAV_LOCK:
            nav = rec.ACTIVE_DIRECT_NAV
        if nav is not None:
            rec._shutdown_direct_nav(nav, "before sort placement")
        rec._stop_active_hold()
        obstacles = rec._estimate_obstacles(cam)
        lift = rec._lift_end_effector_z_slowly(
            robot, SORT_ARM, max_step, max_speed, mass_kg=obj.mass_kg, obstacles=obstacles,
        )
        if not lift.get("success"):
            raise RuntimeError(f"clearance lift failed: {lift}")
        for label, point in ((f"{pattern.name} approach", approach), (f"{pattern.name} release", release)):
            if SORT_STOP_EVENT.is_set():
                raise RuntimeError("sort stopped")
            result = rec._move_ee_to_position(robot, SORT_ARM, point, max_step, max_speed, obstacles, label)
            _log(f"{label} result: success={result.get('success')} reason={result.get('reason', '')}")
            if not result.get("success"):
                raise RuntimeError(f"{label} failed: {result.get('reason')}")
        rec._send_hand_action("open", SORT_ARM)
        _clear_hold_flags()
        _log(f"Released {obj.name} on {pattern.name}.")
        # Rise straight up before swinging back so the fingers clear the object.
        retreat = rec._move_ee_to_position(
            robot, SORT_ARM, approach, max_step, max_speed, obstacles, f"{pattern.name} retreat",
        )
        if not retreat.get("success"):
            _log(f"Retreat above {pattern.name} failed ({retreat.get('reason')}); retracting directly.")
        stable = rec._move_to_wbc_right_arm_forward(robot, max_step, max_speed, obstacles)
        _log(f"Return to WBC stable pose: success={stable.get('success')} reason={stable.get('reason', '')}")
        return True
    except Exception as exc:
        _log(f"Placement on {pattern.name} failed: {exc}")
        with rec.STATE.lock:
            rec.STATE.status_msg = f"Placement on {pattern.name} failed: {exc}"
        return False
    finally:
        with rec.STATE.lock:
            rec.STATE.arm_motion_running = False
            rec.STATE.arm_motion_label = ""
        rec.ARM_CONTROL_LOCK.release()


# ---------------------------------------------------------------------------
# Sort sequence
# ---------------------------------------------------------------------------

def _run_sort() -> None:
    if not SORT_RUN_LOCK.acquire(blocking=False):
        _log("Sort already running.")
        return
    try:
        SORT_STOP_EVENT.clear()
        with SORT.lock:
            patterns = list(SORT.patterns)
            objects = list(SORT.objects)
            errors = list(SORT.config_errors)
            options = {
                "center_waist": SORT.center_waist,
                "allow_unverified_grasp": SORT.allow_unverified_grasp,
                "start_waist_yaw": None,
            }
            SORT.running = True
            SORT.sorted_counts = [0] * OBJECT_COUNT
            SORT.summary = ""
        if errors:
            _set_phase("Sort refused: fix the configuration first")
            for error in errors:
                _log(error)
            return
        if rec._CONTROL_ROBOT is None:
            _set_phase(f"Sort refused: {rec._CONTROL_STATUS}")
            return
        with rec.STATE.lock:
            busy = rec.STATE.grab_running or rec.STATE.arm_motion_running or rec.STATE.holding_object
            rec.STATE.arm_override = SORT_ARM
        if busy:
            _set_phase("Sort refused: the arm is busy or already holding an object")
            return

        _set_phase("Checking that both drop-off patterns are visible")
        t_check = time.time()
        for i, pattern in enumerate(patterns):
            if not _wait_for_pattern(i, t_check):
                _set_phase(f"Sort refused: {pattern.name} ({pattern.describe()}) is not visible")
                return
        if options["center_waist"]:
            try:
                options["start_waist_yaw"] = rec._read_joint_position(rec._CONTROL_ROBOT, rec.WAIST_YAW_JOINT_INDEX)
            except Exception as exc:
                _log(f"Cannot read waist yaw ({exc}); sorting without waist centering.")
                options["center_waist"] = False

        plan = "; ".join(
            f"{obj.count} x {obj.name} -> {patterns[obj.zone].name}" for obj in objects
        )
        _log(f"Sort started: {plan}.")
        sorted_counts = [0] * OBJECT_COUNT
        failures = [0] * OBJECT_COUNT
        finished = [False] * OBJECT_COUNT
        notes: List[str] = []
        aborted = False
        while not aborted and not all(finished):
            for i, obj in enumerate(objects):
                if finished[i]:
                    continue
                if SORT_STOP_EVENT.is_set():
                    aborted = True
                    break
                if sorted_counts[i] >= obj.count:
                    finished[i] = True
                    continue
                _set_phase(f"Looking for {obj.name} ({sorted_counts[i] + 1}/{obj.count})")
                det = _wait_for_candidate(i)
                if det is None:
                    if SORT_STOP_EVENT.is_set():
                        aborted = True
                        break
                    notes.append(f"no more {obj.name} found ({sorted_counts[i]}/{obj.count} sorted)")
                    finished[i] = True
                    continue
                _set_phase(f"Picking {obj.name} ({sorted_counts[i] + 1}/{obj.count})")
                outcome = _grab_object(i, obj, det, options)
                if outcome == "abort":
                    aborted = True
                    break
                if outcome == "unreachable":
                    notes.append(f"{obj.name} out of reach")
                    finished[i] = True
                    continue
                if outcome == "failed":
                    failures[i] += 1
                    if failures[i] >= MAX_FAILED_PICKS_PER_OBJECT:
                        notes.append(f"gave up on {obj.name} after {failures[i]} failed picks")
                        finished[i] = True
                    continue
                pattern = patterns[obj.zone]
                _set_phase(f"Placing {obj.name} on {pattern.name}")
                if not _place_held_object(obj, obj.zone, pattern, options):
                    notes.append(f"placement of {obj.name} on {pattern.name} failed")
                    aborted = True
                    break
                sorted_counts[i] += 1
                with SORT.lock:
                    SORT.sorted_counts = list(sorted_counts)

        start_yaw = options.get("start_waist_yaw")
        if start_yaw is not None and not SORT_STOP_EVENT.is_set() and not rec.STATE.holding_object:
            if _acquire_arm_lock():
                try:
                    current = rec._read_joint_position(rec._CONTROL_ROBOT, rec.WAIST_YAW_JOINT_INDEX)
                    if abs(current - start_yaw) >= rec.DEFAULT_CENTER_WAIST_DEADBAND_RAD:
                        _move_waist(start_yaw, rec.STATE.max_joint_speed_rad_s)
                except Exception as exc:
                    _log(f"Returning waist to start yaw failed: {exc}")
                finally:
                    rec.ARM_CONTROL_LOCK.release()

        result = ", ".join(
            f"{obj.name} {sorted_counts[i]}/{obj.count}" for i, obj in enumerate(objects)
        )
        if SORT_STOP_EVENT.is_set():
            headline = "Sort stopped"
        elif aborted:
            headline = "Sort aborted"
        else:
            headline = "Sort finished"
        summary = f"{headline}: {result}" + (f" ({'; '.join(notes)})" if notes else "")
        with SORT.lock:
            SORT.summary = summary
        if rec.STATE.holding_object:
            summary += " — an object may still be in the hand; use Open hand / Retract."
        _set_phase(summary)
    except Exception as exc:
        _set_phase(f"Sort failed: {exc!r}")
    finally:
        with SORT.lock:
            SORT.running = False
            SORT.current_target = None
        SORT_RUN_LOCK.release()


def _run_stop_sort() -> None:
    SORT_STOP_EVENT.set()
    _log("Stop sorting requested.")
    rec._run_stop_grabbing()


def _run_hand_open() -> None:
    with rec.STATE.lock:
        rec.STATE.arm_override = SORT_ARM
    rec._run_hand_action("open")


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def _draw_label(image: np.ndarray, text: str, origin: Tuple[int, int], color: Tuple[int, int, int]) -> None:
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = origin
    y0 = max(0, y - th - 6)
    cv2.rectangle(image, (x, y0), (x + tw + 6, y0 + th + 6), color, -1)
    cv2.putText(image, text, (x + 3, y0 + th + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def _draw_sort_view(image: np.ndarray, snap: Dict[str, Any], entries: List[Dict[str, Any]]) -> np.ndarray:
    out = image.copy()
    patterns: List[PatternConfig] = snap["patterns"]
    objects: List[ObjectConfig] = snap["objects"]
    with SORT.lock:
        markers = list(SORT.pattern_markers)
        current_target = SORT.current_target
    for corners, ids in markers:
        try:
            cv2.aruco.drawDetectedMarkers(out, corners, ids)
        except Exception:
            pass
    for i, pattern in enumerate(patterns):
        T = _pattern_T_camera(i, require_current_epoch=False)
        if T is None:
            continue
        color = PATTERN_COLORS_BGR[i]
        outline = [
            np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([pattern.width_m, 0.0, 0.0, 1.0]),
            np.array([pattern.width_m, pattern.height_m, 0.0, 1.0]),
            np.array([0.0, pattern.height_m, 0.0, 1.0]),
        ]
        pts = [rec._project_camera_point((T @ p)[:3]) for p in outline]
        if all(p is not None for p in pts):
            cv2.polylines(out, [np.array(pts, dtype=np.int32)], True, color, 2, cv2.LINE_AA)
        centre_uv = rec._project_camera_point((T @ pattern.centre_pattern())[:3])
        if centre_uv is not None:
            cv2.circle(out, centre_uv, 5, color, -1, cv2.LINE_AA)
            owners = [obj.name for obj in objects if obj.zone == i]
            text = f"{PATTERN_LETTERS[i]}: {pattern.name}" + (f" <- {', '.join(owners)}" if owners else "")
            _draw_label(out, text, (centre_uv[0] + 8, centre_uv[1] + 22), color)
    for entry in entries:
        det = entry["det"]
        if det.box is None:
            continue
        x0, y0, x1, y1 = (int(v) for v in det.box)
        if entry["obj"] is None:
            color, text = IGNORED_COLOR_BGR, f"{det.label} (ignored)"
        elif not entry["sortable"]:
            color = SORTED_COLOR_BGR
            text = f"{objects[entry['obj']].name}: sorted"
        else:
            color = OBJECT_COLORS_BGR[entry["obj"]]
            text = f"{objects[entry['obj']].name} -> {PATTERN_LETTERS[objects[entry['obj']].zone]}"
        is_target = current_target is not None and tuple(current_target[1] or ()) == tuple(det.box)
        cv2.rectangle(out, (x0, y0), (x1, y1), color, 4 if is_target else 2)
        _draw_label(out, f"{text} {det.score:.2f}" + ("  [TARGET]" if is_target else ""), (x0, y0), color)
    return out


# ---------------------------------------------------------------------------
# Dash app
# ---------------------------------------------------------------------------

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.DARKLY])
app.title = "G1 Sorting"

PATTERN_FIELDS = ("name", "dict", "layout", "id", "size", "rows", "cols", "sep", "height")
OBJECT_FIELDS = ("prompt", "color", "mass", "count", "zone")
_SMALL = {"fontSize": "12px"}


def _labeled(label: str, component, width: int = 6) -> dbc.Col:
    return dbc.Col([dbc.Label(label, className="mb-0", style=_SMALL), component], width=width, className="mb-2")


def _pattern_card(i: int) -> dbc.Card:
    p = DEFAULT_PATTERNS[i]
    letter = PATTERN_LETTERS[i]
    return dbc.Card([
        dbc.CardHeader(f"Drop-off pattern {letter}", style={"color": "#%02x%02x%02x" % PATTERN_COLORS_BGR[i][::-1]}),
        dbc.CardBody([
            dbc.Row([
                _labeled("Name", dbc.Input(id=f"pat{i}-name", value=p.name, debounce=True, size="sm")),
                _labeled("Dictionary", dcc.Dropdown(
                    id=f"pat{i}-dict", value=p.dictionary, clearable=False,
                    options=[{"label": name, "value": name} for name in ARUCO_DICTIONARIES],
                    style={"color": "#111", "fontSize": "12px"},
                )),
            ]),
            dbc.RadioItems(
                id=f"pat{i}-layout", value=p.layout, inline=True, className="mb-2", style=_SMALL,
                options=[{"label": "Single marker", "value": "single"}, {"label": "Grid board", "value": "grid"}],
            ),
            dbc.Row([
                _labeled("Marker ID (first ID for a grid)", dbc.Input(
                    id=f"pat{i}-id", type="number", min=0, step=1, value=p.first_id, debounce=True, size="sm")),
                _labeled("Marker size (mm, black square)", dbc.Input(
                    id=f"pat{i}-size", type="number", min=5, step=1, value=p.marker_length_m * 1000.0,
                    debounce=True, size="sm")),
            ]),
            dbc.Row([
                _labeled("Rows", dbc.Input(
                    id=f"pat{i}-rows", type="number", min=1, step=1, value=p.rows, debounce=True, size="sm"), 4),
                _labeled("Columns", dbc.Input(
                    id=f"pat{i}-cols", type="number", min=1, step=1, value=p.cols, debounce=True, size="sm"), 4),
                _labeled("Separation (mm)", dbc.Input(
                    id=f"pat{i}-sep", type="number", min=0, step=1, value=p.separation_m * 1000.0,
                    debounce=True, size="sm"), 4),
            ]),
            dbc.Row([
                _labeled("Release height above pattern (cm)", dbc.Input(
                    id=f"pat{i}-height", type="number", min=MIN_RELEASE_HEIGHT_M * 100,
                    max=MAX_RELEASE_HEIGHT_M * 100, step=0.5, value=p.release_height_m * 100.0,
                    debounce=True, size="sm"), 12),
            ]),
            html.Div(id=f"pat{i}-status", style=_SMALL),
        ]),
    ], className="mb-3")


def _object_card(i: int) -> dbc.Card:
    o = DEFAULT_OBJECTS[i]
    return dbc.Card([
        dbc.CardHeader(f"Object {i + 1}", style={"color": "#%02x%02x%02x" % OBJECT_COLORS_BGR[i][::-1]}),
        dbc.CardBody([
            dbc.Row([
                _labeled("Recognition prompt", dbc.Input(
                    id=f"obj{i}-prompt", value=o.prompt, debounce=True, size="sm",
                    placeholder="e.g. cube, soda can, tennis ball"), 8),
                _labeled("Colour filter", dcc.Dropdown(
                    id=f"obj{i}-color", value=o.color, clearable=False,
                    options=[{"label": c, "value": c} for c in COLOR_FILTERS],
                    style={"color": "#111", "fontSize": "12px"},
                ), 4),
            ]),
            dbc.Row([
                _labeled("Mass (kg)", dbc.Input(
                    id=f"obj{i}-mass", type="number", min=0, max=rec.MAX_OBJECT_MASS_KG, step=0.05,
                    value=o.mass_kg, debounce=True, size="sm"), 4),
                _labeled("How many", dbc.Input(
                    id=f"obj{i}-count", type="number", min=1, max=MAX_ITEMS_PER_OBJECT, step=1,
                    value=o.count, debounce=True, size="sm"), 4),
                _labeled("Drop-off spot", dcc.Dropdown(
                    id=f"obj{i}-zone", value=o.zone, clearable=False,
                    options=[{"label": f"Pattern {letter}", "value": k} for k, letter in enumerate(PATTERN_LETTERS)],
                    style={"color": "#111", "fontSize": "12px"},
                ), 4),
            ]),
        ]),
    ], className="mb-3")


def _options_card() -> dbc.Card:
    return dbc.Card(dbc.CardBody([
        html.H5("Motion options"),
        dbc.Row([
            _labeled("Max arm joint increment per command (rad)", dbc.Input(
                id="max-joint-step-rad", type="number", min=0.005, max=rec.ABSOLUTE_MAX_JOINT_STEP_RAD,
                step=0.005, value=rec.STATE.max_joint_step_rad, size="sm")),
            _labeled("Max arm joint speed (rad/s)", dbc.Input(
                id="max-joint-speed-rad-s", type="number", min=0.02, max=rec.ABSOLUTE_MAX_JOINT_SPEED_RAD_S,
                step=0.01, value=rec.STATE.max_joint_speed_rad_s, size="sm")),
        ]),
        dbc.Checkbox(id="dynamic-target-correction", value=rec.STATE.dynamic_target_correction,
                     className="mb-1",
                     label="Correct the grab target from live detections during the approach"),
        dbc.Checkbox(id="center-waist", value=False, className="mb-1",
                     label="Turn the waist to centre each object before grabbing"),
        dbc.Checkbox(id="allow-unverified", value=False, className="mb-1",
                     label="Carry objects even when tactile sensors don't confirm the grasp"),
        html.Small(
            "Sorting uses the right arm and never steps the base: keep all objects and both "
            "patterns within reach and in camera view.",
            className="text-muted",
        ),
    ]), className="mb-3")


app.layout = dbc.Container([
    html.H3("G1 Sorting", className="mt-3"),
    html.Div(id="global-status", className="mb-2", style={"fontSize": "13px", "color": "#9c9"}),

    dbc.Row([dbc.Col(_pattern_card(i), width=6) for i in range(PATTERN_COUNT)]),
    dbc.Row([dbc.Col(_object_card(i), width=6) for i in range(OBJECT_COUNT)]),
    html.Div(id="config-status", className="mb-3", style={"fontSize": "13px"}),

    dbc.Row([
        dbc.Col(dbc.Card([
            dbc.CardHeader(dbc.Row([
                dbc.Col(html.Div("Live view", style={"fontSize": "13px"}), width=3),
                dbc.Col(dcc.Dropdown(
                    id="view-select", value="sort", clearable=False, style={"color": "#111"},
                    options=[
                        {"label": "Sorting overlay", "value": "sort"},
                        {"label": "RGB raw", "value": "rgb"},
                        {"label": "Depth", "value": "depth"},
                        {"label": "Segmentation", "value": "segmentation"},
                        {"label": "Desk / obstacle segmentation", "value": "desk-obstacles"},
                    ],
                ), width=9),
            ], align="center")),
            dbc.CardBody(html.Img(id="panel-view", style={"width": "100%"})),
        ], className="mb-3"), width=6),
        dbc.Col([
            dbc.Card(dbc.CardBody([
                html.H5("Sort"),
                dbc.Button("Sort", id="sort-btn", color="success", className="me-2"),
                dbc.Button("Stop sorting", id="stop-sort-btn", color="danger"),
                html.Div(id="sort-phase", className="mt-2", style={"fontSize": "13px", "color": "#fd8"}),
                html.Div(id="sort-scene", className="mt-2", style=_SMALL),
                html.Hr(),
                dbc.ButtonGroup([
                    dbc.Button("Retract to stable pose", id="retract-stable-btn", color="secondary"),
                    dbc.Button("Open hand", id="hand-open-btn", color="info"),
                    dbc.Button("Release arms", id="release-arms-btn", color="warning"),
                    dbc.Button("Re-engage arms", id="unrelease-arms-btn", color="primary"),
                    dbc.Button("Damp", id="damp-btn", color="danger"),
                ], size="sm"),
                html.Div(id="safety-status", className="mt-2", style=_SMALL),
                html.Pre(id="tactile-status", className="mt-2", style={
                    "fontSize": "11px", "color": "#ada", "marginBottom": 0, "whiteSpace": "pre-wrap",
                }),
            ]), className="mb-3"),
            _options_card(),
        ], width=6),
    ]),

    dbc.Card([
        dbc.CardHeader("Log", style={"fontSize": "13px"}),
        dbc.CardBody(html.Pre(id="grab-log", style={
            "maxHeight": "460px", "overflow": "auto", "fontSize": "11px",
            "background": "#111", "padding": "8px", "marginBottom": 0,
        })),
    ], className="mb-3"),

    dcc.Interval(id="refresh-interval", interval=500, n_intervals=0),
], fluid=True)


# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------

@app.callback(
    Output("config-status", "children"),
    [Input(f"pat{i}-{f}", "value") for i in range(PATTERN_COUNT) for f in PATTERN_FIELDS]
    + [Input(f"obj{i}-{f}", "value") for i in range(OBJECT_COUNT) for f in OBJECT_FIELDS],
)
def _update_config(*values):
    with SORT.lock:
        if SORT.running:
            return html.Span("Sort running — configuration changes apply to the next sort.", className="text-warning")
    n_pat = len(PATTERN_FIELDS)
    patterns, objects, errors = [], [], []
    for i in range(PATTERN_COUNT):
        fields = dict(zip(PATTERN_FIELDS, values[i * n_pat:(i + 1) * n_pat]))
        pattern, pattern_errors = _parse_pattern(fields, DEFAULT_PATTERNS[i], PATTERN_LETTERS[i])
        patterns.append(pattern)
        errors.extend(pattern_errors)
    offset = PATTERN_COUNT * n_pat
    n_obj = len(OBJECT_FIELDS)
    for i in range(OBJECT_COUNT):
        fields = dict(zip(OBJECT_FIELDS, values[offset + i * n_obj:offset + (i + 1) * n_obj]))
        obj, obj_errors = _parse_object(fields, i)
        objects.append(obj)
        errors.extend(obj_errors)
    errors.extend(_validate_pair(patterns, objects))
    _apply_config(patterns, objects, errors)
    if errors:
        return html.Ul([html.Li(e) for e in errors], className="text-danger mb-0")
    lines = [
        f"Pattern {PATTERN_LETTERS[i]} ({p.name}): {p.describe()}" for i, p in enumerate(patterns)
    ] + [
        f"Object {i + 1}: {o.count} x {o.name} (classes: {', '.join(o.classes)}; {o.mass_kg:.2f} kg) "
        f"-> {patterns[o.zone].name}"
        for i, o in enumerate(objects)
    ]
    return html.Div([html.Div(line) for line in lines], className="text-success")


for _i in range(PATTERN_COUNT):
    @app.callback(
        Output(f"pat{_i}-rows", "disabled"),
        Output(f"pat{_i}-cols", "disabled"),
        Output(f"pat{_i}-sep", "disabled"),
        Input(f"pat{_i}-layout", "value"),
    )
    def _toggle_grid_fields(layout):
        single = layout != "grid"
        return single, single, single


@app.callback(
    Output("max-joint-step-rad", "value"),
    Output("max-joint-speed-rad-s", "value"),
    Output("center-waist", "value"),
    Output("allow-unverified", "value"),
    Output("dynamic-target-correction", "value"),
    Input("max-joint-step-rad", "value"),
    Input("max-joint-speed-rad-s", "value"),
    Input("center-waist", "value"),
    Input("allow-unverified", "value"),
    Input("dynamic-target-correction", "value"),
)
def _sync_options(step, speed, center_waist, allow_unverified, dynamic_correction):
    step = max(0.005, min(rec.ABSOLUTE_MAX_JOINT_STEP_RAD, _to_float(step, rec.DEFAULT_MAX_JOINT_STEP_RAD)))
    speed = max(0.02, min(rec.ABSOLUTE_MAX_JOINT_SPEED_RAD_S, _to_float(speed, rec.DEFAULT_MAX_JOINT_SPEED_RAD_S)))
    with rec.STATE.lock:
        rec.STATE.max_joint_step_rad = step
        rec.STATE.max_joint_speed_rad_s = speed
        rec.STATE.dynamic_target_correction = bool(dynamic_correction)
    with SORT.lock:
        SORT.center_waist = bool(center_waist)
        SORT.allow_unverified_grasp = bool(allow_unverified)
    return step, speed, bool(center_waist), bool(allow_unverified), bool(dynamic_correction)


@app.callback(
    Output("panel-view", "src"),
    Output("global-status", "children"),
    Output("pat0-status", "children"),
    Output("pat1-status", "children"),
    Output("sort-phase", "children"),
    Output("sort-scene", "children"),
    Output("sort-btn", "disabled"),
    Output("grab-log", "children"),
    Output("tactile-status", "children"),
    Input("refresh-interval", "n_intervals"),
    Input("view-select", "value"),
)
def _refresh(_n, view_name):
    snap = _scene_snapshot()
    with rec.STATE.lock:
        rgb = None if rec.STATE.rgb_bgr is None else rec.STATE.rgb_bgr.copy()
        depth = None if rec.STATE.depth_m is None else rec.STATE.depth_m.copy()
        masks = list(rec.STATE.masks_for_draw)
        status_msg = rec.STATE.status_msg
        log = "\n".join(rec.STATE.grab_log[-120:])
        arm_busy = rec.STATE.grab_running or rec.STATE.arm_motion_running
        arm_label = rec.STATE.arm_motion_label
    with SORT.lock:
        running = SORT.running
        phase = SORT.phase
        sorted_counts = list(SORT.sorted_counts)
        pattern_status = list(SORT.pattern_status)
        has_errors = bool(SORT.config_errors)
    patterns: List[PatternConfig] = snap["patterns"]
    objects: List[ObjectConfig] = snap["objects"]
    tactile = rec._tactile_status_text()

    entries = _classify(snap)
    if time.time() - snap["frame_ts"] > rec.STALE_AFTER_S:
        status_msg = f"STALE — no fresh frame in >{rec.STALE_AFTER_S:.0f}s. {status_msg}"
    if arm_busy and arm_label:
        status_msg = f"{status_msg} | arm busy: {arm_label}"

    pattern_divs = []
    for i, pattern in enumerate(patterns):
        text = f"{pattern.describe()} — {pattern_status[i]}"
        T = _pattern_T_camera(i, require_current_epoch=False)
        if T is not None:
            centre, _normal = _pattern_drop_geometry(pattern, T, snap["cam"])
            text += f" | centre base xyz=({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f}) m"
        visible = pattern_status[i].startswith("visible")
        pattern_divs.append(html.Span(text, className="text-success" if visible else "text-warning"))

    if running:
        phase_text = phase + " | progress: " + ", ".join(
            f"{obj.name} {sorted_counts[i]}/{obj.count}" for i, obj in enumerate(objects)
        )
    else:
        phase_text = phase

    scene_lines = []
    for i, obj in enumerate(objects):
        mine = [e for e in entries if e["obj"] == i]
        todo = sum(1 for e in mine if e["sortable"])
        done = len(mine) - todo
        scene_lines.append(html.Div(
            f"Object {i + 1} ({obj.name} -> {patterns[obj.zone].name}): "
            f"{todo} to sort, {done} already on its spot",
        ))
    ignored = [e["det"].label for e in entries if e["obj"] is None]
    if ignored:
        scene_lines.append(html.Div(
            f"Ignored (colour filter / no match): {', '.join(ignored)}", className="text-muted",
        ))

    if rgb is None:
        view_src = rec._placeholder_src("waiting for camera…")
    elif view_name == "rgb":
        view_src = rec._encode_jpeg_src(rgb)
    elif view_name == "depth":
        view_src = rec._encode_jpeg_src(rec._colorize_depth(depth)) if depth is not None else rec._placeholder_src("no depth")
    elif view_name == "segmentation":
        view_src = rec._encode_jpeg_src(draw_segmentation_overlay(rgb, masks)) if masks else rec._placeholder_src("no masks")
    elif view_name == "desk-obstacles":
        view_src = rec._encode_jpeg_src(rec._draw_desk_obstacle_segmentation(rgb, depth, snap["cam"], masks))
    else:
        view_src = rec._encode_jpeg_src(_draw_sort_view(rgb, snap, entries))

    sort_disabled = running or arm_busy or has_errors
    return (
        view_src, status_msg, pattern_divs[0], pattern_divs[1], phase_text, scene_lines,
        sort_disabled, log, tactile,
    )


@app.callback(
    Output("safety-status", "children"),
    Input("sort-btn", "n_clicks"),
    Input("stop-sort-btn", "n_clicks"),
    Input("retract-stable-btn", "n_clicks"),
    Input("hand-open-btn", "n_clicks"),
    Input("release-arms-btn", "n_clicks"),
    Input("unrelease-arms-btn", "n_clicks"),
    Input("damp-btn", "n_clicks"),
    prevent_initial_call=True,
)
def _buttons(*_clicks):
    ctx = callback_context
    if not ctx.triggered:
        return no_update
    trigger = ctx.triggered[0]["prop_id"].split(".")[0]
    actions = {
        "sort-btn": (_run_sort, "Sort started — see log below."),
        "stop-sort-btn": (_run_stop_sort, "Stopping sort; holding the current arm pose..."),
        "retract-stable-btn": (rec._run_retract_stable_pose, "Retracting right arm to stable pose..."),
        "hand-open-btn": (_run_hand_open, "Opening right hand..."),
        "release-arms-btn": (rec._run_release_arms, "Releasing arms…"),
        "unrelease-arms-btn": (rec._run_unrelease_arms, "Re-engaging arms..."),
        "damp-btn": (rec._run_damp, "Damping…"),
    }
    if trigger not in actions:
        return no_update
    if trigger in ("release-arms-btn", "damp-btn"):
        SORT_STOP_EVENT.set()
    target, message = actions[trigger]
    threading.Thread(target=target, daemon=True).start()
    return message


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

with rec.STATE.lock:
    rec.STATE.arm_override = SORT_ARM
    rec.STATE.max_visible_detections = 50
_apply_config(list(DEFAULT_PATTERNS), list(DEFAULT_OBJECTS), [])
# v4's grab loop constructs its corrector through this module-level name at
# grab time, so rebinding it here makes every grab in this process match
# candidates the way the sort does.
rec._GrabTargetCorrector = _SortTargetCorrector

threading.Thread(target=_pattern_loop, args=(6.0,), daemon=True).start()

if __name__ == "__main__":
    app.run(host=rec._ARGS.host, port=rec._ARGS.port, debug=rec._ARGS.debug)
