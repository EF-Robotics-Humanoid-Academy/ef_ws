"""Sorting layer for the dashboard's /sort tab -- pick two kinds of objects and
drop each on its ArUco spot.

Port of hand_pose_navigation/sorting_app.py onto academy/grasp_pipeline.py
(sdk_wrapper.G1 only).  As in the original:

  - Two configurable ArUco drop-off patterns (dictionary, grid rows/cols
    -- 1 x 1 is a single marker -- first marker ID, marker size, separation,
    release height).  Each pattern's pose comes from solvePnP over whichever
    of its markers are visible; the drop point is the pattern centre offset
    along its upward normal by the release height.
  - Two object definitions (open-vocabulary prompt, optional colour filter,
    mass, how many to sort, assigned pattern).  Both prompts feed the shared
    YOLO-World detector; the colour filter separates objects of one type.
  - Sort repeatedly picks one object, lifts it to the WBC stable pose, places
    it on its pattern and returns, until every requested object is sorted or
    no more can be found.  Objects already on their own pattern are ignored;
    objects on the other pattern count as misplaced and are picked up.
  - Each grab uses dynamic target correction (Motion options), matched like
    the sort picks: label, colour filter and "not already sorted".

Sorting uses the right arm only and never steps the base, so every object and
both patterns must be within reach.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from grasp_pipeline import (
    ABSOLUTE_MAX_JOINT_SPEED_RAD_S, ABSOLUTE_MAX_JOINT_STEP_RAD, DEFAULT_BOARD_PLACE_RELEASE_M,
    DEFAULT_CENTER_WAIST_DEADBAND_RAD, DEFAULT_CENTER_WAIST_MAX_YAW_RAD, DEFAULT_MAX_JOINT_SPEED_RAD_S,
    DEFAULT_MAX_JOINT_STEP_RAD, DEFAULT_OBJECT_MASS_KG, DETECTION_HOLD_S, MAX_OBJECT_MASS_KG, STALE_AFTER_S,
    TABLE_COLLISION_MARGIN_M, WAIST_YAW_JOINT_INDEX, Detection, GraspPipeline, base_from_camera, colorize_depth,
    copy_detection, placeholder,
)
from hand_pose_navigation.segmentation import draw_segmentation_overlay
from hand_pose_navigation.vision_detector import parse_prompt_to_classes

SORT_ARM = "right"
PATTERN_COUNT = 2
OBJECT_COUNT = 2
PATTERN_LETTERS = ("A", "B")

# Predefined OpenCV ArUco dictionaries with their size, so marker IDs can be
# validated before detection.
ARUCO_DICTIONARIES: Dict[str, int] = {
    "DICT_4X4_50": 50, "DICT_4X4_100": 100, "DICT_4X4_250": 250,
    "DICT_5X5_50": 50, "DICT_5X5_100": 100, "DICT_5X5_250": 250,
    "DICT_6X6_50": 50, "DICT_6X6_100": 100, "DICT_6X6_250": 250,
    "DICT_7X7_50": 50, "DICT_7X7_250": 250, "DICT_ARUCO_ORIGINAL": 1024,
}
COLOR_FILTERS = ["any", "red", "orange", "yellow", "green", "blue", "purple", "black", "white", "gray"]
# Fraction of an object's mask pixels that must fall in the requested colour.
COLOR_MATCH_MIN_FRACTION = 0.30

DEFAULT_RELEASE_HEIGHT_M = DEFAULT_BOARD_PLACE_RELEASE_M
MIN_RELEASE_HEIGHT_M = TABLE_COLLISION_MARGIN_M
MAX_RELEASE_HEIGHT_M = 0.30
PLACE_APPROACH_EXTRA_M = 0.05
# An object within the pattern's half-diagonal plus this margin lies on it.
ZONE_MARGIN_M = 0.04
# With the waist fixed, a pattern pose stays valid while the arm or the
# carried object occludes it; how long a remembered pose is trusted.
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
VIEWS = ("sort", "rgb", "depth", "segmentation", "desk-obstacles")


@dataclass
class PatternConfig:
    name: str
    dictionary: str
    first_id: int
    marker_length_m: float
    rows: int
    cols: int
    separation_m: float
    release_height_m: float

    @property
    def layout(self) -> str:
        # A 1 x 1 grid is a single marker; there is no separate layout switch.
        return "single" if self.rows * self.cols == 1 else "grid"

    @property
    def grid_rows(self) -> int:
        return self.rows

    @property
    def grid_cols(self) -> int:
        return self.cols

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
        """Pattern-frame corners of one marker in OpenCV corner order: origin at
        the outer top-left corner of the first marker, +X across columns, +Y
        down rows (v4's GridBoard convention)."""
        index = int(marker_id) - self.first_id
        col, row = index % self.grid_cols, index // self.grid_cols
        pitch = self.marker_length_m + self.separation_m
        x0, y0, s = col * pitch, row * pitch, self.marker_length_m
        return np.array([[x0, y0, 0.0], [x0 + s, y0, 0.0], [x0 + s, y0 + s, 0.0], [x0, y0 + s, 0.0]], dtype=np.float32)

    def centre_pattern(self) -> np.ndarray:
        return np.array([self.width_m / 2.0, self.height_m / 2.0, 0.0, 1.0], dtype=np.float64)

    def describe(self) -> str:
        ids = self.marker_ids()
        id_text = f"id {ids[0]}" if len(ids) == 1 else f"ids {ids[0]}-{ids[-1]}"
        layout = "single marker" if self.layout == "single" else f"{self.grid_rows}x{self.grid_cols} grid"
        return (f"{self.dictionary} {layout}, {id_text}, {self.marker_length_m * 1000:.0f} mm, "
                f"pattern {self.width_m * 100:.1f}x{self.height_m * 100:.1f} cm")

    def fields(self) -> Dict[str, Any]:
        return {"name": self.name, "dict": self.dictionary, "layout": self.layout, "id": self.first_id,
                "size": round(self.marker_length_m * 1000.0, 3), "rows": self.rows, "cols": self.cols,
                "sep": round(self.separation_m * 1000.0, 3), "height": round(self.release_height_m * 100.0, 3)}


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

    def fields(self) -> Dict[str, Any]:
        return {"prompt": self.prompt, "color": self.color, "mass": self.mass_kg, "count": self.count, "zone": self.zone}


DEFAULT_PATTERNS = [
    # aruco_3x4_30mm_6x6_250 / aruco_3x4_30mm_7x7_250: 3 columns x 4 rows of
    # 30 mm markers, 6 mm gaps, ids 0-11 (recognition_app_v4's board geometry).
    PatternConfig("Spot A", "DICT_6X6_250", 0, 0.030, 4, 3, 0.006, DEFAULT_RELEASE_HEIGHT_M),
    PatternConfig("Spot B", "DICT_7X7_250", 0, 0.030, 4, 3, 0.006, DEFAULT_RELEASE_HEIGHT_M),
]
DEFAULT_OBJECTS = [
    ObjectConfig("cube", "red", 0.10, 1, 0, ["cube"]),
    ObjectConfig("cup", "any", 0.20, 1, 1, ["cup"]),
]


def _to_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _to_float(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def parse_pattern(values: Dict[str, Any], fallback: PatternConfig, letter: str) -> Tuple[PatternConfig, List[str]]:
    errors: List[str] = []
    pattern = PatternConfig(
        name=str(values.get("name") or f"Spot {letter}").strip() or f"Spot {letter}",
        dictionary=values.get("dict") if values.get("dict") in ARUCO_DICTIONARIES else fallback.dictionary,
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
        errors.append(f"Pattern {letter}: marker IDs up to {last_id} exceed {pattern.dictionary} "
                      f"(IDs 0-{ARUCO_DICTIONARIES[pattern.dictionary] - 1}).")
    if not MIN_RELEASE_HEIGHT_M <= pattern.release_height_m <= MAX_RELEASE_HEIGHT_M:
        errors.append(f"Pattern {letter}: release height must be {MIN_RELEASE_HEIGHT_M * 100:.0f}-"
                      f"{MAX_RELEASE_HEIGHT_M * 100:.0f} cm (the table collision margin is "
                      f"{TABLE_COLLISION_MARGIN_M * 100:.0f} cm).")
    return pattern, errors


def parse_object(values: Dict[str, Any], index: int) -> Tuple[ObjectConfig, List[str]]:
    prompt = str(values.get("prompt") or "").strip()
    color = values.get("color") if values.get("color") in COLOR_FILTERS else "any"
    mass_kg = max(0.0, min(MAX_OBJECT_MASS_KG, _to_float(values.get("mass"), DEFAULT_OBJECT_MASS_KG)))
    count = max(1, min(MAX_ITEMS_PER_OBJECT, _to_int(values.get("count"), 1)))
    zone = _to_int(values.get("zone"), index)
    if zone not in range(PATTERN_COUNT):
        zone = index
    errors = [] if prompt else [f"Object {index + 1}: enter a recognition prompt."]
    return ObjectConfig(prompt, color, mass_kg, count, zone, parse_prompt_to_classes(prompt) if prompt else []), errors


def validate_pair(patterns: List[PatternConfig], objects: List[ObjectConfig]) -> List[str]:
    errors: List[str] = []
    a, b = patterns
    if a.dictionary == b.dictionary and set(a.marker_ids()) & set(b.marker_ids()):
        errors.append("Patterns A and B use overlapping marker IDs in the same dictionary.")
    o1, o2 = objects
    shared = set(o1.classes) & set(o2.classes)
    if shared and (o1.color == o2.color or "any" in (o1.color, o2.color)):
        errors.append(f"Objects 1 and 2 both match {', '.join(sorted(shared))!r}; give them different "
                      "prompts or two different (non-'any') colour filters.")
    return errors


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


def color_fraction(rgb: Optional[np.ndarray], mask: Optional[np.ndarray], box: Optional[Tuple[int, int, int, int]],
                   color: str) -> float:
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
    pixels = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)[mask]
    return float(_color_masks(pixels[np.newaxis, ...])[color].mean())


def _draw_label(image: np.ndarray, text: str, origin: Tuple[int, int], color: Tuple[int, int, int]) -> None:
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = origin
    y0 = max(0, y - th - 6)
    cv2.rectangle(image, (x, y0), (x + tw + 6, y0 + th + 6), color, -1)
    cv2.putText(image, text, (x + 3, y0 + th + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


class Sorter:
    """Sort configuration, ArUco pattern tracking and the sort sequence."""

    def __init__(self, pipeline: GraspPipeline) -> None:
        self.p = pipeline
        self.lock = threading.RLock()
        self.patterns: List[PatternConfig] = list(DEFAULT_PATTERNS)
        self.objects: List[ObjectConfig] = list(DEFAULT_OBJECTS)
        self.config_errors: List[str] = []
        self.vision_prompt = ""
        self.center_waist = False
        self.allow_unverified_grasp = False
        # Latest pose of each pattern in the camera frame and the view epoch
        # it was measured in.  The epoch increments whenever the sort turns
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
        # Object definition being grabbed (dynamic correction matcher).
        self.active_object: Optional[int] = None
        self.summary = ""
        self.run_lock = threading.Lock()
        self.stop_event = threading.Event()
        self._aruco_detectors: Dict[str, Any] = {}
        self._pattern_thread: Optional[threading.Thread] = None
        self.p.correction_candidates = self._correction_candidates
        self._apply_config(list(DEFAULT_PATTERNS), list(DEFAULT_OBJECTS), [])

    def _log(self, line: str) -> None:
        self.p.log(f"[sort] {line}")

    def _set_phase(self, phase: str) -> None:
        with self.lock:
            self.phase = phase
        self._log(phase)

    def touch(self) -> None:
        self.p.touch()
        thread = self._pattern_thread
        if thread is None or not thread.is_alive():
            self._pattern_thread = threading.Thread(target=self._pattern_loop, args=(6.0,), name="sort-aruco", daemon=True)
            self._pattern_thread.start()

    # -- configuration ---------------------------------------------------------

    def set_config(self, body: Dict[str, Any]) -> Dict[str, Any]:
        with self.lock:
            if self.running:
                return {"ok": False, "running": True,
                        "message": "Sort running - configuration changes apply to the next sort."}
        patterns, objects, errors = [], [], []
        raw_patterns = list(body.get("patterns") or [])
        raw_objects = list(body.get("objects") or [])
        for i in range(PATTERN_COUNT):
            values = raw_patterns[i] if i < len(raw_patterns) and isinstance(raw_patterns[i], dict) else {}
            pattern, pattern_errors = parse_pattern(values, DEFAULT_PATTERNS[i], PATTERN_LETTERS[i])
            patterns.append(pattern)
            errors.extend(pattern_errors)
        for i in range(OBJECT_COUNT):
            values = raw_objects[i] if i < len(raw_objects) and isinstance(raw_objects[i], dict) else {}
            obj, obj_errors = parse_object(values, i)
            objects.append(obj)
            errors.extend(obj_errors)
        errors.extend(validate_pair(patterns, objects))
        self._apply_config(patterns, objects, errors)
        return {"ok": not errors, **self.config_view()}

    def _apply_config(self, patterns: List[PatternConfig], objects: List[ObjectConfig], errors: List[str]) -> None:
        classes: List[str] = []
        for obj in objects:
            classes.extend(cls for cls in obj.classes if cls not in classes)
        prompt = ", ".join(classes)
        with self.lock:
            self.patterns = patterns
            self.objects = objects
            self.config_errors = errors
            prompt_changed = bool(prompt) and prompt != self.vision_prompt
            if prompt_changed:
                self.vision_prompt = prompt
        if prompt_changed:
            self._log(f"Vision prompt set to: {', '.join(self.p.set_prompt(prompt))}")

    def config_view(self) -> Dict[str, Any]:
        with self.lock:
            patterns, objects, errors = list(self.patterns), list(self.objects), list(self.config_errors)
        lines = [f"Pattern {PATTERN_LETTERS[i]} ({p.name}): {p.describe()}" for i, p in enumerate(patterns)] + [
            f"Object {i + 1}: {o.count} x {o.name} (classes: {', '.join(o.classes) or '-'}; {o.mass_kg:.2f} kg) "
            f"-> {patterns[o.zone].name}" for i, o in enumerate(objects)]
        return {"patterns": [p.fields() for p in patterns], "objects": [o.fields() for o in objects],
                "errors": errors, "lines": [] if errors else lines}

    def set_options(self, body: Dict[str, Any]) -> Dict[str, Any]:
        p = self.p
        with p.lock:
            if "max_joint_step_rad" in body:
                p.max_joint_step_rad = max(0.005, min(ABSOLUTE_MAX_JOINT_STEP_RAD,
                                                      _to_float(body["max_joint_step_rad"], DEFAULT_MAX_JOINT_STEP_RAD)))
            if "max_joint_speed_rad_s" in body:
                p.max_joint_speed_rad_s = max(0.02, min(ABSOLUTE_MAX_JOINT_SPEED_RAD_S,
                                                        _to_float(body["max_joint_speed_rad_s"], DEFAULT_MAX_JOINT_SPEED_RAD_S)))
            if "dynamic_target_correction" in body:
                p.dynamic_target_correction = bool(body["dynamic_target_correction"])
        with self.lock:
            if "center_waist" in body:
                self.center_waist = bool(body["center_waist"])
            if "allow_unverified_grasp" in body:
                self.allow_unverified_grasp = bool(body["allow_unverified_grasp"])
        return {"ok": True, "options": self.options()}

    def options(self) -> Dict[str, Any]:
        with self.p.lock:
            out = {"max_joint_step_rad": self.p.max_joint_step_rad, "max_joint_speed_rad_s": self.p.max_joint_speed_rad_s,
                   "dynamic_target_correction": self.p.dynamic_target_correction}
        with self.lock:
            out.update(center_waist=self.center_waist, allow_unverified_grasp=self.allow_unverified_grasp)
        return out

    @staticmethod
    def meta() -> Dict[str, Any]:
        return {"dictionaries": list(ARUCO_DICTIONARIES), "colors": COLOR_FILTERS, "letters": list(PATTERN_LETTERS),
                "views": list(VIEWS), "max_items": MAX_ITEMS_PER_OBJECT, "max_mass": MAX_OBJECT_MASS_KG,
                "release_cm": [MIN_RELEASE_HEIGHT_M * 100, MAX_RELEASE_HEIGHT_M * 100],
                "max_step": ABSOLUTE_MAX_JOINT_STEP_RAD, "max_speed": ABSOLUTE_MAX_JOINT_SPEED_RAD_S,
                "pattern_colors": ["#%02x%02x%02x" % c[::-1] for c in PATTERN_COLORS_BGR],
                "object_colors": ["#%02x%02x%02x" % c[::-1] for c in OBJECT_COLORS_BGR]}

    # -- ArUco pattern detection -----------------------------------------------

    def _detect_markers(self, gray: np.ndarray, dictionary_name: str) -> Tuple[List[np.ndarray], np.ndarray]:
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        if hasattr(aruco, "ArucoDetector"):
            detector = self._aruco_detectors.get(dictionary_name)
            if detector is None:
                detector = self._aruco_detectors[dictionary_name] = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            corners, ids, _rejected = detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = aruco.detectMarkers(gray, dictionary)
        if ids is None:
            return [], np.zeros((0,), dtype=np.int32)
        return list(corners), ids.reshape(-1).astype(np.int32)

    def _estimate_pattern_pose(self, pattern: PatternConfig, corners: List[np.ndarray],
                               ids: np.ndarray) -> Tuple[Optional[np.ndarray], int, List[np.ndarray], List[int]]:
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
        cam = self.p.camera_matrix()
        ok, rvec, tvec = cv2.solvePnP(obj, img, cam, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(obj, img, cam, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, len(used_ids), used_corners, used_ids
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.reshape(3)
        if T[2, 3] <= 0.05:
            return None, len(used_ids), used_corners, used_ids
        return T, len(used_ids), used_corners, used_ids

    def _pattern_loop(self, rate_hz: float) -> None:
        period = 1.0 / max(0.5, rate_hz)
        last_seq = -1
        if not hasattr(cv2, "aruco"):
            with self.lock:
                self.pattern_status = ["OpenCV was built without cv2.aruco (install opencv-contrib)"] * PATTERN_COUNT
            return
        while self.p._wanted():
            t0 = time.time()
            with self.p.lock:
                seq = self.p.frame_seq
                frame_ts = self.p.last_frame_ts
                rgb = None if self.p.rgb_bgr is None else self.p.rgb_bgr.copy()
            if rgb is None or seq == last_seq:
                time.sleep(0.05)
                continue
            last_seq = seq
            with self.lock:
                patterns = list(self.patterns)
                epoch = self.view_epoch if frame_ts >= self.view_settle_until else -1
            try:
                gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
                markers = {name: self._detect_markers(gray, name) for name in {p.dictionary for p in patterns}}
                overlay: List[Tuple[List[np.ndarray], np.ndarray]] = []
                results = []
                for pattern in patterns:
                    corners, ids = markers[pattern.dictionary]
                    T, count, used_corners, used_ids = self._estimate_pattern_pose(pattern, corners, ids)
                    if used_corners:
                        overlay.append((used_corners, np.asarray(used_ids, dtype=np.int32).reshape(-1, 1)))
                    results.append((T, count))
                with self.lock:
                    self.pattern_markers = overlay
                    for i, (T, count) in enumerate(results):
                        if T is None:
                            self.pattern_status[i] = (f"not visible (0/{patterns[i].marker_count} markers)" if count == 0
                                                      else f"{count}/{patterns[i].marker_count} markers, pose failed")
                            continue
                        self.pattern_T_camera[i] = T
                        self.pattern_ts[i] = frame_ts
                        self.pattern_epoch[i] = epoch
                        self.pattern_status[i] = f"visible ({count}/{patterns[i].marker_count} markers)"
            except Exception as exc:
                with self.lock:
                    self.pattern_status = [f"detection error: {exc}"] * PATTERN_COUNT
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _pattern_T_camera(self, index: int, require_current_epoch: bool) -> Optional[np.ndarray]:
        with self.lock:
            T = self.pattern_T_camera[index]
            age = time.time() - self.pattern_ts[index]
            epoch_ok = self.pattern_epoch[index] == self.view_epoch
        if T is None or age > PATTERN_MEMORY_S or (require_current_epoch and not epoch_ok):
            return None
        return T.copy()

    def _pattern_fresh(self, index: int, since_ts: float) -> bool:
        with self.lock:
            return (self.pattern_T_camera[index] is not None and self.pattern_ts[index] >= since_ts
                    and self.pattern_epoch[index] == self.view_epoch)

    def _wait_for_pattern(self, index: int, since_ts: float, timeout_s: float = PATTERN_WAIT_S) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._pattern_fresh(index, since_ts):
                return True
            if self.stop_event.is_set():
                return False
            time.sleep(0.1)
        return self._pattern_fresh(index, since_ts)

    @staticmethod
    def _pattern_drop_geometry(pattern: PatternConfig, T_camera_pattern: np.ndarray,
                               cam: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """Pattern centre and upward normal in base_link."""
        T_base_pattern = base_from_camera(cam) @ T_camera_pattern
        centre = (T_base_pattern @ pattern.centre_pattern())[:3]
        normal = T_base_pattern[:3, 2].copy()
        if float(normal[2]) < 0.0:
            normal *= -1.0
        return centre, normal

    # -- object classification -------------------------------------------------

    def _scene_snapshot(self) -> Dict[str, Any]:
        p = self.p
        with p.lock:
            snap = {
                "detections": {k: copy_detection(v) for k, v in p.detections.items()},
                "masks": {det_id: mask for det_id, _label, mask in p.masks_for_draw},
                "rgb": None if p.detection_rgb_bgr is None else p.detection_rgb_bgr.copy(),
                "detection_ts": p.last_detection_ts,
                "frame_ts": p.last_frame_ts,
                "cam": dict(p.camera_extrinsic),
            }
        with self.lock:
            snap["patterns"] = list(self.patterns)
            snap["objects"] = list(self.objects)
        return snap

    def _zone_centres(self, patterns: List[PatternConfig], cam: Dict[str, float]) -> List[Optional[np.ndarray]]:
        centres: List[Optional[np.ndarray]] = []
        for i, pattern in enumerate(patterns):
            T = self._pattern_T_camera(i, require_current_epoch=True)
            if T is None:
                T = self._pattern_T_camera(i, require_current_epoch=False)
            centres.append(None if T is None else self._pattern_drop_geometry(pattern, T, cam)[0])
        return centres

    def _classify(self, snap: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Assign each detection to an object definition and drop-off zone.

        One entry per detection: det, obj (index or None), zone (pattern it
        lies on, or None), color_fraction, sortable (assigned and not already
        on its own pattern).
        """
        now = time.time()
        if snap["detection_ts"] <= 0.0 or now - snap["detection_ts"] > STALE_AFTER_S or now - snap["frame_ts"] > STALE_AFTER_S:
            return []
        objects: List[ObjectConfig] = snap["objects"]
        patterns: List[PatternConfig] = snap["patterns"]
        T_base_camera = base_from_camera(snap["cam"])
        centres = self._zone_centres(patterns, snap["cam"])
        entries = []
        for det_id, det in sorted(snap["detections"].items()):
            best_obj, best_fraction = None, -1.0
            for obj_index, obj in enumerate(objects):
                if det.label not in obj.classes:
                    continue
                fraction = color_fraction(snap["rgb"], snap["masks"].get(det_id), det.box, obj.color)
                if obj.color != "any" and fraction < COLOR_MATCH_MIN_FRACTION:
                    continue
                if fraction > best_fraction:
                    best_obj, best_fraction = obj_index, fraction
            p_base = (T_base_camera @ det.T_camera_object)[:3, 3]
            zone = None
            for zone_index, centre in enumerate(centres):
                if centre is not None and float(np.linalg.norm(p_base[:2] - centre[:2])) <= patterns[zone_index].zone_radius_m:
                    zone = zone_index
                    break
            entries.append({"det": det, "obj": best_obj, "zone": zone, "p_base": p_base, "color_fraction": best_fraction,
                            "sortable": best_obj is not None and zone != objects[best_obj].zone})
        return entries

    def _pick_candidate(self, obj_index: int, min_detection_ts: float, prefer_centre: bool) -> Optional[Detection]:
        snap = self._scene_snapshot()
        if snap["detection_ts"] < min_detection_ts:
            return None
        entries = [e for e in self._classify(snap) if e["obj"] == obj_index and e["sortable"]]
        if not entries:
            return None
        K = self.p.K
        if prefer_centre:
            # After the waist turned toward the target it sits nearest the
            # image's vertical centre line.
            def key(e):
                pc = e["det"].T_camera_object[:3, 3]
                return abs(K.fx * float(pc[0]) / max(0.05, float(pc[2])))
        else:
            def key(e):
                return (-round(float(e["det"].score), 2), float(np.linalg.norm(e["p_base"][:2])))
        return min(entries, key=key)["det"]

    def _correction_candidates(self, det: Detection) -> Tuple[float, List[np.ndarray]]:
        """Dynamic grab correction matcher: during a sort grab accept only
        detections classified as the object being grabbed (label, colour
        filter, not already sorted), so a same-type object of the other
        colour, or one already sorted, cannot pull the approach away."""
        with self.lock:
            obj_index = self.active_object
        if obj_index is None:
            with self.p.lock:
                return self.p.last_detection_ts, [d.T_camera_object.copy() for d in self.p.detections.values()
                                                  if d.source == det.source and d.label == det.label]
        snap = self._scene_snapshot()
        return snap["detection_ts"], [e["det"].T_camera_object.copy() for e in self._classify(snap)
                                      if e["obj"] == obj_index and e["sortable"]]

    def _wait_for_candidate(self, obj_index: int, prefer_centre: bool = False) -> Optional[Detection]:
        # Detections are held for DETECTION_HOLD_S after they disappear: wait
        # for a newer pass so an object that just moved (the one just placed)
        # is not reported at its old position.
        min_ts = time.time() + DETECTION_HOLD_S
        deadline = min_ts + CANDIDATE_WAIT_S
        while time.time() < deadline:
            if self.stop_event.is_set():
                return None
            det = self._pick_candidate(obj_index, min_ts, prefer_centre)
            if det is not None:
                return det
            time.sleep(0.2)
        return None

    # -- motion primitives -------------------------------------------------------

    def _acquire_arm_lock(self, timeout_s: float = 10.0) -> bool:
        if self.p.arm_lock.acquire(timeout=timeout_s):
            return True
        self._log("Arm controller stayed busy; aborting this step.")
        return False

    def _move_waist(self, target_yaw: float, max_speed_rad_s: float) -> None:
        """Turn the waist (caller holds arm_lock).  This moves the camera
        relative to the arm base frame, so pattern poses measured before or
        during the turn are no longer used for placement."""
        with self.lock:
            self.view_epoch += 1
            self.view_settle_until = float("inf")
        try:
            self.p.stop_active_hold()
            self.p.ensure_arm_authority(duration_s=0.3)
            result = self.p.move_waist(target_yaw, max_speed_rad_s)
            self._log(f"waist yaw move result: {result.get('duration_s', 0.0):.2f}s")
        finally:
            with self.lock:
                self.view_settle_until = time.time() + VIEW_SETTLE_S

    def _center_waist_on(self, det: Detection, max_speed_rad_s: float) -> bool:
        p_cam = det.T_camera_object[:3, 3]
        if float(p_cam[2]) <= 0.05:
            return False
        yaw_delta = -math.atan2(float(p_cam[0]), float(p_cam[2]))
        yaw_delta = max(-DEFAULT_CENTER_WAIST_MAX_YAW_RAD, min(DEFAULT_CENTER_WAIST_MAX_YAW_RAD, yaw_delta))
        if abs(yaw_delta) < DEFAULT_CENTER_WAIST_DEADBAND_RAD or not self._acquire_arm_lock():
            return False
        try:
            self.p._busy("waist center")
            current = self.p.read_joint_position(WAIST_YAW_JOINT_INDEX)
            self._log(f"Centering target with waist yaw delta {yaw_delta:+.3f} rad.")
            self._move_waist(current + yaw_delta, max_speed_rad_s)
            return True
        except Exception as exc:
            self._log(f"Waist centering failed; continuing without it: {exc}")
            return False
        finally:
            self.p._idle()
            self.p.arm_lock.release()

    def _abandon_grasp(self) -> None:
        """Open the hand at the grasp pose and retract to the stable pose."""
        try:
            self.p.send_hand_action("open", SORT_ARM)
        except Exception as exc:
            self._log(f"Opening hand after rejected grasp failed: {exc}")
        self.p.clear_hold_flags()
        self.p.run_retract_stable_pose()

    def _grab_object(self, obj_index: int, obj: ObjectConfig, det: Detection, options: Dict[str, Any]) -> str:
        """Pick one object and lift it to the WBC stable pose.

        Returns "held", "failed" (nothing in hand; try again), "unreachable",
        or "abort" (an object may be in hand or the sort was stopped).
        """
        p = self.p
        with p.lock:
            p.object_mass_kg = obj.mass_kg
            p.selected_id = det.id
            max_step = p.max_joint_step_rad
            max_speed = p.max_joint_speed_rad_s
            p.grab_running = True
        # Placement leaves the arm at the WBC stable pose, not the extend
        # pose: forget the extend reference so every pick runs the full,
        # table-aware extend recipe again.
        p.forget_extend_reference(SORT_ARM)
        with self.lock:
            self.current_target = (obj_index, det.box)
            self.active_object = obj_index
        try:
            if options["center_waist"] and self._center_waist_on(det, max_speed):
                refreshed = self._wait_for_candidate(obj_index, prefer_centre=True)
                if refreshed is None:
                    self._log(f"{obj.name} not re-detected after waist centering.")
                    return "abort" if self.stop_event.is_set() else "failed"
                det = refreshed
                with self.lock:
                    self.current_target = (obj_index, det.box)
            with p.lock:
                cam = dict(p.camera_extrinsic)
                p.selected_id = det.id
            arm, _T_base_camera, T_base_object = p.resolve_arm_and_base_pose(det, cam, SORT_ARM)
            _arm, reach, max_reach, excess = p.reach_preview(det, cam, SORT_ARM)
            pb = T_base_object[:3, 3]
            self._log(f"Target {obj.name}: label={det.label!r} score={det.score:.2f} "
                      f"base xyz=({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:+.3f}) m reach={reach:.3f}/{max_reach:.3f} m")
            if excess > 0.0:
                self._log(f"{obj.name} is {excess:.3f} m outside arm reach; skipping it.")
                return "unreachable"
            if not p.prepare_hand_visibility_for_grab(arm, cam, max_step, max_speed):
                return "abort" if self.stop_event.is_set() else "failed"
            if self.stop_event.is_set():
                return "abort"
            p.run_direct_grab_inline(det, arm, cam, T_base_object, max_step, max_speed, mass_kg=obj.mass_kg)
        finally:
            with p.lock:
                p.grab_running = False
            with self.lock:
                self.current_target = None
                self.active_object = None
        with p.lock:
            grasped = p.lift_confirmation_pending and p.holding_object
            unverified = p.lift_unverified
        if self.stop_event.is_set():
            return "abort"
        if not grasped:
            self._log(f"Grab of {obj.name} did not end with a closed hand.")
            return "failed"
        if unverified and not options["allow_unverified_grasp"]:
            self._log(f"Grasp of {obj.name} is not tactile-verified; releasing and retrying.")
            self._abandon_grasp()
            return "abort" if self.stop_event.is_set() else "failed"
        self._set_phase(f"Lifting {obj.name} to the stable pose")
        p.run_confirm_lift()
        with p.lock:
            lifted = not p.lift_confirmation_pending
            status = p.status_msg
        if not lifted:
            self._log(f"Lift of {obj.name} failed; stopping with the object held. Status: {status}")
            return "abort"
        return "held"

    def _place_held_object(self, obj: ObjectConfig, pattern_index: int, pattern: PatternConfig,
                           options: Dict[str, Any]) -> bool:
        """Place the object held in the right hand on a pattern and retract."""
        p = self.p
        if not self._acquire_arm_lock():
            return False
        try:
            p._busy(f"place on {pattern.name}")
            with p.lock:
                max_step, max_speed = p.max_joint_step_rad, p.max_joint_speed_rad_s
                cam = dict(p.camera_extrinsic)
            start_yaw = options.get("start_waist_yaw")
            if start_yaw is not None:
                current_yaw = p.read_joint_position(WAIST_YAW_JOINT_INDEX)
                if abs(current_yaw - start_yaw) >= DEFAULT_CENTER_WAIST_DEADBAND_RAD:
                    self._log(f"Returning waist to the sort start yaw ({start_yaw:+.3f} rad) to view {pattern.name}.")
                    self._move_waist(start_yaw, max_speed)
                    with self.lock:
                        settle = self.view_settle_until
                    if not self._wait_for_pattern(pattern_index, settle):
                        raise RuntimeError(f"{pattern.name} is not visible after returning the waist")
            T_camera_pattern = self._pattern_T_camera(pattern_index, require_current_epoch=True)
            if T_camera_pattern is None:
                raise RuntimeError(f"no current pose for {pattern.name}; keep the pattern in camera view")
            centre, normal = self._pattern_drop_geometry(pattern, T_camera_pattern, cam)
            release = centre + normal * pattern.release_height_m
            approach = centre + normal * (pattern.release_height_m + PLACE_APPROACH_EXTRA_M)
            self._log(f"{pattern.name} centre base xyz=({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f}) m; "
                      f"release {pattern.release_height_m * 100:.1f} cm above it.")
            with p.nav_lock:
                nav = p.active_nav
            p.shutdown_nav(nav, "before sort placement")
            p.stop_active_hold()
            obstacles = p.estimate_obstacles(cam)
            lift = p.lift_end_effector_z_slowly(SORT_ARM, max_step, max_speed, mass_kg=obj.mass_kg, obstacles=obstacles)
            if not lift.get("success"):
                raise RuntimeError(f"clearance lift failed: {lift.get('reason')} {lift.get('violations') or ''}")
            for label, point in ((f"{pattern.name} approach", approach), (f"{pattern.name} release", release)):
                if self.stop_event.is_set():
                    raise RuntimeError("sort stopped")
                result = p.move_ee_to_position(SORT_ARM, point, max_step, max_speed, obstacles, label)
                self._log(f"{label} result: success={result.get('success')} reason={result.get('reason', '')}")
                if not result.get("success"):
                    raise RuntimeError(f"{label} failed: {result.get('reason')}")
            p.send_hand_action("open", SORT_ARM)
            p.clear_hold_flags()
            self._log(f"Released {obj.name} on {pattern.name}.")
            # Rise straight up before swinging back so the fingers clear the object.
            retreat = p.move_ee_to_position(SORT_ARM, approach, max_step, max_speed, obstacles, f"{pattern.name} retreat")
            if not retreat.get("success"):
                self._log(f"Retreat above {pattern.name} failed ({retreat.get('reason')}); retracting directly.")
            stable = p.move_to_wbc_right_arm_forward(max_step, max_speed, obstacles)
            self._log(f"Return to WBC stable pose: success={stable.get('success')} reason={stable.get('reason', '')}")
            return True
        except Exception as exc:
            self._log(f"Placement on {pattern.name} failed: {exc}")
            with p.lock:
                p.status_msg = f"Placement on {pattern.name} failed: {exc}"
            return False
        finally:
            p._idle()
            p.arm_lock.release()

    # -- sort sequence -------------------------------------------------------------

    def _run_sort(self) -> None:
        if not self.run_lock.acquire(blocking=False):
            self._log("Sort already running.")
            return
        p = self.p
        p.keep_alive += 1
        try:
            self.stop_event.clear()
            with self.lock:
                patterns = list(self.patterns)
                objects = list(self.objects)
                errors = list(self.config_errors)
                options = {"center_waist": self.center_waist, "allow_unverified_grasp": self.allow_unverified_grasp,
                           "start_waist_yaw": None}
                self.running = True
                self.sorted_counts = [0] * OBJECT_COUNT
                self.summary = ""
            if errors:
                self._set_phase("Sort refused: fix the configuration first")
                for error in errors:
                    self._log(error)
                return
            with p.lock:
                busy = p.grab_running or p.arm_motion_running or p.holding_object
            if busy:
                self._set_phase("Sort refused: the arm is busy or already holding an object")
                return
            self._set_phase("Checking that both drop-off patterns are visible")
            t_check = time.time()
            for i, pattern in enumerate(patterns):
                if not self._wait_for_pattern(i, t_check):
                    self._set_phase(f"Sort refused: {pattern.name} ({pattern.describe()}) is not visible")
                    return
            if options["center_waist"]:
                try:
                    options["start_waist_yaw"] = p.read_joint_position(WAIST_YAW_JOINT_INDEX)
                except Exception as exc:
                    self._log(f"Cannot read waist yaw ({exc}); sorting without waist centering.")
                    options["center_waist"] = False
            self._log("Sort started: " + "; ".join(f"{o.count} x {o.name} -> {patterns[o.zone].name}" for o in objects) + ".")
            sorted_counts = [0] * OBJECT_COUNT
            failures = [0] * OBJECT_COUNT
            finished = [False] * OBJECT_COUNT
            notes: List[str] = []
            aborted = False
            while not aborted and not all(finished):
                for i, obj in enumerate(objects):
                    if finished[i]:
                        continue
                    if self.stop_event.is_set():
                        aborted = True
                        break
                    if sorted_counts[i] >= obj.count:
                        finished[i] = True
                        continue
                    self._set_phase(f"Looking for {obj.name} ({sorted_counts[i] + 1}/{obj.count})")
                    det = self._wait_for_candidate(i)
                    if det is None:
                        if self.stop_event.is_set():
                            aborted = True
                            break
                        notes.append(f"no more {obj.name} found ({sorted_counts[i]}/{obj.count} sorted)")
                        finished[i] = True
                        continue
                    self._set_phase(f"Picking {obj.name} ({sorted_counts[i] + 1}/{obj.count})")
                    outcome = self._grab_object(i, obj, det, options)
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
                    self._set_phase(f"Placing {obj.name} on {pattern.name}")
                    if not self._place_held_object(obj, obj.zone, pattern, options):
                        notes.append(f"placement of {obj.name} on {pattern.name} failed")
                        aborted = True
                        break
                    sorted_counts[i] += 1
                    with self.lock:
                        self.sorted_counts = list(sorted_counts)
            start_yaw = options.get("start_waist_yaw")
            if start_yaw is not None and not self.stop_event.is_set() and not p.holding_object and self._acquire_arm_lock():
                try:
                    if abs(p.read_joint_position(WAIST_YAW_JOINT_INDEX) - start_yaw) >= DEFAULT_CENTER_WAIST_DEADBAND_RAD:
                        self._move_waist(start_yaw, p.max_joint_speed_rad_s)
                except Exception as exc:
                    self._log(f"Returning waist to start yaw failed: {exc}")
                finally:
                    p.arm_lock.release()
            result = ", ".join(f"{obj.name} {sorted_counts[i]}/{obj.count}" for i, obj in enumerate(objects))
            headline = "Sort stopped" if self.stop_event.is_set() else "Sort aborted" if aborted else "Sort finished"
            summary = f"{headline}: {result}" + (f" ({'; '.join(notes)})" if notes else "")
            with self.lock:
                self.summary = summary
            if p.holding_object:
                summary += " - an object may still be in the hand; use Open hand / Retract."
            self._set_phase(summary)
        except Exception as exc:
            self._set_phase(f"Sort failed: {exc!r}")
        finally:
            with self.lock:
                self.running = False
                self.current_target = None
            p.keep_alive -= 1
            self.run_lock.release()

    def action(self, name: str) -> Dict[str, Any]:
        def stop_sort() -> None:
            self.stop_event.set()
            self._log("Stop sorting requested.")
            self.p.run_stop_grabbing(SORT_ARM)

        actions = {
            "sort": (self._run_sort, "Sort started - see the log."),
            "stop": (stop_sort, "Stopping sort; holding the current arm pose..."),
            "retract": (self.p.run_retract_stable_pose, "Retracting right arm to the stable pose..."),
            "hand_open": (lambda: self.p.run_hand_action("open", SORT_ARM), "Opening right hand..."),
            "hand_close": (lambda: self.p.run_hand_action("close", SORT_ARM), "Closing right hand..."),
            "release_arms": (self.p.run_release_arms, "Releasing arms..."),
            "engage_arms": (self.p.run_engage_arms, "Re-engaging arms..."),
            "damp": (self.p.run_damp, "Damping..."),
        }
        if name not in actions:
            raise ValueError(f"Unknown /sort action {name!r}")
        if name == "sort":
            with self.lock:
                if self.running:
                    raise ValueError("A sort is already running")
                if self.config_errors:
                    raise ValueError("Fix the configuration first: " + " ".join(self.config_errors))
        if name in ("release_arms", "damp"):
            self.stop_event.set()
        self.touch()
        target, message = actions[name]
        threading.Thread(target=target, name=f"sort-{name}", daemon=True).start()
        return {"ok": True, "message": message}

    def busy(self) -> bool:
        """A sort or an arm motion is in progress (an idle pose hold is not)."""
        with self.lock:
            running = self.running
        return running or self.p.busy()

    def yield_arms(self) -> None:
        """Stop the idle pose hold so another rt/arm_sdk controller can take over."""
        if not self.busy():
            self.p.yield_arms()

    def close(self) -> None:
        self.stop_event.set()
        self.p.cancel.set()
        with self.p.nav_lock:
            nav = self.p.active_nav
        self.p.shutdown_nav(nav, "dashboard shutdown")
        self.p.stop_active_hold()

    # -- status and views ------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        self.touch()
        p = self.p
        snap = self._scene_snapshot()
        with p.lock:
            status_msg = p.status_msg
            log = list(p.grab_log[-150:])
            arm_busy = p.grab_running or p.arm_motion_running
            arm_label = p.arm_motion_label
            holding = p.holding_object
            frame_ts = p.last_frame_ts
            classes = list(p.vision_classes)
        with self.lock:
            running, phase = self.running, self.phase
            sorted_counts = list(self.sorted_counts)
            pattern_status = list(self.pattern_status)
            has_errors = bool(self.config_errors)
        patterns: List[PatternConfig] = snap["patterns"]
        objects: List[ObjectConfig] = snap["objects"]
        entries = self._classify(snap)
        if time.time() - frame_ts > STALE_AFTER_S:
            status_msg = f"STALE - no fresh frame in >{STALE_AFTER_S:.0f}s. {status_msg}"
        pattern_rows = []
        for i, pattern in enumerate(patterns):
            row = {"letter": PATTERN_LETTERS[i], "name": pattern.name, "describe": pattern.describe(),
                   "status": pattern_status[i], "visible": pattern_status[i].startswith("visible"), "centre": None}
            T = self._pattern_T_camera(i, require_current_epoch=False)
            if T is not None:
                row["centre"] = [float(v) for v in self._pattern_drop_geometry(pattern, T, snap["cam"])[0]]
            pattern_rows.append(row)
        scene = []
        for i, obj in enumerate(objects):
            mine = [e for e in entries if e["obj"] == i]
            todo = sum(1 for e in mine if e["sortable"])
            scene.append({"object": i + 1, "name": obj.name, "zone": patterns[obj.zone].name, "todo": todo,
                          "done": len(mine) - todo, "sorted": sorted_counts[i], "count": obj.count})
        return {
            "running": running, "phase": phase, "status_msg": status_msg, "arm_busy": arm_label if arm_busy else "",
            "holding": holding, "patterns": pattern_rows, "scene": scene,
            "ignored": [e["det"].label for e in entries if e["obj"] is None],
            "sort_disabled": running or arm_busy or has_errors, "log": log, "tactile": p.tactile_status_text(),
            "camera": p.camera_status, "vision": p.vision_status, "classes": classes, "obstacles": p.obstacle_status,
            "options": self.options(),
        }

    def render(self, view: str) -> np.ndarray:
        """BGR image for the live view (sorting overlay, raw, depth, masks, desk)."""
        self.touch()
        p = self.p
        with p.lock:
            rgb = None if p.rgb_bgr is None else p.rgb_bgr.copy()
            depth = None if p.depth_m is None else p.depth_m.copy()
            masks = list(p.masks_for_draw)
            cam = dict(p.camera_extrinsic)
        if rgb is None:
            return placeholder(p.camera_status[:48] or "waiting for camera...")
        if view == "rgb":
            return rgb
        if view == "depth":
            return colorize_depth(depth) if depth is not None else placeholder("no depth")
        if view == "segmentation":
            return draw_segmentation_overlay(rgb, masks) if masks else placeholder("no masks")
        if view == "desk-obstacles":
            return p.draw_desk_obstacle_segmentation(rgb, depth, cam, masks)
        snap = self._scene_snapshot()
        return self._draw_sort_view(rgb, snap, self._classify(snap))

    def _draw_sort_view(self, image: np.ndarray, snap: Dict[str, Any], entries: List[Dict[str, Any]]) -> np.ndarray:
        out = image.copy()
        patterns: List[PatternConfig] = snap["patterns"]
        objects: List[ObjectConfig] = snap["objects"]
        with self.lock:
            markers = list(self.pattern_markers)
            current_target = self.current_target
        for corners, ids in markers:
            try:
                cv2.aruco.drawDetectedMarkers(out, corners, ids)
            except Exception:
                pass
        for i, pattern in enumerate(patterns):
            T = self._pattern_T_camera(i, require_current_epoch=False)
            if T is None:
                continue
            color = PATTERN_COLORS_BGR[i]
            outline = [np.array([0.0, 0.0, 0.0, 1.0]), np.array([pattern.width_m, 0.0, 0.0, 1.0]),
                       np.array([pattern.width_m, pattern.height_m, 0.0, 1.0]), np.array([0.0, pattern.height_m, 0.0, 1.0])]
            pts = [self.p.project_camera_point((T @ q)[:3]) for q in outline]
            if all(q is not None for q in pts):
                cv2.polylines(out, [np.array(pts, dtype=np.int32)], True, color, 2, cv2.LINE_AA)
            centre_uv = self.p.project_camera_point((T @ pattern.centre_pattern())[:3])
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
                color, text = SORTED_COLOR_BGR, f"{objects[entry['obj']].name}: sorted"
            else:
                color = OBJECT_COLORS_BGR[entry["obj"]]
                text = f"{objects[entry['obj']].name} -> {PATTERN_LETTERS[objects[entry['obj']].zone]}"
            is_target = current_target is not None and tuple(current_target[1] or ()) == tuple(det.box)
            cv2.rectangle(out, (x0, y0), (x1, y1), color, 4 if is_target else 2)
            _draw_label(out, f"{text} {det.score:.2f}" + ("  [TARGET]" if is_target else ""), (x0, y0), color)
        return out
