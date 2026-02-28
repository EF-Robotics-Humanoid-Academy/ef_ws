"""
vision_module.py
================

Records ALL 33 MediaPipe Pose landmarks (full body: face, arms, torso,
legs, feet) from the RealSense RGBD feed for a configurable duration.

Pipeline
--------
1. Open a RealSense pipeline following
   ``sensors/manual_streaming/stream_realsense.py``
   (aligned depth-to-color, spatial + temporal filters).
2. Run MediaPipe Pose on every color frame to localise all 33 landmarks.
3. For each landmark store:
   - pixel coords (px, py) in the color image
   - visibility confidence
   - world_xyz  : MediaPipe metric 3-D world coords (hip-centred, metres,
                  x=right  y=up  z=toward-viewer)
   - depth_xyz  : pixel (px, py) + RealSense depth Z in metres
4. Collect samples for ``--duration`` seconds, then write to disk as
   JSON (default) or NumPy archive (.npy/.npz).
5. Live preview: Canny edge overlay + full-body skeleton + depth panel.

Usage
-----
    python3 vision_module.py [OPTIONS]

    --width      INT    Stream width  (default: 640)
    --height     INT    Stream height (default: 480)
    --fps        INT    Frame rate    (default: 30)
    --duration   FLOAT  Recording length in seconds (default: 5.0)
    --output     PATH   Output file  (default: pose_recording.json)
    --no-display        Suppress OpenCV window (headless / SSH mode)

Dependencies
------------
    pip install pyrealsense2 mediapipe opencv-python numpy

References
----------
    ../../../sensors/manual_streaming/sdk_details.md
    ../../../sensors/manual_streaming/stream_realsense.py
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Optional / hardware-dependent imports
# ---------------------------------------------------------------------------

try:
    import pyrealsense2 as rs  # type: ignore
    _HAS_RS = True
except ImportError:
    _HAS_RS = False

try:
    import mediapipe as mp  # type: ignore
    _HAS_MP = True
except ImportError:
    _HAS_MP = False


# ---------------------------------------------------------------------------
# ALL 33 MediaPipe Pose landmark indices
# ---------------------------------------------------------------------------

JOINT_IDS: Dict[str, int] = {
    # Face
    "nose":                0,
    "left_eye_inner":      1,
    "left_eye":            2,
    "left_eye_outer":      3,
    "right_eye_inner":     4,
    "right_eye":           5,
    "right_eye_outer":     6,
    "left_ear":            7,
    "right_ear":           8,
    "mouth_left":          9,
    "mouth_right":         10,
    # Arms & hands
    "left_shoulder":       11,
    "right_shoulder":      12,
    "left_elbow":          13,
    "right_elbow":         14,
    "left_wrist":          15,
    "right_wrist":         16,
    "left_pinky":          17,
    "right_pinky":         18,
    "left_index":          19,
    "right_index":         20,
    "left_thumb":          21,
    "right_thumb":         22,
    # Hips / waist
    "left_hip":            23,
    "right_hip":           24,
    # Legs
    "left_knee":           25,
    "right_knee":          26,
    "left_ankle":          27,
    "right_ankle":         28,
    "left_heel":           29,
    "right_heel":          30,
    "left_foot_index":     31,
    "right_foot_index":    32,
}

# Reverse map: index → name
IDX_TO_NAME: Dict[int, str] = {v: k for k, v in JOINT_IDS.items()}

# Full-body skeleton connections
SKELETON_PAIRS: List[Tuple[str, str]] = [
    # torso box
    ("left_shoulder",   "right_shoulder"),
    ("left_hip",        "right_hip"),
    ("left_shoulder",   "left_hip"),
    ("right_shoulder",  "right_hip"),
    # left arm
    ("left_shoulder",   "left_elbow"),
    ("left_elbow",      "left_wrist"),
    ("left_wrist",      "left_pinky"),
    ("left_wrist",      "left_index"),
    ("left_wrist",      "left_thumb"),
    ("left_pinky",      "left_index"),
    # right arm
    ("right_shoulder",  "right_elbow"),
    ("right_elbow",     "right_wrist"),
    ("right_wrist",     "right_pinky"),
    ("right_wrist",     "right_index"),
    ("right_wrist",     "right_thumb"),
    ("right_pinky",     "right_index"),
    # left leg
    ("left_hip",        "left_knee"),
    ("left_knee",       "left_ankle"),
    ("left_ankle",      "left_heel"),
    ("left_ankle",      "left_foot_index"),
    ("left_heel",       "left_foot_index"),
    # right leg
    ("right_hip",       "right_knee"),
    ("right_knee",      "right_ankle"),
    ("right_ankle",     "right_heel"),
    ("right_ankle",     "right_foot_index"),
    ("right_heel",      "right_foot_index"),
    # face
    ("nose",            "left_eye_inner"),
    ("nose",            "right_eye_inner"),
    ("left_eye_inner",  "left_eye"),
    ("left_eye",        "left_eye_outer"),
    ("left_eye_outer",  "left_ear"),
    ("right_eye_inner", "right_eye"),
    ("right_eye",       "right_eye_outer"),
    ("right_eye_outer", "right_ear"),
    ("nose",            "mouth_left"),
    ("nose",            "mouth_right"),
]

# Body-part groups → colour tint for skeleton drawing
_GROUP_COLORS: Dict[str, Tuple[int, int, int]] = {
    "face":    (200, 200, 200),  # light grey
    "arm":     (0,   255, 0),    # green
    "torso":   (0,   200, 255),  # yellow
    "leg":     (255, 100, 0),    # blue-ish
}

def _pair_color(a: str, b: str) -> Tuple[int, int, int]:
    def group(n: str) -> str:
        if any(k in n for k in ("eye", "ear", "mouth", "nose")):
            return "face"
        if any(k in n for k in ("shoulder", "elbow", "wrist", "pinky", "index", "thumb")):
            return "arm"
        if any(k in n for k in ("hip",)):
            return "torso"
        return "leg"
    ga, gb = group(a), group(b)
    return _GROUP_COLORS.get(ga if ga == gb else "torso", (180, 180, 180))

JOINT_RADIUS = 4
EDGE_COLOR   = (255, 180, 0)   # cyan-ish Canny tint


# ---------------------------------------------------------------------------
# RealSense helpers  (pattern follows stream_realsense.py)
# ---------------------------------------------------------------------------

def build_realsense_pipeline(
    width: int, height: int, fps: int
) -> Tuple["rs.pipeline", "rs.align", float]:
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        raise RuntimeError("No RealSense device found. Connect a camera and retry.")

    pipeline = rs.pipeline(ctx)
    config   = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)

    profile     = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align       = rs.align(rs.stream.color)

    intr = (
        profile.get_stream(rs.stream.color)
               .as_video_stream_profile()
               .get_intrinsics()
    )
    print(
        f"[camera] {profile.get_device().get_info(rs.camera_info.name)}"
        f"  {intr.width}x{intr.height}"
        f"  fx={intr.fx:.1f} fy={intr.fy:.1f}"
        f"  depth_scale={depth_scale:.6f} m/unit"
    )
    return pipeline, align, depth_scale


def get_aligned_frames(
    pipeline: "rs.pipeline",
    align:    "rs.align",
    spatial:  "rs.spatial_filter",
    temporal: "rs.temporal_filter",
) -> Tuple[np.ndarray, np.ndarray]:
    frames      = pipeline.wait_for_frames()
    aligned     = align.process(frames)
    depth_frame = aligned.get_depth_frame()
    color_frame = aligned.get_color_frame()

    if not depth_frame or not color_frame:
        return np.array([]), np.array([])

    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)

    color_bgr = np.asanyarray(color_frame.get_data())   # (H, W, 3) uint8
    depth_raw = np.asanyarray(depth_frame.get_data())   # (H, W)    uint16
    return color_bgr, depth_raw


# ---------------------------------------------------------------------------
# Pose detection
# ---------------------------------------------------------------------------

# Type alias for one landmark entry
LandmarkDict = Dict[str, object]  # {px, py, vis, world_xyz, depth_xyz}


def build_pose_detector() -> "mp.solutions.pose.Pose":
    return mp.solutions.pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        smooth_landmarks=True,
        enable_segmentation=False,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )


def detect_joints(
    pose_detector: "mp.solutions.pose.Pose",
    color_bgr:     np.ndarray,
) -> Optional[Dict[str, LandmarkDict]]:
    """Run MediaPipe Pose on one frame.

    Returns
    -------
    dict  { joint_name: { px, py, vis, world_xyz } }
      or  None if no pose detected.

    world_xyz is from MediaPipe's pose_world_landmarks:
      - origin at hip midpoint
      - x = right, y = up, z = toward viewer  (metres)
    depth_xyz is filled later by PoseRecorder.record() from the RealSense
    aligned depth frame.
    """
    h, w = color_bgr.shape[:2]
    rgb  = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    result = pose_detector.process(rgb)

    if not result.pose_landmarks:
        return None

    lm      = result.pose_landmarks.landmark
    wlm     = (result.pose_world_landmarks.landmark
               if result.pose_world_landmarks else None)

    joints: Dict[str, LandmarkDict] = {}
    for name, idx in JOINT_IDS.items():
        l  = lm[idx]
        px = max(0, min(w - 1, int(l.x * w)))
        py = max(0, min(h - 1, int(l.y * h)))

        world_xyz: Optional[List[float]] = None
        if wlm:
            wl        = wlm[idx]
            world_xyz = [float(wl.x), float(wl.y), float(wl.z)]

        joints[name] = {
            "px":        px,
            "py":        py,
            "vis":       round(float(l.visibility), 3),
            "world_xyz": world_xyz,   # MediaPipe world (metres, hip-centred)
            "depth_xyz": None,         # filled by recorder
        }

    return joints


# ---------------------------------------------------------------------------
# 3-D depth sampling
# ---------------------------------------------------------------------------

def fill_depth_xyz(
    joints:      Dict[str, LandmarkDict],
    depth_raw:   np.ndarray,
    depth_scale: float,
) -> None:
    """Sample RealSense aligned depth at each landmark pixel, in-place."""
    for data in joints.values():
        px, py = int(data["px"]), int(data["py"])
        z_raw  = int(depth_raw[py, px])
        if z_raw == 0 or z_raw == 65535:
            data["depth_xyz"] = None
        else:
            data["depth_xyz"] = [float(px), float(py), float(z_raw) * depth_scale]


# ---------------------------------------------------------------------------
# Drawing utilities
# ---------------------------------------------------------------------------

def draw_canny_overlay(canvas: np.ndarray, color_bgr: np.ndarray) -> None:
    gray  = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 120)
    canvas[edges > 0] = EDGE_COLOR


def draw_skeleton(
    canvas:               np.ndarray,
    joints:               Dict[str, LandmarkDict],
    visibility_threshold: float = 0.4,
) -> None:
    # bones
    for (a, b) in SKELETON_PAIRS:
        if a not in joints or b not in joints:
            continue
        da, db = joints[a], joints[b]
        if float(da["vis"]) < visibility_threshold:
            continue
        if float(db["vis"]) < visibility_threshold:
            continue
        color = _pair_color(a, b)
        cv2.line(
            canvas,
            (int(da["px"]), int(da["py"])),
            (int(db["px"]), int(db["py"])),
            color, 2,
        )

    # joint dots
    for name, data in joints.items():
        if float(data["vis"]) < visibility_threshold:
            continue
        px, py = int(data["px"]), int(data["py"])
        # larger dot for main structural joints
        r = 6 if name in (
            "left_shoulder", "right_shoulder",
            "left_elbow",    "right_elbow",
            "left_wrist",    "right_wrist",
            "left_hip",      "right_hip",
            "left_knee",     "right_knee",
            "left_ankle",    "right_ankle",
        ) else JOINT_RADIUS
        cv2.circle(canvas, (px, py), r, (0, 255, 0), -1)


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------

class PoseRecorder:
    """Collect full-body pose samples over a fixed duration."""

    def __init__(self, duration: float):
        self.duration = duration
        self.samples: List[Dict] = []
        self._start:  Optional[float] = None
        self._done    = False

    def start(self) -> None:
        self._start = time.perf_counter()
        print(f"[recorder] started — {len(JOINT_IDS)} landmarks, {self.duration:.1f} s")

    @property
    def is_recording(self) -> bool:
        if self._done or self._start is None:
            return False
        if (time.perf_counter() - self._start) >= self.duration:
            self._done = True
            print(f"[recorder] done — {len(self.samples)} samples")
            return False
        return True

    @property
    def elapsed(self) -> float:
        return 0.0 if self._start is None else time.perf_counter() - self._start

    def record(
        self,
        joints:      Dict[str, LandmarkDict],
        depth_raw:   np.ndarray,
        depth_scale: float,
    ) -> None:
        if not self.is_recording:
            return
        fill_depth_xyz(joints, depth_raw, depth_scale)
        self.samples.append({
            "t":      round(self.elapsed, 4),
            "joints": {
                name: {
                    "px":        data["px"],
                    "py":        data["py"],
                    "vis":       data["vis"],
                    "world_xyz": data["world_xyz"],
                    "depth_xyz": data["depth_xyz"],
                }
                for name, data in joints.items()
            },
        })

    def save(self, path: Path) -> None:
        suffix = path.suffix.lower()
        joint_order = sorted(JOINT_IDS.keys())

        if suffix in (".npy", ".npz"):
            # Flat float32 array: [t, j0_wx,wy,wz,vis,dx,dy,dz, j1_...] per row
            rows = []
            for s in self.samples:
                row = [s["t"]]
                for jname in joint_order:
                    jd = s["joints"].get(jname, {})
                    wx, wy, wz = (jd.get("world_xyz") or [0.0, 0.0, 0.0])
                    dx, dy, dz = (jd.get("depth_xyz") or [0.0, 0.0, 0.0])
                    row.extend([wx, wy, wz, float(jd.get("vis", 0.0)), dx, dy, dz])
                rows.append(row)
            arr = np.array(rows, dtype=np.float32)
            if suffix == ".npz":
                np.savez(path, data=arr, joint_order=np.array(joint_order))
            else:
                np.save(path, arr)
            print(f"[recorder] saved array {arr.shape} → {path}")

        else:
            payload = {
                "format":      "vision_module_v2",
                "joint_order": joint_order,
                "joint_ids":   {k: v for k, v in JOINT_IDS.items()},
                "samples":     self.samples,
            }
            path.write_text(json.dumps(payload, indent=2))
            print(f"[recorder] saved {len(self.samples)} samples → {path}")


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(
    width:      int   = 640,
    height:     int   = 480,
    fps:        int   = 30,
    duration:   float = 5.0,
    output:     Path  = Path("pose_recording.json"),
    no_display: bool  = False,
) -> None:
    if not _HAS_RS:
        raise SystemExit("pyrealsense2 not installed.  pip install pyrealsense2")
    if not _HAS_MP:
        raise SystemExit("mediapipe not installed.    pip install mediapipe")

    pipeline, align, depth_scale = build_realsense_pipeline(width, height, fps)
    spatial_filter  = rs.spatial_filter()
    temporal_filter = rs.temporal_filter()
    pose            = build_pose_detector()
    recorder        = PoseRecorder(duration)

    # camera warm-up
    for _ in range(10):
        pipeline.wait_for_frames()

    recorder.start()

    try:
        while True:
            color_bgr, depth_raw = get_aligned_frames(
                pipeline, align, spatial_filter, temporal_filter
            )
            if color_bgr.size == 0:
                continue

            joints = detect_joints(pose, color_bgr)

            if joints and recorder.is_recording:
                recorder.record(joints, depth_raw, depth_scale)

            if not no_display:
                canvas = color_bgr.copy()
                draw_canny_overlay(canvas, color_bgr)
                if joints:
                    draw_skeleton(canvas, joints)

                status = (
                    f"REC {recorder.elapsed:.1f}/{duration:.0f}s"
                    f"  {len(recorder.samples)} pts"
                    if recorder.is_recording
                    else "DONE — press q to exit"
                )
                cv2.putText(canvas, status, (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_raw, alpha=0.03),
                    cv2.COLORMAP_JET,
                )
                cv2.imshow("vision_module — full body + depth",
                           cv2.hconcat([canvas, depth_vis]))

                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            if not recorder.is_recording and no_display:
                break

    finally:
        pose.close()
        pipeline.stop()
        if not no_display:
            cv2.destroyAllWindows()

    if recorder.samples:
        recorder.save(output)
    else:
        print("[recorder] no samples — nothing saved.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Record all 33 body landmarks from RealSense RGBD feed.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--width",      type=int,   default=640)
    parser.add_argument("--height",     type=int,   default=480)
    parser.add_argument("--fps",        type=int,   default=30)
    parser.add_argument("--duration",   type=float, default=5.0,
                        help="Recording duration in seconds")
    parser.add_argument("--output",     type=Path,  default=Path("pose_recording.json"),
                        help="Output file (.json or .npy/.npz)")
    parser.add_argument("--no-display", action="store_true",
                        help="Headless mode — no OpenCV window")
    args = parser.parse_args()

    try:
        run(
            width=args.width,
            height=args.height,
            fps=args.fps,
            duration=args.duration,
            output=args.output,
            no_display=args.no_display,
        )
    except (RuntimeError, SystemExit) as err:
        sys.exit(str(err))
