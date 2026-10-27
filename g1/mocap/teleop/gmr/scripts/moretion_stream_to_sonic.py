#!/usr/bin/env python3
"""Moretion live streaming -> GMR retargeting -> SONIC teleop publisher.

Bridges a Moretion motion-capture streaming server (TCP via ZMQ STREAM or UDP
via raw socket) to the SONIC C++ deployment (``gear_sonic_deploy``) over
Protocol v1 (joint-based).

This script combines:
  * GMR's native streaming receiver (``MoretionStreamReceiver`` +
    ``MoretionStreamToGMR``) from ``moretion_stream_to_robot.py`` — no
    external ``moretion`` library required.
  * The SONIC ZMQ publisher architecture (threaded reader + steady-rate
    50 Hz output with interpolation) from ``moretion_zmq_server.py``.

The ZMQ wire-format builders are vendored inline so that the script is
self-contained (no ``gear_sonic`` dependency).

Wire format (Protocol v1 — identical to ``moretion_zmq_server.py``):

Fields sent on each ``pose`` message:
    joint_pos   [N, 29]  f64  — G1 joint positions (IsaacLab order)
    joint_vel   [N, 29]  f64  — G1 joint velocities (IsaacLab order)
    body_quat_w [N,  4]  f64  — Body quaternion (w, x, y, z)
    frame_index [N]      i64  — Monotonically increasing frame indices
    catch_up    [1]      u8   — 1 = enable merger gap-reset (real-time streaming)
    root_pos    [N,  3]  f64  — Root / pelvis position in world frame
    live_frame_index [1] i64  — Current sample, before held future references

Usage examples::

    # TCP (default), with MuJoCo visualization
    python scripts/moretion_stream_to_sonic.py --vis \\
        --server_ip 192.168.2.92 --port 9999 \\
        --sonic_host "*" --sonic_port 5556

    # UDP source
    python scripts/moretion_stream_to_sonic.py --use_udp \\
        --server_ip 192.168.2.92 --port 9999
"""

import argparse
import json
import math
import os
import signal
import struct
import sys
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import zmq
from rich import print

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.moretion_stream import (
    MoretionStreamReceiver,
    MoretionStreamToGMR,
)

G1_NUM_JOINTS = 29

# ---------------------------------------------------------------------------
# Joint-order mapping (GMR MuJoCo order -> SONIC IsaacLab order)
# ---------------------------------------------------------------------------
GMR_G1_29DOF_ORDER: tuple[str, ...] = (
    "left_hip_pitch_joint",      # 0
    "left_hip_roll_joint",       # 1
    "left_hip_yaw_joint",        # 2
    "left_knee_joint",           # 3
    "left_ankle_pitch_joint",    # 4
    "left_ankle_roll_joint",     # 5
    "right_hip_pitch_joint",     # 6
    "right_hip_roll_joint",      # 7
    "right_hip_yaw_joint",       # 8
    "right_knee_joint",          # 9
    "right_ankle_pitch_joint",   # 10
    "right_ankle_roll_joint",    # 11
    "waist_yaw_joint",           # 12
    "waist_roll_joint",          # 13
    "waist_pitch_joint",         # 14
    "left_shoulder_pitch_joint", # 15
    "left_shoulder_roll_joint",  # 16
    "left_shoulder_yaw_joint",   # 17
    "left_elbow_joint",          # 18
    "left_wrist_roll_joint",     # 19
    "left_wrist_pitch_joint",    # 20
    "left_wrist_yaw_joint",      # 21
    "right_shoulder_pitch_joint",# 22
    "right_shoulder_roll_joint", # 23
    "right_shoulder_yaw_joint",  # 24
    "right_elbow_joint",         # 25
    "right_wrist_roll_joint",    # 26
    "right_wrist_pitch_joint",   # 27
    "right_wrist_yaw_joint",     # 28
)

ISAACLAB_G1_29DOF_ORDER: tuple[str, ...] = (
    "left_hip_pitch_joint",      # 0
    "right_hip_pitch_joint",     # 1
    "waist_yaw_joint",           # 2
    "left_hip_roll_joint",       # 3
    "right_hip_roll_joint",      # 4
    "waist_roll_joint",          # 5
    "left_hip_yaw_joint",        # 6
    "right_hip_yaw_joint",       # 7
    "waist_pitch_joint",         # 8
    "left_knee_joint",           # 9
    "right_knee_joint",          # 10
    "left_shoulder_pitch_joint", # 11
    "right_shoulder_pitch_joint",# 12
    "left_ankle_pitch_joint",    # 13
    "right_ankle_pitch_joint",   # 14
    "left_shoulder_roll_joint",  # 15
    "right_shoulder_roll_joint", # 16
    "left_ankle_roll_joint",     # 17
    "right_ankle_roll_joint",    # 18
    "left_shoulder_yaw_joint",   # 19
    "right_shoulder_yaw_joint",  # 20
    "left_elbow_joint",          # 21
    "right_elbow_joint",         # 22
    "left_wrist_roll_joint",     # 23
    "right_wrist_roll_joint",    # 24
    "left_wrist_pitch_joint",    # 25
    "right_wrist_pitch_joint",   # 26
    "left_wrist_yaw_joint",      # 27
    "right_wrist_yaw_joint",     # 28
)


def build_gmr_to_isaaclab_permutation(
    gmr_order: tuple[str, ...] = GMR_G1_29DOF_ORDER,
    isaac_order: tuple[str, ...] = ISAACLAB_G1_29DOF_ORDER,
) -> np.ndarray:
    """Return permutation ``perm`` such that
    ``joint_pos_isaac = joint_pos_gmr[perm]``.
    """
    if set(gmr_order) != set(isaac_order):
        missing_in_gmr = set(isaac_order) - set(gmr_order)
        missing_in_isaac = set(gmr_order) - set(isaac_order)
        raise ValueError(
            "Joint-name mismatch between GMR and IsaacLab orderings. "
            f"Missing in GMR: {sorted(missing_in_gmr)}. "
            f"Missing in IsaacLab: {sorted(missing_in_isaac)}."
        )
    gmr_index = {name: i for i, name in enumerate(gmr_order)}
    return np.array([gmr_index[name] for name in isaac_order], dtype=np.int64)


_DEFAULT_PERM: np.ndarray = build_gmr_to_isaaclab_permutation()


# ---------------------------------------------------------------------------
# Vendored ZMQ wire-format builders (from gear_sonic zmq_planner_sender.py)
# ---------------------------------------------------------------------------
HEADER_SIZE = 1280


def _build_header(fields: list, version: int = 1, count: int = 1) -> bytes:
    header = {
        "v": version,
        "endian": "le",
        "count": count,
        "fields": fields,
    }
    header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(header_json) > HEADER_SIZE:
        raise ValueError(f"Header too large: {len(header_json)} > {HEADER_SIZE}")
    return header_json.ljust(HEADER_SIZE, b"\x00")


def build_command_message(
    start: bool, stop: bool, planner: bool, delta_heading: float | None = None
) -> bytes:
    """Assemble a 'command' topic message."""
    fields = [
        {"name": "start", "dtype": "u8", "shape": [1]},
        {"name": "stop", "dtype": "u8", "shape": [1]},
        {"name": "planner", "dtype": "u8", "shape": [1]},
    ]
    payload = b"".join(
        (
            struct.pack("B", 1 if start else 0),
            struct.pack("B", 1 if stop else 0),
            struct.pack("B", 1 if planner else 0),
        )
    )
    if delta_heading is not None:
        fields.append({"name": "delta_heading", "dtype": "f32", "shape": [1]})
        payload += struct.pack("<f", float(delta_heading))
    header = _build_header(fields, version=1, count=1)
    return b"command" + header + payload


# ---------------------------------------------------------------------------
# Interpolation helper (upsampling low-rate sources to SONIC's 50 Hz)
# ---------------------------------------------------------------------------
def _quat_lerp_normalized(q1: np.ndarray, q2: np.ndarray, alpha: float) -> np.ndarray:
    """Normalized linear interpolation of quaternions (w, x, y, z)."""
    if np.dot(q1, q2) < 0.0:
        q2 = -q2
    q = (1.0 - alpha) * q1 + alpha * q2
    norm = np.linalg.norm(q)
    return q / norm if norm > 0.0 else q


# =====================================================================
# SONIC ZMQ publisher
# =====================================================================
class SonicZmqPublisher:
    """ZMQ PUB socket wrapper that sends SONIC-compatible Protocol v1 messages.

    Binds to *host:port* and publishes on ``command`` and ``pose`` topics.
    The C++ deployment side (``gear_sonic_deploy``) subscribes via
    ``ZMQManager`` / ``ZMQPackedMessageSubscriber``.

    Args:
        host: Bind address.  Use ``"*"`` to bind all interfaces.
        port: TCP port number (default ``5556``).
        verbose: Print diagnostic messages.
    """

    def __init__(self, host: str = "*", port: int = 5556, verbose: bool = True):
        self._ctx = zmq.Context()
        self._socket = self._ctx.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 3)   # Bound pending messages (PUB drops new sends at HWM)
        self._socket.setsockopt(zmq.IMMEDIATE, 1)  # Don't queue for incomplete conns
        self._socket.setsockopt(zmq.LINGER, 0)   # Don't block on close
        self._endpoint = f"tcp://{host}:{port}"
        self._socket.bind(self._endpoint)
        self._verbose = verbose

        # Allow subscribers time to connect before first message.
        time.sleep(0.5)

        if self._verbose:
            print(f"[SonicZmqPublisher] Bound to {self._endpoint}")
            print(f"[SonicZmqPublisher] Topics: command, pose")

    def send_command(
        self,
        start: bool,
        stop: bool,
        planner: bool,
        delta_heading: float | None = None,
    ) -> None:
        """Send a ``command`` topic message."""
        msg = build_command_message(start, stop, planner, delta_heading)
        self._socket.send(msg)
        if self._verbose:
            extra = f", delta_heading={delta_heading:.3f}" if delta_heading is not None else ""
            print(f"[Command] start={start}, stop={stop}, planner={planner}{extra}")

    def send_pose(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        body_quat: np.ndarray,
        frame_indices: np.ndarray,
        catch_up: bool = True,
        root_pos: np.ndarray | None = None,
        live_frame_index: int | None = None,
    ) -> None:
        """Send a ``pose`` topic message using Protocol v1 (joint-based).

        Args:
            joint_pos: ``[N, 29]`` joint positions in IsaacLab order.
            joint_vel: ``[N, 29]`` joint velocities in IsaacLab order.
            body_quat: ``[N, 4]`` body quaternion ``(w, x, y, z)`` per frame.
            frame_indices: ``[N]`` monotonically increasing frame indices.
            catch_up: If ``True``, reset playback when a large frame gap is
                detected. Real-time streams should keep this enabled so a
                dropped message cannot leave playback behind indefinitely.
            root_pos: Optional ``[N, 3]`` root / pelvis position in world frame.
            live_frame_index: Global index to track immediately. Omit for
                buffered trajectory playback; live packets hold future poses.
        """
        n_frames, n_joints = joint_pos.shape
        assert n_joints == G1_NUM_JOINTS, (
            f"joint_pos must have {G1_NUM_JOINTS} joints, got {n_joints}"
        )
        assert joint_vel.shape == (n_frames, G1_NUM_JOINTS), "joint_vel shape mismatch"
        assert body_quat.shape == (n_frames, 4), "body_quat shape mismatch"
        assert frame_indices.shape == (n_frames,), "frame_indices shape mismatch"

        fields = [
            {"name": "joint_pos", "dtype": "f64", "shape": [n_frames, G1_NUM_JOINTS]},
            {"name": "joint_vel", "dtype": "f64", "shape": [n_frames, G1_NUM_JOINTS]},
            {"name": "body_quat_w", "dtype": "f64", "shape": [n_frames, 4]},
            {"name": "frame_index", "dtype": "i64", "shape": [n_frames]},
            {"name": "catch_up", "dtype": "u8", "shape": [1]},
        ]

        payload = b"".join(
            (
                np.ascontiguousarray(joint_pos).tobytes(),
                np.ascontiguousarray(joint_vel).tobytes(),
                np.ascontiguousarray(body_quat).tobytes(),
                np.ascontiguousarray(frame_indices).tobytes(),
                struct.pack("B", 1 if catch_up else 0),
            )
        )

        if root_pos is not None:
            fields.append({"name": "root_pos", "dtype": "f64", "shape": [n_frames, 3]})
            payload += np.ascontiguousarray(root_pos).tobytes()

        if live_frame_index is not None:
            fields.append({"name": "live_frame_index", "dtype": "i64", "shape": [1]})
            payload += struct.pack("<q", live_frame_index)

        header = _build_header(fields, version=1, count=n_frames)
        message = b"pose" + header + payload
        self._socket.send(message)

    def close(self) -> None:
        self._socket.close()
        self._ctx.term()
        if self._verbose:
            print("[SonicZmqPublisher] Closed")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# =====================================================================
# Threaded source frame container & frame buffer
# =====================================================================
class _LatestSourceFrame:
    """Thread-safe 2-slot buffer for the last two retargeted source frames.

    The reader thread appends; the output loop reads the last 2
    non-blocking.  When source rate > output rate, intermediate frames
    are auto-evicted, so ``dt_source`` always reflects the true
    inter-frame period.
    """

    __slots__ = ("_lock", "_buf", "_count")

    def __init__(self):
        self._lock = threading.Lock()
        self._buf: deque = deque(maxlen=2)
        self._count = 0

    def update(
        self,
        joint_pos: np.ndarray,
        body_quat: np.ndarray,
        root_pos: np.ndarray,
        timestamp: float,
    ) -> None:
        with self._lock:
            self._buf.append((joint_pos, body_quat, root_pos, timestamp))
            self._count += 1

    def get_last_2(self) -> tuple:
        """Return ``(frames, count)`` where *frames* is a list of up to 2
        tuples ``(jp, bq, rp, t)`` (oldest-first) and *count* is the total
        number of frames ever written.  Returns ``([], 0)`` if empty.
        """
        with self._lock:
            if not self._buf:
                return [], 0
            return list(self._buf), self._count


class _GmrFrameBuffer:
    """Keep a rolling window of frames for ZMQ pose messages.

    Once full, every new frame produces a window advanced by one frame.  The
    overlapping indices let SONIC refresh its future-reference buffer at the
    control rate without advancing the global motion timeline more than once.
    """

    __slots__ = ("size", "frames")

    def __init__(self, size: int):
        if size < 1:
            raise ValueError(f"batch size must be >= 1, got {size}")
        self.size = size
        self.frames: deque = deque(maxlen=size)

    def push(self, frame: dict) -> None:
        self.frames.append(frame)

    def resize(self, size: int) -> None:
        """Keep the latest samples when changing the preview window."""
        if size < 1:
            raise ValueError(f"batch size must be >= 1, got {size}")
        self.size = size
        self.frames = deque(self.frames, maxlen=size)

    def get_full_window(self) -> Optional[list[dict]]:
        """Return the current window, or ``None`` while it is still filling."""
        if len(self.frames) < self.size:
            return None
        return list(self.frames)


def _build_pose_batch(
    frames: list[dict],
    output_period: float,
    previous_joint_pos: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Stack a pose window and compute training-aligned joint velocities."""
    joint_pos = np.stack([frame["joint_pos"] for frame in frames], axis=0)
    frame_indices = np.array(
        [frame["frame_index"] for frame in frames], dtype=np.int64
    )
    joint_vel = np.zeros_like(joint_pos)
    if len(frames) > 1:
        intervals = np.diff(frame_indices) * output_period
        if np.any(intervals <= 0):
            raise ValueError("frame indices must increase strictly")
        joint_vel[:-1] = (joint_pos[1:] - joint_pos[:-1]) / intervals[:, None]
        # The newest sample has no causal forward difference yet.  It is
        # corrected when it becomes an interior sample in the next window.
        joint_vel[-1] = joint_vel[-2]
    elif previous_joint_pos is not None:
        # Compatibility path only.  A one-frame packet cannot provide the
        # future-reference window expected by the SONIC joint encoder.
        joint_vel[0] = (joint_pos[0] - previous_joint_pos) / output_period

    body_quat = np.stack([frame["body_quat"] for frame in frames], axis=0)
    root_pos = np.stack([frame["root_pos"] for frame in frames], axis=0)
    return joint_pos, joint_vel, body_quat, root_pos, frame_indices


def _build_live_pose_batch(
    frames: list[dict],
    output_period: float,
    future_frames: int = 50,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Keep measured history, then hold the newest pose for unknown future.

    The current sample is frames[-1], not the start of the history window.
    Fifty samples including the current one cover both released G1 encoders
    (10 at step5, or 10 at step1) without waiting for future capture frames.
    Future holds have zero velocity; the current sample keeps its measured
    finite difference. No motion is extrapolated beyond the received pose.
    """
    if future_frames < 1:
        raise ValueError("future_frames must be >= 1")
    joint_pos, joint_vel, body_quat, root_pos, indices = _build_pose_batch(
        frames, output_period
    )
    held_count = future_frames - 1
    return (
        np.concatenate((joint_pos, np.repeat(joint_pos[-1:], held_count, axis=0))),
        np.concatenate((joint_vel, np.zeros((held_count, G1_NUM_JOINTS)))),
        np.concatenate((body_quat, np.repeat(body_quat[-1:], held_count, axis=0))),
        np.concatenate((root_pos, np.repeat(root_pos[-1:], held_count, axis=0))),
        np.concatenate((indices, indices[-1] + np.arange(1, future_frames))),
    )


def _reference_timing(
    model_profile: str, output_period: float, preview_delay_ms: Optional[float]
) -> tuple[int, int]:
    """Return encoder horizon and bounded preview delay in output ticks.

    Include one extra measured sample for the final forward velocity. This
    preserves both positions and velocities at all ten encoder timestamps.
    """
    future_frames = {"default": 46, "low_latency": 10}[model_profile]
    if preview_delay_ms is None:
        return future_frames, future_frames
    if not math.isfinite(preview_delay_ms) or preview_delay_ms < 0:
        raise ValueError("preview_delay_ms must be finite and >= 0")
    delay_frames = math.ceil(preview_delay_ms / (output_period * 1000) - 1e-9)
    return future_frames, delay_frames


def _build_preview_pose_batch(
    frames: list[dict],
    output_period: float,
    future_frames: int,
    delay_frames: int,
    history_frames: int = 2,
) -> Optional[tuple[tuple[np.ndarray, ...], int]]:
    """Track a bounded delayed sample, with measured future motion first.

    No output is produced until the requested preview exists. In particular,
    startup does not first track the newest pose and later move backward.
    Shorter custom delays pad only the missing future samples with a hold.
    history_frames includes the target: two retains one past frame for the
    zero-preview velocity estimate, without shortening the encoder horizon.
    """
    if future_frames < 1 or delay_frames < 0 or history_frames < 1:
        raise ValueError("invalid reference window dimensions")
    if len(frames) <= delay_frames:
        return None
    current_offset = len(frames) - 1 - delay_frames
    start = max(0, current_offset - history_frames + 1)
    window = frames[start:]
    joint_pos, joint_vel, body_quat, root_pos, indices = _build_pose_batch(
        window, output_period
    )
    current_index = frames[current_offset]["frame_index"]
    # Keep at most the encoder horizon plus a derivative sample. Longer
    # preview settings must not let the deployment cursor creep forward.
    count = min(len(window), current_offset - start + future_frames + 1)
    joint_pos, joint_vel = joint_pos[:count], joint_vel[:count]
    body_quat, root_pos, indices = body_quat[:count], root_pos[:count], indices[:count]
    held_count = max(0, current_index + future_frames - 1 - int(indices[-1]))
    batch = (
        np.concatenate((joint_pos, np.repeat(joint_pos[-1:], held_count, axis=0))),
        np.concatenate((joint_vel, np.zeros((held_count, G1_NUM_JOINTS)))),
        np.concatenate((body_quat, np.repeat(body_quat[-1:], held_count, axis=0))),
        np.concatenate((root_pos, np.repeat(root_pos[-1:], held_count, axis=0))),
        np.concatenate((indices, indices[-1] + np.arange(1, held_count + 1))),
    )
    return batch, current_index


# =====================================================================
# Global graceful-shutdown flag
# =====================================================================
g_running = True


def signal_handler(signum, frame):
    global g_running
    print(f"\nReceived signal {signum}, shutting down...")
    g_running = False


# =====================================================================
# Main teleop loop
# =====================================================================
class _ReferenceSettingsUpdates:
    """Coalesce GUI updates; only the output thread changes the reference buffer."""

    def __init__(self, output_period: float):
        self._output_period = output_period
        self._lock = threading.Lock()
        self._pending = None

    def read_stream(self, stream) -> None:
        for line in stream:
            try:
                settings = json.loads(line)
                profile = settings["model_profile"]
                delay_ms = settings["preview_delay_ms"]
                horizon, ticks = _reference_timing(profile, self._output_period, delay_ms)
            except (ValueError, KeyError, TypeError) as e:
                print(f"[Reference] Ignoring invalid settings: {e}")
                continue
            with self._lock:
                self._pending = (profile, delay_ms, horizon, ticks)

    def take_latest(self):
        with self._lock:
            pending = self._pending
            self._pending = None
        return pending


def run_moretion_to_sonic(args: argparse.Namespace) -> int:
    global g_running

    if args.batch < 1:
        print(f"[red]--batch must be at least 1, got {args.batch}[/red]")
        return 2
    if args.live_future_frames < 1:
        print("[red]--live_future_frames must be at least 1[/red]")
        return 2
    output_period = 1.0 / max(1, args.sonic_fps)
    try:
        reference_horizon, preview_ticks = _reference_timing(
            args.model_profile, output_period, args.preview_delay_ms
        )
    except ValueError as e:
        print(f"[red]{e}[/red]")
        return 2
    reference_updates = _ReferenceSettingsUpdates(output_period)
    if args.reference_control_stdin:
        threading.Thread(
            # Raw input avoids holding a buffered stdin lock during shutdown.
            target=reference_updates.read_stream, args=(sys.stdin.buffer.raw,), daemon=True,
        ).start()
    if args.batch == 1 and args.reference_mode == "buffered":
        print(
            "[yellow]Warning: --batch=1 is a diagnostic compatibility mode. "
            "The SONIC Protocol v1 encoder expects a multi-frame reference "
            "window; use --batch=5 for stable tracking.[/yellow]"
        )

    perm = _DEFAULT_PERM

    # ------------------------------------------------------------------
    # 1. Initialize stream receiver and frame converter
    # ------------------------------------------------------------------
    print("[1/4] Initializing moretion stream receiver...")
    receiver = MoretionStreamReceiver(
        host=args.server_ip,
        port=args.port,
        use_udp=args.use_udp,
        timeout=args.timeout,
        device_id=args.device_id,
    )

    print("[2/4] Initializing frame converter...")
    converter = MoretionStreamToGMR(
        ref_bvh_file=args.ref_bvh,
        scale=args.scale,
        verbose=True,
        device_id=args.device_id,
    )

    # ------------------------------------------------------------------
    # 2. Connect and wait for first frame to determine human height
    # ------------------------------------------------------------------
    print(
        f"[3/4] Connecting to {args.server_ip}:{args.port} "
        f"({'UDP' if args.use_udp else 'TCP'})..."
    )
    receiver.connect()
    time.sleep(0.5)

    print("Waiting for first frame...")
    first_frame = None
    first_frame_time: float = 0.0
    while g_running and first_frame is None:
        raw = receiver.recv_frame()
        if raw is None:
            continue
        first_frame_time = time.monotonic()
        try:
            first_frame = converter.convert_frame(raw)
        except Exception as e:
            print(f"  First frame conversion failed: {e}, retrying...")
            continue

    if not g_running:
        receiver.close()
        return 0

    # Determine human height
    if args.human_height is not None:
        human_height = args.human_height
    else:
        human_height = converter.get_human_height(first_frame)
    print(f"  Human height: {human_height:.3f} m")

    # ------------------------------------------------------------------
    # 3. Initialize retargeter and (optional) viewer
    # ------------------------------------------------------------------
    print("[4/4] Initializing retargeter and SONIC publisher...")
    retargeter = GMR(
        src_human="bvh_moretion",
        tgt_robot=args.robot,
        actual_human_height=human_height,
        solver="daqp",
        damping=1.0,
        use_velocity_limit=True,
    )

    viewer = None
    esc_pressed = False

    def _on_viewer_key(keycode):
        nonlocal esc_pressed
        if keycode == 256:  # GLFW_KEY_ESCAPE
            esc_pressed = True

    if args.vis:
        viewer = RobotMotionViewer(
            robot_type=args.robot,
            motion_fps=args.motion_fps,
            transparent_robot=0,
            record_video=args.record_video,
            video_path=args.video_path,
            keyboard_callback=_on_viewer_key,
        )

    # Viewer throttle: render at most once per viewer_period to avoid
    # blocking the reader thread at high source frame rates.
    _viewer_period = 1.0 / max(1, args.motion_fps)
    _last_viewer_render = 0.0

    # ------------------------------------------------------------------
    # 4. Threaded source reader + steady-rate 50 Hz output loop
    # ------------------------------------------------------------------
    target_fps = max(1, args.sonic_fps)
    output_period = 1.0 / target_fps  # 20 ms for 50 Hz
    buf = _GmrFrameBuffer(
        size=args.batch + preview_ticks
        if args.reference_mode == "preview"
        else args.batch
    )

    # Interpolation endpoints (updated when a new source frame arrives).
    src_prev_joint_pos: Optional[np.ndarray] = None
    src_prev_body_quat: Optional[np.ndarray] = None
    src_prev_root_pos: Optional[np.ndarray] = None
    src_prev_time: Optional[float] = None

    src_curr_joint_pos: Optional[np.ndarray] = None
    src_curr_body_quat: Optional[np.ndarray] = None
    src_curr_root_pos: Optional[np.ndarray] = None
    src_curr_time: Optional[float] = None

    # Output timing state.
    next_output_time = time.monotonic()
    global_frame_idx = 0
    send_count = 0
    fps_counter = 0
    source_fps_counter = 0
    last_src_count = 0
    fps_start = time.monotonic()

    dropped_frames = 0

    # Previous generated joint_pos for the batch=1 compatibility path.
    prev_sent_joint_pos: Optional[np.ndarray] = None

    latest = _LatestSourceFrame()
    stop_event = threading.Event()

    def _reader_loop() -> None:
        """Background reader: receive -> convert -> retarget at source rate."""
        nonlocal dropped_frames

        # Retarget the first frame we already received.
        try:
            qpos = retargeter.retarget(first_frame)
            _publish_frame(qpos, latest, viewer, perm, first_frame_time)
        except Exception as e:
            print(f"[Reader] First frame retargeting failed: {e}")

        while not stop_event.is_set() and g_running:
            raw = receiver.recv_frame()
            if raw is None:
                dropped_frames += 1
                time.sleep(0.001)
                continue
            # Timestamp at receipt, before convert/retarget, so dt_source
            # reflects the true source frame interval without processing
            # jitter.
            frame_time = time.monotonic()
            try:
                human_frame = converter.convert_frame(raw)
                qpos = retargeter.retarget(human_frame)
            except Exception as e:
                print(f"[Reader] Frame convert/retarget failed: {e}, skipping")
                dropped_frames += 1
                time.sleep(0.01)
                continue

            _publish_frame(qpos, latest, viewer, perm, frame_time)

    def _publish_frame(qpos, latest_container, vwr, permutation, frame_time):
        """Convert qpos to IsaacLab order and stash in the latest container.

        Also step the MuJoCo viewer (in the reader thread so rendering never
        blocks the 50 Hz output loop).  Viewer rendering is throttled to
        ``motion_fps`` via a time-based skip — no blocking sleep — so the
        reader thread always runs at the full source rate.

        ``latest_container.update()`` is called *before* ``vwr.step()`` so
        that data reaches the output loop immediately after retargeting,
        without waiting for a potentially slow render.
        """
        nonlocal _last_viewer_render

        new_joint_pos = qpos[7:][permutation]
        new_body_quat = qpos[3:7]
        new_root_pos = qpos[0:3]

        latest_container.update(
            new_joint_pos,
            new_body_quat,
            new_root_pos,
            frame_time,
        )

        if vwr is not None:
            if frame_time - _last_viewer_render >= _viewer_period:
                _last_viewer_render = frame_time
                try:
                    vwr.step(
                        root_pos=qpos[:3],
                        root_rot=qpos[3:7],  # [w, x, y, z]
                        dof_pos=qpos[7:],
                        human_motion_data=retargeter.scaled_human_data,
                        rate_limit=False,  # throttled above, no blocking sleep
                        follow_camera=True,
                    )
                except Exception as e:
                    print(f"[Viewer] step failed: {e}")

    reader_thread = threading.Thread(target=_reader_loop, daemon=True)
    reader_thread.start()

    # ------------------------------------------------------------------
    # SONIC publisher: send start command, then stream poses at 50 Hz
    # ------------------------------------------------------------------
    with SonicZmqPublisher(
        host=args.sonic_host, port=args.sonic_port, verbose=args.verbose
    ) as pub:
        # ZMQ PUB/SUB has no delivery guarantee for the first message while
        # the subscriber is still completing its handshake.  SONIC requires
        # this command to leave WAIT_FOR_CONTROL, so retry it briefly during
        # startup.  Repeated start=True commands are idempotent.
        for _ in range(5):
            pub.send_command(start=True, stop=False, planner=False)
            time.sleep(0.5)

        interp_mode = (
            "buffered (one source period)"
            if args.buffered_interp
            else "zero-order hold (latest frame)"
        )
        print(
            f"Streaming to SONIC deploy at tcp://{args.sonic_host}:{args.sonic_port} | "
            f"target_fps={target_fps}, batch={args.batch}, interp={interp_mode}"
            f", reference={args.reference_mode}"
        )
        if args.reference_mode == "preview":
            print(
                f"Preview: model={args.model_profile}, "
                f"delay={preview_ticks * output_period * 1000:.0f} ms, "
                f"measured reference frames={min(preview_ticks + 1, reference_horizon)}"
                f"/{reference_horizon}"
            )
        print("Press Ctrl+C (or ESC in viewer) to stop\n")

        try:
            while not esc_pressed and g_running:
                update = reference_updates.take_latest()
                if update is not None:
                    args.model_profile, args.preview_delay_ms, reference_horizon, preview_ticks = update
                    if args.reference_mode == "preview":
                        buf.resize(args.batch + preview_ticks)
                    print(
                        f"[Reference] Updated: model={args.model_profile}, "
                        f"delay={preview_ticks * output_period * 1000:.0f} ms"
                    )
                # -- 1. Timing: steady target_fps tick ----------------------
                now = time.monotonic()
                if next_output_time < now - output_period:
                    # Keep indices tied to elapsed time after a scheduler stall.
                    skipped_ticks = int((now - next_output_time) / output_period)
                    global_frame_idx += skipped_ticks
                    next_output_time += skipped_ticks * output_period
                    # SONIC infers a constant stride from the first two indices.
                    # Start fresh rather than mixing a gap into a step1 window.
                    buf.frames.clear()
                if now < next_output_time:
                    sleep_t = next_output_time - now
                    if sleep_t > 0.002:
                        time.sleep(sleep_t - 0.001)  # Leave 1ms margin
                    now = time.monotonic()

                # -- 2. Drain all new source frames (non-blocking) ----------
                new_frames, src_count = latest.get_last_2()
                if src_count > last_src_count:
                    source_fps_counter += src_count - last_src_count
                    last_src_count = src_count
                    if len(new_frames) >= 2:
                        # Two or more frames in buffer: use the last two as
                        # prev/curr so that dt_source reflects the true
                        # inter-frame period (no skipped frames).
                        (
                            src_prev_joint_pos,
                            src_prev_body_quat,
                            src_prev_root_pos,
                            src_prev_time,
                        ) = new_frames[-2]
                        (
                            src_curr_joint_pos,
                            src_curr_body_quat,
                            src_curr_root_pos,
                            src_curr_time,
                        ) = new_frames[-1]
                    elif len(new_frames) == 1:
                        # First source frame (only one in buffer so far).
                        (
                            src_curr_joint_pos,
                            src_curr_body_quat,
                            src_curr_root_pos,
                            src_curr_time,
                        ) = new_frames[0]

                # -- 3. Interpolate / extrapolate one output frame ----------
                p_jp = src_prev_joint_pos
                c_jp = src_curr_joint_pos
                p_bq = src_prev_body_quat
                c_bq = src_curr_body_quat
                p_rp = src_prev_root_pos
                c_rp = src_curr_root_pos
                p_t = src_prev_time
                c_t = src_curr_time

                if (
                    p_jp is not None
                    and c_jp is not None
                    and p_bq is not None
                    and c_bq is not None
                    and p_rp is not None
                    and c_rp is not None
                    and p_t is not None
                    and c_t is not None
                ):
                    dt_source = c_t - p_t
                    if dt_source > 1e-6:
                        # When --buffered_interp is on, shift the render time
                        # back by one source period so that it falls inside
                        # [p_t, c_t], producing a true blend (alpha in (0, 1)).
                        # This adds ~one source period of latency but yields
                        # smooth motion.  When off, render_time == next_output_time
                        # >= c_t, so alpha >= 1 is clamped to 1 (zero-order hold,
                        # matching the PICO reference).
                        delay = dt_source if args.buffered_interp else 0.0
                        render_time = next_output_time - delay
                        alpha = (render_time - p_t) / dt_source
                        alpha = max(0.0, min(1.0, alpha))
                    else:
                        alpha = 1.0
                    interp_joint_pos = (1.0 - alpha) * p_jp + alpha * c_jp
                    interp_root_pos = (1.0 - alpha) * p_rp + alpha * c_rp
                    interp_body_quat = _quat_lerp_normalized(p_bq, c_bq, alpha)
                elif c_jp is not None and c_bq is not None and c_rp is not None:
                    interp_joint_pos = c_jp
                    interp_body_quat = c_bq
                    interp_root_pos = c_rp
                else:
                    # No data yet; advance timing and skip.
                    next_output_time += output_period
                    continue

                buf.push(
                    {
                        "joint_pos": interp_joint_pos,
                        "body_quat": interp_body_quat,
                        "root_pos": interp_root_pos,
                        "frame_index": global_frame_idx,
                    }
                )
                global_frame_idx += 1

                # -- 4. Rolling-window send ---------------------------------
                # Preview packets mark a deliberately delayed sample and use
                # its measured future trajectory. The marker still discards
                # startup/transport backlog rather than replaying it.
                batch_frames = (
                    list(buf.frames)
                    if args.reference_mode in ("preview", "live")
                    else buf.get_full_window()
                )
                live_frame_index = None
                if args.reference_mode == "preview":
                    preview = _build_preview_pose_batch(
                        batch_frames, output_period, reference_horizon,
                        preview_ticks, history_frames=args.batch,
                    )
                    if preview is None:
                        next_output_time += output_period
                        continue
                    pose_batch, live_frame_index = preview
                elif batch_frames is not None:
                    pose_batch = (
                        _build_live_pose_batch(
                            batch_frames, output_period, args.live_future_frames
                        )
                        if args.reference_mode == "live"
                        else _build_pose_batch(
                            batch_frames, output_period,
                            previous_joint_pos=prev_sent_joint_pos,
                        )
                    )
                    if args.reference_mode == "live":
                        live_frame_index = batch_frames[-1]["frame_index"]
                if batch_frames is not None:
                    (
                        joint_pos_b,
                        joint_vel_b,
                        body_quat_b,
                        root_pos_b,
                        frame_indices_b,
                    ) = pose_batch
                    prev_sent_joint_pos = interp_joint_pos.copy()

                    pub.send_pose(
                        joint_pos=joint_pos_b,
                        joint_vel=joint_vel_b,
                        body_quat=body_quat_b,
                        frame_indices=frame_indices_b,
                        catch_up=True,
                        root_pos=root_pos_b,
                        live_frame_index=live_frame_index,
                    )
                    send_count += 1

                next_output_time += output_period
                fps_counter += 1

                # -- 5. FPS reporting --------------------------------------
                now2 = time.monotonic()
                duration = now2 - fps_start
                if duration >= 2.0:
                    out_fps = fps_counter / duration
                    src_fps = source_fps_counter / duration
                    print(
                        f"output FPS: {out_fps:.1f}, "
                        f"source FPS: {src_fps:.1f}, "
                        f"messages sent: {send_count}, "
                        f"frames dropped: {dropped_frames + receiver.dropped_count}, "
                        f"frames generated: {global_frame_idx}"
                    )
                    fps_counter = 0
                    fps_start = now2
                    source_fps_counter = 0
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            stop_event.set()
            reader_thread.join(timeout=1.0)
            pub.send_command(start=False, stop=True, planner=False)
            receiver.close()
            if viewer is not None:
                viewer.close()
            print(
                f"Total messages sent: {send_count}, "
                f"Total dropped: {dropped_frames + receiver.dropped_count}, "
                f"frames generated: {global_frame_idx}"
            )

    return 0


# =====================================================================
# CLI
# =====================================================================
def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Moretion live streaming -> GMR retargeting -> SONIC teleop publisher.\n\n"
            "Receives Moretion human motion via TCP/UDP, retargets to a G1 robot "
            "with GMR, and publishes joint poses to the SONIC C++ deployment "
            "over ZMQ Protocol v1."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ---- Moretion streaming source ----
    parser.add_argument(
        "--server_ip",
        type=str,
        default="192.168.2.92",
        help="Moretion streaming server IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9999,
        help="Moretion streaming server port",
    )
    parser.add_argument(
        "--use_udp",
        action="store_true",
        default=False,
        help="Use UDP instead of TCP to receive streaming data",
    )
    parser.add_argument(
        "--device_id",
        type=str,
        default=None,
        help="Device ID to select from deviceList (default: first device)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Receive timeout in seconds",
    )
    parser.add_argument(
        "--ref_bvh",
        type=str,
        default=None,
        help="Reference BVH for custom skeleton (default: built-in moretion skeleton)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.01,
        help="BVH scale (cm->m for offsets; streaming coordinate is already in metres)",
    )

    # ---- Retargeting ----
    parser.add_argument(
        "--robot",
        choices=[
            "unitree_g1",
            "unitree_g1_with_hands",
            "unitree_h1_2",
            "Q1",
            "X1",
        ],
        default="unitree_g1",
        help="Target robot (SONIC publishing currently supports G1 29-DOF)",
    )
    parser.add_argument(
        "--human_height",
        type=float,
        default=None,
        help="Actual human height in metres (auto-detected from first frame if omitted)",
    )

    # ---- SONIC ZMQ output ----
    parser.add_argument(
        "--sonic_host",
        type=str,
        default="*",
        help="SONIC ZMQ bind address (use '*' to bind all interfaces)",
    )
    parser.add_argument(
        "--sonic_port",
        type=int,
        default=5556,
        help="SONIC ZMQ port",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=2,
        help="Measured history-window size including the target (default 2: "
        "one past frame plus the target). Preview also stores "
        "the selected model's measured lookahead. Live sends from the first "
        "sample and tracks the newest sample. Buffered mode waits "
        "(batch-1)/sonic_fps and plays the window as a recorded trajectory.",
    )
    parser.add_argument(
        "--reference_mode", choices=("preview", "live", "buffered"), default="preview",
        help="Preview (default) preserves measured encoder lookahead with a "
        "bounded delay. Live holds all unknown future poses (may degrade gait). "
        "Buffered preserves historical trajectory playback for comparison.",
    )
    parser.add_argument(
        "--model_profile", choices=("default", "low_latency"), default="default",
        help="Match the deployed G1 encoder: step5 default or step1 low_latency.",
    )
    parser.add_argument(
        "--preview_delay_ms", type=float, default=None,
        help="Measured preview delay in ms, rounded up to control ticks. "
        "Omit for full encoder lookahead and forward velocities: default "
        "920 ms, low_latency 200 ms at 50 Hz. Shorter values hold missing "
        "future samples and may reduce gait fidelity; 0 matches live holding.",
    )
    parser.add_argument(
        "--reference_control_stdin", action="store_true",
        help="Accept GUI model/preview settings as JSON lines on stdin without restarting IK.",
    )
    parser.add_argument(
        "--live_future_frames", type=int, default=50,
        help="Future reference samples including the current pose in live mode. "
        "50 covers the default and low-latency G1 encoders without capture delay.",
    )
    parser.add_argument(
        "--sonic_fps",
        type=int,
        default=50,
        help="Target publish rate to SONIC (Hz). Must match SONIC control frequency (50 Hz)",
    )
    parser.add_argument(
        "--buffered_interp",
        action="store_true",
        default=False,
        help="Enable true frame-to-frame interpolation by buffering one source "
        "period. Adds ~one source period (~33 ms at 30 Hz) of latency but "
        "produces smooth motion between frames. When off (default), the "
        "output holds the latest frame (zero-order hold, matching PICO).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=True,
        help="Print diagnostic messages",
    )
    parser.add_argument(
        "--quiet",
        action="store_false",
        dest="verbose",
        help="Suppress log prints",
    )

    # ---- Visualization ----
    vis = parser.add_argument_group("Visualization options")
    vis.add_argument(
        "--vis",
        action="store_true",
        help="Open MuJoCo RobotMotionViewer window to visualize retargeted motion.",
    )
    vis.add_argument(
        "--motion_fps",
        type=int,
        default=60,
        help="Target render FPS for the MuJoCo viewer. The viewer is "
        "throttled via frame-skip (no blocking sleep) so the reader "
        "thread always runs at the full source rate.",
    )
    vis.add_argument(
        "--record_video",
        action="store_true",
        help="Record the viewer window to a video file.",
    )
    vis.add_argument(
        "--video_path",
        default="videos/moretion_to_sonic.mp4",
        help="Output video path (only used with --record_video).",
    )

    return parser


def main() -> int:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    # Windows: CTRL_BREAK_EVENT 触发 SIGBREAK，需额外注册以支持 graceful shutdown
    if os.name == "nt":
        signal.signal(signal.SIGBREAK, signal_handler)

    parser = _build_parser()
    args = parser.parse_args()

    if args.record_video:
        video_dir = os.path.dirname(args.video_path)
        if video_dir:
            os.makedirs(video_dir, exist_ok=True)

    return run_moretion_to_sonic(args)


if __name__ == "__main__":
    sys.exit(main())
