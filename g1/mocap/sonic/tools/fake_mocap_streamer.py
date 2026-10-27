#!/usr/bin/env python3
"""Synthetic GMR/SONIC Protocol v1 publisher for dry-run testing.

Publishes the same single-part packed message the GMR bridge sends
(topic 'pose' + 1280-byte JSON header + binary fields) at 50 Hz with a
window of [1 past, live target, N future] frames and `live_frame_index`.

Fault injections (NaN frames, garbage frames, frame skips, dropouts, step
jumps, root-yaw jumps) can be triggered programmatically (see Streamer) or
from a timeline on the command line.

Standalone:
    python3 tools/fake_mocap_streamer.py --port 5556
"""
import argparse
import json
import math
import threading
import time

import numpy as np
import zmq

HEADER_SIZE = 1280
NUM_JOINTS = 29
RATE_HZ = 50.0

# Default standing angles in hardware/URDF order (policy_parameters.hpp).
DEFAULT_ANGLES_HW = [
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
]
# mujoco_to_isaaclab: hardware index of IsaacLab joint i (policy_parameters.hpp).
HW_OF_ISAAC = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
               16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
ISAAC_OF_HW = {hw: i for i, hw in enumerate(HW_OF_ISAAC)}

L_SHOULDER_PITCH = ISAAC_OF_HW[15]
L_ELBOW = ISAAC_OF_HW[18]
R_SHOULDER_PITCH = ISAAC_OF_HW[22]
L_KNEE = ISAAC_OF_HW[3]


def default_pose_isaac():
    return np.array([DEFAULT_ANGLES_HW[HW_OF_ISAAC[i]] for i in range(NUM_JOINTS)], dtype=np.float64)


def pack_message(topic, fields):
    """fields: list of (name, np.ndarray) -> bytes."""
    meta, payload = [], []
    for name, arr in fields:
        dtype = {np.dtype(np.float32): "f32", np.dtype(np.float64): "f64",
                 np.dtype(np.int64): "i64", np.dtype(np.int32): "i32"}[arr.dtype]
        meta.append({"name": name, "dtype": dtype, "shape": list(arr.shape)})
        payload.append(np.ascontiguousarray(arr).astype(arr.dtype.newbyteorder("<")).tobytes())
    header = json.dumps({"v": 1, "endian": "le", "count": int(fields[0][1].shape[0]), "fields": meta}).encode()
    assert len(header) < HEADER_SIZE
    return topic.encode() + header.ljust(HEADER_SIZE, b"\0") + b"".join(payload)


class Streamer:
    """50 Hz publisher thread with programmable faults."""

    def __init__(self, port=5556, topic="pose", future_frames=10):
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(f"tcp://*:{port}")
        self.topic = topic
        self.future = future_frames
        self.base = default_pose_isaac()
        self.lock = threading.Lock()
        self.frame = 0
        self.t0 = time.monotonic()
        # fault state
        self.paused_until = 0.0
        self.nan_frames = 0
        self.garbage_frames = 0
        self.skip_next = 0
        self.step_offset = np.zeros(NUM_JOINTS)
        self.yaw = 0.0
        self.amplitude = 0.5
        self.sent = 0
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    # ---- programmable faults ------------------------------------------
    def dropout(self, seconds):
        with self.lock:
            self.paused_until = time.monotonic() + seconds

    def inject_nan(self, n):
        with self.lock:
            self.nan_frames = n

    def inject_garbage(self, n):
        with self.lock:
            self.garbage_frames = n

    def skip_frames(self, n):
        with self.lock:
            self.skip_next = n

    def set_step(self, isaac_joint, offset):
        with self.lock:
            self.step_offset[isaac_joint] = offset

    def set_yaw(self, yaw):
        with self.lock:
            self.yaw = yaw

    # ---- generation ----------------------------------------------------
    def pose_at(self, frame):
        t = frame / RATE_HZ
        q = self.base.copy()
        q[L_SHOULDER_PITCH] += self.amplitude * math.sin(2 * math.pi * 0.3 * t)
        q[R_SHOULDER_PITCH] += 0.5 * self.amplitude * math.sin(2 * math.pi * 0.2 * t)
        q[L_ELBOW] += 0.4 * self.amplitude * (1 - math.cos(2 * math.pi * 0.3 * t))
        return q + self.step_offset

    def _run(self):
        period = 1.0 / RATE_HZ
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += period
            with self.lock:
                now = time.monotonic()
                self.frame += 1 + self.skip_next
                self.skip_next = 0
                if now < self.paused_until:
                    pass  # dropout: frames advance, nothing is sent
                else:
                    self._send_locked()
            time.sleep(max(0.0, next_t - time.monotonic()))

    def _send_locked(self):
        idx = np.arange(self.frame - 1, self.frame + self.future + 1, dtype=np.int64)
        pos = np.stack([self.pose_at(int(i)) for i in idx])
        vel = np.gradient(pos, 1.0 / RATE_HZ, axis=0)
        if self.nan_frames > 0:
            pos[:, L_ELBOW] = np.nan
            self.nan_frames -= 1
        elif self.garbage_frames > 0:
            pos[:, L_KNEE] = 5.0
            self.garbage_frames -= 1
        quat = np.zeros((len(idx), 1, 4))
        quat[:, 0, 0] = math.cos(self.yaw / 2)
        quat[:, 0, 3] = math.sin(self.yaw / 2)
        msg = pack_message(self.topic, [
            ("joint_pos", pos.astype(np.float32)),
            ("joint_vel", vel.astype(np.float32)),
            ("body_quat", quat.astype(np.float32)),
            ("frame_index", idx),
            ("live_frame_index", np.array([self.frame], dtype=np.int64)),
        ])
        self.sock.send(msg)
        self.sent += 1

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self._stop.set()
        self.thread.join(timeout=1.0)
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--future-frames", type=int, default=10, help="preview frames after the live target (default 10 = 200 ms)")
    ap.add_argument("--amplitude", type=float, default=0.5, help="arm motion amplitude (rad)")
    args = ap.parse_args()
    s = Streamer(args.port, future_frames=args.future_frames)
    s.amplitude = args.amplitude
    s.start()
    print(f"Publishing synthetic Protocol v1 mocap on tcp://*:{args.port} topic 'pose' at 50 Hz. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(5)
            print(f"  sent {s.sent} messages")
    except KeyboardInterrupt:
        pass
    s.stop()


if __name__ == "__main__":
    main()
