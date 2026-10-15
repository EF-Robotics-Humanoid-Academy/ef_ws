#!/usr/bin/env python3
"""Retarget a five-finger glove CSV to the Dex3 thumb, middle, and index."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path


JOINT_NAMES = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
CONTROLLED_FEATURES = {
    1: ("ThumbUnder", "ThumbMid"),
    2: ("ThumbMid", "ThumbUp"),
    3: ("Hand", "MiddleFingerUnder"),
    4: ("MiddleFingerUnder", "MiddleFingerMid"),
    5: ("Hand", "ForeFingerUnder"),
    6: ("ForeFingerUnder", "ForeFingerMid"),
}


def quaternion(row: dict[str, str], name: str) -> tuple[float, float, float, float]:
    values = tuple(float(row[f"{name}-handBone-Quat-{axis}"]) for axis in "wxyz")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError(f"Invalid quaternion for {name}")
    return tuple(value / norm for value in values)


def multiply(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def relative_rotation_vector(parent: tuple[float, ...], child: tuple[float, ...]) -> tuple[float, float, float]:
    relative = multiply((parent[0], -parent[1], -parent[2], -parent[3]), child)
    if relative[0] < 0.0:
        relative = tuple(-value for value in relative)
    vector_norm = math.sqrt(sum(value * value for value in relative[1:]))
    if vector_norm < 1e-8:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, relative[0]))
    return tuple(angle * value / vector_norm for value in relative[1:])


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    blend = position - lower
    return ordered[lower] * (1.0 - blend) + ordered[upper] * blend


def load_motion(path: Path, source_side: str) -> tuple[list[dict[str, str]], list[list[float]]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError(f"Expected at least two motion rows in {path}")

    prefix = source_side + "Hand"
    features: dict[int, list[tuple[float, float, float]]] = {joint: [] for joint in CONTROLLED_FEATURES}
    for row in rows:
        hand_q = quaternion(row, prefix)
        for joint, (parent_part, child_part) in CONTROLLED_FEATURES.items():
            parent_name = prefix if parent_part == "Hand" else source_side + parent_part
            child_name = source_side + child_part
            parent_q = hand_q if parent_name == prefix else quaternion(row, parent_name)
            child_q = quaternion(row, child_name)
            features[joint].append(relative_rotation_vector(parent_q, child_q))

    # Pick the dominant changing rotation-vector axis per anatomical joint,
    # then use robust capture percentiles to map that motion into Dex3 limits.
    mapped: list[list[float]] = [[0.0] * 7 for _ in rows]
    mapped_limits: dict[int, tuple[int, float, float]] = {}
    for joint, vectors in features.items():
        spans = [max(vector[axis] for vector in vectors) - min(vector[axis] for vector in vectors) for axis in range(3)]
        axis = max(range(3), key=spans.__getitem__)
        values = [vector[axis] for vector in vectors]
        low = percentile(values, 0.05)
        high = percentile(values, 0.95)
        if high - low < 0.04:
            low, high = min(values), max(values)
        if high - low < 1e-4:
            low, high = -0.5, 0.5
        mapped_limits[joint] = (axis, low, high)
        for index, value in enumerate(values):
            closure = min(1.0, max(0.0, (value - low) / (high - low)))
            mapped[index][joint] = closure

    for index in range(len(rows)):
        mapped[index][0] = 0.0
    return rows, mapped


def interpolate_presets(side: str, closure: list[float]) -> list[list[float]]:
    if side == "right":
        opened = [-0.03510913997888565, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0]
        closed = [-0.03510913997888565, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75]
    else:
        opened = [-0.09927542507648468, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75]
        closed = [-0.09927542507648468, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0]
    return [
        [start + (stop - start) * min(1.0, max(0.0, amount)) for start, stop, amount in zip(opened, closed, frame)]
        for frame in closure
    ]


def resample(poses: list[list[float]], source_hz: float, output_hz: float) -> list[list[float]]:
    if len(poses) < 2:
        return poses
    count = max(2, int(round((len(poses) - 1) * output_hz / source_hz)) + 1)
    result = []
    for output_index in range(count):
        source_position = output_index * source_hz / output_hz
        left = min(int(source_position), len(poses) - 1)
        right = min(left + 1, len(poses) - 1)
        blend = source_position - left
        result.append([a + (b - a) * blend for a, b in zip(poses[left], poses[right])])
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path(__file__).with_name("close_fist.csv"))
    parser.add_argument("--side", choices=("left", "right"), default="right", help="Dex3 wrist and glove recording side")
    parser.add_argument("--source-hz", type=float, default=50.0, help="assumed CSV row rate")
    parser.add_argument("--rate-hz", type=float, default=50.0, help="Dex3 command publish rate")
    parser.add_argument("--iface", default=None, help="DDS network interface; defaults to SDK selection")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--kp", type=float, default=1.5)
    parser.add_argument("--kd", type=float, default=0.1)
    parser.add_argument("--tau", type=float, default=0.03)
    parser.add_argument("--max-dq", type=float, default=1.5, help="clamp commanded joint velocity in rad/s")
    parser.add_argument("--execute", action="store_true", help="publish commands to the physical Dex3 hand")
    args = parser.parse_args()
    if args.source_hz <= 0.0 or args.rate_hz <= 0.0:
        parser.error("--source-hz and --rate-hz must be positive")

    rows, closure = load_motion(args.input, args.side)
    poses = resample(interpolate_presets(args.side, closure), args.source_hz, args.rate_hz)
    duration = (len(poses) - 1) / args.rate_hz
    print(f"Loaded {len(rows)} rows; retargeted {duration:.2f}s to {args.side} Dex3.")
    print("Driven joints: thumb, middle, index. Ring and little fingers are omitted.")
    print("Joint order:", ", ".join(JOINT_NAMES))
    if not args.execute:
        for index in sorted({0, len(poses) // 4, len(poses) // 2, 3 * len(poses) // 4, len(poses) - 1}):
            print(f"{index / args.rate_hz:6.2f}s q={poses[index]}")
        print("Preview only. Pass --execute to publish Dex3 commands.")
        return 0

    modules_dir = Path(__file__).resolve().parents[1] / "modules"
    sys.path.insert(0, str(modules_dir))
    from sdk_hand import Dex3HandController, build_hand_msg, clamp_hand_targets

    controller = Dex3HandController(hand=args.side, iface=args.iface, domain_id=args.domain_id)
    interval = 1.0 / args.rate_hz
    next_tick = time.monotonic()
    previous = poses[0]
    try:
        for pose in poses:
            now = time.monotonic()
            dt = interval if previous is poses[0] else max(1e-4, now - previous_time)
            velocity = [0.0 if previous is poses[0] else (target - start) / dt for start, target in zip(previous, pose)]
            velocity = [max(-args.max_dq, min(args.max_dq, value)) for value in velocity]
            targets = clamp_hand_targets(args.side, pose)
            message = build_hand_msg(targets, kp=args.kp, kd=args.kd, tau=args.tau)
            for index, command in enumerate(message.motor_cmd):
                command.dq = float(velocity[index])
            controller._pub.Write(message)
            previous = pose
            previous_time = now
            next_tick += interval
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("Stopped playback.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
