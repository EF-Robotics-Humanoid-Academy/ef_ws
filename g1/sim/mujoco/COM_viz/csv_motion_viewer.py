#!/usr/bin/env python3
"""Visualize G1 joint trajectories from CSV on the MuJoCo 3D model."""

from __future__ import annotations

import argparse
import csv
import re
import time
from pathlib import Path
from typing import Dict, List, Tuple

import mujoco
import mujoco.viewer
import numpy as np

# Unitree G1 body-joint ordering used by lowcmd motor index j0..j28.
G1_BODY_ACTUATORS: List[str] = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def _find_time_key(fieldnames: List[str]) -> str:
    for key in ("t_s", "ts", "time_s", "time"):
        if key in fieldnames:
            return key
    raise ValueError("CSV must include one time column: t_s/ts/time_s/time")


def load_csv_motion(path: Path) -> Tuple[np.ndarray, List[int], np.ndarray]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"CSV has no header: {path}")

        time_key = _find_time_key(reader.fieldnames)
        joint_cols: List[Tuple[int, str]] = []
        for col_name in reader.fieldnames:
            m = re.fullmatch(r"j(\d+)", str(col_name).strip().lower())
            if m:
                joint_cols.append((int(m.group(1)), col_name))

        if not joint_cols:
            raise ValueError(f"CSV must include joint columns like j15,j22,...: {path}")

        ts_vals: List[float] = []
        q_rows: List[List[float]] = []
        for row in reader:
            if not row:
                continue
            t_raw = row.get(time_key)
            if t_raw is None or str(t_raw).strip() == "":
                continue
            ts_vals.append(float(t_raw))
            q_rows.append([float(row[col_name]) for _, col_name in joint_cols])

    if not ts_vals:
        raise ValueError(f"CSV has no valid data rows: {path}")

    ts = np.asarray(ts_vals, dtype=float)
    qs = np.asarray(q_rows, dtype=float)
    if qs.shape[0] != ts.shape[0]:
        raise ValueError("CSV parse error: ts/qs length mismatch")

    # Ensure time is strictly nondecreasing for interpolation.
    if np.any(np.diff(ts) < 0.0):
        order = np.argsort(ts)
        ts = ts[order]
        qs = qs[order]

    joints = [j for j, _ in joint_cols]
    return ts, joints, qs


def _interp_row(ts: np.ndarray, qs: np.ndarray, t: float) -> np.ndarray:
    if t <= float(ts[0]):
        return qs[0]
    if t >= float(ts[-1]):
        return qs[-1]

    hi = int(np.searchsorted(ts, t, side="right"))
    lo = max(0, hi - 1)
    t0 = float(ts[lo])
    t1 = float(ts[hi])
    if t1 <= t0:
        return qs[hi]

    a = (t - t0) / (t1 - t0)
    return (1.0 - a) * qs[lo] + a * qs[hi]


def build_qpos_map(model: mujoco.MjModel, joints: List[int]) -> Dict[int, int]:
    qpos_map: Dict[int, int] = {}
    for jidx in joints:
        if jidx < 0 or jidx >= len(G1_BODY_ACTUATORS):
            continue
        joint_name = G1_BODY_ACTUATORS[jidx]
        try:
            jid = model.joint(joint_name).id
        except Exception:
            continue
        qpos_map[jidx] = int(model.jnt_qposadr[jid])
    return qpos_map


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize G1 CSV joint motion in MuJoCo")
    parser.add_argument(
        "--csv",
        required=True,
        help="CSV path (expects time + jXX columns)",
    )
    parser.add_argument(
        "--scene",
        default=str((Path(__file__).resolve().parent / "scene.xml")),
        help="MuJoCo scene XML",
    )
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed (1.0 = realtime)")
    parser.add_argument("--loop", action="store_true", help="Loop playback")
    parser.add_argument("--dt", type=float, default=0.005, help="Physics step (seconds)")
    parser.add_argument("--hide-contact", action="store_true", help="Hide contact points/forces")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only parse and print mapping; do not open viewer",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    scene_path = Path(args.scene).expanduser().resolve()

    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")
    if not scene_path.exists():
        raise SystemExit(f"Scene not found: {scene_path}")

    ts, joints, qs = load_csv_motion(csv_path)

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    model.opt.timestep = max(1e-4, float(args.dt))

    qpos_map = build_qpos_map(model, joints)
    mapped = sorted(qpos_map.keys())
    missing = sorted([j for j in joints if j not in qpos_map])

    print(f"Loaded: {csv_path}")
    print(f"Samples: {len(ts)}, duration: {ts[-1] - ts[0]:.3f}s")
    print(f"CSV joints: {joints}")
    print(f"Mapped joints ({len(mapped)}): {mapped}")
    if missing:
        print(f"Ignored joints (not in G1 body mapping): {missing}")

    if args.dry_run:
        return

    base_qpos = data.qpos.copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        if args.hide_contact:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
        else:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        t0_csv = float(ts[0])
        t_end_csv = float(ts[-1])
        play_start = time.perf_counter()
        loops = 0

        while viewer.is_running():
            wall_t = time.perf_counter() - play_start
            t_csv = t0_csv + wall_t * max(1e-6, args.speed)

            if t_csv > t_end_csv:
                if args.loop:
                    loops += 1
                    play_start = time.perf_counter()
                    t_csv = t0_csv
                else:
                    t_csv = t_end_csv

            q_row = _interp_row(ts, qs, t_csv)

            data.qpos[:] = base_qpos
            for col, jidx in enumerate(joints):
                qadr = qpos_map.get(jidx)
                if qadr is not None:
                    data.qpos[qadr] = float(q_row[col])

            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            viewer.sync()

            # Keep viewer responsive and paced by desired timestep.
            time.sleep(model.opt.timestep)

            if (not args.loop) and (t_csv >= t_end_csv):
                break

        print(f"Playback finished. loops={loops}")


if __name__ == "__main__":
    main()
