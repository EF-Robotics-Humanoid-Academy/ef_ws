"""
pattern_1.py
============

Obstacle parcour pattern 1: Navigate → Wave → Navigate → Detect → Pick & Place.

Chains together multiple robot capabilities in sequence:
  A ──(navigate)──> B ──(wave hand)──> B ──(navigate)──> C ──(detect soda)──> [pick & place]

1. Navigate from A to B using A* with dynamic obstacle replanning.
2. Wave hand at point B (G1ArmActionClient action 26).
3. Navigate from B to C (reuses accumulated obstacle map).
4. CLIP zero-shot soda can detection at point C.
5. If detected, run pick-and-place sequence.

Prerequisites
-------------
* Robot powered on, standing (FSM-200), and reachable on the network.
* All sibling modules installed (obstacle_avoidance/, obj_detection/, pick_and_place/).

Usage
-----
::

    python pattern_1.py --iface eth0 --bx 2.0 --by 0.0 --cx 4.0 --cy 0.0
    python pattern_1.py --iface eth0 --bx 1.5 --by 1.0 --cx 3.0 --cy 2.0 --viz
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# sys.path setup -- sibling directories
# ---------------------------------------------------------------------------
_SCRIPTS_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
for _subdir in ("obstacle_avoidance", "obj_detection", "pick_and_place"):
    _path = os.path.join(_SCRIPTS_DIR, _subdir)
    if _path not in sys.path:
        sys.path.insert(0, _path)

# ---------------------------------------------------------------------------
# Unitree SDK2 imports
# ---------------------------------------------------------------------------
try:
    from unitree_sdk2py.core.channel import (
        ChannelPublisher,
    )
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__HandCmd_,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, HandCmd_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed.  Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

# ---------------------------------------------------------------------------
# Sibling module imports
# ---------------------------------------------------------------------------
# obstacle_avoidance/
from create_map import OccupancyGrid, create_empty_map
from path_planner import astar, smooth_path, grid_path_to_world_waypoints
from obstacle_detection import ObstacleDetector
from locomotion import Locomotion

# obj_detection/
from soda_can_detect import load_clip, classify_frame

# pick_and_place/
from g1_pick_place_hardcoded import (
    _move_arm,
    _build_hand_msg,
    _apply_arm_targets,
    POSES,
    HAND_OPEN,
    HAND_CLOSED,
    ARM_JOINTS,
    G1JointIndex,
    TOPIC_ARM,
    TOPIC_HAND_RIGHT,
    G1_NUM_MOTOR,
    ARM_ENABLE_IDX,
)

from safety.hanger_boot_sequence import hanger_boot_sequence

# Optional live viewer (may not be installed)
try:
    from map_viewer import MapViewer
except ImportError:
    MapViewer = None  # type: ignore[misc,assignment]

import cv2


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Obstacle parcour pattern 1: Navigate → Wave → Navigate → Detect → Pick & Place",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--iface", default="eth0",
                    help="Network interface connected to the robot")
    # Waypoints
    p.add_argument("--ax", type=float, default=None,
                    help="Point A x (start). Default: current pose")
    p.add_argument("--ay", type=float, default=None,
                    help="Point A y (start). Default: current pose")
    p.add_argument("--bx", type=float, required=True,
                    help="Point B x (wave point)")
    p.add_argument("--by", type=float, required=True,
                    help="Point B y (wave point)")
    p.add_argument("--cx", type=float, required=True,
                    help="Point C x (soda detection point)")
    p.add_argument("--cy", type=float, required=True,
                    help="Point C y (soda detection point)")
    # Tuning
    p.add_argument("--threshold", type=float, default=0.6,
                    help="CLIP detection threshold (0-1)")
    p.add_argument("--max_speed", type=float, default=0.3,
                    help="Maximum forward navigation speed (m/s)")
    p.add_argument("--map_width", type=float, default=10.0,
                    help="Map width in metres")
    p.add_argument("--map_height", type=float, default=10.0,
                    help="Map height in metres")
    p.add_argument("--resolution", type=float, default=0.1,
                    help="Map resolution in metres per cell")
    p.add_argument("--inflation", type=int, default=3,
                    help="Obstacle inflation radius in cells")
    p.add_argument("--spacing", type=float, default=0.5,
                    help="Waypoint spacing in metres")
    p.add_argument("--max_replans", type=int, default=20,
                    help="Max replan attempts per navigation leg")
    p.add_argument("--viz", action="store_true",
                    help="Show a live map window (OpenCV) while navigating")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Navigation helper (reusable A* loop from navigate.py)
# ---------------------------------------------------------------------------


def navigate_to(
    goal_x: float,
    goal_y: float,
    goal_yaw: float | None,
    walker: Locomotion,
    detector: ObstacleDetector,
    occ_grid: OccupancyGrid,
    inflation: int = 3,
    spacing: float = 0.5,
    max_replans: int = 20,
    viewer: object | None = None,
) -> bool:
    """A* navigation with dynamic replanning.  Returns True if goal reached."""
    replan_count = 0
    goal_reached = False

    while replan_count < max_replans and not goal_reached:
        # Current pose
        cx, cy, cyaw = detector.get_pose()

        # Update map with latest obstacle readings
        ranges = detector.get_ranges()
        occ_grid.mark_obstacle_from_range(cx, cy, cyaw, ranges)

        # Inflate for planning
        inflated = occ_grid.inflate(radius_cells=inflation)

        # Grid coordinates
        start_cell = occ_grid.world_to_grid(cx, cy)
        goal_cell = occ_grid.world_to_grid(goal_x, goal_y)

        # A*
        print(f"\n  Planning (attempt {replan_count + 1}/{max_replans}) ...")
        print(f"    From cell {start_cell} to {goal_cell}")
        raw_path = astar(inflated, start_cell, goal_cell)

        if raw_path is None:
            print("  ERROR: no path found.  Goal may be unreachable.")
            return False

        # Smooth + world waypoints
        smoothed = smooth_path(raw_path, inflated)
        waypoints = grid_path_to_world_waypoints(
            smoothed, occ_grid, spacing_m=spacing
        )
        print(f"    {len(raw_path)} cells -> {len(smoothed)} smoothed "
              f"-> {len(waypoints)} waypoints")

        # Update viewer overlays
        if viewer is not None:
            viewer.set_goal(goal_x, goal_y)
            full_world_path = [occ_grid.grid_to_world(r, c) for r, c in smoothed]
            viewer.set_path(full_world_path)
            viewer.set_waypoints(waypoints)

        # Walk each waypoint
        aborted = False
        for i, (wx, wy) in enumerate(waypoints):
            dist_to_goal = math.hypot(wx - goal_x, wy - goal_y)
            is_last = i == len(waypoints) - 1
            final_yaw = goal_yaw if is_last else None

            print(f"    [{i + 1}/{len(waypoints)}] -> ({wx:+.2f}, {wy:+.2f})"
                  f"  dist_to_goal={dist_to_goal:.2f} m")

            def _check_and_viz() -> bool:
                """Obstacle check + optional live viewer update."""
                if viewer is not None:
                    vx, vy, vyaw = detector.get_pose()
                    vranges = detector.get_ranges()
                    occ_grid.mark_obstacle_from_range(vx, vy, vyaw, vranges)
                    viewer.update(vx, vy, vyaw, vranges)
                return detector.front_blocked()

            reached = walker.walk_to(
                wx, wy,
                final_yaw=final_yaw,
                timeout=30.0,
                check_obstacle=_check_and_viz,
            )

            if not reached:
                print("    ** Obstacle or timeout -- replanning **")
                aborted = True
                replan_count += 1

                obs_positions = detector.get_obstacle_world_positions()
                for ox, oy in obs_positions:
                    occ_grid.set_obstacle_world(ox, oy)
                print(f"    Added {len(obs_positions)} obstacle(s) to map")
                break

        if not aborted:
            goal_reached = True

    if goal_reached:
        fx, fy, _ = detector.get_pose()
        dist = math.hypot(fx - goal_x, fy - goal_y)
        print(f"  Goal reached!  Final error={dist:.2f} m")
    else:
        print(f"  Navigation failed after {replan_count} replans.")

    return goal_reached


# ---------------------------------------------------------------------------
# VideoClient frame capture (no ChannelFactoryInitialize)
# ---------------------------------------------------------------------------


def capture_frame_no_init(timeout: float = 3.0) -> np.ndarray:
    """Grab one JPEG frame from the G1 camera without calling ChannelFactoryInitialize."""
    client = VideoClient()
    client.SetTimeout(timeout)
    client.Init()

    code, data = client.GetImageSample()

    if code != 0:
        raise RuntimeError(
            f"VideoClient.GetImageSample() failed with error code {code}.\n"
            "  Is the robot powered on and the videohub service running?"
        )

    jpeg_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
    frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

    if frame is None:
        raise RuntimeError(
            "cv2.imdecode returned None — received %d bytes." % len(data)
        )

    return frame


# ---------------------------------------------------------------------------
# Pick-and-place sequence (no ChannelFactoryInitialize)
# ---------------------------------------------------------------------------


def pick_and_place_no_init(rate_hz: float = 50.0) -> None:
    """Run the pick-and-place sequence without calling ChannelFactoryInitialize."""
    arm_pub = ChannelPublisher(TOPIC_ARM, LowCmd_)
    arm_pub.Init()

    hand_pub = ChannelPublisher(TOPIC_HAND_RIGHT, HandCmd_)
    hand_pub.Init()

    crc = CRC()
    arm_cmd = unitree_hg_msg_dds__LowCmd_()

    # Open hand first (soft, low force)
    hand_open = _build_hand_msg(HAND_OPEN, kp=1.2, kd=0.05, tau=0.05)
    hand_pub.Write(hand_open)
    time.sleep(0.5)

    # Stow then reach forward
    _move_arm(arm_pub, arm_cmd, crc, POSES["stow"],
              duration=1.5, rate_hz=rate_hz, kp=45.0, kd=1.2)
    _move_arm(arm_pub, arm_cmd, crc, POSES["reach_front"],
              duration=2.0, rate_hz=rate_hz, kp=45.0, kd=1.2)

    # Close hand with limited force
    grip_close = _build_hand_msg(HAND_CLOSED, kp=1.0, kd=0.05, tau=0.20)
    for _ in range(int(1.5 * rate_hz)):
        hand_pub.Write(grip_close)
        time.sleep(1.0 / rate_hz)

    # Lift while maintaining grip
    _move_arm(arm_pub, arm_cmd, crc, POSES["lift"],
              duration=1.2, rate_hz=rate_hz, kp=45.0, kd=1.2,
              hand_pub=hand_pub, hand_hold=grip_close)

    # Rotate shoulder to place
    _move_arm(arm_pub, arm_cmd, crc, POSES["place_high"],
              duration=1.5, rate_hz=rate_hz, kp=45.0, kd=1.2,
              hand_pub=hand_pub, hand_hold=grip_close)

    # Lower slowly
    _move_arm(arm_pub, arm_cmd, crc, POSES["place_low"],
              duration=2.5, rate_hz=rate_hz, kp=35.0, kd=1.0,
              hand_pub=hand_pub, hand_hold=grip_close)

    # Release and retract
    hand_pub.Write(hand_open)
    time.sleep(0.5)

    _move_arm(arm_pub, arm_cmd, crc, POSES["stow"],
              duration=2.0, rate_hz=rate_hz, kp=40.0, kd=1.1)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    args = parse_args()

    # --- 1. SDK init (once for entire process) ------------------------------
    print(f"Initialising SDK on interface '{args.iface}' (safe boot) ...")
    loco = hanger_boot_sequence(iface=args.iface)

    # --- 2. Subsystems ------------------------------------------------------

    detector = ObstacleDetector(warn_distance=0.8, stop_distance=0.4)
    detector.start()

    print("Waiting for SportModeState_ ...")
    time.sleep(1.0)
    if detector.is_stale():
        sys.exit("ERROR: no SportModeState_ data received.  "
                 "Is the robot connected and standing?")

    occ_grid = create_empty_map(
        width_m=args.map_width,
        height_m=args.map_height,
        resolution=args.resolution,
        origin_x=-args.map_width / 2,
        origin_y=-args.map_height / 2,
    )
    print(f"Created empty {args.map_width}x{args.map_height} m map, "
          f"resolution={args.resolution} m "
          f"({occ_grid.width_cells}x{occ_grid.height_cells} cells)")

    walker = Locomotion(loco, detector, max_vx=args.max_speed)

    # Optional live map viewer
    viewer = None
    if args.viz:
        if MapViewer is None:
            print("WARNING: map_viewer not available, --viz ignored.")
        else:
            viewer = MapViewer(occ_grid, scale=4, inflation_radius=args.inflation)
            print("Live map viewer enabled.")

    # Preload CLIP model
    print("Loading CLIP model (this may take a moment on first run) ...")
    model, processor, device = load_clip()

    # --- 3. Determine start pose -------------------------------------------
    sx, sy, syaw = detector.get_pose()
    if args.ax is not None and args.ay is not None:
        print(f"Start override: A=({args.ax:+.2f}, {args.ay:+.2f})")
    else:
        args.ax, args.ay = sx, sy
    print(f"Current pose: ({sx:+.2f}, {sy:+.2f}), yaw={math.degrees(syaw):+.1f} deg")
    print(f"Point B: ({args.bx:+.2f}, {args.by:+.2f})")
    print(f"Point C: ({args.cx:+.2f}, {args.cy:+.2f})")

    try:
        # --- Phase 1: Navigate A → B ---------------------------------------
        print("\n" + "=" * 60)
        print("PHASE 1: Navigating to point B ...")
        print("=" * 60)
        ok = navigate_to(
            args.bx, args.by, goal_yaw=None,
            walker=walker, detector=detector, occ_grid=occ_grid,
            inflation=args.inflation, spacing=args.spacing,
            max_replans=args.max_replans, viewer=viewer,
        )
        if not ok:
            print("Navigation to B failed.  Stopping.")
            return

        # --- Phase 2: Wave hand at B ---------------------------------------
        print("\n" + "=" * 60)
        print("PHASE 2: Waving hand ...")
        print("=" * 60)
        arm_client = G1ArmActionClient()
        arm_client.SetTimeout(10.0)
        arm_client.Init()
        arm_client.ExecuteAction(26)  # "high wave"
        print("  Waiting for wave gesture to complete ...")
        time.sleep(4.0)
        print("  Wave done.")

        # --- Phase 3: Navigate B → C ---------------------------------------
        print("\n" + "=" * 60)
        print("PHASE 3: Navigating to point C ...")
        print("=" * 60)
        ok = navigate_to(
            args.cx, args.cy, goal_yaw=None,
            walker=walker, detector=detector, occ_grid=occ_grid,
            inflation=args.inflation, spacing=args.spacing,
            max_replans=args.max_replans, viewer=viewer,
        )
        if not ok:
            print("Navigation to C failed.  Stopping.")
            return

        # --- Phase 4: Soda can detection -----------------------------------
        print("\n" + "=" * 60)
        print("PHASE 4: Checking for soda can ...")
        print("=" * 60)
        frame = capture_frame_no_init()
        print(f"  Captured frame: {frame.shape[1]}x{frame.shape[0]} pixels")

        result = classify_frame(frame, model, processor, device, args.threshold)
        print(f"  Detected  : {result['detected']}")
        print(f"  Confidence: {result['confidence']:.1%}")
        print(f"  Label     : {result['label']}")

        # --- Phase 5: Conditional pick and place ---------------------------
        if result["detected"]:
            print("\n" + "=" * 60)
            print("PHASE 5: Soda can detected! Picking up ...")
            print("=" * 60)
            pick_and_place_no_init()
            print("  Pick and place complete.")
        else:
            print("\nNo soda can detected. Skipping pick and place.")

        print("\n" + "=" * 60)
        print("PARCOUR COMPLETE")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        walker.stop()
        if viewer is not None:
            viewer.close()
        save_path = "/tmp/parcour_map.npz"
        occ_grid.save(save_path)
        print(f"Robot stopped.  Map saved to {save_path}")


if __name__ == "__main__":
    main()
