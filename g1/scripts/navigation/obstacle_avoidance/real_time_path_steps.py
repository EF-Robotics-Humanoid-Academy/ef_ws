#!/usr/bin/env python3
"""
real_time_path_steps.py
=======================

Load a saved occupancy map with start/goal poses, compute an A* path, then
optionally verify/replan against the live SLAM occupancy grid and execute the
path with LocoClient Move commands.
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Optional

from create_map import OccupancyGrid
from map_viewer import MapViewer
from path_planner import astar, grid_path_to_world_waypoints, smooth_path

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed. Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from obstacle_detection import ObstacleDetector
from locomotion import Locomotion
from slam_map import SlamMapSubscriber


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Execute a planned path with live map checks",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--map", required=True, help="Path to saved map (.npz)")
    p.add_argument("--start-x", type=float, default=None, help="Override start x (m)")
    p.add_argument("--start-y", type=float, default=None, help="Override start y (m)")
    p.add_argument("--goal-x", type=float, default=None, help="Override goal x (m)")
    p.add_argument("--goal-y", type=float, default=None, help="Override goal y (m)")
    p.add_argument("--iface", default="eth0", help="Network interface for DDS")
    p.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    p.add_argument("--sport-topic", default="rt/odommodestate", help="SportModeState topic")
    p.add_argument("--slam-map-topic", default="rt/utlidar/map_state", help="SLAM map topic")
    p.add_argument("--slam-height-threshold", type=float, default=0.15, help="HeightMap obstacle threshold")
    p.add_argument("--slam-max-height", type=float, default=None, help="Optional max height clamp")
    p.add_argument("--slam-origin-centered", action="store_true", help="Center SLAM map around (0,0)")
    p.add_argument("--slam-timeout", type=float, default=3.0, help="Seconds to wait for SLAM map")
    p.add_argument("--use-live-map", action="store_true", help="Replan using live SLAM map if path is blocked")
    p.add_argument("--inflation", type=int, default=3, help="Inflation radius (cells)")
    p.add_argument("--allow-diagonal", action="store_true", help="Allow diagonal moves (8-connected)")
    p.add_argument("--smooth", action="store_true", help="Smooth the grid path")
    p.add_argument("--max-skip", type=int, default=5, help="Max skip for smoothing")
    p.add_argument("--spacing", type=float, default=0.5, help="Waypoint spacing (m)")
    p.add_argument("--max-replans", type=int, default=5, help="Max replans before giving up")
    p.add_argument("--pose-mismatch", type=float, default=0.5,
                   help="If robot pose differs from start by this distance, use robot pose")
    p.add_argument("--use-map-start", action="store_true",
                   help="Always use map start_pose, even if it differs from robot pose")
    p.add_argument("--max-speed", type=float, default=0.25, help="Max forward speed (m/s)")
    p.add_argument("--timeout", type=float, default=30.0, help="Timeout per waypoint (s)")
    p.add_argument("--scale", type=int, default=4, help="Viewer scale (px/cell)")
    p.add_argument("--no-viz", action="store_true", help="Disable visualization window")
    return p.parse_args()


def _load_start_goal(occ: OccupancyGrid, args: argparse.Namespace) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    start = occ.start_pose
    goal = occ.goal_pose

    if args.start_x is not None and args.start_y is not None:
        start = (args.start_x, args.start_y, 0.0)
    if args.goal_x is not None and args.goal_y is not None:
        goal = (args.goal_x, args.goal_y, 0.0)

    if start is None or goal is None:
        raise SystemExit("ERROR: start_pose or goal_pose missing. Use --start-x/--start-y and --goal-x/--goal-y.")

    return start, goal


def _wait_slam_map(
    sub: SlamMapSubscriber,
    timeout: float,
    height_threshold: float,
    max_height: Optional[float],
    origin_centered: bool,
) -> Optional[OccupancyGrid]:
    t0 = time.time()
    while time.time() - t0 < timeout:
        occ, _meta = sub.to_occupancy(
            height_threshold=height_threshold,
            max_height=max_height,
            origin_centered=origin_centered,
        )
        if occ is not None:
            return occ
        time.sleep(0.05)
    return None


def _plan_path(
    occ: OccupancyGrid,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    inflation: int,
    allow_diagonal: bool,
    smooth: bool,
    max_skip: int,
    spacing: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    if inflation > 0:
        plan_grid = occ.inflate(radius_cells=inflation)
    else:
        plan_grid = occ.grid.copy()

    start_rc = occ.world_to_grid(start_xy[0], start_xy[1])
    goal_rc = occ.world_to_grid(goal_xy[0], goal_xy[1])

    raw_path = astar(plan_grid, start_rc, goal_rc, allow_diagonal=allow_diagonal)
    if raw_path is None:
        sr, sc = start_rc
        gr, gc = goal_rc
        details = (
            f"start_rc={start_rc} goal_rc={goal_rc} "
            f"start_occ={int(plan_grid[sr, sc]) if 0 <= sr < plan_grid.shape[0] and 0 <= sc < plan_grid.shape[1] else 'oob'} "
            f"goal_occ={int(plan_grid[gr, gc]) if 0 <= gr < plan_grid.shape[0] and 0 <= gc < plan_grid.shape[1] else 'oob'}"
        )
        raise RuntimeError(f"A* failed to find a path. ({details})")

    if smooth:
        raw_path = smooth_path(raw_path, plan_grid, max_skip=max_skip)

    world_path = [occ.grid_to_world(r, c) for r, c in raw_path]
    waypoints = grid_path_to_world_waypoints(raw_path, occ, spacing_m=spacing)
    return waypoints, world_path


def _path_blocked(occ: OccupancyGrid, waypoints: list[tuple[float, float]], inflation: int) -> bool:
    if inflation > 0:
        grid = occ.inflate(radius_cells=inflation)
    else:
        grid = occ.grid
    for wx, wy in waypoints:
        r, c = occ.world_to_grid(wx, wy)
        if grid[r, c] != 0:
            return True
    return False


def main() -> None:
    args = _parse_args()

    occ = OccupancyGrid.load(args.map)
    start, goal = _load_start_goal(occ, args)

    ChannelFactoryInitialize(args.domain_id, args.iface)

    detector = ObstacleDetector(topic=args.sport_topic)
    detector.start()
    time.sleep(0.5)
    if detector.is_stale():
        raise SystemExit("ERROR: no SportModeState_ data. Is the robot connected?")

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    walker = Locomotion(loco, detector, max_vx=args.max_speed)

    # Prefer robot pose if start is far from current pose
    rx, ry, ryaw = detector.get_pose()
    if not args.use_map_start:
        if math.hypot(rx - start[0], ry - start[1]) > args.pose_mismatch:
            print("WARN: start_pose far from robot pose; using robot pose instead.")
            start = (rx, ry, ryaw)

    slam_sub = SlamMapSubscriber(args.slam_map_topic)
    slam_sub.start()

    replan_count = 0
    viz_enabled = not args.no_viz
    print(f"Start: ({start[0]:+.2f}, {start[1]:+.2f}) | Goal: ({goal[0]:+.2f}, {goal[1]:+.2f})")
    waypoints, world_path = _plan_path(
        occ, (start[0], start[1]), (goal[0], goal[1]),
        args.inflation, args.allow_diagonal, args.smooth, args.max_skip, args.spacing,
    )
    print(f"Planned {len(waypoints)} waypoint(s) on saved map.")

    if args.use_live_map:
        live = _wait_slam_map(
            slam_sub, args.slam_timeout,
            args.slam_height_threshold, args.slam_max_height, args.slam_origin_centered,
        )
        if live is not None and _path_blocked(live, waypoints, args.inflation):
            print("Live map indicates blocked path. Replanning on SLAM map.")
            occ = live
            waypoints, world_path = _plan_path(
                occ, (start[0], start[1]), (goal[0], goal[1]),
                args.inflation, args.allow_diagonal, args.smooth, args.max_skip, args.spacing,
            )

    viewer = None
    if viz_enabled:
        viewer = MapViewer(
            occ,
            window_name="Real-Time Path",
            scale=args.scale,
            inflation_radius=max(0, args.inflation),
        )
        viewer.set_start(start[0], start[1], start[2])
        viewer.set_goal(goal[0], goal[1])
        viewer.set_path(world_path)
        viewer.set_waypoints(waypoints)

    try:
        while True:
            aborted = False
            for i, (wx, wy) in enumerate(waypoints):
                is_last = i == len(waypoints) - 1
                print(f"[{i + 1}/{len(waypoints)}] -> ({wx:+.2f}, {wy:+.2f})")

                def _check_obstacle() -> bool:
                    if viewer is not None:
                        vx, vy, vyaw = detector.get_pose()
                        vranges = detector.get_ranges()
                        viewer.update(vx, vy, vyaw, vranges, None)
                    return detector.front_blocked()

                ok = walker.walk_to(
                    wx,
                    wy,
                    final_yaw=goal[2] if is_last else None,
                    timeout=args.timeout,
                    check_obstacle=_check_obstacle,
                )

                if not ok:
                    aborted = True
                    break

            if not aborted:
                print("Path complete.")
                break

            replan_count += 1
            if not args.use_live_map or replan_count > args.max_replans:
                raise SystemExit("Aborted or timed out; replans exhausted.")

            live = _wait_slam_map(
                slam_sub, args.slam_timeout,
                args.slam_height_threshold, args.slam_max_height, args.slam_origin_centered,
            )
            if live is None:
                raise SystemExit("No live SLAM map available for replanning.")

            occ = live
            rx, ry, _ = detector.get_pose()
            waypoints, world_path = _plan_path(
                occ, (rx, ry), (goal[0], goal[1]),
                args.inflation, args.allow_diagonal, args.smooth, args.max_skip, args.spacing,
            )
            print(f"Replanned: {len(waypoints)} waypoint(s).")
            if viewer is not None:
                viewer.close()
                viewer = MapViewer(
                    occ,
                    window_name="Real-Time Path",
                    scale=args.scale,
                    inflation_radius=max(0, args.inflation),
                )
                viewer.set_start(rx, ry, 0.0)
                viewer.set_goal(goal[0], goal[1])
                viewer.set_path(world_path)
                viewer.set_waypoints(waypoints)
    except KeyboardInterrupt:
        print("\nInterrupted; stopping.")
    finally:
        walker.stop()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
