"""
navigate.py
===========

Main orchestrator for obstacle-avoidance navigation on the Unitree G1.

Uses the robot's built-in SLAM (utlidar map_state) to create an occupancy
map, lets the operator pick start + goal points with the mouse, then plans
and executes a path with dynamic replanning when obstacles are detected.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None  # type: ignore

try:
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed.  Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>"
    ) from exc

from create_map import OccupancyGrid, create_empty_map
from path_planner import astar, smooth_path, grid_path_to_world_waypoints
from obstacle_detection import ObstacleDetector
from locomotion import Locomotion
from map_viewer import MapViewer
from slam_map import (
    SlamMapSubscriber,
    LidarSwitch,
    SlamOdomSubscriber,
    SlamInfoSubscriber,
    SlamPointCloudSubscriber,
)
from slam_service import SlamOperateClient
from safety.hanger_boot_sequence import hanger_boot_sequence


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="G1 obstacle-avoidance navigation (SLAM map + A* + replanning)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iface", default="eth0",
                        help="Network interface connected to the robot")
    parser.add_argument("--domain-id", type=int, default=0,
                        help="DDS domain id")
    parser.add_argument("--sport-topic", default="rt/odommodestate",
                        help="SportModeState topic name")
    parser.add_argument("--pose-source", choices=["slam", "sport"], default="slam",
                        help="Pose source for navigation")
    parser.add_argument("--slam-odom-topic", default="rt/unitree/slam_mapping/odom",
                        help="SLAM odom topic (used when pose-source=slam)")
    parser.add_argument("--slam-points-topic", default="",
                        help="SLAM points topic (for mapping overlay)")
    parser.add_argument("--slam-points-stride", type=int, default=6,
                        help="Subsample SLAM points (1=all)")
    parser.add_argument("--slam-points-z-min", type=float, default=-0.5,
                        help="Min Z for SLAM points to use")
    parser.add_argument("--slam-points-z-max", type=float, default=1.5,
                        help="Max Z for SLAM points to use")
    parser.add_argument("--lidar-topic", default="rt/utlidar/cloud_livox_mid360",
                        help="Lidar PointCloud2 DDS topic (fallback overlay)")
    parser.add_argument("--lidar-stride", type=int, default=6,
                        help="Subsample lidar points (1=all)")
    parser.add_argument("--slam-info-topic", default="rt/slam_info",
                        help="SLAM info topic")
    parser.add_argument("--slam-key-topic", default="rt/slam_key_info",
                        help="SLAM key info topic")
    parser.add_argument("--slam-start-mapping", action="store_true",
                        help="Start SLAM mapping via slam_operate service")
    parser.add_argument("--slam-end-map", type=str, default="",
                        help="End mapping and save PCD to this path")
    parser.add_argument("--slam-init-map", type=str, default="",
                        help="Initialize pose using this PCD path")
    parser.add_argument("--slam-init-x", type=float, default=0.0, help="Init pose x")
    parser.add_argument("--slam-init-y", type=float, default=0.0, help="Init pose y")
    parser.add_argument("--slam-init-z", type=float, default=0.0, help="Init pose z")
    parser.add_argument("--slam-init-qx", type=float, default=0.0, help="Init pose qx")
    parser.add_argument("--slam-init-qy", type=float, default=0.0, help="Init pose qy")
    parser.add_argument("--slam-init-qz", type=float, default=0.0, help="Init pose qz")
    parser.add_argument("--slam-init-qw", type=float, default=1.0, help="Init pose qw")
    parser.add_argument("--slam-nav", action="store_true",
                        help="Use slam_operate pose navigation instead of local A*")
    parser.add_argument("--slam-nav-timeout", type=float, default=60.0,
                        help="Timeout for slam pose navigation (seconds)")
    parser.add_argument("--slam-pause", action="store_true",
                        help="Pause SLAM navigation (api 1201) and exit")
    parser.add_argument("--slam-resume", action="store_true",
                        help="Resume SLAM navigation (api 1202) and exit")
    parser.add_argument("--slam-status", action="store_true",
                        help="Print live slam_info / slam_key_info and exit")
    parser.add_argument("--goal_x", type=float, default=None,
                        help="Goal X position in metres (optional if using mouse pick)")
    parser.add_argument("--goal_y", type=float, default=None,
                        help="Goal Y position in metres (optional if using mouse pick)")
    parser.add_argument("--goal_yaw", type=float, default=None,
                        help="Optional final heading in radians")
    parser.add_argument("--map_width", type=float, default=10.0,
                        help="Map width in metres (fallback if SLAM unavailable)")
    parser.add_argument("--map_height", type=float, default=10.0,
                        help="Map height in metres (fallback if SLAM unavailable)")
    parser.add_argument("--resolution", type=float, default=0.1,
                        help="Map resolution in metres per cell (fallback)")
    parser.add_argument("--load_map", type=str, default=None,
                        help="Path to a saved .npz map file to load instead of SLAM")
    parser.add_argument("--max_speed", type=float, default=0.3,
                        help="Maximum forward speed (m/s)")
    parser.add_argument("--inflation", type=int, default=3,
                        help="Obstacle inflation radius in cells")
    parser.add_argument("--spacing", type=float, default=0.5,
                        help="Waypoint spacing in metres (~1 step)")
    parser.add_argument("--max_replans", type=int, default=20,
                        help="Maximum replan attempts before giving up")
    parser.add_argument("--viz", action="store_true",
                        help="Show a live map window (OpenCV) while navigating")
    parser.add_argument("--no-pick", action="store_true",
                        help="Disable mouse picking of start/goal")
    parser.add_argument("--no-slam", action="store_true",
                        help="Disable built-in SLAM map (use empty or loaded map)")
    parser.add_argument("--slam-timeout", type=float, default=6.0,
                        help="Seconds to wait for SLAM map before fallback")
    parser.add_argument("--slam-height-threshold", type=float, default=0.15,
                        help="Height (m) above which a cell is marked obstacle")
    parser.add_argument("--slam-max-height", type=float, default=None,
                        help="Optional max height clamp for SLAM map")
    parser.add_argument("--slam-origin-centered", action="store_true",
                        help="Center SLAM map at (0,0)")
    parser.add_argument("--slam-no-lidar-switch", action="store_true",
                        help="Do not publish rt/utlidar/switch ON")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# SLAM map helpers
# ---------------------------------------------------------------------------


def _wait_for_slam_map(
    sub: SlamMapSubscriber,
    timeout: float,
    height_threshold: float,
    max_height: Optional[float],
    origin_centered: bool,
) -> tuple[OccupancyGrid | None, float]:
    t0 = time.time()
    last_ts = 0.0
    while time.time() - t0 < timeout:
        grid, meta = sub.to_occupancy(
            height_threshold=height_threshold,
            max_height=max_height,
            origin_centered=origin_centered,
        )
        if grid is not None and meta is not None:
            return grid, meta.timestamp
        time.sleep(0.05)
    return None, last_ts


def _refresh_slam_map(
    sub: SlamMapSubscriber,
    occ_grid: OccupancyGrid,
    height_threshold: float,
    max_height: Optional[float],
    origin_centered: bool,
    last_ts: float,
) -> tuple[OccupancyGrid, float, bool]:
    grid, meta = sub.to_occupancy(
        height_threshold=height_threshold,
        max_height=max_height,
        origin_centered=origin_centered,
    )
    if grid is None or meta is None:
        return occ_grid, last_ts, False
    if meta.timestamp <= last_ts:
        return occ_grid, last_ts, False
    last_ts = meta.timestamp

    # If the SLAM map dimensions change, replace the grid object.
    if (grid.width_cells != occ_grid.width_cells
            or grid.height_cells != occ_grid.height_cells
            or abs(grid.resolution - occ_grid.resolution) > 1e-6):
        return grid, last_ts, True

    # Update in-place to keep viewer references.
    occ_grid.grid = grid.grid
    occ_grid.resolution = grid.resolution
    occ_grid.origin = grid.origin
    occ_grid.width_cells = grid.width_cells
    occ_grid.height_cells = grid.height_cells
    return occ_grid, last_ts, False


# ---------------------------------------------------------------------------
# Mouse picking
# ---------------------------------------------------------------------------


def _pick_start_goal(
    viewer: MapViewer,
    detector: ObstacleDetector,
) -> tuple[tuple[float, float], tuple[float, float]]:
    if cv2 is None:
        raise SystemExit("OpenCV is required for mouse picking. Install opencv-python.")

    picked_start: Optional[tuple[float, float]] = None
    picked_goal: Optional[tuple[float, float]] = None

    def _on_mouse(event, x, y, _flags, _param):
        nonlocal picked_start, picked_goal
        if event == cv2.EVENT_LBUTTONDOWN:
            wx, wy = viewer.pixel_to_world(int(x), int(y))
            if picked_start is None:
                picked_start = (wx, wy)
            elif picked_goal is None:
                picked_goal = (wx, wy)
            else:
                picked_start = (wx, wy)
                picked_goal = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            picked_start = None
            picked_goal = None

    cv2.namedWindow(viewer.window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(viewer.window_name, _on_mouse)

    print("Click START then GOAL on the map window. Right-click to reset.")
    print("Press 'c' or Enter to confirm when both are set. Press 'q' to cancel.")

    while True:
        rx, ry, ryaw = detector.get_pose()
        ranges = detector.get_ranges()
        img = viewer.render_image(rx, ry, ryaw, ranges)

        # Overlay selections
        if picked_start is not None:
            sx, sy = viewer.world_to_pixel(*picked_start)  # noqa: SLF001
            cv2.drawMarker(img, (sx, sy), (0, 180, 255), cv2.MARKER_DIAMOND, 18, 2)
        if picked_goal is not None:
            gx, gy = viewer.world_to_pixel(*picked_goal)  # noqa: SLF001
            cv2.drawMarker(img, (gx, gy), (0, 0, 255), cv2.MARKER_STAR, 22, 2)

        cv2.putText(
            img,
            "Click START then GOAL | Right-click reset | c/Enter confirm | q cancel",
            (8, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (60, 60, 60),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow(viewer.window_name, img)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            raise SystemExit("Cancelled by user.")
        if picked_start is not None and picked_goal is not None:
            if key in (ord("c"), ord(" "), 13):
                return picked_start, picked_goal


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    args = parse_args()

    # --- 1. Safe boot + LocoClient ----------------------------------------
    print(f"Initialising SDK on interface '{args.iface}' (safe boot) ...")
    loco: LocoClient = hanger_boot_sequence(iface=args.iface)

    # --- 2. Obstacle detector (subscribes to rt/sportmodestate) ------------
    detector = ObstacleDetector(warn_distance=0.8, stop_distance=0.4, topic=args.sport_topic)
    detector.start()

    slam_odom = None
    if args.pose_source == "slam" and args.slam_odom_topic:
        slam_odom = SlamOdomSubscriber(args.slam_odom_topic)
        slam_odom.start()

    slam_points_sub = None
    slam_points: list[tuple[float, float]] = []
    if args.slam_points_topic:
        slam_points_sub = SlamPointCloudSubscriber(args.slam_points_topic)
        slam_points_sub.start()

        def _decode_slam_xy() -> list[tuple[float, float]]:
            msg, _ts = slam_points_sub.get_latest()
            if msg is None:
                return []
            try:
                fields = {f.name: f for f in msg.fields}
                if "x" not in fields or "y" not in fields:
                    return []
                point_step = int(msg.point_step)
                if point_step <= 0:
                    return []
                data = bytes(msg.data)
                if not data:
                    return []
                xoff = int(fields["x"].offset)
                yoff = int(fields["y"].offset)
                zoff = int(fields["z"].offset) if "z" in fields else xoff + 8
                import numpy as np
                dtype = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": ["<f4", "<f4", "<f4"],
                        "offsets": [xoff, yoff, zoff],
                        "itemsize": point_step,
                    }
                )
                arr = np.frombuffer(data, dtype=dtype, count=len(data) // point_step)
                xs = arr["x"][:: args.slam_points_stride]
                ys = arr["y"][:: args.slam_points_stride]
                zs = arr["z"][:: args.slam_points_stride]
                pts = []
                for x, y, z in zip(xs, ys, zs):
                    if z < args.slam_points_z_min or z > args.slam_points_z_max:
                        continue
                    pts.append((float(x), float(y)))
                return pts
            except Exception:
                return []

        def _refresh_slam_points() -> None:
            nonlocal slam_points
            slam_points = _decode_slam_xy()
    else:
        slam_points_sub = None
        slam_points = []

        def _refresh_slam_points() -> None:
            return

    # Lidar overlay fallback (when SLAM points are unavailable)
    lidar_points: list[tuple[float, float]] = []
    if args.lidar_topic:
        lidar_sub = SlamPointCloudSubscriber(args.lidar_topic)
        lidar_sub.start()

        def _refresh_lidar_points() -> None:
            nonlocal lidar_points
            msg, _ts = lidar_sub.get_latest()
            if msg is None:
                lidar_points = []
                return
            try:
                fields = {f.name: f for f in msg.fields}
                if "x" not in fields or "y" not in fields:
                    lidar_points = []
                    return
                point_step = int(msg.point_step)
                if point_step <= 0:
                    lidar_points = []
                    return
                data = bytes(msg.data)
                if not data:
                    lidar_points = []
                    return
                xoff = int(fields["x"].offset)
                yoff = int(fields["y"].offset)
                zoff = int(fields["z"].offset) if "z" in fields else xoff + 8
                import numpy as np
                dtype = np.dtype(
                    {
                        "names": ["x", "y", "z"],
                        "formats": ["<f4", "<f4", "<f4"],
                        "offsets": [xoff, yoff, zoff],
                        "itemsize": point_step,
                    }
                )
                arr = np.frombuffer(data, dtype=dtype, count=len(data) // point_step)
                xs = arr["x"][:: args.lidar_stride]
                ys = arr["y"][:: args.lidar_stride]
                zs = arr["z"][:: args.lidar_stride]
                pts = []
                for x, y, z in zip(xs, ys, zs):
                    if z < args.slam_points_z_min or z > args.slam_points_z_max:
                        continue
                    pts.append((float(x), float(y)))
                lidar_points = pts
            except Exception:
                lidar_points = []
    else:
        def _refresh_lidar_points() -> None:
            return

    def _get_pose():
        if slam_odom is not None and not slam_odom.is_stale():
            return slam_odom.get_pose()
        return detector.get_pose()

    # --- 2.1 SLAM service client -----------------------------------------
    need_slam_service = any([
        args.slam_start_mapping,
        bool(args.slam_end_map),
        bool(args.slam_init_map),
        args.slam_nav,
        args.slam_pause,
        args.slam_resume,
    ])
    slam_client = None
    slam_info = None
    if need_slam_service:
        slam_client = SlamOperateClient()
        slam_client.Init()
        slam_client.SetTimeout(5.0)
        slam_info = SlamInfoSubscriber(args.slam_info_topic, args.slam_key_topic)
        slam_info.start()

        if args.slam_start_mapping:
            resp = slam_client.start_mapping()
            print(f"SLAM start mapping: code={resp.code} raw={resp.raw}")
        if args.slam_init_map:
            resp = slam_client.init_pose(
                args.slam_init_x, args.slam_init_y, args.slam_init_z,
                args.slam_init_qx, args.slam_init_qy, args.slam_init_qz, args.slam_init_qw,
                args.slam_init_map,
            )
            print(f"SLAM init pose: code={resp.code} raw={resp.raw}")
        if args.slam_pause:
            resp = slam_client.pause_nav()
            print(f"SLAM pause nav: code={resp.code} raw={resp.raw}")
            return
        if args.slam_resume:
            resp = slam_client.resume_nav()
            print(f"SLAM resume nav: code={resp.code} raw={resp.raw}")
            return
        if args.slam_status:
            print("SLAM status (Ctrl+C to stop)...")
            try:
                while True:
                    time.sleep(0.5)
                    info = slam_info.get_info() if slam_info else None
                    key = slam_info.get_key() if slam_info else None
                    if info:
                        print(f"[slam_info] {info}")
                    if key:
                        print(f"[slam_key_info] {key}")
            except KeyboardInterrupt:
                print("SLAM status stopped.")
            return

    # Wait for first pose reading
    print("Waiting for SportModeState_ ...")
    time.sleep(1.0)
    if detector.is_stale():
        sys.exit("ERROR: no SportModeState_ data received.  "
                 "Is the robot connected and standing?")

    # --- 3. SLAM map subscriber -------------------------------------------
    slam_sub: Optional[SlamMapSubscriber] = None
    slam_ts: float = 0.0
    if not args.no_slam and args.load_map is None:
        slam_sub = SlamMapSubscriber()
        slam_sub.start()
        if not args.slam_no_lidar_switch:
            try:
                LidarSwitch().set("ON")
            except Exception:
                print("WARN: failed to publish utlidar switch ON")

    # --- 4. Map ------------------------------------------------------------
    if args.load_map:
        occ_grid = OccupancyGrid.load(args.load_map)
        print(f"Loaded map from {args.load_map}  "
              f"({occ_grid.width_cells}x{occ_grid.height_cells} cells)")
    elif slam_sub is not None:
        occ_grid, slam_ts = _wait_for_slam_map(
            slam_sub,
            timeout=args.slam_timeout,
            height_threshold=args.slam_height_threshold,
            max_height=args.slam_max_height,
            origin_centered=args.slam_origin_centered,
        )
        if occ_grid is None:
            print("WARN: no SLAM map received; using empty map fallback.")
            occ_grid = create_empty_map(
                width_m=args.map_width,
                height_m=args.map_height,
                resolution=args.resolution,
                origin_x=-args.map_width / 2,
                origin_y=-args.map_height / 2,
            )
        else:
            print(f"SLAM map received: {occ_grid.width_cells}x{occ_grid.height_cells} "
                  f"cells @ {occ_grid.resolution:.3f} m")
    else:
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

    # --- 5. Locomotion wrapper --------------------------------------------
    walker = Locomotion(loco, detector, max_vx=args.max_speed)

    # --- 6. Viewer (also used for mouse-pick) -----------------------------
    viewer: Optional[MapViewer] = None
    if args.viz or not args.no_pick:
        viewer = MapViewer(occ_grid, scale=4, inflation_radius=args.inflation)
        print("Map viewer enabled (press 'q' in the window to quit).")

    # --- 7. Start + Goal selection ----------------------------------------
    sx, sy, syaw = _get_pose()
    print(f"Robot pose: ({sx:+.2f}, {sy:+.2f}), yaw={math.degrees(syaw):+.1f} deg")

    if not args.no_pick:
        if viewer is None:
            viewer = MapViewer(occ_grid, scale=4, inflation_radius=args.inflation)
        try:
            picked_start, picked_goal = _pick_start_goal(viewer, detector)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
            if slam_client is not None and args.slam_end_map:
                resp = slam_client.end_mapping(args.slam_end_map)
                print(f"SLAM end mapping: code={resp.code} raw={resp.raw}")
            return
        psx, psy = picked_start
        goal_x, goal_y = picked_goal

        # Safety: prevent planning from a start far away from the robot.
        dist_to_robot = math.hypot(psx - sx, psy - sy)
        if dist_to_robot > 0.5:
            print("WARN: picked start is far from robot pose; using robot pose instead.")
            psx, psy = sx, sy
    else:
        if args.goal_x is None or args.goal_y is None:
            sys.exit("ERROR: goal_x and goal_y are required when --no-pick is set.")
        goal_x, goal_y = args.goal_x, args.goal_y
        psx, psy = sx, sy

    if args.goal_x is not None and args.goal_y is not None and args.no_pick:
        goal_x, goal_y = args.goal_x, args.goal_y

    print(f"Start : ({psx:+.2f}, {psy:+.2f})")
    print(f"Goal  : ({goal_x:+.2f}, {goal_y:+.2f})"
          + (f", yaw={math.degrees(args.goal_yaw):.1f} deg" if args.goal_yaw else ""))

    # --- 7.5 SLAM pose navigation (optional) ------------------------------
    if args.slam_nav:
        if slam_client is None:
            sys.exit("ERROR: slam_operate client not initialised.")

        def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
            half = yaw / 2.0
            return (0.0, 0.0, math.sin(half), math.cos(half))

        qx, qy, qz, qw = _yaw_to_quat(args.goal_yaw or 0.0)
        resp = slam_client.pose_nav(goal_x, goal_y, 0.0, qx, qy, qz, qw, mode=1)
        print(f"SLAM pose nav: code={resp.code} raw={resp.raw}")

        t0 = time.time()
        arrived = False
        while time.time() - t0 < args.slam_nav_timeout:
            time.sleep(0.2)
            if slam_info is None:
                continue
            key = slam_info.get_key()
            info = slam_info.get_info()
            for payload in (key, info):
                if not payload:
                    continue
                try:
                    import json
                    data = json.loads(payload)
                    if data.get("type") in ("task_result", "ctrl_info"):
                        if data.get("data", {}).get("is_arrived"):
                            arrived = True
                            break
                except Exception:
                    continue
            if arrived:
                break

        print("SLAM nav arrived." if arrived else "SLAM nav timeout.")
        walker.stop()
        if viewer is not None:
            viewer.close()
        if slam_client is not None and args.slam_end_map:
            resp = slam_client.end_mapping(args.slam_end_map)
            print(f"SLAM end mapping: code={resp.code} raw={resp.raw}")
        return

    # --- 8. Navigation loop ------------------------------------------------
    replan_count = 0
    goal_reached = False

    try:
        while replan_count < args.max_replans and not goal_reached:
            # 8a. Current pose
            cx, cy, cyaw = _get_pose()

            # 8b. Update map from SLAM (if available)
            if slam_sub is not None:
                occ_grid, slam_ts, replaced = _refresh_slam_map(
                    slam_sub,
                    occ_grid,
                    height_threshold=args.slam_height_threshold,
                    max_height=args.slam_max_height,
                    origin_centered=args.slam_origin_centered,
                    last_ts=slam_ts,
                )
                if replaced and viewer is not None:
                    viewer = MapViewer(occ_grid, scale=4, inflation_radius=args.inflation)

            # 8c. Add near-field obstacles from range sensors (safety)
            ranges = detector.get_ranges()
            occ_grid.mark_obstacle_from_range(cx, cy, cyaw, ranges)

            # 8c.1 Add SLAM pointcloud obstacles (if available)
            _refresh_slam_points()
            _refresh_lidar_points()
            if slam_points:
                overlay_points = slam_points
            else:
                overlay_points = []
                cyaw = math.cos(cyaw)
                syaw = math.sin(cyaw)
                for lx, ly in lidar_points:
                    wx = cx + (lx * cyaw - ly * syaw)
                    wy = cy + (lx * syaw + ly * cyaw)
                    overlay_points.append((wx, wy))
            if overlay_points:
                for ox, oy in overlay_points:
                    occ_grid.set_obstacle_world(ox, oy)

            # 8d. Inflate for planning
            inflated = occ_grid.inflate(radius_cells=args.inflation)

            # 8e. Grid coordinates
            start_cell = occ_grid.world_to_grid(cx, cy)
            goal_cell = occ_grid.world_to_grid(goal_x, goal_y)

            # 8f. A*
            print(f"\nPlanning (attempt {replan_count + 1}/{args.max_replans}) ...")
            print(f"  From cell {start_cell} to {goal_cell}")
            raw_path = astar(inflated, start_cell, goal_cell)

            if raw_path is None:
                print("ERROR: no path found.  Goal may be unreachable.")
                break

            # 8g. Smooth + world waypoints
            smoothed = smooth_path(raw_path, inflated)
            waypoints = grid_path_to_world_waypoints(
                smoothed, occ_grid, spacing_m=args.spacing
            )
            print(f"  {len(raw_path)} cells -> {len(smoothed)} smoothed "
                  f"-> {len(waypoints)} waypoints")

            # 8g-viz. Update viewer overlays with the new plan
            if viewer is not None:
                viewer.set_goal(goal_x, goal_y)
                full_world_path = [occ_grid.grid_to_world(r, c) for r, c in smoothed]
                viewer.set_path(full_world_path)
                viewer.set_waypoints(waypoints)

            # 8h. Walk each waypoint
            aborted = False
            for i, (wx, wy) in enumerate(waypoints):
                dist_to_goal = math.hypot(wx - goal_x, wy - goal_y)
                is_last = i == len(waypoints) - 1
                final_yaw = args.goal_yaw if is_last else None

                print(f"  [{i + 1}/{len(waypoints)}] -> ({wx:+.2f}, {wy:+.2f})"
                      f"  dist_to_goal={dist_to_goal:.2f} m")

                def _check_and_viz() -> bool:
                    """Obstacle check + live viewer update (called every tick)."""
                    if viewer is not None:
                        vx, vy, vyaw = detector.get_pose()
                        vranges = detector.get_ranges()
                        occ_grid.mark_obstacle_from_range(vx, vy, vyaw, vranges)
                        if slam_points:
                            overlay_points = slam_points
                        else:
                            overlay_points = []
                            cyaw = math.cos(vyaw)
                            syaw = math.sin(vyaw)
                            for lx, ly in lidar_points:
                                wx = vx + (lx * cyaw - ly * syaw)
                                wy = vy + (lx * syaw + ly * cyaw)
                                overlay_points.append((wx, wy))
                        viewer.update(vx, vy, vyaw, vranges, overlay_points)
                    return detector.front_blocked()

                reached = walker.walk_to(
                    wx, wy,
                    final_yaw=final_yaw,
                    timeout=30.0,
                    check_obstacle=_check_and_viz,
                )

                if not reached:
                    print("  ** Obstacle or timeout -- replanning **")
                    aborted = True
                    replan_count += 1

                    # Record newly sensed obstacles
                    obs_positions = detector.get_obstacle_world_positions()
                    for ox, oy in obs_positions:
                        occ_grid.set_obstacle_world(ox, oy)
                    print(f"  Added {len(obs_positions)} obstacle(s) to map")
                    break

            if not aborted:
                goal_reached = True

        # --- 9. Result -----------------------------------------------------
        print()
        if goal_reached:
            fx, fy, fyaw = detector.get_pose()
            dist = math.hypot(fx - goal_x, fy - goal_y)
            print(f"Goal reached!  Final pos: ({fx:+.2f}, {fy:+.2f}), "
                  f"error={dist:.2f} m")
        else:
            print(f"Navigation failed after {replan_count} replans.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        # --- 10. Cleanup ---------------------------------------------------
        walker.stop()
        if viewer is not None:
            viewer.close()
        save_path = "/tmp/final_obstacle_map.npz"
        occ_grid.save(save_path)
        print(f"Robot stopped.  Map saved to {save_path}")
        if slam_client is not None and args.slam_end_map:
            resp = slam_client.end_mapping(args.slam_end_map)
            print(f"SLAM end mapping: code={resp.code} raw={resp.raw}")


if __name__ == "__main__":
    main()
