"""
map_viewer.py
=============

Real-time 2D visualisation of the dynamic occupancy grid, robot pose,
planned path, and obstacle sensor readings.

Uses OpenCV ``imshow`` for rendering so it can be called from any thread
at any rate (typically 10 Hz from the navigation control loop).

Can be used in two ways:

1. **Imported by navigate.py** -- pass ``--viz`` to see the map while the
   robot navigates.  The viewer's ``update()`` method is injected into the
   ``check_obstacle`` callback so it refreshes at control-loop rate.

2. **Standalone** -- connects to the robot, builds a live obstacle map, and
   displays it without commanding any motion::

       python map_viewer.py eth0
"""
from __future__ import annotations

import math
from typing import Sequence, Optional

import cv2
import numpy as np

from create_map import OccupancyGrid
from slam_map import SlamOdomSubscriber


# ---------------------------------------------------------------------------
# Colour palette (BGR)
# ---------------------------------------------------------------------------
_COL_FREE = np.array([18, 18, 18], dtype=np.uint8)          # dark background
_COL_OBSTACLE = np.array([230, 230, 230], dtype=np.uint8)   # bright obstacles
_COL_INFLATED = np.array([90, 90, 90], dtype=np.uint8)      # mid grey
_COL_PATH = np.array([255, 180, 80], dtype=np.uint8)        # light blue
_COL_WAYPOINT = np.array([255, 120, 0], dtype=np.uint8)     # blue
_COL_TRAIL = np.array([80, 140, 80], dtype=np.uint8)        # faint green
_COL_ROBOT = np.array([0, 220, 0], dtype=np.uint8)          # green
_COL_GOAL = np.array([0, 0, 255], dtype=np.uint8)           # red
_COL_RANGE_OK = (0, 180, 0)                                 # green line
_COL_RANGE_WARN = (0, 180, 255)                             # orange line
_COL_RANGE_BLOCK = (0, 0, 255)                              # red line
_COL_SLAM_POINTS = np.array([0, 120, 255], dtype=np.uint8)  # orange

# Scale: how many display pixels per grid cell.  Increase for a larger window.
_DEFAULT_SCALE = 4


class MapViewer:
    """Renders the occupancy grid and overlays into an OpenCV window.

    All coordinates are in world metres; the viewer converts them internally.
    """

    def __init__(
        self,
        occ_grid: OccupancyGrid,
        window_name: str = "Obstacle Map",
        scale: int = _DEFAULT_SCALE,
        inflation_radius: int = 3,
    ):
        self.grid = occ_grid
        self.window_name = window_name
        self.scale = scale
        self.inflation_radius = inflation_radius

        # Robot trail (list of world (x, y) positions)
        self._trail: list[tuple[float, float]] = []
        self._max_trail = 2000

        # Latest overlay data (set via update / set_*)
        self._path: list[tuple[float, float]] = []
        self._waypoints: list[tuple[float, float]] = []
        self._goal: tuple[float, float] | None = None

    # ------------------------------------------------------------------
    # Overlay setters (call once per replan)
    # ------------------------------------------------------------------

    def set_path(self, world_path: list[tuple[float, float]]) -> None:
        """Set the planned path (list of world (x, y) points)."""
        self._path = list(world_path)

    def set_waypoints(self, waypoints: list[tuple[float, float]]) -> None:
        """Set the current waypoint list."""
        self._waypoints = list(waypoints)

    def set_goal(self, gx: float, gy: float) -> None:
        self._goal = (gx, gy)

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_pixel(self, wx: float, wy: float) -> tuple[int, int]:
        """World (x, y) -> display pixel (px, py)."""
        return self._world_to_pixel(wx, wy)

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[int, int]:
        """World (x, y) -> display pixel (px, py).

        px = col * scale,  py = (height - 1 - row) * scale  (Y-up flip).
        """
        row, col = self.grid.world_to_grid(wx, wy)
        px = col * self.scale + self.scale // 2
        py = (self.grid.height_cells - 1 - row) * self.scale + self.scale // 2
        return (px, py)

    def pixel_to_world(self, px: int, py: int) -> tuple[float, float]:
        """Display pixel (px, py) -> world (x, y) at the cell centre."""
        col = int(px // self.scale)
        row = int((self.grid.height_cells - 1) - (py // self.scale))
        row = max(0, min(self.grid.height_cells - 1, row))
        col = max(0, min(self.grid.width_cells - 1, col))
        return self.grid.grid_to_world(row, col)

    # ------------------------------------------------------------------
    # Main render + display
    # ------------------------------------------------------------------

    def update(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        ranges: list[float] | None = None,
        slam_points: Optional[np.ndarray] = None,
    ) -> None:
        """Render the current map state and display it.

        Call this at control-loop rate (~10 Hz).  It is fast enough for
        grids up to ~200x200 cells at scale=4.

        Args:
            robot_x:  Robot world x (metres).
            robot_y:  Robot world y (metres).
            robot_yaw: Robot heading (radians).
            ranges:   Optional ``range_obstacle[4]`` for sensor lines.
        """
        # Record trail
        self._trail.append((robot_x, robot_y))
        if len(self._trail) > self._max_trail:
            self._trail.pop(0)

        img = self._render(robot_x, robot_y, robot_yaw, ranges, slam_points)
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)

    def render_image(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        ranges: list[float] | None = None,
        slam_points: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Return a rendered map image without showing it."""
        return self._render(robot_x, robot_y, robot_yaw, ranges, slam_points)

    def _render(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        ranges: list[float] | None,
        slam_points: Optional[np.ndarray],
    ) -> np.ndarray:
        H = self.grid.height_cells
        W = self.grid.width_cells
        s = self.scale

        # --- base image: free / obstacle ---
        img = np.full((H * s, W * s, 3), _COL_FREE, dtype=np.uint8)

        # Inflated obstacles (draw first, lighter)
        try:
            inflated = self.grid.inflate(self.inflation_radius)
            infl_mask = (inflated > 0) & (self.grid.grid == 0)
            # Scale up mask
            infl_big = np.kron(infl_mask[::-1], np.ones((s, s), dtype=bool))
            img[infl_big] = _COL_INFLATED
        except Exception:
            pass

        # Raw obstacles
        obs_mask = self.grid.grid > 0
        obs_big = np.kron(obs_mask[::-1], np.ones((s, s), dtype=bool))
        img[obs_big] = _COL_OBSTACLE

        # --- trail ---
        for i in range(1, len(self._trail)):
            p0 = self._world_to_pixel(*self._trail[i - 1])
            p1 = self._world_to_pixel(*self._trail[i])
            cv2.line(img, p0, p1, _COL_TRAIL.tolist(), 1, cv2.LINE_AA)

        # --- planned path ---
        if len(self._path) >= 2:
            pts = [self._world_to_pixel(x, y) for x, y in self._path]
            for i in range(1, len(pts)):
                cv2.line(img, pts[i - 1], pts[i], _COL_PATH.tolist(), 2, cv2.LINE_AA)

        # --- waypoints ---
        for wx, wy in self._waypoints:
            px, py = self._world_to_pixel(wx, wy)
            cv2.circle(img, (px, py), max(3, s // 2), _COL_WAYPOINT.tolist(), -1,
                       cv2.LINE_AA)

        # --- goal ---
        if self._goal is not None:
            gx, gy = self._world_to_pixel(*self._goal)
            cv2.drawMarker(img, (gx, gy), _COL_GOAL.tolist(),
                           cv2.MARKER_STAR, max(12, s * 3), 2, cv2.LINE_AA)

        # --- range sensor lines ---
        if ranges is not None:
            offsets = [0.0, -math.pi / 2, math.pi, math.pi / 2]
            rpx, rpy = self._world_to_pixel(rx, ry)
            for i, offset in enumerate(offsets):
                if i >= len(ranges):
                    break
                dist = ranges[i]
                if dist <= 0.01 or dist >= 5.0:
                    continue
                angle = ryaw + offset
                ex = rx + dist * math.cos(angle)
                ey = ry + dist * math.sin(angle)
                epx, epy = self._world_to_pixel(ex, ey)
                if dist < 0.4:
                    col = _COL_RANGE_BLOCK
                elif dist < 0.8:
                    col = _COL_RANGE_WARN
                else:
                    col = _COL_RANGE_OK
                cv2.line(img, (rpx, rpy), (epx, epy), col, 1, cv2.LINE_AA)

        # --- SLAM points overlay ---
        if slam_points is not None and len(slam_points) > 0:
            for px, py in slam_points:
                pxi, pyi = self._world_to_pixel(float(px), float(py))
                if 0 <= pxi < img.shape[1] and 0 <= pyi < img.shape[0]:
                    img[pyi, pxi] = _COL_SLAM_POINTS

        # --- robot marker (circle + heading arrow) ---
        rpx, rpy = self._world_to_pixel(rx, ry)
        radius = max(s, 5)
        cv2.circle(img, (rpx, rpy), radius, _COL_ROBOT.tolist(), -1, cv2.LINE_AA)
        # Heading arrow
        arrow_len = radius * 3
        ax = int(rpx + arrow_len * math.cos(-ryaw))   # minus because Y-flip
        ay = int(rpy + arrow_len * math.sin(-ryaw))
        # The Y-flip in pixel space means we need to invert the y component:
        # In world: forward = (cos(yaw), sin(yaw))
        # In pixel: x_pix = col (increases right = positive cos OK)
        #           y_pix = H - row (increases down, so sin must be negated)
        ax = int(rpx + arrow_len * math.cos(ryaw))
        ay = int(rpy - arrow_len * math.sin(ryaw))
        cv2.arrowedLine(img, (rpx, rpy), (ax, ay), _COL_ROBOT.tolist(), 2,
                        cv2.LINE_AA, tipLength=0.35)

        # --- HUD text ---
        hud_y = 20
        for line in [
            f"pos: ({rx:+.2f}, {ry:+.2f})  yaw: {math.degrees(ryaw):+.1f} deg",
            f"obstacles: {int(np.sum(self.grid.grid > 0))} cells",
            f"trail: {len(self._trail)} pts",
        ]:
            cv2.putText(img, line, (8, hud_y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (80, 80, 80), 1, cv2.LINE_AA)
            hud_y += 18

        # --- axis labels (corners) ---
        ox, oy = self.grid.origin
        w_m = self.grid.width_cells * self.grid.resolution
        h_m = self.grid.height_cells * self.grid.resolution
        cv2.putText(img, f"({ox:.1f},{oy:.1f})", (4, H * s - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
        cv2.putText(img, f"({ox + w_m:.1f},{oy + h_m:.1f})", (W * s - 90, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        return img

    def close(self) -> None:
        cv2.destroyWindow(self.window_name)


# ---------------------------------------------------------------------------
# Standalone mode: connect to robot, build live map, display
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import sys
    import time

    parser = argparse.ArgumentParser(description="Live obstacle map viewer.")
    parser.add_argument("--iface", default="eth0", help="network interface for DDS")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")
    parser.add_argument("--sport-topic", default="rt/odommodestate", help="SportModeState topic name")
    parser.add_argument("--slam-odom-topic", default="rt/unitree/slam_mapping/odom", help="SLAM odom topic (optional)")
    parser.add_argument("--slam-points-topic", default="", help="SLAM points topic (optional)")
    parser.add_argument("--slam-points-stride", type=int, default=4, help="Subsample SLAM points")
    parser.add_argument("--lidar-topic", default="rt/utlidar/cloud_livox_mid360", help="Lidar PointCloud2 DDS topic")
    parser.add_argument("--lidar-stride", type=int, default=6, help="Subsample lidar points")
    parser.add_argument("--lidar-z-min", type=float, default=-0.5, help="Min Z for lidar overlay")
    parser.add_argument("--lidar-z-max", type=float, default=1.5, help="Max Z for lidar overlay")
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from obstacle_detection import ObstacleDetector
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

    ChannelFactoryInitialize(args.domain_id, args.iface)

    detector = ObstacleDetector(warn_distance=0.8, stop_distance=0.4, topic=args.sport_topic)
    detector.start()

    slam_odom = SlamOdomSubscriber(args.slam_odom_topic) if args.slam_odom_topic else None
    if slam_odom is not None:
        slam_odom.start()

    slam_points: list[tuple[float, float]] = []
    lidar_points: list[tuple[float, float]] = []

    def _decode_xy(msg: PointCloud2_, stride: int, zmin: float, zmax: float) -> list[tuple[float, float]]:
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
            dtype = np.dtype(
                {
                    "names": ["x", "y", "z"],
                    "formats": ["<f4", "<f4", "<f4"],
                    "offsets": [xoff, yoff, zoff],
                    "itemsize": point_step,
                }
            )
            arr = np.frombuffer(data, dtype=dtype, count=len(data) // point_step)
            xs = arr["x"][:: stride]
            ys = arr["y"][:: stride]
            zs = arr["z"][:: stride]
            pts = []
            for x, y, z in zip(xs, ys, zs):
                if z < zmin or z > zmax:
                    continue
                pts.append((float(x), float(y)))
            return pts
        except Exception:
            return []

    def _slam_points_cb(msg: PointCloud2_) -> None:
        slam_points[:] = _decode_xy(msg, args.slam_points_stride, args.lidar_z_min, args.lidar_z_max)

    def _lidar_points_cb(msg: PointCloud2_) -> None:
        lidar_points[:] = _decode_xy(msg, args.lidar_stride, args.lidar_z_min, args.lidar_z_max)

    if args.slam_points_topic:
        slam_sub = ChannelSubscriber(args.slam_points_topic, PointCloud2_)
        slam_sub.Init(_slam_points_cb, 10)

    if args.lidar_topic:
        lidar_sub = ChannelSubscriber(args.lidar_topic, PointCloud2_)
        lidar_sub.Init(_lidar_points_cb, 10)

    print("Waiting for SportModeState_ ...")
    time.sleep(1.0)
    if detector.is_stale():
        sys.exit("ERROR: no data.  Is the robot connected?")

    # Create map centred on robot's starting position
    sx, sy, _ = detector.get_pose()
    occ_grid = OccupancyGrid(
        width_m=10.0, height_m=10.0, resolution=0.1,
        origin_x=sx - 5.0, origin_y=sy - 5.0,
    )
    print(f"Map centred on robot start ({sx:.2f}, {sy:.2f})")

    viewer = MapViewer(occ_grid, window_name="Live Obstacle Map", scale=4)

    print("Displaying live map.  Press 'q' or ESC to quit.")
    last_log = time.time()
    try:
        while True:
            if detector.is_stale():
                time.sleep(0.1)
                continue

            if slam_odom is not None and not slam_odom.is_stale():
                x, y, yaw = slam_odom.get_pose()
            else:
                x, y, yaw = detector.get_pose()
            ranges = detector.get_ranges()

            # Mark obstacles on the grid
            occ_grid.mark_obstacle_from_range(x, y, yaw, ranges)

            # Render
            if slam_points:
                overlay = slam_points
            else:
                # Lidar points are in the robot frame; transform to world.
                overlay = []
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                for lx, ly in lidar_points:
                    wx = x + (lx * cy - ly * sy)
                    wy = y + (lx * sy + ly * cy)
                    overlay.append((wx, wy))
            viewer.update(x, y, yaw, ranges, np.array(overlay) if overlay else None)

            now = time.time()
            if now - last_log > 2.0:
                print(
                    f"overlay points: slam={len(slam_points)} lidar={len(lidar_points)} "
                    f"(using {'slam' if slam_points else 'lidar'})"
                )
                last_log = now

            key = cv2.waitKey(50) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        save_path = "/tmp/live_obstacle_map.npz"
        occ_grid.save(save_path)
        print(f"\nMap saved to {save_path}")
