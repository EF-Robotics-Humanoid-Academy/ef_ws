#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import numpy as np
from pyproj import Transformer

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class GpsConverter:
    def __init__(self, web_node):
        self.web_node = web_node
        self.latest_llh = None
        self.origin_ecef = None
        self.origin_set = False

        # Transformers
        self.transformer_ecef = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)  # LLH -> ECEF
        self.transformer_ecef_to_llh = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)  # ECEF -> LLH

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.web_node.create_subscription(
            NavSatFix, '/fixposition/odometry_llh', self.gnss_callback, qos_profile
        )
        self.web_node.create_subscription(
            Odometry, '/fixposition/odometry_enu_smooth', self.enu_pose_callback, qos_profile
        )

        # Publisher
        self.marker_publisher = self.web_node.create_publisher(MarkerArray, 'webserver/enu_markers', 10)

    def gnss_callback(self, msg: NavSatFix):
        """Update latest LLH from GNSS."""
        self.latest_llh = (msg.latitude, msg.longitude, msg.altitude)

    def enu_pose_callback(self, msg: Odometry):
        """Set ECEF origin based on ENU pose and latest LLH."""
        if not self.origin_set and self.latest_llh is not None:
            ecef_x, ecef_y, ecef_z = self.llh_to_ecef(*self.latest_llh)
            enu_x = msg.pose.pose.position.x
            enu_y = msg.pose.pose.position.y
            enu_z = msg.pose.pose.position.z

            lat0_rad = np.radians(self.latest_llh[0])
            lon0_rad = np.radians(self.latest_llh[1])
            R = np.array([
                [-np.sin(lon0_rad), -np.sin(lat0_rad) * np.cos(lon0_rad), np.cos(lat0_rad) * np.cos(lon0_rad)],
                [ np.cos(lon0_rad), -np.sin(lat0_rad) * np.sin(lon0_rad), np.cos(lat0_rad) * np.sin(lon0_rad)],
                [0, np.cos(lat0_rad), np.sin(lat0_rad)]
            ])

            origin_vec = np.array([ecef_x, ecef_y, ecef_z]) - R @ np.array([enu_x, enu_y, enu_z])
            self.origin_ecef = tuple(origin_vec)
            self.origin_set = True
            self.web_node.get_logger().info(f"[Origin Set] ENU origin ECEF: {self.origin_ecef}")

    def convert_waypoints(self):
        """Convert GPS waypoints to ENU coordinates and publish markers."""
        if not self.origin_set:
            self.web_node.get_logger().warn("Cannot convert waypoints: ENU origin not set yet.")
            return

        gps_data = self.load_gps_from_json()
        if gps_data:
            enu_coords = self.gps_to_enu_sequence(gps_data, fixed_alt=100.0)
            self.publish_dae_markers(enu_coords)
        else:
            self.web_node.get_logger().warn("No GPS data loaded.")

    def llh_to_ecef(self, lat, lon, alt):
        """Convert LLH to ECEF coordinates."""
        return self.transformer_ecef.transform(lon, lat, alt)

    def ecef_to_enu(self, x, y, z):
        """Convert ECEF to local ENU coordinates."""
        if self.origin_ecef is None:
            raise RuntimeError("ENU origin not set")

        dx, dy, dz = x - self.origin_ecef[0], y - self.origin_ecef[1], z - self.origin_ecef[2]
        lon0, lat0, _ = self.transformer_ecef_to_llh.transform(*self.origin_ecef)
        lat0_rad, lon0_rad = np.radians(lat0), np.radians(lon0)

        R = np.array([
            [-np.sin(lon0_rad), np.cos(lon0_rad), 0],
            [-np.sin(lat0_rad) * np.cos(lon0_rad), -np.sin(lat0_rad) * np.sin(lon0_rad), np.cos(lat0_rad)],
            [np.cos(lat0_rad) * np.cos(lon0_rad), np.cos(lat0_rad) * np.sin(lon0_rad), np.sin(lat0_rad)]
        ])

        return R @ np.array([dx, dy, dz])

    def gps_to_enu_sequence(self, gps_list, fixed_alt):
        """Convert list of GPS waypoints to ENU coordinates."""
        enu_coords = []
        for wp in gps_list:
            if "x" in wp and "y" in wp and "z" in wp:
                ecef = (wp["x"], wp["y"], wp["z"])
            else:
                alt = self.latest_llh[2] if self.latest_llh is not None else fixed_alt
                ecef = self.llh_to_ecef(wp["lat"], wp["lon"], alt)
            enu_coords.append(self.ecef_to_enu(*ecef))
        return enu_coords

    def load_gps_from_json(self):
        """Load GPS waypoints from JSON file."""
        try:
            with open(self.web_node.waypoints_dir, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            self.web_node.get_logger().error(f"Waypoint JSON file not found: {self.web_node.waypoints_dir}")
            return None

    def publish_dae_markers(self, enu_coords):
        """Publish ENU waypoints as MarkerArray."""
        self.web_node.get_logger().info(self.web_node.colorize(f"Publishing {len(enu_coords)} ENU markers.", "yellow"))
        self.web_node.get_logger().info(self.web_node.colorize(f"ENU origin (ECEF): {self.origin_ecef}", "yellow"))

        marker_array = MarkerArray()
        for i, coord in enumerate(enu_coords):
            self.web_node.get_logger().info(self.web_node.colorize(f"Marker {i}: {coord}", "blue"))
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.web_node.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = "package://g1_webserver/g1_webserver/static/models/gps.dae"
            marker.pose.position.x = coord[0]
            marker.pose.position.y = coord[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)
