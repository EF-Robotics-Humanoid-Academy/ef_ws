#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    go2_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')

    nsp = os.environ.get('G1_NS', 'g1_unit_001')
    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('cloud_in', 'hesai/lidar_points'),
        ('scan', 'scan'),

    ]

    node_pcd_laserscan = Node(
        namespace=nsp,
        remappings=rmp,
        name='pointcloud_to_laserscan',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[{
                'target_frame': 'hesai_lidar_link',
                'transform_tolerance': 0.01,
                'min_height': -0.45,
                'max_height': 1.0,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.15,
                'range_max': 40.0,
                'use_inf': False,
                'inf_epsilon': 1.0
        }],
    )

    node_simple_obstacle_avoidance = Node(
        namespace=nsp,
        remappings=rmp,
        name='simple_obstacle_avoidance',
        package='g1_obstacle_avoidance',
        executable='simple_obstacle_avoidance',
        output='screen',
        parameters=[{
            'laser_topic': 'scan',
            'cmd_vel_input_topic': 'unfiltered/steamdeck_joy_teleop/cmd_vel',
            'cmd_vel_output_topic': 'steamdeck_joy_teleop/cmd_vel',

            'stop_radius_m': 0.7,
            'slow_speed_mps': 0.2,
            'avoidance_enabled': True,
            'log_colors': True,
            'check_timer_hz': 20.0,
            'laser_yaw_offset_deg': 0.0,
            'publish_markers': True,
            'base_frame': 'base_link',
            'marker_height': 0.01,

            'slow_zone_radius_m': 0.9,
            'slow_marker_style': "annulus",
            'slow_marker_inner_margin_m': 0.05,

        }],
    )

    ld = LaunchDescription()

    ld.add_action(go2_control_domain_id)
    # ld.add_action(node_pcd_laserscan)
    ld.add_action(node_simple_obstacle_avoidance)

    return ld
