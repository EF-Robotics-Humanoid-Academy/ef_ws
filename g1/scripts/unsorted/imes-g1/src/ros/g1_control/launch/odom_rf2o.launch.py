#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                    'laser_scan_topic': '/g1/scan',
                    'odom_topic': '/rf2o/odom',
                    'publish_tf': True,
                    'base_frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'init_pose_from_topic': '',
                    'freq': 20.0}],
        ),
    ])
