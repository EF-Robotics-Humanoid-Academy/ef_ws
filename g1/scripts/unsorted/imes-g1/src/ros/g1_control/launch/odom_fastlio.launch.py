#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lio_config = PathJoinSubstitution(
        [FindPackageShare('fast_lio'),
         'config',
         'mid360.yaml'],
    )

    remap = [('/fastlio/dometry', 'robot/odom')]

    fast_lio_node = Node(
        package='fast_lio',
        namespace='g1',
        executable='fastlio_mapping',
        parameters=[lio_config,
                    {'use_sim_time': False}],
        remappings=remap,
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(fast_lio_node)

    return ld
