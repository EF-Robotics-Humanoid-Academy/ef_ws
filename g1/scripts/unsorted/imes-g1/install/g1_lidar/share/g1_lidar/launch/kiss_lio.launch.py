#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.actions import ExecuteProcess
import os
import launch

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():

    g1_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')

    g1_ros_params = PathJoinSubstitution(
        [FindPackageShare('g1_lidar'), 'config', 'kiss_icp.yaml'])

    nsp = os.environ.get('G1_NS', 'g1_unit_001')

    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ("pointcloud_topic", 'livox/lidar'),
        ("kiss/odometry", 'odom'),
    ]

    kiss_icp_node = Node(
        namespace=nsp,
        remappings=rmp,
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        parameters=[
            {
                # ROS node configuration
                "base_frame": "base_link",
                "lidar_odom_frame": "odom",
                "publish_odom_tf": True,
                "invert_odom_tf": False,

                # ROS CLI arguments
                "publish_debug_clouds": False,
                "use_sim_time": False,
                "position_covariance": 0.1,
                "orientation_covariance": 0.1,
            },
            g1_ros_params,
        ],
    )

    ld = LaunchDescription()

    ld.add_action(g1_control_domain_id)
    ld.add_action(kiss_icp_node)

    return ld