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

    nsp = os.environ.get('G1_NS', 'g1_unit_001')

    g1_ros_params = PathJoinSubstitution(
        [FindPackageShare('g1_lidar'), 'config', 'fast_lio.yaml'])

    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/Laser_map', 'fast_lio/laser_map'),
        ('/Odometry', 'fast_lio/odom'),
        ('/cloud_effected', 'fast_lio/cloud_effected'),
        ('/cloud_registered', 'fast_lio/cloud_registered'),
        ('/cloud_registered_body', 'fast_lio/cloud_registered_body'),
        ('/path', 'fast_lio/path'),
    ]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    fast_lio_node = Node(
        namespace=nsp,
        remappings=rmp,
        name='fast_lio',
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[g1_ros_params,
                    {'use_sim_time': False}],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(g1_control_domain_id)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(fast_lio_node)

    return ld
