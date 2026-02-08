#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.
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

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    g1_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')

    nsp = os.environ.get('G1_NS', 'g1_unit_001')
    rmp = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    filepath_config_ros2_utils = PathJoinSubstitution(
        [FindPackageShare('g1_webserver'), 'config', ('robot_webserver.yaml')]
    )

    # -------------------- Webserver
    node_webserver = Node(
        namespace=nsp,
        remappings=rmp,
        name='g1_webserver',
        package='g1_webserver',
        executable='webserver',
        output='screen',
        parameters=[filepath_config_ros2_utils]
    )

    # -------------------- Start VNC server
    vncserver_process = ExecuteProcess(
        cmd=['vncserver', ':1', '-geometry', '1920x1080',
             '-depth', '24', '-localhost', 'no'],
        output='screen',
        additional_env={'HOME': '/home/unitree'}
    )

    # -------------------- Start Websockify to forward VNC to browser
    websockify_process = ExecuteProcess(
        cmd=['websockify', '6080', '127.0.0.1:5901'],
        output='screen'
    )

    # -------------------- rosbridge_websocket
    rosbridge_launch_file = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
    ])

    rosbridge = GroupAction([
        PushRosNamespace(nsp),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file),
            launch_arguments={
                'address': TextSubstitution(text='0.0.0.0'),
                'port': TextSubstitution(text='9090'),
                # 'use_compression': TextSubstitution(text='true'),
                # 'fragment_timeout': TextSubstitution(text='600'),
            }.items(),
        ),
    ])

    ld = LaunchDescription()
    
    ld.add_action(g1_control_domain_id)

    ld.add_action(rosbridge)
    ld.add_action(vncserver_process)
    ld.add_action(websockify_process)
    
    ld.add_action(node_webserver)

    return ld
