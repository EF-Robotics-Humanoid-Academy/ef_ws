#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#

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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    filepath_config_teleop = PathJoinSubstitution(
        [FindPackageShare('g1_control'), 'config', ('teleop.yaml')]
    )
        
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", "/g1/cmd_vel")],
        parameters=[filepath_config_teleop]
        )
 
    node_interactive_marker_twist_server = Node(
        package='interactive_marker_twist_server',
        executable='marker_server',
        name='twist_server_node',
        remappings={('cmd_vel', 'twist_marker_server/cmd_vel')},
        parameters=[filepath_config_teleop],
        output='screen',
    )

    node_joy = Node(
        namespace='g1_logitech',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_teleop]
    )

    node_teleop_twist_joy = Node(
        namespace='g1_logitech',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        remappings={('cmd_vel', 'joy_teleop/cmd_vel')},
        parameters=[filepath_config_teleop]
    )

    ld = LaunchDescription()
    
    ld.add_action(node_joy)
    ld.add_action(twist_mux_node)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_interactive_marker_twist_server)

    return ld

