#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of MYBOTSHOP GmbH nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
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
import yaml
import launch
import launch_ros.actions
from collections import OrderedDict
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def represent_ordereddict(dumper, data):
    return dumper.represent_dict(data.items())


yaml.add_representer(OrderedDict, represent_ordereddict,
                     Dumper=yaml.SafeDumper)


def replace_name(nw='domain_bridge'):

    nsp = os.environ.get('G1_NS', 'g1_unit_001')
    new_name = nsp + '_' + nw
    # OrderedDict to maintain key order
    g1_bridge_params = OrderedDict({
        'name': new_name,
        'from_domain': 0,
        'to_domain': 10,
        'topics': OrderedDict({            
            '/lf/lowstate': {
                'type': 'unitree_hg/msg/LowState',
                'remap': '/lf/lowstate',
            },
            '/lf/bmsstate': {
                'type': 'unitree_hg/msg/BmsState',
                'remap': '/lf/bmsstate',
            },
            '/lf/odommodestate': {
                'type': 'unitree_go/msg/SportModeState',
                'remap': '/lf/odommodestate',
            },
        })
    })

    package_share = get_package_share_directory('g1_platform')
    file_path = os.path.join(package_share, 'config', 'g1_domain_bridge.yaml')

    # Write the parameters to the YAML file cleanly
    with open(file_path, 'w') as yaml_file:
        yaml.dump(g1_bridge_params, yaml_file, sort_keys=False,
                  Dumper=yaml.SafeDumper, default_flow_style=False)


replace_name()


def generate_launch_description():

    g1_control_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')
    g1_rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    g1_cyclone = SetEnvironmentVariable('CYCLONEDDS_URI', PathJoinSubstitution([FindPackageShare('g1_bringup'), 'config', 'def_cyclone.xml']))

    nsp = os.environ.get('G1_NS', 'g1_unit_001')

    g1_ros_params = PathJoinSubstitution(
        [FindPackageShare('g1_platform'), 'config', 'g1_domain_bridge.yaml'])

    g1_bridge = launch_ros.actions.Node(
        namespace=nsp,
        name='g1_domain_bridge',
        package='domain_bridge',
        executable='domain_bridge',
        output='screen',
        arguments=[g1_ros_params],
    )

    ld = launch.LaunchDescription()

    ld.add_action(g1_control_domain_id)
    ld.add_action(g1_rmw)
    ld.add_action(g1_cyclone)
    
    ld.add_action(g1_bridge)

    return ld
