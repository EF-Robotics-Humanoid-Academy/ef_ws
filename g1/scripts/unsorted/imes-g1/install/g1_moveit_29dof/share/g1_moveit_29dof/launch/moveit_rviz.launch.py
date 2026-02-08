#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Salman Omar Sohail

import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def replace_namespace():

    file_path = os.path.join(
        get_package_share_directory("g1_moveit_29dof"),
        "config",
        "moveit_controllers.yaml",
    )
    # Load the YAML content (assuming it's in a file called 'config.yaml')
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Get the new namespace from the environment variable
    # Default to 'g1_moveit2' if not set
    new_ns = os.environ.get('G1_NS', 'g1_unit_001')
    new_namespace = new_ns+'/g1_moveit2'

    # Update the controller names with the new namespace
    config['controller_names'] = [
        f"{new_namespace}/{name.split('/')[-1]}" for name in config['controller_names']]

    # Update the controller configurations with the new namespace
    for key in list(config.keys()):
        if key.startswith('g1_moveit2/'):
            new_key = key.replace('g1_moveit2', new_namespace)
            config[new_key] = config.pop(key)

    # Save the updated YAML content back to the file (or print it)
    with open(file_path, 'w') as file:
        yaml.safe_dump(config, file)

replace_namespace()

def generate_launch_description():
    nsp = os.environ.get('G1_NS', 'g1_unit_001')
    rmp = [
        ('/tf', nsp+'/tf'),
        ('/tf_static', nsp+'/tf_static'),
        ('/diagnostics', nsp+'/diagnostics'),
        ('/joint_states', nsp+'/joint_states'),
        ('/robot_description', nsp+'/robot_description'),        
        ('/goal_pose', nsp+'/goal_pose'),
        ('/initialpose', nsp+'/initialpose'),
        ('/monitored_planning_scene', nsp+'/monitored_planning_scene'),
        ('/motion_plan_request', nsp+'/motion_plan_request'),
        ('/planning_scene', nsp+'/planning_scene'),
        ('/planning_scene_world', nsp+'/planning_scene_world'),
    ]

    # Rviz config save file
    rviz_config = os.path.join(
        get_package_share_directory("g1_moveit_29dof"),
        "config",
        "moveit.rviz"
    )

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("g1_description"),
            "xacro",
            "g1_29.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot description Semantic config
    robot_description_semantic_path = os.path.join(
        get_package_share_directory("g1_moveit_29dof"),
        "config",
        "g1_29.srdf",
    )
    with open(robot_description_semantic_path, "r") as file:
        robot_description_semantic_config = file.read()

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization \
            default_planner_request_adapters/FixWorkspaceBounds \
             default_planner_request_adapters/FixStartStateBounds \
            default_planner_request_adapters/FixStartStateCollision \
            default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml_path = os.path.join(
        get_package_share_directory("g1_moveit_29dof"),
        "config",
        "ompl_planning.yaml",
    )
    with open(ompl_planning_yaml_path, "r") as file:
        ompl_planning_yaml = yaml.safe_load(file)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # kinematics yaml
    kinematics_yaml_path = os.path.join(
        get_package_share_directory("g1_moveit_29dof"),
        "config",
        "kinematics.yaml",
    )
    with open(kinematics_yaml_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    ld = LaunchDescription()

    rviz_node = Node(
        # namespace=nsp,
        remappings=rmp,
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ]
    )

    ld.add_action(rviz_node)

    return ld
