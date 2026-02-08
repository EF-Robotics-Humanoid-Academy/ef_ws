#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

import os
import robot_upstart
from ament_index_python.packages import get_package_share_directory


def install_job(job_name, package_name, launch_filename, domain_id=10,
                e_user=None, rmw_type='rmw_fastrtps_cpp', disable_srvs=False):

    # 1. Uninstall existing service
    print(color_string(33, "Uninstalling existing service: {}".format(job_name)))
    os.system("sudo service {} stop".format(job_name))
    uninstall_job = robot_upstart.Job(
        name=job_name, rosdistro=os.environ['ROS_DISTRO'])
    uninstall_job.uninstall()

    # 2. Configure new service
    print(color_string(32, "Installing new service: {}".format(job_name)))
    linux_service = robot_upstart.Job(name=job_name,
                                      user=e_user,
                                      ros_domain_id=domain_id,
                                      rmw=rmw_type,
                                      workspace_setup=os.path.join(get_package_share_directory(
                                          'g1_bringup'), 'config/setup.bash')
                                      )

    linux_service.add(package=package_name, filename=launch_filename)
    linux_service.install()

    # 3. Set service state
    if disable_srvs:
        os.system("sudo systemctl disable {}".format(job_name))
        return

    # 4. Refresh for activation
    os.system(
        "sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name))


def color_string(color, string):
    return "\033[{}m{}\033[0m".format(color, string)


if __name__ == "__main__":

    jobs = [
        {"name": "g1-webserver",
         "package": "g1_webserver",
         "launch_filename": "launch/webserver.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         },

        {"name": "g1-highlevel-ros-controller",
         "package": "g1_platform",
         "launch_filename": "launch/highlevel_ros.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         "disable": True
         },

        {"name": "g1-domain-bridge",
         "package": "g1_platform",
         "launch_filename": "launch/domain_bridge.launch.py",
         "disable": True
         },

        {"name": "g1-audio-service",
         "package": "g1_platform",
         "launch_filename": "launch/audio.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         "disable": True
         },

        {"name": "g1-livox-mid360",
         "package": "g1_lidar",
         "launch_filename": "launch/livox_mid360.launch.py",
         "disable": True
         },

        {"name": "g1-pcd-to-scan",
         "package": "g1_lidar",
         "launch_filename": "launch/2d_scanner.launch.py",
         "disable": True
         },


        {"name": "g1-state-publisher",
         "package": "g1_platform",
         "launch_filename": "launch/state_publisher.launch.py",
         "disable": True
         },

        {"name": "g1-twist-mux",
         "package": "g1_control",
         "launch_filename": "launch/twistmux.launch.py",
         "disable": True
         },
        
        
        # Change to 23/29
        {"name": "g1-description",
         "package": "g1_description",
         "launch_filename": "launch/g1_29_description.launch.py",
         "disable": True
         },

        {"name": "g1-led-service",
         "package": "g1_platform",
         "launch_filename": "launch/led.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         "disable": True
         },

        # {"name": "g1-arm-7dof",
        #  "package": "g1_platform",
        #  "launch_filename": "launch/arm_7dof.launch.py",
        #  "rmw_type": "rmw_cyclonedds_cpp",
        #  "disable": True
        #  },

        {"name": "g1-video-stream",
         "package": "g1_platform",
         "launch_filename": "launch/videostream.launch.py",
         "disable": True
         },
        
        {"name": "g1-odometry",
         "package": "g1_lidar",
         "launch_filename": "launch/kiss_lio.launch.py",
         "disable": True
         },
        
        {"name": "g1-obstacle-avoidance",
         "package": "g1_obstacle_avoidance",
         "launch_filename": "launch/obstacle_avoidance.launch.py",
         "rmw_type": "rmw_cyclonedds_cpp",
         "disable": True
         },
        
        # {"name": "g1-llm",
        #  "package": "g1_llm",
        #  "launch_filename": "launch/llm.launch.py",
        #  "rmw_type": "rmw_cyclonedds_cpp",
        #  "user": "root",
        #  "disable": True
        #  },
    ]

    for job in jobs:
        install_job(job_name=job["name"],
                    package_name=job["package"],
                    e_user=job.get("user", None),
                    launch_filename=job["launch_filename"],
                    disable_srvs=job.get("disable", False),
                    rmw_type=job.get("rmw_type", "rmw_fastrtps_cpp"),
                    )
