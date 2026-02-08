#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

from std_msgs.msg import Bool
from rclpy.action import ActionClient
# -------------------------------------------- Import Custom Interfaces
from g1_interface.srv import G1Modes
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# -------------------------------------------- Advanced Features
# from g1_webserver.libllm import ROS2LLM
# from g1_webserver.libgpsconverter import GpsConverter


# -------------------------------------------- Open Source Web Interface
class ROS2CustomWebInterface():

    def __init__(self, web_node):

        self.web_node = web_node
        self.web_node.create_timer(
            1, self.ros2_webserver_control_buttons_callback)

        # -------------------------------------------- Advanced Features
        # self.llm = ROS2LLM(self.web_node)
        # self.gps_converter = GpsConverter(self.web_node)
        
        # -------------------------------------------- G1 Control Client
        self.g1_hardware_client = self.web_node.create_client(G1Modes, 'hardware_modes')
        # ---- FollowJointTrajectory action client
        self.follow_client = ActionClient(
            self.web_node, FollowJointTrajectory,
            'dual_arm/follow_joint_trajectory'
        )

        # Store handles/state
        self._follow_goal_handle = None
        self._follow_send_goal_future = None
        self._follow_result_future = None
        self._follow_last_feedback = None

        # Poll futures so we can log results without blocking
        self.web_node.create_timer(0.2, self._spin_follow_futures)
        
        self.web_node.get_logger().info(self.web_node.colorize(
            "ROS2 Custom Web Interface Loaded", "green"))

    # -------------------------------------------- Webserver Control Buttons
    def ros2_webserver_control_buttons_callback(self):

        try:
            # -------------------------------------------- Robot Computer Block 1
            if self.web_node.control_block_11:
                self.web_node.get_logger().info("Control Block 11 is active")
                self.g1_hardware_client.call_async(G1Modes.Request(request_data='damp'))
                self.web_node.control_block_11 = False
            elif self.web_node.control_block_12:
                self.web_node.get_logger().info("Control Block 12 is active")
                self.send_follow_trajectory()
                self.web_node.control_block_12 = False
            elif self.web_node.control_block_13:
                self.web_node.get_logger().info("Control Block 13 is active")
                self.send_follow_trajectory_cross()
                self.web_node.control_block_13 = False
            elif self.web_node.control_block_14:
                self.web_node.get_logger().info("Control Block 14 is active")
                self.web_node.control_block_14 = False
            elif self.web_node.control_block_15:
                self.web_node.get_logger().info("Control Block 15 is active")
                self.cancel_follow_trajectory()
                self.web_node.control_block_15 = False
            elif self.web_node.control_block_16:
                self.web_node.get_logger().info("Control Block 16 is active")
                self.send_follow_trajectory_down()
                self.web_node.control_block_16 = False
            elif self.web_node.control_block_17:
                self.web_node.get_logger().info("Control Block 17 is active")
                self.send_follow_trajectory_uncross()
                self.web_node.control_block_17 = False
            elif self.web_node.control_block_18:
                self.web_node.get_logger().info("Control Block 18 is active")
                self.web_node.control_block_18 = False

            # -------------------------------------------- Robot Computer Block 2
            elif self.web_node.control_block_21:
                self.web_node.get_logger().info("Control Block 21 is active")
                self.web_node.control_block_21 = False
            elif self.web_node.control_block_22:
                self.web_node.get_logger().info("Control Block 22 is active")
                self.web_node.control_block_22 = False
            elif self.web_node.control_block_23:
                self.web_node.get_logger().info("Control Block 23 is active")
                self.web_node.control_block_23 = False
            elif self.web_node.control_block_24:
                self.web_node.get_logger().info("Control Block 24 is active")
                self.web_node.control_block_24 = False
            elif self.web_node.control_block_25:
                self.web_node.get_logger().info("Control Block 25 is active")
                self.web_node.control_block_25 = False
            elif self.web_node.control_block_26:
                self.web_node.get_logger().info("Control Block 26 is active")
                self.web_node.control_block_26 = False
            elif self.web_node.control_block_27:
                self.web_node.get_logger().info("Control Block 27 is active")
                self.web_node.control_block_27 = False
            elif self.web_node.control_block_28:
                self.web_node.get_logger().info("Control Block 28 is active")
                self.web_node.control_block_28 = False

            # -------------------------------------------- Robot Computer Block 3
            elif self.web_node.control_block_31:
                self.web_node.get_logger().info("Control Block 31 is active")
                self.web_node.control_block_31 = False
            elif self.web_node.control_block_32:
                self.web_node.get_logger().info("Control Block 32 is active")
                self.web_node.control_block_32 = False
            elif self.web_node.control_block_33:
                self.web_node.get_logger().info("Control Block 33 is active")
                self.web_node.control_block_33 = False
            elif self.web_node.control_block_34:
                self.web_node.get_logger().info("Control Block 34 is active")
                self.web_node.control_block_34 = False
            elif self.web_node.control_block_35:
                self.web_node.get_logger().info("Control Block 35 is active")
                self.web_node.control_block_35 = False
            elif self.web_node.control_block_36:
                self.web_node.get_logger().info("Control Block 36 is active")
                self.web_node.control_block_36 = False
            elif self.web_node.control_block_37:
                self.web_node.get_logger().info("Control Block 37 is active")
                self.web_node.control_block_37 = False
            elif self.web_node.control_block_38:
                self.web_node.get_logger().info("Control Block 38 is active")
                self.web_node.control_block_38 = False

            # -------------------------------------------- Robot Computer Block 4
            elif self.web_node.control_block_41:  # Reboot
                self.web_node.get_logger().info("Control Block 41 is active")
                self.web_node.sudo_command_line("sudo -S shutdown -r now")
                self.web_node.control_block_41 = False
            elif self.web_node.control_block_42:  # Update system
                self.web_node.get_logger().info("Control Block 42 is active")
                # self.web_node.sudo_command_line("sudo -S apt-get update")
                self.web_node.control_block_42 = False
            elif self.web_node.control_block_43:  # Clear Journal and History
                self.web_node.get_logger().info("Control Block 43 is active")
                # self.web_node.sudo_command_line("sudo -S journalctl --vacuum-time=7d; history -c")
                self.web_node.control_block_43 = False
            elif self.web_node.control_block_44:  # Reload Udev Rules
                self.web_node.get_logger().info("Control Block 44 is active")
                self.web_node.sudo_command_line(
                    "sudo -S udevadm control --reload && sudo -S udevadm trigger")
                self.web_node.control_block_44 = False
            elif self.web_node.control_block_45:  # Shutdown
                self.web_node.get_logger().info("Control Block 45 is active")
                self.web_node.sudo_command_line("sudo -S shutdown -h now")
                self.web_node.control_block_45 = False
            elif self.web_node.control_block_46:  # Upgrade System
                self.web_node.get_logger().info("Control Block 46 is active")
                # self.web_node.sudo_command_line("sudo -S apt upgrade -y")
                self.web_node.control_block_46 = False
            elif self.web_node.control_block_47:  # Restart NetworkManager
                self.web_node.get_logger().info("Control Block 47 is active")
                self.web_node.sudo_command_line(
                    "sudo -S systemctl restart NetworkManager")
                self.web_node.control_block_47 = False
            elif self.web_node.control_block_48:  # Clear Memory Cache
                self.web_node.get_logger().info("Control Block 48 is active")
                self.web_node.sudo_command_line(
                    "sudo -S sync; sudo -S sysctl -w vm.drop_caches=3")
                self.web_node.control_block_48 = False

            # -------------------------------------------- GPS Navi
            elif self.web_node.gps_navi:
                self.web_node.get_logger().info("GPS Waypoint is active")
                self.gps_converter.convert_waypoints()
                self.web_node.gps_navi = False
                
            # -------------------------------------------- LLM Action
            elif self.web_node.control_block_gpt:
                self.web_node.get_logger().info("Control Block GPT is active")
                self.web_node.control_block_gpt = False
                # self.llm.driver()
                
        except Exception as e:
            self.web_node.get_logger().error(
                f"Error in ros2_webserver_control_buttons_callback: {e}")

        # -------------------------------------------- Slider Rig
        try:
            if self.web_node.rig1 or self.web_node.rig2 or self.web_node.rig3:
                self.web_node.get_logger().info("Rig 1: " + str(self.web_node.rig1)
                                                + " Rig 2: " +
                                                str(self.web_node.rig2)
                                                + " Rig 3: " + str(self.web_node.rig3))

                # self.rig_control.base_plate = self.web_node.rig1
                # self.rig_control.left_plate = self.web_node.rig2
                # self.rig_control.right_plate = self.web_node.rig3
                # self.srv_rig_control.call_async(self.rig_control)

                # Reset Mechanism
                self.web_node.rig1 = None
                self.web_node.rig2 = None
                self.web_node.rig3 = None
        except Exception as e:
            self.web_node.get_logger().error(f"Rig Slider Error: {e}")

        try:
            if self.web_node.e_stop == 200:
                self.web_node.robot_estop.publish(Bool(data=True))
            if self.web_node.e_stop == 250:
                self.web_node.robot_estop.publish(Bool(data=False))
                self.web_node.e_stop = None
        except Exception as e:
            self.web_node.get_logger().error(f"Robot E-Stop Error: {e}")
            
    # --- Add to ROS2CustomWebInterface -----------------------------------------

    def send_follow_trajectory_down(self):
        # Ensure server exists (non-blocking quick check)
        if not self.follow_client.wait_for_server(timeout_sec=0.0):
            self.web_node.get_logger().warn("FollowJointTrajectory server not available yet")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_down_trajectory()

        self._follow_send_goal_future = self.follow_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback_cb
        )
        self._follow_send_goal_future.add_done_callback(self._follow_goal_response_cb)
        self.web_node.get_logger().info("FollowJointTrajectory DOWN goal sent")


    def _build_dual_arm_down_trajectory(self) -> JointTrajectory:
        """
        Dual-arm 'down/rest' pose based on your single-arm examples:
        left:  [0.0,  +0.2, 0.0, 1.57, 0.0, 0.0, 0.0]
        right: [0.0,  -0.2, 0.0, 1.57, 0.0, 0.0, 0.0]
        """
        jt = JointTrajectory()
        jt.joint_names = [
            "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
            "left_elbow","left_wrist_roll","left_wrist_pitch","left_wrist_yaw",
            "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
            "right_elbow","right_wrist_roll","right_wrist_pitch","right_wrist_yaw"
        ]

        p = JointTrajectoryPoint()
        p.positions = [
            0.0,  0.2,  0.0,  1.57, 0.0, 0.0, 0.0,   # left arm down
            0.0, -0.2,  0.0,  1.57, 0.0, 0.0, 0.0    # right arm down
        ]
        # Reach the pose smoothly in ~2 seconds
        p.time_from_start.sec = 2
        p.time_from_start.nanosec = 0

        jt.points = [p]
        return jt


    def send_follow_trajectory(self):
        # Ensure server exists (non-blocking quick check)
        if not self.follow_client.wait_for_server(timeout_sec=0.0):
            self.web_node.get_logger().warn("FollowJointTrajectory server not available yet")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_trajectory()
        self._follow_send_goal_future = self.follow_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback_cb
        )
        self._follow_send_goal_future.add_done_callback(self._follow_goal_response_cb)
        self.web_node.get_logger().info("FollowJointTrajectory goal sent")

    def _build_dual_arm_trajectory(self) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = [
            "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
            "left_elbow","left_wrist_roll","left_wrist_pitch","left_wrist_yaw",
            "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
            "right_elbow","right_wrist_roll","right_wrist_pitch","right_wrist_yaw"
        ]

        def pt(positions, sec, nsec=0):
            p = JointTrajectoryPoint()
            p.positions = positions
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int(nsec)
            return p

        jt.points = [
            pt([0.0, 0.2, 0.0, 1.57, 0.0, 0.0, 0.0,   0.0, -0.2, 0.0, 1.57, 0.0, 0.0, 0.0],  2, 0),
            pt([0.0, 1.57, 1.57,-1.57, 0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.57,0.0, 0.0, 0.0],  4),
            pt([0.0, 1.57, 1.57,-1.0,  0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.0, 0.0, 0.0, 0.0],  6),
            pt([0.0, 1.57, 1.57,-1.57, 0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.57,0.0, 0.0, 0.0],  8),
            pt([0.0, 1.57, 1.57,-1.0,  0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.0, 0.0, 0.0, 0.0], 10),
            pt([0.0, 1.57, 1.57,-1.57, 0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.57,0.0, 0.0, 0.0], 12),
            pt([0.0, 1.57, 1.57,-1.0,  0.0, 0.0, 0.0, 0.0,-1.57,-1.57,-1.0, 0.0, 0.0, 0.0], 14),
            pt([0.0, 0.2, 0.0, 1.57,   0.0, 0.0, 0.0, 0.0,-0.2, 0.0, 1.57,  0.0, 0.0, 0.0], 16, 0),
        ]
        return jt
    
    def send_follow_trajectory_cross(self):
        # Ensure server exists (non-blocking quick check)
        if not self.follow_client.wait_for_server(timeout_sec=0.0):
            self.web_node.get_logger().warn("FollowJointTrajectory server not available yet")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_cross_trajectory()

        self._follow_send_goal_future = self.follow_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback_cb
        )
        self._follow_send_goal_future.add_done_callback(self._follow_goal_response_cb)
        self.web_node.get_logger().info("FollowJointTrajectory CROSS-ARMS goal sent")


    def _build_dual_arm_cross_trajectory(self) -> JointTrajectory:
        """
        Cross both arms across the torso, derived from the provided CLI sequence.
        """
        jt = JointTrajectory()
        jt.joint_names = [
            "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
            "left_elbow","left_wrist_roll","left_wrist_pitch","left_wrist_yaw",
            "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
            "right_elbow","right_wrist_roll","right_wrist_pitch","right_wrist_yaw"
        ]

        def pt(positions, sec, nsec=0):
            p = JointTrajectoryPoint()
            p.positions = positions
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int(nsec)
            return p

        jt.points = [
            # Initial slight forward bend of both shoulders
            pt([
               -0.2,  0.2,  0.0, 0.7, 0.0, 0.0, 0.0,
               -0.2, -0.2,  0.0, 0.7, 0.0, 0.0, 0.0
            ], 2, 0),

            # Move left arm inward (yaw -1.57), prep right arm
            pt([
               -0.5,  0.2, -1.57, 0.6, 0.0, 0.0, 0.0,
               -0.2, -0.2,  0.0,  0.7, 0.0, 0.0, 0.0
            ], 4, 0),

            # Bring right arm across (yaw +1.57) to complete the cross
            pt([
               -0.5,  0.2, -1.57, 0.6, 0.0, 0.0, 0.0,
               -0.7, -0.2,  1.57, 0.6, 0.0, 0.0, 0.0
            ], 8, 0),
        ]
        return jt


    def send_follow_trajectory_uncross(self):
        # Ensure server exists (non-blocking quick check)
        if not self.follow_client.wait_for_server(timeout_sec=0.0):
            self.web_node.get_logger().warn("FollowJointTrajectory server not available yet")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_uncross_trajectory()

        self._follow_send_goal_future = self.follow_client.send_goal_async(
            goal, feedback_callback=self._follow_feedback_cb
        )
        self._follow_send_goal_future.add_done_callback(self._follow_goal_response_cb)
        self.web_node.get_logger().info("FollowJointTrajectory UN-CROSS goal sent")


    def _build_dual_arm_uncross_trajectory(self) -> JointTrajectory:
        """
        Return from crossed pose to the neutral 'down/rest' like pose,
        derived from the provided CLI sequence.
        """
        jt = JointTrajectory()
        jt.joint_names = [
            "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
            "left_elbow","left_wrist_roll","left_wrist_pitch","left_wrist_yaw",
            "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
            "right_elbow","right_wrist_roll","right_wrist_pitch","right_wrist_yaw"
        ]

        def pt(positions, sec, nsec=0):
            p = JointTrajectoryPoint()
            p.positions = positions
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int(nsec)
            return p

        jt.points = [
            # Start from slight forward shoulder pose
            pt([
               -0.2,  0.2,  0.0, 0.7, 0.0, 0.0, 0.0,
               -0.2, -0.2,  0.0, 0.7, 0.0, 0.0, 0.0
            ], 2, 0),

            # Open to your existing "down" posture variant
            pt([
                0.0,  0.2, 0.0, 1.57, 0.0, 0.0, 0.0,
                0.0, -0.2, 0.0, 1.57, 0.0, 0.0, 0.0
            ], 4, 0),
        ]
        return jt


    def _follow_feedback_cb(self, feedback_msg: FollowJointTrajectory.Feedback):
        self._follow_last_feedback = feedback_msg
        self.web_node.get_logger().info("FollowJT feedback received")

    def _follow_goal_response_cb(self, future):
        try:
            self._follow_goal_handle = future.result()
        except Exception as e:
            self.web_node.get_logger().error(f"FollowJT goal response error: {e}")
            self._follow_goal_handle = None
            return

        if not self._follow_goal_handle.accepted:
            self.web_node.get_logger().warn("FollowJT goal rejected by server")
            self._follow_goal_handle = None
            return

        self.web_node.get_logger().info("FollowJT goal accepted")
        self._follow_result_future = self._follow_goal_handle.get_result_async()

    def _spin_follow_futures(self):
        if self._follow_result_future and self._follow_result_future.done():
            try:
                result = self._follow_result_future.result().result
                status = self._follow_result_future.result().status
                err = getattr(result, "error_code", 0)
                estr = getattr(result, "error_string", "")
                self.web_node.get_logger().info(
                    f"FollowJT finished with status={status}, error_code={err}, msg='{estr}'"
                )
            except Exception as e:
                self.web_node.get_logger().error(f"FollowJT result retrieval error: {e}")
            finally:
                self._follow_result_future = None
                self._follow_goal_handle = None

    def cancel_follow_trajectory(self):
        if self._follow_goal_handle:
            self.web_node.get_logger().info("Cancelling FollowJT goal...")
            self._follow_goal_handle.cancel_goal_async()
