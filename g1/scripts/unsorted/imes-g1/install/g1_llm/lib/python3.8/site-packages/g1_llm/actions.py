#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class G1ArmActions:
    """
    Minimal, self-contained wrapper around FollowJointTrajectory for the dual arms.
    Exposes: send_dual_wave(), send_down(), cancel().
    """

    def __init__(self, node, action_name='dual_arm/follow_joint_trajectory'):
        self.node = node
        self.client = ActionClient(node, FollowJointTrajectory, action_name)

        self._goal_handle = None
        self._send_goal_future = None
        self._result_future = None
        self._last_feedback = None

        # Poll the futures without blocking the executor
        self.node.create_timer(0.2, self._spin_futures)

        self.node.get_logger().info(self.node.colorize(
            f"G1ArmActions ready → action: {action_name}", "green"))

    # ------------------------------- Public API

    def send_dual_wave(self):
        if not self.client.wait_for_server(timeout_sec=0.0):
            self.node.get_logger().warn("FollowJT server not available yet")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_wave_trajectory()
        self._send(goal, tag="WAVE")

    def send_down(self):
        if not self.client.wait_for_server(timeout_sec=0.0):
            self.node.get_logger().warn("FollowJT server not available yet")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_dual_arm_down_trajectory()
        self._send(goal, tag="DOWN")

    def cancel(self):
        if self._goal_handle:
            self.node.get_logger().info("Cancelling FollowJT goal…")
            self._goal_handle.cancel_goal_async()

    # ------------------------------- Internals

    def _send(self, goal, tag=""):
        self._send_goal_future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        self._send_goal_future.add_done_callback(self._goal_response_cb)
        self.node.get_logger().info(f"FollowJT {tag} goal sent")

    def _feedback_cb(self, feedback: FollowJointTrajectory.Feedback):
        self._last_feedback = feedback

    def _goal_response_cb(self, future):
        try:
            self._goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f"FollowJT goal response error: {e}")
            self._goal_handle = None
            return

        if not self._goal_handle.accepted:
            self.node.get_logger().warn("FollowJT goal rejected by server")
            self._goal_handle = None
            return

        self.node.get_logger().info("FollowJT goal accepted")
        self._result_future = self._goal_handle.get_result_async()

    def _spin_futures(self):
        if self._result_future and self._result_future.done():
            try:
                r = self._result_future.result()
                status = getattr(r, "status", None)
                result = getattr(r, "result", None)
                err = getattr(result, "error_code", 0) if result else 0
                estr = getattr(result, "error_string", "") if result else ""
                self.node.get_logger().info(
                    f"FollowJT finished successfully with status={status}, error_code={err}, msg='{estr}'"
                )
            except Exception as e:
                self.node.get_logger().error(f"FollowJT result retrieval error: {e}")
            finally:
                self._result_future = None
                self._goal_handle = None

    # ------------------------------- Trajectories (based on your snippets)

    def _build_dual_arm_down_trajectory(self) -> JointTrajectory:
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
        p.time_from_start.sec = 2
        p.time_from_start.nanosec = 0
        jt.points = [p]
        return jt

    def _build_dual_arm_wave_trajectory(self) -> JointTrajectory:
        """
        Your original 'send_follow_trajectory' points (kept intact).
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
