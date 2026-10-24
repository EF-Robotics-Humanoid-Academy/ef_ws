"""Prompt-based RGB-D grab on sdk_wrapper.G1.

agent.vision_grab.OpenAIVisionGrabber localizes the object with OpenAI vision
(SDK-free, reused unchanged) and then drives hand_pose_navigation's
DirectHandPoseNav, which runs on sdk_client.Robot.  G1VisionGrabber replaces
only that IK part with grasp_pipeline.HandApproach, keeping the original
geometry, gains, timeouts and messages.  Differences:
  - the arm is prepositioned with the grab pipeline's table-aware extend
    recipe instead of sdk_client.Robot.extend_arm_forward();
  - after the approach converges the measured pose is held (no arm_sdk gap)
    while the hand closes, as the /sort grabs do; the hold ends when another
    controller takes the arms.
"""
from __future__ import annotations

import time
from typing import Any, Dict

from agent.vision_grab import (
    DEFAULT_CAMERA_EXTRINSIC, OpenAIVisionGrabber, VisionGrabError, _box_depth_center, _choose_reachable_standoff,
    _compact_status, _plausible_object_base_point, _stable_vision_object_pose,
)

from .robot_port import AgentHost

APPROACH_CONFIG = {
    "ik_solver": "dls", "rate_hz": 8.0, "timeout_s": 18.0, "ik_tol_pos_m": 0.06, "ik_tol_rot_rad": 3.14,
    "convergence_pos_m": 0.06, "convergence_rot_rad": 3.14, "max_joint_step_rad": 0.06,
    "max_joint_speed_rad_s": 0.35, "max_reach_m": 0.52,
}


class G1VisionGrabber(OpenAIVisionGrabber):
    def __init__(self, *, robot: Any, host: AgentHost, iface: str = "eth0", domain_id: int = 0) -> None:
        super().__init__(robot=robot, iface=iface, domain_id=domain_id)
        self.host = host

    def grab(self, *, settings: Any, prompt: str, arm: str = "auto") -> str:
        self.host.require("grab")
        return super().grab(settings=settings, prompt=prompt, arm=arm)

    def _run_ik_grab(self, *, frame: Dict[str, Any], label: str, confidence: float,
                     box_xyxy: tuple, arm_override: str) -> str:
        import numpy as np
        import grasp_pipeline as gp
        from hand_pose_navigation.grasp_planner import GraspPlanner
        from hand_pose_navigation.reachability_checker import ReachabilityChecker
        from hand_pose_navigation.target_detector import CameraIntrinsics

        depth_m = frame["depth_m"]
        u, v, z = _box_depth_center(depth_m, box_xyxy)
        if z <= 0.05:
            raise VisionGrabError(f"no valid depth for localized {label!r} at box={box_xyxy}")
        h, w = depth_m.shape[:2]
        intr = CameraIntrinsics(width=int(w), height=int(h), cx=float(w) / 2.0, cy=float(h) / 2.0)
        p_camera = intr.deproject(float(u), float(v), float(z))
        T_base_camera = gp.base_from_camera(DEFAULT_CAMERA_EXTRINSIC)
        p_base = (T_base_camera @ np.append(p_camera, 1.0))[:3]
        if not _plausible_object_base_point(p_base):
            raise VisionGrabError(
                "localized target has implausible base-frame position "
                f"camera_xyz=({p_camera[0]:+.3f},{p_camera[1]:+.3f},{p_camera[2]:+.3f})m "
                f"base_xyz=({p_base[0]:+.3f},{p_base[1]:+.3f},{p_base[2]:+.3f})m "
                f"box={box_xyxy}. Move the object into the reachable area or check camera extrinsics/depth alignment.")
        T_base_object = _stable_vision_object_pose(p_base)
        side = str(arm_override).strip().lower()
        if side not in {"left", "right"}:
            side = "left" if p_base[1] > 0.12 else "right"
        standoff_m, T_base_desired, reach_reason = _choose_reachable_standoff(
            T_base_object, side, GraspPlanner=GraspPlanner, ReachabilityChecker=ReachabilityChecker)
        if standoff_m is None:
            raise VisionGrabError(
                f"localized {label!r}, but no reachable wrist target was found: {reach_reason}. "
                f"object_base_xyz=({p_base[0]:+.3f},{p_base[1]:+.3f},{p_base[2]:+.3f})m box={box_xyxy}")

        pipeline = self.host.grasp_pipeline()
        if not pipeline.arm_lock.acquire(timeout=5.0):
            raise VisionGrabError("the grab pipeline is busy with another arm motion")
        nav = None
        try:
            pipeline._busy("agent grab")
            pipeline.stop_active_hold()
            extend = pipeline.run_extend_arm_ee_ik(side, APPROACH_CONFIG["max_joint_step_rad"],
                                                   APPROACH_CONFIG["max_joint_speed_rad_s"], dict(pipeline.camera_extrinsic))
            preposition_msg = (f"prepositioned {side} arm forward ({', '.join(extend.get('stages', []))});"
                               if extend.get("success") else f"preposition failed: {extend.get('reason')};")
            nav = gp.HandApproach(pipeline, T_base_desired, {"arm": side, **APPROACH_CONFIG}, None)
            ok = False
            last_status: Dict[str, Any] = {}
            deadline = time.time() + APPROACH_CONFIG["timeout_s"] + 2.0
            while time.time() < deadline:
                last_status = nav.status_snapshot()
                if last_status.get("converged"):
                    ok = True
                    break
                if not last_status.get("running", True):
                    break
                time.sleep(0.2)
            if ok:
                pipeline.hold_current_arm_pose(side)
            pipeline.shutdown_nav(nav, "agent grab")
            nav = None
            if not ok:
                raise VisionGrabError(
                    f"IK did not converge for {label!r}; "
                    f"object_base_xyz=({p_base[0]:+.3f},{p_base[1]:+.3f},{p_base[2]:+.3f})m "
                    f"desired_wrist_xyz=({T_base_desired[0, 3]:+.3f},{T_base_desired[1, 3]:+.3f},{T_base_desired[2, 3]:+.3f})m "
                    f"standoff={float(standoff_m):.3f}m preposition={preposition_msg!r} "
                    f"last_status={_compact_status(last_status)}")
            close = self.robot.close_dex3_hand(hand=side, hold_s=0.6, ramp_s=0.25)
            close_msg = (f"Closed {side} hand." if not isinstance(close, dict) or close.get("ok", True)
                         else f"Hand close stopped: {close.get('reason') or close.get('error')}.")
        finally:
            if nav is not None:
                pipeline.shutdown_nav(nav, "agent grab")
            pipeline._idle()
            pipeline.arm_lock.release()
        return (f"localized {label!r} at pixel=({u},{v}) depth={z:.2f}m; "
                f"{preposition_msg} moved {side} end effector toward target with IK. {close_msg}")
