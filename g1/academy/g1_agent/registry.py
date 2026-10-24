"""Live skill registry on sdk_wrapper.G1 (replaces
agent.skills.build_live_registry, which binds skills to sdk_client.Robot).

The skill names, arguments and descriptions match the original so the
planner, capability resolver and per-skill confirm settings apply unchanged:
move / reach_forward / grab / release, step_forward/back, turn_left/right,
the SLAM skills, announce, the high-level gestures, release_arms, the FSM
modes, the expressive motions, and the agent's own request_sleep /
request_charge.  Skills that move the robot first ask the host
(AgentHost.conflict) and refuse while another controller owns the robot.
"""
from __future__ import annotations

import json
import math
from typing import Any, Callable, Dict, Tuple

from agent.expressive_motion import ExpressiveMotionController
from agent.skills import Skill, SkillRegistry, SkillResult, _new_skills

from .robot_port import AgentHost, AgentRobotPort
from .sensors import SensorWatch
from .slam import G1SlamBackend
from .tools import build_robot_tools
from .vision_grab import G1VisionGrabber

GESTURES = {"wave": "face wave", "face_wave": "face wave", "high_wave": "high wave", "clap": "clap",
            "left_kiss": "left kiss", "shake_hand": "shake hand"}


def _code(result: Any) -> int:
    if isinstance(result, tuple):
        result = result[0]
    if isinstance(result, dict):
        return int(result.get("code", 0) or 0)
    return int(result or 0)


def build_g1_registry(robot: Any, host: AgentHost, sensors: SensorWatch, *, iface: str = "eth0",
                      domain_id: int = 0) -> Tuple[SkillRegistry, G1SlamBackend]:
    import sdk_wrapper

    skills: Dict[str, Skill] = {}
    port = AgentRobotPort(robot, sensors, host)

    def add(name: str, description: str, handler: Callable[..., SkillResult], source: str, *, moves: bool = False) -> None:
        if moves:
            inner = handler

            def handler(**kwargs: Any) -> SkillResult:  # noqa: F811 - guarded wrapper
                conflict = host.conflict()
                if conflict:
                    return SkillResult(ok=False, message=f"{name} not started: {conflict}", detail={"skill": name, "blocked": True})
                return inner(**kwargs)
        skills[name] = Skill(name=name, description=description, handler=handler, source=source)

    def result(ok: bool, message: str, backend: str, skill: str, **detail: Any) -> SkillResult:
        return SkillResult(ok=ok, message=message, detail={"backend": backend, "skill": skill, **detail})

    # -- basic tools (llm_client.robot_tools) -------------------------------------
    def extend_arm(side: str, _duration_s: float) -> Dict[str, Any]:
        import grasp_pipeline as gp

        pipeline = host.grasp_pipeline()
        if not pipeline.arm_lock.acquire(timeout=5.0):
            return {"success": False, "reason": "the grab pipeline is busy with another arm motion"}
        try:
            pipeline._busy("agent reach")
            pipeline.stop_active_hold()
            out = pipeline.run_extend_arm_ee_ik(side, gp.DEFAULT_MAX_JOINT_STEP_RAD, gp.DEFAULT_MAX_JOINT_SPEED_RAD_S,
                                                dict(pipeline.camera_extrinsic))
            if out.get("success"):
                pipeline.save_extend_arm_reference(side, out["final_q"], "agent reach_forward")
                pipeline.hold_current_arm_pose(side)
            return out
        finally:
            pipeline._idle()
            pipeline.arm_lock.release()

    tools = build_robot_tools(robot, host, extend_arm)
    for tool_name, fn in tools.items():
        def tool_handler(fn: Callable[..., str] = fn, tool_name: str = tool_name, **kwargs: Any) -> SkillResult:
            message = str(fn(**kwargs))
            return result(not message.startswith("error:"), message, "g1_agent.tools", tool_name)
        add(tool_name, f"robot tool '{tool_name}' on sdk_wrapper.G1 (see g1_agent/tools.py).", tool_handler,
            "g1_agent.tools.build_robot_tools", moves=tool_name in ("reach_forward", "grab"))

    # -- prompt-based vision grab (replaces the 'grab' tool, as upstream) --------
    grabber = G1VisionGrabber(robot=robot, host=host, iface=iface, domain_id=domain_id)

    def vision_grab(**kwargs: Any) -> SkillResult:
        settings = kwargs.get("agent_settings")
        if settings is None or not hasattr(settings, "vision"):
            return SkillResult(ok=False, message="vision settings were not provided to grab skill")
        prompt = str(kwargs.get("prompt") or kwargs.get("object") or kwargs.get("target") or "object")
        arm = str(kwargs.get("arm") or "auto")
        message = grabber.grab(settings=settings.vision, prompt=prompt, arm=arm)
        return result(True, message, "g1_agent.vision_grab.G1VisionGrabber", "grab", prompt=prompt, arm=arm)

    add("grab", "Prompt-based RGB-D grab: localize an object with OpenAI vision, then move the end effector "
        "toward it with IK and close the hand.", vision_grab, "g1_agent.vision_grab.G1VisionGrabber", moves=True)

    # -- steps and turns ------------------------------------------------------------
    def step(direction: str) -> SkillResult:
        vx = 0.25 if direction == "forward" else -0.25
        robot.loco_move(vx, 0.0, 0.0, duration_s=1.0)
        return result(True, f"stepped {direction} for 1.0s at vx={vx:+.2f}m/s", "sdk_wrapper.G1.loco_move",
                      f"step_{direction}", vx=vx)

    def turn(direction: str, degrees: Any = None) -> SkillResult:
        vyaw = 0.5 if direction == "left" else -0.5
        try:
            angle_deg = float(degrees) if degrees is not None else 28.6
        except (TypeError, ValueError):
            angle_deg = 28.6
        angle_deg = max(5.0, min(180.0, abs(angle_deg)))
        duration_s = math.radians(angle_deg) / abs(vyaw)
        robot.loco_move(0.0, 0.0, vyaw, duration_s=duration_s)
        return result(True, f"turned {direction} about {angle_deg:.0f}deg for {duration_s:.1f}s at vyaw={vyaw:+.2f}rad/s",
                      "sdk_wrapper.G1.loco_move", f"turn_{direction}", vyaw=vyaw, degrees=angle_deg, duration_s=duration_s)

    add("step_forward", "Step forward a short distance.", lambda **_k: step("forward"), "sdk_wrapper.G1.loco_move", moves=True)
    add("step_back", "Step backward a short distance.", lambda **_k: step("backward"), "sdk_wrapper.G1.loco_move", moves=True)
    add("turn_left", "Turn left in place by a requested angle.", lambda **k: turn("left", k.get("degrees")),
        "sdk_wrapper.G1.loco_move", moves=True)
    add("turn_right", "Turn right in place by a requested angle.", lambda **k: turn("right", k.get("degrees")),
        "sdk_wrapper.G1.loco_move", moves=True)

    # -- SLAM ---------------------------------------------------------------------
    slam = G1SlamBackend(robot, sensors)
    src = "g1_agent.slam.G1SlamBackend"
    ok_msg = lambda message: SkillResult(ok=" failed" not in message.split(":", 1)[0], message=message)  # noqa: E731
    add("start_mapping", "Start Unitree SLAM mapping.", lambda **k: ok_msg(slam.start_mapping(str(k.get("slam_type") or "indoor"))), src)
    add("save_map", "Save the current SLAM map.", lambda **k: ok_msg(slam.save_map(k.get("path"))), src)
    add("start_relocation", "Start SLAM relocation/localization on the saved map.",
        lambda **k: ok_msg(slam.start_relocation(k.get("path"))), src)
    add("add_current_nav_pose", "Add current SLAM pose to the navigation task queue.", lambda **_k: ok_msg(slam.add_current_pose()), src)
    add("navigate_selected_pose", "Navigate to the currently selected SLAM pose.",
        lambda **_k: ok_msg(slam.go_to_selected_pose()), src, moves=True)
    add("execute_nav_tasks", "Execute queued SLAM navigation tasks.", lambda **_k: ok_msg(slam.execute_tasks()), src, moves=True)
    add("pause_navigation", "Pause SLAM navigation.", lambda **_k: ok_msg(slam.pause()), src)
    add("resume_navigation", "Resume SLAM navigation.", lambda **_k: ok_msg(slam.resume()), src, moves=True)
    add("stop_slam", "Stop Unitree SLAM.", lambda **_k: ok_msg(slam.stop_slam()), src)
    add("slam_status", "Summarize SLAM state and fresh map topics.", lambda **_k: SkillResult(ok=True, message=slam.status()), src)
    add("slam_preflight", "Check SLAM service and lidar/IMU prerequisites for mapping.",
        lambda **_k: SkillResult(ok=True, message=json.dumps(slam.preflight(), ensure_ascii=False, sort_keys=True)), src)
    add("save_current_nav_point", "Save the current SLAM pose as a named navigation point.",
        lambda **k: SkillResult(ok=True, message=slam.save_current_point(str(k.get("name") or ""))), src)
    add("list_nav_points", "List saved named SLAM navigation points.", lambda **_k: SkillResult(ok=True, message=slam.list_points()), src)
    add("clear_nav_points", "Clear saved named SLAM navigation points.", lambda **_k: SkillResult(ok=True, message=slam.clear_points()), src)
    add("navigate_named_point", "Navigate to a saved named SLAM point, relocating first if needed.",
        lambda **k: ok_msg(slam.go_to_point(str(k.get("name") or ""), auto_relocate=bool(k.get("auto_relocate", True)))),
        src, moves=True)

    # -- speech -------------------------------------------------------------------
    def announce(**kwargs: Any) -> SkillResult:
        text = str(kwargs.get("text", "")).strip()
        if not text:
            return SkillResult(ok=False, message="no announcement text provided")
        requested = str(kwargs.get("language") or "en").strip().lower().replace("-", "_")
        language = requested if requested in sdk_wrapper._PIPER_VOICES else "en"
        code = robot.say(text, language=language, volume=int(kwargs.get("volume") or 100))
        return result(code == 0, f"spoke through sdk_wrapper.G1.say() -> code {code}", "sdk_wrapper.G1.say", "announce",
                      code=code, language=language)

    add("announce", "Speak text through the robot's speaker (Piper TTS via sdk_wrapper.G1.say()).", announce, "sdk_wrapper.G1.say")

    # -- gestures and arm authority -------------------------------------------------
    def arm_action(action_name: str) -> SkillResult:
        code = int(robot.execute_arm_action(action_name))
        return result(code == 0, f"executed high-level arm action {action_name!r} -> code {code}",
                      "sdk_wrapper.G1.execute_arm_action", action_name, code=code)

    def gesture(**kwargs: Any) -> SkillResult:
        name = str(kwargs.get("name", "face wave")).strip()
        if not name:
            return SkillResult(ok=False, message="no gesture name provided")
        if sdk_wrapper._normalize_action(name) not in sdk_wrapper.HL_ARM_ACTIONS:
            return SkillResult(ok=False, message=f"unknown gesture {name!r}; known: {', '.join(sorted(sdk_wrapper.HL_ARM_ACTIONS))}")
        return arm_action(name)

    src = "sdk_wrapper.G1.execute_arm_action"
    add("gesture", "Play a named high-level arm gesture.", gesture, src, moves=True)
    for skill_name, action in GESTURES.items():
        add(skill_name, f"Play the SDK '{action}' high-level arm action.", lambda action=action, **_k: arm_action(action), src, moves=True)

    def release_arms(**kwargs: Any) -> SkillResult:
        try:
            pipeline = host.grasp_pipeline()
            pipeline.stop_active_hold()  # a held pose would keep re-commanding the arm
            pipeline._authority_engaged = False
        except Exception:
            pass
        out = robot.release_arms(duration_s=float(kwargs.get("duration_s", 3.0)))
        return result(True, f"released arm control authority: {out}", "sdk_wrapper.G1.release_arms", "release_arms")

    add("release_arms", "Release arm control authority (arms go limp over the duration).", release_arms,
        "sdk_wrapper.G1.release_arms", moves=True)

    # -- FSM modes ------------------------------------------------------------------
    fsm = {
        "squat": (robot.squat_mode, "entered squat FSM mode"),
        "prepare": (robot.prepare_mode, "entered prepare FSM mode"),
        "damp": (robot.damp_mode, "entered damp FSM mode"),
        "zero_torque": (robot.zero_torque_mode, "entered zero-torque FSM mode"),
        "dev_mode": (lambda: robot.set_ai_sport(False), "entered developer mode (ai_sport off)"),
        "exit_dev_mode": (lambda: robot.set_ai_sport(True), "left developer mode by enabling ai_sport"),
        "walk_mode": (robot.walk_mode, "entered walk locomotion mode"),
        "run_mode": (robot.run_mode, "entered run locomotion mode"),
    }
    for skill_name, (method, message) in fsm.items():
        def run_fsm(method: Callable[[], Any] = method, message: str = message, skill_name: str = skill_name,
                    **_kwargs: Any) -> SkillResult:
            out = method()
            ok = bool(out.get("ok")) if isinstance(out, dict) and "ok" in out else _code(out) == 0
            return result(ok, f"{message}: {out}", "sdk_wrapper.G1", skill_name)
        add(skill_name, message + ".", run_fsm, "sdk_wrapper.G1")

    # -- expressive motions -----------------------------------------------------------
    expressive = ExpressiveMotionController(robot=port)
    for kind in ("thinking", "explain", "thanking"):
        def run_expressive(kind: str = kind, **kwargs: Any) -> SkillResult:
            settings = kwargs.get("agent_settings")
            if settings is None:
                return SkillResult(ok=False, message="agent_settings is required for expressive motion")
            return expressive.run(kind, settings=settings, reason=str(kwargs.get("reason") or "skill"))
        add(f"{kind}_motion", f"Play a random {kind}_*.csv expressive motion through sdk_wrapper.G1.repeat_motion_file().",
            run_expressive, "agent.expressive_motion on sdk_wrapper.G1", moves=True)

    for name, skill in _new_skills().items():  # request_sleep / request_charge: robot-independent
        skills[name] = skill
    sources = sorted({skill.source for skill in skills.values()})
    return SkillRegistry(skills, backend_label="live (" + ", ".join(sources) + ")"), slam
