"""sdk_wrapper.G1 backend for the OpenAI intent agent (modules/scripts/openaiapi/agent).

The agent framework -- planner, router, memory, settings, capability
resolver -- is robot-independent and used unchanged.  Its live robot backend
is not: agent.cli.__main__.build_agent() instantiates sdk_client.Robot and
binds skills, state, SLAM and vision grab to it.  This package provides that
backend on sdk_wrapper.G1 instead, without importing anything from modules/
that needs sdk_client:

  sensors.py      topic freshness            (Robot.sensors_stale & co.)
  state.py        G1RobotStateSource         (agent.state.SdkClientRobotStateSource)
  tools.py        move/reach/grab/release    (llm_client.robot_tools)
  slam.py         G1SlamBackend              (agent.slam.SlamBackend + slam_web_app)
  vision_grab.py  G1VisionGrabber            (agent.vision_grab's DirectHandPoseNav part)
  robot_port.py   AgentHost, AgentRobotPort  (host hooks; robot hooks the framework probes)
  registry.py     build_g1_registry          (agent.skills.build_live_registry)
  knowledge.py    DocumentRAG                (agent.knowledge.document_rag -> nav_bot -> sdk_slam)
  manipulation.py Manipulator                chat arm/hand commands on the /sort V2 pipeline
  chat.py         ChatAgent                  the dashboard V2 chat (replaces the planner's turn)

Usage (the dashboard does this)::

    robot = sdk_wrapper.G1(iface, domain_id)
    host = AgentHost(robot, rgbd_endpoint)
    agent = build_agent(args, robot, host)
    host.bind(arm_conflict=..., grasp_pipeline=...)   # once the host app exists
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

from .robot_port import AgentHost, AgentRobotPort, MotionBlocked

__all__ = ["AgentHost", "AgentRobotPort", "MotionBlocked", "build_agent", "build_chat_agent", "chat_knowledge"]

_ACADEMY = Path(__file__).resolve().parents[1]
CHAT_KNOWLEDGE_FILES = (
    _ACADEMY.parent / "modules" / "scripts" / "ollama_ai" / "g1_faq_knowledge.json",
    _ACADEMY.parent / "modules" / "scripts" / "ollama_ai" / "ef_robotics_faq_knowledge.json",
    Path(__file__).resolve().parent / "knowledge_data" / "unitree_sdk_faq.json",
    Path(__file__).resolve().parent / "knowledge_data" / "academy_robot_faq.json",
)


def chat_knowledge(log: Any = print) -> Any:
    from .knowledge import ChatKnowledge

    return ChatKnowledge(list(CHAT_KNOWLEDGE_FILES), on_error=log)


def build_chat_agent(inner: Any, robot: Any, *, sorter: Any, built_sorter: Any, conflict: Any, notify: Any) -> Any:
    """Compose the ChatAgent around a built G1Agent: its skills, capability
    resolver, settings, state source and language model endpoint are reused."""
    from agent.settings.models import SkillMode
    from agent.state import build_robot_state

    from .chat import ChatAgent, OpenAIChat, Tool, build_tools
    from .manipulation import Manipulator

    manipulator = Manipulator(sorter, built_sorter, conflict, notify)

    def call_skill(name: str, **kwargs: Any) -> Any:
        result = inner.skills.invoke(name, **kwargs)
        return bool(result.ok), str(result.message)

    def status() -> str:
        state = inner.state_source.read()
        parts = [f"posture {state.posture}", f"stability {state.stability}"]
        if state.battery_pct is not None:
            parts.insert(0, f"battery {state.battery_pct:.0f}%")
        if state.active_faults:
            parts.append("faults: " + ", ".join(state.active_faults[:4]))
        holding = manipulator.holding()
        if holding:
            parts.append(f"right hand holding {holding}")
        return ", ".join(parts)

    def policy(tool: Tool, args: dict) -> Any:
        """Tier default, a per-tool settings override (settings.skills), and the capability check."""
        if tool.tier == "read":
            return True, False, ""
        settings = inner.settings.effective()
        override = settings.skills.overrides.get(tool.name)
        if override == SkillMode.DISABLED:
            return False, False, f"{tool.name} is disabled in the settings"
        confirm = override == SkillMode.CONFIRM or (override is None and tool.tier == "confirm")
        if tool.gate_skill is not None:
            if manipulator.busy():  # queued behind our own command: it owns the arm, not a conflict
                if not settings.motion.allow_arm_motion:
                    return False, False, "arm motion is disabled in the settings"
            else:
                decision = inner.resolver.resolve_skill(tool.gate_skill, settings=settings,
                                                        robot_state=build_robot_state(inner.state_source))
                if not decision.allowed:
                    return False, False, decision.reason
                confirm = confirm or decision.requires_approval
        # Opening a hand with an object in it drops the object.
        if tool.name == "set_hand" and args.get("action") == "open" and str(args.get("hand", "right")) == "right" \
                and manipulator.holding():
            confirm = True
        return True, confirm, ""

    def on_stop() -> None:
        robot.loco_stop()
        if inner.navigation.slam_backend.slam_running:
            robot.pause_nav()

    tools = build_tools(manipulator, call_skill, status, describe_view=inner._answer_vision_question)
    planner = inner.planner
    llm = OpenAIChat(planner._model, planner._base) if hasattr(planner, "_model") else None
    return ChatAgent(tools, chat_knowledge(), llm=llm, inner=inner, manipulator=manipulator, policy=policy, on_stop=on_stop,
                     log=lambda line: print(f"[chat] {line}"))


def build_agent(args: Any, robot: Any = None, host: Optional[AgentHost] = None) -> Any:
    """Compose a G1Agent.  ``args`` needs offline, iface, domain_id, model
    (and rgbd for a private grab pipeline).  Live mode requires the G1
    ``robot`` and the OpenAI planner, like g1_cli's agent; offline mode uses
    the agent's offline registry, mock state and mock planner."""
    import agent as agent_package
    from agent.capabilities import CapabilityResolver
    from agent.cli.router import G1Agent
    from agent.knowledge.sdk_wrapper_knowledge import SdkWrapperKnowledge
    from agent.memory.manager import MemoryManager
    from agent.models import LifecycleState
    from agent.planner import MockPlanner, OpenAIPlanner, PlannerError
    from agent.settings.manager import SettingsManager
    from agent.skills import build_offline_registry
    from agent.state import MockRobotStateSource

    live = not bool(getattr(args, "offline", False))
    document_rag = None
    knowledge_file = Path(agent_package.__file__).resolve().parent / "knowledge" / "default_sdk_knowledge.md"
    if knowledge_file.exists():
        try:
            from .knowledge import DocumentRAG

            document_rag = DocumentRAG([str(knowledge_file)])
        except Exception as exc:
            print(f"[warn] documentary RAG unavailable: {exc}")

    agent_robot = None
    slam = None
    if live:
        if robot is None:
            raise RuntimeError("live agent needs the sdk_wrapper.G1 robot")
        from .registry import build_g1_registry
        from .sensors import SensorWatch
        from .state import G1RobotStateSource

        host = host or AgentHost(robot, str(getattr(args, "rgbd", "tcp://127.0.0.1:5555")))
        sensors = SensorWatch(robot)
        skills, slam = build_g1_registry(robot, host, sensors, iface=str(args.iface), domain_id=int(args.domain_id))
        state_source: Any = G1RobotStateSource(robot, sensors)
        # Arm motion is "available" unless another controller owns the arms.
        resolver = CapabilityResolver(arm_sdk_available=lambda: host.conflict() is None, low_cmd_available=lambda: False)
        agent_robot = AgentRobotPort(robot, sensors, host)
        try:
            planner: Any = OpenAIPlanner(model=args.model)
        except PlannerError as exc:
            raise RuntimeError(f"OpenAI planner is unavailable ({exc}); set OPENAI_API_KEY and install the "
                               "openaiapi dependencies.") from exc
    else:
        skills = build_offline_registry()
        state_source = MockRobotStateSource()
        resolver = CapabilityResolver()
        planner = MockPlanner()

    agent = G1Agent(planner=planner, skills=skills, state_source=state_source, settings=SettingsManager(),
                    memory=MemoryManager(), sdk_knowledge=SdkWrapperKnowledge(), document_rag=document_rag,
                    resolver=resolver, auto_confirm=False, robot=agent_robot)
    if slam is not None:
        agent.navigation.slam_backend = slam
    if live:
        # Keep each request to a few sequential model/tool rounds; every extra
        # planner round adds a full API latency (as g1_cli._make_agent).
        agent.settings.set("tools.max_calls_per_turn", 4, persist=False)
        if hasattr(agent.planner, "_max_retries"):
            agent.planner._max_retries = 0
    # Skip the boot-time announcement round trip; keep lifecycle semantics.
    if agent.lifecycle.state != LifecycleState.AWAKE:
        agent.lifecycle.transition(LifecycleState.AWAKE)
    agent._booted = True
    return agent
