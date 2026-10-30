"""The dashboard chat: short natural replies, knowledge, and robot tools.

G1Agent's planner answers with a PlannerDecision JSON built from a large state
dump, which gave three symptoms: long replies (the dashboard shows and speaks
the raw response_text), no reply to small talk (intent no_action or a null
response_text ends as "Request processed."), and errors when the tool rounds
ran out.  ChatAgent replaces that turn with a plain tool-calling conversation:

  * a short system prompt plus a hard length limit on every reply;
  * a reply is guaranteed: the last round may not call tools, and a missing or
    failed model answer falls back to a local reply;
  * matching knowledge facts are added to the turn, for the model to work in
    naturally; small talk gets none;
  * recent turns are remembered, so "and how much does it weigh?" works;
  * motion goes through the tools in build_tools (see manipulation.py), each
    gated by settings, capability state and operator confirmation.

Everything else the dashboard reaches on its agent (settings, ...) is passed on
to the wrapped G1Agent.  Without a language model (off-robot, or no connection)
the same tools are reached through parse_commands.
"""
from __future__ import annotations

import json
import os
import re
from collections import deque
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

from .manipulation import DEFAULT_HAND_STEP_M, DIRECTIONS, MAX_HAND_STEP_M, Manipulator

MAX_ROUNDS = 4
MAX_HISTORY_MESSAGES = 12
SHORT_LIMIT_CHARS = 320
DETAIL_LIMIT_CHARS = 900
SHORT_MAX_TOKENS = 200
DETAIL_MAX_TOKENS = 500
TOOL_LINE_CHARS = 180
MAX_WALK_M = 2.0

SYSTEM_PROMPT = """\
You are G1, a Unitree G1 EDU humanoid robot at the EF Robotics Academy, talking with the person operating you. \
Your words are shown in a chat and spoken aloud.

How you talk:
- Warm, natural and brief: one or two short sentences, about 30 words. Only go longer, up to five sentences, when \
asked to explain or list something.
- Plain sentences: no markdown, bullet points, headings or JSON.
- Always answer. Greetings and small talk get a friendly reply, without tools.

What you know:
- When "Things you know" appears below, use what is relevant and leave out the rest. Say it in your own words as \
something you simply know; never mention notes, documents, a database or sources.
- Do not invent specifications or company facts. If you don't know, say so briefly.

What you do:
- A request to do something physical means calling the tool right away, without asking first. The system asks the \
operator to confirm when needed. Several steps mean several calls, in order.
- Only say something is done when a tool result says it finished. A result saying "started" or "queued" means it is \
under way, so say you're on it. The outcome is announced separately.
- If a tool fails or is declined, say why in a few words and offer the next step. Never pretend it worked.
- "Right" and "left" are your own right and left. A hand direction without a hand means the right hand.
- You grab with your right arm only. Say so if asked to grab with the left."""

STOP_RE = re.compile(r"^\s*(please\s+)?(stop|halt|freeze|abort|cancel|stopp|anhalten|hör auf|hoer auf|emergency stop)"
                     r"(\s+(it|now|moving|that|everything|please|right now|the robot))*\s*[.!]*\s*$", re.IGNORECASE)
DETAIL_RE = re.compile(r"\b(explain|detail|details|detailed|step by step|list|describe|compare|how do i|how can i|why|"
                       r"erklär\w*|erklaer\w*|ausführlich|ausfuehrlich|wie kann ich|warum)\b", re.IGNORECASE)
FOLLOW_UP_RE = re.compile(r"\b(it|its|that|this|they|them|those|there|he|she|es|das|dort)\b", re.IGNORECASE)
CAPABILITY_RE = re.compile(r"\b(what can you do|what are you able|your (abilities|capabilities|skills)|was kannst du)\b",
                           re.IGNORECASE)


def is_stop_command(text: str) -> bool:
    return bool(STOP_RE.match(text or ""))


def shorten(text: str, limit: int) -> str:
    """Plain, spoken text within the limit, ending on a sentence where possible."""
    text = re.sub(r"[*#`]+", "", str(text or ""))
    text = re.sub(r"^\s*(?:[-•]|\d+[.)])\s+", "", text, flags=re.MULTILINE)
    text = " ".join(text.split())
    if len(text) <= limit:
        return text
    head = text[:limit]
    end = max(head.rfind(". "), head.rfind("! "), head.rfind("? "))
    if head[-1] in ".!?":
        end = len(head) - 1
    if end >= limit * 0.4:
        return head[:end + 1]
    return head[:head.rfind(" ")].rstrip(" ,;:") + "…"


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

@dataclass
class Tool:
    name: str
    description: str
    run: Callable[..., Tuple[bool, str]]
    parameters: Dict[str, Any] = field(default_factory=dict)
    required: Sequence[str] = ()
    # read: no motion.  safe: small or non-moving, runs directly.  confirm: asks the operator.
    tier: str = "confirm"
    # Capability check of agent.capabilities applied to this tool.
    gate_skill: Optional[str] = None
    # What the confirmation dialog says; may depend on the arguments.
    ask: Optional[Callable[..., str]] = None

    def schema(self) -> Dict[str, Any]:
        return {"type": "function", "function": {
            "name": self.name, "description": self.description,
            "parameters": {"type": "object", "properties": self.parameters, "required": list(self.required)}}}


def _text(description: str, *choices: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"type": "string", "description": description}
    if choices:
        out["enum"] = list(choices)
    return out


def build_tools(manipulator: Manipulator, call_skill: Callable[..., Tuple[bool, str]], status: Callable[[], str],
                describe_view: Optional[Callable[[str], str]] = None) -> Dict[str, Tool]:
    """``call_skill(name, **kwargs)`` runs one of the agent's existing skills
    (locomotion, SLAM, gestures) and returns (ok, message)."""
    m = manipulator

    def motion(label: str, skill: str, wait_s: float = 0.0, **kwargs: Any) -> Tuple[bool, str]:
        return m.background(label, lambda: call_skill(skill, **kwargs), wait_s)

    def walk(direction: str, distance_m: float = 0.5) -> Tuple[bool, str]:
        distance = max(0.05, min(MAX_WALK_M, float(distance_m)))
        return motion(f"walking {direction} {distance:.1f} m", "move", 20.0, direction=str(direction).lower(),
                      distance_m=distance, speed_mps=0.3)

    def turn(direction: str, degrees: float = 30.0) -> Tuple[bool, str]:
        side = str(direction).lower()
        if side not in ("left", "right"):
            return False, "direction must be left or right"
        return motion(f"turning {side}", f"turn_{side}", 20.0, degrees=float(degrees))

    modes = {"stand_up": "prepare", "balance": "walk_mode", "run": "run_mode", "squat": "squat", "damp": "damp",
             "zero_torque": "zero_torque"}

    def set_mode(mode: str) -> Tuple[bool, str]:
        skill = modes.get(str(mode).lower())
        return (False, f"mode must be one of {', '.join(modes)}") if skill is None else call_skill(skill)

    def gesture(name: str) -> Tuple[bool, str]:
        return m.free_arms_then(f"gesture {name}", lambda: call_skill("gesture", name=str(name)))

    def look() -> Tuple[bool, str]:
        return True, m.scene()

    def hand_ask(hand: str = "right", action: str = "open", **_: Any) -> str:
        return f"{action.capitalize()} the {hand} hand."

    hand_side = _text("Which hand; 'right' if not said.", "left", "right")
    tools = [
        Tool("robot_status", "Battery, posture, faults, what the hand holds and SLAM state.",
             lambda: (True, status()), tier="read"),
        Tool("look_at_table", "What the camera sees now: known objects with positions, and which spots are in view.",
             look, tier="read"),
        Tool("grab_object", "Find a table object with the camera and pick it up with the right arm, then keep holding it. "
             "For requests like 'grab the cup'.", lambda object: m.grab(object),
             {"object": _text("The object as the operator named it, e.g. 'cup'.")}, ["object"], gate_skill="grab",
             ask=lambda object="", **_: f"Grab the {object} with the right arm. Keep the workspace clear."),
        Tool("place_object", "Put the held object on a spot, open the hand and take the arm back. For 'place it on spot A'.",
             lambda spot: m.place(spot), {"spot": _text("The spot.", "Spot A", "Spot B")}, ["spot"], gate_skill="grab",
             ask=lambda spot="", **_: f"Carry the held object to {spot} and let go of it."),
        Tool("move_hand", "Move one hand a small step in a straight line, in the robot's own directions.",
             lambda direction, hand="right", distance_cm=DEFAULT_HAND_STEP_M * 100: m.move_hand(
                 hand, direction, float(distance_cm) / 100.0),
             {"hand": hand_side, "direction": _text("Direction to move.", *DIRECTIONS),
              "distance_cm": {"type": "number", "description": f"Centimetres, default {DEFAULT_HAND_STEP_M * 100:.0f}, "
                              f"at most {MAX_HAND_STEP_M * 100:.0f}."}}, ["direction"], tier="safe", gate_skill="reach_forward"),
        Tool("set_hand", "Open or close a hand.", lambda action, hand="right": m.hand(hand, action),
             {"hand": hand_side, "action": _text("Open or close.", "open", "close")}, ["action"], tier="safe",
             gate_skill="hand_open", ask=hand_ask),
        Tool("balanced_hold_pose", "Bring both arms back to the saved balanced hold pose and hold them there.",
             m.hold_pose, gate_skill="reach_forward", ask=lambda **_: "Move both arms to the balanced hold pose."),
        Tool("release_arms", "Give up arm control: the arms sink back to the robot's default controller.",
             m.release_arms, gate_skill="release_arms", ask=lambda **_: "Release arm control; the arms will sink down."),
        Tool("gesture", "Play a gesture.", gesture,
             {"name": _text("The gesture.", "face wave", "high wave", "clap", "shake hand", "high five", "hug", "heart",
                            "hands up", "left kiss", "right kiss", "reject")}, ["name"], gate_skill="gesture",
             ask=lambda name="", **_: f"Play the '{name}' gesture."),
        Tool("walk", f"Walk in a direction, at most {MAX_WALK_M:.0f} m.", walk,
             {"direction": _text("Direction.", "forward", "backward", "left", "right"),
              "distance_m": {"type": "number", "description": "Metres, default 0.5."}}, ["direction"],
             ask=lambda direction="", distance_m=0.5, **_: f"Walk {direction} {float(distance_m):.1f} m."),
        Tool("turn", "Turn in place.", turn,
             {"direction": _text("Direction.", "left", "right"), "degrees": {"type": "number", "description": "Default 30."}},
             ["direction"], ask=lambda direction="", degrees=30, **_: f"Turn {direction} {float(degrees):.0f} degrees."),
        Tool("set_body_mode", "Change the whole-body mode. damp and zero_torque drop the balance, only when clearly asked for.",
             set_mode, {"mode": _text("The mode.", *modes)}, ["mode"],
             ask=lambda mode="", **_: f"Switch to body mode '{mode}'." + (
                 " THE ROBOT LOSES ITS BALANCE: make sure it is supported." if mode in ("damp", "zero_torque") else "")),
        Tool("start_mapping", "Start SLAM mapping of the surroundings.", lambda: call_skill("start_mapping"), tier="safe"),
        Tool("save_map", "Stop mapping and save the map.", lambda: call_skill("save_map"), tier="safe"),
        Tool("stop_slam", "Stop SLAM without saving.", lambda: call_skill("stop_slam"), tier="safe"),
        Tool("start_relocation", "Localize on the saved map, needed before navigating.",
             lambda: call_skill("start_relocation"), tier="safe"),
        Tool("slam_status", "SLAM state, pose and lidar freshness.", lambda: call_skill("slam_status"), tier="read"),
        Tool("save_nav_point", "Save where the robot stands now under a name.",
             lambda name: call_skill("save_current_nav_point", name=name), {"name": _text("Name of the place.")}, ["name"],
             tier="safe"),
        Tool("list_nav_points", "List the saved places.", lambda: call_skill("list_nav_points"), tier="read"),
        Tool("go_to_nav_point", "Navigate to a saved place.",
             lambda name: motion(f"navigating to {name}", "navigate_named_point", name=name, auto_relocate=True),
             {"name": _text("Name of the place.")}, ["name"], ask=lambda name="", **_: f"Navigate to '{name}'."),
        Tool("pause_navigation", "Pause navigation.", lambda: call_skill("pause_navigation"), tier="safe"),
        Tool("resume_navigation", "Resume navigation.", lambda: call_skill("resume_navigation"),
             ask=lambda **_: "Resume navigation."),
    ]
    if describe_view is not None:
        tools.append(Tool("describe_view", "Answer a question about what the camera sees in general, e.g. 'what do you see?'.",
                          lambda question="What do you see?": (True, describe_view(question)),
                          {"question": _text("The question about the view.")}, tier="read"))
    return {tool.name: tool for tool in tools}


# ---------------------------------------------------------------------------
# Without a language model
# ---------------------------------------------------------------------------

_SMALL_TALK = [
    (r"\b(hello|hi|hey|hallo|good (morning|afternoon|evening)|guten (morgen|tag|abend)|servus|moin)\b",
     "Hello! I'm G1. What can I do for you?"),
    (r"\b(how are you|how's it going|wie geht)\b", "I'm doing well, thanks for asking. How can I help?"),
    (r"\b(thank|thanks|danke)\b", "You're welcome!"),
    (r"\b(bye|goodbye|see you|tschüss|tschuess|auf wiedersehen)\b", "Goodbye, see you soon!"),
    (r"\b(who are you|what are you|your name|wer bist du)\b", "I'm G1, a Unitree G1 humanoid robot at the EF Robotics Academy."),
]
_DIR = r"(forward|forwards|backward|backwards|back|up|down|left|right)"


def _distance_cm(clause: str) -> Optional[float]:
    match = re.search(r"(\d+(?:[.,]\d+)?)\s*(cm|centimet\w*|mm|m|meters?|metres?)\b", clause)
    if not match:
        return None
    value = float(match.group(1).replace(",", "."))
    unit = match.group(2)
    return value / 10.0 if unit == "mm" else value if unit.startswith("c") else value * 100.0


def _parse_clause(clause: str) -> Optional[Tuple[str, Dict[str, Any]]]:
    match = re.search(r"\b(place|put|drop|set|lay)\b.*\bspot\s*([ab])\b", clause) or re.search(r"\bspot\s*([ab])\b", clause)
    if match:
        return "place_object", {"spot": f"Spot {match.group(match.lastindex).upper()}"}
    if re.search(r"\brelease\b.*\barms?\b|\bfree\b.*\barms?\b", clause):
        return "release_arms", {}
    if re.search(r"\b(balanced?|stable|hold(ing)? (pose|position))\b", clause):
        return "balanced_hold_pose", {}
    match = re.search(r"\b(open|close)\b.*?\b(?:(left|right)\s+)?hand\b", clause)
    if match:
        return "set_hand", {"action": match.group(1), "hand": match.group(2) or "right"}
    match = re.search(r"\b(?:(left|right)\s+)?(?:hand|arm)\b.*?\b" + _DIR + r"\b", clause)
    if match:
        direction = {"forwards": "forward", "back": "backward", "backwards": "backward"}.get(match.group(2), match.group(2))
        args: Dict[str, Any] = {"hand": match.group(1) or "right", "direction": direction}
        distance = _distance_cm(clause)
        if distance is not None:
            args["distance_cm"] = distance
        return "move_hand", args
    if re.search(r"\b(start|begin)\b.*\b(mapping|map)\b|^mapping$", clause):
        return "start_mapping", {}
    if re.search(r"\b(stop|finish|end)\b.*\bmapping\b|\bsave\b.*\bmap\b", clause):
        return "save_map", {}
    if re.search(r"\b(stop|close)\b.*\bslam\b", clause):
        return "stop_slam", {}
    if re.search(r"\b(relocate|relocation|localize|localise)\b", clause):
        return "start_relocation", {}
    if re.search(r"\bslam\b.*\bstatus\b|\bstatus\b.*\bslam\b", clause):
        return "slam_status", {}
    if re.search(r"\b(list|show)\b.*\b(points|places|locations)\b", clause):
        return "list_nav_points", {}
    match = re.search(r"\bsave\b.*\b(?:point|place|location|position)\b(?:\s+as|\s+called|\s+named)?\s+(.+)$", clause)
    if match:
        return "save_nav_point", {"name": match.group(1).strip()}
    if re.search(r"\bpause\b.*\b(nav\w*)\b", clause):
        return "pause_navigation", {}
    if re.search(r"\bresume\b.*\b(nav\w*)\b", clause):
        return "resume_navigation", {}
    match = re.search(r"\b(?:go|navigate|walk)\s+to\s+(?:the\s+)?(.+)$", clause)
    if match:
        return "go_to_nav_point", {"name": match.group(1).strip()}
    match = re.search(r"\b(?:walk|step|go|move)\s+" + _DIR + r"\b", clause)
    if match and match.group(1) not in ("up", "down"):
        direction = {"forwards": "forward", "back": "backward", "backwards": "backward"}.get(match.group(1), match.group(1))
        distance = _distance_cm(clause)
        return "walk", {"direction": direction, "distance_m": 0.5 if distance is None else distance / 100.0}
    match = re.search(r"\bturn\s+(left|right)\b(?:.*?(\d+))?", clause)
    if match:
        return "turn", {"direction": match.group(1), "degrees": float(match.group(2) or 30)}
    for gesture_word, name in (("high wave", "high wave"), ("wave", "face wave"), ("clap", "clap"), ("shake", "shake hand"),
                               ("high five", "high five"), ("hug", "hug")):
        if re.search(rf"\b{gesture_word}\b", clause):
            return "gesture", {"name": name}
    if re.search(r"\b(battery|status|how much charge)\b", clause):
        return "robot_status", {}
    if re.search(r"\b(what do you see|what can you see|look at the table|on the table|in view)\b", clause):
        return "look_at_table", {}
    match = re.search(r"\b(?:grab|pick up|pick|take|fetch|get|greif\w*|nimm)\b\s+(?:me\s+)?(?:the\s+|a\s+|an\s+|my\s+)?(.+?)(?:\s+up)?$", clause)
    if match:
        return "grab_object", {"object": match.group(1).strip()}
    return None


def parse_commands(text: str) -> List[Tuple[str, Dict[str, Any]]]:
    """The robot commands in plain text, in order, without a language model."""
    lowered = re.sub(r"[!?.]+\s*$", "", (text or "").strip().lower())
    lowered = re.sub(r"^(please|can you|could you|would you|bitte)\s+", "", lowered)
    out: List[Tuple[str, Dict[str, Any]]] = []
    for clause in re.split(r",|;|\band then\b|\bthen\b|\band\b", lowered):
        clause = clause.strip()
        if clause:
            parsed = _parse_clause(clause)
            if parsed is not None:
                out.append(parsed)
    return out


class _LocalSettings:
    """What the dashboard writes to its agent's settings, when no G1Agent is wrapped."""

    def __init__(self) -> None:
        self.values: Dict[str, Any] = {}

    def set(self, key: str, value: Any, persist: bool = False) -> None:
        self.values[key] = value


# ---------------------------------------------------------------------------
# Language model
# ---------------------------------------------------------------------------

class OpenAIChat:
    """One chat completion over HTTP; the planner's endpoint and key."""

    def __init__(self, model: str, base: str = "https://api.openai.com/v1", api_key_env: str = "OPENAI_API_KEY",
                 timeout_s: float = 30.0) -> None:
        self.model = model
        self.base = base.rstrip("/")
        self.api_key_env = api_key_env
        self.timeout_s = timeout_s

    def complete(self, messages: List[Dict[str, Any]], tools: List[Dict[str, Any]], tool_choice: str,
                 max_tokens: int) -> Dict[str, Any]:
        import requests

        body: Dict[str, Any] = {"model": self.model, "messages": messages, "max_completion_tokens": max_tokens}
        if tools:
            body["tools"] = tools
            body["tool_choice"] = tool_choice
        response = requests.post(f"{self.base}/chat/completions", json=body, timeout=self.timeout_s,
                                 headers={"Authorization": f"Bearer {os.environ.get(self.api_key_env, '')}"})
        if not response.ok:
            try:
                message = response.json().get("error", {}).get("message", response.reason)
            except Exception:
                message = response.reason
            raise RuntimeError(f"chat request failed: {message}")
        return (response.json().get("choices") or [{}])[0].get("message") or {}


# ---------------------------------------------------------------------------
# Agent
# ---------------------------------------------------------------------------

class ChatAgent:
    """``policy(tool, args)`` -> (allowed, needs_confirmation, reason); the
    default just follows the tool's tier.  ``on_stop`` stops the non-arm motion."""

    def __init__(self, tools: Dict[str, Tool], knowledge: Any, llm: Any = None, inner: Any = None,
                 manipulator: Optional[Manipulator] = None, policy: Optional[Callable[[Tool, Dict[str, Any]], Tuple[bool, bool, str]]] = None,
                 on_stop: Optional[Callable[[], None]] = None, log: Callable[[str], None] = lambda _line: None) -> None:
        self._inner = inner
        self.tools = tools
        self.knowledge = knowledge
        self.llm = llm
        self.manipulator = manipulator
        self.policy = policy
        self.on_stop = on_stop
        self.log = log
        self.settings = inner.settings if inner is not None else _LocalSettings()
        self._confirm_prompt: Callable[[str, Any], bool] = lambda _skill, _policy: False
        self._history: Deque[Dict[str, str]] = deque(maxlen=MAX_HISTORY_MESSAGES)

    def __getattr__(self, name: str) -> Any:
        inner = self.__dict__.get("_inner")
        if inner is None:
            raise AttributeError(name)
        return getattr(inner, name)

    # -- entry points -----------------------------------------------------------
    def handle_cli_text(self, text: str) -> Any:
        """Same result shape as G1Agent.handle_cli_text, for G1Console."""
        results: List[Tuple[str, Any]] = []
        with self._thinking():
            reply = self._turn(text.strip(), results)
        limit = DETAIL_LIMIT_CHARS if DETAIL_RE.search(text) else SHORT_LIMIT_CHARS
        reply = shorten(reply, limit) or "Sorry, I didn't catch that. Could you say it again?"
        self._history.append({"role": "user", "content": text.strip()})
        self._history.append({"role": "assistant", "content": reply})
        return SimpleNamespace(decision=SimpleNamespace(response_text=reply), grounded_response=reply, skill_outcomes=results)

    def stop_now(self) -> str:
        """Immediate software stop; the dashboard calls it outside the chat turn."""
        if self.manipulator is not None:
            self.manipulator.stop()
        if self.on_stop is not None:
            try:
                self.on_stop()
            except Exception as exc:
                self.log(f"stop: {exc}")
        return "Stopping. My arm holds where it is. This is a software stop; for an emergency use the e-stop."

    # -- one turn ---------------------------------------------------------------
    def _thinking(self) -> Any:
        import contextlib

        activity = getattr(self._inner, "activity", None) if self._inner is not None else None
        if activity is None:
            return contextlib.nullcontext()
        return activity.activity("thinking", settings=self.settings.effective(), reason="chat")

    def _facts(self, text: str) -> List[str]:
        query = "abilities capabilities" if CAPABILITY_RE.search(text) else text
        facts = self.knowledge.search(query)
        if not facts and FOLLOW_UP_RE.search(text) and self._history:
            last_user = [m["content"] for m in self._history if m["role"] == "user"][-1:]
            facts = self.knowledge.search(" ".join(last_user + [text]))
        return facts

    def _language_line(self) -> str:
        language = "auto"
        try:
            if self._inner is not None:
                language = str(self._inner._effective_reply_language(self.settings.effective()))
            else:
                language = str(self.settings.values.get("interface.reply_language", "auto"))
        except Exception:
            pass
        return {"de": "Reply in German.", "en": "Reply in English."}.get(language, "Reply in the operator's language.")

    def _situation_line(self) -> str:
        if self.manipulator is None:
            return ""
        parts = []
        holding = self.manipulator.holding()
        if holding:
            parts.append(f"your right hand is holding the {holding}")
        if self.manipulator.busy():
            parts.append("an arm or walking command is still running")
        return ("Right now " + " and ".join(parts) + ".") if parts else ""

    def _turn(self, text: str, results: List[Tuple[str, Any]]) -> str:
        facts = self._facts(text)
        if self.llm is None:
            return self._local_turn(text, facts, results)
        system = [SYSTEM_PROMPT, self._language_line(), self._situation_line()]
        if facts:
            system.append("Things you know:\n" + "\n".join(f"- {fact}" for fact in facts))
        messages: List[Dict[str, Any]] = [{"role": "system", "content": "\n\n".join(line for line in system if line)}]
        messages.extend(self._history)
        messages.append({"role": "user", "content": text})
        schemas = [tool.schema() for tool in self.tools.values()]
        max_tokens = DETAIL_MAX_TOKENS if DETAIL_RE.search(text) else SHORT_MAX_TOKENS
        try:
            for round_index in range(MAX_ROUNDS):
                last = round_index == MAX_ROUNDS - 1
                message = self.llm.complete(messages, schemas, "none" if last else "auto", max_tokens)
                calls = message.get("tool_calls") or []
                if not calls or last:
                    answer = (message.get("content") or "").strip()
                    return answer or self._after_tools(results) or self._local_reply(text, facts)
                messages.append({"role": "assistant", "content": message.get("content"), "tool_calls": calls})
                for call in calls:
                    function = call.get("function") or {}
                    output = self._call_tool(function.get("name", ""), function.get("arguments") or "{}", results)
                    messages.append({"role": "tool", "tool_call_id": call.get("id"), "content": output})
        except Exception as exc:
            self.log(f"chat model unavailable: {type(exc).__name__}: {exc}")
            if results:  # the tools ran; never run them a second time
                return self._after_tools(results)
            return self._local_turn(text, facts, results, model_failed=True)
        return self._after_tools(results) or self._local_reply(text, facts)

    def _call_tool(self, name: str, raw_args: Any, results: List[Tuple[str, Any]]) -> str:
        """Run one tool through its gate; the result text goes back to the model."""
        tool = self.tools.get(name)
        if tool is None:
            return f"failed: there is no tool named {name}"
        try:
            args = json.loads(raw_args) if isinstance(raw_args, str) else dict(raw_args)
            if not isinstance(args, dict):
                raise ValueError("arguments are not an object")
        except ValueError as exc:
            return f"failed: bad arguments ({exc})"
        ok, needs_confirmation, reason = self._gate(tool, args)
        if not ok:
            message = f"not allowed: {reason}"
        elif needs_confirmation and not self._confirm_prompt(tool.name, SimpleNamespace(reason=reason)):
            message = "the operator declined"
            ok = False
        else:
            try:
                ok, message = tool.run(**args)
            except TypeError as exc:
                ok, message = False, f"bad arguments ({exc})"
            except Exception as exc:
                ok, message = False, f"{type(exc).__name__}: {exc}"
        results.append((tool.name, SimpleNamespace(ok=ok, message=shorten(message, TOOL_LINE_CHARS))))
        return f"{'ok' if ok else 'failed'}: {message}"

    def _gate(self, tool: Tool, args: Dict[str, Any]) -> Tuple[bool, bool, str]:
        reason = tool.description
        if tool.ask is not None:
            try:
                reason = tool.ask(**args)
            except Exception:
                pass
        if self.policy is not None:
            allowed, confirm, why = self.policy(tool, args)
            return allowed, confirm, reason if allowed else why
        return True, tool.tier == "confirm", reason

    @staticmethod
    def _after_tools(results: List[Tuple[str, Any]]) -> str:
        return " ".join(f"{result.message[0].upper()}{result.message[1:]}." for _name, result in results[-2:]
                        if result.message).replace("..", ".")

    # -- without the language model -----------------------------------------------
    def _local_turn(self, text: str, facts: List[str], results: List[Tuple[str, Any]], model_failed: bool = False) -> str:
        commands = parse_commands(text)
        if commands:
            for name, args in commands:
                self._call_tool(name, args, results)
            return self._after_tools(results)
        reply = self._local_reply(text, facts)
        if model_failed and not facts and not self._small_talk(text):
            return "I can't reach my language model right now, but I can still follow simple commands like 'grab the cup'."
        return reply

    @staticmethod
    def _small_talk(text: str) -> Optional[str]:
        for pattern, reply in _SMALL_TALK:
            if re.search(pattern, text, re.IGNORECASE):
                return reply
        return None

    def _local_reply(self, text: str, facts: List[str]) -> str:
        if facts:
            return facts[0].split("? ", 1)[-1]
        return self._small_talk(text) or ("I'm not sure about that one. You can ask me about the G1, the Unitree SDK or "
                                          "EF Robotics, or give me a command like 'grab the cup'.")
