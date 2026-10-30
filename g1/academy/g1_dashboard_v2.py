#!/usr/bin/env python3
"""G1 Dashboard V2: the original dashboard with the V2 /sort pipeline.

Run from academy/:

    python3 g1_dashboard_v2.py                 # live robot (same flags as g1_dashboard.py)
    python3 g1_dashboard_v2.py --off-robot     # UI test: simulated robot, arm, hand and sort scene

Every tab, route and flag of g1_dashboard.py is reused unchanged (that module
is imported, not edited); only /sort is replaced:

  * sorting_v2.SorterV2       explicit state machine with per-state recovery
  * grasp_pipeline_v2         one arm controller, validated straight-line
                              moves, whole-hand table geometry, Dex3 grip
                              with per-element tactile baselines
  * sort_v2_config            hardware limits / safety thresholds / tuning
  * sim_sort_v2               off-robot simulation (also used by the tests)

and the chat is g1_agent.chat.ChatAgent: short replies, knowledge about the G1,
the Unitree SDK and EF Robotics, and tools for grabbing, placing, hand moves,
the balanced hold pose, walking and SLAM on the same V2 pipeline.  Off-robot it
runs without a language model, on the simulated robot and sort scene.

Root causes fixed relative to V1 (details in grasp_pipeline_v2's docstring):
joint-space partial steps dipping into the table margin with an endless
retry loop; gain halving (kp 60 -> 30) at every controller handover that let
the arm sag and the lift stop half way; a global-maximum tactile test that
could not see contacts; a 3 Hz approach whose convergence/stall rules
delayed finger closure by many seconds.

The Stop button is a software cancellation.  It is not an emergency stop:
use the robot's physical e-stop or damping for that.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import g1_dashboard as base  # handles --off-robot stubbing before robot modules load
from g1_dashboard import LiveRequired, np, cv2
import g1_agent.chat

SORT_V2_TITLE = "G1 Dashboard V2"

# Dex3 pose constants the V2 grasp code imports from sdk_wrapper.  The
# --off-robot stub module does not define them, so they are evaluated from
# sdk_wrapper.py's own source (single source of truth for the mirrored-hand
# limits) without importing its DDS dependencies.
_HAND_CONSTANTS = ("HAND_JOINT_NAMES", "HAND_MAX", "HAND_MIN", "HAND_THUMB0", "HAND_CLOSED", "HAND_OPEN",
                   "DEX3_THUMB_PROXIMAL_IDX", "DEX3_THUMB_OPEN_MARGIN_RAD", "HAND_SAFE_OPEN")


def _ensure_stub_hand_constants() -> None:
    import ast
    import math
    from pathlib import Path

    stub = sys.modules.get("sdk_wrapper")
    if stub is None or all(hasattr(stub, name) for name in _HAND_CONSTANTS):
        return
    tree = ast.parse((Path(__file__).resolve().parent / "sdk_wrapper.py").read_text(encoding="utf-8"))
    namespace: dict = {"math": math}
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in _HAND_CONSTANTS for t in node.targets):
            exec(compile(ast.Module(body=[node], type_ignores=[]), "sdk_wrapper.py", "exec"), namespace)
    for name in _HAND_CONSTANTS:
        if name not in namespace:
            raise RuntimeError(f"sdk_wrapper.py no longer defines {name}")
        setattr(stub, name, namespace[name])


class SortSessionV2(base.SortSession):
    """SortSession with the V2 pipeline; --off-robot builds the simulator."""

    def __init__(self, dash: Any, args: Any) -> None:
        super().__init__(dash, args)
        self.sim: Any = None

    def pipeline(self) -> Any:
        self.dash._require_live()
        with self._lock:
            if self._pipeline is not None:
                return self._pipeline
            if np is None or cv2 is None:
                raise LiveRequired("The V2 sort pipeline needs numpy and opencv-contrib-python.")
            try:
                if self.dash.off_robot:
                    _ensure_stub_hand_constants()
                import sort_v2_config
                if self.dash.off_robot:
                    import sim_sort_v2
                    robot, scene, pipeline, sorter = sim_sort_v2.build_sim(sort_v2_config.default_config())
                    self.sim = (robot, scene)
                    self.sorter = sorter
                    self._pipeline = pipeline
                    return pipeline
                import grasp_pipeline_v2
            except ImportError as exc:
                raise LiveRequired(f"The V2 sort pipeline could not load its modules: {exc}") from exc
            openai_vision = self.args.vision_backend == "openai"
            cfg = sort_v2_config.default_config()
            # ik_calibration.py analyze --apply writes this (bounded corrections only).
            camera, notes = sort_v2_config.load_calibration(cfg, Path(__file__).resolve().parent / "sort_v2_calibration.json")
            self._pipeline = grasp_pipeline_v2.GraspPipelineV2(
                self.dash.robot, self.args.rgbd, cfg=cfg, vision_backend=self.args.vision_backend,
                vision_model=self.args.openai_vision_model if openai_vision else self.args.vision_model)
            if camera is not None:
                with self._pipeline.lock:
                    self._pipeline.camera_extrinsic = camera
            if notes:
                self.dash.say("system", "/sort V2 calibration loaded: " + "; ".join(notes))
        return self._pipeline

    def get(self) -> Any:
        pipeline = self.pipeline()
        with self._lock:
            if self.sorter is None:
                try:
                    import sorting_v2
                except ImportError as exc:
                    raise LiveRequired(f"/sort V2 could not load its modules: {exc}") from exc
                self.sorter = sorting_v2.SorterV2(pipeline)
        return self.sorter

    def close(self) -> None:
        super().close()
        if self.sim is not None:
            robot, scene = self.sim
            scene.close()
            robot.close()


class DashboardV2(base.Dashboard):
    """A stop command acts at once, also while the chat is busy with a turn."""

    def chat(self, text: str) -> dict[str, Any]:
        stop = getattr(self.agent, "stop_now", None)
        if stop is not None and g1_agent.chat.is_stop_command(text):
            self.say("you", text.strip())
            self.say("agent", stop())
            return {"ok": True}
        return super().chat(text)

    def chat_report(self, text: str) -> None:
        """A long chat command finished: shown, and spoken like a reply."""
        self.say("agent", text)
        if self.settings["voice_reply"] and self.robot is not None:
            try:
                self.robot.say(text, language=self.settings["voice_language"], volume=int(self.settings["robot_volume"]))
            except Exception as exc:
                self.say("error", f"Voice reply failed: {exc}")


# ---------------------------------------------------------------------------
# Page: the V1 page plus a V2 diagnostics panel in /sort
# ---------------------------------------------------------------------------

_V2_PANEL = r"""
    <div class="glass panel span-all" id="sort-v2">
      <div class="panel-head"><h2>Sort V2 · state &amp; diagnostics</h2><span class="muted small">Stop is a software cancellation, not an emergency stop.</span></div>
      <div id="v2-headline" style="font-size:20px;font-weight:700;margin:2px 0 10px"></div>
      <div class="grid cols-3">
        <div><div class="section-title">Target &amp; motion</div><div class="kv small" id="v2-motion"></div></div>
        <div><div class="section-title">Hand &amp; grasp</div><div class="kv small" id="v2-hand"></div></div>
        <div><div class="section-title">Progress</div><div class="kv small" id="v2-progress"></div></div>
      </div>
      <div class="section-title">Tuning <span class="muted small">(clamped to safety limits; hardware limits and safety thresholds are not adjustable)</span></div>
      <div id="v2-tuning" class="row" style="flex-wrap:wrap;gap:10px"></div>
    </div>
"""

_V2_JS = r"""
<script>
const v2mm=v=>v==null||!isFinite(v)?'—':(v*1000).toFixed(0)+' mm';
const v2xyz=a=>a?'('+a.map(x=>fmtq(x,3)).join(', ')+') m':'—';
const v2deg=v=>v==null||!isFinite(v)?'—':(v*180/Math.PI).toFixed(1)+'°';
let v2TuningBuilt=false;
function v2kv(id,rows){const el=document.getElementById(id);if(!el)return;el.innerHTML='';rows.forEach(([a,b,c])=>el.append(h('div',{},a),h('div',{class:'mono',style:c?`color:${c}`:''},b==null?'—':String(b))));}
function renderSortV2(s){const v=s.v2;if(!v)return;const d=v.diag||{},m=v.machine||{},c=d.controller||{};
  const st=m.state||'IDLE',bad=['ERROR','ABORTED'].includes(st),warn=st==='RECOVERING';
  const hl=document.getElementById('v2-headline');hl.textContent=v.headline||st;hl.style.color=bad?'var(--err)':warn?'var(--warn)':st==='COMPLETED'?'var(--ok)':'var(--text)';
  const t=d.target||{};
  v2kv('v2-motion',[['state',`${st} · ${fmt(m.since_s,1)} s`],['target',t.object?`${t.object} (${t.label}, ${fmt(t.score,2)}, ${t.consistent} passes)`:'—'],['object xyz',v2xyz(t.xyz)],
    ['waypoint',d.waypoint],['measured EE',v2xyz(d.measured_ee)],['desired EE',v2xyz(d.desired_ee)],['EE error',v2mm(d.ee_error_m)],['palm tilt',v2deg(d.palm_tilt_rad)],
    ['collision check',d.collision,d.collision&&d.collision!=='ok'?'var(--warn)':''],
    ['controller',c.running?`tracking ${fmt(c.tracking_error_rad,3)} rad · sag comp ${fmt(c.sag_compensation_rad,3)} rad${c.sag_saturated?' (SATURATED)':''}`:'not running']]);
  const fp=d.finger_positions,ft=d.finger_targets,td=d.tactile_deltas;
  v2kv('v2-hand',[['grasp',d.grasp_status],['evidence',d.grasp_evidence],['fingers',fp?fp.map(x=>fmt(x,2)).join(' '):'—'],['finger targets',ft?ft.map(x=>fmt(x,2)).join(' '):'—'],
    ['tactile Δ (raw)',td?td.map(x=>Math.round(x)).join(' '):'— (no baseline)'],['lift',d.lift_target_m!=null?`${v2mm(d.lift_actual_m)} / ${v2mm(d.lift_target_m)}`:'—']]);
  const counts=(v.counts||[]).map(o=>`${o.name} ${o.confirmed}/${o.count}${o.unconfirmed?` (+${o.unconfirmed}?)`:''}${o.attempts?` · ${o.attempts} fail`:''}`).join(' · ');
  v2kv('v2-progress',[['progress',d.move_progress!=null?`${Math.round(d.move_progress*100)}% · ${fmt(d.move_elapsed_s,1)} / ${fmt(d.move_timeout_s,0)} s`:'—'],
    ['sorted',counts||'—'],['last failure',m.last_failure?`${m.last_failure}: ${m.last_failure_detail}`:'—',m.last_failure?'var(--err)':''],['recovery',m.recovery_reason||'—'],
    ['preparation pose',v.prep?v2xyz(v.prep):'not recorded yet'],['notes',(v.notes||[]).join('; ')||'—'],
    ['reach map',v.reach?(v.reach.state==='computing'?`computing ${Math.round(v.reach.progress*100)}%`:v.reach.state==='ready'?`ready (table z ${fmt(v.reach.table_z,2)} m, computed in ${fmt(v.reach.computed_s,0)} s)`:v.reach.state+(v.reach.error?': '+v.reach.error:'')):'—']]);
  if(!v2TuningBuilt&&s.options&&s.options.tuning){v2TuningBuilt=true;const root=document.getElementById('v2-tuning');
    Object.entries(s.options.tuning).forEach(([k,val])=>{const b=s.options.tuning_bounds[k];const inp=typeof val==='boolean'?h('input',{type:'checkbox',checked:val}):h('input',{type:'number',step:'any',min:b[0],max:b[1],value:val,style:'width:90px'});
      inp.onchange=async()=>{try{const j=await api('/api/sort/options',{tuning:{[k]:inp.type==='checkbox'?inp.checked:Number(inp.value)}});const nv=j.options.tuning[k];if(inp.type==='checkbox')inp.checked=nv;else inp.value=nv;toast(`${k} = ${nv}`,'ok');}catch(e){toast(e.message,'err');}};
      root.append(h('label',{class:'small',style:'display:flex;gap:6px;align-items:center'},k,inp));});}}
</script>
"""


def build_page(page: str) -> str:
    """V1 page with the V2 title, wording and /sort diagnostics panel."""
    edits = [
        ("<title>G1 Dashboard</title>", f"<title>{SORT_V2_TITLE}</title>"),
        ("<b>G1 Dashboard</b>", f"<b>{SORT_V2_TITLE}</b>"),
        ('<button class="btn sm" data-sort="retract">Retract to stable pose</button>',
         '<button class="btn sm" data-sort="retract">Return to preparation pose</button>'),
        ("The right arm picks each configured object, lifts it to the WBC stable pose and places it on its ArUco spot.",
         "Sort V2: preparation pose, then for each object: validated approach, verified grasp, measured lift, "
         "transport, placement and return. Stop holds the arm where it is; it is not an emergency stop."),
        ("const lg=$('#sort-log'),atBottom", "renderSortV2(s);const lg=$('#sort-log'),atBottom"),
        ('<button data-v="desk-obstacles">Desk</button></div>',
         '<button data-v="desk-obstacles">Desk</button><button data-v="reach">Reach</button></div>'),
        ("</body>", _V2_JS + "</body>"),
    ]
    for old, new in edits:
        if page.count(old) != 1:
            raise RuntimeError(f"g1_dashboard.PAGE changed; V2 patch anchor not found: {old[:60]!r}")
        page = page.replace(old, new)
    anchor = '<div class="glass panel span-all"><div class="panel-head"><h3>Log</h3>'
    if page.count(anchor) != 1:
        raise RuntimeError("g1_dashboard.PAGE changed; /sort log panel anchor not found")
    return page.replace(anchor, _V2_PANEL + anchor)


def _install() -> None:
    base.PAGE = build_page(base.PAGE)
    base.Handler.server_version = "G1DashboardV2/1.0"


def main() -> int:
    _install()
    args = base.parse_args()
    if args.off_robot:
        return _main_off_robot(args)

    try:
        robot = None if args.offline else base.sdk_wrapper.G1(iface=args.iface, domain_id=args.domain_id)
        host = None if robot is None else g1_agent.AgentHost(robot, args.rgbd)
        agent = g1_agent.build_agent(args, robot, host)
    except Exception as exc:
        print(f"Could not start g1_dashboard_v2: {exc}", file=sys.stderr)
        return 2
    dash = DashboardV2(robot, agent, offline=args.offline, args=args)
    dash.sort = SortSessionV2(dash, args)
    if host is not None:
        host.bind(arm_conflict=lambda: dash.arm_conflict("agent"), grasp_pipeline=dash.sort.pipeline)
        _install_chat(dash, g1_agent.build_chat_agent(
            agent, robot, sorter=dash.sort.get, built_sorter=lambda: dash.sort.sorter,
            conflict=lambda: dash.arm_conflict("agent"), notify=dash.chat_report))
    return base._serve(dash, args, robot)


def _install_chat(dash: Any, chat_agent: Any) -> None:
    """What G1Console does with its agent at construction."""
    dash.agent = chat_agent
    chat_agent._confirm_prompt = dash.confirm_prompt
    chat_agent.settings.set("interface.reply_language", dash.settings["ai_language"], persist=False)


def _sim_chat_agent(dash: Any, robot: Any) -> Any:
    """The chat on the simulated robot and sort scene; the skills the live
    agent gets from its registry are small stand-ins here."""
    from g1_agent.chat import ChatAgent, build_tools
    from g1_agent.manipulation import Manipulator

    manipulator = Manipulator(dash.sort.get, lambda: dash.sort.sorter, lambda: dash.arm_conflict("agent"), dash.chat_report)

    def walk(direction: str, distance_m: float, speed_mps: float = 0.3) -> str:
        axis = {"forward": (1, 0), "backward": (-1, 0), "left": (0, 1), "right": (0, -1)}[direction]
        robot.loco_move(axis[0] * speed_mps, axis[1] * speed_mps, 0.0, duration_s=distance_m / speed_mps)
        return f"moved {direction} {distance_m:.2f} m"

    def turn(sign: float, degrees: float) -> str:
        robot.loco_move(0.0, 0.0, sign * 0.5, duration_s=abs(degrees) * 3.14159 / 180.0 / 0.5)
        return f"turned {abs(degrees):.0f} degrees"

    skills = {
        "move": walk, "turn_left": lambda degrees=30.0: turn(1.0, degrees), "turn_right": lambda degrees=30.0: turn(-1.0, degrees),
        "prepare": lambda: robot.prepare_mode(), "walk_mode": lambda: robot.walk_mode(), "run_mode": lambda: robot.run_mode(),
        "damp": lambda: robot.damp_mode(), "zero_torque": lambda: robot.zero_torque_mode(), "squat": lambda: robot.prepare_mode(),
        "gesture": lambda name: f"played {name} (simulated)",
        "start_mapping": lambda: robot.start_mapping(), "save_map": lambda: robot.stop_mapping(),
        "stop_slam": lambda: robot.stop_mapping(save=False), "start_relocation": lambda: robot.relocate(),
        "slam_status": lambda: f"pose x {robot.x:.2f} y {robot.y:.2f}",
        "save_current_nav_point": lambda name: f"saved {name} at {robot.add_point(name)}",
        "list_nav_points": lambda: "no named points in the simulator", "navigate_named_point": lambda name, auto_relocate=True: f"{name} is not known in the simulator",
        "pause_navigation": lambda: robot.pause_nav(), "resume_navigation": lambda: robot.resume_nav(),
    }

    def call_skill(name: str, **kwargs: Any) -> tuple[bool, str]:
        return True, str(skills[name](**kwargs))

    def status() -> str:
        holding = manipulator.holding()
        return f"battery {robot.soc:.0f}%, standing" + (f", right hand holding {holding}" if holding else "")

    return ChatAgent(build_tools(manipulator, call_skill, status), g1_agent.chat_knowledge(), manipulator=manipulator,
                     on_stop=robot.loco_stop, log=lambda line: dash.say("system", line))


def _main_off_robot(args: Any) -> int:
    import shutil
    import tempfile
    from pathlib import Path

    sim_settings = Path(tempfile.gettempdir()) / "g1_dashboard_off_robot_settings.json"
    if not sim_settings.exists() and base.g1_cli.SETTINGS_PATH.exists():
        shutil.copyfile(base.g1_cli.SETTINGS_PATH, sim_settings)
    base.g1_cli.SETTINGS_PATH = sim_settings
    robot = base.g1_dashboard_sim.SimRobot()
    dash = DashboardV2(robot, base.g1_dashboard_sim.SimAgent(robot), offline=False, args=args)
    dash.vision.sim = base.g1_dashboard_sim.SimCamera(robot)
    dash.slam.points_path = Path(tempfile.gettempdir()) / "g1_dashboard_off_robot_points.json"
    dash.sort = SortSessionV2(dash, args)
    _install_chat(dash, _sim_chat_agent(dash, robot))
    dash.say("system", "Off-robot simulator (V2): robot, SLAM, camera, agent and the /sort scene are simulated. "
                       "Simulated sorting exercises the sequence and recovery logic only, not real-robot safety. "
                       "The chat uses its local command parser and knowledge, without a language model.")
    return base._serve(dash, args, robot)


if __name__ == "__main__":
    raise SystemExit(main())
