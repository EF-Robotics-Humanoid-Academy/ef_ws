#!/usr/bin/env python3
"""
hl_agentic_module.py

High-level agentic task executor for Unitree G1 using dev/ef_client.py.
Input is a JSON task specification. The module validates it, creates
`ef_client.Robot`, executes actions sequentially, and prints a JSON report.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, TYPE_CHECKING


_THIS_FILE = Path(__file__).resolve()
_DEV_DIR = _THIS_FILE.parents[2]  # .../scripts/dev
if str(_DEV_DIR) not in sys.path:
    sys.path.insert(0, str(_DEV_DIR))

if TYPE_CHECKING:
    from ef_client import Robot  # noqa: F401


SUPPORTED_SCHEMA_VERSION = "1.0"


@dataclass
class ActionResult:
    index: int
    action_type: str
    ok: bool
    started_at: float
    finished_at: float
    duration_s: float
    return_value: Any = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "type": self.action_type,
            "ok": self.ok,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "duration_s": self.duration_s,
            "return_value": self.return_value,
            "error": self.error,
        }


def _as_float(v: Any, key: str) -> float:
    try:
        return float(v)
    except Exception as exc:
        raise ValueError(f"'{key}' must be numeric") from exc


def _as_int(v: Any, key: str) -> int:
    try:
        return int(v)
    except Exception as exc:
        raise ValueError(f"'{key}' must be integer") from exc


def _require(action: dict[str, Any], key: str) -> Any:
    if key not in action:
        raise ValueError(f"Missing required field '{key}'")
    return action[key]


def _validate_task_schema(task: dict[str, Any]) -> None:
    if not isinstance(task, dict):
        raise ValueError("Task must be a JSON object")

    version = str(task.get("schema_version", ""))
    if version != SUPPORTED_SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported schema_version '{version}'. Supported: '{SUPPORTED_SCHEMA_VERSION}'"
        )

    actions = task.get("actions")
    if not isinstance(actions, list) or not actions:
        raise ValueError("'actions' must be a non-empty list")

    for i, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ValueError(f"actions[{i}] must be an object")
        if "type" not in action:
            raise ValueError(f"actions[{i}] missing 'type'")


def _load_task(args: argparse.Namespace) -> dict[str, Any]:
    if args.task_json and args.task_file:
        raise ValueError("Use either --task-json or --task-file, not both")

    if args.task_json:
        try:
            task = json.loads(args.task_json)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid JSON in --task-json: {exc}") from exc
    elif args.task_file:
        path = Path(args.task_file).expanduser().resolve()
        task = json.loads(path.read_text(encoding="utf-8"))
    else:
        task = json.load(sys.stdin)

    if not isinstance(task, dict):
        raise ValueError("Loaded task JSON must be an object")
    return task


def _build_robot(task: dict[str, Any]) -> Robot:
    from ef_client import Robot  # imported lazily so --print-example works without SDK deps

    cfg = task.get("robot", {})
    if cfg is None:
        cfg = {}
    if not isinstance(cfg, dict):
        raise ValueError("'robot' must be an object when provided")

    iface = str(cfg.get("iface", "eth0"))
    domain_id = _as_int(cfg.get("domain_id", 0), "robot.domain_id")
    safety_boot = bool(cfg.get("safety_boot", True))
    auto_start_sensors = bool(cfg.get("auto_start_sensors", True))

    return Robot(
        iface=iface,
        domain_id=domain_id,
        safety_boot=safety_boot,
        auto_start_sensors=auto_start_sensors,
    )


def _dispatch_action(robot: Robot, action: dict[str, Any]) -> Any:
    t = str(action["type"]).strip().lower()

    # Locomotion primitives
    if t == "balanced_stand":
        mode = _as_int(action.get("mode", 0), "mode")
        robot.balanced_stand(mode=mode)
        return {"mode": mode}

    if t == "stop":
        robot.stop()
        return None

    if t == "sleep":
        duration_s = _as_float(action.get("duration_s", 0.5), "duration_s")
        if duration_s < 0.0:
            raise ValueError("duration_s must be >= 0")
        time.sleep(duration_s)
        return {"slept_s": duration_s}

    if t == "walk":
        vx = _as_float(action.get("vx", 0.0), "vx")
        vy = _as_float(action.get("vy", 0.0), "vy")
        vyaw = _as_float(action.get("vyaw", 0.0), "vyaw")
        return int(robot.walk(vx=vx, vy=vy, vyaw=vyaw))

    if t == "run":
        vx = _as_float(action.get("vx", 0.0), "vx")
        vy = _as_float(action.get("vy", 0.0), "vy")
        vyaw = _as_float(action.get("vyaw", 0.0), "vyaw")
        return int(robot.run(vx=vx, vy=vy, vyaw=vyaw))

    if t == "walk_for":
        distance = _as_float(_require(action, "distance"), "distance")
        return bool(
            robot.walk_for(
                distance=distance,
                max_vx=_as_float(action.get("max_vx", 0.25), "max_vx"),
                max_vyaw=_as_float(action.get("max_vyaw", 0.5), "max_vyaw"),
                pos_tolerance=_as_float(action.get("pos_tolerance", 0.05), "pos_tolerance"),
                yaw_tolerance=_as_float(action.get("yaw_tolerance", 0.20), "yaw_tolerance"),
                timeout=_as_float(action.get("timeout", 20.0), "timeout"),
                tick=_as_float(action.get("tick", 0.05), "tick"),
                kp_lin=_as_float(action.get("kp_lin", 0.9), "kp_lin"),
                kp_yaw=_as_float(action.get("kp_yaw", 1.6), "kp_yaw"),
            )
        )

    if t == "run_for":
        distance = _as_float(_require(action, "distance"), "distance")
        return bool(
            robot.run_for(
                distance=distance,
                max_vx=_as_float(action.get("max_vx", 0.45), "max_vx"),
                max_vyaw=_as_float(action.get("max_vyaw", 0.8), "max_vyaw"),
                pos_tolerance=_as_float(action.get("pos_tolerance", 0.07), "pos_tolerance"),
                yaw_tolerance=_as_float(action.get("yaw_tolerance", 0.25), "yaw_tolerance"),
                timeout=_as_float(action.get("timeout", 15.0), "timeout"),
                tick=_as_float(action.get("tick", 0.05), "tick"),
                kp_lin=_as_float(action.get("kp_lin", 1.0), "kp_lin"),
                kp_yaw=_as_float(action.get("kp_yaw", 1.8), "kp_yaw"),
            )
        )

    if t == "turn_for":
        angle_deg = _as_float(_require(action, "angle_deg"), "angle_deg")
        return bool(
            robot.turn_for(
                angle_deg=angle_deg,
                max_vyaw=_as_float(action.get("max_vyaw", 0.8), "max_vyaw"),
                yaw_tolerance_deg=_as_float(action.get("yaw_tolerance_deg", 2.5), "yaw_tolerance_deg"),
                timeout=_as_float(action.get("timeout", 10.0), "timeout"),
                tick=_as_float(action.get("tick", 0.05), "tick"),
                kp_yaw=_as_float(action.get("kp_yaw", 1.8), "kp_yaw"),
                gait_type=_as_int(action.get("gait_type", 0), "gait_type"),
            )
        )

    # Dexterity / upper body
    if t == "rotate_joint":
        joint_name = str(_require(action, "joint_name"))
        angle_deg = _as_float(_require(action, "angle_deg"), "angle_deg")
        arm = action.get("arm", None)
        return int(
            robot.rotate_joint(
                joint_name=joint_name,
                angle_deg=angle_deg,
                arm=None if arm is None else str(arm),
                duration=_as_float(action.get("duration", 1.0), "duration"),
                hold=_as_float(action.get("hold", 0.0), "hold"),
                cmd_hz=_as_float(action.get("cmd_hz", 50.0), "cmd_hz"),
                kp=_as_float(action.get("kp", 40.0), "kp"),
                kd=_as_float(action.get("kd", 1.0), "kd"),
                easing=str(action.get("easing", "smooth")),
            )
        )

    # Navigation / SLAM
    if t == "slam_start":
        proc = robot.start_slam(
            save_folder=str(action.get("save_folder", "./maps")),
            save_every=_as_int(action.get("save_every", 1), "save_every"),
            save_latest=bool(action.get("save_latest", True)),
            save_prefix=str(action.get("save_prefix", "live_slam_latest")),
            viz=bool(action.get("viz", False)),
        )
        return {"pid": int(proc.pid), "slam_is_running": bool(robot.slam_is_running)}

    if t == "slam_stop":
        robot.stop_slam(save_folder=str(action.get("save_folder", "./maps")))
        return {"slam_is_running": bool(robot.slam_is_running)}

    if t == "slam_nav_pose":
        x = _as_float(_require(action, "x"), "x")
        y = _as_float(_require(action, "y"), "y")
        yaw = _as_float(action.get("yaw", 0.0), "yaw")
        rc = int(
            robot.slam_nav_pose(
                x=x,
                y=y,
                yaw=yaw,
                obs_avoid=bool(action.get("obs_avoid", False)),
                use_dynamic_fallback=bool(action.get("use_dynamic_fallback", True)),
                use_rgbd_depth_guard=bool(action.get("use_rgbd_depth_guard", True)),
            )
        )
        return {"rc": rc, "ok": rc == 0}

    if t == "slam_nav_path":
        points = _require(action, "points")
        if not isinstance(points, list) or not points:
            raise ValueError("'points' must be a non-empty list")
        norm_points: list[tuple[float, float] | tuple[float, float, float]] = []
        for p in points:
            if not isinstance(p, (list, tuple)):
                raise ValueError("Each path point must be [x,y] or [x,y,yaw]")
            if len(p) == 2:
                norm_points.append((float(p[0]), float(p[1])))
            elif len(p) == 3:
                norm_points.append((float(p[0]), float(p[1]), float(p[2])))
            else:
                raise ValueError("Each path point must be [x,y] or [x,y,yaw]")

        ok = bool(
            robot.slam_nav_path(
                points=norm_points,
                obs_avoid=bool(action.get("obs_avoid", True)),
                clear_on_finish=bool(action.get("clear_on_finish", True)),
                append=bool(action.get("append", False)),
            )
        )
        return {"ok": ok}

    # Utility
    if t == "say":
        text = str(_require(action, "text"))
        robot.say(text)
        return {"text": text}

    if t == "headlight":
        args = action.get("args", None)
        duration = action.get("duration", None)
        rc = int(robot.headlight(args=args, duration=None if duration is None else float(duration)))
        return {"rc": rc}

    if t == "get_state":
        state = robot.get_robot_state()
        if hasattr(state.get("imu"), "__dict__"):
            imu = state["imu"]
            state["imu"] = {
                "rpy": list(imu.rpy),
                "gyro": None if imu.gyro is None else list(imu.gyro),
                "acc": None if imu.acc is None else list(imu.acc),
                "quat": None if imu.quat is None else list(imu.quat),
                "temp": imu.temp,
            }
        return state

    raise ValueError(f"Unsupported action type '{t}'")


def _execute_task(task: dict[str, Any]) -> dict[str, Any]:
    _validate_task_schema(task)

    context = task.get("context", {})
    if context is None:
        context = {}
    if not isinstance(context, dict):
        raise ValueError("'context' must be an object when provided")

    stop_on_error = bool(context.get("stop_on_error", True))
    dry_run = bool(context.get("dry_run", False))

    robot = _build_robot(task)

    action_results: list[ActionResult] = []
    all_ok = True

    try:
        for idx, action in enumerate(task["actions"]):
            a_type = str(action.get("type", "unknown"))
            t0 = time.time()
            if dry_run:
                result = ActionResult(
                    index=idx,
                    action_type=a_type,
                    ok=True,
                    started_at=t0,
                    finished_at=t0,
                    duration_s=0.0,
                    return_value={"dry_run": True},
                )
                action_results.append(result)
                continue

            try:
                ret = _dispatch_action(robot, action)
                t1 = time.time()
                result = ActionResult(
                    index=idx,
                    action_type=a_type,
                    ok=True,
                    started_at=t0,
                    finished_at=t1,
                    duration_s=t1 - t0,
                    return_value=ret,
                )
            except Exception as exc:
                t1 = time.time()
                all_ok = False
                result = ActionResult(
                    index=idx,
                    action_type=a_type,
                    ok=False,
                    started_at=t0,
                    finished_at=t1,
                    duration_s=t1 - t0,
                    error=str(exc),
                )
            action_results.append(result)

            if not result.ok and stop_on_error:
                break
    finally:
        try:
            robot.stop()
        except Exception:
            pass

    response = {
        "task_id": task.get("task_id"),
        "schema_version": SUPPORTED_SCHEMA_VERSION,
        "ok": all_ok,
        "executed_actions": len(action_results),
        "results": [r.to_dict() for r in action_results],
    }
    return response


def _example_task() -> dict[str, Any]:
    return {
        "schema_version": "1.0",
        "task_id": "demo_walk_turn_say",
        "robot": {
            "iface": "eth0",
            "domain_id": 0,
            "safety_boot": True,
            "auto_start_sensors": True,
        },
        "context": {
            "stop_on_error": True,
            "dry_run": False,
        },
        "actions": [
            {"type": "balanced_stand"},
            {"type": "walk_for", "distance": 0.6, "timeout": 12.0},
            {"type": "turn_for", "angle_deg": 45},
            {
                "type": "rotate_joint",
                "joint_name": "right elbow",
                "angle_deg": 15,
                "duration": 1.0,
                "hold": 0.2,
            },
            {"type": "say", "text": "Task finished."},
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="High-level JSON task executor using ef_client.Robot",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--task-file", default="", help="Path to JSON task file")
    parser.add_argument("--task-json", default="", help="Inline JSON task string")
    parser.add_argument("--print-example", action="store_true", help="Print example task JSON and exit")
    parser.add_argument(
        "--pretty",
        action="store_true",
        help="Pretty-print execution result JSON",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.print_example:
        print(json.dumps(_example_task(), indent=2, ensure_ascii=True))
        return

    task = _load_task(args)
    result = _execute_task(task)
    if args.pretty:
        print(json.dumps(result, indent=2, ensure_ascii=True))
    else:
        print(json.dumps(result, ensure_ascii=True))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=True), file=sys.stderr)
        sys.exit(1)
