#!/usr/bin/env python3
"""
engine.py

Fuses three sources:
1) RGBD feed received through sensors/manual_streaming (GStreamer UDP receiver)
2) Robot lidar occupancy grid summary from dev/ef_client.Robot
3) Latest output from vision_module.py (JSON/JSONL)

Then prompts a Hugging Face reasoning model to generate an executable task JSON
compatible with hl_agentic_module.py and writes it to disk.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any, Optional


_THIS_FILE = Path(__file__).resolve()
_DEV_DIR = _THIS_FILE.parents[2]  # .../scripts/dev
if str(_DEV_DIR) not in sys.path:
    sys.path.insert(0, str(_DEV_DIR))


TASK_SCHEMA_VERSION = "1.0"


def _read_latest_jsonl(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"vision jsonl not found: {path}")

    last = ""
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if line:
                last = line
    if not last:
        raise RuntimeError(f"vision jsonl is empty: {path}")

    data = json.loads(last)
    if not isinstance(data, dict):
        raise ValueError("Latest vision JSONL line is not an object")
    return data


def _extract_json_object(text: str) -> dict[str, Any]:
    text = text.strip()
    if not text:
        raise ValueError("Empty model output")

    fence = re.search(r"```(?:json)?\s*(\{[\s\S]*?\})\s*```", text, flags=re.IGNORECASE)
    if fence:
        obj = json.loads(fence.group(1))
        if isinstance(obj, dict):
            return obj

    start = text.find("{")
    if start < 0:
        raise ValueError("No JSON object found in model output")

    depth = 0
    end = -1
    for i, ch in enumerate(text[start:], start=start):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = i
                break
    if end < 0:
        raise ValueError("Unbalanced JSON braces in model output")

    candidate = text[start : end + 1]
    obj = json.loads(candidate)
    if not isinstance(obj, dict):
        raise ValueError("Parsed JSON is not an object")
    return obj


def _validate_task(task: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(task, dict):
        raise ValueError("task must be a JSON object")

    if str(task.get("schema_version", "")) != TASK_SCHEMA_VERSION:
        task["schema_version"] = TASK_SCHEMA_VERSION

    robot = task.get("robot", {})
    if not isinstance(robot, dict):
        robot = {}
    robot.setdefault("iface", "eth0")
    robot.setdefault("domain_id", 0)
    robot.setdefault("safety_boot", True)
    robot.setdefault("auto_start_sensors", True)
    task["robot"] = robot

    context = task.get("context", {})
    if not isinstance(context, dict):
        context = {}
    context.setdefault("stop_on_error", True)
    context.setdefault("dry_run", False)
    task["context"] = context

    actions = task.get("actions")
    if not isinstance(actions, list) or not actions:
        raise ValueError("task.actions must be a non-empty list")

    norm_actions: list[dict[str, Any]] = []
    for i, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ValueError(f"actions[{i}] must be an object")
        if "type" not in action:
            raise ValueError(f"actions[{i}] missing type")
        a = dict(action)
        a["type"] = str(a["type"]).strip()
        norm_actions.append(a)
    task["actions"] = norm_actions

    task.setdefault("task_id", f"engine_task_{int(time.time())}")
    return task


def _build_fallback_task(world: dict[str, Any], iface: str, domain_id: int) -> dict[str, Any]:
    vision = world.get("vision", {}) if isinstance(world.get("vision"), dict) else {}
    turn_bias = (
        vision.get("locomotion", {}).get("suggested_turn_bias", "forward")
        if isinstance(vision.get("locomotion"), dict)
        else "forward"
    )
    center_blocked = (
        float(vision.get("locomotion", {}).get("region_blocked_confidence", {}).get("center", 0.0))
        if isinstance(vision.get("locomotion", {}).get("region_blocked_confidence", {}), dict)
        else 0.0
    )

    actions: list[dict[str, Any]] = [{"type": "balanced_stand"}]

    if center_blocked > 0.55:
        if turn_bias == "left":
            actions.append({"type": "turn_for", "angle_deg": 30})
        elif turn_bias == "right":
            actions.append({"type": "turn_for", "angle_deg": -30})
        else:
            actions.append({"type": "stop"})
    else:
        actions.append({"type": "walk_for", "distance": 0.35, "timeout": 8.0})

    dex = vision.get("dexterity", {}) if isinstance(vision.get("dexterity"), dict) else {}
    dex_label = str(dex.get("label", ""))
    if "handle" in dex_label or "button" in dex_label:
        actions.append({"type": "rotate_joint", "joint_name": "right wrist_pitch", "angle_deg": 12, "duration": 0.8})

    return {
        "schema_version": TASK_SCHEMA_VERSION,
        "task_id": f"fallback_{int(time.time())}",
        "robot": {
            "iface": iface,
            "domain_id": int(domain_id),
            "safety_boot": True,
            "auto_start_sensors": True,
        },
        "context": {
            "stop_on_error": True,
            "dry_run": False,
        },
        "actions": actions,
    }


def _capture_rgbd_snapshot(rgb_port: int, depth_port: int, width: int, height: int, fps: int, timeout_s: float) -> dict[str, Any]:
    import cv2
    import numpy as np
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst

    Gst.init(None)

    rgb_pipe = Gst.parse_launch(
        f"udpsrc port={rgb_port} caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true sync=false drop=true"
    )
    depth_pipe = Gst.parse_launch(
        f"udpsrc port={depth_port} caps=application/x-rtp,media=video,encoding-name=H264,payload=97 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true sync=false drop=true"
    )

    rgb_sink = rgb_pipe.get_by_name("sink")
    depth_sink = depth_pipe.get_by_name("sink")
    if rgb_sink is None or depth_sink is None:
        raise RuntimeError("Failed to create GStreamer appsinks for RGBD")

    rgb_pipe.set_state(Gst.State.PLAYING)
    depth_pipe.set_state(Gst.State.PLAYING)

    try:
        deadline = time.time() + max(0.2, float(timeout_s))
        wait_ns = int(Gst.SECOND // max(1, int(fps)))
        rgb = None
        depth = None

        while time.time() < deadline:
            if rgb is None:
                s = rgb_sink.emit("try-pull-sample", wait_ns)
                if s:
                    b = s.get_buffer()
                    raw = np.frombuffer(b.extract_dup(0, b.get_size()), dtype=np.uint8)
                    if raw.size == width * height * 3:
                        rgb = raw.reshape((height, width, 3)).copy()
            if depth is None:
                s = depth_sink.emit("try-pull-sample", wait_ns)
                if s:
                    b = s.get_buffer()
                    raw = np.frombuffer(b.extract_dup(0, b.get_size()), dtype=np.uint8)
                    if raw.size == width * height * 3:
                        depth = raw.reshape((height, width, 3)).copy()
            if rgb is not None and depth is not None:
                break
            time.sleep(0.01)

        if rgb is None or depth is None:
            raise TimeoutError("Timed out waiting for RGBD frames from manual_streaming")

        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)
        edge_density = float((edges > 0).mean())
        brightness = float(gray.mean() / 255.0)

        hsv = cv2.cvtColor(depth, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.float32) / 179.0
        sat = hsv[:, :, 1].astype(np.float32) / 255.0
        val = hsv[:, :, 2].astype(np.float32) / 255.0

        # Proxy depth confidence from plasma-style colorized depth stream.
        near_proxy = float(((hue > 0.55) & (sat > 0.35) & (val > 0.2)).mean())
        far_proxy = float(((hue < 0.25) & (sat > 0.25) & (val > 0.2)).mean())

        return {
            "rgb": {
                "brightness": brightness,
                "edge_density": edge_density,
                "resolution": [int(width), int(height)],
            },
            "depth_visual_proxy": {
                "near_ratio_proxy": near_proxy,
                "far_ratio_proxy": far_proxy,
                "note": "Derived from colorized depth stream (manual_streaming payload=97), not metric depth.",
            },
        }
    finally:
        try:
            rgb_pipe.set_state(Gst.State.NULL)
            depth_pipe.set_state(Gst.State.NULL)
        except Exception:
            pass


def _lidar_occupancy_summary(iface: str, domain_id: int, wait_s: float, threshold: float = 0.15) -> dict[str, Any]:
    import numpy as np
    from ef_client import Robot

    robot = Robot(iface=iface, domain_id=domain_id, safety_boot=True, auto_start_sensors=True)
    time.sleep(max(0.2, float(wait_s)))

    hm = robot.get_lidar_map()
    if hm is None:
        return {
            "available": False,
            "error": "No lidar height map available",
        }

    try:
        w = int(hm.width)
        h = int(hm.height)
        resolution = float(hm.resolution)
        raw = np.array(list(hm.data), dtype=np.float32)
        if w <= 0 or h <= 0 or raw.size != w * h:
            raise ValueError("Invalid height map dimensions")

        heights = raw.reshape((h, w))
        occ = heights >= float(threshold)
        occ_ratio = float(occ.mean())
        max_h = float(np.nanmax(heights)) if heights.size else 0.0
        mean_h = float(np.nanmean(heights)) if heights.size else 0.0

        cy, cx = h // 2, w // 2
        rr = max(1, int(1.0 / max(1e-6, resolution)))
        y0, y1 = max(0, cy - rr), min(h, cy + rr + 1)
        x0, x1 = max(0, cx - rr), min(w, cx + rr + 1)
        near_occ = float(occ[y0:y1, x0:x1].mean()) if y1 > y0 and x1 > x0 else 0.0

        return {
            "available": True,
            "width": w,
            "height": h,
            "resolution_m": resolution,
            "height_threshold_m": float(threshold),
            "occupied_ratio": occ_ratio,
            "near_occupied_ratio_1m": near_occ,
            "mean_height_m": mean_h,
            "max_height_m": max_h,
        }
    except Exception as exc:
        return {
            "available": False,
            "error": str(exc),
        }


def _compose_reasoning_prompt(user_prompt: str, world: dict[str, Any], iface: str, domain_id: int) -> str:
    schema = {
        "schema_version": "1.0",
        "task_id": "string",
        "robot": {
            "iface": iface,
            "domain_id": domain_id,
            "safety_boot": True,
            "auto_start_sensors": True,
        },
        "context": {
            "stop_on_error": True,
            "dry_run": False,
        },
        "actions": [
            {"type": "balanced_stand"},
            {"type": "walk_for", "distance": 0.4, "timeout": 10.0},
            {"type": "turn_for", "angle_deg": 30},
            {"type": "rotate_joint", "joint_name": "right elbow", "angle_deg": 10},
            {"type": "slam_nav_pose", "x": 1.0, "y": 0.0, "yaw": 0.0, "obs_avoid": True},
            {"type": "stop"},
        ],
    }

    return (
        "You are a robot task planner. Create one JSON object only, no markdown.\n"
        "Output must follow schema_version=1.0 and be executable by hl_agentic_module.py.\n"
        "Use conservative safe behavior: include balanced_stand first when locomotion starts, and include stop at end for uncertain scenes.\n"
        "Choose action types only from: balanced_stand, stop, sleep, walk, run, walk_for, run_for, turn_for, rotate_joint, slam_start, slam_stop, slam_nav_pose, slam_nav_path, say, headlight, get_state.\n"
        "Prefer short plans (2-8 actions).\n\n"
        f"User intent/prompt:\n{user_prompt}\n\n"
        f"Observed world state JSON:\n{json.dumps(world, ensure_ascii=True)}\n\n"
        f"Required output schema example:\n{json.dumps(schema, ensure_ascii=True)}\n\n"
        "Return strictly one JSON object."
    )


def _run_hf_reasoner(
    model_id: str,
    prompt: str,
    max_new_tokens: int,
    temperature: float,
    top_p: float,
    device: str,
) -> str:
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer

    tokenizer = AutoTokenizer.from_pretrained(model_id)
    dtype = torch.float16 if (device.startswith("cuda") and torch.cuda.is_available()) else torch.float32

    model = AutoModelForCausalLM.from_pretrained(model_id, torch_dtype=dtype)
    if device.startswith("cuda") and torch.cuda.is_available():
        model = model.to(device)

    if hasattr(tokenizer, "apply_chat_template") and tokenizer.chat_template:
        messages = [
            {"role": "system", "content": "You output valid JSON only."},
            {"role": "user", "content": prompt},
        ]
        inputs = tokenizer.apply_chat_template(messages, tokenize=True, add_generation_prompt=True, return_tensors="pt")
    else:
        inputs = tokenizer(prompt, return_tensors="pt").input_ids

    if device.startswith("cuda") and torch.cuda.is_available():
        inputs = inputs.to(device)

    do_sample = temperature > 0.0
    outputs = model.generate(
        inputs,
        max_new_tokens=max_new_tokens,
        do_sample=do_sample,
        temperature=max(1e-5, temperature),
        top_p=float(top_p),
        eos_token_id=tokenizer.eos_token_id,
        pad_token_id=tokenizer.eos_token_id,
    )

    gen = outputs[0][inputs.shape[-1] :]
    return tokenizer.decode(gen, skip_special_tokens=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Multi-sensor planning engine to generate task.json via Hugging Face reasoning model",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--prompt", default="", help="Task prompt to guide planner")
    parser.add_argument("--prompt-file", default="", help="Path to file containing prompt text")

    parser.add_argument("--iface", default="eth0", help="Robot network interface passed to ef_client")
    parser.add_argument("--domain-id", type=int, default=0, help="DDS domain id")

    parser.add_argument("--rgb-port", type=int, default=5600)
    parser.add_argument("--depth-port", type=int, default=5602)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--rgbd-timeout", type=float, default=3.0, help="Seconds to wait for RGBD snapshot")

    parser.add_argument(
        "--vision-jsonl",
        default="/tmp/vision_state.jsonl",
        help="Path to vision_module JSONL output; latest line is used",
    )
    parser.add_argument("--lidar-wait", type=float, default=0.6, help="Seconds to wait for lidar map warmup")

    parser.add_argument("--hf-model", default="TinyLlama/TinyLlama-1.1B-Chat-v1.0", help="Hugging Face model id")
    parser.add_argument("--hf-device", default="cpu", help="e.g. cpu or cuda:0")
    parser.add_argument("--max-new-tokens", type=int, default=512)
    parser.add_argument("--temperature", type=float, default=0.15)
    parser.add_argument("--top-p", type=float, default=0.9)

    parser.add_argument("--output-task", default="./task.json", help="Output task JSON path")
    parser.add_argument(
        "--save-world-state",
        default="",
        help="Optional path to save fused world-state JSON used in the prompt",
    )
    parser.add_argument("--allow-fallback", action="store_true", help="Use heuristic fallback task if model output is invalid")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print output task file")
    return parser.parse_args()


def _load_prompt(args: argparse.Namespace) -> str:
    if args.prompt and args.prompt_file:
        raise ValueError("Use either --prompt or --prompt-file, not both")
    if args.prompt_file:
        p = Path(args.prompt_file).expanduser().resolve()
        txt = p.read_text(encoding="utf-8").strip()
        if not txt:
            raise ValueError(f"Prompt file is empty: {p}")
        return txt
    if args.prompt.strip():
        return args.prompt.strip()
    raise ValueError("Provide planning prompt via --prompt or --prompt-file")


def main() -> None:
    args = parse_args()
    prompt = _load_prompt(args)

    world: dict[str, Any] = {
        "timestamp": time.time(),
        "vision": {},
        "rgbd": {},
        "lidar": {},
    }

    vision_path = Path(args.vision_jsonl).expanduser().resolve()
    world["vision"] = _read_latest_jsonl(vision_path)

    world["rgbd"] = _capture_rgbd_snapshot(
        rgb_port=int(args.rgb_port),
        depth_port=int(args.depth_port),
        width=int(args.width),
        height=int(args.height),
        fps=int(args.fps),
        timeout_s=float(args.rgbd_timeout),
    )

    world["lidar"] = _lidar_occupancy_summary(
        iface=str(args.iface),
        domain_id=int(args.domain_id),
        wait_s=float(args.lidar_wait),
        threshold=0.15,
    )

    if args.save_world_state:
        ws_path = Path(args.save_world_state).expanduser().resolve()
        ws_path.parent.mkdir(parents=True, exist_ok=True)
        ws_path.write_text(json.dumps(world, indent=2, ensure_ascii=True), encoding="utf-8")

    full_prompt = _compose_reasoning_prompt(prompt, world, iface=str(args.iface), domain_id=int(args.domain_id))

    raw = _run_hf_reasoner(
        model_id=str(args.hf_model),
        prompt=full_prompt,
        max_new_tokens=int(args.max_new_tokens),
        temperature=float(args.temperature),
        top_p=float(args.top_p),
        device=str(args.hf_device),
    )

    try:
        task = _extract_json_object(raw)
        task = _validate_task(task)
    except Exception as exc:
        if not args.allow_fallback:
            raise RuntimeError(f"Model output was not valid task JSON: {exc}\nRaw output:\n{raw}") from exc
        task = _build_fallback_task(world, iface=str(args.iface), domain_id=int(args.domain_id))

    out = Path(args.output_task).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    if args.pretty:
        out.write_text(json.dumps(task, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    else:
        out.write_text(json.dumps(task, ensure_ascii=True) + "\n", encoding="utf-8")

    print(json.dumps({
        "ok": True,
        "output_task": str(out),
        "task_id": task.get("task_id"),
        "model": args.hf_model,
        "vision_source": str(vision_path),
    }, ensure_ascii=True))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=True), file=sys.stderr)
        sys.exit(1)
