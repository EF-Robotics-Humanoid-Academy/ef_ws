# VLA Stack 1 (G1)

This folder contains a minimal but practical Vision-Language-Action pipeline for Unitree G1:

- `vision_module.py`: turns RGB feed into structured semantic state
- `engine.py`: fuses vision + RGBD + lidar occupancy and asks a Hugging Face reasoning model to produce `task.json`
- `hl_agentic_module.py`: validates and executes `task.json` with `dev/ef_client.py`

The design goal is safe, short-horizon, high-level robot task generation for humanoid locomotion and dexterous actions.

## Contents

- [Architecture](#architecture)
- [Data Flow](#data-flow)
- [Requirements](#requirements)
- [Quick Start](#quick-start)
- [1) Vision Module](#1-vision-module)
- [2) Planning Engine](#2-planning-engine)
- [3) High-Level Agentic Executor](#3-high-level-agentic-executor)
- [Task JSON Schema](#task-json-schema)
- [End-to-End Example](#end-to-end-example)
- [Safety Notes](#safety-notes)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)

## Architecture

This stack splits responsibilities cleanly:

- Perception (`vision_module.py`)
- `manual_streaming` RGB feed is decoded via GStreamer (`payload=96` on UDP `5600`)
- CLIP (`open_clip`) classifies locomotion, dexterity affordance, and hazards
- Outputs structured JSON lines (`stdout` and optionally JSONL file)

- Planning (`engine.py`)
- Reads latest vision JSON output
- Captures one RGB + depth-visual snapshot from manual streaming (`payload=96/97`)
- Reads lidar height map via `ef_client.Robot` and computes occupancy summary
- Builds a fused world-state prompt and queries a Hugging Face causal model
- Produces normalized `task.json` for execution

- Execution (`hl_agentic_module.py`)
- Validates `task.json` (`schema_version: "1.0"`)
- Instantiates `ef_client.Robot`
- Runs action list sequentially with per-action status/timing
- Returns execution report as JSON

## Data Flow

1. `manual_streaming` sender provides RGBD-over-UDP.
2. `vision_module.py` turns RGB frames into semantic scene state.
3. `engine.py` fuses:
- latest vision semantics
- RGB/depth visual proxies from manual stream
- lidar occupancy summary from robot
4. `engine.py` prompts HF model and writes `task.json`.
5. `hl_agentic_module.py` executes `task.json` on robot.

## Requirements

Python packages:

- `numpy`
- `opencv-python`
- `torch`
- `open_clip_torch`
- `Pillow`
- `transformers`
- `accelerate` (recommended for HF model runtime)

System packages (GStreamer path):

- `python3-gi`
- `gir1.2-gstreamer-1.0`
- `gir1.2-gst-plugins-base-1.0`
- `gstreamer1.0-plugins-good`
- `gstreamer1.0-plugins-bad`
- `gstreamer1.0-libav`

Robot/runtime dependencies:

- `unitree_sdk2py`
- `dev/ef_client.py` dependencies in this repo
- reachable robot interface (`eth0` or your actual NIC)

## Quick Start

1. Start manual streaming sender on robot/Jetson side (from `sensors/manual_streaming`, usually `jetson_realsense_stream.py`).
2. Run vision semantic stream.
3. Run planner engine to generate `task.json`.
4. Execute task with high-level agentic executor.

## 1) Vision Module

File: `/home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/vision_module.py`

What it outputs:

- `locomotion.label/confidence`
- `dexterity.label/confidence`
- `hazard.label/confidence`
- region blocked confidence (`left/center/right`)
- `suggested_turn_bias`
- `summary_text`

Run:

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/vision_module.py \
  --rgb-port 5600 --width 640 --height 480 --fps 30 \
  --every 6 --alpha 0.35 --show \
  --output-jsonl /tmp/vision_state.jsonl
```

Notes:

- Expects RGB stream format used by `manual_streaming` receiver scripts.
- JSONL last line is used by `engine.py`.

## 2) Planning Engine

File: `/home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/engine.py`

What it does:

- reads latest vision JSON from `--vision-jsonl`
- captures one RGB+depth visual snapshot from UDP ports
- queries robot lidar map and computes occupancy summary
- prompts HF model for task plan
- validates and writes `task.json`

Run:

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/engine.py \
  --prompt "Move safely toward a useful manipulation context" \
  --vision-jsonl /tmp/vision_state.jsonl \
  --iface eth0 --domain-id 0 \
  --rgb-port 5600 --depth-port 5602 --width 640 --height 480 --fps 30 \
  --hf-model TinyLlama/TinyLlama-1.1B-Chat-v1.0 --hf-device cpu \
  --output-task /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/task.json \
  --save-world-state /tmp/world_state.json \
  --allow-fallback --pretty
```

Important depth note:

- Depth on manual stream (`payload=97`) is colorized visualization.
- `engine.py` uses depth-derived proxies, not metric depth.

## 3) High-Level Agentic Executor

File: `/home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/hl_agentic_module.py`

Input methods:

- `--task-file`
- `--task-json`
- `stdin`

Run with file:

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/hl_agentic_module.py \
  --task-file /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/task.json \
  --pretty
```

Print sample schema:

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/hl_agentic_module.py --print-example
```

## Task JSON Schema

Required top-level fields:

- `schema_version`: must be `"1.0"`
- `task_id`: string
- `robot`: object
- `context`: object
- `actions`: non-empty list

Recommended `robot` block:

```json
{
  "iface": "eth0",
  "domain_id": 0,
  "safety_boot": true,
  "auto_start_sensors": true
}
```

Recommended `context` block:

```json
{
  "stop_on_error": true,
  "dry_run": false
}
```

Supported action `type` values:

- `balanced_stand`
- `stop`
- `sleep`
- `walk`
- `run`
- `walk_for`
- `run_for`
- `turn_for`
- `rotate_joint`
- `slam_start`
- `slam_stop`
- `slam_nav_pose`
- `slam_nav_path`
- `say`
- `headlight`
- `get_state`

Minimal valid task example:

```json
{
  "schema_version": "1.0",
  "task_id": "simple_safe_move",
  "robot": {
    "iface": "eth0",
    "domain_id": 0,
    "safety_boot": true,
    "auto_start_sensors": true
  },
  "context": {
    "stop_on_error": true,
    "dry_run": false
  },
  "actions": [
    {"type": "balanced_stand"},
    {"type": "walk_for", "distance": 0.4, "timeout": 10.0},
    {"type": "stop"}
  ]
}
```

## End-to-End Example

Terminal A (vision semantics):

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/vision_module.py \
  --output-jsonl /tmp/vision_state.jsonl
```

Terminal B (plan generation):

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/engine.py \
  --prompt "Take a cautious step forward if clear, otherwise re-orient to safer direction." \
  --vision-jsonl /tmp/vision_state.jsonl \
  --iface eth0 --domain-id 0 \
  --output-task /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/task.json \
  --allow-fallback --pretty
```

Terminal C (execution):

```bash
python3 /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/hl_agentic_module.py \
  --task-file /home/ag/ef_ws/g1/scripts/dev/VLA/stack_1/task.json --pretty
```

## Safety Notes

- Use spotter/harness for first trials.
- Keep `stop` action available in plans when uncertain.
- Start with short motion distances and conservative turn angles.
- Prefer `walk_for` over open-loop velocity commands for controlled movement.
- Validate model outputs before enabling automatic execution loops.

## Troubleshooting

`vision jsonl not found`:

- Start `vision_module.py` with `--output-jsonl`.
- Confirm path passed to `engine.py --vision-jsonl`.

`Timed out waiting for RGBD frames`:

- Verify sender is running.
- Verify ports (`5600`, `5602`) and receiver IP/network.
- Check width/height/fps match stream configuration.

`unitree_sdk2py is not installed`:

- Install Unitree SDK Python package/environment used by this repo.

HF model load or OOM errors:

- Use smaller model.
- Use CPU (`--hf-device cpu`) or lower precision-compatible device.
- Reduce `--max-new-tokens`.

Invalid JSON from model:

- Keep `--allow-fallback` enabled for robust operation.
- Lower temperature.
- Strengthen prompt constraints.

## Known Limitations

- Manual depth stream is colorized, so depth usage in `engine.py` is proxy-only.
- Lidar occupancy summary is coarse and intentionally compact for prompting.
- The planner is only as reliable as the prompt + model + sensor context snapshot.
- `engine.py` currently generates a single-shot plan, not a continuous receding-horizon policy.
