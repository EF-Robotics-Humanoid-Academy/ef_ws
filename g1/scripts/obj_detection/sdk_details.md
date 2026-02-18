# SDK Details: CLIP Object Detection + Unitree RGB-D Access

This document explains how to implement CLIP-based object detection and how to access RGB-D camera data in a Unitree setup.

## 1. Practical Architecture

Use this two-part architecture:

1. Camera ingestion.
2. CLIP inference and decision logic.

In this workspace there are two camera ingestion modes:

1. `unitree_sdk2py` `VideoClient` for network RGB snapshots from the robot (`soda_can_detect.py`).
2. `pyrealsense2` for local USB RealSense RGB-D streaming (`clip_soda_can_detect 1.py`, `soda_can_detect.py.py`).

Important constraint:

- `VideoClient.GetImageSample()` returns JPEG RGB data only.
- For synchronized color + depth (`RGB-D`), use the RealSense pipeline (`pyrealsense2`) on the machine that has direct USB access to the depth camera.

## 2. Environment Setup

Install dependencies (adapt to your environment):

```bash
pip install -U numpy opencv-python torch torchvision transformers open-clip-torch pillow pyrealsense2
pip install -e /path/to/unitree_sdk2_python
```

Network prerequisites for SDK2 video RPC:

1. Robot and host are on the same subnet (commonly `192.168.123.x`).
2. Correct interface passed to `ChannelFactoryInitialize(0, iface)` (for example `eth0`).
3. `videohub` service is running on the robot.

## 3. CLIP Detection Design

CLIP is not a detector by default. It gives image-text similarity. To make it do object detection you need a localization strategy.

Recommended strategy for this project:

1. Generate candidate regions (sliding windows or proposals).
2. Score each region with CLIP prompts.
3. Select highest scoring region.
4. Threshold score for object-present decision.

Prompts should include:

1. Positive class prompt.
2. Negative/background prompt.

Example prompts:

1. `"a photo of a soda can"`
2. `"a photo without a soda can"`

## 4. CLIP Detection Implementation (Code)

Use this pattern for robust implementation.

```python
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np
import torch
from PIL import Image
import open_clip


@dataclass
class Box:
    x1: int
    y1: int
    x2: int
    y2: int


def generate_boxes(width: int, height: int, step_frac: float = 0.25) -> List[Box]:
    boxes: List[Box] = []
    min_dim = min(width, height)
    for scale in [0.6, 0.8, 1.0]:
        size = int(min_dim * scale)
        if size <= 0:
            continue
        step = max(12, int(size * step_frac))
        for y in range(0, height - size + 1, step):
            for x in range(0, width - size + 1, step):
                boxes.append(Box(x, y, x + size, y + size))
    return boxes or [Box(0, 0, width, height)]


def build_clip(device: str = "cuda"):
    model, _, preprocess = open_clip.create_model_and_transforms(
        "ViT-B-32", pretrained="openai", device=device
    )
    model.eval()
    tokenizer = open_clip.get_tokenizer("ViT-B-32")
    prompts = ["a photo of a soda can", "a photo without a soda can"]
    text_tokens = tokenizer(prompts).to(device)
    with torch.inference_mode():
        text_features = model.encode_text(text_tokens)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)
    return model, preprocess, text_features


def best_clip_box(image_bgr: np.ndarray, model, preprocess, text_features, device: str):
    h, w = image_bgr.shape[:2]
    boxes = generate_boxes(w, h, step_frac=0.25)

    best_score = -1.0
    best_box = boxes[0]

    for box in boxes:
        crop = image_bgr[box.y1:box.y2, box.x1:box.x2]
        if crop.size == 0:
            continue
        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        image_input = preprocess(Image.fromarray(crop_rgb)).unsqueeze(0).to(device)

        with torch.inference_mode():
            image_features = model.encode_image(image_input)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            probs = (100.0 * image_features @ text_features.T).softmax(dim=-1)
            positive_score = float(probs[0, 0].item())

        if positive_score > best_score:
            best_score = positive_score
            best_box = box

    return best_score, best_box
```

Inference loop recommendations:

1. Run CLIP every `N` frames (for example every 5 to 10 frames).
2. Downscale image before scoring to reduce latency.
3. Reproject resulting box to original resolution.
4. Keep `last_box` and `last_score` between updates.

## 5. RGB via Unitree SDK2 (`VideoClient`)

Use this when your app runs on a remote dev machine and you only need RGB snapshots.

```python
import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


def capture_rgb_snapshot(iface: str = "eth0", timeout: float = 3.0) -> np.ndarray:
    ChannelFactoryInitialize(0, iface)  # domain 0 = real robot

    client = VideoClient()
    client.SetTimeout(timeout)
    client.Init()

    code, data = client.GetImageSample()
    if code != 0:
        raise RuntimeError(f"GetImageSample failed with code {code}")

    jpeg_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
    frame_bgr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
    if frame_bgr is None:
        raise RuntimeError("Failed to decode JPEG from VideoClient")

    return frame_bgr
```

Operational notes:

1. This is RPC snapshot behavior, not high-rate streaming.
2. Data format is JPEG-compressed RGB image.
3. Depth is not returned by `GetImageSample()`.

## 6. RGB-D via RealSense (`pyrealsense2`)

Use this when you need synchronized color + depth frames.

```python
import cv2
import numpy as np
import pyrealsense2 as rs


def start_rgbd_pipeline(color_w=1280, color_h=720, depth_w=640, depth_h=480, fps=30):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)
    profile = pipeline.start(config)

    align = rs.align(rs.stream.color)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # meters per depth unit

    return pipeline, align, depth_scale


def get_rgbd_frame(pipeline, align):
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)

    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None

    color_bgr = np.asanyarray(color_frame.get_data())
    depth_raw = np.asanyarray(depth_frame.get_data())  # uint16
    return color_bgr, depth_raw


def depth_at_pixel_m(depth_raw: np.ndarray, x: int, y: int, depth_scale: float) -> float:
    z = int(depth_raw[y, x])
    return z * depth_scale
```

Depth handling notes:

1. `depth_raw` is typically `uint16`.
2. Convert to meters using `depth_scale`.
3. Use an aligned depth frame before sampling depth at CLIP box center.

## 7. Combine CLIP + Depth for 3D-Aware Detection

After CLIP returns best box:

1. Compute center pixel.
2. Read depth at center.
3. Reject detection if depth invalid (0 or out of range).
4. Annotate class probability and distance.

Example:

```python
def annotate_detection(frame_bgr, depth_raw, depth_scale, box, score, threshold=0.6):
    color = (0, 255, 0) if score >= threshold else (0, 0, 255)
    cv2.rectangle(frame_bgr, (box.x1, box.y1), (box.x2, box.y2), color, 2)

    cx = (box.x1 + box.x2) // 2
    cy = (box.y1 + box.y2) // 2
    dist_m = float(depth_raw[cy, cx]) * depth_scale

    text = f"score={score:.2f}, z={dist_m:.2f}m"
    cv2.putText(frame_bgr, text, (box.x1, max(20, box.y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return frame_bgr, dist_m
```

## 8. Performance and Reliability Checklist

1. Use GPU (`cuda`) when available for CLIP inference.
2. Run CLIP every `N` frames and track intermediate frames with last known box.
3. Resize input for CLIP (`0.4` to `0.6` scale is usually a good trade-off).
4. Use prompt engineering for your object class variants.
5. Add temporal smoothing on score and depth to reduce jitter.
6. Handle SDK errors explicitly (`code != 0`) and retry with backoff.
7. Fail fast when no device is found (`rs.context().query_devices()`).

## 9. Typical Failure Modes

1. `GetImageSample` non-zero code.
Cause: network/interface mismatch, robot service unavailable, timeout too low.

2. `cv2.imdecode` returns `None`.
Cause: corrupted or empty JPEG bytes.

3. No RealSense device detected.
Cause: camera not connected to the executing host or USB permissions issue.

4. False positives from CLIP.
Cause: weak prompts or insufficient negative prompts.

5. Unstable depth values.
Cause: reflective surfaces, edge pixels, or unaligned frames.

## 10. Recommended Project Structure

```text
obj_detection/
  sdk_details.md
  soda_can_detect.py               # SDK2 RGB snapshot + CLIP classification
  clip_soda_can_detect 1.py        # RealSense RGB-D + CLIP localized detection
  berxel_rgbd_test.py              # basic camera read test
```

If you want one production-ready script, merge the RealSense RGB-D acquisition path with the CLIP localization path and keep SDK2 `VideoClient` as fallback when running remotely.
