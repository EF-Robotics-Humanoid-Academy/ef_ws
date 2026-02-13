import argparse
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np
import torch
from PIL import Image

import open_clip
import pyrealsense2 as rs


@dataclass
class Box:
    x1: int
    y1: int
    x2: int
    y2: int


def _generate_boxes(width: int, height: int) -> List[Box]:
    boxes: List[Box] = []
    min_dim = min(width, height)
    scales = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    for s in scales:
        size = int(min_dim * s)
        if size <= 0:
            continue
        step = max(8, size // 4)
        for y in range(0, height - size + 1, step):
            for x in range(0, width - size + 1, step):
                boxes.append(Box(x, y, x + size, y + size))
    if not boxes:
        boxes.append(Box(0, 0, width, height))
    return boxes


def _best_clip_box(
    image_bgr: np.ndarray,
    model,
    preprocess,
    text_features: torch.Tensor,
    device: torch.device,
) -> Tuple[float, Box]:
    h, w = image_bgr.shape[:2]
    boxes = _generate_boxes(w, h)
    best_score = -1.0
    best_box = boxes[0]

    for box in boxes:
        crop = image_bgr[box.y1:box.y2, box.x1:box.x2]
        if crop.size == 0:
            continue
        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(crop_rgb)
        image_input = preprocess(pil).unsqueeze(0).to(device)
        with torch.no_grad():
            image_features = model.encode_image(image_input)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            logits = (100.0 * image_features @ text_features.T).softmax(dim=-1)
            soda_prob = float(logits[0, 0].item())
        if soda_prob > best_score:
            best_score = soda_prob
            best_box = box

    return best_score, best_box


def _normalize_depth(depth: np.ndarray, max_depth: int) -> np.ndarray:
    clipped = np.clip(depth, 0, max_depth).astype(np.float32)
    scaled = (clipped / max_depth * 255.0).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)


def main() -> int:
    parser = argparse.ArgumentParser(description="CLIP soda can detector (RealSense RGB + depth).")
    parser.add_argument("--threshold", type=float, default=0.6, help="Detection threshold (0-1)")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--every", type=int, default=5, help="Run CLIP every N frames")
    parser.add_argument("--max-depth", type=int, default=4000, help="Max depth (uint16 units) for visualization")
    parser.add_argument("--color-width", type=int, default=1280)
    parser.add_argument("--color-height", type=int, default=720)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    device = torch.device(args.device)
    model, _, preprocess = open_clip.create_model_and_transforms(
        "ViT-B-32", pretrained="openai", device=device
    )
    tokenizer = open_clip.get_tokenizer("ViT-B-32")
    texts = ["a photo of a soda can", "a photo without a soda can"]
    text_tokens = tokenizer(texts).to(device)
    with torch.no_grad():
        text_features = model.encode_text(text_tokens)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)

    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense devices detected.")
        return 2

    pipeline = rs.pipeline()
    config = rs.config()

    def try_start(color_w, color_h, depth_w, depth_h, fps):
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)
        return pipeline.start(cfg)

    try:
        profile = try_start(args.color_width, args.color_height, args.depth_width, args.depth_height, args.fps)
    except Exception:
        # Fallback: pick the first supported color/depth profiles
        device = devices[0]
        color_profile = None
        depth_profile = None
        for sensor in device.sensors:
            for p in sensor.get_stream_profiles():
                if p.stream_type() == rs.stream.color and color_profile is None:
                    vp = p.as_video_stream_profile()
                    if vp.format() == rs.format.bgr8:
                        color_profile = vp
                if p.stream_type() == rs.stream.depth and depth_profile is None:
                    vp = p.as_video_stream_profile()
                    if vp.format() == rs.format.z16:
                        depth_profile = vp
            if color_profile is not None and depth_profile is not None:
                break
        if color_profile is None or depth_profile is None:
            print("Failed to find supported color/depth profiles on the device.")
            return 3

        cfg = rs.config()
        cfg.enable_stream(
            rs.stream.color,
            color_profile.width(),
            color_profile.height(),
            color_profile.format(),
            int(color_profile.fps()),
        )
        cfg.enable_stream(
            rs.stream.depth,
            depth_profile.width(),
            depth_profile.height(),
            depth_profile.format(),
            int(depth_profile.fps()),
        )
        profile = pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    last_score = -1.0
    last_box = None
    frame_idx = 0

    print("Press q to quit.")
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            image_bgr = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth_vis = _normalize_depth(depth, args.max_depth)

            if frame_idx % max(1, args.every) == 0:
                score, box = _best_clip_box(image_bgr, model, preprocess, text_features, device)
                last_score = score
                last_box = box

            if last_box is not None:
                label = f"soda can prob: {last_score:.2f}"
                if last_score >= args.threshold:
                    cv2.rectangle(
                        image_bgr, (last_box.x1, last_box.y1), (last_box.x2, last_box.y2), (0, 255, 0), 2
                    )
                    cv2.putText(
                        image_bgr,
                        label,
                        (last_box.x1, max(20, last_box.y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                else:
                    cv2.putText(
                        image_bgr,
                        f"no soda can (best {label})",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )

            cv2.imshow("CLIP Soda Can Detection", image_bgr)
            cv2.imshow("Depth", depth_vis)

            frame_idx += 1
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
