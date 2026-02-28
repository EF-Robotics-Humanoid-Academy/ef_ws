#!/usr/bin/env python3
"""
vision_module.py

Consumes the RGB stream from sensors/manual_streaming (UDP/H264 over GStreamer),
runs CLIP classification, and emits high-level visual state information suitable
for downstream agentic decision modules (locomotion + dexterity context).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import torch
from PIL import Image

import open_clip

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp


@dataclass
class VisionState:
    timestamp: float
    locomotion_label: str
    locomotion_confidence: float
    dexterity_label: str
    dexterity_confidence: float
    hazard_label: str
    hazard_confidence: float
    forward_progress_confidence: float
    left_blocked_confidence: float
    center_blocked_confidence: float
    right_blocked_confidence: float
    suggested_turn_bias: str
    summary_text: str

    def to_dict(self) -> Dict[str, object]:
        return {
            "timestamp": self.timestamp,
            "locomotion": {
                "label": self.locomotion_label,
                "confidence": self.locomotion_confidence,
                "forward_progress_confidence": self.forward_progress_confidence,
                "region_blocked_confidence": {
                    "left": self.left_blocked_confidence,
                    "center": self.center_blocked_confidence,
                    "right": self.right_blocked_confidence,
                },
                "suggested_turn_bias": self.suggested_turn_bias,
            },
            "dexterity": {
                "label": self.dexterity_label,
                "confidence": self.dexterity_confidence,
            },
            "hazard": {
                "label": self.hazard_label,
                "confidence": self.hazard_confidence,
            },
            "summary_text": self.summary_text,
        }


class EMASmoother:
    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        self._state: np.ndarray | None = None

    def update(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32)
        if self._state is None:
            self._state = x.copy()
        else:
            self._state = self.alpha * x + (1.0 - self.alpha) * self._state
        return self._state


def build_rgb_sink(port: int) -> tuple[GstApp.AppSink, Gst.Pipeline]:
    pipeline = Gst.parse_launch(
        f"udpsrc port={port} caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=true sync=false drop=true"
    )
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("Failed to create RGB appsink")
    return sink, pipeline


def sample_to_bgr(sample: Gst.Sample, width: int, height: int) -> np.ndarray:
    buf = sample.get_buffer()
    raw = np.frombuffer(buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
    expected = width * height * 3
    if raw.size != expected:
        raise RuntimeError(
            f"Unexpected RGB buffer size: got {raw.size}, expected {expected} "
            f"for frame {width}x{height}"
        )
    return raw.reshape((height, width, 3)).copy()


def l2_normalize(x: torch.Tensor) -> torch.Tensor:
    return x / x.norm(dim=-1, keepdim=True)


def encode_text(model, tokenizer, device: torch.device, prompts: Sequence[str]) -> torch.Tensor:
    tokens = tokenizer(list(prompts)).to(device)
    with torch.inference_mode():
        feats = model.encode_text(tokens)
    return l2_normalize(feats)


def clip_probs_for_image(
    image_bgr: np.ndarray,
    model,
    preprocess,
    text_features: torch.Tensor,
    device: torch.device,
) -> np.ndarray:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(image_rgb)
    image_input = preprocess(pil).unsqueeze(0).to(device)
    with torch.inference_mode():
        image_features = model.encode_image(image_input)
        image_features = l2_normalize(image_features)
        logits = 100.0 * image_features @ text_features.T
        probs = logits.softmax(dim=-1)[0]
    return probs.detach().float().cpu().numpy()


def top_label(labels: Sequence[str], probs: np.ndarray) -> tuple[str, float, int]:
    idx = int(np.argmax(probs))
    return labels[idx], float(probs[idx]), idx


def blocked_confidence(labels: Sequence[str], probs: np.ndarray) -> float:
    blocked_names = {
        "obstacle directly ahead",
        "narrow passage",
        "dense cluttered area",
        "stairs going down",
        "wet or slippery floor",
    }
    total = 0.0
    for i, name in enumerate(labels):
        if name in blocked_names:
            total += float(probs[i])
    return min(1.0, max(0.0, total))


def summarize_state(
    locomotion_label: str,
    locomotion_conf: float,
    dex_label: str,
    dex_conf: float,
    hazard_label: str,
    hazard_conf: float,
    forward_progress_conf: float,
    left_blocked: float,
    center_blocked: float,
    right_blocked: float,
) -> tuple[str, str]:
    if center_blocked > 0.6:
        if left_blocked + 0.08 < right_blocked:
            turn = "left"
        elif right_blocked + 0.08 < left_blocked:
            turn = "right"
        else:
            turn = "hold"
    else:
        turn = "forward"

    summary = (
        f"Scene={locomotion_label} ({locomotion_conf:.2f}), "
        f"hazard={hazard_label} ({hazard_conf:.2f}), "
        f"manipulation={dex_label} ({dex_conf:.2f}), "
        f"forward={forward_progress_conf:.2f}, turn_bias={turn}."
    )
    return turn, summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CLIP vision module on manual_streaming RGB feed",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--rgb-port", type=int, default=5600, help="RGB UDP port from manual_streaming sender")
    parser.add_argument("--width", type=int, default=640, help="RGB width")
    parser.add_argument("--height", type=int, default=480, help="RGB height")
    parser.add_argument("--fps", type=int, default=30, help="Expected input FPS")
    parser.add_argument("--every", type=int, default=6, help="Run CLIP every N frames")
    parser.add_argument("--alpha", type=float, default=0.35, help="EMA smoothing factor for CLIP probabilities")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--model", default="ViT-B-32", help="open_clip model name")
    parser.add_argument("--pretrained", default="openai", help="open_clip pretrained tag")
    parser.add_argument("--show", action="store_true", help="Show live annotated RGB window")
    parser.add_argument("--output-jsonl", default="", help="Optional JSONL output file path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    locomotion_labels = [
        "clear open walkway",
        "obstacle directly ahead",
        "narrow passage",
        "stairs going up",
        "stairs going down",
        "dense cluttered area",
        "tabletop workstation scene",
        "doorway or room transition",
    ]
    dexterity_labels = [
        "no immediate manipulation target",
        "small graspable object",
        "tool-like object",
        "container or cup",
        "drawer or door handle",
        "buttons or control panel",
        "text label or screen",
    ]
    hazard_labels = [
        "no significant hazard",
        "human nearby",
        "fragile object nearby",
        "wet or slippery floor",
        "drop or edge nearby",
    ]

    Gst.init(None)

    device = torch.device(args.device)
    print(f"[vision] loading CLIP model={args.model} pretrained={args.pretrained} on {device}")
    model, _, preprocess = open_clip.create_model_and_transforms(
        args.model,
        pretrained=args.pretrained,
        device=device,
    )
    model.eval()
    tokenizer = open_clip.get_tokenizer(args.model)

    loco_text = encode_text(model, tokenizer, device, locomotion_labels)
    dex_text = encode_text(model, tokenizer, device, dexterity_labels)
    hazard_text = encode_text(model, tokenizer, device, hazard_labels)

    loco_smoother = EMASmoother(args.alpha)
    dex_smoother = EMASmoother(args.alpha)
    hazard_smoother = EMASmoother(args.alpha)

    rgb_sink, rgb_pipeline = build_rgb_sink(args.rgb_port)
    rgb_pipeline.set_state(Gst.State.PLAYING)

    out_f = None
    if args.output_jsonl:
        out_path = Path(args.output_jsonl).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_f = out_path.open("a", encoding="utf-8")
        print(f"[vision] writing state JSONL to {out_path}")

    last_state: VisionState | None = None
    frame_idx = 0

    try:
        while True:
            sample = rgb_sink.emit("try-pull-sample", Gst.SECOND // max(1, args.fps))
            if not sample:
                time.sleep(0.003)
                continue

            frame = sample_to_bgr(sample, args.width, args.height)

            if frame_idx % max(1, args.every) == 0:
                loco_probs = loco_smoother.update(clip_probs_for_image(frame, model, preprocess, loco_text, device))
                dex_probs = dex_smoother.update(clip_probs_for_image(frame, model, preprocess, dex_text, device))
                hazard_probs = hazard_smoother.update(clip_probs_for_image(frame, model, preprocess, hazard_text, device))

                h, w = frame.shape[:2]
                left = frame[:, : w // 3]
                center = frame[:, w // 3 : 2 * w // 3]
                right = frame[:, 2 * w // 3 :]

                left_blocked = blocked_confidence(locomotion_labels, clip_probs_for_image(left, model, preprocess, loco_text, device))
                center_blocked = blocked_confidence(
                    locomotion_labels,
                    clip_probs_for_image(center, model, preprocess, loco_text, device),
                )
                right_blocked = blocked_confidence(
                    locomotion_labels,
                    clip_probs_for_image(right, model, preprocess, loco_text, device),
                )

                locomotion_label, locomotion_conf, _ = top_label(locomotion_labels, loco_probs)
                dex_label, dex_conf, _ = top_label(dexterity_labels, dex_probs)
                hazard_label, hazard_conf, _ = top_label(hazard_labels, hazard_probs)

                forward_progress_conf = float(
                    max(0.0, min(1.0, loco_probs[0] - 0.45 * center_blocked - 0.20 * hazard_probs[4]))
                )

                turn_bias, summary = summarize_state(
                    locomotion_label,
                    locomotion_conf,
                    dex_label,
                    dex_conf,
                    hazard_label,
                    hazard_conf,
                    forward_progress_conf,
                    left_blocked,
                    center_blocked,
                    right_blocked,
                )

                last_state = VisionState(
                    timestamp=time.time(),
                    locomotion_label=locomotion_label,
                    locomotion_confidence=locomotion_conf,
                    dexterity_label=dex_label,
                    dexterity_confidence=dex_conf,
                    hazard_label=hazard_label,
                    hazard_confidence=hazard_conf,
                    forward_progress_confidence=forward_progress_conf,
                    left_blocked_confidence=left_blocked,
                    center_blocked_confidence=center_blocked,
                    right_blocked_confidence=right_blocked,
                    suggested_turn_bias=turn_bias,
                    summary_text=summary,
                )

                payload = last_state.to_dict()
                print(json.dumps(payload, ensure_ascii=True))
                if out_f is not None:
                    out_f.write(json.dumps(payload, ensure_ascii=True) + "\n")
                    out_f.flush()

            if args.show and last_state is not None:
                overlay = frame.copy()
                cv2.putText(
                    overlay,
                    f"Loco: {last_state.locomotion_label} ({last_state.locomotion_confidence:.2f})",
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                cv2.putText(
                    overlay,
                    f"Dex: {last_state.dexterity_label} ({last_state.dexterity_confidence:.2f})",
                    (10, 54),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                cv2.putText(
                    overlay,
                    f"Haz: {last_state.hazard_label} ({last_state.hazard_confidence:.2f})",
                    (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                cv2.putText(
                    overlay,
                    f"Forward={last_state.forward_progress_confidence:.2f} Turn={last_state.suggested_turn_bias}",
                    (10, 106),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )

                cv2.imshow("VLA Vision Module (manual_streaming RGB)", overlay)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            frame_idx += 1
    finally:
        rgb_pipeline.set_state(Gst.State.NULL)
        if out_f is not None:
            out_f.close()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[vision] interrupted by user")
    except Exception as exc:
        print(f"[vision] error: {exc}", file=sys.stderr)
        sys.exit(1)
