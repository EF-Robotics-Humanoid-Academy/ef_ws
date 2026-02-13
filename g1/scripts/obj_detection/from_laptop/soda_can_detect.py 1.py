import argparse
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np
import torch
from PIL import Image
import open_clip


# -----------------------------
# Data structure
# -----------------------------
@dataclass
class Box:
    x1: int
    y1: int
    x2: int
    y2: int


# -----------------------------
# Sliding windows (fast)
# -----------------------------
def generate_boxes(width: int, height: int) -> List[Box]:
    boxes: List[Box] = []
    min_dim = min(width, height)

    scales = [0.55, 0.7]
    for s in scales:
        size = int(min_dim * s)
        step = max(40, size // 2)
        for y in range(0, height - size + 1, step):
            for x in range(0, width - size + 1, step):
                boxes.append(Box(x, y, x + size, y + size))

    return boxes


# -----------------------------
# CLIP scoring with NEGATIVES
# -----------------------------
def best_clip_box(
    image_bgr: np.ndarray,
    model,
    preprocess,
    text_features: torch.Tensor,
    device: torch.device,
) -> Tuple[float, Box]:

    h, w = image_bgr.shape[:2]
    boxes = generate_boxes(w, h)

    best_margin = -1.0
    best_box = None

    for box in boxes:
        crop = image_bgr[box.y1:box.y2, box.x1:box.x2]
        if crop.size == 0:
            continue

        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(crop_rgb)
        image_input = preprocess(pil).unsqueeze(0).to(device)

        with torch.no_grad():
            img_feat = model.encode_image(image_input)
            img_feat /= img_feat.norm(dim=-1, keepdim=True)

            logits = (100.0 * img_feat @ text_features.T).squeeze(0)
            probs = logits.softmax(dim=-1)

            pos = probs[0].item()          # Red Bull
            neg = probs[1:].max().item()   # strongest negative
            margin = pos - neg             # 🔴 KEY IDEA

        if margin > best_margin:
            best_margin = margin
            best_box = box

    return best_margin, best_box


# -----------------------------
# Main
# -----------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--margin", type=float, default=0.25, help="Positive vs negative margin")
    parser.add_argument("--every", type=int, default=20)
    parser.add_argument("--confirm", type=int, default=3, help="Frames required to confirm detection")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = parser.parse_args()

    device = torch.device(args.device)

    # -----------------------------
    # Load CLIP
    # -----------------------------
    model, _, preprocess = open_clip.create_model_and_transforms(
        "ViT-B-32", pretrained="openai", device=device
    )
    tokenizer = open_clip.get_tokenizer("ViT-B-32")

    # 🔴 STRONG PROMPTS
    texts = [
        "a photo of a Red Bull energy drink can",
        "a photo of a generic soda can",
        "a photo of a red bottle",
        "a photo of a red cup",
        "a photo of background or empty scene",
        "a photo without any drink can",
    ]

    tokens = tokenizer(texts).to(device)
    with torch.no_grad():
        text_features = model.encode_text(tokens)
        text_features /= text_features.norm(dim=-1, keepdim=True)

    # -----------------------------
    # Camera
    # -----------------------------
    cap = cv2.VideoCapture(0, cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    frame_idx = 0
    last_box = None
    confirm_count = 0

    print("Press q to quit")

    # -----------------------------
    # Loop
    # -----------------------------
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        small = cv2.resize(frame, (256, 192))

        if frame_idx % args.every == 0:
            margin, box = best_clip_box(
                small, model, preprocess, text_features, device
            )

            if box is not None and margin >= args.margin:
                confirm_count += 1
                if confirm_count >= args.confirm:
                    sx = frame.shape[1] / 256
                    sy = frame.shape[0] / 192
                    last_box = Box(
                        int(box.x1 * sx),
                        int(box.y1 * sy),
                        int(box.x2 * sx),
                        int(box.x2 * sy),
                    )
            else:
                confirm_count = 0
                last_box = None

        # Draw ONLY when confirmed
        if last_box is not None:
            cv2.rectangle(
                frame,
                (last_box.x1, last_box.y1),
                (last_box.x2, last_box.y2),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                "Red Bull",
                (last_box.x1, max(30, last_box.y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        cv2.imshow("Red Bull Detection (Robust CLIP)", frame)
        frame_idx += 1

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
