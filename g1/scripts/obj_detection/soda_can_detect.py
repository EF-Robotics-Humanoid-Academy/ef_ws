"""
soda_can_detect.py
==================

Capture a single RGB frame from the Unitree G1's head-mounted camera over
the network using the Unitree SDK2 Python library (``unitree_sdk2py``) and
classify it as **"soda can in view"** or **"soda can not in view"** using
OpenAI's CLIP model (zero-shot, no training data required).

The script talks to the robot's ``videohub`` RPC service over DDS, which
means it runs on your **development machine** — not on the robot itself —
as long as the machine is on the same network segment (e.g. connected via
the ``eth0`` interface to the robot's 192.168.123.x subnet).

Usage
-----
::

    python soda_can_detect.py --iface eth0

Dependencies (all in the workspace requirements.txt)::

    unitree_sdk2py   – Unitree SDK2 Python bindings (editable install)
    torch            – PyTorch runtime
    transformers     – HuggingFace (provides CLIPModel / CLIPProcessor)
    opencv-python    – JPEG decoding and image display
    numpy            – array operations

Author: Generated script
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Unitree SDK2 imports
# ---------------------------------------------------------------------------
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
except ImportError as exc:
    raise SystemExit(
        "unitree_sdk2py is not installed.  Install it with:\n"
        "  pip install -e <path-to-unitree_sdk2_python>\n"
        "See: https://github.com/unitreerobotics/unitree_sdk2_python"
    ) from exc

# ---------------------------------------------------------------------------
# CLIP imports
# ---------------------------------------------------------------------------
try:
    import torch
except ImportError as exc:
    raise SystemExit("torch is not installed.  pip install torch") from exc

try:
    from transformers import CLIPModel, CLIPProcessor  # type: ignore
except ImportError as exc:
    raise SystemExit("transformers is not installed.  pip install transformers") from exc


# ---------------------------------------------------------------------------
# Camera capture via Unitree SDK2
# ---------------------------------------------------------------------------

def capture_frame(iface: str, timeout: float = 3.0) -> np.ndarray:
    """Connect to the G1 over *iface*, grab one JPEG frame, return BGR array.

    The function initialises the DDS channel, creates a ``VideoClient``,
    calls ``GetImageSample()`` which returns raw JPEG bytes, and decodes
    them into an OpenCV BGR image.

    Args:
        iface: Network interface connected to the robot (e.g. ``"eth0"``).
        timeout: RPC timeout in seconds (default 3.0).

    Returns:
        A numpy array of shape ``(H, W, 3)`` in BGR colour order.

    Raises:
        RuntimeError: If the SDK cannot reach the robot or the image
            request fails.
    """
    # Domain ID 0 = real robot (1 = simulation)
    ChannelFactoryInitialize(0, iface)

    client = VideoClient()
    client.SetTimeout(timeout)
    client.Init()

    code, data = client.GetImageSample()

    if code != 0:
        raise RuntimeError(
            f"VideoClient.GetImageSample() failed with error code {code}.\n"
            "  - Is the robot powered on and connected via the correct interface?\n"
            "  - Is the videohub service running on the robot?"
        )

    # `data` is a list of byte values; decode JPEG -> BGR numpy array
    jpeg_bytes = np.frombuffer(bytes(data), dtype=np.uint8)
    frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

    if frame is None:
        raise RuntimeError(
            "cv2.imdecode returned None — the data from the robot may not be "
            "valid JPEG.  Received %d bytes." % len(data)
        )

    return frame


# ---------------------------------------------------------------------------
# CLIP zero-shot classification
# ---------------------------------------------------------------------------

CANDIDATE_LABELS = ["a soda can", "no soda can, an empty scene"]


def load_clip(
    model_name: str = "openai/clip-vit-base-patch32",
    device: str | None = None,
) -> tuple[CLIPModel, CLIPProcessor, str]:
    """Download (first run) and load the CLIP model + processor.

    Args:
        model_name: HuggingFace model identifier.
        device: ``"cuda"`` or ``"cpu"``.  Auto-detected if *None*.

    Returns:
        ``(model, processor, device_str)``
    """
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"

    print(f"Loading CLIP model '{model_name}' on '{device}' …")
    processor = CLIPProcessor.from_pretrained(model_name)
    model = CLIPModel.from_pretrained(model_name).to(device)
    model.eval()
    print("CLIP model loaded.")
    return model, processor, device


def classify_frame(
    frame: np.ndarray,
    model: CLIPModel,
    processor: CLIPProcessor,
    device: str,
    threshold: float = 0.6,
) -> dict[str, Any]:
    """Run CLIP zero-shot classification on a BGR image.

    Args:
        frame: ``(H, W, 3)`` BGR numpy array.
        model: Loaded CLIPModel.
        processor: Loaded CLIPProcessor.
        device: ``"cuda"`` or ``"cpu"``.
        threshold: Minimum probability for ``"a soda can"`` to count as
            detected (default 0.6).

    Returns:
        A dict with keys ``detected`` (bool), ``confidence`` (float 0-1),
        ``label`` (str), and ``scores`` (dict of label -> float).
    """
    # BGR -> RGB
    rgb = np.ascontiguousarray(frame[..., ::-1])

    with torch.no_grad():
        inputs = processor(
            text=CANDIDATE_LABELS,
            images=rgb,
            return_tensors="pt",
            padding=True,
        )
        inputs = {k: v.to(device) for k, v in inputs.items()}

        outputs = model(**inputs)
        probs = outputs.logits_per_image.softmax(dim=1).cpu().numpy()[0]

    scores = {label: float(p) for label, p in zip(CANDIDATE_LABELS, probs)}
    soda_score = scores[CANDIDATE_LABELS[0]]
    best_idx = int(np.argmax(probs))

    return {
        "detected": soda_score >= threshold,
        "confidence": soda_score,
        "label": CANDIDATE_LABELS[best_idx],
        "scores": scores,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture one frame from the G1 camera over the network and "
                    "classify it for soda-can presence using CLIP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--iface",
        type=str,
        default="eth0",
        help="Network interface connected to the robot",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.6,
        help="Confidence threshold for positive detection (0-1)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="RPC timeout in seconds for the VideoClient",
    )
    parser.add_argument(
        "--save",
        type=str,
        default=None,
        help="Optional path to save the annotated snapshot (e.g. result.jpg)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the annotated image in an OpenCV window",
    )
    args = parser.parse_args()

    if not (0 <= args.threshold <= 1):
        sys.exit("Error: --threshold must be in range [0, 1]")

    # ---- 1. capture -------------------------------------------------------
    print(f"Connecting to G1 via interface '{args.iface}' …")
    frame = capture_frame(args.iface, timeout=args.timeout)
    print(f"Captured frame: {frame.shape[1]}x{frame.shape[0]} pixels")

    # ---- 2. classify ------------------------------------------------------
    model, processor, device = load_clip()
    result = classify_frame(frame, model, processor, device, threshold=args.threshold)

    # ---- 3. report --------------------------------------------------------
    print()
    print("Classification results:")
    print(f"  Detected  : {result['detected']}")
    print(f"  Confidence: {result['confidence']:.1%}")
    print(f"  Label     : {result['label']}")
    for label, score in result["scores"].items():
        print(f"  Score for '{label}': {score:.1%}")

    # ---- 4. annotate frame ------------------------------------------------
    status = "DETECTED" if result["detected"] else "NOT DETECTED"
    colour = (0, 255, 0) if result["detected"] else (0, 0, 255)

    cv2.putText(frame, f"Soda can: {status}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)
    cv2.putText(frame, f"Confidence: {result['confidence']:.1%}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

    # ---- 5. save / show ---------------------------------------------------
    if args.save:
        cv2.imwrite(args.save, frame)
        print(f"\nAnnotated image saved to: {args.save}")

    if args.show:
        cv2.imshow("Soda Can Detection (CLIP)", frame)
        print("Press any key to close the window …")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    if not args.save and not args.show:
        print("\nTip: use --save result.jpg or --show to see the annotated image.")


if __name__ == "__main__":
    main()
