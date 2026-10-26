"""OpenAI-backed open-vocabulary detector for the dashboard's grab/sort pipeline.

Drop-in replacement for hand_pose_navigation.vision_detector.VisionDetector:
same `available` / `error` / `set_prompt()` / `classes` / `detect()` surface,
returning the same VisionDetection boxes.  Nothing is loaded locally (YOLO-World
is slow to load on the Jetson and re-encodes the class list on every prompt
change), so a prompt change only goes into the next request.

GPT models name objects well but place boxes poorly (tens of pixels off on the
desk camera), and the grasp pose comes from the depth inside the box.  So the
geometry comes from depth instead: the caller passes `proposals`, the boxes of
everything standing on the fitted table plane (GraspPipeline.object_proposals).
They are drawn on the frame as numbered marks and the model only says which
mark is which class ("set-of-marks" prompting).  Boxes are therefore exactly
where the depth camera sees the objects.  Without proposals (no table plane in
view) the model's own boxes are used, in 0-1000 normalized coordinates, which
it localizes better than raw pixels.

Each detection is a network round trip (about 1.5-3 s with gpt-4.1) and costs
API tokens while the /sort view is open.  Uses `requests` against the
chat-completions endpoint (as llm_client does), so the `openai` package is not
needed.  Requires OPENAI_API_KEY.
"""
from __future__ import annotations

import base64
import json
import os
import threading
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import requests

from hand_pose_navigation.vision_detector import VisionDetection, parse_prompt_to_classes

DEFAULT_OPENAI_VISION_MODEL = os.environ.get("G1_OPENAI_VISION_MODEL", "gpt-4.1")
DEFAULT_OPENAI_BASE = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1")
REQUEST_TIMEOUT_S = 20.0
NO_MATCH = "none"
MARK_COLOR_BGR = (0, 255, 255)

_MARKS_PROMPT = (
    "You label objects for a robot arm that grasps things on a table. Every candidate object in the image is "
    "outlined and tagged with a number. For each tagged candidate, give the class it shows from the list, or "
    f"'{NO_MATCH}' when it is none of them (or only part of something else, like a hand, arm or cable). "
    "Judge by what is inside the outline, not by nearby objects. confidence is your certainty from 0 to 1."
)
_BOXES_PROMPT = (
    "You are the object detector of a robot arm that grasps objects on a table. Find every visible instance "
    "of the requested classes and return a tight bounding box around each, with coordinates normalized to "
    "0-1000 (ymin, xmin, ymax, xmax). Return an empty list when none are present. confidence is your "
    "certainty from 0 to 1."
)

Box = Tuple[int, int, int, int]


class OpenAIVisionDetector:
    """NL-prompted 2-D detector over the OpenAI chat-completions API."""

    # GraspPipeline passes depth-based candidate boxes to detect().
    uses_proposals = True

    def __init__(self, model_name: str = DEFAULT_OPENAI_VISION_MODEL, conf: float = 0.15,
                 base_url: str = DEFAULT_OPENAI_BASE, api_key_env: str = "OPENAI_API_KEY") -> None:
        self.model_name = model_name
        self.conf = conf
        self.base_url = base_url.rstrip("/")
        self.available = False
        self.error = ""
        self._classes: List[str] = parse_prompt_to_classes("")
        self._lock = threading.RLock()
        self._session = requests.Session()
        if not os.environ.get(api_key_env):
            try:
                import llm_client.secrets  # noqa: F401  (side effect: fills os.environ)
            except Exception:
                pass
        self._api_key = os.environ.get(api_key_env, "")
        if not self._api_key:
            self.error = f"{api_key_env} is not set; OpenAI vision is disabled"
            return
        self.available = True

    def set_prompt(self, text: str) -> List[str]:
        classes = parse_prompt_to_classes(text)
        with self._lock:
            self._classes = classes
        return classes

    @property
    def classes(self) -> List[str]:
        return list(self._classes)

    # -- request -------------------------------------------------------------

    def _ask(self, system: str, text: str, image: np.ndarray, schema: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        ok, jpeg = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            self.error = "JPEG encoding failed"
            return None
        body = {
            "model": self.model_name,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": [
                    {"type": "text", "text": text},
                    {"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64," + base64.b64encode(jpeg.tobytes()).decode("ascii"),
                        "detail": "high"}},
                ]},
            ],
            "response_format": {"type": "json_schema", "json_schema": {"name": "detections", "strict": True,
                                                                       "schema": schema}},
        }
        try:
            resp = self._session.post(f"{self.base_url}/chat/completions", json=body, timeout=REQUEST_TIMEOUT_S,
                                      headers={"Authorization": f"Bearer {self._api_key}"})
            if resp.status_code != 200:
                self.error = f"OpenAI vision HTTP {resp.status_code}: {resp.text[:200]}"
                return None
            message = resp.json()["choices"][0]["message"]
            if message.get("refusal"):
                self.error = f"OpenAI vision refused: {message['refusal']}"
                return None
            result = json.loads(message.get("content") or "{}")
        except Exception as exc:
            self.error = f"OpenAI vision request failed: {exc}"
            return None
        self.error = ""
        return result

    @staticmethod
    def _object_list_schema(item_props: Dict[str, Any]) -> Dict[str, Any]:
        item = {"type": "object", "additionalProperties": False, "required": list(item_props),
                "properties": item_props}
        return {"type": "object", "additionalProperties": False, "required": ["objects"],
                "properties": {"objects": {"type": "array", "items": item}}}

    # -- detection -----------------------------------------------------------

    def detect(self, rgb_bgr: np.ndarray, proposals: Optional[Sequence[Box]] = None) -> List[VisionDetection]:
        """Detect the prompted classes in one BGR frame. Returns [] on error.

        With `proposals` (may be empty: nothing stands on the table) the model
        labels those boxes; with None it places its own boxes.
        """
        if not self.available:
            return []
        with self._lock:
            classes = list(self._classes)
        if proposals is None:
            return self._detect_boxes(rgb_bgr, classes)
        return self._label_proposals(rgb_bgr, classes, list(proposals))

    def _label_proposals(self, rgb_bgr: np.ndarray, classes: List[str], proposals: List[Box]) -> List[VisionDetection]:
        if not proposals:
            self.error = ""
            return []
        marked = rgb_bgr.copy()
        for i, (x1, y1, x2, y2) in enumerate(proposals, start=1):
            cv2.rectangle(marked, (x1, y1), (x2, y2), MARK_COLOR_BGR, 1, cv2.LINE_AA)
            tag = str(i)
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            tx, ty = x1, max(th + 4, y1)
            cv2.rectangle(marked, (tx, ty - th - 4), (tx + tw + 4, ty), (0, 0, 0), -1)
            cv2.putText(marked, tag, (tx + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, MARK_COLOR_BGR, 1, cv2.LINE_AA)
        schema = self._object_list_schema({
            "mark": {"type": "integer"}, "label": {"type": "string", "enum": classes + [NO_MATCH]},
            "confidence": {"type": "number"}})
        text = (f"Classes: {', '.join(classes)}. Candidates are tagged 1 to {len(proposals)}; "
                "give one entry per tag.")
        result = self._ask(_MARKS_PROMPT, text, marked, schema)
        if result is None:
            return []
        best: Dict[int, VisionDetection] = {}
        for obj in result.get("objects", []):
            try:
                mark, label, score = int(obj["mark"]), str(obj["label"]), float(obj["confidence"])
            except (KeyError, TypeError, ValueError):
                continue
            if label == NO_MATCH or not 1 <= mark <= len(proposals) or score < self.conf:
                continue
            if mark not in best or score > best[mark].score:
                best[mark] = VisionDetection(box_xyxy=tuple(proposals[mark - 1]), label=label, score=score)
        return [best[mark] for mark in sorted(best)]

    def _detect_boxes(self, rgb_bgr: np.ndarray, classes: List[str]) -> List[VisionDetection]:
        h, w = rgb_bgr.shape[:2]
        schema = self._object_list_schema({
            "label": {"type": "string", "enum": classes}, "ymin": {"type": "integer"}, "xmin": {"type": "integer"},
            "ymax": {"type": "integer"}, "xmax": {"type": "integer"}, "confidence": {"type": "number"}})
        result = self._ask(_BOXES_PROMPT, f"Classes: {', '.join(classes)}.", rgb_bgr, schema)
        if result is None:
            return []
        detections: List[VisionDetection] = []
        for obj in result.get("objects", []):
            try:
                score = float(obj["confidence"])
                x1, x2 = sorted((float(obj["xmin"]) * w / 1000.0, float(obj["xmax"]) * w / 1000.0))
                y1, y2 = sorted((float(obj["ymin"]) * h / 1000.0, float(obj["ymax"]) * h / 1000.0))
            except (KeyError, TypeError, ValueError):
                continue
            box = (int(max(0, x1)), int(max(0, y1)), int(min(w - 1, x2)), int(min(h - 1, y2)))
            if score < self.conf or box[2] - box[0] < 4 or box[3] - box[1] < 4:
                continue
            detections.append(VisionDetection(box_xyxy=box, label=str(obj["label"]), score=score))
        return detections
