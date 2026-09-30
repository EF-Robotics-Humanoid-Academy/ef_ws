"""Local Ollama helpers for Day 4 (academy bonus).

Small, dependency-light building blocks the Day 4 chatbot tasks call
directly. Nothing here talks to the robot -- these functions only talk to a
local Ollama server and to local JSON knowledge files.

Requires a local Ollama server already running on the academy accounts
(`ollama serve`, listening on 127.0.0.1:11434) with these models pulled --
`qwen3.5:9b` (chat) and `qwen2.5vl:7b` (vision, the only vision-language
model in the pulled set); also available for a lighter/faster chat model
via OLLAMA_CHAT_MODEL: `granite4.2:3b`, `gemma3:1b`, `qwen2.5:0.5b`. No API
key needed -- everything runs on-machine.
"""
from __future__ import annotations

import base64
import difflib
import json
import os
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
DEFAULT_CHAT_MODEL = os.environ.get("OLLAMA_CHAT_MODEL", "qwen3.5:9b")
DEFAULT_VISION_MODEL = os.environ.get("OLLAMA_VISION_MODEL", "qwen2.5vl:7b")

DEFAULT_SYSTEM_PROMPT_DE = (
    "Du bist der Sprachassistent eines Unitree G1 Roboters bei der EF Robotics Academy. "
    "Antworte immer auf Deutsch, auch wenn die Frage auf Englisch gestellt wird. "
    "Antworte kurz (1-3 Saetze) und klar, da deine Antwort per Text-zu-Sprache vorgelesen wird."
)


class OllamaClient:
    """Thin handle around a local Ollama server's base URL."""

    def __init__(self, host=None):
        self.host = host or DEFAULT_OLLAMA_HOST


def get_client(host=None):
    """Returns an Ollama client. Reads OLLAMA_HOST from the environment by
    default (http://localhost:11434) -- pass host explicitly only to
    override that. Unlike a cloud API, no key is needed."""
    return OllamaClient(host)


def _chat(client, messages, model, images_b64=None):
    payload_messages = [dict(message) for message in messages]
    if images_b64:
        payload_messages[-1]["images"] = list(images_b64)
    body = {"model": model, "messages": payload_messages, "stream": False}
    request = urllib.request.Request(
        url=f"{client.host.rstrip('/')}/api/chat",
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            raw = response.read()
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        if exc.code == 404:
            raise RuntimeError(
                f"Ollama could not find model {model!r} at {client.host}. "
                f"Run `ollama pull {model}` first."
            ) from exc
        raise RuntimeError(f"Ollama HTTP error {exc.code} at {client.host}: {detail[:200]}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(
            f"Could not reach Ollama at {client.host}: {exc}. Is `ollama serve` running?"
        ) from exc
    return json.loads(raw)["message"]["content"]


def chat_reply(client, user_text, history=None, system_prompt=None, model=None):
    """One chat turn. `history` is an optional list of prior
    {"role": "user"|"assistant", "content": str} dicts -- pass the list back
    in on every call (and append the new turns to it) to keep context across
    turns; omit it for a single stateless reply."""
    messages = [{"role": "system", "content": system_prompt or DEFAULT_SYSTEM_PROMPT_DE}]
    messages.extend(history or [])
    messages.append({"role": "user", "content": user_text})
    return _chat(client, messages, model or DEFAULT_CHAT_MODEL)


def describe_image(client, jpeg_bytes, question, model=None):
    """Asks a local vision-capable model a question about one RGB JPEG frame,
    e.g. from g1.get_rgbd()["rgb_jpeg"]. Returns the model's text answer."""
    image_b64 = base64.b64encode(jpeg_bytes).decode("ascii")
    messages = [{"role": "user", "content": question}]
    return _chat(client, messages, model or DEFAULT_VISION_MODEL, images_b64=[image_b64])


def load_knowledge_base(path):
    """Loads a knowledge JSON file shaped like g1_academy_knowledge_de.json
    (see modules/scripts/ollama_ai/*.sample.json for the same shape) and
    flattens it to a list of {"question", "answer", "category"} dicts."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    flat = []
    for section in data.get("knowledge", []):
        category = section.get("category", section.get("title", ""))
        for faq in section.get("faqs", []):
            flat.append({"question": faq["question"], "answer": faq["answer"], "category": category})
    return flat


def retrieve_relevant(knowledge, query, top_k=3):
    """Ranks flattened knowledge entries (see load_knowledge_base) by plain
    text similarity to `query` and returns the top_k best matches. This is a
    simple baseline retriever -- good enough for a small FAQ file, and a
    reasonable thing to swap out once you've got RAG working end to end."""
    scored = [
        (difflib.SequenceMatcher(None, query.lower(), entry["question"].lower()).ratio(), entry)
        for entry in knowledge
    ]
    scored.sort(key=lambda pair: pair[0], reverse=True)
    return [entry for _score, entry in scored[:top_k]]


def _grab_rgb_jpeg(endpoints=None, timeout_ms=3000):
    """Grab one RGB JPEG frame from the robot's RGBD ZMQ stream (the same
    endpoints G1.get_rgbd() uses). Returns the JPEG bytes, or None if no frame
    / no zmq is available. This is the one place in this module that reads from
    the robot's camera stream."""
    try:
        import zmq
    except ModuleNotFoundError:
        return None
    for endpoint in (endpoints or ["tcp://127.0.0.1:5555", "tcp://0.0.0.0:5555", "tcp://localhost:5555"]):
        try:
            ctx = zmq.Context.instance()
            sock = ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.SUBSCRIBE, b"")
            sock.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
            sock.connect(endpoint)
            try:
                parts = sock.recv_multipart()
            finally:
                sock.close(0)
            if parts and parts[0] and parts[0] != b"0":
                return bytes(parts[0])
        except Exception:
            continue
    return None


def detect_object(label, jpeg_bytes=None, client=None, model=None):
    """Return True if `label` (e.g. "soda can") appears to be clearly visible in
    the robot's current camera frame. Grabs one RGB frame from the RGBD
    stream (pass `jpeg_bytes` to skip that, e.g. g1.get_rgbd()["rgb_jpeg"]),
    asks a local vision model a yes/no question via describe_image(), and
    parses the answer to a bool. Creates a client pointed at the local Ollama
    server if none is given. Returns False when no camera frame is
    available."""
    if jpeg_bytes is None:
        jpeg_bytes = _grab_rgb_jpeg()
        if jpeg_bytes is None:
            return False
    if client is None:
        client = get_client()
    question = (f"Is there a {label} clearly visible in this image? "
                "Answer with only the single word yes or no.")
    answer = describe_image(client, jpeg_bytes, question, model=model)
    return str(answer).strip().lower().startswith("y")
