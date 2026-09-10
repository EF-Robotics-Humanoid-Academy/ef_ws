"""OpenAI API helpers for Day 4 (academy bonus).

Small, dependency-light building blocks the Day 4 chatbot tasks call
directly, and that Codex-written extensions (Tasks 13-15) build on top of.
Nothing here talks to the robot -- these functions only talk to the OpenAI
API and to local JSON knowledge files.

Requires the OPENAI_API_KEY environment variable to already be set (it is,
on the academy accounts) and the `openai` package to be installed.
"""
from __future__ import annotations

import base64
import difflib
import json
import os
from pathlib import Path

DEFAULT_CHAT_MODEL = os.environ.get("OPENAI_CHAT_MODEL", "gpt-4.1-mini")
DEFAULT_VISION_MODEL = os.environ.get("OPENAI_VISION_MODEL", "gpt-4.1-mini")

DEFAULT_SYSTEM_PROMPT_DE = (
    "Du bist der Sprachassistent eines Unitree G1 Roboters bei der EF Robotics Academy. "
    "Antworte immer auf Deutsch, auch wenn die Frage auf Englisch gestellt wird. "
    "Antworte kurz (1-3 Saetze) und klar, da deine Antwort per Text-zu-Sprache vorgelesen wird."
)


def get_client(api_key=None):
    """Returns an OpenAI client. Reads OPENAI_API_KEY from the environment
    by default -- pass api_key explicitly only to override that."""
    try:
        from openai import OpenAI
    except ModuleNotFoundError as exc:
        raise RuntimeError("The 'openai' package is not installed.") from exc
    if api_key is None and not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is not set.")
    return OpenAI(api_key=api_key) if api_key else OpenAI()


def chat_reply(client, user_text, history=None, system_prompt=None, model=None):
    """One chat turn. `history` is an optional list of prior
    {"role": "user"|"assistant", "content": str} dicts -- pass the list back
    in on every call (and append the new turns to it) to keep context across
    turns; omit it for a single stateless reply."""
    messages = [{"role": "system", "content": system_prompt or DEFAULT_SYSTEM_PROMPT_DE}]
    messages.extend(history or [])
    messages.append({"role": "user", "content": user_text})
    response = client.chat.completions.create(model=model or DEFAULT_CHAT_MODEL, messages=messages)
    return response.choices[0].message.content


def describe_image(client, jpeg_bytes, question, model=None):
    """Asks a vision-capable model a question about one RGB JPEG frame, e.g.
    from g1.get_rgbd()["rgb_jpeg"]. Returns the model's text answer."""
    image_url = "data:image/jpeg;base64," + base64.b64encode(jpeg_bytes).decode("ascii")
    response = client.chat.completions.create(
        model=model or DEFAULT_VISION_MODEL,
        messages=[{"role": "user", "content": [
            {"type": "text", "text": question},
            {"type": "image_url", "image_url": {"url": image_url}},
        ]}],
    )
    return response.choices[0].message.content


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
