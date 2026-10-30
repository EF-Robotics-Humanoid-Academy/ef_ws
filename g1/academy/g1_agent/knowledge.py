"""Documentary RAG over the agent's local JSON knowledge files.

Replaces agent.knowledge.document_rag.DocumentRAG, which wraps
modules/scripts/ollama_ai/nav_bot.KnowledgeRetriever -- importing nav_bot
also imports the legacy dds_env and sdk_slam modules.  The retriever below is
nav_bot's keyword/IDF scorer (same tokenizer, FAQ handling, intent boosts and
scores), so search results are unchanged.
"""
from __future__ import annotations

import json
import math
import re
import string
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Tuple

STOP_WORDS = {
    "a", "an", "and", "at", "called", "go", "i", "me", "my", "named", "navigate",
    "please", "point", "robot", "take", "the", "to",
    "als", "bitte", "bring", "den", "der", "die", "du", "fahre", "gehe",
    "ich", "mich", "mir", "nach", "navigiere", "punkt", "roboter", "zu", "zum", "zur",
}
WORD_RE = re.compile(r"[\wäöüÄÖÜß]+", re.UNICODE)


@dataclass(frozen=True)
class KnowledgeEntry:
    title: str
    text: str
    source: str
    path: str
    tokens: Tuple[str, ...]


def compact_text(text: str) -> str:
    return " ".join(str(text).strip().split())


def normalize_text(text: str) -> str:
    text = compact_text(text).lower().strip(string.punctuation + "，。！？、；：")
    return compact_text(re.sub(r"[,;:，。！？、；：!?]+", " ", text))


def tokenize(text: str) -> Tuple[str, ...]:
    return tuple(token for token in (m.group(0).lower() for m in WORD_RE.finditer(text))
                 if len(token) > 1 and token not in STOP_WORDS)


def flatten_leaves(value: Any, path: str = "$") -> List[Tuple[str, str]]:
    if isinstance(value, dict):
        return [leaf for key, child in value.items() for leaf in flatten_leaves(child, f"{path}.{key}")]
    if isinstance(value, list):
        return [leaf for index, child in enumerate(value) for leaf in flatten_leaves(child, f"{path}[{index}]")]
    if value is None:
        return [(path, "null")]
    if isinstance(value, bool):
        return [(path, "true" if value else "false")]
    if isinstance(value, (int, float, str)):
        return [(path, str(value))]
    return [(path, json.dumps(value, ensure_ascii=False, sort_keys=True))]


def title_for(value: Any, fallback: str) -> str:
    if isinstance(value, dict):
        for key in ("title", "name", "question", "id", "label", "type", "category"):
            found = value.get(key)
            if isinstance(found, (str, int, float)) and str(found).strip():
                return str(found).strip()
    return fallback


def entry_from_value(value: Any, *, source: str, path: str, fallback_title: str) -> Optional[KnowledgeEntry]:
    lines = [f"{leaf_path}: {leaf}" for leaf_path, leaf in flatten_leaves(value, path) if str(leaf).strip()]
    title = title_for(value, fallback_title)
    text = f"{title}\n" + "\n".join(lines)
    tokens = tokenize(text)
    if not lines or not tokens:
        return None
    return KnowledgeEntry(title=title, text=text, source=source, path=path, tokens=tokens)


def entries_from_faqs(section: dict, *, source: str, path: str, fallback_title: str) -> List[KnowledgeEntry]:
    faqs = section.get("faqs")
    if not isinstance(faqs, list):
        return []
    section_title = compact_text(str(section.get("title", fallback_title)))
    category = compact_text(str(section.get("category", "")))
    entries = []
    for index, item in enumerate(faqs):
        if not isinstance(item, dict):
            continue
        question = compact_text(str(item.get("question", "")))
        answer = compact_text(str(item.get("answer", "")))
        if not question or not answer:
            continue
        text = "\n".join(line for line in (f"Section: {section_title}", f"Category: {category}" if category else "",
                                           f"Question: {question}", f"Answer: {answer}") if line)
        tokens = tokenize(text)
        if tokens:
            entries.append(KnowledgeEntry(title=question, text=text, source=source, path=f"{path}.faqs[{index}]", tokens=tokens))
    return entries


def entries_from_json(data: Any, *, source: str) -> List[KnowledgeEntry]:
    entries: List[KnowledgeEntry] = []

    def add(item: Any, path: str, title: str) -> None:
        if isinstance(item, dict):
            faq_entries = entries_from_faqs(item, source=source, path=path, fallback_title=title)
            if faq_entries:
                entries.extend(faq_entries)
                return
        entry = entry_from_value(item, source=source, path=path, fallback_title=title)
        if entry:
            entries.append(entry)

    if isinstance(data, list):
        for index, item in enumerate(data):
            add(item, f"$[{index}]", f"record {index + 1}")
    elif isinstance(data, dict):
        for key, value in data.items():
            if isinstance(value, list):
                for index, item in enumerate(value):
                    add(item, f"$.{key}[{index}]", f"{key} {index + 1}")
            elif isinstance(value, dict):
                add(value, f"$.{key}", str(key))
    if not entries:
        root = entry_from_value(data, source=source, path="$", fallback_title=Path(source).stem)
        if root:
            entries.append(root)
    return entries


class KnowledgeRetriever:
    def __init__(self, paths: List[Path]) -> None:
        self.entries: List[KnowledgeEntry] = []
        for path in paths:
            self.entries.extend(entries_from_json(json.loads(path.read_text(encoding="utf-8")), source=str(path)))
        doc_count = max(1, len(self.entries))
        df: dict = {}
        for entry in self.entries:
            for token in set(entry.tokens):
                df[token] = df.get(token, 0) + 1
        self.idf = {token: math.log((doc_count + 1) / (freq + 0.5)) + 1.0 for token, freq in df.items()}

    def search(self, query: str, *, top_k: int, min_score: float,
               intent_text: Optional[str] = None) -> List[Tuple[KnowledgeEntry, float]]:
        """`intent_text`: the full question for the intent boosts when `query` is reduced to its topic words."""
        query_set = set(tokenize(query))
        if not query_set:
            return []
        query_low = normalize_text(intent_text or query)
        scored = []
        for entry in self.entries:
            counts: dict = {}
            for token in entry.tokens:
                counts[token] = counts.get(token, 0) + 1
            score = sum(self.idf.get(t, 1.0) * (1.0 + math.log(counts[t])) for t in query_set if t in counts)
            if query.strip().lower() in entry.text.lower():
                score += 2.0
            score += self._intent_boost(query_low, entry)
            score /= math.sqrt(max(1, len(query_set)) * max(1, len(set(entry.tokens))))
            if score >= min_score:
                scored.append((entry, score))
        scored.sort(key=lambda item: item[1], reverse=True)
        return scored[: max(1, top_k)]

    @staticmethod
    def _intent_boost(query_low: str, entry: KnowledgeEntry) -> float:
        title, text = normalize_text(entry.title), normalize_text(entry.text)
        overview_cues = ("was ist", "wer ist", "erzähl", "erzaehl", "erzähle", "erzaehle", "über dich", "ueber dich",
                         "über ef robotics", "ueber ef robotics", "what is", "who is", "tell me about")
        if "ef robotics" in query_low and any(cue in query_low for cue in overview_cues):
            if title == "what is ef robotics":
                return 8.0
            if "company overview" in text:
                return 1.5
        if ("unitree g1" in query_low or "g1" in query_low) and any(cue in query_low for cue in overview_cues):
            if title == "what is the unitree g1":
                return 8.0
        if "autoxing" in query_low and "relationship" in query_low and "relationship between ef robotics and autoxing" in title:
            return 6.0
        if "cenobots" in query_low and "relationship" in query_low and "relationship between ef robotics and cenobots" in title:
            return 6.0
        return 0.0


class DocumentRAG:
    """Same interface as agent.knowledge.document_rag.DocumentRAG."""

    def __init__(self, knowledge_files: List[str]) -> None:
        existing = [Path(f).expanduser() for f in knowledge_files if Path(f).expanduser().exists()]
        if not existing:
            raise FileNotFoundError(f"none of the given knowledge files exist: {knowledge_files}")
        self._retriever = KnowledgeRetriever(existing)

    def search(self, query: str, *, top_k: int = 4, min_score: float = 0.06) -> list:
        from agent.models import KnowledgeRef

        return [KnowledgeRef(source_type="documentary", source=f"{entry.source} ({entry.title})", text=entry.text,
                             trust="authoritative", note=f"score={score:.3f}")
                for entry, score in self._retriever.search(query, top_k=top_k, min_score=min_score)]


# Words that say nothing about a topic: a greeting or "how are you" must not
# pull facts into the reply.
CHAT_FILLER = {
    "about", "again", "all", "also", "am", "are", "be", "been", "bye", "can", "could", "did", "do", "does", "doing",
    "for", "from", "good", "great", "has", "have", "hello", "hey", "hi", "how", "if", "in", "is", "it", "its", "just",
    "know", "like", "many", "morning", "much", "need", "of", "ok", "okay", "on", "or", "our", "should", "so", "some",
    "tell", "thank", "thanks", "that", "their", "them", "then", "there", "these", "they", "this", "today", "us", "want",
    "was", "we", "were", "what", "when", "where", "which", "who", "why", "will", "with", "would", "yes", "you", "your",
    "explain", "give", "say", "see", "use", "using", "work", "works", "get", "got", "very", "really", "up", "lot", "nice",
    "meet", "doing", "weather",
    "wie", "was", "ist", "sind", "hallo", "danke", "guten", "tag", "ein", "eine", "und", "oder", "dir", "dich", "geht",
}


class ChatKnowledge:
    """Facts for the chat: the JSON FAQ files, searched with the retriever above.
    An unreadable or missing file only removes its facts."""

    def __init__(self, paths: List[Path], *, on_error: Any = None) -> None:
        usable = []
        for path in paths:
            try:
                json.loads(Path(path).read_text(encoding="utf-8"))
                usable.append(Path(path))
            except Exception as exc:
                if on_error is not None:
                    on_error(f"knowledge file {path} skipped: {exc}")
        self._retriever = KnowledgeRetriever(usable)

    def __len__(self) -> int:
        return len(self._retriever.entries)

    def search(self, query: str, *, top_k: int = 4, min_score: float = 0.6, keep_fraction: float = 0.55) -> List[str]:
        """Up to top_k facts (plain text); nothing for small talk.  Scores are
        length-normalized: a real topic match stays above 1, one chance word
        below min_score, and facts far under the best match are dropped."""
        words = [w for w in tokenize(query) if w not in CHAT_FILLER]
        if not words:
            return []
        hits = self._retriever.search(" ".join(words), top_k=top_k, min_score=min_score, intent_text=query)
        return [self._fact(entry) for entry, score in hits if score >= keep_fraction * hits[0][1]]

    @staticmethod
    def _fact(entry: KnowledgeEntry) -> str:
        lines = {key: value for key, _, value in (line.partition(": ") for line in entry.text.splitlines())}
        if "Question" in lines and "Answer" in lines:
            return f"{lines['Question']} {lines['Answer']}"
        return compact_text(entry.text)[:500]
