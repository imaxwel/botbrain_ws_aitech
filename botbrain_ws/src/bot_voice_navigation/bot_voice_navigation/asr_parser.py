"""Normalize text emitted by the G1 native ASR bridge."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class VoiceTranscript:
    text: str
    final: bool
    timestamp: float
    source: str = "asr"
    raw: str = ""


def _find_text(payload: Any) -> str:
    if isinstance(payload, str):
        value = payload.strip()
        if value.startswith(("{", "[")):
            try:
                return _find_text(json.loads(value))
            except json.JSONDecodeError:
                pass
        return value
    if not isinstance(payload, dict):
        return ""
    for key in ("text", "transcript", "utterance", "content"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    for key in ("asr", "result", "data"):
        nested = payload.get(key)
        text = _find_text(nested)
        if text:
            return text
    return ""


def _find_final(payload: Any) -> bool:
    if not isinstance(payload, dict):
        return True
    for key in ("final", "is_final", "isFinal"):
        if key in payload:
            value = payload[key]
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                return bool(value) and math.isfinite(float(value))
            if isinstance(value, str):
                normalized = value.strip().lower()
                if normalized in {"true", "1", "yes", "final", "complete", "completed"}:
                    return True
                if normalized in {"false", "0", "no", "partial", "interim", "listening"}:
                    return False
    state = str(payload.get("state", "")).strip().lower()
    if state in {"partial", "interim", "listening", "start"}:
        return False
    if state in {"final", "complete", "completed", "stop", "end"}:
        return True
    for key in ("asr", "result", "data"):
        nested = payload.get(key)
        if isinstance(nested, dict):
            return _find_final(nested)
        if isinstance(nested, str) and nested.strip().startswith(("{", "[")):
            try:
                return _find_final(json.loads(nested))
            except json.JSONDecodeError:
                pass
    return True


def parse_asr_message(raw: str, *, now: float | None = None) -> VoiceTranscript | None:
    """Parse plain text or JSON ASR payloads into one normalized event."""
    if not isinstance(raw, str) or not raw.strip():
        return None
    raw_text = raw.strip()
    try:
        payload: Any = json.loads(raw_text)
    except json.JSONDecodeError:
        if raw_text.startswith(("{", "[")):
            return None
        payload = raw_text
    text = _find_text(payload)
    if not text:
        return None
    return VoiceTranscript(
        text=text,
        final=_find_final(payload),
        timestamp=time.monotonic() if now is None else float(now),
        raw=raw,
    )


class TranscriptDeduper:
    """Reject repeated final transcripts within a bounded time window."""

    def __init__(self, window_seconds: float = 2.0, max_entries: int = 64):
        self.window_seconds = max(0.0, float(window_seconds))
        self.max_entries = max(1, int(max_entries))
        self._seen: deque[tuple[str, float]] = deque()

    def accepts(self, transcript: VoiceTranscript) -> bool:
        if not transcript.final:
            return False
        now = float(transcript.timestamp)
        while self._seen and now - self._seen[0][1] > self.window_seconds:
            self._seen.popleft()
        key = " ".join(transcript.text.split())
        if any(previous == key for previous, _ in self._seen):
            return False
        self._seen.append((key, now))
        while len(self._seen) > self.max_entries:
            self._seen.popleft()
        return True
