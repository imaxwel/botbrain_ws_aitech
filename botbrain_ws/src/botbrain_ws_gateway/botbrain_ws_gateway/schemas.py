from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def stable_payload_hash(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def normalized_prefix(prefix: str) -> str:
    stripped = str(prefix or "/g1").strip("/")
    return "/" + stripped if stripped else ""


def normalized_namespace(namespace: str) -> str:
    stripped = str(namespace or "").strip("/")
    return "/" + stripped if stripped else ""


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return payload


def topic_for(namespace: str, name: str) -> str:
    if name.startswith("/"):
        return name
    ns = normalized_namespace(namespace)
    return f"{ns}/{name}" if ns else f"/{name}"


def duration_to_seconds(value: Any) -> float | None:
    if value is None:
        return None
    sec = getattr(value, "sec", None)
    nanosec = getattr(value, "nanosec", None)
    if sec is None and hasattr(value, "seconds_nanoseconds"):
        sec, nanosec = value.seconds_nanoseconds()
    if sec is None:
        return None
    return float(sec) + float(nanosec or 0) / 1_000_000_000.0


def stamp_to_dict(stamp: Any) -> dict[str, int] | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None:
        return None
    return {"sec": int(sec), "nanosec": int(nanosec or 0)}


class IdempotencyConflict(Exception):
    pass
