from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from .schemas import stamp_to_dict


@dataclass
class TopicEntry:
    key: str
    topic: str
    max_age_s: float
    last_msg: Any | None = None
    last_received_monotonic: float | None = None
    last_header_stamp: Any | None = None
    count: int = 0
    first_received_monotonic: float | None = None
    summary: dict[str, Any] = field(default_factory=dict)

    def update(self, msg: Any) -> None:
        now = time.monotonic()
        if self.first_received_monotonic is None:
            self.first_received_monotonic = now
        self.last_msg = msg
        self.last_received_monotonic = now
        self.count += 1
        header = getattr(msg, "header", None)
        self.last_header_stamp = getattr(header, "stamp", None)
        self.summary = summarize_message(msg)

    def age_s(self) -> float | None:
        if self.last_received_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self.last_received_monotonic)

    def hz(self) -> float | None:
        if self.first_received_monotonic is None or self.last_received_monotonic is None or self.count < 2:
            return None
        elapsed = self.last_received_monotonic - self.first_received_monotonic
        if elapsed <= 0:
            return None
        return (self.count - 1) / elapsed

    def fresh(self) -> bool:
        age = self.age_s()
        return age is not None and age <= self.max_age_s

    def as_dict(self) -> dict[str, Any]:
        age = self.age_s()
        hz = self.hz()
        return {
            "key": self.key,
            "topic": self.topic,
            "fresh": self.fresh(),
            "age_s": None if age is None else round(age, 3),
            "max_age_s": self.max_age_s,
            "count": self.count,
            "hz": None if hz is None else round(hz, 2),
            "last_header_stamp": stamp_to_dict(self.last_header_stamp),
            "summary": self.summary,
        }


class TopicCache:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.entries: dict[str, TopicEntry] = {}

    def add(self, key: str, topic: str, max_age_s: float) -> TopicEntry:
        with self._lock:
            entry = TopicEntry(key=key, topic=topic, max_age_s=max_age_s)
            self.entries[key] = entry
            return entry

    def update(self, key: str, msg: Any) -> None:
        with self._lock:
            if key in self.entries:
                self.entries[key].update(msg)

    def get(self, key: str) -> TopicEntry | None:
        with self._lock:
            return self.entries.get(key)

    def as_dict(self) -> dict[str, Any]:
        with self._lock:
            return {key: entry.as_dict() for key, entry in self.entries.items()}


def summarize_message(msg: Any) -> dict[str, Any]:
    if hasattr(msg, "floor_label") and hasattr(msg, "floor_index") and hasattr(msg, "confidence"):
        return {
            "schema_version": str(getattr(msg, "schema_version", "")),
            "building_id": str(getattr(msg, "building_id", "")),
            "floor_map_id": str(getattr(msg, "floor_map_id", "")),
            "calibration_id": str(getattr(msg, "calibration_id", "")),
            "base_floor_label": str(getattr(msg, "base_floor_label", "")),
            "floor_label": str(getattr(msg, "floor_label", "")),
            "floor_index": int(getattr(msg, "floor_index", 0)),
            "target_floor": str(getattr(msg, "target_floor", "")),
            "target_floor_reached": bool(getattr(msg, "target_floor_reached", False)),
            "confidence": _finite_or_none(getattr(msg, "confidence", None)),
            "stable_for_s": _finite_or_none(getattr(msg, "stable_for_s", None)),
            "delta_height_m": _finite_or_none(getattr(msg, "delta_height_m", None)),
            "delta_height_std_m": _finite_or_none(getattr(msg, "delta_height_std_m", None)),
            "nearest_error_m": _finite_or_none(getattr(msg, "nearest_error_m", None)),
            "floor_gap_margin_m": _finite_or_none(getattr(msg, "floor_gap_margin_m", None)),
            "paired_samples": int(getattr(msg, "paired_samples", 0)),
            "pair_dt_p95_s": _finite_or_none(getattr(msg, "pair_dt_p95_s", None)),
            "mobile_age_s": _finite_or_none(getattr(msg, "mobile_age_s", None)),
            "base_age_s": _finite_or_none(getattr(msg, "base_age_s", None)),
            "stale": bool(getattr(msg, "stale", True)),
            "moving_vertical": bool(getattr(msg, "moving_vertical", False)),
            "vertical_speed_mps": _finite_or_none(getattr(msg, "vertical_speed_mps", None)),
            "vertical_acc_mps2": _finite_or_none(getattr(msg, "vertical_acc_mps2", None)),
            "status": str(getattr(msg, "status", "")),
            "status_reason": str(getattr(msg, "status_reason", "")),
            "sources": [str(item) for item in list(getattr(msg, "sources", []) or [])],
        }
    if hasattr(msg, "percentage") and hasattr(msg, "voltage"):
        return {
            "percentage": _finite_or_none(getattr(msg, "percentage", None)),
            "voltage": _finite_or_none(getattr(msg, "voltage", None)),
            "current": _finite_or_none(getattr(msg, "current", None)),
            "present": bool(getattr(msg, "present", True)),
        }
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pose = msg.pose.pose
        return {"pose": _pose_summary(pose), "child_frame_id": str(getattr(msg, "child_frame_id", ""))}
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        return {"pose": _pose_summary(msg.pose)}
    if hasattr(msg, "data") and isinstance(getattr(msg, "data"), (float, int, str, bool)):
        return {"data": getattr(msg, "data")}
    if hasattr(msg, "name") and hasattr(msg, "position"):
        names = list(getattr(msg, "name", []) or [])
        return {"joint_count": len(names), "first_joints": names[:5]}
    if hasattr(msg, "width") and hasattr(msg, "height") and hasattr(msg, "fields"):
        return {
            "width": int(getattr(msg, "width", 0)),
            "height": int(getattr(msg, "height", 0)),
            "point_step": int(getattr(msg, "point_step", 0)),
            "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        }
    if hasattr(msg, "info") and hasattr(msg.info, "width"):
        return {
            "width": int(getattr(msg.info, "width", 0)),
            "height": int(getattr(msg.info, "height", 0)),
            "resolution": _finite_or_none(getattr(msg.info, "resolution", None)),
            "frame_id": str(getattr(getattr(msg, "header", None), "frame_id", "")),
        }
    if hasattr(msg, "status"):
        status = getattr(msg, "status")
        if isinstance(status, list):
            return {"status_count": len(status), "names": [str(getattr(item, "name", "")) for item in status[:5]]}
    if hasattr(msg, "containers"):
        containers = getattr(msg, "containers")
        return {
            "status_count": len(containers),
            "statuses": [
                {"name": str(getattr(item, "name", "")), "status": str(getattr(item, "status", ""))}
                for item in list(containers)[:10]
            ],
        }
    return {"type": type(msg).__name__}


def _pose_summary(pose: Any) -> dict[str, Any]:
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    return {
        "x": _finite_or_none(getattr(position, "x", None)),
        "y": _finite_or_none(getattr(position, "y", None)),
        "z": _finite_or_none(getattr(position, "z", None)),
        "qx": _finite_or_none(getattr(orientation, "x", None)),
        "qy": _finite_or_none(getattr(orientation, "y", None)),
        "qz": _finite_or_none(getattr(orientation, "z", None)),
        "qw": _finite_or_none(getattr(orientation, "w", None)),
    }


def _finite_or_none(value: Any) -> float | int | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return round(number, 6)
