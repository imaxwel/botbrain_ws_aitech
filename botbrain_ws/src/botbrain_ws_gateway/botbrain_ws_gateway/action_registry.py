from __future__ import annotations

import itertools
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from .schemas import IdempotencyConflict, now_iso, stable_payload_hash


TERMINAL_STATUSES = {"succeeded", "failed", "cancelled", "timeout", "rejected"}
RUNNING_STATUSES = {"accepted", "running"}


@dataclass
class ActionRecord:
    action_id: str
    request_id: str
    request_hash: str
    mission_run_id: str | None
    kind: str
    status: str
    started_at: str
    updated_at: str
    target_waypoint: str | None = None
    target_poi: str | None = None
    target_pose: dict[str, Any] | None = None
    deadline_monotonic: float | None = None
    ros_action: str = ""
    goal_handle: Any | None = None
    last_feedback: dict[str, Any] = field(default_factory=dict)
    evidence: dict[str, Any] = field(default_factory=dict)
    result: dict[str, Any] = field(default_factory=dict)
    error_code: str | None = None
    message: str = ""

    def touch(self) -> None:
        self.updated_at = now_iso()

    def as_dict(self) -> dict[str, Any]:
        return {
            "action_id": self.action_id,
            "kind": self.kind,
            "status": self.status,
            "request_id": self.request_id,
            "mission_run_id": self.mission_run_id,
            "target_waypoint": self.target_waypoint,
            "target_poi": self.target_poi,
            "target_pose": self.target_pose,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "ros_action": self.ros_action,
            "last_feedback": self.last_feedback,
            "evidence": self.evidence,
            "result": self.result,
            "error_code": self.error_code,
            "message": self.message,
        }


@dataclass
class TaskActionRecord:
    task_action_id: str
    request_id: str
    request_hash: str
    mission_run_id: str | None
    waypoint: str
    action_id: str
    action_class: str
    status: str
    started_at: str
    updated_at: str
    payload: dict[str, Any] = field(default_factory=dict)
    evidence_contract: dict[str, Any] = field(default_factory=dict)
    evidence: dict[str, Any] = field(default_factory=dict)
    error_code: str | None = None
    message: str = ""
    deadline_monotonic: float | None = None

    def touch(self) -> None:
        self.updated_at = now_iso()

    def as_dict(self) -> dict[str, Any]:
        return {
            "task_action_id": self.task_action_id,
            "status": self.status,
            "request_id": self.request_id,
            "mission_run_id": self.mission_run_id,
            "waypoint": self.waypoint,
            "action_id": self.action_id,
            "action_class": self.action_class,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "payload": self.payload,
            "evidence_contract": self.evidence_contract,
            "evidence": self.evidence,
            "error_code": self.error_code,
            "message": self.message,
        }


class ActionRegistry:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._counter = itertools.count(1)
        self._records: dict[str, ActionRecord] = {}
        self._request_index: dict[str, str] = {}

    def create_or_replay(
        self,
        *,
        request_id: str,
        request_payload: dict[str, Any],
        mission_run_id: str | None,
        kind: str,
        target_waypoint: str | None,
        target_poi: str | None,
        target_pose: dict[str, Any] | None,
        timeout_s: float,
        ros_action: str,
    ) -> tuple[ActionRecord, bool]:
        request_hash = stable_payload_hash(request_payload)
        with self._lock:
            existing_id = self._request_index.get(request_id)
            if existing_id:
                existing = self._records[existing_id]
                if existing.request_hash != request_hash:
                    raise IdempotencyConflict(f"request_id {request_id!r} was already used with different payload")
                return existing, True
            action_id = f"nav-{time.strftime('%Y%m%dT%H%M%S')}-{next(self._counter):04d}"
            now = now_iso()
            record = ActionRecord(
                action_id=action_id,
                request_id=request_id,
                request_hash=request_hash,
                mission_run_id=mission_run_id,
                kind=kind,
                status="accepted",
                started_at=now,
                updated_at=now,
                target_waypoint=target_waypoint,
                target_poi=target_poi,
                target_pose=target_pose,
                deadline_monotonic=time.monotonic() + timeout_s,
                ros_action=ros_action,
            )
            self._records[action_id] = record
            self._request_index[request_id] = action_id
            return record, False

    def get(self, action_id: str) -> ActionRecord | None:
        with self._lock:
            return self._records.get(action_id)

    def running(self) -> list[ActionRecord]:
        with self._lock:
            return [record for record in self._records.values() if record.status in RUNNING_STATUSES]

    def update(self, action_id: str, **changes: Any) -> ActionRecord | None:
        with self._lock:
            record = self._records.get(action_id)
            if record is None:
                return None
            for key, value in changes.items():
                setattr(record, key, value)
            record.touch()
            return record


class TaskRegistry:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._counter = itertools.count(1)
        self._records: dict[str, TaskActionRecord] = {}
        self._request_index: dict[str, str] = {}

    def create_or_replay(
        self,
        *,
        request_id: str,
        request_payload: dict[str, Any],
        mission_run_id: str | None,
        waypoint: str,
        action_id: str,
        action_class: str,
        payload: dict[str, Any],
        evidence_contract: dict[str, Any],
        timeout_s: float,
    ) -> tuple[TaskActionRecord, bool]:
        request_hash = stable_payload_hash(request_payload)
        with self._lock:
            existing_id = self._request_index.get(request_id)
            if existing_id:
                existing = self._records[existing_id]
                if existing.request_hash != request_hash:
                    raise IdempotencyConflict(f"request_id {request_id!r} was already used with different payload")
                return existing, True
            task_action_id = f"task-{time.strftime('%Y%m%dT%H%M%S')}-{next(self._counter):04d}"
            now = now_iso()
            record = TaskActionRecord(
                task_action_id=task_action_id,
                request_id=request_id,
                request_hash=request_hash,
                mission_run_id=mission_run_id,
                waypoint=waypoint,
                action_id=action_id,
                action_class=action_class,
                status="accepted",
                started_at=now,
                updated_at=now,
                payload=payload,
                evidence_contract=evidence_contract,
                deadline_monotonic=time.monotonic() + timeout_s,
            )
            self._records[task_action_id] = record
            self._request_index[request_id] = task_action_id
            return record, False

    def get(self, task_action_id: str) -> TaskActionRecord | None:
        with self._lock:
            return self._records.get(task_action_id)

    def running(self) -> list[TaskActionRecord]:
        with self._lock:
            return [record for record in self._records.values() if record.status in RUNNING_STATUSES]

    def stop_all(self, reason: str) -> list[TaskActionRecord]:
        stopped: list[TaskActionRecord] = []
        with self._lock:
            for record in self._records.values():
                if record.status in RUNNING_STATUSES:
                    record.status = "cancelled"
                    record.error_code = "G1_TASK_ACTION_CANCELLED"
                    record.message = reason
                    record.touch()
                    stopped.append(record)
        return stopped
