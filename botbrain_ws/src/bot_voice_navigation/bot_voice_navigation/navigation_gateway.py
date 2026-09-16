"""Policy layer for safe, scene-aware voice navigation requests."""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace
from typing import Sequence

from .destinations import DestinationResolver


class NavigationPolicyError(ValueError):
    """Raised when a voice navigation request is unsafe or invalid."""


class NavigationBusyError(NavigationPolicyError):
    """Raised when another navigation task is already active."""


@dataclass(frozen=True)
class GatewayTask:
    task_id: str
    scene: str
    requested_destination: str
    waypoints: tuple[str, ...]
    state: str = "ACCEPTED"


class GatewayPolicy:
    """Serialize navigation tasks and resolve spoken names locally."""

    def __init__(self, resolver: DestinationResolver):
        self.resolver = resolver
        self._lock = threading.Lock()
        self._task: GatewayTask | None = None

    def active_task(self) -> GatewayTask | None:
        with self._lock:
            return self._task

    def begin(
        self,
        *,
        destination: str,
        sequence: Sequence[str],
        scene: str,
        task_id: str,
    ) -> GatewayTask:
        if not str(task_id).strip():
            raise NavigationPolicyError("task_id is required")
        with self._lock:
            if self._task is not None:
                raise NavigationBusyError("a navigation task is already active")
            try:
                first = self.resolver.resolve(destination, scene)
                resolved = [first.waypoint_name]
                for item in sequence:
                    resolved.append(self.resolver.resolve(item, scene).waypoint_name)
            except ValueError as error:
                raise NavigationPolicyError(str(error)) from error
            self._task = GatewayTask(
                task_id=str(task_id),
                scene=first.scene,
                requested_destination=first.requested,
                waypoints=tuple(resolved),
            )
            return self._task

    def mark_state(self, state: str, *, task_id: str | None = None) -> GatewayTask:
        with self._lock:
            if self._task is None:
                raise NavigationPolicyError("no active navigation task")
            if task_id is not None and task_id != self._task.task_id:
                raise NavigationPolicyError("task_id does not match active task")
            self._task = replace(self._task, state=str(state))
            return self._task

    def cancel(self, task_id: str) -> bool:
        with self._lock:
            if self._task is None or self._task.task_id != task_id:
                return False
            self._task = replace(self._task, state="CANCELED")
            self._task = None
            return True

    def finish(self, task_id: str) -> None:
        with self._lock:
            if self._task is not None and self._task.task_id == task_id:
                self._task = None
