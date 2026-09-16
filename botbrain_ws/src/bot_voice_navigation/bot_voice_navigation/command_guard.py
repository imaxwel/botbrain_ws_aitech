"""Invalidate older voice commands when a local safety cancel is heard."""

from __future__ import annotations

import threading


class CommandCancellationGuard:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._generation = 0

    def snapshot(self) -> int:
        with self._lock:
            return self._generation

    def cancel_prior(self) -> int:
        with self._lock:
            self._generation += 1
            return self._generation

    def is_current(self, generation: int) -> bool:
        with self._lock:
            return generation == self._generation
