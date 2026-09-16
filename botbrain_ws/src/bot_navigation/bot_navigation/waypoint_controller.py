"""
Reusable process controller for the project's existing waypoint navigator.

The navigator remains the single source of truth for Nav2 validation and goal
execution. This wrapper adds a cancellable, feedback-friendly boundary for
higher-level clients without duplicating navigation behavior.
"""

from __future__ import annotations

import queue
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence


_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
_FEEDBACK = re.compile(
    r"distance=(?P<distance>-?[0-9]+(?:\.[0-9]+)?)\s*m,\s*"
    r"path_remaining=(?P<remaining>-?[0-9]+(?:\.[0-9]+)?)\s*m,\s*"
    r"yaw_error=(?P<yaw>-?[0-9]+(?:\.[0-9]+)?)\s*deg,\s*"
    r"recoveries=(?P<recoveries>[0-9]+)"
)
_NAVIGATING = re.compile(r'Navigating to "(?P<name>[^"]+)"')
_REACHED = re.compile(r'Reached "(?P<name>[^"]+)"')


@dataclass(frozen=True)
class NavigationFeedback:
    task_id: str
    current_waypoint: str
    state: str
    distance_remaining: float = -1.0
    distance_to_goal: float = -1.0
    yaw_error_deg: float = -1.0
    recoveries: int = 0


@dataclass(frozen=True)
class NavigationResult:
    success: bool
    status: str
    message: str
    completed_waypoints: tuple[str, ...] = ()
    task_id: str = ""


class NavigationBusyError(RuntimeError):
    """Raised when a second waypoint process is requested."""


class WaypointNavigationController:
    def __init__(
        self,
        script_path: Path | None = None,
        *,
        python_executable: str | None = None,
        robot: str = "g1_robot",
        popen_factory: Callable[..., object] | None = None,
    ):
        self.script_path = script_path or self._default_script_path()
        self.python_executable = python_executable or sys.executable
        self.robot = robot
        self._popen_factory = popen_factory or subprocess.Popen
        self._state_lock = threading.Lock()
        self._process = None
        self._cancel_event = threading.Event()
        self._cancel_signal_sent = False
        self._cancel_requested_at = 0.0

    @staticmethod
    def _default_script_path() -> Path:
        candidates = (
            Path("/botbrain_ws/install/bot_navigation/lib/bot_navigation/waypoint_navigator.py"),
            Path("/botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py"),
            Path(__file__).resolve().parents[1] / "scripts" / "waypoint_navigator.py",
        )
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]

    @staticmethod
    def _validate_name(value: str, label: str) -> str:
        value = str(value).strip()
        if not value or not _SAFE_NAME.fullmatch(value):
            raise ValueError(f"unsafe {label}: {value!r}")
        return value

    def build_command(
        self,
        names: Sequence[str],
        scene: str,
        *,
        loop: bool = False,
    ) -> list[str]:
        if not names:
            raise ValueError("at least one waypoint is required")
        safe_names = [self._validate_name(name, "waypoint") for name in names]
        safe_scene = self._validate_name(scene, "scene")
        command = [
            str(self.python_executable),
            str(self.script_path),
            *safe_names,
            "--scene",
            safe_scene,
            "--robot",
            self._validate_name(self.robot, "robot namespace"),
        ]
        if loop:
            command.append("--loop")
        return command

    @staticmethod
    def parse_output_line(line: str) -> NavigationFeedback | None:
        match = _FEEDBACK.search(line)
        if not match:
            return None
        return NavigationFeedback(
            task_id="",
            current_waypoint="",
            state="NAVIGATING",
            distance_remaining=float(match.group("remaining")),
            distance_to_goal=float(match.group("distance")),
            yaw_error_deg=float(match.group("yaw")),
            recoveries=int(match.group("recoveries")),
        )

    @staticmethod
    def classify_result(
        *,
        returncode: int,
        canceled: bool,
        requested: Sequence[str],
        completed: Sequence[str],
        task_id: str = "",
    ) -> NavigationResult:
        completed_tuple = tuple(completed)
        if canceled:
            return NavigationResult(
                False,
                "CANCELED",
                "navigation canceled",
                completed_tuple,
                task_id,
            )
        if returncode == 0:
            return NavigationResult(
                True,
                "SUCCEEDED",
                "navigation completed",
                completed_tuple or tuple(requested),
                task_id,
            )
        return NavigationResult(
            False,
            "ABORTED",
            f"waypoint navigator exited with code {returncode}",
            completed_tuple,
            task_id,
        )

    def request_cancel(self) -> bool:
        self._cancel_event.set()
        with self._state_lock:
            process = self._process
            if process is not None and process.poll() is None:
                self._send_interrupt_locked(process)
                return True
        if process is None:
            return False
        return False

    def _send_interrupt_locked(self, process: object) -> None:
        if self._cancel_signal_sent:
            return
        try:
            if hasattr(process, "send_signal"):
                process.send_signal(signal.SIGINT)
            else:
                process.terminate()
            self._cancel_signal_sent = True
            self._cancel_requested_at = time.monotonic()
        except Exception:
            return

    def start(
        self,
        names: Sequence[str],
        scene: str,
        *,
        loop: bool = False,
        task_id: str = "",
        feedback_callback: Callable[[NavigationFeedback], None] | None = None,
    ) -> NavigationResult:
        command = self.build_command(names, scene, loop=loop)
        with self._state_lock:
            if self._process is not None and self._process.poll() is None:
                raise NavigationBusyError("a waypoint navigation task is active")
            if self._cancel_event.is_set():
                self._cancel_event.clear()
                return self.classify_result(
                    returncode=-signal.SIGINT,
                    canceled=True,
                    requested=names,
                    completed=(),
                    task_id=task_id,
                )
            self._cancel_event.clear()
            self._cancel_signal_sent = False
            self._cancel_requested_at = 0.0

        process = self._popen_factory(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        with self._state_lock:
            self._process = process

        output_queue: queue.Queue[str | None] = queue.Queue()
        completed: list[str] = []
        current_waypoint = ""

        def read_output() -> None:
            stream = getattr(process, "stdout", None)
            if stream is None:
                output_queue.put(None)
                return
            for output_line in stream:
                output_queue.put(output_line.rstrip("\n"))
            output_queue.put(None)

        reader = threading.Thread(target=read_output, name="waypoint-output", daemon=True)
        reader.start()
        output_closed = False
        try:
            while not output_closed or process.poll() is None:
                if self._cancel_event.is_set() and process.poll() is None:
                    with self._state_lock:
                        self._send_interrupt_locked(process)
                        if (
                            self._cancel_signal_sent
                            and time.monotonic() - self._cancel_requested_at > 2.0
                        ):
                            try:
                                process.terminate()
                            except Exception:
                                pass
                try:
                    line = output_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if line is None:
                    output_closed = True
                    continue
                navigating = _NAVIGATING.search(line)
                if navigating:
                    current_waypoint = navigating.group("name")
                    if feedback_callback:
                        feedback_callback(
                            NavigationFeedback(
                                task_id, current_waypoint, "NAVIGATING"
                            )
                        )
                reached = _REACHED.search(line)
                if reached:
                    reached_name = reached.group("name")
                    if reached_name not in completed:
                        completed.append(reached_name)
                    if feedback_callback:
                        feedback_callback(
                            NavigationFeedback(task_id, reached_name, "WAYPOINT_REACHED")
                        )
                feedback = self.parse_output_line(line)
                if feedback and feedback_callback:
                    feedback_callback(
                        NavigationFeedback(
                            task_id,
                            current_waypoint,
                            feedback.state,
                            feedback.distance_remaining,
                            feedback.distance_to_goal,
                            feedback.yaw_error_deg,
                            feedback.recoveries,
                        )
                    )
            returncode = process.wait()
            return self.classify_result(
                returncode=returncode,
                canceled=self._cancel_event.is_set(),
                requested=names,
                completed=completed,
                task_id=task_id,
            )
        finally:
            with self._state_lock:
                self._process = None
