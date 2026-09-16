from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VOICE_PACKAGE_ROOT = PACKAGE_ROOT.parent / "bot_voice_navigation"
sys.path.insert(0, str(VOICE_PACKAGE_ROOT))
sys.path.insert(0, str(PACKAGE_ROOT.parent / "bot_navigation"))

from bot_navigation.waypoint_controller import (  # noqa: E402
    NavigationResult,
    WaypointNavigationController,
)


def test_controller_builds_argument_list_without_shell_interpolation():
    controller = WaypointNavigationController(
        script_path=Path("/botbrain_ws/install/bot_navigation/lib/bot_navigation/waypoint_navigator.py"),
        python_executable="/usr/bin/python3",
        robot="g1_robot",
    )
    assert controller.build_command(["ug_face_evelator", "office1"], "ug") == [
        "/usr/bin/python3",
        "/botbrain_ws/install/bot_navigation/lib/bot_navigation/waypoint_navigator.py",
        "ug_face_evelator",
        "office1",
        "--scene",
        "ug",
        "--robot",
        "g1_robot",
    ]


def test_controller_builds_loop_command_only_when_requested():
    controller = WaypointNavigationController(script_path=Path("navigator.py"))
    command = controller.build_command(["office1"], "aitech_v2", loop=True)
    assert command[-1] == "--loop"


def test_controller_parses_navigation_feedback_line():
    feedback = WaypointNavigationController.parse_output_line(
        '  distance=1.20 m, path_remaining=2.30 m, yaw_error=4.0 deg, recoveries=1, elapsed=3.5 s'
    )
    assert feedback is not None
    assert feedback.distance_remaining == 2.3
    assert feedback.distance_to_goal == 1.2
    assert feedback.yaw_error_deg == 4.0
    assert feedback.recoveries == 1


def test_controller_classifies_process_result_and_keeps_completed_names():
    result = WaypointNavigationController.classify_result(
        returncode=0,
        canceled=False,
        requested=["office1"],
        completed=["office1"],
    )
    assert result == NavigationResult(
        success=True,
        status="SUCCEEDED",
        message="navigation completed",
        completed_waypoints=("office1",),
    )

    canceled = WaypointNavigationController.classify_result(
        returncode=-15, canceled=True, requested=["office1"], completed=[]
    )
    assert canceled.success is False
    assert canceled.status == "CANCELED"


def test_controller_rejects_empty_or_unsafe_waypoint_names():
    controller = WaypointNavigationController(script_path=Path("navigator.py"))
    try:
        controller.build_command([], "ug")
    except ValueError as error:
        assert "waypoint" in str(error)
    else:
        raise AssertionError("empty waypoint list was accepted")
    try:
        controller.build_command(["../escape"], "ug")
    except ValueError as error:
        assert "unsafe" in str(error)
    else:
        raise AssertionError("unsafe waypoint was accepted")


def test_controller_requests_sigint_before_force_termination():
    class FakeProcess:
        def __init__(self):
            self.stdout = []
            self.signals = []
            self._returncode = None

        def poll(self):
            return self._returncode

        def send_signal(self, signal):
            self.signals.append(signal)
            self._returncode = -signal

        def terminate(self):
            self.signals.append("terminate")
            self._returncode = -15

    process = FakeProcess()
    controller = WaypointNavigationController(
        script_path=Path("navigator.py"),
        popen_factory=lambda *args, **kwargs: process,
    )
    with controller._state_lock:
        controller._process = process
    assert controller.request_cancel() is True
    import signal
    assert process.signals == [signal.SIGINT]


def test_controller_honors_cancel_requested_before_process_start():
    started = []

    class FakeStdout:
        def __iter__(self):
            return iter([])

    class FakeProcess:
        stdout = FakeStdout()

        def poll(self):
            return 0

        def wait(self):
            return 0

    controller = WaypointNavigationController(
        script_path=Path("navigator.py"),
        popen_factory=lambda *args, **kwargs: (
            started.append(args) or FakeProcess()
        ),
    )
    assert controller.request_cancel() is False
    canceled = controller.start(["office1"], "ug", task_id="task-1")
    assert canceled.status == "CANCELED"

    recovered = controller.start(["office1"], "ug", task_id="task-2")
    assert recovered.status == "SUCCEEDED"
    assert len(started) == 1
