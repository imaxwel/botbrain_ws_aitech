from __future__ import annotations

import stat
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "bot_navigation"))

from bot_voice_navigation.voice_navigation_node import (  # noqa: E402
    goal_status_name,
    result_status_name,
)
from bot_navigation.waypoint_controller import WaypointNavigationController  # noqa: E402


def test_ros_entrypoint_scripts_are_executable_for_symlink_install():
    scripts = ROOT / "scripts"
    for name in ("voice_navigation_gateway.py", "voice_navigation_node.py"):
        mode = (scripts / name).stat().st_mode
        assert mode & stat.S_IXUSR, f"{name} must be executable for ros2 run"


def test_goal_status_name_maps_ros_action_status_values():
    assert goal_status_name(4) == "SUCCEEDED"
    assert goal_status_name(5) == "CANCELED"
    assert goal_status_name(6) == "ABORTED"
    assert goal_status_name(99) == "ERROR"


def test_result_status_uses_ros_action_response_code_when_available():
    class Result:
        status = "SUCCEEDED"

    class Response:
        status = 4
        result = Result()

    assert result_status_name(Response()) == "SUCCEEDED"


def test_result_status_falls_back_to_nested_result_text():
    class Result:
        status = "CANCELED"

    class Response:
        result = Result()

    assert result_status_name(Response()) == "CANCELED"


def test_controller_start_streams_feedback_and_result_from_existing_cli():
    class FakeStdout:
        def __iter__(self):
            return iter([
                '→ Navigating to "office1" in scene "ug" (x=1.000, y=2.000)\n',
                '  distance=1.20 m, path_remaining=2.30 m, yaw_error=4.0 deg, recoveries=1, elapsed=3.5 s\n',
                '  ✓ Reached "office1", distance=0.10 m\n',
            ])

    class FakeProcess:
        def __init__(self):
            self.stdout = FakeStdout()
            self._returncode = 0

        def poll(self):
            return self._returncode

        def wait(self):
            return self._returncode

        def terminate(self):
            self._returncode = -15

    calls = []
    controller = WaypointNavigationController(
        script_path=Path("navigator.py"),
        python_executable="/usr/bin/python3",
        popen_factory=lambda *args, **kwargs: (
            calls.append((args, kwargs)) or FakeProcess()
        ),
    )
    feedback = []
    result = controller.start(
        ["office1"], "ug", task_id="task-1", feedback_callback=feedback.append
    )
    assert result.success is True
    assert result.completed_waypoints == ("office1",)
    assert any(item.state == "NAVIGATING" for item in feedback)
    assert any(item.distance_remaining == 2.3 for item in feedback)
    assert calls[0][0][0][0:3] == ["/usr/bin/python3", "navigator.py", "office1"]
