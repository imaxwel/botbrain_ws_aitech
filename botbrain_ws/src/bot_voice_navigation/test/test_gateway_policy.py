from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "bot_navigation"))

from bot_voice_navigation.destinations import DestinationResolver  # noqa: E402
from bot_voice_navigation.navigation_gateway import (  # noqa: E402
    GatewayPolicy,
    NavigationBusyError,
    NavigationPolicyError,
)


@pytest.fixture
def policy():
    resolver = DestinationResolver(
        aliases={"ug": {"电梯": "elevator", "办公室一": "office1"}},
        waypoints={
            "ug": {"elevator": {"x": 1.0}, "office1": {"x": 2.0}},
            "floor2": {"elevator": {"x": 3.0}},
        },
    )
    return GatewayPolicy(resolver)


def test_policy_resolves_aliases_to_canonical_waypoints(policy):
    task = policy.begin(
        destination="电梯",
        sequence=["办公室一"],
        scene="ug",
        task_id="task-1",
    )
    assert task.task_id == "task-1"
    assert task.scene == "ug"
    assert task.waypoints == ("elevator", "office1")
    assert policy.active_task().state == "ACCEPTED"


def test_policy_rejects_concurrent_task_and_allows_matching_cancel(policy):
    policy.begin(destination="电梯", sequence=[], scene="ug", task_id="task-1")
    with pytest.raises(NavigationBusyError):
        policy.begin(destination="办公室一", sequence=[], scene="ug", task_id="task-2")
    assert policy.cancel("task-2") is False
    assert policy.cancel("task-1") is True
    assert policy.active_task() is None


def test_policy_rejects_unknown_destination_without_creating_task(policy):
    with pytest.raises(NavigationPolicyError):
        policy.begin(destination="卫生间", sequence=[], scene="ug", task_id="task-1")
    assert policy.active_task() is None


def test_policy_is_scene_scoped(policy):
    with pytest.raises(NavigationPolicyError):
        policy.begin(destination="办公室一", sequence=[], scene="floor2", task_id="task-1")


def test_policy_marks_completed_and_releases_task(policy):
    policy.begin(destination="电梯", sequence=[], scene="ug", task_id="task-1")
    policy.mark_state("NAVIGATING")
    policy.mark_state("SUCCEEDED")
    policy.finish("task-1")
    assert policy.active_task() is None
