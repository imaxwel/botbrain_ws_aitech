from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
NAV_ROOT = ROOT.parent / "bot_navigation"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(NAV_ROOT))

from bot_voice_navigation.asr_parser import VoiceTranscript  # noqa: E402
from bot_voice_navigation.intent import NavigationIntent  # noqa: E402
from bot_voice_navigation.voice_navigation_node import (  # noqa: E402
    action_goal_fields,
    is_local_cancel_command,
    navigation_result_text,
    should_process_transcript,
)


def test_partial_transcript_is_not_processed():
    assert should_process_transcript(VoiceTranscript("去电梯", False, 1.0)) is False
    assert should_process_transcript(VoiceTranscript("去电梯", True, 1.0)) is True


def test_action_goal_fields_use_canonical_names_and_explicit_scene():
    intent = NavigationIntent(
        intent="navigate",
        destination="电梯",
        sequence=("办公室一",),
        confidence=0.9,
        need_confirmation=False,
    )
    fields = action_goal_fields(intent, scene="ug", task_id="task-1")
    assert fields == {
        "destination": "电梯",
        "sequence": ["办公室一"],
        "scene": "ug",
        "loop": False,
        "request_id": "task-1",
    }


@pytest.mark.parametrize("status,expected", [
    ("SUCCEEDED", "已到达电梯"),
    ("CANCELED", "导航已取消"),
    ("ABORTED", "导航失败"),
    ("REJECTED", "无法开始导航"),
])
def test_navigation_result_text_is_safe_and_concise(status, expected):
    assert navigation_result_text(status, "电梯").startswith(expected)


def test_unknown_status_does_not_claim_arrival():
    assert "到达" not in navigation_result_text("ERROR", "电梯")


@pytest.mark.parametrize(
    "text",
    ["停止", "停止导航", "取消导航", "别走了", "停下来", "stop", "CANCEL"],
)
def test_safety_cancel_commands_are_recognized_locally(text):
    assert is_local_cancel_command(text) is True


def test_destination_sentence_is_not_misclassified_as_local_cancel():
    assert is_local_cancel_command("带我去办公室一") is False
