from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_ROOT = PACKAGE_ROOT.parent / "bot_navigation"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(NAVIGATION_ROOT))

from bot_voice_navigation.asr_parser import (  # noqa: E402
    TranscriptDeduper,
    VoiceTranscript,
    parse_asr_message,
)
from bot_voice_navigation.audio_input import normalize_asr_topics  # noqa: E402
from bot_voice_navigation.destinations import (  # noqa: E402
    DestinationResolver,
    UnknownDestinationError,
)
from bot_voice_navigation.intent import (  # noqa: E402
    IntentValidationError,
    parse_intent_payload,
)


def test_parse_plain_asr_message_is_final_transcript():
    transcript = parse_asr_message("带我去电梯")
    assert transcript == VoiceTranscript(
        text="带我去电梯",
        final=True,
        timestamp=transcript.timestamp,
        source="asr",
        raw="带我去电梯",
    )


def test_parse_json_partial_message_is_not_final():
    transcript = parse_asr_message(
        json.dumps({"state": "partial", "text": "带我去"})
    )
    assert transcript is not None
    assert transcript.text == "带我去"
    assert transcript.final is False


def test_parse_nested_json_final_message():
    transcript = parse_asr_message(
        json.dumps({"asr": {"text": "带我去电梯", "final": True}})
    )
    assert transcript is not None
    assert transcript.text == "带我去电梯"
    assert transcript.final is True


def test_parse_string_boolean_and_nested_string_json_final_flags():
    transcript = parse_asr_message(
        json.dumps({"data": json.dumps({"text": "去办公室一", "final": "false"})})
    )
    assert transcript is not None
    assert transcript.final is False


def test_normalize_asr_topics_supports_native_and_ros_aliases():
    assert normalize_asr_topics(["audio_msg", "/rt/audio_msg", "/audio_msg"]) == (
        "/audio_msg",
        "/rt/audio_msg",
    )


def test_parse_empty_or_malformed_message_returns_none():
    assert parse_asr_message("") is None
    assert parse_asr_message("{not-json") is None
    assert parse_asr_message('{"text": "   "}') is None


def test_deduper_accepts_final_once_within_window():
    deduper = TranscriptDeduper(window_seconds=2.0)
    first = VoiceTranscript("去电梯", True, 10.0, "asr", "去电梯")
    second = VoiceTranscript("去电梯", True, 11.0, "asr", "去电梯")
    assert deduper.accepts(first) is True
    assert deduper.accepts(second) is False


def test_destination_resolver_maps_alias_and_canonical_name():
    resolver = DestinationResolver(
        aliases={"ug": {"电梯": "ug_face_evelator", "回家": "home"}},
        waypoints={
            "ug": {
                "ug_face_evelator": {"x": 1.0, "y": 2.0},
                "home": {"x": 0.0, "y": 0.0},
            }
        },
    )
    elevator = resolver.resolve("  电梯。 ", "ug")
    assert elevator.waypoint_name == "ug_face_evelator"
    assert elevator.requested == "电梯"
    assert resolver.resolve("home", "ug").waypoint_name == "home"


def test_destination_resolver_is_scene_scoped_and_lists_safe_names():
    resolver = DestinationResolver(
        aliases={"ug": {"电梯": "elevator"}, "floor2": {"电梯": "lift2"}},
        waypoints={"ug": {"elevator": {}}, "floor2": {"lift2": {}}},
    )
    assert resolver.available_destinations("ug") == ["elevator", "电梯"]
    with pytest.raises(UnknownDestinationError):
        resolver.resolve("电梯", "missing")


def test_parse_intent_rejects_illegal_destination():
    with pytest.raises(IntentValidationError):
        parse_intent_payload(
            {"intent": "navigate", "destination": "任意坐标", "confidence": 0.9},
            allowed_destinations=["电梯", "home"],
        )


def test_parse_intent_accepts_cancel_and_status_without_destination():
    cancel = parse_intent_payload(
        {"intent": "cancel_navigation", "confidence": 0.8},
        allowed_destinations=["电梯"],
    )
    status = parse_intent_payload(
        {"intent": "navigation_status", "confidence": 0.8},
        allowed_destinations=["电梯"],
    )
    assert cancel.intent == "cancel_navigation"
    assert status.intent == "navigation_status"


def test_parse_intent_rejects_invalid_confidence_and_sequence():
    with pytest.raises(IntentValidationError):
        parse_intent_payload(
            {"intent": "navigate", "destination": "电梯", "confidence": 1.2},
            allowed_destinations=["电梯"],
        )
    with pytest.raises(IntentValidationError):
        parse_intent_payload(
            {"intent": "navigate", "destination": "电梯", "sequence": "电梯"},
            allowed_destinations=["电梯"],
        )
