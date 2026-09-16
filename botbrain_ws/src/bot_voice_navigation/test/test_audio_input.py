from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "bot_navigation"))

from bot_voice_navigation.audio_input import (  # noqa: E402
    NativeAsrSubscriber,
    normalize_asr_topics,
)


def test_native_asr_topic_configuration_is_deduplicated():
    assert normalize_asr_topics(["audio_msg", "/rt/audio_msg", "/audio_msg"]) == (
        "/audio_msg",
        "/rt/audio_msg",
    )


def test_native_asr_subscriber_uses_unitree_audio_topic_and_closes():
    calls = []
    messages = []

    class FakeSubscriber:
        def __init__(self, topic, message_type):
            calls.append(("create", topic, message_type))

        def Init(self, handler, queue_len):
            calls.append(("init", queue_len))
            handler(type("Message", (), {"data": "去办公室一"})())

        def Close(self):
            calls.append(("close",))

    subscriber = NativeAsrSubscriber(
        messages.append,
        network_interface="enP8p1s0",
        channel_initializer=lambda domain, interface: calls.append(
            ("channel", domain, interface)
        ),
        subscriber_factory=FakeSubscriber,
        message_type=object,
    )
    subscriber.start()
    subscriber.close()
    assert messages == ["去办公室一"]
    assert calls == [
        ("channel", 0, "enP8p1s0"),
        ("create", "rt/audio_msg", object),
        ("init", 10),
        ("close",),
    ]
