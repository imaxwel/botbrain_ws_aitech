from __future__ import annotations

import asyncio
import base64
import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_ROOT = PACKAGE_ROOT.parent / "bot_navigation"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(NAVIGATION_ROOT))

from bot_voice_navigation.cloud import (  # noqa: E402
    CloudLLMError,
    CloudTTSClient,
    OpenAICompatibleLLM,
)
from bot_voice_navigation.intent import NavigationContext, IntentValidationError  # noqa: E402
from bot_voice_navigation.speech_output import SpeechDispatcher, SpeechOutput  # noqa: E402


def test_openai_compatible_llm_parses_structured_response_without_network():
    captured = {}

    def opener(request, timeout):
        captured["body"] = json.loads(request.data.decode("utf-8"))
        return {
            "choices": [
                {
                    "message": {
                        "content": '{"intent":"navigate","destination":"电梯","confidence":0.9}'
                    }
                }
            ]
        }

    llm = OpenAICompatibleLLM(
        endpoint="https://llm.example/v1/chat/completions",
        api_key="secret",
        model="cloud-model",
        opener=opener,
    )
    result = asyncio.run(llm.parse_navigation_intent(
        "带我去电梯",
        NavigationContext("ug", ["电梯"], None, "IDLE"),
    ))
    assert result.intent == "navigate"
    assert result.destination == "电梯"
    assert captured["body"]["model"] == "cloud-model"
    assert captured["body"]["messages"][-1]["content"] == "带我去电梯"


def test_openai_compatible_llm_rejects_illegal_destination():
    def opener(request, timeout):
        return {"intent": "navigate", "destination": "任意坐标", "confidence": 0.9}

    llm = OpenAICompatibleLLM("https://llm.example", "key", "model", opener=opener)
    with pytest.raises(IntentValidationError):
        asyncio.run(llm.parse_navigation_intent(
            "去那里", NavigationContext("ug", ["电梯"], None, "IDLE")
        ))


def test_openai_compatible_llm_wraps_http_failures():
    def opener(request, timeout):
        raise OSError("offline")

    llm = OpenAICompatibleLLM("https://llm.example", "key", "model", opener=opener)
    with pytest.raises(CloudLLMError, match="offline"):
        asyncio.run(llm.parse_navigation_intent(
            "去电梯", NavigationContext("ug", ["电梯"], None, "IDLE")
        ))


def test_cloud_tts_decodes_base64_audio_and_rejects_empty_audio():
    pcm = b"\x00\x00\x01\x00"
    client = CloudTTSClient(
        endpoint="https://tts.example",
        api_key="key",
        voice="zh",
        opener=lambda request, timeout: {
            "audio_base64": base64.b64encode(pcm).decode("ascii")
        },
    )
    assert asyncio.run(client.synthesize("正在前往电梯")) == pcm

    empty = CloudTTSClient(
        endpoint="https://tts.example",
        api_key="key",
        voice="zh",
        opener=lambda request, timeout: {"audio_base64": ""},
    )
    with pytest.raises(CloudLLMError, match="empty audio"):
        asyncio.run(empty.synthesize("无"))


def test_speech_output_calls_sink_and_does_not_block_event_loop():
    calls = []

    class FakeTTS:
        async def synthesize(self, text):
            return b"\x00\x00"

    class FakeSink:
        async def play(self, pcm, request_id):
            calls.append((pcm, request_id))

    output = SpeechOutput(FakeTTS(), FakeSink())
    asyncio.run(output.speak("已到达", request_id="task-1"))
    assert calls == [(b"\x00\x00", "task-1")]


def test_cloud_tts_accepts_raw_pcm_http_response():
    class RawResponse:
        def read(self):
            return b"\x00\x00\x01\x00"

    client = CloudTTSClient(
        endpoint="https://tts.example",
        api_key="key",
        voice="zh",
        opener=lambda request, timeout: RawResponse(),
    )
    assert asyncio.run(client.synthesize("原始 PCM")) == b"\x00\x00\x01\x00"


def test_unitree_sink_sends_fixed_size_pcm_chunks():
    calls = []

    class FakeClient:
        def PlayStream(self, app_name, stream_id, chunk):
            calls.append((app_name, stream_id, chunk))
            return 0

        def PlayStop(self, app_name):
            return 0

    from bot_voice_navigation.speech_output import UnitreeAudioSink

    sink = UnitreeAudioSink(client=FakeClient(), chunk_ms=10)
    asyncio.run(sink.play(b"\x00\x00" * 1600, "task-1"))
    assert [len(item[2]) for item in calls] == [320] * 10
    assert all(item[0] == "botbrain_voice_navigation" for item in calls)
    assert all(item[1] == "task-1" for item in calls)


def test_unitree_sink_serializes_playback_across_event_loops():
    import threading
    import time

    active = 0
    maximum = 0
    guard = threading.Lock()

    class SlowClient:
        def PlayStream(self, app_name, stream_id, chunk):
            nonlocal active, maximum
            with guard:
                active += 1
                maximum = max(maximum, active)
            time.sleep(0.01)
            with guard:
                active -= 1
            return 0

        def PlayStop(self, app_name):
            return 0

    from bot_voice_navigation.speech_output import UnitreeAudioSink

    sink = UnitreeAudioSink(client=SlowClient(), chunk_ms=10)
    errors = []

    def run():
        try:
            asyncio.run(sink.play(b"\x00\x00" * 160, "task"))
        except Exception as error:
            errors.append(error)

    first = threading.Thread(target=run)
    second = threading.Thread(target=run)
    first.start()
    second.start()
    first.join()
    second.join()
    assert errors == []
    assert maximum == 1


def test_speech_dispatcher_preserves_order_and_suppresses_asr_during_playback():
    import threading

    started = threading.Event()
    release = threading.Event()
    calls = []

    class FakeSpeech:
        async def speak(self, text):
            calls.append(text)
            started.set()
            while not release.is_set():
                await asyncio.sleep(0.01)

    dispatcher = SpeechDispatcher(FakeSpeech(), cooldown_seconds=0.0)
    dispatcher.enqueue("正在前往办公室一")
    dispatcher.enqueue("已到达办公室一")
    assert started.wait(1.0)
    assert dispatcher.asr_suppressed() is True
    release.set()
    dispatcher.close(timeout=2.0)
    assert calls == ["正在前往办公室一", "已到达办公室一"]


def test_speech_dispatcher_continues_after_one_failed_utterance():
    calls = []
    errors = []

    class FakeSpeech:
        async def speak(self, text):
            calls.append(text)
            if text == "失败":
                raise RuntimeError("tts offline")

    dispatcher = SpeechDispatcher(
        FakeSpeech(), cooldown_seconds=0.0, error_callback=errors.append
    )
    dispatcher.enqueue("失败")
    dispatcher.enqueue("继续")
    dispatcher.close(timeout=2.0)
    assert calls == ["失败", "继续"]
    assert len(errors) == 1
    assert "tts offline" in str(errors[0])
