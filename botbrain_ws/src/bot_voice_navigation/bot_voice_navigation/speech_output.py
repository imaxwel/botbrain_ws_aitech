"""Speech output boundary for cloud TTS and the G1 factory speaker."""

from __future__ import annotations

import asyncio
import queue
import threading
import time
import uuid
from typing import Any, Callable, Protocol


class TTSClient(Protocol):
    async def synthesize(self, text: str) -> bytes:
        ...


class AudioSink(Protocol):
    async def play(self, pcm: bytes, request_id: str) -> None:
        ...


class SpeechOutput:
    def __init__(self, tts_client: TTSClient, sink: AudioSink):
        self.tts_client = tts_client
        self.sink = sink

    async def speak(self, text: str, request_id: str | None = None) -> str:
        request_id = request_id or str(uuid.uuid4())
        pcm = await self.tts_client.synthesize(text)
        await self.sink.play(pcm, request_id)
        return request_id


class SpeechDispatcher:
    """Play speech in FIFO order and expose a half-duplex ASR guard."""

    def __init__(
        self,
        speech: SpeechOutput,
        cooldown_seconds: float = 0.2,
        error_callback: Callable[[Exception], None] | None = None,
    ):
        self.speech = speech
        self.cooldown_seconds = max(0.0, float(cooldown_seconds))
        self.error_callback = error_callback
        self._queue: queue.Queue[str | None] = queue.Queue(maxsize=16)
        self._speaking = threading.Event()
        self._suppress_until = 0.0
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="voice-speech-output",
            daemon=True,
        )
        self._thread.start()

    def enqueue(self, text: str) -> bool:
        if self._closed or not str(text).strip():
            return False
        try:
            self._queue.put_nowait(str(text).strip())
            return True
        except queue.Full:
            return False

    def asr_suppressed(self) -> bool:
        return self._speaking.is_set() or time.monotonic() < self._suppress_until

    def _run(self) -> None:
        while True:
            text = self._queue.get()
            try:
                if text is None:
                    return
                self._speaking.set()
                try:
                    asyncio.run(self.speech.speak(text))
                except Exception as error:
                    if self.error_callback is not None:
                        self.error_callback(error)
            finally:
                self._speaking.clear()
                self._suppress_until = time.monotonic() + self.cooldown_seconds
                self._queue.task_done()

    def close(self, timeout: float = 2.0) -> None:
        if self._closed:
            return
        self._closed = True
        deadline = time.monotonic() + max(0.0, float(timeout))
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(0.01)
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            # A stuck TTS call is isolated in this daemon worker; never block
            # ROS shutdown behind a full speech queue.
            return
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))


class UnitreeAudioSink:
    """Serialize PCM playback through one official G1 AudioClient instance."""

    def __init__(
        self,
        network_interface: str = "enP8p1s0",
        app_name: str = "botbrain_voice_navigation",
        chunk_ms: int = 50,
        client: Any | None = None,
    ):
        self.network_interface = network_interface
        self.app_name = app_name
        self.chunk_ms = min(3000, max(10, int(chunk_ms)))
        self.chunk_bytes = 16000 * 2 * self.chunk_ms // 1000
        self._client = client
        self._threaded_client = client is None
        # Playback can be invoked from several node threads, each with its own
        # event loop. A threading lock is required; asyncio.Lock is loop-bound.
        self._lock = threading.Lock()

    def _ensure_client(self) -> Any:
        if self._client is None:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

            ChannelFactoryInitialize(0, self.network_interface)
            self._client = AudioClient()
            self._client.SetTimeout(10.0)
            self._client.Init()
        return self._client

    async def _acquire_playback_lock(self) -> None:
        while not self._lock.acquire(blocking=False):
            await asyncio.sleep(0.01)

    async def play(self, pcm: bytes, request_id: str) -> None:
        if len(pcm) % 2:
            raise ValueError("PCM16 audio has odd byte length")
        await self._acquire_playback_lock()
        try:
            client = (
                await asyncio.to_thread(self._ensure_client)
                if self._threaded_client
                else self._ensure_client()
            )
            for offset in range(0, len(pcm), self.chunk_bytes):
                chunk = pcm[offset:offset + self.chunk_bytes]
                result = (
                    await asyncio.to_thread(
                        client.PlayStream,
                        self.app_name,
                        str(request_id),
                        chunk,
                    )
                    if self._threaded_client
                    else client.PlayStream(self.app_name, str(request_id), chunk)
                )
                if isinstance(result, tuple) and result and result[0] != 0:
                    raise RuntimeError(f"Unitree PlayStream failed: {result[0]}")
                if isinstance(result, int) and result != 0:
                    raise RuntimeError(f"Unitree PlayStream failed: {result}")
                await asyncio.sleep(len(chunk) / (16000 * 2))
        finally:
            self._lock.release()

    async def stop(self) -> None:
        await self._acquire_playback_lock()
        try:
            if self._client is not None:
                if self._threaded_client:
                    await asyncio.to_thread(self._client.PlayStop, self.app_name)
                else:
                    self._client.PlayStop(self.app_name)
        finally:
            self._lock.release()
