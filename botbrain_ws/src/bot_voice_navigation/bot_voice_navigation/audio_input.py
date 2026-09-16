"""Configuration helpers for G1 native ASR text topics."""

from __future__ import annotations

import threading
from typing import Any, Callable


def normalize_asr_topics(value: Any) -> tuple[str, ...]:
    """Normalize ROS topic parameters while preserving stable subscription order."""
    if isinstance(value, str):
        values = [item.strip() for item in value.split(",")]
    elif isinstance(value, (list, tuple)):
        values = [str(item).strip() for item in value]
    else:
        values = []
    topics: list[str] = []
    for topic in values:
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = f"/{topic}"
        if topic not in topics:
            topics.append(topic)
    return tuple(topics)


class NativeAsrSubscriber:
    """Subscribe to the official Unitree DDS ASR text topic."""

    def __init__(
        self,
        callback: Callable[[str], None],
        network_interface: str = "enP8p1s0",
        *,
        channel_initializer: Callable[..., Any] | None = None,
        subscriber_factory: Callable[..., Any] | None = None,
        message_type: Any | None = None,
        queue_len: int = 10,
    ):
        self.callback = callback
        self.network_interface = str(network_interface).strip()
        self._channel_initializer = channel_initializer
        self._subscriber_factory = subscriber_factory
        self._message_type = message_type
        self.queue_len = max(1, int(queue_len))
        self._subscriber = None
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
            if self._subscriber is not None:
                return
            if self._channel_initializer is None:
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize

                self._channel_initializer = ChannelFactoryInitialize
            if self._subscriber_factory is None or self._message_type is None:
                from unitree_sdk2py.core.channel import ChannelSubscriber
                from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

                self._subscriber_factory = ChannelSubscriber
                self._message_type = String_
            self._channel_initializer(0, self.network_interface)
            self._subscriber = self._subscriber_factory(
                "rt/audio_msg", self._message_type
            )
            self._subscriber.Init(self._on_message, self.queue_len)

    def _on_message(self, message: Any) -> None:
        value = getattr(message, "data", message)
        if isinstance(value, str) and value.strip():
            self.callback(value)

    def close(self) -> None:
        with self._lock:
            if self._subscriber is not None:
                self._subscriber.Close()
                self._subscriber = None
