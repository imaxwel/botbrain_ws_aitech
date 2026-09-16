"""Cloud LLM and TTS adapters with injectable transports for soft tests."""

from __future__ import annotations

import asyncio
import base64
import json
import re
from typing import Any, Callable
from urllib.request import Request, urlopen

from .intent import (
    IntentValidationError,
    NavigationContext,
    NavigationIntent,
    parse_intent_payload,
)


class CloudLLMError(RuntimeError):
    """Raised when a cloud response cannot be used safely."""


def _default_opener(request: Request, timeout: float):
    return urlopen(request, timeout=timeout)


def _read_response(response: Any) -> Any:
    if isinstance(response, (dict, list, bytes, bytearray, str)):
        return response
    body = response.read()
    if isinstance(body, bytes):
        try:
            return json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return body
    return json.loads(body)


def _json_content(value: Any) -> dict[str, Any]:
    if isinstance(value, dict) and "intent" in value:
        return value
    if isinstance(value, dict):
        choices = value.get("choices")
        if isinstance(choices, list) and choices:
            first = choices[0]
            if isinstance(first, dict):
                message = first.get("message") or first.get("text")
                if isinstance(message, dict):
                    value = message.get("content", "")
                else:
                    value = message
        elif "output_text" in value:
            value = value["output_text"]
    if not isinstance(value, str):
        raise CloudLLMError("cloud response does not contain JSON content")
    text = value.strip()
    fenced = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", text, re.DOTALL | re.IGNORECASE)
    if fenced:
        text = fenced.group(1).strip()
    try:
        result = json.loads(text)
    except json.JSONDecodeError as error:
        raise CloudLLMError(f"cloud response is not valid JSON: {error}") from error
    if not isinstance(result, dict):
        raise CloudLLMError("cloud intent response must be an object")
    return result


class OpenAICompatibleLLM:
    """Call an OpenAI-compatible chat endpoint and enforce local intent rules."""

    def __init__(
        self,
        endpoint: str,
        api_key: str,
        model: str,
        *,
        timeout: float = 20.0,
        opener: Callable[[Request, float], Any] | None = None,
    ):
        self.endpoint = endpoint
        self.api_key = api_key
        self.model = model
        self.timeout = max(0.1, float(timeout))
        self.opener = opener or _default_opener
        self._threaded_opener = opener is None

    async def parse_navigation_intent(
        self,
        transcript: str,
        context: NavigationContext,
    ) -> NavigationIntent:
        if not str(transcript).strip():
            raise CloudLLMError("transcript is empty")
        system = (
            "Return JSON only. Allowed intents: navigate, cancel_navigation, "
            "navigation_status, unknown. Select destinations only from this list: "
            f"{json.dumps(context.available_destinations, ensure_ascii=False)}. "
            "Never return coordinates, shell commands, ROS commands, or cmd_vel. "
            "Use need_confirmation=true when the destination is unclear."
        )
        body = {
            "model": self.model,
            "temperature": 0,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": str(transcript).strip()},
            ],
            "response_format": {"type": "json_object"},
        }
        request = Request(
            self.endpoint,
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            response = (
                await asyncio.to_thread(self.opener, request, self.timeout)
                if self._threaded_opener
                else self.opener(request, self.timeout)
            )
            payload = _read_response(response)
            parsed = _json_content(payload)
            return parse_intent_payload(
                parsed,
                allowed_destinations=context.available_destinations,
            )
        except (CloudLLMError, IntentValidationError):
            raise
        except Exception as error:
            raise CloudLLMError(str(error)) from error


class CloudTTSClient:
    """Call a simple cloud TTS endpoint and return raw PCM bytes."""

    def __init__(
        self,
        endpoint: str,
        api_key: str,
        voice: str,
        *,
        model: str = "",
        timeout: float = 20.0,
        opener: Callable[[Request, float], Any] | None = None,
    ):
        self.endpoint = endpoint
        self.api_key = api_key
        self.voice = voice
        self.model = model
        self.timeout = max(0.1, float(timeout))
        self.opener = opener or _default_opener
        self._threaded_opener = opener is None

    async def synthesize(self, text: str) -> bytes:
        if not str(text).strip():
            raise CloudLLMError("TTS text is empty")
        body = {"input": str(text).strip(), "voice": self.voice, "response_format": "pcm_s16le"}
        if self.model:
            body["model"] = self.model
        request = Request(
            self.endpoint,
            data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            response = (
                await asyncio.to_thread(self.opener, request, self.timeout)
                if self._threaded_opener
                else self.opener(request, self.timeout)
            )
            payload = _read_response(response)
            if isinstance(payload, dict):
                encoded = payload.get("audio_base64")
                if encoded is None:
                    encoded = payload.get("audio")
                if not isinstance(encoded, str):
                    raise CloudLLMError("TTS response has no audio_base64 field")
                if not encoded:
                    raise CloudLLMError("empty audio")
                audio = base64.b64decode(encoded, validate=True)
            elif isinstance(payload, str):
                audio = base64.b64decode(payload, validate=True)
            else:
                audio = bytes(payload)
            if not audio:
                raise CloudLLMError("empty audio")
            if len(audio) % 2:
                raise CloudLLMError("PCM16 audio has odd byte length")
            return audio
        except CloudLLMError:
            raise
        except Exception as error:
            raise CloudLLMError(str(error)) from error
