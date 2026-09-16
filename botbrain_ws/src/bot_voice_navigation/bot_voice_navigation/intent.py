"""Strict validation for cloud-produced navigation intents."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


class IntentValidationError(ValueError):
    """Raised when a cloud response violates the local intent contract."""


VALID_INTENTS = {"navigate", "cancel_navigation", "navigation_status", "unknown"}


@dataclass(frozen=True)
class NavigationContext:
    scene: str
    available_destinations: list[str]
    active_task_id: str | None
    navigation_state: str


@dataclass(frozen=True)
class NavigationIntent:
    intent: str
    destination: str = ""
    sequence: tuple[str, ...] = ()
    confidence: float = 0.0
    need_confirmation: bool = False


def parse_intent_payload(
    payload: Any,
    *,
    allowed_destinations: list[str] | tuple[str, ...] | set[str],
) -> NavigationIntent:
    if not isinstance(payload, dict):
        raise IntentValidationError("intent payload must be an object")
    intent = payload.get("intent")
    if intent not in VALID_INTENTS:
        raise IntentValidationError(f"unsupported intent: {intent!r}")
    confidence = payload.get("confidence", 0.0)
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise IntentValidationError("confidence must be numeric")
    confidence = float(confidence)
    if not 0.0 <= confidence <= 1.0:
        raise IntentValidationError("confidence must be between 0 and 1")
    allowed = {str(item).strip() for item in allowed_destinations}
    destination = payload.get("destination", "")
    if not isinstance(destination, str):
        raise IntentValidationError("destination must be a string")
    destination = destination.strip()
    sequence_value = payload.get("sequence", [])
    if not isinstance(sequence_value, list):
        raise IntentValidationError("sequence must be an array")
    sequence: list[str] = []
    for item in sequence_value:
        if not isinstance(item, str) or not item.strip():
            raise IntentValidationError("sequence entries must be non-empty strings")
        sequence.append(item.strip())
    if intent == "navigate":
        if not destination:
            raise IntentValidationError("navigate intent requires destination")
        if destination not in allowed:
            raise IntentValidationError(f"destination is not allowed: {destination!r}")
        illegal = [item for item in sequence if item not in allowed]
        if illegal:
            raise IntentValidationError(f"sequence contains illegal destinations: {illegal}")
    else:
        destination = ""
        sequence = []
    need_confirmation = payload.get("need_confirmation", False)
    if not isinstance(need_confirmation, bool):
        raise IntentValidationError("need_confirmation must be boolean")
    return NavigationIntent(
        intent=intent,
        destination=destination,
        sequence=tuple(sequence),
        confidence=confidence,
        need_confirmation=need_confirmation,
    )
