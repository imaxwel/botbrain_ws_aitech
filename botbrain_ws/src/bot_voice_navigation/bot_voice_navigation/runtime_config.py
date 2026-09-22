"""Load voice-navigation cloud settings from BotBrain robot_config.yaml."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


@dataclass(frozen=True)
class VoiceNavigationConfig:
    llm_endpoint: str = ""
    llm_api_key: str = ""
    llm_model: str = ""
    tts_endpoint: str = ""
    tts_api_key: str = ""
    tts_model: str = ""
    tts_voice: str = "zh-CN"


def _text(value: object) -> str:
    return "" if value is None else str(value).strip()


def load_voice_navigation_config(
    path: Path | str,
    *,
    environ: Mapping[str, str] | None = None,
) -> VoiceNavigationConfig:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    robot = data.get("robot_configuration", {})
    if not isinstance(robot, dict):
        raise ValueError("robot_configuration must be a YAML mapping")
    voice = robot.get("voice_navigation", {}) or {}
    if not isinstance(voice, dict):
        raise ValueError("robot_configuration.voice_navigation must be a YAML mapping")

    values = {
        "llm_endpoint": _text(voice.get("llm_endpoint", "")),
        "llm_api_key": _text(voice.get("llm_api_key", "")),
        "llm_model": _text(voice.get("llm_model", "")),
        "tts_endpoint": _text(voice.get("tts_endpoint", "")),
        "tts_api_key": _text(voice.get("tts_api_key", "")),
        "tts_model": _text(voice.get("tts_model", "")),
        "tts_voice": _text(voice.get("tts_voice", "")) or "zh-CN",
    }
    source = os.environ if environ is None else environ
    overrides = {
        "llm_endpoint": "G1_LLM_ENDPOINT",
        "llm_api_key": "G1_LLM_API_KEY",
        "llm_model": "G1_LLM_MODEL",
        "tts_endpoint": "G1_TTS_ENDPOINT",
        "tts_api_key": "G1_TTS_API_KEY",
        "tts_model": "G1_TTS_MODEL",
        "tts_voice": "G1_TTS_VOICE",
    }
    for field, environment_name in overrides.items():
        if environment_name in source:
            override = _text(source[environment_name])
            if override:
                values[field] = override
    values["tts_voice"] = values["tts_voice"] or "zh-CN"
    return VoiceNavigationConfig(**values)


def validate_llm_configuration(config: VoiceNavigationConfig) -> None:
    required = {
        "llm_endpoint": config.llm_endpoint,
        "llm_api_key": config.llm_api_key,
        "llm_model": config.llm_model,
    }
    missing = [name for name, value in required.items() if not value]
    if missing:
        fields = ", ".join(f"voice_navigation.{name}" for name in missing)
        raise ValueError(f"robot_config.yaml missing required fields: {fields}")
