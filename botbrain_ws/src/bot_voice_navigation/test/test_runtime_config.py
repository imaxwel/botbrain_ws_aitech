from __future__ import annotations

import sys
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAVIGATION_ROOT = PACKAGE_ROOT.parent / "bot_navigation"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(NAVIGATION_ROOT))

from bot_voice_navigation.runtime_config import (  # noqa: E402
    VoiceNavigationConfig,
    load_voice_navigation_config,
    validate_llm_configuration,
)


def test_load_voice_navigation_config_reads_project_yaml_and_tts_default(tmp_path):
    config_file = tmp_path / "robot_config.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "robot_configuration": {
                    "voice_navigation": {
                        "llm_endpoint": "https://llm.example/v1/chat/completions",
                        "llm_api_key": "test-key",
                        "llm_model": "test-model",
                        "tts_endpoint": "",
                        "tts_api_key": "",
                        "tts_model": "",
                        "tts_voice": "",
                    }
                }
            },
            allow_unicode=True,
        ),
        encoding="utf-8",
    )

    config = load_voice_navigation_config(config_file, environ={})

    assert config.llm_endpoint == "https://llm.example/v1/chat/completions"
    assert config.llm_api_key == "test-key"
    assert config.llm_model == "test-model"
    assert config.tts_endpoint == ""
    assert config.tts_voice == "zh-CN"


def test_environment_can_override_project_config_for_compatibility(tmp_path):
    config_file = tmp_path / "robot_config.yaml"
    config_file.write_text(
        "robot_configuration:\n"
        "  voice_navigation:\n"
        "    llm_endpoint: https://project.example/v1/chat/completions\n"
        "    llm_api_key: project-key\n"
        "    llm_model: project-model\n",
        encoding="utf-8",
    )

    config = load_voice_navigation_config(
        config_file,
        environ={"G1_LLM_ENDPOINT": "https://override.example/v1/chat/completions"},
    )

    assert config.llm_endpoint == "https://override.example/v1/chat/completions"
    assert config.llm_api_key == "project-key"
    assert config.llm_model == "project-model"


def test_empty_environment_value_does_not_erase_project_config(tmp_path):
    config_file = tmp_path / "robot_config.yaml"
    config_file.write_text(
        "robot_configuration:\n"
        "  voice_navigation:\n"
        "    llm_endpoint: https://project.example/v1/chat/completions\n"
        "    llm_api_key: project-key\n"
        "    llm_model: project-model\n",
        encoding="utf-8",
    )

    config = load_voice_navigation_config(
        config_file,
        environ={"G1_LLM_ENDPOINT": ""},
    )

    assert config.llm_endpoint == "https://project.example/v1/chat/completions"


def test_validate_llm_configuration_names_every_missing_required_field():
    config = VoiceNavigationConfig(
        llm_endpoint="https://llm.example/v1/chat/completions"
    )

    try:
        validate_llm_configuration(config)
    except ValueError as error:
        message = str(error)
    else:
        raise AssertionError("incomplete LLM configuration must be rejected")

    assert "llm_api_key" in message
    assert "llm_model" in message
    assert "tts_endpoint" not in message


def test_yaml_null_values_remain_unconfigured(tmp_path):
    config_file = tmp_path / "robot_config.yaml"
    config_file.write_text(
        "robot_configuration:\n"
        "  voice_navigation:\n"
        "    llm_endpoint:\n"
        "    llm_api_key:\n"
        "    llm_model:\n",
        encoding="utf-8",
    )

    config = load_voice_navigation_config(config_file, environ={})

    assert config.llm_endpoint == ""
    assert config.llm_api_key == ""
    assert config.llm_model == ""
