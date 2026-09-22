from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "tools" / "test_voice_navigation_dry_run.py"
ASR_MONITOR = (
    ROOT
    / "botbrain_ws"
    / "src"
    / "bot_voice_navigation"
    / "scripts"
    / "test_g1_asr_monitor.py"
)


def test_dry_run_prints_resolved_command_without_executing_navigation():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--transcript",
            "带我去办公室一",
            "--scene",
            "ug",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert 'resolved waypoints: ["office1"]' in result.stdout
    assert "would execute (NOT executed):" in result.stdout
    assert "PASS: ASR -> intent -> scene/alias mapping -> navigator command dry-run" in result.stdout


def test_dry_run_rejects_destination_missing_from_active_scene():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--transcript",
            "带我去电梯",
            "--scene",
            "ug",
            "--intent-json",
            '{"intent":"navigate","destination":"电梯","sequence":[],"confidence":0.99,"need_confirmation":false}',
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert "destination is not allowed" in result.stderr
    assert "Traceback" not in result.stderr
    assert "would execute" not in result.stdout


def test_live_llm_mode_requires_explicit_endpoint():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--llm"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        env={"PATH": os.environ.get("PATH", "")},
    )

    assert result.returncode != 0
    assert "voice_navigation.llm_endpoint" in result.stderr
    assert "would execute" not in result.stdout


def test_native_asr_monitor_help_states_read_only_boundary():
    result = subprocess.run(
        [sys.executable, str(ASR_MONITOR), "--help"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "rt/audio_msg" in result.stdout
    assert "不调用 LLM 或导航" in result.stdout


def test_voice_navigation_compose_uses_project_config_instead_of_env_file():
    compose = yaml.safe_load((ROOT / "docker-compose.yaml").read_text(encoding="utf-8"))
    service = compose["services"]["voice_navigation"]

    assert "env_file" not in service
    overridden = {
        name
        for name in service.get("environment", {})
        if name.startswith("G1_LLM_") or name.startswith("G1_TTS_")
    }
    assert overridden == set()
