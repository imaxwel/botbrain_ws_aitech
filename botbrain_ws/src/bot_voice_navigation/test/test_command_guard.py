from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "bot_navigation"))

from bot_voice_navigation.command_guard import CommandCancellationGuard  # noqa: E402


def test_local_cancel_invalidates_only_older_commands():
    guard = CommandCancellationGuard()
    old = guard.snapshot()
    guard.cancel_prior()
    new = guard.snapshot()
    assert guard.is_current(old) is False
    assert guard.is_current(new) is True
