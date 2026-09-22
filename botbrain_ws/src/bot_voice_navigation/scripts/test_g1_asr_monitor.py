#!/usr/bin/env python3
"""Print the first final Unitree ASR transcript and exit."""

from __future__ import annotations

import argparse
import os
import sys
import threading
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAV_ROOT = PACKAGE_ROOT.parent / "bot_navigation"
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(NAV_ROOT))

from bot_voice_navigation.asr_parser import parse_asr_message  # noqa: E402
from bot_voice_navigation.audio_input import NativeAsrSubscriber  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "只读监听 Unitree rt/audio_msg，打印第一条 final 文本；不调用 LLM 或导航。"
        )
    )
    parser.add_argument(
        "--interface",
        default=os.environ.get("G1_DDS_INTERFACE", "enP8p1s0"),
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    received = threading.Event()

    def handle_message(raw_text: str) -> None:
        transcript = parse_asr_message(raw_text)
        if transcript is None or not transcript.final:
            return
        print(f"ASR final text: {transcript.text}", flush=True)
        received.set()

    subscriber = NativeAsrSubscriber(handle_message, args.interface)
    try:
        subscriber.start()
        print(
            f"Listening on Unitree rt/audio_msg via {args.interface}; "
            "不调用 LLM 或导航。",
            flush=True,
        )
        if not received.wait(max(0.1, args.timeout)):
            print("TIMEOUT: 未收到 final ASR 文本", file=sys.stderr)
            return 2
        return 0
    finally:
        subscriber.close()


if __name__ == "__main__":
    raise SystemExit(main())
