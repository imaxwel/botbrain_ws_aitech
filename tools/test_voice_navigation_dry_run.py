#!/usr/bin/env python3
"""Dry-run the voice-navigation chain without sending a navigation goal."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VOICE_ROOT = ROOT / "botbrain_ws" / "src" / "bot_voice_navigation"
NAV_ROOT = ROOT / "botbrain_ws" / "src" / "bot_navigation"
sys.path.insert(0, str(VOICE_ROOT))
sys.path.insert(0, str(NAV_ROOT))

from bot_voice_navigation.asr_parser import parse_asr_message  # noqa: E402
from bot_voice_navigation.cloud import CloudLLMError, OpenAICompatibleLLM  # noqa: E402
from bot_voice_navigation.destinations import DestinationResolver  # noqa: E402
from bot_voice_navigation.intent import NavigationContext, parse_intent_payload  # noqa: E402
from bot_voice_navigation.runtime_config import (  # noqa: E402
    load_voice_navigation_config,
    validate_llm_configuration,
)
from bot_navigation.waypoint_controller import WaypointNavigationController  # noqa: E402


DEFAULT_INTENT = {
    "intent": "navigate",
    "destination": "办公室一",
    "sequence": [],
    "confidence": 0.99,
    "need_confirmation": False,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="语音导航链路干跑：只打印导航命令，绝不执行导航。"
    )
    parser.add_argument("--transcript", default="带我去办公室一")
    parser.add_argument("--scene", default=os.environ.get("MAP_SCENE", "ug"))
    parser.add_argument("--robot", default="g1_robot")
    parser.add_argument(
        "--intent-json",
        default=json.dumps(DEFAULT_INTENT, ensure_ascii=False),
        help="mock LLM JSON；使用 --llm 时忽略",
    )
    parser.add_argument(
        "--llm",
        action="store_true",
        help="调用 robot_config.yaml 中的 LLM，但仍不发送 ROS Action 或执行导航脚本",
    )
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=ROOT / "botbrain_ws" / "robot_config.yaml",
    )
    parser.add_argument(
        "--waypoint-file",
        type=Path,
        default=NAV_ROOT / "nav_waypoints.yaml",
    )
    parser.add_argument(
        "--aliases-file",
        type=Path,
        default=VOICE_ROOT / "config" / "voice_destinations.yaml",
    )
    parser.add_argument(
        "--navigator-script",
        type=Path,
        default=NAV_ROOT / "scripts" / "waypoint_navigator.py",
    )
    return parser.parse_args()


async def parse_with_llm(
    transcript: str,
    resolver: DestinationResolver,
    scene: str,
    robot_config: Path,
):
    config = load_voice_navigation_config(robot_config)
    validate_llm_configuration(config)
    llm = OpenAICompatibleLLM(
        endpoint=config.llm_endpoint,
        api_key=config.llm_api_key,
        model=config.llm_model,
    )
    context = NavigationContext(
        scene=scene,
        available_destinations=resolver.available_destinations(scene),
        active_task_id=None,
        navigation_state="IDLE",
    )
    return await llm.parse_navigation_intent(transcript, context)


def main() -> int:
    args = parse_args()
    print("DRY-RUN ONLY: 不启动 ROS Action，不调用导航脚本，不会让机器人运动")

    transcript = parse_asr_message(args.transcript)
    if transcript is None or not transcript.final or not transcript.text.strip():
        raise SystemExit("ASR 文本不是非空 final transcript")
    print(f"ASR final text: {transcript.text}")

    resolver = DestinationResolver.from_files(args.aliases_file, args.waypoint_file)
    allowed = resolver.available_destinations(args.scene)
    if not allowed:
        raise SystemExit(f"场景 {args.scene!r} 没有可用点位")
    print(f"scene: {args.scene}")
    print(f"allowed destinations: {json.dumps(allowed, ensure_ascii=False)}")

    if args.llm:
        intent = asyncio.run(
            parse_with_llm(transcript.text, resolver, args.scene, args.robot_config)
        )
        print("intent source: live LLM response")
    else:
        try:
            payload = json.loads(args.intent_json)
        except json.JSONDecodeError as error:
            raise SystemExit(f"--intent-json 不是有效 JSON: {error}") from error
        intent = parse_intent_payload(payload, allowed_destinations=allowed)
        print("intent source: mock intent JSON")

    print(f"intent: {intent.intent}, confidence={intent.confidence:.2f}")
    if intent.intent != "navigate" or intent.need_confirmation:
        raise SystemExit("意图不是无需确认的 navigate，安全停止")

    names = [resolver.resolve(intent.destination, args.scene).waypoint_name]
    names.extend(
        resolver.resolve(item, args.scene).waypoint_name for item in intent.sequence
    )
    controller = WaypointNavigationController(
        args.navigator_script,
        robot=args.robot,
    )
    command = controller.build_command(names, args.scene)
    print(f"resolved waypoints: {json.dumps(names, ensure_ascii=False)}")
    print("would execute (NOT executed):")
    print(json.dumps(command, ensure_ascii=False))
    print("PASS: ASR -> intent -> scene/alias mapping -> navigator command dry-run")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (CloudLLMError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from None
