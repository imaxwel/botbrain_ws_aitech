"""ROS2 node connecting G1 native ASR text to cloud intent and navigation."""

from __future__ import annotations

import asyncio
import os
import re
import threading
import uuid
from typing import Any

from .audio_input import NativeAsrSubscriber, normalize_asr_topics
from .asr_parser import TranscriptDeduper, VoiceTranscript, parse_asr_message
from .cloud import CloudLLMError, CloudTTSClient, OpenAICompatibleLLM
from .command_guard import CommandCancellationGuard
from .destinations import DestinationResolver
from .intent import IntentValidationError, NavigationContext, NavigationIntent
from .runtime_config import (
    VoiceNavigationConfig,
    load_voice_navigation_config,
    validate_llm_configuration,
)
from .speech_output import SpeechDispatcher, SpeechOutput, UnitreeAudioSink


_LOCAL_CANCEL_PATTERN = re.compile(
    r"^(?:请)?(?:立即|马上)?(?:停止|停止导航|取消|取消导航|别走了|不要走了|停下来|停下|stop|cancel)[。！! ]*$",
    re.IGNORECASE,
)


def is_local_cancel_command(text: str) -> bool:
    """Recognize safety-critical cancel commands without cloud dependency."""
    return bool(_LOCAL_CANCEL_PATTERN.fullmatch(str(text).strip()))


def should_process_transcript(transcript: VoiceTranscript) -> bool:
    """Only final, non-empty ASR events may reach the cloud model."""
    return bool(transcript.final and transcript.text.strip())


def action_goal_fields(
    intent: NavigationIntent,
    *,
    scene: str,
    task_id: str,
) -> dict[str, Any]:
    if intent.intent != "navigate":
        raise ValueError("only navigate intents can create action goals")
    return {
        "destination": intent.destination,
        "sequence": list(intent.sequence),
        "scene": scene,
        "loop": False,
        "request_id": task_id,
    }


def navigation_result_text(status: str, destination: str) -> str:
    status = str(status).upper()
    if status == "SUCCEEDED":
        return f"已到达{destination}"
    if status == "CANCELED":
        return "导航已取消"
    if status == "ABORTED":
        return f"导航失败，未到达{destination}"
    if status == "REJECTED":
        return "无法开始导航，请检查目的地和导航状态"
    return "导航暂时不可用，请稍后再试"


def goal_status_name(status: int) -> str:
    """Convert action_msgs GoalStatus constants to stable text names."""
    return {
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }.get(int(status), "ERROR")


def result_status_name(response: Any) -> str:
    """Read the numeric ROS action status, with a test-friendly fallback."""
    status_code = getattr(response, "status", None)
    if status_code is not None:
        return goal_status_name(status_code)
    result = getattr(response, "result", response)
    return str(getattr(result, "status", "ERROR")).upper()


class _NoopSink:
    async def play(self, _pcm: bytes, _request_id: str) -> None:
        return None


def _make_speech_output(
    config: VoiceNavigationConfig,
    audio_client: Any | None = None,
) -> SpeechOutput | None:
    if not config.tts_endpoint:
        return None
    tts = CloudTTSClient(
        endpoint=config.tts_endpoint,
        api_key=config.tts_api_key or config.llm_api_key,
        voice=config.tts_voice,
        model=config.tts_model,
    )
    sink = UnitreeAudioSink(
        network_interface=os.environ.get("G1_DDS_INTERFACE", "enP8p1s0"),
        app_name=os.environ.get("G1_AUDIO_APP_NAME", "botbrain_voice_navigation"),
        client=audio_client,
    )
    return SpeechOutput(tts, sink)


def main(args=None) -> None:
    """Start the ROS2 voice-navigation process."""
    import rclpy
    from bot_custom_interfaces.action import NavigateToWaypoint
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String

    class VoiceNavigationNode(Node):
        def __init__(self) -> None:
            super().__init__("voice_navigation_input")
            self.declare_parameter(
                "robot", os.environ.get("G1_ROBOT_NAMESPACE", "g1_robot")
            )
            self.declare_parameter(
                "waypoint_file", "/botbrain_ws/src/bot_navigation/nav_waypoints.yaml"
            )
            self.declare_parameter(
                "aliases_file",
                "/botbrain_ws/src/bot_voice_navigation/config/voice_destinations.yaml",
            )
            self.declare_parameter("scene_file", "/botbrain_ws/.runtime/map_scene")
            self.declare_parameter("robot_config_file", "/botbrain_ws/robot_config.yaml")
            self.declare_parameter("asr_topic", "")
            self.declare_parameter("asr_topics", ["/audio_msg", "/rt/audio_msg"])
            self.declare_parameter("enable_native_dds_asr", True)
            self.declare_parameter("dedup_window_seconds", 2.0)
            self.declare_parameter("min_intent_confidence", 0.55)
            self.declare_parameter("speech_cooldown_seconds", 0.25)
            self.robot = str(self.get_parameter("robot").value).strip()
            self.scene_file = os.path.expanduser(str(self.get_parameter("scene_file").value))
            self.resolver = DestinationResolver.from_files(
                os.path.expanduser(str(self.get_parameter("aliases_file").value)),
                os.path.expanduser(str(self.get_parameter("waypoint_file").value)),
            )
            self.deduper = TranscriptDeduper(
                float(self.get_parameter("dedup_window_seconds").value)
            )
            self.min_confidence = float(self.get_parameter("min_intent_confidence").value)
            self.voice_config = load_voice_navigation_config(
                os.path.expanduser(str(self.get_parameter("robot_config_file").value))
            )
            try:
                validate_llm_configuration(self.voice_config)
                self._llm_config_error = ""
            except ValueError as error:
                self._llm_config_error = str(error)
            self.llm = OpenAICompatibleLLM(
                endpoint=self.voice_config.llm_endpoint,
                api_key=self.voice_config.llm_api_key,
                model=self.voice_config.llm_model,
            )
            self.speech = _make_speech_output(self.voice_config)
            self.speech_dispatcher = (
                SpeechDispatcher(
                    self.speech,
                    float(self.get_parameter("speech_cooldown_seconds").value),
                    lambda error: self.get_logger().error(
                        f"speech output failed: {error}"
                    ),
                )
                if self.speech is not None
                else None
            )
            self.action_client = ActionClient(
                self,
                NavigateToWaypoint,
                f"/{self.robot}/navigate_to_waypoint",
            )
            self._active_goal = None
            self._goal_pending = False
            self._cancel_requested = False
            self._active_task_id = None
            self._active_destination = ""
            self._goal_lock = threading.Lock()
            self._command_guard = CommandCancellationGuard()
            self._worker = threading.Thread(target=self._worker_loop, daemon=True)
            self._queue: list[tuple[int, VoiceTranscript]] = []
            self._queue_condition = threading.Condition()
            self._stopping = False
            legacy_topic = str(self.get_parameter("asr_topic").value).strip()
            configured_topics = normalize_asr_topics(
                self.get_parameter("asr_topics").value
            )
            if legacy_topic:
                configured_topics = normalize_asr_topics(legacy_topic)
            self.asr_topics = configured_topics or ("/audio_msg",)
            self._asr_subscriptions = [
                self.create_subscription(String, topic, self._ros_asr_callback, 10)
                for topic in self.asr_topics
            ]
            self._native_asr = None
            if bool(self.get_parameter("enable_native_dds_asr").value):
                try:
                    self._native_asr = NativeAsrSubscriber(
                        self._native_asr_callback,
                        os.environ.get("G1_DDS_INTERFACE", "enP8p1s0"),
                    )
                    self._native_asr.start()
                    self.get_logger().info(
                        "native Unitree DDS ASR subscriber ready on rt/audio_msg"
                    )
                except Exception as error:
                    self._native_asr = None
                    self.get_logger().warning(
                        f"native DDS ASR subscriber unavailable: {error}"
                    )
            self._worker.start()
            self.get_logger().info(
                f"Voice navigation input ready on {', '.join(self.asr_topics)}"
            )

        def _ros_asr_callback(self, message: String) -> None:
            self._handle_asr_text(message.data)

        def _native_asr_callback(self, text: str) -> None:
            self._handle_asr_text(text)

        def _handle_asr_text(self, raw_text: str) -> None:
            transcript = parse_asr_message(raw_text)
            if transcript is None or not should_process_transcript(transcript):
                return
            if is_local_cancel_command(transcript.text):
                self._command_guard.cancel_prior()
                with self._queue_condition:
                    self._queue.clear()
                if self._cancel_active_goal():
                    self._speak("正在取消导航")
                else:
                    self._speak("当前没有进行中的导航")
                return
            if self.speech_dispatcher is not None and self.speech_dispatcher.asr_suppressed():
                return
            if not self.deduper.accepts(transcript):
                return
            with self._queue_condition:
                if len(self._queue) >= 8:
                    self.get_logger().warning("voice navigation queue full; dropping transcript")
                    return
                self._queue.append(
                    (self._command_guard.snapshot(), transcript)
                )
                self._queue_condition.notify()

        def _worker_loop(self) -> None:
            while True:
                with self._queue_condition:
                    while not self._queue and not self._stopping:
                        self._queue_condition.wait()
                    if self._stopping:
                        return
                    generation, transcript = self._queue.pop(0)
                try:
                    self._process_transcript(transcript, generation)
                except Exception as error:
                    self.get_logger().error(f"voice command failed: {error}")

        def _process_transcript(
            self, transcript: VoiceTranscript, generation: int
        ) -> None:
            from bot_navigation.waypoint_store import current_scene

            if not self._command_guard.is_current(generation):
                return
            scene, _ = current_scene(None, self.scene_file)
            with self._goal_lock:
                active = self._goal_pending or self._active_goal is not None
                active_task_id = self._active_task_id if active else None
            context = NavigationContext(
                scene=scene,
                available_destinations=self.resolver.available_destinations(scene),
                active_task_id=active_task_id,
                navigation_state="NAVIGATING" if active else "IDLE",
            )
            if self._llm_config_error:
                self.get_logger().error(self._llm_config_error)
                return
            try:
                intent = asyncio.run(self.llm.parse_navigation_intent(transcript.text, context))
            except (CloudLLMError, IntentValidationError) as error:
                self.get_logger().error(f"cloud intent failed: {error}")
                self._speak("语音指令暂时无法解析")
                return
            if not self._command_guard.is_current(generation):
                return
            if intent.intent == "cancel_navigation":
                if self._cancel_active_goal():
                    self._speak("正在取消导航")
                else:
                    self._speak("当前没有进行中的导航")
                return
            if intent.intent == "navigation_status":
                self._speak("当前正在导航" if active else "当前没有进行中的导航")
                return
            if (
                intent.intent != "navigate"
                or intent.need_confirmation
                or intent.confidence < self.min_confidence
            ):
                self._speak("请说明一个已配置的导航位置")
                return
            if active:
                self._speak("当前已有导航任务，请先说停止")
                return
            task_id = f"voice-{uuid.uuid4()}"
            self._send_navigation_goal(intent, scene, task_id)

        def _send_navigation_goal(
            self, intent: NavigationIntent, scene: str, task_id: str
        ) -> None:
            with self._goal_lock:
                if self._goal_pending or self._active_goal is not None:
                    self._speak("当前已有导航任务，请先说停止")
                    return
                self._goal_pending = True
                self._cancel_requested = False
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                with self._goal_lock:
                    self._goal_pending = False
                self._speak("导航服务尚未就绪")
                return
            goal = NavigateToWaypoint.Goal()
            fields = action_goal_fields(intent, scene=scene, task_id=task_id)
            goal.destination = fields["destination"]
            goal.sequence = fields["sequence"]
            goal.scene = fields["scene"]
            goal.loop = fields["loop"]
            goal.request_id = fields["request_id"]
            self._active_destination = intent.destination
            future = self.action_client.send_goal_async(
                goal, feedback_callback=self._feedback_callback
            )

            def goal_done(done_future):
                try:
                    handle = done_future.result()
                except Exception as error:
                    self.get_logger().error(f"navigation goal submission failed: {error}")
                    with self._goal_lock:
                        self._goal_pending = False
                        self._active_task_id = None
                        self._active_destination = ""
                    return
                if not handle.accepted:
                    with self._goal_lock:
                        self._goal_pending = False
                        self._active_task_id = None
                        self._active_destination = ""
                    self._speak("导航目标未被接受")
                    return
                with self._goal_lock:
                    cancel_requested = self._cancel_requested
                    self._goal_pending = False
                    self._active_goal = handle
                    self._active_task_id = task_id
                if cancel_requested:
                    handle.cancel_goal_async()
                else:
                    self._speak(f"正在前往{intent.destination}")
                result_future = handle.get_result_async()
                result_future.add_done_callback(
                    lambda result_done: self._result_callback(result_done, intent.destination)
                )

            future.add_done_callback(goal_done)

        def _feedback_callback(self, feedback_message) -> None:
            feedback = feedback_message.feedback
            self.get_logger().debug(
                f"voice navigation {feedback.state} {feedback.current_waypoint} "
                f"remaining={feedback.distance_remaining:.2f}"
            )

        def _result_callback(self, result_future, destination: str) -> None:
            try:
                response = result_future.result()
                status = result_status_name(response)
            except Exception as error:
                self.get_logger().error(f"navigation result failed: {error}")
                status = "ERROR"
            with self._goal_lock:
                self._active_goal = None
                self._goal_pending = False
                self._cancel_requested = False
                self._active_task_id = None
                self._active_destination = ""
            self._speak(navigation_result_text(status, destination))

        def _cancel_active_goal(self) -> bool:
            with self._goal_lock:
                handle = self._active_goal
                pending = self._goal_pending
                if pending:
                    self._cancel_requested = True
            if handle is not None:
                handle.cancel_goal_async()
                return True
            return pending

        def _speak(self, text: str) -> None:
            if self.speech is None:
                self.get_logger().info(f"speech: {text}")
                return

            if not self.speech_dispatcher.enqueue(text):
                self.get_logger().warning("speech output queue full; dropping utterance")

        def stop(self) -> None:
            with self._queue_condition:
                self._stopping = True
                self._queue_condition.notify_all()
            self._cancel_active_goal()
            self._worker.join(timeout=2.0)
            if self.speech_dispatcher is not None:
                self.speech_dispatcher.close(timeout=2.0)
            if self._native_asr is not None:
                self._native_asr.close()

    rclpy.init(args=args)
    node = VoiceNavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
