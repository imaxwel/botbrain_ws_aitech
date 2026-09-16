"""ROS 2 Action gateway that delegates to the existing waypoint CLI."""

from __future__ import annotations

import threading
import uuid
from pathlib import Path

import rclpy
from bot_custom_interfaces.action import NavigateToWaypoint
from bot_navigation.waypoint_controller import WaypointNavigationController
from bot_navigation.waypoint_store import current_scene
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .destinations import DestinationResolver
from .navigation_gateway import GatewayPolicy, NavigationBusyError, NavigationPolicyError


class VoiceNavigationGateway(Node):
    """Expose a safe named-waypoint Action under the robot namespace."""

    def __init__(self) -> None:
        super().__init__("voice_navigation_gateway")
        self.declare_parameter("robot", "g1_robot")
        self.declare_parameter(
            "waypoint_file", "/botbrain_ws/src/bot_navigation/nav_waypoints.yaml"
        )
        self.declare_parameter(
            "aliases_file",
            "/botbrain_ws/src/bot_voice_navigation/config/voice_destinations.yaml",
        )
        self.declare_parameter("scene_file", "/botbrain_ws/.runtime/map_scene")
        self.declare_parameter("navigator_script", "")
        self.robot = str(self.get_parameter("robot").value).strip()
        waypoint_file = Path(str(self.get_parameter("waypoint_file").value)).expanduser()
        aliases_file = Path(str(self.get_parameter("aliases_file").value)).expanduser()
        self.scene_file = Path(str(self.get_parameter("scene_file").value)).expanduser()
        self.resolver = DestinationResolver.from_files(aliases_file, waypoint_file)
        script_value = str(self.get_parameter("navigator_script").value).strip()
        self.controller = WaypointNavigationController(
            Path(script_value) if script_value else None,
            robot=self.robot,
        )
        self.policy = GatewayPolicy(self.resolver)
        self._task_lock = threading.Lock()
        self._active_goal = None
        self._callback_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            NavigateToWaypoint,
            "navigate_to_waypoint",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"Voice navigation gateway ready: /{self.robot}/navigate_to_waypoint"
        )

    def _goal_callback(self, _request):
        if self.policy.active_task() is not None:
            self.get_logger().warning("rejecting concurrent voice navigation goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        with self._task_lock:
            active_goal = self._active_goal
        if active_goal is not None and active_goal is not goal_handle:
            return CancelResponse.REJECT
        self.controller.request_cancel()
        return CancelResponse.ACCEPT

    def _active_scene(self) -> str:
        scene, _ = current_scene(None, self.scene_file)
        return scene

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        task_id = str(request.request_id).strip() or str(uuid.uuid4())
        result = NavigateToWaypoint.Result()
        try:
            scene = str(request.scene).strip() or self._active_scene()
            active_scene = self._active_scene()
            if scene != active_scene:
                raise NavigationPolicyError(
                    f"requested scene {scene!r} differs from active scene {active_scene!r}"
                )
            task = self.policy.begin(
                destination=request.destination,
                sequence=list(request.sequence),
                scene=scene,
                task_id=task_id,
            )
            with self._task_lock:
                self._active_goal = goal_handle
            self.policy.mark_state("VALIDATING", task_id=task_id)
            self._publish_feedback(
                goal_handle,
                task_id=task_id,
                current_waypoint=task.waypoints[0],
                state="VALIDATING",
            )

            def feedback_callback(feedback):
                self._publish_feedback(
                    goal_handle,
                    task_id=task_id,
                    current_waypoint=feedback.current_waypoint,
                    state=feedback.state,
                    distance_remaining=feedback.distance_remaining,
                    distance_to_goal=feedback.distance_to_goal,
                    yaw_error_deg=feedback.yaw_error_deg,
                    recoveries=feedback.recoveries,
                )

            self.policy.mark_state("NAVIGATING", task_id=task_id)
            navigation_result = self.controller.start(
                task.waypoints,
                scene,
                loop=bool(request.loop),
                task_id=task_id,
                feedback_callback=feedback_callback,
            )
            result.success = bool(navigation_result.success)
            result.status = navigation_result.status
            result.message = navigation_result.message
            result.task_id = task_id
            result.completed_waypoints = list(navigation_result.completed_waypoints)
            if navigation_result.status == "CANCELED" or goal_handle.is_cancel_requested:
                self.policy.finish(task_id)
                goal_handle.canceled()
            elif navigation_result.success:
                self.policy.finish(task_id)
                goal_handle.succeed()
            else:
                self.policy.finish(task_id)
                goal_handle.abort()
        except (NavigationPolicyError, NavigationBusyError) as error:
            result.success = False
            result.status = "REJECTED"
            result.message = str(error)
            result.task_id = task_id
            result.completed_waypoints = []
            goal_handle.abort()
        except Exception as error:
            self.get_logger().error(f"Voice navigation task failed: {error}")
            result.success = False
            result.status = "ERROR"
            result.message = str(error)
            result.task_id = task_id
            result.completed_waypoints = []
            self.policy.finish(task_id)
            goal_handle.abort()
        finally:
            with self._task_lock:
                self._active_goal = None
        return result

    def _publish_feedback(
        self,
        goal_handle,
        *,
        task_id: str,
        current_waypoint: str,
        state: str,
        distance_remaining: float = -1.0,
        distance_to_goal: float = -1.0,
        yaw_error_deg: float = -1.0,
        recoveries: int = 0,
    ) -> None:
        feedback = NavigateToWaypoint.Feedback()
        feedback.task_id = task_id
        feedback.current_waypoint = current_waypoint
        feedback.state = state
        feedback.distance_remaining = float(distance_remaining)
        feedback.distance_to_goal = float(distance_to_goal)
        feedback.yaw_error_deg = float(yaw_error_deg)
        feedback.recoveries = int(recoveries)
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = VoiceNavigationGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.controller.request_cancel()
        executor.shutdown()
        node._server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
