from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from bot_custom_interfaces.msg import StatusArray
from bot_custom_interfaces.srv import CurrentMode
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, PointCloud2
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from .action_registry import ActionRegistry, RUNNING_STATUSES, TaskRegistry
from .schemas import (
    IdempotencyConflict,
    duration_to_seconds,
    normalized_namespace,
    stable_payload_hash,
)
from .topic_cache import TopicCache

try:
    from barometer_interfaces.msg import FloorEstimate
except ImportError:
    FloorEstimate = None  # type: ignore[assignment]


class BotBrainWsGatewayNode(Node):
    def __init__(self, config: dict[str, Any], *, config_path: Path | None = None) -> None:
        super().__init__("botbrain_ws_gateway")
        self.config = config
        self.config_path = config_path
        self.robot = dict(config.get("robot", {}))
        self.robot_id = str(self.robot.get("robot_id", "g1-edu"))
        self.runtime = str(self.robot.get("runtime", "botbrain_ws_aitech"))
        self.navigation_stack = str(self.robot.get("navigation_stack", "fastlio2_icp_nav2"))
        self.ros_namespace = normalized_namespace(str(self.robot.get("ros_namespace", "/g1_robot")))
        self._lock = threading.RLock()

        self.topic_cache = TopicCache()
        self.actions = ActionRegistry()
        self.tasks = TaskRegistry()
        self._cb_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        nav2_config = dict(config.get("nav2", {}))
        self.navigate_action_name = str(nav2_config.get("navigate_to_pose_action", "/g1_robot/navigate_to_pose"))
        self.follow_waypoints_action_name = str(nav2_config.get("follow_waypoints_action", "/g1_robot/follow_waypoints"))
        self.action_server_wait_s = float(nav2_config.get("action_server_wait_s", 2.0))
        self.default_nav_timeout_s = float(nav2_config.get("default_timeout_s", 300.0))
        self.lifecycle_nodes = [str(item) for item in nav2_config.get("lifecycle_nodes", [])]
        self.require_lifecycle_active = bool(nav2_config.get("require_lifecycle_active", True))
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action_name,
            callback_group=self._cb_group,
        )
        self._waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            self.follow_waypoints_action_name,
            callback_group=self._cb_group,
        )

        services = dict(config.get("services", {}))
        self.current_mode_service = str(services.get("current_mode", f"{self.ros_namespace}/current_mode"))
        self.emergency_stop_service = str(services.get("emergency_stop", f"{self.ros_namespace}/emergency_stop"))
        self._current_mode_client = self.create_client(
            CurrentMode,
            self.current_mode_service,
            callback_group=self._cb_group,
        )
        self._emergency_stop_client = self.create_client(
            SetBool,
            self.emergency_stop_service,
            callback_group=self._cb_group,
        )
        self._lifecycle_clients: dict[str, Any] = {
            name: self.create_client(
                GetState,
                self._lifecycle_get_state_service(name),
                callback_group=self._cb_group,
            )
            for name in self.lifecycle_nodes
        }

        self._setup_topic_subscriptions()
        self._watchdog = self.create_timer(0.5, self._watchdog_tick, callback_group=self._cb_group)

    def health(self) -> dict[str, Any]:
        checks: list[dict[str, Any]] = []
        blockers: list[str] = []

        def add(name: str, ok: bool, message: str, details: dict[str, Any] | None = None) -> None:
            checks.append({"name": name, "ok": ok, "message": message, "details": details or {}})
            if not ok:
                blockers.append(message or name)

        add("ros_graph", True, "ROS graph reachable")
        nav_ready = self._nav_client.server_is_ready()
        add(
            "nav2_action_server",
            nav_ready,
            "NavigateToPose action server is available" if nav_ready else "NavigateToPose action server is unavailable",
            {"navigate_to_pose": self.navigate_action_name, "follow_waypoints": self.follow_waypoints_action_name},
        )
        add(
            "current_mode_service",
            self._current_mode_client.service_is_ready(),
            "current_mode service is available"
            if self._current_mode_client.service_is_ready()
            else "current_mode service is unavailable",
            {"service": self.current_mode_service},
        )
        add(
            "emergency_stop_service",
            self._emergency_stop_client.service_is_ready(),
            "emergency_stop service is available"
            if self._emergency_stop_client.service_is_ready()
            else "emergency_stop service is unavailable",
            {"service": self.emergency_stop_service},
        )

        essential_topics = [
            "battery",
            "odom",
            "imu",
            "joint_states",
            "fastlio_odom",
            "fastlio_cloud_body",
            "icp_pose",
            "icp_confidence",
            "map",
        ]
        for key in essential_topics:
            entry = self.topic_cache.get(key)
            if entry is None:
                add(key, False, f"{key} topic is not configured")
                continue
            add(
                f"{key}_fresh",
                entry.fresh(),
                f"{entry.topic} is fresh" if entry.fresh() else f"{entry.topic} is stale or missing",
                entry.as_dict(),
            )

        battery_entry = self.topic_cache.get("battery")
        battery_percentage = None
        if battery_entry is not None:
            battery_percentage = battery_entry.summary.get("percentage")
        min_battery = float(dict(self.config.get("battery", {})).get("min_percentage_go", 0.30))
        battery_ok = isinstance(battery_percentage, (int, float)) and float(battery_percentage) >= min_battery
        add(
            "battery_go_threshold",
            battery_ok,
            "battery is above go threshold" if battery_ok else "battery is below go threshold or unknown",
            {"percentage": battery_percentage, "min_percentage_go": min_battery},
        )

        confidence_entry = self.topic_cache.get("icp_confidence")
        confidence = None
        if confidence_entry is not None:
            confidence = confidence_entry.summary.get("data")
        min_confidence = float(dict(self.config.get("localization", {})).get("min_icp_confidence_go", 0.70))
        confidence_ok = isinstance(confidence, (int, float)) and float(confidence) >= min_confidence
        add(
            "icp_confidence_go_threshold",
            confidence_ok,
            "ICP localization confidence is above go threshold"
            if confidence_ok
            else "ICP localization confidence is below go threshold or unknown",
            {"confidence": confidence, "min_icp_confidence_go": min_confidence},
        )

        if self.require_lifecycle_active:
            lifecycle = self.lifecycle_state_snapshot(timeout_s=0.2)
            lifecycle_ok = bool(lifecycle.get("ok"))
            add(
                "nav2_lifecycle_active",
                lifecycle_ok,
                "Nav2 lifecycle nodes are active" if lifecycle_ok else "Nav2 lifecycle nodes are not active",
                lifecycle,
            )

        tf_snapshot = self.tf_snapshot()
        add(
            "tf_chain",
            bool(tf_snapshot.get("ok")),
            "required TF chains are available" if tf_snapshot.get("ok") else "required TF chains are unavailable",
            tf_snapshot,
        )

        return {
            "ok": not blockers,
            "schema_version": "botbrain_ws.gateway.health.v1",
            "robot_id": self.robot_id,
            "runtime": self.runtime,
            "navigation_stack": self.navigation_stack,
            "ros_namespace": self.ros_namespace,
            "checks": checks,
            "blockers": blockers,
        }

    def state(self) -> dict[str, Any]:
        return {
            "ok": True,
            "schema_version": "botbrain_ws.gateway.state.v1",
            "robot_id": self.robot_id,
            "runtime": self.runtime,
            "navigation_stack": self.navigation_stack,
            "ros_namespace": self.ros_namespace,
            "topics": self.topic_cache.as_dict(),
            "active_actions": [record.as_dict() for record in self.actions.running()],
            "active_task_actions": [record.as_dict() for record in self.tasks.running()],
            "mode": self.current_mode(timeout_s=0.2),
        }

    def sensors(self) -> dict[str, Any]:
        return {
            "ok": True,
            "schema_version": "botbrain_ws.gateway.sensors.v1",
            "topics": self.topic_cache.as_dict(),
            "tf": self.tf_snapshot(),
        }

    def pois(self) -> dict[str, Any]:
        waypoints = self._load_waypoints()
        pois = []
        for name, waypoint in sorted(waypoints.items()):
            if not isinstance(waypoint, dict):
                continue
            pois.append(
                {
                    "name": name,
                    "frame_id": str(waypoint.get("frame") or waypoint.get("frame_id") or self.robot.get("map_frame", "g1_robot/map")),
                    "x": waypoint.get("x"),
                    "y": waypoint.get("y"),
                    "z": waypoint.get("z", 0.0),
                    "qx": waypoint.get("qx", 0.0),
                    "qy": waypoint.get("qy", 0.0),
                    "qz": waypoint.get("qz", 0.0),
                    "qw": waypoint.get("qw", 1.0),
                }
            )
        return {
            "ok": True,
            "schema_version": "botbrain_ws.gateway.pois.v1",
            "map_id": self.robot.get("map_id"),
            "source": str(self._waypoint_file() or ""),
            "pois": pois,
        }

    def send_navigate_to_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            raise ValueError("request_id is required")

        pose = payload.get("pose")
        target_poi = str(payload.get("poi") or "").strip() or None
        if not isinstance(pose, dict):
            if not target_poi:
                raise ValueError("pose or poi is required")
            pose = self._pose_for_poi(target_poi)
            if pose is None:
                record = self._rejected_nav_record(
                    request_id=request_id,
                    payload=payload,
                    error_code="G1_POI_NOT_FOUND",
                    message=f"unknown POI: {target_poi}",
                )
                return {"ok": False, "action": record.as_dict()}

        timeout_s = float(payload.get("timeout_s", self.default_nav_timeout_s))
        try:
            record, replayed = self.actions.create_or_replay(
                request_id=request_id,
                request_payload=payload,
                mission_run_id=_optional_str(payload.get("mission_run_id")),
                kind="navigate_to_pose",
                target_waypoint=_optional_str(payload.get("waypoint")),
                target_poi=target_poi,
                target_pose=dict(pose),
                timeout_s=timeout_s,
                ros_action=self.navigate_action_name,
            )
        except IdempotencyConflict:
            raise

        if replayed:
            return {"ok": record.status not in {"rejected", "failed"}, "action": record.as_dict(), "replayed": True}

        if not self._nav_client.wait_for_server(timeout_sec=self.action_server_wait_s):
            record.status = "rejected"
            record.error_code = "G1_NAV_ACTION_SERVER_UNAVAILABLE"
            record.message = f"NavigateToPose action server unavailable: {self.navigate_action_name}"
            record.touch()
            return {"ok": False, "action": record.as_dict()}

        goal = NavigateToPose.Goal()
        goal.pose = _pose_stamped_from_dict(pose, self.get_clock().now().to_msg())
        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback, action_id=record.action_id: self._on_nav_feedback(action_id, feedback),
        )
        send_future.add_done_callback(lambda future, action_id=record.action_id: self._on_nav_goal_response(action_id, future))
        record.status = "accepted"
        record.evidence = {
            "ros_action": self.navigate_action_name,
            "navigation_stack": self.navigation_stack,
            "map_frame": self.robot.get("map_frame", "g1_robot/map"),
            "speed_limit_mps": payload.get("speed_limit_mps"),
        }
        record.touch()
        return {"ok": True, "action": record.as_dict()}

    def get_action(self, action_id: str) -> dict[str, Any]:
        record = self.actions.get(action_id)
        if record is None:
            return {"ok": False, "error": "action not found", "action_id": action_id}
        return {"ok": True, "action": record.as_dict()}

    def stop_navigation(self, payload: dict[str, Any]) -> dict[str, Any]:
        level = str(payload.get("level", "cancel_nav"))
        reason = str(payload.get("reason", "navigation stop requested"))
        cancelled = []
        for record in self.actions.running():
            goal_handle = record.goal_handle
            if goal_handle is not None:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _future: None)
            record.status = "cancelled"
            record.error_code = "G1_NAV_ACTION_CANCELLED"
            record.message = reason
            record.touch()
            cancelled.append(record.action_id)

        emergency_called = False
        emergency_result: dict[str, Any] | None = None
        if level == "emergency":
            emergency_called = True
            emergency_result = self.set_emergency_stop(True, timeout_s=1.0)

        return {
            "ok": True,
            "stop": {
                "level": level,
                "nav_cancelled": bool(cancelled),
                "cancelled_actions": cancelled,
                "emergency_stop_called": emergency_called,
                "emergency_stop_result": emergency_result,
                "message": reason,
            },
        }

    def run_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            raise ValueError("request_id is required")
        action_class = str(payload.get("action_class", "unknown"))
        waypoint = str(payload.get("waypoint", ""))
        action_id = str(payload.get("action_id", ""))
        timeout_s = float(payload.get("timeout_s", dict(self.config.get("tasks", {})).get("default_timeout_s", 120.0)))
        record, replayed = self.tasks.create_or_replay(
            request_id=request_id,
            request_payload=payload,
            mission_run_id=_optional_str(payload.get("mission_run_id")),
            waypoint=waypoint,
            action_id=action_id,
            action_class=action_class,
            payload=dict(payload.get("payload", {})),
            evidence_contract=dict(payload.get("evidence_contract", {})),
            timeout_s=timeout_s,
        )
        if replayed:
            return {"ok": record.status not in {"rejected", "failed"}, "task_action": record.as_dict(), "replayed": True}

        implemented = {str(item) for item in dict(self.config.get("tasks", {})).get("implemented_action_classes", [])}
        if action_class not in implemented:
            record.status = "failed"
            record.error_code = "G1_TASK_UNIMPLEMENTED"
            record.message = f"task action class is not implemented in gateway: {action_class}"
            record.evidence = {
                "action_class": action_class,
                "reason": "facility/manipulation action is intentionally not faked",
                "navigation_stack": self.navigation_stack,
            }
            record.touch()
            return {"ok": False, "task_action": record.as_dict()}

        record.status = "running"
        record.touch()
        return {"ok": True, "task_action": record.as_dict()}

    def get_task(self, task_action_id: str) -> dict[str, Any]:
        record = self.tasks.get(task_action_id)
        if record is None:
            return {"ok": False, "error": "task action not found", "task_action_id": task_action_id}
        return {"ok": True, "task_action": record.as_dict()}

    def stop_all_tasks(self, payload: dict[str, Any]) -> dict[str, Any]:
        reason = str(payload.get("reason", "task stop requested"))
        stopped = self.tasks.stop_all(reason)
        return {"ok": True, "stopped": [record.task_action_id for record in stopped], "reason": reason}

    def evidence(self, kind: str) -> dict[str, Any]:
        entry = self.topic_cache.get(kind)
        if entry is None:
            return {"ok": False, "error": f"unknown evidence kind: {kind}"}
        return {"ok": True, "kind": kind, "evidence": entry.as_dict()}

    def current_mode(self, timeout_s: float = 0.5) -> dict[str, Any]:
        if not self._current_mode_client.service_is_ready():
            return {"ok": False, "service": self.current_mode_service, "message": "service unavailable"}
        request = CurrentMode.Request()
        future = self._current_mode_client.call_async(request)
        if not _wait_future(future, timeout_s):
            return {"ok": False, "service": self.current_mode_service, "message": "service timeout"}
        response = future.result()
        return {"ok": True, "service": self.current_mode_service, "mode": str(getattr(response, "mode", "unknown"))}

    def set_emergency_stop(self, enabled: bool, timeout_s: float = 1.0) -> dict[str, Any]:
        if not self._emergency_stop_client.service_is_ready():
            return {"ok": False, "service": self.emergency_stop_service, "message": "service unavailable"}
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self._emergency_stop_client.call_async(request)
        if not _wait_future(future, timeout_s):
            return {"ok": False, "service": self.emergency_stop_service, "message": "service timeout"}
        response = future.result()
        return {
            "ok": bool(getattr(response, "success", False)),
            "service": self.emergency_stop_service,
            "message": str(getattr(response, "message", "")),
        }

    def lifecycle_state_snapshot(self, timeout_s: float = 0.2) -> dict[str, Any]:
        nodes = []
        ok = True
        for name, client in self._lifecycle_clients.items():
            service = self._lifecycle_get_state_service(name)
            if not client.service_is_ready():
                ok = False
                nodes.append({"name": name, "service": service, "ok": False, "state": "unavailable"})
                continue
            future = client.call_async(GetState.Request())
            if not _wait_future(future, timeout_s):
                ok = False
                nodes.append({"name": name, "service": service, "ok": False, "state": "timeout"})
                continue
            state = future.result().current_state
            active = state.id == State.PRIMARY_STATE_ACTIVE
            ok = ok and active
            nodes.append(
                {
                    "name": name,
                    "service": service,
                    "ok": active,
                    "state_id": int(state.id),
                    "state": str(state.label),
                }
            )
        return {"ok": ok, "nodes": nodes}

    def tf_snapshot(self) -> dict[str, Any]:
        required = list(dict(self.config.get("localization", {})).get("required_tf", []))
        checks = []
        ok = True
        for pair in required:
            if not isinstance(pair, list) or len(pair) != 2:
                continue
            target, source = str(pair[0]), str(pair[1])
            try:
                tf = self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                checks.append(
                    {
                        "target": target,
                        "source": source,
                        "ok": True,
                        "stamp": {
                            "sec": int(tf.header.stamp.sec),
                            "nanosec": int(tf.header.stamp.nanosec),
                        },
                    }
                )
            except TransformException as exc:
                ok = False
                checks.append({"target": target, "source": source, "ok": False, "message": str(exc)})
        return {"ok": ok, "checks": checks}

    def _setup_topic_subscriptions(self) -> None:
        topics = dict(self.config.get("topics", {}))
        freshness = dict(self.config.get("freshness", {}))
        specs: dict[str, tuple[Any, float]] = {
            "battery": (BatteryState, 2.0),
            "odom": (Odometry, 0.5),
            "imu": (Imu, 0.5),
            "joint_states": (JointState, 0.5),
            "state_machine_status": (StatusArray, 2.0),
            "fastlio_odom": (Odometry, 0.5),
            "fastlio_cloud_world": (PointCloud2, 0.8),
            "fastlio_cloud_body": (PointCloud2, 0.8),
            "fastlio_map": (PointCloud2, 10.0),
            "fastlio_path": (NavPath, 2.0),
            "icp_pose": (PoseStamped, 1.0),
            "icp_confidence": (Float32, 1.0),
            "icp_delay_ms": (Float32, 1.0),
            "icp_baselink2map": (Odometry, 1.0),
            "map": (OccupancyGrid, 10.0),
            "diagnostics": (DiagnosticArray, 5.0),
        }
        if FloorEstimate is not None:
            specs["floor_estimate"] = (FloorEstimate, 2.0)
        elif topics.get("floor_estimate"):
            max_age = float(freshness.get("floor_estimate_max_age_s", 2.0))
            self.topic_cache.add("floor_estimate", str(topics["floor_estimate"]), max_age)
            self.get_logger().warn(
                "floor_estimate topic configured but barometer_interfaces is not available; "
                "floor evidence will stay stale until that package is sourced."
            )

        for key, (msg_type, default_max_age) in specs.items():
            topic = topics.get(key)
            if not topic:
                continue
            max_age = float(freshness.get(f"{key}_max_age_s", default_max_age))
            self.topic_cache.add(key, str(topic), max_age)
            self.create_subscription(
                msg_type,
                str(topic),
                lambda msg, cache_key=key: self.topic_cache.update(cache_key, msg),
                10,
                callback_group=self._cb_group,
            )

    def _on_nav_goal_response(self, action_id: str, future: Any) -> None:
        record = self.actions.get(action_id)
        if record is None:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.actions.update(
                action_id,
                status="rejected",
                error_code="G1_NAV_GOAL_SEND_FAILED",
                message=str(exc),
            )
            return
        if not goal_handle.accepted:
            self.actions.update(
                action_id,
                status="rejected",
                error_code="G1_NAV_GOAL_REJECTED",
                message="NavigateToPose goal rejected",
            )
            return
        record.goal_handle = goal_handle
        record.status = "running"
        record.touch()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda result, nav_action_id=action_id: self._on_nav_result(nav_action_id, result))

    def _on_nav_feedback(self, action_id: str, feedback_msg: Any) -> None:
        feedback = getattr(feedback_msg, "feedback", feedback_msg)
        payload = {
            "distance_remaining": _float_or_none(getattr(feedback, "distance_remaining", None)),
            "navigation_time_s": duration_to_seconds(getattr(feedback, "navigation_time", None)),
            "estimated_time_remaining_s": duration_to_seconds(getattr(feedback, "estimated_time_remaining", None)),
            "number_of_recoveries": getattr(feedback, "number_of_recoveries", None),
        }
        self.actions.update(action_id, last_feedback={k: v for k, v in payload.items() if v is not None})

    def _on_nav_result(self, action_id: str, future: Any) -> None:
        try:
            result = future.result()
            status = int(result.status)
            result_payload = _result_to_dict(getattr(result, "result", None))
        except Exception as exc:
            self.actions.update(
                action_id,
                status="failed",
                error_code="G1_NAV_RESULT_FAILED",
                message=str(exc),
            )
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            mapped_status = "succeeded"
            error_code = None
            message = "navigation goal succeeded"
        elif status == GoalStatus.STATUS_CANCELED:
            mapped_status = "cancelled"
            error_code = "G1_NAV_ACTION_CANCELLED"
            message = "navigation goal cancelled"
        elif status == GoalStatus.STATUS_ABORTED:
            mapped_status = "failed"
            error_code = "G1_NAV_ACTION_ABORTED"
            message = "navigation goal aborted"
        else:
            mapped_status = "failed"
            error_code = "G1_NAV_ACTION_FAILED"
            message = f"navigation goal finished with status {status}"
        self.actions.update(
            action_id,
            status=mapped_status,
            error_code=error_code,
            message=message,
            result=result_payload,
        )

    def _watchdog_tick(self) -> None:
        now = time.monotonic()
        for record in self.actions.running():
            if record.deadline_monotonic is None or record.deadline_monotonic > now:
                continue
            if record.goal_handle is not None:
                record.goal_handle.cancel_goal_async()
            record.status = "timeout"
            record.error_code = "G1_NAV_ACTION_TIMEOUT"
            record.message = "navigation action timed out"
            record.touch()
        for record in self.tasks.running():
            if record.deadline_monotonic is None or record.deadline_monotonic > now:
                continue
            record.status = "timeout"
            record.error_code = "G1_TASK_ACTION_TIMEOUT"
            record.message = "task action timed out"
            record.touch()

    def _pose_for_poi(self, poi: str) -> dict[str, Any] | None:
        waypoint = self._load_waypoints().get(poi)
        if not isinstance(waypoint, dict):
            return None
        return {
            "frame_id": str(waypoint.get("frame") or waypoint.get("frame_id") or self.robot.get("map_frame", "g1_robot/map")),
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "z": float(waypoint.get("z", 0.0)),
            "qx": float(waypoint.get("qx", 0.0)),
            "qy": float(waypoint.get("qy", 0.0)),
            "qz": float(waypoint.get("qz", 0.0)),
            "qw": float(waypoint.get("qw", 1.0)),
        }

    def _load_waypoints(self) -> dict[str, Any]:
        path = self._waypoint_file()
        if path is None:
            return {}
        try:
            payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except OSError:
            return {}
        if not isinstance(payload, dict):
            return {}
        waypoints = payload.get("waypoints", {})
        return waypoints if isinstance(waypoints, dict) else {}

    def _waypoint_file(self) -> Path | None:
        pois_config = dict(self.config.get("pois", {}))
        candidates = []
        if pois_config.get("waypoint_file"):
            candidates.append(Path(str(pois_config["waypoint_file"])).expanduser())
        fallback = pois_config.get("fallback_waypoint_file")
        if fallback:
            candidates.append(Path.cwd() / str(fallback))
            if self.config_path:
                for parent in self.config_path.resolve().parents:
                    if parent.name == "botbrain_ws":
                        candidates.append(parent / str(fallback))
                        break
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0] if candidates else None

    def _lifecycle_get_state_service(self, node_name: str) -> str:
        return f"{self.ros_namespace}/{node_name}/get_state"

    def _rejected_nav_record(
        self,
        *,
        request_id: str,
        payload: dict[str, Any],
        error_code: str,
        message: str,
    ) -> Any:
        record, _replayed = self.actions.create_or_replay(
            request_id=request_id,
            request_payload=payload,
            mission_run_id=_optional_str(payload.get("mission_run_id")),
            kind="navigate_to_pose",
            target_waypoint=_optional_str(payload.get("waypoint")),
            target_poi=_optional_str(payload.get("poi")),
            target_pose=None,
            timeout_s=float(payload.get("timeout_s", self.default_nav_timeout_s)),
            ros_action=self.navigate_action_name,
        )
        record.status = "rejected"
        record.error_code = error_code
        record.message = message
        record.evidence = {"request_hash": stable_payload_hash(payload)}
        record.touch()
        return record


def _pose_stamped_from_dict(payload: dict[str, Any], stamp: Any) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = str(payload.get("frame_id") or payload.get("frame") or "g1_robot/map")
    pose.header.stamp = stamp
    pose.pose.position.x = float(payload.get("x", 0.0))
    pose.pose.position.y = float(payload.get("y", 0.0))
    pose.pose.position.z = float(payload.get("z", 0.0))
    if {"qx", "qy", "qz", "qw"}.issubset(payload):
        pose.pose.orientation.x = float(payload["qx"])
        pose.pose.orientation.y = float(payload["qy"])
        pose.pose.orientation.z = float(payload["qz"])
        pose.pose.orientation.w = float(payload["qw"])
    else:
        yaw = float(payload.get("yaw", 0.0))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _wait_future(future: Any, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return bool(future.done())


def _result_to_dict(result: Any) -> dict[str, Any]:
    if result is None:
        return {}
    payload: dict[str, Any] = {}
    for key in ("error_code", "error_msg", "missed_waypoints"):
        if hasattr(result, key):
            value = getattr(result, key)
            if isinstance(value, list):
                value = list(value)
            payload[key] = value
    return payload


def _float_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _optional_str(value: Any) -> str | None:
    text = str(value or "").strip()
    return text or None
