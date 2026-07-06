from __future__ import annotations

import argparse
import threading
from pathlib import Path
from typing import Any, Callable

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException
from rclpy.executors import MultiThreadedExecutor

from .ros_node import BotBrainWsGatewayNode
from .schemas import IdempotencyConflict, load_yaml, normalized_prefix


def create_app(node: BotBrainWsGatewayNode, *, robot_prefix: str) -> FastAPI:
    app = FastAPI(title="botbrain_ws_gateway", version="0.1.0")
    prefix = normalized_prefix(robot_prefix)

    def add(method: str, suffix: str, handler: Callable[..., Any]) -> None:
        normalized_suffix = "/" + suffix.strip("/")
        paths = [normalized_suffix]
        if prefix:
            paths.insert(0, f"{prefix}{normalized_suffix}")
        for path in paths:
            app.add_api_route(path, handler, methods=[method])

    def healthz() -> dict[str, Any]:
        return node.health()

    def state() -> dict[str, Any]:
        return node.state()

    def sensors() -> dict[str, Any]:
        return node.sensors()

    def pois() -> dict[str, Any]:
        return node.pois()

    def goto(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            return node.send_navigate_to_pose(payload)
        except IdempotencyConflict as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def action(action_id: str) -> dict[str, Any]:
        payload = node.get_action(action_id)
        if not payload.get("ok"):
            raise HTTPException(status_code=404, detail=payload)
        return payload

    def stop(payload: dict[str, Any]) -> dict[str, Any]:
        return node.stop_navigation(payload)

    def emergency_stop(payload: dict[str, Any]) -> dict[str, Any]:
        enabled = bool(payload.get("enabled", True))
        return node.set_emergency_stop(enabled, timeout_s=float(payload.get("timeout_s", 1.0)))

    def run_task(payload: dict[str, Any]) -> dict[str, Any]:
        try:
            return node.run_task(payload)
        except IdempotencyConflict as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    def task(task_action_id: str) -> dict[str, Any]:
        payload = node.get_task(task_action_id)
        if not payload.get("ok"):
            raise HTTPException(status_code=404, detail=payload)
        return payload

    def stop_tasks(payload: dict[str, Any]) -> dict[str, Any]:
        return node.stop_all_tasks(payload)

    def evidence(kind: str) -> dict[str, Any]:
        payload = node.evidence(kind)
        if not payload.get("ok"):
            raise HTTPException(status_code=404, detail=payload)
        return payload

    add("GET", "healthz", healthz)
    add("GET", "state", state)
    add("GET", "sensors", sensors)
    add("GET", "pois", pois)
    add("POST", "tour/goto", goto)
    add("GET", "actions/{action_id}", action)
    add("POST", "navigation/stop", stop)
    add("POST", "emergency-stop", emergency_stop)
    add("POST", "tasks/run", run_task)
    add("GET", "tasks/{task_action_id}", task)
    add("POST", "tasks/stop-all", stop_tasks)
    add("GET", "evidence/{kind}", evidence)
    return app


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="botbrain_ws_aitech HTTP gateway for G1 Mission Supervisor.")
    parser.add_argument("--config", type=Path, default=None, help="Path to gateway YAML config.")
    parser.add_argument("--host", default=None, help="HTTP bind host override.")
    parser.add_argument("--port", type=int, default=None, help="HTTP bind port override.")
    args = parser.parse_args(argv)

    config_path = args.config or _default_config_path()
    config = load_yaml(config_path)
    http_config = dict(config.get("http", {}))
    host = args.host or str(http_config.get("host", "127.0.0.1"))
    port = int(args.port or http_config.get("port", 8899))
    robot_prefix = str(http_config.get("robot_prefix", "/g1"))

    rclpy.init()
    node = BotBrainWsGatewayNode(config, config_path=config_path)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        app = create_app(node, robot_prefix=robot_prefix)
        uvicorn.run(app, host=host, port=port)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


def _default_config_path() -> Path:
    source_config = Path(__file__).resolve().parents[1] / "config" / "gateway.yaml"
    if source_config.exists():
        return source_config
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("botbrain_ws_gateway")) / "config" / "gateway.yaml"
    except Exception:
        return source_config


if __name__ == "__main__":
    main()
