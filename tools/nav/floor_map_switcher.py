#!/usr/bin/env python3
"""Switch the G1 navigation scene after a verified floor-state update.

This program must run on the robot host, outside the navigation container. A
scene switch intentionally stops and recreates that container, so putting the
orchestrator inside it would terminate the transaction halfway through.

The ROS message type is configurable because the floor-estimation workspace is
maintained separately from BotBrain. The default matches the usual
``std_msgs/msg/Int32`` publisher and reads its ``data`` field.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "floor_map_switcher.yaml"
SCENE_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


@dataclass(frozen=True)
class ArrivalPrior:
    name: str
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class TransitionProfile:
    """Localization priors and search bounds for one elevator transition."""

    elevator_id: str
    from_floor: int
    target_floor: int
    arrival_priors: tuple[ArrivalPrior, ...]
    search_radius_m: float = 1.5
    yaw_range_deg: float = 30.0
    search_xy_step_m: float = 0.5
    search_yaw_step_deg: float = 10.0
    max_nearby_candidates: int = 32


@dataclass(frozen=True)
class SwitcherConfig:
    floor_topic: str
    floor_message_type: str
    floor_field: str
    floor_to_scene: Mapping[int, str]
    stable_for_sec: float
    min_observations: int
    message_timeout_sec: float
    motion_topic: str
    motion_message_type: str
    stationary_for_sec: float
    stationary_min_observations: int
    motion_message_timeout_sec: float
    max_linear_speed_mps: float
    max_angular_speed_rps: float
    expected_floor_timeout_sec: float
    retry_interval_sec: float
    max_switch_attempts: int
    ready_timeout_sec: int
    navigation_ready_timeout_sec: int
    restart_fast_lio: bool
    start_navigation: bool
    allow_uncommanded_switch: bool
    expected_floor: int
    from_floor: int
    elevator_id: str
    arrival_confirmed: bool
    require_arrival_confirmation: bool
    arrival_priors: Mapping[int, tuple[ArrivalPrior, ...]]
    transitions: tuple[TransitionProfile, ...]
    stop_navigation_on_timeout: bool
    maps_dir: Path
    selector: Path
    scene_state_file: Path


@dataclass(frozen=True)
class SwitchResult:
    floor: Optional[int]
    scene: Optional[str]
    success: bool
    message: str
    stopped_only: bool = False
    exit_code: Optional[int] = None
    retryable: bool = False


class FloorObservationGate:
    """Require a fresh run of equal floor observations before accepting it."""

    def __init__(
        self,
        *,
        stable_for_sec: float,
        min_observations: int,
        message_timeout_sec: float,
    ) -> None:
        self.stable_for_sec = stable_for_sec
        self.min_observations = min_observations
        self.message_timeout_sec = message_timeout_sec
        self.candidate: Optional[int] = None
        self.candidate_since: Optional[float] = None
        self.observation_count = 0
        self.last_message_at: Optional[float] = None
        self._lock = threading.RLock()

    def observe(self, floor: int, now: float) -> None:
        with self._lock:
            stale_gap = (
                self.last_message_at is not None
                and now - self.last_message_at > self.message_timeout_sec
            )
            if self.candidate != floor or stale_gap:
                self.candidate = floor
                self.candidate_since = now
                self.observation_count = 1
            else:
                self.observation_count += 1
            self.last_message_at = now

    def stable_floor(self, now: float) -> Optional[int]:
        with self._lock:
            if (
                self.candidate is None
                or self.candidate_since is None
                or self.last_message_at is None
                or now - self.last_message_at > self.message_timeout_sec
                or self.observation_count < self.min_observations
                or now - self.candidate_since < self.stable_for_sec
            ):
                return None
            return self.candidate

    def mark_stale(self, now: float) -> bool:
        with self._lock:
            if (
                self.last_message_at is None
                or now - self.last_message_at <= self.message_timeout_sec
            ):
                return False
            self.candidate = None
            self.candidate_since = None
            self.observation_count = 0
            self.last_message_at = None
            return True


class MotionStationarityGate:
    """Require fresh, continuously low odometry twist before a map switch."""

    def __init__(
        self,
        *,
        stationary_for_sec: float,
        min_observations: int,
        message_timeout_sec: float,
        max_linear_speed_mps: float,
        max_angular_speed_rps: float,
    ) -> None:
        self.stationary_for_sec = stationary_for_sec
        self.min_observations = min_observations
        self.message_timeout_sec = message_timeout_sec
        self.max_linear_speed_mps = max_linear_speed_mps
        self.max_angular_speed_rps = max_angular_speed_rps
        self.stationary_since: Optional[float] = None
        self.observation_count = 0
        self.last_message_at: Optional[float] = None
        self.last_linear_speed = math.inf
        self.last_angular_speed = math.inf
        self._lock = threading.RLock()

    def observe(
        self, linear_speed_mps: float, angular_speed_rps: float, now: float
    ) -> None:
        if not all(
            math.isfinite(value)
            for value in (linear_speed_mps, angular_speed_rps, now)
        ):
            raise ValueError("motion samples must contain only finite values")
        with self._lock:
            stale_gap = (
                self.last_message_at is not None
                and now - self.last_message_at > self.message_timeout_sec
            )
            stationary = (
                linear_speed_mps <= self.max_linear_speed_mps
                and angular_speed_rps <= self.max_angular_speed_rps
            )
            if not stationary or stale_gap:
                self.stationary_since = None
                self.observation_count = 0
            if stationary:
                if self.stationary_since is None:
                    self.stationary_since = now
                self.observation_count += 1
            self.last_message_at = now
            self.last_linear_speed = linear_speed_mps
            self.last_angular_speed = angular_speed_rps

    def is_stationary(self, now: float) -> bool:
        with self._lock:
            return bool(
                self.stationary_since is not None
                and self.last_message_at is not None
                and now - self.last_message_at <= self.message_timeout_sec
                and self.observation_count >= self.min_observations
                and now - self.stationary_since >= self.stationary_for_sec
            )

    def status(self, now: float) -> str:
        with self._lock:
            if self.last_message_at is None:
                return "waiting for the first motion sample"
            age = now - self.last_message_at
            if age > self.message_timeout_sec:
                return f"motion sample is stale ({age:.2f}s old)"
            if self.stationary_since is None:
                return (
                    f"robot is moving (linear={self.last_linear_speed:.3f}m/s, "
                    f"angular={self.last_angular_speed:.3f}rad/s)"
                )
            elapsed = now - self.stationary_since
            return (
                f"stationary confirmation {elapsed:.2f}/"
                f"{self.stationary_for_sec:.2f}s, observations="
                f"{self.observation_count}/{self.min_observations}"
            )


def _required_mapping(document: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = document.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key} must be a YAML mapping")
    return value


def _positive_float(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return number


def _positive_int(value: Any, name: str) -> int:
    number = int(value)
    if number <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return number


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a YAML boolean")
    return value


def _nonnegative_float(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return number


def _arrival_priors(
    value: Any, floor_to_scene: Mapping[int, str]
) -> Mapping[int, tuple[ArrivalPrior, ...]]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError("arrival_priors must be a YAML mapping")

    parsed: dict[int, tuple[ArrivalPrior, ...]] = {}
    for raw_floor, raw_priors in value.items():
        try:
            floor = int(raw_floor)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"arrival_priors key {raw_floor!r} is not an integer"
            ) from error
        if floor not in floor_to_scene:
            raise ValueError(
                f"arrival_priors floor {floor} has no floor_to_scene mapping"
            )
        if not isinstance(raw_priors, list) or not raw_priors:
            raise ValueError(
                f"arrival_priors floor {floor} must contain at least one pose"
            )

        floor_priors: list[ArrivalPrior] = []
        for index, raw_prior in enumerate(raw_priors):
            if not isinstance(raw_prior, Mapping):
                raise ValueError(
                    f"arrival_priors floor {floor} item {index} must be a mapping"
                )
            name = str(
                raw_prior.get("name", f"floor{floor}_arrival_{index}")
            ).strip()
            if not name or re.search(r"[,;\r\n]", name):
                raise ValueError(
                    f"arrival prior name {name!r} must not contain ',' or ';'"
                )
            try:
                x = float(raw_prior["x"])
                y = float(raw_prior["y"])
                yaw_deg = float(raw_prior["yaw_deg"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"arrival_priors floor {floor} item {index} requires "
                    "numeric x, y and yaw_deg"
                ) from error
            if not all(math.isfinite(item) for item in (x, y, yaw_deg)):
                raise ValueError(
                    f"arrival_priors floor {floor} item {index} is not finite"
                )
            floor_priors.append(ArrivalPrior(name, x, y, yaw_deg))
        parsed[floor] = tuple(floor_priors)
    return parsed


def _transition_profiles(
    value: Any, floor_to_scene: Mapping[int, str]
) -> tuple[TransitionProfile, ...]:
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ValueError("transitions must be a YAML list")

    profiles: list[TransitionProfile] = []
    keys: set[tuple[str, int, int]] = set()
    for index, raw_profile in enumerate(value):
        if not isinstance(raw_profile, Mapping):
            raise ValueError(f"transitions item {index} must be a mapping")
        elevator_id = str(raw_profile.get("elevator_id", "")).strip()
        if not elevator_id or re.search(r"[,;\r\n]", elevator_id):
            raise ValueError(
                f"transitions item {index} requires a safe elevator_id"
            )
        try:
            from_floor = int(raw_profile["from_floor"])
            target_floor = int(raw_profile["target_floor"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f"transitions item {index} requires integer from_floor and "
                "target_floor"
            ) from error
        if target_floor not in floor_to_scene:
            raise ValueError(
                f"transition target_floor {target_floor} has no floor_to_scene mapping"
            )
        key = (elevator_id, from_floor, target_floor)
        if key in keys:
            raise ValueError(f"duplicate transition profile {key!r}")
        keys.add(key)

        raw_priors = raw_profile.get("arrival_priors", [])
        parsed_by_floor = _arrival_priors(
            {target_floor: raw_priors}, floor_to_scene
        )
        profiles.append(
            TransitionProfile(
                elevator_id=elevator_id,
                from_floor=from_floor,
                target_floor=target_floor,
                arrival_priors=parsed_by_floor[target_floor],
                search_radius_m=_nonnegative_float(
                    raw_profile.get("search_radius_m", 1.5),
                    f"transitions[{index}].search_radius_m",
                ),
                yaw_range_deg=_nonnegative_float(
                    raw_profile.get("yaw_range_deg", 30.0),
                    f"transitions[{index}].yaw_range_deg",
                ),
                search_xy_step_m=_positive_float(
                    raw_profile.get("search_xy_step_m", 0.5),
                    f"transitions[{index}].search_xy_step_m",
                ),
                search_yaw_step_deg=_positive_float(
                    raw_profile.get("search_yaw_step_deg", 10.0),
                    f"transitions[{index}].search_yaw_step_deg",
                ),
                max_nearby_candidates=_positive_int(
                    raw_profile.get("max_nearby_candidates", 32),
                    f"transitions[{index}].max_nearby_candidates",
                ),
            )
        )
    return tuple(profiles)


def _resolve_project_path(value: Any, name: str) -> Path:
    text = str(value).strip()
    if not text:
        raise ValueError(f"{name} must not be empty")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def load_config(path: Path) -> SwitcherConfig:
    path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"configuration root must be a YAML mapping: {path}")

    raw_floor_map = _required_mapping(raw, "floor_to_scene")
    floor_to_scene: dict[int, str] = {}
    for raw_floor, raw_scene in raw_floor_map.items():
        try:
            floor = int(raw_floor)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"floor_to_scene key {raw_floor!r} is not an integer"
            ) from error
        scene = str(raw_scene).strip()
        if not SCENE_PATTERN.fullmatch(scene):
            raise ValueError(
                f"invalid scene {scene!r} for floor {floor}; use only letters, "
                "digits, '_' or '-'"
            )
        if floor in floor_to_scene:
            raise ValueError(f"duplicate floor mapping for {floor}")
        floor_to_scene[floor] = scene
    if not floor_to_scene:
        raise ValueError("floor_to_scene must contain at least one floor")

    expected_floor = int(raw.get("expected_floor", -1))
    if expected_floor >= 0 and expected_floor not in floor_to_scene:
        raise ValueError(
            f"expected_floor {expected_floor} has no floor_to_scene mapping"
        )

    topic = str(raw.get("floor_topic", "/floor_state")).strip()
    message_type = str(
        raw.get("floor_message_type", "std_msgs/msg/Int32")
    ).strip()
    field = str(raw.get("floor_field", "data")).strip()
    if not topic.startswith("/"):
        raise ValueError("floor_topic must be an absolute ROS topic")
    if not message_type or not field:
        raise ValueError("floor_message_type and floor_field must not be empty")

    motion_topic = str(raw.get("motion_topic", "/g1_robot/odom")).strip()
    motion_message_type = str(
        raw.get("motion_message_type", "nav_msgs/msg/Odometry")
    ).strip()
    if not motion_topic.startswith("/"):
        raise ValueError("motion_topic must be an absolute ROS topic")
    if not motion_message_type:
        raise ValueError("motion_message_type must not be empty")

    arrival_priors = _arrival_priors(
        raw.get("arrival_priors"), floor_to_scene
    )
    transitions = _transition_profiles(raw.get("transitions"), floor_to_scene)
    from_floor = int(raw.get("from_floor", -1))
    elevator_id = str(raw.get("elevator_id", "")).strip()
    if re.search(r"[,;\r\n]", elevator_id):
        raise ValueError("elevator_id must not contain ',', ';' or newlines")

    return SwitcherConfig(
        floor_topic=topic,
        floor_message_type=message_type,
        floor_field=field,
        floor_to_scene=floor_to_scene,
        stable_for_sec=_positive_float(
            raw.get("stable_for_sec", 3.0), "stable_for_sec"
        ),
        min_observations=_positive_int(
            raw.get("min_observations", 3), "min_observations"
        ),
        message_timeout_sec=_positive_float(
            raw.get("message_timeout_sec", 5.0), "message_timeout_sec"
        ),
        motion_topic=motion_topic,
        motion_message_type=motion_message_type,
        stationary_for_sec=_positive_float(
            raw.get("stationary_for_sec", 2.0), "stationary_for_sec"
        ),
        stationary_min_observations=_positive_int(
            raw.get("stationary_min_observations", 5),
            "stationary_min_observations",
        ),
        motion_message_timeout_sec=_positive_float(
            raw.get("motion_message_timeout_sec", 1.0),
            "motion_message_timeout_sec",
        ),
        max_linear_speed_mps=_nonnegative_float(
            raw.get("max_linear_speed_mps", 0.05),
            "max_linear_speed_mps",
        ),
        max_angular_speed_rps=_nonnegative_float(
            raw.get("max_angular_speed_rps", 0.10),
            "max_angular_speed_rps",
        ),
        expected_floor_timeout_sec=_positive_float(
            raw.get("expected_floor_timeout_sec", 120.0),
            "expected_floor_timeout_sec",
        ),
        retry_interval_sec=_positive_float(
            raw.get("retry_interval_sec", 30.0), "retry_interval_sec"
        ),
        max_switch_attempts=_positive_int(
            raw.get("max_switch_attempts", 3), "max_switch_attempts"
        ),
        ready_timeout_sec=_positive_int(
            raw.get("ready_timeout_sec", 300), "ready_timeout_sec"
        ),
        navigation_ready_timeout_sec=_positive_int(
            raw.get("navigation_ready_timeout_sec", 90),
            "navigation_ready_timeout_sec",
        ),
        restart_fast_lio=_boolean(
            raw.get("restart_fast_lio", True), "restart_fast_lio"
        ),
        start_navigation=_boolean(
            raw.get("start_navigation", True), "start_navigation"
        ),
        allow_uncommanded_switch=_boolean(
            raw.get("allow_uncommanded_switch", False),
            "allow_uncommanded_switch",
        ),
        expected_floor=expected_floor,
        from_floor=from_floor,
        elevator_id=elevator_id,
        arrival_confirmed=_boolean(
            raw.get("arrival_confirmed", False), "arrival_confirmed"
        ),
        require_arrival_confirmation=_boolean(
            raw.get("require_arrival_confirmation", False),
            "require_arrival_confirmation",
        ),
        arrival_priors=arrival_priors,
        transitions=transitions,
        stop_navigation_on_timeout=_boolean(
            raw.get("stop_navigation_on_timeout", True),
            "stop_navigation_on_timeout",
        ),
        maps_dir=_resolve_project_path(
            raw.get("maps_dir", "botbrain_ws/src/g1_pkg/maps"), "maps_dir"
        ),
        selector=_resolve_project_path(
            raw.get("selector", "tools/nav/select_map_scene.sh"), "selector"
        ),
        scene_state_file=_resolve_project_path(
            raw.get("scene_state_file", "botbrain_ws/.runtime/map_scene"),
            "scene_state_file",
        ),
    )


def validate_map_assets(config: SwitcherConfig) -> None:
    if not config.selector.is_file():
        raise ValueError(f"map selector does not exist: {config.selector}")
    for floor, scene in config.floor_to_scene.items():
        pcd = config.maps_dir / f"{scene}_scans.pcd"
        map_yaml = config.maps_dir / f"{scene}.yaml"
        pgm = config.maps_dir / f"{scene}.pgm"
        for asset in (pcd, map_yaml, pgm):
            if not asset.is_file() or asset.stat().st_size == 0:
                raise ValueError(
                    f"floor {floor} scene {scene!r} has a missing or empty "
                    f"map asset: {asset}"
                )
        map_document = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
        if not isinstance(map_document, Mapping):
            raise ValueError(f"map YAML root must be a mapping: {map_yaml}")
        image = map_document.get("image")
        if not isinstance(image, str) or not image.strip():
            raise ValueError(f"map YAML has no image entry: {map_yaml}")
        image_path = Path(image.strip()).expanduser()
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        try:
            resolved_image = image_path.resolve(strict=True)
            resolved_pgm = pgm.resolve(strict=True)
        except FileNotFoundError as error:
            raise ValueError(f"map image does not exist: {error.filename}") from error
        if resolved_image != resolved_pgm:
            raise ValueError(
                f"{map_yaml} points to {resolved_image}, expected {resolved_pgm}"
            )


def extract_message_field(message: Any, field_path: str) -> Any:
    value = message
    for component in field_path.split("."):
        if not component or not hasattr(value, component):
            raise AttributeError(
                f"message has no configured field path {field_path!r}"
            )
        value = getattr(value, component)
    return value


def normalize_floor(value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError("boolean floor values are invalid")
    number = float(value)
    if not math.isfinite(number) or not number.is_integer():
        raise ValueError(f"floor value must be a finite integer, got {value!r}")
    return int(number)


def floor_is_authorized(
    floor: int,
    *,
    expected_floor: int,
    allow_uncommanded_switch: bool,
) -> bool:
    return allow_uncommanded_switch or (
        expected_floor >= 0 and floor == expected_floor
    )


def serialize_arrival_priors(
    priors: tuple[ArrivalPrior, ...]
) -> tuple[str, str]:
    serialized_poses = ";".join(
        f"{prior.x:.9g},{prior.y:.9g},{prior.yaw_deg:.9g}"
        for prior in priors
    )
    serialized_names = ";".join(prior.name for prior in priors)
    return serialized_poses, serialized_names


def select_transition_profile(
    config: SwitcherConfig,
    *,
    from_floor: int,
    target_floor: int,
    elevator_id: str,
) -> TransitionProfile:
    """Select an exact route prior, with legacy per-floor priors as fallback."""
    matches = [
        profile
        for profile in config.transitions
        if profile.elevator_id == elevator_id
        and profile.from_floor == from_floor
        and profile.target_floor == target_floor
    ]
    if len(matches) > 1:
        raise RuntimeError(
            "ambiguous transition profile for "
            f"elevator={elevator_id!r}, from={from_floor}, target={target_floor}"
        )
    if matches:
        return matches[0]
    return TransitionProfile(
        elevator_id=elevator_id,
        from_floor=from_floor,
        target_floor=target_floor,
        arrival_priors=config.arrival_priors.get(target_floor, ()),
    )


def selector_command(
    config: SwitcherConfig,
    scene: str,
    profile: Optional[TransitionProfile] = None,
) -> list[str]:
    command = [
        "bash",
        str(config.selector),
        scene,
    ]
    if config.restart_fast_lio:
        command.append("--restart-fast-lio")
    if profile is not None:
        command.extend(
            [
                "--localization-mode",
                "floor_transition",
                "--prior-search-radius-m",
                f"{profile.search_radius_m:.9g}",
                "--prior-yaw-range-deg",
                f"{profile.yaw_range_deg:.9g}",
                "--prior-search-xy-step-m",
                f"{profile.search_xy_step_m:.9g}",
                "--prior-search-yaw-step-deg",
                f"{profile.search_yaw_step_deg:.9g}",
                "--prior-max-nearby-candidates",
                str(profile.max_nearby_candidates),
            ]
        )
        if profile.arrival_priors:
            serialized_poses, serialized_names = serialize_arrival_priors(
                profile.arrival_priors
            )
            command.extend(
                [
                    "--initial-pose-priors",
                    serialized_poses,
                    "--initial-pose-prior-names",
                    serialized_names,
                ]
            )
    command.extend(
        [
            "--wait-ready",
            "--ready-timeout",
            str(config.ready_timeout_sec),
        ]
    )
    return command


def acquire_instance_lock() -> Any:
    lock_path = Path("/tmp/botbrain_floor_map_switcher.lock")
    handle = lock_path.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError(
            f"another floor map switcher holds {lock_path}"
        )
    handle.seek(0)
    handle.truncate()
    handle.write(f"{os.getpid()}\n")
    handle.flush()
    return handle


class FloorSwitchWorker:
    def __init__(
        self,
        config: SwitcherConfig,
        *,
        dry_run: bool,
        log: Callable[[str], None],
    ) -> None:
        self.config = config
        self.dry_run = dry_run
        self.log = log
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._process: Optional[subprocess.Popen[str]] = None
        self._result: Optional[SwitchResult] = None
        self._stop_event = threading.Event()

    def is_busy(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    def take_result(self) -> Optional[SwitchResult]:
        with self._lock:
            result = self._result
            self._result = None
            return result

    def start_switch(
        self,
        floor: int,
        scene: str,
        profile: TransitionProfile,
        still_valid: Callable[[], bool],
    ) -> bool:
        return self._start(
            target=lambda: self._switch_floor(
                floor,
                scene,
                profile,
                still_valid,
            ),
            name=f"floor-map-switch-{floor}",
        )

    def start_stop_navigation(self, reason: str) -> bool:
        return self._start(
            target=lambda: self._stop_navigation(reason),
            name="floor-map-navigation-stop",
        )

    def _start(self, *, target: Callable[[], SwitchResult], name: str) -> bool:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return False
            self._result = None

            def run() -> None:
                try:
                    result = target()
                except Exception as error:  # Keep the safety worker observable.
                    result = SwitchResult(
                        None,
                        None,
                        False,
                        f"unexpected switch worker error: {error}",
                    )
                with self._lock:
                    self._result = result

            self._thread = threading.Thread(target=run, name=name, daemon=True)
            self._thread.start()
            return True

    def close(self) -> None:
        self._stop_event.set()
        with self._lock:
            process = self._process
            thread = self._thread
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        if thread is not None:
            thread.join(timeout=10.0)

    def _run_streaming(self, command: list[str]) -> int:
        self.log("$ " + " ".join(command))
        if self.dry_run:
            return 0
        try:
            process = subprocess.Popen(
                command,
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except OSError as error:
            self.log(f"Unable to execute {command[0]!r}: {error}")
            return 127
        with self._lock:
            self._process = process
        try:
            if process.stdout is not None:
                for line in process.stdout:
                    self.log(line.rstrip())
            return process.wait()
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None

    def _run_quiet(self, command: list[str]) -> subprocess.CompletedProcess[str]:
        if self.dry_run:
            self.log("$ " + " ".join(command))
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
        try:
            return subprocess.run(
                command,
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
                timeout=20,
            )
        except subprocess.TimeoutExpired as error:
            return subprocess.CompletedProcess(
                command,
                124,
                stdout=error.stdout or "",
                stderr=error.stderr or "command timed out",
            )
        except OSError as error:
            return subprocess.CompletedProcess(
                command, 127, stdout="", stderr=str(error)
            )

    def _container_running(self, container: str) -> bool:
        result = self._run_quiet(
            ["docker", "inspect", "-f", "{{.State.Running}}", container]
        )
        return result.returncode == 0 and result.stdout.strip() == "true"

    def _localization_scene(self) -> Optional[str]:
        result = self._run_quiet(
            [
                "docker",
                "inspect",
                "--format",
                "{{range .Config.Env}}{{println .}}{{end}}",
                "g1_robot_localization",
            ]
        )
        if result.returncode != 0:
            return None
        scenes = [
            line.partition("=")[2]
            for line in result.stdout.splitlines()
            if line.startswith("MAP_SCENE=")
        ]
        return scenes[-1] if scenes else None

    def _scene_marker(self) -> Optional[str]:
        try:
            scene = self.config.scene_state_file.read_text(
                encoding="utf-8"
            ).strip()
        except OSError:
            return None
        return scene if SCENE_PATTERN.fullmatch(scene) else None

    def _verified_localization_is_running(self, scene: str) -> bool:
        return (
            self._container_running("g1_robot_localization")
            and self._localization_scene() == scene
            and self._scene_marker() == scene
        )

    def _ensure_navigation_stopped(self) -> bool:
        if self._run_streaming(
            ["docker", "compose", "stop", "navigation"]
        ) != 0:
            return False
        return self.dry_run or not self._container_running(
            "g1_robot_navigation"
        )

    def _navigation_is_active(self) -> bool:
        if not self._container_running("g1_robot_navigation"):
            return False
        result = self._run_quiet(
            [
                "docker",
                "exec",
                "g1_robot_navigation",
                "bash",
                "-lc",
                "source /opt/ros/humble/setup.bash; "
                "source /botbrain_ws/install/setup.bash; "
                "timeout 8 ros2 lifecycle get /g1_robot/bt_navigator",
            ]
        )
        return result.returncode == 0 and re.search(
            r"\bactive\b", result.stdout, flags=re.IGNORECASE
        ) is not None

    def _acquire_scene_lock(self) -> Optional[Any]:
        lock_path = Path("/tmp/botbrain_map_scene_switch.lock")
        handle = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.close()
            return None
        return handle

    def _wait_for_navigation(self) -> bool:
        if self.dry_run:
            return True
        deadline = time.monotonic() + self.config.navigation_ready_timeout_sec
        while time.monotonic() < deadline and not self._stop_event.is_set():
            if self._navigation_is_active():
                return True
            if not self._container_running("g1_robot_navigation"):
                logs = self._run_quiet(
                    [
                        "docker",
                        "compose",
                        "logs",
                        "--no-color",
                        "--tail",
                        "120",
                        "navigation",
                    ]
                )
                if logs.stdout.strip():
                    self.log(logs.stdout.rstrip())
                return False
            self._stop_event.wait(2.0)
        return False

    def _start_navigation(self) -> bool:
        command = [
            "docker",
            "compose",
            "--profile",
            "navigation",
            "up",
            "-d",
            "--force-recreate",
            "navigation",
        ]
        if self._run_streaming(command) != 0:
            self._ensure_navigation_stopped()
            return False
        self.log("Waiting for /g1_robot/bt_navigator to become active")
        if self._wait_for_navigation():
            self.log("Navigation is active")
            return True
        self.log("Navigation did not become active; keeping it stopped")
        self._ensure_navigation_stopped()
        return False

    def _switch_floor(
        self,
        floor: int,
        scene: str,
        profile: TransitionProfile,
        still_valid: Callable[[], bool],
    ) -> SwitchResult:
        if self._stop_event.is_set():
            return SwitchResult(floor, scene, False, "manager is stopping")
        if not still_valid():
            return SwitchResult(
                floor, scene, False, "floor request is no longer fresh or armed"
            )

        # Every explicitly armed cross-floor transaction must pass the strict
        # selector readiness checks. A running container and scene marker do
        # not prove that localization ever became ready, especially after a
        # previous timeout or manager restart.
        if not self._ensure_navigation_stopped():
            return SwitchResult(
                floor,
                scene,
                False,
                "could not confirm that old Navigation stopped",
                retryable=True,
            )
        exit_code = self._run_streaming(
            selector_command(self.config, scene, profile)
        )
        if exit_code != 0:
            self._ensure_navigation_stopped()
            return SwitchResult(
                floor,
                scene,
                False,
                f"map selector exited with status {exit_code}",
                exit_code=exit_code,
                retryable=exit_code != 2,
            )

        # The selector releases its own flock when it returns. Reacquire the
        # same lock across the Nav2 startup and final verification so a manual
        # selector cannot replace localization in the middle of this phase.
        scene_lock = self._acquire_scene_lock()
        if scene_lock is None:
            self._ensure_navigation_stopped()
            return SwitchResult(
                floor,
                scene,
                False,
                "another scene transaction started before Navigation startup",
                exit_code=75,
                retryable=True,
            )
        try:
            # The strict selector can spend several minutes rebuilding FAST-LIO
            # and localizing. Recheck both the input and actual runtime before
            # Nav2 is allowed to move the robot.
            if not still_valid():
                self._ensure_navigation_stopped()
                return SwitchResult(
                    floor,
                    scene,
                    False,
                    "floor changed or became stale while localization was loading",
                )
            if not self._verified_localization_is_running(scene):
                self._ensure_navigation_stopped()
                return SwitchResult(
                    floor,
                    scene,
                    False,
                    "target localization scene is no longer the verified runtime",
                    retryable=True,
                )
            if self.config.start_navigation:
                if not self._navigation_is_active() and not self._start_navigation():
                    return SwitchResult(
                        floor,
                        scene,
                        False,
                        "localization passed but Navigation failed",
                        retryable=True,
                    )
                if (
                    not still_valid()
                    or not self._verified_localization_is_running(scene)
                ):
                    self._ensure_navigation_stopped()
                    return SwitchResult(
                        floor,
                        scene,
                        False,
                        "floor or localization changed while Navigation was starting",
                    )
                return SwitchResult(
                    floor, scene, True, "scene verified and Navigation is ready"
                )
            return SwitchResult(
                floor,
                scene,
                True,
                "scene verified; Navigation start is disabled by configuration",
            )
        finally:
            fcntl.flock(scene_lock.fileno(), fcntl.LOCK_UN)
            scene_lock.close()

    def _stop_navigation(self, reason: str) -> SwitchResult:
        self.log(f"Stopping Navigation: {reason}")
        exit_code = self._run_streaming(
            ["docker", "compose", "stop", "navigation"]
        )
        return SwitchResult(
            None,
            None,
            exit_code == 0,
            reason,
            stopped_only=True,
        )


def run_ros_node(config: SwitcherConfig, *, dry_run: bool) -> int:
    try:
        import rclpy
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String
    except ImportError as error:
        print(
            "ROS 2 Python modules are unavailable. Source /opt/ros/humble/"
            f"setup.bash and the floor-estimator workspace first: {error}",
            file=sys.stderr,
        )
        return 2

    try:
        floor_message_class = get_message(config.floor_message_type)
        motion_message_class = get_message(config.motion_message_type)
    except (AttributeError, ModuleNotFoundError, ValueError) as error:
        print(
            "Cannot load configured ROS message type "
            f"{config.floor_message_type!r} or "
            f"{config.motion_message_type!r}: {error}",
            file=sys.stderr,
        )
        return 2

    class FloorMapSwitcherNode(Node):
        def __init__(self) -> None:
            super().__init__("floor_map_switcher")
            self.declare_parameter("enabled", True)
            self.declare_parameter("expected_floor", config.expected_floor)
            self.declare_parameter("from_floor", config.from_floor)
            self.declare_parameter("elevator_id", config.elevator_id)
            self.declare_parameter(
                "arrival_confirmed", config.arrival_confirmed
            )
            self.declare_parameter(
                "allow_uncommanded_switch", config.allow_uncommanded_switch
            )
            self.add_on_set_parameters_callback(self._validate_parameters)

            self.gate = FloorObservationGate(
                stable_for_sec=config.stable_for_sec,
                min_observations=config.min_observations,
                message_timeout_sec=config.message_timeout_sec,
            )
            self.motion_gate = MotionStationarityGate(
                stationary_for_sec=config.stationary_for_sec,
                min_observations=config.stationary_min_observations,
                message_timeout_sec=config.motion_message_timeout_sec,
                max_linear_speed_mps=config.max_linear_speed_mps,
                max_angular_speed_rps=config.max_angular_speed_rps,
            )
            self.worker = FloorSwitchWorker(
                config,
                dry_run=dry_run,
                log=lambda text: self.get_logger().info(text),
            )
            self.last_attempt_floor: Optional[int] = None
            self.retry_not_before = 0.0
            self.switch_attempt_count = 0
            self.arrival_accepted = False
            self.completed_floor: Optional[int] = None
            self.timeout_stop_issued = False
            self.last_waiting_log_at = 0.0
            self.observed_expected_floor = config.expected_floor
            self.expected_floor_armed_at: Optional[float] = (
                time.monotonic() if config.expected_floor >= 0 else None
            )
            self.floor_topic_healthy = False
            self.last_topic_health_check_at = 0.0
            self.last_topic_health_signature: Optional[tuple[Any, ...]] = None

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                # Intentionally do not request the publisher's retained sample.
                # /floor_state has live heartbeat publication, and every switch
                # must be based on messages received after this process starts.
                durability=DurabilityPolicy.VOLATILE,
            )
            self.floor_subscription = self.create_subscription(
                floor_message_class,
                config.floor_topic,
                self._on_floor,
                qos,
            )
            motion_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.motion_subscription = self.create_subscription(
                motion_message_class,
                config.motion_topic,
                self._on_motion,
                motion_qos,
            )
            self.timer = self.create_timer(0.25, self._tick)
            status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status_publisher = self.create_publisher(
                String, "/floor_transition_status", status_qos
            )
            self._publish_status("IDLE", "waiting for a cross-floor task")
            self.get_logger().info(
                "Floor map switcher ready: "
                f"topic={config.floor_topic} type={config.floor_message_type} "
                f"field={config.floor_field} mapping={dict(config.floor_to_scene)} "
                f"motion_topic={config.motion_topic} "
                f"dry_run={dry_run}"
            )
            if not config.allow_uncommanded_switch and config.expected_floor < 0:
                self.get_logger().info(
                    "Waiting for an expected floor. Arm a cross-floor task with: "
                    "ros2 param set /floor_map_switcher expected_floor <N>"
                )

        def _publish_status(self, state: str, detail: str, **fields: Any) -> None:
            message = String()
            message.data = json.dumps(
                {"state": state, "detail": detail, **fields},
                ensure_ascii=False,
                sort_keys=True,
            )
            self.status_publisher.publish(message)

        def _validate_parameters(self, parameters: list[Parameter]) -> Any:
            for parameter in parameters:
                if parameter.name == "enabled" and parameter.type_ != (
                    Parameter.Type.BOOL
                ):
                    return SetParametersResult(
                        successful=False, reason="enabled must be boolean"
                    )
                if parameter.name == "allow_uncommanded_switch" and (
                    parameter.type_ != Parameter.Type.BOOL
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="allow_uncommanded_switch must be boolean",
                    )
                if parameter.name == "arrival_confirmed" and (
                    parameter.type_ != Parameter.Type.BOOL
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="arrival_confirmed must be boolean",
                    )
                if parameter.name == "expected_floor":
                    if parameter.type_ != Parameter.Type.INTEGER:
                        return SetParametersResult(
                            successful=False,
                            reason="expected_floor must be an integer",
                        )
                    floor = int(parameter.value)
                    if floor >= 0 and floor not in config.floor_to_scene:
                        return SetParametersResult(
                            successful=False,
                            reason=f"floor {floor} has no configured map scene",
                        )
                if parameter.name == "from_floor" and parameter.type_ != (
                    Parameter.Type.INTEGER
                ):
                    return SetParametersResult(
                        successful=False, reason="from_floor must be an integer"
                    )
                if parameter.name == "elevator_id":
                    if parameter.type_ != Parameter.Type.STRING:
                        return SetParametersResult(
                            successful=False, reason="elevator_id must be a string"
                        )
                    if re.search(r"[,;\r\n]", str(parameter.value)):
                        return SetParametersResult(
                            successful=False,
                            reason="elevator_id contains an invalid delimiter",
                        )
            return SetParametersResult(successful=True)

        def _on_floor(self, message: Any) -> None:
            try:
                raw_value = extract_message_field(message, config.floor_field)
                floor = normalize_floor(raw_value)
            except (AttributeError, TypeError, ValueError) as error:
                self.get_logger().error(f"Invalid floor message: {error}")
                return
            now = time.monotonic()
            previous = self.gate.candidate
            self.gate.observe(floor, now)
            self.timeout_stop_issued = False
            if previous != floor:
                if not self.worker.is_busy():
                    self.last_attempt_floor = None
                    self.retry_not_before = 0.0
                self.get_logger().info(
                    f"Floor candidate changed to {floor}; waiting "
                    f"{config.stable_for_sec:.1f}s and "
                    f"{config.min_observations} observations"
                )

        def _on_motion(self, message: Any) -> None:
            try:
                twist = message.twist.twist
                linear_speed = math.sqrt(
                    float(twist.linear.x) ** 2
                    + float(twist.linear.y) ** 2
                    + float(twist.linear.z) ** 2
                )
                angular_speed = math.sqrt(
                    float(twist.angular.x) ** 2
                    + float(twist.angular.y) ** 2
                    + float(twist.angular.z) ** 2
                )
                self.motion_gate.observe(
                    linear_speed, angular_speed, time.monotonic()
                )
            except (AttributeError, TypeError, ValueError) as error:
                self.get_logger().error(f"Invalid motion message: {error}")

        def _topic_health_check(self, now: float) -> None:
            if now - self.last_topic_health_check_at < 1.0:
                return
            self.last_topic_health_check_at = now
            publishers = self.get_publishers_info_by_topic(config.floor_topic)
            offered_types = tuple(sorted({item.topic_type for item in publishers}))
            offered_reliability = tuple(
                sorted(str(item.qos_profile.reliability) for item in publishers)
            )
            publisher_is_reliable = all(
                item.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                for item in publishers
            )
            signature = (
                len(publishers),
                offered_types,
                offered_reliability,
            )
            self.floor_topic_healthy = (
                len(publishers) == 1
                and offered_types == (config.floor_message_type,)
                and publisher_is_reliable
            )
            if signature == self.last_topic_health_signature:
                return
            self.last_topic_health_signature = signature
            if self.floor_topic_healthy:
                self.get_logger().info(
                    f"Verified one {config.floor_message_type} publisher on "
                    f"{config.floor_topic}"
                )
            else:
                self.get_logger().error(
                    f"Floor topic is not safe to consume: topic="
                    f"{config.floor_topic} publishers={len(publishers)} "
                    f"types={offered_types or 'none'} "
                    f"reliability={offered_reliability or 'none'}; exactly one "
                    f"RELIABLE publisher of type {config.floor_message_type} "
                    "is required"
                )

        def _handle_result(self, result: SwitchResult, now: float) -> None:
            if result.stopped_only:
                if not result.success:
                    self.get_logger().error(
                        f"Failed to stop Navigation: {result.message}"
                    )
                return
            if result.success:
                self.completed_floor = result.floor
                self.retry_not_before = 0.0
                self.switch_attempt_count = 0
                self.get_logger().info(
                    f"Floor {result.floor} scene {result.scene!r} is ready: "
                    f"{result.message}"
                )
                self._publish_status(
                    "READY", result.message,
                    floor=result.floor, scene=result.scene,
                )
                if (
                    not self.get_parameter(
                        "allow_uncommanded_switch"
                    ).value
                    and self.get_parameter("expected_floor").value
                    == result.floor
                ):
                    self.set_parameters(
                        [
                            Parameter("expected_floor", value=-1),
                            Parameter("arrival_confirmed", value=False),
                        ]
                    )
                    self.get_logger().info(
                        "Expected floor cleared; the manager is disarmed until "
                        "the next cross-floor task"
                    )
            else:
                expected_floor = int(
                    self.get_parameter("expected_floor").value
                )
                can_retry = (
                    result.retryable
                    and result.floor == expected_floor
                    and self.switch_attempt_count
                    < config.max_switch_attempts
                )
                if can_retry:
                    self.retry_not_before = now + config.retry_interval_sec
                    retry_text = (
                        f"automatic attempt {self.switch_attempt_count + 1}/"
                        f"{config.max_switch_attempts} is scheduled after "
                        f"{config.retry_interval_sec:.1f}s"
                    )
                    self._publish_status(
                        "RETRY_WAIT", result.message,
                        floor=result.floor, scene=result.scene,
                        next_attempt=self.switch_attempt_count + 1,
                        max_attempts=config.max_switch_attempts,
                    )
                else:
                    self.retry_not_before = math.inf
                    retry_text = (
                        "retry limit reached; manual diagnosis and re-arming "
                        "are required"
                        if result.retryable
                        else "failure is not retryable; manual diagnosis and "
                        "re-arming are required"
                    )
                    if (
                        not self.get_parameter(
                            "allow_uncommanded_switch"
                        ).value
                        and self.get_parameter("expected_floor").value >= 0
                    ):
                        self.set_parameters(
                            [Parameter("expected_floor", value=-1)]
                        )
                    self._publish_status(
                        "FAILED", result.message,
                        floor=result.floor, scene=result.scene,
                        attempts=self.switch_attempt_count,
                    )
                self.get_logger().error(
                    f"Floor {result.floor} scene {result.scene!r} failed: "
                    f"{result.message}. Navigation remains stopped; {retry_text}"
                )

        def _tick(self) -> None:
            now = time.monotonic()
            self._topic_health_check(now)

            expected_now = int(self.get_parameter("expected_floor").value)
            if expected_now != self.observed_expected_floor:
                self.observed_expected_floor = expected_now
                if expected_now >= 0:
                    self.expected_floor_armed_at = now
                    # A new explicit task must be checked even when it targets
                    # the same floor as the previous completed transaction.
                    self.completed_floor = None
                    self.last_attempt_floor = None
                    self.retry_not_before = 0.0
                    self.switch_attempt_count = 0
                    self.arrival_accepted = False
                    self.get_logger().info(
                        f"Cross-floor task armed for expected floor {expected_now}"
                    )
                    self._publish_status(
                        "ARMED", "cross-floor task armed",
                        expected_floor=expected_now,
                        from_floor=int(self.get_parameter("from_floor").value),
                        elevator_id=str(
                            self.get_parameter("elevator_id").value
                        ),
                    )
                    if self.worker.start_stop_navigation(
                        "cross-floor task armed"
                    ):
                        self.get_logger().info(
                            "Stopping Navigation while the robot is in the "
                            "cross-floor transition"
                        )
                else:
                    self.expected_floor_armed_at = None
                    self.arrival_accepted = False

            result = self.worker.take_result()
            if result is not None:
                self._handle_result(result, now)
                return

            if (
                expected_now >= 0
                and self.expected_floor_armed_at is not None
                and not self.arrival_accepted
                and now - self.expected_floor_armed_at
                > config.expected_floor_timeout_sec
                and not self.worker.is_busy()
            ):
                self.get_logger().error(
                    f"Expected floor {expected_now} was not accepted within "
                    f"{config.expected_floor_timeout_sec:.1f}s; disarming the "
                    "cross-floor task and keeping Navigation stopped"
                )
                self._publish_status(
                    "FAILED", "expected-floor wait timed out",
                    expected_floor=expected_now,
                )
                self.set_parameters([Parameter("expected_floor", value=-1)])
                self.retry_not_before = math.inf
                if not self.timeout_stop_issued:
                    self.timeout_stop_issued = self.worker.start_stop_navigation(
                        "expected-floor wait timed out"
                    )
                return

            if self.gate.mark_stale(now):
                self.get_logger().error(
                    f"No fresh {config.floor_topic} message for "
                    f"{config.message_timeout_sec:.1f}s"
                )
                armed = self.get_parameter("expected_floor").value >= 0
                if (
                    config.stop_navigation_on_timeout
                    and armed
                    and not self.timeout_stop_issued
                    and not self.worker.is_busy()
                ):
                    self.timeout_stop_issued = self.worker.start_stop_navigation(
                        "floor input timed out while a cross-floor task was armed"
                    )
                return

            if (
                not self.get_parameter("enabled").value
                or not self.floor_topic_healthy
                or self.worker.is_busy()
            ):
                return
            floor = self.gate.stable_floor(now)
            if floor is None:
                return
            scene = config.floor_to_scene.get(floor)
            if scene is None:
                if now - self.last_waiting_log_at >= 10.0:
                    self.last_waiting_log_at = now
                    self.get_logger().error(
                        f"Stable floor {floor} has no configured map; refusing "
                        "to switch or navigate"
                    )
                return

            expected_floor = expected_now
            allow_uncommanded = bool(
                self.get_parameter("allow_uncommanded_switch").value
            )
            if not floor_is_authorized(
                floor,
                expected_floor=expected_floor,
                allow_uncommanded_switch=allow_uncommanded,
            ):
                if now - self.last_waiting_log_at >= 10.0:
                    self.last_waiting_log_at = now
                    self.get_logger().info(
                        f"Stable floor is {floor}; waiting for expected_floor "
                        "before changing the navigation runtime"
                    )
                return
            if self.completed_floor == floor:
                return
            if self.last_attempt_floor == floor and now < self.retry_not_before:
                return

            if not self.motion_gate.is_stationary(now):
                if now - self.last_waiting_log_at >= 5.0:
                    self.last_waiting_log_at = now
                    self.get_logger().info(
                        f"Target floor {floor} is stable; waiting for robot "
                        f"stationarity: {self.motion_gate.status(now)}"
                    )
                return
            if (
                config.require_arrival_confirmation
                and not bool(
                    self.get_parameter("arrival_confirmed").value
                )
            ):
                if now - self.last_waiting_log_at >= 5.0:
                    self.last_waiting_log_at = now
                    self.get_logger().info(
                        f"Target floor {floor} and stationarity are confirmed; "
                        "waiting for arrival_confirmed"
                    )
                return
            from_floor = int(self.get_parameter("from_floor").value)
            elevator_id = str(self.get_parameter("elevator_id").value).strip()
            profile = select_transition_profile(
                config,
                from_floor=from_floor,
                target_floor=floor,
                elevator_id=elevator_id,
            )

            self.last_attempt_floor = floor

            def still_valid() -> bool:
                live_floor = self.gate.stable_floor(time.monotonic())
                live_expected = int(
                    self.get_parameter("expected_floor").value
                )
                live_uncommanded = bool(
                    self.get_parameter("allow_uncommanded_switch").value
                )
                return (
                    bool(self.get_parameter("enabled").value)
                    and self.floor_topic_healthy
                    and live_floor == floor
                    and self.motion_gate.is_stationary(time.monotonic())
                    and (
                        not config.require_arrival_confirmation
                        or bool(
                            self.get_parameter("arrival_confirmed").value
                        )
                    )
                    and floor_is_authorized(
                        floor,
                        expected_floor=live_expected,
                        allow_uncommanded_switch=live_uncommanded,
                    )
                )

            if self.worker.start_switch(
                floor,
                scene,
                profile,
                still_valid,
            ):
                self.switch_attempt_count += 1
                self.arrival_accepted = True
                matching_mode = (
                    f"{len(profile.arrival_priors)} elevator-exit prior(s) "
                    f"for elevator={profile.elevator_id!r}, "
                    f"from={profile.from_floor}, target={profile.target_floor}"
                    if profile.arrival_priors
                    else "floor-transition BBS/global point-cloud fallback"
                )
                self.get_logger().info(
                    f"Accepted stable floor {floor}; switching to scene "
                    f"{scene!r} using {matching_mode}, attempt "
                    f"{self.switch_attempt_count}/{config.max_switch_attempts}"
                )
                self._publish_status(
                    "LOCALIZING", matching_mode,
                    floor=floor, scene=scene,
                    attempt=self.switch_attempt_count,
                    max_attempts=config.max_switch_attempts,
                )

        def destroy_node(self) -> bool:
            self.worker.close()
            return super().destroy_node()

    rclpy.init()
    node = FloorMapSwitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read /floor_state and safely switch the G1 map scene"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"configuration YAML (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="subscribe and print transactions without changing Docker state",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate configuration and all mapped map assets, then exit",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        config = load_config(args.config)
        validate_map_assets(config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"Floor map switcher configuration error: {error}", file=sys.stderr)
        return 2
    if args.check_config:
        print(
            "Floor map switcher configuration is valid: "
            f"{dict(config.floor_to_scene)}"
        )
        return 0
    return run_ros_node(config, dry_run=args.dry_run)


if __name__ == "__main__":
    raise SystemExit(main())
