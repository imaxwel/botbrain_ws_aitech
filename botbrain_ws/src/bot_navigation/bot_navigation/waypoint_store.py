"""Scene-aware storage and grid checks for named navigation waypoints."""

import math
import os
import re
from pathlib import Path
from typing import Mapping, Optional, Sequence, Tuple

import yaml


DEFAULT_SCENE = "ug"
DEFAULT_SCENE_FILE = Path("/botbrain_ws/.runtime/map_scene")
FORMAT_VERSION = 2
_SCENE_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


def validate_scene(scene: str) -> str:
    scene = str(scene).strip()
    if not _SCENE_PATTERN.fullmatch(scene):
        raise ValueError(
            f"invalid map scene {scene!r}; use only letters, digits, "
            "'_' or '-'"
        )
    return scene


def current_scene(
    explicit_scene: Optional[str] = None,
    scene_file: Path = DEFAULT_SCENE_FILE,
    environ: Optional[Mapping[str, str]] = None,
) -> Tuple[str, str]:
    """Return the selected scene and a human-readable source description."""
    if explicit_scene:
        return validate_scene(explicit_scene), "--scene"

    scene_file = Path(scene_file).expanduser()
    if scene_file.is_file():
        selected = validate_scene(scene_file.read_text(encoding="utf-8"))
        return selected, str(scene_file)

    env = os.environ if environ is None else environ
    env_scene = env.get("MAP_SCENE", "").strip()
    if env_scene:
        return validate_scene(env_scene), "MAP_SCENE"

    return DEFAULT_SCENE, "default"


def empty_database() -> dict:
    return {"format_version": FORMAT_VERSION, "scenes": {}}


def load_database(
    path: Path,
    missing_ok: bool = False,
    legacy_scene: Optional[str] = None,
) -> dict:
    """Load the current format and safely migrate legacy one-scene files."""
    path = Path(path)
    if not path.exists():
        if missing_ok:
            return empty_database()
        raise FileNotFoundError(path)

    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"waypoints file must contain a YAML mapping: {path}")

    if "scenes" in raw:
        scenes = raw.get("scenes") or {}
        if not isinstance(scenes, dict):
            raise ValueError(f"'scenes' must be a YAML mapping: {path}")
        normalized = empty_database()
        for scene_name, scene_data in scenes.items():
            scene = validate_scene(scene_name)
            if scene_data is None:
                waypoints = {}
            elif isinstance(scene_data, dict) and "waypoints" in scene_data:
                waypoints = scene_data.get("waypoints") or {}
            else:
                raise ValueError(
                    f"scene {scene!r} must contain a 'waypoints' mapping: "
                    f"{path}"
                )
            if not isinstance(waypoints, dict):
                raise ValueError(
                    f"scene {scene!r} waypoints must be a YAML mapping: {path}"
                )
            normalized["scenes"][scene] = {"waypoints": dict(waypoints)}
        return normalized

    # Legacy files had one shared top-level waypoint mapping. Bind them to the
    # currently verified scene unless an explicit marker was already present.
    legacy_waypoints = raw.get("waypoints") or {}
    if not isinstance(legacy_waypoints, dict):
        raise ValueError(f"'waypoints' must be a YAML mapping: {path}")
    selected_legacy_scene = raw.get("scene") or legacy_scene or DEFAULT_SCENE
    selected_legacy_scene = validate_scene(selected_legacy_scene)
    database = empty_database()
    database["scenes"][selected_legacy_scene] = {
        "waypoints": dict(legacy_waypoints)
    }
    database["_legacy_format"] = True
    database["_legacy_scene"] = selected_legacy_scene
    return database


def scene_waypoints(database: dict, scene: str, create: bool = False) -> dict:
    scene = validate_scene(scene)
    scenes = database.setdefault("scenes", {})
    if create:
        return scenes.setdefault(scene, {"waypoints": {}}).setdefault(
            "waypoints", {}
        )
    return (scenes.get(scene) or {}).get("waypoints", {})


def save_database(path: Path, database: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "format_version": FORMAT_VERSION,
        "scenes": database.get("scenes", {}),
    }
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        yaml.safe_dump(document, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def scene_from_map_path(map_path: str) -> str:
    stem = Path(str(map_path).strip()).stem
    if stem.endswith("_scans"):
        stem = stem[: -len("_scans")]
    return validate_scene(stem)


def live_map_scene(node, timeout_sec: float = 5.0) -> str:
    """Read the scene from the map_server that is active on the ROS graph."""
    import rclpy
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(
        GetParameters,
        "/map_server/get_parameters",
    )
    try:
        if not client.wait_for_service(timeout_sec=max(0.1, timeout_sec)):
            raise RuntimeError(
                "/map_server parameter service is not available"
            )
        request = GetParameters.Request()
        request.names = ["yaml_filename"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        try:
            response = future.result() if future.done() else None
        except Exception as error:
            raise RuntimeError(
                f"failed reading /map_server yaml_filename: {error}"
            ) from error
        if response is None:
            raise RuntimeError("timed out reading /map_server yaml_filename")
        if len(response.values) != 1:
            raise RuntimeError("/map_server returned no yaml_filename")
        yaml_filename = response.values[0].string_value.strip()
        if not yaml_filename:
            raise RuntimeError("/map_server yaml_filename is empty")
        return scene_from_map_path(yaml_filename)
    finally:
        node.destroy_client(client)


def goal_grid_occupancy(
    *,
    data: Sequence[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    goal_x: float,
    goal_y: float,
    check_radius: float,
    occupied_threshold: int,
) -> dict:
    """Inspect occupied cells close to a world-frame goal position."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError(
            "occupancy grid dimensions and resolution must be positive"
        )
    if len(data) != width * height:
        raise ValueError(
            "occupancy grid data length does not match its dimensions"
        )

    dx = goal_x - origin_x
    dy = goal_y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    grid_x = cosine * dx + sine * dy
    grid_y = -sine * dx + cosine * dy
    center_x = math.floor(grid_x / resolution)
    center_y = math.floor(grid_y / resolution)
    if not (0 <= center_x < width and 0 <= center_y < height):
        return {
            "in_bounds": False,
            "occupied_count": 0,
            "max_occupancy": None,
            "unknown_count": 0,
        }

    radius = max(0.0, float(check_radius))
    radius_cells = math.ceil(radius / resolution)
    occupied_count = 0
    unknown_count = 0
    max_occupancy = -1
    for cell_y in range(
        max(0, center_y - radius_cells),
        min(height, center_y + radius_cells + 1),
    ):
        for cell_x in range(
            max(0, center_x - radius_cells),
            min(width, center_x + radius_cells + 1),
        ):
            cell_center_x = (cell_x + 0.5) * resolution
            cell_center_y = (cell_y + 0.5) * resolution
            distance = math.hypot(
                cell_center_x - grid_x,
                cell_center_y - grid_y,
            )
            if distance > radius + resolution * 0.71:
                continue
            value = int(data[cell_y * width + cell_x])
            if value < 0:
                unknown_count += 1
                continue
            max_occupancy = max(max_occupancy, value)
            if value >= occupied_threshold:
                occupied_count += 1

    return {
        "in_bounds": True,
        "occupied_count": occupied_count,
        "max_occupancy": max_occupancy if max_occupancy >= 0 else None,
        "unknown_count": unknown_count,
    }
