"""Scene-aware destination aliases backed by the existing waypoint database."""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml

from bot_navigation.waypoint_store import load_database, scene_waypoints, validate_scene


class UnknownDestinationError(ValueError):
    """Raised when a destination is not configured for the active scene."""


@dataclass(frozen=True)
class WaypointResolution:
    requested: str
    scene: str
    waypoint_name: str
    waypoint: Mapping[str, object]


def normalize_destination(value: str) -> str:
    value = str(value).strip()
    value = re.sub(r"[，。！？、,.!?]+$", "", value)
    return "".join(value.split())


class DestinationResolver:
    def __init__(
        self,
        aliases: Mapping[str, Mapping[str, str]],
        waypoints: Mapping[str, Mapping[str, Mapping[str, object]]],
    ):
        self.aliases = {
            validate_scene(scene): {
                normalize_destination(alias): str(name).strip()
                for alias, name in scene_aliases.items()
            }
            for scene, scene_aliases in aliases.items()
        }
        self.waypoints = {
            validate_scene(scene): dict(scene_points)
            for scene, scene_points in waypoints.items()
        }

    @classmethod
    def from_files(cls, alias_file: Path, waypoint_file: Path) -> "DestinationResolver":
        alias_data = yaml.safe_load(Path(alias_file).read_text(encoding="utf-8")) or {}
        database = load_database(Path(waypoint_file), missing_ok=True)
        aliases = {
            scene: (scene_data or {}).get("aliases", {})
            for scene, scene_data in (alias_data.get("scenes", {}) or {}).items()
        }
        waypoints = {
            scene: scene_waypoints(database, scene)
            for scene in database.get("scenes", {})
        }
        return cls(aliases=aliases, waypoints=waypoints)

    def resolve(self, destination: str, scene: str) -> WaypointResolution:
        scene = validate_scene(scene)
        requested = normalize_destination(destination)
        if not requested:
            raise UnknownDestinationError("destination is empty")
        scene_points = self.waypoints.get(scene, {})
        canonical = self.aliases.get(scene, {}).get(requested, requested)
        waypoint = scene_points.get(canonical)
        if waypoint is None:
            raise UnknownDestinationError(
                f"destination {destination!r} is not configured in scene {scene!r}"
            )
        return WaypointResolution(
            requested=requested,
            scene=scene,
            waypoint_name=canonical,
            waypoint=waypoint,
        )

    def available_destinations(self, scene: str) -> list[str]:
        scene = validate_scene(scene)
        scene_points = self.waypoints.get(scene, {})
        canonical = set(scene_points)
        aliases = {
            alias
            for alias, target in self.aliases.get(scene, {}).items()
            if target in scene_points
        }
        return sorted(aliases | canonical)
