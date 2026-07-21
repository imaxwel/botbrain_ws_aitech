import sys
import types
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]
NAVIGATION_SOURCE = PROJECT_ROOT / "botbrain_ws/src/bot_navigation"
sys.path.insert(0, str(NAVIGATION_SOURCE))

from bot_navigation.waypoint_store import (  # noqa: E402
    current_scene,
    goal_grid_occupancy,
    live_map_scene,
    load_database,
    save_database,
    scene_waypoints,
)


def test_scene_resolution_uses_explicit_state_env_then_ug(tmp_path):
    scene_file = tmp_path / "map_scene"
    scene_file.write_text("floor4\n", encoding="utf-8")

    assert current_scene(None, scene_file, {}) == ("floor4", str(scene_file))
    assert current_scene(None, scene_file, {"MAP_SCENE": "aitech"}) == (
        "floor4", str(scene_file))
    assert current_scene("ug", scene_file, {"MAP_SCENE": "aitech"}) == (
        "ug", "--scene")
    assert current_scene(None, tmp_path / "missing", {}) == ("ug", "default")
    assert current_scene(
        None, tmp_path / "missing", {"MAP_SCENE": "aitech"}
    ) == ("aitech", "MAP_SCENE")


def test_scene_database_isolates_name_and_migrates_legacy(tmp_path):
    path = tmp_path / "waypoints.yaml"
    path.write_text(
        yaml.safe_dump({"waypoints": {"home": {"x": 1.0, "y": 2.0}}}),
        encoding="utf-8",
    )

    database = load_database(path, legacy_scene="aitech")
    assert scene_waypoints(database, "aitech")["home"]["x"] == 1.0
    assert scene_waypoints(database, "ug") == {}
    assert scene_waypoints(database, "floor4") == {}

    scene_waypoints(database, "floor4", create=True)["home"] = {
        "x": 40.0,
        "y": 41.0,
    }
    save_database(path, database)
    reloaded = load_database(path)

    assert scene_waypoints(reloaded, "aitech")["home"]["x"] == 1.0
    assert scene_waypoints(reloaded, "floor4")["home"]["x"] == 40.0
    saved = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert saved["format_version"] == 2


def test_live_map_scene_uses_humble_get_parameters_service(monkeypatch):
    class FakeGetParameters:
        class Request:
            def __init__(self):
                self.names = []

    class FakeFuture:
        def done(self):
            return True

        def result(self):
            value = types.SimpleNamespace(
                string_value="/botbrain_ws/src/g1_pkg/maps/aitech.yaml"
            )
            return types.SimpleNamespace(values=[value])

    class FakeClient:
        def __init__(self):
            self.request = None

        def wait_for_service(self, timeout_sec):
            assert timeout_sec == 2.0
            return True

        def call_async(self, request):
            self.request = request
            return FakeFuture()

    class FakeNode:
        def __init__(self):
            self.client = FakeClient()
            self.service_name = None
            self.destroyed = False

        def create_client(self, service_type, service_name):
            assert service_type is FakeGetParameters
            self.service_name = service_name
            return self.client

        def destroy_client(self, client):
            assert client is self.client
            self.destroyed = True

    def fake_spin_until_future_complete(node, future, timeout_sec):
        return None

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.spin_until_future_complete = fake_spin_until_future_complete
    fake_srv = types.ModuleType("rcl_interfaces.srv")
    fake_srv.GetParameters = FakeGetParameters
    fake_interfaces = types.ModuleType("rcl_interfaces")
    fake_interfaces.srv = fake_srv
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rcl_interfaces", fake_interfaces)
    monkeypatch.setitem(sys.modules, "rcl_interfaces.srv", fake_srv)

    node = FakeNode()
    assert live_map_scene(node, timeout_sec=2.0) == "aitech"
    assert node.service_name == "/map_server/get_parameters"
    assert node.client.request.names == ["yaml_filename"]
    assert node.destroyed is True


def test_goal_grid_check_rejects_occupied_near_goal_but_not_distant_cells():
    data = [0] * 100
    data[5 * 10 + 5] = 100

    occupied = goal_grid_occupancy(
        data=data,
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        goal_x=5.5,
        goal_y=5.5,
        check_radius=0.1,
        occupied_threshold=65,
    )
    free = goal_grid_occupancy(
        data=data,
        width=10,
        height=10,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        goal_x=2.5,
        goal_y=2.5,
        check_radius=0.1,
        occupied_threshold=65,
    )

    assert occupied["in_bounds"] is True
    assert occupied["occupied_count"] == 1
    assert occupied["max_occupancy"] == 100
    assert free["occupied_count"] == 0


def test_goal_grid_check_handles_rotated_origins_and_out_of_bounds():
    rotated = goal_grid_occupancy(
        data=[0, 100, 0, 0],
        width=2,
        height=2,
        resolution=1.0,
        origin_x=10.0,
        origin_y=20.0,
        origin_yaw=1.5707963267948966,
        goal_x=9.5,
        goal_y=21.5,
        check_radius=0.0,
        occupied_threshold=65,
    )
    outside = goal_grid_occupancy(
        data=[0, 0, 0, 0],
        width=2,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        goal_x=-0.1,
        goal_y=0.5,
        check_radius=0.0,
        occupied_threshold=65,
    )

    assert rotated["occupied_count"] == 1
    assert outside["in_bounds"] is False


def test_selector_persists_scene_only_after_runtime_map_verification():
    selector = (PROJECT_ROOT / "tools/nav/select_map_scene.sh").read_text(
        encoding="utf-8"
    )
    verification = selector.index('if [ "$runtime_verified" != true ]')
    readiness = selector.index('if [ "$wait_ready" = true ]', verification)
    persistence = selector.index("persist_selected_scene", readiness)
    completed = selector.index("switch_completed=true", persistence)

    assert verification < readiness < persistence < completed
    assert "botbrain_ws/.runtime/map_scene" in selector
    assert "mv -f \"$temporary\" \"$scene_state_file\"" in selector
    assert "waypoint_store.py" in selector
