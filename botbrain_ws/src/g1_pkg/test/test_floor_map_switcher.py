import importlib.util
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = PROJECT_ROOT / "tools" / "nav" / "floor_map_switcher.py"
CONFIG = PROJECT_ROOT / "configs" / "floor_map_switcher.yaml"


def _load_module():
    spec = importlib.util.spec_from_file_location("floor_map_switcher", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_floor_scene_map_has_complete_matching_assets():
    module = _load_module()
    config = module.load_config(CONFIG)

    assert dict(config.floor_to_scene) == {
        13: "floor13",
        14: "floor14",
        15: "floor15",
    }
    assert config.arrival_priors == {}
    assert config.transitions == ()
    assert config.from_floor == -1
    assert config.elevator_id == ""
    assert config.motion_topic == "/g1_robot/odom"
    assert config.max_switch_attempts == 3
    module.validate_map_assets(config)


def test_one_floor_sample_cannot_trigger_switch():
    module = _load_module()
    gate = module.FloorObservationGate(
        stable_for_sec=3.0,
        min_observations=3,
        message_timeout_sec=5.0,
    )

    gate.observe(14, 10.0)
    assert gate.stable_floor(13.1) is None

    gate.observe(14, 11.0)
    gate.observe(14, 12.0)
    assert gate.stable_floor(12.9) is None
    assert gate.stable_floor(13.0) == 14


def test_timeout_resets_floor_confirmation_window():
    module = _load_module()
    gate = module.FloorObservationGate(
        stable_for_sec=2.0,
        min_observations=2,
        message_timeout_sec=3.0,
    )

    gate.observe(13, 1.0)
    gate.observe(13, 2.0)
    assert gate.stable_floor(3.0) == 13
    assert gate.mark_stale(5.1) is True
    assert gate.stable_floor(5.1) is None

    gate.observe(13, 6.0)
    assert gate.stable_floor(8.1) is None


def test_expected_floor_gates_destructive_switch():
    module = _load_module()

    assert not module.floor_is_authorized(
        14, expected_floor=-1, allow_uncommanded_switch=False
    )
    assert not module.floor_is_authorized(
        14, expected_floor=15, allow_uncommanded_switch=False
    )
    assert module.floor_is_authorized(
        14, expected_floor=14, allow_uncommanded_switch=False
    )
    assert module.floor_is_authorized(
        14, expected_floor=-1, allow_uncommanded_switch=True
    )


def test_cross_floor_command_uses_strict_selector_checks():
    module = _load_module()
    config = module.load_config(CONFIG)
    priors = (
        module.ArrivalPrior("elevator_a_forward", 8.2, 6.7, 90.0),
        module.ArrivalPrior("elevator_a_reverse", 8.2, 6.7, -90.0),
    )

    profile = module.TransitionProfile(
        elevator_id="elevator_a",
        from_floor=13,
        target_floor=14,
        arrival_priors=priors,
    )
    command = module.selector_command(config, "floor14", profile)

    assert command[:3] == ["bash", str(config.selector), "floor14"]
    assert "--restart-fast-lio" in command
    assert command[command.index("--localization-mode") + 1] == (
        "floor_transition"
    )
    assert command[command.index("--initial-pose-priors") + 1] == (
        "8.2,6.7,90;8.2,6.7,-90"
    )
    assert command[command.index("--initial-pose-prior-names") + 1] == (
        "elevator_a_forward;elevator_a_reverse"
    )
    assert "--wait-ready" in command
    assert command[-2:] == ["--ready-timeout", "300"]


def test_cross_floor_without_exit_priors_still_uses_transition_fallback():
    module = _load_module()
    config = module.load_config(CONFIG)

    profile = module.select_transition_profile(
        config, from_floor=13, target_floor=14, elevator_id="elevator_a"
    )
    command = module.selector_command(config, "floor14", profile)

    assert command[command.index("--localization-mode") + 1] == (
        "floor_transition"
    )
    assert "--initial-pose-priors" not in command
    assert "--wait-ready" in command
    assert "target floor has no configured elevator-exit prior" not in (
        SCRIPT.read_text(encoding="utf-8")
    )


def test_route_specific_transition_beats_legacy_floor_prior(tmp_path):
    module = _load_module()
    raw = CONFIG.read_text(encoding="utf-8")
    configured = tmp_path / "transitions.yaml"
    configured.write_text(
        raw.replace(
            "transitions: []",
            "transitions:\n"
            "  - elevator_id: elevator_a\n"
            "    from_floor: 13\n"
            "    target_floor: 14\n"
            "    search_radius_m: 1.5\n"
            "    yaw_range_deg: 30.0\n"
            "    search_xy_step_m: 0.5\n"
            "    search_yaw_step_deg: 10.0\n"
            "    max_nearby_candidates: 32\n"
            "    arrival_priors:\n"
            "      - {name: exact_exit, x: 8.2, y: 6.7, yaw_deg: 90.0}",
        ),
        encoding="utf-8",
    )
    config = module.load_config(configured)

    exact = module.select_transition_profile(
        config, from_floor=13, target_floor=14, elevator_id="elevator_a"
    )
    fallback = module.select_transition_profile(
        config, from_floor=15, target_floor=14, elevator_id="elevator_a"
    )

    assert exact.arrival_priors[0].name == "exact_exit"
    assert exact.search_radius_m == 1.5
    assert exact.yaw_range_deg == 30.0
    assert fallback.arrival_priors == ()


def test_motion_gate_requires_fresh_continuous_stationarity():
    module = _load_module()
    gate = module.MotionStationarityGate(
        stationary_for_sec=2.0,
        min_observations=3,
        message_timeout_sec=1.0,
        max_linear_speed_mps=0.05,
        max_angular_speed_rps=0.10,
    )

    gate.observe(0.01, 0.02, 10.0)
    gate.observe(0.01, 0.02, 11.0)
    gate.observe(0.01, 0.02, 12.0)
    assert gate.is_stationary(12.0)

    gate.observe(0.06, 0.02, 12.1)
    assert not gate.is_stationary(12.1)
    gate.observe(0.0, 0.0, 13.0)
    gate.observe(0.0, 0.0, 14.0)
    gate.observe(0.0, 0.0, 15.0)
    assert gate.is_stationary(15.0)
    assert not gate.is_stationary(16.1)


def test_arrival_prior_config_rejects_unknown_floor_and_bad_pose(tmp_path):
    module = _load_module()
    raw = CONFIG.read_text(encoding="utf-8")

    unknown_floor = tmp_path / "unknown_floor.yaml"
    unknown_floor.write_text(
        raw.replace("arrival_priors: {}", (
            "arrival_priors:\n"
            "  99:\n"
            "    - {name: lift, x: 1.0, y: 2.0, yaw_deg: 0.0}"
        )),
        encoding="utf-8",
    )
    try:
        module.load_config(unknown_floor)
    except ValueError as error:
        assert "has no floor_to_scene mapping" in str(error)
    else:
        raise AssertionError("unknown arrival-prior floor was accepted")

    bad_pose = tmp_path / "bad_pose.yaml"
    bad_pose.write_text(
        raw.replace("arrival_priors: {}", (
            "arrival_priors:\n"
            "  14:\n"
            "    - {name: lift, x: nope, y: 2.0, yaw_deg: 0.0}"
        )),
        encoding="utf-8",
    )
    try:
        module.load_config(bad_pose)
    except ValueError as error:
        assert "requires numeric x, y and yaw_deg" in str(error)
    else:
        raise AssertionError("non-numeric arrival prior was accepted")


def test_safety_order_stops_old_navigation_and_bounds_automatic_retries():
    source = SCRIPT.read_text(encoding="utf-8")
    switch_body = source.split("def _switch_floor(", 1)[1].split(
        "def _stop_navigation(", 1
    )[0]

    assert switch_body.index("self._ensure_navigation_stopped()") < (
        switch_body.index("selector_command(self.config, scene, profile)")
    )
    assert "durability=DurabilityPolicy.VOLATILE" in source
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
    assert "config.max_switch_attempts" in source
    assert "result.retryable" in source
    assert "exit_code=75,\n                retryable=True" in switch_body
    assert "self.retry_not_before = math.inf" in source
