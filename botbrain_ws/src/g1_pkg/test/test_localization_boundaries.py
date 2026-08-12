import ast
import re
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]


def _read(relative_path):
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


def _run_selector_filter(function_name, next_function_name, log_text):
    selector = _read("tools/nav/select_map_scene.sh")
    start = selector.index(f"{function_name}() {{")
    end = selector.index(f"\n{next_function_name}() {{", start)
    definition = selector[start:end]
    result = subprocess.run(
        ["bash", "-c", f"{definition}\n{function_name}"],
        input=log_text,
        text=True,
        capture_output=True,
        check=True,
    )
    return result.stdout.splitlines()


def test_open3d_directory_is_an_overridable_cache_default():
    source = _read("botbrain_ws/src/open3d_loc/CMakeLists.txt")
    match = re.search(r"set\s*\(\s*Open3D_DIR\b(?P<body>.*?)\)", source, re.DOTALL)

    assert match is not None
    definition = match.group("body")
    assert '"/opt/open3d/lib/cmake/Open3D"' in definition
    assert "CACHE PATH" in definition
    assert "FORCE" not in definition
    assert "/root/3d_nav_g1" not in source


def test_localization_launch_keeps_pcd_and_grid_map_arguments_separate():
    source = _read("botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")

    assert "DeclareLaunchArgument('map_scene', default_value='ug')" in source
    assert "DeclareLaunchArgument('map_file', default_value='')" in source
    assert "DeclareLaunchArgument('grid_map_file', default_value='')" in source
    assert "'path_map':                 LaunchConfiguration('map_file')" in source
    assert "'yaml_filename': LaunchConfiguration('grid_map_file')" in source
    assert "maps_dir = os.path.join(workspace_dir, 'src', 'g1_pkg', 'maps')" in source
    assert "f'{scene}_scans.pcd'" in source
    assert "f'{scene}.yaml'" in source
    assert "SetLaunchConfiguration('map_file', str(resolved_pcd))" in source
    assert "SetLaunchConfiguration('grid_map_file', str(resolved_grid))" in source
    assert "Map selection: scene=" in source
    assert "3D/2D map scene mismatch" in source
    assert "using_scene_defaults = not map_file and not grid_map_file" in source
    assert "Requested map_scene={scene}" in source
    assert "Map scene {scene!r} is incomplete" in source
    assert "resolved_pcd.stat().st_size == 0" in source
    assert "image_path.suffix.lower() != '.pgm'" in source


def test_default_map_scene_files_are_a_complete_matching_set():
    maps = PROJECT_ROOT / "botbrain_ws/src/g1_pkg/maps"
    pcd = maps / "ug_scans.pcd"
    grid = maps / "ug.yaml"
    grid_data = yaml.safe_load(grid.read_text(encoding="utf-8"))
    image = (grid.parent / grid_data["image"]).resolve(strict=True)

    assert pcd.resolve(strict=True).stem == "ug_scans"
    assert grid.resolve(strict=True).stem == "ug"
    assert image.stem == "ug"
    assert image.suffix.lower() == ".pgm"


def test_localization_service_starts_the_installed_launch_file_directly():
    source = _read("docker-compose.yaml")

    assert "/botbrain_ws/start_localization.sh" not in source
    assert "LOCALIZATION_START_DELAY_SEC" in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "source /botbrain_ws/install/setup.bash" in source
    assert "exec ros2 launch g1_pkg localization_3d.launch.py" in source
    assert "MAP_SCENE: ${MAP_SCENE:-ug}" in source
    compose = yaml.safe_load(source)
    assert compose["services"]["localization"]["restart"] == "no"
    assert compose["services"]["localization"]["environment"]["MAP_SCENE"] == (
        "${MAP_SCENE:-ug}"
    )
    localization_env = compose["services"]["localization"]["environment"]
    assert localization_env["OMP_NUM_THREADS"] == "2"
    assert localization_env["OMP_DYNAMIC"] == "FALSE"
    assert localization_env["OPENBLAS_NUM_THREADS"] == "1"
    assert localization_env["LOCALIZATION_START_DELAY_SEC"] == (
        "${LOCALIZATION_START_DELAY_SEC:-0}"
    )
    assert compose["services"]["fast_lio"]["environment"][
        "FAST_LIO_START_DELAY_SEC"
    ] == "${FAST_LIO_START_DELAY_SEC:-0}"
    localization_command = compose["services"]["localization"]["command"][-1]
    assert localization_command.count("MAP_SCENE") == 1
    assert 'map_scene:="$${MAP_SCENE}"' in localization_command
    assert "map_file:=" not in localization_command
    assert "grid_map_file:=" not in localization_command
    assert compose["services"]["navigation"]["restart"] == "no"
    navigation_command = compose["services"]["navigation"]["command"][-1]
    assert "navigation_preflight.py" in navigation_command
    assert "allow_pose_derived_twist:=false" in navigation_command
    navigation_env = compose["services"]["navigation"]["environment"]
    assert navigation_env["NAVIGATION_START_DELAY_SEC"] == (
        "${NAVIGATION_START_DELAY_SEC:-0}"
    )
    assert navigation_env["NAVIGATION_PREFLIGHT_TIMEOUT_SEC"] == (
        "${NAVIGATION_PREFLIGHT_TIMEOUT_SEC:-60}"
    )
    assert "sleep 30" not in navigation_command


def test_map_scene_selector_recreates_and_verifies_localization_container():
    selector = _read("tools/nav/select_map_scene.sh")
    compact_runbook = _read("建图导航指令.md")

    assert 'MAP_SCENE="$scene" LOCALIZATION_START_DELAY_SEC=0' in selector
    assert "FAST_LIO_START_DELAY_SEC=0" in selector
    assert "FAST_LIO_MAPPING_PROFILE=default" in selector
    assert "docker compose rm -f navigation localization" in selector
    assert "--force-recreate --no-deps localization" in selector
    assert "docker inspect g1_robot_localization" in selector
    assert 'expected_log="Map selection: scene=$scene "' in selector
    assert "ros2 param get /global_localization_node path_map" in selector
    assert "ros2 param get /map_server yaml_filename" in selector
    assert "topic_publisher_count" in selector
    assert "Unable to parse publisher count" in selector
    assert "map_publishers" in selector
    assert "unable to query old map publishers" in selector
    assert "legacy container g1_robot_mapping is running" in selector
    assert "docker compose up -d --force-recreate foxglove" not in selector
    assert "Connect workstation RViz2 through Zenoh." in selector
    assert "--restart-fast-lio" in selector
    assert "--wait-ready" in selector
    assert "--ready-timeout" in selector
    assert "navigation_preflight.py" in selector
    assert "localization is ready for navigation" in selector
    assert "wait_for_consecutive_icp_accepts" in selector
    assert "3 consecutive accepted ICP updates" in selector
    assert "ICP: accepted=true" in selector
    assert "Rejecting ICP|Skipping ICP|ICP: accepted=false" in selector
    assert "fitness_value <= min_fitness" in selector
    assert "rmse_value > max_rmse" in selector
    assert "ICP_MIN_FITNESS=0.50" in selector
    assert "ICP_MAX_RMSE=0.30" in selector
    assert "ICP accepted: %s %s" in selector
    assert 'docker logs --since "$since"' in selector
    assert "localization_active_logs 15s" in selector
    assert 'current_container_id" != "$localization_container_id' in selector
    assert "Latest ICP decisions from the current localization container" in selector
    assert "ICP stability progress: ${streak_count}/3" in selector
    assert "ICP stability progress reset:" in selector
    assert "Localization matching update:" in selector
    assert "verify_navigation_topic_publishers" in selector
    assert "print_unitree_twist_diagnostics" in selector
    assert "verify_navigation_install_matches_source" in selector
    assert "navigation install space is missing or stale" in selector
    assert (
        "bot_navigation/lib/bot_navigation/localization_monitor.py"
    ) in selector
    assert (
        "bot_navigation/share/bot_navigation/launch/nav_utils.launch.py"
    ) in selector
    assert (
        "g1_pkg/share/g1_pkg/launch/robot_interface.launch.py"
    ) in selector
    assert (
        "g1_pkg/share/g1_pkg/config/pointcloud_to_laserscan_params.yaml"
    ) in selector
    assert (
        "bot_bringup/share/bot_bringup/config/twist_mux.yaml"
    ) in selector
    assert (
        "Rebuild fast_lio, open3d_loc, g1_pkg, bot_navigation and "
        "bot_bringup before using --wait-ready."
    ) in selector
    assert "global_localization_node" in selector
    assert "fast_lio/lib/fast_lio/fastlio_mapping" in selector
    assert "g1_write_node" in selector
    assert "allow_pose_derived_twist:=false" in selector
    assert 'grep -Fq "twist_source=unitree"' in selector
    assert "ros2 lifecycle get /g1_robot/robot_read_node" in selector
    assert "ros2 topic hz /lf/odommodestate" in selector
    assert "ros2 topic hz /g1_robot/odom" in selector
    assert "/Odometry_loc" in selector
    assert "/cloud_registered_body_1" in selector
    assert "/g1_robot/odom" in selector
    assert "navigation topics must each have exactly 1 publisher" in selector
    assert "Navigation topic publisher check ${stable_rounds}/${required_rounds}" in selector
    assert '[ "$stable_rounds" -gt 0 ]' in selector
    assert "readonly PUBLISHER_STABLE_ROUNDS=3" in selector
    assert "restore_previous_runtime" in selector
    assert "arm_switch_rollback" in selector
    assert "Previous runtime restored" in selector
    assert "trap 'handle_selector_exit" in selector
    assert "flock -n 9" in selector
    assert 'grep -Fq "IMU Initial Done"' in selector
    assert "fast_lio_timing_window" in selector
    assert 'timing_healthy" -ge 4' in selector
    assert 'timing_latest_ok" -eq 1' in selector
    assert "ros2 topic echo /Odometry_loc --once" in selector
    assert "ros2 topic echo /cloud_registered_1" in selector
    assert "--qos-reliability best_effort --once" in selector
    assert "docker compose ps -aq --all localization" in selector
    assert "old map publishers are still visible" in selector
    assert '[ "$SECONDS" -lt "$old_publisher_deadline" ] ||' in selector
    assert '[ "$old_publisher_zero_rounds" -gt 0 ]' in selector
    assert "botbrain_ws/install/fast_lio/share/fast_lio/config/mid360.yaml" in selector
    assert "pcd_save_en:" in selector
    assert 'bash tools/nav/select_map_scene.sh "$scene"' in compact_runbook

    publisher_count_body = selector.split(
        "topic_publisher_count() {", 1)[1].split("wait_for_restarted_fast_lio() {", 1)[0]
    ros_setup = publisher_count_body.index(
        "source /opt/ros/humble/setup.bash")
    nounset = publisher_count_body.index("set -u", ros_setup)
    assert ros_setup < nounset
    assert "set +u" in publisher_count_body[:ros_setup]

    readiness_body = selector.split(
        "wait_for_localization_ready() {", 1)[1].split("\nmaps=", 1)[0]
    assert readiness_body.index("wait_for_localization_preflight") < (
        readiness_body.index("wait_for_consecutive_icp_accepts")
    )
    assert readiness_body.rindex("wait_for_localization_preflight") > (
        readiness_body.index("wait_for_consecutive_icp_accepts")
    )
    assert readiness_body.index("wait_for_consecutive_icp_accepts") < (
        readiness_body.index("verify_navigation_topic_publishers")
    )


def test_fast_lio_timing_window_allows_one_safely_dropped_scan():
    output = _run_selector_filter(
        "fast_lio_timing_window",
        "verify_fast_lio_topics",
        """[FAST_LIO_TIMING] ok=true sample=1
[FAST_LIO_TIMING] ok=true sample=2
[FAST_LIO_TIMING] ok=false sample=3
[FAST_LIO_TIMING] ok=true sample=4
[FAST_LIO_TIMING] ok=true sample=5
""",
    )
    assert output == ["5", "4", "1"]


def test_icp_streak_requires_three_decisions_after_the_last_rejection():
    output = _run_selector_filter(
        "trailing_accepted_icp_streak",
        "verify_navigation_topic_publishers",
        """ICP: accepted=true fitness=0.900 rmse=0.100
ICP: accepted=true fitness=0.910 rmse=0.090
ICP: accepted=false fitness=0.400 rmse=0.350
ICP: accepted=true fitness=0.800 rmse=0.120
ICP: accepted=true fitness=0.820 rmse=0.110
ICP: accepted=true fitness=0.830 rmse=0.100
""",
    )
    assert output[0] == "3"
    assert len(output) == 4
    assert all("accepted=true" in line for line in output[1:])


def test_fast_lio_service_execs_launch_for_graceful_map_save_shutdown():
    source = _read("docker-compose.yaml")
    compose = yaml.safe_load(source)

    assert "exec ros2 launch --noninteractive g1_pkg fast_lio.launch.py" in source
    assert "stop_signal: SIGINT" in source
    assert "stop_grace_period: 180s" in source
    assert compose["services"]["localization"]["profiles"] == ["navigation"]
    assert compose["services"]["navigation"]["profiles"] == ["navigation"]
    fast_lio_env = compose["services"]["fast_lio"]["environment"]
    assert fast_lio_env["FAST_LIO_MAPPING_MODE"] == (
        "${FAST_LIO_MAPPING_MODE:-auto}"
    )
    assert fast_lio_env["FAST_LIO_MAPPING_SAVE"] == (
        "${FAST_LIO_MAPPING_SAVE:-auto}"
    )
    assert fast_lio_env["FAST_LIO_MAP_FILE"] == "${FAST_LIO_MAP_FILE:-}"


def test_fast_lio_launch_preserves_dense_grid_input_and_pcd_flush():
    source = _read("botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py")
    grid_source = _read("botbrain_ws/src/g1_pkg/scripts/grid_accumulator.py")

    assert "sigterm_timeout='150'" in source
    assert "sigkill_timeout='20'" in source
    assert "'--rate',           '2.0'" in source
    assert "'--process-every',  '1'" in source
    assert "'--debug-clouds'" not in source
    assert "FAST_LIO_MAPPING_MODE" in source
    assert "FAST_LIO_MAPPING_SAVE" in source
    assert "FAST_LIO_MAP_FILE" in source
    assert "if mapping_mode:" in source
    assert "'pcd_save.pcd_save_en': mapping_save_en" in source
    assert "'/cloud_registered_1'" in source
    assert "'/Odometry_loc'" in source
    assert "'--map-frame',      'camera_init'" in source
    assert "'/loop_closure/cloud_registered'" not in source
    assert "'/loop_closure/odometry'" not in source
    assert "'/loop_closure/reset_grid'" not in source
    assert "'--skip-frames',    '60'" in source
    assert "def reset_cb" in grid_source  # optional compatibility, not wired
    assert "if rclpy.ok():" in grid_source


def test_grid_uses_local_floor_height_without_extrapolating_plane_tilt():
    source = _read("botbrain_ws/src/g1_pkg/scripts/grid_accumulator.py")

    assert "floor_at_body = float(plane_height(" in source
    assert "candidate, body_position[0], body_position[1]" in source
    assert (
        "candidate = np.array([0.0, 0.0, floor_at_body], "
        "dtype=np.float64)"
    ) in source


def test_grid_rejected_floor_fit_cannot_write_with_stale_plane():
    source = _read("botbrain_ws/src/g1_pkg/scripts/grid_accumulator.py")

    assert "self.force_plane_refit = True" in source
    assert "self.force_plane_refit or" in source
    assert "plane_valid_for_scan = self._update_ground_plane(" in source
    assert "self.plane_coeffs is None or not plane_valid_for_scan" in source
    candidate_rejection = source.split("if candidate is None:", 1)[1].split(
        "# Each mapped building floor", 1
    )[0]
    assert "self.force_plane_refit = True" in candidate_rejection
    assert "return False" in candidate_rejection
    jump_rejection = source.split(
        'self.plane_metrics["reason"] = "plane_jump"', 1
    )[1].split("self.plane_coeffs = (", 1)[0]
    assert "self.force_plane_refit = True" in jump_rejection
    assert "return False" in jump_rejection


def test_loop_closure_is_observation_only_outside_mapping_and_live_tf_is_opt_in():
    package_root = PROJECT_ROOT / "botbrain_ws/src/g1_loop_closure"
    source = (package_root / "src/loop_closure_node.cpp").read_text(encoding="utf-8")
    optimizer_source = (package_root / "src/pose_graph.cpp").read_text(encoding="utf-8")
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")
    validation_header = (
        package_root / "include/g1_loop_closure/loop_validation.hpp"
    ).read_text(encoding="utf-8")
    config = (package_root / "config/loop_closure.yaml").read_text(encoding="utf-8")
    launch = (package_root / "launch/loop_closure.launch.py").read_text(encoding="utf-8")
    compose = yaml.safe_load(_read("docker-compose.yaml"))
    fast_lio_launch = _read("botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py")

    assert (package_root / "CMakeLists.txt").is_file()
    assert (package_root / "package.xml").is_file()
    assert "BuildDescriptor" in source
    assert "DescriptorSimilarity" in source
    assert "pcl::IterativeClosestPoint" in source
    assert "std::sqrt(mean_squared_fitness)" in source
    assert "RMS correspondence distance in metres" in config
    loop_params = yaml.safe_load(config)["/**"]["ros__parameters"]
    assert loop_params["icp_max_yaw_deg"] == 180.0
    assert loop_params["icp_max_odom_delta_yaw_deg"] == 25.0
    assert loop_params["loop_confirmation_count"] == 3
    assert loop_params["loop_confirmation_max_translation_m"] <= 0.75
    assert loop_params["loop_confirmation_max_yaw_deg"] <= 8.0
    assert "const double yaw_guess = odometry_guess.yaw;" in source
    assert "odometry_guess.yaw + descriptor_yaw" not in source
    assert "IsLoopYawConsistent(" in source
    assert "measurement_yaw - odometry_yaw" in validation_header
    assert '#include "g1_loop_closure/loop_validation.hpp"' in source
    assert "target_include_directories(loop_closure_node BEFORE PRIVATE" in cmake
    assert "install(DIRECTORY include/" in cmake
    assert "ament_export_include_directories(include)" in cmake
    assert (package_root / "test/test_pose_graph_geometry.cpp").is_file()
    assert "ConfirmLoopConstraint" in source
    assert "awaiting consistent confirmations" in source
    assert 'std::max<int64_t>(\n        3, declare_parameter<int>("loop_confirmation_count", 3))' in source
    assert "pending_loop_confirmations_.front().global_correction" in source
    assert "pending_loop_confirmations_.back().global_correction" not in source
    assert source.index("ConfirmLoopConstraint(") < source.index(
        "accepted_loop_edges_.insert("
    )
    assert source.index("ShouldCreateKeyframe(*odom, stamp)") < (
        source.index("pcl::fromROSMsg(*message, *world_cloud)")
    )
    assert "Eigen::SimplicialLDLT" in optimizer_source
    assert "enable_pose_graph" in source
    assert "accepted_loop_edges_" in source
    assert "enable_pose_graph: false" in config
    assert "DeclareLaunchArgument(" in launch
    assert "export_optimized_map_path" in launch

    assert "publish_live_correction: false" in config
    assert "publish_live_correction" in source
    assert "TransformBroadcaster" in source
    assert "map -> camera_init" in source
    assert "publish_corrected_streams" in source
    assert "mapping_cloud_world" in source
    assert "PublishCorrectedWorldCloud" in source
    assert "corrected_stream_every_n_scans" in source
    assert "point.z});" in source
    assert "g1_loop_closure" in fast_lio_launch
    assert "FAST_LIO_LOOP_POSE_GRAPH" in fast_lio_launch
    assert "FAST_LIO_LOOP_LIVE_CORRECTION" in fast_lio_launch
    assert "'enable_pose_graph': loop_pose_graph" in fast_lio_launch
    assert "'publish_live_correction': loop_live_correction" in fast_lio_launch
    assert "FAST_LIO_LOOP_LIVE_CORRECTION=true requires" in fast_lio_launch
    assert "'publish_corrected_streams': False" in fast_lio_launch
    # Loop closure is additive: it must not replace the proven dense input
    # used for live ground/obstacle classification.
    assert "'/cloud_registered_1'" in fast_lio_launch
    assert "'/Odometry_loc'" in fast_lio_launch
    assert "'/loop_closure/reset_grid'" not in fast_lio_launch

    loop_service = compose["services"]["loop_closure"]
    assert loop_service["profiles"] == ["loop_closure"]
    assert loop_service["restart"] == "no"
    assert "ros2 launch g1_loop_closure loop_closure.launch.py" in loop_service["command"][-1]
    assert "g1_loop_closure" in compose["services"]["builder_base"]["command"][-1]

    mapping_save = _read("tools/mapping/mapping_save.sh")
    assert "/loop_closure/export_optimized_map" in mapping_save
    assert "${SCENE}_loop_optimized.pcd" in mapping_save
    assert "not used for navigation" in mapping_save
    assert "using raw FAST-LIO navigation map" in mapping_save
    assert "FAST_LIO_RAW_PCD" in mapping_save
    assert "${SCENE}_fast_lio_raw.pcd" in mapping_save
    assert 'export_optimized_map_path "$LOOP_OPTIMIZED_PCD"' in mapping_save
    assert 'export_optimized_map_path "$PCD"' not in mapping_save
    assert mapping_save.index("ros2 service call /map_save") < (
        mapping_save.index(
            "ros2 service call /loop_closure/export_optimized_map"
        )
    )


def test_loop_closure_rviz_layers_are_preconfigured_but_phase_two_preview_is_off():
    mapping_rviz = _read("configs/g1_mapping_rviz2.rviz")
    navigation_rviz = _read("configs/g1_nav_loc_rviz2.rviz")
    mapping_script = _read("tools/host_side/mapping_rviz2.sh")
    navigation_script = _read("tools/host_side/g1_nav_loc_rviz2.sh")

    for rviz in (mapping_rviz, navigation_rviz):
        assert "/loop_closure/keyframe_path" in rviz
        assert "/loop_closure/markers" in rviz
        assert "/loop_closure/optimized_path" in rviz
        assert "/loop_closure/optimized_map_preview" in rviz
        assert "loop closure optimized map preview (phase 2)" in rviz
    assert "loop-closure keyframes/candidates" in mapping_script
    assert "loop-closure keyframes/candidates" in navigation_script


def test_mapping_scene_launcher_enables_save_grid_and_readiness_gate():
    source = _read("tools/mapping/start_mapping_scene.sh")

    assert "docker compose stop localization navigation" in source
    assert "FAST_LIO_MAPPING_MODE=true" in source
    assert "FAST_LIO_MAPPING_SAVE=true" in source
    assert 'FAST_LIO_MAP_FILE="/botbrain_ws/src/g1_pkg/maps/${scene}_fast_lio_raw.pcd"' in source
    assert "--overwrite" in source
    assert 'touch "$maps/.${scene}_mapping_started"' in source
    assert "IMU Initial Done" in source
    for topic in (
        "/cloud_registered_1",
        "/cloud_registered_body_1",
        "/accumulated_grid",
    ):
        assert topic in source
    assert "tf2_echo camera_init body" in source
    assert "tf2_echo map camera_init" not in source
    assert "FAST_LIO_LOOP_POSE_GRAPH=true" in source
    assert "FAST_LIO_LOOP_LIVE_CORRECTION=\"$live_loop_correction\"" in source
    assert "--live-loop-correction" in source
    assert "MAPPING READY" in source


def test_fast_lio_uses_one_mapping_profile_and_accepts_the_legacy_alias():
    launch = _read("botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py")
    compose = yaml.safe_load(_read("docker-compose.yaml"))

    fast_lio_env = compose["services"]["fast_lio"]["environment"]
    assert fast_lio_env["FAST_LIO_MAPPING_PROFILE"] == (
        "${FAST_LIO_MAPPING_PROFILE:-default}"
    )
    assert fast_lio_env["FAST_LIO_LOOP_POSE_GRAPH"] == (
        "${FAST_LIO_LOOP_POSE_GRAPH:-true}"
    )
    assert fast_lio_env["FAST_LIO_LOOP_LIVE_CORRECTION"] == (
        "${FAST_LIO_LOOP_LIVE_CORRECTION:-false}"
    )
    assert "'default': {}" in launch
    assert "requested_mapping_profile" in launch
    assert "requested_mapping_profile == 'corridor'" in launch
    assert "mapping_profile = 'default'" in launch
    assert "'preprocess.max_range': 20.0" not in launch
    assert "Unsupported FAST_LIO_MAPPING_PROFILE" in launch
    assert "FAST-LIO mapping profile:" in launch

    config = yaml.safe_load(
        _read("botbrain_ws/src/fast_lio/config/mid360.yaml")
    )["/**"]["ros__parameters"]
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    assert config["preprocess"]["max_range"] == 0.0
    assert 'declare_parameter<double>("preprocess.max_range", 0.0)' in source
    assert 'get_parameter_or<double>("preprocess.max_range", preprocess_max_range, 0.0)' in source
    assert "std::isfinite(preprocess_max_range)" in source
    assert source.index("p_imu->Process(Measures, kf, feats_undistort)") < (
        source.index("point_range_sq <= max_range_sq")
    )
    assert source.index("point_range_sq <= max_range_sq") < (
        source.index("downSizeFilterSurf.setInputCloud(feats_undistort)")
    )
    assert "[FAST_LIO_RANGE] max=%.1fm kept=%zu/%zu" in source


def test_rviz_presets_match_runtime_topics_and_keep_live_plus_bounded_history():
    mapping = yaml.safe_load(_read("configs/g1_mapping_rviz2.rviz"))
    navigation = yaml.safe_load(_read("configs/g1_nav_loc_rviz2.rviz"))

    red = "255; 0; 0"
    white = "255; 255; 255"
    dark_blue = "0; 0; 139"

    mapping_manager = mapping["Visualization Manager"]
    nav_manager = navigation["Visualization Manager"]
    assert mapping_manager["Global Options"]["Fixed Frame"] == "camera_init"
    assert nav_manager["Global Options"]["Fixed Frame"] == "map"

    mapping_displays = {
        display["Name"]: display for display in mapping_manager["Displays"]
    }
    nav_displays = {
        display["Name"]: display for display in nav_manager["Displays"]
    }
    mapping_live = mapping_displays["world scan (live)"]
    assert mapping_live["Topic"]["Value"] == "/cloud_registered_1"
    assert mapping_live["Topic"]["Depth"] == 1
    assert mapping_live["Topic"]["Reliability Policy"] == "Best Effort"
    assert float(mapping_live["Decay Time"]) == 0
    assert mapping_live["Enabled"] is True
    assert mapping_live["Color Transformer"] == "FlatColor"
    assert mapping_live["Color"] == red
    mapping_history = mapping_displays["world history (2 min)"]
    assert mapping_history["Topic"]["Value"] == "/cloud_registered_1"
    assert mapping_history["Topic"]["Depth"] == 1
    assert mapping_history["Topic"]["Reliability Policy"] == "Best Effort"
    assert 60 <= float(mapping_history["Decay Time"]) <= 600
    assert mapping_history["Enabled"] is True
    assert mapping_history["Value"] is True
    assert mapping_history["Color"] == red
    mapping_body = mapping_displays["body cloud (robot live scan)"]
    assert mapping_body["Topic"]["Value"] == "/cloud_registered_body_1"
    assert mapping_body["Topic"]["Depth"] == 1
    assert mapping_body["Topic"]["Reliability Policy"] == "Best Effort"
    assert mapping_body["Enabled"] is False
    assert mapping_body["Value"] is False
    assert mapping_body["Color"] == white
    assert nav_displays["Map"]["Topic"]["Value"] == "/map"
    assert nav_displays["Map"]["Update Topic"]["Value"] == "/map_updates"
    assert nav_displays["Path (Nav2 /g1_robot/plan)"]["Topic"]["Value"] == (
        "/g1_robot/plan"
    )
    assert nav_displays["Odometry"]["Topic"]["Value"] == "/g1_robot/nav_odom"
    nav_cloud = nav_displays["registered cloud (FAST-LIO)"]
    assert nav_cloud["Topic"]["Depth"] == 1
    assert nav_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert nav_cloud["Color"] == red
    body_cloud = nav_displays["body cloud (robot live scan)"]
    assert body_cloud["Topic"]["Value"] == "/cloud_registered_body_1"
    assert body_cloud["Topic"]["Depth"] == 1
    assert body_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert body_cloud["Color"] == white
    static_pcd = nav_displays["map (scans.pcd)"]
    assert static_pcd["Topic"]["Reliability Policy"] == "Reliable"
    assert static_pcd["Topic"]["Durability Policy"] == "Transient Local"
    assert float(static_pcd["Alpha"]) >= 0.80
    assert static_pcd["Color Transformer"] == "FlatColor"
    assert static_pcd["Color"] == dark_blue
    candidate_cloud = nav_displays["live/candidate scan preview (scan2map)"]
    assert candidate_cloud["Topic"]["Value"] == "/scan2map"
    assert candidate_cloud["Topic"]["Depth"] == 1
    assert candidate_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert candidate_cloud["Topic"]["Durability Policy"] == "Volatile"
    assert candidate_cloud["Enabled"] is True
    assert candidate_cloud["Value"] is True
    assert candidate_cloud["Color"] == red
    mapping_point_cloud_colors = {
        display["Color"]
        for display in mapping_manager["Displays"]
        if display.get("Class") == "rviz_default_plugins/PointCloud2"
    }
    navigation_point_cloud_colors = {
        display["Color"]
        for display in nav_manager["Displays"]
        if display.get("Class") == "rviz_default_plugins/PointCloud2"
    }
    assert mapping_point_cloud_colors <= {red, white}
    assert navigation_point_cloud_colors <= {red, white, dark_blue}
    assert not any(
        display.get("Class") == "rviz_default_plugins/RobotModel"
        for display in nav_manager["Displays"]
    )
    assert nav_displays["Global Costmap (optional)"]["Topic"]["Value"] == (
        "/g1_robot/global_costmap/costmap"
    )
    assert nav_displays["Local Costmap (optional)"]["Topic"]["Value"] == (
        "/g1_robot/local_costmap/costmap"
    )

    nav_tools = {
        tool["Class"]: tool for tool in nav_manager["Tools"]
    }
    assert nav_tools["rviz_default_plugins/SetInitialPose"]["Topic"]["Value"] == (
        "/initialpose"
    )
    assert nav_tools["rviz_default_plugins/SetGoal"]["Topic"]["Value"] == (
        "/g1_robot/goal_pose"
    )


def test_workstation_rviz_launchers_are_one_command_and_ros_setup_safe():
    mapping = _read("tools/host_side/mapping_rviz2.sh")
    navigation = _read("tools/host_side/g1_nav_loc_rviz2.sh")
    compact = _read("建图导航指令.md")

    for source, preset in (
        (mapping, "configs/g1_mapping_rviz2.rviz"),
        (navigation, "configs/g1_nav_loc_rviz2.rviz"),
    ):
        assert 'REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"' in source
        assert preset in source
        assert "set -euo pipefail" not in source
        assert "source /opt/ros/humble/setup.bash" in source
        assert "install ros-humble-rviz2" in source
        assert 'tcp/${G1_IP}:7448' in source
        assert 'tcp://${G1_IP}:7448' not in source
        assert "cannot reach Zenoh at ${G1_IP}:7448" in source
        assert "--no-daemon" in source
        assert "ros2 daemon stop" not in source
        assert "No manual Add is required" in source
        assert "RVIZ_RENDERING" in source
        assert 'exec rviz2 -d "$RVIZ_CFG"' in source

    assert "ros2 topic echo /cloud_registered_1 --once --field header" in mapping
    assert "ros2 topic echo /cloud_registered_body_1 --once --field header" in mapping
    assert "ros2 topic echo /pcd_map --once --field header" in navigation
    assert "requires the localization Compose service" in navigation

    assert "bash tools/host_side/mapping_rviz2.sh 192.168.100.3" in compact
    assert "bash tools/host_side/g1_nav_loc_rviz2.sh 192.168.100.3" in compact
    assert "cd ~/Workspace/botbrain_project" in compact
    assert (
        'bash tools/nav/select_map_scene.sh "$scene" '
        '--wait-ready --ready-timeout 300'
    ) in compact
    assert 'MAP_SCENE="$scene" LOCALIZATION_START_DELAY_SEC=0' in compact
    assert "--force-recreate --no-deps localization" in compact
    assert "不是日常启动服务的必经步骤" in compact
    assert "直接服务流程中" in compact
    assert (
        "docker compose --profile navigation up -d "
        "--force-recreate navigation"
    ) in compact
    assert "docker compose stop localization navigation" in compact
    assert "docker compose up -d zenoh bringup state_machine" in compact
    assert "FAST_LIO_START_DELAY_SEC=0" in compact
    assert "FAST_LIO_MAPPING_MODE=true" in compact
    assert "FAST_LIO_MAPPING_SAVE=true" in compact
    assert "docker compose up -d --force-recreate fast_lio" in compact
    assert "docker compose ps zenoh bringup state_machine fast_lio" in compact
    assert "docker compose logs -f fast_lio" in compact
    assert "FAST_LIO_MAPPING_PROFILE=default" in compact
    assert "FAST_LIO_LOOP_POSE_GRAPH=true" in compact
    assert "FAST_LIO_LOOP_LIVE_CORRECTION=false" in compact
    assert 'bash tools/mapping/start_mapping_scene.sh "$scene"' not in compact


def test_navigation_restart_explicitly_disables_mapping_outputs():
    source = _read("tools/nav/select_map_scene.sh")
    assert "FAST_LIO_MAPPING_PROFILE=default" in source
    assert "FAST_LIO_MAPPING_MODE=false" in source
    assert "FAST_LIO_MAPPING_SAVE=false" in source
    assert "FAST_LIO_MAP_FILE=" in source
    assert "FAST-LIO is still running in mapping mode" in source
    assert "RVIZ POINT CLOUD READY" in source
    assert "Navigation service must remain STOPPED" in source
    assert "navigation_preflight process below is only a readiness checker" in source
    assert "localization_map_loaded=true" in source
    assert "Starting localization Compose service" in source
    assert "Navigation remains stopped until localization is verified" in source


def test_localization_republishes_static_pcd_and_publishes_candidate_preview():
    source = _read("botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    assert "pcd_map_republish_timer_" in source
    assert "std::chrono::seconds(5)" in source
    assert "Published localization PCD for RViz" in source
    assert source.index("Published localization PCD for RViz") < source.index(
        "Prepared global FPFH scale"
    )
    preparation = source.index("bool GloabalLocalization::PrepareGlobalFeatureLevels")
    first_scale_publish = source.index(
        "pub_map_->publish(map_refresh);", preparation)
    prepared_scale = source.index("Prepared global FPFH scale", preparation)
    assert first_scale_publish < prepared_scale
    assert 'message.header.frame_id = "map"' in source
    assert "pub_scan2map_->publish(message)" in source
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()" in source
    assert "publish_scan_preview(pcd_scan, current_odom2map);" in source
    assert "publish_scan_preview(pcd_scan, candidate_odom2map);" in source
    preview = source.index("Visualization is diagnostic, not authorization")
    quality_rejection = source.index("if (!safe_initialization_step)")
    assert preview < quality_rejection


def test_g1_laserscan_uses_only_accepted_world_cloud_for_navigation_obstacles():
    launch = _read("botbrain_ws/src/g1_pkg/launch/pc2ls.launch.py")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml"))
    scan_params = params["pointcloud_to_laserscan_node"]["ros__parameters"]

    assert "('cloud_in', '/cloud_registered_1')" in launch
    assert "('cloud_in', '/cloud_registered_body_1')" not in launch
    assert "('scan', '/scan')" in launch
    assert "/livox/lidar" not in launch
    assert "f'{robot_name}/base_footprint'" in launch
    assert scan_params["target_frame"] == "g1_robot/base_footprint"
    assert 0.15 <= float(scan_params["min_height"]) <= 0.25
    assert 1.20 <= float(scan_params["max_height"]) <= 1.50
    assert float(scan_params["range_min"]) >= 0.40
    assert float(scan_params["range_max"]) >= 3.0
    assert float(scan_params["transform_tolerance"]) == 0.0
    assert scan_params["use_inf"] is True
    assert int(scan_params["queue_size"]) == 3
    assert "concurrency_level" not in scan_params

    compose_localization_launch = _read(
        "botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")
    generic_localization_launch = _read(
        "botbrain_ws/src/open3d_loc/launch/open3d_loc_g1.launch.py")
    assert "('scan', '/scan_loc')" in compose_localization_launch
    assert "('initialpose', 'initialpose_corrected')" in compose_localization_launch
    assert "remappings=[('scan', '/scan_loc')]" in generic_localization_launch
    assert "('/scan', '/scan_loc')" not in compose_localization_launch
    assert "remappings=[('/scan', '/scan_loc')]" not in generic_localization_launch


def test_icp_confidence_holds_the_last_measurement_until_it_is_stale():
    source = _read("botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    localization = source.split(
        "void GloabalLocalization::Localization()", 1
    )[1].split("void GloabalLocalization::StartLoc()", 1)[0]
    no_new_scan_branch = localization.split(
        "if (pcd_scan->IsEmpty() ||", 1
    )[1].split("double current_odom_stamp", 1)[0]

    assert "LocalizationConfidenceForPublish()" in source
    assert "age_sec > loc_fitness_stale_timeout_sec_" in source
    assert "UpdateLocalizationConfidence(" in localization
    assert "confidence_measurement_valid" in localization
    assert "loc_fitness_" not in no_new_scan_branch


def test_mapping_disables_unbounded_laser_map_publication():
    config = yaml.safe_load(
        _read("botbrain_ws/src/fast_lio/config/mid360.yaml"))
    foxglove = yaml.safe_load(_read(
        "botbrain_ws/src/bot_bringup/config/foxglove_bridge_params.yaml"))

    params = config["/**"]["ros__parameters"]
    bridge_params = foxglove["/**"]["ros__parameters"]
    assert params["publish"]["map_en"] is False
    assert params["pcd_save"]["pcd_save_en"] is False
    assert "/Laser_map_1" not in bridge_params["topic_whitelist"]
    assert "/localization_ready" in bridge_params["topic_whitelist"]
    assert "/cloud_registered_1" in bridge_params["topic_whitelist"]
    assert "/cloud_registered_body_1" in bridge_params["topic_whitelist"]
    launch = _read("botbrain_ws/src/bot_bringup/launch/foxglove_bridge.launch.py")
    assert "foxglove_bridge_params.yaml" in launch
    assert "'foxglove_bridge.yaml'" not in launch


def test_compact_map_review_keeps_live_fast_lio_topics_available():
    source = _read("建图导航指令.md")
    review = source.split("步骤 6：建图完成后查看效果", 1)[1].split("---", 1)[0]

    assert "docker compose up -d zenoh bringup state_machine" in review
    assert "FAST_LIO_MAPPING_MODE=false" in review
    assert "docker compose up -d --force-recreate fast_lio" in review
    assert (
        "docker compose --profile navigation up -d "
        "--force-recreate --no-deps localization"
    ) in review
    assert "但它不是本步骤的启动入口" in review
    assert "docker compose up fast_lio localization" not in source
    assert "/cloud_registered_1" in review
    assert "/pcd_map" in review
    assert "/path_1" in review
    assert "/scan" in review


def test_fast_lio_guard_recovers_early_or_stops_unconfirmed_outputs():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]
    guard = params["mapping"]

    recovery_points = guard["guard_recovery_min_effective_points"]
    strict_points = guard["guard_min_effective_points"]
    unconfirmed_frames = guard["guard_max_unconfirmed_odometry_frames"]
    max_rejections = guard["guard_max_consecutive_rejections"]
    assert recovery_points < strict_points
    assert unconfirmed_frames < max_rejections
    assert "effct_feat_num >= guard_recovery_min_effective_points" in source
    assert (
        "consecutive_guard_rejections <= "
        "guard_max_unconfirmed_odometry_frames" in source
    )
    rejection_limit = source.split(
        "if (consecutive_guard_rejections >= "
        "guard_max_consecutive_rejections)", 1
    )[1].split("// Bridge only a short transient", 1)[0]
    assert "quality remains below gate after %d" in rejection_limit
    assert "guard_failure_latched = true;" not in rejection_limit
    assert "guard_failure_latched = true;" in source


def test_fast_lio_guard_keeps_the_proven_strict_mapping_limits():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]["mapping"]

    assert params["guard_min_effective_points"] == 100
    assert params["guard_recovery_min_effective_points"] == 60
    assert params["guard_min_effective_ratio"] == 0.10
    assert params["guard_recovery_min_effective_ratio"] == 0.15
    assert params["guard_max_residual"] == 0.15
    assert params["guard_max_translation_correction"] == 0.25
    assert params["guard_max_rotation_correction_deg"] == 5.0
    assert params["guard_recovery_max_translation_correction"] == 0.75
    assert params["guard_recovery_max_rotation_correction_deg"] == 15.0
    assert params["guard_max_tilt_correction_deg"] == 2.0
    assert params["guard_recovery_max_tilt_correction_deg"] == 5.0
    # A recovery gate below the proven strict rotation envelope can reject a
    # high-support post-gap correction forever while IMU-only drift grows.
    assert (
        params["guard_recovery_max_tilt_correction_deg"]
        >= params["guard_max_rotation_correction_deg"]
    )
    # State-only recovery must still prohibit the much larger tilt permitted
    # by its yaw/total-rotation envelope.
    assert (
        params["guard_recovery_max_tilt_correction_deg"]
        < params["guard_recovery_max_rotation_correction_deg"]
    )


def test_unsafe_imu_propagation_stops_without_cross_time_rebase():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    propagation = source.split(
        "p_imu->Process(Measures, kf, feats_undistort);", 1
    )[1].split("/*** add the feature points to map kdtree ***/", 1)[0]

    unsafe = propagation.split(
        "if (!IsPlausibleState(state_point) || !covariance_after_imu.allFinite())",
        1,
    )[1].split("pos_lid =", 1)[0]
    assert "guard_failure_latched = true;" in unsafe
    assert "must be restarted" in unsafe
    assert "RebaseAfterGap" not in source
    assert "DropBufferedLidarData" not in source

    # FOV maintenance deletes ikd-tree boxes and therefore belongs only to the
    # strict accepted path, after both reject and state-only recovery returns.
    quality_gate = source.index("const bool quality_ok = timing_ok")
    reject_path = source.index("if (reject_update)", quality_gate)
    recovery_path = source.index("if (recovery_update)", reject_path)
    strict_fov_update = source.index("lasermap_fov_segment();", recovery_path)
    map_insert = source.index("map_incremental();", strict_fov_update)
    assert quality_gate < reject_path < recovery_path < strict_fov_update < map_insert


def test_fast_lio_propagates_but_rejects_lidar_updates_on_an_imu_timing_gap():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]

    imu_process = source.index("p_imu->Process(Measures, kf, feats_undistort)")
    quality_gate = source.index(
        "const bool quality_ok = timing_ok", imu_process)
    rejection = source.index("if (reject_update)", quality_gate)
    assert imu_process < quality_gate < rejection
    assert "this scan cannot update/write the map" in source
    assert "p_imu->RebaseAfterGap(Measures, state_before_gap)" not in source
    assert params["common"]["max_imu_gap"] == 0.02
    assert params["common"]["imu_queue_depth"] == 2000
    assert params["common"]["lidar_queue_depth"] == 100


def test_open3d_localization_pairs_latest_cloud_with_matching_odom_history():
    source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")

    assert "rclcpp::QoS(rclcpp::KeepLast(10)).reliable()" in source
    assert "rclcpp::QoS(rclcpp::KeepLast(20)).reliable()" in source
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()" in source
    assert '"Odometry_loc", odom_input_qos' in source
    assert '"cloud_registered_1", latest_cloud_qos' in source
    assert "Eigen::aligned_allocator<TimedOdomPose>" in source
    assert source.count("if (!SnapshotForScan(") == 2
    assert "unsigned int &manual_pose_generation" in source
    assert "manual_pose_generation = manual_pose_generation_.load();" in source
    assert "manual_pose_generation_.load() == iteration_manual_pose_generation" in source
    assert "Manual pose reset detected during initialization" in source
    assert source.count("icp_candidate_max_age_sec_ + candidate_processing_sec") == 2
    assert "candidate_time - loc_start" in source
    assert "KeepLast(100000)" not in source


def test_fast_lio_live_clouds_do_not_queue_stale_zenoh_frames():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")

    assert "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()" in source
    assert '"cloud_registered_1", latest_cloud_qos' in source
    assert '"cloud_registered_body_1", latest_cloud_qos' in source
    assert "bool published_unconfirmed_pose = false;" in source
    assert (
        "if (published_unconfirmed_pose && scan_pub_en && scan_body_pub_en)"
        in source
    )
    reject_block = source[
        source.index("if (reject_update)"):
        source.index("if (recovery_update)")
    ]
    recovery_block = source[
        source.index("if (recovery_update)"):
        source.index("state_point = updated_state;", source.index(
            "if (recovery_update)")) + len("state_point = updated_state;")
    ]
    assert "publish_frame_world" not in reject_block
    assert "publish_frame_world" not in recovery_block

    recovery_start = source.index("if (recovery_update)")
    recovery_state = source.index("state_point = updated_state;", recovery_start)
    strict_accept = source.index(
        "state_point = updated_state;", recovery_state + 1)
    accepted_odom = source.index(
        "publish_odometry(pubOdomAftMapped_, tf_broadcaster_);", strict_accept)
    accepted_world = source.index(
        "publish_frame_world(pubLaserCloudFull_);", accepted_odom)
    assert accepted_odom < accepted_world


def test_open3d_initializes_from_current_cloud_without_persisted_pose():
    source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    registration = _read(
        "botbrain_ws/src/open3d_loc/src/open3d_registration/"
        "open3d_registration.cpp")
    launch = _read("botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")
    config = yaml.safe_load(_read(
        "botbrain_ws/src/open3d_loc/config/loc_param_g1.yaml"))
    params = config["global_localization_node"]["ros__parameters"]

    assert 'declare_parameter<bool>("enable_global_initialization", false)' in source
    assert "PrepareFpfhCloud" in source
    assert "ComputeGlobalInitializationCandidate" in source
    assert "RegistrationRANSACBasedOnFeatureMatching(" in source
    assert "iteration_manual_pose_generation == 0" in source
    assert "global_initialization_confirmations_" in source
    assert "global_scan_window_size_" in source
    assert "global_min_ransac_fitness_" in source
    assert '"global_voxel_sizes"' in source
    assert "global_feature_levels_" in source
    assert "ParseInitialPosePriors" in source
    assert "ComputePriorInitializationCandidate" in source
    assert "Known startup poses exhausted; preparing whole-map FPFH fallback" in source
    assert source.index("ParseInitialPosePriors();") < source.index(
        "PrepareGlobalFeatureLevels();")
    assert "{1.0, 2.5}" in source
    assert "kPriorInitializationConfirmations = 2" in source
    assert "kPriorMaxConfirmationAttempts = 4" in source
    assert "retrying with the next scan" in source
    assert "mutual_filter" in source
    assert "Global RANSAC rejected" in source
    assert "Global registration seed=" in source
    assert "expire_pending_candidate(global_candidate_max_age_sec_)" in source
    assert "must not erase" in source
    assert '"localization_ready"' in source
    assert "Localization ready: verified map->odom is now available" in source
    assert source.index("if (!loc_initialized_.load())") < source.index(
        "// Do not publish an identity/seeded map transform")
    assert "RegistrationRANSACBasedOnFeatureMatching" in registration
    assert "global_ransac_max_iterations_" in source
    assert "'enable_global_initialization': True" in launch
    assert "f'{scene}_0', f'{scene}_2', f'{scene}_1'" in launch
    assert "priors = [('map_origin', 0.0, 0.0, 0.0)]" in launch
    assert "initial_pose_priors" in launch
    assert "initial_pose_prior_names" in launch
    assert "nav_waypoints.yaml" in launch
    assert "'global_voxel_sizes':         [0.25, 0.40, 0.60]" in launch
    assert "'global_initialization_confirmations': 3" in launch
    assert params["enable_global_initialization"] is False
    assert int(params["global_initialization_confirmations"]) >= 3
    assert int(params["global_scan_window_size"]) >= 10
    assert float(params["global_min_ransac_fitness"]) == 0.0
    assert "ransac_fitness <= global_min_ransac_fitness_" in source
    subscription = source.index(
        "sub_baselink2odom_ = this->create_subscription")
    assert subscription > source.index(
        "if (enable_global_initialization_ && !initial_pose_priors_.empty())")
    assert subscription < source.index("\n    StartLoc();", subscription)
    assert "global_retry_interval_sec_));" in source
    assert "'global_retry_interval_sec':  2.0" in launch
    assert "last_localization_pose" not in source
    assert "last_localization_pose" not in launch


def test_g1_localization_locks_corrected_map_to_planar_transform_in_g1_launch():
    source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    launch = _read("botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")

    assert 'declare_parameter<bool>("lock_map_odom_z", false)' in source
    assert 'declare_parameter<bool>("lock_map_odom_roll_pitch", false)' in source
    assert "Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())" in source
    assert "mat_initialpose_ = ConstrainMapOdom(mat_initialpose_);" in source
    assert "mat_odom2map_ = mat_initialpose_;" in source
    assert "candidate_odom2map = ConstrainMapOdom(" in source
    assert "effective_correction =" in source
    assert "new_odom2map = ConstrainMapOdom(" in source
    assert "requested_yaw - odom_baselink_yaw" in source
    assert "mat_odom2map_kalman_ = ConstrainMapOdom(" in source
    assert "mat_odom2map_(2, 3), 1);" in source
    assert "Rejecting initial pose: waiting for valid odometry" in source
    assert "initialpose_frame != \"map\"" in source
    assert "'lock_map_odom_z':          True" in launch
    assert "'lock_map_odom_roll_pitch': True" in launch
    assert "'map_odom_z':               IMU_HEIGHT" in launch
    assert "'publish_planar_base_tf':   True" in launch
    assert "'planar_base_height':       IMU_HEIGHT" in launch
    assert "body_to_base_footprint" not in launch
    assert 'planar_base_tf.header.frame_id = "odom";' in source
    assert "planar_base_tf.transform.translation.z = -planar_base_height_;" in source


def test_initialpose_relay_rejects_non_map_frames():
    source = _read("botbrain_ws/src/g1_pkg/scripts/initialpose_z_fix.py")

    assert "frame_id = msg.header.frame_id.lstrip('/')" in source
    assert "if frame_id != 'map':" in source
    assert "set the visualization Fixed Frame to 'map'" in source


def test_g1_mppi_period_matches_controller_and_preserves_horizon():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/nav2_params.yaml"))
    controller = params["/**/controller_server"]["ros__parameters"]
    frequency = float(controller["controller_frequency"])
    mppi = controller["FollowPath"]
    fallback = controller["FollowPathFallback"]

    assert float(mppi["model_dt"]) >= (1.0 / frequency) - 1e-12
    assert 1.5 <= float(mppi["model_dt"]) * int(mppi["time_steps"]) <= 1.8
    assert mppi["visualize"] is False
    assert controller["controller_plugins"] == [
        "FollowPath", "FollowPathFallback"]
    handoff_thresholds = {
        float(mppi[critic]["threshold_to_consider"])
        for critic in (
            "GoalCritic", "GoalAngleCritic", "PreferForwardCritic",
            "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
        )
    }
    assert handoff_thresholds == {0.45}
    assert float(mppi["PathAlignCritic"]["cost_weight"]) < 20.0
    assert int(mppi["PathAlignCritic"]["offset_from_furthest"]) < 20
    assert float(fallback["model_dt"]) >= (1.0 / frequency) - 1e-12
    assert math.isclose(
        float(fallback["model_dt"]) * int(fallback["time_steps"]),
        1.25,
        rel_tol=0.02,
    )
    assert int(fallback["batch_size"]) < int(mppi["batch_size"])
    assert float(fallback["vx_max"]) < float(mppi["vx_max"])
    assert math.isclose(float(fallback["vx_min"]), 0.0)
    assert float(fallback["PreferForwardCritic"]["cost_weight"]) > float(
        mppi["PreferForwardCritic"]["cost_weight"])
    assert "publish_optimal_trajectory" not in mppi
    for ignored_parameter in (
            "ax_max", "ax_min", "ay_max", "ay_min", "az_max", "vy_min"):
        assert ignored_parameter not in mppi
    assert "AckermannConstraints" not in mppi
    assert controller["odom_topic"] == "/<prefix>nav_odom"
    local_costmap = params["/**/local_costmap"]["local_costmap"]["ros__parameters"]
    assert local_costmap["global_frame"] == "<prefix>odom"
    bt_params = params["/**/bt_navigator"]["ros__parameters"]
    assert bt_params["odom_topic"] == "/<prefix>nav_odom"
    assert "default_bt_xml_filename" not in bt_params
    source = _read("botbrain_ws/src/g1_pkg/config/nav2_params.yaml")
    assert "obstacle_range:" not in source
    assert "raytrace_range:" not in source
    assert "observation_queue_length:" not in source
    assert source.count("obstacle_max_range:") == 2
    assert source.count("raytrace_max_range:") == 2
    global_costmap = params["/**/global_costmap"]["global_costmap"]["ros__parameters"]
    # Unknown cells must remain unknown so allow_unknown=false prevents map
    # boundary leaks. Dynamic obstacles remain available for initial planning
    # and persistent-failure replanning, but the BT no longer invalidates the
    # active path on each transient global scan update.
    assert global_costmap["track_unknown_space"] is True
    assert global_costmap["plugins"] == [
        "static_layer", "obstacle_layer", "denoise_layer", "inflation_layer"]
    global_scan = global_costmap["obstacle_layer"]["scan"]
    assert global_scan["topic"] == "/scan"
    assert global_scan["data_type"] == "LaserScan"
    assert global_scan["obstacle_min_range"] > global_scan["raytrace_min_range"]
    assert global_scan["raytrace_max_range"] >= global_scan["obstacle_max_range"]
    assert global_scan["inf_is_valid"] is True
    assert 0.5 <= float(global_scan["expected_update_rate"]) <= 1.0
    assert global_costmap["denoise_layer"]["minimal_group_size"] == 2
    assert math.isclose(
        float(global_costmap["inflation_layer"]["cost_scaling_factor"]),
        15.0,
        abs_tol=1e-9,
    )
    global_footprint = ast.literal_eval(global_costmap["footprint"])
    global_padding = float(global_costmap["footprint_padding"])
    assert math.isclose(global_padding, 0.02, abs_tol=1e-9)
    assert math.isclose(
        float(global_costmap["inflation_layer"]["inflation_radius"]),
        0.20,
        abs_tol=1e-9,
    )
    assert local_costmap["plugins"] == [
        "obstacle_layer", "denoise_layer", "inflation_layer"]
    assert float(local_costmap["width"]) >= 8.0
    assert float(local_costmap["height"]) >= 8.0
    local_scan = local_costmap["obstacle_layer"]["scan"]
    assert local_scan["topic"] == "/scan"
    assert local_scan["data_type"] == "LaserScan"
    assert local_scan["obstacle_min_range"] > local_scan["raytrace_min_range"]
    assert local_scan["raytrace_max_range"] >= local_scan["obstacle_max_range"]
    assert local_scan["inf_is_valid"] is True
    assert 0.5 <= float(local_scan["expected_update_rate"]) <= 1.0
    assert local_costmap["denoise_layer"]["minimal_group_size"] == 2
    local_footprint = ast.literal_eval(local_costmap["footprint"])
    local_padding = float(local_costmap["footprint_padding"])
    assert math.isclose(local_padding, 0.03, abs_tol=1e-9)
    assert math.isclose(
        float(local_costmap["inflation_layer"]["inflation_radius"]),
        0.32,
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(local_costmap["inflation_layer"]["cost_scaling_factor"]),
        12.0,
        abs_tol=1e-9,
    )
    assert global_footprint == local_footprint
    assert global_padding < local_padding


def test_waypoints_are_planar_and_never_reanchor_localization():
    recorder = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_recorder.py")
    navigator = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py")
    compact_runbook = _read("建图导航指令.md")
    waypoint_database = yaml.safe_load(_read(
        "botbrain_ws/src/bot_navigation/nav_waypoints.yaml"))
    waypoints = {
        name: waypoint
        for scene_data in waypoint_database["scenes"].values()
        for name, waypoint in scene_data["waypoints"].items()
    }

    assert "choices=['record', 'list', 'delete']" in recorder
    assert 'delete ug_old --scene "$scene"' in compact_runbook
    assert "def _yaw_quaternion(" in recorder
    assert "def _planar_quaternion(" in navigator
    assert "PoseWithCovarianceStamped" not in navigator
    assert "initialpose_pub" not in navigator
    assert "goal.pose.pose.position.z = 0.0" in navigator
    assert "wait_for_server(timeout_sec=60.0)" in navigator
    assert "No fresh {scan_topic} received" in navigator
    assert "cancel_goal_async" in navigator
    assert "SignalHandlerOptions.NO" in navigator
    assert "status != GoalStatus.STATUS_SUCCEEDED" in navigator
    assert "feedback.number_of_recoveries" in navigator
    assert "feedback.navigation_time" in navigator
    assert "yaw_error=" in navigator
    assert "if rclpy.ok():" in navigator
    assert "sys.exit(1)" in navigator
    assert "Stopping waypoint sequence after navigation failure." in navigator
    assert "Waypoints file:" in navigator
    assert "live_map_scene" in navigator
    assert "goal_grid_occupancy" in navigator
    assert 'sending it to Nav2 to resolve against the live costmaps' in navigator
    assert "tf_buffer.lookup_transform(" in navigator
    assert "validating success with the last" in navigator
    assert waypoint_database["format_version"] == 2
    for waypoint in waypoints.values():
        assert float(waypoint["z"]) == 0.0
        assert float(waypoint["qx"]) == 0.0
        assert float(waypoint["qy"]) == 0.0
        norm = math.hypot(float(waypoint["qz"]), float(waypoint["qw"]))
        assert math.isclose(norm, 1.0, abs_tol=2e-6)


def test_navigation_uses_preflight_and_coherent_nav_odometry():
    cmake = _read("botbrain_ws/src/bot_navigation/CMakeLists.txt")
    launch = _read("botbrain_ws/src/bot_navigation/launch/nav_utils.launch.py")
    relay = _read("botbrain_ws/src/bot_navigation/scripts/nav_odom_relay.py")
    preflight = _read(
        "botbrain_ws/src/bot_navigation/scripts/navigation_preflight.py")
    monitor = _read(
        "botbrain_ws/src/bot_navigation/scripts/localization_monitor.py")
    nav_utils_launch = _read(
        "botbrain_ws/src/bot_navigation/launch/nav_utils.launch.py")
    robot_read = _read("botbrain_ws/src/g1_pkg/scripts/g1_read.py")
    robot_interface_launch = _read(
        "botbrain_ws/src/g1_pkg/launch/robot_interface.launch.py")
    compose = yaml.safe_load(_read("docker-compose.yaml"))

    assert "scripts/nav_odom_relay.py" in cmake
    assert "scripts/navigation_preflight.py" in cmake
    assert "scripts/costmap_center_check.py" in cmake
    localization_launch = _read(
        "botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")
    assert "executable='nav_odom_relay.py'" not in launch
    assert "executable='nav_odom_relay.py'" in localization_launch
    assert "'pose_topic': '/Odometry_loc'" in localization_launch
    assert "'derive_twist_from_pose': False" in localization_launch
    assert "nav_odom" in relay
    assert "output.pose.pose.position.z = 0.0" in relay
    assert "output.twist = copy.deepcopy(self._last_twist)" in relay
    assert "derive_twist_from_pose" in relay
    assert "declare_parameter('derive_twist_from_pose', False)" in relay
    assert "def _derive_pose_twist(" in relay
    assert "fast_lio_pose" in relay
    assert "max_derived_linear_speed" in relay
    assert "twist_stamp_age" in relay
    assert "Navigation preflight passed" in preflight
    assert "pose_odom_topic" in preflight
    assert "allow_pose_derived_twist" in preflight
    assert "declare_parameter('allow_pose_derived_twist', False)" in preflight
    assert "max_derived_linear_speed" in preflight
    assert "max_derived_angular_speed" in preflight
    assert "twist_source={twist_source}" in preflight
    assert "unitree_twist={unitree_twist_ok}" in preflight
    assert "declare_parameter('twist_odom_topic', '/g1_robot/nav_odom')" in preflight
    assert "max_twist_variance" in preflight
    assert "declare_parameter('min_confidence', 0.50)" in preflight
    assert "self._confidence > self.min_confidence" in preflight
    assert "qos_profile_sensor_data" in robot_read
    assert "SportModeState, '/lf/odommodestate'" in robot_read
    assert "world_velocity_to_body" in robot_read
    assert "declare_parameter('velocity_frame', 'odom')" in robot_read
    assert "self.velocity_frame == 'odom'" in robot_read
    assert "UNITREE_VELOCITY_FRAME" in robot_interface_launch
    assert "'velocity_frame': velocity_frame" in robot_interface_launch
    assert compose["services"]["bringup"]["environment"][
        "UNITREE_VELOCITY_FRAME"] == "${UNITREE_VELOCITY_FRAME:-odom}"
    assert "min_confidence:=0.50" in compose["services"]["navigation"][
        "command"][-1]
    robot_write = _read("botbrain_ws/src/g1_pkg/src/g1_write.cpp")
    assert "locomotion Move failed" in robot_write
    assert "const int result = g1_driver_->move" in robot_write
    localization_source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    icp_log_prefix = localization_source.split('"ICP: accepted=%s', 1)[0][-250:]
    assert "RCLCPP_INFO(" in icp_log_prefix
    assert "RCLCPP_INFO_THROTTLE" not in icp_log_prefix
    assert "max_base_tilt_deg" in preflight
    assert "twist_odom_topic" in preflight
    assert "_twist_odom_is_fresh" in preflight
    assert "time.monotonic()" in preflight
    center_check = _read(
        "botbrain_ws/src/bot_navigation/scripts/costmap_center_check.py")
    assert "Time.from_msg(msg.header.stamp)" in center_check
    assert "time.monotonic()" in center_check
    assert "math.cos(yaw) * half_width" in center_check
    assert "PoseWithCovarianceStamped" not in monitor
    assert "auto_anchor" not in monitor
    assert "Odometry, self.nav_odom_topic" in monitor
    assert "LaserScan, self.scan_topic" in monitor
    assert "qos_profile_sensor_data" in monitor
    assert "scan_healthy" in monitor
    assert "_stamp_is_fresh" in monitor
    assert "waiting_for_initial_inputs" in monitor
    assert "confidence_timeout_sec" in monitor
    assert "cancel_after_sec" in monitor
    assert "fault_duration >= self.cancel_after" in monitor
    assert "_cancel_requested_for_episode" in monitor
    assert "remove_pending_request" in monitor
    assert "_confidence_stream_unhealthy" in monitor
    assert "max_twist_variance" in monitor
    assert "not math.isfinite(confidence)" in monitor
    assert "low_duration >= self.low_confidence_duration" in monitor
    assert "Navigation cancel requested due to" in monitor
    assert "Trigger.Request()" in monitor
    assert "_poll_cancel_result" in monitor
    assert "Navigation safety stop engaged" in monitor
    assert "executable='localization_monitor.py'" in nav_utils_launch
    assert "'auto_cancel': False" in nav_utils_launch
    assert "'low_confidence_duration_sec': 12.0" in nav_utils_launch
    assert "'publish_safety_stop': True" in nav_utils_launch
    assert "'scan_topic': '/scan'" in nav_utils_launch
    assert "'startup_grace_sec': 2.0" in nav_utils_launch
    assert "'confidence_timeout_sec': 2.0" in nav_utils_launch
    assert "'cancel_after_sec': 3.0" in nav_utils_launch


def test_g1_navigation_avoids_replanning_timeouts_and_slippery_recoveries():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/nav2_params.yaml"))
    planner = params["/**/planner_server"]["ros__parameters"]["GridBased"]
    controller = params["/**/controller_server"]["ros__parameters"]
    bt = params["/**/bt_navigator"]["ros__parameters"]
    tree = _read(
        "botbrain_ws/src/bot_navigation/behavior_trees/g1_navigate_to_pose.xml")
    launch = _read("botbrain_ws/src/bot_navigation/launch/nav2.launch.py")
    cmake = _read("botbrain_ws/src/bot_navigation/CMakeLists.txt")
    package = _read("botbrain_ws/src/bot_navigation/package.xml")
    continuity = _read(
        "botbrain_ws/src/bot_navigation/scripts/cmd_vel_continuity.py")

    assert 0.0 <= float(planner["tolerance"]) <= 0.05
    assert planner["allow_unknown"] is False
    assert 1.0 <= float(planner["cost_travel_multiplier"]) <= 5.0
    goal_checker = controller["general_goal_checker"]
    assert goal_checker["stateful"] is False
    assert 0.08 <= float(goal_checker["xy_goal_tolerance"]) <= 0.12
    assert 0.15 <= float(goal_checker["yaw_goal_tolerance"]) <= 0.20
    assert float(controller["progress_checker"]["required_movement_radius"]) <= 0.15
    assert controller["progress_checker"]["plugin"] == (
        "nav2_controller::PoseProgressChecker")
    assert math.isclose(
        float(controller["progress_checker"]["required_movement_angle"]),
        0.20,
        abs_tol=1e-9,
    )
    assert planner["use_final_approach_orientation"] is False
    assert math.isclose(float(controller["failure_tolerance"]), 1.0)
    assert float(controller["min_y_velocity_threshold"]) <= 0.01
    assert int(controller["FollowPath"]["iteration_count"]) == 1
    assert int(controller["FollowPath"]["CostCritic"][
        "trajectory_point_step"]) == 1
    assert math.isclose(float(controller["FollowPath"]["vx_max"]), 0.50)
    assert controller["controller_plugins"] == [
        "FollowPath", "FollowPathFallback"]
    velocity_smoother = params["/**/velocity_smoother"]["ros__parameters"]
    assert float(velocity_smoother["smoothing_frequency"]) >= 20.0
    assert velocity_smoother["feedback"] == "OPEN_LOOP"
    assert velocity_smoother["max_velocity"] == [0.50, 0.15, 0.80]
    assert velocity_smoother["min_velocity"] == [-0.30, -0.15, -0.80]
    assert float(velocity_smoother["velocity_timeout"]) <= 0.30
    constants = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in ast.parse(continuity).body
        if isinstance(node, ast.Assign) and
        len(node.targets) == 1 and
        isinstance(node.targets[0], ast.Name) and
        node.targets[0].id in {
            "COMMAND_HOLD_SEC", "ZERO_COMMAND_GRACE_SEC", "EMA_ALPHA"}
    }
    assert 0.0 < constants["COMMAND_HOLD_SEC"] < float(
        velocity_smoother["velocity_timeout"])
    assert 0.0 < constants["ZERO_COMMAND_GRACE_SEC"] < constants[
        "COMMAND_HOLD_SEC"]
    assert 0.0 < constants["EMA_ALPHA"] < 1.0
    assert "elif _is_zero(command):" in continuity
    assert "zero_duration >= ZERO_COMMAND_GRACE_SEC" in continuity
    assert "zero_grace_expired" in continuity
    assert "if input_timed_out or zero_grace_expired:" in continuity
    assert "self._filtered = Twist()" in continuity
    assert "Rejected non-finite navigation velocity command" in continuity
    assert "OSCILLATION_MIN_REVERSALS" in continuity
    assert "Navigation command oscillation detected" in continuity
    assert "if self._oscillation_active else EMA_ALPHA" in continuity
    assert int(bt["bt_loop_duration"]) >= 20
    assert int(bt["default_server_timeout"]) >= 200
    assert int(bt["wait_for_service_timeout"]) >= 1000
    assert bt["default_nav_to_pose_bt_xml"] == "<nav_to_pose_bt_xml>"
    assert '<RateController hz="2.0"' in tree
    assert tree.count("<IsPathValid") == 2
    assert '<GoalUpdatedController name="PlanOnGoalUpdate">' in tree
    assert "ConfirmPathInvalidity" in tree
    assert '<Delay delay_msec="750"' in tree
    assert "RefreshLocalCostmapAfterFollowPathFailure" in tree
    assert tree.count("<ComputePathToPose") == 3
    tree_root = ET.fromstring(tree)
    follow_recovery = next(
        node for node in tree_root.iter("RecoveryNode")
        if node.attrib.get("name") == "FollowPathRecovery"
    )
    follow_children = list(follow_recovery)
    assert [node.tag for node in follow_children] == [
        "RecoveryNode", "Sequence"]
    local_degraded = follow_children[0]
    assert local_degraded.attrib["name"] == "LocalControllerDegradedRecovery"
    degraded_children = list(local_degraded)
    assert [node.tag for node in degraded_children] == [
        "FollowPath", "ForceSuccess"]
    timeout = list(degraded_children[1])[0]
    assert timeout.tag == "Timeout"
    assert int(timeout.attrib["msec"]) == 1000
    fallback_follow = list(timeout)[0]
    assert fallback_follow.attrib["controller_id"] == "FollowPathFallback"
    recovery_steps = list(follow_children[1])
    assert [node.tag for node in recovery_steps] == [
        "ClearEntireCostmap", "Wait"]
    assert math.isclose(float(recovery_steps[-1].attrib["wait_duration"]), 0.5)
    assert 'number_of_retries="4"' in tree
    assert "goal_checker_id=\"general_goal_checker\"" in tree
    assert "<Spin" not in tree
    assert "<BackUp" not in tree
    assert "<nav_to_pose_bt_xml>" in launch
    assert '("cmd_vel", "cmd_vel_nav_raw")' in launch
    assert '("cmd_vel", "cmd_vel_nav_filtered")' in launch
    assert '("cmd_vel_smoothed", "cmd_vel_nav")' in launch
    assert 'executable="cmd_vel_continuity.py"' in launch
    assert '"velocity_smoother",' in launch
    assert "\n        velocity_smoother," in launch
    assert "nav2_velocity_smoother" in package
    assert "scripts/cmd_vel_continuity.py" in cmake
    assert "behavior_trees" in cmake
    assert "file(REMOVE" in cmake
    assert "goal_pose_bridge.py" in cmake
    waypoint_navigator = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py")
    assert "--success-distance-limit', type=float, default=0.15" in (
        waypoint_navigator)
    twist_mux = yaml.safe_load(_read(
        "botbrain_ws/src/bot_bringup/config/twist_mux.yaml"))
    mux_params = twist_mux["/**"]["ros__parameters"]
    navigation_timeout = float(mux_params["topics"]["navigation"]["timeout"])
    safety_topic = mux_params["topics"]["navigation_safety"]
    assert 0.40 <= navigation_timeout < 0.50
    assert float(controller["costmap_update_timeout"]) < navigation_timeout
    assert navigation_timeout < float(mux_params["twist_watchdog_timeout"])
    assert safety_topic["topic"] == "cmd_vel_nav_safety"
    assert float(safety_topic["timeout"]) >= 0.5
    assert int(safety_topic["priority"]) > int(
        mux_params["topics"]["navigation"]["priority"])


def test_unitree_world_velocity_is_rotated_into_body_frame():
    source = _read("botbrain_ws/src/g1_pkg/scripts/g1_read.py")
    module = ast.parse(source)
    function = next(
        node for node in module.body
        if isinstance(node, ast.FunctionDef) and
        node.name == "world_velocity_to_body"
    )
    namespace = {"cos": math.cos, "sin": math.sin}
    exec(compile(ast.Module(body=[function], type_ignores=[]),
                 "g1_read.py", "exec"), namespace)
    convert = namespace["world_velocity_to_body"]

    assert convert(1.0, 0.0, 0.0) == (1.0, 0.0)
    vx, vy = convert(0.0, 1.0, math.pi / 2.0)
    assert math.isclose(vx, 1.0, abs_tol=1e-9)
    assert math.isclose(vy, 0.0, abs_tol=1e-9)
    vx, vy = convert(1.0, 0.0, math.pi)
    assert math.isclose(vx, -1.0, abs_tol=1e-9)
    assert math.isclose(vy, 0.0, abs_tol=1e-9)


def test_state_machine_does_not_manage_nav2_utils_as_lifecycle_node():
    config = json.loads(_read(
        "botbrain_ws/src/bot_state_machine/config/navigation.json"))

    assert all(node["name"] != "nav2_utils" for node in config["nodes"])
    graph_source = _read(
        "botbrain_ws/src/bot_state_machine/src/graph_node.cpp")
    assert 'if (doc["nodes"].empty())' in graph_source


def test_pcd_service_and_exit_share_the_validated_map_save_path():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")

    assert "bool save_to_pcd(std::string &message)" in source
    assert "const std::size_t point_count = pcl_wait_save->size();" in source
    assert "pcd_writer.writeBinary(temporary_path, *pcl_wait_save)" in source
    assert "res->success = save_to_pcd(res->message);" in source
    assert "*pcl_wait_save += *feats_down_world;" in source
    assert "last_saved_point_count = point_count;" in source
    assert "pcl_wait_save->size() != last_saved_point_count" in source
    assert "ikdtree.stop_thread();" not in source
    assert "signal(SIGINT" not in source
    assert "signal(SIGTERM" not in source
