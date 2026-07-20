import ast
import re
import json
import math
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[4]


def _read(relative_path):
    return (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")


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
        "${LOCALIZATION_START_DELAY_SEC:-30}"
    )
    assert compose["services"]["fast_lio"]["environment"][
        "FAST_LIO_START_DELAY_SEC"
    ] == "${FAST_LIO_START_DELAY_SEC:-25}"
    localization_command = compose["services"]["localization"]["command"][-1]
    assert localization_command.count("MAP_SCENE") == 1
    assert 'map_scene:="$${MAP_SCENE}"' in localization_command
    assert "map_file:=" not in localization_command
    assert "grid_map_file:=" not in localization_command
    assert compose["services"]["navigation"]["restart"] == "no"
    assert "navigation_preflight.py" in compose["services"]["navigation"]["command"][-1]


def test_map_scene_selector_recreates_and_verifies_localization_container():
    selector = _read("tools/nav/select_map_scene.sh")
    compact_runbook = _read("建图导航指令.md")

    assert 'MAP_SCENE="$scene" LOCALIZATION_START_DELAY_SEC=0' in selector
    assert "FAST_LIO_START_DELAY_SEC=0" in selector
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
    assert "Foxglove connection was preserved" in selector
    assert "--restart-fast-lio" in selector
    assert "--wait-ready" in selector
    assert "--ready-timeout" in selector
    assert "navigation_preflight.py" in selector
    assert "localization is ready for navigation" in selector
    assert "wait_for_consecutive_icp_accepts" in selector
    assert "3 consecutive accepted ICP updates" in selector
    assert "ICP: accepted=true" in selector
    assert "Rejecting ICP|Skipping ICP|ICP: accepted=false" in selector
    assert "fitness_value <= 0.50" in selector
    assert "rmse_value > 0.30" in selector
    assert "ICP accepted: %s %s" in selector
    assert 'docker logs --since "$localization_started_at"' in selector
    assert 'current_container_id" != "$localization_container_id' in selector
    assert "Latest ICP decisions from the current localization container" in selector
    assert "verify_navigation_topic_publishers" in selector
    assert "print_unitree_twist_diagnostics" in selector
    assert "ros2 lifecycle get /g1_robot/robot_read_node" in selector
    assert "ros2 topic hz /lf/odommodestate" in selector
    assert "ros2 topic hz /g1_robot/odom" in selector
    assert "/Odometry_loc" in selector
    assert "/cloud_registered_body_1" in selector
    assert "navigation topics must each have exactly 1 publisher" in selector
    assert "flock -n 9" in selector
    assert 'grep -Fq "IMU Initial Done"' in selector
    assert "latest_timing" in selector
    assert "timing_count" in selector
    assert "ros2 topic echo /Odometry_loc --once" in selector
    assert "ros2 topic echo /cloud_registered_1 --once" in selector
    assert "docker compose ps -aq --all localization" in selector
    assert "old map publishers are still visible" in selector
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
    assert readiness_body.index("wait_for_consecutive_icp_accepts") < (
        readiness_body.index("verify_navigation_topic_publishers")
    )


def test_fast_lio_service_execs_launch_for_graceful_map_save_shutdown():
    source = _read("docker-compose.yaml")
    compose = yaml.safe_load(source)

    assert "exec ros2 launch --noninteractive g1_pkg fast_lio.launch.py" in source
    assert "stop_signal: SIGINT" in source
    assert "stop_grace_period: 180s" in source
    assert compose["services"]["localization"]["profiles"] == ["navigation"]
    assert compose["services"]["navigation"]["profiles"] == ["navigation"]


def test_fast_lio_launch_allows_large_pcd_flush_before_signal_escalation():
    source = _read("botbrain_ws/src/g1_pkg/launch/fast_lio.launch.py")

    assert "sigterm_timeout='150'" in source
    assert "sigkill_timeout='20'" in source
    assert "'--rate',           '2.0'" in source
    assert "'--debug-clouds'" not in source


def test_g1_laserscan_filters_body_cloud_for_navigation_obstacles():
    launch = _read("botbrain_ws/src/g1_pkg/launch/pc2ls.launch.py")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml"))
    scan_params = params["pointcloud_to_laserscan_node"]["ros__parameters"]

    assert "('cloud_in', '/cloud_registered_body_1')" in launch
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
    assert int(scan_params["queue_size"]) >= 5
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


def test_compact_map_review_keeps_live_fast_lio_topics_available():
    source = _read("建图导航指令.md")
    review = source.split("步骤 6：建图完成后查看效果", 1)[1].split("---", 1)[0]

    assert "docker compose up -d bringup state_machine" in review
    assert (
        'bash tools/nav/select_map_scene.sh "$scene" --restart-fast-lio --wait-ready'
        in review
    )
    assert "docker compose up fast_lio localization" not in source
    assert "/cloud_registered_1" in review
    assert "/cloud_registered_body_1" in review
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
    assert "output latched unhealthy after %d consecutive" in source
    assert "guard_failure_latched = true;" in source


def test_fast_lio_rebases_before_processing_an_imu_timing_gap():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    imu_source = _read("botbrain_ws/src/fast_lio/src/IMU_Processing.hpp")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]

    timing_guard = source.index(
        "if (!timing_ok)", source.index("void timer_callback()"))
    imu_process = source.index("p_imu->Process(Measures, kf, feats_undistort)")
    assert timing_guard < imu_process
    assert "dropping this scan before IMU propagation" in source
    assert "p_imu->RebaseAfterGap(Measures, state_before_gap)" in source
    assert "DropBufferedLidarData();" in source
    assert "suppress_unconfirmed_odometry_after_timing_gap = true;" in source
    assert "!suppress_unconfirmed_odometry_after_timing_gap" in source
    assert "last_lidar_end_time_ = meas.lidar_end_time;" in imu_source
    assert "last_imu_ = latest_imu;" in imu_source
    assert "imu_lag_at_scan_end > 0.10" in imu_source
    assert "rejected an invalid IMU rebase baseline" in source
    assert params["common"]["imu_queue_depth"] <= 400
    assert params["common"]["lidar_queue_depth"] <= 20


def test_open3d_localization_pairs_latest_cloud_with_matching_odom_history():
    source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")

    assert "rclcpp::QoS(rclcpp::KeepLast(10)).reliable()" in source
    assert "rclcpp::QoS(rclcpp::KeepLast(20)).reliable()" in source
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable()" in source
    assert '"Odometry_loc", odom_input_qos' in source
    assert '"cloud_registered_1", latest_cloud_qos' in source
    assert "Eigen::aligned_allocator<TimedOdomPose>" in source
    assert source.count("if (!SnapshotForScan(") == 2
    assert "unsigned int &manual_pose_generation" in source
    assert "manual_pose_generation = manual_pose_generation_.load();" in source
    assert "manual_pose_generation_.load() == iteration_manual_pose_generation" in source
    assert "Manual pose reset detected during initialization" in source
    assert "KeepLast(100000)" not in source


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
    assert "'global_voxel_sizes':         [0.25, 0.40, 0.60]" in launch
    assert "'global_initialization_confirmations': 3" in launch
    assert params["enable_global_initialization"] is False
    assert int(params["global_initialization_confirmations"]) >= 3
    assert int(params["global_scan_window_size"]) >= 10
    assert float(params["global_min_ransac_fitness"]) == 0.0
    assert "ransac_fitness <= global_min_ransac_fitness_" in source
    subscription = source.index(
        "sub_baselink2odom_ = this->create_subscription")
    assert subscription > source.index("if (enable_global_initialization_)")
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
    assert "set Foxglove Fixed Frame to 'map'" in source


def test_g1_mppi_period_matches_controller_and_preserves_horizon():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/nav2_params.yaml"))
    controller = params["/**/controller_server"]["ros__parameters"]
    frequency = float(controller["controller_frequency"])
    mppi = controller["FollowPath"]

    assert float(mppi["model_dt"]) >= (1.0 / frequency) - 1e-12
    assert 2.0 <= float(mppi["model_dt"]) * int(mppi["time_steps"]) <= 3.0
    assert mppi["visualize"] is False
    assert mppi["publish_optimal_trajectory"] is True
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
    assert float(global_costmap["inflation_layer"]["cost_scaling_factor"]) >= 15.0
    global_footprint = ast.literal_eval(global_costmap["footprint"])
    global_padding = float(global_costmap["footprint_padding"])
    global_circumscribed_radius = max(
        math.hypot(abs(x) + global_padding, abs(y) + global_padding)
        for x, y in global_footprint)
    assert global_costmap["inflation_layer"]["inflation_radius"] >= global_circumscribed_radius
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
    local_circumscribed_radius = max(
        math.hypot(abs(x) + local_padding, abs(y) + local_padding)
        for x, y in local_footprint)
    assert local_costmap["inflation_layer"]["inflation_radius"] >= local_circumscribed_radius


def test_waypoints_are_planar_and_never_reanchor_localization():
    recorder = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_recorder.py")
    navigator = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py")
    compact_runbook = _read("建图导航指令.md")
    waypoints = yaml.safe_load(_read(
        "botbrain_ws/src/bot_navigation/nav_waypoints.yaml"))["waypoints"]

    assert "choices=['record', 'list', 'delete']" in recorder
    assert "waypoint_recorder.py delete floor1_old" in compact_runbook
    assert "def _yaw_quaternion(" in recorder
    assert "def _planar_quaternion(" in navigator
    assert "PoseWithCovarianceStamped" not in navigator
    assert "initialpose_pub" not in navigator
    assert "goal.pose.pose.position.z = 0.0" in navigator
    assert "wait_for_server(timeout_sec=60.0)" in navigator
    assert "No fresh {scan_topic} received" in navigator
    assert "cancel_goal_async" not in navigator
    assert "status != GoalStatus.STATUS_SUCCEEDED" in navigator
    assert "feedback.number_of_recoveries" in navigator
    assert "feedback.navigation_time" in navigator
    assert "yaw_error=" in navigator
    assert "if rclpy.ok():" in navigator
    assert "sys.exit(1)" in navigator
    assert "Stopping waypoint sequence after navigation failure." in navigator
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
    robot_read = _read("botbrain_ws/src/g1_pkg/scripts/g1_read.py")

    assert "scripts/nav_odom_relay.py" in cmake
    assert "scripts/navigation_preflight.py" in cmake
    assert "scripts/costmap_center_check.py" in cmake
    assert "executable='nav_odom_relay.py'" in launch
    assert "pose_topic': '/Odometry_loc'" in launch
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
    assert "qos_profile_sensor_data" in robot_read
    assert "SportModeState, '/lf/odommodestate'" in robot_read
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

    assert float(planner["tolerance"]) <= 0.10
    goal_checker = controller["general_goal_checker"]
    assert goal_checker["stateful"] is True
    assert 0.20 <= float(goal_checker["xy_goal_tolerance"]) <= 0.30
    assert 0.30 <= float(goal_checker["yaw_goal_tolerance"]) <= 0.40
    assert float(controller["progress_checker"]["required_movement_radius"]) <= 0.15
    assert float(controller["min_y_velocity_threshold"]) <= 0.01
    assert int(controller["FollowPath"]["iteration_count"]) == 1
    assert int(bt["bt_loop_duration"]) >= 20
    assert int(bt["default_server_timeout"]) >= 200
    assert int(bt["wait_for_service_timeout"]) >= 1000
    assert bt["default_nav_to_pose_bt_xml"] == "<nav_to_pose_bt_xml>"
    assert "<IsPathValid path=\"{path}\"/>" in tree
    assert "goal_checker_id=\"general_goal_checker\"" in tree
    assert "<Spin" not in tree
    assert "<BackUp" not in tree
    assert "<nav_to_pose_bt_xml>" in launch
    assert "behavior_trees" in cmake
    assert "file(REMOVE" in cmake
    assert "goal_pose_bridge.py" in cmake
    twist_mux = yaml.safe_load(_read(
        "botbrain_ws/src/bot_bringup/config/twist_mux.yaml"))
    mux_params = twist_mux["/**"]["ros__parameters"]
    navigation_timeout = float(mux_params["topics"]["navigation"]["timeout"])
    assert 0.25 <= navigation_timeout <= 0.35
    assert navigation_timeout < float(mux_params["twist_watchdog_timeout"])


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
