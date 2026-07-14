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

    assert "DeclareLaunchArgument('map_file', default_value=default_pcd_path)" in source
    assert "DeclareLaunchArgument('grid_map_file', default_value=default_grid_yaml)" in source
    assert "'path_map':                 LaunchConfiguration('map_file')" in source
    assert "'yaml_filename': LaunchConfiguration('grid_map_file')" in source
    assert "os.path.join(workspace_dir, 'src', 'g1_pkg', 'maps', 'accumulated.yaml')" in source


def test_localization_service_starts_the_installed_launch_file_directly():
    source = _read("docker-compose.yaml")

    assert "/botbrain_ws/start_localization.sh" not in source
    assert "sleep 30" in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert "source /botbrain_ws/install/setup.bash" in source
    assert "exec ros2 launch g1_pkg localization_3d.launch.py" in source


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


def test_mapping_disables_unbounded_laser_map_publication():
    config = yaml.safe_load(
        _read("botbrain_ws/src/fast_lio/config/mid360.yaml"))
    foxglove = yaml.safe_load(_read(
        "botbrain_ws/src/bot_bringup/config/foxglove_bridge_params.yaml"))

    params = config["/**"]["ros__parameters"]
    bridge_params = foxglove["/**"]["ros__parameters"]
    assert params["publish"]["map_en"] is False
    assert "/Laser_map_1" not in bridge_params["topic_whitelist"]


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
    assert "100000" not in source


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
    local_costmap = params["/**/local_costmap"]["local_costmap"]["ros__parameters"]
    assert local_costmap["global_frame"] == "<prefix>odom"
    bt_params = params["/**/bt_navigator"]["ros__parameters"]
    assert "default_bt_xml_filename" not in bt_params
    source = _read("botbrain_ws/src/g1_pkg/config/nav2_params.yaml")
    assert "obstacle_range:" not in source
    assert "raytrace_range:" not in source
    assert "observation_queue_length:" not in source
    assert source.count("obstacle_max_range:") == 2
    assert source.count("raytrace_max_range:") == 2
    global_cloud = params["/**/global_costmap"]["global_costmap"][
        "ros__parameters"]["obstacle_layer"]["cloud"]
    assert global_cloud["raytrace_max_range"] >= global_cloud["obstacle_max_range"]


def test_waypoints_are_planar_and_never_reanchor_localization():
    recorder = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_recorder.py")
    navigator = _read(
        "botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py")
    waypoints = yaml.safe_load(_read(
        "botbrain_ws/src/bot_navigation/nav_waypoints.yaml"))["waypoints"]

    assert "def _yaw_quaternion(" in recorder
    assert "def _planar_quaternion(" in navigator
    assert "PoseWithCovarianceStamped" not in navigator
    assert "initialpose_pub" not in navigator
    assert "goal.pose.pose.position.z = 0.0" in navigator
    assert "wait_for_server(timeout_sec=60.0)" in navigator
    assert "Stopping waypoint sequence after navigation failure." in navigator
    for waypoint in waypoints.values():
        assert float(waypoint["z"]) == 0.0
        assert float(waypoint["qx"]) == 0.0
        assert float(waypoint["qy"]) == 0.0
        norm = math.hypot(float(waypoint["qz"]), float(waypoint["qw"]))
        assert math.isclose(norm, 1.0, abs_tol=2e-6)


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
