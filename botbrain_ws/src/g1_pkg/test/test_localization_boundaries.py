import re
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


def test_g1_localization_locks_corrected_map_height_only_in_g1_launch():
    source = _read(
        "botbrain_ws/src/open3d_loc/src/global_localization.cpp")
    launch = _read("botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py")

    assert 'declare_parameter<bool>("lock_map_odom_z", false)' in source
    assert "mat_initialpose_ = ConstrainMapOdom(mat_initialpose_);" in source
    assert "mat_odom2map_ = mat_initialpose_;" in source
    assert "candidate_odom2map = ConstrainMapOdom(" in source
    assert "effective_correction =" in source
    assert "new_odom2map = ConstrainMapOdom(" in source
    assert "mat_odom2map_kalman_ = ConstrainMapOdom(" in source
    assert "mat_odom2map_(2, 3), 1);" in source
    assert "Rejecting initial pose: waiting for valid odometry" in source
    assert "initialpose_frame != \"map\"" in source
    assert "'lock_map_odom_z':          True" in launch
    assert "'map_odom_z':               IMU_HEIGHT" in launch


def test_initialpose_relay_rejects_non_map_frames():
    source = _read("botbrain_ws/src/g1_pkg/scripts/initialpose_z_fix.py")

    assert "frame_id = msg.header.frame_id.lstrip('/')" in source
    assert "if frame_id != 'map':" in source
    assert "set Foxglove Fixed Frame to 'map'" in source


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
