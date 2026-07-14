from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
RUNBOOK_PATH = PROJECT_ROOT / "机器人项目run.md"


def _runbook():
    return RUNBOOK_PATH.read_text(encoding="utf-8")


def test_mapping_runbook_uses_current_install_and_launch_paths():
    source = _runbook()

    assert "/data/unitree/botbrain_ws" not in source
    assert "/data/botbrain_ws/botbrain_project-main" in source
    assert "/botbrain_ws/tools/mapping/shift_pcd_z.py" not in source
    assert "/botbrain_ws/install/g1_pkg/lib/g1_pkg/shift_pcd_z.py" in source
    assert "/botbrain_ws/start_localization.sh" not in source
    assert "grid_map_file:=" in source


def test_mapping_runbook_documents_safe_save_and_editor_semantics():
    source = _runbook()
    legacy_save = (PROJECT_ROOT / "tools/mapping/mapping_save.sh").read_text(
        encoding="utf-8")

    service_save = "ros2 service call /map_save std_srvs/srv/Trigger '{}'"
    stop_fast_lio = "docker compose stop -t 180 fast_lio"
    assert service_save in source
    assert stop_fast_lio in source
    assert source.index(service_save) < source.index(stop_fast_lio)
    assert "ros2 param get /fast_lio pcd_save.pcd_save_en" in source
    assert "ros2 param get /fast_lio map_file_path" in source
    assert "test -s \"$maps/floor1_scans.pcd\"" in source
    assert "test \"$maps/floor1_scans.pcd\" -nt \"$marker\"" in source
    assert "建图时 localization/navigation 必须停止" in source
    assert "关闭 `/Laser_map_1`" in source
    assert "ros2 topic info -v /cloud_registered_1" in source
    assert "sleep 1\nros2 topic info /accumulated_grid" in source
    assert "Fixed Frame：`map`" in source
    assert "map_odom_z=1.247" in source
    assert "Map/odom height constraint: enabled=true z=1.247 m" in source
    assert "Map/odom roll/pitch constraint: enabled=true" in source
    assert "Planar base TF: enabled=true odom -> g1_robot/base_footprint" in source
    assert "map_odom_rp=0.00/0.00 deg" in source
    assert "Manual relocalization applied" in source
    assert "ignoring /initialpose in frame 'camera_init'" in source
    assert "Waiting for odometry history" in source
    assert "检查 FAST-LIO 自身是否漂移：Fixed/Display Frame = `camera_init`" in source
    assert "左键**画黑" in source
    assert "右键**画白" in source
    assert "当前不会影响 Nav2" in source
    assert "mode: trinary" in source
    assert "ros2 lifecycle get \"/g1_robot/$node\"" in source
    assert "^/g1_robot/navigate_to_pose " in source
    assert "Nav2 readiness check failed after 90s" in source
    assert "g1_robot_mapping" in source
    assert "open3d_loc g1_pkg bot_navigation bot_state_machine" in source
    assert "python3 /botbrain_ws/tools/mapping/shift_pcd_z.py $PCD" not in legacy_save
    assert 'RAW_PCD="${PCD%.pcd}_raw.pcd"' in legacy_save
    assert "shift $PCD exactly once" in legacy_save


def test_mapping_runbook_has_balanced_markdown_fences():
    assert sum(line.startswith("```") for line in _runbook().splitlines()) % 2 == 0
