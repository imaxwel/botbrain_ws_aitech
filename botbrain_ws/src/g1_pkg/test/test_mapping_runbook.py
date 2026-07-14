from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
RUNBOOK_PATH = PROJECT_ROOT / "机器人项目run.md"


def _runbook():
    return RUNBOOK_PATH.read_text(encoding="utf-8")


def test_mapping_runbook_uses_current_install_and_launch_paths():
    source = _runbook()

    assert "/botbrain_ws/tools/mapping/shift_pcd_z.py" not in source
    assert "/botbrain_ws/install/g1_pkg/lib/g1_pkg/shift_pcd_z.py" in source
    assert "/botbrain_ws/start_localization.sh" not in source
    assert "grid_map_file:=" in source


def test_mapping_runbook_documents_safe_save_and_editor_semantics():
    source = _runbook()

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
    assert "左键**画黑" in source
    assert "右键**画白" in source
    assert "当前不会影响 Nav2" in source
    assert "mode: trinary" in source


def test_mapping_runbook_has_balanced_markdown_fences():
    assert sum(line.startswith("```") for line in _runbook().splitlines()) % 2 == 0
