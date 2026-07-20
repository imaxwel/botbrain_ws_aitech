from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def _read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_mid360_tolerates_short_imu_scheduling_gaps_via_parameter():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    runbook = _read("机器人项目run.md")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]

    assert params["common"]["max_imu_gap"] == 0.03
    assert 'declare_parameter<double>("common.max_imu_gap", 0.02)' in source
    assert 'get_parameter_or<double>("common.max_imu_gap", max_imu_gap, 0.02)' in source
    assert "observed_max_imu_gap <= max_imu_gap" in source
    assert "max_imu_gap <= 0.02" not in source
    assert "limit=%.4fs" in source
    assert "max_imu_gap=0.0300s guard=true" in runbook
    assert runbook.count("common.max_imu_gap") >= 2


def test_timing_rebase_preserves_future_imu_samples():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    runbook = _read("机器人项目run.md")
    helper = source.split("void DropBufferedLidarData()", 1)[1].split(
        "inline void dump_lio_state_to_log", 1
    )[0]
    timing_recovery = source.split("if (!timing_ok)", 1)[1].split(
        "double t0, t1", 1
    )[0]

    assert "lidar_buffer.clear();" in helper
    assert "time_buffer.clear();" in helper
    assert "lidar_pushed = false;" in helper
    assert "imu_buffer.clear();" not in helper
    assert "DropBufferedLidarData();" in timing_recovery
    assert "discarded buffered LiDAR data, preserved future" in source
    assert "discarded buffered LiDAR data, preserved future IMU samples" in runbook


def test_laserscan_does_not_wait_for_a_future_transform():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml"
    ))["pointcloud_to_laserscan_node"]["ros__parameters"]

    assert params["transform_tolerance"] == 0.0
    assert params["queue_size"] >= 5
