from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def _read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_mid360_rejects_motion_damaging_delivery_gaps():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    runbook = _read("机器人项目run.md")
    params = yaml.safe_load(_read(
        "botbrain_ws/src/fast_lio/config/mid360.yaml"
    ))["/**"]["ros__parameters"]

    assert params["common"]["max_imu_gap"] == 0.06
    assert 'declare_parameter<double>("common.max_imu_gap", 0.06)' in source
    assert 'get_parameter_or<double>("common.max_imu_gap", max_imu_gap, 0.06)' in source
    assert "observed_max_imu_gap <= max_imu_gap" in source
    assert "limit=%.4fs" in source
    assert "max_imu_gap=0.0600s max_range=0.0m guard=true" in runbook
    assert runbook.count("common.max_imu_gap") >= 2


def test_timing_gap_propagates_prediction_but_cannot_write_the_map():
    source = _read("botbrain_ws/src/fast_lio/src/laserMapping.cpp")
    runbook = _read("机器人项目run.md")
    imu_process = source.index("p_imu->Process(Measures, kf, feats_undistort)")
    quality_gate = source.index("const bool quality_ok = timing_ok", imu_process)
    assert imu_process < quality_gate
    assert "this scan cannot update/write the map" in source
    assert "时序异常帧可以推进 IMU 预测" in runbook
    assert "不得提交 LiDAR 修正或写入 ikd-tree" in runbook


def test_laserscan_does_not_wait_for_a_future_transform():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml"
    ))["pointcloud_to_laserscan_node"]["ros__parameters"]

    assert 0.05 <= float(params["transform_tolerance"]) <= 0.15
    assert params["queue_size"] >= 5
