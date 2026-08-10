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

    assert params["common"]["max_imu_gap"] == 0.02
    assert 'declare_parameter<double>("common.max_imu_gap", 0.02)' in source
    assert 'get_parameter_or<double>("common.max_imu_gap", max_imu_gap, 0.02)' in source
    assert "observed_max_imu_gap <= max_imu_gap" in source
    assert "limit=%.4fs" in source
    assert "max_imu_gap=0.0200s max_range=0.0m guard=true" in runbook
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


def test_imu_initialization_aligns_gravity_only_from_a_stationary_window():
    source = _read("botbrain_ws/src/fast_lio/src/IMU_Processing.hpp")

    assert "MAX_INI_ATTEMPT_COUNT" in source
    assert "cur_gyr.norm() <= kMaxInitialGyroNorm" in source
    assert "std::abs(cur_acc.norm() - G_m_s2)" in source
    assert "cov_acc.maxCoeff() <= kMaxInitialAccelVariance" in source
    assert "cov_gyr.maxCoeff() <= kMaxInitialGyroVariance" in source
    assert "N = 0;" in source
    assert "Eigen::Quaterniond::FromTwoVectors" in source
    assert "measured_specific_force, world_up" in source
    assert "measured_tilt <= kMaxInitialGravityTiltRad" in source
    assert "alignment_error <= kMaxGravityAlignmentError" in source
    assert "init_state.rot = SO3(gravity_alignment);" in source
    assert "init_state.grav = S2(V3D(0.0, 0.0, -G_m_s2));" in source
    assert "preserving initial world orientation" in source


def test_laserscan_drops_untransformable_history_instead_of_replaying_it():
    params = yaml.safe_load(_read(
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml"
    ))["pointcloud_to_laserscan_node"]["ros__parameters"]

    assert float(params["transform_tolerance"]) == 0.0
    assert params["queue_size"] == 1
