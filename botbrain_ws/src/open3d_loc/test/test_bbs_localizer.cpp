#include "open3d_loc/bbs_localizer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace
{

TEST(BbsBranchAndBound, ReturnsExactTopCandidate)
{
    open3d_loc::bbs::Grid grid(12, 14);
    const std::vector<std::array<double, 2>> scan = {
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
        {0.0, 1.0}, {0.0, 2.0}, {1.0, 2.0},
    };
    constexpr int expected_x = 5;
    constexpr int expected_y = 4;
    for (const auto &point : scan)
    {
        grid.set(expected_y + static_cast<int>(point[1]),
                 expected_x + static_cast<int>(point[0]), 1);
    }

    const auto candidates = open3d_loc::bbs::Search(
        grid, scan, 1.0, open3d_loc::bbs::kPi / 2.0, 3, 5, 0);

    ASSERT_FALSE(candidates.empty());
    EXPECT_DOUBLE_EQ(candidates.front().score, 1.0);
    EXPECT_EQ(candidates.front().x_cell, expected_x);
    EXPECT_EQ(candidates.front().y_cell, expected_y);
    EXPECT_NEAR(candidates.front().yaw_rad, 0.0, 1.0e-12);
}

TEST(BbsLocalizer, ConvertsGridCellThroughRotatedMapOrigin)
{
    open3d_loc::BbsMap map;
    map.occupancy = open3d_loc::bbs::Grid(5, 6);
    map.occupancy.set(2, 3, 1);
    map.resolution_m = 0.5;
    map.origin_x_m = 10.0;
    map.origin_y_m = -2.0;
    map.origin_yaw_rad = open3d_loc::bbs::kPi / 2.0;

    const auto poses = open3d_loc::SearchBbsPoses(
        map, {{0.0, 0.0}}, 2.0 * open3d_loc::bbs::kPi,
        2, 1, 0.0, 0.99);

    ASSERT_EQ(poses.size(), 1U);
    EXPECT_NEAR(poses[0].x_m, 8.75, 1.0e-12);
    EXPECT_NEAR(poses[0].y_m, -0.25, 1.0e-12);
    EXPECT_NEAR(poses[0].yaw_rad, open3d_loc::bbs::kPi / 2.0, 1.0e-12);
}

TEST(BbsLocalizer, ObservabilityRejectsElevatorLikeSmallScan)
{
    open3d_loc::ScanObservabilityConfig config;
    config.min_points = 4;
    config.min_xy_span_m = 3.0;
    config.min_max_range_m = 4.0;

    const auto rejected = open3d_loc::EvaluateObservability(
        {{0.5, 0.5}, {0.6, 0.5}, {0.5, 0.6}, {0.6, 0.6}}, config);
    const auto accepted = open3d_loc::EvaluateObservability(
        {{-3.0, 0.0}, {3.0, 0.0}, {0.0, -4.0}, {0.0, 4.0}}, config);

    EXPECT_FALSE(rejected.observable);
    EXPECT_EQ(rejected.reason, "scan geometry span is too small");
    EXPECT_TRUE(accepted.observable);
    EXPECT_EQ(accepted.reason, "observable");
}

TEST(BbsLocalizer, PlanarPreparationFiltersAndVoxelsDeterministically)
{
    open3d_loc::ScanPreparationConfig config;
    config.z_min_m = 0.0;
    config.z_max_m = 1.0;
    config.min_range_m = 0.5;
    config.max_range_m = 10.0;
    config.voxel_size_m = 1.0;
    config.max_points = 10;
    const std::vector<std::array<double, 3>> points = {
        {1.1, 1.1, 0.5}, {1.2, 1.2, 0.5}, {2.1, 1.1, 0.5},
        {3.0, 3.0, 2.0}, {0.1, 0.1, 0.5},
    };

    const auto scan = open3d_loc::PreparePlanarScan(points, config);

    ASSERT_EQ(scan.size(), 2U);
    EXPECT_DOUBLE_EQ(scan[0][0], 1.1);
    EXPECT_DOUBLE_EQ(scan[1][0], 2.1);
}

}  // namespace
