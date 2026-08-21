#ifndef OPEN3D_LOC__BBS_LOCALIZER_HPP_
#define OPEN3D_LOC__BBS_LOCALIZER_HPP_

#include "open3d_loc/bbs_branch_and_bound.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace open3d_loc
{

struct BbsMap
{
    bbs::Grid occupancy;
    double resolution_m = 0.0;
    double origin_x_m = 0.0;
    double origin_y_m = 0.0;
    double origin_yaw_rad = 0.0;
};

struct BbsPoseCandidate
{
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;
    double score = 0.0;
    int hit_count = 0;
    int point_count = 0;
};

struct ScanPreparationConfig
{
    double z_min_m = -0.5;
    double z_max_m = 1.5;
    double min_range_m = 0.5;
    double max_range_m = 25.0;
    double voxel_size_m = 0.15;
    std::size_t max_points = 512;
};

struct ScanObservabilityConfig
{
    std::size_t min_points = 100;
    double min_xy_span_m = 3.0;
    double min_max_range_m = 4.0;
};

struct ScanObservability
{
    bool observable = false;
    std::size_t point_count = 0;
    double x_span_m = 0.0;
    double y_span_m = 0.0;
    double max_range_m = 0.0;
    std::string reason;
};

bool LoadBbsMap(const std::string &yaml_path, double requested_resolution_m,
                BbsMap *map, std::string *error);

std::vector<std::array<double, 2>> PreparePlanarScan(
    const std::vector<std::array<double, 3>> &points,
    const ScanPreparationConfig &config);

ScanObservability EvaluateObservability(
    const std::vector<std::array<double, 2>> &scan,
    const ScanObservabilityConfig &config);

std::vector<BbsPoseCandidate> SearchBbsPoses(
    const BbsMap &map,
    const std::vector<std::array<double, 2>> &scan,
    double angular_resolution_rad,
    int pyramid_depth,
    int max_candidates,
    double nms_radius_m,
    double min_score);

}  // namespace open3d_loc

#endif  // OPEN3D_LOC__BBS_LOCALIZER_HPP_
