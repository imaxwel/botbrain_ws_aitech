#include "open3d_loc/bbs_localizer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace open3d_loc
{
namespace
{

bool Finite(double value)
{
    return std::isfinite(value);
}

std::string ResolveImagePath(const std::string &yaml_path,
                             const std::string &image_path)
{
    if (!image_path.empty() && image_path.front() == '/')
    {
        return image_path;
    }
    const std::size_t separator = yaml_path.find_last_of('/');
    if (separator == std::string::npos)
    {
        return image_path;
    }
    return yaml_path.substr(0, separator + 1) + image_path;
}

}  // namespace

bool LoadBbsMap(const std::string &yaml_path, double requested_resolution_m,
                BbsMap *map, std::string *error)
{
    if (map == nullptr)
    {
        if (error != nullptr)
        {
            *error = "output map pointer is null";
        }
        return false;
    }
    try
    {
        const YAML::Node document = YAML::LoadFile(yaml_path);
        const double source_resolution = document["resolution"].as<double>();
        const std::vector<double> origin =
            document["origin"].as<std::vector<double>>();
        const double occupied_threshold =
            document["occupied_thresh"].as<double>(0.65);
        const int negate = document["negate"].as<int>(0);
        const std::string image_path = ResolveImagePath(
            yaml_path, document["image"].as<std::string>());
        if (!Finite(source_resolution) || source_resolution <= 0.0 ||
            origin.size() < 3 || !Finite(origin[0]) || !Finite(origin[1]) ||
            !Finite(origin[2]) || !Finite(occupied_threshold) ||
            occupied_threshold < 0.0 || occupied_threshold > 1.0)
        {
            throw std::runtime_error("invalid map metadata");
        }
        const double output_resolution = requested_resolution_m > 0.0
            ? requested_resolution_m : source_resolution;
        if (!Finite(output_resolution) || output_resolution <= 0.0)
        {
            throw std::runtime_error("invalid requested BBS resolution");
        }

        const cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if (image.empty())
        {
            throw std::runtime_error("cannot read map image " + image_path);
        }
        const int width = std::max(
            1, static_cast<int>(std::ceil(
                   image.cols * source_resolution / output_resolution)));
        const int height = std::max(
            1, static_cast<int>(std::ceil(
                   image.rows * source_resolution / output_resolution)));
        BbsMap loaded;
        loaded.occupancy = bbs::Grid(height, width);
        loaded.resolution_m = output_resolution;
        loaded.origin_x_m = origin[0];
        loaded.origin_y_m = origin[1];
        loaded.origin_yaw_rad = origin[2];
        for (int image_row = 0; image_row < image.rows; ++image_row)
        {
            for (int image_column = 0; image_column < image.cols; ++image_column)
            {
                const int pixel = image.at<std::uint8_t>(image_row, image_column);
                const double occupied_probability = negate == 0
                    ? (255.0 - pixel) / 255.0 : pixel / 255.0;
                if (occupied_probability <= occupied_threshold)
                {
                    continue;
                }
                const double x_m =
                    (static_cast<double>(image_column) + 0.5) * source_resolution;
                const double y_m =
                    (static_cast<double>(image.rows - 1 - image_row) + 0.5) *
                    source_resolution;
                const int x_cell = std::min(
                    width - 1, static_cast<int>(std::floor(x_m / output_resolution)));
                const int y_cell = std::min(
                    height - 1, static_cast<int>(std::floor(y_m / output_resolution)));
                loaded.occupancy.set(y_cell, x_cell, 1);
            }
        }
        *map = std::move(loaded);
        return true;
    }
    catch (const std::exception &exception)
    {
        if (error != nullptr)
        {
            *error = exception.what();
        }
        return false;
    }
}

std::vector<std::array<double, 2>> PreparePlanarScan(
    const std::vector<std::array<double, 3>> &points,
    const ScanPreparationConfig &config)
{
    if (config.z_min_m > config.z_max_m || config.min_range_m < 0.0 ||
        config.max_range_m <= config.min_range_m || config.voxel_size_m <= 0.0 ||
        config.max_points == 0)
    {
        throw std::invalid_argument("invalid planar scan preparation config");
    }
    std::map<std::pair<int, int>, std::array<double, 2>> voxels;
    for (const auto &point : points)
    {
        if (!Finite(point[0]) || !Finite(point[1]) || !Finite(point[2]) ||
            point[2] < config.z_min_m || point[2] > config.z_max_m)
        {
            continue;
        }
        const double range = std::hypot(point[0], point[1]);
        if (range < config.min_range_m || range > config.max_range_m)
        {
            continue;
        }
        const int voxel_x = static_cast<int>(
            std::floor(point[0] / config.voxel_size_m));
        const int voxel_y = static_cast<int>(
            std::floor(point[1] / config.voxel_size_m));
        voxels.emplace(std::make_pair(voxel_x, voxel_y),
                       std::array<double, 2>{point[0], point[1]});
    }
    std::vector<std::array<double, 2>> scan;
    scan.reserve(std::min(config.max_points, voxels.size()));
    const std::size_t stride = std::max<std::size_t>(
        1, static_cast<std::size_t>(
               std::ceil(static_cast<double>(voxels.size()) / config.max_points)));
    std::size_t index = 0;
    for (const auto &entry : voxels)
    {
        if (index % stride == 0 && scan.size() < config.max_points)
        {
            scan.push_back(entry.second);
        }
        ++index;
    }
    return scan;
}

ScanObservability EvaluateObservability(
    const std::vector<std::array<double, 2>> &scan,
    const ScanObservabilityConfig &config)
{
    ScanObservability result;
    result.point_count = scan.size();
    if (scan.empty())
    {
        result.reason = "scan contains no usable planar points";
        return result;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto &point : scan)
    {
        min_x = std::min(min_x, point[0]);
        max_x = std::max(max_x, point[0]);
        min_y = std::min(min_y, point[1]);
        max_y = std::max(max_y, point[1]);
        result.max_range_m = std::max(
            result.max_range_m, std::hypot(point[0], point[1]));
    }
    result.x_span_m = max_x - min_x;
    result.y_span_m = max_y - min_y;
    if (result.point_count < config.min_points)
    {
        result.reason = "too few scan points";
    }
    else if (std::max(result.x_span_m, result.y_span_m) < config.min_xy_span_m)
    {
        result.reason = "scan geometry span is too small";
    }
    else if (result.max_range_m < config.min_max_range_m)
    {
        result.reason = "scan range is too short";
    }
    else
    {
        result.observable = true;
        result.reason = "observable";
    }
    return result;
}

std::vector<BbsPoseCandidate> SearchBbsPoses(
    const BbsMap &map,
    const std::vector<std::array<double, 2>> &scan,
    double angular_resolution_rad,
    int pyramid_depth,
    int max_candidates,
    double nms_radius_m,
    double min_score)
{
    const int nms_cells = std::max(
        0, static_cast<int>(std::ceil(nms_radius_m / map.resolution_m)));
    const auto grid_candidates = bbs::Search(
        map.occupancy, scan, map.resolution_m, angular_resolution_rad,
        pyramid_depth, max_candidates, nms_cells);
    std::vector<BbsPoseCandidate> poses;
    const double cosine = std::cos(map.origin_yaw_rad);
    const double sine = std::sin(map.origin_yaw_rad);
    for (const auto &candidate : grid_candidates)
    {
        if (candidate.score < min_score)
        {
            continue;
        }
        const double local_x =
            (static_cast<double>(candidate.x_cell) + 0.5) * map.resolution_m;
        const double local_y =
            (static_cast<double>(candidate.y_cell) + 0.5) * map.resolution_m;
        poses.push_back({
            map.origin_x_m + cosine * local_x - sine * local_y,
            map.origin_y_m + sine * local_x + cosine * local_y,
            bbs::NormalizeAngle(map.origin_yaw_rad + candidate.yaw_rad),
            candidate.score,
            candidate.hit_count,
            candidate.point_count,
        });
    }
    return poses;
}

}  // namespace open3d_loc
