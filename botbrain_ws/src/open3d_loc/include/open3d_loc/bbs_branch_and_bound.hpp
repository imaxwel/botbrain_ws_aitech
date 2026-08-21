#ifndef OPEN3D_LOC__BBS_BRANCH_AND_BOUND_HPP_
#define OPEN3D_LOC__BBS_BRANCH_AND_BOUND_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace open3d_loc
{
namespace bbs
{

constexpr double kPi = 3.14159265358979323846;

struct Grid
{
    int height = 0;
    int width = 0;
    std::vector<std::uint8_t> data;

    Grid() = default;
    Grid(int rows, int columns)
        : height(rows), width(columns),
          data(static_cast<std::size_t>(rows * columns), 0)
    {
        if (rows <= 0 || columns <= 0)
        {
            throw std::invalid_argument("BBS grid dimensions must be positive");
        }
    }

    std::uint8_t at(int row, int column) const
    {
        return data.at(static_cast<std::size_t>(row * width + column));
    }

    void set(int row, int column, std::uint8_t value)
    {
        data.at(static_cast<std::size_t>(row * width + column)) = value ? 1 : 0;
    }
};

struct Candidate
{
    int x_cell = 0;
    int y_cell = 0;
    int yaw_index = 0;
    double yaw_rad = 0.0;
    double score = 0.0;
    int hit_count = 0;
    int point_count = 0;
};

inline double NormalizeAngle(double angle)
{
    const double period = 2.0 * kPi;
    double normalized = std::fmod(angle + kPi, period);
    if (normalized < 0.0)
    {
        normalized += period;
    }
    return normalized - kPi;
}

inline Grid MaxPool2x2(const Grid &input)
{
    Grid output((input.height + 1) / 2, (input.width + 1) / 2);
    for (int row = 0; row < output.height; ++row)
    {
        for (int column = 0; column < output.width; ++column)
        {
            std::uint8_t occupied = 0;
            for (int dy = 0; dy < 2; ++dy)
            {
                const int source_row = row * 2 + dy;
                if (source_row >= input.height)
                {
                    continue;
                }
                for (int dx = 0; dx < 2; ++dx)
                {
                    const int source_column = column * 2 + dx;
                    if (source_column < input.width)
                    {
                        occupied = static_cast<std::uint8_t>(
                            occupied | input.at(source_row, source_column));
                    }
                }
            }
            output.set(row, column, occupied);
        }
    }
    return output;
}

inline Grid DilateOneCell(const Grid &input)
{
    Grid output(input.height, input.width);
    for (int row = 0; row < input.height; ++row)
    {
        for (int column = 0; column < input.width; ++column)
        {
            bool occupied = false;
            for (int dy = -1; dy <= 1 && !occupied; ++dy)
            {
                const int source_row = row + dy;
                if (source_row < 0 || source_row >= input.height)
                {
                    continue;
                }
                for (int dx = -1; dx <= 1; ++dx)
                {
                    const int source_column = column + dx;
                    if (source_column >= 0 && source_column < input.width &&
                        input.at(source_row, source_column) != 0)
                    {
                        occupied = true;
                        break;
                    }
                }
            }
            output.set(row, column, occupied ? 1 : 0);
        }
    }
    return output;
}

inline std::vector<double> YawSamples(double angular_resolution_rad)
{
    if (!std::isfinite(angular_resolution_rad) ||
        angular_resolution_rad <= 0.0)
    {
        throw std::invalid_argument("BBS angular resolution must be positive");
    }
    const int count = std::max(
        1, static_cast<int>(std::ceil(2.0 * kPi / angular_resolution_rad)));
    const double step = 2.0 * kPi / static_cast<double>(count);
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index)
    {
        samples.push_back(NormalizeAngle(static_cast<double>(index) * step));
    }
    return samples;
}

namespace detail
{

struct OffsetCell
{
    int row = 0;
    int column = 0;
    int count = 0;
};

struct Node
{
    double negative_bound = 0.0;
    std::uint64_t sequence = 0;
    int level = 0;
    int x_cell = 0;
    int y_cell = 0;
    int yaw_index = 0;
};

struct NodeGreater
{
    bool operator()(const Node &left, const Node &right) const
    {
        if (left.negative_bound != right.negative_bound)
        {
            return left.negative_bound > right.negative_bound;
        }
        return left.sequence > right.sequence;
    }
};

inline int GatherHits(const Grid &grid,
                      const std::vector<OffsetCell> &offsets,
                      int level, int x_cell, int y_cell)
{
    const int origin_row = y_cell >> level;
    const int origin_column = x_cell >> level;
    int hits = 0;
    for (const auto &offset : offsets)
    {
        const int row = origin_row + offset.row;
        const int column = origin_column + offset.column;
        if (row >= 0 && row < grid.height && column >= 0 &&
            column < grid.width && grid.at(row, column) != 0)
        {
            hits += offset.count;
        }
    }
    return hits;
}

inline bool BetterCandidate(const Candidate &left, const Candidate &right)
{
    if (left.score != right.score)
    {
        return left.score > right.score;
    }
    if (left.yaw_index != right.yaw_index)
    {
        return left.yaw_index < right.yaw_index;
    }
    if (left.y_cell != right.y_cell)
    {
        return left.y_cell < right.y_cell;
    }
    return left.x_cell < right.x_cell;
}

}  // namespace detail

inline std::vector<Candidate> Search(
    const Grid &occupancy,
    const std::vector<std::array<double, 2>> &scan_xy_m,
    double resolution_m,
    double angular_resolution_rad,
    int pyramid_depth,
    int max_candidates,
    int nms_radius_cells)
{
    if (!std::isfinite(resolution_m) || resolution_m <= 0.0)
    {
        throw std::invalid_argument("BBS map resolution must be positive");
    }
    if (pyramid_depth < 0 || max_candidates <= 0 || scan_xy_m.empty())
    {
        return {};
    }

    std::vector<Grid> exact_levels{occupancy};
    for (int level = 0; level < pyramid_depth; ++level)
    {
        if (exact_levels.back().height == 1 && exact_levels.back().width == 1)
        {
            break;
        }
        exact_levels.push_back(MaxPool2x2(exact_levels.back()));
    }
    std::vector<Grid> upper_levels{exact_levels.front()};
    for (std::size_t level = 1; level < exact_levels.size(); ++level)
    {
        upper_levels.push_back(DilateOneCell(exact_levels[level]));
    }

    const auto yaws = YawSamples(angular_resolution_rad);
    const int level_count = static_cast<int>(upper_levels.size());
    const int start_level = level_count - 1;
    const int point_count = static_cast<int>(scan_xy_m.size());
    std::vector<std::vector<std::vector<detail::OffsetCell>>> offsets(
        yaws.size(), std::vector<std::vector<detail::OffsetCell>>(
                         static_cast<std::size_t>(level_count)));

    for (std::size_t yaw_index = 0; yaw_index < yaws.size(); ++yaw_index)
    {
        const double cosine = std::cos(yaws[yaw_index]);
        const double sine = std::sin(yaws[yaw_index]);
        for (int level = 0; level < level_count; ++level)
        {
            const double divisor = static_cast<double>(1 << level);
            std::map<std::pair<int, int>, int> counts;
            for (const auto &point : scan_xy_m)
            {
                const double x = (cosine * point[0] - sine * point[1]) /
                                 resolution_m;
                const double y = (sine * point[0] + cosine * point[1]) /
                                 resolution_m;
                const int column = static_cast<int>(
                    std::floor((0.5 + x) / divisor));
                const int row = static_cast<int>(
                    std::floor((0.5 + y) / divisor));
                ++counts[{row, column}];
            }
            for (const auto &entry : counts)
            {
                offsets[yaw_index][static_cast<std::size_t>(level)].push_back(
                    {entry.first.first, entry.first.second, entry.second});
            }
        }
    }

    std::priority_queue<detail::Node, std::vector<detail::Node>,
                        detail::NodeGreater> queue;
    std::uint64_t sequence = 0;
    const int initial_step = 1 << start_level;
    for (std::size_t yaw_index = 0; yaw_index < yaws.size(); ++yaw_index)
    {
        for (int y = 0; y < occupancy.height; y += initial_step)
        {
            for (int x = 0; x < occupancy.width; x += initial_step)
            {
                const int hits = detail::GatherHits(
                    upper_levels[static_cast<std::size_t>(start_level)],
                    offsets[yaw_index][static_cast<std::size_t>(start_level)],
                    start_level, x, y);
                const double bound = static_cast<double>(hits) / point_count;
                queue.push({-bound, sequence++, start_level, x, y,
                            static_cast<int>(yaw_index)});
            }
        }
    }

    std::vector<Candidate> best;
    double kth_score = -std::numeric_limits<double>::infinity();
    constexpr double epsilon = 1.0e-12;
    while (!queue.empty())
    {
        const detail::Node node = queue.top();
        queue.pop();
        const double bound = -node.negative_bound;
        if (static_cast<int>(best.size()) >= max_candidates &&
            bound <= kth_score + epsilon)
        {
            break;
        }
        if (node.level == 0)
        {
            const int hits = detail::GatherHits(
                upper_levels.front(),
                offsets[static_cast<std::size_t>(node.yaw_index)].front(),
                0, node.x_cell, node.y_cell);
            Candidate candidate{node.x_cell, node.y_cell, node.yaw_index,
                                yaws[static_cast<std::size_t>(node.yaw_index)],
                                static_cast<double>(hits) / point_count,
                                hits, point_count};
            bool suppressed = false;
            if (nms_radius_cells > 0)
            {
                for (auto &accepted : best)
                {
                    if (std::abs(accepted.x_cell - candidate.x_cell) <=
                            nms_radius_cells &&
                        std::abs(accepted.y_cell - candidate.y_cell) <=
                            nms_radius_cells &&
                        accepted.yaw_index == candidate.yaw_index)
                    {
                        if (detail::BetterCandidate(candidate, accepted))
                        {
                            accepted = candidate;
                        }
                        suppressed = true;
                        break;
                    }
                }
            }
            if (!suppressed)
            {
                best.push_back(candidate);
            }
            std::stable_sort(best.begin(), best.end(), detail::BetterCandidate);
            if (static_cast<int>(best.size()) > max_candidates)
            {
                best.resize(static_cast<std::size_t>(max_candidates));
            }
            if (static_cast<int>(best.size()) >= max_candidates)
            {
                kth_score = best.back().score;
            }
            continue;
        }

        const int child_level = node.level - 1;
        const int child_step = 1 << child_level;
        for (int dy = 0; dy <= child_step; dy += child_step)
        {
            const int y = node.y_cell + dy;
            if (y >= occupancy.height)
            {
                continue;
            }
            for (int dx = 0; dx <= child_step; dx += child_step)
            {
                const int x = node.x_cell + dx;
                if (x >= occupancy.width)
                {
                    continue;
                }
                const int hits = detail::GatherHits(
                    upper_levels[static_cast<std::size_t>(child_level)],
                    offsets[static_cast<std::size_t>(node.yaw_index)]
                           [static_cast<std::size_t>(child_level)],
                    child_level, x, y);
                const double child_bound =
                    static_cast<double>(hits) / point_count;
                if (static_cast<int>(best.size()) < max_candidates ||
                    child_bound >= kth_score - epsilon)
                {
                    queue.push({-child_bound, sequence++, child_level, x, y,
                                node.yaw_index});
                }
            }
        }
    }
    std::stable_sort(best.begin(), best.end(), detail::BetterCandidate);
    return best;
}

}  // namespace bbs
}  // namespace open3d_loc

#endif  // OPEN3D_LOC__BBS_BRANCH_AND_BOUND_HPP_
