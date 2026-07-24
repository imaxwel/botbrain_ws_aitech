#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace g1_loop_closure
{

inline constexpr double kPi = 3.14159265358979323846;

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double WrapAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline Pose2 Compose(const Pose2 &left, const Pose2 &right)
{
  const double c = std::cos(left.yaw);
  const double s = std::sin(left.yaw);
  return {
    left.x + c * right.x - s * right.y,
    left.y + s * right.x + c * right.y,
    WrapAngle(left.yaw + right.yaw),
  };
}

inline Pose2 Inverse(const Pose2 &pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {
    -c * pose.x - s * pose.y,
    s * pose.x - c * pose.y,
    WrapAngle(-pose.yaw),
  };
}

inline Pose2 Between(const Pose2 &from, const Pose2 &to)
{
  return Compose(Inverse(from), to);
}

struct PoseGraphEdge
{
  size_t from{0};
  size_t to{0};
  Pose2 measurement;
  double xy_weight{1.0};
  double yaw_weight{1.0};
  bool is_loop{false};
};

// The caller owns the input poses and keeps node 0 fixed.  This function is
// deliberately independent from ROS and FAST-LIO so the optimizer cannot
// publish TF or alter any online estimate.
bool OptimizePoseGraph(
  std::vector<Pose2> &poses,
  const std::vector<PoseGraphEdge> &edges,
  int iterations,
  std::string *failure_reason);

}  // namespace g1_loop_closure
