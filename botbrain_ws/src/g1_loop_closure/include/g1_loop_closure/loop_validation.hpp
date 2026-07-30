#pragma once

#include <cmath>

// Keep validation and geometry on the same source revision even when an older
// workspace overlay is present in the compiler's package include paths.
#include "pose_graph.hpp"

namespace g1_loop_closure
{

inline bool IsLoopYawConsistent(
  const double measurement_yaw, const double odometry_yaw,
  const double max_absolute_yaw, const double max_odometry_delta_yaw)
{
  return std::isfinite(measurement_yaw) && std::isfinite(odometry_yaw) &&
    std::isfinite(max_absolute_yaw) && std::isfinite(max_odometry_delta_yaw) &&
    std::fabs(WrapAngle(measurement_yaw)) <= max_absolute_yaw &&
    std::fabs(WrapAngle(measurement_yaw - odometry_yaw)) <=
    max_odometry_delta_yaw;
}

inline bool AreLoopCorrectionsConsistent(
  const Pose2 &reference, const Pose2 &candidate,
  const double max_translation, const double max_yaw)
{
  if (!std::isfinite(reference.x) || !std::isfinite(reference.y) ||
    !std::isfinite(reference.yaw) || !std::isfinite(candidate.x) ||
    !std::isfinite(candidate.y) || !std::isfinite(candidate.yaw) ||
    !std::isfinite(max_translation) || !std::isfinite(max_yaw) ||
    max_translation < 0.0 || max_yaw < 0.0)
  {
    return false;
  }
  const double translation_delta = std::hypot(
    candidate.x - reference.x, candidate.y - reference.y);
  const double yaw_delta = std::fabs(WrapAngle(candidate.yaw - reference.yaw));
  return std::isfinite(translation_delta) && std::isfinite(yaw_delta) &&
    translation_delta <= max_translation && yaw_delta <= max_yaw;
}

}  // namespace g1_loop_closure
