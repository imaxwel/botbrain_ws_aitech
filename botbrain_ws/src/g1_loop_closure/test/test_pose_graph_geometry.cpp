#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "g1_loop_closure/loop_validation.hpp"

namespace
{

using g1_loop_closure::IsLoopYawConsistent;
using g1_loop_closure::AreLoopCorrectionsConsistent;
using g1_loop_closure::kPi;
using g1_loop_closure::Pose2;

TEST(LoopYawGate, AcceptsOutAndBackWhenOdometryAlsoPredictsHalfTurn)
{
  EXPECT_TRUE(IsLoopYawConsistent(
      kPi, kPi, kPi, 25.0 * kPi / 180.0));
}

TEST(LoopYawGate, RejectsHalfTurnWhenOdometryPredictsSameDirection)
{
  EXPECT_FALSE(IsLoopYawConsistent(
      kPi, 0.0, kPi, 25.0 * kPi / 180.0));
}

TEST(LoopYawGate, AcceptsConsistentQuarterTurn)
{
  EXPECT_TRUE(IsLoopYawConsistent(
      0.5 * kPi, 0.5 * kPi - 5.0 * kPi / 180.0,
      kPi, 25.0 * kPi / 180.0));
}

TEST(LoopConfirmationGate, RejectsCumulativeWalkingCorrection)
{
  const Pose2 first{0.0, 0.0, 0.0};
  const Pose2 second{0.7, 0.0, 4.0 * kPi / 180.0};
  const Pose2 third{1.4, 0.0, 8.0 * kPi / 180.0};

  EXPECT_TRUE(AreLoopCorrectionsConsistent(
      first, second, 0.75, 8.0 * kPi / 180.0));
  EXPECT_FALSE(AreLoopCorrectionsConsistent(
      first, third, 0.75, 8.0 * kPi / 180.0));
}

TEST(LoopConfirmationGate, RejectsNonFiniteCorrection)
{
  const Pose2 reference{0.0, 0.0, 0.0};
  const Pose2 invalid_position{NAN, 0.0, 0.0};
  const Pose2 invalid_yaw{
    0.0, 0.0, std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(AreLoopCorrectionsConsistent(
      reference, invalid_position, 0.75, 8.0 * kPi / 180.0));
  EXPECT_FALSE(AreLoopCorrectionsConsistent(
      reference, invalid_yaw, 0.75, 8.0 * kPi / 180.0));
}

}  // namespace
