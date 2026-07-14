#include "origincar_task/mission_policy.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace policy = origincar_task::mission_policy;

TEST(MissionPolicy, ParsesAndRejectsQrPayloads) {
  EXPECT_EQ(policy::ParseQrDirection("cw"), 1);
  EXPECT_EQ(policy::ParseQrDirection("clockwise"), 1);
  EXPECT_EQ(policy::ParseQrDirection("顺时针"), 1);
  EXPECT_EQ(policy::ParseQrDirection("ccw"), 2);
  EXPECT_EQ(policy::ParseQrDirection("counterclockwise"), 2);
  EXPECT_EQ(policy::ParseQrDirection("逆时针"), 2);
  EXPECT_EQ(policy::ParseQrDirection("3"), 1);
  EXPECT_EQ(policy::ParseQrDirection("4"), 2);
  EXPECT_EQ(policy::ParseQrDirection("3abc"), 0);
  EXPECT_EQ(policy::ParseQrDirection("unknown"), 0);
}

TEST(MissionPolicy, UsesMinimumLinearSpeedGain) {
  EXPECT_DOUBLE_EQ(policy::ComputeSpeedGain(1.0, true, true, 0.85, false, 0.60),
                   0.85);
  EXPECT_DOUBLE_EQ(
      policy::ComputeSpeedGain(0.50, true, true, 0.85, false, 0.60), 0.50);
  EXPECT_DOUBLE_EQ(
      policy::ComputeSpeedGain(1.0, false, true, 0.85, false, 0.60), 1.0);
  EXPECT_DOUBLE_EQ(
      policy::ComputeSpeedGain(0.50, false, false, 0.85, true, 0.60), 0.50);
}

TEST(MissionPolicy, ChoosesClearanceThenVisualSide) {
  EXPECT_DOUBLE_EQ(policy::ChooseAvoidDirection(0.8, 0.4, 0.2), 1.0);
  EXPECT_DOUBLE_EQ(policy::ChooseAvoidDirection(0.3, 0.7, 0.8), -1.0);
  EXPECT_DOUBLE_EQ(policy::ChooseAvoidDirection(0.5, 0.52, 0.2), -1.0);
  EXPECT_DOUBLE_EQ(policy::ChooseAvoidDirection(0.5, 0.52, 0.8), 1.0);
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(policy::ChooseAvoidDirection(inf, inf, 0.2), -1.0);
}
