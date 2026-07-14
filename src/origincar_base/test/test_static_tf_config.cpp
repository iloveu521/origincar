// Copyright 2026 OriginCar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <vector>

#include "origincar_base/static_tf_config.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gtest/gtest.h"

namespace
{
double GetYaw(const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

const geometry_msgs::msg::TransformStamped & FindLaserTf(
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms)
{
  for (const auto & transform : transforms) {
    if (transform.header.frame_id == "base_link" &&
      transform.child_frame_id == "laser")
    {
      return transform;
    }
  }
  throw std::runtime_error("base_link -> laser transform not found");
}
}  // namespace

TEST(StaticTfConfig, DefaultsPreserveExistingLaserTransform) {
  const auto transforms = origincar_base::MakeOriginCarStaticTransforms(
      rclcpp::Time(0, 0, RCL_ROS_TIME), origincar_base::StaticTfConfig{});
  const auto & laser_tf = FindLaserTf(transforms);

  EXPECT_DOUBLE_EQ(laser_tf.transform.translation.x, 0.083);
  EXPECT_DOUBLE_EQ(laser_tf.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(laser_tf.transform.translation.z, 0.102);
  EXPECT_NEAR(GetYaw(laser_tf), 0.0, 1e-9);
}

TEST(StaticTfConfig, AppliesLaserYawOffset) {
  origincar_base::StaticTfConfig config;
  config.laser_yaw = 0.035;

  const auto transforms = origincar_base::MakeOriginCarStaticTransforms(
      rclcpp::Time(0, 0, RCL_ROS_TIME), config);
  const auto & laser_tf = FindLaserTf(transforms);

  EXPECT_NEAR(GetYaw(laser_tf), config.laser_yaw, 1e-9);
}
