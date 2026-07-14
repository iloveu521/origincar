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

#ifndef ORIGINCAR_BASE__STATIC_TF_CONFIG_HPP_
#define ORIGINCAR_BASE__STATIC_TF_CONFIG_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace origincar_base
{

struct StaticTfConfig
{
  double laser_x = 0.083;
  double laser_y = 0.0;
  double laser_z = 0.102;
  double laser_roll = 0.0;
  double laser_pitch = 0.0;
  double laser_yaw = 0.0;
};

inline geometry_msgs::msg::TransformStamped
MakeStaticTf(
  const rclcpp::Time & stamp, double x, double y, double z,
  double roll, double pitch, double yaw, const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf.transform.rotation = tf2::toMsg(q);
  return tf;
}

inline std::vector<geometry_msgs::msg::TransformStamped>
MakeOriginCarStaticTransforms(
  const rclcpp::Time & stamp,
  const StaticTfConfig & config)
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(3);
  transforms.emplace_back(MakeStaticTf(stamp, 0.092, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       "base_footprint", "base_link"));
  transforms.emplace_back(MakeStaticTf(stamp, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       "base_footprint", "gyro_link"));
  transforms.emplace_back(MakeStaticTf(
      stamp, config.laser_x, config.laser_y, config.laser_z, config.laser_roll,
      config.laser_pitch, config.laser_yaw, "base_link", "laser"));
  return transforms;
}

}  // namespace origincar_base

#endif  // ORIGINCAR_BASE__STATIC_TF_CONFIG_HPP_
