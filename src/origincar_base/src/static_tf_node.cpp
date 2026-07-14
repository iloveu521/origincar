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

#include <chrono>
#include <memory>

#include "origincar_base/static_tf_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class OriginCarStaticTfNode : public rclcpp::Node {
public:
  OriginCarStaticTfNode()
  : Node("origincar_static_tf_node")
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    const auto config = ReadStaticTfConfig();
    const auto stamp = this->now();
    const auto transforms =
      origincar_base::MakeOriginCarStaticTransforms(stamp, config);

    broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(this->get_logger(),
                "Published OriginCar static TFs: base_link, gyro_link, "
                "laser(yaw=%.6f rad)",
                config.laser_yaw);
  }

private:
  origincar_base::StaticTfConfig ReadStaticTfConfig()
  {
    origincar_base::StaticTfConfig config;

    // Keep these as ROS parameters so LiDAR mounting yaw can be tuned in RViz
    // without editing C++.
    config.laser_x = this->declare_parameter<double>("laser_x", config.laser_x);
    config.laser_y = this->declare_parameter<double>("laser_y", config.laser_y);
    config.laser_z = this->declare_parameter<double>("laser_z", config.laser_z);
    config.laser_roll =
      this->declare_parameter<double>("laser_roll", config.laser_roll);
    config.laser_pitch =
      this->declare_parameter<double>("laser_pitch", config.laser_pitch);
    config.laser_yaw =
      this->declare_parameter<double>("laser_yaw", config.laser_yaw);

    return config;
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OriginCarStaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
