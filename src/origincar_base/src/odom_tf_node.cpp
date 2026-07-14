#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfNode : public rclcpp::Node
{
public:
  OdomTfNode()
  : Node("odom_tf_node")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg->header.frame_id.empty() || msg->child_frame_id.empty()) {
          return;
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg->header;
        tf.child_frame_id = msg->child_frame_id;
        tf.transform.translation.x = msg->pose.pose.position.x;
        tf.transform.translation.y = msg->pose.pose.position.y;
        tf.transform.translation.z = msg->pose.pose.position.z;
        tf.transform.rotation = msg->pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
      });

    RCLCPP_INFO(this->get_logger(), "Broadcasting TF from /odom messages");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
