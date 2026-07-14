#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelToAckermannDriveNode : public rclcpp::Node
{
public:
  CmdVelToAckermannDriveNode() : Node("cmd_vel_to_ackermann_drive")
  {
    declare_parameter<double>("wheelbase", 0.143);
    declare_parameter<std::string>("frame_id", "odom_combined");
    declare_parameter<bool>("cmd_angle_instead_rotvel", false);

    wheelbase_ = get_parameter("wheelbase").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    cmd_angle_instead_rotvel_ = get_parameter("cmd_angle_instead_rotvel").as_bool();

    publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&CmdVelToAckermannDriveNode::cmd_callback, this, std::placeholders::_1));
  }

private:
  double convert_trans_rot_vel_to_steering_angle(double vel, double omega) const
  {
    if (omega == 0.0 || vel == 0.0) {
      return 0.0;
    }
    const double radius = vel / omega;
    return std::atan(wheelbase_ / radius);
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr data)
  {
    const double vel = data->linear.x;
    const double steering = cmd_angle_instead_rotvel_ ?
      data->angular.z : convert_trans_rot_vel_to_steering_angle(vel, data->angular.z);

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.drive.steering_angle = steering;
    msg.drive.speed = vel;
    publisher_->publish(msg);
  }

  double wheelbase_{0.143};
  std::string frame_id_{"odom_combined"};
  bool cmd_angle_instead_rotvel_{false};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToAckermannDriveNode>());
  rclcpp::shutdown();
  return 0;
}
