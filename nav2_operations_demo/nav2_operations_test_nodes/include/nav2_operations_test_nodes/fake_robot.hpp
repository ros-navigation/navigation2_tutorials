#ifndef NAV2_OPERATIONS_TEST_NODES__FAKE_ROBOT_HPP_
#define NAV2_OPERATIONS_TEST_NODES__FAKE_ROBOT_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace nav2_operations_test_nodes
{

class FakeRobot : public rclcpp::Node
{
public:
  explicit FakeRobot(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timerCallback();
  void publishTF();
  void publishPose();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  double x_, y_, yaw_;
  double linear_vel_, angular_vel_;
  rclcpp::Time last_update_time_;
  nav_msgs::msg::Path traveled_path_;

  // Parameters
  double publish_rate_;
};

}  // namespace nav2_operations_test_nodes

#endif
