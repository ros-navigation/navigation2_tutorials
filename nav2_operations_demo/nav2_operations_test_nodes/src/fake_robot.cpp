#include "nav2_operations_test_nodes/fake_robot.hpp"

#include <cmath>
#include <chrono>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_operations_test_nodes
{

FakeRobot::FakeRobot(const rclcpp::NodeOptions & options)
: Node("fake_robot", options),
  linear_vel_(0.0),
  angular_vel_(0.0)
{
  // Declare and get parameters
  this->declare_parameter("initial_x", 0.0);
  this->declare_parameter("initial_y", 0.0);
  this->declare_parameter("initial_yaw", 0.0);
  this->declare_parameter("publish_rate", 50.0);

  x_ = this->get_parameter("initial_x").as_double();
  y_ = this->get_parameter("initial_y").as_double();
  yaw_ = this->get_parameter("initial_yaw").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  // Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_smoothed", 10,
    std::bind(&FakeRobot::cmdVelCallback, this, std::placeholders::_1));

  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/fake_robot/traveled_path", 10);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/fake_robot/pose", 10);

  // TF broadcasters
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  // Publish static TF map -> odom (identity)
  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->now();
  map_to_odom.header.frame_id = "map";
  map_to_odom.child_frame_id = "odom";
  map_to_odom.transform.translation.x = 0.0;
  map_to_odom.transform.translation.y = 0.0;
  map_to_odom.transform.translation.z = 0.0;
  map_to_odom.transform.rotation.x = 0.0;
  map_to_odom.transform.rotation.y = 0.0;
  map_to_odom.transform.rotation.z = 0.0;
  map_to_odom.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(map_to_odom);

  // Initialize traveled path
  traveled_path_.header.frame_id = "map";

  // Initialize time
  last_update_time_ = this->now();

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&FakeRobot::timerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[FakeRobot] Started at (%.3f, %.3f, %.3f)", x_, y_, yaw_);
}

void FakeRobot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_vel_ = msg->linear.x;
  angular_vel_ = msg->angular.z;
}

void FakeRobot::timerCallback()
{
  // Calculate dt
  rclcpp::Time now = this->now();
  double dt = (now - last_update_time_).seconds();
  last_update_time_ = now;

  // Integrate velocities
  yaw_ += angular_vel_ * dt;
  x_ += linear_vel_ * std::cos(yaw_) * dt;
  y_ += linear_vel_ * std::sin(yaw_) * dt;

  // Publish TF and pose
  publishTF();
  publishPose();

  // Add pose to traveled path if moved more than 0.05m from last point
  bool should_add = traveled_path_.poses.empty();
  if (!should_add) {
    const auto & last = traveled_path_.poses.back();
    double dx = x_ - last.pose.position.x;
    double dy = y_ - last.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    should_add = (dist > 0.05);
  }

  if (should_add) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    pose.pose.orientation = tf2::toMsg(q);

    traveled_path_.poses.push_back(pose);
  }

  // Publish traveled path
  traveled_path_.header.stamp = now;
  path_pub_->publish(traveled_path_);
}

void FakeRobot::publishTF()
{
  geometry_msgs::msg::TransformStamped odom_to_base;
  odom_to_base.header.stamp = this->now();
  odom_to_base.header.frame_id = "odom";
  odom_to_base.child_frame_id = "base_link";

  odom_to_base.transform.translation.x = x_;
  odom_to_base.transform.translation.y = y_;
  odom_to_base.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  odom_to_base.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(odom_to_base);
}

void FakeRobot::publishPose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x_;
  pose.pose.position.y = y_;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  pose.pose.orientation = tf2::toMsg(q);

  pose_pub_->publish(pose);
}

}  // namespace nav2_operations_test_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_operations_test_nodes::FakeRobot>());
  rclcpp::shutdown();
  return 0;
}
