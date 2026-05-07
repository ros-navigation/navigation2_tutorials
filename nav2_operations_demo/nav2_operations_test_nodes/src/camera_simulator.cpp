#include "nav2_operations_test_nodes/camera_simulator.hpp"

#include <cmath>
#include <chrono>

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_operations_test_nodes
{

CameraSimulator::CameraSimulator(const rclcpp::NodeOptions & options)
: Node("camera_simulator", options),
  current_yaw_(0.0),
  target_yaw_(0.0),
  rotating_(false)
{
  // Declare and get parameters
  this->declare_parameter("rotation_speed", 45.0);
  this->declare_parameter("update_rate", 20.0);

  rotation_speed_ = this->get_parameter("rotation_speed").as_double();
  double update_rate = this->get_parameter("update_rate").as_double();

  // Subscriber for camera yaw command
  command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/lawnmower/camera_yaw_command", 10,
    std::bind(&CameraSimulator::commandCallback, this, std::placeholders::_1));

  // Publisher for camera yaw state
  state_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/lawnmower/camera_yaw_state", 10);

  // Publisher for camera marker
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/lawnmower/camera_marker", 10);

  // TF buffer and listener to look up robot position in map frame
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize time
  last_update_time_ = this->now();

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CameraSimulator::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "[CameraSimulator] Started");
}

void CameraSimulator::commandCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  target_yaw_ = static_cast<double>(msg->data);
  rotating_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "[CameraSimulator] Received command to rotate to %.1f deg", target_yaw_);
}

void CameraSimulator::timerCallback()
{
  rclcpp::Time now = this->now();

  // Look up current robot position in map frame
  double robot_x = 0.0;
  double robot_y = 0.0;
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
    robot_x = tf.transform.translation.x;
    robot_y = tf.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "[CameraSimulator] Could not get base_link->map transform: %s", ex.what());
  }

  if (!rotating_) {
    // Just publish current state
    std_msgs::msg::Float32 state_msg;
    state_msg.data = static_cast<float>(current_yaw_);
    state_pub_->publish(state_msg);
    last_update_time_ = now;

    // Publish camera direction marker at current robot position
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.ns = "camera_simulator";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = robot_x;
    marker.pose.position.y = robot_y;
    marker.pose.position.z = 0.2;
    double yaw_rad = current_yaw_ * M_PI / 180.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = std::sin(yaw_rad / 2.0);
    marker.pose.orientation.w = std::cos(yaw_rad / 2.0);
    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_pub_->publish(marker);
    return;
  }

  // Calculate dt
  double dt = (now - last_update_time_).seconds();
  last_update_time_ = now;

  // Move current_yaw_ toward target_yaw_ by rotation_speed_ * dt
  double diff = target_yaw_ - current_yaw_;
  double step = rotation_speed_ * dt;

  if (std::abs(diff) <= 0.5) {
    // Within 0.5 degrees of target
    current_yaw_ = target_yaw_;
    rotating_ = false;
    RCLCPP_INFO(
      this->get_logger(),
      "[CameraSimulator] Reached target %.1f deg", target_yaw_);
  } else {
    // Move toward target
    if (diff > 0.0) {
      current_yaw_ += std::min(step, diff);
    } else {
      current_yaw_ -= std::min(step, -diff);
    }

    // Log approximately every 1 second (check if we crossed a second boundary)
    static rclcpp::Time last_log_time = now;
    if ((now - last_log_time).seconds() >= 1.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "[CameraSimulator] Rotating to %.1f deg, current: %.1f deg",
        target_yaw_, current_yaw_);
      last_log_time = now;
    }
  }

  // Publish current yaw state
  std_msgs::msg::Float32 state_msg;
  state_msg.data = static_cast<float>(current_yaw_);
  state_pub_->publish(state_msg);

  // Publish camera direction marker at current robot position
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now;
  marker.ns = "camera_simulator";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = robot_x;
  marker.pose.position.y = robot_y;
  marker.pose.position.z = 0.2;
  double yaw_rad = current_yaw_ * M_PI / 180.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(yaw_rad / 2.0);
  marker.pose.orientation.w = std::cos(yaw_rad / 2.0);
  marker.scale.x = 0.5;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.0f;
  marker.color.g = 0.5f;
  marker.color.b = 1.0f;
  marker.color.a = 0.8f;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker_pub_->publish(marker);
}

}  // namespace nav2_operations_test_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_operations_test_nodes::CameraSimulator>());
  rclcpp::shutdown();
  return 0;
}
