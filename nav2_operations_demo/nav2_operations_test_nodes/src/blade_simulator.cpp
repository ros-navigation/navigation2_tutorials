#include "nav2_operations_test_nodes/blade_simulator.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_operations_test_nodes
{

BladeSimulator::BladeSimulator(const rclcpp::NodeOptions & options)
: Node("blade_simulator", options),
  blade_state_(false)
{
  // Subscriber for blade command
  command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/lawnmower/blade_command", 10,
    std::bind(&BladeSimulator::commandCallback, this, std::placeholders::_1));

  // Publisher for blade state
  state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/lawnmower/blade_state", 10);

  // Publisher for blade marker
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/lawnmower/blade_marker", 10);

  // TF buffer and listener to look up robot position in map frame
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "[BladeSimulator] Started");
}

void BladeSimulator::commandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  blade_state_ = msg->data;

  if (blade_state_) {
    RCLCPP_INFO(this->get_logger(), "[BladeSimulator] Blade turned ON");
  } else {
    RCLCPP_INFO(this->get_logger(), "[BladeSimulator] Blade turned OFF");
  }

  // Publish current blade state
  std_msgs::msg::Bool state_msg;
  state_msg.data = blade_state_;
  state_pub_->publish(state_msg);

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
      "[BladeSimulator] Could not get base_link->map transform: %s", ex.what());
  }

  // Publish blade state marker at current robot position
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "blade_simulator";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = robot_x;
  marker.pose.position.y = robot_y;
  marker.pose.position.z = 0.1;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.05;
  marker.color.r = blade_state_ ? 0.0f : 1.0f;
  marker.color.g = blade_state_ ? 1.0f : 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker_pub_->publish(marker);
}

}  // namespace nav2_operations_test_nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_operations_test_nodes::BladeSimulator>());
  rclcpp::shutdown();
  return 0;
}
