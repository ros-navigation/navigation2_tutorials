#ifndef NAV2_OPERATIONS_TEST_NODES__CAMERA_SIMULATOR_HPP_
#define NAV2_OPERATIONS_TEST_NODES__CAMERA_SIMULATOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_operations_test_nodes
{

class CameraSimulator : public rclcpp::Node
{
public:
  explicit CameraSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void commandCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void timerCallback();

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double current_yaw_;
  double target_yaw_;
  double rotation_speed_;
  bool rotating_;
  rclcpp::Time last_update_time_;
};

}  // namespace nav2_operations_test_nodes

#endif
