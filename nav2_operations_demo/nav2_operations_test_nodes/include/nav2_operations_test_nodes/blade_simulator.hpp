#ifndef NAV2_OPERATIONS_TEST_NODES__BLADE_SIMULATOR_HPP_
#define NAV2_OPERATIONS_TEST_NODES__BLADE_SIMULATOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_operations_test_nodes
{

class BladeSimulator : public rclcpp::Node
{
public:
  explicit BladeSimulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void commandCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool blade_state_;
  int marker_id_{0};
};

}  // namespace nav2_operations_test_nodes

#endif
