// Copyright (c) 2024 Apache-2.0
#ifndef NAV2_OPERATIONS_SERVERS__CAMERA_SERVER_HPP_
#define NAV2_OPERATIONS_SERVERS__CAMERA_SERVER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav2_operations_msgs/action/camera_command.hpp"

namespace nav2_operations_servers
{

class CameraServer : public nav2_util::LifecycleNode
{
public:
  using CameraCommand = nav2_operations_msgs::action::CameraCommand;
  using ActionServer = nav2_util::SimpleActionServer<CameraCommand>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CameraServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void execute();

  std::unique_ptr<ActionServer> action_server_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr camera_pub_;

  double max_rotation_speed_;
  double tolerance_;
  double camera_mount_offset_;
  float current_yaw_{0.0f};
};

}  // namespace nav2_operations_servers

#endif  // NAV2_OPERATIONS_SERVERS__CAMERA_SERVER_HPP_
