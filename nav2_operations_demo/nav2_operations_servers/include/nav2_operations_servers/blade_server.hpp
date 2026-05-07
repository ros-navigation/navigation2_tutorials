// Copyright (c) 2024 Apache-2.0
#ifndef NAV2_OPERATIONS_SERVERS__BLADE_SERVER_HPP_
#define NAV2_OPERATIONS_SERVERS__BLADE_SERVER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_operations_msgs/action/blade_command.hpp"

namespace nav2_operations_servers
{

class BladeServer : public nav2_util::LifecycleNode
{
public:
  using BladeCommand = nav2_operations_msgs::action::BladeCommand;
  using ActionServer = nav2_util::SimpleActionServer<BladeCommand>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit BladeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  void execute();

  std::unique_ptr<ActionServer> action_server_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr blade_pub_;
  std::string blade_command_topic_;
};

}  // namespace nav2_operations_servers

#endif  // NAV2_OPERATIONS_SERVERS__BLADE_SERVER_HPP_
