// Copyright (c) 2024 Apache-2.0
#include "nav2_operations_servers/blade_server.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_operations_servers
{

BladeServer::BladeServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("blade_server", "", options)
{
  declare_parameter("blade_command_topic", std::string("/lawnmower/blade_command"));
  declare_parameter("action_server_result_timeout", 10.0);
}

BladeServer::CallbackReturn
BladeServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  blade_command_topic_ = get_parameter("blade_command_topic").as_string();

  blade_pub_ = create_publisher<std_msgs::msg::Bool>(blade_command_topic_, 10);

  double result_timeout = get_parameter("action_server_result_timeout").as_double();
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(result_timeout);

  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "blade_server",
    std::bind(&BladeServer::execute, this),
    nullptr,
    std::chrono::milliseconds(500),
    true,
    server_options);

  RCLCPP_INFO(get_logger(), "BladeServer configured (topic: %s)", blade_command_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

BladeServer::CallbackReturn
BladeServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  blade_pub_->on_activate();
  action_server_->activate();
  createBond();
  return CallbackReturn::SUCCESS;
}

BladeServer::CallbackReturn
BladeServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  action_server_->deactivate();
  blade_pub_->on_deactivate();
  destroyBond();
  return CallbackReturn::SUCCESS;
}

BladeServer::CallbackReturn
BladeServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  action_server_.reset();
  blade_pub_.reset();
  return CallbackReturn::SUCCESS;
}

BladeServer::CallbackReturn
BladeServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void
BladeServer::execute()
{
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<BladeCommand::Result>();

  std_msgs::msg::Bool msg;
  msg.data = goal->enable;
  blade_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Blade command: %s", goal->enable ? "ON" : "OFF");

  result->success = true;
  action_server_->succeeded_current(result);
}

}  // namespace nav2_operations_servers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_operations_servers::BladeServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
