// Copyright (c) 2024 Apache-2.0
#include "nav2_operations_servers/camera_server.hpp"

#include <cmath>
#include "rclcpp/rclcpp.hpp"

namespace nav2_operations_servers
{

CameraServer::CameraServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("camera_server", "", options)
{
  declare_parameter("max_rotation_speed", 45.0);
  declare_parameter("tolerance", 0.5);
  declare_parameter("camera_mount_offset", 0.0);
  declare_parameter("action_server_result_timeout", 10.0);
}

CameraServer::CallbackReturn
CameraServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  max_rotation_speed_ = get_parameter("max_rotation_speed").as_double();
  tolerance_ = get_parameter("tolerance").as_double();
  camera_mount_offset_ = get_parameter("camera_mount_offset").as_double();

  camera_pub_ = create_publisher<std_msgs::msg::Float32>("/lawnmower/camera_yaw_command", 10);

  double result_timeout = get_parameter("action_server_result_timeout").as_double();
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(result_timeout);

  action_server_ = std::make_unique<ActionServer>(
    shared_from_this(),
    "camera_server",
    std::bind(&CameraServer::execute, this),
    nullptr,
    std::chrono::milliseconds(500),
    true,
    server_options);

  RCLCPP_INFO(get_logger(), "CameraServer configured (speed: %.1f deg/s)", max_rotation_speed_);
  return CallbackReturn::SUCCESS;
}

CameraServer::CallbackReturn
CameraServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  camera_pub_->on_activate();
  action_server_->activate();
  createBond();
  return CallbackReturn::SUCCESS;
}

CameraServer::CallbackReturn
CameraServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  action_server_->deactivate();
  camera_pub_->on_deactivate();
  destroyBond();
  return CallbackReturn::SUCCESS;
}

CameraServer::CallbackReturn
CameraServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  action_server_.reset();
  camera_pub_.reset();
  return CallbackReturn::SUCCESS;
}

CameraServer::CallbackReturn
CameraServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void
CameraServer::execute()
{
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<CameraCommand::Result>();
  auto feedback = std::make_shared<CameraCommand::Feedback>();

  const float target_yaw = goal->target_yaw;
  const float step = static_cast<float>(max_rotation_speed_) / 20.0f;  // 20 Hz loop

  RCLCPP_INFO(get_logger(), "Camera rotating to %.1f deg", target_yaw);

  rclcpp::WallRate loop_rate(20);

  while (rclcpp::ok()) {
    if (action_server_->is_cancel_requested()) {
      result->success = false;
      action_server_->terminate_current(result);
      return;
    }

    float diff = target_yaw - current_yaw_;
    if (std::abs(diff) <= static_cast<float>(tolerance_)) {
      break;
    }

    current_yaw_ += (diff > 0 ? 1.0f : -1.0f) * std::min(step, std::abs(diff));

    std_msgs::msg::Float32 msg;
    msg.data = current_yaw_;
    camera_pub_->publish(msg);

    feedback->current_yaw = current_yaw_;
    action_server_->publish_feedback(feedback);

    loop_rate.sleep();
  }

  RCLCPP_INFO(get_logger(), "Camera reached %.1f deg", current_yaw_);
  result->success = true;
  action_server_->succeeded_current(result);
}

}  // namespace nav2_operations_servers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_operations_servers::CameraServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
