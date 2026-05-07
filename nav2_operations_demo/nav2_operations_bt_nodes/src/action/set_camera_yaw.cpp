// Copyright (c) 2024 Apache-2.0
#include "nav2_operations_bt_nodes/action/set_camera_yaw.hpp"

namespace nav2_operations_bt_nodes
{

void SetCameraYaw::on_tick()
{
  float target_yaw = 0.0f;
  getInput("target_yaw", target_yaw);
  goal_.target_yaw = target_yaw;
}

// Use the action result to decide the BT node status
BT::NodeStatus SetCameraYaw::on_success()
{
  return result_.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
// Base-class defaults for reference:
// BT::NodeStatus SetCameraYaw::on_aborted()   { return BT::NodeStatus::FAILURE; }
// BT::NodeStatus SetCameraYaw::on_cancelled() { return BT::NodeStatus::SUCCESS; }

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::SetCameraYaw>(
        name, "camera_server", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::SetCameraYaw>(
    "SetCameraYaw", builder);
}
