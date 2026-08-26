// Copyright (c) 2024 Apache-2.0
#include "nav2_operations_bt_nodes/action/set_blade_state.hpp"

namespace nav2_operations_bt_nodes
{

void SetBladeState::on_tick()
{
  bool enable = true;
  getInput("enable", enable);
  goal_.enable = enable;
}

// Use the action result to decide the BT node status
BT::NodeStatus SetBladeState::on_success()
{
  return result_.result->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
// Base-class defaults for reference:
// BT::NodeStatus SetBladeState::on_aborted()   { return BT::NodeStatus::FAILURE; }
// BT::NodeStatus SetBladeState::on_cancelled() { return BT::NodeStatus::SUCCESS; }

}  // namespace nav2_operations_bt_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<nav2_operations_bt_nodes::SetBladeState>(
        name, "blade_server", config);
    };
  factory.registerBuilder<nav2_operations_bt_nodes::SetBladeState>(
    "SetBladeState", builder);
}
