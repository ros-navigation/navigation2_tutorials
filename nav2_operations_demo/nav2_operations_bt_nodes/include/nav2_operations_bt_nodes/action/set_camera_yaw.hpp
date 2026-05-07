// Copyright (c) 2024 Apache-2.0
#ifndef NAV2_OPERATIONS_BT_NODES__ACTION__SET_CAMERA_YAW_HPP_
#define NAV2_OPERATIONS_BT_NODES__ACTION__SET_CAMERA_YAW_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_operations_msgs/action/camera_command.hpp"

namespace nav2_operations_bt_nodes
{

class SetCameraYaw
  : public nav2_behavior_tree::BtActionNode<nav2_operations_msgs::action::CameraCommand>
{
public:
  using Action = nav2_operations_msgs::action::CameraCommand;

  SetCameraYaw(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<Action>(xml_tag_name, action_name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<float>("target_yaw", 0.0f, "Target camera yaw in degrees"),
    });
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;
};

}  // namespace nav2_operations_bt_nodes

#endif  // NAV2_OPERATIONS_BT_NODES__ACTION__SET_CAMERA_YAW_HPP_
