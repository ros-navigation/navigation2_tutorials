// Copyright (c) 2024 Apache-2.0
#include <string>
#include <memory>

#include "nav2_operations_navigator/navigate_with_operations.hpp"

namespace nav2_operations_navigator
{

std::string
NavigateWithOperations::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_navigate_with_operations_bt_xml")) {
    node->declare_parameter<std::string>("default_navigate_with_operations_bt_xml", "");
  }

  node->get_parameter("default_navigate_with_operations_bt_xml", default_bt_xml_filename);
  return default_bt_xml_filename;
}

bool
NavigateWithOperations::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/)
{
  auto node = parent_node.lock();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter<std::string>("path_blackboard_id", "path");
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();
  clock_ = node->get_clock();

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());

  return true;
}

bool
NavigateWithOperations::cleanup()
{
  return true;
}

bool
NavigateWithOperations::goalReceived(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Navigation canceled.", bt_xml_filename.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger_, "NavigateWithOperations goal received with %zu path poses",
    goal->path.poses.size());

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, goal->path);

  active_goal_ = true;
  goal_start_time_ = clock_->now();
  return true;
}

void
NavigateWithOperations::onLoop()
{
  auto feedback = std::make_shared<ActionT::Feedback>();

  feedback->navigation_time = clock_->now() - goal_start_time_;

  // Read a value from the BT blackboard and publish it as feedback.
  // The path is written here once by goalReceived(); a BT action node
  // could write a separate progress key that gets read here instead.
  auto blackboard = bt_action_server_->getBlackboard();
  nav_msgs::msg::Path current_path;
  if (blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path)) {
    feedback->path_poses_count = static_cast<uint32_t>(current_path.poses.size());
  }

  bt_action_server_->publishFeedback(feedback);
}

void
NavigateWithOperations::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const nav2_behavior_tree::BtStatus final_bt_status)
{
  active_goal_ = false;

  if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
    result->error_code = 0;
  } else if (final_bt_status == nav2_behavior_tree::BtStatus::FAILED) {
    result->error_code = 1;
  } else {
    result->error_code = 2;
  }

  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, nav_msgs::msg::Path());
}

void
NavigateWithOperations::onPreempt(
  typename ActionT::Goal::ConstSharedPtr goal)
{
  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    auto pending_goal = bt_action_server_->acceptPendingGoal();
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<nav_msgs::msg::Path>(path_blackboard_id_, pending_goal->path);
  } else {
    bt_action_server_->terminatePendingGoal();
  }
}

}  // namespace nav2_operations_navigator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_operations_navigator::NavigateWithOperations,
  nav2_core::NavigatorBase)
