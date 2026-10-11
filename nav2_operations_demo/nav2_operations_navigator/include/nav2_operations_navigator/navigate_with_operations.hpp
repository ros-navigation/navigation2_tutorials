// Copyright (c) 2024 Apache-2.0
#ifndef NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_
#define NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_operations_msgs/action/navigate_with_operations.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_operations_navigator
{

class NavigateWithOperations
  : public nav2_core::BehaviorTreeNavigator<
      nav2_operations_msgs::action::NavigateWithOperations>
{
public:
  using ActionT = nav2_operations_msgs::action::NavigateWithOperations;

  NavigateWithOperations()
  : BehaviorTreeNavigator() {}

  std::string getName() override { return std::string("navigate_with_operations"); }

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  bool cleanup() override;

  void pause() {}
  void resume() {}

protected:
  bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;
  void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) override;

  std::string path_blackboard_id_;
  bool active_goal_{false};
  rclcpp::Time goal_start_time_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_operations_navigator

#endif  // NAV2_OPERATIONS_NAVIGATOR__NAVIGATE_WITH_OPERATIONS_HPP_
