/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://ros-planning.github.io/navigation2/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/PPM.h>
#include <ompl/tools/config/MagicConstants.h>

#include <iostream>
#include <fstream>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_rrtconnect_planner/rrt_connect.hpp"

namespace nav2_rrtconnect_planner
{

RRTConnect::RRTConnect()
: tf_(nullptr), costmap_(nullptr), ss_(nullptr), ompl_state_space_(nullptr), bounds_(nullptr)
{

}

void RRTConnect::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  ompl_initialized_ = false;

  RCLCPP_INFO(
    node_->get_logger(), "Configuring plugin %s of type NavfnPlanner",
    name_.c_str());

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".solve_time",
    rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".solve_time", solve_time_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_trials",
    rclcpp::ParameterValue(4));
  node_->get_parameter(name_ + ".max_trials", max_trials_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".collision_checking_resolution", rclcpp::ParameterValue(
      0.001));
  node_->get_parameter(name_ + ".collision_checking_resolution", collision_checking_resolution_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(
      true));
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
}

void RRTConnect::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTConnect::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTConnect::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRTConnect::initializeOMPL()
{
  // Defining Bounds
  bounds_ = new ompl::base::RealVectorBounds(2);
  bounds_->setLow(0, costmap_->getOriginX());
  bounds_->setHigh(
    0, costmap_->getSizeInMetersX() +
    costmap_->getOriginX());
  bounds_->setLow(1, costmap_->getOriginY());
  bounds_->setHigh(
    1, costmap_->getSizeInMetersY() +
    costmap_->getOriginY());

  RCLCPP_INFO(
    node_->get_logger(),
    "bounds: %f %f %f %f", bounds_->low[0], bounds_->low[1],
    bounds_->high[0], bounds_->high[1]);

  ompl_state_space_ = std::make_shared<ompl::base::SE2StateSpace>();
  // Bounding the R2 state space
  ompl_state_space_->as<ompl::base::SE2StateSpace>()->setBounds(*bounds_);

  // Simple setup
  ss_.reset(new ompl::geometric::SimpleSetup(ompl_state_space_));
  // Setting state validity checker
  ss_->setStateValidityChecker(
    [this](const ompl::base::State * state) {
      return this->isStateValid(state);
    });
  // Setting the state validity checking resolution
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(collision_checking_resolution_);

  ompl_initialized_ = true;
}

nav_msgs::msg::Path RRTConnect::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!ompl_initialized_) {
    RCLCPP_INFO(node_->get_logger(), "Initializing OMPL!");
    initializeOMPL();
  }
  // Returning variable
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  ompl::base::ScopedState<> ss_start(ompl_state_space_), ss_goal(ompl_state_space_);
  ss_start = std::vector<double>{start.pose.position.x, start.pose.position.y};
  ss_goal = std::vector<double>{goal.pose.position.x, goal.pose.position.y};
  // Setting state and goal state
  ss_->setStartAndGoalStates(ss_start, ss_goal);
  // Assigning RRTConnect
  ss_->setPlanner(
    ompl::base::PlannerPtr(
      new ompl::geometric::RRTConnect(
        ss_->getSpaceInformation())));
  // Load settings
  ss_->setup();
  // Counter for replanning
  int counter = 0;
  // Planning
  while (!ss_->solve(solve_time_) && counter < max_trials_) {
    ++counter;
  }
  // Plan found
  if (counter < max_trials_) {
    RCLCPP_INFO(
      node_->get_logger(), "Found %d solutions",
      ss_->getProblemDefinition()->getSolutionCount());
    // Path simplification
    ss_->simplifySolution();
    auto solution_path = ss_->getSolutionPath();

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;
    // Increasing number of path points
    int min_num_states = round(solution_path.length() / costmap_->getResolution());
    solution_path.interpolate(min_num_states);

    global_path.poses.reserve(solution_path.getStates().size());
    for (const auto ptr : solution_path.getStates()) {
      global_path.poses.emplace_back(getPoseStampedFromState(*ptr));
    }
    global_path.poses[global_path.poses.size() - 1] = goal;
  }

  return global_path;
}

bool RRTConnect::isStateValid(const ompl::base::State * state)
{
  if (!ss_->getSpaceInformation()->satisfiesBounds(state)) {
    return false;
  }

  ompl::base::ScopedState<> ss(ompl_state_space_);
  ss = state;
  unsigned int mx, my;
  costmap_->worldToMap(ss[0], ss[1], mx, my);

  auto cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    ( !allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION))
  {
    return false;
  }

  return true;
}

geometry_msgs::msg::PoseStamped RRTConnect::getPoseStampedFromState(const ompl::base::State & state)
{
  double x, y;

  ompl::base::ScopedState<> ss(ompl_state_space_);
  ss = state;

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = ss[0];
  pose.pose.position.y = ss[1];
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  return pose;
}

} // namespace nav2_rrtconnect_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtconnect_planner::RRTConnect, nav2_core::GlobalPlanner)
