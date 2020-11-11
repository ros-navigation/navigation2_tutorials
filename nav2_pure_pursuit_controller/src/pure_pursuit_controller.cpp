/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_pure_pursuit_controller
{

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
      0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
      1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
      0.1));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pure_pursuit_controller::PurePursuitController",
    plugin_name_.c_str());
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
}


geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &)
{
  auto transformed_plan = transformGlobalPlan(pose);

  // First find the closest pose on the path to the robot
  auto closest_pose_it =
    min_by(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [](const geometry_msgs::msg::PoseStamped & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y);
    });

  // From that point, find the first pose which is at a distance greater than the specified lookahed
  // distance
  auto goal_pose_it = std::find_if(
    closest_pose_it, transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist_;
    });

  // If the last pose is still within lookahed distance, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  auto goal_pose = goal_pose_it->pose;

  double linear_vel, angular_vel;

  // If the goal pose is in front of the robot then compute the velocity using the pure pursuit
  // algorithm, else rotate with the max angular velocity until the goal pose is in front of the
  // robot
  if (goal_pose.position.x > 0) {
    auto curvature = 2.0 * goal_pose.position.y /
      (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
    linear_vel = desired_linear_vel_;
    angular_vel = desired_linear_vel_ * curvature;
  } else {
    linear_vel = 0.0;
    angular_vel = max_angular_vel_;
  }

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = max(
    -1.0 * abs(max_angular_vel_), min(
      angular_vel, abs(
        max_angular_vel_)));

  return cmd_vel;
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // Compute distance threshold of points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double transform_dist_threshold =
    std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Helper function for the transform below. Converts a pose from global
  // frame to robot's frame
  auto transformGlobalPoseToRobotFrame = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, transformed_plan.header.frame_id,
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  std::transform(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToRobotFrame);

  auto transformation_end = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(),
    [&](const auto & global_plan_pose) {
      return hypot(
        global_plan_pose.pose.position.x,
        global_plan_pose.pose.position.y) > transform_dist_threshold;
    });

  // Discard points on the plan that are outside the local costmap
  transformed_plan.poses.erase(transformation_end, transformed_plan.poses.end());

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PurePursuitController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  rclcpp::Duration & transform_tolerance
)
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
