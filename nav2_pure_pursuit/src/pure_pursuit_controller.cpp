/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Author(s): Steve Macenski <stevenmacenski@gmail.com>
 *
 */

#include <algorithm>
#include <string>
#include <memory>
#include <algorithm>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_pure_pursuit/pure_pursuit_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

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
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  last_cmd_.header.stamp = clock_->now();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_decel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_decel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist",
    rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist",
    rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_gain",
    rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(
      false));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_linear_decel", max_linear_decel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_decel", max_angular_decel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_gain", lookahead_gain_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pure_pursuit_controller::PurePursuitController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_deactivate();
}

double PurePursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the default look ahead distance
  double lookahead_dist = 0.0;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = speed.linear.x * lookahead_gain_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  } else {
    lookahead_dist = lookahead_dist_;
  }

  return lookahead_dist;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed)
{
  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path
  const double lookahead_dist = getLookAheadDistance(speed);
  geometry_msgs::msg::PoseStamped carrot_pose = getLookAheadMarker(lookahead_dist, transformed_plan);

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circular arc
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Apply curvature to angular velocity
  linear_vel = desired_linear_vel_;
  angular_vel = desired_linear_vel_ * curvature;

  // Make sure we're in compliance with basic kinematics
  rclcpp::Duration dt = pose.header.stamp - last_cmd_.header.stamp;
  applyKinematicConstraints(
    linear_vel, angular_vel,
    fabs(lookahead_dist - sqrt(carrot_dist2)), lookahead_dist, dt.seconds());

  // Collision checking
  if (isCollisionImminent(pose, carrot_pose)) {
    RCLCPP_ERROR(logger_, "Collision imminent!");
    throw std::runtime_error("PurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  last_cmd_ = cmd_vel;
  return cmd_vel;
}

geometry_msgs::msg::PoseStamped PurePursuitController::getLookAheadMarker(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}

bool PurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // This may be a bit unusual, but the robot_pose is in odom frame
  // and the carrot_pose is in robot base frame. We need to collision
  // check in odom frame, so all values will be relative to robot base pose.
  // But we can still use the carrot pose to find quantities and convert to odom.

  const double carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  const unsigned int num_pts = static_cast<unsigned int>(ceil(carrot_dist / costmap_->getResolution()));
  const double pt_distance = carrot_dist / num_pts;

  geometry_msgs::msg::PoseStamped carrot_in_odom;
  try {
    tf_->transform(carrot_pose, carrot_in_odom,
      costmap_ros_->getGlobalFrameID(), transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Exception attempting to get look ahead point in odom frame: %s", ex.what());
  }

  geometry_msgs::msg::Vector3 unit_vector;
  unit_vector.x = carrot_in_odom.pose.position.x / carrot_dist;
  unit_vector.y = carrot_in_odom.pose.position.y / carrot_dist;

  double curr_dist;
  geometry_msgs::msg::Vector3 cur_pt;

  for (unsigned int i = 0; i != num_pts; i++) {
    curr_dist = i * pt_distance;
    cur_pt.x = robot_pose.pose.position.x + (curr_dist * unit_vector.x);
    cur_pt.y = robot_pose.pose.position.y + (curr_dist * unit_vector.y);
    if (inCollision(cur_pt.x, cur_pt.y)) {
      return true;
    }
  }

  return false;
}

bool PurePursuitController::inCollision(const double & x, const double & y)
{
  using namespace nav2_costmap_2d;  // NOLINT

  unsigned int mx, my;
  costmap_->worldToMap(x, y, mx, my);

  unsigned char cost = costmap_->getCost(mx, my);

  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION;
  } else {
    return cost >= INSCRIBED_INFLATED_OBSTACLE;
  }
}

void PurePursuitController::applyKinematicConstraints(
  double & linear_vel, double & angular_vel,
  const double & dist_error, const double & lookahead_dist, const double & dt)
{
  // if the actual lookahead distance is shorter than requested, that means we're at the
  // end of the path. We'll scale linear velocity by error to slow to a smooth stop
  if (dist_error > 2.0 * costmap_->getResolution()) {
    linear_vel = linear_vel * (dist_error / lookahead_dist);
  }

  // if we're accelerating or decelerating too fast, limit linear velocity
  double measured_lin_accel = (linear_vel - last_cmd_.twist.linear.x) / dt;
  if (measured_lin_accel > max_linear_accel_) {
    linear_vel = last_cmd_.twist.linear.x + max_linear_accel_ * dt;
  } else if (measured_lin_accel < -max_linear_decel_) {
    linear_vel = last_cmd_.twist.linear.x - max_linear_decel_ * dt;
  }

  // if we're accelerating or decelerating too fast, limit angular velocity
  double measured_ang_accel = (angular_vel - last_cmd_.twist.angular.z) / dt;
  if (measured_ang_accel > max_angular_accel_) {
    angular_vel = last_cmd_.twist.angular.z + max_angular_accel_ * dt;
  } else if (measured_ang_accel < -max_angular_decel_) {
    angular_vel = last_cmd_.twist.angular.z - max_angular_decel_ * dt;
  }

  // make sure in range of valid velocities
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
  linear_vel = std::clamp(linear_vel, 0.0, desired_linear_vel_);
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

nav_msgs::msg::Path
PurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist =  max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool PurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
