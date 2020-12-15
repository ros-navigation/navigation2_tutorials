/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Shrijit Singh
 *  Copyright (c) 2020, Samsung Research America
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <string>
#include <memory>
#include <algorithm>

#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

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
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_accel", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_decel", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_vel_scaling", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_approach_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_accel", max_linear_accel_);
  node->get_parameter(plugin_name_ + ".max_linear_decel", max_linear_decel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_approach_vel_scaling", min_approach_vel_scaling_); // TODO use an absolute velocity rather than a %?
  node->get_parameter(plugin_name_ + ".use_approach_linear_velocity_scaling", use_approach_vel_scaling_);
  node->get_parameter(plugin_name_ + ".max_allowed_time_to_collision", max_allowed_time_to_collision_);
  node->get_parameter(plugin_name_ + ".use_regulated_linear_velocity_scaling", use_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_speed", regulated_linear_scaling_min_speed_);
  node->get_parameter("control_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type pure_pursuit_controller::PurePursuitController",
    plugin_name_.c_str());
  global_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type pure_pursuit_controller::PurePursuitController\"  %s",
    plugin_name_.c_str());
  global_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
}

double PurePursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = 0.0;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = speed.linear.x * lookahead_time_;
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

  // Find look ahead distance and point on path and publish
  const double lookahead_dist = getLookAheadDistance(speed);
  auto carrot_pose = getLookAheadMarker(lookahead_dist, transformed_plan);
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  carrot_pub_->publish(std::move(carrot_msg));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Apply curvature to angular velocity
  linear_vel = desired_linear_vel_;
  angular_vel = desired_linear_vel_ * curvature;

  // Make sure we're in compliance with basic constraints
  applyConstraints(
    linear_vel, angular_vel,
    fabs(lookahead_dist - sqrt(carrot_dist2)), lookahead_dist, curvature, speed,
    costAtPose(pose.pose.position.x, pose.pose.position.y));

  // Collision checking on this velocity heading
  if (isCollisionImminent(pose, carrot_pose, curvature, linear_vel, angular_vel)) {
    RCLCPP_ERROR(logger_, "Collision imminent!");
    throw std::runtime_error("PurePursuitController detected collision ahead!");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
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
  const geometry_msgs::msg::PoseStamped & carrot_pose,
  const double & curvature, const double & linear_vel,
  const double & angular_vel)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose is in robot base frame. We need to collision
  // check in odom frame, so all values will be relative to robot base pose.
  // But we can still use the carrot pose in odom to find various quantities.

  geometry_msgs::msg::PoseStamped carrot_in_odom;
  if (!transformPose(costmap_ros_->getGlobalFrameID(), carrot_pose, carrot_in_odom))
  {
    RCLCPP_ERROR(logger_, "Unable to get carrot pose in odom frame, failing collision check!");
    return true;
  }

  // check current point and carrot point are OK, most likely ones to be in collision
  if (inCollision(carrot_in_odom.pose.position.x, carrot_in_odom.pose.position.y) ||
    inCollision(robot_pose.pose.position.x, robot_pose.pose.position.y))
  {
    return true;
  }

  // debug messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = clock_->now();
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  // Using curvature (k = 1 / R) and carrot distance (chord on circle R), project the command
  const double chord_len = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);

  // Find the number of increments by finding the arc length of the chord on circle
  const double r = fabs(1.0 / curvature);
  const double alpha = 2.0 * asin(chord_len / (2 * r)); // central angle
  const double arc_len = alpha * r;
  const double delta_dist = costmap_->getResolution();
  const unsigned int num_pts = static_cast<unsigned int>(ceil(arc_len / delta_dist));
  const double projection_time = costmap_->getResolution() / linear_vel;

  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  for (unsigned int i = 1; i < num_pts; i++) {
    // only forward simulate within time requested
    if (i * projection_time > max_allowed_time_to_collision_) {
      break;
    }

    // apply velocity at curr_pose over distance delta_dist
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at this point
    if (inCollision(curr_pose.x, curr_pose.y)) {
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool PurePursuitController::inCollision(const double & x, const double & y)
{
  unsigned int mx, my;
  costmap_->worldToMap(x, y, mx, my);

  unsigned char cost = costmap_->getCost(mx, my);

  if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown()) {
    return cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION;
  } else {
    return cost >= INSCRIBED_INFLATED_OBSTACLE;
  }
}

double PurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;
  costmap_->worldToMap(x, y, mx, my);

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

void PurePursuitController::applyConstraints(
  double & linear_vel, double & angular_vel,
  const double & dist_error, const double & lookahead_dist,
  const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
  const double & pose_cost)
{
  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    std::cout << radius;
    linear_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
    if (linear_vel < regulated_linear_scaling_min_speed_) {
      linear_vel = regulated_linear_scaling_min_speed_;
    }
  }

  // limit the linear velocity by proximity to obstacles
  if (use_regulated_linear_velocity_scaling_ && pose_cost != static_cast<double>(NO_INFORMATION)) {
    // Note(stevemacenski): We can use the cost at a pose as a derived value proportional to
    // distance to obstacle since we lack a distance map. [0-128] is the freespace costed
    // range, above 128 is possibly inscribed, so it should max out at 128 to be a minimum speed
    // in when in questionable states. This creates a linear mapping of
    // cost [0, 128] to speed [min regulated speed, linear vel]
    double max_non_collision = 128.0
    if (pose_cost > max_non_collision) {
      linear_vel = regulated_linear_scaling_min_speed_;
    } else {
      const double slope = (regulated_linear_scaling_min_speed_ - linear_vel) / max_non_collision;
      linear_vel = slope * pose_cost + linear_vel;
    }
  }

  // if the actual lookahead distance is shorter than requested, that means we're at the
  // end of the path. We'll scale linear velocity by error to slow to a smooth stop
  if (use_approach_vel_scaling_ && dist_error > 2.0 * costmap_->getResolution()) {
    double velocity_scaling = 1.0 - (dist_error / lookahead_dist);
    if (velocity_scaling < min_approach_vel_scaling_) {
      velocity_scaling = min_approach_vel_scaling_;
    }
    linear_vel = linear_vel * velocity_scaling;
  }

  // Limit linear velocities to be kinematically feasible, v = v0 + a * dt
  double & dt = control_duration_;
  const double max_feasible_linear_speed = curr_speed.linear.x + max_linear_accel_ * dt;
  const double min_feasible_linear_speed = curr_speed.linear.x - max_linear_decel_ * dt;
  linear_vel = std::clamp(linear_vel, min_feasible_linear_speed, max_feasible_linear_speed);

  // Note(stevemacenski): for the moment, no smoothing on angular velocities, we find the existing
  // commands are very smooth and we don't want to artifically reduce them if the hardware
  // can handle it. Plus deviating from the commands here can be collision-inducing if users
  // don't properly set these values, so hiding that complexity from them that would likely
  // become the #1 cause of issues.
  // const double max_feasible_angular_speed = curr_speed.angular.z + angular_accel_ * dt;
  // const double min_feasible_angular_speed = curr_speed.angular.z - angular_accel_ * dt;
  // angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Limit range to generally valid velocities
  linear_vel = std::clamp(linear_vel, 0.0, desired_linear_vel_);
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
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
