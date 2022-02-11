// Copyright (c) 2020 Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_GPS_WAYPOINT_FOLLOWER_DEMO__GPS_WAYPOINT_FOLLOWER_DEMO_HPP_
#define NAV2_GPS_WAYPOINT_FOLLOWER_DEMO__GPS_WAYPOINT_FOLLOWER_DEMO_HPP_

#include <vector>
#include <string>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_gps_waypoints.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
/**
 * @brief namespace for way point following, points are from a yaml file
 *
 */
namespace nav2_gps_waypoint_follower_demo
{
enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

/**
 * @brief A ros node that drives robot through gievn way points from YAML file
 *
 */
class GPSWayPointFollowerClient : public rclcpp::Node
{
public:
  using ClientT = nav2_msgs::action::FollowGPSWaypoints;
  // shorten the Goal handler Client type
  using GPSWaypointFollowerGoalHandle =
    rclcpp_action::ClientGoalHandle<ClientT>;

  /**
   * @brief Construct a new WayPoint Folllower Demo object
   *
   */
  GPSWayPointFollowerClient();

  /**
   * @brief Destroy the Way Point Folllower Demo object
   *
   */
  ~GPSWayPointFollowerClient();

  /**
   * @brief send robot through each of the pose in poses vector
   *
   * @param poses
   */
  void startWaypointFollowing();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool is_goal_done() const;

  /**
 * @brief given a parameter name on the yaml file, loads this parameter as geographic_msgs::msg::GeoPose
 *  Note that this parameter needs to be an array of doubles
 *
 * @return geographic_msgs::msg::GeoPose
 */
  std::vector<geographic_msgs::msg::GeoPose>
  loadGPSWaypointsFromYAML();

  void goalResponseCallback(GPSWaypointFollowerGoalHandle::SharedPtr goal_handle);

  void feedbackCallback(
    GPSWaypointFollowerGoalHandle::SharedPtr,
    const std::shared_ptr<const ClientT::Feedback> feedback);

  void resultCallback(const GPSWaypointFollowerGoalHandle::WrappedResult & result);

protected:
  bool goal_done_;
  rclcpp::TimerBase::SharedPtr timer_;
  // client to connect waypoint follower service(FollowWaypoints)
  rclcpp_action::Client<ClientT>::SharedPtr
    gps_waypoint_follower_action_client_;

  // goal handler to query state of goal
  ClientT::Goal gps_waypoint_follower_goal_;

  GPSWaypointFollowerGoalHandle::SharedPtr gps_waypoint_follower_goalhandle_;

  std::vector<geographic_msgs::msg::GeoPose> gps_poses_from_yaml_;
};
}  // namespace nav2_gps_waypoint_follower_demo

#endif  // NAV2_GPS_WAYPOINT_FOLLOWER_DEMO__GPS_WAYPOINT_FOLLOWER_DEMO_HPP_
