// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

#include <string>
#include <utility>
#include <fstream>
#include <ostream>
#include <iostream>
#include <memory>
#include "nav2_gps_waypoint_follower_demo/gps_waypoint_logger.hpp"

namespace nav2_gps_waypoint_follower_demo
{

GPSWaypointLogger::GPSWaypointLogger()
: Node("gps_waypoint_logger_rclcpp_node"), is_first_msg_recieved_(false)
{
  declare_parameter("frequency", 10);
  declare_parameter("yaml_file_out", "/home/user_name/logged_waypoints.yaml");

  get_parameter("frequency", frequency_);
  get_parameter("yaml_file_out", yaml_file_out_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / frequency_),
    std::bind(&GPSWaypointLogger::timerCallback, this));

  navsat_fix_subscriber_.subscribe(this, "/gps", rmw_qos_profile_sensor_data);
  imu_subscriber_.subscribe(this, "/imu", rmw_qos_profile_sensor_data);

  geopose_publisher_ = this->create_publisher<geographic_msgs::msg::GeoPose>(
    "logged_gps_waypoints", rclcpp::SystemDefaultsQoS());

  sensor_data_approx_time_syncher_.reset(
    new SensorDataApprxTimeSyncer(
      SensorDataApprxTimeSyncPolicy(10), navsat_fix_subscriber_,
      imu_subscriber_));

  sensor_data_approx_time_syncher_->registerCallback(
    std::bind(
      &GPSWaypointLogger::sensorDataCallback, this, std::placeholders::_1,
      std::placeholders::_2));
}

GPSWaypointLogger::~GPSWaypointLogger()
{
  dumpLoggedWaypoints();
}

void GPSWaypointLogger::timerCallback()
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Entering to timer callback, this is periodicly called");
  if (is_first_msg_recieved_) {
    std::lock_guard<std::mutex> guard(global_mutex_);
    tf2::Quaternion q(
      reusable_imu_msg_.orientation.x,
      reusable_imu_msg_.orientation.y,
      reusable_imu_msg_.orientation.z,
      reusable_imu_msg_.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      this->get_logger(),
      "curr_gps_waypoint: [%.8f, %.8f, %.8f, %.8f]", reusable_navsat_msg_.latitude,
      reusable_navsat_msg_.longitude, reusable_navsat_msg_.altitude, yaw);

    std::vector<double> curr_waypoint_data =
    {reusable_navsat_msg_.latitude,
      reusable_navsat_msg_.longitude,
      reusable_navsat_msg_.altitude, yaw};

    logged_waypoints_vector_.push_back(curr_waypoint_data);
    geographic_msgs::msg::GeoPose curr_gps_waypoint;
    curr_gps_waypoint.position.latitude = reusable_navsat_msg_.latitude;
    curr_gps_waypoint.position.longitude = reusable_navsat_msg_.longitude;
    curr_gps_waypoint.position.altitude = reusable_navsat_msg_.altitude;
    curr_gps_waypoint.orientation = reusable_imu_msg_.orientation;
    geopose_publisher_->publish(curr_gps_waypoint);
  }
}

void GPSWaypointLogger::sensorDataCallback(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gps,
  const sensor_msgs::msg::Imu::ConstSharedPtr & imu)
{
  std::lock_guard<std::mutex> guard(global_mutex_);
  reusable_navsat_msg_ = *gps;
  reusable_imu_msg_ = *imu;
  is_first_msg_recieved_ = true;
}

void GPSWaypointLogger::dumpLoggedWaypoints()
{
  YAML::Emitter waypoints;
  waypoints << YAML::BeginMap;
  waypoints << YAML::Key << "waypoints" << YAML::Value << "see all points logged here";
  for (size_t i = 0; i < logged_waypoints_vector_.size(); i++) {
    std::string wp_name = "wp" + std::to_string(i);
    waypoints << YAML::Key << wp_name << YAML::Value << wp_name;
    waypoints << YAML::BeginSeq <<
      logged_waypoints_vector_[i][0] <<
      logged_waypoints_vector_[i][1] <<
      logged_waypoints_vector_[i][2] <<
      logged_waypoints_vector_[i][3] << YAML::EndSeq;
  }
  waypoints << YAML::EndMap;
  std::ofstream fout(yaml_file_out_, std::ofstream::out);
  fout << waypoints.c_str();
  fout.close();
}

}  // namespace nav2_gps_waypoint_follower_demo

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_gps_waypoint_follower_demo::GPSWaypointLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
