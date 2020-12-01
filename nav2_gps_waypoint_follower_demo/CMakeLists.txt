cmake_minimum_required(VERSION 3.5)
project(nav2_gps_waypoint_follower_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_gps_waypoint_follower REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(rosidl_default_generators REQUIRED)

nav2_package()

include_directories(
  include)

set(gps_client_executable_name gps_waypoint_follower_demo)

add_executable(${gps_client_executable_name}
  src/gps_waypoint_follower_demo.cpp)
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_util
  nav2_lifecycle_manager
  nav_msgs
  nav2_msgs
  nav2_core
  nav2_gps_waypoint_follower
  tf2_ros
  robot_localization
)

ament_target_dependencies(${gps_client_executable_name}
  ${dependencies})

target_link_libraries(${gps_client_executable_name})
install(TARGETS ${gps_client_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()