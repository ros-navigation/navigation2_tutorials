cmake_minimum_required(VERSION 3.5)
project(nav2_operations_navigator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_operations_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(${PROJECT_NAME} SHARED
  src/navigate_with_operations.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  nav2_core
  nav2_util
  nav2_behavior_tree
  nav_msgs
  nav2_operations_msgs
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  nav2_core
  nav2_util
  nav2_behavior_tree
  nav_msgs
  nav2_operations_msgs
)

ament_package()
