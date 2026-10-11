cmake_minimum_required(VERSION 3.5)
project(nav2_operations_servers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_operations_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  nav2_util
  std_msgs
  nav2_operations_msgs
)

add_executable(blade_server_node src/blade_server.cpp)
target_include_directories(blade_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(blade_server_node ${dependencies})

add_executable(camera_server_node src/camera_server.cpp)
target_include_directories(camera_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_server_node ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS blade_server_node camera_server_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
