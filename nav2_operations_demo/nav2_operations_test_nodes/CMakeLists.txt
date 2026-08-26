cmake_minimum_required(VERSION 3.5)
project(nav2_operations_test_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# FakeRobot
add_executable(fake_robot_node src/fake_robot.cpp)
target_include_directories(fake_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fake_robot_node
  rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

# BladeSimulator
add_executable(blade_simulator_node src/blade_simulator.cpp)
target_include_directories(blade_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(blade_simulator_node
  rclcpp std_msgs visualization_msgs geometry_msgs tf2 tf2_ros)

# CameraSimulator
add_executable(camera_simulator_node src/camera_simulator.cpp)
target_include_directories(camera_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_simulator_node
  rclcpp std_msgs visualization_msgs geometry_msgs tf2 tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  fake_robot_node
  blade_simulator_node
  camera_simulator_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/rviz_goal_client.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
