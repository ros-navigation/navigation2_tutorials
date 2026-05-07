cmake_minimum_required(VERSION 3.5)
project(nav2_operations_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BladeCommand.action"
  "action/CameraCommand.action"
  "action/NavigateWithOperations.action"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
