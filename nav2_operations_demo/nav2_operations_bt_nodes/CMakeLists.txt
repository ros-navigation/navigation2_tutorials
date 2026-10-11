cmake_minimum_required(VERSION 3.5)
project(nav2_operations_bt_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_operations_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp
  nav2_behavior_tree
  nav2_operations_msgs
)

add_library(nav2_set_blade_state_bt_node SHARED
  src/action/set_blade_state.cpp)
target_include_directories(nav2_set_blade_state_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_set_blade_state_bt_node ${dependencies})
target_compile_definitions(nav2_set_blade_state_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(nav2_set_camera_yaw_bt_node SHARED
  src/action/set_camera_yaw.cpp)
target_include_directories(nav2_set_camera_yaw_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_set_camera_yaw_bt_node ${dependencies})
target_compile_definitions(nav2_set_camera_yaw_bt_node PRIVATE BT_PLUGIN_EXPORT)

set(bt_node_libs
  nav2_set_blade_state_bt_node
  nav2_set_camera_yaw_bt_node
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${bt_node_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${bt_node_libs})
ament_export_dependencies(${dependencies})

ament_package()
