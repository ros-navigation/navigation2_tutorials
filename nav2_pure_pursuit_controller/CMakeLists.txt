cmake_minimum_required(VERSION 3.5)
project(nav2_pure_pursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_ros_common REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  nav2_ros_common
)

add_library(nav2_pure_pursuit_controller SHARED
        src/pure_pursuit_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(nav2_pure_pursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(nav2_pure_pursuit_controller PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::filters
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
)

install(TARGETS nav2_pure_pursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_pure_pursuit_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_pure_pursuit_controller.xml)

ament_package()

