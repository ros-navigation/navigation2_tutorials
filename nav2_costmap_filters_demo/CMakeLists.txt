cmake_minimum_required(VERSION 3.5)
project(nav2_costmap_filters_demo)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
