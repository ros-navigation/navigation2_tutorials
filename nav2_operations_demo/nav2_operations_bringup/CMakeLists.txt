cmake_minimum_required(VERSION 3.5)
project(nav2_operations_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
