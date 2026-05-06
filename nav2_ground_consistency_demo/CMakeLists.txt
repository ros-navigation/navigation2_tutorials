cmake_minimum_required(VERSION 3.16)
project(nav2_ground_consistency_demo)

find_package(ament_cmake REQUIRED)

# Install launch files, configs, and simulation files
install(
  DIRECTORY launch config simulation
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
