cmake_minimum_required(VERSION 3.16)
project(nav2_lidar_ground_segmentation_demo)

find_package(ament_cmake REQUIRED)

# Install launch files, configs, and model files
install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
