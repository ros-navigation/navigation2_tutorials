cmake_minimum_required(VERSION 3.16)
project(nav2_ground_consistency_demo)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files - Simulation
install(
  DIRECTORY simulation/
  DESTINATION share/${PROJECT_NAME}/simulation
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install launch files - NAV2 Stack
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files - Simulation
install(
  DIRECTORY simulation/config/
  DESTINATION share/${PROJECT_NAME}/config/simulation
)

# Install config files - NAV2
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install robot models (Gazebo Fuel models)
install(
  DIRECTORY simulation/models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Ament package generation
ament_package()
