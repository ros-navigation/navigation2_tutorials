#!/bin/bash
# Install all dependencies and clone repositories for nav2_ground_consistency_demo

set -e  # Exit on error

# Get workspace path from argument or use default
WORKSPACE="${1:-$HOME/ros2_ws}"

echo "Installing ROS 2 Jazzy dependencies for nav2_ground_consistency_demo..."
echo "Workspace: $WORKSPACE"

sudo apt-get update

# Required packages
echo "Installing required packages..."
sudo apt-get install -y ros-jazzy-ros-gz-sim
sudo apt-get install -y ros-jazzy-ros-gz-interfaces
sudo apt-get install -y ros-jazzy-tf2-tools

# Visualization
echo "Installing visualization packages..."
sudo apt-get install -y ros-jazzy-rviz2
sudo apt-get install -y ros-jazzy-rviz-common

# Navigation
echo "Installing navigation packages..."
sudo apt-get install -y ros-jazzy-nav2-bringup || true

# Joystick support (optional, for teleop)
echo "Installing joystick packages..."
sudo apt-get install -y ros-jazzy-joy || true
sudo apt-get install -y ros-jazzy-teleop-twist-joy || true
sudo apt-get install -y ros-jazzy-teleop-twist-keyboard || true
sudo apt-get install -y joystick || true

# Clone required repositories
echo ""
echo "Cloning required repositories..."

mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE/src"

echo "  Cloning KISS-ICP..."
git clone https://github.com/PRBonn/kiss-icp.git || echo "  (already exists)"

echo "  Cloning ground_segmentation..."
git clone https://github.com/dfki-ric/ground_segmentation.git || echo "  (already exists)"

echo "  Cloning ground_segmentation_ros2..."
git clone https://github.com/dfki-ric/ground_segmentation_ros2.git || echo "  (already exists)"

echo "  Cloning nav2_ground_consistency_costmap_plugin..."
git clone https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin.git || echo "  (already exists)"

# Initialize rosdep if needed
echo "Setting up rosdep..."
sudo rosdep init || true  # Ignore if already initialized
rosdep update

# Install workspace dependencies using rosdep
echo "Installing workspace dependencies with rosdep..."
cd "$WORKSPACE"
rosdep install --from-paths src --ignore-src -r -y || true

echo ""
echo "✓ All dependencies installed and repositories cloned!"
echo ""
echo "Next steps:"
echo "  1. cd $WORKSPACE"
echo "  2. colcon build --packages-up-to nav2_ground_consistency_demo --cmake-args -DCMAKE_BUILD_TYPE=RELEASE"
echo "  3. source install/setup.bash"
