#!/bin/bash
# Setup script for nav2_ground_consistency_demo tutorial
#
# Prerequisites:
#   - ROS 2 Jazzy installed (see: https://docs.ros.org/en/jazzy/Installation.html)
#   - Nav2 installed (see: https://docs.nav2.org/setup_guides/index.html)
#   - vcstool installed: sudo apt-get install python3-vcstool
#
# This script will:
#   1. Clone demo source code (if not already present)
#   2. Import dependencies using vcstool and dependencies.repos
#   3. Install ROS package dependencies using rosdep
#   4. Prepare for building

set -e  # Exit on error

# Get workspace path from argument or use current directory
WORKSPACE="${1:-.}"

echo "=========================================="
echo "Setting up nav2_ground_consistency_demo"
echo "=========================================="
echo ""
echo "Workspace: $(cd "$WORKSPACE" && pwd)"
echo ""

# Check if we're in a ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 environment not sourced"
    echo "Please source your ROS 2 installation first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "ROS Distro: $ROS_DISTRO"
echo ""

# Check if vcstool is installed
if ! command -v vcs &> /dev/null; then
    echo "ERROR: vcstool not found"
    echo "Install it with: sudo apt-get install python3-vcstool"
    exit 1
fi

# Import dependencies from .repos file
echo "Importing dependencies from dependencies.repos..."
cd "$WORKSPACE/src"

if [ -f "navigation2_tutorials/nav2_ground_consistency_demo/dependencies.repos" ]; then
    vcs import < navigation2_tutorials/nav2_ground_consistency_demo/dependencies.repos
else
    echo "ERROR: dependencies.repos not found"
    echo "Make sure you're in a workspace with navigation2_tutorials cloned"
    exit 1
fi

echo ""
echo "Installing ROS package dependencies..."
cd "$WORKSPACE"

rosdep init || true  # Ignore if already initialized

# Update rosdep
rosdep update || true

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y || true

echo ""
echo "✓ Setup complete!"
echo ""
echo "Next steps:"
echo "  1. cd $WORKSPACE"
echo "  2. source /opt/ros/jazzy/setup.bash"
echo "  3. colcon build --symlink-install --packages-up-to nav2_ground_consistency_demo --cmake-args -DCMAKE_BUILD_TYPE=RELEASE"
echo "  4. source install/setup.bash"
echo "  5. ros2 launch nav2_ground_consistency_demo full_stack.launch.py 2>&1 | grep -v "SampleConsensus""
