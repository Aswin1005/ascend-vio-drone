#!/bin/bash
# ============================================================================
# Build Native ROS 1 Workspace (OpenVINS, MAVROS, RealSense, VIO Bridge)
# Run this as your normal user (not root)
# ============================================================================

set -e
# Compute project directory before changing paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "  Building ROS 1 Workspace (catkin_ws)"
echo "============================================"

# Source system ROS
source /opt/ros/melodic/setup.bash

# Create workspace
WS_DIR="$HOME/catkin_ws"
mkdir -p "$WS_DIR/src"
cd "$WS_DIR"

# Initialize catkin workspace
if [ ! -d .catkin_tools ]; then
    catkin init
    catkin config --extend /opt/ros/melodic
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

cd src

# 1. Clone RealSense ROS 1 wrapper (SKIPPED - Already installed globally)
# echo "[1/4] Cloning RealSense ROS..."
# if [ ! -d realsense-ros ]; then
#     git clone --depth 1 --branch 2.3.2 https://github.com/IntelRealSense/realsense-ros.git
# fi
# if [ ! -d ddynamic_reconfigure ]; then
#     git clone --depth 1 --branch melodic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git
# fi

# 2. Clone OpenVINS
echo "[2/4] Cloning OpenVINS..."
if [ ! -d open_vins ]; then
    git clone --depth 1 https://github.com/rpng/open_vins.git
fi

# 3. Clone MAVROS (ROS 1 version) (SKIPPED - Already installed globally)
# echo "[3/4] Cloning MAVROS..."
# if [ ! -d mavros ]; then
#     git clone --depth 1 --branch master https://github.com/mavlink/mavros.git
#     # Install GeographicLib datasets required by MAVROS
#     sudo bash mavros/mavros/scripts/install_geographiclib_datasets.sh
# fi

# 4. Copy custom VIO Bridge
echo "[4/4] Copying VIO Bridge node..."
cp -r "$PROJECT_DIR/src/vio_bridge" "$WS_DIR/src/"

# Install any missing rosdep dependencies
echo "Installing rosdep dependencies..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y --skip-keys="libopencv-dev libopencv-contrib-dev libopencv-core-dev python-opencv"

# Build the workspace
echo ""
echo "Building workspace (This will take a while)..."
# Build in order to manage memory on Nano
# catkin build realsense2_camera -j2  # (SKIPPED)
# catkin build mavros -j1             # (SKIPPED)
catkin build ov_msckf -j1
catkin build vio_bridge -j2

# Setup bashrc
if ! grep -q "catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo "source $WS_DIR/devel/setup.bash" >> ~/.bashrc
fi

echo ""
echo "============================================"
echo "  Build Complete!"
echo "============================================"
echo "Run 'source ~/.bashrc' or open a new terminal to use the packages."
