#!/bin/bash
# ============================================================================
# Native Kalibr Compilation Script for Jetson Nano (Ubuntu 18.04 / Python 2.7)
# Run this as your normal user (not root)
# ============================================================================

set -e
echo "============================================"
echo "  Building Kalibr Natively"
echo "============================================"

# Source system ROS
source /opt/ros/melodic/setup.bash

# Install system dependencies
echo "Installing Python dependencies..."
sudo apt-get update
sudo apt-get install -y \
    python-setuptools python-rosinstall ipython libeigen3-dev \
    libboost-all-dev doxygen libopencv-dev ros-melodic-vision-opencv \
    ros-melodic-image-transport-plugins ros-melodic-cmake-modules \
    software-properties-common libpoco-dev python-matplotlib python-scipy \
    python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev \
    libv4l-dev python-wxgtk4.0

# Install python-igraph via pip for Python 2.7
sudo pip install python-igraph

# Create Kalibr workspace
KALIBR_WS="$HOME/kalibr_ws"
mkdir -p "$KALIBR_WS/src"
cd "$KALIBR_WS"

if [ ! -d .catkin_tools ]; then
    catkin init
    catkin config --extend /opt/ros/melodic
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

cd src

# Clone Kalibr
if [ ! -d kalibr ]; then
    echo "Cloning Kalibr..."
    git clone https://github.com/ethz-asl/kalibr.git
    
    # Patch for Ubuntu 18.04 matplotlib wxagg issue
    echo "Applying Ubuntu 18.04 patch to Kalibr..."
    sed -i 's/from matplotlib.backends.backend_wxagg import NavigationToolbar2Wx as Toolbar/from matplotlib.backends.backend_wx import NavigationToolbar2Wx as Toolbar/' kalibr/Schweizer-Messer/sm_python/python/sm/PlotCollection.py || true
fi

# Build Kalibr
echo ""
echo "Building Kalibr (This will take a LONG time)..."
cd "$KALIBR_WS"
# Limit to 2 jobs to prevent Out Of Memory on Nano
catkin build -j2

echo ""
echo "============================================"
echo "  Kalibr Build Complete!"
echo "============================================"
echo "To use Kalibr, run: source ~/kalibr_ws/devel/setup.bash"
