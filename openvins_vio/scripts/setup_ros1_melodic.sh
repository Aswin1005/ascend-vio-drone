#!/bin/bash
# ============================================================================
# Jetson Nano ROS 1 Melodic Native Setup
# Run this ONCE on a fresh Jetson Nano (L4T 32.7.x / Ubuntu 18.04)
# Usage: chmod +x setup_ros1_melodic.sh && sudo ./setup_ros1_melodic.sh
# ============================================================================

set -e
echo "============================================"
echo "  Jetson Nano Native ROS 1 Melodic Setup"
echo "============================================"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run as root (sudo ./setup_ros1_melodic.sh)"
    exit 1
fi

CURRENT_USER=${SUDO_USER:-$USER}

# 1. Swap Space (8GB) - Critical for native compilation
echo ""
echo "[1/6] Setting up 8GB swap file..."
if [ ! -f /var/swapfile ]; then
    systemctl disable nvzramconfig 2>/dev/null || true
    systemctl stop nvzramconfig 2>/dev/null || true
    fallocate -l 8G /var/swapfile
    chmod 600 /var/swapfile
    mkswap /var/swapfile
    swapon /var/swapfile
    if ! grep -q "swapfile" /etc/fstab; then
        echo "/var/swapfile none swap sw 0 0" >> /etc/fstab
    fi
    echo "  ✓ 8GB swap file created"
else
    echo "  ✓ Swap file already exists"
fi

# 2. Add ROS Melodic Repository
echo ""
echo "[2/6] Configuring ROS Melodic repository..."
if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
fi

apt-get update

# 3. Install ROS 1 Melodic and Dependencies
echo ""
echo "[3/6] Installing ROS Melodic and system dependencies..."
apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    python-catkin-tools \
    git wget curl nano htop \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-tf2-geometry-msgs

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
# Run rosdep update as the normal user
su - ${CURRENT_USER} -c "rosdep update"

# 4. Install Intel RealSense SDK 2.0 (librealsense)
echo ""
echo "[4/6] Installing Intel RealSense SDK..."
apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
if [ ! -f /etc/apt/sources.list.d/realsense-public.list ]; then
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
fi
apt-get install -y librealsense2-utils librealsense2-dev

# 5. Install Ceres Solver (Required for OpenVINS)
echo ""
echo "[5/6] Compiling Ceres Solver..."
if [ ! -d /usr/local/include/ceres ]; then
    cd /tmp
    git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git
    cd ceres-solver
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
    make -j2
    make install
    rm -rf /tmp/ceres-solver
else
    echo "  ✓ Ceres solver already installed"
fi

# 6. Udev Rules and User Groups
echo ""
echo "[6/6] Configuring udev rules and groups..."
usermod -aG dialout ${CURRENT_USER}

cat > /etc/udev/rules.d/99-realsense-libusb.rules << 'EOF'
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", MODE:="0666"
EOF

cat > /etc/udev/rules.d/99-pixhawk.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", SYMLINK+="pixhawk", MODE:="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", SYMLINK+="pixhawk", MODE:="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", SYMLINK+="pixhawk", MODE:="0666"
EOF

udevadm control --reload-rules
udevadm trigger

# Setup bashrc
if ! grep -q "ros/melodic/setup.bash" /home/${CURRENT_USER}/.bashrc; then
    echo "source /opt/ros/melodic/setup.bash" >> /home/${CURRENT_USER}/.bashrc
fi

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo "Please restart your Jetson Nano: sudo reboot"
