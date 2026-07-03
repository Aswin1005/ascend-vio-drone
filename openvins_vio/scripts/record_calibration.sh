#!/bin/bash
# ============================================================================
# Record Calibration Bag for Kalibr (Native ROS 1)
# RealSense D435i stereo IR cameras + Pixhawk IMU via MAVROS
# ============================================================================

set -e
echo "============================================"
echo "  Record Calibration Data for Kalibr"
echo "============================================"
echo ""

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CALIB_DIR="$HOME/calibration_data"
mkdir -p $CALIB_DIR

echo "This script will:"
echo "  1. Start RealSense camera (stereo IR only, IMU disabled)"
echo "  2. Start MAVROS (for Pixhawk IMU)"
echo "  3. Record a ROS bag with camera + IMU data"
echo ""
echo "INSTRUCTIONS:"
echo "  - Hold the ENTIRE DRONE in your hands"
echo "  - Face the AprilGrid calibration target (keep it fully visible)"
echo "  - Move SLOWLY in all 6 DOF"
echo "  - Record for 60-120 seconds"
echo ""
read -p "Press ENTER when ready to start recording..."

# Start RealSense (cameras only, IMU disabled)
echo "Starting RealSense cameras..."
roslaunch realsense2_camera rs_camera.launch \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_color:=false \
    enable_depth:=false \
    enable_gyro:=false \
    enable_accel:=false \
    emitter_enable:=false \
    enable_sync:=true &
RS_PID=$!
sleep 4

# Start MAVROS (Pixhawk IMU)
echo "Starting MAVROS..."
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
MR_PID=$!
sleep 5

echo ""
echo "============================================"
echo "  RECORDING - Move the drone slowly!"
echo "  Press Ctrl+C to stop recording"
echo "============================================"

# Record bag
rosbag record \
    /camera/infra1/image_rect_raw \
    /camera/infra2/image_rect_raw \
    /mavros/imu/data_raw \
    -O ${CALIB_DIR}/calibration_data.bag \
    --duration=120

# Cleanup
kill -SIGINT $RS_PID $MR_PID 2>/dev/null
wait $RS_PID $MR_PID 2>/dev/null

echo "Recording Complete! Bag saved to: ${CALIB_DIR}/calibration_data.bag"
