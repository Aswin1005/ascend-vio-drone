#!/bin/bash
# ============================================================================
# Test RealSense Camera (Native ROS 1)
# ============================================================================

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "Starting RealSense..."
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false enable_gyro:=false enable_accel:=false emitter_enable:=false enable_sync:=true &
PID=$!
sleep 5

echo "Topics:"
rostopic list | grep camera
echo "Rate:"
rostopic hz /camera/infra1/image_rect_raw --window 10

kill -SIGINT $PID
