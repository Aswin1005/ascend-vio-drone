#!/bin/bash
# ============================================================================
# Test MAVROS (Native ROS 1)
# ============================================================================

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "Starting MAVROS..."
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:115200 &
PID=$!
sleep 5

echo "State:"
rostopic echo /mavros/state -n 1
echo "IMU Rate:"
rostopic hz /mavros/imu/data_raw --window 20

kill -SIGINT $PID
