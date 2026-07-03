#!/bin/bash
# ============================================================
# start_with_qual.sh
# Starts MAVProxy as a serial multiplexer, then launches the
# full VIO pipeline. Qual1.py can then connect via UDP 14550.
#
# Architecture:
#   /dev/ttyACM0 ──► MAVProxy ──► UDP 14551 ──► MAVROS (ROS)
#                             └──► UDP 14550 ──► Qual1.py (pymavlink)
#
# Usage:
#   ./start_with_qual.sh
#   (in a second terminal) sudo python3 ~/Qual1.py
# ============================================================

set -e

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Kill any leftover processes
echo "[1/4] Cleaning up old processes..."
killall -9 mavproxy.py mavproxy roslaunch roscore rosmaster mavros_node nodelet run_subscribe_msckf python3 2>/dev/null || true
sleep 2

# Start MAVProxy as serial multiplexer in background
echo "[2/4] Starting MAVProxy multiplexer..."
echo "      /dev/ttyACM0 --> UDP 14551 (MAVROS) + UDP 14550 (Qual1.py)"
/home/isro/.local/bin/mavproxy.py \
    --master=/dev/ttyACM0 \
    --baud=115200 \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14550 \
    --daemon \
    --non-interactive \
    2>/tmp/mavproxy.log &

MAVPROXY_PID=$!
echo "      MAVProxy PID: $MAVPROXY_PID"

# Wait for MAVProxy to open the serial port
echo "[3/4] Waiting 5 seconds for MAVProxy to connect to Pixhawk..."
sleep 5

# Verify MAVProxy is running
if ! ps -p $MAVPROXY_PID > /dev/null 2>&1; then
    echo "ERROR: MAVProxy failed to start! Check /tmp/mavproxy.log"
    cat /tmp/mavproxy.log
    exit 1
fi
echo "      MAVProxy is running. UDP ports 14550 and 14551 are open."

# Launch the full pipeline but pointing MAVROS to UDP instead of serial
echo "[4/4] Launching full VIO pipeline..."
echo "      MAVROS will connect via UDP 14551 (through MAVProxy)"
roslaunch vio_bridge full_pipeline.launch fcu_url:=udp://127.0.0.1:14551@

# Cleanup MAVProxy on exit
echo "Cleaning up MAVProxy..."
kill $MAVPROXY_PID 2>/dev/null || true
