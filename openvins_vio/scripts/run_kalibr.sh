#!/bin/bash
# ============================================================================
# Run Kalibr Calibration (Native ROS 1)
# ============================================================================

set -e
echo "============================================"
echo "  Kalibr Camera-IMU Calibration"
echo "============================================"

source /opt/ros/melodic/setup.bash
source ~/kalibr_ws/devel/setup.bash

CALIB_DIR="$HOME/calibration_data"
CONFIG_DIR="$HOME/openvins_vio/config/kalibr"

if [ ! -f "${CALIB_DIR}/calibration_data.bag" ]; then
    echo "ERROR: Bag file not found at ${CALIB_DIR}/calibration_data.bag"
    exit 1
fi

echo "Running Kalibr Stereo Camera Calibration..."
rosrun kalibr kalibr_calibrate_cameras \
    --target ${CONFIG_DIR}/april_6x6.yaml \
    --bag ${CALIB_DIR}/calibration_data.bag \
    --models pinhole-radtan pinhole-radtan \
    --topics /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw \
    --bag-from-to 5 115

echo "Running Kalibr Camera-IMU Calibration..."
rosrun kalibr kalibr_calibrate_imu_camera \
    --target ${CONFIG_DIR}/april_6x6.yaml \
    --cam ${CALIB_DIR}/camchain.yaml \
    --imu ${CONFIG_DIR}/imu_params.yaml \
    --imu-models calibrated \
    --bag ${CALIB_DIR}/calibration_data.bag \
    --bag-from-to 5 115 \
    --reprojection-sigma 1.0

echo "Kalibr Complete! Check output files in ${CALIB_DIR}"
