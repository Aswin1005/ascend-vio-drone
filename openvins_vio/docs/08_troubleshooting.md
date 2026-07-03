# Troubleshooting Guide

## ROS 1 Issues

### roscore fails to start
Check your IP address and `/etc/hosts`. Ensure `ROS_MASTER_URI` and `ROS_IP` are correct if working over a network, otherwise they should point to localhost.

### catkin build fails (Out of Memory)
- Ensure your 8GB swap file is active: `free -h`
- Compile packages individually with `-j1`: `catkin build package_name -j1`

## RealSense Issues
- Use USB 3.0 port (not 2.0).
- If it throws errors on launch, verify udev rules were set: `cat /etc/udev/rules.d/99-realsense-libusb.rules`
- Restart udev: `sudo udevadm control --reload-rules && sudo udevadm trigger`

## Kalibr Issues
- Kalibr build errors on Ubuntu 18.04: Ensure `python-igraph` is installed via `pip install python-igraph`, not apt.
- Matplotlib `wxagg` errors are patched by the `setup_kalibr.sh` script, but if they persist, edit `PlotCollection.py` inside the kalibr source to use `backend_wx` instead of `backend_wxagg`.

## VIO Issues
- **Waiting for IMU data...**: Check MAVROS: `rostopic hz /mavros/imu/data_raw`. Ensure ArduPilot `SR0_RAW_IMU = 200`.
- **Flies away / Diverges**: Recalibrate with Kalibr. The camera-IMU extrinsic matrix is wrong.
