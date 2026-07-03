# Native Jetson Setup (ROS 1 Melodic)

## Step 1: Transfer Project Files
Copy the `openvins_vio` directory to your Jetson:
```bash
scp -r "d:\open-vins\openvins_vio" isro@<JETSON_IP>:~/openvins_vio
```

## Step 2: Install Native ROS 1 Environment
```bash
cd ~/openvins_vio
chmod +x scripts/*.sh

# This sets up ROS Melodic, dependencies, swap, and udev rules
sudo ./scripts/setup_ros1_melodic.sh
```

## Step 3: Reboot
```bash
sudo reboot
```

## Step 4: Build Catkin Workspaces
After rebooting, SSH back in and build the workspaces:

```bash
cd ~/openvins_vio

# Build the main workspace (OpenVINS, MAVROS, RealSense, VIO Bridge)
./scripts/build_workspace.sh

# Build the Kalibr workspace
./scripts/setup_kalibr.sh
```

## Cooling Setup
The Jetson Nano WILL thermal throttle under native VIO compilation and workload.
```bash
# Set fan to run at full speed
sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'
```

## Next Step
Proceed to [05_calibration.md](./05_calibration.md) to record data using the native ROS 1 tools.
