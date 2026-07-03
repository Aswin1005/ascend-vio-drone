# Building and Running Natively

Since we are running natively on ROS 1 Melodic, there is no Docker overhead. All processes run directly on the Jetson Nano.

## Workspaces
You now have two workspaces in your home directory:
1. `~/catkin_ws`: Contains OpenVINS, MAVROS, RealSense, and the VIO Bridge.
2. `~/kalibr_ws`: Contains Kalibr for calibration.

## Sourcing
Your `~/.bashrc` was updated to automatically source `~/catkin_ws/devel/setup.bash`. 
When you need to use Kalibr, you must manually source its workspace:
```bash
source ~/kalibr_ws/devel/setup.bash
```

## Running Individual Nodes
If you want to run or test things manually:

**RealSense:**
```bash
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true enable_color:=false enable_depth:=false enable_gyro:=false enable_accel:=false emitter_enable:=false enable_sync:=true
```

**MAVROS:**
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:115200
```

**OpenVINS (requires RealSense and MAVROS running first):**
```bash
roslaunch vio_bridge openvins.launch
```

**VIO Bridge:**
```bash
roslaunch vio_bridge vio_bridge.launch
```

## Running the Full System
To start the entire VIO pipeline with correct initialization delays:
```bash
roslaunch vio_bridge full_pipeline.launch
```

## Troubleshooting Build Errors
If `catkin build` fails:
1. Check swap space: `free -h` (must show 8GB).
2. Reduce parallel jobs: `catkin build -j1` limits compilation to a single core to save memory.
