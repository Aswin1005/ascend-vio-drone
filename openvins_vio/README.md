# OpenVINS VIO Pipeline — GPS-Denied Indoor Flight

Fly a drone in **Loiter mode without GPS** using Visual-Inertial Odometry (VIO) natively on a Jetson Nano running Ubuntu 18.04 (ROS 1 Melodic).

## System Overview

```
RealSense D435i (Camera) ──┐
                            ├── OpenVINS (VIO) ── VIO Bridge ── MAVROS ── Pixhawk (EKF3) ── Motors
Pixhawk 6C Mini (IMU) ─────┘
```

| Component | Role |
|---|---|
| **Intel RealSense D435i** | Infrared stereo camera (visual features) |
| **Pixhawk 6C Mini** | IMU data (via MAVROS) + flight control |
| **OpenVINS** | Fuses camera + IMU → 6DOF pose estimate |
| **VIO Bridge** | Transforms VIO output → MAVROS vision_pose |
| **MAVROS** | ROS 1 ↔ ArduPilot communication |
| **ArduPilot EKF3** | Fuses VIO position with Pixhawk IMU for flight |

## Hardware Requirements

- Jetson Nano 4GB (JetPack 4.6.x / L4T 32.7.x)
- Intel RealSense D435i
- Pixhawk 6C Mini (ArduPilot)
- USB-C cable (Pixhawk ↔ Jetson)
- USB 3.0 cable (RealSense ↔ Jetson, short < 0.5m)
- 5V/4A power supply for Jetson (BEC from drone battery)
- Cooling fan for Jetson Nano
- AprilGrid calibration target (printed on rigid board)

## Quick Start Guide

### Step 1: Wire Everything
See [docs/01_hardware_wiring.md](docs/01_hardware_wiring.md)

### Step 2: Install ROS 1
```bash
sudo ./scripts/setup_ros1_melodic.sh
sudo reboot
```

### Step 3: Build Workspace (~2-3 hours)
```bash
./scripts/build_workspace.sh
```

### Step 4: Build Kalibr Natively (~2 hours)
```bash
./scripts/setup_kalibr.sh
```

### Step 5: Test Components
```bash
bash scripts/test_camera.sh
bash scripts/test_mavros.sh
```

### Step 6: Calibrate Camera + IMU
See [docs/05_calibration.md](docs/05_calibration.md) — this is the most important step!

### Step 7: Configure ArduPilot
Set parameters in Mission Planner. See [docs/04_ardupilot_params.md](docs/04_ardupilot_params.md)

### Step 8: Test VIO Indoors
```bash
roslaunch vio_bridge full_pipeline.launch
```
See [docs/06_testing.md](docs/06_testing.md)

### Step 9: First Flight
See [docs/07_first_flight.md](docs/07_first_flight.md)

## Troubleshooting
See [docs/08_troubleshooting.md](docs/08_troubleshooting.md)
