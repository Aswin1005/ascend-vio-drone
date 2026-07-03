# Ascend VIO Drone Autonomous Flight Stack

Welcome to the **Ascend VIO Drone** monorepo. This repository contains the complete autonomous software stack, Ground Control Station (GCS) UI, mission control logic, and Visual-Inertial Odometry (VIO) configurations for a GPS-denied autonomous quadcopter.

---

## 🚁 Project Overview & Architecture
The stack is designed for stable, GPS-denied autonomous flight of a quadcopter running **ArduCopter** on a Pixhawk flight controller, navigated inside GPS-denied/indoor environments using Visual-Inertial Odometry.

```
                  +--------------------------------+
                  |       Jetson Nano (ROS)        |
                  |                                |
  [RealSense] --->|  Open-VINS -> Vio-Bridge Node  |
                  |               |                |
                  |               v                |
                  |      MAVROS / MAVLink          |
                  +---------------+----------------+
                                  |
                                  v
                         [Pixhawk Autopilot] <--- [TFmini Rangefinder]
```

### Hardware Stack
* **Companion Computer:** Jetson Nano (Ubuntu 18.04 LTS, running ROS1 Melodic natively)
* **Visual Sensor:** Intel RealSense D435i / T265 (tracking camera)
* **Flight Controller:** Pixhawk (connected via serial/USB telemetry port to Jetson)
* **Altitude Sensor:** TFmini/Lidar Rangefinder (connected to Pixhawk for Z-axis control)

### Software Stack
* **Open-VINS (MSCKF):** Visual-Inertial Odometry estimator optimized for ARM platforms.
* **MAVROS:** Communication bridge between ROS1 and the Pixhawk autopilot (MAVLink).
* **Ground Control Station (GCS):** A modern PyQT5 GUI running on the ground station laptop, enabling remote launch, real-time telemetry monitoring, ESP-NOW charging telemetry visualization, and batch seed verification.

---

## 📁 Repository Structure

```
ascend-vio-drone/
├── gcs/                       # PyQT5 Ground Control Station
│   ├── gcs/
│   │   ├── tabs/              # GCS UI tabs (Mission, Charging, Telemetry, Seed Viewer)
│   │   ├── batch_verify.py    # SSIM & ORB verification of captured seeds
│   │   ├── workers.py         # Background worker threads (SSH, UDP Charging, Hz monitor)
│   │   └── ascend_gcs.py      # Main GCS entry point
├── openvins_vio/              # Open-VINS setup, launch files, and calibration files
│   ├── config/
│   │   ├── kalibr/            # Kalibr Camera-IMU calibration configurations
│   │   └── openvins/          # Estimator configurations (IMU chain parameters)
│   └── src/vio_bridge/        # yaw coordination & autopilot translation
├── elimination_round/         # Flight strategy & competition mission scripts
│   └── arena_nav_loiter_rtl.py# RTL / Yellow Intercept flight strategy
├── seed_tracker_src/          # Frame capturing and position logging scripts
├── vio_mission_ros1.py        # Primary native VIO flight mission control
├── gps_mission_ros1.py        # GPS-dependent mission script for benchmarking
└── .gitignore                 # Monorepo version control exclusions
```

---

## ⚡ Flight Missions

### 1. Mission 1: RC-Takeoff & Sweeps (`vio_mission_ros1.py`)
* Ramps up the RC throttle override (`1400` PWM to `1750` PWM) to takeoff.
* Surpasses ground offset (+0.10m) using TFmini rangefinder telemetry.
* Holds throttle at `1500` PWM and transitions to **LOITER**.
* Performs pitch forward/backward sweeps to survey the terrain.
* Triggers **LAND** and disarms.

### 2. Mission 2: Waypoint Zigzag (GUIDED) (`vio_mission_ros1.py`)
* RC takeoff and hovers for 10 seconds.
* Pre-streams local coordinates to EKF3, switches to **GUIDED** mode.
* Executes a local NED zigzag path (Forward -> Sideways -> Return).
* Fallback behavior moves to the next waypoint if a timeout is reached.

### 3. RTL & Yellow Intercept (`elimination_round/arena_nav_loiter_rtl.py`)
* Primary autonomous lifecycle logic.
* Checks boundary integrity using `yellow_border_node.py` before flight.
* Integrates post-mission seed transfer from the Jetson and commands an autonomous `sudo shutdown` on the Jetson Nano to conserve battery.

---

## 🖥️ Ground Control Station (GCS)

The PyQT5 GCS provides a consolidated interface for the field operator:
* **Connection Tab:** Manages SSH connection to the Jetson Nano and orchestrates SFTP folder transfers.
* **Mission Tab:** Displays terminal console logs, monitors topic rates (Hz), and triggers sequential launch scripts.
* **Charging Tab:** Monitors real-time ESP-NOW power/telemetry stream (UDP port `12345`).
* **Seed Viewer Tab:** Leverages `batch_verify.py` (Otsu thresholding + SSIM/ORB matching) to analyze geological features and verify collected seeds against databases.

---

## ⚙️ Quick Start

### 1. Launching VIO Pipeline on Jetson Nano
1. Connect via SSH:
   ```bash
   ssh isro@isro.local
   # Password: isro@123
   ```
2. Start the ROS pipeline:
   ```bash
   source /opt/ros/melodic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch vio_bridge full_pipeline.launch
   ```

### 2. Launching Mission Controller
In a separate Jetson terminal:
```bash
rosrun vio_bridge vio_mission_ros1.py
```

### 3. Launching GCS (Ground Station Laptop)
1. Install dependencies:
   ```bash
   pip install PyQt5 paramiko opencv-python scikit-image
   ```
2. Run the UI:
   ```bash
   python gcs/gcs/ascend_gcs.py
   ```

---

## 🛡️ Safety Mechanisms
* **Keyboard Manual Overrides:** Pressing **`l`** triggers an immediate **LAND** mode switch; **`Ctrl+C`** initiates emergency LAND + disarming.
* **VIO Divergence Guard:** Monitors `/mavros/vision_pose/pose` in a background thread. If state estimation drift exceeds ±4.0m (`VIO_XY_LIMIT_M`), it immediately aborts and commands a LAND.
* **Post-Mission Lifecycle:** Automatically downloads logging/feature data, terminates running ROS nodes, and runs `sudo shutdown` on the drone.
