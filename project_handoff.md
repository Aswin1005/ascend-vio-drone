# VIO Drone Autonomous Flight - Project Handoff Document

This document serves as a comprehensive guide for anyone continuing the development, testing, and deployment of the Visual-Inertial Odometry (VIO) based autonomous flight project.

---

## 1. Project Overview & Architecture
The objective of this project is to achieve stable, GPS-denied autonomous flight of a quadcopter (running ArduCopter on a Pixhawk flight controller) inside indoor or GPS-denied environments. 

### Hardware Stack
* **Companion Computer:** Jetson Nano (Ubuntu 18.04 LTS, running ROS1 Melodic natively)
* **Visual Sensor:** Intel RealSense D435i / T265 (tracking camera)
* **Flight Controller:** Pixhawk (connected via serial/USB telemetry port to Jetson)
* **Altitude Sensor:** TFmini/Lidar Rangefinder (connected to Pixhawk)

### Software Architecture
The Jetson runs a native Catkin workspace (`~/catkin_ws`) containing:
* **OpenVINS:** Visual-Inertial Odometry estimator that consumes RealSense IMU + Camera topics and produces state estimation.
* **MAVROS:** Communication bridge between ROS and ArduPilot (MAVLink protocol).
* **Vio Bridge:** Custom package that translates OpenVINS odometry into MAVROS vision pose estimation (`/mavros/vision_pose/pose`), handles yaw coordination, and contains the python mission controllers.

---

## 2. Key Directories & Files

### Controller & Mission Scripts
Located on the Jetson Nano at `/home/isro/catkin_ws/src/vio_bridge/scripts/`:
* **`vio_mission_ros1.py`**: The primary VIO-guided flight script.
* **`gps_mission_ros1.py`**: A GPS-dependent counterpart used for testing and baseline comparison.

### Launcher & Configurations
* **Launch file:** `/home/isro/catkin_ws/src/vio_bridge/launch/full_pipeline.launch` (starts realsense, openvins, mavros, and the yaw-remapping node).
* **MAVROS configurations:** `/home/isro/catkin_ws/src/vio_bridge/config/mavros/apm_config.yaml`
* **OpenVINS calibration:** `/home/isro/catkin_ws/src/openvins_vio/config/kalibr/` (contains custom-calibrated IMU/Camera matrices generated via Kalibr).

---

## 3. Flight Missions Details

### Mission 1: RC-Takeoff & Sweeps
1. **Interactive Prompt:** The user selects the takeoff mode: **FLOWHOLD** (optical flow - Mode 22) or **LOITER**.
2. **RC Override Takeoff:** The script ramps up the RC throttle override value (starting from `1400` PWM up to a maximum of `1850` PWM).
3. **Liftoff Detection:** Uses the rangefinder value (`rf_range`) to detect when altitude surpasses ground offset (+0.10m). Once detected, it freezes the takeoff throttle at a stable value (configured as `1500` PWM).
4. **Takeoff Hover:** Hovers stably at neutral controls for **10 seconds** before transitioning.
5. **LOITER Transition:** Switches the flight mode to **LOITER** (if it took off in FLOWHOLD) and establishes a background RC heartbeat to lock controls.
6. **Pitch Forward:** Sends a forward pitch command (`1550` PWM) for **5 seconds**.
7. **Neutral Hold:** Resets pitch to neutral (`1500` PWM) and hovers for **10 seconds**.
8. **Pitch Backward:** Sends a backward pitch command (`1450` PWM) for **5 seconds**.
9. **Final Neutral Hold:** Resets pitch to neutral (`1500` PWM) and hovers for **10 seconds**.
10. **Land:** Triggers **LAND** mode, waits 20 seconds, and disarms the motors.

### Mission 2: Waypoint Zigzag (GUIDED)
1. **Takeoff Mode:** Prompted at startup (FLOWHOLD or LOITER takeoff).
2. **Post-Takeoff Hover:** Hovers at `1500` PWM for **10 seconds** before starting the guided sequence.
3. **Setpoints Pre-stream:** Pre-streams local coordinates to `/mavros/setpoint_raw/local` for 3 seconds to feed EKF3.
4. **GUIDED Switch:** Changes autopilot mode to **GUIDED**.
5. **Zigzag Setpoints:** Executes a local NED trajectory path:
   * **Forward** by `M2_FWD_M` (e.g. 2.0 meters)
   * **Sideways (Right)** by `M2_SIDE_M` (e.g. 1.0 meters)
   * **Backward** (Return to Starting line)
6. **Fallback Behavior:** If the drone fails to reach a waypoint within the timeout limit (`M2_WP_TIMEOUT_S`), it will issue a warning and **continue to the next waypoint** rather than aborting.
7. **Return & Land:** Switches back to FLOWHOLD/LOITER, triggers **LAND**, and disarms.

---

## 4. Crucial Configurations & Troubleshooting

### Autopilot Parameters (Pixhawk)
The EKF3 parameters must match the active altitude sensor configuration:
* `EK3_SRC1_POSXY = 6` (ExternalNav / VIO)
* `EK3_SRC1_VELXY = 6` (ExternalNav / VIO)
* `EK3_SRC1_POSZ = 1` (RangeFinder)  <-- *Must be 1 when using rangefinder for altitude.*
* `EK3_SRC1_POSZ = 0` (Barometer)    <-- *If the hardware rangefinder goes offline, set this to 0 to use the barometer as a fallback.*

To read or write these parameters via MAVROS, run:
```bash
# Get parameter
rosrun mavros mavparam get EK3_SRC1_POSZ
# Set parameter
rosrun mavros mavparam set EK3_SRC1_POSZ 1
```

### Safety Abort Mechanism
Both scripts run a keyboard monitoring thread in the background:
* Pressing **`l`** at any point will immediately command a **LAND** mode switch.
* Pressing **`Ctrl+C`** will trigger an emergency abort (LAND mode + disarm).
* **VIO Divergence Guard:** (For VIO mission) If the position reported by OpenVINS on `/mavros/vision_pose/pose` drifts beyond ±4.0m (`VIO_XY_LIMIT_M`) on startup, it will abort the mission and trigger an immediate LAND to protect the drone.

---

## 5. How to Run

1. **SSH Connection:**
   ```bash
   ssh isro@isro.local
   # Password: isro@123
   ```

2. **Start the full pipeline:**
   ```bash
   source /opt/ros/melodic/setup.bash
   source /home/isro/catkin_ws/devel/setup.bash
   roslaunch vio_bridge full_pipeline.launch
   ```

3. **Execute the mission script:**
   In another terminal, run:
   ```bash
   source /opt/ros/melodic/setup.bash
   source /home/isro/catkin_ws/devel/setup.bash
   rosrun vio_bridge vio_mission_ros1.py
   # Follow the interactive prompts to select the mission and takeoff mode.
   ```
