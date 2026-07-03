# ArduPilot Parameters for GPS-Denied VIO Flight

## Overview
Configure Pixhawk 6C Mini via **Mission Planner** → **Config** → **Full Parameter List**.

## Parameters to Set

### EKF3 Configuration
| Parameter | Value | Purpose |
|---|---|---|
| `AHRS_EKF_TYPE` | 3 | Use EKF3 |
| `EK3_ENABLE` | 1 | Enable EKF3 |
| `EK2_ENABLE` | 0 | Disable EKF2 |

### External Navigation Source
| Parameter | Value | Purpose |
|---|---|---|
| `EK3_SRC1_POSXY` | 6 | XY from ExternalNav (VIO) |
| `EK3_SRC1_VELXY` | 6 | Velocity from ExternalNav |
| `EK3_SRC1_POSZ` | 1 | Altitude from Barometer (stable indoors) |
| `EK3_SRC1_VELZ` | 0 | No Z velocity source |
| `EK3_SRC1_YAW` | 6 | Yaw from ExternalNav |
| `VISO_TYPE` | 1 | MAVLink vision input |

### Disable GPS
| Parameter | Value | Purpose |
|---|---|---|
| `GPS_TYPE` | 0 | Disable primary GPS |
| `GPS_TYPE2` | 0 | Disable secondary GPS |

### Arming
- In **Config** → **Arming Checks**: uncheck the **GPS** checkbox

### Stream Rates (CRITICAL for Pixhawk IMU → OpenVINS)
| Parameter | Value | Purpose |
|---|---|---|
| `SR0_RAW_IMU` | 200 | Raw IMU at 200Hz over USB |
| `SR0_EXTRA1` | 100 | Attitude data at 100Hz |
| `SR0_POSITION` | 10 | Position at 10Hz |

> `SR0_RAW_IMU = 200` is **critical**! OpenVINS needs high-rate IMU. If default (1-4Hz), VIO fails.

### Failsafe (CRITICAL for safety)
| Parameter | Value | Purpose |
|---|---|---|
| `FS_EKF_ACTION` | 1 | Land on EKF failure |
| `FS_EKF_THRESH` | 0.8 | EKF variance threshold |

### EKF Tuning
| Parameter | Value | Purpose |
|---|---|---|
| `EK3_ACC_P_NSE` | 0.5 | Accelerometer noise |
| `EK3_GYRO_P_NSE` | 0.03 | Gyroscope noise |

## After Setting Parameters
1. Click **Write Params** in Mission Planner
2. Power cycle the Pixhawk

## Setting EKF Origin (Required Before Each Flight)

### Method 1: Mission Planner
Right-click map → **"Set EKF Origin Here"**

### Method 2: Automatic
Our VIO bridge node does this automatically on first connection.

## Verification
In Mission Planner **Status** tab check:
- `local_position.x/y/z` tracks when you move the drone
- `ekf_status_report` shows healthy
- IMU rate shows ~200Hz

## Next Step
Proceed to [05_calibration.md](./05_calibration.md)
