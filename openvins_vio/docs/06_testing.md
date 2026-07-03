# Indoor VIO Testing Guide

## Prerequisites
- ROS 1 Melodic workspace built and RealSense + MAVROS individually tested
- Kalibr calibration completed and config files updated
- ArduPilot parameters set in Mission Planner

## Test Environment Setup
- Well-lit room with **textured** surfaces (walls, floor with patterns)
- Avoid: plain white walls, reflective surfaces, flickering lights
- Place distinctive objects/posters on walls if needed

## Test 1: Static Drift Test (5 minutes)

Place the drone on a table and don't move it.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vio_bridge full_pipeline.launch
```

In another terminal:
```bash
# Watch the VIO output position:
rostopic echo /ov_msckf/odometry/pose/pose/position
```

**Pass criteria**: Position drift < 10cm over 5 minutes.  
**If it drifts badly**: Check calibration, IMU noise params, or lighting.

## Test 2: Walk Test

Pick up the drone and walk around the room slowly.

```bash
# Watch position in real-time:
rostopic echo /mavros/local_position/pose/pose/position
```

**Pass criteria**: Position roughly tracks your movement, returns near origin when you return to start.

## Test 3: Mission Planner Visualization

1. Connect Mission Planner to Pixhawk via telemetry (WiFi/radio)
2. Start VIO pipeline on Jetson
3. In Mission Planner, check:
   - **Flight Data** → map shows drone position
   - **Status** tab → `local_position` values change as you move
   - **EKF** tab → shows healthy status

## Checking Health

```bash
# Topic rates:
rostopic hz /camera/infra1/image_raw    # ~30 Hz
rostopic hz /mavros/imu/data_raw         # ~200 Hz
rostopic hz /ov_msckf/odometry           # ~20-30 Hz
rostopic hz /mavros/vision_pose/pose     # ~30 Hz

# VIO bridge health log (printed every 5 seconds)
# Watch for warnings about stale data or disconnections
```

## Tuning Tips

| Issue | Fix |
|---|---|
| VIO drifts quickly | Re-run Kalibr calibration more carefully |
| Position jumps | Increase IMU noise parameters slightly |
| Low VIO rate | Reduce `num_pts` in estimator_config.yaml |
| "Flying away" | Check cam0_T_imu transform is correct |

## Next Step
Once VIO tracks well indoors, proceed to [07_first_flight.md](./07_first_flight.md)
