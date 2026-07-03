# First Flight Checklist

## ⚠️ SAFETY WARNING
GPS-denied flight is inherently dangerous. A VIO failure mid-flight will cause a crash. **Always use a safety tether for initial flights.**

## Pre-Flight Checklist

### Hardware
- [ ] All USB cables zip-tied and secured
- [ ] Camera lens clean and unobstructed
- [ ] Props clear of camera field of view
- [ ] Cooling fan running on Jetson
- [ ] Battery fully charged
- [ ] Safety tether attached (for first flights)

### Software
- [ ] VIO pipeline running (all 4 nodes started)
- [ ] `rostopic hz /ov_msckf/odometry` shows 20+ Hz
- [ ] `rostopic echo /mavros/state` shows connected=true
- [ ] `rostopic echo /mavros/vision_pose/pose` shows data
- [ ] EKF origin set (check Mission Planner map)
- [ ] Static drift test passed (< 10cm in 5 min)
- [ ] Mission Planner shows local position tracking

### ArduPilot
- [ ] `FS_EKF_ACTION = 1` (land on EKF failure)
- [ ] All parameters from docs/04 are set
- [ ] No pre-arm warnings in Mission Planner

## Flight Procedure

### Flight 1: Tethered Hover (Loiter)
1. Place drone on ground in center of room
2. Ensure VIO pipeline is running and tracking
3. Attach safety tether to drone
4. Switch to **Loiter** mode in Mission Planner
5. Arm the drone
6. Slowly increase throttle to hover at ~30cm
7. **Hold for 30 seconds** — watch for drift
8. If stable: gently move stick left/right — drone should return to position
9. Land by reducing throttle

### Flight 2: Position Hold Test
1. Hover at 1m height
2. Release all sticks
3. Drone should hold position within ~30cm
4. Observe for 1 minute
5. Land

### Flight 3: Movement Test
1. Hover at 1m
2. Gently fly forward 1m, stop
3. Fly back to start
4. Check if drone holds position at each stop
5. Try left/right movements
6. Land

### Flight 4: Free Flight (Remove Tether)
Only after 3+ successful tethered flights:
1. Remove tether
2. Repeat flights 1-3
3. Stay low (< 1.5m) and slow

## Emergency Procedures

| Situation | Action |
|---|---|
| Drone drifts uncontrollably | Switch to **Stabilize** mode, land manually |
| VIO fails (position jumps) | `FS_EKF_ACTION` should auto-land |
| Jetson freezes | Switch to Stabilize, land manually |
| USB disconnects | EKF failsafe triggers auto-land |

## Important Notes

- **Indoor lighting**: Ensure bright, uniform, non-flickering LED lights
- **Textured environment**: Plain walls are bad for VIO — add posters or patterns
- **Start slow**: VIO needs time to initialize. Don't move fast immediately after arming
- **Vibration**: Excessive motor vibration degrades VIO. Use vibration dampening mounts
- **Temperature**: Monitor Jetson temperature with `tegrastats`. Throttling kills VIO performance
