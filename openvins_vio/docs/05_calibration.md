# Camera-IMU Calibration Guide (Kalibr)

## Prerequisites
- AprilGrid target printed on a **rigid, flat board** (A3 size or larger)
- RealSense D435i connected to Jetson via USB 3.0
- Pixhawk connected via USB-C (for IMU data)

## Phase 1: Record Calibration Data

### Before you run the script
- Place the AprilGrid target on a **flat wall or table** — it must be completely still while you move the drone
- Ensure the room is **well-lit** (not dim — the infrared cameras need texture contrast)
- The emitter on the RealSense will be OFF during recording (this is intentional for stereo calibration)

### Run the recording script
```bash
source ~/catkin_ws/devel/setup.bash
cd ~/openvins_vio
./scripts/record_calibration.sh
```

### What to do while recording (60-120 seconds)
The script records for **120 seconds automatically**. While it is recording:

1. **Hold the entire drone** in both hands (not just the camera)
2. **Face the AprilGrid** — keep it fully visible in the camera frame at all times
3. **Move SLOWLY** — fast movements cause motion blur and ruin the calibration
4. Cover all **6 degrees of freedom** in a slow, deliberate sequence:
   - Tilt forward and back (pitch)
   - Tilt left and right (roll)  
   - Rotate left and right (yaw)
   - Slide left and right (lateral)
   - Slide up and down (vertical)
   - Move closer and further from the target (depth)
5. Keep the **AprilGrid fully in frame** throughout — partial views are discarded
6. Stay at **0.3m–1.0m distance** from the target

> **Tip**: Think of it as slowly "painting" all sides of a cube with the camera's view. The more varied angles you cover, the better the calibration.

### After recording completes
The bag file is saved to `~/calibration_data/calibration_data.bag`.

Verify it was recorded correctly:
```bash
rosbag info ~/calibration_data/calibration_data.bag
```
You should see:
- `/camera/infra1/image_rect_raw` at ~30 Hz
- `/camera/infra2/image_rect_raw` at ~30 Hz
- `/mavros/imu/data_raw` at ~200 Hz
- Duration of 60–120 seconds

---

## Phase 2: Run Kalibr

Switch to the Kalibr workspace and run the calibration:
```bash
source ~/kalibr_ws/devel/setup.bash
cd ~/openvins_vio
./scripts/run_kalibr.sh
```

> **Warning**: This takes **30–90 minutes** on the Jetson Nano. Do not interrupt it.

The output files will be saved to `~/calibration_data/`:
- `camchain.yaml` — camera intrinsics + stereo extrinsics
- `camchain-imucam.yaml` — camera-IMU extrinsic transform (the key result)
- `results-imucam.txt` — human-readable summary

### Check if calibration quality is good
At the end of the run, Kalibr will print reprojection errors. Look for:
- **Reprojection error < 0.5 px** — good calibration
- **Reprojection error > 1.0 px** — redo the recording, move more slowly

---

## Phase 3: Update OpenVINS Config

Copy the results from `~/calibration_data/camchain-imucam.yaml` into:
`config/openvins/estimator_config.yaml`

1. Copy camera intrinsics (`fx`, `fy`, `cx`, `cy`, `distortion`) → `cam0_k` and `cam0_d`
2. Copy `T_cam_imu` → `cam0_T_imu`
3. Copy `timeshift_cam_imu` → `calib_imu_cam_dt`

## Next Step
Proceed to [06_testing.md](./06_testing.md) to test the full VIO pipeline indoors.
