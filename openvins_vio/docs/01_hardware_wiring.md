# Hardware Wiring Guide

## Overview

Your setup uses **two USB cables** — one for the camera, one for the Pixhawk. No soldering or UART wiring required.

## Connection Diagram

```
                    ┌──────────────────────┐
                    │    Jetson Nano 4GB    │
                    │                      │
  USB 3.0 (blue)───┤ USB 3.0 Port         │
  to RealSense     │                      │
                    │ USB 2.0/3.0 Port ────┤───USB-C cable
                    │                      │   to Pixhawk
                    │ Barrel Jack / USB-C──┤───5V Power (BEC)
                    └──────────────────────┘

                    ┌──────────────────────┐
                    │  Pixhawk 6C Mini     │
                    │                      │
  USB-C ────────────┤ USB-C Port           │
  from Jetson       │                      │
                    │ POWER1 ──────────────┤───Power Module
                    └──────────────────────┘
```

## 1. RealSense D435i → Jetson Nano

| Item | Detail |
|---|---|
| **Cable** | USB 3.0 Type-A to Micro-B (comes with camera) |
| **Jetson Port** | USB 3.0 port (the blue one) — **this matters!** |
| **Cable Length** | Keep under 0.5m — longer cables cause frame drops |

> **Important**: Use USB 3.0, not 2.0. The infrared stereo streams require the bandwidth. If you get frame drops, try a shorter or higher-quality cable.

### Mounting the Camera
- Your camera is mounted underneath the drone at ~35-50° pitch downward — this is good for VIO
- Ensure the camera has a clear, unobstructed view of the ground/forward area
- Use vibration dampening (double-sided foam tape) between the 3D printed bracket and the drone frame
- Secure the USB cable with zip ties along the frame to prevent vibration disconnects

## 2. Pixhawk 6C Mini → Jetson Nano

| Item | Detail |
|---|---|
| **Cable** | USB-C to USB-A (or USB-C to USB-C depending on Jetson port) |
| **Jetson Port** | Any available USB port |
| **Device** | Shows as `/dev/ttyACM0` on Jetson |
| **Baud Rate** | 115200 (set in MAVROS config) |

> **Note**: Since you're using USB (not UART/TELEM2), no level shifting or custom wiring is needed. USB handles all the protocol and voltage conversion.

### Securing the Cable
- **Critical**: Vibration WILL loosen USB connections
- Use zip ties at both ends
- Add a strain relief loop (small loop of slack cable secured to the frame)
- Consider hot glue on the USB connector edges for extra security

## 3. Power

### Jetson Nano Power
The Jetson Nano needs **5V @ 4A**. Options:

| Method | Details |
|---|---|
| **5V BEC from main battery** | Best for drones. Use a quality 5V/5A BEC connected to the battery |
| **Separate USB power bank** | Heavier but isolated from motor noise |
| **Barrel jack** | 5V/4A supply to the barrel jack connector (recommended over USB-C power) |

> **Warning**: Do NOT power the Jetson from the Pixhawk's USB port. The Pixhawk cannot supply enough current.

### Pixhawk Power
- Powered normally through its power module (PM02 or similar)
- No change from your existing GPS setup

## 4. Weight Budget

| Component | Approximate Weight |
|---|---|
| Jetson Nano (with heatsink) | ~140g |
| RealSense D435i | ~72g |
| USB cables (2x short) | ~30g |
| 3D printed mount | ~20-40g |
| **Total added weight** | **~260-280g** |

Make sure your drone can handle this extra weight. Adjust PID tuning if needed after adding the companion computer.

## 5. Checklist Before Proceeding

- [ ] RealSense connected to Jetson USB 3.0 port with short cable
- [ ] Pixhawk connected to Jetson via USB-C
- [ ] Both cables secured with zip ties + strain relief
- [ ] Jetson powered from 5V BEC (not Pixhawk USB)
- [ ] Camera has clear view (no obstructions from arms/props)
- [ ] Camera mount is rigid (no wobble — wobble kills VIO accuracy)
- [ ] Cooling fan attached to Jetson Nano heatsink
