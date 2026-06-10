# ADR036 Hardware User Manual

**Project:** ADR036 — Vision-Based Autonomous 4WD Robot (CLIMB/AVOID Decision)  
**Platform:** Xycar C-model  
**Club:** RODONG, Chang-141  
**Status:** Archived (Final presentation: 2026-06-10)

---

## Table of Contents

1. [Platform Overview](#1-platform-overview)
2. [Computing Unit](#2-computing-unit)
3. [Drive System](#3-drive-system)
4. [Power System](#4-power-system)
5. [Sensors](#5-sensors)
6. [Camera](#6-camera)
7. [Network & Communication](#7-network--communication)
8. [Device Port Reference](#8-device-port-reference)
9. [Boot & Startup Procedure](#9-boot--startup-procedure)
10. [Known Hardware Behaviors & Caveats](#10-known-hardware-behaviors--caveats)
11. [Wiring & Physical Layout Notes](#11-wiring--physical-layout-notes)

---

## 1. Platform Overview

| Item | Specification |
|------|--------------|
| Vehicle base | Xycar C-model (4WD RC car chassis) |
| Drive type | 4-wheel drive, brushed DC motor via ESC |
| Steering | Servo-based Ackermann steering |
| Onboard computer | Raspberry Pi 4B |
| OS | Ubuntu 20.04 + ROS 1 Noetic |
| Primary sensors | 8-beam ultrasonic array, 6-DOF IMU |
| Vision | USB camera + ArUco marker detection + YOLOv8n ONNX |

---

## 2. Computing Unit

### Raspberry Pi 4B

| Item | Detail |
|------|--------|
| Model | Raspberry Pi 4 Model B |
| RAM | 4 GB (recommended) |
| Storage | microSD card (OS + workspace) |
| OS | Ubuntu 20.04 Server + ROS 1 Noetic |
| Username | `pi` |
| Hostname | `raspberry` (default) |

#### Critical `/boot/config.txt` Settings

The following lines **must be commented out** to prevent boot failure with the vc4 KMS driver:

```
# start_x=1        ← MUST be disabled
# gpu_mem=128      ← MUST be disabled
dtoverlay=vc4-kms-v3d
```

> **Warning:** Enabling `start_x=1` together with `dtoverlay=vc4-kms-v3d` causes a boot conflict. The Pi will fail to start the display stack and may become unresponsive.

#### ROS Workspace

```
~/xycar_ws/
  src/
    rodong/
      scripts/       ← rodong_main.py, vfh_planner.py, aruco_detector.py, yolo_node.py
      models/        ← rodong.onnx (YOLOv8n)
      launch/        ← rodong.launch
    xycar_device/
      xycar_ultrasonic_1x8/
      ...
```

---

## 3. Drive System

### ESC (Electronic Speed Controller)

| Item | Detail |
|------|--------|
| Type | Chinese-manufactured brushed ESC (model unspecified) |
| Interface | PWM via FTDI USB-UART adapter |
| USB port ID | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0` |
| Protocol | ROS topic `/xycar_motor` (`xycar_msgs/xycar_motor`) |

#### Motor Speed Parameters

| Parameter | Value |
|-----------|-------|
| `SPEED_NORMAL` | 20 |
| Deadband (no movement) | speed 8–15 |
| Reliable drive threshold | speed 20+ |
| Steering range | ±90° |
| Steering delta (`delta`) | 70 |

#### ESC Startup Behavior

- **Beeping on power-on is normal** for this ESC model.
- The ESC requires a **neutral PWM signal** before it will accept drive commands.
- **Reverse activation sequence** (must be followed exactly):
  1. Send neutral for 1 s
  2. Send reverse for 1 s
  3. Send neutral for 1 s
  4. Send reverse (actual motion begins)

> Skipping this sequence will result in the ESC ignoring reverse commands.

---

## 4. Power System

### Main Battery

| Item | Specification |
|------|--------------|
| Model | VB4600 (teamsi.co.kr) |
| Chemistry | NiMH |
| Cell count | 6 cells |
| Nominal voltage | 7.2 V |
| Capacity | 4600 mAh |
| Full-charge voltage | 8.4–8.7 V |
| Low-battery threshold | ~5.6 V (intermittent motor failure below this) |

### Charger

| Item | Specification |
|------|--------------|
| Model | NEW B6 v3 Smart Charger |
| Mode | NiMH |
| Charge current | 2 A |

#### Battery Voltage Under Load

Voltage **will drop** under motor load due to internal resistance (`V = E − I×r`). This is normal behavior. Monitor with:

```bash
rostopic echo /xycar_ultrasonic   # proxy for system health
```

If the robot exhibits sudden motor weakness or stops responding, check the battery voltage first.

---

## 5. Sensors

### 5.1 Ultrasonic Array (8-beam)

| Item | Detail |
|------|--------|
| Model | Xycar 1×8 ultrasonic module |
| Beam count | 8 |
| Interface | USB-Serial |
| USB port ID | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| ROS topic | `/xycar_ultrasonic` (`std_msgs/Int32MultiArray`) |
| Launch file | `~/xycar_ws/src/xycar_device/xycar_ultrasonic_1x8/launch/xycar_ultrasonic.launch` (absolute path required) |

#### Beam Angle Mapping

| Index | Angle | Direction |
|-------|-------|-----------|
| 0 | −90° | Left |
| 1 | −45° | Front-Left |
| 2 | +45° | Front-Right |
| 3 | 0° | Front |
| 4 | +90° | Right |
| 5 | +135° | Rear-Right |
| 6 | 180° | Rear |
| 7 | −135° | Rear-Left |

#### Key Distance Thresholds

| Parameter | Value |
|-----------|-------|
| `SONAR_EMERGENCY` | 15 cm |
| `SONAR_REVERSE` | 15 cm |
| `AVOID_CLEAR_CM` | 55 cm |

> **Bench-testing note:** When the robot is placed on a desk, the ultrasonic sensors detect the desk surface as an obstacle. Lower `THRESHOLD` to ~5 cm when validating without floor clearance.

### 5.2 IMU

| Item | Detail |
|------|--------|
| Interface | I²C or USB (via Xycar IMU module) |
| ROS node | `xycar_imu` + `imu_filter` |
| ROS topic | `/imu` (filtered) |
| Primary use | Yaw tracking for U-turn control |

> **Calibration note:** IMU yaw sign vs. steering sign must be verified empirically before deploying U-turn or K-turn maneuvers. Sign convention is hardware-dependent.

---

## 6. Camera

| Item | Detail |
|------|--------|
| Type | USB camera (UVC-compatible) |
| ROS node | `usb_cam` |
| ROS topic | `/usb_cam/image_raw` |
| Primary use | ArUco marker detection, YOLOv8n inference |

### OpenCV API Version Warning

The Pi runs **OpenCV 4.5.3**, which uses the **Old ArUco API**:

```python
# Pi (OpenCV 4.5.3) — correct
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
```

Do **not** use the New API (`ArucoDetector`) introduced in OpenCV 4.7+ — it will crash on the Pi.

### YOLO Model

| Item | Detail |
|------|--------|
| File | `~/xycar_ws/src/rodong/models/rodong.onnx` |
| Architecture | YOLOv8n |
| Input size | 320×320 |
| Classes | 0 = CLIMB (ramp), 1 = AVOID (stairs) |
| Training | Roboflow stairs-ramps v1, 50 epochs, CPU, mAP50 = 0.72 |
| Runtime | `onnxruntime` (OpenCV DNN incompatible — see below) |
| ONNX export | opset=11, `simplify=False` |
| Frame skip | `FRAME_SKIP=5` |

> **Critical:** OpenCV 4.5.3 DNN cannot load YOLOv8 ONNX (fails with `model.22/Add` assertion). Always use `onnxruntime` for inference on the Pi.  
> ONNX opset=12 + `onnxslim` also breaks Pi compatibility — use opset=11 with `simplify=False`.

---

## 7. Network & Communication

### LAN Connection (Pi ↔ Mini PC)

**Mini PC side:**
```bash
sudo ip addr flush dev eno1
sudo ip addr add 192.168.10.1/24 dev eno1
sudo ip link set eno1 up
ssh pi@192.168.10.2
```

**Pi static IP** is configured in:
```
/etc/netplan/01-network-manager-all.yaml
```
with `renderer: NetworkManager` for persistence across reboots.

### Internet Sharing to Pi (via Mini PC)

```bash
# Mini PC: enable NAT (wlp1s0 → eno1)
sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE

# Pi side:
sudo route add default via 192.168.10.1
# DNS: 8.8.8.8 in /etc/resolv.conf
```

### Wi-Fi AP Mode

Pi can also be accessed via:
```bash
ssh pi@10.42.0.1
```

### Mini PC

| Item | Detail |
|------|--------|
| Hostname/user | `yulee23@test2` |
| CPU | Intel Core i5-6500T |
| OS | Ubuntu 24.04 |
| Role | Code editing, SCP deploy, `catkin_make` remote |

> **Mini PC boot issue:** PCI AER errors from the Wi-Fi card can cause unresponsive boot. Fix: add `pci=noaer` to `GRUB_CMDLINE_LINUX` in `/etc/default/grub`.

---

## 8. Device Port Reference

| Device | Port (by-id) |
|--------|-------------|
| Ultrasonic module | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| FTDI motor controller | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0` |
| USB camera | `/dev/video0` (typical) |

> Always use `by-id` paths in launch files — `/dev/ttyUSBx` indices change on reboot.

---

## 9. Boot & Startup Procedure

### Full Stack Launch

```bash
# On Pi
roslaunch rodong rodong.launch
```

This starts all 9 nodes:

| Node | Function |
|------|----------|
| `xycar_ultrasonic` | 8-beam distance sensing |
| `xycar_imu` | Raw IMU data |
| `imu_filter` | Madgwick/complementary filter |
| `usb_cam` | Camera capture |
| `aruco_detector` | ArUco marker pose estimation → `/aruco_pose` |
| `yolo_node` | YOLOv8n ONNX inference → `/yolo_result` |
| `vfh_planner` | VFH-like obstacle avoidance → `/vfh_cmd` |
| `xycar_motor` | Motor/steering PWM output |
| `rodong_main` | Main FSM (IDLE→BUG_DRIVE→MARKER_APPROACH→UTURN) |

### Teleop Keys (during operation)

| Key | Action |
|-----|--------|
| `i` | Start (IDLE → BUG_DRIVE) |
| `s` | Stop |
| `m` | Force MARKER_APPROACH |
| `u` | Force UTURN (test) |
| `q` | Quit |

### Ultrasonic-Only Launch (debug)

```bash
roslaunch ~/xycar_ws/src/xycar_device/xycar_ultrasonic_1x8/launch/xycar_ultrasonic.launch
```

> Absolute path is required — relative path resolution fails for this package.

---

## 10. Known Hardware Behaviors & Caveats

| # | Issue | Detail |
|---|-------|--------|
| 1 | ESC beeping | Normal for this ESC; not a fault |
| 2 | Motor deadband | Speed values 8–15 produce no movement; use 20+ |
| 3 | Reverse sequence | 4-step neutral/reverse sequence required every time |
| 4 | Voltage sag under load | Normal; low battery (<5.6 V) causes motor faults |
| 5 | Desk-surface false obstacle | Ultrasonics detect desk as wall; lower threshold for bench tests |
| 6 | Pi boot conflict | `start_x=1` + `gpu_mem=128` incompatible with vc4-kms-v3d |
| 7 | Mini PC AER errors | `pci=noaer` required in GRUB for stable boot |
| 8 | OpenCV DNN + YOLOv8 | Fails on Pi 4.5.3; use `onnxruntime` only |
| 9 | ArUco API mismatch | Pi uses Old API; do not use `ArucoDetector` class |
| 10 | Static IP not persistent | Must configure via netplan with NetworkManager renderer |
| 11 | IMU yaw sign | Must verify polarity empirically before U-turn deployment |
| 12 | `by-id` ports | Always use — `/dev/ttyUSBx` indices are not stable |

---

## 11. Wiring & Physical Layout Notes

- **Battery** connects directly to ESC power input; ESC BEC powers the servo and (via a regulator) the Pi.
- **FTDI adapter** bridges USB (Pi) ↔ UART (ESC control signal).
- **Ultrasonic module** connects via a separate USB-Serial adapter (CH340 chip, `1a86` vendor ID).
- **IMU** mounted flat on chassis; orientation affects yaw sign — document physical orientation if re-mounting.
- **Camera** mounted forward-facing at bumper height for ground-level obstacle and marker visibility.

---

## Appendix: SD Card Backup

A full SD card image is stored on the mini PC:

```
~/rodong_backups/xycar_original_sd/xycar_sd_backup_20260518_174004.img.gz
```

Restore with:
```bash
gunzip -c xycar_sd_backup_20260518_174004.img.gz | sudo dd of=/dev/sdX bs=4M status=progress
```

Replace `/dev/sdX` with the actual SD card device (verify with `lsblk` before writing).

---

*ADR036 — RODONG Club | Archived 2026-06-10*
