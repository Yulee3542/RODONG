# RODONG 🚗

Autonomous RC-car project built on the **Xycar C-model** platform (Raspberry Pi 4B + ROS Noetic).
The car drives itself around obstacles, homes in on an **ArUco marker** goal, performs IMU-based
**U-turns**, and uses a **YOLOv8** vision model to decide whether to *climb* a ramp or *avoid* stairs.

> Current iteration: **RODONG13 / rev3**. Source comments are in Korean.

---

## How it works

The system is a set of ROS nodes communicating over topics. A VFH+ planner fuses ultrasonic,
camera (ArUco + YOLO), and IMU data; a central state machine arbitrates between autonomous
driving, marker approach, U-turns, and manual control.

```
                 ┌──────────────┐
   /usb_cam ───► │ aruco_detector│──► /aruco_pose ─┐
                 └──────────────┘                  │
                 ┌──────────────┐                  ▼
   /usb_cam ───► │  yolo_node    │──► /rodong/yolo ─┤      ┌───────────────┐
                 └──────────────┘                  ├────► │  vfh_planner  │──► /rodong/vfh_cmd ─┐
   /xycar_ultrasonic ────────────────────────────► ┘      └───────────────┘                    │
                                                                                                ▼
   /rodong/cmd (teleop) ─────────────────────────────────────────────►  ┌──────────────┐
   /rodong/manual_goal (teleop) ─────────────────────────────────────►  │ rodong_main  │──► /xycar_motor
   /imu ──────────────────────────────────────────────────────────────► └──────────────┘
```

### Nodes

| Node | Role |
|------|------|
| **`aruco_detector.py`** | Detects ArUco marker `ID=1` from the USB cam, estimates pixel-based range/bearing → publishes `/aruco_pose` (the navigation goal). |
| **`yolo_node.py`** | Runs `rodong.onnx` via **onnxruntime** on camera frames. 2 classes — `CLIMB` (ramp) and `AVOID` (stairs). Input 320 px, frame-skipping to limit Pi CPU load. |
| **`vfh_planner.py`** | The avoidance brain. Fuses 8-beam ultrasonic + ArUco goal + YOLO into a **VFH+** histogram (7 front sectors, ±90°) to compute `(speed, angle)` → `/rodong/vfh_cmd`. |
| **`rodong_main.py`** | State machine + motor arbitration. Handles emergency reverse, avoidance-hold, IMU-yaw heading recovery, and K-turn U-turns. Publishes `/xycar_motor`. |
| **`rodong_teleop.py`** | Keyboard control node (run in a separate terminal). |

### State machine

```
IDLE → BUG_DRIVE ⇄ { MARKER_APPROACH → UTURN } / MANUAL_DRIVE / STOP
```

Avoidance sub-states: `NONE` (VFH following) → `STEER` (hold avoidance steer) → `RECOVER`
(return to original IMU heading).

### Teleop keys

| Key | Action |
|-----|--------|
| `i` | INIT — start autonomous driving |
| `s` | STOP — full stop |
| `m` | MANUAL — enter relative-coordinate (cm) input mode |
| `u` | UTURN — U-turn test |
| `q` / `Ctrl+C` | quit |

---

## Vision model

YOLOv8n trained **from scratch on CPU** (mini-PC i5-6500T, no GPU) and exported to ONNX for the Pi.

- **Dataset:** Roboflow [*Stairs & ramps*](https://universe.roboflow.com/vasile-grosu-uslqx/stairs-ramps) (Public Domain)
  - `ramp` → **CLIMB** (class 0)
  - `stairs` → **AVOID** (class 1)
- **Training:** `yolov8n`, `imgsz=320`, ~50 epochs, batch 8 (CPU-optimized).
- **Output:** `2. OWOD for Rodong/rodong_yolo/rodong.onnx` (deployed to the Pi).

Train / export pipeline:

```bash
pip install ultralytics roboflow
export ROBOFLOW_API_KEY=...      # free key from app.roboflow.com → Settings → API
python3 rodong_train/rodong_train.py --all   # download + train + export ONNX
```

---

## Repository layout

```
RODONG/
├── rodong_pi_code/              # ROS code that runs ON the Pi
│   ├── patch(20260602)/         #   ← latest version
│   └── downloaded(20260601)/    #   prior pull + dated .bak backups
├── 2. OWOD for Rodong/
│   └── rodong_yolo/             # YOLO training workspace
│       ├── dataset/             #   Roboflow stairs/ramps data
│       ├── runs/                #   training curves, metrics, sample preds
│       └── rodong.onnx          #   exported model
├── rodong_train/                # training script + launch + deploy scripts
│   ├── rodong_train.py
│   ├── rodong.launch
│   └── deploy_rodong13.sh       # SCP code to the Pi + catkin_make
└── original files/              # (gitignored) 5.3 GB dd image of the Pi SD card
```

> **Not in git:** the 5.3 GB SD-card backup image (`original files/`) and the Python virtualenv
> (`rodong_train_venv/`) are excluded via `.gitignore` — they exceed GitHub's file-size limits.

---

## Deploy to the Pi

The launch file and nodes live under `~/xycar_ws/src/rodong/` on the Pi (`pi@192.168.10.2`).

```bash
bash rodong_train/deploy_rodong13.sh   # backup → scp → chmod → catkin_make
```

Then, on the Pi:

```bash
roslaunch rodong rodong.launch                 # device + application nodes
rosrun rodong rodong_teleop.py                 # in a separate terminal
```

---

## Hardware

- **Platform:** Xycar C-model (Raspberry Pi 4B aarch64, Raspberry Pi OS / ROS Noetic)
- **Sensors:** USB camera (640×480), 8-beam ultrasonic array, IMU
- **Actuation:** `xycar_motor` (speed + steering, ±90°)
