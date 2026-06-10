# rodong_sim — simple box-model Gazebo simulation

A minimal simulation to verify that the **RODONG FSM / avoidance logic
(`rodong_main.py` + `vfh_planner.py`)** works, without the real Xycar hardware/sim
packages. It uses a box-shaped vehicle that is **not the real Xycar model** but
**matches the same topic interface**.

```
Gazebo(box+lidar+IMU) ──/scan──┐                 ┌──/xycar_ultrasonic──► vfh_planner ──/rodong/vfh_cmd──► rodong_main
                               ├─ sim_bridge ────┤                                                          │
          planar_move ◄──/cmd_vel──┘             └◄────────────────── /xycar_motor ◄──────────────────────┘
```

- `/scan` (360° lidar) → sampled at 8 beam angles `[-90,-45,45,0,90,135,180,-135]°` → `/xycar_ultrasonic` (cm)
- `/xycar_motor` (speed, angle) → bicycle model → `(v, yaw_rate)` → `/cmd_vel`
- IMU → `/imu/data` (input for the heading-recovery RECOVER phase)
- camera/YOLO/ArUco omitted → with no marker visible, `rodong_main` only does BUG_DRIVE avoidance

> ⚠️ This package was written and verified on a machine without ROS/Gazebo (only up to
> syntax/XML well-formedness). The first real Gazebo run in the environment below may need
> fine-tuning of plugin names/gains.

## Requirements
- ROS1 (Noetic recommended; Melodic mostly works too)
- `gazebo_ros`, `gazebo_plugins` (`libgazebo_ros_planar_move`, `libgazebo_ros_laser`, `libgazebo_ros_imu_sensor`)
- Python: `rospy`, `tf`

This dev box does not have ROS1 installed natively, so it runs via **Docker (ROS1 Noetic +
Gazebo 11)**. (`docker/`, `scripts/sim_*.sh` are at the repo root.)

## Build & run — Docker (recommended, headless)
```bash
# 1) build the image + catkin workspace
scripts/sim_build.sh
# 2) start the sim (headless = gzserver only; pass 'gui' for a Gazebo window)
scripts/sim_run.sh            # or: scripts/sim_run.sh gui
# 3) in another terminal, start driving (IDLE → BUG_DRIVE)
scripts/sim_cmd.sh INIT       # STOP / UTURN / MANUAL work the same way
```
- `sim_build.sh` symlinks `rodong_sim`/`xycar_msgs` into `catkin_ws/src` and runs `catkin_make`
  inside the container. Build artifacts remain on the host in `catkin_ws/`.
- Headless uses a CPU ray lidar (works without a GPU); `xvfb` is injected for the render path.

## Build & run — when ROS1 is installed natively
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s /home/yulee23/RODONG/rodong_sim .
ln -s /home/yulee23/RODONG/xycar_msgs .          # skip if it already exists
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch rodong_sim rodong_sim.launch app_dir:=/home/yulee23/RODONG/rodong_pi_code/patch\(20260602\)
# in another terminal:
rostopic pub -1 /rodong/cmd std_msgs/String "data: 'INIT'"
```

## Perception test — camera + ArUco + YOLO
```bash
scripts/sim_run.sh perception      # camera world (front ArUco id=1 panel) + aruco_detector + yolo_node
scripts/sim_cmd.sh INIT            # approach the marker → MARKER_APPROACH → UTURN
```
- The box model gets a camera (`/usb_cam/image_raw`, 640x480@15Hz, ~10Hz headless via software GL).
- `worlds/perception.world` places `models/aruco_marker` (DICT_4X4_50, id=1, 0.15m) 1.3m ahead.
- **ArUco**: real detection — publishes `/aruco_pose` (distance/bearing/pixel_w) → `rodong_main`
  `BUG_DRIVE → MARKER_APPROACH →` (pixel_w≥`MARKER_CLOSE_PX`) `→ UTURN(K-turn)`. (verified)
- **YOLO**: node-integration only — loads `rodong.onnx` with `onnxruntime` and infers camera
  frames (no error). The model is a real stairs/ramp (`CLIMB`/`AVOID`) detector, so **it does
  not detect Gazebo primitives**. To see the downstream avoidance, publish `/rodong/yolo`
  directly to inject it (`std_msgs/Float32MultiArray`, `[cls, conf, cx_norm, cy_norm, bottom_ratio]`).

## What you can verify
- **BUG_DRIVE avoidance**: VFH sector selection in front of a box obstacle → steer around → pass
- **Emergency reverse**: front dead-end wall (`wall_front`) within `SONAR_EMERGENCY` → `reverse_motor`
- **Straighten after reverse (this fix)**: at the end of reverse, `drive(0,0)` straightens the wheels
  and briefly stops before resuming forward → verify the wheels are not cranked at start-up
- **RECOVER**: return to the original heading via IMU yaw
- **Perception pipeline**: camera → ArUco detection → marker approach → IMU K-turn (the "Perception test" above)

Observe logs:
```bash
rostopic echo /xycar_ultrasonic     # 8 beams (cm)
rostopic echo /xycar_motor          # FSM output (speed, angle)
rostopic echo /cmd_vel              # bridge conversion result
```

## Tuning (`sim_bridge` parameters)
| param | default | meaning |
|---|---|---|
| `speed_to_ms` | 0.03 | motor units → m/s (25 → 0.75 m/s) |
| `wheelbase` | 0.30 | bicycle model L [m]. smaller = sharper turns |
| `max_steer_deg` | 60 | steering clamp to avoid tan blow-up |
| `max_cm` | 300 | reported distance when the lidar gets no return |

## Limitations / notes
- `planar_move` is a **kinematic** (no force/friction) unicycle. The real car's ESC deadband and
  servo mechanical lag are not reproduced, so it is suitable for verifying the **command-sequence
  logic** (reverse→`drive(0,0)`→stop→forward) rather than the *physical* effect of the
  "straighten after reverse" fix.
- Sign convention: Xycar `+angle=right turn` → `/cmd_vel` negative yaw (clockwise, same as the
  real car). If avoidance turns the wrong way in Gazebo, flip the `yaw_rate` sign in
  `sim_bridge.cb_motor`.
- The plugin `.so` names can differ per distribution (e.g. GPU lidar `libgazebo_ros_gpu_laser`).
  Replace with the actual name in the installed `gazebo_plugins` if loading fails.
