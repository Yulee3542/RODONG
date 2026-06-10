#!/bin/bash
# deploy_rodong13.sh — deploy the latest Pi code to the Pi
# Source of truth: rodong_pi_code/patch(20260602)/  (refactored modular layout)
# Usage: bash deploy_rodong13.sh

PI="pi@192.168.10.2"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC="$REPO/rodong_pi_code/patch(20260602)"
REMOTE_PKG="/home/pi/xycar_ws/src/rodong"
REMOTE_SCRIPTS="$REMOTE_PKG/scripts"
REMOTE_LAUNCH="$REMOTE_PKG/launch"
BACKUP_DIR="/home/pi/rodong_backups/before_rodong13_$(date +%Y%m%d_%H%M%S)"

# Node entry points + ROS-independent core modules (imported by the nodes).
# All must live in scripts/ so runtime imports resolve.
SCRIPTS=(
  rodong_main.py
  rodong_teleop.py
  vfh_planner.py
  aruco_detector.py
  yolo_node.py
  line_detector.py
  rodong_config.py
  rodong_geometry.py
  rodong_sonar.py
  rodong_control.py
)

set -e

echo "======================================================"
echo "  Starting RODONG13 deployment"
echo "  source: $SRC"
echo "======================================================"

# 0. Check local files
for f in "${SCRIPTS[@]}" rodong.launch CMakeLists.txt; do
  [ -f "$SRC/$f" ] || { echo "  ✗ missing: $SRC/$f"; exit 1; }
done

# 1. Backup
echo "[1/4] Back up current Pi code: $BACKUP_DIR"
ssh $PI "mkdir -p $BACKUP_DIR && \
  cp -r $REMOTE_SCRIPTS $BACKUP_DIR/scripts 2>/dev/null || true && \
  cp -r $REMOTE_LAUNCH  $BACKUP_DIR/launch  2>/dev/null || true && \
  cp $REMOTE_PKG/CMakeLists.txt $BACKUP_DIR/ 2>/dev/null || true"

# 2. Upload
echo "[2/4] Upload files"
ssh $PI "mkdir -p $REMOTE_SCRIPTS $REMOTE_LAUNCH"
for f in "${SCRIPTS[@]}"; do
  scp "$SRC/$f" "$PI:$REMOTE_SCRIPTS/$f"
done
scp "$SRC/rodong.launch"   "$PI:$REMOTE_LAUNCH/rodong.launch"
scp "$SRC/CMakeLists.txt"  "$PI:$REMOTE_PKG/CMakeLists.txt"

# 3. Executable permission
echo "[3/4] Grant executable permission"
ssh $PI "chmod +x $REMOTE_SCRIPTS/*.py"

# 4. catkin_make
echo "[4/4] catkin_make"
ssh $PI "source /opt/ros/noetic/setup.bash && \
  cd /home/pi/xycar_ws && \
  catkin_make -DCATKIN_BLACKLIST_PACKAGES='test_' 2>&1 | tail -5"

echo ""
echo "======================================================"
echo "  Deployment complete!"
echo ""
echo "  Run order:"
echo "  [terminal 1] roslaunch rodong rodong.launch"
echo "  [terminal 2] rosrun rodong rodong_teleop.py"
echo ""
echo "  Teleop keys:"
echo "    i  -> start autonomous driving"
echo "    m  -> coordinate input mode (x y, in cm)"
echo "    s  -> stop"
echo "    q  -> quit"
echo ""
echo "  Note: copy the ONNX model separately if it changed:"
echo "    scp '$REPO/rodong_yolo/rodong.onnx' $PI:$REMOTE_PKG/models/rodong.onnx"
echo "======================================================"
