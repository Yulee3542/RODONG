#!/bin/bash
# deploy_rodong13.sh — Pi에 RODONG13 배포
# 사용법: bash deploy_rodong13.sh

PI="pi@192.168.10.2"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REMOTE_SCRIPTS="/home/pi/xycar_ws/src/rodong/scripts"
REMOTE_LAUNCH="/home/pi/xycar_ws/src/rodong/launch"
BACKUP_DIR="/home/pi/rodong_backups/rodong12_$(date +%Y%m%d_%H%M%S)"

set -e

echo "======================================================"
echo "  RODONG13 배포 시작"
echo "======================================================"

# 1. 백업
echo "[1/4] RODONG12 백업: $BACKUP_DIR"
ssh $PI "mkdir -p $BACKUP_DIR && \
  cp $REMOTE_SCRIPTS/rodong_main.py $BACKUP_DIR/ 2>/dev/null || true && \
  cp $REMOTE_LAUNCH/rodong.launch    $BACKUP_DIR/ 2>/dev/null || true"

# 2. 업로드
echo "[2/4] 파일 업로드"
scp "$SCRIPT_DIR/rodong_main.py"    $PI:$REMOTE_SCRIPTS/rodong_main.py
scp "$SCRIPT_DIR/rodong_teleop.py"  $PI:$REMOTE_SCRIPTS/rodong_teleop.py
scp "$SCRIPT_DIR/rodong.launch"     $PI:$REMOTE_LAUNCH/rodong.launch

# 3. 실행 권한
echo "[3/4] 실행 권한 부여"
ssh $PI "chmod +x $REMOTE_SCRIPTS/rodong_main.py $REMOTE_SCRIPTS/rodong_teleop.py"

# 4. catkin_make
echo "[4/4] catkin_make"
ssh $PI "source /opt/ros/noetic/setup.bash && \
  cd /home/pi/xycar_ws && \
  catkin_make -DCATKIN_BLACKLIST_PACKAGES='test_' 2>&1 | tail -5"

echo ""
echo "======================================================"
echo "  배포 완료!"
echo ""
echo "  실행 순서:"
echo "  [터미널 1] roslaunch rodong rodong.launch"
echo "  [터미널 2] rosrun rodong rodong_teleop.py"
echo ""
echo "  텔레옵 키:"
echo "    i  → 자율주행 시작"
echo "    m  → 좌표 입력 모드 (x y, cm 단위)"
echo "    s  → 정지"
echo "    q  → 종료"
echo "======================================================"
