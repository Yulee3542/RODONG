#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════
# RODONG12 배포 스크립트 (미니PC에서 실행)
#   1) Pi 기존 파일 백업
#   2) 새 파일 4개 업로드 (vfh_planner, rodong_main, yolo_node, rodong.launch)
#   3) 죽은 코드 삭제 (rodong_main.launch)
#   4) 실행권한 부여 + catkin build
#
# 사용법: bash deploy_rodong12.sh
#   - 새 파일들이 현재 디렉토리에 있어야 함
#   - Pi 비밀번호(xytron) 입력 필요
# ════════════════════════════════════════════════════════════
set -e

PI=pi@192.168.10.2
SCRIPTS=/home/pi/xycar_ws/src/rodong/scripts
LAUNCH=/home/pi/xycar_ws/src/rodong/launch
LOCAL_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "═══ 0. 로컬 파일 확인 ═══"
for f in vfh_planner.py rodong_main.py yolo_node.py rodong.launch; do
  if [ ! -f "$LOCAL_DIR/$f" ]; then
    echo "  ✗ 없음: $f  (이 스크립트와 같은 폴더에 있어야 함)"; exit 1
  fi
  echo "  ✓ $f"
done

echo
echo "═══ 1. Pi 기존 파일 백업 ═══"
TS=$(date +%Y%m%d_%H%M%S)
ssh $PI "mkdir -p ~/rodong_backups/before_rodong12_$TS && \
  cp $SCRIPTS/*.py ~/rodong_backups/before_rodong12_$TS/ 2>/dev/null; \
  cp $LAUNCH/*.launch ~/rodong_backups/before_rodong12_$TS/ 2>/dev/null; \
  echo '  백업 위치: ~/rodong_backups/before_rodong12_$TS' && \
  ls ~/rodong_backups/before_rodong12_$TS/"

echo
echo "═══ 2. 새 파일 업로드 ═══"
scp "$LOCAL_DIR/vfh_planner.py"  "$LOCAL_DIR/rodong_main.py" \
    "$LOCAL_DIR/yolo_node.py"    $PI:$SCRIPTS/
scp "$LOCAL_DIR/rodong.launch"   $PI:$LAUNCH/
echo "  업로드 완료"

echo
echo "═══ 3. 죽은 코드 삭제 (rodong_main.launch) ═══"
ssh $PI "rm -f $LAUNCH/rodong_main.launch && echo '  삭제: rodong_main.launch'"

echo
echo "═══ 4. 실행권한 + 빌드 ═══"
ssh $PI "chmod +x $SCRIPTS/*.py && \
  cd ~/xycar_ws && \
  catkin_make -DCATKIN_BLACKLIST_PACKAGES='test_' >/dev/null 2>&1 && \
  echo '  빌드 완료' || echo '  ⚠ 빌드 경고 (수동 확인 필요)'"

echo
echo "═══ 완료 ═══"
echo "다음 단계:"
echo "  1) ONNX 모델을 Pi로 복사:"
echo "     ssh $PI 'mkdir -p ~/xycar_ws/src/rodong/models'"
echo "     scp <로컬_rodong.onnx> $PI:~/xycar_ws/src/rodong/models/rodong.onnx"
echo "  2) Pi에서 실행:"
echo "     ssh $PI"
echo "     roslaunch rodong rodong.launch"
