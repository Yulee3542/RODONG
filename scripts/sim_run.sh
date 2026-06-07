#!/usr/bin/env bash
# RODONG ROS1 시뮬 실행. 기본 헤드리스(이 박스는 GPU/디스플레이 없음).
#   scripts/sim_run.sh                  # 회피 월드, 헤드리스
#   scripts/sim_run.sh gui              # 회피 월드, Gazebo GUI (데스크톱 터미널)
#   scripts/sim_run.sh perception       # 카메라+ArUco+YOLO 월드, 헤드리스
#   scripts/sim_run.sh perception gui   # 인지 월드, GUI
# 주행 시작은 다른 터미널에서:  scripts/sim_cmd.sh INIT
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="$REPO/docker/compose.yaml"
cd "$REPO"

APP_DIR="/workspace/rodong_pi_code/patch(20260602)"
LAUNCH="rodong_sim.launch"
GUI=false

for a in "$@"; do
  case "$a" in
    perception) LAUNCH="rodong_perception.launch" ;;
    gui)        GUI=true ;;
    headless)   GUI=false ;;
  esac
done

if [ "$GUI" = "true" ]; then
  WRAP=""
  : "${DISPLAY:=:0}"; export DISPLAY
  echo "==> GUI 모드 (DISPLAY=$DISPLAY) — 데스크톱 터미널에서 실행하세요"
  command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
else
  # 헤드리스: 카메라/센서 렌더용 오프스크린 GL 컨텍스트를 xvfb 로 제공.
  WRAP="xvfb-run -a -s '-screen 0 1280x720x24'"
fi

echo "==> RODONG 시뮬 기동 (launch=$LAUNCH gui=$GUI). IDLE → 'scripts/sim_cmd.sh INIT' 로 주행."
docker compose -f "$COMPOSE" run --rm --name rodong-sim sim bash -lc "
  source /opt/ros/noetic/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  $WRAP roslaunch rodong_sim $LAUNCH gui:=$GUI app_dir:='$APP_DIR'
"
