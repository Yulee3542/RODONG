#!/usr/bin/env bash
# 실행 중인 RODONG 시뮬 컨테이너로 FSM 명령을 보낸다 (docker exec).
#   scripts/sim_cmd.sh INIT     # IDLE → BUG_DRIVE (주행/회피 시작)
#   scripts/sim_cmd.sh STOP
#   scripts/sim_cmd.sh UTURN
#   scripts/sim_cmd.sh MANUAL
set -euo pipefail
CMD="${1:-INIT}"
docker exec rodong-sim bash -lc "
  source /opt/ros/noetic/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  rostopic pub -1 /rodong/cmd std_msgs/String \"data: '$CMD'\"
"
