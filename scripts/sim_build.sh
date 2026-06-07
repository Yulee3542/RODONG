#!/usr/bin/env bash
# RODONG ROS1 시뮬: 도커 이미지 빌드 + catkin 워크스페이스 빌드.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="$REPO/docker/compose.yaml"
cd "$REPO"

# catkin_ws/src 안의 패키지 심볼릭 링크 보장 (상대경로 → 호스트/컨테이너 양쪽 유효).
mkdir -p catkin_ws/src
ln -sfn ../../rodong_sim catkin_ws/src/rodong_sim
ln -sfn ../../xycar_msgs catkin_ws/src/xycar_msgs

echo "==> 도커 이미지 빌드 (ros1 noetic + gazebo 11)"
docker compose -f "$COMPOSE" build

echo "==> catkin_make (xycar_msgs + rodong_sim)"
docker compose -f "$COMPOSE" run --rm sim \
  bash -lc "cd /workspace/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make"

echo "==> 완료. 실행:  scripts/sim_run.sh"
