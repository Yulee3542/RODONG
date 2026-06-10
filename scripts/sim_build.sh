#!/usr/bin/env bash
# RODONG ROS1 sim: build the docker image + the catkin workspace.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="$REPO/docker/compose.yaml"
cd "$REPO"

# Ensure the package symlinks inside catkin_ws/src (relative paths → valid on host and container).
mkdir -p catkin_ws/src
ln -sfn ../../rodong_sim catkin_ws/src/rodong_sim
ln -sfn ../../xycar_msgs catkin_ws/src/xycar_msgs

echo "==> building docker image (ros1 noetic + gazebo 11)"
docker compose -f "$COMPOSE" build

echo "==> catkin_make (xycar_msgs + rodong_sim)"
docker compose -f "$COMPOSE" run --rm sim \
  bash -lc "cd /workspace/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make"

echo "==> done. Run:  scripts/sim_run.sh"
