#!/usr/bin/env bash
# Run the RODONG ROS1 simulation. Headless by default (this box has no GPU/display).
#   scripts/sim_run.sh                  # avoidance world, headless
#   scripts/sim_run.sh gui              # avoidance world, Gazebo GUI (desktop terminal)
#   scripts/sim_run.sh perception       # camera+ArUco+YOLO world, headless
#   scripts/sim_run.sh perception gui   # perception world, GUI
#   scripts/sim_run.sh full gui         # full scenario (avoid->front ArUco->U-turn->rear ArUco), GUI
# Start driving from another terminal:  scripts/sim_cmd.sh INIT
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
    full)       LAUNCH="rodong_full.launch" ;;
    gui)        GUI=true ;;
    headless)   GUI=false ;;
  esac
done

if [ "$GUI" = "true" ]; then
  WRAP=""
  : "${DISPLAY:=:0}"; export DISPLAY
  echo "==> GUI mode (DISPLAY=$DISPLAY) — run this from a desktop terminal"
  command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
else
  # Headless: provide an offscreen GL context via xvfb for camera/sensor rendering.
  WRAP="xvfb-run -a -s '-screen 0 1280x720x24'"
fi

echo "==> Starting RODONG sim (launch=$LAUNCH gui=$GUI). IDLE → drive with 'scripts/sim_cmd.sh INIT'."
docker compose -f "$COMPOSE" run --rm --name rodong-sim sim bash -lc "
  source /opt/ros/noetic/setup.bash
  source /workspace/catkin_ws/devel/setup.bash
  # The model path in .bashrc is not read by a non-interactive login shell -> export it directly here
  export GAZEBO_MODEL_PATH=/workspace/rodong_sim/models\${GAZEBO_MODEL_PATH:+:\$GAZEBO_MODEL_PATH}
  $WRAP roslaunch rodong_sim $LAUNCH gui:=$GUI app_dir:='$APP_DIR'
"
