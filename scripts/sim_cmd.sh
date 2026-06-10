#!/usr/bin/env bash
# Send an FSM command to the running RODONG sim container (docker exec).
#   scripts/sim_cmd.sh INIT     # IDLE → BUG_DRIVE (start driving/avoidance)
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
