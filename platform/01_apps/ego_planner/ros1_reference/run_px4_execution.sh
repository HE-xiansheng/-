#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 PYTHONNOUSERSITE=1
unset ROS_HOSTNAME
exec /usr/bin/python3 -B "$WS_DIR/px4_execution/manage.py" "${1:-status}"
