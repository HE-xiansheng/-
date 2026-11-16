#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
unset PYTHONPATH
source /opt/ros/noetic/setup.bash
if [[ -f "$root/workspace/devel/setup.bash" ]];then source "$root/workspace/devel/setup.bash";fi
exec /usr/bin/python3 "$root/management/board.py" "$@"
