#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export DISABLE_ROS1_EOL_WARNINGS=1
unset ROS_HOSTNAME
if [[ -z "${DISPLAY:-}" ]]; then
    printf '%s\n' 'Run this command from the onboard graphical desktop (HDMI or NoMachine).'
    exit 1
fi
exec roslaunch "$WS_DIR/rviz/observe.launch"
