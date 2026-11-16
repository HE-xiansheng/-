#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
exec ros2 launch /home/orangepi/drone_platform/01_apps/mission_demo/launch/aruco_vision.launch.py "$@"
