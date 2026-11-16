#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
exec ros2 launch /home/orangepi/drone_platform/01_apps/fast_lio/launch/localization.launch.py "$@"
