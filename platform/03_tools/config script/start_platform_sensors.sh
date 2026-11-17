#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
exec ros2 launch /home/orangepi/drone_platform/02_drivers/sensors/launch/platform_sensors.launch.py "$@"
