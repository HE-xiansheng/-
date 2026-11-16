#!/usr/bin/env bash
set -e
source /home/orangepi/drone_platform/env.sh
exec ros2 launch /home/orangepi/drone_platform/02_drivers/sensors/launch/sensors.launch.py
