#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
if [[ -f /home/orangepi/drone_platform/02_drivers/usb_camera/workspace/install/setup.bash ]]; then
  source /home/orangepi/drone_platform/02_drivers/usb_camera/workspace/install/setup.bash
fi
exec ros2 launch /home/orangepi/drone_platform/02_drivers/sensors/launch/platform_sensors.launch.py "$@"
