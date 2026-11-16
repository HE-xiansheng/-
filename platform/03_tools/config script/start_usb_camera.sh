#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
source /home/orangepi/drone_platform/02_drivers/usb_camera/workspace/install/setup.bash
exec ros2 launch /home/orangepi/drone_platform/02_drivers/usb_camera/launch/usb_camera.launch.py "$@"
