#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
for platform_overlay in \
  /home/orangepi/drone_platform/02_drivers/sensors/workspace/install/setup.bash \
  /home/orangepi/drone_platform/02_drivers/orbbec/install/setup.bash \
  /home/orangepi/drone_platform/01_apps/fast_lio/workspace/install/setup.bash \
  /home/orangepi/drone_platform/01_apps/mission_demo/workspace/install/setup.bash \
  /home/orangepi/drone_platform/01_apps/ego_planner/workspace/install/setup.bash; do
  if [[ -f "$platform_overlay" ]]; then source "$platform_overlay"; fi
done
unset platform_overlay
