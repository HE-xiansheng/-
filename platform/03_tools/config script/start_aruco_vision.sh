#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
export QT_XCB_GL_INTEGRATION=none
export LIBGL_ALWAYS_SOFTWARE=1
export QT_OPENGL=software
exec python3 /home/orangepi/drone_platform/01_apps/mission_demo/tools/aruco_vision.py "$@"
