#!/usr/bin/env bash
set -eo pipefail
source /home/orangepi/drone_platform/env.sh
exec /usr/bin/python3 -B /home/orangepi/drone_platform/01_apps/fast_lio/manual_session.py relay "$@"
