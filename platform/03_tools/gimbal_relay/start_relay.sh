#!/usr/bin/env bash
set -euo pipefail
cd /home/orangepi/drone_platform/03_tools/gimbal_relay
exec ./mediamtx ./gimbal.yml
