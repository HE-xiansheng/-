#!/usr/bin/env bash
# 独立ROS2修复版入口，保留原桌面入口和机载服务；定位与任务由界面按需启动。
set -euo pipefail
GCS_REPAIRED_BUNDLE="${GCS_BUNDLE_DIR:-$HOME/下载/科研飞行平台-ROS2地面站/drone-gcs-ros2-20261117-ros2-r5-linux-amd64}"
test -x "$GCS_REPAIRED_BUNDLE/start-groundstation.sh"
export GCS_STATE_DIR="$HOME/.local/share/drone-gcs-ros2-r5"
export GCS_RAW_UDP_PORT=19115
export XDG_CONFIG_HOME="$HOME/.config/drone-gcs-ros2-r5"
exec "$GCS_REPAIRED_BUNDLE/start-groundstation.sh" --host 192.168.1.224 \
    --cloud-port 9100 --pose-port 9101 --preflight-port 9103 "$@"
