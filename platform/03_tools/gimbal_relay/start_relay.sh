#!/usr/bin/env bash
set -euo pipefail
# 缺省保留现场配置；可传入独立配置执行隔离地面验证。
if (( $# > 1 )); then
  echo 'Usage: start_relay.sh [config.yml]' >&2
  exit 2
fi
relay_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$relay_root/mediamtx" "${1:-$relay_root/gimbal.yml}"
