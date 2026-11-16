#!/usr/bin/env bash
set -eo pipefail
bundle_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Package: management/build.sh; host: deployment/factory-stack/build.sh.
if [[ "$bundle_root" == */deployment ]]; then
 project_root="$(cd "$bundle_root/.." && pwd)"
 workspace="$project_root/workspaces/factory-fastlio"
 overlay="$project_root/workspaces/fastlio-velocity/devel/setup.bash"
else
 workspace="$bundle_root/workspace"
 overlay="/home/$(id -un)/drone_platform/02_drivers/sensors/setup.bash"
fi
export PYTHONNOUSERSITE=1
unset PYTHONPATH
source /opt/ros/noetic/setup.bash
[[ -f "$overlay" ]] || { echo "缺少已验证的Livox driver2环境: $overlay"; exit 1; }
source "$overlay"
mkdir -p "$workspace/build-records"
cd "$workspace"
# Package selection stays in this workspace; upstream driver/messages are reused.
catkin_make -j1 -DPYTHON_EXECUTABLE=/usr/bin/python3 2>&1 | tee "build-records/build-$(date +%Y%m%d-%H%M%S).log"
[[ -x devel/lib/fast_lio/fastlio_mapping ]] || exit 1
sha256sum devel/lib/fast_lio/fastlio_mapping | tee build-records/binary-sha256.txt

sha256sum src/fast_lio/src/laserMapping.cpp > build-records/source-sha256.txt
