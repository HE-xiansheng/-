#!/usr/bin/env bash
# 手册命令和地面站共用定位实现及管理会话。
set -eo pipefail
localization_root="$HOME/drone_platform/01_apps/fast_lio"
export PYTHONNOUSERSITE=1 ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
unset PYTHONPATH ROS_NAMESPACE
source /opt/ros/noetic/setup.bash
source "$HOME/drone_platform/02_drivers/sensors/setup.bash"
source "$localization_root/workspace/devel/setup.bash" --extend
# 子进程的控制台输出按行刷新，避免重定向到日志后长时间不显示。
# BEGIN configure_livox ROS environment
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=orangepi5ultra
export PYTHONDONTWRITEBYTECODE=1
unset ROS_IP
# END configure_livox ROS environment
exec /usr/bin/stdbuf -oL -eL /usr/bin/python3 "$localization_root/management/manual_session.py" "$@"
