#!/usr/bin/env bash
# 机载端 ego_planner 一键编译脚本 (Release 优化，纯 CPU 8核编译)
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

echo "=========================================================="
echo "正在编译机载 ego_planner 核心包 (Release -O3)..."
echo "工作空间: $WS_DIR"
echo "=========================================================="

source /opt/ros/noetic/setup.bash

# 使用 4 线程或 8 线程编译 (RK3588 8 核)
CORES=$(nproc || echo 4)
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$CORES"

echo "=========================================================="
echo "ego_planner 编译完成！"
echo "可执行程序位于: $WS_DIR/devel/lib/ego_planner/"
echo "=========================================================="
