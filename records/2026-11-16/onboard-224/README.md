# 科研飞行平台 ROS2

目录与192.168.1.235统一：01_apps应用，02_drivers驱动，03_tools工具，04_data数据。

当前设备：Ubuntu22.04/Humble ARM64。source ~/drone_platform/env.sh显式启用环境。无飞行程序自动启动。

相机适配：02_drivers/orbbec；飞控适配：02_drivers/sensors。官方ROS2驱动由apt安装在/opt/ros/humble，模块目录保存参数和launch，不能复制ROS1二进制直接使用。

03_tools/ros2_migration保存迁移工具/文档，04_data/logs/ros2_migration保存版本与验证记录。~/ros2_platform及根目录旧链接仅为先前环境记录兼容。
