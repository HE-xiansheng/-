# 源码与驱动归档完整性

本阶段核对机载 c3dff85 跟踪文件和电脑/GitHub平台目录。发现830个机载已跟踪普通文件尚未入库，现已全部补齐，逐文件SHA256一致；已有匹配文件没有覆盖。机载的config/humble_ws是兼容路径别名，未作为电脑路径复制。

新增内容包含官方Livox ROS2 driver、实际FAST-LIO及子库源码、Orbbec ROS2 wrapper/设备配置、传感器启动入口、云台只读RTSP检查和许可证，以及由COLCON_IGNORE隔离的ROS1参考源码。ROS1参考属于历史对照，不是当前ROS2运行程序。部分上游示例图片/测试代码随实际跟踪源码原样保留。

另补齐参考机Livox SDK2 1.3.0源码（221个清单文件逐个核对）及原有Orbbec v2.10.6源码归档，后者包括Git未跟踪的SDK头文件、ARM64库和许可证。归档SHA256见verified.json。RKNN实际模型/ARM64运行库既有原生副本与参考副本按原路径保留；不将历史armhf文件误用于RK3588。

本阶段只补电脑/GitHub的缺失资料，不改变机载运行代码/服务，不执行新的硬件任务。无需回退机载；若不需要这些新增仓库资料，可撤销本次归档提交，既有部署不受影响。字节一致性证明归档与已测试机载源码一致，不等于新硬件整链验收。

## 仍需准备的运行配置

当前start_sensors.sh使用mavros_telemetry.launch.py，插件白名单仅command/sys_status/sys_time/imu/local_position/global_position/param。这是已有遥测验证入口；完整平台还需要vision_pose、setpoint_raw和rc_io等接口。下一阶段准备独立完整平台配置和模拟MAVLink验证，不在未确认情况下切换正式服务。

.235目前工程在/home/orangepi/drone_platform，Ubuntu20.04/Noetic。原始something1017/TECH路径已不存在，不能以该历史路径判断源码缺失。原Safina_sensor.service还启动usb_cam和RTSP/Livox；独立USB相机与物理云台仍待适配/接线验证。

## 归档过程记录

普通文件复制时遇到已被同步的同名文件，断言终止；随后重新核对830个文件均已存在且SHA256一致，才继续。首次接收tar前缀混入两行路径别名清单；电脑端验证这两行及pax头后剥离，再仅提取普通文件。没有将目标机的兼容符号链接展开到电脑绝对路径。
