# 科研飞行平台 ROS2 迁移

将Orange Pi 5 Ultra参考机`.235`的完整平台迁入`.224`，使用Ubuntu22.04 ARM64 / ROS2 Humble。**迁移进行中，尚未完成实物整链验收或正式入口切换。**

## 当前状态（2026-11-16）

机载`.224`从eMMC启动Ubuntu22.04，Humble和NoMachine8.5.3已安装。当前部署源码`bd6c781`，工作区干净。最近只读检查未发现Safina_sensor.service，也没有运行相机、MAVROS、FAST-LIO或本轮测试进程；不能将以前的短时数据验证当作当前传感器正在运行。以太网口仍NO-CARRIER。

| 模块 | 已有直接证据 | 仍需验证 |
| --- | --- | --- |
| Gemini335L | 官方wrapper/SDK2.10.6，实际约30Hz，当前设备内部标定及相机→NPU预览 | 机体外参、度量精度和完整任务 |
| Livox / FAST-LIO | SDK2 1.3.0、ROS2 driver1.2.6构建；旧机实采雷达/IMU回放实际FAST-LIO | 新机物理雷达采集、网络/时钟及运动精度 |
| 定位转发 / MAVROS2 | 旧二进制71条几何对照；原生100帧初始化；MAVROS2 2.16.0物理未解锁遥测和22项实际MAVLink接口检查 | 实际安装外参、真实PX4外部视觉融合与时间同步 |
| 定位到PX4仿真 | 旧机实采回放→实际FAST-LIO/转发/MAVROS→官方PX4 1.16.0，18项；564条源戳/坐标匹配，EV融合与停流检查 | 回放与软件仿真不替代新雷达、真实飞控和轨迹精度验收 |
| EGO规划 | 原生消息、地图/优化库、状态机、地图选点、执行器及所属会话管理隔离检查 | 实物地图/任务/地面站全链 |
| 航点 / ArUco / 人物 | 原生包、任务管理器、实际RK3588 NPU；ArUco图像到控制器21项、触发路径39项 | 物理云台/标记/运动目标、任务地面整链 |
| USB相机 / 云台视频 | 官方usb_cam0.8.1候选与API适配；MPP/RTSP合成视频及退出检查 | 独立相机未接入，物理云台与RTSP源待验 |
| Linux地面站 | 独立Qt源码/构建；任务、遥测、点云/位姿、视频、定位、规划、录制与SSH接口；原生Linux运行工具安装组件 | 完整安装包/可见桌面/实际任务及正式切换 |
| 录制与回放 | rosbag2/SQLite/CDR、分包下载、实际Qt回放与损坏包拒绝 | 与实物整链持续运行联合验证 |

PX4源码与ULog已证实：其自身同步收敛前会用接收时刻替代源时刻，本次回放约仿真时钟72秒后才保留非零源时龄。不能只凭融合标志或MAVLink源戳正确认定实机时间对齐，也不能把该时间当作实机固定等待时长。

## 证据与源码

- [完整计划](task_plan.md)、[进展](progress.md)、[发现与问题](findings.md)。
- [源码/驱动归档核对](records/2026-11-16/source-completeness/README.md)、[初期原生迁移](records/2026-11-16/native-stage-report.md)。
- [定位及旧转发对照](records/2026-11-16/localization/README.md)、[完整MAVROS候选](records/2026-11-16/mavros-platform/README.md)、[实际定位到PX4仿真](records/2026-11-16/fastlio-px4-sitl/README.md)。
- [EGO原生库](records/2026-11-16/ego-native-stage-report.md)、[状态机](records/2026-11-16/ego-fsm-stage-report.md)、[执行器](records/2026-11-16/ego-execution-stage-report.md)、[Qt规划入口](records/2026-11-16/planner-ui/README.md)。
- [物理相机/NPU](records/2026-11-16/camera-npu/README.md)、[ArUco控制链](records/2026-11-16/aruco-landing-chain/README.md)、[降落触发路径](records/2026-11-16/land-modes/README.md)、[USB相机](records/2026-11-16/usb-camera/README.md)。
- [视频](records/2026-11-16/video/README.md)、[录制回放](records/2026-11-16/recording/README.md)、[Linux ROS2部署入口](groundstation/doc/ros2_deployment.md)、[运行工具安装核对](records/2026-11-16/runtime-install-audit/README.md)。

`platform/`保存机载运行源码及配置，`groundstation/`是独立地面站适配副本，`tools/`是电脑端验证脚本。ROS1参考、旧测试和安装器仍在源码中供对照；不得按历史说明覆盖ROS2入口。Linux运行工具组件只安装当前依赖，历史ROS1自动导航安装器不参与现有Qt调用。

新迁移报告、参数快照和验证日志仅保存在电脑/GitHub。机载保留运行代码、配置、必要运行状态与许可证，不留迁移报告。本仓库各阶段原始记录按发生时点保留，不能把其中的历史状态当作当前部署。

系统/BSP、已有服务停止及正式运行环境切换需用户确认。不自动解锁、起飞或执行飞行任务。
