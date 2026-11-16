# 科研飞行平台 ROS2 迁移

将Orange Pi 5 Ultra参考机`.235`的完整平台迁入`.224`，使用Ubuntu22.04 ARM64 / ROS2 Humble。**迁移进行中，尚未完成实物整链验收或正式入口切换。**

## 当前状态（2026-11-17）

机载`.224`已从eMMC运行Ubuntu22.04，Humble和NoMachine8.5.3已安装。当前源码版本为`b686e55`、Git干净、NoMachine正常。本次诊断后曾临时失联（IPv4无路由、IPv6超时），随后同一地址恢复，已重新确认临时目录清理和无本次测试进程；未见系统重启，断联原因尚未确定。传感器及云台relay服务候选尚未正式安装/启用；最后读取时网口DOWN、无雷达直连。

| 模块 | 已有直接证据 | 仍需验证 |
| --- | --- | --- |
| Gemini335L | 官方wrapper/SDK2.10.6，实际约30Hz，当前设备内部标定及相机→NPU预览 | 机体外参、度量精度和完整任务 |
| Livox / FAST-LIO | SDK2 1.3.0、ROS2 driver1.2.6构建；旧机实采雷达/IMU回放实际FAST-LIO | 新机物理雷达采集、网络/时钟及运动精度 |
| 定位转发 / MAVROS2 | 旧二进制71条几何对照；原生100帧初始化；MAVROS2 2.16.0物理未解锁遥测和22项实际MAVLink接口检查 | 实际安装外参、真实PX4外部视觉融合与时间同步 |
| 定位到PX4仿真 | 旧机实采回放→实际FAST-LIO/转发/MAVROS→官方PX4 1.16.0，18项；564条源戳/坐标匹配，EV融合与停流检查 | 回放与软件仿真不替代新雷达、真实飞控和轨迹精度验收 |
| EGO规划 | 原生地图/优化/FSM及执行器；一轮实际规划→MAVROS→PX4消息38项、原执行器46项回归，支持标准ROS重映射；新增启动就绪门控两轮43项与47项回归通过 | 启动消息遗漏根因、实物地图/任务/地面站全链 |
| 航点 / ArUco / 人物 | 原生包、实际RK3588 NPU；航点→MAVROS→PX4消息34项、ArUco图像控制21项/触发39项 | 物理云台/标记/运动目标、任务地面整链 |
| USB相机 / 云台视频 | usb_cam0.8.1候选；实物云台经只读隧道→新机MPP及Qt面板14项/重连通过 | 新机云台直连、独立相机未接；MPP漏收/延迟未解决 |
| Linux地面站 | 独立Qt源码/构建；任务、遥测、点云/位姿、视频、定位、规划、录制与SSH接口；完整可搬移Linux包及重定位/首帧，实际Qt视频面板与自动重连 | 可见桌面/实际任务及正式切换 |
| 录制与回放 | rosbag2/SQLite/CDR、分包下载、实际Qt回放与损坏包拒绝 | 与实物整链持续运行联合验证 |

PX4源码与ULog已证实：其自身同步收敛前会用接收时刻替代源时刻，本次回放约仿真时钟72秒后才保留非零源时龄。不能只凭融合标志或MAVLink源戳正确认定实机时间对齐，也不能把该时间当作实机固定等待时长。

EGO早期复测228/228通过，但新增两轮诊断分别遗漏前40/41条启动消息，多数发出时DDS已有2个匹配订阅；末轮MAVROS内部出口和PX4均205条、实际发布246条。问题尚未根治，严格全流验收仍失败，不能只凭订阅匹配或单轮通过认定控制链可靠。详见[启动诊断](records/2026-11-17/ego-startup-delivery/README.md)。

## 证据与源码

- [EGO/PX4消息与参数修复](records/2026-11-17/ego-mavros-wire/README.md)、[航点/PX4消息](records/2026-11-17/waypoint-mavros-wire/README.md)、[实际Qt云台面板](records/2026-11-16/gimbal-qt-live/README.md)、[完整Linux独立包](records/2026-11-16/gcs-portable-linux/README.md)。
- [完整计划](task_plan.md)、[进展](progress.md)、[发现与问题](findings.md)。
- [源码/驱动归档核对](records/2026-11-16/source-completeness/README.md)、[初期原生迁移](records/2026-11-16/native-stage-report.md)。
- [定位及旧转发对照](records/2026-11-16/localization/README.md)、[完整MAVROS候选](records/2026-11-16/mavros-platform/README.md)、[实际定位到PX4仿真](records/2026-11-16/fastlio-px4-sitl/README.md)。
- [EGO原生库](records/2026-11-16/ego-native-stage-report.md)、[状态机](records/2026-11-16/ego-fsm-stage-report.md)、[执行器](records/2026-11-16/ego-execution-stage-report.md)、[Qt规划入口](records/2026-11-16/planner-ui/README.md)。
- [物理相机/NPU](records/2026-11-16/camera-npu/README.md)、[ArUco控制链](records/2026-11-16/aruco-landing-chain/README.md)、[降落触发路径](records/2026-11-16/land-modes/README.md)、[USB相机](records/2026-11-16/usb-camera/README.md)。
- [视频](records/2026-11-16/video/README.md)、[录制回放](records/2026-11-16/recording/README.md)、[Linux ROS2部署入口](groundstation/doc/ros2_deployment.md)、[运行工具安装核对](records/2026-11-16/runtime-install-audit/README.md)。

`platform/`保存机载运行源码及配置，`groundstation/`是独立地面站适配副本，`tools/`是电脑端验证脚本。ROS1参考、旧测试和安装器仍在源码中供对照；不得按历史说明覆盖ROS2入口。Linux运行工具组件只安装当前依赖，历史ROS1自动导航安装器不参与现有Qt调用。

新迁移报告、参数快照和验证日志仅保存在电脑/GitHub。机载保留运行代码、配置、必要运行状态与许可证，不留迁移报告。本仓库各阶段原始记录按发生时点保留，不能把其中的历史状态当作当前部署。

系统/BSP、已有服务停止及正式运行环境切换需用户确认。不自动解锁、起飞或执行飞行任务。

最新[EGO独立C++双QoS诊断](records/2026-11-17/ego-native-qos/README.md)在同一运行中观察到两个原生QoS读者收齐230条，而实际MAVROS读者约1.140秒后才出现在订阅图中，内部出口/PX4仅180条。控制入口的启动就绪条件仍待修正验证，不能以观察器订阅数量或单轮通过作为验收。

[EGO启动就绪修复与失败对照](records/2026-11-17/ego-consumer-ready/README.md)：端点及只读响应均不充分，新增路由出口等待后246/246、231/231逐条通过，候选已部署5b1d6b7。前文历史失败完整保留；实物控制链仍待验证。
