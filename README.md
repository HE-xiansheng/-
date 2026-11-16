# 科研飞行平台 ROS2 迁移

将Orange Pi 5 Ultra参考机`.235`的完整平台迁入`.224`，使用Ubuntu22.04 ARM64 / ROS2 Humble。**迁移进行中，尚未完成实物整链验收或正式入口切换。**

## 当前状态（2026-11-17）

机载`.224`已从eMMC运行Ubuntu22.04，Humble和NoMachine8.5.3已安装。当前机载候选源码版本为`d896200`（电脑/GitHub另有地面站显示调度修正）、Git干净、NoMachine正常。传感器及云台relay服务候选尚未正式安装/启用；当前网口DOWN、无雷达/云台直连，飞控与Gemini在位。

| 模块 | 已有直接证据 | 仍需验证 |
| --- | --- | --- |
| Gemini335L | 官方wrapper/SDK2.10.6，实际约30Hz、当前设备内部标定及相机→NPU预览；发布端64MiB DDS配置已部署并验证 | 间歇USB断连原因、机体外参、度量精度和完整任务 |
| Livox / FAST-LIO | SDK2 1.3.0、ROS2 driver1.2.6构建；旧机实采回放实际FAST-LIO；自同步秒/ns转换修正与受控对照已部署 | 新机物理雷达采集、网络/时钟及运动精度 |
| 定位转发 / MAVROS2 | 旧二进制71条几何对照；原生100帧初始化；MAVROS2 2.16.0物理未解锁遥测和22项实际MAVLink接口检查 | 实际安装外参、真实PX4外部视觉融合与时间同步 |
| 定位到PX4仿真 | 旧机实采回放→实际FAST-LIO/转发/MAVROS→官方PX4 1.16.0，18项；564条源戳/坐标匹配，EV融合与停流检查 | 回放与软件仿真不替代新雷达、真实飞控和轨迹精度验收 |
| EGO规划 | 原生地图/优化/FSM及执行器；一轮实际规划→MAVROS→PX4消息38项、原执行器46项回归，支持标准ROS重映射；新增启动就绪门控两轮43项与47项回归通过 | 实物地图/任务/地面站全链、运行中端到端可靠性 |
| 航点 / ArUco / 人物 | 原生包、实际RK3588 NPU；航点→MAVROS→PX4消息34项、ArUco图像控制及PX4速度解析，启动等待修正后两轮44项、61项触发/取消回归；人物NPU→MAVROS→PX4两轮37项及53项回归通过 | 物理云台/标记/运动目标、任务地面整链与持续消息可靠性 |
| USB相机 / 云台视频 | usb_cam0.8.1候选；实物云台经只读转发→MPP双QoS全帧交付；断流/停流/分辨率变化恢复两轮26项通过 | 新机云台直连、独立相机未接、物理延迟与图像时刻/外参 |
| Linux地面站 | 独立Qt源码/构建；任务、遥测、视频、定位、规划及录制接口；r3便携包7项重定位检查，默认显示修正下完整主窗口并发24项GUI+43项机载检查通过 | 人工操作/物理整链、实际任务及正式切换 |
| 录制与回放 | rosbag2/SQLite/CDR、分包下载/Qt回放、进程SIGKILL后副本恢复；恢复官方100ms发现轮询后26项原生回归及实际QML停止收尾 | 底层发现线程竞争未从库级修复；真实掉电、实物整链持续运行 |

PX4源码与ULog已证实：其自身同步收敛前会用接收时刻替代源时刻，本次回放约仿真时钟72秒后才保留非零源时龄。不能只凭融合标志或MAVLink源戳正确认定实机时间对齐，也不能把该时间当作实机固定等待时长。

修复前历史诊断：EGO早期复测228/228通过，但新增两轮诊断分别遗漏前40/41条启动消息，多数发出时DDS已有2个匹配订阅；末轮MAVROS内部出口和PX4均205条、实际发布246条。当时问题尚未根治，严格全流验收失败，不能只凭订阅匹配或单轮通过认定控制链可靠。详见[启动诊断](records/2026-11-17/ego-startup-delivery/README.md)。

## 最新跨模块验证

- [风扇与温控只读核对](records/2026-11-17/fan-thermal-audit/README.md)：新旧机温度阈值/风扇档位一致，PWM即时读数不同不等于配置差异；未替换风扇驱动或启用固定转速服务，短暂限频仍未定因。

- [600秒持续并发](records/2026-11-17/platform-soak-600s/README.md)：43项严格通过，最高84.076°C、最低可用内存9.816GiB；policy6短暂频率上限变化保留待查，不宣称长期热稳定性或USB间歇故障解决。

- [完整地面站显示调度与并发](records/2026-11-17/gcs-render-pipe/README.md)：run8显式修正、run10产品默认均24+43项通过；run9 USB断连失败完整保留。r3独立包仍未替换桌面入口。
- [FAST-LIO时间单位](records/2026-11-17/fastlio-sync/README.md)、[相机DDS](records/2026-11-17/camera-dds/README.md)、[多模块并发](records/2026-11-17/platform-soak/README.md)：区分原始失败、源记录并集复核与真正重跑结果。
- [MPP全帧交付](records/2026-11-17/gimbal-dds-delivery/README.md)、[视频断流恢复](records/2026-11-17/gimbal-reconnect/README.md)、[人物启动接收端等待](records/2026-11-17/person-consumer-ready/README.md)。
- [真实PX4固件与EV参数只读核对](records/2026-11-17/px4-physical-readonly/README.md)：实机报告1.16.0 stable、未解锁，EV配置已读回；尚未证明实际融合或外参。
- [传感器网络候选](candidates/network/README.md)：已按新旧机地址/路由准备，并通过新机libnm解析；未安装/激活，接线和网络切换需确认。
- [录制异常退出与副本恢复](records/2026-11-17/recording-crash/README.md)、[rosbag发现轮询修正](records/2026-11-17/gcs-concurrent/README.md)：SIGKILL不等于真实掉电，恢复100ms不等于修复底层库竞争。

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

最新[EGO独立C++双QoS诊断](records/2026-11-17/ego-native-qos/README.md)在同一运行中观察到两个原生QoS读者收齐230条，而实际MAVROS读者约1.140秒后才出现在订阅图中，内部出口/PX4仅180条。该历史诊断说明控制入口的启动就绪条件需要修正，不能以观察器订阅数量或单轮通过作为验收。

[EGO启动就绪修复与失败对照](records/2026-11-17/ego-consumer-ready/README.md)：端点及只读响应均不充分，新增路由出口等待后246/246、231/231逐条通过，候选已部署5b1d6b7。前文历史失败完整保留；实物控制链仍待验证。

最新[EGO启动就绪修正](records/2026-11-17/ego-consumer-ready/README.md)：端点/只读参数探测两版仍失败，增加MAVROS路由出口就绪后246/246、231/231两轮严格43项通过，47项隔离回归通过。最终候选5b1d6b7已部署，正式服务未切换；不能替代物理整链验收。

修复前[ArUco图像链→MAVROS→PX4验证](records/2026-11-17/aruco-mavros-wire/README.md)：25项图像控制/隔离功能检查通过，但严格39/42，最初5条ROS捕获未进入MAVROS内部出口/PX4，其余逐条一致。未部署产品修改，未忽略启动缺口。

最新[ArUco启动就绪修正](records/2026-11-17/aruco-consumer-ready/README.md)：真实控制端点、只读插件参数响应及路由出口就绪后才预发送和允许任务启动。冷启动两轮872/872、871/871，44项严格检查通过，4组CTest和61项原生触发/取消回归通过；候选dae36a8已部署，未正式切换服务。
