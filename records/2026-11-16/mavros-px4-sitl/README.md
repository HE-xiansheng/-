# ROS2/MAVROS ↔ 官方 PX4 SIH 联合地面仿真

## 实际结果

16项检查通过，见 analysis.json。机载真实 MAVROS2 2.16.0 与电脑官方 PX4 v1.16.0（6ea3539157ca358c70a515878b77077af7d4611d）双向连接；60秒观察中收到位姿/落地状态，所有状态均未解锁。1515条 ROS 位姿均产生 MAVLink VISION_POSITION_ESTIMATE，原始微秒时间戳匹配，严格递增，ENU→NED位置误差0。PX4运行时 estimator_aid_src_ev_pos 和 estimator_aid_src_ev_hgt 均 fused=True、innovation_rejected=False。

这是仿真本地位姿经ROS2反馈的接口检查，不是独立雷达定位；不能据此认定真实雷达/PX4融合、定位精度或任务行为通过。视觉协方差为PoseStamped默认值，后续仍需基于实际定位噪声验证。

## 隔离与配置

电脑 PX4 位于独立 user/network namespace，仅lo；机载ROS_DOMAIN_ID=84、ROS_LOCALHOST_ONLY=1，MAVROS fcu_url固定 udp://127.0.0.1:14640@127.0.0.1:14641，仅通过SSH标准输入输出的有长度边界UDP封包与隔离PX4通信。无串口或局域网飞控输出。传输层拒绝SET_MODE、COMMAND_INT和解锁/起飞COMMAND_LONG。只在模拟器控制台设置EKF2_EV_CTRL=3以启用水平与高度视觉融合；未改真实飞控参数/固件。运行中读取三项listener，命令保存在 live-diagnostics-commands.txt；它们写入该隔离模拟器stdin，输出见px4-runtime/px4.log。正常退出后再次读取并关闭模拟器。

测试程序见 tools/run_mavros_px4_sitl.py、sitl_stdio_transport.py、test_mavros_px4_sitl_ros2.py。重新执行需先按脚本路径部署临时文件，使用全新输出目录。当前编排器运行诊断在退出时执行，本次实时融合诊断为额外记录的只读命令；复现融合状态应在视觉发布过程中查询，停流后的结果不能代替运行中融合结果。

源/部署文件SHA256见manifest.json、cleanup.json；比对一致。remote-runtime/包括机载MAVROS日志、ROS检查结果、视觉源数据与SSH传输计数，px4-runtime/包括模拟器日志、MAVLink包、仿真参数和ULog；日志全部已取回电脑。

## 未解决事项

出现PX4一次timesync RTT过高告警，MAVROS也出现启动时约0.8–1.8秒RTT告警；参数请求有一次超时重试。双向连接最终保持，融合成功，但未独立测量稳态端到端延迟，不能据此接受实际链路时延。PX4收到视觉源时间后如何在不同同步状态下生成timestamp_sample，仍需进一步验证。

真实雷达数据、FAST-LIO→原生位姿转发→MAVROS→PX4全链路、实机外参/滤波参数、控制任务和地面站整链验证尚待完成。

## 清理与运行环境

所有所属测试进程退出，/tmp/mavros-px4-sitl已删除；机载Git干净、NoMachine active，仍bd6c781。未停止或切换现有服务，无正式入口切换；测试无自启动，实际运行环境无需回退。所有本阶段迁移证据仅保留在电脑/GitHub。
