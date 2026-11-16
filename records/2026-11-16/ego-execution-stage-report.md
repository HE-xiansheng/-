# EGO 执行适配器及会话管理 ROS2 阶段

2026-11-16，目标 .224，机载代码版本 `73b952659fb0914ed5d77434f0470bf6efa3a8ab`。

## 实现与接口

对照 .235 当前 adapter.py / execution.py / manage.py 校验源码哈希，见 platform-sources.lock.json。纯轨迹运算与遥控器交接 Session 类原样复用，ROS 通信改为 rclpy；没有解锁、切模式、飞控参数写入或固件操作。

保留 `run_board.sh --start|--stop|--status`、`run_px4_execution.sh enable|stop|status|check` 和地图直接目标更新。状态机仍负责重规划及紧急停止轨迹。执行器保持上游时钟与 PVA，不增加轨迹时间缩放，不自行裁剪规划动态约束；冻结旋转并用最新测量位姿调整平移，与当前旧机算法策略一致。

`/ego_planner/flight_execution` Bool 使用可靠、transient_local QoS；执行轨迹 PolyTraj 使用可靠、volatile QoS；传感器与飞控状态订阅使用 SensorData QoS。状态反馈仍为 std_msgs/String JSON，保留控制状态和位姿诊断字段。进程身份同时检查 PID、启动 tick、命令行；单实例文件锁防止重复发布；停止使用受控请求，OFFBOARD 时保持测量位置，遥控器接管后退出。规划停止命令在执行器仍输出时拒绝终止规划器。

会话状态、控制请求与 ROS 日志默认位于 /tmp，不在源码目录写迁移报告或样本日志；验证输出仅重定向到电脑。录制管理模块尚未移植，不能把当前会话管理完成等同于手动／自动录制链路完成。

## 本阶段发现及修正

实际 ROS2 FAST-LIO launch 将全局点云重映射到 `/factory_fastlio/cloud_world`。此前预览 launch 默认 `/factory_fastlio/cloud_registered` 与其不一致，本阶段已修正预览与执行器默认输入。FAST-LIO 补丁输出 child_frame=body 的 twist，故启用 `fsm.odom_vel_is_body=true`；新增 90 度姿态下起始轨迹导数检查证明速度被转到世界坐标系。

旧执行器虽然声明超时但没有实际使用。本次对必要输入检查源时间与单调接收时间，并在状态／定位／点云／飞控位姿过期、未来位姿、无效姿态、坐标系不一致或断连时停止输出。恢复输入后重新请求原生轨迹，避免使用上一控制周期缓存的轨迹。

MAVROS2 2.16.0 官方 setpoint_raw.cpp 的 local_cb 明确执行 ENU→NED，链接及源码SHA256已登记；PositionTarget 的 FRAME_LOCAL_NED 标志不代表调用方应先把 ROS 数值换成 NED。适配器输出飞控本地 ENU PVA，换轴仅由 MAVROS 执行。

## 验证证据

- `records/ego-execution-validation-published.log`：46 项通过。原生 FSM + 执行器 + 模拟飞控完整进程，观测约44.08Hz设定值；接收148条模拟命令、9条原生执行轨迹。覆盖未解锁／落地无输出、POSCTL测量位置预流、两次OFFBOARD遥控交接、PVA旋转和原时钟、断流／断连／坐标错配／无效姿态／未来时间戳、恢复、受控停止、单实例、只读检查及正常退出。
- `records/ego-lifecycle-validation-final.log`：19 项通过。通过实际启动脚本启动原生 launch、读取状态、重复启动复用会话、拒绝已解锁或断连时启动、输出中拒绝停止规划器、执行器受控停止、停止／重启会话，不影响旁边的测试进程。
- `records/ego-fsm-velocity-validation.log`：32 项通过，包含此前31项及body→world速度检查。
- Python语法、shell语法及git空白检查通过。

所有测试使用 ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1；没有物理 MAVROS 节点或飞控服务参与。模拟 armed/airborne/OFFBOARD 状态只存在于隔离域，未向真实飞控发送设定值或执行飞行任务。第一轮无效姿态测试失败源于测试消息默认四元数仍有效；显式置零修正夹具后通过，失败原始日志保留供追溯。

## 限制与回退

静态模拟输入不能证明实飞控制性能、实际位姿对齐、飞控融合或丢包时PX4回退行为。最新位姿平移策略保留旧实现，但未证明真实运动中不同采样时刻的误差可接受，需物理地面／SITL验证。没有切换正式运行环境，也未改旧机服务、雷达网卡或实际飞控。

若撤销本阶段，在 .224 工程对73b9526执行git revert，恢复d2e419e阶段的源码／配置；本阶段为脚本与参数，无需重新编译原生库。撤销不代表恢复了可实飞环境，正式切换仍需按全链路验证和用户确认执行。源码参考、此前版本以及运行库保留。

下一阶段仍需完成位姿转发语义与融合、人物推理／任务控制、地面站ROS2后端、手动／自动录制，以及物理外设接入测试。
