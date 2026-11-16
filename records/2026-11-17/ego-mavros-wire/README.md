# EGO实际规划/执行/MAVROS/PX4消息链路与ROS2参数修复

本阶段修复EGO执行器拒绝标准ROS2命令行重映射的问题。修复前，`adapter.py --publish/--check --run ... --ros-args -r ...`在argparse阶段退出2；修复后先用rclpy.utilities.remove_ros_args拆分工具参数，rclpy.init继续读取原始ROS参数。原有工具选项、域87隔离检查和控制状态机保持原有语义。修复部署到.224候选工程b686e55，没有启用正式服务。

## 结果与失败证据

最终run2/analysis.json有38项严格检查全部通过，228条ROS控制消息对应228条PX4入口MAVLink消息；源毫秒戳、P/V/A、yaw/yaw_rate、目标ID和消息掩码逐条匹配。规划来自实际plan_manage/planner_node，执行来自本次候选adapter.py与现有execution.py，通信使用真实MAVROS2和官方PX4 v1.16.0。实际非零速度和加速度均已出现；ULog有59条offboard_control_mode记录，位置、速度、加速度使能均出现。真实PX4状态与ULog均未解锁、未进入OFFBOARD。

run1的ROS行为检查通过，但全流严格分析失败：228条源消息只收到219条wire消息。缺失的是最初0到0.181秒的9条静止保持消息，之后均逐条匹配。首轮分析器同时重复计入一次相同的加速度检查，run2已去重，不能按39→38解读为删去失败要求；两项全流数量/覆盖门槛保持不变。第二轮全部228条到达，不证明首轮问题已根治。端点发现/启动调度可能有关，当前没有证据确定根因，也不保证其他运行无漏收。

两轮原始分析均保留，runner-result.json的passed只表示进程及取回清理正常，严格消息验收以analysis.json为准。曾在runner尚未完成取回时读取分析输入，产生FileNotFoundError；确认同一活动句柄终止后再分析，无因此重启进程。

## 输入、任务行为与隔离范围

雷达里程计、点云和飞控位姿/状态是合成输入。EGO使用实际地图/优化/FSM及实际PolyTraj，adapter按90度相对航向将参考对齐到FC ENU；MAVROS再执行ENU→NED。任务输出经过SSH标准输入输出封包送入电脑仅lo的独立network namespace内官方PX4。机载ROS_DOMAIN_ID=87和ROS_LOCALHOST_ONLY=1，MAVROS只连接127.0.0.1 UDP，无串口和物理飞控控制。

标准ROS重映射仅把adapter的三个飞控输入接到/fixture；真正MAVROS状态单独监视。adapter本身没有模式/解锁/参数服务客户端，传输器还拒绝模式、解锁和起飞命令。合成飞控模式允许轨迹采样，不意味着PX4真的切换模式或执行轨迹。

覆盖：就绪时实测位姿保持；地图目标直接生成实际轨迹；完整PVA输出；里程计停流、点云停流、SIGSTOP暂停规划器后停止输出及恢复；操作员stop请求进入STOP_HOLD并保持实测位置/零速度与加速度；合成POSCTL交还后节点退出且无输出。物理闭环运动、定位漂移、障碍物实测精度与完整任务未验证。

## 既有行为回归及部署

对已部署修复运行原有validate_ego_execution.py，46项全部通过，包含不带ROS重映射的既有manager/启停/轨迹/输入保护/单实例行为。测试副本只把ROS日志路径改到/tmp/ego-remap-regression，原工具及本次副本都保留。原生代码与最终测试副本摘要核对见source-sha256.json；部署前后摘要、机载b686e55干净状态、NoMachine active和所属进程检查见deployment/与cleanup.json。两处机载临时目录均已取回清除，无本次任务/规划/MAVROS/传输进程。

回退只涉及候选源码：在独立检出中取修复前c58b4d5的01_apps/ego_planner/px4_execution/adapter.py；无系统、底层驱动、正式服务或桌面入口变更。电脑仓库修复前版本23d87ca也保留同一旧文件，复现器固定取该版本作失败对照，不依赖机载是否已修好。

## 复现与限制

```bash
python3 tools/run_ego_mavros_wire.py /absolute/new-output-directory
/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python tools/analyze_waypoint_mavros_wire.py /absolute/new-output-directory --fixture-subdir ego-mavros-wire
```

使用当前电脑专用SSH指纹、板卡IP和PX4私有构建/venv，路径应先核对。若runner执行失败应检查所属进程及现场，不能覆盖旧临时目录重跑。原始ULog、wire包、源命令、参数、ROS日志和输入归档均保留；Python缓存、来自已锁定构建的etc/test_data副本未纳入，完整运行目录仍在电脑工作区。

MAVROS仍有启动期版本请求超时/参数重试、RTT及EVENT告警；fixture结束后MAVROS停止，而PX4继续至75秒，因此出现mission computer lost。本次共享传输器只在临时模拟器设置EKF2_EV_CTRL=3并在结束查询EV listener，本阶段没有视觉输入，EV未发布不能解释为定位融合失败或成功。没有改变真实PX4参数。

time_boot_ms只是ROS源时间的毫秒低32位转换，本次不测量时间同步精度、曝光时刻或端到端延迟。整平台仍待真实雷达接入、云台直连、真实PX4融合、外参和物理地面站工作流验收。迁移记录仅在电脑/GitHub。
