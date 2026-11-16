# 航点原生控制器 → MAVROS → PX4 消息联合验证

2026-11-17（本地时间）完成34项检查，analysis.json全部通过；ROS侧21项，包括3个所属节点正常退出，wire/ULog分析另13项。198条原生航点输出均在电脑隔离PX4入口收到，毫秒源戳逐条匹配，数量相等，转换后的坐标、掩码和目标系统/组件匹配。官方PX4 v1.16.0 ULog的offboard_control_mode有65个记录，证实接收并解析了位置和速度输入。所有实际PX4状态与ULog均未解锁、未进入OFFBOARD。

## 真实路径与模拟部分

.224已部署waypoint_mission原生二进制 → 实际MAVROS2 setpoint_raw/local插件 → SSH stdio封包 → 电脑独立user/network namespace内官方PX4 SIH。电脑namespace只有lo，机载ROS_DOMAIN_ID=78/ROS_LOCALHOST_ONLY=1，MAVROS仅连127.0.0.1 UDP，无串口/物理飞控连接。

任务节点的RC、armed状态、位姿、里程计均重映射到/fixture，属于人工合成数据。任务采用manual_airborne=True；OFFBOARD请求只由fixture服务返回，解锁请求如出现会被fixture拒绝。真正MAVROS的状态话题独立观察。传输防线沿用sitl_stdio_transport，拒绝SET_MODE、COMMAND_INT和解锁/起飞COMMAND_LONG；本次均未触发该防线。没有向真实飞控发送任务、模式或解锁命令。

两种场景都从POSCTL/遥控低档就绪基线开始，再合成遥控档位边沿。x_right_y_forward配置的右移0.8米产生负ENU y速度；持续速度受现有限幅约束。随后分别撤销遥控档位、令位姿源戳过期3秒，原生节点进入ABORT并停止发布。wire与源数量完全相等，未发现取消后额外输出。实际任务中产生的保持位置、速度、加速度及yaw/yaw_rate数值按ENU→NED核对；本次加速度/yaw_rate主要为零，不代表所有非零组合已覆盖。

## 时间与PX4解释

SET_POSITION_TARGET_LOCAL_NED.time_boot_ms与ROS源时间毫秒低32位匹配。这是消息转换验证，不是PX4时间同步精度或延迟验收。PX4源码只在真正OFFBOARD模式发布trajectory_setpoint，本次刻意未切模式，因此用offboard_control_mode证实解析，没有把它当作轨迹执行或闭环飞行通过。

仍出现启动时timesync高RTT（PX4约1852ms）、MAVROS参数请求重试和未解码的PX4 EVENT告警。fixture停止MAVROS后模拟器继续观察至75秒，出现mission computer lost；这是测试安排的断开，不消除实际通信掉线风险。共享传输器还向本次临时模拟器设置EKF2_EV_CTRL=3并读取EV listener；本次没有视觉位姿输入，listener never published不作为融合验收。真实PX4参数未改。

## 复现、版本与证据

工具为tools/run_waypoint_mavros_wire.py、test_waypoint_mavros_wire_ros2.py及analyze_waypoint_mavros_wire.py。runner使用当前电脑专用SSH指纹/Qt之外的PX4私有venv和构建路径，运行时需要核对这些路径：

```bash
python3 tools/run_waypoint_mavros_wire.py /absolute/new-output-directory
/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python tools/analyze_waypoint_mavros_wire.py /absolute/new-output-directory
```

runner部署到新的/tmp/waypoint-mavros-wire，测试成功后取回并清理。若失败会保留现场，应先读取所属进程/结果再处理，不能覆盖现场重跑。source-sha256.json与机载取回源逐一匹配；cleanup.json记录当前二进制/source摘要、c58b4d5干净和NoMachine active。完整源消息、wire包、ULog、MAVROS/任务日志和原始归档均保留；可再生Python缓存及来自已锁定PX4构建目录的etc/test_data副本未纳入；原始运行目录仍在电脑工作区。

本阶段只增加验证工具，没有产品代码或正式服务变更。机载临时目录已清理，不存在本次所属任务/MAVROS/传输进程，无部署回退操作。整平台仍未完成：真实雷达数据/定位负载、云台直连、真实PX4融合、标定与物理任务/地面站工作流还待验收；此验证不能替代它们。
