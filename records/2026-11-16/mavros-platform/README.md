# 完整MAVROS2接口候选与实际MAVLink验证

现有mavros_telemetry.launch.py仅开放遥测插件，无法接收视觉定位或任务设定值。新增独立mavros_platform.launch.py，使用已安装MAVROS2 2.16.0官方PX4配置与完整PX4插件列表，保持与.235原stock PX4启动方式相近的接口覆盖；默认串口使用已核对的Matek by-id路径，支持显式fcu_url/gcs_url/目标ID参数。

候选已部署机载c2f8226，但没有修改start_sensors.sh、后台服务或正式地面站入口。切换到此候选会开放控制/参数等完整API，需要在后续正式环境切换前由用户确认；本阶段不执行切换。候选不自动启动任务控制器，不写飞控参数。

## 实际验证范围

orangepi用户、DDS域85、ROS_LOCALHOST_ONLY=1。实际ros2 launch运行原生MAVROS，FCU严格指定udp://127.0.0.1:14640@127.0.0.1:14641，gcs_url为空；对端是pymavlink 2.4.49模拟PX4，不打开任何串口，不连接真实飞控。

22项检查全部通过，MAVROS正常退出：

- MAVLink心跳→ROS连接/未解锁MANUAL状态，本地NED→ENU位姿，以及落地和18路RC输入。
- ROS外部位姿→实际VISION_POSITION_ESTIMATE数据包，核对ENU/NED位置、FLU/FRD姿态及原始源时间戳。
- ROS PositionTarget→实际SET_POSITION_TARGET_LOCAL_NED数据包，核对坐标帧、type_mask、目标ID、P/V/A及yaw/yaw_rate只转换一次。
- 实际set_mode服务→模拟对端SET_MODE/POSCTL编码。仅检查arming接口存在，不调用；抓包确认没有解锁或起飞指令。

测试及部署源码摘要一致，详见deployment.json；stock MAVROS配置摘要单独保存。测试依赖只临时解压到/tmp，不安装pip、不更改机载Python环境；取回日志后已清理临时目录，机载Git干净，NoMachine active。

## 首轮问题

首次fixture传入空的gcs_url:=被ROS2 launch解析拒绝，移除这个冗余参数，继续使用候选默认空值。其后测试只等connected就断言MANUAL，遇到首次PX4识别前CMODE(65536)瞬态。改为等待实际状态稳定到预期MANUAL，保持原断言条件；最终状态快照保留了该瞬态。两轮失败日志与结果均保留。

本结果证明当前MAVROS实际传输和转换契约，不能证明真实PX4外部视觉融合、估计器配置正确、完整任务或飞行性能；SITL尚未验收，模拟MAVLink对端不等同PX4固件。物理雷达/云台未接入，本阶段没有相关物理数据。

回退：尚未切换入口，因此现有遥测运行不受影响；若需移除此独立候选，恢复机载c2f8226父提交对应文件状态即可。正式切换的停止/重启与回退将在切换前说明并确认，本阶段不执行。
