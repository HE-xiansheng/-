# EGO启动遗漏：独立C++双QoS对照

新增仅用于诊断的rclcpp观察器，在同一进程/执行器中同时订阅RELIABLE(depth5)和SensorDataQoS(BEST_EFFORT, depth5)，并只读订阅MAVROS内部mavlink_sink。均为VOLATILE。运行RMW为rmw_fastrtps_cpp，使用机载实际Humble/mavros_msgs2.16.0构建。产品代码和底层库未改。

## 结果

第一轮40项严格检查通过：实际发布252条，C++两种QoS、内部出口及PX4均252条，未复现遗漏。

第二轮严格检查38/40通过，数量/源覆盖两项失败：实际发布230条，独立C++的RELIABLE和BEST_EFFORT都230条，MAVROS内部出口与PX4均180条，缺失前50条。在这些消息发出时发布端匹配数为3，发现的是两个独立C++读者及一个RELIABLE读者；后者后来识别为fixture。约1.140秒后匹配数升至4，图中出现/mavros命名空间的setpoint_raw、BEST_EFFORT订阅者，从此其余消息逐条匹配。详见run2/native-qos-analysis.json中的match_transitions和missing_source_indices，以及publisher-trace.jsonl。

本次独立C++的相同SensorDataQoS能收齐，不能把遗漏归结为“所有BEST_EFFORT订阅都会丢同样的数据”。端点数量为正也不能证明真正MAVROS读者已存在。本次数据支持启动顺序/端点就绪因素，但不证明历史各轮遗漏都是同一原因，也不排除特定MAVROS端点的发现、执行器或RMW调度问题。额外观察器改变了启动时间与负载，第一轮通过不构成问题已解决。

## 任务及隔离

仍是实际EGO planner/FSM/adapter/MAVROS到电脑独立network namespace内官方PX4，传感器/控制输入合成。域87仅localhost，无串口或物理飞控任务。PX4和ULog始终未解锁、非OFFBOARD。里程计/点云/规划器停流、恢复、STOP_HOLD和退出检查通过，但严格全流验收第二轮失败，原结果完整保留，不放宽门槛。

## 复现、版本与清理

```bash
EGO_WIRE_TRACE=1 EGO_WIRE_NATIVE=1 python3 tools/run_ego_mavros_wire.py /absolute/new-output-directory
python3 tools/analyze_ego_native_qos.py /absolute/new-output-directory
/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python tools/analyze_waypoint_mavros_wire.py /absolute/new-output-directory --fixture-subdir ego-mavros-wire
```

工具使用当前电脑专用SSH指纹/IP/构建路径。native选项会在机载临时目录自动构建诊断程序；不安装系统包。严格结果写analysis.json，第二轮返回1为真实验收失败；native-qos-analysis.json是描述性诊断，不将丢失当作通过。

两轮取回源与当前工具源码逐一匹配，二进制摘要和构建日志保留；CMake临时构建目录、锁定PX4的etc/test_data副本和可再生缓存未展开纳入Git，原始归档和电脑运行目录仍保留。ULog、源/线包、native每条消息时间戳均保留。

两轮机载目录已取回清理，cleanup.json再次确认b686e55干净、NoMachine正常且无本次observer/planner/MAVROS/fixture进程。没有正式服务或运行环境切换，不涉及部署回退。

下一步应验证实际MAVROS读者就绪条件及持续预发送状态，不能用任意订阅者数量或固定sleep替代；也不应因单轮通过就宣布控制链验收完成。实物雷达、云台直连和真实飞控融合仍待验收。
