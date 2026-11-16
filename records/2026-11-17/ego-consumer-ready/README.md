# EGO控制发送链启动就绪验证

断电续接只读确认.224在线、Ubuntu22.04.4、NoMachine active、已部署b686e55干净；雷达网口DOWN。

## 失败证据

run1仅等待真实/mavros/setpoint_raw订阅端点：235条发布，两个原生QoS观察器各235条，MAVROS内部出口和PX4均212条，严格检查41/43通过，最初23条遗漏。端点图可见不等同整个发送链可用。

run2在端点之上增加控制插件GetParameters只读回复：255条发布/两种原生观察均255条，内部出口和PX4均231条，严格检查41/43通过，最初24条遗漏。只读参数响应证明能够处理服务回调，不能证明路由出口已建立。两轮严格失败完整保留，未部署。

参考源码mavros_uas.cpp显示：先启动插件执行器，依次创建插件，最后connect_to_router创建mavlink_sink；send_message在sink尚不存在时不会publish。该时序为失败提供解释，但没有注入已安装MAVROS内部函数，因此不将所有历史遗漏唯一归因为sink空指针。参考文件来自前期下载目录，并非本轮重新从已安装二进制反编译。

## 最终候选

在实际控制订阅端点、只读thrust_scaling参数响应之上，等待MAVROS内部mavlink_sink发布端点。首次就绪后锁存，避免图缓存抖动反复中断输出；运行中仍由连接、输入时效、坐标等现有保护停发。默认command_consumer_node=/mavros/setpoint_raw，command_transport_topic=/uas1/mavlink_sink；自定义命名空间须通过标准ROS参数覆盖。不写飞控参数，无解锁/模式服务客户端。

run3实际EGO/FSM/执行器/MAVROS→隔离官方PX4严格43项通过，246源/246wire，PVA/yaw/掩码/源毫秒戳一致，ULog验证PVA解析，始终disarmed且非OFFBOARD。本轮为固定出口话题版本，最终参数化版本另行复测。

隔离目录/tmp/ego-consumer-regression使用候选脚本与真实规划库，47项执行器/管理回归通过。假控制读者显式采用/mavros/setpoint_raw身份、参数服务及空Mavlink发布端点，未启动真实飞控。测试允许这些参数服务，仍禁止飞行服务；此项只检查服务图，不能单独证明从未发生过调用。真实wire测试另行检查执行器客户端仅允许只读GetParameters。

## 范围和复现

ROS_DOMAIN_ID=87且ROS_LOCALHOST_ONLY=1，电脑官方PX4在仅lo独立network namespace，通过SSH stdio连接，传感器/飞手状态合成。记录器先就绪、真实MAVROS延迟启动；新增负向检查确认记录器不会提前放行输出。停流、恢复、STOP_HOLD、交还和退出均保留。不能替代真实雷达/PX4融合或实物任务飞行。

运行EGO_WIRE_TRACE=1 EGO_WIRE_NATIVE=1 python3 tools/run_ego_mavros_wire.py /absolute/new-output-directory，然后使用tools/analyze_waypoint_mavros_wire.py --fixture-subdir ego-mavros-wire及tools/analyze_ego_native_qos.py。后者仅诊断，不放宽严格全流检查。

原始归档、ULog、输入源码、逐条跟踪及日志保留；可再生native-build、PX4 etc/test_data与缓存不重复展开。报告只存电脑/GitHub。正式服务未安装或切换，未操作真实飞控参数或飞行任务。

## 复测、部署与清理

run4为最终参数化版本，43项严格检查全部通过，231条源/两种原生QoS/MAVROS内部出口/PX4逐条一致，ULog58条控制模式样本且未解锁、非OFFBOARD。两轮通过只覆盖本次隔离启动实验，不宣称所有DDS问题或实物飞行已解决。无任意前缀豁免、固定启动sleep或自动解锁。

47项回归及run4归档输入的adapter SHA256均与部署字节一致：745bbc0bb5a67e91ab4df0bf42834aeb0cab9d9fbf83021d411200b97e3e8061。机载候选提交5b1d6b7083a6bc100754f1016f108816063ebb75，原b686e55，仅更新执行器脚本；提交和摘要在deployment.txt。正式服务没有启停或切换。

需恢复此次候选代码时，在确认候选执行器不运行后，仅恢复该文件：git restore --source=b686e55 -- 01_apps/ego_planner/px4_execution/adapter.py，然后单独提交恢复变更；不执行整仓库reset。也可从部署前提交提取此文件。

初始只读服务请求尚未返回期间保持等待；本阶段没有注入服务进程在请求中途崩溃/重启，不能声称这个故障场景的自动恢复已验收。图中出现mavlink_sink不是端到端ACK，后续仍须实物通信验证。
