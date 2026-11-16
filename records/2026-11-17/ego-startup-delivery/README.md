# EGO启动消息遗漏：发布匹配与MAVROS内部出口诊断

本阶段增加仅用于验证的publisher跟踪封装，以及MAVROS内部/uas1/mavlink_sink只读订阅。执行器产品代码未改；trace通过运行时封装现有Publisher.publish记录实际发出时的源戳、匹配数和匹配变化时的图信息，然后执行原样adapter.py。此额外观测会改变运行调度，不能用新运行反推历史运行的精确内部时序。

## 新证据

第一轮：实际发布228条，源话题观察器228条，PX4入口188条；缺失前40条，约0.915秒后开始到达。第一条发出时仅匹配源观察器，此后订阅数为2；缺失的另外39条发出时已匹配2个订阅者。因此“所有遗漏都发生在DDS尚未匹配之前”的假设被本次数据否定。第二个订阅端点ROS名为UNKNOWN，QoS为BEST_EFFORT/VOLATILE；在本隔离fixture里MAVROS是另一个控制消费者，不把UNKNOWN名字误报为已识别节点。

第二轮增加内部出口观察：实际发布246条，源话题观察246条，MAVROS内部出口205条，PX4入口205条。缺失前41条，约0.924秒后开始到达，其中38条发出时订阅数已为2。内部出口与PX4入口源戳集合一致；没有任何“PX4入口缺失但内部出口已看到”的源戳。当前证据更指向源话题→MAVROS插件处理/内部出口这一段，不能解释为SSH独自丢掉这些包。但内部出口观察器自身也是BEST_EFFORT，不能仅凭未观察到某条就断言插件绝对未处理；尚未定位到具体DDS/执行器机制。

两轮ROS任务行为/隔离检查通过，严格全流分析均失败，analysis.json保持passed=false，不放宽数量或覆盖门槛。run2内部出口观察与wire的集合核对见discovery-analysis.json；实际PVA转换仍对收到的消息逐条正确。没有把末轮通过的旧记录覆盖为新结论。

## 范围和官方源码参考

沿用实际EGO planner/adapter/MAVROS→电脑隔离PX4，合成传感器/飞控输入，域87仅localhost；PX4始终未解锁、非OFFBOARD。源码和已验证源副本摘要见source-sha256.json。ROS内部出口只读观察没有注入消息或更改QoS/控制行为。

已读取上游MAVROS2.16.0的setpoint_raw、router、UAS与executor源码，URL和下载文件摘要在official-references.json。setpoint_raw使用SensorDataQoS，处理本地消息后做坐标转换并发送；router内部ROS端点为BEST_EFFORT/VOLATILE；executor源码有线程/超时配置。未证明部署库与这些下载文件逐字相同，不能仅凭源码中的1000ms超时把本次0.9秒遗漏认定为该超时造成。GitHub Contents API返回403 rate limit，改读raw路径；源码保留电脑工作区，未修改底层MAVROS库。

## 复现与清理

```bash
EGO_WIRE_TRACE=1 python3 tools/run_ego_mavros_wire.py /absolute/new-output-directory
python3 tools/analyze_ego_discovery_trace.py /absolute/new-output-directory
/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python tools/analyze_waypoint_mavros_wire.py /absolute/new-output-directory --fixture-subdir ego-mavros-wire
```

关闭EGO_WIRE_TRACE时保持原验证入口，不加载trace包装或内部订阅。工具沿用本电脑专用IP/SSH指纹/PX4构建路径。最后一条严格分析返回1是本次记录的真实失败，不能以runner退出0代替。

两轮runner记录均证明退出0、日志取回、机载/tmp/ego-mavros-wire清除。取回后再次SSH复核时.224变成No route to host，旧机.235可连接；已知IPv6链路本地地址也超时，mDNS主机名指向旧机的MAC，未把它误当新机继续操作。断联期间无法复核机载状态；后续同一地址恢复后的重新检查结果见本节末尾与reconnected-state.json。无产品代码/正式服务变更，不需要部署回退。

完整ULog、源命令、线包、匹配跟踪、内部出口观察及原始归档仅在电脑/GitHub；可再生缓存和锁定PX4构建目录副本省略。物理雷达/云台网络、真实飞控融合和整平台地面工作流仍待验证。

随后同一.224地址恢复SSH，源码b686e55干净、NoMachine active、临时目录不存在、所属进程为空，见reconnected-state.json。运行时长未重置，未见系统重启；此前断联原因未定，不再需要用户为本次连接恢复执行操作。
