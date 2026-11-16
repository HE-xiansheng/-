# ROS2飞前遥测与跨机Qt接收阶段

本阶段基线：电脑/GitHub `e9d6fad`，机载平台仍为 `5859393`。机载原有服务未停止或替换；遥测源码经SSH stdin运行，不创建新开机服务。原正式地面站未切换，旧机`.235`未修改。

## 已实现

- ROS1 AnyMsg字节偏移、XMLRPC Master查询替换为ROS2类型化消息和DDS图。沿用原schema1/PCD1/POS1/TLM1/TLG1/PFS1线格式，只传传感器小字段，不把雷达点数组放入状态JSON。
- 原生MID360 CustomMsg、IMU、压力、里程计/位姿、MAVROS连接/着地/估计/RC/电池/提示均按源时刻与单调接收时刻观测。频率取消息回调样本，不把5Hz监视频率当作源频率。无效电池数值转null，无效位姿明确显示无效，不能令整份JSON失效。
- 发布者使用DDS图全名；同名多个实例不合并，定位会话由boot/domain/唯一且符合预期的发布端GID标识。运行提示不再读取旧ROS1 active文件；图上存在节点不等于数据正常。
- 规划订阅按原生ROS2保留型QoS；正确使用camera_init和`/grid_map/occupancy_inflate`。栅格分辨率经`/ego_planner_node/get_parameters`异步只读获取，有界超时并移除待处理请求；不调用参数写入、模式、解锁或任务服务。栅格显示匹配会话/坐标/源时间，接收新快照不能刷新旧栅格寿命。
- Linux Qt连接入口按安装目录或当前源码目录定位本机工具，不依赖旧dianyunGCS-master相对目录。状态位置独立于原`.pointcloud-link`，默认`~/.local/share/drone-gcs-ros2`，可用GCS_STATE_DIR覆盖；状态分片默认UDP9015，避免原地面站已占用的9005。Python与Qt状态位置一致；Windows前台桥同样更新默认分片端口，但Windows实机未验证。
- 录制后端尚未迁移时明确显示UNAVAILABLE，不复用旧ROS1状态假装已可用。

## 验证与证据

- 电脑本地13项unittest通过，另执行继承的21项显示断言及新增camera_init/跨会话/源时刻检查。见local-tests-final.log。
- 机载真实ROS2消息和图联合20项通过，最终相同源码用orangepi运行，见joint-final.log、joint-final-result。包括自定义消息、频率、非有限字段、发布者冲突、MAVROS反馈与指令发布者区分、断源过期、规划/栅格、会话切换和TLM1重组。只有隔离测试才建立模拟控制话题端点，未向其发送任何指令消息。
- Qt ConnectionController认证→本机manage_link→SSH stdin采样→跨机UDP→原PreflightReceiver实际通过，见ssh-qt-user及ssh-qt-final。合成数据在机载DDS域198/localhost运行，经电脑192.168.1.89接收；电脑没有ROS2环境。Qt验证了定位状态、MID360点数、camera_init轨迹、原生栅格与停止桥接后的过期状态。测试只停止自己独立目录中的桥接进程。
- 独立customgeometry与radarsimulator构建完成，见app-configure.log、app-build.log。只改了后端和本机工具定位，未修改航线拖拽上传规则或UDP坐标变换。
- 机载最终源码与电脑摘要见target-final-versions.txt/local-final-versions.txt；首次检查发现测试副本少了最后的预期来源名约束，已重新同步最终源码并复测，未把早期摘要当最终验证依据。

## 已记录问题与边界

- Humble没有rclpy.parameter_client，不能直接采用新版API；实际改用rcl_interfaces/GetParameters客户端。远端inspect首次多层引号错误，改为stdin脚本读取。Humble Context有get_domain_id而Node无该方法，已按实际API实现。
- 首轮跨机测试由root发布合成数据、orangepi采样，监视SSH/UDP可通但源数据缺失、DDS图出现UNKNOWN。同用户orangepi运行后链路通过；现象与FastDDS共享内存跨用户限制一致，尚未做独立因果定位。未为通过测试更改系统DDS配置。失败日志保留ssh-qt-first。
- 此阶段是合成ROS2数据的遥测联合验证，不是物理雷达、IMU、PX4融合、真实相机/云台或飞行任务验收。定位/规划/录制Qt操作入口和视频采样仍有ROS1依赖，整套ROS2地面站尚未完成。原生遥测仅负责显示，不提供解锁许可。

## 重现及撤销

机载先source平台env.sh；测试仅使用ROS_DOMAIN_ID=198和ROS_LOCALHOST_ONLY=1。tools/test_telemetry_ros2.py使用--reader指定本阶段采样器、--output指定临时结果目录。Qt测试在groundstation/tools/preflight/test_qt独立构建，跨机编排脚本tools/test_telemetry_ssh_qt.py只允许认证和只读采样远程命令；必须先把测试代码放入其明确的机载/tmp目录。

停止独立目录自己的桥接即可结束本阶段，正式应用未切换。用同一个GCS_STATE_DIR执行tools/pointcloud-only/manage.py stop及tools/preflight/bridge.py stop；不要删除或停止原`.pointcloud-link`拥有的会话。机载所有测试目录和本次拥有的点云目标锁在取回结果后清理，见cleanup-final.log；报告继续只存电脑和GitHub。

清理补充：本机SSH客户端结束后，远端合成fixture曾继续运行到其75秒期限，首次清理断言据此拒绝删除。确认它自行退出后完成清理；编排工具补充只匹配本次明确/tmp测试命令和--serve参数的远端停止步骤。此补充只影响测试fixture清理，不改变已通过复测的生产遥测代码。
