# ROS2地面站任务管理入口

源机只读参考：.235 gcs_task_manager.py SHA256 0612ca8fec14dfa534fafab606f8eebc35dc22d430ce7f71853e6a365662702f，电脑参考一致。电脑Qt附带副本另有ArUco源时刻检查/诊断参数，未把其当成源机部署版。

保持JSON CLI、自动地图航点version4坐标变换、任务快照原生校验、RC触发流程、自己拥有的启动会话、PID/ticks/boot/进程组身份、遥控接管后稳定3秒才停止的流程。无模式/解锁/设定点客户端；通过原生ROS2任务节点执行任务。生成Python launch替代ROS1 XML；图状态替代ROS Master，Tracking读取原生带时刻消息并保留旧地面站字段。

地图会话使用boot/domain/DDS发布端GID；必须只有一个LIO发布者，位姿必须与逐消息身份标记一致。ROS2没有ROS1 PID查询；已安装Humble的rclpy消息回调只接收msg，底层MessageInfo也仅有收发时刻，缺少GID。首轮回调参数失败、次轮尝试保留MessageInfo仍无法取得GID，原始日志保留。最终增加只读C++ platform_gcs_support/lio_identity，使用rclcpp::MessageInfo取得每帧真实GID；Python不补丁系统库、不依赖私有executor实现。

ArUco metrics新增整型pose_stamp_ns，与Pose源时刻准确配对，保留原输出字段；不使用ROS2不存在的Header.seq。标记源超时/未来/错误坐标系/指标错帧均不得显示为有效；旧latched降落CSV按ros_time判断。更新合成测试时初次文本匹配失败，按当前字典格式修正，无部署运行影响。

临时会话/任务快照/运行日志使用/tmp/gcs-flight-tasks-UID或测试临时目录；迁移文档及证据只在电脑/GitHub。当前未接入Qt正式入口、未切生产，实际云台、任务控制和物理定位不在本阶段验证范围。

## 实测问题与修复

- 首次执行失败：实机Humble rclpy executor仅传消息，不支持MessageInfo参数。尝试读取底层take_message的元数据后，实测仅有源/接收时刻，没有publisher_gid。移除自定义executor，改为标准rclpy executor加platform_gcs_support/lio_identity原生rclcpp辅助节点；不修改系统rclpy。辅助节点只读Odometry，按每帧源时刻提供真实RMW publisher GID，私有输出话题由管理器生成。
- 原生helper第一轮37/39通过：UUID可能以数字开头，stop-worker创建随机话题时失败。统一使用session_前缀，随后复测。该轮使用原例任务含photo，按原管理器逻辑启动RTSP读流，未输出飞控指令或云台控制；最终测试改用关闭photo的临时任务副本，避免依赖外部相机网络。原始日志归档保留。
- Python JSON状态转换增加非对象/非有限数值拒绝；无效状态不伪装成新鲜有效消息。

最终任务入口联合39/39通过，包括真实原生waypoint进程、快照验证、重复启动、冲突拒绝、OFFBOARD等待/稳定接管后停止。追加辅助进程的父进程死亡清理：Linux PR_SET_PDEATHSIG，GUI/SSH监视进程意外退出也不遗留只读helper。此路径另加真实进程终止测试。

真实图像→ArUco→任务状态首轮7/10通过。现场证据显示Python测试夹具在写入2MB Image.data之前打时间戳，构造消息约350ms，检测再约80ms，超过原有400ms有效期；管理器正确拒绝。保留400ms门限，测试改在合成帧payload准备好时打时刻后立即发送，旧时刻/未来时刻反例仍保留。debug记录aruco-pair-first.json，不把此问题通过放宽门限掩盖。

最终identity辅助消息同时携带同一帧的位姿/源时刻/GID，避免跨发布者同时间戳引起歧义；仍核对ROS图唯一发布者GID、camera_init/map帧、四元数、有限数值及双侧时差<100ms。不修改系统rclpy、不依赖私有Pythonexecutor接口。

图像→ArUco→管理器联合初轮7/10：detector识别深度正常，但fixture在给2MiB图像字段赋值前标时，Python序列赋值耗时约0.35秒，叠加识别耗时后超过0.4秒，管理器正确拒绝。记录aruco-pair-first.json保留源/接收时刻。修正为合成图像数据准备完成后、publish前标记该测试源时刻；不放宽0.4秒有效性阈值。该测试不测真实相机曝光时间/编码/RTSP延迟。

来源标识最终消息在同一个原生回调内携带GID+完整位姿/源时刻，不跨两个topic猜测身份；任务管理器用该消息与图中唯一发布端GID匹配，再与飞控姿态对齐。额外增加地图选点实际生成任务/校验快照/启动及停止链路检查。

## 最终结果、版本与回退

机载代码5859393，父版本6357385。任务入口49/49、真实合成图→原生检测→任务状态10/10通过；ArUco两类标记/空白/错误分辨率四组均通过，每组12个指标/位姿源时刻配对。独立CLI status返回可解析JSON；没有飞控消息时fc=null，未冒称已锁桨。运行测试未调用真实飞控控制服务，未切正式环境。

Qt现有源码路径：/home/tyzk/工作项目/地面站与平台/dianyunGCS-review-20261026/dianyunGCS-master。其missioncontroller.cpp仍使用Noetic远程环境，尚未改为env.sh；GUI/视频/规划/录制迁移未完成。迁移完成不以本后端测试作为整平台验收。

复现：source机载drone_platform/env.sh后，在ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1的隔离环境运行电脑仓库tools/test_task_manager_ros2.py和tools/test_aruco_manager_chain_ros2.py。两脚本分别运行，使用/tmp临时目录，归档取回后清理。validate_aruco_synthetic.py自行使用域176。

回退：本阶段未修改自启动服务。结束自己启动的会话后revert 5859393，移除新增install/platform_gcs_support前缀；如需去除新增指标字段，按父版本重建aruco_test。旧Qt入口未切换，源机.235未改动。使用中的真实任务不可直接通过回退代码中断，应先说明影响并经用户确认。
