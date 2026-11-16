# 航点迁移过程

- 只读参考 `.235` 当前版本：mission.cpp e419cd6f2a04c3960c11989412ca09261a7b1d6668ae1e67505dd2f8718cc45e；photo.cpp a1a72d203176696cd321b9acd53bf74bc51e126ed4461209196a5caefffc9cfd；abc_photo.yaml 16c3ee04240358e8697cd15013f881ccde8e23b25d621c90abe445e115e57d74。
- 保留RC第5通道第6档、POSCTL基线/新边沿、预发送1.2秒、自动/人工空中接管、拍照/返航/AUTO.LAND策略；任务YAML保留原格式，不作为ROS参数文件解析。
- 原线程队列可能取消后仍发送旧服务请求。ROS2改成单线程executor异步请求，2秒deadline，取消移除pending，generation/token屏蔽晚到结果；已发送请求无法撤回，不宣称可撤销飞控已收到的请求。
- 云台仍为有界TCP工作线程，发送前检查active/generation，已开始的TCP命令不能撤回。物理云台不在本阶段测试范围。
- 首次编译失败：Humble Node不支持resolve_topic_name；改用订阅的get_topic_name。完整日志build.log。
- 图像默认Reliable，/rtsp_cam/image_color；拍照检查源时刻和单调接收时间，回复与触发路径严格关联。默认诊断/照片/tmp，可显式指定产品数据目录。
- 隔离域87、ROS_LOCALHOST_ONLY=1，模拟MAVROS服务。真实飞控无本阶段控制/模式/解锁指令；不切正式运行环境。

测试过程：第一次63/65通过，stale_rc场景未等DDS发现及RC基线就拨档，任务正确保持IDLE，导致两个前置/ABORT断言失败。测试增加明确等待gate_ready；selected-at-start反例不能等待gate_ready，另行保留反例。记录保留原始日志，不把未开始任务当作异常中止已验证。

额外审查：源时间字段先检查负秒和非法纳秒范围，避免rclcpp::Time构造异常使任务节点退出。补零时刻场景。机载后台dnsmasq仍为原有失败状态，本阶段不改网络服务。

后续地面站任务管理器仍使用rospy/ROS master/roslaunch和全局参数；需改rclpy图查询、ROS2 launch、原生包名及节点参数，保留PID/启动ticks/boot身份校验与安全停止逻辑。尚未部署此部分。

第二轮78/79通过：已选择OFFBOARD档的反例被通用fixture错误要求gate_ready，节点正确始终不触发。修正fixture只为普通场景等待基线，反例等待发现即可；另补零/负秒/非法纳秒输入，最后重跑联合检查。补充pending-only测试6/6通过：2秒无响应超时、持续设定点、RC取消、晚到响应和正常退出。

## 本阶段范围与待办

隔离测试通过不代表可以飞行。尚未验证：真实遥控档位标定、PX4实际模式/解锁/失控策略、LIO/飞控世界系物理对齐、SITL、照片高分辨率/磁盘拥塞、真实RTSP时延与重连、云台外参与TCP时延。图像源为解码节点标时，源相机曝光时刻仍待物理测量。任务管理器及Qt地面站还未适配本包启动接口。禁止把本阶段模拟ARM/OFFBOARD测试等同于真实动作测试。

## 复现

仅在隔离ROS域87、ROS_LOCALHOST_ONLY=1，source机载drone_platform/env.sh后运行电脑仓库tools/test_waypoint_ros2.py；附加--pending-only可独立检查服务无响应/取消。脚本在/tmp/waypoint-verification生成结果、临时任务、合成JPEG/CSV；运行结束后先取回电脑再清理。测试启动真实原生二进制，MAVROS是脚本内的伪服务/消息源。

## 回退

本阶段无启动服务或生产切换。无需使用时不启动waypoint_mission.launch.py即可。若需删除此原生移植，停止本阶段自有测试/任务会话后，revert本阶段机载代码提交，移除仅本阶段新增的install/waypoint_mission前缀，再重新生成任务工作空间环境。旧机.235始终只读可作为行为参考，不修改其服务。不得批量pkill其他任务或回退整套已完成模块。

## 最终结果与版本

最终联合93/93、服务超时/取消6/6、launch --show-args通过。机载提交6357385（父版本73fde8e）；电脑仓库保存同版源码、配置、测试脚本、结果JSON、构建日志及三轮现场日志归档。复测后的负秒和非法纳秒输入会中止任务并正常退出节点，不抛出时间构造异常。

回退本阶段：本包尚未加入正式启动服务。停止自己启动的测试/任务会话后，可在机载代码仓库revert 6357385，并移除仅本次新增的workspace/install/waypoint_mission安装前缀，重新生成工作空间环境。其他已迁移模块保留父版本；不需要重装系统、回退BSP或修改PX4参数。进行真实任务中的进程切换需要另行确认影响。

提交前diff检查发现继承云台C辅助文件有行末空白，已仅清理空白，功能不变；机载提交更新为6357385。最终Git diff检查通过。
