# ROS2定位转发与管理入口

`.235` 保持只读，当前实际部署链路为 `/factory_laserMapping` → `/factory_fastlio/odometry` → `factory_vision2mavros` → `/mavros/vision_pose/pose`。旧路径 `/home/orangepi/lidar_tools/start_lidar_localization.sh` 已不存在。电脑历史快照保留了2026-10-30源时间戳补丁源码，原二进制字符串/反汇编与100帧初始化相符；为避免仅凭历史源码认定等价，直接只读复制 `.235` 原二进制及所需依赖，在 `.224` 临时目录和电脑独立ROS1 master中取得黑盒参考。

原部署二进制SHA256：04390ccaab975f2d3099994a9e919caad6c7f35a5cb97e4971605693017024a6。临时ROS1运行库没有安装到系统，没有连接生产ROS图；测试后已清理。原`window_size=50`启动参数并非滑窗算法，实际初始化为100个有效新测量。30Hz为发布上限，10Hz输入不会重发为30Hz。

原生`to_mavros/vision2mavros`保留100帧位置/RPY算术平均、安装平移补偿、位置分量/RPY分量减零点及map输出；保留源时间戳和严格递增、0.2秒源龄期、0.02秒未来容差、camera_init/body、有限位姿和四元数范数检查。源戳回退必须地面重启，不自动重置原点。输入Best Effort depth1兼容已有定位发布QoS，输出Reliable10，ROS2到MAVROS保持ENU输入，不额外NED转换。

验证证据：

- 原二进制71条参考包含非零安装偏移、非零RPY、yaw分界和大pitch；ROS2逐条几何误差低于1e-9，实际最大值见`ros2-result.json`。该检查还覆盖100帧初始化、重复源戳不推进初始化、源时刻保留、10Hz唯一输出、停流不重发、非法输入拒绝、恢复和正常停止。
- `stack-result.json`为实际FAST-LIO读取旧机雷达/IMU回放、原生转发及生命周期联合验证：21项通过，274帧里程计、114帧转发。隔离域196/localhost，伪飞控，无真实飞控控制输出。
- Qt实际LocalizationPanel及打包SSH helper检查、24项离线编排/前台归属检查、主程序构建通过。
- 原始失败记录保留：第一次DDS发现后立即发送未满足100帧初始化断言；增加发现稳定窗口后参考比较通过。第二次fixture给ROS2浮点字段赋整数触发类型断言，修正fixture后通过。原源码未为通过这些fixture问题改变算法。

原生管理器路径 `~/drone_platform/01_apps/fast_lio/manage_stack.py`，start/stop/status输出JSON。使用所属PID/start tick/boot/进程组/安装可执行程序及DDS来源GID核对，不接管其他来源。与ROS2预览/执行器共享实际管理锁；已解锁、OFFBOARD、状态过期、控制发布者或规划占用时拒绝改变定位。先停止所属转发，再停止所属定位；不停止传感器驱动、MAVROS或录制。不对生产服务做切换。

双终端入口为`03_tools/config script/start_lidar_localization.sh`与`start_pose_forwarding.sh`；首个仅启动/重启所属定位，第二个仅启动/重启转发。前台关闭仅清理自身记录的会话；已解锁/有控制任务时保留后台会话。Qt按钮start复用健康会话，避免无意改变原点。所有会话状态/日志位于/tmp，迁移记录仅在电脑/GitHub。

边界：旧算法的RPY分量减零点不是通用刚体坐标变换，本次保留它以匹配实际部署；不能据软件等价认定飞控融合或物理外参已验收。物理雷达接入/漂移/实时负载、PX4接收与融合以及完整任务整链仍待验证。尚未切换正式运行环境，没有写PX4参数或执行飞行任务。

回退：保留独立原地面站副本，使用原程序即可回退桌面入口。机载以2b0c0f9为本阶段前的代码基线；新增管理器/转发无自启动服务，仅在确认未解锁、没有任务且停止所属会话后按版本恢复脚本/重建to_mavros。本阶段机载代码39edbb1；编译日志及源码摘要随记录保存。禁止在飞行中重启定位或更换估计器输入。

机载源码版本：`39edbb1`；本阶段已同步GitHub，机载Git干净、NoMachine正常。
