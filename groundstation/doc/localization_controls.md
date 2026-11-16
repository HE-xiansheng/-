# 左侧 FAST-LIO 定位与转发

实时模式下，侧栏最上方增加「FAST-LIO 定位与转发」，下方继续保留避障预览、人工确认和手动录制。

- **启动定位与转发**：复用 `~/drone_platform/01_apps/fast_lio/management/board.py` 的 FAST-LIO 会话和 `flight-pose-relay.py` 的原厂 Pose 转发。已运行且来源符合预期时复用当前进程；没有定位时启动原入口，取得有效定位输出后再启动转发。启动后保持机体静止等待初始化。按钮不会自动重启健康定位。
- **停止**：先停止本工具拥有的转发，再停止本工具拥有的定位；不停止 MID360 驱动、MAVROS 或手动录制。需先结束避障预览/执行器、落地并锁定飞控。已解锁、OFFBOARD、飞控状态过期、存在控制发布者或来源冲突时拒绝操作，并在卡片中显示原因。
- **定位 / 转发状态**：使用现有只读 PFS1 采样；「正常」要求话题输出新鲜、来源和坐标系符合预期。只有 PID 存活时显示「运行中 / 等待数据」，不会把初始化当作准备就绪。采样断流时显示未知并禁用按钮。

真实话题链路保持不变：`/livox/lidar`、`/livox/imu` → `/factory_laserMapping` → `/factory_fastlio/odometry`、`/factory_fastlio/cloud_world` → `/factory_vision2mavros` → `/mavros/vision_pose/pose`（修复后每条新测量最多转发一次；当前源约 10 Hz，30 Hz 参数仅作为发布频率上限）。定位采用 `camera_init` / `body`；沿用现有规划接口的 world 别名与既有 PX4 转换。此功能不新增坐标变换、不改雷达外参、不写飞控参数、不发送解锁或模式切换请求。

新定位会话意味着新的初始化原点。只读采样以机载 boot ID 和存活定位进程 PID/start tick 标记会话；会话变化时清空地面站旧累计点云、显示轨迹及人工确认，需重新启动预览并选点。不会把旧 PCD 当作已重定位地图。已存在会话首次接入地面站时保留其当前预览能力。

实现为地面站内置 `tools/localization/manage_stack.py`，通过 SSH stdin 调用已有机载 Python 管理模块；不安装第二套定位/转发程序。与原入口共享定位、转发、预览和执行器生命周期锁；源码校验或进程身份不匹配时不自动覆盖或结束进程。正常操作只显示简短状态，完整返回保留在地面站日志。

运行：`bash /home/tyzk/dianyunGCS-review-20261026/connect-live-pointcloud.sh`。保持先启动定位与转发，再启动避障/预览、地图选点、人工确认的顺序。录制仍由独立「开始录制 / 停止录制」按钮控制。

## 2026-10-30：保留定位源时间戳

机载转发已更新为 `tools/localization/onboard/vision2mavros.cpp`：保留 FAST-LIO `header.stamp`，不再以当前时间重新盖戳；只发布严格递增的新测量。默认 `max_source_age=0.2s`、`future_tolerance=0.02s`，接收和发布时均检查，发布前另检查单调时钟接收年龄。零戳、重复、乱序、过期、过大未来偏差、非法坐标/四元数和错误frame拒绝转发。源停止后不会继续把缓存当成新观测发送。源时钟倒退不会自动放行，需在地面按正常入口重启定位会话。

保留原100个有效新测量初始化、原点平移、原姿态零点处理与雷达安装偏移；本次未修改这些几何模型或飞控参数。转发频率由原先重复30Hz变为约10Hz的真实新观测，不应通过重发旧测量凑频率。测试验证时间戳/频率/拒绝及恢复行为，不代表已完成修复后实飞振荡验收。

部署前确认地面锁桨，停止上轮已取消ArUco任务，通过原定位管理入口重启定位会话。新会话原点已重建，旧选点/地图不能直接沿用。原二进制/源码与运行记录备份位于机载 `~/drone_platform/01_apps/fast_lio/deployment-backups/relay-timestamp-20261030-154210/`。备份恢复也必须满足地面锁桨、无活动控制器，并先停止转发；不能在飞行中替换/重启。

隔离验证：source ROS 环境后执行 `python3 tools/localization/test_relay_timestamps.py <编译后的vision2mavros>`；脚本强制随机localhost ROS master，将输出重映射到测试话题，不接入真实飞控。详细测试和部署回读见 `analysis/relay-timestamp-fix-20261030/`。

## 手册中的终端命令

保留 `slam` 环境切换及原来的两条启动命令。新开终端后，在第一个终端执行：

```bash
slam
roslaunch fast_lio mapping_mid360.launch
```

保持机体静止等待定位初始化，在第二个终端执行：

```bash
slam
roslaunch to_mavros vision2mavros.launch
```

已经打开的旧终端可先执行一次 `source ~/.bashrc`，再运行 `slam`。两个入口都通过 `localization/management/manual_session.py` 复用当前定位管理器；实际算法、配置、会话登记和话题与地面站按钮一致。第一条仅启动定位，第二条取得有效定位数据后才启动位姿转发。重复执行定位命令时先停转发、停止原定位并重新初始化；若原转发正在运行，取得新鲜定位后恢复转发。重复执行转发命令只重启转发。重启定位会重建原点，必须在落地锁定、停止任务与预览后执行；原终端随旧会话结束而退出。地面站启动按钮仍复用终端已创建的会话。

在地面锁定飞控并结束任务、预览后，转发终端按 Ctrl+C 只停止转发；定位终端按 Ctrl+C 先停止关联转发，再停止本终端创建的定位。若已解锁、OFFBOARD、状态过期、业务仍在运行或会话归属变化，则拒绝停止，后台定位继续保留。满足停止条件后可通过地面站停止。此兼容只保留启动命令，输出仍使用上文 `/factory_fastlio/...` 话题；手册中直接检查旧 `/Odometry` 等话题的步骤仍须更新。
