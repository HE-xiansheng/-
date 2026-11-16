# 精准降落原生ROS2移植

参考.235当前 `/home/orangepi/drone_platform/01_apps/mission_demo/src/land_test`，已只读核实源码 `941551c354f3f23d2ab590ee4925fe680ba65cdcb2fc64214979d7ec65ca9d53`、配置 `92f6f51e6e037139b938d4b8302507dd0b2d1976c3f63840ad56ecf315c6ffa9` 以及纯控制头文件与电脑副本一致。

ament/rclcpp原生包保持land_test/Safina_land_test、/aruco_landing/enable、状态/表头/目标/路径主题。`ros2 launch land_test Safina_land.launch.py` 加载独立ROS2参数文件。原 `/aruco_landing` 参数空间改为节点参数；`aruco_marker`原配置保存在YAML，检测器仍需在联合启动时显式加载其独立配置。生产启动脚本尚未切换。

纯控制策略、运动限制、模式起动门控保持原文件；启动仍为POSCTL健康预发送后飞手切OFFBOARD，auto_takeoff=false。未将当前流程替换成历史PWM控制或简化降落。原service触发和可配置自动起飞代码保留，实际测试均使用手动空中接管、不会解锁或起飞。降落服务在后台线程使用非阻塞rclcpp请求及2秒超时清理，主执行器持续处理遥测/控制；退出不发飞控服务。ROS1 master发布者查询改为ROS2 graph端点检查；状态/表头使用Transient Local，飞控数据订阅兼容SensorData。默认诊断CSV只使用/tmp，并非机载持久迁移记录。

标记及机体位姿按源时间配对，继承向下光学坐标变换/姿态偏航/相机偏移语义，添加local_frame和marker_frame校验（默认map与gimbal_camera_optical_frame），状态/速度等同时校验源时间及单调接收时间。完整机体姿态与真实相机外参仍须物理核实；不声称已完成标定。

四组CTest全过：landing_task、mode_start_gate、motion_limits、takeoff_replay。回放只重现原5Hz测量数据的策略行为，不是飞机动力学验证。测试日志见ctest.log。

22/22 ROS2集成检查通过，使用ROS_DOMAIN_ID=87/ROS_LOCALHOST_ONLY=1、合成PoseStamped标记与伪MAVROS状态/服务。覆盖：POSCTL零速度预发送、保留schema、触发前无飞控请求、服务enable受模式策略约束、其他控制发布者阻止待机、90度偏航光学→ENU投影、标记坐标系/过期/未来/无目标拒绝、手动OFFBOARD边沿、全程无arm或OFFBOARD请求、高处目标丢失悬停和重新捕获、飞手退出、路径失效清除、状态源过期/错误位姿坐标系禁止输出、模拟下降到AUTO.LAND交接、首次服务拒绝后重试、落地解除状态确认COMPLETE、ENU有限值及速度限制。测试云台启动和任务控制均关闭，测试相机偏移/延迟置零；正常退出。

测试迭代：第一次脚本把ready保持缩短到0.4s，被原策略必须>1s的校验拒绝，随后恢复部署默认1.2s。第二次21/22：在1.2m丢标记期望悬停，但该高度达到原terminal_loss_depth，原策略转末端下降；没有修改策略来迎合测试。最终将高处丢失场景设1.5m，22/22通过；近地旧日志保留为行为证据，不能视为故障已被代码修复。

限制与后续：本测试直接合成标记位姿，ArUco真实图像→控制器联合验证尚未做；实体云台、视频延迟/时间戳、标定/外参、真实飞控接收/融合、SITL动力学、地面站启动和录制仍未完成。auto_takeoff/service工作流尚未ROS2集成测试，只有继承的纯策略测试覆盖，不得认定这些路径完成。真实飞行未执行。

独立包未接生产服务。退回本阶段之前：停止对应独立测试进程，取消新包launch/overlay；其余驱动及应用安装保持可用。已停止本阶段测试并清理机载测试脚本、图片、CSV、构建/ROS临时日志。代码提交保留机载，迁移文档/验证日志仅电脑与GitHub。
