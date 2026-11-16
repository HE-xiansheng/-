# 人物跟踪控制器 ROS2 迁移

参考.235当前track_test源码，SHA256 `5705a5556e38343d75343ca6b0812c5d74bbe91f6fd00d7e4fb55561afc45e0f`，参数 `8154166d4ceaa74b36fb3c6af8aad3fea72e083d6d9e096ddfc74c79efe431b2`，均只读核实。旧机检查时未运行跟踪/降落控制进程。

原生ament/rclcpp包保留track_test、Safina_track_test命名，ROS2 launch为Safina_track.launch.py。继承.235定距2m、最大前进0.3m/s/后退0.15m/s、偏航及高度控制、斜率限制、OFFBOARD进入保持和SWA上保持/中跟踪/下原地降落流程；进入时SWA已在下档不会触发降落。MAVROS使用SensorData兼容订阅、位置输出ROS ENU/FRAME_LOCAL_NED，速度输出ROS body FLU/FRAME_BODY_NED；MAVROS插件负责转换到PX4 NED/FRD，未手工重复转换。

原ROS1通用OffboardController在构造中阻塞等待/预发送、析构中可能请求AUTO.LAND，并携带未使用的解锁/切OFFBOARD辅助函数。本次原生实现通过状态回调和持续定时保持完成预发送，不复制未使用辅助函数，退出不发服务。AUTO.LAND使用非阻塞rclcpp客户端，有请求超时清理；只有任务降落分支、飞控连接且已解锁、OFFBOARD、位姿和RC有效、SWA下档时可请求。没有解锁服务客户端。

默认订阅/Safina_track_stamped，使用源图时间戳及session_active；旧七字段/Safina_track仅可显式accept_legacy_target=true启用（无源时间戳能力）。保留旧话题供地面站显示。拒绝非有限/越界目标、过期/未来/零时刻，双时钟检查接收与源时刻；位姿四元数/坐标系一致性验证，速度过期不参与高度阻尼。飞控断开或状态过期停止发布；落地状态过期阻止跟踪；位姿/RC过期暂停降落，以零速度输出；接受AUTO.LAND后不重复请求。参数、计时与launch均为ROS2原生。

36/36项检查通过，使用伪PX4消息和服务，ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1，无真实MAVROS/飞控连接；运行命令和结果见bench.txt、node.log。正常退出无飞控服务副作用。

限制：尚未完成NPU检测器与本控制器的联合运行、真实相机多目标遮挡、SITL动力学和真实任务地面验证。没有新增完整任务管理器/控制权互斥，启动器仍不得同时启动EGO、跟踪、降落等控制器。未替换正式启动脚本或服务。landing_test、航点控制及地面站仍待移植。

回退：停止独立节点，取消该包overlay或恢复机载上一源码提交；其他模块不受影响。测试日志仅电脑/GitHub，机载/tmp临时文件在收集后清理。源码许可沿用原包TODO，许可范围待核实。
