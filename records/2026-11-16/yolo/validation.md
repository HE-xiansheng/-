# 人物推理与跟踪 ROS2 隔离验证

断电恢复：.224 SSH可达、eMMC读写挂载、NoMachine active；未变更服务、网络或飞控。

原生包 yolo_ultra_test 已独立安装，ROS2 Humble/Ubuntu22.04 ARM64，实际RK3588 NPU推理通过。RKNN API 2.3.2 (429f97ae6b)，RKNPU driver 0.9.6；使用与.235相同的私有ARM64运行库及模型，没有替换系统运行库或内核。

源码参考路径：.235 /home/orangepi/drone_platform/01_apps/mission_demo/src/yoloUltra_test；模型 SHA256 c918497014847b82201e30cab4079cb20b7d5b2d8434a0e3a1890d6a2ae8f04c；运行库 SHA256 d31fc19c85b85f6091b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8。均已只读核实与本次部署一致。

保留 /Safina_track 七字段及 /Safina_target_id，新增 /Safina_track_stamped 携带源图时刻、坐标系和会话状态。相机图像使用Reliable，MAVROS输入兼容SensorData QoS。加载ROS2参数与launch，保持OFFBOARD/SWA会话及ID行为。补上输入新鲜度检查、NPU错误传递、输出缓存的SDK释放、距离来源变化时滤波历史清理；深度投影使用完整R/t，而非仅旋转。

ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1；全部图像、遥控、飞控状态来自测试发布器，未发布控制指令、调用飞控服务或运行真实任务。18/18检查通过，详见bench.txt：正样本人物/ID、2m深度、图像与目标坐标系、七字段、过期/未来/零深度拒绝、测距来源恢复、空白图人物丢失、SWA降落/保护/重启、模式退出、图像/RC/状态中断、过期图像拒绝。缺失模型退出码1，未崩溃；正常退出可见rknn context destroyed。

首轮15/18：测试发布器共享同一Python时间戳对象，改深度时连带改了其他消息。修正为deepcopy后最终18/18。首轮不构成深度拒绝行为验证。

测试图来源 https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/assets/bus.jpg ，仅电脑暂存，不随仓库分发。官方硬件能力参考 https://github.com/airockchip/rknn-toolkit2 （README列出RK3588；实际版本匹配以本次加载/推理证据为准）。

限制：合成深度不是实际相机测距精度验证；继承.235的相机参数/外参尚须针对.224接入的实体设备校验。未完成真实视频多人物交叉/遮挡、丢失再识别、相机负载/延迟、跟踪控制器及SITL/物理任务链路。本阶段不证明可执行飞行。源码沿用原项目许可声明；原package许可为TODO，发布许可仍需核实。

重现：先source机载env.sh及mission_demo/workspace/install/setup.bash；在隔离域87/localhost启动 ros2 launch yolo_ultra_test tracking.launch.py，再运行 tools/test_yolo_ros2.py <电脑下载的bus.jpg>。生产启动仍未接入。撤销只需停止独立测试进程并取消该包overlay；已安装其他模块不受影响。
