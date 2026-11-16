# 逐模块迁移清单

这是后续实施清单，不是已完成迁移声明。现有 ROS1 工程与运行路径保持原样；当前查到的上游 ROS2 分支仅作为代码来源。目标已确定为原生Ubuntu22.04 ARM64 + ROS2 Humble。

| 模块 | 可复用 ROS2 实现 | 必须补做的源码/配置适配 | bridge 过渡方式 | 主要验收 |
|---|---|---|---|---|
| Livox MID360s | 官方 `livox_ros_driver2` + SDK2 | 对照现场MID360s类型35、设备配置、主机/雷达IP、端口；生成CustomMsg/CustomPoint；核对timebase/offset_time/IMU单位；ARM64构建 | 原始CustomMsg需两边同构消息与桥的源码重建，吞吐大，优先原生ROS2或录包 | 单点时间保留、10Hz点云/200HzIMU对照、时间单调与长时无丢帧 |
| Orbbec Gemini335L | 官方 `OrbbecSDK_ROS2/v2-main`，列出Humble/Jazzy/Lyrical | 保留现场固件1.4.60；选择对应SDK/wrapper；映射彩色/深度/左右IR话题、16UC1/mono8、CameraInfo、TF、按需启流、深度单位/对齐方式 | `Image/CameraInfo/PointCloud2`可短时桥接；不假定ROS1 `orbbec_camera`与ROS2 `orbbec_camera_msgs`自动映射 | 各流正常订阅出图、内参和深度尺度、时间同步/掉线恢复、实际USB3负载 |
| usb_cam | 已有ROS2驱动可评估 | 当前日志报Error starting device且缺标定文件；先查是否为仍需保留的真实独立相机 | 标准Image可桥；无设备时不得伪造正常 | 明确用途和物理设备；若属于历史入口须保留兼容并标明不可用 |
| FAST-LIO | hku-mars/FAST_LIO官方仓库`ROS2`分支（README建议Humble） | 移植本机body线速度补丁、有效配置、话题/TF重映射、软件同步策略；核对ROS2分支单位与缓存行为、PCL/Eigen/ikd-tree与Humble依赖/编译配置 | 优先桥接输出Odometry/PointCloud2/TF；不要同时运行两个雷达驱动争抢端口 | 同包对照轨迹、运动畸变、速度方向/数值、初始化、重启原点/时间跳变、CPU与内存 |
| 位姿转发to_mavros | 没有已确认可直接替换的等价上游 | 现场只有安装二进制；按接口重写rclcpp/rclpy并与旧行为对照；保留外参/原点定义，明确源戳、缓存过期与初始化策略 | 可用PoseStamped/Odometry作为临时边界，但只能有一个真实FC输入源 | 不是只测30Hz；要测独立源时间数、延迟、重复/过期数据、坐标、断流后行为 |
| MAVROS/PX4 | MAVROS2；本次Humble发行索引2.17.0-1，实施时锁定实际deb/源码版本 | 继续MAVLink/USB，移植插件/参数/GeographicLib数据路径；校验topic/service命名、QoS、PositionTarget mask与ENU/FLU→NED/FRD | 短期保留ROS1 MAVROS，单向桥接定位输入/遥测；控制服务默认不暴露到桥 | PX4实际收到外部估计、EKF融合/创新、延迟和丢失输入响应；不改PX4固件 |
| EGO规划核心 | 官方指向`ego-planner-swarm/ros2_version`，以Humble为例 | 确认PolyTraj/MINCOTraj等实际schema；迁移现场rolling map、odom_vel_is_body、地面/虚拟墙/限制参数与真实LIO输入 | 可先ROS2定位→ROS1规划；规划到FC执行始终留在同一侧，避免桥传高频控制 | 相同地图/起终态可行性、碰撞检查、轨迹连续性、重规划耗时；比较FastDDS/CycloneDDS |
| EGO预览/确认/PX4执行 | 现场自定义实现 | 移植manual_session、map_navigation、candidate worker、原生execution轨迹、人工确认与受控停止；保留预览和执行分离 | 暂时整体保留ROS1一侧，桥输入状态和显示 | 只预览不控制；确认绑定候选/会话；换目标/接管/断流/停止不执行旧轨迹 |
| 航点与拍照 | 现场`waypoint_mission`，不是通用ROS2航点包替换 | roscpp→rclcpp、参数/launch、异步服务回调、RC边沿、帧变换、拍照与状态字段 | 保留ROS1原生执行器；桥定位和遥测，任务命令先通过原SSH管理器 | mock/SITL检查准备→触发→航点→拍照→返航/降落及超时/接管；本轮不执行真实任务 |
| ArUco检测 | 可复用OpenCV算法；不直接替换为另一ROS2 ArUco节点 | 图像/CameraInfo接口、选码/位姿策略、源帧戳、ID/Pose/metrics一致性、云台optical/receipt帧约定、OpenCV contrib API | 图像和检测Pose可单向桥；状态JSON/CSV语义另外验证 | 已知标定距离/旋转、同帧关联、采集vs接收时间、无目标/错码/旧图拒绝 |
| 精准降落 | 现场`land_test`中的策略类可复用 | ROS接口/状态机/参数、云台朝下动作、光学→机体→局部系转换、时间配对和MAVROS服务并发 | 先整个控制器留ROS1侧；桥检测结果时要求源戳/坐标不变 | mock/SITL验证引导/丢码/限速/接管、停止语义；未经后续授权不在真实飞控触发 |
| 人物检测 | C++ RKNN推理核可复用，无CUDA需求 | 保留模型、Runtime2.3.2与驱动0.9.6配对；移植cv_bridge、RGB/深度对应、目标选择、结果和显示话题 | 先桥标准图像/检测输出；Float32MultiArray可桥但缺Header，应额外保持源时间语义 | 同图/同模型输出一致、深度尺度、目标ID与丢失、端到端延迟和NPU负载 |
| 人物跟踪控制 | 现场`track_test` | ROS接口、RC/SWA门、距离来源/有效性、FRAME_BODY_NED与MAVROS转换、超时与控制互斥 | 先与MAVROS保持同侧；不做双向setpoint桥 | mock/SITL验证前后/偏航方向、距离/高度控制、RC接管、目标断流 |
| 云台视频/拍照 | MediaMTX与ROS版本无关；MPP解码核可复用 | `rtsp_test`的ROS包装、时间戳/帧、MMAP/MPP/RGA依赖路径和ABI，云台命令独立管理 | 可桥Image，但视频优先保留RTSP原链路 | RTSP首帧/恢复/延迟、硬解CPU、图像源戳与截图关联 |
| 地面站Qt界面 | 现有Qt/C++/QML可保留 | rclpy后端替换rospy/XMLRPC、ros2 launch管理、会话身份和状态；SSH入口不再硬编码Noetic/catkin | 保留现有SSH/UDP协议，在机载一侧换后端 | 原按钮/预览/确认/停止/状态/断线重连/视频切换全流程 |
| 录制/回放 | rosbag2，优先评估MCAP | 录制管理器、metadata/分包/完整结束判断、类型定义、下载API、Qt查看或兼容导出 | ROS1控制期可继续ROS1 bag；两侧记录必须可关联源时间 | 旧bag仍可读；新录制能在原界面列出、下载、显示点云和轨迹 |

## 必须锁定的接口与语义

1. 自定义消息先建立独立接口包。最低清单：`livox_ros_driver2/{CustomMsg,CustomPoint}`、`traj_utils/{PolyTraj,MINCOTraj,DataDisp}`、使用到的`quadrotor_msgs`消息。MAVROS使用目标版本的`mavros_msgs`，核对字段和常量，不混用ROS1生成代码。
2. ROS2的标准Header没有ROS1的`seq`。ArUco metrics同帧校验、点云分片序号和任务确认不能机械删除seq；需要独立帧/请求标识，并保持旧网络协议序号。
3. QoS逐接口约定。雷达/IMU/图像可用SensorData配置，但必须匹配订阅者；MAVROS2 vision_pose源码当前是默认reliable订阅，best_effort发布者无法满足它。先对照`ros2 topic info -v`与实际数据再固定。
4. ROS1 latch对应ROS2 transient_local，但只适合静态TF、显示状态等；不得让旧目标或旧控制命令因持久化自动重放。`/tf_static`要验证晚加入订阅者可获得变换；`/factory_fastlio/tf`的隔离意图保留。
5. ROS2参数属于节点，原先全局`<rosparam>`不能原封不动加载。建立`ros__parameters`配置、显式声明参数，并验证namespace/private topic，避免静默使用默认值。
6. 时间采用测量源戳，超时采用单调时钟；处理boot更换、NTP跳时、rosbag2 use_sim_time与真实硬件时间。转发“30Hz”不等于30个新观测。MAVROS对同戳数据会去重，不能伪造新戳来满足表面频率。
7. 位姿/速度/加速度/四元数方向、ENU/NED与FLU/FRD转换只进行一次。`camera_init`和FC的`map`必须用明确的刚体变换/原点定义关联；初始欧拉角逐项相减不等价于通用刚体旋转。
8. 使用ROS2多线程执行器时重新验证共享状态锁、任务代次、已排队服务、接管后的请求取消。ROS1单线程下的时序不能直接假设继续成立。

## bridge 的可用边界

`ros1_bridge`只是临时接口方案，不是把现有20.04换成24.04后继续原样运行的方案。官方说明24.04没有ROS1且原生不支持桥；桥必须在一个可以同时构建两套ROS的独立运行环境中构建，并连接现场ROS1网络。

当前目标Jammy/Humble也不提供官方Noetic二进制；如确有桥接需要，在独立环境自行构建ROS1与bridge，并连接保留的20.04 ROS1系统。这是额外维护成本，原生ROS2模块优先，不强制经过bridge。不要从README的20.04/Humble表格推导“Focal官方支持Humble apt安装”。Jazzy与Humble跨发行版DDS通信也不是官方保证的兼容边界，不能把Humble桥直接当作Jazzy桥。

优先过渡的是`Odometry`、`PointCloud2`、`PoseStamped`、`Image`、`CameraInfo`和必要TF；通过明确白名单约束方向。自定义类型需要先编译两侧消息，再重建桥并检查类型映射。初期不桥接解锁、模式切换或高频设定点，也不同时让ROS1/ROS2向同一飞控输入定位。

## 版本锁定起点

本次读取的上游分支HEAD（不代表已选定生产版本）：

| 项目 | 分支 | SHA |
|---|---|---|
| Livox driver2 | master | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` |
| FAST-LIO | ROS2 | `cc3c9e8b1469b33b8072764d6db2348ed6dc597f` |
| EGO swarm | ros2_version | `23a8d5a191711dd65633df689bd00f55d4dea8f9` |
| Orbbec ROS2 | v2-main | `ffba347cadc96d0457a91af59fc06cdf86052728` |

实施时生成`repos.lock.yaml`，同时锁定submodule、补丁、deb、编译器、容器镜像digest、SDK/模型/固件和配置摘要。上游算法分支建议Humble，但本机定制代码、SDK/模型与BSP组合仍须实测，不能在此表标记为“已兼容现场”。
