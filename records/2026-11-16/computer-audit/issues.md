# 当前问题与未验证项

状态基于2026-11-16现场检查，不能表示此后进程始终不变。优先级表示对迁移决策与验收的影响；“尚未验证”不等于已确认故障。证据路径相对本审计目录。

| ID / 优先级 | 状态与事实 | 影响与后续动作 | 证据 |
|---|---|---|---|
| I01 / 高 | FAST-LIO、转发、规划与任务控制在快照中未运行；定位话题只有订阅者 | 无法核实LIO10Hz/转发30Hz或融合；后续安排独立定位基线，不能把FC本地位姿30Hz等同于定位有效 | `raw/remote_graph_final.json`、`raw/remote_sample.json` |
| I02 / 高 | 现场转发仅有安装二进制；本机新源码与候选二进制不能证明匹配已部署版本 | 缓存位姿、源戳、30Hz重复观测和坐标行为必须查明；为ROS2建立可追溯实现与对照输入 | `raw/remote_supplement_final.json`、地面站`tools/localization/onboard/`、`fast_lio/management/board.py` |
| I03 / 高 | 用户已选原生22.04/Humble；官方5 Ultra有Jammy的5.10/6.1支持 | 需锁定实际镜像、rknpu/Runtime及MPP/RGA组合并在独立介质验证；不原地升级或自动写盘 | `official/orangepi-ultra.html`、`official/orangepi5ultra.conf` |
| I04 / 高 | FAST-LIO/EGO官方ROS2分支以Humble为例，现场有速度与任务扩展 | Humble下轨迹schema、定制补丁和任务行为仍需实际移植；不能标记为现成完整替换 | `official/fastlio-ros2.md`、`official/ego-ros2.md`、机载源码快照 |
| I05 / 高 | 仅测到局部TF子树，完整LIO/机身/相机/飞控关系及原点未证明；LIO线速度已为body | 需建立明确刚体变换、标定版本和轴向用例，防重复速度旋转/ENU-NED转换 | `raw/remote_sample.json`、`fast_lio/config/lidar.yaml`、`laserMapping.cpp`、EGO launch |
| I06 / 高 | PX4配置EV_CTRL15，但水平估计有效标志false，vision_pose当前无发布者 | 需真实接收、融合/创新和延迟证据；当前不能声称雷达融合正常；任何参数调整另经用户确认 | `raw/px4_parameters.json`、`raw/remote_sample.json` |
| I07 / 高 | 地面站与机载任务管理器159行diff，定位管理器副本一致 | 审定任务语义和部署基线，先以现场版本为准；不能自动覆盖远端 | `raw/source_drift.json`、`raw/task_manager_drift.diff` |
| I08 / 高 | Qt录制回放支持ROS1 bag v2；尚无rosbag2全流程 | 必须补新格式解码或集成导出，并保留旧记录，否则迁移会丢失原操作流程 | 地面站`pointcloudfileloader.cpp:1225`和录制/记录管理源码 |
| I09 / 中 | 系统RKNN1.4.0与工程Runtime2.3.2共存；检测实际链接后者，rknpu0.9.6 | 依赖锁定按实际ldd路径；需保存模型/运行库和验证新用户态ABI/NPU结果；本轮只读摘要并未复制这些二进制 | `raw/remote_supplement_final.json` |
| I10 / 中 | Orbbec旧安装2.5.4与活动workspace2.10.6共存，固件1.4.60；上游建议1.8.10 | 保持实际加载路径与固件；选择经验证SDK组合，不自动刷新固件 | `raw/remote_inventory_resumed.json`、`raw/remote_sample.json`、`official/orbbec-ros2.md` |
| I11 / 中 | 开机出现墙钟跳变，MAVROS有time jump日志；检查期间经历重启 | 记录boot ID/单调时钟；检查源戳、同步、缓存清理及重启原点；不能混用不同boot PID | `raw/remote_inventory*.json`、`raw/remote_supplement_final.json` |
| I12 / 中 | 现场雷达IMU静止加速度约1，FAST-LIO启用软件时间同步 | 先核对驱动与算法单位/处理，不把ROS2常用SI约定直接套入；检验单点相对时间与硬件/软件同步策略 | `raw/remote_sample.json`、`fast_lio/config/lidar.yaml` |
| I13 / 中 | ROS2 Header无seq；MAVROS2视觉订阅reliable且同戳去重 | 重新定义同帧/请求标识、QoS与过期策略；不能只照搬ROS1队列和30Hz定时器 | `official/mavros2-vision-pose.cpp`、ArUco/转发源码 |
| I14 / 中 | `usb_cam`有Error starting device/缺标定日志，未枚举到对应独立USB相机 | 核实是否为仍需保留的物理设备/历史入口；作为独立未解决项，不归咎Gemini，不自动删启动项或停服务 | `raw/remote_supplement_final.json`、`raw/remote_inventory_resumed.json` |
| I15 / 中 | 部署树无Git；旧mission_demo校验文件含NUL；本轮只采集文本与二进制摘要 | 新建独立基线仓库并校验；完整SDK/模型/固件/系统恢复备份仍待后续准备 | `raw/source_manifest.json`、`raw/groundstation_manifest.json` |
| I16 / 中 | 22.04无官方Noetic二进制；ros1_bridge仍需自行构建兼容环境与自定义消息 | 优先原生ROS2和标准显示接口；旧ROS1保留在原20.04；桥按类型/方向白名单验证 | `official/ros1_bridge.md`、`migration_matrix.md` |
| I17 / 中 | 服务入口包含进程清理/设备锁处理；任务“准备”后可受RC边沿触发控制 | 不能为审计重跑服务或真实任务；后续用mock与独立输出验证状态机 | `Safina_sensor.sh`、航点/任务管理器源码 |
| I18 / 低 | 未观察到Qt GUI进程；仅见点云管理器 | 未现场验收全部按钮、视频与任务反馈；后续用模拟后端做完整界面回归 | `raw/groundstation_manifest.json` |

## 已澄清，避免误报

- **Gemini深度和左右IR可读。** 标准ROS订阅每流取得3帧（848×480，深度16UC1，IR mono8），见 `raw/camera_standard_subscription.json`。最初直接TCPROS无帧属于采样限制，不是确认的相机故障；持续帧率、标定精度和长时负载仍待验收。
- **推理并非依赖Jetson/CUDA。** 机载为RK3588，检测使用C++ RKNN。实际加载Runtime2.3.2，不能依据系统目录1.4.0判断检测依赖。
- **旧路径缺失是现场已有状态。** 本轮没有移动或删除TECH与lidar_tools；应适配实际drone_platform入口。
- **LIO与EGO均有官方仓库ROS2分支。** 其存在不代表已覆盖现场定制业务，也不代表已在Humble/当前硬件验证。
- **进程记录文件不等于活动进程。** `active-*.json`含旧PID，必须结合boot、进程启动tick与实际命令核实。

## 下一阶段顺序

先固定接口/有效配置及转发语义，准备5 Ultra专用22.04镜像/恢复方案与独立Humble工作空间及回放；优先完成传感器消息、定位和地面站只读数据。真实设备窗口、飞控融合和控制任务分别推进，按 `validation_and_rollback.md` 保留证据及回退条件。
