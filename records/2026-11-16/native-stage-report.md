# 原生 ROS2 迁移阶段记录

## 状态与断电恢复

2026-11-16 断电后已重新连接 .224；eMMC 系统可访问，运行代码基线 88ccb385135d9f74e5f0cba5aa50e7972535f132 已保存，工作树干净，NoMachine 服务 active。检查未见残留 FAST-LIO、RTSP 或验证进程。没有启用新的生产启动服务。

## 已部署与验证范围

| 模块 | 当前结果 | 仍需验证 |
| --- | --- | --- |
| Gemini335L | ROS2 wrapper/SDK v2.10.6，先前物理数据约30Hz | 长时运行、外参及任务链路 |
| MAVROS | 2.16.0，先前未解锁遥测验证 | 外部定位接收与PX4融合 |
| Livox | 复用SDK2 1.3.0，官方driver 1.2.6构建及私有库链接通过 | 新机雷达网络、数据和负载 |
| FAST-LIO | ROS2固定版本加平台补丁，43云/883IMU隔离回放输出40里程计与40点云 | 物理新机定位、参考轨迹、漂移 |
| ArUco | 原部署双码族算法移植，原始图像时间戳保留；合成图深度约1.495m/1.497m | 实际云台标定、光照、降落控制器 |
| RTSP | FFmpeg+MPP原生移植；合成H264灰度约14.96Hz、预览约9.58Hz；退出0 | 真实云台网络、重连、延迟 |
| EGO | 25个quadrotor_msgs和3个traj_utils消息构建 | 定制规划核心、预览/确认/停止/执行器 |

FAST-LIO 测试按源测量时间裁剪到共同IMU区间，避免ROS1采样接收排队延迟污染回放。权威结果为 `fastlio-aligned-replay-result.json`。RTSP最终结果为 `rtsp-reset-verified-result.json`；早期失败与ASAN日志保留供追溯，不代表当前结果。ArUco结果为 `aruco-synthetic-result.json`。

## 版本和适配决策

- 外部源码固定于根目录 `platform-sources.lock.json`，自行适配代码在 `platform/`，FAST-LIO改动在 `patches/fastlio-platform.patch`。
- Livox最新master依赖SDK1.3.0没有的Avia2/双回波API，选择含MID360s支持的官方1.2.6提交，避免替换底层驱动。
- SDK使用 `02_drivers/sensors/sdk2` 私有前缀，不覆盖系统库。
- FAST-LIO修正发布前协方差赋值、机体系速度与退出处理，禁用源码目录调试文件写入；保留旧机实际参数和 `/factory_fastlio` 话题。
- EGO ROS2不允许的大小写字段、时间类型迁移详见 `custom-message-field-migrations.json`；不能视为无差别ROS1桥接。
- RTSP修复MPP复位/析构顺序、extradata重复释放和关闭阻塞。当前图像时间为解码接收时刻，不是相机曝光时刻。

## 启动与现场限制

新入口均为手动运行，无自动解锁或飞行操作：`03_tools/config script/start_lidar_localization.sh` 仅启动定位，`start_aruco_vision.sh` 仅启动视觉。ROS1参考源码用COLCON_IGNORE隔离。

雷达旧主机地址192.168.1.50属于.235；新机配置预备192.168.1.51，但没有分配到网卡。启动文件会检查地址存在再启动，尚未改变雷达/网卡设置。物理迁移接线与网络切换需要另行确认影响。云台192.168.144.108网络尚未接入新机，relay未启用为服务。

## 恢复方法和后续

用户已有完整恢复镜像，本阶段不另做镜像备份。原生节点为独立工作空间，尚未切换正式任务环境；需要撤销本阶段运行时，停止对应手动启动终端即可，源码可按机载基线提交恢复。不要在运行节点或有未保存改动时使用强制git重置。

继续迁移定制EGO、人物推理、航点/降落控制器和地面站；飞控测试先采用mock/SITL。机载仅保留运行源码和配置，记录、日志、测试结果保存电脑和GitHub。
