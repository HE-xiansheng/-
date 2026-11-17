# ROS2 地面站独立适配版（进行中）

## 当前ROS2迁移副本

此目录是适配Ubuntu22.04/Humble的独立Linux地面站副本，面向机载192.168.1.224；原电脑地面站及.235 ROS1参考机未被替换。ROS2入口、兼容范围和当前验证结果见[ROS2部署说明](doc/ros2_deployment.md)。下方ROS1历史部署、旧工具和早期Windows说明按原时点保留，不能作为向.224安装ROS1脚本的操作指南。


本目录来自电脑当前地面站源码的独立副本，来源摘要见 `../records/2026-11-16/groundstation/source-manifest.json`。不覆盖原程序、桌面入口或原地面站设置。Qt 应用设置使用独立名称 `customgeometry-ros2-staging`，默认机载地址 `.224`，可用 `--host` 指定。

当前已适配：ROS2任务管理入口、只读点云/位姿/飞前遥测桥、视频图像采样器、定位和规划操作入口。使用统一 `~/drone_platform/env.sh`，保留原PCD1/LZ4、POS1、TLM1/TLG1和PFS1格式；ROS2实际节点/话题和camera_init坐标系已接入显示。

Linux 窗口启动的点云/飞前只读桥接随所属窗口退出而结束，并关闭自己的 SSH 读者，避免关窗后继续重连和消耗机载资源。直接从命令行启动的独立桥接仍使用原启停入口；关闭地面站窗口不停止机载传感器或定位。实现与验证见 [ROS2部署说明](doc/ros2_deployment.md)。

Qt连接入口已通过真实SSH、跨机UDP及合成ROS2输入检查，包括传感器小字段、状态、轨迹、栅格和断流过期。机载进程内20项、电脑纯显示/分片/新鲜度及Qt接收检查见 `../records/2026-11-16/telemetry/`。本阶段无需电脑安装ROS2，也没有新装机载服务。

Linux 桌面候选默认使用 `QSG_NO_VSYNC=1`，避免已复现的 X11 垂直同步等待拖慢云台视频管道；显式环境设置会原样交给 Qt；需要恢复旧显示行为时使用原 r2 独立包。此设置影响显示调度，不更改视频源、飞控或任务频率。实测范围及候选包说明见[ROS2部署说明](doc/ros2_deployment.md)。

试验版本机桥接状态使用 `~/.local/share/drone-gcs-ros2`（或`XDG_DATA_HOME/drone-gcs-ros2`），可通过`GCS_STATE_DIR`覆盖，与原 `.pointcloud-link` 独立。机载状态分片默认发至电脑UDP9015，`--raw-udp-port`可指定；主界面PFS1仍为本机9003、点云9000、位姿9001。测试使用独立端口及状态目录，不接管原桥接。机载发布节点和采样器应使用orangepi用户；root跨用户DDS首轮测试没有收到消息，尚未作为支持用法验收。

录制、下载与回放的Qt入口已完成原生ROS2适配，并通过独立SSH与SQLite/CDR回放验证；实时可用状态以机载返回结果为准。独立Linux包已完成重定位、主界面首帧与实际SSH遥测启停检查，打包方法见ROS2部署说明。现阶段不切换正式程序或执行物理任务；真实雷达、飞控融合、相机/云台任务联合验证尚未完成。实际机载任务准备可能启动控制节点，仅在完成地面验证并确认后使用。

视频采用原生rclpy、Reliable depth=1，保留VID1/JPEG字节协议。原画订阅压缩图，任务画面订阅原有标注话题；独立人物预览使用同版本RKNN模型的`person_preview`，仅发布图像。源戳为零、过期、明显超前或未递增的图像不刷新显示；慢网络只保留最新待编码帧。预览进程按用户和ROS域加锁，复用已有发布者，只回收自身启动的检测进程。机载31项、Qt26项、真实SSH到Qt7项及本地3项检查通过，见`../records/2026-11-16/video/`。物理云台RTSP、实机延迟与任务画面联合尚未验收；原RTSP接收路径保留。

规划入口已接ROS2原生会话/目标/执行器；停止保护使用同一执行器管理锁，等待执行器退出与新鲜非OFFBOARD状态后才停止所属规划会话。机载23项、Qt15项隔离检查见`../records/2026-11-16/planner-ui/`，实际飞控整链仍未验收。

定位按钮使用机载原生ROS2管理器，复用健康定位原点；100个新测量完成原部署转发初始化，10Hz输入最多10Hz唯一输出。相同管理锁及所属会话保护、真实FAST-LIO回放与前台退出检查见`../records/2026-11-16/localization/`。物理雷达与PX4融合尚未验收。

独立构建（电脑无需安装 ROS2）：

```bash
/home/tyzk/dianyunGCS-review-20261026/.venv-build/bin/cmake -S groundstation -B /home/tyzk/drone-migration-work/build-groundstation -DCMAKE_PREFIX_PATH=/home/tyzk/dianyunGCS-review-20261026/.deps/Qt -DCMAKE_BUILD_TYPE=Release
/home/tyzk/dianyunGCS-review-20261026/.venv-build/bin/cmake --build /home/tyzk/drone-migration-work/build-groundstation -j4
```

以下为继承版本说明，ROS1机载操作以本节限制为准。

---

# 室内点云航线规划交互 Demo


机载 ArUco、RTSP、精准降落和航点 demo 已更新为 `keyan_xzd/bf049e68` 实现，硬件与地面站适配保留；参见 [机载任务源码说明](tools/missions/onboard/README.md)。

## 当前地图选点流程（2026-10-29）

机载规划包的 ROS 名称已统一为 `ego_planner`；通过 `ego_planner/run_board.sh` 启停预览，节点、话题及监视数据字段同步统一为 ego_planner；不再使用旧前缀。

启动定位与转发 → 启动避障/预览 → 地图选点。选点后直接更新原生目标并自动准备PX4接入，不再点击人工确认。人工POSCTL起飞，遥控切OFFBOARD执行；切回POSCTL接管。同一次OFFBOARD可直接更新目标，不要求先到达或悬停。候选、令牌、10cm漂移和候选地图范围/额外复查工作流已停用；原生膨胀、碰撞、动力学和失败重规划保留。机载安装记录见 `tools/planner/automatic-navigation/`；下方旧版人工确认示例只代表历史实现。


本项目包含两个可执行程序：

- `customgeometry.exe`：点云地面站，负责显示实时/静态障碍点云、在空旷区域生成航点、拖拽航线避障、上传完整航线或单个航点。
- `radarsimulator.exe`：Livox MID-360 风格室内点云模拟器，负责通过 UDP 发送室内点云，并监听地面站回传的航线。

两个程序用于验证“室内点云建图结果 + 航线规划交互 + UDP 回传”的初版流程。

## 文档索引

- [产品功能参数](doc/product_function_parameters.md)：当前已实现能力、预留能力和未实现边界，适合交付前确认功能范围。
- [UDP 协议说明](doc/udp_protocol.md)：`PCD1/raw`、`PCD1/LZ4`、`POS1`、`GOL1`、`RTE1` 的数据格式和版本演进建议。
- [真实 Livox 接入说明](doc/real_livox_integration.md)：后续接入 MID-360/MID-360S 或 ROS 点云链路时的适配方式。
- [QGC 集成建议](doc/qgc_integration_notes.md)：点云高吞吐链路与 MAVLink/QGC 控制链路的职责划分。
- [航点 ID 协议提案](doc/waypoint_id_protocol_proposal.md)：在不修改远端代码前提下的现状说明，以及后续 `GOL2`/扩展协议建议。

## 快速运行

1. 启动地面站：

```powershell
.\customgeometry.exe
```

2. 启动模拟器：

```powershell
.\radarsimulator.exe --host 127.0.0.1 --point-port 56000 --route-port 56001 --hz 5 --points 40000
```

默认配置等价于每秒约 200000 个点：`5 Hz * 40000 点/帧`。单帧点数较高，可以让实时画面更接近室内建图后的点云效果。

3. 在地面站中设置“航点高度”和“安全半径”，短按点云中的空旷区域生成两个自由空间航点。
4. 拖动黄色航点或航线线段插入避障折点，地面站会在上传前检查航点和线段是否靠障碍物过近。
5. 确认后点击“上传航线”。航线只会在点击按钮时上传，拖拽过程不会实时回传。
6. 如果机载端只接受单个目标点，也可以在航点列表中点击对应航点右侧的“发送”，只上传这一个航点。

## UDP 端口

- 点云端口：地面站监听，默认 `56000`。
- 航线端口：模拟器监听，默认 `56001`。
- 坐标单位：米。
- 坐标系：X 前、Y 左、Z 上。

点云协议代码在 `pointcloudprotocol.h/.cpp`，点云接收和航线发送代码在 `udptransport.h/.cpp`。

完整线格式见 [doc/udp_protocol.md](doc/udp_protocol.md)。航点 ID、完整任务下发等尚未和远端确认的扩展方案见 [doc/waypoint_id_protocol_proposal.md](doc/waypoint_id_protocol_proposal.md)。

## 香橙派 something1017 接入

项目根目录已保留从香橙派 `orangepi@192.168.1.196:/home/orangepi/drone_platform/02_drivers/sensor_ros` 拷贝来的 ROS/catkin 工程。它的 UDP 协议和本机模拟器不同：

- `udp_uav_tx_node`：订阅 ROS `PointCloud2`，把 xyz(float32) 做 LZ4 block 压缩后，以 `PCD1` 分片发给 PC，默认点云端口 `9000`。
- `udp_uav_tx_node`：订阅 ROS `PoseStamped`，以 `POS1` 单包发给 PC，默认位姿端口 `9001`。
- `udp_uav_tx_node`：监听 PC 发来的 `GOL1` 单目标点，默认目标点端口 `9002`。

地面站现在兼容两套点云格式：本机模拟器的未压缩 `PCD1`，以及香橙派 ROS 工程的 `PCD1/LZ4` 压缩分片。使用香橙派时，在左侧 UDP 通信区点击“套用香橙派 UDP”，它会自动设置：

```text
点云监听端口：9000
位姿监听端口：9001
航线目标：192.168.1.196
航线端口：9002
航线协议：香橙派 GOL1 逐点
```

注意：香橙派现有 ROS 节点只支持 `GOL1` 单目标点，不支持一次接收完整航线列表。因此地面站在该模式下会把当前航线按航点顺序拆成多个 `GOL1` 包发送；航点列表中的“发送”按钮也会发送单个 `GOL1` 目标点。远端会把每个包发布为一个 `/udp/goal_pose`。当前远端 `geometry_msgs/PoseStamped` 没有独立航点 ID 字段，`header.seq` 也更适合作为 ROS 发布序号看待，不能可靠表示“第几个航点”。如果后续需要飞控一次接收完整航线，建议在香橙派端新增 `RTE1` 或自定义多航点协议接收节点。

航点 ID 目前不是正式协议字段。不修改远端代码时，地面站可以发送单个航点，但不能保证 `/udp/goal_pose` 中携带稳定航点 ID。可协调的协议扩展方案见 [doc/waypoint_id_protocol_proposal.md](doc/waypoint_id_protocol_proposal.md)。

收到 `POS1` 位姿后，地面站会在 3D 点云场景中显示实时位姿 marker：青色球表示当前位置，黄色箭头表示水平航向，地面黄色圆点表示投影位置。左侧 UDP 面板仍会显示原始 `POS1` 序号和米制坐标，日志中会记录完整四元数。

## 静态点云模型

地面站支持上传并显示离线点云模型：

- `.lvx2`：支持加载累计地图，并支持时间轴播放。
- `.bag`：支持 ROS1 bag v2.0（本机构建含未压缩、LZ4、BZ2 chunk），读取 `sensor_msgs/PointCloud2` 点云和 `geometry_msgs/PoseStamped` 位姿，并支持录制时间轴同步播放。
- `.pcd`：支持 ascii 和 binary。
- `.ply`：支持 ascii 和 binary_little_endian。
- `.csv` / `.txt` / `.xyz`：支持常见 `x,y,z,intensity` 点列表。

“加载全部点”会尽量显示文件中的全部有效点，但会占用更多内存，旋转视角和选点可能变慢。大模型交互时程序会临时切换到抽稀预览，停止拖动后恢复完整点云。

## 航点规划逻辑

新版地图航点任务的上升上限已在本地改为0.5m/s，下降仍沿用现场`max_z_speed`（历史配置0.25m/s），水平速度与飞控AUTO.LAND参数保持。此调整尚未部署机载，需更新机载任务管理器和航点二进制后重新准备任务；现有version=4地面站无需为此重启或重新选点。配置兼容性、部署状态见[任务说明](doc/mission_controls.md)。

当前点云默认被视为障碍物地图，而不是可直接点击的航点集合。地面站会把鼠标短按位置投影到“航点高度”对应的水平平面上，生成一个自由空间候选航点，然后用“安全半径”和垂向安全范围检查附近是否存在点云障碍。

- 候选航点附近有障碍物：拒绝添加，并提示重新选择更空旷的位置。
- 上传完整航线前：逐段检查航线线段是否穿过障碍物安全半径，发现冲突会拒绝发送。
- 上传单个航点前：检查该航点邻域是否安全，通过后才发送。

这套逻辑适合初版室内航线交互验证。后续如果要接入真实飞控，仍建议在机载端保留二次碰撞检测和轨迹平滑，因为地面站只能基于当前显示点云做规划辅助。

## 日志文件

两个程序启动后会在 exe 同级目录创建日志。打包版本中路径通常是 `bin/logs`：

```text
bin/logs/customgeometry_yyyyMMdd_HHmmss.log
bin/logs/radarsimulator_yyyyMMdd_HHmmss.log
```

日志覆盖以下关键链路：

- 程序启动、QML 加载、日志路径。
- UDP 点云监听、分片解析、帧重组、丢帧、无效包。
- 点云几何刷新、点数、包围盒、选点命中结果。
- 静态点云文件加载、LVX2/ROS bag 解析、CSV/PLY/PCD 解析、异步加载结果。
- 录制文件时间轴播放、累计地图/交互预览切换、ROS bag 位姿 marker 更新。
- 航线新增、插入、拖动、清空、上传。
- 模拟器点云发送失败、航线回传解析失败。

用户现场反馈问题时，优先收集对应时间的 `.log` 文件。

Android 版本不会把日志写到 APK 安装目录，而是写到应用数据目录下的 `logs` 子目录。调试时可通过 Android Studio/Qt Creator 日志、`adb logcat`，或后续增加应用内“导出日志”按钮取回文件。

## 打包交付

在已配置 Qt/MSVC 的构建目录中执行：

```powershell
cmake --build build\codex-debug --config Debug --target package_runtime
```

打包输出目录：

```text
build/codex-debug/package
```

目录中会包含 `bin/customgeometry.exe`、`bin/radarsimulator.exe`、Qt 运行库、QML 模块、插件和 `README.md`。用户可直接进入 `package/bin` 双击运行 exe。

Release 构建同理：

```powershell
cmake -S . -B build\release -DCMAKE_PREFIX_PATH=C:/Qt/6.10.1/msvc2022_64 -DCMAKE_BUILD_TYPE=Release
cmake --build build\release --config Release --target package_runtime
```

## Android 构建与模拟器调试

Android 版本只打包地面站 `customgeometry`，不打包 `radarsimulator`。模拟器或真机上的地面站仍然可以接收局域网内 PC、香橙派或其他发送端发来的 UDP 点云；如果要用本机模拟器验证实时点云，请继续在 Windows 上运行 `radarsimulator.exe`，并把 UDP 目标 IP 配成 Android 设备或模拟器可访问的地址。

Qt Creator 中建议这样选：

- Android 模拟器：选择 `Qt 6.10.3 for Android x86_64` Kit，并创建 x86_64 AVD。
- Android 真机：通常选择 `Qt 6.10.3 for Android ARM64-v8a` Kit。

命令行构建 x86_64 模拟器包示例：

```powershell
$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.19.10-hotspot'
$env:ANDROID_SDK_ROOT='C:\Users\58420\AppData\Local\Android\Sdk'
$env:ANDROID_HOME=$env:ANDROID_SDK_ROOT
$env:ANDROID_NDK_ROOT='C:\Users\58420\AppData\Local\Android\Sdk\ndk\27.2.12479018'

& 'C:\Qt\6.10.3\android_x86_64\bin\qt-cmake.bat' -S . -B build\android-x86_64 -G Ninja -DANDROID_ABI=x86_64 -DCMAKE_BUILD_TYPE=Debug
cmake --build build\android-x86_64 --target customgeometry_make_apk
```

APK 输出位置通常是：

```text
build/android-x86_64/android-build/build/outputs/apk/debug/android-build-debug.apk
```

Android 适配点：

- `android/AndroidManifest.xml` 已声明 UDP 通信需要的 `INTERNET`、`ACCESS_NETWORK_STATE`、`ACCESS_WIFI_STATE` 权限。
- Android 文件选择器返回的 `content://` 点云文件会先复制到应用缓存目录，再交给现有 LVX2、ROS bag、CSV、PCD、PLY 解析器处理。
- Android 界面采用 3D 全屏 + 左上角“操作”按钮；点击后从底部弹出操作抽屉，可上传模型、配置 UDP、调整规划参数和上传航线。触控支持单指拖动旋转、双指捏合缩放、长按空旷区域生成航点。
- Android 日志写入应用可写数据目录，Windows 仍写入 exe 同级 `logs` 目录。
- Windows 路径加载逻辑保持不变；`package_runtime` 仍然只服务 Windows 交付目录。

## 接入真实 Livox MID-360/MID-360S

建议不要直接改 UI，而是在发送端新增一个“真实雷达适配器”，让它输出与模拟器相同的 UDP 协议：

1. 使用 Livox SDK2 订阅点云数据。
2. 将 SDK 点格式转换为 `PointCloudPoint`：
   - `position.x/y/z`：米制坐标。
   - `intensity`：归一化到 `0.0 ~ 1.0`。
3. 按 `PointCloudProtocol::MaxPointsPerDatagram` 拆分点云帧。
4. 调用 `PointCloudProtocol::encodePointCloudChunk()` 编码并发送到地面站点云端口。
5. 如需接收航线，监听地面站航线端口并调用 `PointCloudProtocol::decodeRoute()`。

这样地面站无需区分“模拟器点云”和“真实雷达点云”。后续如果需要直接读取 Livox 原始包，也建议先封装成独立类，不要把 SDK 回调写进 QML 或渲染类。

如果使用香橙派上的 ROS 链路，也可以沿用 `something1017/src/udp_uav_tx/src/udp_uav_tx_node.cpp` 的方式：从 Livox SDK2 或 ROS 话题拿到 `PointCloud2` 后，发送 `PCD1/LZ4` 压缩分片到地面站 `9000`；地面站无需额外安装 LZ4 运行库，已经内置了 LZ4 block 解压兼容代码。

## 未来集成 QGC

如果后续把点云地面站能力集成到 QGroundControl，建议采用 QGC custom build：点云继续走独立高吞吐数据流，例如当前 `PCD1/raw`、`PCD1/LZ4` UDP 分片，或后续 ROS2/DDS/WebSocket；MAVLink 只承载位姿、任务、航点、ACK、状态和安全确认等低频飞控语义。

不建议把完整实时点云直接封装进 MAVLink 自定义消息或 `TUNNEL`。MID-360 级别点云吞吐较高，强行走 MAVLink 会挤占遥测链路并拖慢 QGC 的 MAVLink 解析、日志和转发链路。当前 UDP 点云协议未来可以作为 QGC 点云模块的数据流输入，航线/航点进入真实飞控控制链路前再迁移到 MAVLink Mission Protocol 或双方确认的 custom dialect。

详细建议见 [doc/qgc_integration_notes.md](doc/qgc_integration_notes.md)。

## 常见排查

- 地面站没有点云：检查地面站点云端口和模拟器 `--point-port` 是否一致，并查看 `customgeometry_*.log` 是否有 UDP 绑定失败或无效包。
- 香橙派没有点云：确认左侧已点击“套用香橙派 UDP”，并确认香橙派端 `uav.launch` 中 `pc_ip` 是当前 Windows 主机 IP，而不是默认示例里的旧地址。
- 航线上传后模拟器无输出：检查地面站航线目标和模拟器 `--route-port` 是否一致，并查看 `radarsimulator_*.log`。
- 香橙派收不到航点：确认航线协议为“香橙派 GOL1 逐点”，航线目标为 `192.168.1.196:9002`，并查看香橙派 ROS 节点是否发布 `/udp/goal_pose`。
- `rostopic echo /udp/goal_pose` 不显示航点 ID：这是当前远端 `GOL1 -> PoseStamped` 协议限制；需要双方协调新增 `GOL2` 或 `RTE1` 后才能可靠携带 ID。
- 加载大文件后旋转卡顿：关闭“加载全部点”或降低显示上限；全量点云适合最终检查，抽稀预览适合交互编辑。
- `.lvx2` 加载失败：当前解析器覆盖常见 Livox Viewer 导出的 LVX2 录制文件；如果遇到新版本字段变化，请保留文件和日志，用日志中的 LVX2 头信息定位兼容点。
- `.bag` 加载失败：当前解析器支持ROS1 bag v2.0；若提示不支持LZ4/BZ2压缩，请检查构建是否找到对应可选库。历史飞行查看入口会筛选FAST-LIO世界点云与里程计；仅原始雷达或其他坐标系的话题不会直接拼成世界地图。


## 本机 FAST-LIO 单向点云接入（2026-10-26）

已在 Ubuntu 20.04 / ROS Noetic / Qt 6.8.3 上验证本机收到香橙派真实点云。

本次工作目录入口：

```bash
bash /home/tyzk/dianyunGCS-review-20261026/connect-live-pointcloud.sh
bash /home/tyzk/dianyunGCS-review-20261026/stop-live-pointcloud.sh
```

第一条连接并打开实时窗口；已有地面站监听同一端口时复用现有窗口。第二条只停止本工具的点云发送会话。
通过连接脚本新开的窗口关闭后也会停止该会话；复用原窗口时请使用停止脚本。

链路为 `/factory_fastlio/cloud_world` → `tools/pointcloud-only/cloud_sender.py` → 既有 PCD1/LZ4 → 上位机 UDP 9000。
机载地址默认192.168.1.235；上位机地址从到机载的现有路由自动读取，本轮为192.168.1.125。不修改网络配置或防火墙。
FAST-LIO 须已通过原管理程序启动；桥接不启动或重启定位。

发送桥只订阅点云及可选的同会话位姿，不发布ROS话题、不监听目标端口，也不连接飞控服务。其源码由本机管理器通过SSH标准输入运行，
不替换机载现有源码或安装常驻服务。连接状态和发送日志位于本次工作目录 `.pointcloud-link/`。
SSH 断线后，本机转发管理进程会退避重连；机载同一目的地使用锁防止旧连接未退出时重复发送。停止入口会停止重连。
只读状态命令：

```bash
python3 /home/tyzk/dianyunGCS-review-20261026/dianyunGCS-master/tools/pointcloud-only/manage.py status
```

点云显示遥测默认最多2Hz、4000点/帧，超限仅对显示数据等间隔抽样。原FAST-LIO话题频率、地图和规划参数不变。
点云必须是 `camera_init/world` 世界坐标，数值按米原样发送；渲染继续使用原地面站的米制到Qt场景变换。
手动配置地面站时，设置点云端口9000并“恢复实时点云”。“航线目标”地址与点云接收无关。
命令行新增 `--cloud-port 9000` 可启动实时监听；不加参数仍保持原有56000端口和离线文件功能。
旧程序进程可读取新发送桥；重启新版程序后，连续2秒无完整新帧会在HUD显示“点云断流（末帧）”。

该通道只验证实时点云显示，不表示飞行任务、目标确认、轨迹规划或执行链路已接入。
累计显示仍受原帧窗口和点数上限控制，不等于永久保存或可重定位的地图。
真实数据验证：3帧完整分片重组、原Qt解码器解压与有限坐标检查、ROS发布列表为空、停止后无遗留发送节点。
详细验证文件位于本次工作目录 `.pointcloud-link/verification/`。


## 同会话 FAST-LIO 位姿显示（2026-10-26）

当前连接脚本同时订阅 `/factory_fastlio/cloud_world` 和 `/factory_fastlio/odometry`，保持机载→地面站单向遥测。
UDP9000仍为PCD1/LZ4点云，UDP9001使用原44字节POS1位置/四元数。直接使用Odometry，不向ROS另发布PoseStamped。
两者必须同为camera_init（或同为world），里程计子帧必须是body；不混用MAVROS local_position的原点。
位置单位米、保留原始四元数和源时间；没有旋转、偏移、航向归零或ENU/NED转换。

启动/复用连接：`bash /home/tyzk/dianyunGCS-review-20261026/connect-live-pointcloud.sh`。
独立升级既有点云连接：`python3 tools/pointcloud-only/manage.py start --with-pose`。
停止入口同时停止本工具的点云/位姿发送，不停止FAST-LIO，也不操作飞控。
不加--with-pose时仍可新建只传点云的会话；已有带位姿会话不会被普通start意外降级。

青色球显示定位参考点的位置，黄色箭头显示body的水平+X前向，地面圆点是投影。该前向需与实际机头安装轴核对，不能把雷达到IMU的标定误当成机体机头标定。
航向是相对本次定位世界+X的角度，不是已验证的真北航向。原三色绘图轴仍固定在地图原点。
新版实时启动自动启用9001位姿接收，HUD显示XYZ和定位前向。POS1超过1.5秒无新源位姿时隐藏旧标记；重复/乱序时间戳不刷新标记。
用户说明航点是误操作后，已清空临时航点并重开新版实时窗口；数字HUD和位姿断流隐藏已生效。本次未载入旧离线样本。

验证包括：Python POS1与Qt原解码接口兼容、90° ROS航向与Qt箭头方向一致、非法坐标/四元数拒绝、断流隐藏、旧包拒绝与新包恢复；真实ROS源和Qt日志已核对。
这只是位置/姿态显示接入，不包括目标、规划轨迹或飞控执行接入。桥接节点仍没有ROS发布话题或飞控服务调用。

### EGO-Planner地面站接入（2026-10-26）

实时启动入口已集成右上角11行飞前话题表、原生局部膨胀栅格、同会话等高地图选点、精确目标预览/人工确认，以及规划/实测路径显示。启动只开预览；确认后复用现有PX4接入，停止复用执行器受控保持。不自动解锁、起飞或切模式。已连接机载只读核对定位、规划和栅格数据；本轮未做飞行验收。完整入口、操作与边界见 [EGO-Planner接入说明](doc/ego_planner_groundstation.md)。

2026-10-27：面板标题为“Ego-Planner 避障”，移除目标高度调节，选点时自动锁定当前FAST-LIO世界Z。已选目标不跟随飞机移动，原预览/确认流程不变。RViz原生2D Nav Goal则在固定帧Z=0平面选XY和yaw，并不自动等高；区别与当前手势边界见上述接入说明。

2026-10-27停止流程修正：停止按钮先请求现有执行器受控停止，确认退出且飞控非OFFBOARD后，再结束原规划预览会话。界面立即清除预览线并显示真实等待/成功/失败；坐标轴与实测历史保留，停止后需重新启动预览再选点。

静态预览启动成功后，以不抢焦点的提示弹窗显示“静态预览已运行。”，3秒后自动消失，不再常驻左侧。失败原因仍留在面板；PID、路径与原始管理返回只记录日志。

桌面视图操作：鼠标中键按住拖动可平移观察中心，左键拖空白或右键拖动旋转，滚轮缩放；“点云与视图”中的“复位视角”恢复默认观察中心、角度和缩放。这些操作只改变相机，不改变点云、定位、目标或飞机状态。

### 手动飞行数据录制

左侧“飞行数据录制”支持开始、停止及HH:MM:SS数字时长，复用机载rosbag并保存在flight-records。另有默认关闭的自动录制选项，启用时从OFFBOARD开始，到落地且锁桨结束；详细操作、目录与验证见 [录制说明](doc/manual_recording.md) 和 [飞行链路分析](doc/flight_chain_analysis.md)。

界面精简：移除常驻视角教程、定位颜色图例、正常等高/栅格统计、目标预览重复坐标提示；保留位置、目标、规划/执行状态与异常原因。

实时模式左侧现已提供 **FAST-LIO 定位与转发** 的启动、停止和实时状态，复用机载现有管理入口；见 [定位与转发操作说明](doc/localization_controls.md)。

地图选点入口检查、飞前监视UDP稳定性修复和可选自动录制见 [飞行链路分析与修复](doc/flight_chain_analysis.md)。

2026-10-27栅格与低频监视：原生膨胀栅格经独立单向显示流压缩转发，空帧、真实断流和上游定位/点云过期分别提示；同一规划会话校验与1.5秒栅格有效期保留。电池频率采用12秒窗口，样本不足显示“—”，电池数据的5秒过期规则不变。栅格只供显示，不修改机载规划地图。

同一次OFFBOARD连续选点：上一目标由原规划器完成、实测到达并稳定悬停后，可选下一目标并查看独立绿色候选预览；核对后人工确认，由原规划器重新生成实飞轨迹。候选计算与当前执行隔离，确认前复查最新地图，遥控接管及原有失效处理保留。当前实现不支持在上一目标尚未完成时切换目标；具体条件与验收边界见 [接入说明](doc/ego_planner_groundstation.md)。

### 双目与云台视频监视

桌面右侧的视频面板可手动连接现有 Orbbec 彩色 ROS 话题和机载云台 RTSP 中继，云台 H.264 由地面站本地解码；面板已移除“人物识别”勾选项，普通拉流显示原画；人物跟踪任务运行时自动切换到该任务的识别画面，无新帧时显示不可用。依赖、识别节点的只监视管理方式、实测情况和操作限制见 [视频监视说明](doc/video_monitor.md)。

点云默认使用鲜艳彩虹高度伪彩色，可在“点云与视图”取消“鲜艳彩虹高度色”恢复原配色。颜色按当前点集的最低至最高 Z 值归一化，实时点集变化时颜色可能变化；它不代表障碍类别。PCD1/LZ4 实时链路只传 XYZ，不能还原反射强度颜色。

在“点云与视图”中可启用“隐藏高处点云”，调整“显示高度上限 (m)”；默认上限是点云坐标系 Z=2.50 m，并非自动识别天花板，需根据定位原点与现场高度调整。高于上限的点只从画面隐藏，原始点集和地面站障碍查询不受影响。地面参考网格默认隐藏，可用“显示地面参考网格”恢复；规划器的膨胀栅格另由原有开关控制。显示点数反映过滤后的主点云数量。


### 机载通信（桌面）

左侧“机载通信”可以设置机载 IP 并点击“应用并连接”。默认地址为 `192.168.1.235`，SSH 用户为 `orangepi`。密码框留空沿用已有密钥；输入密码时仅用于当前会话，不写入日志、命令行参数或磁盘配置。程序没有内置机载密码。密码会在提交后从输入框清除。

先验证 SSH，认证成功后才切换现有点云/位姿、飞前话题桥接，定位、规划、录制、视频共用该地址。更换地址会清除旧地图显示和选点预览；不会启动定位、规划或飞控执行器。飞控报告已解锁或相关操作进行中时，界面暂不允许应用新连接。SSH 连通仅表示可登录，点云和话题仍需相应机载源正常发布。

UDP 端口和监听操作已移至“高级设置 → 通信”。更改端口后，点击“机载通信 → 应用并连接”，使发送端和接收端使用相同端口。IP 和端口会保存供 Linux 实时启动入口复用，密码不会保存。视频需在视频面板手动重新连接。

2026-10-28：OFFBOARD 连续选点移除“上一目标到达并稳定悬停”的前置条件；候选按当前参考P/V/A预览，人工确认后进入原生局部重规划。详见 [EGO-Planner 链路说明](doc/ego_planner_groundstation.md#同一次-offboard-中的下一目标)。机载部署状态以实际部署记录为准。

2026-10-28：恢复局部地图外远目标选点和人工确认，沿用原生滚动地图局部避障；`OUTSIDE_LOCAL_MAP`作为终点尚未验证的提示，不再作为拒绝选点的条件。详见 `doc/ego_planner_groundstation.md`。

连续选点修复：新候选替换旧目标显示与到达提示，确认令牌绑定当前候选，机载确认后立即重置目标状态。详见 `doc/ego_planner_groundstation.md`；机载生效状态以实际部署记录为准。

### 机载飞行记录与三维回放

在“飞行数据录制”中点击“读取机载飞行数据”，打开独立查看窗口。刷新机载列表，选择一轮已结束的飞行及其中一个分包，点击“读取并绘图”；程序复用“机载通信”的SSH地址和会话凭据，只下载该 `.bag` 分包，后台解析世界点云与FAST-LIO轨迹。录制中的记录暂不列入，原始录包不会被删除。也可通过“打开本地录包”选择一个文件。

查看窗口以累计点云为固定背景，时间轴控制橙色已飞实测轨迹与青色位置标记；支持播放、旋转/平移/缩放，以及“聚焦轨迹 / 完整点云 / 隐藏顶部”。历史数据显示独立于实时地图，不会上传航点或操作飞控。机载根目录是 `~/drone_platform/01_apps/ego_planner/flight-records`；下载缓存放在Qt应用数据目录的 `flight-records/<机载地址>/<记录名>/`。

本机已支持未压缩、LZ4和BZ2的ROS1 bag。其他平台构建压缩支持时，需要CMake可找到 `lz4frame.h`/`liblz4`及BZip2；缺少库时会明确提示不支持相应压缩，原有未压缩文件功能保留。具体支持范围与验证方式见 [飞行记录查看](doc/flight_records_viewer.md)。

飞行记录查看现默认一次读取一个分包，可在记录旁选择分包；点云保持累计，拖动时间轴改变当前已飞轨迹与位置。右侧“飞前话题监视”改名为“ROS话题栏”，与视频面板统一标题、宽度及折叠样式。

### 桌面图标与加载动画（Linux）

当前工作区已安装桌面“科研飞行平台-地面站”图标，双击即可启动，也可在应用菜单搜索同名程序。图标使用新生成的无人机点云图案；启动器调用现有 `start-groundstation.sh --live` 并沿用保存的通信端口，不会自动启动机载定位、规划或飞控接入。程序已运行时会唤起现有窗口，避免重复监听端口。

移动工程后可在新位置执行 `./install-desktop-launcher.sh` 重新生成桌面和应用菜单入口。冷启动会显示新图标、旋转点环和加载状态，主界面首帧就绪后淡出；不会等待机载电脑上线。为避免快速闪烁，启动页最短显示约650ms，再淡出180ms，不显示虚构的加载百分比。可使用 `--no-splash` 跳过启动页以便调试。启动或初始化失败会在启动页显示错误，详细日志仍写入原日志目录。

### 实时累计建图与传感器数据（2026-10-29）

桌面右侧“视频流拉取”下新增可折叠“传感器数据”栏，显示飞控 IMU、雷达 IMU 的 XYZ 加速度与角速度、雷达频率/单帧点数、气压。沿用机载只读监视通道，不发送运动指令。没有消息或源采样过期时显示“—/未采样”。雷达加速度保留驱动原始值，不冒充 SI 单位。

机载实时模式默认启用“累计建图显示”：对已有 `/factory_fastlio/cloud_world` 世界坐标点云按 0.1 米体素累计，保留已扫描区域，默认最多 22 万点。达到上限后仍更新已有体素，但不继续加入新体素；可在“点云与视图”调整上限或清空。定位会话/机载连接变化会清空旧累计图。累计显示只作用于画面，不替代 FAST-LIO 建图、不改变避障栅格；移动物体的历史点可能保留。雷达自身坐标系的原始点云不能直接用于持久累计。

栅格显示遇到短时延迟时保留同会话最近一帧至采样年龄3秒，并显示“栅格延迟”，避免在1.5秒临界点整层闪灭。实时有效期限仍为1.5秒，旧帧不用于控制或确认；超过3秒、输入故障、会话变化、空栅格或监视断流时隐藏。

视频无图像排查：云台由地面站 GStreamer 直接连接机载 RTSP 中继并解码 H.264，需确认机载 8554 端口可达且地面站安装 GStreamer 相关插件。双目必须有 `/camera/color/image_raw/compressed` 实际消息；仅相机USB枚举或话题存在不代表出图。若机载日志报 `Device response with bad magic`、设备断开并以exit code 11退出，应先恢复相机设备/驱动通信，地面站无法从无消息的话题生成画面。SSH视频命令使用exec，桥接还监视父进程退出，避免无帧时切换视频遗留旧订阅。

视频连接恢复：双目 SSH 或云台 RTSP 接收退出后均按2～15秒退避重连；进程存活却持续20秒无可解码帧时重建只读接收（人物识别启动等待45秒）。停止会取消两路重试，不会重启相机、ROS或飞控服务。SSH连接超时/No route to host仍需恢复机载网络，自动重连不代表视频源已恢复。

航点任务、ArUco精准降落和人物跟踪功能栏已接入机载任务管理入口，复用现有SSH凭据。航点支持地图连续选点、逐点高度/拍照、固定停留2秒/朝下90度；地面锁桨时准备任务，遥控切OFFBOARD后自动解锁并在起点垂直起飞至首点高度，再依次执行航点，末点完成后返回起飞位置并自动降落；精准降落新版准备时先将云台朝下，切OFFBOARD后才执行降落。自动起飞变更已于2026-10-30部署机载并通过原生解析与隔离状态机测试；旧地面站请求会被拒绝，需要重启新版并重新选点。状态、停止语义、部署与验证范围见[三项任务控制说明](doc/mission_controls.md)。现阶段完成软件接入及隔离验证，未完成三项功能的实飞验收。

### 机载文件名调整（2026-10-30）

仅去掉部分文件名中的diff前缀，目录、ROS包/节点/话题和算法逻辑保持。对应include、CMake、launch可执行文件名与脚本路径已同步，详见[机载文件更名](doc/onboard_file_names.md)。

2026-10-30 ArUco精准降落候选修复与取证方法见[任务控制说明](doc/mission_controls.md)及[机载源码说明](tools/missions/onboard/README.md)。本地回归不等于实机验收；当前机载状态未取得，ArUco候选源码尚未部署，真实标记、标定、安装模型与实飞仍待验证。

2026-10-30后续：已补取机载只读快照与11:55真实飞行数据，隔离ROS准备/完整集成测试通过；分析与限制见[11:55飞行报告](analysis/aruco-flight-20261030/report.md)。候选源码仍未机载部署，未改P或飞控参数。

2026-10-30 14:31：按用户要求将机载视觉位置外环`xy_kp`从0.45降到0.30，其他增益不变，下次准备时加载；已备份和核验，未启动真实任务。见[P调整记录](analysis/aruco-p-reduction-20261030/report.md)。

2026-10-30定位转发时间戳修复已机载部署：保留FAST-LIO源戳，每个新测量最多转发一次，停止重复刷新旧位姿；本地/ARM隔离测试和地面回读通过。见[定位与转发说明](doc/localization_controls.md)。本次重启了定位会话，原点重新建立；尚未完成修复后实飞验收。

## Ubuntu 用户安装包（2026-11-04）

当前提供 Ubuntu 20.04 amd64 的 `.run` 自解压安装包，随包携带 Qt/QML 运行库、通信助手、图标与桌面入口。安装到当前用户目录，无需安装 Qt 开发环境。构建方法、依赖、安装和卸载见 [Linux 安装包说明](doc/linux_installer.md)。本轮仅验证 Linux 包，不代表已经生成 Windows 安装程序。

### Windows 安装包（2026-11-09）

Windows 10/11 64位安装版与便携版包含Qt运行库、SSH客户端、会话密码助手、Python遥测工具和视频解码器，不附带完整工程源码。SSH指纹校验保留；用法、依赖与验证范围见 [Windows安装包说明](doc/windows_installer.md)。

### Windows11 WSL一键安装（2026-11-10）

提供包含Linux地面站安装文件的WSL一键包。完整解压后双击“一键安装.cmd”，配置WSL2镜像网络、UDP9000/9001/9005、图形通信依赖和Windows桌面入口。保留原配置及旧版本，不自动重启Windows；只有系统明确要求重启时提示，重启后再次运行继续。步骤与验证范围见 [WSL一键安装说明](doc/wsl_oneclick.md)。

WSL含Ubuntu版已内置微软运行时与Ubuntu官方镜像，跳过GitHub发行版清单下载，并修复原生WSL输出乱码；Ubuntu软件依赖仍需联网。不清理已有发行版，详情见上述WSL安装说明。

WSL安装脚本2026.11.10.2进一步修复原生命令参数过度加引号导致的“没有已安装的分发”，已有含Ubuntu包可覆盖两份PowerShell补丁后继续。

WSL安装脚本2026.11.10.3已增加依赖下载有限重试、连接超时及失败缓存保留提示，累计小补丁覆盖三份文件后可续装。

WSL安装脚本2026.11.10.4改为复用已正确配置的防火墙规则，仅在缺失或属性不符时处理自身规则；权限错误保留具体步骤，累计补丁包含此前全部修复。

WSL安装脚本2026.11.10.5根据实机HostFirewallLocal证据修复两层防火墙规则同名冲突，使用独立Hyper-V规则名并校验本地来源，累计补丁含此前修复。

WSL监视停留在“正在连接机载只读监视”时，可使用独立只读诊断工具一次采集本地桥接日志及端口状态，使用方法见WSL安装说明。

WSL完整包1.0.20261110.1已将现场验证的lo回环发送和固定后台askpass修复合入正式Python源码，并升级运行文件复用校验。安装脚本2026.11.10.6保留全部此前修复，旧版本升级先关闭旧界面，保留旧目录与数据。每次启动后仍需在机载通信输入密码并应用连接。

WSL完整包1.0.20261110.2继续合入现场云台视频修复：仅对当前机载RTSP中继追加no_proxy/NO_PROXY；启动器读取已保存地址。正式主程序补齐--host参数及视频/通信地址统一初始化，未指定时读取当前用户QSettings，避免换版本目录后视频仍回落到默认IP。两份脚本保留可执行权限，不保存密码或修改系统代理。详见[WSL安装说明](doc/wsl_oneclick.md)。

WSL完整包1.0.20261110.3增加一键卸载.cmd；另提供兼容旧版的独立卸载包。卸载只处理当前用户登记的地面站目录、快捷方式和专用防火墙规则，先备份目录内新增文件，保留WSL、Ubuntu、用户配置及飞行数据。详见[WSL安装说明](doc/wsl_oneclick.md)。

WSL工具1.0.20261111.1修复卸载入口FunctionsOnly参数被dot-source安装脚本覆盖的问题，增加正常入口回归及依赖导入前日志。CMD在调用PowerShell前另写uninstall-launch日志并显示退出码；独立卸载工具兼容之前安装的版本。

ROS2录制与回放适配见 [手动录制](doc/manual_recording.md) 和 [飞行记录查看器](doc/flight_records_viewer.md)。Linux构建安装libsqlite3-dev以启用`.db3`读取；原有ROS1 `.bag`读取保留。当前仍是独立迁移副本，物理整链待验收。

ROS2独立迁移副本：录制入口使用机载`env.sh`，飞行记录支持ROS1 `.bag`及ROS2 sqlite3/CDR `.db3`；Linux构建使用可选SQLite3开发库。详见`doc/manual_recording.md`与`doc/flight_records_viewer.md`。物理整链仍待验证，未替换原桌面入口。


### 2026-11-17 ROS2飞前元数据采样

当前.224基础服务已启用并通过实际重启验证（雷达/IMU、Gemini、MAVROS、云台视频；定位/转发/任务仍按需）。飞前采样仅对Livox CustomMsg使用ROS2 raw订阅，校验CDR1封装、端序、字符串、纳秒、点数及整帧长度后提取源戳、坐标系和点数；其余消息继续类型化订阅，地图仍使用FAST-LIO世界点云。避免Python逐点构造拖慢其他传感器，不更改传感器时间或新鲜度门槛。

实际早期ROS2窗口domain175已对齐0；原约6.2Hz积压已修复。现场消息与官方序列化/解码对照41项、快照并发取时回归及现有14个本地测试通过；最终180个快照/35.9秒持续观察与独立canonical实流检查16项通过，雷达约10Hz、Livox IMU200Hz、飞控IMU50Hz/里程计30Hz。原先的积压和状态闪红失败均保留。详见迁移仓库`records/2026-11-17/base-autostart/`。

基础服务通过不等于整平台验收完成；全负载定位自同步、EGO输入时效、云台灰度帧交付、运动外参及实际任务仍有待解决。正式桌面启动入口尚未切换。
