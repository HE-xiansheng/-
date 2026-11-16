# 室内点云航线规划 Demo 说明

## 目标

本 demo 用于验证室内点云地面站的第一版交互闭环：

1. `radarsimulator` 模拟 Livox MID-360/MID-360S 风格的室内点云，通过 UDP 发送给地面站。
2. `customgeometry` 作为地面站 GUI 接收点云并显示。
3. 用户在点云障碍地图的空旷区域生成两个自由空间航点。
4. 用户可拖动航点或航线线段插入避障折点。
5. 用户确认航线后点击“上传航线”，地面站才将完整航线坐标通过 UDP 回传给模拟器。
6. 用户也可以上传离线建图得到的静态点云模型，在静态模型上规划并上传航线。

## 构建

推荐使用 CMake：

```powershell
cmake -S . -B build/codex-debug -G Ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.10.1/msvc2022_64 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/codex-debug --config Debug --target all
```

构建完成后会生成：

- `build/codex-debug/customgeometry.exe`：点云地面站 GUI。
- `build/codex-debug/radarsimulator.exe`：雷达点云模拟器。

如果从普通终端运行，请确保 Qt 的 `bin` 目录在 `PATH` 中：

```powershell
$env:Path = "C:\Qt\6.10.1\msvc2022_64\bin;" + $env:Path
```

## 运行

先启动模拟器：

```powershell
build/codex-debug/radarsimulator.exe
```

再启动地面站：

```powershell
build/codex-debug/customgeometry.exe
```

默认端口：

- 点云：模拟器发送到 `127.0.0.1:56000`，地面站监听 `56000`。
- 航线：地面站发送到 `127.0.0.1:56001`，模拟器监听 `56001`。

完整 UDP 线格式见 [udp_protocol.md](udp_protocol.md)；航点 ID 等未落地扩展建议见 [waypoint_id_protocol_proposal.md](waypoint_id_protocol_proposal.md)。

## 实时点云操作

- 左键短按点云中的空旷区域：按当前航点高度生成起点或终点。
- 生成航点前会按安全半径检查附近是否有点云障碍，距离过近时拒绝添加。
- 左键拖动黄色/蓝色/粉色航点：移动航点。
- 左键拖动航线线段：插入一个中间避障航点并拖动。
- 左键拖动空白区域，或右键拖动：旋转相机。
- 中键按住拖动：平移视角；“点云与视图”中的“复位视角”恢复默认位置、角度与缩放。
- 鼠标滚轮：缩放视图。
- 点击“上传航线”：通过 UDP `RTE1` 发送完整航线快照。
- 左侧面板显示通信、点云、静态模型和航线状态；视角缩放直接使用鼠标滚轮。

## 静态点云模型

点击“上传点云模型”可加载已经建好的室内点云地图。加载后地面站会暂停实时 UDP 点云刷新，当前画面保持为静态模型；航线选点、拖动避障和“上传航线”的操作与实时点云完全一致。

当前支持格式：

- `.lvx2`：支持 Livox Viewer 2/Livox 官方样例中常见的笛卡尔点云包，按官方 LVX2 规格读取 `Data Type 1` 和 `Data Type 2`。
- `.bag`：支持 ROS1 bag v2.0 未压缩 chunk；构建找到可选LZ4/BZip2库时也支持对应压缩chunk，自动识别 `sensor_msgs/PointCloud2` 点云话题和 `geometry_msgs/PoseStamped` 位姿话题，并保留录制时间轴，适合直接验证香橙派 ROS 工程循环播放的 bag。
- `.pcd`：支持 PCL 常用的 `DATA ascii` 和 `DATA binary`。`binary_compressed` 需要额外解压，当前 demo 会明确提示暂不支持。
- `.ply`：支持 `ascii` 和 `binary_little_endian` 顶点点云。
- `.xyz` / `.txt` / `.csv`：支持简单 `x y z [intensity]`、`x y z r g b`，也支持 Livox Viewer 2 转出的带表头 CSV，会自动读取 `X/Y/Z/Reflectivity` 列。

Livox 官方 `Indoor_sampledata.lvx2` 可以直接上传，也可以转成 CSV 后上传；转出的 CSV 可能超过 1GB。香橙派 ROS 工程里的 `PCLPOSE.bag` 也可以直接上传，地面站会读取其中的 `/occupancy_map/voxel_map` 这类 `PointCloud2` 点云话题，以及 `/mavros/local_position/pose` 这类 `PoseStamped` 位姿话题。地面站会在后台读取 LVX2/BAG/CSV 这类大文件，界面不会被文件读取阻塞。LVX2 和 ROS bag 加载都使用 `QtConcurrent` 后台任务：先快速扫描帧/包索引，再读取包时间戳并按有效点抽样解码，通常比读取 Viewer 转出的 CSV 更快。录制文件加载完成后会保留一份累计静态地图用于航线规划，同时显示“录制时间”播放条，可按录制时间播放或拖动查看局部扫描；如果 bag 内有位姿话题，3D 场景中的飞行器/雷达 marker 会随播放条同步移动。为了保证首版交互流畅，默认最多显示 500000 个累计地图抽样点，左侧状态会显示“可用点 / 文件点记录”；LVX2 文件点记录中可能包含 `0,0,0` 填充点，加载器会过滤掉这些无效点，所以“加载全部点”时可用点数可能小于文件点记录数。如果需要查看完整可用点云，可在上传前勾选“加载全部点”；全量模式会占用更多内存。

为了避免全量点云旋转视角时卡顿，静态模型加载完成后会额外生成一份约 800000 点的交互预览缓存。拖动视角、滚轮缩放或拖动航线时自动切到预览点云，停止操作后恢复完整累计地图。该优化对 LVX2、ROS bag、CSV/TXT/XYZ、PCD、PLY 这些静态模型都生效；实时 UDP 点云仍按实时帧显示，不使用静态模型预览缓存。

静态模型坐标同样按雷达/机器人米制坐标解释：`X` 前、`Y` 左、`Z` 上。若第三方建图软件导出的坐标轴不同，建议在导出前或加载器前置适配层中转换坐标轴。

可用于联调的公开样例来源：

- Open3D 的数据集教程包含 `PCDPointCloud`、`PLYPointCloud`、`LivingRoomPointClouds` 等点云样例。
- Livox 官方下载页提供 `LVX2 Specifications` 和 MID-360 室内/室外 `.lvx2` 样例。
- PCL 官方 PCD 文档说明了 PCD 文件头、字段和 `DATA ascii/binary/binary_compressed` 三种数据段。
- PLY 格式说明可参考 Greg Turk / Paul Bourke 的 PLY 文档。

## 模拟器点云逻辑

模拟器没有使用纯随机散点，而是做了一个“室内建图结果 + 少量实时射线命中”的混合模型：

- 水平视场为 360 度。
- 垂直视场按 MID-360 官方参数近似为 `-7° ~ 52°`。
- 默认场景是多房间室内地图：左侧大开间、中部走廊、右侧房间、下方设备间、斜向隔断、弧形围挡、立柱、桌柜和矮箱障碍物。
- 少量射线仍按 MID-360 的 360° 水平视场和约 59° 垂直视场做命中，用于保留“实时雷达”的动态感。
- 大部分点每帧从同一个室内地图表面抽样，让地面站在“最新帧显示”模式下也能看到类似建图软件的完整点云模型。
- 地面、墙面、障碍物使用不同反射强度，并加入约 2cm 量级的距离噪声。

默认仍按 MID-360 官方最高点率运行，但为了让单帧更接近参考图里的累计地图效果，使用 `5Hz × 40000 点/帧 = 200000 points/s`。如果需要模拟更原始的 10Hz 实时流，可手动指定：

```powershell
build/codex-debug/radarsimulator.exe --hz 10 --points 20000
```

地面站实时 UDP 模式默认只显示最新一帧，因此默认模拟器运行时，左侧“单帧点数”和“显示点数”都会约为 40000。需要接近建图软件里的累计地图观感时，可手动勾选“实时累计显示”，并自定义累计帧数和累计点数上限；例如保留 5 帧、上限约 22 万点时，显示点数会在几帧后稳定到约 200000。

## 后续可扩展点

- 接真实雷达时，可把 `radarsimulator` 替换为 Livox SDK2 接收程序，并按 `PCD1` 协议转发点云。
- 真实 Livox 接入步骤见 `doc/real_livox_integration.md`。
- 航线控制链路进入真实设备前，应在 `RTE1` 基础上增加 ACK、重传、CRC、任务 ID 和安全确认。
- 当前点云拾取是 CPU 最近射线点搜索，万级点交互足够；更大点云可加入体素降采样或 KD-tree。
