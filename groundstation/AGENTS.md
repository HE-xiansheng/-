# AGENTS.md

本文件用于给后续进入本仓库的 Codex/AI 代理提供项目上下文、开发约束和交付边界。正式用户说明仍以 `README.md` 和 `doc/` 目录为准；接到新需求后，应先阅读本文件，再按需求补读相关文档和源码。

## 项目定位

本项目是“室内点云航线规划地面站”的 Demo/原型，核心目标是验证室内点云显示、静态点云模型查看、自由空间航点规划、UDP/ROS 联调和航线/航点下发流程。

当前不是完整物流地面站，也不是完整 PX4/QGroundControl 替代品。MAVLink、QGC、PX4 Mission、配送任务、多机管理、电池/GPS/飞控模式等属于未来集成方向，不能在文档或回复中表述为已经实现。

## 主要程序

- `customgeometry`：点云地面站，负责显示实时/静态点云、规划航点、拖拽航线避障、显示位姿、发送完整航线或单个航点。
- `radarsimulator`：桌面侧 Livox MID-360 风格点云模拟器，通过 UDP 发送室内点云，并监听地面站回传航线。
- `something1017/`：从香橙派复制来的 ROS1/catkin 示例工程，用于点云、位姿和目标点 UDP 桥接联调。除非用户明确要求，不要擅自修改远端香橙派代码；本地文档和地面站应尽量以远端现有协议为准。

## 进入项目后优先阅读

1. `README.md`：运行、打包、Android、Livox、QGC 集成和常见问题总入口。
2. `doc/product_function_parameters.md`：当前产品功能参数、已实现能力和未实现边界。
3. `doc/udp_protocol.md`：当前已经落地的 UDP 协议。
4. `doc/waypoint_id_protocol_proposal.md`：航点 ID、`GOL2`、`RTE1` 等未落地协议提案。
5. `doc/real_livox_integration.md`：真实 Livox MID-360/MID-360S 接入建议。
6. `doc/qgc_integration_notes.md`：未来集成 QGC/MAVLink 时的数据链路分工。
7. `doc/pointcloud_route_demo.md`：早期 Demo 操作和实现说明。

## 当前协议边界

当前已经实现或兼容的协议：

- `PCD1/raw`：本机模拟器未压缩点云分片。
- `PCD1/LZ4`：香橙派 ROS 工程的 LZ4 压缩点云分片。
- `POS1`：香橙派位姿包，地面站在 3D 场景中显示实时位姿 marker。
- `GOL1`：香橙派单目标点下发协议。
- `RTE1`：本机 Demo 完整航线快照协议。

注意：

- 香橙派现有远端主要支持 `GOL1` 单目标点，不支持稳定航点 ID，也不支持一次接收完整航线。
- `GOL1.seq` 只能作为发送序号，不应解释为航线中的航点 ID。
- 如需航点 ID、任务 ID、ACK、重传或完整航线下发，应先更新协议文档，再与远端程序员协调实现。
- 点云高吞吐链路不要强行改成 MAVLink。未来接入 QGC 时，点云仍建议走独立 UDP/ROS2/DDS/WebSocket 等数据流，MAVLink 只承载任务语义、低频状态、ACK 和飞控控制面。

## 平台与打包约束

- Windows 桌面端是当前主要交付目标，修改 Android、Linux 或协议适配时必须确保不破坏 Windows 原有功能。
- Android 版本只打包 `customgeometry`，不打包 `radarsimulator`。`radarsimulator` 是 PC/香橙派侧点云发送模拟器。
- Android 文件选择可能返回 `content://` URI，应先复制到应用缓存目录，再交给现有 LVX2、BAG、CSV、PCD、PLY 解析器处理。
- Android 已有触控适配方向：全屏 3D 视图、操作抽屉、单指旋转、双指缩放、长按选点。
- `package_runtime` 是 Windows 交付目录目标，Android 不走该目标。

## 常用构建命令

Windows Debug 示例：

```powershell
cmake -S . -B build\codex-debug -DCMAKE_PREFIX_PATH=C:/Qt/6.10.1/msvc2022_64
cmake --build build\codex-debug --config Debug
cmake --build build\codex-debug --config Debug --target package_runtime
```

Windows Release 示例：

```powershell
cmake -S . -B build\release -DCMAKE_PREFIX_PATH=C:/Qt/6.10.1/msvc2022_64 -DCMAKE_BUILD_TYPE=Release
cmake --build build\release --config Release
cmake --build build\release --config Release --target package_runtime
```

Android x86_64 模拟器包示例见 `README.md`，通常使用 Qt Creator 的 `Qt 6.10.3 for Android x86_64` Kit。Android 真机通常使用 `Qt 6.10.3 for Android ARM64-v8a` Kit。

## 开发约束

- 代码和文档优先使用中文注释与中文说明，尤其是头文件中的类职责、协作关系和重要方法说明。
- 修改协议、文件加载、点云渲染、航线发送、日志、Android 文件访问等关键链路时，要同步更新 `README.md` 和 `doc/` 中对应文档。
- 新增功能不要把未实现能力写成已实现能力。产品功能边界以 `doc/product_function_parameters.md` 为准。
- 修改文件加载逻辑时，要同时考虑 LVX2、BAG、CSV、PCD、PLY 和 Android `content://` 缓存路径。
- 修改实时点云渲染时，要注意大点云性能：交互预览、抽稀、异步加载、累计显示、渲染上限和 UI 丝滑度。
- 修改航线/航点交互时，要保持“拖拽过程不实时上传，点击上传按钮后才发送”的现有行为，除非用户明确要求改变。
- 修改香橙派联调逻辑时，优先兼容现有 `something1017` 协议；不要假设可以修改远端 ROS 节点。
- 日志是重要交付能力。新增关键链路时应加入 `.log` 追踪，尤其是 UDP 收发、协议解析、文件加载、异常处理、航线上传和平台差异。

## UI/交互约束

- 桌面端要保持鼠标点选、拖拽、滚轮缩放等操作可用。
- Android 端要避免桌面式大侧栏直接照搬，优先使用全屏 3D 视图和固定操作抽屉。
- 航点应表示“自由空间中的规划点”，不要默认把障碍点云点本身当作可飞航点。
- 坐标轴显示应支持隐藏/显示切换，避免遮挡点云和航线。
- 航点、航线、位姿 marker 的尺寸应兼顾可点击性和不遮挡点云，必要时使用屏幕空间尺寸或随距离衰减。

## 文档交付提醒

关闭 session 或完成大功能前，检查这些内容是否已持久化：

- README 是否更新了新的运行方式、限制和常见问题。
- `doc/udp_protocol.md` 是否反映真实协议。
- `doc/product_function_parameters.md` 是否仍准确区分已实现和未实现能力。
- `doc/real_livox_integration.md` 是否覆盖真实雷达接入影响。
- `doc/qgc_integration_notes.md` 是否仍符合“点云数据面独立，MAVLink 控制面低频”的结论。
- 关键源码头文件是否补充了中文类说明和方法说明。

