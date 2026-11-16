# QGC 集成链路建议

本文记录未来把点云地面站能力集成到 QGroundControl（QGC）时的通信链路结论。当前项目尚未实现 QGC 插件或 QGC custom build，本文件只作为后续架构约束和对接说明。

## 总体结论

推荐把链路拆成两层：

- 点云数据面：继续使用独立高吞吐链路，例如当前 `PCD1/raw`、`PCD1/LZ4` UDP 分片，或后续 ROS2/DDS/WebSocket/专用二进制流。
- 飞控控制面：使用 MAVLink 承载位姿、任务、航点、ACK、状态、模式切换和安全确认等低频语义。

不要把完整实时点云直接塞进 MAVLink。MAVLink 更适合飞控遥测和任务协议，不适合承载 Livox MID-360 级别的 dense point cloud。

## 为什么点云不走 MAVLink

当前 demo 默认按 MID-360 级别点率模拟：`5 Hz * 40000 点/帧 = 200000 点/秒`。即使只传 `xyz float32`，原始数据也约为 `2.4 MB/s`，还不包含包头、时间戳、强度、压缩和重传开销。

MAVLink 2 单包 payload 很小，适合遥测、命令和任务交换。如果用 MAVLink 自定义消息或 `TUNNEL` 把完整点云切片，会带来这些问题：

- 每秒产生大量 MAVLink 包，挤占飞控遥测链路。
- QGC 的 MAVLink 解析、日志和转发链路会被高频点云拖慢。
- 点云丢包和乱序处理会污染原本应该清晰可靠的任务/控制语义。
- 后续排障时难以区分“飞控链路异常”和“点云数据流过载”。

因此，点云应保留独立数据流；MAVLink 只传必要的低频状态和任务语义。

## MAVLink 适合承载的内容

未来进入 QGC custom build 后，可优先考虑把以下内容接入 MAVLink 体系：

- 飞行器/机器人位姿：`ODOMETRY`、`LOCAL_POSITION_NED`、姿态类消息，替代或桥接当前 `POS1`。
- 单目标点或任务航点：Mission Protocol、`MISSION_ITEM_INT`，或按项目需要定义 custom dialect。
- 任务控制：`COMMAND_INT`、`COMMAND_LONG`、模式切换、开始/暂停/取消任务。
- 任务可靠性：ACK、任务 ID、航点 ID、当前执行航点、失败原因和安全确认。
- 低频障碍摘要：必要时可用类似 `OBSTACLE_DISTANCE` 的语义传递避障摘要，但它不能替代完整点云地图。

如果需要 custom dialect，应同时修改 QGC custom build、机载端 MAVLink 生成代码和协议文档；不要只在一端临时扩展字段。

## 不推荐的方案

- 不推荐用 MAVLink `TUNNEL` 承载完整点云。它适合少量专用数据透传，不适合作为高吞吐点云通道。
- 不推荐用自定义 MAVLink 消息把每帧点云切成大量分片。技术上可以做，但会把点云数据面和飞控控制面耦合到一起。
- 不推荐让官方未改源码的 QGC 直接承载本项目点云功能。当前判断基于 QGC custom build 或插件化改造。
- 不推荐继续用裸 UDP 作为真实飞控最终控制链路。当前 UDP 航线适合 demo 和联调；进入真实飞控前应迁移到 MAVLink 任务/命令语义，或至少补齐 ACK、重传、任务 ID 和安全确认。

## 推荐 QGC custom build 架构

```text
Livox / ROS / 点云适配器
    |
    | 高吞吐点云流：PCD1/LZ4 UDP、ROS2/DDS 或后续专用流
    v
QGC custom build
    |
    | PointCloudManager：接收、解码、抽稀、缓存点云
    | PointCloudView：显示点云并规划航点
    v
QGC Vehicle / MAVLink 体系
    |
    | MAVLink：位姿、任务航点、ACK、状态、安全确认
    v
飞控 / 机载规划端
```

建议在 QGC 侧新增独立的 `PointCloudManager`，它不参与 MAVLink 包解析，只负责点云流接收、帧重组、压缩解码、抽稀预览和模型缓存。航点规划 UI 可以复用本项目的自由空间选点、线段避障检查、单航点发送和完整航线上传思路，但最终任务下发应接入 QGC 原有 Vehicle/MAVLink 体系。

## 与当前项目协议的对应关系

| 当前协议 | 当前用途 | QGC 集成建议 |
| --- | --- | --- |
| `PCD1/raw` | 本机模拟器实时点云 | 可作为 QGC 点云模块早期输入流 |
| `PCD1/LZ4` | 香橙派 ROS 压缩点云 | 可作为 QGC 点云模块主要实时输入流 |
| `POS1` | 香橙派位姿显示 | 后续桥接或替换为 MAVLink 位姿消息 |
| `GOL1` | 单目标点下发 | 后续迁移为 MAVLink 任务/命令或 custom dialect |
| `RTE1` | demo 完整航线快照 | 可作为原型参考，真实飞控建议转为 MAVLink Mission 或可靠任务协议 |

当前 UDP 协议的线格式见 [udp_protocol.md](udp_protocol.md)。真实 Livox 适配方式见 [real_livox_integration.md](real_livox_integration.md)。项目总说明见 [../README.md](../README.md)。

## 阶段建议

1. QGC 原型阶段：保留 `PCD1/LZ4` 点云 UDP，新增 QGC 点云页面，只做显示和本地规划验证。
2. 飞控联调阶段：将位姿和任务确认接入 MAVLink，保留点云独立数据流。
3. 真实任务阶段：航线/航点下发迁移到 MAVLink Mission Protocol 或双方确认的 custom dialect，并补齐 ACK、任务 ID、航点 ID、安全确认和机载端二次避障。
