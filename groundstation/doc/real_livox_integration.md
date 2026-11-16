# 真实 Livox MID-360 接入说明

当前地面站已经把真实雷达接入口预留在 UDP `PCD1` 协议上。后续接真实 MID-360 时，不需要改地面站的 QML 航线规划交互，只需要把 `radarsimulator` 替换成一个 Livox SDK2 适配器进程。

离线建图结果也已经预留入口：地面站可直接加载 LVX2、ROS1 bag、PCD、PLY、XYZ/TXT/CSV 静态点云模型。静态模型和实时雷达点云共用 `PointCloudGeometry`、同一套米制坐标、同一套航点拾取逻辑，因此后续真实雷达在线建图或离线地图文件都可以复用当前航线规划交互。

当前已实现的 UDP 线格式见 [udp_protocol.md](udp_protocol.md)；航点 ID、完整任务下发等未落地扩展建议见 [waypoint_id_protocol_proposal.md](waypoint_id_protocol_proposal.md)。

## 已预留的接口

地面站侧入口：

- `UdpPointCloudReceiver`：监听 UDP 点云包。
- `PointCloudGeometry`：接收米制点云并渲染。
- `RouteUdpSender`：把完整航线通过 UDP 回传。
- `PointCloudFileLoader`：读取 LVX2、ROS1 bag、PCD/PLY/XYZ 静态点云模型，用于显示雷达已经建好的地图或录制点云。
- `pointcloudprotocol.h/.cpp`：统一的 `PCD1` 点云包和 `RTE1` 航线包编码/解码。

模拟器侧入口：

- `radarsimulator.cpp` 当前只是一个点云源示例。
- 真实雷达接入时，可以新建 `livoxadapter.cpp`，复用 `pointcloudprotocol.h/.cpp` 发 `PCD1`，地面站不用变。

## 推荐架构

```text
Livox MID-360
    |
    | Livox SDK2 callback
    v
livoxadapter.exe
    |
    | UDP PCD1, 200000 points/s
    v
customgeometry.exe 地面站
    |
    | UDP RTE1, 完整航线坐标
    v
机器人/控制端/适配器
```

## 与 QGC 集成时的链路分工

如果未来把该能力迁移到 QGroundControl，真实 Livox 适配器仍建议输出 `PCD1/LZ4` 或后续专用高吞吐点云流，不要求改成 MAVLink。QGC custom build 中可新增独立的点云接收/显示模块，负责接收、解码、抽稀和缓存点云。

MAVLink 更适合作为飞控控制面：承载位姿、任务航点、ACK、当前执行状态、模式切换和安全确认。完整实时点云不建议通过 MAVLink 自定义消息或 `TUNNEL` 分片发送。更完整的 QGC 集成建议见 [qgc_integration_notes.md](qgc_integration_notes.md)。

## 适配器需要做的事

1. 使用 Livox SDK2 初始化设备、发现雷达、启动点云数据流。
2. 在 SDK2 点云回调里读取 `x/y/z/reflectivity`。
3. 把 SDK 点坐标转换成米制雷达坐标：
   - 地面站约定：`X` 向前，`Y` 向左，`Z` 向上。
   - 如果 SDK 点类型字段单位是毫米，发送前除以 `1000.0f`。
   - 反射率归一化到 `0.0 ~ 1.0`，例如 `reflectivity / 255.0f`。
4. 按帧或按固定批次调用 `PointCloudProtocol::encodePointCloudChunk(...)`。
5. 通过 UDP 发送到地面站点云端口，默认 `56000`。
6. 监听地面站航线端口，默认 `56001`，用 `PointCloudProtocol::decodeRoute(...)` 获取航点。

## 适配器伪代码

```cpp
// 伪代码：具体 SDK 类型名以当前 Livox SDK2 头文件为准。
void onLivoxPointCloud(const LivoxPointPacket &packet)
{
    QVector<PointCloudPoint> points;
    points.reserve(packet.pointCount);

    for (const auto &raw : packet.points) {
        PointCloudPoint point;
        point.position = QVector3D(raw.x * 0.001f,
                                   raw.y * 0.001f,
                                   raw.z * 0.001f);
        point.intensity = std::clamp(raw.reflectivity / 255.0f, 0.0f, 1.0f);
        points.append(point);
    }

    const int chunkSize = PointCloudProtocol::MaxPointsPerDatagram;
    const int chunkCount = (points.size() + chunkSize - 1) / chunkSize;
    for (int i = 0; i < chunkCount; ++i) {
        const QByteArray datagram = PointCloudProtocol::encodePointCloudChunk(
            frameSequence,
            i,
            chunkCount,
            points.size(),
            timestampUsec,
            points,
            i * chunkSize,
            std::min(chunkSize, points.size() - i * chunkSize));
        udpSocket.writeDatagram(datagram, groundStationHost, 56000);
    }
}
```

## 推荐保持不变的约定

- 地面站内部永远使用米制坐标。
- 点云包允许丢包，航线包每次发送完整航线快照。
- 真实控制链路上线前，建议在 `RTE1` 外层增加 ACK、重传、CRC、任务 ID 和安全确认；地面站当前已能发完整航线，后续只需要扩展发送器即可。

## 需要确认的真实设备参数

- 雷达 IP、主机 IP 和子网是否按 Livox SDK2 示例配置。
- SDK2 当前点类型的坐标单位和坐标轴方向。
- 是否需要把雷达坐标转换到机器人机体系。
- 是否需要累计多帧点云形成地图，还是只显示实时帧。


## 当前 FAST-LIO 实机单向接入

在现有FAST-LIO持续运行时，使用`tools/pointcloud-only/`读取`/factory_fastlio/cloud_world`（camera_init）。
本轮机载192.168.1.235，当前上位机路由源192.168.1.125，目标UDP9000；连接脚本会自动读取上位机路由地址。
不运行旧`uav.launch`的bag循环回放，不启动原示例的目标接收端，不把MAVROS局部位姿与FAST-LIO世界点云混合发送。
当前只传点云；未来若加入位置显示，应单独从同会话FAST-LIO Odometry提取PoseStamped，并核对原点、轴向和源时间。
一键入口、停止、状态和边界见README末尾“本机FAST-LIO单向点云接入”。


### 位姿与点云同源

已增加可选FAST-LIO Odometry→POS1遥测，与cloud_world使用同一camera_init/body会话。
保持现有雷达/IMU外参与定位不变，不添加位置或旋转补偿；body参考点不等同于机头尖端，实际机头与body+X的安装关系需要现场确认。
当前仅上行显示位置和水平前向，不建立目标下行或任何飞控控制通道。

EGO-Planner实时模式已增加飞前话题表、原生膨胀栅格及同会话目标预览/确认，具体数据源、坐标和实机待验边界见 [EGO-Planner接入](ego_planner_groundstation.md)。原始雷达IMU继续用于监视，地图始终取FAST-LIO cloud_world，不混入cloud_body或MAVROS原点。
