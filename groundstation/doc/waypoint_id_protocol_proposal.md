# 航点 ID 协议扩展建议

本文档是协议扩展提案，不是当前已落地协议。当前正式协议见 [udp_protocol.md](udp_protocol.md)。

## 背景

当前香橙派远端只接收 `GOL1` 单目标点，并发布为 ROS `geometry_msgs/PoseStamped`：

```text
customgeometry.exe --GOL1--> udp_uav_tx_node --/udp/goal_pose--> ROS
```

`PoseStamped` 的字段适合表达一个目标位姿，但不适合表达“这是第几个航点”。因此在不修改远端代码的前提下，地面站可以发送单个航点，但不能保证 `/udp/goal_pose` 中携带稳定的航点 ID。

不建议把航点 ID 塞进以下字段：

- `pose.orientation`：这是姿态四元数，后续接飞控会产生错误姿态。
- `pose.position` 的小数位：会污染真实目标坐标。
- `header.frame_id`：这是坐标系名称，下游 TF/规划节点可能依赖它保持为 `map`。
- `header.seq`：ROS 发布器或中间节点可能重写或重新解释该字段，不适合承载业务 ID。

## 方案 A：新增 `GOL2` 单航点协议

适合远端仍希望“一包一个目标点”，但需要知道航点 ID 的场景。

### 数据方向

```text
customgeometry.exe -> udp_uav_tx_node
```

### 建议端口

沿用当前目标点端口 `9002`，远端可同时兼容 `GOL1` 和 `GOL2`：

- 收到 `GOL1`：按旧逻辑发布 `/udp/goal_pose`。
- 收到 `GOL2`：解析航点 ID，再发布目标点或自定义消息。

### 线格式

建议继续使用 little-endian、`#pragma pack(push,1)`，固定头部长度 60 字节。

magic 为 little-endian `uint32_t`：

```text
值: 0x474F4C32
线字节: 32 4C 4F 47
```

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `uint32` | magic，固定值 `0x474F4C32` |
| 4 | `uint16` | 协议版本，建议 `1` |
| 6 | `uint16` | 头部长度，固定 `60` |
| 8 | `uint32` | route ID，地面站每次上传任务递增 |
| 12 | `uint32` | waypoint ID，从 `1` 开始 |
| 16 | `uint32` | 当前任务航点总数；单点模式可填 `1` |
| 20 | `uint32` | flags，预留；bit0 可表示最后一个航点 |
| 24 | `float64` | 时间戳，单位秒 |
| 32 | `float32` | `px`，米 |
| 36 | `float32` | `py`，米 |
| 40 | `float32` | `pz`，米 |
| 44 | `float32` | 四元数 `qx` |
| 48 | `float32` | 四元数 `qy` |
| 52 | `float32` | 四元数 `qz` |
| 56 | `float32` | 四元数 `qw` |

### 远端发布建议

如果远端仍必须发布 `PoseStamped`，建议保持：

```text
header.frame_id = "map"
pose.position = px/py/pz
pose.orientation = qx/qy/qz/qw
```

航点 ID 不建议塞入 `PoseStamped` 字段。更稳妥的方式是：

- 远端日志打印 `route_id/waypoint_id/waypoint_count`。
- 远端额外发布一个自定义消息，例如 `/udp/goal_waypoint`。
- 或发布一个伴随 topic，例如 `/udp/goal_meta`，内容包含 `route_id/waypoint_id/waypoint_count`。

### 优点

- 对当前 `GOL1` 改动小。
- 仍支持单航点实时下发。
- 可以和旧远端逻辑并存。

### 缺点

- 远端仍是逐点接收，不天然知道完整航线形状。
- 如果多个航点连续发送，仍需要远端处理 UDP 丢包、乱序和重复。

## 方案 B：远端支持现有 `RTE1` 完整航线协议

适合远端需要一次拿到整条航线，并在机载端统一做轨迹平滑、避障复核、任务确认的场景。

### 数据方向

```text
customgeometry.exe -> udp_uav_tx_node 或新建 route_rx_node
```

### 建议端口

可以沿用 `9002`，也可以新开完整航线端口，例如 `9003`。如果沿用 `9002`，远端需要按 magic 区分：

- `GOL1`：旧单目标点。
- `RTE1`：完整航线快照。

### 线格式

直接复用当前地面站已经实现的 `RTE1`，详见 [udp_protocol.md](udp_protocol.md)。

核心字段：

```text
magic: "RTE1"
version: 1
sequence: 航线快照序号
waypoint_count: 航点数量
timestamp_usec: 时间戳，微秒
waypoints[]: float32 x/y/z
```

在完整航线数组中：

```text
waypoint_id = 数组下标 + 1
```

### 远端发布建议

远端可以选择：

- 将整条航线保存为当前任务。
- 发布自定义航线 topic，例如 `/udp/route`。
- 按航点逐个转发为 `/udp/goal_pose`，但同时在日志或 meta topic 中带上航点 ID。

### 优点

- 一次获得完整航线，语义最清晰。
- 航点 ID 可以由数组顺序自然得到。
- 后续更容易加 ACK、任务确认和轨迹优化。

### 缺点

- 远端改动比 `GOL2` 大。
- 如果航点很多，可能需要分片或限制单包大小。

## 推荐决策

| 目标 | 推荐方案 |
| --- | --- |
| 只想让 `rostopic echo` 或日志能区分单个航点 | `GOL2` |
| 飞控端只接受一个目标点，短期不做完整任务管理 | `GOL2` |
| 飞控端需要完整航线、任务确认、平滑和避障复核 | `RTE1` |
| 后续要上真实无人机或移动机器人任务链路 | `RTE1`，再加 ACK/CRC |

首版联调建议：

1. 保留现有 `GOL1`，不破坏旧链路。
2. 新增 `GOL2` 或 `RTE1` 接收节点，先在 ROS 日志中验证 `route_id/waypoint_id`。
3. 验证稳定后，再让下游规划/飞控订阅新的完整语义 topic。
