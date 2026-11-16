> ROS2迁移保持PCD1/POS1/GOL1/RTE1及状态TLM1/TLG1/PFS1线格式。飞前采样器已改类型化ROS2订阅和DDS图；Qt连接路径通过跨机合成输入验证。试验版状态分片端口为9015（可指定），避免占用原ROS1的9005。未新增ACK或稳定航点ID，也未验收真实雷达/飞控融合。

# 点云/航线 UDP 协议

本文档描述当前已经落地实现的 UDP 协议。未确认的航点 ID、任务确认、ACK/重传等扩展方案不写在本文正式协议中，见 [waypoint_id_protocol_proposal.md](waypoint_id_protocol_proposal.md)。

## 总览

当前项目同时支持两套通信链路：

| 链路 | 方向 | 默认端口 | 协议 | 当前用途 |
| --- | --- | --- | --- | --- |
| 本机 demo | `radarsimulator.exe -> customgeometry.exe` | `56000` | `PCD1/raw` | 未压缩点云分片 |
| 本机 demo | `customgeometry.exe -> radarsimulator.exe` | `56001` | `RTE1` | 完整航线快照 |
| 香橙派 ROS | `udp_uav_tx_node -> customgeometry.exe` | `9000` | `PCD1/LZ4` | LZ4 压缩点云分片 |
| 香橙派 ROS | `udp_uav_tx_node -> customgeometry.exe` | `9001` | `POS1` | 机体实时位姿 |
| 香橙派 ROS | `customgeometry.exe -> udp_uav_tx_node` | `9002` | `GOL1` | 单目标点下发 |

注意：`PCD1/raw` 和 `PCD1/LZ4` 名称相近，但线格式不同。`PCD1/raw` 的 magic 是 ASCII `PCD1`；香橙派 `PCD1/LZ4` 的 magic 是一个 little-endian `uint32_t` 值 `0x50434431`。

## 坐标系

UDP 中所有坐标均为米制机器人/雷达坐标，不传 Qt 3D 场景坐标。

| 坐标轴 | 语义 |
| --- | --- |
| `X` | 前方 |
| `Y` | 左方 |
| `Z` | 上方 |

地面站渲染时转换到 Qt Quick 3D 坐标：

```text
sceneX = -Y * sceneScale
sceneY =  Z * sceneScale
sceneZ = -X * sceneScale
```

反向转换：

```text
X = -sceneZ / sceneScale
Y = -sceneX / sceneScale
Z =  sceneY / sceneScale
```

## 字节序和类型

- 所有多字节字段均按 little-endian 解释。
- `float32` 为 IEEE 754 单精度浮点。
- `float64` 为 IEEE 754 双精度浮点。
- 香橙派 ROS 工程使用 `#pragma pack(push,1)` 的 C 结构体，不含填充字节。

## 本机点云协议 `PCD1/raw`

用于 `radarsimulator.exe` 向地面站发送未压缩点云。点云帧可能拆成多个 UDP 分片；接收端只有收到同一帧的全部分片后才刷新画面。

线格式 magic 是 ASCII 字节：

```text
50 43 44 31    "PCD1"
```

头部长度固定为 32 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `char[4]` | magic，固定为 ASCII `PCD1` |
| 4 | `uint16` | 协议版本，当前为 `1` |
| 6 | `uint16` | 头部长度，固定为 `32` |
| 8 | `uint32` | 点云帧序号 |
| 12 | `uint16` | 当前分片下标，从 `0` 开始 |
| 14 | `uint16` | 当前帧分片总数 |
| 16 | `uint32` | 当前帧总点数 |
| 20 | `uint32` | 当前分片点数 |
| 24 | `uint64` | 发送端时间戳，微秒 |

点记录长度固定为 16 字节：

| 顺序 | 类型 | 含义 |
| --- | --- | --- |
| 1 | `float32` | `X`，米 |
| 2 | `float32` | `Y`，米 |
| 3 | `float32` | `Z`，米 |
| 4 | `float32` | `intensity`，范围建议 `0.0 ~ 1.0` |

## 本机航线协议 `RTE1`

用于地面站向本机模拟器发送完整航线快照。当前香橙派远端尚未实现 `RTE1` 接收。

线格式 magic 是 ASCII 字节：

```text
52 54 45 31    "RTE1"
```

头部长度固定为 24 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `char[4]` | magic，固定为 ASCII `RTE1` |
| 4 | `uint16` | 协议版本，当前为 `1` |
| 6 | `uint16` | 头部长度，固定为 `24` |
| 8 | `uint32` | 航线快照序号 |
| 12 | `uint32` | 航点数量 |
| 16 | `uint64` | 发送端时间戳，微秒 |

航点记录长度固定为 12 字节：

| 顺序 | 类型 | 含义 |
| --- | --- | --- |
| 1 | `float32` | `X`，米 |
| 2 | `float32` | `Y`，米 |
| 3 | `float32` | `Z`，米 |

每个 `RTE1` 包都包含完整航线列表，接收端应以后到的合法包为准。当前 demo 不做 ACK、CRC 或重传。

## 香橙派点云协议 `PCD1/LZ4`

来源：`something1017/src/udp_uav_tx/src/udp_uav_tx_node.cpp`。

香橙派节点订阅 ROS `sensor_msgs/PointCloud2`，取其中 `x/y/z(float32)`，拼成连续数组后用 `LZ4_compress_default()` 压缩为一个 LZ4 block，再按 UDP 分片发送到 PC。

线格式 magic 是 little-endian `uint32_t`：

```text
值: 0x50434431
线字节: 31 44 43 50
```

头部长度固定为 22 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `uint32` | magic，固定值 `0x50434431` |
| 4 | `uint32` | 点云帧编号，远端每帧自增 |
| 8 | `uint16` | 当前分片序号，从 `0` 到 `frag_cnt - 1` |
| 10 | `uint16` | 当前帧总分片数 |
| 12 | `uint32` | 解压后的总字节数，必须等于 `点数 * 3 * 4` |
| 16 | `uint32` | 完整压缩帧字节数 |
| 20 | `uint16` | 当前 UDP 包 payload 长度 |
| 22 | `uint8[]` | 当前分片的 LZ4 压缩数据片段 |

解压后的点记录不含 intensity：

| 顺序 | 类型 | 含义 |
| --- | --- | --- |
| 1 | `float32` | `X`，米 |
| 2 | `float32` | `Y`，米 |
| 3 | `float32` | `Z`，米 |

地面站显示该协议点云时，会把 intensity 固定为 `1.0`。

当前地面站校验规则：

- `frag_cnt` 必须大于 `0`。
- `frag_id` 必须小于 `frag_cnt`。
- `payload_len` 必须大于 `0`。
- UDP 包长度必须等于 `22 + payload_len`。
- `uncomp_bytes` 必须能被 `12` 整除。
- 单帧解压前上限：压缩数据 `64 MiB`，解压数据 `256 MiB`。

## 香橙派位姿协议 `POS1`

来源：`something1017/src/udp_uav_tx/src/udp_uav_tx_node.cpp`。

香橙派节点订阅 ROS `geometry_msgs/PoseStamped`，将位姿打包成一个 44 字节 UDP 包发送到 PC。

线格式 magic 是 little-endian `uint32_t`：

```text
值: 0x504F5331
线字节: 31 53 4F 50
```

包长度固定为 44 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `uint32` | magic，固定值 `0x504F5331` |
| 4 | `uint32` | 位姿包序号，远端每发一包自增 |
| 8 | `float64` | ROS 时间戳，单位秒 |
| 16 | `float32` | `px`，米 |
| 20 | `float32` | `py`，米 |
| 24 | `float32` | `pz`，米 |
| 28 | `float32` | 四元数 `qx` |
| 32 | `float32` | 四元数 `qy` |
| 36 | `float32` | 四元数 `qz` |
| 40 | `float32` | 四元数 `qw` |

地面站收到 `POS1` 后会在 3D 场景中显示机体当前位置、水平航向和地面投影。

## 香橙派目标点协议 `GOL1`

来源：`something1017/src/udp_uav_tx/src/udp_uav_tx_node.cpp`。

地面站向香橙派发送单目标点。远端收到后发布 ROS `geometry_msgs/PoseStamped` 到 `/udp/goal_pose`。

线格式 magic 是 little-endian `uint32_t`：

```text
值: 0x474F4C31
线字节: 31 4C 4F 47
```

包长度固定为 44 字节：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | `uint32` | magic，固定值 `0x474F4C31` |
| 4 | `uint32` | 发送序号，地面站每发送一个 `GOL1` 包自增 |
| 8 | `float64` | 时间戳，单位秒 |
| 16 | `float32` | `px`，米 |
| 20 | `float32` | `py`，米 |
| 24 | `float32` | `pz`，米 |
| 28 | `float32` | 四元数 `qx`，当前地面站发送 `0` |
| 32 | `float32` | 四元数 `qy`，当前地面站发送 `0` |
| 36 | `float32` | 四元数 `qz`，当前地面站发送 `0` |
| 40 | `float32` | 四元数 `qw`，当前地面站发送 `1` |

远端发布 `/udp/goal_pose` 时：

```text
header.stamp    = GOL1.stamp
header.frame_id = frame_id 参数，默认 "map"
pose.position   = GOL1.px/py/pz
pose.orientation = GOL1.qx/qy/qz/qw
```

### 当前限制

- `GOL1` 不携带可靠的航点 ID、航点总数或任务 ID。
- `GOL1.seq` 是普通发送序号，不应当解释成“航线中的第几个航点”。
- ROS `PoseStamped.header.seq` 也不适合承载业务航点 ID。
- 地面站在“香橙派 GOL1 逐点”模式下上传完整航线时，会按本地航点列表顺序连续发送多个 `GOL1` 包；但 UDP 本身不保证可靠到达、严格顺序或去重。
- 航点列表中的“发送”按钮会发送一个单独 `GOL1` 目标点。

如果飞控/规划端需要可靠识别航点 ID 或一次接收完整航线，应与远端程序员协调协议扩展，见 [waypoint_id_protocol_proposal.md](waypoint_id_protocol_proposal.md)。

## 地面站香橙派预设

点击地面站左侧“套用香橙派 UDP”后，会设置为：

```text
点云监听端口：9000
位姿监听端口：9001
航线目标：192.168.1.196
航线端口：9002
航线协议：香橙派 GOL1 逐点
```

如果远端 `uav.launch` 中的 `pc_ip` 或实际香橙派 IP 不同，需要同步修改地面站目标地址和远端参数。

## 版本演进建议

当前协议适合首版室内点云地面站联调。真实飞控链路建议补齐：

注意：本 UDP 点云协议属于点云显示/规划数据面，不等同于飞控控制面。未来如果集成到 QGroundControl，点云仍建议保持独立高吞吐链路；MAVLink 只承载任务语义、航点/航线控制、ACK 和低频状态。详细建议见 [qgc_integration_notes.md](qgc_integration_notes.md)。

- 任务 ID 或 route ID。
- 航点 ID 和航点总数。
- ACK/NAK、超时重发和去重。
- CRC 或校验和。
- 坐标系版本字段。
- 协议版本字段和 header length，避免后续扩展破坏兼容性。


## FAST-LIO 单向发送适配器

`tools/pointcloud-only/cloud_sender.py` 复用本文PCD1/LZ4线格式：22字节小端头、magic=0x50434431、LZ4 block、每包总长不超过1400字节。
无新版本、ACK或航点协议；包序号只用于分片重组，不是ROS消息序号、任务ID或源时间戳。原协议不携带frame名称，接入时必须确认源为camera_init/world米制世界坐标。
源过期0.6秒、帧名非法或空点云时暂停发送并在SSH日志中说明。默认5Hz和最多60000点仅限制显示负载，不修改ROS源。
管理器通过SSH运行发送适配器；数据仅由机载UDP发向上位机9000，不创建GOL1/POS1通道。


### FAST-LIO POS1 遥测来源增补

可选位姿来自`/factory_fastlio/odometry`的pose.pose，沿用小端`<IId7f>`44字节：magic0x504F5331、uint32发送序号、double源时间秒、XYZ和xyzw四元数float32。
帧名不在线格式中传输；发送端验证与点云frame一致且子帧为body，有限位置/四元数、单位范数偏差不超过0.05、ROS源时间不超过0.6秒。
Qt接收端增加有限数和四元数校验，丢弃重复/乱序源时间戳，连续1.5秒无有效新包隐藏旧位姿；合法POS1线格式未变。
UDP9001仍仅为显示接收端，位姿不会转发到MAVROS或飞控。Qt场景水平朝向使用原yaw→Y旋转，+90°对世界+Y/Qt-X的映射已测试。

## 上位机回环监视通道（2026-10-26）

EGO-Planner模式新增本机127.0.0.1:9003的PFS1 JSON，来自本机SSH只读采样桥，不是机载目标协议。字段：type=PFS1、schema=1、connected、source_host、rows（固定11个唯一key）、planning（原生status/execution、显示path与有效性、栅格元数据）、stamp。断流2.5秒失效；connected=false携带error，不能刷新旧数据为正常。每包上限60000字节。栅格XYZ继续用原PCD1/LZ4编码到本机9004。两条通道均只做显示，未改变GOL1/RTE1。新导航通过现有SSH map_navigation.py preview/confirm调用ROS候选/确认入口，不走旧航线UDP。详见 [接入说明](ego_planner_groundstation.md)。

PFS1兼容增量：可选recording对象含available、policy、phase、running（录制监督进程）、status_fresh/status_age_s、duration_s、bag_bytes、run、clip_id、request_id、pending_action、error等；未携带录包盘点大数组。开始/停止走SSH recording_manage.py，UDP仍只承载状态，不承载控制。

PFS1 只读监视新增可选 `localization` 对象：`lio_running`、`relay_running` 为经过 `/proc` 身份核对的进程状态；`session_id` 为机载 boot ID + 定位 roslaunch PID/start tick。进程存活不代表话题有效，正常状态仍按 `rows` 的源时间、来源及坐标系判断。旧接收端可忽略此字段；缺失时新定位管理卡片保持未知。

### TLM1 单向只读监视（UDP9005）

机载只读采样器将状态JSON以UTF-8紧凑编码、zlib压缩，按1200字节压缩数据分片；小端头 `<4sIHHI`：magic=`TLM1`、uint32序号、uint16分片下标、uint16分片总数、uint32解压后JSON字节数。最多256片、解压后不超过512KiB；缺片、1.5秒超时、损坏压缩数据或长度不符丢弃。来源IP必须匹配当前配置机载主机。

本机桥重组并核对boot ID/单调采样时间，丢弃同一boot下旧包；再按原格式向127.0.0.1:9003发送PFS1。PFS1新增transport、transport_delay_s、sample_cost_s用于诊断。状态快照5Hz，ROS源Hz单独统计；Qt断流阈值仍为2.5秒。电池源频率统计窗口为12秒，样本不足时为未知，源采样仍以5秒判断过期。UDP9005仅为遥测入口，不接收目标或控制命令。

TLM1 JSON带stream_id绑定本次本机桥实例；旧实例、旧boot或同boot乱序采样不能刷新当前状态。UDP采样进程随其SSH父进程退出，重连不留下重复观察者。

同一次OFFBOARD的下一目标预览通过原 `/ego_planner/navigation/preview_goal` 和 `/ego_planner/navigation/confirm` 入口交互；原 `/ego_planner/manual_preview/status` 在飞行时附带 `candidate` 的会话内编号、状态、原因和独立确认令牌。`/ego_planner/navigation/candidate_trajectory` 只发布供显示的候选多项式，不用于PX4执行。PFS1 `planning.candidate_path` 和 `candidate_path_valid` 只有在当前会话、候选编号和轨迹编号匹配且候选有效时才展示；原 `planning.path` 继续表示原规划轨迹。候选预览进程通过机载本地快照通信，不增加远程控制协议。

### TLG1 原生栅格单向显示流（UDP9005）

另一个只读采样器使用相同的分片头、`TLG1` magic 和独立stream_id，约2Hz向同一上位机UDP端口转发原生膨胀栅格显示抽样，每帧发送两份，并补发同一会话上一帧以降低分片丢失影响；补发保留原始采样时间，接收端去重。点数上限为18000；对满足半格索引及误差条件的格心使用zlib压缩的16位坐标或差分，最大转换误差不超过0.00001米，否则使用原float32压缩。桥解码后仍向本机9004发送PCD1/LZ4。栅格元数据仅匹配当前规划会话，源龄期加传输/排队时间超过1.5秒即过期；空栅格和非法数据分别报告，不刷新为有效障碍地图。此流不改变机载原生栅格、碰撞检查或规划行为。

默认实时PCD1显示发送预算已调为2Hz、最多4000点/帧；POS1仍按源定位更新，FAST-LIO与规划地图不受显示预算影响。

### PFS1 可选传感器字段（2026-10-29）

PFS1 新增可选 `sensors` 对象，键为 `imu`、`fc_imu`、`lidar`、`pressure`。每项含 `ok`、`values`、`topic`、`frame`、`hz`、`age`。`values` 中的 `acceleration`/`angular_velocity` 为 XYZ 数组，`point_count` 为雷达单帧点数，`pressure_pa` 为 Pa。雷达加速度为驱动原值，飞控加速度为 m/s²，角速度为 rad/s。

传感器字段只传小型数值摘要，不传雷达点数组，不新增网络端口。源时间与接收时间均未过期才标记 `ok`；接收端还须按 PFS1 整体新鲜度隐藏旧值。原 11 行 `rows` 不变，旧版本忽略 `sensors`，新版本收到无此字段的旧包时显示未采样。

PFS1 `planning.grid_display_valid` 为可选的绘图保留标记：同会话且规划输入有效时，可显示采样年龄不超过3秒的已接收栅格；`grid_valid` 仍只允许1.5秒以内。1.5～3秒显示“栅格延迟”，不赋予控制有效性。会话身份及整体监视新鲜度依旧必须满足。

### 自动目标入口（2026-10-29）

ROS `/ego_planner/navigation/preview_goal` 继续接收世界坐标PoseStamped，语义改为直接更新原生目标，OFFBOARD内也不经过候选/确认工作流。状态 `confirmation_required=false`；会话、目标和轨迹编号仍供显示、日志关联，均不再是人工授权令牌。旧确认订阅停用。

新增ROS `/ego_planner/flight_execution/trajectory`，镜像原生执行阶段的PolyTraj；接入层只订阅此输出，不把静态预览发给飞控。原有manual_preview/trajectory仍供显示。无人机模式、解锁、降落和原生故障处理不由地面站自动改变。没有新增网络端口。

### 独立任务控制通道（2026-10-29）

三项任务管理不改变PCD/POS/GOL或PFS1报文，命令与约1Hz的状态通过单独SSH通道传输。`gcs_task_manager.py`输出逐行UTF-8 JSON：操作回执含`ok/message`，监视包含`ok/state`；state包含飞控摘要、任务种类/存活/停止处理中/日志、航点状态、ArUco偏移与ID、人物目标。接口语义与字段来源见`mission_controls.md`。上位机4秒未收到监视包即标过期；不将SSH连通或旧回执当作任务持续实时。

### 地图航点任务SSH提交（2026-10-30，机载已部署）

2026-10-30上升限速补充（仅本地更新，尚未机载部署）：JSON请求仍为version=4，机载生成的任务YAML新增`max_z_speed_up: 0.5`；下降继续沿用模板`max_z_speed`。准备时必须通过原生dry-run确认上升限速，旧二进制不支持该字段时拒绝执行，不改变UDP协议。

地图航点不修改UDP协议。`gcs_task_manager.py start-map`从stdin接收JSON：`version:4`、`frame:"fastlio_world"`、`map_session:{pid,ticks,boot}`、`waypoints:[{x,y,z,photo}]`。坐标为FAST-LIO世界系米值；机载用同步位姿转换成MAVROS ENU。hold=2和拍照gimbal_pitch=-90由机载生成，不接受前端任意飞控动作。任务仍由遥控OFFBOARD新边沿触发，原生速度/高度/围栏校验继续有效。回执沿用JSON ok/message/state，状态watch回传map_session以标识当前定位源。
