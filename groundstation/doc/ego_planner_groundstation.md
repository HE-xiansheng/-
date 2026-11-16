# 同会话 EGO-Planner 地面站接入

当前机载 ROS 规划包名为 `ego_planner`，终端预览入口为 `roslaunch ego_planner run_planner_board.launch manual_preview:=true`，运行前需 source `~/drone_platform/01_apps/ego_planner/devel/setup.bash`。地面站仍通过 `run_board.sh` 管理会话。现有 `/ego_planner/...` 话题与节点名保留兼容；本次仅统一 ROS 包名，没有替换规划算法。

本次复用现有定位、规划和PX4执行器。地面站未增加规划算法、碰撞地图或PX4控制器。Linux香橙派接入通过系统SSH密钥认证，目标 `orangepi@192.168.1.235`；Windows/Android原离线Demo与UDP协议保持原入口。

## 启动与操作

在上位机 `/home/tyzk/dianyunGCS-review-20261026`：

```bash
bash connect-live-pointcloud.sh
```

右上角为11行飞前话题表，可收起；点击行/悬停查看完整话题和发布者。Hz是机载源话题频率，龄期是5Hz快照采样时的源龄期。电池按12秒窗口估算源Hz；采样不足时显示“—”，电池数据过期阈值仍为5秒。整条监视超过2.5秒未刷新时全部失效，不能把旧绿色状态当成当前状态。缺失转发、EKF或电池等如实显示，地面站不自动启动缺失节点。该表不构成解锁许可。

左侧流程：

1. 启动避障/预览：仅调用已有 `run_board.sh`，其原有启动检查和记录行为保留。已有会话复用，不重复启动。
2. 勾选显示原生膨胀栅格；这是原发布器的局部前向可视化，最多抽样18000格，不代表完整地图或地面站独立碰撞检查。
3. 选点时自动采样新鲜FAST-LIO POS1的世界Z（保留亚厘米和负值），同时用于投影平面与该次目标消息。选好后高度固定，不随飞机位置变化；定位过期不回退到1m。已按用户要求移除目标Z调节及原点说明控件。
4. 点击“地图选点/更新预览”，在图中短按选择XY。拖动仍旋转视角，滚轮缩放；面板上的操作不穿透地图。
5. 查看原生目标、验证原因和绿色预览线；目标位于占据/膨胀区或规划失败时保留精确目标并显示原因，不调整目标、不绕过检查。
6. 核对当前目标及绿线，勾选确认，再点击“确认当前目标并启用PX4接入”。只有精确 `session_id:goal_id` 的确认回执成功后才检查并复用/启用现有执行器。
7. 保持原飞行流程：人工POSCTL起飞悬停，遥控切OFFBOARD才执行；切回POSCTL暂停并接管。地面站不解锁、不起飞、不切模式、不写飞控参数。橙线为原生飞行重规划，蓝线为FAST-LIO实测轨迹；实测到达与执行器输出分别显示。
8. “停止避障”先调用已有执行器stop，立即清除预览曲线并显示“停止处理中”。OFFBOARD内按原策略先保持，遥控切POSCTL后由原执行器结束输出；确认执行器进程已退出、飞控状态新鲜且不在OFFBOARD后，才调用已有run_board.sh --stop结束规划预览会话。未知/过期状态不会强杀规划器，失败不会显示“已停止”。点云、定位、坐标轴和实测历史保留；重新启动避障/预览后重新选点、确认。

原离线航线/GOL1/RTE1上传区在此模式隐藏；它们不用于EGO-Planner导航。停止本工具遥测用 `stop-live-pointcloud.sh`，此脚本不停止执行器；飞行中先按上述停止/遥控接管流程操作。

## 真实接口与坐标

| 来源节点/入口 | 话题/命令 | 用途 |
|---|---|---|
| `/factory_laserMapping` | `/factory_fastlio/cloud_world` | 当前FAST-LIO世界点云 |
| `/factory_laserMapping` | `/factory_fastlio/odometry` | camera_init → body位姿及实测路径 |
| `/factory_vision2mavros`（若实际运行） | `/mavros/vision_pose/pose` | 外部定位转发监视，缺失如实显示 |
| `/mavros` | state、local_position/odom、extended_state、estimator_status、rc/in、battery | 原飞前采样器监视 |
| 雷达驱动 | `/livox/lidar`、`/livox/imu` | 原始雷达/IMU监视，发布者取实时ROS图 |
| `/drone_0_ego_planner_node` | `grid_map/occupancy_inflate`（节点命名空间下） | 原生膨胀栅格格心 |
| 同上 | `/ego_planner/manual_preview/status` | 目标、验证、会话、确认token、到达状态 |
| 同上 | `/ego_planner/manual_preview/trajectory` | 原生PolyTraj，仅采样显示 |
| `map_navigation.py preview X Y Z` | `/ego_planner/navigation/preview_goal` | 候选PoseStamped，world框架 |
| `map_navigation.py confirm token` | `/ego_planner/navigation/confirm` | 精确目标人工确认 |
| `/ego_planner_px4_execution` | `/ego_planner/px4_execution/status` | 独立执行状态与实际输出类型 |
| `run_px4_execution.sh enable/stop/status` | 现有执行器管理 | 接入、受控停止与状态检查 |

FAST-LIO camera_init与现有规划world采用同一会话数值原点；地面站只应用既有显示映射 `(X,Y,Z) → (-Y,Z,-X)*45`。不额外旋转或平移，不把MAVROS local_position混入地图。body原点为定位/IMU参考，“定位前向”未经机头安装外参标定，不等于真北航向。

实际执行仍由既有适配器处理原生轨迹时钟、P/V/A、机载定位到PX4本地原点的转换与失效保持。绿线是测量位置发起的预览，人工确认后切OFFBOARD由原生FSM重新规划，实飞并非直接播放绿线。地面站不重定时、不将采样折线发给PX4。轨迹显示需原生状态新鲜、trajectory_id匹配且符合原查看器时效；会话变更清除旧轨迹/栅格缓存。实测路径最多2000点，位姿断流后分段重置。

## 数据通道与源码

- 点云/位姿沿用单向PCD1/LZ4 UDP9000与POS1 UDP9001，无目标接收器。
- 监视复用原工作台 `telemetry_reader.py`、`preflight_status.py` 副本（来源哈希在 `tools/preflight/*source.json`），由SSH管理只读采样器，状态与栅格分别经TLM1/TLG1单向UDP送上位机9005。栅格和状态订阅已有节点，关闭ROS日志发布，无ROS控制发布器。
- 上位机桥把11行表和原生规划显示结果送至**本机**127.0.0.1:9003（PFS1 JSON），膨胀栅格复用PCD1编码送本机9004。未新开机载网络端口，未安装或更改机载文件。
- 操作由 `PlannerController` 按钮调用已有SSH命令；预览与人工确认独立。没有自动启动执行器的定时器/连接回调。
- 关键文件：`PreflightPanel.qml`、`preflightreceiver.*`、`PlannerPanel.qml`、`plannercontroller.*`、`tools/preflight/`；日志在构建目录 `logs/` 与上位机 `.pointcloud-link/preflight-*.log`。
- 原生栅格消息本身无有效源时间戳，按接收龄期和原规划状态失效；表中规划点云则严格使用cloud_world原始源时间戳。不把接收时间伪装为源时间。

## 验证与当前限制

Qt6.8.3 Linux编译通过，实际窗口布局与右上角位置已检查；原飞前评估10项、显示15项与POS1回归7项通过；Qt接收/变换/体素/精确目标及停止门控已在假SSH下验证。

隔离窗口已用独立端口验证点云、位姿、目标和绿/蓝路径真实渲染（测试截图在上位机 `.pointcloud-link/ui-isolated/`，均为虚拟输入，不是实飞记录）。鼠标联动因系统锁屏尚未完成；未尝试绕过锁屏。

假SSH只存在临时测试目录，测试不连接真实飞控。2026-10-26曾遇SSH不可达；2026-10-27已连接机载，地面只读观察了ROS源话题和栅格显示流。栅格/低频状态修复经隔离验证，但本轮未做飞行验收；请勿把本地测试等同于飞行验证。断流显示真实过期状态，没有修改网络。

只支持同次定位会话；没有实现重启后的旧地图重定位，保存或加载PCD不构成重定位。

旧EGO-Planner工作台的选点入口未增加“上一目标到达并稳定悬停”的前置检查；具体实飞行为取决于当时机载规划器版本。当前地面站使用独立候选与人工确认流程。

## 同一次 OFFBOARD 中的下一目标

2026-10-28起移除“上一目标必须到达并稳定悬停”的前置检查，允许在同一次OFFBOARD的执行途中选择下一目标。选择下一点仍采用当前定位Z；地面站显示独立候选目标及绿色局部预览，原实飞轨迹与确认状态不被预览改写。用户核对并勾选后，点击“确认新目标并继续飞行”；这个按钮不重新启用执行器，不发送解锁或模式切换命令。

候选预览由短时独立进程使用当前原生栅格和正在运行规划器的参数快照计算。目标处于已知原始占据、膨胀区域或预览失败时明确拒绝；确认前还会对最新原生膨胀栅格作连续轨迹碰撞检查。定位/点云过期、当前目标或其他飞行器轨迹变化会使候选失效；正常移动及同一目标的原生重规划不会仅因位置变化使候选失效。短时进程失败只拒绝候选，不改变当前目标。

候选预览从选点时的实飞参考位置、速度和加速度生成。人工确认后，原规划器进入原生REPLAN_TRAJ，从确认时当前参考P/V/A重新计算实飞轨迹，不从旧终点或候选旧起点跳接。已完成轨迹的参考时间限制在终点，避免多项式外推。绿色候选是选点时的快照预览，实际轨迹继续按原生地图和状态重规划；候选不直接送给执行器。遥控切回POSCTL仍立即接管。修改不涉及PX4执行器、解锁、模式切换或飞控参数；空中验收尚未进行。


## 2026-10-27：核对RViz原生选点与等高模式

ROS Noetic RViz `PoseTool` 在固定坐标系Z=0平面进行射线求交：左键按下确定XY，按住拖动以atan2计算yaw，松开完成。`GoalTool` 发布PoseStamped，写入XY和yaw四元数；Z保留消息默认0，不读取机器人当前高度、不吸附点云表面。原始依据：[pose_tool.cpp](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/tools/pose_tool.cpp)、[goal_tool.cpp](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/tools/goal_tool.cpp)。

本工程 `tools/manual-preview/manual.rviz` 配置Fixed Frame=camera_init、rviz/SetGoal到/goal。本地部署版 `waypointCallback` 原样使用XYZ，没有用当前odom替换Z，也没有读取目标orientation。不能把原生RViz表述为自动等高，也不能把拖出的yaw箭头承诺成现有PX4最终机头指令。

地面站当前仍是点击选目标、拖动旋转相机；尚未实现完整的RViz按住拖出方向手势。等高是无人机侧的目标高度取值规则；它保留地图原点、候选预览/确认和原碰撞检查，不增加机头控制策略。实际PlannerPanel QML已通过等高取值、无新鲜定位拒绝、负值/亚厘米精度与选点快照固定检查；新版已编译。没有向机载发送新的候选、确认或执行命令。

2026-10-27界面精简：用户要求删除目标高度输入，现只在选点时自动取当前定位Z，保留简短等高状态。面板标题按用户命名为“Ego-Planner 避障”；既有机载规划/执行接口继续复用，标题更名不代表更换算法。


## 2026-10-27：停止按钮结束预览会话

旧版停止按钮只请求执行器停止，规划器仍按measured_preview刷新，所以会继续显示细绿线。已改为上述完整停止顺序，而非仅隐藏仍活跃的预览。`tools/planner/stop_preview.py`作为Qt资源随程序打包，经SSH stdin临时执行，不安装新机载服务。它复用原执行器管理器的PID身份校验，并持有原`.px4-execution-management.lock`防止停止预览时并发启用新执行器。只有执行器已退出和飞控非OFFBOARD的状态明确后，才调用原会话管理器停止自己拥有的roslaunch/记录会话；不操作定位、MAVROS或飞行模式。

界面分别显示停止处理中、完成和未确认；旧ACTIVE采样不会让曲线或目标确认重新生效，显式“启动避障/预览”才恢复预览。粗红/绿/蓝线为坐标轴，细蓝线为实测历史，这些不因停止规划消失。等待期间保持窗口运行，遥控切回POSCTL后等待完成回执；连接失败时核对真实执行状态，不以曲线隐藏作为停飞证明。

新增隔离验证覆盖执行器仍运行/身份未知、OFFBOARD、飞控断连/源龄期无效、遥控接管后的停止、原停止入口失败、旧采样不恢复曲线和显式重启。隔离测试不连接真实飞控。随后已在真机未解锁/POSCTL/在地面、执行器未运行的状态下验证同一停止管理链：原执行器stop返回无输出，新保护入口成功结束预览，后续管理状态均running=false；FAST-LIO、点云与外部Pose仍正常。新版GUI按钮仍需重启窗口后生效，尚未进行空中停止验收。


## 2026-10-27：提示弹窗和视角平移

静态预览成功后由单独完成事件触发非模态提示，3秒后消失，不抢焦点。重复成功事件可再次提示，周期状态采样不会反复弹窗或延长显示。左侧不保留成功行，失败原因仍可见。实际QML组件在不占用桌面的窗口下通过自动关闭、重复事件、非模态和不抢焦点检查，没有发起SSH/ROS操作。

桌面中键拖动平移观察中心，左键空白/右键拖动旋转，滚轮缩放；“点云与视图”可复位视角。平移只修改相机支点cameraFocus，并复用当前View3D投影计算屏幕平面位移；点云、目标、定位及其米制转换不变，既有鼠标投影选点继续使用实际相机。中键分支不进入选点或航点编辑。

录制已改为独立手动控制，原“停止规划同时停止录制”的旧说明以 [手动录制说明](manual_recording.md) 为准。新增录制卡片不改变飞行接管/停止流程。已移除用户指出的5处冗余提示，仅保留运行状态和异常原因。

## 2026-10-28：恢复远目标选点

初次选点与同次OFFBOARD候选选点均取消“目标必须位于当前局部栅格内”的硬限制。`OUTSIDE_LOCAL_MAP`只说明终点尚未验证，地面站允许人工确认并显示目标已选定。远目标仍作为原生全局目标，实际只生成规划视距内的局部轨迹，沿飞行方向更新滚动地图并重规划；不要求人工分段选点。

候选轨迹校验采用原生`getInflateOccupancy`的缓存外语义：缓存外未知区不单独拒绝，缓存内已知障碍/膨胀与原有虚拟高度墙仍拒绝，保留有限值、输入新鲜度和人工令牌检查。移除地图范围检查不表示远端终点或未知路线已检查为自由；原生动力学优化失败仍会拒绝本次候选，没有放宽速度、加速度或碰撞参数。源包位于`tools/planner/offboard-retarget/`，本机编译与隔离测试结果见其中的验证记录；复制源码不表示机载已更新。

视频面板标题为“视频流拉取”，点击标题可展开/收起双目、云台和人物识别控件；默认展开，收起保留现有连接及识别状态，展开后继续显示。

“隐藏高处点云”和“显示高度上限”同时作用于普通点云及橙色原生膨胀栅格的显示，用于隐藏顶部遮挡；仅在地面站裁剪绘制，不删除机载规划地图中的障碍物。

## 连续选点的目标与到达状态

新候选产生后，地图仅显示当前候选目标标记，面板改为显示新目标坐标及“新目标待确认”，不把旧执行目标的“已到达”当成候选状态。预览失败或未就绪时，不得回退使用旧目标确认令牌；确认只发送当前READY候选的匹配令牌。预览和监视快照交错到达时，旧快照不能覆盖预览返回的新令牌。

人工确认成功才替换机载实际目标，增加goal_id、清空实测到达状态并进入原生重规划；提交回执立即报告新目标ACTIVE，避免把旧WAIT_TARGET状态附在新目标上。确认前原实际目标保持有效，不因隐藏标记而改变飞控执行。

顶部栅格显示默认高度上限调整为定位坐标Z=1.8m（可在“点云与视图”修改）。实际读取的当前场地顶部密集膨胀层约在Z=2.0～2.75m，先前2.5m上限只去掉了其中上部，无法完全隐藏该层；该默认值不是自动天花板识别，其他场地应按实际高度调整。

## 真实雷达参数教学 demo

保持雷达、FAST-LIO运行；先在原终端停止已有规划预览，再启动独立教学入口：

```bash
source ~/drone_platform/01_apps/ego_planner/devel/setup.bash
roslaunch ego_planner demo.launch
```

`roscd ego_planner` 后编辑 `config/demo.yaml`。可修改最大速度、最大加速度、局部规划距离、地图分辨率、障碍膨胀半径及优化器间距和时间代价。YAML 的 `&name` 定义与 `*name` 引用用于同步管理器和优化器参数，修改定义处即可。参数在启动时读取；每次改一项，Ctrl+C 后重启 demo，再使用相同起点和目标比较轨迹。已有 `run_board.sh` 与 `run_planner_board.launch` 默认值保持原样。

可以用 `rosparam get /drone_0_ego_planner_node/manager/max_vel` 查看生效值；demo固定使用 `manual_preview=true`，只运行规划预览，不启动轨迹执行服务器或PX4执行器。不要同时启动其他规划会话。独立副本可用 `roslaunch ego_planner demo.launch config:=/绝对路径/自己的配置.yaml` 加载。
