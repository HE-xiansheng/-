# 手动飞行数据录制

> ROS2迁移版本（2026-11-16）：独立适配副本通过 `~/drone_platform/env.sh` 使用 Humble。手动开始/停止和默认关闭的自动录制沿用原操作；录包改为 `inputs-<clip>/*.db3`（sqlite3/CDR），保留metadata.yaml、参数JSON及运行源码摘要。自动停止增加0.6秒连续的新鲜落地+锁桨确认，防止两个话题异步到达造成误判。旧ROS1说明和验证记录保留为历史参考。
>
> 飞行记录窗口可通过SSH列出/下载已收尾ROS1 bag与ROS2 db3，原生解析CDR点云/位姿、按源时间合并分包，无需电脑安装ROS2。SQLite可选依赖缺失的构建会明确提示不支持；未收尾active文件和缺少metadata的ROS2会话不允许下载。当前仅验证隔离合成数据和真实SSH传输，物理飞行数据完整性、雷达负载与掉电数据恢复未验收；不自动重放控制话题。


地面站左侧“飞行数据录制”卡片复用现有机载record_session.py和rosbag。现在只由按钮开始、停止；进入/退出OFFBOARD、解锁/落地或停止规划都不自动启停录制。录制不调用解锁、模式切换或飞控参数写入，不改变PX4执行器策略。

## 操作

1. 连接机载电脑后，点击“开始录制”。状态先显示准备/连接录包话题，真实rosbag开始后才显示“正在录制”。
2. 数字计时显示该段累计时长HH:MM:SS。录制没有预设自动结束时间；按用户最新要求不显示时间条和刻度。
3. 点击“停止录制”，等待“已停止并保存”；保存期间计时冻结，按钮不会把“停止请求已提交”误报为文件已收尾。
4. 可再次开始录制，生成新段，不覆盖前一段。正在录制时重复开始复用当前段，不创建第二个录包进程。

保存位置在香橙派 `~/drone_platform/01_apps/ego_planner/flight-records/<会话>/`，文件名为 `inputs-<段UUID>_N.bag`。悬停卡片保存位置可查看完整目录。若规划会话已存在，沿用它的诊断上下文并按UUID分段；未运行规划器时也可录现有遥测。关闭地面站或SSH断开不会自动停掉机载录制，恢复连接后可查看状态并点击停止。

断流/状态过期时，时长冻结并显示状态未知，不能把界面计时当作录包仍健康的证明；可继续发出显式停止请求。只有机载确认收尾、无`.active`且rosbag info检查成功才显示保存完成。录制错误和未收尾文件会阻止开始新段，保留原因供检查。

## 复用的内容与边界

- 原飞行录制话题保留：FAST-LIO世界点云/里程计、MAVROS外部定位与本地位置/速度/IMU/状态/遥控/电池、规划状态/PolyTraj、诊断事件、占据/膨胀地图、控制参考与反馈、TF和日志。补充当前导航候选与确认话题 `/ego_planner/navigation/preview_goal`、`/ego_planner/navigation/confirm`。
- 原有源码/二进制校验清单、参数和运行环境快照、地图/优化诊断关联保留。规划器本身的诊断日志仍按其既有机制生成，录制按钮控制的是rosbag数据段；不改变规划器的日志策略。
- rosbag继续LZ4压缩、256MB分包、128MB缓存、2GiB可用空间保护，SIGINT完整收尾并生成每包`.info.yaml`与每段`segment-<UUID>.json`。
- 计时从实际rosbag进程启动计算，停止请求后冻结；数据的精确首末消息时间以各包rosbag info为准。所有分段保持本次定位会话坐标，不等同于旧地图重定位或自动飞行回放。

## 实现

机载原 `record_session.py` 改为手动请求驱动，新增 `recording_manage.py start|stop|status|prepare`。管理器用 `.flight-recording-management.lock` 和PID/启动时间/命令行身份检查保持单实例；`prepare`只归档准备，不录包。状态在`.flight-recording.json`所指目录的`recording-status.json`，请求通过`recording-control.json`传递。

`manual_session.py`复用准备上下文供原规划器输出诊断，等待WAIT_MANUAL/STOPPED/RECORDING准备完成；停止规划不触碰手动录制。`px4_execution/manage.py`仅改只读录制健康展示，`adapter.py`、`execution.py`保持原哈希。旧闲置WAIT_FLIGHT进程在确认未解锁、非OFFBOARD、无bag后迁移；规划和定位进程不重启，旧归档另存`before-manual-policy/`。

地面站由独立 `RecordingController` 发起SSH管理请求，录制准备/保存不会占用规划控制器的操作进程。原只读监视流PFS1增加可选recording字段，`PreflightReceiver`转给录制卡片；没有另建飞控或规划系统。

## 验证记录

- localhost11382隔离ROS与真实rosbag通过：OFFBOARD不自动开始、按钮请求开始/停止、重复开始单例、模式退出不自动停止、OFFBOARD下显式停止、文件收尾、停止后时长冻结、重复录制、准备时取消、消息计数、无控制发布、未收尾文件阻止新实例。
- 实际Qt RecordingPanel按钮与假SSH检查：开始/停止管理命令、状态确认、计时更新、保存中与完成的区分、断流冻结、异常重试；未发真实飞控命令。
- 2026-10-27机载地面验证：录制7.174秒、30,081,810字节，60帧FAST-LIO世界点云、60条里程计、7条飞控状态，以及地图、外部定位等数据。停止后无active文件、rosbag info成功，时长冻结，规划PID88692前后不变。文件位于 `~/drone_platform/01_apps/ego_planner/flight-records/20261027-102757/inputs-35cdda6bf48541c3b8d7b18ef15077b5_0.bag`。

更新前机载文件备份：`~/drone_platform/01_apps/ego_planner/.manual-recording-update/20261027-103233/before/`；上位机实施与验证记录在`.recording-upgrade/`。本次验证未解锁、起飞、切模式或写参数。

## 可选自动录制

新增默认关闭的自动录制复选框。`recording_manage.py auto-on/auto-off` 配置只读模式观察器 `recording_auto.py`；沿用同一个录制管理器和rosbag进程。OFFBOARD且已解锁时开始，落地与锁桨均新鲜确认时结束；POSCTL接管继续录制。手动停止不会在同次OFFBOARD状态中立即被自动重启，取消勾选只禁止未来自动开始。必须确认卡片“自动录制待命”状态；重启机载后不把保存的勾选值当作守护进程已启动。

保存自动开始/停止的原始飞控时间戳与请求ID到各会话的automatic-recording-events.jsonl。仅监测模式和着地状态，不主动切模式、解锁或控制飞行。
