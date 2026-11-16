EGO-Planner -> PX4：上游实飞 FSM 与遥控优先（2026-10-23）

执行方式
地面/手持预览继续使用实测位置刷新地图和路径。显式启用PX4接入后，已有目标可直接复用；手动POSCTL起飞后向飞控预发送当前位置。程序不增加悬停2秒、READY2秒、新目标编号等门槛。PX4自身进入OFFBOARD要求仍由飞控执行。
遥控切入OFFBOARD后，/ego_planner/flight_execution=true让同一规划进程进入上游原FSM；首次从实测PV起步，后续通过原planFromLocalTraj延续上一条轨迹P/V/A，碰撞检测、求解失败重试、紧急停车及目标更新都由该FSM处理。
桥接器仅做世界坐标转换，并按原start_time、duration和系数采样PVA；不再二次缩放轨迹时间。轨迹结束后按上游traj_server行为保持末位置、速度和加速度归零。
同一专用轨迹话题兼容RViz：/ego_planner/manual_preview/trajectory；状态planning_mode为measured_preview或upstream_fsm，command_output=false表示规划器自身不直接发送飞控命令。

遥控最高优先
软件不调用解锁、模式切换或飞控参数写服务。MAVROS接收ENU参考后转换为MAVLink NED。
已进入OFFBOARD后，遥控切回POSCTL或其他模式立即暂停路径跟踪，执行器保持显式授权待命。发布前再次读取最新模式，避免计算过程中发生的接管被旧缓存覆盖。
POSCTL下只预发送当前位姿、零速度/加速度，供PX4再次手动选择OFFBOARD时满足其输入流要求；PX4在POSCTL内由遥控控制，不执行这些OFFBOARD参考。可以返回原点、更新目标、再次手动切OFFBOARD，同一执行器会重新对齐坐标并初始化上游FSM，无需重新勾选接入。
软件不自动选择OFFBOARD。cycle_id记录每次手动接入轮次，tracking_output_enabled只表示实际路径跟踪；预发送与保持通过output_kind单独标示。
取消勾选/停止按钮为操作员明确停止：OFFBOARD内保持当前位置，等遥控切回POSCTL后结束。只有这个明确停止请求保留STOP_HOLD，不是规划故障锁定。
接入时只读记录COM_RC_IN_MODE、COM_RC_OVERRIDE、COM_OBL_RC_ACT、COM_OF_LOSS_T；仅RC完全禁用(IN_MODE=4)时拒绝启动，不自动改飞控参数。
机载当前读值：RC_IN_MODE=3、RC_OVERRIDE=1、OBL_RC_ACT=0、OF_LOSS_T=1秒。
RC_OVERRIDE=1允许auto模式的摇杆接管，并不表示OFFBOARD摇杆接管已开启；这里使用已配置的遥控模式开关切回POSCTL，和摇杆自动触发切模式是两种机制。没有把该参数偷偷改成3。

取消的附加策略
取消启用后必须重设目标、低速稳定2秒、预发送2秒才能READY、切入后必须1.5秒内有新轨迹。
取消适配器对目标改变、碰撞状态、重新规划等的永久FAULT_HOLD；有效上游轨迹到达即可执行。
取消接管点10m/垂直[-0.5,+1.5]围栏、参考误差0.35m门槛、每周期约0.17m跳变门槛，以及坐标残差0.2m/姿态5度的附加拒绝。
取消轨迹收到0.8秒/起始1秒、最多100段/总时长60秒、段间PVA差异1e-3等适配器额外拒绝规则。
取消把FC system_status非3/4、额外EKF位、RC通道范围、150ms循环门槛独立升级成规划故障的逻辑。飞控仍负责自身估计器、遥控与Offboard-loss行为。

保留且明确公开的基本接口条件
FC模式状态源和接收不超过2秒；FC位置与LIO不超过0.4秒，用来防止从无效坐标发布控制。定位帧和四元数/多项式必须合法且有限。
FC/LIO源时间配对优先插值，缺少括点时采用50ms内最近样本；保留固定旋转+实时配对平移，避免两套定位原点小偏差成为新的位置误差。
规划状态心跳有效时使用其当前轨迹编号，进入OFFBOARD只使用upstream_fsm轨迹。轨迹起始时间超前超过50ms则暂不采样。
这些接口数据暂不可用时暂停输出或保持等待；恢复后可以继续，不形成额外故障锁。RC切回POSCTL始终先于这些分支暂停路径跟踪；只有显式取消/停止才结束持续授权。
不与另一个飞控命令发布者同时输出。

上游本身仍有的行为（不是新增锁定）
原FSM的碰撞检查、emergency_time=0.8秒、连续失败超过10次紧急停车、stuck检测及fail_safe保留。
保留既有不擅自改写最终目标的设置：目标落入障碍时，上游可能停车并等待新目标。桥接器不再额外要求退出再进OFFBOARD；在上游允许的状态下发送新目标即可继续。
原FSM对真实碰撞的停车仍然存在；“去掉附加锁定”并不是让优化器执行判为碰撞的路径。

动力学参数与时间
速度规划配置保留：水平1.0m/s、垂直0.3m/s；速度代价与验收保留垂直轴尺度，速度验收容差0.05。
2026-10-23 按用户要求撤回加速度收紧适配：删除独立垂直加速度尺度和椭球验收，恢复上游三维范数代价与验收，acc_tolerance=1.0，可行性代价权重恢复10000。max_acc=0.5m/s²是软优化目标，最终范数检查阈值为0.5+1.0=1.5m/s²；不再将0.55m/s²附近候选直接作为加速度失败。碰撞检查和上游重试保留。
上述为规划目标及采样检查，不是实际机体硬限，也不对下游指令做分量截断。删除了下游时间缩放，避免规划器与执行器使用不同轨迹进度。
2026-10-23 按用户要求将机载入口地图分辨率从0.20m改为0.10m，膨胀参数仍为0.35m；按上游ceil取整为4格，实际每轴扩张仍为0.40m。更细栅格减少量化误差，不保证柱间通道一定打开；原碰撞检查漏判另需修复。没有把Z强制拉平或新增高度锁。

日志与操作
状态文件/ samples.jsonl记录planning_mode、trajectory_clock=upstream_native、原FSM阶段、输入年龄、实际坐标与参考PVA。
UPSTREAM_HOLD表示上游停车或等待目标，WAIT_PLAN表示桥接等待上游轨迹，WAIT_INPUT表示缺少有效坐标；新轨迹/输入恢复可以继续。
完整bag仍在已起飞、解锁并进入OFFBOARD后开始，增加记录flight_execution元数据，不回放该控制元数据到实机。
命令：bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh enable|stop|status|check
上位机需重新打开控制台窗口加载文字和状态显示更新。

验证
python3 tools/px4-execution/test_execution.py
隔离ROS11377运行test_ros.py和test_descent_ab.py；后者实际跑同一规划器，旧/新适配器受相同0.03m/s合成向下扰动，不是PX4固件SITL或真实机体验收，也不能唯一还原现场最初下沉扰动。
本地/机载构建、源hash和结果存于runs/active-upstream-control.txt指向目录。

依据
上游不可变检出upstream，commit 5f8551203426b371c22de55e5961d04cf7639c60。
https://docs.px4.io/main/en/flight_modes/offboard
https://docs.px4.io/main/en/advanced_config/parameter_reference#COM_RC_OVERRIDE

2026-10-23 多轮修正：取消“Position退出进程”的单次执行设计。明确取消仍会结束进程；故障恢复与上游FSM策略未改。记录器收到停止请求后不再被Position待命中的后台执行器阻止封包。
