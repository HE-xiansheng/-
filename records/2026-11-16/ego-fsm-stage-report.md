# EGO 运行状态机 ROS2 阶段验证

2026-11-16，目标 .224：Ubuntu 22.04 ARM64 / Humble，机载代码提交 d2e419e27e6819211295c4a0a5fae4f1554108b3。

## 实际来源与操作流程

参考 .235 当前 `replan_fsm.cpp`，SHA256 `c5cc6036ea693177a80dc4adbb6f79ed66284b0b75e8703bde2058b4c8922a6a`。当前代码已停用候选工作进程和人工确认；地图选点直接更新原生目标。早期计划中“候选／确认”是历史状态，不能作为本次迁移目标。

移植包括原生状态机、单线程非递归回调、异步预设航点初始化、ROS2 参数／定时器／话题接口、手动启动 launch，以及 `map_navigation.py preview x y z` / `status`。地图工具读取状态中的 `goal_frame`，当前为 camera_init；每次请求等待同会话中新 goal_id 的回执，避免把旧目标状态当成新请求成功。

里程计使用 SensorDataQoS；预览轨迹、状态和模式开关使用 transient_local。点云默认 Reliable，与原生地图库一致。数据时间和单调接收时间共同约束输入新鲜度；过期时暂停产生新轨迹，清除到达保持状态，模拟执行碰撞检查也不再使用旧输入重规划。

## 验证结果

原生 plan_manage 构建通过。最终 `tools/validate_ego_fsm.py` 在 .224 上运行真实 planner_node，31 项检查全部通过：

- 无输入／无目标等待、有效点云和里程计后生成有限系数预览轨迹、唯一轨迹 ID。
- 错误目标坐标系与非有限坐标拒绝；直接目标更新无需确认；命令行目标回执及坐标系一致。
- 目标附近仍移动时 WAIT_STOP，静止保持后 ARRIVED；点云过期清除到达状态。
- 里程计过期、未来时间戳阻止新规划；新鲜输入恢复；停止后不再更新，新增目标恢复。
- 预览阶段执行轨迹为零；隔离模拟执行阶段产生 1 条轨迹，输入过期后不再更新，不能误报到达；切回预览正常。
- 进程正常退出。

共收到 7 条预览话题消息（含模拟执行镜像）。ROS_DOMAIN_ID=87，ROS_LOCALHOST_ONLY=1；未连接物理 MAVROS，未发送飞控指令。此结果不代表实机避障、PX4 接收／融合或飞行验证完成。

原始记录：`records/ego-fsm-build-final.log`、`records/ego-fsm-validation-final.log`。首轮编译因 ROS2 回调签名不兼容失败，已修正，原始失败日志保留供追溯。

## 手动启动与恢复

正式环境尚未切换，没有新增自启动服务。已准备 `01_apps/ego_planner/launch/manual_preview.launch.py`，默认读取 `/factory_fastlio/odometry` 和 `/factory_fastlio/cloud_registered`；连接实际设备前需先完成坐标、外参、物理定位和负载验证。

如需撤销本阶段源码，在机载工程中对 d2e419e 执行 git revert，并在独立临时构建目录重新构建 plan_manage；原生算法库基线 c33f78b 仍在历史中。没有修改 .235、PX4 参数、系统服务或雷达网络配置。

## 未完成与问题

执行适配器／会话管理、物理输入验证、地面站与任务链路仍未完成。云台网络、雷达网络和位姿转发语义仍待处理。点云未来时间戳和损坏消息需要在地图库层进一步验证；本阶段未来时间戳检查针对里程计。

恢复检查：eMMC 根分区可读写、NoMachine active、NTP 已同步。dnsmasq 开机因端口53已占用失败；另有用户态 Timed resync 报错，未更改这些服务。现有 SSH 可用，尚未完成断电后的整机外设复测。
