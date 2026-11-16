# ROS2定位与转发入口

当前独立副本使用`source ~/drone_platform/env.sh`，通过SSH stdin helper调用`~/drone_platform/01_apps/fast_lio/manage_stack.py`。启动按钮复用所属健康会话，不自动重启原点；没有定位时先启动原生FAST-LIO，取得有效camera_init/body新鲜输出后再启动Pose转发。转发沿用100个新测量初始化和原位姿几何，保留源时间戳，30Hz仅为上限。

停止按转发→定位顺序，只停止同一PID/start tick/boot和进程组对应的所属会话。必须已连接、未解锁、非OFFBOARD、状态新鲜且没有控制发布者、规划预览或执行器占用；不停止传感器/MAVROS/录制，不写飞控参数。

终端入口：`~/drone_platform/03_tools/config script/start_lidar_localization.sh`仅定位，`start_pose_forwarding.sh`仅转发。重复执行相应终端入口会按原流程重启该所属会话；Ctrl+C核实归属后清理，处于飞行/控制状态时保留后台定位。ROS1 remap参数不再传入ROS2管理入口，当前使用固定已审计接口。

状态仍来自只读PFS1采样。运行PID不等于定位/转发已准备就绪；新DDS来源会话清空旧累计点云/轨迹，需按当前直接地图选点流程重新准备，不恢复人工确认/候选工作流。没有新鲜采样时按钮禁用。

机载21项实际FAST-LIO回放/转发/前台生命周期、原二进制71条几何对照、Qt实际面板及24项离线策略检查通过。完整记录见`../../records/2026-11-16/localization/`。物理雷达网络/漂移/实时负载和PX4接收融合未验收；当前不切换生产或执行飞行任务。
