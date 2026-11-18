# 科研飞行平台 ROS2 命令手册

适用设备：Orange Pi 5 Ultra（192.168.1.224）  
系统：Ubuntu 22.04 + ROS2 Humble  
用户：`orangepi`

## 1. 连接与环境

电脑上连接机载电脑：

```bash
ssh orangepi@192.168.1.224
```

进入机载终端后，每个新终端先执行：

```bash
source ~/drone_platform/env.sh
export PYTHONNOUSERSITE=1
```

查看 ROS2 节点和话题：

```bash
ros2 node list
ros2 topic list
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
```

不要使用 ROS1 命令：`roscore`、`roslaunch`、`rostopic`、`rosnode`。

## 2. FAST-LIO 定位与位姿转发

查看状态：

```bash
python3 ~/drone_platform/01_apps/fast_lio/manage_stack.py status
```

启动定位和转发：

```bash
python3 ~/drone_platform/01_apps/fast_lio/manage_stack.py start
```

停止定位和转发：

```bash
python3 ~/drone_platform/01_apps/fast_lio/manage_stack.py stop
```

检查关键输出：

```bash
ros2 topic hz /factory_fastlio/odometry
ros2 topic hz /mavros/vision_pose/pose
ros2 topic echo /mavros/state --once
```

停止并重新启动定位会重建地图原点，旧地图选点和航点需要重新选择。

## 3. EGO-Planner 避障预览

启动 ROS2 规划预览：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_board.sh --start
```

查看预览状态：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_board.sh --status
```

停止预览：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_board.sh --stop
```

直接查看规划话题：

```bash
ros2 topic list | grep ego_planner
ros2 topic echo /ego_planner/manual_preview/status --once
```

EGO 的 `plan_env`、`path_searching`、`traj_opt` 等是规划库，由 `plan_manage/planner_node` 加载，不需要分别启动。

## 4. EGO-PX4 执行器

查看执行器状态：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh status
```

只在完成地面检查、确认飞控状态和控制权后使用：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh enable
```

请求受控停止：

```bash
bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh stop
```

执行器会检查飞控连接、模式、定位时效和控制权，不要同时启动航点、精准降落或人物跟踪控制器。

## 5. ArUco 视觉与精准降落

只启动 RTSP 相机和 ArUco 检测器：

```bash
ros2 launch /home/orangepi/drone_platform/01_apps/mission_demo/launch/aruco_vision.launch.py
```

单独启动精准降落控制器：

```bash
ros2 launch land_test Safina_land.launch.py
```

查看节点和检测结果：

```bash
ros2 node list | grep -E 'aruco|land'
ros2 topic list | grep -E 'aruco|land'
```

精准降落控制器可能产生飞控控制输出。当前只完成隔离链路验证，未执行真实降落任务；未经确认不要启动。

## 6. 人物跟踪

启动 RKNN 人物检测器：

```bash
ros2 launch yolo_ultra_test tracking.launch.py
```

启动人物跟踪控制器：

```bash
ros2 launch track_test Safina_track.launch.py
```

查看状态：

```bash
ros2 node list | grep -E 'yolo|track'
ros2 topic list | grep -E 'yolo|track|person'
```

检测器和跟踪控制器可以分开检查；跟踪控制器可能发布飞控控制量，不要与 EGO、航点或精准降落同时运行。

## 7. 航点任务

先验证任务文件，不启动任务：

```bash
python3 -B ~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py validate \
  --mission-file ~/drone_platform/01_apps/mission_demo/workspace/install/waypoint_mission/share/waypoint_mission/missions/abc_photo.yaml
```

查看任务状态：

```bash
python3 -B ~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py status
```

只做干运行预览：

```bash
ros2 launch waypoint_mission waypoint_mission.launch.py \
  dry_run:=true bench_mode:=true start_photo:=false start_camera:=false
```

实际任务入口（可能产生控制输出）：

```bash
python3 -B ~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py start \
  --kind waypoint \
  --mission-file ~/drone_platform/01_apps/mission_demo/workspace/install/waypoint_mission/share/waypoint_mission/missions/abc_photo.yaml
```

请求停止当前任务：

```bash
python3 -B ~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py stop
```

实际任务前必须确认飞控已落地、控制权状态正确、定位原点和航点属于当前地图会话。

## 8. 地面站

电脑上启动 ROS2 r5 地面站：

```bash
bash /home/tyzk/drone-migration-github/tools/launch_ros2_repaired_gcs.sh
```

桌面快捷方式“科研飞行平台-地面站 ROS2”已经指向上述 r5 入口。地面站连接后可查看点云、位姿、飞控状态和 EKF；不要用旧版 ROS2 地面站目录启动。

## 9. 停止顺序

建议停止顺序：

1. 停止任务控制器或执行器。
2. 停止 EGO 预览/执行器。
3. 停止 ArUco 降落或人物跟踪控制器。
4. 停止航点任务管理器。
5. 最后按需停止 FAST-LIO 定位和位姿转发。

当前雷达、双目、MAVROS、云台视频属于基础服务，不要为停止一个任务而停止这些服务。

## 10. 安全边界

- 不自动解锁、不起飞、不切换 OFFBOARD、不修改飞控参数。
- 电量和遥控器暂不纳入本阶段故障判断。
- 不同时启动多个会向 MAVROS 发布控制输出的节点。
- 看到节点已启动不等于已经完成真实飞行验收；真实任务前必须完成台架和地面检查。
