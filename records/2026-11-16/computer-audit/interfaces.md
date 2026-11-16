# 接口与坐标索引

运行图来自 `raw/remote_graph_final.json`；服务类型由稍后的 `raw/remote_service_types.json` 只读 probe 补充。两份证据有各自时间，不能当成同一瞬间的状态。下方源码接口表示配置/逻辑，不表示当前已在运行。

## 关键链路

| 接口 | ROS1消息/服务类型 | 本轮证据与迁移语义 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 有数据，约10Hz；保留CustomPoint单点时间，不用显示点云替代 |
| `/livox/imu` | `sensor_msgs/Imu` | 有数据，约200Hz；当前静止加速度约1，需核对算法单位 |
| `/factory_fastlio/odometry` | `nav_msgs/Odometry` | 管理器重映射输出；本轮无发布者；camera_init→body，线速度为body |
| `/factory_fastlio/cloud_world` / `cloud_body` | `sensor_msgs/PointCloud2` | 本轮无发布者；对应旧cloud_registered/cloud_registered_body |
| `/factory_fastlio/tf` | `tf2_msgs/TFMessage` | LIO专用TF输出；本轮未运行，不能假定出现在全局/tf |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | MAVROS输入，本轮无发布者；转发配置30Hz但未测得；不含速度 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 有数据，约30Hz，frame map；不能据此证明外部定位融合 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 任务代码订阅；需与LIO的body速度区别 |
| `/mavros/state`、`extended_state`、`rc/in` | `mavros_msgs/State`、`ExtendedState`、`RCIn` | 连接/模式/解锁、落地和遥控门；模式边沿可能触发任务 |
| `/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | 采样水平位置/速度有效标志false；后续需融合/创新证据 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | EGO/航点/降落/跟踪的共同控制入口；需互斥与type_mask/坐标验证；本轮未发布 |
| `/mavros/param/get` | `mavros_msgs/ParamGet` | 本轮只读调用获取14个指定参数；不调用set/push等改变状态的服务 |
| `/mavros/set_mode`、`cmd/arming` | `mavros_msgs/SetMode`、`CommandBool` | 只读取接口类型；不调用、不默认桥接 |
| `/camera/{color,depth,left_ir,right_ir}/image_raw` | `sensor_msgs/Image` | 标准订阅可取得图像；深度16UC1、IR mono8、848×480；长时性能待验收 |
| `/camera/{color,depth,left_ir,right_ir}/camera_info` | `sensor_msgs/CameraInfo` | 内参/畸变/尺寸与光学帧；几何精度未实测 |
| `/rtsp_cam/image_raw`、`image_color` | `sensor_msgs/Image` | 云台解码发布；ArUco用raw，航点拍照用color，迁移应保持 |

## 当前停用模块的源码接口

下表来自机载源码快照。实际namespace、候选ID、状态JSON字段和参数细节以对应文件为准；不把历史模板或仿真launch当作正式入口。

| 模块 | 输入 | 输出/服务 | 配置及源码 |
|---|---|---|---|
| EGO预览/执行 | LIO odom/cloud、`/ego_planner/navigation/preview_goal` PoseStamped、`/ego_planner/flight_execution` Bool、MAVROS状态 | manual_preview状态String、预览/候选/执行PolyTraj、px4_execution状态String、PositionTarget | `ego_planner/map_navigation.py`、`manual_session.py`、`px4_execution/adapter.py`、`run_planner_board.launch`；默认manual_preview=true，odom_vel_is_body=true |
| 航点/拍照 | FC pose/state/RC、LIO odom、`/waypoint_mission/photo_done` String、图像 | status/photo_trigger/photo_done String、PositionTarget；SetMode/CommandBool客户端 | `waypoint_mission.launch` remap旧/Odometry→factory；mission YAML与地面站version4协议共同定义任务 |
| ArUco | 默认`/rtsp_cam/image_raw`，可选color | `/aruco_poseresult` PoseStamped、`/aruco_target_id` Int32、metrics String、结果Image | `takeoff_land.yaml`的aruco_marker参数；保留源戳、ID/Pose同帧、光学/receipt帧 |
| 降落 | ArUco Pose、FC pose/velocity/state/extended_state | PositionTarget、aruco_landing状态/路径/目标；`/aruco_landing/enable` SetBool；模式/解锁客户端 | `Safina_land_test.cpp`、`landing_task.h`、`takeoff_land.yaml`；真实控制本轮未运行 |
| 人物检测 | Orbbec color/depth、RC/state | `/Safina_track` Float32MultiArray、`/Safina_target_id` Int32、结果Image | `yolo_track.yaml`、RKNN模型；无Header数组需要补源时间语义 |
| 人物跟踪 | Safina_track、FC pose/velocity/RC/extended_state | PositionTarget | `track.yaml`与`Safina_track_test.cpp`；保留RC门、目标超时和FRAME_BODY_NED语义 |

## 参数与坐标基线

- 完整活动参数已展开至 [ros1_parameters.json](interfaces/ros1_parameters.json) 和 [TSV](interfaces/ros1_parameters.tsv)。这是参数服务器快照；停用LIO/任务的有效配置另在源码文件中，不可只迁移活动参数。
- FAST-LIO实际配置：[lidar.yaml](snapshot/home/orangepi/drone_platform/01_apps/fast_lio/config/lidar.yaml)。软件同步true、盲区0.3m、外参固定，不能用workspace内mid360模板覆盖。
- EGO正式配置入口：[run_planner_board.launch](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/ego_planner/plan_manage/launch/realflight/run_planner_board.launch)。速度/加速度、rolling map、膨胀、虚拟墙/地面等需连同include参数迁移。
- 视觉任务配置：[takeoff_land.yaml](snapshot/home/orangepi/drone_platform/01_apps/mission_demo/src/land_test/config/takeoff_land.yaml)、[track.yaml](snapshot/home/orangepi/drone_platform/01_apps/mission_demo/src/track_test/config/track.yaml)、[yolo_track.yaml](snapshot/home/orangepi/drone_platform/01_apps/mission_demo/src/yoloUltra_test/config/yolo_track.yaml)。原全局/私有加载方式在ROS2改成节点参数，但数值不能静默退回默认。
- LIO `camera_init→body`；MAVROS本地位姿map，静态`map→map_ned`、`odom→odom_ned`、`base_link→base_link_frd`；Orbbec有内部camera/optical标定。完整机身/相机/LIO/FC变换和原点尚未证明。TF采样内容见 `raw/remote_sample.json`。
- 真实PX4参数单列在 [px4_parameters.json](raw/px4_parameters.json)，与ROS参数服务器不同。

## 已采集的自定义消息定义

下方列出本地保存的.msg定义及摘要，ROS2接口包应按实际使用子集迁移，并逐字段核查。生成的ROS1 Python/C++绑定不可复制进ROS2。

| 定义 | SHA256 | 副本 |
|---|---|---|
| `AuxCommand.msg` | `1c506a91ee92fcbebae7f299ed7fc1c49c4fdb969c3f7652a6608ae5fe9f5093` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/AuxCommand.msg) |
| `Bspline.msg` | `307984889d6e502b2419565b13dd579be7cb5b9d076f377b8ae2025787d2574b` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Bspline.msg) |
| `Corrections.msg` | `465b4e298d5a693dc0a03b2bf99d7fcb0a671875c68d6cb39e09704fe2329948` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Corrections.msg) |
| `Gains.msg` | `91961896927a5e51f8545066ca082851bd67b7678b7f0de294be3f542c94e9f2` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Gains.msg) |
| `GoalSet.msg` | `42a41425a41e306c9d6db14550caae64340df01eb7bbe72bfe0c324cd49f98a6` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/GoalSet.msg) |
| `LQRTrajectory.msg` | `6b2bc9ef4704cceeaf2019c1cd698cb1a52f896b46ef81457160a42b720f4f5e` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/LQRTrajectory.msg) |
| `Odometry.msg` | `9c0aded5c3cc619c7a47d9ddde865907f81b7c45841b75be9e7a5fc446980772` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Odometry.msg) |
| `OptimalTimeAllocator.msg` | `f31055801d06e0342ad8fa16e3ea097108057468d82e2386a9bba0690aafa546` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/OptimalTimeAllocator.msg) |
| `OutputData.msg` | `f4c2125a8258d048b25cb4528cbef0de3267dd91eb501df2eaecbeb6cba8bd68` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/OutputData.msg) |
| `PPROutputData.msg` | `53400dc4e6f3e8a866221af5cf35172b267df567e85cdcd664d64cbb0c8b06b0` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/PPROutputData.msg) |
| `PolynomialTrajectory.msg` | `27906a3218fa84ada0e9a22040d43313cc6317733d4fa2c9c4db13e46859c42b` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/PolynomialTrajectory.msg) |
| `PositionCommand.msg` | `350b62e13b6a141a4fc19aaa1a348ee275985c430f562ff4440416d20652961e` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/PositionCommand.msg) |
| `PositionCommand_back.msg` | `208c2fa01938ed5ecd94b7c2135d0e8f070dc2e16901fb0d97b3749f69bb0658` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/PositionCommand_back.msg) |
| `Px4ctrlDebug.msg` | `fea4914da6510fd52345b2f73a8560ac26442b8d8ba1f3a200bbd1aa3de3251b` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Px4ctrlDebug.msg) |
| `Px4ctrlState.msg` | `a9b0552dd00420512ecbce22ca43439d1456d0353afe70607e19f9501f3bc609` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Px4ctrlState.msg) |
| `Replan.msg` | `7950e9fbfe1d0ae75055770a805b7aa422b481598c642cbf8acbf476ed2fe4d0` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Replan.msg) |
| `ReplanCheck.msg` | `d95871a022dcff051a1af7cf7be96b81e60567b3546b5798e9486c8c42f38bb8` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/ReplanCheck.msg) |
| `SO3Command.msg` | `db5de254945dc6f4045819a0df809cd596a65e1fff2d6413e937c29f9b39fde1` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/SO3Command.msg) |
| `Serial.msg` | `b98a312615a71d66e5e7022adce1fdade90873e77060882c8bd22d8603f846eb` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/Serial.msg) |
| `SpatialTemporalTrajectory.msg` | `eb5269bd4c63df0ee1dfd1a5ebd71e201898de595848e7c771324470c087bdc3` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/SpatialTemporalTrajectory.msg) |
| `StatusData.msg` | `6aea60de547e5d46749e592b980e8bb4c066172140e1a0eabf113977cdda8130` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/StatusData.msg) |
| `SwarmCommand.msg` | `d00195518998f0c2582a692668f05e0dd3e47600e6df8103a708086882d531a7` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/SwarmCommand.msg) |
| `SwarmInfo.msg` | `c96fc8b456419c871bb72f674ac7f526feb4affd51531f75810ee0ac1b5a744a` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/SwarmInfo.msg) |
| `SwarmOdometry.msg` | `d10f51cd29e9ba6dfc846bf3abe0c101ab65a27f768be421b3c63dc0a8c13c3f` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/SwarmOdometry.msg) |
| `TRPYCommand.msg` | `c22e8f1db7c33de5324568143dc131d6a2b7cc9a2aad94f471c27761914a720f` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/TRPYCommand.msg) |
| `TakeoffLand.msg` | `3b49252dad12f8d5252def42295cdc09b5c688982efcf8cf0a6dfccac870cac5` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/TakeoffLand.msg) |
| `TrajectoryMatrix.msg` | `5e800783ffc8906e77407e08289a6f2bbc402303f019489a152219bf559512a4` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/Utils/quadrotor_msgs/msg/TrajectoryMatrix.msg) |
| `DataDisp.msg` | `3867c49329e36ca5c781232992a0f38d2c7aa660f86b541150bf4a6e768ce7d2` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/ego_planner/traj_utils/msg/DataDisp.msg) |
| `MINCOTraj.msg` | `763bc2dbcb7787edb470d8ebbcc4a5258e5f692195322e1e8e7c57c138f439d4` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/ego_planner/traj_utils/msg/MINCOTraj.msg) |
| `PolyTraj.msg` | `ddb60743793fd2b99fb18ba4c783b894500b91b61c80ca20b213ba50c060f135` | [源码](snapshot/home/orangepi/drone_platform/01_apps/ego_planner/src/ego_planner/traj_utils/msg/PolyTraj.msg) |
| `Pose6D.msg` | `91b5397207076295c2b20273552b80267e258a749ae1a932dc00f48cfc31334f` | [源码](snapshot/home/orangepi/drone_platform/01_apps/fast_lio/workspace/src/fast_lio/msg/Pose6D.msg) |
| `DepthFilterParam.msg` | `5f3b651ee83602eb77ea2558cf2845a5b88b622bcce8690edeee6766b63849c5` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/DepthFilterParam.msg) |
| `DepthFilterState.msg` | `cad5267f9907c6e1937b75476e78273af6c54d7f952ab61987e2edf15f867c35` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/DepthFilterState.msg) |
| `DepthFiltersStatus.msg` | `fa1cc8eb941891cb5fd0f9035495140262e0228aba90c749dbd11ee6dd3d464c` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/DepthFiltersStatus.msg) |
| `DeviceInfo.msg` | `e481a4200f0cb2dc5d0785aa50f357b8755ab2a53820288e3418f1c474b8211e` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/DeviceInfo.msg) |
| `DeviceStatus.msg` | `68c1af9f516e0abc461232ffe58cc25b8dd3ffe6d92e85c03269781c50213607` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/DeviceStatus.msg) |
| `Extrinsics.msg` | `85e08b5dbe973ceafe3e7303bd4f12c4addac8c8231340a676f27e7534b147e9` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/Extrinsics.msg) |
| `IMUInfo.msg` | `b29c420782998893e5a949f9a9c2e8455fde803238b45ad1ee6e758acd024162` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/IMUInfo.msg) |
| `Metadata.msg` | `8fd83ceb9bff302c8953d480c67aaf27a9b2810467a900b5be001568fb8516a7` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/Metadata.msg) |
| `StreamProfile.msg` | `cba886400c2bc8a4de8d7534d7e01598dc03ce2bf6e952c9eb61683767731c65` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/StreamProfile.msg) |
| `StreamStatus.msg` | `69b8ab4c376cb6ce3ac30c030e397abadc0342d05716766efbf7c75c515e34d0` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/orbbec/src/OrbbecSDK_ROS1-2-main/msg/StreamStatus.msg) |
| `CustomMsg.msg` | `f42d6709db951b1fa307e929e742c0593cbf0d1b0ff977d2ed63ad8d7cee0a96` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/livox_ros_driver2/msg/CustomMsg.msg) |
| `CustomPoint.msg` | `b64b31a8edc8c8b3765d82b5d3ccd2d2e1f217b9525ef7007ab918674c619c59` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/livox_ros_driver2/msg/CustomPoint.msg) |
| `DeviceInfo.msg` | `e481a4200f0cb2dc5d0785aa50f357b8755ab2a53820288e3418f1c474b8211e` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/orbbec_camera/msg/DeviceInfo.msg) |
| `DeviceStatus.msg` | `e3bd670939383ef50225313292960e4e8341d89a59d09c2eedd11340d9c2cafc` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/orbbec_camera/msg/DeviceStatus.msg) |
| `Extrinsics.msg` | `85e08b5dbe973ceafe3e7303bd4f12c4addac8c8231340a676f27e7534b147e9` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/orbbec_camera/msg/Extrinsics.msg) |
| `IMUInfo.msg` | `b29c420782998893e5a949f9a9c2e8455fde803238b45ad1ee6e758acd024162` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/orbbec_camera/msg/IMUInfo.msg) |
| `Metadata.msg` | `8fd83ceb9bff302c8953d480c67aaf27a9b2810467a900b5be001568fb8516a7` | [源码](snapshot/home/orangepi/drone_platform/02_drivers/sensors/share/orbbec_camera/msg/Metadata.msg) |

## 运行图完整话题清单

采集时间：2026-11-16T14:33:58.365628+08:00。205种已登记话题类型；有发布者的条目153，订阅条目70。注册发布者不表示已取得有效数据；采样结果另见raw。

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `/camera/camera` | — |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `/camera/camera` | — |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/color/metadata` | `orbbec_camera/Metadata` | `/camera/camera` | — |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | `/camera/camera` | — |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressed` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressed/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressed/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressedDepth/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/depth/image_raw/compressedDepth/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/depth/image_raw/theora` | `theora_image_transport/Packet` | `/camera/camera` | — |
| `/camera/depth/image_raw/theora/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/depth/image_raw/theora/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/depth/metadata` | `orbbec_camera/Metadata` | `/camera/camera` | — |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | `/camera/camera` | — |
| `/camera/depth_filters/status` | `orbbec_camera/DepthFiltersStatus` | `/camera/camera` | — |
| `/camera/depth_to_color` | `orbbec_camera/Extrinsics` | `/camera/camera` | — |
| `/camera/depth_to_left_ir` | `orbbec_camera/Extrinsics` | `/camera/camera` | — |
| `/camera/depth_to_right_ir` | `orbbec_camera/Extrinsics` | `/camera/camera` | — |
| `/camera/device_status` | `orbbec_camera/DeviceStatus` | `/camera/camera` | — |
| `/camera/left_ir/camera_info` | `sensor_msgs/CameraInfo` | `/camera/camera` | — |
| `/camera/left_ir/image_raw` | `sensor_msgs/Image` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressed` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressed/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressed/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressedDepth/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/compressedDepth/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/theora` | `theora_image_transport/Packet` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/theora/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/left_ir/image_raw/theora/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/left_ir/metadata` | `orbbec_camera/Metadata` | `/camera/camera` | — |
| `/camera/right_ir/camera_info` | `sensor_msgs/CameraInfo` | `/camera/camera` | — |
| `/camera/right_ir/image_raw` | `sensor_msgs/Image` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressed` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressed/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressed/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressedDepth/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/compressedDepth/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/theora` | `theora_image_transport/Packet` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/theora/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/camera/camera` | — |
| `/camera/right_ir/image_raw/theora/parameter_updates` | `dynamic_reconfigure/Config` | `/camera/camera` | — |
| `/camera/right_ir/metadata` | `orbbec_camera/Metadata` | `/camera/camera` | — |
| `/camera/sdk_version` | `std_msgs/String` | `/camera/camera` | — |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `/mavros`<br/>`/camera/camera` | — |
| `/drone_0_ego_planner_node/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | — | `/flight_desk_readonly_monitor_3765_1794802638030`<br/>`/flight_desk_readonly_monitor_19478_1794810830019` |
| `/ego_planner/manual_preview/status` | `std_msgs/String` | — | `/flight_desk_readonly_monitor_3765_1794802638030`<br/>`/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979`<br/>`/flight_desk_readonly_monitor_19478_1794810830019` |
| `/ego_planner/manual_preview/trajectory` | `traj_utils/PolyTraj` | — | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/ego_planner/navigation/candidate_trajectory` | `traj_utils/PolyTraj` | — | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/ego_planner/px4_execution/status` | `std_msgs/String` | — | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/factory_fastlio/cloud_world` | `sensor_msgs/PointCloud2` | — | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/dianyun_gcs_cloud_sender_4157_1794802638566`<br/>`/dianyun_gcs_cloud_sender_5286_1794810410845`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/factory_fastlio/odometry` | `nav_msgs/Odometry` | — | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/dianyun_gcs_cloud_sender_4157_1794802638566`<br/>`/dianyun_gcs_cloud_sender_5286_1794810410845`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/livox/imu` | `sensor_msgs/Imu` | `/livox_lidar_publisher2` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | `/livox_lidar_publisher2` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavlink/from` | `mavros_msgs/Mavlink` | `/mavros` | — |
| `/mavlink/gcs_ip` | `std_msgs/String` | `/mavros` | — |
| `/mavlink/to` | `mavros_msgs/Mavlink` | — | `/mavros` |
| `/mavros/actuator_control` | `mavros_msgs/ActuatorControl` | — | `/mavros` |
| `/mavros/adsb/send` | `mavros_msgs/ADSBVehicle` | — | `/mavros` |
| `/mavros/adsb/vehicle` | `mavros_msgs/ADSBVehicle` | `/mavros` | — |
| `/mavros/altitude` | `mavros_msgs/Altitude` | `/mavros` | — |
| `/mavros/battery` | `sensor_msgs/BatteryState` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/cam_imu_sync/cam_imu_stamp` | `mavros_msgs/CamIMUStamp` | `/mavros` | — |
| `/mavros/camera/image_captured` | `mavros_msgs/CameraImageCaptured` | `/mavros` | — |
| `/mavros/cellular_status/status` | `mavros_msgs/CellularStatus` | — | `/mavros` |
| `/mavros/companion_process/status` | `mavros_msgs/CompanionProcessStatus` | — | `/mavros` |
| `/mavros/debug_value/debug` | `mavros_msgs/DebugValue` | `/mavros` | — |
| `/mavros/debug_value/debug_float_array` | `mavros_msgs/DebugValue` | `/mavros` | — |
| `/mavros/debug_value/debug_vector` | `mavros_msgs/DebugValue` | `/mavros` | — |
| `/mavros/debug_value/named_value_float` | `mavros_msgs/DebugValue` | `/mavros` | — |
| `/mavros/debug_value/named_value_int` | `mavros_msgs/DebugValue` | `/mavros` | — |
| `/mavros/debug_value/send` | `mavros_msgs/DebugValue` | — | `/mavros` |
| `/mavros/esc_info` | `mavros_msgs/ESCInfo` | `/mavros` | — |
| `/mavros/esc_status` | `mavros_msgs/ESCStatus` | `/mavros` | — |
| `/mavros/esc_telemetry` | `mavros_msgs/ESCTelemetry` | `/mavros` | — |
| `/mavros/estimator_status` | `mavros_msgs/EstimatorStatus` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/fake_gps/mocap/tf` | `geometry_msgs/TransformStamped` | — | `/mavros` |
| `/mavros/geofence/waypoints` | `mavros_msgs/WaypointList` | `/mavros` | — |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | `/mavros` | — |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `/mavros` | `/mavros` |
| `/mavros/global_position/gp_lp_offset` | `geometry_msgs/PoseStamped` | `/mavros` | — |
| `/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | `/mavros` | `/mavros` |
| `/mavros/global_position/local` | `nav_msgs/Odometry` | `/mavros` | — |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | `/mavros` | — |
| `/mavros/global_position/raw/gps_vel` | `geometry_msgs/TwistStamped` | `/mavros` | — |
| `/mavros/global_position/raw/satellites` | `std_msgs/UInt32` | `/mavros` | — |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | `/mavros` | — |
| `/mavros/global_position/set_gp_origin` | `geographic_msgs/GeoPointStamped` | — | `/mavros` |
| `/mavros/gps_input/gps_input` | `mavros_msgs/GPSINPUT` | — | `/mavros` |
| `/mavros/gps_rtk/rtk_baseline` | `mavros_msgs/RTKBaseline` | `/mavros` | — |
| `/mavros/gps_rtk/send_rtcm` | `mavros_msgs/RTCM` | — | `/mavros` |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | `/mavros` | — |
| `/mavros/gpsstatus/gps1/rtk` | `mavros_msgs/GPSRTK` | `/mavros` | — |
| `/mavros/gpsstatus/gps2/raw` | `mavros_msgs/GPSRAW` | `/mavros` | — |
| `/mavros/gpsstatus/gps2/rtk` | `mavros_msgs/GPSRTK` | `/mavros` | — |
| `/mavros/hil/actuator_controls` | `mavros_msgs/HilActuatorControls` | `/mavros` | — |
| `/mavros/hil/controls` | `mavros_msgs/HilControls` | `/mavros` | — |
| `/mavros/hil/gps` | `mavros_msgs/HilGPS` | — | `/mavros` |
| `/mavros/hil/imu_ned` | `mavros_msgs/HilSensor` | — | `/mavros` |
| `/mavros/hil/optical_flow` | `mavros_msgs/OpticalFlowRad` | — | `/mavros` |
| `/mavros/hil/rc_inputs` | `mavros_msgs/RCIn` | — | `/mavros` |
| `/mavros/hil/state` | `mavros_msgs/HilStateQuaternion` | — | `/mavros` |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | `/mavros` | `/mavros` |
| `/mavros/home_position/set` | `mavros_msgs/HomePosition` | — | `/mavros` |
| `/mavros/imu/data` | `sensor_msgs/Imu` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | `/mavros` | — |
| `/mavros/imu/diff_pressure` | `sensor_msgs/FluidPressure` | `/mavros` | — |
| `/mavros/imu/mag` | `sensor_msgs/MagneticField` | `/mavros` | — |
| `/mavros/imu/static_pressure` | `sensor_msgs/FluidPressure` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/imu/temperature_baro` | `sensor_msgs/Temperature` | `/mavros` | — |
| `/mavros/imu/temperature_imu` | `sensor_msgs/Temperature` | `/mavros` | — |
| `/mavros/landing_target/lt_marker` | `geometry_msgs/Vector3Stamped` | `/mavros` | — |
| `/mavros/landing_target/pose` | `geometry_msgs/PoseStamped` | — | `/mavros` |
| `/mavros/landing_target/pose_in` | `geometry_msgs/PoseStamped` | `/mavros` | — |
| `/mavros/local_position/accel` | `geometry_msgs/AccelWithCovarianceStamped` | `/mavros` | — |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `/mavros` | `/mavros` |
| `/mavros/local_position/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | `/mavros` | — |
| `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | `/mavros` | — |
| `/mavros/local_position/velocity_body_cov` | `geometry_msgs/TwistWithCovarianceStamped` | `/mavros` | — |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `/mavros` | — |
| `/mavros/log_transfer/raw/log_data` | `mavros_msgs/LogData` | `/mavros` | — |
| `/mavros/log_transfer/raw/log_entry` | `mavros_msgs/LogEntry` | `/mavros` | — |
| `/mavros/mag_calibration/report` | `mavros_msgs/MagnetometerReporter` | `/mavros` | — |
| `/mavros/mag_calibration/status` | `std_msgs/UInt8` | `/mavros` | — |
| `/mavros/manual_control/control` | `mavros_msgs/ManualControl` | `/mavros` | — |
| `/mavros/manual_control/send` | `mavros_msgs/ManualControl` | — | `/mavros` |
| `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | `/mavros` | — |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | `/mavros` | — |
| `/mavros/mocap/pose` | `geometry_msgs/PoseStamped` | — | `/mavros` |
| `/mavros/mount_control/command` | `mavros_msgs/MountControl` | — | `/mavros` |
| `/mavros/mount_control/orientation` | `geometry_msgs/Quaternion` | `/mavros` | — |
| `/mavros/mount_control/status` | `geometry_msgs/Vector3Stamped` | `/mavros` | — |
| `/mavros/nav_controller_output` | `mavros_msgs/NavControllerOutput` | `/mavros` | — |
| `/mavros/obstacle/send` | `sensor_msgs/LaserScan` | — | `/mavros` |
| `/mavros/odometry/in` | `nav_msgs/Odometry` | `/mavros` | — |
| `/mavros/odometry/out` | `nav_msgs/Odometry` | — | `/mavros` |
| `/mavros/onboard_computer/status` | `mavros_msgs/OnboardComputerStatus` | — | `/mavros` |
| `/mavros/param/param_value` | `mavros_msgs/Param` | `/mavros` | — |
| `/mavros/play_tune` | `mavros_msgs/PlayTuneV2` | — | `/mavros` |
| `/mavros/px4flow/ground_distance` | `sensor_msgs/Range` | `/mavros` | — |
| `/mavros/px4flow/raw/optical_flow_rad` | `mavros_msgs/OpticalFlowRad` | `/mavros` | — |
| `/mavros/px4flow/raw/send` | `mavros_msgs/OpticalFlowRad` | — | `/mavros` |
| `/mavros/px4flow/temperature` | `sensor_msgs/Temperature` | `/mavros` | — |
| `/mavros/radio_status` | `mavros_msgs/RadioStatus` | `/mavros` | — |
| `/mavros/rallypoint/waypoints` | `mavros_msgs/WaypointList` | `/mavros` | — |
| `/mavros/rc/in` | `mavros_msgs/RCIn` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/rc/out` | `mavros_msgs/RCOut` | `/mavros` | — |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | — | `/mavros` |
| `/mavros/setpoint_accel/accel` | `geometry_msgs/Vector3Stamped` | — | `/mavros` |
| `/mavros/setpoint_attitude/cmd_vel` | `geometry_msgs/TwistStamped` | — | `/mavros` |
| `/mavros/setpoint_attitude/thrust` | `mavros_msgs/Thrust` | — | `/mavros` |
| `/mavros/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | — | `/mavros` |
| `/mavros/setpoint_position/global_to_local` | `geographic_msgs/GeoPoseStamped` | — | `/mavros` |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | — | `/mavros` |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | — | `/mavros` |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | — | `/mavros` |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | — | `/mavros` |
| `/mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` | `/mavros` | — |
| `/mavros/setpoint_raw/target_global` | `mavros_msgs/GlobalPositionTarget` | `/mavros` | — |
| `/mavros/setpoint_raw/target_local` | `mavros_msgs/PositionTarget` | `/mavros` | — |
| `/mavros/setpoint_trajectory/desired` | `nav_msgs/Path` | `/mavros` | — |
| `/mavros/setpoint_trajectory/local` | `trajectory_msgs/MultiDOFJointTrajectory` | — | `/mavros` |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | — | `/mavros` |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | — | `/mavros` |
| `/mavros/state` | `mavros_msgs/State` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/statustext/recv` | `mavros_msgs/StatusText` | `/mavros` | `/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/statustext/send` | `mavros_msgs/StatusText` | — | `/mavros` |
| `/mavros/sys_status` | `mavros_msgs/SysStatus` | `/mavros` | — |
| `/mavros/target_actuator_control` | `mavros_msgs/ActuatorControl` | `/mavros` | — |
| `/mavros/terrain/report` | `mavros_msgs/TerrainReport` | `/mavros` | — |
| `/mavros/time_reference` | `sensor_msgs/TimeReference` | `/mavros` | — |
| `/mavros/timesync_status` | `mavros_msgs/TimesyncStatus` | `/mavros` | — |
| `/mavros/trajectory/desired` | `mavros_msgs/Trajectory` | `/mavros` | — |
| `/mavros/trajectory/generated` | `mavros_msgs/Trajectory` | — | `/mavros` |
| `/mavros/trajectory/path` | `nav_msgs/Path` | — | `/mavros` |
| `/mavros/tunnel/in` | `mavros_msgs/Tunnel` | — | `/mavros` |
| `/mavros/tunnel/out` | `mavros_msgs/Tunnel` | `/mavros` | — |
| `/mavros/vfr_hud` | `mavros_msgs/VFR_HUD` | `/mavros` | — |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | — | `/mavros`<br/>`/flight_desk_readonly_monitor_3766_1794802638013`<br/>`/flight_desk_readonly_monitor_19248_1794810824979` |
| `/mavros/vision_pose/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | — | `/mavros` |
| `/mavros/vision_speed/speed_twist_cov` | `geometry_msgs/TwistWithCovarianceStamped` | — | `/mavros` |
| `/mavros/wind_estimation` | `geometry_msgs/TwistWithCovarianceStamped` | `/mavros` | — |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | `/mavros` | — |
| `/rosout` | `rosgraph_msgs/Log` | `/mavros`<br/>`/camera/camera`<br/>`/usb_cam`<br/>`/livox_lidar_publisher2`<br/>`/rtsp_hw_pub_ultra` | `/rosout` |
| `/rosout_agg` | `rosgraph_msgs/Log` | `/rosout` | — |
| `/rtsp_cam/image_color` | `sensor_msgs/Image` | `/rtsp_hw_pub_ultra` | — |
| `/rtsp_cam/image_raw` | `sensor_msgs/Image` | `/rtsp_hw_pub_ultra` | — |
| `/tf` | `tf2_msgs/TFMessage` | `/mavros`<br/>`/camera/camera` | `/mavros` |
| `/tf_static` | `tf2_msgs/TFMessage` | `/mavros`<br/>`/camera/camera` | `/mavros` |
| `/usb_cam/camera_info` | `sensor_msgs/CameraInfo` | `/usb_cam` | — |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressed` | `sensor_msgs/CompressedImage` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressed/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressed/parameter_updates` | `dynamic_reconfigure/Config` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressedDepth/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/usb_cam` | — |
| `/usb_cam/image_raw/compressedDepth/parameter_updates` | `dynamic_reconfigure/Config` | `/usb_cam` | — |
| `/usb_cam/image_raw/theora` | `theora_image_transport/Packet` | `/usb_cam` | — |
| `/usb_cam/image_raw/theora/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | `/usb_cam` | — |
| `/usb_cam/image_raw/theora/parameter_updates` | `dynamic_reconfigure/Config` | `/usb_cam` | — |

## 服务完整清单

图快照登记178个服务；类型握手时间2026-11-16T14:41:42.763637+08:00，当时172个接口全部取得类型。中间有6个临时监视/传输节点的日志服务退出；表中明确标识，未把两个时间点合并为一个运行事实。除了另存的ParamGet读取，本轮没有调用这些服务。

| 服务 | 类型（后续probe） | 图快照提供者 |
|---|---|---|
| `/camera/camera/get_loggers` | `roscpp/GetLoggers` | `/camera/camera` |
| `/camera/camera/set_logger_level` | `roscpp/SetLoggerLevel` | `/camera/camera` |
| `/camera/depth/image_raw/compressed/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/depth/image_raw/compressedDepth/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/depth/image_raw/theora/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/export_config_json` | `orbbec_camera/SetString` | `/camera/camera` |
| `/camera/get_auto_white_balance` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_camera_params` | `orbbec_camera/GetCameraParams` | `/camera/camera` |
| `/camera/get_color_auto_exposure` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_color_camera_info` | `orbbec_camera/GetCameraInfo` | `/camera/camera` |
| `/camera/get_color_exposure` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_color_gain` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_color_queue_stats` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/get_depth_auto_exposure` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_depth_camera_info` | `orbbec_camera/GetCameraInfo` | `/camera/camera` |
| `/camera/get_depth_exposure` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_depth_gain` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_device_config` | `orbbec_camera/GetDeviceConfig` | `/camera/camera` |
| `/camera/get_device_info` | `orbbec_camera/GetDeviceInfo` | `/camera/camera` |
| `/camera/get_device_type` | `orbbec_camera/GetString` | `/camera/camera` |
| `/camera/get_laser_status` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_ldp_status` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_left_ir_auto_exposure` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_left_ir_camera_info` | `orbbec_camera/GetCameraInfo` | `/camera/camera` |
| `/camera/get_left_ir_exposure` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_left_ir_gain` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_lrm_measure_distance` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_point_cloud_decimation` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_right_ir_auto_exposure` | `orbbec_camera/GetBool` | `/camera/camera` |
| `/camera/get_right_ir_camera_info` | `orbbec_camera/GetCameraInfo` | `/camera/camera` |
| `/camera/get_right_ir_exposure` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_right_ir_gain` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/get_sdk_version` | `orbbec_camera/GetString` | `/camera/camera` |
| `/camera/get_serial` | `orbbec_camera/GetString` | `/camera/camera` |
| `/camera/get_white_balance` | `orbbec_camera/GetInt32` | `/camera/camera` |
| `/camera/left_ir/image_raw/compressed/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/left_ir/image_raw/compressedDepth/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/left_ir/image_raw/theora/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/reboot_device` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_color_exposure` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_color_gain` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_depth_exposure` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_depth_gain` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_left_ir_exposure` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_left_ir_gain` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_right_ir_exposure` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_right_ir_gain` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/reset_white_balance` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/right_ir/image_raw/compressed/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/right_ir/image_raw/compressedDepth/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/right_ir/image_raw/theora/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/camera/camera` |
| `/camera/save_images` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/save_point_cloud` | `std_srvs/Empty` | `/camera/camera` |
| `/camera/send_software_trigger` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_ae_reference_stream` | `orbbec_camera/SetString` | `/camera/camera` |
| `/camera/set_ae_strategy` | `orbbec_camera/SetString` | `/camera/camera` |
| `/camera/set_auto_white_balance` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_bag_recording` | `orbbec_camera/SetBagRecording` | `/camera/camera` |
| `/camera/set_color_ae_roi` | `orbbec_camera/SetArrays` | `/camera/camera` |
| `/camera/set_color_auto_exposure` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_color_exposure` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_color_flip` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_color_gain` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_color_mirror` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_color_rotation` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_depth_ae_roi` | `orbbec_camera/SetArrays` | `/camera/camera` |
| `/camera/set_depth_auto_exposure` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_depth_exposure` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_depth_flip` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_depth_gain` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_depth_mirror` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_depth_rotation` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_disparity_range_mode` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_disparity_search_offset` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_filter` | `orbbec_camera/SetFilter` | `/camera/camera` |
| `/camera/set_image_registration_mode` | `orbbec_camera/SetString` | `/camera/camera` |
| `/camera/set_laser` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_ldp` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_left_ir_ae_roi` | `orbbec_camera/SetArrays` | `/camera/camera` |
| `/camera/set_left_ir_auto_exposure` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_left_ir_exposure` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_left_ir_flip` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_left_ir_gain` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_left_ir_mirror` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_left_ir_rotation` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_point_cloud_decimation` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_read_customer_data` | `orbbec_camera/GetString` | `/camera/camera` |
| `/camera/set_right_ir_ae_roi` | `orbbec_camera/SetArrays` | `/camera/camera` |
| `/camera/set_right_ir_auto_exposure` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_right_ir_exposure` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_right_ir_flip` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_right_ir_gain` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_right_ir_mirror` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/set_right_ir_rotation` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_stream_profile` | `orbbec_camera/SetStreamProfile` | `/camera/camera` |
| `/camera/set_white_balance` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/set_write_customer_data` | `orbbec_camera/SetString` | `/camera/camera` |
| `/camera/switch_ir_mode` | `orbbec_camera/SetInt32` | `/camera/camera` |
| `/camera/toggle_color` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/toggle_depth` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/toggle_left_ir` | `std_srvs/SetBool` | `/camera/camera` |
| `/camera/toggle_right_ir` | `std_srvs/SetBool` | `/camera/camera` |
| `/dianyun_gcs_cloud_sender_4157_1794802638566/get_loggers` | `roscpp/GetLoggers` | `/dianyun_gcs_cloud_sender_4157_1794802638566` |
| `/dianyun_gcs_cloud_sender_4157_1794802638566/set_logger_level` | `roscpp/SetLoggerLevel` | `/dianyun_gcs_cloud_sender_4157_1794802638566` |
| `/dianyun_gcs_cloud_sender_5286_1794810410845/get_loggers` | `后续快照未登记，类型未取得` | `/dianyun_gcs_cloud_sender_5286_1794810410845` |
| `/dianyun_gcs_cloud_sender_5286_1794810410845/set_logger_level` | `后续快照未登记，类型未取得` | `/dianyun_gcs_cloud_sender_5286_1794810410845` |
| `/flight_desk_readonly_monitor_19248_1794810824979/get_loggers` | `后续快照未登记，类型未取得` | `/flight_desk_readonly_monitor_19248_1794810824979` |
| `/flight_desk_readonly_monitor_19248_1794810824979/set_logger_level` | `后续快照未登记，类型未取得` | `/flight_desk_readonly_monitor_19248_1794810824979` |
| `/flight_desk_readonly_monitor_19478_1794810830019/get_loggers` | `后续快照未登记，类型未取得` | `/flight_desk_readonly_monitor_19478_1794810830019` |
| `/flight_desk_readonly_monitor_19478_1794810830019/set_logger_level` | `后续快照未登记，类型未取得` | `/flight_desk_readonly_monitor_19478_1794810830019` |
| `/flight_desk_readonly_monitor_3765_1794802638030/get_loggers` | `roscpp/GetLoggers` | `/flight_desk_readonly_monitor_3765_1794802638030` |
| `/flight_desk_readonly_monitor_3765_1794802638030/set_logger_level` | `roscpp/SetLoggerLevel` | `/flight_desk_readonly_monitor_3765_1794802638030` |
| `/flight_desk_readonly_monitor_3766_1794802638013/get_loggers` | `roscpp/GetLoggers` | `/flight_desk_readonly_monitor_3766_1794802638013` |
| `/flight_desk_readonly_monitor_3766_1794802638013/set_logger_level` | `roscpp/SetLoggerLevel` | `/flight_desk_readonly_monitor_3766_1794802638013` |
| `/livox_lidar_publisher2/get_loggers` | `roscpp/GetLoggers` | `/livox_lidar_publisher2` |
| `/livox_lidar_publisher2/set_logger_level` | `roscpp/SetLoggerLevel` | `/livox_lidar_publisher2` |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | `/mavros` |
| `/mavros/cmd/command` | `mavros_msgs/CommandLong` | `/mavros` |
| `/mavros/cmd/command_ack` | `mavros_msgs/CommandAck` | `/mavros` |
| `/mavros/cmd/command_int` | `mavros_msgs/CommandInt` | `/mavros` |
| `/mavros/cmd/land` | `mavros_msgs/CommandTOL` | `/mavros` |
| `/mavros/cmd/set_home` | `mavros_msgs/CommandHome` | `/mavros` |
| `/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | `/mavros` |
| `/mavros/cmd/trigger_control` | `mavros_msgs/CommandTriggerControl` | `/mavros` |
| `/mavros/cmd/trigger_interval` | `mavros_msgs/CommandTriggerInterval` | `/mavros` |
| `/mavros/cmd/vtol_transition` | `mavros_msgs/CommandVtolTransition` | `/mavros` |
| `/mavros/ftp/checksum` | `mavros_msgs/FileChecksum` | `/mavros` |
| `/mavros/ftp/close` | `mavros_msgs/FileClose` | `/mavros` |
| `/mavros/ftp/list` | `mavros_msgs/FileList` | `/mavros` |
| `/mavros/ftp/mkdir` | `mavros_msgs/FileMakeDir` | `/mavros` |
| `/mavros/ftp/open` | `mavros_msgs/FileOpen` | `/mavros` |
| `/mavros/ftp/read` | `mavros_msgs/FileRead` | `/mavros` |
| `/mavros/ftp/remove` | `mavros_msgs/FileRemove` | `/mavros` |
| `/mavros/ftp/rename` | `mavros_msgs/FileRename` | `/mavros` |
| `/mavros/ftp/reset` | `std_srvs/Empty` | `/mavros` |
| `/mavros/ftp/rmdir` | `mavros_msgs/FileRemoveDir` | `/mavros` |
| `/mavros/ftp/truncate` | `mavros_msgs/FileTruncate` | `/mavros` |
| `/mavros/ftp/write` | `mavros_msgs/FileWrite` | `/mavros` |
| `/mavros/geofence/clear` | `mavros_msgs/WaypointClear` | `/mavros` |
| `/mavros/geofence/pull` | `mavros_msgs/WaypointPull` | `/mavros` |
| `/mavros/geofence/push` | `mavros_msgs/WaypointPush` | `/mavros` |
| `/mavros/get_loggers` | `roscpp/GetLoggers` | `/mavros` |
| `/mavros/home_position/req_update` | `std_srvs/Trigger` | `/mavros` |
| `/mavros/log_transfer/raw/log_request_data` | `mavros_msgs/LogRequestData` | `/mavros` |
| `/mavros/log_transfer/raw/log_request_end` | `mavros_msgs/LogRequestEnd` | `/mavros` |
| `/mavros/log_transfer/raw/log_request_erase` | `std_srvs/Trigger` | `/mavros` |
| `/mavros/log_transfer/raw/log_request_list` | `mavros_msgs/LogRequestList` | `/mavros` |
| `/mavros/mission/clear` | `mavros_msgs/WaypointClear` | `/mavros` |
| `/mavros/mission/pull` | `mavros_msgs/WaypointPull` | `/mavros` |
| `/mavros/mission/push` | `mavros_msgs/WaypointPush` | `/mavros` |
| `/mavros/mission/set_current` | `mavros_msgs/WaypointSetCurrent` | `/mavros` |
| `/mavros/mount_control/configure` | `mavros_msgs/MountConfigure` | `/mavros` |
| `/mavros/param/get` | `mavros_msgs/ParamGet` | `/mavros` |
| `/mavros/param/pull` | `mavros_msgs/ParamPull` | `/mavros` |
| `/mavros/param/push` | `mavros_msgs/ParamPush` | `/mavros` |
| `/mavros/param/set` | `mavros_msgs/ParamSet` | `/mavros` |
| `/mavros/rallypoint/clear` | `mavros_msgs/WaypointClear` | `/mavros` |
| `/mavros/rallypoint/pull` | `mavros_msgs/WaypointPull` | `/mavros` |
| `/mavros/rallypoint/push` | `mavros_msgs/WaypointPush` | `/mavros` |
| `/mavros/set_logger_level` | `roscpp/SetLoggerLevel` | `/mavros` |
| `/mavros/set_message_interval` | `mavros_msgs/MessageInterval` | `/mavros` |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | `/mavros` |
| `/mavros/set_stream_rate` | `mavros_msgs/StreamRate` | `/mavros` |
| `/mavros/setpoint_position/mav_frame` | `mavros_msgs/SetMavFrame` | `/mavros` |
| `/mavros/setpoint_trajectory/mav_frame` | `mavros_msgs/SetMavFrame` | `/mavros` |
| `/mavros/setpoint_trajectory/reset` | `std_srvs/Trigger` | `/mavros` |
| `/mavros/setpoint_velocity/mav_frame` | `mavros_msgs/SetMavFrame` | `/mavros` |
| `/mavros/vehicle_info_get` | `mavros_msgs/VehicleInfoGet` | `/mavros` |
| `/rosout/get_loggers` | `roscpp/GetLoggers` | `/rosout` |
| `/rosout/set_logger_level` | `roscpp/SetLoggerLevel` | `/rosout` |
| `/rtsp_hw_pub_ultra/get_loggers` | `roscpp/GetLoggers` | `/rtsp_hw_pub_ultra` |
| `/rtsp_hw_pub_ultra/set_logger_level` | `roscpp/SetLoggerLevel` | `/rtsp_hw_pub_ultra` |
| `/usb_cam/get_loggers` | `roscpp/GetLoggers` | `/usb_cam` |
| `/usb_cam/image_raw/compressed/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/usb_cam` |
| `/usb_cam/image_raw/compressedDepth/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/usb_cam` |
| `/usb_cam/image_raw/theora/set_parameters` | `dynamic_reconfigure/Reconfigure` | `/usb_cam` |
| `/usb_cam/set_camera_info` | `sensor_msgs/SetCameraInfo` | `/usb_cam` |
| `/usb_cam/set_logger_level` | `roscpp/SetLoggerLevel` | `/usb_cam` |
