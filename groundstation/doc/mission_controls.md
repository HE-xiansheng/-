# 航点、精准降落、人物跟踪入口

> ROS2迁移副本：当前源文件为平台`platform/01_apps/mission_demo/tools/gcs_task_manager.py`，机载入口仍为`~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py`，运行前source `~/drone_platform/env.sh`。本文下面的`tools/missions/task_manager.py`、roslaunch、Noetic及.235部署记录属于ROS1历史参考，禁止据此覆盖.224原生ROS2入口。当前部署与验证边界见[ROS2部署说明](ros2_deployment.md)。


地面站现有三个功能栏已连接机载独立工作空间 `~/drone_platform/01_apps/mission_demo`。任务入口包含进程管理、只读状态、地图航点编辑。原有遥控触发保留，航点原生状态机增加可选manual_airborne模式；没有真实起飞验收。新版已于2026-10-30部署至机载192.168.1.235，原生航点编译、地图任务dry_run、ArUco启动参数和上位机SSH只读监视通过。未做真实飞行验收。

## 界面操作

2026-10-30 上升限速调整（本地修改，尚未部署机载）：新版地图任务生成`max_z_speed_up: 0.5`，起飞、航点切换及返航中的上升指令上限为0.5m/s，接近目标会减速。下降继续使用现场`max_z_speed`（已有配置快照为0.25m/s），水平速度继续沿用现场配置。AUTO.LAND由飞控控制，不受此上升限速修改影响，飞控参数未修改。旧YAML不声明`max_z_speed_up`时，上升与下降仍共同使用原`max_z_speed`。管理器必须从原生dry-run输出确认新上升限速，否则拒绝准备，不能只替换Python辅助程序。

启用此次调整需要成套部署`tools/gcs_task_manager.py`与重新编译的`waypoint_mission`，先核对实时地面锁桨状态、活动任务和控制发布者并保留备份。当前SSH受环境权限限制，未完成机载部署或实飞验证。部署后下次重新准备任务才加载新快照，已准备或运行的任务不热更新；现有version=4地面站无需因本次速度调整重启或重新选点，定位会话变化时仍需重新选点。

2026-10-30 更新：地图航点任务已改为遥控OFFBOARD边沿后自动解锁起飞到首点高度。已部署并完成隔离状态机及解析验证，尚未进行真实飞行验收。地图请求协议版本为2；旧地面站的版本1会被拒绝，需重启为新版并重新编辑航点。

实时启动、机载连接认证成功或展开“航点规划”/“视觉控制”时，仅启动只读任务监视，约1Hz通过SSH回传。4秒未收到监视包则显示过期，ArUco偏移和ID不继续显示旧值；关闭地面站只关闭监视，已经准备或运行的机载任务继续由原节点和遥控器管理。

- **航点规划**：连接机载，启动FAST-LIO定位与转发。展开栏目，设置选点高度，点击“地图添加航点”后按顺序点击地图，可连续添加；点击“完成地图选点”。每点可编辑高度、是否拍照，也可删除/清空；黄色折线和圆点为本次草稿。每点hold固定2秒，拍照云台指令固定-90度。高度以FAST-LIO定位原点计，不是对地高度。地面锁桨、遥控处于POSCTL时，点击“航点任务执行”上传任务并准备节点；准备本身不解锁或起飞。之后遥控切至OFFBOARD的新边沿触发预发送、OFFBOARD请求和自动解锁，机体保持起飞位置XY垂直升至第一个航点的高度，稳定后再飞向第一航点并依次执行。末点停留及拍照完成后，以首点高度返回起飞位置，稳定后请求 AUTO.LAND 降落，等待飞控落地并锁桨。地图任务不含Ego避障。
- 地图航点经SSH标准输入JSON上传，不经过Ego预览/GOL1或RTE1。机载用同步FAST-LIO里程计和MAVROS位姿做完整刚体坐标对齐，生成本次独立YAML快照；保留现场abc_photo.yaml中的速度、围栏、高度范围、超时和RC映射，由原生解析器校验，原示例文件不改。上传时验证定位进程身份，避免在定位重启、原点改变后执行旧地图航点。源地图请求、转换位姿、实际执行YAML均留档。地图任务使用manual_airborne=false，并把坐标转换后的第一航点Z写入takeoff_z；原生dry-run必须确认模式，首点高度必须高于准备时的飞控本地高度，且准备时必须未解锁。原YAML文件入口仍可通过辅助程序使用，未声明manual_airborne的任务保留自动起飞行为。
- **ArUco码识别降落**：连接机载、启动定位，POSCTL人工飞到标记上方并稳定悬停；展开“视觉控制→ArUco码识别降落”，点击“准备精准降落”。新版准备时立即发送云台朝下90度指令，飞行控制仍等待原生OFFBOARD新边沿；核对识别ID和X/Y偏移，再由遥控器切OFFBOARD触发降落。准备不会自动解锁或起飞。X/Y为相机坐标下米制偏移，不是地图XY。尚未完成真实标记、相机标定、安装偏移及实飞验收。
- **人物跟踪**：“准备跟踪控制”启动新工作空间的YOLO检测和track控制；手动起飞、切OFFBOARD后按原SWA档位跟随/保护/下降。界面显示目标ID及有效测距。原节点启动包含初始化位姿预发送，不是只读识别。人物跟踪实机验证仍待完成，源相机内外参必须针对目标机核对。

三种任务各自有“停止”。若飞机仍已解锁且处于OFFBOARD，停止工作进程保持等待，不直接杀控制器；请先遥控接管。获得接管状态并稳定3秒后，才对本入口拥有的独立进程组发SIGINT并核实退出。断开或关闭上位机不会撤销已提交的停止请求，独立工作进程继续等待接管。停止任务不等于降落、锁桨或电机急停，失去飞控状态时也不会假装已停止。

Ego与这三种控制任务不能同时接入。机载管理器准备新任务前报告已有MAVROS设定点发布者，不停止未知控制器。地面站在任务管理操作或本入口任务运行期间，不再从Ego入口启动/更新执行目标；任务结束后恢复。此项仅用于多控制入口协作，不恢复以前删除的候选令牌、地图边界、到点后才能选点等Ego附加检查。

## 机载辅助程序与部署

本次部署位置：

```
~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py
```

源文件为仓库 `tools/missions/task_manager.py`，以独立文件新增，地图任务还需部署仓库tools/missions/onboard/waypoint_mission.cpp到机载同名源文件并重新编译waypoint_mission；不改变共享传感器服务或飞控参数。其他机载电脑需要先安装同一辅助程序和对应ROS工作空间。SSH凭据复用“机载通信”，Windows沿用既有密钥登录要求。

操作命令均需source ROS及此工作空间环境；上位机会自动完成。`status`、`watch`只订阅数据，不调用飞控服务或发布设定点；`validate`运行原生 `dry_run`。

```
python3 tools/gcs_task_manager.py status
python3 tools/gcs_task_manager.py watch
python3 tools/gcs_task_manager.py validate --mission-file /absolute/path/mission.yaml
python3 tools/gcs_task_manager.py start --kind waypoint --mission-file /absolute/path/mission.yaml
python3 tools/gcs_task_manager.py start --kind aruco
python3 tools/gcs_task_manager.py start --kind tracking
python3 tools/gcs_task_manager.py stop
# 地图任务：stdin为version=4/frame/map_session/waypoints的JSON请求
python3 tools/gcs_task_manager.py start-map < map-request.json
```

`start`为真实准备操作，可能使遥控器后续触发任务；不要将其当作只读测试命令。同一任务重复准备是幂等的，不创建第二份；切换任务需先结束当前任务。已有外部ArUco或 `/Safina_track` 检测节点时返回明确原因，避免覆盖未知节点。

状态与日志保存在 `~/.cache/gcs-flight-tasks`，`active.json`记录PID、进程创建时间、boot_id和任务目录。任务状态记录原子替换并fsync；根据PID身份与独立会话核对进程，不只凭旧记录判断存活。`runs/<时间-随机ID>/task.launch`、`task.log`、`stop.log`和航点任务快照用于追踪。

这套任务识别输出 `/aruco_imgresult`、`/yoloUltra_imgresult`。任务监视确认任务运行后，视频面板分别自动选择已开启的云台 ArUco 画面或已开启的双目跟踪画面；任务结束后恢复先前画面。视频面板的“人物识别”开关仍是另一套 `/manual_person_follow/image` 只监视链路，不是跟踪控制。

## 验证范围

- Python隔离回归使用假的roslaunch进程，不连接飞控，覆盖任务互斥、幂等、错误配置、进程身份、停止等待接管、任务快照、独立dry_run节点名等。
- Qt测试加载实际 `MissionControls.qml`，通过假SSH点击三个准备入口与停止、核对含引号路径传参、遥测显示、过期隐藏及Ego入口互斥。
- 完整地面站构建与离屏启动检查通过；离屏环境不验证真实3D显卡显示。
- 本次机载验收仅调用真实 `status`，确认已连接、未解锁、没有本入口运行任务。没有启动真实任务，没有验证自动起飞或降落。

## 地图任务上一版验证（2026-10-29，手动起飞模式历史记录）

19项Python隔离回归通过，含90度旋转/非零平移的坐标变换、2秒停留/拍照参数、冻结执行快照、旧地图身份和旧程序兼容性拒绝。实际QML按钮通过假SSH验证地图编辑、上传JSON与停止。真实航点C++编译通过，并在私有localhost ROS master、模拟MAVROS与假GCU下验证：空中模式不会从地面自动起飞；遥控触发前不输出；触发后不解锁、不起飞；到点停留2秒，发送向下90度拍照指令；末点悬停，接管后停止设定点；旧YAML自动起飞流程兼容。未在实机执行这些验证。

补充精准降落准备检查：实际机载降落源代码与假GCU链接，在私有ROS master加载实际参数及管理器生成的云台准备覆盖项，确认准备即下转90度，任务保持IDLE/未启用且无飞行输出；新的遥控OFFBOARD边沿后才启用，POSCTL接管后停止设定点。只验证软件边界，不代表实机云台及飞行已验收。

## 2026-10-30上一版机载部署记录（手动起飞模式）

机载源文件及辅助程序与本地发布SHA一致，waypoint_mission已在ARM机载重新编译；真实原生dry_run确认支持manual_airborne。ArUco参数解析确认gimbal_down_on_startup=true、start_trigger=rc_offboard、auto_takeoff=false，现有降落二进制包含云台准备处理逻辑，源文件与隔离测试版本一致。实际Qt控制器经SSH读取新版任务状态成功。当前上位机已运行新版二进制，旧只读watch连接已结束，由上位机自动重连。原现场任务YAML、降落配置和共享传感器服务保持。部署时确认在地面、未解锁、无控制发布者；没有启动真实飞行任务或转动实机云台。

机载备份：~/drone_platform/01_apps/mission_demo/deployment-backups/map-missions-20261030-091626。证据见analysis/map-mission-20261030。首次编译因登录目录误用已自动回滚，修正显式工作空间后编译及核对通过。遥控器需先处于POSCTL，之后重新拨OFFBOARD产生新边沿；准备时已在OFFBOARD档不会立即执行任务。

部署及真实只读验收完成后，网络复核又出现SSH超时/无路由；这是当前通信状态，已完成的安装与编译无需重做。操作前请确认机载通信及ROS话题实时数据已恢复。

2026-10-30：地图航点启用返回起飞位置后自动降落；新旧地面站协议版本不兼容，需重启新版。机载任务的垂直速度上限为 0.25 m/s；AUTO.LAND 的速度由飞控参数控制。隔离测试不代表实飞验收。

## 2026-10-30 ArUco 排查候选修复（未机载部署）

本轮已检查上位机代码、历史机载源码与参数副本及11:55准备/11:57停止的地面站日志。当前沙箱拒绝SSH和localhost socket，无法检查机载当前状态或取得本次取消的landing CSV，不能认定`rc_switch_out_or_invalid`的现场根因。修改前诊断与最终验证记录见`analysis/aruco-audit-20261030/`。原历史部署记录仅适用于当时版本。

历史该取消原因包含：RC通道缺失、PWM无效、RSSI=0、时间戳0/未来/过期、非OFFBOARD目标档位；接收超时另有tick分支。新增机载候选源码把原因细分为`rc_channel_missing/rc_pwm_invalid/rc_rssi_zero/rc_stamp_missing/rc_stamp_future/rc_stamp_stale/rc_switch_out/rc_receive_timeout`，同时冻结触发时的CH/PWM/RSSI/档位和源/接收年龄。准备仍只转云台；保留POSCTL基线、新OFFBOARD边沿与人工已起飞条件，未放宽接管保护。

地面站修改后区分“识别成功”与“控制器接受目标”，显示触发拒绝和取消证据；监视包在线而控制CSV源戳过期时显示控制状态未知。识别Pose需正深度、有限坐标、源戳新鲜且与metrics同帧才能显示ID/偏移；旧检测节点缺少同帧信息时明确显示未知。阶段/原因变化写入地面站.log。机载管理器准备任务时冻结本次配置与源码/二进制指纹，CSV写入该run的`landing-diagnostics/`。

机载候选源码位于`tools/missions/onboard/aruco_test/`和`land_test/`，尚未部署。控制源时间年龄随时间继续累计，防止重复送达旧数据续期；只有缺少采集戳的图像接收别名才补偿0.3s，配对失败不会静默退回；姿态模型明确区分默认yaw稳定光轴和固定相机完整姿态，安装平移随完整机体姿态旋转；终端阈值相对标记平面Z，终端时冻结该平面。码大小、内参默认值、偏移、速度、容差及飞控参数没有按猜测修改，新增配置入口不能替代现场测量。

本地真实节点编译、纯控制策略/合成图像、Python和实际QML假SSH回归已执行；完整私有ROS集成因socket限制未执行成功，不能据此声明已验收。测试及机载部署注意事项见`tools/missions/test-native/README.md`和`tools/missions/onboard/README.md`。运行中新版本未切换，正式启动程序仍需核对二进制版本。

只读取证可从项目目录执行（需网络环境允许；不会准备任务或使云台动作）：

```bash
ssh -T orangepi@192.168.1.235 "bash -lc 'source /opt/ros/noetic/setup.bash; source ~/drone_platform/01_apps/mission_demo/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost; /usr/bin/python3 - --seconds 10'" < tools/missions/inspect_aruco.py > onboard-aruco-audit.json
```

现场仍需核对实际反色231标记及有效边长、1920×1080实际画幅/裁剪与内参、云台稳定模式和安装偏移/轴向，检查RC原始流与CSV事件前后数据，测量图像/定位/状态频率及延迟。先完成无桨台架与配套ROS集成验证，再安排真实精准降落实飞验收。

2026-10-30后续只读分析：网络权限恢复后已读取机载14:14地面未解锁快照及11:55真实飞行CSV。该趟存在往复、固定标记世界位置估计漂移与12cm启停制动问题；没有直接认定P过高，没有修改增益或机载部署。准备与完整私有ROS集成测试均已实际通过。见[飞行数据报告](../analysis/aruco-flight-20261030/report.md)。此前“socket受阻”的记录是首次交付当时状态。

2026-10-30 14:31按用户要求单独降低机载视觉位置外环P：实际`takeoff_land.yaml`的`xy_kp`由0.45降为0.30（约33%），D=0.20/I=0.02及其他配置不变。确认地面未解锁、无活动任务/控制发布者后，持有任务操作锁、备份并原子修改，回读SHA及YAML语义比较通过。新值在下次准备精准降落时加载；本轮没有启动真实任务或修改飞控参数，尚未验证新P的实飞效果。备份与验证见[本次P调整](../analysis/aruco-p-reduction-20261030/report.md)。
