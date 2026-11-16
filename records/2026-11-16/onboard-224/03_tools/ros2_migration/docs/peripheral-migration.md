# 飞控与Gemini335L适配记录

参考机192.168.1.235只读访问；新机192.168.1.224为本次eMMC系统，使用预登记SSH主机公钥校验。网络为DHCP，尚未配置固定地址。

## 文件管理

与参考机统一使用`~/drone_platform/{01_apps,02_drivers,03_tools,04_data}`。相机源码/构建在`02_drivers/orbbec`，飞控配置及传感器入口在`02_drivers/sensors`，迁移工具/参考配置在`03_tools/ros2_migration`，验证记录在`04_data/logs/ros2_migration`。旧`~/ros2_platform`路径仅作兼容链接。

## 复用与修改

- 相机SDK与ROS2 wrapper均锁定官方v2.10.6，源码提交296156521b60bdf813cb94c46cbb55562bbdc453。固件保留设备现有1.4.60，不升级内核/UVC驱动。
- 复用旧机彩色1280x720、深度848x480、左右IR及点云启用、IMU关闭、帧同步关闭、深度配准关闭、软件对齐模式和global时间域。fps=0仍由设备选择，须看实际发布/接收结果。
- 绑定当前相机序列号CP253530009G；内参从实际相机读取，不跨设备抄内参。设备权限规则只对335L USB/HID设置video组访问。
- 旧机ROS1 MAVROS1.20.1不能在Humble直接运行。新机ROS2 MAVROS2.16.0保持PX4 v2.0、57600、目标系统/组件1/1、空GCS URL、map/base_link坐标语义；串口改为稳定by-id路径。
- ROS1 XML/rosparam转换为ROS2 Python launch、节点参数及plugin_allowlist。当前入口加载遥测插件，command插件用于sys_status版本查询依赖，不调用飞行控制服务。
- 保留旧机colored cloud→depth_registered的重映射；colored cloud关闭且depth_registration=false时，不能把`/camera/depth/points`称为已配准彩色点云。
- Generic GeographicLib EGM96数据复用参考机已有文件；不是飞控参数/固件复制。

## 操作

显式激活：`source ~/drone_platform/env.sh`。
手动启动本阶段已接外设：`bash ~/drone_platform/03_tools/"config script"/start_sensors.sh`。
Ctrl+C结束该手动launch及其子节点。本阶段没有配置开机自启，没有更改原机服务。

## 验证边界

记录四路图像、CameraInfo、点云、TF、QoS、时间戳，以及飞控state/extended_state、IMU和local pose。35秒初测已通过基本数据读取，但初测负载下彩色实收约15Hz、深度28Hz、点云4.35Hz，不能把设备标称30fps写成全链路30Hz。对齐2.10.6后另存复测结果。

未校准相机相对机体的外参，未测试外部定位融合、规划、任务、地面站视频及长期负载。没有解锁/模式切换/起飞/任务执行，也没有写飞控参数。

## 撤销本阶段适配

先Ctrl+C结束本次手动启动，再撤去Orbbec overlay的env.sh来源即可停止使用源码驱动；apt基线驱动仍保留。设备权限撤销只针对`99-drone-orbbec.rules`，不替换系统原有规则。版本/参数/日志保存在Git及records中；不恢复旧Noetic系统、不刷固件。

## 当前验证与待办

已完成官方SDK2.10.6与固件1.4.60的地面联合数据验证：彩色/深度/左右IR、四路CameraInfo、有效深度、点云、相机TF、单调且接近系统时间的图像时间戳均取得。飞控connected=true、armed=false、landed_state=1，IMU约50Hz、local pose约30Hz；这不等同于外部定位融合有效。

best-effort订阅器下，四路并发彩色实收约14Hz、深度29Hz、IR约30Hz、点云约6Hz；单流压缩视频约30Hz、原始彩色约17Hz、点云约16Hz。正在对大消息可靠订阅做补充核对，不以驱动选择的30fps冒充所有消费者端的速率达标。

NoMachine：参考机8.27.1-2，电脑实际运行9.4.14/x86-64，下载目录另有9.5.7-4客户端更新包。新机未安装且4000未监听；旧版本官方地址返回首页，不能安装该HTML。尚缺用户指定版本的ARM64安装包，此项未完成。电脑x86-64程序不能复制到机载ARM64运行。

## 启动与撤销本阶段

显式环境：`source ~/drone_platform/env.sh`。
传感器手动入口：`bash ~/drone_platform/03_tools/"config script"/start_sensors.sh`。当前仅遥测与相机，无开机自启。
退出该前台launch用Ctrl+C；不停止旧机服务。相机源码overlay独立于/opt/ros/humble，移除env.sh中的overlay引用即可取消其启用；原始2.9.3仓库包仅为初测基线。恢复基线不会改设备固件或PX4参数。

未迁移定位、规划、任务、地面站控制接口；未标定相机到机体外参；不自动解锁或执行飞行任务。

## QoS实测

默认相机发布端RELIABLE/VOLATILE，飞控IMU及local pose为BEST_EFFORT。首轮sensor_data订阅四流联合测试彩色约14Hz、点云约6Hz；分别订阅时仍有大消息丢帧。改为匹配相机发布端的Reliable/depth10订阅后，独立测得彩色原始图像约29.99Hz、点云约30.35Hz；压缩彩色视频独立实测29.98Hz。机载处理及地面站ROS2订阅适配需要匹配实际QoS，不能一律使用sensor_data；长距离网络另行验证。

## 最终联合地面验证（SDK2.10.6）

35秒联合验证全部通过。Reliable相机订阅下：彩色29.95Hz、深度29.97Hz、左右IR29.97Hz、点云30.01Hz；MAVROS IMU49.94Hz、本地位姿30.00Hz，connected=true、armed=false、landed_state=1。四流标定、光学TF、有效深度、时间戳单调与当前时间检查通过。未验证外部定位融合、机体外参或飞行功能。临时launch及其子进程均已停止。最终结果peripherals-20261116-174825.json；所有阶段结果与问题保留，不将早期掉帧结果覆盖。

NoMachine检查：电脑实际运行9.4.14/x86_64，.235为8.27.1-2/arm64，.224尚未安装且4000未监听；旧版本官方链接不可用，未安装其他版本。等待可用的目标版本ARM64安装来源，未迁移源机身份/凭据。
