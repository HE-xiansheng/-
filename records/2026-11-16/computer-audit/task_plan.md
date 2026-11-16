# 科研飞行平台迁移：当前计划

目标：Orange Pi 5 Ultra 原生 Ubuntu22.04 ARM64 + ROS2 Humble，覆盖驱动、定位、飞控通信、规划、视觉任务及Linux地面站。原始审计和历史操作记录保存在report.md、findings.md和progress.md。

## 已确认的范围

- 用户已授权SD制卡及本次256GB eMMC的Ubuntu22.04覆盖安装，均已完成。
- 用户明确本次目标eMMC旧工程无需保留；先前ROS1审计资料仍保存在电脑。
- 用户要求跳过额外镜像完整性测试、整文件哈希和全盘读回；正常写入同步、配置检查和功能验证仍分阶段进行。
- 后续重装、替换底层驱动、停止现有服务及正式环境切换，按用户要求说明具体影响并确认；已授权动作不重复确认。
- 不解锁、不起飞、不自动执行飞行任务，不调用控制服务。

## 阶段

1. [complete] 机载与地面站只读审计、接口与兼容性研究、迁移矩阵。
2. [complete] 厂商5 Ultra Jammy/XFCE/5.10.160镜像准备及SD启动安装卡制作。
3. [complete] SD上机，Wi-Fi/SSH和在线进度通道实测。
4. [complete] 按CID/容量/GPT确认256GB eMMC后安装、扩容、配置并保存在线日志。
5. [complete] 拔SD重新上电，预登记生产公钥匹配；根/boot均在eMMC，Ubuntu22.04.4与网络/SSH正常。
6. [complete] 安装Humble基础包和开发工具，记录软件版本，验证C++→Python本机DDS通信；独立工作空间已建立。
7. [pending, user-deferred] 用户选择外设稍后接，本阶段不启动外设模块；外设接入后迁移Livox/Orbbec驱动，核对消息、QoS、TF、时间戳和采样率。
8. [pending, user-deferred] FAST-LIO、位姿转发、MAVROS/PX4接收融合的未解锁地面验证。
9. [pending, user-deferred] 规划、ArUco、人物跟踪和地面站任务/状态/视频接口迁移与地面验证。

## 当前现场

- eMMC主机动态地址192.168.1.224，HostKeyAlias=opi-ros2-emmc，公钥登记在emmc_known_hosts。
- eMMC CID=d6290341335630313411560574f2bc00，容量250139901952字节；root=/dev/mmcblk1p2，boot=/dev/mmcblk1p1。
- 独立工作空间/home/orangepi/ros2_platform/humble_ws；环境显式激活，不自动启动飞行程序。
- 当前USB未见外接传感器/飞控，硬件功能阶段不能标记完成。
- 厂商dnsmasq因端口53占用启动失败；当前Wi-Fi/DNS可用。已记录，未擅自停改服务。

## 本次交付完成
Ubuntu22.04/eMMC及Humble基础环境已完成，后续外设和任务迁移由用户暂缓。版本、软件清单和通信验证见raw/ros2_base_environment.json；机载工作空间提交7c47a97。

## 新增阶段：飞控与双目相机接入
用户已接入飞控和双目相机，要求对照192.168.1.235复用配置/驱动并修正差异。先只读采集实际设备及旧配置，再部署ROS2兼容驱动与独立配置，进行不解锁的地面数据验证；不改飞控参数、固件、内核或现有服务。
- [complete] 新旧设备/活动驱动/参数对照。
- [complete] MAVROS2与Orbbec ROS2 v2.10.6部署、参数/QoS适配及数据验证。
- [in_progress] 用户已提供8.5.3-1 ARM64包并授权安装，复制到install_assets，安装并检查XFCE/NoMachine服务和4000端口。

- [complete] 用户提供8.5.3-1/ARM64安装包，已归档install_assets并安装，NoMachine服务及端口4000验证通过；桌面登录待实际连接，9.4.14历史包未取得。
