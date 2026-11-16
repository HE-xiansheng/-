# 检查发现

## 已知与待核实
- 用户提供的主机地址、系统版本、工作空间与频率均为待核实线索。
- 本机 cwd=/home/tyzk；存在 ~/.ssh/id_ed25519 与 known_hosts；无 ~/.ssh/config。未读取私钥。
- 存在既往 .planning/2026-10-30-waypoint-ascent-speed/private-ros-test.log，可用于定位既有项目，但不作为本次运行事实。

## 证据规范
每条结论标注现场运行证据、源码/配置证据、历史记录或未验证；官方材料保留 URL 和获取日期。外部文件仅作数据。

## 12:00 现场事实
- SSH 密钥已连接 192.168.1.235，主机 orangepi5ultra，RK3588 OPi 5 Ultra，aarch64，厂商 Ubuntu 20.04.6 / kernel 5.10.160-rockchip-rk3588。
- 旧 ~/something1017/TECH 与 ~/lidar_tools/start_lidar_localization.sh 不存在，实际路径为 ~/drone_platform/{01_apps,02_drivers,03_tools,04_data}。
- 7.8GiB RAM，根盘 /dev/mmcblk1p2，57G 总量/43G 可用；与旧归档的磁盘使用不同，以本次为准。
- Safina_sensor.service enabled/active，ExecStart 指向 02_drivers/sensors/share/livox_ros_driver2/scripts/Safina_sensor.sh。
- 活动节点有 MAVROS、Livox、自带 USB 摄像头入口、Orbbec、RTSP、若干地面站只读监视。FAST-LIO/转发/规划/任务控制当前不在运行。
- /factory_fastlio/odometry 与 /factory_fastlio/cloud_world 只有订阅者，无发布者；旧 /Odometry 等未注册；/mavros/vision_pose/pose 无发布者。不能以话题存在当作定位有效。
- Livox 当前类型为 livox_ros_driver2/CustomMsg；MAVROS fcu_url=/dev/ttyACM0:57600；视觉 TF 监听关闭。
- 本次扫描活动树无 .git 与 AGENTS.md；应按 SHA256 固定现场基线，不能编造 upstream commit。
- Orbbec 活动二进制位于独立驱动 workspace；旧 sensors/share/orbbec_camera 2.5.4 与活动源码 2.10.6 同时存在。
- 主机 chrony 当前已同步，偏差约0.1ms；本次启动日志含旧启动日期，后续分析不能只用早期墙钟。
- ROS 官方当前已列 Lyrical (2026–2031)，Jazzy (2024–2029)，Humble (2022–2027)。不得沿用过时的“最新 LTS=Humble”假定。

## 检查工具异常
- 含本机重定向的 SSH 命令默认沙箱仍拒绝 socket；按要求显式升级权限后成功。只读采集脚本经 stdin 运行，不远端落盘。
- 地面站源码不是 Git 仓库；不存在可读取的 Git HEAD。
- 初查 mediamtx.service / mediamtx-gimbal.service 不存在；后续确认实际服务名为gimbal-rtsp-relay.service。

## 14:26 后续只读采集
- 用户继续时原IP恢复，机载经历重启；chrony已同步。此后采样证据独立于12:00快照。
- 导出603个文本源码/配置及关键二进制SHA256；本机snapshot/保存可阅读副本。mission_demo/SHA256SUMS含NUL，不能作为有效版本清单，改用现场新计算摘要。
- 最新12秒采样：Livox点云源戳约10.00Hz；IMU源戳约200.01Hz；MAVROS local_position/pose源戳约30.00Hz。订阅初始积压会使接收频率偏高，报告使用源戳频率且注明短窗口。
- FAST-LIO和转发仍无发布者，不能实测LIO10Hz或视觉30Hz。转发配置确为publish_rate=30.0。留存的active*.json是旧PID文件，不代表进程正在运行。
- /mavros/state：connected=true、armed=false、mode=AUTO.LOITER；extended_state landed_state=1。估计器水平位置/速度有效标志false，const_pos=true，不能把local pose30Hz误判为雷达已融合。
- 相机SDK话题：设备固件1.4.60，SDK和ROS wrapper均2.10.6。彩色图和RTSP各取得3帧，非持续性能验收；初次直接探测未收到深度/IR，随后已通过标准ROS订阅每流收到3帧，见下方纠正记录。
- 当前rknpu内核模块0.9.6，人物检测源码调用C++ RKNN API，模型yolov11_s_det_int.rknn，不依赖CUDA/TensorRT。视频源码依赖Rockchip MPP。
- FAST-LIO在用config/lidar.yaml：time_sync_en=true；与上游workspace/config/mid360.yaml默认false及噪声参数不同。迁移应沿用在用配置，不直接拷默认模板。
- FAST-LIO输出camera_init→body，twist.linear已改为body坐标；/tf重映射到/factory_fastlio/tf。EGO odom_vel_is_body=true会转到world，需要一致保留。
- 传感器服务中FAST-LIO和to_mavros启动段为注释。Safina_sensor相机启动还含清理旧节点和设备锁操作；只读取，未执行脚本。
- 定位转发仅留安装包和二进制，share/to_mavros暂无对应.cpp，须查源代码归档或重写并做语义对照。

## 官方兼容性初步证据
- Livox官方driver2当前README明确支持Ubuntu24.04/ROS2 Jazzy；需要CustomMsg及单点时间字段，MID360s分支配置不可混用MID360默认配置。
- Orbbec官方v2-main明确支持Gemini335L及Humble/Jazzy/Lyrical；当前推荐固件1.8.10与现场1.4.60不同，不能据README就刷固件。
- hku-mars/FAST_LIO官方仓库存在ROS2分支cc3c9e8b1469b33b8072764d6db2348ed6dc597f，README建议Humble。Jazzy及本机速度补丁、时间单位需单独回归。
- ZJU官方ego-planner说明指向ego-planner-swarm的ros2_version分支；README为Humble/CycloneDDS并记录FastDDS卡顿。并不包含现场所有预览/人工确认/PX4执行扩展。
- ros1_bridge官方README强调24.04无ROS1且不支持原生bridge。其20.04/Humble表述比REP2000宽泛，不能当成Humble/Focal官方二进制支持。

## 后续核实与纠正
- 标准只读ROS订阅得到深度与左右IR各3帧，848×480；深度16UC1，IR mono8，均有对应optical帧。最初直接TCPROS无帧属于采样限制，不是相机故障。DeviceStatus解码异常也不能当成设备异常。证据：raw/camera_standard_subscription.json。
- 检测二进制ldd实际指向工程内librknnrt.so2.3.2（d31fc19c…），不是系统/usr/lib的1.4.0；模型yolov11_s_det_int.rknn摘要c9184970…。视频依赖MPP/RGA，重装系统不能只考虑ROS deb。
- 飞控只读参数：EKF2_EV_CTRL15、EKF2_HGT_REF3、EV_DELAY0、EV偏移XYZ0；旧AID_MASK/HGT_MODE不支持。未调用任何设置或控制服务。
- 已保存的官方5 Ultra wiki确实可读，列Ubuntu20.04/22.04及5.10/6.1兼容说明；最初部分SSL/HTTP失败并不表示最终未获取。官方板卡构建配置还列next/resolute，但未列noble；镜像可启动与NPU/媒体兼容仍未验证。
- 地面站任务管理器和机载部署版不同（159行diff），定位管理器副本一致；机载转发二进制与本机候选源码/二进制不能建立同版本证明。以现场摘要为基线。
- 14:33:58运行图有205个话题类型、178个服务和473个展开参数。14:41:42对当时172个服务仅做probe类型握手，全部取到类型；6个临时监视/传输节点的日志服务已退出。不把这两个快照混为同时运行状态。
- 603份机载文本和233份地面站文本均逐一通过原清单的SHA256及大小校验。这是文本基线及关键二进制摘要，不是完整系统恢复镜像。

## 当前交付
report.md汇总现场结构和版本选择，migration_matrix.md列逐模块实现/源码/bridge路线，interfaces.md与interfaces/保存完整接口/参数索引，validation_and_rollback.md列S0–S6验收/回退，issues.md列未解决项。ROS2安装、编译、部署与行为验收尚未执行。

## 用户版本选择修订
- 用户明确希望直接使用Ubuntu22.04；当前实施目标改为原生Jammy ARM64 + ROS2 Humble，取代最初Jazzy用户态推荐，不改变重装/写盘/停服务前需确认的约束。
- 重新读取官方5 Ultra wiki，支持表明确列Jammy的5.10与6.1内核及NPU/VPU/USB；优先评估接近当前5.10的镜像，具体镜像和驱动组合尚未验证。
- 新读取Humble rosdistro发行索引：MAVROS2 release为2.17.0-1；不沿用Jazzy索引里的版本。Livox、Orbbec与FAST-LIO/EGO官方资料均有Humble依据。
- Humble安装网页返回反机器人验证页，改取官方ros2_documentation仓库的Humble安装源文档，不将验证页当成安装资料；来源与失败记录另存raw/humble_document_fetch.json。
- 新增ubuntu22_native_plan.md，并同步当前报告、迁移清单、验收与问题记录。原20.04/Noetic恢复以保留原介质为准，不假设22.04可原样运行Noetic安装空间。

## 已有恢复镜像
- 用户确认完整恢复备份镜像已在电脑下载目录，沿用现有备份，不重复制作。本轮在/home/tyzk/下载的常规文件及镜像扩展名搜索中未定位该镜像（部分NX子目录权限不足），因此将备份存在标注为用户确认，未声称已校验文件；不据此要求用户重做备份。

## 新插SD与历史技能
- 当前/dev/sda经lsblk/udev/根目录只读盘点为USB可移动卡，62,534,975,488字节，exFAT，根目录为空；详情raw/sd_card_inventory.json。当前盘符不是未来写入默认值。
- 旧技能历史案例记录eMMC62,579,015,680字节，比本卡多42MiB；不能按“同为64G”裸克隆。历史Armbian Trixie镜像是Debian救援环境，本次需要5 Ultra专用Ubuntu22.04。
- 值得复用身份/容量核对、来源/校验、写后读回、远程网络/SSH、正常关机/弹出和分阶段实测。不执行源机净化、自动采集/恢复、旧root凭据或离线救援卡的删网络收尾。

## eMMC安装目标已确定
官方5 Ultra wiki明确支持从TF/SD启动后将独立系统img写入未挂载eMMC、关机拔卡再从eMMC启动。不能把在线SD根盘克隆当成等价步骤。已找到官方Ubuntu下载目录及历史归档，包含Jammy/Xfce5.10.160与当前6.1.99候选，正在核实下载。

## 官方安装镜像下载限制
- 目标Jammy/XFCE/5.10.160压缩包在两个Orange Pi官方网盘目录同名同大小（1624268933字节）。Google正常下载返回配额超限；百度公开分享验证、列目录成功，但标准下载请求errno2且无链接。未把这个错误推断为明确登录要求。
- 已通过本机浏览器打开官网百度链接，并准备下载说明与文件校验工具。当前仍缺镜像二进制，不具备写卡条件；不拿HTML响应、目录大小或非标准MD5字段冒充校验成功。

## 镜像准备与在线进度更新
- 官方Jammy镜像已下载并解压（img 9,181,331,456字节）；只读确认Ubuntu22.04.4、5 Ultra/ARM64及对应DTB、SSH/NetworkManager。下载阻塞已解除。
- 用户要求跳过额外完整性测试；7z测试在指令到达前已完成，后续全镜像哈希流程已停止，必要解压单独完成。不再安排全盘读回或整文件哈希。
- 应用户要求复查skill远程进度机制：SSH读取原子更新且fsync的状态JSON、systemd日志，电脑保存进度。新增online_install_progress.md与只读monitor_install.py；写入100%与同步/启动成功分开记录。工具尚未部署到SD，未实测远程安装。
- 未卸载/写入SD或eMMC，未停服务。具体SD覆盖范围已保存sd_write_review.json。

## 制卡期间机载只读复查
旧系统仍从/dev/mmcblk1p2运行，Safina_sensor.service活跃；当前MAVROS未解锁、landed_state=1、AUTO.LOITER。与早期审计不同，现已存在FAST-LIO及30Hz位姿转发进程，以及多个GCS task watcher；未确认任务完全空闲，不能自动关机/停定位。此为现场后续变化，不反改原始审计结果。

## SD安装卡完成
- prepare_install_sd.py最终退出0，写入与卡内独立镜像复制均为9,181,331,456字节，fsync成功；已正常卸载boot/root并flush。配置/状态详见raw/sd_ready_report.json。未执行额外全盘读回/哈希，未上机验证。
- 安装卡启用只读启动状态服务和密钥SSH，生产eMMC安装器草稿仅存电脑，不自动写eMMC。
- 临近收尾尝试只读读取机载任务记录时SSH banner超时；未确认当前所有任务空闲，未关机或停止任何机载服务。实际停机前须恢复连接并重新核对状态。

## SD启动已实测确认
- 用户已将SD插入Orange Pi并表示从SD启动。动态发现192.168.1.224，预登记安装卡ED25519主机公钥匹配，密钥SSH登录成功。实际root=/dev/mmcblk0p2、boot=/dev/mmcblk0p1及UUID均与本次SD一致，Ubuntu22.04.4/5.10.160/ARM64；Wi-Fi与SSH及在线状态读取已实测。
- eMMC现场容量250,139,901,952字节、CID d6290341335630313411560574f2bc00、GPT 1E66CE0A-2419-494D-A6D8-A332C78B24A1，非历史64GB案例。所有eMMC分区未挂载、无swap，旧系统Ubuntu20.04.6。机载当前USB列表只有root hub，没有运行ROS控制进程。
- 旧工程目录与先前审计235机载不同，现场有something1017/cooperation，无drone_platform。按用户“旧工程无需保留”停止工程复制并清除本次临时归档，未修改eMMC内容。
- 已把安装器及手动启动服务部署到SD（static、不启用自启动），未创建批准配置、未启动安装。首次状态在NTP前生成时间偏旧；时钟同步后重新生成，只读在线监控取得RECENT状态。
- emmc_write_review.json记录确切目标、覆盖范围和非覆盖范围；待明确此目标覆盖确认后安装。

## eMMC写入及配置完成
- 用户授权后安装成功：systemd Result=success、ExecMainStatus=0。写入9,181,331,456字节并同步完成；root分区扩至249,033,654,784字节。
- 小范围只读核对新root/boot的UUID与fstab/bootenv一致，Wi-Fi及新SSH主机公钥已配置，预登记公钥保存emmc_known_hosts。目标分区检查后已卸载。无全盘读回或哈希，尚未从eMMC启动。
- raw/emmc_online_progress.jsonl与raw/emmc_install_service.log保存安装过程；raw/emmc_install_complete.json保存结果。下一步正常关机，用户拔SD再上电后，以新主机公钥及根/boot来源确认eMMC启动。

## eMMC启动确认
预登记生产SSH主机公钥匹配192.168.1.224；root=/dev/mmcblk1p2、boot=/dev/mmcblk1p1，MMC CID与容量均匹配安装目标，SD未出现在块设备列表中。Ubuntu22.04.4、内核5.10.160、Wi-Fi/SSH和时间同步已实测。eMMC系统安装阶段完成，ROS2尚未安装，外接USB传感器/飞控尚未接入。厂商dnsmasq.service处于failed，需只读诊断，不擅自停/改服务。RGA和rknpu0.9.6内核初始化可见，但尚未推理实测。

- 用户明确当前只装系统、外设稍后接。本阶段收口为Ubuntu22.04/eMMC启动及Humble基础环境与独立工作空间；外设驱动和飞控/任务功能阶段暂不启动。当前有线网口NO-CARRIER、USB未见相机/飞控，未据USB缺失推断MID360（以太网）接入状态。

## 系统与Humble基础环境交付完成
- eMMC启动确认后，Humble ros-base、官方C++/Python demo、colcon、rosdep、vcstool安装成功。293个新包、仅libsqlite3-0升级，无移除/厂商锁定包变化；保留厂商内核及BSP。
- 官方C++ talker→Python listener本机DDS验证通过，收到5条消息；进程正常退出。未运行传感器、飞控或任务验证。
- dpkg --audit无问题，rosdep/vcs版本查询及空工作空间colcon list成功。测试先前由root从不可访问目录降权运行，产生colcon_defaults.yaml权限错误；从工作空间目录执行后通过，无需更改工具权限。
- 归档过程中root跨用户Git访问触发dubious ownership；改为仓库所属用户执行。随后提交缺少作者身份，复用仓库上次提交身份，仅对本次命令指定，未修改全局Git配置。机载最终提交7c47a97b2962f5834d92cda8c8f6eba678605c84。
- raw/ros2_base_environment.json、完整软件清单、厂商锁定清单、apt日志及DDS日志已保存电脑。机载工作空间/home/orangepi/ros2_platform/humble_ws，显式source ~/ros2_platform/env.sh启用。
- 用户要求暂时只装系统，外设稍后接。本次范围完成；后续驱动/定位/飞控/规划/视觉/地面站阶段暂缓。dnsmasq端口53冲突继续记录为非阻塞已知问题，未修改服务。

## 飞控与双目相机迁移：现场对照
- 新机仍为已授权eMMC CID d6290341335630313411560574f2bc00，Ubuntu22.04/Humble；本次重启boot_id=1bb61fa9-1528-4111-beeb-9d0574f387ad。
- 新接Orbbec Gemini335L序列号CP253530009G，USB3 5000M；飞控1b8c:0036，稳定路径/dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00。
- .235现用ROS1 SDK2.10.6、1280x720彩色/848x480深度，左右IR及点云开启，IMU/帧同步关闭，depth_registration=false、time_domain=global。MAVROS串口57600、target1/1、v2.0。原服务脚本包含kill -9及多模块启动，不能直接搬来运行。完整参数只读留存raw/peripherals_ros1_reference.txt。
- apt现场候选MAVROS2 2.16.0（不同于此前发行索引2.17.0），Orbbec ROS2 2.9.3。模拟新增33包、0升级/移除。开始安装ROS2用户态包；不替换固件/内核/BSP，不修改飞控参数。
- 重启后/var/log/ros2-migration不存在（历史日志电脑已归档）；新阶段将日志同步到工作空间records持久保存。
- 新读取官方GitHub旧README路径404；沿用已归档官方资料并检查已安装包实际launch/参数，不凭上游最新版假设已安装版本接口。

## 新增要求：版本与.235一致、NoMachine远程桌面
- 用户截图显示.224:4000拒绝连接；现场未装NoMachine、端口无监听，SSH/桌面正常。旧机NoMachine8.27.1-2；用户要求版本一致，必须使用该ARM64版本，不自动升级新版。
- 同时将相机SDK从已通过初测的apt2.9.3调整为官方ROS2 v2.10.6，对齐旧机ROS1 wrapper/SDK2.10.6。官方tag解析到296156521b60bdf813cb94c46cbb55562bbdc453；保留初测结果，源码overlay单独构建。MAVROS1→2无法以相同版本号替换，保留参数语义并显式记录差异。
- 原四流35秒验证通过；实际同订阅负载下彩色约14.94Hz、深度27.89Hz、左右IR约29.6/30Hz、点云约4.35Hz，不能把设备配置30FPS当所有ROS订阅者都30Hz。飞控IMU50Hz、本地位姿30Hz，connected=true、armed=false、landed_state=1。
- MAVROS command插件依赖修复后18个state样本连接正常、未解锁。相机缺失的compressed/compressedDepth/theora图传插件已补齐，以复用旧话题的压缩接口；仍需在最终版本复验。

## 最终联合地面验证（SDK2.10.6）

35秒联合验证全部通过。Reliable相机订阅下：彩色29.95Hz、深度29.97Hz、左右IR29.97Hz、点云30.01Hz；MAVROS IMU49.94Hz、本地位姿30.00Hz，connected=true、armed=false、landed_state=1。四流标定、光学TF、有效深度、时间戳单调与当前时间检查通过。未验证外部定位融合、机体外参或飞行功能。临时launch及其子进程均已停止。最终结果peripherals-20261116-174825.json；所有阶段结果与问题保留，不将早期掉帧结果覆盖。

NoMachine检查：电脑实际运行9.4.14/x86_64，.235为8.27.1-2/arm64，.224尚未安装且4000未监听；旧版本官方链接不可用，未安装其他版本。等待可用的目标版本ARM64安装来源，未迁移源机身份/凭据。

## NoMachine 9.4.14 ARM64获取尝试
用户要求获取与电脑9.4.14对应的ARM64包。继续读取既有记录，核对官方历史deb/tar、CDN、可能的打包修订和公开历史索引/仓库线索；当前旧链接重定向官网HTML，不是安装文件。GitHub精确版本公开索引未找到可用安装包；历史网页索引超时。未将HTML保存为deb，未安装其他版本或改动机载远程桌面服务。证据保存在raw/nomachine_*probes.json及raw/nomachine_github_version_search.json。精确版本安装包获取仍未完成。

## 更正旧版本获取结论与新证据
不能由已查官方旧URL失效推断全网无法找到。GitHub历史构建记录确认nomachine_9.4.14_1_arm64.deb曾存在：ranshalit/hadron_yocto commit20008ace2fffc3173e487cac6792842c874ce344，配方源URL为/download/9.4/Raspberry/...，声明sha256=3b34641d0fb28bf0bc96e01ab37c84ed8c34ed4a816d794047b85abd536e79ef（第三方历史记录，不等同于已取得文件）。另NixOS/nixpkgs commit0f906ee9b43802e9c40053e5af8240faac534aa3记录2026-05-08时9.4.14电脑版官方tar已撤下并重定向主页，说明问题并非只有ARM64。找到9.8.2 ARM64公共GitHub镜像，仅作来源线索，未自动更换版本。继续沿ARM配方和ConnectTech参考文章检索副本。

后续结果：ConnectTech页面403/API401，附件列表404；Yocto公共源镜像无该文件，OpenEmbedded源站TLS失败；ARM构建仓库未发现安装包副本。另找到nilsl88/container-fedora-xfce-nomachine PR5，明确引用9.4.14_1_aarch64.rpm，支持ARM版本历史存在。当前未取得可下载安装包；不能断言全网无存档。已向用户请求其看到的电脑版旧版下载来源，以便定向对照。

## 用户提供NoMachine8.5.3 ARM64包
- 用户给出电脑/下载/nomachine_8.5.3_1_arm64.deb并指定放到安装工具目录。包控制信息确认nomachine8.5.3-1/arm64、约49MB，真实非空vendor载荷；已复制到机载03_tools/install_assets并保留电脑原文件。
- 已执行dpkg安装8.5.3-1（不是9.4.14或参考机8.27.1），返回0。XFCE入口为/usr/bin/startxfce4，nxserver/nxnode/nxd已启用，nxserver.service active/enabled。电脑到192.168.1.224:4000 TCP实测可达。未做账号登录及桌面交互实测。
- 安装器出现ufw规则配置警告，但4000外部TCP访问通过；未另改防火墙。dpkg --audit为空。安装日志保存在机载04_data/logs/ros2_migration/nomachine-install.log；结果raw/nomachine_8.5.3_install_result.json。
