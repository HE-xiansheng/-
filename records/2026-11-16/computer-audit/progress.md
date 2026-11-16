# 工作记录

## 2026-11-16
- 开始只读部署审计。已读取 planning-with-files 技能，建立独立记录目录。
- 确认不会停止服务、修改飞控状态或执行飞行任务。
- 本机无项目级 AGENTS.md，无用户 SSH 配置，继续读取 SSH 有效配置、已有历史检查和本机项目路径。
- SSH 首次因沙箱 socket 权限失败，按规定申请升级并获准。现已只读连接。
- 已落盘 raw/remote_inventory.json 和 raw/remote_graph.json；获取官方 ROS 发布表。
- 已读地面站 AGENTS.md 和当前入口代码，Linux Qt 地面站依赖 SSH 远端 Noetic Python 适配器与 UDP，当前源码无 Git 元数据。
- 发现路径已整体迁移到 drone_platform，未启动 LIO 与转发。准备直接读取现有发布者采样，不注册/发布 ROS 节点、不调用控制服务。
- 中断前机载失联，用户继续后连接恢复；只读确认机载重启及时间重新同步。
- 已保存新系统清单、ROS图、603份文本源码与配置、驱动/FAST-LIO/MAVROS/RKNN二进制摘要。
- 完成12秒直接TCPROS采样：Lidar10Hz/IMU200Hz/FC local pose30Hz，LIO与视觉转发未运行；收到相机彩色和RTSP帧。
- 补充标准只读ROS订阅后深度/左右IR各收到3帧，纠正最初“未采到即待查”的不完整记录；未做长时性能或标定验收。
- 核实检测实际链接工程RKNN Runtime2.3.2；系统1.4.0不是该二进制的实际依赖。确认MPP/RGA、PX4只读参数及任务管理器源漂移。
- 完成官方生命周期、板卡wiki/构建配置、驱动与算法ROS2分支、MAVROS QoS/去重和bridge限制核对。Jazzy用户态为首选验证目标；不声称原生24.04 BSP已经可用。
- 保存report.md、migration_matrix.md、validation_and_rollback.md、issues.md；生成完整接口索引和473个活动参数。后续服务只读probe取得172个类型，未调用控制接口。
- 603份机载文本与233份地面站文本SHA256/大小逐一匹配。完成证据索引、30份官方资料来源/摘要、接口清单与本地校验；未创建或启动远端ROS2环境。
- 本地校验通过：836份源/配置副本、JSON可解析性、采集脚本语法、99个文档本地链接及官方副本摘要。verification.json明确仅验证资料完整性，不代表ROS2构建或硬件功能验收。
- 本轮审计与迁移路线已交付；最终资料在独立本地Git仓库归档，不改原项目、不推送远端。后续从S1独立用户态/消息接口与回放开始，现场设备/服务/正式环境切换仍须按用户要求确认。

## 用户指定Ubuntu22.04后的方案修订
- 将当前目标统一改为原生Ubuntu22.04 ARM64 + ROS2 Humble，不要求先走Jazzy/24.04容器路线。历史初始建议仅保留作检查记录。
- 核对5 Ultra官方Jammy5.10/6.1支持表，补取Humble安装文档与发行索引，确认MAVROS2索引2.17.0-1；官网安装页被验证页阻拦后改用官方文档仓库源文件。
- 新增原生22.04系统准备/介质/依赖/回退清单，修订报告、模块矩阵、阶段验收与问题清单。没有下载系统镜像、写盘、重装、停服务或修改机载文件。

## 已有恢复镜像
- 收到用户确认：完整恢复镜像已有。已取消重复制作备份的安排并同步原生22.04路线与验收记录；本机下载目录初步搜索尚未定位具体文件，保留为文件定位/校验待核对，不阻塞方案准备。未写盘或修改现有备份。

## SD卡与旧技能复用
- 应用户请求读取drone-system-image技能及rescue-card/workflow/tested-profile，提炼适合原生Ubuntu22.04启动卡的制卡、读回、远程访问与验收步骤。
- 只读识别USB读卡器设备/dev/sda：62,534,975,488字节，exFAT分区挂载，当前根目录为空；原始清单存raw/sd_card_inventory.json。读卡器序列号不等于SD的CID，后续插拔需重新识别。
- 确认历史成功案例用的是Debian Trixie救援镜像，不是Jammy；本次不复用其系统镜像、源机清理、自动eMMC采集或救援root凭据。
- 保存sd_installation_workflow.md。未下载新系统镜像，未格式化、卸载、写卡、停止服务或切换机载环境。

## 官方镜像获取与安装准备
- 用户确定安装到eMMC、SD作启动/安装；已更新emmc_install_plan.md及当前路线，不再要求保留原20.04可启动介质。已有恢复镜像仅作备用。
- 官网HTTP下载页可读，HTTPS部分入口TLS失败。官方Google Drive目录含Jammy1.0.0/XFCE/5.10.160历史包与1.0.2/6.1.99；常规下载返回配额超限。
- 官网百度分享提取码验证通过，Ubuntu/旧镜像备份中的目标文件名和1,624,268,933字节大小与Google一致；其md5字段非标准hex，不作可用校验摘要。
- 百度标准分享下载请求未返回链接（errno2），未将其断言为登录要求。已打开官方百度页面，创建电脑下载准备目录与说明；目标压缩包仍未取得。
- apt-get download成功取得p7zip-full，仅在缓存目录解包，未安装系统包；此前venv缺ensurepip失败，已改用本地解包工具。新增prepare_ubuntu22_image.py，仅做文件校验/解压和镜像分区读取，不写块设备。
- 下载状态、来源/大小、外部限制均已记录。未停服务、重装、格式化或写SD/eMMC。

## eMMC安装目标已确定
用户已明确最终安装到eMMC，SD只负责引导和安装；已将保留旧启动介质的假设移除。用户询问回退后已说明已有恢复镜像只是故障备用，不作为日常安装步骤，不重复制作。继续准备官方Jammy镜像，尚未写入任何介质。

## 镜像准备与在线进度更新
- 官方Jammy镜像已下载并解压（img 9,181,331,456字节）；只读确认Ubuntu22.04.4、5 Ultra/ARM64及对应DTB、SSH/NetworkManager。下载阻塞已解除。
- 用户要求跳过额外完整性测试；7z测试在指令到达前已完成，后续全镜像哈希流程已停止，必要解压单独完成。不再安排全盘读回或整文件哈希。
- 应用户要求复查skill远程进度机制：SSH读取原子更新且fsync的状态JSON、systemd日志，电脑保存进度。新增online_install_progress.md与只读monitor_install.py；写入100%与同步/启动成功分开记录。工具尚未部署到SD，未实测远程安装。
- 未卸载/写入SD或eMMC，未停服务。具体SD覆盖范围已保存sd_write_review.json。

## SD制卡已授权并启动
用户在具体卡覆盖确认后回复“继续”，已授权本次SD整盘覆盖。重新核对读卡器标识/62,534,975,488字节/空exFAT分区一致。已准备NetworkManager私有凭据（不入Git）、SSH公钥与预登记安装卡主机公钥；通过正常pkexec请求执行prepare_install_sd.py。当前正在等待系统认证或写入进度，以raw/sd_prepare_status.json为准，不能提前记完成。eMMC未授权写入，机载服务不受影响。

- 制卡首次执行在写盘前拒绝：旧版lsblk在未包含NAME且未加--tree时输出扁平JSON；已确认并修正为显式--tree。尚未卸载或写入SD。

- SD第二次执行已通过正常系统认证并进入真实写入，电脑端状态持续更新且保存raw/sd_prepare_status.jsonl。新增eMMC安装器草稿install_emmc.py，只准备代码，未部署/执行；需SD上机后按实际CID/容量/GPT和当前boot_id确认，再生成批准配置。
- 只读核对厂商首次扩容脚本，决定已离线完成扩容的卡使用厂商支持的/root/.no_rootfs_resize标记，保留其他硬件初始化；制卡收尾需确认该标记存在。

- 镜像写入9,181,331,456字节并fsync成功，GPT已移至卡尾并扩展root分区；离线resize2fs要求e2fsck -f，因此停止，未运行全文件系统检查。调整为先挂载后通过内核在线扩容；恢复仅针对已记录GPT57c7436b-b766-4d45-a7cc-48fb0274fc59和原root UUID的这张卡，跳过已成功的镜像写入，继续配置。

- 续制卡的第一次系统认证返回Request dismissed，未执行；用户明确要求“现在重新弹出”后再次通过认证，已进入内核在线扩容，未重写9.18GB镜像。

- 内核在线扩容成功，rootfs约61.4GB分区；SD新root UUID477ec00d-1019-49ad-85a8-806e31d03283、boot UUID D131-4571，与独立eMMC镜像不同。已配置Wi-Fi/SSH公钥、已登记主机身份和只读启动状态服务，正在复制eMMC安装img为SD内普通文件。只读小范围配置检查通过，不做全盘读回/哈希，未上机实测。

- 在线扩容后已在新SD根文件系统中通过小范围只读配置检查（UUID/fstab/bootenv、公钥、first-run shell语法、状态服务链接）；所需ARM版sgdisk/parted/resize2fs/sfdisk/python3.10/ssh-keygen均存在。一次盘点脚本使用了电脑Python3.8不支持的Path.readlink，改用os.readlink完成；不影响制卡。

## SD安装卡完成
- prepare_install_sd.py最终退出0，写入与卡内独立镜像复制均为9,181,331,456字节，fsync成功；已正常卸载boot/root并flush。配置/状态详见raw/sd_ready_report.json。未执行额外全盘读回/哈希，未上机验证。
- 安装卡启用只读启动状态服务和密钥SSH，生产eMMC安装器草稿仅存电脑，不自动写eMMC。
- 临近收尾尝试只读读取机载任务记录时SSH banner超时；未确认当前所有任务空闲，未关机或停止任何机载服务。实际停机前须恢复连接并重新核对状态。

- udisksctl power-off对已确认的本次读卡器返回0，SD可安全拔出。未发出机载关机命令。

## SD启动已实测确认
- 用户已将SD插入Orange Pi并表示从SD启动。动态发现192.168.1.224，预登记安装卡ED25519主机公钥匹配，密钥SSH登录成功。实际root=/dev/mmcblk0p2、boot=/dev/mmcblk0p1及UUID均与本次SD一致，Ubuntu22.04.4/5.10.160/ARM64；Wi-Fi与SSH及在线状态读取已实测。
- eMMC现场容量250,139,901,952字节、CID d6290341335630313411560574f2bc00、GPT 1E66CE0A-2419-494D-A6D8-A332C78B24A1，非历史64GB案例。所有eMMC分区未挂载、无swap，旧系统Ubuntu20.04.6。机载当前USB列表只有root hub，没有运行ROS控制进程。
- 旧工程目录与先前审计235机载不同，现场有something1017/cooperation，无drone_platform。按用户“旧工程无需保留”停止工程复制并清除本次临时归档，未修改eMMC内容。
- 已把安装器及手动启动服务部署到SD（static、不启用自启动），未创建批准配置、未启动安装。首次状态在NTP前生成时间偏旧；时钟同步后重新生成，只读在线监控取得RECENT状态。
- emmc_write_review.json记录确切目标、覆盖范围和非覆盖范围；待明确此目标覆盖确认后安装。

## eMMC安装已授权并启动
用户明确回复“开始吧”，授权覆盖已审阅的256GB eMMC。按当前boot_id、CID、容量、GPT和SD根/boot UUID再次核对通过；在SD写入root0600批准配置，手动启动drone-emmc-install.service（无开机自启）。开始在线记录，继续跳过全盘读回和整文件哈希。

## eMMC写入及配置完成
- 用户授权后安装成功：systemd Result=success、ExecMainStatus=0。写入9,181,331,456字节并同步完成；root分区扩至249,033,654,784字节。
- 小范围只读核对新root/boot的UUID与fstab/bootenv一致，Wi-Fi及新SSH主机公钥已配置，预登记公钥保存emmc_known_hosts。目标分区检查后已卸载。无全盘读回或哈希，尚未从eMMC启动。
- raw/emmc_online_progress.jsonl与raw/emmc_install_service.log保存安装过程；raw/emmc_install_complete.json保存结果。下一步正常关机，用户拔SD再上电后，以新主机公钥及根/boot来源确认eMMC启动。

- 安装完成、日志归档后确认无ROS控制进程、eMMC分区未挂载，执行正常systemctl poweroff，命令返回0。未凭SSH离线宣称物理关机完成；待用户关机后拔SD并重新上电，再验证eMMC根/boot及新SSH身份。

## eMMC启动确认
预登记生产SSH主机公钥匹配192.168.1.224；root=/dev/mmcblk1p2、boot=/dev/mmcblk1p1，MMC CID与容量均匹配安装目标，SD未出现在块设备列表中。Ubuntu22.04.4、内核5.10.160、Wi-Fi/SSH和时间同步已实测。eMMC系统安装阶段完成，ROS2尚未安装，外接USB传感器/飞控尚未接入。厂商dnsmasq.service处于failed，需只读诊断，不擅自停/改服务。RGA和rknpu0.9.6内核初始化可见，但尚未推理实测。

- 开始后续Humble基础环境准备：从ros-infrastructure官方GitHub取得ros2-apt-source 1.3.0~jammy并安装；apt-get update正在运行，日志在机载/var/log/ros2-migration。厂商GPU/MPP/RGA/FFmpeg/相机锁定包保留，安装前先模拟依赖，拒绝移除/修改锁定包，不执行全系统upgrade。
- 查阅资料已有official/humble-install.rst；本轮最初尝试官方旧路径返回404，继续使用已归档官方文档。当前systemd/udev为249.11-0ubuntu3.12，不是早期Jammy初版。

- Humble依赖模拟通过：294个变化，无移除、无厂商锁定包变化；仅升级libsqlite3-0，其余新装。不升级内核/systemd/udev。已执行带--no-remove的实际安装，日志/var/log/ros2-migration/apt-install.log。ROS2独立工作空间Git初始提交03fe6d4。
- 一次日志读取猜测文件名apt-simulation.log不存在；实际为apt-install-simulation.log，已使用实际日志检查升级清单。

- 官方源下载走IPv6且部分包极慢。确认仍处纯下载阶段、尚未解包后，仅中断本次apt下载进程（缓存保留），改用Acquire::ForceIPv4=true、相同包集合/官方源继续；旧日志保存在apt-install-ipv6-attempt.log。未停止系统服务或改全局IPv6。

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

- 用户新增要求文件管理与.235统一。已将新机主目录改为~/drone_platform，按01_apps/02_drivers/03_tools/04_data分区；原~/ros2_platform保留兼容链接，基础workspace归入02_drivers/sensors/workspace，迁移记录归入04_data/logs/ros2_migration。

- 目录再次整理时发现config/docs/tools/records已经是指向规范目录的兼容链接；rename拒绝覆盖已有目录，未造成数据覆盖。只读确认四级结构及链接正确后停止重复整理。GeographicLib EGM96数据从.235只读复用到新机，无飞控参数改动。

- 用户态驱动安装成功（MAVROS2 2.16.0，Orbbec ROS2/SDK2.9.3）。原相机固件1.4.60无需更新已启动四流，SDK报彩色1280x720@30、深度/左右IR848x480@30。当前临时本机DDS数据验证使用独立domain175。
- 首次精简MAVROS插件缺command，sys_status版本查询超时；下一次验证增加command以支持版本查询，只采集状态/图像不调用控制服务。此插件暴露命令接口不等于发送解锁/飞行指令。

## 新增要求：版本与.235一致、NoMachine远程桌面
- 用户截图显示.224:4000拒绝连接；现场未装NoMachine、端口无监听，SSH/桌面正常。旧机NoMachine8.27.1-2；用户要求版本一致，必须使用该ARM64版本，不自动升级新版。
- 同时将相机SDK从已通过初测的apt2.9.3调整为官方ROS2 v2.10.6，对齐旧机ROS1 wrapper/SDK2.10.6。官方tag解析到296156521b60bdf813cb94c46cbb55562bbdc453；保留初测结果，源码overlay单独构建。MAVROS1→2无法以相同版本号替换，保留参数语义并显式记录差异。
- 原四流35秒验证通过；实际同订阅负载下彩色约14.94Hz、深度27.89Hz、左右IR约29.6/30Hz、点云约4.35Hz，不能把设备配置30FPS当所有ROS订阅者都30Hz。飞控IMU50Hz、本地位姿30Hz，connected=true、armed=false、landed_state=1。
- MAVROS command插件依赖修复后18个state样本连接正常、未解锁。相机缺失的compressed/compressedDepth/theora图传插件已补齐，以复用旧话题的压缩接口；仍需在最终版本复验。

- 首轮35秒联合验证全部通过：飞控connected且armed=false/landed_state1，IMU约50Hz、local pose30Hz；彩色实收14.9Hz、深度27.9Hz、左右IR约29.6/30Hz、点云4.35Hz。SDK标称30fps不等于所有ROS订阅端均达到30Hz，需分流复测。
- 首轮发现图像压缩插件缺失，已补充compressed/compressedDepth/theora用户态包；发现MAVROS2 sys_status依赖command做版本查询，补上后单独复测连接与未解锁状态正常。无飞行指令。
- 用户要求NoMachine及可复用驱动版本与.235一致。相机从初测SDK2.9.3调整为官方ROS2 v2.10.6，commit296156521b60bdf813cb94c46cbb55562bbdc453，在02_drivers/orbbec独立编译SDK/消息/驱动，不拿Noetic二进制运行。
- .224未安装NoMachine，端口4000未监听；.235 NoMachine8.27.1-2。多个官方旧下载地址重定向首页，未将HTML当deb安装。旧机/usr/share/NX/packages原始归档均为空，不能直接dpkg-repack得到可用安装包。继续寻找同版本可复用安装来源。只读sudo -n无授权，未索要密码。

- 用户进一步指定NoMachine改与电脑当前9.4.14一致，此项覆盖此前对齐.235的8.27.1要求。已确认电脑实际运行~/下载/NX/bin/nxplayer Version9.4.14，x86_64；目录中的9.5.7-4更新包并未启用且为amd64，不能用于OrangePi。开始查找9.4.14 ARM64官方安装包。

- v2.10.6官方源码构建完成（2包，6分36秒），确认实际SDK2.10.6/固件1.4.60，源包与消息overlay均从02_drivers/orbbec/install加载。无内核/固件替换。
- 配置同步旧版scp遇到带空格路径config script失败；改用tar流按相对路径传输，文件已完整部署，无命令插值路径问题。
- 大消息掉帧已定位到订阅QoS：BestEffort独立彩色16.76Hz、点云16.13Hz，Reliable/depth10独立彩色29.99Hz、点云30.35Hz；压缩视频29.98Hz。验证工具及迁移文档已按图像/点云Reliable、飞控sensor_data分别处理。正收尾联合验证。
- 电脑实际NoMachine9.4.14 x86_64，运行路径/home/tyzk/下载/NX/bin/nxplayer；目录内9.5.7-4更新包未启用，不能给ARM64安装。旧机NoMachine8.27.1-2；新机无NoMachine。官方多个8.27/9.4旧链接均重定向HTML，不安装不匹配版本。

## 最终联合地面验证（SDK2.10.6）

35秒联合验证全部通过。Reliable相机订阅下：彩色29.95Hz、深度29.97Hz、左右IR29.97Hz、点云30.01Hz；MAVROS IMU49.94Hz、本地位姿30.00Hz，connected=true、armed=false、landed_state=1。四流标定、光学TF、有效深度、时间戳单调与当前时间检查通过。未验证外部定位融合、机体外参或飞行功能。临时launch及其子进程均已停止。最终结果peripherals-20261116-174825.json；所有阶段结果与问题保留，不将早期掉帧结果覆盖。

NoMachine检查：电脑实际运行9.4.14/x86_64，.235为8.27.1-2/arm64，.224尚未安装且4000未监听；旧版本官方链接不可用，未安装其他版本。等待可用的目标版本ARM64安装来源，未迁移源机身份/凭据。

- 本地核实正在运行的NoMachine为9.4.14/x86_64，.desktop与进程路径均为下载/NX；未启用更新包9.5.7-4/amd64，不能安装到ARM64。9.4.14官方历史入口同样返回HTML，未将其当安装包。
- 最终代码/配置和官方v2.10.6源码已在机载Git提交6b4e5982fb0dc1435bbc1d71e2fac5d00d12d17c，SDK二进制由原始源码归档保存在install_assets并以版本/commit记录，构建产物不入Git。首次提交缺少作者身份，改为本次git -c临时身份提交，未改全局设置；第三方原始源码的空白检查问题保留不改，自己的文件diff检查通过。
- 最终验证与版本、包、安装/构建日志已归档电脑。测试MAVROS和相机容器均已退出，无开机自启动。NoMachine尚未安装，目标ARM64同版本安装来源仍待解决。

## NoMachine 9.4.14 ARM64获取尝试
用户要求获取与电脑9.4.14对应的ARM64包。继续读取既有记录，核对官方历史deb/tar、CDN、可能的打包修订和公开历史索引/仓库线索；当前旧链接重定向官网HTML，不是安装文件。GitHub精确版本公开索引未找到可用安装包；历史网页索引超时。未将HTML保存为deb，未安装其他版本或改动机载远程桌面服务。证据保存在raw/nomachine_*probes.json及raw/nomachine_github_version_search.json。精确版本安装包获取仍未完成。

## 更正旧版本获取结论与新证据
不能由已查官方旧URL失效推断全网无法找到。GitHub历史构建记录确认nomachine_9.4.14_1_arm64.deb曾存在：ranshalit/hadron_yocto commit20008ace2fffc3173e487cac6792842c874ce344，配方源URL为/download/9.4/Raspberry/...，声明sha256=3b34641d0fb28bf0bc96e01ab37c84ed8c34ed4a816d794047b85abd536e79ef（第三方历史记录，不等同于已取得文件）。另NixOS/nixpkgs commit0f906ee9b43802e9c40053e5af8240faac534aa3记录2026-05-08时9.4.14电脑版官方tar已撤下并重定向主页，说明问题并非只有ARM64。找到9.8.2 ARM64公共GitHub镜像，仅作来源线索，未自动更换版本。继续沿ARM配方和ConnectTech参考文章检索副本。

后续结果：ConnectTech页面403/API401，附件列表404；Yocto公共源镜像无该文件，OpenEmbedded源站TLS失败；ARM构建仓库未发现安装包副本。另找到nilsl88/container-fedora-xfce-nomachine PR5，明确引用9.4.14_1_aarch64.rpm，支持ARM版本历史存在。当前未取得可下载安装包；不能断言全网无存档。已向用户请求其看到的电脑版旧版下载来源，以便定向对照。

## 用户提供NoMachine安装包并授权安装
用户给出/home/tyzk/下载/nomachine_8.5.3_1_arm64.deb，随后要求放入安装工具目录并明确授权安装。dpkg-deb元数据确认nomachine8.5.3-1/arm64，约49MiB，内含实际nxserver/nxnode/nxrunner/nxplayer归档。已复制到新机drone_platform/03_tools/install_assets，保留电脑原文件。重新核对生产SSH公钥/eMMC CID/root来源，XFCE与lightdm现有可用，安装前4000未监听。dpkg安装日志写入04_data/logs/ros2_migration/nomachine-8.5.3-install.log；用户本次安装指令更新此前精确版本要求，不再要求额外版本确认。

## 用户提供NoMachine8.5.3 ARM64包
- 用户给出电脑/下载/nomachine_8.5.3_1_arm64.deb并指定放到安装工具目录。包控制信息确认nomachine8.5.3-1/arm64、约49MB，真实非空vendor载荷；已复制到机载03_tools/install_assets并保留电脑原文件。
- 已执行dpkg安装8.5.3-1（不是9.4.14或参考机8.27.1），返回0。XFCE入口为/usr/bin/startxfce4，nxserver/nxnode/nxd已启用，nxserver.service active/enabled。电脑到192.168.1.224:4000 TCP实测可达。未做账号登录及桌面交互实测。
- 安装器出现ufw规则配置警告，但4000外部TCP访问通过；未另改防火墙。dpkg --audit为空。安装日志保存在机载04_data/logs/ros2_migration/nomachine-install.log；结果raw/nomachine_8.5.3_install_result.json。
