# Ubuntu22.04安装到eMMC，SD用于启动/安装

用户已确定该部署方式。目标是原生Ubuntu22.04 ARM64 + ROS2 Humble，正常流程不恢复Ubuntu20.04，不重复制作完整恢复备份。现有恢复镜像只在失败且用户决定恢复时使用。

## 官方依据与镜像候选

官方5 Ultra wiki的“Method of burning Linux images to eMMC using the dd command”明确描述：先用TF卡启动Linux，将官方解压后的系统img保存到TF卡文件系统，核实eMMC设备，写入后正常关机、拔卡，从eMMC启动。本文只借鉴这一方法，不复制文档中的固定盘符、清零命令或示例Debian文件名。

官方入口：<http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-5-Ultra.html>。Ubuntu目录：<https://drive.google.com/drive/folders/1ca_0M3b1cZoXJP7rVUd2fUgdeAbCZbZk>。只读文件清单保存在`raw/ubuntu22_official_files.json`。

优先评估与现有BSP接近的官方历史包：`Orangepi5ultra_1.0.0_ubuntu_jammy_desktop_xfce_linux5.10.160.7z`，目录标注压缩大小1,624,268,933字节；保留桌面环境以兼顾原远程桌面/RViz操作。现行目录也有1.0.2/Jammy/Xfce/Linux6.1.99，作为官方候选，但仍需实测驱动组合。

镜像已下载并解压，已只读核对Ubuntu22.04.4、板型、设备树及分区。用户已要求跳过额外完整性测试；不执行后续整文件哈希或写后全盘读回扫描。实际历史检查结果见image_download_status.md。

## 实施顺序

1. **电脑准备镜像。** 下载、解压并只读检查镜像；保存来源/版本/字节数，按用户要求跳过额外完整性测试。迁移所需源码、配置、模型和SDK运行库另存工作目录，避免新系统安装后只能依赖未验证的上游替代品。
2. **制作SD启动/安装卡。** 重新识别本卡身份与字节容量；当前发现值是USB读卡器`/dev/sda`，62,534,975,488字节，exFAT根目录为空。提交确切镜像、目标卡和覆盖范围后，获准才卸载分区并整盘写入、同步并核对写入返回值与字节数。电脑NVMe不是目标。
3. **配置SD管理通道。** 依据该Jammy镜像机制配置网络、SSH、公钥及首次启动；保留硬件初始化与扩容。把已解压并完成板型/版本检查的独立安装img作为普通文件放入SD剩余文件系统，预算系统占用加img及日志空间；不把运行中的SD整盘当安装源。
4. **从SD启动机载。** 按用户原约定说明停机/换启动环境影响并确认后正常关机插卡。核实实际根/boot所在介质确为SD、板型和Ubuntu版本，验证网络/SSH及基本设备。插卡不等于已经从SD启动。
5. **核实eMMC并安装。** 在SD系统中重新读MMC类型、CID、字节容量、分区表、根/boot来源与挂载状态。eMMC用户区必须不是当前根/boot介质、所有目标分区未挂载，容量足够。列出此时实际目标和覆盖的旧分区/系统范围，按原约定确认后才写入已准备的img；不额外清零、不顺带写SPI或boot0/boot1。
6. **同步与记录。** 记录写入返回值/实际字节数及同步结果；按用户要求不做整盘读回/哈希。任何失败都不能记为写入完成。安装时不启动控制任务，不让两套MAVROS占用飞控。
7. **拔卡验证eMMC启动。** 正常关机后拔SD，再启动；核实根/boot确在eMMC，验证Ubuntu22.04、内核、网络、USB、相机、NPU/RKNN与MPP/RGA。不能把仍从SD启动的成功误记为eMMC成功。
8. **继续ROS2迁移。** 安装锁定Humble环境与独立workspace，按接口/传感器/定位/飞控接收融合/规划视觉/地面站顺序推进。真实飞控保持未解锁，不执行飞行任务。

SD写入、扩容、安装文件复制、网络/SSH与启动状态配置、同步卸载已完成；未执行机载关机/换卡、eMMC写入或任何SPI/boot区修改。当前结果见raw/sd_ready_report.json。

## 在线进度
按[在线监控方案](online_install_progress.md)实现SSH状态JSON与systemd日志读取、电脑端进度留档。写入百分比与同步阶段分别显示；不因断网或100%进度宣告安装成功。当前电脑读取工具已准备，SD安装器与服务尚未部署。

## SD制作现场更新
本次具体SD覆盖已获得用户“继续”确认，镜像写入及fsync已完成；GPT/root分区已扩展，内核在线扩容成功（跳过离线工具要求的完整e2fsck）。已配置网络/SSH与只读启动状态，正在复制独立eMMC安装img。最终以raw/sd_prepare_status.json的SD_PREPARED_UNMOUNTED及安全弹出结果为准；尚未从SD启动，尚未写eMMC或停机载服务。
