# 云台RTSP转发候选与彩色限频修复

实际入口审计确认：当前03_tools启动脚本均调用ROS2，ROS1参考位于隔离目录；.224的ros2_migration目录为空，无迁移报告。旧机独立gimbal-rtsp-relay.service持续提供地面站云台原画，新机没有对应服务。新增工程内systemd候选，但未写入/etc、未启用或启动该服务。

两机MediaMTX均v1.21.1、SHA256 08d61a4c07d8cf124e037a86af09e616fce132e9c1edbbc673caa72cf9794810，保留gimbal.yml原始H264源和访问规则。候选使用orangepi和工程内程序，ProtectHome=read-only以允许读取工程，日志stdout。原旧服务为DynamicUser和/usr/local安装布局，不能原样照搬到home目录。start_relay.sh缺省保持原配置，额外配置参数只用于独立验证；check_rtsp.py缺省目标更正为.224。

只读网络证据：.235云台经USB网卡enx00e09a5b57c2，地址192.168.144.100/24直连192.168.144.108；.224当前没有该网卡/子网，目的路由会经WiFi默认网关。没有自动创建网络地址或连接实物云台控制端口。硬件接入时需确认对应网卡和地址，不能假定只装软件就能通。

## 隔离链路与发现

本阶段只用域80/localhost、两台本机MediaMTX实例、ffmpeg合成1920x1080 H264 15fps；relay配置沿用原文件，仅修改监听地址、TCP传输和upstream源为loopback。所有机载测试文件位于/tmp/gimbal-relay-ros2-check；实际系统服务不参与。

- 第一次服务器监听就被误当作流就绪：上游发布还未开始，转发源返回404并等待重试。测试修正为等待实际stream is available and online，首次失败保留。
- 第二次转发RTP和MPP灰度通过，彩色仅9.37Hz，未达门槛。实际节点日志同样记录9.0–9.5Hz，排除只是Python观察者收帧偏慢。源码发现每次发布把last_color设为当前时刻，15Hz输入的小抖动被当成未满周期而丢帧。
- 修正为steady_clock节拍累积，长停流后重新定期、不补发旧帧。ROS图像header仍为当前解码帧原有时刻，未修改成限频时钟。独立C++测试覆盖15Hz抖动、30→15Hz限频、5Hz慢源、停流恢复/不突发、禁用和非法值；电脑与机载CTest通过。
- 最终机载colcon独立/tmp构建、仅安装候选rtsp_test完成，不替换底层库或系统驱动。第一次节拍修正的彩色13.59Hz，仍有调度抖动损失，随后加入半周期准入预算，固定周期累加并维持平均限频；最终结果另存run4，不以中间通过掩盖差异。

电脑端实际通过SSH端口转发连接本机relay，以与Qt相同的GStreamer管线解码121帧，全部内容不同，640x360 RGB约15.10Hz。见desktop-result.json。这验证传输/解码参数，不宣称实际Qt视频控件显示、物理云台延迟或控制已验收。

MPP图像时间戳表示解码接收时间，不能用它声称曝光时刻对齐。真实云台网络、断流重连、曝光时刻/延迟和相机外参仍待现场验证。整体迁移继续进行，不自动切换服务或执行任何飞行任务。


## 最终复测与保存

run4/result.json：8项检查全部通过；1920x1080 mono8 478帧/14.87Hz，960x540 bgr8 474帧/14.79Hz；时龄分别5.9–55.2ms、16.7–93.5ms，源戳严格递增，frame_id均为gimbal_camera_optical_frame。这些时龄仅从解码发布开始计算，不含真实摄像头采集/编码和合成源前级时延。四个所属进程退出码0/255/0/0，FFmpeg的255为SIGINT退出。

机载版本d5eadc0，源码与电脑逐字节一致，安装二进制摘要见target-final-state.txt，未安装或启用systemd服务。测试脚本与run4实际副本一致；证据取回后删除机载本阶段临时目录及构建日志，源码和候选安装产物保留，NoMachine正常。

如需撤销本阶段候选变化，可在无人使用时恢复上一个机载源码版本5993306中的对应文件并只重编译rtsp_test；前一版可执行文件额外存于电脑/home/tyzk/drone-migration-work/reference/rtsp_hw_pub_ultra-before-rate-fix。没有正式服务切换需要撤销，不自动停止其他会话。

额外只读探测发现旧机.235的实物云台视频当前可读：H264约29.93Hz、2.061Mbps，5秒150个帧标记，见reference235-live-probe.json。尚未将此实物视频接入.224验证；下一步可以在保留旧机服务和网络的前提下，通过临时SSH隧道只读转发验证新机解码，仍不能代替.224物理网卡/直连网络验收。
