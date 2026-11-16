# 完整地面站云台显示管道与Qt同步诊断

本阶段仅修改独立ROS2地面站副本的Linux显示调度及测试工具。机载代码仍为d896200；正式服务、底层驱动、相机固件、物理飞控均未更改。ROS domain83 / localhost用于机载隔离计算；真实主窗口经SSH/UDP显示，相机和NPU为实物，雷达输入为旧实采回放，云台为独立端口的合成1080p H264。测试SSH包装器只允许认证、只读遥测、视频及隔离录制，不允许任务执行。

## 结论与对照

| 测试 | 结果 | 含义 |
| --- | --- | --- |
| run7原设置完整GUI | 云台新鲜度失败 | readyRead约每秒64KiB，约10秒拼成一帧，其他流继续 |
| GUI代理环境/无代理独立GStreamer | 25秒715/709帧，最大间隔0.1644/0.2443秒 | 不支持代理环境导致慢管道的解释 |
| 完整本地主窗口/原控制器/640x360 RGB30 | 默认78、basic91、native89帧/约25秒 | 无SSH、网络、ROS也复现 |
| 同一GUI二进制QSG_NO_VSYNC=1 | 740帧/约25秒 | 本机显示同步等待与吞吐瓶颈有关 |
| QCoreApplication原始管道 | 约10秒300帧；禁glib305帧 | 无GUI时QProcess可跟上输入 |
| run8显式关闭vsync | GUI24/24、机载43/43通过 | 真实按钮录制/收尾及持续双视频、点云/位姿/状态联合通过 |
| run9产品默认设置 | 整轮失败 | 云台持续新鲜，双目USB断开造成一秒采样失败；另保留MPP观察器末帧不全失败 |
| run10产品默认设置+USB状态观察 | GUI24/24、机载43/43通过 | 无USB断开，实际录制按钮正常启停，证据取回/临时目录移除 |

图形日志实际为NVIDIA OpenGL。LIBGL_ALWAYS_SOFTWARE=1没有切换GLVND驱动，因此不宣称软件渲染对照。相同探针二进制摘要见probe-binary-manifest.json。原始管道计数是测试探针估计，产品控制器源码未修改。

run9机载降级失败检查仅执行7项，其中MPP源戳交付4项失败；不能把它写作43项通过或只把相机失败删掉。完整数据仍在电脑work目录，不上传大型bag。run9另有PCD1 UDP分片不全日志，后续帧继续显示，不声称UDP无损。

## USB断连证据

run9/kernel-usb.log：03:30:56设备11断开，03:30:58设备12重新枚举。相机日志先收到SDK设备移除通知，再执行onDeviceDisconnected、resetDevice线程清理；没有证据证明该reset线程先主动重启USB。断开前彩色/深度/IR约30Hz，无已知上游停流预警。是否涉及人工拔插/供电操作尚未取得用户确认。

断电续接时.224 uptime约9h、Git干净、NoMachine active。新旧机内核均5.10.160-rockchip-rk3588，USB power/control均auto、autosuspend_delay_ms均2000；旧机采流active、新机结束后suspended。这种事后差异不能证明自动休眠是原因，未修改USB电源设置。新机两份相机udev权限规则重复，旧机官方规则不同，但均未见电源设置，未替换规则。

## 候选实现、产物与回退

main.cpp在QGuiApplication前仅对Linux非Android、未显式设置QSG_NO_VSYNC时置1。Windows/Android代码路径不变，但本阶段未做这两平台运行验收。依据Qt6.4+场景图官方说明（归档qt-scenegraph-official.html），此配置让显示调度可改用定时器；可能改变画面同步/撕裂/功耗，其他驱动需验证。

r3生产包不含自动化注入。portable-result.json为7项归档清单/依赖/中文空格路径重定位/生产主界面首帧检查，均通过；SHA256为319a5331b378b22a12d966f8b1fa49efb87ca0f01c8be7daa36c949e15c34abb。位于电脑下载/科研飞行平台-ROS2地面站目录，旧r2和桌面入口保持。测试中的Qt设置不安装到机载。

实测显式QSG_NO_VSYNC=0仍出现swapInterval0（见explicit-zero），故不承诺该方式恢复同步。回退运行原r2独立包，或撤销main.cpp新增分支后重建。显式环境变量仍原样传给Qt。

## 验证边界

完整Qt界面自动触发既有QML按钮及grabWindow，不代替人工鼠标操作。run8是一次完整通过，run9失败保留；默认产品入口run10同样24+43项严格通过，采流期间逐秒只读观察USB为active，采流停止后才suspended；仍不能因此断言间歇USB故障已解决。未完成物理雷达/云台新机直连、安装外参、真实PX4融合、实际任务与长期热稳定性验收。本阶段不解锁或飞行。
