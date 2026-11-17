# 基础服务开机自启动部署（已启用并通过实际重启验证）

用户在明确范围后回复“ok，继续吧”，批准雷达/IMU、Gemini335L、MAVROS和云台视频基础服务的开机运行。FAST-LIO及位姿转发、规划、航点、ArUco、人物任务和录制保持原有按需流程，不因上电启动任务。

本次启用Safina_sensor.service（四模块）和gimbal-rtsp-relay.service（MediaMTX）。Safina先等待实际USB设备、雷达.51和云台.144.100固定地址，异常退出5秒后重试；MPP读取本机8554/gimbal转发，避免直接对云台重复拉流。驱动、SDK、飞控参数和算法配置版本不变。

云台使用本机实际USB网卡00:E0:9A:5A:F4:20，永久192.168.144.100/24，autoconnect priority100，无默认路由/DNS。原DHCP profile保留；Wi-Fi .224 DHCP及雷达.51/32到.158路由不变。

安装前只做被动PX4心跳检查（串口发送0字节），确认三条未解锁心跳；所有服务以orangepi运行。安装服务文件需要系统管理员认证，凭据只经不回显输入和sudo标准输入处理，不写入日志/环境/参数或GitHub。系统单元安装失败会清理本次已创建单元。

回退仅针对本阶段新服务和新网络：

```bash
sudo systemctl disable --now Safina_sensor.service gimbal-rtsp-relay.service
sudo rm /etc/systemd/system/Safina_sensor.service /etc/systemd/system/gimbal-rtsp-relay.service
sudo systemctl daemon-reload
nmcli connection down uuid 15ad846c-9c2f-4f0f-b867-ca25838e7d6e
nmcli connection delete uuid 15ad846c-9c2f-4f0f-b867-ca25838e7d6e
nmcli -w 0 connection up uuid bdf7de49-dfe9-3b15-85b0-9cc6241c73c1 ifname enx00e09a5af420
```

服务文件的原版本在机载提交1de5c3d及GitHub历史中，新等待助手此前不存在。回退不重装系统、替换SDK或写飞控参数。当前基础服务部署与上次全负载失败分开记录；定位误同步、规划输入过期和灰度全帧交付仍需继续解决。

## 安装后实流验证

两个单元已安装并enable/start，运行稳定且NRestarts=0。首轮Python检查器同时接收大图像、自定义点云及飞控消息，多路各约6.16Hz，导致速率检查失败；保留activation-result.json和initial-runtime-check.log。用先前实际编译/测试的C++观察器采集图像/原始雷达元数据，Python只读取飞控小消息，再按原有速率、源戳顺序及帧龄门槛检查，32/32通过。native-check-inputs.json记录观察器二进制来源及SHA。

实际LiDAR9.998Hz、IMU199.999Hz、双目彩色深度29.965Hz、云台灰度29.996Hz/彩色15.038Hz、FCU IMU49.997Hz/位姿29.998Hz。PX4始终未解锁，没有外部定位或控制设定点发布者。此检查仅证明基础自启动范围，不替代上轮全负载、外参、运动精度及实际任务验收。

网络配置时nmcli拒绝connection.uuid手工赋值；改为系统生成UUID，实际为15ad846c-9c2f-4f0f-b867-ca25838e7d6e。错误的连接列表查询字段也保留在日志，改用UUID,NAME字段后继续。服务语法检查有两项原系统单元权限警告，退出0，未修改这些系统/BSP单元。


## 实际重启验证完成

实际重启前后boot ID由`0de97dc1-35d4-4017-9098-634ce01a8627`变为`98055533-8b20-4466-9b95-dbffe318060f`。重连后只运行只读检查器，没有手动启动服务；两个单元均enabled/active，NRestarts=0。重启后32/32通过：LiDAR9.999Hz、Livox IMU200.004Hz、双目彩色30.000Hz/深度29.977Hz、云台灰度30.060Hz/彩色14.997Hz、飞控IMU49.998Hz/位姿29.999Hz。飞控保持未解锁，无外部定位或控制设定点发布者。

最终结果见`verified-activation-result.json`和`verified-boot-check-result.log`；完整原始证据在两个`verified-*.tar.gz`，摘要见`evidence-archives.json`。机载临时证据已取回并删除，产品源码29838fa干净。先前认证对话框超时、Python观察器过载和网络命令错误均保留，不将失败改判。

## 当前地面站连接修复

用户实际窗口来自电脑`dianyunGCS-ros2-20261116`的早期独立ROS2构建。其profile原domain175与基础服务domain0不一致，已改为0并只重连只读桥接，没有替换整个源码树或正式桌面入口。

对齐domain后旧Python采样器多路仍仅约6.2Hz。现场401327字节/20064点CustomMsg用官方Humble解码需约138–182ms，超过100ms雷达帧间隔。只读飞前采样改为raw订阅，经CDR封装、端序、字符串、纳秒、序列点数与完整长度校验后，只取原始时间戳/坐标系/点数；实际点云显示仍使用原世界点云链路，不改算法、飞控或传感器源戳。

`gcs-sampler-check/metadata-oracle.json`保存官方现场消息往返、21组官方序列化对齐/UTF8/点数边界、大端官方解码及16种损坏输入拒绝检查；共41项通过。另发现旧快照在图查询前取时、回调随后更新缓存会产生负接收年龄，已改为取得缓存锁后取快照时间，原采样成本及周期仍从循环起点计；不改任何新鲜度门槛。原代码负向复现和修复后14个回归测试日志均保留。

第一次35.9秒GUI观察存在8个快照状态闪红，保留`gui-observation-attempt1.*`，不能用其正确频率掩盖状态失败。最终持续观察结果另见`gcs-sampler-check/live-results.json`；旧版修复前后源码、profile和当前窗口接收的PFS1快照已归档。机载定位/规划/任务仍保持手动；未启动时相关缺失、EKF无有效位置、遥控缺失和电量未知按实况显示，不强行改绿。

地面站回退可从`used-gcs-reader-source/`恢复旧采样器与runtime，从`used-gcs-profile-before.json`恢复旧隔离配置后重连；基础服务回退命令见上文。回退是保留可撤销操作的方法，不是本次要执行的步骤。
