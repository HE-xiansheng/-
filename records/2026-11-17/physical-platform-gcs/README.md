# 实物输入与完整地面站联合验证（进行中）

本阶段使用 `.224` 原生新雷达、Gemini335L和直连实物云台，同时运行已安装FAST-LIO、位姿转发、RKNN人物图像预览、被动EGO、隔离录制和实际Qt主窗口。ROS域83/localhost；GUI仅连接、读取状态/点云/视频及控制隔离录制。本轮不运行真实MAVROS、任务控制器或云台运动。

地面站是Ubuntu22.04/Humble适配的独立Qt版，使用原生ROS2机载采样与管理接口，经SSH/UDP及视频流连接。电脑无需安装ROS2；原桌面快捷方式尚未切换。主窗口通过不表示整平台通过。

## run1失败与恢复

原SSH日志确认真实1080p H264云台和全部处理流曾就绪，但新runner输出缓冲使外层90秒超时，Qt未启动。随后机载SSH255及ARP/ICMP失联，清理与取证未能确认；15:30三轮访问审计后目标受阻。固定主机公钥发现没有找到新地址，不能归因为热、内存、USB、驱动或断电。

用户“继续”后，15:33同一主机恢复连接，uptime及boot ID证明已经重启，产品仍为`1de5c3d`且干净。`tools/recover_physical_soak_check.py`实际执行通过：tmpfs上的原实验目录已消失，所属进程/临时网络不在，原网络与雷达路由恢复。原启动没有持久journal，pstore普通用户不可读，故故障原因仍未知、原始机载记录无法回收。见`reconnected/`、`run1/recovery/`。恢复助手仅接受目录检查0/1，SSH错误不能误判为“目录不存在”。

## run2默认参数：整轮严格失败

- 实际Qt主窗口24/24通过：真实SSH/跨机点云和位姿、双路视频/QML图像、30秒并发新鲜度、真实录制开始/计时/大小/正常停止、关闭一路保留另一路和全部停止。
- 机载43/51，8项失败：转发频率及最大间隔、规划输入过期、录包缺LiDAR5/IMU43/世界点云1，以及灰度图像两种QoS各缺同一帧。原始结果保留为失败；早期笔记写42/50是计数笔误，以原始`result.json`的43/51为准。
- 里程计9.903Hz，转发6.009Hz/最大间隔2.269秒。LIO启动记录`Self sync ... 1.2752034664`；外部readiness已经等待4.901秒并保持源戳新鲜对齐，不能仅凭此日志认定真实硬件时钟有1.275秒偏差。
- 云台灰度发布3207/接收3206，双QoS缺同一中间帧`1794901147417430989`；彩色1316/1316。全部9个处理进程退出0，峰温79.461°C、最小可用12.964GiB、OOM增量0，无本轮断网。

## run3关闭自同步的临时诊断：整轮仍失败

命令使用`PLATFORM_SOAK_PHYSICAL=1 PLATFORM_SOAK_DISABLE_SELF_SYNC_DIAG=1`，只在该进程命令行覆盖`common.time_sync_en:=false`，没有修改正式配置、源码或二进制。结果显式记录此覆盖，不能作为默认参数通过证据。

- 主窗口仍24/24；机载48/51。转发恢复9.988Hz/最大间隔0.282秒，里程计10.006Hz；五条录包数据流稳定内部窗口逐戳覆盖完整。
- 规划仍出现`STALE_INPUT`，灰度发布2618/接收2613，两种QoS缺同样连续5帧；彩色1110/1110。不得以关闭自同步作为永久修复，或删除best-effort交付检查使结果通过。
- 新增C++ `rclcpp::MessageInfo`观察，记录传感器header、DDS source/receive、callback wall/monotonic时间。稳定段IMU“发布减源戳”中位0.091ms、雷达103.466ms，与扫描头语义相符；启动阶段存在秒级延迟。独立观察器回调也会积压，因此尚未证明LIO内部误同步的直接因果，需要LIO内的时间配对/调度进一步取证。
- 峰温81.307°C、最小可用12.786GiB、OOM增量0；处理进程均正常退出0。

两轮原始档案均已取回电脑；所属进程、tmp目录和save-no云台网络清理通过，原DHCP云台profile、管理Wi-Fi和永久雷达路由恢复，NoMachine active、产品源码干净。内核流直接写电脑`fixture/kernel-live.log`，资源采样每秒流式写`fixture/runner.log`，可在线读取。云台持久网络、正式服务/桌面入口、真实全负载飞控融合、安装外参/运动精度、实际目标及长期稳定性仍未验收。

## 原始证据与复查

每轮`result.json`、`qt-result.json`、`fixture/runner-result.json`及展开的参数、日志、元数据均保留。`analysis.json`由以下工具从原始记录生成，不修改验收结论：

```bash
python3 tools/analyze_physical_platform_check.py records/2026-11-17/physical-platform-gcs/run2
python3 tools/analyze_physical_platform_check.py records/2026-11-17/physical-platform-gcs/run3-no-self-sync-diagnostic
```

完整`fixture/evidence.tar.gz`超过GitHub单文件限制，按40MiB分片保存在各轮`fixture/evidence-parts/`，原档与分片SHA256见该目录`manifest.json`。电脑保留原档和展开数据库；GitHub不重复保存展开的db3和observer-build。重组步骤见分片目录`README.md`。原始失败、源码输入摘要及诊断覆盖均保留。
