# 录制进程异常中断与人工恢复

本阶段使用 `.224` 上真实 Humble rosbag2 0.15.17、SQLite3、录制监督器及管理入口。独立 domain89 / localhost，输入为合成里程计和点云；只对本次创建并验证身份的监督器、rosbag2进程组发送 SIGKILL。没有断电、改服务、操作实际飞控或触碰正式录包。

## 证据与边界

- run1 测试把 `.active` 文件也算作分包目录，失败；改成明确的 `is_dir()` 过滤。
- run2 未考虑真实 rosbag2 自身 `/rosout` 消息，源消息哈希检查失败；修正为合成数据逐条匹配源CDR，日志也保留在SQL/reader全量集合比较。
- run3 的21项检查全部通过：异常状态、保留active标记、拒绝覆盖、分包存在、WAL保留、SQLite完整性、源消息内容与时戳、复制后重建索引、rosbag2原生reader全量一致、原目录摘要未变、没有飞行指令发布者。
- run3 已提交的三个分包共229条里程计、46条点云、4条日志。独立SQL/CDR解析给出首尾x=0.44/2.72米、时间轴8023毫秒；实际Qt PointCloudFileLoader倒序加载全部分包，229位姿/70000显示点及源选择、坐标、时间轴、光标检查均通过。offscreen验证加载器，未声称可见桌面操作完成。
- run3最后一个分包不读取WAL时连`messages`表都不存在，保留WAL时读取86条消息；前两个正常分包分别97/96条。这说明不能只取db3或删掉wal/shm后再尝试恢复。
- run4共25项检查全部通过，补充在与管理器相同的锁内归档旧状态指针、人工审阅后建立不同run的新会话、正常收尾且原目录摘要保持不变。

源发布数与已提交行数不同，测试只证明可读取已提交数据，不声称录制就绪之前、DDS未送达或尚未落盘的数据可补回。SQLite WAL + synchronous=NORMAL 在真实掉电时可能丢失最后已提交的事务（官方wal.html对应说明）；SIGKILL不是硬件断电测试，不证明eMMC控制器缓存持久性。

## 人工处理步骤

1. 先读取`recording_manage.py status`，确认监督器和bag进程均已退出。不要在仍录制时复制/重建索引；不要删`.active`，也不要对原目录运行reindex。若仍在运行，按现场情况处理，本步骤不授权停止正式服务。
2. 保存整个会话目录到电脑，包含所有`.db3`、`.db3-wal`、`.db3-shm`、active标记、状态、manifest及参数。原目录作为证据保留不动，记录各文件SHA256。
3. 在可用Humble环境中创建**新的副本目录**，保留分包与WAL的文件名关系。例如在独立机载临时目录中：

   ```bash
   source /home/orangepi/drone_platform/env.sh
   cp -a -- "$interrupted_clip_dir" "$recovered_clip_dir"
   ros2 bag reindex -s sqlite3 "$recovered_clip_dir"
   ros2 bag info "$recovered_clip_dir"
   ```

   两个变量分别为已确认不再写入的旧clip目录和不存在的新目录。复制要涵盖WAL/SHM；不能只复制`*.db3`。SQLite检查/读取只对副本执行，允许其正常恢复或checkpoint；不要在原件上使用immutable模式来声称WAL数据完整。不可读或结构损坏就保留并停止本次恢复，不能把失败副本标作已完成。
4. 对副本每个分包执行`PRAGMA integrity_check`，通过后逐条反序列化CDR，核对话题、数量、源戳、坐标系和数据长度；用SequentialReader复核消息集合。在电脑地面站加载全部恢复分包，核对轨迹/点云及时间轴。将恢复副本和验证结果留在电脑；清理本次机载临时资料前先确认复制成功。
5. 如果确需继续新录制，完成上述检查并经人工审阅后，归档根目录`.flight-recording.json`这一**旧会话指针**，不要删除原会话`.active`。归档前用管理器`current(root)`及旧bag PID身份确认都已退出，持有`.flight-recording-management.lock`并复读指针避免与启动竞争。归档命名如`.flight-recording-interrupted-<uuid>.json`。随后通过正常`recording_manage.py start`创建不同run的新会话；旧run保持active，因此地面站历史列表继续排除原始未收尾数据。测试run4执行这一流程并检查旧目录摘要不变。

此处是人工恢复说明，不是自动修复功能；原有启动拒绝未完成会话的行为保持。不存在的帧不能重建，索引重建成功也不等于任务数据完整。恢复方案仅用于录包，和系统恢复镜像无关。

## 工具

- `tools/run_recording_crash_check.py`：上传独立测试目录、运行、取回证据并清理机载临时目录。
- `tools/test_recording_crash_ros2.py`：实际进程异常中断及副本恢复验证。
- `tools/recording_recovery_viewer_check`：调用当前地面站加载器；期望值来自独立最小CDR/SQL解析。
- `official/`：rosbag2 0.15.17 reindex源码与SQLite WAL官方资料；两个README地址404按实际保留为失败请求。

未修改或部署任何产品录制逻辑。物理掉电、多传感器持续并发吞吐、实际雷达/云台接线和全任务链路仍待验证。

首轮Qt构建因系统CMake3.16过旧失败，改用电脑现有3.31工具；随后QJsonArray反向迭代接口不兼容，测试改为索引倒序，最终构建/实际加载通过。初次清理检查pgrep匹配了检查器自身，已保留原输出并用/proc中精确argv路径扫描复核，final-state.json确认无测试进程。
