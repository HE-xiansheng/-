# ROS2录制、归档与地面站回放

2026-11-16，.224 Ubuntu22.04/Humble；机载最终代码 `b779e2373b910bf5dcdd7d24388a8c989ba9f943`。断电恢复后确认机载Git干净、NoMachine正常。.235只读核对部署版本（reference-sha256.txt），未停止原服务或录制。

## 结果

- 原生录制26项：手动请求、重复运行边界、默认关闭的自动策略、实际SQLite分包、BE点云、TF静态历史消息、源/配置摘要及参数采样、POSCTL继续、落地与锁桨共同确认、过期观测不触发、退出收尾、未收尾标记阻止新实例；无控制发布者。见ros2-result-final.json及native-final.log。
- 只读遥测20项回归通过（telemetry-result.json及telemetry/）。遥测仅读取管理器健康状态，不启动录制器。
- 远端列举/流读取13项，包括原ROS1数据、ROS2 SQLite、缺metadata、active、路径穿越/软链接及大小/时刻变化。见remote-reader-tests-final.log。
- Qt真实SSH按钮开始/停止、状态确认、保存及过期禁止开始通过（qt-ui-result.json、qt-ui5.log）。测试SSH包装仅添加目标指纹选项、将操作路径指向独立/tmp测试副本并设localhost域89；Qt仍执行实际SSH及机载管理器。
- Qt真实SSH目录列举、多分包传输、逐文件SHA记录与原生回放通过（qt-ssh-result.json、qt-ssh.log）。下载和播放不触发ROS任务。
- Qt逆序两分包合并240条FAST-LIO位姿、源时间4780ms、57600点；排除错误MAVROS局部坐标轨迹，回放游标可用（qt-replay.json/log）。截断CDR明确拒绝（qt-reject.log）。电脑无需ROS2。
- 独立主地面站构建成功（qt-build-final2.log），原桌面入口未替换。

## 实现与边界

机载保留recording_manage.py start/stop/status/prepare/auto-on/auto-off接口。SQLite/CDR默认256MiB分包、64MiB缓存、2GiB空间下限，WAL/NORMAL；保留metadata.yaml、参数JSON、源码归档与运行二进制SHA。自动停止需连续0.6秒新鲜落地且锁桨，避免两个异步话题瞬时混合状态。自动选项不自动开启，也不设置开机服务。

就绪只等待实际有发布者的话题；缺失话题独立上报。RECORDING不代表所有传感器或任务数据完整。持久化话题使用可靠transient_local；其余使用BE订阅。记录仅观察飞控，不提供解锁/模式/参数写入。

Viewer读取ROS2 sqlite3/plain CDR的世界PointCloud2与Odometry/PoseStamped，保留ROS1读取能力；不支持MCAP、XCDR2或压缩ROS2容器。Windows未编入SQLite时会明确提示不支持ROS2录包；未做Windows新构建验证。下载提供db3供查看器使用，机载metadata/源码/参数仍保存在飞行归档中，当前GUI不下载完整会话附件。

## 问题与验证限制

首轮Humble发现间隔只接受整数导致启动失败；第二轮两话题瞬时混合状态导致误停，增加持续确认并排空回调；Qt首次封装未关stdin、重复启动时前次测试录制仍在、仅有订阅的图就绪判定，以及遥测测试缺依赖/错误隔离域等失败均保留日志。最后通过的证据以上述最终文件为准。

尚未验收真实全平台持续吞吐、消息丢失率、满盘注入、真实断电后rosbag2 reindex与人工恢复、物理融合/视觉任务联合。此次保留active标记并拒绝覆盖，不自动修改中断录包。

## 版本与撤回

机载录制提交a103227、b779e23，基线39edbb1。仅增加录制运行源码，无开机启用或正式环境切换。若要撤回，先在地面站正常停止并确认保存，核对所属录制进程退出，再由维护者revert这两个提交；不能在活跃录包时替换源码。原ROS1 .235未动。地面站使用独立构建，可继续使用原桌面入口。

cleanup.json证明测试进程已退出、8个临时目录已回收、部署源码与测试摘要一致、Git干净且NoMachine active。完整原始测试录包/快照位于电脑drone-migration-work/fixtures/*evidence.tgz，未提交大体积录包。机载不留迁移报告；正常飞行录包归档是运行功能，未来由用户显式启动。
