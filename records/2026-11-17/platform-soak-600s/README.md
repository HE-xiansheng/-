# 600秒多模块并发地面数据检查

## 结果

同一机载.224、已部署d896200，正式服务未启用/切换。实际Gemini335L→RKNN预览、旧机实采Livox/IMU循环回放→实际FAST-LIO→位姿转发、合成1080p30 H264→MediaMTX→MPP、隔离副本录制同时运行。localhost domain83，不启动实际MAVROS或飞行任务。采样600秒，43/43严格检查通过；录制正常收尾。

| 流 | 采样消息数 | 平均Hz | 最大接收间隔s | 最大源时龄s |
| --- | --- | --- | --- | --- |
| 彩色 | 17979 | 29.964 | 0.207 | 0.518 |
| 深度 | 17981 | 29.968 | 0.210 | 0.521 |
| 人物预览 | 9949 | 16.581 | 0.300 | 0.396 |
| 里程计 | 5686 | 9.475 | 0.549 | 0.297 |
| 世界点云 | 5685 | 9.474 | 0.550 | 0.298 |
| 位姿转发 | 5656 | 9.427 | 0.572 | 0.300 |

五条录制流稳定内部区间的源戳全部覆盖：IMU116415/116415、LiDAR5669/5669、里程计5670/5670、点云5670/5670、位姿转发5641/5641。覆盖采用各流源戳内部区间，和按接收时间计算的上表消息数不同，不把窗口差异当作丢帧或篡改结果。各SQLite分包检查通过。NPU预览头与两路独立相机源记录并集逐条一致，仍保留native-only诊断。

MPP实际发布灰度19083、彩色9539帧；原生reliable和BEST_EFFORT观察者均与全部发布源戳逐帧一致。源先停止、队列排空后才停止观察，沿用已验证的严格收尾流程，没有为长测放宽门槛。

## 温度、CPU与USB

119个约5秒间隔的系统快照：最高温度84.076°C、最低MemAvailable9.816GiB（包含tmpfs录包负载）、整机CPU忙碌率均值56.26%，区间54.16–59.98%。当前频率采样见resources.png，数值及计算结果见summary.json、result.json；plot_resources.py可重建图。

policy6的scaling_max_freq在约398.3秒短降至2016MHz，403.4秒恢复2400MHz；约559.8秒为2304MHz，564.8秒恢复2400MHz。其余采样为2400MHz。policy4上限2400MHz、policy0为1800MHz。ondemand下当前频率变化本身不等于热降频，最高频率限制的原因也未由本轮证明；本次未采集同步cooling_device状态。后续只读温区trip点为75/85/115°C，不能把事后阈值直接当作这些短暂变化的因果证据。数据检查通过不能替代长期温控/散热验收。

USB相机在119个采样点均存在且runtime_status=active，未修改control=auto设置。采样开始至结束后的内核记录没有usb disconnect，仅结束时UVC接口重新绑定日志；5秒采样不排除短暂未捕获状态变化。本轮未复现之前run9的真实USB断连，不认定故障已解决。

测试期间另外进行了约20秒的物理PX4固件/EV参数只读查询，记录见../px4-physical-readonly。该查询无ROS节点、无估计器输入/控制指令，但增加少量串口/解析负载；不能将本轮CPU称为完全无其他活动的纯基线。

## 源码、数据与清理

本轮仅为测试resource采样增加USB/CPU频率/墙钟信息，产品代码/参数/驱动未改。inputs-manifest.json约束实际输入tar及测试/分析源码摘要。大文件原始包和SQLite仅留电脑/home/tyzk/drone-migration-work/platform-soak-600s，GitHub保存结果、必要日志、数据摘要。取回/清理状态以runner-result.json及final-state.json为准，不能只根据本README推断清理已经完成。

## 完成范围与下一步

这是一次600秒隔离计算/录制联合通过，不包含完整Qt窗口、物理雷达/云台直连、独立USB相机、机体外参、真实PX4接收融合或实际任务。Qt完整主窗口另有../gcs-render-pipe的24+43项证据。雷达/云台接入、网络候选确认和真实地面整链仍是整体迁移的必要步骤。


收尾实读：runner返回0、evidence_retrieved=true、remote_removed=true；两处测试/tmp目录不存在、所属进程为空、机载d896200干净、NoMachine active，见final-state.json。原始归档1983618450字节，SHA256为5bfcf8676d8d1bcdb22e8744d50754ba5629d1796b51b4fcdb97293f54f30ceb；108个SQLite分包共3690205184字节，电脑路径与清单见computer-data-manifest.json。GitHub无大型bag副本。

保留告警：启动时Color preset不支持、FAST-LIO初始化空点提示；末段相机报告一次Color frame queue overflow（capacity_frames=1）；SIGINT退出后出现一次rcl context无效的帧回调错误。日志原文保留，进程最终退出；不能将43项通过描述成相机每一源帧必达、无告警或该退出回调问题已修复。此轮未修改这些产品行为。
