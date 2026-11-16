# 实物云台视频：实际 Qt 面板与断流恢复验证

本阶段新增测试入口，产品视频源码未修改。`qt.log` 中14项PASS，测试退出0；`result.json`确认临时转发已主动中断并恢复，`owner-result.json`确认临时relay正常退出0。

## 数据路径和范围

旧机.235原有实物云台/MediaMTX:8554 → 电脑仅loopback:21654 → .224仅loopback:18655 → .224独立临时MediaMTX:18554 → 电脑仅loopback:8554 → 实际VideoController、VideoPanel.qml、图像提供器与QQuickWindow。

测试调用真实按钮clicked信号；确认640x360解码、QML Image Ready、连续变化的像素、窗口渲染帧、关闭回收和重新打开。随后关闭本测试所属SSH隧道，确认失效画面和重连安排；重新建立转发后自动收取新画面，退避复位，最后回收全部接收进程。断流期间GStreamer连接失败是预期注入结果，原始日志保留。

采用offscreen/software渲染，未验证用户可见桌面、鼠标命中或完整主程序交互。按钮信号与实际QML组件已覆盖。未启动双目、ROS控制任务、云台动作或飞行任务。此路径也不经过MPP图像发布器，MPP吞吐问题见上一阶段gimbal-live，不由本测试解决。

新机云台网卡/物理直连仍未完成；测试借用.235只读视频，不证明.224独立物理接线可用。未测量端到端延迟、相机曝光时刻或机体外参。

## 构建与复现

构建源为groundstation/tools/video/tests，目标video_reconnect_test；使用当前电脑Qt6及独立build-groundstation-video-test目录。完整构建结果见build.log，测试驱动见tools/run_qt_gimbal_live.py。运行入口：

```bash
python3 tools/run_qt_gimbal_live.py /absolute/new-evidence-directory
```

这是当前电脑专用验证工具，SSH指纹路径、两台IP、Qt前缀和测试二进制路径须先核对；需要先构建测试目标。不要并发运行或复用已有机载临时目录。结果已通过，无需为归档重复测试。archived-sha256.json记录续接归档时源码和二进制摘要。

## 清理与续接

机载/tmp/gimbal-qt-live-check已取回并清除；临时relay和SSH转发已结束。续接只读确认.224源码c58b4d5干净、NoMachine active；.235原MediaMTX仍PID1419。Qt/GStreamer缓存为可再生运行缓存，未纳入Git归档，其余日志及原始机载证据归档均保留。

本阶段未安装或启用正式服务，也未替换桌面入口。撤回新增测试代码不会影响现有产品入口。迁移整体仍在进行，下一步实物雷达、云台直连及物理PX4融合验收。
