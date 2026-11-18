# 待确认的定位会话停止与验证

本步骤已于2026-11-18完成。用户明确授权继续后，原定位进程曾被内存耗尽杀死；管理器在重新门控后停止残留转发，随后安装并启动v3。当前正式launch/mapping/relay为1760834/1760835/1764424。原只读预检和停止记录仍保留在`../2026-11-18/localization-authorized-fix/`。

实际执行的停止入口（已完成）：

```bash
source /home/orangepi/drone_platform/env.sh
python3 /home/orangepi/drone_platform/01_apps/fast_lio/manage_stack.py stop
```

管理器持有定位/规划/执行器锁，核对PID、启动时间、boot和DDS来源；只处理所属定位与转发。雷达、IMU、Gemini、MAVROS、云台视频与基础服务保持运行。

随后完成无旧LIO的原始双QoS交付对照、v3单实例默认Fast DDS及64MiB profile对照、90秒稳定段和60秒暂停恢复段；均通过原门槛。v3已按固定SHA原子安装并由管理器启动，120秒和300秒正式只读验收均通过。

影响：停止期间无定位位姿继续转发至PX4，地图原点已重建，旧地图选点和航点需重新选择。操作期间未启动任务、未切模式、未改参数、未解锁、未起飞。电量和遥控器仍不属于本轮修复项。

当前正式状态和回退资料已记录在`../2026-11-18/localization-authorized-fix/`。固定SHA安装前回退包为`fastlio-v3-preinstall.tar.gz`；正式软件提交为机载`2ebe64fb4d5e208f2758a2d47499e88c981ed816`，电脑候选和记录已推送GitHub。若需回退，先停止所属定位/转发并重新门控，再恢复该包，不恢复旧地图原点。
