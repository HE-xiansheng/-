# 待确认的定位会话停止与验证

当前定位和转发为本用户所属会话，launch/mapping/relay PID为73086/73150/73273。只读预检记录在`next-stop-readonly-preflight.json`；实际操作前重新执行未解锁、非OFFBOARD、状态新鲜、无控制发布者及无规划执行器检查。预检通过不代表已获得停止授权。

确认后，以orangepi普通用户执行现有管理入口，不使用sudo、不kill无关进程：

```bash
source /home/orangepi/drone_platform/env.sh
python3 /home/orangepi/drone_platform/01_apps/fast_lio/manage_stack.py stop
```

管理器持有定位/规划/执行器锁，核对PID、启动时间、boot和DDS来源，仅对所属定位及转发发送SIGINT；不会自动强杀。保留雷达、IMU、Gemini、MAVROS、云台视频与基础服务。

随后先测不叠加旧LIO的原始双QoS交付，再用相同v3二进制运行私有输出、原门槛及恢复测试；使用`--require-formal-stopped`拒绝其他窗口重新启动正式定位，避免混入多实例结果。测试只读取真实源，输出仍在`/gcs_receive_fix/*`，不接入飞控。重做必要回放和单实例时效分析后，只有通过才准备最终安装清单和正式会话；当前过期v1安装器不执行。

影响：停止后无定位位姿继续转发至PX4，现有地图原点及选点不能沿用到新定位会话，需重新建图选点。操作期间不启动任务、不切模式、不改参数、不解锁、不起飞。电量和遥控器不作为本轮修复项。

如果仍失败，记录失败并保留传感器及MAVROS；不把旧源戳刷新后继续发送，不放宽0.2秒/0.4秒门槛，不把未验收候选切入正式路径。旧程序和配置已归档，仅供按条件恢复软件使用，原定位地图原点无法靠重启旧二进制恢复。
