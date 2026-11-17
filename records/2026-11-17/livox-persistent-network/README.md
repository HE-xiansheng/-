# .224雷达网口固定与旧机启动方式核对

2026-11-17用户授权“网口像.235一样固定吧”，随后要求“关于错误自同步，你可以参考一下.235的”。

## 已应用的网络与产品配置

- 板载enP3p49s0 / C0:74:2B:F9:C0:D8固定192.168.1.51/32，仅将192.168.1.158/32指向此口；Wi-Fi管理.224/23及默认网关.1保持，Wi-Fi仍为DHCP。
- 持久NetworkManager连接ROS2 Livox，UUID4fb850bb-6fb0-4380-aacc-7a641a3dc761，自动连接、优先级100。libnm unsaved=false且文件root:root/0600，已激活、ping3/3。整机未重启，不将配置持久化证据等同于重启实测。
- 新雷达序列号ARMCP6S0039858，旧机另一个ARMCP1V0038817继续使用.117/.50，不改旧机。
- 驱动目标JSON由.117改.158，host.51不变；产品提交d9caef8。新增定位入口数据就绪等待，产品提交7a3c0cb，部署三个文件SHA与电脑一致、机载Git干净，见deployed-manifest.json。
- 正式服务/真实MAVROS/飞行任务均未启用。全部本阶段记录只在电脑/GitHub；临时检查目录取回后移除。

## 旧机.235只读核实

现场路径已为/home/orangepi/drone_platform。原提示的/home/orangepi/lidar_tools/start_lidar_localization.sh已不存在，不继续按旧路径推断。

实际Safina_sensor.service→02_drivers/sensors/share/livox_ros_driver2/scripts/Safina_sensor.sh：依次启动MAVROS、Orbbec、USB相机、Livox和RTSP；FAST-LIO/转发自动启动段被注释。management/board.py与manage_stack.py为后续单独启动定位/转发的入口。现场ROS图仅/camera/camera、/livox_lidar_publisher2、/mavros、/rosout、/usb_cam，FAST-LIO未运行，因此ROS参数服务器中没有相应定位参数；不能声称取得正在运行的旧机LIO参数。

实际配置文件common.time_sync_en=true、time_offset_lidar_to_imu=0.0，与新机相同。新旧源码都有“首次LiDAR/IMU时间差>1秒→取一次固定偏移”的自同步分支。.235的驱动长期先运行，定位稍后手动启动；旧脚本未明确实现本轮新增的新鲜度等待，不能声称旧机也已有这个保护。

## 原生联合检查与修正

run4沿用已持久网络和部署JSON，同时启动驱动/LIO/转发/被动EGO。驱动线程先等3秒再释放缓存，启动时两路缓存消费进度不同，FAST-LIO记录Self sync time diff=-1.4368522167；定位源年龄1.45～2.84秒，转发因过期无输出、EGO状态STALE_INPUT。此轮严格失败，不修改源时间戳或接受过期定位。

修正保持ROS驱动1.2.6、FAST-LIO算法源码和原配置参数不变。在管理入口start_lio前，订阅真实点云/IMU，连续1秒确认数据新鲜、源戳递增且同处300ms范围，再启动原launch；无数据15秒拒绝启动。10Hz点云头戳为扫描起点，因此等待条件使用点云300ms/IMU100ms以及接收200ms的界限；最终链路原有源年龄/输出时效门槛不变。原始启动缓存年龄与等待轨迹单独保存，不能把它们当成不存在。

run5较早等待候选未满足就绪条件，未启动FAST-LIO，严格失败；同轮驱动退出-11。独立Context需要明确绑定Executor：首次空域负例暴露此问题，失败日志readiness-missing-sensors.log保留，已修正；ready-helper-positive验证真实流就绪且保留外层Context，ready-helper-negative验证空隔离域拒绝就绪。此前用全局Executor的run6实际就绪成功，但部署版本使用明确的独立Executor，源码SHA与正/负例均保存。

run6先等真实数据就绪4.508秒（观察到原始缓存最大年龄点云3.012秒/IMU2.746秒），然后执行60秒原生驱动→FAST-LIO→位姿转发→EGO地图被动检查；无目标点、无真实FCU或控制接口。运行38项通过；退出检查失败，因此整轮passed=false、38/39，不能写为全通过。

| 流 | 稳定频率Hz | 全轮最大源年龄s |
| --- | ---: | ---: |
| lidar | 10.000 | 0.302 |
| imu | 199.995 | 0.203 |
| odom | 9.998 | 0.076 |
| cloud | 9.999 | 0.079 |
| vision | 10.000 | 0.103 |

EGO取得587帧真实world云、469帧膨胀栅格（最多16987点），稳定状态WAIT_GOAL，输入新鲜、camera_init坐标系匹配。没有发送目标点或执行轨迹，不证明避障轨迹/真实飞控融合或地面站实物整链已验收。

run3的463对实际源戳匹配位姿证明固定位置/RPY零点相减一致（结果run3-pose-pairs.json）。三轴变化范围约3.3/1.5/2.6cm，无外部基准及现场运动记录，不作为静止漂移或定位精度结论。

## 未解决问题

Livox子进程在部分SIGINT退出后报exit code -11，SDK Deinit完成及Lddc析构打印之后发生。ROS launch本身返回0不能代表内部节点正常退出；本轮新增对子节点process has died日志检查，保留run6失败。无gdb/coredumpctl，/var/crash没有这次驱动报告，现有三份旧报告未动；不能凭析构打印确定根因或宣称已修复。驱动底层未替换。

部署版管理入口隔离回归21/21通过：启动/复用/停发、无关进程保留、前台终端中断和模拟已解锁拒绝等；使用录制传感器及域196模拟FCU，无实际飞控命令。证据已取回且/tmp/localization-readiness-check已移除，见manager-regression。正式实机FCU状态下的入口启停、真实融合、外参/度量精度、云台实物、整链地面操作与正式服务激活仍待完成。

## 恢复方法与证据

原DHCP连接UUID832dd3b5-8a1a-3c4b-93db-c09f90bdd28a未删除。若确需恢复，停止本次所属测试进程后停用/删除ROS2 Livox并重新激活原连接，从config-before.json恢复雷达JSON；用manager-before.py恢复管理入口并移除sensor_readiness.py即可撤销本轮就绪等待。相关恢复操作仍遵守用户对正式环境切换的授权边界。

完整配置、命令输出、旧机源码片段、原生三轮证据归档与SHA均在本目录。部署源版本7a3c0cb，电脑platform归档同步；旧机未启动/停止任何服务。
