# 原生ArUco图像链到MAVROS/PX4消息验证

新增隔离验收工具，没有改动或部署产品代码。实际机载5b1d6b7、NoMachine正常；飞控/Gemini在位但未用于此测试，雷达网口DOWN、云台直连网卡未出现。

测试使用实际aruco_test图像检测器和land_test降落控制器、完整MAVROS2以及电脑官方PX4 v1.16.0 SIH。域83只允许localhost，电脑PX4网络空间仅lo；SSH stdio只连接临时UDP仿真端口，无串口访问。控制器遥测输入全部重映射到/fixture，模式/解锁服务也重映射为拒绝请求的隔离桩，控制输出仍经真实MAVROS送往模拟PX4。禁用云台网络动作；未修改物理飞控参数或执行物理飞行任务。

## 检查内容

从相机内参/畸变和0.035m标记生成1080p图像，实际PnP输出经过历史机体位姿配对、optical→ENU转换到原生降落控制器。检查图像源时刻/坐标系、飞机航向90度、空白/错误ID/尺寸及过期/未来帧拒绝，模拟飞手OFFBOARD边沿、横向控制、图像断流保持/恢复和POSCTL取消。扩展最终用例将标记居中，验证DESCEND负ENU垂向速度到MAVLink正NED垂向速度，核对服务客户端仅指向隔离桩。

线包分析保留严格全流匹配，要求捕获ROS控制消息和PX4线包数量/毫秒源戳/ENU-NED的PVA、yaw/yaw_rate及掩码一致；ULog应表明velocity输入且没有position输入。PX4全程未解锁、非OFFBOARD。传感器、状态和飞机运动为合成，不能证明闭环着陆、实物标定或真实机体动态正确。ROS订阅捕获数量不等同完整实际publish数量，未加入产品级发布函数跟踪。

## 已保留失败

run1图像链和取消等检查通过，但严格全流失败：765条ROS捕获/755条PX4线包，缺少最初10条；匹配到的线包几何/时刻一致，ULog速度输入通过。这证明有初始交付缺口，尚未定位在控制入口、内部出口还是路由发现，不能直接套用EGO根因。

run2新增居中下降场景时，测试代码在发送模拟遥测的线程内生成1080p透视图，随后诊断显示px4_mode_exit_or_telemetry_timeout，控制器CANCELLED，新增下降检查失败；这是测试程序阻塞触发保护，不能归结为产品下降算法失败。本轮749条捕获/709条线包，严格全流也失败；失败原样保留。最终工具预先生成所有图像。

## 复现

```bash
python3 tools/run_aruco_mavros_wire.py /absolute/new-output-directory
/home/tyzk/drone-migration-work/px4-sitl-venv/bin/python tools/analyze_waypoint_mavros_wire.py /absolute/new-output-directory --fixture-subdir aruco-mavros-wire
```

工具使用当前电脑专用SSH指纹、.224地址及独立PX4构建路径。fixture失败时runner仍取回归档，保留机载临时目录供核查；必须确认所属进程退出、证据取回后清理，不能直接重启覆盖旧运行。原始归档和ULog保留，可再生PX4 etc/test_data和Python缓存不重复展开。部署源码/二进制/config摘要见deployed-inputs.txt，输入工具源码见各轮inputs.tar。

没有产品部署或正式服务切换，故无运行环境回退操作；新增工具可直接停用。迁移记录只保留电脑/GitHub，机载仅使用/tmp临时目录。物理雷达、云台网络、相机机体外参及真实PX4融合仍待验收。

run3预生成居中图后，25项图像/控制/隔离/退出检查通过，实际进入DESCEND；869条ROS捕获/865条wire，缺少最初5条，另外1条wire未在ROS捕获中出现，严格36/39通过。退出时先等待进程而不再spin观察器，可能造成尾部采样缺口，尚不能仅凭本轮断言原因；最终工具增加所有进程退出后的0.4秒读取，并只读观察内部mavlink_sink。

分析器对原航点证据和EGO最终证据重新运行均保持通过（日志另附）。一次航点回归使用不存在的路径而失败，改用实际waypoint-mavros-wire-run1路径后通过，没有重跑或改动那些运行。

## 最终结果

run3图像/居中下降/隔离服务等25项均通过；严格全流36/39通过，869捕获/865wire，最初5条未到达wire，另1条wire未被ROS观察器捕获。finally中原先阻塞等待进程退出，没有继续spin接收末尾数据，最终工具新增停止进程后的0.4秒spin排空，只改观察逻辑，没有过滤首部或放宽门槛。

run4完整25项功能检查通过；严格39/42通过，870捕获、865内部出口、865wire。三个失败均对应最初5条（约0.1666秒）未到达内部出口和wire，其余毫秒源戳/PVA/yaw/yaw_rate/掩码一致，内部出口与wire集合完全一致，末尾没有未观察源消息。ULog278条offboard_control_mode确认velocity=true、position=false，且全程disarmed/nonOFFBOARD。启动缺口仍未修复，不能把25项功能通过写成整链验收完成；不能仅凭出口缺少消息区分ROS订阅发现、插件回调、内部publisher或RMW调度。没有对land_test部署修改。

分析器针对已存EGO最终证据及航点证据回归通过。下一步应为原生降落控制入口增加并验证真实MAVROS订阅/路由就绪条件，同时覆盖读取超时、探测恢复及原触发/取消语义，不能靠固定sleep或忽略初始消息解决。

cleanup.txt确认机载仍5b1d6b7干净、NoMachine active，无所属测试进程；runner均取回证据，失败的run2临时目录在取回后人工清理，最终/tmp目录已不存在。分析器回归第一次使用不存在的航点运行目录，改为已有waypoint-mavros-wire-run1后通过，未因此重跑实验。
