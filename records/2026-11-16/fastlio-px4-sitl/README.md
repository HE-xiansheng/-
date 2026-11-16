# 实际 FAST-LIO / 位姿转发 / MAVROS / PX4 回放联合验证

18项通过，见analysis.json。输入此前从.235只读采集的MID360/IMU短样本；按测量时刻与点偏移循环发布，仅对雷达与IMU同步平移源时间，保留测量间隔。机载实际FAST-LIO、原生vision2mavros、MAVROS2和电脑官方PX4 v1.16.0参与，未伪造/mavros/state或中间定位输出。

输入711帧雷达、14611条IMU，产生665条里程计、564条转发位姿，564条均匹配实际MAVLink VISION_POSITION_ESTIMATE微秒源戳且唯一递增，ENU/NED位置最大误差约2.33e-10米（编码浮点误差）。转发100帧初始化、camera_init/body→map及期望接口检查通过。停止传感器回放后5秒内无旧位姿重发，原生管理器正常停止所属会话。全部真实MAVROS观察状态未解锁。

## PX4融合与时间

独立PX4 user/network namespace只有lo，机载ROS域82、ROS_LOCALHOST_ONLY=1，回环UDP通过SSH有长度边界stdIO传输。仿真单独EKF2_EV_CTRL=3，未更改真实飞控参数。运行65/85秒自动读取uORB视觉输入与水平/高度aid状态，两次均fused=True且innovation_rejected=False。退出时的第三份listener可能保留旧aid状态，不能把其fused布尔值单独当作实时健康证据。

独立观察窗口76对里程计/位姿源戳完全一致；视觉时龄52.44–118.30ms（中位84.49ms），同期70条MAVROS同步RTT为2.70–33.30ms（中位3.50ms）。这是短观察窗口，不代表全程最大延迟。

ULog揭示PX4最初用接收时刻替代视觉源时刻：前410条timestamp_sample等于timestamp；约仿真时钟72.376秒后152条显示非零源时龄，最后100条为70.007–132.150ms（中位111.166ms）。官方锁定源码src/lib/timesync/Timesync.cpp的sync_stamp明确在sync_converged之前返回hrt_absolute_time；Timesync.hpp收敛窗口为500个样本。启动时仍有RTT过高告警。因此不能以ROS戳已正确编码或EKF融合成功推断PX4已采用源时刻，实机必须另验时间同步收敛。此处时间不是可照搬实机的固定等待时长。

## 覆盖范围和剩余工作

这完成旧机实采数据→新机实际定位/转发/MAVROS→官方PX4仿真的接口及停流验证，不是实物雷达新驱动采集、真实PX4融合或轨迹精度验收。静态短样本循环不能验证运动/大姿态/长时漂移。当前.224以太网仍NO-CARRIER，硬件状态见onboard-manifest.json。

真实雷达接入和时钟、机体外参、真实PX4估计器配置、云台设备、人物/降落/航点与地面站实物整链仍待验证；不更改正式运行环境。

源码及部署/样本/二进制摘要见computer-manifest.json、onboard-manifest.json；实际测试与部署脚本摘要已比对。全部机载日志已取回电脑remote-runtime.tgz及解压目录；机载/tmp测试目录、测试进程清理证据见cleanup.json，机载bd6c781干净、NoMachine active。运行环境未变，无需回退正式配置；重新运行应使用全新临时目录。
