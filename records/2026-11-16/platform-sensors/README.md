# ROS2传感器启动候选与实机地面检查

机载部署：`599330685bef0c58a54883caf6b0265cc8e85e00`。本阶段仅添加候选launch、shell和service源码；未安装/启用systemd单元，未替换现有入口。NoMachine保持active。完整迁移仍进行中。

## 行为

参照旧Safina_sensor的五模块职责：完整MAVROS、Gemini、独立USB相机、Livox及RTSP图像。原ROS1的固定延迟、批量kill -9及删除锁操作没有移植。定位/位姿转发/任务控制不由该服务自动启动。候选提供MAVROS命令API，但本阶段没有调用解锁、模式切换、起飞、参数写入或任务执行接口。

选中模块先完成软件/串口/独立USB身份/Livox本机IP检查再启动。不能把预检等同实际设备数据健康。任意子进程意外结束，launch报错并关闭其余子进程。日志路径由service指定为/run；候选默认五模块全开，当前缺硬件时不可直接启用。

## 验证

`platform-sensors-audit/run2/result.json`：21项检查通过，包括空选择/错误布尔/缺串口/缺USB/缺雷达IP启动前拒绝、参数枚举、真实连接及数据、完整插件接口和正常SIGINT退出。使用域81、localhost，由orangepi运行；真实飞控与Gemini335L，独立USB/雷达/RTSP关闭。

约25秒稳态接收窗口：

| 数据 | 接收数 | 接收率Hz | 源时间戳时龄范围ms | frame_id |
|---|---:|---:|---:|---|
| 彩色 | 749 | 29.97 | 44.3–119.5 | camera_color_optical_frame |
| 深度 | 749 | 29.95 | 64.9–112.0 | camera_depth_optical_frame |
| 飞控IMU | 1251 | 50.01 | 0.4–22.4 | base_link |
| 飞控本地位姿 | 750 | 30.00 | 0.5–27.1 | map |

四路源戳逐条递增。收到的State始终armed=false；末段原有模式AUTO.LOITER，仅观察未设置。位姿有数据不代表雷达外部视觉融合验收。

`unexpected-exit.json`：使用实际候选退出回调和无害Python子进程，故意返回7，launch返回1，剩余长运行子进程被回收。不是实物传感器拔插测试。

`systemd-analyze verify` root执行返回0；系统已有rkaiq_3A权限告警保留，未擅自更改BSP服务。非root验证另有现存netplan权限告警，见相邻platform-sensors-startup/environment.txt。

## 已知问题及边界

- 首轮图像订阅使用BEST_EFFORT，彩色接收率未达到20Hz门槛；相机日志显示配置30fps。首轮脚本在断言失败前未保存具体接收率，此证据缺口如实保留。改用此前物理预览已验证的RELIABLE depth20后通过；复测结果不能确定根因，也不能声称BEST_EFFORT问题已修复。原始失败日志/脚本保留。
- MAVROS启动有timesync高RTT、参数拉取重试及版本请求超时告警，不能从短时topic时龄推导PX4时间同步已充分收敛。
- 第一轮相机退出时有context invalid日志；第二轮MAVROS退出code=-2（SIGINT），顶层launch返回0。所有测试子进程均退出，不隐去这些原始输出。
- 雷达网口carrier=0、无192.168.1.51；没有自动修改网络。独立USB、云台及RTSP物理数据仍未验收。
- 正式服务切换、全模块联启、真实雷达→飞控融合、视觉/航点/地面站物理整链仍需完成。

## 保存与撤销

源码逐字节核对机载与电脑，测试脚本与实际运行副本一致；摘要见tested-source-sha256.txt。证据先取回电脑，再删除机载本阶段/tmp/platform-sensors-audit。测试进程已清理，机载Git干净；记录仅电脑/GitHub。

当前尚未激活服务，无系统服务可撤销；停止候选前台会话使用Ctrl-C，现有入口继续可用。若以后需要安装/启用服务，应先核对外设和IP并取得用户确认。
