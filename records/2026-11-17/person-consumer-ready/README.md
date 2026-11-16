# 人物跟踪：MAVROS启动接收端与路由等待

原生track_test曾在MAVROS输出路由完成初始化前发布，严格基线两轮分别缺最初4/8条消息（见../person-mavros-wire）。候选增加非阻塞启动等待：检查实际/mavros/setpoint_raw的PositionTarget读者，读取其thrust_scaling参数并收到DOUBLE响应，同时发现/uas1/mavlink_sink发布者后锁存就绪；只读请求超时1秒后移除并重试。启动等待期间不发送指令、不处理模式/SWA动作。就绪后沿用既有状态、位姿、RC、目标、地面保护和源戳检查。

## 验证

实际RKNN推理→原生track_test→完整MAVROS2→独立网络命名空间内官方PX4 v1.16.0 SIH，ROS domain87 localhost。图像/深度、遥测、RC及模式服务均为隔离合成输入，真实飞控不参与，PX4一直未解锁且非OFFBOARD。

- 冷启动run1：514源/514内部出口/514 PX4消息，37/37项通过。
- 冷启动run2：511源/511内部出口/511 PX4消息，37/37项通过。
- 覆盖局部ENU位置保持和机体FLU速度跟随的NED/FRD转换、源毫秒戳、mask、非零yaw rate符号、ULog位置/速度输入，以及实际NPU横向偏差、2米停距、正反向限速、目标/RGB断流恢复、ID会话、SWA保护/恢复和飞手模式退出。
- 控制器39/39项通过（原36项+观察者不足以就绪、缺路由仍等待、消费者和路由迟到后恢复3项）；实际NPU联合14/14项通过。fake节点仅在隔离夹具实现实际消费者身份/只读参数和空路由发布者，产品无绕过开关。
- 扩展分析器加入body FLU→FRD检查，并拒绝NaN/Inf。既有航点34、EGO43、ArUco44项保存基线回归仍全部通过。

两轮候选和53项回归使用相同二进制SHA256：faa167cb32cae0b182489f16eb9aed3c98f5c62fcfe78fd16195f4390c05aa41。测试输入归档与最终4份源码逐字核对；回归脚本也与实际执行文件核对。runtime-inputs.json保存实际安装配置与推理程序/模型摘要。

## 部署与还原

仅更新候选工程4份源码、installed二进制和package.xml；远端先检查旧版本dae36a8、工作区干净、控制器未运行及旧文件摘要。部署后机载版本73dd98c，工作区干净，NoMachine active，摘要匹配。未停止/安装/启用正式服务，未操作物理飞控。

旧二进制保存在电脑/home/tyzk/drone-migration-work/person-consumer-before-deploy/Safina_track_test，SHA256 e5c657d4b0c8f8617a284822b3554f8707ddc4b9cc25ec1405d89fac6da3cc62；旧installed package.xml在同目录。需要撤销时，在控制器未运行条件下revert机载73dd98c并恢复该二进制（0755）和旧installed package.xml，或从上一版源码重新构建。禁止运行本部署脚本重复部署：它故意约束到旧HEAD和旧摘要。

原始归档含完整候选构建/安装和二进制，避免重复展开；PX4 ULog、传输全流、配置、测试脚本及失败基线均保存在电脑/GitHub。机载两个测试临时目录已取回并删除，所属测试进程已退出。

## 尚未覆盖

这是合成输入的消息链路验证，不能证明真实动态目标、外参、实际停距或飞行闭环性能。FRAME_BODY_NED源yaw为0且忽略，未验证任意绝对body yaw。只读参数请求无响应超时分支未单独故障注入。首次就绪锁存不表示永久DDS无丢失或消费者重启已覆盖。当前雷达网口无carrier、缺云台直连网卡；物理融合、完整任务/GCS操作流程和正式服务切换仍待验收。
