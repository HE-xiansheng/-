# 人物跟踪原始消息交付基线（严格失败）

使用实际RKNN检测器、原生track_test、MAVROS2和官方PX4 v1.16.0 SIH。输入为预存bus.jpg、合成RGB/深度及重映射到/fixture的状态、遥测、RC和飞行服务。DDS domain87 localhost，PX4独立网络命名空间仅lo，经SSH stdio传递测试UDP；物理飞控不参与，未调用解锁或切换OFFBOARD。

- run1：519条ROS指令，内部出口及PX4各515条；最初4条缺失，35项严格检查有3项失败。
- run2：519条ROS指令，内部出口及PX4各511条；最初8条缺失（约0.4秒），37项严格检查有3项失败。
- 两轮所有收到的指令源戳、局部ENU→NED、机体FLU→FRD、yaw rate符号、掩码及目标ID均匹配；ULog解析到位置及速度输入。run2额外核对实际推理横向偏差驱动非零同符号body yaw rate。
- 功能包含2米停距、前进/后退限速、空白图、RGB断流恢复、ID会话重置、SWA保护/恢复、飞手模式退出、未解锁拒绝跟随。功能通过不豁免全流丢失，分析passed仍为false。

每轮保留实际输入归档、取回机载原始归档、fixture日志、分析和PX4 ULog。机载临时目录已由正常退出runner取回后删除。产品修改与后续候选验证另见../person-consumer-ready。

限制：静态测试图及合成遥测，不验证真实运动目标、相机到机体外参、物理飞控融合或飞行性能。FRAME_BODY_NED源yaw为0且被mask忽略，不宣称覆盖任意body绝对yaw。
