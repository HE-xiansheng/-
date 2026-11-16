# 实际 ArUco 检测器到降落控制器联合验证

此前检测器图像/地面站指标与控制器模拟Pose验证分别通过。本阶段将两个已部署原生节点实际连接：1920×1080合成图 → Safina_aruco_test → /aruco_poseresult → Safina_land_test → 隔离域中的模拟MAVROS订阅者。机载ROS_DOMAIN_ID=83、ROS_LOCALHOST_ONLY=1；飞控状态和模式均由fixture生成，不接真实飞控，也不调用真实MAVROS。command_gimbal_down与gimbal_down_on_startup均显式false，避免DDS隔离之外的物理云台网络操作。

21项检查通过，包括：实际PnP恢复光学偏移/深度；检测Pose保留图像源时间戳；静态与90度机体航向下目标映射到ENU；未触发时仅原策略待机；错误帧/过期/未来图像、空图/错误分辨率/非配置ID拒绝；模拟飞手OFFBOARD边沿激活后速度朝目标；1.5m高度断流进入LOST_HOLD、停止下降；新图恢复；模拟飞手POSCTL取消；全部速度/yaw_rate有限且限幅；两个节点正常退出。

这些检查使用实际节点和消息传递，但合成相机/机体状态不证明真实云台标定或闭环飞行性能。近地短时丢标记后继承的盲降策略未更改，1.5m断流测试不代表所有高度都保持。PX4接收/融合、物理云台网络与延迟、实测外参、SITL动力学及整站操作仍待验收。

没有修改运行代码或参数；机载仍bd6c781，源码/安装程序及fixture摘要见manifest.json。runtime/保留节点日志、逐帧诊断及阶段结果；临时目录已取回并删除。无需回退运行环境。

归档首次使用错误的detector.log/controller.log名称，tar报告缺失；检查实际文件名aruco_test.log/land_test.log后重新取回完整证据。旧失败命令未改变机载进程或代码。
