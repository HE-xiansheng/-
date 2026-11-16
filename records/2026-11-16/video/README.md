# 视频阶段验证（2026-11-16）

断电后恢复：.224可连接、NoMachine active、平台父版本5859393干净。未修改.235或正式地面站入口，未解锁或执行飞行任务。

- person_preview沿用yolo_ultra_test模型、RKNN私有运行库，只订阅相机和发布标注图，保留源header。
- ros_image_bridge原生rclpy，Reliable1，原VID1/JPEG协议与640宽/8Hz限制保留；所有模式检查源戳与编码后新鲜度。控制节点不会因查看跟踪/ArUco画面而启动。
- 人物预览2秒发现窗口、UID/domain锁、已有发布者复用、窗口接管、所属进程回收及父死亡信号。

31项机载检查全过（native-final.log/results.json）；26项Qt回归（qt-result.log）；7项真实SSH到Qt（ssh-qt-result.log）；本地最新帧/源戳/断管3项。native-first.log为31项扩展前的27项通过日志。构建与源代码/二进制摘要一并保存。

重现：在.224以orangepi加载env.sh，使用ROS_DOMAIN_ID=198、ROS_LOCALHOST_ONLY=1，把tools/test_video_ros2.py、ros_image_bridge.py与bus.jpg置于专用/tmp目录；运行python3 test_video_ros2.py <目录>。电脑tools/test_video_ssh.py使用独立域199和有期限的合成源；CMake测试位于groundstation/tools/video/tests。电脑无需ROS2。

边界：合成图像经过实际NPU与真实SSH，未验收物理云台RTSP、真实相机延迟/负载、任务控制联合或SITL。无飞控发布/参数变更。定位/规划/录制Qt入口仍待迁移。

回退：不覆盖原地面站及桌面入口；停止本独立副本即可。机载新增person_preview无自启动服务，正常视频关闭会回收自身启动的预览；需要代码回退时以5859393为基线，在独立构建目录重建yolo_ultra_test。原跟踪节点及模型未替换。
