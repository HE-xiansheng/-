# ArUco 前台操作流程恢复

旧机 start_aruco_vision.sh 通过图像话题复用已有 RTSP/ArUco 程序，等待图像、按需打开 rqt，Ctrl+C 只停止本次启动的程序。原 ROS2 包装器直接 launch 两个节点，缺少这些行为；本阶段新增原生 rclpy 前台监督器补齐。

- 复用现有发布者，不因断流重复启动替代进程；只有收到2秒内、非未来的有效图像才报告就绪。
- 未发现图像发布者时启动 RTSP/检测器；每个本次启动的进程使用独立进程组，退出时逐级回收。已有程序不停止。
- 支持 --no-view、--rtsp-url 和已有 ROS2 rtsp_url:= 参数形式；同用户同 DDS 域的重复入口会被锁拒绝。
- 无显示器时不启动窗口。有 DISPLAY 时使用 rqt_image_view，保留旧软件渲染环境。补装 Humble rqt_image_view 1.2.0，APT模拟及安装记录保留，0升级/0删除，新增23个包。

## 验证

ROS_DOMAIN_ID=86、ROS_LOCALHOST_ONLY=1、orangepi用户，真实飞控/云台不参与。16项联合检查通过：1920×1080合成图驱动实际原生检测器、复用一个/两个发布者、重复入口拒绝、Ctrl+C所属回收、过期/未来图像拒绝、localhost拒绝连接时回收RTSP。6项实际rqt offscreen检查通过：指定话题订阅、保持运行、仅拥有窗口、正常退出、进程组回收及外部发布者存活。

机载源码 c3dff85，测试/部署监督器SHA256 fb6f9a1e136c16889784fbfe036b976d272bed39e0302ceb0017f12defdcd6b5。未重启现有服务或切换正式地面站入口。机载Git干净、NoMachine active；临时目录取回证据后清理。runtime/提供原始日志，tools/提供测试代码。

## 问题与限制

首次以orangepi写入新文件因tools目录权限失败；随后以root完成精确文件写入，将两个文件所属者设为orangepi。未改变整个目录权限。

首次已部署shell入口 --help 检查返回139，后续直接检查以及相同Python subprocess捕获方式连续3次均返回0。内核日志未提供对应崩溃信息；原因尚未确定，保留为现场待查项，不能因重试成功宣称已根治。见wrapper-repeat.json。offscreen只验证窗口程序订阅/退出，未验收NoMachine可见窗口渲染。

没有真实云台RTSP重连、延迟或ArUco度量精度验收；内部标定及相机到机体外参仍需现场验证。下层aruco_vision.launch.py仍可直接启动两个节点；需要复用/前台管理时使用start_aruco_vision.sh。未启动任何降落控制器或真实飞控任务。

回退：正常退出该前台会话后，将机载c3dff85父提交中的03_tools/config script/start_aruco_vision.sh恢复即可；其调用的旧ROS2 launch保留。新增监督器不被其他入口调用，rqt包可保留，无需变更服务或底层驱动。回退未执行。
