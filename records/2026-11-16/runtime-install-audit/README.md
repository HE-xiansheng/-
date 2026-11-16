# Linux地面站运行入口与安装清单核对

实际Qt控制器调用机载原生ROS2任务/定位/规划/录制程序；groundstation/tools/planner/automatic-navigation及offboard-retarget为ROS1历史参考，当前Qt不调用其安装器。groundstation/tools/missions/task_manager.py仍为旧ROS1文件，已移除未使用的Qt嵌入资源，并修正当前文档避免误部署。

发现Linux CMake install(DIRECTORY tools/)会附带ROS1安装器与旧测试；改为cmake/ros2_runtime_tools.cmake明确的12文件运行清单，包含点云/遥测/SSH依赖和可执行askpass。Linux条件之外的Windows/Android安装规则未改。本轮没有执行旧安装器或改变机载服务。

主程序/模拟器重新构建通过（build.log）。在电脑独立stage执行CMake ros2_tools组件安装（install.log），12个文件与源码SHA256一致（installed-manifest.json）。真实安装目录中8项检查通过（installed-final/result.json）：文件范围、排除历史工具、askpass权限、三个入口的依赖导入与help、实际SSH到.224接收PFS1、桥接运行/正常停止。

首次installed-test读取遥测失败，因为测试直接启动后台工具、没有完成正常GUI第一步的SSH指纹登记，StrictHostKeyChecking=yes正确拒绝。使用仅测试PATH中的ssh包装器，指定此前核实的专用known_hosts及HostKeyAlias后复测通过；没有关闭主机指纹验证，没有修改全局SSH配置。首次失败保留。运行使用独立GCS_STATE_DIR和19100/19101/19103/19115端口，原地面站状态和端口不受影响。

PFS1 connected=true仅表示实时采样/传输正常，不表示传感器正常；本次话题缺失被如实显示为缺失/过期。只读检查确认当前.224没有Safina_sensor.service，也无相机/MAVROS/FAST-LIO进程（onboard-current-runtime.txt）。本轮没有启动传感器或飞行任务。

地面站部署说明见groundstation/doc/ros2_deployment.md，根README已依据当前证据重写，区分短时历史验证与当前运行状态。旧package_linux.py使用不同目录布局，尚未作为本ROS2副本正式交付包；本轮只完成CMake工具安装和构建验证，没有验证可搬移Qt完整包、Windows/Android运行或可见3D桌面。

所有测试日志在电脑/GitHub；测试桥接已停止，未在机载安装报告或变更运行代码。可将独立stage目录删除以回收本轮安装；不存在正式切换回退动作。
