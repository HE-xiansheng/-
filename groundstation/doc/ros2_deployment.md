# Ubuntu22.04/Humble独立地面站

本说明针对本仓库groundstation迁移副本和机载192.168.1.224。原地面站、.235 Noetic参考工程不自动替换。独立主程序已构建；独立Linux包已在当前电脑验证重定位和主界面首帧；物理任务仍需验证。

## 实际运行入口

| 功能 | 当前ROS2入口 |
| --- | --- |
| 点云、位姿与状态 | 本机tools/communication/manage_link.py，调用pointcloud-only/manage.py与preflight/bridge.py；原生ROS2采样代码通过SSH运行 |
| 定位启停 | Qt资源tools/localization/manage_stack.py调用机载01_apps/fast_lio/manage_stack.py |
| 避障与地图目标 | 机载01_apps/ego_planner/run_board.sh、map_navigation.py、run_px4_execution.sh |
| 航点、ArUco、人物任务 | 机载01_apps/mission_demo/tools/gcs_task_manager.py |
| 图像视频 | Qt资源tools/video/ros_image_bridge.py；云台原图仍使用既有RTSP路径 |
| 录制与文件读取 | 机载01_apps/ego_planner/recording_manage.py与Qt资源tools/flight-records/remote_records.py |

机载命令统一先source `~/drone_platform/env.sh`。点云PCD1/LZ4、POS1、PFS1和视频VID1继续使用已有协议；任务JSON保持version4及定位会话身份。默认本机原始遥测UDP端口9015，运行状态目录为用户数据目录drone-gcs-ros2，可用GCS_STATE_DIR指定独立位置。

`tools/missions/task_manager.py`、`tools/planner/automatic-navigation/`、`tools/planner/offboard-retarget/`以及Noetic/roslaunch资料属于历史ROS1参考。当前Qt任务入口不使用该task_manager资源（Linux安装组件也不打包它），地图选点也不运行automatic-navigation安装器。ROS2任务管理器源码见仓库platform/01_apps/mission_demo/tools/gcs_task_manager.py。不要按旧说明把ROS1管理器覆盖到.224。

## Linux独立安装

主程序、Qt插件部署规则保持原有构建方式。Linux运行工具由cmake/ros2_runtime_tools.cmake显式列出，保持bin/../tools相对布局。该组件只安装11个当前运行所需Python文件及一个具有执行权限的gcs-askpass；历史ROS1安装器、开发测试和备份源码仍保存在仓库，不随Linux运行工具安装。

可以使用独立前缀先查看工具组件，不切换现有地面站：

```bash
cmake --install /absolute/build-directory --prefix /absolute/staging-directory --component ros2_tools
```

此命令仅安装工具组件，不能单独作为包含主程序和Qt依赖的完整交付包。电脑端Python需要numpy，LZ4动态库用于既有点云压缩。本机无需安装ROS2，机载需有Humble和对应源码构建产物。Windows/Android安装条件和现有交付路径不因本次Linux工具清单调整而改变；未在本次声明Windows/Android运行验收。

## 已有验证及限制

原生任务、规划、定位、录制、视频、真实相机/NPU和SSH/Qt接口验证见仓库records/2026-11-16各阶段记录。实际FAST-LIO/转发/MAVROS到官方PX4仿真已完成回放数据融合及停流验证；这不等同新雷达采集、真实PX4融合或实飞。

正式入口切换、传感器后台服务变更和真实飞控估计器配置仍需用户确认。实际任务准备会保留遥控触发语义，不应用于只读连通测试；只读检查可使用status/watch，地面阶段不解锁或执行飞行任务。

旧tools/packaging/package_linux.py及install_user.py面向原布局，保持历史参考。当前独立包由tools/packaging/package_ros2_linux.py生成，直接使用CMake完整安装规则；不覆盖电脑现有快捷方式。

```bash
python3 tools/packaging/package_ros2_linux.py \
  --build-dir /absolute/build-directory --cmake /absolute/cmake \
  --output /absolute/package-output --version VERSION \
  --extra-runtime /absolute/unpacked-ubuntu-runtime
```

补充运行库参数用于本机缺少的libxcb-cursor及许可文件。解压归档后执行start-groundstation.sh，可从任意工作目录启动。包含Qt库/QML/插件，系统Python/numpy、OpenSSH、OpenGL/X11和GStreamer仍为外部依赖；本机不需要ROS。此轮Ubuntu20.04 amd64桌面已验证，其他发行版未验收。文件摘要、权限和符号链接记录在payload-manifest.json；外层归档有sha256文件。

下载目录的20261116-22c9b52-r2产物在含空格新路径解压后完成7项清单/依赖/主界面首帧检查、8项实际SSH/PFS1及桥接启停检查；测试没有发送飞行任务，完整物理工作流仍待验收。见records/2026-11-16/gcs-portable-linux。仅删除已关闭的解压目录即可移除此包，用户日志与设置继续留在各自用户数据目录。

实际VideoPanel云台按钮与视频渲染另完成14项实物视频检查：开关/重开、640x360连续变化画面、QML Image Ready、offscreen窗口帧、主动中断临时转发后的失效和恢复自动重连均通过。视频来自.235经.224临时relay与SSH转发，未启动飞行或云台控制；不替代.224云台网卡直连、可见桌面鼠标操作、延迟或外参验收。MPP订阅吞吐问题仍保留。日志见records/2026-11-16/gimbal-qt-live。


## Linux 视频显示调度候选（2026-11-17）

当前 X11 / NVIDIA OpenGL 环境中，完整窗口的垂直同步等待会拖慢云台 QProcess 原始视频管道。相同测试二进制本地合成 30Hz 输入约25秒仅显示78–91帧；设置 `QSG_NO_VSYNC=1` 后显示740帧。独立 QCoreApplication 管道约10秒读取300帧。`LIBGL_ALWAYS_SOFTWARE=1` 未使本机 NVIDIA GLVND 切换为软件渲染，不能把这些结果描述为软件渲染验收。

Linux 桌面程序在创建 QGuiApplication 前，仅在未指定环境变量时默认设置 `QSG_NO_VSYNC=1`；显式环境设置原样交给 Qt。实际归档测试中设为0仍出现swapInterval0，因此不能用0作为恢复垂直同步的操作方法。Windows / Android 不进入此分支。依据 Qt 6.4+ 官方场景图说明，此设置用系统定时器驱动显示；可能改变画面同步、撕裂与功耗表现，其他显示驱动尚未验收。回退使用原r2独立包或撤销main.cpp这一修改后重建。

r3独立包位于电脑下载目录 `科研飞行平台-ROS2地面站/drone-gcs-ros2-20261117-ros2-r3-linux-amd64.tar.gz`，SHA256为 `319a5331b378b22a12d966f8b1fa49efb87ca0f01c8be7daa36c949e15c34abb`。真实归档中文/空格路径重定位、完整依赖与生产主界面首帧7项通过，实际日志为NVIDIA、swapInterval0；旧桌面入口未切换。

完整实际主窗口、SSH/UDP、真实相机/NPU、雷达回放定位、合成云台及隔离录制的run8通过24项GUI和43项机载检查，包含实际QML停止录制并正常收尾。该轮通过显式环境变量启用修正。run9改由产品默认设置启用，云台逐秒正常，但03:30:56相机USB断开、03:30:58重枚举，双目新鲜度失败，故整轮失败。保留这一失败及不完整UDP点云分片日志；该轮不能作为默认路径通过证据，也不能把USB故障归因于显示修正。随后run10产品默认路径严格24项GUI及43项机载检查通过；USB采流期间active、无断开，停止后suspended。一次复测通过不等于间歇USB问题解决。详见仓库 `records/2026-11-17/gcs-render-pipe`。物理雷达/云台直连、外参、真实PX4融合与飞行任务仍未验收。
