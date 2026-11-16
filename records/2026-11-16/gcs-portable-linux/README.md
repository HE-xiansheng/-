# Linux ROS2地面站独立包

本阶段补齐完整桌面运行包，而非仅tools组件。新增package_ros2_linux.py调用现有CMake完整install，带主程序、radarsimulator、Qt动态库/QML/插件、12个ROS2运行工具，以及只用于子进程的RTSP代理绕过包装器。旧打包器、Windows/Android规则及原电脑地面站入口未改。

产物在电脑下载目录：
`/home/tyzk/下载/科研飞行平台-ROS2地面站/drone-gcs-ros2-20261116-22c9b52-r2-linux-amd64.tar.gz`

SHA256：`0ed30c3017f3065ce6794469f3dd7461233544ff797e7de9fa8d2c41dcb60aea`

对应解压目录包含start-groundstation.sh，从任意cwd启动。没有安装或覆盖桌面快捷方式；删除关闭后的解压目录即可移除此包。原始失败首版收到下载目录下.failed-builds，不作为可用交付。归档约43MB，完整清单见payload-manifest.json；大二进制留电脑，代码/摘要/证据推送GitHub。

## 验证范围

实际归档解压到含空格的新路径portable relocation test3，使用独立XDG配置/数据/缓存及GCS_STATE_DIR，cwd=/tmp。

result.json的7项通过：全部文件摘要/权限/符号链接、12工具清单、排除ROS1安装器、所有ELF的ldd依赖和Qt/ICU重定位、完整主界面首帧、进程maps中Qt从新包加载、无QML组件加载失败。实际DISPLAY=:1，成功出现主界面首帧日志；测试用SIGTERM结束，exit=-15属主动终止，不是正常点关闭退出的证据。未把视觉检查或物理工作流列为已完成。

installed-tools/result.json的8项通过：新解压目录工具导入/help、权限/清单、真实SSH到.224接收PFS1、点云桥运行和两个管理器停止。使用先前核实的专用known_hosts，未关闭主机校验。当前传感器服务未启用，PFS1如实显示雷达等缺话题，不能把收到状态包当作传感器全就绪。无arm、任务启动或物理控制。

## 实际发现及处理

1. 首版测试封装把ldd含空格路径按空白切开，返回1；finally仅根据已记录项生成了误导的passed=true。原始失败保留于相邻first-harness，解释文件明确不能视为通过。验证器修正路径解析，并要求完整流程completed。
2. 第二次真正发现Qt X11依赖libxcb-cursor.so.0缺失，失败记录在相邻missing-xcb。通过显式extra-runtime收录原电脑已解包的Ubuntu libxcb-cursor0运行库及版权文件，不依赖原工程运行路径；r2复测通过。Qt/ICU许可一并保留。
3. 重复打包同名版本被拒绝，输出日志保留；不会清空或覆盖既有产物。

## 尚未完成

只验证当前Ubuntu20.04 amd64桌面，其他发行版需要实际测试；系统OpenGL/X11、numpy、OpenSSH、LZ4/bzip2/SQLite及GStreamer仍为目标系统依赖。本机无需ROS。

无机载运行源码变更或服务切换。本阶段不在机载保留报告；真实雷达/PX4外部视觉融合、云台、独立USB以及任务/地面站物理整链仍待验证。整平台目标保持进行中。
