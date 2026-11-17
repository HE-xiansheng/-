# 当前地面站管理接口修复

截图中“需要api_version=1”的原因是运行中的早期Qt二进制仍内嵌空modules配置和占位管理器。修改外部profile不会更新已经嵌入、加载的资源。canonical源码已有原生ROS2定位/规划/任务接口，故从canonical重新Release构建并生成20261117-ros2-r4包，直接委托机载管理器。

r4窗口PID69447，独立使用点云9100/位姿9101/状态9103/原始遥测19115和独立设置目录，ROS域0；原桌面快捷方式未替换。17:14用户实际连接并点击启动定位后，管理器返回成功，点云/位姿和ROS定位状态到达。随后原生复用检查3/3通过，LIO/转发PID、启动tick、boot和DDS GID均保持，不重建定位原点。package-verification.json记录包清单、采样器和内嵌原生helper检查。

只读任务watch的Ctrl+C退出复现两个错误：Humble先关闭context使后台Executor抛ExternalShutdownException；finally再次shutdown导致already called、退出1。修复为显式处理正常外部context关闭、幂等close和try_shutdown，不改变任务状态机或飞控命令。

候选和安装版本各12/12生命周期检查通过，覆盖正常关闭、外部提前关闭、重复清理和实际只读watch/SIGINT/helper回收。旧版同工具失败保留在shutdown-original，最早真实失败也保留于native-mission-watch.jsonl及stderr。机载已提交7dd15a5be69148d4b124583f843d4bdfbe49fe46，其他现场文件未改。

首次原子部署因父目录root所有而拒绝，目标未改；随后使用用户此前已提供并授权的sudo认证，仅安装固定旧/新SHA单文件并保留所有者/权限。没有停止基础服务或用户定位。before/gcs_task_manager.py为原文件；如需撤销，在没有管理请求时恢复该文件即可，不影响已运行的定位。所有修复与检查均不解锁、不起飞、不准备任务。

电脑入口为`bash /home/tyzk/drone-migration-github/tools/launch_ros2_repaired_gcs.sh`，默认指向下载目录的r4包，GCS_BUNDLE_DIR可指定同布局包。当前窗口已经运行，不需重复打开。

接口可用与定位时效分别验收：17:26当前旧LIO仍误校+1.171秒导致后续转发过期，源时间修复候选见../fastlio-source-clock；该候选尚未应用到用户正在编辑航点的会话。规划高负载、云台灰度交付和运动外参等未借此标为完成。
