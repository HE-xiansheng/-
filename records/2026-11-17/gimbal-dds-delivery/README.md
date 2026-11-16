# MPP大图发布的Fast DDS共享内存配置

历史实物云台流出现已发布图像在独立订阅端漏收；仅增大订阅队列无效。机载Ubuntu22.04/Humble实际使用ros-humble-fastrtps2.6.12、rmw_fastrtps_cpp6.2.10。按官方2.6.12文档及源码，默认SHM段为512KiB，段大小接近/小于消息有丢失风险；1920x1080 mono8单帧2073600字节。官方来源见official/sources-2.6.12.json与所存文档/代码。网页访问403后使用官方GitHub原文件读取。

## 改动

rtsp_hw_pub_ultra在rclcpp初始化前，若用户未指定FASTRTPS_DEFAULT_PROFILES_FILE或FASTDDS_DEFAULT_PROFILES_FILE，且RMW未指定为其他实现，则加载本包share/rtsp_test/config/fastdds_large_images.xml。仅影响该发布进程，SHM为64MiB，保留UDPv4网络传输；不把全平台限制到localhost。显式配置优先，启动日志显示选择。CMake增加ament_index_cpp依赖并安装配置文件。现有ros2 run、launch和GCS任务入口均调用此程序，无须修改调用方式。队列深度、主题、格式、源戳、颜色限频及传感器服务保持既有逻辑。

每个MPP进程增加约64MiB的SHM段（含管理开销实际文件67133984字节），不全局放大所有ROS节点。32MiB显式配置也通过，本次选64MiB留出余量；未宣称任何负载下都无丢失。

## 对照与实际候选验证

所有测试均在机载独立DDS domain80、ROS_LOCALHOST_ONLY=1中执行，只发布图像、不连接飞控或发送云台运动命令。原生单线程观察器同时记录reliable与BEST_EFFORT各两个订阅，保存源戳、收件时刻、几何信息和稀疏像素摘要；逐条与真实发布DEBUG日志的源戳集合核对，严格要求每一条发布被接收且无重复/多余。源戳为解码收件时间，不代表相机曝光时间或端到端延迟。

| 运行 | 配置 | 严格检查 | 灰度 Reliable/发布 | 彩色 Reliable/发布 | 灰度 BE/发布 | 彩色 BE/发布 |
|---|---|---|---|---|---|---|
| run2 | default | 17/24 | 643/726 | 348/363 | 154/726 | 27/363 |
| run2 | shm64 | 22/24 | 715/715 | 357/358 | 715/715 | 357/358 |
| run3 | shm64 | 24/24 | 727/727 | 364/364 | 727/727 | 364/364 |
| run3 | default | 17/24 | 655/723 | 357/361 | 223/723 | 72/361 |
| run3 | publisher_shm64 | 24/24 | 714/714 | 357/357 | 714/714 | 357/357 |
| candidate-run1 | candidate_default | 26/26 | 726/726 | 363/363 | 726/726 | 363/363 |
| candidate-run1 | candidate_explicit32 | 26/26 | 716/716 | 358/358 | 716/716 | 358/358 |
| live-run1 | candidate_default | 26/26 | 691/691 | 344/344 | 691/691 | 344/344 |
| live-run1 | candidate_explicit32 | 26/26 | 682/682 | 340/340 | 682/682 | 340/340 |

run1观察器退出时-11，末行CSV截断，未完成，不计通过。根因尚未单独确定；后续改为停止标记退出，移除订阅后shutdown，未复现。run2修正观察器后，64MiB灰度全部到达，但两种QoS的最后一条彩色缺失；实际日志显示此条在SIGINT后仍发布，保留严格22/24失败。之后测试先停止本次所属RTSP源/relay，让解码及DDS队列处理完，再停止解码器，最后让观察器退出；没有删掉首尾帧或豁免缺失。

run3默认配置仍失败，两端64MiB及仅发布端64MiB均24/24通过。publisher_shm64的观察端环境无profile，证明可以只修发布进程。run3编译结束前修正了该组观察器环境设置；原始输入、修正输入、执行前记录及机载实际执行副本均保留，分析以实际副本为准。

candidate-run1是实际修改后的程序：自动64MiB及显式32MiB各26/26通过。live-run1经旧机已有云台流→临时SSH loopback转发→新机实际relay/MPP，自动64MiB及显式32MiB各26/26通过，灰度约30Hz、彩色约15Hz。没有存实物图像；稀疏摘要变化检查通过。进程映射证明实际使用SHM。两次候选构建中的限频门控CTest均通过，实际源码与输入归档逐字核对，运行二进制均f391437954c37c5b2b253c637db141421fc5479b808d5d7d2915a33810d85cc1。

## 部署与记录

机载提交1f81c3b，部署4个源码/配置文件以及installed二进制、package.xml和XML；摘要见deployment-manifest.json，部署脚本先验证旧HEAD73dd98c、工作区干净、无MPP运行及旧文件摘要。未切换或启用正式service。安装后以闭合loopback端口确认正式安装路径可加载该profile、无XML解析错误（预期RTSP连接失败退出255），不访问实际设备。

机载临时资料先取回电脑，再删除；正常退出、Git干净、NoMachine active见post-deploy-state.txt。旧机原MediaMTX进程仍PID1419，起始时间不变，未修改旧机服务/配置。所有归档结果已与原始tar内文件逐字核对；summary.json汇总结果，展开目录不重复保存构建/安装/合成视频，完整原始tar仍含这些文件。迁移记录仅在电脑/GitHub。

撤销方法：还原机载1f81c3b这4个源文件修改（新增XML随之移除），恢复电脑/home/tyzk/drone-migration-work/gimbal-delivery-before-deploy/rtsp_hw_pub_ultra（二进制SHA004ab1fa39f01bc9b9072c7fae540e3fdec0467c556987297cf7ad8c6e33a630，0755）及同目录package.xml；移除这次新增的installed share/rtsp_test/config/fastdds_large_images.xml。也可按原提交重新构建候选包。没有systemd切换需要撤销。

## 未验收边界

真实流仍经.235只读转发，不是新机物理云台网卡直连验证。本阶段未重测完整Qt界面，不把原生订阅器当作Qt显示验收；Qt实物视频先前独立记录见2026-11-16/gimbal-qt-live。新机雷达网口仍无carrier。长时间/多负载录制吞吐、跨机DDS原始图像、物理标定/融合与完整任务，以及源断流后MPP自身的恢复行为仍需进一步验证。整个迁移目标未完成。
