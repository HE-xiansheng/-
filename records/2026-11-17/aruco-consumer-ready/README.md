# ArUco降落控制启动就绪修正

已为原生land_test增加真实MAVROS控制读者/内部路由出口就绪条件，并部署候选dae36a8。没有安装或切换正式服务，没有访问真实飞控执行任务或改参数。修复前严格失败记录完整保存在相邻aruco-mavros-wire目录。

## 行为

首次发送前等待/mavros/setpoint_raw的PositionTarget读者、只读GetParameters(thrust_scaling, DOUBLE)回复及/uas1/mavlink_sink发布端点。参数请求超过1秒则移除pending请求并重试；不阻塞30Hz控制循环。记录器的订阅不能代替真实消费者。就绪前不预发送、不进入模式READY、不接受任务enable；就绪后锁存，原有遥测时效/飞手取消/竞争控制器保护继续生效。新诊断列command_consumer_ready明确显示等待状态。

新增ROS参数command_consumer_node与command_transport_topic，默认如上，命名空间/uas_url变动时应一起配置；参数必须是绝对ROS名称。新增显式rcl_interfaces依赖。GetParameters只读，不更改MAVROS或PX4参数。

## 验证与失败

- 独立/tmp目录构建候选，四组CTest通过；没有覆盖正式install或运行进程进行试验。
- run1在MAVROS先启动条件下859条捕获/858条内部出口和PX4，严格39/42，缺少最后一条。初始缺口本轮未复现，严格仍失败。末条源戳1794848327.2466416与控制器/MAVROS同时SIGINT的1794848327.24687秒重合，支持测试退出顺序造成尾部丢失解释，不能作为通过豁免。
- run2记录器/健康合成输入及控制器先启动，真实MAVROS后启动；负向检查确认等待时没有输出或模式READY，实际插件/路由就绪后放行。872源/872内部出口/872wire，44项严格检查通过。
- run3明确先停控制器、读取其末尾回调/队列，再停止MAVROS，避免测试先关闭消费者；同样覆盖延迟MAVROS启动，871/871/871、44项严格通过。没有忽略头尾消息或改统计区间。
- 原22项降落回归及39项其他触发路径回归通过，覆盖服务/px4_mode触发、自动起飞与交还逻辑、拒绝重试/取消等，均使用假PX4、禁用云台指令。假消费者明确具有实际控制插件身份、参数服务和Mavlink出口发布端点，不在产品中增加测试绕过开关。假飞机模型只用于回归，不是PX4闭环飞行。

图像链使用实际aruco_test检测器、候选land_test、真实完整MAVROS及隔离官方PX4 v1.16.0。ROS域83只在localhost，电脑PX4网络空间仅lo，SSH stdio只连接仿真UDP端口。合成遥测输入和飞行服务都重映射到/fixture隔离桩，真实PX4始终未解锁、非OFFBOARD。最终加入实际状态原始记录，既检查ROS状态也检查ULog。

严格比较捕获ROS指令、内部出口及PX4线包的毫秒源戳和数量，检查ENU/NED坐标、PVA/yaw/yaw_rate、mask，ULog确认仅速度输入。实际图像→PnP→历史机体位姿配对、旋转、下降、断流保持、恢复及飞手取消均通过。ROS捕获不是产品publish函数注入，因此不声称证明发布者的每条内部调用均被观测。两次冷启动通过不保证任意负载/网络下永不丢包；新增参数请求超时/异常类型分支没有单独故障注入，运行中路由重启/持续交付也未验证。

## 部署和回退

回归与最终部署二进制SHA256均e38939c431c42e966e3897773bbf592d7e40137f6cb94fdbc8dd6ff3c266bab6。deployment-manifest.json记录原/新源码和安装产物摘要，deployment.txt包含机载提交与Git状态。部署前核对5b1d6b7干净、旧文件摘要且无降落进程，逐文件原子替换，异常恢复原内容；只提交四个运行源码文件，报告没有写入机载。第一次部署预检读取/proc/1/exe遇权限限制，在写任何文件前退出；改用只读ps进程名检查后成功，没有中断现有服务。

可在没有候选会话运行时撤销机载提交dae36a8，并将电脑/home/tyzk/drone-migration-work/land-consumer-before-deploy/Safina_land_test以0755权限复制回install/land_test/lib/land_test/Safina_land_test，并用撤销后的源码package.xml恢复install/land_test/share/land_test/package.xml；也可从旧源码重新构建安装；旧二进制SHA256为ca44b93128497bca0cdaae1b9755b830366a179081c3512d90c3992cd385003a。仅git revert不会替换被忽略的install二进制，应同时恢复二进制及安装package.xml。未切换正式服务，无服务回退操作。

## 复现及记录

ARUCO_WIRE_CANDIDATE=1 python3 tools/run_aruco_mavros_wire.py /absolute/new-output-directory，随后用tools/analyze_waypoint_mavros_wire.py --fixture-subdir aruco-mavros-wire分析。runner会编译/CTest/安装到独立/tmp目录；不设置candidate选项时检查已部署版本。fixture验证原控制器与含只读参数客户端的新版本，并单独限制飞行服务客户端为隔离桩。

输入源码、日志、原始归档、ULog和摘要保留，candidate-build/candidate-install、PX4 etc/test_data及缓存不重复展开。完整归档包含候选二进制，电脑原始工作目录仍保留。regression保存实际使用脚本、配置、CSV和结果；两种回归分开运行，归档中的run_land_regressions.py仅执行第一种，modes按LAND_EXECUTABLE/LAND_CONFIG/LAND_MODES_OUTPUT变量独立运行。记录只存电脑/GitHub，机载临时资料取回后删除。

真实雷达/云台直连、外参、真实PX4外部视觉融合与完整地面站/任务链仍待验证，本阶段不能替代它们。

最终run3为871/871、44项严格检查全部通过；run2为872/872，同样44项全部通过。最终二进制e38939c431c42e966e3897773bbf592d7e40137f6cb94fdbc8dd6ff3c266bab6与61项回归使用二进制逐字一致；4组CTest通过。cleanup.json确认机载dae36a8干净、NoMachine active、所属临时目录/测试进程清理。门控1秒超时分支未注入“服务已接收请求但不回复”的专项故障；不能以本轮通过声称所有网络/重启故障均已覆盖。
