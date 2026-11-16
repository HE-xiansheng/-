# 精准降落补充触发路径

使用 .224 原生 land_test、域90/localhost 和伪 PX4 接口，39/39 检查通过，控制器均正常退出。覆盖 service 空中激活、service 自动起飞、手动 OFFBOARD 边沿、模式拒绝重试、服务缺失、飞手 POSCTL 取消及停用。所有起飞/解锁/模式请求仅发生在模拟接口；未连接真实飞控控制。

`result.json` 为逐项结果，`runtime/` 保存控制器日志及模拟任务 CSV，`first.log` 为运行记录。源码与运行库摘要见相邻 camera-npu/manifest.json，机载代码在测试时为 b779e23，后续仅相机配置改为 dcbb549。

真实图像到控制器、SITL、物理降落和机体外参尚未验收。该阶段未修改降落控制代码，无需代码回退。临时测试进程已退出，机载 /tmp/land-modes-ros2 已清理，记录仅留电脑/GitHub。
