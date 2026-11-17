# SDK2 1.3.0日志符号隔离修复（.224已安装）

用于.224 Ubuntu22.04/Humble中真实Livox驱动退出SIGSEGV。只修改同版本SDK共享库的Linux链接选项：-Wl,-Bsymbolic；算法、雷达驱动源码、SDK通信逻辑、接口头和系统spdlog均不改。补丁isolate-spdlog-symbols.patch，完整源/编译证据见records/2026-11-17/livox-sdk-symbol-fix/build。

候选、原安装库、同编译对象去掉该选项的对照库在arm64-libraries.tar.gz。全部为AArch64 ELF，SDK API均报告1.3.0，55个公开C导出名字相同。binary-manifest.json记录实测SHA、大小和电脑原件路径；只可选择candidate目录中的库安装，control用于复现故障。

## 验证范围

原安装库三次真实采流退出均-11；候选三次退出均0，/proc映射确认实际使用候选；同对象仅去掉Bsymbolic的对照再次-11。候选随后不用调试preload完成真实驱动→LIO→转发→被动EGO 60秒40/40检查，包括内部子进程正常退出。无目标点、无真实MAVROS/FCU、无飞行任务。此证据不替代长时稳定性、移动精度/外参、真实飞控融合或全平台验收。

## 正式安装状态

2026-11-17用户明确授权“那你先修复一下吧”后，已替换/home/orangepi/drone_platform/02_drivers/sensors/sdk2/lib/liblivox_lidar_sdk_shared.so。原SHA480fd4377886adca14c95256d9ff0a7a1a0817b15f62eba93c965c8b6a68a6d7，现SHA292c8fdd45ad82feb960ee3c6aa9708f74e2e4362dedcc5973178616a3b437b3，运行时SDK API仍为1.3.0、权限0644、属主orangepi。具体应用及正式路径验收见[安装记录](../../records/2026-11-17/livox-sdk-installed-fix/README.md)。此依赖目录不受产品Git跟踪，产品源码仍7a3c0cb干净；安装二进制由上述SHA和归档锁定。不修改系统/内核、/opt/ros或系统spdlog，不启用正式服务。

应用脚本已核对SSH主机身份、原SHA及进程占用，再用同目录暂存文件、SHA复核和fsync后原子替换；正式路径验证不通过LD_LIBRARY_PATH临时选择。若确需恢复，停止使用该库的自有测试进程后，按同样方式使用归档original中的原库替换回来。原库会恢复已知的ROS2退出故障；无需在机载保存报告或副本，配置与原库均已保存电脑/GitHub。

## 后续重建

records/2026-11-17/livox-sdk-symbol-fix/build/source.tar.gz中的source目录已包含本补丁，构建证据和公开C导出清单位于同目录。若使用原始install_assets/Livox_SDK2源码重新构建，须先应用isolate-spdlog-symbols.patch；只重建未打补丁的旧源码会再次产生原有日志符号冲突。重新构建后的二进制摘要可能变化，需重新完成同版API/导出及原生运行和退出验收。
