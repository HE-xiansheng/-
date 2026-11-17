# Livox SDK2与ROS2 spdlog符号冲突的候选修正

2026-11-17，在已固定的.224雷达网口/.158实物雷达上执行。产品当前仍为7a3c0cb，原安装SDK库未换入候选；本阶段只用/tmp候选和子进程LD_LIBRARY_PATH，测试结束目录/进程按清理记录核对。

## 故障与对照

原安装SDK2 API1.3.0自带header-only spdlog1.3.1；Ubuntu22.04/ROS2实际运行依赖系统spdlog1.9.2。崩溃栈为SDK std::shared_ptr<spdlog::logger>析构进入系统libspdlog.so.1的符号，发生于SDK Deinit和Lddc析构打印后，不能仅按上一条打印认定是队列释放故障。

临时SIGSEGV调用栈捕获库只用于三轮定位，不安装到系统。直接启动/单次SIGINT一轮以及ROS launch/进程组SIGINT两轮全部复现-11，排除仅由重复信号导致的解释；launch自身返回0仍会包含子节点-11。

使用已归档旧机SDK源码（220个文件，API版本1.3.0），仅给Linux共享库链接添加-Wl,-Bsymbolic，使SDK内部调用优先使用自身的日志实现；不修改SDK通信算法/头文件或系统spdlog。候选三轮真实采流均正常退出0，/proc映射确认实际SDK候选路径。随后用同一批编译对象仅去掉此链接选项重新链接control库，单次SIGINT再次-11且栈相同。55个公开C导出名称与原安装库一致；此对照支持修正针对SDK/ROS2同名日志符号混用。

## 实物联合验收范围

run7在原有持久.51/32→.158/32网络上，仅驱动子进程选择候选SDK，不使用SIGSEGV调试preload。先确认两路原始数据新鲜，再执行60秒真实驱动→FAST-LIO→Pose转发→被动EGO：40/40通过，包括实际候选映射、子节点正常退出、所有所属进程清理和证据取回。FAST-LIO仍time_sync_en=true、offset0。无目标点、无真实MAVROS/FCU或控制接口。

| 数据流 | 稳定频率Hz | 全轮最大源年龄s |
| --- | ---: | ---: |
| lidar | 10.000 | 0.303 |
| imu | 200.023 | 0.199 |
| odom | 9.999 | 0.091 |
| cloud | 10.006 | 0.117 |
| vision | 10.001 | 0.113 |

此结论仅覆盖以上实际输入/运行时间/退出序列，不证明运动精度、外参、长时热稳定性、真实PX4融合、人工地面站全流程或全平台迁移完成。

## 交付与待执行操作

candidates/sdk2中提供单文件链接补丁、原库/候选/control三份ARM64库归档、二进制SHA、公开导出对照和安装/恢复步骤。所有库的SDK API实测均1.3.0，ROS雷达驱动版本1.2.6不变。原库SHA480fd437...、候选SHA292c8fdd...；完整值见binary-manifest.json。

本目录保存完整源、220项源码SHA、编译日志、CMake/对象/ELF归档、依赖和SDK版本实读、三类对照及run7原始证据；构建归档中的全部源码/候选/control库已与电脑原件逐个验证。候选尚未替换正式私有目录中的SDK共享库，需按用户原有更换驱动/运行环境前确认的边界，说明单库变更后再应用。

应用对象仅/home/orangepi/drone_platform/02_drivers/sensors/sdk2/lib/liblivox_lidar_sdk_shared.so，不改系统spdlog、内核/BSP、飞控参数或正式服务。应用后必须用默认已安装路径验证；现有原库已在电脑及GitHub候选归档中保存，可按SHA恢复，不在机载长期保留迁移记录。
