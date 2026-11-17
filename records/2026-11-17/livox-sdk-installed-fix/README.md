# .224 SDK日志兼容修复正式安装与验收

2026-11-17用户明确授权“那你先修复一下吧”后，已将同版Livox SDK2 1.3.0的日志符号隔离修复安装到.224。真实雷达→FAST-LIO→位姿转发→被动EGO在默认安装路径运行60秒，40/40检查通过，全部节点正常退出。

## 变更与版本

- 仅原子替换 `/home/orangepi/drone_platform/02_drivers/sensors/sdk2/lib/liblivox_lidar_sdk_shared.so`，权限0644、属主/属组orangepi，暂存文件已删除。
- 原SHA256：`480fd4377886adca14c95256d9ff0a7a1a0817b15f62eba93c965c8b6a68a6d7`；安装后：`292c8fdd45ad82feb960ee3c6aa9708f74e2e4362dedcc5973178616a3b437b3`。
- SDK运行时API为1.3.0，驱动启动日志为1.2.6。ROS2模板package.xml源文件与安装副本均写1.0.0，此元数据与驱动实际版本分别记录，未更改版本标记。
- 产品源码保持 `7a3c0cb71864d02a5764a22d8e004debb615232e` 干净；SDK二进制目录原本被Git忽略，实际部署依赖由上述摘要和归档锁定。
- 修复源、Linux `-Wl,-Bsymbolic` 补丁、同版API/55项公开C导出和对照证据见 [SDK包](../../../candidates/sdk2/README.md) 与 [候选阶段](../livox-sdk-symbol-fix/)。原安装库及已打补丁的完整源均在电脑/GitHub，可重建。

## 正式路径实物验收

运行命令：

```bash
python3 tools/run_livox_native_check.py \
  /home/tyzk/drone-migration-work/livox-sdk-installed-run1 \
  192.168.1.158 --existing-network --passive-planner --ready-start
```

使用.224永久雷达网络与产品JSON，不带 `--candidate-sdk`，无调试 `LD_PRELOAD`。实际进程映射指向上述正式安装文件，SHA256与候选完全一致。

| 数据流 | 稳定阶段频率 | 最大源时间戳年龄 |
| --- | ---: | ---: |
| Livox点云 | 9.999 Hz | 298 ms |
| Livox IMU | 199.995 Hz | 198 ms |
| FAST-LIO里程计 | 9.993 Hz | 82 ms |
| 位姿转发 | 10.008 Hz | 115 ms |

40项检查包括持续频率/间断、递增源时间戳、有限数据、CDR与ROS反序列化一致性、坐标系/四元数、转发保留原里程计时间戳、实际非空膨胀栅格、WAIT_GOAL及无目标/指令接口、子节点退出和默认SDK映射。驱动、LIO、转发及规划退出码均0，launch内部无子节点崩溃。

本轮限定为地面静态采集和被动建图。未接入真实MAVROS/飞控、未发送目标点、未修改PX4参数或启用正式服务，未写入.235。长时间稳定性、移动精度/外参、真实飞控融合和完整平台验收仍待完成。

## 记录、失败与清理

- `apply_sdk_fix.py` 为本次一次性应用脚本；固定SSH主机公钥、原库摘要、产品版本和暂存文件独占检查，fsync后原子替换。已应用时会拒绝再次执行。
- 首次应用在进程预检阶段拒绝，无文件写入：同UID的sd-pam/认证进程限制读取maps。`sudo -n`需要密码，未修改认证或权限；后续核对并仅允许明确的系统会话进程排除，任何受限应用或传感器进程仍拒绝，排除身份已记入application.json。
- 最后只读核对曾错误地把package.xml的1.0.0当成应为1.2.6，保留失败记录；改为分别保存包元数据和实际驱动启动版本，物理验收40/40不受影响，未重跑或放宽检查。
- `installed-run1/evidence.tar.gz` 包含完整机载原始日志、逐帧记录和测试输入；选出的结果、参数、进程日志同时展开便于检查。完整展开副本位于电脑 `/home/tyzk/drone-migration-work/livox-sdk-installed-run1`。
- `final-state.json` 独立读回确认正式库摘要、源码干净、永久雷达网络及Wi-Fi默认路由保持、NoMachine active；无SDK映射及所属测试进程、四个/tmp测试目录与暂存库均不存在。机载无本次报告或副本。

如需撤销本次修复，使用 [SDK包](../../../candidates/sdk2/README.md) 中arm64-libraries.tar.gz的original库，先核对其原SHA和当前正式库SHA、确认相关测试节点已停止，再按本次同目录暂存/复核/原子替换方法恢复原文件。原库在当前ROS2环境的退出故障已有实证，恢复后应据此重新评估验收结果。
