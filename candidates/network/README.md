# .224传感器网络配置

2026-11-17用户明确授权“网口像.235一样固定吧”。Livox网络已持久应用；云台配置仍为未安装候选。

## Livox已应用

- 机载Wi-Fi管理地址192.168.1.224/23与默认路由192.168.1.1保持原样，Wi-Fi当前仍由DHCP分配。
- 板载网口enP3p49s0 / C0:74:2B:F9:C0:D8：固定192.168.1.51/32，仅添加192.168.1.158/32精确路由，不覆盖管理网段。
- 实际新雷达ARMCP6S0039858，192.168.1.158；SDK2只读发现20/20响应。旧机.235上的雷达是另一台ARMCP1V0038817/.117；旧机网口.50不改。
- NetworkManager持久连接ROS2 Livox，UUID4fb850bb-6fb0-4380-aacc-7a641a3dc761，autoconnect=true、priority100、绑定板载MAC、never-default=true、无网关/DNS、IPv6 disabled。
- 配置已写入root:root/0600的/etc/NetworkManager/system-connections/ROS2 Livox.nmconnection；libnm unsaved=false，已激活，ping3/3，默认路由仍经wlan0。用户11月17日14:28左右重新上电后，该地址/路由和SDK修复保持；未将此读回替代长时稳定性检查。
- 机载02_drivers/sensors/config/MID360s_config.json已更新目标.158（host.51、端口56101/56201/56301/56401/56501），网络JSON产品版本d9caef8，随后增加定位启动就绪等待后的产品版本7a3c0cb。本目录JSON及nmconnection作为已应用配置的电脑存档。

历史.117候选/临时测试/失败证据见records/2026-11-17/livox-native-validation；持久应用和后续联合验证见records/2026-11-17/livox-persistent-network。未启用正式传感器服务、真实MAVROS或飞行任务。

## 云台候选尚未应用

ros2-gimbal.nmconnection现已按新机实际USB网卡enx00e09a5af420 / MAC00:E0:9A:5A:F4:20重建，绑定192.168.144.100/24，无网关/DNS/默认路由，目标云台192.168.144.108:554。旧候选绑定的是.235另一块网卡00:E0:9A:5B:57:C2，不应在新机直接使用。候选autoconnect=false，尚未写入系统持久目录或激活为正式网络。

使用save-no、autoconnect=false的临时连接已验证实际网卡/云台H264 1920×1080@30Hz，以及ROS2 MPP解码、QoS全帧交付；每轮结束删除临时连接并恢复原连接。管理默认路由仍经wlan0，雷达仍为.51→.158。具体结果与失败见records/2026-11-17/new-physical-cameras。正式网络/服务启用前须按用户原约束说明影响并确认；本目录不隐式触发安装。

## 恢复方法

如确需恢复原网络：先停止本次所属定位进程，再停用/删除ROS2 Livox连接，启用原DHCP连接UUID832dd3b5-8a1a-3c4b-93db-c09f90bdd28a；从电脑records/2026-11-17/livox-persistent-network/config-before.json恢复原雷达JSON。原DHCP配置未删除。在雷达直连场景下原DHCP无服务器，恢复后网口无IPv4属原先状态。无需恢复旧机.235，它始终只读。
