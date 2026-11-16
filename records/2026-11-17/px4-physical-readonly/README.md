# 真实PX4固件及外部视觉参数只读核对

机载.224的物理飞控，经原by-id串口读取20秒。先以sudo fuser确认串口无人占用，再由TIOCEXCL独占；原Linux串口属性在finally恢复，独占锁释放。临时代码和官方pymavlink2.4.49生成的common.py取自电脑已有环境，未安装机载Python包；执行结果只输出到电脑，机载/tmp/px4-physical-readonly已删除。

## 实测结果

- HEARTBEAT：PX4、system1/component1、四旋翼，报告未解锁。
- AUTOPILOT_VERSION：1.16.0 stable（[1,16,0,255]）；wire custom bytes为000000579153a36e，倒序可见前缀6ea3539157，与电脑官方v1.16.0提交6ea3539157ca358c70a515878b77077af7d4611d前缀一致。只能核对声明版本与前缀，不证明完整固件构建相同。
- 返回11个参数，按capabilities声明的MAVLink bytewise编码解码；不能把INT32值直接当浮点数取整。保留原始float/4字节hex及MAV_PARAM_TYPE。

| 参数 | 实读值 | 官方1.16语义 |
| --- | --- | --- |
| EKF2_EV_CTRL | 15 | 允许水平位置、垂直位置、三维速度及yaw融合 |
| EKF2_HGT_REF | 3 | Vision高度参考 |
| EKF2_EV_DELAY | 0 ms | EV相对IMU延迟配置；不证明实际链路零延迟 |
| EKF2_EV_POS_X/Y/Z | 各0 m | EV传感器相对机体的配置偏移；不能据此认定物理外参为零 |
| EKF2_EV_NOISE_MD | 0 | 使用EV报告方差，参数为下限 |
| EKF2_EVP_NOISE / EVV_NOISE / EVA_NOISE | 各约0.1 | 位置/速度/角度噪声参数 |
| EKF2_EV_QMIN | 0 | EV最低质量门槛配置 |

EKF2_AID_MASK、EKF2_HGT_MODE及EV_P/V/Y_NOISE探测名称没有返回；它们不在此次官方1.16的现行参数定义中。缺失项保持记录，不能将所有参数请求写作成功，也不据此添加或改写参数。官方参数定义以Git对象原文归档params_external_vision.yaml及module.yaml；出处为https://github.com/PX4/PX4-Autopilot/tree/6ea3539157ca358c70a515878b77077af7d4611d/src/modules/ekf2。

## 发送范围与检查

工具只允许GCS心跳、明确名单PARAM_REQUEST_READ和COMMAND_LONG MAV_CMD_REQUEST_MESSAGE(512)，后者仅请求AUTOPILOT_VERSION(148)，其余参数全零。未发送PARAM_SET、SET_MODE、解锁、设定点、外部视觉或任务数据。完整发送列表在stdout.json；没有伪装成只读的实际MAVROS节点。

本地PTY模拟13项通过：拒绝解锁/模式/参数写入及非名单请求，固件和bytewise参数解码，线包类型限制，原终端设置恢复及锁释放。物理6项连接/固件/关键参数/未解锁/发送范围检查通过。测试代码仅电脑tools，不加入机载运行入口。读参数/固件不会改变PX4配置，部署回退不涉及飞控。

该查询与600秒并发测试时段重叠约20秒，加入少量只读串口/协议解析负载，不能将该段称为完全无其他进程的CPU基线。

## 仍需验证

这不是外部视觉融合验收。需要真实雷达输入、实际安装外参、源时刻/飞控时间同步及PX4实际接收与融合状态。特别是vision_pose仅提供位姿，配置允许速度融合不表示已经提供速度测量；15不能作为四类融合均生效的证据。未修改任何飞控参数或为测试注入回放位姿。
