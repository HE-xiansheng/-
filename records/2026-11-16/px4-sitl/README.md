# PX4 1.16.0 独立 SIH SITL 基线

断电后已确认 .224 在线、NoMachine active、机载 bd6c781 干净；此前记录 f2ed248 已在 GitHub。

官方 https://github.com/PX4/PX4-Autopilot/tree/v1.16.0 ，实际提交 6ea3539157ca358c70a515878b77077af7d4611d，与此前飞控报告的提交前缀一致。独立源码、构建及 Python 环境均在电脑 drone-migration-work 下；现有用户固件工程只用于读取 Git 对象，未更改。子模块按官方锁定提交取出，源码状态和摘要见 manifest.json。

## 验证结果

独立 user/network namespace 中只有 lo，SIH quadx 启动，7 项通过：至少5条心跳、全部未解锁、PX4 身份、LOCAL_POSITION_NED、ATTITUDE、HIGHRES_IMU、正常退出。证据见 runtime-final/result.json、runtime-final/px4.log 和 startup-final.log。只发送版本/状态/listener/shutdown 控制台命令；无解锁、模式切换、起飞或航点指令。测试进程已退出。

这只完成官方仿真运行基线，尚未验证 ROS2/MAVROS 与该 SITL 的联合链路，也不等同真实雷达、外部视觉融合或实际任务验收。

## 复现

电脑私有 venv 内锁定依赖见 python-lock.txt。构建配置使用 CMake 3.31.10、Ninja，CONFIG=px4_sitl_default，禁用 Gazebo Classic 自动发现（本阶段只用 SIH）；构建目标 px4，4 并发。使用一个全新运行目录：

```bash
unshare --user --map-root-user --net \
 /home/tyzk/drone-migration-work/px4-sitl-venv/bin/python \
 tools/verify_px4_sitl_startup.py \
 --build /home/tyzk/drone-migration-work/build-px4-sitl \
 --runtime /home/tyzk/drone-migration-work/sitl-new-run
```

## 问题与处理

- 系统 venv 缺 ensurepip：通过已有 pip --python 安装到私有 venv，未更改系统 Python。
- Git 2.25.1 部分克隆持续重复获取：停止该进程，保留 clone.log，改无 filter 的普通浅克隆 clone-full.log。
- 默认配置发现本地 Gazebo Classic 并自动下载其子模块：停止该配置，禁用对应 find_package 后重新配置，仅构建 SIH 基线；不是 Gazebo 验收。
- 首次构建缺嵌套 pymavlink/mavgen.py：按锁定提交复制嵌套 Git 对象；辅助递归脚本对空 .gitmodules 返回1，嵌套 checkout 已完成。
- 随后缺 genmsg：私有环境补 pyros-genmsg、lxml、future 后构建成功。
- 首次网络保护检查错误读取宿主 sysfs：改 socket.if_nameindex() 检查当前网络空间。
- 首次结果序列化不支持 MAVLink bytearray：显式转换为整数列表，最终重跑通过；失败日志保留。
- 对所有未初始化子模块递归列状态失败：实际使用的模块单独核对，见 manifest.json；不把可选仿真器或 NuttX 列为已准备。

机载运行环境未切换，本阶段无机载部署或迁移记录。要撤销本阶段，只需删除电脑上独立的源码/构建/venv/运行目录；不会影响机载或既有用户固件树。全部实验日志保留在电脑和 GitHub。
