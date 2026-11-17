# 移除独立 USB 相机 ROS 模块

2026-11-17，用户明确授权“usb相机的ros节点可以删除”。本次取消的是独立 XCX-S8130B / usb_cam 模块；Gemini335L 双目、云台视频及其他平台功能仍属于迁移范围。历史 USB 实验结果和失败不改写，不再将独立 USB 的接线、内参或候选补丁部署作为后续验收门槛。

## 已部署变更

- `.224` 产品 Git：`e35a2f5c8915a20369842fc82725f7171d0235fe` → `1de5c3d35e01b7fdcc5dd5443cb5c179a0087054`，最终工作区干净。
- 删除 `/home/orangepi/drone_platform/02_drivers/usb_camera`，包括私有源码、配置、兼容辅助节点和 install 产物；删除专用 `start_usb_camera.sh`。
- 统一传感器 launch 删除 USB 包/设备预检、include、`enable_usb_camera` 和 `video_device` 参数，统一入口删除 USB overlay。剩余模块为 MAVROS、Gemini、Livox、RTSP。
- 电脑/GitHub 同步删除独立 USB 专用工具、补丁和当前源码锁条目；相机检查工具只使用已安装的双目与云台实现，Gemini 实测通过更新后的统一入口启动。
- 没有修改内核 USB/UVC 驱动、雷达 SDK 或 MPP 二进制；未停止现有服务、启用正式入口或访问真实飞控控制接口。NoMachine 最终 active，Safina_sensor 保持 inactive。

## 验证

| 范围 | 结果 | 证据 |
| --- | --- | --- |
| 启动预检 | 5/5：空选择、非法布尔、缺串口、未分配雷达地址均在启动设备前拒绝；参数仅包含现有四模块 | `startup-evidence/remove-independent-usb-ros2/checks/preflight-result.json` |
| 异常退出清理 | 1/1：无害子进程退出7后 launch 返回非零，其余哨兵被回收 | `startup-evidence/remove-independent-usb-ros2/checks/unexpected-exit.json` |
| 包移除 | 正常产品环境下 `ros2 pkg prefix usb_cam` 返回 Package not found | `startup-checks.stdout.log`、`startup-checks.stderr.log` |
| 自动启动引用 | 检查90个系统/用户服务、autostart及shell启动文件，没有遗留 USB 节点引用 | `startup-reference-audit.json` |
| 双目+实际云台 | 32/32；像素变化、几何/格式、递增源戳、新鲜度、双QoS逐帧交付及正常退出均通过 | `camera-validation/new-camera-ground-check/result.json` |
| 独立最终读回 | 删除路径不存在；两个运行文件SHA一致；SDK/MPP摘要保持；临时目录/所属进程/临时网络均清理 | `final-readback.json` |

相机实际数据：

| 图像 | 格式与坐标系 | 实测频率 | 消息数 |
| --- | --- | ---: | ---: |
| Gemini彩色 | 1280×720 rgb8 / `camera_color_optical_frame` | 29.977 Hz | 1121 |
| Gemini深度 | 848×480 16UC1 / `camera_depth_optical_frame` | 29.958 Hz | 1132 |
| 云台灰度 | 1920×1080 mono8 / `gimbal_camera_optical_frame` | 30.065 Hz | 1029 |
| 云台彩色 | 960×540 bgr8 / `gimbal_camera_optical_frame` | 15.033 Hz | 511 |

云台灰度和彩色的 reliable 与 best_effort 读者分别收齐1029及511个发布源戳，彩色源戳为灰度子集。relay、observer、统一入口Gemini、MPP四个进程返回码均0。云台网络只在本轮使用 `save no` 临时配置；验证后恢复原连接，未启用永久网络或正式服务。

## 记录与恢复

原始相机取证完整保存在 `camera-validation/evidence.tar.gz`，展开目录保留源码、参数、结果与日志，构建中间文件仅在原始档案内保存。

`deployment-attempt2` 保存应用清单、应用脚本、旧产品文件档案和产品提交结果。`attempt1-readonly` 保留首次归档失败：电脑专用 `03_tools/systemd/README.md` 不在机载，tar因此返回2；当时未创建部署目录或改动产品，随后仅部署两个运行文件，文档仍留电脑/GitHub。全部物理检查一次通过，没有以删除独立 USB 为由改判旧实验失败。

若以后明确要恢复独立 USB，旧源码可从机载 Git 提交 `e35a2f5` 或电脑提交 `234e878` 读取；`deployment-attempt2/previous-product-files.tar.gz` 另外保留删除前的私有安装产物，恢复时同步还原入口和 overlay。此次没有执行恢复。

本轮仅完成用户授权的模块删除及剩余相机验证；完整平台仍需持久云台网络/正式入口确认、安装外参和运动精度、真实目标与任务地面整链及长期稳定性验收。
