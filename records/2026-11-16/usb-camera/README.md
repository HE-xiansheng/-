# 独立 USB 相机 ROS2 适配

参考机实际启动usb_cam 0.3.7，配置/dev/usb_cam、1280×720 MJPEG/25Hz、head_camera坐标系，话题/usb_cam/image_raw与/usb_cam/camera_info。只读检查时两台机载均未接独立USB相机，.235别名不存在，ROS图虽有发布者注册，但这不能证明有图像数据。.224的video0为HDMI，其余当前视频节点属于Gemini335L。

使用[官方usb_cam 0.8.1](https://github.com/ros-drivers/usb_cam/tree/0.8.1)，固定提交cb2ae6bc0a312de629f84fef933fb3e07dac1ef8。官方README说明ROS2支持及原生安装方式；此处在独立私有workspace构建，不更换内核/UVC/BSP、不安装udev别名、不修改既有传感器服务。

## 配置与接口对应

| ROS1配置/接口 | ROS2处理 |
| --- | --- |
| /dev/usb_cam | 保留默认别名；缺失时失败，不改选video0或Gemini |
| image_width/height=1280/720 | 保持，实际设备需另核实支持模式 |
| framerate=25 | ROS2要求double，配置25.0 |
| pixel_format=mjpeg | 使用ROS2明确的mjpeg2rgb，输出sensor_msgs/Image rgb8 |
| color_format=yuv422p | av_device_format=YUV422P |
| camera_frame_id=head_camera | frame_id=head_camera，保持原话题命名空间 |
| 自动白平衡/曝光/对焦 | 对应auto_white_balance/autoexposure/autofocus=true |
| exposure_auto_priority=true/power_line_frequency=1 | 补充上游缺少的两个可选启动参数，固定为1/1，运行中只读 |
| ignore列表中的亮度/对比度/饱和度/增益/锐度等 | -1表示不设置；自动曝光/白平衡/对焦开启时不写对应手动值 |
| start_capture/stop_capture Empty配置名称 | 兼容节点转发到原生set_capture SetBool；原机无设备时尚未注册采集服务，名称依据配置语义准备 |

原生SetBool返回实际采集状态；修正上游0.8.1未设置response.success的问题，并捕获采集错误返回false。Empty无法表达拒绝，兼容服务将结果保留在/usb_cam/capture_status（JSON、transient_local）；原生响应拒绝、缺失和超时不会伪装为状态成功。兼容等待服务1秒、响应2秒，没有自动重试。

独立入口03_tools/config script/start_usb_camera.sh仅在显式运行时加载USB overlay。launch预检设备存在、字符设备、V4L2、USB祖先、读写权限，并拒绝Orbbec厂商2bc5和HDMI/platform节点。完成绑定后才启动采集及兼容节点；采集进程退出触发整个本次launch退出。

## 验证证据

- RK3588/ARM64/Humble原生构建成功；修改后的最终10组上游CTest通过（其中2组gtest及8组静态检查）。
- 19项隔离ROS2检查通过：缺失别名/普通文件/非视频字符设备/当前8个Gemini节点/HDMI拒绝，旧采集服务名称转发、拒绝/缺失/无响应时限及正常退出。域84、localhost，仅服务mock参与，不打开实际视频设备。
- 实际MJPEG2RGB解码器处理5帧1280×720合成422 JPEG，输出rgb8、缓冲区前后保护区完整；与OpenCV独立解码平均绝对差4.98/255。实测合成解码约10.1ms/帧，不代表硬件帧率或整链延迟。
- 实际shell入口参数查看通过；缺失别名及HDMI均在启动节点前明确失败。
- 54个机载源码/配置文件与电脑SHA256一致，机载bd6c781；临时构建、测试、图像及日志目录取回后清理，私有运行安装产物保留，Git干净、NoMachine active。完整日志、上游测试结果、清单均在本目录。

尚未验收真实独立USB相机的设备身份/udev绑定、支持格式、帧率、曝光/电源频率控制、源时刻、采集服务和图像持续性。camera_info_url为空沿用旧配置，不代表已有有效标定；不得用于未经标定的度量定位。没有将Gemini或HDMI替代为已验收的独立相机。

回退：该模块未加入既有start_sensors入口或服务，不使用新入口即保持当前部署行为；若要移除，可恢复机载bd6c781父提交中新增模块/脚本的状态，私有install可在确认无使用进程后移除。系统驱动、udev和正式入口均未变更，无需回退底层驱动。回退未执行。
