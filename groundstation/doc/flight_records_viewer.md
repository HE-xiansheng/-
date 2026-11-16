# 飞行数据读取与三维回放

入口：左侧“飞行数据录制 → 读取机载飞行数据”。独立窗口读取、下载与解析，不依赖当前静态预览是否启动，不修改实时点云、规划目标、录制管理器或飞控状态。关闭查看窗口会取消尚未完成的文件读取。

## 读取和文件完整性

复用当前机载IP与SSH会话凭据，经SSH标准输出直接传输二进制文件，无需新端口、ROS网络暴露或远端安装。随程序携带的Python3工具仅列举 `~/drone_platform/01_apps/ego_planner/flight-records` 中已经结束、存在完整 `.bag` 的子目录。目录内有 `.active` 或录制状态仍是准备/录制/收尾时不开放读取；不读取符号链接或目录外文件。每个文件在传输前后复查大小与纳秒修改时间，机载文件变化则拒绝本次下载。上位机核对字节数，经QSaveFile原子落盘，取消/失败不会提交半包。

每次选择一轮记录中的一个分包，只下载与解析该文件。本地缓存使用Qt AppLocalDataLocation下 `flight-records/<host>/<record>/`，原始包仍保留在机载。超时、认证失败、目录不存在、磁盘不足或文件发生变化均显示原因，并可刷新重试。读取使用独立QProcess，不占用录制控制的操作进程。首次下载需要正常机载网络；缓存 `.bag` 可通过“打开本地录包”再次选择。

## 数据解析

- 支持ROS1 bag v2，未压缩、LZ4 frame与BZ2 chunk。LZ4/BZ2依赖构建时可选系统库，解压时检查长度与压缩校验；正式查看器不要求安装ROS或Python ROS模块。
- 世界点云优先 `/factory_fastlio/cloud_world`，其次 `/cloud_registered`、`/laser_cloud_surround`、`/livox/cloud_world`。不把原始雷达坐标点云或占据/膨胀栅格混入世界地图。
- 定位优先 `/factory_fastlio/odometry`，其次 `/Odometry`、`/aft_mapped_to_init`、`/mavros/vision_pose/pose`。支持nav_msgs/Odometry与geometry_msgs/PoseStamped，不混用MAVROS局部里程计或 `/goal` 规划目标。
- 窗口只显示所选单分包，独立时间轴对应这个分包。缺少世界点云会明确失败；缺少匹配位姿则显示点云，并说明没有轨迹，不能以规划轨迹代替实测轨迹。
- 点云按上限抽稀，当前窗口累计点云上限40万；位姿保留原样，轨迹渲染最多约2万点。使用后台解析、文件映射和单chunk解压，避免在UI线程扫描大录包或同时解压整轮记录。

## 查看

橙线为实测定位轨迹，青点为当前时间对应位置。默认聚焦轨迹；“完整点云”可重新适配整个模型。左键拖动旋转，中键平移，滚轮缩放；点云始终显示该分包的累计地图；播放/拖动时间轴只显示截至当前时刻的轨迹与位置，向后拖动会移除未来轨迹。加载后初始进度位于分包末尾，点击播放从头开始。顶部隐藏默认按定位原点Z=1.8m裁剪显示，不改变原文件和轨迹；取消勾选即可恢复。

Logo采用深灰圆角背景板，蓝绿图标从原JPEG在渲染时裁切、去白底，不改动原始品牌图片。相邻文字继续显示“科研飞行平台-地面站”。

## 验证

`tools/flight-records/tests/` 包含只读工具、Qt控制器、真实机载归档解析及实际三维窗口截图检查。`make_fixtures.py`需要ROS开发环境，仅用于从真实归档生成LZ4/BZ2分包和独立基线。通过倒序分包、注入错误MAVROS坐标和逐字节下载校验，验证时间/坐标来源与文件完整性。验证产物位于工作区 `.planning/flight-records-viewer/`。

开发环境复现示例（Qt路径按本机安装调整）：

```bash
python3 tools/flight-records/tests/test_remote_records.py
cmake -S tools/flight-records/tests -B build/record-check -DCMAKE_PREFIX_PATH=/path/to/Qt
cmake --build build/record-check
# 在已source ROS的开发环境生成基线；正式程序无需ROS：
python3 tools/flight-records/tests/make_fixtures.py /path/to/real/inputs_0.bag /tmp/record-fixtures
# 使用实际桌面/OpenGL显示，生成三维窗口与品牌徽章截图：
build/record-check/check_records /path/to/real/inputs_0.bag /tmp/record-evidence /tmp/record-fixtures
```

集成检查以真实机载归档为输入，替换SSH执行端为本地只读测试端，覆盖列表、完整二进制传输、整轮多分包、取消和错误反馈，以及点击真实窗口“读取并绘图”后的完整解析与渲染。该检查不代表已在当前机载IP上完成现场在线拉取；机载网络不可达时会如实提示连接错误。

## 2026-10-29 回放交互调整

回放窗口改为单分包：先选择飞行记录，再选择一个 `.bag` 分包，点击“读取并绘图”；只下载、解析该分包。打开本地录包也只选择一个文件，标题显示当前文件名，不再默认合并整轮。

点云固定为该分包的累计地图，播放和拖动时间轴不再切换点云时间窗。橙色轨迹只显示从分包开始至所选时刻的实测定位，向后拖动会移除未来部分，青色位置标记与可见轨迹末端对应。加载完成先显示分包末尾的完整轨迹，点击播放从头开始。普通静态文件查看器保留原先的点云时间窗功能。

删除常驻点数/轨迹条数、完整加载信息、话题来源、操作图例等小字，仅保留必要的加载进度、错误/缺失数据反馈和时间轴。ROS监视面板改名为“ROS话题栏”，与“视频流拉取”共用折叠标题及面板宽度。
