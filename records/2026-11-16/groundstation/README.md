# Qt任务入口及ROS2显示通道阶段记录

断电后重新连接 `.224`，NoMachine active，机载 Git `5859393` 干净。GitHub远端基线 `b345f7d` 可读。未做磁盘完整性测试。

## 实现与验证

- `groundstation/` 是原电脑地面站源码/运行资源的独立副本；来源文件SHA256见source-manifest.json。不包含历史ROS1机载源码、构建产物和示例大包。当前正式程序、桌面入口和服务未切换。
- Qt任务命令改为统一 `env.sh && exec`，不再加载Noetic。独立应用名隔离QSettings，默认地址`.224`。保留现有JSON命令、任务路径安全引用、航点version4和不透明会话对象。
- Qt实际QML交互125条断言通过，包括DDS会话字段透传、地图高度/拍照、ArUco源过期/控制状态/RC取消证据、人物ID和断线隐藏。属于模拟SSH，未启动真实控制器；截图为合成状态。
- 真实SSH→Humble原生管理器→Qt只读watch通过（隔离DDS域198、localhost）：没有飞控数据时fc=null，没有定位数据时map_session=null，任务未运行。没有用假数据表示真实飞控连接。
- `cloud_sender.py` 原生rclpy订阅，Reliable/depth1点云、sensor_data里程计；保留PCD1/LZ4和POS1二进制格式与米制坐标，不重复ENU/NED变换。点云/里程计源时间与接收时效、frame和四元数校验；退出释放线程/节点/socket/锁。ROS2电脑端无需本地ROS2安装。
- 真实ROS2消息→原生发送器→回环UDP隔离检查14条通过。原有Qt点云解码器正确解出3个有效点（含大端、行填充、FLOAT64输入及NaN剔除）；原Qt位姿接收器7组检查通过，包含90度方向、过期隐藏、旧时刻拒绝及新样本恢复。
- Qt主程序customgeometry与radarsimulator独立构建成功。首次系统CMake过旧失败，改用原地面站.venv-build现有CMake；没有升级系统构建工具。新增解码测试初次路径错误修正后构建通过。所有编译日志保留。

## 尚未验收

连接按钮依赖的飞前遥测、视频、定位/规划/录制入口仍有ROS1依赖；整体Qt界面尚不可视为ROS2可用。点云本阶段只验证机载回环UDP和电脑Qt解码，跨机网络数据链路/实际FAST-LIO输入尚待验证。没有物理雷达、云台、相机联合验收或PX4融合验证，没有向真实飞控发送控制命令。

## 重现与撤销

构建命令见groundstation/README.md。Qt任务测试使用tools/missions/test_qt独立CMake项目及Qt6.8.3；`QT_QPA_PLATFORM=offscreen check_mission_controls`为模拟交互。只读SSH测试有专用临时ssh代理，只允许watch并强制远端DDS域198/localhost，不能用作任务操作代理。

机载隔离测试命令：`ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 test_cloud_sender_ros2.py --sender cloud_sender.py --output <临时结果目录>`，需先source机载env.sh。测试资料取回电脑后清理机载临时目录。未部署地面站正式程序或新开机服务，无需恢复原应用；停止独立副本/发送器即可结束本阶段，原程序仍在原路径。不要删除其他地面站拥有的会话或切换生产服务。

附注：独立副本继承的ROS1参考源码存在原始尾随空白（git diff --cached --check报告），保留来源内容，不为迁移改写历史代码格式。再次清理时临时目录已不存在，未删除任何其他任务数据。
