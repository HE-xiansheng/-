# Windows 安装包

目标平台为 Windows 10（1809及以后）/11 64位，使用当前地面站C++/QML源码构建，Qt6.8.3动态链接。不包含完整工程源码、开发机密码、SSH私钥或飞行录包；包含运行必需的Python通信助手。

## 使用

双击“科研飞行平台-地面站_1.0.20261109_windows_x64_Setup.exe”，安装完成后从桌面或开始菜单启动。默认安装到当前用户的LOCALAPPDATA/Programs/ResearchFlightGroundStation，无需管理员权限、Qt开发环境或另装Python。也可完整解压同版本便携ZIP，在bin中运行customgeometry.exe。

左侧“机载通信”填写目标IP和密码，默认用户orangepi；留空使用已有SSH密钥。密码仅在本次进程环境中传递，Windows随包原生askpass读取，不写配置和日志。SSH主机指纹检查保留，与Linux版一样仅自动登记新地址；已有地址指纹变化仍拒绝，需核实设备后更新known_hosts。

防火墙首次询问时允许地面站和随包Python在专用网络通信。电脑和机载必须网络可达。点云默认UDP9000，位姿9001，话题9003，栅格9004，原始监视9005。修改前3个端口后重新应用连接。启动程序只监听和显示，不自动启动机载定位、规划、解锁或切换模式。

## Windows适配

- 复用既有OpenSSH与机载命令，遥测使用随包Python3.11.9、NumPy1.26.4及LZ4。Windows匿名管道由读线程采样，保留超时与最新消息语义。
- Windows遥测管理器持有Job，正常关闭地面站或终止遥测管理器时清理其后代遥测进程，不管理其他进程或机载ROS服务。Linux原有管理入口保持。
- 云台RTSP使用随包FFmpeg输出640×360 RGB，双目和识别画面仍通过现有SSH图像接口。
- 录包支持未压缩、LZ4、BZ2；数据缓存由Qt用户应用数据目录管理。
- 卸载会将bin/logs复制到LOCALAPPDATA/ResearchFlightGroundStation/uninstall-logs，录包缓存和用户Qt设置保留。

## 复现

Windows构建支持现有CMake工程，新增ssh-askpass目标和程序图标。Linux交叉构建使用项目外置MinGW交叉工具链、Windows Qt6.8.3与相同版本Linux Qt主机工具；CMake需设置CMAKE_TOOLCHAIN_FILE、QT_HOST_PATH和CMAKE_PREFIX_PATH。LZ4和BZip2必须使用Windows目标库，不能混入Linux库。

打包入口tools/packaging/package_windows.py从build/windows收集Release二进制，收集Qt DLL/QML/插件、MinGW运行库和依赖，用NSIS生成64位Unicode用户安装程序及同内容ZIP。构建产物有SHA256SUMS和包内payload-manifest.json。实际验证范围以输出目录的验证结果.json为准。

在本机已有工具链下，可执行 `bash tools/packaging/build_windows_cross.sh` 重建；再执行 `python3 tools/packaging/package_windows.py --mingw-runtime ../.deps/windows-assets/qt-tools/Tools/mingw1310_64/bin` 制包。交叉构建必须先成功链接，再移除Qt入口库调试节；打包器会拒绝空文件和节越界的PE文件。

Qt目标系统下限参考：https://doc.qt.io/qt-6.8/supported-platforms.html 。本轮在Wine10隔离环境完成启动、认证、回放与生命周期检查；未完成Windows实体机与真实机载联网验收。
