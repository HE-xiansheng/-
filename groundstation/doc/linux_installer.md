# Linux 安装包

当前版本面向 Ubuntu 20.04、x86_64/amd64 桌面电脑。Qt 6.8.3、QML 模块、Qt 插件、ICU、必要的补充动态库和本地通信助手随包提供；使用系统图形驱动、Python 3、NumPy、OpenSSH 和 GStreamer。其他发行版未在本轮实机验证。

## 安装与启动

在安装包所在目录打开终端：

```bash
bash 科研飞行平台-地面站_1.0.20261104_ubuntu20.04_amd64.run
```

安装完成后，从桌面或应用菜单打开“科研飞行平台-地面站”。默认安装位置是 `~/.local/opt/research-flight-groundstation`。点击启动不会自动启动机载定位、规划或执行器；在应用中填写机载 IP 与认证信息后，使用已有功能入口。

安装器先检查 Python NumPy、SSH 和视频解码插件。缺少依赖时可执行提示中的 apt 命令，或给安装命令加 `--install-deps`；此选项使用 sudo 和系统软件源，需要联网。

包内不包含开发机的 SSH 私钥、密码、`.pointcloud-link` 现场配置、飞行录包或运行日志。机载侧应已部署当前地面站使用的程序接口；本安装包不部署机载算法。

## 卸载

先关闭该安装版地面站，再执行：

```bash
python3 ~/.local/opt/research-flight-groundstation/install_user.py --uninstall
```

卸载会还原安装前的同名桌面/菜单快捷方式；日志和通信记录保存在 `~/.local/share/research-flight-groundstation/uninstall-时间/`。不删除现有 Qt 应用设置。

## 重复制包

在源码目录执行：

```bash
../build-groundstation.sh
python3 tools/packaging/package_linux.py
```

默认输出目录为 `~/下载/科研飞行平台-地面站安装包/`。包含 `.run` 安装包、同内容的 `.tar.gz` 便携归档、安装说明及 SHA256SUMS。`package_linux.py --output 路径 --version 版本号` 可更改交付目录和版本。

安装包保持二进制、资源助手和运行时的相对目录布局，使原连接管理入口无需修改。安装时校验归档 SHA-256 和包内文件清单。运行记录仍写到用户可写的安装目录；因此使用用户安装器，不将此布局直接安装到只读的 `/opt`。

## 本轮验证

已实际完成自解压安装、带空格及中文路径安装、桌面入口格式检查、助手状态读取、Qt/QML 离屏启动、X11/OpenGL 三维界面首帧和卸载还原。验证在独立临时目录中进行，并用假 SSH 隔离机载访问。未替换开发目录中的运行程序，未进行实飞或远端部署。验证证据在 `analysis/linux-installer-20261104/validation.json` 和 `x11-validation.json`。

2026-11-10补充：1.0.20261110.1载荷合入Windows/WSL现场验证的Python监视回环和后台固定askpass修复。gcs-askpass必须保持可执行权限；不包含密码、私钥或用户连接数据。Qt主程序未重编译。WSL完整包装及升级校验见wsl_oneclick.md。

2026-11-10视频修复补充：1.0.20261110.2包含可执行GStreamer包装器与启动地址读取，主程序已补齐--host及用户设置初始化。包装器只让当前机载8554/gimbal直连，保留其他代理环境；不自动认证或保存密码。
