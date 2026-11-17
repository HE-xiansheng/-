# .224 VS Code安装

按用户要求在机载.224安装微软官方VS Code stable ARM64 1.146.0，提交69047191fc427f2f92828279aae1d90d8eeda7ab。安装初始连接中断，用户确认重启；确认旧进程/文件已丢失后改电脑下载。官方tar.gz 382878987字节，保留下载前缀并通过HTTPS Range补齐中断区段，传入机载后正常解包。

安装为orangepi用户版本，无需sudo或系统软件源变更：~/.local/opt/vscode/1.146.0，current符号链接，~/.local/bin/code，应用菜单~/.local/share/applications/code.desktop。~/.bashrc追加有标记的.local/bin PATH条件块，以便新终端可直接code；保留原ROS配置。

ldd未见缺库；命令版本/arm64检查通过；普通用户命名空间可用，未禁用Electron沙箱。通过现有DISPLAY=:0实际启动，CLI退出0，X11窗口Visual Studio Code为1024x737且Map State IsViewable，启动日志为空。新交互shell command -v code与版本检查通过。update-desktop-database报告既有xfce4-about.desktop格式警告，非本次code.desktop；未修改该旧文件。SSH下交互bash的无TTY作业控制提示不影响验证。

应用已打开供用户使用。机载安装缓存/临时安装器/检查日志均清理，迁移工程Git仍干净；只留下应用本体、菜单、命令入口和正常应用用户数据。所有安装记录仅电脑/GitHub，安装包保存在电脑work目录。

若需卸载，先正常退出VS Code，删除本次code.desktop、~/.local/bin/code、~/.local/opt/vscode/current及1.146.0目录，并撤销.bashrc中以“# VS Code user installation: command path”标记的整个case块。保留用户工作文件和~/.config/Code，除非用户另行要求清除个人设置。
