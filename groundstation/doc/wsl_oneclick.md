# Windows11 WSL一键配置与地面站安装

交付包是“科研飞行平台-WSL一键安装包.zip”，内含Windows入口、PowerShell配置脚本、Linux依赖安装脚本及1.0.20261110版Linux地面站安装文件。目标为Windows11 22H2及以上、x64架构；使用Ubuntu20.04与WSL2。含Ubuntu版将WSL3.0.5运行时和Ubuntu官方镜像一并打包，只有Ubuntu软件依赖及部分Windows组件补充文件仍可能需要联网。

完整解压后双击“一键安装.cmd”。脚本在原Windows用户下操作其WSL；仅Windows组件、WSL MSI与防火墙步骤弹出管理员授权。已有Ubuntu20.04会被复用；网络配置保留原有.wslconfig其他字段并备份。UDP9000/9001/9005分别用于点云、位姿、机载原始话题监视，Windows规则限制本地子网；不关闭防火墙、不把默认入站策略改为全放行。

只有系统组件或WSL更新明确报告需要重启时才提示重启Windows，不自动重启。网络模式变化时使用wsl --shutdown重启WSL，其他WSL工作需要事先保存。重新运行会检查真实状态并继续；进度和日志存于LOCALAPPDATA/ResearchFlightGCS-WSL-Setup。

Linux依赖由WSL root安装，图形地面站由普通用户安装并启动，不需要收集Linux或机载密码。已有默认普通用户时使用该用户；默认root时为程序创建普通用户gcs，不设置密码或免密sudo。只有本工具新建的发行版才修改其默认用户；已有发行版默认用户保留。同版本主程序校验一致则复用，遇到其他安装内容时使用独立版本目录，不删除旧程序或用户数据。

完成后创建“科研飞行平台-地面站（WSL）”桌面及开始菜单快捷方式，显式使用点云9000、位姿9001和预检9003的实时启动参数，保留完整功能栏。机载IP和认证信息在程序中填写；安装过程不访问或控制机载电脑，也不改变SSH指纹校验。

实现位于tools/packaging/wsl/。package_wsl.py从Linux.run提取版本和主程序SHA256，生成PayloadInfo.json，PowerShell脚本使用UTF-8 BOM兼容Windows PowerShell5.1，批处理使用ASCII/CRLF。主程序和全部Qt依赖在.run中；不附带完整C++/QML源码、SSH私钥、密码或飞行录包。

构建示例：先用build-groundstation.sh和tools/packaging/package_linux.py生成Linux.run，再执行：

```bash
python3 tools/packaging/wsl/package_wsl.py --payload /path/to/地面站.run
```

验证包括PowerShell语法和模拟Windows API的配置保留、重启条件、指定规则与端口、安装包篡改拒绝和启动参数检查，以及Linux载荷实际安装、主程序哈希、Qt/QML首帧和卸载还原。尚未在Windows实体机或实际WSL环境完成端到端验收。

官方配置依据：[WSL网络](https://learn.microsoft.com/windows/wsl/networking)、[WSL配置](https://learn.microsoft.com/windows/wsl/wsl-config)、[Hyper-V防火墙](https://learn.microsoft.com/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall)。

## 内置WSL离线运行时

一键包现可附带微软官方WSL3.0.5 x64 MSI（官方SHA256 `2313efc5873e3f8552f42e77f950fabd913c0289078fa89d2100f3421a525eaa`）。主程序安装前校验摘要，管理员助手检查Windows Authenticode微软签名后才调用msiexec，使用`/norestart`与`/L*V`详细日志。已有版本>=2.0且版本探测成功时复用，不强制升级；已有更高版本但不可用时停止而不降级。

含Ubuntu版中，WSL本体、Ubuntu镜像和地面站在包内；apt依赖及部分Windows组件仍需联网。不会清空任何WSL缓存、发行版或用户数据；本包不提供自动清理功能。离线MSI不代表已修复通用“灾难性故障”，需要根据详细日志判断。

打包时追加`--wsl-msi /path/to/wsl.3.0.5.0.x64.msi`；构建器对照微软发布页固定摘要验证文件，避免误包其他程序。Windows端签名、MSI安装和WSL端到端流程未在本轮实机验证。

## Ubuntu本地导入与乱码修复（用户实机日志）

2026-11-10用户日志显示：第一次启用WSL组件要求重启；第二次组件检查已通过，WSL MSI退出码0且日志确认安装成功。随后`wsl --install`访问`raw.githubusercontent.com/microsoft/WSL/master/distributions/DistributionInfo.json`超时，错误为`Wsl/InstallDistro/WININET_E_TIMEOUT`。这轮故障发生于Ubuntu清单下载，而非WSL运行时安装。

含Ubuntu版追加官方`ubuntu-20.04.6-wsl-amd64.wsl`。其SHA256为`a9073f3726aab9661076506603706eadf284b80a791dec3adfde8e1989906fa4`，由Ubuntu官方`SHA256SUMS`及有效GPG签名核验。文件是gzip压缩的根文件系统tar；使用`wsl --import Ubuntu-20.04 <目录> <镜像> --version 2`导入，不使用在线清单。已有同名发行版直接复用，非空导入目录不清理而改用新目录。构建命令追加：

```bash
python3 tools/packaging/wsl/package_wsl.py --payload /path/to/地面站.run \
  --wsl-msi /path/to/wsl.3.0.5.0.x64.msi \
  --ubuntu-image /path/to/ubuntu-20.04.6-wsl-amd64.wsl \
  --output /path/to/科研飞行平台-WSL一键安装包_含Ubuntu.zip
```

WSL原生CLI重定向输出常为无BOM的UTF-16LE，Linux子进程通常输出UTF-8。新的进程读取器保留stdout/stderr原始字节，各自解码并保留退出码；双管道异步读取，长任务持续显示运行时长，结束后显示输出并写入日志。已回归中文错误、双管道大输出及非零退出码，不再仅去除空字符掩盖乱码。

无需卸载已安装的WSL或清理发行版，完整解压新版后再次运行即可。依赖安装仍需访问Ubuntu软件源，本包不声称完全离线。PowerShell语法/模拟Windows API及真实跨进程读取已本地验证，Windows实体机上导入和WSLg完整流程仍待复测。

来源：[Ubuntu官方镜像及校验文件](https://releases.ubuntu.com/focal/)、[微软导入Linux发行版说明](https://learn.microsoft.com/windows/wsl/use-custom-distro)。

## 原生WSL参数修复（安装脚本2026.11.10.2）

用户后续截图显示版本检查收到“没有已安装的分发”。已确认脚本将所有参数都用Windows通用引号包装，生成了`wsl.exe "--version"`。WSL3.0.5原生选项按原始文本比较，不会去除此引号，因此误入Linux命令启动分支；这也是安装器未复用可用运行时的原因。修复为无空格的WSL选项、发行版名及用户名保持原样，包含空格的路径仍安全转义；安装过程和桌面快捷方式统一使用新参数生成器。失败日志包含所用参数，窗口显示脚本修订号。

原完整包可覆盖补丁中的Common.ps1和Setup-GroundStation.ps1后继续，不需要清理WSL。修复新增8项回归检查，共91项通过；仍需Windows实机复测，不以模拟测试替代实际WSL验收。依据：[微软3.0.5命令行处理](https://github.com/microsoft/WSL/blob/3.0.5/src/windows/common/WslClient.cpp)、[原始参数读取](https://github.com/microsoft/WSL/blob/3.0.5/src/windows/common/helpers.cpp)。

## 依赖下载重试（2026.11.10.3）

用户实机通过Ubuntu导入、镜像网络和防火墙步骤，APT下载135MB后仅libio-socket-ssl-perl连接失败而返回100。安装脚本为APT update/install增加`Acquire::Retries=5`、`Acquire::http::Timeout=30`和`Acquire::https::Timeout=30`，选项只作用于当前命令。安装失败保留原退出码及明确的缓存/续装提示，仍停止后续用户创建和应用安装，避免把依赖失败误报为成功。不执行clean/autoclean或切换软件源，缓存中的有效deb继续由APT校验并使用。

已有安装目录可用累计补丁覆盖Common.ps1、Setup-GroundStation.ps1和install-in-wsl.sh后再次双击。自动重试应对临时网络故障，不能保证绕过持续不可达的软件源。

依据：[Ubuntu20.04 APT Acquire重试与超时配置](https://manpages.ubuntu.com/manpages/focal/man5/apt.conf.5.html)。

## 防火墙重跑与错误诊断（2026.11.10.4）

用户14:53重跑：组件检查成功，可用WSL3.0.5复用，第4步管理员助手返回“拒绝访问”。host-Firewall JSON只含phase/reason/success，无具体命令，无法据此把权限拒绝归因到删除、创建或读取。原脚本每次都会删除再创建两层同名规则；现已改为检查本工具名称、方向/动作、启用状态、WSL标识、UDP协议、9000/9001/9005端口，Windows规则另检查Any配置文件和LocalSubnet来源。符合时复用，不符时原位Set自身规则，缺少时New。查询只把ObjectNotFound视为缺失，权限错误不被吞掉或按缺失重新创建。不关闭防火墙、不改变默认入站策略、不跳过失败继续报告成功。

异常包含具体防火墙步骤及命令名；管理员结果JSON新增原始错误ID、类别、位置、异常类型与HRESULT，主窗口包含结果文件路径。即使后续仍拒绝访问，也可据新日志定位受限的调用。修复不代表Windows策略限制已解除，仍需用户Windows实机复测。

验证覆盖首次创建、重复无写操作、端口顺序/UDP17形式、禁用及错误范围原位更新、查询/更新拒绝、ObjectNotFound和仅自身规则；104项PowerShell检查通过。

微软接口说明：[读取Hyper-V规则](https://learn.microsoft.com/powershell/module/netsecurity/get-netfirewallhypervrule)、[原位修改Hyper-V规则](https://learn.microsoft.com/powershell/module/netsecurity/set-netfirewallhypervrule)、[Windows端口过滤器](https://learn.microsoft.com/powershell/module/netsecurity/get-netfirewallportfilter)。

## 同名Windows映射规则冲突（2026.11.10.5）

用户新版日志定位到Set-NetFirewallHyperVRule拒绝访问；随后只读查询截图给出确定证据：旧规则Name=ResearchFlightGCS-WSL-UDP，VMCreatorId=Any，PolicyStoreSourceType=HostFirewallLocal，其余字段为UDP、9000/9001/9005、Inbound、Allow、Enabled=True。微软CIM文档说明HostFirewallLocal表示规则来自宿主Windows防火墙本地存储。原脚本对两层使用同名标识，查询到该映射后因WSL GUID不符而误尝试更新。上版仅改为复用仍不足以解决来源冲突。

现将Windows规则名保持ResearchFlightGCS-WSL-UDP；Hyper-V专用本地规则使用ResearchFlightGCS-WSL-HyperV-UDP。仅更新PolicyStoreSourceType为Local/1的本地Hyper-V规则；其他来源一律报告并停止，不修改映射或MDM规则。按GUID值比较WSL标识，忽略大小写和括号表示差异。旧规则及Windows映射保留，不卸载、不清缓存、不关闭防火墙、不改全局入站默认策略。

新增按实际截图复现的HostFirewallLocal/Any场景、两层不同名称、反复运行复用、GUID表示差异和拒绝修改非本地规则检查，共113项本地检查通过。完整包和四脚本累计补丁同步更新，用户Windows完整续装仍待复测。

字段定义：[微软MSFT_NetFirewallHyperVRule](https://learn.microsoft.com/windows/win32/fwp/wmi/wfascimprov/msft-netfirewallhypervrule)。

## 只读诊断包（监视一直连接）

用户已在Windows/WSLg打开完整界面并通过SSH认证，但话题栏显示“正在连接机载只读监视”。此文本由本地bridge.py发送为connected=false采样状态，至少表明接收器收到过本地桥接消息；不能据此认定ROS话题或9005回传正常。先前9003绑定失败提示与后续等待状态需要按时间区分；用户查询当时WSL仅一份customgeometry，WSL及Windows9003端点均无结果，不能归因为重复进程。

只读诊断源代码位于tools/packaging/wsl/diagnostics。独立解压“科研飞行平台-WSL只读诊断.zip”，保持界面打开，双击一键诊断.cmd；桌面生成JSON，包含本地桥接/点云最近日志、进程和UDP端点、Windows保留端口、WSL网络模式等。不执行安装、不重启服务、不登录机载、不发飞行控制，不读取密码环境或私钥。日志读取限已知文件及尾部，常见明文密码赋值额外脱敏。

Linux采集已在本机运行；PowerShell主流程通过模拟Windows原生命令及真实Linux采集组合验证，覆盖UTF16/UTF8、中文/空格路径与JSON完整输出。Windows真实结果仍需用户运行后分析。

## 合入实机运行修复并重新打包（1.0.20261110.1）

用户导出的地面站运行修复源文件.tar.gz摘要为b15f209dcd24174eac2db5235ee0a2285074c0ae4ca1db26e2289ad64af292c9，与随附SHA256一致。归档仅含tools/preflight/bridge.py、tools/pointcloud-only/manage.py、tools/communication/ssh_session.py和tools/communication/gcs-askpass四个普通文件，已审查并逐字节合入正式源码，固定助手保留0700可执行权限。Qt主程序沿用既有已验证二进制，不因Python运行修复重编译。

WSL内部状态与栅格发送仅在检测到Microsoft Linux内核时绑定lo，避免该现场镜像网络下的回环分片/丢包；普通Linux与原生Windows路径保留。状态发送错误限频记录，不再静默吞掉。Linux后台SSH采用固定助手及已登记主机指纹严格校验；Windows继续使用已有原生askpass入口。密码只从会话环境读取，不写入脚本或日志。每次重新启动GUI后需在机载通信输入密码并应用连接，这是前台会话认证的原有行为。

WSL PayloadInfo新增payload-manifest.json摘要，安装时核对可信清单、版本、主程序及全部通信/桌面助手，固定askpass还检查可执行权限。即使主程序字节未变，只要旧Python助手不同也不会判为可以复用。版本升为1.0.20261110.1，Linux.run与外层含WSL/Ubuntu ZIP均重新生成；安装脚本修订2026.11.10.6。

旧版在独立目录保留。upgrade_prepare.py只识别用户安装标记所属的旧安装，先检查旧GUI是否关闭，再以该普通用户调用旧版bridge.py stop及manage.py stop，回收其本地只读遥测端口，不改变机载服务。正在运行旧GUI时提示关闭并重跑，不强行终止。

验证区分：用户Windows现场已验证原四文件修复及正常输入密码后的通信；本次整合执行本地UDP/askpass/重连/安装复用/升级隔离检查和Linux安装/首帧/卸载验证，不能替代最终新完整包的Windows首次安装/升级验收。

## 合入云台视频修复（1.0.20261110.2）

用户导出的地面站云台视频修复源文件.tar.gz摘要087d5ac2c7418875260a7b58e403e57d830decb2fe8fc37f384e2f5df643d8b7与侧车一致。两份0755脚本逐字节合入：根启动脚本回写打包源tools/packaging/start_bundle.sh，解码包装器放在tools/desktop/video-bin/gst-launch-1.0。后者只为rtsp://当前机载地址:8554/gimbal追加no_proxy和NO_PROXY，随后调用系统/usr/bin/gst-launch-1.0；不删除其他代理配置，也不改变Windows或WSL系统代理。

整合时发现正式main.cpp原来忽略--host，补齐--host与--host=两种参数及地址校验。启动时用同一地址初始化VideoController和ConnectionController，其他任务控制器继续绑定通信地址；没有显式参数时沿用当前用户QSettings保存的onboard/host。启动器在--live模式下从当前安装目录connection.json补参数，显式参数优先。新安装目录没有connection.json时仍读取用户设置；启动不自动认证、保存新地址或执行机载任务，保存仍在SSH认证成功之后。

安装复用校验补查视频包装器和根启动器的可执行权限。Qt主程序因地址初始化修复重新编译，载荷与完整WSL包版本升至1.0.20261110.2；保留前版全部安装和通信修复。用户现场已确认两路视频恢复；本地脚本/实际Linux安装启动验证不代替新版完整包的Windows首次安装、升级及真实视频验收。

## 一键卸载（1.0.20261110.3）

完整包增加一键卸载.cmd、Uninstall-GroundStation.ps1、uninstall_in_wsl.py和卸载说明.txt，同时提供不含载荷的独立卸载ZIP，兼容此前WSL包。先关闭地面站窗口，再双击卸载入口，专用规则清理通过已有管理员助手执行。

Windows入口沿用安装用户的WSL注册和SID互斥锁，避免安装/卸载并行。读取当前用户安装结果选择Linux账户；Ubuntu或目标账户未创建时跳过程序卸载。Linux入口只用root发现账户，实际备份和删除均以原普通用户执行。只匹配其~/.local/opt下带产品登记文件的默认/WSL版本目录，不删除源码目录、其他用户或未登记目录。

删除前核对所有版本的运行窗口，回收这些版本自管的本地只读遥测后台；不调用机载定位、规划或飞控服务。安装目录内不属于payload-manifest的文件先复制到~/.local/share/research-flight-groundstation/uninstall-时间-随机编号，全部备份成功后才删除程序。符号链接不跟随到目标；安装目录外的回放录包、用户设置和SSH文件均保留。Linux快捷方式按登记内容一致性恢复，Windows快捷方式按目标/描述/参数归属检查；被用户改动的同名入口保留。

仅移除ResearchFlightGCS-WSL-UDP和ResearchFlightGCS-WSL-HyperV-UDP两条名称明确、来源本地、协议及端口匹配的规则；非本地映射或属性不符时保留并提示。依据微软Remove-NetFirewallHyperVRule和Remove-NetFirewallRule文档，调用始终指定Name，绝不执行无参数删除或修改全局防火墙策略。

保留WSL、Ubuntu、Linux用户、系统依赖、.wslconfig及安装/卸载日志，不重启系统或发行版，不提供注销发行版功能。失败可重跑，已有数据备份保留。新增卸载逻辑需要Windows现场验收，Linux及模拟Windows测试只验证对应范围。

安装脚本修订2026.11.10.7会将五份卸载运行文件复制到Windows用户安装日志目录的uninstall子目录，并创建开始菜单卸载入口，因此删除原解压包后仍可卸载。卸载运行文件与日志保留以支持失败重跑。

## 卸载入口修复（1.0.20261111.1）

旧入口与安装脚本均定义FunctionsOnly。卸载器dot-source安装器并传-FunctionsOnly后，这一变量在共享作用域中变为true；后续if检查直接return，导致退出码0、无卸载动作、无日志。原先函数模式测试未覆盖正常入口，此缺陷已用原始脚本真实子进程复现。

卸载入口改为UninstallFunctionsOnly，保留FunctionsOnly别名兼容已有测试；函数导入路径与正常运行路径明确分开。正常路径在平台检查及导入依赖前落盘并启动Transcript。CMD先写独立uninstall-launch日志，再检查/启动PowerShell，并显示最终退出码，避免早期失败无记录。开始菜单和新安装器复制的本机卸载脚本同步更新；使用旧版的用户直接解压新独立工具并运行其中CMD即可，无需重装。

新增完整入口控制流测试，临时副本仅替换平台/SID查询，卸载动作替换为临时轨迹记录；覆盖普通入口、函数导入、依赖导入失败及未改写入口的实际平台报错日志。该验证与模拟Windows API仍不替代Windows实机端到端验收。
