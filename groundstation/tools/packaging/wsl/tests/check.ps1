param([Parameter(Mandatory=$true)][string]$SourceDirectory)
$ErrorActionPreference='Stop';Set-StrictMode -Version 2.0
. (Join-Path $SourceDirectory 'Common.ps1')
$script:checks=0
function Assert([bool]$Value,[string]$Message) { if(-not $Value){throw "FAIL: $Message"};$script:checks++ }
foreach($file in Get-ChildItem -LiteralPath $SourceDirectory -Filter '*.ps1') {
 $tokens=$null;$errors=$null
 [Management.Automation.Language.Parser]::ParseFile($file.FullName,[ref]$tokens,[ref]$errors) | Out-Null
 Assert ($errors.Count -eq 0) ("PowerShell语法："+$file.Name)
}
$cases=@(
 @{name='空配置';text='';contains=@('[wsl2]','networkingMode=mirrored')},
 @{name='保留内存和其他段';text=";my config`r`n[wsl2]`r`nmemory=4GB`r`nprocessors=2`r`n[experimental]`r`nautoMemoryReclaim=gradual`r`n";contains=@('memory=4GB','processors=2','[experimental]','autoMemoryReclaim=gradual')},
 @{name='只替换WSL2的网络键';text="[other]`nnetworkingMode=nat`n[WSL2]`nNetworkingMode = NAT # old`nswap=2GB`n";contains=@('[other]',"networkingMode=nat",'networkingMode=mirrored','swap=2GB')},
 @{name='保留注释';text="[wsl2]`n# networkingMode=nat`n; kept`nnetworkingMode=nat`nnetworkingMode=virtioproxy`n";contains=@('# networkingMode=nat','; kept')},
 @{name='添加到末尾段';text="[experimental]`nsparseVhd=true";contains=@('sparseVhd=true','[wsl2]')}
)
foreach($case in $cases) {
 $new=Get-MirroredConfigText $case.text
 foreach($item in $case.contains){Assert ($new.Contains($item)) ($case.name+': '+$item)}
 Assert ((Get-MirroredConfigText $new) -ceq $new) ($case.name+'重复运行不改变配置')
 Assert (([regex]::Matches($new,'(?im)^networkingMode=mirrored\r?$')).Count -eq 1) ($case.name+'无重复镜像键')
}
Assert ((Quote-ProcessArgument 'C:\Chinese 中文\') -ceq '"C:\Chinese 中文\\"') 'Windows路径末尾反斜杠'
Assert ((Quote-ProcessArgument 'a"b') -ceq '"a\"b"') 'Windows参数内部双引号'
Assert ((Quote-ProcessArgument '') -ceq '""') '空参数'
Assert ((Quote-ProcessArgument '$(secret); & echo') -ceq '"$(secret); & echo"') '参数不作为PowerShell代码执行'
$launch=Get-GcsLaunchArguments 'Ubuntu-20.04' 'gcs' '/home/gcs/a path'
Assert ($launch.Contains('--preflight-port 9003')) '桌面入口启用实时功能栏'
Assert ($launch.Contains('"/home/gcs/a path/start-groundstation.sh"')) 'Linux目录空格作为一个启动参数'
Assert ($launch.Contains('-u gcs')) '桌面入口使用普通用户'

# WSL3.0.5在原始命令行上比较选项，给--version加引号会落入Linux命令分支。
Assert ((Get-GcsWslArgumentLine @('--version')) -ceq '--version') '版本检查不添加WSL无法识别的引号'
Assert ((Get-GcsWslArgumentLine @('--list','--quiet')) -ceq '--list --quiet') '发行版列表不误启动Linux'
Assert ((Get-GcsWslArgumentLine @('--shutdown')) -ceq '--shutdown') '网络重启使用真正的WSL管理参数'
Assert ($launch.StartsWith('-d Ubuntu-20.04 -u gcs --exec ')) '桌面入口发行版、用户和exec不包含引号'
$import=Get-GcsWslArgumentLine @('--import','Ubuntu-20.04','C:\Users\尚\a path','C:\包\ubuntu.wsl','--version','2')
Assert ($import -ceq '--import Ubuntu-20.04 "C:\Users\尚\a path" C:\包\ubuntu.wsl --version 2') '本地导入仅为空格路径加引号'
$path=Get-GcsWslArgumentLine @('-d','Ubuntu-20.04','-u','root','--exec','wslpath','-a','-u','C:\Users\尚\文件 夹\install.sh')
Assert ($path -ceq '-d Ubuntu-20.04 -u root --exec wslpath -a -u "C:\Users\尚\文件 夹\install.sh"') 'WSL原生选项与Linux参数边界正确'
Assert ((Get-GcsWslArgumentLine @('--exec','printf','')) -ceq '--exec printf ""') 'Linux空参数保留'
$bad=$false;try{Get-GcsWslArgumentLine @('--exec',("a"+[char]0+"b"))|Out-Null}catch{$bad=$true}
Assert $bad 'WSL命令拒绝空字符'

$distro='Ubuntu-20.04'
foreach($text in @("  NAME STATE VERSION`r`n* Ubuntu-20.04 Running 2`r`n", "名称 状态 版本`n  Ubuntu-20.04 已停止 2`n", "* Ubuntu-20.04 Stopped 1")){
 $version=Get-DistroVersionFromText $text $distro
 Assert ($version -eq $(if($text.EndsWith('1')){1}else{2})) '不同语言的发行版版本读取'
}
Assert ((ConvertFrom-WslList ("U"+[char]0+"buntu-20.04"+[char]0+"`r`n")).Count -eq 1) '清理WSL列表空字符'
Assert ((ConvertFrom-WslList '').Count -eq 0) '空发行版列表'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('gcs-wsl-test-'+[guid]::NewGuid().ToString('N'));[IO.Directory]::CreateDirectory($temp)|Out-Null
try {
 $fresh=Join-Path $temp 'new.wslconfig'
 Assert (Set-MirroredConfig $fresh (Join-Path $temp 'unused-backup')) '首次创建WSL配置文件'
 Assert (-not (Set-MirroredConfig $fresh (Join-Path $temp 'unused-backup'))) '新配置重复运行幂等'
 $config=Join-Path $temp '.wslconfig';$backup=Join-Path $temp 'backup'
 [IO.File]::WriteAllText($config,"[wsl2]`r`nmemory=6GB`r`nnetworkingMode=nat`r`n")
 Assert (Set-MirroredConfig $config $backup) '配置第一次更新'
 Assert (-not (Set-MirroredConfig $config $backup)) '配置第二次无需重启'
 Assert (@(Get-ChildItem -LiteralPath $backup -Force).Count -eq 1) '只生成一份原配置备份'
 Assert ((Get-Content -LiteralPath (Get-ChildItem $backup -Force)[0].FullName -Raw).Contains('networkingMode=nat')) '备份保留原始内容'
 $payload=Join-Path $temp '中文 payload.run';[IO.File]::WriteAllText($payload,'fixture')
 $hash=(Get-FileHash -LiteralPath $payload -Algorithm SHA256).Hash.ToLowerInvariant()
 @{schema=1;file='中文 payload.run';sha256=$hash;binary_sha256=('a'*64);version='1.0.20261110'}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $temp 'PayloadInfo.json') -Encoding utf8
 Assert ((Assert-Payload $temp).version -eq '1.0.20261110') '有效安装包与中文路径校验'
 [IO.File]::AppendAllText($payload,'tampered');$rejected=$false
 try{Assert-Payload $temp|Out-Null}catch{$rejected=$true}
 Assert $rejected '篡改安装包被拒绝'
 Save-SetupState $temp 'RestartWindows' '测试组件明确要求重启'
 $state=Get-Content -LiteralPath (Join-Path $temp 'progress.json') -Raw|ConvertFrom-Json
 Assert ($state.phase -eq 'RestartWindows') '进度保留以便重启后再次运行'
} finally { Remove-Item -LiteralPath $temp -Recurse -Force }
# 模拟Windows组件，检查已启用时不会要求重启。
$script:featureState='Enabled';$script:restart=$false;$script:enabledCalls=0
# 使用switch模拟真实cmdlet。
function Get-WindowsOptionalFeature {param([switch]$Online,$FeatureName) return [pscustomobject]@{State=$script:featureState} }
function Enable-WindowsOptionalFeature {param([switch]$Online,$FeatureName,[switch]$All,[switch]$NoRestart) $script:enabledCalls++;return [pscustomobject]@{RestartNeeded=$script:restart} }
$r=Ensure-GcsWindowsFeatures
Assert (-not $r.restartRequired -and $script:enabledCalls -eq 0) '已有WSL组件不要求重启Windows'
$script:featureState='Disabled';$script:restart=$true
$r=Ensure-GcsWindowsFeatures
Assert ($r.restartRequired -and $script:enabledCalls -eq 2) '仅系统报告RestartNeeded时提示Windows重启'
$script:featureState='Disabled';$script:restart=$false;$script:enabledCalls=0
$r=Ensure-GcsWindowsFeatures
Assert (-not $r.restartRequired -and $script:enabledCalls -eq 2) '系统未报告RestartNeeded时不要求重启Windows'
$script:featureState='EnablePending';$script:enabledCalls=0
$r=Ensure-GcsWindowsFeatures
Assert ($r.restartRequired -and $script:enabledCalls -eq 0) '待重启组件不反复启用'
# 模拟防火墙：首次创建、重跑只读复用、不删除规则、权限错误不被吞掉。
$script:rules=[Collections.Generic.List[object]]::new();$script:sets=[Collections.Generic.List[object]]::new()
$script:hyperRule=$null;$script:hostRule=$null;$script:mirroredRule=$null;$script:firewallDenied=''
function Get-NetFirewallHyperVRule {
 param($Name,$ErrorAction)
 if($script:firewallDenied -eq 'readHyper'){throw [UnauthorizedAccessException]::new('拒绝访问（读Hyper-V）')}
 if($script:mirroredRule -and $script:mirroredRule.Name -eq $Name){return $script:mirroredRule}
 if($script:hyperRule -and $script:hyperRule.Name -eq $Name){return $script:hyperRule}
}
function Get-NetFirewallRule {param($Name,$ErrorAction) if($script:hostRule){return $script:hostRule} }
function Remove-NetFirewallHyperVRule {throw '禁止删除已有Hyper-V规则'}
function Remove-NetFirewallRule {throw '禁止删除已有Windows规则'}
function New-NetFirewallHyperVRule {
 param($Name,$DisplayName,$Direction,$Action,$Enabled,$VMCreatorId,$Protocol,$LocalPorts,$ErrorAction)
 $script:rules.Add($PSBoundParameters)
 $script:hyperRule=[pscustomobject]@{Name=$Name;Enabled=$Enabled;Direction=$Direction;Action=$Action;VMCreatorId=$VMCreatorId;Protocol=$Protocol;LocalPorts=$LocalPorts;PolicyStoreSourceType='Local'}
}
function New-NetFirewallRule {
 param($Name,$DisplayName,$Direction,$Action,$Enabled,$Protocol,$LocalPort,$RemoteAddress,$Profile,$ErrorAction)
 $script:rules.Add($PSBoundParameters)
 $script:hostRule=[pscustomobject]@{Name=$Name;Enabled=$Enabled;Direction=$Direction;Action=$Action;Protocol=$Protocol;LocalPort=$LocalPort;RemoteAddress=$RemoteAddress;Profile=$Profile}
}
function Get-NetFirewallPortFilter {param($AssociatedNetFirewallRule,$ErrorAction) return [pscustomobject]@{Protocol=$AssociatedNetFirewallRule.Protocol;LocalPort=$AssociatedNetFirewallRule.LocalPort} }
function Get-NetFirewallAddressFilter {param($AssociatedNetFirewallRule,$ErrorAction) return [pscustomobject]@{RemoteAddress=$AssociatedNetFirewallRule.RemoteAddress} }
function Set-NetFirewallHyperVRule {
 param($Name,$Direction,$Action,$Enabled,$VMCreatorId,$Protocol,$LocalPorts,$ErrorAction)
 if($script:firewallDenied -eq 'setHyper'){throw [UnauthorizedAccessException]::new('拒绝访问（更新Hyper-V）')}
 $script:sets.Add($PSBoundParameters)
 foreach($key in @('Enabled','Direction','Action','VMCreatorId','Protocol','LocalPorts')){$script:hyperRule.$key=$PSBoundParameters[$key]}
}
function Set-NetFirewallRule {
 param($Name,$Direction,$Action,$Enabled,$Protocol,$LocalPort,$RemoteAddress,$Profile,$ErrorAction)
 $script:sets.Add($PSBoundParameters)
 foreach($key in @('Enabled','Direction','Action','Protocol','LocalPort','RemoteAddress','Profile')){$script:hostRule.$key=$PSBoundParameters[$key]}
}
$first=Ensure-GcsFirewallRules
Assert ($script:rules.Count -eq 2 -and $first.Contains('新建2')) '首次创建Windows与Hyper-V两层规则'
foreach($r in $script:rules){
 Assert ($r.Name -in @('ResearchFlightGCS-WSL-HyperV-UDP','ResearchFlightGCS-WSL-UDP')) '只创建自己的规则'
 Assert ($r.Direction -eq 'Inbound' -and $r.Action -eq 'Allow' -and $r.Protocol -eq 'UDP') '只放行UDP入站'
}
Assert (($script:rules[0].LocalPorts -join ',') -eq '9000,9001,9005') 'Hyper-V只开放三个遥测端口'
Assert (($script:rules[1].LocalPort -join ',') -eq '9000,9001,9005' -and $script:rules[1].RemoteAddress -eq 'LocalSubnet') 'Windows规则限制本地子网'
$again=Ensure-GcsFirewallRules
Assert ($script:rules.Count -eq 2 -and $script:sets.Count -eq 0 -and $again.Contains('复用2')) '重跑正确规则不删除、不创建、不更新'
$script:hyperRule.LocalPorts=@('9005','9000','9001');$script:hyperRule.Protocol='17'
$script:hostRule.LocalPort='9005,9001,9000';$script:hostRule.Protocol='17'
Ensure-GcsFirewallRules|Out-Null
Assert ($script:sets.Count -eq 0) '规则端口顺序与UDP数字形式不引起不必要更新'
$script:hostRule.Enabled='False';$script:hostRule.RemoteAddress='Any';$script:hyperRule.LocalPorts=@('9000')
$fixed=Ensure-GcsFirewallRules
Assert ($script:sets.Count -eq 2 -and $script:rules.Count -eq 2 -and $fixed.Contains('更新2')) '错误的自身规则原位更新、不删除重建'
foreach($r in $script:sets){Assert ($r.Name -in @('ResearchFlightGCS-WSL-HyperV-UDP','ResearchFlightGCS-WSL-UDP')) '只更新自己的规则'}
Assert ($script:hostRule.Enabled -eq 'True' -and $script:hostRule.RemoteAddress -eq 'LocalSubnet') 'Windows规则恢复启用且限制子网'
Assert ((Test-GcsValueSet $script:hyperRule.LocalPorts @('9000','9001','9005'))) 'Hyper-V更新仍限定三个端口'
$script:firewallDenied='readHyper';$denied=''
try{Ensure-GcsFirewallRules|Out-Null}catch{$denied=$_.Exception.Message}
Assert ($denied.Contains('Get-NetFirewallHyperVRule') -and $denied.Contains('拒绝访问')) '读取权限拒绝保留具体阶段'
Assert ($script:rules.Count -eq 2 -and $script:sets.Count -eq 2) '读取失败不被当作规则缺失，不执行写操作'
Assert ($script:GcsFirewallFailure.exception -eq 'System.UnauthorizedAccessException') '诊断保留原始异常类型'
$script:firewallDenied='setHyper';$script:hyperRule.LocalPorts=@('1');$denied=''
try{Ensure-GcsFirewallRules|Out-Null}catch{$denied=$_.Exception.Message}
Assert ($denied.Contains('Set-NetFirewallHyperVRule') -and $denied.Contains('拒绝访问')) '更新权限拒绝保留具体阶段'
Assert ($script:rules.Count -eq 2) '更新拒绝后不会尝试删除重建'
$script:firewallDenied='';$script:hyperRule.LocalPorts=@('9000','9001','9005')
# 按用户实机截图构造：旧同名规则来自Windows映射，VMCreatorId为Any。
$script:hyperRule=$null
$script:mirroredRule=[pscustomobject]@{Name='ResearchFlightGCS-WSL-UDP';Enabled='True';Direction='Inbound';Action='Allow';VMCreatorId='Any';Protocol='UDP';LocalPorts=@('9000','9001','9005');PolicyStoreSourceType='HostFirewallLocal'}
$beforeNew=$script:rules.Count;$beforeSet=$script:sets.Count
$actualCase=Ensure-GcsFirewallRules
Assert ($script:rules.Count -eq $beforeNew+1 -and $script:sets.Count -eq $beforeSet) '实机映射场景创建独立Hyper-V规则，不尝试更新映射'
Assert ($script:hyperRule.Name -eq 'ResearchFlightGCS-WSL-HyperV-UDP' -and $script:hostRule.Name -eq 'ResearchFlightGCS-WSL-UDP') 'Windows与Hyper-V使用不同规则名'
Assert ($script:hyperRule.VMCreatorId -eq '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}') '新Hyper-V规则只针对WSL'
Assert ($script:mirroredRule.VMCreatorId -eq 'Any' -and $script:mirroredRule.PolicyStoreSourceType -eq 'HostFirewallLocal') '旧Windows映射未修改'
Ensure-GcsFirewallRules|Out-Null
Assert ($script:rules.Count -eq $beforeNew+1 -and $script:sets.Count -eq $beforeSet) '修复后重跑两层规则都复用'
$script:hyperRule.VMCreatorId='40e0ac32-46a5-438a-a0b2-2b479e8f2e90';$script:hyperRule.PolicyStoreSourceType='1'
Ensure-GcsFirewallRules|Out-Null
Assert ($script:sets.Count -eq $beforeSet) 'WSL GUID大小写/括号及本地来源数字形式不触发误更新'
$script:hyperRule.PolicyStoreSourceType='HostFirewallLocal';$denied=''
try{Ensure-GcsFirewallRules|Out-Null}catch{$denied=$_.Exception.Message}
Assert ($denied.Contains('PolicyStoreSourceType') -and $denied.Contains('HostFirewallLocal')) '即使新名称发生映射碰撞，也拒绝修改非本地规则'
Assert ($script:sets.Count -eq $beforeSet -and $script:rules.Count -eq $beforeNew+1) '来源不正确时不尝试写入或绕过'
$script:hyperRule.PolicyStoreSourceType='MDM';$denied=''
try{Ensure-GcsFirewallRules|Out-Null}catch{$denied=$_.Exception.Message}
Assert ($denied.Contains('MDM') -and $script:sets.Count -eq $beforeSet) '不修改MDM规则'
$script:hyperRule.PolicyStoreSourceType='Local'
# Windows常见“无此规则”错误必须可识别为首次创建。
function Test-NotFoundQuery {
 $record=[Management.Automation.ErrorRecord]::new([Exception]::new('rule missing'),'CmdletizationQuery_NotFound_Name',[Management.Automation.ErrorCategory]::ObjectNotFound,$null)
 throw $record
}
function Get-NetFirewallRule {param($Name,$ErrorAction) Test-NotFoundQuery }
$none=@(Get-GcsNamedFirewallRule 'Windows' 'missing')
Assert ($none.Count -eq 0) '仅ObjectNotFound被当作缺少规则'
# 离线WSL安装决策不降级、不强制更新正常环境。
Assert ((Get-GcsWslRuntimeAction ([version]'2.6.0.0') ([version]'3.0.5.0') $true) -eq 'Reuse') '可用旧版WSL不强制更新'
Assert ((Get-GcsWslRuntimeAction ([version]'3.1.0.0') ([version]'3.0.5.0') $true) -eq 'Reuse') '可用新版WSL不降级'
Assert ((Get-GcsWslRuntimeAction ([version]'3.1.0.0') ([version]'3.0.5.0') $false) -eq 'RefuseDowngrade') '损坏的较新版本也不自动降级'
Assert ((Get-GcsWslRuntimeAction $null ([version]'3.0.5.0') $false) -eq 'InstallOffline') '未安装时使用本地MSI'
Assert ((Get-GcsWslRuntimeAction ([version]'1.2.0.0') ([version]'3.0.5.0') $true) -eq 'InstallOffline') '不支持镜像网络的旧运行时使用MSI升级'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('gcs-msi-test-'+[guid]::NewGuid().ToString('N'));[IO.Directory]::CreateDirectory($temp)|Out-Null
try {
 $msi=Join-Path $temp 'wsl.msi';[IO.File]::WriteAllText($msi,'isolated MSI fixture')
 $sha=(Get-FileHash -LiteralPath $msi -Algorithm SHA256).Hash.ToLowerInvariant()
 @{schema=1;wsl=@{file='wsl.msi';version='3.0.5.0';sha256=$sha}}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $temp 'PayloadInfo.json') -Encoding utf8
 Assert ((Assert-WslInstaller $temp).version -eq '3.0.5.0') '离线MSI完整性校验'
 [IO.File]::AppendAllText($msi,'tamper');$bad=$false
 try {Assert-WslInstaller $temp | Out-Null}catch{$bad=$true}
 Assert $bad '篡改的MSI被拒绝'
} finally {Remove-Item -LiteralPath $temp -Recurse -Force}

# 用户实机回归：原生WSL输出UTF-16LE，而Linux进程输出UTF-8。
$native='安装发行版失败：无法下载 https://raw.githubusercontent.com/microsoft/WSL/master/distributions/DistributionInfo.json Wsl/InstallDistro/WININET_E_TIMEOUT'
$linux='安装路径：/home/gcs/科研 平台，完成 ✓'
foreach($enc in @([Text.Encoding]::Unicode,[Text.Encoding]::BigEndianUnicode,[Text.UTF8Encoding]::new($true))) {
 $bytes=[byte[]]($enc.GetPreamble()+$enc.GetBytes($native))
 Assert ((ConvertFrom-GcsWslBytes $bytes) -ceq $native) '带BOM的中文WSL输出'
}
Assert ((ConvertFrom-GcsWslBytes ([Text.Encoding]::Unicode.GetBytes($native))) -ceq $native) '无BOM的WSL超时日志保留中文与错误码'
Assert ((ConvertFrom-GcsWslBytes ([Text.Encoding]::UTF8.GetBytes($linux))) -ceq $linux) 'Linux UTF-8中文路径不被误解码'
Assert ((ConvertFrom-GcsWslBytes ([byte[]]@())) -ceq '') '空输出不产生乱码'
$temp=Join-Path ([IO.Path]::GetTempPath()) ('gcs-ubuntu-test-'+[guid]::NewGuid().ToString('N'));[IO.Directory]::CreateDirectory($temp)|Out-Null
try {
 $image=Join-Path $temp 'ubuntu.wsl';[IO.File]::WriteAllText($image,'isolated Ubuntu fixture')
 $sha=(Get-FileHash -LiteralPath $image -Algorithm SHA256).Hash.ToLowerInvariant()
 @{schema=1;ubuntu=@{file='ubuntu.wsl';version='20.04.6';sha256=$sha}}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $temp 'PayloadInfo.json') -Encoding utf8
 Assert ((Assert-UbuntuImage $temp).version -eq '20.04.6') '本地Ubuntu镜像完整性校验'
 [IO.File]::AppendAllText($image,'tamper');$bad=$false
 try {Assert-UbuntuImage $temp|Out-Null}catch{$bad=$true}
 Assert $bad '损坏的Ubuntu镜像不导入'
 @{schema=1;ubuntu=@{file='../ubuntu.wsl';version='20.04.6';sha256=$sha}}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $temp 'PayloadInfo.json') -Encoding utf8
 $bad=$false;try {Assert-UbuntuImage $temp|Out-Null}catch{$bad=$true}
 Assert $bad 'Ubuntu清单拒绝跨目录路径'
 $root=Join-Path $temp 'Ubuntu-20.04'
 Assert ((Get-GcsImportDirectory $root) -ceq $root) '新发行版使用固定导入目录'
 [IO.Directory]::CreateDirectory($root)|Out-Null
 Assert ((Get-GcsImportDirectory $root) -ceq $root) '空目录可以用于导入'
 $disk=Join-Path $root 'ext4.vhdx';[IO.File]::WriteAllText($disk,'existing data')
 $next=Get-GcsImportDirectory $root
 Assert ($next -ne $root -and $next.StartsWith($root+'-')) '非空目录不复用，以新目录导入'
 Assert (([IO.File]::ReadAllText($disk)) -ceq 'existing data') '原有磁盘和数据完全保留'
} finally {Remove-Item -LiteralPath $temp -Recurse -Force}
# 真正启动子进程，验证原始字节捕获（不运行WSL、不修改系统）。
. (Join-Path $SourceDirectory 'Setup-GroundStation.ps1') -FunctionsOnly
$script:WslPath=[Diagnostics.Process]::GetCurrentProcess().MainModule.FileName
$child=@'
$o=[Console]::OpenStandardOutput();$e=[Console]::OpenStandardError()
$b=[Text.Encoding]::Unicode.GetBytes('中文原生命令 Wsl/InstallDistro/WININET_E_TIMEOUT')
$o.Write($b,0,$b.Length)
$b=[Text.Encoding]::UTF8.GetBytes('Linux错误：路径 /home/gcs/科研 平台')
$e.Write($b,0,$b.Length)
exit 7
'@
$encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($child))
$argsNative=@('-NoProfile','-NonInteractive','-EncodedCommand',$encoded)
$result=Invoke-WslCommand -Arguments $argsNative -AllowFailure
Assert ($result -is [pscustomobject] -and $result.ExitCode -eq 7) '真实子进程返回退出码且不混入异步任务输出'
Assert ($result.Text.Contains('中文原生命令') -and $result.Text.Contains('路径 /home/gcs/科研 平台')) 'stdout/stderr分别解码中文'
$failed=$false;try{Invoke-WslCommand -Arguments $argsNative|Out-Null}catch{$failed=$_.Exception.Message.Contains('中文原生命令')}
Assert $failed '真实子进程失败保留可读原因'
$child=@'
$o=[Console]::OpenStandardOutput();$e=[Console]::OpenStandardError()
$b=[Text.Encoding]::UTF8.GetBytes(('a'*131072));$o.Write($b,0,$b.Length)
$b=[Text.Encoding]::UTF8.GetBytes(('b'*131072));$e.Write($b,0,$b.Length)
'@
$encoded=[Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($child))
$result=Invoke-WslCommand -Arguments @('-NoProfile','-NonInteractive','-EncodedCommand',$encoded)
Assert ($result.Text.Length -eq 262145 -and $result.ExitCode -eq 0) '双输出管道超过缓冲区时仍完整读取且不死锁'

Write-Output (@{passed=$true;checks=$script:checks;scope='PowerShell parser and mocked Windows APIs; no real Windows changes'}|ConvertTo-Json -Compress)
