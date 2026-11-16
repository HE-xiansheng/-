param([Parameter(Mandatory=$true)][string]$SourceDirectory)
$ErrorActionPreference='Stop';Set-StrictMode -Version 2.0
. (Join-Path $SourceDirectory 'Uninstall-GroundStation.ps1') -FunctionsOnly
$script:checks=0
function Assert([bool]$Value,[string]$Message) {if(-not $Value){throw "FAIL $Message"};$script:checks++}
foreach($file in Get-ChildItem -LiteralPath $SourceDirectory -Filter '*.ps1') {
 $tokens=$null;$errors=$null
 [Management.Automation.Language.Parser]::ParseFile($file.FullName,[ref]$tokens,[ref]$errors) | Out-Null
 Assert ($errors.Count -eq 0) ('语法：'+$file.Name)
}
$wsl='C:\Windows\System32\wsl.exe'
foreach($prefix in @('/home/gcs/.local/opt/research-flight-groundstation','/home/test name/.local/opt/research-flight-groundstation-wsl-1.0.20261110.2')) {
 $link=[pscustomobject]@{TargetPath=$wsl;Description='通过WSL2启动科研飞行平台地面站';Arguments=(Get-GcsLaunchArguments 'Ubuntu-20.04' 'gcs' $prefix)}
 Assert (Test-GcsOwnedShortcut $link $wsl) '只识别本安装器入口（含空格路径）'
 $link.Arguments+=' --other';Assert (-not (Test-GcsOwnedShortcut $link $wsl)) '额外参数保留'
 $link.Arguments=Get-GcsLaunchArguments 'OtherUbuntu' 'gcs' $prefix;Assert (-not (Test-GcsOwnedShortcut $link $wsl)) '其他发行版保留'
 $link.Arguments=Get-GcsLaunchArguments 'Ubuntu-20.04' 'gcs' $prefix;$link.TargetPath='other.exe';Assert (-not (Test-GcsOwnedShortcut $link $wsl)) '不同目标保留'
}
$script:hyper=$null;$script:windows=$null;$script:removed=@()
function Get-NetFirewallHyperVRule {param($Name,$ErrorAction) return $script:hyper}
function Get-NetFirewallRule {param($Name,$ErrorAction) return $script:windows}
function Get-NetFirewallPortFilter {param($AssociatedNetFirewallRule,$ErrorAction) return [pscustomobject]@{Protocol='UDP';LocalPort=@('9000','9001','9005')}}
function Remove-NetFirewallHyperVRule {param($Name,$ErrorAction) Assert ($Name -ceq 'ResearchFlightGCS-WSL-HyperV-UDP') 'HyperV始终指定专用名称';$script:removed+=('HyperV:'+$Name)}
function Remove-NetFirewallRule {param($Name,$ErrorAction) Assert ($Name -ceq 'ResearchFlightGCS-WSL-UDP') 'Windows始终指定专用名称';$script:removed+=('Windows:'+$Name)}
Remove-GcsFirewallRules | Out-Null;Assert ($script:removed.Count -eq 0) '未安装可重复运行'
$script:hyper=[pscustomobject]@{PolicyStoreSourceType='Local';VMCreatorId='{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}';Protocol='UDP';LocalPorts=@('9000','9001','9005')}
$script:windows=[pscustomobject]@{PolicyStoreSourceType='Local'}
Remove-GcsFirewallRules | Out-Null;Assert ($script:removed.Count -eq 2) '只清理两条专用规则'
$script:removed=@();$script:hyper.PolicyStoreSourceType='HostFirewallLocal';$script:windows.PolicyStoreSourceType='GroupPolicy'
Remove-GcsFirewallRules | Out-Null;Assert ($script:removed.Count -eq 0) '保留映射和组策略规则'
$script:hyper.PolicyStoreSourceType='Local';$script:hyper.LocalPorts=@('80');$script:windows=$null
Remove-GcsFirewallRules | Out-Null;Assert ($script:removed.Count -eq 0) '同名其他端口保留'
$script:hyper.LocalPorts=@('9000','9001','9005');$script:hyper.VMCreatorId=[guid]::NewGuid().ToString()
Remove-GcsFirewallRules | Out-Null;Assert ($script:removed.Count -eq 0) '其他虚拟机规则保留'

$temp=Join-Path ([IO.Path]::GetTempPath()) ('gcs-uninstall-check-'+[guid]::NewGuid().ToString('N'))
[IO.Directory]::CreateDirectory($temp) | Out-Null
try {
 Install-GcsUninstallFiles $temp
 foreach($name in @('Common.ps1','Configure-Host.ps1','Setup-GroundStation.ps1','Uninstall-GroundStation.ps1','uninstall_in_wsl.py')) {
  Assert ((Get-FileHash -LiteralPath (Join-Path $temp ('uninstall/'+$name))).Hash -ceq (Get-FileHash -LiteralPath (Join-Path $SourceDirectory $name)).Hash) '卸载器保存到本机，删除解压目录后仍可使用'
 }
 Assert ((Get-GcsUninstallArguments $temp).Contains('Uninstall-GroundStation.ps1')) '开始菜单卸载入口指向本机副本'
 $script:WslPath=Join-Path $temp 'wsl-fixture';[IO.File]::WriteAllText($script:WslPath,'fixture')
 $script:calls=@();$script:listed='Ubuntu-20.04';$script:fail=$false
 function Invoke-WslCommand {
  param([string[]]$Arguments,[switch]$AllowFailure,[switch]$Visible)
  $script:calls+=,@($Arguments)
  $text=''
  if ($Arguments[0] -eq '--list') {$text=$script:listed}
  elseif ($Arguments -contains 'wslpath') {$text=$Arguments[-1]}
  elseif ($Arguments -contains 'id') {$text='root'}
  elseif ($Arguments -contains 'python3') {
   if ($script:fail) {throw 'fixture Linux failure'}
   [IO.File]::WriteAllText($Arguments[-1],'{"success":true,"removed":[],"saved":[]}')
  } else {throw ('unexpected command '+($Arguments -join ' '))}
  return [pscustomobject]@{ExitCode=0;Text=$text}
 }
 Invoke-GcsLinuxUninstall $temp
 $linux=@($script:calls | Where-Object {$_ -contains 'python3'})
 Assert ($linux.Count -eq 1 -and $linux[0][-3] -ceq 'gcs') '部分安装root默认回退gcs'
 foreach($call in $script:calls) {Assert (-not ($call -contains '--unregister') -and -not ($call -contains '--shutdown') -and -not ($call -contains '--install')) '卸载不注销或重启/安装发行版'}
 $script:listed='OtherUbuntu';$script:calls=@();Invoke-GcsLinuxUninstall $temp
 Assert ($script:calls.Count -eq 1) 'Ubuntu不存在时不执行Linux卸载'
 $script:listed='Ubuntu-20.04';$script:calls=@();$script:fail=$true
 $caught=$false;try {Invoke-GcsLinuxUninstall $temp}catch{$caught=$_.Exception.Message -match 'fixture Linux failure'}
 Assert $caught 'Linux失败不伪报成功'
 [IO.File]::WriteAllText((Join-Path $temp 'linux-install-result.json'),'{"user":"root"}')
 $caught=$false;try {Invoke-GcsLinuxUninstall $temp}catch{$caught=$_.Exception.Message -match '安装用户记录无效'}
 Assert $caught '拒绝root安装记录'
} finally {Remove-Item -LiteralPath $temp -Recurse -Force}
Write-Output (@{passed=$true;checks=$script:checks;scope='Uninstaller PowerShell parser, ownership and mocked APIs; no Windows changes'} | ConvertTo-Json -Compress)
