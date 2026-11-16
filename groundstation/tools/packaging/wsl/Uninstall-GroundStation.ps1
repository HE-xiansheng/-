# 当前Windows用户卸载入口：保留WSL/Ubuntu、配置、日志和飞行数据。
param([Alias('FunctionsOnly')][switch]$UninstallFunctionsOnly)
Set-StrictMode -Version 2.0
$ErrorActionPreference='Stop'

function Test-GcsOwnedShortcut {
    param($Link,[string]$WslPath)
    if ($Link.TargetPath -ine $WslPath -or $Link.Description -cne '通过WSL2启动科研飞行平台地面站') {return $false}
    # 仅匹配本安装器的固定入口结构；同名但目标被用户修改的快捷方式保留。
    $user='[a-z_][a-z0-9_-]*\$?'
    $prefix='/[^"\r\n]+/\.local/opt/research-flight-groundstation(?:-wsl-[A-Za-z0-9._-]+)?/start-groundstation\.sh'
    $pattern='^-d Ubuntu-20\.04 -u '+$user+' --exec (?:'+$prefix+'|"'+$prefix+'") --live 9000 --pose-port 9001 --preflight-port 9003$'
    return [regex]::IsMatch([string]$Link.Arguments,$pattern)
}

function Remove-GcsWindowsShortcuts {
    param([string]$WslPath,[string]$StateDirectory)
    $shell=New-Object -ComObject WScript.Shell
    $folders=@([Environment]::GetFolderPath('Desktop'),(Join-Path ([Environment]::GetFolderPath('Programs')) '科研飞行平台'))
    foreach($folder in $folders) {
        $path=Join-Path $folder '科研飞行平台-地面站（WSL）.lnk'
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            $link=$shell.CreateShortcut($path)
            if (Test-GcsOwnedShortcut $link $WslPath) {Remove-Item -LiteralPath $path -Force}
            else {Write-Host ('快捷方式目标已改变，保留：'+$path)}
        }
    }
    $path=Join-Path $folders[1] '卸载科研飞行平台-地面站（WSL）.lnk'
    if (Test-Path -LiteralPath $path -PathType Leaf) {
        $link=$shell.CreateShortcut($path)
        $powershell=Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
        if ($link.TargetPath -ieq $powershell -and $link.Description -ceq '卸载WSL地面站并保留Ubuntu和用户数据' -and
            $link.Arguments -ceq (Get-GcsUninstallArguments $StateDirectory)) {Remove-Item -LiteralPath $path -Force}
    }
}

function Invoke-GcsLinuxUninstall {
    param([string]$StateDirectory)
    if (-not (Test-Path -LiteralPath $script:WslPath)) {Write-Host 'WSL未安装，跳过Linux程序卸载。';return}
    $listed=Invoke-WslCommand -Arguments @('--list','--quiet')
    if ((ConvertFrom-WslList $listed.Text) -notcontains 'Ubuntu-20.04') {
        Write-Host 'Ubuntu-20.04未注册，跳过Linux程序卸载。';return
    }
    $linuxUser=''
    $receipt=Join-Path $StateDirectory 'linux-install-result.json'
    if (Test-Path -LiteralPath $receipt) {
        $previous=Get-Content -LiteralPath $receipt -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($previous.PSObject.Properties['user']) {$linuxUser=[string]$previous.user}
    }
    if (-not $linuxUser) {
        $linuxUser=(Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','--exec','id','-un')).Text.Trim()
        if ($linuxUser -eq 'root') {$linuxUser='gcs'}
    }
    if ($linuxUser -notmatch '^[a-z_][a-z0-9_-]*\$?$' -or $linuxUser -eq 'root') {throw '安装用户记录无效，未删除Linux程序。'}
    $helper=(Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','-u','root','--exec','wslpath','-a','-u',
        (Join-Path $PSScriptRoot 'uninstall_in_wsl.py'))).Text.Trim()
    $result=Join-Path $StateDirectory ('linux-uninstall-'+[guid]::NewGuid().ToString('N')+'.json')
    $linuxResult=(Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','-u','root','--exec','wslpath','-a','-u',$result)).Text.Trim()
    Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','-u','root','--exec','python3',$helper,
        '--user',$linuxUser,'--result-file',$linuxResult) -Visible | Out-Null
    if (-not (Test-Path -LiteralPath $result)) {throw '未收到Linux卸载结果。'}
    $status=Get-Content -LiteralPath $result -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not $status.success) {throw 'Linux卸载未完成，保留Windows入口以便排查。'}
    foreach($item in $status.saved) {Write-Host ('用户文件已保留：'+$item.directory)}
}

# dot-source共享作用域，安装脚本的FunctionsOnly不能覆盖卸载入口开关。
if ($UninstallFunctionsOnly) {
    . (Join-Path $PSScriptRoot 'Setup-GroundStation.ps1') -FunctionsOnly
    return
}
$transcriptStarted=$false;$mutex=$null;$locked=$false
try {
    $localData=$env:LOCALAPPDATA
    if ([string]::IsNullOrWhiteSpace($localData)) {$localData=[Environment]::GetFolderPath('LocalApplicationData')}
    if ([string]::IsNullOrWhiteSpace($localData)) {throw '无法确定用户日志目录，请从一键卸载.cmd启动。'}
    $script:StateDirectory=Join-Path $localData 'ResearchFlightGCS-WSL-Setup'
    [IO.Directory]::CreateDirectory($script:StateDirectory) | Out-Null
    $log=Join-Path $script:StateDirectory ('uninstall-'+[DateTime]::Now.ToString('yyyyMMdd-HHmmss-fff')+'-'+$PID+'.log')
    # 导入依赖及检查运行环境之前先落盘，即使入口异常也有记录。
    [IO.File]::WriteAllText($log,"Uninstall entry started.`r`n",[Text.UTF8Encoding]::new($false))
    Start-Transcript -LiteralPath $log -Append -Force | Out-Null;$transcriptStarted=$true
    Write-Host ('卸载日志：'+$log)
    if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or -not [Environment]::Is64BitProcess) {throw '请在64位Windows上双击一键卸载.cmd。'}
    . (Join-Path $PSScriptRoot 'Setup-GroundStation.ps1') -FunctionsOnly
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $mutex=[Threading.Mutex]::new($false,('Local\ResearchFlightGCS-Wsl-Setup-'+$sid))
    try {$locked=$mutex.WaitOne(0)}catch [Threading.AbandonedMutexException] {$locked=$true}
    if (-not $locked) {throw '地面站安装或卸载正在进行，请等待它完成。'}
    [Console]::OutputEncoding=[Text.UTF8Encoding]::new($false)
    $OutputEncoding=[Text.UTF8Encoding]::new($false)
    Write-Host '科研飞行平台：卸载地面站，保留WSL、Ubuntu及用户数据。' -ForegroundColor Cyan
    $script:WslPath=Join-Path $env:SystemRoot 'System32\wsl.exe'
    Invoke-GcsLinuxUninstall $script:StateDirectory
    Remove-GcsWindowsShortcuts $script:WslPath $script:StateDirectory
    Write-Host '清理本产品专用防火墙规则（弹出管理员授权）。'
    $firewall=Invoke-HostPhase 'UninstallFirewall' $script:StateDirectory
    Write-Host $firewall.reason
    Save-SetupState $script:StateDirectory 'Uninstalled' '地面站已卸载；保留WSL、Ubuntu、日志和用户数据。'
    Write-Host '卸载完成。WSL、Ubuntu、飞行数据、日志、SSH配置与应用设置均保留。' -ForegroundColor Green
    Write-Host ('卸载日志：'+$log)
    exit 0
} catch {
    Write-Host ('卸载未全部完成：'+$_.Exception.Message) -ForegroundColor Red
    if (Get-Variable log -ErrorAction SilentlyContinue) {Write-Host ('详细日志：'+$log)}
    Write-Host '请按提示修复后重试；也可查看启动器显示的uninstall-launch日志。'
    exit 1
} finally {
    if ($transcriptStarted) {Stop-Transcript | Out-Null}
    if ($locked -and $mutex) {$mutex.ReleaseMutex()}
    if ($mutex) {$mutex.Dispose()}
}
