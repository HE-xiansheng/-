# 一键配置入口：在当前Windows用户下管理其WSL与桌面入口。
param([switch]$FunctionsOnly)
Set-StrictMode -Version 2.0
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Common.ps1')

function Invoke-WslCommand {
    param([string[]]$Arguments,[switch]$AllowFailure,[switch]$Visible)
    $process=[Diagnostics.Process]::new();$stdout=[IO.MemoryStream]::new();$stderr=[IO.MemoryStream]::new()
    try {
        $process.StartInfo.FileName=$script:WslPath
        $process.StartInfo.Arguments=Get-GcsWslArgumentLine $Arguments
        $process.StartInfo.UseShellExecute=$false;$process.StartInfo.CreateNoWindow=$true
        $process.StartInfo.RedirectStandardOutput=$true;$process.StartInfo.RedirectStandardError=$true
        [void]$process.Start()
        $outTask=$process.StandardOutput.BaseStream.CopyToAsync($stdout)
        $errTask=$process.StandardError.BaseStream.CopyToAsync($stderr)
        $timer=[Diagnostics.Stopwatch]::StartNew()
        while (-not $process.WaitForExit(500)) {
            if ($Visible) { Write-Progress -Activity '正在执行WSL任务' -Status ('已运行'+[int]$timer.Elapsed.TotalSeconds+'秒') }
        }
        [void]$outTask.GetAwaiter().GetResult();[void]$errTask.GetAwaiter().GetResult()
        if ($Visible) {Write-Progress -Activity '正在执行WSL任务' -Completed}
        $text=((ConvertFrom-GcsWslBytes ($stdout.ToArray()))+"`n"+(ConvertFrom-GcsWslBytes ($stderr.ToArray()))).Trim()
        $code=$process.ExitCode
        if ($Visible -and $text) {Write-Host $text}
        if (-not $AllowFailure -and $code -ne 0) {
            if ($text.Length -gt 2400) {$text=$text.Substring($text.Length-2400)}
            throw ("WSL命令未完成（退出码$code；参数："+$process.StartInfo.Arguments+"）：$text")
        }
        return [pscustomobject]@{ExitCode=$code;Text=$text}
    } finally {$process.Dispose();$stdout.Dispose();$stderr.Dispose()}

}

function Invoke-HostPhase {
    param([string]$Phase,[string]$StateDirectory)
    $resultFile=Join-Path $StateDirectory ('host-'+$Phase+'-'+[guid]::NewGuid().ToString('N')+'.json')
    $helper=Join-Path $PSScriptRoot 'Configure-Host.ps1'
    $nativeArgs=@('-NoProfile','-ExecutionPolicy','Bypass','-File',$helper,'-Phase',$Phase,'-ResultFile',$resultFile)
    $argumentLine=($nativeArgs | ForEach-Object { Quote-ProcessArgument $_ }) -join ' '
    $powershell=Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
    try { $child=Start-Process -FilePath $powershell -ArgumentList $argumentLine -Verb RunAs -Wait -PassThru }
    catch { throw '管理员步骤未执行：请允许Windows弹出的管理员授权。未修改其他账号的WSL。' }
    if (-not (Test-Path -LiteralPath $resultFile)) { throw '未收到管理员配置结果。' }
    $result=Get-Content -LiteralPath $resultFile -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($child.ExitCode -ne 0 -or -not $result.success) { throw ($result.reason+'；详细结果：'+$resultFile) }
    return $result
}

function Assert-WslExit {
    param($Result,[string]$Description)
    if ($Result.ExitCode -in @(3010,1641)) {
        Save-SetupState $script:StateDirectory 'RestartWindows' ($Description+'报告需要重启Windows')
        Write-Host ($Description+'报告需要重启Windows。请保存工作并自行重启，之后再次双击一键安装.cmd继续。') -ForegroundColor Yellow
        exit 3010
    }
    if ($Result.ExitCode -ne 0) { throw ($Description+'失败：'+$Result.Text) }
}

function Install-GcsUninstallFiles {
    param([string]$StateDirectory)
    $directory=Join-Path $StateDirectory 'uninstall'
    [IO.Directory]::CreateDirectory($directory) | Out-Null
    foreach($name in @('Common.ps1','Configure-Host.ps1','Setup-GroundStation.ps1','Uninstall-GroundStation.ps1','uninstall_in_wsl.py')) {
        Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $directory $name) -Force
    }
}

function New-GroundStationShortcut {
    param([string]$Distribution,[string]$LinuxUser,[string]$LinuxPrefix,[string]$StateDirectory)
    $icon=Join-Path $StateDirectory 'groundstation.ico'
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'groundstation.ico') -Destination $icon -Force
    $arguments=Get-GcsLaunchArguments $Distribution $LinuxUser $LinuxPrefix
    $shell=New-Object -ComObject WScript.Shell
    $folders=@([Environment]::GetFolderPath('Desktop'),(Join-Path ([Environment]::GetFolderPath('Programs')) '科研飞行平台'))
    foreach ($folder in $folders) {
        [IO.Directory]::CreateDirectory($folder) | Out-Null
        $path=Join-Path $folder '科研飞行平台-地面站（WSL）.lnk'
        $link=$shell.CreateShortcut($path);$link.TargetPath=$script:WslPath;$link.Arguments=$arguments
        $link.IconLocation=$icon;$link.WorkingDirectory=$StateDirectory;$link.WindowStyle=7
        $link.Description='通过WSL2启动科研飞行平台地面站';$link.Save()
    }
    Install-GcsUninstallFiles $StateDirectory
    $path=Join-Path $folders[1] '卸载科研飞行平台-地面站（WSL）.lnk'
    $link=$shell.CreateShortcut($path)
    $link.TargetPath=Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
    $link.Arguments=Get-GcsUninstallArguments $StateDirectory
    $link.WorkingDirectory=$StateDirectory;$link.IconLocation=$icon
    $link.Description='卸载WSL地面站并保留Ubuntu和用户数据';$link.Save()
}

if ($FunctionsOnly) { return }
$transcriptStarted=$false;$mutex=$null;$locked=$false
try {
    if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw '请在Windows11电脑上双击一键安装.cmd。' }
    if (-not [Environment]::Is64BitProcess -or $env:PROCESSOR_ARCHITECTURE -ne 'AMD64') { throw '此安装包需要x64版Windows11，请使用64位PowerShell。' }
    $build=[int](Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion').CurrentBuildNumber
    if ($build -lt 22621) { throw '镜像网络需要Windows11 22H2或更新版本；当前系统版本过低。' }
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $mutex=[Threading.Mutex]::new($false,('Local\ResearchFlightGCS-Wsl-Setup-'+$sid))
    try { $locked=$mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $locked=$true }
    if (-not $locked) { throw '已有一个地面站WSL安装进程，请等待它完成。' }
    [Console]::OutputEncoding=[Text.UTF8Encoding]::new($false)
    $OutputEncoding=[Text.UTF8Encoding]::new($false)
    $script:StateDirectory=Join-Path $env:LOCALAPPDATA 'ResearchFlightGCS-WSL-Setup'
    [IO.Directory]::CreateDirectory($script:StateDirectory) | Out-Null
    $log=Join-Path $script:StateDirectory ('setup-'+[DateTime]::Now.ToString('yyyyMMdd-HHmmss')+'.log')
    Start-Transcript -LiteralPath $log -Force | Out-Null;$transcriptStarted=$true
    Write-Host '科研飞行平台：WSL2配置与地面站安装' -ForegroundColor Cyan
    Write-Host '安装脚本修订：2026.11.11.1（修复卸载入口和提前记录日志）'
    Write-Host '配置网络时可能重启WSL，其中运行的程序会退出。不会自动重启Windows。'
    Write-Host "安装日志：$log"
    $payload=Assert-Payload $PSScriptRoot
    $ubuntuOffline=$null
    if ($payload.PSObject.Properties['ubuntu']) {$ubuntuOffline=Assert-UbuntuImage $PSScriptRoot}
    $wslOffline=$null
    if ($payload.PSObject.Properties['wsl']) { $wslOffline=Assert-WslInstaller $PSScriptRoot }
    Save-SetupState $script:StateDirectory 'Prerequisites'
    Write-Host '[1/6] 检查Windows组件（弹出管理员授权）'
    $features=Invoke-HostPhase 'Prerequisites' $script:StateDirectory
    if ($features.restartRequired) {
        Save-SetupState $script:StateDirectory 'RestartWindows' $features.reason
        Write-Host $features.reason -ForegroundColor Yellow
        Write-Host '请保存工作并自行重启Windows，再次双击一键安装.cmd即可继续。' -ForegroundColor Yellow
        exit 3010
    }
    $script:WslPath=Join-Path $env:SystemRoot 'System32\wsl.exe'
    if (-not (Test-Path -LiteralPath $script:WslPath)) { throw '未找到wsl.exe，请完成Windows组件安装后重试。' }
    Write-Host '[2/6] 检查WSL与Ubuntu20.04（优先使用包内文件）'
    Save-SetupState $script:StateDirectory 'Distribution'
    if ($wslOffline) {
        $installedVersion=Get-GcsInstalledWslVersion
        $probeWorks=$false
        if ($installedVersion) {
            $probe=Invoke-WslCommand -Arguments @('--version') -AllowFailure
            $probeWorks=$probe.ExitCode -eq 0
        }
        $action=Get-GcsWslRuntimeAction $installedVersion ([version]$wslOffline.version) $probeWorks
        if ($action -eq 'RefuseDowngrade') { throw '已有较新WSL版本但版本检查失败，未降级。请先检查现有WSL安装。' }
        if ($action -eq 'Reuse') { Write-Host "复用可用的WSL $installedVersion，不强制更新。" }
        else {
            Write-Host '从包内安装微软官方WSL（弹出管理员授权），失败时保留MSI详细日志。'
            Save-SetupState $script:StateDirectory 'WSLOfflineInstallation'
            $runtime=Invoke-HostPhase 'WSLRuntime' $script:StateDirectory
            Write-Host $runtime.reason
            if ($runtime.restartRequired) {
                Save-SetupState $script:StateDirectory 'RestartWindows' $runtime.reason
                Write-Host 'WSL安装器报告需要重启Windows。请保存工作、自行重启后再次双击继续。' -ForegroundColor Yellow
                exit 3010
            }
        }
        Invoke-WslCommand -Arguments @('--version') | Out-Null
    } else {
        $updated=Invoke-WslCommand -Arguments @('--update') -AllowFailure -Visible
        Assert-WslExit $updated 'WSL更新'
    }
    $listed=Invoke-WslCommand -Arguments @('--list','--quiet') -AllowFailure
    $distributions=ConvertFrom-WslList $listed.Text
    $distro='Ubuntu-20.04';$newMarker=Join-Path $script:StateDirectory 'created-Ubuntu-20.04.txt'
    if ($distributions -notcontains $distro) {
        if ($ubuntuOffline) {
            Write-Host '从包内导入Ubuntu，跳过GitHub发行版清单。'
            $root=Get-GcsImportDirectory (Join-Path $env:LOCALAPPDATA 'ResearchFlightGCS-WSL\Ubuntu-20.04')
            [IO.Directory]::CreateDirectory($root) | Out-Null
            $image=Join-Path $PSScriptRoot $ubuntuOffline.file
            $installed=Invoke-WslCommand -Arguments @('--import',$distro,$root,$image,'--version','2') -AllowFailure -Visible
            Assert-WslExit $installed 'Ubuntu本地导入'
        } else {
            $installed=Invoke-WslCommand -Arguments @('--install','--distribution',$distro,'--no-launch') -AllowFailure -Visible
            Assert-WslExit $installed 'Ubuntu安装'
        }
        [IO.File]::WriteAllText($newMarker,$distro,[Text.UTF8Encoding]::new($false))
    }
    $versions=Invoke-WslCommand -Arguments @('--list','--verbose')
    $distroVersion=Get-DistroVersionFromText $versions.Text $distro
    if ($distroVersion -eq 1) {
        Write-Host '将Ubuntu20.04转换为WSL2，此操作可能需要一些时间。'
        $converted=Invoke-WslCommand -Arguments @('--set-version',$distro,'2') -AllowFailure -Visible
        Assert-WslExit $converted 'WSL2转换'
    }
    Write-Host '[3/6] 配置镜像网络，保留原有其他设置'
    Save-SetupState $script:StateDirectory 'MirroredNetworking'
    $changed=Set-MirroredConfig (Join-Path $env:USERPROFILE '.wslconfig') $script:StateDirectory
    $mode=Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslinfo','--networking-mode') -AllowFailure
    if ($changed -or $mode.ExitCode -ne 0 -or $mode.Text.Trim() -ne 'mirrored') {
        Invoke-WslCommand -Arguments @('--shutdown') | Out-Null
        $mode=Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslinfo','--networking-mode') -AllowFailure
    }
    if ($mode.ExitCode -ne 0 -or $mode.Text.Trim() -ne 'mirrored') { throw ('镜像网络未生效，请检查Windows/WSL版本。当前结果：'+$mode.Text) }
    Write-Host '[4/6] 放行地面站UDP端口（弹出管理员授权）'
    $firewall=Invoke-HostPhase 'Firewall' $script:StateDirectory
    Write-Host $firewall.reason
    Save-SetupState $script:StateDirectory 'GroundStationInstall'
    Write-Host '[5/6] 安装图形、通信依赖与地面站；无需输入Ubuntu密码'
    $isNew=if (Test-Path -LiteralPath $newMarker) { '1' } else { '0' }
    if ($isNew -eq '1') { $linuxUser='gcs' }
    else {
        $defaultUser=(Invoke-WslCommand -Arguments @('-d',$distro,'--exec','id','-un')).Text.Trim()
        $linuxUser=if ($defaultUser -eq 'root') { 'gcs' } else { $defaultUser }
    }
    if ($linuxUser -notmatch '^[a-z_][a-z0-9_-]*\$?$') { throw "Linux用户名格式不支持：$linuxUser" }
    $bootstrap=(Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslpath','-a','-u',(Join-Path $PSScriptRoot 'install-in-wsl.sh'))).Text.Trim()
    $linuxPayload=(Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslpath','-a','-u',(Join-Path $PSScriptRoot $payload.file))).Text.Trim()
    $resultFile=Join-Path $script:StateDirectory 'linux-install-result.json'
    if (Test-Path -LiteralPath $resultFile) { Remove-Item -LiteralPath $resultFile }
    $linuxResult=(Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslpath','-a','-u',$resultFile)).Text.Trim()
    $arguments=@('-d',$distro,'-u','root','--exec','bash',$bootstrap,'--payload',$linuxPayload,
        '--sha256',$payload.sha256,'--binary-sha256',$payload.binary_sha256,'--version',$payload.version,
        '--user',$linuxUser,'--new-distro',$isNew,'--result-file',$linuxResult)
    if ($payload.PSObject.Properties['manifest_sha256']) {$arguments+=@('--manifest-sha256',$payload.manifest_sha256)}
    $upgradeHelper=(Invoke-WslCommand -Arguments @('-d',$distro,'-u','root','--exec','wslpath','-a','-u',(Join-Path $PSScriptRoot 'upgrade_prepare.py'))).Text.Trim()
    $arguments+=@('--upgrade-helper',$upgradeHelper)
    Invoke-WslCommand -Arguments $arguments -Visible | Out-Null
    if (-not (Test-Path -LiteralPath $resultFile)) { throw 'Ubuntu安装未返回结果，请查看日志。' }
    $linux=Get-Content -LiteralPath $resultFile -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not $linux.success -or $linux.user -ne $linuxUser -or $linux.prefix -notmatch '^/') { throw 'Ubuntu安装结果无效。' }
    Write-Host '[6/6] 创建Windows桌面与开始菜单入口'
    New-GroundStationShortcut $distro $linux.user $linux.prefix $script:StateDirectory
    $wayland=Invoke-WslCommand -Arguments @('-d',$distro,'-u',$linux.user,'--exec','printenv','WAYLAND_DISPLAY') -AllowFailure
    $display=Invoke-WslCommand -Arguments @('-d',$distro,'-u',$linux.user,'--exec','printenv','DISPLAY') -AllowFailure
    if ([string]::IsNullOrWhiteSpace($wayland.Text) -and [string]::IsNullOrWhiteSpace($display.Text)) {
        throw '地面站已安装，但未检测到WSLg图形显示。请更新Windows显卡驱动及WSL，然后重新运行本脚本检查。'
    }
    Save-SetupState $script:StateDirectory 'Complete' ('Ubuntu用户：'+$linux.user+'；目录：'+$linux.prefix)
    Write-Host '配置完成。双击桌面“科研飞行平台-地面站（WSL）”即可启动。' -ForegroundColor Green
    Write-Host '每次打开地面站后，请在机载通信输入密码并应用连接，再使用定位、任务和视频。安装过程不连接或控制无人机。'
    exit 0
} catch {
    Write-Host ('配置未完成：'+$_.Exception.Message) -ForegroundColor Red
    if (Get-Variable StateDirectory -Scope Script -ErrorAction SilentlyContinue) {
        Save-SetupState $script:StateDirectory 'Failed' $_.Exception.Message
        Write-Host "修复提示中的问题后再次双击即可继续。日志目录：$script:StateDirectory"
    }
    exit 1
} finally {
    if ($transcriptStarted) { Stop-Transcript | Out-Null }
    if ($locked -and $mutex) { $mutex.ReleaseMutex() }
    if ($mutex) { $mutex.Dispose() }
}
