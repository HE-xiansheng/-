# 只采集本机Windows/WSL状态及本地桥接日志，不调用远端或修改防火墙。
param([string]$OutputDirectory='')
Set-StrictMode -Version 2.0
$ErrorActionPreference='Stop'
try {
    . (Join-Path $PSScriptRoot 'Setup-GroundStation.ps1') -FunctionsOnly
    $script:WslPath=Join-Path $env:SystemRoot 'System32\wsl.exe'
    $report=@{schema=1;time=[DateTime]::Now.ToString('o');kind='read-only WSL ground station diagnostics'}
    try {
        $endpoints=@(Get-NetUDPEndpoint -ErrorAction Stop | Where-Object {$_.LocalPort -in @(9000,9001,9003,9004,9005,19003)})
        $report.windowsUdp=@($endpoints | ForEach-Object {
            $endpoint=$_;$name='unknown'
            try {$name=(Get-Process -Id $endpoint.OwningProcess -ErrorAction Stop).ProcessName}catch{}
            @{address=$endpoint.LocalAddress;port=$endpoint.LocalPort;pid=$endpoint.OwningProcess;process=$name}
        })
    } catch {$report.windowsUdpError=$_.Exception.Message}
    foreach($version in @('ipv4','ipv6')) {
        try {$report['excludedUdp'+$version]=(& (Join-Path $env:SystemRoot 'System32\netsh.exe') interface $version show excludedportrange protocol=udp 2>&1 | Out-String)}
        catch {$report['excludedUdp'+$version]=$_.Exception.Message}
    }
    $listed=Invoke-WslCommand -Arguments @('--list','--verbose') -AllowFailure
    $report.distributions=$listed.Text
    $linuxPath=(Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','-u','root','--exec','wslpath','-a','-u',(Join-Path $PSScriptRoot 'collect_linux.py'))).Text.Trim()
    $linux=Invoke-WslCommand -Arguments @('-d','Ubuntu-20.04','-u','root','--exec','python3',$linuxPath) -AllowFailure
    $report.linuxExitCode=$linux.ExitCode
    # 保留原始内容，包括WSL启动警告，以免JSON解析掩盖故障。
    $report.linuxText=$linux.Text
    $desktop=if($OutputDirectory){$OutputDirectory}else{[Environment]::GetFolderPath('Desktop')}
    [IO.Directory]::CreateDirectory($desktop)|Out-Null
    $output=Join-Path $desktop ('地面站诊断-'+[DateTime]::Now.ToString('yyyyMMdd-HHmmss')+'.json')
    [IO.File]::WriteAllText($output,($report|ConvertTo-Json -Depth 12),[Text.UTF8Encoding]::new($false))
    Write-Host ('诊断完成，文件保存在桌面：'+$output) -ForegroundColor Green
    Write-Host '请将这个JSON文件发回分析。没有重启服务、清理缓存或发送飞行控制指令。'
} catch {
    Write-Host ('诊断未完成：'+$_.Exception.Message) -ForegroundColor Red
    exit 1
}
