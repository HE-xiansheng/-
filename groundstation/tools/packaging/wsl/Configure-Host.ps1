# 仅在管理员子进程中处理Windows组件和本工具防火墙规则；不注册Ubuntu。
param(
    [Parameter(Mandatory=$true)][ValidateSet('Prerequisites','Firewall','WSLRuntime','UninstallFirewall')][string]$Phase,
    [Parameter(Mandatory=$true)][string]$ResultFile
)
Set-StrictMode -Version 2.0
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Common.ps1')
$result=@{success=$false;restartRequired=$false;reason='';phase=$Phase}
try {
    $identity=[Security.Principal.WindowsIdentity]::GetCurrent()
    $principal=[Security.Principal.WindowsPrincipal]::new($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { throw '需要Windows管理员权限。' }
    if ($Phase -eq 'Prerequisites') {
        $features=Ensure-GcsWindowsFeatures
        $result.restartRequired=$features.restartRequired; $result.reason=$features.reason
    } elseif ($Phase -eq 'WSLRuntime') {
        $wsl=Assert-WslInstaller $PSScriptRoot
        $msi=Join-Path $PSScriptRoot $wsl.file
        $signature=Get-AuthenticodeSignature -LiteralPath $msi
        if ($signature.Status -ne 'Valid' -or -not $signature.SignerCertificate -or
            $signature.SignerCertificate.Subject -notmatch 'O=Microsoft Corporation(?:,|$)') {
            throw 'WSL安装文件的微软数字签名未通过Windows验证，未执行安装。'
        }
        $directory=Split-Path -Parent $ResultFile
        $log=Join-Path $directory ('wsl-msi-'+[DateTime]::Now.ToString('yyyyMMdd-HHmmss-fff')+'.log')
        $result.log=$log
        $argsLine='/i '+(Quote-ProcessArgument $msi)+' /passive /norestart /L*V '+(Quote-ProcessArgument $log)
        $process=Start-Process -FilePath (Join-Path $env:SystemRoot 'System32\msiexec.exe') -ArgumentList $argsLine -Wait -PassThru
        $result.exitCode=$process.ExitCode
        if ($process.ExitCode -in @(3010,1641)) { $result.restartRequired=$true }
        elseif ($process.ExitCode -ne 0) {
            $detail=if($process.ExitCode -eq 1618){'另一个Windows安装任务正在运行，请等待其完成。'}else{'Windows安装器未完成WSL安装。'}
            throw ($detail+' MSI退出码：'+$process.ExitCode+'；详细日志：'+$log)
        }
        $result.reason='已运行微软官方WSL离线安装程序。日志：'+$log
    } elseif ($Phase -eq 'UninstallFirewall') {
        $result.reason=Remove-GcsFirewallRules
    } else {
        $result.reason=Ensure-GcsFirewallRules
    }
    $result.success=$true
} catch {
    $result.reason=$_.Exception.Message
    $result.errorId=$_.FullyQualifiedErrorId
    $result.errorCategory=[string]$_.CategoryInfo.Category
    $result.errorPosition=$_.InvocationInfo.PositionMessage
    $result.errorStack=$_.ScriptStackTrace
    if (Get-Variable GcsFirewallFailure -Scope Script -ErrorAction SilentlyContinue) {
        $result.firewallFailure=$script:GcsFirewallFailure
    }
}
finally {
    [IO.File]::WriteAllText($ResultFile,($result | ConvertTo-Json),[Text.UTF8Encoding]::new($false))
}
if (-not $result.success) { exit 1 }
exit 0
