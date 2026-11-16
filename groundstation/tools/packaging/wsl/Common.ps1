# 与 Windows PowerShell 5.1 兼容的配置、参数和校验函数。
Set-StrictMode -Version 2.0

function Get-MirroredConfigText {
    param([AllowEmptyString()][string]$Text)
    $lines = [System.Collections.Generic.List[string]]::new()
    foreach ($line in [regex]::Split($Text, '\r?\n')) { $lines.Add($line) }
    if ($lines.Count -eq 1 -and $lines[0] -eq '') { $lines.Clear() }
    $inWsl = $false; $foundSection = $false; $written = $false
    $result = [System.Collections.Generic.List[string]]::new()
    foreach ($line in $lines) {
        if ($line -match '^\s*\[([^\]]+)\]\s*(?:[;#].*)?$') {
            if ($inWsl -and -not $written) { $result.Add('networkingMode=mirrored'); $written=$true }
            $inWsl = $Matches[1].Trim() -ieq 'wsl2'
            if ($inWsl) { $foundSection=$true }
        }
        if ($inWsl -and $line -match '^\s*networkingMode\s*=') {
            if (-not $written) { $result.Add('networkingMode=mirrored'); $written=$true }
            continue
        }
        $result.Add($line)
    }
    if (-not $foundSection) {
        if ($result.Count -gt 0 -and $result[$result.Count-1] -ne '') { $result.Add('') }
        $result.Add('[wsl2]'); $result.Add('networkingMode=mirrored')
    } elseif (-not $written) { $result.Add('networkingMode=mirrored') }
    return (($result -join "`r`n").TrimEnd([char[]]"`r`n") + "`r`n")
}

function Set-MirroredConfig {
    param([string]$Path, [string]$BackupDirectory)
    $before = if (Test-Path -LiteralPath $Path) { [IO.File]::ReadAllText($Path) } else { '' }
    $after = Get-MirroredConfigText $before
    if ($before -ceq $after) { return $false }
    [IO.Directory]::CreateDirectory($BackupDirectory) | Out-Null
    if (Test-Path -LiteralPath $Path) {
        $backup = Join-Path $BackupDirectory ('.wslconfig-' + [DateTime]::Now.ToString('yyyyMMdd-HHmmss-fff') + '.bak')
        [IO.File]::Copy($Path, $backup, $false)
        Write-Host "原有 WSL 配置已备份：$backup"
    }
    $temp = $Path + '.gcs-' + [guid]::NewGuid().ToString('N') + '.tmp'
    [IO.File]::WriteAllText($temp, $after, [Text.UTF8Encoding]::new($false))
    try {
        if (Test-Path -LiteralPath $Path) { [IO.File]::Replace($temp, $Path, $backup) }
        else { [IO.File]::Move($temp, $Path) }
    } finally { if (Test-Path -LiteralPath $temp) { Remove-Item -LiteralPath $temp -Force } }
    return $true
}

function Quote-ProcessArgument {
    param([AllowEmptyString()][string]$Value)
    if ($Value.IndexOf([char]0) -ge 0) { throw '命令参数包含非法空字符。' }
    $escaped = [regex]::Replace($Value, '(\\*)"', '$1$1\"')
    $escaped = [regex]::Replace($escaped, '(\\+)$', '$1$1')
    return '"' + $escaped + '"'
}

function Get-GcsWslArgumentLine {
    param([string[]]$Arguments)
    # WSL先按原始命令行识别选项，不会去除--version、-d、--exec外的引号。
    # 无空格/双引号的参数保持原样；含空格的路径才使用Windows参数转义。
    # 调用方直接创建进程，不经过cmd或PowerShell解释器。
    return (($Arguments | ForEach-Object {
        if ($_.IndexOf([char]0) -ge 0) { throw 'WSL参数包含非法空字符。' }
        if ($_ -ne '' -and $_ -notmatch '[\s"]') { $_ }
        else { Quote-ProcessArgument $_ }
    }) -join ' ')
}

function ConvertFrom-WslList {
    param([AllowEmptyString()][string]$Text)
    return ,@($Text.Replace([string][char]0, '').Trim([char]0xFEFF) -split '\r?\n' |
        ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' })
}

function Get-DistroVersionFromText {
    param([string]$Text,[string]$Distribution)
    $clean=$Text.Replace([string][char]0,'')
    $pattern='(?m)^\s*\*?\s*'+[regex]::Escape($Distribution)+'\s+.+?\s+([12])\s*$'
    $match=[regex]::Match($clean,$pattern)
    if (-not $match.Success) { throw "无法确定 $Distribution 的WSL版本，未尝试转换。" }
    return [int]$match.Groups[1].Value
}

function Assert-Payload {
    param([string]$Directory)
    $info=Get-Content -LiteralPath (Join-Path $Directory 'PayloadInfo.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($info.schema -ne 1 -or $info.sha256 -notmatch '^[a-f0-9]{64}$' -or
        $info.binary_sha256 -notmatch '^[a-f0-9]{64}$' -or $info.version -notmatch '^[a-zA-Z0-9._-]+$' -or
        [IO.Path]::GetFileName($info.file) -cne $info.file -or $info.file -match '[\\/]') {
        throw '安装文件清单格式无效。'
    }
    if ($info.PSObject.Properties['manifest_sha256'] -and $info.manifest_sha256 -notmatch '^[a-f0-9]{64}$') {throw '运行清单SHA256格式无效。'}
    $payload=Join-Path $Directory $info.file
    if (-not (Test-Path -LiteralPath $payload -PathType Leaf)) { throw "缺少安装文件：$payload，请完整解压ZIP。" }
    $actual=(Get-FileHash -LiteralPath $payload -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -cne $info.sha256) { throw '地面站安装文件SHA256校验失败，请重新复制完整安装包。' }
    return $info
}

function Save-SetupState {
    param([string]$Directory,[string]$Phase,[string]$Detail='')
    [IO.Directory]::CreateDirectory($Directory) | Out-Null
    $value=@{schema=1;phase=$Phase;detail=$Detail;updated=[DateTime]::Now.ToString('o')}
    [IO.File]::WriteAllText((Join-Path $Directory 'progress.json'),($value | ConvertTo-Json),[Text.UTF8Encoding]::new($false))
}

function Ensure-GcsWindowsFeatures {
    $restartRequired=$false; $reason=''
        $changed=@()
        foreach ($featureName in @('Microsoft-Windows-Subsystem-Linux','VirtualMachinePlatform')) {
            $feature=Get-WindowsOptionalFeature -Online -FeatureName $featureName
            if ([string]$feature.State -eq 'EnablePending') {
                $restartRequired=$true; $changed += $featureName; continue
            }
            if ([string]$feature.State -ne 'Enabled') {
                $enabled=Enable-WindowsOptionalFeature -Online -FeatureName $featureName -All -NoRestart
                if ($enabled.RestartNeeded) { $restartRequired=$true }
                $changed += $featureName
            }
        }
        if ($restartRequired) { $reason='Windows报告以下组件需要重启后生效：'+($changed -join '、') }
    return [pscustomobject]@{restartRequired=$restartRequired;reason=$reason}
}

function Test-GcsValueSet {
    param($Actual,[string[]]$Expected)
    $a=@($Actual | ForEach-Object { ([string]$_).Split(',') } | ForEach-Object { $_.Trim().ToLowerInvariant() } | Sort-Object -Unique)
    $b=@($Expected | ForEach-Object { $_.Trim().ToLowerInvariant() } | Sort-Object -Unique)
    return (($a -join '|') -ceq ($b -join '|'))
}

function Get-GcsNamedFirewallRule {
    param([ValidateSet('HyperV','Windows')][string]$Layer,[string]$Name)
    try {
        if ($Layer -eq 'HyperV') { return (Get-NetFirewallHyperVRule -Name $Name -ErrorAction Stop) }
        return (Get-NetFirewallRule -Name $Name -ErrorAction Stop)
    } catch {
        # 仅“没有此规则”按首次创建处理；权限等其他错误必须保留，不能当成不存在。
        if ($_.CategoryInfo.Category -eq 'ObjectNotFound' -or $_.FullyQualifiedErrorId -like 'CmdletizationQuery_NotFound*') {return}
        throw
    }
}

function Ensure-GcsFirewallRules {
    $step='检查Hyper-V防火墙命令';$ruleName='ResearchFlightGCS-WSL-UDP'
    $hyperName='ResearchFlightGCS-WSL-HyperV-UDP'
    $vmId='{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}';$ports=@('9000','9001','9005')
    $reused=0;$created=0;$updated=0;$script:GcsFirewallFailure=$null
    try {
        if (-not (Get-Command New-NetFirewallHyperVRule -ErrorAction SilentlyContinue)) {
            throw '系统缺少Hyper-V防火墙命令；需要Windows11 22H2或更高版本，并完成WSL组件安装。'
        }
        $step='读取Hyper-V规则（Get-NetFirewallHyperVRule）'
        $hyper=@(Get-GcsNamedFirewallRule 'HyperV' $hyperName | Where-Object {$null -ne $_})
        if ($hyper.Count -gt 1) {throw '同名Hyper-V规则不唯一，未修改。'}
        if ($hyper.Count -eq 0) {
            $step='创建Hyper-V规则（New-NetFirewallHyperVRule）'
            New-NetFirewallHyperVRule -Name $hyperName -DisplayName '科研飞行平台WSL虚拟网络UDP' `
                -Direction Inbound -Action Allow -Enabled True -VMCreatorId $vmId -Protocol UDP -LocalPorts $ports -ErrorAction Stop | Out-Null
            $created++
        } else {
            $h=$hyper[0]
            # 只操作本地Hyper-V规则；Windows映射、MDM等规则不可当作本工具可写规则。
            $step='核对Hyper-V规则来源（PolicyStoreSourceType）'
            $source=[string]$h.PolicyStoreSourceType
            if ($source -notin @('Local','1')) {
                throw "规则$hyperName 来源为$source，未修改非本地Hyper-V规则。"
            }
            $sameCreator=$false
            try {$sameCreator=([guid]$h.VMCreatorId -eq [guid]$vmId)}catch{}
            $correct=([string]$h.Enabled -eq 'True' -and [string]$h.Direction -eq 'Inbound' -and [string]$h.Action -eq 'Allow' -and
                $sameCreator -and [string]$h.Protocol -in @('UDP','17') -and (Test-GcsValueSet $h.LocalPorts $ports))
            if ($correct) { $reused++ }
            else {
                $step='更新Hyper-V规则（Set-NetFirewallHyperVRule）'
                Set-NetFirewallHyperVRule -Name $hyperName -Direction Inbound -Action Allow -Enabled True `
                    -VMCreatorId $vmId -Protocol UDP -LocalPorts $ports -ErrorAction Stop | Out-Null
                $updated++
            }
        }
        $step='读取Windows防火墙规则（Get-NetFirewallRule）'
        $hostRules=@(Get-GcsNamedFirewallRule 'Windows' $ruleName | Where-Object {$null -ne $_})
        if ($hostRules.Count -gt 1) {throw '同名Windows规则不唯一，未修改。'}
        if ($hostRules.Count -eq 0) {
            $step='创建Windows防火墙规则（New-NetFirewallRule）'
            New-NetFirewallRule -Name $ruleName -DisplayName '科研飞行平台WSL遥测UDP' `
                -Direction Inbound -Action Allow -Enabled True -Protocol UDP -LocalPort $ports `
                -RemoteAddress LocalSubnet -Profile Any -ErrorAction Stop | Out-Null
            $created++
        } else {
            $h=$hostRules[0]
            $step='读取Windows端口过滤器（Get-NetFirewallPortFilter）'
            $port=Get-NetFirewallPortFilter -AssociatedNetFirewallRule $h -ErrorAction Stop
            $step='读取Windows地址过滤器（Get-NetFirewallAddressFilter）'
            $address=Get-NetFirewallAddressFilter -AssociatedNetFirewallRule $h -ErrorAction Stop
            $correct=([string]$h.Enabled -eq 'True' -and [string]$h.Direction -eq 'Inbound' -and [string]$h.Action -eq 'Allow' -and
                [string]$h.Profile -eq 'Any' -and [string]$port.Protocol -in @('UDP','17') -and
                (Test-GcsValueSet $port.LocalPort $ports) -and (Test-GcsValueSet $address.RemoteAddress @('LocalSubnet')))
            if ($correct) { $reused++ }
            else {
                $step='更新Windows防火墙规则（Set-NetFirewallRule）'
                Set-NetFirewallRule -Name $ruleName -Direction Inbound -Action Allow -Enabled True -Protocol UDP `
                    -LocalPort $ports -RemoteAddress LocalSubnet -Profile Any -ErrorAction Stop | Out-Null
                $updated++
            }
        }
        return "UDP9000、9001、9005规则检查完成：复用$reused 条、新建$created 条、更新$updated 条；未关闭防火墙。"
    } catch {
        $script:GcsFirewallFailure=@{step=$step;errorId=$_.FullyQualifiedErrorId;category=[string]$_.CategoryInfo.Category;
            position=$_.InvocationInfo.PositionMessage;exception=$_.Exception.GetType().FullName;hresult=$_.Exception.HResult}
        throw ("防火墙步骤失败：$step；"+$_.Exception.Message)
    }
}

function Remove-GcsFirewallRules {
    # 始终明确指定名称；不碰Windows映射、组策略或其他程序的规则。
    $ports=@('9000','9001','9005');$removed=0;$kept=0
    $hyperName='ResearchFlightGCS-WSL-HyperV-UDP'
    if (Get-Command Get-NetFirewallHyperVRule -ErrorAction SilentlyContinue) {
        $hyper=@(Get-GcsNamedFirewallRule 'HyperV' $hyperName | Where-Object {$null -ne $_})
        if ($hyper.Count -gt 1) {throw '同名Hyper-V规则不唯一，未自动删除。'}
        foreach($rule in $hyper) {
            $creator=$false
            try {$creator=([guid]$rule.VMCreatorId -eq [guid]'{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}')}catch{}
            if ([string]$rule.PolicyStoreSourceType -in @('Local','1') -and $creator -and
                [string]$rule.Protocol -in @('UDP','17') -and (Test-GcsValueSet $rule.LocalPorts $ports)) {
                Remove-NetFirewallHyperVRule -Name $hyperName -ErrorAction Stop | Out-Null
                $removed++
            } else {$kept++}
        }
    }
    if (Get-Command Get-NetFirewallRule -ErrorAction SilentlyContinue) {
        $name='ResearchFlightGCS-WSL-UDP'
        $rules=@(Get-GcsNamedFirewallRule 'Windows' $name | Where-Object {$null -ne $_})
        if ($rules.Count -gt 1) {throw '同名Windows规则不唯一，未自动删除。'}
        foreach($rule in $rules) {
            $filter=Get-NetFirewallPortFilter -AssociatedNetFirewallRule $rule -ErrorAction Stop
            if ([string]$rule.PolicyStoreSourceType -in @('Local','1') -and
                [string]$filter.Protocol -in @('UDP','17') -and (Test-GcsValueSet $filter.LocalPort $ports)) {
                Remove-NetFirewallRule -Name $name -ErrorAction Stop | Out-Null
                $removed++
            } else {$kept++}
        }
    }
    return "专用防火墙规则已删除$removed 条；非本地或属性不符的同名规则保留$kept 条。"
}

function Get-GcsLaunchArguments {
    param([string]$Distribution,[string]$LinuxUser,[string]$LinuxPrefix)
    $values=@('-d',$Distribution,'-u',$LinuxUser,'--exec',($LinuxPrefix+'/start-groundstation.sh'),
              '--live','9000','--pose-port','9001','--preflight-port','9003')
    return (Get-GcsWslArgumentLine $values)
}

function Get-GcsUninstallArguments {
    param([string]$StateDirectory)
    $entry=Join-Path $StateDirectory 'uninstall/Uninstall-GroundStation.ps1'
    return ('-NoLogo -NoProfile -ExecutionPolicy Bypass -File '+(Quote-ProcessArgument $entry))
}


function Assert-WslInstaller {
    param([string]$Directory)
    $info=Get-Content -LiteralPath (Join-Path $Directory 'PayloadInfo.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not $info.PSObject.Properties['wsl']) { throw '安装包没有内置WSL离线安装文件。' }
    $wsl=$info.wsl
    if ([IO.Path]::GetFileName($wsl.file) -cne $wsl.file -or $wsl.file -match '[\\/]' -or
        $wsl.file -notmatch '\.msi$' -or $wsl.sha256 -notmatch '^[a-f0-9]{64}$') { throw 'WSL离线安装清单无效。' }
    $version=[version]$wsl.version
    $file=Join-Path $Directory $wsl.file
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw '缺少WSL离线MSI，请完整解压安装包。' }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant() -cne $wsl.sha256) {
        throw 'WSL离线MSI校验失败，请重新复制完整安装包。'
    }
    return $wsl
}

function Get-GcsInstalledWslVersion {
    $versions=[Collections.Generic.List[version]]::new()
    # 只读注册信息；不使用Win32_Product，避免触发其他MSI自修复。
    foreach($root in @('HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall',
                       'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall')) {
        foreach($item in @(Get-ItemProperty ($root+'\*') -ErrorAction SilentlyContinue)) {
            if ($item.PSObject.Properties['DisplayName'] -and $item.DisplayName -match '^(Windows Subsystem for Linux|适用于\s*Linux\s*的\s*Windows\s*子系统)$' -and
                $item.PSObject.Properties['DisplayVersion']) {
                try{$versions.Add([version]$item.DisplayVersion)}catch{}
            }
        }
    }
    if (Get-Command Get-AppxPackage -ErrorAction SilentlyContinue) {
        foreach($app in @(Get-AppxPackage -Name 'MicrosoftCorporationII.WindowsSubsystemForLinux' -ErrorAction SilentlyContinue)) {
            try{$versions.Add([version]$app.Version)}catch{}
        }
    }
    if ($versions.Count -eq 0) { return $null }
    return ($versions | Sort-Object -Descending | Select-Object -First 1)
}

function Get-GcsWslRuntimeAction {
    param([AllowNull()]$InstalledVersion,[version]$BundledVersion,[bool]$VersionProbeWorks)
    if ($InstalledVersion -and [version]$InstalledVersion -ge [version]'2.0.0.0' -and $VersionProbeWorks) { return 'Reuse' }
    if ($InstalledVersion -and [version]$InstalledVersion -gt $BundledVersion) { return 'RefuseDowngrade' }
    return 'InstallOffline'
}


function ConvertFrom-GcsWslBytes {
    param([AllowEmptyCollection()][byte[]]$Bytes)
    if ($Bytes.Length -eq 0) { return '' }
    if ($Bytes.Length -ge 2 -and $Bytes[0] -eq 255 -and $Bytes[1] -eq 254) {
        return [Text.Encoding]::Unicode.GetString($Bytes,2,$Bytes.Length-2)
    }
    if ($Bytes.Length -ge 2 -and $Bytes[0] -eq 254 -and $Bytes[1] -eq 255) {
        return [Text.Encoding]::BigEndianUnicode.GetString($Bytes,2,$Bytes.Length-2)
    }
    if ($Bytes.Length -ge 3 -and $Bytes[0] -eq 239 -and $Bytes[1] -eq 187 -and $Bytes[2] -eq 191) {
        return [Text.Encoding]::UTF8.GetString($Bytes,3,$Bytes.Length-3)
    }
    # WSL原生命令常输出无BOM的UTF-16LE；Linux --exec输出通常为UTF-8。
    $oddZero=0;$pairs=[int][Math]::Floor($Bytes.Length/2)
    for($i=1;$i -lt $Bytes.Length;$i+=2){if($Bytes[$i] -eq 0){$oddZero++}}
    if ($pairs -gt 0 -and $oddZero -gt $pairs*0.15) { return [Text.Encoding]::Unicode.GetString($Bytes) }
    try { return [Text.UTF8Encoding]::new($false,$true).GetString($Bytes) }
    catch {
        if ($Bytes.Length%2 -eq 0) { return [Text.Encoding]::Unicode.GetString($Bytes) }
        return [Text.Encoding]::Default.GetString($Bytes)
    }
}

function Assert-UbuntuImage {
    param([string]$Directory)
    $info=Get-Content -LiteralPath (Join-Path $Directory 'PayloadInfo.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not $info.PSObject.Properties['ubuntu']) { throw '缺少Ubuntu本地镜像清单。' }
    $image=$info.ubuntu
    if ([IO.Path]::GetFileName($image.file) -cne $image.file -or $image.file -match '[\\/]' -or
        $image.sha256 -notmatch '^[a-f0-9]{64}$') { throw 'Ubuntu镜像清单无效。' }
    $file=Join-Path $Directory $image.file
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw '缺少Ubuntu本地镜像，请完整解压安装包。' }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant() -cne $image.sha256) {
        throw 'Ubuntu本地镜像SHA256校验失败，未导入。'
    }
    return $image
}

function Get-GcsImportDirectory {
    param([string]$Root)
    if (-not (Test-Path -LiteralPath $Root)) { return $Root }
    if (@(Get-ChildItem -LiteralPath $Root -Force).Count -eq 0) { return $Root }
    # 不清理任何残留磁盘；目录非空时选择新目录，保留原有文件。
    return $Root+'-'+[DateTime]::Now.ToString('yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,6)
}
