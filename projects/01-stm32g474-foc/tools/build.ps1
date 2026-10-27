<#
.SYNOPSIS
自动查找 Keil 并编译当前工作区的最小工程。
.PARAMETER KeilPath
可选的本机 UV4.exe 路径；未指定时依次检查环境变量、PATH 和桌面快捷方式。
.PARAMETER ProjectDirectory
选择开发工程 project 或焊接后测试工程 project-test，默认 project。
.PARAMETER Target
正式工程默认 foc_dual，对应已确认的 DBANK=1；测试工程默认 foc。
#>
param(
    [string]$KeilPath,
    [ValidateSet('project', 'project-test')]
    [string]$ProjectDirectory = 'project',
    [ValidateSet('foc','foc_dual')]
    [string]$Target = $(if ($ProjectDirectory -eq 'project-test') { 'foc' } else { 'foc_dual' })
)

$ErrorActionPreference = 'Stop'

# 根据脚本位置定位工程，使工作区可以跨电脑移动。
$projectRoot = Split-Path -Parent $PSScriptRoot
$projectFile = Join-Path $projectRoot "$ProjectDirectory/MDK-ARM/foc.uvprojx"
$keilExecutable = $KeilPath

if (-not $keilExecutable) {
    $keilExecutable = $env:KEIL_UVISION
}
if (-not $keilExecutable) {
    $keilCommand = Get-Command UV4.exe -CommandType Application -ErrorAction SilentlyContinue
    if ($keilCommand) {
        $keilExecutable = ($keilCommand | Select-Object -First 1).Source
    }
}
if (-not $keilExecutable) {
    # 只读取快捷方式目标，不修改快捷方式或本机安装配置。
    $desktopDirectory = [Environment]::GetFolderPath('Desktop')
    $shortcutReader = New-Object -ComObject WScript.Shell
    $shortcutTargets = @(Get-ChildItem -LiteralPath $desktopDirectory -Filter '*Keil*.lnk' -File |
        ForEach-Object { $shortcutReader.CreateShortcut($_.FullName).TargetPath } |
        Where-Object { (Split-Path -Leaf $_) -eq 'UV4.exe' -and (Test-Path -LiteralPath $_ -PathType Leaf) } |
        Sort-Object -Unique)
    if ($shortcutTargets.Count -gt 1) {
        throw '检测到多个 Keil 安装，请用 -KeilPath 指定本次使用的 UV4.exe。'
    }
    if ($shortcutTargets.Count -eq 1) {
        $keilExecutable = $shortcutTargets[0]
    }
}
if (-not $keilExecutable -or -not (Test-Path -LiteralPath $keilExecutable -PathType Leaf)) {
    throw '未找到有效的 UV4.exe。请设置本机 KEIL_UVISION 环境变量，或使用 -KeilPath 指定路径。'
}
if (-not (Test-Path -LiteralPath $projectFile -PathType Leaf)) {
    throw "缺少 $ProjectDirectory/MDK-ARM/foc.uvprojx，请检查工作区目录。"
}

# 本机 µVision 5.43.1 读取带 UTF-8 BOM 的工程时无法进入构建，先明确报错。
$projectBytes = [IO.File]::ReadAllBytes($projectFile)
if ($projectBytes.Length -ge 3 -and $projectBytes[0] -eq 0xEF -and
    $projectBytes[1] -eq 0xBB -and $projectBytes[2] -eq 0xBF) {
    throw 'Keil 工程文件带有 UTF-8 BOM，请将 .uvprojx 保存为 UTF-8 无 BOM 后重试。'
}

# 安装路径仅在运行时使用，不写回工程；不启动烧录或调试。
Write-Output "Keil: $keilExecutable"
Write-Output "Project: $projectFile"
$objectDirectory = if ($Target -eq 'foc_dual') { 'Objects_dual' } else { 'Objects' }
$buildLog = Join-Path $projectRoot "$ProjectDirectory/MDK-ARM/$objectDirectory/keil-build.log"
New-Item -ItemType Directory -Path (Split-Path -Parent $buildLog) -Force | Out-Null
$previousLogWriteTime = if (Test-Path -LiteralPath $buildLog) { (Get-Item -LiteralPath $buildLog).LastWriteTimeUtc } else { $null }
$buildArguments = '-b "' + $projectFile + '" -t ' + $Target + ' -o "' + $buildLog + '"'
$buildProcess = Start-Process -FilePath $keilExecutable -WorkingDirectory $projectRoot -ArgumentList $buildArguments -WindowStyle Hidden -PassThru
if (-not $buildProcess.WaitForExit(60000)) {
    # 只结束本脚本启动的构建进程，不影响用户打开的其他 Keil 窗口。
    $buildProcess.Kill()
    $buildProcess.WaitForExit()
    throw 'Keil 构建超过 60 秒，已结束本次构建。请检查工程是否能正常加载；旧日志不能作为本次结果。'
}
Write-Output "BuildExitCode=$($buildProcess.ExitCode)"
if (-not (Test-Path -LiteralPath $buildLog) -or
    (Get-Item -LiteralPath $buildLog).LastWriteTimeUtc -eq $previousLogWriteTime) {
    throw 'Keil 没有生成新的构建日志，不能确认本次构建结果。'
}
$buildOutput = Get-Content -LiteralPath $buildLog -Raw
Write-Output $buildOutput
if ($buildProcess.ExitCode -eq 0 -and $buildOutput -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw 'Keil 退出但日志未确认 0 错误、0 警告，请检查上述输出。'
}
exit $buildProcess.ExitCode
