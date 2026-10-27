<#
.SYNOPSIS
在主机上验证外部芯片协议、断线及功率测试门禁；不连接板卡。
#>
param([string]$CompilerPath)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$outputDirectory = Join-Path $projectRoot 'tem/selftest_tests'
if ($CompilerPath) {
    $resolvedCompiler = (Resolve-Path -LiteralPath $CompilerPath -ErrorAction Stop).Path
    $compiler = Get-Command -Name $resolvedCompiler -CommandType Application -ErrorAction Stop | Select-Object -First 1
}
else {
    $compiler = Get-Command gcc -All -CommandType Application -ErrorAction Stop | Select-Object -First 1
}
Write-Host "GCC: $($compiler.Source)"
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
Push-Location $projectRoot
try {
    $executable = Join-Path $outputDirectory 'test_drivers.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror -Iproject-test/app -Iproject-test/devices/drv8353f -Iproject-test/devices/mt6835 tests/selftest/test_drivers.c project-test/devices/drv8353f/drv8353f.c project-test/devices/mt6835/mt6835.c project-test/app/test_logic.c -o $executable
    if ($LASTEXITCODE -ne 0) { throw '自检协议测试编译失败。' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw '自检协议或功率门禁测试失败。' }
    $appExecutable = Join-Path $outputDirectory 'test_app.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror -Itests/selftest/mock -Iproject-test/app -Iproject-test/board -Iproject-test/devices/drv8353f -Iproject-test/devices/mt6835 -Iproject-test/devices/ssd1306 tests/selftest/test_app.c project-test/devices/drv8353f/drv8353f.c project-test/devices/mt6835/mt6835.c project-test/devices/ssd1306/ssd1306.c project-test/app/test_logic.c -lm -o $appExecutable
    if ($LASTEXITCODE -ne 0) { throw '自检应用模拟编译失败。' }
    & $appExecutable
    if ($LASTEXITCODE -ne 0) { throw '自检应用故障注入失败。' }
    # 提取实际板卡函数供寄存器替身调用，保留源码中的全部前置检查和故障路径。
    $boardSource = Get-Content 'project-test/board/board_test.c' -Raw -Encoding UTF8
    $functions = foreach ($name in @('pulse_failed', 'board_gate_pulse', 'six_step_failed', 'board_six_step_pulse', 'board_watchdog_init', 'board_test_tick', 'board_load_start', 'board_load_running')) {
        $match = [regex]::Match($boardSource, '(?ms)^(?:static )?(?:bool|void)\r?\n' + $name + '\(.*?(?=^/\*\*|\z)')
        if (-not $match.Success) { throw "无法提取被测函数 $name，需核对测试入口。" }
        $match.Value
    }
    [System.IO.File]::WriteAllText((Join-Path $outputDirectory 'board_under_test.inc'), ($functions -join "`n"), [System.Text.UTF8Encoding]::new($false))
    $boardExecutable = Join-Path $outputDirectory 'test_board.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror "-I$outputDirectory" -Iproject-test/board tests/selftest/test_board.c -o $boardExecutable
    if ($LASTEXITCODE -ne 0) { throw '板卡寄存器模型测试编译失败。' }
    & $boardExecutable
    if ($LASTEXITCODE -ne 0) { throw '板卡单脉冲/看门狗回归失败。' }
    $uartFunctions = foreach ($name in @('board_uart_try_write', 'board_uart_get', 'board_uart_irq')) {
        $match = [regex]::Match($boardSource, '(?ms)^(?:bool|void)\r?\n' + $name + '\(.*?(?=^/\*\*|\z)')
        if (-not $match.Success) { throw "无法提取串口函数 $name。" }
        $match.Value
    }
    [System.IO.File]::WriteAllText((Join-Path $outputDirectory 'uart_under_test.inc'), ($uartFunctions -join "`n"), [System.Text.UTF8Encoding]::new($false))
    $uartExecutable = Join-Path $outputDirectory 'test_uart.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror "-I$outputDirectory" -Iproject-test/board tests/selftest/test_uart.c -o $uartExecutable
    if ($LASTEXITCODE -ne 0) { throw '串口负载诊断测试编译失败。' }
    & $uartExecutable
    if ($LASTEXITCODE -ne 0) { throw '串口非阻塞发送或急停回归失败。' }
}
finally { Pop-Location }
