<#
.SYNOPSIS
在主机上验证 I2C 故障处理和 SSD1306 数据边界，不连接板卡。
#>
param([string]$CompilerPath)

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$outputDirectory = Join-Path $projectRoot 'tem/display_tests'
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
    $i2cExecutable = Join-Path $outputDirectory 'test_i2c.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror -Itests/display/mock -Iproject-test/board tests/display/test_i2c.c project-test/board/board_i2c.c -o $i2cExecutable
    if ($LASTEXITCODE -ne 0) { throw 'I2C 主机测试编译失败。' }
    & $i2cExecutable
    if ($LASTEXITCODE -ne 0) { throw 'I2C 故障注入测试失败。' }

    $oledExecutable = Join-Path $outputDirectory 'test_ssd1306.exe'
    & $compiler.Source -std=c11 -Wall -Wextra -Werror -Iproject-test/devices/ssd1306 tests/display/test_ssd1306.c project-test/devices/ssd1306/ssd1306.c -o $oledExecutable
    if ($LASTEXITCODE -ne 0) { throw 'SSD1306 主机测试编译失败。' }
    & $oledExecutable
    if ($LASTEXITCODE -ne 0) { throw 'SSD1306 边界测试失败。' }
}
finally {
    Pop-Location
}
