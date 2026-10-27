<#
.SYNOPSIS
采集独立测试固件V1.8.1的串口原始日志，默认只查询；-Run显式执行一次电阻测试。
.DESCRIPTION
要求24.5欧姆电阻组接MOTA-MOTB、MOTC悬空、无电机。脚本不烧录、不选择端口、不会自动重复输出。
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^COM[1-9][0-9]*$')]
    [string]$Port,
    [switch]$Run
)
$ErrorActionPreference = 'Stop'
$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$captureDirectory = Join-Path $taskRoot 'tem/load_capture'
New-Item -ItemType Directory -Force -Path $captureDirectory | Out-Null
$capturePath = Join-Path $captureDirectory ((Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '-' + $Port + '.log')
$captureWriter = [System.IO.StreamWriter]::new($capturePath, $false, [System.Text.UTF8Encoding]::new($false))
$captureWriter.AutoFlush = $true
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.NewLine = "`n"
$serial.ReadTimeout = 200
$serial.WriteTimeout = 1000
$runRequested = $false
$runFinished = $false

# 原始接收行按顺序保留，不根据预期生成或改写设备数据。
function Receive-Until([string]$Pattern, [int]$TimeoutMs = 5000) {
    $watch = [System.Diagnostics.Stopwatch]::StartNew()
    while ($watch.ElapsedMilliseconds -lt $TimeoutMs) {
        try { $line = $serial.ReadLine().TrimEnd("`r") }
        catch [System.TimeoutException] { continue }
        $captureWriter.WriteLine($line)
        Write-Host $line
        if ($line -match '^LOAD RESULT VALID=0|LOAD (CHECK|RUN) BLOCKED|RX ERROR') {
            throw "设备拒绝或终止测试：$line"
        }
        if ($line -match $Pattern) { return $line }
    }
    throw "等待设备响应超时：$Pattern"
}

# 命令另以TX标记记录，不混淆为设备已确认的数据。
function Send-Line([string]$Command) {
    $captureWriter.WriteLine('TX> ' + $Command)
    $serial.WriteLine($Command)
}

try {
    $serial.Open()
    $captureWriter.WriteLine('HOST> ' + (Get-Date -Format o) + ' ' + $Port + ' 115200/8N1')
    Send-Line 'VERSION'
    $null = Receive-Until '^PCB SELFTEST V1\.8\.1 LOAD_R24500_GAIN40$'
    Send-Line 'PING'
    $null = Receive-Until '^PONG USART3 '
    Send-Line 'LOG OFF'
    Send-Line 'STATUS'
    $null = Receive-Until '^STATUS |^LOAD STATE='
    if ($Run) {
        Send-Line 'LOAD CHECK'
        $ready = Receive-Until '^LOAD STATE=READY '
        if ($ready -notmatch 'COOL_MS=0(?:\s|$)') { throw '仍在冷却；本轮不启动输出。' }
        # 接收到READY后的最后一个零点统计，避免响应交叉造成误判。
        $null = Receive-Until '^LOAD ZERO_C '
        $runRequested = $true
        Send-Line 'LOAD RUN'
        $null = Receive-Until '^LOAD RESULT VALID=1 '
        $runFinished = $true
    }
    else {
        Send-Line 'LOAD STATUS'
        $null = Receive-Until '^LOAD STATE='
    }
}
finally {
    if ($serial.IsOpen) {
        if ($runRequested -and -not $runFinished) {
            try { $serial.Write("!`n"); $captureWriter.WriteLine('TX> ! (host abort)') } catch { }
        }
        $serial.Close()
    }
    $serial.Dispose()
    $captureWriter.Dispose()
    Write-Host "原始日志：$capturePath"
}
