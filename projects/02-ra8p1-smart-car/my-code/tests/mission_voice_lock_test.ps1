$sourcePath = Join-Path $PSScriptRoot '..\software\mission_task.c'
$source = Get-Content -LiteralPath $sourcePath -Raw

$checks = [ordered]@{
    '定义语音 start 锁定状态' =
        $source -match 'static bool g_mission_voice_start_locked = false;'
    '语音 start 成功后进入锁定状态' =
        $source -match '(?s)if \(VOICE_AI_LABEL_START == label\).*?mission_request_start_impl\(\).*?g_mission_voice_start_locked = true;'
    '锁定状态仅允许 stop' =
        $source -match '(?s)g_mission_voice_start_locked.*?VOICE_AI_LABEL_STOP != label.*?return;'
    '退出 Mission 界面时解除锁定' =
        $source -match '(?s)mission_request_monitor_exited_impl\(void\).*?g_mission_voice_start_locked = false;'
}

$failed = $false
foreach ($check in $checks.GetEnumerator())
{
    if (-not $check.Value)
    {
        Write-Error "FAIL: $($check.Key)"
        $failed = $true
    }
}

if ($failed)
{
    exit 1
}

Write-Output 'PASS: Mission 语音 start 锁定规则完整'
