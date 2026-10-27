param([string]$Python = "")

$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
$environmentPython = Join-Path $projectRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $environmentPython)) {
    if ($Python) {
        if (-not (Test-Path -LiteralPath $Python)) { throw "指定 Python 路径不存在。" }
        & $Python -m venv (Join-Path $projectRoot ".venv")
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
        & py -3.12 -m venv (Join-Path $projectRoot ".venv")
    } else {
        $detectedPython = Get-Command python -ErrorAction Stop
        & $detectedPython.Source -m venv (Join-Path $projectRoot ".venv")
    }
    if ($LASTEXITCODE -ne 0) { throw "无法创建虚拟环境，请通过 -Python 指定 Python 3.12。" }
}
& $environmentPython -X utf8 -m pip install -r (Join-Path $projectRoot "requirements-lock.txt") --cache-dir (Join-Path $projectRoot "checks\pip_cache")
if ($LASTEXITCODE -ne 0) { throw "依赖安装失败。" }
Write-Output "FocControl 独立环境已准备完成。"
