param([switch]$ReadOnly, [switch]$Demo)

$ErrorActionPreference = "Stop"
$environmentPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $environmentPython)) { throw "请先运行 setup.ps1。" }
$launchArguments = @("-X", "utf8", (Join-Path $PSScriptRoot "src\main.py"))
if ($ReadOnly) { $launchArguments += "--read-only" }
if ($Demo) { $launchArguments += "--demo" }
& $environmentPython @launchArguments
