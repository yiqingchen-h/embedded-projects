param([switch]$SkipChecks)

$ErrorActionPreference = "Stop"
$projectRoot = $PSScriptRoot
$environmentPython = Join-Path $projectRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $environmentPython)) { throw "请先运行 setup.ps1。" }
Push-Location -LiteralPath $projectRoot
try {
    if (-not $SkipChecks) {
        & $environmentPython -X utf8 -m ruff check src
        if ($LASTEXITCODE -ne 0) { throw "静态检查失败。" }
        & $environmentPython -X utf8 -m black src --check --workers 1
        if ($LASTEXITCODE -ne 0) { throw "格式检查失败。" }
        & $environmentPython -X utf8 -m mypy src/foc_control src/main.py --strict
        if ($LASTEXITCODE -ne 0) { throw "类型检查失败。" }
        & $environmentPython -X utf8 -m pytest -q --basetemp checks/pytest_build
        if ($LASTEXITCODE -ne 0) { throw "离线测试失败。" }
    }
    # 按源码版本独立发布，旧版及其用户data保持原目录。
    $versionFile = Join-Path $projectRoot 'src\foc_control\__init__.py'
    $versionMatch = [regex]::Match((Get-Content -LiteralPath $versionFile -Raw), '__version__\s*=\s*"([0-9]+\.[0-9]+\.[0-9]+)"')
    if (-not $versionMatch.Success) { throw '无法读取发布版本。' }
    $releaseName = 'FocControl-' + $versionMatch.Groups[1].Value
    & $environmentPython -X utf8 -m PyInstaller src/foc_control.spec --noconfirm --distpath build/bundle --workpath build/pyinstaller
    if ($LASTEXITCODE -ne 0) { throw "EXE 打包失败。" }
    $bundleDirectory = Join-Path $projectRoot "build\bundle\FocControl"
    $releaseDirectory = Join-Path $projectRoot ("release\" + $releaseName)
    # 将使用说明同时放入压缩包的程序根目录，便于解压后直接查阅。
    $releaseReadme = Get-Content -LiteralPath (Join-Path $projectRoot 'README.md') -Raw
    Set-Content -LiteralPath (Join-Path $bundleDirectory "README.md") -Value $releaseReadme -Encoding utf8
    Copy-Item -LiteralPath (Join-Path $projectRoot "THIRD_PARTY_NOTICES.md") -Destination $bundleDirectory -Force
    New-Item -ItemType Directory -Path $releaseDirectory -Force | Out-Null
    # 清除本程序旧依赖，防止已删除的同名 DLL 在下一次构建中残留。
    $internalDirectory = [IO.Path]::GetFullPath((Join-Path $releaseDirectory "_internal"))
    $expectedDirectory = [IO.Path]::GetFullPath((Join-Path $projectRoot ("release\" + $releaseName + '\_internal')))
    if ($internalDirectory -ne $expectedDirectory) { throw "内部依赖路径校验失败。" }
    if (Test-Path -LiteralPath $internalDirectory) {
        Remove-Item -LiteralPath $internalDirectory -Recurse -Force
    }
    foreach ($entry in (Get-ChildItem -LiteralPath $bundleDirectory)) {
        Copy-Item -LiteralPath $entry.FullName -Destination $releaseDirectory -Recurse -Force
    }
    $zipPath = Join-Path $projectRoot ("release\" + $releaseName + '-win64.zip')
    # 压缩全新的构建目录，不将用户预设、日志和偏好收入发布 ZIP。
    Compress-Archive -Path $bundleDirectory -DestinationPath $zipPath -Force
    Write-Output "发布程序：$releaseDirectory\FocControl.exe"
    Write-Output "发布压缩包：$zipPath"
} finally {
    Pop-Location
}
