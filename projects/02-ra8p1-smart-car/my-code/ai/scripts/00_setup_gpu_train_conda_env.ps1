param(
    [string]$EnvDir = "",
    [string]$PythonVersion = "3.10",
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$AiRoot = Resolve-Path (Join-Path $ScriptDir "..")

if ([string]::IsNullOrWhiteSpace($EnvDir))
{
    $EnvDir = Join-Path $AiRoot ".gpu-train"
}

$PythonExe = Join-Path $EnvDir "python.exe"

Write-Host "GPU 训练环境目录: $EnvDir"
Write-Host "该环境只用于 KWS radical/QAT GPU 训练，不用于录音转写或 RUHMI。"

$CondaCommand = Get-Command conda -ErrorAction SilentlyContinue
if ($null -eq $CondaCommand)
{
    throw "未找到 conda。请先打开 Anaconda/Miniconda PowerShell，或把 conda 加入 PATH。"
}

if ((Test-Path -LiteralPath $EnvDir) -and (-not $Force))
{
    Write-Host "环境已存在，跳过创建。如需重建，请追加 -Force。"
}
else
{
    if ((Test-Path -LiteralPath $EnvDir) -and $Force)
    {
        Write-Host "删除旧环境: $EnvDir"
        conda env remove -y -p $EnvDir
    }

    Write-Host "创建 conda prefix 环境..."
    conda create -y -p $EnvDir "python=$PythonVersion" pip
}

if (-not (Test-Path -LiteralPath $PythonExe))
{
    throw "未找到环境 Python: $PythonExe"
}

Write-Host "升级 pip 基础工具..."
& $PythonExe -m pip install --upgrade pip setuptools wheel

Write-Host "安装 TensorFlow 2.10 DirectML GPU 训练依赖..."
& $PythonExe -m pip install `
    "numpy<1.24" `
    "tensorflow-cpu==2.10.0" `
    "tensorflow-directml-plugin==0.4.0.dev230202" `
    "tensorflow-model-optimization==0.7.5" `
    "tflite==2.10.0" `
    "scikit-learn" `
    "matplotlib" `
    "seaborn"

$CheckCode = @"
import os
os.environ.setdefault("TF_USE_LEGACY_KERAS", "1")

import tensorflow as tf
import tensorflow_model_optimization as tfmot
import tflite

print("python", os.sys.executable)
print("tensorflow", tf.__version__)
print("tfmot", tfmot.__version__)
print("tflite ok")
print("gpu", tf.config.list_physical_devices("GPU"))

if not tf.config.list_physical_devices("GPU"):
    raise SystemExit("未检测到 TensorFlow GPU 设备，请不要继续训练。")
"@

Write-Host "验证 GPU 训练环境..."
& $PythonExe -c $CheckCode

Write-Host "GPU 训练环境创建完成。"
