# FocControl 上位机

基于 Python、PySide6、pyqtgraph 与 NumPy 的电机调试界面，支持串口参数读取与编辑、初始化、启停、波形显示、位置控制及虚拟档位/阻尼配置。

## 启动

在本目录运行，使用 Python 3.12：

```powershell
pwsh -File setup.ps1
pwsh -File run.ps1 -Demo
```

`-Demo` 使用内置模拟板卡，可离线体验界面。连接硬件时执行 `run.ps1`，在界面选择实际串口及波特率；只读监视执行 `run.ps1 -ReadOnly`。运行数据保存在本机 `data/`，不随源码提供。

参数修改和控制操作按固件能力、单位、输出状态及静止确认开放；写入后查询回执与参数回读。实时曲线保留数据缺口，查询延迟时显示最近读数。参数备份与恢复由用户在界面操作。

## 开发与打包

源码入口为 `src/main.py`，模块位于 `src/foc_control/`，测试位于 `src/tests/`。依赖版本由 `requirements-lock.txt` 固定。

```powershell
.venv/Scripts/python.exe -m pytest -q
pwsh -File build.ps1
```

打包脚本执行静态检查与离线测试，生成独立目录发布包。第三方许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。固件入口见 [项目说明](../README.md)。
