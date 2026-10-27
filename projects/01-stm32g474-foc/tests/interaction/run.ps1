$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'tem/interaction'
New-Item -ItemType Directory -Force $out | Out-Null
Push-Location $root
try {
    $base = @('-std=c11','-Wall','-Wextra','-Werror','-Iproject/input','-Iproject/protocol')
    # 独立三路极性与方向互换，共16种构建组合。
    for ($mask = 0; $mask -lt 16; ++$mask) {
        $defines = @("-DSWITCH_PRESS_ACTIVE_LEVEL=$($mask -band 1)", "-DSWITCH_DOWN_ACTIVE_LEVEL=$(($mask -shr 1) -band 1)", "-DSWITCH_UP_ACTIVE_LEVEL=$(($mask -shr 2) -band 1)", "-DSWITCH_SWAP_UP_DOWN=$(($mask -shr 3) -band 1)")
        & gcc @base @defines tests/interaction/test_logic.c project/input/switch_input.c project/protocol/console.c project/protocol/serial_buffer.c -o "$out/logic.exe"
        if ($LASTEXITCODE -ne 0) { throw '逻辑测试编译失败' }
        & "$out/logic.exe"
        if ($LASTEXITCODE -ne 0) { throw "极性组合 $mask 测试失败" }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -Iproject/app tests/interaction/test_model.c project/app/control_model.c -o "$out/model.exe"
    if ($LASTEXITCODE -ne 0) { throw '模型测试编译失败' }
    & "$out/model.exe"
    if ($LASTEXITCODE -ne 0) { throw '模型测试失败' }
    # 无OLED/I²C替身地编译正式交互层，残留显示依赖会直接链接失败。
    $includes = @('-Iproject/app','-Iproject/board','-Iproject/control')
    $sources = @('tests/interaction/test_protocol.c','project/app/interaction.c','project/protocol/wave_stream.c','project/app/encoder_readout.c','project/app/settings.c','project/control/encoder_haptic.c','project/app/control_model.c','project/input/switch_input.c','project/protocol/console.c','project/protocol/serial_buffer.c')
    & gcc @base @includes @sources -o "$out/protocol.exe"
    if ($LASTEXITCODE -ne 0) { throw '串口集成测试编译失败' }
    & "$out/protocol.exe"
    if ($LASTEXITCODE -ne 0) { throw '串口集成测试失败' }
    # 实际纯协议和UART DMA状态机另行运行，不能用硬件替身结果冒充实板验收。
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Iproject/protocol tests/interaction/test_wave_stream.c project/protocol/wave_stream.c -lm -o "$out/wave.exe"
    if ($LASTEXITCODE -ne 0) { throw '波形协议编译失败' }
    & "$out/wave.exe"
    if ($LASTEXITCODE -ne 0) { throw '波形协议回归失败' }
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Wno-pointer-to-int-cast -Itests/interaction/mocks -Iproject/board -Iproject/protocol tests/interaction/test_uart_dma.c project/protocol/serial_buffer.c -o "$out/uart_dma.exe"
    if ($LASTEXITCODE -ne 0) { throw 'UART DMA编译失败' }
    & "$out/uart_dma.exe"
    if ($LASTEXITCODE -ne 0) { throw 'UART DMA回归失败' }
} finally { Pop-Location }
