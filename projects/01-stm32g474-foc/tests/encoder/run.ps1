$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'tem/encoder_tests'
New-Item -ItemType Directory -Force $out | Out-Null
Push-Location $root
try {
    # 运行实际驱动和观测算法，只替换SPI事务，不访问板卡。
    & gcc -std=c11 -Wall -Wextra -Werror -Iproject/app -Iproject/devices/mt6835 tests/encoder/test_encoder.c project/app/encoder_readout.c project/devices/mt6835/mt6835.c -o "$out/encoder.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器测试编译失败' }
    & "$out/encoder.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器测试失败' }
    # 真实有感换相算法：两方向、回绕、标定、无反馈、超速和反转/堵转。
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_six_step.c project/control/encoder_six_step.c -lm -o "$out/encoder_six.exe"
    if ($LASTEXITCODE -ne 0) { throw '有感六步算法编译失败' }
    & "$out/encoder_six.exe"
    if ($LASTEXITCODE -ne 0) { throw '有感六步算法测试失败' }
    # 独立有感FOC直接闭环：校准角、首帧等待、CRC/年龄/堵转以及多角度机械植物。
    & gcc -DMOTOR_PROFILE_G2208=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_foc.c project/control/encoder_foc.c project/control/encoder_position.c project/control/foc_current.c project/control/encoder_haptic.c -lm -o "$out/encoder_foc.exe"
    if ($LASTEXITCODE -ne 0) { throw '有感FOC算法编译失败' }
    & "$out/encoder_foc.exe"
    if ($LASTEXITCODE -ne 0) { throw '有感FOC算法测试失败' }
    # 独立Iq：静止正负0.5A、换向/过零、双向轨迹、原超速/反馈及抗饱和。
    & gcc -DMOTOR_PROFILE_G2208=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_iq.c project/control/encoder_foc.c project/control/encoder_position.c project/control/foc_current.c project/control/encoder_haptic.c -lm -o "$out/encoder_iq.exe"
    if ($LASTEXITCODE -ne 0) { throw '独立Iq算法编译失败' }
    & "$out/encoder_iq.exe"
    if ($LASTEXITCODE -ne 0) { throw '独立Iq算法测试失败' }
    # 多圈位置：原始跨圈、零点、双方向和完整dq/RL机械模型。
    & gcc -DMOTOR_PROFILE_G2208=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_position.c project/control/encoder_foc.c project/control/encoder_position.c project/control/foc_current.c project/control/encoder_haptic.c -lm -o "$out/encoder_position.exe"
    if ($LASTEXITCODE -ne 0) { throw '位置算法编译失败' }
    & "$out/encoder_position.exe"
    if ($LASTEXITCODE -ne 0) { throw '位置算法回归失败' }
    # 手感模块：双向阻尼、档位回差、多圈计数及真实dq/PWM延迟模型。
    & gcc -DMOTOR_PROFILE_G2208=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_haptic.c project/control/encoder_foc.c project/control/encoder_position.c project/control/foc_current.c project/control/encoder_haptic.c -lm -o "$out/encoder_haptic.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感算法编译失败' }
    & "$out/encoder_haptic.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感算法回归失败' }
    # 同时闭合真实PLL/电流/PWM和机械植物，覆盖手拨后松手持续振荡及传感噪声。
    & gcc -DMOTOR_PROFILE_G2208=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_haptic_stability.c project/control/encoder_foc.c project/control/encoder_position.c project/control/foc_current.c project/control/encoder_haptic.c -lm -o "$out/haptic_stability.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感耦合稳定性编译失败' }
    & "$out/haptic_stability.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感耦合稳定性回归失败' }
    # 真实机械测量：90秒、双方向、DWT多次回绕和循环缓存批读/缺样记录。
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_encoder_motion.c project/control/encoder_motion.c -o "$out/encoder_motion.exe"
    if ($LASTEXITCODE -ne 0) { throw '机械测量编译失败' }
    & "$out/encoder_motion.exe"
    if ($LASTEXITCODE -ne 0) { throw '机械测量回归失败' }
    # 实际整脉冲预算：原序列兼容、累计脉宽、即时限流和固定音调谱线。
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/control tests/encoder/test_pulse_density.c project/control/six_step.c -lm -o "$out/pulse_density.exe"
    if ($LASTEXITCODE -ne 0) { throw '脉冲密度算法编译失败' }
    & "$out/pulse_density.exe"
    if ($LASTEXITCODE -ne 0) { throw '脉冲密度算法测试失败' }
    # 正式板卡SPI1 DMA状态机，不连接设备，检查完整性及无主循环时的采集。
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -Wno-pointer-to-int-cast -Itests/encoder/mocks -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/mt6835 tests/encoder/test_encoder_dma.c project/control/encoder_position.c project/app/encoder_readout.c project/devices/mt6835/mt6835.c -o "$out/encoder_dma.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器DMA编译失败' }
    & "$out/encoder_dma.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器DMA测试失败' }
    # 编译真实应用后端，只替换Flash、功率寄存器和机械轨迹。
    & gcc -DMOTOR_PROFILE_G2208=1 -DMOTOR_ENCODER_ENABLED=1 -DMOTOR_FOC_ENABLED=1 -DMOTOR_SENSORLESS_TEST_TICK=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/drv8353f tests/encoder/test_encoder_backend.c project/app/settings.c project/app/motor_backend.c project/app/control_model.c project/control/encoder_six_step.c project/control/encoder_position.c project/control/six_step.c project/control/sensorless_six_step.c project/control/foc_drive.c project/control/foc_current.c project/control/foc_smo.c project/control/foc_voltage.c project/devices/drv8353f/drv8353f.c project/control/encoder_haptic.c -lm -o "$out/encoder_backend.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器六步后端编译失败' }
    & "$out/encoder_backend.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器六步后端测试失败' }
} finally { Pop-Location }
