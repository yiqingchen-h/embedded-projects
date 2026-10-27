$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$out = Join-Path $root 'tem/motor_tests'
New-Item -ItemType Directory -Force $out | Out-Null
Push-Location $root
try {
    # 有感手感的冻结V5迁移、V6掉电边界和位置参数隔离，纯内存执行。
    & gcc -DMOTOR_ENCODER_ENABLED=1 -DMOTOR_PROFILE_G2208=1 -DMOTOR_FOC_ENABLED=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/control tests/motor/test_settings_haptic.c project/app/settings.c project/app/control_model.c project/control/encoder_haptic.c -lm -o "$out/settings_haptic.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感参数迁移编译失败' }
    & "$out/settings_haptic.exe"
    if ($LASTEXITCODE -ne 0) { throw '手感参数迁移回归失败' }
    # 历史场景固定原电机档，新默认G2208另行验证，不能把原电机模型通过当作新机通过。
    # FOC算法使用正式源码；验证因果电压历史及含硬件作用延迟的电流闭环。
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/control -Iproject/app tests/motor/test_foc_voltage.c project/control/foc_voltage.c -lm -o "$out/foc_voltage.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC电压历史编译失败' }
    & "$out/foc_voltage.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC电压历史回归失败' }
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/control -Iproject/app tests/motor/test_foc_current.c project/control/foc_current.c project/control/foc_voltage.c -lm -o "$out/foc_current.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC电流环编译失败' }
    & "$out/foc_current.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC电流环回归失败' }
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/control -Iproject/app tests/motor/test_foc_smo.c project/control/foc_smo.c project/control/foc_voltage.c -lm -o "$out/foc_smo.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC观察器编译失败' }
    & "$out/foc_smo.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC观察器回归失败' }
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/control -Iproject/app tests/motor/test_foc_drive.c project/control/foc_drive.c project/control/foc_current.c project/control/foc_smo.c project/control/foc_voltage.c -lm -o "$out/foc_drive.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC速度状态机编译失败' }
    & "$out/foc_drive.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC速度状态机回归失败' }
    # 实际应用后端与参数模块，覆盖普通START/在线目标/实际接管资格及停机保存。
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=1 -DMOTOR_SENSORLESS_TEST_TICK=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/drv8353f tests/motor/test_foc_backend.c project/app/settings.c project/app/motor_backend.c project/app/control_model.c project/control/six_step.c project/control/sensorless_six_step.c project/control/foc_drive.c project/control/foc_current.c project/control/foc_smo.c project/control/foc_voltage.c project/devices/drv8353f/drv8353f.c project/control/encoder_haptic.c -lm -o "$out/foc_backend.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC应用集成编译失败' }
    & "$out/foc_backend.exe"
    if ($LASTEXITCODE -ne 0) { throw 'FOC应用集成回归失败' }
    # 新档运行真实参数迁移、后端、7极对换算和电流控制；板卡及Flash为内存替身。
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=1 -DMOTOR_FOC_ENABLED=1 -DMOTOR_SENSORLESS_TEST_TICK=1 -std=c11 -O2 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/drv8353f tests/motor/test_g2208.c project/app/settings.c project/app/motor_backend.c project/app/control_model.c project/control/six_step.c project/control/sensorless_six_step.c project/control/foc_drive.c project/control/foc_current.c project/control/foc_smo.c project/control/foc_voltage.c project/devices/drv8353f/drv8353f.c project/control/encoder_haptic.c -lm -o "$out/g2208.exe"
    if ($LASTEXITCODE -ne 0) { throw 'G2208参数与迁移编译失败' }
    & "$out/g2208.exe"
    if ($LASTEXITCODE -ne 0) { throw 'G2208参数与迁移回归失败' }
    # 从正式算法提取实际宽除法函数，独立核对32位快路径和64位回退的数学等价性。
    $algorithmSource = Get-Content project/control/sensorless_six_step.c -Raw -Encoding UTF8
    $mathFunction = [regex]::Match($algorithmSource, '(?ms)^static uint32_t\r?\ndivide_wide\(.*?(?=^/\*\*|\z)')
    if (-not $mathFunction.Success) { throw '无法提取实际无感算术函数' }
    [IO.File]::WriteAllText("$out/sensorless_math_under_test.inc", $mathFunction.Value, [Text.UTF8Encoding]::new($false))
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -O2 -Wall -Wextra -Werror "-I$out" tests/motor/test_sensorless_math.c -o "$out/sensorless_math.exe"
    if ($LASTEXITCODE -ne 0) { throw '无感算术测试编译失败' }
    & "$out/sensorless_math.exe"
    if ($LASTEXITCODE -ne 0) { throw '无感算术回归失败' }
    # 相同物理时长分别在原10 kHz和正式20 kHz下验证，防止频率变更改变换向或截止时间。
    foreach ($frequency in @(10000,20000)) {
        & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror "-DMOTOR_CONTROL_HZ=$frequency" -DMOTOR_SENSORLESS_TEST_TICK=1 -DMOTOR_SENSORLESS_CLOSED_LOOP=0 -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/drv8353f tests/motor/test_motor.c project/app/settings.c project/app/motor_backend.c project/app/control_model.c project/control/six_step.c project/control/sensorless_six_step.c project/devices/drv8353f/drv8353f.c project/control/encoder_haptic.c -o "$out/motor_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "电机与参数测试编译失败，频率=$frequency" }
        & "$out/motor_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "电机与参数回归失败，频率=$frequency" }
        # 同一真实后端运行正式闭环分支，与观察构建分别覆盖接管与失锁路径。
        & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror "-DMOTOR_CONTROL_HZ=$frequency" -DMOTOR_SENSORLESS_TEST_TICK=1 -DMOTOR_SENSORLESS_CLOSED_LOOP=1 -Iproject/app -Iproject/board -Iproject/control -Iproject/devices/drv8353f tests/motor/test_motor.c project/app/settings.c project/app/motor_backend.c project/app/control_model.c project/control/six_step.c project/control/sensorless_six_step.c project/devices/drv8353f/drv8353f.c project/control/encoder_haptic.c -o "$out/motor_closed_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "闭环电机与参数测试编译失败，频率=$frequency" }
        & "$out/motor_closed_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "闭环电机与参数回归失败，频率=$frequency" }
        # 独立注入过零、丢样、失锁和PI状态，编译实际无感算法。
        & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror "-DMOTOR_CONTROL_HZ=$frequency" -DMOTOR_SENSORLESS_TEST_TICK=1 -Iproject/app -Iproject/control tests/motor/test_sensorless.c project/control/sensorless_six_step.c project/control/six_step.c -o "$out/sensorless_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "无感算法测试编译失败，频率=$frequency" }
        & "$out/sensorless_$frequency.exe"
        if ($LASTEXITCODE -ne 0) { throw "无感算法测试失败，频率=$frequency" }
    }
    # 旧版本参数必须完整保留有效开环配置并安全迁移。
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/control tests/motor/test_settings_migration.c project/app/settings.c project/app/control_model.c project/control/encoder_haptic.c -o "$out/settings_migration.exe"
    if ($LASTEXITCODE -ne 0) { throw '参数迁移测试编译失败' }
    & "$out/settings_migration.exe"
    if ($LASTEXITCODE -ne 0) { throw '参数迁移测试失败' }
    # 提取真实调度和 COM 提交函数，避免用实现副本冒充回归。
    $source = Get-Content project/board/board_motor.c -Raw -Encoding UTF8
    $defines = [regex]::Matches($source, '(?m)^#define (?:TIMER_HZ|TIMER_TICKS_US|PWM_ARR|SAMPLE_COMPARE|REGULAR_LAUNCH_TICKS|REGULAR_INTERVAL_TICKS|REGULAR_ITEMS|ADC1_DMA_REQUEST|ADC_IDLE_WARMUP_CYCLES|TIMER_DEADTIME_NS|DRIVER_DEADTIME_NS|SWITCH_SETTLE_NS|CURRENT_SAMPLE_SETTING|BEMF_HOLD_TICKS|BEMF_CONVERSION_TICKS|BEMF_EDGE_GUARD_TICKS|PWM_PERIOD_US|SPEED_LOOP_DIVIDER) .*') | ForEach-Object Value
    $functions = foreach ($name in @('motor_time_us','motor_clock_update','sampling_configure','board_motor_stop','apply_output','encoder_apply_output','regular_opportunity','bemf_publish','ADC1_2_IRQHandler','TIM1_UP_TIM16_IRQHandler','regular_configure','timer_prime_peak','board_motor_min_duty','board_motor_duty_ticks','regular_timing_ready','board_motor_observation','board_motor_set_peak_callback','board_motor_running','board_motor_set_control_callbacks','board_motor_control_lock','board_motor_control_unlock','board_motor_schedule','board_motor_set_control_duty','board_motor_cycles','board_motor_storage_safe','board_motor_start')) {
        $match = [regex]::Match($source, '(?ms)^(?:static )?(?:void|bool|uint16_t|uint32_t|board_motor_observation_t)(?: MOTOR_FAST_CODE)?\r?\n' + $name + '\(.*?(?=^/\*\*|\z)')
        if (-not $match.Success) { throw "无法提取实际函数 $name" }
        $match.Value
    }
    [IO.File]::WriteAllText("$out/sampling_under_test.inc", (($defines + $functions) -join "`n"), [Text.UTF8Encoding]::new($false))
    # 同时检查临时标记开启和撤除后的功率输出，避免回退标记影响六步使能。
    foreach ($frequency in @(10000,20000)) {
        foreach ($trace in @(0,1)) {
            & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror -Wno-pointer-to-int-cast "-DMOTOR_CONTROL_HZ=$frequency" -DMOTOR_SENSORLESS_TEST_TICK=1 "-DMOTOR_ADC_TRACE_ENC_CAL=$trace" -Iproject/app -Iproject/board -Iproject/control "-I$out" -include project/app/motor_config.h tests/motor/test_sampling.c project/control/six_step.c -o "$out/sampling_${frequency}_$trace.exe"
            if ($LASTEXITCODE -ne 0) { throw "ADC 调度测试编译失败，频率=$frequency，标记=$trace" }
            & "$out/sampling_${frequency}_$trace.exe"
            if ($LASTEXITCODE -ne 0) { throw "ADC 调度回归失败，频率=$frequency，标记=$trace" }
        }
    }
    # 运行实际校准、慢速发布和超时函数，覆盖100 ms等待及旧调度时间导致的误停。
    # 单个宏的管道结果是string，显式数组避免与函数数组相加时直接拼接首函数。
    $calibrationDefines = @([regex]::Matches($source, '(?m)^#define REGULAR_ITEMS .*') | ForEach-Object Value)
    $calibrationFunctions = foreach ($name in @('publish_regular','regular_read','board_motor_calibrate','board_motor_calibrate_gain','board_motor_service','board_motor_standstill')) {
        $match = [regex]::Match($source, '(?ms)^(?:static )?(?:bool|void)(?: MOTOR_BACKGROUND_CODE)?\r?\n' + $name + '\(.*?(?=^/\*\*|\z)')
        if (-not $match.Success) { throw "无法提取实际校准/监控函数 $name" }
        $match.Value
    }
    [IO.File]::WriteAllText("$out/calibration_under_test.inc", (($calibrationDefines + $calibrationFunctions) -join "`n"), [Text.UTF8Encoding]::new($false))
    & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/devices/drv8353f "-I$out" tests/motor/test_calibration.c -o "$out/calibration.exe"
    if ($LASTEXITCODE -ne 0) { throw '停机校准测试编译失败' }
    & "$out/calibration.exe"
    if ($LASTEXITCODE -ne 0) { throw '停机校准回归失败' }
    # 相同真实静止函数开启编码器协作采集，核对采样间隔且保留所有原拒绝条件。
    & gcc -DMOTOR_ENCODER_ENABLED=1 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -Wall -Wextra -Werror -Iproject/app -Iproject/board -Iproject/devices/drv8353f "-I$out" tests/motor/test_calibration.c -o "$out/calibration_encoder.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器协作静止预检编译失败' }
    & "$out/calibration_encoder.exe"
    if ($LASTEXITCODE -ne 0) { throw '编码器协作静止预检失败' }
    # 从正式存储模块运行实际地址/页号和错误传播，只替换 HAL 与 Flash 内存。
    $storageSource = Get-Content project/board/board_storage.c -Raw -Encoding UTF8
    $storageSource = $storageSource.Replace('#include "stm32g4xx_hal.h"','')
    [IO.File]::WriteAllText("$out/storage_under_test.inc", $storageSource, [Text.UTF8Encoding]::new($false))
    foreach ($bank in @(0,1)) {
        & gcc -DMOTOR_ENCODER_ENABLED=0 -DMOTOR_PROFILE_G2208=0 -DMOTOR_FOC_ENABLED=0 -std=c11 -D_GNU_SOURCE -Wall -Wextra -Werror -Wno-int-to-pointer-cast "-DBOARD_FLASH_DUAL_BANK=$bank" -Iproject/board "-I$out" tests/motor/test_storage.c -o "$out/storage_$bank.exe"
        if ($LASTEXITCODE -ne 0) { throw "Flash 布局 $bank 测试编译失败" }
        & "$out/storage_$bank.exe"
        if ($LASTEXITCODE -ne 0) { throw "Flash 布局 $bank 测试失败" }
    }
} finally { Pop-Location }
