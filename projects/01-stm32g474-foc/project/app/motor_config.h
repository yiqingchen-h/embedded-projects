#ifndef FOC_APP_MOTOR_CONFIG_H
#define FOC_APP_MOTOR_CONFIG_H

/* 默认使用G2208配置；旧电机配置用于离线回归，不支持运行时切换。 */
#ifndef MOTOR_PROFILE_G2208
#define MOTOR_PROFILE_G2208 1U
#endif
#if MOTOR_PROFILE_G2208
/* G2208图纸极对数、相电阻mΩ及等效相电感µH，尚待新电机实测确认。 */
#define MOTOR_POLE_PAIRS 7U
#define MOTOR_FOC_RESISTANCE_MOHM 10100U
#define MOTOR_FOC_INDUCTANCE_UH 1500U
/* 2026-10-25四轮300/600RPM以PWM电压模型估算约7.63mWb，非独立端电压实测。 */
#define MOTOR_FOC_FLUX_UWB 7630U
/* 初次调试相峰值0.5A/600RPM/5秒；相峰值0.8A为暂定铜损预算，非母线额定电流换算。 */
#define MOTOR_LIMIT_DEFAULT_MA 500U
#define MOTOR_LIMIT_MAX_MA 800U
#define MOTOR_TARGET_DEFAULT_RPM 600U
#define MOTOR_SPEED_MAX_RPM 1800U
#define MOTOR_RUN_DEFAULT_MS 5000U
/* 0.8A为记录的母线额定值，与相电流指令口径不同；板卡为50mΩ/12V。 */
#define MOTOR_POLES_DEFAULT MOTOR_POLE_PAIRS
#define MOTOR_SHUNT_DEFAULT_MOHM 50U
#define MOTOR_RATED_CURRENT_MA 800U
#define MOTOR_RATED_SPEED_RPM 1800U
#define MOTOR_SPEED_LIMIT_DEFAULT_RPM MOTOR_SPEED_MAX_RPM
/* 保留原六步50Hz牵引及20%占空比作为未标定起点，不表示新电机已验收。 */
#define MOTOR_DUTY_DEFAULT 200U
#define MOTOR_ALIGN_DEFAULT_MS 200U
#define MOTOR_START_DEFAULT_MHZ 50000U
#define MOTOR_RAMP_DEFAULT_MHZ_S 20000U
/* 新档机械速度越限保护；100ms软越限时间和立即硬关断方式保持。 */
#define MOTOR_FOC_SOFT_MAX_RPM 1900U
#define MOTOR_FOC_HARD_MAX_RPM 2000U
#define MOTOR_FOC_TARGET_DEFAULT_RPM MOTOR_TARGET_DEFAULT_RPM
#define MOTOR_FOC_LIMIT_MAX_MA MOTOR_LIMIT_MAX_MA
#define MOTOR_FOC_RUN_DEFAULT_MS MOTOR_RUN_DEFAULT_MS
#define MOTOR_FOC_SPEED_KP_DEFAULT 400U
/* 50mV上板仍失去融合，恢复100mV；仍保留6倍噪声及独立R/L误差界。 */
#define MOTOR_FOC_EMF_FLOOR_V .1f
/* G2208只降低无位置反馈的强拖电流A，运行限流不变；定位仍沿用0.4A。 */
#define MOTOR_FOC_START_CURRENT_A .4f
/* FOC启动接管点为300RPM，与六步换相最低资格分别配置。 */
#define MOTOR_FOC_STARTUP_RPM 300U
#else
/* 原3极对电机的冻结模型和默认值，用于原有离线回归。 */
#define MOTOR_POLE_PAIRS 3U
#define MOTOR_FOC_RESISTANCE_MOHM 790U
#define MOTOR_FOC_INDUCTANCE_UH 240U
#define MOTOR_FOC_FLUX_UWB 0U
#define MOTOR_LIMIT_DEFAULT_MA 1000U
#define MOTOR_LIMIT_MAX_MA 1500U
#define MOTOR_TARGET_DEFAULT_RPM 300U
#define MOTOR_SPEED_MAX_RPM 5400U
#define MOTOR_RUN_DEFAULT_MS 5000U
#define MOTOR_POLES_DEFAULT 0U
#define MOTOR_SHUNT_DEFAULT_MOHM 1U
#define MOTOR_RATED_CURRENT_MA 0U
#define MOTOR_RATED_SPEED_RPM 5400U
#define MOTOR_SPEED_LIMIT_DEFAULT_RPM MOTOR_TARGET_DEFAULT_RPM
#define MOTOR_DUTY_DEFAULT 100U
#define MOTOR_ALIGN_DEFAULT_MS 100U
#define MOTOR_START_DEFAULT_MHZ 1000U
#define MOTOR_RAMP_DEFAULT_MHZ_S 5000U
#define MOTOR_FOC_SOFT_MAX_RPM 5800U
#define MOTOR_FOC_HARD_MAX_RPM 6000U
#define MOTOR_FOC_TARGET_DEFAULT_RPM 1000U
#define MOTOR_FOC_LIMIT_MAX_MA 2000U
#define MOTOR_FOC_RUN_DEFAULT_MS 20000U
#define MOTOR_FOC_SPEED_KP_DEFAULT 200U
/* 原电机冻结的额外EMF裕量下限V，历史回归不跟随新电机试调。 */
#define MOTOR_FOC_EMF_FLOOR_V .1f
/* 原电机已验证的强拖1A上限和约50Hz接管点。 */
#define MOTOR_FOC_START_CURRENT_A 1.0f
#define MOTOR_FOC_STARTUP_RPM 1000U
#endif
/* 控制器与板卡诊断共用同一模型及电角速度→机械RPM换算。 */
#define MOTOR_FOC_RESISTANCE_OHM (MOTOR_FOC_RESISTANCE_MOHM * .001f)
#define MOTOR_FOC_INDUCTANCE_H (MOTOR_FOC_INDUCTANCE_UH * .000001f)
#define MOTOR_FOC_FLUX_WB (MOTOR_FOC_FLUX_UWB * .000001f)
/* 仅G2208有感后端默认启用；编译期关闭供同模型对照，不新增Flash或在线开关。 */
#ifndef MOTOR_FOC_FEEDFORWARD_ENABLED
#define MOTOR_FOC_FEEDFORWARD_ENABLED MOTOR_PROFILE_G2208
#endif
#define MOTOR_FOC_RPM_PER_RAD_S (9.54929658551f / MOTOR_POLE_PAIRS)
/* 保持800Hz电流环带宽；六步约50Hz最低资格独立于FOC启动参数。 */
#define MOTOR_FOC_BANDWIDTH_HZ 800U
/* 正式FOC会话采样点采用旧10kHz坐标，20kHz实际CCR4减4250。
 * G2208/CSA20在高调制下原点过早，A/B实测后移70tick（0.412us）；旧电机保持8400。 */
#define MOTOR_FOC_TRIGGER_DEFAULT_TICKS (MOTOR_PROFILE_G2208 ? 8470U : 8400U)
#define MOTOR_SIX_MIN_RPM ((3000U + MOTOR_POLE_PAIRS / 2U) / MOTOR_POLE_PAIRS)

/* 正式双Bank镜像由scatter初始化搬入SRAM，降低快中断的Flash取指等待；主机模型不改段。 */
#if defined(__ARMCC_VERSION)
#define MOTOR_FAST_CODE __attribute__((section(".motor_fastcode")))
#else
#define MOTOR_FAST_CODE
#endif
/* 停机初始化及主循环参数发布按体积优化，为128KiB器件保留位置功能空间。 */
#if defined(__ARMCOMPILER_VERSION)
#define MOTOR_BACKGROUND_CODE __attribute__((minsize))
#else
#define MOTOR_BACKGROUND_CODE
#endif

/* 独立FOC时序测试；六步主机回归可关闭该板卡分支，FOC另有真实算法回归。 */
#ifndef MOTOR_FOC_ENABLED
#define MOTOR_FOC_ENABLED 1U
#endif
/* 编码器有感六步独立实现；历史主机回归可显式关闭该硬件分支。 */
#ifndef MOTOR_ENCODER_ENABLED
#define MOTOR_ENCODER_ENABLED 1U
#endif

/* PWM及同步电流/浮相采样时基；与速度PI频率分别定义。 */
#ifndef MOTOR_PWM_HZ
#ifdef MOTOR_CONTROL_HZ
/* 兼容原主机回归的10/20kHz启动积分时基覆盖。 */
#define MOTOR_PWM_HZ MOTOR_CONTROL_HZ
#else
#define MOTOR_PWM_HZ 20000U
#endif
#endif
/* 旧宏只表示启动积分/统计的PWM时基，不表示闭环PI执行频率。 */
#ifndef MOTOR_CONTROL_HZ
#define MOTOR_CONTROL_HZ MOTOR_PWM_HZ
#endif
_Static_assert(MOTOR_CONTROL_HZ == MOTOR_PWM_HZ, "Legacy startup tick must match PWM");
/* TIM峰顶精确分频的速度PI；仅支持1kHz，增益和变化率以1ms定义。 */
#define MOTOR_SPEED_LOOP_HZ 1000U
/* 本板门限标定用停止快照；正常构建关闭，无打印和控制队列依赖。 */
#ifndef MOTOR_SENSORLESS_TRACE
#define MOTOR_SENSORLESS_TRACE 0U
#endif
/* 默认启用连续过零确认后的闭环换相；调试观测构建可覆盖为0。 */
#ifndef MOTOR_SENSORLESS_CLOSED_LOOP
#define MOTOR_SENSORLESS_CLOSED_LOOP 1U
#endif
/* 保存值0的速度PI初值：100RPM误差对应2%比例幅度及10%/s积分，仍需本板带载标定。 */
#define MOTOR_SENSORLESS_KP_DEFAULT_MILLI 200U
#define MOTOR_SENSORLESS_KI_DEFAULT_MILLI 1000U
/* 编码器已接回，PC5专用于CAL低电平；关闭外部标记，内部ADC触发保持原配置。 */
#ifndef MOTOR_ADC_TRACE_ENC_CAL
#define MOTOR_ADC_TRACE_ENC_CAL 0U
#endif
/* 首版软件采样过流门限；不等同于硬件瞬时峰值保护，单位 mA。 */
#define MOTOR_TRIP_MA 2000U
/* PWM 占空比以千分比表示；首版最大 25%。 */
/* G2208六步调试占空比上限为60%；原电机配置为25%。 */
#define MOTOR_DUTY_MAX (MOTOR_PROFILE_G2208 ? 600U : 250U)
/* 有感六步高占空比换相预留一拍全关闭窗口；G2208配置上限为95%。 */
#define MOTOR_ENCODER_DUTY_MAX (MOTOR_PROFILE_G2208 ? 950U : 250U)
/* 普通开环启动先按初始电频率牵引完整电周期，再开始既有加速。 */
#define MOTOR_START_PULLIN_CYCLES 1U
/* 电源工作范围，单位 mV；用于本次 12 V 电机。 */
#define MOTOR_BUS_MIN_MV 8000U
#define MOTOR_BUS_MAX_MV 15000U
/* 停机合并保存等待；电压监控过期门限，单位 ms。 */
#define MOTOR_SAVE_DELAY_MS 1000U
#define MOTOR_BUS_STALE_MS 100U
/* 停机零点平均次数。 */
#define MOTOR_ZERO_SAMPLES 256U
/* 停机重新配置前保持 ENABLE 低，覆盖 DRV8353F 的 1 ms 休眠复位时间。 */
#define MOTOR_DRIVER_SLEEP_US 1000U
/* 唤醒后再访问 SPI；本板冷启动实测采用100 ms，仅用于停机配置和校准。 */
#define MOTOR_DRIVER_WAKE_US 100000U
/* CSA 短接校准及恢复正常输入后的稳定等待，单位 us。 */
#define MOTOR_CSA_SETTLE_US 1000U
/* 仅上电初始化使用：最多两次校准，持续故障不在主循环无限重试。 */
#define MOTOR_BOOT_CAL_ATTEMPTS 2U
/* DRV8353F 线性输出距供电轨的裕量，单位 mV，用于启动前校核量程。 */
#define MOTOR_CSA_LINEAR_MARGIN_MV 250U

#endif
