#ifndef CHASSIS_CONFIG_H_
#define CHASSIS_CONFIG_H_

#include <stdint.h>

/* 底盘统一使用 4 个轮子。数组下标固定对应电机编号，便于和 IO 表核对。 */
typedef enum e_chassis_wheel
{
    CHASSIS_WHEEL_1 = 0,
    CHASSIS_WHEEL_2 = 1,
    CHASSIS_WHEEL_3 = 2,
    CHASSIS_WHEEL_4 = 3,
    CHASSIS_WHEEL_COUNT = 4
} chassis_wheel_t;

/* 轮位映射：按当前实车接线定义。
 * W1 = 左前，W2 = 右前，W3 = 左后，W4 = 右后。
 * 如果后续电机接线位置变化，只改这里的宏，不需要改麦轮逆解公式。
 */
#define CHASSIS_WHEEL_LEFT_FRONT   CHASSIS_WHEEL_1
#define CHASSIS_WHEEL_RIGHT_FRONT  CHASSIS_WHEEL_2
#define CHASSIS_WHEEL_LEFT_REAR    CHASSIS_WHEEL_3
#define CHASSIS_WHEEL_RIGHT_REAR   CHASSIS_WHEEL_4

/* 车体几何参数来自 WHEELTEC 麦轮车型参数。
 * 这里需要的是“半轮距”和“半轴距”，不是轴直径。
 * 半轮距 = 左右轮接地点中心距离 / 2。
 * 半轴距 = 前后轮接地点中心距离 / 2。
 * 麦轮旋转项使用二者之和。
 */
#define CHASSIS_HALF_WHEEL_SPACING_M  (0.1035f)
#define CHASSIS_HALF_AXLE_SPACING_M   (0.0985f)

/* 编码器参数
 * 500 线编码器、4 倍频、28 减速比、75 mm 轮径。
 */
#define CHASSIS_ENCODER_PPR           (500.0f)
#define CHASSIS_ENCODER_MULTIPLE      (4.0f)
#define CHASSIS_MOTOR_GEAR_RATIO      (28.0f)
#define CHASSIS_WHEEL_DIAMETER_M      (0.075f)
#define CHASSIS_PI                    (3.1415926535f)
#define CHASSIS_PULSES_PER_WHEEL_REV  (CHASSIS_ENCODER_PPR * CHASSIS_ENCODER_MULTIPLE * CHASSIS_MOTOR_GEAR_RATIO)

/* 速度命令限幅。单位：vx/vy 为 m/s，wz 为 rad/s。
 * 默认值偏保守，第一次上车调试时降低风险，后续可按实车能力调整。
 */
#define CHASSIS_MAX_VX_MPS            (1.0f)
#define CHASSIS_MAX_VY_MPS            (1.0f)
#define CHASSIS_MAX_WZ_RADPS          (3.14f)

/* 单轮目标速度限幅。WHEELTEC 原程序使用约 3.5 m/s，这里保留该上限。 */
#define CHASSIS_MAX_WHEEL_SPEED_MPS   (3.5f)

/* 增量 PI 输出限幅，单位为 GPT PWM 计数。
 * PI 只作为死区补偿和速度前馈的小范围修正量，不需要大输出范围。
 */
#define CHASSIS_PID_OUTPUT_LIMIT      (5000.0f)
#define CHASSIS_PID_DEFAULT_KP        (80.0f)
#define CHASSIS_PID_DEFAULT_KI        (5.0f)

/* 电机 PWM 死区补偿阈值，单位为 GPT PWM 计数。目标速度非零时叠加此偏置跨过死区。 */
#define CHASSIS_PWM_DEADBAND_COUNTS          (13500L)

/* 目标轮速判零阈值，单位 m/s。低于此值视为速度命令为零。 */
#define CHASSIS_ZERO_TARGET_EPS_MPS          (0.01f)

/* 速度前馈系数，单位 PWM 计数 / (m/s)。实测8200。 */
#define CHASSIS_SPEED_FF_COUNTS_PER_MPS     (8200.0f)

/* 开环 PWM 诊断限幅，单位为 GPT PWM 计数。 */
#define CHASSIS_PWM_TEST_LIMIT_COUNTS (30000L)

/* UI 和 Flash 保存允许的 PID 参数范围。 */
#define CHASSIS_PID_PARAM_MIN         (0.0f)
#define CHASSIS_PID_PARAM_MAX         (500.0f)

/* 电机 PWM 方向校正。若某一路电机正方向与期望相反，将对应值改为 -1。 */
#define CHASSIS_MOTOR1_PWM_POLARITY   (1)
#define CHASSIS_MOTOR2_PWM_POLARITY   (-1)
#define CHASSIS_MOTOR3_PWM_POLARITY   (1)
#define CHASSIS_MOTOR4_PWM_POLARITY   (-1)

/* 编码器方向校正。根据实车手转测试确定正转对应的计数方向和极性。
 * 前转为正方向（车前进时轮子正转方向），校正后前转应输出正的 m/s。
 */
#define CHASSIS_ENCODER1_POLARITY     (-1)
#define CHASSIS_ENCODER2_POLARITY     (-1)
#define CHASSIS_ENCODER3_POLARITY     (1)
#define CHASSIS_ENCODER4_POLARITY     (-1)

/* GPT PWM 周期兜底值。motor.init() 会优先通过 FSP infoGet 读取真实周期。 */
#define CHASSIS_PWM_PERIOD_FALLBACK   (30000U)

#endif /* CHASSIS_CONFIG_H_ */
