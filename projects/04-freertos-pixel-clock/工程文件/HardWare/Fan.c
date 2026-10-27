#include "Fan.h"

// 定义定时器句柄
TIM_HandleTypeDef htim5_fan;

/**
 * @brief  初始化 TIM5 Channel 2 (PA1) 用于控制 4-Pin 风扇
 * @note   目标频率 25kHz (标准风扇频率)
 *         基于 APB1 Timer Clock = 84MHz
 */
void Fan_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // ============================================================
    // 1. 开启时钟
    // ============================================================
    __HAL_RCC_TIM5_CLK_ENABLE();  // 开启定时器5时钟
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 开启GPIOA时钟 (PA1)

    // ============================================================
    // 2. 配置 GPIO (PA1 -> TIM5_CH2)
    // ============================================================
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;      // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // PWM不需要太高GPIO速度，低速可减少噪声
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;   // 复用映射到 TIM5
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ============================================================
    // 3. 配置定时器基础参数 (Time Base)
    // ============================================================
    htim5_fan.Instance = TIM5;
    // 预分频器 (PSC) = 0，即不分频，计数频率 = 84MHz
    htim5_fan.Init.Prescaler = 0;
    // 计数模式：向上计数
    htim5_fan.Init.CounterMode = TIM_COUNTERMODE_UP;
    // 自动重装载值 (ARR) = 3359
    // 频率 = 84,000,000 / (0+1) / (3359+1) = 25,000 Hz (25kHz)
    htim5_fan.Init.Period = 3359;
    htim5_fan.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5_fan.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // 使能自动重装载预装载

    if (HAL_TIM_PWM_Init(&htim5_fan) != HAL_OK)
    {
        // 初始化错误处理
        while (1)
            ;
    }

    // ============================================================
    // 4. 配置 PWM 通道 (Channel 2)
    // ============================================================
    sConfigOC.OCMode = TIM_OCMODE_PWM1;         // PWM模式1 (计数值 < 比较值时为有效电平)
    sConfigOC.Pulse = 0;                        // 初始占空比 0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW; // 有效电平为低电平 
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim5_fan, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        // 配置错误处理
        while (1)
            ;
    }

    // ============================================================
    // 5. 启动 PWM 输出
    // ============================================================
    HAL_TIM_PWM_Start(&htim5_fan, TIM_CHANNEL_2);
}

/**
 * @brief  设置风扇速度
 * @param  duty_cycle: 0 - 100 (百分比)
 *         0   = 停止 (或风扇最低转速)
 *         100 = 全速
 */
void Fan_Set_Speed(uint8_t duty_cycle)
{
    uint32_t pulse_value;

    // 限制输入范围
    if (duty_cycle > 100)
        duty_cycle = 100;

    // 计算 CCR 寄存器的值
    // ARR = 3359, 总周期计数值为 3360
    // Pulse = (3360 * duty) / 100
    pulse_value = (3360 * duty_cycle) / 100;

    // 修改 TIM5 Channel 2 的比较寄存器 (CCR2)
    __HAL_TIM_SET_COMPARE(&htim5_fan, TIM_CHANNEL_2, pulse_value);
}
