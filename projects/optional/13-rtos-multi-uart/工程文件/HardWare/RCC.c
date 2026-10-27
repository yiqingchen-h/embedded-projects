#include "stm32f1xx_hal.h"
#include "RCC.h"

// 配置系统时钟-使用内部高速时钟
void RccClock_HSI_Init(void)
{
    // 配置时钟源结构体
    RCC_OscInitTypeDef RCC_OscInitType;
    RCC_OscInitType.OscillatorType = RCC_OSCILLATORTYPE_HSI; // 内部高速时钟
    // RCC_OscInitType.HSEtate = RCC_HSE_OFF ;       //关闭外部高速时钟
    RCC_OscInitType.HSIState            = RCC_HSI_ON;                 // 开启内部高速时钟
    RCC_OscInitType.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // 防止内部时钟不稳定设置的微调值
    RCC_OscInitType.PLL.PLLState        = RCC_PLL_ON;                 // 开启PLL
    RCC_OscInitType.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;     // 使用内部时钟HSI进行二分频作为PLL的时钟源
    RCC_OscInitType.PLL.PLLMUL          = RCC_PLL_MUL8;               // 对内部时钟进行16倍频
    HAL_RCC_OscConfig(&RCC_OscInitType);                              // 配置时钟源

    RCC_ClkInitTypeDef RCC_ClkInitType;                                                                                     // 总线配置结构体
    RCC_ClkInitType.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // 选择要配置的时钟
    RCC_ClkInitType.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;                                                               // 选择PLL作为系统时钟
    RCC_ClkInitType.AHBCLKDivider  = RCC_SYSCLK_DIV1;                                                                       // 配置AHB总线时钟(HCLK)分频系数为1
    RCC_ClkInitType.APB1CLKDivider = RCC_HCLK_DIV2;                                                                         // 配置APB1分频系数为2
    RCC_ClkInitType.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitType, FLASH_LATENCY_2); // 配置各总线分频值和设置Flsah等待周期

    SystemCoreClockUpdate(); // HCLK被改变时必须调用否则将不准确
}
// 配置系统时钟-使用外部高速时钟
void RccClock_HSE_Init(void)
{
    // 配置时钟源结构体
    RCC_OscInitTypeDef RCC_OscInitType;
    RCC_OscInitType.OscillatorType = RCC_OSCILLATORTYPE_HSE; // 选择要配置的震荡电路
    RCC_OscInitType.HSEState       = RCC_HSE_ON;             // 打开HSE
    RCC_OscInitType.HSEPredivValue = RCC_HSE_PREDIV_DIV1;    // 设置HSE预分频值为1
    RCC_OscInitType.PLL.PLLMUL     = RCC_PLL_MUL9;           // 设置PLL倍频值9
    RCC_OscInitType.PLL.PLLSource  = RCC_PLLSOURCE_HSE;      // 设置PLL输入源
    RCC_OscInitType.PLL.PLLState   = RCC_PLL_ON;             // 设置PLL状态
    HAL_RCC_OscConfig(&RCC_OscInitType);                     // 配置时钟源

    RCC_ClkInitTypeDef RCC_ClkInitType;                                                                                     // 总线配置结构体
    RCC_ClkInitType.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // 选择要配置的时钟
    RCC_ClkInitType.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;                                                               // 选择PLL作为系统时钟
    RCC_ClkInitType.AHBCLKDivider  = RCC_SYSCLK_DIV1;                                                                       // 配置AHB总线时钟(HCLK)分频系数
    RCC_ClkInitType.APB1CLKDivider = RCC_HCLK_DIV2;                                                                         // 配置APB1分频系数为2
    RCC_ClkInitType.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitType, FLASH_LATENCY_2); // 配置各总线分频值和设置Flsah等待周期

    SystemCoreClockUpdate(); // HCLK被改变时必须调用否则将不准确
}
// FreeRtos会降低原systick时钟的优先级 所有需要修改hak库时钟现将systick 改为Time1
// HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
// {
//     TIM_HandleTypeDef htim1;
//     RCC_ClkInitTypeDef clkconfig;
//     uint32_t uwTimclock       = 0;
//     uint32_t uwPrescalerValue = 0;
//     uint32_t pFLatency;
//     //设置中断优先级
//     HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, TickPriority, 0);

//     //启用定时器全局中断
//     HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

//     //启动定时器1
//     __HAL_RCC_TIM1_CLK_ENABLE();

//     //获取时钟配置
//     HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

//     //计算时钟
//     uwTimclock = 2 * HAL_RCC_GetPCLK2Freq();
//     /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
//     // 计算预分频值使时钟频率==1Mhz
//     uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

//     htim1.Instance = TIM1;

//     /* Initialize TIMx peripheral as follow:
//     + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
//     + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
//     + ClockDivision = 0
//     + Counter direction = Up
//     */
//     htim1.Init.Period        = (1000000U / 1000U) - 1U;
//     htim1.Init.Prescaler     = uwPrescalerValue;
//     htim1.Init.ClockDivision = 0;
//     htim1.Init.CounterMode   = TIM_COUNTERMODE_UP;

//     if (HAL_TIM_Base_Init(&htim1) == HAL_OK) {
//         /* Start the TIM time Base generation in interrupt mode */
//         //启动定时器中断模式
//         return HAL_TIM_Base_Start_IT(&htim1);
//     }

//     /* Return function status */
//     return HAL_ERROR;
// }
