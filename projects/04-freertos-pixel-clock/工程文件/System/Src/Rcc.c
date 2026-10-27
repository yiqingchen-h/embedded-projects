#include "main.h"
/**
  * @brief  时钟初始化失败时进入
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	// 处理时钟初始化失败错误
  while(1)
  {
  }
}
/*f4固件中默认hse频率为25Mhz在stm32f4xx_hal_conf.h中
一定要根据板子上hse频率进行修改！！！！！！！
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  __HAL_RCC_PWR_CLK_ENABLE();		//开启RCC时钟
  /*
	设置电压匹配频率
	PWR_REGULATOR_VOLTAGE_SCALE1：最高电压（1.2v）最大频率-高性能高功耗
	PWR_REGULATOR_VOLTAGE_SCALE2：中电压（1.15v）中频率-中性能中功耗
	PWR_REGULATOR_VOLTAGE_SCALE3：最低电压（1.0v）最小频率-低性能低功耗
	*/
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);	

	/*外部高速时钟配置*/
	// 1. 配置主PLL (HSE作为PLL源)
  // 外部晶振 HSE = 8MHz
  // PLL_VCO = (HSE / PLL_M) * PLL_N = (8 / 8) * 336 = 336 MHz
  // PLLCLK = PLL_VCO / PLL_P = 336 / 2 = 168 MHz (系统时钟)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;	// 设置外部高速时钟
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;										// 开启外部高速时钟
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;								// 开启PPL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;				// 外部高速时钟不分频做PPL时钟源
  RCC_OscInitStruct.PLL.PLLM = 8;															// PPL输入分频器8分频
  RCC_OscInitStruct.PLL.PLLN = 336;														// PPL 倍频336
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;									// PPL VCO输出信号分频
  RCC_OscInitStruct.PLL.PLLQ = 7;															// VCO分频
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
   // 2. 配置系统时钟总线源和分频器
  // AHB总线 (HCLK) = SYSCLK = 168 MHz
  // APB1总线 (PCLK1) = HCLK / 4 = 42 MHz (TIMx时钟 = PCLK1 * 2 = 84 MHz)
  // APB2总线 (PCLK2) = HCLK / 2 = 84 MHz
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);	// 要配置的时钟
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;		// 选择PPL作系统时钟
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;					// 配置AHB总线时钟(HCLK)分频系数为1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  					// 配置APB1分频系数为4
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  					// 配置APB2分频系数为2
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z and upper devices: prefetch is supported  */
  if (HAL_GetREVID() >= 0x1001)		// 检查版本号决定是否开启预取功能
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();	//开启Flash预取功能
  }
}

// FreeRtos会降低原systick时钟的优先级 所有需要修改hak库时钟现将systick 改为Time1
TIM_HandleTypeDef htim7;
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
   RCC_ClkInitTypeDef clkconfig;
   uint32_t uwTimclock       = 0;
   uint32_t uwPrescalerValue = 0;
   uint32_t pFLatency;
   //设置中断优先级
   HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0);

   //启用定时器全局中断
   HAL_NVIC_EnableIRQ(TIM7_IRQn);

   //启动定时器1
   __HAL_RCC_TIM7_CLK_ENABLE();

   //获取时钟配置
   HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

   //计算时钟
   uwTimclock = 2 * HAL_RCC_GetPCLK1Freq();
   /* Compute the prescaler value to have TIM1 counter clock equal to 1MHz */
   // 计算预分频值使时钟频率==1Mhz
   uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

   htim7.Instance = TIM7;

   /* Initialize TIMx peripheral as follow:
   + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
   + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   + ClockDivision = 0
   + Counter direction = Up
   */
   htim7.Init.Period        = (1000000U / 1000U) - 1U;
   htim7.Init.Prescaler     = uwPrescalerValue;
   htim7.Init.ClockDivision = 0;
   htim7.Init.CounterMode   = TIM_COUNTERMODE_UP;

   if (HAL_TIM_Base_Init(&htim7) == HAL_OK) {
       /* Start the TIM time Base generation in interrupt mode */
       //启动定时器中断模式
       return HAL_TIM_Base_Start_IT(&htim7);
   }

   /* Return function status */
   return HAL_ERROR;
}
// TIM7 更新中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        HAL_IncTick();  // 更新 HAL 库的时基
    }
}
