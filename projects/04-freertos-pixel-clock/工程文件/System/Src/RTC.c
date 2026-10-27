#include "RTC.h"

RTC_HandleTypeDef hrtc;



void RTC_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  // 1. 开启电源时钟并允许访问备份域
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  // 2. 开启 RTC 外设时钟 (必须先开启才能读取备份寄存器)
  __HAL_RCC_RTC_ENABLE();

  // 3. 初始化 hrtc 句柄基本参数
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  // 4. 检查备份寄存器 DR0
  // 如果读出来的值是我们设定的魔术字，说明RTC已经在运行，不需要重新初始化时间
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != RTC_BKP_DR_VALID)
  {
    // --- 第一次上电或电池掉电后执行 ---

    // 配置 LSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      // LSE 启动失败处理 (可选：尝试 LSI)
    }

    // 选择 RTC 时钟源
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
    }

    // 初始化 RTC
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
    }

    // 注意：这里不写入魔术字，只有当时间被成功设置（比如NTP成功）后才写入
    // 或者在这里设置一个默认时间（如 2025-01-01）
  }
  else
  {
    // --- RTC 已经在运行 ---
    // 只需要等待同步，不需要重新配置时钟源和时间
    // HAL_RTC_Init 会检查 INITS 标志，如果置位了不会重置时间，但为了保险，我们依靠 BKP 判断

    // 确保 LSE 是开启的 (防止系统复位后时钟配置丢失)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    // 重新初始化句柄以恢复中断等设置，但不会重置时间
    HAL_RTC_Init(&hrtc);

    // 等待 RTC 寄存器同步
    HAL_RTC_WaitForSynchro(&hrtc);
  }
}
