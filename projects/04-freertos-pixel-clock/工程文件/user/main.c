#include "main.h"
//#include "lcd_init.h"
//#include "lcd.h"
//#include "pic.h"

int main(void)
{
  HAL_Init();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  SystemClock_Config(); // 主频 168MHz
  RTC_Config();
  UART1_Init(115200);
  UART2_Init(115200);
  UART3_Init(115200);
  U1_print("sysclock:%d\r\n", HAL_RCC_GetSysClockFreq()); // 打印主频
  IIC1_Init();
  LED_Init();
  BSP_Key_Init();
  Fan_PWM_Init();
  if (AT24C512_Check())
  { // 检查 24C512
    U1_print("24C512_no_online\r\n");
  }
  else
  {
    uint8_t saved_brightness = 0;
    AT24C512_ReadBuffer(0x0010, &saved_brightness, 1);
    // 校验亮度参数
    if (saved_brightness > 100)
    {
      // 用默认亮度
      saved_brightness = 30;
      AT24C512_WriteBuffer(0x0010, &saved_brightness, 1);
    }
    luminance_set = saved_brightness;
  }
  if (AHT20_Init())
  { // 检查 AHT20
    U1_print("AHT20_init_fail\r\n");
  }
  // 初始化灯板
  WS2812_Init();
  // 映射实际亮度
  uint8_t hw_brightness = 10 + (luminance_set * 190) / 100;
  // 设置亮度
  WS2812_SetBrightness(hw_brightness);
  WS2812_Clear();
  HAL_Delay(10);
  // WS2812_DrawImage(0, 1, 8, 8, Icon_Heart, COLOR_RED);
  // WS2812_ShowString(9, 1, "main", COLOR_WHITE, COLOR_BLACK);
  test_WS2812B();
  WS2812_Update();
  HAL_Delay(100);
//  LCD_Init(); // LCD 调试先留着
//  LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
//  LCD_ShowString(0, 40, "LCD_W:", RED, WHITE, 16, 0);
//  LCD_ShowString(80, 40, "LCD_H:", RED, WHITE, 16, 0);
//  LCD_ShowString(80, 40, "LCD_H:", RED, WHITE, 16, 0);
//  LCD_ShowString(0, 70, "Increaseing Nun:", RED, WHITE, 16, 0);

  freertos_start(); // 启动任务
  while (1)
  {
    // 串口1转发
    if (Uart1_Prt.RxOutptr != Uart1_Prt.RxInptr)
    {
      UART1_Tx_Data(Uart1_Prt.RxOutptr->start, (Uart1_Prt.RxOutptr->end - Uart1_Prt.RxOutptr->start + 1));
      Uart1_Prt.RxOutptr++;
      if (Uart1_Prt.RxOutptr == Uart1_Prt.RxEndptr)
      {
        Uart1_Prt.RxOutptr = &Uart1_Prt.RxLocation[0];
      }
    }
    if ((Uart1_Prt.TxOutptr != Uart1_Prt.TxInptr) && (Uart1_Prt.TxStater != 1))
    {
      Uart1_Prt.TxStater = 1;
      HAL_UART_Transmit_IT(&UART_HandleType, Uart1_Prt.TxOutptr->start, (Uart1_Prt.TxOutptr->end - Uart1_Prt.TxOutptr->start + 1));
      Uart1_Prt.TxOutptr++;
      if (Uart1_Prt.TxOutptr == Uart1_Prt.TxEndptr)
      {
        Uart1_Prt.TxOutptr = &Uart1_Prt.TxLocation[0];
      }
    }
    // 串口2转发
    if (Uart2_Prt.RxOutptr != Uart2_Prt.RxInptr)
    {
      UART2_Tx_Data(Uart2_Prt.RxOutptr->start, (Uart2_Prt.RxOutptr->end - Uart2_Prt.RxOutptr->start + 1));
      Uart2_Prt.RxOutptr++;
      if (Uart2_Prt.RxOutptr == Uart2_Prt.RxEndptr)
      {
        Uart2_Prt.RxOutptr = &Uart2_Prt.RxLocation[0];
      }
    }
    if ((Uart2_Prt.TxOutptr != Uart2_Prt.TxInptr) && (Uart2_Prt.TxStater != 1))
    {
      Uart2_Prt.TxStater = 1;
      HAL_UART_Transmit_IT(&UART_HandleType2, Uart2_Prt.TxOutptr->start, (Uart2_Prt.TxOutptr->end - Uart2_Prt.TxOutptr->start + 1));
      Uart2_Prt.TxOutptr++;
      if (Uart2_Prt.TxOutptr == Uart2_Prt.TxEndptr)
      {
        Uart2_Prt.TxOutptr = &Uart2_Prt.TxLocation[0];
      }
    }
  }
}
