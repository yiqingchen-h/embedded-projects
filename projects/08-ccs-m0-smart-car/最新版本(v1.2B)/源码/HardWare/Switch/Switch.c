#include "Switch.h"
unsigned char Get_Switch_state(void)
{
    if(DL_GPIO_readPins(Switch_PORT , Switch_K1_PIN))
	{
		while(DL_GPIO_readPins(Switch_PORT , Switch_K1_PIN));
		return 1;
	}
    if(DL_GPIO_readPins(Switch_PORT , Switch_K2_PIN))
	{
		while(DL_GPIO_readPins(Switch_PORT , Switch_K2_PIN));
		return 2;
	}
    if(DL_GPIO_readPins(Switch_PORT , Switch_K3_PIN))
	{
		while(DL_GPIO_readPins(Switch_PORT , Switch_K3_PIN));
		return 3;
	}
	return 0;
}
unsigned char switch_Key = 0;
unsigned char Get_switch (void)
{
	unsigned x = switch_Key;
	switch_Key = 0;
	return x;
}
//GPIO_中断处理函数  -- 在Re.c中
// void GROUP1_IRQHandler(void)
// {
//     // 获取那些GPIO触发了中断
//      uint32_t GpioB = DL_GPIO_getEnabledInterruptStatus(Switch_PORT,Switch_K1_PIN | Switch_K2_PIN |Switch_K3_PIN);
//      //判断是否是当前引脚触发
//      if((GpioB & Switch_K1_PIN) == Switch_K1_PIN)
//      {
// 			Key = 1;
//             //清楚未处理的中断
//             DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K1_PIN); 
//      }
//      if((GpioB & Switch_K2_PIN) == Switch_K2_PIN)
//      {
// 			Key = 2;
//             //清楚未处理的中断
//             DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K2_PIN); 
//      }
// 	 if((GpioB & Switch_K3_PIN) == Switch_K3_PIN)
//      {
// 			Key = 3;
//             //清楚未处理的中断
//             DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K3_PIN); 
//      }
// }


