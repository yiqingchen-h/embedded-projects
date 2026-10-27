#include "Re.h"
#include "lcd.h"
#include "lcd_init.h"
int Re_Flag = 0;
unsigned char Ping_A_25_state = 0;
unsigned char Before_Re_Ping_14 = 1;
char Get_Re_Data(void)
{
    if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN) & GPIO_Re_PIN_14_PIN) && Before_Re_Ping_14 == 0)
    {
        delay_us(100);
        if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_16_PIN) & GPIO_Re_PIN_16_PIN) == 0)
        {
            Before_Re_Ping_14 = 1;
            return 1;
        }
    }
    if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN) & GPIO_Re_PIN_14_PIN) && Before_Re_Ping_14 == 0)
    {
        delay_us(100);
        if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_16_PIN) & GPIO_Re_PIN_16_PIN))
        {
            Before_Re_Ping_14 = 1;
            return -1;
        }
    }
    //判断上升沿
    Before_Re_Ping_14 = ((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN) & GPIO_Re_PIN_14_PIN) >> 14);
    return 0;
}

int Get_Re (void)
{
    int i = Re_Flag;
    Re_Flag = 0;
    return i;
}

void GROUP1_IRQHandler(void)
{
   
// 获取那些GPIO触发了中断
     uint32_t GpioA = DL_GPIO_getEnabledInterruptStatus(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN | GPIO_Re_PIN_16_PIN | GPIO_Re_PIN_25_PIN);
     //判断是否是当前引脚触发
     if((GpioA & GPIO_Re_PIN_14_PIN) == GPIO_Re_PIN_14_PIN)
     {
        //delay_us(100);
         if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_16_PIN) & GPIO_Re_PIN_16_PIN))
        {
            Re_Flag -= 1;
        }
            //清楚未处理的中断
            DL_GPIO_clearInterruptStatus(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN); 
     }
      if((GpioA & GPIO_Re_PIN_16_PIN) == GPIO_Re_PIN_16_PIN)
     {
         //delay_us(100);
         if((DL_GPIO_readPins(GPIO_Re_PORT,GPIO_Re_PIN_14_PIN) & GPIO_Re_PIN_14_PIN))
        {
            Re_Flag += 1;
        }
            //清楚未处理的中断
            DL_GPIO_clearInterruptStatus(GPIO_Re_PORT,GPIO_Re_PIN_16_PIN); 
     }

     if((GpioA & GPIO_Re_PIN_25_PIN) == GPIO_Re_PIN_25_PIN)
     {
        Ping_A_25_state = 1;
            //清楚未处理的中断
        DL_GPIO_clearInterruptStatus(GPIO_Re_PORT,GPIO_Re_PIN_25_PIN); 
     }

    // 获取那些GPIO触发了中断
     uint32_t GpioB = DL_GPIO_getEnabledInterruptStatus(Switch_PORT,Switch_K1_PIN | Switch_K2_PIN |Switch_K3_PIN);
     //判断是否是当前引脚触发
     if((GpioB & Switch_K1_PIN) == Switch_K1_PIN)
     {
			switch_Key = 1;
            //清楚未处理的中断
            DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K1_PIN); 
     }
     if((GpioB & Switch_K2_PIN) == Switch_K2_PIN)
     {
			switch_Key = 2;
            //清楚未处理的中断
            DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K2_PIN); 
     }
	 if((GpioB & Switch_K3_PIN) == Switch_K3_PIN)
     {
			switch_Key = 3;
            //清楚未处理的中断
            DL_GPIO_clearInterruptStatus(Switch_PORT,Switch_K3_PIN); 
     }


}

unsigned char Get_Re_Ping (void)
{
    unsigned char i = Ping_A_25_state;
    Ping_A_25_state = 0;
    return i;
}

