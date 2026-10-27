#include "stm32f10x.h"                  // Device header
#include "Delay.h"  
#include "Key.h"
#include "OLED.h"
#include "Servo.h"
#include "PWM.h"
#include "Serial.h"
//#include "board.h"
//#include "stdio.h"
//#include "bsp_uart.h"
#include "bsp_VL53L0X.h"


extern VL53L0X_Dev_t vl53l0x_dev;//设备I2C数据参数

int main(void)
{
//		board_init();        // 开发板初始化
		Serial_Init();
	Serial_SendByte(1);
		PWM_Init();
		OLED_Init();
		OLED_ShowString(1,1,"A");
//			uart1_init(115200);        // 串口1波特率115200

			uint8_t mode = 3;//0：默认;1:高精度;2:长距离;3:高速
			VL53L0X_Error Status=VL53L0X_ERROR_NONE;//工作状态

			printf("Start\r\n");
            OLED_ShowNum(2,2,1,1);
    		while(vl53l0x_init(&vl53l0x_dev))//vl53l0x初始化
			{
				OLED_ShowNum(1,2,3,1);
			//	printf("VL53L0X Error!!!\n\r");
				OLED_ShowString(1,1,"VL53L0X Error!!!");		
				Delay_ms(500);
			}
//			printf("VL53L0X OK\r\n");
			OLED_ShowString(1,1,"OK");

			while(vl53l0x_set_mode(&vl53l0x_dev,mode))//配置测量模式
			{
			//	printf("Mode Set Error\r\n");
				OLED_ShowString(1,1,"Mode Set Erro");
				Delay_ms(500);
			}

			while(1)
			{
				 if(Status==VL53L0X_ERROR_NONE)
				 {
						//执行单次测距并获取测距测量数据
						Status = VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &vl53l0x_data);
				//		printf("d: %4imm\r\n",vl53l0x_data.RangeMilliMeter);//打印测量距离
					    OLED_ShowNum(2,1,vl53l0x_data.RangeMilliMeter,4);
				}
				else
				{
				//   printf("error\r\n");
					OLED_ShowString(1,1,"Erroer");
				}
				Delay_ms(500);
			}
		
}
