#include "stm32f10x.h"                  // Device header
#include "Delay.h"  
#include "LED.h"
#include "Key.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Time.h"
#include "LED.h"
#include "PWM.h"
float Kp ,Ki ,Kd ;

float VKp ,VKi ;

extern int Xun_Ji_GeShu;

extern float pitch,roll,yaw;//¸©Ñö½Ç£¬·­¹ö½Ç£¬Æ«º½½Ç

//u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

int main(void)
{
	PWM_Init();
//	OLED_Init();
//	MPU6050_Init();
//	MPU6050_DMP_Init();
//	Time_Init();
//	Hui_Du_Init();
	PWM_OC2_CCR(800);
	PWM3_OC2_CCR(800);
	while(1)
	{
		//¶ÁÈ¡×ËÌ¬ÐÅÏ¢
//		MPU6050_DMP_Get_Data(&pitch,&roll,&yaw);
//		MPU_Get_Gyroscope(&gx,&gy,&gz);
//		MPU_Get_Accelerometer(&ax,&ay,&az);
		
		
//		OLED_ShowSignedNum(2,1,pitch,3);
//		OLED_ShowSignedNum(3,1,roll,3);
//		OLED_ShowSignedNum(4,1,yaw,3);
		OLED_ShowString(1,1,"Xun:");
		OLED_ShowBinNum(1,6,Xun_Ji_GeShu,8);
		//MPU6050_GetData(&Acc_X,&Acc_Y,&Acc_Z,&Gyro_X,&Gyro_Y,&Gyro_Z);
		
		
	}
}
