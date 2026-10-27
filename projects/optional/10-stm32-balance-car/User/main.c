#include "stm32f10x.h"                  // Device header
#include "Delay.h"  
#include "LED.h"
#include "Key.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Time.h"
#include "Speed.h"


float Kp = -540 ,Ki= 0,Kd = -1800;
//-900 -3000
//-540 -1800
//float Kp=0,Ki=0,Kd=0;
float VKp =180,VKi = 0.9	;
//float VKp = 0,VKi = 0;
int Mu_Biao_SuDu = 0;

float ZKp = 10,ZKd = 0.6;

//u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
//u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

int main(void)
{
//	JTAG_JieChu();
	OLED_Init();
//	MPU6050_Init();
	Speed_Init();
//	Tim_BIanMaQi_Init();
//	MPU6050_DMP_Init();
	OLED_ShowSignedNum(4,7,MPU6050_DMP_Init(),1);
	
	OLED_ShowString(1,1,"Zuo_S");
	OLED_ShowString(2,1,"Roll");
	OLED_ShowString(3,1,"GYRZ");
	OLED_ShowString(1,9,"You_S");
	OLED_ShowString(4,1,"PWM1");
//	OLED_ShowString(4,10,"PWM2");
	
	Speed_FangXiang(600,600);
	
//	Time_Init();
	while(1)
	{
		OLED_ShowSignedNum(4,1,MPU6050_DMP_Get_Data(&pitch,&roll,&yaw),1);
//			OLED_ShowSignedNum(1,6,Zuo_SuDu,2);
//			OLED_ShowSignedNum(2,6,CeLiang,3);
//			OLED_ShowSignedNum(3,6,gz,3);
//			OLED_ShowSignedNum(1,14,Yuo_SuDu,2);
//			OLED_ShowSignedNum(4,5,PWM1,4);
//		//¶ÁÈ¡×ËÌ¬ÐÅÏ¢
//		OLED_ShowSignedNum(4,1,MPU6050_DMP_Get_Data(&pitch,&roll,&yaw),1);

//		MPU_Get_Gyroscope(&gx,&gy,&gz);
//		MPU_Get_Accelerometer(&ax,&ay,&az);
//		
//		
//		OLED_ShowSignedNum(1,1,pitch,3);
//		OLED_ShowSignedNum(2,1,roll,3);
//		OLED_ShowSignedNum(3,1,yaw,3);
		
		//MPU6050_GetData(&Acc_X,&Acc_Y,&Acc_Z,&Gyro_X,&Gyro_Y,&Gyro_Z);
		
		
	}
}
