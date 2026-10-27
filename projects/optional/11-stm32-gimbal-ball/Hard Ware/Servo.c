#include "stm32f10x.h"                  // Device header
#include "PWM.h"
void Sever_Init(void)
{
	PWM_Init();
}

void Sever_SetAngle(unsigned short Angle)
{
	PWM_OC2_CCR(Angle / 180 * 2000 + 500);
}

