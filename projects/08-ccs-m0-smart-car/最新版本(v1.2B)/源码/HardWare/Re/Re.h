#ifndef __RE_H
#define __RE_H		
#include "SYS.h"

extern int Re_Flag;

//读取旋转编码器数据
//阻塞式
char Get_Re_Data(void);
//中断读取
int Get_Re (void);
//读取编码器按键按下
unsigned char Get_Re_Ping (void);
#endif



