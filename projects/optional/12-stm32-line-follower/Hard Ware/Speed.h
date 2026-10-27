#ifndef __SPEED_H
#define __SPEED_H

#define Speed_Zero 0
#define Speed_One 1
#define Speed_Two 2
#define Speed_Three 3
#define Speed_Four 4
#define Speed_Five 5

void Speed_Init(void);

void Qian_Jin(unsigned char Speed_X);

void Hou_Tui(unsigned char Speed_X);

void Zuo_Zhuan(unsigned char Speed_X);

void You_Zhuan(unsigned char Speed_X);

void Zhi_Dong(void);

void Zuo_Qian(unsigned short CCR);

void You_Qian(unsigned short CCR);	

void Zuo_Hou(unsigned short CCR);

void You_Hou(unsigned short CCR);

#endif
