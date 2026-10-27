#ifndef __OLED_H
#define __OLED_H
/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define OLED_8X16 8
#define OLED_6X8  6
void OLED_Init(void);
// 设置光标位置 - 屏幕的X轴和Y轴
void OLED_SetCursor(uint8_t Page, uint8_t X);

void OLED_WriteData(unsigned char Data);

// 显示单字符
void OLED_ShowChar(unsigned char x, unsigned char page, char Char, unsigned char Flog);

//显示字符串
void OLED_ShowString(unsigned char x, unsigned char page, char *String, unsigned char Flag);

/*
unsigned char x  X轴位置0-127
unsigned char page  页位置（行）0-7
int metrication     要显示的十进制数
unsigned char Long  十进制数长度
unsigned char Flag  显示大小 - 6（6 * 8） -8 （8 * 16）
*/
// 显示十进制数
void OLED_ShouMetrication(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag);

//显示十六进制数
void OLED_ShouHexadecimal(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag);

// 显示二进制数
void OLED_ShouBinary(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag);
#endif