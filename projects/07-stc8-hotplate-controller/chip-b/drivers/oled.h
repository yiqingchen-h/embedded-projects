#ifndef OLED_H
#define OLED_H

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearArea(unsigned char x, unsigned char page, unsigned char width);
void OLED_SetCursor(unsigned char x, unsigned char page);
void OLED_ShowChar(unsigned char x, unsigned char page, char ch);
void OLED_ShowString(unsigned char x, unsigned char page, const unsigned char *str);
void OLED_ShowCodeString(unsigned char x, unsigned char page, const unsigned char code *str);
void OLED_ShowNum(unsigned char x, unsigned char page, unsigned int num, unsigned char len);
void OLED_ShowHexByte(unsigned char x, unsigned char page, unsigned char value);

#endif /* OLED_H */
