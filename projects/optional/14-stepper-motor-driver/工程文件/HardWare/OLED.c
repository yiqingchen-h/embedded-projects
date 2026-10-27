#include "stm32f1xx_hal.h"
#include "IIC.h"
//#include "UART.h"
#include "OLED_Data.h"
// 写命令
void OLED_WriteCommand(uint8_t Command)
{
    uint8_t cmd[2] = {0x00, Command};
    uint8_t retry  = 3;
    while (retry--) {
        //定义状态结构体 接收传输是否成功 若传输失败则再次发送 共尝试3次
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&I2C_HandleType_IIC1, 0x78, cmd, 2,100);
        if (status == HAL_OK) break;
        HAL_Delay(1); // 重试前短暂等待
    }
    if (retry == 255) {
        //U1_print("Command_Tx_Off");
    }
}

// 写数据
void OLED_WriteData(unsigned char Data)
{
    uint8_t cmd[2] = {0x40, Data};
    uint8_t retry  = 3;
    while (retry--) {
        // 定义状态结构体 接收传输是否成功 若传输失败则再次发送 共尝试3次
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&I2C_HandleType_IIC1, 0x78, cmd, 2,100);
        if (status == HAL_OK) break;
        HAL_Delay(1); // 重试前短暂等待
    }
    if(retry == 255){
        //U1_print("Data_Tx_Off");
    }
}
void OLED_Clear(void);
void OLED_Init(void)
{
    IIC1_Init();
    /*写入一系列的命令，对OLED进行初始化配置*/
    OLED_WriteCommand(0xAE); // 设置显示开启/关闭，0xAE关闭，0xAF开启

    OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80); // 0x00~0xFF

    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F); // 0x0E~0x3F

    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00); // 0x00~0x7F

    OLED_WriteCommand(0x40); // 设置显示开始行，0x40~0x7F

    OLED_WriteCommand(0xA1); // 设置左右方向，0xA1正常，0xA0左右反置

    OLED_WriteCommand(0xC8); // 设置上下方向，0xC8正常，0xC0上下反置

    OLED_WriteCommand(0xDA); // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81); // 设置对比度
    OLED_WriteCommand(0xCF); // 0x00~0xFF

    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB); // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4); // 设置整个显示打开/关闭

    OLED_WriteCommand(0xA6); // 设置正常/反色显示，0xA6正常，0xA7反色

    OLED_WriteCommand(0x8D); // 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF); // 开启显示
    //清屏
    OLED_Clear();
}


//设置光标位置 - 屏幕的X轴和Y轴
/**
 * 函    数：OLED设置显示光标位置
 * 参    数：Page 指定光标所在的页，范围：0~7
 * 参    数：X 指定光标所在的X轴坐标，范围：0~127
 * 返 回 值：无
 * 说    明：OLED默认的Y轴，只能8个Bit为一组写入，即1页等于8个Y轴坐标
 */
void OLED_SetCursor(uint8_t X, uint8_t Page)
{
    /*如果使用此程序驱动1.3寸的OLED显示屏，则需要解除此注释*/
    /*因为1.3寸的OLED驱动芯片（SH1106）有132列*/
    /*屏幕的起始列接在了第2列，而不是第0列*/
    /*所以需要将X加2，才能正常显示*/
    //	X += 2;

    /*通过指令设置页地址和列地址*/
    OLED_WriteCommand(0xB0 | Page);              // 设置页位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); // 设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));        // 设置X位置低4位
}
//清空屏幕
void OLED_Clear(void)
{
    for (unsigned char f = 0; f < 8; f++) {
        OLED_SetCursor(0, f);
        for (unsigned char i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

//显示单字符
void OLED_ShowChar(unsigned char x, unsigned char page, char Char,unsigned char Flag)
{
    if( Flag == 6)
    {
        OLED_SetCursor(x, page);
        for (unsigned char i = 0; i < 6; i++) {
            OLED_WriteData(OLED_F6x8[Char - ' '][i]);
        }
    }
    else if (Flag == 8)
    {
        OLED_SetCursor(x, page);
        for (unsigned char i = 0; i < 8; i++) {
            OLED_WriteData(OLED_F8x16[Char - ' '][i]);
        }
        OLED_SetCursor(x, page + 1);
        for (unsigned char i = 0; i < 8; i++) {
            OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
        }
    }
    
}
//字符串显示
void OLED_ShowString(unsigned char x, unsigned char page, char* String, unsigned char Flag)
{
    for (unsigned char i = 0; String[i] != '\0';i++)
    {
        OLED_ShowChar(x + i * Flag, page, String[i], Flag);
    }
}
//取十进制数位
unsigned char bit_metrication(long long x,unsigned char size)
{
    unsigned int j = 1;
    for (unsigned char i = size ; i > 0; i--)
    {
        j *= 10;
    }
    x %= j;
    return x /( j / 10);
}
//十进制数显示
void OLED_ShouMetrication(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag)
{
    if (metrication >= 0) {
        OLED_ShowChar(x , page, '+', Flag);
    }
    else
    {
        OLED_ShowChar(x, page, '-', Flag);
    }
    for (unsigned char i = Long; i > 0; i--) {
        OLED_ShowChar(x + (Long - i + 1) * Flag, page, (char)bit_metrication(metrication, i) + '0', Flag);
    }
}
#include <string.h>
//二进制数提取位
long long bit_Binary(int x)
{
        long long binaryNumber = 0;
        int remainder, i = 1;

        while (x != 0) {
            remainder = x % 2;
            x /= 2;
            binaryNumber += remainder * i;
            i *= 10;
        }
        return binaryNumber;
}
//二进制转10进制
#include <math.h>
int convertBinaryToDecimal(long long n)
{
    int decimalNumber = 0, i = 0, remainder;
    while (n != 0) {
        remainder = n % 10;
        n /= 10;
        decimalNumber += remainder * pow(2, i);
        ++i;
    }
    return decimalNumber;
}
unsigned char bit_Hexadecimal(int x,unsigned char i)
{
    unsigned char Bit = (convertBinaryToDecimal(bit_Binary(x)) >> (4 * i)) & 0x0F;
    switch(Bit)
    {
        case 10:
            return 'A';
        case 11:
            return 'B';
        case 12:
            return 'C';
        case 13:
            return 'D';
        case 14:
            return 'E';
        case 15:
            return 'F';
        default:
            return Bit + '0';
    }
}
    // 十六进制数显示
void OLED_ShouHexadecimal(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag)
{
    for (unsigned char i = Long; i > 0; i--) {
        OLED_ShowChar(x + i * Flag, page, bit_Hexadecimal(metrication , Long - i), Flag);
    }
}
// 二进制数显示
void OLED_ShouBinary(unsigned char x, unsigned char page, int metrication, unsigned char Long, unsigned char Flag)
{
    long long Ling = bit_Binary(metrication);
    for (unsigned char i = Long; i > 0; i--) {
        OLED_ShowChar(x + (Long - i) * Flag, page, bit_metrication(Ling, i) + '0', Flag);
    }
}
