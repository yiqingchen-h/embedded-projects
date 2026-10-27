#include "delay.h"
#include "oled.h"
#include "oledfont.h"
#include "stc8g1k08a.h"

#define OLED_ADDRESS                0x78

static void OLED_I2CDelay(void)
{
    unsigned char i;

    for (i = 0U; i < 2U; i++)
    {
    }
}

static void OLED_SCL_High(void)
{
    P32 = 1;
}

static void OLED_SCL_Low(void)
{
    P32 = 0;
}

static void OLED_SDA_High(void)
{
    P33 = 1;
}

static void OLED_SDA_Low(void)
{
    P33 = 0;
}

static void OLED_I2CStart(void)
{
    OLED_SDA_High();
    OLED_SCL_High();
    OLED_I2CDelay();
    OLED_SDA_Low();
    OLED_I2CDelay();
    OLED_SCL_Low();
}

static void OLED_I2CStop(void)
{
    OLED_SDA_Low();
    OLED_SCL_High();
    OLED_I2CDelay();
    OLED_SDA_High();
    OLED_I2CDelay();
}

static void OLED_I2CWriteByte(unsigned char dat)
{
    unsigned char i;

    for (i = 0U; i < 8U; i++)
    {
        OLED_SCL_Low();
        if (dat & 0x80U)
        {
            OLED_SDA_High();
        }
        else
        {
            OLED_SDA_Low();
        }
        dat <<= 1;
        OLED_I2CDelay();
        OLED_SCL_High();
        OLED_I2CDelay();
    }
    OLED_SCL_Low();
}

static void OLED_I2CWaitAck(void)
{
    OLED_SDA_High();
    OLED_I2CDelay();
    OLED_SCL_High();
    OLED_I2CDelay();
    OLED_SCL_Low();
    OLED_I2CDelay();
}

static void OLED_WriteByte(unsigned char dat, bit is_data)
{
    OLED_I2CStart();
    OLED_I2CWriteByte(OLED_ADDRESS);
    OLED_I2CWaitAck();
    if (is_data)
    {
        OLED_I2CWriteByte(0x40U);
    }
    else
    {
        OLED_I2CWriteByte(0x00U);
    }
    OLED_I2CWaitAck();
    OLED_I2CWriteByte(dat);
    OLED_I2CWaitAck();
    OLED_I2CStop();
}

void OLED_SetCursor(unsigned char x, unsigned char page)
{
    OLED_WriteByte((unsigned char)(0xB0U + page), 0);
    OLED_WriteByte((unsigned char)(((x & 0xF0U) >> 4) | 0x10U), 0);
    OLED_WriteByte((unsigned char)(x & 0x0FU), 0);
}

void OLED_Clear(void)
{
    unsigned char page;
    unsigned char col;

    for (page = 0U; page < 4U; page++)
    {
        OLED_SetCursor(0U, page);
        for (col = 0U; col < 128U; col++)
        {
            OLED_WriteByte(0x00U, 1);
        }
    }
}

void OLED_ClearArea(unsigned char x, unsigned char page, unsigned char width)
{
    unsigned char i;

    OLED_SetCursor(x, page);
    for (i = 0U; i < width; i++)
    {
        OLED_WriteByte(0x00U, 1);
    }
}

void OLED_ShowChar(unsigned char x, unsigned char page, char ch)
{
    unsigned char i;
    unsigned char index;

    if (ch < ' ' || ch > '~')
    {
        ch = '?';
    }

    index = (unsigned char)(ch - ' ');
    OLED_SetCursor(x, page);
    for (i = 0U; i < 6U; i++)
    {
        OLED_WriteByte(asc2_0806[index][i], 1);
    }
}

void OLED_ShowString(unsigned char x, unsigned char page, const unsigned char *str)
{
    while ((*str != '\0') && (x <= 122U))
    {
        OLED_ShowChar(x, page, (char)*str);
        x += 6U;
        str++;
    }
}

void OLED_ShowCodeString(unsigned char x, unsigned char page, const unsigned char code *str)
{
    while ((*str != '\0') && (x <= 122U))
    {
        OLED_ShowChar(x, page, (char)*str);
        x += 6U;
        str++;
    }
}

void OLED_ShowNum(unsigned char x, unsigned char page, unsigned int num, unsigned char len)
{
    unsigned char i;
    unsigned int divider;
    unsigned char digit;

    divider = 1U;
    for (i = 1U; i < len; i++)
    {
        divider = (unsigned int)(divider * 10U);
    }

    for (i = 0U; i < len; i++)
    {
        digit = (unsigned char)(num / divider);
        OLED_ShowChar(x, page, (char)('0' + digit));
        x += 6U;
        num = (unsigned int)(num % divider);
        if (divider >= 10U)
        {
            divider = (unsigned int)(divider / 10U);
        }
    }
}

void OLED_ShowHexByte(unsigned char x, unsigned char page, unsigned char value)
{
    unsigned char nibble;

    nibble = (unsigned char)((value >> 4) & 0x0FU);
    OLED_ShowChar(x, page, (char)(nibble < 10U ? ('0' + nibble) : ('A' + nibble - 10U)));
    nibble = (unsigned char)(value & 0x0FU);
    OLED_ShowChar((unsigned char)(x + 6U), page, (char)(nibble < 10U ? ('0' + nibble) : ('A' + nibble - 10U)));
}

void OLED_Init(void)
{

    P3M1 &= (unsigned char)(~0x0CU);
    P3M0 |= 0x0CU;

    P32 = 1;
    P33 = 1;
    DelayMs(100U);

    OLED_WriteByte(0xAEU, 0);
    OLED_WriteByte(0x00U, 0);
    OLED_WriteByte(0x10U, 0);
    OLED_WriteByte(0x40U, 0);
    OLED_WriteByte(0x81U, 0);
    OLED_WriteByte(0xCFU, 0);
    OLED_WriteByte(0xA1U, 0);
    OLED_WriteByte(0xC8U, 0);
    OLED_WriteByte(0xA6U, 0);
    OLED_WriteByte(0xA8U, 0);
    OLED_WriteByte(0x1FU, 0);
    OLED_WriteByte(0xD3U, 0);
    OLED_WriteByte(0x00U, 0);
    OLED_WriteByte(0xD5U, 0);
    OLED_WriteByte(0x80U, 0);
    OLED_WriteByte(0xD9U, 0);
    OLED_WriteByte(0xF1U, 0);
    OLED_WriteByte(0xDAU, 0);
    OLED_WriteByte(0x02U, 0);
    OLED_WriteByte(0xDBU, 0);
    OLED_WriteByte(0x40U, 0);
    OLED_WriteByte(0x8DU, 0);
    OLED_WriteByte(0x14U, 0);
    OLED_WriteByte(0xAFU, 0);

    OLED_Clear();
}
