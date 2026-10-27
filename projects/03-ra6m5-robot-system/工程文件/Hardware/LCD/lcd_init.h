#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "hal_data.h"
#include "SPI.h"
#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif




//-----------------LCD端口定义---------------- 

#define LCD_RES_Clr()  g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_05, 0);
#define LCD_RES_Set()  g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_05, 1);

#define LCD_DC_Clr()   g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_04, 0);
#define LCD_DC_Set()   g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_04, 1);
 		     
#define LCD_CS_Clr()   g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_03, 0);
#define LCD_CS_Set()   g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_01_PIN_03, 1);

#define LCD_BLK_Clr()  g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_06_PIN_08, 0);
#define LCD_BLK_Set()  g_ioport.p_api->pinWrite(g_ioport.p_ctrl, BSP_IO_PORT_06_PIN_08, 1);



void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
//void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化

void LCD_WR_DATA(uint16_t dat);
void LCD_Write_Buffer(uint8_t *buf, uint32_t len);
void LCD_Write_Bus_Large(uint8_t *buf, uint32_t len);

#endif




