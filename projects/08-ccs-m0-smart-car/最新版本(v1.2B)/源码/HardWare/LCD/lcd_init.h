#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "SYS.h"

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 280

#else
#define LCD_W 280
#define LCD_H 240
#endif

//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_SCL_PIN)//SCL=SCLK
#define LCD_SCLK_Set() DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_SCL_PIN)

#define LCD_MOSI_Clr() DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_SDA_PIN)//SDA=MOSI
#define LCD_MOSI_Set() DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_SDA_PIN)

#define LCD_RES_Clr()  DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_RSE_PIN)//RES
#define LCD_RES_Set()  DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_RSE_PIN)

#define LCD_DC_Clr()   DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_DC_PIN)//DC
#define LCD_DC_Set()   DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_DC_PIN)
 		     
#define LCD_CS_Clr()   DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_CS_PIN)//CS
#define LCD_CS_Set()   DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_CS_PIN)

#define LCD_BLK_Clr()  DL_GPIO_clearPins(GPIO_SPI_PORT,GPIO_SPI_BLK_PIN)//BLK
#define LCD_BLK_Set()  DL_GPIO_setPins(GPIO_SPI_PORT,GPIO_SPI_BLK_PIN)


void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




