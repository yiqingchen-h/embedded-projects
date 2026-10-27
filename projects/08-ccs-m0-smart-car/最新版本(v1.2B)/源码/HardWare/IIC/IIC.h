#ifndef __IIC_H
#define __IIC_H		
#include "SYS.h"
#include "stdio.h"
#include "stdint.h"
 
// #define GPIO_sda_PIN_0_PIN Software_IIC_My_SDA_PIN
// #define GPIO_scl_PIN_1_PIN Software_IIC_My_SCL_PIN
// #define GPIO_sda_PORT Software_IIC_PORT
// #define GPIO_scl_PORT Software_IIC_PORT
// #define GPIO_sda_PIN_0_IOMUX Software_IIC_My_SDA_IOMUX

// void MyI2C_Init(void);
// void MyI2C_Start(void);
// void MyI2C_Stop(void);
// void MyI2C_SendByte(uint8_t Byte);
// uint8_t MyI2C_R_SDA(void);
// uint8_t MyI2C_ReceiveByte(void);
// void MyI2C_SendAck(uint8_t AckBit);
// uint8_t MyI2C_ReceiveAck(void);
// void MyI2C_W_SCL(uint8_t BitValue);
// void MyI2C_W_SDA(uint8_t BitValue);
// void Delay_us(uint16_t us);
 
#define AHT10_ID 0x38
float AHT10_Temperature(void);
float AHT10_Humidity(void);


#define Grayscale_ID 0x4C
#define get_Ping 0x00
#define get_Digital_Output 0x01
#define One_Channel_Analog 0x02


unsigned char Ping(void);
unsigned char get_Digital_Output_Grayscale(void);
// 读取单通道模拟量灰度
// Channel : 通道号 - 1 - 8； 
unsigned char get_One_Channel_Analog_Output_Grayscale(unsigned char Channel);
// 读取多通道模拟量灰度
/*
Channel : 要使能的通道
data ：存放数据位置
size ：启用通道数量
*/
void get_More_Channel_Analog_Output_Grayscale(unsigned char Channel , unsigned char data[],unsigned char size);


#endif

