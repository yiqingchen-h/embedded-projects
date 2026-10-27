#ifndef __IIC_H
#define __IIC_H

extern I2C_HandleTypeDef I2C_HandleType_IIC1;
extern unsigned char T_Data[10];
extern unsigned char R_Data[10];
void IIC1_Init(void);

//发送一个字节数据
void set_byte_IIC(unsigned char address, unsigned char Byte);

//接收一个字节数据
void get_byte_IIC(unsigned char address, unsigned char Byte);


#endif
