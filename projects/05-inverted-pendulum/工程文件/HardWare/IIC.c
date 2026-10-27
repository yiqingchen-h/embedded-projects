#include "stm32f1xx_hal.h"
#include "UART.h"
#include "IIC.h"

I2C_HandleTypeDef I2C_HandleType_IIC1;

unsigned char T_Data[10] = {0xac, 0x33, 0x00};
unsigned char R_Data[10];

void IIC1_Init(void)
{
    I2C_HandleType_IIC1.Instance = I2C1;        //使用IIC1
    I2C_HandleType_IIC1.Init.ClockSpeed = 100000;   //设置通信速率
    I2C_HandleType_IIC1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  //设置寻址模式

    HAL_I2C_Init(&I2C_HandleType_IIC1);
}


//IIC初始化硬件回调函数
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_AFIO_CLK_ENABLE();    //重映射时需要开启
        __HAL_AFIO_REMAP_I2C1_ENABLE(); // 开启IIC1的重映射
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin   = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitType.Mode  = GPIO_MODE_AF_OD;  //复用开漏
        GPIO_InitType.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(GPIOB, &GPIO_InitType);

        // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 4, 0);
        // 使能中断
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    }
}


//中断发送完成回调函数
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    U1_print("Tx_Ok \t\n");
}
//中断接收完成回调函数
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    U1_print("Rx_Ok \t\n");
}

void set_byte_IIC(unsigned char address, unsigned char Byte)
{
    unsigned char x[1];
    x[0] = Byte;
    HAL_I2C_Master_Transmit_IT(&I2C_HandleType_IIC1, address, x, 1);
}

void get_byte_IIC(unsigned char address, unsigned char length)
{
    HAL_I2C_Master_Receive_IT(&I2C_HandleType_IIC1, address, R_Data, length);
}
