#include "IIC.h"

I2C_HandleTypeDef I2C_HandleType_IIC1;

/* 定义一个互斥锁句柄 */
SemaphoreHandle_t IIC1_MutexHandle = NULL;

void IIC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 1. 创建互斥锁 */
  /**
   * @brief 创建动态互斥信号量
   * 返回值：句柄
   * 在使用信号量时可以防止 高低优先级任务同时访问信号量时，当低优先级任务获取信号量时会继承高优先级任务的优先级，释放信号量时优先级恢复
   * 可以避免低优先级任务获取信号量被中优先级任务打断，使高优先级任务等待。发生优先级翻转问题
   */
  if (IIC1_MutexHandle == NULL)
  {
    IIC1_MutexHandle = xSemaphoreCreateMutex();
  }

  /* 2. 硬件初始化 (保持不变) */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitStruct.Pin = PIN_SCL | PIN_SDA;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOx_I2C, &GPIO_InitStruct);

  I2C_HandleType_IIC1.Instance = I2C1;
  I2C_HandleType_IIC1.Init.ClockSpeed = 100000;
  I2C_HandleType_IIC1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_HandleType_IIC1.Init.OwnAddress1 = 0;
  I2C_HandleType_IIC1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_HandleType_IIC1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C_HandleType_IIC1.Init.OwnAddress2 = 0;
  I2C_HandleType_IIC1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C_HandleType_IIC1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&I2C_HandleType_IIC1) != HAL_OK)
  {
    // Error_Handler();
  }
}

/* ================= 线程安全的封装函数 ================= */

/* 辅助函数：获取锁 */
static void IIC_Lock(void)
{
  if (IIC1_MutexHandle != NULL)
  {
    // 等待锁- 一直等待
    xSemaphoreTake(IIC1_MutexHandle, pdMS_TO_TICKS(portMAX_DELAY));
  }
}

/* 辅助函数：释放锁 */
static void IIC_Unlock(void)
{
  if (IIC1_MutexHandle != NULL)
  {
    xSemaphoreGive(IIC1_MutexHandle);
  }
}

/* 1. 写寄存器封装 (用于 AT24C512 写, AHT20 发命令) */
uint8_t IIC_Write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;

  IIC_Lock(); // 拿锁
  status = HAL_I2C_Mem_Write(&I2C_HandleType_IIC1, DevAddress, MemAddress, MemAddSize, pData, Size, 100);
  IIC_Unlock(); // 还锁

  return (status == HAL_OK) ? 0 : 1;
}

/* 2. 读寄存器封装 (用于 AT24C512 读) */
uint8_t IIC_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;

  IIC_Lock(); // 拿锁
  status = HAL_I2C_Mem_Read(&I2C_HandleType_IIC1, DevAddress, MemAddress, MemAddSize, pData, Size, 100);
  IIC_Unlock(); // 还锁

  return (status == HAL_OK) ? 0 : 1;
}

/* 3. 直接接收封装 (专门用于 AHT20 读取数据) */
uint8_t IIC_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef status;

  IIC_Lock(); // 拿锁
  status = HAL_I2C_Master_Receive(&I2C_HandleType_IIC1, DevAddress, pData, Size, 100);
  IIC_Unlock(); // 还锁

  return (status == HAL_OK) ? 0 : 1;
}
