#include "key.h"

/* ================= 公共变量与定义 ================= */

static volatile int32_t Encoder_Count = 0;
static uint8_t Enc_A_Last = 0;

// 硬件引脚定义
#define WKUP_PIN GPIO_PIN_0
#define WKUP_PORT GPIOA
#define ENC_A_PIN GPIO_PIN_2
#define ENC_B_PIN GPIO_PIN_3
#define ENC_BTN_PIN GPIO_PIN_4
#define ENC_PORT GPIOE

// PC0-PC3 数组，方便循环
static GPIO_TypeDef *PC_Port = GPIOC;
static uint16_t PC_Pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
static Key_ID_t PC_IDs[] = {KEY_PC1, KEY_PC2, KEY_PC3, KEY_PC4};

/* ================= 硬件初始化 (公共) ================= */

void BSP_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 开启时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    // 2. WKUP (PA0) - 中断, 上升沿
    GPIO_InitStruct.Pin = WKUP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(WKUP_PORT, &GPIO_InitStruct);

    // 3. 编码器 (PE2, PE3) - 中断, 双边沿
    GPIO_InitStruct.Pin = ENC_A_PIN | ENC_B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_PORT, &GPIO_InitStruct);

    Enc_A_Last = HAL_GPIO_ReadPin(ENC_PORT, ENC_A_PIN);

    // 4. PC0-PC3 - 输入, 轮询
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 5. PE4 (编码器按键) - 输入, 轮询
    GPIO_InitStruct.Pin = ENC_BTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_PORT, &GPIO_InitStruct);

    // 6. NVIC 配置 (WKUP & Encoder)
    // PA0 -> EXTI0
    HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    // PE2 -> EXTI2
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    // PE3 -> EXTI3
    HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

#if KEY_USE_FREERTOS
    // FreeRTOS 队列创建
    if (Key_QueueHandle == NULL)
    {
        Key_QueueHandle = xQueueCreate(20, sizeof(Key_ID_t));
    }
#endif
}

int32_t BSP_Encoder_GetCount(void)
{
    return Encoder_Count;
}

/* ================================================================= */
/*                           FreeRTOS 实现部分                        */
/* ================================================================= */
#if KEY_USE_FREERTOS

QueueHandle_t Key_QueueHandle = NULL;

// --- FreeRTOS 内部函数: 发送事件 ---
static void FreeRTOS_SendEvent(Key_ID_t key)
{
    if (Key_QueueHandle == NULL)
        return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (__get_IPSR())
    { // 中断中
        xQueueSendFromISR(Key_QueueHandle, &key, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    { // 任务中
        xQueueSend(Key_QueueHandle, &key, 0);
    }
}

// --- 接口实现: 获取消息 ---
BaseType_t BSP_Key_GetMessage(Key_ID_t *key, uint32_t timeout)
{
    if (Key_QueueHandle == NULL)
        return pdFALSE;
    return xQueueReceive(Key_QueueHandle, key, timeout);
}

// --- 接口实现: 轮询任务 ---
void BSP_Key_Poll_Task(void *argument)
{
    for (;;)
    {
        // 1. 扫描 PC0-PC3 (Active High)
        for (int i = 0; i < 4; i++)
        {
            if (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
            {
                vTaskDelay(pdMS_TO_TICKS(KEY_DEBOUNCE_TIME)); // 阻塞消抖
                if (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
                {
                    FreeRTOS_SendEvent(PC_IDs[i]);
                    // 等待释放
                    while (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
                    {
                        vTaskDelay(10);
                    }
                }
            }
        }

        // 2. 扫描 PE4 (Active Low)
        if (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
        {
            vTaskDelay(pdMS_TO_TICKS(KEY_DEBOUNCE_TIME));
            if (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
            {
                FreeRTOS_SendEvent(KEY_ENC_BTN);
                while (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
                {
                    vTaskDelay(10);
                }
            }
        }

        vTaskDelay(10); // 轮询周期
    }
}

// --- 中断回调 (FreeRTOS版) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // WKUP
    if (GPIO_Pin == WKUP_PIN)
    {
        if (HAL_GPIO_ReadPin(WKUP_PORT, WKUP_PIN) == GPIO_PIN_SET)
        {
            FreeRTOS_SendEvent(KEY_WKUP);
        }
    }
    // Encoder
    if (GPIO_Pin == ENC_A_PIN || GPIO_Pin == ENC_B_PIN)
    {
        uint8_t A_Curr = HAL_GPIO_ReadPin(ENC_PORT, ENC_A_PIN);
        uint8_t B_Curr = HAL_GPIO_ReadPin(ENC_PORT, ENC_B_PIN);
        if (A_Curr != Enc_A_Last)
        {
            if (A_Curr == 0)
            { // 下降沿
                if (B_Curr == 0)
                {
                    Encoder_Count++;
                    FreeRTOS_SendEvent(KEY_ENC_CW);
                }
                else
                {
                    Encoder_Count--;
                    FreeRTOS_SendEvent(KEY_ENC_CCW);
                }
            }
            Enc_A_Last = A_Curr;
        }
    }
}

/* ================================================================= */
/*                            裸机 实现部分                           */
/* ================================================================= */
#else

// 裸机需要一个简单的FIFO来缓存中断产生的按键，否则主循环在做别的事时会丢失编码器事件
#define KEY_FIFO_SIZE 16
typedef struct
{
    Key_ID_t buf[KEY_FIFO_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
} Key_FIFO_t;

static Key_FIFO_t g_KeyFIFO = {0};

// --- 裸机内部函数: 写入FIFO ---
static void BareMetal_PushFIFO(Key_ID_t key)
{
    uint8_t next = (g_KeyFIFO.head + 1) % KEY_FIFO_SIZE;
    if (next != g_KeyFIFO.tail)
    { // 未满
        g_KeyFIFO.buf[g_KeyFIFO.head] = key;
        g_KeyFIFO.head = next;
    }
}

// --- 裸机内部函数: 读取FIFO ---
static Key_ID_t BareMetal_PopFIFO(void)
{
    if (g_KeyFIFO.head == g_KeyFIFO.tail)
        return KEY_NONE;
    Key_ID_t key = g_KeyFIFO.buf[g_KeyFIFO.tail];
    g_KeyFIFO.tail = (g_KeyFIFO.tail + 1) % KEY_FIFO_SIZE;
    return key;
}

// --- 接口实现: 读取按键 (包含轮询逻辑) ---
Key_ID_t BSP_Key_Read(void)
{
    // 1. 优先检查中断缓冲区 (WKUP, Encoder)
    Key_ID_t key = BareMetal_PopFIFO();
    if (key != KEY_NONE)
        return key;

    // 2. 轮询 PC0-PC3
    for (int i = 0; i < 4; i++)
    {
        if (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
        {
            HAL_Delay(KEY_DEBOUNCE_TIME); // 阻塞消抖
            if (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
            {
                // 等待释放 (可选，防止连发)
                while (HAL_GPIO_ReadPin(PC_Port, PC_Pins[i]) == GPIO_PIN_SET)
                    ;
                return PC_IDs[i]; // 直接返回
            }
        }
    }

    // 3. 轮询 PE4
    if (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
    {
        HAL_Delay(KEY_DEBOUNCE_TIME);
        if (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(ENC_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
                ;
            return KEY_ENC_BTN;
        }
    }

    return KEY_NONE;
}

// --- 中断回调 (裸机版) ---
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // WKUP
    if (GPIO_Pin == WKUP_PIN)
    {
        if (HAL_GPIO_ReadPin(WKUP_PORT, WKUP_PIN) == GPIO_PIN_SET)
        {
            BareMetal_PushFIFO(KEY_WKUP);
        }
    }
    // Encoder
    if (GPIO_Pin == ENC_A_PIN || GPIO_Pin == ENC_B_PIN)
    {
        uint8_t A_Curr = HAL_GPIO_ReadPin(ENC_PORT, ENC_A_PIN);
        uint8_t B_Curr = HAL_GPIO_ReadPin(ENC_PORT, ENC_B_PIN);
        if (A_Curr != Enc_A_Last)
        {
            if (A_Curr == 0)
            {
                if (B_Curr == 0)
                {
                    Encoder_Count++;
                    BareMetal_PushFIFO(KEY_ENC_CW);
                }
                else
                {
                    Encoder_Count--;
                    BareMetal_PushFIFO(KEY_ENC_CCW);
                }
            }
            Enc_A_Last = A_Curr;
        }
    }
}

#endif // KEY_USE_FREERTOS

/* ================= 中断入口 (通用) ================= */
// 确保 stm32f4xx_it.c 中没有重复定义
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(WKUP_PIN); }
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(ENC_A_PIN); }
void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(ENC_B_PIN); }