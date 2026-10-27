#include "stm32f1xx_hal.h"
#include "UART.h"
#include "ADC.h"

ADC_HandleTypeDef ADC_HandleType_ADC7;
ADC_ChannelConfTypeDef ADC_ChannelConfType_ADC7;
DMA_HandleTypeDef DMA_HandleType_ADC7;
unsigned short ADC7_DmaBuff[10];
void ADC7_Init(void)
{
    ADC_HandleType_ADC7.Instance                = ADC1;                // 选择需要使用的ADC
    ADC_HandleType_ADC7.Init.DataAlign          = ADC_DATAALIGN_RIGHT; // 配置转换数据结果的对齐方式
    ADC_HandleType_ADC7.Init.ScanConvMode       = ADC_SCAN_DISABLE;    // 配置是否开启扫描模式
    ADC_HandleType_ADC7.Init.ContinuousConvMode = ENABLE;             // 配置是否开启连续转换模式 -- 若关闭则在进行一次ADC转换后会直接关闭ADC需要手动再次开启ADC转换 -- 使能时需要手动关闭调用对应的API函数如使用轮询打开则调用HAL_ADC_Stop。
    ADC_HandleType_ADC7.Init.ExternalTrigConv   = ADC_SOFTWARE_START;  // 配置触发方式
    HAL_ADC_Init(&ADC_HandleType_ADC7);

    ADC_ChannelConfType_ADC7.Channel      = ADC_CHANNEL_7;             // 配置ADC通道
    ADC_ChannelConfType_ADC7.Rank         = ADC_REGULAR_RANK_1;        // 配置通道转换的排名
    ADC_ChannelConfType_ADC7.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; // 配置采样时间
    HAL_ADC_ConfigChannel(&ADC_HandleType_ADC7, &ADC_ChannelConfType_ADC7);

    HAL_ADCEx_Calibration_Start(&ADC_HandleType_ADC7); // 自校准 -- 校准时ADC必须处于关闭状态--校准用于增加ADC精度至少需要2个的ADC时钟周期

    // HAL_ADC_Start(&ADC_HandleType_ADC7);        //开启转换-轮询方式
    // HAL_ADC_Start_IT(&ADC_HandleType_ADC7); // 开启转换-中断方式
    HAL_ADC_Start_DMA(&ADC_HandleType_ADC7, (uint32_t *)&ADC7_DmaBuff, 5); // 开启转换-DMA方式10次传输
}

// ADC初始化硬件回调函数
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitType;
        GPIO_InitType.Pin  = GPIO_PIN_7;
        GPIO_InitType.Mode = GPIO_MODE_ANALOG; // 模数转换 -- 当开启模拟口功能时无需配置速率和上下拉
        HAL_GPIO_Init(GPIOA, &GPIO_InitType);
        // 中断模式打开ADC使用
        //  ----------------------------------------------------------
        //  // 设置中断源和优先级-中断源在stm32f103xb.h文件69行开始
        //  HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
        //  // 使能中断
        //  HAL_NVIC_EnableIRQ(ADC1_IRQn);
        //-----------------------------------------------------------
        DMA_HandleType_ADC7.Instance                 = DMA1_Channel1;           // 选择DMA1通道
        DMA_HandleType_ADC7.Init.Direction           = DMA_PERIPH_TO_MEMORY;    // DMA传输方向
        DMA_HandleType_ADC7.Init.PeriphInc           = DMA_PINC_DISABLE;        // 外设地址是否递增
        DMA_HandleType_ADC7.Init.MemInc              = DMA_MINC_ENABLE;         // 存储区地址是否递增
        DMA_HandleType_ADC7.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 外设数据宽度
        DMA_HandleType_ADC7.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD; // 存储区数据宽度
        DMA_HandleType_ADC7.Init.Mode                = DMA_CIRCULAR;            // 工作方式
        DMA_HandleType_ADC7.Init.Priority            = DMA_PRIORITY_MEDIUM;     // 优先级
        __HAL_LINKDMA(hadc, DMA_Handle, DMA_HandleType_ADC7);                   // 双向链接
        HAL_DMA_Init(&DMA_HandleType_ADC7);

        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
}
short ADC_B = 0;
// ADC_中断转换完成回调函数 -- 同DMA完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    unsigned char i  = 0;
    unsigned int sum = 0;
    if (hadc->Instance == ADC1) {
        for (i = 0; i < 5; i++) {
            sum += ADC7_DmaBuff[i];
        }
        //ADC_B = (sum / 10.0) * (3.3 / 0xfff);
        ADC_B = (sum / 5);
    }
}