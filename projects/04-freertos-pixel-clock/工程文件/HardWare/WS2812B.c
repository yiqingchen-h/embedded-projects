#include "WS2812B.h"

// ============================================================
//                      全局变量定义
// ============================================================
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;

// 显存数组：存储每个像素点的 RGB 颜色值
// 大小 = 灯珠总数 * 3字节
RGB_Color_t PixelBuffer[PIXEL_MAX];

// DMA 发送缓冲区：存储转换后的 PWM 占空比数据
// 大小 = (灯珠总数 * 24位 + 复位信号) * 2字节(uint16_t)
#define DMA_BUF_SIZE (PIXEL_MAX * 24 + WS_RESET_SLOTS)
uint16_t DMA_Buffer[DMA_BUF_SIZE];

// 传输状态标志位 (1=正在发送, 0=空闲)
volatile uint8_t is_transferring = 0;

// 全局亮度变量 (0-255)，默认 50
uint8_t Global_Brightness = 50;

// ============================================================
//                  图形/自模数据区
// ============================================================

// 一个 8x8 的爱心图标
const uint8_t Icon_Heart[8] = {
    0x0C, 0x1E, 0x3E, 0x7C, 0x7C, 0x3E, 0x1E, 0x0C};

// 一个 8x8 的笑脸图标
const uint8_t Icon_Smile[8] = {
    0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C};

// 8x8箭头图标
const uint8_t Icon_arrowhead[8] = {
    0x00, 0x1C, 0x1C, 0x7F, 0x3E, 0x1C, 0x08, 0x00};

// 8x8对号
const uint8_t Icon_correct[8] = {
    0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03};
// 8x8错号
const uint8_t Icon_Wrong[8] = {0xC3, 0xE7, 0x7E, 0x3C, 0x3C, 0x7E, 0xE7, 0xC3};
// ============================================================
//                  5x7 ASCII 标准字模库
// ============================================================
// 包含 ASCII 32(空格) 到 126(~)
// 格式：5列 x 7行
const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // \ (back slash)
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // ~
};

// ============================================================
//                  底层硬件初始化
// ============================================================
void WS2812_HW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 1. 开启外设时钟
    __HAL_RCC_GPIOD_CLK_ENABLE(); // 开启 GPIO 时钟
    __HAL_RCC_DMA1_CLK_ENABLE();  // 开启 DMA 时钟
    __HAL_RCC_TIM4_CLK_ENABLE();  // 开启 定时器 时钟

    // 2. 配置 GPIO 引脚 (PD12)
    GPIO_InitStruct.Pin = WS_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;                // 无上下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速模式
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;         // 复用映射到 TIM4
    HAL_GPIO_Init(WS_GPIO_PORT, &GPIO_InitStruct);

    // 3. 配置 DMA (TIM4_CH1 对应 DMA1_Stream0_Channel2)
    hdma_tim4_ch1.Instance = DMA1_Stream0;
    hdma_tim4_ch1.Init.Channel = DMA_CHANNEL_2;
    hdma_tim4_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;              // 方向：内存 -> 外设
    hdma_tim4_ch1.Init.PeriphInc = DMA_PINC_DISABLE;                  // 外设地址不自增 (始终发给 CCR1)
    hdma_tim4_ch1.Init.MemInc = DMA_MINC_ENABLE;                      // 内存地址自增 (遍历 Buffer)
    hdma_tim4_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // 外设数据宽度 16位
    hdma_tim4_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // 内存数据宽度 16位
    hdma_tim4_ch1.Init.Mode = DMA_NORMAL;                             // 普通模式 (发完一次停止)
    hdma_tim4_ch1.Init.Priority = DMA_PRIORITY_HIGH;                  // 高优先级
    hdma_tim4_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;               // 禁用 FIFO
    if (HAL_DMA_Init(&hdma_tim4_ch1) != HAL_OK)
        while (1)
            ; // 初始化失败卡死

    // 关联 DMA 句柄到定时器句柄
    __HAL_LINKDMA(&htim4, hdma[TIM_DMA_ID_CC1], hdma_tim4_ch1);

    // 4. 配置 DMA 中断 (用于判断发送结束)
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    // 5. 配置定时器 TIM4
    htim4.Instance = WS_TIM_INSTANCE;
    htim4.Init.Prescaler = 0;                    // 不分频 (84MHz)
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP; // 向上计数
    htim4.Init.Period = WS_ARR_PERIOD;           // 周期 104 (800kHz)
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);

    // 配置时钟源
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
    HAL_TIM_PWM_Init(&htim4);

    // 配置 PWM 通道
    sConfigOC.OCMode = TIM_OCMODE_PWM1;         // PWM模式1
    sConfigOC.Pulse = 0;                        // 初始占空比 0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 有效电平为高
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, WS_TIM_CHANNEL);
}

// DMA 传输完成回调函数 (发送完毕后停止 PWM)
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == WS_TIM_INSTANCE)
    {
        HAL_TIM_PWM_Stop_DMA(&htim4, WS_TIM_CHANNEL);
        is_transferring = 0; // 清除忙碌标志
    }
}

// ============================================================
//                      核心控制逻辑
// ============================================================

// 初始化函数
void WS2812_Init(void)
{
    WS2812_HW_Init();                          // 初始化硬件
    WS2812_Clear();                            // 清空显存
    memset(DMA_Buffer, 0, sizeof(DMA_Buffer)); // 清空 DMA 缓冲
}

// 设置全局亮度
void WS2812_SetBrightness(uint8_t scale)
{
    Global_Brightness = scale;
}

// 刷新显示
void WS2812_Update(void)
{
    if (is_transferring)
        return; // 如果上一帧还没发完，直接退出

    uint32_t buf_idx = 0;

    // 遍历每一个像素点
    for (int i = 0; i < PIXEL_MAX; i++)
    {
        // 1. 亮度计算: (颜色值 * 亮度) / 256
        // 使用移位运算 (>>8) 代替除法，效率更高
        uint8_t r = (PixelBuffer[i].R * Global_Brightness) >> 8;
        uint8_t g = (PixelBuffer[i].G * Global_Brightness) >> 8;
        uint8_t b = (PixelBuffer[i].B * Global_Brightness) >> 8;

        // 2. 组合颜色: WS2812 协议要求的顺序是 G -> R -> B
        uint32_t color_val = (g << 16) | (r << 8) | b;

        // 3. 将 24bit 颜色转换为 24个 PWM 占空比数据
        for (int bit = 23; bit >= 0; bit--)
        {
            // 如果该位是1，发送长高电平；如果是0，发送短高电平
            DMA_Buffer[buf_idx++] = (color_val & (1 << bit)) ? WS_1_CODE : WS_0_CODE;
        }
    }

    // 4. 填充复位信号
    for (int i = 0; i < WS_RESET_SLOTS; i++)
    {
        DMA_Buffer[buf_idx++] = 0;
    }

    // 5. 启动 DMA 传输
    is_transferring = 1;
    HAL_TIM_PWM_Start_DMA(&htim4, WS_TIM_CHANNEL, (uint32_t *)DMA_Buffer, buf_idx);
}
// 清屏 (填充黑色)
void WS2812_Clear(void)
{
    memset(PixelBuffer, 0, sizeof(PixelBuffer));
}
// ============================================================
//                  坐标映射算法
// ============================================================
// 功能：将逻辑坐标 (x,y) 转换为灯珠的物理索引 Index
// 输入：x (0-31), y (0-7)
// 输出：0-255 的数组索引
static uint16_t Get_Pixel_Index(uint16_t x, uint16_t y)
{
    // 负数检查
    if (x < 0 || x >= PIXEL_WIDTH || y < 0 || y >= PIXEL_HEIGHT)
        return 0xFFFF;
    // 1. 边界检查，防止数组越界
    if (x >= PIXEL_WIDTH || y >= PIXEL_HEIGHT)
        return 0xFFFF;

#if CURRENT_LAYOUT == LAYOUT_COL_ZIGZAG

    if (x % 2 == 0)
    {
        // 偶数列：索引 = 当前列 * 8 + 当前行
        return x * PIXEL_HEIGHT + y;
    }
    else
    {
        // 奇数列：索引 = 当前列 * 8 + (7 - 当前行)
        // 因为是从下往上，y=0时在最下面(索引最大)，y=7时在最上面(索引最小)
        return x * PIXEL_HEIGHT + (PIXEL_HEIGHT - 1 - y);
    }

#elif CURRENT_LAYOUT == LAYOUT_ROW_ZIGZAG
    // 水平 S 型
    if (y % 2 == 0)
        return y * PIXEL_WIDTH + x;
    else
        return y * PIXEL_WIDTH + (PIXEL_WIDTH - 1 - x);

#elif CURRENT_LAYOUT == LAYOUT_ROW_PROGRESSIVE
    // 逐行
    return y * PIXEL_WIDTH + x;

#elif CURRENT_LAYOUT == LAYOUT_COL_PROGRESSIVE
    // 逐列
    return x * PIXEL_HEIGHT + y;
#endif

    return 0;
}

// ============================================================
//                      图形绘制函数
// ============================================================

// 画点
void WS2812_DrawPoint(int16_t x, int16_t y, RGB_Color_t color)
{
    uint16_t idx = Get_Pixel_Index(x, y);
    if (idx != 0xFFFF)
        PixelBuffer[idx] = color;
}

// 画线
void WS2812_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, RGB_Color_t color)
{
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy, e2;
    while (1)
    {
        WS2812_DrawPoint(x1, y1, color);
        if (x1 == x2 && y1 == y2)
            break;
        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x1 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

// 画矩形(空心)
void WS2812_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB_Color_t color)
{
    WS2812_DrawLine(x, y, x + w - 1, y, color);                 // 上边
    WS2812_DrawLine(x, y + h - 1, x + w - 1, y + h - 1, color); // 下边
    WS2812_DrawLine(x, y, x, y + h - 1, color);                 // 左边
    WS2812_DrawLine(x + w - 1, y, x + w - 1, y + h - 1, color); // 右边
}
// 画矩形（实心）
void WS2812_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB_Color_t color)
{
    for (int i = 0; i < w; i++)
    {
        for (int j = 0; j < h; j++)
        {
            WS2812_DrawPoint(x + i, y + j, color);
        }
    }
}
// 画圆
void WS2812_DrawCircle(int16_t x0, int16_t y0, uint8_t r, RGB_Color_t color)
{
    int a = 0, b = r;
    int di = 3 - (r << 1);
    while (a <= b)
    {
        WS2812_DrawPoint(x0 + a, y0 - b, color);
        WS2812_DrawPoint(x0 + b, y0 - a, color);
        WS2812_DrawPoint(x0 + b, y0 + a, color);
        WS2812_DrawPoint(x0 + a, y0 + b, color);
        WS2812_DrawPoint(x0 - a, y0 + b, color);
        WS2812_DrawPoint(x0 - b, y0 + a, color);
        WS2812_DrawPoint(x0 - a, y0 - b, color);
        WS2812_DrawPoint(x0 - b, y0 - a, color);
        a++;
        if (di < 0)
            di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

// 画单个字符
void WS2812_DrawChar(int16_t x, int16_t y, char c, RGB_Color_t color, RGB_Color_t bg_color)
{
    if (c < ' ' || c > '~')
        return;            // 超出 ASCII 范围不显示
    uint8_t idx = c - ' '; // 计算字模索引

    for (uint8_t i = 0; i < 5; i++)
    { // 遍历 5 列
        uint8_t line = Font5x7[idx][i];
        for (uint8_t j = 0; j < 7; j++)
        { // 遍历 7 行
            if (line & (1 << j))
            {
                WS2812_DrawPoint(x + i, y + j, color); // 画字体颜色
            }
            else if (bg_color.R || bg_color.G || bg_color.B)
            {
                WS2812_DrawPoint(x + i, y + j, bg_color); // 画背景颜色
            }
        }
    }
}

// 显示字符串
void WS2812_ShowString(int16_t x, int16_t y, char *str, RGB_Color_t color, RGB_Color_t bg_color)
{
    while (*str)
    {
        WS2812_DrawChar(x, y, *str, color, bg_color);
        x += 6; // 字宽 5 + 间距 1
        str++;
    }
}

// 画自定义图片
// 参数：x,y 起始坐标; w,h 图片宽高; img 图片数据数组
void WS2812_DrawImage(int16_t x, int16_t y, uint16_t w, int16_t h, const uint8_t *img, RGB_Color_t color)
{
    for (uint16_t i = 0; i < w; i++)
    {
        uint8_t temp = img[i];
        for (uint16_t j = 0; j < h; j++)
        {
            if (temp & (1 << j))
            {
                WS2812_DrawPoint(x + i, y + j, color);
            }
        }
    }
}

// ============================================================
//               场景 A: 显示 HH:MM (时:分)
// ============================================================
// 布局: 使用标准 5x7 字体，宽敞居中
// 相对 X 偏移量 (4个数字)
static const int16_t Offset_HM_Digits[4] = {0, 6, 14, 20};
static const int16_t Offset_HM_Colon = 12;

/**
 * @brief 显示 HH:MM (标准5x7字体，支持动画速度控制，独立冒号颜色)
 * @param x_start 建议设为 4 (居中)
 * @param duration_ms 动画时长(ms)，建议 300-500
 */
void WS2812_DrawTime_HM_Animate(
    TimeAnim_Handle_t *handle,
    uint8_t hour, uint8_t min,
    int16_t x_start,
    RGB_Color_t color,       // 数字颜色
    RGB_Color_t colon_color, // 冒号颜色
    uint32_t tick,
    uint16_t duration_ms // 动画时长控制
)
{
    // --- 1. 初始化 ---
    if (handle->initialized == 0)
    {
        handle->last_hour = hour;
        handle->last_min = min;
        handle->last_sec = 0;
        handle->anim_state = 0;
        handle->offset = 0;
        handle->initialized = 1;
    }

    // --- 2. 触发动画检测 ---
    if (handle->anim_state == 0)
    {
        if (handle->last_hour != hour || handle->last_min != min)
        {
            handle->anim_state = 1;
            handle->anim_start_tick = tick;
            handle->offset = 0;
        }
    }

    // --- 3. 计算动画进度 ---
    if (handle->anim_state == 1)
    {
        if (duration_ms == 0)
            duration_ms = 1;
        uint32_t elapsed = tick - handle->anim_start_tick;

        if (elapsed >= duration_ms)
        {
            handle->offset = 8; // 动画结束
        }
        else
        {
            // 线性插值计算位移
            handle->offset = (elapsed * 8) / duration_ms;
        }
    }

    // --- 4. 拆分数字 ---
    uint8_t digits_old[4];
    uint8_t digits_new[4];

    digits_old[0] = handle->last_hour / 10;
    digits_old[1] = handle->last_hour % 10;
    digits_old[2] = handle->last_min / 10;
    digits_old[3] = handle->last_min % 10;

    digits_new[0] = hour / 10;
    digits_new[1] = hour % 10;
    digits_new[2] = min / 10;
    digits_new[3] = min % 10;

    // --- 5. 绘制冒号 ---
    if ((tick % 1000) < 500)
    {
        int16_t cx = x_start + Offset_HM_Colon;
        // 冒号由上下两组像素组成，每组两点
        WS2812_DrawPoint(cx, 2, colon_color);
        WS2812_DrawPoint(cx, 3, colon_color);
        WS2812_DrawPoint(cx, 5, colon_color);
        WS2812_DrawPoint(cx, 6, colon_color);
    }

    // --- 6. 绘制数字  ---
    for (int i = 0; i < 4; i++)
    {
        int16_t x = x_start + Offset_HM_Digits[i];
        int16_t y_static = 1; // 5x7字体在8行屏幕中，y=1垂直居中

        if (handle->anim_state == 1 && digits_old[i] != digits_new[i])
        {
            // 滚动动画
            WS2812_DrawChar(x, y_static - handle->offset, digits_old[i] + '0', color, COLOR_BLACK);
            WS2812_DrawChar(x, y_static + (8 - handle->offset), digits_new[i] + '0', color, COLOR_BLACK);
        }
        else
        {
            // 静态显示
            WS2812_DrawChar(x, y_static, digits_old[i] + '0', color, COLOR_BLACK);
        }
    }

    // --- 7. 动画结束结算 ---
    if (handle->anim_state == 1 && handle->offset >= 8)
    {
        handle->anim_state = 0;
        handle->offset = 0;
        handle->last_hour = hour;
        handle->last_min = min;
    }
}
// 格式：每行3位，1亮0灭
static const uint8_t Font3x5[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
    {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
    {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
    {0b111, 0b001, 0b111, 0b001, 0b111}, // 3
    {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
    {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
    {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
    {0b111, 0b001, 0b001, 0b001, 0b001}, // 7
    {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
    {0b111, 0b101, 0b111, 0b001, 0b111}  // 9
};

// ============================================================
//               配置与定义
// ============================================================

// 紧凑布局 (总宽 23px)
static const int16_t Tight_Digits_X[6] = {0, 4, 8, 12, 16, 20};
static const int16_t Tight_Colons_X[2] = {7, 15};

/**
 * @brief 绘制3x5字符，带底部裁剪功能
 * @param clip_y 裁剪线Y坐标。任何 >= clip_y 的像素都不会被绘制。
 */
void WS2812_DrawChar3x5_Clip(int16_t x, int16_t y, uint8_t num, RGB_Color_t color, int16_t clip_y)
{
    if (num > 9)
        return;
    for (int row = 0; row < 5; row++)
    {
        int16_t draw_y = y + row;

        if (draw_y >= clip_y)
            continue;

        uint8_t line = Font3x5[num][row];
        for (int col = 0; col < 3; col++)
        {
            if ((line >> (2 - col)) & 0x01)
            {
                WS2812_DrawPoint(x + col, draw_y, color);
            }
        }
    }
}

// 辅助：计算流光颜色
RGB_Color_t Get_Flow_Color(RGB_Color_t color1, RGB_Color_t color2, uint32_t tick, int16_t x_pos)
{
    float time_factor = (float)tick / 120.0f;
    float space_factor = (float)x_pos * 0.6f;
    float mix = (sinf(time_factor - space_factor) + 1.0f) / 2.0f;

    RGB_Color_t out;
    out.R = (uint8_t)(color1.R * mix + color2.R * (1.0f - mix));
    out.G = (uint8_t)(color1.G * mix + color2.G * (1.0f - mix));
    out.B = (uint8_t)(color1.B * mix + color2.B * (1.0f - mix));
    return out;
}

// ============================================================
//           主功能：带裁剪区域的时间滚动绘制
// ============================================================

void WS2812_DrawTime_Final_Clipped(
    TimeAnim_Handle_t *handle,
    uint8_t hour, uint8_t min, uint8_t sec,
    int16_t x_start, int16_t y_start,
    RGB_Color_t num_color,
    RGB_Color_t colon_color,
    RGB_Color_t line_base_color,
    RGB_Color_t line_flow_color,
    uint32_t tick,
    uint16_t duration_ms)
{
    // --- 1. 初始化 ---
    if (handle->initialized == 0)
    {
        handle->last_hour = hour;
        handle->last_min = min;
        handle->last_sec = sec;
        handle->anim_state = 0;
        handle->offset = 0;
        handle->initialized = 1;
    }

    // --- 2. 动画触发 ---
    if (handle->anim_state == 0)
    {
        if (handle->last_sec != sec || handle->last_min != min || handle->last_hour != hour)
        {
            handle->anim_state = 1;
            handle->anim_start_tick = tick;
            handle->offset = 0;
        }
    }

    // --- 3. 计算滚动位移 ---
    const uint8_t SCROLL_H = 6;
    if (handle->anim_state == 1)
    {
        if (duration_ms == 0)
            duration_ms = 1;
        uint32_t elapsed = tick - handle->anim_start_tick;
        if (elapsed >= duration_ms)
        {
            handle->offset = SCROLL_H;
        }
        else
        {
            handle->offset = (elapsed * SCROLL_H) / duration_ms;
        }
    }

    // --- 4. 准备数据 ---
    uint8_t digits_now[6] = {hour / 10, hour % 10, min / 10, min % 10, sec / 10, sec % 10};
    uint8_t digits_old[6] = {handle->last_hour / 10, handle->last_hour % 10,
                             handle->last_min / 10, handle->last_min % 10,
                             handle->last_sec / 10, handle->last_sec % 10};

    // --- 5. 确定关键坐标 ---
    int16_t line_y = y_start + 6; // 横线所在的行

    // --- 6. 绘制冒号 ---
    for (int k = 0; k < 2; k++)
    {
        int16_t cx = x_start + Tight_Colons_X[k];
        WS2812_DrawPoint(cx, y_start + 1, colon_color);
        WS2812_DrawPoint(cx, y_start + 3, colon_color);
    }

    // --- 7. 绘制横线 ---
    int active_line_index = 5 - (sec / 10); // 60s 倒退循环
    for (int i = 0; i < 6; i++)
    {
        int16_t x = x_start + Tight_Digits_X[i];
        if (i == active_line_index)
        {
            for (int w = 0; w < 3; w++)
            {
                RGB_Color_t c = Get_Flow_Color(line_base_color, line_flow_color, tick, x + w);
                WS2812_DrawPoint(x + w, line_y, c);
            }
        }
        else
        {
            for (int w = 0; w < 3; w++)
            {
                WS2812_DrawPoint(x + w, line_y, line_base_color);
            }
        }
    }

    // --- 8. 绘制数字 ---
    for (int i = 0; i < 6; i++)
    {
        int16_t x = x_start + Tight_Digits_X[i];

        if (handle->anim_state == 1 && digits_old[i] != digits_now[i])
        {
            // 滚动动画
            // 旧数字向上移
            WS2812_DrawChar3x5_Clip(x, y_start - handle->offset, digits_old[i], num_color, line_y);

            // 新数字从下往上移
            // 当 offset=0 时，y = y_start + 6，完全被裁剪
            // 当 offset=6 时，y = y_start，完全显示
            WS2812_DrawChar3x5_Clip(x, y_start + (SCROLL_H - handle->offset), digits_now[i], num_color, line_y);
        }
        else
        {
            // 静态显示
            WS2812_DrawChar3x5_Clip(x, y_start, digits_now[i], num_color, line_y);
        }
    }

    // --- 9. 状态更新 ---
    if (handle->anim_state == 1 && handle->offset >= SCROLL_H)
    {
        handle->anim_state = 0;
        handle->offset = 0;
        handle->last_hour = hour;
        handle->last_min = min;
        handle->last_sec = sec;
    }
}
// 2. 绘制跑马灯
void WS2812_Draw_Marquee(uint16_t y_line, float pos, RGB_Color_t color)
{
    // 步骤1：衰减整行的亮度
    for (int x = 0; x < PIXEL_WIDTH; x++)
    {
        uint16_t idx = Get_Pixel_Index(x, y_line);
        if (idx != 0xFFFF)
        {
            // 衰减因子
            PixelBuffer[idx].R = (PixelBuffer[idx].R * 160) >> 8;
            PixelBuffer[idx].G = (PixelBuffer[idx].G * 160) >> 8;
            PixelBuffer[idx].B = (PixelBuffer[idx].B * 160) >> 8;
        }
    }

    // 步骤2：绘制新的光点
    int x_int = (int)pos;

    // 绘制中心点
    WS2812_DrawPoint(x_int, y_line, color);

    // 绘制辉光
    RGB_Color_t dim_color;
    dim_color.R = color.R >> 2;
    dim_color.G = color.G >> 2;
    dim_color.B = color.B >> 2;

    if (x_int > 0)
        WS2812_DrawPoint(x_int - 1, y_line, dim_color);
    if (x_int < PIXEL_WIDTH - 1)
        WS2812_DrawPoint(x_int + 1, y_line, dim_color);
}
// 一个 8x8 日历数字
const uint8_t figure_8x8[][8] = {
    0x00, 0x00, 0x44, 0x7E, 0x7E, 0x40, 0x00, 0x00,
    0x00, 0x00, 0xF2, 0x92, 0x92, 0x9E, 0x00, 0x00,
    0x00, 0x00, 0x92, 0x92, 0x92, 0xFE, 0x00, 0x00,
    0x00, 0x00, 0x1E, 0x10, 0x10, 0xFE, 0x00, 0x00,
    0x00, 0x00, 0x9E, 0x92, 0x92, 0xF2, 0x00, 0x00,
    0x00, 0x00, 0xFE, 0x92, 0x92, 0xF2, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x02, 0x02, 0x7E, 0x00, 0x00};
// 显示日历or星期
void WS2812_show_calendar_or_week(int16_t x_start, int16_t y_start, uint8_t time)
{
    WS2812_DrawLine(x_start, y_start, x_start + 7, y_start, COLOR_RED);
    for (uint8_t i = 1; i < 8; i++)
    {
        WS2812_DrawLine(x_start, y_start + i, x_start + 7, y_start + i, COLOR_WHITE);
    }
    WS2812_DrawImage(x_start, y_start, 8, 8, &figure_8x8[time - 1][0], COLOR_GREEN);
}

void test_WS2812B(void)
{
    unsigned char i = 0;
    for (i = 0; i < PIXEL_WIDTH; i++)
    {
        for (unsigned char j = 0; j < PIXEL_HEIGHT; j++)
        {
            WS2812_DrawPoint(i, j, COLOR_RED);
        }
    }
}

const uint8_t Icon_Fan_Solid[4][8] = {
    // 垂直/水平
    {0x18, 0x18, 0x18, 0x7E, 0x7E, 0x18, 0x18, 0x18},

    // 对角线
    {0x07, 0x0E, 0x1C, 0x38, 0x1C, 0x38, 0x70, 0xE0},

    //  垂直/水平
    {0x00, 0x7E, 0x7E, 0x18, 0x18, 0x7E, 0x7E, 0x00},

    // 对角线
    {0xE0, 0x70, 0x38, 0x1C, 0x38, 0x1C, 0x0E, 0x07}};

/**
 * @brief 绘制旋转的风扇动画 (8x8)
 * @param x, y        起始坐标
 * @param color       风扇颜色
 * @param bg_color    背景颜色
 * @param tick        当前系统时间戳
 * @param interval_ms 旋转速度
 */
void WS2812_DrawFan_Animate(int16_t x, int16_t y, RGB_Color_t color, RGB_Color_t bg_color, uint32_t tick, uint16_t interval_ms)
{
    if (interval_ms == 0)
        interval_ms = 80;

    // 1. 计算帧索引
    uint8_t frame_index = (tick / interval_ms) % 4;

    // 2. 清除背景
    WS2812_FillRect(x, y, 8, 8, bg_color);

    // 4. 绘制
    WS2812_DrawImage(x, y, 8, 8, Icon_Fan_Solid[frame_index], color);
}

// 1. 太阳
const uint8_t BMP_Sun_Static[8] = {
    0b00011000, // Y=0    **
    0b10000001, // Y=1 *      *
    0b00111100, // Y=2   ****
    0b10111101, // Y=3 * **** *
    0b10111101, // Y=4 * **** *
    0b00111100, // Y=5   ****
    0b10000001, // Y=6 *      *
    0b00011000  // Y=7    **
};

// 2. 云朵
const uint8_t BMP_Cloud_Top[8] = {
    0b00000000, // Y=0
    0b00011100, // Y=1    ***
    0b00111110, // Y=2   *****
    0b01111111, // Y=3  *******
    0b00000000, // Y=4
    0b00000000, // Y=5
    0b00000000, // Y=6
    0b00000000  // Y=7
};

// 3. 雪花
const uint8_t BMP_Snow_Crystal[8] = {
    0b00011000, // Y=0    **
    0b10011001, // Y=1 *  **  *
    0b01011010, // Y=2  * ** *
    0b00111100, // Y=3   ****
    0b00111100, // Y=4   ****
    0b01011010, // Y=5  * ** *
    0b10011001, // Y=6 *  **  *
    0b00011000  // Y=7    **
};

// 4. 闪电
const uint8_t BMP_Bolt[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00001100,
    0b00011000,
    0b00001100,
    0b00010000,
    0b00100000};

// 5. 雾
const uint8_t BMP_Fog_Lines[8] = {
    0b00000000,
    0b01111110,
    0b00000000,
    0b10101010,
    0b00000000,
    0b01111110,
    0b00000000,
    0b00000000};

// ============================================================
//                  绘图辅助函数
// ============================================================

void Draw_Bitmap_Direct(int16_t x_offset, int16_t y_offset, const uint8_t *bitmap, RGB_Color_t color)
{
    for (int y = 0; y < 8; y++)
    {
        uint8_t row_data = bitmap[y];
        for (int x = 0; x < 8; x++)
        {
            if (row_data & (0x80 >> x))
            {
                WS2812_DrawPoint(x_offset + x, y_offset + y, color);
            }
        }
    }
}

// ============================================================
//                  天气逻辑实现
// ============================================================

void WS2812_ShowWeather_Scene(char *code_str, char *temp_str, uint32_t tick)
{
    int code = atoi(code_str);

    // 动画变量
    uint8_t step_flash = (tick / 50) % 20; // 闪电周期
    uint8_t step_flow = (tick / 150) % 8;  // 雾/风流动

    // ---------------------------------------------------------
    // 1. 晴 (Sunny) - 静态
    // ---------------------------------------------------------
    if ((code >= 0 && code <= 3) || code == 38)
    {
        // 直接画静态太阳，无动画
        Draw_Bitmap_Direct(0, 0, BMP_Sun_Static, COLOR_ORANGE);
    }
    // ---------------------------------------------------------
    // 2. 多云/阴 (Cloudy)
    // ---------------------------------------------------------
    else if (code >= 4 && code <= 9)
    {
        Draw_Bitmap_Direct(0, 0, BMP_Cloud_Top, (RGB_Color_t){80, 80, 80});
        WS2812_DrawLine(1, 4, 6, 4, (RGB_Color_t){60, 60, 60});
        WS2812_DrawLine(2, 5, 5, 5, (RGB_Color_t){40, 40, 40});
    }
    // ---------------------------------------------------------
    // 3. 雨 (Rain) & 雷雨 (Thunderstorm)
    // ---------------------------------------------------------
    else if ((code >= 10 && code <= 19))
    {
        bool is_thunder = (code == 11 || code == 12 || code >= 16);

        RGB_Color_t cloud_color = is_thunder ? (RGB_Color_t){60, 0, 80} : COLOR_BLUE;
        Draw_Bitmap_Direct(0, 0, BMP_Cloud_Top, cloud_color);

        // 乱雨动画
        int y1 = 4 + ((tick / 80) % 6);
        if (y1 < 8)
            WS2812_DrawPoint(1, y1, COLOR_CYAN);

        int y2 = 4 + (((tick / 100) + 2) % 6);
        if (y2 < 8)
            WS2812_DrawPoint(4, y2, COLOR_CYAN);
        if (y2 - 1 >= 4 && y2 - 1 < 8)
            WS2812_DrawPoint(4, y2 - 1, COLOR_CYAN);

        int y3 = 4 + (((tick / 120) + 4) % 6);
        if (y3 < 8)
            WS2812_DrawPoint(6, y3, COLOR_CYAN);

        // 雷雨特效
        if (is_thunder)
        {
            if (step_flash < 3)
            {
                Draw_Bitmap_Direct(0, 0, BMP_Bolt, COLOR_YELLOW);
                if (step_flash == 0)
                {
                    Draw_Bitmap_Direct(0, 0, BMP_Cloud_Top, COLOR_WHITE);
                }
            }
        }
    }
    // ---------------------------------------------------------
    // 4. 雪 (Snow) - 静态/无闪烁
    // ---------------------------------------------------------
    else if ((code >= 20 && code <= 25) || code == 37)
    {
        // 直接画新的六角雪花，移除所有动画代码
        Draw_Bitmap_Direct(0, 0, BMP_Snow_Crystal, COLOR_WHITE);
    }
    // ---------------------------------------------------------
    // 5. 雾/霾 (Fog)
    // ---------------------------------------------------------
    else if (code >= 26 && code <= 31)
    {
        RGB_Color_t fog_c = (code >= 30) ? (RGB_Color_t){100, 100, 100} : (RGB_Color_t){100, 80, 0};
        Draw_Bitmap_Direct(0, 0, BMP_Fog_Lines, fog_c);

        int offset = step_flow % 8;
        WS2812_DrawPoint((2 + offset) % 8, 3, fog_c);
        WS2812_DrawPoint((5 + offset) % 8, 3, fog_c);
    }
    // ---------------------------------------------------------
    // 6. 风 (Wind)
    // ---------------------------------------------------------
    else if (code >= 32 && code <= 36)
    {
        int x_pos = (tick / 40) % 14;
        int x1 = x_pos;
        if (x1 < 8)
            WS2812_DrawLine(x1, 2, x1 - 2, 2, COLOR_GREEN);
        int x2 = (x_pos + 5) % 14;
        if (x2 < 8)
            WS2812_DrawLine(x2, 5, x2 - 2, 5, COLOR_GREEN);
    }
    // ---------------------------------------------------------
    // 7. 未知
    // ---------------------------------------------------------
    else
    {
        WS2812_DrawRect(1, 1, 6, 6, COLOR_RED);
        WS2812_DrawChar(2, 1, '?', COLOR_RED, COLOR_BLACK);
    }

    // ==========================================
    //              右侧显示温度
    // ==========================================
    RGB_Color_t temp_color;
    int temp_val = atoi(temp_str);
    if (temp_val >= 30)
        temp_color = COLOR_RED;
    else if (temp_val <= 10)
        temp_color = COLOR_BLUE;
    else
        temp_color = COLOR_GREEN;

    WS2812_ShowString(9, 1, temp_str, temp_color, COLOR_BLACK);

    int len = strlen(temp_str);
    int unit_x = 9 + (len * 6);

    WS2812_DrawPoint(unit_x, 1, temp_color);
    WS2812_DrawPoint(unit_x + 1, 1, temp_color);
    WS2812_DrawPoint(unit_x, 2, temp_color);
    WS2812_DrawPoint(unit_x + 1, 2, temp_color);

    WS2812_DrawChar(unit_x + 3, 1, 'C', temp_color, COLOR_BLACK);
}

// ============================================================
//           通用温度计轮廓 (5x8)
// ============================================================
const uint8_t BMP_Thermo_Outline[8] = {
    0b00011100, // Y=0   ***
    0b00010100, // Y=1   * *
    0b00010100, // Y=2   * *
    0b00010100, // Y=3   * *
    0b00010100, // Y=4   * *
    0b00100010, // Y=5  *   *
    0b00100010, // Y=6  *   *
    0b00011100  // Y=7   ***
};

// ============================================================
//                  绘图辅助函数
// ============================================================

// 绘制位图轮廓
void Draw_Bitmap_Outline(int16_t x_offset, int16_t y_offset, const uint8_t *bitmap, RGB_Color_t color)
{
    for (int y = 0; y < 8; y++)
    {
        uint8_t row_data = bitmap[y];
        for (int x = 0; x < 8; x++)
        {
            if (row_data & (0x80 >> x))
            {
                WS2812_DrawPoint(x_offset + x, y_offset + y, color);
            }
        }
    }
}

// 绘制 3x5 数字
void WS2812_DrawChar3x5_Simple(int16_t x, int16_t y, char c, RGB_Color_t color)
{
    if (c < '0' || c > '9')
        return;
    uint8_t idx = c - '0';

    for (int row = 0; row < 5; row++)
    {
        uint8_t line = Font3x5[idx][row];
        // 遍历3列 (Bit2, Bit1, Bit0)
        for (int col = 0; col < 3; col++)
        {
            if ((line >> (2 - col)) & 0x01)
            {
                WS2812_DrawPoint(x + col, y + row, color);
            }
        }
    }
}

// ============================================================
//                  环境场景实现 (温湿度)
// ============================================================

void WS2812_Show_Env_Scene(float temp, float humi, uint32_t tick)
{
    // =========================================================
    // 左侧：温度
    // 区域：X = 0 ~ 15
    // =========================================================
    int16_t x_temp_icon = 1; // 图标位置
    int16_t x_temp_num = 8;  // 数字位置

    // 1. 计算红色液柱高度 (0-40度 -> 0-6像素)
    int t_val = (int)temp;
    if (t_val < 0)
        t_val = 0;
    if (t_val > 40)
        t_val = 40;
    int t_height = (t_val * 6) / 40;
    if (t_val > 0 && t_height == 0)
        t_height = 1;

    // 2. 绘制红色填充
    // 液泡
    WS2812_DrawLine(x_temp_icon + 3, 6, x_temp_icon + 5, 6, COLOR_RED);
    WS2812_DrawLine(x_temp_icon + 3, 5, x_temp_icon + 5, 5, COLOR_RED);
    // 管子
    for (int h = 0; h < t_height; h++)
    {
        int draw_y = 4 - h;
        if (draw_y >= 1)
            WS2812_DrawPoint(x_temp_icon + 4, draw_y, COLOR_RED);
    }

    // 3. 绘制白色轮廓
    Draw_Bitmap_Outline(x_temp_icon, 0, BMP_Thermo_Outline, COLOR_WHITE);

    // 4. 显示温度数字
    char t_str[5];
    sprintf(t_str, "%2d", (int)temp);
    if (t_str[0] != ' ')
        WS2812_DrawChar3x5_Simple(x_temp_num, 2, t_str[0], COLOR_RED);
    WS2812_DrawChar3x5_Simple(x_temp_num + 4, 2, t_str[1], COLOR_RED);

    // =========================================================
    // 右侧：湿度
    // 区域：X = 17 ~ 31
    // =========================================================
    int16_t x_humi_icon = 17; // 图标位置
    int16_t x_humi_num = 24;  // 数字位置

    // 1. 计算蓝色液柱高度
    int h_val = (int)humi;
    if (h_val > 100)
        h_val = 100;
    int h_height = (h_val * 6) / 100;
    if (h_val > 0 && h_height == 0)
        h_height = 1;

    // 2. 绘制蓝色填充
    // 液泡
    WS2812_DrawLine(x_humi_icon + 3, 6, x_humi_icon + 5, 6, COLOR_BLUE);
    WS2812_DrawLine(x_humi_icon + 3, 5, x_humi_icon + 5, 5, COLOR_BLUE);
    // 管子
    for (int h = 0; h < h_height; h++)
    {
        int draw_y = 4 - h;
        if (draw_y >= 1)
            WS2812_DrawPoint(x_humi_icon + 4, draw_y, COLOR_BLUE);
    }

    // 3. 绘制白色轮廓
    Draw_Bitmap_Outline(x_humi_icon, 0, BMP_Thermo_Outline, COLOR_WHITE);

    // 4. 显示湿度数字
    char h_str[5];
    sprintf(h_str, "%2d", (int)humi);
    if (h_str[0] != ' ')
        WS2812_DrawChar3x5_Simple(x_humi_num, 2, h_str[0], COLOR_CYAN);
    WS2812_DrawChar3x5_Simple(x_humi_num + 4, 2, h_str[1], COLOR_CYAN);
}

/**
 * @brief 显示亮度调节界面
 * @param percent 用户设定的亮度百分比
 */
void WS2812_Show_Brightness_Scene(uint8_t percent)
{
    // 1. 安全范围限制
    if (percent > 100)
        percent = 100;

    // 2. 映射亮度值: 0-100 映射到 10-200
    // 公式: Min + (Percent * (Max - Min) / 100)
    uint8_t hw_brightness = 10 + (percent * 190) / 100;

    // 3. 设置实际硬件亮度
    WS2812_SetBrightness(hw_brightness);

    // ================= UI 绘制 =================

    // 4. 左侧绘制太阳图标
    // 颜色随亮度略微变化：低亮度偏红，高亮度偏黄/白
    RGB_Color_t sun_color;
    if (percent < 30)
        sun_color = (RGB_Color_t){100, 20, 0}; // 暗红
    else if (percent < 70)
        sun_color = COLOR_ORANGE; // 橙色
    else
        sun_color = (RGB_Color_t){255, 200, 50}; // 亮黄

    WS2812_DrawImage(0, 0, 8, 8, BMP_Sun_Static, sun_color);

    // 5. 右侧绘制进度条
    // 进度条区域: x=10, y=2, 宽=21, 高=4

    // 绘制边框
    RGB_Color_t frame_color = {40, 40, 40};
    WS2812_DrawRect(10, 2, 22, 4, frame_color);

    // 计算填充宽度
    uint8_t fill_width = (percent * 20) / 100;

    // 绘制填充条
    if (fill_width > 0)
    {
        // 在边框内部填充
        WS2812_FillRect(11, 3, fill_width, 2, COLOR_WHITE);
    }

    // 如果亮度为0，在进度条里画一个红点表示最低
    if (percent == 0)
    {
        WS2812_DrawPoint(11, 3, COLOR_RED);
        WS2812_DrawPoint(11, 4, COLOR_RED);
    }
}
//=================================动画绘制==================================
// ============================================================
//           俄罗斯方块
// ============================================================

// --- 游戏配置 ---
#define TETRIS_X_MAX 32
#define TETRIS_Y_MAX 8
#define TETRIS_SPEED 80 // 游戏刷新速度(ms)

// 宏：判断颜色是否被占用
#define IS_OCCUPIED(color) ((color.R | color.G | color.B) > 0)
#define ABS(x) ((x) > 0 ? (x) : -(x))

// --- 状态机 ---
typedef enum
{
    STATE_SPAWN = 0,
    STATE_NAVIGATE,
    STATE_LOCK,
    STATE_CHECK_LINE,
    STATE_CLEAR_ANIM,
    STATE_GAMEOVER
} Tetris_State_t;

static Tetris_State_t Cur_State = STATE_SPAWN;
static uint32_t State_Timer = 0;
static uint8_t Anim_Counter = 0;

// --- 游戏数据 ---
static RGB_Color_t Board[TETRIS_X_MAX][TETRIS_Y_MAX];
static RGB_Color_t Cur_Color;
static int8_t Cur_Type = 0;
static int8_t Cur_Rot = 0;
static int8_t Cur_X = 0;
static int8_t Cur_Y = 0;

// AI 目标
static int8_t Target_X = 0;
static int8_t Target_Rot = 0;

// --- 方块形状 (7种 x 4旋转 x 4点) ---
const int8_t Shapes[7][4][4][2] = {
    // I (长条)
    {
        {{0, 1}, {1, 1}, {2, 1}, {3, 1}}, // 横
        {{2, 0}, {2, 1}, {2, 2}, {2, 3}}, // 竖
        {{0, 2}, {1, 2}, {2, 2}, {3, 2}}, // 横
        {{1, 0}, {1, 1}, {1, 2}, {1, 3}}  // 竖
    },
    // J
    {
        {{0, 0}, {0, 1}, {1, 1}, {2, 1}},
        {{1, 0}, {2, 0}, {1, 1}, {1, 2}},
        {{0, 1}, {1, 1}, {2, 1}, {2, 2}},
        {{1, 0}, {1, 1}, {0, 2}, {1, 2}}},
    // L
    {
        {{2, 0}, {0, 1}, {1, 1}, {2, 1}},
        {{1, 0}, {1, 1}, {1, 2}, {2, 2}},
        {{0, 1}, {1, 1}, {2, 1}, {0, 2}},
        {{0, 0}, {1, 0}, {1, 1}, {1, 2}}},
    // O (正方)
    {
        {{1, 0}, {2, 0}, {1, 1}, {2, 1}},
        {{1, 0}, {2, 0}, {1, 1}, {2, 1}},
        {{1, 0}, {2, 0}, {1, 1}, {2, 1}},
        {{1, 0}, {2, 0}, {1, 1}, {2, 1}}},
    // S
    {
        {{1, 0}, {2, 0}, {0, 1}, {1, 1}},
        {{1, 0}, {1, 1}, {2, 1}, {2, 2}},
        {{1, 0}, {2, 0}, {0, 1}, {1, 1}},
        {{1, 0}, {1, 1}, {2, 1}, {2, 2}}},
    // T
    {
        {{1, 0}, {0, 1}, {1, 1}, {2, 1}},
        {{1, 0}, {1, 1}, {2, 1}, {1, 2}},
        {{0, 1}, {1, 1}, {2, 1}, {1, 2}},
        {{1, 0}, {0, 1}, {1, 1}, {1, 2}}},
    // Z
    {
        {{0, 0}, {1, 0}, {1, 1}, {2, 1}},
        {{2, 0}, {1, 1}, {2, 1}, {1, 2}},
        {{0, 0}, {1, 0}, {1, 1}, {2, 1}},
        {{2, 0}, {1, 1}, {2, 1}, {1, 2}}}};

// --- OVER 字模 ---
const uint8_t Font_OVER[4][5] = {
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x19, 0x29, 0x46}  // R
};

// --- 辅助函数 ---

static RGB_Color_t Get_Random_Color(void)
{
    uint8_t r = rand() % 6;
    switch (r)
    {
    case 0:
        return COLOR_RED;
    case 1:
        return COLOR_ORANGE;
    case 2:
        return COLOR_YELLOW;
    case 3:
        return COLOR_GREEN;
    case 4:
        return COLOR_CYAN;
    default:
        return COLOR_MAGENTA;
    }
}

// 碰撞检测
static uint8_t Check_Collision(int8_t type, int8_t rot, int8_t x, int8_t y)
{
    for (int i = 0; i < 4; i++)
    {
        int8_t px = x + Shapes[type][rot][i][0];
        int8_t py = y + Shapes[type][rot][i][1];

        if (px < 0 || px >= TETRIS_X_MAX)
            return 1; // 左右越界
        if (py >= TETRIS_Y_MAX)
            return 1; // 触底
        if (py >= 0 && IS_OCCUPIED(Board[px][py]))
            return 1; // 撞方块
    }
    return 0;
}

// --- AI 评分 (平方级高度惩罚) ---
static int Calculate_Score(int8_t type, int8_t rot, int8_t x, int8_t y)
{
    uint8_t temp_board[TETRIS_X_MAX][TETRIS_Y_MAX] = {0};

    // 1. 复制棋盘
    for (int i = 0; i < TETRIS_X_MAX; i++)
        for (int j = 0; j < TETRIS_Y_MAX; j++)
            if (IS_OCCUPIED(Board[i][j]))
                temp_board[i][j] = 1;

    // 2. 放入模拟方块
    for (int i = 0; i < 4; i++)
    {
        int px = x + Shapes[type][rot][i][0];
        int py = y + Shapes[type][rot][i][1];
        if (px >= 0 && px < TETRIS_X_MAX && py >= 0 && py < TETRIS_Y_MAX)
            temp_board[px][py] = 1;
    }

    // 3. 计算指标
    int lines = 0, holes = 0, max_h = 0, bumpiness = 0;
    int col_h[TETRIS_X_MAX] = {0};
    long sq_height_sum = 0; // 平方高度和

    for (int i = 0; i < TETRIS_X_MAX; i++)
    {
        int top = 0;
        for (int j = 0; j < TETRIS_Y_MAX; j++)
        {
            if (temp_board[i][j])
            {
                if (top == 0)
                    top = TETRIS_Y_MAX - j;
            }
            else if (top > 0)
            {
                holes++;
            }
        }
        col_h[i] = top;
        sq_height_sum += (top * top);
        if (top > max_h)
            max_h = top;
    }

    for (int j = 0; j < TETRIS_Y_MAX; j++)
    {
        int full = 1;
        for (int i = 0; i < TETRIS_X_MAX; i++)
            if (!temp_board[i][j])
                full = 0;
        if (full)
            lines++;
    }

    for (int i = 0; i < TETRIS_X_MAX - 1; i++)
        bumpiness += ABS(col_h[i] - col_h[i + 1]);

    // 4. 评分
    int score = 0;
    score += lines * 10000;      // 消除行：最高优先级
    score -= holes * 5000;       // 空洞：绝对禁止
    score -= sq_height_sum * 20; // 平方高度：强力压制堆高
    score -= bumpiness * 5;      // 崎岖度：适当平整

    if (max_h >= 6)
        score -= 5000; // 警戒线

    return score;
}

// --- AI 规划路径 (加入 10% 的失误率) ---
static void AI_Plan_Move(void)
{
    // 模拟人类失误：不计算最佳位置，而是随机选一个目标
    if ((rand() % 100) < 10)
    {
        // 随机选一个 X (0 到 28)
        Target_X = rand() % (TETRIS_X_MAX - 4);
        // 随机选一个旋转
        Target_Rot = rand() % 4;
        return; // 直接返回，跳过智能计算
    }

    // --- 下面是正常的智商 ---
    int best_score = -99999999;
    int best_x = Cur_X;
    int best_rot = 0;

    for (int r = 0; r < 4; r++)
    {
        // 扩大搜索范围
        for (int x = -2; x < TETRIS_X_MAX; x++)
        {
            if (Check_Collision(Cur_Type, r, x, -4))
                continue;

            int y = -3;
            while (!Check_Collision(Cur_Type, r, x, y + 1))
                y++;

            if (y < -1)
                continue;

            int score = Calculate_Score(Cur_Type, r, x, y);

            if (score > best_score)
            {
                best_score = score;
                best_x = x;
                best_rot = r;
            }
        }
    }
    Target_X = best_x;
    Target_Rot = best_rot;
}

// 绘制 OVER
static void Draw_Big_OVER(void)
{
    int start_x = 5;
    for (int char_idx = 0; char_idx < 4; char_idx++)
    {
        for (int col = 0; col < 5; col++)
        {
            uint8_t line = Font_OVER[char_idx][col];
            for (int row = 0; row < 7; row++)
            {
                if (line & (1 << row))
                {
                    WS2812_DrawPoint(start_x + col, row + 1, COLOR_WHITE);
                }
            }
        }
        start_x += 6;
    }
}

// ============================================================
//                 主运行函数
// ============================================================
void Tetris_Game_Run(uint32_t tick)
{
    // -------------------------------------------------
    // 逻辑更新
    // -------------------------------------------------
    if (tick - State_Timer >= TETRIS_SPEED)
    {
        State_Timer = tick;

        switch (Cur_State)
        {
        case STATE_SPAWN:
        {
            Cur_Type = rand() % 7;
            Cur_Color = Get_Random_Color();
            AI_Plan_Move();

            if (Target_X < 10)
                Cur_X = 8;
            else if (Target_X > 22)
                Cur_X = 24;
            else
                Cur_X = 16;
            Cur_X += (rand() % 3) - 1;

            Cur_Rot = 0;
            Cur_Y = -3;

            if (Check_Collision(Cur_Type, Cur_Rot, Cur_X, Cur_Y))
            {
                Cur_State = STATE_GAMEOVER;
                Anim_Counter = 0;
            }
            else
            {
                Cur_State = STATE_NAVIGATE;
            }
            break;
        }

        case STATE_NAVIGATE:
        {
            // 1. 旋转
            if (Cur_Rot != Target_Rot)
            {
                int8_t next_rot = (Cur_Rot + 1) % 4;
                if (!Check_Collision(Cur_Type, next_rot, Cur_X, Cur_Y))
                    Cur_Rot = next_rot;
            }
            // 2. 移动
            for (int speed = 0; speed < 4; speed++)
            {
                if (Cur_X != Target_X)
                {
                    int8_t next_x = Cur_X;
                    if (Cur_X < Target_X)
                        next_x++;
                    else if (Cur_X > Target_X)
                        next_x--;

                    if (!Check_Collision(Cur_Type, Cur_Rot, next_x, Cur_Y))
                        Cur_X = next_x;
                    else
                        break;
                }
            }
            // 3. 下落
            if (Check_Collision(Cur_Type, Cur_Rot, Cur_X, Cur_Y + 1))
                Cur_State = STATE_LOCK;
            else
                Cur_Y++;
            break;
        }

        case STATE_LOCK:
        {
            int visible = 0;
            for (int i = 0; i < 4; i++)
            {
                int px = Cur_X + Shapes[Cur_Type][Cur_Rot][i][0];
                int py = Cur_Y + Shapes[Cur_Type][Cur_Rot][i][1];
                if (px >= 0 && px < TETRIS_X_MAX && py >= 0 && py < TETRIS_Y_MAX)
                {
                    Board[px][py] = Cur_Color;
                    visible++;
                }
            }
            if (visible == 0)
            {
                Cur_State = STATE_GAMEOVER;
                Anim_Counter = 0;
            }
            else
            {
                Cur_State = STATE_CHECK_LINE;
            }
            break;
        }

        case STATE_CHECK_LINE:
        {
            int found = 0;
            for (int y = 0; y < TETRIS_Y_MAX; y++)
            {
                int cnt = 0;
                for (int x = 0; x < TETRIS_X_MAX; x++)
                    if (IS_OCCUPIED(Board[x][y]))
                        cnt++;
                if (cnt == TETRIS_X_MAX)
                    found = 1;
            }
            if (found)
            {
                Cur_State = STATE_CLEAR_ANIM;
                Anim_Counter = 0;
            }
            else
            {
                Cur_State = STATE_SPAWN;
            }
            break;
        }

        case STATE_CLEAR_ANIM:
        {
            Anim_Counter++;
            if (Anim_Counter > 6)
            {
                RGB_Color_t NewBoard[TETRIS_X_MAX][TETRIS_Y_MAX];
                memset(NewBoard, 0, sizeof(NewBoard));
                int new_y = TETRIS_Y_MAX - 1;
                for (int y = TETRIS_Y_MAX - 1; y >= 0; y--)
                {
                    int cnt = 0;
                    for (int x = 0; x < TETRIS_X_MAX; x++)
                        if (IS_OCCUPIED(Board[x][y]))
                            cnt++;
                    if (cnt < TETRIS_X_MAX)
                    {
                        for (int x = 0; x < TETRIS_X_MAX; x++)
                            NewBoard[x][new_y] = Board[x][y];
                        new_y--;
                    }
                }
                memcpy(Board, NewBoard, sizeof(Board));
                Cur_State = STATE_SPAWN;
            }
            break;
        }

        case STATE_GAMEOVER:
        {
            Anim_Counter++;
            int fill_y = TETRIS_Y_MAX - 1 - (Anim_Counter / 2);
            if (fill_y >= 0)
            {
                for (int x = 0; x < TETRIS_X_MAX; x++)
                    Board[x][fill_y] = COLOR_RED;
            }
            if (Anim_Counter > 80)
            {
                memset(Board, 0, sizeof(Board));
                Cur_State = STATE_SPAWN;
            }
            break;
        }
        }
    }

    // 1. 画棋盘
    for (int x = 0; x < TETRIS_X_MAX; x++)
    {
        for (int y = 0; y < TETRIS_Y_MAX; y++)
        {
            if (IS_OCCUPIED(Board[x][y]))
                WS2812_DrawPoint(x, y, Board[x][y]);
        }
    }

    // 2. 画当前方块
    if (Cur_State == STATE_NAVIGATE || Cur_State == STATE_SPAWN || Cur_State == STATE_LOCK)
    {
        for (int i = 0; i < 4; i++)
        {
            int px = Cur_X + Shapes[Cur_Type][Cur_Rot][i][0];
            int py = Cur_Y + Shapes[Cur_Type][Cur_Rot][i][1];
            WS2812_DrawPoint(px, py, Cur_Color);
        }
    }

    // 3. 消除闪烁
    if (Cur_State == STATE_CLEAR_ANIM && (Anim_Counter % 2))
    {
        for (int y = 0; y < TETRIS_Y_MAX; y++)
        {
            int cnt = 0;
            for (int x = 0; x < TETRIS_X_MAX; x++)
                if (IS_OCCUPIED(Board[x][y]))
                    cnt++;
            if (cnt == TETRIS_X_MAX)
            {
                for (int x = 0; x < TETRIS_X_MAX; x++)
                    WS2812_DrawPoint(x, y, COLOR_WHITE);
            }
        }
    }

    // 4. OVER 文字
    if (Cur_State == STATE_GAMEOVER && Anim_Counter > 16)
    {
        Draw_Big_OVER();
    }
}

// ============================================================
// 2. 黑客帝国数字雨
// ============================================================
static int8_t Rain_Drops[32]; // 记录每一列雨滴的Y坐标

void Anim_Matrix(uint32_t tick)
{
    static uint8_t init = 0;
    static uint32_t last_drop = 0;

    // 1. 初始化
    if (!init)
    {
        for (int i = 0; i < 32; i++)
            Rain_Drops[i] = -1; // -1 表示该列空闲
        init = 1;
    }

    // ==================================================
    // Part 1: 逻辑更新
    // ==================================================
    if (tick - last_drop > 60) // 速度控制
    {
        last_drop = tick;

        for (int x = 0; x < 32; x++)
        {
            // A. 生成新雨滴
            if (Rain_Drops[x] == -1)
            {
                // 5% 概率生成
                if ((rand() % 100) < 10)
                {
                    Rain_Drops[x] = 0;
                }
            }

            // B. 下落逻辑
            if (Rain_Drops[x] >= 0)
            {
                Rain_Drops[x]++;

                if (Rain_Drops[x] > 15)
                {
                    Rain_Drops[x] = -1;
                }
            }
        }
    }

    for (int x = 0; x < 32; x++)
    {
        int head_y = Rain_Drops[x];

        // 如果这一列有雨滴
        if (head_y >= 0)
        {
            // 1. 画头部
            // 只有在屏幕范围内才画
            if (head_y < 8)
            {
                WS2812_DrawPoint(x, head_y, (RGB_Color_t){180, 255, 180});
            }

            // 2. 画拖尾
            // 尾巴1
            if (head_y - 1 >= 0 && head_y - 1 < 8)
                WS2812_DrawPoint(x, head_y - 1, (RGB_Color_t){0, 200, 0});

            // 尾巴2
            if (head_y - 2 >= 0 && head_y - 2 < 8)
                WS2812_DrawPoint(x, head_y - 2, (RGB_Color_t){0, 100, 0});

            // 尾巴3
            if (head_y - 3 >= 0 && head_y - 3 < 8)
                WS2812_DrawPoint(x, head_y - 3, (RGB_Color_t){0, 40, 0});
        }
    }
}

// ============================================================
//           场景: 马里奥跑酷 - 丰富场景 & 修正重心
// ============================================================

#define M_RED (RGB_Color_t){200, 0, 0}       // 红
#define M_BLUE (RGB_Color_t){0, 0, 180}      // 蓝
#define M_SKIN (RGB_Color_t){255, 140, 100}  // 肤色
#define M_BROWN (RGB_Color_t){100, 50, 0}    // 砖块/地
#define M_G_DARK (RGB_Color_t){60, 30, 0}    // 地面纹理
#define M_PIPE (RGB_Color_t){0, 180, 0}      // 水管
#define M_GOLD (RGB_Color_t){255, 200, 0}    // 金币/箱子
#define M_GOOMBA (RGB_Color_t){180, 80, 0}   // 板栗仔(褐色)
#define M_CLOUD (RGB_Color_t){100, 100, 100} // 云朵(暗白)
#define M_BUSH (RGB_Color_t){0, 100, 0}      // 草丛(暗绿)
#define M_SKY (RGB_Color_t){0, 0, 0}         // 黑背景

// --- 2. 5x5 微型马里奥 ---
// 0=空, 1=红, 2=肤, 3=蓝, 4=褐
const uint8_t Mario_5x5[2][5][5] = {
    // 帧0: 站立/迈左腿
    {
        {1, 1, 1, 1, 0}, // 帽
        {2, 2, 2, 2, 0}, // 脸
        {1, 1, 3, 1, 0}, // 身
        {0, 3, 3, 3, 0}, // 裤
        {4, 4, 0, 4, 0}  // 脚
    },
    // 帧1: 跑动/迈右腿
    {
        {1, 1, 1, 1, 0}, // 帽
        {2, 2, 2, 2, 0}, // 脸
        {0, 1, 3, 1, 0}, // 身
        {0, 3, 3, 3, 0}, // 裤
        {0, 4, 4, 0, 0}  // 脚
    }};

// 板栗仔  3x3
const uint8_t Sprite_Goomba[3][3] = {
    {0, 1, 0}, // 眉毛
    {1, 2, 1}, // 脸 (1=褐, 2=肤)
    {3, 0, 3}  // 脚 (3=黑)
};

static RGB_Color_t Get_M_Color(uint8_t c)
{
    switch (c)
    {
    case 1:
        return M_RED;
    case 2:
        return M_SKIN;
    case 3:
        return M_BLUE;
    case 4:
        return M_BROWN;
    default:
        return M_SKY;
    }
}

// --- 3. 场景生成器 ---
// 根据世界坐标 world_x 返回该位置的对象类型
// 0=空, 1=云, 2=草, 3=砖+币, 4=水管, 5=板栗仔
uint8_t Get_Scene_Obj(uint32_t wx)
{
    // 使用伪随机算法，保证每次滚动到相同位置景物一致
    // 简单的哈希: (x * 质数) % 周期

    // 1. 稀有物体：水管
    if (wx % 50 == 40)
        return 4;
    if (wx % 50 == 41)
        return 4; // 水管宽2像素

    // 2. 敌人：板栗仔
    if (wx % 70 == 20)
        return 5;

    // 3. 空中物体：砖块
    if (wx % 30 == 10)
        return 3;
    if (wx % 30 == 11)
        return 3; // 砖块宽2像素

    // 4. 背景装饰：云
    if ((wx * 13) % 40 < 3)
        return 1;

    // 5. 背景装饰：草
    if ((wx * 7) % 20 < 2)
        return 2;

    return 0;
}

// --- 4. 主动画逻辑 ---
#define RUN_SPEED 60 // 滚动速度

void Anim_Mario_Run(uint32_t tick)
{
    static uint32_t last_tick = 0;
    static uint32_t distance = 0;

    // ==================================================
    // Part 1: 逻辑更新
    // ==================================================
    if (tick - last_tick >= RUN_SPEED)
    {
        last_tick = tick;
        distance++; // 只有这里更新距离
    }

    // ==================================================
    // Part 2: 绘图渲染
    // ==================================================
    // 外部任务已清屏，这里直接根据当前的 distance 绘制画面

    // 1. 绘制场景
    for (int x = 0; x < PIXEL_WIDTH; x++)
    {
        uint32_t wx = distance + x;
        uint8_t obj = Get_Scene_Obj(wx);

        // --- 地面 ---
        if (wx % 2 == 0)
            WS2812_DrawPoint(x, 7, M_BROWN);
        else
            WS2812_DrawPoint(x, 7, M_G_DARK);

        // --- 绘制对象 ---
        switch (obj)
        {
        case 1: // 云
            WS2812_DrawPoint(x, 1, M_CLOUD);
            if (wx % 2 == 0)
                WS2812_DrawPoint(x, 0, M_CLOUD);
            break;
        case 2: // 草
            WS2812_DrawPoint(x, 6, M_BUSH);
            break;
        case 3: // 砖块+金币
            WS2812_DrawPoint(x, 3, M_BROWN);
            if ((tick / 100) % 2)
                WS2812_DrawPoint(x, 1, M_GOLD);
            break;
        case 4: // 水管
            WS2812_DrawLine(x, 6, x, 5, M_PIPE);
            if (wx % 50 == 40)
                WS2812_DrawPoint(x, 5, (RGB_Color_t){100, 255, 100});
            break;
        case 5: // 板栗仔
            WS2812_DrawPoint(x, 6, M_GOOMBA);
            WS2812_DrawPoint(x, 5, M_GOOMBA);
            break;
        }
    }

    // 2. 绘制马里奥
    int mx = 2; // 固定X坐标

    // --- 自动跳跃逻辑 ---
    int jump_offset = 0;
    for (int i = 1; i <= 5; i++)
    {
        uint8_t next_obj = Get_Scene_Obj(distance + mx + i);
        if (next_obj == 4 || next_obj == 5)
        {
            if (i < 3)
                jump_offset = -3;
            else
                jump_offset = -1;
            break;
        }
        if (next_obj == 3 && i == 1)
        {
            if ((distance % 4) == 0)
                jump_offset = -1;
        }
    }

    // --- 确定位置 ---
    int base_y = 2 + jump_offset;

    // --- 动画帧 ---
    int frame = (jump_offset != 0) ? 0 : ((distance / 2) % 2);

    for (int row = 0; row < 5; row++)
    {
        for (int col = 0; col < 5; col++)
        {
            uint8_t c = Mario_5x5[frame][row][col];
            if (c != 0)
            {
                int dy = base_y + row;
                if (dy >= 0 && dy < 7)
                {
                    WS2812_DrawPoint(mx + col, dy, Get_M_Color(c));
                }
            }
        }
    }
}

// ============================================================
//                  统一接口实现
// ============================================================
void WS2812_Show_Animation_Scene(uint8_t anim_id, uint32_t tick)
{
    switch (anim_id)
    {
    case 0:
        Tetris_Game_Run(tick); // 俄罗斯方块
        break;
    case 1:
        Anim_Matrix(tick); // 黑客帝国
        break;
    case 2:
        Anim_Mario_Run(tick); // 马里奥
        break;
    case 3:
        // 预留给下一个动画
        break;
    default:
        Tetris_Game_Run(tick); // 默认
        break;
    }
}