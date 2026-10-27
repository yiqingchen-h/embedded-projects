#ifndef __WS2812B_H
#define __WS2812B_H
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
// 定义灯板的物理尺寸
// 矩阵尺寸为 32 列、8 行
#define PIXEL_WIDTH 32 // 宽度：32列 (x范围 0-31)
#define PIXEL_HEIGHT 8 // 高度：8行  (y范围 0-7)

// 计算总灯珠数量
#define PIXEL_MAX (PIXEL_WIDTH * PIXEL_HEIGHT)

// 模式 0: 逐行扫描
//         像写字一样，第一行左->右，第二行也是左->右。
#define LAYOUT_ROW_PROGRESSIVE 0

// 模式 1: 水平 S 型
//         第一行左->右，第二行右->左。常见于柔性屏。
#define LAYOUT_ROW_ZIGZAG 1

// 模式 2: 逐列扫描
//         第一列上->下，第二列也是上->下。
#define LAYOUT_COL_PROGRESSIVE 2

// 模式 3: 垂直 S 型
//         第一列(偶数)从上往下(1-8)，第二列(奇数)从下往上(16-9)。
#define LAYOUT_COL_ZIGZAG 3

// 当前使用的模式
#define CURRENT_LAYOUT LAYOUT_COL_ZIGZAG

#define WS_GPIO_PORT GPIOD
#define WS_GPIO_PIN GPIO_PIN_12
#define WS_TIM_INSTANCE TIM4
#define WS_TIM_CHANNEL TIM_CHANNEL_1

// 基于 84MHz 定时器时钟 (STM32F407 APB1 x2)
// 目标频率 800kHz -> 周期 = 84MHz / 800kHz = 105 ticks
#define WS_ARR_PERIOD 104 // 自动重装载值 (周期)
#define WS_0_CODE 32      // 逻辑0的高电平时间 (~30% 占空比)
#define WS_1_CODE 68      // 逻辑1的高电平时间 (~65% 占空比)
#define WS_RESET_SLOTS 80 // 复位信号需要的静默周期数 (>50us)

// ================= 颜色结构体定义 =================
typedef struct
{
    uint8_t R; // 红色分量 (0-255)
    uint8_t G; // 绿色分量 (0-255)
    uint8_t B; // 蓝色分量 (0-255)
} RGB_Color_t;
extern RGB_Color_t PixelBuffer[PIXEL_MAX];
// ================= 常用颜色宏定义 =================
#define COLOR_BLACK (RGB_Color_t){0, 0, 0}       // 黑色 (灭)
#define COLOR_RED (RGB_Color_t){255, 0, 0}       // 红色
#define COLOR_GREEN (RGB_Color_t){0, 255, 0}     // 绿色
#define COLOR_BLUE (RGB_Color_t){0, 0, 255}      // 蓝色
#define COLOR_WHITE (RGB_Color_t){255, 255, 255} // 白色
#define COLOR_YELLOW (RGB_Color_t){255, 255, 0}  // 黄色
#define COLOR_CYAN (RGB_Color_t){0, 255, 255}    // 青色
#define COLOR_MAGENTA (RGB_Color_t){255, 0, 255} // 紫色
#define COLOR_PINK (RGB_Color_t){255, 192, 203}  // 粉色
#define COLOR_ORANGE (RGB_Color_t){255, 140, 0}  // 橙色

// 外部变量声明
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern const uint8_t Icon_Heart[];     // 爱心
extern const uint8_t Icon_Smile[];     // 笑脸
extern const uint8_t Icon_arrowhead[]; // 箭头
extern const uint8_t Icon_correct[];   // 对号
extern const uint8_t Icon_Wrong[];     // 错号
// ================= API 函数接口声明 =================

// 1. 基础控制
void WS2812_Init(void);                   // 初始化驱动
void WS2812_SetBrightness(uint8_t scale); // 设置全局亮度 (0-255)
void WS2812_Clear(void);                  // 清空屏幕 (全黑)
void WS2812_Update(void);                 // 刷新显示
// 2. 基础绘图
void WS2812_DrawPoint(int16_t x, int16_t y, RGB_Color_t color);                          // 画点
void WS2812_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, RGB_Color_t color); // 画线
void WS2812_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB_Color_t color);     // 画矩形框
void WS2812_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, RGB_Color_t color);     // 画矩形(实心)
void WS2812_DrawCircle(int16_t x0, int16_t y0, uint8_t r, RGB_Color_t color);            // 画圆
void test_WS2812B(void);                                                                 // 测试全屏
// 3. 字符与图形
void WS2812_DrawChar(int16_t x, int16_t y, char c, RGB_Color_t color, RGB_Color_t bg_color);               // 写单个字符
void WS2812_ShowString(int16_t x, int16_t y, char *str, RGB_Color_t color, RGB_Color_t bg_color);          // 写字符串
void WS2812_DrawImage(int16_t x, int16_t y, uint16_t w, int16_t h, const uint8_t *img, RGB_Color_t color); // 画自定义图片
void WS2812_Draw_Marquee(uint16_t y_line, float pos, RGB_Color_t color);                                   // 画跑马灯
void WS2812_show_calendar_or_week(int16_t x_start, int16_t y_start, uint8_t time);                         // 显示日历或星期
// ==========================================
// 动画状态句柄结构体
// ==========================================
typedef struct
{
    uint8_t last_hour;
    uint8_t last_min;
    uint8_t last_sec;
    uint8_t anim_state;       // 0:静止, 1:动画进行中
    int16_t offset;           // 当前滚动偏移量 (0-8)
    uint32_t anim_start_tick; // 记录动画开始时的系统时间
    uint8_t initialized;
} TimeAnim_Handle_t;

/**
 * @brief 绘制带有局部滚动动画的时间
 *
 * @param handle   动画状态句柄
 * @param hour      当前小时
 * @param min      当前分钟
 * @param sec      当前秒
 * @param x_start  起始X坐标
 * @param y_start  起始y坐标
 * @param color    字体颜色
 * @param colon_color     冒号颜色
 * @param line_base_color     横线总体颜色
 * @param line_flow_color     横线流动颜色
 * @param tick     当前系统滴答 (ms)，用于控制冒号闪烁
 * @param duration_ms     动画时长
 */
void WS2812_DrawTime_Final_Clipped(
    TimeAnim_Handle_t *handle,
    uint8_t hour, uint8_t min, uint8_t sec,
    int16_t x_start, int16_t y_start,
    RGB_Color_t num_color,
    RGB_Color_t colon_color,
    RGB_Color_t line_base_color,
    RGB_Color_t line_flow_color,
    uint32_t tick,
    uint16_t duration_ms);

/**
 * @brief 绘制旋转的风扇动画 (8x8)
 * @param x, y        起始坐标
 * @param color       风扇颜色
 * @param bg_color    背景颜色
 * @param tick        当前系统时间戳 (HAL_GetTick() 或传入的计数器)
 * @param interval_ms 旋转速度(每帧保持的时间)，建议 50-100ms
 */
void WS2812_DrawFan_Animate(int16_t x, int16_t y, RGB_Color_t color, RGB_Color_t bg_color, uint32_t tick, uint16_t interval_ms);

/**
 * @brief 显示天气场景
 * @param code_str  心知天气返回的代码字符串
 * @param temp_str  温度字符串
 * @param tick      系统时钟
 */
void WS2812_ShowWeather_Scene(char *code_str, char *temp_str, uint32_t tick);
/**
 * @brief 显示环境信息
 * @param temp 温度值
 * @param humi 湿度值
 * @param tick 系统时钟
 */
void WS2812_Show_Env_Scene(float temp, float humi, uint32_t tick);

/**
 * @brief 亮度调节界面
 *
 * @param percent 亮度
 */
void WS2812_Show_Brightness_Scene(uint8_t percent);
/**
 * @brief 统一动画显示接口
 * @param anim_id 0:俄罗斯方块, 1:彩色流星, 2:吃豆人
 * @param tick 系统时钟
 */
void WS2812_Show_Animation_Scene(uint8_t anim_id, uint32_t tick);
#endif
