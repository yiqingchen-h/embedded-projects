#include "SysTick.h"

fsp_err_t SystickInit(void)
{
    /* 1. 获取处理器主时钟频率 */
    uint32_t uwSysclk = R_BSP_SourceClockHzGet(FSP_PRIV_CLOCK_PLL);

    /* 2. 计算重装载值，配置1ms中断一次 */
    if (SysTick_Config(uwSysclk / 1000) != 0)
    {
        return FSP_ERR_ASSERTION;
    }

    return FSP_SUCCESS;
}
volatile uint32_t dwTick = 0; // 全局变量，记录系统运行的毫秒数

void SysTick_Handler(void) // SysTick中断发生时自动调用
{
    dwTick += 1; // 每毫秒增加1
}
uint32_t HAL_GetTick(void)
{
    return dwTick;
}
void HAL_Delay(uint32_t dwTime) // dwTime = 要延时的毫秒数
{
    uint32_t dwStart = dwTick; // 记录开始时间
    uint32_t dwWait = dwTime;  // 要等待的时间

    /* 加1保证最小等待时间 */
    if (dwWait < HAL_MAX_DELAY)
    {
        dwWait += (uint32_t)(1);
    }

    /* 循环等待，直到经过足够的时间 */
    while ((dwTick - dwStart) < dwWait)
    {
        // 空循环，等待时间到达
    }
}