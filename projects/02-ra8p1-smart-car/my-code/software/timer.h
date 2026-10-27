#ifndef SOFTWARE_TIMER_H_
#define SOFTWARE_TIMER_H_

#include "hal_data.h"

/**
 * @brief  初始化并启动系统周期定时器 g_timer7 和 g_timer4
 *
 * @return fsp_err_t FSP_SUCCESS 表示启动成功
 */
fsp_err_t Timer_Init(void);

/**
 * @brief  停止并关闭系统周期定时器 g_timer7 和 g_timer4
 *
 * @return void
 */
void Timer_Deinit(void);

#endif /* SOFTWARE_TIMER_H_ */
