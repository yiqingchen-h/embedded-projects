#ifndef TAKES_H_
#define TAKES_H_

#include "FreeRTOS.h"
#include "task.h"


/**
 * @brief  从 ISR 通知低优先级 1ms 周期任务
 *
 * @return void
 */
void takes_key_notify_from_isr(void);

/**
 * @brief  从 ISR 通知高优先级 1ms 周期任务
 *
 * @return void
 */
void takes_high_priority_notify_from_isr(void);

/**
 * @brief  创建系统任务，由 system_init_entry 调用
 *
 * @return BaseType_t pdPASS 表示初始化任务创建成功
 */
BaseType_t takes_create_tasks(void);

#endif /* TAKES_H_ */
