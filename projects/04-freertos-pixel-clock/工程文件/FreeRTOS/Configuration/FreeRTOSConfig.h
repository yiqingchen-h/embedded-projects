/*
 * FreeRTOS V202212.01
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION     1
#define configUSE_IDLE_HOOK      0
#define configUSE_TICK_HOOK      0
#define configCPU_CLOCK_HZ       ((unsigned long)168000000)
#define configTICK_RATE_HZ       ((TickType_t)1000)
#define configMAX_PRIORITIES     (10)
#define configMINIMAL_STACK_SIZE ((unsigned short)128)
#define configTOTAL_HEAP_SIZE    ((size_t)(40 * 1024))
#define configMAX_TASK_NAME_LEN  (16)
#define configUSE_TRACE_FACILITY 1
#define configUSE_16_BIT_TICKS   0
#define configIDLE_SHOULD_YIELD  1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet      1
#define INCLUDE_uxTaskPriorityGet     1
#define INCLUDE_vTaskDelete           1
#define INCLUDE_vTaskCleanUpResources 0
#define INCLUDE_vTaskSuspend          1
#define INCLUDE_vTaskDelayUntil       1
#define INCLUDE_vTaskDelay            1
/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 191 /* equivalent to 0xb0, or priority 11. */

/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15

/*添加必须的三个宏*/
#define xPortPendSVHandler             PendSV_Handler
#define vPortSVCHandler                SVC_Handler
#define INCLUDE_xTaskGetSchedulerState 1
/**开启静态分配模式 -- 默认为关闭 */
#define configSUPPORT_STATIC_ALLOCATION 0
/**开启软件定时器 */
#define configUSE_TIMERS             0                               // 在配置静态任务时如开启软件定时器则需创建
#define configTIMER_TASK_PRIORITY    (configMAX_PRIORITIES - 1)      /* 定义软件定时器任务的优先级, 无默认configUSE_TIMERS为1时需定义 */
#define configTIMER_QUEUE_LENGTH     5 	                             /* 定义软件定时器命令队列的长度, 无默认configUSE_TIMERS为1时需定义 */
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)  /* 定义软件定时器任务的栈空间大小, 无默认configUSE_TIMERS为1时需定义 */
/**开启任务挂起和恢复功能 */
#define INCLUDE_vTaskSuspend   1
#define INCLUDE_xResumeFromISR 1

/* 开启跟踪task信息 */
#define configUSE_TRACE_FACILITY             1   //在51行以定义将值修改为1即可
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
// 开启任务栈历史剩余最小值查询
#define INCLUDE_uxTaskGetStackHighWaterMark 1
/**开启计数信号量定义 */
#define configUSE_COUNTING_SEMAPHORES 1
/**开启互斥信号量 */
#define configUSE_MUTEXES 1

#endif /* FREERTOS_CONFIG_H */
