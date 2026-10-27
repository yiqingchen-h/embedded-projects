#include "AllHeader.h"

#include <stdbool.h>

/* 初始化任务栈大小，单位 word。 */
#define INIT_TASK_STACK_SIZE          (1024U)

/* 初始化任务优先级，负责创建 UI 任务并启动周期定时器。 */
#define INIT_TASK_PRIORITY            (15U)

/* UI 任务栈大小，LVGL 初始化和绘制对象需要较多栈空间。 */
#define UI_TASK_STACK_SIZE            (4096U)

/* UI 任务优先级，低于初始化任务。 */
#define UI_TASK_PRIORITY              (5U)

/* 按键任务栈大小，单位 word。 */
#define KEY_TASK_STACK_SIZE           (256U)

/* 按键任务优先级，高于 UI 任务以保证按键响应。 */
#define KEY_TASK_PRIORITY             (6U)

/* 电源采样任务栈大小，单位 word。 */
#define POWER_TASK_STACK_SIZE         (256U)

/* 电源采样任务优先级。 */
#define POWER_TASK_PRIORITY           (4U)

/* 电源采样周期，单位 ms。 */
#define POWER_TASK_PERIOD_MS          (100U)

/* 机械臂反馈任务栈大小，单位 word。 */
#define MECHANICAL_ARM_FEEDBACK_TASK_STACK_SIZE (512U)

/* 机械臂反馈任务优先级，低于 UI 和按键，避免舵机 UART 读取阻塞显示。 */
#define MECHANICAL_ARM_FEEDBACK_TASK_PRIORITY (4U)

/* 机械臂反馈任务周期，单位 ms。 */
#define MECHANICAL_ARM_FEEDBACK_TASK_PERIOD_MS (100U)

/* 机械臂初始化完成后的自动 Home 动作时间，单位 ms。 */
#define MECH_HAND_INIT_HOME_TIME_MS (1000U)

/* 初始化阶段自动 Home 等待机械臂总线互斥锁超时时间，单位 ms。 */
#define MECH_HAND_INIT_HOME_LOCK_TIMEOUT_MS (500U)

/* 底盘控制任务栈大小，单位 word。 */
#define CHASSIS_TASK_STACK_SIZE       (512U)

/* 底盘控制任务优先级，高于按键和 UI，保证 10ms 闭环周期优先执行。 */
#define CHASSIS_TASK_PRIORITY         (14U)

/* 底盘闭环控制周期，单位 ms。 */
#define CHASSIS_TASK_PERIOD_MS        (10U)

/* N10 雷达任务栈大小，单位 word。 */
#define RADAR_TASK_STACK_SIZE         (512U)

/* N10 雷达任务优先级，低于底盘闭环，高于 UI 和按键任务。 */
#define RADAR_TASK_PRIORITY           (13U)

/* N10 雷达后台解析周期，单位 ms。 */
#define RADAR_TASK_PERIOD_MS          (10U)

/* 语音 AI 后台任务栈大小，单位 word。 */
#define VOICE_AI_TASK_STACK_SIZE        (4096U)

/* 语音 AI 后台任务优先级，比按键任务低一级。 */
#define VOICE_AI_TASK_PRIORITY          (5U)

/* 语音 AI 后台处理周期，单位 ms。 */
#define VOICE_AI_TASK_PERIOD_MS         (200U)

/* 视觉任务栈大小，单位 word。 */
#define VISION_TASK_STACK_SIZE        (1024U)

/* 视觉任务优先级，低于底盘和雷达，高于 UI。 */
#define VISION_TASK_PRIORITY          (12U)

/* 视觉协议解析周期，单位 ms。 */
#define VISION_TASK_PERIOD_MS         (10U)

/* 系统监视任务栈大小，单位 word。 */
#define MONITOR_TASK_STACK_SIZE       (384U)

/* 系统监视任务优先级。 */
#define MONITOR_TASK_PRIORITY         (3U)

/* 系统监视周期，单位 ms。 */
#define MONITOR_TASK_PERIOD_MS        (5000U)

/* timer7 周期，单位 ms。 */
#define UI_TIMER_PERIOD_MS            (1U)

/* timer4 高优先级定时器周期，单位 ms。 */
#define HIGH_TIMER_PERIOD_MS          (1U)

/* LCD 上电后的稳定等待时间，避免过早初始化导致屏幕不亮。 */
#define UI_LCD_POWER_ON_DELAY_MS      (500U)

/* 按键任务句柄，由 timer7 中断通知。 */
static TaskHandle_t g_key_task_handle = NULL;

/* 电源采样任务句柄，由 timer7 中断通知。 */
static TaskHandle_t g_power_task_handle = NULL;

/* 机械臂反馈任务句柄，由 timer7 中断通知。 */
static TaskHandle_t g_mechanical_arm_feedback_task_handle = NULL;

/* 底盘控制任务句柄，由 timer4 高优先级中断通知。 */
static TaskHandle_t g_chassis_task_handle = NULL;

/* N10 雷达任务句柄，由 timer4 高优先级中断通知。 */
static TaskHandle_t g_radar_task_handle = NULL;

/* 视觉任务句柄，由 timer4 高优先级中断通知。 */
static TaskHandle_t g_vision_task_handle = NULL;

/* 系统监视任务句柄，由 timer7 中断通知。 */
static TaskHandle_t g_monitor_task_handle = NULL;

/* 语音 AI 后台任务句柄，由 takes.c 统一创建。 */
static TaskHandle_t g_voice_ai_task_handle = NULL;

/* UI 任务句柄，由 timer7 中断通知。 */
static TaskHandle_t g_ui_task_handle = NULL;

/* UI 任务就绪标志，LCD 初始化完成后才允许 ISR 通知。 */
static volatile bool g_ui_task_ready = false;

/**
 * @brief  从 ISR 通知低优先级周期任务
 *
 * timer7 中断中只发送低优先级周期任务通知，不读取 GPIO、不处理显示逻辑。
 *
 * @return void
 */
void takes_key_notify_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (NULL != g_key_task_handle)
    {
        vTaskNotifyGiveFromISR(g_key_task_handle, &higher_priority_task_woken);
    }

    if (NULL != g_power_task_handle)
    {
        vTaskNotifyGiveFromISR(g_power_task_handle, &higher_priority_task_woken);
    }

    if (NULL != g_mechanical_arm_feedback_task_handle)
    {
        vTaskNotifyGiveFromISR(g_mechanical_arm_feedback_task_handle, &higher_priority_task_woken);
    }

    if (NULL != g_monitor_task_handle)
    {
        vTaskNotifyGiveFromISR(g_monitor_task_handle, &higher_priority_task_woken);
    }

    if ((NULL != g_ui_task_handle) && g_ui_task_ready)
    {
        vTaskNotifyGiveFromISR(g_ui_task_handle, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief  从 ISR 通知高优先级周期任务
 *
 * timer4 高优先级中断中只发送底盘闭环和雷达任务通知，不执行控制和解析。
 *
 * @return void
 */
void takes_high_priority_notify_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (NULL != g_chassis_task_handle)
    {
        vTaskNotifyGiveFromISR(g_chassis_task_handle, &higher_priority_task_woken);
    }

    if (NULL != g_radar_task_handle)
    {
        vTaskNotifyGiveFromISR(g_radar_task_handle, &higher_priority_task_woken);
    }

    if (NULL != g_vision_task_handle)
    {
        vTaskNotifyGiveFromISR(g_vision_task_handle, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief  按键扫描任务入口
 *
 * 任务按 timer7 通知驱动 key.process() 扫描按键，并从 FIFO 读取事件打印到串口。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void key_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t now_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        now_ms += (notify_count * KEY_TIMER_PERIOD_MS);
        key.process(now_ms);
    }
}

/**
 * @brief  底盘闭环控制任务入口
 *
 * 任务按 timer4 高优先级通知累计 10ms 后调用一次 chassis.control_step()。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void chassis_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;
    fsp_err_t err = FSP_SUCCESS;
    fsp_err_t last_err = FSP_SUCCESS;
    bool control_enabled = false;
    bool last_control_enabled = false;
    ui_speed_control speed_control =
    {
        .vx_mps = 0.0f,
        .vy_mps = 0.0f,
        .wz_radps = 0.0f,
    };
    float wheel_speed_mps[CHASSIS_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    int32_t pwm_counts[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};
    chassis_control_mode_t control_mode = CHASSIS_CONTROL_MODE_BODY;
    chassis_control_mode_t last_control_mode = CHASSIS_CONTROL_MODE_BODY;
    mission_chassis_control_t mission_control =
    {
        .enabled = false,
        .use_avoidance = false,
        .vx_mps = 0.0f,
        .vy_mps = 0.0f,
        .wz_radps = 0.0f,
    };
    bool mission_active = false;
    bool last_mission_active = false;
    bool last_mission_use_avoidance = false;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * HIGH_TIMER_PERIOD_MS);
        while (elapsed_ms >= CHASSIS_TASK_PERIOD_MS)
        {
            elapsed_ms -= CHASSIS_TASK_PERIOD_MS;

            mission_active = mission.chassis_control_read(&mission_control);
            if (mission_active)
            {
                if (!last_mission_active)
                {
                    chassis.stop();
                    chassis_avoidance.reset();
                    last_err = FSP_SUCCESS;
                }
                else if (last_mission_use_avoidance != mission_control.use_avoidance)
                {
                    if (last_mission_use_avoidance)
                    {
                        chassis_avoidance.stop();
                    }
                    if (mission_control.use_avoidance)
                    {
                        chassis_avoidance.reset();
                    }
                    chassis.stop();
                    last_err = FSP_SUCCESS;
                }

                last_mission_active = true;
                last_mission_use_avoidance = mission_control.use_avoidance;
                last_control_enabled = false;
                last_control_mode = CHASSIS_CONTROL_MODE_BODY;

                if (mission_control.use_avoidance)
                {
                    err = chassis_avoidance.control(mission_control.vx_mps,
                                                    mission_control.vy_mps,
                                                    mission_control.wz_radps,
                                                    CHASSIS_TASK_PERIOD_MS);
                    if (FSP_SUCCESS == err)
                    {
                        err = chassis.control_step(CHASSIS_TASK_PERIOD_MS);
                    }
                    else
                    {
                        chassis_avoidance.stop();
                        chassis.stop();
                    }
                }
                else
                {
                    chassis.set_velocity(mission_control.vx_mps,
                                         mission_control.vy_mps,
                                         mission_control.wz_radps);
                    err = chassis.control_step(CHASSIS_TASK_PERIOD_MS);
                    if (FSP_SUCCESS != err)
                    {
                        chassis.stop();
                    }
                }

                if ((FSP_SUCCESS != err) && (last_err != err))
                {
                    printf("chassis:mission_step:fail:%d\r\n", (int)err);
                }
                last_err = err;
                continue;
            }

            if (last_mission_active)
            {
                chassis_avoidance.stop();
                chassis.stop();
                last_mission_active = false;
                last_mission_use_avoidance = false;
                last_control_enabled = false;
                last_control_mode = CHASSIS_CONTROL_MODE_BODY;
                last_err = FSP_SUCCESS;
            }

            control_enabled = display_speed_control.read(&speed_control,
                                                         wheel_speed_mps,
                                                         pwm_counts,
                                                         &control_mode);
            if (!control_enabled)
            {
                if (last_control_enabled)
                {
                    chassis_avoidance.stop();
                    chassis.stop();
                    last_control_enabled = false;
                    last_control_mode = CHASSIS_CONTROL_MODE_BODY;
                }
                continue;
            }

            if (!last_control_enabled)
            {
                /* 进入调参页后先清掉上一次 PI 积分，避免旧输出直接冲到电机。 */
                chassis.stop();
                if (CHASSIS_CONTROL_MODE_BODY == control_mode)
                {
                    chassis_avoidance.reset();
                }
                last_control_enabled = true;
                last_control_mode = control_mode;
                last_err = FSP_SUCCESS;
            }
            else if (last_control_mode != control_mode)
            {
                if (CHASSIS_CONTROL_MODE_BODY == last_control_mode)
                {
                    chassis_avoidance.stop();
                }
                if (CHASSIS_CONTROL_MODE_BODY == control_mode)
                {
                    chassis_avoidance.reset();
                }
                last_control_mode = control_mode;
                last_err = FSP_SUCCESS;
            }

            if (CHASSIS_CONTROL_MODE_PWM == control_mode)
            {
                chassis.set_pwm(pwm_counts);
                err = chassis.control_step(CHASSIS_TASK_PERIOD_MS);
            }
            else if (CHASSIS_CONTROL_MODE_WHEEL == control_mode)
            {
                chassis.set_wheel_speed(wheel_speed_mps);
                err = chassis.control_step(CHASSIS_TASK_PERIOD_MS);
            }
            else
            {
                err = chassis_avoidance.control(speed_control.vx_mps,
                                                speed_control.vy_mps,
                                                speed_control.wz_radps,
                                                CHASSIS_TASK_PERIOD_MS);
                if (FSP_SUCCESS == err)
                {
                    err = chassis.control_step(CHASSIS_TASK_PERIOD_MS);
                }
                else
                {
                    chassis_avoidance.stop();
                    chassis.stop();
                }
            }

            if (FSP_SUCCESS != err)
            {
                if (CHASSIS_CONTROL_MODE_BODY == control_mode)
                {
                    chassis_avoidance.stop();
                }
                else
                {
                    chassis.stop();
                }
                if (last_err != err)
                {
                    printf("chassis:step:fail:%d\r\n", (int)err);
                }
            }
            last_err = err;
        }
    }
}

/**
 * @brief  N10 雷达后台处理任务入口
 *
 * 任务按 timer4 通知累计 10ms 后处理 UART6 接收缓存并发布完整雷达容器。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void radar_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * HIGH_TIMER_PERIOD_MS);
        while (elapsed_ms >= RADAR_TASK_PERIOD_MS)
        {
            elapsed_ms -= RADAR_TASK_PERIOD_MS;
            (void)n10_radar.process();
        }
    }
}

/**
 * @brief  视觉 UART8 后台处理任务入口
 *
 * 任务按 timer4 通知累计 10ms 后处理 UART8 接收缓存，并同步推进 mission 状态机。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void vision_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * HIGH_TIMER_PERIOD_MS);
        while (elapsed_ms >= VISION_TASK_PERIOD_MS)
        {
            elapsed_ms -= VISION_TASK_PERIOD_MS;
            (void)vision_uart8.process();
            mission.process(VISION_TASK_PERIOD_MS);
        }
    }
}

/**
 * @brief  电源采样任务入口
 *
 * 任务按 timer7 通知每 100ms 读取一次 INA226。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void power_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * KEY_TIMER_PERIOD_MS);
        if (elapsed_ms >= POWER_TASK_PERIOD_MS)
        {
            elapsed_ms = 0U;

            (void)ina226.update();
        }
    }
}

/**
 * @brief  机械臂舵机反馈任务入口
 *
 * 任务按 timer7 通知累计 20ms 后轮询一路舵机真实位置、温度和电压。
 * 若舵机读取超时，不补跑积压通知，避免低优先级任务追赶式阻塞系统。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void mechanical_arm_feedback_task_entry(void *pvParameters)
{
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * KEY_TIMER_PERIOD_MS);
        if (elapsed_ms >= MECHANICAL_ARM_FEEDBACK_TASK_PERIOD_MS)
        {
            elapsed_ms = 0U;
            mechanical_arm_feedback.process_100ms_tick();
        }
    }
}

/**
 * @brief  系统监视任务入口
 *
 * 任务按 timer7 通知每 5s 打印各任务栈余量及 INA226 电压电流。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void monitor_task_entry(void *pvParameters)
{
    ina226_sample_t sample;
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms += (notify_count * KEY_TIMER_PERIOD_MS);
        if (elapsed_ms >= MONITOR_TASK_PERIOD_MS)
        {
            elapsed_ms = 0U;

            printf("[monitor]\r\n"
                   "key stack_free:%lu\r\n"
                   "power stack_free:%lu\r\n"
                   "arm_feedback stack_free:%lu\r\n"
                   "chassis stack_free:%lu\r\n"
                   "vision stack_free:%lu\r\n"
                   "ui stack_free:%lu\r\n",
                   (unsigned long)uxTaskGetStackHighWaterMark(g_key_task_handle),
                   (unsigned long)uxTaskGetStackHighWaterMark(g_power_task_handle),
                   (NULL != g_mechanical_arm_feedback_task_handle) ?
                   (unsigned long)uxTaskGetStackHighWaterMark(g_mechanical_arm_feedback_task_handle) : 0UL,
                   (NULL != g_chassis_task_handle) ?
                   (unsigned long)uxTaskGetStackHighWaterMark(g_chassis_task_handle) : 0UL,
                   (unsigned long)uxTaskGetStackHighWaterMark(g_vision_task_handle),
                   (unsigned long)uxTaskGetStackHighWaterMark(g_ui_task_handle));

            if (FSP_SUCCESS == ina226.read(&sample))
            {
                int32_t curr_ua_abs = sample.current_ua;

                if (curr_ua_abs < 0)
                {
                    curr_ua_abs = -curr_ua_abs;
                }

                printf("ina226:bus=%lu.%03luV curr=%ld.%03ldA\r\n",
                       (unsigned long)(sample.bus_voltage_mv / 1000U),
                       (unsigned long)(sample.bus_voltage_mv % 1000U),
                       (long)(sample.current_ua / 1000000L),
                       (long)((curr_ua_abs / 1000L) % 1000L));
            }
        }
    }
}

/**
 * @brief  UI 周期任务入口
 *
 * 任务只负责初始化 LCD 开机页，并按 timer7 通知推进 LVGL 刷新。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void ui_task_entry(void *pvParameters)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t notify_count = 0U;
    uint32_t elapsed_ms = 0U;

    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(UI_LCD_POWER_ON_DELAY_MS));

    err = lcd_show.init();
    if (FSP_SUCCESS != err)
    {
        printf("lcd_show:init:fail:%d\r\n", (int)err);
        vTaskDelete(NULL);
    }

    g_ui_task_ready = true;
    printf("lcd_show:init:ok\r\n");

    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (0U == notify_count)
        {
            continue;
        }

        elapsed_ms = notify_count * UI_TIMER_PERIOD_MS;
        lcd_show.process_1ms_tick(elapsed_ms);
    }
}

/**
 * @brief  语音 AI 后台任务入口
 *
 * 任务按固定周期调用 voice_ai.process()，只负责调度，不在 takes.c 中处理识别结果。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void voice_ai_task_entry(void *pvParameters)
{
    TickType_t last_wake = 0U;

    (void)pvParameters;

    last_wake = xTaskGetTickCount();
    for (;;)
    {
        (void)voice_ai.process();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(VOICE_AI_TASK_PERIOD_MS));
    }
}

/**
 * @brief  初始化任务入口
 *
 * 初始化串口、按键和 INA226，创建系统任务，启动周期定时器后删除自身。
 *
 * @param pvParameters FreeRTOS 任务参数，未使用
 * @return void
 */
static void init_task_entry(void *pvParameters)
{
    BaseType_t ok = pdPASS;
    fsp_err_t err = FSP_SUCCESS;
    chassis_pid_params_t pid_params =
    {
        .kp = CHASSIS_PID_DEFAULT_KP,
        .ki = CHASSIS_PID_DEFAULT_KI,
    };
    bool chassis_ready = false;
    bool radar_ready = false;
    bool mission_ready = false;
    int home_err = MECH_HAND_OK;

    (void)pvParameters;

    uart.init();
    printf("uart_init_ok\r\n");

    key.init();
    ina226.init();

    err = mechanical_arm_feedback.init();
    if (FSP_SUCCESS == err)
    {
        printf("mechanical_hand:init:ok\r\n");
        if (mechanical_arm_feedback.lock_bus(MECH_HAND_INIT_HOME_LOCK_TIMEOUT_MS))
        {
            home_err = mechanical_hand.home(MECH_HAND_INIT_HOME_TIME_MS);
            mechanical_arm_feedback.unlock_bus();
            mechanical_arm_feedback.request_refresh_all();
            if (MECH_HAND_OK == home_err)
            {
                printf("mechanical_hand:home:ok\r\n");
            }
            else
            {
                printf("mechanical_hand:home:fail:%d\r\n", home_err);
            }
        }
        else
        {
            printf("mechanical_hand:home:lock_fail\r\n");
        }
    }
    else
    {
        printf("mechanical_hand:init:fail:%d\r\n", mechanical_hand.get_last_error());
    }

    err = n10_radar.init();
    if (FSP_SUCCESS == err)
    {
        radar_ready = true;
        printf("n10:init:ok\r\n");
    }
    else
    {
        printf("n10:init:fail:%d\r\n", (int)err);
    }

    err = chassis.init();
    if (FSP_SUCCESS == err)
    {
        chassis_ready = true;
        printf("chassis:init:ok\r\n");

        err = chassis_avoidance.init();
        if (FSP_SUCCESS == err)
        {
            printf("chassis_avoidance:init:ok\r\n");
        }
        else
        {
            printf("chassis_avoidance:init:fail:%d\r\n", (int)err);
        }

        err = chassis_param.load(&pid_params);
        if (FSP_SUCCESS == err)
        {
            chassis.set_pid(pid_params.kp, pid_params.ki);
            printf("chassis:param:load:ok\r\n");
        }
        else
        {
            printf("chassis:param:default:%d\r\n", (int)err);
        }
    }
    else
    {
        printf("chassis:init:fail:%d\r\n", (int)err);
    }

    err = mission.init();
    mission_ready = true;
    if (FSP_SUCCESS == err)
    {
        printf("mission:init:ok\r\n");
    }
    else
    {
        printf("mission:init:warn:%d\r\n", (int)err);
    }

    err = voice_ai.init();
    if (FSP_SUCCESS == err)
    {
        printf("voice_ai:init:ok\r\n");
    }
    else
    {
        printf("voice_ai:init:fail:%d\r\n", (int)err);
    }

    if (FSP_SUCCESS == err)
    {
        ok = xTaskCreate(voice_ai_task_entry,
                         "voice_ai",
                         VOICE_AI_TASK_STACK_SIZE,
                         NULL,
                         VOICE_AI_TASK_PRIORITY,
                         &g_voice_ai_task_handle);
        if (pdPASS != ok)
        {
            printf("voice_ai_task:create:fail\r\n");
        }
    }

    ok = xTaskCreate(key_task_entry,
                     "key_task",
                     KEY_TASK_STACK_SIZE,
                     NULL,
                     KEY_TASK_PRIORITY,
                     &g_key_task_handle);
    if (pdPASS != ok)
    {
        printf("key_task:create:fail\r\n");
    }

    ok = xTaskCreate(power_task_entry,
                     "power_task",
                     POWER_TASK_STACK_SIZE,
                     NULL,
                     POWER_TASK_PRIORITY,
                     &g_power_task_handle);
    if (pdPASS != ok)
    {
        printf("power_task:create:fail\r\n");
    }

    ok = xTaskCreate(mechanical_arm_feedback_task_entry,
                     "arm_feedback",
                     MECHANICAL_ARM_FEEDBACK_TASK_STACK_SIZE,
                     NULL,
                     MECHANICAL_ARM_FEEDBACK_TASK_PRIORITY,
                     &g_mechanical_arm_feedback_task_handle);
    if (pdPASS != ok)
    {
        printf("arm_feedback_task:create:fail\r\n");
    }

    if (chassis_ready)
    {
        ok = xTaskCreate(chassis_task_entry,
                         "chassis",
                         CHASSIS_TASK_STACK_SIZE,
                         NULL,
                         CHASSIS_TASK_PRIORITY,
                         &g_chassis_task_handle);
        if (pdPASS != ok)
        {
            printf("chassis_task:create:fail\r\n");
        }
    }

    if (radar_ready)
    {
        ok = xTaskCreate(radar_task_entry,
                         "n10_radar",
                         RADAR_TASK_STACK_SIZE,
                         NULL,
                         RADAR_TASK_PRIORITY,
                         &g_radar_task_handle);
        if (pdPASS != ok)
        {
            printf("n10_task:create:fail\r\n");
        }
    }

    err = vision_uart8.init();
    if (FSP_SUCCESS == err)
    {
        printf("vision_uart8:init:ok\r\n");

        ok = xTaskCreate(vision_task_entry,
                         "vision",
                         VISION_TASK_STACK_SIZE,
                         NULL,
                         VISION_TASK_PRIORITY,
                         &g_vision_task_handle);
        if (pdPASS != ok)
        {
            printf("vision_task:create:fail\r\n");
        }

    }
    else
    {
        printf("vision_uart8:init:fail:%d\r\n", (int)err);
    }

    ok = xTaskCreate(monitor_task_entry,
                     "monitor",
                     MONITOR_TASK_STACK_SIZE,
                     NULL,
                     MONITOR_TASK_PRIORITY,
                     &g_monitor_task_handle);
    if (pdPASS != ok)
    {
        printf("monitor_task:create:fail\r\n");
    }

    ok = xTaskCreate(ui_task_entry,
                     "ui_task",
                     UI_TASK_STACK_SIZE,
                     NULL,
                     UI_TASK_PRIORITY,
                     &g_ui_task_handle);
    if (pdPASS != ok)
    {
        printf("ui_task:create:fail\r\n");
    }

    Timer_Init();
		printf("Timer_init_ok\r\n");
    vTaskDelete(NULL);
}

/**
 * @brief  创建系统初始化任务
 *
 * @return BaseType_t pdPASS 表示任务创建成功
 */
BaseType_t takes_create_tasks(void)
{
    return xTaskCreate(init_task_entry,
                       "init_task",
                       INIT_TASK_STACK_SIZE,
                       NULL,
                       INIT_TASK_PRIORITY,
                       NULL);
}

/**
 * @brief  FreeRTOS malloc 失败钩子
 *
 * @return void
 */
void vApplicationMallocFailedHook(void)
{
    printf("malloc_failed\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/**
 * @brief  FreeRTOS 栈溢出钩子
 *
 * @param xTask      溢出的任务句柄
 * @param pcTaskName 溢出的任务名称
 * @return void
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("stack_overflow:%s\r\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
