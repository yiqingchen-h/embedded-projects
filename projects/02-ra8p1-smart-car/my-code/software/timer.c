#include "timer.h"
#include "software/takes.h"

/**
 * @brief  初始化并启动系统周期定时器
 *
 * @return fsp_err_t FSP_SUCCESS 表示定时器启动成功
 */
fsp_err_t Timer_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = g_timer7.p_api->open(g_timer7.p_ctrl, g_timer7.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        return err;
    }

    err = g_timer7.p_api->start(g_timer7.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = g_timer4.p_api->open(g_timer4.p_ctrl, g_timer4.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        (void)g_timer7.p_api->stop(g_timer7.p_ctrl);
        (void)g_timer7.p_api->close(g_timer7.p_ctrl);
        return err;
    }

    err = g_timer4.p_api->start(g_timer4.p_ctrl);
    if (FSP_SUCCESS != err)
    {
        (void)g_timer4.p_api->close(g_timer4.p_ctrl);
        (void)g_timer7.p_api->stop(g_timer7.p_ctrl);
        (void)g_timer7.p_api->close(g_timer7.p_ctrl);
        return err;
    }

    return FSP_SUCCESS;
}

/**
 * @brief  停止并关闭系统周期定时器
 *
 * @return void
 */
void Timer_Deinit(void)
{
    (void)g_timer4.p_api->stop(g_timer4.p_ctrl);
    (void)g_timer4.p_api->close(g_timer4.p_ctrl);
    (void)g_timer7.p_api->stop(g_timer7.p_ctrl);
    (void)g_timer7.p_api->close(g_timer7.p_ctrl);
}

/**
 * @brief  GPT7 低优先级用户周期定时器回调
 *
 * 中断中只通知周期任务，不读取 GPIO、不读取 INA226、不处理 UI。
 *
 * @param p_args FSP 定时器回调参数
 * @return void
 */
void timer7_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    takes_key_notify_from_isr();
}

/**
 * @brief  GPT4 高优先级周期定时器回调
 *
 * 中断中只通知底盘闭环和 N10 雷达后台任务，不执行控制或解析。
 *
 * @param p_args FSP 定时器回调参数
 * @return void
 */
void timer4_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    takes_high_priority_notify_from_isr();
}
