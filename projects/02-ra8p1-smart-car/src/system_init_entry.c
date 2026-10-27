#include "../my-code/software/takes.h"
#include "system_init.h"
/* system_init entry function */
/* pvParameters contains TaskHandle_t */
void system_init_entry(void *pvParameters)
{
    BaseType_t err;
    FSP_PARAMETER_NOT_USED(pvParameters);

    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 临时提到最高优先级，避免被同级/低级任务打断 */
    UBaseType_t old_prio = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, (UBaseType_t)(configMAX_PRIORITIES - 1U));

    /* 恢复原优先级 */
    vTaskPrioritySet(NULL, old_prio);
    err = takes_create_tasks();
    if (pdPASS != err)
    {
        // 创建错误处理逻辑
    }
    // 删除自身
    vTaskDelete(NULL);
}
