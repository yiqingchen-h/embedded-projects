#include "ADC.h"

void ADC_init(void)
{

    g_adc3.p_api->open(g_adc3.p_ctrl, g_adc3.p_cfg);
    g_adc3.p_api->scanCfg(g_adc3.p_ctrl, g_adc3.p_channel_cfg); // 初始化ADC通道
}
// 等待ADC转换完成
static void ADCWaitConvCplt(void)
{
    adc_status_t status;
    status.state = ADC_STATE_SCAN_IN_PROGRESS;

    while (ADC_STATE_SCAN_IN_PROGRESS == status.state)
    {
        g_adc3.p_api->scanStatusGet(g_adc3.p_ctrl, &status);
    }
}

int ADC_Read(unsigned short *value, unsigned short num)
{
    if (NULL == value || num <= 0)
        return -1;

    for (uint16_t i = 0; i < num; i++)
    {
        fsp_err_t err = g_adc3.p_api->scanStart(g_adc3.p_ctrl);
        assert(FSP_SUCCESS == err);
        ADCWaitConvCplt();
        err = g_adc3.p_api->read(g_adc3.p_ctrl, ADC_CHANNEL_3, &value[i]);
        assert(FSP_SUCCESS == err);
    }

    return 0;
}
