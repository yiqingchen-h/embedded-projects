#ifndef FOC_TEST_LOGIC_H
#define FOC_TEST_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
/** @brief 12位ADC按13倍分压换算。 @param raw 原始值0～4095。 @param vref_mv 实测参考mV。 @return 母线或相电压mV。 */
uint32_t test_voltage_mv(uint16_t raw, uint32_t vref_mv);
/** @brief 检查本次12V空载脉冲的前置条件。 @param adc ADC有效。 @param driver 驱动通信和配置有效。 @param csa 三路零点有效。 @param pg 电源PG高。 @param fault 驱动存在故障。 @param bus_mv 母线mV。 @return 全部通过且10～14V为true。 */
bool test_power_allowed(bool adc, bool driver, bool csa, bool pg, bool fault, uint32_t bus_mv);
/** @brief 检查电流采样零点与静态噪声，不能验证实际增益。 @param mean 平均ADC值。 @param span 极差ADC值。 @return 位于量程35%～65%且极差<=32码为true。 */
bool test_csa_valid(uint16_t mean, uint16_t span);
/** @brief 检查高侧相电压跟随。 @param phase_raw 相ADC。 @param bus_raw 母线ADC。 @return 相电压在母线80%～110%内为true。 */
bool test_high_side_valid(uint16_t phase_raw, uint16_t bus_raw);
#endif
