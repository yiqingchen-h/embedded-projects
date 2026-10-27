#include "test_logic.h"

/** @brief 见 test_logic.h。 */
uint32_t
test_voltage_mv(uint16_t raw, uint32_t vref_mv)
{
    return (uint32_t)(((uint64_t)raw * vref_mv * 13U + 2047U) / 4095U);
}

/** @brief 见 test_logic.h。 */
bool
test_power_allowed(bool adc, bool driver, bool csa, bool pg, bool fault, uint32_t bus_mv)
{
    return adc && driver && csa && pg && !fault && bus_mv >= 10000U && bus_mv <= 14000U;
}

/** @brief 见 test_logic.h。 */
bool
test_csa_valid(uint16_t mean, uint16_t span)
{
    return mean >= 1433U && mean <= 2662U && span <= 32U;
}

/** @brief 见 test_logic.h。 */
bool
test_high_side_valid(uint16_t phase_raw, uint16_t bus_raw)
{
    return bus_raw > 0U && (uint32_t)phase_raw * 100U >= (uint32_t)bus_raw * 80U &&
           (uint32_t)phase_raw * 100U <= (uint32_t)bus_raw * 110U;
}
