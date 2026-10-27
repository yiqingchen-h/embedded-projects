#include "board.h"
#include "board_test.h"
#include "board_i2c.h"
#include "selftest.h"
#include "stm32g4xx_hal.h"

/** @brief 独立整板自检入口，不运行电机算法。 @return 正常不返回。 */
int
main(void)
{
    board_init();
    board_i2c_init();
    board_test_init_t initialized = board_test_init();
    selftest_init(initialized);
    if (!board_watchdog_init())
    {
        board_uart_write("IWDG INIT FAIL; OUTPUTS DISABLED\r\n");
        /* 初始化失败已关断输出；不再喂狗，等待复位。 */
        while (1) { __WFI(); }
    }
    board_uart_write("IWDG READY; MAIN LOOP RUNNING\r\n");
    while (1)
    {
        selftest_poll();
        board_watchdog_feed();
        __WFI();
    }
}
