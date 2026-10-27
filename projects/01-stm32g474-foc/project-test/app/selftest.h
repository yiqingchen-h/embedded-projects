#ifndef FOC_SELFTEST_H
#define FOC_SELFTEST_H
#include "board_test.h"
/** @brief 自动执行通信、GPIO及静态采样测试，桥臂脉冲仅在ENABLE低时进行。 @param initialized 板卡外设初始化结果。 */
void selftest_init(board_test_init_t initialized);
/** @brief 主循环服务串口、OLED和低速采样；手动命令才执行功率输出。 */
void selftest_poll(void);
#endif
