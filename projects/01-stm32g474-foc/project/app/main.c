#include "board.h"
#include "board_io.h"
#include "board_encoder.h"
#include "interaction.h"
#include "motor_backend.h"
#include "stm32g4xx.h"

/** @brief 初始化交互与电机后端，只有显式 START 请求才允许输出。 @return 正常运行不返回。 */
int
main(void)
{
    board_init();
    board_encoder_init();
    bool uart_online = board_io_init();
    interaction_init(uart_online);
    motor_backend_init();
    while (1)
    {
        /* 串口无待处理输入时等待中断，电机硬实时控制继续由ADC/TIM推进。 */
        board_uart_service();
        motor_backend_service(board_millis());
        interaction_service();
        board_encoder_service();
        if (!interaction_pending()) { __WFI(); }
    }
}
