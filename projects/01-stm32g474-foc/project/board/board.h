#ifndef FOC_BOARD_H
#define FOC_BOARD_H

/**
 * @brief 初始化 HAL、本板 24 MHz HSE/170 MHz 系统时钟及 PB9 低电平输出。
 * @note 在主函数中调用；初始化失败时保持驱动禁用并停止执行。
 */
void board_init(void);

#endif
