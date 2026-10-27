#ifndef FOC_BOARD_ENCODER_H
#define FOC_BOARD_ENCODER_H
#include "encoder_readout.h"
#include "encoder_six_step.h"
#include "encoder_position.h"
#include <stddef.h>

/** @brief 初始化SPI1/PA5/6/7、PC4片选及PC5 CAL低电平，不启用功率、不触发芯片标定。 */
void board_encoder_init(void);
/** @brief 主循环最多每毫秒读取一次角度，不增加ADC/TIM中断工作；失败后有限间隔重探测。 */
void board_encoder_service(void);
/** @brief 读取主循环观测快照。 @param now_ms 当前毫秒。 @return 含新鲜度与有效标记的快照。 */
encoder_snapshot_t board_encoder_snapshot(uint32_t now_ms);
/** @brief 格式化详细编码器状态，只读缓存，不在命令中等待SPI。
 * @param out 输出缓冲。 @param size 缓冲容量。 @param now_ms 当前毫秒。 */
void board_encoder_diagnostics(char *out, size_t size, uint32_t now_ms);
/** @brief 主循环且DMA采集已结束时修改迟滞易失影子，再双读更新诊断缓存。
 * @param code 4..7。 @return 成功且原码/带宽均验证；不改方向、零点或CAL引脚。 */
bool board_encoder_hysteresis(uint8_t code);
/** @brief 停机启用SPI1 DMA控制采集，独占SPI1；不输出功率、不做芯片标定。
 * @return 已探测到芯片且可启用。 */
bool board_encoder_control_begin(void);
/** @brief 每50us推进非阻塞DMA；六步用TIM峰顶，FOC用ADC保护采样，无等待或SPI轮询。
 * @return 最新完整CRC及磁场校验后的控制反馈。 */
encoder_six_feedback_t board_encoder_control_tick(void);
/** @brief 输出禁用后回收DMA并恢复主循环观测；释放片选，不修改PC5。 */
void board_encoder_control_end(void);
/** @brief 短临界区读取多圈坐标，返回值供调用者在临界区外换算。 */
encoder_position_t board_encoder_position(void);
/** @brief 主循环建立机械零点；调用方先核对功率关闭、静止、方向和标定。 */
bool board_encoder_position_zero(int8_t direction);
/** @brief 电角重新标定后撤销旧机械坐标资格，保留原始计数供诊断。 */
void board_encoder_position_invalidate(void);
#endif
