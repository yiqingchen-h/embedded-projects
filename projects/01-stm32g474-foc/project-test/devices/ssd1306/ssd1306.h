#ifndef FOC_SSD1306_H
#define FOC_SSD1306_H
#include <stdbool.h>
#include <stdint.h>
/** I2C 写入适配函数：7 位地址、控制字节、数据、长度；成功返回 true。 */
typedef bool (*ssd1306_write_t)(uint8_t, uint8_t, const uint8_t *, uint16_t);
/** @brief 初始化 128x64 SSD1306，尝试 0x3C/0x3D。 @param write 总线函数。 @return 所有初始化写入成功返回 true，不代表型号已经识别。 */
bool ssd1306_init(ssd1306_write_t write);
/** @brief 清除本地画面，不访问总线。 */
void ssd1306_clear(void);
/** @brief 填充全屏测试图案。 @param pattern 0全灭、1全亮、2棋盘。 */
void ssd1306_pattern(uint8_t pattern);
/** @brief 写一行 5x7 英文与数字。 @param row 行号0～7。 @param text 文本，最多显示21字符，不支持字符显示问号。 */
void ssd1306_text(uint8_t row, const char *text);
/** @brief 刷新全部8页。 @return 全部数据收到应答返回 true。 */
bool ssd1306_refresh(void);
/** @brief 获取选中的7位地址。 @return 0表示初始化未成功。 */
uint8_t ssd1306_address(void);
#endif
