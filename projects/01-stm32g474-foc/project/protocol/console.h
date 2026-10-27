#ifndef FOC_CONSOLE_H
#define FOC_CONSOLE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 最大命令长度，不包含行结束符。 */
#define CONSOLE_LINE_MAX 96U
/* 单条完整响应的缓冲容量，包含结尾零字节。 */
#define CONSOLE_RESPONSE_MAX 512U
typedef struct {
    char line[CONSOLE_LINE_MAX + 1U]; /* 正在收集的命令。 */
    uint16_t length; /* 当前有效字符数。 */
    bool invalid; /* 非法字符或通信损坏。 */
    bool too_long; /* 超过命令上限。 */
} console_t;
/** @brief 命令执行器；仅在主循环调用。
 * @param argc 参数数。 @param argv 大写的参数数组。
 * @param response 输出响应，不含换行。 @param capacity 输出容量。 */
typedef void (*console_handler_t)(unsigned argc, char **argv, char *response, size_t capacity);
/** @brief 初始化命令接收器。 @param console 接收器。 */
void console_init(console_t *console);
/** @brief 输入一个字节或损坏标记，收到换行时生成一条完整响应。
 * @param console 接收器。 @param token 低八位为字节，bit8 表示本行损坏。
 * @param handler 命令执行器。 @param response 至少 CONSOLE_RESPONSE_MAX 字节。
 * @return 一条响应已就绪返回 true。 */
bool console_feed(console_t *console, uint16_t token, console_handler_t handler, char *response);
#endif
