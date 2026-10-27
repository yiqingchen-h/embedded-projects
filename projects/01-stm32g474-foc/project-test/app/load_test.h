#ifndef FOC_LOAD_TEST_H
#define FOC_LOAD_TEST_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化独立负载诊断状态，不访问功率级；仅在系统启动调用。 */
void load_test_init(void);
/** @brief 请求准备24.5Ω的A-B负载测试。 @param allowed 原自检时钟、ADC、供电、DRV、PWM均通过。 @param now 当前毫秒。 @return 接受请求；不代表准备完成。 */
bool load_test_check(bool allowed, uint32_t now);
/** @brief 请求一次1秒输出；需要READY和冷却完成。 @param now 当前毫秒。 @return 接受请求。 */
bool load_test_run(uint32_t now);
/** @brief 每轮主循环推进一次，内部限制每毫秒最多采样一次。 @param now 当前毫秒。 */
void load_test_poll(uint32_t now);
/** @brief 停止并退出负载模式，保留本次上电的冷却时间；仅空闲时执行RETEST前调用。 */
void load_test_exit(void);
/** @brief 是否已进入负载模式，含结束/失败状态；用于隔离原功率命令。 @return 模式状态。 */
bool load_test_mode(void);
/** @brief 是否正在准备、采样或输出；用于禁止阻塞日志及重入。 @return 活动状态。 */
bool load_test_busy(void);
/** @brief 查询阶段或打印完成结果；活动时异步发送短状态，完整统计只在关断后打印。 */
void load_test_report(void);
#endif
