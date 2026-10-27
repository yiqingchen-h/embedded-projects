#ifndef VISION_UART9_COMM_H_
#define VISION_UART9_COMM_H_

#include <stdint.h>

/*
 * UART9 视觉通信模块
 *
 * 这个模块只负责 Ubuntu 和 MCU 之间通过 UART9 进行的数据收发，不直接参与：
 * 1. 标定换算
 * 2. 机械手执行
 * 3. 底盘转向控制
 *
 * 模块职责刻意保持单一：
 * 1. 接收：收字节、组帧、校验、解析、缓存、置新数据标志
 * 2. 发送：把最多 4 个 float 数据拼成固定协议帧并发出
 *
 * 串口层缓存接收结果，业务层读取后决定是否进行标定和执行。
 */

/* 通用返回码定义。 */
#define VISION_UART9_OK            (0)
#define VISION_UART9_ERR_PARAM     (-1)
#define VISION_UART9_ERR_OPEN      (-2)
#define VISION_UART9_ERR_NOT_READY (-3)
#define VISION_UART9_ERR_BUSY      (-4)
#define VISION_UART9_ERR_WRITE     (-5)
#define VISION_UART9_ERR_TIMEOUT   (-6)

/*
 * 最近一帧有效数据及统计信息。
 *
 * 字段说明：
 * 1. u / v / depth / yaw
 *    接收成功后解析出的 4 个浮点字段。
 *
 * 2. frame_valid
 *    是否至少成功接收并解析过一帧有效数据。
 *    0 表示还没有成功帧，1 表示已有至少一帧有效数据。
 *
 * 3. new_data_flag
 *    是否存在“尚未被上层消费”的新数据。
 *    模块每次成功收新帧都会置 1，由上层在读取后手动清零。
 *
 * 4. tick_ms
 *    最近一帧成功解析时的系统时间戳，单位 ms。
 *
 * 5. rx_* 统计项
 *    便于调试串口质量、协议错误、缓存溢出等问题。
 *
 * 6. tx_* 统计项
 *    便于观察发送是否繁忙、成功次数、失败次数。
 */
typedef struct
{
    float u;
    float v;
    float depth;
    float yaw;

    uint8_t frame_valid;
    uint8_t new_data_flag;
    uint32_t tick_ms;

    uint32_t rx_ok_count;
    uint32_t rx_checksum_error_count;
    uint32_t rx_format_error_count;
    uint32_t rx_overflow_count;

    uint8_t tx_busy;
    uint32_t tx_ok_count;
    uint32_t tx_error_count;
} vision_uart9_frame_t;

/*
 * 初始化 UART9 通信模块。
 *
 * 作用：
 * 1. 清空内部接收环形缓冲区
 * 2. 清空最近一帧缓存和状态机
 * 3. 打开 FSP 已配置好的 g_uart9
 *
 * 说明：
 * 1. 这个函数应在系统初始化阶段调用一次
 * 2. 如果 UART9 已经打开，函数会按成功处理，不会重复报错
 */
int VisionUart9_Init(void);

/*
 * 接收处理轮询函数。
 *
 * 作用：
 * 1. 从 UART9 的环形缓冲区取出已接收到的字节
 * 2. 按固定长度协议组帧
 * 3. 完成起始位、结束位、校验位、字段格式检查
 * 4. 若帧有效，则更新最新缓存并置 new_data_flag
 *
 * 使用要求：
 * 1. 该函数不能省略，否则虽然中断在收字节，但不会解析成有效数据
 * 2. 建议在主循环或周期任务中频繁调用
 */
void VisionUart9_Process(void);

/*
 * 读取最近一次缓存的数据快照。
 *
 * 参数：
 * 1. frame：调用者提供的输出结构体指针
 *
 * 说明：
 * 1. 这里返回的是“最近一次缓存副本”
 * 2. 读取本身不会自动清 new_data_flag
 * 3. 如果希望“读完即清空”，需要额外调用 VisionUart9_ClearNewDataFlag()
 */
int VisionUart9_GetLatest(vision_uart9_frame_t *frame);

/*
 * 查询是否有尚未使用的新数据。
 *
 * 返回：
 * 1. 0：没有新数据
 * 2. 1：有新数据
 */
uint8_t VisionUart9_HasNewData(void);

/*
 * 清除“有新数据”标志。
 *
 * 说明：
 * 1. 该函数不会清除缓存内容
 * 2. 只会把 new_data_flag 清零
 */
void VisionUart9_ClearNewDataFlag(void);

/*
 * 发送最多 4 个浮点数据。
 *
 * 参数：
 * 1. values：待发送 float 数组首地址
 * 2. length：实际传入的数据个数
 *
 * 发送规则：
 * 1. length == 0：四个字段全部补 0
 * 2. 1 <= length <= 4：不足的字段补 0
 * 3. length > 4：只取前 4 个
 *
 * 协议格式：
 * @ + 4个固定宽度字段 + 2位XOR校验 + #
 */
int VisionUart9_Send4(const float *values, uint8_t length);

/*
 * 测试注入接口。
 *
 * 作用：
 * 1. 按正常 UART9 协议格式构造一帧测试数据。
 * 2. 直接把构造好的字节灌入 UART9 接收环形缓冲区。
 * 3. 后续仍然需要调用 VisionUart9_Process() 才会完成真正的组帧和解析。
 *
 * 设计目的：
 * 1. 让上层可以在不接真实视觉串口的情况下，测试“接收 -> 解析 -> 控制”整条链路。
 * 2. 保持测试路径和真实路径一致，避免测试时绕过协议解析层。
 */
int VisionUart9_TestInjectFrame(float u, float v, float depth, float yaw);

#endif
