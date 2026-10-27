#include "Vision_Uart9_Comm.h"

#include "SysTick.h"
#include "hal_data.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*
 * ========================= 协议约定 =========================
 *
 * 固定收发协议如下：
 * 1. 起始位：'@'
 * 2. 数据区：4 个字段，每个字段固定 8 字符
 * 3. 校验位：2 个 ASCII 十六进制字符，表示数据区 32 字节的 XOR
 * 4. 截止位：'#'
 *
 * 单字段格式固定为：
 * [符号][4位整数].[2位小数]
 * 例如：
 * +0123.45
 * -0007.80
 *
 * 完整帧示例：
 * @+0123.45-0067.89+0430.00-0012.34A5#
 *
 * 说明：
 * 1. 接收和发送都使用这一套格式，便于 Ubuntu 和 MCU 两端统一解析
 * 2. 由于字段宽度固定，MCU 无需做复杂分隔符切割，解析更稳定
 */

/* 接收环形缓冲区大小。该值越大，越能抵抗短时间内主循环处理不及时。 */
#define VISION_UART9_RX_BUF_SIZE (256U)

/* 协议固定为 4 个字段。 */
#define VISION_UART9_FIELD_COUNT (4U)

/* 单个字段固定 8 字符。 */
#define VISION_UART9_FIELD_WIDTH (8U)

/* 数据区总长度 = 4 * 8 = 32 字节。 */
#define VISION_UART9_DATA_LEN (VISION_UART9_FIELD_COUNT * VISION_UART9_FIELD_WIDTH)

/* 总帧长 = 起始位1 + 数据区32 + 校验位2 + 截止位1 = 36 字节。 */
#define VISION_UART9_FRAME_LEN (1U + VISION_UART9_DATA_LEN + 2U + 1U)

/* 发送等待超时时间，单位 ms。 */
#define VISION_UART9_TX_TIMEOUT_MS (200U)

/* 单字段可编码的最大绝对值，超过后会被钳制。 */
#define VISION_UART9_FIELD_MAX_VALUE (9999.99f)

/*
 * UART9 接收环形缓冲区。
 *
 * 说明：
 * 1. UART9 中断里只往这里塞字节，不在中断中直接做组帧解析
 * 2. 主循环里调用 VisionUart9_Process() 再从这里取字节解析
 * 3. 这样可以让中断足够轻，降低打断业务逻辑的风险
 */
typedef struct
{
    volatile uint16_t read_index;
    volatile uint16_t write_index;
    uint8_t data[VISION_UART9_RX_BUF_SIZE];
} vision_uart9_ring_t;

/* 模块运行状态。 */
static volatile bool g_vision_uart9_initialized = false;
static volatile bool g_vision_uart9_tx_complete = false;
static volatile bool g_vision_uart9_tx_busy = false;

/* 接收缓冲、最近一帧缓存、当前组帧缓存。 */
static vision_uart9_ring_t g_vision_uart9_rx_ring = {0};
static vision_uart9_frame_t g_vision_uart9_latest = {0};
static char g_vision_uart9_rx_frame[VISION_UART9_FRAME_LEN] = {0};
static uint8_t g_vision_uart9_rx_index = 0U;

/* 私有函数声明。 */
static void VisionUart9_RingReset(void);
static void VisionUart9_RingPutByte(uint8_t byte);
static bool VisionUart9_RingGetByte(uint8_t *byte);
static uint8_t VisionUart9_CalcChecksum(const char *data, uint32_t length);
static char VisionUart9_NibbleToHex(uint8_t nibble);
static int VisionUart9_HexToNibble(char hex);
static bool VisionUart9_ParseField(const char *text, float *value);
static void VisionUart9_FormatField(float value, char *out_text);
static bool VisionUart9_TryParseFrame(const char *frame);
static int VisionUart9_SendFrame(const char *frame, uint32_t length);

/*
 * 初始化 UART9 通信模块。
 *
 * 执行内容：
 * 1. 清空接收环形缓冲区
 * 2. 清空最近一帧缓存
 * 3. 清空当前组帧状态
 * 4. 打开 g_uart9
 *
 * 特别说明：
 * 1. FSP 层 UART9 已经在 ra_gen 中配置好，这里只负责使用，不修改自动生成文件
 * 2. 如果 UART9 之前已打开，FSP 返回 FSP_ERR_ALREADY_OPEN 时仍视为可继续工作
 */
int VisionUart9_Init(void)
{
    fsp_err_t err;

    VisionUart9_RingReset();
    memset(&g_vision_uart9_latest, 0, sizeof(g_vision_uart9_latest));
    memset(g_vision_uart9_rx_frame, 0, sizeof(g_vision_uart9_rx_frame));
    g_vision_uart9_rx_index = 0U;
    g_vision_uart9_tx_complete = false;
    g_vision_uart9_tx_busy = false;

    err = g_uart9.p_api->open(g_uart9.p_ctrl, g_uart9.p_cfg);
    if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
    {
        g_vision_uart9_initialized = false;
        return VISION_UART9_ERR_OPEN;
    }

    g_vision_uart9_initialized = true;
    return VISION_UART9_OK;
}

/*
 * 接收处理函数。
 *
 * 处理逻辑：
 * 1. 不断从环形缓冲区取字节
 * 2. 只有遇到 '@' 才开始组一帧
 * 3. 组帧过程中如果再次遇到 '@'，说明前面那段数据失步，直接以新的 '@' 重新开始
 * 4. 当累计到 36 字节后，尝试按完整协议解析
 *
 * 这样做可以让模块在脏数据、半帧、失步场景下自动重新同步。
 */
void VisionUart9_Process(void)
{
    uint8_t byte;

    if (!g_vision_uart9_initialized)
    {
        return;
    }

    while (VisionUart9_RingGetByte(&byte))
    {
        if (0U == g_vision_uart9_rx_index)
        {
            if ('@' == (char)byte)
            {
                g_vision_uart9_rx_frame[0] = (char)byte;
                g_vision_uart9_rx_index = 1U;
            }

            continue;
        }

        if ('@' == (char)byte)
        {
            /* 重新同步到新的帧头。 */
            g_vision_uart9_rx_frame[0] = (char)byte;
            g_vision_uart9_rx_index = 1U;
            continue;
        }

        g_vision_uart9_rx_frame[g_vision_uart9_rx_index] = (char)byte;
        g_vision_uart9_rx_index++;

        if (g_vision_uart9_rx_index >= VISION_UART9_FRAME_LEN)
        {
            (void)VisionUart9_TryParseFrame(g_vision_uart9_rx_frame);
            g_vision_uart9_rx_index = 0U;
        }
    }
}

/*
 * 读取最近一帧缓存副本。
 *
 * 这里不会清除 new_data_flag，因为有些场景需要“重复读取同一帧但暂时不消费”。
 */
int VisionUart9_GetLatest(vision_uart9_frame_t *frame)
{
    if (NULL == frame)
    {
        return VISION_UART9_ERR_PARAM;
    }

    memcpy(frame, &g_vision_uart9_latest, sizeof(*frame));
    return VISION_UART9_OK;
}

/*
 * 查询是否存在新数据。
 */
uint8_t VisionUart9_HasNewData(void)
{
    return g_vision_uart9_latest.new_data_flag;
}

/*
 * 清除新数据标志。
 *
 * 注意：只清标志，不清内容。
 */
void VisionUart9_ClearNewDataFlag(void)
{
    g_vision_uart9_latest.new_data_flag = 0U;
}

/*
 * 发送最多 4 个 float 数据。
 *
 * 行为规则：
 * 1. length == 0：4 个字段全部补 0
 * 2. 1 <= length <= 4：按传入数量使用，后续字段补 0
 * 3. length > 4：只取前 4 个
 *
 * 发送格式与接收格式完全一致，方便两端共用同一套协议文档。
 */
int VisionUart9_Send4(const float *values, uint8_t length)
{
    float tx_values[VISION_UART9_FIELD_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    char frame[VISION_UART9_FRAME_LEN];
    uint8_t checksum;
    uint8_t count;
    uint8_t i;

    if (!g_vision_uart9_initialized)
    {
        return VISION_UART9_ERR_NOT_READY;
    }

    if ((length > 0U) && (NULL == values))
    {
        return VISION_UART9_ERR_PARAM;
    }

    count = length;
    if (count > VISION_UART9_FIELD_COUNT)
    {
        count = VISION_UART9_FIELD_COUNT;
    }

    for (i = 0U; i < count; i++)
    {
        tx_values[i] = values[i];
    }

    frame[0] = '@';
    for (i = 0U; i < VISION_UART9_FIELD_COUNT; i++)
    {
        VisionUart9_FormatField(tx_values[i], &frame[1U + (i * VISION_UART9_FIELD_WIDTH)]);
    }

    checksum = VisionUart9_CalcChecksum(&frame[1], VISION_UART9_DATA_LEN);
    frame[1U + VISION_UART9_DATA_LEN] = VisionUart9_NibbleToHex((uint8_t)((checksum >> 4) & 0x0FU));
    frame[2U + VISION_UART9_DATA_LEN] = VisionUart9_NibbleToHex((uint8_t)(checksum & 0x0FU));
    frame[3U + VISION_UART9_DATA_LEN] = '#';

    return VisionUart9_SendFrame(frame, VISION_UART9_FRAME_LEN);
}

/*
 * 清空接收环形缓冲区。
 */
/*
 * 测试注入一帧视觉数据。
 *
 * 说明：
 * 1. 该函数会按正常通信协议格式组一帧数据。
 * 2. 组包后直接写入 UART9 接收环形缓冲区，模拟“串口已经收到一整帧字节”。
 * 3. 上层仍然必须调用 VisionUart9_Process() 才会真正完成解析。
 */
int VisionUart9_TestInjectFrame(float u, float v, float depth, float yaw)
{
    float values[VISION_UART9_FIELD_COUNT];
    char frame[VISION_UART9_FRAME_LEN];
    uint8_t checksum;
    uint8_t i;

    if (!g_vision_uart9_initialized)
    {
        return VISION_UART9_ERR_NOT_READY;
    }

    values[0] = u;
    values[1] = v;
    values[2] = depth;
    values[3] = yaw;

    frame[0] = '@';
    for (i = 0U; i < VISION_UART9_FIELD_COUNT; i++)
    {
        VisionUart9_FormatField(values[i], &frame[1U + (i * VISION_UART9_FIELD_WIDTH)]);
    }

    checksum = VisionUart9_CalcChecksum(&frame[1], VISION_UART9_DATA_LEN);
    frame[1U + VISION_UART9_DATA_LEN] = VisionUart9_NibbleToHex((uint8_t)((checksum >> 4) & 0x0FU));
    frame[2U + VISION_UART9_DATA_LEN] = VisionUart9_NibbleToHex((uint8_t)(checksum & 0x0FU));
    frame[3U + VISION_UART9_DATA_LEN] = '#';

    for (i = 0U; i < VISION_UART9_FRAME_LEN; i++)
    {
        VisionUart9_RingPutByte((uint8_t)frame[i]);
    }

    return VISION_UART9_OK;
}

static void VisionUart9_RingReset(void)
{
    g_vision_uart9_rx_ring.read_index = 0U;
    g_vision_uart9_rx_ring.write_index = 0U;
    memset(g_vision_uart9_rx_ring.data, 0, sizeof(g_vision_uart9_rx_ring.data));
}

/*
 * 向接收环形缓冲区写入 1 个字节。
 *
 * 如果缓冲区满，则丢弃本字节并增加溢出计数。
 * 这里选择“丢弃新字节”，而不是覆盖旧字节，便于维护对数据丢失的认知。
 */
static void VisionUart9_RingPutByte(uint8_t byte)
{
    uint16_t next_write;

    next_write = (uint16_t)(g_vision_uart9_rx_ring.write_index + 1U);
    if (next_write >= VISION_UART9_RX_BUF_SIZE)
    {
        next_write = 0U;
    }

    if (next_write == g_vision_uart9_rx_ring.read_index)
    {
        g_vision_uart9_latest.rx_overflow_count++;
        return;
    }

    g_vision_uart9_rx_ring.data[g_vision_uart9_rx_ring.write_index] = byte;
    g_vision_uart9_rx_ring.write_index = next_write;
}

/*
 * 从接收环形缓冲区读取 1 个字节。
 *
 * 返回值：
 * 1. true：成功读到 1 字节
 * 2. false：缓冲区为空或参数无效
 */
static bool VisionUart9_RingGetByte(uint8_t *byte)
{
    if (NULL == byte)
    {
        return false;
    }

    if (g_vision_uart9_rx_ring.read_index == g_vision_uart9_rx_ring.write_index)
    {
        return false;
    }

    *byte = g_vision_uart9_rx_ring.data[g_vision_uart9_rx_ring.read_index];
    g_vision_uart9_rx_ring.read_index++;
    if (g_vision_uart9_rx_ring.read_index >= VISION_UART9_RX_BUF_SIZE)
    {
        g_vision_uart9_rx_ring.read_index = 0U;
    }

    return true;
}

/*
 * 计算数据区 XOR 校验。
 *
 * 说明：
 * 1. 只对数据区做 XOR
 * 2. 不包含帧头 '@'、校验位本身和帧尾 '#'
 */
static uint8_t VisionUart9_CalcChecksum(const char *data, uint32_t length)
{
    uint8_t checksum = 0U;
    uint32_t i;

    for (i = 0U; i < length; i++)
    {
        checksum ^= (uint8_t)data[i];
    }

    return checksum;
}

/*
 * 把 4bit 数值转成一个十六进制字符。
 */
static char VisionUart9_NibbleToHex(uint8_t nibble)
{
    nibble &= 0x0FU;

    if (nibble < 10U)
    {
        return (char)('0' + nibble);
    }

    return (char)('A' + (nibble - 10U));
}

/*
 * 把一个十六进制字符转成数值。
 *
 * 返回：
 * 1. >=0：解析成功
 * 2. -1：非法十六进制字符
 */
static int VisionUart9_HexToNibble(char hex)
{
    if ((hex >= '0') && (hex <= '9'))
    {
        return hex - '0';
    }

    if ((hex >= 'A') && (hex <= 'F'))
    {
        return 10 + (hex - 'A');
    }

    if ((hex >= 'a') && (hex <= 'f'))
    {
        return 10 + (hex - 'a');
    }

    return -1;
}

/*
 * 解析单个 8 字符字段。
 *
 * 期望格式：
 * +0123.45
 * -0007.80
 *
 * 若格式不符合，返回 false。
 */
static bool VisionUart9_ParseField(const char *text, float *value)
{
    uint32_t int_part;
    uint32_t frac_part;
    float parsed_value;
    uint8_t i;

    if ((NULL == text) || (NULL == value))
    {
        return false;
    }

    if ((text[0] != '+') && (text[0] != '-'))
    {
        return false;
    }

    for (i = 1U; i <= 4U; i++)
    {
        if ((text[i] < '0') || (text[i] > '9'))
        {
            return false;
        }
    }

    if ('.' != text[5])
    {
        return false;
    }

    if (((text[6] < '0') || (text[6] > '9')) ||
        ((text[7] < '0') || (text[7] > '9')))
    {
        return false;
    }

    int_part = (uint32_t)((text[1] - '0') * 1000U);
    int_part += (uint32_t)((text[2] - '0') * 100U);
    int_part += (uint32_t)((text[3] - '0') * 10U);
    int_part += (uint32_t)(text[4] - '0');

    frac_part = (uint32_t)((text[6] - '0') * 10U);
    frac_part += (uint32_t)(text[7] - '0');

    parsed_value = (float)int_part + ((float)frac_part / 100.0f);
    if ('-' == text[0])
    {
        parsed_value = -parsed_value;
    }

    *value = parsed_value;
    return true;
}

/*
 * 把一个 float 编码为固定宽度字段。
 *
 * 处理规则：
 * 1. 自动带正负号
 * 2. 保留两位小数，四舍五入
 * 3. 超出 ±9999.99 的值会被钳制
 */
static void VisionUart9_FormatField(float value, char *out_text)
{
    bool negative;
    float abs_value;
    uint32_t scaled;
    uint32_t int_part;
    uint32_t frac_part;

    if (NULL == out_text)
    {
        return;
    }

    negative = (value < 0.0f);
    abs_value = negative ? -value : value;
    if (abs_value > VISION_UART9_FIELD_MAX_VALUE)
    {
        abs_value = VISION_UART9_FIELD_MAX_VALUE;
    }

    scaled = (uint32_t)((abs_value * 100.0f) + 0.5f);
    int_part = scaled / 100U;
    frac_part = scaled % 100U;

    out_text[0] = negative ? '-' : '+';
    out_text[1] = (char)('0' + ((int_part / 1000U) % 10U));
    out_text[2] = (char)('0' + ((int_part / 100U) % 10U));
    out_text[3] = (char)('0' + ((int_part / 10U) % 10U));
    out_text[4] = (char)('0' + (int_part % 10U));
    out_text[5] = '.';
    out_text[6] = (char)('0' + ((frac_part / 10U) % 10U));
    out_text[7] = (char)('0' + (frac_part % 10U));
}

/*
 * 尝试解析一整帧数据。
 *
 * 检查顺序：
 * 1. 帧头 '@'
 * 2. 帧尾 '#'
 * 3. 校验位是否合法
 * 4. XOR 校验是否一致
 * 5. 4 个字段是否都符合固定格式
 *
 * 只有全部成功时，才更新 g_vision_uart9_latest。
 */
static bool VisionUart9_TryParseFrame(const char *frame)
{
    uint8_t checksum;
    uint8_t received_checksum;
    int high_nibble;
    int low_nibble;
    float parsed_values[VISION_UART9_FIELD_COUNT];
    uint8_t i;

    if (NULL == frame)
    {
        g_vision_uart9_latest.rx_format_error_count++;
        return false;
    }

    if ((frame[0] != '@') || (frame[VISION_UART9_FRAME_LEN - 1U] != '#'))
    {
        g_vision_uart9_latest.rx_format_error_count++;
        return false;
    }

    checksum = VisionUart9_CalcChecksum(&frame[1], VISION_UART9_DATA_LEN);
    high_nibble = VisionUart9_HexToNibble(frame[1U + VISION_UART9_DATA_LEN]);
    low_nibble = VisionUart9_HexToNibble(frame[2U + VISION_UART9_DATA_LEN]);
    if ((high_nibble < 0) || (low_nibble < 0))
    {
        g_vision_uart9_latest.rx_format_error_count++;
        return false;
    }

    received_checksum = (uint8_t)(((uint8_t)high_nibble << 4) | (uint8_t)low_nibble);
    if (checksum != received_checksum)
    {
        g_vision_uart9_latest.rx_checksum_error_count++;
        return false;
    }

    for (i = 0U; i < VISION_UART9_FIELD_COUNT; i++)
    {
        if (!VisionUart9_ParseField(&frame[1U + (i * VISION_UART9_FIELD_WIDTH)], &parsed_values[i]))
        {
            g_vision_uart9_latest.rx_format_error_count++;
            return false;
        }
    }

    g_vision_uart9_latest.u = parsed_values[0];
    g_vision_uart9_latest.v = parsed_values[1];
    g_vision_uart9_latest.depth = parsed_values[2];
    g_vision_uart9_latest.yaw = parsed_values[3];
    g_vision_uart9_latest.frame_valid = 1U;
    g_vision_uart9_latest.new_data_flag = 1U;
    g_vision_uart9_latest.tick_ms = HAL_GetTick();
    g_vision_uart9_latest.rx_ok_count++;
	  printf("========%f,%f,%f,%f=========\r\n",parsed_values[0],parsed_values[1],parsed_values[2],parsed_values[3]);
		//VisionUart9_Send4(parsed_values,4);
    return true;
}

/*
 * 发送完整帧并等待发送完成。
 *
 * 说明：
 * 1. 发送采用阻塞等待 TX_COMPLETE 的方式，便于上层直接拿返回值判断成功失败
 * 2. 如果超时，会主动中止本次发送
 * 3. 如果当前已经有发送在进行中，则返回 BUSY
 */
static int VisionUart9_SendFrame(const char *frame, uint32_t length)
{
    fsp_err_t err;
    uint32_t start_tick;

    if ((NULL == frame) || (0U == length))
    {
        return VISION_UART9_ERR_PARAM;
    }

    if (g_vision_uart9_tx_busy)
    {
        return VISION_UART9_ERR_BUSY;
    }

    g_vision_uart9_tx_complete = false;
    g_vision_uart9_tx_busy = true;
    g_vision_uart9_latest.tx_busy = 1U;

    err = g_uart9.p_api->write(g_uart9.p_ctrl, (uint8_t const *)frame, length);
    if (FSP_SUCCESS != err)
    {
        g_vision_uart9_tx_busy = false;
        g_vision_uart9_latest.tx_busy = 0U;
        g_vision_uart9_latest.tx_error_count++;
        return VISION_UART9_ERR_WRITE;
    }

    start_tick = HAL_GetTick();
    while (!g_vision_uart9_tx_complete)
    {
        if ((HAL_GetTick() - start_tick) > VISION_UART9_TX_TIMEOUT_MS)
        {
            g_uart9.p_api->communicationAbort(g_uart9.p_ctrl, UART_DIR_TX);
            g_vision_uart9_tx_busy = false;
            g_vision_uart9_latest.tx_busy = 0U;
            g_vision_uart9_latest.tx_error_count++;
            return VISION_UART9_ERR_TIMEOUT;
        }
    }

    g_vision_uart9_latest.tx_ok_count++;
    return VISION_UART9_OK;
}

/*
 * UART9 中断回调函数。
 *
 * 设计原则：
 * 1. 中断里只做最小工作量
 * 2. RX 只收字节，不解析协议
 * 3. TX 只更新状态位
 * 4. 错误只做计数，不在中断里做重恢复逻辑
 */
void UART9_Callback(uart_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
    case UART_EVENT_TX_COMPLETE: {
        g_vision_uart9_tx_complete = true;
        g_vision_uart9_tx_busy = false;
        g_vision_uart9_latest.tx_busy = 0U;
        break;
    }

    case UART_EVENT_RX_CHAR: {
        VisionUart9_RingPutByte((uint8_t)p_args->data);
        break;
    }

    case UART_EVENT_ERR_OVERFLOW: {
        g_vision_uart9_latest.rx_overflow_count++;
        break;
    }

    case UART_EVENT_ERR_FRAMING:
    case UART_EVENT_ERR_PARITY:
    case UART_EVENT_BREAK_DETECT: {
        g_vision_uart9_latest.rx_format_error_count++;
        break;
    }

    case UART_EVENT_RX_COMPLETE:
    case UART_EVENT_TX_DATA_EMPTY:
    default: {
        break;
    }
    }
}
