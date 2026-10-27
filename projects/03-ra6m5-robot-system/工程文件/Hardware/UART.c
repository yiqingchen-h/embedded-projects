#include "UART.h"

static volatile int g_uart7_tx_complete = 0;
static volatile int g_uart7_rx_complete = 0;
circle_buf_t g_rx_buf;

void printf_init(void)
{
    g_ioport.p_api->open(g_ioport.p_ctrl, g_ioport.p_cfg);
    // err = SPI0.p_api->open(SPI0.p_ctrl,SPI0.p_cfg);
    UART7.p_api->open(UART7.p_ctrl, UART7.p_cfg);
}

void uart7_wait_for_tx(void)
{
    while (!g_uart7_tx_complete)
        ;
    g_uart7_tx_complete = 0;
}
void uart7_wait_for_rx(void)
{
    while (!g_uart7_rx_complete)
        ;
    g_uart7_rx_complete = 0;
}

static int32_t circlebuf_put(struct circle_buf *pcb, uint8_t v)
{
    uint32_t next_w;

    /* 计算"下一个写位置的下一个", 如果越界就要设置为0 */
    next_w = pcb->w + 1;
    if (next_w == pcb->max_len)
        next_w = 0;

    /* 未满? */
    if (next_w != pcb->r)
    {
        /* 写入数据 */
        pcb->buffer[pcb->w] = v;

        /* 设置下一个写位置 */
        pcb->w = next_w;
        return 0;
    }
    return -1;
}
static int32_t circlebuf_get(struct circle_buf *pcb, uint8_t *pv)
{
    /* 不空? */
    if (pcb->w != pcb->r)
    {
        /* 读出数据 */
        *pv = pcb->buffer[pcb->r];

        /* 计算"下一个读位置", 如果越界就要设置为0 */
        pcb->r++;
        if (pcb->r == pcb->max_len)
            pcb->r = 0;

        return 0;
    }
    return -1;
}

uint8_t rx_buf[1024];
void circlebuf_init(void)
{
    g_rx_buf.r = g_rx_buf.w = 0;
    g_rx_buf.buffer = rx_buf;
    g_rx_buf.max_len = sizeof(rx_buf);
    g_rx_buf.get = circlebuf_get;
    g_rx_buf.put = circlebuf_put;
}

/** UART Event codes
typedef enum e_sf_event
{
    UART_EVENT_RX_COMPLETE   = (1UL << 0), /// 使用read函数启动接收后，接收完毕
    UART_EVENT_TX_COMPLETE   = (1UL << 1), /// 发送完毕
    UART_EVENT_RX_CHAR       = (1UL << 2), /// 未调用read启动接收，但是接收到了数据
    UART_EVENT_ERR_PARITY    = (1UL << 3), /// 校验错误
    UART_EVENT_ERR_FRAMING   = (1UL << 4), /// < Mode fault error event
    UART_EVENT_ERR_OVERFLOW  = (1UL << 5), /// FIFO溢出
    UART_EVENT_BREAK_DETECT  = (1UL << 6), /// < Break detect error event
    UART_EVENT_TX_DATA_EMPTY = (1UL << 7), /// 最后一个字节已经发送出去了，但是还没发送完毕
} uart_event_t;
*/
void UART7_Callback(uart_callback_args_t *p_args) // 回调函数
{
    switch (p_args->event)
    {
    case UART_EVENT_TX_COMPLETE: {
        g_uart7_tx_complete = 1;
        break;
    }
    case UART_EVENT_RX_COMPLETE: {
        g_uart7_rx_complete = 1;
        break;
    }
    case UART_EVENT_RX_CHAR: {
        g_rx_buf.put(&g_rx_buf, (uint8_t)p_args->data);
        break;
    }
    default: {
        break;
    }
    }
}

int fputc(int ch, FILE *f)
{
    (void)f;

    /* 启动发送字符 */
    UART7.p_api->write(UART7.p_ctrl, (uint8_t const *const)&ch, 1);

    /* 等待发送完毕 */
    uart7_wait_for_tx();

    return ch;
}

/* 重写这个函数,重定向scanf */
int fgetc(FILE *f)
{
    uint8_t ch;
    //	uint32_t go_timer = dwTick;		-- 需要完成fsp_err_t SystickInit(void); 滴答定时器初始化使用
    (void)f;
    while (g_rx_buf.get(&g_rx_buf, &ch))
    {
        //			if(dwTick - go_timer > 1000){	//等待数据时间
        //			return EOF;	// 返回-1，表示超时/无数据
        //			}
    };

    //    /* 回显 */
    //    {
    //        fputc(ch, NULL);

    //        /* 回车之后加换行 */
    //        if (ch == '\r')
    //        {
    //            fputc('\n', NULL);;
    //        }
    //    }

    return (int)ch;
}

//====================雷达部分=============================
static volatile int g_uart5_tx_complete = 0;
uint8_t g_lidar_temp_buf[LIDAR_FRAME_SIZE]; // DTC 搬运的目的地
// 雷达专用环形缓冲区
circle_buf_t g_lidar_rx_buf;
uint8_t lidar_raw_buf[2048];

// 全局避障地图数组
Lidar_Obstacle_Point_t g_lidar_map[TOTAL_SLOTS];

// --- 雷达专用缓冲区函数 (避免与UART7冲突) ---
static int32_t lidar_buf_put(struct circle_buf *pcb, uint8_t v)
{
    uint32_t next_w = pcb->w + 1;
    if (next_w == pcb->max_len)
        next_w = 0;
    if (next_w != pcb->r)
    {
        pcb->buffer[pcb->w] = v;
        pcb->w = next_w;
        return 0;
    }
    return -1;
}

static int32_t lidar_buf_get(struct circle_buf *pcb, uint8_t *pv)
{
    if (pcb->w != pcb->r)
    {
        *pv = pcb->buffer[pcb->r];
        pcb->r++;
        if (pcb->r == pcb->max_len)
            pcb->r = 0;
        return 0; // 成功
    }
    return -1; // 失败
}

// --- 雷达初始化 ---
void Lidar_Init(void)
{
    // 初始化环形缓冲区
    g_lidar_rx_buf.r = 0;
    g_lidar_rx_buf.w = 0;
    g_lidar_rx_buf.buffer = lidar_raw_buf;
    g_lidar_rx_buf.max_len = sizeof(lidar_raw_buf);
    g_lidar_rx_buf.get = lidar_buf_get;
    g_lidar_rx_buf.put = lidar_buf_put;

    // 初始化地图数据
    for (int i = 0; i < TOTAL_SLOTS; i++)
    {
        g_lidar_map[i].distance_mm = 0xFFFF; // 初始设为无穷远
        g_lidar_map[i].valid = 0;
    }

    // 打开UART5
    UART5.p_api->open(UART5.p_ctrl, UART5.p_cfg);
	// 启动 DTC 接收：告诉 DTC 往 g_lidar_temp_buf 搬 58 个字节
	UART5.p_api->read(UART5.p_ctrl, g_lidar_temp_buf, LIDAR_FRAME_SIZE);
}

// --- UART5 回调函数 ---
void UART5_Callback(uart_callback_args_t *p_args)
{
   switch (p_args->event)
    {
    // 当 DTC 搬完 58 字节时触发
    case UART_EVENT_RX_COMPLETE:
        // 批量把 DTC 搬好的数据存储至原有的环形缓冲区
        for (int i = 0; i < LIDAR_FRAME_SIZE; i++)
        {
            g_lidar_rx_buf.put(&g_lidar_rx_buf, g_lidar_temp_buf[i]);
        }
        // 再次启动 DTC 接收
        UART5.p_api->read(UART5.p_ctrl, g_lidar_temp_buf, LIDAR_FRAME_SIZE);
        break;

    case UART_EVENT_TX_COMPLETE:
        g_uart5_tx_complete = 1;
        break;
        
    // 错误处理：如果发生溢出等错误，也要重新启动接收
    case UART_EVENT_ERR_OVERFLOW:
    case UART_EVENT_ERR_FRAMING:
        UART5.p_api->read(UART5.p_ctrl, g_lidar_temp_buf, LIDAR_FRAME_SIZE);
        break;

    default:
        break;
    }
}

// --- 解析一帧数据 (58字节) ---
void Lidar_Parse_Frame(uint8_t *frame)
{
    // 1. 校验 CRC
    uint8_t crc_cal = 0;
    for (int i = 0; i < 57; i++)
    {
        crc_cal += frame[i];
    }
    if (crc_cal != frame[57])
        return; // CRC 错误

    // 2. 提取角度信息 (修复警告：增加强制类型转换)
    uint16_t start_angle_raw = (uint16_t)((frame[5] << 8) | frame[6]);
    uint16_t stop_angle_raw = (uint16_t)((frame[55] << 8) | frame[56]);

    // 处理跨越 0 度的情况
    if (stop_angle_raw < start_angle_raw)
    {
        stop_angle_raw += 36000;
    }

    float diff_angle = (float)(stop_angle_raw - start_angle_raw);
    float step_angle = diff_angle / 15.0f;

    // 3. 遍历 16 个点
    for (int i = 0; i < 16; i++)
    {
        // 提取距离 (修复警告：增加强制类型转换)
        uint16_t dist_h = frame[7 + i * 3];
        uint16_t dist_l = frame[8 + i * 3];
        uint16_t distance = (uint16_t)((dist_h << 8) | dist_l);

        if (distance == 0)
            continue; // 无效点

        // 计算当前点的角度 (修复警告：增加强制类型转换)
        float current_angle_raw = (float)start_angle_raw + (step_angle * (float)i);
        if (current_angle_raw >= 36000.0f)
            current_angle_raw -= 36000.0f;

        // 转换为实际角度 (度)
        float angle_deg = current_angle_raw / 100.0f;

        // 4. 角度映射 (-50 ~ +50)
        float map_angle = 0;

        if (angle_deg <= DETECT_ANGLE_HALF)
        {
            // 0 ~ 50度 (左侧)
            map_angle = angle_deg;
        }
        else if (angle_deg >= (360.0f - DETECT_ANGLE_HALF))
        {
            // 310 ~ 360度 (右侧) -> 转换为 -50 ~ 0
            map_angle = angle_deg - 360.0f;
        }
        else
        {
            continue; // 超出前方100度范围
        }

        // 5. 映射到数组索引 (0-49)
        // Index 0 = -50度 (最右), Index 25 = 0度 (正前), Index 49 = +50度 (最左)
        int index = (int)((map_angle + DETECT_ANGLE_HALF) / 2.0f);

        if (index < 0)
            index = 0;
        if (index >= TOTAL_SLOTS)
            index = TOTAL_SLOTS - 1;

        // 6. 刷新数据
        g_lidar_map[index].distance_mm = distance;
        g_lidar_map[index].valid = 1;
    }
}

// --- 数据处理主循环 (需定期调用) ---
void Lidar_Process_Data(void)
{
    uint8_t byte = 0;
    static uint8_t frame_buf[60];
    static uint8_t state = 0;
    static uint8_t data_idx = 0;

    // 从环形缓冲区读取
    while (g_lidar_rx_buf.get(&g_lidar_rx_buf, &byte) == 0)
    {
        switch (state)
        {
        case 0: // Header A5
            if (byte == 0xA5)
            {
                frame_buf[0] = byte;
                state = 1;
            }
            break;
        case 1: // Header 5A
            if (byte == 0x5A)
            {
                frame_buf[1] = byte;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;
        case 2: // Length
            frame_buf[2] = byte;
            data_idx = 3;
            state = 3;
            break;
        case 3: // Read Body
            frame_buf[data_idx++] = byte;
            if (data_idx >= 58)
            {
                Lidar_Parse_Frame(frame_buf);
                state = 0;
            }
            break;
        default:
            state = 0;
            break;
        }
    }
}

/**
 * @brief 打印所有50个点的详细距离数据
 *        触发机制：当最左侧点(Index 49)更新时触发
 */
void Lidar_Debug_Print_Detail(void)
{
    // 1. 等待一帧扫描结束 (利用最左侧点的标志位)
    if (g_lidar_map[49].valid == 1)
    {
        // (可选) 降低打印频率：每 5 帧打印一次，防止刷屏太快看不清
        static uint8_t print_div = 0;
        print_div++;
        if (print_div < 5)
        {
            g_lidar_map[49].valid = 0; // 记得清零标志位，否则下次进不来
            return;
        }
        print_div = 0;

        // 2. 开始打印
        printf("\r\n====== Lidar Data (Unit: mm) ======\r\n");

        // --- 打印右侧区域 (Index 0 ~ 24) ---
        // 对应角度：-50度(Index 0) 到 0度(Index 24)
        printf("--- RIGHT Side (-50 deg -> 0 deg) ---\r\n");
        for (int i = 0; i < 25; i++)
        {
            // 格式： [索引]:距离
            // 如果距离是 0 或 65535，打印 "----" 表示无效
            if (g_lidar_map[i].distance_mm == 0 || g_lidar_map[i].distance_mm == 0xFFFF)
                printf("[%02d]:----  ", i);
            else
                printf("[%02d]:%4d  ", i, g_lidar_map[i].distance_mm);

            // 每行打印 5 个数据，方便查看
            if ((i + 1) % 5 == 0)
                printf("\r\n");
        }

        // --- 打印左侧区域 (Index 25 ~ 49) ---
        // 对应角度：0度(Index 25) 到 +50度(Index 49)
        printf("--- LEFT Side (0 deg -> +50 deg) ---\r\n");
        for (int i = 25; i < 50; i++)
        {
            if (g_lidar_map[i].distance_mm == 0 || g_lidar_map[i].distance_mm == 0xFFFF)
                printf("[%02d]:----  ", i);
            else
                printf("[%02d]:%4d  ", i, g_lidar_map[i].distance_mm);

            // 每行打印 5 个数据
            if ((i + 1) % 5 == 0)
                printf("\r\n");
        }

        printf("===================================\r\n");

        // 3. 清除标志位，等待下一帧
        g_lidar_map[49].valid = 0;
    }
}


