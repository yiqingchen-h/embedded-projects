#ifndef UART_H_
#define UART_H_
#include "hal_data.h"
#include <stdio.h>
#include "SysTick.h"
// fsp_err_t err;		//接收报错返回值
// err = UART7.p_api->open(UART7.p_ctrl, UART7.p_cfg);		//配置串口
/*UART7.p_api->write(UART7.p_ctrl, (const uint8_t *)"100ask\r\n", 8); *串口发送函数
 *参数1：串口的状态参数
 *参数2：要发送内容
 *参数3：长度
 */
//
/*err = UART7.p_api->read(UART7.p_ctrl, &c, 1);		//启动接收函数
 *
 *
 */
void uart7_wait_for_tx(void);
void uart7_wait_for_rx(void);

typedef struct circle_buf {
    uint32_t r;
    uint32_t w;
    uint32_t max_len;
    uint8_t *buffer;
    int32_t (*put)(struct circle_buf *pcb, uint8_t v);
    int32_t (*get)(struct circle_buf *pcb, uint8_t *pv);
} circle_buf_t;
extern uint8_t rx_buf[];
extern circle_buf_t g_rx_buf;
void circlebuf_init(void); // 环形缓冲区初始化

//============================================雷达部分======================================================
// ================= 宏定义 =================
#define DETECT_ANGLE_HALF 50  // 单侧探测角度范围 (度)，即左右各50度
#define TOTAL_SLOTS       50  // 将前方100度分为50个数据槽 (每个槽代表2度)
#define LIDAR_FRAME_SIZE  58  // 雷达一帧的长度
// ================= 数据结构 =================

// 用于存储简化后的避障数据数组
// 索引 0-24 代表左侧 (0度 -> 50度)
// 索引 25-49 代表右侧 (0度 -> -50度) 实际正负方向按传感器安装方向调整
// 这里我们统一定义：Index 0对应最左(-50度), Index 49对应最右(+50度), 中间是0度
typedef struct
{
    uint16_t distance_mm; // 该角度范围内的平均/最近距离
    uint8_t valid;        // 数据是否有效/更新
} Lidar_Obstacle_Point_t;

extern Lidar_Obstacle_Point_t g_lidar_map[TOTAL_SLOTS];

// ================= 函数声明 =================
void printf_init(void);
void Lidar_Init(void);
void Lidar_Process_Data(void);       // 在主循环或定时器中调用解析
void Lidar_Debug_Print_Detail(void); // 打印数据
#endif
